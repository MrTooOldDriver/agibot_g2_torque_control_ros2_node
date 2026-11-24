// G2 torque subscriber: direct joint torque control of the AGIBOT G2 arms from ROS 2 topics.
//
// The G2 counterpart of franka_example_controllers/TorqueSubscriberController. Each arm listens on
// its own topic (std_msgs/Float64MultiArray, 7 values, joint1..joint7), like one Franka controller
// per arm. The node can add the same model terms as the vendor demo joint_torque_demo.py
// (tau = rnea(q, dq, ddq_d) + ...), each behind its own flag, all OFF by default:
//
//   tau = [compensate_gravity]  g(q)
//       + [compensate_coriolis] C(q, dq) dq        (measured velocities, waist/head included)
//       + [compensate_inertia]  M(q) ddq_d         (ddq_d from ~/<arm>/desired_accelerations)
//       + tau_ext - joint_damping * dq             (14 arm joints, Nm)
//   tau = clamp(tau, +-cap)                 cap = effort_limit_scale * URDF effort limit
//   tau = tau_prev + clamp(tau - tau_prev, +-max_torque_step)   (tau_prev of frame 0 = measured effort)
//   tau = clamp(tau, +-cap)                 (no-op: the measured holding torque must be inside the
//                                            cap, the node refuses to start otherwise)
//
// With compensate_gravity on, a command means what it means on a Franka (torque on top of gravity
// compensation) and a timed-out arm keeps the compensation terms (tau_ext = 0, the arm floats).
// With compensate_gravity OFF the command must carry the full torque, gravity included: zero
// torque would drop a G2 arm (mode 9 compensates nothing). The node then enters mode 9 only once
// both arms are commanded and the first torque is close to the robot's holding torque, and a
// timed-out arm ends the stream (previous mode restored, the robot holds the arms).
//
// The 1 kHz loop runs on its own thread (GDK JointTorqueControl: read frame -> compute -> write).
// The ROS executor runs on the main thread; commands are handed over under a mutex, as in the
// Franka controller. Joint states are published by a third thread so ROS never blocks the loop.
//
// Lifecycle: connect -> load URDF -> pre-flight check (finite state, holding torque inside the
// effort cap, gravity model) -> remember the control mode -> (gravity off: wait for commands,
// publishing the measured state) -> the mode must still be the remembered one -> mode 9
// (confirmed) -> torque loop -> restore the previous mode (confirmed) -> exit. Ctrl+C, a GDK error
// or a safety abort all leave through the same "restore the previous mode" path. If the loop ends
// for any reason the node shuts down (restart it to re-enter mode 9).

#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "g2_torque_subscriber/dynamics_model.hpp"
#include "g2_torque_subscriber/robot_backend.hpp"

namespace g2_torque_subscriber {

namespace {

using Clock = std::chrono::steady_clock;
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
constexpr std::array<const char*, 2> kArmLabels = {"left", "right"};
constexpr int kModeSwitchTimeoutMs = 500;  // WaitControlMode when entering and leaving mode 9

// The G2 model confirmed for this robot, installed with the package (urdf/). It is the default of
// urdf_path. The robot's reported urdf_type (G2_t2_crsB) is NOT the right model.
constexpr const char* kDefaultUrdfName = "genie_g2_crs_omnipicker.urdf";

std::string DefaultUrdfPath() {
  try {
    return ament_index_cpp::get_package_share_directory("g2_torque_subscriber") + "/urdf/" +
           kDefaultUrdfName;
  } catch (const std::exception&) {
    return kDefaultUrdfName;  // package not found: loading fails loudly, no silent fallback
  }
}

enum Phase : int { kIdle, kConnecting, kPreparing, kStreaming, kRestoring, kDone };

struct Params {
  std::string backend;                   // "gdk", "fake" or a path to a backend module
  std::string urdf_path;                 // URDF of the dynamics model (default: kDefaultUrdfName,
                                         // "" = the robot's urdf_type, not the confirmed model)
  std::string urdf_dir;                  // where <urdf_type>/<urdf_type>.urdf lives
  bool compensate_gravity{false};        // + g(q)
  bool compensate_coriolis{false};       // + C(q, dq) dq
  bool compensate_inertia{false};        // + M(q) ddq_d
  std::array<std::string, 2> torque_topics;
  std::array<std::string, 2> acceleration_topics;
  std::array<std::string, 2> arm_state_topics;
  std::string joint_states_topic;
  std::string commanded_torques_topic;
  std::string compensation_torques_topic;
  int publish_decimation{1};
  double command_timeout{0.1};           // s
  double max_torque_step{15.0};          // Nm per frame
  double effort_limit_scale{1.0};
  double joint_damping{0.0};             // Nm*s/rad
  double velocity_limit{2.0};            // rad/s, 0 = off
  double gravity_check_tolerance{10.0};  // Nm, 0 = off
  bool dry_run{false};
  double dry_run_rate{100.0};            // Hz
  bool enforce_realtime{false};
  int state_timeout_ms{100};
};

bool AllFinite(const ArmVector& v) {
  return std::all_of(v.begin(), v.end(), [](double x) { return std::isfinite(x); });
}

const char* OnOff(bool b) { return b ? "on" : "off"; }

}  // namespace

class TorqueSubscriberNode : public rclcpp::Node {
 public:
  explicit TorqueSubscriberNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("g2_torque_subscriber", options) {
    DeclareParameters();

    // Same QoS as the Franka torque subscriber: KeepLast(1), BestEffort, Volatile.
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    for (std::size_t arm = 0; arm < 2; ++arm) {
      torque_subs_[arm] = create_subscription<Float64MultiArray>(
          p_.torque_topics[arm], cmd_qos, [this, arm](const Float64MultiArray::SharedPtr msg) {
            OnArmInput(arm, *msg, torque_in_, "torque command");
          });
      accel_subs_[arm] = create_subscription<Float64MultiArray>(
          p_.acceleration_topics[arm], cmd_qos,
          [this, arm](const Float64MultiArray::SharedPtr msg) {
            OnArmInput(arm, *msg, accel_in_, "desired acceleration");
          });
    }
    // Reliable publishers match both reliable and best-effort subscribers.
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    auto make_pub = [&](const std::string& topic) {
      return topic.empty() ? nullptr
                           : create_publisher<sensor_msgs::msg::JointState>(topic, state_qos);
    };
    joint_states_pub_ = make_pub(p_.joint_states_topic);
    for (std::size_t arm = 0; arm < 2; ++arm) arm_state_pubs_[arm] = make_pub(p_.arm_state_topics[arm]);
    commanded_pub_ = make_pub(p_.commanded_torques_topic);
    compensation_pub_ = make_pub(p_.compensation_torques_topic);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { StatusTick(); });

    RCLCPP_INFO(get_logger(),
                "G2 torque subscriber: left '%s', right '%s' (7 values each, Nm; QoS "
                "KeepLast(1)/BestEffort/Volatile), timeout %.3f s%s",
                torque_subs_[0]->get_topic_name(), torque_subs_[1]->get_topic_name(),
                p_.command_timeout, p_.dry_run ? ", DRY RUN (nothing is sent to the robot)" : "");
    RCLCPP_INFO(get_logger(), "compensation: gravity %s, coriolis %s, inertia %s%s",
                OnOff(p_.compensate_gravity), OnOff(p_.compensate_coriolis),
                OnOff(p_.compensate_inertia),
                p_.compensate_inertia ? " (desired accelerations on ~/<arm>/desired_accelerations)"
                                      : "");
    if (!p_.compensate_gravity) {
      RCLCPP_WARN(get_logger(),
                  "gravity compensation is OFF: torque commands must contain the FULL torque, "
                  "gravity included. Mode 9 is entered only once both arms are commanded close to "
                  "the holding torque; a missing command ends the torque loop.");
    }
  }

  ~TorqueSubscriberNode() override {
    stop_requested_ = true;
    StopPublisherThread();
    if (control_thread_.joinable()) control_thread_.join();
  }

  void Start() {
    publisher_thread_ = std::thread([this] { PublisherThread(); });
    control_thread_ = std::thread([this] {
      int rc = 1;
      try {
        rc = RunControl();
      } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "control thread: %s", e.what());
      }
      phase_ = kDone;
      exit_code_.set_value(rc);
      if (rclcpp::ok()) rclcpp::shutdown();  // the robot is no longer under our control
    });
  }

  void RequestStop() { stop_requested_ = true; }

  // Wait for the control thread to restore the control mode and finish. Returns its exit code.
  int Join() {
    auto fut = exit_code_.get_future();
    const auto t0 = Clock::now();
    while (fut.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      if (phase_ == kConnecting && Clock::now() - t0 > std::chrono::seconds(3)) {
        // Still inside GDKInit / Robot(): no mode was changed yet, nothing to clean up.
        RCLCPP_ERROR(get_logger(), "GDK still connecting (robot not reachable?); exiting");
        std::fflush(nullptr);
        std::_Exit(1);
      }
      RCLCPP_INFO(get_logger(), "waiting for the torque loop to stop and the mode to be restored");
    }
    const int rc = fut.get();
    StopPublisherThread();
    if (control_thread_.joinable()) control_thread_.join();
    return rc;
  }

 private:
  // Latest per-arm input (torque command or desired acceleration) from the executor thread.
  struct ArmInput {
    std::array<double, kJointsPerArm> value{};
    Clock::time_point stamp{};
    bool valid{false};
  };
  using ArmInputs = std::array<ArmInput, 2>;

  // --------------------------------------------------------------------------------------------
  // Parameters
  // --------------------------------------------------------------------------------------------
  void DeclareParameters() {
    p_.backend = declare_parameter<std::string>("backend", "gdk");
    p_.urdf_path = declare_parameter<std::string>("urdf_path", DefaultUrdfPath());
    p_.urdf_dir = declare_parameter<std::string>(
        "urdf_dir", "/home/agi/app/share/genie_robot_description/urdf");
    p_.compensate_gravity = declare_parameter<bool>("compensate_gravity", false);
    p_.compensate_coriolis = declare_parameter<bool>("compensate_coriolis", false);
    p_.compensate_inertia = declare_parameter<bool>("compensate_inertia", false);
    p_.torque_topics[0] =
        declare_parameter<std::string>("left_torque_topic", "~/left/external_torques");
    p_.torque_topics[1] =
        declare_parameter<std::string>("right_torque_topic", "~/right/external_torques");
    p_.acceleration_topics[0] =
        declare_parameter<std::string>("left_acceleration_topic", "~/left/desired_accelerations");
    p_.acceleration_topics[1] = declare_parameter<std::string>("right_acceleration_topic",
                                                               "~/right/desired_accelerations");
    p_.arm_state_topics[0] =
        declare_parameter<std::string>("left_joint_states_topic", "~/left/joint_states");
    p_.arm_state_topics[1] =
        declare_parameter<std::string>("right_joint_states_topic", "~/right/joint_states");
    p_.joint_states_topic = declare_parameter<std::string>("joint_states_topic", "joint_states");
    p_.commanded_torques_topic =
        declare_parameter<std::string>("commanded_torques_topic", "~/commanded_torques");
    p_.compensation_torques_topic =
        declare_parameter<std::string>("compensation_torques_topic", "~/compensation_torques");
    p_.publish_decimation =
        std::max<int>(1, declare_parameter<int>("publish_decimation", p_.publish_decimation));
    p_.command_timeout = declare_parameter<double>("command_timeout", p_.command_timeout);
    p_.max_torque_step = declare_parameter<double>("max_torque_step", p_.max_torque_step);
    p_.effort_limit_scale = declare_parameter<double>("effort_limit_scale", p_.effort_limit_scale);
    p_.joint_damping = declare_parameter<double>("joint_damping", p_.joint_damping);
    p_.velocity_limit = declare_parameter<double>("velocity_limit", p_.velocity_limit);
    p_.gravity_check_tolerance =
        declare_parameter<double>("gravity_check_tolerance", p_.gravity_check_tolerance);
    p_.dry_run = declare_parameter<bool>("dry_run", p_.dry_run);
    p_.dry_run_rate = declare_parameter<double>("dry_run_rate", p_.dry_run_rate);
    p_.enforce_realtime = declare_parameter<bool>("enforce_realtime", p_.enforce_realtime);
    p_.state_timeout_ms = declare_parameter<int>("state_timeout_ms", p_.state_timeout_ms);

    if (p_.max_torque_step <= 0.0 || p_.max_torque_step >= 50.0) {
      throw std::invalid_argument("max_torque_step must be in (0, 50) Nm (robot e-stops at 50)");
    }
    if (p_.effort_limit_scale <= 0.0 || p_.effort_limit_scale > 1.0) {
      throw std::invalid_argument("effort_limit_scale must be in (0, 1]");
    }
    if (p_.command_timeout <= 0.0) throw std::invalid_argument("command_timeout must be > 0");
    if (p_.dry_run_rate <= 0.0) throw std::invalid_argument("dry_run_rate must be > 0");
    command_timeout_ = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(p_.command_timeout));
  }

  // --------------------------------------------------------------------------------------------
  // Inputs (executor thread)
  // --------------------------------------------------------------------------------------------
  void OnArmInput(std::size_t arm, const Float64MultiArray& msg, ArmInputs& inputs,
                  const char* what) {
    if (msg.data.size() != kJointsPerArm) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "%s arm: %s with %zu elements, expected %zu; ignored", kArmLabels[arm],
                            what, msg.data.size(), kJointsPerArm);
      return;
    }
    if (!std::all_of(msg.data.begin(), msg.data.end(), [](double x) { return std::isfinite(x); })) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s arm: %s contains NaN/Inf; ignored",
                            kArmLabels[arm], what);
      return;
    }
    std::lock_guard<std::mutex> lock(input_mutex_);
    std::copy(msg.data.begin(), msg.data.end(), inputs[arm].value.begin());
    inputs[arm].stamp = Clock::now();
    inputs[arm].valid = true;
  }

  // Latest torque command and desired acceleration of each arm (stale ones are 0).
  // fresh[arm]: that arm's torque command is younger than command_timeout. RT-safe.
  void LatestInputs(ArmVector& ext, ArmVector& ddq, std::array<bool, 2>& fresh) {
    ext.fill(0.0);
    ddq.fill(0.0);
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(input_mutex_);
    for (std::size_t arm = 0; arm < 2; ++arm) {
      const std::size_t o = arm * kJointsPerArm;
      fresh[arm] = torque_in_[arm].valid && now - torque_in_[arm].stamp < command_timeout_;
      if (fresh[arm]) std::copy(torque_in_[arm].value.begin(), torque_in_[arm].value.end(), ext.begin() + o);
      if (accel_in_[arm].valid && now - accel_in_[arm].stamp < command_timeout_) {
        std::copy(accel_in_[arm].value.begin(), accel_in_[arm].value.end(), ddq.begin() + o);
      }
    }
  }

  // 10 Hz: report when an arm's input stream starts or times out.
  void StatusTick() {
    const auto now = Clock::now();
    std::array<bool, 2> cmd_fresh{}, acc_fresh{};
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      for (std::size_t arm = 0; arm < 2; ++arm) {
        cmd_fresh[arm] = torque_in_[arm].valid && now - torque_in_[arm].stamp < command_timeout_;
        acc_fresh[arm] = accel_in_[arm].valid && now - accel_in_[arm].stamp < command_timeout_;
      }
    }
    for (std::size_t arm = 0; arm < 2; ++arm) {
      if (cmd_fresh[arm] != was_cmd_fresh_[arm]) {
        if (cmd_fresh[arm]) {
          RCLCPP_INFO(get_logger(), "%s arm: receiving torque commands", kArmLabels[arm]);
        } else if (p_.compensate_gravity) {
          RCLCPP_WARN(get_logger(), "%s arm: no torque command for %.3f s -> external torque 0 "
                      "(compensation terms continue)", kArmLabels[arm], p_.command_timeout);
        } else {
          RCLCPP_WARN(get_logger(), "%s arm: no torque command for %.3f s (gravity compensation "
                      "is off: the torque loop does not run without commands)", kArmLabels[arm],
                      p_.command_timeout);
        }
        was_cmd_fresh_[arm] = cmd_fresh[arm];
      }
      if (p_.compensate_inertia && acc_fresh[arm] != was_acc_fresh_[arm]) {
        if (acc_fresh[arm]) {
          RCLCPP_INFO(get_logger(), "%s arm: receiving desired accelerations", kArmLabels[arm]);
        } else {
          RCLCPP_WARN(get_logger(), "%s arm: no desired acceleration for %.3f s -> inertia term 0",
                      kArmLabels[arm], p_.command_timeout);
        }
        was_acc_fresh_[arm] = acc_fresh[arm];
      }
    }
  }

  // --------------------------------------------------------------------------------------------
  // Control thread
  // --------------------------------------------------------------------------------------------
  int RunControl() {
    phase_ = kConnecting;
    std::string error;
    if (!LoadBackend(error)) {
      RCLCPP_FATAL(get_logger(), "%s", error.c_str());
      return 1;
    }
    RCLCPP_INFO(get_logger(), "backend '%s': connecting to the robot", backend_->Name().c_str());
    BackendOptions options;
    options.urdf_path = p_.urdf_path;
    if (!backend_->Connect(options, error)) {
      RCLCPP_FATAL(get_logger(), "%s", error.c_str());
      return 1;
    }
    phase_ = kPreparing;
    const int rc = PrepareAndRun();
    backend_->Disconnect();
    return rc;
  }

  int PrepareAndRun() {
    // ---- Dynamics model ----
    std::string urdf = p_.urdf_path;
    if (urdf.empty()) {
      std::string type;
      if (!backend_->GetUrdfType(type) || type.empty()) {
        RCLCPP_FATAL(get_logger(), "robot did not report its urdf_type; set parameter urdf_path");
        return 1;
      }
      urdf = p_.urdf_dir + "/" + type + "/" + type + ".urdf";
      RCLCPP_WARN(get_logger(),
                  "urdf_path is empty: using the robot's urdf_type '%s', not the confirmed model "
                  "%s (unset urdf_path to use it)", type.c_str(), kDefaultUrdfName);
    }
    try {
      model_ = std::make_unique<DynamicsModel>(urdf);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "%s", e.what());
      return 1;
    }
    RCLCPP_INFO(get_logger(), "dynamics model: %s", urdf.c_str());

    // ---- Pre-flight: state stream alive, all arm joints present, model plausible ----
    JointFrame frame;
    if (!backend_->ReadJointStates(frame)) {
      RCLCPP_FATAL(get_logger(), "cannot read the robot joint states");
      return 1;
    }
    if (!model_->Bind(frame)) {
      RCLCPP_FATAL(get_logger(), "the robot state misses one of the 14 arm joints");
      return 1;
    }
    if (!PreflightCheck(frame)) return 1;
    InitStateBuffers(frame);

    if (p_.dry_run) return RunDryRun();

    // ---- Remember the control mode ----
    int old_mode = -1;
    if (!backend_->GetControlMode(old_mode)) {
      RCLCPP_FATAL(get_logger(), "cannot read the control mode");
      return 1;
    }
    if (old_mode == kModeJointTorque) {
      RCLCPP_FATAL(get_logger(),
                   "robot is already in mode 9 (a previous run did not exit cleanly?); cannot "
                   "tell which mode to restore. Switch it to mode 3 first.");
      return 1;
    }
    // ---- Gravity compensation off: the commands must hold the arms before mode 9 ----
    if (!p_.compensate_gravity && !WaitForCommands()) return stop_requested_ ? 0 : 1;
    if (stop_requested_) return 0;
    // ---- Right before taking control: the mode must still be the one to restore ----
    int mode_now = -1;
    if (!backend_->GetControlMode(mode_now)) {
      RCLCPP_FATAL(get_logger(), "cannot read the control mode");
      return 1;
    }
    if (mode_now != old_mode) {
      RCLCPP_FATAL(get_logger(),
                   "control mode changed from %d to %d while preparing (someone else operating "
                   "the robot?); not entering mode 9",
                   old_mode, mode_now);
      return 1;
    }

    // ---- Enter mode 9. From the first SetControlMode(9) on, every exit restores old_mode ----
    RCLCPP_INFO(get_logger(), "control mode %d -> 9 (direct torque). Do not touch the arms.",
                old_mode);
    int rc = 0;
    if (!backend_->SetControlMode(kModeJointTorque)) {
      RCLCPP_FATAL(get_logger(), "SetControlMode(9) failed");
      rc = 1;
    } else if (!backend_->WaitControlMode(kModeJointTorque, kModeSwitchTimeoutMs) ||
               !ModeIs(kModeJointTorque)) {
      RCLCPP_FATAL(get_logger(),
                   "mode 9 not confirmed by the robot (arm under external force, unregistered "
                   "payload or failed force-control admission?)");
      rc = 1;
    } else {
      phase_ = kStreaming;
      RCLCPP_INFO(get_logger(), "torque loop running (1 kHz)");
      std::string error;
      const bool ok = backend_->RunTorqueLoop(
          [this](const JointFrame& f, uint64_t period_ns, ArmVector& tau) {
            return OnFrame(f, period_ns, tau);
          },
          p_.enforce_realtime, p_.state_timeout_ms, error);
      if (!ok) {
        RCLCPP_ERROR(get_logger(), "torque loop failed: %s", error.c_str());
        rc = 2;
      }
      if (aborted_) {
        RCLCPP_ERROR(get_logger(), "safety abort: %s", abort_reason_);
        rc = 3;
      }
      RCLCPP_INFO(get_logger(), "torque loop ended after %llu frames",
                  static_cast<unsigned long long>(frame_count_));
    }

    // ---- Leave mode 9 ourselves: GDK never does it. Report it only once the robot confirms ----
    phase_ = kRestoring;
    if (!backend_->SetControlMode(old_mode)) {
      RCLCPP_ERROR(get_logger(),
                   "SetControlMode(%d) failed: restore the mode manually and check the robot's "
                   "motion-control error state (ClearMotionControlError)",
                   old_mode);
      rc = rc ? rc : 2;
    } else if (!backend_->WaitControlMode(old_mode, kModeSwitchTimeoutMs) || !ModeIs(old_mode)) {
      RCLCPP_ERROR(get_logger(),
                   "control mode %d requested but not confirmed by the robot within %d ms: check "
                   "the mode and the motion-control error state (ClearMotionControlError)",
                   old_mode, kModeSwitchTimeoutMs);
      rc = rc ? rc : 2;
    } else {
      RCLCPP_INFO(get_logger(), "control mode restored to %d", old_mode);
    }
    return rc;
  }

  bool ModeIs(int mode) {
    int now = -1;
    return backend_->GetControlMode(now) && now == mode;
  }

  // The node clamps every torque to the cap (effort_limit_scale * URDF effort limit) and slews
  // from the measured holding torque, so it can only take over arms whose holding torque is
  // inside the cap. False (logged) otherwise.
  bool HoldingTorqueWithinCap(const ArmVector& effort) {
    const ArmVector& limit = model_->EffortLimits();
    for (std::size_t i = 0; i < kArmDof; ++i) {
      const double cap = p_.effort_limit_scale * limit[i];
      if (std::abs(effort[i]) > cap) {
        RCLCPP_FATAL(get_logger(),
                     "measured holding torque %.3f Nm on %s exceeds the effort cap %.3f Nm "
                     "(effort_limit_scale %.3f x URDF limit %.3f Nm): the node could not hold "
                     "the arm. Refusing to enter mode 9.",
                     effort[i], kArmJointNames[i], cap, p_.effort_limit_scale, limit[i]);
        return false;
      }
    }
    return true;
  }

  // Model gravity vs measured effort with the arms at rest. The state must be finite and the
  // holding torque inside the effort cap. With gravity compensation on, a large difference
  // (wrong URDF / payload / waist pose) also refuses the start; with it off the table is
  // informational (the commands carry gravity, see WaitForCommands).
  bool PreflightCheck(const JointFrame& frame) {
    ArmVector q{}, dq{}, effort{};
    model_->Extract(frame, q, dq, effort);
    model_->SetState(frame);
    const ArmVector g = model_->Gravity();
    double worst = 0.0, max_speed = 0.0;
    std::ostringstream os;
    os << "pre-flight: model gravity vs measured effort (Nm)\n";
    for (std::size_t i = 0; i < kArmDof; ++i) {
      const double diff = g[i] - effort[i];
      worst = std::max(worst, std::abs(diff));
      max_speed = std::max(max_speed, std::abs(dq[i]));
      char line[128];
      std::snprintf(line, sizeof(line), "  %-20s g=%8.3f  effort=%8.3f  diff=%8.3f\n",
                    kArmJointNames[i], g[i], effort[i], diff);
      os << line;
    }
    RCLCPP_INFO(get_logger(), "%s  max |diff| = %.3f Nm", os.str().c_str(), worst);
    if (max_speed > 0.05) {
      RCLCPP_WARN(get_logger(), "arms are moving (%.3f rad/s): the gravity check is unreliable",
                  max_speed);
    }
    // Before any comparison: a NaN would drop out of `worst` and pass the tolerance check.
    if (!AllFinite(q) || !AllFinite(dq) || !AllFinite(effort)) {
      RCLCPP_FATAL(get_logger(), "the robot reports a non-finite arm joint position, velocity or "
                   "effort (see the table); refusing to start");
      return false;
    }
    if (!AllFinite(g)) {
      RCLCPP_FATAL(get_logger(), "dynamics model returned NaN/Inf");
      return false;
    }
    if (p_.compensate_gravity && p_.gravity_check_tolerance > 0.0 &&
        worst > p_.gravity_check_tolerance) {
      RCLCPP_FATAL(get_logger(),
                   "gravity model differs from the measured effort by %.3f Nm > "
                   "gravity_check_tolerance %.3f Nm: wrong URDF / payload / waist posture? "
                   "Refusing to enter mode 9.",
                   worst, p_.gravity_check_tolerance);
      return false;
    }
    if (!HoldingTorqueWithinCap(effort)) return false;
    // Fallback torque for an abort before the first loop frame (frame 0 re-seeds it).
    prev_tau_ = effort;
    return true;
  }

  // Gravity compensation off: the commands carry the full torque, so the arms drop without them.
  // Wait (before mode 9) until both arms are commanded and the first torque is within
  // gravity_check_tolerance of the holding torque (measured effort). The measured state is
  // published meanwhile, for controllers that need it to compute their first command (no
  // commanded / compensation torques yet). False on stop, read error or invalid state.
  bool WaitForCommands() {
    RCLCPP_INFO(get_logger(), "waiting for torque commands on both arms (full torque, gravity "
                "included) before entering mode 9");
    JointFrame frame;
    ArmVector ext{}, ddq{};
    std::array<bool, 2> fresh{};
    auto next = Clock::now();
    while (!stop_requested_ && rclcpp::ok()) {
      if (!backend_->ReadJointStates(frame) || !model_->Bind(frame)) {
        RCLCPP_FATAL(get_logger(), "cannot read the robot joint states");
        return false;
      }
      model_->Extract(frame, q_, dq_, effort_);
      if (!AllFinite(q_) || !AllFinite(dq_) || !AllFinite(effort_)) {
        RCLCPP_FATAL(get_logger(), "the robot reports a non-finite arm joint position, velocity "
                     "or effort; not entering mode 9");
        return false;
      }
      model_->SetState(frame);
      HandOffState(frame, nullptr);
      LatestInputs(ext, ddq, fresh);
      if (fresh[0] && fresh[1]) {
        ModelTorque(ddq, comp_);
        double worst = 0.0;
        std::size_t worst_i = 0;
        for (std::size_t i = 0; i < kArmDof; ++i) {
          const double d = std::abs(comp_[i] + ext[i] - p_.joint_damping * dq_[i] - effort_[i]);
          if (d > worst) {
            worst = d;
            worst_i = i;
          }
        }
        if (p_.gravity_check_tolerance <= 0.0 || worst <= p_.gravity_check_tolerance) {
          if (!HoldingTorqueWithinCap(effort_)) return false;
          RCLCPP_INFO(get_logger(), "commands on both arms, within %.3f Nm of the holding torque",
                      worst);
          return true;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "first torque differs from the holding torque by %.3f Nm on %s "
                             "(> gravity_check_tolerance %.3f): not entering mode 9. Do the "
                             "commands include gravity?",
                             worst, kArmJointNames[worst_i], p_.gravity_check_tolerance);
      } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "waiting for torque commands: left %s, right %s",
                             fresh[0] ? "ok" : "missing", fresh[1] ? "ok" : "missing");
      }
      next += std::chrono::milliseconds(10);
      std::this_thread::sleep_until(next);
    }
    return false;
  }

  int RunDryRun() {
    phase_ = kStreaming;
    RCLCPP_INFO(get_logger(), "dry run: reading states at %.0f Hz, computing torques, sending "
                "nothing, never switching the control mode", p_.dry_run_rate);
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / p_.dry_run_rate));
    const auto period_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period).count());
    JointFrame frame;
    ArmVector tau{};
    auto next = Clock::now();
    while (!stop_requested_ && rclcpp::ok()) {
      if (!backend_->ReadJointStates(frame)) {
        RCLCPP_ERROR(get_logger(), "cannot read the robot joint states");
        return 2;
      }
      if (OnFrame(frame, frame_count_ ? period_ns : 0, tau) && aborted_) {
        RCLCPP_ERROR(get_logger(), "dry run: safety abort would trigger: %s", abort_reason_);
        return 3;
      }
      next += period;
      std::this_thread::sleep_until(next);
    }
    return 0;
  }

  // Sum of the enabled model terms at the model state (Bind + SetState first). RT-safe.
  void ModelTorque(const ArmVector& ddq, ArmVector& comp) {
    comp.fill(0.0);
    if (p_.compensate_gravity) {
      const ArmVector& g = model_->Gravity();
      for (std::size_t i = 0; i < kArmDof; ++i) comp[i] += g[i];
    }
    if (p_.compensate_coriolis) {
      const ArmVector& c = model_->Coriolis();
      for (std::size_t i = 0; i < kArmDof; ++i) comp[i] += c[i];
    }
    if (p_.compensate_inertia) {
      const ArmVector& m = model_->Inertia(ddq);
      for (std::size_t i = 0; i < kArmDof; ++i) comp[i] += m[i];
    }
  }

  // One frame of the torque law. Runs at 1 kHz on the loop thread: no logging, no allocation.
  bool OnFrame(const JointFrame& frame, uint64_t /*period_ns*/, ArmVector& tau) {
    ++frame_count_;
    if (!model_->Bind(frame)) return Abort("robot state misses an arm joint", tau);
    model_->Extract(frame, q_, dq_, effort_);
    model_->SetState(frame);

    const ArmVector& limit = model_->EffortLimits();
    if (first_frame_) {  // the robot compares frame 0 with its holding torque (~ measured effort)
      prev_tau_ = AllFinite(effort_) ? effort_ : model_->Gravity();
      first_frame_ = false;
      for (std::size_t i = 0; i < kArmDof; ++i) {  // checked before mode 9 too; the arms may differ now
        const double cap = p_.effort_limit_scale * limit[i];
        if (std::abs(prev_tau_[i]) > cap) {
          std::snprintf(abort_reason_, sizeof(abort_reason_),
                        "holding torque %.3f Nm on %s exceeds the effort cap %.3f Nm",
                        prev_tau_[i], kArmJointNames[i], cap);
          return Abort(nullptr, tau);
        }
      }
    }

    ArmVector ext{}, ddq{};
    std::array<bool, 2> fresh{};
    LatestInputs(ext, ddq, fresh);
    ModelTorque(ddq, comp_);
    if (!p_.compensate_gravity && !p_.dry_run) {
      for (std::size_t arm = 0; arm < 2; ++arm) {
        if (!fresh[arm]) {
          std::snprintf(abort_reason_, sizeof(abort_reason_),
                        "%s arm: no torque command for %.3f s and gravity compensation is off "
                        "(zero torque would drop the arm)",
                        kArmLabels[arm], p_.command_timeout);
          return Abort(nullptr, tau);
        }
      }
    }

    for (std::size_t i = 0; i < kArmDof; ++i) {
      const double lim = p_.effort_limit_scale * limit[i];
      double t = comp_[i] + ext[i] - p_.joint_damping * dq_[i];
      t = std::clamp(t, -lim, lim);
      t = prev_tau_[i] + std::clamp(t - prev_tau_[i], -p_.max_torque_step, p_.max_torque_step);
      tau[i] = std::clamp(t, -lim, lim);  // no-op: tau_prev of frame 0 is inside the cap
    }

    if (!AllFinite(q_) || !AllFinite(dq_)) return Abort("non-finite joint state", tau);
    if (!AllFinite(tau)) return Abort("non-finite torque", tau);
    if (p_.velocity_limit > 0.0) {
      for (std::size_t i = 0; i < kArmDof; ++i) {
        if (std::abs(dq_[i]) > p_.velocity_limit) {
          std::snprintf(abort_reason_, sizeof(abort_reason_),
                        "%s at %.3f rad/s > velocity_limit %.3f rad/s", kArmJointNames[i], dq_[i],
                        p_.velocity_limit);
          return Abort(nullptr, tau);
        }
      }
    }
    prev_tau_ = tau;
    if (frame_count_ % static_cast<uint64_t>(p_.publish_decimation) == 0) HandOffState(frame, &tau);
    return stop_requested_.load();
  }

  // Make this the final frame: repeat the last torque that was sent (finite, within the step
  // limit); the control thread then restores the previous mode, which holds the arms.
  bool Abort(const char* reason, ArmVector& tau) {
    if (reason) std::snprintf(abort_reason_, sizeof(abort_reason_), "%s", reason);
    tau = prev_tau_;
    aborted_ = true;
    return true;
  }

  bool LoadBackend(std::string& error) {
    std::string lib = p_.backend;
    if (lib == "gdk" || lib == "fake") {
      try {
        lib = ament_index_cpp::get_package_prefix("g2_torque_subscriber") +
              "/lib/g2_torque_subscriber/libg2_" + lib + "_backend.so";
      } catch (const std::exception& e) {
        error = std::string("cannot locate the package: ") + e.what();
        return false;
      }
    }
    // RTLD_LOCAL | RTLD_DEEPBIND: keep the GDK's bundled Fast DDS 3 / boost / protobuf out of the
    // global symbol scope that ROS uses, and make the GDK prefer its own copies (robot_backend.hpp).
    backend_handle_ = dlopen(lib.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);
    if (!backend_handle_) {
      error = std::string("cannot load backend: ") + dlerror();
      if (p_.backend == "gdk") {
        error += "\n  Was the GDK found when building (GDK_APP_DIR)? Did you source the GDK "
                 "env.sh in this shell?";
      }
      return false;
    }
    auto create = reinterpret_cast<G2CreateRobotBackendFn>(
        dlsym(backend_handle_, G2_BACKEND_FACTORY_SYMBOL));
    if (!create) {
      error = "backend " + lib + " has no " + G2_BACKEND_FACTORY_SYMBOL;
      return false;
    }
    backend_.reset(create());
    // The module is never dlclose()d: the GDK keeps threads and static state.
    return backend_ != nullptr;
  }

  // --------------------------------------------------------------------------------------------
  // State publishing: the loop hands a snapshot over (try_lock, never blocks); a separate thread
  // publishes it.
  // --------------------------------------------------------------------------------------------
  struct StateSnapshot {
    JointFrame frame;
    ArmVector q{}, dq{}, effort{}, tau{}, comp{};
    bool has_torques{false};  // tau / comp valid (torque loop or dry run, not while waiting)
    bool fresh{false};
  };

  void InitStateBuffers(const JointFrame& frame) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snapshot_.frame = frame;  // size the buffers once, outside the loop
  }

  // Hand the measured state (q_, dq_, effort_ of `frame`) and, unless tau is null, the commanded
  // torque and comp_ over to the publisher thread.
  void HandOffState(const JointFrame& frame, const ArmVector* tau) {
    {
      std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
      if (!lock.owns_lock()) return;  // publisher busy: skip this sample
      snapshot_.frame.timestamp_ns = frame.timestamp_ns;
      if (snapshot_.frame.names != frame.names) snapshot_.frame.names = frame.names;
      snapshot_.frame.position = frame.position;
      snapshot_.frame.velocity = frame.velocity;
      snapshot_.frame.effort = frame.effort;
      snapshot_.q = q_;
      snapshot_.dq = dq_;
      snapshot_.effort = effort_;
      snapshot_.has_torques = tau != nullptr;
      if (tau) {
        snapshot_.tau = *tau;
        snapshot_.comp = comp_;
      }
      snapshot_.fresh = true;
    }
    state_cv_.notify_one();
  }

  void PublisherThread() {
    StateSnapshot local;
    sensor_msgs::msg::JointState full_msg, cmd_msg, comp_msg;
    std::array<sensor_msgs::msg::JointState, 2> arm_msgs;
    for (std::size_t arm = 0; arm < 2; ++arm) {
      auto& m = arm_msgs[arm];
      for (std::size_t j = 0; j < kJointsPerArm; ++j) {
        m.name.emplace_back(kArmJointNames[arm * kJointsPerArm + j]);
      }
      m.position.resize(kJointsPerArm);
      m.velocity.resize(kJointsPerArm);
      m.effort.resize(kJointsPerArm);
    }
    cmd_msg.name.assign(kArmJointNames.begin(), kArmJointNames.end());
    comp_msg.name = cmd_msg.name;
    while (true) {
      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        state_cv_.wait(lock, [this] { return snapshot_.fresh || publisher_stop_; });
        if (publisher_stop_) return;
        std::swap(local, snapshot_);
        snapshot_.fresh = false;
      }
      if (!rclcpp::ok()) continue;
      try {
        const auto stamp = now();
        if (joint_states_pub_) {
          full_msg.header.stamp = stamp;
          full_msg.name = local.frame.names;
          full_msg.position = local.frame.position;
          full_msg.velocity = local.frame.velocity;
          full_msg.effort = local.frame.effort;
          joint_states_pub_->publish(full_msg);
        }
        for (std::size_t arm = 0; arm < 2; ++arm) {
          if (!arm_state_pubs_[arm]) continue;
          auto& m = arm_msgs[arm];
          m.header.stamp = stamp;
          const std::size_t o = arm * kJointsPerArm;
          std::copy_n(local.q.begin() + o, kJointsPerArm, m.position.begin());
          std::copy_n(local.dq.begin() + o, kJointsPerArm, m.velocity.begin());
          std::copy_n(local.effort.begin() + o, kJointsPerArm, m.effort.begin());
          arm_state_pubs_[arm]->publish(m);
        }
        for (auto* pm : {&cmd_msg, &comp_msg}) {
          auto& pub = pm == &cmd_msg ? commanded_pub_ : compensation_pub_;
          if (!pub || !local.has_torques) continue;
          const ArmVector& effort = pm == &cmd_msg ? local.tau : local.comp;
          pm->header.stamp = stamp;
          pm->position.assign(local.q.begin(), local.q.end());
          pm->velocity.assign(local.dq.begin(), local.dq.end());
          pm->effort.assign(effort.begin(), effort.end());
          pub->publish(*pm);
        }
      } catch (const std::exception& e) {  // e.g. context shut down while publishing
        if (rclcpp::ok()) RCLCPP_WARN(get_logger(), "publish failed: %s", e.what());
      }
    }
  }

  void StopPublisherThread() {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      publisher_stop_ = true;
    }
    state_cv_.notify_all();
    if (publisher_thread_.joinable()) publisher_thread_.join();
  }

  // --------------------------------------------------------------------------------------------
  Params p_;
  Clock::duration command_timeout_{};

  std::mutex input_mutex_;
  ArmInputs torque_in_{}, accel_in_{};
  std::array<bool, 2> was_cmd_fresh_{}, was_acc_fresh_{};

  std::array<rclcpp::Subscription<Float64MultiArray>::SharedPtr, 2> torque_subs_, accel_subs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr, 2> arm_state_pubs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr commanded_pub_, compensation_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  void* backend_handle_{nullptr};
  std::unique_ptr<RobotBackend> backend_;
  std::unique_ptr<DynamicsModel> model_;

  // Loop-thread state
  ArmVector q_{}, dq_{}, effort_{}, comp_{}, prev_tau_{};
  bool first_frame_{true};
  uint64_t frame_count_{0};
  std::atomic<bool> aborted_{false};
  char abort_reason_[256] = {};

  std::atomic<bool> stop_requested_{false};
  std::atomic<int> phase_{kIdle};
  std::promise<int> exit_code_;
  std::thread control_thread_;

  std::mutex state_mutex_;
  std::condition_variable state_cv_;
  StateSnapshot snapshot_;
  bool publisher_stop_{false};
  std::thread publisher_thread_;
};

}  // namespace g2_torque_subscriber

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 1;
  try {
    auto node = std::make_shared<g2_torque_subscriber::TorqueSubscriberNode>(rclcpp::NodeOptions());
    node->Start();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();     // returns on Ctrl+C or when the control thread shuts ROS down
    node->RequestStop();  // final frame, then the previous control mode is restored
    rc = node->Join();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("g2_torque_subscriber"), "%s", e.what());
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return rc;
}
