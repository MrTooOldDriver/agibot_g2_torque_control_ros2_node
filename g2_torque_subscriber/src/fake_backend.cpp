// Offline stand-in for the G2: RobotBackend with a simulated dual-arm plant (Pinocchio ABA on the
// URDF, non-arm joints locked at zero), for rehearsing the node without a robot. Selected only by
// the node parameter backend:=fake; nothing falls back to it silently.
//
// Emulated robot behaviour (direct torque mode 9, see the GDK direct torque docs):
//  - starts in control mode 3 and holds the arms until a torque stream runs;
//  - no gravity compensation: the plant feels exactly the torques the node sends;
//  - a torque change > 50 Nm on one joint between frames (the first frame is compared with the
//    holding torque g(q)) is an e-stop: the loop fails and the mode drops to 1;
//  - a frame with a non-finite torque is dropped (the previous torque is kept);
//  - URDF joint limits act as hard stops.
// Environment: G2_FAKE_FRICTION (viscous friction, Nm*s/rad, default 0.5),
//              G2_FAKE_INITIAL_POSE (14 comma-separated rad, default all 0 clamped to limits).

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "g2_torque_subscriber/robot_backend.hpp"

namespace {

using g2_torque_subscriber::ArmVector;
using g2_torque_subscriber::BackendOptions;
using g2_torque_subscriber::JointFrame;
using g2_torque_subscriber::kArmDof;
using g2_torque_subscriber::kArmJointNames;
using g2_torque_subscriber::kModeJointTorque;
using g2_torque_subscriber::TorqueCallback;

constexpr double kDt = 0.001;
constexpr double kEstopStepNm = 50.0;

double EnvDouble(const char* name, double fallback) {
  const char* v = std::getenv(name);
  return v ? std::atof(v) : fallback;
}

class FakeBackend final : public g2_torque_subscriber::RobotBackend {
 public:
  std::string Name() const override { return "fake"; }

  bool Connect(const BackendOptions& options, std::string& error) override {
    if (options.urdf_path.empty()) {
      error = "fake backend: set parameter urdf_path (the simulated arms are built from it)";
      return false;
    }
    try {
      pinocchio::Model full;
      pinocchio::urdf::buildModel(options.urdf_path, full);
      // Lock every joint except the 14 arm joints at the neutral configuration.
      std::vector<pinocchio::JointIndex> lock;
      for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full.njoints); ++j) {
        const auto& name = full.names[j];
        if (std::find_if(kArmJointNames.begin(), kArmJointNames.end(),
                         [&](const char* n) { return name == n; }) == kArmJointNames.end()) {
          lock.push_back(j);
          if (full.joints[j].nq() == 1 && full.joints[j].nv() == 1) other_names_.push_back(name);
        }
      }
      model_ = pinocchio::buildReducedModel(full, lock, pinocchio::neutral(full));
      data_ = pinocchio::Data(model_);
      for (std::size_t i = 0; i < kArmDof; ++i) {
        if (!model_.existJointName(kArmJointNames[i])) {
          error = std::string("URDF has no joint ") + kArmJointNames[i];
          return false;
        }
        const auto& joint = model_.joints[model_.getJointId(kArmJointNames[i])];
        idx_q_[i] = joint.idx_q();
        idx_v_[i] = joint.idx_v();
      }
      q_ = pinocchio::neutral(model_);
      v_ = Eigen::VectorXd::Zero(model_.nv);
      friction_ = EnvDouble("G2_FAKE_FRICTION", 0.5);
      ApplyInitialPose();
      ClampToLimits();
      held_effort_ = GravityArm();
      applied_ = held_effort_;
      return true;
    } catch (const std::exception& e) {
      error = std::string("fake backend: ") + e.what();
      return false;
    }
  }

  bool ReadJointStates(JointFrame& frame) override {
    std::lock_guard<std::mutex> lock(mutex_);
    FillFrame(frame);
    return true;
  }

  bool GetUrdfType(std::string& urdf_type) override {
    urdf_type = "";  // the node must be given urdf_path
    return true;
  }

  bool GetControlMode(int& mode) override {
    mode = mode_.load();
    return true;
  }

  bool SetControlMode(int mode) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode != kModeJointTorque) {  // any other mode: hold the arms where they are
      v_.setZero();
      held_effort_ = GravityArm();
      applied_ = held_effort_;
    }
    mode_ = mode;
    return true;
  }

  bool WaitControlMode(int mode, int /*timeout_ms*/) override { return mode_.load() == mode; }

  bool RunTorqueLoop(const TorqueCallback& callback, bool /*enforce_realtime*/,
                     int /*state_timeout_ms*/, std::string& error) override {
    if (mode_.load() != kModeJointTorque) {
      error = "fake robot: not in mode 9";
      return false;
    }
    JointFrame frame;
    ArmVector tau{};
    ArmVector prev = held_effort_;  // first frame is compared with the holding torque
    uint64_t k = 0;
    auto next = std::chrono::steady_clock::now();
    while (true) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        FillFrame(frame);
      }
      if (mode_.load() != kModeJointTorque) {
        error = "fake robot: left mode 9 during the stream";
        return false;
      }
      const bool finished = callback(frame, k == 0 ? 0 : 1000000, tau);
      bool finite = true;
      for (double t : tau) finite = finite && std::isfinite(t);
      if (finite) {
        for (std::size_t i = 0; i < kArmDof; ++i) {
          if (std::abs(tau[i] - prev[i]) > kEstopStepNm) {
            std::ostringstream os;
            os << "fake robot E-STOP: torque step " << (tau[i] - prev[i]) << " Nm on "
               << kArmJointNames[i] << " (limit " << kEstopStepNm << ")";
            error = os.str();
            SetControlMode(1);
            return false;
          }
        }
        prev = tau;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        Step(prev);
      }
      ++k;
      if (finished) return true;
      next += std::chrono::microseconds(1000);
      std::this_thread::sleep_until(next);
    }
  }

  void Disconnect() override {}

 private:
  void ApplyInitialPose() {
    const char* env = std::getenv("G2_FAKE_INITIAL_POSE");
    if (!env) return;
    std::stringstream ss(env);
    std::string item;
    for (std::size_t i = 0; i < kArmDof && std::getline(ss, item, ','); ++i) {
      q_[idx_q_[i]] = std::atof(item.c_str());
    }
  }

  void ClampToLimits() {
    for (std::size_t i = 0; i < kArmDof; ++i) {
      const double lo = model_.lowerPositionLimit[idx_q_[i]];
      const double hi = model_.upperPositionLimit[idx_q_[i]];
      if (lo < hi && (q_[idx_q_[i]] < lo || q_[idx_q_[i]] > hi)) {
        q_[idx_q_[i]] = std::clamp(q_[idx_q_[i]], lo, hi);
        v_[idx_v_[i]] = 0.0;
      }
    }
  }

  ArmVector GravityArm() {
    const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(model_, data_, q_);
    ArmVector out{};
    for (std::size_t i = 0; i < kArmDof; ++i) out[i] = g[idx_v_[i]];
    return out;
  }

  void Step(const ArmVector& tau_arm) {
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model_.nv);
    for (std::size_t i = 0; i < kArmDof; ++i) {
      tau[idx_v_[i]] = tau_arm[i] - friction_ * v_[idx_v_[i]];
    }
    const Eigen::VectorXd& a = pinocchio::aba(model_, data_, q_, v_, tau);
    v_ += a * kDt;  // semi-implicit Euler
    q_ = pinocchio::integrate(model_, q_, v_ * kDt);
    ClampToLimits();
    applied_ = tau_arm;
  }

  void FillFrame(JointFrame& frame) const {
    const std::size_t n = kArmDof + other_names_.size();
    if (frame.names.size() != n) {
      frame.Resize(n);
      for (std::size_t i = 0; i < kArmDof; ++i) frame.names[i] = kArmJointNames[i];
      for (std::size_t j = 0; j < other_names_.size(); ++j) frame.names[kArmDof + j] = other_names_[j];
    }
    frame.timestamp_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    for (std::size_t i = 0; i < kArmDof; ++i) {
      frame.position[i] = q_[idx_q_[i]];
      frame.velocity[i] = v_[idx_v_[i]];
      frame.effort[i] = applied_[i];
    }
    for (std::size_t j = 0; j < other_names_.size(); ++j) {  // locked at neutral (0)
      frame.position[kArmDof + j] = 0.0;
      frame.velocity[kArmDof + j] = 0.0;
      frame.effort[kArmDof + j] = 0.0;
    }
  }

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_, v_;
  std::array<int, kArmDof> idx_q_{}, idx_v_{};
  std::vector<std::string> other_names_;
  ArmVector held_effort_{}, applied_{};
  double friction_{0.5};
  std::atomic<int> mode_{3};
  std::mutex mutex_;
};

}  // namespace

extern "C" __attribute__((visibility("default"))) g2_torque_subscriber::RobotBackend*
g2_create_robot_backend() {
  return new FakeBackend();
}
