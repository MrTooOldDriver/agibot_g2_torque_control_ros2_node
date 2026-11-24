// Robot access for the G2 torque subscriber node, behind an abstract interface.
//
// Why an interface loaded with dlopen(): the AGIBOT GDK bundles its own Fast DDS 3.x, boost 1.83,
// protobuf, ... Linked into the node directly, those libraries land in the global symbol scope and
// ROS 2's rmw_fastrtps (Fast DDS 2.x, same C++ namespace eprosima::fastdds::dds) binds to them and
// aborts on the first rclcpp::Node. The node therefore loads the GDK through a small module
// (libg2_gdk_backend.so) opened with RTLD_LOCAL | RTLD_DEEPBIND: the GDK keeps its own copies of
// everything and ROS keeps its own. The node only talks to the RobotBackend vtable below, so no
// symbol is resolved across the boundary. libg2_fake_backend.so implements the same interface with
// a simulated arm for offline tests.
//
// Keep this header free of GDK and ROS types, and of function-local statics (with RTLD_DEEPBIND each
// module gets its own copy of inline functions).

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace g2_torque_subscriber {

constexpr std::size_t kJointsPerArm = 7;
constexpr std::size_t kArmDof = 2 * kJointsPerArm;  // left joint1..7, then right joint1..7
using ArmVector = std::array<double, kArmDof>;

// The 14 arm joints in the node's canonical order: left arm joint1..7, then right arm joint1..7.
// Every 14-vector in this package uses this order; index 0..6 = left, 7..13 = right.
inline constexpr std::array<const char*, kArmDof> kArmJointNames = {
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3", "idx24_arm_l_joint4",
    "idx25_arm_l_joint5", "idx26_arm_l_joint6", "idx27_arm_l_joint7",
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3", "idx64_arm_r_joint4",
    "idx65_arm_r_joint5", "idx66_arm_r_joint6", "idx67_arm_r_joint7"};

constexpr int kModeJointTorque = 9;  // GDK control mode of direct joint torque control

// One whole-body robot state frame (arms, waist, head, ...), joints in the robot's order.
struct JointFrame {
  uint64_t timestamp_ns{0};
  std::vector<std::string> names;
  std::vector<double> position;  // rad
  std::vector<double> velocity;  // rad/s
  std::vector<double> effort;    // Nm, measured

  void Resize(std::size_t n) {
    names.resize(n);
    position.resize(n);
    velocity.resize(n);
    effort.resize(n);
  }
};

// Called once per 1 ms frame inside RunTorqueLoop, on the loop thread. Fill `tau` with the full
// joint torques of the 14 arm joints (Nm, kArmJointNames order, gravity included). Return true to
// make this the final frame of the stream. Must not throw.
using TorqueCallback =
    std::function<bool(const JointFrame& frame, uint64_t period_ns, ArmVector& tau)>;

struct BackendOptions {
  std::string urdf_path;  // used by the fake backend only (its simulated plant)
};

class RobotBackend {
 public:
  virtual ~RobotBackend() = default;

  virtual std::string Name() const = 0;

  // GDKInit() + Robot(). May block for a long time when no robot answers.
  virtual bool Connect(const BackendOptions& options, std::string& error) = 0;

  // Plain whole-body state read, usable in any control mode.
  virtual bool ReadJointStates(JointFrame& frame) = 0;

  // URDF type reported by the robot ("" if unknown).
  virtual bool GetUrdfType(std::string& urdf_type) = 0;

  virtual bool GetControlMode(int& mode) = 0;
  virtual bool SetControlMode(int mode) = 0;
  virtual bool WaitControlMode(int mode, int timeout_ms) = 0;

  // Blocking 1 kHz torque loop (mode 9 must already be active): read frame -> callback -> write.
  // Returns true when the callback ended the stream, false on an SDK error or read timeout
  // (`error` says why). The stream is ended on return; the control mode is NOT changed.
  virtual bool RunTorqueLoop(const TorqueCallback& callback, bool enforce_realtime,
                             int state_timeout_ms, std::string& error) = 0;

  // Destroy the Robot and GDKRelease().
  virtual void Disconnect() = 0;
};

}  // namespace g2_torque_subscriber

// Every backend module exports this factory (extern "C", looked up with dlsym).
#define G2_BACKEND_FACTORY_SYMBOL "g2_create_robot_backend"
extern "C" {
using G2CreateRobotBackendFn = g2_torque_subscriber::RobotBackend* (*)();
}
