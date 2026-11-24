// Rigid-body dynamics terms of the 14 G2 arm joints from a URDF (Pinocchio), the same terms as the
// vendor demo joint_torque_demo.py, which sends tau = rnea(q, dq, ddq_d) + impedance:
//
//   rnea(q, dq, ddq) = M(q) ddq + C(q, dq) dq + g(q)
//
//   Gravity()        g(q)
//   Coriolis()       C(q, v) v       Coriolis + centrifugal, v = measured velocities of ALL joints
//   Inertia(ddq)     M(q) ddq        ddq = desired acceleration of the arm joints (others 0)
//
// q (and v) of the non-arm joints (waist, head) are the measured ones from the frame, as in the
// demo: the waist posture changes the direction of gravity seen by the arms. In direct torque
// mode (9) the robot compensates none of these terms itself.

#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "g2_torque_subscriber/robot_backend.hpp"

namespace g2_torque_subscriber {

class DynamicsModel {
 public:
  // Throws std::runtime_error on an unreadable URDF or a missing / non-1-DoF arm joint.
  explicit DynamicsModel(const std::string& urdf_path);

  // Locate the arm joints (and the non-arm joints of the model) in a frame. Cheap and
  // allocation-free when the frame's joint names (all of them, in order) are unchanged. Returns
  // false if an arm joint is missing from the frame.
  bool Bind(const JointFrame& frame);

  // Arm position / velocity / measured effort of a bound frame, kArmJointNames order.
  void Extract(const JointFrame& frame, ArmVector& q, ArmVector& dq, ArmVector& effort) const;

  // Load the frame's positions and velocities (all joints in the model) for the terms below.
  void SetState(const JointFrame& frame);

  // Terms of the 14 arm joints at the state of the last SetState(). No allocation.
  const ArmVector& Gravity();
  const ArmVector& Coriolis();
  const ArmVector& Inertia(const ArmVector& ddq);

  // URDF effort limits of the arm joints (Nm, at least 1.0 each).
  const ArmVector& EffortLimits() const { return effort_limit_; }

 private:
  void EnsureGravity();

  struct Other {  // non-arm 1-DoF joint of the model
    std::string name;
    int idx_q;
    int idx_v;
  };
  struct BoundOther {
    std::size_t frame_idx;
    int idx_q;
    int idx_v;
  };

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_, v_, zero_v_, a_, g_full_;
  std::array<int, kArmDof> arm_q_{};
  std::array<int, kArmDof> arm_v_{};
  std::array<std::size_t, kArmDof> arm_frame_idx_{};
  ArmVector effort_limit_{};
  ArmVector gravity_{}, coriolis_{}, inertia_{};
  bool gravity_valid_{false};
  std::vector<Other> others_;
  std::vector<BoundOther> bound_others_;
  std::vector<std::string> bound_names_;  // frame layout the indices above belong to (empty: none)
};

}  // namespace g2_torque_subscriber
