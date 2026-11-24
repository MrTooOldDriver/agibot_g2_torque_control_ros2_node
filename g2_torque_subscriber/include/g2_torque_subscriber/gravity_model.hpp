// Gravity torques of the 14 G2 arm joints from a URDF (Pinocchio).
//
// In direct torque mode (9) the robot does no gravity compensation, so the node adds
// g(q) = computeGeneralizedGravity(q) to every external command, as libfranka does internally on
// a Franka. Non-arm joints (waist, head) take their measured position from the frame, because the
// waist posture changes the direction of gravity seen by the arms.

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

class GravityModel {
 public:
  // Throws std::runtime_error on an unreadable URDF or a missing / non-1-DoF arm joint.
  explicit GravityModel(const std::string& urdf_path);

  // Locate the arm joints (and the non-arm joints of the model) in a frame. Cheap when the frame
  // layout is unchanged. Returns false if an arm joint is missing from the frame.
  bool Bind(const JointFrame& frame);

  // Arm position / velocity / measured effort of a bound frame, kArmJointNames order.
  void Extract(const JointFrame& frame, ArmVector& q, ArmVector& dq, ArmVector& effort) const;

  // Gravity torques of the 14 arm joints at the frame's configuration (call Bind first).
  const ArmVector& Gravity(const JointFrame& frame);

  // URDF effort limits of the arm joints (Nm, at least 1.0 each).
  const ArmVector& EffortLimits() const { return effort_limit_; }

  const std::string& UrdfPath() const { return urdf_path_; }

 private:
  struct Other {  // non-arm 1-DoF joint of the model
    std::string name;
    int idx_q;
  };
  struct BoundOther {
    std::size_t frame_idx;
    int idx_q;
  };

  std::string urdf_path_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;
  std::array<int, kArmDof> arm_q_{};
  std::array<int, kArmDof> arm_v_{};
  std::array<std::size_t, kArmDof> arm_frame_idx_{};
  ArmVector effort_limit_{};
  ArmVector gravity_{};
  std::vector<Other> others_;
  std::vector<BoundOther> bound_others_;
  std::size_t bound_size_{0};
};

}  // namespace g2_torque_subscriber
