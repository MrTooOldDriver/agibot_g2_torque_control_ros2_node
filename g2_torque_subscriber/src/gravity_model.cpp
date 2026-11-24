#include "g2_torque_subscriber/gravity_model.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace g2_torque_subscriber {

namespace {
bool IsArmJoint(const std::string& name) {
  return std::any_of(kArmJointNames.begin(), kArmJointNames.end(),
                     [&](const char* n) { return name == n; });
}
}  // namespace

GravityModel::GravityModel(const std::string& urdf_path) : urdf_path_(urdf_path) {
  if (urdf_path.empty()) throw std::runtime_error("empty URDF path");
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);  // fixed base
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot load URDF '" + urdf_path + "': " + e.what());
  }
  data_ = pinocchio::Data(model_);
  q_ = pinocchio::neutral(model_);

  for (std::size_t i = 0; i < kArmDof; ++i) {
    const std::string name = kArmJointNames[i];
    if (!model_.existJointName(name)) throw std::runtime_error("URDF has no joint " + name);
    const auto& joint = model_.joints[model_.getJointId(name)];
    if (joint.nq() != 1 || joint.nv() != 1) {
      throw std::runtime_error("URDF joint " + name + " is not 1-DoF");
    }
    arm_q_[i] = joint.idx_q();
    arm_v_[i] = joint.idx_v();
    effort_limit_[i] = std::max(1.0, static_cast<double>(model_.effortLimit[joint.idx_v()]));
  }
  for (std::size_t j = 1; j < static_cast<std::size_t>(model_.njoints); ++j) {
    const auto& joint = model_.joints[j];
    if (joint.nq() == 1 && joint.nv() == 1 && !IsArmJoint(model_.names[j])) {
      others_.push_back({model_.names[j], joint.idx_q()});
    }
  }
}

bool GravityModel::Bind(const JointFrame& frame) {
  const std::size_t n = frame.names.size();
  if (bound_size_ == n && n > 0) {
    bool same = true;
    for (std::size_t i = 0; i < kArmDof && same; ++i) {
      same = arm_frame_idx_[i] < n && frame.names[arm_frame_idx_[i]] == kArmJointNames[i];
    }
    if (same) return true;
  }
  // Layout changed (or first frame): rebuild the index. Allocates, but only on layout changes.
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < n; ++i) index[frame.names[i]] = i;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    const auto it = index.find(kArmJointNames[i]);
    if (it == index.end()) {
      bound_size_ = 0;
      return false;
    }
    arm_frame_idx_[i] = it->second;
  }
  bound_others_.clear();
  for (const auto& o : others_) {
    const auto it = index.find(o.name);
    if (it != index.end()) bound_others_.push_back({it->second, o.idx_q});
  }
  bound_size_ = n;
  return true;
}

void GravityModel::Extract(const JointFrame& frame, ArmVector& q, ArmVector& dq,
                           ArmVector& effort) const {
  for (std::size_t i = 0; i < kArmDof; ++i) {
    const std::size_t k = arm_frame_idx_[i];
    q[i] = frame.position[k];
    dq[i] = frame.velocity[k];
    effort[i] = frame.effort[k];
  }
}

const ArmVector& GravityModel::Gravity(const JointFrame& frame) {
  for (const auto& b : bound_others_) q_[b.idx_q] = frame.position[b.frame_idx];
  for (std::size_t i = 0; i < kArmDof; ++i) q_[arm_q_[i]] = frame.position[arm_frame_idx_[i]];
  const Eigen::VectorXd& g = pinocchio::computeGeneralizedGravity(model_, data_, q_);
  for (std::size_t i = 0; i < kArmDof; ++i) gravity_[i] = g[arm_v_[i]];
  return gravity_;
}

}  // namespace g2_torque_subscriber
