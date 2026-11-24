#include "g2_torque_subscriber/dynamics_model.hpp"

#include <algorithm>
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

DynamicsModel::DynamicsModel(const std::string& urdf_path) {
  if (urdf_path.empty()) throw std::runtime_error("empty URDF path");
  try {
    pinocchio::urdf::buildModel(urdf_path, model_);  // fixed base
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot load URDF '" + urdf_path + "': " + e.what());
  }
  data_ = pinocchio::Data(model_);
  q_ = pinocchio::neutral(model_);
  v_ = Eigen::VectorXd::Zero(model_.nv);
  zero_v_ = Eigen::VectorXd::Zero(model_.nv);
  a_ = Eigen::VectorXd::Zero(model_.nv);
  g_full_ = Eigen::VectorXd::Zero(model_.nv);

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
      others_.push_back({model_.names[j], joint.idx_q(), joint.idx_v()});
    }
  }
}

bool DynamicsModel::Bind(const JointFrame& frame) {
  // Same layout as the bound one: every name, not only the arm joints, so that a reordering of the
  // waist / head joints cannot leave a stale index (wrong waist pose in g(q)). No allocation.
  if (!bound_names_.empty() && frame.names == bound_names_) return true;
  // Layout changed (or first frame): rebuild the index. Allocates, but only on layout changes.
  bound_names_.clear();
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i = 0; i < frame.names.size(); ++i) index[frame.names[i]] = i;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    const auto it = index.find(kArmJointNames[i]);
    if (it == index.end()) return false;
    arm_frame_idx_[i] = it->second;
  }
  bound_others_.clear();
  for (const auto& o : others_) {
    const auto it = index.find(o.name);
    if (it != index.end()) bound_others_.push_back({it->second, o.idx_q, o.idx_v});
  }
  bound_names_ = frame.names;
  return true;
}

void DynamicsModel::Extract(const JointFrame& frame, ArmVector& q, ArmVector& dq,
                            ArmVector& effort) const {
  for (std::size_t i = 0; i < kArmDof; ++i) {
    const std::size_t k = arm_frame_idx_[i];
    q[i] = frame.position[k];
    dq[i] = frame.velocity[k];
    effort[i] = frame.effort[k];
  }
}

void DynamicsModel::SetState(const JointFrame& frame) {
  for (const auto& b : bound_others_) {
    q_[b.idx_q] = frame.position[b.frame_idx];
    v_[b.idx_v] = frame.velocity[b.frame_idx];
  }
  for (std::size_t i = 0; i < kArmDof; ++i) {
    q_[arm_q_[i]] = frame.position[arm_frame_idx_[i]];
    v_[arm_v_[i]] = frame.velocity[arm_frame_idx_[i]];
  }
  gravity_valid_ = false;
}

void DynamicsModel::EnsureGravity() {
  if (gravity_valid_) return;
  g_full_ = pinocchio::computeGeneralizedGravity(model_, data_, q_);
  for (std::size_t i = 0; i < kArmDof; ++i) gravity_[i] = g_full_[arm_v_[i]];
  gravity_valid_ = true;
}

const ArmVector& DynamicsModel::Gravity() {
  EnsureGravity();
  return gravity_;
}

const ArmVector& DynamicsModel::Coriolis() {
  EnsureGravity();
  // nonLinearEffects = C(q, v) v + g(q)
  const Eigen::VectorXd& nle = pinocchio::nonLinearEffects(model_, data_, q_, v_);
  for (std::size_t i = 0; i < kArmDof; ++i) coriolis_[i] = nle[arm_v_[i]] - gravity_[i];
  return coriolis_;
}

const ArmVector& DynamicsModel::Inertia(const ArmVector& ddq) {
  EnsureGravity();
  // rnea(q, 0, a) = M(q) a + g(q), a = ddq on the arm joints, 0 elsewhere
  a_.setZero();
  for (std::size_t i = 0; i < kArmDof; ++i) a_[arm_v_[i]] = ddq[i];
  const Eigen::VectorXd& tau = pinocchio::rnea(model_, data_, q_, zero_v_, a_);
  for (std::size_t i = 0; i < kArmDof; ++i) inertia_[i] = tau[arm_v_[i]] - gravity_[i];
  return inertia_;
}

}  // namespace g2_torque_subscriber
