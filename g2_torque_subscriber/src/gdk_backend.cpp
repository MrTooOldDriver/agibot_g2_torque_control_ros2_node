// RobotBackend on top of the real AGIBOT GDK (C++ API, libgdk_adapter.so).
// Built as a MODULE and opened by the node with dlopen(RTLD_LOCAL | RTLD_DEEPBIND); see
// robot_backend.hpp for why.

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <string>

#include "g2_torque_subscriber/robot_backend.hpp"
#include "gdk/gdk.h"

namespace {

using agibot::gdk::GDKRes;
using g2_torque_subscriber::ArmVector;
using g2_torque_subscriber::BackendOptions;
using g2_torque_subscriber::JointFrame;
using g2_torque_subscriber::kArmDof;
using g2_torque_subscriber::kArmJointNames;
using g2_torque_subscriber::TorqueCallback;

std::string ResToString(GDKRes res) {
  switch (res) {
    case GDKRes::kSuccess: return "kSuccess";
    case GDKRes::kInvalidInput: return "kInvalidInput";
    case GDKRes::kTimeout: return "kTimeout";
    case GDKRes::kRuntimeError: return "kRuntimeError";
    default: return "GDKRes(" + std::to_string(static_cast<int>(res)) + ")";
  }
}

// Copy one GDK frame into a JointFrame. After the first frame the vectors keep their capacity and
// the joint names are unchanged, so this does not allocate inside the 1 kHz loop.
void CopyFrame(const agibot::gdk::JointStates& in, JointFrame& out) {
  const std::size_t n = in.states.size();
  if (out.names.size() != n) out.Resize(n);
  out.timestamp_ns = in.timestamp;
  for (std::size_t i = 0; i < n; ++i) {
    const auto& s = in.states[i];
    if (out.names[i] != s.name) out.names[i] = s.name;
    out.position[i] = s.position;
    out.velocity[i] = s.velocity;
    out.effort[i] = s.effort;
  }
}

class GdkBackend final : public g2_torque_subscriber::RobotBackend {
 public:
  ~GdkBackend() override { Disconnect(); }

  std::string Name() const override { return "gdk"; }

  bool Connect(const BackendOptions& /*options*/, std::string& error) override {
    if (robot_) return true;
    try {
      const GDKRes res = agibot::gdk::GDKInit();
      if (res != GDKRes::kSuccess) {
        error = "GDKInit failed: " + ResToString(res);
        return false;
      }
      initialized_ = true;
      robot_ = std::make_unique<agibot::gdk::Robot>();
      return true;
    } catch (const std::exception& e) {
      error = std::string("GDK connect failed: ") + e.what();
    } catch (...) {
      error = "GDK connect failed: unknown exception";
    }
    return false;
  }

  bool ReadJointStates(JointFrame& frame) override {
    if (!robot_) return false;
    agibot::gdk::JointStates s;
    if (robot_->GetJointStates(s) != GDKRes::kSuccess) return false;
    CopyFrame(s, frame);
    return true;
  }

  bool GetUrdfType(std::string& urdf_type) override {
    if (!robot_) return false;
    agibot::gdk::WholeBodyStatus status;
    if (robot_->GetWholeBodyStatus(status) != GDKRes::kSuccess) return false;
    urdf_type = status.urdf_type;
    return true;
  }

  bool GetControlMode(int& mode) override {
    if (!robot_) return false;
    agibot::gdk::MotionControlStatus mc;
    if (robot_->GetMotionControlStatus(mc) != GDKRes::kSuccess) return false;
    mode = static_cast<int>(mc.control_mode);
    return true;
  }

  bool SetControlMode(int mode) override {
    return robot_ && robot_->SetControlMode(static_cast<uint8_t>(mode)) == GDKRes::kSuccess;
  }

  bool WaitControlMode(int mode, int timeout_ms) override {
    return robot_ && robot_->WaitControlMode(static_cast<uint8_t>(mode),
                                             std::chrono::milliseconds(timeout_ms)) ==
                         GDKRes::kSuccess;
  }

  bool RunTorqueLoop(const TorqueCallback& callback, bool enforce_realtime, int state_timeout_ms,
                     std::string& error) override {
    if (!robot_) {
      error = "not connected";
      return false;
    }
    agibot::gdk::JointTorqueControlReq req;
    req.joint_names.assign(kArmJointNames.begin(), kArmJointNames.end());
    req.joint_efforts.assign(kArmDof, 0.0);
    JointFrame frame;
    ArmVector tau{};
    bool finished_by_callback = false;

    auto gdk_callback = [&](const agibot::gdk::JointStates& s, uint64_t period_ns) {
      CopyFrame(s, frame);
      req.finished = callback(frame, period_ns, tau);
      std::copy(tau.begin(), tau.end(), req.joint_efforts.begin());
      finished_by_callback = req.finished;
      return req;
    };

    agibot::gdk::JointTorqueControlOptions options;
    options.enforce_realtime = enforce_realtime;
    options.state_timeout = std::chrono::milliseconds(state_timeout_ms);
    GDKRes res = GDKRes::kRuntimeError;
    try {
      res = robot_->JointTorqueControl(gdk_callback, options);
    } catch (const std::exception& e) {
      error = std::string("JointTorqueControl threw: ") + e.what();
      return false;
    } catch (...) {
      error = "JointTorqueControl threw an unknown exception";
      return false;
    }
    if (res != GDKRes::kSuccess) {
      error = "JointTorqueControl returned " + ResToString(res);
      if (res == GDKRes::kTimeout) error += " (no robot state frame within the state timeout)";
      return false;
    }
    if (!finished_by_callback) {
      error = "JointTorqueControl returned without a finished frame";
      return false;
    }
    return true;
  }

  void Disconnect() override {
    robot_.reset();
    if (initialized_) {
      agibot::gdk::GDKRelease();
      initialized_ = false;
    }
  }

 private:
  std::unique_ptr<agibot::gdk::Robot> robot_;
  bool initialized_{false};
};

}  // namespace

extern "C" __attribute__((visibility("default"))) g2_torque_subscriber::RobotBackend*
g2_create_robot_backend() {
  return new GdkBackend();
}
