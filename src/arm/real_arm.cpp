#include "rtctrl/arm/real_arm.hpp"

#include <chrono>

#include "rtctrl/dxl/conversions.hpp"

namespace rtctrl::arm {

using model::kCanonicalDof;

namespace {
double monotonicSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
}  // namespace

double RealArm::dt() const { return hw_.options().control_cycle_s; }

bool RealArm::activate() {
  if (!hw_.activate()) return false;
  if (!hw_.startThread()) {
    hw_.deactivate();  // never leave a torqued arm behind a failure
    return false;
  }
  t0_ = monotonicSeconds();
  return true;
}

bool RealArm::deactivate() { return hw_.deactivate(); }

bool RealArm::setMode(ControlMode mode) {
  if (hw_.activated()) return false;
  // The group's operating mode comes from the deployment config; this
  // only verifies the requested bridge mode matches it.
  for (const auto& joint : hw_.config().joints) {
    if (joint.operating_mode != static_cast<std::uint8_t>(mode)) {
      return false;
    }
  }
  return true;
}

bool RealArm::readState(JointState& state) {
  const auto fb = hw_.lastFeedback();
  if (fb.size() != static_cast<std::size_t>(kCanonicalDof)) return false;
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(state.q.get(), i) = fb[i].position;
    zVecElemNC(state.dq.get(), i) = fb[i].velocity;
    zVecElemNC(state.tau.get(), i) =
        fb[i].current *
        dxl::torqueConstant(hw_.config().joints[i].model_number);
  }
  state.t = t0_ < 0.0 ? 0.0 : monotonicSeconds() - t0_;
  return true;
}

bool RealArm::writeCommand(const JointCommand& cmd) {
  std::vector<double> values(kCanonicalDof);
  switch (cmd.mode) {
    case ControlMode::Position:
      for (int i = 0; i < kCanonicalDof; ++i) {
        values[i] = zVecElemNC(cmd.q.get(), i);
      }
      return hw_.setTargetPositions(values);
    case ControlMode::Velocity:
      for (int i = 0; i < kCanonicalDof; ++i) {
        values[i] = zVecElemNC(cmd.dq.get(), i);
      }
      return hw_.setTargetVelocities(values);
    case ControlMode::Current:
      for (int i = 0; i < kCanonicalDof; ++i) {
        // torque command -> current through the per-model constant
        values[i] = zVecElemNC(cmd.tau.get(), i) /
                    dxl::torqueConstant(hw_.config().joints[i].model_number);
      }
      return hw_.setTargetCurrents(values);
  }
  return false;
}

bool RealArm::step() {
  const auto seq = hw_.waitCycle(cycle_seen_);
  if (seq == 0) return false;  // thread stopped (escalation or stop)
  cycle_seen_ = seq;
  return !hw_.escalated();
}

}  // namespace rtctrl::arm
