#include "rtctrl/arm/real_arm.hpp"

#include "rtctrl/dxl/conversions.hpp"

namespace rtctrl::arm {

using model::kCanonicalDof;

double RealArm::dt() const { return hw_.options().control_cycle_s; }

bool RealArm::activate() {
  if (!hw_.activate()) return false;
  if (!hw_.startThread()) {
    hw_.deactivate();  // never leave a torqued arm behind a failure
    return false;
  }
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

bool RealArm::readState(JointState& state, CommandSnapshot* cmds) {
  // One producer-side lock hold covers feedback AND command records —
  // the background thread cannot advance between them.
  const auto stamped = hw_.lastFeedbackStamped(cmds);
  const auto& fb = stamped.feedback;
  if (fb.size() != static_cast<std::size_t>(kCanonicalDof)) return false;
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(state.q.get(), i) = fb[i].position;
    zVecElemNC(state.dq.get(), i) = fb[i].velocity;
    zVecElemNC(state.tau.get(), i) =
        fb[i].current *
        dxl::torqueConstant(hw_.config().joints[i].model_number);
  }
  // Absolute acquisition stamp on CraneX7's injectable clock — the
  // runner owns the time origin (types.hpp).
  state.t = stamped.time;
  state.seq = stamped.seq;
  return true;
}

bool RealArm::writeCommand(const JointCommand& cmd,
                           CommandReceipt* receipt) {
  std::vector<double> values(kCanonicalDof);
  std::uint64_t seq = 0;
  double time = 0.0;
  bool ok = false;
  switch (cmd.mode) {
    case ControlMode::Position:
      for (int i = 0; i < kCanonicalDof; ++i) {
        values[i] = zVecElemNC(cmd.q.get(), i);
      }
      ok = hw_.setTargetPositions(values, &seq, &time);
      break;
    case ControlMode::Velocity:
      for (int i = 0; i < kCanonicalDof; ++i) {
        values[i] = zVecElemNC(cmd.dq.get(), i);
      }
      ok = hw_.setTargetVelocities(values, &seq, &time);
      break;
    case ControlMode::Current:
      for (int i = 0; i < kCanonicalDof; ++i) {
        // torque command -> current through the per-model constant
        values[i] = zVecElemNC(cmd.tau.get(), i) /
                    dxl::torqueConstant(hw_.config().joints[i].model_number);
      }
      ok = hw_.setTargetCurrents(values, &seq, &time);
      break;
  }
  if (receipt != nullptr) *receipt = {ok, seq, time};
  return ok;
}

bool RealArm::step() {
  const auto seq = hw_.waitCycle(cycle_seen_);
  if (seq == 0) return false;  // thread stopped (escalation or stop)
  cycle_seen_ = seq;
  return !hw_.escalated();
}

}  // namespace rtctrl::arm
