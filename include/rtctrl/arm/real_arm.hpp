#pragma once

#include "rtctrl/arm/arm.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace rtctrl::arm {

// The real robot behind the sim⇄real bridge: adapts hw::CraneX7 (and
// its background read-write thread) to the Arm interface, so the same
// Controller binary runs here and on SimArm.
//
// tau in JointState is estimated from the measured current via the
// per-model torque constants. A successful bus read wakes the controller;
// step() blocks until the next read opens its bounded same-cycle command
// window. A deadman escalation makes step() return false.
class RealArm : public Arm {
 public:
  explicit RealArm(hw::CraneX7& hw)
      : hw_(hw), command_values_(model::kCanonicalDof) {}

  int dof() const override { return model::kCanonicalDof; }
  double dt() const override;
  bool activate() override;
  bool deactivate() override;
  bool setMode(ControlMode mode) override;
  bool readState(JointState& state,
                 CommandSnapshot* cmds = nullptr) override;
  bool writeCommand(const JointCommand& cmd,
                    CommandReceipt* receipt = nullptr) override;
  bool step() override;

 private:
  hw::CraneX7& hw_;
  std::uint64_t feedback_seen_ = 0;
  std::uint64_t last_state_read_seq_ = 0;
  std::uint64_t last_feedback_seq_ = 0;
  double last_feedback_time_ = 0.0;
  bool have_feedback_ = false;
  std::vector<double> command_values_;
};

}  // namespace rtctrl::arm
