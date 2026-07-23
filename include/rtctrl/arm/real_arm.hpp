#pragma once

#include "rtctrl/arm/arm.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace rtctrl::arm {

// The real robot behind the sim⇄real bridge: adapts hw::CraneX7 (and
// its background read-write thread) to the Arm interface, so the same
// Controller binary runs here and on SimArm.
//
// tau in JointState is estimated from the measured current via the
// per-model torque constants. step() blocks until the next background
// cycle completes; a deadman escalation makes step() return false.
class RealArm : public Arm {
 public:
  explicit RealArm(hw::CraneX7& hw) : hw_(hw) {}

  int dof() const override { return model::kCanonicalDof; }
  double dt() const override;
  bool activate() override;
  bool deactivate() override;
  bool setMode(ControlMode mode) override;
  bool readState(JointState& state) override;
  bool writeCommand(const JointCommand& cmd) override;
  bool step() override;

 private:
  hw::CraneX7& hw_;
  std::uint64_t cycle_seen_ = 0;
};

}  // namespace rtctrl::arm
