#pragma once

#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

namespace rtctrl::arm {

// Raw Dynamixel operating-mode values.
enum class ControlMode : int { Current = 0, Velocity = 1, Position = 3 };

// All vectors are in the canonical 8-DOF order (arm joints 1..7, then
// the gripper) — see rtctrl::model::canonicalJoints().
struct JointState {
  model::ZVector q{model::kCanonicalDof};
  model::ZVector dq{model::kCanonicalDof};
  model::ZVector tau{model::kCanonicalDof};  // real HW: estimated from current
  // ABSOLUTE feedback-acquisition time on the producer's clock (real
  // HW: CraneX7's injectable now_(); sim: sim time). The RUNNER alone
  // subtracts the first sample's t to obtain the controller-relative
  // time — no other layer keeps a time origin.
  double t = 0.0;
  std::uint64_t seq = 0;  // bumps once per fresh feedback sample
};

struct JointCommand {
  ControlMode mode = ControlMode::Position;
  model::ZVector q{model::kCanonicalDof};    // Position mode
  model::ZVector dq{model::kCanonicalDof};   // Velocity mode
  model::ZVector tau{model::kCanonicalDof};  // Current (torque) mode
};

}  // namespace rtctrl::arm
