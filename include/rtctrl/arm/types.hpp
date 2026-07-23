#pragma once

#include <cstdint>

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

// Per-joint flags on an applied command.
inline constexpr std::uint8_t kCmdClamped = 1;  // magnitude-limited
inline constexpr std::uint8_t kCmdGated = 2;    // position-limit gated

// A successfully applied target. The producer re-evaluates limits and
// position gating on EVERY retransmission of the same target, so this
// record separates two facts about one target_seq: the FIRST
// application (latency verification; preserved across retransmissions)
// and the LATEST transmission (the current actuator goal with its
// current saturation/gate state). All times are on the producer's
// absolute clock (JointState::t's clock).
struct AppliedTargetRecord {
  bool valid = false;  // false at startup — no target applied yet
  std::uint64_t target_seq = 0;
  std::uint64_t first_cycle = 0;
  double first_time = 0.0;
  std::uint64_t latest_cycle = 0;
  double latest_time = 0.0;
  std::uint8_t mode = 0;  // raw DXL operating mode of the applied values
  // Mode-native units: rad (position), rad/s (velocity), Nm (current
  // mode, converted through the per-model torque constant).
  double applied[model::kCanonicalDof] = {};
  std::uint8_t flags[model::kCanonicalDof] = {};  // kCmdClamped|kCmdGated
};

// The most recent write ATTEMPT — every transmission and
// retransmission, including failures (which never touch the applied
// record: the actuator retains its previous goal).
struct WriteAttemptRecord {
  bool valid = false;
  std::uint64_t target_seq = 0;
  double time = 0.0;
  bool ok = false;
};

// Both command records copied atomically with the feedback they
// accompany (one lock hold on the producer).
struct CommandSnapshot {
  AppliedTargetRecord applied;
  WriteAttemptRecord last_attempt;
};

// Returned by Arm::writeCommand: which submission this call became.
struct CommandReceipt {
  bool accepted = false;
  std::uint64_t submitted_seq = 0;
  double submission_time = 0.0;  // producer's absolute clock
};

}  // namespace rtctrl::arm
