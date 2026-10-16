#pragma once

#include <array>
#include <cstdint>
#include <limits>

#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::arm {

// Outcome of one simulated substep (or step attempt) under a prescribed
// link force. Every status other than Ok rejects the substep: nothing it
// computed is committed, and the SimArm latches (see SimArm::setLinkForce).
enum class LinkForceStatus {
  Ok,
  AnchorFault,     // nonzero force requested, direction undefined
  NonFiniteForce,  // force, point, or generalized torque not finite
  SolveFailed,     // the dynamics solve failed
  NonFiniteState,  // acceleration or unclipped state update not finite
  FaultLatched,    // step attempted after an earlier rejection
};

const char* linkForceStatusName(LinkForceStatus status);

// Telemetry of one step() attempt. Times are SimArm sim time [s]; seq
// values use JointState::seq numbering (end_seq = start_seq + 1 only for
// a complete step). Impulses and last_* fields cover COMPLETED substeps
// only, and are exactly what those substeps applied: the external force
// is constant within a substep, so each impulse is a plain sum of
// force x substep length.
struct LinkForceInterval {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  std::uint64_t attempt = 0;
  std::uint64_t start_seq = 0;
  std::uint64_t end_seq = 0;
  double start_s = 0.0;
  double end_s = 0.0;  // clock after the last completed substep
  int substeps_completed = 0;
  bool complete = false;
  LinkForceStatus status = LinkForceStatus::Ok;
  bool rejected = false;          // a substep was evaluated, then rejected
  double rejected_eval_s = kNaN;  // its evaluation time
  bool prepared = false;          // a spec was set (strict accounting)
  bool started = false;           // the profile clock was running
  bool active = false;            // a completed substep applied force
  double last_eval_s = kNaN;      // last completed substep: start time,
  double last_substep_dt = kNaN;  // length,
  double last_scale = 0.0;        // and requested fraction of magnitude
  std::array<double, 3> last_force_world{};
  std::array<double, 3> last_point_world{kNaN, kNaN, kNaN};
  std::array<double, model::kCanonicalDof> last_joint_torque_nm{};
  std::array<double, 3> force_impulse_ns{};
  std::array<double, model::kCanonicalDof> joint_impulse_nms{};
};

namespace detail {

// What one disturbance-enabled substep established, in evaluation order.
// Fields after the first failure are not consulted.
struct SubstepFacts {
  bool force_requested = false;   // scale x magnitude > 0
  bool direction_defined = true;  // only read when force_requested
  bool force_finite = true;       // only read when force_requested
  bool solve_ok = true;
  bool state_finite = true;
};

// The single precedence rule for rejecting a substep.
LinkForceStatus classifySubstep(const SubstepFacts& facts);

}  // namespace detail

}  // namespace rtctrl::arm
