#include "rtctrl/arm/link_force_interval.hpp"

namespace rtctrl::arm {

const char* linkForceStatusName(LinkForceStatus status) {
  switch (status) {
    case LinkForceStatus::Ok:
      return "ok";
    case LinkForceStatus::AnchorFault:
      return "anchor_fault";
    case LinkForceStatus::NonFiniteForce:
      return "non_finite_force";
    case LinkForceStatus::SolveFailed:
      return "solve_failed";
    case LinkForceStatus::NonFiniteState:
      return "non_finite_state";
    case LinkForceStatus::FaultLatched:
      return "fault_latched";
  }
  return "unknown";
}

namespace detail {

LinkForceStatus classifySubstep(const SubstepFacts& facts) {
  if (facts.force_requested && !facts.direction_defined) {
    return LinkForceStatus::AnchorFault;
  }
  if (facts.force_requested && !facts.force_finite) {
    return LinkForceStatus::NonFiniteForce;
  }
  if (!facts.solve_ok) return LinkForceStatus::SolveFailed;
  if (!facts.state_finite) return LinkForceStatus::NonFiniteState;
  return LinkForceStatus::Ok;
}

}  // namespace detail

}  // namespace rtctrl::arm
