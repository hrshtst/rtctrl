#include <catch2/catch_test_macros.hpp>
#include <string>

#include "rtctrl/arm/link_force_interval.hpp"

using rtctrl::arm::LinkForceStatus;
using rtctrl::arm::detail::classifySubstep;
using rtctrl::arm::detail::SubstepFacts;

TEST_CASE("substep classification follows the evaluation order",
          "[link_force][substep]") {
  // Every combination of the five facts: the first failing stage in
  // evaluation order decides, and the force checks apply only when a
  // nonzero force is requested.
  for (int bits = 0; bits < 32; ++bits) {
    SubstepFacts facts;
    facts.force_requested = (bits & 1) != 0;
    facts.direction_defined = (bits & 2) != 0;
    facts.force_finite = (bits & 4) != 0;
    facts.solve_ok = (bits & 8) != 0;
    facts.state_finite = (bits & 16) != 0;
    LinkForceStatus expected = LinkForceStatus::Ok;
    if (facts.force_requested && !facts.direction_defined) {
      expected = LinkForceStatus::AnchorFault;
    } else if (facts.force_requested && !facts.force_finite) {
      expected = LinkForceStatus::NonFiniteForce;
    } else if (!facts.solve_ok) {
      expected = LinkForceStatus::SolveFailed;
    } else if (!facts.state_finite) {
      expected = LinkForceStatus::NonFiniteState;
    }
    INFO("bits " << bits);
    CHECK(classifySubstep(facts) == expected);
  }
  // spot checks of the rule's intent
  SubstepFacts idle;
  idle.direction_defined = false;  // never evaluated without a request
  idle.force_finite = false;
  CHECK(classifySubstep(idle) == LinkForceStatus::Ok);
  SubstepFacts solve;
  solve.solve_ok = false;
  CHECK(classifySubstep(solve) == LinkForceStatus::SolveFailed);
}

TEST_CASE("link-force statuses have stable names", "[link_force][substep]") {
  using rtctrl::arm::linkForceStatusName;
  CHECK(std::string(linkForceStatusName(LinkForceStatus::Ok)) == "ok");
  CHECK(std::string(linkForceStatusName(LinkForceStatus::AnchorFault)) ==
        "anchor_fault");
  CHECK(std::string(linkForceStatusName(LinkForceStatus::NonFiniteForce)) ==
        "non_finite_force");
  CHECK(std::string(linkForceStatusName(LinkForceStatus::SolveFailed)) ==
        "solve_failed");
  CHECK(std::string(linkForceStatusName(LinkForceStatus::NonFiniteState)) ==
        "non_finite_state");
  CHECK(std::string(linkForceStatusName(LinkForceStatus::FaultLatched)) ==
        "fault_latched");
}
