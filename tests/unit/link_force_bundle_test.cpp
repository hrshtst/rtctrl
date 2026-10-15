#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <toml++/toml.hpp>

#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

#include "common/link_force_bundle.hpp"
#include "common/link_force_run.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/link_force.hpp"

using Catch::Matchers::ContainsSubstring;

namespace {

namespace fs = std::filesystem;

rtctrl::model::LinkForceSpec pulse(double onset, double end) {
  rtctrl::model::LinkForceSpec spec;
  spec.link = "crane_x7_tcp_link";
  spec.direction_world = {0.0, 1.0, 0.0};
  spec.magnitude_n = 1.0;
  spec.profile.onset_s = onset;
  spec.profile.rise_s = 0.1;
  spec.profile.fall_s = 0.1;
  spec.profile.hold_s = end - onset - 0.2;
  return spec;
}

}  // namespace

TEST_CASE("the disturbance log sits next to the simulation log",
          "[link_force][bundle]") {
  CHECK(x7::disturbanceLogPath("out/simulation.csv") ==
        fs::path("out/simulation_disturbance.csv"));
  CHECK(x7::disturbanceLogPath("run") == fs::path("run_disturbance.csv"));
  CHECK(x7::disturbanceLogPath("/tmp/a.b/log.csv") ==
        fs::path("/tmp/a.b/log_disturbance.csv"));
}

TEST_CASE("simulation provenance records actual application",
          "[link_force][bundle]") {
  x7::LinkForceOutcome outcome;
  auto table = toml::parse(x7::simulationLinkForceProvenance(outcome));
  CHECK(table["link_force_spec"].value_or(std::string{}) == "link_force.toml");
  CHECK(table["link_force_enabled"].value_or(false));
  CHECK_FALSE(table["link_force_started"].value_or(true));
  CHECK_FALSE(table["link_force_applied"].value_or(true));
  CHECK_FALSE(table.contains("link_force_first_applied_s"));

  outcome.started = true;
  table = toml::parse(x7::simulationLinkForceProvenance(outcome));
  CHECK(table["link_force_started"].value_or(false));
  CHECK_FALSE(table["link_force_applied"].value_or(true));

  outcome.applied = true;
  outcome.first_applied_s = 1.25;
  outcome.last_applied_s = 2.5;
  table = toml::parse(x7::simulationLinkForceProvenance(outcome));
  CHECK(table["link_force_applied"].value_or(false));
  CHECK(table["link_force_first_applied_s"].value_or(0.0) == 1.25);
  CHECK(table["link_force_last_applied_s"].value_or(0.0) == 2.5);
}

TEST_CASE("hardware provenance marks the spec as unapplied",
          "[link_force][bundle]") {
  const auto table = toml::parse(x7::hardwareLinkForceProvenance());
  CHECK(table["link_force_spec"].value_or(std::string{}) == "link_force.toml");
  CHECK_FALSE(table["link_force_enabled"].value_or(true));
  CHECK_THAT(table["link_force_note"].value_or(std::string{}),
             ContainsSubstring("not applied by this run"));
  CHECK_THAT(table["link_force_note"].value_or(std::string{}),
             ContainsSubstring("not evidence that any apparatus applied"));
  CHECK_FALSE(table.contains("link_force_applied"));
}

TEST_CASE("link-force preflight rejects a force that could never act",
          "[link_force][bundle]") {
  rtctrl::model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  CHECK_NOTHROW(x7::preflightLinkForce(pulse(0.4, 1.4), chain, 4.0));
  // cut off at the end of the run: warned, not rejected
  CHECK_NOTHROW(x7::preflightLinkForce(pulse(0.4, 5.0), chain, 4.0));
  CHECK_THROWS_WITH(x7::preflightLinkForce(pulse(4.0, 5.0), chain, 4.0),
                    ContainsSubstring("would never act"));
  auto unknown = pulse(0.4, 1.4);
  unknown.link = "no_such_link";
  CHECK_THROWS_WITH(x7::preflightLinkForce(unknown, chain, 4.0),
                    ContainsSubstring("unknown link 'no_such_link'"));
}
