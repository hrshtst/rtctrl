#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/arm/runner.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/joint_map.hpp"

using rtctrl::arm::Controller;
using rtctrl::arm::ControlMode;
using rtctrl::arm::JointCommand;
using rtctrl::arm::JointState;
using rtctrl::arm::SimArm;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;

namespace {

SimArm::Options baseOptions() {
  SimArm::Options opt;
  opt.model_path = "models/crane_x7/crane_x7.ztk";
  return opt;
}

// Commands a fixed target pose every cycle.
struct HoldController : Controller {
  explicit HoldController(const JointState& target) {
    zVecCopyNC(target.q.get(), q_target.get());
  }
  void update(const JointState& state, JointCommand& cmd, double t) override {
    (void)state;
    (void)t;
    cmd.mode = ControlMode::Position;
    zVecCopyNC(q_target.get(), cmd.q.get());
  }
  rtctrl::model::ZVector q_target{kCanonicalDof};
};

}  // namespace

TEST_CASE("deactivated arm falls under gravity without blowing up",
          "[sim]") {
  SimArm arm(baseOptions());
  arm.deactivate();

  JointState state;
  bool moved = false;
  for (double t = 0.0; t < 5.0; t += arm.dt()) {
    REQUIRE(arm.step());
  }
  REQUIRE(arm.readState(state));
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(std::isfinite(state.q[i]));
    CHECK(std::isfinite(state.dq[i]));
    if (std::fabs(state.q[i]) > 1e-3) moved = true;
  }
  CHECK(moved);  // gravity actually acted on the limp arm
}

TEST_CASE("position-mode PD holds a gravity-loaded pose", "[sim]") {
  auto opt = baseOptions();
  opt.initial_q8 = {0.0, 0.6, 0.0, -1.2, 0.0, -0.5, 0.0, 0.2};
  SimArm arm(opt);
  REQUIRE(arm.setMode(ControlMode::Position));
  REQUIRE(arm.activate());

  JointState initial;
  REQUIRE(arm.readState(initial));

  HoldController hold(initial);
  REQUIRE(rtctrl::arm::run(arm, hold, 3.0));

  JointState final;
  REQUIRE(arm.readState(final));
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(std::fabs(final.q[i] - initial.q[i]) < 0.05);
  }
}

TEST_CASE("simulation is deterministic across identical runs", "[sim]") {
  auto opt = baseOptions();
  opt.initial_q8 = {0.0, 0.4, 0.0, -1.0, 0.0, -0.4, 0.0, 0.1};

  auto runOnce = [&opt]() {
    SimArm arm(opt);
    arm.setMode(ControlMode::Position);
    arm.activate();
    for (double t = 0.0; t < 1.0; t += arm.dt()) {
      REQUIRE(arm.step());
    }
    JointState state;
    REQUIRE(arm.readState(state));
    return state;
  };

  const JointState a = runOnce();
  const JointState b = runOnce();
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(a.q[i] == b.q[i]);  // bitwise identical
    CHECK(a.dq[i] == b.dq[i]);
  }
}

TEST_CASE("finger coupling bounds mimic divergence under asymmetric load",
          "[sim]") {
  auto opt = baseOptions();
  opt.initial_q8 = {0.0, 0.0, 0.0, -0.8, 0.0, 0.0, 0.0, 0.4};
  SimArm arm(opt);
  arm.setMode(ControlMode::Position);
  arm.activate();

  // Emulate contact blocking one finger: constant external torque on
  // finger_b only.
  arm.setDisturbance(-1, 0.5);

  for (double t = 0.0; t < 2.0; t += arm.dt()) {
    REQUIRE(arm.step());
  }

  // Read committed state — the chain's per-joint values are solver
  // workspace after rkFDUpdate.
  JointState state;
  REQUIRE(arm.readState(state));
  const double q_a = state.q[kCanonicalDof - 1];
  const double q_b = arm.fingerBDis();

  CHECK(std::isfinite(q_a));
  CHECK(std::isfinite(q_b));
  // With couple_k = 50 Nm/rad, a 0.5 Nm asymmetric load statically
  // diverges 0.01 rad; the acceptance threshold documents the bound.
  CHECK(std::fabs(q_b - q_a) < 0.03);
}
