#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

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

struct CurrentBasedHoldController : Controller {
  explicit CurrentBasedHoldController(const JointState& target) {
    zVecCopyNC(target.q.get(), q_target.get());
  }
  void update(const JointState&, JointCommand& cmd, double) override {
    cmd.mode = ControlMode::CurrentBasedPosition;
    zVecCopyNC(q_target.get(), cmd.q.get());
    for (int i = 0; i < kCanonicalDof; ++i) cmd.effort_limit[i] = 2.0;
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

TEST_CASE("simulation advances the exact nonintegral control period",
          "[sim][timing]") {
  auto opt = baseOptions();
  opt.control_dt = 1.0 / 60.0;
  opt.sim_dt = 0.001;
  SimArm arm(opt);
  REQUIRE(arm.step());
  CHECK(arm.time() == Catch::Approx(opt.control_dt).margin(1e-14));
  REQUIRE(arm.step());
  CHECK(arm.time() == Catch::Approx(2.0 * opt.control_dt).margin(1e-14));
}

TEST_CASE("current-based position simulation enforces its effort ceiling",
          "[sim][modes]") {
  auto opt = baseOptions();
  opt.initial_q8 = {0.0, 0.4, 0.0, -0.8, 0.0, -0.3, 0.0, 0.1};
  SimArm arm(opt);
  REQUIRE(arm.setMode(ControlMode::CurrentBasedPosition));
  REQUIRE(arm.activate());
  JointState target;
  REQUIRE(arm.readState(target));
  CurrentBasedHoldController hold(target);
  REQUIRE(rtctrl::arm::run(arm, hold, 0.2));
  JointState state;
  REQUIRE(arm.readState(state));
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(std::fabs(state.tau[i]) <= 2.0 + 1e-12);
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

namespace {

// Commands `target` in `mode` every cycle (the CBP ceiling at 2 Nm).
void runTo(SimArm& arm, ControlMode mode, const JointState& target,
           double duration) {
  JointCommand cmd;
  cmd.mode = mode;
  zVecCopyNC(target.q.get(), cmd.q.get());
  for (int i = 0; i < kCanonicalDof; ++i) cmd.effort_limit[i] = 2.0;
  for (double t = 0.0; t < duration; t += arm.dt()) {
    REQUIRE(arm.writeCommand(cmd));
    REQUIRE(arm.step());
  }
}

// Bitwise equality of two states (q, dq, tau and time).
void requireSameState(SimArm& a, SimArm& b) {
  JointState sa, sb;
  REQUIRE(a.readState(sa));
  REQUIRE(b.readState(sb));
  CHECK(sa.t == sb.t);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(sa.q[i] == sb.q[i]);
    CHECK(sa.dq[i] == sb.dq[i]);
    CHECK(sa.tau[i] == sb.tau[i]);
  }
  CHECK(a.fingerBDis() == b.fingerBDis());
}

}  // namespace

TEST_CASE("per-joint simulated servo gains equal to the scalars are bit-identical",
          "[sim][servo_like]") {
  for (const auto mode :
       {ControlMode::Position, ControlMode::CurrentBasedPosition}) {
    auto scalar = baseOptions();
    scalar.initial_q8 = {0.2, 0.5, -0.1, -1.1, 0.3, -0.4, 0.2, 0.3};
    auto vector = scalar;
    vector.kp8.assign(kCanonicalDof, scalar.kp);
    vector.kd8.assign(kCanonicalDof, scalar.kd);
    SimArm a(scalar), b(vector);
    for (SimArm* arm : {&a, &b}) {
      REQUIRE(arm->setMode(mode));
      REQUIRE(arm->activate());
    }
    JointState target;
    REQUIRE(a.readState(target));
    for (int i = 0; i < kCanonicalDof; ++i) target.q[i] += 0.1;  // move
    runTo(a, mode, target, 1.0);
    runTo(b, mode, target, 1.0);
    requireSameState(a, b);
  }
}

TEST_CASE("per-joint simulated servo gains deflect each joint by tau_g / kp",
          "[sim][servo_like]") {
  auto opt = baseOptions();
  opt.initial_q8 = {0.3, 0.6, -0.2, -1.2, 0.4, -0.5, 0.3, 0.2};
  // Distinct gains, so that a joint-index mix-up shows; kd = 0.2 kp
  // overdamps every joint (slowest pole near kp/kd = 5 /s).
  opt.kp8 = {20.0, 60.0, 25.0, 20.0, 30.0, 22.0, 40.0, 20.0};
  for (const double kp : opt.kp8) opt.kd8.push_back(0.2 * kp);
  SimArm arm(opt);
  REQUIRE(arm.setMode(ControlMode::Position));
  REQUIRE(arm.activate());
  JointState goal;
  REQUIRE(arm.readState(goal));  // activation snaps the goal here
  runTo(arm, ControlMode::Position, goal, 8.0);
  JointState state;
  REQUIRE(arm.readState(state));

  // At rest the servo torque kp (u - q) carries gravity alone: numeric
  // damping and kd vanish with the velocity, and the finger coupling
  // cancels in the canonical gripper torque (finger_a + finger_b).
  rtctrl::model::ChainModel model(opt.model_path);
  JointMap map(model);
  rtctrl::model::ZVector gravity(kCanonicalDof);
  model.gravityTorque(map, state.q.get(), gravity.get());
  double largest = 0.0;
  for (int i = 0; i < kCanonicalDof; ++i) {
    INFO("joint " << i);
    CHECK(std::fabs(state.dq[i]) < 1e-7);
    // 1e-6 rad: the settling residual (eight seconds are about forty
    // time constants of the slowest pole) and finger_b's departure from
    // the exact mimic that gravityTorque assumes (together about 1e-8
    // rad), orders below the deflections (up to about 0.04 rad here).
    CHECK(goal.q[i] - state.q[i] ==
          Catch::Approx(gravity[i] / opt.kp8[i]).margin(1e-6));
    largest = std::max(largest, std::fabs(gravity[i] / opt.kp8[i]));
  }
  CHECK(largest > 0.02);  // the posture really loads the soft servos
}

TEST_CASE("simulated servo gain and friction vectors are validated",
          "[sim][servo_like]") {
  const auto rejects = [](auto mutate) {
    auto opt = baseOptions();
    mutate(opt);
    CHECK_THROWS_AS(SimArm(opt), std::invalid_argument);
  };
  const std::vector<double> seven(7, 1.0);
  rejects([&](SimArm::Options& o) { o.kp8 = seven; });
  rejects([&](SimArm::Options& o) { o.kd8 = seven; });
  rejects([&](SimArm::Options& o) { o.fc8 = seven; });
  rejects([&](SimArm::Options& o) { o.fv8 = seven; });
  rejects([](SimArm::Options& o) { o.kp8.assign(9, 1.0); });
  const auto with = [](std::size_t index, double value) {
    std::vector<double> values(kCanonicalDof, 1.0);
    values[index] = value;
    return values;
  };
  const double nan = std::nan("");
  const double inf = HUGE_VAL;
  rejects([&](SimArm::Options& o) { o.kp8 = with(3, 0.0); });
  rejects([&](SimArm::Options& o) { o.kp8 = with(0, -20.0); });
  rejects([&](SimArm::Options& o) { o.kp8 = with(7, nan); });
  rejects([&](SimArm::Options& o) { o.kp8 = with(1, inf); });
  rejects([&](SimArm::Options& o) { o.kd8 = with(2, -0.1); });
  rejects([&](SimArm::Options& o) { o.kd8 = with(5, nan); });
  rejects([&](SimArm::Options& o) { o.fc8 = with(6, -0.01); });
  rejects([&](SimArm::Options& o) { o.fc8 = with(4, inf); });
  rejects([&](SimArm::Options& o) { o.fv8 = with(0, -1.0); });
  rejects([&](SimArm::Options& o) { o.fv8 = with(7, nan); });
  rejects([](SimArm::Options& o) { o.friction_v0 = 0.0; });
  rejects([](SimArm::Options& o) { o.friction_v0 = -0.05; });
  rejects([&](SimArm::Options& o) { o.friction_v0 = nan; });

  auto valid = baseOptions();
  valid.kd8.assign(kCanonicalDof, 0.0);  // kd = 0 and zero friction pass
  valid.fc8.assign(kCanonicalDof, 0.0);
  valid.fv8.assign(kCanonicalDof, 0.0);
  CHECK_NOTHROW(SimArm(valid));
}

TEST_CASE("zero simulated joint friction is bit-identical to friction off",
          "[sim][servo_like]") {
  auto off = baseOptions();
  off.initial_q8 = {0.2, 0.5, -0.1, -1.1, 0.3, -0.4, 0.2, 0.3};
  auto zero = off;
  zero.fc8.assign(kCanonicalDof, 0.0);
  zero.fv8.assign(kCanonicalDof, 0.0);
  SimArm a(off), b(zero);
  for (SimArm* arm : {&a, &b}) {
    REQUIRE(arm->setMode(ControlMode::Position));
    REQUIRE(arm->activate());
  }
  JointState target;
  REQUIRE(a.readState(target));
  for (int i = 0; i < kCanonicalDof; ++i) target.q[i] -= 0.1;
  runTo(a, ControlMode::Position, target, 1.0);
  runTo(b, ControlMode::Position, target, 1.0);
  requireSameState(a, b);
}

TEST_CASE("simulated joint friction follows the Coulomb plus viscous law",
          "[sim][servo_like]") {
  // Joint 0 turns about the vertical, so gravity never acts on it. It is
  // left free (a vanishing stiffness, no damping) while the stiff default
  // servo holds every other joint; numeric damping is off, so friction is
  // the only resistance it meets.
  const double fc = 0.05, fv = 0.5, v0 = 0.05, drive = 0.3;
  auto opt = baseOptions();
  opt.initial_q8 = {0.0, 0.6, 0.0, -1.2, 0.0, -0.5, 0.0, 0.2};
  opt.numeric_damping_ratio = 0.0;
  opt.kp8.assign(kCanonicalDof, opt.kp);
  opt.kd8.assign(kCanonicalDof, opt.kd);
  opt.kp8[0] = 1e-9;  // [Nm/rad]: under 1e-8 Nm over the whole run
  opt.kd8[0] = 0.0;
  opt.fc8 = {fc, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  opt.fv8 = {fv, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  opt.friction_v0 = v0;
  SimArm arm(opt);
  REQUIRE(arm.setMode(ControlMode::Position));
  REQUIRE(arm.activate());
  JointState goal;
  REQUIRE(arm.readState(goal));
  const auto law = [&](double v) {
    return -fc * std::tanh(v / v0) - fv * v;
  };

  // Held velocity: a constant drive torque against friction settles at
  // the speed where the law balances it (bisection; the law is monotone).
  // The discrete integrator has the same fixed point, so the match is
  // limited only by the remaining transient: two seconds are about
  // seventeen time constants I / fv (I near 0.058 kg m^2).
  arm.setDisturbance(0, drive);
  runTo(arm, ControlMode::Position, goal, 2.0);
  double lo = 0.0, hi = drive / fv;
  for (int k = 0; k < 200; ++k) {
    const double mid = 0.5 * (lo + hi);
    (law(mid) + drive > 0.0 ? lo : hi) = mid;
  }
  JointState state;
  REQUIRE(arm.readState(state));
  CHECK(state.dq[0] == Catch::Approx(lo).margin(1e-5));
  CHECK(lo > 0.4);  // the Coulomb level is saturated at this speed

  // Release: I dv/dt = law(v) with I the joint's own inertia, its
  // mass-matrix diagonal at the posture plus the reflected motor inertia.
  rtctrl::model::ChainModel model(opt.model_path);
  JointMap map(model);
  rtctrl::model::ZVector q9(rtctrl::model::kModelDof);
  map.expand(state.q.get(), q9.get());
  rkChainSetJointDisAll(model.chain(), q9.get());
  rkChainUpdateFK(model.chain());
  zMat mass = zMatAllocSqr(rtctrl::model::kModelDof);
  REQUIRE(mass != nullptr);
  rkChainInertiaMat(model.chain(), mass);
  const int offset = map.rokiOffset(0);
  const double inertia =
      zMatElemNC(mass, offset, offset) + opt.reflected_inertia;
  zMatFree(mass);

  arm.setDisturbance(0, 0.0);
  double v = state.dq[0];
  double worst = 0.0;
  const int substeps = 1000;  // RK4 at 1e-5 s: exact to far below the check
  const double h = arm.dt() / substeps;
  for (int cycle = 0; cycle < 60; ++cycle) {
    runTo(arm, ControlMode::Position, goal, arm.dt());
    for (int k = 0; k < substeps; ++k) {
      const double k1 = law(v) / inertia;
      const double k2 = law(v + 0.5 * h * k1) / inertia;
      const double k3 = law(v + 0.5 * h * k2) / inertia;
      const double k4 = law(v + h * k3) / inertia;
      v += h * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
    }
    REQUIRE(arm.readState(state));
    worst = std::max(worst, std::fabs(state.dq[0] - v));
  }
  // 1e-3 rad/s, against a fall from 0.5 rad/s to rest: the simulator's
  // semi-implicit Euler at 1e-4 s carries a first-order error (about
  // 1e-4 rad/s here), and the held joints' small deflections barely
  // change the inertia; a misplaced or mis-scaled term would miss by
  // tenths of the speed.
  CHECK(worst < 1e-3);
  CHECK(std::fabs(state.dq[0]) < 0.01);  // came to rest within 0.6 s
}
