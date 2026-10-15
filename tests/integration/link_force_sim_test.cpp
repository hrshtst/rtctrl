#include <algorithm>
#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/link_force.hpp"
#include "rtctrl/model/zvector.hpp"

using Catch::Approx;
using rtctrl::arm::ControlMode;
using rtctrl::arm::JointCommand;
using rtctrl::arm::JointState;
using rtctrl::arm::LinkForceInterval;
using rtctrl::arm::LinkForceStatus;
using rtctrl::arm::SimArm;
using rtctrl::model::ChainModel;
using rtctrl::model::ForceDirection;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::kModelDof;
using rtctrl::model::LinkForceSpec;
using rtctrl::model::ZVector;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr const char* kTcp = "crane_x7_tcp_link";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const std::vector<double> kStart = {0.3, -0.6, 0.2, -1.4, 0.25, -0.5, 0.4, 0.0};

SimArm::Options options(double control_dt = 0.01, double sim_dt = 1e-4) {
  SimArm::Options opt;
  opt.model_path = kModelPath;
  opt.control_dt = control_dt;
  opt.sim_dt = sim_dt;
  opt.initial_q8 = kStart;
  return opt;
}

LinkForceSpec spec(double magnitude, double onset, double rise, double hold,
                   double fall,
                   std::array<double, 3> direction = {0.3, 1.0, -0.2}) {
  LinkForceSpec s;
  s.link = kTcp;
  s.point_link_m = {0.0, 0.0, 0.0};
  s.direction = ForceDirection::World;
  s.direction_world = direction;
  s.magnitude_n = magnitude;
  s.profile.onset_s = onset;
  s.profile.rise_s = rise;
  s.profile.hold_s = hold;
  s.profile.fall_s = fall;
  return s;
}

// A force already in its (long) hold when the clock starts.
LinkForceSpec holding(double magnitude,
                      std::array<double, 3> direction = {0.3, 1.0, -0.2}) {
  return spec(magnitude, 0.0, 0.05, 1000.0, 0.05, direction);
}

std::array<double, 3> unit(const std::array<double, 3>& v) {
  const double n = std::hypot(v[0], v[1], v[2]);
  return {v[0] / n, v[1] / n, v[2] / n};
}

void activate(SimArm& arm, ControlMode mode) {
  REQUIRE(arm.setMode(mode));
  REQUIRE(arm.activate());
}

struct Snapshot {
  std::array<double, kCanonicalDof> q{}, dq{}, tau{};
  double t = 0.0;
};

Snapshot snapshot(SimArm& arm) {
  JointState state;
  REQUIRE(arm.readState(state));
  Snapshot s;
  for (int i = 0; i < kCanonicalDof; ++i) {
    s.q[i] = zVecElemNC(state.q.get(), i);
    s.dq[i] = zVecElemNC(state.dq.get(), i);
    s.tau[i] = zVecElemNC(state.tau.get(), i);
  }
  s.t = state.t;
  return s;
}

void stepN(SimArm& arm, int n) {
  for (int k = 0; k < n; ++k) REQUIRE(arm.step());
}

double dot(const std::array<double, 3>& a, const std::array<double, 3>& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// Independent reference for one substep from rest: the model mass matrix
// (CRB, plus the simulator's reflected-inertia diagonal) and J^T f.
struct Reference {
  ChainModel model{kModelPath};
  JointMap map{model};
  ZVector q9{kModelDof};
  zMat mass = zMatAllocSqr(kModelDof);
  ~Reference() { zMatFree(mass); }

  explicit Reference(double reflected_inertia) {
    ZVector q8(kCanonicalDof);
    for (int i = 0; i < kCanonicalDof; ++i) q8[i] = kStart[i];
    map.expand(q8.get(), q9.get());
    model.fk(q9.get());
    rkChainInertiaMatCRB(model.chain(), mass);
    for (int i = 0; i < kModelDof; ++i) {
      zMatElemNC(mass, i, i) += reflected_inertia;
    }
  }
  // M^-1 J^T f in model coordinates
  void response(const std::array<double, 3>& f, ZVector* acc9) {
    ZVector ext9(kModelDof);
    model.forceJointTorqueModel(model.linkIndex(kTcp), {}, f, ext9.get());
    zMat m = zMatClone(mass);  // zLESolveGauss works in place
    REQUIRE(zLESolveGauss(m, ext9.get(), acc9->get()) != nullptr);
    zMatFree(m);
  }
};

std::array<double, 3> tcpPosition(ChainModel& model, const JointMap& map,
                                  const std::array<double, kCanonicalDof>& q) {
  ZVector q8(kCanonicalDof), q9(kModelDof);
  for (int i = 0; i < kCanonicalDof; ++i) q8[i] = q[i];
  map.expand(q8.get(), q9.get());
  model.fk(q9.get());
  return model.linkPointWorldPos(model.linkIndex(kTcp), {});
}

}  // namespace

TEST_CASE("a held link force balances the position servo at equilibrium",
          "[sim][link_force]") {
  SimArm arm(options());
  activate(arm, ControlMode::Position);
  const auto s = holding(5.0);
  arm.setLinkForce(s);
  arm.startLinkForce(arm.time() - 0.1);
  stepN(arm, 300);
  const Snapshot end = snapshot(arm);
  for (double v : end.dq) REQUIRE(std::abs(v) < 1e-6);

  ChainModel model(kModelPath);
  JointMap map(model);
  ZVector q8(kCanonicalDof), g8(kCanonicalDof), ext8(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) q8[i] = end.q[i];
  model.gravityTorque(map, q8.get(), g8.get());
  const auto u = unit(s.direction_world);
  model.forceJointTorque(map, q8.get(), model.linkIndex(kTcp), {},
                         {5.0 * u[0], 5.0 * u[1], 5.0 * u[2]}, ext8.get());
  double largest = 0.0;
  for (int i = 0; i < kCanonicalDof - 1; ++i) {
    INFO("canonical joint " << i);
    CHECK(end.tau[i] == Approx(g8[i] - ext8[i]).margin(2e-3));
    largest = std::max(largest, std::abs(ext8[i]));
  }
  CHECK(largest > 0.5);  // the force matters at this tolerance
}

TEST_CASE("one substep matches the dynamics equation and moves along u",
          "[sim][link_force]") {
  const auto opt = options(1e-4, 1e-4);
  SimArm free_arm(opt), pushed(opt);
  activate(free_arm, ControlMode::Current);
  activate(pushed, ControlMode::Current);
  const auto s = holding(7.0, {0.2, 1.0, -0.3});
  pushed.setLinkForce(s);
  pushed.startLinkForce(-0.1);
  REQUIRE(free_arm.step());
  REQUIRE(pushed.step());
  const Snapshot a = snapshot(free_arm);
  const Snapshot b = snapshot(pushed);

  Reference ref(opt.reflected_inertia);
  const auto u = unit(s.direction_world);
  ZVector acc9(kModelDof), acc_u(kModelDof), ext_u(kModelDof);
  ref.response({7.0 * u[0], 7.0 * u[1], 7.0 * u[2]}, &acc9);
  for (int i = 0; i < kCanonicalDof; ++i) {
    INFO("canonical joint " << i);
    const double expected = 1e-4 * acc9[ref.map.rokiOffset(i)];
    CHECK(b.dq[i] - a.dq[i] == Approx(expected).epsilon(1e-8).margin(1e-14));
  }

  // u^T J M^-1 J^T u > 0 (M positive definite), so one substep from rest
  // moves the point along u: delta p . u ~ F dt^2 u^T J M^-1 J^T u.
  ref.model.forceJointTorqueModel(ref.model.linkIndex(kTcp), {}, u,
                                  ext_u.get());
  ref.response(u, &acc_u);
  double quadratic = 0.0;
  for (int j = 0; j < kModelDof; ++j) quadratic += ext_u[j] * acc_u[j];
  REQUIRE(quadratic > 1.0);
  const auto pa = tcpPosition(ref.model, ref.map, a.q);
  const auto pb = tcpPosition(ref.model, ref.map, b.q);
  const std::array<double, 3> dp = {pb[0] - pa[0], pb[1] - pa[1],
                                    pb[2] - pa[2]};
  CHECK(dot(dp, u) > 0.0);
  CHECK(dot(dp, u) == Approx(7.0 * 1e-8 * quadratic).epsilon(1e-3));
}

TEST_CASE("the link force acts outside the saturated actuator clamp",
          "[sim][link_force]") {
  const auto opt = options(1e-4, 1e-4);
  SimArm none(opt), single(opt), twice(opt);
  for (SimArm* arm : {&none, &single, &twice}) {
    activate(*arm, ControlMode::Position);
    JointCommand cmd;
    cmd.mode = ControlMode::Position;
    for (int i = 0; i < kCanonicalDof; ++i) {
      zVecElemNC(cmd.q.get(), i) = kStart[i] + (i < 7 ? 1.0 : 0.0);
    }
    REQUIRE(arm->writeCommand(cmd));
  }
  single.setLinkForce(holding(100.0));
  single.startLinkForce(-0.1);
  twice.setLinkForce(holding(200.0));
  twice.startLinkForce(-0.1);
  REQUIRE(none.step());
  REQUIRE(single.step());
  REQUIRE(twice.step());

  const Snapshot c = snapshot(none);
  const Snapshot a = snapshot(single);
  const Snapshot b = snapshot(twice);
  // the servo is saturated on every arm joint (pre-clamp torque reported)
  for (int i = 0; i < kCanonicalDof - 1; ++i) {
    REQUIRE(std::abs(c.tau[i]) > opt.effort_limit8[i]);
  }
  double above_limit = 0.0;
  const auto& ext = twice.lastLinkForceInterval().last_joint_torque_nm;
  for (int i = 0; i < kCanonicalDof - 1; ++i) {
    above_limit =
        std::max(above_limit, std::abs(ext[i]) - opt.effort_limit8[i]);
  }
  CHECK(above_limit > 0.0);
  bool moved = false;
  for (int i = 0; i < kCanonicalDof; ++i) {
    INFO("canonical joint " << i);
    const double d1 = a.dq[i] - c.dq[i];
    const double d2 = b.dq[i] - c.dq[i];
    moved = moved || std::abs(d1) > 1e-6;
    CHECK(d2 == Approx(2.0 * d1).epsilon(1e-6).margin(1e-12));
  }
  CHECK(moved);
}

TEST_CASE("a complete pulse is followed by force-free recovery",
          "[sim][link_force]") {
  SimArm free_arm(options()), pushed(options());
  activate(free_arm, ControlMode::Position);
  activate(pushed, ControlMode::Position);
  const auto s = spec(5.0, 0.1, 0.05, 0.2, 0.05);  // ends at 0.4 s
  pushed.setLinkForce(s);
  pushed.startLinkForce(pushed.time());

  std::array<double, 3> impulse{};
  double deviation = 0.0;
  for (int k = 0; k < 150; ++k) {
    REQUIRE(free_arm.step());
    REQUIRE(pushed.step());
    const LinkForceInterval& iv = pushed.lastLinkForceInterval();
    CHECK(iv.complete);
    CHECK(iv.attempt == static_cast<std::uint64_t>(k));
    CHECK(iv.end_seq == iv.start_seq + 1);
    for (int j = 0; j < 3; ++j) impulse[j] += iv.force_impulse_ns[j];
    if (iv.start_s >= s.profile.end_s() - 1e-9) {
      CHECK_FALSE(iv.active);
      CHECK(iv.force_impulse_ns == std::array<double, 3>{0.0, 0.0, 0.0});
      CHECK(iv.joint_impulse_nms == std::array<double, kCanonicalDof>{});
    }
    const Snapshot a = snapshot(free_arm);
    const Snapshot b = snapshot(pushed);
    for (int i = 0; i < kCanonicalDof - 1; ++i) {
      deviation = std::max(deviation, std::abs(b.q[i] - a.q[i]));
    }
  }
  // the integrated impulse is the profile's, to O(sim_dt)
  const auto u = unit(s.direction_world);
  const double expected = 5.0 * s.profile.impulseScale();
  for (int j = 0; j < 3; ++j) {
    CHECK(impulse[j] == Approx(expected * u[j]).margin(5.0 * 1e-4));
  }
  CHECK(deviation > 1e-3);  // the pulse visibly displaced the arm
  const Snapshot a = snapshot(free_arm);
  const Snapshot b = snapshot(pushed);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(b.q[i] == Approx(a.q[i]).margin(1e-5));  // recovered
  }
  CHECK(pushed.linkForceApplied());
  CHECK(pushed.firstLinkForceTime() > 0.1);
  CHECK(pushed.firstLinkForceTime() < 0.1 + 2e-4);
  CHECK(pushed.lastLinkForceTime() < 0.4);
}

TEST_CASE("release before and inside a control interval", "[sim][link_force]") {
  SECTION("release completing just before a control boundary") {
    SimArm arm(options());
    activate(arm, ControlMode::Position);
    // ends half a substep before the 0.30 s boundary
    arm.setLinkForce(spec(5.0, 0.1, 0.05, 0.09995, 0.05));
    arm.startLinkForce(0.0);
    stepN(arm, 30);
    CHECK(arm.lastLinkForceInterval().active);
    REQUIRE(arm.step());
    const auto& next = arm.lastLinkForceInterval();
    CHECK_FALSE(next.active);
    CHECK(next.force_impulse_ns == std::array<double, 3>{0.0, 0.0, 0.0});
  }
  SECTION("release inside an interval") {
    SimArm arm(options());
    activate(arm, ControlMode::Position);
    arm.setLinkForce(spec(5.0, 0.1, 0.05, 0.105, 0.05));  // ends at 0.305 s
    arm.startLinkForce(0.0);
    stepN(arm, 31);
    const auto iv = arm.lastLinkForceInterval();
    CHECK(iv.start_s == Approx(0.30).margin(1e-9));
    CHECK(iv.active);
    CHECK(iv.last_scale == 0.0);  // its last substep is past the release
    CHECK(std::abs(iv.force_impulse_ns[1]) > 0.0);
    REQUIRE(arm.step());
    CHECK_FALSE(arm.lastLinkForceInterval().active);
  }
}

TEST_CASE("a profile shorter than one control period is fully accounted",
          "[sim][link_force]") {
  SimArm arm(options());
  activate(arm, ControlMode::Position);
  const auto s = spec(5.0, 0.0021, 0.001, 0.002, 0.001, {0.0, 1.0, 0.0});
  arm.setLinkForce(s);
  arm.startLinkForce(arm.time());
  REQUIRE(arm.step());
  const auto iv = arm.lastLinkForceInterval();
  CHECK(iv.active);
  CHECK(iv.last_scale == 0.0);  // the sampled end of the interval sees none
  CHECK(iv.force_impulse_ns[0] == 0.0);
  CHECK(iv.force_impulse_ns[1] ==
        Approx(5.0 * s.profile.impulseScale()).margin(5.0 * 1e-4));
  REQUIRE(arm.step());
  CHECK_FALSE(arm.lastLinkForceInterval().active);
}

TEST_CASE("interval timing reports a shortened final substep",
          "[sim][link_force]") {
  SimArm arm(options(0.01, 3e-4));
  activate(arm, ControlMode::Position);
  arm.setLinkForce(holding(2.0));
  arm.startLinkForce(-0.1);
  REQUIRE(arm.step());
  const auto& iv = arm.lastLinkForceInterval();
  CHECK(iv.substeps_completed == 34);
  CHECK(iv.last_substep_dt == Approx(1e-4).margin(1e-12));
  CHECK(iv.last_eval_s + iv.last_substep_dt == iv.end_s);
  CHECK(iv.end_s == Approx(0.01).margin(1e-12));
  CHECK(iv.start_s == 0.0);
}

TEST_CASE("zero requested force never evaluates a degenerate anchor",
          "[sim][link_force]") {
  // a world-fixed point with the anchor on it: undefined whenever a
  // nonzero force is requested
  LinkForceSpec pinned = spec(3.0, 0.05, 0.05, 0.1, 0.05);
  pinned.link = "world";
  pinned.direction = ForceDirection::Anchor;
  pinned.direction_world = {};
  pinned.anchor_world_m = {0.0, 0.0, 0.0};

  SECTION("prepared but not started") {
    SimArm arm(options());
    activate(arm, ControlMode::Position);
    arm.setLinkForce(pinned);
    stepN(arm, 20);
  }
  SECTION("after release") {
    SimArm arm(options());
    activate(arm, ControlMode::Position);
    arm.setLinkForce(pinned);
    arm.startLinkForce(-100.0);
    stepN(arm, 20);
    CHECK_FALSE(arm.linkForceApplied());
  }
  SECTION("zero magnitude") {
    SimArm arm(options());
    activate(arm, ControlMode::Position);
    LinkForceSpec zero = pinned;
    zero.magnitude_n = 0.0;
    arm.setLinkForce(zero);
    arm.startLinkForce(-0.06);  // in the hold
    stepN(arm, 20);
    CHECK_FALSE(arm.linkForceApplied());
  }
}

TEST_CASE("an anchor fault rejects the substep and latches the arm",
          "[sim][link_force]") {
  LinkForceSpec pinned = spec(3.0, 0.0153, 0.05, 0.1, 0.05);
  pinned.link = "world";
  pinned.direction = ForceDirection::Anchor;
  pinned.anchor_world_m = {0.0, 0.0, 0.0};

  SimArm arm(options());
  activate(arm, ControlMode::Position);
  arm.setLinkForce(pinned);
  arm.startLinkForce(arm.time());
  REQUIRE(arm.step());  // [0, 0.01] is before onset
  CHECK_FALSE(arm.step());
  const auto iv = arm.lastLinkForceInterval();
  CHECK(iv.attempt == 1);
  CHECK_FALSE(iv.complete);
  CHECK(iv.rejected);
  CHECK(iv.status == LinkForceStatus::AnchorFault);
  CHECK(iv.start_seq == 1);
  CHECK(iv.end_seq == iv.start_seq);
  CHECK(iv.rejected_eval_s > 0.0153);
  CHECK(iv.rejected_eval_s < 0.0153 + 1.5e-4);
  CHECK(iv.end_s == iv.rejected_eval_s);  // clock at the last completed
  CHECK(iv.substeps_completed > 0);
  CHECK_FALSE(iv.active);
  CHECK(iv.force_impulse_ns == std::array<double, 3>{0.0, 0.0, 0.0});
  CHECK(arm.linkForceFaulted());
  CHECK(arm.time() == iv.end_s);

  // latched: no feedback, no commands, no advance
  JointState state;
  CHECK_FALSE(arm.readState(state));
  JointCommand cmd;
  cmd.mode = ControlMode::Position;
  rtctrl::arm::CommandReceipt receipt;
  CHECK_FALSE(arm.writeCommand(cmd, &receipt));
  CHECK_FALSE(receipt.accepted);
  CHECK_FALSE(arm.step());
  const auto latched = arm.lastLinkForceInterval();
  CHECK(latched.attempt == 2);
  CHECK(latched.status == LinkForceStatus::FaultLatched);
  CHECK(latched.substeps_completed == 0);
  CHECK(latched.start_s == iv.end_s);
  CHECK(latched.end_s == iv.end_s);
  CHECK(latched.start_seq == latched.end_seq);
  CHECK(latched.start_seq == iv.end_seq);
  CHECK(arm.deactivate());
  arm.clearLinkForce();
  CHECK_FALSE(arm.step());
  CHECK(arm.lastLinkForceInterval().status == LinkForceStatus::FaultLatched);
  CHECK(arm.time() == iv.end_s);
}

TEST_CASE("an anchor fault on the first substep completes nothing",
          "[sim][link_force]") {
  LinkForceSpec pinned = holding(3.0);
  pinned.link = "world";
  pinned.direction = ForceDirection::Anchor;
  pinned.anchor_world_m = {0.0, 0.0, 0.0};
  SimArm arm(options());
  activate(arm, ControlMode::Position);
  arm.setLinkForce(pinned);
  arm.startLinkForce(-0.5);
  CHECK_FALSE(arm.step());
  const auto& iv = arm.lastLinkForceInterval();
  CHECK(iv.status == LinkForceStatus::AnchorFault);
  CHECK(iv.substeps_completed == 0);
  CHECK(iv.start_s == iv.end_s);
  CHECK(iv.start_seq == iv.end_seq);
  CHECK(iv.rejected_eval_s == 0.0);
  CHECK(std::isnan(iv.last_eval_s));
}

TEST_CASE(
    "a non-finite plant rejects the substep under link-force "
    "accounting",
    "[sim][link_force]") {
  // With the positive reflected-inertia diagonal the solve cannot fail
  // through public options; a NaN inertia makes the plant non-finite.
  // zLESolveGauss's pivoting decides whether that surfaces as a solve
  // failure or as a non-finite state; the observed outcome is pinned.
  auto opt = options();
  opt.reflected_inertia = kNaN;
  SimArm arm(opt);
  activate(arm, ControlMode::Position);
  arm.setLinkForce(holding(2.0));
  arm.startLinkForce(-0.1);
  CHECK_FALSE(arm.step());
  const auto& iv = arm.lastLinkForceInterval();
  CHECK(iv.rejected);
  CHECK(iv.status == LinkForceStatus::NonFiniteState);
  CHECK(iv.substeps_completed == 0);
  CHECK(iv.force_impulse_ns == std::array<double, 3>{0.0, 0.0, 0.0});
  CHECK(arm.time() == 0.0);
  CHECK(arm.linkForceFaulted());
}

TEST_CASE("an armed force before its onset is bitwise identical to none",
          "[sim][link_force]") {
  SimArm plain(options()), armed(options());
  activate(plain, ControlMode::Position);
  activate(armed, ControlMode::Position);
  armed.setLinkForce(spec(5.0, 100.0, 0.1, 0.1, 0.1));
  armed.startLinkForce(armed.time());
  for (int k = 0; k < 80; ++k) {
    for (SimArm* arm : {&plain, &armed}) {
      JointCommand cmd;
      cmd.mode = ControlMode::Position;
      for (int i = 0; i < kCanonicalDof; ++i) {
        zVecElemNC(cmd.q.get(), i) = kStart[i] + 0.2 * std::sin(0.05 * k + i);
      }
      REQUIRE(arm->writeCommand(cmd));
      REQUIRE(arm->step());
    }
  }
  const Snapshot a = snapshot(plain);
  const Snapshot b = snapshot(armed);
  CHECK(a.q == b.q);
  CHECK(a.dq == b.dq);
  CHECK(a.tau == b.tau);
  CHECK(a.t == b.t);
  CHECK(plain.fingerBDis() == armed.fingerBDis());
}

TEST_CASE("link-force runs are deterministic", "[sim][link_force]") {
  auto run = [] {
    SimArm arm(options());
    activate(arm, ControlMode::CurrentBasedPosition);
    arm.setLinkForce(spec(4.0, 0.05, 0.1, 0.2, 0.1));
    arm.startLinkForce(arm.time());
    for (int k = 0; k < 60; ++k) {
      JointCommand cmd;
      cmd.mode = ControlMode::CurrentBasedPosition;
      for (int i = 0; i < kCanonicalDof; ++i) {
        zVecElemNC(cmd.q.get(), i) = kStart[i];
        zVecElemNC(cmd.effort_limit.get(), i) = 2.0;
      }
      REQUIRE(arm.writeCommand(cmd));
      REQUIRE(arm.step());
    }
    return std::make_pair(snapshot(arm), arm.lastLinkForceInterval());
  };
  const auto [a, ia] = run();
  const auto [b, ib] = run();
  CHECK(a.q == b.q);
  CHECK(a.dq == b.dq);
  CHECK(ia.joint_impulse_nms == ib.joint_impulse_nms);
}

TEST_CASE("link-force lifecycle rejects misuse", "[sim][link_force]") {
  SimArm arm(options());
  CHECK_THROWS_AS(arm.startLinkForce(0.0), std::logic_error);
  auto unknown = holding(1.0);
  unknown.link = "no_such_link";
  CHECK_THROWS_AS(arm.setLinkForce(unknown), std::invalid_argument);
  auto invalid = holding(1.0);
  invalid.profile.rise_s = 0.0;
  CHECK_THROWS_AS(arm.setLinkForce(invalid), std::invalid_argument);
  arm.setLinkForce(holding(1.0));
  CHECK_THROWS_AS(arm.startLinkForce(kNaN), std::invalid_argument);
  arm.clearLinkForce();
  CHECK_THROWS_AS(arm.startLinkForce(0.0), std::logic_error);
}
