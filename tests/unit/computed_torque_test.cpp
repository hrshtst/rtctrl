// ComputedTorque hardening regressions (docs/REMEDIATION_PLAN.md D5):
// first-sample initialization, duplicate-timestamp freeze, measured-dt
// velocity estimation, and direction-aware anti-windup with the
// committed-state output identity.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

using Catch::Approx;
namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
using model::kCanonicalDof;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

struct Harness {
  explicit Harness(double kp = 6.0, double kd = 1.0)
      : chain(kModelPath),
        map(chain),
        hold(zero.get(), zero.get(), 1.0),
        ctl(chain, map, hold, kp, kd) {}

  void set(double q1, double dq_servo, double t) {
    for (int i = 0; i < kCanonicalDof; ++i) {
      zVecElemNC(state.q.get(), i) = 0.0;
      zVecElemNC(state.dq.get(), i) = dq_servo;
    }
    zVecElemNC(state.q.get(), 1) = q1;
    state.t = t;
    ctl.update(state, cmd, t);
  }

  model::ZVector zero{kCanonicalDof};
  model::ChainModel chain;
  model::JointMap map;
  model::MinJerkTrajectory hold;  // constant reference at the zero pose
  arm::ComputedTorque ctl;
  arm::JointState state;
  arm::JointCommand cmd;
};

}  // namespace

TEST_CASE("first sample emits feedforward only; servo dq never seeds",
          "[computed_torque]") {
  Harness h;
  // large position error AND a large (lagged, untrustworthy) servo dq
  h.set(/*q1=*/-0.3, /*dq_servo=*/5.0, /*t=*/0.0);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(zVecElemNC(h.cmd.tau.get(), i) ==
          Approx(h.ctl.feedforward()[i]).margin(1e-12));
    CHECK(h.ctl.velocityEstimate()[i] == 0.0);
    CHECK(h.ctl.pdTerm()[i] == 0.0);
    CHECK(h.ctl.integralTerm()[i] == 0.0);
  }
  // second fresh sample: the D path comes from the position backward
  // difference, still not from the servo estimate
  h.set(-0.3, 5.0, 0.01);
  CHECK(std::fabs(h.ctl.velocityEstimate()[1]) < 0.5);  // not ~5 rad/s
}

TEST_CASE("duplicate timestamps freeze every controller state",
          "[computed_torque]") {
  Harness h;
  h.set(-0.2, 0.0, 0.00);
  h.set(-0.2, 0.0, 0.01);
  h.set(-0.2, 0.0, 0.02);
  const double i_before = h.ctl.integralTerm()[1];
  const double pd_before = h.ctl.pdTerm()[1];
  const double est_before = h.ctl.velocityEstimate()[1];
  const double out_before = zVecElemNC(h.cmd.tau.get(), 1);
  // same t, garbage state: everything must hold
  h.set(+0.7, -9.0, 0.02);
  CHECK(h.ctl.integralTerm()[1] == Approx(i_before));
  CHECK(h.ctl.pdTerm()[1] == Approx(pd_before));
  CHECK(h.ctl.velocityEstimate()[1] == Approx(est_before));
  CHECK(zVecElemNC(h.cmd.tau.get(), 1) == Approx(out_before));
}

TEST_CASE("velocity estimate honors measured, irregular dt",
          "[computed_torque]") {
  Harness h;
  // joint 1 moving at exactly 1 rad/s, one sample 30 ms late: with the
  // measured dt the raw difference is 1 rad/s every cycle; a nominal-dt
  // assumption would have read 3 rad/s at the gap
  double q = 0.0;
  h.set(q, 0.0, 0.0);
  const double times[] = {0.01, 0.02, 0.03, 0.06, 0.07, 0.08};
  double prev_t = 0.0;
  for (const double t : times) {
    q += (t - prev_t) * 1.0;
    h.set(q, 0.0, t);
    prev_t = t;
    CHECK(h.ctl.velocityEstimate()[1] <= 1.0 + 1e-9);
  }
  CHECK(h.ctl.velocityEstimate()[1] > 0.5);  // converging toward 1
}

TEST_CASE("anti-windup: entry, sustained saturation, unwinding, recovery",
          "[computed_torque]") {
  // kd = 0: the synthetic error flips would otherwise produce a huge
  // backward-difference D kick that legitimately saturates the
  // candidate in the OPPOSITE direction (a correct rejection, but not
  // the scenario under test)
  Harness h(6.0, 0.0);
  h.ctl.setIntegral(6.0, 1.5);
  double tau_max[kCanonicalDof];
  for (int i = 0; i < kCanonicalDof; ++i) tau_max[i] = 0.3;
  h.ctl.setTorqueLimits(tau_max);

  // entry: constant positive error drives the command into saturation
  double t = 0.0;
  for (int k = 0; k < 200; ++k) {
    t += 0.01;
    h.set(-0.2, 0.0, t);  // e = +0.2 on joint 1
    // committed-state identity holds EVERY cycle, saturated or not
    CHECK(h.ctl.tauRaw()[1] ==
          Approx(h.ctl.feedforward()[1] + h.ctl.pdTerm()[1] +
                 h.ctl.integralTerm()[1])
              .margin(1e-12));
  }
  CHECK(h.ctl.controllerSaturated(1));
  CHECK(zVecElemNC(h.cmd.tau.get(), 1) == Approx(0.3));
  const double i_frozen = h.ctl.integralTerm()[1];
  CHECK(i_frozen < 1.5);  // frozen by saturation, not by the state clamp

  // sustained: further same-direction error must not integrate deeper
  for (int k = 0; k < 50; ++k) {
    t += 0.01;
    h.set(-0.2, 0.0, t);
  }
  CHECK(h.ctl.integralTerm()[1] == Approx(i_frozen));

  // unwinding: the error flips — integration must resume IMMEDIATELY
  // even though the command is still saturated positive
  t += 0.01;
  h.set(+0.2, 0.0, t);
  CHECK(h.ctl.integralTerm()[1] < i_frozen);

  // recovery: with the error settled to zero the PD decays and the
  // command leaves saturation
  for (int k = 0; k < 300; ++k) {
    t += 0.01;
    h.set(0.0, 0.0, t);
  }
  CHECK_FALSE(h.ctl.controllerSaturated(1));
}
