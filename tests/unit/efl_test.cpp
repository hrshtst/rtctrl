// EFL-0 mathematical checks (docs/records/history.md (EFL frozen specification)):
// estimator parity against the SHIPPED controller, the zero-error
// feedforward identity, mass-matrix linearity of the acceleration
// channel, PRACTICAL-GF replica parity, gravity-free rest behavior,
// clamp parity, and duplicate-timestamp handling.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "study/exact_feedback_linearization.hpp"
#include "rtctrl/arm/practical_computed_torque.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
using Catch::Matchers::WithinAbs;
using model::kCanonicalDof;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

// A deterministic state trace with dt jitter, one duplicate timestamp,
// and one stall beyond 3x nominal dt.
const std::vector<double> kTraceT = {0.0,   0.01,  0.021, 0.021,
                                     0.033, 0.083, 0.093};

arm::JointState traceState(double t) {
  arm::JointState s;
  for (int i = 0; i < kCanonicalDof; ++i) {
    s.q[i] = 0.1 * std::sin(3.0 * t + 0.5 * i) + 0.02 * i;
    s.dq[i] = 0.3 * std::cos(3.0 * t + 0.5 * i);
  }
  s.t = t;
  return s;
}

}  // namespace

TEST_CASE("host estimator parity with PracticalComputedTorque on a jittered trace",
          "[efl]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) qf[i] = 0.1;
  const auto traj = model::MinJerkTrajectory::withVelocityLimit(q0, qf, 0.3);

  arm::PracticalComputedTorque shipped(chain, map, traj, 6.0, 1.0);
  shipped.setNominalDt(0.01);
  x7::HostVelocityEstimator replica(0.01);

  arm::JointCommand cmd;
  for (const double t : kTraceT) {
    const auto state = traceState(t);
    shipped.update(state, cmd, t);
    replica.update(state.q.get(), t);
    for (int i = 0; i < kCanonicalDof; ++i) {
      // identical arithmetic, identical order — exact equality
      REQUIRE(replica.estimate(i) == shipped.velocityEstimate()[i]);
    }
  }
}

TEST_CASE("zero tracking error reduces EFL to the pure desired feedforward",
          "[efl]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) {
    q0[i] = 0.05 * i;
    qf[i] = 0.05 * i + 0.2;
  }
  const auto traj = model::MinJerkTrajectory::withVelocityLimit(q0, qf, 0.3);

  x7::AccelDomainOptions opt;
  opt.eval = x7::AccelDomainOptions::Eval::Measured;
  opt.state_velocity = true;  // exact velocity: e and de are exactly 0
  x7::AccelDomainController efl(chain, map, traj, 36.0, 8.4, opt);

  const double t = 0.4 * traj.duration();
  model::ZVector q_d(kCanonicalDof), dq_d(kCanonicalDof),
      ddq_d(kCanonicalDof), expect(kCanonicalDof);
  traj.sample(t, q_d.get(), dq_d.get(), ddq_d.get());
  chain.inverseDynamics(map, q_d.get(), dq_d.get(), ddq_d.get(),
                        expect.get());

  arm::JointState state;
  zVecCopyNC(q_d.get(), state.q.get());
  zVecCopyNC(dq_d.get(), state.dq.get());
  state.t = t;
  arm::JointCommand cmd;
  efl.update(state, cmd, 0.0);  // first sample: soft start
  state.t = t;
  efl.update(state, cmd, t);    // on-trajectory sample
  for (int i = 0; i < kCanonicalDof; ++i) {
    REQUIRE_THAT(zVecElemNC(cmd.tau.get(), i),
                 WithinAbs(expect[i], 1e-9));
  }
}

TEST_CASE("acceleration channel is the mass-matrix action", "[efl]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector q(kCanonicalDof), dq(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) {
    q[i] = 0.1 + 0.07 * i;
    dq[i] = 0.05 * i;
  }
  model::ZVector v1(kCanonicalDof), v2(kCanonicalDof), dv(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) {
    v1[i] = 0.3 - 0.02 * i;
    v2[i] = -0.1 + 0.05 * i;
    dv[i] = v1[i] - v2[i];
  }
  model::ZVector tau1(kCanonicalDof), tau2(kCanonicalDof);
  chain.inverseDynamics(map, q.get(), dq.get(), v1.get(), tau1.get());
  chain.inverseDynamics(map, q.get(), dq.get(), v2.get(), tau2.get());
  // M(q)·dv via two calls at zero velocity (C and g cancel)
  model::ZVector zero(kCanonicalDof), m_dv(kCanonicalDof),
      g_only(kCanonicalDof);
  chain.inverseDynamics(map, q.get(), zero.get(), dv.get(), m_dv.get());
  chain.inverseDynamics(map, q.get(), zero.get(), zero.get(),
                        g_only.get());
  for (int i = 0; i < kCanonicalDof; ++i) {
    REQUIRE_THAT(tau1[i] - tau2[i],
                 WithinAbs(m_dv[i] - g_only[i], 1e-9));
  }
}

TEST_CASE("PRACTICAL-GF replica matches PracticalComputedTorque in ordinary mode",
          "[efl]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) qf[i] = 0.3;
  const auto traj = model::MinJerkTrajectory::withVelocityLimit(q0, qf, 0.3);

  // tight limits so the anti-windup and clamp paths are exercised
  double tau_max[kCanonicalDof];
  for (int i = 0; i < kCanonicalDof; ++i) tau_max[i] = 0.8;
  const double scales[kCanonicalDof] = {1.0, 1.0, 0.7, 0.7,
                                        0.1, 0.3, 0.2, 0.2};

  arm::PracticalComputedTorque shipped(chain, map, traj, 6.0, 1.0);
  x7::PracticalReplica replica(chain, map, traj, 6.0, 1.0, false);
  shipped.setIntegral(6.0, 1.5);
  shipped.setGainScales(scales);
  shipped.setNominalDt(0.01);
  shipped.setTorqueLimits(tau_max);
  replica.setIntegral(6.0, 1.5);
  replica.setGainScales(scales);
  replica.setNominalDt(0.01);
  replica.setTorqueLimits(tau_max);

  arm::JointCommand cmd_a, cmd_b;
  for (const double t : kTraceT) {
    const auto state = traceState(t);
    shipped.update(state, cmd_a, t);
    replica.update(state, cmd_b, t);
    for (int i = 0; i < kCanonicalDof; ++i) {
      REQUIRE(zVecElemNC(cmd_b.tau.get(), i) ==
              zVecElemNC(cmd_a.tau.get(), i));
      REQUIRE(replica.controllerSaturated(i) ==
              shipped.controllerSaturated(i));
      REQUIRE(replica.integralTerm()[i] == shipped.integralTerm()[i]);
    }
  }
}

TEST_CASE("gravity-free controllers emit zero torque at rest", "[efl]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector pose(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) pose[i] = 0.1 * i - 0.3;
  const x7::ConstantTrajectory traj(pose);

  x7::AccelDomainOptions opt;
  opt.eval = x7::AccelDomainOptions::Eval::Measured;
  opt.state_velocity = true;
  opt.gravity_free = true;
  x7::AccelDomainController efl(chain, map, traj, 36.0, 8.4, opt);

  arm::JointState state;
  zVecCopyNC(pose.get(), state.q.get());
  arm::JointCommand cmd;
  efl.update(state, cmd, 0.0);
  efl.update(state, cmd, 0.01);  // post-soft-start: v = 0 at rest
  for (int i = 0; i < kCanonicalDof; ++i) {
    REQUIRE_THAT(zVecElemNC(cmd.tau.get(), i), WithinAbs(0.0, 1e-9));
  }
}

TEST_CASE("acceleration-domain clamp mirrors the shipped semantics",
          "[efl]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) qf[i] = 0.4;
  const auto traj = model::MinJerkTrajectory::withVelocityLimit(q0, qf, 0.3);
  double tau_max[kCanonicalDof];
  for (int i = 0; i < kCanonicalDof; ++i) tau_max[i] = 0.05;

  x7::AccelDomainOptions opt;
  opt.tau_max = tau_max;
  x7::AccelDomainController efl(chain, map, traj, 64.0, 11.2, opt);

  arm::JointCommand cmd;
  efl.update(traceState(0.0), cmd, 0.0);
  efl.update(traceState(0.01), cmd, 0.01);
  bool any_saturated = false;
  for (int i = 0; i < kCanonicalDof; ++i) {
    const double out = zVecElemNC(cmd.tau.get(), i);
    REQUIRE(std::abs(out) <= 0.05 + 1e-15);
    if (efl.controllerSaturated(i)) {
      any_saturated = true;
      REQUIRE_THAT(std::abs(out), WithinAbs(0.05, 1e-15));
    }
  }
  REQUIRE(any_saturated);  // gravity alone exceeds a 0.05 Nm limit
}

TEST_CASE("duplicate timestamps re-emit the previous command", "[efl]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) qf[i] = 0.2;
  const auto traj = model::MinJerkTrajectory::withVelocityLimit(q0, qf, 0.3);

  x7::AccelDomainOptions opt;
  x7::AccelDomainController efl(chain, map, traj, 36.0, 8.4, opt);

  arm::JointCommand cmd;
  efl.update(traceState(0.0), cmd, 0.0);
  efl.update(traceState(0.01), cmd, 0.01);
  std::vector<double> before(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) {
    before[i] = zVecElemNC(cmd.tau.get(), i);
  }
  // same timestamp, different (bogus) state: nothing may change
  auto dup = traceState(0.5);
  dup.t = 0.01;
  efl.update(dup, cmd, 0.01);
  for (int i = 0; i < kCanonicalDof; ++i) {
    REQUIRE(zVecElemNC(cmd.tau.get(), i) == before[i]);
  }
}
