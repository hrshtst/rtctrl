// M8 acceptance in sim, through the bridge: computed-torque tracking of
// a minimum-jerk trajectory in current mode, and its superiority over
// the same PD without the inverse-dynamics feedforward.
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/crane_x7_tuning.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

using rtctrl::arm::ComputedTorque;
using rtctrl::arm::Controller;
using rtctrl::arm::ControlMode;
using rtctrl::arm::JointCommand;
using rtctrl::arm::JointState;
using rtctrl::arm::SimArm;
using rtctrl::model::ChainModel;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::MinJerkTrajectory;
using rtctrl::model::Trajectory;
using rtctrl::model::ZVector;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

// PD in torque space WITHOUT the inverse-dynamics feedforward — the
// baseline computed torque must beat.
struct PdOnly : Controller {
  PdOnly(const Trajectory& trajectory, double kp, double kd)
      : trajectory_(trajectory), kp_(kp), kd_(kd) {}
  void update(const JointState& state, JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d.get(), dq_d.get());
    cmd.mode = ControlMode::Current;
    for (int i = 0; i < kCanonicalDof; ++i) {
      zVecElemNC(cmd.tau.get(), i) =
          kp_ * (q_d[i] - zVecElemNC(state.q.get(), i)) +
          kd_ * (dq_d[i] - zVecElemNC(state.dq.get(), i));
    }
  }
  const Trajectory& trajectory_;
  double kp_, kd_;
  ZVector q_d{kCanonicalDof}, dq_d{kCanonicalDof};
};

// Runs `controller` over the trajectory on a fresh sim; returns the
// tracking RMS across all joints and samples.
double trackingRms(const Trajectory& trajectory,
                   Controller& controller, double duration) {
  SimArm::Options opt;
  opt.model_path = kModelPath;
  opt.initial_q8 = {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0, 0.1};
  SimArm arm(opt);
  arm.setMode(ControlMode::Current);
  arm.activate();

  ZVector q_d(kCanonicalDof);
  double sum_sq = 0.0;
  int samples = 0;
  JointState state;
  JointCommand cmd;
  for (double t = 0.0; t < duration; t += arm.dt()) {
    arm.readState(state);
    controller.update(state, cmd, t);
    arm.writeCommand(cmd);
    if (!arm.step()) return 1e9;
    trajectory.sample(t, q_d.get());
    for (int i = 0; i < kCanonicalDof; ++i) {
      const double e = q_d[i] - state.q[i];
      sum_sq += e * e;
      ++samples;
    }
  }
  return std::sqrt(sum_sq / samples);
}

}  // namespace

TEST_CASE("computed torque tracks in sim and beats bare PD",
          "[tracking][sim]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);

  ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  const double start[] = {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0, 0.1};
  const double goal[] = {0.4, 0.7, -0.3, -1.2, 0.3, -0.6, 0.5, 0.3};
  for (int i = 0; i < kCanonicalDof; ++i) {
    q0[i] = start[i];
    qf[i] = goal[i];
  }
  const auto trajectory =
      MinJerkTrajectory::withVelocityLimit(q0, qf, 1.0, 3.0);
  const double duration = trajectory.duration() + 1.0;

  constexpr double kKp = 20.0, kKd = 2.0;
  ComputedTorque computed(chain, map, trajectory, kKp, kKd);
  // This test checks the computed-torque MATH in the ideal rigid sim,
  // so the hardware countermeasures (PD low-pass sized for the gear
  // resonance, friction integrator) are switched off; at Kp=20 the
  // filter pole would sit on the loop crossover and dominate the RMS.
  computed.setPdFilterTau(0.0);
  computed.setIntegral(0.0, 0.0);
  PdOnly pd(trajectory, kKp, kKd);

  const double rms_computed = trackingRms(trajectory, computed, duration);
  const double rms_pd = trackingRms(trajectory, pd, duration);

  INFO("computed-torque RMS " << rms_computed << " rad, PD-only RMS "
                              << rms_pd << " rad");
  CHECK(rms_computed < 0.02);        // tracks tightly
  CHECK(rms_computed < 0.5 * rms_pd);  // and clearly beats bare PD
}

TEST_CASE("shipped configuration tracks the x7_track excursion",
          "[tracking][sim]") {
  // The exact numbers the hardware runs (crane_x7_tuning.hpp) on the
  // exact excursion shape x7_track commands, over one continuous round
  // trip — the configuration that once went untested. Rigid-joint sim:
  // this pins the shipped loop's regression numbers; it cannot certify
  // stability against gear elasticity.
  namespace tuning = rtctrl::arm::tuning;
  using rtctrl::model::RoundTripTrajectory;
  ChainModel chain(kModelPath);
  JointMap map(chain);

  const double start[] = {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0, 0.1};
  ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) q0[i] = qf[i] = start[i];
  const double scale = 0.6;  // the hardware app's capped default
  qf[1] += 0.20 * scale;
  qf[3] -= 0.30 * scale;
  qf[5] += 0.25 * scale;
  const double min_T = 2.0 * std::sqrt(scale / 0.5);
  const RoundTripTrajectory trip(
      MinJerkTrajectory::withVelocityLimit(q0, qf, 0.3, min_T),
      MinJerkTrajectory::withVelocityLimit(qf, q0, 0.3, min_T));

  ComputedTorque ctl(chain, map, trip, tuning::kKp, tuning::kKd);
  ctl.setIntegral(tuning::kKi, tuning::kIntegralClampNm);
  ctl.setGainScales(tuning::kGainScale);
  ctl.setNominalDt(tuning::kNominalDt);
  double tau_max[kCanonicalDof] = {10.0, 10.0, 4.0, 4.0,
                                   4.0,  4.0,  4.0, 4.0};
  ctl.setTorqueLimits(tau_max);

  // track error and the commanded-torque continuity across the split
  struct Wrap : Controller {
    Wrap(ComputedTorque& inner, const Trajectory& trip, double split)
        : inner_(inner), trip_(trip), split_(split) {}
    void update(const JointState& state, JointCommand& cmd,
                double t) override {
      inner_.update(state, cmd, t);
      trip_.sample(t, q_d.get());
      for (int i = 0; i < kCanonicalDof; ++i) {
        const double e = q_d[i] - state.q[i];
        sum_sq += e * e;
        ++n;
        const double tau = zVecElemNC(cmd.tau.get(), i);
        if (have_prev) {
          const double d = std::fabs(tau - prev_tau[i]);
          if (!split_seen && t >= split_) {
            split_delta[i] = d;
          } else {
            max_delta[i] = std::max(max_delta[i], d);
          }
        }
        prev_tau[i] = tau;
      }
      if (!split_seen && t >= split_) split_seen = true;
      have_prev = true;
    }
    ComputedTorque& inner_;
    const Trajectory& trip_;
    double split_;
    ZVector q_d{kCanonicalDof};
    double prev_tau[kCanonicalDof] = {};
    double split_delta[kCanonicalDof] = {};
    double max_delta[kCanonicalDof] = {};
    double sum_sq = 0.0;
    long n = 0;
    bool have_prev = false;
    bool split_seen = false;
  } wrap(ctl, trip, trip.outDuration());

  SimArm::Options opt;
  opt.model_path = kModelPath;
  opt.initial_q8 = {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0, 0.1};
  SimArm sim(opt);
  sim.setMode(ControlMode::Current);
  sim.activate();
  REQUIRE(rtctrl::arm::run(sim, wrap, trip.duration()));

  const double rms = std::sqrt(wrap.sum_sq / wrap.n);
  INFO("shipped-config RMS " << rms << " rad");
  CHECK(rms < 0.02);  // regression pin for the shipped numbers

  // turnaround continuity: the torque step across the split must be of
  // the same order as ordinary cycle-to-cycle deltas — separate
  // per-leg controllers used to step multi-Nm discontinuities here
  for (const int j : {1, 3, 5}) {
    INFO("joint " << j << " split delta " << wrap.split_delta[j]
                  << " vs max ordinary delta " << wrap.max_delta[j]);
    CHECK(wrap.split_delta[j] < std::max(2.0 * wrap.max_delta[j], 0.05));
  }
}
