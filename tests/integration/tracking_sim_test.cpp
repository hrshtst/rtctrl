// M8 acceptance in sim, through the bridge: computed-torque tracking of
// a minimum-jerk trajectory in current mode, and its superiority over
// the same PD without the inverse-dynamics feedforward.
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/arm/computed_torque.hpp"
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
using rtctrl::model::ZVector;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

// PD in torque space WITHOUT the inverse-dynamics feedforward — the
// baseline computed torque must beat.
struct PdOnly : Controller {
  PdOnly(const MinJerkTrajectory& trajectory, double kp, double kd)
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
  const MinJerkTrajectory& trajectory_;
  double kp_, kd_;
  ZVector q_d{kCanonicalDof}, dq_d{kCanonicalDof};
};

// Runs `controller` over the trajectory on a fresh sim; returns the
// tracking RMS across all joints and samples.
double trackingRms(const MinJerkTrajectory& trajectory,
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
  PdOnly pd(trajectory, kKp, kKd);

  const double rms_computed = trackingRms(trajectory, computed, duration);
  const double rms_pd = trackingRms(trajectory, pd, duration);

  INFO("computed-torque RMS " << rms_computed << " rad, PD-only RMS "
                              << rms_pd << " rad");
  CHECK(rms_computed < 0.02);        // tracks tightly
  CHECK(rms_computed < 0.5 * rms_pd);  // and clearly beats bare PD
}
