// M8 hardware phase: computed-torque trajectory tracking on the real
// arm — tau = ID(q_d, dq_d, ddq_d) + Kp e + Kd de in current mode,
// a small minimum-jerk excursion from the present pose and back, at
// reduced speed with the sim-verified gains.
//
// SAFETY: current mode. Power cutoff in reach, workspace clear. The
// controller and gains passed the sim acceptance first
// (tracking_sim_test: RMS 0.0050 rad, 3.1x tighter than bare PD).
//
// Usage: x7_track [--config path] [--port dev] [scale]
//   scale in (0,1] shrinks the excursion (default 1.0 ≈ ±0.3 rad max)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

namespace {

// Wraps ComputedTorque and accumulates the tracking error.
struct TrackingRun : arm::Controller {
  TrackingRun(model::ChainModel& chain, const model::JointMap& map,
              const model::MinJerkTrajectory& trajectory, double kp,
              double kd)
      : inner(chain, map, trajectory, kp, kd), trajectory_(trajectory) {}

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    inner.update(state, cmd, t);
    trajectory_.sample(t, q_d.get());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d[i] - zVecElemNC(state.q.get(), i);
      sum_sq += e * e;
      ++samples;
    }
  }
  double rms() const { return samples ? std::sqrt(sum_sq / samples) : 0.0; }

  arm::ComputedTorque inner;
  const model::MinJerkTrajectory& trajectory_;
  model::ZVector q_d{model::kCanonicalDof};
  double sum_sq = 0.0;
  long samples = 0;
};

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  double scale = cli.argi < argc ? std::atof(argv[cli.argi]) : 1.0;
  scale = std::clamp(scale, 0.05, 1.0);

  try {
    auto session = x7::openSession(cli, /*operating_mode_override=*/0);
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);
    arm::RealArm robot(*session.arm);

    if (!robot.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   session.arm->lastError().c_str());
      return 1;
    }
    arm::JointState start;
    if (!robot.readState(start)) return 1;

    model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
    zVecCopyNC(start.q.get(), q0.get());
    zVecCopyNC(start.q.get(), qf.get());
    // gentle excursion on tilt / elbow / wrist pitch
    qf[1] += 0.20 * scale;
    qf[3] -= 0.30 * scale;
    qf[5] += 0.25 * scale;

    constexpr double kVel = 0.3;  // rad/s — reduced speed
    constexpr double kKp = 20.0, kKd = 2.0;  // sim-verified
    const auto out = model::MinJerkTrajectory::withVelocityLimit(
        q0, qf, kVel, 2.0);
    const auto back = model::MinJerkTrajectory::withVelocityLimit(
        qf, q0, kVel, 2.0);

    std::printf("computed-torque tracking: %.1f s out, %.1f s back "
                "(scale %.2f)\n", out.duration(), back.duration(), scale);

    TrackingRun leg1(chain, map, out, kKp, kKd);
    bool ok = arm::run(robot, leg1, out.duration());
    std::printf("out  RMS: %.4f rad\n", leg1.rms());

    if (ok) {
      TrackingRun leg2(chain, map, back, kKp, kKd);
      ok = arm::run(robot, leg2, back.duration());
      std::printf("back RMS: %.4f rad\n", leg2.rms());
    }

    robot.deactivate();
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
