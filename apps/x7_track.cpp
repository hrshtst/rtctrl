// M8 hardware phase: computed-torque trajectory tracking on the real
// arm — tau = ID(q_d, dq_d, ddq_d) + Kp e + Kd de in current mode,
// a small minimum-jerk excursion from the present pose and back, at
// reduced speed. Preview the identical run with x7_track_sim first.
//
// SAFETY: current mode. Power cutoff in reach, workspace clear.
//
// Stability, learned the hard way (three 2026-07-21 hardware runs):
// the oscillation is a ~13 Hz gear-train structural resonance (output
// encoder, motor-side torque, elastic gearing between — measured in
// the run logs), which the host PD pumps because at 13 Hz the 100 Hz
// bus loop's ~2-cycle delay puts every feedback term >90 degrees out
// of phase. Countermeasures, all of which this app relies on:
//   * ComputedTorque low-passes the PD correction (~3 Hz, 4x gain cut
//     at the resonance) — rigid-joint sims cannot show this mode, so
//     do not trust them on it;
//   * damping uses a host-side velocity estimate — the servo's own
//     PresentVelocity lags ~50 ms with ~2x attenuation;
//   * a DAMPED settle phase that runs until the arm is quiescent —
//     current-mode torque-on starts from free fall, pure gravity comp
//     never damps the resulting swing, and tracking must not anchor on
//     a moving arm (run 4 entered tracking at 2.6 rad/s on the twist);
//   * moderate stiffness — the filtered loop's crossover must stay
//     below the filter pole, capping Kp around 6; the ~0.15 rad
//     friction sag that such a Kp would leave is absorbed by the
//     controller's slow clamped integrator instead;
//   * per-joint PD scaling (track_common.hpp) — at uniform gains the
//     low-inertia wrist joints limit-cycle through their backlash
//     (~5 Hz in run 4's log);
//   * hw-side: feedback positions wrap to the principal angle — the
//     servo's multi-turn readout after hand-repositioning (run 4:
//     twist = +6.54 rad) otherwise poisons the soft position limits,
//     whose gate then silently blocks a whole torque direction.
//
// Usage: x7_track [--config path] [--port dev]
//                 [--kp v] [--kd v] [--ki v] [--log out.csv] [scale]
//   scale in (0,1] shrinks the excursion (default 1.0 ≈ ±0.3 rad max)
//   --log writes t, q_d, q, dq_d, dq, tau per joint each cycle

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "track_common.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  double scale = 1.0;
  // Hardware defaults — see header for how they were chosen.
  double kp = 6.0, kd = 1.0, ki = 6.0;
  std::string log_path;
  for (int i = cli.argi; i < argc; ++i) {
    if (std::strcmp(argv[i], "--kp") == 0 && i + 1 < argc) {
      kp = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--kd") == 0 && i + 1 < argc) {
      kd = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--ki") == 0 && i + 1 < argc) {
      ki = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
      log_path = argv[++i];
    } else {
      scale = std::atof(argv[i]);
    }
  }
  scale = std::clamp(scale, 0.05, 1.0);
  kp = std::clamp(kp, 0.0, 50.0);
  kd = std::clamp(kd, 0.0, 5.0);
  ki = std::clamp(ki, 0.0, 20.0);

  std::FILE* log = nullptr;
  if (!log_path.empty()) {
    log = x7::openCsvLog(log_path);
    if (!log) {
      std::fprintf(stderr, "cannot open log file %s\n", log_path.c_str());
      return 1;
    }
  }

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

    // Current-mode activation leaves goal currents at zero — the arm is
    // in free fall until the first controller command. Hold it with
    // gravity compensation immediately, then run the DAMPED settle
    // until the arm is quiescent: tracking must not anchor on (or fight)
    // a swinging arm.
    arm::JointCommand hold;
    hold.mode = arm::ControlMode::Current;
    chain.gravityTorque(map, start.q.get(), hold.tau.get());
    robot.writeCommand(hold);
    x7::SettleController settle(chain, map, 0.8);
    if (!x7::settleArm(robot, settle, 6.0)) {
      std::fprintf(stderr, "settle phase aborted\n");
      robot.deactivate();
      return 1;
    }
    if (!robot.readState(start)) return 1;

    // Refuse to control a joint parked inside its soft-limit margin:
    // the current gate blocks one whole torque direction there, and the
    // loop bounces against that wall instead of tracking.
    const auto& lo = session.arm->softLimitLo();
    const auto& hi = session.arm->softLimitHi();
    constexpr double kBuffer = 0.05;  // [rad] beyond the margin
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double q = zVecElemNC(start.q.get(), i);
      if (q < lo[i] + kBuffer || q > hi[i] - kBuffer) {
        std::fprintf(stderr,
                     "joint %d (%s) is at %.3f rad, within its soft "
                     "position limit band [%.3f, %.3f] — move it toward "
                     "mid-range and rerun\n",
                     i, model::canonicalJoints()[i].urdf_joint, q,
                     lo[i], hi[i]);
        robot.deactivate();
        return 1;
      }
    }

    model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
    zVecCopyNC(start.q.get(), q0.get());
    zVecCopyNC(start.q.get(), qf.get());
    // gentle excursion on tilt / elbow / wrist pitch, kept clear of the
    // soft-limit band
    qf[1] += 0.20 * scale;
    qf[3] -= 0.30 * scale;
    qf[5] += 0.25 * scale;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double clamped =
          std::clamp(qf[i], lo[i] + kBuffer, hi[i] - kBuffer);
      if (clamped != qf[i]) {
        std::printf("excursion on joint %d clamped to %.3f rad (soft "
                    "limit)\n", i, clamped);
        qf[i] = clamped;
      }
    }

    constexpr double kVel = 0.3;  // rad/s — reduced speed
    // Hold the peak acceleration at the half-scale-proven level: with
    // the duration pinned at 2 s, full scale ran at DOUBLE the rates
    // and pumped the arm's ~4 Hz structural mode (2026-07-21 run 7 —
    // every proximal joint oscillating coherently). Min-jerk peak
    // accel ~ A/T^2, so the duration grows with sqrt(amplitude).
    const double min_T = 2.0 * std::sqrt(std::max(scale, 0.5) / 0.5);
    const auto out = model::MinJerkTrajectory::withVelocityLimit(
        q0, qf, kVel, min_T);
    const auto back = model::MinJerkTrajectory::withVelocityLimit(
        qf, q0, kVel, min_T);

    std::printf("computed-torque tracking: %.1f s out, %.1f s back "
                "(scale %.2f, Kp %.1f, Kd %.2f, Ki %.1f)\n",
                out.duration(), back.duration(), scale, kp, kd, ki);

    x7::TrackingRun leg1(chain, map, out, kp, kd, 0, log);
    leg1.inner.setIntegral(ki, 1.5);
    leg1.inner.setGainScales(x7::kGainScale);
    bool ok = arm::run(robot, leg1, out.duration());
    leg1.report();

    if (ok) {
      x7::TrackingRun leg2(chain, map, back, kp, kd, 1, log);
      leg2.inner.setIntegral(ki, 1.5);
      leg2.inner.setGainScales(x7::kGainScale);
      ok = arm::run(robot, leg2, back.duration());
      leg2.report();
    }

    const auto stats = session.arm->cycleStats();
    robot.deactivate();
    if (log) std::fclose(log);
    std::printf("cycles %llu, overruns %llu, read failures %llu\n",
                static_cast<unsigned long long>(stats.cycles),
                static_cast<unsigned long long>(stats.overruns),
                static_cast<unsigned long long>(stats.read_failures));
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    if (log) std::fclose(log);
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
