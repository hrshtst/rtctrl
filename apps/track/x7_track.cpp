// M8 hardware phase: computed-torque trajectory tracking on the real
// arm in current mode, with the hardware-hardened law
//   tau = ID(q_d, dq_d, ddq_d) + LP(s_i (Kp e + Kd de_host))
//         + clamp(Ki int e)
// — a small minimum-jerk excursion from the present pose and back, at
// reduced speed. Preview the identical run with x7_track_sim first.
//
// SAFETY: current mode. Power cutoff in reach, workspace clear.
//
// PARKED (2026-07-28): do NOT run on hardware until the
// reviewer-approved settle-phase fix lands. A scale-0.5 session never
// left the settle phase — the pan anti-damped at 4.33 Hz from a
// compact resting posture (operator power cut; telemetry archived as
// track9.csv / track9.csv.settle). The failure precedes the scale
// argument: NO scale is safe. See docs/hardware/bringup.md.
//
// Stability, learned the hard way (nine 2026-07-21 hardware runs,
// logged as track*.csv):
// the oscillation is a ~13 Hz gear-train structural resonance (output
// encoder, motor-side torque, elastic gearing between — measured in
// the run logs), which the host PD pumps because at 13 Hz the 100 Hz
// bus loop's historically measured ~2-cycle command-to-current delay put
// every feedback term >90 degrees out of phase. The feedback-synchronized
// scheduler now removes one host-side waiting period, but that change has not
// been validated on hardware and does not unpark this app. Countermeasures,
// all of which this app relies on:
//   * ComputedTorque low-passes the PD correction (~3 Hz, 4x gain cut
//     at the resonance) — rigid-joint sims cannot show this mode, so
//     do not trust them on it;
//   * damping uses a host-side velocity estimate — the servo's own
//     PresentVelocity lags ~50 ms with ~2x attenuation;
//   * a DAMPED settle phase that runs until the arm is quiescent —
//     current-mode torque-on starts from free fall, pure gravity comp
//     never damps the resulting swing, and tracking must not anchor on
//     a moving arm (track3.csv entered tracking at 2.6 rad/s on the
//     twist);
//   * moderate stiffness — the filtered loop's crossover must stay
//     below the filter pole, capping Kp around 6; the ~0.15 rad
//     friction sag that such a Kp would leave is absorbed by the
//     controller's slow clamped integrator instead;
//   * per-joint PD scaling (track_common.hpp) — at uniform gains the
//     low-inertia wrist joints limit-cycle through their backlash
//     (~5 Hz in the track3.csv log);
//   * hw-side: feedback positions wrap to the principal angle — the
//     servo's multi-turn readout after hand-repositioning (track3.csv:
//     twist = +6.54 rad) otherwise poisons the soft position limits,
//     whose gate then silently blocks a whole torque direction;
//   * excursion scale capped at 0.6 and duration grown with
//     sqrt(amplitude) — larger excursions extend the arm into
//     configurations whose ~4-5 Hz structural mode this loop actively
//     pumps (track7/track8.csv: coherent whole-arm oscillation,
//     growing even at matched trajectory rates).
//
// Usage: x7_track [--config path] [--port dev]
//                 [--kp v] [--kd v] [--ki v] [--log out.csv] [scale]
//   scale in (0, 0.6] shrinks the excursion (default 0.6 ≈ ±0.18 rad
//     max); larger values are capped — the structural-mode stability
//     envelope
//   --log writes the full loop telemetry per cycle (feedback stamp and
//     sequence, submission receipts, applied-command record with
//     first-vs-latest application, controller ff/pd/i components,
//     controller and hardware saturation/gate flags, measured torque);
//     the CSV's '#' header line documents the event semantics

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "track/track_common.hpp"
#include "common/x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  double scale = 0.6;
  bool scale_given = false;
  // Hardware defaults — see header for how they were chosen.
  double kp = x7::tuning::kKp, kd = x7::tuning::kKd, ki = x7::tuning::kKi;
  std::string log_path;
  // STRICT: an unknown flag used to fall through atof() into the
  // positional scale — a typo silently ran the arm at minimum scale
  // (review finding).
  const auto& rest = cli.rest;
  for (std::size_t i = 0; i < rest.size(); ++i) {
    double* flag_dst = nullptr;
    if (std::strcmp(rest[i], "--kp") == 0) {
      flag_dst = &kp;
    } else if (std::strcmp(rest[i], "--kd") == 0) {
      flag_dst = &kd;
    } else if (std::strcmp(rest[i], "--ki") == 0) {
      flag_dst = &ki;
    } else if (std::strcmp(rest[i], "--log") == 0) {
      // A flag-looking next token is a MISSING value, not a filename
      // (same rule as parseCli and x7_float; "--log --bogus" must not
      // create ./--bogus and proceed).
      if (i + 1 >= rest.size() ||
          std::strncmp(rest[i + 1], "--", 2) == 0) {
        std::fprintf(stderr, "--log requires a value\n");
        return 1;
      }
      log_path = rest[++i];
      continue;
    } else if (rest[i][0] == '-' && rest[i][1] == '-') {
      std::fprintf(stderr, "unknown argument: %s\n", rest[i]);
      return 1;
    } else {
      if (scale_given) {
        std::fprintf(stderr, "unexpected argument: %s\n", rest[i]);
        return 1;
      }
      if (!x7::parseStrictDouble(rest[i], &scale)) {
        std::fprintf(stderr, "invalid scale: %s\n", rest[i]);
        return 1;
      }
      scale_given = true;
      continue;
    }
    if (i + 1 >= rest.size()) {
      std::fprintf(stderr, "%s requires a value\n", rest[i]);
      return 1;
    }
    if (!x7::parseStrictDouble(rest[++i], flag_dst)) {
      std::fprintf(stderr, "%s: invalid value %s\n", rest[i - 1],
                   rest[i]);
      return 1;
    }
  }
  // Beyond ~0.6 the excursion extends the arm into configurations
  // where its ~4-5 Hz structural mode (shoulder gear compliance vs arm
  // inertia) goes actively unstable under this 100 Hz loop — verified
  // twice at scale 1.0 on 2026-07-21 (runs 7-8, both ended in manual
  // power cuts). The cap is FINAL (owner closure 2026-07-28): the
  // identification a compensator design required stopped at its first
  // gate — joint 1 at P1 stiction-locked below one encoder count up
  // to the probe's 0.3 Nm hard cap; other routes untested, closed by
  // decision (docs/records/history.md (identification), Closure). Use position
  // mode for larger fast motions.
  if (scale > 0.6) {
    std::printf("scale %.2f capped to 0.60 — the final supported "
                "boundary (see x7_track.cpp header)\n", scale);
  }
  scale = std::clamp(scale, 0.05, 0.6);
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
    x7::ShutdownGuard shutdown{*session.arm};
    arm::JointState start;
    if (!robot.readState(start)) {
      std::fprintf(stderr, "initial state read failed — aborting\n");
      shutdown.run();
      return 1;
    }

    // Current-mode activation leaves goal currents at zero — the arm is
    // in free fall until the first controller command. Hold it with
    // gravity compensation immediately, then run the DAMPED settle
    // until the arm is quiescent: tracking must not anchor on (or fight)
    // a swinging arm.
    arm::JointCommand hold;
    hold.mode = arm::ControlMode::Current;
    chain.gravityTorque(map, start.q.get(), hold.tau.get());
    robot.writeCommand(hold);
    x7::SettleController settle(chain, map, x7::tuning::kSettleKd);
    // the settle phase gets its own small log next to the tracking one:
    // a gate refusal must leave evidence
    std::FILE* settle_log = nullptr;
    if (!log_path.empty()) {
      settle_log = std::fopen((log_path + ".settle").c_str(), "w");
    }
    const auto settled = x7::settleArm(robot, settle, 6.0, settle_log);
    if (settle_log != nullptr) std::fclose(settle_log);
    if (!settled.io_ok || !settled.quiescent) {
      // No override: a settle timeout means real residual motion or a
      // broken quiescence metric — both grounds to stop. Tracking must
      // not anchor on (or fight) a moving arm.
      std::fprintf(stderr, "settle phase %s — aborting\n",
                   settled.io_ok ? "did not reach quiescence"
                                 : "aborted");
      shutdown.run();
      return 1;
    }
    if (!robot.readState(start)) {
      std::fprintf(stderr, "post-settle state read failed — aborting\n");
      shutdown.run();
      return 1;
    }

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
        shutdown.run();
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
    // ONE C2-continuous round trip driven by ONE controller: separate
    // per-leg controllers reset the estimator/filter/integrator at the
    // turnaround and stepped multi-Nm discontinuities into the arm
    // (track8.csv).
    const model::RoundTripTrajectory trip(
        model::MinJerkTrajectory::withVelocityLimit(q0, qf, kVel, min_T),
        model::MinJerkTrajectory::withVelocityLimit(qf, q0, kVel, min_T));

    std::printf("computed-torque tracking: %.1f s out, %.1f s back "
                "(scale %.2f, Kp %.1f, Kd %.2f, Ki %.1f)\n",
                trip.outDuration(), trip.duration() - trip.outDuration(),
                scale, kp, kd, ki);

    x7::TrackingRun tracking(chain, map, trip, kp, kd,
                             trip.outDuration(), log);
    tracking.inner.setIntegral(ki, x7::tuning::kIntegralClampNm);
    tracking.inner.setGainScales(x7::kGainScale);
    tracking.inner.setNominalDt(x7::tuning::kNominalDt);
    // exact per-joint torque limits, mirroring writeCurrents: enables
    // the direction-aware anti-windup and controller-side clamping
    {
      const auto& joints = session.config.joints;
      const auto& servo_amps = session.arm->servoCurrentLimitAmps();
      double tau_max[model::kCanonicalDof];
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double kt = rtctrl::dxl::torqueConstant(
            joints[i].model_number);
        tau_max[i] = std::max(
            0.0, std::min(joints[i].effort_limit, kt * servo_amps[i]) -
                     joints[i].current_limit_margin * kt);
      }
      tracking.inner.setTorqueLimits(tau_max);
    }

    const bool ok = arm::run(robot, tracking, trip.duration(), &tracking);
    // Safe transition FIRST: on an abort (observer veto included) the
    // background thread is still healthily retransmitting the last
    // torque command — reporting must not delay going limp.
    const bool clean = shutdown.run();
    const auto stats = session.arm->cycleStats();
    tracking.report();
    if (log) std::fclose(log);
    std::printf("cycles %llu, overruns %llu, skipped periods %llu, "
                "max cycle %.3f ms, max lateness %.3f ms, stale submissions "
                "%llu, command-window misses %llu, max feedback age %.3f ms, "
                "read failures %llu, write failures %llu\n",
                static_cast<unsigned long long>(stats.cycles),
                static_cast<unsigned long long>(stats.overruns),
                static_cast<unsigned long long>(stats.skipped_periods),
                1e3 * stats.max_cycle_time_s, 1e3 * stats.max_lateness_s,
                static_cast<unsigned long long>(stats.stale_submissions),
                static_cast<unsigned long long>(
                    stats.controller_deadline_misses),
                1e3 * stats.max_feedback_age_at_submission_s,
                static_cast<unsigned long long>(stats.read_failures),
                static_cast<unsigned long long>(stats.write_failures));
    if (!clean) {
      std::printf("SHUTDOWN FAULT (run %s)\n", ok ? "done" : "ABORTED");
      return 1;
    }
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    if (log) std::fclose(log);
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
