// Pass-2 flexible-mode identification on the real arm: a stepped-sine
// torque probe on ONE joint superposed on the proven constant-anchor
// ComputedTorque, per docs/IDENTIFICATION_PLAN.md and the operator
// procedure in docs/IDENTIFICATION_PROTOCOL.md. Preview the identical
// pipeline with x7_ident_sim first.
//
// SAFETY: current mode. Power cutoff in reach, workspace clear. The
// posture is HAND-PLACED with the arm limp, held by activation +
// gravity comp, and verified still by the strict settle gate. One
// probe joint, one posture per invocation — the session duration
// budget (T_stop 177.5 s) is enforced at three levels: pre-activation
// schedule refusal, post-settle admission against the activation
// timestamp, and the independent SessionWatchdog whose expiry at
// T_quiesce = 179.5 s silences the bus (requestQuiesce) so the servo
// Bus Watchdogs stop the arm regardless of host state.
//
// Usage: x7_ident --joint N [--config path] [--port dev]
//                 [--freqs 4.1,4.25,...] [--amp cap] [--label name]
//                 [--log out.csv] [--anchor-ref file]
//   --joint       REQUIRED, exactly one probe joint (canonical index)
//   --freqs       dwell grid [Hz]; default = the survey grid
//   --amp         amplitude cap [Nm], default 0.15, hard cap 0.3
//   --anchor-ref  refuse to run unless the settled anchor matches the
//                 reference (JSON sidecar or 8 numbers) within
//                 +/-0.02 rad per joint
//   --log         full-loop ident telemetry CSV; a per-dwell JSON
//                 sidecar lands next to it as <log>.dwells.json

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ident_common.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "session_watchdog.hpp"
#include "track_common.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

namespace {

std::vector<x7::FreqSpec> defaultSurvey() {
  std::vector<x7::FreqSpec> specs;
  for (const double f : x7::surveyGridHz()) specs.push_back({f, 0.0});
  return specs;
}

void printDwellSummary(const x7::IdentRun& run) {
  for (const auto& r : run.results()) {
    std::printf(
        "  %6.2f Hz  amp %.3f Nm  %s%s%s hold %.2f s window %.2f s "
        "(%d periods)%s%s\n",
        r.spec.freq_hz, r.spec.amp_nm,
        r.completed ? "done" : (r.skipped ? "SKIPPED" : "INCOMPLETE"),
        r.low_confidence ? " low-confidence" : "",
        r.retried ? " retried" : "", r.hold_s, r.window_s,
        r.window_periods, r.note.empty() ? "" : "  — ",
        r.note.c_str());
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  int probe_joint = -1;
  std::vector<x7::FreqSpec> freqs = defaultSurvey();
  double a_cap = 0.15;
  std::string label, log_path, anchor_ref_path;
  for (int i = cli.argi; i < argc; ++i) {
    if (std::strcmp(argv[i], "--joint") == 0 && i + 1 < argc) {
      probe_joint = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--freqs") == 0 && i + 1 < argc) {
      if (!x7::parseFreqList(argv[++i], &freqs)) {
        std::fprintf(stderr,
                     "--freqs rejected (nothing runs on a truncated "
                     "schedule): every entry must be f or f@amp with "
                     "finite values, f in [%.1f, %.1f] Hz, amp > 0\n",
                     x7::kFreqMinHz, x7::kFreqMaxHz);
        return 1;
      }
    } else if (std::strcmp(argv[i], "--amp") == 0 && i + 1 < argc) {
      a_cap = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--label") == 0 && i + 1 < argc) {
      label = argv[++i];
    } else if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
      log_path = argv[++i];
    } else if (std::strcmp(argv[i], "--anchor-ref") == 0 && i + 1 < argc) {
      anchor_ref_path = argv[++i];
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 1;
    }
  }
  // Exactly ONE probe joint per activation session, validated BEFORE
  // activation — a multi-joint loop would reset the per-run schedule
  // while the 180 s deadline kept counting.
  if (probe_joint < 0 || probe_joint >= model::kCanonicalDof) {
    std::fprintf(stderr,
                 "--joint N (one canonical index in [0, %d]) is "
                 "required\n",
                 model::kCanonicalDof - 1);
    return 1;
  }
  if (freqs.empty()) {
    std::fprintf(stderr, "--freqs parsed to an empty grid\n");
    return 1;
  }
  a_cap = std::clamp(a_cap, x7::kAmpFloorNm, x7::kAmpCapHardNm);

  double anchor_ref[model::kCanonicalDof];
  if (!anchor_ref_path.empty() &&
      !x7::loadAnchorRef(anchor_ref_path, anchor_ref)) {
    std::fprintf(stderr, "cannot read an %d-joint anchor reference "
                         "from %s\n",
                 model::kCanonicalDof, anchor_ref_path.c_str());
    return 1;
  }

  try {
    auto session = x7::openSession(cli, /*operating_mode_override=*/0);
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);
    arm::RealArm robot(*session.arm);
    rtctrl::hw::CraneX7* hw = session.arm.get();

    // Health provider over the grouped feedback (slow signals; snapshot
    // coherence not required).
    x7::HealthProvider health = [hw](
        std::array<x7::JointHealth, model::kCanonicalDof>& h) {
      const auto fb = hw->lastFeedback();
      if (static_cast<int>(fb.size()) != model::kCanonicalDof) {
        return false;
      }
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        h[i] = {fb[i].temperature, fb[i].voltage};
      }
      return true;
    };

    if (!robot.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   hw->lastError().c_str());
      return 1;
    }
    const auto t_activate = std::chrono::steady_clock::now();
    // Independent session watchdog: expiry does exactly one thing —
    // requestQuiesce() + report. Bus silence trips the servo Bus
    // Watchdogs; the watchdog never calls deactivate() or touches the
    // port (neither is concurrency-safe against a blocked shutdown).
    x7::SessionWatchdog watchdog(
        std::chrono::milliseconds(
            static_cast<long>(1e3 * x7::kTQuiesceS)),
        [hw] {
          hw->requestQuiesce();
          std::fprintf(stderr,
                       "SESSION WATCHDOG: T_quiesce reached — bus "
                       "silenced; servo watchdogs will halt the arm\n");
        });
    const auto sinceActivation = [&t_activate] {
      return std::chrono::duration<double>(
                 std::chrono::steady_clock::now() - t_activate)
          .count();
    };
    // Every post-activation exit funnels through this. A deactivation
    // that could not CONFIRM zero + torque-off on every servo leaves
    // the failed servos' Bus Watchdogs armed — silence the bus so they
    // fire, and say so loudly; the run must then not report success.
    // The session watchdog disarms only after the shutdown attempt
    // (its deadline covers deactivation).
    const auto shutdown = [&robot, hw, &watchdog]() -> bool {
      const bool clean = robot.deactivate();
      if (!clean) {
        hw->requestQuiesce();
        std::fprintf(stderr,
                     "SHUTDOWN FAULT: deactivation incomplete — bus "
                     "silenced, armed servo watchdogs will halt any "
                     "still-torqued joint. Verify the arm is limp "
                     "before approaching; power-cycle before the next "
                     "run.\n");
      }
      watchdog.disarm();
      return clean;
    };

    arm::JointState start;
    if (!robot.readState(start)) {
      shutdown();
      return 1;
    }
    arm::JointCommand hold;
    hold.mode = arm::ControlMode::Current;
    chain.gravityTorque(map, start.q.get(), hold.tau.get());
    robot.writeCommand(hold);

    // Strict settle gate, no override: the probe must anchor on a
    // quiescent arm.
    x7::SettleController settle(chain, map, x7::tuning::kSettleKd);
    std::FILE* settle_log = nullptr;
    if (!log_path.empty()) {
      settle_log = std::fopen((log_path + ".settle").c_str(), "w");
    }
    const auto settled = x7::settleArm(robot, settle, 6.0, settle_log);
    if (settle_log != nullptr) std::fclose(settle_log);
    if (!settled.io_ok || !settled.quiescent) {
      std::fprintf(stderr, "settle phase %s — aborting\n",
                   settled.io_ok ? "did not reach quiescence" : "aborted");
      shutdown();
      return 1;
    }
    if (!robot.readState(start)) {
      shutdown();
      return 1;
    }

    // Start guard: refuse to probe from inside the soft-limit band.
    const auto& lo = hw->softLimitLo();
    const auto& hi = hw->softLimitHi();
    constexpr double kBuffer = 0.05;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double q = zVecElemNC(start.q.get(), i);
      if (q < lo[i] + kBuffer || q > hi[i] - kBuffer) {
        std::fprintf(stderr,
                     "joint %d (%s) is at %.3f rad, within its soft "
                     "position limit band [%.3f, %.3f] — re-place the "
                     "arm and rerun\n",
                     i, model::canonicalJoints()[i].urdf_joint, q, lo[i],
                     hi[i]);
        shutdown();
        return 1;
      }
    }

    // Thermal/voltage pre-run gate (doubles as the cooldown rule).
    {
      std::array<x7::JointHealth, model::kCanonicalDof> h;
      std::string why;
      if (!health(h) || !x7::preRunHealthOk(h, &why)) {
        std::fprintf(stderr, "pre-run health gate: %s — let the arm "
                             "cool / check supply, then rerun\n",
                     why.empty() ? "health read failed" : why.c_str());
        shutdown();
        return 1;
      }
    }

    // Anchor-reference gate: cross-invocation FRF data is only
    // combinable when the settled anchor matches within +/-0.02 rad.
    std::vector<double> anchor(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      anchor[i] = zVecElemNC(start.q.get(), i);
    }
    if (!anchor_ref_path.empty()) {
      double deltas[model::kCanonicalDof];
      if (!x7::anchorWithinTolerance(anchor.data(), anchor_ref,
                                     x7::kAnchorToleranceRad, deltas)) {
        std::fprintf(stderr, "anchor outside the +/-%.3f rad reference "
                             "tolerance — re-place the arm:\n",
                     x7::kAnchorToleranceRad);
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          std::fprintf(stderr, "  joint %d: settled %+.4f ref %+.4f "
                               "delta %+.4f%s\n",
                       i, anchor[i], anchor_ref[i], deltas[i],
                       std::fabs(deltas[i]) > x7::kAnchorToleranceRad
                           ? "  <-- out"
                           : "");
        }
        shutdown();
        return 1;
      }
    }

    // Schedule with the amplitude rule at THIS anchor's inertia.
    model::ZVector q_anchor(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      zVecElemNC(q_anchor.get(), i) = anchor[i];
    }
    const double j_hat = x7::diagInertia(chain, map, q_anchor,
                                         probe_joint);
    auto dwells = x7::buildScheduleFromSpecs(freqs, j_hat, a_cap);
    // Pre-activation-style refusal (checked before settle would be
    // ideal, but the amplitudes depend on the settled anchor; the
    // authoritative admission below covers the difference).
    if (!x7::scheduleFitsBudget(dwells)) {
      std::fprintf(stderr,
                   "schedule worst case %.1f s + %.0f s setup exceeds "
                   "T_stop %.1f s — split it (--freqs)\n",
                   x7::baseWorstSeconds(dwells), x7::kSetupAllowanceS,
                   x7::kTStopS);
      shutdown();
      return 1;
    }
    std::printf("probe joint %d (J_hat %.4f kg m^2), %zu dwells, lead-in "
                "%.1f s, worst case %.1f s:\n",
                probe_joint, j_hat, dwells.size(),
                x7::leadInSeconds(dwells), x7::baseWorstSeconds(dwells));
    for (const auto& d : dwells) {
      std::printf("  %6.2f Hz  amp %.3f Nm%s\n", d.freq_hz, d.amp_nm,
                  d.window_mult > 1 ? "  (x4 window requested)" : "");
    }

    // POST-SETTLE ADMISSION (authoritative): the deadline runs from
    // activation, and settling/gates have been spending it.
    const double setup_elapsed = sinceActivation();
    if (setup_elapsed + x7::baseWorstSeconds(dwells) > x7::kTStopS) {
      std::fprintf(stderr,
                   "setup consumed %.1f s: the worst-case schedule no "
                   "longer fits T_stop — deactivating\n",
                   setup_elapsed);
      shutdown();
      return 1;
    }

    std::FILE* log = nullptr;
    if (!log_path.empty()) {
      std::string meta = "label=" + label + " probe_joint=" +
                         std::to_string(probe_joint) + " anchor=[";
      char num[32];
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::snprintf(num, sizeof num, "%s%.5f", i ? "," : "",
                      anchor[i]);
        meta += num;
      }
      meta += "] dwells=[";
      for (std::size_t k = 0; k < dwells.size(); ++k) {
        std::snprintf(num, sizeof num, "%s%.3f:%.3f", k ? "," : "",
                      dwells[k].freq_hz, dwells[k].amp_nm);
        meta += num;
      }
      std::snprintf(num, sizeof num,
                    "] kp=%.1f kd=%.2f ki=%.1f", x7::tuning::kKp,
                    x7::tuning::kKd, x7::tuning::kKi);
      meta += num;
      log = x7::openIdentCsvLog(log_path, meta);
      if (!log) {
        std::fprintf(stderr, "cannot open log file %s\n",
                     log_path.c_str());
        shutdown();
        return 1;
      }
    }

    x7::IdentRun::Options opt;
    opt.probe_joint = probe_joint;
    opt.dwells = dwells;
    opt.anchor = anchor;
    opt.setup_offset_s = setup_elapsed;
    opt.health = health;
    // exact per-joint torque limits, mirroring writeCurrents
    opt.tau_max.resize(model::kCanonicalDof);
    {
      const auto& joints = session.config.joints;
      const auto& servo_amps = hw->servoCurrentLimitAmps();
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double kt =
            rtctrl::dxl::torqueConstant(joints[i].model_number);
        opt.tau_max[i] = std::max(
            0.0, std::min(joints[i].effort_limit, kt * servo_amps[i]) -
                     joints[i].current_limit_margin * kt);
      }
    }

    x7::IdentRun ident(chain, map, opt, log);
    // duration = the remaining budget; the run normally ends earlier by
    // its own Done veto
    const bool ran = arm::run(robot, ident,
                              x7::kTStopS - setup_elapsed, &ident);
    (void)ran;  // every path is judged by the run's own outcome below
    // Safe transition FIRST (shutdown() escalates to bus silence if it
    // cannot confirm every servo stopped), then report.
    const bool clean_shutdown = shutdown();
    const auto stats = hw->cycleStats();
    if (log) std::fclose(log);
    if (!log_path.empty()) {
      x7::writeDwellJson(log_path + ".dwells.json", opt, ident);
    }

    printDwellSummary(ident);
    std::printf("cycles %llu, overruns %llu, read failures %llu, write "
                "failures %llu\n",
                static_cast<unsigned long long>(stats.cycles),
                static_cast<unsigned long long>(stats.overruns),
                static_cast<unsigned long long>(stats.read_failures),
                static_cast<unsigned long long>(stats.write_failures));
    if (!clean_shutdown) {
      // never report an ordinary result over an unconfirmed shutdown
      std::printf("SHUTDOWN FAULT (run outcome %d: %s)\n",
                  static_cast<int>(ident.outcome()),
                  ident.faultReason().c_str());
      return 1;
    }
    switch (ident.outcome()) {
      case x7::IdentRun::Outcome::Completed:
        std::printf("done\n");
        return 0;
      case x7::IdentRun::Outcome::DeadlineStop:
        std::printf("GRACEFUL STOP at T_stop — remaining dwells not "
                    "run\n");
        return 0;
      case x7::IdentRun::Outcome::SoftAbort:
        std::printf("ABORTED (soft-event recurrence): %s\n",
                    ident.faultReason().c_str());
        return 1;
      default:
        std::printf("ABORTED: %s\n", ident.faultReason().empty()
                                         ? "runner/policy fault"
                                         : ident.faultReason().c_str());
        return 1;
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
