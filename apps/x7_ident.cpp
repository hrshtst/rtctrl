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
//                 [--pose-first [--vel v]]
//   --joint       REQUIRED, exactly one probe joint (canonical index)
//   --freqs       dwell grid [Hz]; default = the survey grid
//   --amp         amplitude cap [Nm], default 0.15, hard cap 0.3
//   --anchor-ref  refuse to run unless the settled anchor matches the
//                 reference (JSON sidecar or 8 numbers) within
//                 +/-0.02 rad per joint
//   --pose-first  integrated placement (reviewer-approved): move to
//                 the --anchor-ref posture in POSITION mode with
//                 measured-posture convergence, switch to current mode
//                 in-process with a clamped gravity-current preload,
//                 then capture the residual onto the canonical anchor
//                 under the restoring controller before probing. No
//                 hands needed; the quiescence and +/-0.02 gates still
//                 decide.
//   --vel         placement speed limit [rad/s], default 0.25, max 0.5
//   --hold        STABILIZATION DIAGNOSTIC (requires --pose-first, no
//                 --freqs): capture, then an unforced anchor hold for
//                 the given seconds under CONTINUOUS gates (±0.02 rad
//                 and the 0.05 rad/s metric at every sample after a
//                 1 s grace); reports per-joint maxima. No probing.
//   --scale0      diagnostic-only joint-0 PD scale override in
//                 [0.05, 1.0) — the campaign runs ONE fixed, recorded
//                 tuning; this knob exists only to find it
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
#include "pose_common.hpp"
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

// Strict full-token numeric parsing: atof() turned "--hold garbage"
// into zero — which silently selected the normal survey instead of
// refusing (review finding).
bool parseStrictDouble(const char* text, double* out) {
  char* end = nullptr;
  const double v = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(v)) return false;
  *out = v;
  return true;
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
  bool freqs_given = false;
  double a_cap = 0.15;
  bool pose_first = false;
  double pose_vel = 0.25;
  double hold_s = 0.0;
  double scale0 = 0.0;
  bool hold_given = false;
  bool scale0_given = false;
  bool freeze_i = false;
  std::string label, log_path, anchor_ref_path;
  for (int i = cli.argi; i < argc; ++i) {
    if (std::strcmp(argv[i], "--joint") == 0 && i + 1 < argc) {
      probe_joint = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--freqs") == 0 && i + 1 < argc) {
      freqs_given = true;
      if (!x7::parseFreqList(argv[++i], &freqs)) {
        std::fprintf(stderr,
                     "--freqs rejected (nothing runs on a truncated "
                     "schedule): every entry must be f or f@amp with "
                     "finite values, f in [%.1f, %.1f] Hz, amp > 0\n",
                     x7::kFreqMinHz, x7::kFreqMaxHz);
        return 1;
      }
    } else if (std::strcmp(argv[i], "--amp") == 0 && i + 1 < argc) {
      if (!x7::parseAmpCap(argv[++i], &a_cap)) {
        std::fprintf(stderr,
                     "--amp rejected: one finite value in [%.2f, %.2f] "
                     "Nm required\n",
                     x7::kAmpFloorNm, x7::kAmpCapHardNm);
        return 1;
      }
    } else if (std::strcmp(argv[i], "--label") == 0 && i + 1 < argc) {
      label = argv[++i];
    } else if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
      log_path = argv[++i];
    } else if (std::strcmp(argv[i], "--anchor-ref") == 0 && i + 1 < argc) {
      anchor_ref_path = argv[++i];
    } else if (std::strcmp(argv[i], "--pose-first") == 0) {
      pose_first = true;
    } else if (std::strcmp(argv[i], "--vel") == 0 && i + 1 < argc) {
      if (!parseStrictDouble(argv[++i], &pose_vel)) {
        std::fprintf(stderr, "--vel rejected: one finite value "
                             "required\n");
        return 1;
      }
    } else if (std::strcmp(argv[i], "--hold") == 0 && i + 1 < argc) {
      hold_given = true;
      if (!parseStrictDouble(argv[++i], &hold_s)) {
        std::fprintf(stderr, "--hold rejected: one finite duration in "
                             "[5, 120] s required\n");
        return 1;
      }
    } else if (std::strcmp(argv[i], "--scale0") == 0 && i + 1 < argc) {
      scale0_given = true;
      if (!parseStrictDouble(argv[++i], &scale0)) {
        std::fprintf(stderr, "--scale0 rejected: one finite value in "
                             "[0.05, 1.0) required\n");
        return 1;
      }
    } else if (std::strcmp(argv[i], "--freeze-i") == 0) {
      freeze_i = true;
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

  double anchor_ref[model::kCanonicalDof];
  if (!anchor_ref_path.empty() &&
      !x7::loadAnchorRef(anchor_ref_path, anchor_ref)) {
    std::fprintf(stderr, "cannot read an %d-joint anchor reference "
                         "from %s\n",
                 model::kCanonicalDof, anchor_ref_path.c_str());
    return 1;
  }
  if (pose_first && anchor_ref_path.empty()) {
    std::fprintf(stderr,
                 "--pose-first needs --anchor-ref: the placement target "
                 "IS the canonical anchor\n");
    return 1;
  }
  if (!std::isfinite(pose_vel)) pose_vel = 0.25;
  pose_vel = std::clamp(pose_vel, 0.05, 0.5);
  // Unforced-hold diagnostic (the stabilization procedure): strict
  // validation, --scale0 is REQUIRED with --hold (the zero sentinel
  // would silently run the known-failing effective scale 1.0 — review
  // finding), and it exists ONLY here so arbitrary survey tuning
  // cannot slip into the campaign (the FRFs are controller-specific).
  const bool hold_mode = hold_given;
  if (hold_mode) {
    if (!pose_first) {
      std::fprintf(stderr, "--hold requires --pose-first (the "
                           "diagnostic runs capture-then-hold)\n");
      return 1;
    }
    if (freqs_given) {
      std::fprintf(stderr, "--hold takes no --freqs: the diagnostic "
                           "never probes\n");
      return 1;
    }
    if (hold_s < 5.0 || hold_s > 120.0) {
      std::fprintf(stderr, "--hold rejected: one finite duration in "
                           "[5, 120] s required\n");
      return 1;
    }
    if (!scale0_given) {
      std::fprintf(stderr,
                   "--hold requires an explicit --scale0: the "
                   "diagnostic exists to test a REDUCED joint-0 scale "
                   "(1.0 is the already-characterized failing case)\n");
      return 1;
    }
    if (scale0 >= 1.0) {
      std::fprintf(stderr,
                   "--scale0 %.3f rejected: 1.0 is the "
                   "already-characterized failing case (2026-07-27 "
                   "run) — pick a value in [0.05, 1.0)\n",
                   scale0);
      return 1;
    }
    if (scale0 < 0.05) {
      std::fprintf(stderr, "--scale0 rejected: one finite value in "
                           "[0.05, 1.0) required\n");
      return 1;
    }
  } else if (scale0_given) {
    std::fprintf(stderr, "--scale0 is diagnostic-only: it requires "
                         "--hold\n");
    return 1;
  }
  if (freeze_i && !hold_mode) {
    std::fprintf(stderr, "--freeze-i is diagnostic-only (experimental): "
                         "it requires --hold\n");
    return 1;
  }

  try {
    auto session =
        x7::openSession(cli, /*operating_mode_override=*/pose_first ? 3
                                                                    : 0);
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);

    if (pose_first) {
      // EARLY budget refusal, before ANY motion (review finding: the
      // old check ran after placement had already moved the robot).
      // The anchor is the reference vector, so the whole schedule is
      // known up front.
      {
        double worst = x7::kCaptureWorstS +
                       x7::kPoseFirstSetupAllowanceS + hold_s;
        if (!hold_mode) {
          model::ZVector q_ref(model::kCanonicalDof);
          for (int i = 0; i < model::kCanonicalDof; ++i) {
            zVecElemNC(q_ref.get(), i) = anchor_ref[i];
          }
          const double j0 =
              x7::diagInertia(chain, map, q_ref, probe_joint);
          const auto d0 = x7::buildScheduleFromSpecs(freqs, j0, a_cap);
          worst += x7::baseWorstSeconds(d0);
        }
        if (worst > x7::kTStopS) {
          std::fprintf(stderr,
                       "schedule worst case %.1f s (incl. capture + "
                       "setup) exceeds T_stop %.1f s — split it "
                       "(--freqs); refusing BEFORE any motion\n",
                       worst, x7::kTStopS);
          return 1;
        }
      }
      // Integrated placement (reviewer-approved): position-mode move
      // with MEASURED-posture convergence, then the minimal in-place
      // transition to current mode with the clamped gravity-current
      // preload landing before torque re-enable.
      if (!session.arm->activate()) {
        std::fprintf(stderr, "position activation failed: %s\n",
                     session.arm->lastError().c_str());
        return 1;
      }
      // Every position-phase abort must VERIFY its shutdown: a failed
      // deactivation leaves the failed servos torqued with their Bus
      // Watchdogs still armed (by design) — they halt on the bus
      // silence when this program exits, but the operator must be told
      // the truth about the state (review finding: one path claimed
      // "released" while the arm was still held).
      const auto positionPhaseAbort = [&session]() -> bool {
        const bool clean = session.arm->deactivate();
        if (!clean) {
          std::fprintf(stderr,
                       "SHUTDOWN FAULT: position-phase deactivation "
                       "incomplete — still-torqued servos keep their "
                       "Bus Watchdogs armed and will halt on bus "
                       "silence at exit; verify the arm is limp "
                       "before approaching\n");
        }
        return clean;
      };
      // Health gate BEFORE motion (review finding: an overheated arm
      // must not perform the placement move first).
      {
        const auto fb = session.arm->lastFeedback();
        std::array<x7::JointHealth, model::kCanonicalDof> h;
        std::string why;
        bool ok = static_cast<int>(fb.size()) == model::kCanonicalDof;
        for (int i = 0; ok && i < model::kCanonicalDof; ++i) {
          h[i] = {fb[i].temperature, fb[i].voltage};
        }
        if (!ok || !x7::preRunHealthOk(h, &why)) {
          std::fprintf(stderr,
                       "pre-placement health gate: %s — let the arm "
                       "cool / check supply, then rerun\n",
                       why.empty() ? "health read failed" : why.c_str());
          positionPhaseAbort();
          return 1;
        }
      }
      const auto placed =
          x7::movePose(*session.arm, anchor_ref, pose_vel, 0.01, 5);
      constexpr double kPreSwitchTol = 0.03;  // capture envelope margin
      if (!placed.ok || placed.worst_dev > kPreSwitchTol) {
        std::fprintf(stderr,
                     "placement did not reach a safe capture envelope "
                     "(worst joint %d at %.4f rad, bound %.3f) — "
                     "deactivating\n",
                     placed.worst_joint, placed.worst_dev, kPreSwitchTol);
        positionPhaseAbort();
        return 1;
      }
      // gravity-current preload from the MEASURED pre-switch posture
      model::ZVector q_meas(model::kCanonicalDof);
      model::ZVector tau_g(model::kCanonicalDof);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        zVecElemNC(q_meas.get(), i) = placed.measured[i];
      }
      chain.gravityTorque(map, q_meas.get(), tau_g.get());
      std::vector<double> preload(model::kCanonicalDof);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        preload[i] = tau_g[i] / rtctrl::dxl::torqueConstant(
                                    session.config.joints[i].model_number);
      }
      // Minimal in-place transition (review finding: the rebuild path
      // repeated the FULL activation — verification, indirect maps,
      // limits — all unsupported): ~25 transactions, torque-off to
      // torque-on, preload in the goal registers before the enable.
      std::printf("switching to current mode in place (gravity preload "
                  "armed)...\n");
      if (!session.arm->switchToCurrentModeWithPreload(preload)) {
        std::fprintf(stderr, "mode switch failed: %s\n",
                     session.arm->lastError().c_str());
        // Two distinct hardware states (review finding — the old
        // message claimed "released" for both): a PRE-sequence refusal
        // (state read, clipped/gated preload) leaves the arm ACTIVE
        // and held in position mode — deactivate it and verify; only
        // a mid-sequence failure has already released it.
        if (session.arm->activated()) {
          if (positionPhaseAbort()) {
            std::fprintf(stderr,
                         "the switch was refused before any torque-off; "
                         "the arm has been deactivated\n");
          }
        } else {
          std::fprintf(stderr,
                       "the switch failed mid-sequence and released "
                       "the arm — check it before rerunning\n");
        }
        return 1;
      }
      // keep session.config consistent with the switched arm
      for (auto& joint : session.config.joints) joint.operating_mode = 0;
    }

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
    // Every post-activation exit funnels through this — INCLUDING an
    // exception unwinding to the outer catch (the destructor runs the
    // same path; a review finding showed exceptions bypassed it). A
    // deactivation that could not CONFIRM zero + torque-off on every
    // servo leaves the failed servos' Bus Watchdogs armed — silence
    // the bus so they fire, and say so loudly; the run must then not
    // report success. The session watchdog disarms only after the
    // shutdown attempt (its deadline covers deactivation).
    struct ShutdownGuard {
      arm::RealArm& robot;
      rtctrl::hw::CraneX7* hw;
      x7::SessionWatchdog& watchdog;
      bool done = false;
      bool run() {
        done = true;
        const bool clean = robot.deactivate();
        if (!clean) {
          hw->requestQuiesce();
          std::fprintf(stderr,
                       "SHUTDOWN FAULT: deactivation incomplete — bus "
                       "silenced, armed servo watchdogs will halt any "
                       "still-torqued joint. Verify the arm is limp "
                       "before approaching; power-cycle before the "
                       "next run.\n");
        }
        watchdog.disarm();
        return clean;
      }
      ~ShutdownGuard() {
        if (done) return;
        try {
          run();
        } catch (...) {
        }
      }
    } shutdown{robot, hw, watchdog};

    arm::JointState start;
    if (!robot.readState(start)) {
      shutdown.run();
      return 1;
    }
    arm::JointCommand hold;
    hold.mode = arm::ControlMode::Current;
    chain.gravityTorque(map, start.q.get(), hold.tau.get());
    arm::CommandReceipt hold_receipt;
    robot.writeCommand(hold, &hold_receipt);
    // The cue promises only what is CONFIRMED (review finding: printing
    // on submission preceded application by a cycle): wait until the
    // applied record shows our submission on the actuators.
    {
      bool applied = false;
      std::uint64_t seen = 0;
      for (int k = 0; k < 100 && !applied; ++k) {  // <= ~1 s of cycles
        seen = hw->waitCycle(seen);
        if (seen == 0) break;  // thread stopped; the runner would abort
        arm::CommandSnapshot snap;
        hw->lastFeedbackStamped(&snap);
        applied = snap.applied.valid &&
                  snap.applied.target_seq >= hold_receipt.submitted_seq;
      }
      if (!applied) {
        std::fprintf(stderr,
                     "gravity hold was not confirmed applied — "
                     "aborting\n");
        shutdown.run();
        return 1;
      }
    }
    if (pose_first) {
      std::printf("gravity hold applied (confirmed); the capture phase "
                  "will pull the residual onto the anchor\n");
    } else {
      // Operator cue for the manual handover: the hold FLOATS — a hand
      // that keeps steadying or lifting re-poses the arm permanently
      // (a first-session settle log showed the tilt raised 0.4 rad by
      // a well-meaning supporting hand).
      std::printf(
          ">>> gravity hold applied (confirmed) — RELEASE the arm now.\n"
          ">>> Do not keep steadying or lifting it: the hold floats,\n"
          ">>> and a touched arm is re-posed, not corrected.\n");
    }

    if (!pose_first) {
      // Strict settle gate for the hand-placed flow, no override: the
      // probe must anchor on a quiescent arm. 10 s leaves room for a
      // slow hand-release tail on top of the equilibration the gate
      // must ride out. (--pose-first replaces this with the capture
      // phase's quiescence-under-hold gates inside the run.)
      x7::SettleController settle(chain, map, x7::tuning::kSettleKd);
      std::FILE* settle_log = nullptr;
      if (!log_path.empty()) {
        settle_log = std::fopen((log_path + ".settle").c_str(), "w");
      }
      const auto settled =
          x7::settleArm(robot, settle, 10.0, settle_log);
      if (settle_log != nullptr) std::fclose(settle_log);
      if (!settled.io_ok || !settled.quiescent) {
        std::fprintf(stderr, "settle phase %s — aborting\n",
                     settled.io_ok ? "did not reach quiescence"
                                   : "aborted");
        shutdown.run();
        return 1;
      }
    }
    if (!robot.readState(start)) {
      shutdown.run();
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
        shutdown.run();
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
        shutdown.run();
        return 1;
      }
    }

    // The run's anchor: with --pose-first it IS the canonical
    // reference (the capture phase pulls onto it and enforces the
    // tolerance under the hold); the hand-placed flow anchors on the
    // settled posture and gates it against the reference here.
    std::vector<double> anchor(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      anchor[i] = pose_first ? anchor_ref[i]
                             : zVecElemNC(start.q.get(), i);
    }
    if (!pose_first && !anchor_ref_path.empty()) {
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
        shutdown.run();
        return 1;
      }
    }

    // Schedule with the amplitude rule at THIS anchor's inertia (the
    // hold diagnostic never probes: empty dwell list).
    model::ZVector q_anchor(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      zVecElemNC(q_anchor.get(), i) = anchor[i];
    }
    const double j_hat = x7::diagInertia(chain, map, q_anchor,
                                         probe_joint);
    auto dwells = hold_mode
                      ? std::vector<x7::DwellSpec>{}
                      : x7::buildScheduleFromSpecs(freqs, j_hat, a_cap);
    // Pre-activation-style refusal (checked before settle would be
    // ideal, but the amplitudes depend on the settled anchor; the
    // authoritative admission below covers the difference). The
    // capture phase spends session time too.
    const double capture_worst =
        pose_first ? x7::kCaptureWorstS + hold_s : 0.0;
    const double setup_allow = pose_first
                                   ? x7::kPoseFirstSetupAllowanceS
                                   : x7::kSetupAllowanceS;
    if (x7::baseWorstSeconds(dwells) + setup_allow + capture_worst >
        x7::kTStopS) {
      std::fprintf(stderr,
                   "schedule worst case %.1f s + %.0f s setup exceeds "
                   "T_stop %.1f s — split it (--freqs)\n",
                   x7::baseWorstSeconds(dwells) + capture_worst,
                   setup_allow, x7::kTStopS);
      shutdown.run();
      return 1;
    }
    if (hold_mode) {
      std::printf("HOLD DIAGNOSTIC: capture, then %.0f s unforced "
                  "anchor hold under continuous gates (joint-0 scale "
                  "%.2f%s); no probing\n",
                  hold_s, scale0 > 0.0 ? scale0 : x7::kGainScale[0],
                  freeze_i ? ", integral FROZEN at capture" : "");
    } else {
      std::printf("probe joint %d (J_hat %.4f kg m^2), %zu dwells, "
                  "lead-in %.1f s, worst case %.1f s:\n",
                  probe_joint, j_hat, dwells.size(),
                  x7::leadInSeconds(dwells),
                  x7::baseWorstSeconds(dwells));
      for (const auto& d : dwells) {
        std::printf("  %6.2f Hz  amp %.3f Nm%s\n", d.freq_hz, d.amp_nm,
                    d.window_mult > 1 ? "  (x4 window requested)" : "");
      }
    }

    // POST-SETTLE ADMISSION (authoritative): the deadline runs from
    // activation, and settling/gates have been spending it. With
    // --pose-first the capture phase's worst case rides inside the
    // run and must fit too.
    const double setup_elapsed = sinceActivation();
    if (setup_elapsed + capture_worst + x7::baseWorstSeconds(dwells) >
        x7::kTStopS) {
      std::fprintf(stderr,
                   "setup consumed %.1f s: the worst-case schedule no "
                   "longer fits T_stop — deactivating\n",
                   setup_elapsed);
      shutdown.run();
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
      if (hold_mode) {
        std::snprintf(num, sizeof num, " hold=%.0f scale0=%.3f freeze_i=%d",
                      hold_s, scale0, freeze_i ? 1 : 0);
        meta += num;
      }
      log = x7::openIdentCsvLog(log_path, meta);
      if (!log) {
        std::fprintf(stderr, "cannot open log file %s\n",
                     log_path.c_str());
        shutdown.run();
        return 1;
      }
    }

    x7::IdentRun::Options opt;
    opt.probe_joint = probe_joint;
    opt.dwells = dwells;
    opt.anchor = anchor;
    opt.setup_offset_s = setup_elapsed;
    opt.health = health;
    if (pose_first) {
      opt.capture_envelope_rad = x7::kCaptureEnvelopeRad;
    }
    if (hold_mode) {
      opt.hold_only_s = hold_s;
      opt.hold_scale0 = scale0;
      opt.freeze_integral_at_capture = freeze_i;
    }
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
    const bool clean_shutdown = shutdown.run();
    const auto stats = hw->cycleStats();
    if (log) std::fclose(log);
    if (!log_path.empty()) {
      x7::writeDwellJson(log_path + ".dwells.json", opt, ident);
    }

    if (pose_first && ident.captureAcceptedAt() > 0.0) {
      std::printf("capture: initial residual %.4f rad, gates passed at "
                  "%.2f s under the hold\n",
                  ident.captureInitialDev(), ident.captureAcceptedAt());
    }
    if (hold_mode) {
      std::printf("hold diagnostic (joint-0 scale %.2f, %.1f of %.0f s "
                  "completed; enforced = after the %.0f s grace):\n",
                  ident.gainScales()[0], ident.holdCompletedS(), hold_s,
                  x7::kHoldDiagGraceS);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::printf(
            "  joint %d: max metric %.4f enforced %.4f rad/s "
            "(bound 0.05)  p-p %.5f rad%s\n",
            i, ident.holdMaxSpeed(i), ident.holdMaxSpeedEnforced(i),
            ident.holdRangeRad(i),
            ident.holdMaxSpeedEnforced(i) >= 0.05 ? "  <-- FAIL" : "");
      }
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
