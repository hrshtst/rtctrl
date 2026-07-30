// M7 hardware phase: gravity compensation on the real arm. The arm
// floats — held against gravity, freely back-drivable by hand — while
// the console compares measured torque (current × torque constant)
// against the rkChainID_G prediction each second.
//
// SAFETY: current mode. Keep the power cutoff in reach. Verified in
// sim first (gravity_sim_test): drift < 0.05 rad over 10 s.
//
// PARKED (2026-07-29): do NOT run on hardware until the gravity
// calibration passes its hardware milestone
// (docs/GRAVITY_CALIBRATION_PLAN.md M-GC3). A float session
// accelerated the UNTOUCHED arm toward the upright posture (peak
// ~2.37 rad/s) — gravity over-compensation; this file now implements
// the M-GC1 remediation startup, but the parking stands until the
// displacement-bounded hardware test passes on the vendor-scale
// configuration.
//
// Startup (M-GC1, the x7_ident pose-first pattern): the arm activates
// in POSITION mode — servo-held, no free-fall instant — the held
// posture is read, a start inside the soft-limit margin band is
// refused, and the scale-calibrated gravity currents are staged
// through the in-place switch to current mode
// (CraneX7::switchToCurrentModeWithPreload), so support flows from
// the first torque-on instant. The per-joint command_torque_scale is
// printed and recorded in the log header.
//
// Release marker (M-GC3 protocol): press ENTER while STILL SUPPORTING
// the arm; the console acknowledges and the CSV records the
// `released` column transition — release the arm ON that cue, so the
// logged marker slightly precedes the physical release. The run ends
// automatically 5 s after the marker; without a marker within 8 s of
// the switch the run aborts (the commanded duration is only the outer
// deadline).
//
// Usage: x7_float [--config path] [--port dev] [--log out.csv] [seconds]
//   --log writes one row per cycle through the CycleObserver — AFTER
//   writeCommand, so each row pairs three distinct events explicitly:
//   the FEEDBACK snapshot the cycle started from (q, dqservo — the
//   servo's lagged estimate — and current-derived tau_meas), THIS
//   CYCLE'S REQUEST (tau_request, the g(q) command, with its receipt:
//   submitted_seq / submission_time / accepted — it is applied LATER),
//   and the LATEST APPLIED command in the COHERENT SNAPSHOT
//   (tau_applied with the hardware clamp/gate flags). The applied
//   record may POSTDATE this row's feedback — the producer reads
//   before it writes — so tau_meas vs tau_applied in one row is not a
//   causal pair either: use the SIGNED feedback_minus_latest_apply
//   column to order the events before comparing torques. With a
//   command scale s < 1 the agreement print and tau_applied read
//   ~s × tau_request BY DESIGN — the scale is finally visible. Raw
//   hardware CSVs land at the repo root gitignored and belong in the
//   private archive (docs/DATA_ARCHIVE.md); use a unique filename per
//   attempt.

#include <fcntl.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/hw/command_current.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;

namespace {

struct ReportingGravityComp : arm::GravityComp {
  ReportingGravityComp(model::ChainModel& chain,
                       const model::JointMap& map)
      : GravityComp(chain, map), chain_(chain), map_(map) {}

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    GravityComp::update(state, cmd, t);
    if (t - last_report_ >= 1.0) {
      last_report_ = t;
      chain_.gravityTorque(map_, state.q.get(), predicted_.get());
      std::printf("t=%4.0fs  measured vs predicted tau [Nm]:\n", t);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::printf("  j%d %+6.2f / %+6.2f", i,
                    zVecElemNC(state.tau.get(), i), predicted_[i]);
      }
      std::printf("\n");
    }
  }

  model::ChainModel& chain_;
  const model::JointMap& map_;
  model::ZVector predicted_{model::kCanonicalDof};
  double last_report_ = -1.0;
};

// Nonblocking check for the release key (ENTER). Only actual bytes
// count — EOF on a piped stdin must never read as a keypress.
bool pollReleaseKey() {
  static bool configured = false;
  if (!configured) {
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    configured = true;
  }
  char buf[64];
  return read(STDIN_FILENO, buf, sizeof buf) > 0;
}

// Marker state shared between the observer (which detects and
// terminates) and main (which interprets the run's end).
struct ReleaseMarker {
  bool released = false;
  double t_released = 0.0;
  bool window_complete = false;  // ended 5 s after the marker
  bool marker_timeout = false;   // no marker within 8 s of the switch
};

constexpr double kEvaluationWindowS = 5.0;  // after the marker
constexpr double kMarkerDeadlineS = 8.0;    // after the mode switch

// Per-cycle telemetry via the CycleObserver — invoked AFTER
// writeCommand, so the row carries this cycle's receipt (was the
// request accepted?) and the pre-write snapshot's latest-applied
// record, keeping the three events distinguishable (review finding:
// logging inside update() recorded a request before knowing whether
// it was ever accepted or applied). Also owns the release-marker
// detection and the marker-anchored termination.
struct FloatLogObserver : arm::CycleObserver {
  FloatLogObserver(std::FILE* log, ReleaseMarker* marker)
      : log_(log), marker_(marker) {}

  bool observe(double t, const arm::JointState& state,
               const arm::CommandSnapshot& cmds,
               const arm::JointCommand& cmd,
               const arm::CommandReceipt& receipt) override {
    if (!marker_->released && pollReleaseKey()) {
      marker_->released = true;
      marker_->t_released = t;
      std::printf("RELEASED (t=%.2f s) — release the arm NOW; %.0f s "
                  "evaluation window started\n",
                  t, kEvaluationWindowS);
    }
    if (log_ != nullptr) {
      const auto& applied = cmds.applied;
      // Timing fields as in x7_track's log: the latest apply may
      // POSTDATE this row's feedback (the producer reads before it
      // writes), so the offset is signed; both are meaningful only
      // when applied_valid = 1.
      std::fprintf(log_,
                   "%.4f,%.6f,%llu,%llu,%.6f,%d,%d,%llu,%.6f,%.6f,%d",
                   t, state.t,
                   static_cast<unsigned long long>(state.seq),
                   static_cast<unsigned long long>(receipt.submitted_seq),
                   receipt.submission_time, receipt.accepted ? 1 : 0,
                   applied.valid ? 1 : 0,
                   static_cast<unsigned long long>(applied.target_seq),
                   applied.valid ? applied.latest_time : 0.0,
                   applied.valid ? state.t - applied.latest_time : 0.0,
                   marker_->released ? 1 : 0);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::fprintf(log_, ",%.6f,%.6f,%.4f,%.4f,%.4f,%d,%d",
                     zVecElemNC(state.q.get(), i),
                     zVecElemNC(state.dq.get(), i),
                     zVecElemNC(state.tau.get(), i),
                     zVecElemNC(cmd.tau.get(), i),
                     applied.valid ? applied.applied[i] : 0.0,
                     (applied.flags[i] & arm::kCmdClamped) ? 1 : 0,
                     (applied.flags[i] & arm::kCmdGated) ? 1 : 0);
      }
      std::fprintf(log_, "\n");
    }
    // Marker-anchored termination (GRAVITY_CALIBRATION_PLAN M-GC1):
    // the full evaluation window is always logged; a run past the
    // marker deadline without a marker is an aborted test. Runs whose
    // commanded duration ends before the deadline (emulator smokes)
    // finish normally.
    if (marker_->released &&
        t >= marker_->t_released + kEvaluationWindowS) {
      marker_->window_complete = true;
      return false;
    }
    if (!marker_->released && t > kMarkerDeadlineS) {
      marker_->marker_timeout = true;
      return false;
    }
    return true;
  }

  std::FILE* log_;
  ReleaseMarker* marker_;
};

// fopen only — BEFORE any bus contact, so a bad path fails without
// ever torquing the arm. The headers need the loaded config (the
// scales), so they are written separately once it exists.
std::FILE* openFloatLogFile(const std::string& path) {
  return std::fopen(path.c_str(), "w");
}

void writeFloatLogHeader(std::FILE* log, const hw::Config& config) {
  std::fprintf(log,
               "# events per row: FEEDBACK (feedback_time/feedback_seq, "
               "q, dqservo, tau_meas) | THIS CYCLE'S REQUEST "
               "(submitted_seq/submission_time/accepted, tau_request — "
               "applied LATER) | LATEST APPLIED in the coherent "
               "snapshot (applied_valid/applied_seq/latest_apply_time, "
               "tau_applied, clamped/gated). feedback_minus_latest_apply "
               "is SIGNED: the producer reads before it writes, so the "
               "latest apply may post-date this row's feedback; applied "
               "fields are meaningful only when applied_valid=1. "
               "released flips to 1 at the operator's marker, which "
               "PRECEDES the physical release.\n");
  std::fprintf(log, "# command_torque_scale:");
  for (const auto& joint : config.joints) {
    std::fprintf(log, " %.6f", joint.command_torque_scale);
  }
  std::fprintf(log, "\n");
  std::fprintf(log, "t,feedback_time,feedback_seq,submitted_seq,"
                    "submission_time,accepted,applied_valid,applied_seq,"
                    "latest_apply_time,feedback_minus_latest_apply,"
                    "released");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log,
                 ",q%d,dqservo%d,tau_meas%d,tau_request%d,tau_applied%d,"
                 "clamped%d,gated%d",
                 i, i, i, i, i, i, i);
  }
  std::fprintf(log, "\n");
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  double duration_s = 30.0;
  bool duration_given = false;
  std::string log_path;
  const auto& rest = cli.rest;
  for (std::size_t i = 0; i < rest.size(); ++i) {
    if (std::strcmp(rest[i], "--log") == 0) {
      // A flag-looking next token is a MISSING value, not a filename —
      // otherwise "--log --bogus" silently creates ./--bogus and
      // proceeds toward the hardware session (review finding). A
      // genuinely flag-like filename can be written as ./--name.
      if (i + 1 >= rest.size() ||
          std::strncmp(rest[i + 1], "--", 2) == 0) {
        std::fprintf(stderr, "--log requires a value\n");
        return 1;
      }
      log_path = rest[++i];
    } else if (rest[i][0] == '-' && rest[i][1] == '-') {
      std::fprintf(stderr, "unknown argument: %s\n", rest[i]);
      return 1;
    } else if (duration_given) {
      std::fprintf(stderr, "usage: x7_float [--config path] [--port dev] "
                           "[--log out.csv] [seconds]\n");
      return 1;
    } else if (!x7::parseStrictDouble(rest[i], &duration_s)) {
      std::fprintf(stderr, "invalid duration: %s\n", rest[i]);
      return 1;
    } else {
      duration_given = true;
    }
  }

  // Open the log BEFORE any bus contact: a bad path must fail without
  // ever torquing the arm. (Headers follow once the config is loaded.)
  std::FILE* log = nullptr;
  if (!log_path.empty()) {
    log = openFloatLogFile(log_path);
    if (log == nullptr) {
      std::fprintf(stderr, "cannot open log file %s\n", log_path.c_str());
      return 1;
    }
  }

  try {
    // POSITION-mode session: activation holds the arm (goals snap to
    // the present posture) — no free-fall instant. The switch to
    // current mode below carries the calibrated gravity preload.
    auto session = x7::openSession(cli, /*operating_mode_override=*/3);
    if (log != nullptr) writeFloatLogHeader(log, session.config);
    std::printf("command_torque_scale:");
    for (const auto& joint : session.config.joints) {
      std::printf(" %.6f", joint.command_torque_scale);
    }
    std::printf("\n");

    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);

    if (!session.arm->activate()) {
      std::fprintf(stderr, "position activation failed: %s\n",
                   session.arm->lastError().c_str());
      return 1;
    }
    x7::ShutdownGuard shutdown{*session.arm};

    // Held posture — the preload's evaluation point.
    const auto fb = session.arm->lastFeedback();
    if (static_cast<int>(fb.size()) != model::kCanonicalDof) {
      std::fprintf(stderr, "held-posture read failed — aborting\n");
      shutdown.run();
      return 1;
    }
    // Refuse a float from inside the soft-limit margin band, as
    // x7_track does: the current gate would cut the gravity support in
    // one whole direction there.
    {
      const auto& lo = session.arm->softLimitLo();
      const auto& hi = session.arm->softLimitHi();
      constexpr double kBuffer = 0.05;  // [rad] beyond the margin
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        if (fb[i].position < lo[i] + kBuffer ||
            fb[i].position > hi[i] - kBuffer) {
          std::fprintf(stderr,
                       "joint %d is at %.3f rad, within its soft-limit "
                       "margin band [%.3f, %.3f] — reposition mid-range "
                       "and rerun\n",
                       i, fb[i].position, lo[i], hi[i]);
          shutdown.run();
          return 1;
        }
      }
    }

    // Calibrated gravity preload from the HELD posture, through THE
    // shared torque boundary (scale applied exactly once).
    model::ZVector q_held(model::kCanonicalDof);
    model::ZVector tau_g(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      zVecElemNC(q_held.get(), i) = fb[i].position;
    }
    chain.gravityTorque(map, q_held.get(), tau_g.get());
    std::vector<double> preload(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      preload[i] = hw::commandCurrentFromTorque(session.config.joints[i],
                                                tau_g[i]);
    }
    std::printf("switching to current mode in place (calibrated gravity "
                "preload armed)...\n");
    if (!session.arm->switchToCurrentModeWithPreload(preload)) {
      std::fprintf(stderr, "mode switch failed: %s\n",
                   session.arm->lastError().c_str());
      // Two distinct hardware states: a PRE-sequence refusal (state
      // read, clipped/gated preload) leaves the arm ACTIVE and HELD in
      // position mode — deactivate and verify; a mid-sequence failure
      // has already best-effort released it.
      if (session.arm->activated()) {
        shutdown.run();
        std::fprintf(stderr, "the switch was refused before any "
                             "torque-off; the arm has been "
                             "deactivated\n");
      } else {
        std::fprintf(stderr, "the switch failed mid-sequence and "
                             "released the arm — check it before "
                             "rerunning\n");
      }
      return 1;
    }
    for (auto& joint : session.config.joints) joint.operating_mode = 0;

    arm::RealArm robot(*session.arm);
    if (!robot.activate()) {  // already active: starts the thread,
                              // which retransmits the preload
      std::fprintf(stderr, "thread start failed: %s\n",
                   session.arm->lastError().c_str());
      shutdown.run();
      return 1;
    }
    ReleaseMarker marker;
    ReportingGravityComp controller(chain, map);
    FloatLogObserver observer(log, &marker);
    std::printf("floating (outer deadline %.0f s) — press ENTER while "
                "still supporting to mark release (deadline %.0f s), "
                "then release on the cue; the run ends %.0f s after "
                "the marker\n",
                duration_s, kMarkerDeadlineS, kEvaluationWindowS);
    const bool ran = arm::run(robot, controller, duration_s, &observer);
    const bool clean = shutdown.run();
    if (log != nullptr) std::fclose(log);
    // A marker-completed window is a SUCCESSFUL end (the observer
    // vetoes the runner to stop the run); a marker timeout is a void
    // attempt; anything else follows the runner's verdict.
    const bool ok = marker.window_complete || (ran && !marker.marker_timeout);
    if (!clean) {
      std::printf("SHUTDOWN FAULT (run %s)\n", ok ? "done" : "ABORTED");
      return 1;
    }
    if (marker.window_complete) {
      std::printf("evaluation window complete (marker at %.2f s) — "
                  "done\n", marker.t_released);
      return 0;
    }
    if (marker.marker_timeout) {
      std::printf("NO release marker within %.0f s — aborted test "
                  "(void attempt, arm deactivated)\n",
                  kMarkerDeadlineS);
      return 1;
    }
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
