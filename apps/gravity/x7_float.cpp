// M7 hardware phase: gravity compensation on the real arm. The arm
// floats — held against gravity, freely back-drivable by hand — while
// the console compares measured torque (current × torque constant)
// against the rkChainID_G prediction each second.
//
// SAFETY: current mode. Keep the power cutoff in reach. Verified in
// sim first (gravity_sim_test): drift < 0.05 rad over 10 s.
//
// UN-PARKED (2026-07-31, owner decision recorded in
// docs/records/history.md (gravity calibration); parked 2026-07-29 after gravity
// over-compensation drove the untouched arm upright at ~2.37 rad/s):
// the M-GC3 objective acceptance PASSED on the vendor-scale
// configuration; the subjective back-drive criterion remains FAILED
// and is WAIVED as an explicit risk/quality acceptance of two
// characterized behaviors — the j1 notch (energized actuator-side
// behavior, strongly associated with crossing the low-current
// transition region q1 ~ +0.27..+0.53 rad; mechanism not isolated)
// and the small j4 positive bias (model-amplitude mismatch, command
// <= 0.055 Nm). Default-scale adoption was DECLINED — the repo
// default config keeps the known-failed all-1.0 scales — so EVERY
// x7_float session refuses to touch the bus without the APPROVED
// vendor calibration (config/crane_x7_vendor_scale.toml) and a
// REQUIRED --log file, created exclusively (an existing file is
// never overwritten). x7_track remains parked (separate
// disposition).
//
// Startup (M-GC1, the pose-first placement pattern): the arm activates
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
// automatically after the selected evaluation window (10 s by
// default); without a marker within 8 s of the switch the run aborts
// (the commanded duration is only the outer deadline).
//
// Usage: x7_float [--config path] [--port dev] [--log out.csv]
//                 [--evaluation-time seconds] [--feel] [seconds]
//   --config MUST carry the approved vendor scales in every mode
//   (config/crane_x7_vendor_scale.toml) — refused before bus contact
//   otherwise; the repo default config is deliberately NOT accepted.
//   --evaluation-time sets the normal-mode, marker-anchored evaluation
//   window (default 10 s, range 5..50 s). The outer duration must cover
//   the 8 s marker deadline, the selected window, and a 2 s margin.
//   A non-default window is demonstration-only: its log self-labels
//   "demonstration" and is never acceptance evidence.
//   It is rejected with --feel, whose duration already controls the
//   complete post-marker demonstration session.
//   --feel: feel-check mode for the M-GC3 back-drive confirmation.
//   The marker contract is UNCHANGED (press ENTER while supporting
//   within 8 s or the run aborts), but the session then runs to the
//   commanded outer deadline instead of ending after the evaluation
//   window — room to back-drive each joint carefully. TORQUE REMAINS
//   ENABLED until that deadline (the release cue says so). Duration is
//   bounded to 60 s in EVERY mode (normal runs self-terminate at the
//   selected marker-anchored window), and EVERY mode requires the
//   APPROVED vendor calibration vector (±1e-6) BEFORE bus contact — the
//   known-failed all-1.0 default, near-1 lookalikes, and
//   under-supporting low scales are all refused (feel-only
//   originally; mode-independent since the 2026-07-31 un-parking).
//   The log self-marks "# run_mode: feel-check" and is NEVER
//   acceptance evidence; the acceptance protocol (no --feel) is
//   unchanged.
//   --log is REQUIRED (un-parking condition): the file is created
//   EXCLUSIVELY — an existing file is refused, never overwritten —
//   enforcing the unique-filename-per-attempt rule before bus
//   contact. It writes one row per cycle through the CycleObserver — AFTER
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
//   ~s × tau_request BY DESIGN — the scale is finally visible.
//   Reviewer-directed instrumentation (back-drive exceptions): after
//   the release marker, every further ENTER press is an OPERATOR
//   EVENT MARK — operator_event=1 on that row, its t the event
//   timestamp — so a felt notch can be correlated with current
//   behavior; goal_cnt/present_cnt carry the RAW servo current counts
//   (goal from the latest applied record, present from feedback),
//   reconstructed exactly through the wire's own dxl conversion (a
//   deliberate raw-unit exception confined to this log). Raw
//   hardware CSVs land at the repo root gitignored and belong in the
//   private archive (docs/records/data-archive.md); use a unique filename per
//   attempt.

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/hw/command_current.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "common/x7_common.hpp"
#include "gravity/gravity_support.hpp"

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
  bool window_complete = false;  // selected window ended after marker
  bool marker_timeout = false;   // no marker within 8 s of the switch
};

constexpr double kDefaultEvaluationWindowS = 10.0;  // after the marker
constexpr double kMarkerDeadlineS = 8.0;             // after the mode switch
constexpr double kDurationMarginS = 2.0;
constexpr double kMinEvaluationWindowS = 5.0;
constexpr double kMaxEvaluationWindowS = 50.0;
constexpr double kMinFeelDurationS = 15.0;
// GLOBAL upper bound (reviewed): one feel joint per run needs no
// more, normal runs self-terminate at the selected window anyway, an
// unbounded torqued session with a healthy command stream never trips
// any watchdog, and a huge duration reaches the runner's
// floating-to-long cycle conversion (review findings).
constexpr double kMaxDurationS = 60.0;
// The APPROVED M-GC3 calibration vector (mirrors
// config/crane_x7_vendor_scale.toml and the log checker): feel mode
// requires exactly this — near-1 values are effectively the failed
// calibration, very low values under-support the arm.
// Per-cycle telemetry via the CycleObserver — invoked AFTER
// writeCommand, so the row carries this cycle's receipt (was the
// request accepted?) and the pre-write snapshot's latest-applied
// record, keeping the three events distinguishable (review finding:
// logging inside update() recorded a request before knowing whether
// it was ever accepted or applied). Also owns the release-marker
// detection and the marker-anchored termination.
struct FloatLogObserver : arm::CycleObserver {
  FloatLogObserver(std::FILE* log, ReleaseMarker* marker,
                   const hw::Config* config, bool feel,
                   double outer_deadline_s, double evaluation_window_s)
      : log_(log), marker_(marker), config_(config), feel_(feel),
        outer_deadline_s_(outer_deadline_s),
        evaluation_window_s_(evaluation_window_s) {}

  bool observe(double t, const arm::JointState& state,
               const arm::CommandSnapshot& cmds,
               const arm::JointCommand& cmd,
               const arm::CommandReceipt& receipt) override {
    bool event_mark = false;
    if (pollReleaseKey()) {
      if (!marker_->released) {
        marker_->released = true;
        marker_->t_released = t;
        if (feel_) {
          // The normal-mode cue would mislead here: after its evaluation
          // window NOTHING ends — an operator expecting a limp arm could
          // be surprised by a still-torqued one (review finding).
          std::printf("RELEASED (t=%.2f s) — release the arm NOW; "
                      "FEEL-CHECK continues, TORQUE REMAINS ENABLED "
                      "until the outer deadline (%.0f s); press ENTER "
                      "again at each notch to log an EVENT MARK\n",
                      t, outer_deadline_s_);
        } else {
          std::printf("RELEASED (t=%.2f s) — release the arm NOW; "
                      "%.1f s evaluation window started\n",
                      t, evaluation_window_s_);
        }
      } else {
        // Post-release ENTER = operator event mark
        // (reviewer-directed instrumentation, back-drive exceptions):
        // the row's t timestamps the sensation so a felt notch can be
        // correlated offline with count-level current behavior.
        // Presses landing within one cycle merge into one mark.
        event_mark = true;
        std::printf("EVENT MARK recorded (t=%.2f s)\n", t);
      }
    }
    if (log_ != nullptr) {
      const auto& applied = cmds.applied;
      // Timing fields as in x7_track's log: the latest apply may
      // POSTDATE this row's feedback (the producer reads before it
      // writes), so the offset is signed; both are meaningful only
      // when applied_valid = 1.
      std::fprintf(log_,
                   "%.4f,%.6f,%llu,%llu,%.6f,%d,%d,%llu,%.6f,%.6f,%d,%d",
                   t, state.t,
                   static_cast<unsigned long long>(state.seq),
                   static_cast<unsigned long long>(receipt.submitted_seq),
                   receipt.submission_time, receipt.accepted ? 1 : 0,
                   applied.valid ? 1 : 0,
                   static_cast<unsigned long long>(applied.target_seq),
                   applied.valid ? applied.latest_time : 0.0,
                   applied.valid ? state.t - applied.latest_time : 0.0,
                   marker_->released ? 1 : 0, event_mark ? 1 : 0);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        // RAW goal/present current counts (reviewer-directed
        // instrumentation — a deliberate raw-unit exception in this
        // log only): the wire path converts SI amps with
        // dxl::ampsToCurrent (lround at the 2.69 mA LSB) and feedback
        // amps are count × LSB exactly, so reconverting the SI
        // telemetry through the SAME function reproduces the wire's
        // int16 (a 1-ulp round-trip on the applied record; an exact
        // half-count boundary is the only — measure-zero — exception).
        const double kt = rtctrl::dxl::torqueConstant(
            config_->joints[i].model_number);
        const int goal_cnt =
            applied.valid
                ? rtctrl::dxl::ampsToCurrent(applied.applied[i] / kt)
                : 0;
        const int present_cnt = rtctrl::dxl::ampsToCurrent(
            zVecElemNC(state.tau.get(), i) / kt);
        std::fprintf(log_, ",%.6f,%.6f,%.4f,%.4f,%.4f,%d,%d,%d,%d",
                     zVecElemNC(state.q.get(), i),
                     zVecElemNC(state.dq.get(), i),
                     zVecElemNC(state.tau.get(), i),
                     zVecElemNC(cmd.tau.get(), i),
                     applied.valid ? applied.applied[i] : 0.0,
                     (applied.flags[i] & arm::kCmdClamped) ? 1 : 0,
                     (applied.flags[i] & arm::kCmdGated) ? 1 : 0,
                     goal_cnt, present_cnt);
      }
      std::fprintf(log_, "\n");
    }
    // Marker-anchored termination (docs/records/history.md gravity calibration M-GC1):
    // the full evaluation window is always logged; a run past the
    // marker deadline without a marker is an aborted test. (The
    // enforced minimum duration guarantees these branches are always
    // reachable before the outer deadline.) Feel-check mode keeps the
    // marker deadline but runs to the OUTER deadline after the marker.
    if (!feel_ && marker_->released &&
        t >= marker_->t_released + evaluation_window_s_) {
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
  const hw::Config* config_;  // per-joint model numbers for the counts
  bool feel_;
  double outer_deadline_s_;
  double evaluation_window_s_;
};

// fopen only — BEFORE any bus contact, so a bad path fails without
// ever torquing the arm. EXCLUSIVE creation ("x"): an existing file
// is refused, never truncated — the unique-filename-per-attempt rule
// is enforced, not advisory (review finding: the stated un-parking
// condition was not enforced). The headers need the loaded config
// (the scales), so they are written separately once it exists.
std::FILE* openFloatLogFile(const std::string& path) {
  return std::fopen(path.c_str(), "wx");
}

void writeFloatLogHeader(std::FILE* log, const hw::Config& config,
                         bool feel, bool demonstration, double duration_s,
                         double evaluation_window_s) {
  // The log self-classifies: feel-check and non-default demonstration
  // windows must never be readable as acceptance evidence. The requested
  // duration and evaluation window let the checker prove that the selected
  // timing contract completed (review finding: without duration provenance
  // a truncated feel session passed validation).
  const char* run_mode =
      feel ? "feel-check" : (demonstration ? "demonstration" : "acceptance");
  std::fprintf(log, "# run_mode: %s\n", run_mode);
  std::fprintf(log, "# duration_s: %.1f\n", duration_s);
  if (!feel) {
    std::fprintf(log, "# evaluation_window_s: %.17g\n",
                 evaluation_window_s);
  }
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
               "PRECEDES the physical release. operator_event marks a "
               "post-release ENTER press on its row (the row's t is "
               "the operator's event timestamp). goal_cnt/present_cnt "
               "are RAW servo current counts (goal from the LATEST "
               "APPLIED record, 0 while applied_valid=0; present from "
               "this row's feedback), reconstructed through the "
               "wire's own conversion at the 2.69 mA LSB.\n");
  std::fprintf(log, "# command_torque_scale:");
  for (const auto& joint : config.joints) {
    std::fprintf(log, " %.6f", joint.command_torque_scale);
  }
  std::fprintf(log, "\n");
  std::fprintf(log, "t,feedback_time,feedback_seq,submitted_seq,"
                    "submission_time,accepted,applied_valid,applied_seq,"
                    "latest_apply_time,feedback_minus_latest_apply,"
                    "released,operator_event");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log,
                 ",q%d,dqservo%d,tau_meas%d,tau_request%d,tau_applied%d,"
                 "clamped%d,gated%d,goal_cnt%d,present_cnt%d",
                 i, i, i, i, i, i, i, i, i);
  }
  std::fprintf(log, "\n");
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  double duration_s = 30.0;
  double evaluation_window_s = kDefaultEvaluationWindowS;
  bool duration_given = false;
  bool evaluation_window_given = false;
  bool feel_mode = false;
  std::string log_path;
  std::string evaluation_window_token;
  const auto& rest = cli.rest;
  for (std::size_t i = 0; i < rest.size(); ++i) {
    if (std::strcmp(rest[i], "--feel") == 0) {
      feel_mode = true;
    } else if (std::strcmp(rest[i], "--log") == 0) {
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
    } else if (std::strcmp(rest[i], "--evaluation-time") == 0) {
      if (i + 1 >= rest.size() ||
          std::strncmp(rest[i + 1], "--", 2) == 0) {
        std::fprintf(stderr, "--evaluation-time requires a value\n");
        return 1;
      }
      evaluation_window_token = rest[++i];
      if (!x7::parseStrictDouble(evaluation_window_token.c_str(),
                                 &evaluation_window_s)) {
        std::fprintf(stderr, "invalid evaluation time: %s\n", rest[i]);
        return 1;
      }
      evaluation_window_given = true;
    } else if (rest[i][0] == '-' && rest[i][1] == '-') {
      std::fprintf(stderr, "unknown argument: %s\n", rest[i]);
      return 1;
    } else if (duration_given) {
      std::fprintf(stderr, "usage: x7_float [--config path] [--port dev] "
                           "[--log out.csv] [--evaluation-time seconds] "
                           "[--feel] [seconds]\n");
      return 1;
    } else if (!x7::parseStrictDouble(rest[i], &duration_s)) {
      std::fprintf(stderr, "invalid duration: %s\n", rest[i]);
      return 1;
    } else {
      duration_given = true;
    }
  }

  if (evaluation_window_s < kMinEvaluationWindowS ||
      evaluation_window_s > kMaxEvaluationWindowS) {
    std::fprintf(stderr,
                 "evaluation time %s s is outside the reviewed "
                 "[%.0f, %.0f] s range\n",
                 evaluation_window_token.c_str(), kMinEvaluationWindowS,
                 kMaxEvaluationWindowS);
    return 1;
  }
  if (feel_mode && evaluation_window_given) {
    std::fprintf(stderr,
                 "--evaluation-time is not used with --feel; set the "
                 "feel session duration instead\n");
    return 1;
  }
  const bool demonstration_mode =
      !feel_mode && evaluation_window_s != kDefaultEvaluationWindowS;

  // Normal mode needs room for the marker deadline plus the selected
  // evaluation window and margin. The margin guarantees that the outer
  // deadline cannot truncate a window whose marker lands at the deadline's
  // edge. Feel mode runs to its outer deadline and retains its reviewed
  // 15 s minimum. Rejected BEFORE bus contact.
  const double min_duration_s =
      feel_mode ? kMinFeelDurationS
                : kMarkerDeadlineS + evaluation_window_s +
                      kDurationMarginS;
  if (duration_s < min_duration_s) {
    if (feel_mode) {
      std::fprintf(stderr,
                   "duration %.1f s is shorter than the reviewed feel "
                   "session minimum (%.1f s)\n",
                   duration_s, min_duration_s);
    } else {
      std::fprintf(stderr,
                   "duration %.1f s is shorter than the marker deadline "
                   "+ evaluation window + margin (%.1f s minimum) — "
                   "the marker-anchored protocol requires at least "
                   "that\n",
                   duration_s, min_duration_s);
    }
    return 1;
  }
  if (duration_s > kMaxDurationS) {
    std::fprintf(stderr,
                 "duration %.0f s exceeds the reviewed %.0f s bound — "
                 "normal runs end at the selected evaluation time, one "
                 "feel joint per run needs no more, and a torqued "
                 "session with a healthy command stream never trips a "
                 "watchdog\n",
                 duration_s, kMaxDurationS);
    return 1;
  }

  std::FILE* log = nullptr;
  try {
    // EVERY session requires the approved vendor vector BEFORE any
    // bus contact (un-parking decision 2026-07-31): default-scale
    // adoption was DECLINED, so the repo default config remains the
    // known-failed all-1.0 float configuration — an un-parked
    // x7_float must not be runnable against it by accident. (The
    // gate began as feel-only — a review finding on torque held to
    // the outer deadline — and is now mode-independent.)
    {
      const auto probe = hw::Config::load(cli.config_path);
      // Not merely "< 1.0" (review finding: 0.999 is effectively the
      // failed calibration, and 0.5 would under-support): the APPROVED
      // vendor vector, within a fixed tolerance.
      if (const auto mismatch = x7::gravity::calibrationMismatch(probe)) {
        const auto& joint = probe.joints[mismatch->joint];
        std::fprintf(stderr,
                     "x7_float requires the APPROVED vendor "
                     "calibration (%.6f for this servo model); "
                     "joint '%s' carries %.6f — use "
                     "config/crane_x7_vendor_scale.toml\n",
                     mismatch->expected, joint.name.c_str(),
                     mismatch->actual);
        return 1;
      }
    }
    // The log is REQUIRED and opened BEFORE any bus contact
    // (un-parking condition, review finding: "unique --log per
    // attempt" was stated but not enforced): an unlogged hardware
    // attempt and an accidental overwrite are both refused here.
    if (log_path.empty()) {
      std::fprintf(stderr,
                   "--log is REQUIRED: every x7_float attempt records "
                   "telemetry under a unique filename\n");
      return 1;
    }
    log = openFloatLogFile(log_path);
    if (log == nullptr) {
      if (errno == EEXIST) {
        std::fprintf(stderr,
                     "log file %s already exists — never overwritten; "
                     "use a unique --log filename per attempt\n",
                     log_path.c_str());
      } else {
        std::fprintf(stderr, "cannot open log file %s\n",
                     log_path.c_str());
      }
      return 1;
    }
    // POSITION-mode session: activation holds the arm (goals snap to
    // the present posture) — no free-fall instant. The switch to
    // current mode below carries the calibrated gravity preload.
    auto session = x7::openSession(cli, /*operating_mode_override=*/3);
    if (log != nullptr) {
      writeFloatLogHeader(log, session.config, feel_mode,
                          demonstration_mode, duration_s,
                          evaluation_window_s);
    }
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
    if (const auto violation = x7::gravity::startLimitViolation(
            fb, session.arm->softLimitLo(), session.arm->softLimitHi())) {
      std::fprintf(stderr,
                   "joint %zu is at %.3f rad, within its soft-limit "
                   "margin band [%.3f, %.3f] — reposition mid-range "
                   "and rerun\n",
                   violation->joint, violation->position, violation->lower,
                   violation->upper);
      shutdown.run();
      return 1;
    }

    // Calibrated gravity preload from the HELD posture, through THE
    // shared torque boundary (scale applied exactly once).
    const auto preload = x7::gravity::gravityPreload(
        session.config, chain, map, fb);
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
        // "deactivated" only when the verified shutdown actually
        // SUCCEEDED — a false reassurance after a SHUTDOWN FAULT
        // could send the operator toward a still-torqued arm (review
        // finding); on failure the guard has already reported the
        // fault and silenced the bus.
        if (shutdown.run()) {
          std::fprintf(stderr, "the switch was refused before any "
                               "torque-off; the arm has been "
                               "deactivated\n");
        }
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
    FloatLogObserver observer(log, &marker, &session.config, feel_mode,
                              duration_s, evaluation_window_s);
    if (feel_mode) {
      std::printf("FEEL-CHECK session (outer deadline %.0f s) — press "
                  "ENTER while still supporting (deadline %.0f s), "
                  "release on the cue, then back-drive each joint; the "
                  "run lasts until the deadline. After release, press "
                  "ENTER again at each notch — every press is logged "
                  "as a timestamped EVENT MARK\n",
                  duration_s, kMarkerDeadlineS);
    } else {
      std::printf("floating%s (outer deadline %.0f s) — press ENTER "
                  "while still supporting to mark release (deadline "
                  "%.0f s), then release on the cue; the run ends "
                  "%.1f s after the marker\n",
                  demonstration_mode ? " DEMONSTRATION" : "",
                  duration_s, kMarkerDeadlineS, evaluation_window_s);
    }
    const bool ran = arm::run(robot, controller, duration_s, &observer);
    const bool clean = shutdown.run();
    if (log != nullptr) std::fclose(log);
    // Verdict. Acceptance: ONLY a completed evaluation window is
    // success (review finding: a ran-based form could print "done"
    // after a marker whose window the outer deadline truncated).
    // Feel-check: success is the marker followed by the session
    // reaching its outer deadline. Marker timeout and runner failure
    // abort in both modes.
    const bool success_pending =
        feel_mode ? (ran && marker.released && !marker.marker_timeout)
                  : marker.window_complete;
    if (!clean) {
      std::printf("SHUTDOWN FAULT (run %s)\n",
                  success_pending ? "done" : "ABORTED");
      return 1;
    }
    if (marker.marker_timeout) {
      std::printf("NO release marker within %.0f s — aborted test "
                  "(void attempt, arm deactivated)\n",
                  kMarkerDeadlineS);
      return 1;
    }
    if (feel_mode) {
      if (success_pending) {
        std::printf("feel-check session complete (marker at %.2f s; "
                    "NOT acceptance evidence) — done\n",
                    marker.t_released);
        return 0;
      }
      std::printf("ABORTED\n");
      return 1;
    }
    if (marker.window_complete) {
      if (demonstration_mode) {
        std::printf("demonstration window complete (marker at %.2f s; "
                    "NOT acceptance evidence) — done\n",
                    marker.t_released);
      } else {
        std::printf("evaluation window complete (marker at %.2f s) — "
                    "done\n",
                    marker.t_released);
      }
      return 0;
    }
    if (marker.released) {
      std::printf("evaluation window INCOMPLETE (marker at %.2f s, "
                  "run ended before marker + %.1f s) — aborted "
                  "attempt\n",
                  marker.t_released, evaluation_window_s);
      return 1;
    }
    std::printf("ABORTED\n");
    return 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
