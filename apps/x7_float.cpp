// M7 hardware phase: gravity compensation on the real arm. The arm
// floats — held against gravity, freely back-drivable by hand — while
// the console compares measured torque (current × torque constant)
// against the rkChainID_G prediction each second.
//
// SAFETY: current mode. Keep the power cutoff in reach; support the
// arm lightly on the first run. Verified in sim first
// (gravity_sim_test): drift < 0.05 rad over 10 s.
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
//   column (negative = the apply happened after the feedback
//   timestamp) to order the events before comparing torques. Raw
//   hardware CSVs land at the repo root gitignored and belong in the
//   private archive (docs/DATA_ARCHIVE.md); use a unique filename per
//   attempt.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
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

// Per-cycle telemetry via the CycleObserver — invoked AFTER
// writeCommand, so the row carries this cycle's receipt (was the
// request accepted?) and the pre-write snapshot's latest-applied
// record, keeping the three events distinguishable (review finding:
// logging inside update() recorded a request before knowing whether
// it was ever accepted or applied).
struct FloatLogObserver : arm::CycleObserver {
  explicit FloatLogObserver(std::FILE* log) : log_(log) {}

  bool observe(double t, const arm::JointState& state,
               const arm::CommandSnapshot& cmds,
               const arm::JointCommand& cmd,
               const arm::CommandReceipt& receipt) override {
    if (log_ == nullptr) return true;
    const auto& applied = cmds.applied;
    // Timing fields as in x7_track's log: the latest apply may
    // POSTDATE this row's feedback (the producer reads before it
    // writes), so the offset is signed; both are meaningful only when
    // applied_valid = 1.
    std::fprintf(log_, "%.4f,%.6f,%llu,%llu,%.6f,%d,%d,%llu,%.6f,%.6f",
                 t, state.t, static_cast<unsigned long long>(state.seq),
                 static_cast<unsigned long long>(receipt.submitted_seq),
                 receipt.submission_time, receipt.accepted ? 1 : 0,
                 applied.valid ? 1 : 0,
                 static_cast<unsigned long long>(applied.target_seq),
                 applied.valid ? applied.latest_time : 0.0,
                 applied.valid ? state.t - applied.latest_time : 0.0);
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
    return true;
  }

  std::FILE* log_;
};

std::FILE* openFloatLog(const std::string& path) {
  std::FILE* log = std::fopen(path.c_str(), "w");
  if (log == nullptr) return nullptr;
  std::fprintf(log,
               "# events per row: FEEDBACK (feedback_time/feedback_seq, "
               "q, dqservo, tau_meas) | THIS CYCLE'S REQUEST "
               "(submitted_seq/submission_time/accepted, tau_request — "
               "applied LATER) | LATEST APPLIED in the coherent "
               "snapshot (applied_valid/applied_seq/latest_apply_time, "
               "tau_applied, clamped/gated). feedback_minus_latest_apply "
               "is SIGNED: the producer reads before it writes, so the "
               "latest apply may post-date this row's feedback; applied "
               "fields are meaningful only when applied_valid=1.\n");
  std::fprintf(log, "t,feedback_time,feedback_seq,submitted_seq,"
                    "submission_time,accepted,applied_valid,applied_seq,"
                    "latest_apply_time,feedback_minus_latest_apply");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log,
                 ",q%d,dqservo%d,tau_meas%d,tau_request%d,tau_applied%d,"
                 "clamped%d,gated%d",
                 i, i, i, i, i, i, i);
  }
  std::fprintf(log, "\n");
  return log;
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
  // ever torquing the arm.
  std::FILE* log = nullptr;
  if (!log_path.empty()) {
    log = openFloatLog(log_path);
    if (log == nullptr) {
      std::fprintf(stderr, "cannot open log file %s\n", log_path.c_str());
      return 1;
    }
  }

  try {
    // gravity compensation runs in current (torque) mode
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
    ReportingGravityComp controller(chain, map);
    FloatLogObserver observer(log);
    std::printf("floating for %.0f s — the arm is back-drivable; keep the "
                "power cutoff in reach\n",
                duration_s);
    const bool ok = arm::run(robot, controller, duration_s,
                             log != nullptr ? &observer : nullptr);
    const bool clean = shutdown.run();
    if (log != nullptr) std::fclose(log);
    if (!clean) {
      std::printf("SHUTDOWN FAULT (run %s)\n", ok ? "done" : "ABORTED");
      return 1;
    }
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
