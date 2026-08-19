// Gravity-compensation DEMONSTRATION with customizable effective
// torque constants: the arm floats — held against gravity, freely
// back-drivable by hand — while the console compares measured torque
// (current x kt_effective) against the rkChainID_G command each
// second. The SIMPLIFIED sibling of x7_float, which remains the M-GC3
// acceptance instrument: no marker protocol, no evaluation windows,
// no feel mode. This app's log self-labels "demonstration" and is
// NEVER acceptance evidence.
//
// Torque-constant customization (--kt-xm430 / --kt-xm540, [Nm/A]):
// mapped onto command_torque_scale = kt_nominal / kt_effective, so
// every command still flows through THE one calibrated
// torque->current boundary (hw::commandCurrentFromTorque; see
// gravity_demo_common.hpp) and the reviewed config bound [0.5, 1.0]
// binds unchanged — in kt terms [kt_nominal, 2 x kt_nominal]. The
// DEFAULTS are the vendor-empirical constants (2.20 / 3.60 Nm/A),
// i.e. a flagless run reproduces exactly the approved vendor
// calibration; constants below the vendor values command HOTTER
// currents than the approved calibration (the 2026-07-29
// over-compensation incident was excess current) — permitted within
// the bound, but warned prominently.
//
// SAFETY: current mode on real hardware — keep the power cutoff in
// reach (docs/HARDWARE_BRINGUP.md). Startup is the pose-first
// placement pattern proven by x7_float: activate in POSITION mode
// (servo-held, no free-fall instant), refuse a start inside the
// soft-limit margin band, then switch to current mode in place with
// the scale-calibrated gravity currents preloaded, so support flows
// from the first torque-on instant. --log is REQUIRED and created
// EXCLUSIVELY (unique filename per attempt; raw CSVs stay gitignored
// at the repo root and belong in the private archive per
// docs/DATA_ARCHIVE.md).
//
// Usage: x7_gravity_demo [--config path] [--port dev] --log out.csv
//                        [--kt-xm430 Nm_per_A] [--kt-xm540 Nm_per_A]
//                        [seconds]
//   seconds: session length, default 20, bounded to (0, 60] (the
//   reviewed global bound). Press ENTER at any time to end the
//   session early through the verified shutdown.

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "gravity_demo_common.hpp"
#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/hw/command_current.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;

namespace {

constexpr double kDefaultDurationS = 20.0;
// The reviewed GLOBAL upper bound shared with x7_float: a torqued
// session with a healthy command stream never trips a watchdog, and a
// huge duration would reach the runner's floating-to-long cycle
// conversion (review findings there).
constexpr double kMaxDurationS = 60.0;

// Nonblocking check for the stop key (ENTER). Only actual bytes
// count — EOF on a piped stdin must never read as a keypress.
bool pollStopKey() {
  static bool configured = false;
  if (!configured) {
    const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    configured = true;
  }
  char buf[64];
  return read(STDIN_FILENO, buf, sizeof buf) > 0;
}

// Per-cycle telemetry plus the 1 Hz console comparison. tau_model is
// this cycle's g(q) request; tau_meas is present current x
// kt_effective, computed from the bridge's nominal-constant estimate
// as tau / scale. When the arm floats untouched the two agree — an
// agreement that verifies the servo CURRENT LOOP tracking the
// command, not output-shaft torque
// (docs/theory/gravity-compensation.md).
struct DemoObserver : arm::CycleObserver {
  DemoObserver(std::FILE* log, std::vector<double> inv_scale)
      : log_(log), inv_scale_(std::move(inv_scale)) {}

  bool observe(double t, const arm::JointState& state,
               const arm::CommandSnapshot& cmds,
               const arm::JointCommand& cmd,
               const arm::CommandReceipt& receipt) override {
    (void)receipt;
    if (pollStopKey()) {
      user_stop_ = true;
      t_stop_ = t;
      return false;  // main reads user_stop_: a clean early finish
    }
    if (log_ != nullptr) {
      const auto& applied = cmds.applied;
      std::fprintf(log_, "%.4f", t);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::fprintf(
            log_, ",%.6f,%.6f,%.4f,%.4f,%d,%d",
            zVecElemNC(state.q.get(), i), zVecElemNC(state.dq.get(), i),
            zVecElemNC(cmd.tau.get(), i),
            zVecElemNC(state.tau.get(), i) * inv_scale_[i],
            (applied.valid && (applied.flags[i] & arm::kCmdClamped)) ? 1
                                                                     : 0,
            (applied.valid && (applied.flags[i] & arm::kCmdGated)) ? 1
                                                                   : 0);
      }
      std::fprintf(log_, "\n");
    }
    if (t - last_report_ >= 1.0) {
      last_report_ = t;
      std::printf("t=%4.0fs  measured (i x kt_eff) vs model g(q) [Nm]:\n",
                  t);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::printf("  j%d %+6.2f / %+6.2f", i,
                    zVecElemNC(state.tau.get(), i) * inv_scale_[i],
                    zVecElemNC(cmd.tau.get(), i));
      }
      std::printf("\n");
    }
    return true;
  }

  std::FILE* log_;
  std::vector<double> inv_scale_;  // 1/scale = kt_effective/kt_nominal
  double last_report_ = -1.0;
  bool user_stop_ = false;
  double t_stop_ = 0.0;
};

void writeDemoLogHeader(std::FILE* log, const hw::Config& config,
                        const gravity_demo::TorqueConstants& kt,
                        double duration_s) {
  std::fprintf(log, "# app: x7_gravity_demo\n");
  // Self-classification, same contract as x7_float's demonstration
  // windows: this log must never be readable as acceptance evidence.
  std::fprintf(log, "# run_mode: demonstration\n");
  std::fprintf(log, "# duration_s: %.1f\n", duration_s);
  std::fprintf(log, "# kt_nominal_Nm_per_A:");
  for (const auto& joint : config.joints) {
    std::fprintf(log, " %.6f",
                 rtctrl::dxl::torqueConstant(joint.model_number));
  }
  std::fprintf(log, "\n# kt_effective_Nm_per_A:");
  for (const auto& joint : config.joints) {
    std::fprintf(log, " %.6f",
                 gravity_demo::ktFor(joint.model_number, kt));
  }
  std::fprintf(log, "\n# command_torque_scale:");
  for (const auto& joint : config.joints) {
    std::fprintf(log, " %.6f", joint.command_torque_scale);
  }
  std::fprintf(log, "\n");
  std::fprintf(log,
               "# tau_model is the cycle's g(q) torque request [Nm]; "
               "tau_meas is present current x kt_effective [Nm] — a "
               "current-loop agreement check, NOT an output-shaft "
               "measurement; clamped/gated are the limiter flags of "
               "the LATEST APPLIED command at the row's snapshot "
               "(0 before the first application).\n");
  std::fprintf(log, "t");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log, ",q%d,dq%d,tau_model%d,tau_meas%d,clamped%d,gated%d",
                 i, i, i, i, i, i);
  }
  std::fprintf(log, "\n");
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  double duration_s = kDefaultDurationS;
  bool duration_given = false;
  std::string log_path;
  gravity_demo::TorqueConstants kt;
  const auto& rest = cli.rest;
  for (std::size_t i = 0; i < rest.size(); ++i) {
    if (std::strcmp(rest[i], "--log") == 0) {
      // A flag-looking next token is a MISSING value, not a filename
      // (x7_float review finding); ./--name spells a flag-like name.
      if (i + 1 >= rest.size() ||
          std::strncmp(rest[i + 1], "--", 2) == 0) {
        std::fprintf(stderr, "--log requires a value\n");
        return 1;
      }
      log_path = rest[++i];
    } else if (std::strcmp(rest[i], "--kt-xm430") == 0 ||
               std::strcmp(rest[i], "--kt-xm540") == 0) {
      const char* flag = rest[i];
      if (i + 1 >= rest.size() ||
          std::strncmp(rest[i + 1], "--", 2) == 0) {
        std::fprintf(stderr, "%s requires a value\n", flag);
        return 1;
      }
      double value = 0.0;
      if (!x7::parseStrictDouble(rest[++i], &value)) {
        std::fprintf(stderr, "invalid torque constant: %s\n", rest[i]);
        return 1;
      }
      (std::strcmp(flag, "--kt-xm430") == 0 ? kt.kt_xm430
                                            : kt.kt_xm540) = value;
    } else if (rest[i][0] == '-' && rest[i][1] == '-') {
      std::fprintf(stderr, "unknown argument: %s\n", rest[i]);
      return 1;
    } else if (duration_given) {
      std::fprintf(stderr,
                   "usage: x7_gravity_demo [--config path] [--port dev] "
                   "--log out.csv [--kt-xm430 Nm_per_A] "
                   "[--kt-xm540 Nm_per_A] [seconds]\n");
      return 1;
    } else if (!x7::parseStrictDouble(rest[i], &duration_s)) {
      std::fprintf(stderr, "invalid duration: %s\n", rest[i]);
      return 1;
    } else {
      duration_given = true;
    }
  }

  if (duration_s <= 0.0) {
    std::fprintf(stderr, "duration must be positive\n");
    return 1;
  }
  if (duration_s > kMaxDurationS) {
    std::fprintf(stderr,
                 "duration %.0f s exceeds the reviewed %.0f s bound — "
                 "a demonstration needs no more\n",
                 duration_s, kMaxDurationS);
    return 1;
  }
  // kt-denominated refusal BEFORE the config backstop runs, so the
  // message speaks the operator's units.
  const struct {
    const char* name;
    std::uint16_t model;
    double kt;
    double vendor;
  } kt_checks[] = {
      {"XM430-W350", rtctrl::dxl::kModelXm430W350, kt.kt_xm430,
       gravity_demo::kVendorKtXm430},
      {"XM540-W270", rtctrl::dxl::kModelXm540W270, kt.kt_xm540,
       gravity_demo::kVendorKtXm540},
  };
  for (const auto& check : kt_checks) {
    // negated form: NaN must land in the refusal branch
    if (!(check.kt >= gravity_demo::ktMin(check.model) &&
          check.kt <= gravity_demo::ktMax(check.model))) {
      std::fprintf(stderr,
                   "kt %.3f Nm/A for the %s is outside [%.3f, %.3f] — "
                   "the reviewed command-scale bound [0.5, 1.0] "
                   "expressed as an effective torque constant (below "
                   "nominal would OVER-compensate; above 2x nominal "
                   "under-supports)\n",
                   check.kt, check.name, gravity_demo::ktMin(check.model),
                   gravity_demo::ktMax(check.model));
      return 1;
    }
    if (check.kt < check.vendor) {
      std::printf("WARNING: kt %.3f Nm/A for the %s is below the "
                  "vendor-calibrated %.2f — every torque command runs "
                  "HOTTER than the approved calibration (the 2026-07-29 "
                  "incident class was excess current); keep the power "
                  "cutoff in hand\n",
                  check.kt, check.name, check.vendor);
    }
  }

  std::FILE* log = nullptr;
  try {
    // Refusals before ANY device open: the REQUIRED log path, the
    // customized config (probe load; the session reloads and
    // re-applies), then the exclusive log creation — an existing file
    // is refused, never overwritten (unique filename per attempt).
    if (log_path.empty()) {
      std::fprintf(stderr,
                   "--log is REQUIRED: every hardware attempt records "
                   "telemetry under a unique filename\n");
      return 1;
    }
    const auto customize = [&kt](hw::Config& config) {
      gravity_demo::applyTorqueConstants(config, kt);
    };
    {
      auto probe = hw::Config::load(cli.config_path);
      customize(probe);
      probe.validate();
    }
    log = std::fopen(log_path.c_str(), "wx");
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
    auto session = x7::openSession(cli, /*operating_mode_override=*/3,
                                   customize);
    std::printf("torque constants (kt_nominal -> kt_effective, scale):\n");
    for (const auto& joint : session.config.joints) {
      std::printf("  %-45s %5.3f -> %5.3f  (scale %.6f)\n",
                  joint.name.c_str(),
                  rtctrl::dxl::torqueConstant(joint.model_number),
                  gravity_demo::ktFor(joint.model_number, kt),
                  joint.command_torque_scale);
    }
    writeDemoLogHeader(log, session.config, kt, duration_s);

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
    // Refuse a float from inside the soft-limit margin band (as
    // x7_float does): the current gate would cut the gravity support
    // in one whole direction there.
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
      // Two distinct hardware states: a PRE-sequence refusal leaves
      // the arm ACTIVE and HELD in position mode — deactivate and
      // verify; a mid-sequence failure has already best-effort
      // released it.
      if (session.arm->activated()) {
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

    arm::RealArm robot(*session.arm);
    if (!robot.activate()) {  // already active: starts the thread,
                              // which retransmits the preload
      std::fprintf(stderr, "thread start failed: %s\n",
                   session.arm->lastError().c_str());
      shutdown.run();
      return 1;
    }
    std::vector<double> inv_scale(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      inv_scale[i] = 1.0 / session.config.joints[i].command_torque_scale;
    }
    arm::GravityComp controller(chain, map);
    DemoObserver observer(log, std::move(inv_scale));
    std::printf("floating DEMONSTRATION (%.0f s) — the arm is supported "
                "by the calibrated gravity currents; back-drive it "
                "gently; press ENTER to end the session early\n",
                duration_s);
    const bool ran = arm::run(robot, controller, duration_s, &observer);
    const bool clean = shutdown.run();
    if (log != nullptr) std::fclose(log);
    // Verdict: a completed session or the operator's stop are the two
    // clean ends; anything else aborted. SHUTDOWN FAULT dominates —
    // success text must never print over an unverified shutdown.
    const bool success = ran || observer.user_stop_;
    if (!clean) {
      std::printf("SHUTDOWN FAULT (run %s)\n",
                  success ? "done" : "ABORTED");
      return 1;
    }
    if (observer.user_stop_) {
      std::printf("stopped by operator (t=%.2f s) — demonstration "
                  "complete (NOT acceptance evidence)\n",
                  observer.t_stop_);
      return 0;
    }
    if (ran) {
      std::printf("demonstration complete (%.0f s; NOT acceptance "
                  "evidence) — done\n",
                  duration_s);
      return 0;
    }
    std::printf("ABORTED\n");
    return 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
