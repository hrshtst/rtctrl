// Manual motion teaching for CRANE-X7.
// Usage: x7_teach --config FILE [--port DEV]
//                 [--mode torque-off|gravity-compensation]
//                 [--sample-rate HZ] [--max-duration SEC]
//                 [--output FILE] [--log FILE] [--bundle DIR] [--check]

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "common/periodic_loop.hpp"
#include "common/x7_common.hpp"
#include "gravity/gravity_support.hpp"
#include "plan/ptp_bundle.hpp"
#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/version.hpp"
#include "teach/teach_bundle.hpp"
#include "teach/teach_config.hpp"
#include "teach/teach_recording.hpp"

namespace x7::gitrev {
extern const char* const kBuildSha;
extern const bool kBuildDirty;
}  // namespace x7::gitrev

namespace arm = rtctrl::arm;
namespace dxl = rtctrl::dxl;
namespace gravity = x7::gravity;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;
namespace teach = x7::teach;

namespace {

void usage() {
  std::printf(
      "usage: x7_teach --config FILE [options]\n"
      "  --port DEV\n"
      "  --mode torque-off|gravity-compensation\n"
      "  --sample-rate HZ\n"
      "  --max-duration SEC\n"
      "  --output FILE\n"
      "  --log FILE\n"
      "  --bundle NEW_DIRECTORY\n"
      "  --check\n");
}

class EnterReader {
 public:
  EnterReader() {
    original_flags_ = ::fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ >= 0) {
      enabled_ =
          ::fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) == 0;
    }
  }

  EnterReader(const EnterReader&) = delete;
  EnterReader& operator=(const EnterReader&) = delete;

  ~EnterReader() {
    if (enabled_) ::fcntl(STDIN_FILENO, F_SETFL, original_flags_);
  }

  bool ready() const { return enabled_; }

  bool pressed() {
    if (!enabled_) return false;
    char bytes[64];
    bool received = false;
    while (::read(STDIN_FILENO, bytes, sizeof(bytes)) > 0) received = true;
    if (first_poll_) {
      first_poll_ = false;
      return false;
    }
    return received;
  }

 private:
  int original_flags_ = -1;
  bool enabled_ = false;
  bool first_poll_ = true;
};

struct RunResult {
  std::string status = "control_failure";
  bool recording_complete = false;
  bool clean_shutdown = false;
  std::uint64_t raw_samples = 0;
  double duration_s = 0.0;
  teach::MotionRecorder recording;
};

teach::JointArray positions(const std::vector<dxl::Feedback>& feedback) {
  teach::JointArray q{};
  for (int i = 0; i < model::kCanonicalDof; ++i) q[i] = feedback[i].position;
  return q;
}

teach::LogRow passiveRow(double session_time, double recording_time,
                         std::uint64_t seq, teach::Phase phase,
                         teach::Event event,
                         const std::vector<dxl::Feedback>& feedback,
                         const hw::Config& config) {
  teach::LogRow row;
  row.session_time_s = session_time;
  row.recording_time_s = recording_time;
  row.feedback_time_s = session_time;
  row.feedback_seq = seq;
  row.phase = phase;
  row.event = event;
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    row.q[i] = feedback[i].position;
    row.dq[i] = feedback[i].velocity;
    row.tau[i] = feedback[i].current *
                 dxl::torqueConstant(config.joints[i].model_number);
  }
  return row;
}

RunResult runPassive(const teach::Config& config, const hw::Config& hardware,
                     teach::TeachCsvLog* log, EnterReader* enter) {
  RunResult result;
  rtctrl::dxl::Port port(hardware.port, hardware.baudrate);
  hw::CraneX7 crane(port, hardware);
  if (!crane.preparePassiveFeedback()) {
    result.status = "passive_setup_failed";
    return result;
  }

  std::printf("torque is OFF; support and guide the arm, then press Enter "
              "to start recording (timeout %.1f s)\n",
              config.recording.start_timeout_s);
  x7::PeriodicLoop loop(1.0 / teach::kHardwareRateHz);
  teach::Phase phase = teach::Phase::AwaitStart;
  double recording_start = 0.0;
  double previous_time = -1.0;
  std::uint64_t seq = 0;
  while (true) {
    std::vector<dxl::Feedback> feedback;
    if (!crane.readPassiveFeedback(feedback) ||
        feedback.size() != model::kCanonicalDof) {
      result.status = "feedback_failure";
      break;
    }
    const double session_time = loop.elapsed();
    if (previous_time >= 0.0 && session_time - previous_time > 0.025) {
      result.status = "feedback_gap";
      break;
    }
    previous_time = session_time;
    ++seq;
    ++result.raw_samples;
    teach::Event event = teach::Event::None;
    const auto row_phase = phase;
    bool stop = false;
    double recording_time = -1.0;
    const bool pressed = enter->pressed();
    if (phase == teach::Phase::AwaitStart) {
      if (pressed) {
        phase = teach::Phase::Recording;
        recording_start = session_time;
        recording_time = 0.0;
        event = teach::Event::Start;
        result.recording.append(0.0, positions(feedback));
        std::printf("RECORDING — guide the arm; press Enter to stop "
                    "(automatic stop after %.1f s)\n",
                    config.recording.max_duration_s);
      } else if (session_time >= config.recording.start_timeout_s) {
        result.status = "start_timeout";
        stop = true;
      }
    } else {
      recording_time = session_time - recording_start;
      result.recording.append(recording_time, positions(feedback));
      if (pressed) {
        event = teach::Event::Stop;
        result.status = "success";
        result.recording_complete = true;
        stop = true;
      } else if (recording_time >= config.recording.max_duration_s) {
        event = teach::Event::DurationStop;
        result.status = "success";
        result.recording_complete = true;
        stop = true;
      }
    }
    auto row = passiveRow(session_time, recording_time, seq, row_phase, event,
                          feedback, hardware);
    log->row(row, config.mode);
    if (stop) break;
    loop.waitNext();
  }
  if (!crane.preparePassiveFeedback()) {
    result.status = "torque_off_verification_failed";
    result.recording_complete = false;
  } else {
    result.clean_shutdown = true;
  }
  if (!result.recording.samples().empty()) {
    result.duration_s = result.recording.samples().back().time_s;
  }
  if (loop.skippedPeriods() > 0) {
    std::fprintf(stderr,
                 "warning: passive acquisition skipped %llu period(s), "
                 "maximum lateness %.3f ms\n",
                 static_cast<unsigned long long>(loop.skippedPeriods()),
                 1e3 * loop.maxLateness());
  }
  return result;
}

class GravityTeachObserver : public arm::CycleObserver {
 public:
  GravityTeachObserver(const teach::Config& config, teach::TeachCsvLog* log,
                       EnterReader* enter, RunResult* result)
      : config_(config), log_(log), enter_(enter), result_(result) {}

  bool observe(double t, const arm::JointState& state,
               const arm::CommandSnapshot& commands,
               const arm::JointCommand& command,
               const arm::CommandReceipt& receipt) override {
    teach::Event event = teach::Event::None;
    const auto row_phase = phase_;
    bool keep_running = true;
    double recording_time = -1.0;
    const bool pressed = enter_->pressed();
    const auto q = statePositions(state);
    if (phase_ == teach::Phase::AwaitStart) {
      if (pressed) {
        phase_ = teach::Phase::Recording;
        recording_start_ = t;
        recording_time = 0.0;
        event = teach::Event::Start;
        result_->recording.append(0.0, q);
        std::printf("RECORDING — release and guide the arm NOW; press Enter "
                    "to stop (automatic stop after %.1f s)\n",
                    config_.recording.max_duration_s);
      } else if (t >= config_.recording.start_timeout_s) {
        result_->status = "start_timeout";
        keep_running = false;
      }
    } else if (phase_ == teach::Phase::Recording) {
      recording_time = t - recording_start_;
      result_->recording.append(recording_time, q);
      if (pressed || recording_time >= config_.recording.max_duration_s) {
        event = pressed ? teach::Event::Stop : teach::Event::DurationStop;
        result_->recording_complete = true;
        result_->duration_s = recording_time;
        phase_ = teach::Phase::AwaitSupport;
        support_start_ = t;
        std::printf("recording stopped; gravity compensation remains active. "
                    "Support the arm from below, then press Enter to disable "
                    "torque (timeout %.1f s)\n",
                    config_.finalization.operator_timeout_s);
      }
    } else {
      if (pressed) {
        event = teach::Event::Support;
        result_->status = "success";
        keep_running = false;
      } else if (t - support_start_ >=
                 config_.finalization.operator_timeout_s) {
        result_->status = "operator_timeout";
        keep_running = false;
      }
    }
    log_->row(makeRow(t, recording_time, row_phase, event, state, commands,
                      command, receipt),
              config_.mode);
    ++result_->raw_samples;
    return keep_running;
  }

 private:
  static teach::JointArray statePositions(const arm::JointState& state) {
    teach::JointArray q{};
    for (int i = 0; i < model::kCanonicalDof; ++i) q[i] = state.q[i];
    return q;
  }

  static teach::LogRow makeRow(double t, double recording_time,
                               teach::Phase phase, teach::Event event,
                               const arm::JointState& state,
                               const arm::CommandSnapshot& commands,
                               const arm::JointCommand& command,
                               const arm::CommandReceipt& receipt) {
    teach::LogRow row;
    row.session_time_s = t;
    row.recording_time_s = recording_time;
    row.feedback_time_s = state.t;
    row.feedback_seq = state.seq;
    row.phase = phase;
    row.event = event;
    row.command_valid = true;
    row.submitted_seq = receipt.submitted_seq;
    row.submission_time_s = receipt.submission_time;
    row.receipt_accepted = receipt.accepted;
    row.applied_valid = commands.applied.valid;
    row.applied_seq = commands.applied.target_seq;
    row.latest_apply_time_s = commands.applied.latest_time;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      row.q[i] = state.q[i];
      row.dq[i] = state.dq[i];
      row.tau[i] = state.tau[i];
      row.tau_request[i] = command.tau[i];
      if (commands.applied.valid) {
        row.tau_applied[i] = commands.applied.applied[i];
        row.clamped[i] =
            (commands.applied.flags[i] & arm::kCmdClamped) ? 1 : 0;
        row.gated[i] =
            (commands.applied.flags[i] & arm::kCmdGated) ? 1 : 0;
      }
    }
    return row;
  }

  const teach::Config& config_;
  teach::TeachCsvLog* log_;
  EnterReader* enter_;
  RunResult* result_;
  teach::Phase phase_ = teach::Phase::AwaitStart;
  double recording_start_ = 0.0;
  double support_start_ = 0.0;
};

RunResult runGravity(const teach::Config& config, hw::Config hardware,
                     model::ChainModel* chain, const model::JointMap& map,
                     teach::TeachCsvLog* log, EnterReader* enter) {
  RunResult result;
  for (auto& joint : hardware.joints) joint.operating_mode = 3;
  rtctrl::dxl::Port port(hardware.port, hardware.baudrate);
  hw::CraneX7 crane(port, hardware);
  crane.onEscalate([&port] {
    std::fprintf(stderr,
                 "DEADMAN: command stream stale; closing the bus so servo "
                 "watchdogs halt the arm\n");
    port.close();
  });
  const auto torque_session_start = std::chrono::steady_clock::now();
  if (!crane.activate()) {
    result.status = "position_activation_failed";
    return result;
  }
  x7::ShutdownGuard shutdown(crane);
  const auto held = crane.lastFeedback();
  if (held.size() != model::kCanonicalDof) {
    result.status = "held_posture_read_failed";
    result.clean_shutdown = shutdown.run();
    return result;
  }
  if (const auto violation = gravity::startLimitViolation(
          held, crane.softLimitLo(), crane.softLimitHi())) {
    std::fprintf(stderr,
                 "joint %zu is at %.3f rad, within its soft-limit margin "
                 "band [%.3f, %.3f]; reposition mid-range and rerun\n",
                 violation->joint, violation->position, violation->lower,
                 violation->upper);
    result.status = "soft_limit_start";
    result.clean_shutdown = shutdown.run();
    return result;
  }
  const auto preload = gravity::gravityPreload(hardware, *chain, map, held);
  std::printf("switching to current mode in place with calibrated gravity "
              "preload...\n");
  if (!crane.switchToCurrentModeWithPreload(preload)) {
    std::fprintf(stderr, "mode switch failed: %s\n",
                 crane.lastError().c_str());
    result.status = "mode_switch_failed";
    result.clean_shutdown = crane.activated() ? shutdown.run() : true;
    return result;
  }
  arm::RealArm robot(crane);
  if (!robot.activate()) {
    result.status = "thread_start_failed";
    result.clean_shutdown = shutdown.run();
    return result;
  }
  arm::GravityComp controller(*chain, map);
  GravityTeachObserver observer(config, log, enter, &result);
  std::printf("gravity compensation ACTIVE; keep supporting the arm and press "
              "Enter to start recording (timeout %.1f s)\n",
              config.recording.start_timeout_s);
  const double setup_time = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() -
                                torque_session_start)
                                .count();
  const double remaining_session = teach::kMaxGravitySessionS - setup_time;
  const bool ran = remaining_session > 0.0 &&
                   arm::run(robot, controller, remaining_session, &observer);
  if ((remaining_session <= 0.0 || ran) &&
      result.status == "control_failure") {
    result.status = "session_deadline";
  }
  result.clean_shutdown = shutdown.run();
  if (!result.clean_shutdown) result.status = "shutdown_fault";
  return result;
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto cli = teach::parseCli(argc, argv);
    if (cli.help) {
      usage();
      return 0;
    }
    std::unique_ptr<x7::ptp::BundleWorkspace> bundle;
    if (cli.bundle_path) {
      bundle = std::make_unique<x7::ptp::BundleWorkspace>(*cli.bundle_path);
    }
    auto config = teach::loadConfig(cli.config_path);
    teach::applyOverrides(cli, &config);
    if (bundle) {
      const auto prepared = teach::prepareBundle(bundle->staging(), config);
      config = teach::loadConfig(prepared.config_path);
      config.output.motion = prepared.motion_path;
      config.output.log = prepared.log_path;
    }

    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    auto hardware = hw::Config::load(config.hardware_config_path.string());
    if (cli.port) hardware.port = *cli.port;
    if (config.mode == teach::Mode::GravityCompensation) {
      if (const auto mismatch = gravity::calibrationMismatch(hardware)) {
        const auto& joint = hardware.joints[mismatch->joint];
        throw std::runtime_error(
            "gravity compensation requires the approved vendor scale for " +
            joint.name + "; use config/crane_x7_vendor_scale.toml");
      }
    }
    if (cli.check) {
      std::printf("configuration, model, and hardware preflight passed\n");
      return 0;
    }

    teach::requireNewOutput(config.output.motion, "motion output");
    teach::requireNewOutput(config.output.log, "teach log");
    std::printf("bus: %s @ %d baud, %zu joints\n", hardware.port.c_str(),
                hardware.baudrate, hardware.joints.size());
    EnterReader enter;
    if (!enter.ready()) {
      throw std::runtime_error("cannot make stdin nonblocking for teaching");
    }

    RunResult result;
    int output_frames = 0;
    {
      teach::ExclusiveZvsOutput motion(config.output.motion);
      teach::TeachCsvLog log(config.output.log, config.mode,
                             config.recording.sample_rate_hz);
      if (config.mode == teach::Mode::TorqueOff) {
        result = runPassive(config, hardware, &log, &enter);
      } else {
        result = runGravity(config, hardware, &chain, map, &log, &enter);
      }
      if (result.recording_complete && result.clean_shutdown) {
        try {
          const auto samples = teach::uniformResample(
              result.recording.samples(), config.recording.sample_rate_hz);
          output_frames = motion.write(samples,
                                       config.recording.sample_rate_hz, map);
        } catch (const std::exception& error) {
          std::fprintf(stderr, "recorded motion rejected: %s\n",
                       error.what());
          result.status = "recording_output_failed";
          result.recording_complete = false;
        }
      }
      log.finish(result.status);
    }

    std::string display_motion = config.output.motion.string();
    std::string display_log = config.output.log.string();
    if (bundle) {
      teach::writeBundleResult(
          bundle->staging(), config.mode, result.status, result.raw_samples,
          static_cast<std::uint64_t>(output_frames), result.duration_s);
      x7::ptp::writeBundleManifestFor(
          bundle->staging(), "rtctrl-x7-teach-bundle", 1, rtctrl::version(),
          x7::gitrev::kBuildSha, x7::gitrev::kBuildDirty);
      bundle->publish();
      display_motion = (bundle->target() / "trajectory.zvs").string();
      display_log = (bundle->target() / "recording.csv").string();
      std::printf("created bundle %s\n", bundle->target().string().c_str());
    }
    std::printf("teach %s: %s, %llu raw samples, %d output frames, %.3f s\n",
                teach::modeName(config.mode), result.status.c_str(),
                static_cast<unsigned long long>(result.raw_samples),
                output_frames, result.duration_s);
    std::printf("motion: %s\nlog: %s\n", display_motion.c_str(),
                display_log.c_str());
    if (output_frames > 0) {
      const auto display_model =
          bundle ? bundle->target() / "model" / config.model_path.filename()
                 : config.model_path;
      std::printf("view: rk_anim %s %s\n", display_model.string().c_str(),
                  display_motion.c_str());
    }
    return result.status == "success" && output_frames >= 2 ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_teach: %s\n", error.what());
    return 1;
  }
}
