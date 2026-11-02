// Hardware frontend for servo-side trajectory following.
// Usage: x7_follow --config run.toml [input/control overrides]
//                  [--port DEV] [--log out.csv] [--timing-log cycles.csv]
//                  [--hold-current S | --sweep-current J:A:D]
//                  [--bundle DIR] [--check]

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>

#include "bus/dxl_parameters.hpp"
#include "common/link_force_bundle.hpp"
#include "common/timing_log.hpp"
#include "common/x7_common.hpp"
#include "follow/follow_config.hpp"
#include "follow/follow_bundle.hpp"
#include "follow/follow_hardware.hpp"
#include "follow/follow_preflight.hpp"
#include "follow/follow_run.hpp"
#include "follow/posture_reference.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/version.hpp"

namespace x7::gitrev {
extern const char* const kBuildSha;
extern const bool kBuildDirty;
}  // namespace x7::gitrev

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;
namespace parameters = rtctrl::apps::dxl_parameters;

namespace {

void usage() {
  std::printf(
      "usage: x7_follow --config FILE [options]\n"
      "  --reference FILE\n"
      "  --mode position|current-based-position\n"
      "  --motor-parameters FILE\n"
      "  --effort-limit-nm VALUE\n"
      "  --filter none|low-pass|moving-average|savitzky-golay\n"
      "  --interpolation linear|shape-preserving-cubic\n"
      "  --port DEV\n"
      "  --log FILE\n"
      "  --timing-log FILE    per-cycle host timing (hardware only)\n"
      "  --hold-current SECONDS   hold the measured posture (no homing)\n"
      "  --sweep-current J:A:D    sweep joint J by A rad and back over D s\n"
      "                           from the measured posture (no homing)\n"
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

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto cli = follow::parseCli(argc, argv, false);
    if (cli.help) {
      usage();
      return 0;
    }
    std::unique_ptr<x7::ptp::BundleWorkspace> bundle;
    if (cli.bundle_path) {
      bundle = std::make_unique<x7::ptp::BundleWorkspace>(*cli.bundle_path);
    }
    auto config = follow::loadConfig(cli.config_path);
    follow::applyOverrides(cli, &config);
    if (config.simulation.link_force_path) {
      std::fprintf(stderr, "%s\n", x7::kHardwareLinkForceNotice);
    }
    if (cli.log_path) config.output.hardware_log = *cli.log_path;
    if (bundle) {
      const auto prepared = follow::prepareBundle(bundle->staging(), config);
      config = follow::loadConfig(prepared.config_path);
      config.output.hardware_log = prepared.hardware_log;
    }

    std::optional<follow::PostureReference> posture;
    if (cli.hold_current) {
      posture = follow::parseHoldCurrent(*cli.hold_current);
    } else if (cli.sweep_current) {
      posture = follow::parseSweepCurrent(*cli.sweep_current);
    }
    if (posture && config.reference.filter != model::ReferenceFilter::None) {
      throw std::runtime_error(
          "a reference built from the measured posture must start exactly "
          "there: use reference_processing.filter = none (--filter none)");
    }

    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    // A posture reference is built and validated after the posture is
    // read, still before torque-on.
    std::unique_ptr<model::ZvsTrajectory> reference;
    if (!posture) {
      reference = std::make_unique<model::ZvsTrajectory>(
          config.reference_path.string(), map, config.reference);
    }
    auto hardware = hw::Config::load(config.hardware_config_path.string());
    if (reference) {
      follow::validateReference(config, chain, map, *reference, hardware);
    }

    std::optional<parameters::ParameterDump> parameter_dump;
    if (config.motor_parameters_path) {
      parameter_dump =
          parameters::parseFile(config.motor_parameters_path->string());
      follow::validateMotorParameters(*parameter_dump, hardware);
    }
    follow::selectHardwareMode(&hardware, config.mode, cli.port);
    if (config.control_rate_hz != 100.0) {
      std::fprintf(stderr,
                   "warning: hardware timing is validated at 100 Hz; %.1f "
                   "Hz is experimental\n",
                   config.control_rate_hz);
    }
    if (cli.check) {
      std::printf("configuration and reference preflight passed\n");
      return 0;
    }
    follow::requireNewOutput(config.output.hardware_log, "hardware log");
    if (cli.timing_log_path) {
      follow::requireNewOutput(*cli.timing_log_path, "timing log");
    }
    const std::filesystem::path posture_path =
        config.output.hardware_log.string() + ".reference.zvs";
    if (posture) follow::requireNewOutput(posture_path, "posture reference");
    follow::RunResult result;
    hw::CraneX7::CycleStats stats;
    bool released = false;
    bool log_ok = false;
    bool timing_ok = true;
    {
      EnterReader enter;
      if (config.finalization.wait_time_s == 0.0 && !enter.ready()) {
        throw std::runtime_error(
            "cannot make stdin nonblocking for the final Enter hold");
      }
      follow::FollowCsvLog log(config.output.hardware_log.string());
      std::printf("bus: %s @ %d baud, %zu joints\n", hardware.port.c_str(),
                  hardware.baudrate, hardware.joints.size());
      rtctrl::dxl::Port port(hardware.port, hardware.baudrate);
      auto options = follow::hardwareOptions(config, parameter_dump);
      if (cli.timing_log_path) {
        options.timing_ring_capacity =
            x7::timingRingCapacity(config.control_rate_hz);
      }
      hw::CraneX7 crane(port, hardware, options);
      crane.onEscalate([&port] {
        std::fprintf(
            stderr,
            "DEADMAN: command stream stale; closing the bus so servo "
            "watchdogs halt the arm\n");
        port.close();
      });
      const auto cbp_limits =
          follow::currentBasedPositionLimits(config, hardware);
      if (!cbp_limits.empty()) {
        crane.setActivationCurrentBasedPositionLimits(cbp_limits);
      }
      if (posture) {
        // Torque-off read of the posture the arm is resting in.
        std::vector<rtctrl::dxl::Feedback> fb;
        if (!crane.preparePassiveFeedback() ||
            !crane.readPassiveFeedback(fb) ||
            fb.size() != static_cast<std::size_t>(model::kCanonicalDof)) {
          throw std::runtime_error("posture read failed: " +
                                   crane.lastError());
        }
        follow::CanonicalPosture q0{};
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          q0[i] = fb[i].position;
        }
        follow::writePostureReferenceZvs(
            posture_path, follow::postureReferenceFrames(*posture, q0));
        reference = std::make_unique<model::ZvsTrajectory>(
            posture_path.string(), map, config.reference);
        follow::validateReference(config, chain, map, *reference, hardware);
        config.home.max_distance_rad = follow::kPostureMatchToleranceRad;
        std::printf("posture reference: %s (%s)\n",
                    follow::describePostureReference(*posture).c_str(),
                    posture_path.c_str());
      }

      arm::RealArm robot(crane);
      if (!robot.setMode(config.mode)) {
        throw std::runtime_error("hardware operating-mode setup mismatch");
      }
      std::unique_ptr<x7::CycleTimingLog> timing_log;
      if (cli.timing_log_path) {
        timing_log = std::make_unique<x7::CycleTimingLog>(
            crane, cli.timing_log_path->string());
      }
      if (!robot.activate()) {
        throw std::runtime_error("activation failed: " + crane.lastError());
      }
      x7::ShutdownGuard shutdown(crane);
      if (timing_log) timing_log->start();
      follow::FollowRun run(robot, *reference, config, false, &log,
                            [&enter] { return enter.pressed(); });
      result = run.run();
      stats = crane.cycleStats();
      released = shutdown.run();
      // Drains the queued rows: file I/O only after the arm is released.
      log_ok = log.close();
      if (timing_log) timing_ok = timing_log->stop();
    }
    std::string display_log = config.output.hardware_log.string();
    if (bundle) {
      follow::writeBundleResult(
          bundle->staging(), "hardware", follow::statusName(result.status),
          result.cycles, result.worst_home_error_rad,
          result.worst_tracking_error_rad,
          config.simulation.link_force_path ? x7::hardwareLinkForceProvenance()
                                            : std::string());
      x7::ptp::writeBundleManifestFor(
          bundle->staging(), "rtctrl-x7-follow-bundle", 1,
          rtctrl::version(), x7::gitrev::kBuildSha, x7::gitrev::kBuildDirty);
      bundle->publish();
      display_log = (bundle->target() / "hardware.csv").string();
      std::printf("created bundle %s\n", bundle->target().string().c_str());
    }
    std::printf("follow hardware: %s, %llu controller cycles, home %.4f rad, "
                "tracking %.4f rad\n",
                follow::statusName(result.status),
                static_cast<unsigned long long>(result.cycles),
                result.worst_home_error_rad,
                result.worst_tracking_error_rad);
    x7::printCycleStats(stats);
    std::printf("log: %s\n", display_log.c_str());
    if (!log_ok) {
      std::fprintf(stderr, "log: writing %s failed; the log is incomplete\n",
                   display_log.c_str());
    }
    if (cli.timing_log_path) {
      std::printf("timing log: %s\n", cli.timing_log_path->c_str());
      if (!timing_ok) {
        std::fprintf(stderr,
                     "timing log: %s is incomplete (a record or row was "
                     "dropped, or a write failed)\n",
                     cli.timing_log_path->c_str());
      }
    }
    return result.status == follow::RunStatus::Success && released &&
                   log_ok && timing_ok
               ? 0
               : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_follow: %s\n", error.what());
    return 1;
  }
}
