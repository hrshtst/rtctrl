// Hardware frontend for servo-side trajectory following.
// Usage: x7_follow --config run.toml [input/control overrides]
//                  [--port DEV] [--log out.csv] [--bundle DIR] [--check]

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>

#include "bus/dxl_parameters.hpp"
#include "common/x7_common.hpp"
#include "follow/follow_config.hpp"
#include "follow/follow_bundle.hpp"
#include "follow/follow_hardware.hpp"
#include "follow/follow_preflight.hpp"
#include "follow/follow_run.hpp"
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
    if (cli.log_path) config.output.hardware_log = *cli.log_path;
    if (bundle) {
      const auto prepared = follow::prepareBundle(bundle->staging(), config);
      config = follow::loadConfig(prepared.config_path);
      config.output.hardware_log = prepared.hardware_log;
    }

    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    model::ZvsTrajectory reference(config.reference_path.string(), map,
                                   config.reference);
    auto hardware = hw::Config::load(config.hardware_config_path.string());
    follow::validateReference(config, chain, map, reference, hardware);

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
    follow::RunResult result;
    hw::CraneX7::CycleStats stats;
    bool released = false;
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

      arm::RealArm robot(crane);
      if (!robot.setMode(config.mode)) {
        throw std::runtime_error("hardware operating-mode setup mismatch");
      }
      if (!robot.activate()) {
        throw std::runtime_error("activation failed: " + crane.lastError());
      }
      x7::ShutdownGuard shutdown(crane);
      follow::FollowRun run(robot, reference, config, false, &log,
                            [&enter] { return enter.pressed(); });
      result = run.run();
      stats = crane.cycleStats();
      released = shutdown.run();
    }
    std::string display_log = config.output.hardware_log.string();
    if (bundle) {
      follow::writeBundleResult(
          bundle->staging(), "hardware", follow::statusName(result.status),
          result.cycles, result.worst_home_error_rad,
          result.worst_tracking_error_rad);
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
    std::printf("hardware timing: %llu cycles, %llu overruns, %llu skipped, "
                "%llu stale, %llu deadline misses, max %.3f ms\n",
                static_cast<unsigned long long>(stats.cycles),
                static_cast<unsigned long long>(stats.overruns),
                static_cast<unsigned long long>(stats.skipped_periods),
                static_cast<unsigned long long>(stats.stale_submissions),
                static_cast<unsigned long long>(
                    stats.controller_deadline_misses),
                1000.0 * stats.max_cycle_time_s);
    std::printf("log: %s\n", display_log.c_str());
    return result.status == follow::RunStatus::Success && released ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_follow: %s\n", error.what());
    return 1;
  }
}
