// Hardware frontend for servo-side trajectory following.
// Usage: x7_follow --config run.toml [--port DEV] [--log out.csv]
//                  [--bundle DIR] [--check]

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <exception>
#include <optional>
#include <stdexcept>

#include "bus/dxl_parameters.hpp"
#include "common/x7_common.hpp"
#include "follow/follow_config.hpp"
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

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;
namespace parameters = rtctrl::apps::dxl_parameters;

namespace {

void usage() {
  std::printf(
      "usage: x7_follow --config FILE [--port DEV] [--log FILE] "
      "[--bundle DIR] [--check]\n");
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
    if (cli.bundle_path) {
      throw std::runtime_error("--bundle support is not available yet");
    }
    auto config = follow::loadConfig(cli.config_path);
    if (cli.log_path) config.output.hardware_log = *cli.log_path;

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
    follow::FollowCsvLog log(config.output.hardware_log.string());

    std::printf("bus: %s @ %d baud, %zu joints\n", hardware.port.c_str(),
                hardware.baudrate, hardware.joints.size());
    rtctrl::dxl::Port port(hardware.port, hardware.baudrate);
    auto options = follow::hardwareOptions(config, parameter_dump);
    hw::CraneX7 crane(port, hardware, options);
    crane.onEscalate([&port] {
      std::fprintf(stderr,
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
    EnterReader enter;
    follow::FollowRun run(robot, reference, config, false, &log,
                          [&enter] { return enter.pressed(); });
    const auto result = run.run();
    const auto stats = crane.cycleStats();
    const bool released = shutdown.run();
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
    return result.status == follow::RunStatus::Success && released ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_follow: %s\n", error.what());
    return 1;
  }
}
