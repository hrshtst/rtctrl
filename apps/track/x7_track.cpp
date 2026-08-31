// Hardware frontend for host-side computed-torque trajectory tracking.
// SAFETY: current mode. Keep the power cutoff in reach and support the arm
// before ending the final hold.

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>

#include "common/x7_common.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/version.hpp"
#include "track/track_bundle.hpp"
#include "track/track_config.hpp"
#include "track/track_hardware.hpp"
#include "track/track_run.hpp"

namespace x7::gitrev {
extern const char* const kBuildSha;
extern const bool kBuildDirty;
}  // namespace x7::gitrev

namespace arm = rtctrl::arm;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;
namespace track = x7::track;

namespace {

void usage() {
  std::printf(
      "usage: x7_track --config FILE [options]\n"
      "  --reference FILE\n"
      "  --kp VALUE\n"
      "  --kd VALUE\n"
      "  --playback-rate VALUE\n"
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
    const auto cli = track::parseCli(argc, argv, false);
    if (cli.help) {
      usage();
      return 0;
    }
    std::unique_ptr<x7::ptp::BundleWorkspace> bundle;
    if (cli.bundle_path) {
      bundle = std::make_unique<x7::ptp::BundleWorkspace>(*cli.bundle_path);
    }
    auto config = track::loadConfig(cli.config_path);
    track::applyOverrides(cli, &config);
    if (cli.log_path) config.output.hardware_log = *cli.log_path;
    if (bundle) {
      const auto prepared = track::prepareBundle(bundle->staging(), config);
      config = track::loadConfig(prepared.config_path);
      config.output.hardware_log = prepared.hardware_log;
    }

    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    model::ZvsTrajectory source(config.reference_path.string(), map,
                                config.reference);
    track::PlaybackTrajectory reference(source, config.playback_rate);
    auto hardware = hw::Config::load(config.hardware_config_path.string());
    track::prepareHardwareConfig(config, &hardware, cli.port);
    track::validateReference(config, chain, map, reference, hardware);
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
    track::requireNewOutput(config.output.hardware_log, "hardware log");

    track::RunResult result;
    hw::CraneX7::CycleStats stats;
    bool released = false;
    {
      EnterReader enter;
      if (config.finalization.wait_time_s == 0.0 && !enter.ready()) {
        throw std::runtime_error(
            "cannot make stdin nonblocking for the final Enter hold");
      }
      track::TrackCsvLog log(config.output.hardware_log.string());
      std::printf("bus: %s @ %d baud, %zu joints\n", hardware.port.c_str(),
                  hardware.baudrate, hardware.joints.size());
      rtctrl::dxl::Port port(hardware.port, hardware.baudrate);
      hw::CraneX7 crane(port, hardware, track::hardwareOptions(config));
      crane.onEscalate([&port] {
        std::fprintf(stderr,
                     "DEADMAN: command stream stale; closing the bus so "
                     "servo watchdogs halt the arm\n");
        port.close();
      });
      x7::ShutdownGuard shutdown(crane);

      arm::RealArm position_arm(crane);
      if (!position_arm.setMode(arm::ControlMode::Position) ||
          !position_arm.activate()) {
        throw std::runtime_error("position activation failed: " +
                                 crane.lastError());
      }
      const auto home =
          track::runPositionHome(position_arm, reference, config, &log);
      if (home.status != track::RunStatus::Success) {
        result = home;
      } else {
        crane.stopThread();
        const auto held = crane.lastFeedback();
        if (held.size() != model::kCanonicalDof) {
          throw std::runtime_error("held-posture feedback size mismatch");
        }
        constexpr double kLimitBuffer = 0.05;
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          if (held[i].position < crane.softLimitLo()[i] + kLimitBuffer ||
              held[i].position > crane.softLimitHi()[i] - kLimitBuffer) {
            throw std::runtime_error(
                "reference start lies inside the current-mode soft-limit band "
                "on joint " +
                std::to_string(i));
          }
        }
        const auto preload = track::gravityPreload(hardware, chain, map, held);
        std::printf("home settled; switching stationary to current mode with "
                    "gravity preload\n");
        if (!crane.switchToCurrentModeWithPreload(preload)) {
          throw std::runtime_error("current-mode transition failed: " +
                                   crane.lastError());
        }
        arm::RealArm current_arm(crane);
        if (!current_arm.setMode(arm::ControlMode::Current) ||
            !current_arm.activate()) {
          throw std::runtime_error("current controller activation failed: " +
                                   crane.lastError());
        }
        track::TrackingRun run(current_arm, chain, map, reference, config,
                               false, &log,
                               [&enter] { return enter.pressed(); });
        result = run.run();
        result.cycles += home.cycles;
        result.worst_home_error_rad = home.worst_home_error_rad;
      }
      stats = crane.cycleStats();
      released = shutdown.run();
    }

    std::string display_log = config.output.hardware_log.string();
    if (bundle) {
      track::writeBundleResult(bundle->staging(), "hardware", result);
      x7::ptp::writeBundleManifestFor(
          bundle->staging(), "rtctrl-x7-track-bundle", 1,
          rtctrl::version(), x7::gitrev::kBuildSha, x7::gitrev::kBuildDirty);
      bundle->publish();
      display_log = (bundle->target() / "hardware.csv").string();
      std::printf("created bundle %s\n", bundle->target().string().c_str());
    }
    std::printf(
        "track hardware: %s, assessment %s, RMS %.4f rad, worst-joint "
        "RMS %.4f rad, peak %.4f rad\n",
        track::statusName(result.status), result.tracking_pass ? "PASS" : "FAIL",
        result.aggregate_rms_error_rad, result.worst_joint_rms_error_rad,
        result.peak_error_rad);
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
    return result.status == track::RunStatus::Success && released ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_track: %s\n", error.what());
    return 1;
  }
}
