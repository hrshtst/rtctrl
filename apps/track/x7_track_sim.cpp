// Dynamics-simulation frontend for host-side computed-torque tracking.

#include <cstdio>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>

#include "common/link_force_bundle.hpp"
#include "common/link_force_run.hpp"
#include "rtctrl/arm/link_force_log.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/link_force.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/model/zvs_writer.hpp"
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
      "usage: x7_track_sim --config FILE [options]\n"
      "  --reference FILE\n"
      "  --kp VALUE\n"
      "  --kd VALUE\n"
      "  --playback-rate VALUE\n"
      "  --effort-limit-nm VALUE\n"
      "  --filter none|low-pass|moving-average|savitzky-golay\n"
      "  --interpolation linear|shape-preserving-cubic\n"
      "  --motion FILE\n"
      "  --log FILE\n"
      "  --bundle NEW_DIRECTORY\n"
      "  --check\n");
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto cli = track::parseCli(argc, argv, true);
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
    if (cli.log_path) config.output.simulation_log = *cli.log_path;
    if (cli.motion_path) config.output.simulation_motion = *cli.motion_path;
    if (bundle) {
      const auto prepared = track::prepareBundle(bundle->staging(), config);
      config = track::loadConfig(prepared.config_path);
      config.output.simulation_motion = prepared.simulation_motion;
      config.output.simulation_log = prepared.simulation_log;
    }

    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    model::ZvsTrajectory source(config.reference_path.string(), map,
                                config.reference);
    track::PlaybackTrajectory reference(source, config.playback_rate);
    auto hardware = hw::Config::load(config.hardware_config_path.string());
    track::prepareHardwareConfig(config, &hardware, std::nullopt);
    track::validateReference(config, chain, map, reference, hardware);
    if (config.control_rate_hz != 100.0) {
      std::fprintf(stderr,
                   "warning: hardware timing is validated at 100 Hz; %.1f "
                   "Hz is experimental\n",
                   config.control_rate_hz);
    }
    std::optional<model::LinkForceSpec> link_force;
    if (config.simulation.link_force_path) {
      link_force =
          model::loadLinkForce(config.simulation.link_force_path->string());
      // the profile clock runs in elapsed tracking time: the playback-scaled
      // reference duration plus the final hold
      x7::preflightLinkForce(
          *link_force, chain,
          reference.duration() + config.finalization.simulation_hold_time_s);
    }
    if (cli.check) {
      std::printf("configuration and reference preflight passed\n");
      return 0;
    }
    track::requireNewOutput(config.output.simulation_motion,
                            "simulation motion");
    track::requireNewOutput(config.output.simulation_log, "simulation log");
    const auto disturbance_log_path =
        x7::disturbanceLogPath(config.output.simulation_log);
    if (link_force) {
      track::requireNewOutput(disturbance_log_path,
                              "simulation disturbance log");
    }

    arm::SimArm::Options options;
    options.model_path = config.model_path.string();
    options.control_dt = 1.0 / config.control_rate_hz;
    options.sim_dt = config.simulation.integration_step_s;
    options.kp = config.simulation.position_kp;
    options.kd = config.simulation.position_kd;
    options.effort_limit8.clear();
    for (const auto& joint : hardware.joints) {
      options.effort_limit8.push_back(joint.effort_limit);
    }
    if (config.simulation.initial_posture ==
        track::SimulationConfig::InitialPosture::Explicit) {
      options.initial_q8.assign(config.simulation.initial_joints.begin(),
                                config.simulation.initial_joints.end());
    } else if (config.simulation.initial_posture ==
               track::SimulationConfig::InitialPosture::Model) {
      model::ZVector q9(model::kModelDof), q8(model::kCanonicalDof);
      chain.jointDisplacement(q9.get());
      map.reduce(q9.get(), q8.get());
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        options.initial_q8.push_back(q8[i]);
      }
    }

    track::RunResult result;
    bool released = false;
    x7::LinkForceRunReport force_report;
    {
      model::ZvsWriter motion(config.output.simulation_motion.string());
      track::TrackCsvLog log(config.output.simulation_log.string());
      std::unique_ptr<arm::LinkForceCsvLog> force_log;
      if (link_force) {
        force_log =
            std::make_unique<arm::LinkForceCsvLog>(disturbance_log_path);
      }
      arm::SimArm robot(options);
      robot.logTo(&motion);
      // logged from the first homing step; started at the first tracking
      // cycle, after the current-mode switch
      if (link_force) {
        x7::prepareLinkForce(robot, *link_force, force_log.get());
      }
      if (!robot.setMode(arm::ControlMode::Position) || !robot.activate()) {
        throw std::runtime_error("simulation position activation failed");
      }
      const auto home = track::runPositionHome(robot, reference, config, &log);
      if (home.status != track::RunStatus::Success) {
        result = home;
        robot.deactivate();
      } else {
        if (!robot.deactivate() || !robot.setMode(arm::ControlMode::Current) ||
            !robot.activate()) {
          throw std::runtime_error("simulation current transition failed");
        }
        track::TrackingRun run(robot, chain, map, reference, config, true,
                               &log);
        if (link_force) {
          x7::startLinkForceAtTracking(robot, run, &force_report);
        }
        result = run.run();
        result.cycles += home.cycles;
        result.worst_home_error_rad = home.worst_home_error_rad;
        released = robot.deactivate();
      }
      if (link_force) x7::finishLinkForce(robot, *force_log, &force_report);
    }

    std::string display_motion = config.output.simulation_motion.string();
    std::string display_log = config.output.simulation_log.string();
    if (bundle) {
      track::writeBundleResult(
          bundle->staging(), "simulation", result,
          link_force ? x7::simulationLinkForceProvenance(force_report.outcome)
                     : std::string());
      x7::ptp::writeBundleManifestFor(
          bundle->staging(), "rtctrl-x7-track-bundle", 1,
          rtctrl::version(), x7::gitrev::kBuildSha, x7::gitrev::kBuildDirty);
      bundle->publish();
      display_motion = (bundle->target() / "simulation.zvs").string();
      display_log = (bundle->target() / "simulation.csv").string();
      std::printf("created bundle %s\n", bundle->target().string().c_str());
    }
    std::printf(
        "track simulation: %s, assessment %s, RMS %.4f rad, worst-joint "
        "RMS %.4f rad, peak %.4f rad\n",
        track::statusName(result.status), result.tracking_pass ? "PASS" : "FAIL",
        result.aggregate_rms_error_rad, result.worst_joint_rms_error_rad,
        result.peak_error_rad);
    std::printf("motion: %s\nlog: %s\n", display_motion.c_str(),
                display_log.c_str());
    bool force_ok = true;
    if (link_force) {
      const auto display_force =
          bundle ? (bundle->target() / "simulation_disturbance.csv").string()
                 : disturbance_log_path.string();
      force_ok = x7::printLinkForceReport(force_report, display_force);
    }
    return result.status == track::RunStatus::Success && released &&
                   force_ok
               ? 0
               : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_track_sim: %s\n", error.what());
    return 1;
  }
}
