// Dynamics-simulation frontend for servo-side trajectory following.
// Usage: x7_follow_sim --config run.toml [input/control overrides]
//                      [--motion out.zvs] [--log out.csv]
//                      [--bundle dir] [--check]

#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>

#include "bus/dxl_parameters.hpp"
#include "follow/follow_bundle.hpp"
#include "follow/follow_config.hpp"
#include "follow/follow_hardware.hpp"
#include "follow/follow_preflight.hpp"
#include "follow/follow_run.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/model/zvs_writer.hpp"
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
      "usage: x7_follow_sim --config FILE [options]\n"
      "  --reference FILE\n"
      "  --mode position|current-based-position\n"
      "  --motor-parameters FILE\n"
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
    const auto cli = follow::parseCli(argc, argv, true);
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
    if (cli.log_path) config.output.simulation_log = *cli.log_path;
    if (cli.motion_path) config.output.simulation_motion = *cli.motion_path;
    if (bundle) {
      const auto prepared = follow::prepareBundle(bundle->staging(), config);
      config = follow::loadConfig(prepared.config_path);
      config.output.simulation_motion = prepared.simulation_motion;
      config.output.simulation_log = prepared.simulation_log;
    }
    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    model::ZvsTrajectory reference(config.reference_path.string(), map,
                                   config.reference);
    auto hardware = hw::Config::load(config.hardware_config_path.string());
    follow::validateReference(config, chain, map, reference, hardware);
    if (config.motor_parameters_path) {
      const auto parameter_dump =
          parameters::parseFile(config.motor_parameters_path->string());
      follow::validateMotorParameters(parameter_dump, hardware);
    }
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
    follow::requireNewOutput(config.output.simulation_motion,
                             "simulation motion");
    follow::requireNewOutput(config.output.simulation_log, "simulation log");

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
        follow::SimulationConfig::InitialPosture::Explicit) {
      options.initial_q8.assign(config.simulation.initial_joints.begin(),
                                config.simulation.initial_joints.end());
    } else if (config.simulation.initial_posture ==
               follow::SimulationConfig::InitialPosture::Model) {
      model::ZVector q9(model::kModelDof), q8(model::kCanonicalDof);
      chain.jointDisplacement(q9.get());
      map.reduce(q9.get(), q8.get());
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        options.initial_q8.push_back(q8[i]);
      }
    }

    follow::RunResult result;
    bool released = false;
    {
      model::ZvsWriter motion(config.output.simulation_motion.string());
      follow::FollowCsvLog log(config.output.simulation_log.string());
      arm::SimArm robot(options);
      robot.logTo(&motion);
      if (!robot.setMode(config.mode) || !robot.activate()) {
        throw std::runtime_error("simulation arm activation failed");
      }
      follow::FollowRun run(robot, reference, config, true, &log);
      result = run.run();
      released = robot.deactivate();
    }
    std::string display_motion = config.output.simulation_motion.string();
    std::string display_log = config.output.simulation_log.string();
    if (bundle) {
      follow::writeBundleResult(
          bundle->staging(), "simulation", follow::statusName(result.status),
          result.cycles, result.worst_home_error_rad,
          result.worst_tracking_error_rad);
      x7::ptp::writeBundleManifestFor(
          bundle->staging(), "rtctrl-x7-follow-bundle", 1,
          rtctrl::version(), x7::gitrev::kBuildSha, x7::gitrev::kBuildDirty);
      bundle->publish();
      display_motion = (bundle->target() / "simulation.zvs").string();
      display_log = (bundle->target() / "simulation.csv").string();
      std::printf("created bundle %s\n", bundle->target().string().c_str());
    }
    std::printf("follow simulation: %s, %llu cycles, home %.4f rad, "
                "tracking %.4f rad\n",
                follow::statusName(result.status),
                static_cast<unsigned long long>(result.cycles),
                result.worst_home_error_rad,
                result.worst_tracking_error_rad);
    std::printf("motion: %s\nlog: %s\n", display_motion.c_str(),
                display_log.c_str());
    return result.status == follow::RunStatus::Success && released ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_follow_sim: %s\n", error.what());
    return 1;
  }
}
