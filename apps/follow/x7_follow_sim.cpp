// Dynamics-simulation frontend for servo-side trajectory following.
// Usage: x7_follow_sim --config run.toml [--motion out.zvs]
//                      [--log out.csv] [--bundle dir] [--check]

#include <cstdio>
#include <exception>
#include <stdexcept>

#include "follow/follow_config.hpp"
#include "follow/follow_preflight.hpp"
#include "follow/follow_run.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/model/zvs_writer.hpp"

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;

namespace {

void usage() {
  std::printf(
      "usage: x7_follow_sim --config FILE [--motion FILE] [--log FILE] "
      "[--bundle DIR] [--check]\n");
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto cli = follow::parseCli(argc, argv, true);
    if (cli.help) {
      usage();
      return 0;
    }
    if (cli.bundle_path) {
      throw std::runtime_error("--bundle support is not available yet");
    }
    auto config = follow::loadConfig(cli.config_path);
    if (cli.log_path) config.output.simulation_log = *cli.log_path;
    if (cli.motion_path) config.output.simulation_motion = *cli.motion_path;
    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    model::ZvsTrajectory reference(config.reference_path.string(), map,
                                   config.reference);
    auto hardware = hw::Config::load(config.hardware_config_path.string());
    follow::validateReference(config, chain, map, reference, hardware);
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
    options.kv = config.simulation.velocity_kp;
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

    model::ZvsWriter motion(config.output.simulation_motion.string());
    follow::FollowCsvLog log(config.output.simulation_log.string());
    arm::SimArm robot(options);
    robot.logTo(&motion);
    if (!robot.setMode(config.mode) || !robot.activate()) {
      throw std::runtime_error("simulation arm activation failed");
    }
    follow::FollowRun run(robot, reference, config, true, &log);
    const auto result = run.run();
    const bool released = robot.deactivate();
    std::printf("follow simulation: %s, %llu cycles, home %.4f rad, "
                "tracking %.4f rad\n",
                follow::statusName(result.status),
                static_cast<unsigned long long>(result.cycles),
                result.worst_home_error_rad,
                result.worst_tracking_error_rad);
    return result.status == follow::RunStatus::Success && released ? 0 : 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_follow_sim: %s\n", error.what());
    return 1;
  }
}
