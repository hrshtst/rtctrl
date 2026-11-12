// The simulated plant of x7_follow_sim, built from a follow config: the
// SimArm options (servo gains, friction, effort limits, initial posture)
// and, for the servo-like keys, the LaggedArm that adds the command delay
// and the encoder readout. Shared with x7_adaptive_follow_study so that a
// study runs on exactly the plant x7_follow_sim simulates. Simulation only:
// nothing here opens a bus.
#pragma once

#include <optional>

#include "common/lagged_arm.hpp"
#include "follow/follow_config.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7::follow {

// Time constant of the servo-style velocity estimate when the encoder is
// quantized (the lag model's value in x7_efl_study and its unit tests).
inline constexpr double kServoVelocityLagS = 0.12;

// The SimArm options of a follow simulation. The effort limits come from
// the hardware config's joints. With initial_posture = "model" the start
// is the chain's currently loaded joint displacement, so call this before
// any dynamics evaluation on `chain` (gravityTorque and inverseDynamics
// move that displacement).
inline arm::SimArm::Options simArmOptions(const Config& config,
                                          const rtctrl::hw::Config& hardware,
                                          const model::ChainModel& chain,
                                          const model::JointMap& map) {
  arm::SimArm::Options options;
  options.model_path = config.model_path.string();
  options.control_dt = 1.0 / config.control_rate_hz;
  options.sim_dt = config.simulation.integration_step_s;
  options.kp = config.simulation.position_kp;
  options.kd = config.simulation.position_kd;
  const auto assign = [](const auto& values, std::vector<double>* target) {
    if (values) target->assign(values->begin(), values->end());
  };
  assign(config.simulation.position_kp_joints, &options.kp8);
  assign(config.simulation.position_kd_joints, &options.kd8);
  assign(config.simulation.coulomb_friction_nm, &options.fc8);
  assign(config.simulation.viscous_friction_nm_s, &options.fv8);
  options.friction_v0 = config.simulation.friction_v0_rad_s;
  options.effort_limit8.clear();
  for (const auto& joint : hardware.joints) {
    options.effort_limit8.push_back(joint.effort_limit);
  }
  if (config.simulation.initial_posture ==
      SimulationConfig::InitialPosture::Explicit) {
    options.initial_q8.assign(config.simulation.initial_joints.begin(),
                              config.simulation.initial_joints.end());
  } else if (config.simulation.initial_posture ==
             SimulationConfig::InitialPosture::Model) {
    model::ZVector q9(model::kModelDof), q8(model::kCanonicalDof);
    chain.jointDisplacement(q9.get());
    map.reduce(q9.get(), q8.get());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      options.initial_q8.push_back(q8[i]);
    }
  }
  return options;
}

// Transport delay and encoder readout: the command accepted at cycle k
// reaches the servo at cycle k + command_delay_cycles, the arm holding its
// activation posture until then; feedback exact unless quantize_encoder.
// Empty when neither is configured (the arm is used unwrapped).
inline std::optional<LaggedArmOptions> laggedArmOptions(const Config& config) {
  if (config.simulation.command_delay_cycles <= 0 &&
      !config.simulation.quantize_encoder) {
    return std::nullopt;
  }
  LaggedArmOptions lag;
  lag.delay_cycles = config.simulation.command_delay_cycles;
  lag.first_passthrough = false;
  lag.hold_preload = true;
  lag.quantize_pos = config.simulation.quantize_encoder;
  lag.exact_velocity = !config.simulation.quantize_encoder;
  return lag;
}

}  // namespace x7::follow
