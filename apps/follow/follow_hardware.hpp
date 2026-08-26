#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "bus/dxl_parameters.hpp"
#include "follow/follow_config.hpp"
#include "rtctrl/hw/command_current.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace x7::follow {

namespace parameters = rtctrl::apps::dxl_parameters;

inline void validateMotorParameters(
    const parameters::ParameterDump& dump,
    const rtctrl::hw::Config& hardware) {
  std::set<std::uint8_t> configured;
  for (const auto& joint : hardware.joints) configured.insert(joint.id);
  for (const auto& motor : dump.motors) {
    if (configured.count(motor.id) == 0) {
      throw std::runtime_error(
          "follow preflight: parameter dump contains unconfigured motor id " +
          std::to_string(motor.id));
    }
    for (const auto& parameter : motor.parameters) {
      if (parameter.def->reg.addr ==
          rtctrl::dxl::reg::kOperatingMode.addr) {
        throw std::runtime_error(
            "follow preflight: motor_parameters must omit operating_mode; "
            "control.mode owns it");
      }
    }
  }
}

inline rtctrl::hw::CraneX7::Options hardwareOptions(
    const Config& config,
    const std::optional<parameters::ParameterDump>& parameter_dump) {
  rtctrl::hw::CraneX7::Options options;
  options.control_cycle_s = 1.0 / config.control_rate_hz;
  options.controller_deadline_s = options.control_cycle_s;
  options.controller_write_margin_s =
      std::min(0.002, 0.2 * options.control_cycle_s);
  if (parameter_dump) {
    options.activation_configurator =
        [dump = *parameter_dump](rtctrl::dxl::PacketIO& io,
                                 std::string* error) {
          const auto result = parameters::apply(io, dump);
          if (!result.ok) {
            *error = "motor parameter update failed: " + result.error;
            if (result.rollback_attempted && !result.rollback_ok) {
              *error += "; rollback incomplete";
            }
            return false;
          }
          return true;
        };
  }
  return options;
}

inline void selectHardwareMode(rtctrl::hw::Config* hardware,
                               arm::ControlMode mode,
                               const std::optional<std::string>& port) {
  if (port) hardware->port = *port;
  for (auto& joint : hardware->joints) {
    joint.operating_mode = static_cast<std::uint8_t>(mode);
  }
  hardware->validate();
}

inline std::vector<double> currentBasedPositionLimits(
    const Config& config, const rtctrl::hw::Config& hardware) {
  std::vector<double> amps;
  if (config.mode != arm::ControlMode::CurrentBasedPosition) return amps;
  amps.reserve(model::kCanonicalDof);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    amps.push_back(std::fabs(rtctrl::hw::commandCurrentFromTorque(
        hardware.joints[i], config.effort_limit_nm[i])));
  }
  return amps;
}

}  // namespace x7::follow
