#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <toml++/toml.hpp>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/packet_io.hpp"

namespace rtctrl::apps::dxl_parameters {

inline constexpr std::string_view kFormat = "rtctrl-dynamixel-parameters";
inline constexpr std::int64_t kVersion = 1;

// A dump includes the configuration/tuning registers defined by the
// XM430-W350 and XM540-W270 control tables. Model- and firmware-specific
// entries are emitted only when supported. A few fields are deliberately
// dump-only: communication changes defeat exact readback/rollback, while
// StartupConfiguration can enable torque automatically at power-on.
struct ParameterDef {
  const char* name;
  dxl::Reg reg;
  bool signed_value;
  bool loadable;
  std::uint8_t model_mask = 0x03;
  std::uint8_t min_firmware = 0;
};

inline constexpr std::uint8_t kModelXm430 = 0x01;
inline constexpr std::uint8_t kModelXm540 = 0x02;
inline constexpr std::uint8_t kBothModels = kModelXm430 | kModelXm540;

inline constexpr std::array<ParameterDef, 31> kParameters{{
    {"baud_rate", dxl::reg::kBaudRate, false, false},
    {"return_delay_time", dxl::reg::kReturnDelayTime, false, true},
    {"drive_mode", dxl::reg::kDriveMode, false, true},
    {"operating_mode", dxl::reg::kOperatingMode, false, true},
    {"secondary_id", dxl::reg::kSecondaryId, false, false},
    {"protocol_type", dxl::reg::kProtocolType, false, false},
    {"homing_offset", dxl::reg::kHomingOffset, true, true},
    {"moving_threshold", dxl::reg::kMovingThreshold, false, true},
    {"temperature_limit", dxl::reg::kTemperatureLimit, false, true},
    {"max_voltage_limit", dxl::reg::kMaxVoltageLimit, false, true},
    {"min_voltage_limit", dxl::reg::kMinVoltageLimit, false, true},
    {"pwm_limit", dxl::reg::kPwmLimit, false, true},
    {"current_limit", dxl::reg::kCurrentLimit, false, true},
    {"velocity_limit", dxl::reg::kVelocityLimit, false, true},
    {"max_position_limit", dxl::reg::kMaxPositionLimit, false, true},
    {"min_position_limit", dxl::reg::kMinPositionLimit, false, true},
    {"external_port_mode_1", dxl::reg::kExternalPortMode1, false, true,
     kModelXm540},
    {"external_port_mode_2", dxl::reg::kExternalPortMode2, false, true,
     kModelXm540},
    {"external_port_mode_3", dxl::reg::kExternalPortMode3, false, true,
     kModelXm540},
    {"startup_configuration", dxl::reg::kStartupConfiguration, false, false,
     kBothModels, 45},
    {"shutdown", dxl::reg::kShutdown, false, true},
    {"status_return_level", dxl::reg::kStatusReturnLevel, false, false},
    {"velocity_i_gain", dxl::reg::kVelocityIGain, false, true},
    {"velocity_p_gain", dxl::reg::kVelocityPGain, false, true},
    {"position_d_gain", dxl::reg::kPositionDGain, false, true},
    {"position_i_gain", dxl::reg::kPositionIGain, false, true},
    {"position_p_gain", dxl::reg::kPositionPGain, false, true},
    {"feedforward_2nd_gain", dxl::reg::kFeedforward2ndGain, false, true},
    {"feedforward_1st_gain", dxl::reg::kFeedforward1stGain, false, true},
    {"profile_acceleration", dxl::reg::kProfileAcceleration, false, true},
    {"profile_velocity", dxl::reg::kProfileVelocity, false, true},
}};

struct ParameterValue {
  const ParameterDef* def = nullptr;
  std::int64_t value = 0;
};

struct MotorRecord {
  std::uint8_t id = 0;
  std::uint16_t model_number = 0;
  std::uint8_t firmware_version = 0;
  std::vector<ParameterValue> parameters;
};

struct ParameterDump {
  std::vector<MotorRecord> motors;
};

inline const ParameterDef* findParameter(std::string_view name) {
  for (const auto& def : kParameters) {
    if (name == def.name) return &def;
  }
  return nullptr;
}

inline const ParameterDef* findParameter(dxl::Reg reg) {
  for (const auto& def : kParameters) {
    if (def.reg.addr == reg.addr && def.reg.len == reg.len) return &def;
  }
  return nullptr;
}

inline bool supportsModel(const ParameterDef& def, std::uint16_t model) {
  if (model == dxl::kModelXm430W350) {
    return (def.model_mask & kModelXm430) != 0;
  }
  if (model == dxl::kModelXm540W270) {
    return (def.model_mask & kModelXm540) != 0;
  }
  return false;
}

inline std::string ioError(const dxl::IoResult& result) {
  return "comm " + std::to_string(result.comm) + ", error 0x" + [&] {
    constexpr char kHex[] = "0123456789ABCDEF";
    std::string text(2, '0');
    text[0] = kHex[(result.error >> 4) & 0xF];
    text[1] = kHex[result.error & 0xF];
    return text;
  }();
}

inline bool readParameter(dxl::PacketIO& io, std::uint8_t id,
                          const ParameterDef& def, std::int64_t* value,
                          std::string* error) {
  std::uint8_t data[4] = {};
  const auto result = io.read(id, def.reg.addr, data, def.reg.len);
  if (!result.ok()) {
    *error = "id " + std::to_string(id) + ": cannot read " + def.name + " (" +
             ioError(result) + ")";
    return false;
  }
  std::uint32_t raw = 0;
  for (int i = def.reg.len - 1; i >= 0; --i) raw = (raw << 8) | data[i];
  *value = def.signed_value ? static_cast<std::int32_t>(raw)
                            : static_cast<std::int64_t>(raw);
  return true;
}

inline bool writeParameter(dxl::PacketIO& io, std::uint8_t id,
                           const ParameterValue& parameter,
                           std::string* error) {
  std::uint8_t data[4] = {};
  const auto raw = static_cast<std::uint32_t>(parameter.value);
  for (int i = 0; i < parameter.def->reg.len; ++i) {
    data[i] = static_cast<std::uint8_t>(raw >> (8 * i));
  }
  const auto result =
      io.write(id, parameter.def->reg.addr, data, parameter.def->reg.len);
  if (!result.ok()) {
    *error = "id " + std::to_string(id) + ": cannot write " +
             parameter.def->name + " (" + ioError(result) + ")";
    return false;
  }
  std::int64_t actual = 0;
  if (!readParameter(io, id, *parameter.def, &actual, error)) return false;
  if (actual != parameter.value) {
    *error = "id " + std::to_string(id) + ": " + parameter.def->name +
             " readback is " + std::to_string(actual) + ", expected " +
             std::to_string(parameter.value);
    return false;
  }
  return true;
}

inline bool captureMotor(dxl::PacketIO& io, std::uint8_t id, MotorRecord* motor,
                         std::string* error) {
  std::uint16_t model = 0;
  const auto ping = io.ping(id, &model);
  if (!ping.ok()) {
    *error =
        "id " + std::to_string(id) + ": ping failed (" + ioError(ping) + ")";
    return false;
  }
  if (model != dxl::kModelXm430W350 && model != dxl::kModelXm540W270) {
    *error = "id " + std::to_string(id) + ": unsupported model " +
             std::to_string(model) + " (expected XM430-W350 or XM540-W270)";
    return false;
  }
  std::uint8_t firmware = 0;
  const auto fw = io.read8(id, dxl::reg::kFirmwareVersion.addr, &firmware);
  if (!fw.ok()) {
    *error = "id " + std::to_string(id) + ": cannot read firmware_version (" +
             ioError(fw) + ")";
    return false;
  }
  motor->id = id;
  motor->model_number = model;
  motor->firmware_version = firmware;
  motor->parameters.clear();
  for (const auto& def : kParameters) {
    if (!supportsModel(def, model)) continue;
    if (firmware < def.min_firmware) continue;
    std::int64_t value = 0;
    if (!readParameter(io, id, def, &value, error)) return false;
    motor->parameters.push_back({&def, value});
  }
  return true;
}

inline std::string serialize(const ParameterDump& dump) {
  std::ostringstream out;
  out << "format = \"" << kFormat << "\"\n"
      << "version = " << kVersion << "\n";
  for (const auto& motor : dump.motors) {
    out << "\n[[motor]]\n"
        << "id = " << static_cast<unsigned>(motor.id) << "\n"
        << "model_number = " << motor.model_number << "\n"
        << "firmware_version = "
        << static_cast<unsigned>(motor.firmware_version) << "\n\n"
        << "[motor.parameters]\n";
    for (const auto& parameter : motor.parameters) {
      out << parameter.def->name << " = " << parameter.value << "\n";
    }
  }
  return out.str();
}

inline void validateValue(const ParameterDef& def, std::int64_t value) {
  std::int64_t min = 0;
  std::int64_t max = 0;
  if (def.signed_value) {
    min = std::numeric_limits<std::int32_t>::min();
    max = std::numeric_limits<std::int32_t>::max();
  } else {
    max = def.reg.len == 1   ? std::numeric_limits<std::uint8_t>::max()
          : def.reg.len == 2 ? std::numeric_limits<std::uint16_t>::max()
                             : std::numeric_limits<std::uint32_t>::max();
  }
  if (value < min || value > max) {
    throw std::runtime_error(std::string(def.name) + " value " +
                             std::to_string(value) + " does not fit its " +
                             std::to_string(def.reg.len) + "-byte register");
  }
  if (def.reg.addr == dxl::reg::kOperatingMode.addr && value != 0 &&
      value != 1 && value != 3 && value != 4 && value != 5 && value != 16) {
    throw std::runtime_error("operating_mode must be 0, 1, 3, 4, 5 or 16");
  }
}

inline bool knownKey(std::string_view key,
                     std::initializer_list<std::string_view> allowed) {
  return std::find(allowed.begin(), allowed.end(), key) != allowed.end();
}

inline ParameterDump parse(const toml::table& table) {
  for (const auto& [key, node] : table) {
    (void)node;
    if (!knownKey(key.str(), {"format", "version", "motor"})) {
      throw std::runtime_error("unknown top-level key '" +
                               std::string(key.str()) + "'");
    }
  }
  const auto format = table["format"].value<std::string>();
  const auto version = table["version"].value<std::int64_t>();
  if (!format || *format != kFormat) {
    throw std::runtime_error("unsupported or missing parameter-dump format");
  }
  if (!version || *version != kVersion) {
    throw std::runtime_error("unsupported or missing parameter-dump version");
  }
  const auto* motors = table["motor"].as_array();
  if (motors == nullptr || motors->empty()) {
    throw std::runtime_error("parameter dump has no [[motor]] entries");
  }

  ParameterDump dump;
  std::set<int> seen_ids;
  for (const auto& node : *motors) {
    const auto* source = node.as_table();
    if (source == nullptr) {
      throw std::runtime_error("each motor entry must be a table");
    }
    for (const auto& [key, child] : *source) {
      (void)child;
      if (!knownKey(key.str(),
                    {"id", "model_number", "firmware_version", "parameters"})) {
        throw std::runtime_error("unknown motor key '" +
                                 std::string(key.str()) + "'");
      }
    }
    const auto id = (*source)["id"].value<std::int64_t>();
    const auto model = (*source)["model_number"].value<std::int64_t>();
    const auto firmware = (*source)["firmware_version"].value<std::int64_t>();
    if (!id || *id < 0 || *id > 252) {
      throw std::runtime_error("motor id must be an integer in [0, 252]");
    }
    if (!seen_ids.insert(static_cast<int>(*id)).second) {
      throw std::runtime_error("duplicate motor id " + std::to_string(*id));
    }
    if (!model || *model < 0 || *model > 65535) {
      throw std::runtime_error("model_number must be an integer in [0, 65535]");
    }
    if (*model != dxl::kModelXm430W350 && *model != dxl::kModelXm540W270) {
      throw std::runtime_error("unsupported model_number " +
                               std::to_string(*model));
    }
    if (!firmware || *firmware < 0 || *firmware > 255) {
      throw std::runtime_error(
          "firmware_version must be an integer in [0, 255]");
    }
    const auto* parameters = (*source)["parameters"].as_table();
    if (parameters == nullptr) {
      throw std::runtime_error("motor " + std::to_string(*id) +
                               " has no [motor.parameters] table");
    }

    MotorRecord motor;
    motor.id = static_cast<std::uint8_t>(*id);
    motor.model_number = static_cast<std::uint16_t>(*model);
    motor.firmware_version = static_cast<std::uint8_t>(*firmware);
    for (const auto& [key, value_node] : *parameters) {
      const auto* def = findParameter(key.str());
      if (def == nullptr) {
        throw std::runtime_error("unknown parameter '" +
                                 std::string(key.str()) + "'");
      }
      const auto value = value_node.value<std::int64_t>();
      if (!value) {
        throw std::runtime_error("parameter '" + std::string(key.str()) +
                                 "' must be an integer");
      }
      validateValue(*def, *value);
      if (!supportsModel(*def, motor.model_number)) {
        throw std::runtime_error("parameter '" + std::string(key.str()) +
                                 "' is not supported by model " +
                                 std::to_string(motor.model_number));
      }
      motor.parameters.push_back({def, *value});
    }
    std::sort(motor.parameters.begin(), motor.parameters.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.def->reg.addr < rhs.def->reg.addr;
              });
    dump.motors.push_back(std::move(motor));
  }
  return dump;
}

inline ParameterDump parseFile(const std::string& path) {
  try {
    return parse(toml::parse_file(path));
  } catch (const toml::parse_error& error) {
    throw std::runtime_error("cannot parse '" + path +
                             "': " + std::string(error.description()));
  }
}

struct ApplyResult {
  bool ok = false;
  std::size_t changed = 0;
  std::size_t unchanged = 0;
  bool rollback_attempted = false;
  bool rollback_ok = false;
  std::string error;
};

struct Operation {
  std::uint8_t id = 0;
  ParameterValue desired;
  ParameterValue original;
};

inline bool isModeResetParameter(const ParameterDef& def) {
  return def.reg.addr == dxl::reg::kVelocityIGain.addr ||
         def.reg.addr == dxl::reg::kVelocityPGain.addr ||
         def.reg.addr == dxl::reg::kPositionDGain.addr ||
         def.reg.addr == dxl::reg::kPositionIGain.addr ||
         def.reg.addr == dxl::reg::kPositionPGain.addr ||
         def.reg.addr == dxl::reg::kFeedforward2ndGain.addr ||
         def.reg.addr == dxl::reg::kFeedforward1stGain.addr ||
         def.reg.addr == dxl::reg::kProfileAcceleration.addr ||
         def.reg.addr == dxl::reg::kProfileVelocity.addr;
}

inline ApplyResult apply(dxl::PacketIO& io, const ParameterDump& dump) {
  ApplyResult result;
  std::vector<Operation> operations;

  // Finish every preflight read before the first write. A malformed target,
  // model mismatch, torque-on motor, or unreadable register therefore leaves
  // the complete bus untouched.
  for (const auto& motor : dump.motors) {
    std::uint16_t actual_model = 0;
    const auto ping = io.ping(motor.id, &actual_model);
    if (!ping.ok()) {
      result.error = "id " + std::to_string(motor.id) + ": ping failed (" +
                     ioError(ping) + ")";
      return result;
    }
    if (actual_model != motor.model_number) {
      result.error = "id " + std::to_string(motor.id) + ": model is " +
                     std::to_string(actual_model) + ", dump expects " +
                     std::to_string(motor.model_number);
      return result;
    }
    std::uint8_t actual_firmware = 0;
    const auto firmware_read =
        io.read8(motor.id, dxl::reg::kFirmwareVersion.addr, &actual_firmware);
    if (!firmware_read.ok()) {
      result.error = "id " + std::to_string(motor.id) +
                     ": cannot read firmware_version (" +
                     ioError(firmware_read) + ")";
      return result;
    }
    std::uint8_t torque = 0;
    const auto torque_read =
        io.read8(motor.id, dxl::reg::kTorqueEnable.addr, &torque);
    if (!torque_read.ok()) {
      result.error = "id " + std::to_string(motor.id) +
                     ": cannot verify torque is off (" + ioError(torque_read) +
                     ")";
      return result;
    }
    if (torque != 0) {
      result.error = "id " + std::to_string(motor.id) +
                     ": torque is enabled; refusing parameter changes";
      return result;
    }

    bool mode_changes = false;
    std::vector<Operation> motor_ops;
    for (const auto& requested : motor.parameters) {
      if (!supportsModel(*requested.def, actual_model)) {
        result.error = "id " + std::to_string(motor.id) + ": " +
                       requested.def->name + " is not supported by model " +
                       std::to_string(actual_model);
        return result;
      }
      if (actual_firmware < requested.def->min_firmware) {
        result.error = "id " + std::to_string(motor.id) + ": " +
                       requested.def->name + " requires firmware >= " +
                       std::to_string(requested.def->min_firmware) +
                       ", motor reports " + std::to_string(actual_firmware);
        return result;
      }
      std::int64_t current = 0;
      if (!readParameter(io, motor.id, *requested.def, &current,
                         &result.error)) {
        return result;
      }
      if (!requested.def->loadable) {
        if (current != requested.value) {
          result.error = "id " + std::to_string(motor.id) + ": " +
                         requested.def->name +
                         " differs, but this field is dump-only and cannot "
                         "be restored safely";
          return result;
        }
        ++result.unchanged;
        continue;
      }
      if (current == requested.value) {
        ++result.unchanged;
        continue;
      }
      motor_ops.push_back({motor.id, requested, {requested.def, current}});
      if (requested.def->reg.addr == dxl::reg::kOperatingMode.addr) {
        mode_changes = true;
      }
    }

    const auto checkOrderedPair = [&](dxl::Reg lower_reg, dxl::Reg upper_reg,
                                      const char* label) {
      const auto lower_requested =
          std::find_if(motor.parameters.begin(), motor.parameters.end(),
                       [&](const ParameterValue& value) {
                         return value.def->reg.addr == lower_reg.addr;
                       });
      const auto upper_requested =
          std::find_if(motor.parameters.begin(), motor.parameters.end(),
                       [&](const ParameterValue& value) {
                         return value.def->reg.addr == upper_reg.addr;
                       });
      if (lower_requested == motor.parameters.end() &&
          upper_requested == motor.parameters.end()) {
        return true;
      }
      std::int64_t lower = 0;
      std::int64_t upper = 0;
      const auto* lower_def = findParameter(lower_reg);
      const auto* upper_def = findParameter(upper_reg);
      if (lower_requested != motor.parameters.end()) {
        lower = lower_requested->value;
      } else if (!readParameter(io, motor.id, *lower_def, &lower,
                                &result.error)) {
        return false;
      }
      if (upper_requested != motor.parameters.end()) {
        upper = upper_requested->value;
      } else if (!readParameter(io, motor.id, *upper_def, &upper,
                                &result.error)) {
        return false;
      }
      if (lower > upper) {
        result.error = "id " + std::to_string(motor.id) + ": " + label +
                       " minimum " + std::to_string(lower) +
                       " exceeds maximum " + std::to_string(upper);
        return false;
      }
      return true;
    };
    if (!checkOrderedPair(dxl::reg::kMinVoltageLimit,
                          dxl::reg::kMaxVoltageLimit, "voltage limit") ||
        !checkOrderedPair(dxl::reg::kMinPositionLimit,
                          dxl::reg::kMaxPositionLimit, "position limit")) {
      return result;
    }

    // OperatingMode resets gains and profiles. Snapshot and restore every
    // omitted reset-sensitive field so a one-line mode edit cannot silently
    // alter unrelated tuning.
    if (mode_changes) {
      for (const auto& def : kParameters) {
        if (!isModeResetParameter(def)) continue;
        const auto already = std::find_if(
            motor_ops.begin(), motor_ops.end(),
            [&](const Operation& op) { return op.desired.def == &def; });
        if (already != motor_ops.end()) continue;
        std::int64_t current = 0;
        if (!readParameter(io, motor.id, def, &current, &result.error)) {
          return result;
        }
        motor_ops.push_back({motor.id, {&def, current}, {&def, current}});
      }
    }
    std::sort(motor_ops.begin(), motor_ops.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.desired.def->reg.addr < rhs.desired.def->reg.addr;
              });
    operations.insert(operations.end(), motor_ops.begin(), motor_ops.end());
  }

  for (const auto& operation : operations) {
    if (!writeParameter(io, operation.id, operation.desired, &result.error)) {
      result.rollback_attempted = true;
      result.rollback_ok = true;
      // Reapply originals in address order: OperatingMode must precede the
      // gains/profile values that it resets. Include the failing operation:
      // a lost status packet does not prove that its write was not applied.
      for (const auto& rollback : operations) {
        std::string rollback_error;
        if (!writeParameter(io, rollback.id, rollback.original,
                            &rollback_error)) {
          result.rollback_ok = false;
          result.error += "; rollback failed: " + rollback_error;
        }
      }
      return result;
    }
    ++result.changed;
  }
  result.ok = true;
  return result;
}

}  // namespace rtctrl::apps::dxl_parameters
