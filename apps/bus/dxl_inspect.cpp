// Motor inspection tool: bus scan, register read/write by name or raw
// address, full parameter dump. Works identically against real hardware
// and the pty emulator (dxl_emu).
//
// Usage:
//   dxl_inspect --port <dev> [--baud 3000000] scan
//   dxl_inspect --port <dev> dump <id>
//   dxl_inspect --port <dev> dump-params <file|-> [id ...]
//   dxl_inspect --port <dev> read <id> <register|addr> [len]
//   dxl_inspect --port <dev> write <id> <register|addr> <value> [len]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "bus/dxl_parameters.hpp"
#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/dxl/port.hpp"

namespace dxl = rtctrl::dxl;
namespace reg = rtctrl::dxl::reg;
namespace params = rtctrl::apps::dxl_parameters;

namespace {

const std::pair<const char*, dxl::Reg> kRegisters[] = {
    {"model_number", reg::kModelNumber},
    {"firmware_version", reg::kFirmwareVersion},
    {"id", reg::kId},
    {"baud_rate", reg::kBaudRate},
    {"return_delay_time", reg::kReturnDelayTime},
    {"drive_mode", reg::kDriveMode},
    {"operating_mode", reg::kOperatingMode},
    {"secondary_id", reg::kSecondaryId},
    {"protocol_type", reg::kProtocolType},
    {"homing_offset", reg::kHomingOffset},
    {"moving_threshold", reg::kMovingThreshold},
    {"temperature_limit", reg::kTemperatureLimit},
    {"max_voltage_limit", reg::kMaxVoltageLimit},
    {"min_voltage_limit", reg::kMinVoltageLimit},
    {"pwm_limit", reg::kPwmLimit},
    {"current_limit", reg::kCurrentLimit},
    {"velocity_limit", reg::kVelocityLimit},
    {"max_position_limit", reg::kMaxPositionLimit},
    {"min_position_limit", reg::kMinPositionLimit},
    {"external_port_mode_1", reg::kExternalPortMode1},
    {"external_port_mode_2", reg::kExternalPortMode2},
    {"external_port_mode_3", reg::kExternalPortMode3},
    {"startup_configuration", reg::kStartupConfiguration},
    {"shutdown", reg::kShutdown},
    {"torque_enable", reg::kTorqueEnable},
    {"status_return_level", reg::kStatusReturnLevel},
    {"hardware_error_status", reg::kHardwareErrorStatus},
    {"velocity_i_gain", reg::kVelocityIGain},
    {"velocity_p_gain", reg::kVelocityPGain},
    {"position_d_gain", reg::kPositionDGain},
    {"position_i_gain", reg::kPositionIGain},
    {"position_p_gain", reg::kPositionPGain},
    {"feedforward_2nd_gain", reg::kFeedforward2ndGain},
    {"feedforward_1st_gain", reg::kFeedforward1stGain},
    {"bus_watchdog", reg::kBusWatchdog},
    {"goal_current", reg::kGoalCurrent},
    {"goal_velocity", reg::kGoalVelocity},
    {"profile_acceleration", reg::kProfileAcceleration},
    {"profile_velocity", reg::kProfileVelocity},
    {"goal_position", reg::kGoalPosition},
    {"moving", reg::kMoving},
    {"present_current", reg::kPresentCurrent},
    {"present_velocity", reg::kPresentVelocity},
    {"present_position", reg::kPresentPosition},
    {"present_input_voltage", reg::kPresentInputVoltage},
    {"present_temperature", reg::kPresentTemperature},
};

bool lookupRegister(const std::string& name, dxl::Reg* out) {
  for (const auto& [reg_name, r] : kRegisters) {
    if (name == reg_name) {
      *out = r;
      return true;
    }
  }
  char* end = nullptr;
  const long addr = std::strtol(name.c_str(), &end, 0);
  if (end != nullptr && *end == '\0' && addr >= 0) {
    *out = {static_cast<std::uint16_t>(addr), 1};
    return true;
  }
  return false;
}

std::uint32_t readValue(dxl::Port& port, std::uint8_t id, dxl::Reg r,
                        dxl::IoResult* result) {
  std::uint8_t buf[4] = {};
  *result = port.read(id, r.addr, buf, r.len);
  std::uint32_t value = 0;
  for (int i = r.len - 1; i >= 0; --i) value = (value << 8) | buf[i];
  return value;
}

int scan(dxl::Port& port) {
  int found = 0;
  for (int id = 0; id <= 30; ++id) {
    std::uint16_t model = 0;
    const auto r = port.ping(static_cast<std::uint8_t>(id), &model);
    if (r.ok()) {
      std::printf("id %3d: model %u\n", id, model);
      ++found;
    }
  }
  std::printf("%d servo(s) found\n", found);
  return found > 0 ? 0 : 1;
}

int dump(dxl::Port& port, std::uint8_t id) {
  for (const auto& [name, r] : kRegisters) {
    dxl::IoResult result;
    const auto value = readValue(port, id, r, &result);
    if (!result.ok()) {
      std::printf("%-24s <comm %d, error 0x%02X>\n", name, result.comm,
                  result.error);
      continue;
    }
    std::printf("%-24s %10u", name, value);
    if (r.addr == reg::kPresentPosition.addr ||
        r.addr == reg::kGoalPosition.addr) {
      std::printf("  (%.4f rad)",
                  dxl::pulseToRad(static_cast<std::int32_t>(value)));
    } else if (r.addr == reg::kPresentCurrent.addr ||
               r.addr == reg::kGoalCurrent.addr) {
      std::printf("  (%.3f A)",
                  dxl::currentToAmps(static_cast<std::int16_t>(value)));
    } else if (r.addr == reg::kPresentVelocity.addr) {
      std::printf("  (%.3f rad/s)",
                  dxl::velocityToRadPerSec(static_cast<std::int32_t>(value)));
    } else if (r.addr == reg::kPresentInputVoltage.addr) {
      std::printf("  (%.1f V)",
                  dxl::voltageToVolts(static_cast<std::uint16_t>(value)));
    }
    std::printf("\n");
  }
  return 0;
}

std::uint8_t parseId(const char* text) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);
  if (end == text || *end != '\0' || value < 0 || value > 252) {
    throw std::runtime_error("invalid motor id '" + std::string(text) +
                             "' (expected 0..252)");
  }
  return static_cast<std::uint8_t>(value);
}

int dumpParameters(dxl::Port& port, const std::string& path,
                   const std::vector<std::uint8_t>& selected_ids) {
  std::vector<std::uint8_t> ids = selected_ids;
  if (ids.empty()) {
    for (int id = 0; id <= 252; ++id) {
      if (port.ping(static_cast<std::uint8_t>(id), nullptr).ok()) {
        ids.push_back(static_cast<std::uint8_t>(id));
      }
    }
  }
  if (ids.empty()) throw std::runtime_error("no motors found");

  params::ParameterDump parameter_dump;
  for (const auto id : ids) {
    params::MotorRecord motor;
    std::string error;
    if (!params::captureMotor(port, id, &motor, &error)) {
      throw std::runtime_error(error);
    }
    parameter_dump.motors.push_back(std::move(motor));
  }
  const auto document = params::serialize(parameter_dump);
  if (path == "-") {
    std::fwrite(document.data(), 1, document.size(), stdout);
  } else {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    if (!output) throw std::runtime_error("cannot open '" + path + "'");
    output << document;
    if (!output) throw std::runtime_error("cannot write '" + path + "'");
    std::printf("wrote %s (%zu motor(s))\n", path.c_str(), ids.size());
  }
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string device;
  int baud = 3000000;
  int argi = 1;
  for (; argi < argc - 1; ++argi) {
    if (std::strcmp(argv[argi], "--port") == 0) {
      device = argv[++argi];
    } else if (std::strcmp(argv[argi], "--baud") == 0) {
      baud = std::atoi(argv[++argi]);
    } else {
      break;
    }
  }
  if (device.empty() || argi >= argc) {
    std::fprintf(stderr,
                 "usage: dxl_inspect --port <dev> [--baud N] "
                 "scan | dump <id> | dump-params <file|-> [id ...] | "
                 "read <id> <reg> [len] | "
                 "write <id> <reg> <value> [len]\n");
    return 2;
  }

  try {
    dxl::Port port(device, baud);
    const std::string cmd = argv[argi++];
    if (cmd == "scan") return scan(port);
    if (cmd == "dump-params") {
      if (argi >= argc) throw std::runtime_error("missing output file");
      const std::string path = argv[argi++];
      std::vector<std::uint8_t> ids;
      std::set<std::uint8_t> unique;
      while (argi < argc) {
        const auto id = parseId(argv[argi++]);
        if (!unique.insert(id).second) {
          throw std::runtime_error("duplicate motor id " + std::to_string(id));
        }
        ids.push_back(id);
      }
      return dumpParameters(port, path, ids);
    }

    if (argi >= argc) throw std::runtime_error("missing id");
    const auto id = parseId(argv[argi++]);
    if (cmd == "dump") return dump(port, id);

    if (argi >= argc) throw std::runtime_error("missing register");
    dxl::Reg r{};
    if (!lookupRegister(argv[argi++], &r)) {
      throw std::runtime_error("unknown register");
    }
    if (cmd == "read") {
      if (argi < argc) r.len = static_cast<std::uint16_t>(std::atoi(argv[argi]));
      dxl::IoResult result;
      const auto value = readValue(port, id, r, &result);
      if (!result.ok()) {
        std::fprintf(stderr, "comm %d, error 0x%02X\n", result.comm,
                     result.error);
        return 1;
      }
      std::printf("%u\n", value);
      return 0;
    }
    if (cmd == "write") {
      if (argi >= argc) throw std::runtime_error("missing value");
      const auto value =
          static_cast<std::uint32_t>(std::strtoul(argv[argi++], nullptr, 0));
      if (argi < argc) r.len = static_cast<std::uint16_t>(std::atoi(argv[argi]));
      std::uint8_t buf[4];
      for (int i = 0; i < r.len; ++i) {
        buf[i] = static_cast<std::uint8_t>(value >> (8 * i));
      }
      const auto result = port.write(id, r.addr, buf, r.len);
      if (!result.ok()) {
        std::fprintf(stderr, "comm %d, error 0x%02X\n", result.comm,
                     result.error);
        return 1;
      }
      return 0;
    }
    throw std::runtime_error("unknown command: " + cmd);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
