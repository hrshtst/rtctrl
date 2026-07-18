#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rtctrl::hw {

struct JointConfig {
  std::string name;                 // URDF joint name (canonical identity)
  std::uint8_t id = 0;              // DXL bus id
  std::uint16_t model_number = 0;   // resolved from the config's model string
  std::uint8_t operating_mode = 3;  // raw DXL value: 3=pos, 1=vel, 0=current
  double velocity_limit = 0.0;      // [rad/s]
  double effort_limit = 0.0;        // [Nm]
  double pos_limit_margin = 0.0;    // [rad]
  double current_limit_margin = 0.0;  // [A]
};

struct Config {
  std::string port = "/dev/ttyUSB0";
  int baudrate = 3000000;
  std::vector<JointConfig> joints;

  // Parses config/crane_x7.toml (throws std::runtime_error with context
  // on malformed input). Joint order in the file is the canonical order.
  static Config load(const std::string& toml_path);
};

}  // namespace rtctrl::hw
