// Shared plumbing for the x7_* bring-up apps: config/port CLI parsing
// and a CraneX7 wired to silence the bus on deadman escalation.
#pragma once

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "rtctrl/dxl/port.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace x7 {

struct Cli {
  std::string config_path = "config/crane_x7.toml";
  std::string port_override;
  int argi = 1;  // first unconsumed argument
};

inline Cli parseCli(int argc, char* argv[]) {
  Cli cli;
  int i = 1;
  for (; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], "--config") == 0) {
      cli.config_path = argv[++i];
    } else if (std::strcmp(argv[i], "--port") == 0) {
      cli.port_override = argv[++i];
    } else {
      break;
    }
  }
  cli.argi = i;
  return cli;
}

struct Session {
  rtctrl::hw::Config config;
  std::unique_ptr<rtctrl::dxl::Port> port;
  std::unique_ptr<rtctrl::hw::CraneX7> arm;
};

// operating_mode_override: -1 keeps the config's modes; 0/1/3 forces
// that mode on every joint (e.g. x7_float forces current mode).
inline Session openSession(const Cli& cli, int operating_mode_override = -1) {
  Session s;
  s.config = rtctrl::hw::Config::load(cli.config_path);
  if (!cli.port_override.empty()) s.config.port = cli.port_override;
  if (operating_mode_override >= 0) {
    for (auto& joint : s.config.joints) {
      joint.operating_mode =
          static_cast<std::uint8_t>(operating_mode_override);
    }
  }
  std::printf("bus: %s @ %d baud, %zu joints\n", s.config.port.c_str(),
              s.config.baudrate, s.config.joints.size());
  s.port = std::make_unique<rtctrl::dxl::Port>(s.config.port,
                                               s.config.baudrate);
  s.arm = std::make_unique<rtctrl::hw::CraneX7>(*s.port, s.config);
  // Deadman escalation ends with a silent bus so the servo-side
  // watchdogs are guaranteed to fire.
  rtctrl::dxl::Port* port = s.port.get();
  s.arm->onEscalate([port] {
    std::fprintf(stderr,
                 "DEADMAN: command stream stale — closing the bus; servo "
                 "watchdogs will halt the arm\n");
    port->close();
  });
  return s;
}

}  // namespace x7
