// Standalone CRANE-X7 bus emulator: serves the motor set from
// config/crane_x7.toml over a pseudo-terminal so any Dynamixel client
// (dxl_inspect, the bring-up apps, rt tools) can run without hardware.
//
// Usage: dxl_emu [--config config/crane_x7.toml] [--link /tmp/ttyDXL]
//                [--firmware N]
//
// --firmware sets every motor's Firmware Version (default 44); 45 and
// later enable Drive Mode's Torque On by Goal Update.

#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <toml++/toml.hpp>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"

namespace {
volatile std::sig_atomic_t g_stop = 0;
void onSignal(int) { g_stop = 1; }
}  // namespace

int main(int argc, char* argv[]) {
  std::string config_path = "config/crane_x7.toml";
  std::string link_path;
  int firmware = rtctrl::emu::MotorEmulator::Config{}.firmware_version;
  for (int i = 1; i < argc - 1; ++i) {
    if (std::strcmp(argv[i], "--config") == 0) config_path = argv[i + 1];
    if (std::strcmp(argv[i], "--link") == 0) link_path = argv[i + 1];
    if (std::strcmp(argv[i], "--firmware") == 0) {
      char* end = nullptr;
      const long value = std::strtol(argv[i + 1], &end, 10);
      if (end == argv[i + 1] || *end != '\0' || value < 0 || value > 255) {
        std::fprintf(stderr, "--firmware: an integer in [0, 255]\n");
        return 2;
      }
      firmware = static_cast<int>(value);
    }
  }

  rtctrl::emu::MotorBus bus;
  try {
    const auto tbl = toml::parse_file(config_path);
    const auto* joints = tbl["joint"].as_array();
    if (joints == nullptr) throw std::runtime_error("no [[joint]] entries");
    for (const auto& entry : *joints) {
      const auto* joint = entry.as_table();
      rtctrl::emu::MotorEmulator::Config config;
      config.id = static_cast<std::uint8_t>(
          joint->at("id").value<int>().value());
      const auto model = joint->at("model").value<std::string>().value();
      config.model_number = (model == "XM540-W270")
                                ? rtctrl::dxl::kModelXm540W270
                                : rtctrl::dxl::kModelXm430W350;
      config.firmware_version = static_cast<std::uint8_t>(firmware);
      bus.add(config);
      std::printf("motor id %u (%s, firmware v%d)\n", config.id,
                  model.c_str(), firmware);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error reading %s: %s\n", config_path.c_str(),
                 e.what());
    return 1;
  }

  rtctrl::emu::PtyBus pty(bus);
  std::printf("serving on %s\n", pty.slavePath().c_str());
  if (!link_path.empty()) {
    unlink(link_path.c_str());
    if (symlink(pty.slavePath().c_str(), link_path.c_str()) == 0) {
      std::printf("linked as %s\n", link_path.c_str());
    }
  }
  std::fflush(stdout);

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  constexpr double kDt = 0.001;
  while (g_stop == 0) {
    if (!pty.poll(kDt)) break;
    usleep(1000);
  }
  if (!link_path.empty()) unlink(link_path.c_str());
  return 0;
}
