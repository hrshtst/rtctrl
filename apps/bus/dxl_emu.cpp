// Standalone CRANE-X7 bus emulator: serves the motor set from
// config/crane_x7.toml over a pseudo-terminal so any Dynamixel client
// (dxl_inspect, the bring-up apps, rt tools) can run without hardware.
//
// Usage: dxl_emu [--config config/crane_x7.toml] [--link /tmp/ttyDXL]
//                [--firmware N]
//
// --firmware sets every motor's Firmware Version (default 44); 45 and
// later enable Drive Mode's Torque On by Goal Update.
//
// Timing (emu::StandaloneLoop): input is read when it arrives, not on a
// polling sleep; the motor model ticks every 1 ms of wall time (steady
// clock), each step running its due ticks before the commands it reads.
// If the loop falls more than kMaxCatchUpTicks ticks behind (it was kept
// from running for longer than that), model time cannot be kept on wall
// time: dxl_emu says so and exits with status 3. SIGINT or SIGTERM stops
// it (status 0); a pty failure exits with status 1. At exit it prints its
// model time against the wall time since it started serving.

#include <unistd.h>

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <toml++/toml.hpp>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"
#include "rtctrl/emu/standalone_loop.hpp"

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

  // No SA_RESTART: a signal ends the loop's wait at once (EINTR).
  struct sigaction action {};
  action.sa_handler = onSignal;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);

  constexpr std::uint64_t kMaxCatchUpTicks = 50;  // 50 ms of model time
  rtctrl::emu::StandaloneLoop::Config loop_config;
  loop_config.tick = std::chrono::milliseconds(1);
  loop_config.max_catch_up_ticks = kMaxCatchUpTicks;
  loop_config.max_read_bytes = 4096;
  const auto start = rtctrl::emu::StandaloneLoop::Clock::now();
  rtctrl::emu::StandaloneLoop loop(pty, bus, loop_config, start);
  const auto status = loop.run([] { return g_stop != 0; });
  const double wall_s = std::chrono::duration<double>(
                            rtctrl::emu::StandaloneLoop::Clock::now() - start)
                            .count();
  if (!link_path.empty()) unlink(link_path.c_str());

  const auto& st = loop.stats();
  const double tick_s =
      std::chrono::duration<double>(loop_config.tick).count();
  std::printf(
      "dxl_emu: model time %.3f s (%llu ticks of %.3f ms) over %.3f s of "
      "wall time; steps %llu, of them late (more than one tick) %llu, most "
      "ticks at once %llu; reads at the %zu-byte bound %llu; waits by time "
      "alone (no client) %llu\n",
      static_cast<double>(st.ticks) * tick_s,
      static_cast<unsigned long long>(st.ticks), tick_s * 1e3, wall_s,
      static_cast<unsigned long long>(st.steps),
      static_cast<unsigned long long>(st.late_steps),
      static_cast<unsigned long long>(st.max_ticks_in_step),
      loop_config.max_read_bytes,
      static_cast<unsigned long long>(st.limited_reads),
      static_cast<unsigned long long>(st.hangup_waits));
  std::fflush(stdout);
  switch (status) {
    case rtctrl::emu::StandaloneLoop::Status::kOk:
      return 0;
    case rtctrl::emu::StandaloneLoop::Status::kOverrun:
      std::fprintf(stderr,
                   "dxl_emu: %llu model ticks were due at once, more than "
                   "the cap of %llu (%.0f ms): it was kept from running too "
                   "long to keep model time on wall time; stopping\n",
                   static_cast<unsigned long long>(st.overrun_due),
                   static_cast<unsigned long long>(kMaxCatchUpTicks),
                   static_cast<double>(kMaxCatchUpTicks) * tick_s * 1e3);
      return 3;
    case rtctrl::emu::StandaloneLoop::Status::kIoError:
      std::fprintf(stderr, "dxl_emu: the pseudo-terminal failed; stopping\n");
      return 1;
  }
  return 1;
}
