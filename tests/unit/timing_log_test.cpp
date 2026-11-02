#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "common/timing_log.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;

namespace {

emu::MotorBus busFor(const hw::Config& config) {
  emu::MotorBus bus;
  for (const auto& joint : config.joints) {
    emu::MotorEmulator::Config c;
    c.id = joint.id;
    c.model_number = joint.model_number;
    c.firmware_version = 44;
    bus.add(c);
  }
  return bus;
}

std::vector<std::string> readLines(const std::string& path) {
  std::ifstream in(path);
  std::vector<std::string> lines;
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

}  // namespace

TEST_CASE("the cycle timing log holds one row per background cycle",
          "[hw][timing]") {
  const std::string path = "build/timing_log_test.csv";
  std::filesystem::remove(path);
  const auto config = hw::Config::load("config/crane_x7.toml");
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.control_cycle_s = 0.002;
  options.timing_ring_capacity = x7::timingRingCapacity(500.0);
  hw::CraneX7 crane(io, config, options);
  bool complete = false;
  std::uint64_t cycles = 0;
  {
    x7::CycleTimingLog log(crane, path);
    REQUIRE(crane.activate());
    REQUIRE(crane.startThread());
    log.start();
    std::uint64_t seen = 0;
    while (seen < 25) seen = crane.waitCycle(seen);
    crane.stopThread();
    REQUIRE(crane.deactivate());
    complete = log.stop();
    CHECK(log.stop() == complete);  // idempotent
    cycles = crane.cycleStats().cycles;
  }
  CHECK(complete);
  const auto lines = readLines(path);
  REQUIRE(lines.size() == cycles + 1);
  CHECK(lines.front() == x7::kCycleRecordCsvHeader);
  const auto columns = [](const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    for (std::string cell; std::getline(ss, cell, ',');) out.push_back(cell);
    return out;
  };
  const std::size_t width = columns(lines.front()).size();
  for (std::size_t i = 1; i < lines.size(); ++i) {
    const auto cells = columns(lines[i]);
    REQUIRE(cells.size() == width);
    CHECK(cells.front() == std::to_string(i));
    CHECK(cells.back() == "periodic");
  }
  std::filesystem::remove(path);
}

TEST_CASE("a timing log that lost records reports itself incomplete",
          "[hw][timing]") {
  const std::string path = "build/timing_log_drop_test.csv";
  std::filesystem::remove(path);
  const auto config = hw::Config::load("config/crane_x7.toml");
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.control_cycle_s = 0.002;
  options.timing_ring_capacity = 4;
  hw::CraneX7 crane(io, config, options);
  x7::CycleTimingLog log(crane, path);
  REQUIRE(crane.activate());
  REQUIRE(crane.startThread());
  // No drain thread: the four-record ring fills and drops the rest.
  std::uint64_t seen = 0;
  while (seen < 12) seen = crane.waitCycle(seen);
  crane.stopThread();
  REQUIRE(crane.deactivate());
  CHECK_FALSE(log.stop());
  CHECK(crane.cycleStats().timing_drops > 0);
  CHECK(readLines(path).size() == 1 + 4);
  std::filesystem::remove(path);

  // An existing file is never reused.
  { std::ofstream(path) << "keep\n"; }
  CHECK_THROWS(x7::CycleTimingLog(crane, path));
  CHECK(readLines(path) == std::vector<std::string>{"keep"});
  std::filesystem::remove(path);
}
