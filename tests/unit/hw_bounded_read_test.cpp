// The bus-profile discovery and arming of the bounded read (control-rate
// Phase R3, contract C-4a) in CraneX7::activate(), over the in-process
// emulator: what is checked, in which order relative to torque-on, and
// what a refusal leaves behind.
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/hw/bus_profile.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;

namespace {

hw::Config craneConfig() { return hw::Config::load("config/crane_x7.toml"); }

emu::MotorBus busFor(const hw::Config& config, std::uint8_t firmware = 44) {
  emu::MotorBus bus;
  for (const auto& joint : config.joints) {
    emu::MotorEmulator::Config c;
    c.id = joint.id;
    c.model_number = joint.model_number;
    c.firmware_version = firmware;
    bus.add(c);
  }
  return bus;
}

// The emulator starts every servo at the factory Return Delay Time (250);
// the arm runs at 0.
void setReturnDelay(emu::MotorBus& bus, std::uint8_t value) {
  for (auto& motor : bus.motors()) motor.poke(reg::kReturnDelayTime, value);
}

// A transport that can arm a bounded read, logging torque writes, the
// watchdog arming and the bounded read's arming in order.
class BoundedIO : public emu::FakePacketIO {
 public:
  explicit BoundedIO(emu::MotorBus& bus) : emu::FakePacketIO(bus) {}

  dxl::IoResult write(std::uint8_t id, std::uint16_t addr,
                      const std::uint8_t* data, std::uint16_t len) override {
    if (addr == reg::kTorqueEnable.addr && len == 1) {
      events.push_back(data[0] != 0 ? "torque-on" : "torque-off");
    }
    if (addr == reg::kBusWatchdog.addr && len == 1 && data[0] != 0) {
      events.push_back("watchdog");
      if (id == watchdog_ignored_by) {
        const std::uint8_t other = 4;  // the servo keeps another interval
        return emu::FakePacketIO::write(id, addr, &other, 1);
      }
    }
    return emu::FakePacketIO::write(id, addr, data, len);
  }
  bool armBoundedSyncRead(const dxl::BoundedReadConfig& config) override {
    if (!can_bound) return false;
    armed = config;
    events.push_back("arm");
    return true;
  }
  void disarmBoundedSyncRead() override { events.push_back("disarm"); }
  std::optional<int> latencyTimerMs() const override { return latency; }

  bool has(const std::string& event) const {
    return std::find(events.begin(), events.end(), event) != events.end();
  }
  std::size_t first(const std::string& event) const {
    return static_cast<std::size_t>(
        std::find(events.begin(), events.end(), event) - events.begin());
  }
  std::size_t last(const std::string& event) const {
    const auto it = std::find(events.rbegin(), events.rend(), event);
    return static_cast<std::size_t>(events.rend() - it) - 1;
  }

  std::vector<std::string> events;
  dxl::BoundedReadConfig armed;
  bool can_bound = true;
  std::optional<int> latency;
  std::uint8_t watchdog_ignored_by = 0;
};

// 20 + 20 + 10 + 10 = 60 ms of silence at most, against the default
// 100 ms watchdog less one 20 ms unit: 80 ms.
hw::CraneX7::Options boundedOptions() {
  hw::CraneX7::Options options;
  options.bounded_read.enabled = true;
  options.bounded_read.budget = milliseconds(20);
  options.bounded_read.guard = milliseconds(20);
  options.bounded_read.scheduling_allowance = milliseconds(10);
  options.bounded_read.transport_allowance = milliseconds(10);
  return options;
}

bool contains(const std::string& text, const std::string& part) {
  return text.find(part) != std::string::npos;
}

}  // namespace

TEST_CASE("disabled, activation checks no bus profile and arms nothing",
          "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config);  // Return Delay Time 250, as from the factory
  BoundedIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());
  CHECK_FALSE(io.has("arm"));
  CHECK_FALSE(arm.boundedReadArmed());
  CHECK(arm.busDiscovery().profile.servos.empty());
  REQUIRE(arm.deactivate());
  CHECK_FALSE(io.has("disarm"));
}

TEST_CASE("a matching bus arms the bounded read after the watchdog and "
          "before torque-on",
          "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  setReturnDelay(bus, 0);
  BoundedIO io(bus);
  hw::CraneX7 arm(io, config, boundedOptions());
  REQUIRE(arm.activate());

  REQUIRE(io.has("arm"));
  CHECK(io.first("arm") > io.last("watchdog"));
  CHECK(io.first("arm") < io.first("torque-on"));
  CHECK(io.armed.addr == dxl::SyncGroup::kFeedbackAddr);
  CHECK(io.armed.len == dxl::SyncGroup::kFeedbackSlots);
  const std::vector<std::uint8_t> ids{2, 3, 4, 5, 6, 7, 8, 9};
  CHECK(io.armed.ids == ids);
  CHECK(io.armed.budget == milliseconds(20));
  CHECK(io.armed.guard == milliseconds(20));
  CHECK(arm.boundedReadArmed());

  const auto& d = arm.busDiscovery();
  REQUIRE(d.profile.servos.size() == 8);
  for (const auto& servo : d.profile.servos) {
    CAPTURE(static_cast<int>(servo.id));
    CHECK(servo.return_delay_time == 0);
    CHECK(servo.status_return_level == 2);
    CHECK(servo.baud_register == 5);  // 3 Mbps
    CHECK(servo.firmware == 44);
  }
  // 100 ms in 20 ms units, read back from every joint.
  CHECK(d.bus_watchdog == std::vector<int>(8, 5));
  CHECK_FALSE(d.profile.latency_timer_ms.has_value());  // unverified
  CHECK(d.max_instruction_gap_s == 0.060);
  CHECK(d.watchdog_gap_limit_s == 0.080);
  CHECK(d.armed);

  REQUIRE(arm.deactivate());
  CHECK(io.events.back() == "disarm");
  CHECK_FALSE(arm.boundedReadArmed());
}

TEST_CASE("activation refuses a bus the bounded read cannot rely on, "
          "with no torque-on and nothing armed",
          "[hw][bounded]") {
  const auto config = craneConfig();
  std::uint8_t firmware = 44;
  hw::CraneX7::Options options = boundedOptions();
  std::string expected_error;
  bool watchdog_programmed = false;  // refused after the watchdog arm
  auto bus = busFor(config, firmware);
  setReturnDelay(bus, 0);
  BoundedIO io(bus);

  SECTION("the factory Return Delay Time") {
    setReturnDelay(bus, 250);
    expected_error = "Return Delay Time 250 (500 us)";
  }
  SECTION("another baud rate on one servo") {
    bus.find(5)->poke(reg::kBaudRate, 4);  // 2 Mbps
    expected_error = "id 5 has Baud Rate register 4";
  }
  SECTION("a servo that does not reply to reads") {
    bus.find(7)->poke(reg::kStatusReturnLevel, 0);
    expected_error = "id 7 has Status Return Level 0";
  }
  SECTION("Fast Sync Read on firmware older than v45") {
    options.bounded_read.profile.fast_sync_read = true;
    expected_error = "lacks Fast Sync Read";
  }
  SECTION("an adapter latency timer of 16 ms") {
    io.latency = 16;
    expected_error = "latency timer is 16 ms";
  }
  SECTION("a servo that keeps another watchdog interval") {
    io.watchdog_ignored_by = 6;
    expected_error = "id 6 Bus Watchdog reads 4, not the programmed 5";
    watchdog_programmed = true;
  }
  SECTION("a transport that cannot bound its read") {
    io.can_bound = false;
    expected_error = "cannot bound its grouped read";
    watchdog_programmed = true;
  }

  hw::CraneX7 arm(io, config, options);
  REQUIRE_FALSE(arm.activate());
  CAPTURE(arm.lastError());
  CHECK(contains(arm.lastError(), expected_error));
  CHECK_FALSE(io.has("torque-on"));
  CHECK_FALSE(io.has("arm"));
  CHECK_FALSE(arm.boundedReadArmed());
  // A profile refused before any write stops before the watchdog too.
  CHECK(io.has("watchdog") == watchdog_programmed);
  for (const auto& motor : bus.motors()) {
    CHECK(motor.peek(reg::kTorqueEnable) == 0);
  }
}

TEST_CASE("Fast Sync Read is accepted from firmware v45", "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config, 45);
  setReturnDelay(bus, 0);
  BoundedIO io(bus);
  hw::CraneX7::Options options = boundedOptions();
  options.bounded_read.profile.fast_sync_read = true;
  io.latency = 1;  // exposed and right
  hw::CraneX7 arm(io, config, options);
  REQUIRE(arm.activate());
  CHECK(arm.boundedReadArmed());
  CHECK(arm.busDiscovery().profile.latency_timer_ms == 1);
}

TEST_CASE("the longest bus silence must stay below the programmed watchdog "
          "interval less one unit",
          "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  setReturnDelay(bus, 0);
  BoundedIO io(bus);
  hw::CraneX7::Options options = boundedOptions();

  SECTION("exactly at the 80 ms limit is refused before any write") {
    options.bounded_read.guard = milliseconds(40);  // 20+40+10+10 = 80
    hw::CraneX7 arm(io, config, options);
    REQUIRE_FALSE(arm.activate());
    CHECK(contains(arm.lastError(), "is not below 80.000000 ms"));
    CHECK(arm.busDiscovery().max_instruction_gap_s == 0.080);
    CHECK_FALSE(io.has("watchdog"));
    CHECK_FALSE(io.has("torque-on"));
  }
  SECTION("one nanosecond less is accepted") {
    options.bounded_read.guard = milliseconds(40) - nanoseconds(1);
    hw::CraneX7 arm(io, config, options);
    CHECK(arm.activate());
  }
  SECTION("a longer programmed watchdog accepts the same silence") {
    options.bounded_read.guard = milliseconds(40);
    options.bus_watchdog_timeout_s = 0.2;  // 10 units: 180 ms allowed
    hw::CraneX7 arm(io, config, options);
    REQUIRE(arm.activate());
    CHECK(arm.busDiscovery().watchdog_gap_limit_s == 0.180);
    CHECK(arm.busDiscovery().bus_watchdog.front() == 10);
  }
  SECTION("an allowance left at zero is refused") {
    options.bounded_read.transport_allowance = nanoseconds(0);
    hw::CraneX7 arm(io, config, options);
    REQUIRE_FALSE(arm.activate());
    CHECK(contains(arm.lastError(), "allowances must be positive"));
  }
}

TEST_CASE("the bus profile is checked again after the activation "
          "configurator",
          "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  setReturnDelay(bus, 0);
  BoundedIO io(bus);
  hw::CraneX7::Options options = boundedOptions();
  // A parameter set that restores the factory Return Delay Time on id 4.
  options.activation_configurator = [](dxl::PacketIO& io, std::string*) {
    return io.write8(4, reg::kReturnDelayTime.addr, 250).ok();
  };
  hw::CraneX7 arm(io, config, options);
  REQUIRE_FALSE(arm.activate());
  CHECK(contains(arm.lastError(), "id 4 has Return Delay Time 250"));
  CHECK_FALSE(io.has("torque-on"));
  CHECK_FALSE(io.has("arm"));
}

TEST_CASE("a bounded read armed by a failing activation is disarmed",
          "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  setReturnDelay(bus, 0);
  BoundedIO io(bus);
  io.setWriteFailureOn(9, reg::kTorqueEnable.addr, 1, -3001);
  hw::CraneX7 arm(io, config, boundedOptions());
  REQUIRE_FALSE(arm.activate());
  REQUIRE(io.has("arm"));
  CHECK(io.last("disarm") > io.first("arm"));
  CHECK_FALSE(arm.boundedReadArmed());
}

TEST_CASE("the destructor disarms a bounded read", "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  setReturnDelay(bus, 0);
  BoundedIO io(bus);
  {
    hw::CraneX7 arm(io, config, boundedOptions());
    REQUIRE(arm.activate());
    REQUIRE(io.events.back() != "disarm");
  }
  CHECK(io.events.back() == "disarm");
}

TEST_CASE("a refused activation keeps the Bus Watchdog of a servo whose "
          "stop is unconfirmed",
          "[hw][bounded][safety]") {
  // Every servo starts torque-on with an armed watchdog (a session that
  // ended without its release), and activation is refused on the factory
  // Return Delay Time after Drive Mode was verified, so the release writes
  // zero goal current, torque off and the watchdog clear.
  const auto config = craneConfig();
  auto bus = busFor(config, 47);
  for (auto& motor : bus.motors()) {
    motor.poke(reg::kTorqueEnable, 1);
    motor.poke(reg::kBusWatchdog, 5);
  }
  emu::FakePacketIO io(bus);
  SECTION("its torque-off write fails") {
    io.setWriteFailureOn(9, reg::kTorqueEnable.addr, 0, -3001);
  }
  SECTION("its zero goal current write fails") {
    io.setWriteFailureOn(9, reg::kGoalCurrent.addr, 0, -3001);
  }
  hw::CraneX7 arm(io, config, boundedOptions());
  REQUIRE_FALSE(arm.activate());
  REQUIRE(contains(arm.lastError(), "Return Delay Time 250"));
  CHECK(bus.find(9)->peek(reg::kBusWatchdog) == 5);  // still protected
  for (std::uint8_t id = 2; id <= 8; ++id) {
    CAPTURE(static_cast<int>(id));
    CHECK(bus.find(id)->peek(reg::kTorqueEnable) == 0);
    CHECK(bus.find(id)->peek(reg::kBusWatchdog) == 0);
  }
}

namespace {

// Counts every transaction that could change a servo.
class WriteCountIO : public emu::FakePacketIO {
 public:
  explicit WriteCountIO(emu::MotorBus& bus) : emu::FakePacketIO(bus) {}
  dxl::IoResult write(std::uint8_t id, std::uint16_t addr,
                      const std::uint8_t* data, std::uint16_t len) override {
    ++writes;
    return emu::FakePacketIO::write(id, addr, data, len);
  }
  dxl::IoResult syncWrite(std::uint16_t addr, std::uint16_t len,
                          const std::vector<std::uint8_t>& ids,
                          const std::vector<std::uint8_t>& data) override {
    ++writes;
    return emu::FakePacketIO::syncWrite(addr, len, ids, data);
  }
  int writes = 0;
};

}  // namespace

TEST_CASE("the bus profile check only reads", "[hw][bounded]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  WriteCountIO io(bus);
  const std::vector<std::uint8_t> ids{2, 3, 4, 5, 6, 7, 8, 9};
  hw::BusProfileReport report;
  std::string error;
  SECTION("a matching bus") {
    setReturnDelay(bus, 0);
    CHECK(hw::checkBusProfile(io, ids, 3000000, hw::BusProfile{}, &report,
                              &error));
    CHECK(report.servos.size() == 8);
  }
  SECTION("a refused bus") {
    CHECK_FALSE(hw::checkBusProfile(io, ids, 3000000, hw::BusProfile{},
                                    &report, &error));
    CHECK(contains(error, "id 2 has Return Delay Time 250"));
    CHECK(report.servos.size() == 1);  // what was read until the refusal
  }
  SECTION("a line speed the servos cannot run") {
    CHECK_FALSE(hw::checkBusProfile(io, ids, 250000, hw::BusProfile{},
                                    &report, &error));
    CHECK(contains(error, "no Baud Rate register value"));
  }
  CHECK(io.writes == 0);
}
