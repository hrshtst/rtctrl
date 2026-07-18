#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"

using Catch::Approx;
namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace reg = rtctrl::dxl::reg;

namespace {

emu::MotorBus craneBus() {
  emu::MotorBus bus;
  for (std::uint8_t id = 2; id <= 9; ++id) {
    emu::MotorEmulator::Config config;
    config.id = id;
    config.model_number =
        (id == 3) ? dxl::kModelXm540W270 : dxl::kModelXm430W350;
    bus.add(config);
  }
  return bus;
}

}  // namespace

TEST_CASE("unit conversions round-trip", "[dxl]") {
  CHECK(dxl::pulseToRad(2048) == Approx(0.0));
  CHECK(dxl::radToPulse(0.0) == 2048);
  for (const double rad : {-2.5, -0.7, 0.0, 1.3, 2.9}) {
    CHECK(dxl::pulseToRad(dxl::radToPulse(rad)) == Approx(rad).margin(2e-3));
  }
  for (const double amps : {-2.0, -0.1, 0.0, 0.5, 3.1}) {
    CHECK(dxl::currentToAmps(dxl::ampsToCurrent(amps)) ==
          Approx(amps).margin(2e-3));
  }
  for (const double w : {-3.0, 0.0, 1.5, 4.8}) {
    CHECK(dxl::velocityToRadPerSec(dxl::radPerSecToVelocity(w)) ==
          Approx(w).margin(2e-2));
  }
  CHECK(dxl::torqueConstant(dxl::kModelXm540W270) == Approx(2.409));
  CHECK(dxl::torqueConstant(dxl::kModelXm430W350) == Approx(1.783));
}

TEST_CASE("torque-enable gating", "[emu]") {
  emu::MotorBus bus = craneBus();
  emu::FakePacketIO io(bus);

  // torque off: operating mode and indirect slots accept writes
  CHECK(io.write8(2, reg::kOperatingMode.addr, 0).ok());
  CHECK(io.write16(2, reg::kIndirectAddressBase, 132).ok());

  REQUIRE(io.write8(2, reg::kTorqueEnable.addr, 1).ok());
  // torque on: EEPROM and indirect-address writes rejected with 0x07
  CHECK(io.write8(2, reg::kOperatingMode.addr, 3).error == dxl::kErrAccess);
  CHECK(io.write16(2, reg::kIndirectAddressBase, 116).error ==
        dxl::kErrAccess);
  // RAM goals still writable
  CHECK(io.write32(2, reg::kGoalPosition.addr, 2100).ok());
}

TEST_CASE("mode change resets goals and torque-on snaps goal to present",
          "[emu]") {
  emu::MotorBus bus = craneBus();
  emu::MotorEmulator* motor = bus.find(2);
  emu::FakePacketIO io(bus);

  motor->poke(reg::kPresentPosition, 1500);
  REQUIRE(io.write8(2, reg::kTorqueEnable.addr, 1).ok());
  CHECK(motor->peek(reg::kGoalPosition) == 1500);  // no motion on enable

  REQUIRE(io.write8(2, reg::kTorqueEnable.addr, 0).ok());
  REQUIRE(io.write8(2, reg::kOperatingMode.addr, 0).ok());
  CHECK(motor->peek(reg::kGoalCurrent) == 0);
  CHECK(motor->peek(reg::kGoalVelocity) == 0);
}

TEST_CASE("position mode moves toward the goal, bounded by the profile",
          "[emu]") {
  emu::MotorBus bus = craneBus();
  emu::MotorEmulator* motor = bus.find(2);
  emu::FakePacketIO io(bus);

  REQUIRE(io.write8(2, reg::kTorqueEnable.addr, 1).ok());
  REQUIRE(io.write32(2, reg::kGoalPosition.addr, 3000).ok());
  bus.tick(0.05);
  const auto p1 = motor->peek(reg::kPresentPosition);
  CHECK(p1 > 2048);
  CHECK(p1 < 3000);  // bounded speed, not teleporting
  CHECK(motor->moving());
  for (int i = 0; i < 100; ++i) bus.tick(0.05);
  CHECK(motor->peek(reg::kPresentPosition) == 3000);
  CHECK_FALSE(motor->moving());
}

TEST_CASE("goal writes clamp against limits", "[emu]") {
  emu::MotorBus bus = craneBus();
  emu::MotorEmulator* motor = bus.find(2);
  emu::FakePacketIO io(bus);

  REQUIRE(io.write32(2, reg::kMaxPositionLimit.addr, 3000).ok());
  REQUIRE(io.write32(2, reg::kGoalPosition.addr, 4000).ok());
  CHECK(motor->peek(reg::kGoalPosition) == 3000);

  const auto limit = static_cast<std::int16_t>(motor->peek(reg::kCurrentLimit));
  REQUIRE(io.write16(2, reg::kGoalCurrent.addr,
                     static_cast<std::uint16_t>(limit + 500))
              .ok());
  CHECK(static_cast<std::int16_t>(motor->peek(reg::kGoalCurrent)) == limit);
}

TEST_CASE("indirect data redirects to the mapped registers", "[emu]") {
  emu::MotorBus bus = craneBus();
  emu::MotorEmulator* motor = bus.find(2);
  emu::FakePacketIO io(bus);

  // Map slot 0/1 to PresentPosition low bytes; write through slot to a
  // goal register to prove write redirection too.
  REQUIRE(io.write16(2, reg::kIndirectAddressBase,
                     reg::kPresentPosition.addr).ok());
  motor->poke(reg::kPresentPosition, 0xAB);
  std::uint8_t value = 0;
  REQUIRE(io.read8(2, reg::kIndirectDataBase, &value).ok());
  CHECK(value == 0xAB);

  REQUIRE(io.write16(2, reg::kIndirectAddressBase,
                     reg::kGoalVelocity.addr).ok());
  REQUIRE(io.write8(2, reg::kIndirectDataBase, 0x11).ok());
  CHECK((motor->peek(reg::kGoalVelocity) & 0xFF) == 0x11);
}

TEST_CASE("bus watchdog triggers on silence and not on chatty reads",
          "[emu]") {
  emu::MotorBus bus = craneBus();
  emu::MotorEmulator* motor = bus.find(2);
  emu::FakePacketIO io(bus);

  REQUIRE(io.write8(2, reg::kBusWatchdog.addr, 5).ok());  // 100 ms

  // Chatty reads keep it fed even though nothing is written: the trap
  // case — the servo-side watchdog alone cannot catch a stalled writer.
  std::uint8_t v = 0;
  for (int i = 0; i < 20; ++i) {
    bus.tick(0.05);
    REQUIRE(io.read8(2, reg::kPresentTemperature.addr, &v).ok());
  }
  CHECK_FALSE(motor->watchdogTriggered());

  // Total silence past the timeout: triggered, goals rejected, motion
  // halted; register reads back 0xFF until cleared with 0.
  bus.tick(0.2);
  CHECK(motor->watchdogTriggered());
  CHECK(io.write32(2, reg::kGoalPosition.addr, 2100).error ==
        dxl::kErrAccess);
  REQUIRE(io.read8(2, reg::kBusWatchdog.addr, &v).ok());
  CHECK(v == dxl::kBusWatchdogTriggered);
  REQUIRE(io.write8(2, reg::kBusWatchdog.addr, 0).ok());
  CHECK_FALSE(motor->watchdogTriggered());
  CHECK(io.write32(2, reg::kGoalPosition.addr, 2100).ok());
}

TEST_CASE("bus watchdog refuses to arm on firmware below v38", "[emu]") {
  emu::MotorEmulator::Config config;
  config.id = 2;
  config.firmware_version = 37;
  emu::MotorBus bus;
  bus.add(config);
  emu::FakePacketIO io(bus);

  REQUIRE(io.write8(2, reg::kBusWatchdog.addr, 5).ok());
  std::uint8_t v = 0xEE;
  REQUIRE(io.read8(2, reg::kBusWatchdog.addr, &v).ok());
  CHECK(v == 0);  // silently not armed — caller must check
}

TEST_CASE("hardware error status surfaces in transactions", "[emu]") {
  emu::MotorBus bus = craneBus();
  bus.find(2)->setHardwareError(0x04);  // overheating
  emu::FakePacketIO io(bus);
  std::uint8_t v = 0;
  CHECK(io.read8(2, reg::kPresentTemperature.addr, &v).error ==
        dxl::kErrResultFail);
}

TEST_CASE("SyncGroup reads all signals in one transaction and writes goals",
          "[dxl][emu]") {
  emu::MotorBus bus = craneBus();
  emu::FakePacketIO io(bus);
  dxl::SyncGroup group(io, {2, 3, 4, 5, 6, 7, 8, 9});

  REQUIRE(group.setupIndirect().ok());
  for (const auto id : group.ids()) {
    REQUIRE(io.write8(id, reg::kTorqueEnable.addr, 1).ok());
  }

  // Command distinct positions; run the motion sim to convergence.
  std::vector<double> currents(8, 0.0), positions(8);
  for (int i = 0; i < 8; ++i) positions[i] = 0.1 * (i + 1);
  REQUIRE(group.writeGoals(currents, positions).ok());
  for (int i = 0; i < 200; ++i) bus.tick(0.05);

  std::vector<dxl::Feedback> fb;
  REQUIRE(group.readAll(fb).ok());
  REQUIRE(fb.size() == 8);
  for (int i = 0; i < 8; ++i) {
    CHECK(fb[i].position == Approx(positions[i]).margin(2e-3));
    CHECK(fb[i].voltage == Approx(12.0));
    CHECK(fb[i].temperature == Approx(35.0));
  }
}

TEST_CASE("SyncGroup indirect setup is rejected while torque is on",
          "[dxl][emu]") {
  emu::MotorBus bus = craneBus();
  emu::FakePacketIO io(bus);
  dxl::SyncGroup group(io, {2, 3});
  REQUIRE(io.write8(2, reg::kTorqueEnable.addr, 1).ok());
  CHECK(group.setupIndirect().error == dxl::kErrAccess);
}

TEST_CASE("comm failures surface through the seam", "[dxl][emu]") {
  emu::MotorBus bus = craneBus();
  emu::FakePacketIO io(bus);
  io.setCommFailure(-3001);
  std::uint8_t v = 0;
  CHECK(io.read8(2, reg::kPresentTemperature.addr, &v).comm == -3001);
  CHECK_FALSE(io.ping(2).ok());
}
