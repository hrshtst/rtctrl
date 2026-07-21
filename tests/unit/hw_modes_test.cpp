#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

using Catch::Approx;
namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;

namespace {

hw::Config configWithMode(std::uint8_t mode) {
  auto config = hw::Config::load("config/crane_x7.toml");
  for (auto& joint : config.joints) joint.operating_mode = mode;
  return config;
}

emu::MotorBus busFor(const hw::Config& config) {
  emu::MotorBus bus;
  for (const auto& joint : config.joints) {
    emu::MotorEmulator::Config c;
    c.id = joint.id;
    c.model_number = joint.model_number;
    bus.add(c);
  }
  return bus;
}

}  // namespace

TEST_CASE("mode-mismatched commands are rejected", "[hw][modes]") {
  const auto config = configWithMode(3);  // position mode
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());

  const std::vector<double> v(8, 0.1);
  CHECK_FALSE(arm.writeVelocities(v));
  CHECK(arm.lastError().find("velocity command rejected") !=
        std::string::npos);
  CHECK_FALSE(arm.writeCurrents(v));
  CHECK(arm.writePositions(std::vector<double>(8, 0.0)));
}

TEST_CASE("velocity commands clamp and gate at position limits",
          "[hw][modes]") {
  const auto config = configWithMode(1);  // velocity mode
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());

  std::vector<dxl::Feedback> fb;
  REQUIRE(arm.readAll(fb));

  // magnitude clamp to the configured velocity limit
  std::vector<double> v(8, 100.0);
  REQUIRE(arm.writeVelocities(v));
  const auto raw = static_cast<std::int32_t>(
      bus.find(2)->peek(reg::kGoalVelocity));
  CHECK(dxl::velocityToRadPerSec(raw) ==
        Approx(config.joints[0].velocity_limit).epsilon(0.02));

  // joint parked at its upper limit: outward commands zero, inward pass
  bus.find(2)->poke(reg::kPresentPosition, 4095);
  REQUIRE(arm.readAll(fb));
  REQUIRE(arm.writeVelocities(std::vector<double>(8, 1.0)));
  CHECK(static_cast<std::int32_t>(bus.find(2)->peek(reg::kGoalVelocity)) ==
        0);
  REQUIRE(arm.writeVelocities(std::vector<double>(8, -1.0)));
  CHECK(static_cast<std::int32_t>(bus.find(2)->peek(reg::kGoalVelocity)) <
        0);
}

TEST_CASE("current commands clamp to the effort limit minus margin",
          "[hw][modes]") {
  const auto config = configWithMode(0);  // current mode
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());
  std::vector<dxl::Feedback> fb;
  REQUIRE(arm.readAll(fb));

  REQUIRE(arm.writeCurrents(std::vector<double>(8, 50.0)));
  // The bound is min(effort/torque_const, servo CurrentLimit) - margin.
  // The emulated servos carry the XM430 factory CurrentLimit (1193 raw
  // = 3.209 A), which is tighter than both effort-derived bounds here.
  const double servo_limit = dxl::currentToAmps(1193);
  const double expect0 =
      std::min(10.0 / 1.783, servo_limit) - 0.5;
  const auto raw0 =
      static_cast<std::int16_t>(bus.find(2)->peek(reg::kGoalCurrent));
  CHECK(dxl::currentToAmps(raw0) == Approx(expect0).epsilon(0.01));
  const double expect1 =
      std::min(10.0 / 2.409, servo_limit) - 0.5;
  const auto raw1 =
      static_cast<std::int16_t>(bus.find(3)->peek(reg::kGoalCurrent));
  CHECK(dxl::currentToAmps(raw1) == Approx(expect1).epsilon(0.01));
}

TEST_CASE("velocity commands without feedback are rejected", "[hw][modes]") {
  const auto config = configWithMode(1);
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());
  // activation itself reads feedback once — construct a fresh wrapper
  // to model a caller that never read. Since the pre-activation guard
  // was added it rejects even earlier: never activated, no limits.
  hw::CraneX7 fresh(io, config);
  CHECK_FALSE(fresh.writeVelocities(std::vector<double>(8, 0.1)));
  CHECK(fresh.lastError().find("not activated") != std::string::npos);
}
