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

}  // namespace

TEST_CASE("config TOML loads with bus settings and canonical joints",
          "[hw]") {
  const auto config = craneConfig();
  CHECK(config.port == "/dev/ttyUSB0");
  CHECK(config.baudrate == 3000000);
  REQUIRE(config.joints.size() == 8);
  CHECK(config.joints[0].id == 2);
  CHECK(config.joints[1].model_number == dxl::kModelXm540W270);
  CHECK(config.joints[7].id == 9);
}

TEST_CASE("activation sequence arms safety and causes no motion", "[hw]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);

  // servo sitting away from home: activation must not move it
  bus.find(4)->poke(reg::kPresentPosition, 1500);

  REQUIRE(arm.activate());
  for (const auto& joint : config.joints) {
    auto* motor = bus.find(joint.id);
    CHECK(motor->peek(reg::kTorqueEnable) == 1);
    CHECK(motor->peek(reg::kBusWatchdog) == 5);  // 100 ms / 20 ms units
    CHECK(motor->peek(reg::kPositionPGain) == 800);
    CHECK(motor->peek(reg::kOperatingMode) == joint.operating_mode);
    CHECK(motor->peek(reg::kGoalPosition) ==
          motor->peek(reg::kPresentPosition));
  }
  // keep the cycle alive (reads feed the armed Bus Watchdog) while
  // checking that nothing moves
  std::vector<dxl::Feedback> fb;
  for (int i = 0; i < 50; ++i) {
    REQUIRE(arm.readAll(fb));
    bus.tick(0.01);
  }
  CHECK(bus.find(4)->peek(reg::kPresentPosition) == 1500);  // did not move

  REQUIRE(arm.deactivate());
  for (const auto& joint : config.joints) {
    auto* motor = bus.find(joint.id);
    CHECK(motor->peek(reg::kTorqueEnable) == 0);
    CHECK(motor->peek(reg::kBusWatchdog) == 0);
    CHECK(motor->peek(reg::kPositionPGain) == 5);  // limp
  }
}

TEST_CASE("activation fails on firmware without Bus Watchdog", "[hw]") {
  const auto config = craneConfig();
  auto bus = busFor(config, /*firmware=*/37);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  CHECK_FALSE(arm.activate());
  CHECK(arm.lastError().find("firmware") != std::string::npos);
  // and nothing was torqued on
  CHECK(bus.find(2)->peek(reg::kTorqueEnable) == 0);
}

TEST_CASE("activation fails on model mismatch", "[hw]") {
  auto config = craneConfig();
  auto bus = busFor(config);
  config.joints[2].model_number = dxl::kModelXm540W270;  // wrong on purpose
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  CHECK_FALSE(arm.activate());
  CHECK(arm.lastError().find("model") != std::string::npos);
}

TEST_CASE("position commands clamp to servo limits minus margin", "[hw]") {
  auto config = craneConfig();
  for (auto& joint : config.joints) joint.pos_limit_margin = 0.1;
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());

  // command far beyond the +pi position limit
  std::vector<double> target(8, 10.0);
  REQUIRE(arm.writePositions(target));
  const auto goal = bus.find(2)->peek(reg::kGoalPosition);
  const double goal_rad = dxl::pulseToRad(static_cast<std::int32_t>(goal));
  const double servo_hi = dxl::pulseToRad(4095);
  CHECK(goal_rad == Approx(servo_hi - 0.1).margin(2e-3));
}

TEST_CASE(
    "deadman escalation: stalled writes with healthy reads end with the "
    "motors actually stopped",
    "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.host_command_timeout_s = 0.05;
  hw::CraneX7 arm(io, config, options);

  double sim_time = 0.0;
  arm.setTimeSource([&sim_time] { return sim_time; });
  auto tick = [&](double dt) {
    bus.tick(dt);
    sim_time += dt;
  };

  bool bus_silenced = false;
  arm.onEscalate([&bus_silenced] { bus_silenced = true; });

  REQUIRE(arm.activate());
  // start a long move so the motor is visibly in motion
  std::vector<double> target(8, 0.0);
  target[0] = 2.0;
  REQUIRE(arm.writePositions(target));
  tick(0.01);
  REQUIRE(bus.find(2)->moving());

  // Now the write path dies while reads stay healthy — the servo-side
  // watchdog alone would never fire.
  io.setWriteFailure(-3001);
  std::vector<dxl::Feedback> fb;
  bool escalated = false;
  for (int cycle = 0; cycle < 100; ++cycle) {
    REQUIRE(arm.readAll(fb));               // reads keep succeeding
    arm.writePositions(target);             // fails silently now
    tick(0.01);
    if (!arm.checkDeadman()) {
      escalated = true;
      break;
    }
  }
  REQUIRE(escalated);
  CHECK(bus_silenced);  // the escalation hook silenced the bus

  // ACCEPTANCE IS THE MOTOR'S END STATE: after escalation the host is
  // silent; the servo Bus Watchdog fires and motion stops even though
  // the best-effort torque-off writes were failing.
  for (int i = 0; i < 30; ++i) tick(0.01);
  auto* motor = bus.find(2);
  CHECK(motor->watchdogTriggered());
  const auto pos_after_trigger = motor->peek(reg::kPresentPosition);
  for (int i = 0; i < 30; ++i) tick(0.01);
  CHECK(motor->peek(reg::kPresentPosition) == pos_after_trigger);  // stopped
  // and the arm object refuses further commands
  CHECK_FALSE(arm.writePositions(target));
}

TEST_CASE("deadman does not fire while commands flow", "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.host_command_timeout_s = 0.05;
  hw::CraneX7 arm(io, config, options);
  double sim_time = 0.0;
  arm.setTimeSource([&sim_time] { return sim_time; });
  REQUIRE(arm.activate());

  std::vector<dxl::Feedback> fb;
  REQUIRE(arm.readAll(fb));
  std::vector<double> hold(8);
  for (int i = 0; i < 8; ++i) hold[i] = fb[i].position;
  for (int cycle = 0; cycle < 50; ++cycle) {
    REQUIRE(arm.writePositions(hold));
    bus.tick(0.001);
    sim_time += 0.001;
    REQUIRE(arm.checkDeadman());
  }
  CHECK_FALSE(arm.escalated());
}
