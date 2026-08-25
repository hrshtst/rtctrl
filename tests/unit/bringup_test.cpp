#include <catch2/catch_test_macros.hpp>

#include "bringup/set_param_common.hpp"
#include "bringup/write_monitor.hpp"
#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/hw/config.hpp"

namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;

namespace {

struct Fixture {
  Fixture() : config(hw::Config::load("config/crane_x7.toml")), io(bus) {
    for (const auto& joint : config.joints) {
      emu::MotorEmulator::Config motor;
      motor.id = joint.id;
      motor.model_number = joint.model_number;
      bus.add(motor);
    }
  }

  hw::Config config;
  emu::MotorBus bus;
  emu::FakePacketIO io;
};

}  // namespace

TEST_CASE("parameter changes require every servo torque to be off",
          "[bringup][safety]") {
  Fixture fixture;
  CHECK(x7::checkAllTorqueOff(fixture.io, fixture.config).status ==
        x7::TorqueCheckStatus::kAllOff);

  const auto enabled_id = fixture.config.joints[3].id;
  REQUIRE(fixture.io.write8(enabled_id, reg::kTorqueEnable.addr, 1).ok());
  const auto enabled = x7::checkAllTorqueOff(fixture.io, fixture.config);
  CHECK(enabled.status == x7::TorqueCheckStatus::kEnabled);
  CHECK(enabled.id == enabled_id);
}

TEST_CASE("parameter torque check fails closed on a read error",
          "[bringup][safety]") {
  Fixture fixture;
  fixture.io.setCommFailure(-3001);
  const auto failed = x7::checkAllTorqueOff(fixture.io, fixture.config);
  CHECK(failed.status == x7::TorqueCheckStatus::kReadFailed);
  CHECK(failed.id == fixture.config.joints.front().id);
}

TEST_CASE("parameter reads fail closed instead of printing zero values",
          "[bringup][safety]") {
  Fixture fixture;
  fixture.io.setReadFailure(-3001);
  const auto failed = x7::readAllParameters(fixture.io, fixture.config);
  CHECK_FALSE(failed.ok);
  CHECK(failed.failed_id == fixture.config.joints.front().id);
  CHECK(std::string(failed.failed_register) == "position_p_gain");
  CHECK(failed.values.empty());
}

TEST_CASE("parameter readback verifies every requested register and servo",
          "[bringup]") {
  Fixture fixture;
  constexpr std::uint16_t kPGain = 640;
  constexpr std::uint32_t kProfileVelocity = 120;
  for (const auto& joint : fixture.config.joints) {
    REQUIRE(fixture.io.write16(joint.id, reg::kPositionPGain.addr, kPGain)
                .ok());
    REQUIRE(fixture.io
                .write32(joint.id, reg::kProfileVelocity.addr,
                         kProfileVelocity)
                .ok());
  }
  const auto read = x7::readAllParameters(fixture.io, fixture.config);
  REQUIRE(read.ok);

  x7::ParameterRequest requested;
  requested.have_p_gain = true;
  requested.p_gain = kPGain;
  requested.have_profile_velocity = true;
  requested.profile_velocity = kProfileVelocity;
  CHECK(x7::verifyParameterReadback(read.values, requested, nullptr));

  requested.p_gain = kPGain + 1;
  x7::ParameterMismatch mismatch;
  CHECK_FALSE(
      x7::verifyParameterReadback(read.values, requested, &mismatch));
  CHECK(mismatch.id == fixture.config.joints.front().id);
  CHECK(std::string(mismatch.name) == "position_p_gain");
  CHECK(mismatch.expected == kPGain + 1);
  CHECK(mismatch.actual == kPGain);
}

TEST_CASE("position write monitor retains transient failures",
          "[bringup][safety]") {
  x7::PositionWriteMonitor writes(nullptr);
  CHECK(writes.record(true, "test"));
  CHECK(writes.ok());
  CHECK_FALSE(writes.record(false, "test"));
  CHECK_FALSE(writes.record(false, "test"));
  CHECK_FALSE(writes.ok());
  CHECK(writes.failures() == 2);
}
