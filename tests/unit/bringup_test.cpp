#include <catch2/catch_test_macros.hpp>

#include "bringup/set_param_common.hpp"
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
