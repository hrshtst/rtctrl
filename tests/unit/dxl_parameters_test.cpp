#include "bus/dxl_parameters.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <string>
#include <toml++/toml.hpp>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"

namespace params = rtctrl::apps::dxl_parameters;
namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace reg = rtctrl::dxl::reg;

namespace {

struct Fixture {
  Fixture() : io(bus) {
    bus.add({2, dxl::kModelXm430W350, 44});
    bus.add({3, dxl::kModelXm540W270, 45});
  }

  params::MotorRecord capture(std::uint8_t id) {
    params::MotorRecord motor;
    std::string error;
    REQUIRE(params::captureMotor(io, id, &motor, &error));
    return motor;
  }

  emu::MotorBus bus;
  emu::FakePacketIO io;
};

params::MotorRecord update(std::uint8_t id, std::uint16_t model,
                           const char* name, std::int64_t value) {
  params::MotorRecord motor;
  motor.id = id;
  motor.model_number = model;
  motor.firmware_version = 44;
  motor.parameters.push_back({params::findParameter(name), value});
  return motor;
}

}  // namespace

TEST_CASE("Dynamixel parameter dumps round-trip as versioned TOML",
          "[dxl][parameters]") {
  Fixture fixture;
  params::ParameterDump original{{fixture.capture(2), fixture.capture(3)}};

  const auto document = params::serialize(original);
  const auto parsed = params::parse(toml::parse(document));

  REQUIRE(parsed.motors.size() == 2);
  CHECK(parsed.motors[0].id == 2);
  CHECK(parsed.motors[0].model_number == dxl::kModelXm430W350);
  CHECK(parsed.motors[0].parameters.size() == params::kParameters.size() - 4);
  const auto p_gain = std::find_if(
      parsed.motors[0].parameters.begin(), parsed.motors[0].parameters.end(),
      [](const params::ParameterValue& value) {
        return std::string(value.def->name) == "position_p_gain";
      });
  REQUIRE(p_gain != parsed.motors[0].parameters.end());
  CHECK(p_gain->value == 800);
}

TEST_CASE("parameter dump parser rejects unknown and out-of-range values",
          "[dxl][parameters]") {
  const std::string prefix = R"(
format = "rtctrl-dynamixel-parameters"
version = 1
[[motor]]
id = 2
model_number = 1020
firmware_version = 44
[motor.parameters]
)";
  CHECK_THROWS(params::parse(toml::parse(prefix + "mystery = 1\n")));
  CHECK_THROWS(
      params::parse(toml::parse(prefix + "position_p_gain = 70000\n")));
  CHECK_THROWS(params::parse(toml::parse(prefix + "operating_mode = 2\n")));
}

TEST_CASE("parameter loader changes only keys present in the dump",
          "[dxl][parameters]") {
  Fixture fixture;
  params::ParameterDump changes{{
      update(2, dxl::kModelXm430W350, "position_p_gain", 640),
  }};

  const auto result = params::apply(fixture.io, changes);
  REQUIRE(result.ok);
  CHECK(result.changed == 1);
  CHECK(fixture.bus.find(2)->peek(reg::kPositionPGain) == 640);
  CHECK(fixture.bus.find(2)->peek(reg::kProfileVelocity) == 0);
  CHECK(fixture.bus.find(3)->peek(reg::kPositionPGain) == 800);
}

TEST_CASE("parameter loader completes all safety preflight before writing",
          "[dxl][parameters][safety]") {
  Fixture fixture;
  REQUIRE(fixture.io.write8(3, reg::kTorqueEnable.addr, 1).ok());
  params::ParameterDump changes{{
      update(2, dxl::kModelXm430W350, "position_p_gain", 640),
      update(3, dxl::kModelXm540W270, "position_p_gain", 640),
  }};

  const auto result = params::apply(fixture.io, changes);
  CHECK_FALSE(result.ok);
  CHECK(result.error.find("torque is enabled") != std::string::npos);
  CHECK(fixture.bus.find(2)->peek(reg::kPositionPGain) == 800);
}

TEST_CASE("parameter loader rejects inverted limits before writing",
          "[dxl][parameters][safety]") {
  Fixture fixture;
  auto motor = update(2, dxl::kModelXm430W350, "position_p_gain", 640);
  motor.parameters.push_back(
      {params::findParameter("min_position_limit"), 4096});

  const auto result = params::apply(fixture.io, {{motor}});
  CHECK_FALSE(result.ok);
  CHECK(result.error.find("position limit minimum") != std::string::npos);
  CHECK(fixture.bus.find(2)->peek(reg::kPositionPGain) == 800);
}

TEST_CASE("parameter loader refuses model and communication-setting changes",
          "[dxl][parameters][safety]") {
  Fixture fixture;
  auto wrong_model = update(2, dxl::kModelXm540W270, "position_p_gain", 640);
  auto result = params::apply(fixture.io, {{wrong_model}});
  CHECK_FALSE(result.ok);
  CHECK(result.error.find("model is") != std::string::npos);

  auto baud = update(2, dxl::kModelXm430W350, "baud_rate", 4);
  result = params::apply(fixture.io, {{baud}});
  CHECK_FALSE(result.ok);
  CHECK(result.error.find("dump-only") != std::string::npos);
}

TEST_CASE("operating-mode updates preserve omitted gains and profiles",
          "[dxl][parameters][safety]") {
  Fixture fixture;
  fixture.bus.find(2)->poke(reg::kPositionPGain, 640);
  fixture.bus.find(2)->poke(reg::kProfileVelocity, 75);
  params::ParameterDump changes{{
      update(2, dxl::kModelXm430W350, "operating_mode", 1),
  }};

  const auto result = params::apply(fixture.io, changes);
  REQUIRE(result.ok);
  CHECK(fixture.bus.find(2)->peek(reg::kOperatingMode) == 1);
  CHECK(fixture.bus.find(2)->peek(reg::kPositionPGain) == 640);
  CHECK(fixture.bus.find(2)->peek(reg::kProfileVelocity) == 75);
}

TEST_CASE("parameter loader rolls prior writes back after a later failure",
          "[dxl][parameters][safety]") {
  Fixture fixture;
  auto motor = update(2, dxl::kModelXm430W350, "position_p_gain", 640);
  motor.parameters.push_back({params::findParameter("profile_velocity"), 77});
  fixture.io.setWriteFailureOn(2, reg::kProfileVelocity.addr, 77, -3001);

  const auto result = params::apply(fixture.io, {{motor}});
  CHECK_FALSE(result.ok);
  CHECK(result.rollback_attempted);
  CHECK(result.rollback_ok);
  CHECK(fixture.bus.find(2)->peek(reg::kPositionPGain) == 800);
  CHECK(fixture.bus.find(2)->peek(reg::kProfileVelocity) == 0);
}
