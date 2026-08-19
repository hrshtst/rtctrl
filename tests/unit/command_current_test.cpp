// The torque->command-current boundary and its calibration scale
// (docs/records/history.md (gravity calibration) M-GC1): the formula, the config
// validation interval (rejected before any bus contact), the
// vendor-equivalent config variant, and preload/write continuity at
// the amp level through the emulator.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/hw/command_current.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

using Catch::Approx;
namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;

TEST_CASE("commandCurrentFromTorque: i = s * tau / kt, exactly",
          "[hw][calibration]") {
  hw::JointConfig j;
  j.model_number = dxl::kModelXm430W350;
  // default scale 1.0 is BIT-IDENTICAL to the historical expression
  REQUIRE(hw::commandCurrentFromTorque(j, 2.022) ==
          2.022 / dxl::torqueConstant(dxl::kModelXm430W350));
  // the vendor-derived scale ATTENUATES (the formula multiplies the
  // resulting current, not the denominator)
  j.command_torque_scale = 0.810455;
  REQUIRE(hw::commandCurrentFromTorque(j, 2.022) ==
          Approx(0.810455 * 2.022 / 1.783));
  REQUIRE(hw::commandCurrentFromTorque(j, 2.022) <
          2.022 / dxl::torqueConstant(dxl::kModelXm430W350));
  // per-model constant: the XM540 divides by 2.409
  hw::JointConfig s;
  s.model_number = dxl::kModelXm540W270;
  s.command_torque_scale = 0.669167;
  REQUIRE(hw::commandCurrentFromTorque(s, 2.022) ==
          Approx(0.669167 * 2.022 / 2.409));
  // sign-preserving for negative torques
  REQUIRE(hw::commandCurrentFromTorque(s, -1.0) < 0.0);

  // M-GC2 vendor-current parity: with the vendor-equivalent scale the
  // boundary reproduces the vendor's own conversion tau / kt_vendor
  // within the preregistered 1e-4 relative tolerance (the incident
  // example: 2.022 Nm on the XM540 shoulder -> 0.562 A, not 0.839 A).
  REQUIRE(hw::commandCurrentFromTorque(j, 2.022) ==
          Approx(2.022 / 2.20).epsilon(1e-4));
  REQUIRE(hw::commandCurrentFromTorque(s, 2.022) ==
          Approx(2.022 / 3.60).epsilon(1e-4));
}

TEST_CASE("command_torque_scale validates to [0.5, 1.0] before bus "
          "contact",
          "[hw][calibration]") {
  auto config = hw::Config::load("config/crane_x7.toml");
  REQUIRE_NOTHROW(config.validate());
  auto expect_reject = [&](double s) {
    auto bad = config;
    bad.joints[3].command_torque_scale = s;
    REQUIRE_THROWS_AS(bad.validate(), std::runtime_error);
  };
  expect_reject(std::numeric_limits<double>::quiet_NaN());
  expect_reject(std::numeric_limits<double>::infinity());
  expect_reject(0.0);
  expect_reject(-0.8);
  expect_reject(0.499);
  expect_reject(1.001);
  auto ok = config;
  ok.joints[3].command_torque_scale = 0.5;
  REQUIRE_NOTHROW(ok.validate());
  ok.joints[3].command_torque_scale = 0.810455;
  REQUIRE_NOTHROW(ok.validate());
}

TEST_CASE("a malformed scale type fails loudly, never silently 1.0",
          "[hw][calibration]") {
  // toml++'s value_or() would substitute the default on a wrong-typed
  // node; a quoted number must throw (review finding).
  REQUIRE_THROWS_AS(
      hw::Config::load("tests/data/crane_x7_bad_scale.toml"),
      std::runtime_error);
}

TEST_CASE("the vendor-scale config matches deployment field-for-field "
          "except the scale",
          "[hw][calibration]") {
  // Drift guard (review finding): a later safety-limit, margin, bus,
  // or joint change to the deployment config must not leave the
  // mandatory M-GC3 configuration stale.
  const auto a = hw::Config::load("config/crane_x7.toml");
  const auto b = hw::Config::load("config/crane_x7_vendor_scale.toml");
  REQUIRE(a.port == b.port);
  REQUIRE(a.baudrate == b.baudrate);
  REQUIRE(a.joints.size() == b.joints.size());
  for (std::size_t i = 0; i < a.joints.size(); ++i) {
    REQUIRE(a.joints[i].name == b.joints[i].name);
    REQUIRE(a.joints[i].id == b.joints[i].id);
    REQUIRE(a.joints[i].model_number == b.joints[i].model_number);
    REQUIRE(a.joints[i].operating_mode == b.joints[i].operating_mode);
    REQUIRE(a.joints[i].velocity_limit == b.joints[i].velocity_limit);
    REQUIRE(a.joints[i].effort_limit == b.joints[i].effort_limit);
    REQUIRE(a.joints[i].pos_limit_margin == b.joints[i].pos_limit_margin);
    REQUIRE(a.joints[i].current_limit_margin ==
            b.joints[i].current_limit_margin);
  }
}

TEST_CASE("the vendor-scale config carries the full vendor ratios",
          "[hw][calibration]") {
  const auto config =
      hw::Config::load("config/crane_x7_vendor_scale.toml");
  for (std::size_t i = 0; i < config.joints.size(); ++i) {
    const bool xm540 =
        config.joints[i].model_number == dxl::kModelXm540W270;
    REQUIRE(config.joints[i].command_torque_scale ==
            Approx(xm540 ? 0.669167 : 0.810455));
  }
  // only the shoulder tilt (canonical 1) is the XM540
  REQUIRE(config.joints[1].model_number == dxl::kModelXm540W270);
}

TEST_CASE("preload and cyclic write are continuous at the amp level",
          "[hw][calibration]") {
  // Both producers convert through the ONE boundary, so the current
  // staged by the mode-switch preload equals the current a subsequent
  // write of the same torque produces — scaled exactly once.
  auto config = hw::Config::load("config/crane_x7_vendor_scale.toml");
  for (auto& joint : config.joints) joint.operating_mode = 3;
  emu::MotorBus bus;
  for (const auto& joint : config.joints) {
    emu::MotorEmulator::Config c;
    c.id = joint.id;
    c.model_number = joint.model_number;
    bus.add(c);
  }
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());  // position mode: held

  const double tau_demo = 1.5;  // [Nm] same torque through both paths
  std::vector<double> preload(config.joints.size());
  for (std::size_t i = 0; i < config.joints.size(); ++i) {
    preload[i] = hw::commandCurrentFromTorque(config.joints[i], tau_demo);
  }
  REQUIRE(arm.switchToCurrentModeWithPreload(preload));
  const auto staged =
      static_cast<std::int16_t>(bus.find(2)->peek(reg::kGoalCurrent));
  REQUIRE(staged != 0);
  // scaled once: the register holds scale * tau / kt, not scale^2 *...
  REQUIRE(dxl::currentToAmps(staged) ==
          Approx(0.810455 * tau_demo / 1.783).margin(2 * 0.00269));

  std::vector<dxl::Feedback> fb;
  REQUIRE(arm.readAll(fb));
  REQUIRE(arm.writeCurrents(preload));
  const auto cyclic =
      static_cast<std::int16_t>(bus.find(2)->peek(reg::kGoalCurrent));
  REQUIRE(cyclic == staged);  // continuity: identical register value
}
