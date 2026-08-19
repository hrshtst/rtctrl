// x7_gravity_demo's torque-constant customization (pure logic): the
// kt <-> command_torque_scale mapping onto the one calibrated
// boundary, the vendor-default identity, and the kt-terms image of
// the reviewed [0.5, 1.0] scale bound.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <stdexcept>

#include "gravity/gravity_demo_common.hpp"
#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/hw/config.hpp"

using Catch::Approx;
namespace dxl = rtctrl::dxl;
namespace hw = rtctrl::hw;

TEST_CASE("default kt reproduces the approved vendor scale vector",
          "[gravity_demo][calibration]") {
  // A flagless x7_gravity_demo run must be EXACTLY the approved M-GC3
  // vendor calibration, whatever config it loads (the demo overwrites
  // every scale) — compared against the tracked vendor config within
  // x7_float's own gate tolerance.
  auto config = hw::Config::load("config/crane_x7.toml");
  gravity_demo::applyTorqueConstants(config, {});
  const auto vendor =
      hw::Config::load("config/crane_x7_vendor_scale.toml");
  REQUIRE(config.joints.size() == vendor.joints.size());
  for (std::size_t i = 0; i < config.joints.size(); ++i) {
    REQUIRE(config.joints[i].command_torque_scale ==
            Approx(vendor.joints[i].command_torque_scale).margin(1e-6));
  }
  REQUIRE_NOTHROW(config.validate());
}

TEST_CASE("scale = kt_nominal / kt_effective per servo model",
          "[gravity_demo][calibration]") {
  auto config = hw::Config::load("config/crane_x7.toml");
  gravity_demo::applyTorqueConstants(config, {2.0, 3.0});
  for (const auto& joint : config.joints) {
    const double expected =
        dxl::torqueConstant(joint.model_number) /
        (joint.model_number == dxl::kModelXm540W270 ? 3.0 : 2.0);
    REQUIRE(joint.command_torque_scale == expected);
  }
  REQUIRE_NOTHROW(config.validate());
}

TEST_CASE("the kt bounds are the config bound's exact image",
          "[gravity_demo][calibration]") {
  // scale = kt_nom/kt in [0.5, 1.0]  <=>  kt in [kt_nom, 2 * kt_nom]
  REQUIRE(gravity_demo::ktMin(dxl::kModelXm430W350) == 1.783);
  REQUIRE(gravity_demo::ktMax(dxl::kModelXm430W350) == Approx(3.566));
  REQUIRE(gravity_demo::ktMin(dxl::kModelXm540W270) == 2.409);
  REQUIRE(gravity_demo::ktMax(dxl::kModelXm540W270) == Approx(4.818));

  // both endpoints validate (scale exactly 1.0 / exactly 0.5)
  auto config = hw::Config::load("config/crane_x7.toml");
  gravity_demo::applyTorqueConstants(
      config, {gravity_demo::ktMin(dxl::kModelXm430W350),
               gravity_demo::ktMin(dxl::kModelXm540W270)});
  REQUIRE_NOTHROW(config.validate());
  gravity_demo::applyTorqueConstants(
      config, {gravity_demo::ktMax(dxl::kModelXm430W350),
               gravity_demo::ktMax(dxl::kModelXm540W270)});
  REQUIRE_NOTHROW(config.validate());
}

TEST_CASE("out-of-bound kt is refused by config validation before bus "
          "contact",
          "[gravity_demo][calibration]") {
  const auto config = hw::Config::load("config/crane_x7.toml");
  auto expect_reject = [&](const gravity_demo::TorqueConstants& kt) {
    auto bad = config;
    gravity_demo::applyTorqueConstants(bad, kt);
    REQUIRE_THROWS_AS(bad.validate(), std::runtime_error);
  };
  // below nominal: scale > 1 — the over-compensation incident class
  expect_reject({1.70, gravity_demo::kVendorKtXm540});
  expect_reject({gravity_demo::kVendorKtXm430, 2.40});
  // above 2 x nominal: scale < 0.5 — past the under-support floor
  expect_reject({3.60, gravity_demo::kVendorKtXm540});
  expect_reject({gravity_demo::kVendorKtXm430, 4.90});
  // degenerate values map to nonfinite/out-of-range scales
  expect_reject({0.0, gravity_demo::kVendorKtXm540});
  expect_reject({-2.20, gravity_demo::kVendorKtXm540});
  expect_reject({std::numeric_limits<double>::quiet_NaN(),
                 gravity_demo::kVendorKtXm540});
}
