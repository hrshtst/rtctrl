#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <vector>

#include "gravity/gravity_support.hpp"

using Catch::Approx;
namespace gravity = x7::gravity;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;

TEST_CASE("gravity support accepts only the reviewed calibration",
          "[gravity][safety]") {
  auto config = hw::Config::load("config/crane_x7_vendor_scale.toml");
  CHECK_FALSE(gravity::calibrationMismatch(config));

  config.joints[1].command_torque_scale = 0.999;
  const auto mismatch = gravity::calibrationMismatch(config);
  REQUIRE(mismatch);
  CHECK(mismatch->joint == 1);
  CHECK(mismatch->expected == Approx(gravity::kVendorScaleXm540));
  CHECK(mismatch->actual == Approx(0.999));
}

TEST_CASE("gravity support identifies soft-limit start postures",
          "[gravity][safety]") {
  std::vector<rtctrl::dxl::Feedback> feedback(model::kCanonicalDof);
  std::vector<double> lower(model::kCanonicalDof, -1.0);
  std::vector<double> upper(model::kCanonicalDof, 1.0);
  CHECK_FALSE(gravity::startLimitViolation(feedback, lower, upper));

  feedback[3].position = 0.97;
  const auto violation =
      gravity::startLimitViolation(feedback, lower, upper);
  REQUIRE(violation);
  CHECK(violation->joint == 3);
  CHECK(violation->position == Approx(0.97));
}

TEST_CASE("gravity support computes a canonical preload",
          "[gravity][safety]") {
  const auto config =
      hw::Config::load("config/crane_x7_vendor_scale.toml");
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  std::vector<rtctrl::dxl::Feedback> feedback(model::kCanonicalDof);
  feedback[1].position = -0.4;
  feedback[3].position = 0.8;

  const auto preload = gravity::gravityPreload(config, chain, map, feedback);
  REQUIRE(preload.size() == model::kCanonicalDof);
  for (const double current : preload) CHECK(std::isfinite(current));
  CHECK(std::fabs(preload[3]) > 0.01);
}
