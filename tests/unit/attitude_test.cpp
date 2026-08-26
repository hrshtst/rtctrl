#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include "rtctrl/model/attitude.hpp"

TEST_CASE("world RPY radians use Rz Ry Rx order", "[attitude]") {
  constexpr double roll = 0.31;
  constexpr double pitch = -0.27;
  constexpr double yaw = 0.42;
  const double sr = std::sin(roll);
  const double cr = std::cos(roll);
  const double sp = std::sin(pitch);
  const double cp = std::cos(pitch);
  const double sy = std::sin(yaw);
  const double cy = std::cos(yaw);

  zMat3D expected;
  zMat3DCreate(&expected, cy * cp, cy * sp * sr - sy * cr,
               cy * sp * cr + sy * sr, sy * cp,
               sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp,
               cp * sr, cp * cr);
  const auto actual =
      rtctrl::model::worldAttitudeFromRpyRad(roll, pitch, yaw);
  CHECK(zMat3DEqual(&actual, &expected));
}

TEST_CASE("world RPY radians reject non-finite angles", "[attitude]") {
  CHECK_THROWS(rtctrl::model::worldAttitudeFromRpyRad(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  CHECK_THROWS(rtctrl::model::worldAttitudeFromRpyRad(
      0.0, std::numeric_limits<double>::infinity(), 0.0));
}
