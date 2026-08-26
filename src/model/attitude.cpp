#include "rtctrl/model/attitude.hpp"

#include <cmath>
#include <stdexcept>

namespace rtctrl::model {

zMat3D worldAttitudeFromRpyRad(double roll_rad, double pitch_rad,
                               double yaw_rad) {
  if (!std::isfinite(roll_rad) || !std::isfinite(pitch_rad) ||
      !std::isfinite(yaw_rad)) {
    throw std::invalid_argument(
        "worldAttitudeFromRpyRad: angles must be finite radians");
  }
  zMat3D attitude;
  zMat3DFromZYX(&attitude, yaw_rad, pitch_rad, roll_rad);
  return attitude;
}

}  // namespace rtctrl::model
