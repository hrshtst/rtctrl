#pragma once

#include <zeo/zeo_mat3d.h>

namespace rtctrl::model {

// Constructs a world-frame attitude from roll, pitch, and yaw in radians:
// R = Rz(yaw) Ry(pitch) Rx(roll). Non-finite inputs throw.
zMat3D worldAttitudeFromRpyRad(double roll_rad, double pitch_rad,
                               double yaw_rad);

}  // namespace rtctrl::model
