#pragma once

#include <array>

#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::learn {

using JointArray = std::array<double, model::kCanonicalDof>;  // canonical 8-DOF order

}  // namespace rtctrl::learn
