#pragma once

#include <string>

#include "rtctrl/model/ptp_planner.hpp"

namespace rtctrl::model {

// Writes the complete sampled PTP state as a stable, wide CSV table. Joint
// rates and accelerations are finite-difference estimates; Cartesian target
// derivatives are analytic, and achieved derivatives come from RoKi forward
// kinematics evaluated with those sampled joint derivatives.
void writePtpCsv(const std::string& path, const PtpPlan& plan);

}  // namespace rtctrl::model
