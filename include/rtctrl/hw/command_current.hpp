#pragma once

#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/hw/config.hpp"

namespace rtctrl::hw {

// THE torque->command-current boundary
// (docs/HISTORY.md (gravity calibration) M-GC1):
//
//   i_cmd = command_torque_scale * tau / kt_nominal
//
// Every producer of a current that REPRESENTS a torque — the RealArm
// bridge's cyclic commands and every gravity preload — must convert
// through this one function, so the calibration scale applies exactly
// once and can never be double-applied or skipped. CraneX7's
// explicitly amp-valued APIs stay unscaled by design. The nominal
// electrical constant keeps its meaning for limits and telemetry
// (dxl::torqueConstant is deliberately untouched); the scale is the
// configured, provenanced calibration of effective output torque.
inline double commandCurrentFromTorque(const JointConfig& joint,
                                       double tau_nm) {
  return joint.command_torque_scale * tau_nm /
         dxl::torqueConstant(joint.model_number);
}

}  // namespace rtctrl::hw
