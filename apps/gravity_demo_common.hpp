// Pure logic for x7_gravity_demo: the demonstration's torque-constant
// customization. The operator thinks in effective torque constants
// [Nm/A]; the library thinks in command_torque_scale at THE one
// torque->current boundary (hw::commandCurrentFromTorque,
// docs/HISTORY.md (gravity calibration) M-GC1):
//
//   i_cmd = scale * tau / kt_nominal = tau / kt_effective,
//   so      scale = kt_nominal / kt_effective.
//
// dxl::torqueConstant stays untouched (a standing non-goal: the
// calibration is an explicit, configured, command-side quantity), and
// Config::validate()'s reviewed scale bound [0.5, 1.0] remains the
// enforcement — in kt terms every effective constant must lie in
// [kt_nominal, 2 * kt_nominal]: no over-compensation past the nominal
// constant (the 2026-07-29 incident class) and halving as the
// under-support floor.
#pragma once

#include <cstdint>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/hw/config.hpp"

namespace gravity_demo {

// The vendor-empirical effective constants (rt_manipulators' tuned
// values): the demo's DEFAULTS, chosen so a flagless run reproduces
// exactly the approved M-GC3 vendor calibration —
// 1.783 / 2.20 = 0.810455 and 2.409 / 3.60 = 0.669167
// (config/crane_x7_vendor_scale.toml).
inline constexpr double kVendorKtXm430 = 2.20;  // [Nm/A]
inline constexpr double kVendorKtXm540 = 3.60;  // [Nm/A]

struct TorqueConstants {
  double kt_xm430 = kVendorKtXm430;  // [Nm/A] every XM430-W350 joint
  double kt_xm540 = kVendorKtXm540;  // [Nm/A] the XM540-W270 shoulder
};

inline double ktFor(std::uint16_t model_number,
                    const TorqueConstants& kt) {
  return model_number == rtctrl::dxl::kModelXm540W270 ? kt.kt_xm540
                                                      : kt.kt_xm430;
}

// The config's reviewed scale interval [0.5, 1.0] expressed in kt
// terms — used for kt-denominated refusal messages BEFORE the config
// backstop (Config::validate()) even runs.
inline double ktMin(std::uint16_t model_number) {
  return rtctrl::dxl::torqueConstant(model_number);
}
inline double ktMax(std::uint16_t model_number) {
  return 2.0 * rtctrl::dxl::torqueConstant(model_number);
}

// Overwrites every joint's command_torque_scale with
// kt_nominal / kt_effective. The caller MUST re-validate the config
// (Config::validate() owns the [0.5, 1.0] bound) before any bus
// contact; a nonpositive or nonfinite kt maps to an out-of-range or
// nonfinite scale and is refused there.
inline void applyTorqueConstants(rtctrl::hw::Config& config,
                                 const TorqueConstants& kt) {
  for (auto& joint : config.joints) {
    joint.command_torque_scale =
        rtctrl::dxl::torqueConstant(joint.model_number) /
        ktFor(joint.model_number, kt);
  }
}

}  // namespace gravity_demo
