#pragma once

#include <cmath>
#include <cstdint>

#include "rtctrl/dxl/control_table.hpp"

namespace rtctrl::dxl {

// XM-series raw-unit conversions (values from the DynamixelSDK .model
// data and the ROBOTIS e-manual).
inline constexpr double kPulsesPerRevolution = 4096.0;
inline constexpr std::int32_t kHomePulse = 2048;  // raw at 0 rad
inline constexpr double kVelocityUnitRevPerMin = 0.229;
inline constexpr double kAccelerationUnitRevPerMinSq = 214.577;
inline constexpr double kCurrentUnitAmps = 0.00269;
inline constexpr double kVoltageUnitVolts = 0.1;

// SI -> profile-register conversions, vendor-exact (rt_manipulators'
// DynamixelX::to_profile_velocity/acceleration): TRUNCATING, clamped
// to [1, 32767]. Register 0 means MAXIMUM (no profile limit) on the X
// series, so a nonpositive SI request maps to the SLOWEST profile (1),
// never to the unlimited zero — reaching 0 takes the raw setters.
inline constexpr std::uint32_t kProfileRegisterMax = 32767;
inline std::uint32_t profileVelocityFromRadPerSec(double rad_per_sec) {
  const double raw =
      rad_per_sec * 60.0 / (kVelocityUnitRevPerMin * 2.0 * M_PI);
  if (raw > kProfileRegisterMax) return kProfileRegisterMax;
  const auto truncated = static_cast<std::int64_t>(raw);
  return truncated <= 0 ? 1u : static_cast<std::uint32_t>(truncated);
}
inline std::uint32_t profileAccelerationFromRadPerSec2(
    double rad_per_sec2) {
  const double raw = rad_per_sec2 * 3600.0 /
                     (kAccelerationUnitRevPerMinSq * 2.0 * M_PI);
  if (raw > kProfileRegisterMax) return kProfileRegisterMax;
  const auto truncated = static_cast<std::int64_t>(raw);
  return truncated <= 0 ? 1u : static_cast<std::uint32_t>(truncated);
}

inline double pulseToRad(std::int32_t pulse) {
  return (pulse - kHomePulse) * (2.0 * M_PI / kPulsesPerRevolution);
}
inline std::int32_t radToPulse(double rad) {
  return kHomePulse +
         static_cast<std::int32_t>(std::lround(rad * kPulsesPerRevolution /
                                               (2.0 * M_PI)));
}
inline double velocityToRadPerSec(std::int32_t raw) {
  return raw * kVelocityUnitRevPerMin * (2.0 * M_PI) / 60.0;
}
inline std::int32_t radPerSecToVelocity(double rad_per_sec) {
  return static_cast<std::int32_t>(
      std::lround(rad_per_sec * 60.0 / (kVelocityUnitRevPerMin * 2.0 * M_PI)));
}
inline double currentToAmps(std::int16_t raw) { return raw * kCurrentUnitAmps; }
inline std::int16_t ampsToCurrent(double amps) {
  return static_cast<std::int16_t>(std::lround(amps / kCurrentUnitAmps));
}
inline double voltageToVolts(std::uint16_t raw) {
  return raw * kVoltageUnitVolts;
}

// Output-shaft torque constant [Nm/A] per motor model (from the RT link
// spreadsheet; refined against measurements in M7).
inline double torqueConstant(std::uint16_t model_number) {
  switch (model_number) {
    case kModelXm540W270:
      return 2.409;
    case kModelXm430W350:
    default:
      return 1.783;
  }
}

inline double currentToTorque(std::int16_t raw, std::uint16_t model_number) {
  return currentToAmps(raw) * torqueConstant(model_number);
}
inline std::int16_t torqueToCurrentRaw(double torque_nm,
                                       std::uint16_t model_number) {
  return ampsToCurrent(torque_nm / torqueConstant(model_number));
}

}  // namespace rtctrl::dxl
