#pragma once

#include <cstdint>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/packet_io.hpp"
#include "rtctrl/hw/config.hpp"

namespace x7 {

enum class TorqueCheckStatus {
  kAllOff,
  kEnabled,
  kReadFailed,
};

struct TorqueCheckResult {
  TorqueCheckStatus status = TorqueCheckStatus::kAllOff;
  std::uint8_t id = 0;
};

struct ServoParameters {
  std::uint8_t id = 0;
  std::uint16_t p_gain = 0;
  std::uint32_t profile_velocity = 0;
  std::uint32_t profile_acceleration = 0;
};

struct ParameterReadResult {
  bool ok = false;
  std::uint8_t failed_id = 0;
  const char* failed_register = nullptr;
  std::vector<ServoParameters> values;
};

struct ParameterRequest {
  bool have_p_gain = false;
  std::uint16_t p_gain = 0;
  bool have_profile_velocity = false;
  std::uint32_t profile_velocity = 0;
  bool have_profile_acceleration = false;
  std::uint32_t profile_acceleration = 0;
};

struct ParameterMismatch {
  std::uint8_t id = 0;
  const char* name = nullptr;
  std::uint32_t expected = 0;
  std::uint32_t actual = 0;
};

inline TorqueCheckResult checkAllTorqueOff(
    rtctrl::dxl::PacketIO& io, const rtctrl::hw::Config& config) {
  for (const auto& joint : config.joints) {
    std::uint8_t enabled = 0;
    if (!io.read8(joint.id, rtctrl::dxl::reg::kTorqueEnable.addr, &enabled)
             .ok()) {
      return {TorqueCheckStatus::kReadFailed, joint.id};
    }
    if (enabled != 0) {
      return {TorqueCheckStatus::kEnabled, joint.id};
    }
  }
  return {};
}

inline ParameterReadResult readAllParameters(
    rtctrl::dxl::PacketIO& io, const rtctrl::hw::Config& config) {
  ParameterReadResult result;
  result.values.reserve(config.joints.size());
  for (const auto& joint : config.joints) {
    ServoParameters values;
    values.id = joint.id;
    if (!io.read16(joint.id, rtctrl::dxl::reg::kPositionPGain.addr,
                   &values.p_gain)
             .ok()) {
      result.failed_id = joint.id;
      result.failed_register = "position_p_gain";
      return result;
    }
    if (!io.read32(joint.id, rtctrl::dxl::reg::kProfileVelocity.addr,
                   &values.profile_velocity)
             .ok()) {
      result.failed_id = joint.id;
      result.failed_register = "profile_velocity";
      return result;
    }
    if (!io.read32(joint.id, rtctrl::dxl::reg::kProfileAcceleration.addr,
                   &values.profile_acceleration)
             .ok()) {
      result.failed_id = joint.id;
      result.failed_register = "profile_acceleration";
      return result;
    }
    result.values.push_back(values);
  }
  result.ok = true;
  return result;
}

inline bool verifyParameterReadback(
    const std::vector<ServoParameters>& values,
    const ParameterRequest& requested, ParameterMismatch* mismatch) {
  for (const auto& value : values) {
    auto check = [&](bool requested_value, const char* name,
                     std::uint32_t expected, std::uint32_t actual) {
      if (!requested_value || expected == actual) return true;
      if (mismatch != nullptr) {
        *mismatch = {value.id, name, expected, actual};
      }
      return false;
    };
    if (!check(requested.have_p_gain, "position_p_gain", requested.p_gain,
               value.p_gain) ||
        !check(requested.have_profile_velocity, "profile_velocity",
               requested.profile_velocity, value.profile_velocity) ||
        !check(requested.have_profile_acceleration, "profile_acceleration",
               requested.profile_acceleration, value.profile_acceleration)) {
      return false;
    }
  }
  return true;
}

}  // namespace x7
