#pragma once

#include <cstdint>

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

}  // namespace x7
