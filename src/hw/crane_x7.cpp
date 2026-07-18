#include "rtctrl/hw/crane_x7.hpp"

#include <algorithm>
#include <cmath>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"

namespace rtctrl::hw {

namespace reg = rtctrl::dxl::reg;

CraneX7::CraneX7(dxl::PacketIO& io, Config config)
    : CraneX7(io, std::move(config), Options()) {}

CraneX7::CraneX7(dxl::PacketIO& io, Config config, Options options)
    : io_(io),
      config_(std::move(config)),
      options_(options),
      group_(io, [this] {
        std::vector<std::uint8_t> ids;
        for (const auto& joint : config_.joints) ids.push_back(joint.id);
        return ids;
      }()),
      now_([] {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
      }),
      last_command_(now_()) {}

std::vector<std::uint8_t> CraneX7::ids() const {
  std::vector<std::uint8_t> out;
  for (const auto& joint : config_.joints) out.push_back(joint.id);
  return out;
}

bool CraneX7::verifyServos() {
  for (const auto& joint : config_.joints) {
    std::uint16_t model = 0;
    const auto ping = io_.ping(joint.id, &model);
    if (!ping.ok()) {
      last_error_ = "no response from id " + std::to_string(joint.id);
      return false;
    }
    if (model != joint.model_number) {
      last_error_ = "id " + std::to_string(joint.id) + " reports model " +
                    std::to_string(model) + ", config expects " +
                    std::to_string(joint.model_number);
      return false;
    }
    std::uint8_t firmware = 0;
    if (!io_.read8(joint.id, reg::kFirmwareVersion.addr, &firmware).ok()) {
      last_error_ = "cannot read firmware of id " + std::to_string(joint.id);
      return false;
    }
    if (firmware < dxl::kMinFirmwareBusWatchdog) {
      last_error_ = "id " + std::to_string(joint.id) + " firmware v" +
                    std::to_string(firmware) +
                    " lacks Bus Watchdog (needs >= v38)";
      return false;
    }
  }
  return true;
}

bool CraneX7::activate() {
  if (activated_) return true;
  escalated_ = false;

  if (!verifyServos()) return false;

  // EEPROM-side setup requires torque off.
  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kTorqueEnable.addr, 0).ok()) {
      last_error_ = "torque-off failed on id " + std::to_string(joint.id);
      return false;
    }
  }
  if (!group_.setupIndirect().ok()) {
    last_error_ = "indirect map setup failed";
    return false;
  }
  limit_lo_.clear();
  limit_hi_.clear();
  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kOperatingMode.addr,
                    joint.operating_mode).ok()) {
      last_error_ = "operating-mode write failed on id " +
                    std::to_string(joint.id);
      return false;
    }
    std::uint32_t lo = 0, hi = 0;
    if (!io_.read32(joint.id, reg::kMinPositionLimit.addr, &lo).ok() ||
        !io_.read32(joint.id, reg::kMaxPositionLimit.addr, &hi).ok()) {
      last_error_ = "limit read failed on id " + std::to_string(joint.id);
      return false;
    }
    limit_lo_.push_back(dxl::pulseToRad(static_cast<std::int32_t>(lo)) +
                        joint.pos_limit_margin);
    limit_hi_.push_back(dxl::pulseToRad(static_cast<std::int32_t>(hi)) -
                        joint.pos_limit_margin);
  }

  // Snap goals to the present posture so torque-on causes no motion.
  std::vector<dxl::Feedback> present;
  if (!readAll(present)) {
    last_error_ = "present-state read failed";
    return false;
  }
  std::vector<double> zeros(config_.joints.size(), 0.0);
  std::vector<double> positions(config_.joints.size());
  for (std::size_t i = 0; i < present.size(); ++i) {
    positions[i] = present[i].position;
  }
  if (!group_.writeGoals(zeros, positions).ok()) {
    last_error_ = "goal snap failed";
    return false;
  }

  if (!writePositionPGain(options_.active_p_gain)) return false;

  const auto watchdog_units = static_cast<std::uint8_t>(std::clamp(
      std::lround(options_.bus_watchdog_timeout_s /
                  dxl::kBusWatchdogUnitSeconds),
      1L, 127L));
  for (const auto& joint : config_.joints) {
    // clear a previously triggered watchdog, then arm
    if (!io_.write8(joint.id, reg::kBusWatchdog.addr, 0).ok() ||
        !io_.write8(joint.id, reg::kBusWatchdog.addr, watchdog_units).ok()) {
      last_error_ = "bus-watchdog arm failed on id " +
                    std::to_string(joint.id);
      return false;
    }
  }

  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kTorqueEnable.addr, 1).ok()) {
      last_error_ = "torque-on failed on id " + std::to_string(joint.id);
      return false;
    }
  }
  last_command_ = now_();
  activated_ = true;
  return true;
}

bool CraneX7::deactivate() {
  bool ok = true;
  ok &= writePositionPGain(options_.limp_p_gain);
  // zero goal currents (relevant in current mode; harmless otherwise)
  for (const auto& joint : config_.joints) {
    ok &= io_.write16(joint.id, reg::kGoalCurrent.addr, 0).ok();
  }
  for (const auto& joint : config_.joints) {
    ok &= io_.write8(joint.id, reg::kTorqueEnable.addr, 0).ok();
    ok &= io_.write8(joint.id, reg::kBusWatchdog.addr, 0).ok();
  }
  activated_ = false;
  return ok;
}

bool CraneX7::readAll(std::vector<dxl::Feedback>& out) {
  return group_.readAll(out).ok();
}

bool CraneX7::writePositions(const std::vector<double>& rad) {
  if (escalated_) return false;
  std::vector<double> clamped(rad.size());
  for (std::size_t i = 0; i < rad.size(); ++i) {
    clamped[i] = std::clamp(rad[i], limit_lo_[i], limit_hi_[i]);
  }
  std::vector<double> zeros(rad.size(), 0.0);
  if (!group_.writeGoals(zeros, clamped).ok()) return false;
  last_command_ = now_();
  return true;
}

bool CraneX7::checkDeadman() {
  if (escalated_) return false;
  if (!activated_) return true;
  const double stale = now_() - last_command_;
  if (stale > options_.host_command_timeout_s) {
    escalate();
    return false;
  }
  return true;
}

void CraneX7::escalate() {
  if (escalated_) return;
  escalated_ = true;
  // Best-effort stop — these writes may be the very thing that is
  // failing, which is why the bus goes silent afterwards regardless.
  deactivate();
  if (on_escalate_) on_escalate_();
}

bool CraneX7::writePositionPGain(std::uint16_t gain) {
  for (const auto& joint : config_.joints) {
    if (!io_.write16(joint.id, reg::kPositionPGain.addr, gain).ok()) {
      last_error_ = "P-gain write failed on id " + std::to_string(joint.id);
      return false;
    }
  }
  return true;
}

bool CraneX7::writeProfileVelocity(std::uint32_t raw) {
  for (const auto& joint : config_.joints) {
    if (!io_.write32(joint.id, reg::kProfileVelocity.addr, raw).ok()) {
      return false;
    }
  }
  return true;
}

bool CraneX7::writeProfileAcceleration(std::uint32_t raw) {
  for (const auto& joint : config_.joints) {
    if (!io_.write32(joint.id, reg::kProfileAcceleration.addr, raw).ok()) {
      return false;
    }
  }
  return true;
}

}  // namespace rtctrl::hw
