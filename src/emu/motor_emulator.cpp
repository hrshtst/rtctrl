#include "rtctrl/emu/motor_emulator.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "rtctrl/dxl/conversions.hpp"

namespace rtctrl::emu {

namespace dxl = rtctrl::dxl;
namespace reg = rtctrl::dxl::reg;

namespace {

bool inGoalArea(std::uint16_t addr) {
  return (addr >= reg::kGoalCurrent.addr &&
          addr < reg::kGoalPosition.addr + reg::kGoalPosition.len);
}

}  // namespace

MotorEmulator::MotorEmulator(const Config& config) : config_(config) {
  poke(reg::kModelNumber, config_.model_number);
  poke(reg::kFirmwareVersion, config_.firmware_version);
  poke(reg::kId, config_.id);
  poke(reg::kOperatingMode,
       static_cast<std::uint8_t>(dxl::OperatingMode::kPosition));
  poke(reg::kCurrentLimit, 1193);          // XM430-W350 default
  poke(reg::kVelocityLimit, 210);          // ~5.03 rad/s
  poke(reg::kMaxPositionLimit, 4095);
  poke(reg::kMinPositionLimit, 0);
  poke(reg::kPositionPGain, 800);
  poke(reg::kPositionIGain, 0);
  poke(reg::kPositionDGain, 0);
  poke(reg::kGoalPosition, dxl::kHomePulse);
  poke(reg::kPresentPosition, dxl::kHomePulse);
  poke(reg::kPresentInputVoltage, 120);    // 12.0 V
  poke(reg::kPresentTemperature, 35);
  // Identity indirect map so unprogrammed slots behave harmlessly.
  for (int i = 0; i < reg::kIndirectSlots; ++i) {
    storeAt(reg::kIndirectAddressBase + 2 * i, 2,
            reg::kIndirectDataBase + i);
  }
}

std::uint32_t MotorEmulator::rawAt(std::uint16_t addr,
                                   std::uint16_t len) const {
  std::uint32_t value = 0;
  for (int i = len - 1; i >= 0; --i) {
    value = (value << 8) | table_[addr + i];
  }
  return value;
}

void MotorEmulator::storeAt(std::uint16_t addr, std::uint16_t len,
                            std::uint32_t value) {
  for (int i = 0; i < len; ++i) {
    table_[addr + i] = static_cast<std::uint8_t>(value >> (8 * i));
  }
}

std::uint32_t MotorEmulator::peek(dxl::Reg r) const {
  return rawAt(r.addr, r.len);
}

void MotorEmulator::poke(dxl::Reg r, std::uint32_t value) {
  storeAt(r.addr, r.len, value);
}

std::uint16_t MotorEmulator::redirect(std::uint16_t addr) const {
  if (addr >= reg::kIndirectDataBase &&
      addr < reg::kIndirectDataBase + reg::kIndirectSlots) {
    const int slot = addr - reg::kIndirectDataBase;
    return static_cast<std::uint16_t>(
        rawAt(reg::kIndirectAddressBase + 2 * slot, 2));
  }
  return addr;
}

bool MotorEmulator::writable(std::uint16_t addr, std::uint8_t* error) const {
  const bool torque_on = table_[reg::kTorqueEnable.addr] != 0;
  if (torque_on &&
      (dxl::isEepromAddr(addr) || dxl::isIndirectAddressAddr(addr))) {
    *error = dxl::kErrAccess;
    return false;
  }
  if (watchdog_triggered_ && inGoalArea(addr)) {
    *error = dxl::kErrAccess;
    return false;
  }
  return true;
}

dxl::IoResult MotorEmulator::read(std::uint16_t addr, std::uint8_t* out,
                                  std::uint16_t len) {
  dxl::IoResult result;
  if (addr + len > table_.size()) {
    result.error = dxl::kErrDataLength;
    return result;
  }
  for (std::uint16_t i = 0; i < len; ++i) {
    out[i] = table_[redirect(addr + i)];
  }
  result.error = table_[reg::kHardwareErrorStatus.addr] != 0
                     ? dxl::kErrResultFail
                     : dxl::kErrNone;
  return result;
}

dxl::IoResult MotorEmulator::write(std::uint16_t addr,
                                   const std::uint8_t* data,
                                   std::uint16_t len) {
  dxl::IoResult result;
  if (addr + len > table_.size()) {
    result.error = dxl::kErrDataLength;
    return result;
  }
  // Validate the whole window first: real servos reject atomically.
  for (std::uint16_t i = 0; i < len; ++i) {
    if (!writable(redirect(addr + i), &result.error)) return result;
  }
  for (std::uint16_t i = 0; i < len; ++i) {
    table_[redirect(addr + i)] = data[i];
  }
  for (std::uint16_t i = 0; i < len; ++i) {
    onWrite(redirect(addr + i));
  }
  return result;
}

void MotorEmulator::resetGoals() {
  poke(reg::kGoalCurrent, 0);
  poke(reg::kGoalVelocity, 0);
  poke(reg::kGoalPosition, peek(reg::kPresentPosition));
}

void MotorEmulator::onWrite(std::uint16_t addr) {
  if (addr == reg::kOperatingMode.addr) {
    resetGoals();
  } else if (addr == reg::kTorqueEnable.addr) {
    if (table_[reg::kTorqueEnable.addr] != 0) {
      poke(reg::kGoalPosition, peek(reg::kPresentPosition));
    }
  } else if (addr == reg::kBusWatchdog.addr) {
    if (table_[reg::kBusWatchdog.addr] == 0) {
      watchdog_triggered_ = false;  // cleared
      watchdog_elapsed_ = 0.0;
    } else if (config_.firmware_version < dxl::kMinFirmwareBusWatchdog) {
      table_[reg::kBusWatchdog.addr] = 0;  // unsupported: refuse to arm
    }
  } else if (addr == reg::kGoalPosition.addr + reg::kGoalPosition.len - 1) {
    const auto lo = peek(reg::kMinPositionLimit);
    const auto hi = peek(reg::kMaxPositionLimit);
    const auto goal = peek(reg::kGoalPosition);
    poke(reg::kGoalPosition, std::clamp(goal, lo, hi));
  } else if (addr == reg::kGoalCurrent.addr + reg::kGoalCurrent.len - 1) {
    const auto limit = static_cast<std::int16_t>(peek(reg::kCurrentLimit));
    const auto goal =
        static_cast<std::int16_t>(peek(reg::kGoalCurrent) & 0xFFFF);
    poke(reg::kGoalCurrent, static_cast<std::uint16_t>(
                                std::clamp<std::int16_t>(goal, -limit, limit)));
  }
}

void MotorEmulator::tick(double dt) {
  // Bus Watchdog.
  const std::uint8_t wd = table_[reg::kBusWatchdog.addr];
  if (wd != 0 && wd != dxl::kBusWatchdogTriggered && !watchdog_triggered_) {
    watchdog_elapsed_ += dt;
    if (watchdog_elapsed_ >= wd * dxl::kBusWatchdogUnitSeconds) {
      watchdog_triggered_ = true;
      table_[reg::kBusWatchdog.addr] = dxl::kBusWatchdogTriggered;
    }
  }

  const bool torque_on = table_[reg::kTorqueEnable.addr] != 0;
  const bool halted = watchdog_triggered_ || !torque_on;

  const auto mode = static_cast<dxl::OperatingMode>(
      table_[reg::kOperatingMode.addr]);
  auto present =
      static_cast<std::int32_t>(peek(reg::kPresentPosition));

  double velocity_pulses = 0.0;  // signed pulses/s this tick
  if (!halted) {
    switch (mode) {
      case dxl::OperatingMode::kPosition: {
        const auto goal = static_cast<std::int32_t>(peek(reg::kGoalPosition));
        auto profile = static_cast<std::int32_t>(peek(reg::kProfileVelocity));
        if (profile == 0) profile = peek(reg::kVelocityLimit);
        const double max_step =
            dxl::velocityToRadPerSec(profile) * dxl::kPulsesPerRevolution /
            (2.0 * M_PI) * dt;
        const double delta = std::clamp<double>(goal - present, -max_step,
                                                max_step);
        present += static_cast<std::int32_t>(std::lround(delta));
        velocity_pulses = delta / dt;
        break;
      }
      case dxl::OperatingMode::kVelocity: {
        const auto goal_raw =
            static_cast<std::int32_t>(peek(reg::kGoalVelocity));
        const double pulses_per_sec = dxl::velocityToRadPerSec(goal_raw) *
                                      dxl::kPulsesPerRevolution /
                                      (2.0 * M_PI);
        present += static_cast<std::int32_t>(std::lround(pulses_per_sec * dt));
        velocity_pulses = pulses_per_sec;
        break;
      }
      case dxl::OperatingMode::kCurrent:
        poke(reg::kPresentCurrent, peek(reg::kGoalCurrent));
        break;
      default:
        break;
    }
  } else {
    poke(reg::kPresentCurrent, 0);
  }

  poke(reg::kPresentPosition, static_cast<std::uint32_t>(present));
  const double rad_per_sec =
      velocity_pulses * (2.0 * M_PI) / dxl::kPulsesPerRevolution;
  poke(reg::kPresentVelocity,
       static_cast<std::uint32_t>(dxl::radPerSecToVelocity(rad_per_sec)));
  poke(reg::kMoving, std::fabs(velocity_pulses) > 1.0 ? 1 : 0);
}

void MotorEmulator::setHardwareError(std::uint8_t status) {
  table_[reg::kHardwareErrorStatus.addr] = status;
}

double MotorEmulator::presentPositionRad() const {
  return dxl::pulseToRad(static_cast<std::int32_t>(peek(reg::kPresentPosition)));
}

bool MotorEmulator::moving() const {
  return table_[reg::kMoving.addr] != 0;
}

MotorEmulator* MotorBus::add(const MotorEmulator::Config& config) {
  motors_.emplace_back(config);
  return &motors_.back();
}

MotorEmulator* MotorBus::find(std::uint8_t id) {
  for (auto& motor : motors_) {
    if (motor.id() == id) return &motor;
  }
  return nullptr;
}

void MotorBus::tick(double dt) {
  for (auto& motor : motors_) motor.tick(dt);
}

}  // namespace rtctrl::emu
