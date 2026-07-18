#pragma once

#include <cstdint>

namespace rtctrl::dxl {

// One control-table register: address and byte length.
struct Reg {
  std::uint16_t addr;
  std::uint16_t len;
};

// XM-series control table (Protocol 2.0). XM430-W350 and XM540-W270
// share this layout; model-specific constants live in conversions.hpp.
// Cross-checked against DynamixelSDK's control_table/*.model data.
namespace reg {
inline constexpr Reg kModelNumber{0, 2};
inline constexpr Reg kFirmwareVersion{6, 1};
inline constexpr Reg kId{7, 1};
inline constexpr Reg kBaudRate{8, 1};
inline constexpr Reg kReturnDelayTime{9, 1};
inline constexpr Reg kOperatingMode{11, 1};
inline constexpr Reg kHomingOffset{20, 4};
inline constexpr Reg kCurrentLimit{38, 2};
inline constexpr Reg kVelocityLimit{44, 4};
inline constexpr Reg kMaxPositionLimit{48, 4};
inline constexpr Reg kMinPositionLimit{52, 4};
inline constexpr Reg kTorqueEnable{64, 1};
inline constexpr Reg kLed{65, 1};
inline constexpr Reg kStatusReturnLevel{68, 1};
inline constexpr Reg kHardwareErrorStatus{70, 1};
inline constexpr Reg kVelocityIGain{76, 2};
inline constexpr Reg kVelocityPGain{78, 2};
inline constexpr Reg kPositionDGain{80, 2};
inline constexpr Reg kPositionIGain{82, 2};
inline constexpr Reg kPositionPGain{84, 2};
inline constexpr Reg kBusWatchdog{98, 1};
inline constexpr Reg kGoalCurrent{102, 2};
inline constexpr Reg kGoalVelocity{104, 4};
inline constexpr Reg kProfileAcceleration{108, 4};
inline constexpr Reg kProfileVelocity{112, 4};
inline constexpr Reg kGoalPosition{116, 4};
inline constexpr Reg kMoving{122, 1};
inline constexpr Reg kPresentCurrent{126, 2};
inline constexpr Reg kPresentVelocity{128, 4};
inline constexpr Reg kPresentPosition{132, 4};
inline constexpr Reg kPresentInputVoltage{144, 2};
inline constexpr Reg kPresentTemperature{146, 1};
// Indirect bank 1: 28 slots. Address slot i (2 bytes each) maps data
// slot i (1 byte each).
inline constexpr std::uint16_t kIndirectAddressBase = 168;
inline constexpr std::uint16_t kIndirectDataBase = 224;
inline constexpr int kIndirectSlots = 28;
}  // namespace reg

// Operating Mode register values.
enum class OperatingMode : std::uint8_t {
  kCurrent = 0,
  kVelocity = 1,
  kPosition = 3,
  kExtendedPosition = 4,
  kCurrentBasedPosition = 5,
  kPwm = 16,
};

// EEPROM area (and the indirect-address slots) reject writes while
// torque is enabled.
inline bool isEepromAddr(std::uint16_t addr) { return addr < 64; }
inline bool isIndirectAddressAddr(std::uint16_t addr) {
  return addr >= reg::kIndirectAddressBase &&
         addr < reg::kIndirectAddressBase + 2 * reg::kIndirectSlots;
}

// Known model numbers.
inline constexpr std::uint16_t kModelXm430W350 = 1020;
inline constexpr std::uint16_t kModelXm540W270 = 1120;

// Bus Watchdog (register 98) semantics: unit 20 ms; 0 disables; on
// timeout the register reads back 0xFF (-1) and goal writes are
// rejected until 0 is written. Requires firmware >= 38.
inline constexpr double kBusWatchdogUnitSeconds = 0.020;
inline constexpr std::uint8_t kBusWatchdogTriggered = 0xFF;
inline constexpr std::uint8_t kMinFirmwareBusWatchdog = 38;

}  // namespace rtctrl::dxl
