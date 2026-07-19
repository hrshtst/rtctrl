#pragma once

#include <cstdint>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/packet_io.hpp"

namespace rtctrl::dxl {

// Per-servo feedback in SI units.
struct Feedback {
  double position = 0.0;     // [rad]
  double velocity = 0.0;     // [rad/s]
  double current = 0.0;      // [A]
  double voltage = 0.0;      // [V]
  double temperature = 0.0;  // [deg C]
};

// One-transaction bus IO for a group of XM servos, via indirect
// addressing: the non-contiguous feedback registers (present current/
// velocity/position + voltage + temperature) are mapped into one
// contiguous IndirectData window read with a single syncRead, and the
// goal registers (current + position) into a second window written with
// a single syncWrite.
//
// setupIndirect() programs the indirect-address slots and must run
// while torque is OFF (they are EEPROM-gated).
class SyncGroup {
 public:
  SyncGroup(PacketIO& io, std::vector<std::uint8_t> ids);

  const std::vector<std::uint8_t>& ids() const { return ids_; }

  // Program the indirect maps on every servo. Torque must be off.
  IoResult setupIndirect();

  // One syncRead of all feedback signals; `out` is resized to ids().
  IoResult readAll(std::vector<Feedback>& out);

  // One syncWrite of all goal registers (the servo uses what its
  // operating mode consumes). Sizes must equal ids().
  IoResult writeGoals(const std::vector<double>& current_amps,
                      const std::vector<double>& velocity_rad_s,
                      const std::vector<double>& position_rad);
  // Partial writes of a single goal signal (one syncWrite each).
  IoResult writeGoalCurrents(const std::vector<double>& amps);
  IoResult writeGoalVelocities(const std::vector<double>& rad_s);
  IoResult writeGoalPositions(const std::vector<double>& rad);

 private:
  // Feedback window layout (13 bytes per servo, slots 0..12):
  //   [0..1]  PresentCurrent   [2..5] PresentVelocity
  //   [6..9]  PresentPosition  [10..11] PresentInputVoltage
  //   [12]    PresentTemperature
  // Goal window layout (10 bytes per servo, slots 13..22):
  //   [0..1]  GoalCurrent  [2..5] GoalVelocity  [6..9] GoalPosition
  static constexpr int kFeedbackSlots = 13;
  static constexpr int kGoalSlots = 10;
  static constexpr std::uint16_t kFeedbackAddr = reg::kIndirectDataBase;
  static constexpr std::uint16_t kGoalAddr =
      reg::kIndirectDataBase + kFeedbackSlots;
  static constexpr std::uint16_t kGoalVelocityAddr = kGoalAddr + 2;
  static constexpr std::uint16_t kGoalPositionAddr = kGoalAddr + 6;

  PacketIO& io_;
  std::vector<std::uint8_t> ids_;
};

}  // namespace rtctrl::dxl
