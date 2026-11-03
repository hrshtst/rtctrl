#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/error.hpp"

namespace rtctrl::emu {

// Control-table state machine of one XM-series servo. Shared by the two
// emulator transports: FakePacketIO (in-process unit tests) and the pty
// bus emulator (integration tests / offline app demos).
//
// Behavior modeled (per the XM430 e-manual):
// - EEPROM area and the indirect-address slots reject writes while
//   torque is enabled (access error 0x07).
// - Writing OperatingMode resets the goal registers; enabling torque
//   snaps GoalPosition to PresentPosition.
// - Reads/writes touching IndirectData redirect through the mapped
//   registers.
// - Goal registers clamp against Min/Max position, velocity, and
//   current limits.
// - Drive Mode bit 3, Torque On by Goal Update (firmware >= 45): a
//   write touching a goal register (Goal PWM, Current, Velocity or
//   Position; directly or through IndirectData) while torque is off
//   switches Torque Enable to 1 and keeps the written goal.
// - Torque-off goal overwrite, as observed on hardware (2026-11-03): an
//   XM430-W350 on firmware v47 in operating mode 0 acknowledges a goal
//   write while torque is off but keeps present values, Goal Position
//   reading back the present position and Goal Current 0. Modeled for
//   that firmware, model and mode only; elsewhere it is unobserved, and
//   the written goal stays.
// - Bus Watchdog (98): armed value counts 20 ms; EVERY instruction
//   packet addressed to this motor feeds it (touch()). On timeout the
//   register reads 0xFF, motion stops, and goal writes are rejected
//   until 0 is written. Arming requires firmware >= 38.
// - tick(dt) advances the motion simulation: Position mode moves
//   first-order toward the goal bounded by ProfileVelocity; current-based
//   position does the same while capping PresentCurrent at GoalCurrent; Velocity
//   mode integrates GoalVelocity; Current mode mirrors GoalCurrent into
//   PresentCurrent.
class MotorEmulator {
 public:
  struct Config {
    std::uint8_t id = 1;
    std::uint16_t model_number = dxl::kModelXm430W350;
    std::uint8_t firmware_version = 44;
  };

  explicit MotorEmulator(const Config& config);

  std::uint8_t id() const { return config_.id; }
  std::uint16_t modelNumber() const { return config_.model_number; }

  // Bus-facing register access (after indirect redirection and access
  // rules). Multi-byte quantities are little-endian, as on the wire.
  dxl::IoResult read(std::uint16_t addr, std::uint8_t* out,
                     std::uint16_t len);
  dxl::IoResult write(std::uint16_t addr, const std::uint8_t* data,
                      std::uint16_t len);

  // An instruction packet addressed this motor (feeds the Bus Watchdog).
  void touch() { watchdog_elapsed_ = 0.0; }
  // Advance simulated time: motion + watchdog timeout.
  void tick(double dt);

  // Test hooks.
  void setHardwareError(std::uint8_t status);
  bool watchdogTriggered() const { return watchdog_triggered_; }
  double presentPositionRad() const;
  bool moving() const;

  // Raw table access for assertions (no access rules, no redirection).
  std::uint32_t peek(dxl::Reg reg) const;
  void poke(dxl::Reg reg, std::uint32_t value);

 private:
  std::uint32_t rawAt(std::uint16_t addr, std::uint16_t len) const;
  void storeAt(std::uint16_t addr, std::uint16_t len, std::uint32_t value);
  bool writable(std::uint16_t addr, std::uint8_t* error) const;
  std::uint16_t redirect(std::uint16_t addr) const;
  void onWrite(std::uint16_t addr);
  void resetGoals();
  bool overwritesTorqueOffGoals() const;

  Config config_;
  std::array<std::uint8_t, 256> table_{};
  double watchdog_elapsed_ = 0.0;
  bool watchdog_triggered_ = false;
};

// A set of motors on one bus, sharing simulated time.
class MotorBus {
 public:
  MotorEmulator* add(const MotorEmulator::Config& config);
  MotorEmulator* find(std::uint8_t id);
  void tick(double dt);
  std::vector<MotorEmulator>& motors() { return motors_; }

 private:
  std::vector<MotorEmulator> motors_;
};

}  // namespace rtctrl::emu
