#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "rtctrl/dxl/packet_io.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/hw/config.hpp"

namespace rtctrl::hw {

// Minimal CRANE-X7 hardware wrapper for the M5 bring-up (grows to full
// rt_manipulators parity in M6). Owns the grouped IO and the safety
// sequences; the PacketIO is injected, so the same code runs against
// dxl::Port (hardware / pty emulator) and emu::FakePacketIO (tests).
//
// Loss-of-comms safety, two layers:
//  1. Servo-side Bus Watchdog (reg 98), armed by activate() — the servo
//     stops itself when NO instruction packet arrives in time. Arming
//     requires firmware >= 38 on every servo; otherwise activation
//     fails. The watchdog counts reads too, so it cannot catch a host
//     whose reads continue while command writes stall — that is layer 2:
//  2. The host-side deadman: feedCommand() marks each successful command
//     write; checkDeadman() escalates once the last one is older than
//     the timeout — best-effort zero + torque off, then STOP ALL BUS
//     TRAFFIC (onEscalate, e.g. dxl::Port::close), which guarantees
//     layer 1 fires on the servos.
//
// deactivate() is NOT an emergency stop: hardware sessions require an
// independent actuator-power cutoff within reach.
class CraneX7 {
 public:
  struct Options {
    double bus_watchdog_timeout_s = 0.1;    // reg 98, rounded to 20 ms units
    double host_command_timeout_s = 0.25;   // deadman bound on command writes
    std::uint16_t active_p_gain = 800;      // position P while active
    std::uint16_t limp_p_gain = 5;          // position P while going limp
  };

  CraneX7(dxl::PacketIO& io, Config config);
  CraneX7(dxl::PacketIO& io, Config config, Options options);

  // Time source for the deadman, in seconds. Defaults to the steady
  // clock; tests inject simulated time.
  void setTimeSource(std::function<double()> now) { now_ = std::move(now); }

  // Verifies ids/models/firmware, programs indirect maps and operating
  // modes (torque off), snaps goals to present, sets the active gains,
  // arms the servo Bus Watchdogs, then enables torque. No motion
  // results. Returns false (with lastError()) on any mismatch.
  bool activate();

  // Going limp gently: limp gains, zero goal currents, torque off,
  // watchdogs disarmed. Best-effort — continues through failures.
  bool deactivate();

  bool activated() const { return activated_; }
  const Config& config() const { return config_; }
  const std::string& lastError() const { return last_error_; }

  // Grouped IO in canonical joint order.
  bool readAll(std::vector<dxl::Feedback>& out);
  // Position command [rad], clamped to the servo position limits minus
  // pos_limit_margin. Feeds the deadman on success.
  bool writePositions(const std::vector<double>& rad);

  // Deadman: escalates if the last successful command write is older
  // than host_command_timeout_s. Call once per control cycle. Returns
  // false once escalated.
  bool checkDeadman();
  // The escalation path (also callable directly): best-effort zero +
  // torque off, then invoke onEscalate to silence the bus.
  void escalate();
  bool escalated() const { return escalated_; }
  void onEscalate(std::function<void()> hook) { on_escalate_ = std::move(hook); }

  // Per-servo parameter writes (all joints).
  bool writePositionPGain(std::uint16_t gain);
  bool writeProfileVelocity(std::uint32_t raw);
  bool writeProfileAcceleration(std::uint32_t raw);

 private:
  bool verifyServos();
  std::vector<std::uint8_t> ids() const;

  dxl::PacketIO& io_;
  Config config_;
  Options options_;
  dxl::SyncGroup group_;
  bool activated_ = false;
  bool escalated_ = false;
  std::function<double()> now_;
  double last_command_ = 0.0;
  std::string last_error_;
  std::function<void()> on_escalate_;
  // position limits read from the servos at activation [rad]
  std::vector<double> limit_lo_, limit_hi_;
};

}  // namespace rtctrl::hw
