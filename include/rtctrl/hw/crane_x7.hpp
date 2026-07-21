#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
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
    double control_cycle_s = 0.01;          // background thread period
    // Consecutive failed cycle reads before escalation. A controller
    // fed frozen feedback keeps commanding torques into a state it can
    // no longer see — as dangerous as a stalled command stream.
    int max_read_failures = 5;
  };

  struct CycleStats {
    std::uint64_t cycles = 0;
    std::uint64_t overruns = 0;  // cycles that finished past their deadline
    std::uint64_t read_failures = 0;  // cycles whose feedback read failed
  };

  CraneX7(dxl::PacketIO& io, Config config);
  CraneX7(dxl::PacketIO& io, Config config, Options options);

  // Time source for the deadman, in seconds. Defaults to the steady
  // clock; tests inject simulated time.
  void setTimeSource(std::function<double()> now) { now_ = std::move(now); }

  // Joins the background thread if it is still running. Deliberately
  // does NOT torque off: destruction without deactivate() usually means
  // an abnormal exit, and a silent bus lets the servo watchdogs freeze
  // the arm in place — safer than an uncommanded gravity drop.
  ~CraneX7();

  // Verifies ids/models/firmware, programs indirect maps and operating
  // modes (torque off), snaps goals to present, sets the active gains,
  // arms the servo Bus Watchdogs, then enables torque. No motion
  // results. Returns false (with lastError()) on any mismatch — and a
  // mid-sequence failure best-effort releases every servo the sequence
  // may already have touched (no partially torqued arm).
  bool activate();

  // Going limp gently: limp gains, zero goal currents, torque off,
  // watchdogs disarmed. Best-effort — continues through failures.
  bool deactivate();

  bool activated() const { return activated_; }
  const Config& config() const { return config_; }
  const Options& options() const { return options_; }
  const std::string& lastError() const { return last_error_; }

  // Grouped IO in canonical joint order. readAll also refreshes the
  // cached feedback used by the software limiters and lastFeedback().
  bool readAll(std::vector<dxl::Feedback>& out);
  std::vector<dxl::Feedback> lastFeedback() const;  // thread-safe copy

  // Mode-checked command writers (every joint's configured operating
  // mode must match, or the call is rejected). All feed the deadman on
  // success.
  // Position [rad]: clamped to the servo limits minus pos_limit_margin.
  bool writePositions(const std::vector<double>& rad);
  // Velocity [rad/s]: requires fresh position feedback (the on-servo
  // position limits are inactive outside position mode, so the host
  // enforces them): commands driving a joint past a limit are zeroed,
  // magnitudes clamp to the configured velocity_limit.
  bool writeVelocities(const std::vector<double>& rad_s);
  // Current [A]: same positional gating; magnitudes clamp to
  // effort_limit/torque_constant minus current_limit_margin.
  bool writeCurrents(const std::vector<double>& amps);

  // Soft position limits [rad] enforced by the writers above (servo
  // limits with pos_limit_margin applied); valid after activation. A
  // joint PARKED inside the margin band turns the current-mode gate
  // into a one-way wall — apps should refuse to control from there
  // (observed 2026-07-21 run 5: forearm anchored at -2.60 next to its
  // -2.64 soft limit bounced against the gate at ~6 Hz).
  const std::vector<double>& softLimitLo() const { return limit_lo_; }
  const std::vector<double>& softLimitHi() const { return limit_hi_; }

  // Background read→limit→write thread at Options::control_cycle_s.
  // Requires a homogeneous operating mode across the group. On each
  // cycle: readAll → route the latest targets through the mode's
  // limited writer → checkDeadman. stopThread() zeroes velocity/
  // current targets first (torque state is left as-is).
  bool startThread();
  void stopThread();
  bool threadRunning() const { return thread_.joinable(); }
  CycleStats cycleStats() const;
  // Blocks until a cycle newer than last_seen completes; returns its
  // sequence number (0 if the thread is not running).
  std::uint64_t waitCycle(std::uint64_t last_seen);

  // Thread-safe command targets consumed by the background thread.
  // False (with lastError()) on a size mismatch. Each successful
  // submission also feeds the deadman's submission-freshness check.
  bool setTargetPositions(const std::vector<double>& rad);
  bool setTargetVelocities(const std::vector<double>& rad_s);
  bool setTargetCurrents(const std::vector<double>& amps);

  // Deadman: escalates if the last successful command write — or, once
  // a controller has submitted targets, the last fresh submission — is
  // older than host_command_timeout_s. (Write success alone is not
  // liveness: the background thread's retransmissions succeed even
  // when the controller is dead.) Call once per control cycle. Returns
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
  bool activateSteps();
  void bestEffortRelease();
  bool requireMode(std::uint8_t mode, const char* what);
  bool requireSize(std::size_t n, const char* what);
  bool requireActive(const char* what);
  std::vector<std::uint8_t> ids() const;
  void threadLoop();

  dxl::PacketIO& io_;
  Config config_;
  Options options_;
  dxl::SyncGroup group_;
  bool activated_ = false;
  std::atomic<bool> escalated_{false};
  std::function<double()> now_;
  std::atomic<double> last_command_{0.0};
  std::string last_error_;
  std::function<void()> on_escalate_;
  // limits read from the servos at activation
  std::vector<double> limit_lo_, limit_hi_;          // [rad], margin applied
  std::vector<double> servo_current_limit_amps_;     // [A]

  mutable std::mutex state_mutex_;
  std::vector<dxl::Feedback> feedback_;     // last successful readAll
  std::vector<double> targets_;             // canonical, unit per mode
  bool have_targets_ = false;
  double last_submission_ = 0.0;   // last setTarget* call (deadman)
  bool submission_armed_ = false;  // a controller has submitted targets

  std::thread thread_;
  std::atomic<bool> thread_run_{false};
  mutable std::mutex cycle_mutex_;
  std::condition_variable cycle_cv_;
  std::uint64_t cycle_seq_ = 0;
  CycleStats stats_;
};

}  // namespace rtctrl::hw
