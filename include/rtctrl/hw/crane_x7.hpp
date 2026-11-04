#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "rtctrl/dxl/packet_io.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/arm/types.hpp"  // shared bridge DATA types only
#include "rtctrl/hw/bus_profile.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/cycle_record.hpp"
#include "rtctrl/util/spsc_ring.hpp"

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
//  2. The host-side deadman: each successful command write feeds it;
//     checkDeadman() escalates once the last one is older than
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
    double control_cycle_s = 0.01;          // background thread period
    // Maximum feedback age at a controller target submission. Zero uses
    // control_cycle_s. Tagged submissions are also rejected if any newer
    // feedback arrived while the controller was computing.
    double controller_deadline_s = 0.0;
    // Time reserved at the end of a feedback-synchronized cycle for
    // limiting, bus transmission, and safety bookkeeping.
    double controller_write_margin_s = 0.002;
    // Consecutive failed cycle reads before escalation. A controller
    // fed frozen feedback keeps commanding torques into a state it can
    // no longer see — as dangerous as a stalled command stream.
    int max_read_failures = 5;
    // Consecutive failed cycle WRITES before escalation — the mirror
    // trap: reads stay healthy while writes fail, leaving an old
    // actuator goal active. The 250 ms deadman would catch it late;
    // this catches it in ~max_write_failures cycles.
    int max_write_failures = 5;
    // Optional torque-off customization after Operating Mode and the default
    // active P gain are written, but before effective limits are read and
    // torque is enabled. Intended for the transactional parameter loader.
    // The callback must leave torque disabled on every servo. Drive Mode
    // is checked again after it (bit 3 refused); until then no cleanup,
    // after a failure or an exception, writes a goal.
    std::function<bool(dxl::PacketIO&, std::string*)>
        activation_configurator;
    // Per-cycle CycleRecords kept for popCycleRecord(), rounded up to a
    // power of two. 0 (the default) keeps no records and allocates
    // nothing; the CycleStats timing histograms are kept either way.
    std::size_t timing_ring_capacity = 0;

    // The bounded grouped read for rates above 100 Hz (control-rate plan,
    // contracts C-4 and C-4a). Disabled (the default), activation neither
    // checks the bus profile nor arms anything: the 100 Hz session.
    // Enabled, activation verifies every assumption below with read-only
    // checks before its first torque write, again after the activation
    // configurator (which can rewrite these EEPROM registers), reads the
    // programmed Bus Watchdog back, and only then arms the transport's
    // bounded read of the feedback window, just before torque-on. Any
    // failure refuses activation and leaves the bounded read unarmed;
    // the refusal's cleanup is the usual release (zero goal current
    // once Drive Mode is verified, torque off, and the watchdog cleared
    // only where the stop is confirmed).
    struct BoundedRead {
      bool enabled = false;
      // The servos' and adapter's assumptions, checked read-only.
      BusProfile profile;
      // The bounded read's limits (dxl::Port): budget positive, guard at
      // least zero.
      std::chrono::nanoseconds budget{0};
      std::chrono::nanoseconds guard{0};
      // After an ambiguous failure the bus is silent until the guard
      // ends; the next instruction packet then leaves after host
      // scheduling and transport delays. Both allowances must be
      // positive, and budget + guard + both must stay below the
      // programmed Bus Watchdog interval less one 20 ms unit, a
      // conservative engineering reserve (the documented 20 ms register
      // unit does not establish the timer's uncertainty), or the
      // watchdog could fire.
      std::chrono::nanoseconds scheduling_allowance{0};
      std::chrono::nanoseconds transport_allowance{0};
    };
    BoundedRead bounded_read;

    // What the background cycle reads (control-rate plan, Phase B2). By
    // default every cycle reads the full feedback window (current,
    // velocity, position, input voltage, temperature), the 100 Hz
    // session's read. motion_only reads current, velocity and position
    // alone and the full window only on auxiliary cycles: the thread's
    // first cycle, then every aux_interval_s rounded up to whole cycles.
    // With aux_interval_s 0 the thread never reads it, and voltage and
    // temperature keep their values from activation. Nothing else carries
    // over: whichever window the thread reads first, the writers that need
    // feedback refuse until a read of this thread succeeds, as with the
    // full read. A failed full read is retried the next cycle. The bounded
    // read must be enabled (it arms both windows); activation refuses
    // otherwise, before any write.
    // Nothing in the control path uses voltage or temperature: reading
    // them less often loses monitoring, not protection (each servo's own
    // limits and the Alert bit in every reply remain).
    //
    // instruction kFastSync (Phase B3) reads the feedback by Fast Sync
    // Read: one status packet for every servo. It too needs the bounded
    // read; activation then also checks firmware v45 or later on every
    // servo (whatever bounded_read.profile says), and reads by Fast Sync
    // Read from the arming on. The reads before it stay Sync Reads.
    struct FeedbackRead {
      bool motion_only = false;
      double aux_interval_s = 0.1;
      dxl::GroupedRead instruction = dxl::GroupedRead::kSync;
    };
    FeedbackRead feedback_read;
  };

  // What activation found while checking a bounded read's bus profile.
  struct BusDiscovery {
    // The read-only check (checkBusProfile). Its latency timer is none
    // when the device does not expose one, which leaves it unverified.
    BusProfileReport profile;
    // Each joint's Bus Watchdog, read back after activation programs it.
    std::vector<int> bus_watchdog;
    // budget + guard + both allowances, and the most it may be.
    double max_instruction_gap_s = 0.0;
    double watchdog_gap_limit_s = 0.0;
    bool armed = false;
  };

  struct CycleStats {
    std::uint64_t cycles = 0;
    std::uint64_t overruns = 0;  // cycles that finished past their deadline
    // Period boundaries deliberately skipped after an overrun. The loop
    // advances to the next future boundary instead of issuing catch-up IO.
    std::uint64_t skipped_periods = 0;
    std::uint64_t read_failures = 0;   // cycles whose feedback read failed
    std::uint64_t write_failures = 0;  // cycles whose target write failed
    std::uint64_t stale_submissions = 0;
    std::uint64_t controller_deadline_misses = 0;
    double max_cycle_time_s = 0.0;     // read through write/check completion
    double max_lateness_s = 0.0;       // finish time past scheduled deadline
    double max_feedback_age_at_submission_s = 0.0;
    // Records a full timing ring refused (never blocks the cycle).
    std::uint64_t timing_drops = 0;
    // Host-measured phase intervals (see CycleRecord for what each
    // stamp does and does not establish).
    CycleTimingStats timing;
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
  // arms the servo Bus Watchdogs, then enables torque. No motion is
  // COMMANDED — but only position mode holds the posture: current-mode
  // goals snap to zero current, so the arm is unsupported until the
  // first controller command unless setActivationCurrentPreload staged
  // a gravity current. Returns false (with lastError()) on any
  // mismatch — and a mid-sequence failure best-effort releases every
  // servo the sequence may already have touched (no partially torqued
  // arm). It refuses, before any write, a servo whose Drive Mode has
  // bit 3 (Torque On by Goal Update) set: its goal snap would enable
  // torque ahead of the sequence. It never changes Drive Mode.
  bool activate();

  // Direct release: zero goal currents, torque off, watchdogs disarmed.
  // Best-effort: continues through failures without changing gains. The
  // zero goal is written only once this activation has verified Drive
  // Mode bit 3 clear (a goal write could otherwise enable torque).
  bool deactivate();

  bool activated() const { return activated_; }
  const Config& config() const { return config_; }
  const Options& options() const { return options_; }
  const std::string& lastError() const { return last_error_; }

  // Prepare read-only observation with every servo verified torque-off. This
  // path deliberately does not program operating modes, gains, watchdogs,
  // goals, limits, or indirect-address slots. It is valid only while the arm
  // is inactive and the background thread is stopped.
  bool preparePassiveFeedback();
  bool readPassiveFeedback(std::vector<dxl::Feedback>& out);

  // Grouped IO in canonical joint order. readAll also refreshes the
  // cached feedback used by the software limiters and lastFeedback().
  bool readAll(std::vector<dxl::Feedback>& out);
  std::vector<dxl::Feedback> lastFeedback() const;  // thread-safe copy

  // Mode-checked command writers (every joint's configured operating
  // mode must match, or the call is rejected). All feed the deadman on
  // success.
  // Telemetry from one write: the post-limit values (in the write's
  // native units) and per-joint arm::kCmdClamped/kCmdGated flags.
  // Filled only on request; consumed by the background thread's
  // applied-target record.
  struct WriteOutcome {
    std::vector<double> values;
    // Optional second mode-native value. Current-based position writes use
    // this for the applied positive current ceiling [A].
    std::vector<double> auxiliary;
    std::vector<std::uint8_t> flags;
    // When limiting finished and the sync write was about to be issued;
    // left at the epoch if the writer refused before reaching the bus.
    std::chrono::steady_clock::time_point bus_write_start{};
  };

  // Position [rad]: clamped to the servo limits minus pos_limit_margin.
  bool writePositions(const std::vector<double>& rad,
                      WriteOutcome* out = nullptr);
  // Velocity [rad/s]: requires fresh position feedback (the on-servo
  // position limits are inactive outside position mode, so the host
  // enforces them): commands driving a joint past a limit are zeroed,
  // magnitudes clamp to the configured velocity_limit.
  bool writeVelocities(const std::vector<double>& rad_s,
                       WriteOutcome* out = nullptr);
  // Current [A]: same positional gating; magnitudes clamp to
  // effort_limit/torque_constant minus current_limit_margin.
  bool writeCurrents(const std::vector<double>& amps,
                     WriteOutcome* out = nullptr);
  // Current-based position: position [rad] plus a positive Goal Current
  // ceiling [A]. Position and current are sent in one grouped transaction.
  bool writeCurrentBasedPositions(const std::vector<double>& rad,
                                  const std::vector<double>& current_limit_amps,
                                  WriteOutcome* out = nullptr);

  // Feedback with its acquisition stamp: `time` is the injectable
  // now_() clock at the successful read, `seq` bumps once per
  // successful read (a failed read freezes both). One state_mutex_
  // hold — time/seq/values are mutually coherent. When `cmds` is
  // non-null, the applied-target and write-attempt records are copied
  // in the SAME hold (the background thread must not advance between
  // the feedback and the command records of one snapshot).
  struct StampedFeedback {
    std::vector<dxl::Feedback> feedback;
    double time = 0.0;
    std::uint64_t seq = 0;
  };
  StampedFeedback lastFeedbackStamped(
      arm::CommandSnapshot* cmds = nullptr) const;
  // Feedback-synchronized controller snapshot. In synchronized thread mode,
  // waits for a newer successful acquisition whose command window is still
  // open, then copies feedback and command records under the same lock.
  // Returns seq == 0 if the producer stops.
  StampedFeedback waitFeedbackStamped(
      std::uint64_t last_seen, arm::CommandSnapshot* cmds = nullptr);

  // Soft position limits [rad] enforced by the writers above (servo
  // limits with pos_limit_margin applied); valid after activation. A
  // joint PARKED inside the margin band turns the current-mode gate
  // into a one-way wall — apps should refuse to control from there
  // (observed 2026-07-21 run 5: forearm anchored at -2.60 next to its
  // -2.64 soft limit bounced against the gate at ~6 Hz).
  const std::vector<double>& softLimitLo() const { return limit_lo_; }
  const std::vector<double>& softLimitHi() const { return limit_hi_; }
  // Per-servo CurrentLimit registers [A], read at activation. Combined
  // with the config effort limits this reproduces writeCurrents' exact
  // torque clamp (e.g. for the controller's anti-windup limits).
  const std::vector<double>& servoCurrentLimitAmps() const {
    return servo_current_limit_amps_;
  }

  // Background read→limit→write thread at Options::control_cycle_s.
  // Requires a homogeneous operating mode across the group. On each
  // cycle: readAll → route the latest targets through the mode's
  // limited writer → checkDeadman. stopThread() zeroes velocity/
  // current targets first (torque state is left as-is). When
  // synchronize_commands is true, every successful read wakes a controller
  // and waits only until the cycle's write margin for a command tagged with
  // that feedback. RealArm uses this bounded same-cycle path; direct bus
  // clients retain the ordinary periodic read/write behavior.
  bool startThread(bool synchronize_commands = false);
  void stopThread();
  bool threadRunning() const { return thread_.joinable(); }
  CycleStats cycleStats() const;
  // Takes the oldest unread CycleRecord (false when none is pending or
  // Options::timing_ring_capacity is 0). One consumer thread only; it
  // never blocks the background thread, which drops records (counted in
  // CycleStats::timing_drops) rather than wait for it.
  bool popCycleRecord(CycleRecord* out);
  // Blocks until a cycle newer than last_seen completes; returns its
  // sequence number (0 if the thread is not running).
  std::uint64_t waitCycle(std::uint64_t last_seen);
  // Controller-side phase wait: blocks until a successful feedback
  // acquisition newer than last_seen opens its same-cycle command window.
  std::uint64_t waitFeedback(std::uint64_t last_seen);

  // Thread-safe command targets consumed by the background thread.
  // False (with lastError()) on a size mismatch. Each successful
  // submission also feeds the deadman's submission-freshness check and
  // is sequenced: `seq`/`time` (optional) report the submission's
  // sequence number and now_() timestamp, stored atomically with the
  // targets.
  bool setTargetPositions(const std::vector<double>& rad,
                          std::uint64_t* seq = nullptr,
                          double* time = nullptr);
  bool setTargetVelocities(const std::vector<double>& rad_s,
                           std::uint64_t* seq = nullptr,
                           double* time = nullptr);
  bool setTargetCurrents(const std::vector<double>& amps,
                         std::uint64_t* seq = nullptr,
                         double* time = nullptr);
  bool setTargetCurrentBasedPositions(
      const std::vector<double>& rad,
      const std::vector<double>& current_limit_amps,
      std::uint64_t* seq = nullptr, double* time = nullptr);
  // Controller-facing tagged variants. The target is accepted only when
  // source_feedback_seq is still the latest feedback and its age is within
  // controller_deadline_s. On rejection the last valid target remains active.
  bool setTargetPositionsFromFeedback(
      const std::vector<double>& rad, std::uint64_t source_feedback_seq,
      double source_feedback_time, std::uint64_t* seq = nullptr,
      double* time = nullptr);
  bool setTargetVelocitiesFromFeedback(
      const std::vector<double>& rad_s, std::uint64_t source_feedback_seq,
      double source_feedback_time, std::uint64_t* seq = nullptr,
      double* time = nullptr);
  bool setTargetCurrentsFromFeedback(
      const std::vector<double>& amps, std::uint64_t source_feedback_seq,
      double source_feedback_time, std::uint64_t* seq = nullptr,
      double* time = nullptr);
  bool setTargetCurrentBasedPositionsFromFeedback(
      const std::vector<double>& rad,
      const std::vector<double>& current_limit_amps,
      std::uint64_t source_feedback_seq, double source_feedback_time,
      std::uint64_t* seq = nullptr, double* time = nullptr);

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

  // Emergency bus silence (a session deadline watchdog's expiry
  // action): atomic, idempotent, safe from ANY thread — unlike
  // deactivate()/escalate(), which must never be entered concurrently.
  // Once set, the background thread stops issuing instruction packets
  // (reads AND writes — the servo Bus Watchdog counts reads too)
  // without being joined, and deactivate()/stopThread() issue no
  // further bus operations either: the quiesced cleanup only stops the
  // thread and marks the session inactive. The resulting bus silence
  // makes the armed servo Bus Watchdog stop every servo within its
  // 0.1 s interval regardless of host state. One-way for the lifetime
  // of this object. The flag is also registered with the transport
  // (PacketIO::setTransmitInhibit), so a transmission already waiting in
  // it, such as a write held by the bounded read's quiet guard, is
  // abandoned rather than sent.
  void requestQuiesce();
  bool quiesced() const { return quiesced_.load(); }

  // The last activation's bus-profile check (Options::bounded_read);
  // empty when the bounded read is disabled.
  const BusDiscovery& busDiscovery() const { return bus_discovery_; }
  bool boundedReadArmed() const { return bounded_armed_; }

  // Optional CURRENT-mode activation preload: the per-joint currents
  // [A] are clamped and position-gated exactly like writeCurrents
  // (shared limiter) and written to the goal registers BEFORE torque
  // enable, so the servos produce (e.g. gravity-holding) torque from
  // the instant torque comes on — closing the zero-current free-fall
  // interval of an activation at a gravity-loaded posture. The
  // background thread retransmits the preload until the first
  // setTargetCurrents submission replaces it. Cleared by deactivate();
  // a preload in any other operating mode fails activation.
  void setActivationCurrentPreload(const std::vector<double>& amps) {
    preload_amps_ = amps;
  }
  // Required before activating in current-based position mode. The positive
  // ceilings are staged with the measured-position goal before torque-on.
  void setActivationCurrentBasedPositionLimits(
      const std::vector<double>& amps) {
    cbp_activation_limits_amps_ = amps;
  }

  // Minimal in-place position->current transition for the integrated
  // identification startup: torque off -> operating-mode writes -> one
  // grouped goal write carrying the (limited) preload currents ->
  // torque on. ~25 transactions total; NO re-verification, NO indirect
  // re-setup, limits and Bus Watchdogs left as activated — the
  // unsupported interval is exactly the span between the first
  // torque-off and the last torque-on. Requires an ACTIVE arm with the
  // background thread stopped.
  //
  // CONTRACT on `false` — two distinct hardware states, distinguished
  // by activated():
  //  * PRE-sequence refusal (bad size, state-read failure, a preload
  //    the limiter would clip or gate — a limited gravity preload
  //    cannot hold the arm): NOTHING was written; the arm stays
  //    ACTIVE and HELD in position mode. Deliberate: a refusal must
  //    not drop a held arm. The caller must deactivate (and verify)
  //    before exiting.
  //  * Mid-sequence failure: best-effort release of every servo,
  //    activated() becomes false.
  // On success the config's operating modes and the thread's default
  // targets are updated to current/preload.
  bool switchToCurrentModeWithPreload(const std::vector<double>& amps);

  // Per-servo parameter writes (all joints). The profile setters come
  // in two forms: RAW register passthroughs (deliberate register-level
  // work — 0 means MAXIMUM on the X series) and SI variants matching
  // the vendor API's conversion exactly (truncating, clamped to
  // [1, 32767]; nonpositive requests select the SLOWEST profile,
  // never the unlimited zero).
  bool writePositionPGain(std::uint16_t gain);
  bool writeProfileVelocityRadPerSec(double rad_per_sec);
  bool writeProfileAccelerationRadPerSec2(double rad_per_sec2);
  bool writeProfileVelocity(std::uint32_t raw);
  bool writeProfileAcceleration(std::uint32_t raw);

 private:
  bool verifyServos();
  bool verifyDriveModes();
  // Options::bounded_read: the bus profile and the watchdog timing
  // (reads only), the programmed watchdog read back, and the arming.
  bool verifyBusProfile();
  bool verifyBoundedReadTiming();
  bool armBoundedRead();
  void disarmBoundedRead();
  std::uint8_t watchdogUnits() const;
  // Options::feedback_read: refuses a motion-only read or Fast Sync Read
  // without the bounded read, or an unusable interval (no bus access).
  bool verifyFeedbackRead();
  // How often the background cycle reads the full feedback window: every
  // cycle (1), every n cycles, or never (0).
  std::uint64_t fullReadEvery() const;
  // readAll (full) or the motion-only read into `out`; on success the
  // cached feedback. With `auxiliary` (the background cycle's motion-only
  // read), a full read refreshes it and a motion-only read takes voltage
  // and temperature from it.
  bool readFeedback(std::vector<dxl::Feedback>& out, bool full,
                    std::vector<dxl::Feedback>* auxiliary);
  bool activateSteps();
  void bestEffortRelease();
  bool requireMode(std::uint8_t mode, const char* what);
  bool requireSize(std::size_t n, const char* what);
  bool requireActive(const char* what);
  bool writeVelocitiesWithFeedback(
      const std::vector<double>& rad_s,
      const std::vector<dxl::Feedback>& feedback, WriteOutcome* out);
  bool writeCurrentsWithFeedback(
      const std::vector<double>& amps,
      const std::vector<dxl::Feedback>& feedback, WriteOutcome* out);
  bool writeCurrentBasedPositionsWithFeedback(
      const std::vector<double>& rad,
      const std::vector<double>& current_limit_amps,
      const std::vector<dxl::Feedback>& feedback, WriteOutcome* out);
  bool setTargets(const std::vector<double>& values,
                  const std::vector<double>* auxiliary,
                  std::uint64_t source_feedback_seq,
                  double source_feedback_time, bool tagged,
                  std::uint64_t* seq, double* time);
  // The one current limiter (URDF effort + servo current limit -
  // margin, plus the soft position gates) shared by writeCurrents and
  // the activation preload.
  void limitCurrents(const std::vector<double>& amps,
                     const std::vector<dxl::Feedback>& fb,
                     std::vector<double>* limited,
                     std::vector<std::uint8_t>* flags) const;
  void limitCurrentMagnitudes(const std::vector<double>& amps,
                              std::vector<double>* limited,
                              std::vector<std::uint8_t>* flags) const;
  std::vector<std::uint8_t> ids() const;
  void threadLoop();
  void stampControllerWake();  // caller holds state_mutex_
  // CycleRecord stamp: steady_clock ns since construction.
  std::int64_t timingNs(std::chrono::steady_clock::time_point t) const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               t - timing_epoch_)
        .count();
  }

  dxl::PacketIO& io_;
  Config config_;
  Options options_;
  dxl::SyncGroup group_;
  bool activated_ = false;
  bool passive_feedback_ready_ = false;
  // This activation read Drive Mode bit 3 clear on every servo, after the
  // activation configurator if any, so a goal write cannot enable torque
  // (EEPROM, locked while torque is on). Cleared by deactivate().
  bool drive_modes_verified_ = false;
  BusDiscovery bus_discovery_;
  bool bounded_armed_ = false;
  std::atomic<bool> escalated_{false};
  std::atomic<bool> quiesced_{false};
  std::function<double()> now_;
  std::atomic<double> last_command_{0.0};
  std::string last_error_;
  std::function<void()> on_escalate_;
  // limits read from the servos at activation
  std::vector<double> limit_lo_, limit_hi_;          // [rad], margin applied
  std::vector<double> servo_current_limit_amps_;     // [A]
  std::vector<double> preload_amps_;  // activation preload [A]; may be empty
  std::vector<double> cbp_activation_limits_amps_;

  mutable std::mutex state_mutex_;
  std::vector<dxl::Feedback> feedback_;     // last successful readAll
  double feedback_time_ = 0.0;              // now_() at acquisition
  std::uint64_t feedback_seq_ = 0;          // bumps per successful read
  std::vector<double> targets_;             // canonical, unit per mode
  std::vector<double> auxiliary_targets_;   // mode-specific paired values
  bool have_targets_ = false;
  double last_submission_ = 0.0;   // last setTarget* call (deadman)
  bool submission_armed_ = false;  // a controller has submitted targets
  std::uint64_t target_seq_ = 0;        // bumps per setTarget* call
  double target_submit_time_ = 0.0;     // now_() at that call
  std::uint64_t target_source_feedback_seq_ = 0;
  double target_source_feedback_time_ = 0.0;
  arm::AppliedTargetRecord applied_rec_;   // first/latest application
  arm::WriteAttemptRecord attempt_rec_;    // every transmission attempt
  bool synchronize_commands_ = false;
  bool command_window_open_ = false;
  std::uint64_t command_window_feedback_seq_ = 0;
  // Timing stamps crossing threads (CycleRecord ns): the first controller
  // return for the open window, and the accepted target's submission.
  std::int64_t window_controller_wake_ns_ = -1;
  std::int64_t target_submit_ns_ = -1;
  std::condition_variable feedback_cv_;
  std::condition_variable target_cv_;

  std::thread thread_;
  std::atomic<bool> thread_run_{false};
  mutable std::mutex cycle_mutex_;
  std::condition_variable cycle_cv_;
  std::uint64_t cycle_seq_ = 0;
  std::uint32_t thread_segment_ = 0;  // startThread() runs so far
  CycleStats stats_;
  const std::chrono::steady_clock::time_point timing_epoch_ =
      std::chrono::steady_clock::now();
  // Allocated by startThread (never on the background thread) when
  // Options::timing_ring_capacity > 0.
  std::unique_ptr<util::SpscRing<CycleRecord>> timing_ring_;
};

}  // namespace rtctrl::hw
