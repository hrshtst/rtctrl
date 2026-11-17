#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace rtctrl::emu {

class MotorBus;
class PtyBus;

// The standalone emulator's loop (dxl_emu): serves a PtyBus as its input
// arrives and keeps the motor model on wall time. The in-process fixtures
// keep driving PtyBus::poll(dt) themselves; this loop is not theirs.
//
// - Model time: the bus ticks on an absolute schedule of the steady
//   clock, start + k * tick (k = 1, 2, ...), however often the loop
//   wakes; model time is ticks * tick, behind wall time by less than one
//   tick while the loop keeps up.
// - Order: a step first runs every tick due by its time, THEN services
//   the pty. A command read after a stall therefore applies after the
//   elapsed model time, not retroactively over it, and an overdue Bus
//   Watchdog expires before the packet that would have fed it.
// - Catch-up cap: a step owing more than max_catch_up_ticks runs none of
//   them and reports kOverrun: model time could not be kept on wall time,
//   a test-rig failure (dxl_emu exits non-zero). Elapsed model time is
//   never dropped silently.
// - Bounded input: a step reads at most max_read_bytes, so sustained
//   input cannot postpone the ticks, the due replies or a stop request;
//   the rest stays in the pty for the next step.
// - Waits: until the pty is readable or the deadline (the next tick, or
//   the earliest delayed reply if sooner). While no client holds the pty
//   after one has detached, the master reports a hangup at once; the loop
//   then waits by time alone (no spin) and returns to readiness waits
//   when a client is back.
class StandaloneLoop {
 public:
  using Clock = std::chrono::steady_clock;

  struct Config {
    std::chrono::nanoseconds tick{std::chrono::milliseconds(1)};
    std::uint64_t max_catch_up_ticks = 50;
    std::size_t max_read_bytes = 4096;
  };

  enum class Status {
    kOk,
    kOverrun,  // more than max_catch_up_ticks were due at one step
    kIoError,  // the pty failed
  };

  struct Stats {
    std::uint64_t ticks = 0;             // model time = ticks * tick
    std::uint64_t steps = 0;
    std::uint64_t late_steps = 0;        // steps that ran more than one tick
    std::uint64_t max_ticks_in_step = 0;
    std::uint64_t limited_reads = 0;     // steps whose read hit the bound
    std::uint64_t hangup_waits = 0;      // waits by time alone (detached)
    std::uint64_t overrun_due = 0;       // ticks due at the overrun, if any
  };

  // Serves pty (which serves bus); the schedule starts at start.
  StandaloneLoop(PtyBus& pty, MotorBus& bus, const Config& config,
                 Clock::time_point start);

  // One step at now, in time order: the ticks due by now, then
  // PtyBus::serviceIo(max_read_bytes).
  Status step(Clock::time_point now);

  // When the next step is owed: the next tick, or the earliest delayed
  // reply if that is sooner.
  Clock::time_point deadline() const;

  // Waits (real clock) until the pty is readable, the deadline passes, or
  // a signal interrupts the wait; by time alone while the pty reports a
  // hangup. False if the pty cannot be waited on.
  bool wait(Clock::time_point deadline);

  // Steps and waits until stop() is true (checked before each step and
  // after it) or a step fails. A stop request is therefore seen within
  // one step and one wait, and no wait outlasts the next deadline.
  Status run(const std::function<bool()>& stop);

  Clock::time_point nextTick() const { return next_tick_; }
  const Stats& stats() const { return stats_; }
  const Config& config() const { return config_; }

 private:
  PtyBus& pty_;
  MotorBus& bus_;
  Config config_;
  double dt_;  // the tick in seconds, as MotorBus::tick takes it
  Clock::time_point next_tick_;
  Stats stats_;
};

}  // namespace rtctrl::emu
