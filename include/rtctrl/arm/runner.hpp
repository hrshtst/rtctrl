#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "rtctrl/arm/arm.hpp"

namespace rtctrl::arm {

// A controller receives the current state and fills in the command for
// this cycle. t is MEASURED time relative to the run's first feedback
// sample (derived from JointState::t, never from a loop counter).
class Controller {
 public:
  virtual ~Controller() = default;
  virtual void update(const JointState& state, JointCommand& cmd,
                      double t) = 0;
};

// Invoked once per cycle AFTER writeCommand (before step) with the
// coherent snapshot read at the top of the cycle and the receipt of
// this cycle's submission — the only point where a row of telemetry
// can pair its own submission sequence with the pre-write snapshot.
// Returning false aborts the run.
class CycleObserver {
 public:
  virtual ~CycleObserver() = default;
  virtual bool observe(double t, const JointState& state,
                       const CommandSnapshot& cmds,
                       const JointCommand& cmd,
                       const CommandReceipt& receipt) = 0;
};

// Stale-feedback policy (docs/records/history.md (pass-1 record) D4). At the arm's
// ~4.5 Hz structural mode a 25 ms gap is ~40 degrees of modal phase —
// a bounded perturbation — whereas larger gaps followed by a
// trajectory catch-up could themselves excite the mode; the run aborts
// instead of catching up.
inline constexpr std::uint64_t kMaxSeqGap = 2;      // <=1 missed sample
inline constexpr double kMaxSampleInterval = 0.025;  // [s]

// Why run() returned. Each value is one distinct return of run(), so a
// caller can say which condition ended a run.
enum class RunEnd {
  kCompleted,        // measured time reached the duration (returns true)
  kInvalidDuration,  // rejected before the arm was touched
  kReadFailed,       // arm.readState() returned false
  kBackwardClock,    // a sample's timestamp was earlier than the last one
  kSequenceGap,      // the sequence advanced by more than kMaxSeqGap
  kSampleInterval,   // the measured interval exceeded kMaxSampleInterval
  kCommandRejected,  // arm.writeCommand() returned false
  kObserverVeto,     // the observer returned false
  kStepFailed,       // arm.step() returned false
  kCycleCap,         // the iteration backstop: time never reached duration
};

inline const char* describe(RunEnd end) {
  switch (end) {
    case RunEnd::kCompleted: return "completed";
    case RunEnd::kInvalidDuration: return "invalid duration";
    case RunEnd::kReadFailed: return "feedback read failed";
    case RunEnd::kBackwardClock: return "feedback timestamp went backwards";
    case RunEnd::kSequenceGap: return "feedback sequence gap";
    case RunEnd::kSampleInterval: return "feedback interval too long";
    case RunEnd::kCommandRejected: return "command rejected";
    case RunEnd::kObserverVeto: return "observer stopped the run";
    case RunEnd::kStepFailed: return "arm step failed";
    case RunEnd::kCycleCap: return "cycle cap reached";
  }
  return "unknown";
}

// Where and why a run ended, filled once when run() returns: nothing is
// printed or formatted during the run. Sample fields are NaN (times) or
// 0 (sequences) until a sample of that kind exists.
struct RunReport {
  RunEnd end = RunEnd::kCycleCap;
  long cycle = -1;  // loop iteration (0-based) in which the run ended
  // The last sample the policy accepted, and the run time of that
  // sample (relative to the first fresh sample).
  std::uint64_t accepted_seq = 0;
  double accepted_sample_t = std::numeric_limits<double>::quiet_NaN();
  double run_t = std::numeric_limits<double>::quiet_NaN();
  // The sample read in the final cycle: for a backward-clock, gap or
  // interval end, the offending sample. Unset after a failed read.
  std::uint64_t final_seq = 0;
  double final_sample_t = std::numeric_limits<double>::quiet_NaN();
  // Samples skipped as duplicates (sequence unchanged) over the run.
  std::uint64_t duplicates = 0;
};

// Drives `arm` for `duration` seconds of MEASURED time:
// readState(+snapshot) -> update -> writeCommand(+receipt) ->
// observer -> step. Policy per cycle:
//  * duplicate sample (seq unchanged): skip update/write, still step()
//    (no busy wait);
//  * backward timestamp: abort (clock fault);
//  * sequence gap > kMaxSeqGap or interval > kMaxSampleInterval: abort;
//  * an iteration cap (~4x the expected cycles) is the final backstop
//    against a clock that never advances.
// Returns false on any arm failure, policy abort, or observer veto. If
// report is given, it says which (RunReport).
inline bool run(Arm& arm, Controller& controller, double duration,
                CycleObserver* observer = nullptr,
                RunReport* report = nullptr) {
  RunReport local;
  RunReport& r = report != nullptr ? *report : local;
  r = RunReport{};
  // Validate BEFORE any arithmetic: NaN survives std::min and a
  // sufficiently negative duration produces an out-of-range value —
  // casting either to long is undefined behavior (review finding).
  // Rejected before the arm is touched at all.
  if (!std::isfinite(duration) || duration <= 0.0) {
    r.end = RunEnd::kInvalidDuration;
    return false;
  }
  // Records the end and the cycle; the sample fields are kept current
  // by the loop below.
  const auto end = [&r](RunEnd why, long cycle) {
    r.end = why;
    r.cycle = cycle;
    return why == RunEnd::kCompleted;
  };
  JointState state;
  JointCommand cmd;
  CommandSnapshot cmds;
  CommandReceipt receipt;
  const double dt = arm.dt();
  // Clamped BEFORE the cast: a huge finite duration must not convert
  // out of long's range (undefined behavior — review finding). 1e9
  // cycles sits far beyond any legitimate session while staying
  // representable.
  const long max_cycles = static_cast<long>(std::min(
                              4.0 * duration / (dt > 0.0 ? dt : 1e-3),
                              1.0e9)) +
                          8;
  // The very first read may carry an arbitrarily old sample (e.g. the
  // activation-era read), so neither the time origin nor the interval
  // policy may derive from it: it is controlled at t = 0 (commands must
  // flow every cycle — an uncommanded step would free-fall a sim or
  // leave stale goals active), and t0 latches on the first FRESH
  // sample. Controllers handle the resulting duplicate t = 0 via their
  // first-sample/duplicate branches.
  bool have_ref = false;
  bool have_t0 = false;
  double t0 = 0.0;
  double t_prev = 0.0;
  std::uint64_t seq_prev = 0;
  long cycle = 0;
  for (; cycle < max_cycles; ++cycle) {
    if (!arm.readState(state, &cmds)) return end(RunEnd::kReadFailed, cycle);
    r.final_seq = state.seq;
    r.final_sample_t = state.t;
    double t = 0.0;
    if (!have_ref) {
      seq_prev = state.seq;
      have_ref = true;
    } else if (state.seq == seq_prev) {  // duplicate feedback
      ++r.duplicates;
      if (!arm.step()) return end(RunEnd::kStepFailed, cycle);
      continue;
    } else if (!have_t0) {  // first fresh sample defines the origin
      t0 = t_prev = state.t;
      seq_prev = state.seq;
      have_t0 = true;
    } else {
      if (state.t < t_prev) return end(RunEnd::kBackwardClock, cycle);
      if (state.seq - seq_prev > kMaxSeqGap) {
        return end(RunEnd::kSequenceGap, cycle);
      }
      if (state.t - t_prev > kMaxSampleInterval) {
        return end(RunEnd::kSampleInterval, cycle);
      }
      t_prev = state.t;
      seq_prev = state.seq;
      t = state.t - t0;
    }
    r.accepted_seq = state.seq;
    r.accepted_sample_t = state.t;
    r.run_t = t;
    // epsilon: FP-robust endpoint
    if (t >= duration - 1e-9) return end(RunEnd::kCompleted, cycle);
    controller.update(state, cmd, t);
    if (!arm.writeCommand(cmd, &receipt)) {
      return end(RunEnd::kCommandRejected, cycle);
    }

    if (observer != nullptr &&
        !observer->observe(t, state, cmds, cmd, receipt)) {
      return end(RunEnd::kObserverVeto, cycle);
    }
    if (!arm.step()) return end(RunEnd::kStepFailed, cycle);
  }
  // backstop: measured time never reached `duration`
  return end(RunEnd::kCycleCap, cycle);
}

}  // namespace rtctrl::arm
