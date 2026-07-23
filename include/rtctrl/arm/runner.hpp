#pragma once

#include <cstdint>

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

// Stale-feedback policy (docs/REMEDIATION_PLAN.md D4). At the arm's
// ~4.5 Hz structural mode a 25 ms gap is ~40 degrees of modal phase —
// a bounded perturbation — whereas larger gaps followed by a
// trajectory catch-up could themselves excite the mode; the run aborts
// instead of catching up.
inline constexpr std::uint64_t kMaxSeqGap = 2;      // <=1 missed sample
inline constexpr double kMaxSampleInterval = 0.025;  // [s]

// Drives `arm` for `duration` seconds of MEASURED time:
// readState(+snapshot) -> update -> writeCommand(+receipt) ->
// observer -> step. Policy per cycle:
//  * duplicate sample (seq unchanged): skip update/write, still step()
//    (no busy wait);
//  * backward timestamp: abort (clock fault);
//  * sequence gap > kMaxSeqGap or interval > kMaxSampleInterval: abort;
//  * an iteration cap (~4x the expected cycles) is the final backstop
//    against a clock that never advances.
// Returns false on any arm failure, policy abort, or observer veto.
inline bool run(Arm& arm, Controller& controller, double duration,
                CycleObserver* observer = nullptr) {
  JointState state;
  JointCommand cmd;
  CommandSnapshot cmds;
  CommandReceipt receipt;
  const double dt = arm.dt();
  const long max_cycles =
      static_cast<long>(4.0 * duration / (dt > 0.0 ? dt : 1e-3)) + 8;
  bool have_t0 = false;
  double t0 = 0.0;
  double t_prev = 0.0;
  std::uint64_t seq_prev = 0;
  for (long cycle = 0; cycle < max_cycles; ++cycle) {
    if (!arm.readState(state, &cmds)) return false;
    if (!have_t0) {
      t0 = t_prev = state.t;
      seq_prev = state.seq;
      have_t0 = true;
    } else {
      if (state.t < t_prev) return false;  // backward clock
      if (state.seq == seq_prev) {         // duplicate feedback
        if (!arm.step()) return false;
        continue;
      }
      if (state.seq - seq_prev > kMaxSeqGap) return false;
      if (state.t - t_prev > kMaxSampleInterval) return false;
      t_prev = state.t;
      seq_prev = state.seq;
    }
    const double t = state.t - t0;
    if (t >= duration - 1e-9) return true;  // epsilon: FP-robust endpoint
    controller.update(state, cmd, t);
    if (!arm.writeCommand(cmd, &receipt)) return false;
    if (observer != nullptr &&
        !observer->observe(t, state, cmds, cmd, receipt)) {
      return false;
    }
    if (!arm.step()) return false;
  }
  return false;  // backstop: measured time never reached `duration`
}

}  // namespace rtctrl::arm
