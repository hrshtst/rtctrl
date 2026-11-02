#pragma once

#include <string>

namespace rtctrl::util {

// Real-time scheduling for a control or measurement thread: opt-in, and
// either granted completely or refused.
//
// The default policy changes nothing. A non-default policy is applied to
// the calling thread; if any part of it is not granted, the thread's
// previous affinity, timer slack and scheduling are restored and the call
// reports why. A caller that then refuses to start leaves no partial
// setting behind. Running a loop under less than the requested policy
// would misattribute its timing, so there is no best-effort mode.
//
// A SCHED_FIFO priority requires a dedicated CPU: the Dynamixel receive
// path busy-polls, and a real-time thread spinning on a shared core can
// starve the threads it waits for.
struct RealtimePolicy {
  int priority = 0;               // 0 = normal scheduling; 1..99 = SCHED_FIFO
  int cpu = -1;                   // -1 = no affinity; otherwise pin to it
  bool lock_memory = false;       // mlockall(MCL_CURRENT | MCL_FUTURE)
  bool fine_timer_slack = false;  // 1 ns instead of the 50 us default

  bool isDefault() const {
    return priority == 0 && cpu == -1 && !lock_memory && !fine_timer_slack;
  }
};

// Static checks: ranges and the dedicated-CPU rule. Empty when valid.
std::string validateRealtimePolicy(const RealtimePolicy& policy);

// Applies policy to the calling thread. Returns false and sets *error
// (when non-null) if the policy is invalid or any part is not granted;
// the thread's previous settings are then restored. The memory lock is
// process-wide and is attempted last, so a refusal never leaves it set.
bool applyRealtimePolicy(const RealtimePolicy& policy, std::string* error);

// The calling thread's current settings, for reports.
struct RealtimeState {
  int policy = 0;  // SCHED_OTHER, SCHED_FIFO, ...
  int priority = 0;
  long timer_slack_ns = 0;
  std::string cpus;  // affinity as a CPU list, e.g. "0-3" or "2"
};
RealtimeState currentRealtimeState();
std::string describeRealtimeState(const RealtimeState& state);

}  // namespace rtctrl::util
