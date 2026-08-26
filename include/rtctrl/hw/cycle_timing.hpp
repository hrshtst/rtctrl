#pragma once

#include <chrono>
#include <cstdint>

namespace rtctrl::hw {

// Result of closing one absolute-deadline cycle. If work finished late,
// wake_time is advanced directly to the next future phase boundary so the
// hardware loop never emits catch-up bursts.
struct DeadlineAdvance {
  std::chrono::steady_clock::time_point wake_time;
  std::chrono::steady_clock::duration lateness{};
  std::uint64_t skipped_periods = 0;
};

inline DeadlineAdvance advanceCycleDeadline(
    std::chrono::steady_clock::time_point deadline,
    std::chrono::steady_clock::time_point finished,
    std::chrono::steady_clock::duration period) {
  DeadlineAdvance result{deadline};
  if (finished <= deadline) return result;

  result.lateness = finished - deadline;
  result.skipped_periods =
      1 + static_cast<std::uint64_t>(result.lateness / period);
  result.wake_time = deadline + period * result.skipped_periods;
  return result;
}

}  // namespace rtctrl::hw
