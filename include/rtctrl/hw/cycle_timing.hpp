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

// Pure absolute-period schedule. Call closeIteration() after each iteration,
// sleep until its returned wake_time, then begin the next iteration. Passing
// time points in makes drift/overrun behavior deterministic in unit tests.
class PeriodicSchedule {
 public:
  PeriodicSchedule(std::chrono::steady_clock::time_point start,
                   std::chrono::steady_clock::duration period)
      : start_(start), boundary_(start), period_(period) {}

  double elapsed(std::chrono::steady_clock::time_point now) const {
    return std::chrono::duration<double>(now - start_).count();
  }

  DeadlineAdvance closeIteration(
      std::chrono::steady_clock::time_point finished) {
    boundary_ += period_;
    auto result = advanceCycleDeadline(boundary_, finished, period_);
    boundary_ = result.wake_time;
    return result;
  }

 private:
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point boundary_;
  std::chrono::steady_clock::duration period_;
};

}  // namespace rtctrl::hw
