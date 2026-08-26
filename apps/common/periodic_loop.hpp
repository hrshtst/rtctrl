#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <thread>

#include "rtctrl/hw/cycle_timing.hpp"

namespace x7 {

// Monotonic, absolute-deadline pacing for direct hardware loops. elapsed()
// reports wall time rather than an iteration count; waitNext() skips missed
// boundaries instead of accumulating work time or issuing catch-up commands.
class PeriodicLoop {
 public:
  explicit PeriodicLoop(double period_s)
      : period_(toDuration(period_s)), schedule_(Clock::now(), period_) {}

  double elapsed() const { return schedule_.elapsed(Clock::now()); }

  rtctrl::hw::DeadlineAdvance waitNext() {
    auto result = schedule_.closeIteration(Clock::now());
    skipped_periods_ += result.skipped_periods;
    max_lateness_ = std::max(max_lateness_, result.lateness);
    std::this_thread::sleep_until(result.wake_time);
    return result;
  }

  std::uint64_t skippedPeriods() const { return skipped_periods_; }
  double maxLateness() const {
    return std::chrono::duration<double>(max_lateness_).count();
  }

 private:
  using Clock = std::chrono::steady_clock;
  static Clock::duration toDuration(double period_s) {
    if (!std::isfinite(period_s) || period_s <= 0.0) {
      throw std::invalid_argument(
          "PeriodicLoop: period must be finite and positive");
    }
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(period_s));
    if (period <= Clock::duration::zero()) {
      throw std::invalid_argument(
          "PeriodicLoop: period is below clock resolution");
    }
    return period;
  }
  Clock::duration period_;
  rtctrl::hw::PeriodicSchedule schedule_;
  std::uint64_t skipped_periods_ = 0;
  Clock::duration max_lateness_{};
};

}  // namespace x7
