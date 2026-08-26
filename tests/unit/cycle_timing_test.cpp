#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <limits>

#include "common/periodic_loop.hpp"
#include "rtctrl/hw/cycle_timing.hpp"

namespace hw = rtctrl::hw;

TEST_CASE("absolute cycle deadline keeps phase when work finishes early",
          "[hw][timing]") {
  using namespace std::chrono_literals;
  const std::chrono::steady_clock::time_point origin{};
  const auto result =
      hw::advanceCycleDeadline(origin + 10ms, origin + 4ms, 10ms);

  CHECK(result.wake_time == origin + 10ms);
  CHECK(result.lateness == std::chrono::steady_clock::duration::zero());
  CHECK(result.skipped_periods == 0);
}

TEST_CASE("late hardware cycle skips catch-up periods",
          "[hw][timing][safety]") {
  using namespace std::chrono_literals;
  const std::chrono::steady_clock::time_point origin{};

  SECTION("a small overrun drops the immediately missed period") {
    const auto result =
        hw::advanceCycleDeadline(origin + 10ms, origin + 11ms, 10ms);
    CHECK(result.wake_time == origin + 20ms);
    CHECK(result.lateness == 1ms);
    CHECK(result.skipped_periods == 1);
  }

  SECTION("a serial timeout advances directly beyond every missed slot") {
    const auto result =
        hw::advanceCycleDeadline(origin + 10ms, origin + 35ms, 10ms);
    CHECK(result.wake_time == origin + 40ms);
    CHECK(result.lateness == 25ms);
    CHECK(result.skipped_periods == 3);
  }
}

TEST_CASE("periodic schedule derives elapsed time from the monotonic clock",
          "[hw][timing]") {
  using namespace std::chrono_literals;
  const std::chrono::steady_clock::time_point origin{};
  hw::PeriodicSchedule schedule(origin, 10ms);

  CHECK(schedule.elapsed(origin + 35ms) == 0.035);
  auto first = schedule.closeIteration(origin + 4ms);
  CHECK(first.wake_time == origin + 10ms);
  CHECK(first.skipped_periods == 0);

  // The second iteration misses the 20 ms boundary by 15 ms. It advances
  // straight to 40 ms, and the following boundary remains phase-aligned.
  auto late = schedule.closeIteration(origin + 35ms);
  CHECK(late.wake_time == origin + 40ms);
  CHECK(late.skipped_periods == 2);
  auto next = schedule.closeIteration(origin + 44ms);
  CHECK(next.wake_time == origin + 50ms);
  CHECK(next.skipped_periods == 0);
}

TEST_CASE("direct-loop period validation rejects unusable clocks",
          "[hw][timing]") {
  CHECK_THROWS(x7::PeriodicLoop(0.0));
  CHECK_THROWS(x7::PeriodicLoop(-0.01));
  CHECK_THROWS(x7::PeriodicLoop(std::numeric_limits<double>::quiet_NaN()));
}
