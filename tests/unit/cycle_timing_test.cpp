#include <catch2/catch_test_macros.hpp>

#include <chrono>

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
