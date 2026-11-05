#include "rtctrl/util/clock.hpp"

#include <catch2/catch_test_macros.hpp>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <thread>

namespace util = rtctrl::util;
using Clock = std::chrono::steady_clock;

TEST_CASE("a steady_clock time point maps to its CLOCK_MONOTONIC time",
          "[util][clock]") {
  // Bracketed by clock_gettime(CLOCK_MONOTONIC) reads: on any other clock
  // (another epoch, or one counting suspend) the mapped time falls
  // outside the bracket.
  for (int i = 0; i < 1000; ++i) {
    const std::int64_t before = util::clockMonotonicNs();
    const std::int64_t t = util::monotonicNs(Clock::now());
    const std::int64_t after = util::clockMonotonicNs();
    REQUIRE(before <= t);
    REQUIRE(t <= after);
  }
}

TEST_CASE("an origin plus a stamp relative to it is the absolute time",
          "[util][clock]") {
  // The probe's log stamps count nanoseconds from its epoch; the origin
  // it records turns each back into CLOCK_MONOTONIC exactly.
  const Clock::time_point epoch = Clock::now();
  const std::int64_t origin = util::monotonicNs(epoch);
  for (const Clock::duration offset :
       {Clock::duration::zero(), Clock::duration(std::chrono::nanoseconds(1)),
        Clock::duration(std::chrono::microseconds(1234)),
        Clock::duration(std::chrono::seconds(3600))}) {
    const Clock::time_point t = epoch + offset;
    const std::int64_t stamp =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t - epoch)
            .count();
    CHECK(origin + stamp == util::monotonicNs(t));
  }
  const Clock::time_point later = Clock::now();
  const std::int64_t stamp =
      std::chrono::duration_cast<std::chrono::nanoseconds>(later - epoch)
          .count();
  CHECK(origin + stamp == util::monotonicNs(later));
}

TEST_CASE("the thread ID names the calling thread", "[util][clock]") {
  // Catch2 runs the tests on the process's main thread.
  const long main_tid = util::currentThreadId();
  CHECK(main_tid == static_cast<long>(::getpid()));
  CHECK(util::currentThreadId() == main_tid);

  long first = 0;
  long second = 0;
  std::thread worker([&] {
    first = util::currentThreadId();
    second = util::currentThreadId();
  });
  worker.join();
  CHECK(first > 0);
  CHECK(first == second);
  CHECK(first != main_tid);
}
