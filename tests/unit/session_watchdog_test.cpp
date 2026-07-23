// The session deadline watchdog must fire exactly once on expiry even
// while the arming thread is blocked, and never once disarmed — the
// independence the identification session's 180 s bound relies on.
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <thread>

#include "session_watchdog.hpp"

using namespace std::chrono_literals;

TEST_CASE("session watchdog does not fire before the deadline",
          "[session_watchdog]") {
  std::atomic<int> fired{0};
  x7::SessionWatchdog wd(10s, [&] { ++fired; });
  std::this_thread::sleep_for(50ms);
  CHECK_FALSE(wd.fired());
  wd.disarm();
  std::this_thread::sleep_for(50ms);
  CHECK(fired.load() == 0);
}

TEST_CASE("session watchdog never fires after disarm", "[session_watchdog]") {
  std::atomic<int> fired{0};
  {
    x7::SessionWatchdog wd(100ms, [&] { ++fired; });
    wd.disarm();  // returns only after the watchdog thread has exited
  }
  std::this_thread::sleep_for(250ms);
  CHECK(fired.load() == 0);
}

TEST_CASE("session watchdog fires exactly once while the arming thread "
          "is blocked",
          "[session_watchdog]") {
  std::atomic<int> fired{0};
  x7::SessionWatchdog wd(50ms, [&] { ++fired; });
  // the arming thread deliberately blocks, as it would inside a hung
  // synchronous deactivate(); the watchdog must fire regardless
  std::this_thread::sleep_for(300ms);
  CHECK(fired.load() == 1);
  CHECK(wd.fired());
  wd.disarm();  // safe after fire; still exactly once
  std::this_thread::sleep_for(50ms);
  CHECK(fired.load() == 1);
}
