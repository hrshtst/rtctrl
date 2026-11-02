#include "rtctrl/util/realtime.hpp"

#include <catch2/catch_test_macros.hpp>
#include <pthread.h>
#include <sched.h>

#include <string>
#include <thread>

using rtctrl::util::RealtimePolicy;
using rtctrl::util::RealtimeState;
using rtctrl::util::applyRealtimePolicy;
using rtctrl::util::currentRealtimeState;
using rtctrl::util::validateRealtimePolicy;

namespace {

// What one application attempt did to the thread it ran on.
struct Attempt {
  RealtimeState before;
  RealtimeState after;
  bool granted = false;
  std::string error;
};

// Applies the policy on a fresh thread, so the test runner's own thread
// keeps its scheduling whatever is granted. Assertions stay on the main
// thread: Catch2 assertions are not for worker threads.
Attempt applyOnFreshThread(const RealtimePolicy& policy) {
  Attempt attempt;
  std::thread t([&] {
    attempt.before = currentRealtimeState();
    attempt.error = "stale";
    attempt.granted = applyRealtimePolicy(policy, &attempt.error);
    attempt.after = currentRealtimeState();
  });
  t.join();
  return attempt;
}

int firstAllowedCpu() {
  cpu_set_t set;
  CPU_ZERO(&set);
  REQUIRE(pthread_getaffinity_np(pthread_self(), sizeof(set), &set) == 0);
  for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
    if (CPU_ISSET(cpu, &set)) return cpu;
  }
  FAIL("no allowed cpu");
  return -1;
}

bool sameState(const RealtimeState& a, const RealtimeState& b) {
  return a.policy == b.policy && a.priority == b.priority &&
         a.timer_slack_ns == b.timer_slack_ns && a.cpus == b.cpus;
}

}  // namespace

TEST_CASE("the default real-time policy is valid and changes nothing") {
  const RealtimePolicy policy;
  CHECK(policy.isDefault());
  CHECK(validateRealtimePolicy(policy).empty());
  const Attempt attempt = applyOnFreshThread(policy);
  CHECK(attempt.granted);
  CHECK(attempt.error.empty());
  CHECK(sameState(attempt.before, attempt.after));
}

TEST_CASE("invalid real-time policies are refused before anything applies") {
  RealtimePolicy policy;
  SECTION("negative priority") { policy.priority = -1; }
  SECTION("priority above 99") {
    policy.priority = 100;
    policy.cpu = 0;
  }
  SECTION("cpu below -1") { policy.cpu = -2; }
  SECTION("cpu beyond the cpu set") { policy.cpu = CPU_SETSIZE; }
  SECTION("priority without a dedicated cpu") {
    policy.priority = 10;
    CHECK(validateRealtimePolicy(policy).find("dedicated cpu") !=
          std::string::npos);
  }
  CHECK_FALSE(validateRealtimePolicy(policy).empty());
  const Attempt attempt = applyOnFreshThread(policy);
  CHECK_FALSE(attempt.granted);
  CHECK(attempt.error == validateRealtimePolicy(policy));
  CHECK(sameState(attempt.before, attempt.after));
}

TEST_CASE("unprivileged parts of a real-time policy apply to the thread") {
  RealtimePolicy policy;
  policy.cpu = firstAllowedCpu();
  policy.fine_timer_slack = true;
  const Attempt attempt = applyOnFreshThread(policy);
  REQUIRE(attempt.granted);
  CHECK(attempt.error.empty());
  CHECK(attempt.after.timer_slack_ns == 1);
  CHECK(attempt.after.cpus == std::to_string(policy.cpu));
  CHECK(rtctrl::util::describeRealtimeState(attempt.after)
            .find("timer slack 1 ns") != std::string::npos);
}

TEST_CASE("a real-time priority is granted whole or refused and restored") {
  // Unprivileged environments (no RLIMIT_RTPRIO, no CAP_SYS_NICE) refuse
  // SCHED_FIFO; privileged ones grant it. Either way the outcome is all
  // or nothing: the affinity and timer slack applied before a refused
  // priority are rolled back.
  RealtimePolicy policy;
  policy.priority = 1;
  policy.cpu = firstAllowedCpu();
  policy.fine_timer_slack = true;
  const Attempt attempt = applyOnFreshThread(policy);
  if (attempt.granted) {
    CHECK(attempt.after.policy == SCHED_FIFO);
    CHECK(attempt.after.priority == 1);
    CHECK(attempt.after.timer_slack_ns == 1);
    CHECK(attempt.after.cpus == std::to_string(policy.cpu));
  } else {
    CHECK(attempt.error.find("SCHED_FIFO priority 1") != std::string::npos);
    CHECK(sameState(attempt.before, attempt.after));
  }
}
