// Allocation accounting for the background thread's timing path. Lives in
// its own executable because it replaces the global operator new.
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdlib>
#include <new>

#include "rtctrl/hw/adapter_phase.hpp"
#include "rtctrl/hw/cycle_record.hpp"
#include "rtctrl/util/spsc_ring.hpp"

namespace {
thread_local bool g_counting = false;
thread_local std::size_t g_allocations = 0;
}  // namespace

void* operator new(std::size_t n) {
  if (g_counting) ++g_allocations;
  if (void* p = std::malloc(n == 0 ? 1 : n)) return p;
  throw std::bad_alloc();
}
void* operator new[](std::size_t n) { return operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

TEST_CASE("recording, accumulating and publishing a cycle never allocates",
          "[hw][timing]") {
  rtctrl::util::SpscRing<rtctrl::hw::CycleRecord> ring(64);
  rtctrl::hw::CycleTimingStats stats;
  g_counting = true;
  g_allocations = 0;
  std::int64_t prev_wake = -1;
  for (int i = 0; i < 10000; ++i) {
    rtctrl::hw::CycleRecord r;
    r.cycle = static_cast<std::uint64_t>(i + 1);
    r.scheduled_wake_ns = i * 1000000LL;
    r.wake_ns = r.scheduled_wake_ns + 20000;
    r.read_start_ns = r.wake_ns + 1000;
    r.read_end_ns = r.read_start_ns + 650000;
    r.limit_start_ns = r.read_end_ns + 50000;
    r.bus_write_start_ns = r.limit_start_ns + 5000;
    r.write_return_ns = r.bus_write_start_ns + 30000;
    r.cycle_end_ns = r.write_return_ns + 5000;
    rtctrl::hw::accumulateCycleTiming(r, prev_wake, &stats);
    prev_wake = r.wake_ns;
    if (!ring.push(r)) {
      rtctrl::hw::CycleRecord drained;
      while (ring.pop(&drained)) {
      }
    }
  }
  const std::size_t allocations = g_allocations;
  g_counting = false;
  CHECK(allocations == 0);
  CHECK(stats.host_work.count() == 10000);
}

TEST_CASE("adapter-referenced tracking, planning and accounting never allocate once locked",
          "[hw][adapter]") {
  namespace ad = rtctrl::hw::adapter;
  ad::ScheduleConfig cfg;
  ad::ReferencedSchedule sched(cfg, 0);
  // A lattice at 412 us + k * 999.9425 us; replies 10 us after the targeted
  // completion; acquisition on the grid first (its fit allocates, uncounted).
  const double period = 1e6 * (1 - 57.5e-6);
  auto completion_after = [&](double t) {
    const double k = std::ceil((t - 412000.0) / period);
    return 412000.0 + k * period;
  };
  ad::WakePlan plan = sched.first();
  auto cycle = [&]() {
    const double ready = static_cast<double>(plan.wake_ns) + 10000.0 + 660000.0;
    const auto stamp = static_cast<std::int64_t>(completion_after(ready) + 10000.0);
    plan = sched.closeCycle(stamp, stamp + 5000, stamp + 5000);
  };
  while (sched.estimator().state() != ad::EstimatorState::kLocked ||
         !plan.referenced) {
    cycle();
    REQUIRE(plan.wake_ns < 10000000000LL);
  }
  g_counting = true;
  g_allocations = 0;
  for (int i = 0; i < 20000; ++i) cycle();
  const std::size_t allocations = g_allocations;
  g_counting = false;
  CHECK(allocations == 0);
  CHECK(sched.estimator().state() == ad::EstimatorState::kLocked);
  CHECK(sched.estimator().counts().gated >= 20000);
}
