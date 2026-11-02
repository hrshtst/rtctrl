// Allocation accounting for the background thread's timing path. Lives in
// its own executable because it replaces the global operator new.
#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <new>

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
