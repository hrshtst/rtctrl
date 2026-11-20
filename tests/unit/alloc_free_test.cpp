// Allocation accounting for the background thread's timing path. Lives in
// its own executable because it replaces the global operator new.
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdlib>
#include <new>

#include "bus/bus_timing_adapter.hpp"
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
    plan = sched.closeAndPublish(stamp, stamp + 5000, stamp + 5000);
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

TEST_CASE("the lead alternation (sequencer, offset plans, paired detector) never allocates once locked",
          "[hw][adapter][coupling]") {
  namespace ad = rtctrl::hw::adapter;
  ad::ScheduleConfig cfg;
  cfg.lead_alternation_ns = 20000;
  ad::ReferencedSchedule sched(cfg, 0);
  // the same lattice and replies as above: independent of the send time
  const double period = 1e6 * (1 - 57.5e-6);
  auto completion_after = [&](double t) {
    const double k = std::ceil((t - 412000.0) / period);
    return 412000.0 + k * period;
  };
  ad::WakePlan plan = sched.first();
  std::int64_t k = 0;
  auto cycle = [&]() {
    const bool full = k % 100 == 0, next_full = (k + 1) % 100 == 0;
    ++k;
    const double ready = static_cast<double>(plan.wake_ns) + 10000.0 + 660000.0 + (full ? 110000.0 : 0.0);
    const auto stamp = static_cast<std::int64_t>(completion_after(ready) + 10000.0);
    plan = sched.closeAndPublish(stamp, stamp + 5000, stamp + 5000, full ? ad::ReadClass::kFull : ad::ReadClass::kMotion,
                                 next_full ? ad::ReadClass::kFull : ad::ReadClass::kMotion);
  };
  while (sched.estimator().state() != ad::EstimatorState::kLocked || !plan.referenced) {
    cycle();
    REQUIRE(plan.wake_ns < 10000000000LL);
  }
  const std::uint64_t evaluations_before = sched.coupling().evaluations();
  g_counting = true;
  g_allocations = 0;
  for (int i = 0; i < 20000; ++i) cycle();
  const std::size_t allocations = g_allocations;
  g_counting = false;
  CHECK(allocations == 0);
  CHECK(sched.estimator().state() == ad::EstimatorState::kLocked);
  CHECK(sched.coupling().evaluations() - evaluations_before > 9000);  // the median ran, in the counted loop
  CHECK(sched.coupling().state() == ad::CouplingState::kNoCouplingDetected);
}

TEST_CASE("the worker's side of the fit exchange and publication never allocate",
          "[hw][adapter]") {
  namespace ad = rtctrl::hw::adapter;
  ad::ScheduleConfig cfg;
  ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
  ad::ReferencedSchedule sched(cfg, 0, &ex);
  const double period = 1e6 * (1 - 57.5e-6);
  auto completion_after = [&](double t) {
    const double k = std::ceil((t - 412000.0) / period);
    return 412000.0 + k * period;
  };
  ad::WakePlan plan = sched.first();
  std::size_t worker_allocations = 0;
  // Acquisition, the hand-off and tracking, on the worker's calls only: the
  // fitter's service() (it allocates) runs with counting off.
  for (int i = 0; i < 25000; ++i) {
    const double ready = static_cast<double>(plan.wake_ns) + 10000.0 + 660000.0;
    const auto stamp = static_cast<std::int64_t>(completion_after(ready) + 10000.0);
    g_counting = true;
    g_allocations = 0;
    sched.closeCycle(stamp, stamp + 5000, stamp + 5000);
    plan = sched.publish(stamp + 6000);
    worker_allocations += g_allocations;
    g_counting = false;
    ex.service();
  }
  CHECK(worker_allocations == 0);
  CHECK(sched.counts().fits_applied == 1);
  CHECK(sched.estimator().state() == ad::EstimatorState::kLocked);
  CHECK(sched.estimator().counts().gated > 20000);
}

TEST_CASE("x7_bus_timing's adapter driver never allocates once constructed",
          "[bus_timing][adapter]") {
  // The probe's worker calls (close, publish, finish) through acquisition,
  // the hand-off, locks on request-tied replies, their losses and fallbacks,
  // with the alternation and full-window reads; the fitter's service() (it
  // allocates) runs with counting off, as the probe's main thread.
  namespace ad = rtctrl::hw::adapter;
  namespace bt = rtctrl::apps::bus_timing;
  bt::TimingChoice c;
  c.adapter = true;
  c.rate_hz = 1000.0;
  c.duration_s = 60.0;
  c.bounded = true;
  c.motion_only = true;
  c.full_every = 100;
  c.acquisition_s = 0.6;
  c.settling_s = 0.0;
  c.alternation_us = 20.0;
  REQUIRE(bt::timingChoiceError(c).empty());
  const ad::ScheduleConfig cfg = bt::scheduleConfig(c);
  ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
  bt::AdapterCycles a(cfg, &ex);
  std::int64_t wake = a.first();
  std::uint64_t full_due_in = 0;
  std::size_t worker_allocations = 0;
  for (std::uint64_t k = 1; k <= 30000; ++k) {
    const std::int64_t read_start = wake + 7000;
    const bool full = full_due_in == 0;
    if (!full) --full_due_in;
    else full_due_in = 99;
    // 150 us after the request: a false lock, lost after 64 cycles, again.
    const std::int64_t stamp = read_start + 150000 + 500 * static_cast<std::int64_t>(k % 7);
    const std::int64_t end = stamp + 40000;
    g_counting = true;
    g_allocations = 0;
    a.close(k, stamp, end, end + 1000, full ? ad::ReadClass::kFull : ad::ReadClass::kMotion,
            full_due_in == 0 ? ad::ReadClass::kFull : ad::ReadClass::kMotion);
    bt::AdapterRow row;
    wake = a.publish(end + 3000, &row).wake_ns;
    worker_allocations += g_allocations;
    g_counting = false;
    ex.service();
  }
  g_counting = true;
  g_allocations = 0;
  a.finish();
  worker_allocations += g_allocations;
  g_counting = false;
  CHECK(worker_allocations == 0);
  CHECK(a.lockCount() >= 10);
  CHECK(a.schedule().estimator().counts().losses >= 10);
  CHECK(a.fitCount() >= 20);
}
