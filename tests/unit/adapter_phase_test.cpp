// Adapter-referenced timing (rtctrl/hw/adapter_phase.hpp): the acquisition
// fit, the estimator, the planner and the skip accounting, against synthetic
// ground truth. The simulated adapter has a true completion lattice; the host
// records each completion with jitter; replies ride the first completion after
// they are ready and reach the probe a receive delay later, sometimes whole
// periods later. Estimator error is measured against the true lattice, apart
// from the recorded timestamps' jitter.
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <thread>
#include <atomic>
#include <vector>

#include "rtctrl/hw/adapter_phase.hpp"
#include "rtctrl/hw/cycle_timing.hpp"

namespace ad = rtctrl::hw::adapter;

namespace {

constexpr double kP = 1e6;

std::uint64_t mix(std::uint64_t x) {  // splitmix64
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
double unit(std::uint64_t seed, std::int64_t n) {  // deterministic in [0, 1)
  return static_cast<double>(mix(seed ^ static_cast<std::uint64_t>(n)) >> 11) * 0x1.0p-53;
}

// The adapter's true completions on the host's clock, piecewise: a period,
// an optional change of period (a change of the host clock's discipline) and
// an optional phase step; and the host's record of each, with jitter.
struct Lattice {
  double t0 = 412000.0;
  double period = kP * (1 - 57.5e-6);
  double change_at = std::numeric_limits<double>::infinity();
  double period_after = kP;
  double step_at = std::numeric_limits<double>::infinity();
  double step = 0.0;
  // The host's record of a completion: 90 % within +-3 us, 10 % from +3 to
  // +30 us late (the traced lattices' residuals: p1 -12, p50 -0.6, p99 +30 us).
  double jitter_core = 3000.0, jitter_tail = 30000.0, tail_share = 0.1;
  std::uint64_t seed = 1;

  std::int64_t changeIndex() const {
    if (!std::isfinite(change_at)) return std::numeric_limits<std::int64_t>::max();
    return static_cast<std::int64_t>(std::ceil((change_at - t0) / period));
  }
  double base(std::int64_t n) const {
    const std::int64_t nc = changeIndex();
    if (n < nc) return t0 + static_cast<double>(n) * period;
    return t0 + static_cast<double>(nc) * period + static_cast<double>(n - nc) * period_after;
  }
  double trueNs(std::int64_t n) const {
    const double b = base(n);
    return b >= step_at ? b + step : b;
  }
  double recordNs(std::int64_t n) const {
    const double u = unit(seed * 7919, n);
    const double core = 1 - tail_share;
    const double j = u < core ? -jitter_core + 2 * jitter_core * (u / core)
                              : jitter_core + (jitter_tail - jitter_core) * ((u - core) / tail_share);
    return trueNs(n) + j;
  }
  std::int64_t firstAtOrAfter(double t) const {
    auto n = static_cast<std::int64_t>(std::floor((t - t0) / period));
    n = std::max<std::int64_t>(n - 3, 0);
    while (trueNs(n) < t) ++n;
    return n;
  }
  std::int64_t nearest(double t) const {
    const std::int64_t n = firstAtOrAfter(t);
    return (n > 0 && t - trueNs(n - 1) < trueNs(n) - t) ? n - 1 : n;
  }
};

// The probe and the bus: when a reply is ready, its receive delay, the write.
struct Probe {
  double readiness = 650000.0, readiness_spread = 20000.0;
  double full_extra = 110000.0;  // the full-window read, every 100th cycle
  double late_fraction = 0.02;   // replies delayed by 1 to 3 whole periods
  double tail_fraction = 0.04;   // receive delays of 13 to 60 us
  double missing_fraction = 0.0;
  double wake_late_lo = 1000.0, wake_late_hi = 5000.0;
  double wake_to_read = 8000.0, before_send = 4000.0, write = 5000.0;
  std::uint64_t seed = 2;
};

struct CycleOut {
  std::int64_t stamp;
  std::int64_t end;
  std::int64_t carrying;  // the true completion index that carried the reply
};

CycleOut runCycle(const Lattice& lat, const Probe& pr, std::int64_t k, std::int64_t wake) {
  const double u1 = unit(pr.seed, 4 * k), u2 = unit(pr.seed, 4 * k + 1),
               u3 = unit(pr.seed, 4 * k + 2), u4 = unit(pr.seed, 4 * k + 3);
  const double woke = static_cast<double>(wake) + pr.wake_late_lo + (pr.wake_late_hi - pr.wake_late_lo) * u1;
  const double read_start = woke + pr.wake_to_read;
  if (u4 < pr.missing_fraction) {
    return {ad::kInvalidStamp, static_cast<std::int64_t>(read_start + 1.5e6), -1};
  }
  double ready = read_start + pr.before_send + pr.readiness + pr.readiness_spread * (u2 - 0.5);
  if (k % 100 == 0) ready += pr.full_extra;
  const std::int64_t n = lat.firstAtOrAfter(ready);
  double delay = 9000.0 + 4000.0 * u3;
  if (u4 < pr.late_fraction) delay += kP * static_cast<double>(1 + static_cast<int>(u3 * 3));
  else if (u4 < pr.late_fraction + pr.tail_fraction) delay = 13000.0 + 47000.0 * u3;
  const double stamp = lat.recordNs(n) + delay;
  return {static_cast<std::int64_t>(stamp), static_cast<std::int64_t>(stamp + pr.write), n};
}

struct Run {
  std::vector<double> err_true, err_record;  // locked cycles after settling
  ad::SkipCounts whole, acquisition, settling, locked;
  std::uint64_t locks = 0, losses = 0;
  std::uint64_t past_wakes = 0, backward_targets = 0;
  std::uint64_t locked_window_cycles = 0;
  double locked_window_ns = 0.0;
};

// The closed loop: plans from ReferencedSchedule, cycles from the simulation.
Run closedLoop(const Lattice& lat, const Probe& pr, ad::ScheduleConfig cfg,
               double duration_ns = 20e9, double settle_after_lock_ns = 1e9) {
  Run r;
  const std::int64_t epoch = 0;
  ad::ReferencedSchedule sched(cfg, epoch);
  ad::WakePlan plan = sched.first();
  std::int64_t last_target = std::numeric_limits<std::int64_t>::min();
  double locked_since = std::numeric_limits<double>::infinity();
  const double locked_from = static_cast<double>(cfg.windows.acquisition_ns + cfg.windows.settling_ns);
  for (std::int64_t k = 0; static_cast<double>(plan.wake_ns) < duration_ns; ++k) {
    if (plan.referenced) {
      if (plan.target_index <= last_target) ++r.backward_targets;
      last_target = plan.target_index;
      if (!std::isfinite(locked_since)) locked_since = static_cast<double>(plan.wake_ns);
      if (static_cast<double>(plan.wake_ns) >= locked_since + settle_after_lock_ns) {
        const double pred = sched.estimator().completionNs(plan.target_index);
        const std::int64_t n = lat.nearest(pred);
        r.err_true.push_back(pred - lat.trueNs(n));
        r.err_record.push_back(pred - lat.recordNs(n));
      }
    } else {
      last_target = std::numeric_limits<std::int64_t>::min();
      locked_since = std::numeric_limits<double>::infinity();
    }
    if (static_cast<double>(plan.wake_ns) >= locked_from) ++r.locked_window_cycles;
    const CycleOut c = runCycle(lat, pr, k, plan.wake_ns);
    const std::int64_t now = c.end;
    plan = sched.closeAndPublish(c.stamp, c.end, now);
    if (plan.wake_ns < now) ++r.past_wakes;
  }
  r.locked_window_ns = duration_ns - locked_from;
  r.whole = sched.accounting().wholeRun();
  r.acquisition = sched.accounting().window(ad::Window::kAcquisition);
  r.settling = sched.accounting().window(ad::Window::kSettling);
  r.locked = sched.accounting().window(ad::Window::kLocked);
  r.locks = sched.estimator().counts().locks;
  r.losses = sched.estimator().counts().losses;
  return r;
}

double quantile(std::vector<double> v, double p) {
  if (v.empty()) return std::numeric_limits<double>::quiet_NaN();
  std::sort(v.begin(), v.end());
  const double pos = p * static_cast<double>(v.size() - 1);
  const auto i = static_cast<std::size_t>(pos);
  const double f = pos - static_cast<double>(i);
  return i + 1 < v.size() ? v[i] * (1 - f) + v[i + 1] * f : v[i];
}
double spread(const std::vector<double>& v, double lo = 0.001, double hi = 0.999) {
  return quantile(v, hi) - quantile(v, lo);
}

// Stamps on the host grid (cycles of 1 or 2 slots) for the fit tests.
std::vector<std::int64_t> gridStamps(const Lattice& lat, const Probe& pr, double until_ns) {
  std::vector<std::int64_t> s;
  std::int64_t wake = 0;
  for (std::int64_t k = 0; static_cast<double>(wake) < until_ns; ++k) {
    const CycleOut c = runCycle(lat, pr, k, wake);
    if (c.stamp >= 0) s.push_back(c.stamp);
    const auto d = static_cast<std::int64_t>(kP);
    wake = c.end <= wake + d ? wake + d : wake + d * (2 + (c.end - wake - d) / d);
  }
  std::sort(s.begin(), s.end());
  return s;
}

ad::ScheduleConfig defaultConfig() {
  ad::ScheduleConfig cfg;
  cfg.planner.lead_ns = 780000;
  cfg.planner.wake_to_read_ns = 10000;
  return cfg;
}

}  // namespace

TEST_CASE("the fit's slope grids are built as numpy.arange builds them",
          "[hw][adapter]") {
  const auto coarse = ad::detail::arange(-80.0, 20.0, 0.5);
  CHECK(coarse.size() == 200);
  CHECK(coarse.front() == -80.0);
  CHECK(coarse.back() == 19.5);
  CHECK(ad::detail::arange(-200.0, 200.0, 0.5).size() == 800);
  // numpy 2.3.5: np.arange(s0 - 0.5, s0 + 0.5, 0.01), its length and values
  struct Ref { double s0; std::size_t n; double v1, last, v57; };
  const Ref refs[] = {
      {-28.0, 100, -28.49, -27.509999999999845, -27.92999999999991},
      {-57.5, 100, -57.99, -57.0100000000002, -57.43000000000011},
      {-27.81, 100, -28.299999999999997, -27.319999999999844, -27.73999999999991},
      {-40.41, 100, -40.9, -39.92000000000019, -40.34000000000011},
      {3.5, 100, 3.01, 3.989999999999979, 3.569999999999988},
  };
  for (const Ref& r : refs) {
    const auto f = ad::detail::arange(r.s0 - 0.5, r.s0 + 0.5, 0.01);
    REQUIRE(f.size() == r.n);
    CHECK(f[1] == r.v1);
    CHECK(f.back() == r.last);
    CHECK(f[57] == r.v57);
  }
  CHECK(ad::detail::pymod(-1e-13, 1000.0) == 1000.0 - 1e-13);
  CHECK(ad::detail::pymod(1500.0, 1000.0) == 500.0);
}

TEST_CASE("the lattice fit recovers a known line under delayed and jittered stamps",
          "[hw][adapter]") {
  for (const double ppm : {-57.5, -27.8, -40.3, 12.0}) {
    Lattice lat;
    lat.period = kP * (1 + ppm * 1e-6);
    lat.t0 = 412000.0;
    Probe pr;
    const auto stamps = gridStamps(lat, pr, 2e9);
    ad::LatticeFitConfig cfg;  // the default grid, +-200 ppm
    const ad::LatticeFit fit = ad::fitLattice(stamps.data(), stamps.size(), cfg);
    INFO("ppm " << ppm << " fit slope " << fit.slope_ppm << " phase " << fit.phase0_us
                << " share " << fit.share);
    // 2 s of stamps resolve the slope to about 1 to 2 ppm (1 ppm moves the
    // line 2 us over them, against a 20 us window); the tracker absorbs that
    CHECK(std::fabs(fit.slope_ppm - ppm) < 3.0);
    // what locking uses: the line's position at the acquisition's end opens
    // near the stamps' floor above the true lattice there (the record's
    // jitter from -3 us, a delay from 9 us)
    const double at = 2e9;
    const double line_us = ad::detail::pymod(fit.phase0_us + fit.slope_ppm * at / 1e9, 1000.0);
    const double true_us = ad::detail::pymod(lat.trueNs(lat.firstAtOrAfter(at)), kP) / 1e3;
    const double d = ad::detail::pymod(line_us - true_us + 500.0, 1000.0) - 500.0;
    CHECK(d > -10.0);
    CHECK(d < 25.0);
    CHECK(fit.share > 0.75);
  }
}

TEST_CASE("locking needs enough stamps, a slope within the limit and a share",
          "[hw][adapter]") {
  Lattice lat;
  Probe pr;
  const auto good = gridStamps(lat, pr, 2e9);
  SECTION("a good acquisition locks, the anchor the first lattice point at or after its end") {
    ad::AdapterPhaseEstimator est;
    for (auto s : good) est.addAcquisitionStamp(s);
    REQUIRE(est.finishAcquisition(2000000000));
    CHECK(est.state() == ad::EstimatorState::kLocked);
    CHECK(est.anchorNs() >= 2e9);
    CHECK(est.anchorNs() < 2e9 + kP);
    CHECK(std::fabs(est.periodNs() - lat.period) < 3.0);  // within 3 ppm (the 2 s fit)
    CHECK(est.counts().locks == 1);
  }
  SECTION("too few stamps stay acquiring") {
    ad::AdapterPhaseEstimator est;
    for (std::size_t i = 0; i < 100; ++i) est.addAcquisitionStamp(good[i]);
    CHECK_FALSE(est.finishAcquisition(2000000000));
    CHECK(est.state() == ad::EstimatorState::kAcquiring);
    CHECK(est.counts().failed_acquisitions == 1);
    CHECK(est.acquisitionStamps() == 0);
  }
  SECTION("stamps with no lattice stay acquiring") {
    ad::AdapterPhaseEstimator est;
    std::mt19937_64 rng(3);
    std::uniform_int_distribution<std::int64_t> u(0, 2000000000);
    std::vector<std::int64_t> noise(1500);
    for (auto& x : noise) x = u(rng);
    std::sort(noise.begin(), noise.end());
    for (auto s : noise) est.addAcquisitionStamp(s);
    CHECK_FALSE(est.finishAcquisition(2000000000));
    CHECK(est.lastFit().share < 0.75);
  }
  SECTION("a slope beyond the limit stays acquiring") {
    ad::EstimatorConfig cfg;
    cfg.lock_slope_limit_ppm = 30.0;
    ad::AdapterPhaseEstimator est(cfg);
    for (auto s : good) est.addAcquisitionStamp(s);
    CHECK_FALSE(est.finishAcquisition(2000000000));
  }
  SECTION("the acquisition keeps at most its capacity, without allocating more") {
    ad::EstimatorConfig cfg;
    cfg.acquisition_capacity = 500;
    ad::AdapterPhaseEstimator est(cfg);
    REQUIRE(good.size() > 500);
    std::size_t dropped = 0;
    for (auto s : good) dropped += est.addAcquisitionStamp(s) == ad::Observation::kDropped;
    CHECK(est.acquisitionStamps() == 500);
    CHECK(dropped == good.size() - 500);
  }
}

TEST_CASE("a reply delayed by whole periods is late for its intended completion",
          "[hw][adapter]") {
  ad::EstimatorConfig cfg;
  cfg.supervise = false;
  ad::AdapterPhaseEstimator intended(cfg), nearest(cfg);
  intended.lockFrom(0.0, 100.0, 0);  // the true lattice: 100 us + k ms, period 1 ms
  nearest.lockFrom(0.0, 100.0, 0);
  const double c0 = intended.completionNs(5);
  for (std::int64_t k = 1; k <= 300; ++k) {
    const std::int64_t target = k * 3;
    // carried by its intended completion, delivered 960 us after it: about
    // a period, the case the nearest mapping takes for the next completion
    const std::int64_t stamp = 100000 + target * 1000000 + 960000;
    CHECK(intended.observeIntended(stamp, target) == ad::Observation::kLate);
    nearest.observeNearest(stamp);
  }
  // the intended mapping moved nothing; the nearest one pulled its lattice early
  CHECK(std::fabs(intended.completionNs(5) - c0) < 1e-6);
  CHECK(nearest.completionNs(5) - c0 < -40000.0);
  SECTION("1, 2 and 3 whole periods late are late, an on-time reply is gated") {
    ad::AdapterPhaseEstimator est(cfg);
    est.lockFrom(0.0, 100.0, 0);
    std::int64_t target = 10;
    for (int periods : {1, 2, 3}) {
      const auto stamp = static_cast<std::int64_t>(est.completionNs(target) + periods * kP + 10000.0);
      CHECK(est.observeIntended(stamp, target) == ad::Observation::kLate);
      target += 5;
    }
    const auto on_time = static_cast<std::int64_t>(est.completionNs(target) + 10000.0);
    CHECK(est.observeIntended(on_time, target) == ad::Observation::kGated);
  }
}

TEST_CASE("invalid samples change nothing", "[hw][adapter]") {
  ad::EstimatorConfig cfg;
  cfg.supervise = false;
  ad::AdapterPhaseEstimator est(cfg);
  CHECK(est.observeIntended(5000000, 1) == ad::Observation::kInvalid);  // not locked
  est.lockFrom(-30.0, 250.0, 0);
  const auto stamp = static_cast<std::int64_t>(est.completionNs(20) + 10000.0);
  REQUIRE(est.observeIntended(stamp, 20) == ad::Observation::kGated);
  const double anchor = est.anchorNs(), period = est.periodNs();
  const std::int64_t index = est.anchorIndex();
  CHECK(est.observeIntended(ad::kInvalidStamp, 21) == ad::Observation::kInvalid);
  CHECK(est.observeIntended(stamp - 1, 21) == ad::Observation::kInvalid);  // out of order
  CHECK(est.observeIntended(stamp + 1000000, 19) == ad::Observation::kInvalid);  // behind the anchor
  CHECK(est.observeNearest(-5) == ad::Observation::kInvalid);
  CHECK(est.addAcquisitionStamp(stamp + 2000000) == ad::Observation::kInvalid);  // locked
  CHECK(est.anchorNs() == anchor);
  CHECK(est.periodNs() == period);
  CHECK(est.anchorIndex() == index);
  CHECK(est.counts().invalid == 6);
}

TEST_CASE("the grid schedule is hw::advanceCycleDeadline's", "[hw][adapter]") {
  ad::AdapterPhaseEstimator acquiring;  // never locked: the planner stays on the grid
  ad::PlannerConfig cfg;
  ad::WakePlanner planner(cfg, 987654321);  // a nonzero epoch
  ad::WakePlan plan = planner.first();
  std::mt19937_64 rng(11);
  std::uniform_int_distribution<int> kind(0, 9);
  std::uniform_int_distribution<std::int64_t> early(1, 999999), late(1, 4999999);
  using namespace std::chrono;
  const steady_clock::time_point origin{};
  for (int i = 0; i < 20000; ++i) {
    const std::int64_t deadline = plan.wake_ns + 1000000;
    std::int64_t end = 0;
    switch (kind(rng)) {
      case 0: end = deadline; break;              // exactly on the deadline
      case 1: end = deadline + 1000000; break;    // exactly on a later boundary
      case 2: end = deadline + 1; break;
      case 3: case 4: end = deadline + late(rng); break;
      default: end = plan.wake_ns + early(rng); break;
    }
    const auto want = rtctrl::hw::advanceCycleDeadline(
        origin + nanoseconds(deadline), origin + nanoseconds(end), nanoseconds(1000000));
    plan = planner.next(acquiring, end, end);
    REQUIRE(plan.wake_ns == duration_cast<nanoseconds>(want.wake_time - origin).count());
    REQUIRE(plan.skipped == want.skipped_periods);
    CHECK_FALSE(plan.referenced);
  }
}

TEST_CASE("no wake is planned in the past, and targets only move forward",
          "[hw][adapter][safety]") {
  ad::EstimatorConfig ecfg;
  ecfg.supervise = false;
  ad::AdapterPhaseEstimator locked(ecfg), acquiring(ecfg);
  locked.lockFrom(-57.7, 321.0, 0);  // a fractional period, 999,942.3 ns
  ad::PlannerConfig cfg;
  cfg.min_gap_ns = 2000;
  ad::WakePlanner planner(cfg, 1234567);  // a nonzero epoch
  ad::WakePlan plan = planner.first();
  std::mt19937_64 rng(5);
  std::uniform_int_distribution<int> mode(0, 19), kind(0, 9);
  std::uniform_int_distribution<std::int64_t> work(600000, 999999), over(1, 7000000), gap(0, 300000);
  bool use_locked = false;
  std::int64_t last_target = std::numeric_limits<std::int64_t>::min();
  for (int i = 0; i < 50000; ++i) {
    if (mode(rng) == 0) use_locked = !use_locked;  // lock and fall back now and then
    const std::int64_t end = plan.wake_ns + (kind(rng) < 7 ? work(rng) : 1000000 + over(rng));
    const std::int64_t now = end + gap(rng);
    const ad::WakePlan prev = plan;
    plan = planner.next(use_locked ? locked : acquiring, end, now);
    REQUIRE(plan.wake_ns >= now + cfg.min_gap_ns);
    REQUIRE(plan.wake_ns >= end);
    REQUIRE(plan.wake_ns > prev.wake_ns);
    REQUIRE(plan.referenced == use_locked);
    REQUIRE(plan.transition == (prev.referenced != plan.referenced));
    if (plan.referenced) {
      if (prev.referenced) REQUIRE(plan.target_index > last_target);
      last_target = plan.target_index;
      // the wake is its target's lattice point less the lead and the allowance
      const double c = locked.completionNs(plan.target_index);
      CHECK(std::fabs(static_cast<double>(plan.wake_ns) - (c - 790000.0)) <= 1.0);
    }
    // skipped, recounted from the definition, slot by slot: the outgoing
    // schedule's slots after the last plan whose wakes come before the new
    // wake (a lattice slot's wake: its completion less the lead and the
    // allowance, rounded up, on the lattice that plan used; this test's
    // locked estimator never moves)
    std::uint64_t count = 0;
    if (!prev.referenced) {
      for (std::int64_t w = prev.wake_ns + 1000000; w < plan.wake_ns; w += 1000000) ++count;
    } else {
      for (std::int64_t n = prev.target_index + 1;
           static_cast<std::int64_t>(std::ceil(locked.completionNs(n) - 790000.0)) < plan.wake_ns; ++n) {
        ++count;
      }
    }
    INFO("prev " << (prev.referenced ? "lattice" : "grid") << " wake " << prev.wake_ns << " target "
                 << prev.target_index << "; next " << (plan.referenced ? "lattice" : "grid") << " wake "
                 << plan.wake_ns << " target " << plan.target_index << "; end " << end << " now " << now);
    REQUIRE(plan.skipped == count);
  }
}

TEST_CASE("skips are counted per declared window and for the whole run, apart",
          "[hw][adapter]") {
  ad::WindowBounds b;  // [0, 2 s), [2, 3 s), then locked
  ad::SkipAccounting acc(1000, b);
  CHECK(acc.windowOf(1000) == ad::Window::kAcquisition);
  CHECK(acc.windowOf(1000 + 1999999999) == ad::Window::kAcquisition);
  CHECK(acc.windowOf(1000 + 2000000000) == ad::Window::kSettling);
  CHECK(acc.windowOf(1000 + 3000000000) == ad::Window::kLocked);
  // acquisition: 4 cycles, one skip of 1
  acc.add(1000, 0, false);
  acc.add(1000 + 1000000, 1, false);
  acc.add(1000 + 3000000, 0, false);
  acc.add(1000 + 1999000000, 1, false);  // its neighbour across the boundary also overruns
  // settling: 2 cycles
  acc.add(1000 + 2000000000, 1, true);
  acc.add(1000 + 2002000000, 0, true);
  // locked: 3 cycles, one skip of 2
  acc.add(1000 + 3000000000, 0, true);
  acc.add(1000 + 3001000000, 2, true);
  acc.add(1000 + 3004000000, 0, true);
  const auto& a = acc.window(ad::Window::kAcquisition);
  const auto& s = acc.window(ad::Window::kSettling);
  const auto& l = acc.window(ad::Window::kLocked);
  const auto& w = acc.wholeRun();
  CHECK(a.cycles == 4);
  CHECK(a.skipped == 2);
  CHECK(a.consecutive == 0);
  CHECK(a.grid_cycles == 4);
  CHECK(s.cycles == 2);
  CHECK(s.skipped == 1);
  CHECK(s.consecutive == 0);  // the adjacent pair straddles the boundary
  CHECK(l.cycles == 3);
  CHECK(l.skipped == 2);
  CHECK(l.consecutive == 1);  // a skip of two slots
  CHECK(w.cycles == 9);
  CHECK(w.skipped == 5);
  CHECK(w.overruns == 4);
  CHECK(w.consecutive == 2);  // the cross-boundary pair and the skip of two
  CHECK(w.referenced_cycles == 5);
  CHECK(a.cycles + s.cycles + l.cycles == w.cycles);
  CHECK(a.skipped + s.skipped + l.skipped == w.skipped);
  // T2 takes the whole run
  const auto t2 = ad::t2Loaded(w);
  CHECK(t2.allowed == 9.0 / 1e4);
  CHECK_FALSE(t2.pass);
  ad::SkipCounts clean;
  clean.cycles = 20000;
  clean.skipped = 2;
  CHECK(ad::t2Loaded(clean).pass);
  clean.consecutive = 1;
  CHECK_FALSE(ad::t2Loaded(clean).pass);
}

TEST_CASE("synthetic ground truth separates estimator error from timestamp jitter",
          "[hw][adapter]") {
  Lattice lat;  // the host records each completion with jitter (Lattice::recordNs)
  Probe pr;
  const Run r = closedLoop(lat, pr, defaultConfig());
  REQUIRE(r.locks == 1);
  CHECK(r.losses == 0);
  CHECK(r.past_wakes == 0);
  CHECK(r.backward_targets == 0);
  REQUIRE(r.err_true.size() > 10000);
  const double bias = quantile(r.err_true, 0.5);
  const double s_true = spread(r.err_true);
  const double s_record = spread(r.err_record);
  INFO("bias " << bias << " ns; spread against the truth " << s_true
               << " ns, against the record " << s_record << " ns");
  // the estimate's own spread is a few us; against the record it carries the
  // record's jitter (about 33 us from p0.1 to p99.9)
  CHECK(s_true < 8000.0);
  CHECK(s_record > 25000.0);
  CHECK(s_record > 3.0 * s_true);
}

TEST_CASE("the tracker follows drift and a change of the host clock's rate",
          "[hw][adapter]") {
  for (const double ppm : {-200.0, -57.5, 0.0, 150.0}) {
    Lattice lat;
    lat.period = kP * (1 + ppm * 1e-6);
    lat.change_at = 10e9;  // a change like the 12.6 ppm one between attempt 26's runs
    lat.period_after = lat.period * (1 - 12.6e-6);
    Probe pr;
    const Run r = closedLoop(lat, pr, defaultConfig());
    INFO("ppm " << ppm);
    CHECK(r.locks == 1);
    CHECK(r.losses == 0);
    CHECK(r.past_wakes == 0);
    CHECK(spread(r.err_true) < 12000.0);
  }
}

TEST_CASE("a phase step loses the lock, falls back to the grid and reacquires",
          "[hw][adapter][safety]") {
  Lattice lat;
  lat.step_at = 10e9;
  lat.step = 300000.0;  // the adapter's completions move 300 us
  Probe pr;
  const Run r = closedLoop(lat, pr, defaultConfig());
  CHECK(r.locks == 2);
  CHECK(r.losses == 1);
  CHECK(r.past_wakes == 0);
  CHECK(r.backward_targets == 0);
  CHECK(r.locked.grid_cycles > 0);        // the fallback, inside the locked window
  CHECK(r.locked.referenced_cycles > 0);  // and the relock
  CHECK(r.whole.cycles == r.acquisition.cycles + r.settling.cycles + r.locked.cycles);
  // after relocking, the estimate follows the stepped lattice again
  CHECK(std::fabs(quantile(r.err_true, 0.99) - quantile(r.err_true, 0.5)) < 10000.0);
}

TEST_CASE("the period clamp is a loss of lock", "[hw][adapter]") {
  Lattice lat;
  lat.period = kP * (1 - 57.5e-6);
  Probe pr;
  ad::ScheduleConfig cfg = defaultConfig();
  cfg.estimator.period_clamp_ppm = 20.0;  // tighter than the drift
  cfg.estimator.lock_slope_limit_ppm = 200.0;
  ad::ReferencedSchedule sched(cfg, 0);
  ad::WakePlan plan = sched.first();
  bool clamped = false;
  for (std::int64_t k = 0; plan.wake_ns < 20000000000LL && !clamped; ++k) {
    const CycleOut c = runCycle(lat, pr, k, plan.wake_ns);
    plan = sched.closeAndPublish(c.stamp, c.end, c.end);
    clamped = sched.estimator().counts().losses > 0 &&
              sched.estimator().lossCause() == ad::LossCause::kPeriodClamp;
  }
  CHECK(clamped);
}

TEST_CASE("missing replies end the lock after the ungated limit", "[hw][adapter][safety]") {
  ad::EstimatorConfig cfg;
  ad::AdapterPhaseEstimator est(cfg);
  est.lockFrom(0.0, 100.0, 0);
  for (int k = 1; k < cfg.loss_max_ungated_cycles; ++k) {
    est.observeIntended(ad::kInvalidStamp, k);
    REQUIRE(est.state() == ad::EstimatorState::kLocked);
  }
  est.observeIntended(ad::kInvalidStamp, cfg.loss_max_ungated_cycles);
  CHECK(est.state() == ad::EstimatorState::kAcquiring);
  CHECK(est.lossCause() == ad::LossCause::kNoGatedStamp);
}

TEST_CASE("once locked, the start phase no longer decides the rate", "[hw][adapter]") {
  // The same adapter and probe, the lattice starting at ten phases of the
  // host grid. On the grid alone (never locking) the rate depends on the
  // phase; referenced, the locked window's rate does not.
  std::vector<double> grid_rates, locked_rates;
  for (int i = 0; i < 10; ++i) {
    Lattice lat;
    lat.period = kP * (1 - 27.8e-6);  // attempt 26's confined timer run: 556 us swept in 20 s
    lat.t0 = 100000.0 * i + 7000.0;
    Probe pr;
    ad::ScheduleConfig grid = defaultConfig();
    grid.estimator.lock_min_share = 2.0;  // never locks
    const Run g = closedLoop(lat, pr, grid);
    grid_rates.push_back(static_cast<double>(g.whole.cycles) / 20.0);
    const Run r = closedLoop(lat, pr, defaultConfig());
    REQUIRE(r.locks == 1);
    locked_rates.push_back(static_cast<double>(r.locked_window_cycles) / (r.locked_window_ns / 1e9));
  }
  const auto [gmin, gmax] = std::minmax_element(grid_rates.begin(), grid_rates.end());
  const auto [lmin, lmax] = std::minmax_element(locked_rates.begin(), locked_rates.end());
  INFO("grid " << *gmin << " to " << *gmax << " Hz; locked window " << *lmin << " to " << *lmax << " Hz");
  CHECK(*gmax - *gmin > 100.0);
  CHECK(*lmax - *lmin < 10.0);
  CHECK(*lmin > 950.0);  // the replies delayed by whole periods still cost their slots
}

TEST_CASE("a plan depends only on what was observed before it", "[hw][adapter][safety]") {
  // Two runs fed the same cycles up to K and different ones after: every plan
  // made up to K is the same in both.
  Lattice lat;
  Probe a, b;
  b.seed = 99;  // a different future
  const std::int64_t K = 6000;
  ad::ReferencedSchedule sa(defaultConfig(), 0), sb(defaultConfig(), 0);
  ad::WakePlan pa = sa.first(), pb = sb.first();
  std::int64_t compared = 0;
  for (std::int64_t k = 0; k < 9000; ++k) {
    const CycleOut ca = runCycle(lat, a, k, pa.wake_ns);
    const CycleOut cb = k < K ? ca : runCycle(lat, b, k, pb.wake_ns);
    pa = sa.closeAndPublish(ca.stamp, ca.end, ca.end);
    pb = sb.closeAndPublish(cb.stamp, cb.end, cb.end);
    if (k < K) {
      REQUIRE(pa.wake_ns == pb.wake_ns);
      REQUIRE(pa.target_index == pb.target_index);
      REQUIRE(pa.skipped == pb.skipped);
      ++compared;
    }
  }
  CHECK(compared == K);
  CHECK(sa.estimator().state() == ad::EstimatorState::kLocked);
}

namespace {
// A lattice slot's wake, from the definition: its completion on the
// estimator's lattice less the lead and the allowance, rounded up.
std::int64_t slotWake(const ad::AdapterPhaseEstimator& est, std::int64_t n, const ad::PlannerConfig& c) {
  return static_cast<std::int64_t>(
      std::ceil(est.completionNs(n) - static_cast<double>(c.lead_ns + c.wake_to_read_ns)));
}
}  // namespace

TEST_CASE("the review's case: a fallback wake on the next lattice slot skips nothing",
          "[hw][adapter][safety]") {
  // the reviewer's reproducer (review-notes/2026-11-20-adapter-timing-prototype-review)
  ad::EstimatorConfig ec;
  ec.supervise = false;
  ad::AdapterPhaseEstimator locked(ec), acquiring(ec);
  locked.lockFrom(-57.7, 100.0, 0);
  ad::PlannerConfig pc;
  ad::WakePlanner planner(pc, 2309770);
  planner.first();
  const ad::WakePlan before = planner.next(locked, 2309771, 2309771);
  const std::int64_t next_wake = slotWake(locked, before.target_index + 1, pc);
  const ad::WakePlan after = planner.next(acquiring, next_wake, next_wake);
  CHECK(std::fabs(locked.periodNs() - 999942.3) < 1e-6);
  CHECK(next_wake == 3309770);
  CHECK(after.wake_ns == next_wake);
  CHECK(after.skipped == 0);
}

TEST_CASE("schedule changes and late cycles at a slot's rounded wake and +-1 ns",
          "[hw][adapter][safety]") {
  // Independent boundary checks: fractional periods, nonzero epochs, and each
  // incoming wake placed at an outgoing slot's rounded wake, 1 ns before it
  // and 1 ns after it. A slot whose wake equals the new wake was used.
  const ad::PlannerConfig pc;  // lead 780 us, allowance 10 us, no gap
  const std::int64_t P = pc.grid_period_ns;
  ad::EstimatorConfig ec;
  ec.supervise = false;
  int checked = 0;
  for (const double slope : {-57.7, -27.83, 13.37}) {
    for (const double phase : {100.0, 433.3, 987.6}) {
      ad::AdapterPhaseEstimator locked(ec), acquiring(ec);
      locked.lockFrom(slope, phase, 0);
      REQUIRE(locked.periodNs() != std::floor(locked.periodNs()));  // fractional
      for (const std::int64_t delta : {std::int64_t{-1}, std::int64_t{0}, std::int64_t{1}}) {
        const std::int64_t n = 3000;  // a lattice slot about 3 s in
        const std::int64_t w_n = slotWake(locked, n, pc);
        const std::int64_t w_next = slotWake(locked, n + 1, pc);
        INFO("slope " << slope << " phase " << phase << " delta " << delta << " w_n " << w_n
                      << " w_next " << w_next);
        {
          // Falling back: the lattice plan targets n; the grid's first slot
          // after the cycle's end is w_next + delta. Only delta = +1 passes
          // the outgoing slot n + 1 without a cycle.
          const std::int64_t grid_wake = w_next + delta;
          const std::int64_t epoch = grid_wake - 5 * P;  // nonzero, on the grid wake
          ad::WakePlanner planner(pc, epoch);
          planner.first();
          const ad::WakePlan lat = planner.next(locked, w_n - 300, w_n - 300);
          REQUIRE(lat.referenced);
          REQUIRE(lat.target_index == n);
          REQUIRE(lat.wake_ns == w_n);
          const std::int64_t end = grid_wake - 400;  // the cycle ends before the grid wake
          const ad::WakePlan fb = planner.next(acquiring, end, end);
          REQUIRE_FALSE(fb.referenced);
          REQUIRE(fb.wake_ns == grid_wake);
          CHECK(fb.skipped == (delta > 0 ? 1U : 0U));
          ++checked;
        }
        {
          // Locking: the grid's next slot is w_n + delta and the lattice plan
          // wakes at w_n. Only delta = -1 passes that grid slot.
          const std::int64_t epoch = w_n + delta - P;  // the grid's slot 1 at w_n + delta
          ad::WakePlanner planner(pc, epoch);
          planner.first();
          const ad::WakePlan lat = planner.next(locked, w_n - 300, w_n - 300);
          REQUIRE(lat.referenced);
          REQUIRE(lat.target_index == n);
          REQUIRE(lat.wake_ns == w_n);
          CHECK(lat.skipped == (delta < 0 ? 1U : 0U));
          ++checked;
        }
        {
          // Within the lattice: the cycle targeting n ends at w_next + delta.
          // At or before w_next it is in time; 1 ns after, n + 1 is skipped.
          const std::int64_t epoch = 1 + 7 * P / 3;  // nonzero
          ad::WakePlanner planner(pc, epoch);
          planner.first();
          const ad::WakePlan lat = planner.next(locked, w_n - 300, w_n - 300);
          REQUIRE(lat.target_index == n);
          const std::int64_t end = w_next + delta;
          const ad::WakePlan nx = planner.next(locked, end, end);
          REQUIRE(nx.referenced);
          CHECK(nx.target_index == (delta > 0 ? n + 2 : n + 1));
          CHECK(nx.skipped == (delta > 0 ? 1U : 0U));
          CHECK(nx.wake_ns >= end);
          ++checked;
        }
      }
    }
  }
  CHECK(checked == 3 * 3 * 3 * 3);
}

// ------------------------------------------------------------ step 1 (the integration's pure logic)

static_assert(static_cast<int>(ad::Observation::kNone) == 0 && static_cast<int>(ad::Observation::kStored) == 1 &&
                  static_cast<int>(ad::Observation::kDropped) == 2 && static_cast<int>(ad::Observation::kGated) == 3 &&
                  static_cast<int>(ad::Observation::kLate) == 4 && static_cast<int>(ad::Observation::kEarly) == 5 &&
                  static_cast<int>(ad::Observation::kInvalid) == 6,
              "the log's observation encoding");
static_assert(static_cast<int>(ad::Schedule::kGrid) == 0 && static_cast<int>(ad::Schedule::kLattice) == 1,
              "the log's schedule encoding");
static_assert(static_cast<int>(ad::LockState::kAcquiring) == 0 && static_cast<int>(ad::LockState::kFitPending) == 1 &&
                  static_cast<int>(ad::LockState::kLocked) == 2,
              "the log's lock-state encoding");

TEST_CASE("the fit exchange hands each buffer to one owner at a time", "[hw][adapter]") {
  using Owner = ad::FitExchange::Owner;
  ad::FitExchange ex(100);
  CHECK(ex.owner(0) == Owner::kWorker);
  CHECK(ex.owner(1) == Owner::kWorker);
  CHECK_FALSE(ex.queue(1000, 1));  // nothing to queue
  for (int i = 0; i < 50; ++i) REQUIRE(ex.append(1000000 * i + 412000));
  CHECK(ex.workerBufferCount() == 50);
  REQUIRE(ex.queue(50000000, 1));
  CHECK(ex.owner(0) == Owner::kQueued);
  // the frozen buffer is not the worker's any more: stamps go to the other
  for (int i = 0; i < 10; ++i) REQUIRE(ex.append(60000000 + 1000000 * i + 412000));
  CHECK(ex.workerBufferCount() == 10);
  REQUIRE(ex.queue(70000000, 2));
  CHECK(ex.owner(1) == Owner::kQueued);
  CHECK_FALSE(ex.append(80000000));  // neither buffer is the worker's: dropped
  CHECK(ex.workerCounts().dropped == 1);
  ad::FitResult r;
  CHECK_FALSE(ex.take(&r));  // nothing completed
  REQUIRE(ex.service());     // the fitter claims and completes one
  CHECK(ex.owner(0) == Owner::kDone);
  CHECK(ex.owner(1) == Owner::kQueued);
  REQUIRE(ex.take(&r));
  CHECK(r.generation == 1);
  CHECK(r.at_ns == 50000000);
  CHECK(r.fit.stamps == 50);
  CHECK(r.fit.share == 1.0);
  CHECK(ex.owner(0) == Owner::kWorker);  // returned, cleared
  REQUIRE(ex.append(90000000));
  CHECK(ex.workerBufferCount() == 1);
  REQUIRE(ex.service());
  REQUIRE(ex.take(&r));
  CHECK(r.generation == 2);
  CHECK(r.fit.stamps == 10);
  CHECK_FALSE(ex.service());  // nothing queued
  ex.clearWorkerBuffer();
  CHECK(ex.workerBufferCount() == 0);
  CHECK(ex.fitted() == 2);
  CHECK(ex.workerCounts().taken == 2);
}

TEST_CASE("a fitter thread never reads a payload the worker is changing", "[hw][adapter]") {
  // Each generation's stamps sit on its own phase, a count of its own: a
  // result mixing two generations' stamps would fit a share below 1, and a
  // count changed under the fitter would not match.
  ad::LatticeFitConfig fc;
  fc.slope_min_ppm = -10.0;  // a short grid: quick fits
  fc.slope_max_ppm = 10.0;
  ad::FitExchange ex(600, fc);
  std::atomic<bool> stop{false};
  std::thread fitter([&] {
    while (!stop.load()) {
      if (!ex.service()) std::this_thread::yield();
    }
  });
  const int kGenerations = 300;
  auto count_of = [](int g) { return static_cast<std::size_t>(200 + (g * 37) % 300); };
  std::vector<int> seen(kGenerations + 1, 0);
  int bad = 0, received = 0;
  auto drain = [&] {
    ad::FitResult r;
    while (ex.take(&r)) {
      const int g = static_cast<int>(r.generation);
      ++seen[static_cast<std::size_t>(g)];
      ++received;
      if (r.fit.stamps != count_of(g) || r.fit.share != 1.0) ++bad;
    }
  };
  for (int g = 1; g <= kGenerations; ++g) {
    const std::int64_t phase = (g * 37) % 1000 * 1000;  // ns within the slot
    const std::size_t n = count_of(g);
    for (std::size_t k = 0; k < n; ++k) {
      while (!ex.append(static_cast<std::int64_t>(k) * 1000000 + phase + 1000000)) {
        drain();  // both buffers busy: take a result to free one
        std::this_thread::yield();
      }
    }
    while (!ex.queue(static_cast<std::int64_t>(n) * 1000000, static_cast<std::uint64_t>(g))) {
      drain();
      std::this_thread::yield();
    }
    drain();
  }
  while (received < kGenerations) {
    drain();
    std::this_thread::yield();
  }
  stop.store(true);
  fitter.join();
  CHECK(bad == 0);
  CHECK(received == kGenerations);
  CHECK(std::count(seen.begin() + 1, seen.end(), 1) == kGenerations);  // each exactly once
}

TEST_CASE("publication counts each cycle once and refuses calls out of turn", "[hw][adapter][safety]") {
  ad::ReferencedSchedule s(defaultConfig(), 1000);
  ad::WakePlan plan = s.first();
  CHECK(s.published());
  CHECK(plan.lock_state == ad::LockState::kAcquiring);
  CHECK(s.publish(5000).wake_ns == plan.wake_ns);  // nothing to publish
  CHECK(s.counts().protocol_errors == 1);
  const ad::WakePlan prov = s.closeCycle(ad::kInvalidStamp, 900000, 900000);
  CHECK_FALSE(s.published());
  CHECK(s.closeCycle(ad::kInvalidStamp, 950000, 950000).wake_ns == prov.wake_ns);  // out of turn
  CHECK(s.counts().protocol_errors == 2);
  CHECK(s.accounting().wholeRun().cycles == 0);  // nothing counted before publication
  plan = s.publish(900000);
  CHECK(s.accounting().wholeRun().cycles == 1);
  s.publish(900000);
  CHECK(s.accounting().wholeRun().cycles == 1);
  CHECK(s.counts().protocol_errors == 3);
  for (int i = 0; i < 10; ++i) {
    const std::int64_t end = plan.wake_ns + 600000;
    plan = s.closeAndPublish(ad::kInvalidStamp, end, end);
  }
  CHECK(s.accounting().wholeRun().cycles == 11);
  CHECK(s.counts().protocol_errors == 3);
}

TEST_CASE("the publication's recheck at a slot's rounded wake and +-1 ns", "[hw][adapter][safety]") {
  ad::PlannerConfig pc;
  pc.min_gap_ns = 20000;
  const std::int64_t gap = pc.min_gap_ns;
  ad::EstimatorConfig ec;
  ec.supervise = false;
  for (const double slope : {-57.7, -27.83, 13.37}) {
    ad::AdapterPhaseEstimator locked(ec), acquiring(ec);
    locked.lockFrom(slope, 433.3, 0);
    for (const std::int64_t delta : {std::int64_t{-1}, std::int64_t{0}, std::int64_t{1}}) {
      INFO("slope " << slope << " delta " << delta);
      {  // on the lattice
        ad::WakePlanner planner(pc, 1 + 7 * pc.grid_period_ns / 3);
        planner.first();
        const std::int64_t w_n = slotWake(locked, 3000, pc);
        const ad::WakePlan prov = planner.next(locked, w_n - 300000, w_n - 300000);
        REQUIRE(prov.target_index == 3000);
        const ad::WakePlan pub = planner.recheck(w_n + delta - gap);  // now + gap = w_n + delta
        if (delta <= 0) {
          CHECK(pub.wake_ns == w_n);
          CHECK(pub.target_index == 3000);
          CHECK(pub.recheck_skipped == 0);
          CHECK(pub.skipped == prov.skipped);
        } else {
          CHECK(pub.target_index == 3001);
          CHECK(pub.wake_ns == slotWake(locked, 3001, pc));
          CHECK(pub.recheck_skipped == 1);
          CHECK(pub.skipped == prov.skipped + 1);
          CHECK(pub.predicted_completion_ns == std::llround(locked.completionNs(3001)));
        }
      }
      {  // on the grid
        const std::int64_t epoch = 987654321;
        ad::WakePlanner planner(pc, epoch);
        planner.first();
        const ad::WakePlan prov = planner.next(acquiring, epoch + 400000, epoch + 400000);
        REQUIRE(prov.wake_ns == epoch + pc.grid_period_ns);
        const ad::WakePlan pub = planner.recheck(prov.wake_ns + delta - gap);
        CHECK(pub.wake_ns == (delta <= 0 ? prov.wake_ns : prov.wake_ns + pc.grid_period_ns));
        CHECK(pub.recheck_skipped == (delta > 0 ? 1U : 0U));
      }
    }
    {  // a scheduler delay of 2.5 periods is visible in recheck_skipped (each slope)
      ad::WakePlanner planner(pc, 5);
      planner.first();
      const std::int64_t w_n = slotWake(locked, 3000, pc);
      planner.next(locked, w_n - 300000, w_n - 300000);
      const ad::WakePlan pub = planner.recheck(w_n + 2500000 - gap);
      CHECK(pub.recheck_skipped == 3);
      CHECK(pub.target_index == 3003);
      CHECK(pub.wake_ns >= w_n + 2500000);
    }
  }
  SECTION("through the schedule, the recheck's skips are counted for the cycle that ended") {
    ad::ScheduleConfig cfg = defaultConfig();
    cfg.planner.min_gap_ns = gap;
    ad::ReferencedSchedule s(cfg, 0);
    s.first();
    const ad::WakePlan prov = s.closeCycle(ad::kInvalidStamp, 500000, 500000);
    REQUIRE(prov.wake_ns == 1000000);
    const ad::WakePlan pub = s.publish(1000000 - gap + 1);  // 1 ns too late for the slot
    CHECK(pub.wake_ns == 2000000);
    CHECK(pub.recheck_skipped == 1);
    CHECK(s.accounting().wholeRun().skipped == 1);
    CHECK(s.accounting().wholeRun().overruns == 1);
  }
}

namespace {

struct ExchangeRun {
  Run run;
  ad::ScheduleCounts counts;
  ad::FitExchange::Counts exchange;
  std::uint64_t pending_cycles = 0;
};

// The closed loop with a FitExchange: the fitter is serviced (on this thread,
// in simulated time) `fit_delay_ns` after each request, or never.
ExchangeRun closedLoopExchange(const Lattice& lat, const Probe& pr, ad::ScheduleConfig cfg, double fit_delay_ns,
                               bool never = false, double duration_ns = 20e9) {
  ExchangeRun er;
  ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
  ad::ReferencedSchedule sched(cfg, 0, &ex);
  ad::WakePlan plan = sched.first();
  std::vector<double> due;  // when each request's fit is serviced
  std::uint64_t requested = 0;
  std::int64_t last_target = std::numeric_limits<std::int64_t>::min();
  for (std::int64_t k = 0; static_cast<double>(plan.wake_ns) < duration_ns; ++k) {
    if (plan.referenced) {
      if (plan.target_index <= last_target) ++er.run.backward_targets;
      last_target = plan.target_index;
    } else {
      last_target = std::numeric_limits<std::int64_t>::min();
    }
    if (plan.lock_state == ad::LockState::kFitPending) ++er.pending_cycles;
    const CycleOut c = runCycle(lat, pr, k, plan.wake_ns);
    while (!due.empty() && static_cast<double>(c.end) >= due.front()) {
      ex.service();
      due.erase(due.begin());
    }
    plan = sched.closeAndPublish(c.stamp, c.end, c.end);
    if (plan.wake_ns < c.end) ++er.run.past_wakes;
    if (sched.counts().fits_requested > requested) {
      requested = sched.counts().fits_requested;
      if (!never) due.push_back(static_cast<double>(c.end) + fit_delay_ns);
    }
  }
  er.run.whole = sched.accounting().wholeRun();
  er.run.acquisition = sched.accounting().window(ad::Window::kAcquisition);
  er.run.settling = sched.accounting().window(ad::Window::kSettling);
  er.run.locked = sched.accounting().window(ad::Window::kLocked);
  er.run.locks = sched.estimator().counts().locks;
  er.run.losses = sched.estimator().counts().losses;
  er.counts = sched.counts();
  er.exchange = ex.workerCounts();
  return er;
}

}  // namespace

TEST_CASE("delayed, stale, abandoned and missing fits keep the grid running", "[hw][adapter][safety]") {
  Lattice lat;
  Probe pr;
  SECTION("a fit 20 ms late locks inside the settling window") {
    const ExchangeRun r = closedLoopExchange(lat, pr, defaultConfig(), 20e6);
    CHECK(r.counts.fits_requested == 1);
    CHECK(r.counts.fits_applied == 1);
    CHECK(r.run.locks == 1);
    CHECK(r.pending_cycles > 0);  // published as fit pending while it waited
    CHECK(r.run.past_wakes == 0);
    CHECK(r.run.locked.referenced_cycles > 0);
    CHECK(r.run.locked.grid_cycles == 0);
  }
  SECTION("a fit 600 ms late is abandoned and its late result discarded") {
    const ExchangeRun r = closedLoopExchange(lat, pr, defaultConfig(), 600e6);
    CHECK(r.run.locks == 0);
    CHECK(r.counts.abandoned_requests >= 5);
    CHECK(r.counts.old_generation_results >= 5);
    CHECK(r.counts.fits_applied == 0);
    CHECK(r.run.past_wakes == 0);
    CHECK(r.run.whole.referenced_cycles == 0);  // the grid throughout
    CHECK(r.run.whole.cycles == r.run.acquisition.cycles + r.run.settling.cycles + r.run.locked.cycles);
  }
  SECTION("a fitter that never answers leaves the worker on the grid, its buffers held and stamps dropped") {
    const ExchangeRun r = closedLoopExchange(lat, pr, defaultConfig(), 0.0, true);
    CHECK(r.run.locks == 0);
    CHECK(r.counts.fits_requested == 2);  // both buffers queued, never returned
    CHECK(r.counts.queue_failures > 0);
    CHECK(r.exchange.dropped > 0);
    CHECK(r.run.past_wakes == 0);
    CHECK(r.run.whole.referenced_cycles == 0);
  }
}

TEST_CASE("a fit is applied up to its age limit and refused 1 ns after", "[hw][adapter]") {
  for (const std::int64_t late : {std::int64_t{0}, std::int64_t{1}}) {
    Lattice lat;
    Probe pr;
    const ad::ScheduleConfig cfg = defaultConfig();
    ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
    ad::ReferencedSchedule s(cfg, 0, &ex);
    ad::WakePlan plan = s.first();
    std::int64_t k = 0;
    while (s.counts().fits_requested == 0) {
      const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
      plan = s.closeAndPublish(c.stamp, c.end, c.end);
    }
    REQUIRE(ex.service());
    // the next cycle ends exactly at the age limit, or 1 ns after it
    const std::int64_t at = cfg.windows.acquisition_ns;
    const std::int64_t end = at + cfg.fit_max_age_ns + late;
    plan = s.closeAndPublish(ad::kInvalidStamp, end, end);
    INFO("late " << late);
    CHECK(s.counts().fits_applied == (late == 0 ? 1U : 0U));
    CHECK(s.counts().stale_results == (late == 0 ? 0U : 1U));
    CHECK(plan.wake_ns >= end);
  }
}

TEST_CASE("each close reports its fit events once, in order, with the window and the time they concern",
          "[hw][adapter]") {
  const ad::ScheduleConfig cfg = defaultConfig();
  Lattice lat;
  Probe pr;
  pr.late_fraction = 0.0;  // no whole-period delays: the fit qualifies
  pr.tail_fraction = 0.0;
  std::vector<ad::FitEvent> events;
  std::size_t reports = 0;
  auto close = [&](ad::ReferencedSchedule& s, ad::WakePlan& plan, std::int64_t stamp, std::int64_t end) {
    plan = s.closeAndPublish(stamp, end, end);
    for (std::size_t i = 0; i < s.fitEventCount(); ++i) events.push_back(s.fitEvent(i));
    if (s.fitEventCount() > 0) ++reports;
    return s.fitEventCount();
  };
  {
    // A request at the window's end, the fitter's copy of its result, and
    // the result applied at the next close; nothing in between.
    ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
    ad::ReferencedSchedule s(cfg, 0, &ex);
    ad::WakePlan plan = s.first();
    std::int64_t k = 0;
    while (s.counts().fits_requested == 0) {
      const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
      close(s, plan, c.stamp, c.end);
    }
    REQUIRE(events.size() == 1);
    CHECK(reports == 1);
    const ad::FitEvent request = events[0];
    CHECK(request.outcome == ad::FitOutcome::kRequested);
    CHECK(request.generation == 1);
    CHECK(request.window_end_ns == cfg.windows.acquisition_ns);
    CHECK(request.at_ns >= request.window_end_ns);
    CHECK(request.at_ns < request.window_end_ns + 2000000);
    ad::FitResult done;
    REQUIRE(ex.service(&done));
    CHECK(done.generation == 1);
    CHECK(done.at_ns == cfg.windows.acquisition_ns);
    CHECK(done.fit.stamps == ex.workerCounts().appended);
    CHECK(done.fit.share >= cfg.estimator.lock_min_share);
    const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
    REQUIRE(close(s, plan, c.stamp, c.end) == 1);
    const ad::FitEvent applied = events[1];
    CHECK(applied.outcome == ad::FitOutcome::kApplied);
    CHECK(applied.generation == 1);
    CHECK(applied.window_end_ns == request.window_end_ns);
    CHECK(applied.at_ns == c.end);
    CHECK(applied.fit.slope_ppm == done.fit.slope_ppm);
    CHECK(applied.fit.phase0_us == done.fit.phase0_us);
    CHECK(applied.fit.share == done.fit.share);
    CHECK(applied.fit.stamps == done.fit.stamps);
    CHECK(s.estimator().state() == ad::EstimatorState::kLocked);
    for (int i = 0; i < 100; ++i) {
      const CycleOut d = runCycle(lat, pr, k++, plan.wake_ns);
      CHECK(close(s, plan, d.stamp, d.end) == 0);
    }
  }
  events.clear();
  {
    // No fitter: abandoned at the first close past the age limit; the late
    // result, when it comes, an old generation; with both buffers queued, a
    // queue failure.
    ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
    ad::ReferencedSchedule s(cfg, 0, &ex);
    ad::WakePlan plan = s.first();
    std::int64_t k = 0;
    while (s.counts().abandoned_requests == 0) {
      const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
      close(s, plan, c.stamp, c.end);
    }
    REQUIRE(events.size() == 2);
    CHECK(events[1].outcome == ad::FitOutcome::kAbandoned);
    CHECK(events[1].generation == 1);
    CHECK(events[1].window_end_ns == cfg.windows.acquisition_ns);
    CHECK(events[1].at_ns - events[1].window_end_ns > cfg.fit_max_age_ns);
    CHECK(events[1].at_ns - events[1].window_end_ns < cfg.fit_max_age_ns + 2000000);
    ad::FitResult late;
    REQUIRE(ex.service(&late));  // the abandoned request's fit, done late
    CHECK(late.generation == 1);
    const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
    REQUIRE(close(s, plan, c.stamp, c.end) == 1);
    CHECK(events[2].outcome == ad::FitOutcome::kOldGeneration);
    CHECK(events[2].generation == 1);
    CHECK(events[2].fit.stamps == late.fit.stamps);
    CHECK(s.counts().old_generation_results == 1);
    // The next two requests, never fitted, hold both buffers: the third
    // window's end cannot queue.
    while (s.counts().queue_failures == 0) {
      const CycleOut d = runCycle(lat, pr, k++, plan.wake_ns);
      close(s, plan, d.stamp, d.end);
    }
    std::vector<ad::FitOutcome> outcomes;
    for (const ad::FitEvent& e : events) outcomes.push_back(e.outcome);
    const std::vector<ad::FitOutcome> want = {
        ad::FitOutcome::kRequested, ad::FitOutcome::kAbandoned, ad::FitOutcome::kOldGeneration,
        ad::FitOutcome::kRequested, ad::FitOutcome::kAbandoned, ad::FitOutcome::kRequested,
        ad::FitOutcome::kAbandoned, ad::FitOutcome::kQueueFailed};
    CHECK(outcomes == want);
    CHECK(events.back().generation == 0);  // none was made
    CHECK(s.counts().fits_requested == 3);
  }
  events.clear();
  {
    // Without an exchange (tests and offline use) the fit runs in the close:
    // its outcome is reported as applied, with the estimator's fit.
    ad::ReferencedSchedule s(cfg, 0);
    ad::WakePlan plan = s.first();
    std::int64_t k = 0;
    while (s.estimator().state() != ad::EstimatorState::kLocked) {
      const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
      close(s, plan, c.stamp, c.end);
    }
    REQUIRE(events.size() == 1);
    CHECK(events[0].outcome == ad::FitOutcome::kApplied);
    CHECK(events[0].generation == 0);
    CHECK(events[0].window_end_ns == cfg.windows.acquisition_ns);
    CHECK(events[0].fit.share == s.estimator().lastFit().share);
  }
}

TEST_CASE("the intended completion is fixed when the plan is published", "[hw][adapter]") {
  Lattice lat;
  Probe pr;
  ad::ReferencedSchedule s(defaultConfig(), 0);
  ad::WakePlan plan = s.first();
  std::int64_t k = 0;
  while (!plan.referenced || plan.transition) {
    const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
    plan = s.closeAndPublish(c.stamp, c.end, c.end);
  }
  REQUIRE(plan.lock_state == ad::LockState::kLocked);
  const ad::WakePlan published = plan;
  CHECK(published.predicted_completion_ns == std::llround(s.estimator().completionNs(published.target_index)));
  // its reply, 10 us after the predicted completion: gated against that target
  const std::int64_t stamp = published.predicted_completion_ns + 10000;
  s.closeCycle(stamp, stamp + 5000, stamp + 5000);
  CHECK(s.lastObservation() == ad::Observation::kGated);
  CHECK(published.target_index == plan.target_index);  // the published record is its own
  plan = s.publish(stamp + 5000);
  // the next reply, a whole period after its own completion: late, not the next target's
  const std::int64_t late = plan.predicted_completion_ns + 1000000 + 10000;
  s.closeCycle(late, late + 5000, late + 5000);
  CHECK(s.lastObservation() == ad::Observation::kLate);
  s.publish(late + 5000);
}

TEST_CASE("replies at a constant delay after the request lock falsely, and supervision ends the lock",
          "[hw][adapter][safety]") {
  // The pty emulator answers a constant delay after each request: no adapter
  // cadence, yet on the 1 ms grid its replies form a lattice. Locking alone
  // cannot establish independent adapter timing.
  auto run = [](double delay_ns) {
    struct Out {
      ad::EstimatorCounts est;
      double share = 0, first_lock_s = -1, first_loss_s = -1;
      std::uint64_t past = 0, referenced = 0, cycles = 0, first_lock_referenced = 0;
      ad::SkipCounts whole;
    } out;
    ad::ReferencedSchedule s(defaultConfig(), 0);
    ad::WakePlan plan = s.first();
    for (std::int64_t k = 0; plan.wake_ns < 20000000000LL; ++k) {
      const double wake = static_cast<double>(plan.wake_ns) + 2000.0 + 1000.0 * static_cast<double>((k * 7919) % 5);
      const double send = wake + 12000.0;
      const auto stamp = static_cast<std::int64_t>(send + delay_ns + 300.0 * static_cast<double>((k * 104729) % 7));
      const std::int64_t end = stamp + 5000;
      if (plan.referenced) {
        ++out.referenced;  // over the whole simulated run
        if (out.first_loss_s < 0) ++out.first_lock_referenced;  // within the first lock
      }
      const std::uint64_t losses = s.estimator().counts().losses;
      plan = s.closeAndPublish(stamp, end, end);
      if (plan.wake_ns < end) ++out.past;
      if (out.first_lock_s < 0 && s.estimator().state() == ad::EstimatorState::kLocked) {
        out.first_lock_s = static_cast<double>(end) / 1e9;
        out.share = s.estimator().lastFit().share;
      }
      if (out.first_loss_s < 0 && s.estimator().counts().losses > losses) out.first_loss_s = static_cast<double>(end) / 1e9;
    }
    out.est = s.estimator().counts();
    out.whole = s.accounting().wholeRun();
    return out;
  };
  SECTION("700 us: the replies fall outside the gate, the lock ends within 64 cycles, again and again") {
    const auto o = run(700000.0);
    INFO("first lock " << o.first_lock_s << " s, first loss " << o.first_loss_s << " s");
    CHECK(o.share > 0.99);  // the false lock's fit
    CHECK(o.est.locks >= 5);
    CHECK(o.est.lost_no_gated == o.est.losses);
    CHECK(o.first_loss_s - o.first_lock_s < 0.1);
    CHECK(o.past == 0);
  }
  SECTION("800 us, inside the gate: the lock holds for seconds until the period clamp ends it") {
    const auto o = run(800000.0);
    INFO("first lock " << o.first_lock_s << " s, first loss " << o.first_loss_s << " s; referenced cycles "
                       << o.first_lock_referenced << " in the first lock, " << o.referenced << " in the whole run");
    CHECK(o.share > 0.99);
    CHECK(o.est.lost_period_clamp >= 1);
    CHECK(o.first_loss_s - o.first_lock_s > 1.0);  // supervision is slow here: recorded, not a pass
    CHECK(o.past == 0);
  }
}

TEST_CASE("every call in every phase is accepted where it belongs and otherwise changes nothing",
          "[hw][adapter][safety]") {
  using Phase = ad::SchedulePhase;
  enum Call { kFirst, kClose, kPublish, kCloseAndPublish };
  const char* names[] = {"first", "closeCycle", "publish", "closeAndPublish"};
  auto make = [](Phase ph) {
    auto s = std::make_unique<ad::ReferencedSchedule>(defaultConfig(), 1000);
    if (ph != Phase::kUnstarted) s->first();
    if (ph == Phase::kProvisional) s->closeCycle(ad::kInvalidStamp, 700000, 700000);
    REQUIRE(s->phase() == ph);
    REQUIRE(s->counts().protocol_errors == 0);
    return s;
  };
  auto call = [](ad::ReferencedSchedule& s, Call c, std::int64_t t) {
    switch (c) {
      case kFirst: s.first(); break;
      case kClose: s.closeCycle(ad::kInvalidStamp, t, t); break;
      case kPublish: s.publish(t); break;
      case kCloseAndPublish: s.closeAndPublish(ad::kInvalidStamp, t, t); break;
    }
  };
  // what is accepted, and the phase and counted cycles it leaves
  struct Want { bool accepted; Phase after; std::uint64_t cycles; };
  const Want table[3][4] = {
      // unstarted
      {{true, Phase::kPublished, 0}, {false, Phase::kUnstarted, 0}, {false, Phase::kUnstarted, 0},
       {false, Phase::kUnstarted, 0}},
      // published
      {{false, Phase::kPublished, 0}, {true, Phase::kProvisional, 0}, {false, Phase::kPublished, 0},
       {true, Phase::kPublished, 1}},
      // provisional
      {{false, Phase::kProvisional, 0}, {false, Phase::kProvisional, 0}, {true, Phase::kPublished, 1},
       {false, Phase::kProvisional, 0}},
  };
  const Phase phases[] = {Phase::kUnstarted, Phase::kPublished, Phase::kProvisional};
  for (int pi = 0; pi < 3; ++pi) {
    for (int ci = 0; ci < 4; ++ci) {
      INFO("phase " << pi << ", " << names[ci]);
      auto s = make(phases[pi]);
      const ad::WakePlan before = s->current();
      call(*s, static_cast<Call>(ci), 1800000);
      const Want& w = table[pi][ci];
      CHECK(s->phase() == w.after);
      CHECK(s->accounting().wholeRun().cycles == w.cycles);
      CHECK(s->counts().protocol_errors == (w.accepted ? 0U : 1U));
      if (!w.accepted) {
        CHECK(s->current().wake_ns == before.wake_ns);  // nothing changed
        CHECK(s->current().target_index == before.target_index);
      }
      // after it, the proper calls still work: start if unstarted, publish a
      // pending plan, then two whole cycles; each publication counts once
      const std::uint64_t errors = s->counts().protocol_errors;
      const std::uint64_t cycles = s->accounting().wholeRun().cycles;
      std::uint64_t publications = 0;
      if (s->phase() == Phase::kUnstarted) s->first();
      if (s->phase() == Phase::kProvisional) {
        s->publish(1900000);
        ++publications;
      }
      for (int k = 0; k < 2; ++k) {
        const std::int64_t end = s->current().wake_ns + 600000;
        s->closeAndPublish(ad::kInvalidStamp, end, end);
        ++publications;
      }
      CHECK(s->phase() == Phase::kPublished);
      CHECK(s->counts().protocol_errors == errors);
      CHECK(s->accounting().wholeRun().cycles == cycles + publications);
    }
  }
}

TEST_CASE("freshness and timeouts are judged when the call runs, not when the cycle ended",
          "[hw][adapter][safety]") {
  // A delay between a cycle's end and its closeCycle() call makes a result
  // older, never younger.
  const ad::ScheduleConfig cfg = defaultConfig();
  const std::int64_t at = cfg.windows.acquisition_ns;  // the fit window's end (epoch 0)
  auto toRequest = [&](ad::FitExchange& ex, ad::ReferencedSchedule& s) {
    Lattice lat;
    Probe pr;
    ad::WakePlan plan = s.first();
    std::int64_t k = 0;
    while (s.counts().fits_requested == 0) {
      const CycleOut c = runCycle(lat, pr, k++, plan.wake_ns);
      plan = s.closeAndPublish(c.stamp, c.end, c.end);
    }
    (void)ex;
    return plan;
  };
  for (const std::int64_t late : {std::int64_t{0}, std::int64_t{1}}) {
    INFO("late " << late);
    {  // a completed fit taken by a late call: applied at the limit, refused 1 ns past it
       // (plain blocks, not SECTIONs: a SECTION in a loop runs on its first pass only)
      ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
      ad::ReferencedSchedule s(cfg, 0, &ex);
      const ad::WakePlan plan = toRequest(ex, s);
      REQUIRE(ex.service());
      const std::int64_t end = plan.wake_ns + 700000;  // the cycle ended early
      REQUIRE(end - at < cfg.fit_max_age_ns);
      const std::int64_t now = at + cfg.fit_max_age_ns + late;  // the call runs late
      s.closeCycle(ad::kInvalidStamp, end, now);
      const ad::WakePlan pub = s.publish(now);
      CHECK(s.counts().fits_applied == (late == 0 ? 1U : 0U));
      CHECK(s.counts().stale_results == (late == 0 ? 0U : 1U));
      CHECK(pub.wake_ns >= now);
    }
    {  // a request with no result: abandoned once the call's own time passes the limit
      ad::FitExchange ex(cfg.estimator.acquisition_capacity, cfg.estimator.fit);
      ad::ReferencedSchedule s(cfg, 0, &ex);
      const ad::WakePlan plan = toRequest(ex, s);
      const std::int64_t end = plan.wake_ns + 700000;
      const std::int64_t now = at + cfg.fit_max_age_ns + late;
      s.closeCycle(ad::kInvalidStamp, end, now);
      s.publish(now);
      CHECK(s.counts().abandoned_requests == (late == 0 ? 0U : 1U));
      CHECK(s.lockState() == (late == 0 ? ad::LockState::kFitPending : ad::LockState::kAcquiring));
    }
  }
}

// ------------------------------------------------------------ the lead alternation

static_assert(static_cast<int>(ad::CouplingState::kInsufficient) == 0 &&
                  static_cast<int>(ad::CouplingState::kNoCouplingDetected) == 1 &&
                  static_cast<int>(ad::CouplingState::kRequestTiedDetected) == 2,
              "the log's coupling-state encoding");

TEST_CASE("the review's counterexamples: a negative declaration withdrawn, stale evidence expired, a verdict latched",
          "[hw][adapter][coupling]") {
  using St = ad::CouplingState;
  ad::CouplingConfig cc;  // delta 20 us, 32 pairs, confirm 8, freshness 4 x 2 x 32 cycles
  {
    ad::CouplingDetector d(cc);
    std::int64_t n = 0;
    for (int i = 0; i < 80; ++i, ++n) d.onCycle(n, i % 2 == 0 ? 1 : -1, true, 0.0);
    REQUIRE(d.state() == St::kNoCouplingDetected);
    // pairs of 42 us: after 16 of them the median is 21 us, above delta: the
    // first positive evaluation withdraws the negative declaration
    for (int j = 0; j < 16; ++j) {
      d.onCycle(n++, 1, true, -21000.0);
      d.onCycle(n++, -1, true, 21000.0);
      if (j < 15) CHECK(d.state() == St::kNoCouplingDetected);  // the median still 0: its band
    }
    CHECK(d.lastStatistic() == 21000.0);
    CHECK(d.state() == St::kInsufficient);
    for (int j = 0; j < 6; ++j) {
      d.onCycle(n++, 1, true, -21000.0);
      d.onCycle(n++, -1, true, 21000.0);
      CHECK(d.state() == St::kInsufficient);  // confirming
    }
    d.onCycle(n++, 1, true, -21000.0);
    d.onCycle(n++, -1, true, 21000.0);
    CHECK(d.state() == St::kRequestTiedDetected);  // 8 consecutive positive evaluations
    // latched: neither staleness nor contrary evidence withdraws it; reset() does
    d.onCycle(n + 100000, 0, false, 0.0);
    for (int i = 0; i < 80; ++i) d.onCycle(n + 100001 + i, i % 2 == 0 ? 1 : -1, true, 0.0);
    CHECK(d.state() == St::kRequestTiedDetected);
    d.reset();
    CHECK(d.state() == St::kInsufficient);
    CHECK(d.pairs() == 0);
  }
  {
    ad::CouplingDetector s(cc);
    for (int i = 0; i < 80; ++i) s.onCycle(i, i % 2 == 0 ? 1 : -1, true, 0.0);
    REQUIRE(s.state() == St::kNoCouplingDetected);
    s.onCycle(1000, 0, false, 0.0);  // an excluded read: every pair now stale
    CHECK(s.state() == St::kInsufficient);
  }
  {
    // freshness advances on full-window reads and missing observations alike
    ad::CouplingDetector s(cc);
    std::int64_t n = 0;
    for (int i = 0; i < 80; ++i, ++n) s.onCycle(n, i % 2 == 0 ? 1 : -1, true, 0.0);
    REQUIRE(s.state() == St::kNoCouplingDetected);
    const std::int64_t horizon = 4 * 2 * 32;
    // the oldest pair formed at cycle 17 (the window keeps the last 32 of
    // 40): fresh at an age of exactly the horizon, stale 1 cycle later
    for (; n <= 17 + horizon; ++n) {
      s.onCycle(n, n % 2 == 0 ? 0 : 1, false, 0.0);  // full-window reads (sign 0) and missing replies
      REQUIRE(s.state() == St::kNoCouplingDetected);
    }
    s.onCycle(n++, 0, false, 0.0);
    CHECK(s.state() == St::kInsufficient);
    // re-established only by a full fresh window and K confirmations: the
    // 32nd new pair is the first evaluation, the 39th the 8th
    for (int j = 1; j <= 39; ++j) {
      s.onCycle(n++, 1, true, 0.0);
      s.onCycle(n++, -1, true, 0.0);
      if (j < 39) REQUIRE(s.state() == St::kInsufficient);
    }
    CHECK(s.state() == St::kNoCouplingDetected);
  }
}

TEST_CASE("pairs form only from adjacent opposite signs, each sample once; rejected samples break them",
          "[hw][adapter][coupling]") {
  ad::CouplingConfig cc;
  ad::CouplingDetector d(cc);
  d.onCycle(0, 1, true, 5000.0);
  d.onCycle(4, -1, true, 9000.0);  // 4 cycles apart: a pair
  CHECK(d.pairs() == 1);
  d.onCycle(10, 1, true, 0.0);
  d.onCycle(15, -1, true, 0.0);  // 5 apart: no pair; the second sample waits
  CHECK(d.pairs() == 1);
  d.onCycle(16, 1, true, 0.0);  // pairs with cycle 15
  CHECK(d.pairs() == 2);
  d.onCycle(17, 1, true, 0.0);
  d.onCycle(18, 1, true, 0.0);  // the same sign: replaces the waiting sample
  d.onCycle(19, -1, true, 0.0);
  CHECK(d.pairs() == 3);
  d.onCycle(20, 1, true, 0.0);
  d.onCycle(21, -1, true, 600000.0);  // off-period: rejected, and it breaks the waiting sample
  d.onCycle(22, -1, true, 0.0);       // nothing to pair with: it waits
  CHECK(d.pairs() == 3);
  CHECK(d.rejected() == 1);
  d.onCycle(23, 1, true, 0.0);
  CHECK(d.pairs() == 4);
  d.onCycle(24, -1, false, 0.0);  // not usable: neither a sample nor a break
  d.onCycle(25, 1, true, 0.0);
  d.onCycle(26, -1, true, 0.0);
  CHECK(d.pairs() == 5);
  // +, -, +, -: two pairs, not three
  ad::CouplingDetector e(cc);
  e.onCycle(0, 1, true, 0.0);
  e.onCycle(1, -1, true, 0.0);
  e.onCycle(2, 1, true, 0.0);
  e.onCycle(3, -1, true, 0.0);
  CHECK(e.pairs() == 2);
  e.reset();
  CHECK(e.pairs() == 0);
  e.onCycle(4, -1, true, 0.0);  // the waiting sample was cleared too
  CHECK(e.pairs() == 0);
}

TEST_CASE("the sequencer alternates over executed motion-only reads; full-window reads stay at the base lead",
          "[hw][adapter][coupling]") {
  ad::LeadSequencer s;
  CHECK(s.signFor(ad::ReadClass::kMotion) == 1);
  CHECK(s.signFor(ad::ReadClass::kFull) == 0);
  s.executed(ad::ReadClass::kMotion, 1);
  CHECK(s.signFor(ad::ReadClass::kMotion) == -1);
  s.executed(ad::ReadClass::kFull, 0);  // a full-window read: no advance
  CHECK(s.signFor(ad::ReadClass::kMotion) == -1);
  s.executed(ad::ReadClass::kMotion, 0);  // a read executed without a sign: no advance
  CHECK(s.signFor(ad::ReadClass::kMotion) == -1);
  s.executed(ad::ReadClass::kMotion, -1);
  CHECK(s.signFor(ad::ReadClass::kMotion) == 1);
  s.reset();
  CHECK(s.signFor(ad::ReadClass::kMotion) == 1);
}

TEST_CASE("lead offsets: each plan's own rounded wake and +-1 ns, full-window transitions, and falling back",
          "[hw][adapter][coupling][safety]") {
  const ad::PlannerConfig pc;  // lead 780 us, allowance 10 us
  ad::EstimatorConfig ec;
  ec.supervise = false;
  const std::int64_t d = 20000;
  auto wakeOf = [&](const ad::AdapterPhaseEstimator& est, std::int64_t n, std::int64_t off) {
    return static_cast<std::int64_t>(std::ceil(est.completionNs(n) - static_cast<double>(pc.lead_ns + off + pc.wake_to_read_ns)));
  };
  int checked = 0;
  for (const double slope : {-57.7, 13.37}) {
    ad::AdapterPhaseEstimator locked(ec), acquiring(ec);
    locked.lockFrom(slope, 433.3, 0);
    // (the outgoing plan's offset, the incoming plan's): alternation, and into and out of a full-window read
    const std::int64_t pairs[][2] = {{d, -d}, {-d, d}, {d, 0}, {0, -d}, {-d, 0}, {0, d}};
    for (const auto& pr : pairs) {
      for (const std::int64_t delta : {std::int64_t{-1}, std::int64_t{0}, std::int64_t{1}}) {
        INFO("slope " << slope << " offsets " << pr[0] << " -> " << pr[1] << " delta " << delta);
        ad::WakePlanner planner(pc, 1 + 7 * pc.grid_period_ns / 3);
        planner.first();
        const std::int64_t n = 3000;
        const ad::WakePlan a = planner.next(locked, wakeOf(locked, n, pr[0]) - 300000,
                                            wakeOf(locked, n, pr[0]) - 300000, pr[0]);
        REQUIRE(a.target_index == n);
        REQUIRE(a.wake_ns == wakeOf(locked, n, pr[0]));
        REQUIRE(a.lead_offset_ns == pr[0]);
        // the cycle ends at the next plan's own wake, 1 ns before or after
        const std::int64_t own = wakeOf(locked, n + 1, pr[1]);
        const ad::WakePlan b = planner.next(locked, own + delta, own + delta, pr[1]);
        CHECK(b.lead_offset_ns == pr[1]);
        CHECK(b.target_index == (delta > 0 ? n + 2 : n + 1));
        CHECK(b.skipped == (delta > 0 ? 1U : 0U));
        CHECK(b.wake_ns == wakeOf(locked, b.target_index, pr[1]));
        ++checked;
      }
    }
    // falling back from a plan at +delta: the outgoing slot counted with its offset
    for (const std::int64_t delta : {std::int64_t{-1}, std::int64_t{0}, std::int64_t{1}}) {
      const std::int64_t n = 3000;
      const std::int64_t next_slot = wakeOf(locked, n + 1, d);  // the outgoing schedule's next slot
      const std::int64_t grid_wake = next_slot + delta;
      ad::WakePlanner planner(pc, grid_wake - 5 * pc.grid_period_ns);
      planner.first();
      const ad::WakePlan a = planner.next(locked, wakeOf(locked, n, d) - 300000, wakeOf(locked, n, d) - 300000, d);
      REQUIRE(a.target_index == n);
      const ad::WakePlan fb = planner.next(acquiring, grid_wake - 400, grid_wake - 400, d);
      REQUIRE_FALSE(fb.referenced);
      CHECK(fb.lead_offset_ns == 0);  // the grid has no offset
      CHECK(fb.wake_ns == grid_wake);
      CHECK(fb.skipped == (delta > 0 ? 1U : 0U));
      ++checked;
    }
  }
  CHECK(checked == 2 * (6 * 3 + 3));
}

namespace {

struct DitherRun;
void checkOffsets(const DitherRun& r);

// The closed loop with the lead alternation: full-window reads every 100th
// executed cycle (the probe's counter), the read class of each cycle and the
// next passed to the schedule. Request-tied replies when tied_delay > 0.
struct DitherRun {
  std::vector<ad::WakePlan> plans;  // every published plan, in order
  std::vector<bool> full;           // whether each plan's cycle was a full-window read
  ad::SkipCounts whole;
  std::uint64_t published = 0, skipped_published = 0, past = 0;
  std::uint64_t locks = 0, losses = 0;
  std::vector<std::int64_t> tied_at;  // executed cycles from each lock to request-tied
  std::uint64_t locks_seen = 0, tied_locks = 0, free_locks = 0, reset_ok = 0;
  std::uint64_t fallbacks = 0, reset_on_loss = 0;
  std::uint64_t rechecked = 0, offset_changed = 0;
  // each lock: its length in executed cycles and whether request-tied was declared
  std::vector<std::pair<std::int64_t, bool>> lock_outcomes;
};

// publish_delay_every > 0: every so many cycles the publication runs 2.5
// periods after the plan was made (a scheduler delay), so its recheck moves
// the plan.
DitherRun ditherLoop(const Lattice& lat, const Probe& pr, double tied_delay, double duration_ns = 12e9,
                     int publish_delay_every = 0, std::int64_t alternation_ns = 20000) {
  DitherRun r;
  ad::ScheduleConfig cfg = defaultConfig();
  cfg.lead_alternation_ns = alternation_ns;
  cfg.coupling.delta_ns = 99999.0;  // replaced by the alternation's magnitude
  ad::ReferencedSchedule s(cfg, 0);
  ad::WakePlan plan = s.first();
  std::int64_t lock_start = -1;
  bool tied_seen = false, free_seen = false;
  for (std::int64_t k = 0; static_cast<double>(plan.wake_ns) < duration_ns; ++k) {
    const bool full = k % 100 == 0;
    const bool next_full = (k + 1) % 100 == 0;
    std::int64_t stamp = 0, end = 0;
    if (tied_delay > 0) {
      const double send = static_cast<double>(plan.wake_ns) + 2000.0 + 12000.0;
      stamp = static_cast<std::int64_t>(send + tied_delay + (full ? 110000.0 : 0.0) +
                                        2000.0 * (unit(pr.seed, k) - 0.5));
      end = stamp + 5000;
    } else {
      const CycleOut c = runCycle(lat, pr, k, plan.wake_ns);
      stamp = c.stamp;
      end = c.end;
    }
    r.plans.push_back(plan);
    r.full.push_back(full);
    const std::int64_t now =
        publish_delay_every > 0 && k % publish_delay_every == publish_delay_every - 1 ? end + 2500000 : end;
    const ad::WakePlan prov = s.closeCycle(stamp, end, end, full ? ad::ReadClass::kFull : ad::ReadClass::kMotion,
                                           next_full ? ad::ReadClass::kFull : ad::ReadClass::kMotion);
    plan = s.publish(now);
    ++r.published;
    r.skipped_published += plan.skipped;
    if (plan.wake_ns < now) ++r.past;
    if (plan.lead_offset_ns != prov.lead_offset_ns) ++r.offset_changed;
    if (plan.referenced && plan.recheck_skipped > 0) ++r.rechecked;
    if (!plan.referenced && plan.transition) {  // fallen back: the lock was lost at this cycle's reply
      ++r.fallbacks;
      if (s.coupling().pairs() == 0 && s.coupling().state() == ad::CouplingState::kInsufficient &&
          s.sequencer().signFor(ad::ReadClass::kMotion) == 1) {
        ++r.reset_on_loss;
      }
      r.lock_outcomes.emplace_back(k - lock_start, tied_seen);
    }
    if (plan.referenced && plan.transition) {  // a new lock: the detector starts over
      ++r.locks_seen;
      if (s.coupling().pairs() == 0 && s.coupling().state() == ad::CouplingState::kInsufficient) ++r.reset_ok;
      if (tied_seen) ++r.tied_locks;
      if (free_seen) ++r.free_locks;
      lock_start = k;
      tied_seen = free_seen = false;
    }
    if (plan.referenced && plan.coupling_state == 2 && !tied_seen) {
      tied_seen = true;
      r.tied_at.push_back(k - lock_start);
    }
    if (plan.referenced && plan.coupling_state == 1) free_seen = true;
  }
  if (tied_seen) ++r.tied_locks;
  if (free_seen) ++r.free_locks;
  if (lock_start >= 0 && plan.referenced) r.lock_outcomes.emplace_back(static_cast<std::int64_t>(r.plans.size()) - lock_start, tied_seen);
  r.whole = s.accounting().wholeRun();
  r.locks = s.estimator().counts().locks;
  r.losses = s.estimator().counts().losses;
  return r;
}

// The offsets of a run's published plans:
//   - each lock's first motion-only read at +delta, then motion-only reads
//     alternating over the reads executed, skipped targets or not;
//   - full-window reads at the base lead;
//   - each wake from its own plan's offset: consecutive wakes differ by
//     their predicted completions' difference less their offsets' difference,
//     to the rounding (ceil against llround: 1 ns);
//   - consecutive wakes with no skip near P - 2 delta or P + 2 delta, and
//     P -+ delta into and out of a full-window read (the nominal spacing;
//     the estimator's steps move a wake by up to eta, plus 1 us rounding).
void checkOffsets(const DitherRun& r) {
  int last_sign = 0, alternations = 0, breaks = 0, after_skip = 0;
  std::map<long, int> spacing;  // us -> count
  for (std::size_t i = 1; i < r.plans.size(); ++i) {
    const ad::WakePlan& p = r.plans[i];
    const ad::WakePlan& q = r.plans[i - 1];
    if (!p.referenced) {
      last_sign = 0;
      continue;
    }
    if (p.transition) last_sign = 0;
    if (q.referenced && !p.transition) {
      const std::int64_t nominal =
          (p.predicted_completion_ns - q.predicted_completion_ns) - (p.lead_offset_ns - q.lead_offset_ns);
      CHECK(std::llabs(p.wake_ns - q.wake_ns - nominal) <= 1);
      if (p.skipped == 0) ++spacing[std::lround(static_cast<double>(p.wake_ns - q.wake_ns) / 1e3)];
    }
    if (r.full[i]) {
      CHECK(p.lead_offset_ns == 0);
      continue;
    }
    const int sign = p.lead_offset_ns > 0 ? 1 : -1;
    CHECK(std::abs(p.lead_offset_ns) == 20000);
    if (last_sign == 0) {
      CHECK(sign == 1);
    } else if (sign == -last_sign) {
      ++alternations;
      if (p.skipped > 0) ++after_skip;
    } else {
      ++breaks;
    }
    last_sign = sign;
  }
  CHECK(breaks == 0);
  CHECK(alternations > 5000);
  CHECK(after_skip > 10);  // alternation over executed reads, exercised across skipped targets
  const long tol = 3;      // eta (2 us) plus rounding
  for (const long nominal : {960L, 980L, 1020L, 1040L}) {
    int n = 0;
    for (const auto& kv : spacing) {
      if (std::labs(kv.first - nominal) <= tol) n += kv.second;
    }
    INFO("nominal " << nominal << " us");
    CHECK(n > 0);  // full-read transitions both ways, and both alternations, seen
  }
  for (const auto& kv : spacing) {
    INFO("spacing " << kv.first << " us, " << kv.second << " times");
    const bool near = std::labs(kv.first - 960) <= tol || std::labs(kv.first - 980) <= tol ||
                      std::labs(kv.first - 1020) <= tol || std::labs(kv.first - 1040) <= tol;
    CHECK(near);
  }
}

}  // namespace

TEST_CASE("the lead alternation in closed loop: offsets over executed reads, accounting once, outcomes per lock",
          "[hw][adapter][coupling][safety]") {
  SECTION("an independent cadence: no coupling detected, never request-tied") {
    Lattice lat;
    Probe pr;
    const DitherRun r = ditherLoop(lat, pr, 0.0);
    REQUIRE(r.locks == 1);
    CHECK(r.losses == 0);
    CHECK(r.past == 0);
    CHECK(r.tied_at.empty());
    CHECK(r.free_locks == 1);
    CHECK(r.reset_ok == r.locks_seen);
    // exactly once: every publication counted, with the skips it carried
    CHECK(r.whole.cycles == r.published);
    CHECK(r.whole.skipped == r.skipped_published);
    checkOffsets(r);
  }
  SECTION("a negative magnitude is its absolute value; the detector's delta follows it") {
    Lattice lat;
    Probe pr;
    const DitherRun r = ditherLoop(lat, pr, 0.0, 12e9, 0, -20000);
    REQUIRE(r.locks == 1);
    CHECK(r.tied_at.empty());
    CHECK(r.free_locks == 1);
    checkOffsets(r);
    ad::ScheduleConfig cfg = defaultConfig();
    cfg.lead_alternation_ns = -15000;
    cfg.coupling.delta_ns = 99999.0;
    const ad::ReferencedSchedule s(cfg, 0);
    CHECK(s.coupling().config().delta_ns == 15000.0);
  }
  SECTION("delayed publications: each recheck keeps its plan's offset; the accounting stays exact") {
    Lattice lat;
    Probe pr;
    const DitherRun r = ditherLoop(lat, pr, 0.0, 12e9, 37);
    REQUIRE(r.locks == 1);
    CHECK(r.past == 0);
    CHECK(r.rechecked > 100);
    CHECK(r.offset_changed == 0);
    CHECK(r.tied_at.empty());
    CHECK(r.whole.cycles == r.published);
    CHECK(r.whole.skipped == r.skipped_published);
    checkOffsets(r);
  }
  SECTION("request-tied replies (800 us after the send): detected in each lock that lasts") {
    Lattice lat;
    Probe pr;
    const DitherRun r = ditherLoop(lat, pr, 800000.0);
    CHECK(r.locks >= 1);
    CHECK(r.past == 0);
    CHECK(r.free_locks == 0);  // never "no coupling detected"
    REQUIRE_FALSE(r.tied_at.empty());
    for (std::int64_t at : r.tied_at) CHECK(at < 200);
    CHECK(r.reset_ok == r.locks_seen);
    // every false lock ends (the period clamp), and each that lasted 200
    // executed cycles was declared request-tied first
    REQUIRE(r.fallbacks >= 1);
    CHECK(r.reset_on_loss == r.fallbacks);
    REQUIRE(r.lock_outcomes.size() == r.locks_seen);
    for (const auto& lo : r.lock_outcomes) {
      INFO("lock of " << lo.first << " executed cycles");
      CHECK((lo.second || lo.first < 200));
    }
    CHECK(r.whole.cycles == r.published);
    CHECK(r.whole.skipped == r.skipped_published);
  }
}

TEST_CASE("a publication's recheck keeps the plan's lead offset", "[hw][adapter][coupling]") {
  ad::PlannerConfig pc;
  pc.min_gap_ns = 20000;
  ad::EstimatorConfig ec;
  ec.supervise = false;
  ad::AdapterPhaseEstimator locked(ec);
  locked.lockFrom(-57.7, 433.3, 0);
  ad::WakePlanner planner(pc, 12345);
  planner.first();
  const std::int64_t w = static_cast<std::int64_t>(std::ceil(locked.completionNs(3000) - (780000.0 - 20000.0 + 10000.0)));
  const ad::WakePlan prov = planner.next(locked, w - 300000, w - 300000, -20000);
  REQUIRE(prov.target_index == 3000);
  REQUIRE(prov.wake_ns == w);
  const ad::WakePlan pub = planner.recheck(w + 2500000);  // a delay of 2.5 periods
  CHECK(pub.lead_offset_ns == -20000);
  CHECK(pub.recheck_skipped == 3);
  const std::int64_t want =
      static_cast<std::int64_t>(std::ceil(locked.completionNs(pub.target_index) - (780000.0 - 20000.0 + 10000.0)));
  CHECK(pub.wake_ns == want);
}
