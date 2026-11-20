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
#include <random>
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
    plan = sched.closeCycle(c.stamp, c.end, now);
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
  ad::WakePlanner planner(cfg, 0);
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
  locked.lockFrom(-57.5, 321.0, 0);
  ad::PlannerConfig cfg;
  cfg.min_gap_ns = 2000;
  ad::WakePlanner planner(cfg, 0);
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
    // skipped, recounted: the outgoing schedule's slots before the new wake
    // (the grid's; the lattice's targets passed over, or on falling back its
    // slots from the last lattice plan's wake and period)
    std::uint64_t count = 0;
    if (!prev.referenced) {
      for (std::int64_t w = prev.wake_ns + 1000000; w < plan.wake_ns; w += 1000000) ++count;
    } else if (plan.referenced) {
      count = static_cast<std::uint64_t>(plan.target_index - prev.target_index - 1);
    } else {
      for (double w = static_cast<double>(prev.wake_ns) + locked.periodNs();
           w < static_cast<double>(plan.wake_ns); w += locked.periodNs()) {
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
    plan = sched.closeCycle(c.stamp, c.end, c.end);
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
    pa = sa.closeCycle(ca.stamp, ca.end, ca.end);
    pb = sb.closeCycle(cb.stamp, cb.end, cb.end);
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
