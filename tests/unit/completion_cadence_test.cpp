// The emulator's opt-in adapter cadence (rtctrl/emu/completion_cadence.hpp):
// header-only pure logic, checked against its definition on synthetic
// times: the lattice and its drift, the first completion at or after a
// reply's ready time, delayed and missed completions, phase steps, the
// request-tied control, monotone releases, and the configurations refused
// (overflow-safely, at the int64 extremes too).
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include "rtctrl/emu/completion_cadence.hpp"

namespace emu = rtctrl::emu;

namespace {

constexpr std::int64_t kOrigin = 5000000000LL;  // an arbitrary clock reading

emu::CadenceConfig independent(double ppm = 0.0, std::int64_t phase_ns = 0) {
  emu::CadenceConfig c;
  c.ppm = ppm;
  c.phase_ns = phase_ns;
  return c;
}

// Deterministic pseudo-random numbers (a 64-bit LCG).
struct Lcg {
  std::uint64_t s;
  std::uint64_t next() {
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return s >> 11;
  }
  std::int64_t below(std::int64_t n) { return static_cast<std::int64_t>(next() % static_cast<std::uint64_t>(n)); }
};

}  // namespace

TEST_CASE("the completions are a lattice from the origin, drifting by the adapter's ppm", "[emu][cadence]") {
  for (const double ppm : {0.0, -57.5, 150.0, -999.0}) {
    const emu::CompletionCadence c(independent(ppm, 250000), kOrigin);
    const double p = 1e6 * (1 + ppm * 1e-6);
    CHECK(c.periodNs() == p);
    CHECK(c.completionNs(0) == kOrigin + 250000);
    std::uint64_t bad = 0;
    for (std::int64_t k = 1; k <= 3600000; k += 997) {  // an hour of completions
      const double want = static_cast<double>(kOrigin) + 250000.0 + static_cast<double>(k) * p;
      if (std::llabs(c.completionNs(k) - static_cast<std::int64_t>(want)) > 1) ++bad;
      if (c.completionNs(k) <= c.completionNs(k - 1)) ++bad;
    }
    CHECK(bad == 0);
  }
}

TEST_CASE("a reply goes at the first completion at or after its ready time", "[emu][cadence]") {
  emu::CadenceConfig cfg = independent(0.0, 300000);
  cfg.ready_ns = 200000;
  cfg.byte_ns = 1000;
  const emu::CompletionCadence c(cfg, kOrigin);
  // Completions at origin + 300 us + k ms. Ready = arrival + 200 us + 1 us per byte.
  const auto at = [&](std::int64_t arrival_after_origin, std::size_t bytes) {
    return c.release(kOrigin + arrival_after_origin, bytes);
  };
  emu::CadenceRelease r = at(0, 100);  // ready at 300 us: exactly completion 0
  CHECK(r.ready_ns == kOrigin + 300000);
  CHECK(r.index == 0);
  CHECK(r.completion_ns == kOrigin + 300000);
  CHECK(r.release_ns == kOrigin + 300000);
  CHECK_FALSE(r.delayed);
  CHECK(r.missed == 0);
  r = at(1, 100);  // 1 ns later: completion 1
  CHECK(r.index == 1);
  CHECK(r.release_ns == kOrigin + 1300000);
  r = at(-5000000, 0);  // ready before the first completion: the first
  CHECK(r.index == 0);
  r = at(2000000, 50);  // ready at 2.25 ms: completion 2 at 2.3 ms
  CHECK(r.index == 2);
  CHECK(r.release_ns == kOrigin + 2300000);
  // Independent of when the request came, within one completion's span.
  // (ready in (1.3, 2.3] ms: completion 2, at 2.3 ms).
  std::uint64_t other = 0;
  for (std::int64_t a = 1100001; a <= 2100000; a += 1000) {
    if (at(a, 0).index != 2) ++other;
  }
  CHECK(other == 0);
}

TEST_CASE("a delayed completion delivers late what was ready by its instant; a missed one passes it on",
          "[emu][cadence]") {
  emu::CadenceConfig cfg = independent();
  cfg.delay_every = 4;  // completions 3, 7, 11, ...
  cfg.delay_ns = 120000;
  cfg.miss_every = 6;  // completions 5, 11, 17, ...
  const emu::CompletionCadence c(cfg, kOrigin);
  emu::CadenceRelease r = c.release(kOrigin + 2500000, 0);  // ready before completion 3
  CHECK(r.index == 3);
  CHECK(r.delayed);
  CHECK(r.release_ns == kOrigin + 3000000 + 120000);
  r = c.release(kOrigin + 3000001, 0);  // after completion 3's instant, inside its delay
  CHECK(r.index == 4);
  CHECK_FALSE(r.delayed);
  CHECK(r.release_ns == kOrigin + 4000000);
  r = c.release(kOrigin + 4500000, 0);  // completion 5 missed: 6 carries it
  CHECK(r.index == 6);
  CHECK(r.missed == 1);
  CHECK(r.release_ns == kOrigin + 6000000);
  r = c.release(kOrigin + 10500000, 0);  // 11 both delayed and missed: missed wins, 12 carries it
  CHECK(r.index == 12);
  CHECK(r.missed == 1);
  CHECK_FALSE(r.delayed);
  // Over many completions: the delayed and missed ones exactly where the
  // pattern puts them, never two missed in a row.
  std::uint64_t delayed = 0, missed = 0, bad = 0;
  for (std::int64_t k = 0; k < 6000; ++k) {
    const emu::CadenceRelease q = c.release(c.completionNs(k) - 1, 0);
    if (q.missed > 1) ++bad;
    if ((k + 1) % 6 == 0) {
      ++missed;
      if (q.index != k + 1 || q.missed != 1) ++bad;
    } else if (q.index != k) {
      ++bad;
    }
    if (q.delayed) {
      ++delayed;
      if ((q.index + 1) % 4 != 0 || q.release_ns != q.completion_ns + 120000) ++bad;
    }
  }
  CHECK(bad == 0);
  CHECK(missed == 1000);
  CHECK(delayed == 1000);  // k + 1 a multiple of 4 but not of 12 (missed), of 6000
}

TEST_CASE("delayed and missed completions begin at the impairments' start, not before", "[emu][cadence]") {
  emu::CadenceConfig cfg = independent();
  cfg.delay_every = 4;
  cfg.delay_ns = 120000;
  cfg.miss_every = 6;
  cfg.impair_from_ns = 2000000000LL;  // 2 s: completion 2000 on
  const emu::CompletionCadence c(cfg, kOrigin);
  std::uint64_t before = 0, delayed = 0, missed = 0;
  for (std::int64_t k = 0; k < 4000; ++k) {
    if (k < 2000) {
      before += c.delayed(k) || c.missed(k);
      continue;
    }
    delayed += c.delayed(k);
    missed += c.missed(k);
    CHECK(c.delayed(k) == ((k + 1) % 4 == 0));
    CHECK(c.missed(k) == ((k + 1) % 6 == 0));
  }
  CHECK(before == 0);
  CHECK(delayed == 500);
  CHECK(missed == 333);  // k + 1 = 2004, ..., 3996
  CHECK(c.release(c.completionNs(1999) - 1, 0).index == 1999);  // (k + 1 = 2000: both, but before the start)
}

TEST_CASE("phase steps move every later completion, cumulatively, the lattice still in order", "[emu][cadence]") {
  emu::CadenceConfig cfg = independent(-57.5, 100000);
  cfg.steps[0] = {2000000000LL, -300000};
  cfg.steps[1] = {3000000000LL, 450000};
  cfg.step_count = 2;
  const emu::CompletionCadence c(cfg, kOrigin);
  const emu::CompletionCadence plain(independent(-57.5, 100000), kOrigin);
  std::uint64_t bad = 0;
  for (std::int64_t k = 1; k < 5000; ++k) {
    const std::int64_t unshifted = plain.completionNs(k) - kOrigin;
    const std::int64_t want = unshifted >= 3000000000LL   ? 150000
                              : unshifted >= 2000000000LL ? -300000
                                                          : 0;
    if (c.completionNs(k) - plain.completionNs(k) != want) ++bad;
    if (c.completionNs(k) - c.completionNs(k - 1) < 500000) ++bad;  // above half a period
  }
  CHECK(bad == 0);
  // A release after the first step goes at the moved completion.
  const std::int64_t k = 2100;  // unshifted at about 2.0999 s
  const emu::CadenceRelease r = c.release(c.completionNs(k) - 10, 0);
  CHECK(r.index == k);
  CHECK(r.release_ns == plain.completionNs(k) - 300000);
}

TEST_CASE("releases never go backwards as the ready time advances", "[emu][cadence]") {
  Lcg rng{12345};
  std::uint64_t bad = 0, checked = 0;
  for (int config = 0; config < 40; ++config) {
    emu::CadenceConfig cfg = independent(static_cast<double>(rng.below(2001) - 1000) * 0.5,
                                         rng.below(999000));
    cfg.ready_ns = rng.below(800000);
    cfg.byte_ns = rng.below(4000);
    cfg.delay_every = static_cast<std::uint64_t>(rng.below(5));
    cfg.delay_ns = cfg.delay_every > 0 ? 1 + rng.below(490000) : 0;
    cfg.miss_every = static_cast<std::uint64_t>(rng.below(4)) * 3;  // 0, 3, 6, 9
    if (cfg.delay_every > 0 || cfg.miss_every > 0) cfg.impair_from_ns = rng.below(3) * 500000000LL;
    cfg.step_count = static_cast<std::size_t>(rng.below(4));
    for (std::size_t i = 0; i < cfg.step_count; ++i) {
      cfg.steps[i] = {static_cast<std::int64_t>(i + 1) * 700000000LL, rng.below(980001) - 490000};
      if (cfg.steps[i].shift_ns == 0) cfg.steps[i].shift_ns = 1;
    }
    REQUIRE(emu::cadenceConfigError(cfg).empty());
    const emu::CompletionCadence c(cfg, kOrigin);
    std::int64_t t = kOrigin - 2000000, last = INT64_MIN;
    for (int i = 0; i < 4000; ++i) {
      t += rng.below(1500000);
      const emu::CadenceRelease r = c.release(t, 120);
      ++checked;
      if (r.release_ns < last) ++bad;               // monotone
      if (r.completion_ns < r.ready_ns) ++bad;      // never before it is ready
      const std::int64_t first = r.index - static_cast<std::int64_t>(r.missed);
      if (first > 0 && c.completionNs(first - 1) >= r.ready_ns) ++bad;  // the first one
      for (std::int64_t k = first; k < r.index; ++k) {
        if (!c.missed(k)) ++bad;  // only missed ones passed over
      }
      last = r.release_ns;
    }
  }
  CHECK(bad == 0);
  CHECK(checked == 160000);
}

TEST_CASE("the request-tied control: each reply a fixed delay after its request, no lattice", "[emu][cadence]") {
  emu::CadenceConfig cfg;
  cfg.mode = emu::CadenceMode::kRequestTied;
  cfg.tie_ns = 150000;
  const emu::CompletionCadence c(cfg, kOrigin);
  for (const std::int64_t a : {kOrigin - 7, kOrigin + 1, kOrigin + 1234567}) {
    const emu::CadenceRelease r = c.release(a, 144);
    CHECK(r.release_ns == a + 150000);
    CHECK(r.index == -1);
    CHECK(r.completion_ns == -1);
  }
}

TEST_CASE("unsupported cadence configurations are refused, naming the rule", "[emu][cadence]") {
  const auto refused = [](const emu::CadenceConfig& c, const std::string& why) {
    const std::string e = emu::cadenceConfigError(c);
    INFO(e);
    CHECK(e.find(why) != std::string::npos);
    CHECK_THROWS_AS(emu::CompletionCadence(c, 0), std::invalid_argument);
  };
  CHECK(emu::cadenceConfigError(emu::CadenceConfig{}).empty());
  emu::CadenceConfig c;
  c.ppm = 1000.5;
  refused(c, "drift");
  c = {};
  c.phase_ns = 1000000;
  refused(c, "phase");
  c = {};
  c.phase_ns = -1;
  refused(c, "phase");
  c = {};
  c.nominal_period_ns = 50000;
  refused(c, "nominal period");
  c = {};
  c.ready_ns = -1;
  refused(c, "ready time");
  c = {};
  c.byte_ns = 100001;
  refused(c, "per byte");
  c = {};
  c.delay_every = 3;
  refused(c, "half a period");  // no delay given
  c.delay_ns = 500000;
  refused(c, "half a period");
  c = {};
  c.delay_ns = 1000;
  refused(c, "needs delayed completions");
  c = {};
  c.miss_every = 1;
  refused(c, "N >= 2");
  c = {};
  c.impair_from_ns = -1;
  refused(c, "impairments' start must be at or after");
  c.impair_from_ns = 1000;
  refused(c, "needs delayed or missed completions");
  c.miss_every = 2;
  CHECK(emu::cadenceConfigError(c).empty());
  c = {};
  c.step_count = 1;
  c.steps[0] = {1000000000LL, 500000};
  refused(c, "below half a period");
  c.steps[0].shift_ns = 0;
  refused(c, "nonzero");
  c.steps[0] = {-1, 1000};
  refused(c, "at or after the start");
  c.step_count = 2;
  c.steps[0] = {1000000000LL, 1000};
  c.steps[1] = {1001000000LL, 1000};
  refused(c, "two periods apart");
  c = {};
  c.tie_ns = 1000;
  refused(c, "request-tied cadence only");
  c = {};
  c.mode = emu::CadenceMode::kRequestTied;
  c.tie_ns = 30000001;
  refused(c, "[0, 30000] us");
  c.tie_ns = 1000;
  c.ppm = 1.0;
  refused(c, "independent cadence only");
  c.ppm = 0.0;
  c.ready_ns = 1;
  refused(c, "independent cadence only");
  c.ready_ns = 0;
  CHECK(emu::cadenceConfigError(c).empty());
}

TEST_CASE("validation is overflow-safe: int64 extremes are refused, never wrapped into range, and each bound holds "
          "on both sides",
          "[emu][cadence]") {
  constexpr std::int64_t kMin = std::numeric_limits<std::int64_t>::min();
  constexpr std::int64_t kMax = std::numeric_limits<std::int64_t>::max();
  const auto ok = [](const emu::CadenceConfig& c) {
    const std::string e = emu::cadenceConfigError(c);
    INFO(e);
    CHECK(e.empty());
  };
  const auto refused = [](const emu::CadenceConfig& c, const std::string& why) {
    const std::string e = emu::cadenceConfigError(c);
    INFO(e);
    CHECK(e.find(why) != std::string::npos);
    CHECK_THROWS_AS(emu::CompletionCadence(c, kOrigin), std::invalid_argument);
  };

  // A phase step's shift, |shift| < P / 2 on both sides, at the nominal
  // period, a drifted one (P / 2 not a whole ns) and the drift's limit.
  for (const double ppm : {0.0, -57.5, 1000.0}) {
    emu::CadenceConfig c = independent(ppm);
    c.step_count = 1;
    const double p = emu::cadencePeriodNs(c);
    const auto largest = static_cast<std::int64_t>(std::ceil(p / 2.0)) - 1;
    INFO("ppm " << ppm << ", largest shift " << largest);
    for (const std::int64_t sign : {1, -1}) {
      c.steps[0] = {1000000000LL, sign * largest};
      ok(c);
      c.steps[0].shift_ns = sign * (largest + 1);
      refused(c, "below half a period");
    }
  }
  // The review's case: INT64_MIN, whose llabs overflows, and the others.
  for (const std::int64_t shift : {kMin, kMin + 1, kMax}) {
    emu::CadenceConfig c;
    c.step_count = 1;
    c.steps[0] = {1000000000LL, shift};
    refused(c, "below half a period");
  }

  // Every other int64 field at both extremes.
  emu::CadenceConfig c;
  for (const std::int64_t v : {kMin, kMax}) {
    INFO("value " << v);
    c = {};
    c.nominal_period_ns = v;
    refused(c, "nominal period");
    c = {};
    c.phase_ns = v;
    refused(c, "phase");
    c = {};
    c.ready_ns = v;
    refused(c, "ready time");
    c = {};
    c.byte_ns = v;
    refused(c, "per byte");
    c = {};
    c.delay_every = 3;
    c.delay_ns = v;
    refused(c, "half a period");
    c.delay_every = 0;
    refused(c, "needs delayed completions");
    c = {};
    c.tie_ns = v;
    refused(c, "request-tied cadence only");
    c.mode = emu::CadenceMode::kRequestTied;
    refused(c, "[0, 30000] us");
  }
  c = {};
  c.miss_every = 2;
  c.impair_from_ns = kMin;
  refused(c, "impairments' start must be at or after");
  c.impair_from_ns = kMax;  // never reached: no impairment
  ok(c);
  c = {};
  c.step_count = 1;
  c.steps[0] = {kMin, 1000};
  refused(c, "at or after the start");
  c.steps[0] = {kMax, 1000};  // never reached
  ok(c);
  c.step_count = 2;
  c.steps[0] = {0, 1000};
  c.steps[1] = {kMax, 1000};
  ok(c);
  c.steps[0] = {kMax, 1000};
  c.steps[1] = {0, 1000};  // out of order, the difference -kMax
  refused(c, "time order");
  c.steps[1] = {kMax, 1000};
  refused(c, "time order");
  c.step_count = emu::CadenceConfig::kMaxSteps + 1;
  refused(c, "at most 16");
  for (const double ppm : {std::nan(""), std::numeric_limits<double>::infinity(),
                           -std::numeric_limits<double>::infinity()}) {
    c = {};
    c.ppm = ppm;
    refused(c, "drift");
  }
}

TEST_CASE("releases stay prompt and on the lattice next to valid extreme steps", "[emu][cadence]") {
  // The largest valid shifts, both ways, a step never reached, and a
  // reply ready long after them: each release a completion at or after
  // its ready time, less than a period later.
  emu::CadenceConfig c;
  c.step_count = 3;
  c.steps[0] = {1000000000LL, -499999};
  c.steps[1] = {2000000000LL, 499999};
  c.steps[2] = {std::numeric_limits<std::int64_t>::max(), 1000};
  const emu::CompletionCadence cadence(c, kOrigin);
  for (const std::int64_t at : {500000000LL, 1500000000LL, 2500000000LL, 3600000000000LL}) {
    const emu::CadenceRelease r = cadence.release(kOrigin + at, 0);
    INFO("ready " << at);
    CHECK(r.completion_ns >= r.ready_ns);
    CHECK(r.completion_ns - r.ready_ns < 1000000);
    CHECK(r.completion_ns == cadence.completionNs(r.index));
  }
}

TEST_CASE("the request-tied control refuses each of the independent cadence's options, the delay among them",
          "[emu][cadence]") {
  const auto refused = [](const emu::CadenceConfig& c) {
    const std::string e = emu::cadenceConfigError(c);
    INFO(e);
    CHECK(e.find("independent cadence only") != std::string::npos);
  };
  const auto tied = [] {
    emu::CadenceConfig c;
    c.mode = emu::CadenceMode::kRequestTied;
    c.tie_ns = 790000;
    return c;
  };
  CHECK(emu::cadenceConfigError(tied()).empty());
  for (const std::int64_t delay : {std::int64_t{120000}, std::int64_t{-120000}, std::int64_t{1},
                                   std::numeric_limits<std::int64_t>::min()}) {
    emu::CadenceConfig c = tied();
    c.delay_ns = delay;  // the review's case: silently ignored before
    INFO("delay " << delay);
    refused(c);
  }
  emu::CadenceConfig c = tied();
  c.delay_every = 3;
  refused(c);
  c = tied();
  c.miss_every = 2;
  refused(c);
  c = tied();
  c.phase_ns = 1;
  refused(c);
  c = tied();
  c.impair_from_ns = 1;
  refused(c);
  c = tied();
  c.step_count = 1;
  c.steps[0] = {1000000000LL, 1000};
  refused(c);
  c = tied();
  c.byte_ns = 1;
  refused(c);
}
