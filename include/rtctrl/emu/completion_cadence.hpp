#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace rtctrl::emu {

// An opt-in model of when a USB-serial adapter hands a reply to the host,
// for dxl_emu --cadence and the probe's deterministic tests
// (tests/unit/bus_timing_adapter_test.cpp). Pure logic on the int64
// nanoseconds of one clock: no I/O, no allocation. Without it (the
// default) the emulator answers as before, after the pty's own delays.
//
// Independent cadence: the adapter delivers what it has received only at
// its own completions, independent of when the requests were sent,
//   c_k = origin + phase + k * P (+ the phase steps in force), k = 0, 1, ...
// with P = nominal * (1 + ppm * 1e-6) on the host clock (the adapter's
// clock drifts against the host's by ppm). A reply is ready at its
// request's arrival plus ready_ns plus byte_ns per reply byte (turnaround
// and wire time, modelled), and goes at the first completion at or after
// that which is not missed:
// - delayed completions: every delay_every-th (k + 1 a multiple) delivers
//   delay_ns after its instant (host-side lateness; what it carries is
//   still what was ready by the instant);
// - missed completions: every miss_every-th delivers nothing, and what it
//   would have carried goes at the next one;
// - both only from impair_from_ns after the origin on (by the unshifted
//   instant; 0, the default: from the start), so that a run can lock on
//   the plain cadence before they begin;
// - phase steps: from a step's time on (by the unshifted instant), every
//   completion moves by its shift; each shift adds to the earlier ones.
// The whole reply goes at once: its first bytes are not delivered before
// its last (first_reply_ns equals last_reply_ns at the host).
//
// Request-tied cadence (a negative control): a reply goes tie_ns after its
// request arrived, with no completion lattice, as replies tied to the
// request do.
//
// Releases are monotone in the ready time (the steps' and delays' limits
// below guarantee it); a caller that sends replies in order also keeps
// each release at or after the previous one.

enum class CadenceMode : std::uint8_t { kIndependent = 0, kRequestTied = 1 };

struct PhaseStep {
  std::int64_t at_ns = 0;     // after the origin, by the unshifted instant
  std::int64_t shift_ns = 0;  // |shift| < P / 2
};

struct CadenceConfig {
  static constexpr std::size_t kMaxSteps = 16;

  CadenceMode mode = CadenceMode::kIndependent;
  std::int64_t nominal_period_ns = 1000000;
  double ppm = 0.0;            // |ppm| <= 1000
  std::int64_t phase_ns = 0;   // [0, P)
  std::int64_t ready_ns = 0;   // [0, 1 s]
  std::int64_t byte_ns = 0;    // [0, 100 us]
  std::uint64_t delay_every = 0;  // 0: none
  std::int64_t delay_ns = 0;      // (0, P / 2) when delay_every > 0
  std::uint64_t miss_every = 0;   // 0: none, else >= 2
  std::int64_t impair_from_ns = 0;  // delays and misses from here on
  std::array<PhaseStep, kMaxSteps> steps{};
  std::size_t step_count = 0;  // in time order, at least 2 P apart
  std::int64_t tie_ns = 0;     // request-tied: [0, 30 ms]
};

// The adapter's period on the host clock.
inline double cadencePeriodNs(const CadenceConfig& c) {
  return static_cast<double>(c.nominal_period_ns) * (1.0 + c.ppm * 1e-6);
}

// Why a configuration is not supported (empty: it is).
inline std::string cadenceConfigError(const CadenceConfig& c) {
  if (c.mode == CadenceMode::kRequestTied) {
    if (c.ppm != 0.0 || c.phase_ns != 0 || c.delay_every != 0 || c.miss_every != 0 || c.step_count != 0 ||
        c.impair_from_ns != 0) {
      return "drift, phase, delayed and missed completions and phase steps are for the independent cadence only";
    }
    if (c.tie_ns < 0 || c.tie_ns > 30000000) return "the request-tied delay must be within [0, 30000] us";
    if (c.ready_ns != 0 || c.byte_ns != 0) return "the ready time is for the independent cadence only";
    return {};
  }
  if (c.mode != CadenceMode::kIndependent) return "unknown cadence mode";
  if (c.tie_ns != 0) return "the request-tied delay is for the request-tied cadence only";
  if (c.nominal_period_ns < 100000 || c.nominal_period_ns > 100000000) {
    return "the nominal period must be within [100, 100000] us";
  }
  if (!(std::fabs(c.ppm) <= 1000.0)) return "the drift must be within [-1000, 1000] ppm";
  const double p = cadencePeriodNs(c);
  if (c.phase_ns < 0 || static_cast<double>(c.phase_ns) >= p) return "the phase must be within [0, one period)";
  if (c.ready_ns < 0 || c.ready_ns > 1000000000) return "the ready time must be within [0, 1] s";
  if (c.byte_ns < 0 || c.byte_ns > 100000) return "the time per byte must be within [0, 100] us";
  if (c.delay_every > 0 && !(c.delay_ns > 0 && 2.0 * static_cast<double>(c.delay_ns) < p)) {
    return "a delayed completion's delay must be above 0 and below half a period";
  }
  if (c.delay_every == 0 && c.delay_ns != 0) return "a delay needs delayed completions (every N)";
  if (c.miss_every == 1) return "missed completions: every N with N >= 2 (not every completion)";
  if (c.impair_from_ns < 0) return "the impairments' start must be at or after the start";
  if (c.impair_from_ns > 0 && c.delay_every == 0 && c.miss_every == 0) {
    return "an impairments' start needs delayed or missed completions";
  }
  if (c.step_count > CadenceConfig::kMaxSteps) return "at most 16 phase steps";
  for (std::size_t i = 0; i < c.step_count; ++i) {
    const PhaseStep& s = c.steps[i];
    if (s.at_ns < 0) return "a phase step's time must be at or after the start";
    if (!(2.0 * static_cast<double>(std::llabs(s.shift_ns)) < p) || s.shift_ns == 0) {
      return "a phase step's shift must be nonzero and below half a period";
    }
    if (i > 0 && static_cast<double>(s.at_ns - c.steps[i - 1].at_ns) < 2.0 * p) {
      return "phase steps must be in time order, at least two periods apart";
    }
  }
  return {};
}

// One reply's release.
struct CadenceRelease {
  std::int64_t ready_ns = 0;
  std::int64_t index = -1;          // the completion that carries it (-1: request-tied)
  std::int64_t completion_ns = -1;  // that completion's instant (-1: request-tied)
  std::int64_t release_ns = 0;      // when it goes to the host
  bool delayed = false;             // its completion was a delayed one
  std::uint32_t missed = 0;         // missed completions passed over
};

class CompletionCadence {
 public:
  CompletionCadence(const CadenceConfig& cfg, std::int64_t origin_ns)
      : cfg_(cfg), origin_(origin_ns), period_(cadencePeriodNs(cfg)) {
    const std::string why = cadenceConfigError(cfg);
    if (!why.empty()) throw std::invalid_argument("CompletionCadence: " + why);
  }

  const CadenceConfig& config() const { return cfg_; }
  std::int64_t origin() const { return origin_; }
  double periodNs() const { return period_; }

  // Completion k's instant (independent cadence), its phase steps applied.
  std::int64_t completionNs(std::int64_t k) const {
    const std::int64_t u = unshifted(k);
    return origin_ + u + shiftAt(u);
  }
  bool delayed(std::int64_t k) const {
    return cfg_.delay_every > 0 && (static_cast<std::uint64_t>(k) + 1) % cfg_.delay_every == 0 &&
           unshifted(k) >= cfg_.impair_from_ns;
  }
  bool missed(std::int64_t k) const {
    return cfg_.miss_every > 0 && (static_cast<std::uint64_t>(k) + 1) % cfg_.miss_every == 0 &&
           unshifted(k) >= cfg_.impair_from_ns;
  }

  // When a reply of `bytes` bytes whose request arrived at arrival_ns goes
  // to the host.
  CadenceRelease release(std::int64_t arrival_ns, std::size_t bytes) const {
    CadenceRelease r;
    if (cfg_.mode == CadenceMode::kRequestTied) {
      r.ready_ns = arrival_ns;
      r.release_ns = arrival_ns + cfg_.tie_ns;
      return r;
    }
    r.ready_ns = arrival_ns + cfg_.ready_ns + cfg_.byte_ns * static_cast<std::int64_t>(bytes);
    std::int64_t k = firstAtOrAfter(r.ready_ns);
    while (missed(k)) {
      ++k;
      ++r.missed;
    }
    r.index = k;
    r.completion_ns = completionNs(k);
    r.delayed = delayed(k);
    r.release_ns = r.completion_ns + (r.delayed ? cfg_.delay_ns : 0);
    return r;
  }

  // The first completion at or after t (k >= 0).
  std::int64_t firstAtOrAfter(std::int64_t t) const {
    const double guess = std::ceil(static_cast<double>(t - origin_ - cfg_.phase_ns) / period_);
    std::int64_t k = std::max<std::int64_t>(0, static_cast<std::int64_t>(guess));
    while (k > 0 && completionNs(k - 1) >= t) --k;
    while (completionNs(k) < t) ++k;
    return k;
  }

 private:
  // Completion k's instant after the origin, before the phase steps.
  std::int64_t unshifted(std::int64_t k) const {
    const double base = static_cast<double>(cfg_.phase_ns) + static_cast<double>(k) * period_;
    return static_cast<std::int64_t>(std::llround(base));
  }
  std::int64_t shiftAt(std::int64_t unshifted) const {
    std::int64_t s = 0;
    for (std::size_t i = 0; i < cfg_.step_count; ++i) {
      if (unshifted >= cfg_.steps[i].at_ns) s += cfg_.steps[i].shift_ns;
    }
    return s;
  }

  CadenceConfig cfg_;
  std::int64_t origin_;
  double period_;
};

}  // namespace rtctrl::emu
