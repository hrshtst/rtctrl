#pragma once

// Adapter-referenced cycle timing: pure logic, not wired into any probe or
// controller (the stage approved on 2026-11-20 is this logic, its unit tests
// and an offline replay; integration needs its own review).
//
// The USB serial adapter returns data in bulk IN completions on its own
// cadence near 1 ms, which drifts against the host's monotonic clock. The
// probe sees a completion only when it carries reply bytes: its read returns
// them a host receive delay after the completion, sometimes whole periods
// later. This header holds:
//   - fitLattice(): the acquisition fit, a port of the offline analysis's
//     phase-line fit (review-notes/2026-11-20-h2r-attempt26-offline-r2,
//     deadline_grid.fit_phase): the slope and phase whose 20 us window on the
//     1 ms circle holds the most reply stamps;
//   - AdapterPhaseEstimator: acquisition, then a gated, sign-based quantile
//     tracker of the completion lattice (anchor and period), with lock
//     supervision; under adapter-referenced timing a reply is compared with
//     the completion its cycle intended, so a reply delayed by whole periods
//     is late, never mistaken for an on-time reply of a later completion;
//   - WakePlanner: each next wake from what is already known (the estimator
//     as it stands after the cycle just ended), never before now, on the host
//     grid when not locked and on the lattice when locked; it counts the
//     slots of the schedule being left that pass without a cycle;
//   - SkipAccounting: cycles and skips per declared window (acquisition,
//     settling, locked measurement) and for the whole run, apart;
//   - ReferencedSchedule: the three together, one call per cycle.
//
// Times are int64 nanoseconds on one monotonic clock; lattice positions are
// doubles in the same nanoseconds (0.06 ns resolution at 4e14 ns).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace rtctrl::hw::adapter {

inline constexpr std::int64_t kInvalidStamp = -1;

// ---------------------------------------------------------------- the fit

struct LatticeFitConfig {
  // The coarse slope grid (us per s, i.e. ppm of the 1 ms grid) and the
  // fine one around its best value, built as numpy.arange builds them so
  // that the offline replay reproduces the analysis's fit exactly.
  double slope_min_ppm = -200.0;
  double slope_max_ppm = 200.0;
  double slope_step_ppm = 0.5;
  double fine_half_width_ppm = 0.5;
  double fine_step_ppm = 0.01;
  int window_us = 20;
};

struct LatticeFit {
  double slope_ppm = 0.0;  // the lattice's drift on the 1 ms grid (us per s)
  double phase0_us = 0.0;  // the window's start within the slot at t = 0
  double share = 0.0;      // the share of stamps inside the window
  std::size_t stamps = 0;
};

namespace detail {

// numpy.mod for a positive divisor: the result takes the divisor's sign.
inline double pymod(double x, double y) {
  double m = std::fmod(x, y);
  if (m < 0.0) m += y;  // fmod keeps the dividend's sign
  return m;
}

// numpy.arange(start, stop, step) for doubles: the length from the
// arguments, then start + i * (the first two values' difference).
inline std::vector<double> arange(double start, double stop, double step) {
  std::vector<double> v;
  const double len = std::ceil((stop - start) / step);
  if (!(len > 0)) return v;
  const auto n = static_cast<std::size_t>(len);
  v.reserve(n);
  v.push_back(start);
  if (n > 1) {
    const double second = start + step;
    const double delta = second - start;
    v.push_back(second);
    for (std::size_t i = 2; i < n; ++i)
      v.push_back(start + static_cast<double>(i) * delta);
  }
  return v;
}

struct WindowCount {
  int count = 0;
  int start_bin = 0;
};

// The most stamps in a circular window of w 1 us bins, for slope s.
inline WindowCount bestWindow(const std::int64_t* stamps, std::size_t n,
                              double s, int w) {
  std::array<int, 1000> h{};
  for (std::size_t k = 0; k < n; ++k) {
    const double ts = static_cast<double>(stamps[k]) / 1e9;
    const double y = static_cast<double>(stamps[k] % 1000000) / 1e3;
    const double z = pymod(y - s * ts, 1000.0);
    const auto b = static_cast<long long>(std::floor(z));
    h[static_cast<std::size_t>(((b % 1000) + 1000) % 1000)] += 1;
  }
  int sum = 0;
  for (int i = 0; i < w; ++i) sum += h[static_cast<std::size_t>(i % 1000)];
  WindowCount best{sum, 0};
  for (int i = 1; i < 1000; ++i) {
    sum += h[static_cast<std::size_t>((i + w - 1) % 1000)] -
           h[static_cast<std::size_t>(i - 1)];
    if (sum > best.count) best = WindowCount{sum, i};  // the first maximum
  }
  return best;
}

}  // namespace detail

// The acquisition fit over reply stamps (ns, ascending, all >= 0). Not for a
// control thread: it bins every stamp once per candidate slope (some ms for
// 2,000 stamps on the default grid).
inline LatticeFit fitLattice(const std::int64_t* stamps, std::size_t n,
                             const LatticeFitConfig& cfg = {}) {
  LatticeFit out;
  out.stamps = n;
  if (n == 0) return out;
  const auto coarse = detail::arange(cfg.slope_min_ppm, cfg.slope_max_ppm,
                                     cfg.slope_step_ppm);
  if (coarse.empty()) return out;
  std::size_t best_i = 0;
  int best_count = -1;
  for (std::size_t i = 0; i < coarse.size(); ++i) {
    const int c = detail::bestWindow(stamps, n, coarse[i], cfg.window_us).count;
    if (c > best_count) {
      best_count = c;
      best_i = i;
    }
  }
  const double s0 = coarse[best_i];
  const auto fine = detail::arange(s0 - cfg.fine_half_width_ppm,
                                   s0 + cfg.fine_half_width_ppm,
                                   cfg.fine_step_ppm);
  detail::WindowCount best{-1, 0};
  double best_s = s0;
  for (double s : fine) {
    const auto wc = detail::bestWindow(stamps, n, s, cfg.window_us);
    if (wc.count > best.count) {
      best = wc;
      best_s = s;
    }
  }
  out.slope_ppm = best_s;
  out.phase0_us = static_cast<double>(best.start_bin);
  out.share = static_cast<double>(best.count) / static_cast<double>(n);
  return out;
}

// ---------------------------------------------------------- the estimator

struct EstimatorConfig {
  std::int64_t nominal_period_ns = 1000000;
  // The tracker: a + delay settles at the tau-quantile of the stamps above
  // the lattice; within the gate the anchor steps +eta*tau or
  // -eta*(1-tau); outside it nothing moves.
  double eta_ns = 2000.0;
  double tau = 0.05;
  double delay_ns = 10000.0;
  double gate_ns = 50000.0;
  // The period: every period_update_stamps gated stamps, gain times the
  // anchor's drift per period, within +-period_clamp_ppm of nominal.
  int period_update_stamps = 512;
  double period_gain = 0.5;
  double period_clamp_ppm = 500.0;
  // Locking after acquisition.
  double lock_slope_limit_ppm = 200.0;
  double lock_min_share = 0.75;
  std::size_t lock_min_stamps = 500;
  std::size_t acquisition_capacity = 4096;
  // Lock supervision (supervise = false only for the offline replay of the
  // analysis's tracker, which had none): lost when more than
  // loss_max_outside_share of the last loss_window observed cycles fell
  // outside the gate, after loss_max_ungated_cycles cycles in a row with
  // no gated stamp, at the period clamp, or on a non-finite value.
  bool supervise = true;
  int loss_window = 256;
  double loss_max_outside_share = 0.25;
  int loss_max_ungated_cycles = 64;
  LatticeFitConfig fit;
};

enum class EstimatorState : std::uint8_t { kAcquiring, kLocked };

enum class LossCause : std::uint8_t {
  kNone,
  kOutsideGate,
  kNoGatedStamp,
  kPeriodClamp,
  kNonFinite,
};

enum class Observation : std::uint8_t {
  kStored,    // acquisition: kept for the fit
  kDropped,   // acquisition: the buffer is full
  kGated,     // tracking: within the gate, the anchor stepped
  kLate,      // tracking: after the gate (receive delay, or a later completion)
  kEarly,     // tracking: before the gate (the estimate is late)
  kInvalid,   // no stamp, out of order, or not possible for its cycle
};

struct EstimatorCounts {
  std::uint64_t stored = 0, dropped = 0, gated = 0, late = 0, early = 0,
                invalid = 0, locks = 0, failed_acquisitions = 0, losses = 0;
};

class AdapterPhaseEstimator {
 public:
  explicit AdapterPhaseEstimator(EstimatorConfig cfg = {})
      : cfg_(cfg),
        window_(static_cast<std::size_t>(std::max(cfg.loss_window, 1)), 0) {
    stamps_.reserve(cfg_.acquisition_capacity);
  }

  EstimatorState state() const { return state_; }
  LossCause lossCause() const { return loss_cause_; }
  const EstimatorCounts& counts() const { return counts_; }
  const LatticeFit& lastFit() const { return last_fit_; }
  const EstimatorConfig& config() const { return cfg_; }
  double periodNs() const { return period_; }
  double anchorNs() const { return anchor_; }
  std::int64_t anchorIndex() const { return anchor_index_; }
  std::size_t acquisitionStamps() const { return stamps_.size(); }

  // The lattice point of a completion index, on the current estimate.
  double completionNs(std::int64_t index) const {
    return anchor_ + static_cast<double>(index - anchor_index_) * period_;
  }
  // The smallest index whose lattice point is at or after t.
  std::int64_t firstIndexAtOrAfter(double t) const {
    auto k = static_cast<std::int64_t>(std::ceil((t - anchor_) / period_));
    std::int64_t n = anchor_index_ + k;
    while (completionNs(n) < t) ++n;
    while (completionNs(n - 1) >= t) --n;
    return n;
  }

  // Acquisition: a reply stamp, kept for the fit. Never allocates.
  Observation addAcquisitionStamp(std::int64_t stamp) {
    if (state_ != EstimatorState::kAcquiring || stamp < 0 ||
        stamp < last_stamp_) {
      ++counts_.invalid;
      return Observation::kInvalid;
    }
    last_stamp_ = stamp;
    if (stamps_.size() >= cfg_.acquisition_capacity) {
      ++counts_.dropped;
      return Observation::kDropped;
    }
    stamps_.push_back(stamp);
    ++counts_.stored;
    return Observation::kStored;
  }

  // Ends an acquisition window at at_ns: fits the stored stamps and locks
  // if the fit qualifies (the anchor is then the first lattice point at or
  // after at_ns, index 0). Otherwise the stamps are discarded and it stays
  // acquiring. Not for a control thread (fitLattice).
  bool finishAcquisition(std::int64_t at_ns) {
    if (state_ != EstimatorState::kAcquiring) return false;
    last_fit_ = fitLattice(stamps_.data(), stamps_.size(), cfg_.fit);
    const bool ok = stamps_.size() >= cfg_.lock_min_stamps &&
                    std::fabs(last_fit_.slope_ppm) <= cfg_.lock_slope_limit_ppm &&
                    last_fit_.share >= cfg_.lock_min_share;
    stamps_.clear();
    if (!ok) {
      ++counts_.failed_acquisitions;
      return false;
    }
    lockFrom(last_fit_.slope_ppm, last_fit_.phase0_us, at_ns);
    return true;
  }

  // Locks from a given line (the replay's entry; finishAcquisition uses it).
  void lockFrom(double slope_ppm, double phase0_us, std::int64_t at_ns) {
    const auto p = static_cast<double>(cfg_.nominal_period_ns);
    period_ = p * (1 + slope_ppm * 1e-6);
    const auto at = static_cast<double>(at_ns);
    const double a = phase0_us * 1e3 + slope_ppm * 1e3 * at / 1e9;
    anchor_ = at + detail::pymod(a - at, p);
    anchor_index_ = 0;
    steps_ = 0.0;
    periods_ = 0;
    gated_since_update_ = 0;
    ungated_run_ = 0;
    window_fill_ = 0;
    window_pos_ = 0;
    window_outside_ = 0;
    std::fill(window_.begin(), window_.end(), 0);
    loss_cause_ = LossCause::kNone;
    state_ = EstimatorState::kLocked;
    ++counts_.locks;
  }

  // Tracking under adapter-referenced timing: the reply stamp of the cycle
  // that intended completion `index` (kInvalidStamp when the cycle had no
  // reply). A stamp is compared with that completion only: whole periods
  // late is kLate, not a later completion's on-time reply.
  Observation observeIntended(std::int64_t stamp, std::int64_t index) {
    if (state_ != EstimatorState::kLocked) {
      ++counts_.invalid;
      return Observation::kInvalid;
    }
    if (stamp < 0 || stamp < last_stamp_ || index < anchor_index_) {
      ++counts_.invalid;
      supervise(false);
      return Observation::kInvalid;
    }
    return track(stamp, index - anchor_index_);
  }

  // The offline analysis's mapping, kept for the replay that reproduces it:
  // the nearest lattice point. Under adapter-referenced timing it would
  // alias a reply delayed by about a period onto the next completion.
  Observation observeNearest(std::int64_t stamp) {
    if (state_ != EstimatorState::kLocked || stamp < 0 || stamp < last_stamp_) {
      ++counts_.invalid;
      return Observation::kInvalid;
    }
    const double s = static_cast<double>(stamp);
    const auto n = static_cast<std::int64_t>(
        std::nearbyint((s - anchor_ - cfg_.delay_ns) / period_));
    return track(stamp, n);
  }

 private:
  Observation track(std::int64_t stamp, std::int64_t n) {
    last_stamp_ = stamp;
    const double s = static_cast<double>(stamp);
    const double e = s - (anchor_ + static_cast<double>(n) * period_) - cfg_.delay_ns;
    anchor_ += static_cast<double>(n) * period_;
    anchor_index_ += n;
    periods_ += n;
    Observation out;
    if (std::fabs(e) <= cfg_.gate_ns) {
      const double step = e > 0 ? cfg_.eta_ns * cfg_.tau
                                : -cfg_.eta_ns * (1 - cfg_.tau);
      anchor_ += step;
      steps_ += step;
      ++gated_since_update_;
      ++counts_.gated;
      out = Observation::kGated;
    } else if (e > 0) {
      ++counts_.late;
      out = Observation::kLate;
    } else {
      ++counts_.early;
      out = Observation::kEarly;
    }
    if (gated_since_update_ == cfg_.period_update_stamps && periods_ > 0) {
      const auto p = static_cast<double>(cfg_.nominal_period_ns);
      const double lo = p * (1 - cfg_.period_clamp_ppm * 1e-6);
      const double hi = p * (1 + cfg_.period_clamp_ppm * 1e-6);
      const double want = period_ + cfg_.period_gain * steps_ / static_cast<double>(periods_);
      period_ = std::min(std::max(want, lo), hi);
      steps_ = 0.0;
      periods_ = 0;
      gated_since_update_ = 0;
      if (cfg_.supervise && (want <= lo || want >= hi)) {
        lose(LossCause::kPeriodClamp);
        return out;
      }
    }
    if (cfg_.supervise && (!std::isfinite(anchor_) || !std::isfinite(period_))) {
      lose(LossCause::kNonFinite);
      return out;
    }
    supervise(out == Observation::kGated);
    return out;
  }

  void supervise(bool gated) {
    if (!cfg_.supervise || state_ != EstimatorState::kLocked) return;
    ungated_run_ = gated ? 0 : ungated_run_ + 1;
    const std::uint8_t outside = gated ? 0 : 1;
    if (window_fill_ == window_.size()) window_outside_ -= window_[window_pos_];
    else ++window_fill_;
    window_[window_pos_] = outside;
    window_outside_ += outside;
    window_pos_ = (window_pos_ + 1) % window_.size();
    if (ungated_run_ >= cfg_.loss_max_ungated_cycles) {
      lose(LossCause::kNoGatedStamp);
    } else if (window_fill_ == window_.size() &&
               static_cast<double>(window_outside_) >
                   cfg_.loss_max_outside_share * static_cast<double>(window_.size())) {
      lose(LossCause::kOutsideGate);
    }
  }

  void lose(LossCause cause) {
    loss_cause_ = cause;
    state_ = EstimatorState::kAcquiring;
    stamps_.clear();
    ++counts_.losses;
  }

  EstimatorConfig cfg_;
  EstimatorState state_ = EstimatorState::kAcquiring;
  LossCause loss_cause_ = LossCause::kNone;
  EstimatorCounts counts_;
  LatticeFit last_fit_;
  std::vector<std::int64_t> stamps_;
  std::int64_t last_stamp_ = std::numeric_limits<std::int64_t>::min();
  double period_ = 1e6;
  double anchor_ = 0.0;
  std::int64_t anchor_index_ = 0;
  double steps_ = 0.0;
  std::int64_t periods_ = 0;
  int gated_since_update_ = 0;
  int ungated_run_ = 0;
  std::vector<std::uint8_t> window_;
  std::size_t window_fill_ = 0, window_pos_ = 0, window_outside_ = 0;
};

// ------------------------------------------------------------ the planner

struct PlannerConfig {
  std::int64_t grid_period_ns = 1000000;
  std::int64_t lead_ns = 780000;         // read start to the target completion
  std::int64_t wake_to_read_ns = 10000;  // the planned wake to the read start
  std::int64_t min_gap_ns = 0;           // no wake before now + min_gap
};

struct WakePlan {
  std::int64_t wake_ns = 0;
  std::int64_t target_index = -1;  // the intended completion; -1 on the grid
  std::uint64_t skipped = 0;  // slots of the schedule left, passed without a cycle
  bool referenced = false;    // on the lattice
  bool transition = false;    // the schedule changed at this plan
};

inline std::int64_t ceilDiv(std::int64_t a, std::int64_t b) {
  const std::int64_t q = a / b;
  return (a % b != 0 && ((a > 0) == (b > 0))) ? q + 1 : q;
}

class WakePlanner {
 public:
  WakePlanner(PlannerConfig cfg, std::int64_t epoch_ns)
      : cfg_(cfg), epoch_(epoch_ns) {}

  // The first cycle wakes at the epoch, on the grid.
  WakePlan first() {
    last_ = WakePlan{epoch_, -1, 0, false, false};
    grid_index_ = 0;
    return last_;
  }

  // The next wake after a cycle that ended at cycle_end_ns, planned at
  // now_ns (>= cycle_end_ns) from the estimator as it stands; t is
  // max(cycle_end, now + min_gap), and the wake is never before t.
  //   - Within one schedule, hw::advanceCycleDeadline's rule: the next slot
  //     if t is at or before it (no skip), else the first slot after t.
  //   - On a change of schedule (locking, or falling back), the incoming
  //     schedule's first slot at or after t.
  // skipped counts the slots of the outgoing schedule, after the last plan,
  // that come before the new wake: within one schedule exactly the probe's
  // skipped periods (on the lattice, the targets passed over); on falling
  // back, the lattice's slots as they stood when the last lattice plan was
  // made (its reference: anchor, index and period), each rounded as that
  // plan's own wake was, not as the estimator that lost the lock now has
  // them. A slot whose wake equals the new wake was used, not skipped.
  WakePlan next(const AdapterPhaseEstimator& est, std::int64_t cycle_end_ns,
                std::int64_t now_ns) {
    const std::int64_t t = std::max(cycle_end_ns, now_ns + cfg_.min_gap_ns);
    const bool locked = est.state() == EstimatorState::kLocked;
    WakePlan p;
    p.referenced = locked;
    p.transition = locked != last_.referenced;
    if (locked) {
      std::int64_t n = 0;
      if (last_.referenced && t <= wakeOf(est, last_.target_index + 1)) {
        n = last_.target_index + 1;
      } else {
        n = est.firstIndexAtOrAfter(static_cast<double>(t) + offset());
        if (last_.referenced) {
          n = std::max(n, last_.target_index + 1);
          while (wakeOf(est, n) <= t) ++n;  // late: the first slot after t
        } else {
          while (wakeOf(est, n) < t) ++n;
        }
      }
      p.target_index = n;
      p.wake_ns = wakeOf(est, n);
      p.skipped = last_.referenced
                      ? static_cast<std::uint64_t>(n - last_.target_index - 1)
                      : passedGridSlots(p.wake_ns);
      last_ = p;
      last_ref_ = LatticeRef{est.anchorNs(), est.anchorIndex(), est.periodNs()};
      return p;
    } else {
      const std::int64_t P = cfg_.grid_period_ns;
      std::int64_t g = 0;
      if (!last_.referenced) {
        const std::int64_t deadline = epoch_ + (grid_index_ + 1) * P;
        g = t <= deadline ? grid_index_ + 1 : grid_index_ + 2 + (t - deadline) / P;
      } else {
        g = ceilDiv(t - epoch_, P);
      }
      p.wake_ns = epoch_ + g * P;
      p.skipped = last_.referenced ? passedLatticeSlots(p.wake_ns) : passedGridSlots(p.wake_ns);
      grid_index_ = g;
      last_ = p;
      return p;
    }
  }

  const WakePlan& last() const { return last_; }

 private:
  // A lattice as a plan used it: the estimator's anchor, its index and the
  // period. wakeAt() is the one rounding of a lattice slot to a wake, for the
  // live estimator and for a kept reference alike.
  struct LatticeRef {
    double anchor = 0.0;
    std::int64_t anchor_index = 0;
    double period = 0.0;
  };
  double offset() const { return static_cast<double>(cfg_.lead_ns + cfg_.wake_to_read_ns); }
  std::int64_t wakeAt(const LatticeRef& r, std::int64_t n) const {
    // the same expression as AdapterPhaseEstimator::completionNs
    const double c = r.anchor + static_cast<double>(n - r.anchor_index) * r.period;
    return static_cast<std::int64_t>(std::ceil(c - offset()));
  }
  std::int64_t wakeOf(const AdapterPhaseEstimator& est, std::int64_t n) const {
    return wakeAt(LatticeRef{est.anchorNs(), est.anchorIndex(), est.periodNs()}, n);
  }

  // The grid's slots after the last grid plan that come before `wake`.
  std::uint64_t passedGridSlots(std::int64_t wake) const {
    const std::int64_t m = ceilDiv(wake - epoch_, cfg_.grid_period_ns);
    return static_cast<std::uint64_t>(std::max<std::int64_t>(0, m - 1 - grid_index_));
  }
  // The lattice's slots after the last lattice plan whose wakes, rounded as
  // that plan's were (its kept reference), come before `wake`.
  std::uint64_t passedLatticeSlots(std::int64_t wake) const {
    const std::int64_t first = last_.target_index + 1;
    if (wakeAt(last_ref_, first) >= wake) return 0;
    auto m = last_ref_.anchor_index +
             static_cast<std::int64_t>(std::ceil((static_cast<double>(wake) + offset() - last_ref_.anchor) /
                                                 last_ref_.period));
    if (m <= first) m = first + 1;
    while (wakeAt(last_ref_, m) < wake) ++m;                        // m: the first slot at or after wake
    while (m - 1 > first && wakeAt(last_ref_, m - 1) >= wake) --m;
    return static_cast<std::uint64_t>(m - first);
  }

  PlannerConfig cfg_;
  std::int64_t epoch_;
  std::int64_t grid_index_ = 0;
  WakePlan last_;
  LatticeRef last_ref_;  // the lattice as the last lattice plan used it
};

// --------------------------------------------------------- the accounting

enum class Window : std::uint8_t { kAcquisition, kSettling, kLocked };

struct WindowBounds {
  std::int64_t acquisition_ns = 2000000000;  // [0, acquisition)
  std::int64_t settling_ns = 1000000000;     // [acquisition, + settling)
};                                            // the locked measurement after

struct SkipCounts {
  std::uint64_t cycles = 0;
  std::uint64_t skipped = 0;      // skipped periods
  std::uint64_t overruns = 0;     // cycles with a skip
  std::uint64_t consecutive = 0;  // timing_summary.py's rule: skips of 2 or
                                  // more, plus adjacent overrun cycles
  std::uint64_t referenced_cycles = 0;
  std::uint64_t grid_cycles = 0;
};

class SkipAccounting {
 public:
  SkipAccounting(std::int64_t epoch_ns, WindowBounds bounds)
      : epoch_(epoch_ns), bounds_(bounds) {}

  Window windowOf(std::int64_t scheduled_wake_ns) const {
    const std::int64_t t = scheduled_wake_ns - epoch_;
    if (t < bounds_.acquisition_ns) return Window::kAcquisition;
    if (t < bounds_.acquisition_ns + bounds_.settling_ns) return Window::kSettling;
    return Window::kLocked;
  }

  // One cycle: its scheduled wake, the skip its close produced, its schedule.
  void add(std::int64_t scheduled_wake_ns, std::uint64_t skipped, bool referenced) {
    const Window w = windowOf(scheduled_wake_ns);
    const bool overran = skipped > 0;
    addTo(whole_, skipped, referenced, overran && prev_overran_);
    auto& c = windows_[static_cast<std::size_t>(w)];
    addTo(c, skipped, referenced, overran && prev_overran_ && prev_window_ == w);
    prev_overran_ = overran;
    prev_window_ = w;
  }

  const SkipCounts& window(Window w) const { return windows_[static_cast<std::size_t>(w)]; }
  const SkipCounts& wholeRun() const { return whole_; }

 private:
  static void addTo(SkipCounts& c, std::uint64_t skipped, bool referenced, bool adjacent) {
    ++c.cycles;
    c.skipped += skipped;
    if (skipped > 0) ++c.overruns;
    if (skipped >= 2) ++c.consecutive;
    if (adjacent) ++c.consecutive;
    if (referenced) ++c.referenced_cycles;
    else ++c.grid_cycles;
  }

  std::int64_t epoch_;
  WindowBounds bounds_;
  std::array<SkipCounts, 3> windows_{};
  SkipCounts whole_;
  bool prev_overran_ = false;
  Window prev_window_ = Window::kAcquisition;
};

// T2 as frozen (loaded): at most per_10k skipped periods per 10^4 cycles of
// the run, never two in a row. A whole-run criterion: give it the whole run's
// counts, not a window's.
struct T2Verdict {
  double allowed = 0.0;
  bool pass = false;
};
inline T2Verdict t2Loaded(const SkipCounts& whole_run, double per_10k = 1.0) {
  T2Verdict v;
  v.allowed = per_10k * static_cast<double>(whole_run.cycles) / 1e4;
  v.pass = static_cast<double>(whole_run.skipped) <= v.allowed && whole_run.consecutive == 0;
  return v;
}

// ------------------------------------------------- the three, per cycle

struct ScheduleConfig {
  EstimatorConfig estimator;
  PlannerConfig planner;
  WindowBounds windows;
};

class ReferencedSchedule {
 public:
  ReferencedSchedule(ScheduleConfig cfg, std::int64_t epoch_ns)
      : cfg_(cfg),
        est_(cfg.estimator),
        planner_(cfg.planner, epoch_ns),
        accounting_(epoch_ns, cfg.windows),
        acquisition_end_(epoch_ns + cfg.windows.acquisition_ns) {}

  WakePlan first() {
    current_ = planner_.first();
    return current_;
  }

  // After each cycle: its reply stamp (kInvalidStamp if none), when it ended,
  // and now (>= cycle_end). Observes the stamp as the state allows, ends an
  // acquisition window that has run out, plans the next wake and counts the
  // cycle under its own schedule.
  WakePlan closeCycle(std::int64_t reply_stamp_ns, std::int64_t cycle_end_ns,
                      std::int64_t now_ns) {
    if (est_.state() == EstimatorState::kAcquiring) {
      if (reply_stamp_ns >= 0) est_.addAcquisitionStamp(reply_stamp_ns);
    } else if (current_.referenced) {
      est_.observeIntended(reply_stamp_ns, current_.target_index);
      if (est_.state() == EstimatorState::kAcquiring) {
        acquisition_end_ = cycle_end_ns + cfg_.windows.acquisition_ns;  // reacquire
      }
    }
    if (est_.state() == EstimatorState::kAcquiring && cycle_end_ns >= acquisition_end_) {
      if (!est_.finishAcquisition(acquisition_end_)) {
        acquisition_end_ = cycle_end_ns + cfg_.windows.acquisition_ns;
      }
    }
    const WakePlan next = planner_.next(est_, cycle_end_ns, now_ns);
    accounting_.add(current_.wake_ns, next.skipped, current_.referenced);
    current_ = next;
    return next;
  }

  const AdapterPhaseEstimator& estimator() const { return est_; }
  const SkipAccounting& accounting() const { return accounting_; }
  const WakePlan& current() const { return current_; }

 private:
  ScheduleConfig cfg_;
  AdapterPhaseEstimator est_;
  WakePlanner planner_;
  SkipAccounting accounting_;
  std::int64_t acquisition_end_;
  WakePlan current_;
};

}  // namespace rtctrl::hw::adapter
