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
//   - FitExchange: the acquisition stamps and their fit, handed between the
//     real-time worker and a non-real-time fitter by explicit ownership, so
//     that the fit (some ms, allocating) never runs on the worker;
//   - ReferencedSchedule: the three together, per cycle in two phases: plan
//     provisionally at the cycle's end, then recheck the clock and publish,
//     counting the cycle exactly once at publication.
//
// Times are int64 nanoseconds on one monotonic clock; lattice positions are
// doubles in the same nanoseconds (0.06 ns resolution at 4e14 ns).
//
// Log encoding, for the probe's per-cycle log (integers only: its reader
// converts every column but command_kind to an integer):
//   target_index             the intended completion, fixed at publication;
//                            -1 on the grid
//   schedule                 Schedule: 0 grid, 1 lattice
//   lock_state               LockState at publication: 0 acquiring, 1 fit
//                            pending, 2 locked
//   transition               1 if the schedule changed at this plan, else 0
//   observation              Observation of the cycle's own reply: 0 none,
//                            1 stored, 2 dropped, 3 gated, 4 late, 5 early,
//                            6 invalid
//   predicted_completion_ns  the intended completion on the plan's lattice at
//                            publication, rounded to ns; -1 on the grid

#include <algorithm>
#include <array>
#include <atomic>
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
  kNone = 0,
  kOutsideGate = 1,
  kNoGatedStamp = 2,
  kPeriodClamp = 3,
  kNonFinite = 4,
};

// The values are the log's encoding (above); they do not change.
enum class Observation : std::uint8_t {
  kNone = 0,     // no stamp used this cycle
  kStored = 1,   // acquisition: kept for the fit
  kDropped = 2,  // acquisition: no buffer free, or the buffer is full
  kGated = 3,    // tracking: within the gate, the anchor stepped
  kLate = 4,     // tracking: after the gate (receive delay, or a later completion)
  kEarly = 5,    // tracking: before the gate (the estimate is late)
  kInvalid = 6,  // no stamp, out of order, or not possible for its cycle
};

enum class Schedule : std::uint8_t { kGrid = 0, kLattice = 1 };

enum class LockState : std::uint8_t { kAcquiring = 0, kFitPending = 1, kLocked = 2 };

struct EstimatorCounts {
  std::uint64_t stored = 0, dropped = 0, gated = 0, late = 0, early = 0,
                invalid = 0, locks = 0, failed_acquisitions = 0, losses = 0;
  // the losses by cause
  std::uint64_t lost_outside_gate = 0, lost_no_gated = 0, lost_period_clamp = 0,
                lost_non_finite = 0;
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
    const LatticeFit fit = fitLattice(stamps_.data(), stamps_.size(), cfg_.fit);
    stamps_.clear();
    return applyFit(fit, at_ns);
  }

  // Locks on a fit made elsewhere (FitExchange's fitter, on the stamps the
  // worker handed over) if it qualifies: enough stamps, a slope within the
  // limit, a share; at_ns is the end of the window it was fitted on. O(1);
  // never allocates.
  bool applyFit(const LatticeFit& fit, std::int64_t at_ns) {
    if (state_ != EstimatorState::kAcquiring) return false;
    last_fit_ = fit;
    const bool ok = fit.stamps >= cfg_.lock_min_stamps &&
                    std::fabs(fit.slope_ppm) <= cfg_.lock_slope_limit_ppm &&
                    fit.share >= cfg_.lock_min_share;
    if (!ok) {
      ++counts_.failed_acquisitions;
      return false;
    }
    lockFrom(fit.slope_ppm, fit.phase0_us, at_ns);
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
    switch (cause) {
      case LossCause::kOutsideGate: ++counts_.lost_outside_gate; break;
      case LossCause::kNoGatedStamp: ++counts_.lost_no_gated; break;
      case LossCause::kPeriodClamp: ++counts_.lost_period_clamp; break;
      case LossCause::kNonFinite: ++counts_.lost_non_finite; break;
      case LossCause::kNone: break;
    }
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
  std::uint64_t skipped = 0;  // slots passed without a cycle (see WakePlanner)
  bool referenced = false;    // on the lattice
  bool transition = false;    // the schedule changed at this plan
  // The intended completion on the plan's own lattice, rounded; -1 on the grid.
  std::int64_t predicted_completion_ns = -1;
  // Of skipped: the plan's own slots passed when its publication rechecked the
  // clock (a scheduler delay between planning and publishing, kept visible).
  std::uint64_t recheck_skipped = 0;
  LockState lock_state = LockState::kAcquiring;  // set at publication
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
      const LatticeRef ref{est.anchorNs(), est.anchorIndex(), est.periodNs()};
      std::int64_t n = 0;
      if (last_.referenced && t <= wakeAt(ref, last_.target_index + 1)) {
        n = last_.target_index + 1;
      } else {
        n = est.firstIndexAtOrAfter(static_cast<double>(t) + offset());
        if (last_.referenced) {
          n = std::max(n, last_.target_index + 1);
          while (wakeAt(ref, n) <= t) ++n;  // late: the first slot after t
        } else {
          while (wakeAt(ref, n) < t) ++n;
        }
      }
      p.target_index = n;
      p.wake_ns = wakeAt(ref, n);
      p.predicted_completion_ns = std::llround(completionAt(ref, n));
      p.skipped = last_.referenced
                      ? static_cast<std::uint64_t>(n - last_.target_index - 1)
                      : passedGridSlots(p.wake_ns);
      last_ = p;
      last_ref_ = ref;
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

  // The publication's recheck of the last plan against the clock: if its wake
  // is before now + min_gap, the plan moves to its own schedule's first slot
  // at or after now + min_gap (the lattice as the plan used it), and the
  // slots it passes are added to skipped and recorded as recheck_skipped. A
  // wake exactly at now + min_gap stands.
  WakePlan recheck(std::int64_t now_ns) {
    const std::int64_t t = now_ns + cfg_.min_gap_ns;
    WakePlan p = last_;
    p.recheck_skipped = 0;
    if (p.wake_ns >= t) {
      last_ = p;
      return p;
    }
    std::int64_t extra = 0;
    if (p.referenced) {
      std::int64_t n = p.target_index + 1;
      auto m = last_ref_.anchor_index +
               static_cast<std::int64_t>(std::ceil((static_cast<double>(t) + offset() - last_ref_.anchor) /
                                                   last_ref_.period));
      n = std::max(n, m);
      while (wakeAt(last_ref_, n) < t) ++n;
      while (n - 1 > p.target_index && wakeAt(last_ref_, n - 1) >= t) --n;
      extra = n - p.target_index;
      p.target_index = n;
      p.wake_ns = wakeAt(last_ref_, n);
      p.predicted_completion_ns = std::llround(completionAt(last_ref_, n));
    } else {
      const std::int64_t g = ceilDiv(t - epoch_, cfg_.grid_period_ns);
      extra = g - grid_index_;
      grid_index_ = g;
      p.wake_ns = epoch_ + g * cfg_.grid_period_ns;
    }
    p.skipped += static_cast<std::uint64_t>(extra);
    p.recheck_skipped = static_cast<std::uint64_t>(extra);
    last_ = p;
    return p;
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
  static double completionAt(const LatticeRef& r, std::int64_t n) {
    // the same expression as AdapterPhaseEstimator::completionNs
    return r.anchor + static_cast<double>(n - r.anchor_index) * r.period;
  }
  std::int64_t wakeAt(const LatticeRef& r, std::int64_t n) const {
    return static_cast<std::int64_t>(std::ceil(completionAt(r, n) - offset()));
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

// ------------------------------------------------------ the fit exchange

// The acquisition's stamps and their fit, handed between the worker (the
// real-time thread: it appends stamps, queues a buffer and takes results)
// and the fitter (a non-real-time thread: it claims a queued buffer, fits it
// and completes it). Two buffers, each with an owner, and only a buffer's
// owner touches its stamps and its result:
//
//   kWorker   the worker's: it appends stamps; it may queue the buffer
//   kQueued   nobody's: the payload is frozen; the fitter may claim it
//   kFitting  the fitter's: it reads the stamps and writes the result
//   kDone     the worker's to take: it reads the result, clears the buffer
//             and returns it to kWorker
//
// Every change of owner is made by the current owner, as a release store (the
// fitter's claim as an acquire-release compare-exchange), so the next owner,
// loading the state with acquire order, sees the payload complete; no thread
// touches a payload it does not own. The claim is the fitter's
// acknowledgment of a request, the return to kWorker the worker's
// acknowledgment of a result. A queued buffer carries the worker's request
// generation, copied into its result, so that a result the worker has since
// abandoned is recognised and discarded.
//
// One worker thread and one fitter thread. The buffers are allocated at
// construction; the worker's calls never allocate; the fitter's fit does.
struct FitResult {
  std::uint64_t generation = 0;
  std::int64_t at_ns = 0;  // the end of the window fitted
  LatticeFit fit;          // fit.stamps: the buffer's stamps
};

class FitExchange {
 public:
  enum class Owner : std::uint8_t { kWorker = 0, kQueued = 1, kFitting = 2, kDone = 3 };

  struct Counts {
    std::uint64_t appended = 0, dropped = 0, queued = 0, taken = 0;  // the worker's
    std::uint64_t fitted = 0;                                        // the fitter's
  };

  explicit FitExchange(std::size_t capacity = 4096, LatticeFitConfig fit = {})
      : fit_cfg_(fit) {
    for (auto& s : slots_) s.stamps.assign(capacity, 0);
  }
  FitExchange(const FitExchange&) = delete;
  FitExchange& operator=(const FitExchange&) = delete;

  // ---- the worker ----

  // Appends a stamp to the worker's buffer; false (counted as dropped) if no
  // buffer is the worker's or it is full.
  bool append(std::int64_t stamp) {
    const int i = workerSlot();
    if (i < 0 || slots_[static_cast<std::size_t>(i)].count >= slots_[static_cast<std::size_t>(i)].stamps.size()) {
      ++worker_counts_.dropped;
      return false;
    }
    Slot& s = slots_[static_cast<std::size_t>(i)];
    s.stamps[s.count++] = stamp;
    ++worker_counts_.appended;
    return true;
  }

  // Discards the worker's buffer's stamps (a restarted acquisition).
  void clearWorkerBuffer() {
    const int i = workerSlot();
    if (i >= 0) slots_[static_cast<std::size_t>(i)].count = 0;
  }

  std::size_t workerBufferCount() const {
    const int i = workerSlotConst();
    return i < 0 ? 0 : slots_[static_cast<std::size_t>(i)].count;
  }

  // Freezes the worker's buffer and queues it for the fitter, with the end of
  // its window and the request's generation; false if no buffer is the
  // worker's or it is empty.
  bool queue(std::int64_t at_ns, std::uint64_t generation) {
    const int i = workerSlot();
    if (i < 0 || slots_[static_cast<std::size_t>(i)].count == 0) return false;
    Slot& s = slots_[static_cast<std::size_t>(i)];
    s.at_ns = at_ns;
    s.generation = generation;
    s.owner.store(Owner::kQueued, std::memory_order_release);
    active_ = -1;
    ++worker_counts_.queued;
    return true;
  }

  // Takes a completed result, if any: copies it out, clears its buffer and
  // returns the buffer to the worker.
  bool take(FitResult* out) {
    for (std::size_t i = 0; i < slots_.size(); ++i) {
      Slot& s = slots_[i];
      if (s.owner.load(std::memory_order_acquire) != Owner::kDone) continue;
      out->generation = s.generation;
      out->at_ns = s.at_ns;
      out->fit = s.fit;
      s.count = 0;
      s.owner.store(Owner::kWorker, std::memory_order_release);
      ++worker_counts_.taken;
      return true;
    }
    return false;
  }

  const Counts& workerCounts() const { return worker_counts_; }

  // ---- the fitter ----

  // Claims a queued buffer, fits it and completes it; false if none was
  // queued. Allocates (fitLattice): never call it on the worker.
  bool service() {
    for (auto& s : slots_) {
      Owner expected = Owner::kQueued;
      if (!s.owner.compare_exchange_strong(expected, Owner::kFitting, std::memory_order_acq_rel,
                                           std::memory_order_acquire)) {
        continue;
      }
      s.fit = fitLattice(s.stamps.data(), s.count, fit_cfg_);
      fitted_.fetch_add(1, std::memory_order_relaxed);
      s.owner.store(Owner::kDone, std::memory_order_release);
      return true;
    }
    return false;
  }

  std::uint64_t fitted() const { return fitted_.load(std::memory_order_relaxed); }

  // ---- either thread, for inspection ----
  Owner owner(std::size_t i) const { return slots_[i].owner.load(std::memory_order_acquire); }

 private:
  struct Slot {
    std::atomic<Owner> owner{Owner::kWorker};
    std::vector<std::int64_t> stamps;  // sized at construction, never resized
    std::size_t count = 0;
    std::int64_t at_ns = 0;
    std::uint64_t generation = 0;
    LatticeFit fit;
  };

  // The worker's buffer: the one it last appended to while it stays the
  // worker's, else the first buffer the worker owns; -1 if none.
  int workerSlot() {
    if (active_ >= 0 &&
        slots_[static_cast<std::size_t>(active_)].owner.load(std::memory_order_acquire) == Owner::kWorker) {
      return active_;
    }
    active_ = workerSlotConst();
    return active_;
  }
  int workerSlotConst() const {
    if (active_ >= 0 &&
        slots_[static_cast<std::size_t>(active_)].owner.load(std::memory_order_acquire) == Owner::kWorker) {
      return active_;
    }
    for (std::size_t i = 0; i < slots_.size(); ++i) {
      if (slots_[i].owner.load(std::memory_order_acquire) == Owner::kWorker) return static_cast<int>(i);
    }
    return -1;
  }

  LatticeFitConfig fit_cfg_;
  std::array<Slot, 2> slots_;
  int active_ = 0;
  Counts worker_counts_;
  std::atomic<std::uint64_t> fitted_{0};
};

// ------------------------------------------------- the three, per cycle

struct ScheduleConfig {
  EstimatorConfig estimator;
  PlannerConfig planner;
  WindowBounds windows;
  // A fit is applied only within this of its window's end, and a request
  // older than this is abandoned (provisional diagnostic value, 2026-11-20).
  std::int64_t fit_max_age_ns = 500000000;
};

struct ScheduleCounts {
  std::uint64_t fits_requested = 0, fits_applied = 0, fits_refused = 0;
  std::uint64_t stale_results = 0;      // arrived after fit_max_age_ns
  std::uint64_t abandoned_requests = 0;  // no result within fit_max_age_ns
  std::uint64_t old_generation_results = 0;  // results of abandoned requests
  std::uint64_t queue_failures = 0;     // no stamps, or no buffer the worker's
  std::uint64_t protocol_errors = 0;    // a call out of turn (see ReferencedSchedule)
};

// Where a schedule is in its calls: first() starts it (kUnstarted to
// kPublished); closeCycle() moves kPublished to kProvisional; publish() moves
// kProvisional to kPublished. Any other call changes nothing and counts a
// protocol error.
enum class SchedulePhase : std::uint8_t { kUnstarted = 0, kPublished = 1, kProvisional = 2 };

class ReferencedSchedule {
 public:
  // exchange: the fit exchange shared with a fitter thread; null fits on the
  // calling thread at the window's end (tests and offline use only).
  ReferencedSchedule(ScheduleConfig cfg, std::int64_t epoch_ns, FitExchange* exchange = nullptr)
      : cfg_(cfg),
        est_(cfg.estimator),
        planner_(cfg.planner, epoch_ns),
        accounting_(epoch_ns, cfg.windows),
        exchange_(exchange),
        acquisition_end_(epoch_ns + cfg.windows.acquisition_ns) {}

  // The first cycle's plan, published: once, before anything else. Called
  // again it changes nothing and counts a protocol error.
  const WakePlan& first() {
    if (phase_ != SchedulePhase::kUnstarted) {
      ++counts_.protocol_errors;
      return current_;
    }
    current_ = planner_.first();
    current_.lock_state = lockState();
    phase_ = SchedulePhase::kPublished;
    return current_;
  }

  // Phase 1, at a cycle's end: observes its reply stamp (kInvalidStamp if
  // none) against the published plan's intended completion, hands the
  // acquisition over and takes fit results, and plans the next wake
  // provisionally. Nothing is counted until publish(). Its decisions about
  // time (a fit's freshness, a request's timeout, a restarted acquisition
  // window) are taken at max(now, cycle end), now being when the call runs:
  // a delay before the call makes a result older, never younger. Out of
  // turn (unstarted, or before the last plan was published) it changes
  // nothing and counts a protocol error.
  const WakePlan& closeCycle(std::int64_t reply_stamp_ns, std::int64_t cycle_end_ns, std::int64_t now_ns) {
    if (phase_ != SchedulePhase::kPublished) {
      ++counts_.protocol_errors;
      return phase_ == SchedulePhase::kProvisional ? provisional_ : current_;
    }
    const std::int64_t decision_ns = std::max(now_ns, cycle_end_ns);
    observation_ = Observation::kNone;
    if (est_.state() == EstimatorState::kAcquiring) {
      if (!pending_ && reply_stamp_ns >= 0) {
        if (exchange_ != nullptr) {
          observation_ = exchange_->append(reply_stamp_ns) ? Observation::kStored : Observation::kDropped;
        } else {
          observation_ = est_.addAcquisitionStamp(reply_stamp_ns);
        }
      }
    } else if (current_.referenced) {
      observation_ = est_.observeIntended(reply_stamp_ns, current_.target_index);
      if (est_.state() == EstimatorState::kAcquiring) restartAcquisition(decision_ns);  // lost
    }
    if (exchange_ != nullptr) takeResults(decision_ns);
    if (est_.state() == EstimatorState::kAcquiring && !pending_ && cycle_end_ns >= acquisition_end_) {
      if (exchange_ != nullptr) {
        if (exchange_->queue(acquisition_end_, generation_ + 1)) {
          ++generation_;
          pending_ = true;
          ++counts_.fits_requested;
        } else {
          ++counts_.queue_failures;
          restartAcquisition(decision_ns);
        }
      } else if (!est_.finishAcquisition(acquisition_end_)) {
        restartAcquisition(decision_ns);
      }
    }
    provisional_ = planner_.next(est_, cycle_end_ns, decision_ns);
    phase_ = SchedulePhase::kProvisional;
    return provisional_;
  }

  // Phase 2: rechecks the provisional plan against now (the clock read just
  // before publishing), counts the cycle that ended, exactly once, with the
  // skips it produced, and publishes the plan. Out of turn it changes nothing
  // and counts a protocol error.
  const WakePlan& publish(std::int64_t now_ns) {
    if (phase_ != SchedulePhase::kProvisional) {
      ++counts_.protocol_errors;
      return current_;
    }
    WakePlan p = planner_.recheck(now_ns);
    p.lock_state = lockState();
    accounting_.add(current_.wake_ns, p.skipped, current_.referenced);
    current_ = p;
    phase_ = SchedulePhase::kPublished;
    return current_;
  }

  // Both phases at once (tests; the integration rereads the clock between).
  // Only where closeCycle() would be accepted; otherwise it changes nothing
  // and counts one protocol error (it never publishes a plan left pending).
  const WakePlan& closeAndPublish(std::int64_t reply_stamp_ns, std::int64_t cycle_end_ns, std::int64_t now_ns) {
    if (phase_ != SchedulePhase::kPublished) {
      ++counts_.protocol_errors;
      return phase_ == SchedulePhase::kProvisional ? provisional_ : current_;
    }
    closeCycle(reply_stamp_ns, cycle_end_ns, now_ns);
    return publish(now_ns);
  }

  LockState lockState() const {
    if (est_.state() == EstimatorState::kLocked) return LockState::kLocked;
    return pending_ ? LockState::kFitPending : LockState::kAcquiring;
  }
  Observation lastObservation() const { return observation_; }
  const AdapterPhaseEstimator& estimator() const { return est_; }
  const SkipAccounting& accounting() const { return accounting_; }
  const ScheduleCounts& counts() const { return counts_; }
  const WakePlan& current() const { return current_; }
  SchedulePhase phase() const { return phase_; }
  bool published() const { return phase_ == SchedulePhase::kPublished; }

 private:
  void restartAcquisition(std::int64_t at_ns) {
    acquisition_end_ = at_ns + cfg_.windows.acquisition_ns;
    if (exchange_ != nullptr) exchange_->clearWorkerBuffer();
  }

  void takeResults(std::int64_t now_ns) {
    FitResult r;
    while (exchange_->take(&r)) {
      if (!pending_ || r.generation != generation_) {
        ++counts_.old_generation_results;
        continue;
      }
      pending_ = false;
      if (now_ns - r.at_ns > cfg_.fit_max_age_ns) {
        ++counts_.stale_results;
        restartAcquisition(now_ns);
      } else if (est_.applyFit(r.fit, r.at_ns)) {
        ++counts_.fits_applied;
      } else {
        ++counts_.fits_refused;
        restartAcquisition(now_ns);
      }
    }
    if (pending_ && now_ns - acquisition_end_ > cfg_.fit_max_age_ns) {
      // no result in time: abandon the request; a late result is discarded
      pending_ = false;
      ++counts_.abandoned_requests;
      restartAcquisition(now_ns);
    }
  }

  ScheduleConfig cfg_;
  AdapterPhaseEstimator est_;
  WakePlanner planner_;
  SkipAccounting accounting_;
  FitExchange* exchange_;
  std::int64_t acquisition_end_;
  std::uint64_t generation_ = 0;
  bool pending_ = false;
  SchedulePhase phase_ = SchedulePhase::kUnstarted;
  Observation observation_ = Observation::kNone;
  WakePlan current_;
  WakePlan provisional_;
  ScheduleCounts counts_;
};

}  // namespace rtctrl::hw::adapter
