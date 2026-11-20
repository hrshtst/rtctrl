#pragma once

// x7_bus_timing's adapter-referenced timing (--timing adapter): the choices,
// their combination rules, and the worker's per-cycle driver of
// hw::adapter::ReferencedSchedule with the records the probe logs.
//
// Grid timing stays the default and does not use this file. Under adapter
// timing the worker (the real-time probe thread) plans each wake from the
// adapter's completion lattice once locked, and from the probe's own 1 ms grid
// before that; the acquisition's fit runs on the probe's main thread, which is
// not real-time, through hw::adapter::FitExchange. The lead alternation, when
// chosen, is a check of the lock and only that: its coupling result is logged
// and summarized, and nothing in the probe (its schedule, its stop rule, its
// exit status) reads it.
//
// Times here are nanoseconds from the run's epoch, the origin of every *_ns
// stamp in the probe's log: the schedule is built with epoch 0, the reply
// stamps it observes are the log's last_reply_ns, and its wakes are converted
// back to the steady clock by adding the epoch.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "rtctrl/hw/adapter_phase.hpp"
#include "rtctrl/hw/cycle_record.hpp"

namespace rtctrl::apps::bus_timing {

namespace ad = rtctrl::hw::adapter;

// The stamp a cycle gives the schedule: its reply's last bytes
// (last_reply_ns), from a complete read only. first_reply_ns marks the
// first servo's reply, which a reply split over two completions can carry
// in an earlier one (the integration design's stamp choice, 2026-11-20).
inline std::int64_t replyStamp(const rtctrl::hw::CycleRecord& c) {
  return c.read_ok && c.last_reply_ns >= 0 ? c.last_reply_ns : ad::kInvalidStamp;
}

// The reviewed provisional diagnostic values (2026-11-20), fixed, with no
// flags: neither is a demonstrated bound.
inline constexpr std::int64_t kAdapterPeriodNs = 1000000;   // the probe's grid, and the lattice's nominal period
inline constexpr std::int64_t kPublicationMarginNs = 20000;  // no published wake before now + 20 us
inline constexpr std::int64_t kFitMaxAgeNs = 500000000;      // a fit applies within 500 ms of its window's end

// The probe's timing choices as given on its command line, for the rules
// below (checked before the probe opens the port).
struct TimingChoice {
  bool adapter = false;               // --timing adapter
  bool adapter_values_given = false;  // --lead-us, --wake-to-read-us, --acquisition-s or --settling-s
  bool alternation_given = false;     // --lead-alternation-us
  double rate_hz = 0.0;
  double duration_s = 0.0;
  bool calibration = false;  // --confirm-writes or --drain
  bool bounded = false;      // --read-path bounded
  bool motion_only = false;  // --feedback motion
  // Cycles per full-window read: 1 every cycle, n every n-th, 0 none after the
  // initial read (Probe::fullReadEvery).
  std::uint64_t full_every = 1;
  double lead_us = 780.0;
  double wake_to_read_us = 10.0;
  double acquisition_s = 2.0;
  double settling_s = 1.0;
  double alternation_us = 0.0;
};

// Why the choice is not supported, or empty if it is.
inline std::string timingChoiceError(const TimingChoice& c) {
  if (!c.adapter) {
    if (c.adapter_values_given || c.alternation_given) {
      return "--lead-us, --wake-to-read-us, --acquisition-s, --settling-s and --lead-alternation-us need "
             "--timing adapter";
    }
    return "";
  }
  if (c.rate_hz != 1000.0) {
    return "--timing adapter runs at --rate-hz 1000 only (the adapter's 1 ms completion cadence)";
  }
  if (c.calibration) {
    return "--timing adapter runs production traffic only (no --confirm-writes or --drain)";
  }
  if (!c.bounded) return "--timing adapter needs --read-path bounded";
  if (!(c.lead_us > 0.0 && c.lead_us < 1000.0)) return "--lead-us must be within (0, 1000)";
  if (!(c.wake_to_read_us >= 0.0 && c.wake_to_read_us <= 100.0)) {
    return "--wake-to-read-us must be within [0, 100]";
  }
  if (!(c.alternation_us >= 0.0 && c.alternation_us <= 100.0)) {
    return "--lead-alternation-us must be within [0, 100]";
  }
  if (!(c.lead_us - c.alternation_us > 0.0 && c.lead_us + c.alternation_us + c.wake_to_read_us < 1000.0)) {
    return "the lead less the alternation must stay above 0 and the lead, the alternation and the wake-to-read "
           "allowance together below 1000 us (one period)";
  }
  if (!(c.acquisition_s > 0.0 && c.acquisition_s <= 4.0)) {
    return "--acquisition-s must be within (0, 4] (the acquisition holds 4,096 stamps)";
  }
  if (!(c.settling_s >= 0.0 && c.settling_s <= 60.0)) return "--settling-s must be within [0, 60]";
  if (!(c.acquisition_s + c.settling_s < c.duration_s)) {
    return "the acquisition and settling windows must end before --duration";
  }
  if (c.alternation_us > 0.0 && !(c.motion_only && c.full_every != 1)) {
    return "--lead-alternation-us needs motion-only reads (--feedback motion, with full-window reads less often "
           "than every cycle): full-window reads stay at the base lead";
  }
  return "";
}

inline std::int64_t usToNs(double us) { return std::llround(us * 1e3); }
inline std::int64_t sToNs(double s) { return std::llround(s * 1e9); }

// The schedule's configuration for a supported choice: the probe's 1 ms grid,
// the lead and allowance, the declared windows, the reviewed margin and
// freshness limit, and the alternation's magnitude (0: none). The estimator's
// and the detector's own parameters stay as tested (no flags).
inline ad::ScheduleConfig scheduleConfig(const TimingChoice& c) {
  ad::ScheduleConfig s;
  s.estimator.nominal_period_ns = kAdapterPeriodNs;
  s.planner.grid_period_ns = kAdapterPeriodNs;
  s.planner.lead_ns = usToNs(c.lead_us);
  s.planner.wake_to_read_ns = usToNs(c.wake_to_read_us);
  s.planner.min_gap_ns = kPublicationMarginNs;
  s.windows.acquisition_ns = sToNs(c.acquisition_s);
  s.windows.settling_ns = sToNs(c.settling_s);
  s.fit_max_age_ns = kFitMaxAgeNs;
  s.lead_alternation_ns = usToNs(c.alternation_us);
  return s;
}

// The adapter columns of one log row. The first seven are the plan the row's
// read executed, as published before its wake; the last two are its close:
// the observation of its reply against that plan, and of the skips its close
// produced (the row's skipped_periods) the slots the publication's recheck
// added. Integers only (see the encoding in rtctrl/hw/adapter_phase.hpp).
struct AdapterRow {
  std::int64_t target_index = -1;
  std::uint8_t schedule = 0;
  std::uint8_t lock_state = 0;
  std::uint8_t transition = 0;
  std::int64_t predicted_completion_ns = -1;
  std::int64_t lead_offset_ns = 0;
  std::uint8_t coupling_state = 0;
  std::uint8_t observation = 0;
  std::uint64_t recheck_skipped = 0;
};

// Appended to the probe's header under adapter timing (grid logs keep theirs).
inline constexpr const char* kAdapterColumns =
    "target_index,schedule,lock_state,transition,predicted_completion_ns,lead_offset_ns,coupling_state,"
    "observation,recheck_skipped";

// One lock: from the close that applied its fit to the close that lost it, or
// to the run's end.
struct LockRecord {
  std::uint64_t locked_cycle = 0;  // the cycle (the log's number) whose close locked
  std::int64_t locked_ns = 0;      // that close's decision time
  std::uint64_t ended_cycle = 0;   // the cycle whose close lost it; 0: held to the run's end
  std::int64_t ended_ns = -1;
  ad::LossCause cause = ad::LossCause::kNone;  // kNone: held to the run's end
  std::uint64_t referenced_cycles = 0;         // the cycles its lattice planned that ran
  // The coupling detector (with the alternation): its state when the lock
  // ended or the run did, and its first declaration within the lock (0: none)
  // with the lock's referenced cycles by then.
  std::uint8_t coupling_at_end = 0;
  std::uint8_t first_declaration = 0;
  std::uint64_t first_declaration_after = 0;
};

// A fit event as the worker saw it, with the cycle whose close it was.
struct FitRecord {
  std::uint64_t cycle = 0;
  ad::FitEvent event;
};

// The worker's driver. Construct it before the run (it allocates there, as
// the schedule does); after that close(), publish() and finish() never
// allocate. Its record arrays are bounded: beyond them it counts what it could
// not keep.
class AdapterCycles {
 public:
  static constexpr std::size_t kMaxLocks = 256;
  static constexpr std::size_t kMaxFits = 512;

  AdapterCycles(const ad::ScheduleConfig& cfg, ad::FitExchange* exchange) : schedule_(cfg, 0, exchange) {}

  // The first cycle's wake, at the epoch.
  std::int64_t first() { return schedule_.first().wake_ns; }

  // At the end of the run's cycle number `cycle`: observes its reply stamp
  // (ad::kInvalidStamp for none) against the plan its read executed, and plans
  // the next cycle provisionally. executed and next are the read classes of
  // this cycle and of the next (from the probe's full-read counter).
  void close(std::uint64_t cycle, std::int64_t stamp, std::int64_t cycle_end_ns, std::int64_t now_ns,
             ad::ReadClass executed, ad::ReadClass next) {
    used_ = schedule_.current();
    const bool was_locked = schedule_.estimator().state() == ad::EstimatorState::kLocked;
    const auto coupling_before = static_cast<std::uint8_t>(schedule_.coupling().state());
    if (open_ && used_.referenced) ++locks_[open_index_].referenced_cycles;
    schedule_.closeCycle(stamp, cycle_end_ns, now_ns, executed, next);
    const std::int64_t decision_ns = std::max(now_ns, cycle_end_ns);
    for (std::size_t i = 0; i < schedule_.fitEventCount(); ++i) {
      if (fit_count_ < kMaxFits) {
        fits_[fit_count_++] = FitRecord{cycle, schedule_.fitEvent(i)};
      } else {
        ++fits_dropped_;
      }
    }
    const bool locked = schedule_.estimator().state() == ad::EstimatorState::kLocked;
    if (was_locked && !locked && open_) {  // lost at this close
      LockRecord& l = locks_[open_index_];
      l.ended_cycle = cycle;
      l.ended_ns = decision_ns;
      l.cause = schedule_.estimator().lossCause();
      l.coupling_at_end = coupling_before;  // the detector resets on the loss
      open_ = false;
    }
    if (!was_locked && locked) {  // a new lock
      if (lock_count_ < kMaxLocks) {
        open_index_ = lock_count_++;
        locks_[open_index_] = LockRecord{};
        locks_[open_index_].locked_cycle = cycle;
        locks_[open_index_].locked_ns = decision_ns;
        open_ = true;
      } else {
        ++locks_dropped_;
      }
    }
  }

  // Rechecks the provisional plan at now_ns (the clock read again just
  // before), publishes it, and fills `row` for the cycle just closed.
  const ad::WakePlan& publish(std::int64_t now_ns, AdapterRow* row) {
    const ad::WakePlan& p = schedule_.publish(now_ns);
    row->target_index = used_.target_index;
    row->schedule = static_cast<std::uint8_t>(used_.referenced ? ad::Schedule::kLattice : ad::Schedule::kGrid);
    row->lock_state = static_cast<std::uint8_t>(used_.lock_state);
    row->transition = used_.transition ? 1 : 0;
    row->predicted_completion_ns = used_.predicted_completion_ns;
    row->lead_offset_ns = used_.lead_offset_ns;
    row->coupling_state = used_.coupling_state;
    row->observation = static_cast<std::uint8_t>(schedule_.lastObservation());
    row->recheck_skipped = p.recheck_skipped;
    if (open_ && p.coupling_state != 0 && locks_[open_index_].first_declaration == 0) {
      locks_[open_index_].first_declaration = p.coupling_state;
      locks_[open_index_].first_declaration_after = locks_[open_index_].referenced_cycles;
    }
    return p;
  }

  // At the run's end: the lock still held, if any, keeps ended_cycle 0 and
  // takes the detector's state at the end.
  void finish() {
    if (open_) locks_[open_index_].coupling_at_end = static_cast<std::uint8_t>(schedule_.coupling().state());
  }

  const ad::ReferencedSchedule& schedule() const { return schedule_; }
  std::size_t lockCount() const { return lock_count_; }
  const LockRecord& lock(std::size_t i) const { return locks_[i]; }
  std::uint64_t locksDropped() const { return locks_dropped_; }
  std::size_t fitCount() const { return fit_count_; }
  const FitRecord& fit(std::size_t i) const { return fits_[i]; }
  std::uint64_t fitsDropped() const { return fits_dropped_; }

 private:
  ad::ReferencedSchedule schedule_;
  ad::WakePlan used_;  // the plan the cycle being closed executed
  std::array<LockRecord, kMaxLocks> locks_{};
  std::size_t lock_count_ = 0, open_index_ = 0;
  bool open_ = false;
  std::uint64_t locks_dropped_ = 0;
  std::array<FitRecord, kMaxFits> fits_{};
  std::size_t fit_count_ = 0;
  std::uint64_t fits_dropped_ = 0;
};

// The names the summary uses for the encodings.
inline const char* lossCauseName(ad::LossCause c) {
  switch (c) {
    case ad::LossCause::kNone: return "none";
    case ad::LossCause::kOutsideGate: return "outside_gate";
    case ad::LossCause::kNoGatedStamp: return "no_gated_stamp";
    case ad::LossCause::kPeriodClamp: return "period_clamp";
    case ad::LossCause::kNonFinite: return "non_finite";
  }
  return "?";
}

inline const char* fitOutcomeName(ad::FitOutcome o) {
  switch (o) {
    case ad::FitOutcome::kRequested: return "requested";
    case ad::FitOutcome::kQueueFailed: return "queue_failed";
    case ad::FitOutcome::kApplied: return "applied";
    case ad::FitOutcome::kRefused: return "refused";
    case ad::FitOutcome::kStale: return "stale";
    case ad::FitOutcome::kOldGeneration: return "old_generation";
    case ad::FitOutcome::kAbandoned: return "abandoned";
  }
  return "?";
}

inline const char* couplingName(std::uint8_t s) {
  switch (s) {
    case 0: return "insufficient_evidence";
    case 1: return "no_coupling_detected";
    case 2: return "request_tied_detected";
  }
  return "?";
}

}  // namespace rtctrl::apps::bus_timing
