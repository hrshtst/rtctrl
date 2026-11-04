#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace rtctrl::hw {

// What one background-thread cycle transmitted.
enum class CommandKind : std::uint8_t {
  kNone = 0,      // no write was attempted (stopping or quiesced)
  kFresh = 1,     // a target accepted in this cycle's command window
  kFallback = 2,  // synchronized mode, no target accepted in this window:
                  // the last accepted (or default) target is resent
  kZeroVelocityFallback = 3,  // the same in velocity mode: zeros are sent
  kPeriodic = 4,  // unsynchronized mode: the latest target, whenever set
};

// Host-side stamps of one cycle, in nanoseconds of steady_clock since the
// CraneX7 was constructed; -1 means the cycle did not reach that point.
// These are host observations only. read_start is when the host began
// the feedback transaction, not when the servos sampled: a register read
// does not establish when its contents were measured. write_return is
// when the host's write() returned, not when the bytes left the adapter.
struct CycleRecord {
  // Numbered 1, 2, ... over the CraneX7's lifetime, across thread
  // restarts: the CycleStats::cycles count once this cycle completed.
  std::uint64_t cycle = 0;
  // Which startThread() run produced the record (1 for the first). A
  // restart begins a new segment; no period spans two segments.
  std::uint32_t segment = 0;
  std::int64_t scheduled_wake_ns = -1;
  std::int64_t wake_ns = -1;
  std::int64_t read_start_ns = -1;
  std::int64_t instr_return_ns = -1;  // instruction write() returned
  // When the SDK returned the first and the last status packet: host
  // observations that include USB buffering, driver delivery and SDK
  // polling, not the packets' arrival on the bus.
  std::int64_t first_reply_ns = -1;
  std::int64_t last_reply_ns = -1;
  std::int64_t read_end_ns = -1;
  std::int64_t window_open_ns = -1;
  std::int64_t controller_wake_ns = -1;  // first controller return
  std::int64_t submit_ns = -1;           // fresh targets only
  std::int64_t limit_start_ns = -1;      // limiting and gating begin
  std::int64_t bus_write_start_ns = -1;  // just before the sync write
  std::int64_t write_return_ns = -1;
  std::int64_t cycle_end_ns = -1;  // after bookkeeping and safety checks
  std::uint64_t feedback_seq = 0;  // latest successful acquisition
  std::uint64_t applied_target_seq = 0;
  std::uint64_t applied_source_feedback_seq = 0;
  std::uint32_t skipped_periods = 0;  // boundaries skipped after this cycle
  // Data bytes per servo the cycle's feedback read requested: the full
  // window, or the motion-only read between auxiliary cycles (Phase B2);
  // 0 when it read nothing.
  std::uint16_t read_len = 0;
  bool read_ok = false;
  bool write_ok = false;
  // The window closed with no submission while a controller was armed:
  // exactly the cycles CycleStats::controller_deadline_misses counts, and
  // counted in the same update as CycleStats::cycles, so a snapshot with
  // cycles == N counts the misses of records 1..N.
  bool deadline_miss = false;
  CommandKind command_kind = CommandKind::kNone;
};

// Log-spaced histogram of durations: 4 bins per octave from 1 us up to
// 2^20 us (about 1.05 s), plus an underflow and an overflow bin. Fixed
// size, O(1) add, no allocation. Negative or non-finite durations (an
// interval the cycle did not complete) are ignored.
class LatencyHistogram {
 public:
  static constexpr int kBinsPerOctave = 4;
  static constexpr int kOctaves = 20;
  static constexpr int kBins = kBinsPerOctave * kOctaves + 2;
  static constexpr double kFloorS = 1e-6;

  void add(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0) return;
    ++bins_[binOf(seconds)];
    ++count_;
    if (seconds > max_s_) max_s_ = seconds;
  }

  std::uint64_t count() const { return count_; }
  double max() const { return max_s_; }
  const std::array<std::uint64_t, kBins>& bins() const { return bins_; }

  static int binOf(double seconds) {
    if (seconds < kFloorS) return 0;
    const int i =
        1 + static_cast<int>(std::floor(kBinsPerOctave *
                                        std::log2(seconds / kFloorS)));
    return i >= kBins - 1 ? kBins - 1 : i;
  }
  // Exclusive upper edge of a bin; the overflow bin has none (+inf).
  static double binUpperEdge(int bin) {
    if (bin <= 0) return kFloorS;
    if (bin >= kBins - 1) return INFINITY;
    return kFloorS * std::exp2(static_cast<double>(bin) / kBinsPerOctave);
  }

  // Upper edge of the bin holding the q-quantile of the recorded values,
  // capped at the recorded maximum (which no quantile exceeds). It is
  // never below that empirical quantile; it carries no statistical claim
  // beyond the recorded sample. 0 when empty.
  double quantileUpperEdge(double q) const {
    if (count_ == 0) return 0.0;
    const double rank = std::ceil(q * static_cast<double>(count_));
    const std::uint64_t target =
        rank < 1.0 ? 1 : static_cast<std::uint64_t>(rank);
    std::uint64_t seen = 0;
    for (int b = 0; b < kBins; ++b) {
      seen += bins_[b];
      if (seen >= target) {
        return b == kBins - 1 ? max_s_ : std::min(binUpperEdge(b), max_s_);
      }
    }
    return max_s_;
  }

 private:
  std::array<std::uint64_t, kBins> bins_{};
  std::uint64_t count_ = 0;
  double max_s_ = 0.0;
};

// Host-measured intervals of the background cycle (each from the
// CycleRecord stamps named in the comment), accumulated over the
// thread's lifetime. Only intervals a cycle completed are recorded.
struct CycleTimingStats {
  LatencyHistogram wake_lateness;       // scheduled_wake -> wake
  LatencyHistogram period;              // previous wake -> wake
  LatencyHistogram host_transaction;    // read_start -> read_end
  LatencyHistogram reply_span;          // first_reply -> last_reply
  LatencyHistogram controller_handoff;  // window_open -> controller_wake
  LatencyHistogram controller_time;     // controller_wake -> submit
  LatencyHistogram write_handoff;       // submit -> limit_start
  LatencyHistogram limiter;             // limit_start -> bus_write_start
  LatencyHistogram host_write;          // bus_write_start -> write_return
  LatencyHistogram post_write;          // write_return -> cycle_end
  LatencyHistogram host_work;           // read_start -> cycle_end
};

// Adds one completed cycle's intervals to stats; prev_wake_ns is the
// previous cycle's wake (-1 for the first cycle).
inline void accumulateCycleTiming(const CycleRecord& r,
                                  std::int64_t prev_wake_ns,
                                  CycleTimingStats* stats) {
  const auto span = [](std::int64_t from, std::int64_t to) {
    return from < 0 || to < 0 ? -1.0 : static_cast<double>(to - from) * 1e-9;
  };
  stats->wake_lateness.add(span(r.scheduled_wake_ns, r.wake_ns));
  stats->period.add(span(prev_wake_ns, r.wake_ns));
  stats->host_transaction.add(span(r.read_start_ns, r.read_end_ns));
  stats->reply_span.add(span(r.first_reply_ns, r.last_reply_ns));
  stats->controller_handoff.add(span(r.window_open_ns, r.controller_wake_ns));
  stats->controller_time.add(span(r.controller_wake_ns, r.submit_ns));
  stats->write_handoff.add(span(r.submit_ns, r.limit_start_ns));
  stats->limiter.add(span(r.limit_start_ns, r.bus_write_start_ns));
  stats->host_write.add(span(r.bus_write_start_ns, r.write_return_ns));
  stats->post_write.add(span(r.write_return_ns, r.cycle_end_ns));
  stats->host_work.add(span(r.read_start_ns, r.cycle_end_ns));
}

}  // namespace rtctrl::hw
