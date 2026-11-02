// Per-cycle timing log and end-of-run cycle statistics for hardware apps.
//
// CycleTimingLog writes the background thread's CycleRecords (host stamps
// of every cycle; see rtctrl/hw/cycle_record.hpp) to a CSV file without
// touching a control thread: a drain thread pops the CraneX7 timing ring
// and queues rows through util::AsyncWriter. The log is complete only if
// every cycle's record reached the file: no record dropped by a full ring
// and no row dropped or failed in the writer.
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/hw/cycle_record.hpp"
#include "rtctrl/util/async_writer.hpp"

namespace x7 {

// Ring capacity for an app run at rate_hz: at least four seconds of
// records, so a briefly descheduled drain thread loses nothing.
inline std::size_t timingRingCapacity(double rate_hz) {
  return std::max<std::size_t>(4096,
                               static_cast<std::size_t>(4.0 * rate_hz));
}

inline const char* commandKindName(rtctrl::hw::CommandKind kind) {
  switch (kind) {
    case rtctrl::hw::CommandKind::kNone: return "none";
    case rtctrl::hw::CommandKind::kFresh: return "fresh";
    case rtctrl::hw::CommandKind::kFallback: return "fallback";
    case rtctrl::hw::CommandKind::kZeroVelocityFallback:
      return "zero-velocity-fallback";
    case rtctrl::hw::CommandKind::kPeriodic: return "periodic";
  }
  return "unknown";
}

// cycle runs 1, 2, ... over the whole file; segment counts thread starts
// (x7_track restarts its thread at the mode switch).
inline constexpr const char* kCycleRecordCsvHeader =
    "cycle,segment,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,"
    "first_reply_ns,last_reply_ns,read_end_ns,window_open_ns,"
    "controller_wake_ns,submit_ns,limit_start_ns,bus_write_start_ns,"
    "write_return_ns,cycle_end_ns,feedback_seq,applied_target_seq,"
    "applied_source_feedback_seq,skipped_periods,read_ok,write_ok,"
    "command_kind";

inline void appendCycleRecordCsv(rtctrl::util::AsyncWriter& log,
                                 const rtctrl::hw::CycleRecord& r) {
  log.appendf(
      "%llu,%u,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,"
      "%lld,%lld,%llu,%llu,%llu,%u,%d,%d,%s\n",
      static_cast<unsigned long long>(r.cycle),
      static_cast<unsigned>(r.segment),
      static_cast<long long>(r.scheduled_wake_ns),
      static_cast<long long>(r.wake_ns),
      static_cast<long long>(r.read_start_ns),
      static_cast<long long>(r.instr_return_ns),
      static_cast<long long>(r.first_reply_ns),
      static_cast<long long>(r.last_reply_ns),
      static_cast<long long>(r.read_end_ns),
      static_cast<long long>(r.window_open_ns),
      static_cast<long long>(r.controller_wake_ns),
      static_cast<long long>(r.submit_ns),
      static_cast<long long>(r.limit_start_ns),
      static_cast<long long>(r.bus_write_start_ns),
      static_cast<long long>(r.write_return_ns),
      static_cast<long long>(r.cycle_end_ns),
      static_cast<unsigned long long>(r.feedback_seq),
      static_cast<unsigned long long>(r.applied_target_seq),
      static_cast<unsigned long long>(r.applied_source_feedback_seq),
      static_cast<unsigned>(r.skipped_periods), r.read_ok ? 1 : 0,
      r.write_ok ? 1 : 0, commandKindName(r.command_kind));
}

class CycleTimingLog {
 public:
  // Creates path exclusively (throws std::runtime_error if it exists).
  // The CraneX7 must have Options::timing_ring_capacity > 0.
  CycleTimingLog(rtctrl::hw::CraneX7& crane, const std::string& path)
      : crane_(crane),
        writer_(std::make_unique<rtctrl::util::AsyncWriter>(path)) {
    writer_->append(kCycleRecordCsvHeader);
    writer_->append("\n");
    writer_->commitRow();
  }

  CycleTimingLog(const CycleTimingLog&) = delete;
  CycleTimingLog& operator=(const CycleTimingLog&) = delete;
  ~CycleTimingLog() { stop(); }

  // Call once the background thread has started (it allocates the ring).
  void start() {
    if (drain_.joinable() || stopped_) return;
    drain_ = std::thread([this] {
      while (!stop_.load()) {
        if (!drainPending()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    });
  }

  // Call after the background thread has stopped: joins the drain thread,
  // drains the remaining records on the calling thread (now the only
  // consumer) and closes the file. True only if the log is complete.
  // Idempotent.
  bool stop() {
    if (!stopped_) {
      stopped_ = true;
      stop_.store(true);
      if (drain_.joinable()) drain_.join();
      drainPending();
      const bool closed = writer_->close();
      complete_ = closed && crane_.cycleStats().timing_drops == 0;
    }
    return complete_;
  }

 private:
  // Queues every pending record; false when there was none.
  bool drainPending() {
    bool any = false;
    rtctrl::hw::CycleRecord r;
    while (crane_.popCycleRecord(&r)) {
      appendCycleRecordCsv(*writer_, r);
      writer_->commitRow();
      any = true;
    }
    return any;
  }

  rtctrl::hw::CraneX7& crane_;
  std::unique_ptr<rtctrl::util::AsyncWriter> writer_;
  std::thread drain_;
  std::atomic<bool> stop_{false};
  bool stopped_ = false;
  bool complete_ = false;
};

inline void printLatencyLine(const char* name,
                             const rtctrl::hw::LatencyHistogram& h) {
  if (h.count() == 0) return;
  // Quantiles are histogram bin upper edges capped at the maximum: never
  // below the recorded quantile, no claim beyond the recorded sample.
  std::printf("  %-18s p50<=%.3f p99<=%.3f p99.9<=%.3f max %.3f ms "
              "(n=%llu)\n",
              name, 1e3 * h.quantileUpperEdge(0.5),
              1e3 * h.quantileUpperEdge(0.99),
              1e3 * h.quantileUpperEdge(0.999), 1e3 * h.max(),
              static_cast<unsigned long long>(h.count()));
}

// Every CycleStats field, then the host-measured phase intervals.
inline void printCycleStats(const rtctrl::hw::CraneX7::CycleStats& stats) {
  std::printf("hardware timing: %llu cycles, %llu overruns, %llu skipped, "
              "%llu stale, %llu deadline misses, max %.3f ms\n",
              static_cast<unsigned long long>(stats.cycles),
              static_cast<unsigned long long>(stats.overruns),
              static_cast<unsigned long long>(stats.skipped_periods),
              static_cast<unsigned long long>(stats.stale_submissions),
              static_cast<unsigned long long>(
                  stats.controller_deadline_misses),
              1000.0 * stats.max_cycle_time_s);
  std::printf("hardware faults: %llu read failures, %llu write failures; "
              "max lateness %.3f ms, max feedback age at submission "
              "%.3f ms, %llu timing records dropped\n",
              static_cast<unsigned long long>(stats.read_failures),
              static_cast<unsigned long long>(stats.write_failures),
              1000.0 * stats.max_lateness_s,
              1000.0 * stats.max_feedback_age_at_submission_s,
              static_cast<unsigned long long>(stats.timing_drops));
  const auto& t = stats.timing;
  std::printf("host-measured phases:\n");
  printLatencyLine("wake_lateness", t.wake_lateness);
  printLatencyLine("period", t.period);
  printLatencyLine("host_transaction", t.host_transaction);
  printLatencyLine("controller_handoff", t.controller_handoff);
  printLatencyLine("controller_time", t.controller_time);
  printLatencyLine("write_handoff", t.write_handoff);
  printLatencyLine("limiter", t.limiter);
  printLatencyLine("host_write", t.host_write);
  printLatencyLine("host_work", t.host_work);
}

}  // namespace x7
