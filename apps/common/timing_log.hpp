// Per-cycle timing log and end-of-run cycle statistics for hardware apps.
//
// CycleTimingLog writes the background thread's CycleRecords (host stamps
// of every cycle; see rtctrl/hw/cycle_record.hpp) to a CSV file without
// touching a control thread: a drain thread pops the CraneX7 timing ring
// and queues rows through util::AsyncWriter. The log is complete only if
// every cycle's record reached the file: no record dropped by a full ring
// and no row dropped or failed in the writer. The app then writes the
// run's summary next to it (TimingRunSummary, below).
#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include "rtctrl/arm/runner.hpp"
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
    "deadline_miss,command_kind";

inline void appendCycleRecordCsv(rtctrl::util::AsyncWriter& log,
                                 const rtctrl::hw::CycleRecord& r) {
  log.appendf(
      "%llu,%u,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,"
      "%lld,%lld,%llu,%llu,%llu,%u,%d,%d,%d,%s\n",
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
      r.write_ok ? 1 : 0, r.deadline_miss ? 1 : 0,
      commandKindName(r.command_kind));
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

// The evidence a timing log needs beyond its rows, written next to it as
// <log>.summary.toml once the arm is released: the run's own verdict,
// whether every record reached the file, and the CraneX7 counters when
// the controller finished (before shutdown, whose windows close with no
// submission by design) and after the thread stopped. timing_summary.py
// reports the completeness and controller criteria as indeterminate
// without it.
struct TimingRunSummary {
  std::string app;
  std::string status;       // the app's run status name
  bool successful = false;  // that status was success and release verified
  bool log_complete = false;
  bool escalated = false;
  double rate_hz = 0.0;
  rtctrl::hw::CraneX7::CycleStats at_controller_end;
  rtctrl::hw::CraneX7::CycleStats final_stats;
};

inline std::string timingSummaryPath(const std::string& log_path) {
  return log_path + ".summary.toml";
}

// Creates path exclusively. False if it exists or a write fails.
inline bool writeTimingRunSummary(const std::string& path,
                                  const TimingRunSummary& s) {
  std::FILE* f = std::fopen(path.c_str(), "wx");
  if (f == nullptr) return false;
  const auto counters = [f](const char* table, const char* comment,
                            const rtctrl::hw::CraneX7::CycleStats& c) {
    const auto u = [](std::uint64_t v) {
      return static_cast<unsigned long long>(v);
    };
    std::fprintf(f, "\n# %s\n[%s]\n", comment, table);
    std::fprintf(f, "cycles = %llu\noverruns = %llu\nskipped_periods = %llu\n",
                 u(c.cycles), u(c.overruns), u(c.skipped_periods));
    std::fprintf(f, "read_failures = %llu\nwrite_failures = %llu\n",
                 u(c.read_failures), u(c.write_failures));
    std::fprintf(f, "stale_submissions = %llu\n", u(c.stale_submissions));
    std::fprintf(f, "controller_deadline_misses = %llu\n",
                 u(c.controller_deadline_misses));
    std::fprintf(f, "timing_drops = %llu\n", u(c.timing_drops));
  };
  std::fprintf(f, "format = \"rtctrl-cycle-timing-run\"\nversion = 1\n");
  std::fprintf(f, "app = \"%s\"\nstatus = \"%s\"\n", s.app.c_str(),
               s.status.c_str());
  std::fprintf(f, "successful = %s\nlog_complete = %s\nescalated = %s\n",
               s.successful ? "true" : "false",
               s.log_complete ? "true" : "false",
               s.escalated ? "true" : "false");
  std::fprintf(f, "rate_hz = %.9g\n", s.rate_hz);
  counters("controller_end",
           "CycleStats when the controller finished, before shutdown",
           s.at_controller_end);
  counters("final", "CycleStats after the background thread stopped",
           s.final_stats);
  const bool ok = std::fflush(f) == 0 && std::ferror(f) == 0;
  return std::fclose(f) == 0 && ok;
}

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

// The arm's state when a run ended. Capture it right after the run and
// before shutdown, which changes it.
struct ArmEndState {
  rtctrl::hw::CraneX7::CycleStats stats;
  bool escalated = false;
  std::string last_error;
};

inline ArmEndState captureArmEnd(const rtctrl::hw::CraneX7& arm) {
  return {arm.cycleStats(), arm.escalated(), arm.lastError()};
}

// Why a run ended, from the runner's report (nothing is printed during
// the run), then the arm's state at that moment and every CycleStats
// field.
inline void printRunEnd(const rtctrl::arm::RunReport& report,
                        const ArmEndState& arm) {
  std::printf("run ended: %s in cycle %ld; last accepted sample seq "
              "%llu at %.6f s (run time %.3f s); final sample seq %llu "
              "at %.6f s; %llu duplicate samples\n",
              rtctrl::arm::describe(report.end), report.cycle,
              static_cast<unsigned long long>(report.accepted_seq),
              report.accepted_sample_t, report.run_t,
              static_cast<unsigned long long>(report.final_seq),
              report.final_sample_t,
              static_cast<unsigned long long>(report.duplicates));
  std::printf("arm at the end: %s, last error: %s\n",
              arm.escalated ? "escalated" : "not escalated",
              arm.last_error.empty() ? "(none)" : arm.last_error.c_str());
  printCycleStats(arm.stats);
}

}  // namespace x7
