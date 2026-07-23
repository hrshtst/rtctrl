// Sequence-keyed submission→application latency verification, shared
// by the tracking and identification runs.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>

#include "rtctrl/arm/crane_x7_tuning.hpp"
#include "rtctrl/arm/types.hpp"

namespace x7 {

namespace arm = rtctrl::arm;
namespace tuning = rtctrl::arm::tuning;

// Latency verification (docs/REMEDIATION_PLAN.md D3 round 9): match
// the snapshot's applied sequence to ITS OWN historical receipt —
// never the receipt created in this cycle. check() returns false on a
// violation (a skipped submission, a first application past the
// deadline, or a pending submission that aged out unapplied);
// `delay_out` receives the matched first-apply delay for telemetry
// (NaN when the snapshot still shows the pre-run baseline).
class LatencyVerifier {
 public:
  bool check(double feedback_time, const arm::CommandSnapshot& cmds,
             const arm::CommandReceipt& receipt, double* delay_out) {
    const auto& ap = cmds.applied;
    double first_apply_delay = std::nan("");
    if (ap.valid) {
      // a pending sequence OLDER than the applied one was skipped
      for (const auto& [seq, t_sub] : pending_) {
        (void)t_sub;
        if (seq < ap.target_seq) {
          std::fprintf(stderr,
                       "LATENCY: submission seq %llu was skipped "
                       "(applied jumped to %llu) — aborting\n",
                       static_cast<unsigned long long>(seq),
                       static_cast<unsigned long long>(ap.target_seq));
          return false;
        }
      }
      const auto it = pending_.find(ap.target_seq);
      if (it != pending_.end()) {
        first_apply_delay = ap.first_time - it->second;
        delay_cache_seq_ = ap.target_seq;
        delay_cache_ = first_apply_delay;
        pending_.erase(it);
        if (first_apply_delay > 2.0 * tuning::kNominalDt + 1e-9) {
          std::fprintf(stderr,
                       "LATENCY: seq %llu first applied %.1f ms after "
                       "submission (bound %.1f ms) — aborting\n",
                       static_cast<unsigned long long>(ap.target_seq),
                       1e3 * first_apply_delay,
                       2e3 * tuning::kNominalDt);
          return false;
        }
      } else if (ap.target_seq == delay_cache_seq_) {
        // completed sequence still current: re-emit the cached delay
        first_apply_delay = delay_cache_;
      }
      // else: pre-run baseline (e.g. the settle phase's last command)
      // — not a tracked submission, delay stays NaN
    }
    // any still-pending receipt past the deadline was never applied
    for (const auto& [seq, t_sub] : pending_) {
      if (feedback_time - t_sub > 2.0 * tuning::kNominalDt + 1e-9) {
        std::fprintf(stderr,
                     "LATENCY: submission seq %llu unapplied after "
                     "%.1f ms — aborting\n",
                     static_cast<unsigned long long>(seq),
                     1e3 * (feedback_time - t_sub));
        return false;
      }
    }
    if (receipt.accepted) {
      pending_[receipt.submitted_seq] = receipt.submission_time;
    }
    if (delay_out != nullptr) *delay_out = first_apply_delay;
    return true;
  }

 private:
  std::map<std::uint64_t, double> pending_;  // submitted_seq -> time
  std::uint64_t delay_cache_seq_ = 0;
  double delay_cache_ = std::nan("");
};

}  // namespace x7
