// The hardware-validated quiescence metric, shared by the settle gate
// (track/legacy/practical_track_common.hpp) and the identification capture
// phase.
#pragma once

#include <algorithm>

#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7 {

namespace model = rtctrl::model;

// Per-joint position RANGE across a ~0.15 s window, divided by the
// window span, then slow-filtered. Two hardware failure modes shaped
// this exact form:
//  * the per-sample backward difference sits on the encoder-LSB noise
//    floor (one 0.0015 rad flip over 10 ms reads 0.15 rad/s; a real
//    arm at rest under current control dithers within +/-1 count,
//    which held an early metric at ~0.07 rad/s on a STILL arm);
//  * an endpoint difference over the window ALIASES periodic motion
//    whose period divides the window (review repro: 6.67 and 13 Hz at
//    0.02 rad amplitude read as quiescent — 13 Hz being precisely the
//    gear mode the gate must catch).
// The range cancels for neither: dither reads ~0.02 rad/s, monotonic
// motion its true speed, and oscillation of amplitude A reads 2A/span
// regardless of phase alignment. Starts PESSIMISTIC (1.0) until the
// window fills — it must never read "quiet" before it has measured.
class QuiescenceMetric {
 public:
  QuiescenceMetric() {
    for (int i = 0; i < model::kCanonicalDof; ++i) speed_[i] = 1.0;
  }

  void update(double t, const zVec q8) {
    hist_t_[hist_head_] = t;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      hist_q_[hist_head_][i] = zVecElemNC(q8, i);
    }
    hist_head_ = (hist_head_ + 1) % kWindow;
    if (hist_count_ < kWindow) {
      ++hist_count_;  // stays pessimistic until the window fills
      return;
    }
    const int oldest = hist_head_;  // overwritten next call = oldest
    const double span = t - hist_t_[oldest];
    if (span <= 0.0) return;
    const double dt = hist_count_ >= 2
                          ? t - hist_t_[(hist_head_ + kWindow - 2) %
                                        kWindow]
                          : 0.0;
    const double a_m = dt / (0.15 + dt);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      double lo = hist_q_[0][i];
      double hi = lo;
      for (int k = 1; k < kWindow; ++k) {
        lo = std::min(lo, hist_q_[k][i]);
        hi = std::max(hi, hist_q_[k][i]);
      }
      speed_[i] += a_m * ((hi - lo) / span - speed_[i]);
    }
  }

  double maxSpeed() const {
    double m = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      m = std::max(m, speed_[i]);
    }
    return m;
  }
  double speed(int i) const { return speed_[i]; }

  static constexpr int kWindow = 16;  // ~0.15 s at the 10 ms cycle

 private:
  double speed_[model::kCanonicalDof] = {};
  double hist_t_[kWindow] = {};
  double hist_q_[kWindow][model::kCanonicalDof] = {};
  int hist_head_ = 0;
  int hist_count_ = 0;
};

}  // namespace x7
