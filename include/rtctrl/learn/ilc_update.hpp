#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::learn {

// Iterative learning control in position form, the offline update between
// trials (docs/theory/learning-control.md, "Iterative learning control in
// position form"). Pure logic on arrays: no files, no model, no bus.
//
// The follower commands q_d + delta_k(t) during trial k. With the logged
// tracking error e_k = q_d - q (the theory note's sign; the follower's CSV
// log stores q - q_ref, the opposite), the update is
//
//   delta_{k+1}(t) = Q[ delta_k(t) + Phi e_k(t + Delta)
//                         + Psi edot_k(t + Delta) ]
//
// with Delta a time advance (dead time and servo lag) and Q a zero-phase
// low-pass, followed by a per-sample clamp on the change and on the value.

using JointArray = std::array<double, model::kCanonicalDof>;  // canonical 8-DOF order

// Zero-phase low-pass: a 2nd-order Butterworth low-pass (bilinear transform
// with frequency prewarping) applied forward, then backward. The analog
// prototype of the combined forward-backward magnitude is
// 1 / (1 + (f/f0)^4), so the design corner is
// f0 = cutoff_hz / (sqrt(2) - 1)^(1/4) and f0 is prewarped. The digital
// combined magnitude is then exactly
// 1 / (1 + (tan(pi f dt) / tan(pi f0 dt))^4): 1/sqrt(2) at cutoff_hz up to
// the bilinear warping between f0 and cutoff_hz (+0.09 % at 2 Hz and
// dt = 0.01 s; 0.83 instead of 0.71 at half the Nyquist frequency, and one
// at the 0.8 limit below). The phase is zero.
//
// Edges: each column is padded at both ends by odd reflection
// (2*x[0] - x[k], likewise at the end) of P = min(n - 1,
// ceil(1.5 / (cutoff_hz * dt))) samples; each pass starts from the filter's
// steady state for its first input sample, so a constant comes back
// unchanged to rounding. The padding is removed from the result. Each joint
// column is filtered independently.
//
// Throws std::invalid_argument naming the argument unless x has at least 2
// samples, dt is finite and > 0, cutoff_hz is finite, > 0 and at most 0.8 of
// the Nyquist frequency 0.5 / dt, and every input is finite.
std::vector<JointArray> zeroPhaseLowPass(const std::vector<JointArray>& x,
                                         double dt, double cutoff_hz);

struct IlcOptions {
  JointArray phi{};                  // position-error gain per joint (dimensionless)
  JointArray psi_s{};                // velocity-error gain per joint [s]
  double advance_s = 0.05;           // Delta >= 0
  double cutoff_hz = 2.0;            // Q: zero-phase low-pass, -3 dB point
  double max_abs_rad = 0.15;         // clamp on |delta_{k+1}|
  double max_trial_step_rad = 0.05;  // clamp on |delta_{k+1} - delta_k| per sample
};

// Per joint, what one update did; the app layer logs it.
struct IlcJointReport {
  bool enabled = false;          // phi or psi_s nonzero
  std::size_t step_clamped = 0;  // samples the trial-step clamp changed
  std::size_t abs_clamped = 0;   // samples the absolute clamp changed
  double error_rms_rad = 0.0;    // raw e over the tracking grid (rows 0..N-1)
  double delta_rms_rad = 0.0;    // delta_{k+1}
  double delta_max_abs_rad = 0.0;
  double change_rms_rad = 0.0;  // delta_{k+1} - delta_k
  double change_max_abs_rad = 0.0;
};

struct IlcReport {
  std::array<IlcJointReport, model::kCanonicalDof> joints{};
};

// One update. delta_k has N samples, sample i at t = i*dt (the tracking
// grid). error has M >= N samples of e = q_d - q on the same grid: rows
// 0..N-1 during tracking, rows N..M-1 the hold after the reference ends
// (error against the final reference value), which the advance reads. It
// needs M - N >= ceil(advance_s / dt - 1e-9) (an advance within 1e-9 of a
// sample past the last error row reads that row).
//
// Per joint j:
//  1. phi[j] == 0 and psi_s[j] == 0: the column of delta_k is returned
//     bit-exactly (a disabled joint is never filtered or clamped).
//     Otherwise:
//  2. ef = zero-phase low-pass of the error column over all M samples;
//  3. edot = time derivative of ef: central differences inside, one-sided
//     first differences at the two ends;
//  4. ea[i], eda[i] = ef, edot at t = i*dt + advance_s, linearly
//     interpolated between grid samples;
//  5. c[i] = delta_k[i][j] + phi[j]*ea[i] + psi_s[j]*eda[i];
//  6. d = zero-phase low-pass of c over the N samples (Q acts on the whole
//     bracket, delta_k included);
//  7. d[i] = delta_k[i][j] + clamp(d[i] - delta_k[i][j], +-max_trial_step_rad),
//     then clamp(d[i], +-max_abs_rad). The absolute clamp comes last and
//     wins: where delta_k itself lies outside +-max_abs_rad the change can
//     exceed max_trial_step_rad.
//
// Throws std::invalid_argument naming the argument unless N >= 2; every
// value of delta_k and error is finite; dt is finite and > 0; advance_s is
// finite and >= 0; every phi and psi_s is finite and >= 0; both clamps are
// finite and > 0; cutoff_hz meets zeroPhaseLowPass's conditions; and the
// tail is long enough (the message says how many samples it needs). All of
// it is checked before anything is computed, disabled joints included.
// Deterministic; the inputs are not modified.
std::vector<JointArray> ilcUpdate(const std::vector<JointArray>& delta_k,
                                  const std::vector<JointArray>& error,
                                  double dt, const IlcOptions& options);

// The same, also filling report (left untouched when it throws).
std::vector<JointArray> ilcUpdate(const std::vector<JointArray>& delta_k,
                                  const std::vector<JointArray>& error,
                                  double dt, const IlcOptions& options,
                                  IlcReport& report);

}  // namespace rtctrl::learn
