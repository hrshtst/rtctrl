// Desired-trajectory feedforward for the servo-side follower, learned by
// feedback-error learning in position form (docs/theory/learning-control.md,
// "Command offset as torque" and "Feedback-error learning in position
// form"). The follower commands q_d + delta(t) with
//
//   delta_j(t) = sum_l w_jl * phi_jl(t),
//
// the regressors phi evaluated on the DESIRED trajectory only (never on
// measured state), so the weights carry over to other trajectories. The
// weights are updated between trials from the logged tracking error.
//
// The regressors are the ones tools/follow_learnability.py (predict) builds,
// in the same order and from the same smoothed reference, so offline
// predictions and the controller agree (to rounding: summation orders
// differ). Pure logic: model calls, no bus. Joint order is canonical
// (8 DOF); the gripper (joint 7) is never learned.
#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::model {
class ChainModel;
}

namespace rtctrl::learn {

using JointArray = std::array<double, model::kCanonicalDof>;  // canonical 8-DOF order

// Regressor columns, identical in order and name to
// tools/follow_learnability.py REGRESSORS.
inline constexpr int kRegressorCount = 6;
inline constexpr std::array<std::string_view, kRegressorCount> kRegressorNames =
    {"g", "tau_dyn", "ddq", "dq", "coulomb", "constant"};
inline constexpr int kRegressorGravity = 0;       // model gravity torque, Nm
inline constexpr int kRegressorDynamics = 1;      // inverse dynamics minus gravity, Nm
inline constexpr int kRegressorAcceleration = 2;  // reference ddq, rad/s^2
inline constexpr int kRegressorVelocity = 3;      // reference dq, rad/s
inline constexpr int kRegressorCoulomb = 4;       // tanh(dq / v0), dimensionless
inline constexpr int kRegressorConstant = 5;      // 1

// One sample: [joint][regressor]. The gripper row is all zeros.
using RegressorRow =
    std::array<std::array<double, kRegressorCount>, model::kCanonicalDof>;

struct RegressorOptions {
  double smooth_sigma_s = 0.04;  // zero-phase Gaussian on the reference (0 = none)
  double coulomb_v0_rad_s = 0.05;
};

// sigma finite and >= 0, v0 finite and > 0; std::invalid_argument otherwise.
void validateRegressorOptions(const RegressorOptions& options);

// The kernel half-width beyond which smoothing is refused (it bounds memory
// and time; 10^6 taps is sigma = 2500 s at 100 Hz).
inline constexpr double kMaxSmoothingHalfWidth = 1e6;

// follow_learnability.py gaussian_smooth: zero-phase Gaussian smoothing with
// max(1, ceil(4 sigma / dt - 1e-9)) taps on each side, normalised weights
// exp(-0.5 (m dt / sigma)^2), and the ends padded by repeating the end
// values, so the length is preserved. sigma_s == 0 returns a copy. Throws
// std::invalid_argument for an empty or non-finite series, dt not finite
// and > 0, sigma not finite and >= 0, or a half-width above
// kMaxSmoothingHalfWidth.
std::vector<double> zeroPhaseGaussianSmooth(const std::vector<double>& x,
                                            double sigma_s, double dt);

// numpy.gradient on a uniform grid (edge_order 1): central differences
// (f[i+1] - f[i-1]) / (2 dt) in the interior, first-order one-sided
// differences at both ends. Needs at least 2 samples and dt finite and > 0.
std::vector<double> numpyGradient(const std::vector<double>& x, double dt);

// follow_learnability.py reference_derivatives: the smoothed reference and
// numpy.gradient applied once (dq) and twice (ddq), per joint, all 8 joints.
struct SmoothedReference {
  std::vector<JointArray> q;
  std::vector<JointArray> dq;
  std::vector<JointArray> ddq;
};

// qref: N >= 3 finite samples on a uniform grid of step dt.
SmoothedReference smoothReference(const std::vector<JointArray>& qref,
                                  double dt, double sigma_s);

// The N regressor rows of the reference qref. Model terms come from
// ChainModel::gravityTorque and ChainModel::inverseDynamics at the smoothed
// (q, dq, ddq) of all 8 joints; tau_dyn is inverse dynamics minus gravity.
// Arm joints 0..6 get all six columns, the gripper row is zero. Throws
// std::invalid_argument for N < 3, non-finite samples, dt not finite and
// > 0, or invalid options; std::runtime_error naming the sample and joint
// for a non-finite model torque (finite but huge inputs).
std::vector<RegressorRow> buildRegressors(model::ChainModel& chain,
                                          const model::JointMap& map,
                                          const std::vector<JointArray>& qref,
                                          double dt,
                                          const RegressorOptions& options);

// --------------------------------------------------------------- weights

struct FeedforwardWeights {
  int trial = 0;  // number of updates applied
  // [joint][regressor], rad per regressor unit
  std::array<std::array<double, kRegressorCount>, model::kCanonicalDof> w{};
  RegressorOptions regressors;  // the options these weights were learned with
  std::string parent_sha256;      // weights file this was updated from; empty for trial 0
  std::string source_log_sha256;  // trial log used; empty for trial 0
  std::string reference_sha256;   // reference trained on last; empty for trial 0
};

inline constexpr const char* kFeedforwardWeightsFormat =
    "rtctrl-x7-follow-weights";
inline constexpr int kFeedforwardWeightsVersion = 1;

// The file rules, each violation named by its TOML key: trial >= 0; valid
// regressor options; every weight finite; the gripper's weights zero; each
// hash empty or 64 lowercase hex digits; trial 0 has all weights zero and
// all hashes empty, trial >= 1 has all three hashes. Throws
// std::invalid_argument. (The library stores hash strings; it does not
// compute SHA-256.)
void validateFeedforwardWeights(const FeedforwardWeights& weights);

// The TOML text (format = "rtctrl-x7-follow-weights", version = 1). Doubles
// are written with %.17g, plus ".0" when that text has neither '.' nor an
// exponent so TOML reads a float (this keeps -0.0), so a round trip is
// bit-exact and write -> read -> write byte-identical. Validates first.
std::string formatFeedforwardWeights(const FeedforwardWeights& weights);

// Strict reading: a TOML error, unknown or missing keys, the wrong format,
// version or regressor names, arrays of the wrong length, values of the
// wrong type, non-finite numbers, and every validateFeedforwardWeights rule
// are errors naming the key. Integers are accepted where a number is
// expected (exact up to 2^53). source names the text in messages. Throws
// std::runtime_error.
FeedforwardWeights parseFeedforwardWeights(std::string_view text,
                                           const std::string& source);
FeedforwardWeights readFeedforwardWeights(const std::filesystem::path& path);

// Creates path exclusively (an existing file, or symlink, is refused and
// left untouched) and writes formatFeedforwardWeights; invalid weights are
// refused before the file is created, and on a failure after creation the
// partial file is removed. std::invalid_argument for invalid weights,
// std::runtime_error for file errors.
void writeFeedforwardWeights(const std::filesystem::path& path,
                             const FeedforwardWeights& weights);

// -------------------------------------------------------------- synthesis

// delta[i][j] = sum_l w[j][l] * phi[i][j][l] (summed in column order), the
// gripper zero. No clamping: limits are enforced where the offset file is
// validated. Throws std::invalid_argument for an empty or non-finite phi,
// non-finite or gripper weights, or a non-finite result.
std::vector<JointArray> synthesizeOffset(
    const std::vector<RegressorRow>& regressors,
    const FeedforwardWeights& weights);

// ----------------------------------------------------------------- update

struct FelOptions {
  enum class Step { NormalisedGradient, LeastSquares };
  Step step = Step::NormalisedGradient;
  double gain = 0.2;                   // eta, in (0, 1]
  double advance_s = 0.05;             // pairs phi(t) with the error at t + advance
  double error_lead_s = 0.0;           // T_s in s = e + T_s * edot, >= 0
  double error_smooth_sigma_s = 0.04;  // zero-phase Gaussian on the error (0 = none)
  // LeastSquares: penalty ridge * N on RMS-normalised columns (as the
  // Python tool's ridge_fit).
  double ridge = 1e-6;
  // A regressor column with smaller RMS over the N samples is skipped (its
  // weight is left unchanged). Must be > 0: an all-zero column cannot be
  // normalised.
  double negligible_rms = 1e-6;
  // The gripper (joint 7) must stay disabled.
  std::array<bool, model::kCanonicalDof> joint_enabled{
      true, true, true, true, true, true, true, false};
};

// LeastSquares refuses a joint whose kept columns are numerically dependent:
// lambda_min(C + ridge I) <= kLeastSquaresConditionLimit * lambda_max(C +
// ridge I), C as in FelJointReport (a solve of the normal equations past
// that keeps fewer than about four significant digits). With ridge > 0 at
// the default 1e-6 this never trips.
inline constexpr double kLeastSquaresConditionLimit = 1e-12;

struct FelJointReport {
  bool enabled = false;
  // RMS of each regressor column over the N samples, and (enabled joints
  // only) the columns below negligible_rms, whose weights are left unchanged.
  std::array<double, kRegressorCount> column_rms{};
  std::array<bool, kRegressorCount> skipped{};
  int kept_columns = 0;
  // Extreme eigenvalues of C, the kept columns' normalised Gram matrix
  // mean(phi_a phi_b) / sqrt(mean(phi_a^2) mean(phi_b^2)), by cyclic Jacobi
  // rotations to an off-diagonal norm of eps |C|_F (absolute error below
  // 1e-10: Weyl's inequality plus the rounding of at most 100 sweeps of
  // backward-stable rotations, with |C|_F <= 6). 0 when no column is kept.
  double lambda_max = 0.0;
  double lambda_min = 0.0;
  double s_rms = 0.0;  // RMS of the error signal s over the N samples
  // RMS of s - Phi_K * (w_{k+1} - w_k)_K: the predicted remaining error.
  double residual_rms = 0.0;
};

struct FelReport {
  std::array<FelJointReport, model::kCanonicalDof> joints{};
};

// The error signal of the update, N samples per joint (all 8 joints):
// e smoothed over all M samples with zeroPhaseGaussianSmooth
// (error_smooth_sigma_s), differentiated with numpyGradient, and
// s[i] = e_s(t_i + advance) + error_lead_s * edot_s(t_i + advance) by
// linear interpolation. An advance within 1e-9 samples of a whole number of
// samples is that whole number, so the interpolation is exact there.
// error: M >= N + ceil(advance_s / dt - 1e-9) finite samples of
// e = q_d - q (the NEGATIVE of the x7_follow log's error column, which is
// q - qref), on the tracking grid and continuing past it (the final hold).
// N >= 3. Throws std::invalid_argument naming the argument; a short tail
// names the number of samples needed.
std::vector<JointArray> felErrorSignal(const std::vector<JointArray>& error,
                                       std::size_t samples, double dt,
                                       const FelOptions& options);

// One trial-wise update. phi: N samples of the regressors (buildRegressors
// with w_k.regressors) on the tracking grid; error: M samples of
// e = q_d - q measured WITH the offset synthesised from w_k applied (see
// felErrorSignal). Per enabled joint, with K the kept columns:
//
// - NormalisedGradient (Kawato's gradient rule, one learning rate per
//   weight): w_l += gain * mean(phi_l s) / mean(phi_l^2), l in K. Refused
//   (std::invalid_argument naming the joint and the largest admissible
//   gain) when gain * lambda_max >= 2: for a unit-gain plant the update
//   iterates u <- (I - gain C) u on the RMS-normalised weight error, which
//   converges only while gain * lambda_max < 2.
// - LeastSquares: dw = argmin |Phi_K dw - s|^2 + ridge * N * |dw_n|^2 with
//   dw_n the weights of the RMS-normalised columns (the Python tool's
//   ridge_fit), then w_K += gain * dw.
//
// The eigenvalue guard is the convergence condition for an aligned,
// unit-gain plant whose error enters unsmoothed and without lead; it proves
// nothing about the real servo, nor about the smoothed or led error signal.
// The normalised gradient learns weakly excited weight directions (small
// eigenvalues of C) slowly, so what converges quickly is the RMS of the
// remaining error, not the weights.
//
// Disabled joints and skipped columns keep their weights bit-exactly. The
// result has trial = w_k.trial + 1, the same regressor options and empty
// hashes (the caller fills them). Throws std::invalid_argument naming the
// argument for sizes, non-finite values, gain outside (0, 1], negative
// times, ridge < 0, negligible_rms <= 0, an enabled gripper, or invalid
// w_k values. report, when given, receives the per-joint diagnostics (only
// when the update succeeds).
FeedforwardWeights felUpdate(const FeedforwardWeights& w_k,
                             const std::vector<RegressorRow>& phi,
                             const std::vector<JointArray>& error, double dt,
                             const FelOptions& options,
                             FelReport* report = nullptr);

}  // namespace rtctrl::learn
