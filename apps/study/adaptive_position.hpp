// Slotine and Li's adaptive controller in position form: the pure logic of
// x7_adaptive_follow_study, a SIMULATION-ONLY study (docs/theory/
// learning-control.md, "Slotine and Li adaptive control in position form").
// Nothing here talks to a bus or an Arm; the study binary drives a SimArm.
// It is not a controller for the real arm and must not become one.
//
// Notation as in the theory note: reference q_d, measured q, e = q_d - q,
// command u = q_d + delta. Per arm joint j (0..6) the offset is
//
//   delta_j = clamp(sum_l w_jl phi_jl, +-max_offset),
//
// with the library's six regressors (rtctrl/learn/regressor_feedforward.hpp,
// kRegressor* order: model gravity torque, model dynamic torque,
// acceleration, velocity, smooth Coulomb sign, constant) and weights in
// position units that absorb 1/K. The gripper (joint 7) is never adapted:
// its row and weights stay zero, its offset is zero. The weights adapt in
// the loop, every control cycle,
//
//   w_jl <- w_jl + dt * gamma_jl * phi_jl(t - Delta) * s_j(t),
//   s_j = e_j + T_s * edot_j,  edot_j = dq_d,j - dq_j,
//
// with the regressor row of Delta ago (AdvanceBuffer) paired with the
// present error, and gamma_jl = gamma / mean(phi_jl^2) over the desired-
// trajectory regressors of the reference (normalisedGains), so that gamma
// [1/s] is the common rate; a column whose RMS over the reference is
// negligible is not adapted.
//
// Two forms of phi:
// - Desired compensation (Sadegh and Horowitz's desired trajectory adaptive
//   control; structurally in-loop feedback-error learning): phi is the
//   library's buildRegressors row of the current grid sample, evaluated on
//   the (smoothed) desired trajectory only. No measured signal enters delta
//   except through the slowly adapting weights.
// - Proper form (properRegressorRow): phi on the measured state and the
//   reference velocity dq_r = dq_d + Lambda e, ddq_r = ddq_d + Lambda edot:
//   g(q), [M(q) ddq_r + C(q, dq) dq_r], ddq_r, dq_r, tanh(dq_r / v0), 1,
//   with the Christoffel-consistent C (ModelProducts). Lambda = 0 still
//   evaluates the model terms at the measured posture and velocity.
//
// What is and is not Slotine and Li's law (textbook form: tau = Y a_hat -
// K_D s, a_hat' = -Gamma Y^T s, s = q' - q_r', q_r' = q_d' - Lambda q~):
// - The feedback term -K_D s is not computed here. The servo's own PD,
//   tau = K (u - q) - D q', supplies K e - D q' = -D s_servo - D q_d' with
//   Lambda_servo = K / D, fixed by the servo (25 1/s in
//   config/follow_sim_servo_like.toml, kd/kp = 0.04 s); the remaining
//   -D q_d' is one more feedforward term, absorbed by the velocity
//   regressor. The textbook uses ONE Lambda in q_r', in s and in the
//   feedback; here the regressor's Lambda is a free study parameter, equal
//   to the servo's only when Lambda = K / D, and the adaptation signal
//   e + T_s edot has its own lead T_s (the textbook's s is proportional to
//   it only for T_s = 1 / Lambda).
// - The model term Y a_hat is sent as the offset delta = K^-1 Y a_hat,
//   divided by a stiffness K that is not known; the weights absorb it.
// - The per-regressor weights replace the physical parameter vector: a
//   diagonal reparametrisation, one scale per regressor and joint (the
//   model torque columns keep the nominal model's inertial parameters and
//   the weights only scale them per joint), not the base inertial
//   parameters, and the regressors of different joints share no weight.
// - The adaptation pairs phi(t - Delta) with s(t) (a lag compensation the
//   textbook does not have), uses normalised per-regressor gains, and runs
//   through the offset clamp with no parameter projection. The gains are
//   normalised on the desired trajectory in both forms, so gamma is the
//   common rate only there: a proper-form column used with a larger mean
//   square (Lambda e, Lambda edot from the servo's velocity estimate)
//   adapts faster by that ratio.
// - The plant has a transport delay, a velocity estimate with its own lag,
//   encoder quantisation and reflected rotor inertia: none of the Lyapunov
//   arguments of Slotine and Li or of Sadegh and Horowitz is assumed to
//   hold. The study is empirical.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/learn/joint_array.hpp"
#include "rtctrl/learn/regressor_feedforward.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7::study {

using rtctrl::learn::JointArray;
using rtctrl::learn::kRegressorCount;
using rtctrl::learn::RegressorRow;

inline constexpr int kAdaptiveDof = rtctrl::model::kCanonicalDof;
inline constexpr int kAdaptiveArmJoints = 7;  // joints 0..6; 7 is the gripper

// [joint][regressor], canonical order and kRegressor* order.
using RegressorMatrix =
    std::array<std::array<double, kRegressorCount>, kAdaptiveDof>;

// ------------------------------------------------------- model products

// The rigid-body terms of the nominal model in canonical coordinates, from
// ChainModel::inverseDynamics and gravityTorque only. With ID(q, v, a) the
// inverse dynamics and g(q) the gravity torque,
//
//   c(q; v)    = ID(q, v, 0) - g(q)                 (velocity torque C(q, v) v)
//   b(q; u, v) = [c(q; u + v) - c(q; u) - c(q; v)] / 2
//   M(q) a     = ID(q, 0, a) - g(q).
//
// c is quadratic in v (its polarisation b is the symmetric bilinear form
// with b(v, v) = c(v)), and b(q; u, v) = C(q, u) v with the Christoffel-
// consistent C, C_kj = sum_i Gamma_ijk u_i, for which dM/dt - 2 C(q, q') is
// skew-symmetric. tests/unit/adaptive_position_test.cpp checks these
// properties numerically. Every call moves the chain's joint displacement
// (the ChainModel functions do).
class ModelProducts {
 public:
  ModelProducts(rtctrl::model::ChainModel& chain,
                const rtctrl::model::JointMap& map)
      : chain_(chain), map_(map) {}

  JointArray gravity(const JointArray& q) {
    load(q, &q_);
    chain_.gravityTorque(map_, q_, tau_);
    return result();
  }

  JointArray inverseDynamics(const JointArray& q, const JointArray& dq,
                             const JointArray& ddq) {
    load(q, &q_);
    load(dq, &dq_);
    load(ddq, &ddq_);
    chain_.inverseDynamics(map_, q_, dq_, ddq_, tau_);
    return result();
  }

  // c(q; v) = C(q, v) v.
  JointArray velocityTorque(const JointArray& q, const JointArray& v) {
    return minus(inverseDynamics(q, v, JointArray{}), gravity(q));
  }

  // b(q; u, v) = C(q, u) v, the Christoffel-consistent product.
  JointArray coriolis(const JointArray& q, const JointArray& u,
                      const JointArray& v) {
    JointArray sum{};
    for (int i = 0; i < kAdaptiveDof; ++i) sum[i] = u[i] + v[i];
    const JointArray c_sum = velocityTorque(q, sum);
    const JointArray c_u = velocityTorque(q, u);
    const JointArray c_v = velocityTorque(q, v);
    JointArray out{};
    for (int i = 0; i < kAdaptiveDof; ++i) {
      out[i] = 0.5 * (c_sum[i] - c_u[i] - c_v[i]);
    }
    return out;
  }

  // M(q) a.
  JointArray mass(const JointArray& q, const JointArray& a) {
    return minus(inverseDynamics(q, JointArray{}, a), gravity(q));
  }

 private:
  static void load(const JointArray& values, rtctrl::model::ZVector* out) {
    for (int i = 0; i < kAdaptiveDof; ++i) (*out)[i] = values[i];
  }
  JointArray result() const {
    JointArray out{};
    for (int i = 0; i < kAdaptiveDof; ++i) out[i] = tau_[i];
    return out;
  }
  static JointArray minus(const JointArray& a, const JointArray& b) {
    JointArray out{};
    for (int i = 0; i < kAdaptiveDof; ++i) out[i] = a[i] - b[i];
    return out;
  }

  rtctrl::model::ChainModel& chain_;
  const rtctrl::model::JointMap& map_;
  rtctrl::model::ZVector q_{kAdaptiveDof};
  rtctrl::model::ZVector dq_{kAdaptiveDof};
  rtctrl::model::ZVector ddq_{kAdaptiveDof};
  rtctrl::model::ZVector tau_{kAdaptiveDof};
};

// ------------------------------------------------- proper-form regressor

// The proper form's regressor row at the measured state (q, dq) for the
// reference (q_d, dq_d, ddq_d), all 8 joints:
//
//   e = q_d - q,  edot = dq_d - dq,
//   dq_r = dq_d + lambda e,  ddq_r = ddq_d + lambda edot,
//   phi_j = [g_j(q), [M(q) ddq_r + C(q, dq) dq_r]_j, ddq_r,j, dq_r,j,
//            tanh(dq_r,j / v0), 1]   for arm joints j = 0..6,
//
// the gripper row zero. With (q, dq) equal to the reference the library's
// buildRegressors evaluates (its smoothed q, dq, ddq) and lambda = 0 this
// is buildRegressors' row up to rounding (its tau_dyn is one inverse
// dynamics call minus gravity, here the mass and Christoffel products).
// Throws std::invalid_argument for lambda not finite and >= 0, v0 not
// finite and > 0, or a non-finite input; std::runtime_error for a
// non-finite model torque.
inline RegressorRow properRegressorRow(
    ModelProducts& model, const JointArray& q, const JointArray& dq,
    const JointArray& q_d, const JointArray& dq_d, const JointArray& ddq_d,
    double lambda, double coulomb_v0) {
  if (!std::isfinite(lambda) || lambda < 0.0) {
    throw std::invalid_argument(
        "properRegressorRow: lambda must be finite and >= 0");
  }
  if (!std::isfinite(coulomb_v0) || coulomb_v0 <= 0.0) {
    throw std::invalid_argument(
        "properRegressorRow: coulomb_v0 must be finite and > 0");
  }
  JointArray dq_r{};
  JointArray ddq_r{};
  for (int i = 0; i < kAdaptiveDof; ++i) {
    for (const double value : {q[i], dq[i], q_d[i], dq_d[i], ddq_d[i]}) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument(
            "properRegressorRow: non-finite input on joint " +
            std::to_string(i));
      }
    }
    dq_r[i] = dq_d[i] + lambda * (q_d[i] - q[i]);
    ddq_r[i] = ddq_d[i] + lambda * (dq_d[i] - dq[i]);
  }
  const JointArray g = model.gravity(q);
  const JointArray inertial = model.mass(q, ddq_r);
  const JointArray coriolis = model.coriolis(q, dq, dq_r);
  RegressorRow row{};
  for (int j = 0; j < kAdaptiveArmJoints; ++j) {
    const double dynamics = inertial[j] + coriolis[j];
    if (!std::isfinite(g[j]) || !std::isfinite(dynamics)) {
      throw std::runtime_error(
          "properRegressorRow: non-finite model torque on joint " +
          std::to_string(j));
    }
    row[j][rtctrl::learn::kRegressorGravity] = g[j];
    row[j][rtctrl::learn::kRegressorDynamics] = dynamics;
    row[j][rtctrl::learn::kRegressorAcceleration] = ddq_r[j];
    row[j][rtctrl::learn::kRegressorVelocity] = dq_r[j];
    row[j][rtctrl::learn::kRegressorCoulomb] = std::tanh(dq_r[j] / coulomb_v0);
    row[j][rtctrl::learn::kRegressorConstant] = 1.0;
  }
  return row;
}

// ---------------------------------------------------- normalised gains

// The FelOptions default: a column with a smaller RMS is not adapted.
inline constexpr double kNegligibleRegressorRms = 1e-6;

struct AdaptationGains {
  RegressorMatrix gain{};  // gamma / mean(phi^2); 0 where not adapted
  RegressorMatrix rms{};   // RMS of each desired column over the reference
  // Columns adapted: arm joints whose column RMS is at least the negligible
  // level (with gamma = 0 their gain is 0 but they still count as adapted).
  std::array<std::array<bool, kRegressorCount>, kAdaptiveDof> adapted{};
};

// The per-regressor gains of the in-loop update, computed once from the N
// desired-trajectory rows of the reference (buildRegressors): gain_jl =
// gamma / mean_n(phi_jl[n]^2) for arm joints with RMS >= negligible_rms;
// the gripper and negligible columns get gain 0 and are not adapted. Throws
// std::invalid_argument for no rows, a non-finite row, gamma not finite and
// >= 0, or negligible_rms not finite and > 0.
inline AdaptationGains normalisedGains(
    const std::vector<RegressorRow>& desired, double gamma_per_s,
    double negligible_rms = kNegligibleRegressorRms) {
  if (desired.empty()) {
    throw std::invalid_argument("normalisedGains: no regressor rows");
  }
  if (!std::isfinite(gamma_per_s) || gamma_per_s < 0.0) {
    throw std::invalid_argument(
        "normalisedGains: gamma must be finite and >= 0");
  }
  if (!std::isfinite(negligible_rms) || negligible_rms <= 0.0) {
    throw std::invalid_argument(
        "normalisedGains: negligible_rms must be finite and > 0");
  }
  AdaptationGains gains;
  for (int j = 0; j < kAdaptiveArmJoints; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      double sum = 0.0;
      for (const auto& row : desired) {
        if (!std::isfinite(row[j][l])) {
          throw std::invalid_argument("normalisedGains: non-finite row");
        }
        sum += row[j][l] * row[j][l];
      }
      const double mean_square = sum / static_cast<double>(desired.size());
      gains.rms[j][l] = std::sqrt(mean_square);
      gains.adapted[j][l] = gains.rms[j][l] >= negligible_rms;
      gains.gain[j][l] = gains.adapted[j][l] ? gamma_per_s / mean_square : 0.0;
    }
  }
  return gains;
}

// ------------------------------------------------------ advance pairing

// The advance Delta as a whole number of control cycles: round(Delta /
// dt). Throws std::invalid_argument unless Delta is finite and >= 0, dt
// finite and > 0, and Delta / dt within 1e-6 of a whole number.
inline std::size_t advanceCycles(double advance_s, double dt_s) {
  if (!std::isfinite(advance_s) || advance_s < 0.0 || !std::isfinite(dt_s) ||
      dt_s <= 0.0) {
    throw std::invalid_argument(
        "advanceCycles: advance must be finite and >= 0, dt finite and > 0");
  }
  const double cycles = advance_s / dt_s;
  const double whole = std::round(cycles);
  if (std::fabs(cycles - whole) > 1e-6) {
    throw std::invalid_argument(
        "advanceCycles: the advance must be a whole number of control "
        "cycles");
  }
  return static_cast<std::size_t>(whole);
}

// The regressor rows of the last delay + 1 cycles, newest last, each with
// a flag saying whether it may be paired for an update. paired() is the
// row pushed `delay` pushes before the newest (the newest itself for delay
// 0): phi(t - Delta) against the present error.
class AdvanceBuffer {
 public:
  explicit AdvanceBuffer(std::size_t delay_cycles)
      : delay_(delay_cycles),
        rows_(delay_cycles + 1),
        adaptable_(delay_cycles + 1, false) {}

  std::size_t delay() const { return delay_; }

  void clear() {
    count_ = 0;
    next_ = 0;
  }

  void push(const RegressorRow& row, bool adaptable) {
    rows_[next_] = row;
    adaptable_[next_] = adaptable;
    next_ = (next_ + 1) % rows_.size();
    if (count_ < rows_.size()) ++count_;
  }

  // Null while fewer than delay + 1 rows were pushed since clear(), and
  // when the paired row was pushed as not adaptable.
  const RegressorRow* paired() const {
    if (count_ < rows_.size()) return nullptr;
    // With the buffer full, next_ is the oldest slot: delay pushes before
    // the newest.
    if (!adaptable_[next_]) return nullptr;
    return &rows_[next_];
  }

 private:
  std::size_t delay_;
  std::vector<RegressorRow> rows_;
  std::vector<bool> adaptable_;
  std::size_t next_ = 0;
  std::size_t count_ = 0;
};

// ------------------------------------------------------- in-loop update

class AdaptiveOffset {
 public:
  struct Cycle {
    JointArray delta{};      // the clamped offset sent; the gripper's is 0
    JointArray unclamped{};  // sum_l w_jl phi_jl before the clamp
    std::array<bool, kAdaptiveDof> clamped{};
    bool any_clamped = false;
    bool updated = false;  // whether the weights moved this cycle
  };

  // dt_s: the control period; advance_cycles: Delta in cycles;
  // max_offset_rad: the clamp. Throws std::invalid_argument for dt or the
  // clamp not finite and > 0.
  AdaptiveOffset(const AdaptationGains& gains, double dt_s,
                 std::size_t advance_cycles, double max_offset_rad)
      : gains_(gains),
        dt_(dt_s),
        max_offset_(max_offset_rad),
        buffer_(advance_cycles) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0 || !std::isfinite(max_offset_rad) ||
        max_offset_rad <= 0.0) {
      throw std::invalid_argument(
          "AdaptiveOffset: dt and max_offset must be finite and > 0");
    }
  }

  // A new pass: the regressor history is dropped, the weights kept.
  void startPass() { buffer_.clear(); }

  const RegressorMatrix& weights() const { return w_; }

  // Throws std::invalid_argument for a non-finite weight or a nonzero
  // gripper weight.
  void setWeights(const RegressorMatrix& w) {
    for (int j = 0; j < kAdaptiveDof; ++j) {
      for (int l = 0; l < kRegressorCount; ++l) {
        if (!std::isfinite(w[j][l]) ||
            (j >= kAdaptiveArmJoints && w[j][l] != 0.0)) {
          throw std::invalid_argument(
              "AdaptiveOffset: weights must be finite, the gripper's zero");
        }
      }
    }
    w_ = w;
  }

  // The offset of row phi with the present weights: no update, no history.
  Cycle offset(const RegressorRow& phi) const {
    Cycle out;
    for (int j = 0; j < kAdaptiveArmJoints; ++j) {
      double sum = 0.0;
      for (int l = 0; l < kRegressorCount; ++l) sum += w_[j][l] * phi[j][l];
      out.unclamped[j] = sum;
      // A non-finite sum stays non-finite: the caller must see it.
      out.clamped[j] = std::isfinite(sum) && std::fabs(sum) > max_offset_;
      out.delta[j] = out.clamped[j] ? std::copysign(max_offset_, sum) : sum;
      out.any_clamped = out.any_clamped || out.clamped[j];
    }
    return out;
  }

  // One control cycle: phi joins the history (adaptable: whether it may be
  // paired for a later update); if the row of `advance` cycles ago is
  // there and adaptable, w_jl += dt * gain_jl * phi_jl(t - Delta) * s_j
  // for the adapted columns; then delta = clamp(w . phi) with the updated
  // weights. s is the error signal e + T_s edot of this cycle.
  Cycle cycle(const RegressorRow& phi, const JointArray& s, bool adaptable) {
    buffer_.push(phi, adaptable);
    bool updated = false;
    if (const RegressorRow* paired = buffer_.paired()) {
      for (int j = 0; j < kAdaptiveArmJoints; ++j) {
        for (int l = 0; l < kRegressorCount; ++l) {
          if (!gains_.adapted[j][l]) continue;
          w_[j][l] += dt_ * gains_.gain[j][l] * (*paired)[j][l] * s[j];
        }
      }
      updated = true;
    }
    Cycle out = offset(phi);
    out.updated = updated;
    return out;
  }

 private:
  AdaptationGains gains_;
  double dt_;
  double max_offset_;
  AdvanceBuffer buffer_;
  RegressorMatrix w_{};
};

// ------------------------------------------------------------- metrics

// RMS of a series (0 for an empty one).
inline double seriesRms(const std::vector<double>& x) {
  if (x.empty()) return 0.0;
  double sum = 0.0;
  for (const double v : x) sum += v * v;
  return std::sqrt(sum / static_cast<double>(x.size()));
}

// sqrt(mean over the first `joints` joints of rms^2): the RMS over all
// samples of those joints (x7_follow_learn's aggregate).
inline double aggregateRms(const JointArray& rms, int joints) {
  double sum = 0.0;
  for (int j = 0; j < joints; ++j) sum += rms[j] * rms[j];
  return std::sqrt(sum / static_cast<double>(joints));
}

// The upper band of tools/follow_learnability.py band_split: x minus its
// zero-phase lower band, which is the mean plus the inverse real DFT of
// the mean-removed spectrum with every bin above split_hz zeroed (bins at
// split_hz within 1e-9 relative are kept). Direct DFT over the kept bins:
// O(N * kept). Throws std::invalid_argument for an empty series, dt or
// split_hz not finite and > 0, or a non-finite sample.
inline std::vector<double> aboveBand(const std::vector<double>& x, double dt_s,
                                     double split_hz) {
  if (x.empty() || !std::isfinite(dt_s) || dt_s <= 0.0 ||
      !std::isfinite(split_hz) || split_hz <= 0.0) {
    throw std::invalid_argument("aboveBand: invalid arguments");
  }
  const std::size_t n = x.size();
  double mean = 0.0;
  for (const double v : x) {
    if (!std::isfinite(v)) {
      throw std::invalid_argument("aboveBand: non-finite sample");
    }
    mean += v;
  }
  mean /= static_cast<double>(n);
  std::vector<double> centred(n);
  for (std::size_t i = 0; i < n; ++i) centred[i] = x[i] - mean;
  // numpy.fft.rfftfreq: bin k at k / (n dt), k = 0..n/2.
  const double limit = split_hz * (1.0 + 1e-9);
  std::vector<double> below(n, mean);
  const double two_pi_over_n = 2.0 * M_PI / static_cast<double>(n);
  for (std::size_t k = 0; k <= n / 2; ++k) {
    if (static_cast<double>(k) / (static_cast<double>(n) * dt_s) > limit) {
      break;
    }
    double re = 0.0;
    double im = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double angle = two_pi_over_n * static_cast<double>((k * i) % n);
      re += centred[i] * std::cos(angle);
      im -= centred[i] * std::sin(angle);
    }
    // irfft: bin 0 and (even n) bin n/2 once and real, the others twice.
    const bool single = k == 0 || (n % 2 == 0 && k == n / 2);
    const double weight = (single ? 1.0 : 2.0) / static_cast<double>(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double angle = two_pi_over_n * static_cast<double>((k * i) % n);
      below[i] += single
                      ? weight * re * std::cos(angle)
                      : weight * (re * std::cos(angle) - im * std::sin(angle));
    }
  }
  std::vector<double> above(n);
  for (std::size_t i = 0; i < n; ++i) above[i] = x[i] - below[i];
  return above;
}

}  // namespace x7::study
