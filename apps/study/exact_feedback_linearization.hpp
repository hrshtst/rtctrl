// Offline EFL study controllers (docs/records/history.md (EFL study),
// docs/records/history.md (EFL frozen specification)). Experimental code — consumed
// only by x7_efl_study and its tests, never by the hardware apps.
//
//  * HostVelocityEstimator — a standalone duplicate of PracticalComputedTorque's
//    host-side estimator. Deliberately NOT shared code: the study
//    definition requires parity to be unit-tested against
//    PracticalComputedTorque::velocityEstimate(), not assumed
//    (tests/unit/efl_test.cpp).
//  * AccelDomainController — the acceleration-domain family:
//      v = q̈_d + K_d'(q̇_d − v̂) + K_p'(q_d − q)
//      DESIRED-host:  τ = ID(q_d, q̇_d, v)     (desired-state model)
//      EFL-host:      τ = ID(q,   v̂,   v)     (measured-state model)
//      EFL-ideal:     EFL with exact state.dq as v̂ (diagnostic only)
//    One RNEA call per cycle in every mode.
//  * PracticalReplica — the shipped practical law, replicated so the
//    flexible-mode comparator PRACTICAL-GF can remove model gravity
//    from the feedforward BEFORE anti-windup and the final clamp
//    (subtracting after PracticalComputedTorque's internal clamp would corrupt
//    the saturation telemetry, let anti-windup react to fictitious
//    gravity, and break the common limit — review finding). Ordinary
//    (gravity-on) mode is parity-tested against PracticalComputedTorque.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7 {

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

// A fixed-pose reference (the flexible screens' zero reference).
class ConstantTrajectory : public model::Trajectory {
 public:
  explicit ConstantTrajectory(const model::ZVector& q) : q_(q) {}
  int size() const override { return q_.size(); }
  double duration() const override { return 0.0; }
  void sample(double, zVec q, zVec dq = nullptr,
              zVec ddq = nullptr) const override {
    zVecCopyNC(q_.get(), q);
    if (dq != nullptr) zVecZero(dq);
    if (ddq != nullptr) zVecZero(ddq);
  }

 private:
  model::ZVector q_;
};

// Standalone duplicate of PracticalComputedTorque's host velocity estimator:
// first sample records positions with a zero estimate; raw-dt backward
// difference; EMA with alpha = dt_f/(0.02 + dt_f), dt_f = min(dt,
// 3*nominal_dt); duplicate timestamps hold all state.
class HostVelocityEstimator {
 public:
  enum class Sample { First, Duplicate, Updated };
  static constexpr double kVelFilterTau = 0.02;  // [s]

  explicit HostVelocityEstimator(double nominal_dt)
      : nominal_dt_(nominal_dt) {}

  Sample update(const zVec q, double t) {
    if (t_prev_ < 0.0) {
      zVecCopyNC(q, q_prev_.get());
      t_prev_ = t;
      return Sample::First;
    }
    const double dt = t - t_prev_;
    if (dt <= 0.0) return Sample::Duplicate;
    const double dt_f = std::min(dt, 3.0 * nominal_dt_);
    const double alpha_v = dt_f / (kVelFilterTau + dt_f);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double raw =
          (zVecElemNC(q, i) - q_prev_[i]) / dt;  // raw dt
      dq_est_[i] += alpha_v * (raw - dq_est_[i]);
      q_prev_[i] = zVecElemNC(q, i);
    }
    t_prev_ = t;
    return Sample::Updated;
  }

  double estimate(int i) const { return dq_est_[i]; }
  const model::ZVector& estimateVector() const { return dq_est_; }

 private:
  double nominal_dt_;
  double t_prev_ = -1.0;
  model::ZVector q_prev_{model::kCanonicalDof};
  model::ZVector dq_est_{model::kCanonicalDof};
};

struct AccelDomainOptions {
  enum class Eval { Desired, Measured };
  Eval eval = Eval::Measured;
  bool state_velocity = false;  // true: exact state.dq (EFL-ideal)
  bool gravity_free = false;    // subtract model gravity (F screens)
  double nominal_dt = 0.01;
  const double* tau_max = nullptr;  // per-joint clamp [Nm]; may be null
};

class AccelDomainController : public arm::Controller {
 public:
  AccelDomainController(model::ChainModel& chain,
                        const model::JointMap& map,
                        const model::Trajectory& trajectory,
                        double kp_prime, double kd_prime,
                        AccelDomainOptions options)
      : chain_(chain), map_(map), trajectory_(trajectory),
        kp_(kp_prime), kd_(kd_prime), opt_(options),
        est_(options.nominal_dt) {
    if (opt_.tau_max != nullptr) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        tau_max_[i] = opt_.tau_max[i];
      }
    }
  }

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d_.get(), dq_d_.get(), ddq_d_.get());
    cmd.mode = arm::ControlMode::Current;

    // One shared cycle gate (mirrors the estimator's) so both velocity
    // sources get the identical soft-start and duplicate handling.
    const auto est_status = est_.update(state.q.get(), t);
    if (est_status == HostVelocityEstimator::Sample::First) {
      // Soft start: pure desired feedforward, no feedback — exactly
      // the practical controller's first-sample behavior.
      chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(),
                             ddq_d_.get(), tau_raw_.get());
      if (opt_.gravity_free) subtractGravity(q_d_.get());
      emitOutput(cmd);
      return;
    }
    if (est_status == HostVelocityEstimator::Sample::Duplicate) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        zVecElemNC(cmd.tau.get(), i) = tau_cmd_[i];
      }
      return;
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      v_est_[i] = opt_.state_velocity ? zVecElemNC(state.dq.get(), i)
                                      : est_.estimate(i);
      const double e = q_d_[i] - zVecElemNC(state.q.get(), i);
      const double de = dq_d_[i] - v_est_[i];
      v_[i] = ddq_d_[i] + kd_ * de + kp_ * e;
    }
    if (opt_.eval == AccelDomainOptions::Eval::Measured) {
      chain_.inverseDynamics(map_, state.q.get(), v_est_.get(),
                             v_.get(), tau_raw_.get());
      if (opt_.gravity_free) subtractGravity(state.q.get());
    } else {
      chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(), v_.get(),
                             tau_raw_.get());
      if (opt_.gravity_free) subtractGravity(q_d_.get());
    }
    emitOutput(cmd);
  }

  const model::ZVector& tauCommanded() const { return tau_cmd_; }
  const model::ZVector& velocityEstimate() const { return v_est_; }
  bool controllerSaturated(int i) const { return sat_[i] != 0; }

 private:
  void subtractGravity(const zVec q) {
    chain_.gravityTorque(map_, q, g_.get());
    for (int i = 0; i < model::kCanonicalDof; ++i) tau_raw_[i] -= g_[i];
  }

  // Identical clamp semantics to PracticalComputedTorque::emitOutput.
  void emitOutput(arm::JointCommand& cmd) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      double out = tau_raw_[i];
      sat_[i] = 0;
      if (opt_.tau_max != nullptr) {
        const double clamped = std::clamp(out, -tau_max_[i], tau_max_[i]);
        if (clamped != out) sat_[i] = 1;
        out = clamped;
      }
      tau_cmd_[i] = out;
      zVecElemNC(cmd.tau.get(), i) = out;
    }
  }

  model::ChainModel& chain_;
  const model::JointMap& map_;
  const model::Trajectory& trajectory_;
  double kp_;
  double kd_;
  AccelDomainOptions opt_;
  HostVelocityEstimator est_;
  model::ZVector q_d_{model::kCanonicalDof};
  model::ZVector dq_d_{model::kCanonicalDof};
  model::ZVector ddq_d_{model::kCanonicalDof};
  model::ZVector v_est_{model::kCanonicalDof};
  model::ZVector v_{model::kCanonicalDof};
  model::ZVector g_{model::kCanonicalDof};
  model::ZVector tau_raw_{model::kCanonicalDof};
  model::ZVector tau_cmd_{model::kCanonicalDof};
  model::ZVector tau_max_{model::kCanonicalDof};
  std::uint8_t sat_[model::kCanonicalDof] = {};
};

// The shipped practical law, replicated statement-for-statement from
// arm::PracticalComputedTorque (defaults included), plus the ONE study change:
// with gravity_free, model gravity leaves the feedforward BEFORE the
// integrator's anti-windup and the final clamp. Ordinary mode is
// parity-tested against PracticalComputedTorque in tests/unit/efl_test.cpp —
// any divergence from the shipped controller is a test failure, not a
// silent drift.
class PracticalReplica : public arm::Controller {
 public:
  PracticalReplica(model::ChainModel& chain, const model::JointMap& map,
                   const model::Trajectory& trajectory, double kp,
                   double kd, bool gravity_free)
      : chain_(chain), map_(map), trajectory_(trajectory), kp_(kp),
        kd_(kd), gravity_free_(gravity_free) {
    for (int i = 0; i < model::kCanonicalDof; ++i) scale_[i] = 1.0;
  }

  void setPdFilterTau(double tau_s) { pd_tau_ = tau_s; }
  void setIntegral(double ki, double clamp_nm) {
    ki_ = ki;
    i_clamp_ = clamp_nm;
  }
  void setGainScales(const double* scales) {
    for (int i = 0; i < model::kCanonicalDof; ++i) scale_[i] = scales[i];
  }
  void setNominalDt(double dt_s) { nominal_dt_ = dt_s; }
  void setTorqueLimits(const double* tau_max) {
    for (int i = 0; i < model::kCanonicalDof; ++i) tau_max_[i] = tau_max[i];
    has_limits_ = true;
  }

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d_.get(), dq_d_.get(), ddq_d_.get());
    chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(), ddq_d_.get(),
                           ff_.get());
    if (gravity_free_) {
      // The study change: gravity leaves the FEEDFORWARD, so the
      // anti-windup candidates and the clamp below see gravity-free
      // torques throughout.
      chain_.gravityTorque(map_, q_d_.get(), g_.get());
      for (int i = 0; i < model::kCanonicalDof; ++i) ff_[i] -= g_[i];
    }
    cmd.mode = arm::ControlMode::Current;

    if (t_prev_ < 0.0) {
      zVecCopyNC(state.q.get(), q_prev_.get());
      t_prev_ = t;
      emitOutput(cmd);
      return;
    }
    const double dt = t - t_prev_;
    if (dt <= 0.0) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        zVecElemNC(cmd.tau.get(), i) = tau_cmd_[i];
      }
      return;
    }
    const double dt_f = std::min(dt, 3.0 * nominal_dt_);

    const double alpha_v = dt_f / (kVelFilterTau + dt_f);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double raw =
          (zVecElemNC(state.q.get(), i) - q_prev_[i]) / dt;  // raw dt
      dq_est_[i] += alpha_v * (raw - dq_est_[i]);
    }
    const double pd_alpha =
        pd_tau_ <= 0.0 ? 1.0 : dt_f / (pd_tau_ + dt_f);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d_[i] - zVecElemNC(state.q.get(), i);
      const double de = dq_d_[i] - dq_est_[i];
      const double pd = scale_[i] * (kp_ * e + kd_ * de);
      pd_filt_[i] += pd_alpha * (pd - pd_filt_[i]);
      if (ki_ > 0.0) {
        const double i_cand = std::clamp(i_term_[i] + ki_ * e * dt_f,
                                         -i_clamp_, i_clamp_);
        const double lim = has_limits_ ? tau_max_[i] : 1e300;
        const double tau_cand = ff_[i] + pd_filt_[i] + i_cand;
        const bool deeper_pos = tau_cand > lim && i_cand > i_term_[i];
        const bool deeper_neg = tau_cand < -lim && i_cand < i_term_[i];
        if (!deeper_pos && !deeper_neg) i_term_[i] = i_cand;
      }
    }
    zVecCopyNC(state.q.get(), q_prev_.get());
    t_prev_ = t;
    emitOutput(cmd);
  }

  const model::ZVector& integralTerm() const { return i_term_; }
  const model::ZVector& velocityEstimate() const { return dq_est_; }
  const model::ZVector& tauCommanded() const { return tau_cmd_; }
  bool controllerSaturated(int i) const { return sat_[i] != 0; }

 private:
  static constexpr double kVelFilterTau = 0.02;  // [s]

  void emitOutput(arm::JointCommand& cmd) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      double out = ff_[i] + pd_filt_[i] + i_term_[i];
      sat_[i] = 0;
      if (has_limits_) {
        const double clamped = std::clamp(out, -tau_max_[i], tau_max_[i]);
        if (clamped != out) sat_[i] = 1;
        out = clamped;
      }
      tau_cmd_[i] = out;
      zVecElemNC(cmd.tau.get(), i) = out;
    }
  }

  model::ChainModel& chain_;
  const model::JointMap& map_;
  const model::Trajectory& trajectory_;
  double kp_;
  double kd_;
  bool gravity_free_;
  double pd_tau_ = 0.05;      // [s] — practical default
  double ki_ = 6.0;           // [Nm/(rad s)] — practical default
  double i_clamp_ = 1.5;      // [Nm] — practical default
  double nominal_dt_ = 0.01;  // [s]
  bool has_limits_ = false;
  double t_prev_ = -1.0;
  model::ZVector q_d_{model::kCanonicalDof};
  model::ZVector dq_d_{model::kCanonicalDof};
  model::ZVector ddq_d_{model::kCanonicalDof};
  model::ZVector q_prev_{model::kCanonicalDof};
  model::ZVector dq_est_{model::kCanonicalDof};
  model::ZVector pd_filt_{model::kCanonicalDof};
  model::ZVector i_term_{model::kCanonicalDof};
  model::ZVector scale_{model::kCanonicalDof};
  model::ZVector ff_{model::kCanonicalDof};
  model::ZVector g_{model::kCanonicalDof};
  model::ZVector tau_cmd_{model::kCanonicalDof};
  model::ZVector tau_max_{model::kCanonicalDof};
  std::uint8_t sat_[model::kCanonicalDof] = {};
};

}  // namespace x7
