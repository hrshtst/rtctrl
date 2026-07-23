#pragma once

#include <algorithm>
#include <cstdint>

#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace rtctrl::arm {

// Computed-torque trajectory tracking in current mode:
//   tau = ID(q_d, dq_d, ddq_d) + LP(Kp e + Kd de) + clamp(Ki int(e))
// The inverse-dynamics feedforward realizes the desired motion; the
// filtered PD absorbs dynamic model error; the slow clamped integrator
// absorbs static friction/model error. Verified in sim through the
// bridge before any hardware run (M8).
//
// Every non-default term exists because a 2026-07-21 hardware run
// failed without it:
//  * Host-side velocity (default): the servo's PresentVelocity lags
//    ~50 ms with ~2x attenuation — Kd on it drives instead of damps.
//    The default estimates velocity from the fresh position feedback
//    (backward difference + low-pass, ~15 ms); useStateVelocity(true)
//    restores the raw state.dq path.
//  * PD low-pass (default 0.05 s): the gear trains resonate at ~13 Hz,
//    where the 100 Hz bus loop's delay makes all feedback pump the
//    mode. The filter removes loop gain there. It also caps usable
//    stiffness: keep sqrt(Kp/J) well below 1/tau (Kp ~ 6 at the
//    shoulder's J ~ 0.1 kg m^2, NOT the 20 an unfiltered ideal loop
//    tolerates).
//  * Integrator (default Ki 6, clamp 1.5 Nm): ~1 Nm of unmodeled
//    friction sags the arm by tau_f/Kp, which the modest Kp above
//    cannot hide. The clamp bounds windup against the torque limits.
class ComputedTorque : public Controller {
 public:
  ComputedTorque(model::ChainModel& chain, const model::JointMap& map,
                 const model::Trajectory& trajectory, double kp,
                 double kd)
      : chain_(chain), map_(map), trajectory_(trajectory), kp_(kp), kd_(kd) {
    for (int i = 0; i < model::kCanonicalDof; ++i) scale_[i] = 1.0;
  }

  // true: damp on state.dq as-is (ideal sim, or a trusted velocity
  // source); false (default): host-side estimate from positions.
  void useStateVelocity(bool use) { use_state_velocity_ = use; }

  // PD low-pass time constant; 0 disables (see class comment).
  void setPdFilterTau(double tau_s) { pd_tau_ = tau_s; }
  // Integral gain [Nm/(rad s)] and windup clamp [Nm]; Ki 0 disables.
  void setIntegral(double ki, double clamp_nm) {
    ki_ = ki;
    i_clamp_ = clamp_nm;
  }
  // Per-joint multiplier on the PD gains (not the integrator). Distal
  // joints carry a fraction of the shoulder's link-side inertia and hit
  // backlash limit cycles at gains the proximal joints need (~5 Hz on
  // the wrist in the 2026-07-21 track3.csv log); scale them down and
  // let the feedforward plus integrator carry the tracking there.
  void setGainScales(const double* scales) {
    for (int i = 0; i < model::kCanonicalDof; ++i) scale_[i] = scales[i];
  }
  // Nominal control period [s] — bounds the dt used by the filters and
  // the integrator to 3x nominal (a one-off stall must not defeat the
  // PD low-pass or triple the integration step; the raw dt still
  // serves the velocity backward difference, where it is correct).
  // Explicit rather than guessed from Arm::dt(); apps pass
  // crane_x7_tuning.hpp's kNominalDt.
  void setNominalDt(double dt_s) { nominal_dt_ = dt_s; }
  // Per-joint torque limits [Nm], mirroring the hardware clamp exactly
  // (max(0, min(effort, kt*servo_limit) - margin*kt)). Enables the
  // direction-aware anti-windup and the controller-side output clamp.
  void setTorqueLimits(const double* tau_max) {
    for (int i = 0; i < model::kCanonicalDof; ++i) tau_max_[i] = tau_max[i];
    has_limits_ = true;
  }

  void update(const JointState& state, JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d_.get(), dq_d_.get(), ddq_d_.get());
    chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(), ddq_d_.get(),
                           ff_.get());
    cmd.mode = ControlMode::Current;

    if (t_prev_ < 0.0) {
      // First sample: record positions and emit feedforward only. The
      // velocity estimate stays zero (never seeded from the servo's
      // lagged dq — it would contaminate the filtered samples after
      // it) and no PD/I applies (soft start: a controller constructed
      // mid-motion must not step the residual error into the torque).
      zVecCopyNC(state.q.get(), q_prev_.get());
      t_prev_ = t;
      emitOutput(cmd);
      return;
    }
    const double dt = t - t_prev_;
    if (dt <= 0.0) {
      // Duplicate sample (defense in depth — the runner skips these):
      // hold ALL state, re-emit the previously computed command.
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        zVecElemNC(cmd.tau.get(), i) = tau_cmd_[i];
      }
      return;
    }
    const double dt_f = std::min(dt, 3.0 * nominal_dt_);

    if (use_state_velocity_) {
      zVecCopyNC(state.dq.get(), dq_est_.get());
    } else {
      const double alpha_v = dt_f / (kVelFilterTau + dt_f);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double raw =
            (zVecElemNC(state.q.get(), i) - q_prev_[i]) / dt;  // raw dt
        dq_est_[i] += alpha_v * (raw - dq_est_[i]);
      }
    }
    const double pd_alpha =
        pd_tau_ <= 0.0 ? 1.0 : dt_f / (pd_tau_ + dt_f);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d_[i] - zVecElemNC(state.q.get(), i);
      const double de = dq_d_[i] - dq_est_[i];
      const double pd = scale_[i] * (kp_ * e + kd_ * de);
      pd_filt_[i] += pd_alpha * (pd - pd_filt_[i]);
      // Direction-aware anti-windup via the candidate update: reject
      // the integration only when the CANDIDATE command is saturated
      // AND the update drives farther into that saturation; the
      // unwinding direction always commits. The emitted command is
      // then recomputed from the COMMITTED state — a rejected
      // candidate never leaks into the output.
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

  // Per-cycle telemetry (docs/REMEDIATION_PLAN.md D8): feedforward,
  // filtered PD, committed integral, host velocity estimate, the
  // pre-/post-clamp commands and the controller-side saturation flags
  // (controller clamping would otherwise hide saturation from the
  // hardware's writer, which then sees an already-limited value).
  const model::ZVector& feedforward() const { return ff_; }
  const model::ZVector& pdTerm() const { return pd_filt_; }
  const model::ZVector& integralTerm() const { return i_term_; }
  const model::ZVector& velocityEstimate() const { return dq_est_; }
  const model::ZVector& tauRaw() const { return tau_raw_; }
  const model::ZVector& tauCommanded() const { return tau_cmd_; }
  bool controllerSaturated(int i) const { return sat_[i] != 0; }

 private:
  static constexpr double kVelFilterTau = 0.02;  // [s]

  // tau_raw = ff + pd + committed i (always, by construction);
  // tau_commanded clamps it with exactly the hardware's limits.
  void emitOutput(JointCommand& cmd) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      tau_raw_[i] = ff_[i] + pd_filt_[i] + i_term_[i];
      double out = tau_raw_[i];
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
  bool use_state_velocity_ = false;
  double pd_tau_ = 0.05;      // [s] — see setPdFilterTau
  double ki_ = 6.0;           // [Nm/(rad s)] — see setIntegral
  double i_clamp_ = 1.5;      // [Nm]
  double nominal_dt_ = 0.01;  // [s] — see setNominalDt
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
  model::ZVector tau_raw_{model::kCanonicalDof};
  model::ZVector tau_cmd_{model::kCanonicalDof};
  model::ZVector tau_max_{model::kCanonicalDof};
  std::uint8_t sat_[model::kCanonicalDof] = {};
};

}  // namespace rtctrl::arm
