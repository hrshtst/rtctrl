#pragma once

#include <algorithm>

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
                 const model::MinJerkTrajectory& trajectory, double kp,
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
  // the wrist in the 2026-07-21 run-4 log); scale them down and let the
  // feedforward plus integrator carry the tracking there.
  void setGainScales(const double* scales) {
    for (int i = 0; i < model::kCanonicalDof; ++i) scale_[i] = scales[i];
  }

  void update(const JointState& state, JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d_.get(), dq_d_.get(), ddq_d_.get());
    chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(), ddq_d_.get(),
                           cmd.tau.get());
    cmd.mode = ControlMode::Current;
    const double dt = t_prev_ < 0.0 ? -1.0 : t - t_prev_;
    estimateVelocity(state, dt);
    const double pd_alpha =
        (pd_tau_ > 0.0 && dt > 0.0) ? dt / (pd_tau_ + dt) : 1.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d_[i] - zVecElemNC(state.q.get(), i);
      const double de = dq_d_[i] - dq_est_[i];
      const double pd = scale_[i] * (kp_ * e + kd_ * de);
      pd_filt_[i] += pd_alpha * (pd - pd_filt_[i]);
      if (ki_ > 0.0 && dt > 0.0) {
        i_term_[i] = std::clamp(i_term_[i] + ki_ * e * dt, -i_clamp_,
                                i_clamp_);
      }
      zVecElemNC(cmd.tau.get(), i) += pd_filt_[i] + i_term_[i];
    }
    zVecCopyNC(state.q.get(), q_prev_.get());
    t_prev_ = t;
  }

 private:
  static constexpr double kVelFilterTau = 0.02;  // [s]

  void estimateVelocity(const JointState& state, double dt) {
    if (use_state_velocity_ || dt <= 0.0) {
      zVecCopyNC(state.dq.get(), dq_est_.get());
      return;
    }
    const double alpha = dt / (kVelFilterTau + dt);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double raw = (zVecElemNC(state.q.get(), i) - q_prev_[i]) / dt;
      dq_est_[i] += alpha * (raw - dq_est_[i]);
    }
  }

  model::ChainModel& chain_;
  const model::JointMap& map_;
  const model::MinJerkTrajectory& trajectory_;
  double kp_;
  double kd_;
  bool use_state_velocity_ = false;
  double pd_tau_ = 0.05;   // [s] — see setPdFilterTau
  double ki_ = 6.0;        // [Nm/(rad s)] — see setIntegral
  double i_clamp_ = 1.5;   // [Nm]
  double t_prev_ = -1.0;
  model::ZVector q_d_{model::kCanonicalDof};
  model::ZVector dq_d_{model::kCanonicalDof};
  model::ZVector ddq_d_{model::kCanonicalDof};
  model::ZVector q_prev_{model::kCanonicalDof};
  model::ZVector dq_est_{model::kCanonicalDof};
  model::ZVector pd_filt_{model::kCanonicalDof};
  model::ZVector i_term_{model::kCanonicalDof};
  model::ZVector scale_{model::kCanonicalDof};
};

}  // namespace rtctrl::arm
