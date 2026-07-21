#pragma once

#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace rtctrl::arm {

// Computed-torque trajectory tracking in current mode:
//   tau = ID(q_d, dq_d, ddq_d) + Kp e + Kd de
// The inverse-dynamics feedforward realizes the desired motion; the PD
// term absorbs model error. Verified in sim through the bridge before
// any hardware run (M8).
//
// The damping term needs a velocity signal. The Dynamixel's own
// PresentVelocity estimate was measured (x7_track log, 2026-07-21) at
// ~50 ms lag with ~2x attenuation around 2-3 Hz — at those frequencies
// Kd on it stops damping and starts driving, which is what destabilized
// the hardware runs. By default the controller therefore estimates
// velocity host-side from the *fresh* position feedback (backward
// difference + first-order low-pass, ~15 ms total lag);
// useStateVelocity(true) restores the raw state.dq path.
class ComputedTorque : public Controller {
 public:
  ComputedTorque(model::ChainModel& chain, const model::JointMap& map,
                 const model::MinJerkTrajectory& trajectory, double kp,
                 double kd)
      : chain_(chain), map_(map), trajectory_(trajectory), kp_(kp), kd_(kd) {}

  // true: damp on state.dq as-is (ideal sim, or a trusted velocity
  // source); false (default): host-side estimate from positions.
  void useStateVelocity(bool use) { use_state_velocity_ = use; }

  void update(const JointState& state, JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d_.get(), dq_d_.get(), ddq_d_.get());
    chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(), ddq_d_.get(),
                           cmd.tau.get());
    cmd.mode = ControlMode::Current;
    estimateVelocity(state, t);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d_[i] - zVecElemNC(state.q.get(), i);
      const double de = dq_d_[i] - dq_est_[i];
      zVecElemNC(cmd.tau.get(), i) += kp_ * e + kd_ * de;
    }
  }

 private:
  static constexpr double kVelFilterTau = 0.02;  // [s]

  void estimateVelocity(const JointState& state, double t) {
    const double dt = t - t_prev_;
    if (use_state_velocity_ || t_prev_ < 0.0 || dt <= 0.0) {
      zVecCopyNC(state.dq.get(), dq_est_.get());
    } else {
      const double alpha = dt / (kVelFilterTau + dt);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double raw =
            (zVecElemNC(state.q.get(), i) - q_prev_[i]) / dt;
        dq_est_[i] += alpha * (raw - dq_est_[i]);
      }
    }
    zVecCopyNC(state.q.get(), q_prev_.get());
    t_prev_ = t;
  }

  model::ChainModel& chain_;
  const model::JointMap& map_;
  const model::MinJerkTrajectory& trajectory_;
  double kp_;
  double kd_;
  bool use_state_velocity_ = false;
  double t_prev_ = -1.0;
  model::ZVector q_d_{model::kCanonicalDof};
  model::ZVector dq_d_{model::kCanonicalDof};
  model::ZVector ddq_d_{model::kCanonicalDof};
  model::ZVector q_prev_{model::kCanonicalDof};
  model::ZVector dq_est_{model::kCanonicalDof};
};

}  // namespace rtctrl::arm
