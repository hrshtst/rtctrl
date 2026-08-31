#pragma once

#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace rtctrl::arm {

// Textbook inverse-dynamics feedforward with torque-space PD feedback:
//   tau = ID(q_d, dq_d, ddq_d) + Kp (q_d - q) + Kd (dq_d - dq)
//
// This controller deliberately uses JointState::dq as supplied and has no
// filtering, integral action, gain scheduling, soft start, or output clamp.
// Actuator limits remain the Arm implementation's responsibility.
class ComputedTorque : public Controller {
 public:
  ComputedTorque(model::ChainModel& chain, const model::JointMap& map,
                 const model::Trajectory& trajectory, double kp, double kd)
      : chain_(chain), map_(map), trajectory_(trajectory), kp_(kp), kd_(kd) {}

  void update(const JointState& state, JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d_.get(), dq_d_.get(), ddq_d_.get());
    chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(), ddq_d_.get(),
                           ff_.get());
    cmd.mode = ControlMode::Current;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      pd_[i] = kp_ * (q_d_[i] - zVecElemNC(state.q.get(), i)) +
               kd_ * (dq_d_[i] - zVecElemNC(state.dq.get(), i));
      zVecElemNC(cmd.tau.get(), i) = ff_[i] + pd_[i];
    }
  }

  const model::ZVector& desiredPosition() const { return q_d_; }
  const model::ZVector& desiredVelocity() const { return dq_d_; }
  const model::ZVector& desiredAcceleration() const { return ddq_d_; }
  const model::ZVector& feedforward() const { return ff_; }
  const model::ZVector& feedback() const { return pd_; }

 private:
  model::ChainModel& chain_;
  const model::JointMap& map_;
  const model::Trajectory& trajectory_;
  double kp_;
  double kd_;
  model::ZVector q_d_{model::kCanonicalDof};
  model::ZVector dq_d_{model::kCanonicalDof};
  model::ZVector ddq_d_{model::kCanonicalDof};
  model::ZVector ff_{model::kCanonicalDof};
  model::ZVector pd_{model::kCanonicalDof};
};

}  // namespace rtctrl::arm
