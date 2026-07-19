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
class ComputedTorque : public Controller {
 public:
  ComputedTorque(model::ChainModel& chain, const model::JointMap& map,
                 const model::MinJerkTrajectory& trajectory, double kp,
                 double kd)
      : chain_(chain), map_(map), trajectory_(trajectory), kp_(kp), kd_(kd) {}

  void update(const JointState& state, JointCommand& cmd,
              double t) override {
    trajectory_.sample(t, q_d_.get(), dq_d_.get(), ddq_d_.get());
    chain_.inverseDynamics(map_, q_d_.get(), dq_d_.get(), ddq_d_.get(),
                           cmd.tau.get());
    cmd.mode = ControlMode::Current;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d_[i] - zVecElemNC(state.q.get(), i);
      const double de = dq_d_[i] - zVecElemNC(state.dq.get(), i);
      zVecElemNC(cmd.tau.get(), i) += kp_ * e + kd_ * de;
    }
  }

 private:
  model::ChainModel& chain_;
  const model::JointMap& map_;
  const model::MinJerkTrajectory& trajectory_;
  double kp_;
  double kd_;
  model::ZVector q_d_{model::kCanonicalDof};
  model::ZVector dq_d_{model::kCanonicalDof};
  model::ZVector ddq_d_{model::kCanonicalDof};
};

}  // namespace rtctrl::arm
