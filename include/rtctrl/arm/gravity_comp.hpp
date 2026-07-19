#pragma once

#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::arm {

// Pure gravity compensation in current (torque) mode: the arm floats —
// it holds against gravity but can be pushed around freely. The same
// controller verifies on SimArm first, then runs on the real robot
// through RealArm (the bridge converts torque to current per model).
class GravityComp : public Controller {
 public:
  GravityComp(model::ChainModel& chain, const model::JointMap& map)
      : chain_(chain), map_(map) {}

  void update(const JointState& state, JointCommand& cmd,
              double t) override {
    (void)t;
    cmd.mode = ControlMode::Current;
    chain_.gravityTorque(map_, state.q.get(), cmd.tau.get());
  }

 private:
  model::ChainModel& chain_;
  const model::JointMap& map_;
};

}  // namespace rtctrl::arm
