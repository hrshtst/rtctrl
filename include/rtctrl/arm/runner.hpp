#pragma once

#include "rtctrl/arm/arm.hpp"

namespace rtctrl::arm {

// A controller receives the current state and fills in the command for
// this cycle. t starts at 0 when the run begins.
class Controller {
 public:
  virtual ~Controller() = default;
  virtual void update(const JointState& state, JointCommand& cmd,
                      double t) = 0;
};

// Drives arm for `duration` seconds of control cycles:
// readState → controller.update → writeCommand → step.
// Returns false as soon as any arm call fails.
inline bool run(Arm& arm, Controller& controller, double duration) {
  JointState state;
  JointCommand cmd;
  const double dt = arm.dt();
  for (double t = 0.0; t < duration; t += dt) {
    if (!arm.readState(state)) return false;
    controller.update(state, cmd, t);
    if (!arm.writeCommand(cmd)) return false;
    if (!arm.step()) return false;
  }
  return true;
}

}  // namespace rtctrl::arm
