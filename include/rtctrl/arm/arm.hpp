#pragma once

#include "rtctrl/arm/types.hpp"

namespace rtctrl::arm {

// The sim⇄real bridge: a controller written against this interface runs
// unchanged on the roki-fd simulator (SimArm) and the real robot
// (CraneX7Hardware). One cycle is readState → writeCommand → step.
class Arm {
 public:
  virtual ~Arm() = default;

  virtual int dof() const = 0;
  virtual double dt() const = 0;  // control period [s]

  // Torque on. Activation never COMMANDS motion, but only position
  // mode holds the arm: goals snap to the present posture there. In
  // current mode the initial goal is ZERO current — the arm is
  // unsupported and can fall under gravity until the controller's
  // first command, unless a preload was staged (see
  // CraneX7::setActivationCurrentPreload). On real hardware this also
  // arms safety (active gains, servo Bus Watchdog).
  virtual bool activate() = 0;
  // Zero commands, torque off. NOT an emergency stop.
  virtual bool deactivate() = 0;
  // Only while deactivated.
  virtual bool setMode(ControlMode mode) = 0;

  // When `cmds` is non-null it is filled ATOMICALLY with the state
  // (one producer-side lock hold): the applied-target and write-attempt
  // records as of this feedback sample.
  virtual bool readState(JointState& state,
                         CommandSnapshot* cmds = nullptr) = 0;
  // When `receipt` is non-null it reports the submission this call
  // became (sequence + submission time on the producer's clock).
  virtual bool writeCommand(const JointCommand& cmd,
                            CommandReceipt* receipt = nullptr) = 0;
  // Advance one control period: sim integrates dt(); real HW blocks on
  // the read-write cycle tick.
  virtual bool step() = 0;
};

}  // namespace rtctrl::arm
