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

  // Torque on; clamps the goal to the present posture so activation
  // causes no motion. On real hardware this also arms safety (soft
  // gains, servo Bus Watchdog).
  virtual bool activate() = 0;
  // Zero commands, torque off. NOT an emergency stop.
  virtual bool deactivate() = 0;
  // Only while deactivated.
  virtual bool setMode(ControlMode mode) = 0;

  virtual bool readState(JointState& state) = 0;
  virtual bool writeCommand(const JointCommand& cmd) = 0;
  // Advance one control period: sim integrates dt(); real HW blocks on
  // the read-write cycle tick.
  virtual bool step() = 0;
};

}  // namespace rtctrl::arm
