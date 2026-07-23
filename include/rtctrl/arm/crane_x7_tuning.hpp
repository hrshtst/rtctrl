#pragma once

#include "rtctrl/model/joint_map.hpp"

// The CRANE-X7's shipped computed-torque tuning — ONE source of truth
// consumed by the hardware app, the sim twin, and the tests (tests link
// only the library; a copy in apps/ is how the shipped configuration
// once went untested). Every value was set by a logged hardware
// finding; see docs/theory/computed-torque.md ("what the hardware
// taught us") before changing any of them.
namespace rtctrl::arm::tuning {

// PD stiffness/damping [Nm/rad, Nms/rad]. Bounded above by the 50 ms
// PD low-pass: the loop crossover sqrt(Kp/J) must stay below the
// filter pole (Kp ~ 6 at the shoulder's J ~ 0.1 kg m^2 — NOT the 20 an
// ideal rigid sim tolerates).
inline constexpr double kKp = 6.0;
inline constexpr double kKd = 1.0;

// Slow integrator absorbing the ~1 Nm unmodeled friction the modest Kp
// cannot hide; the clamp bounds windup against the torque limits.
inline constexpr double kKi = 6.0;              // [Nm/(rad s)]
inline constexpr double kIntegralClampNm = 1.5;  // [Nm]

// PD-correction low-pass (removes loop gain at the ~13 Hz gear-train
// resonance; the smooth feedforward passes unfiltered).
inline constexpr double kPdFilterTau = 0.05;  // [s]

// Nominal control period — ComputedTorque::setNominalDt (bounds the
// filter/integrator dt at 3x nominal). Matches
// hw::CraneX7::Options::control_cycle_s.
inline constexpr double kNominalDt = 0.01;  // [s]

// Damping gain of the pre-tracking settle phase (gravity comp + light
// filtered damping until quiescence).
inline constexpr double kSettleKd = 0.8;  // [Nms/rad]

// Per-joint PD scale: proximal joints take the full gains; distal
// joints, whose link-side inertia is a small fraction of the
// shoulder's, take a fraction to stay clear of backlash limit cycles.
// The forearm TWIST gets the smallest scale of all: the hand's mass
// sits nearly on its axis (J ~ 1e-3 kg m^2), and at 0.35 it rang at
// ~5 Hz (track5.csv log).
inline constexpr double kGainScale[model::kCanonicalDof] = {
    1.0, 1.0, 0.7, 0.7, 0.1, 0.3, 0.2, 0.2};

}  // namespace rtctrl::arm::tuning
