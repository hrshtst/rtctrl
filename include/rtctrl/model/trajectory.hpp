#pragma once

#include <zm/zm.h>

#include "rtctrl/model/zvector.hpp"

namespace rtctrl::model {

// Fifth-order (minimum-jerk) point-to-point joint trajectory with zero
// velocity and acceleration at both endpoints. Sampling outside
// [0, duration] clamps to the endpoints.
class MinJerkTrajectory {
 public:
  MinJerkTrajectory(const zVec q0, const zVec qf, double duration);

  // Chooses the duration so that the peak joint speed (15/8 · |Δq|/T for
  // the quintic) stays within velocity_limit on every joint.
  static MinJerkTrajectory withVelocityLimit(const zVec q0, const zVec qf,
                                             double velocity_limit,
                                             double min_duration = 0.5);

  double duration() const { return duration_; }
  int size() const { return q0_.size(); }

  // q must match the trajectory size; dq/ddq may be null when not
  // needed.
  void sample(double t, zVec q, zVec dq = nullptr,
              zVec ddq = nullptr) const;

 private:
  ZVector q0_;
  ZVector qf_;
  double duration_;
};

// Per-joint sinusoid around a center pose: q = c + A sin(2π t / period).
class SinusoidTrajectory {
 public:
  SinusoidTrajectory(const zVec center, const zVec amplitude, double period);

  int size() const { return center_.size(); }
  void sample(double t, zVec q, zVec dq = nullptr) const;

 private:
  ZVector center_;
  ZVector amplitude_;
  double period_;
};

}  // namespace rtctrl::model
