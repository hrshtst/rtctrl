#pragma once

#include <zm/zm.h>

#include "rtctrl/model/zvector.hpp"

namespace rtctrl::model {

// A sampleable joint-space reference: position (and optionally
// velocity/acceleration) as a function of time. Controllers hold
// references through this interface so composite references (e.g. a
// round trip) keep one controller instance alive across segments.
class Trajectory {
 public:
  virtual ~Trajectory() = default;
  virtual int size() const = 0;
  virtual double duration() const = 0;
  // q must match size(); dq/ddq may be null when not needed.
  virtual void sample(double t, zVec q, zVec dq = nullptr,
                      zVec ddq = nullptr) const = 0;
};

// Fifth-order (minimum-jerk) point-to-point joint trajectory with zero
// velocity and acceleration at both endpoints. Sampling outside
// [0, duration] clamps to the endpoints.
class MinJerkTrajectory : public Trajectory {
 public:
  MinJerkTrajectory(const zVec q0, const zVec qf, double duration);

  // Chooses the duration so that the peak joint speed (15/8 · |Δq|/T for
  // the quintic) stays within velocity_limit on every joint.
  static MinJerkTrajectory withVelocityLimit(const zVec q0, const zVec qf,
                                             double velocity_limit,
                                             double min_duration = 0.5);

  double duration() const override { return duration_; }
  int size() const override { return q0_.size(); }

  // q must match the trajectory size; dq/ddq may be null when not
  // needed.
  void sample(double t, zVec q, zVec dq = nullptr,
              zVec ddq = nullptr) const override;

 private:
  ZVector q0_;
  ZVector qf_;
  double duration_;
};

// Two minimum-jerk segments played back to back (out, then back). Both
// segments have zero velocity/acceleration at their endpoints and must
// agree on the split position, so the composite is C²-continuous — a
// single controller instance tracks the whole round trip with no state
// reset at the turnaround.
class RoundTripTrajectory : public Trajectory {
 public:
  // Segments are stored by value (safe to pass temporaries). Throws if
  // sizes differ or out's endpoint does not match back's start.
  RoundTripTrajectory(MinJerkTrajectory out, MinJerkTrajectory back);

  double outDuration() const { return out_.duration(); }
  double duration() const override {
    return out_.duration() + back_.duration();
  }
  int size() const override { return out_.size(); }
  void sample(double t, zVec q, zVec dq = nullptr,
              zVec ddq = nullptr) const override;

 private:
  MinJerkTrajectory out_;
  MinJerkTrajectory back_;
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
