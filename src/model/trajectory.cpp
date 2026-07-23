#include "rtctrl/model/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rtctrl::model {

namespace {

// peak of s'(tau) = 30 tau^2 (1 - tau)^2 over [0,1]
constexpr double kQuinticPeakSpeedFactor = 15.0 / 8.0;

void requireMatch(const zVec a, const zVec b, const char* what) {
  if (a == nullptr || b == nullptr || zVecSizeNC(a) != zVecSizeNC(b)) {
    throw std::invalid_argument(std::string("trajectory: ") + what +
                                ": vector sizes must match");
  }
}

}  // namespace

MinJerkTrajectory::MinJerkTrajectory(const zVec q0, const zVec qf,
                                     double duration)
    : q0_(zVecSizeNC(q0)), qf_(zVecSizeNC(qf)), duration_(duration) {
  requireMatch(q0, qf, "MinJerkTrajectory");
  if (duration <= 0.0) {
    throw std::invalid_argument("MinJerkTrajectory: duration must be > 0");
  }
  zVecCopyNC(q0, q0_.get());
  zVecCopyNC(qf, qf_.get());
}

MinJerkTrajectory MinJerkTrajectory::withVelocityLimit(const zVec q0,
                                                       const zVec qf,
                                                       double velocity_limit,
                                                       double min_duration) {
  requireMatch(q0, qf, "withVelocityLimit");
  if (velocity_limit <= 0.0) {
    throw std::invalid_argument(
        "MinJerkTrajectory: velocity_limit must be > 0");
  }
  double max_delta = 0.0;
  for (int i = 0; i < zVecSizeNC(q0); ++i) {
    max_delta = std::max(max_delta,
                         std::fabs(zVecElemNC(qf, i) - zVecElemNC(q0, i)));
  }
  const double duration = std::max(
      min_duration, kQuinticPeakSpeedFactor * max_delta / velocity_limit);
  return MinJerkTrajectory(q0, qf, duration);
}

void MinJerkTrajectory::sample(double t, zVec q, zVec dq, zVec ddq) const {
  requireMatch(q, q0_.get(), "sample");
  if (dq != nullptr) requireMatch(dq, q0_.get(), "sample dq");
  if (ddq != nullptr) requireMatch(ddq, q0_.get(), "sample ddq");

  const double tau = std::clamp(t / duration_, 0.0, 1.0);
  const double s = tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
  const double sq = tau * (1.0 - tau);
  const double ds = 30.0 * sq * sq / duration_;
  const double dds =
      (60.0 * tau - 180.0 * tau * tau + 120.0 * tau * tau * tau) /
      (duration_ * duration_);

  for (int i = 0; i < q0_.size(); ++i) {
    const double delta = qf_[i] - q0_[i];
    zVecElemNC(q, i) = q0_[i] + delta * s;
    if (dq != nullptr) zVecElemNC(dq, i) = delta * ds;
    if (ddq != nullptr) zVecElemNC(ddq, i) = delta * dds;
  }
}

RoundTripTrajectory::RoundTripTrajectory(MinJerkTrajectory out,
                                         MinJerkTrajectory back)
    : out_(std::move(out)), back_(std::move(back)) {
  if (out_.size() != back_.size()) {
    throw std::invalid_argument(
        "RoundTripTrajectory: segment sizes must match");
  }
  ZVector out_end(out_.size()), back_start(back_.size());
  out_.sample(out_.duration(), out_end.get());
  back_.sample(0.0, back_start.get());
  for (int i = 0; i < out_.size(); ++i) {
    if (std::fabs(out_end[i] - back_start[i]) > 1e-9) {
      throw std::invalid_argument(
          "RoundTripTrajectory: segments must meet at the split");
    }
  }
}

void RoundTripTrajectory::sample(double t, zVec q, zVec dq, zVec ddq) const {
  if (t < out_.duration()) {
    out_.sample(t, q, dq, ddq);
  } else {
    back_.sample(t - out_.duration(), q, dq, ddq);
  }
}

SinusoidTrajectory::SinusoidTrajectory(const zVec center, const zVec amplitude,
                                       double period)
    : center_(zVecSizeNC(center)),
      amplitude_(zVecSizeNC(amplitude)),
      period_(period) {
  requireMatch(center, amplitude, "SinusoidTrajectory");
  if (period <= 0.0) {
    throw std::invalid_argument("SinusoidTrajectory: period must be > 0");
  }
  zVecCopyNC(center, center_.get());
  zVecCopyNC(amplitude, amplitude_.get());
}

void SinusoidTrajectory::sample(double t, zVec q, zVec dq) const {
  requireMatch(q, center_.get(), "sample");
  if (dq != nullptr) requireMatch(dq, center_.get(), "sample dq");

  const double omega = 2.0 * M_PI / period_;
  for (int i = 0; i < center_.size(); ++i) {
    zVecElemNC(q, i) = center_[i] + amplitude_[i] * std::sin(omega * t);
    if (dq != nullptr) {
      zVecElemNC(dq, i) = amplitude_[i] * omega * std::cos(omega * t);
    }
  }
}

}  // namespace rtctrl::model
