#include "rtctrl/model/ptp_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "rtctrl/model/zvector.hpp"

namespace rtctrl::model {

namespace {

void requireFinitePositive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and > 0");
  }
}

void validateAccelerationFraction(double fraction) {
  if (!std::isfinite(fraction) || fraction <= 0.0 || fraction > 0.5) {
    throw std::invalid_argument(
        "trapezoid acceleration fraction must be in (0, 0.5]");
  }
}

void validatePose(const CartesianPose& pose, const char* name) {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(pose.position.e[i])) {
      throw std::invalid_argument(std::string(name) +
                                  " position must be finite");
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (!std::isfinite(pose.attitude.e[row][col])) {
        throw std::invalid_argument(std::string(name) +
                                    " attitude must be finite");
      }
    }
  }
}

}  // namespace

PtpProgress ptpProgress(PtpProfile profile, double u,
                        double acceleration_fraction) {
  if (!std::isfinite(u)) {
    throw std::invalid_argument("normalized time must be finite");
  }
  u = std::clamp(u, 0.0, 1.0);
  switch (profile) {
    case PtpProfile::Linear:
      return {u, 1.0};
    case PtpProfile::MinimumJerk: {
      const double position =
          u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
      const double velocity = 30.0 * u * u * (1.0 - u) * (1.0 - u);
      return {position, velocity};
    }
    case PtpProfile::Trapezoidal: {
      validateAccelerationFraction(acceleration_fraction);
      const double denominator = acceleration_fraction *
                                 (1.0 - acceleration_fraction);
      if (u < acceleration_fraction) {
        return {0.5 * u * u / denominator, u / denominator};
      }
      if (u <= 1.0 - acceleration_fraction) {
        return {(u - 0.5 * acceleration_fraction) /
                    (1.0 - acceleration_fraction),
                1.0 / (1.0 - acceleration_fraction)};
      }
      const double remaining = 1.0 - u;
      return {1.0 - 0.5 * remaining * remaining / denominator,
              remaining / denominator};
    }
  }
  throw std::invalid_argument("unknown PTP profile");
}

double ptpPeakSpeedFactor(PtpProfile profile,
                          double acceleration_fraction) {
  switch (profile) {
    case PtpProfile::Linear:
      return 1.0;
    case PtpProfile::Trapezoidal:
      validateAccelerationFraction(acceleration_fraction);
      return 1.0 / (1.0 - acceleration_fraction);
    case PtpProfile::MinimumJerk:
      return 15.0 / 8.0;
  }
  throw std::invalid_argument("unknown PTP profile");
}

CartesianPose interpolateCartesianPose(const CartesianPose& start,
                                       const CartesianPose& end,
                                       double progress) {
  if (!std::isfinite(progress)) {
    throw std::invalid_argument("Cartesian progress must be finite");
  }
  progress = std::clamp(progress, 0.0, 1.0);
  CartesianPose pose;
  zVec3DInterDiv(&start.position, &end.position, progress, &pose.position);
  zMat3DInterDiv(&start.attitude, &end.attitude, progress, &pose.attitude);
  return pose;
}

double cartesianTranslationDistance(const CartesianPose& start,
                                    const CartesianPose& end) {
  return zVec3DDist(&start.position, &end.position);
}

double cartesianRotationDistance(const CartesianPose& start,
                                 const CartesianPose& end) {
  zVec3D error;
  zMat3DError(&end.attitude, &start.attitude, &error);
  return zVec3DNorm(&error);
}

double choosePtpDuration(const CartesianPose& start, const CartesianPose& end,
                         PtpProfile profile, const PtpTiming& timing,
                         double acceleration_fraction) {
  validatePose(start, "start pose");
  validatePose(end, "end pose");
  requireFinitePositive(timing.fallback_motion_time, "fallback motion time");
  if (timing.motion_time) {
    requireFinitePositive(*timing.motion_time, "motion time");
  }
  if (timing.max_linear_velocity.has_value() !=
      timing.max_angular_velocity.has_value()) {
    throw std::invalid_argument(
        "maximum linear and angular velocities must be specified together");
  }

  double duration = timing.motion_time.value_or(0.0);
  if (timing.max_linear_velocity) {
    requireFinitePositive(*timing.max_linear_velocity,
                          "maximum linear velocity");
    requireFinitePositive(*timing.max_angular_velocity,
                          "maximum angular velocity");
    const double peak = ptpPeakSpeedFactor(profile, acceleration_fraction);
    duration = std::max(
        {duration,
         peak * cartesianTranslationDistance(start, end) /
             *timing.max_linear_velocity,
         peak * cartesianRotationDistance(start, end) /
             *timing.max_angular_velocity});
  }
  if (!timing.motion_time && !timing.max_linear_velocity) {
    duration = timing.fallback_motion_time;
  }
  return duration;
}

PtpPlanningError::PtpPlanningError(const std::string& message,
                                   std::size_t sample, double time,
                                   IkResult result)
    : std::runtime_error(message),
      sample_(sample),
      time_(time),
      result_(result) {}

CartesianPtpPlanner::CartesianPtpPlanner(ChainModel& model,
                                         const JointMap& map,
                                         const std::string& effector_link)
    : model_(model), map_(map), solver_(model, map, effector_link) {}

PtpPlan CartesianPtpPlanner::plan(const CartesianPose& start,
                                  const CartesianPose& end,
                                  const zVec initial_displacement,
                                  const PtpPlanOptions& options) {
  validatePose(start, "start pose");
  validatePose(end, "end pose");
  if (initial_displacement == nullptr ||
      zVecSizeNC(initial_displacement) != model_.jointSize()) {
    throw std::invalid_argument(
        "initial displacement must match the model joint size");
  }
  requireFinitePositive(options.sample_rate, "sample rate");
  requireFinitePositive(options.position_tolerance, "position tolerance");
  requireFinitePositive(options.attitude_tolerance, "attitude tolerance");
  if (options.max_iterations <= 0) {
    throw std::invalid_argument("maximum IK iterations must be > 0");
  }
  validateAccelerationFraction(options.trapezoid_acceleration_fraction);
  for (int canonical = 0; canonical < kCanonicalDof; ++canonical) {
    const int offset = map_.rokiOffset(canonical);
    const double q = zVecElemNC(initial_displacement, offset);
    if (!std::isfinite(q) || q < model_.jointMin(map_.linkId(canonical)) ||
        q > model_.jointMax(map_.linkId(canonical))) {
      throw std::invalid_argument(
          "initial displacement is non-finite or outside joint limits");
    }
  }

  PtpPlan plan;
  plan.duration = choosePtpDuration(
      start, end, options.profile, options.timing,
      options.trapezoid_acceleration_fraction);

  std::size_t intervals = 0;
  if (plan.duration > 0.0) {
    const double count = std::ceil(plan.duration * options.sample_rate);
    if (!std::isfinite(count) ||
        count > static_cast<double>(std::numeric_limits<int>::max())) {
      throw std::invalid_argument("PTP plan has too many samples");
    }
    intervals = std::max<std::size_t>(1, static_cast<std::size_t>(count));
    plan.interval = plan.duration / static_cast<double>(intervals);
  } else {
    plan.interval = 1.0 / options.sample_rate;
  }
  plan.samples.reserve(intervals + 1);

  ZVector seed(model_.jointSize());
  ZVector solution(model_.jointSize());
  zVecCopyNC(initial_displacement, seed.get());
  zVecElemNC(seed.get(), map_.rokiOffsetFingerB()) =
      zVecElemNC(seed.get(), map_.rokiOffset(kCanonicalDof - 1));
  for (std::size_t i = 0; i <= intervals; ++i) {
    const double time = intervals == 0 ? 0.0 : plan.interval * i;
    const double u = intervals == 0 ? 0.0 :
                     static_cast<double>(i) / static_cast<double>(intervals);
    const auto progress =
        ptpProgress(options.profile, u,
                    options.trapezoid_acceleration_fraction);
    const auto target = interpolateCartesianPose(start, end, progress.position);
    const auto result = solver_.solve(
        target.position, target.attitude, seed.get(), solution.get(),
        options.position_tolerance, options.attitude_tolerance,
        options.max_iterations);

    plan.worst_position_residual =
        std::max(plan.worst_position_residual, result.pos_residual);
    plan.worst_attitude_residual =
        std::max(plan.worst_attitude_residual, result.att_residual);
    if (!result.converged) {
      if (options.strict_ik || !result.finite || !result.within_limits) {
        throw PtpPlanningError("IK failed during Cartesian PTP planning", i,
                               time, result);
      }
      plan.ik_warnings.push_back({i, time, result});
    }

    PtpSample sample;
    sample.time = time;
    sample.displacement.resize(model_.jointSize());
    for (int joint = 0; joint < model_.jointSize(); ++joint) {
      sample.displacement[joint] = solution[joint];
    }
    if (!plan.samples.empty() && plan.interval > 0.0) {
      const auto& previous = plan.samples.back().displacement;
      for (int joint = 0; joint < model_.jointSize(); ++joint) {
        plan.peak_joint_velocity = std::max(
            plan.peak_joint_velocity,
            std::fabs(sample.displacement[joint] - previous[joint]) /
                plan.interval);
      }
    }
    plan.samples.push_back(std::move(sample));
    zVecCopyNC(solution.get(), seed.get());
  }
  return plan;
}

}  // namespace rtctrl::model
