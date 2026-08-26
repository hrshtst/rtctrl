#pragma once

#include <zeo/zeo_ep.h>

#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/model/ik_solver.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::model {

enum class PtpProfile { Linear, Trapezoidal, MinimumJerk };

struct PtpProgress {
  double position = 0.0;
  double velocity = 0.0;  // derivative with respect to normalized time
  double acceleration = 0.0;  // second derivative wrt normalized time
  // True where the ideal profile has no unique two-sided derivative. The
  // reported velocity/acceleration is the profile's in-motion convention.
  bool derivative_discontinuity = false;
};

// Scalar path progress on normalized time u. Values outside [0, 1] clamp to
// the endpoints. acceleration_fraction is used only by Trapezoidal and is the
// fraction of the motion spent in each acceleration/deceleration ramp.
PtpProgress ptpProgress(PtpProfile profile, double u,
                        double acceleration_fraction = 0.2);
double ptpPeakSpeedFactor(PtpProfile profile,
                          double acceleration_fraction = 0.2);

struct CartesianPose {
  zVec3D position{};
  zMat3D attitude{};
};

CartesianPose interpolateCartesianPose(const CartesianPose& start,
                                       const CartesianPose& end,
                                       double progress);
double cartesianTranslationDistance(const CartesianPose& start,
                                    const CartesianPose& end);
double cartesianRotationDistance(const CartesianPose& start,
                                 const CartesianPose& end);

struct PtpTiming {
  std::optional<double> motion_time;
  std::optional<double> max_linear_velocity;
  std::optional<double> max_angular_velocity;
  double fallback_motion_time = 5.0;
};

// Returns the longest of the explicit motion time and durations required by
// the paired Cartesian speed limits. When no constraint is present, uses the
// fallback motion time. A zero-distance, velocity-only request returns zero.
double choosePtpDuration(const CartesianPose& start, const CartesianPose& end,
                         PtpProfile profile, const PtpTiming& timing,
                         double acceleration_fraction = 0.2);

struct PtpPlanOptions {
  PtpProfile profile = PtpProfile::MinimumJerk;
  PtpTiming timing;
  double sample_rate = 100.0;
  double trapezoid_acceleration_fraction = 0.2;
  bool strict_ik = true;
  double position_tolerance = 1e-4;
  double attitude_tolerance = 1e-3;
  int max_iterations = 200;
};

struct PtpSample {
  double time = 0.0;
  PtpProgress progress;
  std::vector<double> displacement;  // model coordinates
  CartesianPose target;
  CartesianPose achieved;
  zVec3D target_linear_velocity{};      // world frame [m/s]
  zVec3D target_linear_acceleration{};  // world frame [m/s^2]
  zVec3D target_angular_velocity{};     // world frame [rad/s]
  zVec3D target_angular_acceleration{};  // world frame [rad/s^2]
  zVec3D achieved_linear_velocity{};      // world frame [m/s]
  zVec3D achieved_linear_acceleration{};  // world frame [m/s^2]
  zVec3D achieved_angular_velocity{};     // world frame [rad/s]
  zVec3D achieved_angular_acceleration{};  // world frame [rad/s^2]
  zVec3D position_error{};  // target - achieved, world frame [m]
  zVec3D attitude_error{};  // achieved-to-target rotation, world frame [rad]
  std::array<double, kCanonicalDof> joint_position{};
  // Second-order finite-difference estimates of the sampled joint path.
  std::array<double, kCanonicalDof> joint_velocity{};
  std::array<double, kCanonicalDof> joint_acceleration{};
  std::array<double, kCanonicalDof> joint_limit_margin{};
  IkResult ik;
};

struct PtpIkWarning {
  std::size_t sample = 0;
  double time = 0.0;
  IkResult result;
};

struct PtpPlan {
  double duration = 0.0;
  double interval = 0.0;
  std::vector<PtpSample> samples;
  std::vector<PtpIkWarning> ik_warnings;
  double worst_position_residual = 0.0;
  double worst_attitude_residual = 0.0;
  double peak_joint_velocity = 0.0;
  double peak_joint_acceleration = 0.0;
  double minimum_joint_limit_margin = 0.0;
};

class PtpPlanningError : public std::runtime_error {
 public:
  PtpPlanningError(const std::string& message, std::size_t sample,
                   double time, IkResult result);

  std::size_t sample() const { return sample_; }
  double time() const { return time_; }
  const IkResult& result() const { return result_; }

 private:
  std::size_t sample_;
  double time_;
  IkResult result_;
};

class CartesianPtpPlanner {
 public:
  CartesianPtpPlanner(ChainModel& model, const JointMap& map,
                      const std::string& effector_link =
                          "crane_x7_tcp_link");

  // initial_displacement is a model-space seed. Each subsequent solve uses
  // the preceding solution so the IK branch is continuous where possible.
  PtpPlan plan(const CartesianPose& start, const CartesianPose& end,
               const zVec initial_displacement,
               const PtpPlanOptions& options = {});

 private:
  ChainModel& model_;
  const JointMap& map_;
  int effector_index_;
  IkSolver solver_;
};

}  // namespace rtctrl::model
