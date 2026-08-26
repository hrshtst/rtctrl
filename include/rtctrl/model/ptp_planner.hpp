#pragma once

#include <zeo/zeo_ep.h>

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
  std::vector<double> displacement;  // model coordinates
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
  IkSolver solver_;
};

}  // namespace rtctrl::model
