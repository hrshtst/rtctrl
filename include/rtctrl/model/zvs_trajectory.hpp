#pragma once

#include <string>
#include <vector>

#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"

namespace rtctrl::model {

enum class ReferenceInterpolation { Linear, ShapePreservingCubic };
enum class ReferenceFilter { None, LowPass, MovingAverage, SavitzkyGolay };

struct ZvsTrajectoryOptions {
  ReferenceInterpolation interpolation =
      ReferenceInterpolation::ShapePreservingCubic;
  ReferenceFilter filter = ReferenceFilter::None;
  double low_pass_cutoff_hz = 5.0;
  int filter_window = 5;
  int savitzky_golay_order = 3;
  double mimic_tolerance = 1e-9;
};

// A model- or canonical-width ZVS reference sampled in canonical coordinates.
// A cell's dt is the duration for which that frame is displayed. Therefore the
// first frame starts at t=0 and the final frame is held for its own dt.
class ZvsTrajectory : public Trajectory {
 public:
  ZvsTrajectory(const std::string& path, const JointMap& map,
                ZvsTrajectoryOptions options = {});

  double duration() const override { return duration_; }
  int size() const override { return kCanonicalDof; }
  void sample(double t, zVec q, zVec dq = nullptr,
              zVec ddq = nullptr) const override;

  std::size_t frames() const { return positions_.size(); }
  const std::vector<double>& frameTimes() const { return times_; }
  const std::vector<double>& frameDurations() const { return durations_; }
  const std::vector<double>& frame(std::size_t index) const {
    return positions_.at(index);
  }

 private:
  void filterPositions(const ZvsTrajectoryOptions& options);
  void computeSlopes();

  ReferenceInterpolation interpolation_;
  std::vector<double> times_;
  std::vector<double> durations_;
  std::vector<std::vector<double>> positions_;
  std::vector<std::vector<double>> slopes_;
  double duration_ = 0.0;
};

}  // namespace rtctrl::model
