#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "rtctrl/dxl/sync_group.hpp"

namespace x7 {

struct MoveEndpoint {
  double target = 0.0;
  double displacement = 0.0;
  bool clamped = false;
};

inline MoveEndpoint clampMoveEndpoint(double start, double displacement,
                                      double lo, double hi) {
  MoveEndpoint result;
  const double requested = start + displacement;
  result.target = std::clamp(requested, lo, hi);
  result.displacement = result.target - start;
  result.clamped = result.target != requested;
  return result;
}

struct ReturnCheck {
  bool valid = false;
  bool within_tolerance = false;
  int worst_joint = -1;
  double worst_deviation = 0.0;
};

inline ReturnCheck checkReturnPosture(
    const std::vector<rtctrl::dxl::Feedback>& feedback,
    const std::vector<double>& start, double tolerance) {
  ReturnCheck result;
  if (feedback.size() != start.size() || feedback.empty()) return result;
  result.valid = true;
  for (std::size_t i = 0; i < start.size(); ++i) {
    const double deviation = std::fabs(feedback[i].position - start[i]);
    if (deviation > result.worst_deviation) {
      result.worst_deviation = deviation;
      result.worst_joint = static_cast<int>(i);
    }
  }
  result.within_tolerance = result.worst_deviation <= tolerance;
  return result;
}

}  // namespace x7
