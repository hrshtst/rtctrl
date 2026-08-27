#pragma once

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/hw/command_current.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7::gravity {

namespace dxl = rtctrl::dxl;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;

inline constexpr double kVendorScaleXm430 = 0.810455;
inline constexpr double kVendorScaleXm540 = 0.669167;
inline constexpr double kVendorScaleTolerance = 1e-6;
inline constexpr double kStartLimitBufferRad = 0.05;

struct CalibrationMismatch {
  std::size_t joint = 0;
  double expected = 0.0;
  double actual = 0.0;
};

inline std::optional<CalibrationMismatch> calibrationMismatch(
    const hw::Config& config) {
  for (std::size_t i = 0; i < config.joints.size(); ++i) {
    const auto& joint = config.joints[i];
    const double expected =
        joint.model_number == dxl::kModelXm540W270 ? kVendorScaleXm540
                                                   : kVendorScaleXm430;
    if (std::fabs(joint.command_torque_scale - expected) >
        kVendorScaleTolerance) {
      return CalibrationMismatch{i, expected, joint.command_torque_scale};
    }
  }
  return std::nullopt;
}

struct LimitViolation {
  std::size_t joint = 0;
  double position = 0.0;
  double lower = 0.0;
  double upper = 0.0;
};

inline std::optional<LimitViolation> startLimitViolation(
    const std::vector<dxl::Feedback>& feedback,
    const std::vector<double>& lower, const std::vector<double>& upper,
    double buffer_rad = kStartLimitBufferRad) {
  if (feedback.size() != lower.size() || feedback.size() != upper.size() ||
      !std::isfinite(buffer_rad) || buffer_rad < 0.0) {
    return LimitViolation{};
  }
  for (std::size_t i = 0; i < feedback.size(); ++i) {
    if (feedback[i].position < lower[i] + buffer_rad ||
        feedback[i].position > upper[i] - buffer_rad) {
      return LimitViolation{i, feedback[i].position, lower[i], upper[i]};
    }
  }
  return std::nullopt;
}

inline std::vector<double> gravityPreload(
    const hw::Config& config, model::ChainModel& chain,
    const model::JointMap& map, const std::vector<dxl::Feedback>& feedback) {
  if (feedback.size() != config.joints.size() ||
      feedback.size() != model::kCanonicalDof) {
    return {};
  }
  model::ZVector q(model::kCanonicalDof);
  model::ZVector tau(model::kCanonicalDof);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    q[i] = feedback[i].position;
  }
  chain.gravityTorque(map, q.get(), tau.get());
  std::vector<double> preload(model::kCanonicalDof);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    preload[i] = hw::commandCurrentFromTorque(config.joints[i], tau[i]);
  }
  return preload;
}

}  // namespace x7::gravity
