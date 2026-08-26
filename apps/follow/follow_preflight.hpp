#pragma once

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "follow/follow_config.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace x7::follow {

inline void requireNewOutput(const std::filesystem::path& path,
                             const char* label) {
  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  if (!error && status.type() != std::filesystem::file_type::not_found) {
    throw std::runtime_error(std::string(label) + " already exists: " +
                             path.string());
  }
  if (error && error != std::errc::no_such_file_or_directory) {
    throw std::runtime_error("cannot inspect " + path.string() + ": " +
                             error.message());
  }
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
}

inline void validateReference(const Config& config,
                              const model::ChainModel& chain,
                              const model::JointMap& map,
                              const model::ZvsTrajectory& trajectory,
                              const rtctrl::hw::Config& hardware) {
  if (hardware.joints.size() != model::kCanonicalDof) {
    throw std::runtime_error("follow preflight: hardware joint count mismatch");
  }
  if (config.mode == arm::ControlMode::CurrentBasedPosition) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (config.effort_limit_nm[i] > hardware.joints[i].effort_limit) {
        throw std::runtime_error(
            "follow preflight: current-based position effort exceeds "
            "deployment limit on joint " +
            std::to_string(i));
      }
    }
  }
  model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
      ddq(model::kCanonicalDof);
  const long intervals = std::max(
      1L, static_cast<long>(
              std::ceil(trajectory.duration() * config.control_rate_hz)));
  for (long sample = 0; sample <= intervals; ++sample) {
    const double time = trajectory.duration() * sample / intervals;
    trajectory.sample(time, q.get(), dq.get(), ddq.get());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double lo = chain.jointMin(map.linkId(i));
      const double hi = chain.jointMax(map.linkId(i));
      if (!std::isfinite(q[i]) || !std::isfinite(dq[i]) ||
          !std::isfinite(ddq[i]) || q[i] < lo || q[i] > hi) {
        throw std::runtime_error(
            "follow preflight: reference violates joint " +
            std::to_string(i) + " displacement/finite limits at t=" +
            std::to_string(time));
      }
      if (std::fabs(dq[i]) > hardware.joints[i].velocity_limit) {
        throw std::runtime_error(
            "follow preflight: reference velocity exceeds deployment limit "
            "on joint " +
            std::to_string(i) + " at t=" + std::to_string(time));
      }
    }
  }
}

}  // namespace x7::follow
