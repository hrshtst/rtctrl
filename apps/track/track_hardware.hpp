#pragma once

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <vector>

#include "rtctrl/hw/command_current.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "track/track_config.hpp"

namespace x7::track {

namespace arm = rtctrl::arm;
namespace hw = rtctrl::hw;
namespace model = rtctrl::model;

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

inline hw::CraneX7::Options hardwareOptions(const Config& config) {
  hw::CraneX7::Options options;
  options.control_cycle_s = 1.0 / config.control_rate_hz;
  options.controller_deadline_s = options.control_cycle_s;
  options.controller_write_margin_s =
      std::min(0.002, 0.2 * options.control_cycle_s);
  return options;
}

inline void prepareHardwareConfig(const Config& config, hw::Config* hardware,
                                  const std::optional<std::string>& port) {
  if (port) hardware->port = *port;
  if (hardware->joints.size() != model::kCanonicalDof) {
    throw std::runtime_error("track preflight: hardware joint count mismatch");
  }
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    hardware->joints[i].operating_mode =
        static_cast<std::uint8_t>(arm::ControlMode::Position);
    if (config.effort_limit_set) {
      if (config.effort_limit_nm[i] > hardware->joints[i].effort_limit) {
        throw std::runtime_error(
            "track preflight: effort ceiling exceeds deployment limit on joint " +
            std::to_string(i));
      }
      hardware->joints[i].effort_limit = config.effort_limit_nm[i];
    }
  }
  hardware->validate();
}

inline void validateReference(const Config& config,
                              const model::ChainModel& chain,
                              const model::JointMap& map,
                              const model::Trajectory& trajectory,
                              const hw::Config& hardware) {
  const long intervals = std::max(
      1L, static_cast<long>(
              std::ceil(trajectory.duration() * config.control_rate_hz)));
  model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
      ddq(model::kCanonicalDof);
  for (long sample = 0; sample <= intervals; ++sample) {
    const double t = trajectory.duration() * sample / intervals;
    trajectory.sample(t, q.get(), dq.get(), ddq.get());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double lo = chain.jointMin(map.linkId(i));
      const double hi = chain.jointMax(map.linkId(i));
      if (!std::isfinite(q[i]) || !std::isfinite(dq[i]) ||
          !std::isfinite(ddq[i]) || q[i] < lo || q[i] > hi) {
        throw std::runtime_error(
            "track preflight: reference violates joint " +
            std::to_string(i) + " displacement/finite limits at t=" +
            std::to_string(t));
      }
      if (std::fabs(dq[i]) > hardware.joints[i].velocity_limit) {
        throw std::runtime_error(
            "track preflight: reference velocity exceeds deployment limit on joint " +
            std::to_string(i) + " at t=" + std::to_string(t));
      }
    }
  }
}

inline std::vector<double> gravityPreload(const hw::Config& config,
                                          model::ChainModel& chain,
                                          const model::JointMap& map,
                                          const std::vector<rtctrl::dxl::Feedback>& feedback) {
  if (feedback.size() != model::kCanonicalDof) {
    throw std::runtime_error("track: held-posture feedback size mismatch");
  }
  model::ZVector q(model::kCanonicalDof), tau(model::kCanonicalDof);
  for (int i = 0; i < model::kCanonicalDof; ++i) q[i] = feedback[i].position;
  chain.gravityTorque(map, q.get(), tau.get());
  std::vector<double> amps(model::kCanonicalDof);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    amps[i] = hw::commandCurrentFromTorque(config.joints[i], tau[i]);
  }
  return amps;
}

}  // namespace x7::track
