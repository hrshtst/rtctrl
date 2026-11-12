#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>

#include "follow/follow_config.hpp"
#include "plan/ptp_bundle.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/learn/command_offset.hpp"
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

// The grid FollowRun::trackReference tracks a reference of this duration
// on: learn::trackingGrid at the control period 1 / control.rate_hz, which
// is the arm's dt() in every frontend. The follower builds the same grid
// only after homing, with torque on; a duration it cannot be built for
// (non-finite, not > 0, or too many intervals for a long) is refused here
// instead, naming the rate. Throws std::runtime_error. Pure arithmetic:
// call it before any bus contact.
inline rtctrl::learn::TrackingGrid preflightTrackingGrid(double duration_s,
                                                         const Config& config) {
  try {
    return rtctrl::learn::trackingGrid(duration_s,
                                       1.0 / config.control_rate_hz);
  } catch (const std::exception& error) {
    char rate[32];
    std::snprintf(rate, sizeof rate, "%.9g", config.control_rate_hz);
    throw std::runtime_error(
        std::string("follow preflight: the reference cannot be tracked at "
                    "control.rate_hz = ") +
        rate + ": " + error.what());
  }
}

// The time of tracking-grid sample i, as FollowRun::trackReference
// computes it: min(duration, i * dt).
inline double trackingSampleTime(const rtctrl::learn::TrackingGrid& grid,
                                 double duration_s, std::size_t sample) {
  return std::min(duration_s, static_cast<double>(sample) * grid.dt_s);
}

// Every sample the follower will command (preflightTrackingGrid, sample
// for sample) inside the model joint limits, finite, and within the
// deployment velocity limits; current-based position effort within the
// deployment limits. Throws std::runtime_error naming the problem. Reads
// nothing: call it before any bus contact.
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
  const auto grid = preflightTrackingGrid(trajectory.duration(), config);
  for (std::size_t sample = 0; sample < grid.samples(); ++sample) {
    const double time =
        trackingSampleTime(grid, trajectory.duration(), sample);
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

// A learned command offset that preflightOffset accepted for one run.
struct AcceptedOffset {
  rtctrl::learn::CommandOffset offset;
  std::string sha256;        // of the offset file's bytes
  double max_abs_rad = 0.0;  // largest |delta| over every sample and joint
};

// A reference built from the measured posture (--hold-current,
// --sweep-current) takes no learned offset: an offset belongs to one
// reference file, and this reference is made from the posture at run time.
inline void refusePostureOffset(const Config& config) {
  if (config.learning.offset_path) {
    throw std::runtime_error(
        "follow preflight: a learned offset ([learning] offset or --offset) "
        "cannot be applied to a reference built from the measured posture "
        "(--hold-current, --sweep-current)");
  }
}

// Empty when no offset is configured. Otherwise reads the offset file and
// accepts it only for this run: learn::checkOffsetForRun against the
// loaded reference's duration, the control period 1 / control.rate_hz, the
// reference file's SHA-256 and the [learning] limits; then every command
// it produces (q_d(t_i) + delta[i] at each tracking-grid sample i, and the
// final hold's q_d(T) + delta[N-1]) inside the model joint limits, the
// limit source of validateReference. Throws std::runtime_error naming the
// problem. Reads files only: call it before any bus contact.
inline std::optional<AcceptedOffset> preflightOffset(
    const Config& config, const model::ChainModel& chain,
    const model::JointMap& map, const model::ZvsTrajectory& reference) {
  if (!config.learning.offset_path) return std::nullopt;
  if (config.posture) refusePostureOffset(config);
  const auto& path = *config.learning.offset_path;
  const auto hash = [&path] {
    try {
      return x7::ptp::sha256File(path);
    } catch (const std::exception& error) {
      throw std::runtime_error(
          "follow preflight: cannot read the offset file '" + path.string() +
          "' (" + error.what() + ")");
    }
  };
  AcceptedOffset accepted;
  // Hashed before and after the read, so the recorded hash is of the
  // bytes that were parsed.
  accepted.sha256 = hash();
  accepted.offset = rtctrl::learn::readCommandOffset(path);
  if (hash() != accepted.sha256) {
    throw std::runtime_error("follow preflight: the offset file '" +
                             path.string() + "' changed while it was read");
  }
  const double control_dt_s = 1.0 / config.control_rate_hz;
  rtctrl::learn::checkOffsetForRun(
      accepted.offset, reference.duration(), control_dt_s,
      x7::ptp::sha256File(config.reference_path),
      {config.learning.max_offset_rad, config.learning.max_offset_rate_rad_s});

  const auto& delta = accepted.offset.delta_rad;
  model::ZVector q(model::kCanonicalDof);
  const auto check = [&](std::size_t sample, double time, const char* what) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double lo = chain.jointMin(map.linkId(i));
      const double hi = chain.jointMax(map.linkId(i));
      const double command = q[i] + delta[sample][i];
      if (!std::isfinite(command) || command < lo || command > hi) {
        char text[320];
        std::snprintf(text, sizeof text,
                      "follow preflight: reference plus offset leaves the "
                      "joint %d limits [%.9g, %.9g] rad at %s %zu (t = %.9g "
                      "s): q_d %.9g + delta %.9g = %.9g rad",
                      i, lo, hi, what, sample, time, q[i], delta[sample][i],
                      command);
        throw std::runtime_error(text);
      }
      accepted.max_abs_rad =
          std::max(accepted.max_abs_rad, std::fabs(delta[sample][i]));
    }
  };
  // The grid FollowRun::trackReference commands, sample for sample.
  const auto grid = preflightTrackingGrid(reference.duration(), config);
  for (std::size_t sample = 0; sample < grid.samples(); ++sample) {
    const double time = trackingSampleTime(grid, reference.duration(), sample);
    reference.sample(time, q.get());
    check(sample, time, "tracking sample");
  }
  reference.sample(reference.duration(), q.get());
  check(delta.size() - 1, reference.duration(), "the final hold, sample");
  return accepted;
}

// The line a frontend prints once it accepted an offset.
inline std::string describeOffset(const AcceptedOffset& accepted) {
  char text[256];
  std::snprintf(text, sizeof text,
                "command offset accepted: method %s, trial %d, max |delta| "
                "%.6g rad, sha256 %s",
                accepted.offset.method.c_str(), accepted.offset.trial,
                accepted.max_abs_rad, accepted.sha256.c_str());
  return text;
}

}  // namespace x7::follow
