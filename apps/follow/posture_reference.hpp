// Follow references built from the arm's measured posture.
//
// A hold (--hold-current SECONDS) keeps the posture read before
// activation; a sweep (--sweep-current JOINT:AMPLITUDE:DURATION) moves one
// canonical arm joint out by AMPLITUDE and back along two minimum-jerk
// halves. Both start exactly at the measured posture, so following them
// needs no homing: the run sets HomeConfig::max_distance_rad to
// kPostureMatchToleranceRad and refuses, before any motion command, if the
// arm has moved away from that posture in the meantime. The generated
// reference is written as a .zvs next to the run log, so it is archived
// with the run like any other reference.
#pragma once

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "follow/follow_config.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace x7::follow {

// About three encoder ticks (1.53 mrad each).
inline constexpr double kPostureMatchToleranceRad = 0.005;
inline constexpr double kPostureFrameS = 0.01;

struct PostureReference {
  enum class Kind { kHold, kSweep };
  Kind kind = Kind::kHold;
  double duration_s = 0.0;
  int joint = -1;  // sweep only: canonical arm joint 0..6
  double amplitude_rad = 0.0;
};

inline double parsePostureNumber(const std::string& what,
                                 const std::string& text) {
  std::size_t used = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &used);
  } catch (const std::exception&) {
    used = 0;
  }
  if (used == 0 || used != text.size() || !std::isfinite(value)) {
    throw std::runtime_error(what + ": not a number: '" + text + "'");
  }
  return value;
}

inline PostureReference parseHoldCurrent(const std::string& seconds) {
  PostureReference r;
  r.kind = PostureReference::Kind::kHold;
  r.duration_s = parsePostureNumber("--hold-current", seconds);
  if (!(r.duration_s > 0.0 && r.duration_s <= 600.0)) {
    throw std::runtime_error("--hold-current must be within (0, 600] s");
  }
  return r;
}

inline PostureReference parseSweepCurrent(const std::string& spec) {
  const auto first = spec.find(':');
  const auto second =
      first == std::string::npos ? first : spec.find(':', first + 1);
  if (second == std::string::npos ||
      spec.find(':', second + 1) != std::string::npos) {
    throw std::runtime_error(
        "--sweep-current expects JOINT:AMPLITUDE_RAD:DURATION_S");
  }
  PostureReference r;
  r.kind = PostureReference::Kind::kSweep;
  const double joint =
      parsePostureNumber("--sweep-current joint", spec.substr(0, first));
  r.amplitude_rad = parsePostureNumber(
      "--sweep-current amplitude", spec.substr(first + 1, second - first - 1));
  r.duration_s =
      parsePostureNumber("--sweep-current duration", spec.substr(second + 1));
  if (joint != std::floor(joint) || joint < 0.0 ||
      joint > model::kCanonicalDof - 2) {
    throw std::runtime_error(
        "--sweep-current joint must be a canonical arm joint 0..6");
  }
  r.joint = static_cast<int>(joint);
  if (!(r.amplitude_rad != 0.0 && std::fabs(r.amplitude_rad) <= 1.0)) {
    throw std::runtime_error(
        "--sweep-current amplitude must be nonzero and within 1 rad");
  }
  if (!(r.duration_s >= 1.0 && r.duration_s <= 120.0)) {
    throw std::runtime_error("--sweep-current duration must be within "
                             "[1, 120] s");
  }
  return r;
}

using CanonicalPosture = std::array<double, model::kCanonicalDof>;

// Frames at kPostureFrameS; the first and the last frame equal q0
// exactly. A sweep's peak joint speed is 15/8 * |amplitude| / (duration/2).
inline std::vector<CanonicalPosture> postureReferenceFrames(
    const PostureReference& r, const CanonicalPosture& q0) {
  const long intervals =
      std::max(1L, std::lround(r.duration_s / kPostureFrameS));
  std::vector<CanonicalPosture> frames;
  frames.reserve(static_cast<std::size_t>(intervals) + 1);
  for (long k = 0; k <= intervals; ++k) {
    CanonicalPosture q = q0;
    if (r.kind == PostureReference::Kind::kSweep) {
      // Out over the first half, back over the second.
      const double u = static_cast<double>(k) / intervals;
      const double half = u <= 0.5 ? 2.0 * u : 2.0 * (1.0 - u);
      const double s =
          half * half * half * (10.0 - 15.0 * half + 6.0 * half * half);
      q[r.joint] = q0[r.joint] + r.amplitude_rad * s;
    }
    frames.push_back(q);
  }
  return frames;
}

// Writes canonical (8-value) frames, each kPostureFrameS long. Refuses an
// existing path.
inline void writePostureReferenceZvs(
    const std::filesystem::path& path,
    const std::vector<CanonicalPosture>& frames) {
  std::FILE* f = std::fopen(path.c_str(), "wx");
  if (f == nullptr) {
    throw std::runtime_error("cannot create posture reference '" +
                             path.string() + "' (it must not exist)");
  }
  bool ok = true;
  for (const auto& q : frames) {
    ok = ok && std::fprintf(f, "%.17g %d (", kPostureFrameS,
                            model::kCanonicalDof) > 0;
    for (const double v : q) ok = ok && std::fprintf(f, " %.17g", v) > 0;
    ok = ok && std::fprintf(f, " )\n") > 0;
  }
  ok = std::fflush(f) == 0 && ok && std::ferror(f) == 0;
  if (std::fclose(f) != 0 || !ok) {
    throw std::runtime_error("cannot finish posture reference '" +
                             path.string() + "'");
  }
}

inline std::string describePostureReference(const PostureReference& r) {
  char text[160];
  if (r.kind == PostureReference::Kind::kHold) {
    std::snprintf(text, sizeof(text),
                  "hold the measured posture for %.3g s", r.duration_s);
  } else {
    std::snprintf(text, sizeof(text),
                  "sweep joint %d by %+.3g rad and back over %.3g s from "
                  "the measured posture",
                  r.joint, r.amplitude_rad, r.duration_s);
  }
  return text;
}

}  // namespace x7::follow
