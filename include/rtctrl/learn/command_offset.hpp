// Learned command offset for the servo-side follower (x7_follow): the
// position offset delta(t) added to the tracking command, q_d + delta (see
// docs/theory/learning-control.md, "Command offset as torque"). The offset
// is computed offline between trials and handed to the follower as a file;
// this module holds its grid, validation, limits and file format. Pure
// logic: no bus, no model. The SHA-256 lineage strings are computed by the
// app layer; this module only stores and checks their form.
#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::learn {

using JointArray = std::array<double, model::kCanonicalDof>;  // canonical 8-DOF order

inline constexpr const char* kCommandOffsetFormat = "rtctrl-x7-follow-offset";
inline constexpr int kCommandOffsetVersion = 1;

// (samples - 1) * dt_s against reference_duration_s, in the structural check.
inline constexpr double kOffsetSpanToleranceS = 1e-9;
// The offset's dt_s against the tracking grid step, in the run check.
inline constexpr double kOffsetGridStepToleranceS = 1e-12;
// A file's time_s column against i * dt_s, when reading.
inline constexpr double kOffsetTimeToleranceS = 1e-6;

struct CommandOffset {
  std::string method;               // "zero", "ilc" or "fel"
  int trial = 0;                    // number of learning updates applied
  double dt_s = 0.0;                // grid step [s]
  double reference_duration_s = 0;  // duration of the reference this belongs to [s]
  std::string reference_sha256;     // of the reference file's bytes: 64 lowercase hex
  std::string parent_sha256;        // of the offset file this was updated from; empty for trial 0
  std::string source_log_sha256;    // of the trial log the update used; empty for trial 0
  std::vector<JointArray> delta_rad;  // sample i is at t = i * dt_s
};

struct OffsetLimits {
  double max_abs_rad = 0.15;     // matches the follower's home-correction clamp
  double max_rate_rad_s = 0.5;   // |delta[i+1] - delta[i]| / dt
};

// The grid the follower tracks a reference on: samples 0..intervals, sample
// i commanded at t = i * dt_s.
struct TrackingGrid {
  long intervals = 0;
  double dt_s = 0.0;
  std::size_t samples() const {
    return static_cast<std::size_t>(intervals) + 1;
  }
};

// The tracking grid of a reference of the given duration at the given
// control period, computed with the same floating-point expression as
// FollowRun::trackReference (apps/follow/follow_run.hpp), so the two never
// disagree. Throws std::runtime_error unless both arguments are finite and
// > 0 and the interval count fits a long.
TrackingGrid trackingGrid(double reference_duration_s, double control_dt_s);

// The offset of trial 0: method "zero", all zeros on the tracking grid,
// empty parent and source hashes. Throws std::runtime_error on an invalid
// grid or reference hash.
CommandOffset zeroOffset(double reference_duration_s, double control_dt_s,
                         const std::string& reference_sha256);

// Structural check: method one of zero/ilc/fel; trial >= 0; dt_s finite
// and > 0; reference_duration_s finite and > 0; at least 2 samples;
// (samples - 1) * dt_s equal to reference_duration_s within
// kOffsetSpanToleranceS; every value finite; reference_sha256 64 lowercase
// hex, parent_sha256 and source_log_sha256 each empty or 64 lowercase hex;
// trial 0 if and only if method "zero", and then every value exactly zero
// and no parent or source; trial >= 1 with both parent and source. Throws
// std::runtime_error naming the first problem.
void checkOffsetStructure(const CommandOffset& offset);

// The offset may be applied to this run: limits finite and > 0, the
// structural check, the tracking grid of (reference_duration_s,
// control_dt_s) (same sample count, step within kOffsetGridStepToleranceS),
// the reference hash, |delta| <= max_abs_rad and
// |delta[i+1] - delta[i]| / dt_s <= max_rate_rad_s on every joint. Throws
// std::runtime_error naming the joint, sample index, time and value of a
// limit violation.
void checkOffsetForRun(const CommandOffset& offset, double reference_duration_s,
                       double control_dt_s, const std::string& reference_sha256,
                       const OffsetLimits& limits);

// The file's exact bytes: the "# key: value" preamble (an empty value is
// written as "# key:"), the column header, one row per sample with time_s
// as %.9f of i * dt_s and every other number as %.17g, so a write -> read
// round trip is bit exact and a rewrite byte identical. Runs the
// structural check first.
std::string formatCommandOffset(const CommandOffset& offset);

// Structural check, then creates path exclusively (an existing file or
// symlink is refused and left untouched) and writes formatCommandOffset;
// on any failure after creation the partial file is removed before the
// error propagates. Throws std::runtime_error.
void writeCommandOffset(const std::filesystem::path& path,
                        const CommandOffset& offset);

// Strict reader of the format above: the preamble keys exactly in order,
// each once; format and version; the column header; nine fields per row,
// every number finite; as many rows as samples; time_s within
// kOffsetTimeToleranceS of i * dt_s; a final newline. Then the structural
// check. Throws std::runtime_error naming source and line.
CommandOffset parseCommandOffset(std::istream& input,
                                 const std::string& source = "command offset");
CommandOffset readCommandOffset(const std::filesystem::path& path);

}  // namespace rtctrl::learn
