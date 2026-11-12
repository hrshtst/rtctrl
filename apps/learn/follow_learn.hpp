// The learning step between follow trials (x7_follow_learn): reads a
// trial's follow log, the command offset that trial ran with and the learn
// config, checks that they belong together and to the follow config's
// reference, calls the learning library (rtctrl/learn: ILC or feedback-error
// learning) and writes the next offset (and the next feedforward weights)
// with its lineage hashes. It has no control law of its own and never opens
// the bus. Every doubt is a refusal that names the file and the problem;
// outputs are created exclusively, and a failure after the first output
// removes every output already written.
#pragma once

#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <istream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <toml++/toml.hpp>
#include <type_traits>
#include <utility>
#include <vector>

#include "follow/follow_config.hpp"
#include "learn/model_torque.hpp"  // the strict CSV field helpers
#include "plan/ptp_bundle.hpp"     // sha256File
#include "rtctrl/learn/command_offset.hpp"
#include "rtctrl/learn/ilc_update.hpp"
#include "rtctrl/learn/joint_array.hpp"
#include "rtctrl/learn/regressor_feedforward.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace x7::learn {

namespace rl = rtctrl::learn;

inline constexpr const char* kLearnConfigFormat = "rtctrl-x7-follow-learn";
inline constexpr int kLearnConfigVersion = 1;
inline constexpr const char* kLearnReportFormat =
    "rtctrl-x7-follow-learn-report";
inline constexpr int kLearnReportVersion = 1;

// The log prints positions with %.9g and times with %.9f.
inline constexpr double kLogPositionToleranceRad = 1e-6;
inline constexpr double kLogTimeToleranceS = 1e-6;
// At most this many final-hold rows after the tracking block are used.
inline constexpr std::size_t kMaxTailRows = 200;
// A feedback-error-learning offset against the synthesis of its weights.
inline constexpr double kSynthesisToleranceRad = 1e-9;

inline constexpr int kDof = model::kCanonicalDof;
inline constexpr int kArmJoints = 7;     // canonical joints 0..6
inline constexpr int kGripperJoint = 7;  // never learned

// The learn config (format "rtctrl-x7-follow-learn", version 1). Absent
// keys keep the library defaults; ilc.max_abs_rad is limits.max_abs_rad.
struct LearnConfig {
  rl::OffsetLimits limits;
  rl::IlcOptions ilc;
  rl::FelOptions fel;
  rl::RegressorOptions regressors;
};

// ------------------------------------------------------------- helpers

namespace learn_detail {

// The shortest text that reads back as the same double.
inline std::string exactNumber(double value) {
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
  return std::string(buffer, result.ptr);
}

inline std::string shortNumber(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof buffer, "%.9g", value);
  return buffer;
}

inline std::string quotedPath(const fs::path& path) {
  return "'" + path.string() + "'";
}

// A library message without its own "prefix: ", for a caller that names
// the file itself.
inline std::string withoutPrefix(const std::string& message,
                                 std::string_view prefix) {
  if (message.compare(0, prefix.size(), prefix) == 0) {
    return message.substr(prefix.size());
  }
  return message;
}

[[noreturn]] inline void configFail(const std::string& source,
                                    const std::string& message) {
  throw std::runtime_error("learn config '" + source + "': " + message);
}

inline void rejectUnknownKeys(const toml::table& table,
                              std::initializer_list<std::string_view> allowed,
                              const std::string& where,
                              const std::string& source) {
  for (const auto& [key, node] : table) {
    (void)node;
    if (std::find(allowed.begin(), allowed.end(), key.str()) == allowed.end()) {
      configFail(source,
                 "unknown key '" + std::string(key.str()) + "' in " + where);
    }
  }
}

inline const toml::table* subTable(const toml::table& parent,
                                   std::string_view key,
                                   const std::string& name,
                                   const std::string& source) {
  const toml::node* node = parent.get(key);
  if (node == nullptr) return nullptr;
  const toml::table* table = node->as_table();
  if (table == nullptr) configFail(source, name + " must be a table");
  return table;
}

// A TOML float, or an integer that a double holds exactly; finite.
inline double numberValue(const toml::node& node, const std::string& name,
                          const std::string& source,
                          const char* expected = "a number") {
  double value = 0.0;
  if (const auto* floating = node.as_floating_point()) {
    value = floating->get();
  } else if (const auto* integer = node.as_integer()) {
    constexpr std::int64_t kExact = std::int64_t{1} << 53;
    const std::int64_t n = integer->get();
    if (n > kExact || n < -kExact) {
      configFail(source, name + " is an integer a double cannot hold exactly");
    }
    value = static_cast<double>(n);
  } else {
    configFail(source, name + " must be " + expected);
  }
  if (!std::isfinite(value)) configFail(source, name + " must be finite");
  return value;
}

inline std::optional<double> optionalNumber(const toml::table& table,
                                            std::string_view key,
                                            const std::string& name,
                                            const std::string& source) {
  const toml::node* node = table.get(key);
  if (node == nullptr) return std::nullopt;
  return numberValue(*node, name, source);
}

inline void requireAtLeastZero(double value, const std::string& name,
                               const std::string& source) {
  if (!(value >= 0.0)) {
    configFail(source, name + " must be >= 0, got " + exactNumber(value));
  }
}

inline void requirePositive(double value, const std::string& name,
                            const std::string& source) {
  if (!(value > 0.0)) {
    configFail(source, name + " must be > 0, got " + exactNumber(value));
  }
}

// One number for the seven arm joints (the gripper gets 0), or an array of
// all eight canonical joints whose gripper entry is 0. Every gain >= 0.
// Absent: all zeros (the library default: learning off).
inline rl::JointArray jointGains(const toml::table& table, std::string_view key,
                                 const std::string& name,
                                 const std::string& source) {
  rl::JointArray gains{};
  const toml::node* node = table.get(key);
  if (node == nullptr) return gains;
  if (const toml::array* values = node->as_array()) {
    if (values->size() != static_cast<std::size_t>(kDof)) {
      configFail(source, name +
                             " must be one number (the seven arm joints) or an "
                             "array of 8 (the canonical joints, gripper last), "
                             "has " +
                             std::to_string(values->size()) + " elements");
    }
    for (int j = 0; j < kDof; ++j) {
      gains[j] = numberValue((*values)[static_cast<std::size_t>(j)],
                             name + "[" + std::to_string(j) + "]", source);
    }
    if (gains[kGripperJoint] != 0.0) {
      configFail(source, name +
                             "[7] (the gripper) must be 0: the gripper is "
                             "never learned, got " +
                             exactNumber(gains[kGripperJoint]));
    }
    gains[kGripperJoint] = 0.0;
  } else {
    const double value =
        numberValue(*node, name, source, "a number or an array of 8 numbers");
    for (int j = 0; j < kArmJoints; ++j) gains[j] = value;
  }
  for (int j = 0; j < kDof; ++j) {
    requireAtLeastZero(gains[j], name + "[" + std::to_string(j) + "]", source);
  }
  return gains;
}

}  // namespace learn_detail

// Strict: unknown keys, wrong types, non-finite numbers and values outside
// their ranges are errors naming the key. The limits may only be lowered
// from the rl::OffsetLimits defaults. Throws std::runtime_error naming
// source.
inline LearnConfig parseLearnConfig(std::string_view text,
                                    const std::string& source) {
  using learn_detail::configFail;
  using learn_detail::exactNumber;
  using learn_detail::optionalNumber;
  using learn_detail::rejectUnknownKeys;
  using learn_detail::requireAtLeastZero;
  using learn_detail::requirePositive;
  using learn_detail::subTable;
  toml::table root;
  try {
    root = toml::parse(text, source);
  } catch (const toml::parse_error& error) {
    configFail(source, "cannot parse (line " +
                           std::to_string(error.source().begin.line) +
                           "): " + std::string(error.description()));
  }
  const toml::node* format = root.get("format");
  if (format == nullptr) configFail(source, "missing key format");
  const auto* format_text = format->as_string();
  if (format_text == nullptr || format_text->get() != kLearnConfigFormat) {
    configFail(source,
               std::string("format must be \"") + kLearnConfigFormat + "\"");
  }
  const toml::node* version = root.get("version");
  if (version == nullptr) configFail(source, "missing key version");
  const auto* version_value = version->as_integer();
  if (version_value == nullptr || version_value->get() != kLearnConfigVersion) {
    configFail(source, "version must be the integer " +
                           std::to_string(kLearnConfigVersion));
  }
  rejectUnknownKeys(root, {"format", "version", "limits", "ilc", "fel"},
                    "the root table", source);

  LearnConfig config;
  const rl::OffsetLimits ceiling;
  if (const toml::table* limits =
          subTable(root, "limits", "[limits]", source)) {
    rejectUnknownKeys(*limits, {"max_offset_rad", "max_offset_rate_rad_s"},
                      "[limits]", source);
    if (const auto value = optionalNumber(*limits, "max_offset_rad",
                                          "limits.max_offset_rad", source)) {
      if (!(*value > 0.0 && *value <= ceiling.max_abs_rad)) {
        configFail(source,
                   "limits.max_offset_rad must be > 0 and at most " +
                       exactNumber(ceiling.max_abs_rad) +
                       " (the default; the limits may only be lowered), "
                       "got " +
                       exactNumber(*value));
      }
      config.limits.max_abs_rad = *value;
    }
    if (const auto value =
            optionalNumber(*limits, "max_offset_rate_rad_s",
                           "limits.max_offset_rate_rad_s", source)) {
      if (!(*value > 0.0 && *value <= ceiling.max_rate_rad_s)) {
        configFail(source,
                   "limits.max_offset_rate_rad_s must be > 0 and at most " +
                       exactNumber(ceiling.max_rate_rad_s) +
                       " (the default; the limits may only be lowered), "
                       "got " +
                       exactNumber(*value));
      }
      config.limits.max_rate_rad_s = *value;
    }
  }
  config.ilc.max_abs_rad = config.limits.max_abs_rad;

  if (const toml::table* ilc = subTable(root, "ilc", "[ilc]", source)) {
    rejectUnknownKeys(
        *ilc, {"phi", "psi_s", "advance_s", "cutoff_hz", "max_trial_step_rad"},
        "[ilc]", source);
    config.ilc.phi = learn_detail::jointGains(*ilc, "phi", "ilc.phi", source);
    config.ilc.psi_s =
        learn_detail::jointGains(*ilc, "psi_s", "ilc.psi_s", source);
    if (const auto value =
            optionalNumber(*ilc, "advance_s", "ilc.advance_s", source)) {
      requireAtLeastZero(*value, "ilc.advance_s", source);
      config.ilc.advance_s = *value;
    }
    if (const auto value =
            optionalNumber(*ilc, "cutoff_hz", "ilc.cutoff_hz", source)) {
      requirePositive(*value, "ilc.cutoff_hz", source);
      config.ilc.cutoff_hz = *value;
    }
    if (const auto value = optionalNumber(*ilc, "max_trial_step_rad",
                                          "ilc.max_trial_step_rad", source)) {
      requirePositive(*value, "ilc.max_trial_step_rad", source);
      config.ilc.max_trial_step_rad = *value;
    }
  }

  if (const toml::table* fel = subTable(root, "fel", "[fel]", source)) {
    rejectUnknownKeys(*fel,
                      {"step", "gain", "advance_s", "error_lead_s",
                       "error_smooth_sigma_s", "ridge", "regressors"},
                      "[fel]", source);
    if (const toml::node* step = fel->get("step")) {
      const auto* name = step->as_string();
      if (name == nullptr) configFail(source, "fel.step must be a string");
      if (name->get() == "normalised-gradient") {
        config.fel.step = rl::FelOptions::Step::NormalisedGradient;
      } else if (name->get() == "least-squares") {
        config.fel.step = rl::FelOptions::Step::LeastSquares;
      } else {
        configFail(source,
                   "fel.step must be \"normalised-gradient\" or "
                   "\"least-squares\", got \"" +
                       name->get() + "\"");
      }
    }
    if (const auto value = optionalNumber(*fel, "gain", "fel.gain", source)) {
      if (!(*value > 0.0 && *value <= 1.0)) {
        configFail(source,
                   "fel.gain must be in (0, 1], got " + exactNumber(*value));
      }
      config.fel.gain = *value;
    }
    if (const auto value =
            optionalNumber(*fel, "advance_s", "fel.advance_s", source)) {
      requireAtLeastZero(*value, "fel.advance_s", source);
      config.fel.advance_s = *value;
    }
    if (const auto value =
            optionalNumber(*fel, "error_lead_s", "fel.error_lead_s", source)) {
      requireAtLeastZero(*value, "fel.error_lead_s", source);
      config.fel.error_lead_s = *value;
    }
    if (const auto value = optionalNumber(*fel, "error_smooth_sigma_s",
                                          "fel.error_smooth_sigma_s", source)) {
      requireAtLeastZero(*value, "fel.error_smooth_sigma_s", source);
      config.fel.error_smooth_sigma_s = *value;
    }
    if (const auto value = optionalNumber(*fel, "ridge", "fel.ridge", source)) {
      requireAtLeastZero(*value, "fel.ridge", source);
      config.fel.ridge = *value;
    }
    if (const toml::table* regressors =
            subTable(*fel, "regressors", "[fel.regressors]", source)) {
      rejectUnknownKeys(*regressors, {"smooth_sigma_s", "coulomb_v0_rad_s"},
                        "[fel.regressors]", source);
      if (const auto value =
              optionalNumber(*regressors, "smooth_sigma_s",
                             "fel.regressors.smooth_sigma_s", source)) {
        requireAtLeastZero(*value, "fel.regressors.smooth_sigma_s", source);
        config.regressors.smooth_sigma_s = *value;
      }
      if (const auto value =
              optionalNumber(*regressors, "coulomb_v0_rad_s",
                             "fel.regressors.coulomb_v0_rad_s", source)) {
        requirePositive(*value, "fel.regressors.coulomb_v0_rad_s", source);
        config.regressors.coulomb_v0_rad_s = *value;
      }
    }
  }
  try {
    rl::validateRegressorOptions(config.regressors);
  } catch (const std::exception& error) {
    configFail(source, std::string("[fel.regressors]: ") + error.what());
  }
  return config;
}

inline LearnConfig loadLearnConfig(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("learn config '" + path.string() +
                             "': cannot open");
  }
  const std::string text((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  if (input.bad()) {
    throw std::runtime_error("learn config '" + path.string() +
                             "': read error");
  }
  return parseLearnConfig(text, path.string());
}

// --------------------------------------------------------- input files

// "kind 'path'": how messages name a file.
inline std::string named(const std::string& kind, const fs::path& path) {
  return kind + " " + learn_detail::quotedPath(path);
}

// The SHA-256 of an input, which must be a regular file (symlinks to one
// are followed). `name` names the file in messages.
inline std::string hashInput(const fs::path& path, const std::string& name) {
  std::error_code error;
  const auto status = fs::status(path, error);
  if (error || !fs::is_regular_file(status)) {
    throw std::runtime_error(name + ": not an existing regular file");
  }
  try {
    return x7::ptp::sha256File(path);
  } catch (const std::exception& failure) {
    throw std::runtime_error(name + ": " + failure.what());
  }
}

// read() between two hashes of the file: the hash recorded is that of the
// bytes read, or the input changed while it was read and is refused.
template <class Read>
auto readStable(const fs::path& path, const std::string& name,
                std::string* sha256, Read&& read) -> decltype(read()) {
  const std::string before = hashInput(path, name);
  auto value = read();
  const std::string after = hashInput(path, name);
  if (before != after) {
    throw std::runtime_error(name + " changed while it was read");
  }
  *sha256 = before;
  return value;
}

// One input or output of a command, for the summary and the report.
struct FileRecord {
  std::string role;
  fs::path path;
  std::string sha256;
};

// The follow config's reference, loaded exactly as the follower loads it:
// its model, a ZvsTrajectory with the config's reference options, and the
// tracking grid of control.rate_hz.
struct FollowReference {
  fs::path follow_config_path;
  follow::Config config;
  std::string follow_config_sha256;
  std::string model_sha256;
  std::string reference_sha256;
  std::unique_ptr<model::ChainModel> chain;
  std::unique_ptr<model::JointMap> map;
  double control_dt_s = 0.0;
  double duration_s = 0.0;
  rl::TrackingGrid grid;
  // Sample i at t = min(duration, i * grid.dt_s), as FollowRun samples it.
  std::vector<JointArray> q;
  JointArray q_final{};  // at the duration: the final hold

  std::string label() const {
    return "reference " + learn_detail::quotedPath(config.reference_path) +
           " (follow config " + learn_detail::quotedPath(follow_config_path) +
           ")";
  }
};

inline FollowReference loadFollowReference(const fs::path& follow_config) {
  using learn_detail::quotedPath;
  FollowReference reference;
  reference.follow_config_path = follow_config;
  reference.config = readStable(
      follow_config, named("follow config", follow_config),
      &reference.follow_config_sha256, [&] {
        try {
          return follow::loadConfig(follow_config);
        } catch (const std::exception& error) {
          throw std::runtime_error(
              "follow config " + quotedPath(follow_config) + ": " +
              learn_detail::withoutPrefix(error.what(), "follow config: "));
        }
      });
  const follow::Config& config = reference.config;
  const std::string from = " (follow config " + quotedPath(follow_config) + ")";
  reference.chain = readStable(
      config.model_path, named("model", config.model_path) + from,
      &reference.model_sha256, [&] {
        try {
          return std::make_unique<model::ChainModel>(
              config.model_path.string());
        } catch (const std::exception& error) {
          throw std::runtime_error("model " + quotedPath(config.model_path) +
                                   from + ": " + error.what());
        }
      });
  try {
    reference.map = std::make_unique<model::JointMap>(*reference.chain);
  } catch (const std::exception& error) {
    throw std::runtime_error("model " + quotedPath(config.model_path) + from +
                             ": " + error.what());
  }
  const auto trajectory = readStable(
      config.reference_path, named("reference", config.reference_path) + from,
      &reference.reference_sha256, [&] {
        try {
          return std::make_unique<model::ZvsTrajectory>(
              config.reference_path.string(), *reference.map, config.reference);
        } catch (const std::exception& error) {
          throw std::runtime_error("reference " +
                                   quotedPath(config.reference_path) + from +
                                   ": " + error.what());
        }
      });
  reference.control_dt_s = 1.0 / config.control_rate_hz;
  reference.duration_s = trajectory->duration();
  try {
    reference.grid =
        rl::trackingGrid(reference.duration_s, reference.control_dt_s);
  } catch (const std::exception& error) {
    throw std::runtime_error(reference.label() + ": " + error.what());
  }
  model::ZVector q(kDof);
  reference.q.resize(reference.grid.samples());
  for (std::size_t i = 0; i < reference.grid.samples(); ++i) {
    const double t = std::min(reference.duration_s,
                              static_cast<double>(i) * reference.grid.dt_s);
    trajectory->sample(t, q.get());
    for (int j = 0; j < kDof; ++j) reference.q[i][j] = q[j];
  }
  trajectory->sample(reference.duration_s, q.get());
  for (int j = 0; j < kDof; ++j) reference.q_final[j] = q[j];
  return reference;
}

inline void appendReferenceInputs(const FollowReference& reference,
                                  std::vector<FileRecord>* inputs) {
  inputs->push_back({"follow_config", reference.follow_config_path,
                     reference.follow_config_sha256});
  inputs->push_back(
      {"model", reference.config.model_path, reference.model_sha256});
  inputs->push_back({"reference", reference.config.reference_path,
                     reference.reference_sha256});
}

inline std::string offsetLabel(const fs::path& path) {
  return named("offset", path);
}

inline std::string logLabel(const fs::path& path) {
  return named("follow log", path);
}

// An offset file that may be applied to the run of this reference under
// these limits (rl::checkOffsetForRun), with a gripper column of zeros.
inline rl::CommandOffset readOffsetForRun(const fs::path& path,
                                          const FollowReference& reference,
                                          const rl::OffsetLimits& limits,
                                          std::string* sha256) {
  rl::CommandOffset offset =
      readStable(path, named("offset", path), sha256,
                 [&] { return rl::readCommandOffset(path); });
  try {
    rl::checkOffsetForRun(offset, reference.duration_s, reference.control_dt_s,
                          reference.reference_sha256, limits);
  } catch (const std::exception& error) {
    throw std::runtime_error(
        offsetLabel(path) + " does not belong to the run of " +
        reference.label() + " under the configured limits: " +
        learn_detail::withoutPrefix(error.what(), "command offset: "));
  }
  for (std::size_t i = 0; i < offset.delta_rad.size(); ++i) {
    if (offset.delta_rad[i][kGripperJoint] != 0.0) {
      throw std::runtime_error(
          offsetLabel(path) + ": the gripper (joint 7) offset at sample " +
          std::to_string(i) + " is " +
          learn_detail::exactNumber(offset.delta_rad[i][kGripperJoint]) +
          ": the gripper is never learned, so no learned offset moves it");
    }
  }
  return offset;
}

// ----------------------------------------------------------- follow log

enum class LogPhase {
  Home,
  HomeCorrection,
  OffsetRamp,
  Tracking,
  Finalizing,
  AbortHold
};

inline const char* logPhaseName(LogPhase phase) {
  switch (phase) {
    case LogPhase::Home: return "home";
    case LogPhase::HomeCorrection: return "home_correction";
    case LogPhase::OffsetRamp: return "offset_ramp";
    case LogPhase::Tracking: return "tracking";
    case LogPhase::Finalizing: return "finalizing";
    case LogPhase::AbortHold: return "abort_hold";
  }
  return "unknown";
}

inline bool parseLogPhase(std::string_view text, LogPhase* phase) {
  for (const LogPhase candidate :
       {LogPhase::Home, LogPhase::HomeCorrection, LogPhase::OffsetRamp,
        LogPhase::Tracking, LogPhase::Finalizing, LogPhase::AbortHold}) {
    if (text == logPhaseName(candidate)) {
      *phase = candidate;
      return true;
    }
  }
  return false;
}

// One cycle of the follow log (FollowCsvLog), the columns this tool uses.
struct LogRow {
  std::size_t line = 0;  // 1-based line in the file (the header is line 1)
  LogPhase phase = LogPhase::Home;
  double phase_time_s = 0.0;
  bool accepted = false;  // receipt_accepted
  JointArray qref{};
  JointArray q{};
  JointArray qcmd{};
};

namespace learn_detail {

[[noreturn]] inline void logFail(const std::string& source, std::size_t line,
                                 const std::string& problem) {
  throw std::runtime_error(source + ": line " + std::to_string(line) + ": " +
                           problem);
}

// The columns read, by name: schema_version, phase, phase_time_s,
// receipt_accepted, then qref{j}_rad, q{j}_rad and qcmd{j}_rad.
inline std::vector<std::string> logColumns() {
  std::vector<std::string> names = {"schema_version", "phase", "phase_time_s",
                                    "receipt_accepted"};
  for (const char* prefix : {"qref", "q", "qcmd"}) {
    for (int j = 0; j < kDof; ++j) {
      names.push_back(prefix + std::to_string(j) + "_rad");
    }
  }
  return names;
}

}  // namespace learn_detail

// Strict reader of the follow log: the columns above by name (any order,
// other columns ignored, a required name twice refused); every row with the
// header's field count, schema_version 1, a known phase (offset_ramp
// included), receipt_accepted 0 or 1 and finite numbers; a final newline
// (a log that ends inside a row is incomplete). Throws std::runtime_error
// naming source and line.
inline std::vector<LogRow> parseFollowLog(std::istream& input,
                                          const std::string& source) {
  using learn_detail::logFail;
  namespace csv = detail;
  std::string line;
  if (!std::getline(input, line)) {
    if (input.bad()) throw std::runtime_error(source + ": read error");
    throw std::runtime_error(source + ": empty (no header line)");
  }
  if (input.eof()) {
    logFail(source, 1, "missing final newline (an incomplete log)");
  }
  std::vector<std::string_view> fields;
  csv::splitFields(line, &fields);
  const std::size_t width = fields.size();
  const std::vector<std::string> names = learn_detail::logColumns();
  std::vector<std::size_t> column(names.size(), std::string_view::npos);
  for (std::size_t c = 0; c < width; ++c) {
    for (std::size_t k = 0; k < names.size(); ++k) {
      if (fields[c] != names[k]) continue;
      if (column[k] != std::string_view::npos) {
        logFail(source, 1, "duplicate column '" + names[k] + "'");
      }
      column[k] = c;
    }
  }
  std::string missing;
  for (std::size_t k = 0; k < names.size(); ++k) {
    if (column[k] != std::string_view::npos) continue;
    missing += (missing.empty() ? "" : ", ") + names[k];
  }
  if (!missing.empty()) {
    logFail(source, 1, "missing column(s): " + missing);
  }

  std::vector<LogRow> rows;
  std::size_t number = 1;
  while (std::getline(input, line)) {
    ++number;
    if (input.eof()) {
      logFail(source, number, "missing final newline (an incomplete log)");
    }
    csv::splitFields(line, &fields);
    if (fields.size() != width) {
      logFail(source, number,
              std::to_string(fields.size()) + " fields, the header has " +
                  std::to_string(width));
    }
    LogRow row;
    row.line = number;
    const std::string_view schema = fields[column[0]];
    if (schema != "1") {
      logFail(source, number,
              "schema_version " + csv::quotedField(schema) + ", expected 1");
    }
    const std::string_view phase = fields[column[1]];
    if (!parseLogPhase(phase, &row.phase)) {
      logFail(source, number, "unknown phase " + csv::quotedField(phase));
    }
    if (!csv::parseFinite(fields[column[2]], &row.phase_time_s)) {
      logFail(source, number,
              "column 'phase_time_s': not a finite number: " +
                  csv::quotedField(fields[column[2]]));
    }
    const std::string_view accepted = fields[column[3]];
    if (accepted == "1") {
      row.accepted = true;
    } else if (accepted != "0") {
      logFail(source, number,
              "column 'receipt_accepted' must be 0 or 1, got " +
                  csv::quotedField(accepted));
    }
    for (std::size_t k = 4; k < names.size(); ++k) {
      const std::size_t index = k - 4;
      const int j = static_cast<int>(index % kDof);
      JointArray* target = index < static_cast<std::size_t>(kDof) ? &row.qref
                           : index < static_cast<std::size_t>(2 * kDof)
                               ? &row.q
                               : &row.qcmd;
      if (!csv::parseFinite(fields[column[k]], &(*target)[j])) {
        logFail(source, number,
                "column '" + names[k] + "': not a finite number: " +
                    csv::quotedField(fields[column[k]]));
      }
    }
    rows.push_back(row);
  }
  if (input.bad()) throw std::runtime_error(source + ": read error");
  if (rows.empty()) throw std::runtime_error(source + ": no data rows");
  return rows;
}

inline std::vector<LogRow> readFollowLog(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error(logLabel(path) + ": cannot open");
  }
  return parseFollowLog(input, logLabel(path));
}

// What the learning library gets from one trial.
struct TrialRecord {
  // e = qref - q (the negative of the log's error column): the tracking
  // samples, then the final-hold tail.
  std::vector<JointArray> error;
  std::size_t tracking_samples = 0;
  std::size_t tail_samples = 0;
};

// The log must be a completed run of this reference with this offset: no
// abort_hold row; one contiguous block of exactly grid.samples() tracking
// rows, preceded only by home, home_correction and offset_ramp rows and
// followed only by finalizing rows. Tracking row i: receipt_accepted 1,
// phase_time_s the grid time min(duration, i * dt) within
// kLogTimeToleranceS, qref the reference sample within
// kLogPositionToleranceRad and qcmd - qref the offset's delta[i] within
// the same, on every joint. The tail is the first kMaxTailRows (or fewer)
// finalizing rows; finalizing row k: receipt_accepted 1, phase_time_s
// k * control_dt, qref the reference's final value and qcmd - qref the
// offset's last sample. Throws std::runtime_error naming source, line and
// value.
inline TrialRecord trialRecord(const std::vector<LogRow>& rows,
                               const FollowReference& reference,
                               const rl::CommandOffset& offset,
                               const std::string& source,
                               const std::string& offset_label) {
  using learn_detail::exactNumber;
  using learn_detail::logFail;
  using learn_detail::shortNumber;
  const std::size_t n = reference.grid.samples();
  if (offset.delta_rad.size() != n) {
    throw std::logic_error("trialRecord: offset size differs from the grid");
  }
  for (const LogRow& row : rows) {
    if (row.phase == LogPhase::AbortHold) {
      logFail(source, row.line,
              "phase 'abort_hold': the run aborted, so the trial did not "
              "complete");
    }
  }
  std::size_t begin = 0;
  while (begin < rows.size() && rows[begin].phase != LogPhase::Tracking) {
    ++begin;
  }
  if (begin == rows.size()) {
    throw std::runtime_error(source +
                             ": no tracking rows: the trial never reached "
                             "tracking");
  }
  std::size_t end = begin;
  while (end < rows.size() && rows[end].phase == LogPhase::Tracking) ++end;
  for (std::size_t r = end; r < rows.size(); ++r) {
    if (rows[r].phase == LogPhase::Tracking) {
      logFail(source, rows[r].line,
              "a tracking row after the tracking block that started at "
              "line " +
                  std::to_string(rows[begin].line) +
                  " was interrupted at "
                  "line " +
                  std::to_string(rows[end].line) + " (phase '" +
                  logPhaseName(rows[end].phase) +
                  "'): the tracking rows are not one contiguous block");
    }
  }
  const std::size_t count = end - begin;
  const std::string grid_text = "the tracking grid of " + reference.label() +
                                " at " +
                                shortNumber(reference.config.control_rate_hz) +
                                " Hz has " + std::to_string(n) + " samples";
  if (count < n) {
    throw std::runtime_error(
        source + ": the tracking block (lines " +
        std::to_string(rows[begin].line) + " to " +
        std::to_string(rows[end - 1].line) + ") has " + std::to_string(count) +
        " rows, " + grid_text +
        ": the trial did not complete (or the log is not a run of this "
        "reference at this control rate)");
  }
  if (count > n) {
    throw std::runtime_error(
        source + ": the tracking block (lines " +
        std::to_string(rows[begin].line) + " to " +
        std::to_string(rows[end - 1].line) + ") has " + std::to_string(count) +
        " rows, " + grid_text +
        ": the log is not a run of this reference at this control rate");
  }
  for (std::size_t r = 0; r < begin; ++r) {
    const LogPhase phase = rows[r].phase;
    if (phase != LogPhase::Home && phase != LogPhase::HomeCorrection &&
        phase != LogPhase::OffsetRamp) {
      logFail(source, rows[r].line,
              std::string("phase '") + logPhaseName(phase) +
                  "' before the tracking block");
    }
  }
  for (std::size_t r = end; r < rows.size(); ++r) {
    if (rows[r].phase != LogPhase::Finalizing) {
      logFail(source, rows[r].line,
              std::string("phase '") + logPhaseName(rows[r].phase) +
                  "' after the tracking block; only 'finalizing' may "
                  "follow it");
    }
  }

  const double tolerance = kLogPositionToleranceRad;
  const auto checkRow = [&](const LogRow& row, const std::string& what,
                            double expected_time, const JointArray& qref,
                            const std::string& qref_what,
                            const JointArray& delta,
                            const std::string& delta_what) {
    if (!row.accepted) {
      logFail(source, row.line,
              what +
                  ": receipt_accepted is 0: the command of that cycle was "
                  "not accepted");
    }
    if (!(std::fabs(row.phase_time_s - expected_time) <= kLogTimeToleranceS)) {
      logFail(source, row.line,
              what + ": phase_time_s " + exactNumber(row.phase_time_s) +
                  " differs from the expected " + exactNumber(expected_time) +
                  " s by more than 1e-6 s");
    }
    for (int j = 0; j < kDof; ++j) {
      const std::string joint = std::to_string(j);
      const double reference_gap = std::fabs(row.qref[j] - qref[j]);
      if (!(reference_gap <= tolerance)) {
        logFail(source, row.line,
                what + ": qref" + joint + "_rad " + exactNumber(row.qref[j]) +
                    " differs from " + qref_what + " " + exactNumber(qref[j]) +
                    " by " + shortNumber(reference_gap) +
                    " rad (tolerance 1e-6 rad): the log is not a run of " +
                    reference.label());
      }
      const double applied = row.qcmd[j] - row.qref[j];
      const double offset_gap = std::fabs(applied - delta[j]);
      if (!(offset_gap <= tolerance)) {
        logFail(source, row.line,
                what + ": qcmd" + joint + "_rad - qref" + joint + "_rad = " +
                    exactNumber(applied) + " differs from " + delta_what +
                    " delta" + joint + "_rad = " + exactNumber(delta[j]) +
                    " of " + offset_label + " by " + shortNumber(offset_gap) +
                    " rad (tolerance 1e-6 rad): the log is not a run with "
                    "this offset");
      }
    }
  };

  TrialRecord record;
  record.tracking_samples = n;
  record.tail_samples = std::min(rows.size() - end, kMaxTailRows);
  record.error.reserve(n + record.tail_samples);
  for (std::size_t i = 0; i < n; ++i) {
    const LogRow& row = rows[begin + i];
    const double t = std::min(reference.duration_s,
                              static_cast<double>(i) * reference.grid.dt_s);
    const std::string what = "tracking sample " + std::to_string(i) +
                             " (t = " + shortNumber(t) + " s)";
    checkRow(row, what, t, reference.q[i],
             "the reference at t = " + shortNumber(t) + " s",
             offset.delta_rad[i], "sample " + std::to_string(i) + "'s");
    JointArray e{};
    for (int j = 0; j < kDof; ++j) e[j] = row.qref[j] - row.q[j];
    record.error.push_back(e);
  }
  for (std::size_t k = 0; k < record.tail_samples; ++k) {
    const LogRow& row = rows[end + k];
    const double t = static_cast<double>(k) * reference.control_dt_s;
    checkRow(row, "final-hold row " + std::to_string(k), t, reference.q_final,
             "the reference's final value", offset.delta_rad.back(),
             "the last sample's");
    JointArray e{};
    for (int j = 0; j < kDof; ++j) e[j] = row.qref[j] - row.q[j];
    record.error.push_back(e);
  }
  return record;
}

// ------------------------------------------------------------- outputs

// The outputs of one command: checked before any work (each non-empty,
// distinct and absent; the exclusive creation when written is the
// authority) and removed again, newest first, unless commit() is reached.
class NewOutputs {
 public:
  explicit NewOutputs(
      const std::vector<std::pair<std::string, fs::path>>& outputs) {
    std::vector<fs::path> seen;
    for (const auto& [option, path] : outputs) {
      if (path.empty()) {
        throw std::runtime_error(option + " must be a non-empty path");
      }
      const fs::path normal = fs::absolute(path).lexically_normal();
      for (std::size_t k = 0; k < seen.size(); ++k) {
        if (seen[k] == normal) {
          throw std::runtime_error(
              option + " " + learn_detail::quotedPath(path) + " is also " +
              outputs[k].first + ": every output needs its own file");
        }
      }
      seen.push_back(normal);
      std::error_code error;
      if (fs::exists(fs::symlink_status(path, error))) {
        throw std::runtime_error(option + ": refusing to overwrite " +
                                 learn_detail::quotedPath(path));
      }
    }
  }
  NewOutputs(const NewOutputs&) = delete;
  NewOutputs& operator=(const NewOutputs&) = delete;
  ~NewOutputs() {
    if (committed_) return;
    for (auto it = created_.rbegin(); it != created_.rend(); ++it) {
      std::error_code error;
      fs::remove(*it, error);
      if (error) {
        std::fprintf(stderr,
                     "x7_follow_learn: could not remove the partial output "
                     "'%s': %s\n",
                     it->string().c_str(), error.message().c_str());
      }
    }
  }

  void created(const fs::path& path) { created_.push_back(path); }
  void commit() { committed_ = true; }

 private:
  std::vector<fs::path> created_;
  bool committed_ = false;
};

namespace learn_detail {

inline bool sameOffset(const rl::CommandOffset& a, const rl::CommandOffset& b) {
  if (a.method != b.method || a.trial != b.trial || a.dt_s != b.dt_s ||
      a.reference_duration_s != b.reference_duration_s ||
      a.reference_sha256 != b.reference_sha256 ||
      a.parent_sha256 != b.parent_sha256 ||
      a.source_log_sha256 != b.source_log_sha256 ||
      a.delta_rad.size() != b.delta_rad.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.delta_rad.size(); ++i) {
    if (std::memcmp(a.delta_rad[i].data(), b.delta_rad[i].data(),
                    sizeof(double) * kDof) != 0) {
      return false;
    }
  }
  return true;
}

inline bool sameWeights(const rl::FeedforwardWeights& a,
                        const rl::FeedforwardWeights& b) {
  return a.trial == b.trial && std::memcmp(&a.w, &b.w, sizeof a.w) == 0 &&
         a.regressors.smooth_sigma_s == b.regressors.smooth_sigma_s &&
         a.regressors.coulomb_v0_rad_s == b.regressors.coulomb_v0_rad_s &&
         a.parent_sha256 == b.parent_sha256 &&
         a.source_log_sha256 == b.source_log_sha256 &&
         a.reference_sha256 == b.reference_sha256;
}

}  // namespace learn_detail

// Writes the offset exclusively, reads it back (it must be what was
// written, bit for bit) and returns the file's SHA-256.
inline std::string writeOffsetChecked(const fs::path& path,
                                      const rl::CommandOffset& offset,
                                      NewOutputs* outputs) {
  rl::writeCommandOffset(path, offset);
  outputs->created(path);
  std::string sha256;
  const rl::CommandOffset back =
      readStable(path, named("written offset", path), &sha256,
                 [&] { return rl::readCommandOffset(path); });
  if (!learn_detail::sameOffset(back, offset)) {
    throw std::runtime_error("written offset " +
                             learn_detail::quotedPath(path) +
                             " does not read back as written");
  }
  return sha256;
}

// The same for weights; returns the weights as read back.
inline rl::FeedforwardWeights writeWeightsChecked(
    const fs::path& path, const rl::FeedforwardWeights& weights,
    NewOutputs* outputs, std::string* sha256) {
  rl::writeFeedforwardWeights(path, weights);
  outputs->created(path);
  rl::FeedforwardWeights back =
      readStable(path, named("written weights", path), sha256,
                 [&] { return rl::readFeedforwardWeights(path); });
  if (!learn_detail::sameWeights(back, weights)) {
    throw std::runtime_error("written weights " +
                             learn_detail::quotedPath(path) +
                             " do not read back as written");
  }
  return back;
}

// Creates path exclusively and writes text; a partial file is removed.
inline void writeNewText(const fs::path& path, const std::string& text,
                         const std::string& label) {
  std::FILE* file = std::fopen(path.string().c_str(), "wx");
  if (file == nullptr) {
    const int error = errno;
    if (error == EEXIST) {
      throw std::runtime_error(label + ": refusing to overwrite " +
                               learn_detail::quotedPath(path));
    }
    throw std::runtime_error(label + ": cannot create " +
                             learn_detail::quotedPath(path) + ": " +
                             std::strerror(error));
  }
  bool ok = std::fwrite(text.data(), 1, text.size(), file) == text.size();
  ok = ok && std::fflush(file) == 0 && std::ferror(file) == 0;
  const int closed = std::fclose(file);
  if (!ok || closed != 0) {
    std::error_code ignored;
    fs::remove(path, ignored);
    throw std::runtime_error(label + ": cannot write " +
                             learn_detail::quotedPath(path));
  }
}

// ------------------------------------------------------------- results

// What a command did: printed as the operator's table and, for ilc and
// fel, written as the --report TOML.
struct LearnResult {
  std::string command;
  int measured_trial = -1;  // the trial the log recorded; -1: no log
  int output_trial = 0;
  std::size_t tracking_samples = 0;
  std::size_t tail_samples = 0;
  double dt_s = 0.0;
  JointArray error_rms{};      // tracking RMS of e = qref - q per joint
  double aggregate_rms = 0.0;  // over all 8 joints
  double arm_rms = 0.0;        // over joints 0..6
  bool has_offset = false;
  std::size_t offset_samples = 0;
  JointArray offset_rms{};
  JointArray offset_max_abs{};
  std::optional<rl::IlcReport> ilc;
  std::optional<rl::FelReport> fel;
  std::vector<FileRecord> inputs;
  std::vector<FileRecord> outputs;
};

// sqrt(mean over the first `joints` joints of rms^2): the RMS over all
// samples of those joints.
inline double aggregateRms(const JointArray& rms, int joints) {
  double sum = 0.0;
  for (int j = 0; j < joints; ++j) sum += rms[j] * rms[j];
  return std::sqrt(sum / static_cast<double>(joints));
}

inline void fillTracking(const TrialRecord& record, LearnResult* result) {
  result->tracking_samples = record.tracking_samples;
  result->tail_samples = record.tail_samples;
  for (int j = 0; j < kDof; ++j) {
    double sum = 0.0;
    for (std::size_t i = 0; i < record.tracking_samples; ++i) {
      sum += record.error[i][j] * record.error[i][j];
    }
    result->error_rms[j] =
        std::sqrt(sum / static_cast<double>(record.tracking_samples));
  }
  result->aggregate_rms = aggregateRms(result->error_rms, kDof);
  result->arm_rms = aggregateRms(result->error_rms, kArmJoints);
}

inline void fillOffset(const rl::CommandOffset& offset, LearnResult* result) {
  result->has_offset = true;
  result->offset_samples = offset.delta_rad.size();
  result->dt_s = offset.dt_s;
  result->output_trial = offset.trial;
  for (int j = 0; j < kDof; ++j) {
    double sum = 0.0;
    double peak = 0.0;
    for (const auto& sample : offset.delta_rad) {
      sum += sample[j] * sample[j];
      peak = std::max(peak, std::fabs(sample[j]));
    }
    result->offset_rms[j] =
        std::sqrt(sum / static_cast<double>(offset.delta_rad.size()));
    result->offset_max_abs[j] = peak;
  }
}

namespace learn_detail {

inline std::string tomlString(const std::string& text) {
  std::string out = "\"";
  for (const char c : text) {
    const auto byte = static_cast<unsigned char>(c);
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (byte < 0x20 || byte == 0x7f) {
      char buffer[8];
      std::snprintf(buffer, sizeof buffer, "\\u%04x", byte);
      out += buffer;
    } else {
      out += c;
    }
  }
  return out + "\"";
}

inline std::string tomlFloat(double value) {
  if (std::isnan(value)) return "nan";
  if (std::isinf(value)) return value > 0 ? "inf" : "-inf";
  char buffer[40];
  std::snprintf(buffer, sizeof buffer, "%.17g", value);
  std::string text = buffer;
  if (text.find_first_of(".e") == std::string::npos) text += ".0";
  return text;
}

template <class T, class Format>
std::string tomlArray(const T& values, Format&& format) {
  std::string out = "[";
  bool first = true;
  for (const auto& value : values) {
    out += (first ? "" : ", ") + format(value);
    first = false;
  }
  return out + "]";
}

inline std::string tomlFloats(const JointArray& values) {
  return tomlArray(values, [](double v) { return tomlFloat(v); });
}

inline std::string skippedNames(const rl::FelJointReport& joint,
                                bool for_toml) {
  std::string out;
  for (int l = 0; l < rl::kRegressorCount; ++l) {
    if (!joint.skipped[l]) continue;
    const std::string name(rl::kRegressorNames[l]);
    if (for_toml) {
      out += (out.empty() ? "" : ", ") + tomlString(name);
    } else {
      out += (out.empty() ? "" : "+") + name;
    }
  }
  return for_toml ? "[" + out + "]" : (out.empty() ? "-" : out);
}

}  // namespace learn_detail

// The --report file: the summary's numbers and the SHA-256 of every input
// and of the outputs written before the report.
inline std::string formatReport(const LearnResult& result) {
  using learn_detail::tomlArray;
  using learn_detail::tomlFloat;
  using learn_detail::tomlFloats;
  using learn_detail::tomlString;
  std::string out;
  out += "format = " + tomlString(kLearnReportFormat) + "\n";
  out += "version = " + std::to_string(kLearnReportVersion) + "\n";
  out += "command = " + tomlString(result.command) + "\n";
  out += "measured_trial = " + std::to_string(result.measured_trial) + "\n";
  out += "output_trial = " + std::to_string(result.output_trial) + "\n";
  out += "dt_s = " + tomlFloat(result.dt_s) + "\n";
  out += "tracking_samples = " + std::to_string(result.tracking_samples) + "\n";
  out += "tail_samples = " + std::to_string(result.tail_samples) + "\n";
  out +=
      "\n# The measured trial's tracking error e = qref - q over the "
      "tracking samples.\n[tracking]\n";
  out += "aggregate_rms_rad = " + tomlFloat(result.aggregate_rms) +
         "  # all 8 joints\n";
  out += "arm_rms_rad = " + tomlFloat(result.arm_rms) + "  # joints 0-6\n";
  out += "error_rms_rad = " + tomlFloats(result.error_rms) + "\n";
  out += "\n# The new offset.\n[offset]\n";
  out += "rms_rad = " + tomlFloats(result.offset_rms) + "\n";
  out += "max_abs_rad = " + tomlFloats(result.offset_max_abs) + "\n";
  if (result.ilc) {
    const auto& joints = result.ilc->joints;
    const auto field = [&joints](auto member) {
      std::array<std::remove_cv_t<
                     std::remove_reference_t<decltype(joints[0].*member)>>,
                 kDof>
          values{};
      for (int j = 0; j < kDof; ++j) values[j] = joints[j].*member;
      return values;
    };
    out += "\n[ilc]\n";
    out += "enabled = " +
           tomlArray(field(&rl::IlcJointReport::enabled),
                     [](bool v) { return std::string(v ? "true" : "false"); }) +
           "\n";
    const auto count = [](std::size_t v) { return std::to_string(v); };
    out += "step_clamped = " +
           tomlArray(field(&rl::IlcJointReport::step_clamped), count) + "\n";
    out += "abs_clamped = " +
           tomlArray(field(&rl::IlcJointReport::abs_clamped), count) + "\n";
    out += "change_rms_rad = " +
           tomlFloats(field(&rl::IlcJointReport::change_rms_rad)) + "\n";
    out += "change_max_abs_rad = " +
           tomlFloats(field(&rl::IlcJointReport::change_max_abs_rad)) + "\n";
  }
  if (result.fel) {
    const auto& joints = result.fel->joints;
    const auto field = [&joints](auto member) {
      std::array<std::remove_cv_t<
                     std::remove_reference_t<decltype(joints[0].*member)>>,
                 kDof>
          values{};
      for (int j = 0; j < kDof; ++j) values[j] = joints[j].*member;
      return values;
    };
    out += "\n[fel]\n";
    out += "enabled = " +
           tomlArray(field(&rl::FelJointReport::enabled),
                     [](bool v) { return std::string(v ? "true" : "false"); }) +
           "\n";
    out += "kept_columns = " +
           tomlArray(field(&rl::FelJointReport::kept_columns),
                     [](int v) { return std::to_string(v); }) +
           "\n";
    std::string skipped = "[";
    for (int j = 0; j < kDof; ++j) {
      skipped +=
          (j == 0 ? "" : ", ") + learn_detail::skippedNames(joints[j], true);
    }
    out += "skipped = " + skipped + "]\n";
    out +=
        "lambda_max = " + tomlFloats(field(&rl::FelJointReport::lambda_max)) +
        "\n";
    out +=
        "lambda_min = " + tomlFloats(field(&rl::FelJointReport::lambda_min)) +
        "\n";
    out +=
        "s_rms_rad = " + tomlFloats(field(&rl::FelJointReport::s_rms)) + "\n";
    out += "residual_rms_rad = " +
           tomlFloats(field(&rl::FelJointReport::residual_rms)) +
           "  # predicted\n";
  }
  for (const auto* list : {&result.inputs, &result.outputs}) {
    const char* table = list == &result.inputs ? "input" : "output";
    for (const FileRecord& file : *list) {
      out += std::string("\n[[") + table + "]]\n";
      out += "role = " + tomlString(file.role) + "\n";
      out += "path = " + tomlString(file.path.string()) + "\n";
      out += "sha256 = " + tomlString(file.sha256) + "\n";
    }
  }
  return out;
}

// The operator's table.
inline std::string formatSummary(const LearnResult& result) {
  std::string out;
  // Fixed-width cells only; the paths are appended whole below.
  char line[256];
  const auto add = [&out, &line](int length) {
    if (length > 0) {
      out.append(line, std::min<std::size_t>(length, sizeof line - 1));
    }
  };
  for (const FileRecord& file : result.outputs) {
    out += result.command + ": wrote " + file.role + " " + file.path.string() +
           " (sha256 " + file.sha256 + ")\n";
  }
  if (!result.has_offset) {
    add(std::snprintf(line, sizeof line, "%s: trial %d\n",
                      result.command.c_str(), result.output_trial));
    return out;
  }
  add(std::snprintf(line, sizeof line,
                    "%s: trial %d offset, %zu samples at dt %.9g s\n",
                    result.command.c_str(), result.output_trial,
                    result.offset_samples, result.dt_s));
  const bool measured = result.measured_trial >= 0;
  if (measured) {
    add(std::snprintf(line, sizeof line,
                      "log of trial %d: %zu tracking samples, %zu final-hold "
                      "samples\n",
                      result.measured_trial, result.tracking_samples,
                      result.tail_samples));
  }
  std::string header = "joint";
  if (measured) header += "  error_rms_rad";
  header += "  offset_rms_rad  offset_max_rad";
  if (result.ilc) header += "  step_clamped  abs_clamped";
  if (result.fel) header += "    lambda_max  skipped";
  out += header + "\n";
  for (int j = 0; j < kDof; ++j) {
    add(std::snprintf(line, sizeof line, "%5d", j));
    if (measured) {
      add(std::snprintf(line, sizeof line, "  %13.6e", result.error_rms[j]));
    }
    add(std::snprintf(line, sizeof line, "  %14.6e  %14.6e",
                      result.offset_rms[j], result.offset_max_abs[j]));
    if (result.ilc) {
      const auto& joint = result.ilc->joints[j];
      if (joint.enabled) {
        add(std::snprintf(line, sizeof line, "  %12zu  %11zu",
                          joint.step_clamped, joint.abs_clamped));
      } else {
        add(std::snprintf(line, sizeof line, "  %12s  %11s", "off", "off"));
      }
    }
    if (result.fel) {
      const auto& joint = result.fel->joints[j];
      if (joint.enabled) {
        add(std::snprintf(line, sizeof line, "  %12.6g  %s", joint.lambda_max,
                          learn_detail::skippedNames(joint, false).c_str()));
      } else {
        add(std::snprintf(line, sizeof line, "  %12s  %s", "off", "-"));
      }
    }
    out += "\n";
  }
  if (measured) {
    add(std::snprintf(line, sizeof line,
                      "tracking RMS of trial %d: %.6e rad over all 8 joints, "
                      "%.6e rad over arm joints 0-6\n",
                      result.measured_trial, result.aggregate_rms,
                      result.arm_rms));
  }
  return out;
}

// ------------------------------------------------------------ commands

struct IlcInputs {
  fs::path config;
  fs::path follow_config;
  fs::path log;
  fs::path offset;
  fs::path output;
  std::optional<fs::path> report;
};

struct FelInputs {
  fs::path config;
  fs::path follow_config;
  fs::path log;
  fs::path offset;
  fs::path weights;
  fs::path output_weights;
  fs::path output;
  std::optional<fs::path> report;
};

struct SynthInputs {
  fs::path config;
  fs::path weights;
  fs::path follow_config;
  fs::path output;
};

namespace learn_detail {

inline LearnConfig readConfig(const fs::path& path, LearnResult* result) {
  std::string sha256;
  LearnConfig config = readStable(path, named("learn config", path), &sha256,
                                  [&] { return loadLearnConfig(path); });
  result->inputs.push_back({"config", path, sha256});
  return config;
}

inline rl::FeedforwardWeights readWeights(const fs::path& path,
                                          const LearnConfig& config,
                                          const fs::path& config_path,
                                          std::string* sha256,
                                          LearnResult* result) {
  rl::FeedforwardWeights weights =
      readStable(path, named("weights", path), sha256,
                 [&] { return rl::readFeedforwardWeights(path); });
  result->inputs.push_back({"weights", path, *sha256});
  if (weights.regressors.smooth_sigma_s != config.regressors.smooth_sigma_s ||
      weights.regressors.coulomb_v0_rad_s !=
          config.regressors.coulomb_v0_rad_s) {
    throw std::runtime_error("weights " + quotedPath(path) +
                             " were learned with regressors.smooth_sigma_s = " +
                             exactNumber(weights.regressors.smooth_sigma_s) +
                             ", coulomb_v0_rad_s = " +
                             exactNumber(weights.regressors.coulomb_v0_rad_s) +
                             "; learn config " + quotedPath(config_path) +
                             " has fel.regressors.smooth_sigma_s = " +
                             exactNumber(config.regressors.smooth_sigma_s) +
                             ", coulomb_v0_rad_s = " +
                             exactNumber(config.regressors.coulomb_v0_rad_s) +
                             ": the regressors would differ");
  }
  return weights;
}

inline std::vector<rl::RegressorRow> regressorsOf(
    FollowReference& reference, const rl::RegressorOptions& options) {
  try {
    return rl::buildRegressors(*reference.chain, *reference.map, reference.q,
                               reference.grid.dt_s, options);
  } catch (const std::exception& error) {
    throw std::runtime_error(reference.label() +
                             ": cannot build the regressors: " + error.what());
  }
}

inline rl::CommandOffset offsetOnGrid(const FollowReference& reference,
                                      const std::string& method, int trial) {
  rl::CommandOffset offset;
  offset.method = method;
  offset.trial = trial;
  offset.dt_s = reference.grid.dt_s;
  offset.reference_duration_s = reference.duration_s;
  offset.reference_sha256 = reference.reference_sha256;
  return offset;
}

inline void requireNextTrial(int trial, const std::string& what) {
  if (trial == INT_MAX) {
    throw std::runtime_error(what + " is trial " + std::to_string(trial) +
                             ": no trial number follows it");
  }
}

}  // namespace learn_detail

// init: the trial-0 offset of the follow config's reference.
inline LearnResult runInit(const fs::path& follow_config,
                           const fs::path& output) {
  NewOutputs outputs({{"--output", output}});
  LearnResult result;
  result.command = "init";
  const FollowReference reference = loadFollowReference(follow_config);
  appendReferenceInputs(reference, &result.inputs);
  const rl::CommandOffset offset = rl::zeroOffset(
      reference.duration_s, reference.control_dt_s, reference.reference_sha256);
  rl::checkOffsetForRun(offset, reference.duration_s, reference.control_dt_s,
                        reference.reference_sha256, rl::OffsetLimits{});
  fillOffset(offset, &result);
  result.outputs.push_back(
      {"offset", output, writeOffsetChecked(output, offset, &outputs)});
  outputs.commit();
  return result;
}

// init-weights: trial-0 weights (all zero) with the learn config's
// regressor options.
inline LearnResult runInitWeights(const fs::path& config_path,
                                  const fs::path& output) {
  NewOutputs outputs({{"--output", output}});
  LearnResult result;
  result.command = "init-weights";
  const LearnConfig config = learn_detail::readConfig(config_path, &result);
  rl::FeedforwardWeights weights;
  weights.regressors = config.regressors;
  std::string sha256;
  writeWeightsChecked(output, weights, &outputs, &sha256);
  result.outputs.push_back({"weights", output, sha256});
  result.output_trial = 0;
  outputs.commit();
  return result;
}

// ilc: the next offset by learn::ilcUpdate from the trial's log.
inline LearnResult runIlc(const IlcInputs& in) {
  using learn_detail::quotedPath;
  std::vector<std::pair<std::string, fs::path>> paths = {
      {"--output", in.output}};
  if (in.report) paths.push_back({"--report", *in.report});
  NewOutputs outputs(paths);
  LearnResult result;
  result.command = "ilc";
  const LearnConfig config = learn_detail::readConfig(in.config, &result);
  const FollowReference reference = loadFollowReference(in.follow_config);
  appendReferenceInputs(reference, &result.inputs);
  std::string offset_sha256;
  const rl::CommandOffset prev =
      readOffsetForRun(in.offset, reference, config.limits, &offset_sha256);
  result.inputs.push_back({"offset", in.offset, offset_sha256});
  learn_detail::requireNextTrial(prev.trial, offsetLabel(in.offset));
  std::string log_sha256;
  const std::vector<LogRow> rows =
      readStable(in.log, logLabel(in.log), &log_sha256,
                 [&] { return readFollowLog(in.log); });
  result.inputs.push_back({"log", in.log, log_sha256});
  const TrialRecord record = trialRecord(
      rows, reference, prev, logLabel(in.log), offsetLabel(in.offset));
  bool enabled = false;
  for (int j = 0; j < kDof; ++j) {
    enabled = enabled || config.ilc.phi[j] != 0.0 || config.ilc.psi_s[j] != 0.0;
  }
  if (!enabled) {
    throw std::runtime_error(
        "learn config " + quotedPath(in.config) +
        ": ilc.phi and ilc.psi_s are 0 on every joint, so the update would "
        "change nothing; set [ilc] phi (and psi_s)");
  }

  rl::IlcReport report;
  rl::CommandOffset next =
      learn_detail::offsetOnGrid(reference, "ilc", prev.trial + 1);
  try {
    next.delta_rad = rl::ilcUpdate(prev.delta_rad, record.error,
                                   reference.grid.dt_s, config.ilc, report);
  } catch (const std::exception& error) {
    throw std::runtime_error("ilc: the update refuses " + logLabel(in.log) +
                             " with " + offsetLabel(in.offset) + " under " +
                             "learn config " + quotedPath(in.config) + ": " +
                             error.what());
  }
  next.parent_sha256 = offset_sha256;
  next.source_log_sha256 = log_sha256;
  try {
    rl::checkOffsetForRun(next, reference.duration_s, reference.control_dt_s,
                          reference.reference_sha256, config.limits);
  } catch (const std::exception& error) {
    throw std::runtime_error(
        "ilc: the updated offset is refused and nothing was written: " +
        learn_detail::withoutPrefix(error.what(), "command offset: ") +
        "; lower [ilc] cutoff_hz or the gains phi and psi_s in learn "
        "config " +
        quotedPath(in.config) +
        " (the offset is never adjusted to fit the limits)");
  }
  result.measured_trial = prev.trial;
  fillTracking(record, &result);
  fillOffset(next, &result);
  result.ilc = report;
  result.outputs.push_back(
      {"offset", in.output, writeOffsetChecked(in.output, next, &outputs)});
  if (in.report) {
    writeNewText(*in.report, formatReport(result), "--report");
    outputs.created(*in.report);
  }
  outputs.commit();
  return result;
}

// fel: the next weights by learn::felUpdate from the trial's log, and the
// offset they give on the same reference.
inline LearnResult runFel(const FelInputs& in) {
  using learn_detail::exactNumber;
  using learn_detail::quotedPath;
  std::vector<std::pair<std::string, fs::path>> paths = {
      {"--output-weights", in.output_weights}, {"--output", in.output}};
  if (in.report) paths.push_back({"--report", *in.report});
  NewOutputs outputs(paths);
  LearnResult result;
  result.command = "fel";
  const LearnConfig config = learn_detail::readConfig(in.config, &result);
  FollowReference reference = loadFollowReference(in.follow_config);
  appendReferenceInputs(reference, &result.inputs);
  std::string offset_sha256;
  const rl::CommandOffset prev =
      readOffsetForRun(in.offset, reference, config.limits, &offset_sha256);
  result.inputs.push_back({"offset", in.offset, offset_sha256});
  std::string weights_sha256;
  const rl::FeedforwardWeights weights = learn_detail::readWeights(
      in.weights, config, in.config, &weights_sha256, &result);
  learn_detail::requireNextTrial(weights.trial,
                                 "weights " + quotedPath(in.weights));

  // The offset the trial ran with is the synthesis of these weights.
  if (prev.trial != weights.trial) {
    throw std::runtime_error(
        offsetLabel(in.offset) + " is trial " + std::to_string(prev.trial) +
        ", weights " + quotedPath(in.weights) + " are trial " +
        std::to_string(weights.trial) +
        ": the offset was not synthesised from these weights");
  }
  if (weights.trial >= 1) {
    if (prev.method != "fel") {
      throw std::runtime_error(offsetLabel(in.offset) + " has method '" +
                               prev.method +
                               "', not 'fel': it was not synthesised from "
                               "weights");
    }
    if (prev.parent_sha256 != weights_sha256) {
      throw std::runtime_error(
          offsetLabel(in.offset) + " was synthesised from the weights file " +
          "with sha256 " + prev.parent_sha256 + ", not from " +
          quotedPath(in.weights) + " (sha256 " + weights_sha256 + ")");
    }
    if (prev.source_log_sha256 != weights.source_log_sha256) {
      throw std::runtime_error(
          offsetLabel(in.offset) + " names source log " +
          prev.source_log_sha256 + ", weights " + quotedPath(in.weights) +
          " name " + weights.source_log_sha256 +
          ": the offset was not synthesised from these weights");
    }
  }
  const std::vector<rl::RegressorRow> regressors =
      learn_detail::regressorsOf(reference, weights.regressors);
  const std::vector<JointArray> expected =
      rl::synthesizeOffset(regressors, weights);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    for (int j = 0; j < kDof; ++j) {
      const double gap = std::fabs(prev.delta_rad[i][j] - expected[i][j]);
      if (!(gap <= kSynthesisToleranceRad)) {
        throw std::runtime_error(
            offsetLabel(in.offset) + ": sample " + std::to_string(i) +
            ", joint " + std::to_string(j) + ": delta " +
            exactNumber(prev.delta_rad[i][j]) + " differs from the synthesis " +
            exactNumber(expected[i][j]) + " of weights " +
            quotedPath(in.weights) + " on " + reference.label() + " by " +
            learn_detail::shortNumber(gap) +
            " rad (tolerance 1e-9 rad): the logged error would not be the "
            "error under these weights");
      }
    }
  }

  std::string log_sha256;
  const std::vector<LogRow> rows =
      readStable(in.log, logLabel(in.log), &log_sha256,
                 [&] { return readFollowLog(in.log); });
  result.inputs.push_back({"log", in.log, log_sha256});
  const TrialRecord record = trialRecord(
      rows, reference, prev, logLabel(in.log), offsetLabel(in.offset));

  rl::FelReport report;
  rl::FeedforwardWeights next_weights;
  try {
    next_weights = rl::felUpdate(weights, regressors, record.error,
                                 reference.grid.dt_s, config.fel, &report);
  } catch (const std::exception& error) {
    throw std::runtime_error("fel: the update refuses " + logLabel(in.log) +
                             " with weights " + quotedPath(in.weights) +
                             " under learn config " + quotedPath(in.config) +
                             ": " + error.what());
  }
  next_weights.parent_sha256 = weights_sha256;
  next_weights.source_log_sha256 = log_sha256;
  next_weights.reference_sha256 = reference.reference_sha256;
  std::string next_weights_sha256;
  const rl::FeedforwardWeights written = writeWeightsChecked(
      in.output_weights, next_weights, &outputs, &next_weights_sha256);
  result.outputs.push_back({"weights", in.output_weights, next_weights_sha256});

  // A fel offset's parent is the weights file it was synthesised from.
  rl::CommandOffset next =
      learn_detail::offsetOnGrid(reference, "fel", written.trial);
  next.delta_rad = rl::synthesizeOffset(regressors, written);
  next.parent_sha256 = next_weights_sha256;
  next.source_log_sha256 = written.source_log_sha256;
  try {
    rl::checkOffsetForRun(next, reference.duration_s, reference.control_dt_s,
                          reference.reference_sha256, config.limits);
  } catch (const std::exception& error) {
    throw std::runtime_error(
        "fel: the offset of the updated weights is refused, and nothing was "
        "written (the weights output " +
        quotedPath(in.output_weights) + " is removed again): " +
        learn_detail::withoutPrefix(error.what(), "command offset: ") +
        "; lower [fel] gain in learn config " + quotedPath(in.config) +
        " (the offset is never adjusted to fit the limits)");
  }
  result.measured_trial = prev.trial;
  fillTracking(record, &result);
  fillOffset(next, &result);
  result.fel = report;
  result.outputs.push_back(
      {"offset", in.output, writeOffsetChecked(in.output, next, &outputs)});
  if (in.report) {
    writeNewText(*in.report, formatReport(result), "--report");
    outputs.created(*in.report);
  }
  outputs.commit();
  return result;
}

// synth: the offset trained weights give on another reference.
inline LearnResult runSynth(const SynthInputs& in) {
  using learn_detail::quotedPath;
  NewOutputs outputs({{"--output", in.output}});
  LearnResult result;
  result.command = "synth";
  const LearnConfig config = learn_detail::readConfig(in.config, &result);
  std::string weights_sha256;
  const rl::FeedforwardWeights weights = learn_detail::readWeights(
      in.weights, config, in.config, &weights_sha256, &result);
  if (weights.trial < 1) {
    throw std::runtime_error(
        "weights " + quotedPath(in.weights) +
        " are trial 0 (never trained): they give the zero offset; use init "
        "for a trial-0 offset");
  }
  FollowReference reference = loadFollowReference(in.follow_config);
  appendReferenceInputs(reference, &result.inputs);
  const std::vector<rl::RegressorRow> regressors =
      learn_detail::regressorsOf(reference, weights.regressors);
  rl::CommandOffset offset =
      learn_detail::offsetOnGrid(reference, "fel", weights.trial);
  offset.delta_rad = rl::synthesizeOffset(regressors, weights);
  offset.parent_sha256 = weights_sha256;
  offset.source_log_sha256 = weights.source_log_sha256;
  try {
    rl::checkOffsetForRun(offset, reference.duration_s, reference.control_dt_s,
                          reference.reference_sha256, config.limits);
  } catch (const std::exception& error) {
    throw std::runtime_error(
        "synth: the offset weights " + quotedPath(in.weights) + " give on " +
        reference.label() + " is refused and nothing was written: " +
        learn_detail::withoutPrefix(error.what(), "command offset: "));
  }
  fillOffset(offset, &result);
  result.outputs.push_back(
      {"offset", in.output, writeOffsetChecked(in.output, offset, &outputs)});
  outputs.commit();
  return result;
}

// ----------------------------------------------------------------- CLI

enum class LearnCommand { Init, InitWeights, Ilc, Fel, Synth };

inline const char* learnCommandName(LearnCommand command) {
  switch (command) {
    case LearnCommand::Init: return "init";
    case LearnCommand::InitWeights: return "init-weights";
    case LearnCommand::Ilc: return "ilc";
    case LearnCommand::Fel: return "fel";
    case LearnCommand::Synth: return "synth";
  }
  return "unknown";
}

struct LearnCli {
  std::optional<LearnCommand> command;  // empty only with help
  bool help = false;
  std::optional<fs::path> config;
  std::optional<fs::path> follow_config;
  std::optional<fs::path> log;
  std::optional<fs::path> offset;
  std::optional<fs::path> weights;
  std::optional<fs::path> output;
  std::optional<fs::path> output_weights;
  std::optional<fs::path> report;
};

namespace learn_detail {

struct OptionSpec {
  const char* name;
  std::optional<fs::path> LearnCli::* member;
};

inline const std::array<OptionSpec, 8>& optionSpecs() {
  static const std::array<OptionSpec, 8> specs = {{
      {"--config", &LearnCli::config},
      {"--follow-config", &LearnCli::follow_config},
      {"--log", &LearnCli::log},
      {"--offset", &LearnCli::offset},
      {"--weights", &LearnCli::weights},
      {"--output", &LearnCli::output},
      {"--output-weights", &LearnCli::output_weights},
      {"--report", &LearnCli::report},
  }};
  return specs;
}

// Required options of each command, then its optional ones.
inline std::vector<std::string_view> requiredOptions(LearnCommand command) {
  switch (command) {
    case LearnCommand::Init: return {"--follow-config", "--output"};
    case LearnCommand::InitWeights: return {"--config", "--output"};
    case LearnCommand::Ilc:
      return {"--config", "--follow-config", "--log", "--offset", "--output"};
    case LearnCommand::Fel:
      return {"--config",  "--follow-config",  "--log",   "--offset",
              "--weights", "--output-weights", "--output"};
    case LearnCommand::Synth:
      return {"--config", "--weights", "--follow-config", "--output"};
  }
  return {};
}

inline std::vector<std::string_view> optionalOptions(LearnCommand command) {
  if (command == LearnCommand::Ilc || command == LearnCommand::Fel) {
    return {"--report"};
  }
  return {};
}

}  // namespace learn_detail

inline std::string learnUsage(std::optional<LearnCommand> command) {
  const char* general =
      "usage: x7_follow_learn COMMAND [options]   (offline: never opens the "
      "bus)\n"
      "  init          --follow-config F.toml --output OFFSET.csv\n"
      "  init-weights  --config LEARN.toml --output WEIGHTS.toml\n"
      "  ilc           --config LEARN.toml --follow-config F.toml --log "
      "TRIAL.csv\n"
      "                --offset PREV.csv --output NEXT.csv [--report "
      "REPORT.toml]\n"
      "  fel           --config LEARN.toml --follow-config F.toml --log "
      "TRIAL.csv\n"
      "                --offset PREV.csv --weights PREVW.toml\n"
      "                --output-weights NEXTW.toml --output NEXT.csv "
      "[--report REPORT.toml]\n"
      "  synth         --config LEARN.toml --weights W.toml --follow-config "
      "OTHER.toml\n"
      "                --output OFFSET.csv\n"
      "  x7_follow_learn COMMAND --help describes one command. Outputs must "
      "not exist.\n";
  if (!command) return general;
  switch (*command) {
    case LearnCommand::Init:
      return "usage: x7_follow_learn init --follow-config F.toml --output "
             "OFFSET.csv\n"
             "  writes the zero offset (trial 0) on the tracking grid of the "
             "follow config's\n"
             "  reference and control rate\n";
    case LearnCommand::InitWeights:
      return "usage: x7_follow_learn init-weights --config LEARN.toml "
             "--output WEIGHTS.toml\n"
             "  writes zero feedforward weights (trial 0) with the learn "
             "config's\n"
             "  [fel.regressors] options\n";
    case LearnCommand::Ilc:
      return "usage: x7_follow_learn ilc --config LEARN.toml --follow-config "
             "F.toml\n"
             "         --log TRIAL.csv --offset PREV.csv --output NEXT.csv "
             "[--report REPORT.toml]\n"
             "  checks that TRIAL.csv is a completed run of the follow "
             "config's reference\n"
             "  with offset PREV.csv, then writes the next offset by "
             "iterative learning\n"
             "  control (method ilc, trial + 1)\n";
    case LearnCommand::Fel:
      return "usage: x7_follow_learn fel --config LEARN.toml --follow-config "
             "F.toml\n"
             "         --log TRIAL.csv --offset PREV.csv --weights PREVW.toml\n"
             "         --output-weights NEXTW.toml --output NEXT.csv "
             "[--report REPORT.toml]\n"
             "  checks that PREV.csv is the synthesis of PREVW.toml on the "
             "reference and that\n"
             "  TRIAL.csv is a completed run with it, then writes the next "
             "weights by\n"
             "  feedback-error learning and the offset they give (method "
             "fel)\n";
    case LearnCommand::Synth:
      return "usage: x7_follow_learn synth --config LEARN.toml --weights "
             "W.toml\n"
             "         --follow-config OTHER.toml --output OFFSET.csv\n"
             "  writes the offset trained weights give on the reference of "
             "OTHER.toml\n";
  }
  return general;
}

// COMMAND first, then its options, each once with a value; --help after a
// command describes it, alone it describes all.
inline LearnCli parseLearnCli(int argc, char* argv[]) {
  LearnCli cli;
  if (argc < 2) {
    throw std::runtime_error(
        "a command is required: init, init-weights, ilc, fel or synth "
        "(see --help)");
  }
  const std::string first = argv[1];
  if (first == "--help" || first == "-h") {
    if (argc > 2) {
      throw std::runtime_error("--help takes no further arguments");
    }
    cli.help = true;
    return cli;
  }
  for (const LearnCommand command :
       {LearnCommand::Init, LearnCommand::InitWeights, LearnCommand::Ilc,
        LearnCommand::Fel, LearnCommand::Synth}) {
    if (first == learnCommandName(command)) cli.command = command;
  }
  if (!cli.command) {
    throw std::runtime_error("unknown command '" + first +
                             "': init, init-weights, ilc, fel or synth");
  }
  const std::string name = learnCommandName(*cli.command);
  const auto required = learn_detail::requiredOptions(*cli.command);
  const auto optional = learn_detail::optionalOptions(*cli.command);
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      cli.help = true;
      continue;
    }
    const learn_detail::OptionSpec* spec = nullptr;
    for (const auto& candidate : learn_detail::optionSpecs()) {
      if (arg == candidate.name) spec = &candidate;
    }
    if (spec == nullptr) throw std::runtime_error("unknown option: " + arg);
    const auto allowed = [&arg](const std::vector<std::string_view>& list) {
      return std::find(list.begin(), list.end(), arg) != list.end();
    };
    if (!allowed(required) && !allowed(optional)) {
      throw std::runtime_error(arg + " is not an option of '" + name + "'");
    }
    if (i + 1 >= argc || argv[i + 1][0] == '-' || argv[i + 1][0] == '\0') {
      throw std::runtime_error(arg + " requires a value");
    }
    auto& slot = cli.*(spec->member);
    if (slot) throw std::runtime_error(arg + " is given twice");
    slot = fs::path(argv[++i]);
  }
  if (cli.help) return cli;
  for (const std::string_view option : required) {
    for (const auto& spec : learn_detail::optionSpecs()) {
      if (option == spec.name && !(cli.*(spec.member))) {
        throw std::runtime_error(std::string(option) + " is required for '" +
                                 name + "'");
      }
    }
  }
  return cli;
}

inline LearnResult runLearnCommand(const LearnCli& cli) {
  switch (*cli.command) {
    case LearnCommand::Init: return runInit(*cli.follow_config, *cli.output);
    case LearnCommand::InitWeights:
      return runInitWeights(*cli.config, *cli.output);
    case LearnCommand::Ilc:
      return runIlc({*cli.config, *cli.follow_config, *cli.log, *cli.offset,
                     *cli.output, cli.report});
    case LearnCommand::Fel:
      return runFel({*cli.config, *cli.follow_config, *cli.log, *cli.offset,
                     *cli.weights, *cli.output_weights, *cli.output,
                     cli.report});
    case LearnCommand::Synth:
      return runSynth(
          {*cli.config, *cli.weights, *cli.follow_config, *cli.output});
  }
  throw std::logic_error("unknown command");
}

}  // namespace x7::learn
