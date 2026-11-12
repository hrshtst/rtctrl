#pragma once

#include <toml++/toml.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "rtctrl/arm/types.hpp"
#include "rtctrl/learn/command_offset.hpp"
#include "rtctrl/model/ptp_planner.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace x7::follow {

namespace fs = std::filesystem;
namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

struct HomeConfig {
  model::PtpProfile profile = model::PtpProfile::MinimumJerk;
  std::optional<double> motion_time;
  double velocity_limit = 0.25;
  double trapezoid_acceleration_fraction = 0.2;
  bool strict = true;
  double tolerance_rad = 0.01;
  int correction_retries = 5;
  double settle_time_s = 0.3;
};

// Set at run time (never from TOML) for references built from the
// measured posture (follow/posture_reference.hpp). Such a run never homes
// or corrects: it has no Home or HomeCorrection phase, and every command
// is the reference itself, or after a refusal or abort the measured
// posture.
struct PostureRunConfig {
  // Refused before any command when the arm's start is farther than this
  // from the reference's first frame on any joint.
  double start_tolerance_rad = 0.0;
  // Tracking aborts, holding where the arm is, once any joint is this far
  // from the reference.
  double abort_error_rad = 0.0;
};

struct SimulationConfig {
  enum class InitialPosture { Model, Zeros, Explicit };
  InitialPosture initial_posture = InitialPosture::Model;
  std::array<double, model::kCanonicalDof> initial_joints{};
  double integration_step_s = 1e-4;
  double position_kp = 1000.0;
  double position_kd = 5.0;
  // Prescribed link-force disturbance (rtctrl-x7-link-force spec);
  // simulation-only, hardware frontends ignore it.
  std::optional<fs::path> link_force_path;
  // Servo-like simulation (all optional; absent = the ideal servo above).
  // position_kp/kd given as an array of 8: per canonical joint, replacing
  // the scalar (which then keeps its default and is unused).
  using JointValues = std::array<double, model::kCanonicalDof>;
  std::optional<JointValues> position_kp_joints;
  std::optional<JointValues> position_kd_joints;
  // Joint friction, a number (every joint) or 8 values; absent = off.
  std::optional<JointValues> coulomb_friction_nm;
  std::optional<JointValues> viscous_friction_nm_s;
  double friction_v0_rad_s = 0.05;
  // Commands reach the simulated servo this many cycles after acceptance.
  int command_delay_cycles = 0;
  // Feedback position rounded to the encoder LSB, velocity a servo-style
  // estimate (common/lagged_arm.hpp).
  bool quantize_encoder = false;

  // Whether any servo-like key is set.
  bool servoLike() const {
    return position_kp_joints || position_kd_joints || coulomb_friction_nm ||
           viscous_friction_nm_s || command_delay_cycles != 0 ||
           quantize_encoder;
  }
};

struct SafetyConfig {
  double warning_error_rad = 0.2;
  double sustained_abort_error_rad = 0.5;
  double sustained_abort_time_s = 0.2;
  double immediate_abort_error_rad = 1.0;
};

struct FinalizationConfig {
  double wait_time_s = 0.0;
  double operator_timeout_s = 60.0;
  double simulation_hold_time_s = 2.0;
};

struct OutputConfig {
  fs::path simulation_motion = "follow_sim.zvs";
  fs::path simulation_log = "follow_sim.csv";
  fs::path hardware_log = "follow_hw.csv";
};

// A learned command offset (rtctrl/learn/command_offset.hpp), computed
// offline between trials: tracking commands q_d + delta. The follower only
// reads and checks it; it never computes or changes one. The limits can
// only be lowered from learn::OffsetLimits' defaults, the rate limit not
// below learn::kMinOffsetRateLimitRadS.
struct LearningConfig {
  std::optional<fs::path> offset_path;
  double max_offset_rad = rtctrl::learn::OffsetLimits{}.max_abs_rad;
  double max_offset_rate_rad_s = rtctrl::learn::OffsetLimits{}.max_rate_rad_s;
};

struct Config {
  fs::path source_path;
  fs::path model_path;
  fs::path reference_path;
  fs::path hardware_config_path;
  std::optional<fs::path> motor_parameters_path;
  double control_rate_hz = 100.0;
  arm::ControlMode mode = arm::ControlMode::Position;
  std::array<double, model::kCanonicalDof> effort_limit_nm{};
  bool effort_limit_set = false;
  HomeConfig home;
  std::optional<PostureRunConfig> posture;
  model::ZvsTrajectoryOptions reference;
  SimulationConfig simulation;
  SafetyConfig safety;
  FinalizationConfig finalization;
  OutputConfig output;
  LearningConfig learning;
};

struct Cli {
  fs::path config_path;
  std::optional<fs::path> reference_path;
  std::optional<fs::path> motor_parameters_path;
  std::optional<fs::path> log_path;
  std::optional<fs::path> timing_log_path;
  // References built from the measured posture (follow/posture_reference.hpp).
  std::optional<std::string> hold_current;
  std::optional<std::string> sweep_current;
  std::optional<fs::path> motion_path;
  std::optional<fs::path> bundle_path;
  std::optional<fs::path> offset_path;
  std::optional<std::string> port;
  std::optional<arm::ControlMode> mode;
  std::optional<double> effort_limit_nm;
  std::optional<model::ReferenceFilter> filter;
  std::optional<model::ReferenceInterpolation> interpolation;
  bool check = false;
  bool help = false;
};

inline void rejectUnknown(const toml::table& table,
                          std::initializer_list<std::string_view> allowed,
                          const std::string& name) {
  for (const auto& [key, node] : table) {
    (void)node;
    if (std::find(allowed.begin(), allowed.end(), key.str()) == allowed.end()) {
      throw std::runtime_error("follow config: unknown key '" +
                               std::string(key.str()) + "' in " + name);
    }
  }
}

inline const toml::table& optionalTable(const toml::table& root,
                                        const char* key) {
  static const toml::table empty;
  const auto node = root[key];
  if (!node) return empty;
  const auto* table = node.as_table();
  if (table == nullptr) {
    throw std::runtime_error(std::string("follow config: ") + key +
                             " must be a table");
  }
  return *table;
}

inline double number(const toml::table& table, const char* key,
                     double fallback, const char* table_name) {
  const auto node = table[key];
  if (!node) return fallback;
  const auto value = node.value<double>();
  if (!value || !std::isfinite(*value)) {
    throw std::runtime_error(std::string("follow config: ") + table_name +
                             "." + key + " must be finite");
  }
  return *value;
}

inline int integer(const toml::table& table, const char* key, int fallback,
                   const char* table_name) {
  const auto node = table[key];
  if (!node) return fallback;
  const auto value = node.value<std::int64_t>();
  if (!value || *value < std::numeric_limits<int>::min() ||
      *value > std::numeric_limits<int>::max()) {
    throw std::runtime_error(std::string("follow config: ") + table_name +
                             "." + key + " must be an integer");
  }
  return static_cast<int>(*value);
}

inline bool boolean(const toml::table& table, const char* key, bool fallback,
                    const char* table_name) {
  const auto node = table[key];
  if (!node) return fallback;
  const auto value = node.value<bool>();
  if (!value) {
    throw std::runtime_error(std::string("follow config: ") + table_name +
                             "." + key + " must be boolean");
  }
  return *value;
}

inline fs::path inputPath(const toml::table& root, const char* key,
                          const fs::path& config_path, bool required) {
  const auto value = root[key].value<std::string>();
  if (!value || value->empty()) {
    if (!root[key] && !required) return {};
    throw std::runtime_error(std::string("follow config: ") + key +
                             " must be a non-empty path");
  }
  fs::path path(*value);
  if (path.is_relative()) path = fs::absolute(config_path).parent_path() / path;
  return path.lexically_normal();
}

inline fs::path outputPath(const toml::table& table, const char* key,
                           fs::path fallback, const char* extension) {
  const auto node = table[key];
  if (node) {
    const auto value = node.value<std::string>();
    if (!value || value->empty()) {
      throw std::runtime_error(std::string("follow config: output.") + key +
                               " must be a non-empty path");
    }
    fallback = *value;
  }
  if (fallback.extension() != extension) fallback += extension;
  return fallback;
}

inline arm::ControlMode parseMode(std::string_view value) {
  if (value == "position") return arm::ControlMode::Position;
  if (value == "current-based-position") {
    return arm::ControlMode::CurrentBasedPosition;
  }
  throw std::runtime_error(
      "follow config: control.mode must be position or "
      "current-based-position; velocity mode requires a host-side "
      "position loop and is not supported by x7_follow");
}

inline model::ReferenceFilter parseFilter(std::string_view value) {
  if (value == "none") return model::ReferenceFilter::None;
  if (value == "low-pass") return model::ReferenceFilter::LowPass;
  if (value == "moving-average") return model::ReferenceFilter::MovingAverage;
  if (value == "savitzky-golay") return model::ReferenceFilter::SavitzkyGolay;
  throw std::runtime_error(
      "follow config: filter must be none, low-pass, moving-average, or "
      "savitzky-golay");
}

inline model::ReferenceInterpolation parseInterpolation(
    std::string_view value) {
  if (value == "linear") return model::ReferenceInterpolation::Linear;
  if (value == "shape-preserving-cubic") {
    return model::ReferenceInterpolation::ShapePreservingCubic;
  }
  throw std::runtime_error(
      "follow config: interpolation must be linear or "
      "shape-preserving-cubic");
}

// The config names parseFilter and parseInterpolation read. Throws
// std::logic_error for a value outside the enumeration.
inline const char* referenceFilterName(model::ReferenceFilter filter) {
  switch (filter) {
    case model::ReferenceFilter::None: return "none";
    case model::ReferenceFilter::LowPass: return "low-pass";
    case model::ReferenceFilter::MovingAverage: return "moving-average";
    case model::ReferenceFilter::SavitzkyGolay: return "savitzky-golay";
  }
  throw std::logic_error("follow config: unknown reference filter");
}

inline const char* referenceInterpolationName(
    model::ReferenceInterpolation interpolation) {
  switch (interpolation) {
    case model::ReferenceInterpolation::Linear: return "linear";
    case model::ReferenceInterpolation::ShapePreservingCubic:
      return "shape-preserving-cubic";
  }
  throw std::logic_error("follow config: unknown reference interpolation");
}

inline model::PtpProfile parseProfile(std::string_view value) {
  if (value == "linear") return model::PtpProfile::Linear;
  if (value == "trapezoidal") return model::PtpProfile::Trapezoidal;
  if (value == "minimum-jerk") return model::PtpProfile::MinimumJerk;
  throw std::runtime_error(
      "follow config: home.profile must be linear, trapezoidal, or "
      "minimum-jerk");
}

inline Config loadConfig(const fs::path& path) {
  toml::table root;
  try {
    root = toml::parse_file(path.string());
  } catch (const toml::parse_error& error) {
    throw std::runtime_error("follow config: cannot parse '" + path.string() +
                             "': " + std::string(error.description()));
  }
  rejectUnknown(root,
                {"format", "version", "model", "reference",
                 "hardware_config", "motor_parameters", "control", "home",
                 "reference_processing", "current_based_position",
                 "simulation", "safety", "finalization", "output",
                 "learning"},
                "root");
  if (root["format"].value_or(std::string{}) != "rtctrl-x7-follow" ||
      root["version"].value_or<std::int64_t>(0) != 1) {
    throw std::runtime_error(
        "follow config: format must be rtctrl-x7-follow version 1");
  }
  Config config;
  config.source_path = fs::absolute(path).lexically_normal();
  config.model_path = inputPath(root, "model", path, true);
  config.reference_path = inputPath(root, "reference", path, true);
  config.hardware_config_path = inputPath(root, "hardware_config", path, true);
  const auto motor_path = inputPath(root, "motor_parameters", path, false);
  if (!motor_path.empty()) config.motor_parameters_path = motor_path;

  const auto& control = optionalTable(root, "control");
  rejectUnknown(control, {"rate_hz", "mode"}, "control");
  config.control_rate_hz =
      number(control, "rate_hz", config.control_rate_hz, "control");
  if (const auto value = control["mode"].value<std::string>()) {
    config.mode = parseMode(*value);
  } else if (control["mode"]) {
    throw std::runtime_error("follow config: control.mode must be a string");
  }
  if (config.control_rate_hz < 20.0 || config.control_rate_hz > 200.0) {
    throw std::runtime_error(
        "follow config: control.rate_hz must be in [20, 200]");
  }

  const auto& home = optionalTable(root, "home");
  rejectUnknown(home,
                {"profile", "motion_time", "velocity_limit",
                 "trapezoid_acceleration_fraction", "strict",
                 "tolerance_rad", "correction_retries", "settle_time_s"},
                "home");
  if (const auto value = home["profile"].value<std::string>()) {
    config.home.profile = parseProfile(*value);
  } else if (home["profile"]) {
    throw std::runtime_error("follow config: home.profile must be a string");
  }
  if (home["motion_time"]) {
    config.home.motion_time = number(home, "motion_time", 0.0, "home");
  }
  config.home.velocity_limit =
      number(home, "velocity_limit", config.home.velocity_limit, "home");
  config.home.trapezoid_acceleration_fraction = number(
      home, "trapezoid_acceleration_fraction",
      config.home.trapezoid_acceleration_fraction, "home");
  config.home.strict = boolean(home, "strict", config.home.strict, "home");
  config.home.tolerance_rad =
      number(home, "tolerance_rad", config.home.tolerance_rad, "home");
  config.home.correction_retries = integer(
      home, "correction_retries", config.home.correction_retries, "home");
  config.home.settle_time_s =
      number(home, "settle_time_s", config.home.settle_time_s, "home");
  if (config.home.velocity_limit <= 0.0 ||
      (config.home.motion_time && *config.home.motion_time <= 0.0) ||
      config.home.tolerance_rad <= 0.0 ||
      config.home.correction_retries < 0 || config.home.settle_time_s < 0.0 ||
      config.home.trapezoid_acceleration_fraction <= 0.0 ||
      config.home.trapezoid_acceleration_fraction > 0.5) {
    throw std::runtime_error("follow config: invalid home motion or gate value");
  }

  const auto& processing = optionalTable(root, "reference_processing");
  rejectUnknown(processing,
                {"filter", "interpolation", "low_pass_cutoff_hz",
                 "filter_window", "savitzky_golay_order"},
                "reference_processing");
  if (const auto value = processing["interpolation"].value<std::string>()) {
    config.reference.interpolation = parseInterpolation(*value);
  }
  if (const auto value = processing["filter"].value<std::string>()) {
    config.reference.filter = parseFilter(*value);
  }
  config.reference.low_pass_cutoff_hz = number(
      processing, "low_pass_cutoff_hz", config.reference.low_pass_cutoff_hz,
      "reference_processing");
  config.reference.filter_window = integer(
      processing, "filter_window", config.reference.filter_window,
      "reference_processing");
  config.reference.savitzky_golay_order = integer(
      processing, "savitzky_golay_order",
      config.reference.savitzky_golay_order, "reference_processing");

  const auto& cbp = optionalTable(root, "current_based_position");
  rejectUnknown(cbp, {"effort_limit_nm"}, "current_based_position");
  if (const auto* limits = cbp["effort_limit_nm"].as_array()) {
    if (limits->size() != 1 && limits->size() != model::kCanonicalDof) {
      throw std::runtime_error(
          "follow config: effort_limit_nm needs one or eight values");
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const auto value = (*limits)[limits->size() == 1 ? 0 : i].value<double>();
      if (!value || !std::isfinite(*value) || *value <= 0.0) {
        throw std::runtime_error(
            "follow config: effort limits must be finite and positive");
      }
      config.effort_limit_nm[i] = *value;
    }
    config.effort_limit_set = true;
  } else if (cbp["effort_limit_nm"]) {
    throw std::runtime_error(
        "follow config: effort_limit_nm must be an array");
  }
  if (config.mode == arm::ControlMode::CurrentBasedPosition &&
      !config.effort_limit_set) {
    throw std::runtime_error(
        "follow config: current-based position requires effort_limit_nm");
  }

  const auto& simulation = optionalTable(root, "simulation");
  rejectUnknown(simulation,
                {"initial_posture", "initial_joints", "integration_step_s",
                 "position_kp", "position_kd", "velocity_kp", "link_force",
                 "coulomb_friction_nm", "viscous_friction_nm_s",
                 "friction_v0_rad_s", "command_delay_cycles",
                 "quantize_encoder"},
                "simulation");
  // The servo-like keys take only their own TOML types (no conversion
  // between booleans and numbers).
  const auto fail = [](const char* key, const char* problem) {
    throw std::runtime_error(std::string("follow config: simulation.") + key +
                             problem);
  };
  // A number (every joint) or an array of eight numbers (canonical order).
  const auto joint_values = [&simulation, &fail](const char* key) {
    SimulationConfig::JointValues values{};
    const auto node = simulation[key];
    const auto* list = node.as_array();
    if (list == nullptr && !node.is_number()) {
      fail(key, " must be a number or eight numbers");
    }
    if (list != nullptr && list->size() != model::kCanonicalDof) {
      fail(key, " needs one number or eight values");
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const toml::node& item = list != nullptr ? *list->get(i) : *node.node();
      if (!item.is_number()) fail(key, " must be a number or eight numbers");
      const double value = *item.value<double>();
      if (!std::isfinite(value)) fail(key, " must be finite");
      values[i] = value;
    }
    return values;
  };
  if (simulation["friction_v0_rad_s"] &&
      !simulation["friction_v0_rad_s"].is_number()) {
    fail("friction_v0_rad_s", " must be finite");
  }
  if (simulation["command_delay_cycles"] &&
      !simulation["command_delay_cycles"].is_integer()) {
    fail("command_delay_cycles", " must be an integer");
  }
  if (simulation["quantize_encoder"] &&
      !simulation["quantize_encoder"].is_boolean()) {
    fail("quantize_encoder", " must be boolean");
  }
  const auto link_force = inputPath(simulation, "link_force", path, false);
  if (!link_force.empty()) config.simulation.link_force_path = link_force;
  if (const auto value = simulation["initial_posture"].value<std::string>()) {
    if (*value == "model") {
      config.simulation.initial_posture = SimulationConfig::InitialPosture::Model;
    } else if (*value == "zeros") {
      config.simulation.initial_posture = SimulationConfig::InitialPosture::Zeros;
    } else {
      throw std::runtime_error(
          "follow config: initial_posture must be model or zeros");
    }
  }
  if (const auto* initial = simulation["initial_joints"].as_array()) {
    if (initial->size() != model::kCanonicalDof) {
      throw std::runtime_error(
          "follow config: initial_joints needs eight values");
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const auto value = (*initial)[i].value<double>();
      if (!value || !std::isfinite(*value)) {
        throw std::runtime_error(
            "follow config: initial_joints must be finite");
      }
      config.simulation.initial_joints[i] = *value;
    }
    config.simulation.initial_posture = SimulationConfig::InitialPosture::Explicit;
  }
  config.simulation.integration_step_s = number(
      simulation, "integration_step_s", config.simulation.integration_step_s,
      "simulation");
  if (simulation["position_kp"].as_array() != nullptr) {
    config.simulation.position_kp_joints = joint_values("position_kp");
  } else {
    config.simulation.position_kp = number(
        simulation, "position_kp", config.simulation.position_kp, "simulation");
  }
  if (simulation["position_kd"].as_array() != nullptr) {
    config.simulation.position_kd_joints = joint_values("position_kd");
  } else {
    config.simulation.position_kd = number(
        simulation, "position_kd", config.simulation.position_kd, "simulation");
  }
  if (simulation["coulomb_friction_nm"]) {
    config.simulation.coulomb_friction_nm = joint_values("coulomb_friction_nm");
  }
  if (simulation["viscous_friction_nm_s"]) {
    config.simulation.viscous_friction_nm_s =
        joint_values("viscous_friction_nm_s");
  }
  config.simulation.friction_v0_rad_s =
      number(simulation, "friction_v0_rad_s",
             config.simulation.friction_v0_rad_s, "simulation");
  config.simulation.command_delay_cycles =
      integer(simulation, "command_delay_cycles",
              config.simulation.command_delay_cycles, "simulation");
  config.simulation.quantize_encoder =
      boolean(simulation, "quantize_encoder",
              config.simulation.quantize_encoder, "simulation");
  // Schema-v1 archives may contain this former velocity-adapter gain. Keep
  // accepting and validating it so position/CBP bundles remain replayable.
  const double legacy_velocity_kp =
      number(simulation, "velocity_kp", 5.0, "simulation");
  if (config.simulation.integration_step_s <= 0.0 ||
      config.simulation.position_kp <= 0.0 ||
      config.simulation.position_kd < 0.0 ||
      legacy_velocity_kp <= 0.0) {
    throw std::runtime_error("follow config: invalid simulation gain or step");
  }
  const auto every_joint =
      [](const std::optional<SimulationConfig::JointValues>& values,
         bool positive) {
        return !values ||
               std::all_of(values->begin(), values->end(),
                           [positive](double v) {
                             return positive ? v > 0.0 : v >= 0.0;
                           });
      };
  if (!every_joint(config.simulation.position_kp_joints, true) ||
      !every_joint(config.simulation.position_kd_joints, false)) {
    throw std::runtime_error(
        "follow config: simulation.position_kp must be positive and "
        "position_kd non-negative on every joint");
  }
  if (!every_joint(config.simulation.coulomb_friction_nm, false) ||
      !every_joint(config.simulation.viscous_friction_nm_s, false) ||
      config.simulation.friction_v0_rad_s <= 0.0) {
    throw std::runtime_error(
        "follow config: simulation friction must be non-negative with a "
        "positive friction_v0_rad_s");
  }
  if (config.simulation.command_delay_cycles < 0) {
    throw std::runtime_error(
        "follow config: simulation.command_delay_cycles must be "
        "non-negative");
  }

  const auto& safety = optionalTable(root, "safety");
  rejectUnknown(safety,
                {"warning_error_rad", "sustained_abort_error_rad",
                 "sustained_abort_time_s", "immediate_abort_error_rad"},
                "safety");
  config.safety.warning_error_rad = number(
      safety, "warning_error_rad", config.safety.warning_error_rad, "safety");
  config.safety.sustained_abort_error_rad = number(
      safety, "sustained_abort_error_rad",
      config.safety.sustained_abort_error_rad, "safety");
  config.safety.sustained_abort_time_s = number(
      safety, "sustained_abort_time_s", config.safety.sustained_abort_time_s,
      "safety");
  config.safety.immediate_abort_error_rad = number(
      safety, "immediate_abort_error_rad",
      config.safety.immediate_abort_error_rad, "safety");
  if (config.safety.warning_error_rad <= 0.0 ||
      config.safety.sustained_abort_error_rad <
          config.safety.warning_error_rad ||
      config.safety.immediate_abort_error_rad <
          config.safety.sustained_abort_error_rad ||
      config.safety.sustained_abort_time_s <= 0.0) {
    throw std::runtime_error("follow config: invalid safety threshold ordering");
  }

  const auto& finalization = optionalTable(root, "finalization");
  rejectUnknown(finalization,
                {"wait_time_s", "operator_timeout_s",
                 "simulation_hold_time_s"},
                "finalization");
  config.finalization.wait_time_s = number(
      finalization, "wait_time_s", config.finalization.wait_time_s,
      "finalization");
  config.finalization.operator_timeout_s = number(
      finalization, "operator_timeout_s",
      config.finalization.operator_timeout_s, "finalization");
  config.finalization.simulation_hold_time_s = number(
      finalization, "simulation_hold_time_s",
      config.finalization.simulation_hold_time_s, "finalization");
  if (config.finalization.wait_time_s < 0.0 ||
      config.finalization.operator_timeout_s <= 0.0 ||
      config.finalization.simulation_hold_time_s < 0.0) {
    throw std::runtime_error("follow config: invalid finalization time");
  }

  const auto& output = optionalTable(root, "output");
  rejectUnknown(output, {"simulation_motion", "simulation_log", "hardware_log"},
                "output");
  config.output.simulation_motion = outputPath(
      output, "simulation_motion", config.output.simulation_motion, ".zvs");
  config.output.simulation_log = outputPath(
      output, "simulation_log", config.output.simulation_log, ".csv");
  config.output.hardware_log = outputPath(
      output, "hardware_log", config.output.hardware_log, ".csv");

  const auto& learning = optionalTable(root, "learning");
  rejectUnknown(learning, {"offset", "max_offset_rad", "max_offset_rate_rad_s"},
                "learning");
  const auto offset = inputPath(learning, "offset", path, false);
  if (!offset.empty()) config.learning.offset_path = offset;
  config.learning.max_offset_rad = number(
      learning, "max_offset_rad", config.learning.max_offset_rad, "learning");
  config.learning.max_offset_rate_rad_s =
      number(learning, "max_offset_rate_rad_s",
             config.learning.max_offset_rate_rad_s, "learning");
  const rtctrl::learn::OffsetLimits ceiling;
  const auto requireLowered = [](const char* key, double value, double bound,
                                 const char* unit) {
    if (value > 0.0 && value <= bound) return;
    char text[160];
    std::snprintf(text, sizeof text,
                  "follow config: learning.%s must be > 0 and at most %g %s "
                  "(the limit can only be lowered)",
                  key, bound, unit);
    throw std::runtime_error(text);
  };
  requireLowered("max_offset_rad", config.learning.max_offset_rad,
                 ceiling.max_abs_rad, "rad");
  requireLowered("max_offset_rate_rad_s",
                 config.learning.max_offset_rate_rad_s, ceiling.max_rate_rad_s,
                 "rad/s");
  if (config.learning.max_offset_rate_rad_s <
      rtctrl::learn::kMinOffsetRateLimitRadS) {
    char text[200];
    std::snprintf(text, sizeof text,
                  "follow config: learning.max_offset_rate_rad_s must be at "
                  "least %g rad/s (the offset ramp runs at this rate)",
                  rtctrl::learn::kMinOffsetRateLimitRadS);
    throw std::runtime_error(text);
  }
  return config;
}

inline double parseCliNumber(const std::string& option, const char* value) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0' || !std::isfinite(parsed)) {
    throw std::runtime_error(option + " requires a finite number");
  }
  return parsed;
}

inline void applyOverrides(const Cli& cli, Config* config) {
  if (cli.reference_path) {
    config->reference_path =
        fs::absolute(*cli.reference_path).lexically_normal();
  }
  if (cli.motor_parameters_path) {
    config->motor_parameters_path =
        fs::absolute(*cli.motor_parameters_path).lexically_normal();
  }
  if (cli.offset_path) {
    config->learning.offset_path =
        fs::absolute(*cli.offset_path).lexically_normal();
  }
  if (cli.mode) config->mode = *cli.mode;
  if (cli.effort_limit_nm) {
    if (*cli.effort_limit_nm <= 0.0) {
      throw std::runtime_error("--effort-limit-nm must be positive");
    }
    config->effort_limit_nm.fill(*cli.effort_limit_nm);
    config->effort_limit_set = true;
  }
  if (cli.filter) config->reference.filter = *cli.filter;
  if (cli.interpolation) {
    config->reference.interpolation = *cli.interpolation;
  }
  if (config->mode == arm::ControlMode::CurrentBasedPosition &&
      !config->effort_limit_set) {
    throw std::runtime_error(
        "follow config: current-based position requires effort_limit_nm");
  }
}

inline Cli parseCli(int argc, char* argv[], bool simulation) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      cli.help = true;
    } else if (arg == "--check") {
      cli.check = true;
    } else if (arg == "--config" || arg == "--reference" ||
               arg == "--motor-parameters" || arg == "--mode" ||
               arg == "--effort-limit-nm" || arg == "--filter" ||
               arg == "--interpolation" || arg == "--log" ||
               arg == "--timing-log" || arg == "--bundle" ||
               arg == "--port" || arg == "--motion" ||
               arg == "--hold-current" || arg == "--sweep-current" ||
               arg == "--offset") {
      if (i + 1 >= argc || argv[i + 1][0] == '-') {
        throw std::runtime_error(arg + " requires a value");
      }
      const fs::path value = argv[++i];
      if (arg == "--config") cli.config_path = value;
      else if (arg == "--reference") cli.reference_path = value;
      else if (arg == "--motor-parameters")
        cli.motor_parameters_path = value;
      else if (arg == "--mode") cli.mode = parseMode(value.string());
      else if (arg == "--effort-limit-nm")
        cli.effort_limit_nm = parseCliNumber(arg, value.c_str());
      else if (arg == "--filter") cli.filter = parseFilter(value.string());
      else if (arg == "--interpolation")
        cli.interpolation = parseInterpolation(value.string());
      else if (arg == "--log") cli.log_path = value;
      else if (arg == "--timing-log") cli.timing_log_path = value;
      else if (arg == "--hold-current") cli.hold_current = value.string();
      else if (arg == "--sweep-current") cli.sweep_current = value.string();
      else if (arg == "--bundle") cli.bundle_path = value;
      else if (arg == "--offset") cli.offset_path = value;
      else if (arg == "--port") cli.port = value.string();
      else cli.motion_path = value;
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (!cli.help && cli.config_path.empty()) {
    throw std::runtime_error("--config is required");
  }
  if (!simulation && cli.motion_path) {
    throw std::runtime_error("--motion is simulation-only");
  }
  if (simulation && cli.timing_log_path) {
    throw std::runtime_error("--timing-log is hardware-only");
  }
  if (cli.bundle_path && cli.timing_log_path) {
    throw std::runtime_error("--timing-log cannot be used with --bundle");
  }
  if (cli.hold_current || cli.sweep_current) {
    if (simulation) {
      throw std::runtime_error(
          "--hold-current and --sweep-current are hardware-only");
    }
    if (cli.hold_current && cli.sweep_current) {
      throw std::runtime_error(
          "--hold-current and --sweep-current are mutually exclusive");
    }
    if (cli.reference_path || cli.check || cli.bundle_path) {
      throw std::runtime_error(
          "a reference built from the measured posture cannot be combined "
          "with --reference, --check, or --bundle");
    }
  }
  if (cli.check && cli.bundle_path) {
    throw std::runtime_error("--check and --bundle cannot be used together");
  }
  if (cli.bundle_path && (cli.log_path || cli.motion_path)) {
    throw std::runtime_error("--bundle owns --log and --motion outputs");
  }
  return cli;
}

}  // namespace x7::follow
