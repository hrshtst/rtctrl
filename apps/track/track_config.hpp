#pragma once

#include <toml++/toml.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "follow/follow_config.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace x7::track {

namespace fs = std::filesystem;
namespace model = rtctrl::model;

using HomeConfig = x7::follow::HomeConfig;
using SimulationConfig = x7::follow::SimulationConfig;
using SafetyConfig = x7::follow::SafetyConfig;
using FinalizationConfig = x7::follow::FinalizationConfig;

struct AssessmentConfig {
  double rms_error_rad = 0.02;
  double peak_error_rad = 0.10;
};

struct OutputConfig {
  fs::path simulation_motion = "track_sim.zvs";
  fs::path simulation_log = "track_sim.csv";
  fs::path hardware_log = "track_hw.csv";
};

struct Config {
  fs::path source_path;
  fs::path model_path;
  fs::path reference_path;
  fs::path hardware_config_path;
  double control_rate_hz = 100.0;
  double kp = 20.0;
  double kd = 2.0;
  double playback_rate = 1.0;
  std::array<double, model::kCanonicalDof> effort_limit_nm{};
  bool effort_limit_set = false;
  HomeConfig home;
  model::ZvsTrajectoryOptions reference;
  SimulationConfig simulation;
  SafetyConfig safety;
  AssessmentConfig assessment;
  FinalizationConfig finalization;
  OutputConfig output;
};

struct Cli {
  fs::path config_path;
  std::optional<fs::path> reference_path;
  std::optional<fs::path> log_path;
  std::optional<fs::path> motion_path;
  std::optional<fs::path> bundle_path;
  std::optional<std::string> port;
  std::optional<double> kp;
  std::optional<double> kd;
  std::optional<double> playback_rate;
  std::optional<double> effort_limit_nm;
  std::optional<model::ReferenceFilter> filter;
  std::optional<model::ReferenceInterpolation> interpolation;
  bool check = false;
  bool help = false;
};

inline std::array<double, model::kCanonicalDof> readEffortLimits(
    const toml::table& table, bool* set) {
  std::array<double, model::kCanonicalDof> result{};
  const auto node = table["effort_limit_nm"];
  if (!node) return result;
  const auto* values = node.as_array();
  if (values == nullptr || (values->size() != 1 &&
                            values->size() != model::kCanonicalDof)) {
    throw std::runtime_error(
        "track config: control.effort_limit_nm needs one or eight values");
  }
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    const auto value = (*values)[values->size() == 1 ? 0 : i].value<double>();
    if (!value || !std::isfinite(*value) || *value <= 0.0) {
      throw std::runtime_error(
          "track config: effort limits must be finite and positive");
    }
    result[i] = *value;
  }
  *set = true;
  return result;
}

inline Config loadConfig(const fs::path& path) {
  using namespace x7::follow;
  toml::table root;
  try {
    root = toml::parse_file(path.string());
  } catch (const toml::parse_error& error) {
    throw std::runtime_error("track config: cannot parse '" + path.string() +
                             "': " + std::string(error.description()));
  }
  rejectUnknown(root,
                {"format", "version", "model", "reference",
                 "hardware_config", "control", "home",
                 "reference_processing", "simulation", "safety",
                 "assessment", "finalization", "output"},
                "root");
  if (root["format"].value_or(std::string{}) != "rtctrl-x7-track" ||
      root["version"].value_or<std::int64_t>(0) != 1) {
    throw std::runtime_error(
        "track config: format must be rtctrl-x7-track version 1");
  }

  Config config;
  config.source_path = fs::absolute(path).lexically_normal();
  config.model_path = inputPath(root, "model", path, true);
  config.reference_path = inputPath(root, "reference", path, true);
  config.hardware_config_path = inputPath(root, "hardware_config", path, true);

  const auto& control = optionalTable(root, "control");
  rejectUnknown(control,
                {"rate_hz", "kp", "kd", "playback_rate",
                 "effort_limit_nm"},
                "control");
  config.control_rate_hz =
      number(control, "rate_hz", config.control_rate_hz, "control");
  config.kp = number(control, "kp", config.kp, "control");
  config.kd = number(control, "kd", config.kd, "control");
  config.playback_rate = number(control, "playback_rate",
                                config.playback_rate, "control");
  config.effort_limit_nm =
      readEffortLimits(control, &config.effort_limit_set);
  if (config.control_rate_hz < 20.0 || config.control_rate_hz > 200.0 ||
      config.kp < 0.0 || config.kd < 0.0 || config.playback_rate <= 0.0 ||
      config.playback_rate > 1.0) {
    throw std::runtime_error("track config: invalid control value");
  }

  const auto& home = optionalTable(root, "home");
  rejectUnknown(home,
                {"profile", "motion_time", "velocity_limit",
                 "trapezoid_acceleration_fraction", "strict",
                 "tolerance_rad", "settle_time_s"},
                "home");
  if (const auto value = home["profile"].value<std::string>()) {
    config.home.profile = parseProfile(*value);
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
  config.home.settle_time_s =
      number(home, "settle_time_s", config.home.settle_time_s, "home");
  if (config.home.velocity_limit <= 0.0 ||
      (config.home.motion_time && *config.home.motion_time <= 0.0) ||
      config.home.tolerance_rad <= 0.0 || config.home.settle_time_s < 0.0 ||
      config.home.trapezoid_acceleration_fraction <= 0.0 ||
      config.home.trapezoid_acceleration_fraction > 0.5) {
    throw std::runtime_error("track config: invalid home value");
  }

  const auto& processing = optionalTable(root, "reference_processing");
  rejectUnknown(processing,
                {"filter", "interpolation", "low_pass_cutoff_hz",
                 "filter_window", "savitzky_golay_order"},
                "reference_processing");
  if (const auto value = processing["filter"].value<std::string>()) {
    config.reference.filter = parseFilter(*value);
  }
  if (const auto value = processing["interpolation"].value<std::string>()) {
    config.reference.interpolation = parseInterpolation(*value);
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

  const auto& simulation = optionalTable(root, "simulation");
  rejectUnknown(simulation,
                {"initial_posture", "initial_joints", "integration_step_s",
                 "position_kp", "position_kd"},
                "simulation");
  if (const auto value = simulation["initial_posture"].value<std::string>()) {
    if (*value == "model") {
      config.simulation.initial_posture = SimulationConfig::InitialPosture::Model;
    } else if (*value == "zeros") {
      config.simulation.initial_posture = SimulationConfig::InitialPosture::Zeros;
    } else {
      throw std::runtime_error(
          "track config: initial_posture must be model or zeros");
    }
  }
  if (const auto* initial = simulation["initial_joints"].as_array()) {
    if (initial->size() != model::kCanonicalDof) {
      throw std::runtime_error("track config: initial_joints needs eight values");
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const auto value = (*initial)[i].value<double>();
      if (!value || !std::isfinite(*value)) {
        throw std::runtime_error("track config: initial_joints must be finite");
      }
      config.simulation.initial_joints[i] = *value;
    }
    config.simulation.initial_posture = SimulationConfig::InitialPosture::Explicit;
  }
  config.simulation.integration_step_s = number(
      simulation, "integration_step_s", config.simulation.integration_step_s,
      "simulation");
  config.simulation.position_kp = number(
      simulation, "position_kp", config.simulation.position_kp, "simulation");
  config.simulation.position_kd = number(
      simulation, "position_kd", config.simulation.position_kd, "simulation");
  if (config.simulation.integration_step_s <= 0.0 ||
      config.simulation.position_kp <= 0.0 ||
      config.simulation.position_kd < 0.0) {
    throw std::runtime_error("track config: invalid simulation value");
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
      config.safety.sustained_abort_error_rad < config.safety.warning_error_rad ||
      config.safety.immediate_abort_error_rad <
          config.safety.sustained_abort_error_rad ||
      config.safety.sustained_abort_time_s <= 0.0) {
    throw std::runtime_error("track config: invalid safety threshold ordering");
  }

  const auto& assessment = optionalTable(root, "assessment");
  rejectUnknown(assessment, {"rms_error_rad", "peak_error_rad"},
                "assessment");
  config.assessment.rms_error_rad = number(
      assessment, "rms_error_rad", config.assessment.rms_error_rad,
      "assessment");
  config.assessment.peak_error_rad = number(
      assessment, "peak_error_rad", config.assessment.peak_error_rad,
      "assessment");
  if (config.assessment.rms_error_rad <= 0.0 ||
      config.assessment.peak_error_rad < config.assessment.rms_error_rad) {
    throw std::runtime_error("track config: invalid assessment thresholds");
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
    throw std::runtime_error("track config: invalid finalization time");
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
  return config;
}

inline void applyOverrides(const Cli& cli, Config* config) {
  if (cli.reference_path) {
    config->reference_path = fs::absolute(*cli.reference_path).lexically_normal();
  }
  if (cli.kp) config->kp = *cli.kp;
  if (cli.kd) config->kd = *cli.kd;
  if (cli.playback_rate) config->playback_rate = *cli.playback_rate;
  if (cli.effort_limit_nm) {
    config->effort_limit_nm.fill(*cli.effort_limit_nm);
    config->effort_limit_set = true;
  }
  if (cli.filter) config->reference.filter = *cli.filter;
  if (cli.interpolation) config->reference.interpolation = *cli.interpolation;
  if (config->kp < 0.0 || config->kd < 0.0 ||
      config->playback_rate <= 0.0 || config->playback_rate > 1.0 ||
      (cli.effort_limit_nm && *cli.effort_limit_nm <= 0.0)) {
    throw std::runtime_error("track override: invalid control value");
  }
}

inline Cli parseCli(int argc, char* argv[], bool simulation) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") cli.help = true;
    else if (arg == "--check") cli.check = true;
    else if (arg == "--config" || arg == "--reference" || arg == "--kp" ||
             arg == "--kd" || arg == "--playback-rate" ||
             arg == "--effort-limit-nm" || arg == "--filter" ||
             arg == "--interpolation" || arg == "--log" ||
             arg == "--bundle" || arg == "--port" || arg == "--motion") {
      if (i + 1 >= argc || argv[i + 1][0] == '-') {
        throw std::runtime_error(arg + " requires a value");
      }
      const std::string value = argv[++i];
      if (arg == "--config") cli.config_path = value;
      else if (arg == "--reference") cli.reference_path = value;
      else if (arg == "--kp")
        cli.kp = x7::follow::parseCliNumber(arg, value.c_str());
      else if (arg == "--kd")
        cli.kd = x7::follow::parseCliNumber(arg, value.c_str());
      else if (arg == "--playback-rate")
        cli.playback_rate = x7::follow::parseCliNumber(arg, value.c_str());
      else if (arg == "--effort-limit-nm")
        cli.effort_limit_nm = x7::follow::parseCliNumber(arg, value.c_str());
      else if (arg == "--filter") cli.filter = x7::follow::parseFilter(value);
      else if (arg == "--interpolation")
        cli.interpolation = x7::follow::parseInterpolation(value);
      else if (arg == "--log") cli.log_path = value;
      else if (arg == "--bundle") cli.bundle_path = value;
      else if (arg == "--port") cli.port = value;
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
  if (cli.check && cli.bundle_path) {
    throw std::runtime_error("--check and --bundle cannot be used together");
  }
  if (cli.bundle_path && (cli.log_path || cli.motion_path)) {
    throw std::runtime_error("--bundle owns --log and --motion outputs");
  }
  return cli;
}

}  // namespace x7::track
