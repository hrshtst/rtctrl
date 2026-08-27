#pragma once

#include <toml++/toml.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace x7::teach {

namespace fs = std::filesystem;

inline constexpr double kHardwareRateHz = 100.0;
inline constexpr double kMaxGravitySessionS = 60.0;

enum class Mode { TorqueOff, GravityCompensation };

inline const char* modeName(Mode mode) {
  return mode == Mode::TorqueOff ? "torque-off" : "gravity-compensation";
}

inline Mode parseMode(std::string_view value) {
  if (value == "torque-off") return Mode::TorqueOff;
  if (value == "gravity-compensation") return Mode::GravityCompensation;
  throw std::runtime_error(
      "teach mode must be torque-off or gravity-compensation");
}

struct RecordingConfig {
  double sample_rate_hz = 100.0;
  double start_timeout_s = 8.0;
  double max_duration_s = 30.0;
};

struct FinalizationConfig {
  double operator_timeout_s = 15.0;
};

struct OutputConfig {
  fs::path motion = "taught.zvs";
  fs::path log = "taught.csv";
};

struct Config {
  fs::path source_path;
  fs::path model_path;
  fs::path hardware_config_path;
  Mode mode = Mode::TorqueOff;
  RecordingConfig recording;
  FinalizationConfig finalization;
  OutputConfig output;
};

struct Cli {
  fs::path config_path;
  std::optional<std::string> port;
  std::optional<Mode> mode;
  std::optional<double> sample_rate_hz;
  std::optional<double> max_duration_s;
  std::optional<fs::path> output_path;
  std::optional<fs::path> log_path;
  std::optional<fs::path> bundle_path;
  bool check = false;
  bool help = false;
};

inline void rejectUnknown(const toml::table& table,
                          std::initializer_list<std::string_view> allowed,
                          const std::string& name) {
  for (const auto& [key, node] : table) {
    (void)node;
    if (std::find(allowed.begin(), allowed.end(), key.str()) == allowed.end()) {
      throw std::runtime_error("teach config: unknown key '" +
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
    throw std::runtime_error(std::string("teach config: ") + key +
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
    throw std::runtime_error(std::string("teach config: ") + table_name +
                             "." + key + " must be a finite number");
  }
  return *value;
}

inline fs::path inputPath(const toml::table& root, const char* key,
                          const fs::path& source) {
  const auto value = root[key].value<std::string>();
  if (!value || value->empty()) {
    throw std::runtime_error(std::string("teach config: ") + key +
                             " must be a non-empty path");
  }
  fs::path path(*value);
  if (path.is_relative()) path = fs::absolute(source).parent_path() / path;
  return path.lexically_normal();
}

inline fs::path outputPath(const toml::table& table, const char* key,
                           fs::path fallback, const char* extension) {
  if (const auto value = table[key].value<std::string>()) {
    if (value->empty()) {
      throw std::runtime_error(std::string("teach config: output.") + key +
                               " must not be empty");
    }
    fallback = *value;
  } else if (table[key]) {
    throw std::runtime_error(std::string("teach config: output.") + key +
                             " must be a path string");
  }
  if (fallback.extension() != extension) fallback += extension;
  return fallback;
}

inline void validate(Config* config) {
  if (config->recording.sample_rate_hz < 1.0 ||
      config->recording.sample_rate_hz > kHardwareRateHz) {
    throw std::runtime_error(
        "teach config: recording.sample_rate_hz must be in [1, 100]");
  }
  if (config->recording.start_timeout_s <= 0.0 ||
      config->recording.max_duration_s <= 0.0 ||
      config->finalization.operator_timeout_s <= 0.0) {
    throw std::runtime_error("teach config: recording times must be positive");
  }
  if (config->mode == Mode::GravityCompensation &&
      config->recording.start_timeout_s +
              config->recording.max_duration_s +
              config->finalization.operator_timeout_s >
          kMaxGravitySessionS) {
    throw std::runtime_error(
        "teach config: gravity-compensation start, recording, and support "
        "timeouts must total at most 60 s");
  }
  if (fs::absolute(config->output.motion).lexically_normal() ==
      fs::absolute(config->output.log).lexically_normal()) {
    throw std::runtime_error("teach config: motion and log paths must differ");
  }
}

inline Config loadConfig(const fs::path& path) {
  toml::table root;
  try {
    root = toml::parse_file(path.string());
  } catch (const toml::parse_error& error) {
    throw std::runtime_error("teach config: cannot parse '" + path.string() +
                             "': " + std::string(error.description()));
  }
  rejectUnknown(root,
                {"format", "version", "model", "hardware_config", "mode",
                 "recording", "finalization", "output"},
                "root");
  if (root["format"].value_or(std::string()) != "rtctrl-x7-teach" ||
      root["version"].value_or(0) != 1) {
    throw std::runtime_error(
        "teach config: expected format rtctrl-x7-teach version 1");
  }

  Config config;
  config.source_path = fs::absolute(path).lexically_normal();
  config.model_path = inputPath(root, "model", path);
  config.hardware_config_path = inputPath(root, "hardware_config", path);
  if (const auto value = root["mode"].value<std::string>()) {
    config.mode = parseMode(*value);
  } else if (root["mode"]) {
    throw std::runtime_error("teach config: mode must be a string");
  }

  const auto& recording = optionalTable(root, "recording");
  rejectUnknown(recording,
                {"sample_rate_hz", "start_timeout_s", "max_duration_s"},
                "recording");
  config.recording.sample_rate_hz =
      number(recording, "sample_rate_hz", config.recording.sample_rate_hz,
             "recording");
  config.recording.start_timeout_s =
      number(recording, "start_timeout_s", config.recording.start_timeout_s,
             "recording");
  config.recording.max_duration_s =
      number(recording, "max_duration_s", config.recording.max_duration_s,
             "recording");

  const auto& finalization = optionalTable(root, "finalization");
  rejectUnknown(finalization, {"operator_timeout_s"}, "finalization");
  config.finalization.operator_timeout_s = number(
      finalization, "operator_timeout_s",
      config.finalization.operator_timeout_s, "finalization");

  const auto& output = optionalTable(root, "output");
  rejectUnknown(output, {"motion", "log"}, "output");
  config.output.motion =
      outputPath(output, "motion", config.output.motion, ".zvs");
  config.output.log = outputPath(output, "log", config.output.log, ".csv");
  validate(&config);
  return config;
}

inline void applyOverrides(const Cli& cli, Config* config) {
  if (cli.mode) config->mode = *cli.mode;
  if (cli.sample_rate_hz) {
    config->recording.sample_rate_hz = *cli.sample_rate_hz;
  }
  if (cli.max_duration_s) {
    config->recording.max_duration_s = *cli.max_duration_s;
  }
  if (cli.output_path) {
    config->output.motion = *cli.output_path;
    if (config->output.motion.extension() != ".zvs") {
      config->output.motion += ".zvs";
    }
  }
  if (cli.log_path) {
    config->output.log = *cli.log_path;
    if (config->output.log.extension() != ".csv") {
      config->output.log += ".csv";
    }
  }
  validate(config);
}

inline double parseNumber(const std::string& option, const char* value) {
  char* end = nullptr;
  const double parsed = std::strtod(value, &end);
  if (end == value || *end != '\0' || !std::isfinite(parsed)) {
    throw std::runtime_error(option + " requires a finite number");
  }
  return parsed;
}

inline Cli parseCli(int argc, char* argv[]) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      cli.help = true;
    } else if (arg == "--check") {
      cli.check = true;
    } else if (arg == "--config" || arg == "--port" || arg == "--mode" ||
               arg == "--sample-rate" || arg == "--max-duration" ||
               arg == "--output" || arg == "--log" || arg == "--bundle") {
      if (i + 1 >= argc || argv[i + 1][0] == '-') {
        throw std::runtime_error(arg + " requires a value");
      }
      const char* value = argv[++i];
      if (arg == "--config") cli.config_path = value;
      else if (arg == "--port") cli.port = value;
      else if (arg == "--mode") cli.mode = parseMode(value);
      else if (arg == "--sample-rate")
        cli.sample_rate_hz = parseNumber(arg, value);
      else if (arg == "--max-duration")
        cli.max_duration_s = parseNumber(arg, value);
      else if (arg == "--output") cli.output_path = value;
      else if (arg == "--log") cli.log_path = value;
      else cli.bundle_path = value;
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (!cli.help && cli.config_path.empty()) {
    throw std::runtime_error("--config is required");
  }
  if (cli.check && cli.bundle_path) {
    throw std::runtime_error("--check and --bundle cannot be used together");
  }
  if (cli.bundle_path && (cli.output_path || cli.log_path)) {
    throw std::runtime_error(
        "--bundle cannot be used with --output or --log");
  }
  return cli;
}

}  // namespace x7::teach
