#pragma once

#include <toml++/toml.hpp>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>

#include "rtctrl/model/attitude.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/ptp_planner.hpp"

namespace x7::ptp {

namespace fs = std::filesystem;
namespace model = rtctrl::model;

struct Config {
  fs::path model_path;
  fs::path output_path = "ptp.zvs";
  std::string end_effector = "crane_x7_tcp_link";
  model::CartesianPose start;
  model::CartesianPose end;
  std::array<double, 3> start_rpy_rad{};
  std::array<double, 3> end_rpy_rad{};
  model::PtpPlanOptions options;
  std::array<double, model::kCanonicalDof> initial_joints{};
};

struct Cli {
  bool ok = true;
  bool help = false;
  std::string error;
  fs::path config_path;
  std::optional<fs::path> output_path;
  std::optional<fs::path> bundle_path;
  std::optional<double> motion_time;
  std::optional<double> max_linear_velocity;
  std::optional<double> max_angular_velocity;
  std::optional<double> sample_rate;
  std::optional<model::PtpProfile> profile;
  std::optional<bool> strict_ik;
};

inline void rejectUnknown(const toml::table& table,
                          const std::unordered_set<std::string>& allowed,
                          const std::string& table_name) {
  for (const auto& [key, value] : table) {
    (void)value;
    if (allowed.count(std::string(key.str())) == 0) {
      throw std::runtime_error("PTP config: unknown key '" +
                               std::string(key.str()) + "' in " + table_name);
    }
  }
}

inline std::optional<double> optionalNumber(const toml::table& table,
                                            const char* key,
                                            const std::string& table_name) {
  const auto node = table[key];
  if (!node) return std::nullopt;
  const auto value = node.value<double>();
  if (!value || !std::isfinite(*value)) {
    throw std::runtime_error("PTP config: " + table_name + "." + key +
                             " must be a finite number");
  }
  return value;
}

template <std::size_t Size>
std::array<double, Size> numberArray(const toml::table& table,
                                    const char* key,
                                    const std::string& table_name) {
  const auto* array = table[key].as_array();
  if (array == nullptr || array->size() != Size) {
    throw std::runtime_error("PTP config: " + table_name + "." + key +
                             " must contain exactly " +
                             std::to_string(Size) + " numbers");
  }
  std::array<double, Size> values{};
  for (std::size_t i = 0; i < Size; ++i) {
    const auto value = (*array)[i].value<double>();
    if (!value || !std::isfinite(*value)) {
      throw std::runtime_error("PTP config: " + table_name + "." + key +
                               " must contain only finite numbers");
    }
    values[i] = *value;
  }
  return values;
}

inline model::PtpProfile parseProfile(std::string_view value) {
  if (value == "linear") return model::PtpProfile::Linear;
  if (value == "trapezoidal") return model::PtpProfile::Trapezoidal;
  if (value == "minimum-jerk") return model::PtpProfile::MinimumJerk;
  throw std::runtime_error(
      "PTP profile must be linear, trapezoidal, or minimum-jerk");
}

inline const char* profileName(model::PtpProfile profile) {
  switch (profile) {
    case model::PtpProfile::Linear:
      return "linear";
    case model::PtpProfile::Trapezoidal:
      return "trapezoidal";
    case model::PtpProfile::MinimumJerk:
      return "minimum-jerk";
  }
  return "unknown";
}

inline model::CartesianPose parsePose(const toml::table& table,
                                      const std::string& name,
                                      std::array<double, 3>* rpy_out) {
  rejectUnknown(table, {"position", "rpy_rad"}, name);
  const auto position = numberArray<3>(table, "position", name);
  const auto rpy_rad = numberArray<3>(table, "rpy_rad", name);
  *rpy_out = rpy_rad;
  model::CartesianPose pose;
  zVec3DCreate(&pose.position, position[0], position[1], position[2]);
  pose.attitude = model::worldAttitudeFromRpyRad(
      rpy_rad[0], rpy_rad[1], rpy_rad[2]);
  return pose;
}

inline fs::path withZvsExtension(fs::path path) {
  if (path.extension() != ".zvs") path += ".zvs";
  return path;
}

inline Config loadConfig(const fs::path& config_path) {
  toml::table root;
  try {
    root = toml::parse_file(config_path.string());
  } catch (const toml::parse_error& error) {
    throw std::runtime_error("PTP config: cannot parse '" +
                             config_path.string() + "': " +
                             std::string(error.description()));
  }
  rejectUnknown(root,
                {"model", "output", "end_effector", "trajectory", "start",
                 "end", "ik"},
                "root");

  Config config;
  const auto model_path = root["model"].value<std::string>();
  if (!model_path || model_path->empty()) {
    throw std::runtime_error("PTP config: model must be a non-empty path");
  }
  fs::path resolved_model(*model_path);
  if (resolved_model.is_relative()) {
    resolved_model = fs::absolute(config_path).parent_path() / resolved_model;
  }
  config.model_path = resolved_model.lexically_normal();

  if (const auto output = root["output"].value<std::string>()) {
    if (output->empty()) {
      throw std::runtime_error("PTP config: output must not be empty");
    }
    config.output_path = *output;
  } else if (root["output"]) {
    throw std::runtime_error("PTP config: output must be a path string");
  }
  config.output_path = withZvsExtension(config.output_path);

  if (const auto effector = root["end_effector"].value<std::string>()) {
    if (effector->empty()) {
      throw std::runtime_error("PTP config: end_effector must not be empty");
    }
    config.end_effector = *effector;
  } else if (root["end_effector"]) {
    throw std::runtime_error("PTP config: end_effector must be a string");
  }

  const auto* start = root["start"].as_table();
  const auto* end = root["end"].as_table();
  if (start == nullptr || end == nullptr) {
    throw std::runtime_error("PTP config: [start] and [end] are required");
  }
  config.start = parsePose(*start, "start", &config.start_rpy_rad);
  config.end = parsePose(*end, "end", &config.end_rpy_rad);

  if (const auto* trajectory = root["trajectory"].as_table()) {
    rejectUnknown(*trajectory,
                  {"profile", "sample_rate", "motion_time",
                   "max_linear_velocity", "max_angular_velocity",
                   "trapezoid_acceleration_fraction"},
                  "trajectory");
    if (const auto value = (*trajectory)["profile"].value<std::string>()) {
      config.options.profile = parseProfile(*value);
    } else if ((*trajectory)["profile"]) {
      throw std::runtime_error("PTP config: trajectory.profile must be a string");
    }
    if (const auto value =
            optionalNumber(*trajectory, "sample_rate", "trajectory")) {
      config.options.sample_rate = *value;
    }
    config.options.timing.motion_time =
        optionalNumber(*trajectory, "motion_time", "trajectory");
    config.options.timing.max_linear_velocity = optionalNumber(
        *trajectory, "max_linear_velocity", "trajectory");
    config.options.timing.max_angular_velocity = optionalNumber(
        *trajectory, "max_angular_velocity", "trajectory");
    if (const auto value = optionalNumber(
            *trajectory, "trapezoid_acceleration_fraction", "trajectory")) {
      config.options.trapezoid_acceleration_fraction = *value;
    }
  } else if (root["trajectory"]) {
    throw std::runtime_error("PTP config: trajectory must be a table");
  }

  if (const auto* ik = root["ik"].as_table()) {
    rejectUnknown(*ik,
                  {"strict", "position_tolerance", "attitude_tolerance",
                   "max_iterations", "initial_joints"},
                  "ik");
    if (const auto strict = (*ik)["strict"].value<bool>()) {
      config.options.strict_ik = *strict;
    } else if ((*ik)["strict"]) {
      throw std::runtime_error("PTP config: ik.strict must be a boolean");
    }
    if (const auto value =
            optionalNumber(*ik, "position_tolerance", "ik")) {
      config.options.position_tolerance = *value;
    }
    if (const auto value =
            optionalNumber(*ik, "attitude_tolerance", "ik")) {
      config.options.attitude_tolerance = *value;
    }
    if (const auto node = (*ik)["max_iterations"]) {
      const auto value = node.value<int>();
      if (!value) {
        throw std::runtime_error(
            "PTP config: ik.max_iterations must be an integer");
      }
      config.options.max_iterations = *value;
    }
    if ((*ik)["initial_joints"]) {
      config.initial_joints =
          numberArray<model::kCanonicalDof>(*ik, "initial_joints", "ik");
    }
  } else if (root["ik"]) {
    throw std::runtime_error("PTP config: ik must be a table");
  }

  // Central model-layer validation checks numeric ranges. Check the paired
  // limit contract here as well so malformed configs fail before model load.
  if (config.options.timing.max_linear_velocity.has_value() !=
      config.options.timing.max_angular_velocity.has_value()) {
    throw std::runtime_error(
        "PTP config: maximum linear and angular velocities must be specified "
        "together");
  }
  return config;
}

inline bool parseDouble(const char* text, double* value) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  errno = 0;
  const double parsed = std::strtod(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  *value = parsed;
  return true;
}

inline Cli cliError(const std::string& message) {
  Cli cli;
  cli.ok = false;
  cli.error = message;
  return cli;
}

inline Cli parseCli(int argc, char* argv[]) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    auto requireValue = [&](const char* option) -> const char* {
      if (i + 1 >= argc || std::string_view(argv[i + 1]).rfind("--", 0) == 0) {
        cli.ok = false;
        cli.error = std::string(option) + " requires a value";
        return nullptr;
      }
      return argv[++i];
    };
    auto parseNumber = [&](const char* option, std::optional<double>* output) {
      const char* text = requireValue(option);
      if (text == nullptr) return;
      double value = 0.0;
      if (!parseDouble(text, &value)) {
        cli.ok = false;
        cli.error = std::string(option) + ": invalid value " + text;
        return;
      }
      *output = value;
    };

    if (arg == "--help" || arg == "-h") {
      cli.help = true;
    } else if (arg == "--config") {
      const char* value = requireValue("--config");
      if (value) cli.config_path = value;
    } else if (arg == "--output") {
      const char* value = requireValue("--output");
      if (value) cli.output_path = fs::path(value);
    } else if (arg == "--bundle") {
      const char* value = requireValue("--bundle");
      if (value) cli.bundle_path = fs::path(value);
    } else if (arg == "--motion-time") {
      parseNumber("--motion-time", &cli.motion_time);
    } else if (arg == "--max-linear-velocity") {
      parseNumber("--max-linear-velocity", &cli.max_linear_velocity);
    } else if (arg == "--max-angular-velocity") {
      parseNumber("--max-angular-velocity", &cli.max_angular_velocity);
    } else if (arg == "--sample-rate") {
      parseNumber("--sample-rate", &cli.sample_rate);
    } else if (arg == "--profile") {
      const char* value = requireValue("--profile");
      if (value) {
        try {
          cli.profile = parseProfile(value);
        } catch (const std::exception& error) {
          cli.ok = false;
          cli.error = error.what();
        }
      }
    } else if (arg == "--strict-ik") {
      cli.strict_ik = true;
    } else if (arg == "--no-strict-ik") {
      cli.strict_ik = false;
    } else {
      return cliError("unknown argument: " + std::string(arg));
    }
    if (!cli.ok) return cli;
  }
  if (!cli.help && cli.config_path.empty()) {
    return cliError("--config PLAN.toml is required");
  }
  if (cli.bundle_path && cli.output_path) {
    return cliError("--bundle and --output are exclusive");
  }
  return cli;
}

inline void applyOverrides(const Cli& cli, Config* config) {
  if (cli.output_path) config->output_path = withZvsExtension(*cli.output_path);
  if (cli.motion_time) config->options.timing.motion_time = cli.motion_time;
  if (cli.max_linear_velocity) {
    config->options.timing.max_linear_velocity = cli.max_linear_velocity;
  }
  if (cli.max_angular_velocity) {
    config->options.timing.max_angular_velocity = cli.max_angular_velocity;
  }
  if (cli.sample_rate) config->options.sample_rate = *cli.sample_rate;
  if (cli.profile) config->options.profile = *cli.profile;
  if (cli.strict_ik) config->options.strict_ik = *cli.strict_ik;
  if (config->options.timing.max_linear_velocity.has_value() !=
      config->options.timing.max_angular_velocity.has_value()) {
    throw std::runtime_error(
        "maximum linear and angular velocities must be specified together");
  }
}

inline std::string quoteTomlString(std::string_view value) {
  std::ostringstream output;
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          output << character;
        }
    }
  }
  output << '"';
  return output.str();
}

template <std::size_t Size>
void writeTomlArray(std::ostream& output,
                    const std::array<double, Size>& values) {
  output << '[';
  for (std::size_t i = 0; i < Size; ++i) {
    if (i != 0) output << ", ";
    output << std::setprecision(std::numeric_limits<double>::max_digits10)
           << values[i];
  }
  output << ']';
}

inline std::string serializeEffectiveConfig(
    const Config& config, const fs::path& model_path,
    const fs::path& output_path) {
  std::ostringstream output;
  output << "model = " << quoteTomlString(model_path.generic_string())
         << "\noutput = " << quoteTomlString(output_path.generic_string())
         << "\nend_effector = " << quoteTomlString(config.end_effector)
         << "\n\n[trajectory]\nprofile = "
         << quoteTomlString(profileName(config.options.profile))
         << "\nsample_rate = "
         << std::setprecision(std::numeric_limits<double>::max_digits10)
         << config.options.sample_rate;
  if (config.options.timing.motion_time) {
    output << "\nmotion_time = " << *config.options.timing.motion_time;
  }
  if (config.options.timing.max_linear_velocity) {
    output << "\nmax_linear_velocity = "
           << *config.options.timing.max_linear_velocity
           << "\nmax_angular_velocity = "
           << *config.options.timing.max_angular_velocity;
  }
  output << "\ntrapezoid_acceleration_fraction = "
         << config.options.trapezoid_acceleration_fraction
         << "\n\n[start]\nposition = [" << config.start.position.c.x << ", "
         << config.start.position.c.y << ", " << config.start.position.c.z
         << "]\nrpy_rad = ";
  writeTomlArray(output, config.start_rpy_rad);
  output << "\n\n[end]\nposition = [" << config.end.position.c.x << ", "
         << config.end.position.c.y << ", " << config.end.position.c.z
         << "]\nrpy_rad = ";
  writeTomlArray(output, config.end_rpy_rad);
  output << "\n\n[ik]\nstrict = "
         << (config.options.strict_ik ? "true" : "false")
         << "\nposition_tolerance = " << config.options.position_tolerance
         << "\nattitude_tolerance = " << config.options.attitude_tolerance
         << "\nmax_iterations = " << config.options.max_iterations
         << "\ninitial_joints = ";
  writeTomlArray(output, config.initial_joints);
  output << '\n';
  return output.str();
}

}  // namespace x7::ptp
