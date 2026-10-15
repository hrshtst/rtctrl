#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>

#include "follow/follow_config.hpp"
#include "rtctrl/model/force_budget.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace x7::disturb {

namespace fs = std::filesystem;
namespace model = rtctrl::model;

// x7_force_budget configuration (format = "rtctrl-x7-force-budget",
// version = 1). Input paths resolve relative to the config file; the CSV
// output path is relative to the invocation directory.
struct Config {
  fs::path source_path;
  fs::path model_path;
  fs::path reference_path;
  fs::path link_force_path;
  model::ZvsTrajectoryOptions reference;
  model::ForceBudgetOptions budget;
  fs::path csv_path = "force_budget.csv";
};

struct Cli {
  fs::path config_path;
  std::optional<fs::path> reference_path;
  std::optional<fs::path> output_path;
  bool check = false;
  bool help = false;
};

inline std::array<double, model::kCanonicalDof> readAllowances(
    const toml::table& budget) {
  const auto* values = budget["joint_allowance_nm"].as_array();
  if (values == nullptr ||
      (values->size() != 1 && values->size() != model::kCanonicalDof)) {
    throw std::runtime_error(
        "force budget config: budget.joint_allowance_nm needs one or eight "
        "values");
  }
  std::array<double, model::kCanonicalDof> result{};
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    const auto value = (*values)[values->size() == 1 ? 0 : i].value<double>();
    if (!value || !std::isfinite(*value) || *value <= 0.0) {
      throw std::runtime_error(
          "force budget config: joint allowances must be finite and positive");
    }
    result[i] = *value;
  }
  return result;
}

inline Config loadConfig(const fs::path& path) {
  using x7::follow::integer;
  using x7::follow::number;
  using x7::follow::optionalTable;
  using x7::follow::rejectUnknown;
  toml::table root;
  try {
    root = toml::parse_file(path.string());
  } catch (const toml::parse_error& error) {
    throw std::runtime_error("force budget config: cannot parse '" +
                             path.string() +
                             "': " + std::string(error.description()));
  }
  rejectUnknown(root,
                {"format", "version", "model", "reference", "link_force",
                 "reference_processing", "budget", "output"},
                "root");
  if (root["format"].value_or(std::string{}) != "rtctrl-x7-force-budget" ||
      root["version"].value_or<std::int64_t>(0) != 1) {
    throw std::runtime_error(
        "force budget config: format must be rtctrl-x7-force-budget "
        "version 1");
  }
  Config config;
  config.source_path = fs::absolute(path).lexically_normal();
  config.model_path = x7::follow::inputPath(root, "model", path, true);
  config.reference_path = x7::follow::inputPath(root, "reference", path, true);
  config.link_force_path =
      x7::follow::inputPath(root, "link_force", path, true);

  const auto& processing = optionalTable(root, "reference_processing");
  rejectUnknown(processing,
                {"filter", "interpolation", "low_pass_cutoff_hz",
                 "filter_window", "savitzky_golay_order"},
                "reference_processing");
  if (const auto value = processing["interpolation"].value<std::string>()) {
    config.reference.interpolation = x7::follow::parseInterpolation(*value);
  }
  if (const auto value = processing["filter"].value<std::string>()) {
    config.reference.filter = x7::follow::parseFilter(*value);
  }
  config.reference.low_pass_cutoff_hz =
      number(processing, "low_pass_cutoff_hz",
             config.reference.low_pass_cutoff_hz, "reference_processing");
  config.reference.filter_window =
      integer(processing, "filter_window", config.reference.filter_window,
              "reference_processing");
  config.reference.savitzky_golay_order =
      integer(processing, "savitzky_golay_order",
              config.reference.savitzky_golay_order, "reference_processing");

  const auto* budget = root["budget"].as_table();
  if (budget == nullptr) {
    throw std::runtime_error("force budget config: missing [budget] table");
  }
  rejectUnknown(*budget, {"joint_allowance_nm", "sample_rate_hz"}, "budget");
  config.budget.joint_allowance_nm = readAllowances(*budget);
  config.budget.sample_rate_hz =
      number(*budget, "sample_rate_hz", config.budget.sample_rate_hz, "budget");
  if (config.budget.sample_rate_hz <= 0.0) {
    throw std::runtime_error(
        "force budget config: budget.sample_rate_hz must be positive");
  }

  const auto& output = optionalTable(root, "output");
  rejectUnknown(output, {"csv"}, "output");
  config.csv_path =
      x7::follow::outputPath(output, "csv", config.csv_path, ".csv");
  return config;
}

inline void applyOverrides(const Cli& cli, Config* config) {
  if (cli.reference_path) {
    config->reference_path =
        fs::absolute(*cli.reference_path).lexically_normal();
  }
  if (cli.output_path) config->csv_path = *cli.output_path;
}

inline Cli parseCli(int argc, char* argv[]) {
  Cli cli;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      cli.help = true;
    } else if (arg == "--check") {
      cli.check = true;
    } else if (arg == "--config" || arg == "--reference" || arg == "--output") {
      if (i + 1 >= argc || argv[i + 1][0] == '-') {
        throw std::runtime_error(arg + " requires a value");
      }
      const fs::path value = argv[++i];
      if (arg == "--config") {
        cli.config_path = value;
      } else if (arg == "--reference") {
        cli.reference_path = value;
      } else {
        cli.output_path = value;
      }
    } else {
      throw std::runtime_error("unknown option: " + arg);
    }
  }
  if (!cli.help && cli.config_path.empty()) {
    throw std::runtime_error("--config is required");
  }
  return cli;
}

}  // namespace x7::disturb
