#pragma once

#include <filesystem>
#include <string>

#include "plan/ptp_bundle.hpp"
#include "track/track_config.hpp"
#include "track/track_run.hpp"

namespace x7::track {

struct PreparedBundle {
  std::filesystem::path config_path;
  std::filesystem::path simulation_motion;
  std::filesystem::path simulation_log;
  std::filesystem::path hardware_log;
};

PreparedBundle prepareBundle(const std::filesystem::path& staging,
                             const Config& effective_config);
void writeBundleResult(const std::filesystem::path& root,
                       const std::string& frontend, const RunResult& result);

}  // namespace x7::track
