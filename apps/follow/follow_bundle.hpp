#pragma once

#include <filesystem>
#include <string>

#include "follow/follow_config.hpp"
#include "plan/ptp_bundle.hpp"

namespace x7::follow {

struct PreparedBundle {
  std::filesystem::path config_path;
  std::filesystem::path simulation_motion;
  std::filesystem::path simulation_log;
  std::filesystem::path hardware_log;
};

PreparedBundle prepareBundle(const std::filesystem::path& staging,
                             const Config& effective_config);
void writeBundleResult(const std::filesystem::path& root,
                       const std::string& frontend,
                       const std::string& status, std::uint64_t cycles,
                       double worst_home_error,
                       double worst_tracking_error);

}  // namespace x7::follow
