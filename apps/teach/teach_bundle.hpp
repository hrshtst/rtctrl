#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "teach/teach_config.hpp"

namespace x7::teach {

struct PreparedBundle {
  std::filesystem::path config_path;
  std::filesystem::path motion_path;
  std::filesystem::path log_path;
};

PreparedBundle prepareBundle(const std::filesystem::path& staging,
                             const Config& config);
void writeBundleResult(const std::filesystem::path& root, Mode mode,
                       const std::string& status, std::uint64_t raw_samples,
                       std::uint64_t output_frames, double duration_s);

}  // namespace x7::teach
