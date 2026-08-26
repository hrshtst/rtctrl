#pragma once

#include <filesystem>
#include <string>

#include "plan/ptp_config.hpp"

namespace x7::ptp {

struct PreparedBundle {
  std::filesystem::path config_path;
  std::filesystem::path output_path;
};

// Owns a sibling staging directory and publishes it without replacing an
// existing path. Construction checks the target before any config is loaded.
class BundleWorkspace {
 public:
  explicit BundleWorkspace(const std::filesystem::path& target);
  ~BundleWorkspace();

  BundleWorkspace(const BundleWorkspace&) = delete;
  BundleWorkspace& operator=(const BundleWorkspace&) = delete;

  const std::filesystem::path& target() const { return target_; }
  const std::filesystem::path& staging() const { return staging_; }
  void publish();

 private:
  std::filesystem::path target_;
  std::filesystem::path staging_;
  bool published_ = false;
};

PreparedBundle prepareBundle(const std::filesystem::path& staging,
                             const std::filesystem::path& source_config,
                             const Config& effective_config);
void writeBundleManifest(const std::filesystem::path& root,
                         const std::string& rtctrl_version,
                         const std::string& git_commit, bool git_dirty);

}  // namespace x7::ptp
