#pragma once

#include <filesystem>
#include <string>

#include "plan/ptp_config.hpp"

namespace x7::ptp {

struct PreparedBundle {
  std::filesystem::path config_path;
  std::filesystem::path output_path;
  std::filesystem::path diagnostics_path;
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

// Shared archive primitives used by other offline/online reproducibility
// bundles. Destinations must be inside a caller-owned staging directory.
void copyBundleRegularFile(const std::filesystem::path& source,
                           const std::filesystem::path& destination);
void copyBundleModelDependencies(
    const std::filesystem::path& source_model,
    const std::filesystem::path& destination_root);
void writeBundleManifestFor(const std::filesystem::path& root,
                            const std::string& format, int format_version,
                            const std::string& rtctrl_version,
                            const std::string& git_commit, bool git_dirty);

PreparedBundle prepareBundle(const std::filesystem::path& staging,
                             const std::filesystem::path& source_config,
                             const Config& effective_config);
void writeBundleManifest(const std::filesystem::path& root,
                         const std::string& rtctrl_version,
                         const std::string& git_commit, bool git_dirty);

}  // namespace x7::ptp
