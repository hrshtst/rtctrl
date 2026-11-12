#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "follow/follow_config.hpp"
#include "follow/follow_preflight.hpp"
#include "plan/ptp_bundle.hpp"

namespace x7::follow {

// Archived name of the learned command offset inside a follow bundle.
inline constexpr const char* kOffsetBundleFile = "offset.csv";

// Result-file lines (root-level TOML keys) recording which learned offset
// a run applied; empty without one.
inline std::string offsetProvenance(
    const std::optional<AcceptedOffset>& accepted) {
  if (!accepted) return {};
  // The method is one of zero/ilc/fel and the hash 64 hex characters (the
  // offset passed its structural check), so neither needs escaping.
  return "offset_method = \"" + accepted->offset.method +
         "\"\noffset_trial = " + std::to_string(accepted->offset.trial) +
         "\noffset_sha256 = \"" + accepted->sha256 + "\"\n";
}

struct PreparedBundle {
  std::filesystem::path config_path;
  std::filesystem::path simulation_motion;
  std::filesystem::path simulation_log;
  std::filesystem::path hardware_log;
};

PreparedBundle prepareBundle(const std::filesystem::path& staging,
                             const Config& effective_config);
// provenance: extra root-level TOML lines appended verbatim (e.g. the
// link-force provenance from common/link_force_bundle.hpp).
void writeBundleResult(const std::filesystem::path& root,
                       const std::string& frontend,
                       const std::string& status, std::uint64_t cycles,
                       double worst_home_error,
                       double worst_tracking_error,
                       const std::string& provenance = std::string());

}  // namespace x7::follow
