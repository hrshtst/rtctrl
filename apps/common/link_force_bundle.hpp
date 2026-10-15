// Shared link-force plumbing for the follow and track frontends: the
// disturbance log path next to a simulation log, and the provenance lines
// a bundle result records when [simulation] link_force is configured.
#pragma once

#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace x7 {

// Archived name of the link-force spec inside follow and track bundles.
inline constexpr const char* kLinkForceBundleFile = "link_force.toml";

inline constexpr const char* kHardwareLinkForceNotice =
    "note: [simulation] link_force is simulation-only and is NOT applied on "
    "hardware; a physical disturbance needs its own apparatus";

// <simulation_log without extension>_disturbance.csv, e.g.
// out/simulation.csv -> out/simulation_disturbance.csv.
inline std::filesystem::path disturbanceLogPath(
    const std::filesystem::path& simulation_log) {
  std::filesystem::path base = simulation_log;
  base.replace_extension();
  base += "_disturbance.csv";
  return base;
}

// What a simulation actually did with its configured link force:
// started = the tracking-start hook fired; applied = some completed
// substep applied a nonzero force (first/last: those substeps' start
// times on the simulator clock).
struct LinkForceOutcome {
  bool started = false;
  bool applied = false;
  double first_applied_s = std::numeric_limits<double>::quiet_NaN();
  double last_applied_s = std::numeric_limits<double>::quiet_NaN();
};

// Result-file lines (root-level TOML keys) for a simulation run.
inline std::string simulationLinkForceProvenance(
    const LinkForceOutcome& outcome) {
  std::ostringstream out;
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "link_force_spec = \"" << kLinkForceBundleFile << "\"\n"
      << "link_force_enabled = true\n"
      << "link_force_started = " << (outcome.started ? "true" : "false")
      << "\nlink_force_applied = " << (outcome.applied ? "true" : "false")
      << '\n';
  if (outcome.applied) {
    out << "link_force_first_applied_s = " << outcome.first_applied_s
        << "\nlink_force_last_applied_s = " << outcome.last_applied_s << '\n';
  }
  return out.str();
}

// Result-file lines for a hardware run whose configuration carries a
// simulation link force: archived for provenance only.
inline std::string hardwareLinkForceProvenance() {
  return std::string("link_force_spec = \"") + kLinkForceBundleFile +
         "\"\n"
         "link_force_enabled = false\n"
         "link_force_note = \"simulation specification archived for "
         "provenance; not applied by this run; not evidence that any "
         "apparatus applied a force\"\n";
}

}  // namespace x7
