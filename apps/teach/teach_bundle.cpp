#include "teach/teach_bundle.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

#include "plan/ptp_bundle.hpp"

namespace x7::teach {

namespace fs = std::filesystem;

namespace {

void writeEffectiveConfig(const fs::path& path, const Config& config) {
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "format = \"rtctrl-x7-teach\"\nversion = 1\n"
      << "model = \"model/" << config.model_path.filename().generic_string()
      << "\"\nhardware_config = \"hardware.toml\"\nmode = \""
      << modeName(config.mode) << "\"\n\n[recording]\nsample_rate_hz = "
      << config.recording.sample_rate_hz
      << "\nstart_timeout_s = " << config.recording.start_timeout_s
      << "\nmax_duration_s = " << config.recording.max_duration_s
      << "\n\n[finalization]\noperator_timeout_s = "
      << config.finalization.operator_timeout_s
      << "\n\n[output]\nmotion = \"trajectory.zvs\""
      << "\nlog = \"recording.csv\"\n";
  out.close();
  if (!out) throw std::runtime_error("cannot finish " + path.string());
}

}  // namespace

PreparedBundle prepareBundle(const fs::path& staging, const Config& config) {
  x7::ptp::copyBundleRegularFile(config.source_path, staging / "source.toml");
  x7::ptp::copyBundleModelDependencies(config.model_path, staging / "model");
  x7::ptp::copyBundleRegularFile(config.hardware_config_path,
                                 staging / "hardware.toml");
  const auto config_path = staging / "teach.toml";
  writeEffectiveConfig(config_path, config);
  return {config_path, staging / "trajectory.zvs",
          staging / "recording.csv"};
}

void writeBundleResult(const fs::path& root, Mode mode,
                       const std::string& status, std::uint64_t raw_samples,
                       std::uint64_t output_frames, double duration_s) {
  const auto path = root / "result.toml";
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "format = \"rtctrl-x7-teach-result\"\nversion = 1\nmode = \""
      << modeName(mode) << "\"\nstatus = " << std::quoted(status)
      << "\nraw_samples = " << raw_samples
      << "\noutput_frames = " << output_frames
      << "\nduration_s = " << duration_s << '\n';
}

}  // namespace x7::teach
