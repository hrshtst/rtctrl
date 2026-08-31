#include "track/track_bundle.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace x7::track {

namespace fs = std::filesystem;

namespace {

std::string quote(const std::string& value) {
  std::ostringstream out;
  out << std::quoted(value);
  return out.str();
}

const char* profileName(model::PtpProfile profile) {
  switch (profile) {
    case model::PtpProfile::Linear: return "linear";
    case model::PtpProfile::Trapezoidal: return "trapezoidal";
    case model::PtpProfile::MinimumJerk: return "minimum-jerk";
  }
  return "minimum-jerk";
}

const char* filterName(model::ReferenceFilter filter) {
  switch (filter) {
    case model::ReferenceFilter::None: return "none";
    case model::ReferenceFilter::LowPass: return "low-pass";
    case model::ReferenceFilter::MovingAverage: return "moving-average";
    case model::ReferenceFilter::SavitzkyGolay: return "savitzky-golay";
  }
  return "none";
}

const char* interpolationName(model::ReferenceInterpolation interpolation) {
  return interpolation == model::ReferenceInterpolation::Linear
             ? "linear"
             : "shape-preserving-cubic";
}

template <class Values>
void writeArray(std::ostream& out, const Values& values) {
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0) out << ", ";
    out << values[i];
  }
  out << ']';
}

void writeEffectiveConfig(const fs::path& path, const Config& config) {
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "format = \"rtctrl-x7-track\"\nversion = 1\n"
      << "model = \"model/" << config.model_path.filename().generic_string()
      << "\"\nreference = \"reference.zvs\"\n"
      << "hardware_config = \"hardware.toml\"\n"
      << "\n[control]\nrate_hz = " << config.control_rate_hz
      << "\nkp = " << config.kp << "\nkd = " << config.kd
      << "\nplayback_rate = " << config.playback_rate;
  if (config.effort_limit_set) {
    out << "\neffort_limit_nm = ";
    writeArray(out, config.effort_limit_nm);
  }
  out << "\n\n[home]\nprofile = " << quote(profileName(config.home.profile));
  if (config.home.motion_time) out << "\nmotion_time = " << *config.home.motion_time;
  out << "\nvelocity_limit = " << config.home.velocity_limit
      << "\ntrapezoid_acceleration_fraction = "
      << config.home.trapezoid_acceleration_fraction
      << "\nstrict = " << (config.home.strict ? "true" : "false")
      << "\ntolerance_rad = " << config.home.tolerance_rad
      << "\nsettle_time_s = " << config.home.settle_time_s
      << "\n\n[reference_processing]\nfilter = "
      << quote(filterName(config.reference.filter))
      << "\ninterpolation = "
      << quote(interpolationName(config.reference.interpolation))
      << "\nlow_pass_cutoff_hz = " << config.reference.low_pass_cutoff_hz
      << "\nfilter_window = " << config.reference.filter_window
      << "\nsavitzky_golay_order = "
      << config.reference.savitzky_golay_order
      << "\n\n[simulation]\nintegration_step_s = "
      << config.simulation.integration_step_s
      << "\nposition_kp = " << config.simulation.position_kp
      << "\nposition_kd = " << config.simulation.position_kd;
  if (config.simulation.initial_posture ==
      SimulationConfig::InitialPosture::Explicit) {
    out << "\ninitial_joints = ";
    writeArray(out, config.simulation.initial_joints);
  } else {
    out << "\ninitial_posture = "
        << quote(config.simulation.initial_posture ==
                         SimulationConfig::InitialPosture::Model
                     ? "model"
                     : "zeros");
  }
  out << "\n\n[safety]\nwarning_error_rad = "
      << config.safety.warning_error_rad
      << "\nsustained_abort_error_rad = "
      << config.safety.sustained_abort_error_rad
      << "\nsustained_abort_time_s = "
      << config.safety.sustained_abort_time_s
      << "\nimmediate_abort_error_rad = "
      << config.safety.immediate_abort_error_rad
      << "\n\n[assessment]\nrms_error_rad = "
      << config.assessment.rms_error_rad
      << "\npeak_error_rad = " << config.assessment.peak_error_rad
      << "\n\n[finalization]\nwait_time_s = "
      << config.finalization.wait_time_s
      << "\noperator_timeout_s = "
      << config.finalization.operator_timeout_s
      << "\nsimulation_hold_time_s = "
      << config.finalization.simulation_hold_time_s
      << "\n\n[output]\nsimulation_motion = \"simulation.zvs\""
      << "\nsimulation_log = \"simulation.csv\""
      << "\nhardware_log = \"hardware.csv\"\n";
  out.close();
  if (!out) throw std::runtime_error("cannot finish " + path.string());
}

}  // namespace

PreparedBundle prepareBundle(const fs::path& staging,
                             const Config& effective_config) {
  x7::ptp::copyBundleRegularFile(effective_config.source_path,
                                 staging / "source.toml");
  x7::ptp::copyBundleModelDependencies(effective_config.model_path,
                                       staging / "model");
  x7::ptp::copyBundleRegularFile(effective_config.reference_path,
                                 staging / "reference.zvs");
  x7::ptp::copyBundleRegularFile(effective_config.hardware_config_path,
                                 staging / "hardware.toml");
  const auto config_path = staging / "track.toml";
  writeEffectiveConfig(config_path, effective_config);
  return {config_path, staging / "simulation.zvs",
          staging / "simulation.csv", staging / "hardware.csv"};
}

void writeBundleResult(const fs::path& root, const std::string& frontend,
                       const RunResult& result) {
  const auto path = root / "result.toml";
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << std::setprecision(std::numeric_limits<double>::max_digits10)
      << "format = \"rtctrl-x7-track-result\"\nversion = 1\n"
      << "frontend = " << quote(frontend)
      << "\nstatus = " << quote(statusName(result.status))
      << "\ntracking_pass = " << (result.tracking_pass ? "true" : "false")
      << "\ncycles = " << result.cycles
      << "\ntracking_samples = " << result.tracking_samples
      << "\nworst_home_error_rad = " << result.worst_home_error_rad
      << "\naggregate_rms_error_rad = " << result.aggregate_rms_error_rad
      << "\nworst_joint_rms_error_rad = "
      << result.worst_joint_rms_error_rad
      << "\npeak_error_rad = " << result.peak_error_rad
      << "\njoint_rms_error_rad = ";
  writeArray(out, result.joint_rms_error_rad);
  out << "\njoint_peak_error_rad = ";
  writeArray(out, result.joint_peak_error_rad);
  out << '\n';
}

}  // namespace x7::track
