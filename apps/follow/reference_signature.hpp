// The reference signature: what a learned command offset (and feedforward
// weights) is bound to. The follower commands the reference after its
// [reference_processing] (filter, cut-off, interpolation) on the tracking
// grid of control.rate_hz, in control.mode; an offset learned under one of
// these settings is the wrong offset under another, although the reference
// file is the same. The signature covers the file and those settings, and
// it is what the reference_sha256 field of an offset or weights file holds
// (the learning library only stores and compares it). x7_follow,
// x7_follow_sim and x7_follow_learn all compute it here, from the
// effective config: the frontends after their command-line overrides
// (--reference, --mode, --filter, --interpolation), x7_follow_learn from
// the follow config file alone. An override that changes a bound setting
// therefore makes the follower refuse an offset learned without it.
//
// The signature is the lowercase hex SHA-256 (x7::ptp::sha256Text) of this
// canonical text, one "key value" line each, every line ending in "\n":
//
//   rtctrl-x7-follow-reference-signature 1
//   reference_file_sha256 <the reference file's sha256File, 64 hex>
//   control.rate_hz <control_rate_hz, %.17g>
//   control.mode <the arm::ControlMode raw integer: 3 position, 5
//                 current-based position>
//   interpolation <linear | shape-preserving-cubic>
//   filter <none | low-pass | moving-average | savitzky-golay>
//   low_pass_cutoff_hz <%.17g>
//   filter_window <integer>
//   savitzky_golay_order <integer>
//   mimic_tolerance <%.17g>
//
// The last six are every field of model::ZvsTrajectoryOptions (the
// struct loadConfig fills from [reference_processing]) by its name, enums
// by their config names, each included whether or not the filter uses it.
// The first line names the layout and its version.
//
// Not bound, by decision: the servo gains (the servos' own registers, a
// motor_parameters file, or the simulation's [simulation] keys) and the
// model. Neither are the current-based position effort limits, nor the
// homing, safety and finalization settings, which do not change the
// tracked trajectory. Running an offset with the servo and model it was
// learned with is the operator's responsibility.
#pragma once

#include <cstdio>
#include <string>

#include "follow/follow_config.hpp"
#include "plan/ptp_bundle.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace x7::follow {

inline constexpr const char* kReferenceSignatureHeader =
    "rtctrl-x7-follow-reference-signature 1";

// For refusal messages: what the signature covers.
inline constexpr const char* kReferenceSignatureCovers =
    "the reference file's SHA-256, control.rate_hz, control.mode and every "
    "[reference_processing] setting";

// The canonical text above, for the reference file whose sha256File is
// reference_file_sha256.
inline std::string referenceSignatureText(
    const Config& config, const std::string& reference_file_sha256) {
  const auto g17 = [](double value) {
    char text[40];
    std::snprintf(text, sizeof text, "%.17g", value);
    return std::string(text);
  };
  // Exactly the fields of ZvsTrajectoryOptions: this binding stops
  // compiling when the struct gains or loses one, so a new reference
  // processing setting cannot be left out of the signature unnoticed.
  const auto& [interpolation, filter, low_pass_cutoff_hz, filter_window,
               savitzky_golay_order, mimic_tolerance] = config.reference;
  std::string text;
  const auto line = [&text](const char* key, const std::string& value) {
    text += key;
    text += ' ';
    text += value;
    text += '\n';
  };
  text += kReferenceSignatureHeader;
  text += '\n';
  line("reference_file_sha256", reference_file_sha256);
  line("control.rate_hz", g17(config.control_rate_hz));
  line("control.mode", std::to_string(static_cast<int>(config.mode)));
  line("interpolation", referenceInterpolationName(interpolation));
  line("filter", referenceFilterName(filter));
  line("low_pass_cutoff_hz", g17(low_pass_cutoff_hz));
  line("filter_window", std::to_string(filter_window));
  line("savitzky_golay_order", std::to_string(savitzky_golay_order));
  line("mimic_tolerance", g17(mimic_tolerance));
  return text;
}

inline std::string referenceSignature(
    const Config& config, const std::string& reference_file_sha256) {
  return x7::ptp::sha256Text(
      referenceSignatureText(config, reference_file_sha256));
}

// Hashes config.reference_path itself. Throws std::runtime_error if the
// file cannot be read.
inline std::string referenceSignature(const Config& config) {
  return referenceSignature(config, x7::ptp::sha256File(config.reference_path));
}

}  // namespace x7::follow
