#include "rtctrl/model/posture.hpp"

#include <toml++/toml.hpp>

#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace rtctrl::model {

Posture loadPostureToml(const std::string& path) {
  toml::table root;
  try {
    root = toml::parse_file(path);
  } catch (const toml::parse_error& error) {
    throw std::runtime_error("Posture: cannot parse '" + path + "': " +
                             std::string(error.description()));
  }

  const std::unordered_set<std::string> allowed = {
      "format_version", "name", "joint_positions"};
  for (const auto& [key, value] : root) {
    (void)value;
    if (allowed.count(std::string(key.str())) == 0) {
      throw std::runtime_error("Posture: unknown key '" +
                               std::string(key.str()) + "'");
    }
  }

  const auto version = root["format_version"].value<int>();
  if (!version || *version != kPostureFormatVersion) {
    throw std::runtime_error(
        "Posture: format_version must be " +
        std::to_string(kPostureFormatVersion));
  }

  const auto name = root["name"].value<std::string>();
  if (!name || name->empty()) {
    throw std::runtime_error("Posture: name must be a non-empty string");
  }

  const auto* positions = root["joint_positions"].as_array();
  if (positions == nullptr || positions->size() != kCanonicalDof) {
    throw std::runtime_error("Posture: joint_positions must contain exactly " +
                             std::to_string(kCanonicalDof) + " numbers");
  }

  Posture posture;
  posture.name = *name;
  for (int i = 0; i < kCanonicalDof; ++i) {
    const auto value = (*positions)[i].value<double>();
    if (!value || !std::isfinite(*value)) {
      throw std::runtime_error(
          "Posture: joint_positions must contain only finite numbers");
    }
    posture.joint_positions[i] = *value;
  }
  return posture;
}

}  // namespace rtctrl::model
