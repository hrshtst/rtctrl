#pragma once

#include <array>
#include <string>

#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::model {

inline constexpr int kPostureFormatVersion = 1;

struct Posture {
  std::string name;
  std::array<double, kCanonicalDof> joint_positions{};
};

// Loads an authored posture from strict, versioned TOML. Unknown keys,
// incompatible versions, malformed values, and non-finite positions throw.
Posture loadPostureToml(const std::string& path);

}  // namespace rtctrl::model
