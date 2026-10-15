#include "rtctrl/model/link_force.hpp"

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <toml++/toml.hpp>

namespace rtctrl::model {

namespace {

bool finite3(const std::array<double, 3>& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument("link force: " + message);
  }
}

[[noreturn]] void fail(const std::string& path, const std::string& message) {
  throw std::runtime_error("link force '" + path + "': " + message);
}

void rejectUnknown(const toml::table& table,
                   std::initializer_list<const char*> allowed,
                   const std::string& where, const std::string& path) {
  for (const auto& [key, value] : table) {
    (void)value;
    bool known = false;
    for (const char* name : allowed) known = known || key.str() == name;
    if (!known) {
      fail(path, "unknown key '" + std::string(key.str()) + "' in " + where);
    }
  }
}

double requiredNumber(const toml::table& table, const char* key,
                      const std::string& name, const std::string& path) {
  const auto value = table[key].value<double>();
  if (!value) fail(path, name + " must be a number");
  if (!std::isfinite(*value)) fail(path, name + " must be finite");
  return *value;
}

std::array<double, 3> vector3(const toml::table& table, const char* key,
                              const std::string& path) {
  const auto* array = table[key].as_array();
  if (array == nullptr || array->size() != 3) {
    fail(path, std::string(key) + " must be an array of three numbers");
  }
  std::array<double, 3> result{};
  for (std::size_t i = 0; i < 3; ++i) {
    const auto value = (*array)[i].value<double>();
    if (!value || !std::isfinite(*value)) {
      fail(path, std::string(key) + " must contain three finite numbers");
    }
    result[i] = *value;
  }
  return result;
}

}  // namespace

double ForceProfile::end_s() const {
  return onset_s + rise_s + hold_s + fall_s;
}

double ForceProfile::scale(double t) const {
  // Written so that NaN t falls through every comparison to zero.
  if (!(t >= onset_s)) return 0.0;
  const double rise_end = onset_s + rise_s;
  if (t < rise_end) {
    return 0.5 * (1.0 - std::cos(M_PI * (t - onset_s) / rise_s));
  }
  const double hold_end = rise_end + hold_s;
  if (t <= hold_end) return 1.0;
  if (t < end_s()) {
    return 0.5 * (1.0 + std::cos(M_PI * (t - hold_end) / fall_s));
  }
  return 0.0;
}

double ForceProfile::impulseScale() const {
  return 0.5 * rise_s + hold_s + 0.5 * fall_s;
}

void validateLinkForce(const LinkForceSpec& spec) {
  require(!spec.link.empty(), "link name must not be empty");
  require(finite3(spec.point_link_m), "point_link_m must be finite");
  require(std::isfinite(spec.magnitude_n) && spec.magnitude_n >= 0.0,
          "magnitude_n must be finite and >= 0");
  const auto& profile = spec.profile;
  require(std::isfinite(profile.onset_s) && profile.onset_s >= 0.0,
          "profile.onset_s must be finite and >= 0");
  require(std::isfinite(profile.rise_s) && profile.rise_s > 0.0,
          "profile.rise_s must be finite and > 0");
  require(std::isfinite(profile.hold_s) && profile.hold_s >= 0.0,
          "profile.hold_s must be finite and >= 0");
  require(std::isfinite(profile.fall_s) && profile.fall_s > 0.0,
          "profile.fall_s must be finite and > 0");
  require(std::isfinite(profile.end_s()),
          "profile end time (onset + rise + hold + fall) must be finite");
  switch (spec.direction) {
    case ForceDirection::World:
      require(finite3(spec.direction_world) &&
                  std::hypot(spec.direction_world[0], spec.direction_world[1],
                             spec.direction_world[2]) >= kMinDirectionNorm,
              "direction_world must be finite and nonzero");
      break;
    case ForceDirection::Anchor:
      require(finite3(spec.anchor_world_m), "anchor_world_m must be finite");
      break;
    default:
      require(false, "unknown direction mode");
  }
}

bool linkForceDirection(const LinkForceSpec& spec,
                        const std::array<double, 3>& point_world,
                        std::array<double, 3>* unit) {
  std::array<double, 3> v{};
  double minimum = 0.0;
  if (spec.direction == ForceDirection::World) {
    v = spec.direction_world;
    minimum = kMinDirectionNorm;
  } else if (spec.direction == ForceDirection::Anchor) {
    for (int i = 0; i < 3; ++i) v[i] = spec.anchor_world_m[i] - point_world[i];
    minimum = kAnchorMinDistanceM;
  } else {
    return false;
  }
  const double norm = std::hypot(v[0], v[1], v[2]);
  // NaN norms fail the comparison as well.
  if (!(norm >= minimum) || !std::isfinite(norm)) return false;
  for (int i = 0; i < 3; ++i) (*unit)[i] = v[i] / norm;
  return true;
}

LinkForceSpec loadLinkForce(const std::string& path) {
  toml::table root;
  try {
    root = toml::parse_file(path);
  } catch (const toml::parse_error& error) {
    fail(path, "cannot parse: " + std::string(error.description()));
  }
  rejectUnknown(root,
                {"format", "version", "link", "point_link_m", "magnitude_n",
                 "direction_world", "anchor_world_m", "profile"},
                "root", path);
  if (root["format"].value_or(std::string{}) != kLinkForceFormat ||
      root["version"].value_or<std::int64_t>(0) != kLinkForceFormatVersion) {
    fail(path, std::string("format must be ") + kLinkForceFormat + " version " +
                   std::to_string(kLinkForceFormatVersion));
  }

  LinkForceSpec spec;
  const auto link = root["link"].value<std::string>();
  if (!link || link->empty()) fail(path, "link must be a non-empty string");
  spec.link = *link;
  spec.point_link_m = vector3(root, "point_link_m", path);
  spec.magnitude_n = requiredNumber(root, "magnitude_n", "magnitude_n", path);

  const bool has_direction = root.contains("direction_world");
  const bool has_anchor = root.contains("anchor_world_m");
  if (has_direction == has_anchor) {
    fail(path, "set exactly one of direction_world or anchor_world_m");
  }
  if (has_direction) {
    spec.direction = ForceDirection::World;
    spec.direction_world = vector3(root, "direction_world", path);
  } else {
    spec.direction = ForceDirection::Anchor;
    spec.anchor_world_m = vector3(root, "anchor_world_m", path);
  }

  const auto* profile = root["profile"].as_table();
  if (profile == nullptr) fail(path, "missing [profile] table");
  rejectUnknown(*profile, {"onset_s", "rise_s", "hold_s", "fall_s"},
                "[profile]", path);
  spec.profile.onset_s =
      requiredNumber(*profile, "onset_s", "profile.onset_s", path);
  spec.profile.rise_s =
      requiredNumber(*profile, "rise_s", "profile.rise_s", path);
  spec.profile.hold_s =
      requiredNumber(*profile, "hold_s", "profile.hold_s", path);
  spec.profile.fall_s =
      requiredNumber(*profile, "fall_s", "profile.fall_s", path);

  try {
    validateLinkForce(spec);
  } catch (const std::invalid_argument& error) {
    fail(path, error.what());
  }
  return spec;
}

}  // namespace rtctrl::model
