#include "rtctrl/hw/config.hpp"

#include <toml++/toml.hpp>

#include <stdexcept>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::hw {

Config Config::load(const std::string& toml_path) {
  Config config;
  toml::table tbl;
  try {
    tbl = toml::parse_file(toml_path);
  } catch (const toml::parse_error& e) {
    throw std::runtime_error("Config: cannot parse '" + toml_path +
                             "': " + std::string(e.description()));
  }

  if (const auto* bus = tbl["bus"].as_table()) {
    config.port = (*bus)["port"].value_or(config.port);
    config.baudrate = (*bus)["baudrate"].value_or(config.baudrate);
  }

  const auto* joints = tbl["joint"].as_array();
  if (joints == nullptr || joints->empty()) {
    throw std::runtime_error("Config: '" + toml_path +
                             "' has no [[joint]] entries");
  }
  for (const auto& entry : *joints) {
    const auto* t = entry.as_table();
    if (t == nullptr) continue;
    JointConfig joint;
    joint.name = (*t)["name"].value_or(std::string{});
    const auto id = (*t)["id"].value<int>();
    if (joint.name.empty() || !id) {
      throw std::runtime_error("Config: joint entry missing name or id");
    }
    // Range-check BEFORE narrowing: id 258 must not wrap into a valid 2.
    if (*id < 1 || *id > 252) {
      throw std::runtime_error("Config: joint '" + joint.name +
                               "' has out-of-range id " +
                               std::to_string(*id));
    }
    joint.id = static_cast<std::uint8_t>(*id);
    const int mode = (*t)["operating_mode"].value_or(3);
    if (mode != 0 && mode != 1 && mode != 3) {
      throw std::runtime_error(
          "Config: joint '" + joint.name + "' has invalid operating_mode " +
          std::to_string(mode) + " (expected 0, 1 or 3)");
    }
    joint.operating_mode = static_cast<std::uint8_t>(mode);
    const auto model = (*t)["model"].value_or(std::string{"XM430-W350"});
    if (model == "XM430-W350") {
      joint.model_number = dxl::kModelXm430W350;
    } else if (model == "XM540-W270") {
      joint.model_number = dxl::kModelXm540W270;
    } else {
      throw std::runtime_error("Config: unknown servo model '" + model +
                               "' on joint '" + joint.name +
                               "' (expected XM430-W350 or XM540-W270)");
    }
    joint.velocity_limit = (*t)["velocity_limit"].value_or(0.0);
    joint.effort_limit = (*t)["effort_limit"].value_or(0.0);
    joint.pos_limit_margin = (*t)["pos_limit_margin"].value_or(0.0);
    joint.current_limit_margin = (*t)["current_limit_margin"].value_or(0.0);
    config.joints.push_back(std::move(joint));
  }

  config.validate();
  return config;
}

void Config::validate() const {
  // The joints vector IS the canonical order, and every consumer
  // (JointMap, RealArm, the sync group, the limit arrays) assumes it.
  // A silently tolerated deviation would map controller outputs onto
  // the wrong servos — reject loudly instead. Called by load() and
  // re-checked by the CraneX7 constructor, since Config is a plain
  // struct that callers can also build or mutate by hand.
  const auto& canonical = model::canonicalJoints();
  if (joints.size() != canonical.size()) {
    throw std::runtime_error(
        "Config: expected " + std::to_string(canonical.size()) +
        " [[joint]] entries in canonical order, got " +
        std::to_string(joints.size()));
  }
  for (std::size_t i = 0; i < canonical.size(); ++i) {
    const auto& joint = joints[i];
    if (joint.name != canonical[i].urdf_joint) {
      throw std::runtime_error(
          "Config: joint " + std::to_string(i) + " is '" + joint.name +
          "' but the canonical order requires '" + canonical[i].urdf_joint +
          "'");
    }
    if (joint.id != canonical[i].dxl_id) {
      throw std::runtime_error(
          "Config: joint '" + joint.name + "' has id " +
          std::to_string(joint.id) + " but the canonical mapping requires " +
          std::to_string(canonical[i].dxl_id));
    }
    if (joint.model_number != dxl::kModelXm430W350 &&
        joint.model_number != dxl::kModelXm540W270) {
      // torqueConstant() would otherwise silently treat an unknown
      // model as an XM430 — wrong torque scaling on a hand-built config
      throw std::runtime_error(
          "Config: joint '" + joint.name + "' has unknown model_number " +
          std::to_string(joint.model_number));
    }
    if (joint.operating_mode != 0 && joint.operating_mode != 1 &&
        joint.operating_mode != 3) {
      throw std::runtime_error(
          "Config: joint '" + joint.name + "' has invalid operating_mode " +
          std::to_string(joint.operating_mode) + " (expected 0, 1 or 3)");
    }
    if (joint.velocity_limit <= 0.0 || joint.effort_limit <= 0.0) {
      throw std::runtime_error(
          "Config: joint '" + joint.name +
          "' needs positive velocity_limit and effort_limit — they gate "
          "every command");
    }
    if (joint.pos_limit_margin < 0.0 || joint.current_limit_margin < 0.0) {
      throw std::runtime_error("Config: joint '" + joint.name +
                               "' has a negative safety margin");
    }
  }
}

}  // namespace rtctrl::hw
