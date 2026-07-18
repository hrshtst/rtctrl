#include "rtctrl/hw/config.hpp"

#include <toml++/toml.hpp>

#include <stdexcept>

#include "rtctrl/dxl/control_table.hpp"

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
    joint.id = static_cast<std::uint8_t>(*id);
    const auto model = (*t)["model"].value_or(std::string{"XM430-W350"});
    joint.model_number = (model == "XM540-W270") ? dxl::kModelXm540W270
                                                 : dxl::kModelXm430W350;
    joint.operating_mode =
        static_cast<std::uint8_t>((*t)["operating_mode"].value_or(3));
    joint.velocity_limit = (*t)["velocity_limit"].value_or(0.0);
    joint.effort_limit = (*t)["effort_limit"].value_or(0.0);
    joint.pos_limit_margin = (*t)["pos_limit_margin"].value_or(0.0);
    joint.current_limit_margin = (*t)["current_limit_margin"].value_or(0.0);
    config.joints.push_back(std::move(joint));
  }
  return config;
}

}  // namespace rtctrl::hw
