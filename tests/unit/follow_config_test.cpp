#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>

#include "follow/follow_config.hpp"

namespace follow = x7::follow;
namespace arm = rtctrl::arm;

namespace {

const char* writeConfig(const char* path, const std::string& extra = {}) {
  std::ofstream out(path);
  out << "format = \"rtctrl-x7-follow\"\n"
      << "version = 1\n"
      << "model = \"../../models/crane_x7/crane_x7.ztk\"\n"
      << "reference = \"reference.zvs\"\n"
      << "hardware_config = \"../../config/crane_x7.toml\"\n"
      << extra;
  return path;
}

}  // namespace

TEST_CASE("follow config applies safe defaults and resolves inputs",
          "[follow][config]") {
  const char* path = "build/follow_default.toml";
  writeConfig(path);
  const auto config = follow::loadConfig(path);
  CHECK(config.control_rate_hz == 100.0);
  CHECK(config.mode == arm::ControlMode::Position);
  CHECK(config.home.strict);
  CHECK(config.home.tolerance_rad == 0.01);
  CHECK(config.finalization.wait_time_s == 0.0);
  CHECK(config.finalization.operator_timeout_s == 60.0);
  CHECK(config.model_path.filename() == "crane_x7.ztk");
  std::remove(path);
}

TEST_CASE("current-based position requires an explicit effort ceiling",
          "[follow][config]") {
  const char* missing = "build/follow_cbp_missing.toml";
  writeConfig(missing, "[control]\nmode = \"current-based-position\"\n");
  CHECK_THROWS(follow::loadConfig(missing));
  std::remove(missing);

  const char* valid = "build/follow_cbp.toml";
  writeConfig(valid,
              "[control]\nmode = \"current-based-position\"\n"
              "[current_based_position]\neffort_limit_nm = [1.2]\n");
  const auto config = follow::loadConfig(valid);
  CHECK(config.effort_limit_set);
  CHECK(config.effort_limit_nm[7] == 1.2);
  std::remove(valid);
}

TEST_CASE("follow config rejects unsafe rates, threshold order, and typos",
          "[follow][config]") {
  const char* rate = "build/follow_bad_rate.toml";
  writeConfig(rate, "[control]\nrate_hz = 201\n");
  CHECK_THROWS(follow::loadConfig(rate));
  std::remove(rate);

  const char* safety = "build/follow_bad_safety.toml";
  writeConfig(safety,
              "[safety]\nwarning_error_rad = 0.8\n"
              "sustained_abort_error_rad = 0.5\n");
  CHECK_THROWS(follow::loadConfig(safety));
  std::remove(safety);

  const char* typo = "build/follow_typo.toml";
  writeConfig(typo, "contrl = 5\n");
  CHECK_THROWS(follow::loadConfig(typo));
  std::remove(typo);
}

TEST_CASE("follow CLI requires explicit config and separates frontends",
          "[follow][config]") {
  char app[] = "x7_follow_sim";
  char config[] = "--config";
  char path[] = "run.toml";
  char motion[] = "--motion";
  char zvs[] = "out.zvs";
  char* argv[] = {app, config, path, motion, zvs};
  const auto cli = follow::parseCli(5, argv, true);
  CHECK(cli.config_path == "run.toml");
  CHECK(cli.motion_path == std::filesystem::path("out.zvs"));
  CHECK_THROWS(follow::parseCli(5, argv, false));
  CHECK_THROWS(follow::parseCli(1, argv, true));
}
