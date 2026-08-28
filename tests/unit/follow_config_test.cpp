#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdio>
#include <fstream>

#include "follow/follow_config.hpp"
#include "follow/follow_hardware.hpp"

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

TEST_CASE("follow config rejects velocity mode without a host position loop",
          "[follow][config]") {
  const char* path = "build/follow_velocity.toml";
  writeConfig(path, "[control]\nmode = \"velocity\"\n");
  CHECK_THROWS_WITH(follow::loadConfig(path),
                    Catch::Matchers::ContainsSubstring("host-side"));
  std::remove(path);
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

  char bundle[] = "--bundle";
  char archive[] = "archive";
  char check[] = "--check";
  char* incompatible[] = {app, config, path, bundle, archive, check};
  CHECK_THROWS(follow::parseCli(6, incompatible, true));

  char log[] = "--log";
  char csv[] = "out.csv";
  char* owned[] = {app, config, path, bundle, archive, log, csv};
  CHECK_THROWS(follow::parseCli(7, owned, true));
}

TEST_CASE("follow CLI overrides comparison inputs and control policy",
          "[follow][config]") {
  char app[] = "x7_follow_sim";
  char config_arg[] = "--config";
  char config_path[] = "run.toml";
  char reference[] = "--reference";
  char reference_path[] = "motion.zvs";
  char mode[] = "--mode";
  char cbp[] = "current-based-position";
  char effort[] = "--effort-limit-nm";
  char effort_value[] = "2.5";
  char parameters[] = "--motor-parameters";
  char parameters_path[] = "motors.toml";
  char filter[] = "--filter";
  char low_pass[] = "low-pass";
  char interpolation[] = "--interpolation";
  char linear[] = "linear";
  char* argv[] = {app,
                  config_arg,
                  config_path,
                  reference,
                  reference_path,
                  mode,
                  cbp,
                  effort,
                  effort_value,
                  parameters,
                  parameters_path,
                  filter,
                  low_pass,
                  interpolation,
                  linear};
  const auto cli = follow::parseCli(15, argv, true);
  follow::Config effective;
  follow::applyOverrides(cli, &effective);
  CHECK(effective.mode == arm::ControlMode::CurrentBasedPosition);
  CHECK(effective.effort_limit_set);
  CHECK(effective.effort_limit_nm[7] == 2.5);
  CHECK(effective.reference.filter ==
        rtctrl::model::ReferenceFilter::LowPass);
  CHECK(effective.reference.interpolation ==
        rtctrl::model::ReferenceInterpolation::Linear);
  CHECK(effective.reference_path.is_absolute());
  CHECK(effective.reference_path.filename() == "motion.zvs");
  REQUIRE(effective.motor_parameters_path);
  CHECK(effective.motor_parameters_path->is_absolute());
}

TEST_CASE("follow CLI validates comparison override values",
          "[follow][config]") {
  follow::Config config;
  follow::Cli cli;
  cli.mode = arm::ControlMode::CurrentBasedPosition;
  CHECK_THROWS(follow::applyOverrides(cli, &config));

  cli.effort_limit_nm = 0.0;
  CHECK_THROWS(follow::applyOverrides(cli, &config));

  char app[] = "x7_follow";
  char config_arg[] = "--config";
  char config_path[] = "run.toml";
  char filter[] = "--filter";
  char invalid[] = "butterworth";
  char* argv[] = {app, config_arg, config_path, filter, invalid};
  CHECK_THROWS(follow::parseCli(5, argv, false));
}

TEST_CASE("hardware follow timing and mode derive from one control contract",
          "[follow][hardware]") {
  follow::Config config;
  config.control_rate_hz = 200.0;
  config.mode = arm::ControlMode::CurrentBasedPosition;
  auto hardware = rtctrl::hw::Config::load("config/crane_x7.toml");
  follow::selectHardwareMode(&hardware, config.mode,
                             std::string("/dev/follow-test"));
  const auto options = follow::hardwareOptions(config, std::nullopt);
  CHECK(hardware.port == "/dev/follow-test");
  for (const auto& joint : hardware.joints) {
    CHECK(joint.operating_mode == 5);
  }
  CHECK(options.control_cycle_s == 0.005);
  CHECK(options.controller_deadline_s == 0.005);
  CHECK(options.controller_write_margin_s == 0.001);
}

TEST_CASE("hardware follow refuses a parameter dump that owns mode",
          "[follow][hardware][parameters]") {
  auto hardware = rtctrl::hw::Config::load("config/crane_x7.toml");
  rtctrl::apps::dxl_parameters::ParameterDump dump;
  rtctrl::apps::dxl_parameters::MotorRecord motor;
  motor.id = hardware.joints.front().id;
  motor.model_number = hardware.joints.front().model_number;
  const auto* operating_mode =
      rtctrl::apps::dxl_parameters::findParameter("operating_mode");
  REQUIRE(operating_mode != nullptr);
  motor.parameters.push_back({operating_mode, 3});
  dump.motors.push_back(motor);
  CHECK_THROWS_WITH(follow::validateMotorParameters(dump, hardware),
                    Catch::Matchers::ContainsSubstring("must omit"));

  dump.motors.front().parameters.clear();
  dump.motors.front().id = 42;
  CHECK_THROWS_WITH(follow::validateMotorParameters(dump, hardware),
                    Catch::Matchers::ContainsSubstring("unconfigured"));
}
