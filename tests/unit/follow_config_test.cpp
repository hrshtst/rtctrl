#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdio>
#include <filesystem>
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

TEST_CASE("follow CLI accepts a timing log on hardware only",
          "[follow][config]") {
  char app[] = "x7_follow";
  char config[] = "--config";
  char path[] = "run.toml";
  char timing[] = "--timing-log";
  char csv[] = "cycles.csv";
  char* argv[] = {app, config, path, timing, csv};
  const auto cli = follow::parseCli(5, argv, false);
  REQUIRE(cli.timing_log_path);
  CHECK(*cli.timing_log_path == std::filesystem::path("cycles.csv"));
  CHECK_THROWS(follow::parseCli(5, argv, true));  // simulation has no ring

  char bundle[] = "--bundle";
  char archive[] = "archive";
  char* bundled[] = {app, config, path, timing, csv, bundle, archive};
  CHECK_THROWS(follow::parseCli(7, bundled, false));
  char* missing[] = {app, config, path, timing};
  CHECK_THROWS(follow::parseCli(4, missing, false));
}

TEST_CASE("follow CLI keeps posture references hardware-only and alone",
          "[follow][config][posture]") {
  char app[] = "x7_follow";
  char config[] = "--config";
  char path[] = "run.toml";
  char hold[] = "--hold-current";
  char seconds[] = "60";
  char sweep[] = "--sweep-current";
  char spec[] = "6:-0.3:6";
  char* hold_argv[] = {app, config, path, hold, seconds};
  CHECK(follow::parseCli(5, hold_argv, false).hold_current ==
        std::string("60"));
  CHECK_THROWS(follow::parseCli(5, hold_argv, true));
  char* sweep_argv[] = {app, config, path, sweep, spec};
  CHECK(follow::parseCli(5, sweep_argv, false).sweep_current ==
        std::string("6:-0.3:6"));
  char* both[] = {app, config, path, hold, seconds, sweep, spec};
  CHECK_THROWS(follow::parseCli(7, both, false));
  char reference[] = "--reference";
  char zvs[] = "ref.zvs";
  char* with_reference[] = {app, config, path, hold, seconds, reference, zvs};
  CHECK_THROWS(follow::parseCli(7, with_reference, false));
  char check[] = "--check";
  char* with_check[] = {app, config, path, hold, seconds, check};
  CHECK_THROWS(follow::parseCli(6, with_check, false));
  char bundle[] = "--bundle";
  char archive[] = "archive";
  char* with_bundle[] = {app, config, path, sweep, spec, bundle, archive};
  CHECK_THROWS(follow::parseCli(7, with_bundle, false));
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

TEST_CASE("follow config keeps the ideal simulated servo by default",
          "[follow][config][servo_like]") {
  const char* path = "build/follow_servo_default.toml";
  writeConfig(path, "[simulation]\nposition_kp = 300\nposition_kd = 2.5\n");
  const auto simulation = follow::loadConfig(path).simulation;
  std::remove(path);
  CHECK(simulation.position_kp == 300.0);  // the scalar form, as before
  CHECK(simulation.position_kd == 2.5);
  CHECK_FALSE(simulation.position_kp_joints);
  CHECK_FALSE(simulation.position_kd_joints);
  CHECK_FALSE(simulation.coulomb_friction_nm);
  CHECK_FALSE(simulation.viscous_friction_nm_s);
  CHECK(simulation.friction_v0_rad_s == 0.05);
  CHECK(simulation.command_delay_cycles == 0);
  CHECK_FALSE(simulation.quantize_encoder);
  CHECK_FALSE(simulation.servoLike());
  CHECK_FALSE(follow::SimulationConfig{}.servoLike());
}

TEST_CASE("follow config reads the servo-like simulation keys",
          "[follow][config][servo_like]") {
  const char* path = "build/follow_servo_like.toml";
  writeConfig(path,
              "[simulation]\n"
              "position_kp = [20, 60, 20.5, 20, 20, 20, 20, 21]\n"
              "position_kd = [0.8, 2.4, 0.8, 0, 0.8, 0.8, 0.8, 0.84]\n"
              "coulomb_friction_nm = 0.1\n"
              "viscous_friction_nm_s = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]\n"
              "friction_v0_rad_s = 0.02\n"
              "command_delay_cycles = 2\n"
              "quantize_encoder = true\n");
  const auto simulation = follow::loadConfig(path).simulation;
  std::remove(path);
  REQUIRE(simulation.position_kp_joints);
  REQUIRE(simulation.position_kd_joints);
  CHECK((*simulation.position_kp_joints)[1] == 60.0);
  CHECK((*simulation.position_kp_joints)[2] == 20.5);
  CHECK((*simulation.position_kp_joints)[7] == 21.0);
  CHECK((*simulation.position_kd_joints)[3] == 0.0);
  CHECK((*simulation.position_kd_joints)[7] == 0.84);
  CHECK(simulation.position_kp == 1000.0);  // the unused scalar default
  REQUIRE(simulation.coulomb_friction_nm);
  for (const double value : *simulation.coulomb_friction_nm) {
    CHECK(value == 0.1);  // one number applies to every joint
  }
  REQUIRE(simulation.viscous_friction_nm_s);
  CHECK((*simulation.viscous_friction_nm_s)[0] == 0.0);
  CHECK((*simulation.viscous_friction_nm_s)[7] == 0.7);
  CHECK(simulation.friction_v0_rad_s == 0.02);
  CHECK(simulation.command_delay_cycles == 2);
  CHECK(simulation.quantize_encoder);
  CHECK(simulation.servoLike());

  // each key alone marks the simulation servo-like (friction_v0 alone
  // changes nothing and does not)
  for (const char* key :
       {"position_kp = [1, 1, 1, 1, 1, 1, 1, 1]",
        "position_kd = [1, 1, 1, 1, 1, 1, 1, 1]", "coulomb_friction_nm = 0",
        "viscous_friction_nm_s = 0", "command_delay_cycles = 1",
        "quantize_encoder = true"}) {
    INFO(key);
    const char* single = "build/follow_servo_single.toml";
    writeConfig(single, std::string("[simulation]\n") + key + "\n");
    CHECK(follow::loadConfig(single).simulation.servoLike());
    std::remove(single);
  }
  const char* v0 = "build/follow_servo_v0.toml";
  writeConfig(v0, "[simulation]\nfriction_v0_rad_s = 0.1\n");
  CHECK_FALSE(follow::loadConfig(v0).simulation.servoLike());
  std::remove(v0);
}

TEST_CASE("follow config rejects invalid servo-like simulation keys",
          "[follow][config][servo_like]") {
  const auto rejects = [](const std::string& line, const char* message) {
    INFO(line);
    const char* path = "build/follow_servo_bad.toml";
    writeConfig(path, "[simulation]\n" + line + "\n");
    CHECK_THROWS_WITH(follow::loadConfig(path),
                      Catch::Matchers::ContainsSubstring(message));
    std::remove(path);
  };
  // wrong lengths (one-element arrays included) and types
  rejects("position_kp = [20, 60, 20, 20, 20, 20, 20]",
          "position_kp needs one number or eight values");
  rejects("position_kd = [0.8]", "position_kd needs one number or eight");
  rejects("coulomb_friction_nm = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]",
          "coulomb_friction_nm needs one number or eight values");
  rejects("viscous_friction_nm_s = \"0.1\"",
          "viscous_friction_nm_s must be a number or eight numbers");
  rejects("position_kp = [20, 60, 20, 20, \"20\", 20, 20, 20]",
          "position_kp must be a number or eight numbers");
  rejects("position_kd = [0.8, 2.4, 0.8, true, 0.8, 0.8, 0.8, 0.8]",
          "position_kd must be a number or eight numbers");
  rejects("position_kp = \"20\"", "position_kp must be finite");  // as before
  rejects("coulomb_friction_nm = true",
          "coulomb_friction_nm must be a number or eight numbers");
  rejects("friction_v0_rad_s = true", "friction_v0_rad_s must be finite");
  // non-finite and out-of-range values
  rejects("position_kp = [20, 60, 20, 20, 20, nan, 20, 20]",
          "position_kp must be finite");
  rejects("position_kd = [0.8, inf, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8]",
          "position_kd must be finite");
  rejects("coulomb_friction_nm = nan", "coulomb_friction_nm must be finite");
  rejects("position_kp = [20, 60, 20, 0, 20, 20, 20, 20]",
          "position_kp must be positive");
  rejects("position_kp = [20, -60, 20, 20, 20, 20, 20, 20]",
          "position_kp must be positive");
  rejects("position_kd = [0.8, 2.4, -0.8, 0.8, 0.8, 0.8, 0.8, 0.8]",
          "position_kd non-negative");
  rejects("position_kp = 0", "invalid simulation gain");  // scalar, as before
  rejects("position_kd = -1", "invalid simulation gain");
  rejects("coulomb_friction_nm = -0.1", "friction must be non-negative");
  rejects("viscous_friction_nm_s = [0, 0, 0, 0, 0, 0, 0, -0.1]",
          "friction must be non-negative");
  rejects("friction_v0_rad_s = 0", "positive friction_v0_rad_s");
  rejects("friction_v0_rad_s = -0.05", "positive friction_v0_rad_s");
  rejects("friction_v0_rad_s = inf", "friction_v0_rad_s must be finite");
  rejects("command_delay_cycles = -1",
          "command_delay_cycles must be non-negative");
  rejects("command_delay_cycles = 1.5",
          "command_delay_cycles must be an integer");
  rejects("command_delay_cycles = true",
          "command_delay_cycles must be an integer");
  rejects("quantize_encoder = 1", "quantize_encoder must be boolean");
  // an unknown key is still rejected
  rejects("command_delay = 1", "unknown key 'command_delay' in simulation");
}

TEST_CASE("follow config resolves an optional simulation link force",
          "[follow][config][link_force]") {
  const char* absent = "build/follow_no_link_force.toml";
  writeConfig(absent);
  CHECK_FALSE(follow::loadConfig(absent).simulation.link_force_path);
  std::remove(absent);

  const char* relative = "build/follow_link_force.toml";
  writeConfig(relative, "[simulation]\nlink_force = \"push.toml\"\n");
  const auto config = follow::loadConfig(relative);
  REQUIRE(config.simulation.link_force_path);
  CHECK(*config.simulation.link_force_path ==
        std::filesystem::absolute("build/push.toml").lexically_normal());
  std::remove(relative);

  const char* empty = "build/follow_empty_link_force.toml";
  writeConfig(empty, "[simulation]\nlink_force = \"\"\n");
  CHECK_THROWS_WITH(follow::loadConfig(empty),
                    Catch::Matchers::ContainsSubstring(
                        "link_force must be a non-empty path"));
  std::remove(empty);

  const char* number = "build/follow_numeric_link_force.toml";
  writeConfig(number, "[simulation]\nlink_force = 3\n");
  CHECK_THROWS_WITH(follow::loadConfig(number),
                    Catch::Matchers::ContainsSubstring(
                        "link_force must be a non-empty path"));
  std::remove(number);
}

TEST_CASE("follow config reads an optional learned command offset",
          "[follow][config][learning]") {
  const char* absent = "build/follow_no_offset.toml";
  writeConfig(absent);
  {
    const auto config = follow::loadConfig(absent);
    CHECK_FALSE(config.learning.offset_path);
    CHECK(config.learning.max_offset_rad == 0.15);
    CHECK(config.learning.max_offset_rate_rad_s == 0.5);
  }
  std::remove(absent);

  // Resolved against the config's directory, like every other input.
  const char* relative = "build/follow_offset.toml";
  writeConfig(relative, "[learning]\noffset = \"trial1.csv\"\n");
  {
    const auto config = follow::loadConfig(relative);
    REQUIRE(config.learning.offset_path);
    CHECK(*config.learning.offset_path ==
          std::filesystem::absolute("build/trial1.csv").lexically_normal());
    CHECK(config.learning.max_offset_rad == 0.15);
    CHECK(config.learning.max_offset_rate_rad_s == 0.5);
  }
  std::remove(relative);

  const char* lowered = "build/follow_offset_lowered.toml";
  writeConfig(lowered,
              "[learning]\noffset = \"/abs/trial2.csv\"\n"
              "max_offset_rad = 0.05\nmax_offset_rate_rad_s = 0.2\n");
  {
    const auto config = follow::loadConfig(lowered);
    REQUIRE(config.learning.offset_path);
    CHECK(*config.learning.offset_path == "/abs/trial2.csv");
    CHECK(config.learning.max_offset_rad == 0.05);
    CHECK(config.learning.max_offset_rate_rad_s == 0.2);
  }
  std::remove(lowered);

  // The defaults themselves are allowed; limits without an offset too (the
  // offset may come from --offset).
  const char* ceiling = "build/follow_offset_ceiling.toml";
  writeConfig(ceiling,
              "[learning]\nmax_offset_rad = 0.15\n"
              "max_offset_rate_rad_s = 0.5\n");
  {
    const auto config = follow::loadConfig(ceiling);
    CHECK_FALSE(config.learning.offset_path);
    CHECK(config.learning.max_offset_rad == 0.15);
    CHECK(config.learning.max_offset_rate_rad_s == 0.5);
  }
  std::remove(ceiling);
  // The rate limit's floor itself is allowed.
  const char* floor = "build/follow_offset_floor.toml";
  writeConfig(floor, "[learning]\nmax_offset_rate_rad_s = 0.01\n");
  CHECK(follow::loadConfig(floor).learning.max_offset_rate_rad_s == 0.01);
  std::remove(floor);

  const auto refused = [](const std::string& table, const char* expected) {
    const char* path = "build/follow_offset_refused.toml";
    writeConfig(path, "[learning]\n" + table);
    CAPTURE(table);
    CHECK_THROWS_WITH(follow::loadConfig(path),
                      Catch::Matchers::ContainsSubstring(expected));
    std::remove(path);
  };
  // Raising a limit is a config error, however slightly.
  refused("max_offset_rad = 0.15000000000000002\n",
          "learning.max_offset_rad must be > 0 and at most 0.15 rad (the "
          "limit can only be lowered)");
  refused("max_offset_rad = 0.2\n", "can only be lowered");
  refused("max_offset_rate_rad_s = 0.5000000000000001\n",
          "learning.max_offset_rate_rad_s must be > 0 and at most 0.5 "
          "rad/s (the limit can only be lowered)");
  refused("max_offset_rate_rad_s = 1.0\n", "can only be lowered");
  refused("max_offset_rad = 0.0\n", "learning.max_offset_rad must be > 0");
  refused("max_offset_rate_rad_s = -0.1\n",
          "learning.max_offset_rate_rad_s must be > 0");
  // A rate near zero would make the offset ramp arbitrarily long.
  refused("max_offset_rate_rad_s = 0.009999999999999998\n",
          "learning.max_offset_rate_rad_s must be at least 0.01 rad/s (the "
          "offset ramp runs at this rate)");
  refused("max_offset_rate_rad_s = 1e-300\n", "must be at least 0.01 rad/s");
  refused("max_offset_rad = nan\n", "learning.max_offset_rad must be finite");
  refused("max_offset_rad = \"0.1\"\n",
          "learning.max_offset_rad must be finite");
  refused("offset = \"\"\n", "offset must be a non-empty path");
  refused("offset = 3\n", "offset must be a non-empty path");
  refused("offst = \"trial1.csv\"\n", "unknown key 'offst' in learning");
  refused("max_offset_rad_s = 0.1\n",
          "unknown key 'max_offset_rad_s' in learning");

  const char* scalar = "build/follow_offset_scalar.toml";
  writeConfig(scalar, "learning = \"trial1.csv\"\n");
  CHECK_THROWS_WITH(follow::loadConfig(scalar),
                    Catch::Matchers::ContainsSubstring(
                        "learning must be a table"));
  std::remove(scalar);
}

TEST_CASE("follow CLI --offset overrides the configured offset path",
          "[follow][config][learning]") {
  const char* path = "build/follow_offset_cli.toml";
  writeConfig(path,
              "[learning]\noffset = \"configured.csv\"\n"
              "max_offset_rad = 0.1\n");
  char app[] = "x7_follow_sim";
  char config_arg[] = "--config";
  char config_path[] = "build/follow_offset_cli.toml";
  char offset_arg[] = "--offset";
  char offset_path[] = "trial3.csv";
  char* argv[] = {app, config_arg, config_path, offset_arg, offset_path};
  for (const bool simulation : {true, false}) {
    const auto cli = follow::parseCli(5, argv, simulation);
    REQUIRE(cli.offset_path);
    CHECK(*cli.offset_path == "trial3.csv");
    auto config = follow::loadConfig(cli.config_path);
    follow::applyOverrides(cli, &config);
    REQUIRE(config.learning.offset_path);
    // Resolved against the working directory, like --reference.
    CHECK(*config.learning.offset_path ==
          std::filesystem::absolute("trial3.csv").lexically_normal());
    CHECK(config.learning.max_offset_rad == 0.1);  // the limits stay
  }
  // Without --offset the configured path stays.
  {
    const auto cli = follow::parseCli(3, argv, true);
    CHECK_FALSE(cli.offset_path);
    auto config = follow::loadConfig(cli.config_path);
    follow::applyOverrides(cli, &config);
    REQUIRE(config.learning.offset_path);
    CHECK(config.learning.offset_path->filename() == "configured.csv");
  }
  char* missing[] = {app, config_arg, config_path, offset_arg};
  CHECK_THROWS_WITH(follow::parseCli(4, missing, true),
                    Catch::Matchers::ContainsSubstring(
                        "--offset requires a value"));
  std::remove(path);
}
