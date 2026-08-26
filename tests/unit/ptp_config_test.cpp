#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "plan/ptp_config.hpp"

using Catch::Approx;

namespace {

namespace fs = std::filesystem;

const char* kMinimalConfig = R"(
model = "../../models/crane_x7/crane_x7.ztk"

[start]
position = [0.2, 0.0, 0.25]
rpy_rad = [0.0, 0.0, 0.0]

[end]
position = [0.21, 0.0, 0.25]
rpy_rad = [0.0, 0.0, 0.0]
)";

class ConfigFile {
 public:
  explicit ConfigFile(const std::string& content) {
    fs::create_directories("build/ptp_config_test");
    path_ = "build/ptp_config_test/plan.toml";
    std::ofstream output(path_);
    output << content;
  }
  ~ConfigFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

}  // namespace

TEST_CASE("PTP config applies defaults and resolves its model path",
          "[ptp][config]") {
  ConfigFile file(kMinimalConfig);
  const auto config = x7::ptp::loadConfig(file.path());
  CHECK(config.model_path ==
        fs::absolute("models/crane_x7/crane_x7.ztk").lexically_normal());
  CHECK(config.output_path == fs::path("ptp.zvs"));
  CHECK(config.diagnostics_path == fs::path("ptp.csv"));
  CHECK(config.diagnostics_enabled);
  CHECK(config.end_effector == "crane_x7_tcp_link");
  CHECK(config.options.sample_rate == Approx(100.0));
  CHECK(config.options.profile == rtctrl::model::PtpProfile::MinimumJerk);
  CHECK(config.options.strict_ik);
  CHECK_FALSE(config.options.timing.motion_time);
  CHECK(config.start.position.c.x == Approx(0.2));
}

TEST_CASE("PTP CLI overrides common config values", "[ptp][config]") {
  ConfigFile file(kMinimalConfig);
  auto config = x7::ptp::loadConfig(file.path());
  const char* raw[] = {"x7_plan_ptp",
                       "--config",
                       "plan.toml",
                       "--output",
                       "motion",
                       "--motion-time",
                       "2.5",
                       "--max-linear-velocity",
                       "0.1",
                       "--max-angular-velocity",
                       "0.4",
                       "--sample-rate",
                       "250",
                       "--profile",
                       "trapezoidal",
                       "--no-strict-ik"};
  auto argv = const_cast<char**>(raw);
  const auto cli = x7::ptp::parseCli(
      static_cast<int>(sizeof(raw) / sizeof(raw[0])), argv);
  REQUIRE(cli.ok);
  x7::ptp::applyOverrides(cli, &config);
  CHECK(config.output_path == fs::path("motion.zvs"));
  CHECK(config.options.timing.motion_time == Approx(2.5));
  CHECK(config.options.timing.max_linear_velocity == Approx(0.1));
  CHECK(config.options.timing.max_angular_velocity == Approx(0.4));
  CHECK(config.options.sample_rate == Approx(250.0));
  CHECK(config.options.profile == rtctrl::model::PtpProfile::Trapezoidal);
  CHECK_FALSE(config.options.strict_ik);
}

TEST_CASE("PTP config rejects typos and incomplete speed limits",
          "[ptp][config]") {
  SECTION("unknown key") {
    ConfigFile file(std::string(kMinimalConfig) + "\nmotion_tim = 2.0\n");
    CHECK_THROWS_WITH(x7::ptp::loadConfig(file.path()),
                      Catch::Matchers::ContainsSubstring("unknown key"));
  }
  SECTION("unpaired velocity") {
    ConfigFile file(std::string(kMinimalConfig) +
                    "\n[trajectory]\nmax_linear_velocity = 0.1\n");
    CHECK_THROWS_WITH(
        x7::ptp::loadConfig(file.path()),
        Catch::Matchers::ContainsSubstring("must be specified together"));
  }
  SECTION("bad pose length") {
    std::string text(kMinimalConfig);
    const auto offset = text.find("position = [0.21, 0.0, 0.25]");
    REQUIRE(offset != std::string::npos);
    text.replace(offset, std::string("position = [0.21, 0.0, 0.25]").size(),
                 "position = [0.21, 0.0]");
    ConfigFile file(text);
    CHECK_THROWS_WITH(x7::ptp::loadConfig(file.path()),
                      Catch::Matchers::ContainsSubstring("exactly 3"));
  }
  SECTION("ambiguous legacy RPY key") {
    std::string text(kMinimalConfig);
    const auto offset = text.find("rpy_rad");
    REQUIRE(offset != std::string::npos);
    text.replace(offset, std::string("rpy_rad").size(), "rpy");
    ConfigFile file(text);
    CHECK_THROWS_WITH(
        x7::ptp::loadConfig(file.path()),
        Catch::Matchers::ContainsSubstring("unknown key 'rpy'"));
  }
}

TEST_CASE("PTP config interprets nonzero world RPY as radians",
          "[ptp][config][attitude]") {
  std::string text(kMinimalConfig);
  const auto offset = text.find("rpy_rad = [0.0, 0.0, 0.0]");
  REQUIRE(offset != std::string::npos);
  text.replace(offset, std::string("rpy_rad = [0.0, 0.0, 0.0]").size(),
               "rpy_rad = [0.31, -0.27, 0.42]");
  ConfigFile file(text);

  const auto config = x7::ptp::loadConfig(file.path());
  const auto expected =
      rtctrl::model::worldAttitudeFromRpyRad(0.31, -0.27, 0.42);
  zVec3D error;
  zMat3DError(&expected, &config.start.attitude, &error);
  CHECK(zVec3DNorm(&error) == Approx(0.0).margin(1e-12));
}

TEST_CASE("PTP CLI rejects missing values before config loading",
          "[ptp][config]") {
  char app[] = "x7_plan_ptp";
  char option[] = "--output";
  char flag[] = "--strict-ik";
  char* argv[] = {app, option, flag};
  const auto cli = x7::ptp::parseCli(3, argv);
  CHECK_FALSE(cli.ok);
  CHECK(cli.error.find("requires a value") != std::string::npos);
}

TEST_CASE("PTP bundle CLI owns its output location", "[ptp][config][bundle]") {
  SECTION("bundle path is accepted") {
    const char* raw[] = {"x7_plan_ptp", "--config", "plan.toml", "--bundle",
                         "archive/run-1"};
    auto argv = const_cast<char**>(raw);
    const auto cli = x7::ptp::parseCli(
        static_cast<int>(sizeof(raw) / sizeof(raw[0])), argv);
    REQUIRE(cli.ok);
    REQUIRE(cli.bundle_path);
    CHECK(*cli.bundle_path == fs::path("archive/run-1"));
  }

  SECTION("external output is incompatible") {
    const char* raw[] = {"x7_plan_ptp", "--config", "plan.toml", "--bundle",
                         "archive/run-1", "--output", "elsewhere.zvs"};
    auto argv = const_cast<char**>(raw);
    const auto cli = x7::ptp::parseCli(
        static_cast<int>(sizeof(raw) / sizeof(raw[0])), argv);
    CHECK_FALSE(cli.ok);
    CHECK(cli.error.find("owns") != std::string::npos);
  }

  SECTION("bundle diagnostics cannot be disabled") {
    const char* raw[] = {"x7_plan_ptp", "--config", "plan.toml", "--bundle",
                         "archive/run-1", "--no-diagnostics"};
    auto argv = const_cast<char**>(raw);
    const auto cli = x7::ptp::parseCli(
        static_cast<int>(sizeof(raw) / sizeof(raw[0])), argv);
    CHECK_FALSE(cli.ok);
    CHECK(cli.error.find("always includes") != std::string::npos);
  }
}

TEST_CASE("PTP diagnostics follow output unless explicitly configured",
          "[ptp][config][diagnostics]") {
  ConfigFile source(kMinimalConfig);
  auto config = x7::ptp::loadConfig(source.path());
  const char* raw[] = {"x7_plan_ptp", "--config", "plan.toml", "--output",
                       "motion"};
  auto argv = const_cast<char**>(raw);
  const auto cli = x7::ptp::parseCli(
      static_cast<int>(sizeof(raw) / sizeof(raw[0])), argv);
  REQUIRE(cli.ok);
  x7::ptp::applyOverrides(cli, &config);
  CHECK(config.output_path == fs::path("motion.zvs"));
  CHECK(config.diagnostics_path == fs::path("motion.csv"));
}

TEST_CASE("effective PTP config serializes overrides and portable paths",
          "[ptp][config][bundle]") {
  ConfigFile source(kMinimalConfig);
  auto config = x7::ptp::loadConfig(source.path());
  config.options.profile = rtctrl::model::PtpProfile::Linear;
  config.options.sample_rate = 250.0;
  config.options.timing.motion_time = 1.25;
  config.options.strict_ik = false;

  const auto serialized = x7::ptp::serializeEffectiveConfig(
      config, "model/copied.ztk", "trajectory.zvs", "trajectory.csv");
  const auto table = toml::parse(serialized);
  CHECK(table["model"].value_or(std::string()) == "model/copied.ztk");
  CHECK(table["output"].value_or(std::string()) == "trajectory.zvs");
  CHECK(table["diagnostics"]["enabled"].value_or(false));
  CHECK(table["diagnostics"]["output"].value_or(std::string()) ==
        "trajectory.csv");
  CHECK(table["trajectory"]["profile"].value_or(std::string()) == "linear");
  CHECK(table["trajectory"]["sample_rate"].value_or(0.0) == Approx(250.0));
  CHECK(table["trajectory"]["motion_time"].value_or(0.0) == Approx(1.25));
  CHECK_FALSE(table["ik"]["strict"].value_or(true));
  CHECK(table["start"]["rpy_rad"].as_array()->size() == 3);
  CHECK(table["ik"]["initial_joints"].as_array()->size() ==
        rtctrl::model::kCanonicalDof);
}
