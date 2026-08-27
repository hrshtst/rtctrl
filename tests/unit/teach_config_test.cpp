#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <string>

#include "teach/teach_config.hpp"

namespace fs = std::filesystem;
namespace teach = x7::teach;

namespace {

fs::path writeConfig(const std::string& body) {
  static int serial = 0;
  const auto path = fs::path("/tmp") /
                    ("rtctrl-teach-config-" + std::to_string(serial++) +
                     ".toml");
  std::ofstream output(path);
  output << body;
  return path;
}

const char* validConfig() {
  return R"(
format = "rtctrl-x7-teach"
version = 1
model = "../home/atsuta/develop/rtctrl/models/crane_x7/crane_x7.ztk"
hardware_config = "../home/atsuta/develop/rtctrl/config/crane_x7_vendor_scale.toml"
mode = "torque-off"

[recording]
sample_rate_hz = 50
start_timeout_s = 8
max_duration_s = 30

[finalization]
operator_timeout_s = 15

[output]
motion = "motion"
log = "motion-log"
)";
}

}  // namespace

TEST_CASE("teach config loads paths, mode, timing, and outputs", "[teach]") {
  const auto path = writeConfig(validConfig());
  const auto config = teach::loadConfig(path);
  CHECK(config.mode == teach::Mode::TorqueOff);
  CHECK(config.recording.sample_rate_hz == 50.0);
  CHECK(config.recording.max_duration_s == 30.0);
  CHECK(config.model_path.filename() == "crane_x7.ztk");
  CHECK(config.output.motion == "motion.zvs");
  CHECK(config.output.log == "motion-log.csv");
  fs::remove(path);
}

TEST_CASE("teach config enforces gravity session and sample bounds",
          "[teach]") {
  const auto path = writeConfig(validConfig());
  auto config = teach::loadConfig(path);
  config.mode = teach::Mode::GravityCompensation;
  config.recording.max_duration_s = 38.0;
  CHECK_THROWS(teach::validate(&config));
  config.recording.max_duration_s = 30.0;
  CHECK_NOTHROW(teach::validate(&config));
  config.recording.sample_rate_hz = 100.1;
  CHECK_THROWS(teach::validate(&config));
  fs::remove(path);
}

TEST_CASE("teach CLI parses recording overrides", "[teach]") {
  const char* raw[] = {"x7_teach",      "--config",      "teach.toml",
                       "--mode",        "gravity-compensation",
                       "--sample-rate", "40",            "--max-duration",
                       "12",            "--output",      "take",
                       "--log",         "take-log.csv"};
  auto cli = teach::parseCli(static_cast<int>(std::size(raw)),
                             const_cast<char**>(raw));
  REQUIRE(cli.mode);
  CHECK(*cli.mode == teach::Mode::GravityCompensation);
  CHECK(*cli.sample_rate_hz == 40.0);
  CHECK(*cli.max_duration_s == 12.0);
  CHECK(*cli.output_path == "take");
  CHECK(*cli.log_path == "take-log.csv");
}

TEST_CASE("teach bundle owns its output paths", "[teach]") {
  const char* raw[] = {"x7_teach", "--config", "teach.toml", "--bundle",
                       "archive", "--output", "motion.zvs"};
  CHECK_THROWS(teach::parseCli(static_cast<int>(std::size(raw)),
                               const_cast<char**>(raw)));
}
