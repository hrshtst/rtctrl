#include "disturb/force_budget_config.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;

namespace {

namespace fs = std::filesystem;

const char* kMinimalConfig = R"(
format = "rtctrl-x7-force-budget"
version = 1
model = "../../models/crane_x7/crane_x7.ztk"
reference = "ref.zvs"
link_force = "push.toml"

[budget]
joint_allowance_nm = [1.5]
)";

class ConfigFile {
 public:
  explicit ConfigFile(const std::string& content) {
    fs::create_directories("build/force_budget_config_test");
    path_ = "build/force_budget_config_test/budget.toml";
    std::ofstream output(path_);
    output << content;
  }
  ~ConfigFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

std::string replaced(std::string text, const std::string& from,
                     const std::string& to) {
  const auto at = text.find(from);
  REQUIRE(at != std::string::npos);
  return text.replace(at, from.size(), to);
}

void expectError(const std::string& content, const std::string& message) {
  ConfigFile file(content);
  CHECK_THROWS_WITH(x7::disturb::loadConfig(file.path()),
                    ContainsSubstring(message));
}

x7::disturb::Cli parse(std::vector<std::string> args) {
  std::vector<char*> argv;
  args.insert(args.begin(), "x7_force_budget");
  for (auto& arg : args) argv.push_back(arg.data());
  return x7::disturb::parseCli(static_cast<int>(argv.size()), argv.data());
}

}  // namespace

TEST_CASE("force budget config resolves inputs and applies defaults",
          "[force_budget][config]") {
  ConfigFile file(kMinimalConfig);
  const auto config = x7::disturb::loadConfig(file.path());
  const fs::path dir = fs::absolute("build/force_budget_config_test");
  CHECK(config.model_path ==
        fs::absolute("models/crane_x7/crane_x7.ztk").lexically_normal());
  CHECK(config.reference_path == (dir / "ref.zvs").lexically_normal());
  CHECK(config.link_force_path == (dir / "push.toml").lexically_normal());
  for (double value : config.budget.joint_allowance_nm) CHECK(value == 1.5);
  CHECK(config.budget.sample_rate_hz == Approx(1000.0));
  CHECK(config.csv_path == fs::path("force_budget.csv"));
  CHECK(config.reference.interpolation ==
        rtctrl::model::ReferenceInterpolation::ShapePreservingCubic);
}

TEST_CASE("force budget config reads eight allowances and options",
          "[force_budget][config]") {
  ConfigFile file(replaced(kMinimalConfig, "joint_allowance_nm = [1.5]",
                           "joint_allowance_nm = [1, 2, 3, 4, 5, 6, 7, 8]\n"
                           "sample_rate_hz = 250\n"
                           "[reference_processing]\n"
                           "filter = \"low-pass\"\n"
                           "interpolation = \"linear\"\n"
                           "[output]\n"
                           "csv = \"out/screen\""));
  const auto config = x7::disturb::loadConfig(file.path());
  for (int i = 0; i < 8; ++i) {
    CHECK(config.budget.joint_allowance_nm[i] == i + 1.0);
  }
  CHECK(config.budget.sample_rate_hz == 250.0);
  CHECK(config.reference.filter == rtctrl::model::ReferenceFilter::LowPass);
  CHECK(config.reference.interpolation ==
        rtctrl::model::ReferenceInterpolation::Linear);
  CHECK(config.csv_path == fs::path("out/screen.csv"));
}

TEST_CASE("the example force budget config loads", "[force_budget][config]") {
  const auto config =
      x7::disturb::loadConfig("config/force_budget_example.toml");
  CHECK(config.link_force_path ==
        fs::absolute("config/link_force_example.toml").lexically_normal());
  CHECK(config.budget.joint_allowance_nm[0] > 0.0);
}

TEST_CASE("force budget config rejects malformed input",
          "[force_budget][config]") {
  expectError(replaced(kMinimalConfig, "joint_allowance_nm = [1.5]", ""),
              "joint_allowance_nm needs one or eight values");
  expectError(replaced(kMinimalConfig, "[1.5]", "[1, 2, 3]"),
              "joint_allowance_nm needs one or eight values");
  expectError(replaced(kMinimalConfig, "[1.5]", "[0.0]"),
              "joint allowances must be finite and positive");
  expectError(replaced(kMinimalConfig, "[1.5]", "[-1.0]"),
              "joint allowances must be finite and positive");
  expectError(replaced(kMinimalConfig, "[1.5]", "[1.5]\nsample_rate_hz = 0"),
              "sample_rate_hz must be positive");
  expectError(
      replaced(kMinimalConfig, "[budget]\njoint_allowance_nm = [1.5]", ""),
      "missing [budget] table");
  expectError(kMinimalConfig + std::string("extra = 1\n"),
              "unknown key 'extra'");
  expectError(replaced(kMinimalConfig, "version = 1", "version = 2"),
              "format must be rtctrl-x7-force-budget version 1");
  expectError(replaced(kMinimalConfig, "link_force = \"push.toml\"", ""),
              "link_force must be a non-empty path");
}

TEST_CASE("force budget CLI requires a config and validates options",
          "[force_budget][config]") {
  const auto cli = parse({"--config", "a.toml", "--check", "--output", "o.csv",
                          "--reference", "r.zvs"});
  CHECK(cli.config_path == fs::path("a.toml"));
  CHECK(cli.check);
  CHECK(*cli.output_path == fs::path("o.csv"));
  CHECK(*cli.reference_path == fs::path("r.zvs"));
  CHECK(parse({"--help"}).help);
  CHECK_THROWS_WITH(parse({}), ContainsSubstring("--config is required"));
  CHECK_THROWS_WITH(parse({"--config", "a.toml", "--bogus"}),
                    ContainsSubstring("unknown option: --bogus"));
  CHECK_THROWS_WITH(parse({"--config", "a.toml", "--output", "--check"}),
                    ContainsSubstring("--output requires a value"));
}
