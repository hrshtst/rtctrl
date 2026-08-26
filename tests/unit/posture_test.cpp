#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdio>
#include <fstream>
#include <string>
#include <utility>

#include "common/legacy_anchor.hpp"
#include "rtctrl/model/posture.hpp"

using Catch::Approx;

namespace {

class TemporaryFile {
 public:
  TemporaryFile(std::string path, const std::string& content)
      : path_(std::move(path)) {
    std::ofstream output(path_);
    output << content;
  }
  ~TemporaryFile() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

}  // namespace

TEST_CASE("authored postures load from strict versioned TOML",
          "[posture][config]") {
  const auto posture =
      rtctrl::model::loadPostureToml("config/postures/p1.toml");
  CHECK(posture.name == "P1");
  CHECK(posture.joint_positions[0] == Approx(-0.357));
  CHECK(posture.joint_positions[7] == Approx(-0.014));
}

TEST_CASE("posture TOML rejects schema and value errors",
          "[posture][config]") {
  SECTION("unknown key") {
    TemporaryFile file(
        "build/posture_unknown.toml",
        "format_version = 1\nname = \"test\"\n"
        "joint_positions = [0,0,0,0,0,0,0,0]\nextra = 1\n");
    CHECK_THROWS_WITH(
        rtctrl::model::loadPostureToml(file.path()),
        Catch::Matchers::ContainsSubstring("unknown key"));
  }
  SECTION("wrong version") {
    TemporaryFile file(
        "build/posture_version.toml",
        "format_version = 2\nname = \"test\"\n"
        "joint_positions = [0,0,0,0,0,0,0,0]\n");
    CHECK_THROWS_WITH(
        rtctrl::model::loadPostureToml(file.path()),
        Catch::Matchers::ContainsSubstring("format_version"));
  }
  SECTION("wrong joint count") {
    TemporaryFile file(
        "build/posture_count.toml",
        "format_version = 1\nname = \"test\"\n"
        "joint_positions = [0,0,0]\n");
    CHECK_THROWS_WITH(
        rtctrl::model::loadPostureToml(file.path()),
        Catch::Matchers::ContainsSubstring("exactly 8"));
  }
}

TEST_CASE("legacy anchor reader is explicit and requires the anchor field",
          "[posture][legacy]") {
  double joints[rtctrl::model::kCanonicalDof] = {};
  TemporaryFile sidecar(
      "build/posture_legacy.json",
      "{\"time\": 123, \"anchor\": [1,2,3,4,5,6,7,8], \"other\": 9}");
  REQUIRE(x7::loadLegacyAnchorSidecar(sidecar.path(), joints));
  for (int i = 0; i < rtctrl::model::kCanonicalDof; ++i) {
    CHECK(joints[i] == Approx(i + 1.0));
  }

  TemporaryFile arbitrary("build/posture_arbitrary.txt",
                          "1 2 3 4 5 6 7 8");
  CHECK_FALSE(x7::loadLegacyAnchorSidecar(arbitrary.path(), joints));

  TemporaryFile extra(
      "build/posture_legacy_extra.json",
      "{\"anchor\": [1,2,3,4,5,6,7,8,9]}");
  CHECK_FALSE(x7::loadLegacyAnchorSidecar(extra.path(), joints));

  TemporaryFile nonfinite(
      "build/posture_legacy_nonfinite.json",
      "{\"anchor\": [1,2,3,4,5,6,7,1e999]}");
  CHECK_FALSE(x7::loadLegacyAnchorSidecar(nonfinite.path(), joints));
}
