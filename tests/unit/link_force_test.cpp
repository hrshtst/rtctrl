#include "rtctrl/model/link_force.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using rtctrl::model::ForceDirection;
using rtctrl::model::ForceProfile;
using rtctrl::model::LinkForceSpec;

namespace {

namespace fs = std::filesystem;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

ForceProfile exampleProfile() {
  ForceProfile profile;
  profile.onset_s = 0.4;
  profile.rise_s = 0.2;
  profile.hold_s = 0.6;
  profile.fall_s = 0.3;
  return profile;
}

LinkForceSpec worldSpec() {
  LinkForceSpec spec;
  spec.link = "crane_x7_tcp_link";
  spec.point_link_m = {0.0, 0.01, 0.0};
  spec.direction = ForceDirection::World;
  spec.direction_world = {0.0, 2.0, 0.0};
  spec.magnitude_n = 3.0;
  spec.profile = exampleProfile();
  return spec;
}

void expectInvalid(const std::function<void(LinkForceSpec&)>& mutate) {
  LinkForceSpec spec = worldSpec();
  mutate(spec);
  CHECK_THROWS_AS(rtctrl::model::validateLinkForce(spec),
                  std::invalid_argument);
}

constexpr const char* kValidSpec = R"(
format = "rtctrl-x7-link-force"
version = 1
link = "crane_x7_tcp_link"
point_link_m = [0.0, 0.0, 0.01]
magnitude_n = 2
direction_world = [1, 0, 0]

[profile]
onset_s = 0.5
rise_s = 0.1
hold_s = 0
fall_s = 0.1
)";

class SpecFile {
 public:
  explicit SpecFile(const std::string& content) {
    fs::create_directories("build/link_force_test");
    path_ = "build/link_force_test/spec.toml";
    std::ofstream output(path_);
    output << content;
  }
  ~SpecFile() { std::remove(path_.c_str()); }
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

void expectLoadError(const std::string& content, const std::string& message) {
  SpecFile file(content);
  CHECK_THROWS_WITH(rtctrl::model::loadLinkForce(file.path()),
                    ContainsSubstring(message));
}

}  // namespace

TEST_CASE("force profile is zero outside its window and one in the hold",
          "[link_force][profile]") {
  const ForceProfile profile = exampleProfile();
  CHECK(profile.end_s() == Approx(1.5));
  CHECK(profile.scale(-1.0) == 0.0);
  CHECK(profile.scale(0.399999) == 0.0);
  CHECK(profile.scale(0.4) == 0.0);
  CHECK(profile.scale(0.5) == Approx(0.5).margin(1e-12));
  CHECK(profile.scale(0.6) == 1.0);
  CHECK(profile.scale(0.9) == 1.0);
  CHECK(profile.scale(1.2) == 1.0);
  CHECK(profile.scale(1.35) == Approx(0.5).margin(1e-12));
  CHECK(profile.scale(1.5) == 0.0);
  CHECK(profile.scale(10.0) == 0.0);
  CHECK(profile.scale(kNaN) == 0.0);
  CHECK(profile.scale(kInf) == 0.0);
}

TEST_CASE("force profile ramps are monotonic with flat ends",
          "[link_force][profile]") {
  const ForceProfile profile = exampleProfile();
  double previous = 0.0;
  for (int k = 1; k <= 200; ++k) {
    const double value = profile.scale(0.4 + 0.2 * k / 200.0);
    CHECK(value >= previous);
    previous = value;
  }
  for (int k = 1; k <= 300; ++k) {
    const double value = profile.scale(1.2 + 0.3 * k / 300.0);
    CHECK(value <= previous);
    previous = value;
  }
  // One-sided difference quotients at each ramp end vanish with h.
  const double h = 1e-6;
  CHECK(std::abs(profile.scale(0.4 + h) - profile.scale(0.4)) / h < 1e-4);
  CHECK(std::abs(profile.scale(0.6) - profile.scale(0.6 - h)) / h < 1e-4);
  CHECK(std::abs(profile.scale(1.2 + h) - profile.scale(1.2)) / h < 1e-4);
  CHECK(std::abs(profile.scale(1.5) - profile.scale(1.5 - h)) / h < 1e-4);
}

TEST_CASE("force profile with no hold peaks exactly at the rise end",
          "[link_force][profile]") {
  ForceProfile profile = exampleProfile();
  profile.hold_s = 0.0;
  CHECK(profile.scale(0.6) == 1.0);
  CHECK(profile.scale(0.6 + 1e-3) < 1.0);
  CHECK(profile.end_s() == Approx(0.9));
}

TEST_CASE("force profile integrates to its analytic impulse scale",
          "[link_force][profile]") {
  const ForceProfile profile = exampleProfile();
  CHECK(profile.impulseScale() == Approx(0.1 + 0.6 + 0.15));
  const double dt = 1e-5;
  double integral = 0.0;
  for (double t = 0.0; t < 2.0; t += dt) {
    integral += 0.5 * (profile.scale(t) + profile.scale(t + dt)) * dt;
  }
  CHECK(integral == Approx(profile.impulseScale()).margin(1e-6));
}

TEST_CASE("link-force validation accepts both direction modes",
          "[link_force][validate]") {
  CHECK_NOTHROW(rtctrl::model::validateLinkForce(worldSpec()));

  // the unused vector of the other mode is never read
  LinkForceSpec world = worldSpec();
  world.anchor_world_m = {0.0, 0.0, 0.0};
  CHECK_NOTHROW(rtctrl::model::validateLinkForce(world));
  world.anchor_world_m = {kNaN, kNaN, kNaN};
  CHECK_NOTHROW(rtctrl::model::validateLinkForce(world));

  LinkForceSpec anchor = worldSpec();
  anchor.direction = ForceDirection::Anchor;
  anchor.direction_world = {0.0, 0.0, 0.0};
  anchor.anchor_world_m = {0.3, 0.2, 0.1};
  CHECK_NOTHROW(rtctrl::model::validateLinkForce(anchor));

  LinkForceSpec zero_force = worldSpec();
  zero_force.magnitude_n = 0.0;
  CHECK_NOTHROW(rtctrl::model::validateLinkForce(zero_force));
}

TEST_CASE("link-force validation rejects each invalid field",
          "[link_force][validate]") {
  expectInvalid([](LinkForceSpec& s) { s.link.clear(); });
  expectInvalid([](LinkForceSpec& s) { s.point_link_m[1] = kNaN; });
  expectInvalid([](LinkForceSpec& s) { s.point_link_m[2] = kInf; });
  expectInvalid([](LinkForceSpec& s) { s.magnitude_n = -1.0; });
  expectInvalid([](LinkForceSpec& s) { s.magnitude_n = kNaN; });
  expectInvalid([](LinkForceSpec& s) { s.magnitude_n = kInf; });
  expectInvalid([](LinkForceSpec& s) { s.profile.onset_s = -0.1; });
  expectInvalid([](LinkForceSpec& s) { s.profile.onset_s = kNaN; });
  expectInvalid([](LinkForceSpec& s) { s.profile.rise_s = 0.0; });
  expectInvalid([](LinkForceSpec& s) { s.profile.rise_s = -0.2; });
  expectInvalid([](LinkForceSpec& s) { s.profile.hold_s = -0.1; });
  expectInvalid([](LinkForceSpec& s) { s.profile.hold_s = kInf; });
  expectInvalid([](LinkForceSpec& s) { s.profile.fall_s = 0.0; });
  expectInvalid([](LinkForceSpec& s) { s.profile.fall_s = kNaN; });
  // every term finite, the sum not
  expectInvalid([](LinkForceSpec& s) {
    s.profile.onset_s = 1e308;
    s.profile.hold_s = 1e308;
  });
  expectInvalid([](LinkForceSpec& s) { s.direction_world = {0.0, 0.0, 0.0}; });
  expectInvalid([](LinkForceSpec& s) { s.direction_world = {1e-10, 0, 0}; });
  expectInvalid([](LinkForceSpec& s) { s.direction_world[0] = kNaN; });
  expectInvalid([](LinkForceSpec& s) {
    s.direction = ForceDirection::Anchor;
    s.anchor_world_m = {0.1, kInf, 0.0};
  });
  expectInvalid(
      [](LinkForceSpec& s) { s.direction = static_cast<ForceDirection>(7); });
}

TEST_CASE("link-force direction normalizes and detects degenerate geometry",
          "[link_force][direction]") {
  std::array<double, 3> unit{9.0, 9.0, 9.0};
  const LinkForceSpec world = worldSpec();
  REQUIRE(rtctrl::model::linkForceDirection(world, {5.0, 5.0, 5.0}, &unit));
  CHECK(unit[0] == 0.0);
  CHECK(unit[1] == 1.0);
  CHECK(unit[2] == 0.0);

  LinkForceSpec anchor = worldSpec();
  anchor.direction = ForceDirection::Anchor;
  anchor.anchor_world_m = {0.3, 0.4, 0.0};
  REQUIRE(rtctrl::model::linkForceDirection(anchor, {0.0, 0.0, 0.0}, &unit));
  CHECK(unit[0] == Approx(0.6));
  CHECK(unit[1] == Approx(0.8));
  CHECK(unit[2] == 0.0);

  // closer than the minimum anchor distance: undefined, output untouched
  unit = {9.0, 9.0, 9.0};
  CHECK_FALSE(
      rtctrl::model::linkForceDirection(anchor, {0.3, 0.4, 0.0009}, &unit));
  CHECK_FALSE(
      rtctrl::model::linkForceDirection(anchor, {kNaN, 0.0, 0.0}, &unit));
  CHECK(unit[0] == 9.0);

  LinkForceSpec tiny = worldSpec();
  tiny.direction_world = {1e-12, 0.0, 0.0};
  CHECK_FALSE(rtctrl::model::linkForceDirection(tiny, {0.0, 0.0, 0.0}, &unit));
}

TEST_CASE("the example link-force spec loads", "[link_force][load]") {
  const auto spec =
      rtctrl::model::loadLinkForce("config/link_force_example.toml");
  CHECK(spec.link == "crane_x7_tcp_link");
  CHECK(spec.direction == ForceDirection::World);
  CHECK(spec.magnitude_n > 0.0);
  CHECK(spec.anchor_world_m == std::array<double, 3>{0.0, 0.0, 0.0});
  CHECK(spec.profile.end_s() == Approx(1.4));
}

TEST_CASE("the link-force loader reads both modes and integer numbers",
          "[link_force][load]") {
  {
    SpecFile file(kValidSpec);
    const auto spec = rtctrl::model::loadLinkForce(file.path());
    CHECK(spec.direction == ForceDirection::World);
    CHECK(spec.direction_world == std::array<double, 3>{1.0, 0.0, 0.0});
    CHECK(spec.magnitude_n == 2.0);
    CHECK(spec.profile.hold_s == 0.0);
    CHECK(spec.point_link_m[2] == 0.01);
  }
  {
    SpecFile file(replaced(kValidSpec, "direction_world = [1, 0, 0]",
                           "anchor_world_m = [0.2, 0.0, 0.3]"));
    const auto spec = rtctrl::model::loadLinkForce(file.path());
    CHECK(spec.direction == ForceDirection::Anchor);
    CHECK(spec.anchor_world_m == std::array<double, 3>{0.2, 0.0, 0.3});
    CHECK(spec.direction_world == std::array<double, 3>{0.0, 0.0, 0.0});
  }
}

TEST_CASE("the link-force loader rejects malformed specs",
          "[link_force][load]") {
  expectLoadError(kValidSpec + std::string("extra = 1\n"),
                  "unknown key 'extra'");
  expectLoadError(replaced(kValidSpec, "fall_s = 0.1", "fall_s = 0.1\nx = 1"),
                  "unknown key 'x' in [profile]");
  expectLoadError(replaced(kValidSpec, "hold_s = 0\n", ""),
                  "profile.hold_s must be a number");
  expectLoadError(replaced(kValidSpec, "direction_world = [1, 0, 0]",
                           "direction_world = [1, 0, 0]\n"
                           "anchor_world_m = [0, 0, 1]"),
                  "exactly one of");
  expectLoadError(replaced(kValidSpec, "direction_world = [1, 0, 0]", ""),
                  "exactly one of");
  expectLoadError(replaced(kValidSpec, "rtctrl-x7-link-force", "other"),
                  "format must be");
  expectLoadError(replaced(kValidSpec, "version = 1", "version = 2"),
                  "format must be");
  expectLoadError(replaced(kValidSpec, "[0.0, 0.0, 0.01]", "[0.0, 0.01]"),
                  "point_link_m must be an array of three numbers");
  expectLoadError(replaced(kValidSpec, "[1, 0, 0]", "[1, \"y\", 0]"),
                  "direction_world must contain three finite numbers");
  expectLoadError(replaced(kValidSpec, "magnitude_n = 2", "magnitude_n = nan"),
                  "magnitude_n must be finite");
  expectLoadError(
      replaced(kValidSpec, "link = \"crane_x7_tcp_link\"", "link = \"\""),
      "link must be a non-empty string");
  // the shared validator runs after parsing
  expectLoadError(replaced(kValidSpec, "[1, 0, 0]", "[0, 0, 0]"),
                  "direction_world must be finite and nonzero");
  expectLoadError(replaced(kValidSpec, "rise_s = 0.1", "rise_s = 0"),
                  "profile.rise_s must be finite and > 0");
  expectLoadError("format = [", "cannot parse");
}
