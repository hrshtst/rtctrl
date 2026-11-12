// The follower's learned-offset preflight (follow/follow_preflight.hpp) and
// its bundle provenance (follow/follow_bundle.hpp): every refusal happens
// before any bus contact, from files alone.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>

#include "follow/follow_bundle.hpp"
#include "follow/follow_config.hpp"
#include "follow/follow_preflight.hpp"
#include "plan/ptp_bundle.hpp"
#include "rtctrl/learn/command_offset.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace follow = x7::follow;
namespace fs = std::filesystem;
namespace learn = rtctrl::learn;
namespace model = rtctrl::model;

using Catch::Matchers::ContainsSubstring;

namespace {

const std::string kOtherSha(64, 'b');

// A 0.06 s reference: three 0.02 s frames, joint 0 from `start` up by 0.02
// rad per frame, every other joint at 0.
fs::path writeReference(const std::string& name, double start) {
  const fs::path path = "build/" + name;
  std::ofstream out(path);
  out.precision(17);
  for (int frame = 0; frame < 3; ++frame) {
    out << "0.02 8 (" << start + 0.02 * frame << " 0 0 0 0 0 0 0 )\n";
  }
  return path;
}

fs::path writeOffset(const std::string& name,
                     const learn::CommandOffset& offset) {
  const fs::path path = "build/" + name;
  fs::remove(path);
  learn::writeCommandOffset(path, offset);
  return path;
}

// A trial-1 offset on the follower's 100 Hz grid of `reference`.
learn::CommandOffset learned(
    const model::ZvsTrajectory& reference, const std::string& reference_sha256,
    const std::function<double(std::size_t, int)>& value) {
  const auto grid = learn::trackingGrid(reference.duration(), 0.01);
  learn::CommandOffset offset;
  offset.method = "ilc";
  offset.trial = 1;
  offset.dt_s = grid.dt_s;
  offset.reference_duration_s = reference.duration();
  offset.reference_sha256 = reference_sha256;
  offset.parent_sha256 = kOtherSha;
  offset.source_log_sha256 = kOtherSha;
  offset.delta_rad.resize(grid.samples());
  for (std::size_t i = 0; i < grid.samples(); ++i) {
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      offset.delta_rad[i][j] = value(i, j);
    }
  }
  return offset;
}

struct Fixture {
  Fixture()
      : chain("models/crane_x7/crane_x7.ztk"),
        map(chain),
        reference_path(writeReference("follow_offset_reference.zvs", 0.1)),
        reference(reference_path.string(), map),
        reference_sha256(x7::ptp::sha256File(reference_path)) {
    config.reference_path = reference_path;
  }
  ~Fixture() {
    std::error_code ignored;
    fs::remove(reference_path, ignored);
  }

  // Writes the offset, points the config at it and runs the preflight;
  // the file is removed again however the preflight ends.
  std::optional<follow::AcceptedOffset> preflight(
      const learn::CommandOffset& offset) {
    offset_path = writeOffset("follow_offset_preflight.csv", offset);
    struct Remove {
      fs::path path;
      ~Remove() {
        std::error_code ignored;
        fs::remove(path, ignored);
      }
    } remove{offset_path};
    config.learning.offset_path = offset_path;
    return follow::preflightOffset(config, chain, map, reference);
  }

  model::ChainModel chain;
  model::JointMap map;
  fs::path reference_path;
  model::ZvsTrajectory reference;
  std::string reference_sha256;
  fs::path offset_path;
  follow::Config config;
};

}  // namespace

TEST_CASE("follow preflight without an offset reads nothing",
          "[follow][preflight][offset]") {
  Fixture fixture;
  fixture.config.reference_path = "build/no_such_reference.zvs";
  CHECK_FALSE(follow::preflightOffset(fixture.config, fixture.chain,
                                      fixture.map, fixture.reference));
  CHECK(follow::offsetProvenance(std::nullopt).empty());
  CHECK_NOTHROW(follow::refusePostureOffset(fixture.config));
}

TEST_CASE("follow preflight accepts an offset made for this run and "
          "reports it",
          "[follow][preflight][offset]") {
  Fixture fixture;
  SECTION("the zero offset of trial 0") {
    const auto zero = learn::zeroOffset(fixture.reference.duration(), 0.01,
                                        fixture.reference_sha256);
    fixture.offset_path = writeOffset("follow_offset_zero.csv", zero);
    const auto sha256 = x7::ptp::sha256File(fixture.offset_path);
    fixture.config.learning.offset_path = fixture.offset_path;
    const auto accepted = follow::preflightOffset(
        fixture.config, fixture.chain, fixture.map, fixture.reference);
    REQUIRE(accepted);
    CHECK(accepted->offset.method == "zero");
    CHECK(accepted->offset.trial == 0);
    CHECK(accepted->offset.delta_rad.size() == zero.delta_rad.size());
    CHECK(accepted->sha256 == sha256);
    CHECK(accepted->max_abs_rad == 0.0);
    CHECK(follow::describeOffset(*accepted) ==
          "command offset accepted: method zero, trial 0, max |delta| 0 "
          "rad, sha256 " +
              sha256);
    CHECK(follow::offsetProvenance(accepted) ==
          "offset_method = \"zero\"\noffset_trial = 0\noffset_sha256 = \"" +
              sha256 + "\"\n");
    fs::remove(fixture.offset_path);
  }
  SECTION("a learned offset inside lowered limits") {
    fixture.config.learning.max_offset_rad = 0.05;
    fixture.config.learning.max_offset_rate_rad_s = 0.3;
    // 0.0025 rad per 0.01 s step on joint 3: 0.25 rad/s.
    const auto offset = learned(
        fixture.reference, fixture.reference_sha256,
        [](std::size_t i, int j) {
          return j == 3 ? -0.0025 * static_cast<double>(i) : 0.0;
        });
    const double widest = std::fabs(offset.delta_rad.back()[3]);
    REQUIRE(widest > 0.01);
    const auto accepted = fixture.preflight(offset);
    REQUIRE(accepted);
    CHECK(accepted->offset.method == "ilc");
    CHECK(accepted->offset.trial == 1);
    CHECK(accepted->max_abs_rad == widest);
    char expected[64];
    std::snprintf(expected, sizeof expected, "max |delta| %.6g rad", widest);
    CHECK_THAT(follow::describeOffset(*accepted),
               ContainsSubstring("method ilc, trial 1, ") &&
                   ContainsSubstring(expected));
  }
}

TEST_CASE("follow preflight refuses an offset made for another reference",
          "[follow][preflight][offset]") {
  Fixture fixture;
  const auto zero = [&](const std::string& sha256) {
    return learn::zeroOffset(fixture.reference.duration(), 0.01, sha256);
  };
  CHECK_THROWS_WITH(fixture.preflight(zero(kOtherSha)),
                    ContainsSubstring("does not match the reference being "
                                      "run"));
  // The same duration and grid, other bytes: the hash tells them apart.
  const auto other_path = writeReference("follow_offset_other.zvs", 0.2);
  const auto other_sha256 = x7::ptp::sha256File(other_path);
  CHECK(other_sha256 != fixture.reference_sha256);
  CHECK_THROWS_WITH(fixture.preflight(zero(other_sha256)),
                    ContainsSubstring("does not match the reference being "
                                      "run"));
  fs::remove(other_path);
}

TEST_CASE("follow preflight refuses an offset off the run's tracking grid",
          "[follow][preflight][offset]") {
  Fixture fixture;
  // Made for a 200 Hz run.
  CHECK_THROWS_WITH(
      fixture.preflight(learn::zeroOffset(fixture.reference.duration(),
                                          0.005, fixture.reference_sha256)),
      ContainsSubstring("samples, the tracking grid"));
  // Made for 100 Hz, run at 50 Hz.
  fixture.config.control_rate_hz = 50.0;
  CHECK_THROWS_WITH(
      fixture.preflight(learn::zeroOffset(fixture.reference.duration(), 0.01,
                                          fixture.reference_sha256)),
      ContainsSubstring("samples, the tracking grid"));
}

TEST_CASE("follow preflight refuses an offset beyond the configured limits",
          "[follow][preflight][offset]") {
  Fixture fixture;
  const auto constant = [&](double value) {
    return learned(
        fixture.reference, fixture.reference_sha256,
        [value](std::size_t, int j) { return j == 2 ? value : 0.0; });
  };
  // The default magnitude limit.
  CHECK_THROWS_WITH(fixture.preflight(constant(0.16)),
                    ContainsSubstring("joint 2, sample 0 (t = 0 s): delta "
                                      "0.16 rad exceeds max_abs_rad 0.15"));
  // A lowered one.
  fixture.config.learning.max_offset_rad = 0.05;
  CHECK_NOTHROW(fixture.preflight(constant(0.05)));
  CHECK_THROWS_WITH(fixture.preflight(constant(-0.06)),
                    ContainsSubstring("exceeds max_abs_rad 0.05"));
  // The rate limit, default and lowered: a step of 0.004 rad per 0.01 s.
  const auto step = learned(fixture.reference, fixture.reference_sha256,
                            [](std::size_t i, int j) {
                              return j == 5 && i >= 3 ? 0.004 : 0.0;
                            });
  CHECK_NOTHROW(fixture.preflight(step));
  fixture.config.learning.max_offset_rate_rad_s = 0.3;
  CHECK_THROWS_WITH(fixture.preflight(step),
                    ContainsSubstring("joint 5, sample 2"));
  CHECK_THROWS_WITH(fixture.preflight(step),
                    ContainsSubstring("exceeds max_rate_rad_s 0.3"));
}

TEST_CASE("follow preflight refuses a reference plus offset outside the "
          "joint limits",
          "[follow][preflight][offset]") {
  Fixture fixture;
  const int joint = 0;
  const double hi = fixture.chain.jointMax(fixture.map.linkId(joint));
  const double lo = fixture.chain.jointMin(fixture.map.linkId(joint));
  // Joint 0 ends 0.01 rad below its upper limit: the reference passes,
  // reference plus 0.02 rad does not, from the sample where it crosses.
  const auto path = writeReference("follow_offset_limit.zvs", hi - 0.05);
  model::ZvsTrajectory near(path.string(), fixture.map);
  fixture.config.reference_path = path;
  const auto sha256 = x7::ptp::sha256File(path);
  const auto offset_path = writeOffset(
      "follow_offset_limit.csv",
      learned(near, sha256,
              [](std::size_t, int j) { return j == 0 ? 0.02 : 0.0; }));
  fixture.config.learning.offset_path = offset_path;
  CHECK_THROWS_WITH(
      follow::preflightOffset(fixture.config, fixture.chain, fixture.map,
                              near),
      ContainsSubstring("reference plus offset leaves the joint 0 limits"));
  // Downward, past the lower limit.
  fs::remove(path);
  fs::remove(offset_path);
  const auto low_path = writeReference("follow_offset_limit.zvs", lo + 0.01);
  model::ZvsTrajectory low(low_path.string(), fixture.map);
  fixture.config.reference_path = low_path;
  const auto low_offset = writeOffset(
      "follow_offset_limit.csv",
      learned(low, x7::ptp::sha256File(low_path),
              [](std::size_t, int j) { return j == 0 ? -0.02 : 0.0; }));
  CHECK_THROWS_WITH(
      follow::preflightOffset(fixture.config, fixture.chain, fixture.map,
                              low),
      ContainsSubstring("joint 0 limits") &&
          ContainsSubstring("tracking sample 0 (t = 0 s)"));
  fs::remove(low_path);
  fs::remove(low_offset);
}

TEST_CASE("follow preflight refuses an offset with a posture reference",
          "[follow][preflight][offset][posture]") {
  Fixture fixture;
  fixture.config.learning.offset_path = "build/any_offset.csv";
  CHECK_THROWS_WITH(follow::refusePostureOffset(fixture.config),
                    ContainsSubstring("cannot be applied to a reference "
                                      "built from the measured posture"));
  fixture.config.posture = follow::PostureRunConfig{0.005, 0.05};
  CHECK_THROWS_WITH(
      fixture.preflight(learn::zeroOffset(fixture.reference.duration(), 0.01,
                                          fixture.reference_sha256)),
      ContainsSubstring("measured posture"));
}

TEST_CASE("follow preflight refuses a missing or malformed offset file",
          "[follow][preflight][offset]") {
  Fixture fixture;
  fixture.config.learning.offset_path = "build/no_such_offset.csv";
  CHECK_THROWS_WITH(
      follow::preflightOffset(fixture.config, fixture.chain, fixture.map,
                              fixture.reference),
      ContainsSubstring("cannot read the offset file "
                        "'build/no_such_offset.csv'"));
  const auto zero = learn::zeroOffset(fixture.reference.duration(), 0.01,
                                      fixture.reference_sha256);
  const auto text = learn::formatCommandOffset(zero);
  const fs::path path = "build/follow_offset_truncated.csv";
  {
    std::ofstream out(path, std::ios::binary);
    out << text.substr(0, text.size() - 1);  // no final newline
  }
  fixture.config.learning.offset_path = path;
  CHECK_THROWS_WITH(
      follow::preflightOffset(fixture.config, fixture.chain, fixture.map,
                              fixture.reference),
      ContainsSubstring("missing final newline"));
  fs::remove(path);
}
