// The follower's learned-offset preflight (follow/follow_preflight.hpp) and
// its bundle provenance (follow/follow_bundle.hpp): every refusal happens
// before any bus contact, from files alone.
#include <unistd.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include "follow/follow_bundle.hpp"
#include "follow/follow_config.hpp"
#include "follow/follow_preflight.hpp"
#include "follow/reference_signature.hpp"
#include "plan/ptp_bundle.hpp"
#include "rtctrl/learn/command_offset.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace fs = std::filesystem;
namespace learn = rtctrl::learn;
namespace model = rtctrl::model;

using Catch::Matchers::ContainsSubstring;

namespace {

const std::string kOtherSha(64, 'b');

// build/NAME with this process's id before the extension: ctest runs each
// test case as a process of its own, in parallel, and the cases share names.
fs::path scratch(const std::string& name) {
  const fs::path file(name);
  return fs::path("build") /
         (file.stem().string() + "." + std::to_string(::getpid()) +
          file.extension().string());
}

// A 0.06 s reference: three 0.02 s frames, joint 0 from `start` up by 0.02
// rad per frame, every other joint at 0.
fs::path writeReference(const std::string& name, double start) {
  const fs::path path = scratch(name);
  std::ofstream out(path);
  out.precision(17);
  for (int frame = 0; frame < 3; ++frame) {
    out << "0.02 8 (" << start + 0.02 * frame << " 0 0 0 0 0 0 0 )\n";
  }
  return path;
}

fs::path writeOffset(const std::string& name,
                     const learn::CommandOffset& offset) {
  const fs::path path = scratch(name);
  fs::remove(path);
  learn::writeCommandOffset(path, offset);
  return path;
}

// A trial-1 offset on the follower's 100 Hz grid of `reference`, bound to
// the reference signature `signature`.
learn::CommandOffset learned(
    const model::ZvsTrajectory& reference, const std::string& signature,
    const std::function<double(std::size_t, int)>& value) {
  const auto grid = learn::trackingGrid(reference.duration(), 0.01);
  learn::CommandOffset offset;
  offset.method = "ilc";
  offset.trial = 1;
  offset.dt_s = grid.dt_s;
  offset.reference_duration_s = reference.duration();
  offset.reference_sha256 = signature;
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

// The default follow config (100 Hz, position mode, default reference
// processing) on a 0.06 s reference; `signature` is its reference signature.
struct Fixture {
  Fixture()
      : chain("models/crane_x7/crane_x7.ztk"),
        map(chain),
        reference_path(writeReference("follow_offset_reference.zvs", 0.1)),
        reference(reference_path.string(), map) {
    config.reference_path = reference_path;
    signature = follow::referenceSignature(config);
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
  std::string signature;
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
                                        fixture.signature);
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
        fixture.reference, fixture.signature,
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
  // The same duration and grid, other bytes: the signature tells them
  // apart.
  const auto other_path = writeReference("follow_offset_other.zvs", 0.2);
  follow::Config other = fixture.config;
  other.reference_path = other_path;
  const auto other_signature = follow::referenceSignature(other);
  CHECK(other_signature != fixture.signature);
  CHECK_THROWS_WITH(fixture.preflight(zero(other_signature)),
                    ContainsSubstring("does not match the reference being "
                                      "run"));
  fs::remove(other_path);
  // The reference file's own hash is no signature: an offset bound to the
  // file alone (as offsets were before the signature) is refused too.
  CHECK_THROWS_WITH(
      fixture.preflight(zero(x7::ptp::sha256File(fixture.reference_path))),
      ContainsSubstring("does not match the reference being run"));
}

TEST_CASE("follow preflight refuses an offset learned under other reference "
          "settings",
          "[follow][preflight][offset][signature]") {
  // Each change keeps the offset's tracking grid, so only the signature
  // can refuse it: the follower would command another trajectory.
  struct Case {
    const char* what;
    std::function<void(follow::Config&)> base;
    std::function<void(follow::Config&)> change;
  };
  const auto none = [](follow::Config&) {};
  const std::vector<Case> cases = {
      {"filter", none,
       [](follow::Config& c) {
         c.reference.filter = model::ReferenceFilter::LowPass;
       }},
      {"cut-off",
       [](follow::Config& c) {
         c.reference.filter = model::ReferenceFilter::LowPass;
       },
       [](follow::Config& c) { c.reference.low_pass_cutoff_hz = 4.0; }},
      {"interpolation", none,
       [](follow::Config& c) {
         c.reference.interpolation = model::ReferenceInterpolation::Linear;
       }},
      // The 0.06 s reference has 6 intervals at 100 Hz and at 99 Hz.
      {"control rate", none,
       [](follow::Config& c) { c.control_rate_hz = 99.0; }},
      {"control mode", none, [](follow::Config& c) {
         c.mode = arm::ControlMode::CurrentBasedPosition;
       }}};
  for (const Case& c : cases) {
    INFO(c.what);
    Fixture fixture;
    c.base(fixture.config);
    model::ZvsTrajectory base_reference(fixture.reference_path.string(),
                                        fixture.map, fixture.config.reference);
    const auto zero = learn::zeroOffset(base_reference.duration(),
                                        1.0 / fixture.config.control_rate_hz,
                                        follow::referenceSignature(
                                            fixture.config));
    fixture.offset_path = writeOffset("follow_offset_settings.csv", zero);
    fixture.config.learning.offset_path = fixture.offset_path;
    // Unchanged settings: accepted.
    CHECK(follow::preflightOffset(fixture.config, fixture.chain, fixture.map,
                                  base_reference));
    // Changed, with the reference loaded as the frontends load it.
    c.change(fixture.config);
    model::ZvsTrajectory changed(fixture.reference_path.string(), fixture.map,
                                 fixture.config.reference);
    const auto grid =
        follow::preflightTrackingGrid(changed.duration(), fixture.config);
    REQUIRE(grid.samples() == zero.delta_rad.size());
    REQUIRE(std::fabs(grid.dt_s - zero.dt_s) <=
            learn::kOffsetGridStepToleranceS);
    CHECK_THROWS_WITH(
        follow::preflightOffset(fixture.config, fixture.chain, fixture.map,
                                changed),
        ContainsSubstring("does not match the reference being run") &&
            ContainsSubstring(
                "; the offset's reference_sha256 is not this run's reference "
                "signature, which covers the reference file's SHA-256, "
                "control.rate_hz, control.mode and every "
                "[reference_processing] setting"));
    fs::remove(fixture.offset_path);
  }
}

TEST_CASE("the reference signature is the SHA-256 of its documented text",
          "[follow][offset][signature]") {
  follow::Config config;
  const std::string file(64, 'a');
  const std::string text = follow::referenceSignatureText(config, file);
  CHECK(text ==
        "rtctrl-x7-follow-reference-signature 1\n"
        "reference_file_sha256 " + file + "\n"
        "control.rate_hz 100\n"
        "control.mode 3\n"
        "interpolation shape-preserving-cubic\n"
        "filter none\n"
        "low_pass_cutoff_hz 5\n"
        "filter_window 5\n"
        "savitzky_golay_order 3\n"
        "mimic_tolerance 1.0000000000000001e-09\n");
  CHECK(follow::referenceSignature(config, file) == x7::ptp::sha256Text(text));
  config.control_rate_hz = 1.0 / 3.0;
  config.mode = arm::ControlMode::CurrentBasedPosition;
  config.reference.interpolation = model::ReferenceInterpolation::Linear;
  config.reference.filter = model::ReferenceFilter::SavitzkyGolay;
  config.reference.low_pass_cutoff_hz = 0.1;
  config.reference.filter_window = 7;
  config.reference.savitzky_golay_order = 2;
  config.reference.mimic_tolerance = 0.0;
  CHECK(follow::referenceSignatureText(config, file) ==
        "rtctrl-x7-follow-reference-signature 1\n"
        "reference_file_sha256 " + file + "\n"
        "control.rate_hz 0.33333333333333331\n"
        "control.mode 5\n"
        "interpolation linear\n"
        "filter savitzky-golay\n"
        "low_pass_cutoff_hz 0.10000000000000001\n"
        "filter_window 7\n"
        "savitzky_golay_order 2\n"
        "mimic_tolerance 0\n");
  config.reference.filter = model::ReferenceFilter::MovingAverage;
  CHECK_THAT(follow::referenceSignatureText(config, file),
             ContainsSubstring("\nfilter moving-average\n"));

  // Every field moves the signature, the unused ones too.
  const follow::Config base;
  const std::string reference = follow::referenceSignature(base, file);
  const std::vector<std::function<void(follow::Config&)>> changes = {
      [](follow::Config& c) { c.control_rate_hz = 100.00000000000001; },
      [](follow::Config& c) { c.mode = arm::ControlMode::CurrentBasedPosition; },
      [](follow::Config& c) {
        c.reference.interpolation = model::ReferenceInterpolation::Linear;
      },
      [](follow::Config& c) {
        c.reference.filter = model::ReferenceFilter::LowPass;
      },
      [](follow::Config& c) { c.reference.low_pass_cutoff_hz = 5.5; },
      [](follow::Config& c) { c.reference.filter_window = 7; },
      [](follow::Config& c) { c.reference.savitzky_golay_order = 2; },
      [](follow::Config& c) { c.reference.mimic_tolerance = 2e-9; }};
  for (std::size_t k = 0; k < changes.size(); ++k) {
    INFO("change " << k);
    follow::Config changed = base;
    changes[k](changed);
    CHECK(follow::referenceSignature(changed, file) != reference);
  }
  CHECK(follow::referenceSignature(base, std::string(64, 'b')) != reference);
  // Not bound: the servo gains, the model, homing, safety.
  follow::Config unbound = base;
  unbound.simulation.position_kp = 20.0;
  unbound.model_path = "other.ztk";
  unbound.home.velocity_limit = 0.1;
  unbound.safety.warning_error_rad = 0.1;
  unbound.effort_limit_nm.fill(1.0);
  CHECK(follow::referenceSignature(unbound, file) == reference);

  // The one-argument form hashes config.reference_path.
  Fixture fixture;
  CHECK(fixture.signature ==
        follow::referenceSignature(
            fixture.config, x7::ptp::sha256File(fixture.reference_path)));
}

TEST_CASE("follow preflight refuses an offset off the run's tracking grid",
          "[follow][preflight][offset]") {
  Fixture fixture;
  // Made for a 200 Hz run.
  CHECK_THROWS_WITH(
      fixture.preflight(learn::zeroOffset(fixture.reference.duration(),
                                          0.005, fixture.signature)),
      ContainsSubstring("samples, the tracking grid"));
  // Made for 100 Hz, run at 50 Hz.
  fixture.config.control_rate_hz = 50.0;
  CHECK_THROWS_WITH(
      fixture.preflight(learn::zeroOffset(fixture.reference.duration(), 0.01,
                                          fixture.signature)),
      ContainsSubstring("samples, the tracking grid"));
}

TEST_CASE("follow preflight refuses an offset beyond the configured limits",
          "[follow][preflight][offset]") {
  Fixture fixture;
  const auto constant = [&](double value) {
    return learned(
        fixture.reference, fixture.signature,
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
  const auto step = learned(fixture.reference, fixture.signature,
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
  const auto signature = follow::referenceSignature(fixture.config);
  const auto offset_path = writeOffset(
      "follow_offset_limit.csv",
      learned(near, signature,
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
      learned(low, follow::referenceSignature(fixture.config),
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
                                          fixture.signature)),
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
                                      fixture.signature);
  const auto text = learn::formatCommandOffset(zero);
  const fs::path path = scratch("follow_offset_truncated.csv");
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
