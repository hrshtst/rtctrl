#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

#include "follow/posture_reference.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

using Catch::Approx;
namespace follow = x7::follow;
namespace model = rtctrl::model;

namespace {
const follow::CanonicalPosture kQ0 = {0.1, -0.2, 0.3, -1.4, 0.05, 0.6, -0.7,
                                      0.2};
}  // namespace

TEST_CASE("posture reference specs are parsed strictly", "[follow][posture]") {
  const auto hold = follow::parseHoldCurrent("60");
  CHECK(hold.kind == follow::PostureReference::Kind::kHold);
  CHECK(hold.duration_s == 60.0);
  for (const char* bad : {"0", "-1", "601", "abc", "1x", ""}) {
    CAPTURE(bad);
    CHECK_THROWS(follow::parseHoldCurrent(bad));
  }

  const auto sweep = follow::parseSweepCurrent("6:-0.3:6");
  CHECK(sweep.kind == follow::PostureReference::Kind::kSweep);
  CHECK(sweep.joint == 6);
  CHECK(sweep.amplitude_rad == -0.3);
  CHECK(sweep.duration_s == 6.0);
  for (const char* bad :
       {"6:0.3", "6:0.3:6:1", "7:0.3:6", "-1:0.3:6", "2.5:0.3:6", "6:0:6",
        "6:1.5:6", "6:0.3:0.5", "6:0.3:121", "x:0.3:6"}) {
    CAPTURE(bad);
    CHECK_THROWS(follow::parseSweepCurrent(bad));
  }
}

TEST_CASE("a hold reference is the measured posture throughout",
          "[follow][posture]") {
  const auto frames =
      follow::postureReferenceFrames(follow::parseHoldCurrent("2"), kQ0);
  CHECK(frames.size() == 201);
  for (const auto& q : frames) CHECK(q == kQ0);
}

TEST_CASE("a sweep moves one joint out and back from the measured posture",
          "[follow][posture]") {
  const auto spec = follow::parseSweepCurrent("6:0.3:6");
  const auto frames = follow::postureReferenceFrames(spec, kQ0);
  REQUIRE(frames.size() == 601);
  CHECK(frames.front() == kQ0);  // exactly: no homing is ever needed
  CHECK(frames.back() == kQ0);
  CHECK(frames[300][6] == Approx(kQ0[6] + 0.3));
  double peak_speed = 0.0;
  for (std::size_t k = 0; k < frames.size(); ++k) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (i != 6) CHECK(frames[k][i] == kQ0[i]);
    }
    if (k > 0) {
      peak_speed = std::max(
          peak_speed, std::fabs(frames[k][6] - frames[k - 1][6]) /
                          follow::kPostureFrameS);
    }
  }
  // Minimum-jerk halves of 3 s: 15/8 * 0.3 / 3 = 0.1875 rad/s.
  CHECK(peak_speed == Approx(0.1875).epsilon(0.01));
}

TEST_CASE("posture references round-trip through the ZVS reader and are "
          "never overwritten",
          "[follow][posture]") {
  const std::filesystem::path path = "build/posture_reference_test.zvs";
  std::filesystem::remove(path);
  const auto frames = follow::postureReferenceFrames(
      follow::parseSweepCurrent("3:-0.2:2"), kQ0);
  follow::writePostureReferenceZvs(path, frames);
  CHECK_THROWS(follow::writePostureReferenceZvs(path, frames));

  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  model::ZvsTrajectory reference(path.string(), map);
  CHECK(reference.duration() ==
        Approx(frames.size() * follow::kPostureFrameS));
  model::ZVector q(model::kCanonicalDof);
  reference.sample(0.0, q.get());
  for (int i = 0; i < model::kCanonicalDof; ++i) CHECK(q[i] == kQ0[i]);
  reference.sample(1.0, q.get());
  CHECK(q[3] == Approx(kQ0[3] - 0.2));
  std::filesystem::remove(path);
}
