#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/ptp_planner.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

using Catch::Approx;
using namespace rtctrl::model;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

CartesianPose pose(double x, double y, double z, double roll = 0.0,
                   double pitch = 0.0, double yaw = 0.0) {
  CartesianPose value;
  zVec3DCreate(&value.position, x, y, z);
  zMat3DFromZYX(&value.attitude, yaw, pitch, roll);
  return value;
}

}  // namespace

TEST_CASE("PTP progress laws have the requested boundary behavior",
          "[ptp][trajectory]") {
  for (const auto profile : {PtpProfile::Linear, PtpProfile::Trapezoidal,
                             PtpProfile::MinimumJerk}) {
    CHECK(ptpProgress(profile, 0.0).position == Approx(0.0));
    CHECK(ptpProgress(profile, 1.0).position == Approx(1.0));
    CHECK(ptpProgress(profile, 0.5).position == Approx(0.5));
    double previous = 0.0;
    double observed_peak = 0.0;
    for (int i = 0; i <= 10000; ++i) {
      const auto sample = ptpProgress(profile, i / 10000.0);
      CHECK(sample.position >= previous - 1e-12);
      previous = sample.position;
      observed_peak = std::max(observed_peak, sample.velocity);
    }
    CHECK(observed_peak == Approx(ptpPeakSpeedFactor(profile)).epsilon(1e-6));
  }
  CHECK(ptpProgress(PtpProfile::Trapezoidal, 0.0).velocity == Approx(0.0));
  CHECK(ptpProgress(PtpProfile::Trapezoidal, 1.0).velocity == Approx(0.0));
  CHECK(ptpProgress(PtpProfile::MinimumJerk, 0.0).velocity == Approx(0.0));
  CHECK(ptpProgress(PtpProfile::MinimumJerk, 1.0).velocity == Approx(0.0));
}

TEST_CASE("PTP timing uses the longest requested constraint",
          "[ptp][trajectory]") {
  const auto start = pose(0.0, 0.0, 0.0);
  const auto end = pose(0.2, 0.0, 0.0, 0.0, 0.0, 0.4);

  PtpTiming timing;
  CHECK(choosePtpDuration(start, end, PtpProfile::Linear, timing) ==
        Approx(5.0));

  timing.motion_time = 3.0;
  timing.max_linear_velocity = 0.1;
  timing.max_angular_velocity = 0.1;
  CHECK(choosePtpDuration(start, end, PtpProfile::Linear, timing) ==
        Approx(4.0));
  CHECK(choosePtpDuration(start, end, PtpProfile::MinimumJerk, timing) ==
        Approx(7.5));

  timing.max_angular_velocity.reset();
  CHECK_THROWS(choosePtpDuration(start, end, PtpProfile::Linear, timing));
}

TEST_CASE("Cartesian PTP interpolation follows a line and shortest rotation",
          "[ptp][trajectory]") {
  const auto start = pose(0.1, -0.2, 0.3, 0.0, 0.0, zDeg2Rad(170.0));
  const auto end = pose(0.3, 0.2, 0.5, 0.0, 0.0, zDeg2Rad(-170.0));
  const auto middle = interpolateCartesianPose(start, end, 0.5);
  CHECK(middle.position.c.x == Approx(0.2));
  CHECK(middle.position.c.y == Approx(0.0));
  CHECK(middle.position.c.z == Approx(0.4));

  zMat3D expected;
  zMat3DFromZYX(&expected, M_PI, 0.0, 0.0);
  zVec3D error;
  zMat3DError(&expected, &middle.attitude, &error);
  CHECK(zVec3DNorm(&error) == Approx(0.0).margin(1e-9));
  CHECK(cartesianRotationDistance(start, end) ==
        Approx(zDeg2Rad(20.0)).margin(1e-9));
}

TEST_CASE("Cartesian PTP planner solves every sample by continuation",
          "[ptp][ik]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  ZVector initial(kModelDof);
  PtpPlanOptions options;
  options.timing.motion_time = 0.2;
  options.sample_rate = 20.0;

  PtpPlan plan_result;
  {
    CartesianPtpPlanner planner(model, map);
    plan_result = planner.plan(pose(0.2, 0.0, 0.25),
                               pose(0.21, 0.0, 0.25), initial.get(), options);
  }
  REQUIRE(plan_result.samples.size() == 5);
  CHECK(plan_result.duration == Approx(0.2));
  CHECK(plan_result.interval == Approx(0.05));
  CHECK(plan_result.ik_warnings.empty());

  const int tcp = model.linkIndex("crane_x7_tcp_link");
  for (std::size_t i = 0; i < plan_result.samples.size(); ++i) {
    ZVector q(kModelDof);
    for (int joint = 0; joint < kModelDof; ++joint) {
      q[joint] = plan_result.samples[i].displacement[joint];
    }
    model.fk(q.get());
    const double u = static_cast<double>(i) / 4.0;
    const double expected_x =
        0.2 + 0.01 * ptpProgress(PtpProfile::MinimumJerk, u).position;
    CHECK(rkChainLinkWldPos(model.chain(), tcp)->c.x ==
          Approx(expected_x).margin(1e-4));
    CHECK(plan_result.samples[i].displacement[map.rokiOffsetFingerB()] ==
          Approx(plan_result.samples[i].displacement[map.rokiOffset(7)]));
  }
}

TEST_CASE("Cartesian PTP strictness controls finite IK failures",
          "[ptp][ik]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  ZVector initial(kModelDof);
  const auto target = pose(0.201234, 0.0, 0.25);

  PtpPlanOptions options;
  options.timing.motion_time = 0.1;
  options.sample_rate = 10.0;
  options.position_tolerance = 1e-16;
  options.attitude_tolerance = 1e-16;
  CartesianPtpPlanner planner(model, map);
  CHECK_THROWS_AS(planner.plan(target, target, initial.get(), options),
                  PtpPlanningError);

  options.strict_ik = false;
  const auto result = planner.plan(target, target, initial.get(), options);
  CHECK(result.samples.size() == 2);
  REQUIRE_FALSE(result.ik_warnings.empty());
  CHECK(result.ik_warnings.front().result.finite);
  CHECK(result.ik_warnings.front().result.within_limits);
}

TEST_CASE("zero-distance velocity-only PTP emits one posture", "[ptp]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  ZVector initial(kModelDof);
  PtpPlanOptions options;
  options.timing.max_linear_velocity = 0.1;
  options.timing.max_angular_velocity = 0.5;
  CartesianPtpPlanner planner(model, map);
  const auto target = pose(0.2, 0.0, 0.25);
  const auto result = planner.plan(target, target, initial.get(), options);
  CHECK(result.duration == Approx(0.0));
  CHECK(result.samples.size() == 1);
  CHECK(result.interval == Approx(0.01));
}

TEST_CASE("Cartesian PTP output is a parseable 9-DOF zvs sequence",
          "[ptp][zvs]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  ZVector initial(kModelDof);
  PtpPlanOptions options;
  options.timing.motion_time = 0.1;
  options.sample_rate = 20.0;
  CartesianPtpPlanner planner(model, map);
  const auto plan = planner.plan(pose(0.2, 0.0, 0.25),
                                 pose(0.205, 0.0, 0.25), initial.get(),
                                 options);

  const char* path = "build/test_ptp_plan.zvs";
  {
    ZvsWriter writer(path);
    ZVector displacement(kModelDof);
    for (const auto& sample : plan.samples) {
      for (int i = 0; i < kModelDof; ++i) {
        displacement[i] = sample.displacement[i];
      }
      writer.frame(plan.interval, displacement.get());
    }
    CHECK(writer.frames() == static_cast<int>(plan.samples.size()));
  }

  zSeq sequence;
  REQUIRE(zSeqScanFile(&sequence, const_cast<char*>(path)));
  int frames = 0;
  zSeqCell* cell;
  zListForEach(&sequence, cell) {
    CHECK(cell->data.dt == Approx(plan.interval));
    CHECK(zVecSizeNC(cell->data.v) == kModelDof);
    ++frames;
  }
  CHECK(frames == static_cast<int>(plan.samples.size()));
  zSeqFree(&sequence);
  std::remove(path);
}
