#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"

using Catch::Approx;
using namespace rtctrl::model;

namespace {

struct Fixture {
  ChainModel chain{"models/crane_x7/crane_x7.ztk"};
  JointMap map{chain};
};

void writeReference(const char* path, int width, bool bad_mimic = false) {
  std::ofstream out(path);
  for (int frame = 0; frame < 5; ++frame) {
    out << (frame == 0 ? 0.02 : 0.01) << ' ' << width << " (";
    for (int joint = 0; joint < width; ++joint) {
      if (joint) out << ' ';
      double value = joint == 0 ? 0.1 * frame : 0.0;
      if (bad_mimic && width == 9 && joint == 8) value = 0.2;
      out << value;
    }
    out << ")\n";
  }
}

}  // namespace

TEST_CASE_METHOD(Fixture, "ZVS reference accepts canonical and model widths",
                 "[trajectory][zvs]") {
  for (const int width : {kCanonicalDof, kModelDof}) {
    const std::string path = "build/reference_" + std::to_string(width) + ".zvs";
    writeReference(path.c_str(), width);
    ZvsTrajectory trajectory(path, map);
    CHECK(trajectory.frames() == 5);
    CHECK(trajectory.duration() == Approx(0.06));
    ZVector q(kCanonicalDof), dq(kCanonicalDof), ddq(kCanonicalDof);
    trajectory.sample(0.025, q.get(), dq.get(), ddq.get());
    CHECK(q[0] > 0.1);
    CHECK(q[0] < 0.2);
    CHECK(dq[0] > 0.0);
    trajectory.sample(trajectory.duration(), q.get(), dq.get(), ddq.get());
    CHECK(q[0] == Approx(0.4));
    CHECK(dq[0] == Approx(0.0));
    std::remove(path.c_str());
  }
}

TEST_CASE_METHOD(Fixture, "ZVS reference rejects a broken mimic pair",
                 "[trajectory][zvs]") {
  const char* path = "build/reference_bad_mimic.zvs";
  writeReference(path, kModelDof, true);
  CHECK_THROWS(ZvsTrajectory(path, map));
  std::remove(path);
}

TEST_CASE_METHOD(Fixture, "reference filters preserve authored endpoints",
                 "[trajectory][filter]") {
  const char* path = "build/reference_filter.zvs";
  writeReference(path, kCanonicalDof);
  for (const auto filter : {ReferenceFilter::LowPass,
                            ReferenceFilter::MovingAverage,
                            ReferenceFilter::SavitzkyGolay}) {
    ZvsTrajectoryOptions options;
    options.filter = filter;
    options.filter_window = 5;
    options.savitzky_golay_order = 2;
    ZvsTrajectory trajectory(path, map, options);
    CHECK(trajectory.frame(0)[0] == Approx(0.0));
    CHECK(trajectory.frame(4)[0] == Approx(0.4));
  }
  std::remove(path);
}

TEST_CASE_METHOD(Fixture, "linear reference exposes segment velocity",
                 "[trajectory][zvs]") {
  const char* path = "build/reference_linear.zvs";
  writeReference(path, kCanonicalDof);
  ZvsTrajectoryOptions options;
  options.interpolation = ReferenceInterpolation::Linear;
  ZvsTrajectory trajectory(path, map, options);
  ZVector q(kCanonicalDof), dq(kCanonicalDof), ddq(kCanonicalDof);
  trajectory.sample(0.025, q.get(), dq.get(), ddq.get());
  CHECK(dq[0] == Approx(10.0));
  CHECK(ddq[0] == Approx(0.0));
  std::remove(path);
}
