#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

using Catch::Approx;
using rtctrl::model::MinJerkTrajectory;
using rtctrl::model::SinusoidTrajectory;
using rtctrl::model::ZvsWriter;
using rtctrl::model::ZVector;

namespace {
constexpr int kDof = 8;
constexpr double kVelocityLimit = 4.81710873;
}  // namespace

TEST_CASE("min-jerk trajectory hits its boundary conditions",
          "[trajectory]") {
  ZVector q0(kDof), qf(kDof), q(kDof), dq(kDof);
  for (int i = 0; i < kDof; ++i) {
    q0[i] = 0.1 * i;
    qf[i] = 1.0 - 0.2 * i;
  }
  MinJerkTrajectory traj(q0, qf, 2.0);

  traj.sample(0.0, q, dq);
  for (int i = 0; i < kDof; ++i) {
    CHECK(q[i] == Approx(q0[i]));
    CHECK(dq[i] == Approx(0.0).margin(1e-12));
  }
  traj.sample(traj.duration(), q, dq);
  for (int i = 0; i < kDof; ++i) {
    CHECK(q[i] == Approx(qf[i]));
    CHECK(dq[i] == Approx(0.0).margin(1e-12));
  }
  // clamping beyond the end
  traj.sample(traj.duration() + 5.0, q, dq);
  CHECK(q[0] == Approx(qf[0]));
  CHECK(dq[0] == Approx(0.0).margin(1e-12));
}

TEST_CASE("velocity-limited min-jerk stays within the limit",
          "[trajectory]") {
  ZVector q0(kDof), qf(kDof), q(kDof), dq(kDof);
  for (int i = 0; i < kDof; ++i) qf[i] = 2.8;  // large swing
  const auto traj =
      MinJerkTrajectory::withVelocityLimit(q0, qf, kVelocityLimit);

  double peak = 0.0;
  for (double t = 0.0; t <= traj.duration(); t += traj.duration() / 2000.0) {
    traj.sample(t, q, dq);
    for (int i = 0; i < kDof; ++i) peak = std::max(peak, std::fabs(dq[i]));
  }
  CHECK(peak <= kVelocityLimit * (1.0 + 1e-9));
  CHECK(peak == Approx(kVelocityLimit).epsilon(1e-3));  // limit is tight
}

TEST_CASE("trajectory constructors validate their inputs", "[trajectory]") {
  ZVector a(kDof), b(kDof - 1);
  CHECK_THROWS(MinJerkTrajectory(a, b, 1.0));
  CHECK_THROWS(MinJerkTrajectory(a, a, 0.0));
  CHECK_THROWS(SinusoidTrajectory(a, b, 1.0));
  CHECK_THROWS(SinusoidTrajectory(a, a, -1.0));
}

TEST_CASE("sinusoid oscillates around its center", "[trajectory]") {
  ZVector c(kDof), amp(kDof), q(kDof), dq(kDof);
  for (int i = 0; i < kDof; ++i) {
    c[i] = 0.5;
    amp[i] = 0.25;
  }
  SinusoidTrajectory traj(c, amp, 2.0);

  traj.sample(0.0, q, dq);
  CHECK(q[0] == Approx(0.5));
  traj.sample(0.5, q);  // quarter period: peak
  CHECK(q[0] == Approx(0.75));
  double lo = 1e9, hi = -1e9;
  for (double t = 0.0; t < 4.0; t += 0.001) {
    traj.sample(t, q);
    lo = std::min(lo, q[0]);
    hi = std::max(hi, q[0]);
  }
  CHECK(lo == Approx(0.25).margin(1e-4));
  CHECK(hi == Approx(0.75).margin(1e-4));
}

TEST_CASE("zvs output re-parses as a zm sequence", "[trajectory][zvs]") {
  const char* path = "build/test_motion.zvs";
  constexpr int kFrames = 25;
  constexpr double kDt = 0.01;
  {
    ZVector q(kDof);
    ZvsWriter writer(path);
    for (int f = 0; f < kFrames; ++f) {
      q[0] = 0.01 * f;
      writer.frame(kDt, q);
    }
    CHECK(writer.frames() == kFrames);
  }

  zSeq seq;
  REQUIRE(zSeqScanFile(&seq, const_cast<char*>(path)));
  int count = 0;
  zSeqCell* cp;
  zListForEach(&seq, cp) {
    CHECK(cp->data.dt == Approx(kDt));
    CHECK(zVecSizeNC(cp->data.v) == kDof);
    ++count;
  }
  CHECK(count == kFrames);
  zSeqFree(&seq);
  std::remove(path);
}

TEST_CASE("round trip is C2-continuous at the split", "[trajectory]") {
  using rtctrl::model::MinJerkTrajectory;
  using rtctrl::model::RoundTripTrajectory;
  using rtctrl::model::ZVector;
  ZVector q0(3), qf(3);
  q0[0] = 0.1;
  qf[0] = 0.6;
  qf[1] = -0.4;
  qf[2] = 0.2;
  RoundTripTrajectory rt(MinJerkTrajectory(q0.get(), qf.get(), 2.0),
                         MinJerkTrajectory(qf.get(), q0.get(), 1.5));
  CHECK(rt.outDuration() == Approx(2.0));
  CHECK(rt.duration() == Approx(3.5));
  CHECK(rt.size() == 3);

  // position, velocity and acceleration continuous across the split
  ZVector qa(3), dqa(3), dda(3), qb(3), dqb(3), ddb(3);
  const double eps = 1e-6;
  rt.sample(2.0 - eps, qa.get(), dqa.get(), dda.get());
  rt.sample(2.0 + eps, qb.get(), dqb.get(), ddb.get());
  for (int i = 0; i < 3; ++i) {
    CHECK(qa[i] == Approx(qb[i]).margin(1e-6));
    CHECK(dqa[i] == Approx(dqb[i]).margin(1e-4));   // both ~0 (min-jerk ends)
    CHECK(dda[i] == Approx(ddb[i]).margin(1e-3));   // both ~0
    CHECK(std::fabs(dqa[i]) < 1e-4);
    CHECK(std::fabs(dda[i]) < 1e-3);
  }

  // endpoints clamp: before 0 and past the total duration
  ZVector q(3);
  rt.sample(-1.0, q.get());
  CHECK(q[0] == Approx(q0[0]));
  rt.sample(99.0, q.get());
  CHECK(q[0] == Approx(q0[0]));  // returns to start
  rt.sample(2.0, q.get());
  CHECK(q[0] == Approx(qf[0]));  // split sits at the turn pose

  // mismatched segments are rejected
  ZVector other(3);
  other[0] = 9.0;
  CHECK_THROWS(RoundTripTrajectory(
      MinJerkTrajectory(q0.get(), qf.get(), 1.0),
      MinJerkTrajectory(other.get(), q0.get(), 1.0)));
}
