#include "rtctrl/model/force_budget.hpp"

#include <array>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/link_force.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

using Catch::Approx;
using Catch::Matchers::ContainsSubstring;
using rtctrl::model::BudgetStatus;
using rtctrl::model::ChainModel;
using rtctrl::model::ForceBudgetOptions;
using rtctrl::model::ForceDirection;
using rtctrl::model::JointBound;
using rtctrl::model::jointForceBound;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::LinkForceSpec;
using rtctrl::model::ZVector;

namespace {

namespace fs = std::filesystem;

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

using Pose = std::array<double, kCanonicalDof>;

// Linear joint-space reference with test hooks (size, NaN injection).
class TestTrajectory : public rtctrl::model::Trajectory {
 public:
  TestTrajectory(const Pose& from, const Pose& to, double duration,
                 int size = kCanonicalDof, double nan_from = kInf)
      : from_(from),
        to_(to),
        duration_(duration),
        size_(size),
        nan_from_(nan_from) {}
  int size() const override { return size_; }
  double duration() const override { return duration_; }
  void sample(double t, zVec q, zVec = nullptr, zVec = nullptr) const override {
    const double s = duration_ > 0.0 ? t / duration_ : 0.0;
    for (int i = 0; i < size_ && i < kCanonicalDof; ++i) {
      zVecElemNC(q, i) = from_[i] + s * (to_[i] - from_[i]);
    }
    if (t >= nan_from_) zVecElemNC(q, 2) = kNaN;
  }

 private:
  Pose from_, to_;
  double duration_;
  int size_;
  double nan_from_;
};

constexpr Pose kZero{};
constexpr Pose kBent = {0.3, -0.6, 0.2, -1.4, 0.25, -0.5, 0.4, 0.3};

LinkForceSpec tcpSpec(const std::array<double, 3>& direction) {
  LinkForceSpec spec;
  spec.link = "crane_x7_tcp_link";
  spec.direction = ForceDirection::World;
  spec.direction_world = direction;
  spec.magnitude_n = 1.0;
  spec.profile.rise_s = 0.1;
  spec.profile.fall_s = 0.1;
  return spec;
}

ForceBudgetOptions allowances(double value) {
  ForceBudgetOptions options;
  options.joint_allowance_nm.fill(value);
  options.sample_rate_hz = 100.0;
  return options;
}

struct Fixture {
  ChainModel model{kModelPath};
  JointMap map{model};
};

// Rigid-body lever formula, independent of the Jacobian code:
// (Jᵀu)_i = ((p - o_i) x u) . z_i for the revolute joint at link origin o_i
// with world axis z_i (roki's revolute axis is the link z axis).
double leverTorque(ChainModel& model, int link, const std::array<double, 3>& p,
                   const std::array<double, 3>& u) {
  const zVec3D* o = rkChainLinkWldPos(model.chain(), link);
  const zVec3D& z = rkChainLinkWldAtt(model.chain(), link)->v[2];
  const double r[3] = {p[0] - o->c.x, p[1] - o->c.y, p[2] - o->c.z};
  const double m[3] = {r[1] * u[2] - r[2] * u[1], r[2] * u[0] - r[0] * u[2],
                       r[0] * u[1] - r[1] * u[0]};
  return m[0] * z.c.x + m[1] * z.c.y + m[2] * z.c.z;
}

bool satisfies(double g, double a, double allow, double force) {
  return std::abs(g - a * force) <= allow * (1.0 + 1e-12);
}

class TempPath {
 public:
  explicit TempPath(const std::string& name) {
    fs::create_directories("build/force_budget_test");
    path_ = "build/force_budget_test/" + name;
    std::remove(path_.c_str());
  }
  ~TempPath() { std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

}  // namespace

TEST_CASE("joint force bound admits every amplitude up to it",
          "[force_budget][bound]") {
  struct Case {
    double g, a, allow, expected;
  };
  const Case cases[] = {
      {0.5, 0.2, 1.0, 7.5},     // a > 0: (allow + g) / a
      {0.5, -0.2, 1.0, 2.5},    // a < 0: (allow - g) / |a|
      {-1.0, 0.5, 1.0, 0.0},    // F_max = 0: the load already sits at -allow
      {1.0, 1.0, 1.0, 2.0},     // |g| = allow edge
      {-0.3, -0.4, 2.0, 5.75},  // a < 0 with negative load
  };
  for (const auto& c : cases) {
    INFO("g=" << c.g << " a=" << c.a << " allow=" << c.allow);
    const JointBound bound = jointForceBound(c.g, c.a, c.allow);
    REQUIRE(bound.kind == JointBound::Kind::Bounded);
    CHECK(bound.value == Approx(c.expected).margin(1e-12));
    for (int k = 0; k <= 1000; ++k) {
      CHECK(satisfies(c.g, c.a, c.allow, bound.value * k / 1000.0));
    }
    // additive perturbation: also meaningful when F_max = 0
    CHECK_FALSE(satisfies(c.g, c.a, c.allow, bound.value + 1e-6));
  }
}

TEST_CASE("joint force bound classifies infeasible, unbounded and overflow",
          "[force_budget][bound]") {
  // a positive force would compensate this load, but F = 0 must hold too
  const JointBound infeasible = jointForceBound(1.5, 1.0, 1.0);
  CHECK(infeasible.kind == JointBound::Kind::Infeasible);
  CHECK(std::isnan(infeasible.value));

  const JointBound unbounded = jointForceBound(0.4, 0.0, 1.0);
  CHECK(unbounded.kind == JointBound::Kind::Unbounded);
  CHECK(unbounded.value == kInf);

  // same sign: (allow + g) would overflow; dividing first gives 2
  const JointBound same = jointForceBound(1e308, 1e308, 1e308);
  REQUIRE(same.kind == JointBound::Kind::Bounded);
  CHECK(same.value == Approx(2.0));

  // cancellation: allow / |a| and g / |a| each overflow, the bound does not
  const JointBound cancel = jointForceBound(-9e307, 0.5, 1e308);
  REQUIRE(cancel.kind == JointBound::Kind::Bounded);
  CHECK(cancel.value == Approx(2e307).epsilon(1e-12));

  // genuinely unrepresentable
  const JointBound overflow = jointForceBound(1e308, 0.5, 1e308);
  CHECK(overflow.kind == JointBound::Kind::Overflow);
  CHECK(std::isnan(overflow.value));
  CHECK(jointForceBound(0.0, 1e-320, 1.0).kind == JointBound::Kind::Overflow);
}

TEST_CASE("joint force bound rejects invalid arguments",
          "[force_budget][bound]") {
  CHECK_THROWS_AS(jointForceBound(kNaN, 1.0, 1.0), std::invalid_argument);
  CHECK_THROWS_AS(jointForceBound(0.0, kInf, 1.0), std::invalid_argument);
  CHECK_THROWS_AS(jointForceBound(0.0, 1.0, 0.0), std::invalid_argument);
  CHECK_THROWS_AS(jointForceBound(0.0, 1.0, -1.0), std::invalid_argument);
  CHECK_THROWS_AS(jointForceBound(0.0, 1.0, kInf), std::invalid_argument);
}

TEST_CASE_METHOD(Fixture,
                 "force budget matches the lever formula at the upright pose",
                 "[force_budget]") {
  const auto spec = tcpSpec({1.0, 0.0, 0.0});
  TestTrajectory reference(kZero, kZero, 0.0);
  auto options = allowances(100.0);
  options.joint_allowance_nm[1] = 0.5;  // make the shoulder pitch bind

  const auto result =
      rtctrl::model::evaluateForceBudget(model, map, reference, spec, options);
  REQUIRE(result.status == BudgetStatus::Ok);
  REQUIRE(result.samples.size() == 1);  // zero duration: t = 0 only
  const auto& sample = result.samples[0];
  CHECK(sample.time_s == 0.0);

  ZVector q9(rtctrl::model::kModelDof);
  model.fk(q9.get());
  const auto p = model.linkPointWorldPos(model.linkIndex(spec.link), {});
  // upright: the TCP sits on the vertical pan axis, so the pan joint sees
  // no torque from a horizontal force
  CHECK(std::abs(p[0]) < 1e-9);
  CHECK(std::abs(p[1]) < 1e-9);
  const std::array<double, 3> u = {1.0, 0.0, 0.0};
  for (int i = 0; i < kCanonicalDof - 1; ++i) {
    INFO("canonical joint " << i);
    CHECK(sample.torque_per_newton[i] ==
          Approx(leverTorque(model, map.linkId(i), p, u)).margin(1e-12));
  }
  CHECK(sample.torque_per_newton[0] == Approx(0.0).margin(1e-12));
  CHECK(sample.torque_per_newton[kCanonicalDof - 1] == 0.0);  // not a child
  CHECK(std::isinf(sample.joint_bound_n[kCanonicalDof - 1]));

  const double g1 = sample.gravity_nm[1];
  const double a1 = sample.torque_per_newton[1];
  REQUIRE(std::abs(a1) > 0.1);
  REQUIRE(std::abs(g1) < 0.5);
  const double expected = (0.5 + (a1 > 0 ? g1 : -g1)) / std::abs(a1);
  CHECK(result.binding_joint == 1);
  CHECK(result.amplitude_bound_n == Approx(expected).margin(1e-12));
  CHECK(result.disturbance_only_bound_n ==
        Approx(0.5 / std::abs(a1)).margin(1e-12));
  CHECK(std::isnan(result.first_failure_time_s));
  CHECK(std::isnan(result.min_anchor_distance_m));  // World mode
}

TEST_CASE_METHOD(Fixture,
                 "force budget samples the whole path and keeps the "
                 "earliest binding time",
                 "[force_budget]") {
  const auto spec = tcpSpec({0.2, 1.0, -0.3});
  TestTrajectory reference(kZero, kBent, 2.0);
  const auto options = allowances(3.0);
  const auto result =
      rtctrl::model::evaluateForceBudget(model, map, reference, spec, options);
  REQUIRE(result.status == BudgetStatus::Ok);
  REQUIRE(result.samples.size() == 201);
  CHECK(result.samples.front().time_s == 0.0);
  CHECK(result.samples.back().time_s == 2.0);

  // brute force over the same samples with the C1 primitives
  ZVector q8(kCanonicalDof), a8(kCanonicalDof), g8(kCanonicalDof);
  const int link = model.linkIndex(spec.link);
  std::array<double, 3> u{};
  double best = kInf, best_time = kNaN;
  int best_joint = -1;
  for (const auto& sample : result.samples) {
    reference.sample(sample.time_s, q8.get());
    REQUIRE(rtctrl::model::linkForceDirection(spec, {}, &u));
    model.forceJointTorque(map, q8.get(), link, spec.point_link_m, u, a8.get());
    model.gravityTorque(map, q8.get(), g8.get());
    for (int i = 0; i < kCanonicalDof; ++i) {
      CHECK(sample.torque_per_newton[i] == a8[i]);
      CHECK(sample.gravity_nm[i] == g8[i]);
      const auto jb = jointForceBound(g8[i], a8[i], 3.0);
      REQUIRE(jb.kind != JointBound::Kind::Infeasible);
      if (jb.value < best) {
        best = jb.value;
        best_time = sample.time_s;
        best_joint = i;
      }
    }
  }
  CHECK(result.amplitude_bound_n == best);
  CHECK(result.binding_time_s == best_time);
  CHECK(result.binding_joint == best_joint);
}

TEST_CASE_METHOD(Fixture,
                 "force budget follows an anchor and flags "
                 "degenerate anchors as failures",
                 "[force_budget]") {
  ZVector q9(rtctrl::model::kModelDof);
  model.fk(q9.get());
  const auto p =
      model.linkPointWorldPos(model.linkIndex("crane_x7_tcp_link"), {});
  LinkForceSpec spec = tcpSpec({1.0, 0.0, 0.0});
  spec.direction = ForceDirection::Anchor;
  spec.anchor_world_m = {p[0] + 0.3, p[1] - 0.4, p[2]};
  TestTrajectory still(kZero, kZero, 0.5);
  auto result = rtctrl::model::evaluateForceBudget(model, map, still, spec,
                                                   allowances(100.0));
  REQUIRE(result.status == BudgetStatus::Ok);
  CHECK(result.samples[0].direction_world[0] == Approx(0.6));
  CHECK(result.samples[0].direction_world[1] == Approx(-0.8));
  CHECK(result.min_anchor_distance_m == Approx(0.5));

  spec.anchor_world_m = {p[0], p[1], p[2] + 0.0005};
  result = rtctrl::model::evaluateForceBudget(model, map, still, spec,
                                              allowances(100.0));
  CHECK(result.status == BudgetStatus::DegenerateAnchor);
  CHECK(result.first_failure_time_s == 0.0);
  CHECK(std::isnan(result.amplitude_bound_n));  // never an unbounded result
  CHECK(result.min_anchor_distance_m == Approx(0.0005));
  CHECK(std::isnan(result.samples[0].direction_world[0]));
}

TEST_CASE_METHOD(Fixture,
                 "force budget reports infeasible and non-finite "
                 "evaluations as status",
                 "[force_budget]") {
  const auto spec = tcpSpec({0.2, 1.0, -0.3});
  TestTrajectory bent(kBent, kBent, 1.0);

  // a pose whose own gravity load exceeds a joint allowance
  ZVector q8(kCanonicalDof), g8(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) q8[i] = kBent[i];
  model.gravityTorque(map, q8.get(), g8.get());
  REQUIRE(std::abs(g8[1]) > 0.1);
  auto options = allowances(100.0);
  options.joint_allowance_nm[1] = 0.5 * std::abs(g8[1]);
  auto result =
      rtctrl::model::evaluateForceBudget(model, map, bent, spec, options);
  CHECK(result.status == BudgetStatus::Infeasible);
  CHECK(result.first_failure_joint == 1);
  CHECK(result.first_failure_time_s == 0.0);
  CHECK(std::isnan(result.amplitude_bound_n));
  CHECK(result.samples.size() == 101);  // evaluation continues

  // a reference that turns NaN partway: never Ok, never inf
  TestTrajectory broken(kZero, kBent, 1.0, kCanonicalDof, 0.5);
  result = rtctrl::model::evaluateForceBudget(model, map, broken, spec,
                                              allowances(100.0));
  CHECK(result.status == BudgetStatus::NonFiniteEvaluation);
  CHECK(result.first_failure_time_s == Approx(0.5));
  CHECK(std::isnan(result.amplitude_bound_n));
  CHECK(result.samples[50].status == BudgetStatus::NonFiniteEvaluation);
  CHECK(std::isnan(result.samples[50].gravity_nm[0]));
  CHECK(result.samples[49].status == BudgetStatus::Ok);

  // an allowance so large that a finite bound is not representable
  options = allowances(100.0);
  options.joint_allowance_nm[2] = 1e308;
  result = rtctrl::model::evaluateForceBudget(model, map, bent, spec, options);
  CHECK(result.status == BudgetStatus::NonFiniteEvaluation);
  CHECK(result.first_failure_joint == 2);
  CHECK(std::isnan(result.amplitude_bound_n));
}

TEST_CASE_METHOD(Fixture,
                 "force budget rejects invalid inputs before "
                 "sampling",
                 "[force_budget]") {
  const auto spec = tcpSpec({1.0, 0.0, 0.0});
  const auto options = allowances(1.0);
  auto expectThrow = [&](const rtctrl::model::Trajectory& reference,
                         const LinkForceSpec& s, const ForceBudgetOptions& o) {
    CHECK_THROWS_AS(
        rtctrl::model::evaluateForceBudget(model, map, reference, s, o),
        std::invalid_argument);
  };
  TestTrajectory good(kZero, kBent, 1.0);
  expectThrow(TestTrajectory(kZero, kBent, 1.0, 7), spec, options);
  expectThrow(TestTrajectory(kZero, kBent, -1.0), spec, options);
  expectThrow(TestTrajectory(kZero, kBent, kNaN), spec, options);
  expectThrow(TestTrajectory(kZero, kBent, kInf), spec, options);
  // 1e5 s at 1 kHz is 1e8 intervals
  auto fast = options;
  fast.sample_rate_hz = 1000.0;
  expectThrow(TestTrajectory(kZero, kBent, 1e5), spec, fast);
  auto bad = options;
  bad.sample_rate_hz = 0.0;
  expectThrow(good, spec, bad);
  bad.sample_rate_hz = kNaN;
  expectThrow(good, spec, bad);
  bad = options;
  bad.joint_allowance_nm[4] = 0.0;
  expectThrow(good, spec, bad);
  bad.joint_allowance_nm[4] = kNaN;
  expectThrow(good, spec, bad);
  auto unknown = spec;
  unknown.link = "no_such_link";
  expectThrow(good, unknown, options);
  auto invalid = spec;
  invalid.profile.rise_s = 0.0;
  expectThrow(good, invalid, options);
}

TEST_CASE_METHOD(Fixture, "force budget CSV carries the screening preamble",
                 "[force_budget][csv]") {
  const auto spec = tcpSpec({0.2, 1.0, -0.3});
  TestTrajectory reference(kZero, kBent, 0.5);
  auto options = allowances(3.0);
  options.sample_rate_hz = 20.0;
  const auto result =
      rtctrl::model::evaluateForceBudget(model, map, reference, spec, options);
  TempPath out("budget.csv");
  rtctrl::model::writeForceBudgetCsv(out.path(), result,
                                     {"sample_rate_hz: 20", "spec: test"});
  std::ifstream input(out.path());
  std::vector<std::string> comments, rows;
  std::string header, line;
  while (std::getline(input, line)) {
    if (line.rfind("# ", 0) == 0) {
      REQUIRE(header.empty());  // the preamble comes first
      comments.push_back(line);
    } else if (header.empty()) {
      header = line;
    } else {
      rows.push_back(line);
    }
  }
  const auto preamble = rtctrl::model::forceBudgetPreamble();
  REQUIRE(comments.size() == preamble.size() + 2);
  CHECK_THAT(comments[0],
             ContainsSubstring("compute the largest force amplitude "
                               "satisfying user-supplied joint "
                               "holding-torque allowances at sampled "
                               "reference poses under a quasi-static model"));
  bool qualified = false;
  for (const auto& c : comments) {
    qualified = qualified || c.find("not a hardware safety or fuse rating") !=
                                 std::string::npos;
  }
  CHECK(qualified);
  CHECK(comments.back() == "# spec: test");
  CHECK(header.rfind("schema_version,time_s,status,", 0) == 0);
  CHECK_THAT(header, ContainsSubstring("torque_per_newton7_nm_per_n"));
  CHECK(rows.size() == result.samples.size());
  CHECK(rows.front().rfind("1,0.000000000,ok,", 0) == 0);

  // exclusive creation
  CHECK_THROWS_AS(rtctrl::model::writeForceBudgetCsv(out.path(), result, {}),
                  std::runtime_error);
}
