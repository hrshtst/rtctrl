#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <toml++/toml.hpp>

#include <cmath>
#include <set>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"

using Catch::Approx;
using rtctrl::model::canonicalJoints;
using rtctrl::model::ChainModel;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::kModelDof;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr const char* kConfigPath = "config/crane_x7.toml";

// Ground truth extracted from the flat URDF (xacro expansion of
// third_party/crane_x7_description with gazebo/D435 off).
constexpr double kTotalMassKg = 1.8177;
constexpr double kGripperBaseZeroPoseZ = 0.624;
constexpr double kJoint4LowerRad = -2.8099800957108707;
constexpr double kJoint4UpperRad = 1.7453292519943296e-05;
constexpr double kVelocityLimit = 4.81710873;
constexpr double kEffortLimits[kCanonicalDof] = {10.0, 10.0, 4.0, 4.0,
                                                 4.0,  4.0,  4.0, 4.0};

struct ZVecGuard {
  explicit ZVecGuard(int size) : vec(zVecAlloc(size)) { zVecZero(vec); }
  ~ZVecGuard() { zVecFree(vec); }
  zVec vec;
};

}  // namespace

TEST_CASE("crane_x7.ztk loads with the expected structure", "[model]") {
  ChainModel model(kModelPath);
  CHECK(model.jointSize() == kModelDof);
  CHECK(model.totalMass() == Approx(kTotalMassKg).margin(1e-9));
}

TEST_CASE("joint 4 keeps its asymmetric limits", "[model]") {
  ChainModel model(kModelPath);
  const int idx = model.linkIndex("crane_x7_lower_arm_fixed_part_link");
  REQUIRE(idx >= 0);
  // the ztk stores degrees with limited precision; allow the round-trip
  CHECK(model.jointMin(idx) == Approx(kJoint4LowerRad).margin(1e-5));
  CHECK(model.jointMax(idx) == Approx(kJoint4UpperRad).margin(1e-5));
}

TEST_CASE("forward kinematics at zero pose matches the URDF chain",
          "[model]") {
  ChainModel model(kModelPath);
  ZVecGuard dis(model.jointSize());
  model.fk(dis.vec);
  const int idx = model.linkIndex("crane_x7_gripper_base_link");
  REQUIRE(idx >= 0);
  const zVec3D pos = model.linkWorldPos(idx);
  CHECK(pos.c.z == Approx(kGripperBaseZeroPoseZ).margin(1e-9));
  CHECK(pos.c.x == Approx(0.0).margin(1e-9));
  CHECK(pos.c.y == Approx(0.0).margin(1e-9));
}

TEST_CASE("JointMap resolves all canonical joints by name", "[model]") {
  ChainModel model(kModelPath);
  JointMap map(model);

  std::set<int> offsets;
  for (int i = 0; i < kCanonicalDof; ++i) {
    const int off = map.rokiOffset(i);
    CHECK(off >= 0);
    CHECK(off < kModelDof);
    offsets.insert(off);
  }
  offsets.insert(map.rokiOffsetFingerB());
  CHECK(offsets.size() == kModelDof);  // all distinct, covering the model
}

TEST_CASE("JointMap maps DXL IDs both ways", "[model]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  for (int i = 0; i < kCanonicalDof; ++i) {
    const auto id = map.dxlId(i);
    CHECK(id == i + 2);  // IDs 2..9 in canonical order
    REQUIRE(map.canonicalOf(id).has_value());
    CHECK(*map.canonicalOf(id) == i);
  }
  CHECK_FALSE(map.canonicalOf(1).has_value());
  CHECK_FALSE(map.canonicalOf(10).has_value());
}

TEST_CASE("JointMap coordinate projection round-trips", "[model]") {
  ChainModel model(kModelPath);
  JointMap map(model);

  ZVecGuard q8(kCanonicalDof), q9(kModelDof), back(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(q8.vec, i) = 0.1 * (i + 1);
  }
  map.expand(q8.vec, q9.vec);
  CHECK(zVecElemNC(q9.vec, map.rokiOffsetFingerB()) ==
        zVecElemNC(q8.vec, kCanonicalDof - 1));
  map.reduce(q9.vec, back.vec);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(zVecElemNC(back.vec, i) == zVecElemNC(q8.vec, i));
  }
}

TEST_CASE("JointMap torque reduction satisfies virtual work", "[model]") {
  ChainModel model(kModelPath);
  JointMap map(model);

  // A generalized-force field over the 9 model coordinates must produce
  // the same power in canonical coordinates: tau8^T dq8 = tau9^T dq9 for
  // any dq8 expanded to dq9.
  ZVecGuard tau9(kModelDof), tau8(kCanonicalDof);
  ZVecGuard dq8(kCanonicalDof), dq9(kModelDof);
  for (int i = 0; i < kModelDof; ++i) {
    zVecElemNC(tau9.vec, i) = 0.5 + 0.25 * i;
  }
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(dq8.vec, i) = 1.0 - 0.1 * i;
  }
  map.reduceTorque(tau9.vec, tau8.vec);
  map.expand(dq8.vec, dq9.vec);

  double power8 = 0.0, power9 = 0.0;
  for (int i = 0; i < kCanonicalDof; ++i) {
    power8 += zVecElemNC(tau8.vec, i) * zVecElemNC(dq8.vec, i);
  }
  for (int i = 0; i < kModelDof; ++i) {
    power9 += zVecElemNC(tau9.vec, i) * zVecElemNC(dq9.vec, i);
  }
  CHECK(power8 == Approx(power9).margin(1e-12));

  // and the gripper entry is the sum of both finger torques
  const double finger_a =
      zVecElemNC(tau9.vec, map.rokiOffset(kCanonicalDof - 1));
  const double finger_b = zVecElemNC(tau9.vec, map.rokiOffsetFingerB());
  CHECK(zVecElemNC(tau8.vec, kCanonicalDof - 1) ==
        Approx(finger_a + finger_b));
}

TEST_CASE("config TOML mirrors the URDF limits in canonical order",
          "[model][config]") {
  const auto tbl = toml::parse_file(kConfigPath);
  const auto* joints = tbl["joint"].as_array();
  REQUIRE(joints != nullptr);
  REQUIRE(joints->size() == kCanonicalDof);

  const auto& canonical = canonicalJoints();
  for (int i = 0; i < kCanonicalDof; ++i) {
    const auto* joint = (*joints)[i].as_table();
    REQUIRE(joint != nullptr);
    CHECK(joint->at("name").value<std::string>() == canonical[i].urdf_joint);
    CHECK(joint->at("id").value<int>() == canonical[i].dxl_id);
    CHECK(joint->at("velocity_limit").value<double>() ==
          Approx(kVelocityLimit));
    CHECK(joint->at("effort_limit").value<double>() ==
          Approx(kEffortLimits[i]));
    const auto model_name = joint->at("model").value<std::string>();
    REQUIRE(model_name.has_value());
    if (i == 1) {
      CHECK(*model_name == "XM540-W270");  // joint 2 is the strong tilt servo
    } else {
      CHECK(*model_name == "XM430-W350");
    }
  }
}
