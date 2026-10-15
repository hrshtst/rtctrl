#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <toml++/toml.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

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

TEST_CASE("mass-property scaling scales gravity torque linearly",
          "[model]") {
  // Newton-Euler torques are linear in the inertial parameters, so a
  // uniform mass/inertia scale must multiply the gravity torque field
  // by exactly the same factor at any configuration.
  constexpr double kScale = 1.3;
  ChainModel truth(kModelPath);
  ChainModel scaled(kModelPath);
  scaled.scaleMassProperties(kScale);
  CHECK(scaled.totalMass() == Approx(kScale * truth.totalMass()));

  JointMap map_truth(truth);
  JointMap map_scaled(scaled);
  const double pose[kCanonicalDof] = {0.0, 0.4, 0.2, -0.9,
                                      0.3, -0.5, 0.4, 0.05};
  ZVecGuard q(kCanonicalDof), tau(kCanonicalDof), tau_scaled(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) zVecSetElemNC(q.vec, i, pose[i]);
  truth.gravityTorque(map_truth, q.vec, tau.vec);
  scaled.gravityTorque(map_scaled, q.vec, tau_scaled.vec);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(zVecElemNC(tau_scaled.vec, i) ==
          Approx(kScale * zVecElemNC(tau.vec, i)).margin(1e-12));
  }

  CHECK_THROWS(scaled.scaleMassProperties(0.0));
  CHECK_THROWS(scaled.scaleMassProperties(-1.0));
  CHECK_THROWS(scaled.scaleMassProperties(std::nan("")));
  CHECK_THROWS(scaled.scaleMassProperties(
      std::numeric_limits<double>::infinity()));
}

TEST_CASE("mass-property perturbation is seeded, bounded, and COM-aware",
          "[model]") {
  const double pose[kCanonicalDof] = {0.0, 0.4, 0.2, -0.9,
                                      0.3, -0.5, 0.4, 0.05};
  ZVecGuard q(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) zVecSetElemNC(q.vec, i, pose[i]);
  ChainModel truth(kModelPath);
  JointMap map_truth(truth);
  ZVecGuard tau_truth(kCanonicalDof);
  truth.gravityTorque(map_truth, q.vec, tau_truth.vec);

  // Same seed -> bit-identical model; different seed -> different.
  ChainModel a(kModelPath), b(kModelPath), c(kModelPath);
  a.perturbMassProperties(0.2, 0.01, 42);
  b.perturbMassProperties(0.2, 0.01, 42);
  c.perturbMassProperties(0.2, 0.01, 43);
  CHECK(a.totalMass() == b.totalMass());
  CHECK(a.totalMass() != c.totalMass());
  JointMap map_a(a), map_b(b);
  ZVecGuard tau_a(kCanonicalDof), tau_b(kCanonicalDof);
  a.gravityTorque(map_a, q.vec, tau_a.vec);
  b.gravityTorque(map_b, q.vec, tau_b.vec);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(zVecElemNC(tau_a.vec, i) == zVecElemNC(tau_b.vec, i));
  }

  // Symmetric per-link error bounds the total mass.
  CHECK(a.totalMass() > 0.8 * truth.totalMass());
  CHECK(a.totalMass() < 1.2 * truth.totalMass());

  // Zero-error perturbation is the identity (exercises the
  // parallel-axis transfer at zero displacement).
  ChainModel ident(kModelPath);
  ident.perturbMassProperties(0.0, 0.0, 7);
  CHECK(ident.totalMass() == Approx(truth.totalMass()).margin(1e-12));
  JointMap map_ident(ident);
  ZVecGuard tau_ident(kCanonicalDof);
  ident.gravityTorque(map_ident, q.vec, tau_ident.vec);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(zVecElemNC(tau_ident.vec, i) ==
          Approx(zVecElemNC(tau_truth.vec, i)).margin(1e-12));
  }

  // COM-only error keeps the total mass but reshapes the gravity
  // field — the error class a bias integrator cannot absorb.
  ChainModel com_only(kModelPath);
  com_only.perturbMassProperties(0.0, 0.02, 5);
  CHECK(com_only.totalMass() == Approx(truth.totalMass()).margin(1e-12));
  JointMap map_com(com_only);
  ZVecGuard tau_com(kCanonicalDof);
  com_only.gravityTorque(map_com, q.vec, tau_com.vec);
  double max_diff = 0.0;
  for (int i = 0; i < kCanonicalDof; ++i) {
    max_diff = std::max(max_diff, std::fabs(zVecElemNC(tau_com.vec, i) -
                                            zVecElemNC(tau_truth.vec, i)));
  }
  CHECK(max_diff > 1e-4);

  CHECK_THROWS(a.perturbMassProperties(1.0, 0.0, 0));
  CHECK_THROWS(a.perturbMassProperties(-0.1, 0.0, 0));
  CHECK_THROWS(a.perturbMassProperties(0.0, -0.01, 0));
  CHECK_THROWS(a.perturbMassProperties(std::nan(""), 0.0, 0));
}

TEST_CASE("the TCP virtual link rides the gripper base rigidly",
          "[model][tcp]") {
  ChainModel model(kModelPath);
  const int tcp = model.linkIndex("crane_x7_tcp_link");
  const int gb = model.linkIndex("crane_x7_gripper_base_link");
  REQUIRE(tcp >= 0);
  REQUIRE(gb >= 0);
  // the fixed link adds a frame, never a degree of freedom
  CHECK(model.jointSize() == kModelDof);

  // rigid attachment at an arbitrary posture:
  //   p_tcp = p_gb + R_gb (0, 0, 0.084),   R_tcp = R_gb Ry(-90 deg)
  ZVecGuard dis(model.jointSize());
  const double qs[kModelDof] = {0.3, -0.7, 0.4, -1.1, 0.5,
                                -0.4, 0.8, 0.2, 0.2};
  for (int i = 0; i < kModelDof; ++i) zVecSetElemNC(dis.vec, i, qs[i]);
  model.fk(dis.vec);
  rkChain* ch = model.chain();

  zVec3D offset, expect_p;
  zVec3DCreate(&offset, 0.0, 0.0, 0.084);
  zMulMat3DVec3D(rkChainLinkWldAtt(ch, gb), &offset, &expect_p);
  zVec3DAddDRC(&expect_p, rkChainLinkWldPos(ch, gb));
  CHECK(zVec3DDist(&expect_p, rkChainLinkWldPos(ch, tcp)) ==
        Approx(0.0).margin(1e-9));

  zMat3D ry_m90, expect_r;
  zMat3DCreate(&ry_m90, 0, 0, -1, 0, 1, 0, 1, 0, 0);
  zMulMat3DMat3D(rkChainLinkWldAtt(ch, gb), &ry_m90, &expect_r);
  zVec3D att_err;
  zMat3DError(&expect_r, rkChainLinkWldAtt(ch, tcp), &att_err);
  CHECK(zVec3DNorm(&att_err) == Approx(0.0).margin(1e-9));
}

TEST_CASE("the TCP frame matches the fingertip midpoint and the "
          "forward-alignment convention",
          "[model][tcp]") {
  ChainModel model(kModelPath);
  const int tcp = model.linkIndex("crane_x7_tcp_link");
  REQUIRE(tcp >= 0);

  // zero (upright) posture: the gripper-base frame equals the world
  // frame, so the TCP sits at the fingertip midpoint 0.084 m (finger
  // joint 0.024 + finger length 0.060) above the gripper base
  ZVecGuard dis(model.jointSize());
  model.fk(dis.vec);
  const zVec3D pos0 = model.linkWorldPos(tcp);
  CHECK(pos0.c.x == Approx(0.0).margin(1e-9));
  CHECK(pos0.c.y == Approx(0.0).margin(1e-9));
  CHECK(pos0.c.z ==
        Approx(kGripperBaseZeroPoseZ + 0.084).margin(1e-9));

  // gripper pointing forward (+x world, via shoulder tilt): the TCP
  // axes coincide with the world axes — the convention this link
  // exists to provide
  const int tilt =
      model.jointOffset("crane_x7_upper_arm_fixed_part_link");
  REQUIRE(tilt >= 0);
  zVecSetElemNC(dis.vec, tilt, -M_PI_2);
  model.fk(dis.vec);
  zMat3D ident;
  zMat3DIdent(&ident);
  zVec3D att_err;
  zMat3DError(&ident, rkChainLinkWldAtt(model.chain(), tcp), &att_err);
  CHECK(zVec3DNorm(&att_err) == Approx(0.0).margin(1e-9));
  // and "forward" genuinely means the reach direction, not its mirror
  const zVec3D pos_fwd = model.linkWorldPos(tcp);
  CHECK(pos_fwd.c.x > 0.1);
}

TEST_CASE("the committed ztk ends with the hand-tuned override patch",
          "[model][tcp]") {
  // port_model.py appends tools/crane_x7_motor.patch.ztk verbatim; the
  // TCP link lives in the patch, so an edit to only one of the two
  // files would silently drift the next regeneration.
  std::ifstream zf(kModelPath);
  std::ifstream pf("tools/crane_x7_motor.patch.ztk");
  REQUIRE(zf.good());
  REQUIRE(pf.good());
  std::stringstream zs, ps;
  zs << zf.rdbuf();
  ps << pf.rdbuf();
  const std::string ztk = zs.str();
  const std::string patch = ps.str();
  REQUIRE(ztk.size() > patch.size());
  CHECK(ztk.compare(ztk.size() - patch.size(), patch.size(), patch) ==
        0);
}

TEST_CASE("the rk_pen initial-state file round-trips through roki's reader",
          "[model][init]") {
  // rk_pen -init parses joint displacements in DEGREES (roki's
  // revolute reader applies zDeg2Rad), so the radian file the first
  // writer emitted loaded as a near-zero posture. The writer is now
  // roki's own; this pins the round trip through the very reader
  // rk_pen calls, plus the unit on disk.
  const char* path = "build/test_pose.init.ztk";
  ChainModel model(kModelPath);
  JointMap map(model);
  const double q8[kCanonicalDof] = {0.3, -0.7, 0.4, -1.1, 0.5, -0.4, 0.8, 0.2};
  ZVecGuard q8v(kCanonicalDof), q9(kModelDof);
  for (int i = 0; i < kCanonicalDof; ++i) zVecSetElemNC(q8v.vec, i, q8[i]);
  map.expand(q8v.vec, q9.vec);
  REQUIRE(model.writeInitZtk(path, q9.vec));

  std::ifstream in(path);
  REQUIRE(in.good());
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string text = ss.str();
  CHECK(text.find("[roki::chain::init]") != std::string::npos);
  // -0.7 rad on the shoulder tilt is -40.107 degrees on disk
  CHECK(text.find("joint: crane_x7_upper_arm_fixed_part_link -40.107") !=
        std::string::npos);
  CHECK(text.find("joint: crane_x7_upper_arm_fixed_part_link -0.7") ==
        std::string::npos);

  ChainModel loaded(kModelPath);
  REQUIRE(rkChainInitReadZTK(loaded.chain(), path) != nullptr);
  ZVecGuard back(kModelDof);
  rkChainGetJointDisAll(loaded.chain(), back.vec);
  for (int i = 0; i < kModelDof; ++i) {
    CHECK(zVecElemNC(back.vec, i) ==
          Approx(zVecElemNC(q9.vec, i)).margin(1e-6));
  }
  // and the frame rk_pen draws is the one the solve produced
  const int tcp = model.linkIndex("crane_x7_tcp_link");
  REQUIRE(tcp >= 0);
  const zVec3D want = model.linkWorldPos(tcp);
  const zVec3D got = loaded.linkWorldPos(tcp);
  CHECK(got.c.x == Approx(want.c.x).margin(1e-6));
  CHECK(got.c.y == Approx(want.c.y).margin(1e-6));
  CHECK(got.c.z == Approx(want.c.z).margin(1e-6));
  std::remove(path);
}

namespace {

// A generic, non-singular canonical pose (finger opened slightly).
constexpr double kForcePose[kCanonicalDof] = {0.3,  -0.6, 0.2, -1.4,
                                              0.25, -0.5, 0.4, 0.3};

double project(const std::array<double, 3>& f, const std::array<double, 3>& a,
               const std::array<double, 3>& b, double scale) {
  return (f[0] * (a[0] - b[0]) + f[1] * (a[1] - b[1]) + f[2] * (a[2] - b[2])) *
         scale;
}

}  // namespace

TEST_CASE("link-point force torque satisfies virtual work in canonical "
          "coordinates",
          "[model][link_force]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  const std::array<double, 3> p_link = {0.012, -0.021, 0.034};
  const std::array<double, 3> f_world = {1.3, -0.7, 2.1};
  constexpr double kEps = 1e-6;

  for (const char* name :
       {"crane_x7_lower_arm_revolute_part_link", "crane_x7_tcp_link",
        "crane_x7_gripper_finger_a_link", "crane_x7_gripper_finger_b_link"}) {
    INFO(name);
    const int link = model.linkIndex(name);
    REQUIRE(link >= 0);
    ZVecGuard q8(kCanonicalDof), q9(kModelDof), tau8(kCanonicalDof);
    for (int i = 0; i < kCanonicalDof; ++i) zVecElemNC(q8.vec, i) = kForcePose[i];
    model.forceJointTorque(map, q8.vec, link, p_link, f_world, tau8.vec);

    // tau8[i] = f . dp/dq8[i]: the gripper coordinate moves both fingers
    // through the mimic expansion, exactly as the reduction assumes.
    for (int i = 0; i < kCanonicalDof; ++i) {
      INFO("canonical joint " << i);
      const double q0 = zVecElemNC(q8.vec, i);
      zVecElemNC(q8.vec, i) = q0 + kEps;
      map.expand(q8.vec, q9.vec);
      model.fk(q9.vec);
      const auto plus = model.linkPointWorldPos(link, p_link);
      zVecElemNC(q8.vec, i) = q0 - kEps;
      map.expand(q8.vec, q9.vec);
      model.fk(q9.vec);
      const auto minus = model.linkPointWorldPos(link, p_link);
      zVecElemNC(q8.vec, i) = q0;
      CHECK(zVecElemNC(tau8.vec, i) ==
            Approx(project(f_world, plus, minus, 0.5 / kEps)).margin(1e-7));
    }
  }
}

TEST_CASE("model-coordinate force torque keeps finger_b independent",
          "[model][link_force]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  const int link = model.linkIndex("crane_x7_gripper_finger_b_link");
  REQUIRE(link >= 0);
  const std::array<double, 3> p_link = {0.005, 0.01, 0.02};
  const std::array<double, 3> f_world = {-0.4, 1.1, 0.6};
  constexpr double kEps = 1e-6;

  ZVecGuard q8(kCanonicalDof), q9(kModelDof), tau9(kModelDof);
  ZVecGuard before(kModelDof), after(kModelDof);
  for (int i = 0; i < kCanonicalDof; ++i) zVecElemNC(q8.vec, i) = kForcePose[i];
  map.expand(q8.vec, q9.vec);
  // finger_b deliberately away from the mimic value, as in the simulator
  zVecElemNC(q9.vec, map.rokiOffsetFingerB()) = 0.1;
  model.fk(q9.vec);
  model.jointDisplacement(before.vec);
  model.forceJointTorqueModel(link, p_link, f_world, tau9.vec);

  // no state change (compared with roki's stored displacement, which
  // need not round-trip the input bit for bit)
  model.jointDisplacement(after.vec);
  for (int j = 0; j < kModelDof; ++j) {
    CHECK(zVecElemNC(after.vec, j) == zVecElemNC(before.vec, j));
  }
  // tau9[j] = f . dp/dq9[j] per independent model coordinate
  for (int j = 0; j < kModelDof; ++j) {
    INFO("model joint " << j);
    const double q0 = zVecElemNC(q9.vec, j);
    zVecElemNC(q9.vec, j) = q0 + kEps;
    model.fk(q9.vec);
    const auto plus = model.linkPointWorldPos(link, p_link);
    zVecElemNC(q9.vec, j) = q0 - kEps;
    model.fk(q9.vec);
    const auto minus = model.linkPointWorldPos(link, p_link);
    zVecElemNC(q9.vec, j) = q0;
    CHECK(zVecElemNC(tau9.vec, j) ==
          Approx(project(f_world, plus, minus, 0.5 / kEps)).margin(1e-7));
  }
  // the finger_a column is zero for a point on finger_b
  CHECK(zVecElemNC(tau9.vec, map.rokiOffset(kCanonicalDof - 1)) == 0.0);
}

TEST_CASE("roki's link external wrench changes inverse dynamics by minus "
          "the force joint torque",
          "[model][link_force]") {
  // Independent oracle: rk_link.c subtracts the link external wrench
  // (link frame, moment about the link origin) in the Newton-Euler
  // recursion. Pins both the frame convention and the sign.
  ChainModel model(kModelPath);
  JointMap map(model);
  const int link = model.linkIndex("crane_x7_lower_arm_revolute_part_link");
  REQUIRE(link >= 0);
  const std::array<double, 3> p_link = {0.02, -0.01, 0.05};
  const std::array<double, 3> f_world = {0.9, 1.7, -2.4};

  ZVecGuard q8(kCanonicalDof), dq8(kCanonicalDof), ddq8(kCanonicalDof);
  ZVecGuard q9(kModelDof), plain(kCanonicalDof), loaded(kCanonicalDof);
  ZVecGuard force(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(q8.vec, i) = kForcePose[i];
    zVecElemNC(dq8.vec, i) = 0.3 - 0.07 * i;
    zVecElemNC(ddq8.vec, i) = -0.5 + 0.11 * i;
  }
  model.inverseDynamics(map, q8.vec, dq8.vec, ddq8.vec, plain.vec);
  model.forceJointTorque(map, q8.vec, link, p_link, f_world, force.vec);

  map.expand(q8.vec, q9.vec);
  model.fk(q9.vec);
  zVec3D fw, fl, p;
  zVec3DCreate(&fw, f_world[0], f_world[1], f_world[2]);
  zMulMat3DTVec3D(rkChainLinkWldAtt(model.chain(), link), &fw, &fl);
  zVec3DCreate(&p, p_link[0], p_link[1], p_link[2]);
  rkLinkSetExtForce(rkChainLink(model.chain(), link), &fl, &p);
  // The wrench persists on this throwaway model; nothing else reads it.
  model.inverseDynamics(map, q8.vec, dq8.vec, ddq8.vec, loaded.vec);

  for (int i = 0; i < kCanonicalDof; ++i) {
    INFO("canonical joint " << i);
    CHECK(zVecElemNC(loaded.vec, i) - zVecElemNC(plain.vec, i) ==
          Approx(-zVecElemNC(force.vec, i)).margin(1e-9));
  }
}

TEST_CASE("link-point helpers validate their arguments",
          "[model][link_force]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  const std::array<double, 3> zero = {0.0, 0.0, 0.0};
  ZVecGuard q8(kCanonicalDof), tau8(kCanonicalDof), tau9(kModelDof);
  ZVecGuard wrong(kCanonicalDof);
  CHECK_THROWS_AS(model.linkPointWorldPos(-1, zero), std::invalid_argument);
  CHECK_THROWS_AS(model.linkPointWorldPos(model.linkCount(), zero),
                  std::invalid_argument);
  CHECK_THROWS_AS(model.forceJointTorqueModel(-1, zero, zero, tau9.vec),
                  std::invalid_argument);
  CHECK_THROWS_AS(
      model.forceJointTorque(map, q8.vec, model.linkCount(), zero, zero,
                             tau8.vec),
      std::invalid_argument);
  const int link = model.linkIndex("crane_x7_tcp_link");
  CHECK_THROWS_AS(model.forceJointTorqueModel(link, zero, zero, wrong.vec),
                  std::invalid_argument);

  // the point helper is the link frame transform: the link origin maps to
  // the link's world position
  ZVecGuard q9(kModelDof);
  model.fk(q9.vec);
  const auto origin = model.linkPointWorldPos(link, zero);
  const zVec3D expected = model.linkWorldPos(link);
  CHECK(origin[0] == expected.c.x);
  CHECK(origin[1] == expected.c.y);
  CHECK(origin[2] == expected.c.z);
}
