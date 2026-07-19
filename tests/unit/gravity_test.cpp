#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

using Catch::Approx;
using rtctrl::model::ChainModel;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::kModelDof;
using rtctrl::model::ZVector;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr double kGravity = 9.80665;  // roki's RK_G

// Total potential energy at canonical configuration q8.
double potentialEnergy(ChainModel& model, const JointMap& map,
                       const zVec q8) {
  ZVector q9(kModelDof);
  map.expand(q8, q9);
  model.fk(q9);
  double energy = 0.0;
  rkChain* chain = model.chain();
  for (int i = 0; i < rkChainLinkNum(chain); ++i) {
    rkLink* link = rkChainLink(chain, i);
    const double mass = rkLinkMass(link);
    if (mass <= 0.0) continue;
    zVec3D com_world;
    zXform3D(rkLinkWldFrame(link), rkLinkCOM(link), &com_world);
    energy += mass * kGravity * com_world.c.z;
  }
  return energy;
}

}  // namespace

TEST_CASE("gravity torque matches the potential-energy gradient",
          "[gravity]") {
  ChainModel model(kModelPath);
  JointMap map(model);

  ZVector q8(kCanonicalDof), tau8(kCanonicalDof);
  q8[0] = 0.2;
  q8[1] = 0.6;
  q8[2] = -0.3;
  q8[3] = -1.1;
  q8[4] = 0.25;
  q8[5] = -0.45;
  q8[6] = 0.5;
  q8[7] = 0.3;
  model.gravityTorque(map, q8, tau8);

  // tau_g must equal dU/dq (the torque holding the pose): central
  // finite differences of the potential energy, joint by joint.
  constexpr double kEps = 1e-6;
  for (int i = 0; i < kCanonicalDof; ++i) {
    ZVector plus(q8), minus(q8);
    plus[i] += kEps;
    minus[i] -= kEps;
    const double numeric =
        (potentialEnergy(model, map, plus) -
         potentialEnergy(model, map, minus)) /
        (2.0 * kEps);
    CHECK(tau8[i] == Approx(numeric).margin(1e-5));
  }
}

TEST_CASE("gravity torque vanishes for vertical axes at zero pose",
          "[gravity]") {
  ChainModel model(kModelPath);
  JointMap map(model);
  ZVector q8(kCanonicalDof), tau8(kCanonicalDof);
  model.gravityTorque(map, q8, tau8);

  // Upright arm: pan/twist joints (0,2,4,6) rotate about vertical axes
  // and bear no gravity moment; the tilt group is near zero too since
  // the arm is straight up.
  for (const int i : {0, 2, 4, 6}) {
    CHECK(std::fabs(tau8[i]) < 1e-9);
  }
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(std::isfinite(tau8[i]));
  }
}

TEST_CASE("gravity torque call is stable across repeated invocations",
          "[gravity]") {
  // regression for the null-vector hazard: repeated calls reuse the
  // member-owned scratch vectors and stay bit-identical
  ChainModel model(kModelPath);
  JointMap map(model);
  ZVector q8(kCanonicalDof), a(kCanonicalDof), b(kCanonicalDof);
  q8[1] = 0.7;
  q8[3] = -0.9;
  model.gravityTorque(map, q8, a);
  model.gravityTorque(map, q8, b);
  for (int i = 0; i < kCanonicalDof; ++i) {
    CHECK(a[i] == b[i]);
  }
}
