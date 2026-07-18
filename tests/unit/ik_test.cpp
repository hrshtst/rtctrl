#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/ik_solver.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

using rtctrl::model::ChainModel;
using rtctrl::model::IkSolver;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::kModelDof;
using rtctrl::model::ZVector;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr const char* kEffector = "crane_x7_gripper_base_link";
constexpr double kPosTol = 1e-4;
constexpr double kAttTol = 1e-3;

struct Fixture {
  Fixture() : model(kModelPath), map(model), solver(model, map, kEffector) {}

  // Effector pose at canonical configuration q8 — reachable by construction.
  void poseAt(const zVec q8, zVec3D* pos, zMat3D* att) {
    ZVector q9(kModelDof);
    map.expand(q8, q9);
    model.fk(q9);
    const int idx = model.linkIndex(kEffector);
    *pos = *rkChainLinkWldPos(model.chain(), idx);
    *att = *rkChainLinkWldAtt(model.chain(), idx);
  }

  ChainModel model;
  JointMap map;
  IkSolver solver;
};

}  // namespace

TEST_CASE_METHOD(Fixture, "IK converges to a reachable pose", "[ik]") {
  ZVector q8(kCanonicalDof);
  q8[0] = 0.3;
  q8[1] = 0.5;
  q8[2] = -0.2;
  q8[3] = -1.2;
  q8[4] = 0.4;
  q8[5] = -0.6;
  q8[6] = 0.8;
  zVec3D target_pos;
  zMat3D target_att;
  poseAt(q8, &target_pos, &target_att);

  ZVector init(kModelDof), solution(kModelDof);
  const auto result =
      solver.solve(target_pos, target_att, init, solution, kPosTol, kAttTol);

  CHECK(result.converged);
  CHECK(result.finite);
  CHECK(result.within_limits);
  CHECK(result.pos_residual <= kPosTol);
  CHECK(result.att_residual <= kAttTol);
}

TEST_CASE_METHOD(Fixture,
                 "IK converges at a reachable singular configuration",
                 "[ik]") {
  // The fully upright zero pose aligns several joint axes — a classic
  // singularity, and exactly where the error-damped solver must still
  // converge when the target itself is reachable.
  ZVector q8_zero(kCanonicalDof);
  zVec3D target_pos;
  zMat3D target_att;
  poseAt(q8_zero, &target_pos, &target_att);

  ZVector init(kModelDof), solution(kModelDof);
  ZVector init8(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof - 1; ++i) init8[i] = 0.05;  // perturbed
  map.expand(init8, init);

  const auto result =
      solver.solve(target_pos, target_att, init, solution, kPosTol, kAttTol);

  CHECK(result.converged);
  CHECK(result.pos_residual <= kPosTol);
  CHECK(result.att_residual <= kAttTol);
}

TEST_CASE_METHOD(Fixture,
                 "IK reports explicit non-convergence for an unreachable "
                 "target",
                 "[ik]") {
  // Far outside the ~0.6 m workspace.
  zVec3D target_pos;
  zVec3DCreate(&target_pos, 1.5, 0.0, 0.2);
  zMat3D target_att;
  zMat3DIdent(&target_att);

  ZVector init(kModelDof), solution(kModelDof);
  const auto result =
      solver.solve(target_pos, target_att, init, solution, kPosTol, kAttTol);

  CHECK_FALSE(result.converged);   // flagged failure, not a silent answer
  CHECK(result.finite);            // ...but still finite, no NaN
  CHECK(result.pos_residual > 0.5);  // honestly far from the target
  for (int i = 0; i < kModelDof; ++i) {
    CHECK(std::isfinite(solution[i]));
  }
}
