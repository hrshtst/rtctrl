#pragma once

#include <roki/rk_chain.h>

#include <string>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::model {

// Structured IK outcome — a solve never returns a bare vector: callers
// must check `converged` before trusting the solution. A finite but
// non-converged result (e.g. an unreachable target) is reported
// explicitly, not silently.
struct IkResult {
  bool converged = false;   // residuals within tolerance AND limits held
  double pos_residual = 0;  // |target - effector| [m]
  double att_residual = 0;  // attitude error angle [rad]
  int iterations = 0;
  bool within_limits = false;
  bool finite = false;  // solution vector contains no NaN/inf
};

// Numerical inverse kinematics for a world-frame effector pose, using
// roki's Levenberg-Marquardt iteration with the error-damped equation
// solver (the roki default — robust at and near singularities). Only the
// seven arm joints participate; finger joints are not registered and
// keep their initial displacements.
//
// Registers IK state on the model's chain: create one IkSolver per
// ChainModel, and keep the ChainModel alive while the solver exists.
class IkSolver {
 public:
  IkSolver(ChainModel& model, const JointMap& map,
           const std::string& effector_link = "crane_x7_gripper_base_link");
  ~IkSolver();

  IkSolver(const IkSolver&) = delete;
  IkSolver& operator=(const IkSolver&) = delete;

  // Solves from the model-space initial configuration q_init (size 9)
  // and writes the solution to q_out (size 9). pos_tol/att_tol are the
  // acceptance thresholds for `converged`.
  IkResult solve(const zVec3D& target_pos, const zMat3D& target_att,
                 const zVec q_init, zVec q_out, double pos_tol = 1e-4,
                 double att_tol = 1e-3, int max_iter = 200);

 private:
  ChainModel& model_;
  int effector_index_;
  rkIKCell* pos_cell_;
  rkIKCell* att_cell_;
};

}  // namespace rtctrl::model
