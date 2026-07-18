#include "rtctrl/model/ik_solver.hpp"

#include <cmath>
#include <stdexcept>

namespace rtctrl::model {

namespace {

void requireSize(const zVec vec, int size, const char* what) {
  if (vec == nullptr || zVecSizeNC(vec) != size) {
    throw std::invalid_argument(std::string("IkSolver: ") + what +
                                " must be a vector of size " +
                                std::to_string(size));
  }
}

}  // namespace

IkSolver::IkSolver(ChainModel& model, const JointMap& map,
                   const std::string& effector_link)
    : model_(model),
      effector_index_(model.linkIndex(effector_link)),
      pos_cell_(nullptr),
      att_cell_(nullptr) {
  if (effector_index_ < 0) {
    throw std::runtime_error("IkSolver: no such link: " + effector_link);
  }
  (void)map;  // the canonical table itself names the arm links

  rkChain* chain = model_.chain();
  const auto& joints = canonicalJoints();
  for (int i = 0; i < kCanonicalDof - 1; ++i) {  // arm only, no fingers
    const int link_id = model_.linkIndex(joints[i].link);
    if (link_id < 0 ||
        !rkChainRegisterIKJointID(chain, link_id, RK_IK_JOINT_WEIGHT_DEFAULT)) {
      throw std::runtime_error(std::string("IkSolver: cannot register joint ") +
                               joints[i].link);
    }
  }

  rkIKAttr attr{};
  attr.id = effector_index_;
  pos_cell_ = rkChainRegisterIKCellWldPos(chain, "rtctrl_pos", 0, &attr,
                                          RK_IK_ATTR_MASK_ID);
  att_cell_ = rkChainRegisterIKCellWldAtt(chain, "rtctrl_att", 0, &attr,
                                          RK_IK_ATTR_MASK_ID);
  if (pos_cell_ == nullptr || att_cell_ == nullptr) {
    throw std::runtime_error("IkSolver: cannot register constraint cells");
  }
}

IkSolver::~IkSolver() {
  rkChain* chain = model_.chain();
  if (att_cell_ != nullptr) rkChainUnregisterAndDestroyIKCell(chain, att_cell_);
  if (pos_cell_ != nullptr) rkChainUnregisterAndDestroyIKCell(chain, pos_cell_);
}

IkResult IkSolver::solve(const zVec3D& target_pos, const zMat3D& target_att,
                         const zVec q_init, zVec q_out, double pos_tol,
                         double att_tol, int max_iter) {
  const int size = model_.jointSize();
  requireSize(q_init, size, "q_init");
  requireSize(q_out, size, "q_out");

  rkChain* chain = model_.chain();
  rkChainFK(chain, q_init);
  rkChainBindIK(chain);

  zVec3D ref_pos = target_pos;
  zMat3D ref_att = target_att;
  rkIKCellSetRefVec(pos_cell_, &ref_pos);
  rkIKCellSetRefAtt(att_cell_, &ref_att);

  zVecCopyNC(q_init, q_out);
  IkResult result;
  result.iterations = rkChainIK(chain, q_out, zTOL, max_iter);

  // rkChainIK leaves the chain at the solution posture; refresh anyway so
  // the residuals are guaranteed to describe q_out.
  rkChainFK(chain, q_out);

  result.finite = true;
  for (int i = 0; i < size; ++i) {
    if (!std::isfinite(zVecElemNC(q_out, i))) result.finite = false;
  }

  result.pos_residual =
      zVec3DDist(&ref_pos, rkChainLinkWldPos(chain, effector_index_));
  zVec3D att_err;
  zMat3DError(&ref_att, rkChainLinkWldAtt(chain, effector_index_), &att_err);
  result.att_residual = zVec3DNorm(&att_err);

  result.within_limits = true;
  const auto& joints = canonicalJoints();
  for (int i = 0; i < kCanonicalDof - 1; ++i) {
    const int link_id = model_.linkIndex(joints[i].link);
    const int offset = model_.jointOffset(joints[i].link);
    const double q = zVecElemNC(q_out, offset);
    if (q < model_.jointMin(link_id) - zTOL ||
        q > model_.jointMax(link_id) + zTOL) {
      result.within_limits = false;
    }
  }

  result.converged = result.finite && result.within_limits &&
                     result.pos_residual <= pos_tol &&
                     result.att_residual <= att_tol;
  return result;
}

}  // namespace rtctrl::model
