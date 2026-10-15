#include "rtctrl/model/chain_model.hpp"

#include <roki/rk_jacobi.h>

#include <cmath>
#include <filesystem>
#include <random>
#include <stdexcept>
#include <utility>

#include "rtctrl/model/joint_map.hpp"

namespace rtctrl::model {

namespace fs = std::filesystem;

ChainModel::ChainModel(const std::string& ztk_path) {
  const fs::path path(ztk_path);
  rkChainInit(&chain_);
  owns_ = true;

  // mi-lib convention (cf. rk_pen): shape imports are relative to the
  // model file, so read it from its own directory.
  bool loaded = false;
  if (path.has_parent_path()) {
    const fs::path previous = fs::current_path();
    fs::current_path(path.parent_path());
    loaded = rkChainReadZTK(&chain_, path.filename().c_str()) != nullptr;
    fs::current_path(previous);
  } else {
    loaded = rkChainReadZTK(&chain_, ztk_path.c_str()) != nullptr;
  }

  if (!loaded) {
    rkChainDestroy(&chain_);
    owns_ = false;
    throw std::runtime_error("ChainModel: failed to load '" + ztk_path + "'");
  }
}

ChainModel::~ChainModel() {
  zVecFree(q9_);
  zVecFree(tau9_);
  zVecFree(zero_vel9_);
  zVecFree(zero_acc9_);
  zVecFree(vel9_);
  zVecFree(acc9_);
  zVecFree(force9_);
  zMatFree(lin_jacobi_);
  if (owns_) rkChainDestroy(&chain_);
}

void ChainModel::allocScratch() {
  if (q9_ != nullptr) return;
  const int n = jointSize();
  q9_ = zVecAlloc(n);
  tau9_ = zVecAlloc(n);
  zero_vel9_ = zVecAlloc(n);
  zero_acc9_ = zVecAlloc(n);
  vel9_ = zVecAlloc(n);
  acc9_ = zVecAlloc(n);
  force9_ = zVecAlloc(n);
  lin_jacobi_ = zMatAlloc(3, n);
  if (q9_ == nullptr || tau9_ == nullptr || zero_vel9_ == nullptr ||
      zero_acc9_ == nullptr || vel9_ == nullptr || acc9_ == nullptr ||
      force9_ == nullptr || lin_jacobi_ == nullptr) {
    throw std::bad_alloc();
  }
  zVecZero(zero_vel9_);
  zVecZero(zero_acc9_);
}

void ChainModel::gravityTorque(const JointMap& map, const zVec q8,
                               zVec tau8) {
  allocScratch();
  map.expand(q8, q9_);
  rkChainID_G(&chain_, q9_, zero_vel9_, zero_acc9_, RK_GRAVITY6D, tau9_);
  map.reduceTorque(tau9_, tau8);
}

void ChainModel::inverseDynamics(const JointMap& map, const zVec q8,
                                 const zVec dq8, const zVec ddq8,
                                 zVec tau8) {
  allocScratch();
  map.expand(q8, q9_);
  map.expand(dq8, vel9_);
  map.expand(ddq8, acc9_);
  rkChainID_G(&chain_, q9_, vel9_, acc9_, RK_GRAVITY6D, tau9_);
  map.reduceTorque(tau9_, tau8);
}

void ChainModel::checkLinkIndex(int link_index, const char* caller) const {
  if (link_index < 0 || link_index >= rkChainLinkNum(&chain_)) {
    throw std::invalid_argument(std::string("ChainModel::") + caller +
                                ": link index " + std::to_string(link_index) +
                                " out of range");
  }
}

std::array<double, 3> ChainModel::linkPointWorldPos(
    int link_index, const std::array<double, 3>& p_link) const {
  checkLinkIndex(link_index, "linkPointWorldPos");
  zVec3D p;
  zVec3DCreate(&p, p_link[0], p_link[1], p_link[2]);
  zVec3D world;
  rkChainLinkPointWldPos(&chain_, link_index, &p, &world);
  return {world.c.x, world.c.y, world.c.z};
}

void ChainModel::forceJointTorqueModel(int link_index,
                                       const std::array<double, 3>& p_link,
                                       const std::array<double, 3>& f_world,
                                       zVec tau_model) {
  checkLinkIndex(link_index, "forceJointTorqueModel");
  if (tau_model == nullptr || zVecSizeNC(tau_model) != jointSize()) {
    throw std::invalid_argument(
        "ChainModel::forceJointTorqueModel: vector size mismatch");
  }
  allocScratch();
  // roki takes the point non-const; it is only read (rk_jacobi.c).
  zVec3D p;
  zVec3DCreate(&p, p_link[0], p_link[1], p_link[2]);
  rkChainLinkWldLinJacobi(&chain_, link_index, &p, lin_jacobi_);
  const double f[3] = {f_world[0], f_world[1], f_world[2]};
  for (int j = 0; j < jointSize(); ++j) {
    double value = 0.0;
    for (int r = 0; r < 3; ++r) value += zMatElemNC(lin_jacobi_, r, j) * f[r];
    zVecElemNC(tau_model, j) = value;
  }
}

void ChainModel::forceJointTorque(const JointMap& map, const zVec q8,
                                  int link_index,
                                  const std::array<double, 3>& p_link,
                                  const std::array<double, 3>& f_world,
                                  zVec tau8) {
  checkLinkIndex(link_index, "forceJointTorque");
  allocScratch();
  map.expand(q8, q9_);
  fk(q9_);
  forceJointTorqueModel(link_index, p_link, f_world, force9_);
  map.reduceTorque(force9_, tau8);
}

ChainModel::ChainModel(ChainModel&& other) noexcept
    : chain_(other.chain_), owns_(std::exchange(other.owns_, false)) {}

int ChainModel::linkCount() const { return rkChainLinkNum(&chain_); }

int ChainModel::jointSize() const { return rkChainJointSize(&chain_); }

int ChainModel::linkIndex(const std::string& link_name) const {
  return rkChainFindLinkID(&chain_, link_name.c_str());
}

int ChainModel::jointOffset(const std::string& link_name) const {
  return rkChainFindLinkJointIDOffset(&chain_, link_name.c_str());
}

void ChainModel::jointDisplacement(zVec dis) const {
  if (dis == nullptr || zVecSizeNC(dis) != jointSize()) {
    throw std::invalid_argument(
        "ChainModel::jointDisplacement: vector size mismatch");
  }
  rkChainGetJointDisAll(&chain_, dis);
}

double ChainModel::totalMass() const {
  double mass = 0.0;
  for (int i = 0; i < rkChainLinkNum(&chain_); ++i) {
    mass += rkChainLinkMass(&chain_, i);
  }
  return mass;
}

void ChainModel::scaleMassProperties(double factor) {
  // NaN fails every comparison, so test finiteness explicitly.
  if (!std::isfinite(factor) || factor <= 0.0) {
    throw std::invalid_argument(
        "ChainModel::scaleMassProperties: factor must be finite and > 0");
  }
  for (int i = 0; i < rkChainLinkNum(&chain_); ++i) {
    rkLink* link = rkChainLink(&chain_, i);
    rkLinkSetMass(link, factor * rkLinkMass(link));
    zMat3DMulDRC(rkLinkInertia(link), factor);
  }
}

void ChainModel::perturbMassProperties(double mass_error, double com_error,
                                       std::uint64_t seed) {
  if (!std::isfinite(mass_error) || mass_error < 0.0 || mass_error >= 1.0 ||
      !std::isfinite(com_error) || com_error < 0.0) {
    throw std::invalid_argument(
        "ChainModel::perturbMassProperties: require 0 <= mass_error < 1 "
        "and finite com_error >= 0");
  }
  std::mt19937_64 rng(seed);
  // 53-bit engine output mapped to [0,1) directly:
  // uniform_real_distribution's sequence is implementation-defined,
  // which would break cross-platform reproducibility of a seed.
  auto uniform = [&rng](double lo, double hi) {
    return lo + (hi - lo) * ((rng() >> 11) * 0x1.0p-53);
  };
  for (int i = 0; i < rkChainLinkNum(&chain_); ++i) {
    rkLink* link = rkChainLink(&chain_, i);
    if (zIsTiny(rkLinkMass(link))) continue;
    const double ratio = 1.0 + uniform(-mass_error, mass_error);
    zVec3D com_diff;
    zVec3DCreate(&com_diff, uniform(-com_error, com_error),
                 uniform(-com_error, com_error),
                 uniform(-com_error, com_error));
    rkMP mp;
    rkMPSetMass(&mp, ratio * rkLinkMass(link));
    zVec3DAdd(rkLinkCOM(link), &com_diff, rkMPCOM(&mp));
    zMat3D scaled;
    zMat3DMul(rkLinkInertia(link), ratio, &scaled);
    // Parallel-axis transfer to the displaced COM (rkMP stores the
    // tensor about the COM; cf. rkMPShiftInertia).
    zMat3DCatVec3DDoubleOuterProd(&scaled, -rkMPMass(&mp), &com_diff,
                                  rkMPInertia(&mp));
    rkLinkSetMP(link, &mp);
  }
}

double ChainModel::jointMin(int link_index) const {
  rkJoint* joint = rkChainLinkJoint(&chain_, link_index);
  if (rkJointDOF(joint) != 1) {
    throw std::invalid_argument("ChainModel::jointMin: link " +
                                std::to_string(link_index) +
                                " does not own a 1-DOF joint");
  }
  double value = 0.0;
  rkJointGetMin(joint, &value);
  return value;
}

double ChainModel::jointMax(int link_index) const {
  rkJoint* joint = rkChainLinkJoint(&chain_, link_index);
  if (rkJointDOF(joint) != 1) {
    throw std::invalid_argument("ChainModel::jointMax: link " +
                                std::to_string(link_index) +
                                " does not own a 1-DOF joint");
  }
  double value = 0.0;
  rkJointGetMax(joint, &value);
  return value;
}

void ChainModel::fk(const zVec dis) {
  // rkChainFK() also runs loop-closure IK whenever the chain owns IK state,
  // mutating `dis` despite its const-qualified API type. This wrapper's
  // contract is forward kinematics at exactly the supplied posture.
  rkChainSetJointDisAll(&chain_, dis);
  rkChainUpdateFK(&chain_);
}

zVec3D ChainModel::linkWorldPos(int link_index) const {
  return *rkChainLinkWldPos(&chain_, link_index);
}

bool ChainModel::writeInitZtk(const std::string& path, const zVec dis) {
  rkChainSetJointDisAll(&chain_, dis);
  rkChainUpdateFK(&chain_);
  return rkChainInitWriteZTK(&chain_, path.c_str());
}

}  // namespace rtctrl::model
