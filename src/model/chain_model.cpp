#include "rtctrl/model/chain_model.hpp"

#include <filesystem>
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
  if (q9_ == nullptr || tau9_ == nullptr || zero_vel9_ == nullptr ||
      zero_acc9_ == nullptr || vel9_ == nullptr || acc9_ == nullptr) {
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

double ChainModel::totalMass() const {
  double mass = 0.0;
  for (int i = 0; i < rkChainLinkNum(&chain_); ++i) {
    mass += rkChainLinkMass(&chain_, i);
  }
  return mass;
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

void ChainModel::fk(const zVec dis) { rkChainFK(&chain_, dis); }

zVec3D ChainModel::linkWorldPos(int link_index) const {
  return *rkChainLinkWldPos(&chain_, link_index);
}

}  // namespace rtctrl::model
