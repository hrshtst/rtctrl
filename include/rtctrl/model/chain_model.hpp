#pragma once

#include <roki/rk_chain.h>

#include <string>

namespace rtctrl::model {

class JointMap;

// RAII owner of a roki kinematic chain loaded from a .ztk model file.
// Wraps only what the library layers need; chain() is the escape hatch
// for direct roki calls.
class ChainModel {
 public:
  // Shape import paths inside the .ztk are relative to the model file;
  // loading temporarily switches the working directory to the model's
  // directory (the mi-lib tool convention, cf. rk_pen), so construction
  // is not thread-safe against concurrent working-directory users.
  explicit ChainModel(const std::string& ztk_path);
  ~ChainModel();

  ChainModel(const ChainModel&) = delete;
  ChainModel& operator=(const ChainModel&) = delete;
  ChainModel(ChainModel&& other) noexcept;
  ChainModel& operator=(ChainModel&&) = delete;

  rkChain* chain() const { return &chain_; }

  int linkCount() const;
  int jointSize() const;

  int linkIndex(const std::string& link_name) const;  // -1 if absent
  // Joint-vector offset of the joint owned by the named link, -1 if absent.
  int jointOffset(const std::string& link_name) const;

  double totalMass() const;

  // Displacement limits of the 1-DOF joint owned by link i (radians).
  double jointMin(int link_index) const;
  double jointMax(int link_index) const;

  // Forward kinematics at joint vector dis (size jointSize()).
  void fk(const zVec dis);
  zVec3D linkWorldPos(int link_index) const;

  // Gravity-compensation torques in canonical coordinates: expands q8
  // to the 9 model coordinates (finger_b mimics), evaluates
  // rkChainID_G at zero velocity/acceleration — with properly sized
  // member-owned zero vectors, never null: rkChainSetJointRateAll
  // dereferences both unconditionally (rk_chain.c:478,:334) — and
  // reduces the 9 generalized torques back through the constraint
  // Jacobian (gripper torque = finger_a + finger_b, virtual work).
  void gravityTorque(const JointMap& map, const zVec q8, zVec tau8);

 private:
  void allocScratch();

  mutable rkChain chain_{};  // the roki C API takes non-const rkChain*
  bool owns_{false};
  // ID scratch, allocated on first gravityTorque call
  zVec q9_ = nullptr;
  zVec tau9_ = nullptr;
  zVec zero_vel9_ = nullptr;
  zVec zero_acc9_ = nullptr;
};

}  // namespace rtctrl::model
