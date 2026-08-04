#pragma once

#include <roki/rk_chain.h>

#include <cstdint>
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

  // Uniformly scales every link's mass and rotational inertia by
  // factor (> 0) — a density-style perturbation for model-error
  // studies (e.g. a deliberately wrong controller model against a
  // true-model plant). The Newton-Euler torques are linear in the
  // inertial parameters, so gravityTorque/inverseDynamics outputs
  // scale by exactly factor at any state; COMs and geometry are
  // untouched.
  void scaleMassProperties(double factor);

  // Randomized per-link perturbation (cf. mi-lib-tutorial roki008's
  // add_mp_error, made symmetric and seeded): every link with
  // non-negligible mass draws an independent mass factor
  // 1 + U(-mass_error, +mass_error) — its inertia density-scaled by
  // the same factor — and a COM offset uniform in the ±com_error cube
  // [m], with the tensor transferred to the displaced COM by the
  // parallel-axis term (rkMP stores inertia ABOUT the COM, so a COM
  // move without the transfer would corrupt it). The draw order is
  // fixed and the engine output (std::mt19937_64, standard-specified)
  // maps to doubles without uniform_real_distribution (whose sequence
  // is implementation-defined), so one seed reproduces bit-identically
  // everywhere. Requires 0 <= mass_error < 1 and com_error >= 0.
  void perturbMassProperties(double mass_error, double com_error,
                             std::uint64_t seed);

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

  // Full inverse dynamics in canonical coordinates: the torques
  // realizing (q8, dq8, ddq8) under gravity. Same expansion/reduction
  // as gravityTorque.
  void inverseDynamics(const JointMap& map, const zVec q8, const zVec dq8,
                       const zVec ddq8, zVec tau8);

 private:
  void allocScratch();

  mutable rkChain chain_{};  // the roki C API takes non-const rkChain*
  bool owns_{false};
  // ID scratch, allocated on first use
  zVec q9_ = nullptr;
  zVec tau9_ = nullptr;
  zVec zero_vel9_ = nullptr;  // stays zero — gravityTorque only
  zVec zero_acc9_ = nullptr;  // stays zero — gravityTorque only
  zVec vel9_ = nullptr;
  zVec acc9_ = nullptr;
};

}  // namespace rtctrl::model
