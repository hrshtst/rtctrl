#pragma once

#include <zm/zm.h>

#include <array>
#include <cstdint>
#include <optional>

namespace rtctrl::model {

class ChainModel;

// Canonical controller coordinates: 8 DOF.
// [0..6] arm joints 1..7 (DXL IDs 2..8), [7] gripper (DXL ID 9 = finger_a).
// The roki model has 9 revolute joints: the canonical 8 plus finger_b,
// which mimics finger_a (multiplier 1) and has no servo; the simulator
// drives it through the coupling constraint.
inline constexpr int kCanonicalDof = 8;
inline constexpr int kModelDof = 9;

struct CanonicalJoint {
  const char* urdf_joint;  // URDF joint name — the identity used in config
  const char* link;        // ztk/roki link that owns the joint
  std::uint8_t dxl_id;
};

// The canonical order, fixed for the whole project.
const std::array<CanonicalJoint, kCanonicalDof>& canonicalJoints();
const char* fingerBLink();

// Explicit index mappings between canonical coordinates, DXL IDs, and the
// roki joint vector. Offsets are resolved by link NAME at construction —
// never by ordinal — so a reordered model file cannot silently break them.
class JointMap {
 public:
  explicit JointMap(const ChainModel& model);  // throws on missing links

  int rokiOffset(int canonical) const { return offsets_[canonical]; }
  int rokiOffsetFingerB() const { return offset_finger_b_; }
  std::uint8_t dxlId(int canonical) const;
  std::optional<int> canonicalOf(std::uint8_t dxl_id) const;

  // q9[mapped] = q8; finger_b = gripper (mimic multiplier 1).
  void expand(const zVec q8, zVec q9) const;
  // q8 = q9[mapped]; gripper state = finger_a.
  void reduce(const zVec q9, zVec q8) const;
  // Generalized-force reduction (virtual work): the gripper coordinate
  // moves both fingers, so its torque is tau9[finger_a] + tau9[finger_b].
  void reduceTorque(const zVec tau9, zVec tau8) const;

 private:
  std::array<int, kCanonicalDof> offsets_{};
  int offset_finger_b_ = -1;
};

}  // namespace rtctrl::model
