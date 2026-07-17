#include "rtctrl/model/joint_map.hpp"

#include <stdexcept>
#include <string>

#include "rtctrl/model/chain_model.hpp"

namespace rtctrl::model {

const std::array<CanonicalJoint, kCanonicalDof>& canonicalJoints() {
  static const std::array<CanonicalJoint, kCanonicalDof> table = {{
      {"crane_x7_shoulder_fixed_part_pan_joint",
       "crane_x7_shoulder_revolute_part_link", 2},
      {"crane_x7_shoulder_revolute_part_tilt_joint",
       "crane_x7_upper_arm_fixed_part_link", 3},
      {"crane_x7_upper_arm_revolute_part_twist_joint",
       "crane_x7_upper_arm_revolute_part_link", 4},
      {"crane_x7_upper_arm_revolute_part_rotate_joint",
       "crane_x7_lower_arm_fixed_part_link", 5},
      {"crane_x7_lower_arm_fixed_part_joint",
       "crane_x7_lower_arm_revolute_part_link", 6},
      {"crane_x7_lower_arm_revolute_part_joint", "crane_x7_wrist_link", 7},
      {"crane_x7_wrist_joint", "crane_x7_gripper_base_link", 8},
      {"crane_x7_gripper_finger_a_joint", "crane_x7_gripper_finger_a_link", 9},
  }};
  return table;
}

const char* fingerBLink() { return "crane_x7_gripper_finger_b_link"; }

namespace {

void requireSize(const zVec vec, int size, const char* what) {
  if (vec == nullptr || zVecSizeNC(vec) != size) {
    throw std::invalid_argument(std::string("JointMap: ") + what +
                                " must be a vector of size " +
                                std::to_string(size));
  }
}

}  // namespace

JointMap::JointMap(const ChainModel& model) {
  std::string missing;
  const auto& joints = canonicalJoints();
  for (int i = 0; i < kCanonicalDof; ++i) {
    offsets_[i] = model.jointOffset(joints[i].link);
    if (offsets_[i] < 0) missing += std::string(" ") + joints[i].link;
  }
  offset_finger_b_ = model.jointOffset(fingerBLink());
  if (offset_finger_b_ < 0) missing += std::string(" ") + fingerBLink();
  if (!missing.empty()) {
    throw std::runtime_error("JointMap: model lacks expected links:" + missing);
  }
}

std::uint8_t JointMap::dxlId(int canonical) const {
  return canonicalJoints()[canonical].dxl_id;
}

std::optional<int> JointMap::canonicalOf(std::uint8_t dxl_id) const {
  const auto& joints = canonicalJoints();
  for (int i = 0; i < kCanonicalDof; ++i) {
    if (joints[i].dxl_id == dxl_id) return i;
  }
  return std::nullopt;
}

void JointMap::expand(const zVec q8, zVec q9) const {
  requireSize(q8, kCanonicalDof, "q8");
  requireSize(q9, kModelDof, "q9");
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(q9, offsets_[i]) = zVecElemNC(q8, i);
  }
  zVecElemNC(q9, offset_finger_b_) = zVecElemNC(q8, kCanonicalDof - 1);
}

void JointMap::reduce(const zVec q9, zVec q8) const {
  requireSize(q9, kModelDof, "q9");
  requireSize(q8, kCanonicalDof, "q8");
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(q8, i) = zVecElemNC(q9, offsets_[i]);
  }
}

void JointMap::reduceTorque(const zVec tau9, zVec tau8) const {
  reduce(tau9, tau8);
  zVecElemNC(tau8, kCanonicalDof - 1) += zVecElemNC(tau9, offset_finger_b_);
}

}  // namespace rtctrl::model
