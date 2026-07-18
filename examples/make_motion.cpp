// Generates a simple CRANE-X7 motion as a .zvs sequence for rk_anim:
// zero pose -> ready pose -> zero pose, both legs velocity-limited
// minimum-jerk segments.
//
// Usage: make_motion [output.zvs]   (default: motion.zvs)
// View:  rk_anim models/crane_x7/crane_x7.ztk motion.zvs

#include <cstdio>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

namespace model = rtctrl::model;

int main(int argc, char* argv[]) {
  const char* out_path = argc > 1 ? argv[1] : "motion.zvs";
  constexpr double kDt = 0.01;
  constexpr double kVelocityLimit = 4.81710873 * 0.25;  // stay gentle

  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);

  model::ZVector zero(model::kCanonicalDof);
  model::ZVector ready(model::kCanonicalDof);
  ready[1] = 0.6;   // shoulder tilt
  ready[3] = -1.4;  // elbow
  ready[5] = -0.7;  // wrist pitch
  ready[7] = 0.5;   // open gripper

  const auto up = model::MinJerkTrajectory::withVelocityLimit(
      zero, ready, kVelocityLimit, 1.0);
  const auto down = model::MinJerkTrajectory::withVelocityLimit(
      ready, zero, kVelocityLimit, 1.0);

  model::ZVector q8(model::kCanonicalDof);
  model::ZVector q9(model::kModelDof);
  model::ZvsWriter writer(out_path);

  for (double t = 0.0; t <= up.duration(); t += kDt) {
    up.sample(t, q8);
    map.expand(q8, q9);
    writer.frame(kDt, q9);
  }
  for (double t = 0.0; t <= down.duration(); t += kDt) {
    down.sample(t, q8);
    map.expand(q8, q9);
    writer.frame(kDt, q9);
  }

  std::printf("wrote %d frames to %s\n", writer.frames(), out_path);
  std::printf("view with:  rk_anim models/crane_x7/crane_x7.ztk %s\n",
              out_path);
  return 0;
}
