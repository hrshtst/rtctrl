// Emits a STATIC posture in both roki viewer formats, e.g. to visually
// confirm a canonical identification posture before hand-placing the
// arm (docs/records/history.md (identification)):
//   <out>.zvs      — a few 1 s frames;  rk_anim <model.ztk> <out>.zvs
//   <out>.init.ztk — [roki::chain::init] joint displacements;
//                    rk_pen -model <model.ztk> -init <out>.init.ztk
//
// Usage: make_pose <out-basename> --posture <file.toml>
//        make_pose <out-basename> --legacy-anchor-sidecar <file.dwells.json>
//        make_pose <out-basename> q0 q1 q2 q3 q4 q5 q6 q7   [rad]

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

#include "common/legacy_anchor.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/posture.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

namespace model = rtctrl::model;

int main(int argc, char* argv[]) {
  double q[model::kCanonicalDof] = {};
  if (argc == 4 && std::strcmp(argv[2], "--posture") == 0) {
    try {
      const auto posture = model::loadPostureToml(argv[3]);
      std::copy(posture.joint_positions.begin(),
                posture.joint_positions.end(), q);
    } catch (const std::exception& error) {
      std::fprintf(stderr, "cannot load posture %s: %s\n", argv[3],
                   error.what());
      return 1;
    }
  } else if (argc == 4 &&
             std::strcmp(argv[2], "--legacy-anchor-sidecar") == 0) {
    if (!x7::loadLegacyAnchorSidecar(argv[3], q)) {
      std::fprintf(stderr, "cannot read legacy anchor from %s\n", argv[3]);
      return 1;
    }
  } else if (argc == 2 + model::kCanonicalDof) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      q[i] = std::atof(argv[2 + i]);
    }
  } else {
    std::fprintf(stderr,
                 "usage: make_pose <out-basename> --posture <file.toml>\n"
                 "       make_pose <out-basename> "
                 "--legacy-anchor-sidecar <file.dwells.json>\n"
                 "       make_pose <out-basename> q0 .. q%d  [rad]\n",
                 model::kCanonicalDof - 1);
    return 1;
  }
  const std::string base = argv[1];

  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  model::ZVector q8(model::kCanonicalDof), q9(model::kModelDof);
  for (int i = 0; i < model::kCanonicalDof; ++i) q8[i] = q[i];
  map.expand(q8, q9);  // canonical 8 -> model 9 incl. the mimic finger

  {
    model::ZvsWriter writer(base + ".zvs");
    for (int k = 0; k < 5; ++k) writer.frame(1.0, q9.get());
  }

  // rk_pen initial-state file, through roki's own writer: the
  // [roki::chain::init] format stores revolute displacements in
  // DEGREES, so a hand-rolled radian file loads as a near-zero pose.
  const std::string init_path = base + ".init.ztk";
  if (!chain.writeInitZtk(init_path, q9.get())) {
    std::fprintf(stderr, "cannot write %s\n", init_path.c_str());
    return 1;
  }

  std::printf("wrote %s.zvs and %s\n"
              "view: rk_anim models/crane_x7/crane_x7.ztk %s.zvs\n"
              "  or: rk_pen -model models/crane_x7/crane_x7.ztk -init %s\n",
              base.c_str(), init_path.c_str(), base.c_str(),
              init_path.c_str());
  return 0;
}
