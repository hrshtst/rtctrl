// Emits a STATIC posture in both roki viewer formats, e.g. to visually
// confirm a canonical identification posture before hand-placing the
// arm (docs/HISTORY.md (identification)):
//   <out>.zvs      — a few 1 s frames;  rk_anim <model.ztk> <out>.zvs
//   <out>.init.ztk — [roki::chain::init] joint displacements;
//                    rk_pen -model <model.ztk> -init <out>.init.ztk
//
// Usage: make_pose <out-basename> --posture <file>   (e.g. the
//            checked-in config/postures/p1.json or a .dwells.json
//            sidecar — the 8 numbers after the "anchor" key)
//        make_pose <out-basename> q0 q1 q2 q3 q4 q5 q6 q7   [rad]

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

namespace model = rtctrl::model;

namespace {

// Reads the 8 numbers after an "anchor" key (JSON posture/sidecar
// files), or the first 8 numbers of a plain text file.
bool loadPosture(const char* path, double* out) {
  std::FILE* f = std::fopen(path, "r");
  if (!f) return false;
  std::string text;
  char buf[4096];
  std::size_t n;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
  std::fclose(f);
  std::size_t pos = 0;
  const auto key = text.find("\"anchor\"");
  if (key != std::string::npos) pos = key + 8;
  int found = 0;
  while (pos < text.size() && found < model::kCanonicalDof) {
    const char c = text[pos];
    if ((c >= '0' && c <= '9') ||
        ((c == '-' || c == '+') && pos + 1 < text.size() &&
         text[pos + 1] >= '0' && text[pos + 1] <= '9')) {
      char* end = nullptr;
      out[found++] = std::strtod(text.c_str() + pos, &end);
      pos = end - text.c_str();
    } else {
      ++pos;
    }
  }
  return found == model::kCanonicalDof;
}

}  // namespace

int main(int argc, char* argv[]) {
  double q[model::kCanonicalDof] = {};
  if (argc == 4 && std::strcmp(argv[2], "--posture") == 0) {
    if (!loadPosture(argv[3], q)) {
      std::fprintf(stderr, "cannot read %d joint values from %s\n",
                   model::kCanonicalDof, argv[3]);
      return 1;
    }
  } else if (argc == 2 + model::kCanonicalDof) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      q[i] = std::atof(argv[2 + i]);
    }
  } else {
    std::fprintf(stderr,
                 "usage: make_pose <out-basename> --posture <file>\n"
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

  // rk_pen initial-state file: [roki::chain::init] with one
  // "joint: <link name> <dis>" line per revolute joint.
  const std::string init_path = base + ".init.ztk";
  std::FILE* f = std::fopen(init_path.c_str(), "w");
  if (!f) {
    std::fprintf(stderr, "cannot open %s\n", init_path.c_str());
    return 1;
  }
  std::fprintf(f, "[roki::chain::init]\n");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(f, "joint: %s %.6f\n",
                 zName(rkChainLink(chain.chain(), map.linkId(i))), q[i]);
  }
  std::fprintf(f, "joint: %s %.6f\n",
               zName(rkChainLink(chain.chain(), map.linkIdFingerB())),
               q[model::kCanonicalDof - 1]);
  std::fclose(f);

  std::printf("wrote %s.zvs and %s\n"
              "view: rk_anim models/crane_x7/crane_x7.ztk %s.zvs\n"
              "  or: rk_pen -model models/crane_x7/crane_x7.ztk -init %s\n",
              base.c_str(), init_path.c_str(), base.c_str(),
              init_path.c_str());
  return 0;
}
