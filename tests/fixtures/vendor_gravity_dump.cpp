// INDEPENDENT vendor-gravity fixture generator
// (docs/HISTORY.md (gravity calibration) M-GC2). Runs rt_manipulators'
// samples03 gravity computation — their link CSV, their recursion,
// their hand-excluding route, their tuned torque-to-current
// constants — at seven reference postures (four development, three
// held-out) and prints the per-joint currents plus the link-8 world
// position frozen in tests/unit/vendor_gravity_test.cpp.
//
// The static-torque-sum cross-check is EMBEDDED: the program aborts
// nonzero unless the vendor recursion equals an independent static sum
// over its own FK state to 1e-4 at every joint of every posture
// (review finding: a cross-check that lives only in a comment is not
// reproducible). The FK-agreement cross-check against rtctrl's model
// is a LIVE assertion in the unit test (frozen link-8 positions).
//
// With --replay (reviewer-directed sim step, back-drive exceptions):
// reads postures q0..q6 [rad], whitespace-separated, one per stdin
// line, and prints the vendor per-joint currents [A] per line — the
// same computation and embedded cross-check as the fixture mode.
// tools/replay_compare.py drives it over recorded feel logs.
//
// NOT part of the normal build (needs Eigen3 and the vendored tree).
// Regenerate from the repo root with:
//
//   g++ -std=c++17 -O2 \
//     -I rt_manipulators_cpp/rt_manipulators_lib/include \
//     -I rt_manipulators_cpp/samples/samples03/include \
//     -I /usr/include/eigen3 \
//     tests/fixtures/vendor_gravity_dump.cpp \
//     rt_manipulators_cpp/rt_manipulators_lib/src/kinematics.cpp \
//     rt_manipulators_cpp/rt_manipulators_lib/src/kinematics_utils.cpp \
//     rt_manipulators_cpp/samples/samples03/src/rt_manipulators_dynamics.cpp \
//     -o /tmp/vendor_gravity_dump && /tmp/vendor_gravity_dump
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "rt_manipulators_dynamics.hpp"

namespace {

// One posture through the vendor pipeline: set q, FK, gravity
// recursion, then the EMBEDDED static-sum cross-check. Returns false
// (with a stderr diagnostic) on any failure.
bool vendorCurrentsChecked(std::vector<manipulators_link::Link>& links,
                           const samples03_dynamics::torque_to_current_t& t2c,
                           const double* kt, const std::vector<double>& q,
                           kinematics_utils::q_list_t& q_list) {
  for (int i = 0; i < 7; ++i) links[2 + i].q = q[i];
  kinematics::forward_kinematics(links, 1);
  if (!samples03_dynamics::gravity_compensation(links, 8, t2c, q_list)) {
    std::fprintf(stderr, "vendor computation failed\n");
    return false;
  }
  // EMBEDDED cross-check: the recursion must equal an independent
  // static torque sum over the SAME FK state (route masses 2..8)
  Eigen::Vector3d g(0, 0, -9.8);
  int idx = 0;
  for (const auto& [id, cur] : q_list) {
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (int k = static_cast<int>(id); k <= 8; ++k) {
      const Eigen::Vector3d com_w = links[k].p + links[k].R * links[k].c;
      n += (com_w - links[id].p).cross(links[k].m * g);
    }
    const Eigen::Vector3d axis_w = links[id].R * links[id].a;
    const double tau_static = -axis_w.dot(n);
    if (std::fabs(tau_static - cur * kt[idx]) > 1e-4) {
      std::fprintf(stderr,
                   "CROSS-CHECK FAILED id %d: recursion %.6f vs "
                   "static %.6f\n",
                   static_cast<int>(id), cur * kt[idx], tau_static);
      return false;
    }
    ++idx;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const bool replay = argc > 1 && std::strcmp(argv[1], "--replay") == 0;
  auto links = kinematics_utils::parse_link_config_file(
      "rt_manipulators_cpp/samples/samples03/config/crane-x7_links.csv");
  kinematics::forward_kinematics(links, 1);
  samples03_dynamics::torque_to_current_t t2c = {
      {2, 1.0 / 2.20}, {3, 1.0 / 3.60}, {4, 1.0 / 2.20}, {5, 1.0 / 2.20},
      {6, 1.0 / 2.20}, {7, 1.0 / 2.20}, {8, 1.0 / 2.20}};
  const double kt[] = {2.20, 3.60, 2.20, 2.20, 2.20, 2.20, 2.20};

  if (replay) {
    // Posture stream mode: every line is checked by the same embedded
    // static sum as the fixture postures — a divergent replay aborts
    // instead of emitting silently wrong currents.
    std::vector<double> q(7);
    long lines = 0;
    while (std::scanf("%lf %lf %lf %lf %lf %lf %lf", &q[0], &q[1], &q[2],
                      &q[3], &q[4], &q[5], &q[6]) == 7) {
      kinematics_utils::q_list_t q_list;
      if (!vendorCurrentsChecked(links, t2c, kt, q, q_list)) return 1;
      for (const auto& [id, cur] : q_list) std::printf("%.9f ", cur);
      std::printf("\n");
      ++lines;
    }
    std::fprintf(stderr,
                 "replay: %ld postures, static-sum cross-check PASSED\n",
                 lines);
    return lines > 0 ? 0 : 1;
  }

  const std::vector<std::vector<double>> postures = {
      // development postures (used to set the engineering envelope)
      {0, 0, 0, 0, 0, 0, 0},
      {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0},
      {-0.10738, 0.50928, 0.19021, -1.87299, 0.02608, -0.79153, -0.05983},
      {-0.63353, -1.39285, 1.77942, -1.74260, -0.96487, -0.05676, -0.63200},
      // held-out postures (envelope evaluated, never tuned, on these)
      {-0.357, -0.831, 2.126, -1.572, -2.456, -0.106, 0.563},
      {0.4, 0.7, -0.3, -1.2, 0.3, -0.6, 0.5},
      {0.3, -0.9, 0.8, -1.2, 0.5, 0.4, 0.3}};
  for (const auto& p : postures) {
    kinematics_utils::q_list_t q_list;
    if (!vendorCurrentsChecked(links, t2c, kt, p, q_list)) return 1;
    std::printf("    {");
    for (const auto& [id, cur] : q_list) std::printf("%.9f, ", cur);
    std::printf("},  // link8 p: %.6f %.6f %.6f\n", links[8].p.x(),
                links[8].p.y(), links[8].p.z());
  }
  std::fprintf(stderr, "static-sum cross-check PASSED for all postures\n");
  return 0;
}
