// INDEPENDENT vendor-gravity fixture generator
// (docs/GRAVITY_CALIBRATION_PLAN.md M-GC2, review finding: the parity
// gate must exercise the VENDOR's own algorithm, not rtctrl's model on
// both sides). Runs rt_manipulators' samples03 gravity computation —
// their link CSV, their recursion, their hand-excluding route, their
// torque-to-current constants — at the reference postures and prints
// the per-joint currents frozen in tests/unit/vendor_gravity_test.cpp.
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
//
// Verified during generation (2026-07-30): the vendor recursion equals
// an independent static torque sum over its own FK to 4 decimals at
// every joint, and both models' forward kinematics agree to the
// millimeter at the reference postures — the joint conventions are
// identical, no sign mapping is needed.
#include <cstdio>
#include <vector>
#include "rt_manipulators_cpp/kinematics.hpp"
#include "rt_manipulators_cpp/kinematics_utils.hpp"
#include "rt_manipulators_cpp/link.hpp"
#include "rt_manipulators_dynamics.hpp"
int main() {
  auto links = kinematics_utils::parse_link_config_file(
      "rt_manipulators_cpp/samples/samples03/config/crane-x7_links.csv");
  kinematics::forward_kinematics(links, 1);
  samples03_dynamics::torque_to_current_t t2c = {
      {2, 1.0 / 2.20}, {3, 1.0 / 3.60}, {4, 1.0 / 2.20}, {5, 1.0 / 2.20},
      {6, 1.0 / 2.20}, {7, 1.0 / 2.20}, {8, 1.0 / 2.20}};
  const std::vector<std::vector<double>> postures = {
      {0, 0, 0, 0, 0, 0, 0},
      {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0},
      {-0.10738, 0.50928, 0.19021, -1.87299, 0.02608, -0.79153, -0.05983},
      {-0.63353, -1.39285, 1.77942, -1.74260, -0.96487, -0.05676,
       -0.63200}};
  for (const auto& p : postures) {
    for (int i = 0; i < 7; ++i) links[2 + i].q = p[i];
    kinematics::forward_kinematics(links, 1);
    kinematics_utils::q_list_t q_list;
    samples03_dynamics::gravity_compensation(links, 8, t2c, q_list);
    std::printf("    {");
    for (const auto& [id, cur] : q_list) std::printf("%.9f, ", cur);
    std::printf("},\n");
  }
  return 0;
}
