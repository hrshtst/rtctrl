// Bring-up step 4: a small, slow minimum-jerk move of ONE joint and
// back, with limit clamps and the two-layer watchdog live. Start with
// the wrist (canonical index 6, DXL id 8) per the bring-up checklist.
//
// Usage: x7_move_simple [--config path] [--port dev] [joint_index] [delta_rad]
//   joint_index: canonical 0..7 (default 6 = wrist)
//   delta_rad:   move amplitude (default 0.3, clamped to 0.6)

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "x7_common.hpp"

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  int joint = 6;
  double delta = 0.3;
  if (cli.argi < argc) joint = std::atoi(argv[cli.argi]);
  if (cli.argi + 1 < argc) delta = std::atof(argv[cli.argi + 1]);
  delta = std::clamp(delta, -0.6, 0.6);
  if (joint < 0 || joint > 7) {
    std::fprintf(stderr, "joint index must be 0..7\n");
    return 2;
  }

  try {
    auto session = x7::openSession(cli);
    auto& arm = *session.arm;
    if (!arm.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   arm.lastError().c_str());
      return 1;
    }

    std::vector<rtctrl::dxl::Feedback> fb;
    if (!arm.readAll(fb)) return 1;
    const int n = static_cast<int>(fb.size());
    rtctrl::model::ZVector start(n), target(n), q(n);
    for (int i = 0; i < n; ++i) start[i] = fb[i].position;
    zVecCopyNC(start.get(), target.get());
    target[joint] += delta;

    constexpr double kGentleVel = 0.5;  // rad/s
    const auto out = rtctrl::model::MinJerkTrajectory::withVelocityLimit(
        start, target, kGentleVel, 1.5);
    const auto back = rtctrl::model::MinJerkTrajectory::withVelocityLimit(
        target, start, kGentleVel, 1.5);
    std::printf("moving joint %d by %+.2f rad and back (%.1f s each way)\n",
                joint, delta, out.duration());

    constexpr int kCycleUs = 10000;  // 100 Hz
    std::vector<double> cmd(n);
    auto runLeg = [&](const rtctrl::model::MinJerkTrajectory& leg) {
      for (double t = 0.0; t <= leg.duration(); t += 1e-6 * kCycleUs) {
        leg.sample(t, q);
        for (int i = 0; i < n; ++i) cmd[i] = q[i];
        if (!arm.writePositions(cmd) && arm.escalated()) return false;
        if (!arm.checkDeadman()) return false;
        usleep(kCycleUs);
      }
      return true;
    };

    const bool ok = runLeg(out) && runLeg(back);
    std::printf("%s — deactivating\n", ok ? "complete" : "ABORTED");
    arm.deactivate();
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
