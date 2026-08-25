// Bring-up step 6: a small, slow minimum-jerk move of ONE joint and
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
#include "common/x7_common.hpp"

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  long joint = 6;
  double delta = 0.3;
  if (cli.rest.size() > 2) {
    std::fprintf(stderr, "usage: x7_move_simple [--config path] "
                         "[--port dev] [joint_index] [delta_rad]\n");
    return 2;
  }
  if (!cli.rest.empty() && !x7::parseStrictLong(cli.rest[0], &joint)) {
    std::fprintf(stderr, "invalid joint index: %s\n", cli.rest[0]);
    return 2;
  }
  if (cli.rest.size() > 1 &&
      !x7::parseStrictDouble(cli.rest[1], &delta)) {
    std::fprintf(stderr, "invalid delta: %s\n", cli.rest[1]);
    return 2;
  }
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
    x7::ShutdownGuard shutdown{arm};

    std::vector<rtctrl::dxl::Feedback> fb;
    if (!arm.readAll(fb)) {
      std::fprintf(stderr, "initial read failed\n");
      shutdown.run();
      return 1;
    }
    const int n = static_cast<int>(fb.size());
    const int j = static_cast<int>(joint);
    rtctrl::model::ZVector start(n), target(n), q(n);
    for (int i = 0; i < n; ++i) start[i] = fb[i].position;
    zVecCopyNC(start.get(), target.get());
    target[j] += delta;

    constexpr double kGentleVel = 0.5;  // rad/s
    const auto out = rtctrl::model::MinJerkTrajectory::withVelocityLimit(
        start, target, kGentleVel, 1.5);
    const auto back = rtctrl::model::MinJerkTrajectory::withVelocityLimit(
        target, start, kGentleVel, 1.5);
    std::printf("moving joint %d by %+.2f rad and back (%.1f s each way)\n",
                j, delta, out.duration());

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

    // deactivate FIRST, then report — success text must never print
    // over an unverified shutdown (review finding)
    const bool ok = runLeg(out) && runLeg(back);
    const bool clean = shutdown.run();
    if (!clean) {
      std::printf("SHUTDOWN FAULT (move %s)\n",
                  ok ? "complete" : "ABORTED");
      return 1;
    }
    std::printf("%s\n", ok ? "complete" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
