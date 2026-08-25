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
#include "bringup/move_simple_common.hpp"
#include "bringup/write_monitor.hpp"
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
    std::vector<double> start_values(n);
    for (int i = 0; i < n; ++i) {
      start[i] = fb[i].position;
      start_values[i] = fb[i].position;
    }
    zVecCopyNC(start.get(), target.get());
    const auto endpoint = x7::clampMoveEndpoint(
        start[j], delta, arm.softLimitLo()[j], arm.softLimitHi()[j]);
    target[j] = endpoint.target;
    if (endpoint.clamped) {
      std::fprintf(stderr,
                   "WARNING: joint %d displacement clamped from %+.3f to "
                   "%+.3f rad (soft limit)\n",
                   j, delta, endpoint.displacement);
    }

    constexpr double kGentleVel = 0.5;  // rad/s
    const auto out = rtctrl::model::MinJerkTrajectory::withVelocityLimit(
        start, target, kGentleVel, 1.5);
    const auto back = rtctrl::model::MinJerkTrajectory::withVelocityLimit(
        target, start, kGentleVel, 1.5);
    std::printf("moving joint %d by %+.2f rad and back (%.1f s each way)\n",
                j, endpoint.displacement, out.duration());

    constexpr int kCycleUs = 10000;  // 100 Hz
    std::vector<double> cmd(n);
    x7::PositionWriteMonitor writes;
    auto runLeg = [&](const rtctrl::model::MinJerkTrajectory& leg) {
      for (double t = 0.0; t <= leg.duration(); t += 1e-6 * kCycleUs) {
        leg.sample(t, q);
        for (int i = 0; i < n; ++i) cmd[i] = q[i];
        writes.record(arm.writePositions(cmd), "move");
        if (arm.escalated()) return false;
        if (!arm.checkDeadman()) return false;
        usleep(kCycleUs);
      }
      return true;
    };

    // deactivate FIRST, then report — success text must never print
    // over an unverified shutdown (review finding)
    bool legs_ok = runLeg(out) && runLeg(back);
    // Let the position loop settle at the start posture while continuing
    // to feed both watchdog layers, then verify the measured return.
    for (int k = 0; legs_ok && k < 30; ++k) {  // 0.3 s
      writes.record(arm.writePositions(start_values), "return hold");
      if (arm.escalated() || !arm.checkDeadman()) {
        legs_ok = false;
        break;
      }
      usleep(kCycleUs);
    }
    bool returned = false;
    if (legs_ok) {
      std::vector<rtctrl::dxl::Feedback> final_fb;
      if (!arm.readAll(final_fb)) {
        std::fprintf(stderr, "final posture read failed\n");
      } else {
        constexpr double kReturnTolerance = 0.05;  // rad
        const auto check = x7::checkReturnPosture(
            final_fb, start_values, kReturnTolerance);
        if (!check.valid) {
          std::fprintf(stderr, "final posture has the wrong joint count\n");
        } else if (!check.within_tolerance) {
          std::fprintf(stderr,
                       "return posture verification failed: joint %d is "
                       "%.4f rad from its start (limit %.4f rad)\n",
                       check.worst_joint, check.worst_deviation,
                       kReturnTolerance);
        } else {
          returned = true;
        }
      }
    }
    writes.reportSummary("move");
    const bool ok = legs_ok && writes.ok() && returned;
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
