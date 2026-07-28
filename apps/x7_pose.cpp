// Positioning aid for the identification campaign: moves the arm to a
// canonical posture in POSITION mode so the operator sees the real
// target on hardware, holds it there, then goes limp on request while
// the operator supports the arm. x7_ident then runs UNCHANGED (its
// settle gate and +/-0.02 rad anchor gate still do the verifying) —
// hand placement reduces to briefly steadying an already-posed arm
// through the torque-off.
//
// SAFETY: the arm is servo-stiff during the move and DROPS under
// gravity the moment it goes limp — support the shoulder and elbow
// BEFORE pressing Enter. Power cutoff within reach.
//
// Usage: x7_pose [--config path] [--port dev] --posture <file> [--vel v]
//   --posture  target vector: a checked-in config/postures/*.json, a
//              .dwells.json sidecar, or any file with 8 numbers
//   --vel      joint speed limit [rad/s], default 0.25, max 0.5

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ident_common.hpp"
#include "pose_common.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "x7_common.hpp"

namespace model = rtctrl::model;

namespace {

bool enterPressed() {
  struct pollfd pfd = {0 /*stdin*/, POLLIN, 0};
  if (poll(&pfd, 1, 0) <= 0) return false;
  char buf[64];
  return read(0, buf, sizeof buf) > 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  std::string posture_path;
  double vel = 0.25;
  const auto& rest = cli.rest;
  for (std::size_t i = 0; i < rest.size(); ++i) {
    if (std::strcmp(rest[i], "--posture") == 0 && i + 1 < rest.size()) {
      posture_path = rest[++i];
    } else if (std::strcmp(rest[i], "--vel") == 0 &&
               i + 1 < rest.size()) {
      if (!x7::parseStrictDouble(rest[++i], &vel)) {
        std::fprintf(stderr, "--vel: invalid value %s\n", rest[i]);
        return 1;
      }
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", rest[i]);
      return 1;
    }
  }
  if (posture_path.empty()) {
    std::fprintf(stderr, "--posture <file> is required\n");
    return 1;
  }
  vel = std::clamp(vel, 0.05, 0.5);
  double posture[model::kCanonicalDof];
  if (!x7::loadAnchorRef(posture_path, posture)) {
    std::fprintf(stderr, "cannot read %d joint values from %s\n",
                 model::kCanonicalDof, posture_path.c_str());
    return 1;
  }

  try {
    auto session = x7::openSession(cli, /*operating_mode_override=*/3);
    auto& arm = *session.arm;
    if (!arm.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   arm.lastError().c_str());
      return 1;
    }
    x7::ShutdownGuard shutdown{arm};

    // converging placement: goal-offset iterations close the friction
    // sag so "reached" means the MEASURED posture
    const auto placed = x7::movePose(arm, posture, vel, 0.01, 5);
    if (!placed.ok) {
      std::fprintf(stderr, "placement failed — deactivating\n");
      shutdown.run();
      return 1;
    }
    if (!placed.converged) {
      std::printf("NOTE: joint %d still %.4f rad from target after "
                  "convergence iterations\n",
                  placed.worst_joint, placed.worst_dev);
    }
    const int n = static_cast<int>(placed.hold_goal.size());
    constexpr int kCycleUs = 10000;  // 100 Hz

    std::printf(
        "\nHOLDING the posture in position mode.\n"
        ">>> The arm goes LIMP and drops when you continue. CATCH,\n"
        ">>> don't hold: support from below (forearm/under-elbow)\n"
        ">>> only against the drop — do NOT grip or lift, or the\n"
        ">>> posture is lost. Have the x7_ident command ready in\n"
        ">>> another terminal, then press Enter to torque off.\n");
    // keep the command stream (and both watchdog layers) alive while
    // waiting — a silent bus would trip the servo Bus Watchdogs
    (void)n;
    while (!enterPressed()) {
      if (!arm.writePositions(placed.hold_goal) && arm.escalated()) {
        shutdown.run();
        return 1;
      }
      if (!arm.checkDeadman()) {
        shutdown.run();
        return 1;
      }
      usleep(kCycleUs);
    }
    const bool clean = shutdown.run();
    std::printf("%s — the arm is limp. Catch the drop, start x7_ident "
                "immediately, and RELEASE fully the moment it prints "
                "its release cue (its gravity hold floats — sustained "
                "contact re-poses the arm).\n",
                clean ? "torque off" : "torque off INCOMPLETE — check "
                                       "the arm before proceeding");
    return clean ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
