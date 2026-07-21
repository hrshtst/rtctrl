// M8 hardware phase: computed-torque trajectory tracking on the real
// arm — tau = ID(q_d, dq_d, ddq_d) + Kp e + Kd de in current mode,
// a small minimum-jerk excursion from the present pose and back, at
// reduced speed. Preview the identical run with x7_track_sim first.
//
// SAFETY: current mode. Power cutoff in reach, workspace clear.
//
// Gains: the sim acceptance runs Kp=20/Kd=2 (tracking_sim_test, RMS
// 0.0050 rad), but on the real arm those gains oscillate (2026-07-21:
// RMS 0.07 rad at half scale). The x7_track_sim lag sweep pinned the
// cause: the servo's slow PresentVelocity estimate (~0.1 s effective
// lag) turns the Kd term destabilizing at Kp=20; the loop's ~1-cycle
// command pipeline latency alone is harmless. The softer defaults
// below survive a 0.2 s estimator lag in sim with margin; the
// inverse-dynamics feedforward still carries the trajectory.
//
// Usage: x7_track [--config path] [--port dev]
//                 [--kp v] [--kd v] [--log out.csv] [scale]
//   scale in (0,1] shrinks the excursion (default 1.0 ≈ ±0.3 rad max)
//   --log writes t, q_d, q, dq_d, dq, tau per joint each cycle

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "track_common.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  double scale = 1.0;
  // Hardware defaults, softer than the sim-verified 20/2 — see header.
  double kp = 8.0, kd = 1.2;
  std::string log_path;
  for (int i = cli.argi; i < argc; ++i) {
    if (std::strcmp(argv[i], "--kp") == 0 && i + 1 < argc) {
      kp = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--kd") == 0 && i + 1 < argc) {
      kd = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
      log_path = argv[++i];
    } else {
      scale = std::atof(argv[i]);
    }
  }
  scale = std::clamp(scale, 0.05, 1.0);
  kp = std::clamp(kp, 0.0, 50.0);
  kd = std::clamp(kd, 0.0, 5.0);

  std::FILE* log = nullptr;
  if (!log_path.empty()) {
    log = x7::openCsvLog(log_path);
    if (!log) {
      std::fprintf(stderr, "cannot open log file %s\n", log_path.c_str());
      return 1;
    }
  }

  try {
    auto session = x7::openSession(cli, /*operating_mode_override=*/0);
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);
    arm::RealArm robot(*session.arm);

    if (!robot.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   session.arm->lastError().c_str());
      return 1;
    }
    arm::JointState start;
    if (!robot.readState(start)) return 1;

    model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
    zVecCopyNC(start.q.get(), q0.get());
    zVecCopyNC(start.q.get(), qf.get());
    // gentle excursion on tilt / elbow / wrist pitch
    qf[1] += 0.20 * scale;
    qf[3] -= 0.30 * scale;
    qf[5] += 0.25 * scale;

    constexpr double kVel = 0.3;  // rad/s — reduced speed
    const auto out = model::MinJerkTrajectory::withVelocityLimit(
        q0, qf, kVel, 2.0);
    const auto back = model::MinJerkTrajectory::withVelocityLimit(
        qf, q0, kVel, 2.0);

    std::printf("computed-torque tracking: %.1f s out, %.1f s back "
                "(scale %.2f, Kp %.1f, Kd %.2f)\n",
                out.duration(), back.duration(), scale, kp, kd);

    x7::TrackingRun leg1(chain, map, out, kp, kd, 0, log);
    bool ok = arm::run(robot, leg1, out.duration());
    leg1.report();

    if (ok) {
      x7::TrackingRun leg2(chain, map, back, kp, kd, 1, log);
      ok = arm::run(robot, leg2, back.duration());
      leg2.report();
    }

    robot.deactivate();
    if (log) std::fclose(log);
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    if (log) std::fclose(log);
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
