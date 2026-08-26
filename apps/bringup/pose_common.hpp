// Position-mode placement used by x7_pose (and formerly the removed
// x7_ident --pose-first startup):
// a velocity-limited min-jerk move followed by goal-offset iterations
// that converge the MEASURED posture (position mode has no integral
// action — the stock P gain leaves 0.03-0.06 rad of friction sag, which
// already exceeds the identification anchor tolerance), with a final
// re-read immediately before the caller acts on the result.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "bringup/write_monitor.hpp"
#include "common/periodic_loop.hpp"

namespace x7 {

namespace model = rtctrl::model;

struct PoseResult {
  bool ok = false;          // moved, converged, and had no write failure
  bool converged = false;   // measured error within tolerance
  double worst_dev = 0.0;   // final measured-vs-target [rad]
  int worst_joint = -1;
  int write_failures = 0;
  std::vector<double> measured;   // the FINAL re-read posture
  std::vector<double> hold_goal;  // last commanded goal (for holding)
};

inline bool placementAccepted(bool converged, int write_failures) {
  return converged && write_failures == 0;
}

// Moves the activated position-mode arm to `target8`, then iterates
// goal offsets (goal = target + accumulated measured error) until the
// measured posture is within `tol` or `max_iters` is exhausted. The
// command stream stays alive throughout (both watchdog layers fed).
inline PoseResult movePose(rtctrl::hw::CraneX7& arm,
                           const double* target8, double vel, double tol,
                           int max_iters) {
  PoseResult res;
  PositionWriteMonitor writes;
  const auto failed = [&] {
    res.write_failures = writes.failures();
    return res;
  };
  constexpr double kCycleS = 0.01;  // 100 Hz
  constexpr double kBuffer = 0.05;  // stay clear of the gate band

  std::vector<rtctrl::dxl::Feedback> fb;
  if (!arm.readAll(fb)) return failed();
  const int n = static_cast<int>(fb.size());
  const auto& lo = arm.softLimitLo();
  const auto& hi = arm.softLimitHi();
  model::ZVector start(n), target(n), q(n);
  for (int i = 0; i < n; ++i) {
    start[i] = fb[i].position;
    target[i] = std::clamp(target8[i], lo[i] + kBuffer, hi[i] - kBuffer);
    if (target[i] != target8[i]) {
      std::printf("joint %d target clamped %.3f -> %.3f (soft limit)\n",
                  i, target8[i], target[i]);
    }
  }

  const auto move = model::MinJerkTrajectory::withVelocityLimit(
      start, target, vel, 2.0);
  std::printf("moving to the posture in %.1f s (vel limit %.2f rad/s)\n",
              move.duration(), vel);
  std::vector<double> cmd(n);
  PeriodicLoop move_loop(kCycleS);
  while (true) {
    const double t = std::min(move_loop.elapsed(), move.duration());
    move.sample(t, q);
    for (int i = 0; i < n; ++i) cmd[i] = q[i];
    writes.record(arm.writePositions(cmd), "placement");
    if (arm.escalated()) return failed();
    if (!arm.checkDeadman()) return failed();
    if (t >= move.duration()) break;
    move_loop.waitNext();
  }
  if (move_loop.skippedPeriods() > 0) {
    std::fprintf(stderr,
                 "placement timing missed %llu period(s), max lateness "
                 "%.3f ms\n",
                 static_cast<unsigned long long>(move_loop.skippedPeriods()),
                 1e3 * move_loop.maxLateness());
    return failed();
  }

  // Goal-offset convergence: command target + accumulated measured
  // error; P = 800 pulls the measured posture through the friction
  // sag. BOUNDED (review finding): per-iteration steps and the total
  // accumulated offset are capped, and a stalled correction (a stuck
  // or non-improving joint) stops iterating instead of winding the
  // goal further into the mechanism.
  constexpr double kStepMax = 0.05;    // per-iteration offset [rad]
  constexpr double kOffsetMax = 0.15;  // total accumulated offset [rad]
  std::vector<double> goal(n), offset(n, 0.0);
  for (int i = 0; i < n; ++i) goal[i] = target[i];
  double prev_worst = 1e9;
  for (int iter = 0; iter <= max_iters; ++iter) {
    // settle the servo loop while keeping the stream alive
    PeriodicLoop settle_loop(kCycleS);
    while (settle_loop.elapsed() < 0.3) {
      writes.record(arm.writePositions(goal), "placement");
      if (arm.escalated()) return failed();
      if (!arm.checkDeadman()) return failed();
      settle_loop.waitNext();
    }
    if (settle_loop.skippedPeriods() > 0) {
      std::fprintf(stderr,
                   "placement settle timing missed %llu period(s), max "
                   "lateness %.3f ms\n",
                   static_cast<unsigned long long>(
                       settle_loop.skippedPeriods()),
                   1e3 * settle_loop.maxLateness());
      return failed();
    }
    if (!arm.readAll(fb)) return failed();
    res.worst_dev = 0.0;
    for (int i = 0; i < n; ++i) {
      const double e = target[i] - fb[i].position;
      if (std::fabs(e) > res.worst_dev) {
        res.worst_dev = std::fabs(e);
        res.worst_joint = i;
      }
    }
    if (res.worst_dev <= tol) {
      res.converged = true;
      break;
    }
    if (iter == max_iters) break;
    if (res.worst_dev > 0.9 * prev_worst) {
      std::printf("placement correction stalled (%.4f -> %.4f rad on "
                  "joint %d) — stopping the iterations\n",
                  prev_worst, res.worst_dev, res.worst_joint);
      break;
    }
    prev_worst = res.worst_dev;
    for (int i = 0; i < n; ++i) {
      const double step = std::clamp(target[i] - fb[i].position,
                                     -kStepMax, kStepMax);
      offset[i] = std::clamp(offset[i] + step, -kOffsetMax, kOffsetMax);
      goal[i] = std::clamp(target[i] + offset[i], lo[i] + kBuffer,
                           hi[i] - kBuffer);
    }
  }

  // the FINAL re-read is what the caller may hand to a mode change —
  // "reached" must mean the posture NOW, not an earlier sample
  if (!arm.readAll(fb)) return failed();
  res.measured.resize(n);
  res.worst_dev = 0.0;
  for (int i = 0; i < n; ++i) {
    res.measured[i] = fb[i].position;
    const double e = target[i] - fb[i].position;
    if (std::fabs(e) > res.worst_dev) {
      res.worst_dev = std::fabs(e);
      res.worst_joint = i;
    }
    std::printf("  joint %d: measured %+.4f vs target %+.4f (%+.4f)\n",
                i, fb[i].position, target[i], fb[i].position - target[i]);
  }
  res.hold_goal = goal;
  res.write_failures = writes.failures();
  res.ok = placementAccepted(res.converged, res.write_failures);
  return res;
}

}  // namespace x7
