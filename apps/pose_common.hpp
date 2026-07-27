// Position-mode placement shared by x7_pose and x7_ident --pose-first:
// a velocity-limited min-jerk move followed by goal-offset iterations
// that converge the MEASURED posture (position mode has no integral
// action — the stock P gain leaves 0.03-0.06 rad of friction sag, which
// already exceeds the identification anchor tolerance), with a final
// re-read immediately before the caller acts on the result.
#pragma once

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7 {

namespace model = rtctrl::model;

struct PoseResult {
  bool ok = false;          // moved and converged (or hit max iters)
  bool converged = false;   // measured error within tolerance
  double worst_dev = 0.0;   // final measured-vs-target [rad]
  int worst_joint = -1;
  std::vector<double> measured;   // the FINAL re-read posture
  std::vector<double> hold_goal;  // last commanded goal (for holding)
};

// Moves the activated position-mode arm to `target8`, then iterates
// goal offsets (goal = target + accumulated measured error) until the
// measured posture is within `tol` or `max_iters` is exhausted. The
// command stream stays alive throughout (both watchdog layers fed).
inline PoseResult movePose(rtctrl::hw::CraneX7& arm,
                           const double* target8, double vel, double tol,
                           int max_iters) {
  PoseResult res;
  constexpr int kCycleUs = 10000;  // 100 Hz
  constexpr double kBuffer = 0.05;  // stay clear of the gate band

  std::vector<rtctrl::dxl::Feedback> fb;
  if (!arm.readAll(fb)) return res;
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
  for (double t = 0.0; t <= move.duration(); t += 1e-6 * kCycleUs) {
    move.sample(t, q);
    for (int i = 0; i < n; ++i) cmd[i] = q[i];
    if (!arm.writePositions(cmd) && arm.escalated()) return res;
    if (!arm.checkDeadman()) return res;
    usleep(kCycleUs);
  }

  // goal-offset convergence: command target + accumulated measured
  // error; P = 800 pulls the measured posture through the friction sag
  std::vector<double> goal(n);
  for (int i = 0; i < n; ++i) goal[i] = target[i];
  for (int iter = 0; iter <= max_iters; ++iter) {
    // settle the servo loop while keeping the stream alive
    for (int k = 0; k < 30; ++k) {  // 0.3 s
      if (!arm.writePositions(goal) && arm.escalated()) return res;
      if (!arm.checkDeadman()) return res;
      usleep(kCycleUs);
    }
    if (!arm.readAll(fb)) return res;
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
    for (int i = 0; i < n; ++i) {
      goal[i] = std::clamp(goal[i] + (target[i] - fb[i].position),
                           lo[i] + kBuffer, hi[i] - kBuffer);
    }
  }

  // the FINAL re-read is what the caller may hand to a mode change —
  // "reached" must mean the posture NOW, not an earlier sample
  if (!arm.readAll(fb)) return res;
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
  res.ok = true;
  return res;
}

}  // namespace x7
