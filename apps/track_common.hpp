// Shared between x7_track (hardware) and x7_track_sim: the wrapped
// computed-torque controller with per-joint tracking statistics and
// optional per-cycle CSV logging.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/crane_x7_tuning.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7 {

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
namespace tuning = rtctrl::arm::tuning;

// The shipped tuning lives in the library (one source of truth, also
// exercised by the tests): rtctrl/arm/crane_x7_tuning.hpp.
using tuning::kGainScale;

// Gravity compensation plus light filtered damping: holds the arm AND
// bleeds off swing, unlike pure GravityComp which floats. Used before
// tracking so the trajectory anchors on a quiescent arm (the
// track3.csv log caught the twist entering the tracking loop at
// 2.6 rad/s after a fixed 1 s undamped settle).
struct SettleController : arm::Controller {
  SettleController(model::ChainModel& chain, const model::JointMap& map,
                   double kd)
      : chain_(chain), map_(map), kd_(kd) {}

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    cmd.mode = arm::ControlMode::Current;
    chain_.gravityTorque(map_, state.q.get(), cmd.tau.get());
    if (t_prev_ < 0.0) {
      // First sample: record positions only. dq_est_ stays zero and no
      // damping is emitted — the servo's lagged dq must never seed the
      // estimator (it would contaminate the filtered samples after it).
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        q_prev_[i] = zVecElemNC(state.q.get(), i);
      }
      t_prev_ = t;
      return;
    }
    const double dt = t - t_prev_;
    if (dt <= 0.0) {
      // Duplicate sample (defense in depth — the loop already skips
      // these): hold every filter state, keep emitting the held damping.
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        zVecElemNC(cmd.tau.get(), i) += d_filt_[i];
      }
      return;
    }
    const double a_v = dt / (0.02 + dt);
    const double a_d = dt / (0.05 + dt);
    const double a_m = dt / (0.15 + dt);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double q = zVecElemNC(state.q.get(), i);
      const double raw = (q - q_prev_[i]) / dt;
      dq_est_[i] += a_v * (raw - dq_est_[i]);
      d_filt_[i] += a_d * (-kd_ * kGainScale[i] * dq_est_[i] - d_filt_[i]);
      zVecElemNC(cmd.tau.get(), i) += d_filt_[i];
      // slow-filtered |speed| for the quiescence metric: the fast
      // estimator hovers near the position-LSB noise floor (~0.05
      // rad/s spikes) even on a perfectly still arm
      speed_[i] += a_m * (std::fabs(dq_est_[i]) - speed_[i]);
      q_prev_[i] = q;
    }
    t_prev_ = t;
  }

  double maxSpeed() const {
    double m = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      m = std::max(m, speed_[i]);
    }
    return m;
  }

  model::ChainModel& chain_;
  const model::JointMap& map_;
  double kd_;
  double t_prev_ = -1.0;
  model::ZVector q_prev_{model::kCanonicalDof};
  model::ZVector dq_est_{model::kCanonicalDof};
  model::ZVector d_filt_{model::kCanonicalDof};
  model::ZVector speed_{model::kCanonicalDof};
};

// Policy-free settle outcome — the CALLER decides whether a
// non-quiescent timeout is fatal (hardware: yes; sim twin with
// --disturb seeds: warn and continue).
struct SettleResult {
  bool io_ok = false;      // the arm layer stayed healthy
  bool quiescent = false;  // residual below threshold at exit
  double elapsed = 0.0;    // measured time spent settling [s]
  double residual = 0.0;   // final slow-filtered max joint speed [rad/s]
};

// Runs the settle controller until the arm is quiescent (max estimated
// joint speed below 0.05 rad/s for 0.3 s of measured time) or timeout,
// under the same measured-time/stale-feedback policy as arm::run().
inline SettleResult settleArm(arm::Arm& robot, SettleController& settle,
                              double timeout_s) {
  arm::JointState state;
  arm::JointCommand cmd;
  SettleResult res;
  const double dt = robot.dt();
  const long max_cycles =
      static_cast<long>(4.0 * timeout_s / (dt > 0.0 ? dt : 1e-3)) + 8;
  bool have_t0 = false;
  double t0 = 0.0;
  double t_prev = 0.0;
  std::uint64_t seq_prev = 0;
  double quiet = 0.0;
  double t = 0.0;
  for (long cycle = 0; cycle < max_cycles; ++cycle) {
    if (!robot.readState(state)) return res;
    if (!have_t0) {
      t0 = t_prev = state.t;
      seq_prev = state.seq;
      have_t0 = true;
    } else {
      if (state.t < t_prev) return res;  // backward clock
      if (state.seq == seq_prev) {       // duplicate feedback
        if (!robot.step()) return res;
        continue;
      }
      if (state.seq - seq_prev > arm::kMaxSeqGap) return res;
      if (state.t - t_prev > arm::kMaxSampleInterval) return res;
      quiet = settle.maxSpeed() < 0.05 ? quiet + (state.t - t_prev) : 0.0;
      t_prev = state.t;
      seq_prev = state.seq;
    }
    t = state.t - t0;
    if (t >= timeout_s) break;
    settle.update(state, cmd, t);
    if (!robot.writeCommand(cmd)) return res;
    if (!robot.step()) return res;
    if (t > 0.5 && quiet >= 0.3) break;
  }
  res.io_ok = true;
  res.elapsed = t;
  res.residual = settle.maxSpeed();
  res.quiescent = res.residual < 0.05;
  std::printf("settled in %.1f s (residual %.3f rad/s)%s\n", res.elapsed,
              res.residual, res.quiescent ? "" : " — still moving");
  return res;
}

// Wraps ComputedTorque, accumulates per-joint tracking error, and
// optionally logs every cycle to CSV.
struct TrackingRun : arm::Controller {
  TrackingRun(model::ChainModel& chain, const model::JointMap& map,
              const model::Trajectory& trajectory, double kp,
              double kd, int leg, std::FILE* log)
      : inner(chain, map, trajectory, kp, kd),
        trajectory_(trajectory),
        leg_(leg),
        log_(log) {}

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    inner.update(state, cmd, t);
    trajectory_.sample(t, q_d.get(), dq_d.get());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d[i] - zVecElemNC(state.q.get(), i);
      sum_sq[i] += e * e;
      max_abs[i] = std::max(max_abs[i], std::fabs(e));
    }
    ++samples;
    if (log_) {
      std::fprintf(log_, "%d,%.4f", leg_, t);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::fprintf(log_, ",%.5f,%.5f,%.5f,%.5f,%.4f", q_d[i],
                     zVecElemNC(state.q.get(), i), dq_d[i],
                     zVecElemNC(state.dq.get(), i),
                     zVecElemNC(cmd.tau.get(), i));
      }
      std::fprintf(log_, "\n");
    }
  }

  double rms() const {
    if (!samples) return 0.0;
    double total = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) total += sum_sq[i];
    return std::sqrt(total / (samples * model::kCanonicalDof));
  }
  double rms(int i) const {
    return samples ? std::sqrt(sum_sq[i] / samples) : 0.0;
  }

  void report() const {
    std::printf("%s RMS: %.4f rad\n", leg_ == 0 ? "out " : "back", rms());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (max_abs[i] < 1e-4) continue;  // joint not commanded to move
      std::printf("  [%d] %-22s RMS %.4f  max %.4f rad\n", i,
                  model::canonicalJoints()[i].urdf_joint, rms(i),
                  max_abs[i]);
    }
  }

  arm::ComputedTorque inner;
  const model::Trajectory& trajectory_;
  int leg_;
  std::FILE* log_;
  model::ZVector q_d{model::kCanonicalDof};
  model::ZVector dq_d{model::kCanonicalDof};
  double sum_sq[model::kCanonicalDof] = {};
  double max_abs[model::kCanonicalDof] = {};
  long samples = 0;
};

inline std::FILE* openCsvLog(const std::string& path) {
  std::FILE* log = std::fopen(path.c_str(), "w");
  if (!log) return nullptr;
  std::fprintf(log, "leg,t");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log, ",qd%d,q%d,dqd%d,dq%d,tau%d", i, i, i, i, i);
  }
  std::fprintf(log, "\n");
  return log;
}

}  // namespace x7
