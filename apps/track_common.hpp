// Shared between x7_track (hardware) and x7_track_sim: the wrapped
// computed-torque controller with per-joint tracking statistics and
// optional per-cycle CSV logging.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>

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
      : chain_(chain), map_(map), kd_(kd) {
    // The quiescence metric must not read "quiet" before it has ever
    // measured anything: it starts pessimistic and only relaxes once
    // the position window is full.
    for (int i = 0; i < model::kCanonicalDof; ++i) speed_[i] = 1.0;
  }

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
      q_prev_[i] = q;
    }
    // Quiescence metric: velocity over a ~0.15 s WINDOWED position
    // difference, then slow-filtered. The per-sample backward
    // difference sits on the encoder-LSB noise floor — a real arm at
    // rest under current control dithers within +/-1 count, and one
    // 0.0015 rad flip over 10 ms reads 0.15 rad/s, which held the old
    // metric at ~0.07 rad/s on a STILL arm (hardware pass-1 run, gate
    // never opened). The same flip over the window is 0.010 rad/s.
    hist_t_[hist_head_] = t;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      hist_q_[hist_head_][i] = zVecElemNC(state.q.get(), i);
    }
    hist_head_ = (hist_head_ + 1) % kWindow;
    if (hist_count_ < kWindow) {
      ++hist_count_;  // metric stays pessimistic until the window fills
    } else {
      const int oldest = hist_head_;  // overwritten next call = oldest
      const double span = t - hist_t_[oldest];
      if (span > 0.0) {
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          const double wraw =
              std::fabs(zVecElemNC(state.q.get(), i) -
                        hist_q_[oldest][i]) /
              span;
          speed_[i] += a_m * (wraw - speed_[i]);
        }
      }
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
  double speed(int i) const { return speed_[i]; }

  static constexpr int kWindow = 16;  // ~0.15 s at the 10 ms cycle

  model::ChainModel& chain_;
  const model::JointMap& map_;
  double kd_;
  double t_prev_ = -1.0;
  model::ZVector q_prev_{model::kCanonicalDof};
  model::ZVector dq_est_{model::kCanonicalDof};
  model::ZVector d_filt_{model::kCanonicalDof};
  model::ZVector speed_{model::kCanonicalDof};
  double hist_t_[kWindow] = {};
  double hist_q_[kWindow][model::kCanonicalDof] = {};
  int hist_head_ = 0;
  int hist_count_ = 0;
};

// Policy-free settle outcome — the CALLER decides whether a
// non-quiescent timeout is fatal (hardware: yes; sim twin with
// --disturb seeds: warn and continue).
struct SettleResult {
  bool io_ok = false;      // the arm layer stayed healthy
  bool quiescent = false;  // the SUSTAINED 0.3 s quiet window was reached
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
  // Same first-sample discipline as arm::run(): the initial read may be
  // arbitrarily old, so it is controlled at t = 0 (commands must flow
  // every cycle) and the origin latches on the first FRESH sample.
  bool have_ref = false;
  bool have_t0 = false;
  double t0 = 0.0;
  double t_prev = 0.0;
  std::uint64_t seq_prev = 0;
  double quiet = 0.0;
  double t = 0.0;
  bool reached_quiet_window = false;
  for (long cycle = 0; cycle < max_cycles; ++cycle) {
    if (!robot.readState(state)) return res;
    double step_dt = 0.0;
    if (!have_ref) {
      seq_prev = state.seq;
      have_ref = true;
      t = 0.0;
    } else if (state.seq == seq_prev) {  // duplicate feedback
      if (!robot.step()) return res;
      continue;
    } else if (!have_t0) {
      t0 = t_prev = state.t;
      seq_prev = state.seq;
      have_t0 = true;
      t = 0.0;
    } else {
      if (state.t < t_prev) return res;  // backward clock
      if (state.seq - seq_prev > arm::kMaxSeqGap) return res;
      if (state.t - t_prev > arm::kMaxSampleInterval) return res;
      step_dt = state.t - t_prev;
      t_prev = state.t;
      seq_prev = state.seq;
      t = state.t - t0;
    }
    if (t >= timeout_s) break;
    settle.update(state, cmd, t);
    // The quiet window must be judged on the speed AFTER this sample
    // was processed: accumulating before update() once let motion that
    // began exactly on the acceptance sample slip through the gate.
    quiet = settle.maxSpeed() < 0.05 ? quiet + step_dt : 0.0;
    if (!robot.writeCommand(cmd)) return res;
    if (!robot.step()) return res;
    if (t > 0.5 && quiet >= 0.3) {
      reached_quiet_window = true;
      break;
    }
  }
  res.io_ok = true;
  res.elapsed = t;
  res.residual = settle.maxSpeed();
  // Quiescence means the SUSTAINED 0.3 s quiet window was reached — a
  // timeout whose final sample merely dips below the threshold is NOT
  // quiescent (that near-miss defeated the gate once in review).
  res.quiescent = reached_quiet_window;
  std::printf("settled in %.1f s (residual %.3f rad/s)%s\n", res.elapsed,
              res.residual, res.quiescent ? "" : " — still moving");
  if (!res.quiescent) {
    // name the offenders so the operator knows what to look at
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (settle.speed(i) >= 0.05) {
        std::printf("  [%d] %-22s residual %.3f rad/s\n", i,
                    model::canonicalJoints()[i].urdf_joint,
                    settle.speed(i));
      }
    }
  }
  return res;
}

// One continuous tracking run: ComputedTorque wrapped with per-joint,
// per-leg statistics, sequence-keyed submission→application latency
// verification, and the full-telemetry CSV. Controller (update) and
// CycleObserver (observe) on ONE object, so the controller state stays
// alive across the turnaround while every CSV row pairs its own
// submission receipt with the pre-write snapshot.
struct TrackingRun : arm::Controller, arm::CycleObserver {
  TrackingRun(model::ChainModel& chain, const model::JointMap& map,
              const model::Trajectory& trajectory, double kp, double kd,
              double leg_split_s, std::FILE* log)
      : inner(chain, map, trajectory, kp, kd),
        trajectory_(trajectory),
        leg_split_s_(leg_split_s),
        log_(log) {}

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    inner.update(state, cmd, t);
    trajectory_.sample(t, q_d.get(), dq_d.get());
    const int leg = t < leg_split_s_ ? 0 : 1;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d[i] - zVecElemNC(state.q.get(), i);
      sum_sq[leg][i] += e * e;
      max_abs[leg][i] = std::max(max_abs[leg][i], std::fabs(e));
    }
    ++samples[leg];
  }

  // Latency verification (docs/REMEDIATION_PLAN.md D3 round 9): match
  // the snapshot's applied sequence to ITS OWN historical receipt —
  // never the receipt created in this row.
  bool observe(double t, const arm::JointState& state,
               const arm::CommandSnapshot& cmds,
               const arm::JointCommand& cmd,
               const arm::CommandReceipt& receipt) override {
    (void)cmd;  // controller telemetry is read from `inner` directly
    const auto& ap = cmds.applied;
    double first_apply_delay = std::nan("");
    if (ap.valid) {
      // a pending sequence OLDER than the applied one was skipped
      for (const auto& [seq, t_sub] : pending_) {
        if (seq < ap.target_seq) {
          std::fprintf(stderr,
                       "LATENCY: submission seq %llu was skipped "
                       "(applied jumped to %llu) — aborting\n",
                       static_cast<unsigned long long>(seq),
                       static_cast<unsigned long long>(ap.target_seq));
          return false;
        }
      }
      const auto it = pending_.find(ap.target_seq);
      if (it != pending_.end()) {
        first_apply_delay = ap.first_time - it->second;
        delay_cache_seq_ = ap.target_seq;
        delay_cache_ = first_apply_delay;
        pending_.erase(it);
        if (first_apply_delay > 2.0 * tuning::kNominalDt + 1e-9) {
          std::fprintf(stderr,
                       "LATENCY: seq %llu first applied %.1f ms after "
                       "submission (bound %.1f ms) — aborting\n",
                       static_cast<unsigned long long>(ap.target_seq),
                       1e3 * first_apply_delay,
                       2e3 * tuning::kNominalDt);
          return false;
        }
      } else if (ap.target_seq == delay_cache_seq_) {
        // completed sequence still current: re-emit the cached delay
        first_apply_delay = delay_cache_;
      }
      // else: pre-run baseline (e.g. the settle phase's last command)
      // — not a tracked submission, delay stays NaN
    }
    // any still-pending receipt past the deadline was never applied
    for (const auto& [seq, t_sub] : pending_) {
      if (state.t - t_sub > 2.0 * tuning::kNominalDt + 1e-9) {
        std::fprintf(stderr,
                     "LATENCY: submission seq %llu unapplied after "
                     "%.1f ms — aborting\n",
                     static_cast<unsigned long long>(seq),
                     1e3 * (state.t - t_sub));
        return false;
      }
    }
    if (receipt.accepted) {
      pending_[receipt.submitted_seq] = receipt.submission_time;
    }
    if (log_ != nullptr) writeRow(t, state, cmds, receipt,
                                  first_apply_delay);
    return true;
  }

  double rms(int leg) const {
    if (!samples[leg]) return 0.0;
    double total = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      total += sum_sq[leg][i];
    }
    return std::sqrt(total / (samples[leg] * model::kCanonicalDof));
  }
  double rms(int leg, int i) const {
    return samples[leg] ? std::sqrt(sum_sq[leg][i] / samples[leg]) : 0.0;
  }

  void report() const {
    for (int leg = 0; leg < 2; ++leg) {
      if (!samples[leg]) continue;
      std::printf("%s RMS: %.4f rad\n", leg == 0 ? "out " : "back",
                  rms(leg));
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        if (max_abs[leg][i] < 1e-4) continue;  // not commanded to move
        std::printf("  [%d] %-22s RMS %.4f  max %.4f rad\n", i,
                    model::canonicalJoints()[i].urdf_joint, rms(leg, i),
                    max_abs[leg][i]);
      }
    }
  }

  arm::ComputedTorque inner;

 private:
  void writeRow(double t, const arm::JointState& state,
                const arm::CommandSnapshot& cmds,
                const arm::CommandReceipt& receipt,
                double first_apply_delay) {
    const auto& ap = cmds.applied;
    const auto& at = cmds.last_attempt;
    const int leg = t < leg_split_s_ ? 0 : 1;
    std::fprintf(log_, "%d,%.4f,%.6f,%llu,%llu,%.6f", leg, t, state.t,
                 static_cast<unsigned long long>(state.seq),
                 static_cast<unsigned long long>(receipt.submitted_seq),
                 receipt.submission_time);
    std::fprintf(log_, ",%d,%llu,%.6f,%llu,%.6f,%llu,%.6f,%.6f",
                 ap.valid ? 1 : 0,
                 static_cast<unsigned long long>(ap.target_seq),
                 ap.first_time,
                 static_cast<unsigned long long>(ap.first_cycle),
                 ap.latest_time,
                 static_cast<unsigned long long>(ap.latest_cycle),
                 first_apply_delay, state.t - ap.latest_time);
    std::fprintf(log_, ",%d,%llu,%.6f,%d", at.valid ? 1 : 0,
                 static_cast<unsigned long long>(at.target_seq), at.time,
                 at.ok ? 1 : 0);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(
          log_, ",%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,"
                "%.4f,%d,%d,%.4f",
          q_d[i], zVecElemNC(state.q.get(), i), dq_d[i],
          zVecElemNC(state.dq.get(), i), inner.velocityEstimate()[i],
          inner.feedforward()[i], inner.pdTerm()[i],
          inner.integralTerm()[i], inner.tauRaw()[i],
          inner.tauCommanded()[i], inner.controllerSaturated(i) ? 1 : 0,
          ap.applied[i], (ap.flags[i] & arm::kCmdClamped) ? 1 : 0,
          (ap.flags[i] & arm::kCmdGated) ? 1 : 0,
          zVecElemNC(state.tau.get(), i));
    }
    std::fprintf(log_, "\n");
  }

  const model::Trajectory& trajectory_;
  double leg_split_s_;
  std::FILE* log_;
  model::ZVector q_d{model::kCanonicalDof};
  model::ZVector dq_d{model::kCanonicalDof};
  double sum_sq[2][model::kCanonicalDof] = {};
  double max_abs[2][model::kCanonicalDof] = {};
  long samples[2] = {};
  std::map<std::uint64_t, double> pending_;  // submitted_seq -> time
  std::uint64_t delay_cache_seq_ = 0;
  double delay_cache_ = std::nan("");
};

// CSV with explicit event semantics (docs/REMEDIATION_PLAN.md D8). A
// row mixes three events documented in the '#' comment line; all
// timestamps share the producer's absolute clock.
inline std::FILE* openCsvLog(const std::string& path) {
  std::FILE* log = std::fopen(path.c_str(), "w");
  if (!log) return nullptr;
  std::fprintf(
      log,
      "# events per row: FEEDBACK (feedback_seq/feedback_time, q, "
      "dq_servo, tau_meas) | THIS CYCLE'S REQUEST (submitted_seq/"
      "submission_time, qd..tau_commanded; applied LATER) | LATEST "
      "APPLIED (applied_*, tau_applied/hsat/gate from the latest "
      "successful transmission; first_apply_* preserved across "
      "retransmissions). first_apply_delay = first_apply_time - "
      "MATCHED submission_time (receipt-map join; NaN for the pre-run "
      "baseline). feedback_minus_latest_apply is SIGNED: the producer "
      "reads before it writes, so the latest apply may post-date this "
      "row's feedback.\n");
  std::fprintf(log,
               "leg,control_t,feedback_time,feedback_seq,submitted_seq,"
               "submission_time,applied_valid,applied_seq,"
               "first_apply_time,first_apply_cycle,latest_apply_time,"
               "latest_apply_cycle,first_apply_delay,"
               "feedback_minus_latest_apply,attempt_valid,attempt_seq,"
               "attempt_time,attempt_ok");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log,
                 ",qd%d,q%d,dqd%d,dq%d,dqest%d,ff%d,pd%d,i%d,tauraw%d,"
                 "taucmd%d,csat%d,tauapp%d,hsat%d,gate%d,taumeas%d",
                 i, i, i, i, i, i, i, i, i, i, i, i, i, i, i);
  }
  std::fprintf(log, "\n");
  return log;
}

}  // namespace x7
