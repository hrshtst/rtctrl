#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>

#include "follow/follow_config.hpp"
#include "rtctrl/arm/arm.hpp"
#include "rtctrl/model/ptp_planner.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7::follow {

enum class Phase { Home, HomeCorrection, Tracking, Finalizing, AbortHold };

inline const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::Home: return "home";
    case Phase::HomeCorrection: return "home_correction";
    case Phase::Tracking: return "tracking";
    case Phase::Finalizing: return "finalizing";
    case Phase::AbortHold: return "abort_hold";
  }
  return "unknown";
}

enum class RunStatus {
  Success,
  IoFailure,
  HomeNotConverged,
  TrackingError,
  OperatorTimeout
};

inline const char* statusName(RunStatus status) {
  switch (status) {
    case RunStatus::Success: return "success";
    case RunStatus::IoFailure: return "io_failure";
    case RunStatus::HomeNotConverged: return "home_not_converged";
    case RunStatus::TrackingError: return "tracking_error";
    case RunStatus::OperatorTimeout: return "operator_timeout";
  }
  return "unknown";
}

struct RunResult {
  RunStatus status = RunStatus::IoFailure;
  double worst_home_error_rad = 0.0;
  double worst_tracking_error_rad = 0.0;
  int worst_joint = -1;
  bool tracking_warning = false;
  std::uint64_t cycles = 0;
};

struct CycleRecord {
  Phase phase = Phase::Home;
  double time_s = 0.0;
  double phase_time_s = 0.0;
  const arm::JointState* state = nullptr;
  const arm::JointCommand* command = nullptr;
  const model::ZVector* reference_q = nullptr;
  const model::ZVector* reference_dq = nullptr;
  const model::ZVector* reference_ddq = nullptr;
  const arm::CommandSnapshot* snapshot = nullptr;
  const arm::CommandReceipt* receipt = nullptr;
};

class CycleSink {
 public:
  virtual ~CycleSink() = default;
  virtual bool record(const CycleRecord& cycle) = 0;
};

class FollowCsvLog : public CycleSink {
 public:
  explicit FollowCsvLog(const std::string& path)
      : file_(std::fopen(path.c_str(), "wx")) {
    if (file_ == nullptr) {
      throw std::runtime_error("follow log: cannot create '" + path +
                               "' (it may already exist)");
    }
    std::fprintf(file_,
                 "schema_version,time_s,phase,command_mode,phase_time_s,"
                 "feedback_time_s,"
                 "feedback_seq,receipt_accepted,submitted_seq,applied_seq");
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(file_, ",qref%d_rad,dqref%d_rad_s,ddqref%d_rad_s2", i, i,
                   i);
      std::fprintf(file_, ",q%d_rad,dq%d_rad_s,tau%d_nm", i, i, i);
      std::fprintf(file_,
                   ",qcmd%d_rad,dqcmd%d_rad_s,effort_limit%d_nm,error%d_rad,"
                   "applied%d,applied_effort_limit%d_nm,flags%d",
                   i, i, i, i, i, i, i);
    }
    std::fprintf(file_, "\n");
  }

  ~FollowCsvLog() override {
    if (file_ != nullptr) std::fclose(file_);
  }

  bool record(const CycleRecord& cycle) override {
    const auto& state = *cycle.state;
    const auto& cmd = *cycle.command;
    const auto& snapshot = *cycle.snapshot;
    const auto& receipt = *cycle.receipt;
    std::fprintf(file_, "1,%.9f,%s,%u,%.9f,%.9f,%llu,%d,%llu,%llu",
                 cycle.time_s, phaseName(cycle.phase),
                 static_cast<unsigned>(cmd.mode), cycle.phase_time_s, state.t,
                 static_cast<unsigned long long>(state.seq),
                 receipt.accepted ? 1 : 0,
                 static_cast<unsigned long long>(receipt.submitted_seq),
                 static_cast<unsigned long long>(
                     snapshot.applied.valid ? snapshot.applied.target_seq : 0));
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double qref = (*cycle.reference_q)[i];
      std::fprintf(
          file_,
          ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u",
          qref, (*cycle.reference_dq)[i], (*cycle.reference_ddq)[i], state.q[i],
          state.dq[i], state.tau[i], cmd.q[i], cmd.dq[i], cmd.effort_limit[i],
          state.q[i] - qref,
          snapshot.applied.valid ? snapshot.applied.applied[i] : 0.0,
          snapshot.applied.valid ? snapshot.applied.effort_limit[i] : 0.0,
          snapshot.applied.valid ? snapshot.applied.flags[i] : 0);
    }
    std::fprintf(file_, "\n");
    return std::fflush(file_) == 0;
  }

 private:
  std::FILE* file_ = nullptr;
};

class FollowRun {
 public:
  FollowRun(arm::Arm& robot, const model::ZvsTrajectory& reference,
            const Config& config, bool simulation, CycleSink* sink = nullptr,
            std::function<bool()> enter_pressed = {})
      : robot_(robot),
        reference_(reference),
        config_(config),
        simulation_(simulation),
        sink_(sink),
        enter_pressed_(std::move(enter_pressed)) {}

  RunResult run() {
    RunResult result;
    arm::JointState initial;
    if (!robot_.readState(initial)) return result;
    global_origin_ = initial.t;
    storeState(initial);

    model::ZVector home(model::kCanonicalDof);
    reference_.sample(0.0, home.get());
    if (!moveHome(initial.q.get(), home.get(), &result)) {
      result.status = result.status == RunStatus::IoFailure
                          ? RunStatus::IoFailure
                          : RunStatus::HomeNotConverged;
      finalize(Phase::AbortHold, last_state_.q.get(), &result);
      result.cycles = cycles_;
      return result;
    }

    if (!trackReference(&result)) {
      if (result.status == RunStatus::TrackingError) {
        finalize(Phase::AbortHold, last_state_.q.get(), &result);
      }
      result.cycles = cycles_;
      return result;
    }

    model::ZVector final_q(model::kCanonicalDof);
    reference_.sample(reference_.duration(), final_q.get());
    if (!finalize(Phase::Finalizing, final_q.get(), &result)) {
      result.cycles = cycles_;
      return result;
    }
    result.status = RunStatus::Success;
    result.cycles = cycles_;
    return result;
  }

 private:
  void storeState(const arm::JointState& source) {
    zVecCopyNC(source.q.get(), last_state_.q.get());
    zVecCopyNC(source.dq.get(), last_state_.dq.get());
    zVecCopyNC(source.tau.get(), last_state_.tau.get());
    last_state_.t = source.t;
    last_state_.seq = source.seq;
  }

  double homeDuration(const zVec start, const zVec goal) const {
    double delta = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      delta = std::max(delta, std::fabs(zVecElemNC(goal, i) -
                                        zVecElemNC(start, i)));
    }
    const double by_speed =
        model::ptpPeakSpeedFactor(
            config_.home.profile,
            config_.home.trapezoid_acceleration_fraction) *
        delta / config_.home.velocity_limit;
    return std::max({robot_.dt(), by_speed,
                     config_.home.motion_time.value_or(0.0)});
  }

  void fillCommand(const model::ZVector& q, const model::ZVector& dq,
                   arm::JointCommand* command) const {
    command->mode = config_.mode;
    zVecCopyNC(q.get(), command->q.get());
    zVecCopyNC(dq.get(), command->dq.get());
    if (config_.mode == arm::ControlMode::CurrentBasedPosition) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        command->effort_limit[i] = config_.effort_limit_nm[i];
      }
    }
  }

  bool submit(Phase phase, double phase_time, const model::ZVector& qref,
              const model::ZVector& dqref, const model::ZVector& ddqref,
              const model::ZVector* command_q = nullptr,
              const model::ZVector* command_dq = nullptr) {
    arm::JointState state;
    arm::CommandSnapshot snapshot;
    if (!robot_.readState(state, &snapshot)) return false;
    storeState(state);
    arm::JointCommand command;
    fillCommand(command_q != nullptr ? *command_q : qref,
                command_dq != nullptr ? *command_dq : dqref, &command);
    arm::CommandReceipt receipt;
    if (!robot_.writeCommand(command, &receipt)) return false;
    ++cycles_;
    if (sink_ != nullptr &&
        !sink_->record({phase, state.t - global_origin_, phase_time, &state,
                        &command, &qref, &dqref, &ddqref, &snapshot,
                        &receipt})) {
      return false;
    }
    return robot_.step();
  }

  bool runPtp(Phase phase, const zVec start, const zVec goal) {
    const double duration = homeDuration(start, goal);
    const long intervals = std::max(1L, static_cast<long>(
                                           std::ceil(duration / robot_.dt())));
    const double dt = duration / intervals;
    model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
        ddq(model::kCanonicalDof);
    for (long step = 0; step <= intervals; ++step) {
      const double t = std::min(duration, step * dt);
      const auto progress = model::ptpProgress(
          config_.home.profile, t / duration,
          config_.home.trapezoid_acceleration_fraction);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double delta = zVecElemNC(goal, i) - zVecElemNC(start, i);
        q[i] = zVecElemNC(start, i) + delta * progress.position;
        dq[i] = delta * progress.velocity / duration;
        ddq[i] = delta * progress.acceleration / (duration * duration);
      }
      if (!submit(phase, t, q, dq, ddq)) return false;
    }
    return true;
  }

  bool settleHome(const zVec home, const zVec command_goal) {
    model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
        ddq(model::kCanonicalDof), zero(model::kCanonicalDof),
        goal(model::kCanonicalDof);
    zVecCopyNC(home, q.get());
    zVecCopyNC(command_goal, goal.get());
    const long cycles = static_cast<long>(
        std::ceil(config_.home.settle_time_s / robot_.dt()));
    for (long i = 0; i < cycles; ++i) {
      if (!submit(Phase::HomeCorrection, i * robot_.dt(), q, dq, ddq,
                  config_.mode == arm::ControlMode::Velocity ? &q : &goal,
                  &zero)) {
        return false;
      }
    }
    return true;
  }

  double measureError(const zVec target, int* worst_joint) {
    arm::JointState state;
    if (!robot_.readState(state)) return -1.0;
    storeState(state);
    double worst = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double error = std::fabs(zVecElemNC(target, i) - state.q[i]);
      if (error > worst) {
        worst = error;
        *worst_joint = i;
      }
    }
    return worst;
  }

  bool moveHome(const zVec start, const zVec home, RunResult* result) {
    if (!runPtp(Phase::Home, start, home)) return false;
    model::ZVector goal(model::kCanonicalDof), correction_start(model::kCanonicalDof);
    zVecCopyNC(home, goal.get());
    std::array<double, model::kCanonicalDof> offset{};
    for (int attempt = 0; attempt <= config_.home.correction_retries; ++attempt) {
      if (!settleHome(home, goal.get())) return false;
      const double error = measureError(home, &result->worst_joint);
      if (error < 0.0) return false;
      result->worst_home_error_rad = error;
      if (error <= config_.home.tolerance_rad) return true;
      if (attempt == config_.home.correction_retries) break;
      if (config_.mode == arm::ControlMode::Velocity) {
        zVecCopyNC(last_state_.q.get(), correction_start.get());
        if (!runPtp(Phase::HomeCorrection, correction_start.get(), home)) {
          return false;
        }
      } else {
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          const double step = std::clamp(
              zVecElemNC(home, i) - last_state_.q[i], -0.05, 0.05);
          offset[i] = std::clamp(offset[i] + step, -0.15, 0.15);
          goal[i] = zVecElemNC(home, i) + offset[i];
        }
        model::ZVector zero(model::kCanonicalDof), ddq(model::kCanonicalDof),
            qref(model::kCanonicalDof);
        zVecCopyNC(home, qref.get());
        const long cycles = static_cast<long>(
            std::ceil(config_.home.settle_time_s / robot_.dt()));
        for (long i = 0; i < cycles; ++i) {
          if (!submit(Phase::HomeCorrection, i * robot_.dt(), qref, zero,
                      ddq, &goal, &zero)) {
            return false;
          }
        }
      }
    }
    if (!config_.home.strict) {
      std::fprintf(stderr,
                   "warning: home error %.4f rad on joint %d; strict gate "
                   "disabled\n",
                   result->worst_home_error_rad, result->worst_joint);
      return true;
    }
    result->status = RunStatus::HomeNotConverged;
    return false;
  }

  bool trackReference(RunResult* result) {
    const long intervals = std::max(
        1L, static_cast<long>(std::ceil(reference_.duration() / robot_.dt())));
    const double dt = reference_.duration() / intervals;
    double over_limit_time = 0.0;
    model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
        ddq(model::kCanonicalDof);
    for (long step = 0; step <= intervals; ++step) {
      const double t = std::min(reference_.duration(), step * dt);
      reference_.sample(t, q.get(), dq.get(), ddq.get());
      double worst = 0.0;
      int worst_joint = -1;
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double error = std::fabs(last_state_.q[i] - q[i]);
        if (error > worst) {
          worst = error;
          worst_joint = i;
        }
      }
      if (worst > result->worst_tracking_error_rad) {
        result->worst_tracking_error_rad = worst;
        result->worst_joint = worst_joint;
      }
      if (worst >= config_.safety.warning_error_rad &&
          !result->tracking_warning) {
        result->tracking_warning = true;
        std::fprintf(stderr,
                     "warning: tracking error %.4f rad on joint %d\n", worst,
                     worst_joint);
      }
      over_limit_time = worst >= config_.safety.sustained_abort_error_rad
                            ? over_limit_time + robot_.dt()
                            : 0.0;
      if (worst >= config_.safety.immediate_abort_error_rad ||
          over_limit_time >= config_.safety.sustained_abort_time_s) {
        result->status = RunStatus::TrackingError;
        return false;
      }
      if (!submit(Phase::Tracking, t, q, dq, ddq)) return false;
    }
    return true;
  }

  bool finalize(Phase phase, const zVec hold, RunResult* result) {
    model::ZVector q(model::kCanonicalDof), zero(model::kCanonicalDof),
        ddq(model::kCanonicalDof);
    zVecCopyNC(hold, q.get());
    if (simulation_) {
      const double duration = config_.finalization.wait_time_s > 0.0
                                  ? config_.finalization.wait_time_s
                                  : config_.finalization.simulation_hold_time_s;
      const long cycles = static_cast<long>(std::ceil(duration / robot_.dt()));
      for (long i = 0; i < cycles; ++i) {
        if (!submit(phase, i * robot_.dt(), q, zero, ddq, &q, &zero)) {
          return false;
        }
      }
      return true;
    }
    const bool interactive = config_.finalization.wait_time_s == 0.0;
    const double deadline = interactive
                                ? config_.finalization.operator_timeout_s
                                : config_.finalization.wait_time_s;
    const long cycles = static_cast<long>(std::ceil(deadline / robot_.dt()));
    if (interactive) {
      std::fprintf(stderr,
                   "%s: support the arm from below, then "
                   "press Enter to disable torque (timeout %.1f s)\n",
                   phase == Phase::Finalizing ? "tracking complete"
                                              : "run aborted",
                   deadline);
    }
    for (long i = 0; i < cycles; ++i) {
      if (interactive && enter_pressed_ && enter_pressed_()) return true;
      if (!submit(phase, i * robot_.dt(), q, zero, ddq, &q, &zero)) {
        return false;
      }
    }
    if (interactive) {
      result->status = RunStatus::OperatorTimeout;
      return false;
    }
    return true;
  }

  arm::Arm& robot_;
  const model::ZvsTrajectory& reference_;
  const Config& config_;
  bool simulation_;
  CycleSink* sink_;
  std::function<bool()> enter_pressed_;
  arm::JointState last_state_;
  double global_origin_ = 0.0;
  std::uint64_t cycles_ = 0;
};

}  // namespace x7::follow
