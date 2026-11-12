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
#include "rtctrl/learn/command_offset.hpp"
#include "rtctrl/model/ptp_planner.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/util/async_writer.hpp"

namespace x7::follow {

// OffsetRamp runs only with a command offset (setCommandOffset): after
// homing, before tracking.
enum class Phase {
  Home,
  HomeCorrection,
  OffsetRamp,
  Tracking,
  Finalizing,
  AbortHold
};

inline const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::Home: return "home";
    case Phase::HomeCorrection: return "home_correction";
    case Phase::OffsetRamp: return "offset_ramp";
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
  OperatorTimeout,
  HomeRefused,
  PostureDeparture
};

inline const char* statusName(RunStatus status) {
  switch (status) {
    case RunStatus::Success: return "success";
    case RunStatus::IoFailure: return "io_failure";
    case RunStatus::HomeNotConverged: return "home_not_converged";
    case RunStatus::TrackingError: return "tracking_error";
    case RunStatus::OperatorTimeout: return "operator_timeout";
    case RunStatus::HomeRefused: return "home_refused";
    case RunStatus::PostureDeparture: return "posture_departure";
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
  // The header and every row go through an AsyncWriter, so the control
  // loop never blocks on file I/O.
  explicit FollowCsvLog(const std::string& path)
      : writer_(openExclusive(path), rtctrl::util::AsyncWriter::AdoptStream{}) {
    writeHeader();
  }
  // Takes ownership of an already-open stream (tests inject failing ones).
  FollowCsvLog(std::FILE* stream, rtctrl::util::AsyncWriter::AdoptStream)
      : writer_(stream, rtctrl::util::AsyncWriter::AdoptStream{}) {
    writeHeader();
  }

  bool record(const CycleRecord& cycle) override {
    const auto& state = *cycle.state;
    const auto& cmd = *cycle.command;
    const auto& snapshot = *cycle.snapshot;
    const auto& receipt = *cycle.receipt;
    writer_.appendf(
        "1,%.9f,%s,%u,%.9f,%.9f,%llu,%d,%llu,%llu", cycle.time_s,
        phaseName(cycle.phase), static_cast<unsigned>(cmd.mode),
        cycle.phase_time_s, state.t, static_cast<unsigned long long>(state.seq),
        receipt.accepted ? 1 : 0,
        static_cast<unsigned long long>(receipt.submitted_seq),
        static_cast<unsigned long long>(
            snapshot.applied.valid ? snapshot.applied.target_seq : 0));
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double qref = (*cycle.reference_q)[i];
      writer_.appendf(
          ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u",
          qref, (*cycle.reference_dq)[i], (*cycle.reference_ddq)[i], state.q[i],
          state.dq[i], state.tau[i], cmd.q[i], cmd.dq[i], cmd.effort_limit[i],
          state.q[i] - qref,
          snapshot.applied.valid ? snapshot.applied.applied[i] : 0.0,
          snapshot.applied.valid ? snapshot.applied.effort_limit[i] : 0.0,
          snapshot.applied.valid ? snapshot.applied.flags[i] : 0);
    }
    writer_.appendf("\n");
    // False once a row was dropped (full queue) or a queued write failed;
    // a write's failure surfaces here only after that write returns.
    return writer_.commitRow();
  }

  // Writes out the queued rows and closes the file (blocks on I/O: call
  // it after the run). False if any row was dropped or a write failed.
  bool close() { return writer_.close(); }

 private:
  void writeHeader() {
    writer_.appendf(
        "schema_version,time_s,phase,command_mode,phase_time_s,"
        "feedback_time_s,"
        "feedback_seq,receipt_accepted,submitted_seq,applied_seq");
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      writer_.appendf(",qref%d_rad,dqref%d_rad_s,ddqref%d_rad_s2", i, i, i);
      writer_.appendf(",q%d_rad,dq%d_rad_s,tau%d_nm", i, i, i);
      writer_.appendf(
          ",qcmd%d_rad,dqcmd%d_rad_s,effort_limit%d_nm,error%d_rad,"
          "applied%d,applied_effort_limit%d_nm,flags%d",
          i, i, i, i, i, i, i);
    }
    writer_.appendf("\n");
    if (!writer_.commitRow()) {
      throw std::runtime_error("follow log: cannot queue the header");
    }
  }

  static std::FILE* openExclusive(const std::string& path) {
    std::FILE* file = std::fopen(path.c_str(), "wx");
    if (file == nullptr) {
      throw std::runtime_error("follow log: cannot create '" + path +
                               "' (it may already exist)");
    }
    return file;
  }

  rtctrl::util::AsyncWriter writer_;
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

  // Called exactly once, at the first Tracking cycle: after that cycle's
  // feedback is read and before its command is written and stepped. It is
  // not called when the run ends before tracking. Hardware apps leave it
  // unset; simulators use it to start disturbance profiles.
  void setTrackingStartHook(
      std::function<void(const arm::JointState&)> hook) {
    tracking_start_hook_ = std::move(hook);
  }

  // A learned command offset (rtctrl/learn/command_offset.hpp) that the
  // frontend's preflight accepted for this reference (follow_preflight.hpp).
  // With it, the run adds an OffsetRamp phase after homing (from the last
  // homing command to q_d(0) + delta[0] at no more than
  // learning.max_offset_rate_rad_s, then a hold of home.settle_time_s),
  // tracking sample i commands q_d(t_i) + delta[i], and the final hold
  // commands q_d(T) + delta[N-1]. The logged reference and every tracking
  // gate stay on q_d; abort holds carry no offset. Null (the default) runs
  // exactly as without this call. The offset must outlive run(). Throws
  // std::invalid_argument for a posture run or an offset off this run's
  // tracking grid.
  void setCommandOffset(const rtctrl::learn::CommandOffset* offset) {
    if (offset != nullptr) {
      if (config_.posture) {
        throw std::invalid_argument(
            "follow run: a reference built from the measured posture takes "
            "no command offset");
      }
      const auto grid =
          rtctrl::learn::trackingGrid(reference_.duration(), robot_.dt());
      if (offset->delta_rad.size() != grid.samples()) {
        throw std::invalid_argument(
            "follow run: the command offset has " +
            std::to_string(offset->delta_rad.size()) +
            " samples, the tracking grid " + std::to_string(grid.samples()));
      }
    }
    offset_ = offset;
  }

  RunResult run() {
    RunResult result;
    arm::JointState initial;
    if (!robot_.readState(initial)) return result;
    global_origin_ = initial.t;
    storeState(initial);

    model::ZVector home(model::kCanonicalDof);
    reference_.sample(0.0, home.get());
    if (config_.posture) {
      double distance = 0.0;
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        distance = std::max(distance, std::fabs(initial.q[i] - home[i]));
      }
      result.worst_home_error_rad = distance;
      if (!(distance <= config_.posture->start_tolerance_rad)) {
        // Hold where the arm is: no homing motion is ever commanded.
        result.status = RunStatus::HomeRefused;
        finalize(Phase::AbortHold, initial.q.get(), &result);
        result.cycles = cycles_;
        return result;
      }
      // No homing and no correction: tracking starts at the arm's posture.
    } else if (!moveHome(initial.q.get(), home.get(), &result)) {
      result.status = result.status == RunStatus::IoFailure
                          ? RunStatus::IoFailure
                          : RunStatus::HomeNotConverged;
      finalize(Phase::AbortHold, last_state_.q.get(), &result);
      result.cycles = cycles_;
      return result;
    }

    if (offset_ != nullptr && !rampOffset(home)) {
      // An I/O failure ends the run as in homing.
      result.status = RunStatus::IoFailure;
      finalize(Phase::AbortHold, last_state_.q.get(), &result);
      result.cycles = cycles_;
      return result;
    }

    if (!trackReference(&result)) {
      if (result.status == RunStatus::TrackingError) {
        finalize(Phase::AbortHold, last_state_.q.get(), &result);
      } else if (result.status == RunStatus::PostureDeparture) {
        // The hold starts in the cycle whose feedback departed, at that
        // feedback's posture: no reference is commanded from it.
        finalize(Phase::AbortHold, feedback_.state.q.get(), &result,
                 &feedback_);
      }
      result.cycles = cycles_;
      return result;
    }

    model::ZVector final_q(model::kCanonicalDof);
    reference_.sample(reference_.duration(), final_q.get());
    // With an offset the hold keeps its last sample: the learned static
    // compensation stays while the logged reference is q_d(T).
    model::ZVector final_command(model::kCanonicalDof);
    if (offset_ != nullptr) {
      offsetCommand(final_q, offset_->delta_rad.size() - 1, &final_command);
    }
    if (!finalize(Phase::Finalizing, final_q.get(), &result, nullptr,
                  offset_ != nullptr ? final_command.get() : nullptr)) {
      result.cycles = cycles_;
      return result;
    }
    result.status = RunStatus::Success;
    result.cycles = cycles_;
    return result;
  }

 private:
  // One cycle's feedback, read before that cycle's command is written.
  struct Feedback {
    arm::JointState state;
    arm::CommandSnapshot snapshot;
  };

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

  void fillCommand(const model::ZVector& q,
                   arm::JointCommand* command) const {
    command->mode = config_.mode;
    zVecCopyNC(q.get(), command->q.get());
    if (config_.mode == arm::ControlMode::CurrentBasedPosition) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        command->effort_limit[i] = config_.effort_limit_nm[i];
      }
    }
  }

  bool acquire(Phase phase, Feedback* feedback) {
    if (!robot_.readState(feedback->state, &feedback->snapshot)) return false;
    storeState(feedback->state);
    if (phase == Phase::Tracking && !tracking_started_) {
      tracking_started_ = true;
      if (tracking_start_hook_) tracking_start_hook_(feedback->state);
    }
    return true;
  }

  // Writes, records and steps the command of the cycle `feedback` opened.
  bool send(Phase phase, double phase_time, const Feedback& feedback,
            const model::ZVector& qref, const model::ZVector& dqref,
            const model::ZVector& ddqref, const model::ZVector& command_q) {
    arm::JointCommand command;
    fillCommand(command_q, &command);
    arm::CommandReceipt receipt;
    if (!robot_.writeCommand(command, &receipt)) return false;
    zVecCopyNC(command.q.get(), last_command_q_.get());
    ++cycles_;
    if (sink_ != nullptr &&
        !sink_->record({phase, feedback.state.t - global_origin_, phase_time,
                        &feedback.state, &command, &qref, &dqref, &ddqref,
                        &feedback.snapshot, &receipt})) {
      return false;
    }
    return robot_.step();
  }

  bool submit(Phase phase, double phase_time, const model::ZVector& qref,
              const model::ZVector& dqref, const model::ZVector& ddqref,
              const model::ZVector* command_q = nullptr) {
    Feedback feedback;
    if (!acquire(phase, &feedback)) return false;
    return send(phase, phase_time, feedback, qref, dqref, ddqref,
                command_q != nullptr ? *command_q : qref);
  }

  // A posture run's gate, on the feedback just read and before the
  // reference q is commanded from it.
  bool postureDeparted(const arm::JointState& state, const model::ZVector& q,
                       RunResult* result) const {
    double worst = 0.0;
    int worst_joint = -1;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double error = std::fabs(state.q[i] - q[i]);
      if (error > worst) {
        worst = error;
        worst_joint = i;
      }
    }
    if (worst < config_.posture->abort_error_rad) return false;
    if (worst > result->worst_tracking_error_rad) {
      result->worst_tracking_error_rad = worst;
      result->worst_joint = worst_joint;
    }
    std::fprintf(stderr,
                 "posture departure: joint %d is %.4f rad from the "
                 "reference; holding where the arm is\n",
                 worst_joint, worst);
    result->status = RunStatus::PostureDeparture;
    return true;
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
        ddq(model::kCanonicalDof), goal(model::kCanonicalDof);
    zVecCopyNC(home, q.get());
    zVecCopyNC(command_goal, goal.get());
    const long cycles = static_cast<long>(
        std::ceil(config_.home.settle_time_s / robot_.dt()));
    for (long i = 0; i < cycles; ++i) {
      if (!submit(Phase::HomeCorrection, i * robot_.dt(), q, dq, ddq,
                  &goal)) {
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
    model::ZVector goal(model::kCanonicalDof);
    zVecCopyNC(home, goal.get());
    std::array<double, model::kCanonicalDof> offset{};
    for (int attempt = 0; attempt <= config_.home.correction_retries; ++attempt) {
      if (!settleHome(home, goal.get())) return false;
      const double error = measureError(home, &result->worst_joint);
      if (error < 0.0) return false;
      result->worst_home_error_rad = error;
      if (error <= config_.home.tolerance_rad) return true;
      if (attempt == config_.home.correction_retries) break;
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
                    ddq, &goal)) {
          return false;
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

  // q + delta[sample]: the offset command at a tracking-grid sample.
  void offsetCommand(const model::ZVector& q, std::size_t sample,
                     model::ZVector* command) const {
    const auto& delta = offset_->delta_rad[sample];
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      (*command)[i] = q[i] + delta[i];
    }
  }

  // From the last command sent (the homing goal, any home correction
  // included) to q_d(0) + delta[0], the first tracking command: `steps`
  // equal increments of at most max_offset_rate_rad_s * dt on every joint,
  // the last one landing exactly on the target, then a hold there for
  // home.settle_time_s. The logged reference is q_d(0) at rest. As in
  // homing, no tracking gate is evaluated.
  bool rampOffset(const model::ZVector& home) {
    model::ZVector start(model::kCanonicalDof), target(model::kCanonicalDof),
        q(model::kCanonicalDof), zero(model::kCanonicalDof),
        ddq(model::kCanonicalDof);
    zVecCopyNC(last_command_q_.get(), start.get());
    offsetCommand(home, 0, &target);
    double widest = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      widest = std::max(widest, std::fabs(target[i] - start[i]));
    }
    const double dt = robot_.dt();
    const long steps = static_cast<long>(
        std::ceil(widest / config_.learning.max_offset_rate_rad_s / dt));
    for (long k = 1; k <= steps; ++k) {
      if (k == steps) {
        zVecCopyNC(target.get(), q.get());
      } else {
        const double fraction =
            static_cast<double>(k) / static_cast<double>(steps);
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          q[i] = start[i] + (target[i] - start[i]) * fraction;
        }
      }
      if (!submit(Phase::OffsetRamp, (k - 1) * dt, home, zero, ddq, &q)) {
        return false;
      }
    }
    const long hold =
        static_cast<long>(std::ceil(config_.home.settle_time_s / dt));
    for (long i = 0; i < hold; ++i) {
      if (!submit(Phase::OffsetRamp, (steps + i) * dt, home, zero, ddq,
                  &target)) {
        return false;
      }
    }
    return true;
  }

  bool trackReference(RunResult* result) {
    // One definition of the grid for the follower and the offset.
    const auto grid =
        rtctrl::learn::trackingGrid(reference_.duration(), robot_.dt());
    const long intervals = grid.intervals;
    const double dt = grid.dt_s;
    double over_limit_time = 0.0;
    model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
        ddq(model::kCanonicalDof), command(model::kCanonicalDof);
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
      if (config_.posture) {
        if (!acquire(Phase::Tracking, &feedback_)) return false;
        if (postureDeparted(feedback_.state, q, result)) return false;
        if (!send(Phase::Tracking, t, feedback_, q, dq, ddq, q)) return false;
      } else if (offset_ != nullptr) {
        // The gates above judged q_d; only the command carries the offset.
        offsetCommand(q, static_cast<std::size_t>(step), &command);
        if (!submit(Phase::Tracking, t, q, dq, ddq, &command)) return false;
      } else if (!submit(Phase::Tracking, t, q, dq, ddq)) {
        return false;
      }
    }
    return true;
  }

  // `first`, when given, is the feedback of the hold's first cycle,
  // already read: that cycle is answered with the hold before anything
  // else, operator input included. `command_hold`, when given, is
  // commanded instead of `hold`, which stays the logged reference.
  bool finalize(Phase phase, const zVec hold, RunResult* result,
                const Feedback* first = nullptr,
                const zVec command_hold = nullptr) {
    model::ZVector q(model::kCanonicalDof), zero(model::kCanonicalDof),
        ddq(model::kCanonicalDof), command(model::kCanonicalDof);
    zVecCopyNC(hold, q.get());
    zVecCopyNC(command_hold != nullptr ? command_hold : hold, command.get());
    long start = 0;
    if (first != nullptr) {
      if (!send(phase, 0.0, *first, q, zero, ddq, command)) return false;
      start = 1;
    }
    if (simulation_) {
      const double duration = config_.finalization.wait_time_s > 0.0
                                  ? config_.finalization.wait_time_s
                                  : config_.finalization.simulation_hold_time_s;
      const long cycles = static_cast<long>(std::ceil(duration / robot_.dt()));
      for (long i = start; i < cycles; ++i) {
        if (!submit(phase, i * robot_.dt(), q, zero, ddq, &command)) {
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
    for (long i = start; i < cycles; ++i) {
      if (interactive && enter_pressed_ && enter_pressed_()) return true;
      if (!submit(phase, i * robot_.dt(), q, zero, ddq, &command)) {
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
  std::function<void(const arm::JointState&)> tracking_start_hook_;
  bool tracking_started_ = false;
  arm::JointState last_state_;
  Feedback feedback_;  // a posture run's current cycle
  const rtctrl::learn::CommandOffset* offset_ = nullptr;
  model::ZVector last_command_q_{model::kCanonicalDof};  // of the last write
  double global_origin_ = 0.0;
  std::uint64_t cycles_ = 0;
};

}  // namespace x7::follow
