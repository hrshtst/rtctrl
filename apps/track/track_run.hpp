#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <stdexcept>
#include <string>

#include "rtctrl/arm/arm.hpp"
#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/model/ptp_planner.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "track/track_config.hpp"

namespace x7::track {

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

class PlaybackTrajectory : public model::Trajectory {
 public:
  PlaybackTrajectory(const model::Trajectory& source, double rate)
      : source_(source), rate_(rate) {
    if (!std::isfinite(rate) || rate <= 0.0 || rate > 1.0) {
      throw std::invalid_argument("playback rate must be in (0, 1]");
    }
  }

  double duration() const override { return source_.duration() / rate_; }
  int size() const override { return source_.size(); }

  void sample(double t, zVec q, zVec dq = nullptr,
              zVec ddq = nullptr) const override {
    source_.sample(t * rate_, q, dq, ddq);
    if (dq != nullptr) zVecMulDRC(dq, rate_);
    if (ddq != nullptr) zVecMulDRC(ddq, rate_ * rate_);
  }

 private:
  const model::Trajectory& source_;
  double rate_;
};

enum class Phase { Home, HomeSettle, Tracking, FinalHold };

inline const char* phaseName(Phase phase) {
  switch (phase) {
    case Phase::Home: return "home";
    case Phase::HomeSettle: return "home_settle";
    case Phase::Tracking: return "tracking";
    case Phase::FinalHold: return "final_hold";
  }
  return "unknown";
}

enum class RunStatus {
  Success,
  IoFailure,
  HomeNotConverged,
  HardTrackingError,
  PositionGate,
  OperatorTimeout
};

inline const char* statusName(RunStatus status) {
  switch (status) {
    case RunStatus::Success: return "success";
    case RunStatus::IoFailure: return "io_failure";
    case RunStatus::HomeNotConverged: return "home_not_converged";
    case RunStatus::HardTrackingError: return "hard_tracking_error";
    case RunStatus::PositionGate: return "position_gate";
    case RunStatus::OperatorTimeout: return "operator_timeout";
  }
  return "unknown";
}

struct RunResult {
  RunStatus status = RunStatus::IoFailure;
  bool tracking_pass = false;
  bool tracking_warning = false;
  double worst_home_error_rad = 0.0;
  double aggregate_rms_error_rad = 0.0;
  double worst_joint_rms_error_rad = 0.0;
  double peak_error_rad = 0.0;
  std::array<double, model::kCanonicalDof> joint_rms_error_rad{};
  std::array<double, model::kCanonicalDof> joint_peak_error_rad{};
  std::uint64_t cycles = 0;
  std::uint64_t tracking_samples = 0;
};

struct CycleRecord {
  Phase phase;
  double phase_time_s;
  const arm::JointState& state;
  const arm::JointCommand& command;
  const model::ZVector& qref;
  const model::ZVector& dqref;
  const model::ZVector& ddqref;
  const model::ZVector& feedforward;
  const model::ZVector& feedback;
  const arm::CommandSnapshot& snapshot;
  const arm::CommandReceipt& receipt;
};

class CycleSink {
 public:
  virtual ~CycleSink() = default;
  virtual bool record(const CycleRecord& cycle) = 0;
};

class TrackCsvLog : public CycleSink {
 public:
  explicit TrackCsvLog(const std::string& path)
      : file_(std::fopen(path.c_str(), "wx")) {
    if (file_ == nullptr) {
      throw std::runtime_error("track log: cannot create '" + path +
                               "' (it may already exist)");
    }
    std::fprintf(file_,
                 "schema_version,time_s,phase,phase_time_s,command_mode,"
                 "feedback_seq,receipt_accepted,submitted_seq,applied_seq");
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(file_,
                   ",qref%d_rad,dqref%d_rad_s,ddqref%d_rad_s2,q%d_rad,"
                   "dq%d_rad_s,tau_measured%d_nm,tau_ff%d_nm,tau_pd%d_nm,"
                   "tau_command%d_nm,error%d_rad,applied%d,flags%d",
                   i, i, i, i, i, i, i, i, i, i, i, i);
    }
    std::fprintf(file_, "\n");
  }

  ~TrackCsvLog() override {
    if (file_ != nullptr) std::fclose(file_);
  }

  bool record(const CycleRecord& cycle) override {
    const auto& applied = cycle.snapshot.applied;
    std::fprintf(file_, "1,%.9f,%s,%.9f,%u,%llu,%d,%llu,%llu",
                 cycle.state.t, phaseName(cycle.phase), cycle.phase_time_s,
                 static_cast<unsigned>(cycle.command.mode),
                 static_cast<unsigned long long>(cycle.state.seq),
                 cycle.receipt.accepted ? 1 : 0,
                 static_cast<unsigned long long>(cycle.receipt.submitted_seq),
                 static_cast<unsigned long long>(
                     applied.valid ? applied.target_seq : 0));
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double error = cycle.qref[i] - cycle.state.q[i];
      std::fprintf(file_,
                   ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,"
                   "%.9g,%u",
                   cycle.qref[i], cycle.dqref[i], cycle.ddqref[i],
                   cycle.state.q[i], cycle.state.dq[i], cycle.state.tau[i],
                   cycle.feedforward[i], cycle.feedback[i],
                   cycle.command.tau[i], error,
                   applied.valid ? applied.applied[i] : 0.0,
                   applied.valid ? applied.flags[i] : 0);
    }
    std::fprintf(file_, "\n");
    return std::fflush(file_) == 0;
  }

 private:
  std::FILE* file_ = nullptr;
};

inline double homeDuration(const arm::Arm& robot, const HomeConfig& config,
                           const zVec start, const zVec goal) {
  double delta = 0.0;
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    delta = std::max(delta,
                     std::fabs(zVecElemNC(goal, i) - zVecElemNC(start, i)));
  }
  const double by_speed =
      model::ptpPeakSpeedFactor(config.profile,
                                config.trapezoid_acceleration_fraction) *
      delta / config.velocity_limit;
  return std::max({robot.dt(), by_speed, config.motion_time.value_or(0.0)});
}

inline bool submitPosition(arm::Arm& robot, Phase phase, double phase_time,
                           const model::ZVector& qref,
                           const model::ZVector& dqref,
                           const model::ZVector& ddqref, CycleSink* sink,
                           std::uint64_t* cycles) {
  arm::JointState state;
  arm::CommandSnapshot snapshot;
  if (!robot.readState(state, &snapshot)) return false;
  arm::JointCommand command;
  command.mode = arm::ControlMode::Position;
  zVecCopyNC(qref.get(), command.q.get());
  arm::CommandReceipt receipt;
  if (!robot.writeCommand(command, &receipt) || !receipt.accepted) return false;
  model::ZVector zero(model::kCanonicalDof);
  if (sink != nullptr &&
      !sink->record({phase, phase_time, state, command, qref, dqref, ddqref,
                     zero, zero, snapshot, receipt})) {
    return false;
  }
  ++*cycles;
  return robot.step();
}

inline RunResult runPositionHome(arm::Arm& robot,
                                 const model::Trajectory& reference,
                                 const Config& config,
                                 CycleSink* sink = nullptr) {
  RunResult result;
  arm::JointState initial;
  if (!robot.readState(initial)) return result;
  model::ZVector goal(model::kCanonicalDof), zero(model::kCanonicalDof);
  reference.sample(0.0, goal.get());
  const double duration =
      homeDuration(robot, config.home, initial.q.get(), goal.get());
  const long intervals =
      std::max(1L, static_cast<long>(std::ceil(duration / robot.dt())));
  model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
      ddq(model::kCanonicalDof);
  for (long step = 0; step <= intervals; ++step) {
    const double t = std::min(duration, duration * step / intervals);
    const auto progress = model::ptpProgress(
        config.home.profile, t / duration,
        config.home.trapezoid_acceleration_fraction);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double delta = goal[i] - initial.q[i];
      q[i] = initial.q[i] + delta * progress.position;
      dq[i] = delta * progress.velocity / duration;
      ddq[i] = delta * progress.acceleration / (duration * duration);
    }
    if (!submitPosition(robot, Phase::Home, t, q, dq, ddq, sink,
                        &result.cycles)) {
      return result;
    }
  }
  const long settle_cycles =
      static_cast<long>(std::ceil(config.home.settle_time_s / robot.dt()));
  for (long i = 0; i < settle_cycles; ++i) {
    if (!submitPosition(robot, Phase::HomeSettle, i * robot.dt(), goal, zero,
                        zero, sink, &result.cycles)) {
      return result;
    }
  }
  arm::JointState final;
  if (!robot.readState(final)) return result;
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    result.worst_home_error_rad = std::max(
        result.worst_home_error_rad, std::fabs(goal[i] - final.q[i]));
  }
  if (config.home.strict &&
      result.worst_home_error_rad > config.home.tolerance_rad) {
    result.status = RunStatus::HomeNotConverged;
    return result;
  }
  result.status = RunStatus::Success;
  return result;
}

class TrackingRun {
 public:
  TrackingRun(arm::Arm& robot, model::ChainModel& chain,
              const model::JointMap& map, const model::Trajectory& reference,
              const Config& config, bool simulation, CycleSink* sink = nullptr,
              std::function<bool()> enter_pressed = {})
      : robot_(robot),
        reference_(reference),
        config_(config),
        simulation_(simulation),
        sink_(sink),
        enter_pressed_(std::move(enter_pressed)),
        controller_(chain, map, reference, config.kp, config.kd) {}

  RunResult run() {
    RunResult result;
    arm::JointState initial;
    if (!robot_.readState(initial)) return result;
    origin_ = initial.t;
    if (!runTracking(&result)) return result;
    calculateAssessment(&result);
    if (!runFinalHold(&result)) return result;
    result.status = RunStatus::Success;
    return result;
  }

 private:
  bool submit(Phase phase, double phase_time, RunResult* result,
              bool assess, bool* phase_complete = nullptr) {
    arm::JointState state;
    arm::CommandSnapshot snapshot;
    if (!robot_.readState(state, &snapshot)) return false;
    if (phase_complete != nullptr) {
      phase_time = std::max(0.0, state.t - origin_);
      if (phase_time >= reference_.duration() - 1e-9) {
        *phase_complete = true;
        return true;
      }
    }
    arm::JointCommand command;
    controller_.update(state, command, phase_time);
    arm::CommandReceipt receipt;
    if (!robot_.writeCommand(command, &receipt) || !receipt.accepted) {
      return false;
    }
    const auto& qref = controller_.desiredPosition();
    if (sink_ != nullptr &&
        !sink_->record({phase, phase_time, state, command, qref,
                        controller_.desiredVelocity(),
                        controller_.desiredAcceleration(),
                        controller_.feedforward(), controller_.feedback(),
                        snapshot, receipt})) {
      return false;
    }
    ++result->cycles;
    if (assess) {
      double worst = 0.0;
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double error = std::fabs(qref[i] - state.q[i]);
        sum_sq_[i] += error * error;
        result->joint_peak_error_rad[i] =
            std::max(result->joint_peak_error_rad[i], error);
        worst = std::max(worst, error);
      }
      ++result->tracking_samples;
      result->peak_error_rad = std::max(result->peak_error_rad, worst);
      if (worst >= config_.safety.warning_error_rad) {
        result->tracking_warning = true;
      }
      if (worst >= config_.safety.immediate_abort_error_rad) {
        result->status = RunStatus::HardTrackingError;
        return false;
      }
      if (worst >= config_.safety.sustained_abort_error_rad) {
        sustained_error_s_ += robot_.dt();
        if (sustained_error_s_ >= config_.safety.sustained_abort_time_s) {
          result->status = RunStatus::HardTrackingError;
          return false;
        }
      } else {
        sustained_error_s_ = 0.0;
      }
      if (snapshot.applied.valid) {
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          if ((snapshot.applied.flags[i] & arm::kCmdGated) != 0) {
            result->status = RunStatus::PositionGate;
            return false;
          }
        }
      }
    }
    return robot_.step();
  }

  bool runTracking(RunResult* result) {
    const long max_cycles = std::max(
        2L, static_cast<long>(std::ceil(4.0 * reference_.duration() /
                                        robot_.dt())));
    for (long cycle = 0; cycle < max_cycles; ++cycle) {
      bool complete = false;
      if (!submit(Phase::Tracking, 0.0, result, true, &complete)) {
        return false;
      }
      if (complete) return true;
    }
    return false;
  }

  void calculateAssessment(RunResult* result) {
    if (result->tracking_samples == 0) return;
    double total = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      result->joint_rms_error_rad[i] =
          std::sqrt(sum_sq_[i] / result->tracking_samples);
      total += sum_sq_[i];
      result->worst_joint_rms_error_rad = std::max(
          result->worst_joint_rms_error_rad, result->joint_rms_error_rad[i]);
    }
    result->aggregate_rms_error_rad = std::sqrt(
        total / (result->tracking_samples * model::kCanonicalDof));
    result->tracking_pass =
        result->worst_joint_rms_error_rad <=
            config_.assessment.rms_error_rad &&
        result->peak_error_rad <= config_.assessment.peak_error_rad;
  }

  bool runFinalHold(RunResult* result) {
    const double duration = simulation_
                                ? config_.finalization.simulation_hold_time_s
                                : config_.finalization.wait_time_s;
    const long fixed_cycles =
        duration > 0.0
            ? static_cast<long>(std::ceil(duration / robot_.dt()))
            : static_cast<long>(std::ceil(
                  config_.finalization.operator_timeout_s / robot_.dt()));
    for (long cycle = 0; cycle < fixed_cycles; ++cycle) {
      if (!simulation_ && duration == 0.0 && enter_pressed_ &&
          enter_pressed_()) {
        return true;
      }
      if (!submit(Phase::FinalHold, reference_.duration(), result, false)) {
        return false;
      }
    }
    if (!simulation_ && duration == 0.0) {
      result->status = RunStatus::OperatorTimeout;
      return false;
    }
    return true;
  }

  arm::Arm& robot_;
  const model::Trajectory& reference_;
  const Config& config_;
  bool simulation_;
  CycleSink* sink_;
  std::function<bool()> enter_pressed_;
  arm::ComputedTorque controller_;
  std::array<double, model::kCanonicalDof> sum_sq_{};
  double origin_ = 0.0;
  double sustained_error_s_ = 0.0;
};

}  // namespace x7::track
