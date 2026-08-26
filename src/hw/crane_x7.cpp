#include "rtctrl/hw/crane_x7.hpp"

#include <algorithm>
#include <cmath>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/hw/cycle_timing.hpp"

namespace rtctrl::hw {

namespace reg = rtctrl::dxl::reg;

CraneX7::CraneX7(dxl::PacketIO& io, Config config)
    : CraneX7(io, std::move(config), Options()) {}

CraneX7::CraneX7(dxl::PacketIO& io, Config config, Options options)
    : io_(io),
      config_(std::move(config)),
      options_(options),
      group_(io, [this] {
        std::vector<std::uint8_t> ids;
        for (const auto& joint : config_.joints) ids.push_back(joint.id);
        return ids;
      }()),
      now_([] {
        return std::chrono::duration<double>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
      }),
      last_command_(now_()) {
  config_.validate();  // Config is plain data — re-check the invariant
}

CraneX7::~CraneX7() { stopThread(); }

std::vector<std::uint8_t> CraneX7::ids() const {
  std::vector<std::uint8_t> out;
  for (const auto& joint : config_.joints) out.push_back(joint.id);
  return out;
}

bool CraneX7::verifyServos() {
  for (const auto& joint : config_.joints) {
    std::uint16_t model = 0;
    const auto ping = io_.ping(joint.id, &model);
    if (!ping.ok()) {
      last_error_ = "no response from id " + std::to_string(joint.id);
      return false;
    }
    if (model != joint.model_number) {
      last_error_ = "id " + std::to_string(joint.id) + " reports model " +
                    std::to_string(model) + ", config expects " +
                    std::to_string(joint.model_number);
      return false;
    }
    std::uint8_t firmware = 0;
    if (!io_.read8(joint.id, reg::kFirmwareVersion.addr, &firmware).ok()) {
      last_error_ = "cannot read firmware of id " + std::to_string(joint.id);
      return false;
    }
    if (firmware < dxl::kMinFirmwareBusWatchdog) {
      last_error_ = "id " + std::to_string(joint.id) + " firmware v" +
                    std::to_string(firmware) +
                    " lacks Bus Watchdog (needs >= v38)";
      return false;
    }
  }
  return true;
}

bool CraneX7::activate() {
  if (activated_) return true;
  const bool current_based_position = std::all_of(
      config_.joints.begin(), config_.joints.end(),
      [](const auto& joint) { return joint.operating_mode == 5; });
  if (current_based_position) {
    if (cbp_activation_limits_amps_.size() != config_.joints.size() ||
        std::any_of(cbp_activation_limits_amps_.begin(),
                    cbp_activation_limits_amps_.end(), [](double value) {
                      return !std::isfinite(value) || value <= 0.0;
                    })) {
      last_error_ = "current-based position activation requires one finite "
                    "positive current ceiling per joint";
      return false;
    }
  }
  escalated_ = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    submission_armed_ = false;  // fresh session for the deadman
  }
  if (!activateSteps()) {
    // A mid-sequence failure must not leave a partially torqued arm:
    // best-effort release of everything the sequence may have touched
    // (preserves last_error_ from the failing step).
    bestEffortRelease();
    return false;
  }
  last_command_ = now_();
  activated_ = true;
  return true;
}

void CraneX7::bestEffortRelease() {
  for (const auto& joint : config_.joints) {
    io_.write16(joint.id, reg::kGoalCurrent.addr, 0);
    io_.write8(joint.id, reg::kTorqueEnable.addr, 0);
    io_.write8(joint.id, reg::kBusWatchdog.addr, 0);
  }
}

bool CraneX7::activateSteps() {
  if (!verifyServos()) return false;

  // EEPROM-side setup requires torque off; a previously triggered Bus
  // Watchdog (reads 0xFF, rejects goal writes) must be cleared before
  // the goal snap below, so write 0 first — verified on hardware after
  // a real USB-pull trip.
  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kTorqueEnable.addr, 0).ok()) {
      last_error_ = "torque-off failed on id " + std::to_string(joint.id);
      return false;
    }
    if (!io_.write8(joint.id, reg::kBusWatchdog.addr, 0).ok()) {
      last_error_ = "bus-watchdog clear failed on id " +
                    std::to_string(joint.id);
      return false;
    }
  }
  if (!group_.setupIndirect().ok()) {
    last_error_ = "indirect map setup failed";
    return false;
  }
  limit_lo_.clear();
  limit_hi_.clear();
  servo_current_limit_amps_.clear();
  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kOperatingMode.addr,
                    joint.operating_mode).ok()) {
      last_error_ = "operating-mode write failed on id " +
                    std::to_string(joint.id);
      return false;
    }
  }

  // Preserve the established default, then let an explicitly loaded motor
  // parameter set override it while torque is still disabled. Operating Mode
  // must precede both because the servo resets gains and profiles on a mode
  // write.
  if (!writePositionPGain(options_.active_p_gain)) return false;
  if (options_.activation_configurator) {
    std::string error;
    if (!options_.activation_configurator(io_, &error)) {
      last_error_ = error.empty() ? "activation parameter setup failed"
                                  : error;
      return false;
    }
  }

  for (const auto& joint : config_.joints) {
    std::uint8_t torque = 1;
    if (!io_.read8(joint.id, reg::kTorqueEnable.addr, &torque).ok() ||
        torque != 0) {
      last_error_ = "activation configurator left torque enabled on id " +
                    std::to_string(joint.id);
      return false;
    }
    std::uint32_t lo = 0, hi = 0;
    if (!io_.read32(joint.id, reg::kMinPositionLimit.addr, &lo).ok() ||
        !io_.read32(joint.id, reg::kMaxPositionLimit.addr, &hi).ok()) {
      last_error_ = "limit read failed on id " + std::to_string(joint.id);
      return false;
    }
    limit_lo_.push_back(dxl::pulseToRad(static_cast<std::int32_t>(lo)) +
                        joint.pos_limit_margin);
    limit_hi_.push_back(dxl::pulseToRad(static_cast<std::int32_t>(hi)) -
                        joint.pos_limit_margin);
    std::uint16_t current_limit_raw = 0;
    if (!io_.read16(joint.id, reg::kCurrentLimit.addr,
                    &current_limit_raw).ok()) {
      last_error_ = "current-limit read failed on id " +
                    std::to_string(joint.id);
      return false;
    }
    servo_current_limit_amps_.push_back(
        dxl::currentToAmps(static_cast<std::int16_t>(current_limit_raw)));
  }

  // Snap goals to the present posture so torque-on commands no motion.
  // This HOLDS the arm only in position mode: current-mode goals are
  // zero (or the staged preload) — the arm is otherwise unsupported
  // until the first controller command.
  std::vector<dxl::Feedback> present;
  if (!readAll(present)) {
    last_error_ = "present-state read failed";
    return false;
  }
  std::vector<double> zeros(config_.joints.size(), 0.0);
  std::vector<double> positions(config_.joints.size());
  for (std::size_t i = 0; i < present.size(); ++i) {
    positions[i] = present[i].position;
  }
  std::vector<double> activation_currents = zeros;
  if (config_.joints.front().operating_mode == 5) {
    std::vector<std::uint8_t> flags;
    limitCurrentMagnitudes(cbp_activation_limits_amps_,
                           &activation_currents, &flags);
    for (std::size_t i = 0; i < flags.size(); ++i) {
      if (flags[i] != 0) {
        last_error_ = "current-based position activation ceiling clipped "
                      "on joint " +
                      std::to_string(i) + " — refusing";
        return false;
      }
    }
  }
  if (!group_.writeGoals(activation_currents, zeros, positions).ok()) {
    last_error_ = "goal snap failed";
    return false;
  }

  const auto watchdog_units = static_cast<std::uint8_t>(std::clamp(
      std::lround(options_.bus_watchdog_timeout_s /
                  dxl::kBusWatchdogUnitSeconds),
      1L, 127L));
  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kBusWatchdog.addr, watchdog_units).ok()) {
      last_error_ = "bus-watchdog arm failed on id " +
                    std::to_string(joint.id);
      return false;
    }
  }

  // Current-mode activation preload: written while torque is still off
  // (the register accepts it and it takes effect at the enable
  // instant), through the SAME limiter as writeCurrents, against the
  // present-state read above. A failed preload fails the activation —
  // the caller's rollback releases everything.
  if (!preload_amps_.empty()) {
    for (const auto& joint : config_.joints) {
      if (joint.operating_mode != 0) {  // EVERY joint, not the first
        last_error_ = "activation preload requires current mode on all "
                      "joints (id " + std::to_string(joint.id) + ")";
        return false;
      }
    }
    if (preload_amps_.size() != config_.joints.size()) {
      last_error_ = "activation preload size mismatch";
      return false;
    }
    std::vector<double> limited;
    std::vector<std::uint8_t> flags;
    limitCurrents(preload_amps_, present, &limited, &flags);
    for (std::size_t i = 0; i < flags.size(); ++i) {
      if (flags[i] != 0) {  // a limited gravity preload cannot hold
        last_error_ = "activation preload clipped/gated on joint " +
                      std::to_string(i) + " — refusing";
        return false;
      }
    }
    if (!group_.writeGoalCurrents(limited).ok()) {
      last_error_ = "activation preload write failed";
      return false;
    }
  }

  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kTorqueEnable.addr, 1).ok()) {
      last_error_ = "torque-on failed on id " + std::to_string(joint.id);
      return false;
    }
  }
  return true;
}

bool CraneX7::deactivate() {
  stopThread();
  preload_amps_.clear();
  cbp_activation_limits_amps_.clear();
  // Once quiesced, the deadline watchdog has silenced the bus so the
  // servo Bus Watchdog can stop the servos — ANY further instruction
  // packet here (including these stop/torque-off writes) would feed
  // that watchdog and defeat the stop. The flag is re-checked between
  // transactions so a quiesce landing mid-sequence suppresses the
  // remainder; the quiesced cleanup only marks the session inactive.
  bool ok = true;
  // zero goal currents (relevant in current mode; harmless otherwise)
  std::vector<bool> zeroed(config_.joints.size(), false);
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (quiesced_.load()) break;
    zeroed[i] =
        io_.write16(config_.joints[i].id, reg::kGoalCurrent.addr, 0).ok();
    ok &= zeroed[i];
  }
  for (std::size_t i = 0; i < config_.joints.size(); ++i) {
    if (quiesced_.load()) break;
    const bool off =
        io_.write8(config_.joints[i].id, reg::kTorqueEnable.addr, 0).ok();
    // The firmware Bus Watchdog is a servo's LAST protection: disarm
    // it only after a CONFIRMED zero + torque-off. A joint whose stop
    // writes failed must keep its watchdog armed, so the caller's
    // escalation (bus silence) still stops it — disarming first could
    // leave a torqued servo with no watchdog (review finding).
    if (!(zeroed[i] && off)) {
      ok = false;
      continue;
    }
    if (quiesced_.load()) break;
    ok &= io_.write8(config_.joints[i].id, reg::kBusWatchdog.addr, 0).ok();
  }
  activated_ = false;
  return ok && !quiesced_.load();
}

bool CraneX7::readAll(std::vector<dxl::Feedback>& out) {
  if (!group_.readAll(out).ok()) return false;
  // In current/velocity modes the servo reports MULTI-TURN position, so
  // hand-moving a limp joint across the encoder boundary leaves a
  // +/-2pi offset in every later reading (observed 2026-07-21: the
  // twist read +6.54 rad after repositioning, which made its soft
  // position limit gate block every positive current for a whole run).
  // All CRANE-X7 joint ranges fit inside one turn, so the principal
  // angle is the physical truth.
  for (auto& fb : out) {
    fb.position = std::remainder(fb.position, 2.0 * M_PI);
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  feedback_ = out;
  feedback_time_ = now_();
  ++feedback_seq_;
  return true;
}

std::vector<dxl::Feedback> CraneX7::lastFeedback() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return feedback_;
}

CraneX7::StampedFeedback CraneX7::lastFeedbackStamped(
    arm::CommandSnapshot* cmds) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (cmds != nullptr) {
    cmds->applied = applied_rec_;
    cmds->last_attempt = attempt_rec_;
  }
  return {feedback_, feedback_time_, feedback_seq_};
}

CraneX7::StampedFeedback CraneX7::waitFeedbackStamped(
    std::uint64_t last_seen, arm::CommandSnapshot* cmds) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (!thread_run_.load() || quiesced_.load()) return {};
  feedback_cv_.wait(lock, [this, last_seen] {
    const bool window_ready =
        !synchronize_commands_ ||
        (command_window_open_ &&
         command_window_feedback_seq_ == feedback_seq_);
    return (feedback_seq_ > last_seen && window_ready) ||
           !thread_run_.load() || quiesced_.load();
  });
  if (!thread_run_.load() || quiesced_.load() ||
      feedback_seq_ <= last_seen) {
    return {};
  }
  if (cmds != nullptr) {
    cmds->applied = applied_rec_;
    cmds->last_attempt = attempt_rec_;
  }
  return {feedback_, feedback_time_, feedback_seq_};
}

bool CraneX7::requireMode(std::uint8_t mode, const char* what) {
  for (const auto& joint : config_.joints) {
    if (joint.operating_mode != mode) {
      last_error_ = std::string(what) + " rejected: joint '" + joint.name +
                    "' is configured for operating mode " +
                    std::to_string(joint.operating_mode);
      return false;
    }
  }
  return true;
}

bool CraneX7::requireSize(std::size_t n, const char* what) {
  if (n == config_.joints.size()) return true;
  last_error_ = std::string(what) + " rejected: got " + std::to_string(n) +
                " values for " + std::to_string(config_.joints.size()) +
                " joints";
  return false;
}

bool CraneX7::requireActive(const char* what) {
  if (activated_) return true;
  // The limit arrays are read from the servos during activation — a
  // pre-activation command would index them empty.
  last_error_ = std::string(what) + " rejected: not activated";
  return false;
}

bool CraneX7::writePositions(const std::vector<double>& rad,
                             WriteOutcome* out) {
  if (escalated_) return false;
  if (!requireActive("position command")) return false;
  if (!requireMode(3, "position command")) return false;
  if (!requireSize(rad.size(), "position command")) return false;
  std::vector<double> local_values;
  std::vector<std::uint8_t> local_flags;
  auto& clamped = out != nullptr ? out->values : local_values;
  auto& flags = out != nullptr ? out->flags : local_flags;
  clamped.resize(rad.size());
  flags.assign(rad.size(), 0);
  for (std::size_t i = 0; i < rad.size(); ++i) {
    clamped[i] = std::clamp(rad[i], limit_lo_[i], limit_hi_[i]);
    if (clamped[i] != rad[i]) flags[i] |= arm::kCmdClamped;
  }
  if (!group_.writeGoalPositions(clamped).ok()) return false;
  last_command_ = now_();
  return true;
}

bool CraneX7::writeVelocities(const std::vector<double>& rad_s,
                              WriteOutcome* out) {
  return writeVelocitiesWithFeedback(rad_s, lastFeedback(), out);
}

bool CraneX7::writeVelocitiesWithFeedback(
    const std::vector<double>& rad_s,
    const std::vector<dxl::Feedback>& fb, WriteOutcome* out) {
  if (escalated_) return false;
  if (!requireActive("velocity command")) return false;
  if (!requireMode(1, "velocity command")) return false;
  if (!requireSize(rad_s.size(), "velocity command")) return false;
  if (fb.size() != config_.joints.size()) {
    last_error_ = "velocity command rejected: no position feedback yet "
                  "(software limits need it)";
    return false;
  }
  std::vector<double> local_values;
  std::vector<std::uint8_t> local_flags;
  auto& limited = out != nullptr ? out->values : local_values;
  auto& flags = out != nullptr ? out->flags : local_flags;
  limited.resize(rad_s.size());
  flags.assign(rad_s.size(), 0);
  for (std::size_t i = 0; i < rad_s.size(); ++i) {
    const double vmax = config_.joints[i].velocity_limit;
    double v = std::clamp(rad_s[i], -vmax, vmax);
    if (v != rad_s[i]) flags[i] |= arm::kCmdClamped;
    // On-servo position limits are inactive outside position mode:
    // zero any command that drives a joint past a limit.
    if ((fb[i].position >= limit_hi_[i] && v > 0.0) ||
        (fb[i].position <= limit_lo_[i] && v < 0.0)) {
      v = 0.0;
      flags[i] |= arm::kCmdGated;
    }
    limited[i] = v;
  }
  if (!group_.writeGoalVelocities(limited).ok()) return false;
  last_command_ = now_();
  return true;
}

bool CraneX7::writeCurrents(const std::vector<double>& amps,
                            WriteOutcome* out) {
  return writeCurrentsWithFeedback(amps, lastFeedback(), out);
}

bool CraneX7::writeCurrentsWithFeedback(
    const std::vector<double>& amps, const std::vector<dxl::Feedback>& fb,
    WriteOutcome* out) {
  if (escalated_) return false;
  if (!requireActive("current command")) return false;
  if (!requireMode(0, "current command")) return false;
  if (!requireSize(amps.size(), "current command")) return false;
  if (fb.size() != config_.joints.size()) {
    last_error_ = "current command rejected: no position feedback yet "
                  "(software limits need it)";
    return false;
  }
  std::vector<double> local_values;
  std::vector<std::uint8_t> local_flags;
  auto* limited = out != nullptr ? &out->values : &local_values;
  auto* flags = out != nullptr ? &out->flags : &local_flags;
  limitCurrents(amps, fb, limited, flags);
  if (!group_.writeGoalCurrents(*limited).ok()) return false;
  last_command_ = now_();
  return true;
}

bool CraneX7::writeCurrentBasedPositions(
    const std::vector<double>& rad,
    const std::vector<double>& current_limit_amps, WriteOutcome* out) {
  return writeCurrentBasedPositionsWithFeedback(
      rad, current_limit_amps, lastFeedback(), out);
}

bool CraneX7::writeCurrentBasedPositionsWithFeedback(
    const std::vector<double>& rad,
    const std::vector<double>& current_limit_amps,
    const std::vector<dxl::Feedback>& feedback, WriteOutcome* out) {
  (void)feedback;
  if (escalated_) return false;
  if (!requireActive("current-based position command")) return false;
  if (!requireMode(5, "current-based position command")) return false;
  if (!requireSize(rad.size(), "current-based position command") ||
      !requireSize(current_limit_amps.size(),
                   "current-based position effort ceiling")) {
    return false;
  }
  std::vector<double> local_positions;
  std::vector<double> local_currents;
  std::vector<std::uint8_t> local_flags;
  auto& positions = out != nullptr ? out->values : local_positions;
  auto& currents = out != nullptr ? out->auxiliary : local_currents;
  auto& flags = out != nullptr ? out->flags : local_flags;
  positions.resize(rad.size());
  flags.assign(rad.size(), 0);
  for (std::size_t i = 0; i < rad.size(); ++i) {
    positions[i] = std::clamp(rad[i], limit_lo_[i], limit_hi_[i]);
    if (positions[i] != rad[i]) flags[i] |= arm::kCmdClamped;
  }
  std::vector<std::uint8_t> current_flags;
  limitCurrentMagnitudes(current_limit_amps, &currents, &current_flags);
  for (std::size_t i = 0; i < flags.size(); ++i) flags[i] |= current_flags[i];
  const std::vector<double> zeros(rad.size(), 0.0);
  if (!group_.writeGoals(currents, zeros, positions).ok()) return false;
  last_command_ = now_();
  return true;
}

bool CraneX7::setTargetPositions(const std::vector<double>& rad,
                                 std::uint64_t* seq, double* time) {
  return setTargets(rad, nullptr, 0, 0.0, false, seq, time);
}

bool CraneX7::setTargets(const std::vector<double>& values,
                         const std::vector<double>* auxiliary,
                         std::uint64_t source_feedback_seq,
                         double source_feedback_time, bool tagged,
                         std::uint64_t* seq, double* time) {
  if (!requireSize(values.size(), "target submission")) return false;
  if (auxiliary != nullptr &&
      !requireSize(auxiliary->size(), "auxiliary target submission")) {
    return false;
  }
  const double submit_time = now_();
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (tagged) {
    const double age = submit_time - source_feedback_time;
    const double deadline = options_.controller_deadline_s > 0.0
                                ? options_.controller_deadline_s
                                : options_.control_cycle_s;
    {
      std::lock_guard<std::mutex> cycle_lock(cycle_mutex_);
      if (std::isfinite(age) && age >= 0.0) {
        stats_.max_feedback_age_at_submission_s =
            std::max(stats_.max_feedback_age_at_submission_s, age);
      }
    }
    if (source_feedback_seq == 0 ||
        source_feedback_seq != feedback_seq_ || !std::isfinite(age) ||
        age < 0.0 || age > deadline ||
        (synchronize_commands_ &&
         (!command_window_open_ || source_feedback_seq !=
                                      command_window_feedback_seq_))) {
      // Arm the submission deadman even for the first rejected attempt:
      // a caller that ignores false cannot leave the default target flowing
      // indefinitely while being exempt as a monitor-only session.
      submission_armed_ = true;
      last_submission_ = submit_time;
      {
        std::lock_guard<std::mutex> cycle_lock(cycle_mutex_);
        ++stats_.stale_submissions;
      }
      last_error_ = "target submission rejected: source feedback is stale";
      if (seq != nullptr) *seq = 0;
      if (time != nullptr) *time = submit_time;
      return false;
    }
  }
  targets_ = values;
  auxiliary_targets_ = auxiliary != nullptr ? *auxiliary
                                             : std::vector<double>{};
  have_targets_ = true;
  // Submission freshness feeds the deadman: the background thread's own
  // retransmissions refresh last_command_, so without this a frozen
  // CONTROLLER would leave the last command active forever while both
  // watchdog layers stay fed.
  last_submission_ = submit_time;
  submission_armed_ = true;
  // Sequence + timestamp stored atomically with the targets: the
  // thread's write attempts carry this sequence, so requested-to-
  // applied causality stays unambiguous.
  ++target_seq_;
  target_submit_time_ = last_submission_;
  target_source_feedback_seq_ = tagged ? source_feedback_seq : 0;
  target_source_feedback_time_ = tagged ? source_feedback_time : 0.0;
  if (seq != nullptr) *seq = target_seq_;
  if (time != nullptr) *time = target_submit_time_;
  lock.unlock();
  target_cv_.notify_all();
  return true;
}
bool CraneX7::setTargetVelocities(const std::vector<double>& rad_s,
                                  std::uint64_t* seq, double* time) {
  return setTargetPositions(rad_s, seq, time);  // same storage; units
                                                // follow the mode
}
bool CraneX7::setTargetCurrents(const std::vector<double>& amps,
                                std::uint64_t* seq, double* time) {
  return setTargetPositions(amps, seq, time);
}

bool CraneX7::setTargetCurrentBasedPositions(
    const std::vector<double>& rad,
    const std::vector<double>& current_limit_amps, std::uint64_t* seq,
    double* time) {
  return setTargets(rad, &current_limit_amps, 0, 0.0, false, seq, time);
}

bool CraneX7::setTargetPositionsFromFeedback(
    const std::vector<double>& rad, std::uint64_t source_feedback_seq,
    double source_feedback_time, std::uint64_t* seq, double* time) {
  return setTargets(rad, nullptr, source_feedback_seq, source_feedback_time,
                    true, seq, time);
}

bool CraneX7::setTargetVelocitiesFromFeedback(
    const std::vector<double>& rad_s, std::uint64_t source_feedback_seq,
    double source_feedback_time, std::uint64_t* seq, double* time) {
  return setTargets(rad_s, nullptr, source_feedback_seq, source_feedback_time,
                    true, seq, time);
}

bool CraneX7::setTargetCurrentsFromFeedback(
    const std::vector<double>& amps, std::uint64_t source_feedback_seq,
    double source_feedback_time, std::uint64_t* seq, double* time) {
  return setTargets(amps, nullptr, source_feedback_seq, source_feedback_time,
                    true, seq, time);
}

bool CraneX7::setTargetCurrentBasedPositionsFromFeedback(
    const std::vector<double>& rad,
    const std::vector<double>& current_limit_amps,
    std::uint64_t source_feedback_seq, double source_feedback_time,
    std::uint64_t* seq, double* time) {
  return setTargets(rad, &current_limit_amps, source_feedback_seq,
                    source_feedback_time, true, seq, time);
}

bool CraneX7::startThread(bool synchronize_commands) {
  if (thread_.joinable()) return true;
  if (!activated_) {
    last_error_ = "startThread: activate first";
    return false;
  }
  if (!std::isfinite(options_.control_cycle_s) ||
      options_.control_cycle_s <= 0.0) {
    last_error_ = "startThread: control cycle must be finite and positive";
    return false;
  }
  if (!std::isfinite(options_.controller_deadline_s) ||
      options_.controller_deadline_s < 0.0) {
    last_error_ =
        "startThread: controller deadline must be finite and nonnegative";
    return false;
  }
  if (synchronize_commands &&
      (!std::isfinite(options_.controller_write_margin_s) ||
       options_.controller_write_margin_s < 0.0 ||
       options_.controller_write_margin_s >= options_.control_cycle_s)) {
    last_error_ = "startThread: controller write margin must be finite, "
                  "nonnegative, and shorter than the control cycle";
    return false;
  }
  const auto mode = config_.joints.front().operating_mode;
  for (const auto& joint : config_.joints) {
    if (joint.operating_mode != mode) {
      last_error_ = "startThread: mixed operating modes in the group";
      return false;
    }
  }
  {
    // default target: hold the present state (positions) / stay still
    std::lock_guard<std::mutex> lock(state_mutex_);
    synchronize_commands_ = synchronize_commands;
    command_window_open_ = false;
    command_window_feedback_seq_ = 0;
    if (!have_targets_) {
      targets_.assign(config_.joints.size(), 0.0);
      if (mode == 3) {
        for (std::size_t i = 0; i < feedback_.size(); ++i) {
          targets_[i] = feedback_[i].position;
        }
      } else if (mode == 0 && !preload_amps_.empty()) {
        // keep the activation preload flowing (the writer re-clamps
        // every cycle) until the first controller submission
        targets_ = preload_amps_;
      } else if (mode == 5) {
        for (std::size_t i = 0; i < feedback_.size(); ++i) {
          targets_[i] = feedback_[i].position;
        }
        auxiliary_targets_ = cbp_activation_limits_amps_;
      }
      have_targets_ = true;
    }
  }
  thread_run_.store(true);
  thread_ = std::thread(&CraneX7::threadLoop, this);
  return true;
}

void CraneX7::stopThread() {
  if (!thread_.joinable()) return;
  thread_run_.store(false);
  feedback_cv_.notify_all();
  target_cv_.notify_all();
  if (std::this_thread::get_id() == thread_.get_id()) {
    // called from the thread itself (deadman escalation path): just
    // signal; the loop exits on its own and join happens later
    return;
  }
  thread_.join();
  if (quiesced_.load()) return;  // the bus must stay silent
  // Safety on stop: zero motion-producing targets (vendor parity).
  const auto mode = config_.joints.front().operating_mode;
  const std::vector<double> zeros(config_.joints.size(), 0.0);
  if (mode == 1) {
    group_.writeGoalVelocities(zeros);
  } else if (mode == 0) {
    group_.writeGoalCurrents(zeros);
  }
}

CraneX7::CycleStats CraneX7::cycleStats() const {
  std::lock_guard<std::mutex> lock(cycle_mutex_);
  return stats_;
}

std::uint64_t CraneX7::waitCycle(std::uint64_t last_seen) {
  std::unique_lock<std::mutex> lock(cycle_mutex_);
  if (!thread_run_.load()) return 0;
  cycle_cv_.wait(lock, [this, last_seen] {
    return cycle_seq_ > last_seen || !thread_run_.load();
  });
  return cycle_seq_;
}

std::uint64_t CraneX7::waitFeedback(std::uint64_t last_seen) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (!thread_run_.load() || quiesced_.load()) return 0;
  feedback_cv_.wait(lock, [this, last_seen] {
    const bool window_ready =
        !synchronize_commands_ ||
        (command_window_open_ &&
         command_window_feedback_seq_ == feedback_seq_);
    return (feedback_seq_ > last_seen && window_ready) ||
           !thread_run_.load() || quiesced_.load();
  });
  return !quiesced_.load() && feedback_seq_ > last_seen ? feedback_seq_ : 0;
}

void CraneX7::requestQuiesce() {
  quiesced_.store(true);
  feedback_cv_.notify_all();
  target_cv_.notify_all();
}

void CraneX7::threadLoop() {
  const auto cycle = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(options_.control_cycle_s));
  const auto write_margin = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(options_.controller_write_margin_s));
  const auto mode = config_.joints.front().operating_mode;
  auto next = std::chrono::steady_clock::now() + cycle;

  std::vector<dxl::Feedback> fb;
  std::vector<double> targets;
  std::vector<double> auxiliary_targets;
  fb.reserve(config_.joints.size());
  targets.reserve(config_.joints.size());
  auxiliary_targets.reserve(config_.joints.size());
  WriteOutcome outcome;
  outcome.values.reserve(config_.joints.size());
  outcome.flags.reserve(config_.joints.size());
  int failed_reads_row = 0;
  int failed_writes_row = 0;
  std::uint64_t cycle_number = 0;
  auto cycle_started = std::chrono::steady_clock::now();
  const auto end_cycle = [&] {
    const auto finished = std::chrono::steady_clock::now();
    const auto advance = advanceCycleDeadline(next, finished, cycle);
    const double cycle_time_s =
        std::chrono::duration<double>(finished - cycle_started).count();
    const double lateness_s =
        std::chrono::duration<double>(advance.lateness).count();
    {
      std::lock_guard<std::mutex> lock(cycle_mutex_);
      ++cycle_seq_;
      ++stats_.cycles;
      if (advance.skipped_periods > 0) ++stats_.overruns;
      stats_.skipped_periods += advance.skipped_periods;
      stats_.max_cycle_time_s =
          std::max(stats_.max_cycle_time_s, cycle_time_s);
      stats_.max_lateness_s = std::max(stats_.max_lateness_s, lateness_s);
    }
    cycle_cv_.notify_all();
    std::this_thread::sleep_until(advance.wake_time);
    next = advance.wake_time + cycle;
    cycle_started = std::chrono::steady_clock::now();
  };
  while (thread_run_.load()) {
    ++cycle_number;
    bool read_ok = false;
    // Quiesce gates: before the read and again before the write. Once
    // requested, no further instruction packet leaves this thread —
    // reads count against the servo Bus Watchdog too, and the ensuing
    // bus silence is what lets the servos stop themselves. Cycle
    // bookkeeping stays alive so waitCycle()/stopThread() remain
    // responsive.
    if (!quiesced_.load()) {
      if (readAll(fb)) {
        read_ok = true;
        failed_reads_row = 0;
      } else {
        ++failed_reads_row;
        std::lock_guard<std::mutex> lock(cycle_mutex_);
        ++stats_.read_failures;
      }
    }
    if (quiesced_.load()) {
      end_cycle();
      continue;
    }
    std::uint64_t tseq = 0;
    std::uint64_t source_feedback_seq = 0;
    double source_feedback_time = 0.0;
    bool controller_deadline_missed = false;
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      if (synchronize_commands_ && read_ok) {
        const auto target_before_window = target_seq_;
        command_window_feedback_seq_ = feedback_seq_;
        command_window_open_ = true;
        feedback_cv_.notify_all();
        target_cv_.wait_until(
            lock, next - write_margin, [this, target_before_window] {
              return target_seq_ != target_before_window ||
                     !thread_run_.load() || quiesced_.load();
        });
        const bool received = target_seq_ != target_before_window;
        command_window_open_ = false;
        controller_deadline_missed =
            !received && submission_armed_ && thread_run_.load() &&
            !quiesced_.load();
      }
      targets = targets_;
      auxiliary_targets = auxiliary_targets_;
      tseq = target_seq_;
      source_feedback_seq = target_source_feedback_seq_;
      source_feedback_time = target_source_feedback_time_;
    }
    if (controller_deadline_missed) {
      std::lock_guard<std::mutex> lock(cycle_mutex_);
      ++stats_.controller_deadline_misses;
    }
    // A missed velocity update must not keep driving indefinitely. Position
    // mode naturally holds the preceding goal; current mode retains one
    // bounded prior command and is terminated by the submission deadman if
    // the controller does not recover.
    if (controller_deadline_missed && mode == 1) {
      std::fill(targets.begin(), targets.end(), 0.0);
    }
    if (quiesced_.load() || !thread_run_.load()) {
      end_cycle();
      continue;
    }
    bool wrote_ok = false;
    switch (mode) {
      case 3: wrote_ok = writePositions(targets, &outcome); break;
      case 1:
        wrote_ok = writeVelocitiesWithFeedback(targets, fb, &outcome);
        break;
      case 0:
        wrote_ok = writeCurrentsWithFeedback(targets, fb, &outcome);
        break;
      case 5:
        wrote_ok = writeCurrentBasedPositionsWithFeedback(
            targets, auxiliary_targets, fb, &outcome);
        break;
      default: break;
    }
    const double write_time = now_();
    {
      // Every attempt lands in the attempt record; only a SUCCESS may
      // touch the applied record — and its first_-fields only on the
      // first success of a NEW sequence, so retransmissions never
      // rewrite first-application facts while the latest transmission
      // owns the current limit/gate state (re-evaluated every write).
      std::lock_guard<std::mutex> lock(state_mutex_);
      attempt_rec_ = {true, tseq, write_time, wrote_ok};
      if (wrote_ok) {
        auto& rec = applied_rec_;
        if (!rec.valid || rec.target_seq != tseq) {
          rec.target_seq = tseq;
          rec.source_feedback_seq = source_feedback_seq;
          rec.source_feedback_time = source_feedback_time;
          rec.first_cycle = cycle_number;
          rec.first_time = write_time;
          rec.valid = true;
        }
        rec.latest_cycle = cycle_number;
        rec.latest_time = write_time;
        rec.mode = mode;
        for (std::size_t i = 0; i < outcome.values.size() &&
                                i < static_cast<std::size_t>(
                                        model::kCanonicalDof);
             ++i) {
          // mode-native units: current mode converts A -> Nm
          rec.applied[i] =
              mode == 0 ? outcome.values[i] * dxl::torqueConstant(
                                                  config_.joints[i]
                                                      .model_number)
                        : outcome.values[i];
          rec.effort_limit[i] =
              mode == 5 && i < outcome.auxiliary.size()
                  ? outcome.auxiliary[i] *
                        dxl::torqueConstant(config_.joints[i].model_number) /
                        config_.joints[i].command_torque_scale
                  : 0.0;
          rec.flags[i] = outcome.flags[i];
        }
      }
    }
    if (wrote_ok) {
      failed_writes_row = 0;
    } else {
      ++failed_writes_row;
      std::lock_guard<std::mutex> lock(cycle_mutex_);
      ++stats_.write_failures;
    }
    // Frozen feedback is the read-side trap: lastFeedback() keeps
    // serving the last good state, the controller keeps commanding into
    // a robot it can no longer see, and — sync writes being broadcast,
    // hence always "successful" — the deadman never fires. Persistent
    // write failure is the mirror trap (healthy reads, old actuator
    // goal stuck active). Both escalate exactly as a stale command
    // stream would.
    // The failure/deadman escalation path writes to the bus (via
    // deactivate()): it must not run once quiesced.
    if (!quiesced_.load()) {
      if (failed_reads_row >= options_.max_read_failures ||
          failed_writes_row >= options_.max_write_failures) {
        escalate();
        thread_run_.store(false);
      } else if (!checkDeadman()) {
        thread_run_.store(false);
      }
    }
    end_cycle();
  }
  cycle_cv_.notify_all();
  feedback_cv_.notify_all();
  target_cv_.notify_all();
}

bool CraneX7::checkDeadman() {
  if (escalated_) return false;
  if (!activated_) return true;
  const double stale = now_() - last_command_;
  if (stale > options_.host_command_timeout_s) {
    escalate();
    return false;
  }
  // Once a controller has submitted targets, the submissions themselves
  // must stay fresh: the thread's retransmissions keep last_command_
  // alive even when the controller is dead. Monitor-only sessions that
  // never submit stay exempt.
  bool armed = false;
  double last_sub = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    armed = submission_armed_;
    last_sub = last_submission_;
  }
  if (armed && now_() - last_sub > options_.host_command_timeout_s) {
    escalate();
    return false;
  }
  return true;
}

void CraneX7::escalate() {
  if (escalated_) return;
  escalated_ = true;
  // Best-effort stop — these writes may be the very thing that is
  // failing, which is why the bus goes silent afterwards regardless.
  deactivate();
  if (on_escalate_) on_escalate_();
}

bool CraneX7::switchToCurrentModeWithPreload(
    const std::vector<double>& amps) {
  if (!requireActive("mode switch")) return false;
  if (escalated_ || quiesced_.load()) return false;
  if (threadRunning()) {
    last_error_ = "mode switch requires the background thread stopped";
    return false;
  }
  if (!requireSize(amps.size(), "mode switch preload")) return false;
  std::vector<dxl::Feedback> present;
  if (!readAll(present)) {
    last_error_ = "mode switch: present-state read failed";
    return false;
  }
  std::vector<double> limited;
  std::vector<std::uint8_t> flags;
  limitCurrents(amps, present, &limited, &flags);
  for (std::size_t i = 0; i < flags.size(); ++i) {
    if (flags[i] != 0) {
      last_error_ = "mode-switch preload clipped/gated on joint " +
                    std::to_string(i) +
                    " — refusing (a limited gravity preload cannot "
                    "hold the arm)";
      return false;
    }
  }
  // The unsupported interval: first torque-off to last torque-on.
  // Everything already programmed at activation (verification,
  // indirect maps, limits, gains, Bus Watchdogs) is deliberately NOT
  // repeated.
  bool ok = true;
  for (const auto& joint : config_.joints) {
    ok = ok && io_.write8(joint.id, reg::kTorqueEnable.addr, 0).ok();
  }
  for (const auto& joint : config_.joints) {
    ok = ok && io_.write8(joint.id, reg::kOperatingMode.addr, 0).ok();
  }
  std::vector<double> zeros(config_.joints.size(), 0.0);
  std::vector<double> positions(config_.joints.size());
  for (std::size_t i = 0; i < present.size(); ++i) {
    positions[i] = present[i].position;
  }
  ok = ok && group_.writeGoals(limited, zeros, positions).ok();
  if (ok) {
    for (const auto& joint : config_.joints) {
      ok = ok && io_.write8(joint.id, reg::kTorqueEnable.addr, 1).ok();
    }
  }
  if (!ok) {
    last_error_ = "mode switch failed mid-sequence — releasing";
    bestEffortRelease();
    activated_ = false;
    return false;
  }
  for (auto& joint : config_.joints) joint.operating_mode = 0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    targets_ = amps;  // the writer re-clamps every retransmission
    have_targets_ = true;
  }
  last_command_ = now_();
  return true;
}

void CraneX7::limitCurrents(const std::vector<double>& amps,
                            const std::vector<dxl::Feedback>& fb,
                            std::vector<double>* limited,
                            std::vector<std::uint8_t>* flags) const {
  limited->resize(amps.size());
  flags->assign(amps.size(), 0);
  for (std::size_t i = 0; i < amps.size(); ++i) {
    const auto& joint = config_.joints[i];
    // bound by both the URDF effort limit and the servo's own current
    // limit (read at activation), minus the configured margin
    const double imax = std::max(
        0.0, std::min(joint.effort_limit /
                          dxl::torqueConstant(joint.model_number),
                      servo_current_limit_amps_[i]) -
                 joint.current_limit_margin);
    double a = std::clamp(amps[i], -imax, imax);
    if (a != amps[i]) (*flags)[i] |= arm::kCmdClamped;
    if ((fb[i].position >= limit_hi_[i] && a > 0.0) ||
        (fb[i].position <= limit_lo_[i] && a < 0.0)) {
      a = 0.0;
      (*flags)[i] |= arm::kCmdGated;
    }
    (*limited)[i] = a;
  }
}

void CraneX7::limitCurrentMagnitudes(
    const std::vector<double>& amps, std::vector<double>* limited,
    std::vector<std::uint8_t>* flags) const {
  limited->resize(amps.size());
  flags->assign(amps.size(), 0);
  for (std::size_t i = 0; i < amps.size(); ++i) {
    const auto& joint = config_.joints[i];
    const double imax = std::max(
        0.0, std::min(joint.effort_limit /
                          dxl::torqueConstant(joint.model_number),
                      servo_current_limit_amps_[i]) -
                 joint.current_limit_margin);
    const double requested =
        std::isfinite(amps[i]) ? std::fabs(amps[i]) : 0.0;
    (*limited)[i] = std::clamp(requested, 0.0, imax);
    if (!std::isfinite(amps[i]) || (*limited)[i] != requested) {
      (*flags)[i] |= arm::kCmdClamped;
    }
  }
}

bool CraneX7::writePositionPGain(std::uint16_t gain) {
  for (const auto& joint : config_.joints) {
    if (quiesced_.load()) return false;
    if (!io_.write16(joint.id, reg::kPositionPGain.addr, gain).ok()) {
      last_error_ = "P-gain write failed on id " + std::to_string(joint.id);
      return false;
    }
  }
  return true;
}

bool CraneX7::writeProfileVelocityRadPerSec(double rad_per_sec) {
  return writeProfileVelocity(
      dxl::profileVelocityFromRadPerSec(rad_per_sec));
}

bool CraneX7::writeProfileAccelerationRadPerSec2(double rad_per_sec2) {
  return writeProfileAcceleration(
      dxl::profileAccelerationFromRadPerSec2(rad_per_sec2));
}

bool CraneX7::writeProfileVelocity(std::uint32_t raw) {
  for (const auto& joint : config_.joints) {
    if (!io_.write32(joint.id, reg::kProfileVelocity.addr, raw).ok()) {
      return false;
    }
  }
  return true;
}

bool CraneX7::writeProfileAcceleration(std::uint32_t raw) {
  for (const auto& joint : config_.joints) {
    if (!io_.write32(joint.id, reg::kProfileAcceleration.addr, raw).ok()) {
      return false;
    }
  }
  return true;
}

}  // namespace rtctrl::hw
