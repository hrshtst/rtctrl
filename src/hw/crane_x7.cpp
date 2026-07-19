#include "rtctrl/hw/crane_x7.hpp"

#include <algorithm>
#include <cmath>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"

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
      last_command_(now_()) {}

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
  escalated_ = false;

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

  // Snap goals to the present posture so torque-on causes no motion.
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
  if (!group_.writeGoals(zeros, zeros, positions).ok()) {
    last_error_ = "goal snap failed";
    return false;
  }

  if (!writePositionPGain(options_.active_p_gain)) return false;

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

  for (const auto& joint : config_.joints) {
    if (!io_.write8(joint.id, reg::kTorqueEnable.addr, 1).ok()) {
      last_error_ = "torque-on failed on id " + std::to_string(joint.id);
      return false;
    }
  }
  last_command_ = now_();
  activated_ = true;
  return true;
}

bool CraneX7::deactivate() {
  stopThread();
  bool ok = true;
  ok &= writePositionPGain(options_.limp_p_gain);
  // zero goal currents (relevant in current mode; harmless otherwise)
  for (const auto& joint : config_.joints) {
    ok &= io_.write16(joint.id, reg::kGoalCurrent.addr, 0).ok();
  }
  for (const auto& joint : config_.joints) {
    ok &= io_.write8(joint.id, reg::kTorqueEnable.addr, 0).ok();
    ok &= io_.write8(joint.id, reg::kBusWatchdog.addr, 0).ok();
  }
  activated_ = false;
  return ok;
}

bool CraneX7::readAll(std::vector<dxl::Feedback>& out) {
  if (!group_.readAll(out).ok()) return false;
  std::lock_guard<std::mutex> lock(state_mutex_);
  feedback_ = out;
  return true;
}

std::vector<dxl::Feedback> CraneX7::lastFeedback() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return feedback_;
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

bool CraneX7::writePositions(const std::vector<double>& rad) {
  if (escalated_) return false;
  if (!requireMode(3, "position command")) return false;
  std::vector<double> clamped(rad.size());
  for (std::size_t i = 0; i < rad.size(); ++i) {
    clamped[i] = std::clamp(rad[i], limit_lo_[i], limit_hi_[i]);
  }
  if (!group_.writeGoalPositions(clamped).ok()) return false;
  last_command_ = now_();
  return true;
}

bool CraneX7::writeVelocities(const std::vector<double>& rad_s) {
  if (escalated_) return false;
  if (!requireMode(1, "velocity command")) return false;
  std::vector<dxl::Feedback> fb = lastFeedback();
  if (fb.size() != config_.joints.size()) {
    last_error_ = "velocity command rejected: no position feedback yet "
                  "(software limits need it)";
    return false;
  }
  std::vector<double> limited(rad_s.size());
  for (std::size_t i = 0; i < rad_s.size(); ++i) {
    const double vmax = config_.joints[i].velocity_limit;
    double v = std::clamp(rad_s[i], -vmax, vmax);
    // On-servo position limits are inactive outside position mode:
    // zero any command that drives a joint past a limit.
    if ((fb[i].position >= limit_hi_[i] && v > 0.0) ||
        (fb[i].position <= limit_lo_[i] && v < 0.0)) {
      v = 0.0;
    }
    limited[i] = v;
  }
  if (!group_.writeGoalVelocities(limited).ok()) return false;
  last_command_ = now_();
  return true;
}

bool CraneX7::writeCurrents(const std::vector<double>& amps) {
  if (escalated_) return false;
  if (!requireMode(0, "current command")) return false;
  std::vector<dxl::Feedback> fb = lastFeedback();
  if (fb.size() != config_.joints.size()) {
    last_error_ = "current command rejected: no position feedback yet "
                  "(software limits need it)";
    return false;
  }
  std::vector<double> limited(amps.size());
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
    if ((fb[i].position >= limit_hi_[i] && a > 0.0) ||
        (fb[i].position <= limit_lo_[i] && a < 0.0)) {
      a = 0.0;
    }
    limited[i] = a;
  }
  if (!group_.writeGoalCurrents(limited).ok()) return false;
  last_command_ = now_();
  return true;
}

void CraneX7::setTargetPositions(const std::vector<double>& rad) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  targets_ = rad;
  have_targets_ = true;
}
void CraneX7::setTargetVelocities(const std::vector<double>& rad_s) {
  setTargetPositions(rad_s);  // same storage; units follow the mode
}
void CraneX7::setTargetCurrents(const std::vector<double>& amps) {
  setTargetPositions(amps);
}

bool CraneX7::startThread() {
  if (thread_.joinable()) return true;
  if (!activated_) {
    last_error_ = "startThread: activate first";
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
    if (!have_targets_) {
      targets_.assign(config_.joints.size(), 0.0);
      if (mode == 3) {
        for (std::size_t i = 0; i < feedback_.size(); ++i) {
          targets_[i] = feedback_[i].position;
        }
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
  if (std::this_thread::get_id() == thread_.get_id()) {
    // called from the thread itself (deadman escalation path): just
    // signal; the loop exits on its own and join happens later
    return;
  }
  thread_.join();
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

void CraneX7::threadLoop() {
  const auto cycle = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(options_.control_cycle_s));
  const auto mode = config_.joints.front().operating_mode;
  auto next = std::chrono::steady_clock::now() + cycle;

  std::vector<dxl::Feedback> fb;
  std::vector<double> targets;
  while (thread_run_.load()) {
    readAll(fb);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      targets = targets_;
    }
    switch (mode) {
      case 3: writePositions(targets); break;
      case 1: writeVelocities(targets); break;
      case 0: writeCurrents(targets); break;
      default: break;
    }
    if (!checkDeadman()) {
      thread_run_.store(false);
    }

    {
      std::lock_guard<std::mutex> lock(cycle_mutex_);
      ++cycle_seq_;
      ++stats_.cycles;
      if (std::chrono::steady_clock::now() > next) ++stats_.overruns;
    }
    cycle_cv_.notify_all();

    std::this_thread::sleep_until(next);
    next += cycle;
  }
  cycle_cv_.notify_all();
}

bool CraneX7::checkDeadman() {
  if (escalated_) return false;
  if (!activated_) return true;
  const double stale = now_() - last_command_;
  if (stale > options_.host_command_timeout_s) {
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

bool CraneX7::writePositionPGain(std::uint16_t gain) {
  for (const auto& joint : config_.joints) {
    if (!io_.write16(joint.id, reg::kPositionPGain.addr, gain).ok()) {
      last_error_ = "P-gain write failed on id " + std::to_string(joint.id);
      return false;
    }
  }
  return true;
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
