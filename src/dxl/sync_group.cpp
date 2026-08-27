#include "rtctrl/dxl/sync_group.hpp"

#include <stdexcept>

#include "rtctrl/dxl/conversions.hpp"

namespace rtctrl::dxl {

namespace {

std::uint16_t leU16(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}
std::uint32_t leU32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) |
         (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) |
         (static_cast<std::uint32_t>(p[3]) << 24);
}

}  // namespace

SyncGroup::SyncGroup(PacketIO& io, std::vector<std::uint8_t> ids)
    : io_(io), ids_(std::move(ids)) {
  if (ids_.empty()) {
    throw std::invalid_argument("SyncGroup: empty id list");
  }
}

IoResult SyncGroup::readMotion(std::vector<Feedback>& out) {
  constexpr std::uint16_t kMotionAddr = reg::kPresentCurrent.addr;
  constexpr std::uint16_t kMotionBytes =
      reg::kPresentPosition.addr + reg::kPresentPosition.len - kMotionAddr;
  static_assert(kMotionBytes == 10);
  const IoResult r = io_.syncRead(kMotionAddr, kMotionBytes, ids_, raw_motion_);
  if (r.comm != 0) return r;

  out.assign(ids_.size(), {});
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    const std::uint8_t* p = &raw_motion_[i * kMotionBytes];
    out[i].current = currentToAmps(static_cast<std::int16_t>(leU16(p)));
    out[i].velocity =
        velocityToRadPerSec(static_cast<std::int32_t>(leU32(p + 2)));
    out[i].position =
        pulseToRad(static_cast<std::int32_t>(leU32(p + 6)));
  }
  return r;
}

IoResult SyncGroup::setupIndirect() {
  // Source registers backing each indirect slot, in window order.
  std::vector<std::uint16_t> sources;
  auto append = [&sources](Reg r) {
    for (std::uint16_t i = 0; i < r.len; ++i) {
      sources.push_back(static_cast<std::uint16_t>(r.addr + i));
    }
  };
  append(reg::kPresentCurrent);
  append(reg::kPresentVelocity);
  append(reg::kPresentPosition);
  append(reg::kPresentInputVoltage);
  append(reg::kPresentTemperature);
  append(reg::kGoalCurrent);
  append(reg::kGoalVelocity);
  append(reg::kGoalPosition);

  for (const std::uint8_t id : ids_) {
    for (std::size_t slot = 0; slot < sources.size(); ++slot) {
      const IoResult r = io_.write16(
          id,
          static_cast<std::uint16_t>(reg::kIndirectAddressBase + 2 * slot),
          sources[slot]);
      if (!r.ok()) return r;
    }
  }
  return {};
}

IoResult SyncGroup::readAll(std::vector<Feedback>& out) {
  const IoResult r =
      io_.syncRead(kFeedbackAddr, kFeedbackSlots, ids_, raw_feedback_);
  if (r.comm != 0) return r;

  out.resize(ids_.size());
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    const std::uint8_t* p = &raw_feedback_[i * kFeedbackSlots];
    out[i].position =
        pulseToRad(static_cast<std::int32_t>(leU32(p + 6)));
    out[i].velocity =
        velocityToRadPerSec(static_cast<std::int32_t>(leU32(p + 2)));
    out[i].current = currentToAmps(static_cast<std::int16_t>(leU16(p)));
    out[i].voltage = voltageToVolts(leU16(p + 10));
    out[i].temperature = p[12];
  }
  return r;
}

namespace {

void putU16(std::uint8_t* p, std::uint16_t v) {
  p[0] = static_cast<std::uint8_t>(v & 0xFF);
  p[1] = static_cast<std::uint8_t>(v >> 8);
}
void putU32(std::uint8_t* p, std::uint32_t v) {
  p[0] = static_cast<std::uint8_t>(v & 0xFF);
  p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<std::uint8_t>(v >> 24);
}

}  // namespace

IoResult SyncGroup::writeGoals(const std::vector<double>& current_amps,
                               const std::vector<double>& velocity_rad_s,
                               const std::vector<double>& position_rad) {
  if (current_amps.size() != ids_.size() ||
      velocity_rad_s.size() != ids_.size() ||
      position_rad.size() != ids_.size()) {
    throw std::invalid_argument("SyncGroup::writeGoals: size mismatch");
  }
  goal_data_.resize(ids_.size() * kGoalSlots);
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    std::uint8_t* p = &goal_data_[i * kGoalSlots];
    putU16(p, static_cast<std::uint16_t>(ampsToCurrent(current_amps[i])));
    putU32(p + 2, static_cast<std::uint32_t>(
                      radPerSecToVelocity(velocity_rad_s[i])));
    putU32(p + 6, static_cast<std::uint32_t>(radToPulse(position_rad[i])));
  }
  return io_.syncWrite(kGoalAddr, kGoalSlots, ids_, goal_data_);
}

IoResult SyncGroup::writeGoalCurrents(const std::vector<double>& amps) {
  if (amps.size() != ids_.size()) {
    throw std::invalid_argument("SyncGroup::writeGoalCurrents: size mismatch");
  }
  goal_data_.resize(ids_.size() * 2);
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    putU16(&goal_data_[i * 2],
           static_cast<std::uint16_t>(ampsToCurrent(amps[i])));
  }
  return io_.syncWrite(kGoalAddr, 2, ids_, goal_data_);
}

IoResult SyncGroup::writeGoalVelocities(const std::vector<double>& rad_s) {
  if (rad_s.size() != ids_.size()) {
    throw std::invalid_argument(
        "SyncGroup::writeGoalVelocities: size mismatch");
  }
  goal_data_.resize(ids_.size() * 4);
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    putU32(&goal_data_[i * 4],
           static_cast<std::uint32_t>(radPerSecToVelocity(rad_s[i])));
  }
  return io_.syncWrite(kGoalVelocityAddr, 4, ids_, goal_data_);
}

IoResult SyncGroup::writeGoalPositions(const std::vector<double>& rad) {
  if (rad.size() != ids_.size()) {
    throw std::invalid_argument(
        "SyncGroup::writeGoalPositions: size mismatch");
  }
  goal_data_.resize(ids_.size() * 4);
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    putU32(&goal_data_[i * 4],
           static_cast<std::uint32_t>(radToPulse(rad[i])));
  }
  return io_.syncWrite(kGoalPositionAddr, 4, ids_, goal_data_);
}

}  // namespace rtctrl::dxl
