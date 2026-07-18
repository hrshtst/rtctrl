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
  std::vector<std::uint8_t> raw;
  const IoResult r = io_.syncRead(kFeedbackAddr, kFeedbackSlots, ids_, raw);
  if (r.comm != 0) return r;

  out.resize(ids_.size());
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    const std::uint8_t* p = &raw[i * kFeedbackSlots];
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

IoResult SyncGroup::writeGoals(const std::vector<double>& current_amps,
                               const std::vector<double>& position_rad) {
  if (current_amps.size() != ids_.size() ||
      position_rad.size() != ids_.size()) {
    throw std::invalid_argument("SyncGroup::writeGoals: size mismatch");
  }
  std::vector<std::uint8_t> data(ids_.size() * kGoalSlots);
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    std::uint8_t* p = &data[i * kGoalSlots];
    const auto cur =
        static_cast<std::uint16_t>(ampsToCurrent(current_amps[i]));
    const auto pos = static_cast<std::uint32_t>(radToPulse(position_rad[i]));
    p[0] = static_cast<std::uint8_t>(cur & 0xFF);
    p[1] = static_cast<std::uint8_t>(cur >> 8);
    p[2] = static_cast<std::uint8_t>(pos & 0xFF);
    p[3] = static_cast<std::uint8_t>((pos >> 8) & 0xFF);
    p[4] = static_cast<std::uint8_t>((pos >> 16) & 0xFF);
    p[5] = static_cast<std::uint8_t>(pos >> 24);
  }
  return io_.syncWrite(kGoalAddr, kGoalSlots, ids_, data);
}

}  // namespace rtctrl::dxl
