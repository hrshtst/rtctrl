#include "rtctrl/emu/fake_packet_io.hpp"

namespace rtctrl::emu {

namespace {
constexpr int kCommRxTimeout = -3001;  // mirrors SDK COMM_RX_TIMEOUT
}

dxl::IoResult FakePacketIO::ping(std::uint8_t id,
                                 std::uint16_t* model_number) {
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  MotorEmulator* motor = bus_.find(id);
  if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
  motor->touch();
  if (model_number != nullptr) *model_number = motor->modelNumber();
  return {};
}

dxl::IoResult FakePacketIO::read(std::uint8_t id, std::uint16_t addr,
                                 std::uint8_t* data, std::uint16_t len) {
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  if (forced_read_comm_ != 0) return {forced_read_comm_, dxl::kErrNone};
  MotorEmulator* motor = bus_.find(id);
  if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
  motor->touch();
  return motor->read(addr, data, len);
}

dxl::IoResult FakePacketIO::write(std::uint8_t id, std::uint16_t addr,
                                  const std::uint8_t* data,
                                  std::uint16_t len) {
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  if (forced_write_comm_ != 0) return {forced_write_comm_, dxl::kErrNone};
  if (fail_on_comm_ != 0 && id == fail_id_ && addr == fail_addr_ &&
      len >= 1 && data[0] == fail_value_) {
    return {fail_on_comm_, dxl::kErrNone};
  }
  MotorEmulator* motor = bus_.find(id);
  if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
  motor->touch();
  return motor->write(addr, data, len);
}

dxl::IoResult FakePacketIO::syncRead(std::uint16_t addr, std::uint16_t len,
                                     const std::vector<std::uint8_t>& ids,
                                     std::vector<std::uint8_t>& out) {
  using Clock = std::chrono::steady_clock;
  dxl::SyncReadTiming& timing = last_sync_read_timing_;
  timing = {};
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  if (forced_read_comm_ != 0) return {forced_read_comm_, dxl::kErrNone};
  timing.instruction_sent = Clock::now();
  timing.sent = true;
  out.assign(ids.size() * len, 0);
  dxl::IoResult result;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    MotorEmulator* motor = bus_.find(ids[i]);
    if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
    motor->touch();
    const dxl::IoResult r = motor->read(addr, &out[i * len], len);
    timing.last_reply = Clock::now();
    if (timing.replies++ == 0) timing.first_reply = timing.last_reply;
    if (result.error == dxl::kErrNone) result.error = r.error;
  }
  return result;
}

dxl::IoResult FakePacketIO::fastSyncRead(std::uint16_t addr,
                                         std::uint16_t len,
                                         const std::vector<std::uint8_t>& ids,
                                         std::vector<std::uint8_t>& out) {
  using Clock = std::chrono::steady_clock;
  dxl::SyncReadTiming& timing = last_sync_read_timing_;
  timing = {};
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  if (forced_read_comm_ != 0) return {forced_read_comm_, dxl::kErrNone};
  timing.instruction_sent = Clock::now();
  timing.sent = true;
  // One reply the servos build in turn: one absent, or predating the
  // instruction (firmware below v45), breaks the chain and nothing comes.
  for (const std::uint8_t id : ids) {
    const MotorEmulator* motor = bus_.find(id);
    if (motor == nullptr || motor->peek(dxl::reg::kFirmwareVersion) <
                                dxl::kMinFirmwareFastSyncRead) {
      return {kCommRxTimeout, dxl::kErrNone};
    }
  }
  out.assign(ids.size() * len, 0);
  dxl::IoResult result;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    MotorEmulator* motor = bus_.find(ids[i]);
    motor->touch();
    const dxl::IoResult r = motor->read(addr, &out[i * len], len);
    if (result.error == dxl::kErrNone) result.error = r.error;
  }
  timing.first_reply = timing.last_reply = Clock::now();
  timing.replies = static_cast<int>(ids.size());
  return result;
}

dxl::IoResult FakePacketIO::syncWrite(std::uint16_t addr, std::uint16_t len,
                                      const std::vector<std::uint8_t>& ids,
                                      const std::vector<std::uint8_t>& data) {
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  if (forced_write_comm_ != 0) return {forced_write_comm_, dxl::kErrNone};
  dxl::IoResult result;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    MotorEmulator* motor = bus_.find(ids[i]);
    if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
    motor->touch();
    const dxl::IoResult r = motor->write(addr, &data[i * len], len);
    if (result.error == dxl::kErrNone) result.error = r.error;
  }
  return result;
}

}  // namespace rtctrl::emu
