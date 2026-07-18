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
  MotorEmulator* motor = bus_.find(id);
  if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
  motor->touch();
  return motor->read(addr, data, len);
}

dxl::IoResult FakePacketIO::write(std::uint8_t id, std::uint16_t addr,
                                  const std::uint8_t* data,
                                  std::uint16_t len) {
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  MotorEmulator* motor = bus_.find(id);
  if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
  motor->touch();
  return motor->write(addr, data, len);
}

dxl::IoResult FakePacketIO::syncRead(std::uint16_t addr, std::uint16_t len,
                                     const std::vector<std::uint8_t>& ids,
                                     std::vector<std::uint8_t>& out) {
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
  out.assign(ids.size() * len, 0);
  dxl::IoResult result;
  for (std::size_t i = 0; i < ids.size(); ++i) {
    MotorEmulator* motor = bus_.find(ids[i]);
    if (motor == nullptr) return {kCommRxTimeout, dxl::kErrNone};
    motor->touch();
    const dxl::IoResult r = motor->read(addr, &out[i * len], len);
    if (result.error == dxl::kErrNone) result.error = r.error;
  }
  return result;
}

dxl::IoResult FakePacketIO::syncWrite(std::uint16_t addr, std::uint16_t len,
                                      const std::vector<std::uint8_t>& ids,
                                      const std::vector<std::uint8_t>& data) {
  if (forced_comm_ != 0) return {forced_comm_, dxl::kErrNone};
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
