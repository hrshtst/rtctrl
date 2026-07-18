#include "rtctrl/dxl/port.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

#include <stdexcept>

namespace rtctrl::dxl {

Port::Port(const std::string& device, int baudrate)
    : port_(dynamixel::PortHandler::getPortHandler(device.c_str())),
      packet_(dynamixel::PacketHandler::getPacketHandler(2.0)) {
  if (!port_->openPort()) {
    throw std::runtime_error("Port: cannot open " + device);
  }
  if (!port_->setBaudRate(baudrate)) {
    port_->closePort();
    throw std::runtime_error("Port: cannot set baudrate on " + device);
  }
  open_ = true;
}

Port::~Port() { close(); }

void Port::close() {
  if (open_) {
    port_->closePort();
    open_ = false;
  }
}

IoResult Port::ping(std::uint8_t id, std::uint16_t* model_number) {
  IoResult r;
  std::uint16_t model = 0;
  r.comm = packet_->ping(port_, id, &model, &r.error);
  if (model_number != nullptr) *model_number = model;
  return r;
}

IoResult Port::read(std::uint8_t id, std::uint16_t addr, std::uint8_t* data,
                    std::uint16_t len) {
  IoResult r;
  r.comm = packet_->readTxRx(port_, id, addr, len, data, &r.error);
  return r;
}

IoResult Port::write(std::uint8_t id, std::uint16_t addr,
                     const std::uint8_t* data, std::uint16_t len) {
  IoResult r;
  r.comm = packet_->writeTxRx(port_, id, addr, len,
                              const_cast<std::uint8_t*>(data), &r.error);
  return r;
}

IoResult Port::syncRead(std::uint16_t addr, std::uint16_t len,
                        const std::vector<std::uint8_t>& ids,
                        std::vector<std::uint8_t>& out) {
  IoResult r;
  dynamixel::GroupSyncRead group(port_, packet_, addr, len);
  for (const auto id : ids) group.addParam(id);
  r.comm = group.txRxPacket();
  if (r.comm != COMM_SUCCESS) return r;

  out.assign(ids.size() * len, 0);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    std::uint8_t err = 0;
    if (group.getError(ids[i], &err) && err != 0 && r.error == 0) {
      r.error = err;
    }
    if (!group.isAvailable(ids[i], addr, len)) {
      r.comm = COMM_RX_CORRUPT;
      return r;
    }
    for (std::uint16_t b = 0; b < len; ++b) {
      out[i * len + b] = static_cast<std::uint8_t>(
          group.getData(ids[i], addr + b, 1));
    }
  }
  return r;
}

IoResult Port::syncWrite(std::uint16_t addr, std::uint16_t len,
                         const std::vector<std::uint8_t>& ids,
                         const std::vector<std::uint8_t>& data) {
  IoResult r;
  dynamixel::GroupSyncWrite group(port_, packet_, addr, len);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    group.addParam(ids[i], const_cast<std::uint8_t*>(&data[i * len]));
  }
  r.comm = group.txPacket();
  return r;
}

}  // namespace rtctrl::dxl
