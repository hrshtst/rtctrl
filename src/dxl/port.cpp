#include "rtctrl/dxl/port.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <stdexcept>

namespace rtctrl::dxl {

namespace {

speed_t speedConstant(int baudrate) {
  switch (baudrate) {
    case 57600: return B57600;
    case 115200: return B115200;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    case 3000000: return B3000000;
    case 4000000: return B4000000;
    default: return B0;
  }
}

// DynamixelSDK's PortHandlerLinux::setupPort encodes the baud as CBAUD
// bits OR'd into c_cflag of a zeroed struct termios and never calls
// cfsetispeed/cfsetospeed. glibc >= 2.42 programs the line speed from
// the struct's c_ispeed/c_ospeed fields (kernel termios2), so the SDK
// leaves the port at a garbage speed — every servo reply is framing
// noise. Reassert a correct raw-8N1 configuration through a side fd
// (termios state is per-device, shared across fds).
void reassertLineSettings(const std::string& device, int baudrate) {
  const speed_t speed = speedConstant(baudrate);
  if (speed == B0) {
    throw std::runtime_error("Port: unsupported baudrate " +
                             std::to_string(baudrate));
  }
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error("Port: cannot reopen " + device +
                             " for line configuration");
  }
  struct termios tio {};
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tio.c_iflag = IGNPAR;
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  const int rc = tcsetattr(fd, TCSANOW, &tio);
  ::close(fd);
  if (rc != 0) {
    throw std::runtime_error("Port: tcsetattr failed on " + device);
  }
}

// The SDK's PortHandlerLinux strcpy()s the device path into a fixed
// 100-byte buffer — a longer path is a fortify abort deep inside
// getPortHandler (observed with a 101-char pty path). Fail loudly and
// name the limit instead.
dynamixel::PortHandler* checkedPortHandler(const std::string& device) {
  if (device.size() >= 100) {
    throw std::runtime_error(
        "Port: device path exceeds the DynamixelSDK's 100-character "
        "limit (" + std::to_string(device.size()) + " chars): " + device);
  }
  return dynamixel::PortHandler::getPortHandler(device.c_str());
}

}  // namespace

Port::Port(const std::string& device, int baudrate)
    : port_(checkedPortHandler(device)),
      packet_(dynamixel::PacketHandler::getPacketHandler(2.0)) {
  if (!port_->openPort()) {
    throw std::runtime_error("Port: cannot open " + device);
  }
  if (!port_->setBaudRate(baudrate)) {
    port_->closePort();
    throw std::runtime_error("Port: cannot set baudrate on " + device);
  }
  reassertLineSettings(device, baudrate);
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
