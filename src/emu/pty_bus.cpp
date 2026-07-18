#include "rtctrl/emu/pty_bus.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace rtctrl::emu {

namespace {

constexpr std::uint8_t kInstPing = 0x01;
constexpr std::uint8_t kInstRead = 0x02;
constexpr std::uint8_t kInstWrite = 0x03;
constexpr std::uint8_t kInstStatus = 0x55;
constexpr std::uint8_t kInstSyncRead = 0x82;
constexpr std::uint8_t kInstSyncWrite = 0x83;
constexpr std::uint8_t kBroadcastId = 0xFE;

}  // namespace

std::uint16_t dxlCrc16(const std::uint8_t* data, std::size_t size) {
  std::uint16_t crc = 0;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x8005)
                           : static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

// Byte stuffing over the instruction+params region: any FF FF FD gains
// a trailing FD.
void dxlAddStuffing(std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> out;
  out.reserve(payload.size());
  int run = 0;
  for (const std::uint8_t byte : payload) {
    out.push_back(byte);
    if (run >= 2 && byte == 0xFD) {
      out.push_back(0xFD);
      run = 0;
      continue;
    }
    run = (byte == 0xFF) ? run + 1 : 0;
  }
  payload = std::move(out);
}

void dxlRemoveStuffing(std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> out;
  out.reserve(payload.size());
  int run = 0;
  for (std::size_t i = 0; i < payload.size(); ++i) {
    const std::uint8_t byte = payload[i];
    if (run >= 2 && byte == 0xFD && i + 1 < payload.size() &&
        payload[i + 1] == 0xFD) {
      out.push_back(0xFD);
      ++i;  // skip the stuffing byte
      run = 0;
      continue;
    }
    out.push_back(byte);
    run = (byte == 0xFF) ? run + 1 : 0;
  }
  payload = std::move(out);
}

PtyBus::PtyBus(MotorBus& bus) : bus_(bus) {
  master_fd_ = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (master_fd_ < 0 || grantpt(master_fd_) != 0 ||
      unlockpt(master_fd_) != 0) {
    throw std::runtime_error("PtyBus: cannot open pseudo-terminal");
  }
  const char* name = ptsname(master_fd_);
  if (name == nullptr) {
    throw std::runtime_error("PtyBus: ptsname failed");
  }
  slave_path_ = name;
}

PtyBus::~PtyBus() {
  if (master_fd_ >= 0) ::close(master_fd_);
}

bool PtyBus::poll(double dt) {
  std::uint8_t buf[512];
  while (true) {
    const ssize_t n = ::read(master_fd_, buf, sizeof(buf));
    if (n > 0) {
      rx_.insert(rx_.end(), buf, buf + n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    if (n < 0 && errno == EIO) break;  // no client attached yet
    if (n == 0) break;
    if (n < 0) return false;
  }
  extractFrames();
  bus_.tick(dt);
  return true;
}

void PtyBus::extractFrames() {
  while (true) {
    // Find the header.
    std::size_t start = 0;
    bool found = false;
    for (; start + 3 < rx_.size(); ++start) {
      if (rx_[start] == 0xFF && rx_[start + 1] == 0xFF &&
          rx_[start + 2] == 0xFD && rx_[start + 3] == 0x00) {
        found = true;
        break;
      }
    }
    if (!found) {
      if (rx_.size() > 3) rx_.erase(rx_.begin(), rx_.end() - 3);
      return;
    }
    if (start > 0) rx_.erase(rx_.begin(), rx_.begin() + start);
    if (rx_.size() < 7) return;  // header + id + length pending

    const std::uint16_t length =
        static_cast<std::uint16_t>(rx_[5] | (rx_[6] << 8));
    const std::size_t total = 7 + length;
    if (rx_.size() < total) return;

    const std::uint16_t crc_rx = static_cast<std::uint16_t>(
        rx_[total - 2] | (rx_[total - 1] << 8));
    const std::uint16_t crc = dxlCrc16(rx_.data(), total - 2);
    if (crc == crc_rx) {
      const std::uint8_t id = rx_[4];
      std::vector<std::uint8_t> body(rx_.begin() + 7,
                                     rx_.begin() + total - 2);
      dxlRemoveStuffing(body);
      if (!body.empty()) {
        const std::uint8_t instruction = body.front();
        body.erase(body.begin());
        handleFrame(id, instruction, body);
      }
    }
    rx_.erase(rx_.begin(), rx_.begin() + total);
  }
}

void PtyBus::sendStatus(std::uint8_t id, std::uint8_t error,
                        const std::vector<std::uint8_t>& params) {
  std::vector<std::uint8_t> body = {kInstStatus, error};
  body.insert(body.end(), params.begin(), params.end());
  dxlAddStuffing(body);

  std::vector<std::uint8_t> frame = {0xFF, 0xFF, 0xFD, 0x00, id};
  const std::uint16_t length = static_cast<std::uint16_t>(body.size() + 2);
  frame.push_back(static_cast<std::uint8_t>(length & 0xFF));
  frame.push_back(static_cast<std::uint8_t>(length >> 8));
  frame.insert(frame.end(), body.begin(), body.end());
  const std::uint16_t crc = dxlCrc16(frame.data(), frame.size());
  frame.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<std::uint8_t>(crc >> 8));

  std::size_t off = 0;
  while (off < frame.size()) {
    const ssize_t n = ::write(master_fd_, frame.data() + off,
                              frame.size() - off);
    if (n <= 0) return;
    off += static_cast<std::size_t>(n);
  }
}

void PtyBus::handleFrame(std::uint8_t id, std::uint8_t instruction,
                         const std::vector<std::uint8_t>& params) {
  switch (instruction) {
    case kInstPing: {
      auto reply = [this](MotorEmulator& motor) {
        motor.touch();
        sendStatus(motor.id(), 0,
                   {static_cast<std::uint8_t>(motor.modelNumber() & 0xFF),
                    static_cast<std::uint8_t>(motor.modelNumber() >> 8),
                    static_cast<std::uint8_t>(
                        motor.peek(dxl::reg::kFirmwareVersion))});
      };
      if (id == kBroadcastId) {
        for (auto& motor : bus_.motors()) reply(motor);
      } else if (MotorEmulator* motor = bus_.find(id)) {
        reply(*motor);
      }
      break;
    }
    case kInstRead: {
      MotorEmulator* motor = bus_.find(id);
      if (motor == nullptr || params.size() < 4) break;
      motor->touch();
      const std::uint16_t addr =
          static_cast<std::uint16_t>(params[0] | (params[1] << 8));
      const std::uint16_t len =
          static_cast<std::uint16_t>(params[2] | (params[3] << 8));
      std::vector<std::uint8_t> data(len, 0);
      const dxl::IoResult r = motor->read(addr, data.data(), len);
      sendStatus(id, r.error, data);
      break;
    }
    case kInstWrite: {
      MotorEmulator* motor = bus_.find(id);
      if (motor == nullptr || params.size() < 2) break;
      motor->touch();
      const std::uint16_t addr =
          static_cast<std::uint16_t>(params[0] | (params[1] << 8));
      const dxl::IoResult r = motor->write(
          addr, params.data() + 2,
          static_cast<std::uint16_t>(params.size() - 2));
      sendStatus(id, r.error, {});
      break;
    }
    case kInstSyncRead: {
      if (params.size() < 4) break;
      const std::uint16_t addr =
          static_cast<std::uint16_t>(params[0] | (params[1] << 8));
      const std::uint16_t len =
          static_cast<std::uint16_t>(params[2] | (params[3] << 8));
      // one status packet per listed id, in order
      for (std::size_t i = 4; i < params.size(); ++i) {
        MotorEmulator* motor = bus_.find(params[i]);
        if (motor == nullptr) continue;
        motor->touch();
        std::vector<std::uint8_t> data(len, 0);
        const dxl::IoResult r = motor->read(addr, data.data(), len);
        sendStatus(motor->id(), r.error, data);
      }
      break;
    }
    case kInstSyncWrite: {
      if (params.size() < 4) break;
      const std::uint16_t addr =
          static_cast<std::uint16_t>(params[0] | (params[1] << 8));
      const std::uint16_t len =
          static_cast<std::uint16_t>(params[2] | (params[3] << 8));
      for (std::size_t i = 4; i + len < params.size() + 1; i += 1 + len) {
        MotorEmulator* motor = bus_.find(params[i]);
        if (motor == nullptr) continue;
        motor->touch();
        motor->write(addr, params.data() + i + 1, len);
      }
      break;  // sync write has no status replies
    }
    default:
      if (id != kBroadcastId) {
        sendStatus(id, dxl::kErrInstruction, {});
      }
      break;
  }
}

}  // namespace rtctrl::emu
