#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include "rtctrl/dxl/error.hpp"

namespace rtctrl::dxl {

// Host-side stamps of one syncRead, for timing records only. They mark
// when the host's calls returned, not when bytes were on the wire.
struct SyncReadTiming {
  std::chrono::steady_clock::time_point instruction_sent{};  // write returned
  std::chrono::steady_clock::time_point first_reply{};
  std::chrono::steady_clock::time_point last_reply{};
  int replies = 0;      // status packets received
  bool sent = false;    // the instruction was transmitted
};

// Abstract Dynamixel bus transactions — the mocking seam. Implemented
// by dxl::Port over DynamixelSDK and by emu::FakePacketIO over the
// motor emulator, so every layer above can run without hardware.
class PacketIO {
 public:
  virtual ~PacketIO() = default;

  virtual IoResult ping(std::uint8_t id,
                        std::uint16_t* model_number = nullptr) = 0;
  virtual IoResult read(std::uint8_t id, std::uint16_t addr,
                        std::uint8_t* data, std::uint16_t len) = 0;
  virtual IoResult write(std::uint8_t id, std::uint16_t addr,
                         const std::uint8_t* data, std::uint16_t len) = 0;

  // One bus transaction over multiple ids at the same (addr, len)
  // window. `out` receives ids.size()*len bytes in id order; `data`
  // supplies the same shape. Per-id status errors are reported through
  // the return value's `error` (first nonzero wins).
  virtual IoResult syncRead(std::uint16_t addr, std::uint16_t len,
                            const std::vector<std::uint8_t>& ids,
                            std::vector<std::uint8_t>& out) = 0;
  virtual IoResult syncWrite(std::uint16_t addr, std::uint16_t len,
                             const std::vector<std::uint8_t>& ids,
                             const std::vector<std::uint8_t>& data) = 0;

  // Stamps of the most recent syncRead, read back by the thread that made
  // it; an implementation without stamps reports none (sent == false).
  virtual SyncReadTiming lastSyncReadTiming() const { return {}; }

  // Typed single-register conveniences.
  IoResult write8(std::uint8_t id, std::uint16_t addr, std::uint8_t value) {
    return write(id, addr, &value, 1);
  }
  IoResult write16(std::uint8_t id, std::uint16_t addr, std::uint16_t value) {
    const std::uint8_t b[2] = {static_cast<std::uint8_t>(value & 0xFF),
                               static_cast<std::uint8_t>(value >> 8)};
    return write(id, addr, b, 2);
  }
  IoResult write32(std::uint8_t id, std::uint16_t addr, std::uint32_t value) {
    const std::uint8_t b[4] = {static_cast<std::uint8_t>(value & 0xFF),
                               static_cast<std::uint8_t>((value >> 8) & 0xFF),
                               static_cast<std::uint8_t>((value >> 16) & 0xFF),
                               static_cast<std::uint8_t>(value >> 24)};
    return write(id, addr, b, 4);
  }
  IoResult read8(std::uint8_t id, std::uint16_t addr, std::uint8_t* value) {
    return read(id, addr, value, 1);
  }
  IoResult read16(std::uint8_t id, std::uint16_t addr, std::uint16_t* value) {
    std::uint8_t b[2] = {};
    const IoResult r = read(id, addr, b, 2);
    *value = static_cast<std::uint16_t>(b[0] | (b[1] << 8));
    return r;
  }
  IoResult read32(std::uint8_t id, std::uint16_t addr, std::uint32_t* value) {
    std::uint8_t b[4] = {};
    const IoResult r = read(id, addr, b, 4);
    *value = static_cast<std::uint32_t>(b[0]) |
             (static_cast<std::uint32_t>(b[1]) << 8) |
             (static_cast<std::uint32_t>(b[2]) << 16) |
             (static_cast<std::uint32_t>(b[3]) << 24);
    return r;
  }
};

}  // namespace rtctrl::dxl
