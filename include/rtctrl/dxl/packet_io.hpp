#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
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

// One grouped read the bounded path takes over: its first register, its
// data bytes per servo, and its instruction (Sync Read, or Fast Sync Read
// from Phase B3).
struct BoundedReadWindow {
  std::uint16_t addr = 0;
  std::uint16_t len = 0;
  GroupedRead instruction = GroupedRead::kSync;
};

// The bounded grouped read (control-rate plan, contract C-4): which reads
// it replaces, and its time limits. dxl::Port documents the transaction.
// Several windows serve a feedback read that alternates between lengths
// (the motion-only read and its auxiliary full read, Phase B2); they share
// the servos and the limits.
struct BoundedReadConfig {
  std::vector<BoundedReadWindow> windows;
  std::vector<std::uint8_t> ids;   // request order
  // B_txn: from the instruction's write() return to the last reply byte.
  std::chrono::nanoseconds budget{0};
  // G: further quiet time after the budget of a failed read, before the
  // next transmission of any kind.
  std::chrono::nanoseconds guard{0};
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

  // The same window as syncRead, by Fast Sync Read (0x8A, firmware v45 and
  // later): one status packet answers for every servo. A transport that
  // cannot send it returns kCommNotAvailable and sends nothing (the
  // default; dxl::Port sends it only on an armed bounded window).
  virtual IoResult fastSyncRead(std::uint16_t addr, std::uint16_t len,
                                const std::vector<std::uint8_t>& ids,
                                std::vector<std::uint8_t>& out) {
    (void)addr;
    (void)len;
    (void)ids;
    (void)out;
    return {kCommNotAvailable, kErrNone};
  }

  // Stamps of the most recent syncRead or fastSyncRead, read back by the
  // thread that made it; an implementation without stamps reports none
  // (sent == false). A Fast Sync Read's replies are its servos' sections
  // of the one packet.
  virtual SyncReadTiming lastSyncReadTiming() const { return {}; }

  // While *flag is true, an implementation that supports it refuses every
  // transmission (kCommTransmitInhibited) and abandons any wait before
  // one. The owner of the flag sets it once and keeps it alive while
  // registered; nullptr unregisters. The default ignores the flag (the
  // in-process emulator: the arm's own quiesce gates are what its tests
  // check).
  virtual void setTransmitInhibit(const std::atomic<bool>* flag) {
    (void)flag;
  }

  // The bounded grouped read for rates above 100 Hz. armBoundedSyncRead
  // returns whether this transport armed it (the default cannot);
  // disarming returns to the plain path.
  virtual bool armBoundedSyncRead(const BoundedReadConfig& config) {
    (void)config;
    return false;
  }
  virtual void disarmBoundedSyncRead() {}

  // The serial adapter's latency timer [ms], when the device exposes it
  // (an FTDI adapter under Linux does); none otherwise, so unverified.
  virtual std::optional<int> latencyTimerMs() const { return std::nullopt; }

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
