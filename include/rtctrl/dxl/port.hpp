#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rtctrl/dxl/packet_io.hpp"
#include "rtctrl/dxl/status_parser.hpp"

namespace dynamixel {
class PortHandler;
class PacketHandler;
}  // namespace dynamixel

namespace rtctrl::dxl {

// Counters of the bounded read, for diagnostics and the timing probe.
struct BoundedReadStats {
  std::uint64_t reads = 0;          // bounded transactions started
  std::uint64_t timeouts = 0;       // the budget ran out
  std::uint64_t malformed = 0;      // the replies could not be valid
  std::uint64_t servo_errors = 0;   // well formed, a nonzero error byte
  std::uint64_t recoveries = 0;     // clean recovery reads
  std::uint64_t guard_waits = 0;    // transmissions held by the quiet guard
  std::uint64_t inhibited = 0;      // transmissions refused by the inhibit
  StatusReplyParser::Fault last_fault = StatusReplyParser::Fault::kNone;
};

// PacketIO over DynamixelSDK (Protocol 2.0): the real serial bus.
//
// By default every transaction goes through the SDK, as at 100 Hz. Above
// 100 Hz the caller arms a bounded grouped read for its (instruction,
// addr, len) windows over one list of ids (armBoundedSyncRead). Such a
// syncRead, or fastSyncRead for a Fast Sync Read window, then sends its
// instruction
// through the SDK but receives itself: it polls the port against a
// deadline of `budget` after the instruction's write() returned, on the
// monotonic clock (the SDK's own timeout uses the wall clock), and checks
// the replies with StatusReplyParser as they arrive. A read whose budget
// runs out, or whose bytes cannot be a valid reply (garbage, a partial or
// corrupted packet, an unexpected ID, extra bytes), is an ambiguous
// failure: late bytes may still be on their way. Then:
//  1. the read fails and its data is not used;
//  2. quiet guard: no transmission of any kind (grouped or single reads,
//     writes, pings, fallback writes included) leaves before
//     t0 + budget + guard; a transmission attempted earlier waits;
//  3. received input is discarded when the guard ends;
//  4. recovery epoch: the next bounded read, of whichever window, asks for
//     its len + 1 bytes, so a late reply to a normal request fails on its
//     length field. A clean recovery read returns its first len bytes per
//     servo, and the normal length resumes. The byte after each window
//     must be readable. A status reply does not name its register, so
//     two windows of one instruction differ in length by at least 2: then
//     no reply to one window, at its normal or its recovery length, has
//     the length of another window's request in either epoch. Replies to
//     the two instructions differ in their first packet's ID (a servo's,
//     or the broadcast ID), so one cannot pass for the other.
// A servo's nonzero error byte in a well-formed reply fails the read like
// the SDK path (IoResult::error) and is not ambiguous. The unarmed path
// is the SDK's, unchanged; no guard is ever set on it. Fast Sync Read
// has no unarmed path: fastSyncRead outside an armed window returns
// kCommNotAvailable without sending.
//
// Nothing is accepted at or after the deadline: the clock is read after
// every receive, so a receiver paused across the deadline (preempted, or
// stopped) fails the read, even if the bytes it then holds would
// complete it.
//
// setTransmitInhibit registers the owner's quiesce flag. Once it is set,
// every transmission is refused with kCommTransmitInhibited, checked
// immediately before sending, and a transmission waiting for the quiet
// guard gives up without sending: the wait polls the flag every
// millisecond, and the response also includes scheduling delay.
class Port : public PacketIO {
 public:
  Port(const std::string& device, int baudrate);
  ~Port() override;

  Port(const Port&) = delete;
  Port& operator=(const Port&) = delete;

  IoResult ping(std::uint8_t id, std::uint16_t* model_number) override;
  IoResult read(std::uint8_t id, std::uint16_t addr, std::uint8_t* data,
                std::uint16_t len) override;
  IoResult write(std::uint8_t id, std::uint16_t addr,
                 const std::uint8_t* data, std::uint16_t len) override;
  IoResult syncRead(std::uint16_t addr, std::uint16_t len,
                    const std::vector<std::uint8_t>& ids,
                    std::vector<std::uint8_t>& out) override;
  IoResult syncWrite(std::uint16_t addr, std::uint16_t len,
                     const std::vector<std::uint8_t>& ids,
                     const std::vector<std::uint8_t>& data) override;
  IoResult fastSyncRead(std::uint16_t addr, std::uint16_t len,
                        const std::vector<std::uint8_t>& ids,
                        std::vector<std::uint8_t>& out) override;
  SyncReadTiming lastSyncReadTiming() const override {
    return last_sync_read_timing_;
  }
  void setTransmitInhibit(const std::atomic<bool>* flag) override {
    inhibit_ = flag;
  }

  // Stop all bus traffic: the loss-of-comms escalation path — after
  // this the servo-side Bus Watchdogs necessarily fire.
  void close();
  bool isOpen() const { return open_; }

  // Arms the bounded grouped read for exactly these windows over these ids
  // and returns true; any other syncRead keeps the SDK path. Allocates its
  // buffers here, none per read. Throws std::invalid_argument for no ids,
  // no window, a len of 0 or 0xFFFF, two lengths of one instruction less
  // than 2 apart, a budget that is not positive, or a negative guard.
  // Arming again (or disarming) ends neither a running quiet guard nor a
  // recovery still due: only a clean recovery read ends recovery.
  bool armBoundedSyncRead(const BoundedReadConfig& config) override;
  // Back to the SDK path.
  void disarmBoundedSyncRead() override;
  // From /sys/bus/usb-serial/devices/<tty>/latency_timer; none for a
  // device without one (a pseudo-terminal, for example).
  std::optional<int> latencyTimerMs() const override;
  bool boundedSyncReadArmed() const { return bounded_ != nullptr; }
  const BoundedReadStats& boundedReadStats() const { return stats_; }
  // Whether the next bounded read is a recovery read.
  bool inRecovery() const { return recovery_; }
  // Whether a quiet guard holds transmissions, and until when.
  bool quietGuardPending() const { return quiet_pending_; }
  std::chrono::steady_clock::time_point quietUntil() const {
    return quiet_until_;
  }
  // When the quiet guard last released a held transmission (none before:
  // the epoch).
  std::chrono::steady_clock::time_point lastGuardRelease() const {
    return last_guard_release_;
  }

 private:
  struct Bounded;
  struct BoundedWindowState;

  // Before every transmission: waits out a pending quiet guard (then
  // discards received input) and checks the inhibit. False: send nothing.
  bool clearToTransmit();
  bool transmitInhibited() const {
    return inhibit_ != nullptr && inhibit_->load();
  }
  IoResult inhibitedResult();
  IoResult boundedRead(BoundedWindowState& window,
                       std::vector<std::uint8_t>& out);

  std::string device_;
  dynamixel::PortHandler* port_;
  dynamixel::PacketHandler* packet_;
  std::vector<std::uint8_t> sync_write_param_;
  SyncReadTiming last_sync_read_timing_;
  bool open_ = false;
  std::unique_ptr<Bounded> bounded_;
  BoundedReadStats stats_;
  bool recovery_ = false;
  bool quiet_pending_ = false;
  std::chrono::steady_clock::time_point quiet_until_{};
  std::chrono::steady_clock::time_point last_guard_release_{};
  const std::atomic<bool>* inhibit_ = nullptr;
};

}  // namespace rtctrl::dxl
