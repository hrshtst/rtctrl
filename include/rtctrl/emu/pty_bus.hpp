#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rtctrl/emu/motor_emulator.hpp"

namespace rtctrl::emu {

// A fault applied to the replies of one grouped read, Sync Read or Fast
// Sync Read (fault tests of the bounded read).
struct ReplyFault {
  enum class Kind {
    kDelay,     // send the replies `delay` after the instruction arrived
    kTruncate,  // send only the first `keep` bytes of the replies
    kGarbage,   // send `garbage` first, then the replies
    kDrop,      // send nothing
  };
  Kind kind = Kind::kDelay;
  std::chrono::microseconds delay{0};
  std::size_t keep = 0;
  std::vector<std::uint8_t> garbage;
};

// An instruction packet as the emulator received it.
struct Arrival {
  std::uint8_t instruction = 0;
  std::uint8_t id = 0;
  std::uint16_t data_length = 0;  // a grouped read's data length, else 0
  // When poll() extracted it: at or after its transmission.
  std::chrono::steady_clock::time_point at{};
};

// Serves a MotorBus over a pseudo-terminal speaking Dynamixel Protocol
// 2.0 (header/CRC-16/byte stuffing), so the UNMODIFIED DynamixelSDK
// PortHandler connects to the slave path like a real serial adapter.
// Handles Ping (incl. broadcast), Read, Write, SyncRead, FastSyncRead
// (firmware v45 and later), SyncWrite.
//
// Single-threaded ownership: whoever calls poll() (or serviceIo()) owns
// the MotorBus; clients on the other end of the pty interact only through
// the fd. The fault script and the arrival record are the exception:
// other threads may use them while poll() runs.
class PtyBus {
 public:
  // What one serviceIo() call did.
  struct Serviced {
    bool ok = true;              // false: an unrecoverable pty error
    std::size_t bytes_read = 0;  // input bytes read from the pty
    bool limited = false;        // stopped at max_read_bytes (input may remain)
  };
  static constexpr std::size_t kNoReadLimit =
      std::numeric_limits<std::size_t>::max();

  explicit PtyBus(MotorBus& bus);
  ~PtyBus();

  PtyBus(const PtyBus&) = delete;
  PtyBus& operator=(const PtyBus&) = delete;

  const std::string& slavePath() const { return slave_path_; }

  // Service pending frames, send delayed replies that are due, then
  // advance simulated time by dt. Returns false on an unrecoverable pty
  // error. Exactly serviceIo() (unbounded) then MotorBus::tick(dt).
  bool poll(double dt);

  // The I/O half of poll(), without advancing simulated time: send the
  // delayed replies that are due, read the input (at most max_read_bytes;
  // by default until the pty has none), answer every complete frame read
  // so far (a partial frame waits for its rest), then send the delayed
  // replies that are due again.
  Serviced serviceIo(std::size_t max_read_bytes = kNoReadLimit);

  // The earliest due time of a delayed reply still to be sent (scripted
  // kDelay faults only), or none.
  std::optional<std::chrono::steady_clock::time_point> nextReplyDue() const;

  // The pseudo-terminal's master, for readiness waits (poll/ppoll) only:
  // reading or writing it bypasses the emulator.
  int masterFd() const { return master_fd_; }

  // Faults for the next grouped reads (sync or fast), one each, in order;
  // later ones reply normally. Appends to any faults still pending.
  void scriptSyncReadFaults(const std::vector<ReplyFault>& faults);
  // Every instruction received so far, in order.
  std::vector<Arrival> arrivals() const;

 private:
  struct Pending {
    std::chrono::steady_clock::time_point due;
    std::vector<std::uint8_t> bytes;
  };

  void handleFrame(std::uint8_t id, std::uint8_t instruction,
                   const std::vector<std::uint8_t>& params);
  std::vector<std::uint8_t> encodeStatus(
      std::uint8_t id, std::uint8_t error,
      const std::vector<std::uint8_t>& params) const;
  void sendStatus(std::uint8_t id, std::uint8_t error,
                  const std::vector<std::uint8_t>& params);
  void writeBytes(const std::vector<std::uint8_t>& bytes);
  void sendSyncReadReplies(std::vector<std::uint8_t> replies);
  void sendDueReplies();
  void extractFrames();

  MotorBus& bus_;
  int master_fd_ = -1;
  std::string slave_path_;
  std::vector<std::uint8_t> rx_;
  std::vector<Pending> pending_;
  mutable std::mutex script_mutex_;  // guards faults_ and arrivals_
  std::deque<ReplyFault> faults_;
  std::vector<Arrival> arrivals_;
};

// Protocol 2.0 helpers, exposed for tests.
std::uint16_t dxlCrc16(const std::uint8_t* data, std::size_t size);
void dxlAddStuffing(std::vector<std::uint8_t>& payload);
void dxlRemoveStuffing(std::vector<std::uint8_t>& payload);

}  // namespace rtctrl::emu
