#pragma once

#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rtctrl/emu/completion_cadence.hpp"
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
  // The start address of a single Read or Write, or of a grouped
  // instruction (Sync Read, Fast Sync Read, Sync Write); a grouped one's
  // listed IDs, in order; and a Sync Write's data bytes, each listed servo's
  // in turn (its length per servo is data.size() / ids.size(); data_length
  // stays 0). 0 and empty otherwise.
  std::uint16_t addr = 0;
  std::vector<std::uint8_t> ids;
  std::vector<std::uint8_t> data;
  // When poll() extracted it: at or after its transmission.
  std::chrono::steady_clock::time_point at{};
};

// One reply held for an adapter cadence (setCadence), with its release.
// Times are steady_clock nanoseconds (CLOCK_MONOTONIC on Linux).
struct CadenceRecord {
  std::int64_t arrival_ns = 0;  // its request's Arrival::at
  std::uint8_t instruction = 0;  // its request's
  std::uint8_t id = 0;
  std::uint16_t data_length = 0;  // a grouped read's, else 0
  std::size_t bytes = 0;          // the reply's bytes, as held
  CadenceRelease release;
  // When it was due: the release, or the previous reply's due time if
  // that is later (replies go in order).
  std::int64_t due_ns = 0;
  // When the pty had accepted the last of its bytes (-1: it has not; see
  // written_bytes and write_error).
  std::int64_t written_ns = -1;
  std::size_t written_bytes = 0;  // of `bytes`, accepted by the pty
  // 0, or why its write failed: the failed write(2)'s errno, or -1 for a
  // write that accepted nothing and reported no error.
  int write_error = 0;
};

// The held reply whose write failed (PtyBus::cadenceWriteFailure).
struct CadenceWriteFailure {
  std::uint64_t reply = 0;  // its place among the held replies, from 0
  std::int64_t record = -1;  // its CadenceRecord (-1: beyond the record)
  std::size_t bytes = 0;
  std::size_t accepted = 0;  // bytes the pty accepted before the failure
  int error = 0;             // as CadenceRecord::write_error
};

// A scripted outcome of one write(2) call for held replies (fault tests,
// PtyBus::scriptCadenceWrites): error != 0 fails the call (-1, errno set
// to error) with nothing written; otherwise at most `accept` bytes are
// written for real, accept 0 making the call return 0.
struct CadenceWrite {
  std::size_t accept = std::numeric_limits<std::size_t>::max();
  int error = 0;
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
  // kDelay faults, or every reply held for a cadence), or none.
  std::optional<std::chrono::steady_clock::time_point> nextReplyDue() const;

  // The pseudo-terminal's master, for readiness waits (poll/ppoll) only:
  // reading or writing it bypasses the emulator.
  int masterFd() const { return master_fd_; }

  // Faults for the next grouped reads (sync or fast), one each, in order;
  // later ones reply normally. Appends to any faults still pending.
  void scriptSyncReadFaults(const std::vector<ReplyFault>& faults);
  // Every instruction received so far, in order.
  std::vector<Arrival> arrivals() const;

  // Opt-in (dxl_emu --cadence): from now on every reply is held and sent
  // when cadence (emu::CompletionCadence, its origin at `origin`) releases
  // it, in order, each at or after the one before; a scripted kDelay fault
  // adds its delay to the reply's ready time. Without it (the default)
  // replies go as before. The first record_capacity replies are recorded
  // (cadenceRecords); later ones are counted only. Call before serving,
  // from the thread that serves.
  //
  // A held reply is written in full or the run fails. After a partial
  // write the rest is written at once, and an interrupted write (EINTR) is
  // retried at once, at most kMaxWriteInterrupts times in a row. Any other
  // outcome (EAGAIN: the pty is full, its client not reading; a hard
  // error; a write that accepts nothing) fails the run explicitly: the
  // reply's record keeps the bytes accepted and the error, it and every
  // later reply stay unsent (no reply is written after an incomplete one),
  // and from then on serviceIo() and poll() return !ok without reading or
  // writing. (Without a cadence, writes are as before: what a failed write
  // leaves is dropped.)
  void setCadence(const CadenceConfig& cadence,
                  std::chrono::steady_clock::time_point origin,
                  std::size_t record_capacity);
  bool cadenceActive() const { return cadence_ != nullptr; }
  const CompletionCadence* cadence() const { return cadence_.get(); }
  const std::vector<CadenceRecord>& cadenceRecords() const {
    return cadence_records_;
  }
  std::uint64_t cadenceReplies() const { return cadence_replies_; }
  std::uint64_t cadenceRecordsDropped() const {
    return cadence_records_dropped_;
  }
  // Held replies the pty accepted in full, and held replies not (yet)
  // written in full: cadenceReplies() is their sum.
  std::uint64_t cadenceWritten() const { return cadence_written_; }
  std::size_t cadencePending() const {
    return cadence_ ? pending_.size() : 0;
  }
  // The write that failed the run, if one has.
  const std::optional<CadenceWriteFailure>& cadenceWriteFailure() const {
    return cadence_failure_;
  }
  static constexpr int kMaxWriteInterrupts = 3;
  // Fault tests: outcomes for the next write(2) calls of held replies, one
  // each, in order; later calls write for real. Appends to any pending.
  void scriptCadenceWrites(const std::vector<CadenceWrite>& writes);

 private:
  struct Pending {
    std::chrono::steady_clock::time_point due;
    std::vector<std::uint8_t> bytes;
    std::size_t record = kNoRecord;  // its CadenceRecord, if any
    std::uint64_t reply = 0;         // held replies: its place, from 0
  };
  static constexpr std::size_t kNoRecord =
      std::numeric_limits<std::size_t>::max();

  void handleFrame(std::uint8_t id, std::uint8_t instruction,
                   const std::vector<std::uint8_t>& params);
  std::vector<std::uint8_t> encodeStatus(
      std::uint8_t id, std::uint8_t error,
      const std::vector<std::uint8_t>& params) const;
  void sendStatus(std::uint8_t id, std::uint8_t error,
                  const std::vector<std::uint8_t>& params);
  void writeBytes(const std::vector<std::uint8_t>& bytes);
  void sendSyncReadReplies(std::vector<std::uint8_t> replies);
  // False: a held reply's write failed (cadence_failure_ set).
  bool sendDueReplies();
  // Cadence mode: all of p's bytes, or the failure recorded (false).
  bool writeHeld(const Pending& p);
  // One write(2) of held bytes, or its scripted outcome.
  ssize_t writeHeldOnce(const std::uint8_t* data, std::size_t size);
  void extractFrames();
  // Cadence mode: hold bytes for the current request's release, its ready
  // time extra_delay later.
  void holdForCadence(std::vector<std::uint8_t> bytes,
                      std::chrono::nanoseconds extra_delay);

  MotorBus& bus_;
  int master_fd_ = -1;
  std::string slave_path_;
  std::vector<std::uint8_t> rx_;
  std::vector<Pending> pending_;
  // guards faults_, arrivals_ and write_script_
  mutable std::mutex script_mutex_;
  std::deque<ReplyFault> faults_;
  std::deque<CadenceWrite> write_script_;
  std::vector<Arrival> arrivals_;
  // Cadence mode only (null by default).
  std::unique_ptr<CompletionCadence> cadence_;
  Arrival current_;  // the request being answered
  std::vector<CadenceRecord> cadence_records_;
  std::size_t cadence_record_capacity_ = 0;
  std::uint64_t cadence_replies_ = 0;
  std::uint64_t cadence_records_dropped_ = 0;
  std::uint64_t cadence_written_ = 0;
  std::optional<CadenceWriteFailure> cadence_failure_;
  std::chrono::steady_clock::time_point last_due_{};
};

// Protocol 2.0 helpers, exposed for tests.
std::uint16_t dxlCrc16(const std::uint8_t* data, std::size_t size);
void dxlAddStuffing(std::vector<std::uint8_t>& payload);
void dxlRemoveStuffing(std::vector<std::uint8_t>& payload);

}  // namespace rtctrl::emu
