#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rtctrl::dxl {

// CRC-16 of Dynamixel Protocol 2.0: polynomial 0x8005, initial value 0, not
// reflected, no final XOR (the SDK's updateCRC). Pass an earlier result as
// `crc` to continue over a further block.
std::uint16_t crc16(const std::uint8_t* data, std::size_t size,
                    std::uint16_t crc = 0);

// One status packet as a servo transmits it: header, the reserved byte,
// the ID, the length, instruction 0x55, the error byte and `size` data
// bytes with byte stuffing applied, then the CRC. For tests and the
// emulator; it allocates.
std::vector<std::uint8_t> encodeStatusPacket(std::uint8_t id,
                                             std::uint8_t error,
                                             const std::uint8_t* data,
                                             std::size_t size);

// Strict parser of the status packets that answer one grouped read: one
// packet per servo, in the requested order, each carrying exactly
// param_len data bytes. It checks, in wire order:
//  - the header FF FF FD and the reserved byte 00;
//  - the ID, which must be the next servo requested;
//  - the length field, within the range that param_len and the most byte
//    stuffing it allows can give;
//  - instruction 0x55 (status);
//  - byte stuffing: every FF FF FD in the body is followed by a stuffing
//    FD, and the unstuffed body is exactly the instruction, the error byte
//    and param_len data bytes;
//  - the CRC over the packet as transmitted (stuffing included);
//  - nothing after the last expected packet.
// Nothing is skipped or resynchronized. Garbage before a header, a reply
// from another servo, a late reply of another length, and a truncated or
// corrupted packet all fail. Each servo's error byte is kept: a nonzero one
// (the Alert bit included) makes a well-formed reply a failed read, which
// the caller tells from a malformed one with anyServoError().
//
// parse() takes everything received so far for the transaction, from its
// first byte, and is cheap to call again as more arrives: it reports a
// fault as soon as the bytes present rule out a valid reply, so a receive
// loop need not wait for its deadline. It allocates nothing (the buffers
// are sized at construction), so a control thread can call it.
class StatusReplyParser {
 public:
  enum class Status : std::uint8_t {
    kIncomplete,  // a valid prefix of the expected replies
    kComplete,    // every reply present and well formed, nothing after
    kMalformed,   // cannot become valid; see fault() and faultOffset()
  };
  enum class Fault : std::uint8_t {
    kNone,
    kHeader,       // not FF FF FD where a packet must start
    kReserved,     // the reserved byte is not 00
    kId,           // not the next servo requested
    kLength,       // the length field is outside the possible range
    kInstruction,  // not a status packet (0x55)
    kStuffing,     // FF FF FD in the body without its stuffing FD
    kParamCount,   // the unstuffed body is not instruction, error, data
    kCrc,          // the CRC does not match
    kExtraBytes,   // bytes after the last expected packet
  };

  // ids: the servos in request order, each 0..252 and none repeated.
  // param_len: data bytes per servo, at least 1. Throws
  // std::invalid_argument otherwise.
  StatusReplyParser(std::vector<std::uint8_t> ids, std::uint16_t param_len);

  Status parse(const std::uint8_t* data, std::size_t size);

  Fault fault() const { return fault_; }
  // Where the fault was found, as an index into the bytes parsed.
  std::size_t faultOffset() const { return fault_offset_; }
  // Packets complete and valid, in order, at the last parse().
  std::size_t packetsParsed() const { return packets_; }

  // After kComplete: param_len unstuffed data bytes per servo in request
  // order, and each servo's error byte.
  const std::vector<std::uint8_t>& params() const { return params_; }
  const std::vector<std::uint8_t>& errors() const { return errors_; }
  bool anyServoError() const;

  const std::vector<std::uint8_t>& ids() const { return ids_; }
  std::uint16_t paramLength() const { return param_len_; }
  // The length field's range for this request: no stuffing, and the most
  // stuffing param_len data bytes can carry.
  std::size_t minLengthField() const { return min_length_; }
  std::size_t maxLengthField() const { return max_length_; }

 private:
  Status fail(Fault fault, std::size_t offset);

  std::vector<std::uint8_t> ids_;
  std::uint16_t param_len_;
  std::size_t min_length_;
  std::size_t max_length_;
  std::vector<std::uint8_t> params_;
  std::vector<std::uint8_t> errors_;
  Fault fault_ = Fault::kNone;
  std::size_t fault_offset_ = 0;
  std::size_t packets_ = 0;
};

}  // namespace rtctrl::dxl
