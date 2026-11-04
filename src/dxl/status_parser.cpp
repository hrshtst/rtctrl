#include "rtctrl/dxl/status_parser.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>

namespace rtctrl::dxl {

namespace {

constexpr std::array<std::uint16_t, 256> makeCrcTable() {
  std::array<std::uint16_t, 256> table{};
  for (std::size_t i = 0; i < table.size(); ++i) {
    auto crc = static_cast<std::uint16_t>(i << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) != 0
                ? static_cast<std::uint16_t>((crc << 1) ^ 0x8005)
                : static_cast<std::uint16_t>(crc << 1);
    }
    table[i] = crc;
  }
  return table;
}

constexpr std::array<std::uint16_t, 256> kCrcTable = makeCrcTable();

constexpr std::uint8_t kStatusInstruction = 0x55;
constexpr std::array<std::uint8_t, 4> kHeader = {0xFF, 0xFF, 0xFD, 0x00};
constexpr std::size_t kIdOffset = 4;
constexpr std::size_t kLengthOffset = 5;
constexpr std::size_t kBodyOffset = 7;  // the instruction
constexpr std::uint8_t kMaxId = 252;    // 253..255 are reserved or broadcast
constexpr std::uint8_t kBroadcastId = 0xFE;
// A Fast Sync Read reply: header, ID and length (7), the instruction, then
// per servo its error byte, ID, data and CRC.
constexpr std::size_t kFastSectionOverhead = 4;

std::size_t fastLengthField(std::size_t n, std::size_t len) {
  return 1 + n * (len + kFastSectionOverhead);
}

}  // namespace

std::uint16_t crc16(const std::uint8_t* data, std::size_t size,
                    std::uint16_t crc) {
  for (std::size_t j = 0; j < size; ++j) {
    const auto i = static_cast<std::uint8_t>((crc >> 8) ^ data[j]);
    crc = static_cast<std::uint16_t>((crc << 8) ^ kCrcTable[i]);
  }
  return crc;
}

std::vector<std::uint8_t> encodeStatusPacket(std::uint8_t id,
                                             std::uint8_t error,
                                             const std::uint8_t* data,
                                             std::size_t size) {
  std::vector<std::uint8_t> body{kStatusInstruction, error};
  body.insert(body.end(), data, data + size);
  // Byte stuffing: a stuffing FD follows every FF FF FD of the body.
  std::vector<std::uint8_t> stuffed;
  stuffed.reserve(body.size() + body.size() / 3);
  int run = 0;  // consecutive FF just sent
  for (const std::uint8_t b : body) {
    stuffed.push_back(b);
    if (run >= 2 && b == 0xFD) {
      stuffed.push_back(0xFD);
      run = 0;
    } else {
      run = b == 0xFF ? run + 1 : 0;
    }
  }
  const std::size_t length = stuffed.size() + 2;  // + the CRC
  std::vector<std::uint8_t> packet(kHeader.begin(), kHeader.end());
  packet.push_back(id);
  packet.push_back(static_cast<std::uint8_t>(length & 0xFF));
  packet.push_back(static_cast<std::uint8_t>(length >> 8));
  packet.insert(packet.end(), stuffed.begin(), stuffed.end());
  const std::uint16_t crc = crc16(packet.data(), packet.size());
  packet.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  packet.push_back(static_cast<std::uint8_t>(crc >> 8));
  return packet;
}

std::vector<std::uint8_t> encodeFastSyncStatusPacket(
    const std::vector<std::uint8_t>& ids,
    const std::vector<std::uint8_t>& errors,
    const std::vector<std::uint8_t>& data, std::size_t len) {
  if (errors.size() != ids.size() || data.size() != ids.size() * len) {
    throw std::invalid_argument(
        "encodeFastSyncStatusPacket: one error byte and len data bytes per "
        "servo");
  }
  const std::size_t length = fastLengthField(ids.size(), len);
  std::vector<std::uint8_t> packet(kHeader.begin(), kHeader.end());
  packet.push_back(kBroadcastId);
  packet.push_back(static_cast<std::uint8_t>(length & 0xFF));
  packet.push_back(static_cast<std::uint8_t>(length >> 8));
  packet.push_back(kStatusInstruction);
  for (std::size_t k = 0; k < ids.size(); ++k) {
    packet.push_back(errors[k]);
    packet.push_back(ids[k]);
    packet.insert(packet.end(), data.begin() + static_cast<std::ptrdiff_t>(k * len),
                  data.begin() + static_cast<std::ptrdiff_t>((k + 1) * len));
    const std::uint16_t crc = crc16(packet.data(), packet.size());
    packet.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    packet.push_back(static_cast<std::uint8_t>(crc >> 8));
  }
  return packet;
}

StatusReplyParser::StatusReplyParser(std::vector<std::uint8_t> ids,
                                     std::uint16_t param_len,
                                     GroupedRead instruction)
    : ids_(std::move(ids)),
      param_len_(param_len),
      instruction_(instruction),
      // Sync Read, per packet: instruction, error byte, the data and the
      // CRC; FF FF FD can only form inside the data (the instruction is
      // 0x55 and no error byte is FF), at most once per three data bytes.
      // Fast Sync Read: one exact length, never stuffed.
      min_length_(instruction == GroupedRead::kFastSync
                      ? fastLengthField(ids_.size(), param_len)
                      : static_cast<std::size_t>(param_len) + 4),
      max_length_(instruction == GroupedRead::kFastSync
                      ? min_length_
                      : static_cast<std::size_t>(param_len) + 4 +
                            param_len / 3) {
  if (instruction_ != GroupedRead::kSync &&
      instruction_ != GroupedRead::kFastSync) {
    throw std::invalid_argument("StatusReplyParser: not a grouped read");
  }
  if (ids_.empty()) {
    throw std::invalid_argument("StatusReplyParser: no servo requested");
  }
  if (param_len_ == 0) {
    throw std::invalid_argument("StatusReplyParser: no data requested");
  }
  std::array<bool, 256> seen{};
  for (const std::uint8_t id : ids_) {
    if (id > kMaxId) {
      throw std::invalid_argument("StatusReplyParser: id " +
                                  std::to_string(id) + " cannot reply");
    }
    if (seen[id]) {
      throw std::invalid_argument("StatusReplyParser: id " +
                                  std::to_string(id) + " requested twice");
    }
    seen[id] = true;
  }
  params_.assign(ids_.size() * param_len_, 0);
  errors_.assign(ids_.size(), 0);
}

std::size_t StatusReplyParser::maxReplyBytes() const {
  if (instruction_ == GroupedRead::kFastSync) return kBodyOffset + max_length_;
  return ids_.size() * (kBodyOffset + max_length_);
}

StatusReplyParser::Status StatusReplyParser::parse(const std::uint8_t* data,
                                                   std::size_t size) {
  fault_ = Fault::kNone;
  fault_offset_ = 0;
  packets_ = 0;
  return instruction_ == GroupedRead::kFastSync ? parseFastSync(data, size)
                                                : parseSync(data, size);
}

StatusReplyParser::Status StatusReplyParser::parseFastSync(
    const std::uint8_t* data, std::size_t size) {
  for (std::size_t i = 0; i < kHeader.size(); ++i) {
    if (i >= size) return Status::kIncomplete;
    if (data[i] != kHeader[i]) {
      return fail(i + 1 < kHeader.size() ? Fault::kHeader : Fault::kReserved,
                  i);
    }
  }
  if (kIdOffset >= size) return Status::kIncomplete;
  if (data[kIdOffset] != kBroadcastId) return fail(Fault::kId, kIdOffset);
  if (kLengthOffset + 1 >= size) return Status::kIncomplete;
  const std::size_t length =
      static_cast<std::size_t>(data[kLengthOffset]) |
      (static_cast<std::size_t>(data[kLengthOffset + 1]) << 8);
  if (length != min_length_) return fail(Fault::kLength, kLengthOffset);
  if (kBodyOffset >= size) return Status::kIncomplete;
  if (data[kBodyOffset] != kStatusInstruction) {
    return fail(Fault::kInstruction, kBodyOffset);
  }
  // Sections: error byte, ID, the data and a CRC each.
  std::size_t pos = kBodyOffset + 1;
  const std::size_t section = param_len_ + kFastSectionOverhead;
  for (std::size_t k = 0; k < ids_.size(); ++k, pos += section) {
    if (pos >= size) return Status::kIncomplete;
    errors_[k] = data[pos];
    if (pos + 1 >= size) return Status::kIncomplete;
    if (data[pos + 1] != ids_[k]) return fail(Fault::kId, pos + 1);
    const std::size_t first = pos + 2;
    const std::size_t present = std::min(size, first + param_len_) - first;
    std::copy_n(data + first, present,
                params_.begin() + static_cast<std::ptrdiff_t>(k * param_len_));
    const std::size_t crc_at = first + param_len_;
    if (crc_at + 2 > size) return Status::kIncomplete;
    if (k + 1 == ids_.size()) {
      const std::uint16_t crc = crc16(data, crc_at);
      const auto sent = static_cast<std::uint16_t>(
          data[crc_at] | (static_cast<std::uint16_t>(data[crc_at + 1]) << 8));
      if (crc != sent) return fail(Fault::kCrc, crc_at);
    }
    ++packets_;
  }
  if (size > pos) return fail(Fault::kExtraBytes, pos);
  return Status::kComplete;
}

StatusReplyParser::Status StatusReplyParser::parseSync(const std::uint8_t* data,
                                                       std::size_t size) {
  std::size_t pos = 0;  // where the packet being parsed starts
  for (std::size_t k = 0; k < ids_.size(); ++k) {
    for (std::size_t i = 0; i < kHeader.size(); ++i) {
      if (pos + i >= size) return Status::kIncomplete;
      if (data[pos + i] != kHeader[i]) {
        return fail(i + 1 < kHeader.size() ? Fault::kHeader : Fault::kReserved,
                    pos + i);
      }
    }
    if (pos + kIdOffset >= size) return Status::kIncomplete;
    if (data[pos + kIdOffset] != ids_[k]) {
      return fail(Fault::kId, pos + kIdOffset);
    }
    if (pos + kLengthOffset + 1 >= size) return Status::kIncomplete;
    const std::size_t length =
        static_cast<std::size_t>(data[pos + kLengthOffset]) |
        (static_cast<std::size_t>(data[pos + kLengthOffset + 1]) << 8);
    if (length < min_length_ || length > max_length_) {
      return fail(Fault::kLength, pos + kLengthOffset);
    }

    // The body (instruction, error byte, data; stuffed as sent) runs up to
    // the CRC. Unstuff whatever of it has arrived, so a fault shows early.
    const std::size_t body = pos + kBodyOffset;
    const std::size_t body_end = body + length - 2;
    std::size_t emitted = 0;  // unstuffed bytes: instruction, error, data
    int run = 0;              // consecutive FF just received
    bool stuffing_due = false;
    for (std::size_t i = body; i < std::min(body_end, size); ++i) {
      const std::uint8_t b = data[i];
      if (stuffing_due) {
        if (b != 0xFD) return fail(Fault::kStuffing, i);
        stuffing_due = false;
        run = 0;
        continue;
      }
      if (emitted == 0) {
        if (b != kStatusInstruction) return fail(Fault::kInstruction, i);
      } else if (emitted == 1) {
        errors_[k] = b;
      } else if (emitted - 2 < param_len_) {
        params_[k * param_len_ + (emitted - 2)] = b;
      } else {
        return fail(Fault::kParamCount, i);
      }
      ++emitted;
      if (run >= 2 && b == 0xFD) {
        stuffing_due = true;
        run = 0;
      } else {
        run = b == 0xFF ? run + 1 : 0;
      }
    }
    if (body_end > size) return Status::kIncomplete;
    if (stuffing_due) return fail(Fault::kStuffing, body_end);
    if (emitted != static_cast<std::size_t>(param_len_) + 2) {
      return fail(Fault::kParamCount, body_end);
    }
    if (body_end + 2 > size) return Status::kIncomplete;
    const std::uint16_t crc = crc16(data + pos, body_end - pos);
    const auto sent = static_cast<std::uint16_t>(
        data[body_end] | (static_cast<std::uint16_t>(data[body_end + 1]) << 8));
    if (crc != sent) return fail(Fault::kCrc, body_end);
    pos = body_end + 2;
    ++packets_;
  }
  if (size > pos) return fail(Fault::kExtraBytes, pos);
  return Status::kComplete;
}

bool StatusReplyParser::anyServoError() const {
  return std::any_of(errors_.begin(), errors_.end(),
                     [](std::uint8_t e) { return e != 0; });
}

StatusReplyParser::Status StatusReplyParser::fail(Fault fault,
                                                  std::size_t offset) {
  fault_ = fault;
  fault_offset_ = offset;
  return Status::kMalformed;
}

}  // namespace rtctrl::dxl
