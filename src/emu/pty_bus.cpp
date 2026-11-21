#include "rtctrl/emu/pty_bus.hpp"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <utility>

#include "rtctrl/dxl/status_parser.hpp"

namespace rtctrl::emu {

namespace {

constexpr std::uint8_t kInstPing = 0x01;
constexpr std::uint8_t kInstRead = 0x02;
constexpr std::uint8_t kInstWrite = 0x03;
constexpr std::uint8_t kInstStatus = 0x55;
constexpr std::uint8_t kInstSyncRead = 0x82;
constexpr std::uint8_t kInstSyncWrite = 0x83;
constexpr std::uint8_t kInstFastSyncRead = 0x8A;
constexpr std::uint8_t kBroadcastId = 0xFE;

std::int64_t nsOf(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             t.time_since_epoch())
      .count();
}

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
  if (!serviceIo().ok) return false;
  bus_.tick(dt);
  return true;
}

PtyBus::Serviced PtyBus::serviceIo(std::size_t max_read_bytes) {
  Serviced result;
  sendDueReplies();
  std::uint8_t buf[512];
  while (true) {
    const std::size_t room = max_read_bytes - result.bytes_read;
    if (room == 0) {
      result.limited = true;
      break;
    }
    const ssize_t n =
        ::read(master_fd_, buf, std::min<std::size_t>(sizeof(buf), room));
    if (n > 0) {
      rx_.insert(rx_.end(), buf, buf + n);
      result.bytes_read += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    if (n < 0 && errno == EIO) break;  // no client attached (or detached)
    if (n == 0) break;
    if (n < 0) {
      result.ok = false;
      return result;
    }
  }
  extractFrames();
  sendDueReplies();
  return result;
}

std::optional<std::chrono::steady_clock::time_point> PtyBus::nextReplyDue()
    const {
  if (pending_.empty()) return std::nullopt;
  return pending_.front().due;  // kept in due order
}

void PtyBus::scriptSyncReadFaults(const std::vector<ReplyFault>& faults) {
  const std::lock_guard<std::mutex> lock(script_mutex_);
  faults_.insert(faults_.end(), faults.begin(), faults.end());
}

std::vector<Arrival> PtyBus::arrivals() const {
  const std::lock_guard<std::mutex> lock(script_mutex_);
  return arrivals_;
}

void PtyBus::setCadence(const CadenceConfig& cadence,
                        std::chrono::steady_clock::time_point origin,
                        std::size_t record_capacity) {
  cadence_ = std::make_unique<CompletionCadence>(cadence, nsOf(origin));
  cadence_records_.clear();
  cadence_records_.reserve(record_capacity);
  cadence_record_capacity_ = record_capacity;
  cadence_replies_ = 0;
  cadence_records_dropped_ = 0;
  last_due_ = {};
}

void PtyBus::holdForCadence(std::vector<std::uint8_t> bytes,
                            std::chrono::nanoseconds extra_delay) {
  if (bytes.empty()) return;  // nothing to send, as writeBytes would
  const std::int64_t arrival = nsOf(current_.at);
  const CadenceRelease release =
      cadence_->release(arrival + extra_delay.count(), bytes.size());
  auto due = std::chrono::steady_clock::time_point(
      std::chrono::nanoseconds(release.release_ns));
  if (due < last_due_) due = last_due_;  // in order
  last_due_ = due;
  ++cadence_replies_;
  std::size_t record = kNoRecord;
  if (cadence_records_.size() < cadence_record_capacity_) {
    CadenceRecord r;
    r.arrival_ns = arrival;
    r.instruction = current_.instruction;
    r.id = current_.id;
    r.data_length = current_.data_length;
    r.bytes = bytes.size();
    r.release = release;
    r.due_ns = nsOf(due);
    record = cadence_records_.size();
    cadence_records_.push_back(r);
  } else {
    ++cadence_records_dropped_;
  }
  Pending p{due, std::move(bytes), record};
  const auto at = std::upper_bound(
      pending_.begin(), pending_.end(), p.due,
      [](const auto& d, const Pending& q) { return d < q.due; });
  pending_.insert(at, std::move(p));
}

void PtyBus::sendDueReplies() {
  const auto now = std::chrono::steady_clock::now();
  // In order of arrival: an earlier delayed reply goes out first.
  while (!pending_.empty() && pending_.front().due <= now) {
    writeBytes(pending_.front().bytes);
    if (pending_.front().record != kNoRecord) {
      cadence_records_[pending_.front().record].written_ns =
          nsOf(std::chrono::steady_clock::now());
    }
    pending_.erase(pending_.begin());
  }
}

void PtyBus::sendSyncReadReplies(std::vector<std::uint8_t> replies) {
  ReplyFault fault;
  bool scripted = false;
  {
    const std::lock_guard<std::mutex> lock(script_mutex_);
    if (!faults_.empty()) {
      fault = faults_.front();
      faults_.pop_front();
      scripted = true;
    }
  }
  if (!scripted) {
    if (cadence_) {
      holdForCadence(std::move(replies), std::chrono::nanoseconds::zero());
      return;
    }
    writeBytes(replies);
    return;
  }
  switch (fault.kind) {
    case ReplyFault::Kind::kDelay: {
      if (cadence_) {
        holdForCadence(std::move(replies), fault.delay);
        break;
      }
      Pending p{std::chrono::steady_clock::now() + fault.delay,
                std::move(replies)};
      const auto at = std::upper_bound(
          pending_.begin(), pending_.end(), p.due,
          [](const auto& due, const Pending& q) { return due < q.due; });
      pending_.insert(at, std::move(p));
      break;
    }
    case ReplyFault::Kind::kTruncate:
      replies.resize(std::min(fault.keep, replies.size()));
      if (cadence_) {
        holdForCadence(std::move(replies), std::chrono::nanoseconds::zero());
        break;
      }
      writeBytes(replies);
      break;
    case ReplyFault::Kind::kGarbage: {
      std::vector<std::uint8_t> bytes = fault.garbage;
      bytes.insert(bytes.end(), replies.begin(), replies.end());
      if (cadence_) {
        holdForCadence(std::move(bytes), std::chrono::nanoseconds::zero());
        break;
      }
      writeBytes(bytes);
      break;
    }
    case ReplyFault::Kind::kDrop:
      break;
  }
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
        Arrival arrival;
        arrival.instruction = instruction;
        arrival.id = id;
        arrival.at = std::chrono::steady_clock::now();
        if ((instruction == kInstSyncRead ||
             instruction == kInstFastSyncRead) &&
            body.size() >= 4) {
          arrival.data_length =
              static_cast<std::uint16_t>(body[2] | (body[3] << 8));
        }
        {
          const std::lock_guard<std::mutex> lock(script_mutex_);
          arrivals_.push_back(arrival);
        }
        if (cadence_) current_ = arrival;
        handleFrame(id, instruction, body);
      }
    }
    rx_.erase(rx_.begin(), rx_.begin() + total);
  }
}

std::vector<std::uint8_t> PtyBus::encodeStatus(
    std::uint8_t id, std::uint8_t error,
    const std::vector<std::uint8_t>& params) const {
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
  return frame;
}

void PtyBus::sendStatus(std::uint8_t id, std::uint8_t error,
                        const std::vector<std::uint8_t>& params) {
  if (cadence_) {
    holdForCadence(encodeStatus(id, error, params),
                   std::chrono::nanoseconds::zero());
    return;
  }
  writeBytes(encodeStatus(id, error, params));
}

void PtyBus::writeBytes(const std::vector<std::uint8_t>& bytes) {
  std::size_t off = 0;
  while (off < bytes.size()) {
    const ssize_t n = ::write(master_fd_, bytes.data() + off,
                              bytes.size() - off);
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
      // one status packet per listed id, in order, sent together (or as
      // a scripted fault says)
      std::vector<std::uint8_t> replies;
      for (std::size_t i = 4; i < params.size(); ++i) {
        MotorEmulator* motor = bus_.find(params[i]);
        if (motor == nullptr) continue;
        motor->touch();
        std::vector<std::uint8_t> data(len, 0);
        const dxl::IoResult r = motor->read(addr, data.data(), len);
        const std::vector<std::uint8_t> status =
            encodeStatus(motor->id(), r.error, data);
        replies.insert(replies.end(), status.begin(), status.end());
      }
      sendSyncReadReplies(std::move(replies));
      break;
    }
    case kInstFastSyncRead: {
      if (params.size() < 4) break;
      const std::uint16_t addr =
          static_cast<std::uint16_t>(params[0] | (params[1] << 8));
      const std::uint16_t len =
          static_cast<std::uint16_t>(params[2] | (params[3] << 8));
      // One status packet for all listed ids, sent together (or as a
      // scripted fault says). The servos build it in turn, so a listed
      // servo that is absent or predates the instruction (firmware below
      // v45) breaks the chain: nothing is sent, and the host times out.
      std::vector<std::uint8_t> ids(params.begin() + 4, params.end());
      std::vector<std::uint8_t> errors;
      std::vector<std::uint8_t> data(ids.size() * len, 0);
      bool chain = !ids.empty();
      for (std::size_t i = 0; i < ids.size() && chain; ++i) {
        MotorEmulator* motor = bus_.find(ids[i]);
        chain = motor != nullptr &&
                motor->peek(dxl::reg::kFirmwareVersion) >=
                    dxl::kMinFirmwareFastSyncRead;
        if (!chain) break;
        motor->touch();
        errors.push_back(motor->read(addr, &data[i * len], len).error);
      }
      if (chain) {
        sendSyncReadReplies(
            dxl::encodeFastSyncStatusPacket(ids, errors, data, len));
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
