#include "rtctrl/dxl/port.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

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

// The bounded read's deadlines and the quiet guard must not move with the
// wall clock (the SDK's packet timeout uses CLOCK_REALTIME).
using Clock = std::chrono::steady_clock;
static_assert(Clock::is_steady, "the bounded read needs a monotonic clock");

}  // namespace

// One armed window's parsers, allocated once at arming.
struct Port::BoundedWindowState {
  BoundedWindowState(const std::vector<std::uint8_t>& ids,
                     BoundedReadWindow w)
      : window(w),
        normal(ids, w.len, w.instruction),
        recovery(ids, static_cast<std::uint16_t>(w.len + 1), w.instruction) {}

  // Every recovery reply at its longest.
  std::size_t maxReplyBytes() const { return recovery.maxReplyBytes(); }

  BoundedReadWindow window;
  StatusReplyParser normal;
  StatusReplyParser recovery;
};

// What the armed bounded read needs, allocated once at arming.
struct Port::Bounded {
  explicit Bounded(BoundedReadConfig c) : config(std::move(c)) {
    windows.reserve(config.windows.size());
    std::size_t longest = 0;
    for (const BoundedReadWindow& w : config.windows) {
      windows.emplace_back(config.ids, w);
      longest = std::max(longest, windows.back().maxReplyBytes());
    }
    // The longest reply, and one byte to show any excess (the parser
    // reports extra bytes before the buffer fills).
    rx.resize(longest + 1);
  }

  BoundedWindowState* find(GroupedRead instruction, std::uint16_t addr,
                           std::uint16_t len,
                           const std::vector<std::uint8_t>& ids) {
    if (ids != config.ids) return nullptr;
    for (BoundedWindowState& w : windows) {
      if (w.window.instruction == instruction && w.window.addr == addr &&
          w.window.len == len) {
        return &w;
      }
    }
    return nullptr;
  }

  BoundedReadConfig config;
  std::vector<BoundedWindowState> windows;
  std::vector<std::uint8_t> rx;
};

Port::Port(const std::string& device, int baudrate)
    : device_(device),
      port_(checkedPortHandler(device)),
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

bool Port::clearToTransmit() {
  if (quiet_pending_) {  // the SDK path never sets a guard
    if (Clock::now() < quiet_until_) ++stats_.guard_waits;
    // Wait in slices of at most 1 ms, so an inhibit cancels the wait.
    for (Clock::time_point now = Clock::now(); now < quiet_until_;
         now = Clock::now()) {
      if (transmitInhibited()) return false;  // the guard stays pending
      std::this_thread::sleep_until(
          std::min(quiet_until_, now + std::chrono::milliseconds(1)));
    }
    quiet_pending_ = false;
    // Whatever arrived during the guard is a late reply: discard it.
    port_->clearPort();
    last_guard_release_ = Clock::now();
  }
  // Immediately before sending: an inhibit set while this call waited, or
  // at any time before, keeps the bus silent.
  return !transmitInhibited();
}

IoResult Port::inhibitedResult() {
  ++stats_.inhibited;
  IoResult r;
  r.comm = kCommTransmitInhibited;
  return r;
}

bool Port::armBoundedSyncRead(const BoundedReadConfig& config) {
  if (config.ids.empty() || config.windows.empty()) {
    throw std::invalid_argument("Port: a bounded read needs ids and a window");
  }
  for (std::size_t i = 0; i < config.windows.size(); ++i) {
    const std::uint16_t len = config.windows[i].len;
    if (len == 0 || len == 0xFFFF) {
      throw std::invalid_argument(
          "Port: a bounded window needs 1 to 65534 data bytes");
    }
    const GroupedRead instruction = config.windows[i].instruction;
    if (instruction != GroupedRead::kSync &&
        instruction != GroupedRead::kFastSync) {
      throw std::invalid_argument("Port: a bounded window needs a grouped read");
    }
    // A reply does not name its register: only its length tells a late
    // reply from the one awaited, in either epoch (len or len + 1). The
    // two instructions' replies differ in their first packet's ID.
    for (std::size_t j = 0; j < i; ++j) {
      if (config.windows[j].instruction != instruction) continue;
      const int other = config.windows[j].len;
      if (std::abs(static_cast<int>(len) - other) < 2) {
        throw std::invalid_argument(
            "Port: bounded windows of " + std::to_string(other) + " and " +
            std::to_string(len) +
            " bytes cannot be told apart in recovery; lengths must differ "
            "by at least 2");
      }
    }
  }
  if (config.budget <= std::chrono::nanoseconds::zero() ||
      config.guard < std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument(
        "Port: a bounded read needs a positive budget and a guard of at "
        "least zero");
  }
  // A recovery still due stays due: only a clean recovery read ends it.
  bounded_ = std::make_unique<Bounded>(config);
  return true;
}

void Port::disarmBoundedSyncRead() { bounded_.reset(); }

std::optional<int> Port::latencyTimerMs() const {
  std::error_code ec;
  const std::filesystem::path dev = std::filesystem::canonical(device_, ec);
  const std::string tty =
      (ec ? std::filesystem::path(device_) : dev).filename().string();
  std::ifstream in("/sys/bus/usb-serial/devices/" + tty + "/latency_timer");
  int ms = 0;
  if (!(in >> ms)) return std::nullopt;
  return ms;
}

IoResult Port::ping(std::uint8_t id, std::uint16_t* model_number) {
  if (!clearToTransmit()) return inhibitedResult();
  IoResult r;
  std::uint16_t model = 0;
  r.comm = packet_->ping(port_, id, &model, &r.error);
  if (model_number != nullptr) *model_number = model;
  return r;
}

IoResult Port::read(std::uint8_t id, std::uint16_t addr, std::uint8_t* data,
                    std::uint16_t len) {
  if (!clearToTransmit()) return inhibitedResult();
  IoResult r;
  r.comm = packet_->readTxRx(port_, id, addr, len, data, &r.error);
  return r;
}

IoResult Port::write(std::uint8_t id, std::uint16_t addr,
                     const std::uint8_t* data, std::uint16_t len) {
  if (!clearToTransmit()) return inhibitedResult();
  IoResult r;
  r.comm = packet_->writeTxRx(port_, id, addr, len,
                              const_cast<std::uint8_t*>(data), &r.error);
  return r;
}

IoResult Port::syncRead(std::uint16_t addr, std::uint16_t len,
                        const std::vector<std::uint8_t>& ids,
                        std::vector<std::uint8_t>& out) {
  if (!clearToTransmit()) return inhibitedResult();
  if (bounded_ != nullptr) {
    if (BoundedWindowState* w =
            bounded_->find(GroupedRead::kSync, addr, len, ids)) {
      out.resize(ids.size() * len);
      return boundedRead(*w, out);
    }
  }
  SyncReadTiming& timing = last_sync_read_timing_;
  timing = {};
  IoResult r;
  r.comm = packet_->syncReadTx(port_, addr, len,
                               const_cast<std::uint8_t*>(ids.data()),
                               static_cast<std::uint16_t>(ids.size()));
  if (r.comm != COMM_SUCCESS) return r;
  timing.instruction_sent = Clock::now();
  timing.sent = true;

  out.resize(ids.size() * len);
  for (std::size_t i = 0; i < ids.size(); ++i) {
    std::uint8_t err = 0;
    r.comm = packet_->readRx(port_, ids[i], len, &out[i * len], &err);
    if (r.comm != COMM_SUCCESS) return r;
    timing.last_reply = Clock::now();
    if (timing.replies++ == 0) timing.first_reply = timing.last_reply;
    if (err != 0 && r.error == 0) r.error = err;
  }
  return r;
}

IoResult Port::syncWrite(std::uint16_t addr, std::uint16_t len,
                         const std::vector<std::uint8_t>& ids,
                         const std::vector<std::uint8_t>& data) {
  if (!clearToTransmit()) return inhibitedResult();
  IoResult r;
  sync_write_param_.resize(ids.size() * (1 + len));
  for (std::size_t i = 0; i < ids.size(); ++i) {
    auto* param = &sync_write_param_[i * (1 + len)];
    param[0] = ids[i];
    std::copy_n(&data[i * len], len, param + 1);
  }
  r.comm = packet_->syncWriteTxOnly(
      port_, addr, len, sync_write_param_.data(),
      static_cast<std::uint16_t>(sync_write_param_.size()));
  return r;
}

IoResult Port::fastSyncRead(std::uint16_t addr, std::uint16_t len,
                            const std::vector<std::uint8_t>& ids,
                            std::vector<std::uint8_t>& out) {
  // No SDK path: nothing is sent outside an armed window.
  BoundedWindowState* w =
      bounded_ == nullptr
          ? nullptr
          : bounded_->find(GroupedRead::kFastSync, addr, len, ids);
  if (w == nullptr) {
    IoResult r;
    r.comm = kCommNotAvailable;
    return r;
  }
  if (!clearToTransmit()) return inhibitedResult();
  out.resize(ids.size() * len);
  return boundedRead(*w, out);
}

IoResult Port::boundedRead(BoundedWindowState& w,
                           std::vector<std::uint8_t>& out) {
  Bounded& b = *bounded_;
  const bool recovering = recovery_;
  const std::uint16_t len = w.window.len;
  const std::uint16_t requested =
      recovering ? static_cast<std::uint16_t>(len + 1) : len;
  StatusReplyParser& parser = recovering ? w.recovery : w.normal;
  SyncReadTiming& timing = last_sync_read_timing_;
  timing = {};
  ++stats_.reads;

  IoResult r;
  // The SDK sends the instruction, discarding received input first.
  const auto n = static_cast<std::uint16_t>(b.config.ids.size());
  auto* ids = const_cast<std::uint8_t*>(b.config.ids.data());
  r.comm = w.window.instruction == GroupedRead::kFastSync
               ? packet_->fastSyncReadTx(port_, w.window.addr, requested, ids,
                                         n)
               : packet_->syncReadTx(port_, w.window.addr, requested, ids, n);
  if (r.comm != COMM_SUCCESS) return r;
  const Clock::time_point t0 = Clock::now();
  timing.instruction_sent = t0;
  timing.sent = true;
  const Clock::time_point deadline = t0 + b.config.budget;

  // Receive until the replies are complete, cannot be valid, or the
  // budget runs out, parsing what has arrived after every read.
  using Status = StatusReplyParser::Status;
  Status status = Status::kIncomplete;
  std::size_t received = 0;
  std::size_t packets = 0;
  bool expired = false;
  while (status == Status::kIncomplete) {
    const int got = port_->readPort(b.rx.data() + received,
                                    static_cast<int>(b.rx.size() - received));
    // The clock after the receive: bytes held only at or after the
    // deadline (a receiver paused across it) are not accepted, even if
    // they would complete the reply.
    const Clock::time_point now = Clock::now();
    if (now >= deadline) {
      expired = true;
      break;
    }
    if (got > 0) {
      received += static_cast<std::size_t>(got);
      status = parser.parse(b.rx.data(), received);
      if (parser.packetsParsed() > packets) {
        if (packets == 0) timing.first_reply = now;
        timing.last_reply = now;
        packets = parser.packetsParsed();
        timing.replies = static_cast<int>(packets);
      }
      if (status != Status::kIncomplete) break;
    }
  }
  // The SDK marks the port busy until its own receive returns.
  port_->is_using_ = false;

  if (!expired && status == Status::kComplete) {
    const std::vector<std::uint8_t>& params = parser.params();
    for (std::size_t i = 0; i < b.config.ids.size(); ++i) {
      std::copy_n(params.begin() + static_cast<std::ptrdiff_t>(i * requested),
                  len, out.begin() + static_cast<std::ptrdiff_t>(i * len));
    }
    if (parser.anyServoError()) {
      ++stats_.servo_errors;
      for (const std::uint8_t e : parser.errors()) {
        if (e != 0) {
          r.error = e;
          break;
        }
      }
    }
    if (recovering) {
      recovery_ = false;
      ++stats_.recoveries;
    }
    return r;
  }

  // Ambiguous: late bytes may still arrive. Nothing leaves before the
  // budget and the guard have passed, and the next read is a recovery read.
  if (!expired && status == Status::kMalformed) {
    ++stats_.malformed;
    stats_.last_fault = parser.fault();
    r.comm = COMM_RX_CORRUPT;
  } else {
    ++stats_.timeouts;
    r.comm = COMM_RX_TIMEOUT;
  }
  quiet_until_ = t0 + b.config.budget + b.config.guard;
  quiet_pending_ = true;
  recovery_ = true;
  return r;
}

}  // namespace rtctrl::dxl
