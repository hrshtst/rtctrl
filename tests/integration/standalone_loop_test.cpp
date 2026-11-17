// Integration: PtyBus's I/O half (serviceIo, nextReplyDue) and the
// standalone emulator loop (emu::StandaloneLoop, dxl_emu's), over a real
// pseudo-terminal. The model-time cases step the loop at synthetic times,
// so they are deterministic; the run() cases use the real clock and assert
// only loose bounds (host dispatch is not the code's).
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"
#include "rtctrl/emu/standalone_loop.hpp"

namespace emu = rtctrl::emu;
namespace dxl = rtctrl::dxl;
namespace reg = rtctrl::dxl::reg;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;
using Loop = emu::StandaloneLoop;

namespace {

constexpr std::uint8_t kPing = 0x01;
constexpr std::uint8_t kRead = 0x02;
constexpr std::uint8_t kWrite = 0x03;
constexpr std::uint8_t kSyncRead = 0x82;

std::vector<std::uint8_t> frame(std::uint8_t id, std::uint8_t instruction,
                                const std::vector<std::uint8_t>& params) {
  std::vector<std::uint8_t> body = {instruction};
  body.insert(body.end(), params.begin(), params.end());
  emu::dxlAddStuffing(body);
  std::vector<std::uint8_t> f = {0xFF, 0xFF, 0xFD, 0x00, id};
  const auto length = static_cast<std::uint16_t>(body.size() + 2);
  f.push_back(static_cast<std::uint8_t>(length & 0xFF));
  f.push_back(static_cast<std::uint8_t>(length >> 8));
  f.insert(f.end(), body.begin(), body.end());
  const std::uint16_t crc = emu::dxlCrc16(f.data(), f.size());
  f.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  f.push_back(static_cast<std::uint8_t>(crc >> 8));
  return f;
}

std::vector<std::uint8_t> le(std::uint32_t v, int n) {
  std::vector<std::uint8_t> out;
  for (int i = 0; i < n; ++i) out.push_back((v >> (8 * i)) & 0xFF);
  return out;
}

std::vector<std::uint8_t> writeFrame(std::uint8_t id, dxl::Reg r,
                                     std::uint32_t value) {
  std::vector<std::uint8_t> p = le(r.addr, 2);
  const auto v = le(value, r.len);
  p.insert(p.end(), v.begin(), v.end());
  return frame(id, kWrite, p);
}

std::vector<std::uint8_t> readFrame(std::uint8_t id, dxl::Reg r) {
  std::vector<std::uint8_t> p = le(r.addr, 2);
  const auto n = le(r.len, 2);
  p.insert(p.end(), n.begin(), n.end());
  return frame(id, kRead, p);
}

std::vector<std::uint8_t> syncReadFrame(dxl::Reg r,
                                        const std::vector<std::uint8_t>& ids) {
  std::vector<std::uint8_t> p = le(r.addr, 2);
  const auto n = le(r.len, 2);
  p.insert(p.end(), n.begin(), n.end());
  p.insert(p.end(), ids.begin(), ids.end());
  return frame(0xFE, kSyncRead, p);
}

// The client end: the slave, raw, as the SDK's port sets it.
class Client {
 public:
  explicit Client(const std::string& path) {
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("cannot open " + path);
    termios t{};
    tcgetattr(fd_, &t);
    cfmakeraw(&t);
    tcsetattr(fd_, TCSANOW, &t);
  }
  ~Client() { close(); }
  void close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
  }
  void send(const std::vector<std::uint8_t>& bytes) const {
    std::size_t off = 0;
    while (off < bytes.size()) {
      const ssize_t n = ::write(fd_, bytes.data() + off, bytes.size() - off);
      if (n > 0) off += static_cast<std::size_t>(n);
    }
  }
  // Bytes received within timeout (stops early once want bytes are in).
  std::vector<std::uint8_t> receive(std::size_t want, milliseconds timeout) {
    std::vector<std::uint8_t> out;
    const auto give_up = Clock::now() + timeout;
    while (out.size() < want && Clock::now() < give_up) {
      pollfd p{fd_, POLLIN, 0};
      ::poll(&p, 1, 5);
      std::uint8_t buf[256];
      const ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n > 0) out.insert(out.end(), buf, buf + n);
    }
    return out;
  }

 private:
  int fd_ = -1;
};

// Waits until the master holds at least n input bytes (the tty layer
// delivers a client's write asynchronously).
bool masterHas(const emu::PtyBus& pty, int n, milliseconds timeout) {
  const auto give_up = Clock::now() + timeout;
  while (Clock::now() < give_up) {
    int avail = 0;
    if (ioctl(pty.masterFd(), FIONREAD, &avail) == 0 && avail >= n) return true;
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  return false;
}

// Status packets in a byte stream: (id, error, params).
struct Status {
  std::uint8_t id;
  std::uint8_t error;
  std::vector<std::uint8_t> params;
};
std::vector<Status> statuses(const std::vector<std::uint8_t>& bytes) {
  std::vector<Status> out;
  std::size_t i = 0;
  while (i + 7 <= bytes.size()) {
    if (!(bytes[i] == 0xFF && bytes[i + 1] == 0xFF && bytes[i + 2] == 0xFD)) {
      ++i;
      continue;
    }
    const std::size_t length = bytes[i + 5] | (bytes[i + 6] << 8);
    if (i + 7 + length > bytes.size()) break;
    std::vector<std::uint8_t> body(bytes.begin() + i + 7,
                                   bytes.begin() + i + 7 + length - 2);
    emu::dxlRemoveStuffing(body);
    if (body.size() >= 2) {
      out.push_back({bytes[i + 4], body[1],
                     std::vector<std::uint8_t>(body.begin() + 2, body.end())});
    }
    i += 7 + length;
  }
  return out;
}

emu::MotorEmulator* oneMotor(emu::MotorBus& bus, std::uint8_t id = 2) {
  emu::MotorEmulator::Config config;
  config.id = id;
  return bus.add(config);
}

// A motor turning at Goal Velocity 64 (0.229 rpm units): 1000.5 pulses/s,
// so exactly one pulse per 1 ms tick after rounding.
void spin(emu::MotorEmulator& m, std::uint32_t goal_velocity) {
  m.poke(reg::kOperatingMode,
         static_cast<std::uint32_t>(dxl::OperatingMode::kVelocity));
  m.poke(reg::kGoalVelocity, goal_velocity);
  m.poke(reg::kTorqueEnable, 1);
}

const Clock::time_point kT0 = Clock::time_point{} + std::chrono::hours(1);

}  // namespace

TEST_CASE("serviceIo then tick is poll", "[emu-loop]") {
  // Two identical buses fed the same input: one through poll(dt), one
  // through serviceIo() then MotorBus::tick(dt).
  emu::MotorBus bus_a, bus_b;
  spin(*oneMotor(bus_a), 64);
  spin(*oneMotor(bus_b), 64);
  emu::PtyBus pty_a(bus_a), pty_b(bus_b);
  Client a(pty_a.slavePath()), b(pty_b.slavePath());
  const std::vector<std::vector<std::uint8_t>> script = {
      frame(2, kPing, {}), writeFrame(2, reg::kGoalVelocity, 128),
      readFrame(2, reg::kPresentPosition), frame(7, kPing, {}),
      readFrame(2, reg::kGoalVelocity)};
  std::vector<std::uint8_t> got_a, got_b;
  for (const auto& f : script) {
    a.send(f);
    b.send(f);
    REQUIRE(masterHas(pty_a, static_cast<int>(f.size()), milliseconds(500)));
    REQUIRE(masterHas(pty_b, static_cast<int>(f.size()), milliseconds(500)));
    for (int i = 0; i < 3; ++i) {
      REQUIRE(pty_a.poll(0.001));
      const auto io = pty_b.serviceIo();
      REQUIRE(io.ok);
      CHECK_FALSE(io.limited);
      bus_b.tick(0.001);
    }
    const auto ra = a.receive(1, milliseconds(200));
    const auto rb = b.receive(1, milliseconds(200));
    got_a.insert(got_a.end(), ra.begin(), ra.end());
    got_b.insert(got_b.end(), rb.begin(), rb.end());
  }
  for (auto* pair : {&got_a, &got_b}) {
    const auto rest = (pair == &got_a ? a : b).receive(1 << 12, milliseconds(50));
    pair->insert(pair->end(), rest.begin(), rest.end());
  }
  CHECK(got_a == got_b);
  CHECK(statuses(got_a).size() == 4);  // id 7 is absent: no reply
  CHECK(bus_a.find(2)->peek(reg::kPresentPosition) ==
        bus_b.find(2)->peek(reg::kPresentPosition));
  CHECK(bus_a.find(2)->peek(reg::kGoalVelocity) == 128);
}

TEST_CASE("serviceIo reads at most its bound and partial frames wait",
          "[emu-loop]") {
  emu::MotorBus bus;
  oneMotor(bus);
  emu::PtyBus pty(bus);
  Client c(pty.slavePath());
  const auto ping = frame(2, kPing, {});  // 10 bytes
  REQUIRE(ping.size() == 10);
  std::vector<std::uint8_t> three;
  for (int i = 0; i < 3; ++i) three.insert(three.end(), ping.begin(), ping.end());
  c.send(three);
  REQUIRE(masterHas(pty, 30, milliseconds(500)));

  auto io = pty.serviceIo(15);  // one frame and half of the next
  CHECK(io.ok);
  CHECK(io.bytes_read == 15);
  CHECK(io.limited);
  CHECK(pty.arrivals().size() == 1);
  io = pty.serviceIo(15);
  CHECK(io.bytes_read == 15);
  CHECK(pty.arrivals().size() == 3);
  io = pty.serviceIo(15);
  CHECK(io.bytes_read == 0);
  CHECK_FALSE(io.limited);
  CHECK(statuses(c.receive(3 * 14, milliseconds(500))).size() == 3);

  // A frame split across two writes is answered once, when complete.
  c.send(std::vector<std::uint8_t>(ping.begin(), ping.begin() + 6));
  REQUIRE(masterHas(pty, 6, milliseconds(500)));
  pty.serviceIo();
  CHECK(pty.arrivals().size() == 3);
  c.send(std::vector<std::uint8_t>(ping.begin() + 6, ping.end()));
  REQUIRE(masterHas(pty, 4, milliseconds(500)));
  pty.serviceIo();
  CHECK(pty.arrivals().size() == 4);
  CHECK(statuses(c.receive(14, milliseconds(500))).size() == 1);
}

TEST_CASE("nextReplyDue with no one and several delayed replies", "[emu-loop]") {
  emu::MotorBus bus;
  for (std::uint8_t id = 2; id <= 3; ++id) oneMotor(bus, id);
  emu::PtyBus pty(bus);
  Client c(pty.slavePath());
  CHECK_FALSE(pty.nextReplyDue().has_value());

  emu::ReplyFault slow, fast;
  slow.delay = milliseconds(400);
  fast.delay = milliseconds(200);
  pty.scriptSyncReadFaults({slow, fast});
  const auto sr = syncReadFrame(reg::kPresentPosition, {2, 3});

  c.send(sr);
  REQUIRE(masterHas(pty, static_cast<int>(sr.size()), milliseconds(500)));
  const auto before1 = Clock::now();
  pty.serviceIo();
  const auto after1 = Clock::now();
  REQUIRE(pty.nextReplyDue().has_value());
  const auto due1 = *pty.nextReplyDue();
  CHECK(due1 >= before1 + slow.delay);
  CHECK(due1 <= after1 + slow.delay);

  c.send(sr);
  REQUIRE(masterHas(pty, static_cast<int>(sr.size()), milliseconds(500)));
  const auto before2 = Clock::now();
  pty.serviceIo();
  const auto after2 = Clock::now();
  REQUIRE(pty.nextReplyDue().has_value());
  const auto due2 = *pty.nextReplyDue();  // the second, sooner reply
  CHECK(due2 >= before2 + fast.delay);
  CHECK(due2 <= after2 + fast.delay);
  CHECK(due2 < due1);

  // Nothing is sent before its due time; each goes when serviced after it.
  std::this_thread::sleep_until(due2 + milliseconds(5));
  pty.serviceIo();
  CHECK(pty.nextReplyDue() == std::optional<Clock::time_point>(due1));
  std::this_thread::sleep_until(due1 + milliseconds(5));
  pty.serviceIo();
  CHECK_FALSE(pty.nextReplyDue().has_value());
  CHECK(statuses(c.receive(2 * 2 * 15, milliseconds(500))).size() == 4);
}

TEST_CASE("the loop runs the due ticks before the commands it reads",
          "[emu-loop]") {
  emu::MotorBus bus, reference;
  auto* motor = oneMotor(bus);
  auto* ref = oneMotor(reference);
  spin(*motor, 64);
  spin(*ref, 64);
  emu::PtyBus pty(bus);
  Client c(pty.slavePath());
  Loop loop(pty, bus, Loop::Config{}, kT0);

  REQUIRE(loop.step(kT0 + milliseconds(1)) == Loop::Status::kOk);
  ref->tick(0.001);
  CHECK(loop.stats().ticks == 1);
  CHECK(motor->peek(reg::kPresentPosition) == ref->peek(reg::kPresentPosition));

  SECTION("a goal changed after a stall applies after the elapsed time") {
    // The command arrives during a 10 ms stall; the step owes 10 ticks.
    const auto w = writeFrame(2, reg::kGoalVelocity, 128);
    c.send(w);
    REQUIRE(masterHas(pty, static_cast<int>(w.size()), milliseconds(500)));
    REQUIRE(loop.step(kT0 + milliseconds(11)) == Loop::Status::kOk);
    for (int i = 0; i < 10; ++i) ref->tick(0.001);  // at the old goal
    CHECK(loop.stats().ticks == 11);
    CHECK(loop.stats().late_steps == 1);
    CHECK(loop.stats().max_ticks_in_step == 10);
    CHECK(motor->peek(reg::kPresentPosition) ==
          ref->peek(reg::kPresentPosition));
    CHECK(motor->peek(reg::kGoalVelocity) == 128);
    // Applied retroactively the position would be 10 pulses further on.
    ref->poke(reg::kGoalVelocity, 128);
    REQUIRE(loop.step(kT0 + milliseconds(12)) == Loop::Status::kOk);
    ref->tick(0.001);
    CHECK(motor->peek(reg::kPresentPosition) ==
          ref->peek(reg::kPresentPosition));
  }

  SECTION("an overdue Bus Watchdog expires before the packet that feeds it") {
    motor->poke(reg::kBusWatchdog, 1);  // 20 ms
    motor->touch();
    REQUIRE(loop.step(kT0 + milliseconds(5)) == Loop::Status::kOk);
    // The packet is read only at 30 ms, past the 20 ms deadline.
    const auto r = readFrame(2, reg::kBusWatchdog);
    c.send(r);
    REQUIRE(masterHas(pty, static_cast<int>(r.size()), milliseconds(500)));
    REQUIRE(loop.step(kT0 + milliseconds(30)) == Loop::Status::kOk);
    CHECK(motor->watchdogTriggered());
    const auto replies = statuses(c.receive(12, milliseconds(500)));
    REQUIRE(replies.size() == 1);
    REQUIRE(replies[0].params.size() == 1);
    CHECK(replies[0].params[0] == dxl::kBusWatchdogTriggered);
  }

  SECTION("a packet read before the watchdog deadline feeds it") {
    motor->poke(reg::kBusWatchdog, 1);  // 20 ms
    motor->touch();
    const auto r = readFrame(2, reg::kBusWatchdog);
    c.send(r);
    REQUIRE(masterHas(pty, static_cast<int>(r.size()), milliseconds(500)));
    REQUIRE(loop.step(kT0 + milliseconds(15)) == Loop::Status::kOk);
    REQUIRE(loop.step(kT0 + milliseconds(30)) == Loop::Status::kOk);
    CHECK_FALSE(motor->watchdogTriggered());
    const auto replies = statuses(c.receive(12, milliseconds(500)));
    REQUIRE(replies.size() == 1);
    CHECK(replies[0].params == std::vector<std::uint8_t>{1});
  }
}

TEST_CASE("the catch-up cap runs ticks up to it and is an overrun past it",
          "[emu-loop]") {
  emu::MotorBus bus;
  auto* motor = oneMotor(bus);
  spin(*motor, 64);
  emu::PtyBus pty(bus);
  Loop::Config config;
  config.max_catch_up_ticks = 50;
  Loop loop(pty, bus, config, kT0);
  // The first tick is owed at kT0 + 1 ms; at kT0 + 50 ms, 50 are due.
  REQUIRE(loop.step(kT0 + milliseconds(50)) == Loop::Status::kOk);
  CHECK(loop.stats().ticks == 50);
  CHECK(loop.stats().max_ticks_in_step == 50);
  const auto position = motor->peek(reg::kPresentPosition);
  // The next is owed at kT0 + 51 ms; 50 ms later, 51 are due.
  CHECK(loop.step(kT0 + milliseconds(100)) == Loop::Status::kOk);  // 50
  CHECK(loop.step(kT0 + milliseconds(151)) == Loop::Status::kOverrun);
  CHECK(loop.stats().overrun_due == 51);
  CHECK(loop.stats().ticks == 100);  // none of the 51 ran
  CHECK(motor->peek(reg::kPresentPosition) == position + 50);
  CHECK(loop.nextTick() == kT0 + milliseconds(101));
}

TEST_CASE("the loop deadline is the next tick or a sooner delayed reply",
          "[emu-loop]") {
  emu::MotorBus bus;
  for (std::uint8_t id = 2; id <= 3; ++id) oneMotor(bus, id);
  emu::PtyBus pty(bus);
  Client c(pty.slavePath());
  Loop::Config config;
  config.tick = std::chrono::seconds(1);  // the next tick is far off
  const auto start = Clock::now();
  Loop loop(pty, bus, config, start);
  CHECK(loop.deadline() == start + config.tick);

  emu::ReplyFault soon, late;
  soon.delay = milliseconds(50);
  late.delay = std::chrono::seconds(5);
  pty.scriptSyncReadFaults({soon, late});
  const auto sr = syncReadFrame(reg::kPresentPosition, {2, 3});
  c.send(sr);
  REQUIRE(masterHas(pty, static_cast<int>(sr.size()), milliseconds(500)));
  REQUIRE(loop.step(Clock::now()) == Loop::Status::kOk);
  REQUIRE(pty.nextReplyDue().has_value());
  CHECK(loop.deadline() == *pty.nextReplyDue());
  CHECK(loop.deadline() < loop.nextTick());

  std::this_thread::sleep_until(*pty.nextReplyDue() + milliseconds(2));
  c.send(sr);
  REQUIRE(masterHas(pty, static_cast<int>(sr.size()), milliseconds(500)));
  REQUIRE(loop.step(Clock::now()) == Loop::Status::kOk);
  REQUIRE(pty.nextReplyDue().has_value());  // the 5 s one
  CHECK(loop.deadline() == loop.nextTick());
}

TEST_CASE("run(): a delayed reply goes out at its due time without input",
          "[emu-loop]") {
  emu::MotorBus bus;
  for (std::uint8_t id = 2; id <= 3; ++id) oneMotor(bus, id);
  emu::PtyBus pty(bus);
  Client c(pty.slavePath());
  Loop::Config config;
  config.tick = milliseconds(200);  // ticks alone would send it far later
  emu::ReplyFault delay;
  delay.delay = milliseconds(30);
  pty.scriptSyncReadFaults({delay});
  std::atomic<bool> stop{false};
  Loop loop(pty, bus, config, Clock::now());
  std::thread runner([&] { loop.run([&] { return stop.load(); }); });

  const auto sr = syncReadFrame(reg::kPresentPosition, {2, 3});
  const auto sent = Clock::now();
  c.send(sr);
  const auto reply = c.receive(1, milliseconds(1000));
  const auto got = Clock::now();
  stop.store(true);
  runner.join();
  REQUIRE_FALSE(reply.empty());
  const auto waited =
      std::chrono::duration_cast<std::chrono::microseconds>(got - sent);
  INFO("reply after " << waited.count() << " us");
  CHECK(waited >= delay.delay);          // never early
  CHECK(waited < milliseconds(150));     // not held for the 200 ms tick
}

TEST_CASE("wait(): by time alone after the client detaches", "[emu-loop]") {
  emu::MotorBus bus;
  oneMotor(bus);
  emu::PtyBus pty(bus);
  Loop loop(pty, bus, Loop::Config{}, Clock::now());

  // Never attached: no hangup is reported; the wait runs to its deadline.
  auto t = Clock::now();
  REQUIRE(loop.wait(t + milliseconds(20)));
  CHECK(Clock::now() - t >= milliseconds(19));
  CHECK(loop.stats().hangup_waits == 0);

  {
    Client c(pty.slavePath());
    c.send(frame(2, kPing, {}));
    REQUIRE(masterHas(pty, 10, milliseconds(500)));
    t = Clock::now();
    REQUIRE(loop.wait(t + milliseconds(500)));  // input: returns at once
    CHECK(Clock::now() - t < milliseconds(100));
    pty.serviceIo();
    CHECK(statuses(c.receive(14, milliseconds(500))).size() == 1);
  }
  // Detached: the master reports a hangup at once; no spin to the deadline.
  t = Clock::now();
  REQUIRE(loop.wait(t + milliseconds(20)));
  CHECK(Clock::now() - t >= milliseconds(19));
  CHECK(loop.stats().hangup_waits == 1);

  // Reattached: input ends the wait again, and no frame is lost.
  Client again(pty.slavePath());
  for (int i = 0; i < 5; ++i) again.send(frame(2, kPing, {}));
  REQUIRE(masterHas(pty, 50, milliseconds(500)));
  t = Clock::now();
  REQUIRE(loop.wait(t + milliseconds(500)));
  CHECK(Clock::now() - t < milliseconds(100));
  CHECK(loop.stats().hangup_waits == 1);
  pty.serviceIo();
  CHECK(statuses(again.receive(5 * 14, milliseconds(500))).size() == 5);
}

TEST_CASE("run(): a stop request is seen while waiting and while busy",
          "[emu-loop]") {
  emu::MotorBus bus;
  oneMotor(bus);
  emu::PtyBus pty(bus);
  std::atomic<bool> stop{false};
  Loop loop(pty, bus, Loop::Config{}, Clock::now());
  Loop::Status status = Loop::Status::kIoError;
  std::optional<Client> c;
  c.emplace(pty.slavePath());

  // Busy: a client sending pings without pause (its replies unread, so
  // the emulator's writes are dropped once the client's buffer is full).
  std::atomic<bool> flood{false};
  std::thread writer;
  SECTION("while waiting, a client attached and idle") {}
  bool detached = false;
  SECTION("while waiting, the client detached") {
    c.reset();
    detached = true;
  }
  SECTION("while busy") {
    flood.store(true);
    writer = std::thread([&] {
      const auto f = frame(2, kPing, {});
      std::vector<std::uint8_t> burst;
      for (int i = 0; i < 400; ++i) burst.insert(burst.end(), f.begin(), f.end());
      const int fd = ::open(pty.slavePath().c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      termios t{};
      tcgetattr(fd, &t);
      cfmakeraw(&t);
      tcsetattr(fd, TCSANOW, &t);
      std::size_t off = 0;
      while (flood.load()) {
        const ssize_t n = ::write(fd, burst.data() + off, burst.size() - off);
        if (n > 0) off = (off + static_cast<std::size_t>(n)) % burst.size();
        else std::this_thread::yield();
      }
      ::close(fd);
    });
  }
  std::thread runner([&] { status = loop.run([&] { return stop.load(); }); });
  std::this_thread::sleep_for(milliseconds(300));
  const auto asked = Clock::now();
  stop.store(true);
  runner.join();
  const auto stopped = Clock::now() - asked;
  flood.store(false);
  if (writer.joinable()) writer.join();
  INFO("stopped after "
       << std::chrono::duration_cast<std::chrono::microseconds>(stopped).count()
       << " us; reads at the bound " << loop.stats().limited_reads
       << ", ticks " << loop.stats().ticks << ", waits by time alone "
       << loop.stats().hangup_waits);
  CHECK(status == Loop::Status::kOk);
  CHECK(stopped < milliseconds(100));
  CHECK(loop.stats().ticks >= 250);  // the ticks kept running throughout
  if (detached) CHECK(loop.stats().hangup_waits > 0);
}
