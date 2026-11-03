// Integration: the bounded grouped read (control-rate Phase R2) of
// dxl::Port against the pty bus emulator, with scripted reply faults. The
// emulator records when each instruction reached it, so the quiet guard
// can be checked on the wire side as well as in the port.
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"

namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace reg = rtctrl::dxl::reg;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

namespace {

constexpr std::uint8_t kSyncRead = 0x82;
constexpr std::uint8_t kSyncWrite = 0x83;
constexpr std::uint8_t kRead = 0x02;
constexpr std::uint8_t kWrite = 0x03;
constexpr std::uint8_t kPing = 0x01;

// A CRANE-X7-shaped bus served on a background thread. The poll loop and
// withBus() share a mutex, so a test can change the emulated servos
// between transactions.
class FaultBus {
 public:
  FaultBus() {
    for (std::uint8_t id = 2; id <= 9; ++id) {
      emu::MotorEmulator::Config config;
      config.id = id;
      config.model_number =
          (id == 3) ? dxl::kModelXm540W270 : dxl::kModelXm430W350;
      bus_.add(config);
    }
    pty_ = std::make_unique<emu::PtyBus>(bus_);
    thread_ = std::thread([this] {
      while (!stop_.load()) {
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          pty_->poll(0.0001);
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });
  }

  ~FaultBus() {
    stop_.store(true);
    thread_.join();
  }

  const std::string& path() const { return pty_->slavePath(); }
  emu::PtyBus& pty() { return *pty_; }

  template <class F>
  void withBus(F f) {
    const std::lock_guard<std::mutex> lock(mutex_);
    f(bus_);
  }

  void setPositions(std::uint32_t base) {
    withBus([base](emu::MotorBus& bus) {
      for (auto& motor : bus.motors()) {
        motor.poke(reg::kPresentPosition, base + motor.id());
      }
    });
  }

  // Waits until the emulator has received n instructions.
  std::vector<emu::Arrival> waitArrivals(std::size_t n) {
    const auto give_up = Clock::now() + milliseconds(500);
    std::vector<emu::Arrival> a = pty_->arrivals();
    while (a.size() < n && Clock::now() < give_up) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      a = pty_->arrivals();
    }
    return a;
  }

 private:
  emu::MotorBus bus_;
  std::unique_ptr<emu::PtyBus> pty_;
  std::thread thread_;
  std::mutex mutex_;
  std::atomic<bool> stop_{false};
};

const std::vector<std::uint8_t> kIds{2, 3, 4, 5, 6, 7, 8, 9};
constexpr auto kBudget = milliseconds(30);
constexpr auto kGuard = milliseconds(30);

dxl::BoundedReadConfig positionRead() {
  dxl::BoundedReadConfig config;
  config.addr = reg::kPresentPosition.addr;
  config.len = reg::kPresentPosition.len;
  config.ids = kIds;
  config.budget = kBudget;
  config.guard = kGuard;
  return config;
}

std::vector<std::uint32_t> positions(const std::vector<std::uint8_t>& out) {
  std::vector<std::uint32_t> p;
  for (std::size_t i = 0; i + 3 < out.size(); i += 4) {
    p.push_back(static_cast<std::uint32_t>(out[i]) |
                (static_cast<std::uint32_t>(out[i + 1]) << 8) |
                (static_cast<std::uint32_t>(out[i + 2]) << 16) |
                (static_cast<std::uint32_t>(out[i + 3]) << 24));
  }
  return p;
}

std::vector<std::uint32_t> expected(std::uint32_t base) {
  std::vector<std::uint32_t> p;
  for (const std::uint8_t id : kIds) p.push_back(base + id);
  return p;
}

dxl::IoResult readPositions(dxl::Port& port, std::vector<std::uint8_t>& out) {
  return port.syncRead(reg::kPresentPosition.addr, reg::kPresentPosition.len,
                       kIds, out);
}

std::vector<std::uint16_t> syncReadLengths(
    const std::vector<emu::Arrival>& arrivals) {
  std::vector<std::uint16_t> lengths;
  for (const auto& a : arrivals) {
    if (a.instruction == kSyncRead) lengths.push_back(a.data_length);
  }
  return lengths;
}

}  // namespace

TEST_CASE("an armed grouped read returns what the SDK path returns",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  std::vector<std::uint8_t> sdk;
  REQUIRE(readPositions(port, sdk).ok());

  port.armBoundedSyncRead(positionRead());
  std::vector<std::uint8_t> bounded;
  REQUIRE(readPositions(port, bounded).ok());
  CHECK(bounded == sdk);
  CHECK(positions(bounded) == expected(2000));
  CHECK(port.boundedReadStats().reads == 1);
  const dxl::SyncReadTiming t = port.lastSyncReadTiming();
  CHECK(t.sent);
  CHECK(t.replies == 8);
  CHECK(t.first_reply >= t.instruction_sent);
  CHECK(t.last_reply >= t.first_reply);
  CHECK_FALSE(port.quietGuardPending());
}

TEST_CASE("the production feedback window reads over the bounded path",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(1500);
  dxl::Port port(bus.path(), 3000000);
  dxl::SyncGroup group(port, kIds);
  REQUIRE(group.setupIndirect().ok());
  // SyncGroup's feedback window: 13 bytes at the start of Indirect Data.
  dxl::BoundedReadConfig config = positionRead();
  config.addr = reg::kIndirectDataBase;
  config.len = 13;
  port.armBoundedSyncRead(config);
  std::vector<dxl::Feedback> fb;
  REQUIRE(group.readAll(fb).ok());
  CHECK(port.boundedReadStats().reads == 1);
  REQUIRE(fb.size() == kIds.size());
  for (std::size_t i = 0; i < kIds.size(); ++i) {
    CHECK(fb[i].position == dxl::pulseToRad(1500 + kIds[i]));
  }
}

TEST_CASE("a reply delayed within the budget is accepted", "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(positionRead());
  bus.pty().scriptSyncReadFaults(
      {{emu::ReplyFault::Kind::kDelay, milliseconds(5), 0, {}}});
  std::vector<std::uint8_t> out;
  const auto start = Clock::now();
  REQUIRE(readPositions(port, out).ok());
  CHECK(Clock::now() - start >= milliseconds(5));
  CHECK(positions(out) == expected(2000));
  CHECK(port.boundedReadStats().timeouts == 0);
  CHECK_FALSE(port.quietGuardPending());
  CHECK_FALSE(port.inRecovery());
}

TEST_CASE("a late reply fails at the deadline, and nothing is sent before "
          "the quiet guard ends",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(positionRead());
  // The replies come long after the test is done: the read times out.
  bus.pty().scriptSyncReadFaults(
      {{emu::ReplyFault::Kind::kDelay, milliseconds(400), 0, {}}});
  std::vector<std::uint8_t> out;
  const auto start = Clock::now();
  const dxl::IoResult failed = readPositions(port, out);
  const auto elapsed = Clock::now() - start;
  REQUIRE_FALSE(failed.ok());
  CHECK(failed.comm != 0);
  CHECK(port.boundedReadStats().timeouts == 1);
  CHECK(elapsed >= kBudget);
  CHECK(elapsed < kBudget + milliseconds(25));

  const auto t0 = port.lastSyncReadTiming().instruction_sent;
  REQUIRE(port.quietGuardPending());
  CHECK(port.quietUntil() == t0 + kBudget + kGuard);
  CHECK(port.inRecovery());
  const std::size_t before = bus.pty().arrivals().size();
  const auto quiet_until = port.quietUntil();

  // Every kind of transmission waits for the guard: a fallback goal write
  // like the one the background thread sends after a failed read, a single
  // write, a single read, a ping and the next grouped read.
  SECTION("a fallback sync write") {
    const std::vector<std::uint8_t> goals(kIds.size() * 4, 0);
    CHECK(port.syncWrite(reg::kGoalPosition.addr, 4, kIds, goals).ok());
  }
  SECTION("a single write") {
    CHECK(port.write32(5, reg::kGoalPosition.addr, 2048).ok());
  }
  SECTION("a single read") {
    std::uint8_t model[2] = {};
    CHECK(port.read(5, reg::kModelNumber.addr, model, 2).ok());
  }
  SECTION("a ping") { CHECK(port.ping(5, nullptr).ok()); }
  SECTION("the next grouped read") {
    CHECK(readPositions(port, out).ok());
    CHECK(positions(out) == expected(2000));
  }

  CHECK(port.boundedReadStats().guard_waits == 1);
  CHECK(port.lastGuardRelease() >= quiet_until);
  CHECK_FALSE(port.quietGuardPending());
  // On the wire: the emulator received nothing in the guard. It notes an
  // instruction when it polls (every 0.1 ms here), at or after the
  // transmission, so allow one millisecond against a 60 ms guard.
  const auto arrivals = bus.waitArrivals(before + 1);
  REQUIRE(arrivals.size() > before);
  for (std::size_t i = before; i < arrivals.size(); ++i) {
    CAPTURE(i, static_cast<int>(arrivals[i].instruction));
    CHECK(arrivals[i].at >= quiet_until - milliseconds(1));
  }
  const std::uint8_t first = arrivals[before].instruction;
  CHECK((first == kSyncWrite || first == kWrite || first == kRead ||
         first == kPing || first == kSyncRead));
}

TEST_CASE("the recovery read asks for one byte more, rejects a late "
          "normal-length reply and returns fresh values",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(positionRead());
  // Read 1's replies arrive 10 ms into the recovery read (the guard ends
  // 60 ms after its instruction); the recovery read's own replies are held
  // for 25 ms, so the late normal-length replies reach it first.
  bus.pty().scriptSyncReadFaults(
      {{emu::ReplyFault::Kind::kDelay, kBudget + kGuard + milliseconds(10),
        0, {}},
       {emu::ReplyFault::Kind::kDelay, milliseconds(25), 0, {}}});
  std::vector<std::uint8_t> out;
  REQUIRE_FALSE(readPositions(port, out).ok());
  REQUIRE(port.inRecovery());

  // The servos move between the failed read and the recovery.
  bus.setPositions(3000);

  // Recovery read 1: the late replies to read 1 arrive first and fail on
  // their length field; nothing from them is returned.
  REQUIRE_FALSE(readPositions(port, out).ok());
  CHECK(port.boundedReadStats().malformed == 1);
  CHECK(port.boundedReadStats().last_fault ==
        dxl::StatusReplyParser::Fault::kLength);
  CHECK(port.inRecovery());

  // Recovery read 2, after a second guard: clean, and fresh.
  REQUIRE(readPositions(port, out).ok());
  CHECK(positions(out) == expected(3000));
  CHECK_FALSE(port.inRecovery());
  CHECK(port.boundedReadStats().recoveries == 1);

  // The normal length resumes.
  REQUIRE(readPositions(port, out).ok());
  CHECK(positions(out) == expected(3000));
  const auto lengths = syncReadLengths(bus.waitArrivals(4));
  const std::vector<std::uint16_t> want{4, 5, 5, 4};
  CHECK(lengths == want);
}

TEST_CASE("a partial reply, garbage and a missing reply are recovered from",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(positionRead());
  std::vector<std::uint8_t> out;

  SECTION("a partial reply times out") {
    bus.pty().scriptSyncReadFaults(
        {{emu::ReplyFault::Kind::kTruncate, {}, 20, {}}});
    REQUIRE_FALSE(readPositions(port, out).ok());
    CHECK(port.boundedReadStats().timeouts == 1);
  }
  SECTION("garbage fails at once, without waiting for the deadline") {
    bus.pty().scriptSyncReadFaults(
        {{emu::ReplyFault::Kind::kGarbage, {}, 0, {0x00, 0x11, 0x22}}});
    const auto start = Clock::now();
    REQUIRE_FALSE(readPositions(port, out).ok());
    CHECK(Clock::now() - start < kBudget / 2);
    CHECK(port.boundedReadStats().malformed == 1);
    CHECK(port.boundedReadStats().last_fault ==
          dxl::StatusReplyParser::Fault::kHeader);
  }
  SECTION("a missing reply times out") {
    bus.pty().scriptSyncReadFaults({{emu::ReplyFault::Kind::kDrop, {}, 0, {}}});
    REQUIRE_FALSE(readPositions(port, out).ok());
    CHECK(port.boundedReadStats().timeouts == 1);
  }

  REQUIRE(port.inRecovery());
  bus.setPositions(2500);
  REQUIRE(readPositions(port, out).ok());
  CHECK(positions(out) == expected(2500));
  CHECK_FALSE(port.inRecovery());
  CHECK(port.boundedReadStats().recoveries == 1);
}

TEST_CASE("unarmed, a grouped read keeps the SDK path and sets no guard",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  std::vector<std::uint8_t> out;

  SECTION("a delayed reply within the SDK's timeout") {
    bus.pty().scriptSyncReadFaults(
        {{emu::ReplyFault::Kind::kDelay, milliseconds(5), 0, {}}});
    REQUIRE(readPositions(port, out).ok());
    CHECK(positions(out) == expected(2000));
  }
  SECTION("a missing reply fails, and the next transmission is not held") {
    bus.pty().scriptSyncReadFaults({{emu::ReplyFault::Kind::kDrop, {}, 0, {}}});
    REQUIRE_FALSE(readPositions(port, out).ok());
    CHECK_FALSE(port.quietGuardPending());
    CHECK(port.write32(5, reg::kGoalPosition.addr, 2048).ok());
    CHECK(port.boundedReadStats().guard_waits == 0);
    CHECK(port.lastGuardRelease() == Clock::time_point{});
  }
  SECTION("armed, a different window still takes the SDK path") {
    port.armBoundedSyncRead(positionRead());
    std::vector<std::uint8_t> other;
    REQUIRE(port.syncRead(reg::kPresentVelocity.addr,
                          reg::kPresentVelocity.len, kIds, other)
                .ok());
    CHECK(other.size() == kIds.size() * 4);
  }
  CHECK(port.boundedReadStats().reads == 0);
  CHECK_FALSE(port.inRecovery());
}

TEST_CASE("arming again keeps a recovery that is still due",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(positionRead());
  bus.pty().scriptSyncReadFaults({{emu::ReplyFault::Kind::kDrop, {}, 0, {}}});
  std::vector<std::uint8_t> out;
  REQUIRE_FALSE(readPositions(port, out).ok());
  REQUIRE(port.inRecovery());

  // Arming the same window again, or disarming and arming, does not turn
  // the next read back into a normal one.
  SECTION("armed again") { port.armBoundedSyncRead(positionRead()); }
  SECTION("disarmed, then armed") {
    port.disarmBoundedSyncRead();
    CHECK(port.inRecovery());
    port.armBoundedSyncRead(positionRead());
  }
  CHECK(port.inRecovery());
  REQUIRE(readPositions(port, out).ok());
  CHECK_FALSE(port.inRecovery());
  const auto lengths = syncReadLengths(bus.waitArrivals(2));
  const std::vector<std::uint16_t> want{4, 5};
  CHECK(lengths == want);
}

TEST_CASE("a bounded read is armed only with a usable window and limits",
          "[pty][bounded]") {
  FaultBus bus;
  dxl::Port port(bus.path(), 3000000);
  auto with = [](auto change) {
    dxl::BoundedReadConfig c = positionRead();
    change(c);
    return c;
  };
  CHECK_THROWS_AS(port.armBoundedSyncRead(with([](auto& c) { c.ids = {}; })),
                  std::invalid_argument);
  CHECK_THROWS_AS(port.armBoundedSyncRead(with([](auto& c) { c.len = 0; })),
                  std::invalid_argument);
  CHECK_THROWS_AS(port.armBoundedSyncRead(
                      with([](auto& c) { c.budget = milliseconds(0); })),
                  std::invalid_argument);
  CHECK_THROWS_AS(port.armBoundedSyncRead(
                      with([](auto& c) { c.guard = milliseconds(-1); })),
                  std::invalid_argument);
  CHECK_FALSE(port.boundedSyncReadArmed());
  port.armBoundedSyncRead(positionRead());
  CHECK(port.boundedSyncReadArmed());
  port.disarmBoundedSyncRead();
  CHECK_FALSE(port.boundedSyncReadArmed());
}
