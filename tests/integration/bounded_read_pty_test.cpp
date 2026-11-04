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
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

#include "fault_bus.hpp"

// The fixture (FaultBus) and its helpers, with the dxl/emu/reg aliases.
using namespace fault_bus;  // NOLINT(build/namespaces)

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
  config.windows = {{reg::kIndirectDataBase, 13}};
  port.armBoundedSyncRead(config);
  std::vector<dxl::Feedback> fb;
  REQUIRE(group.readAll(fb).ok());
  CHECK(port.boundedReadStats().reads == 1);
  REQUIRE(fb.size() == kIds.size());
  for (std::size_t i = 0; i < kIds.size(); ++i) {
    CHECK(fb[i].position == dxl::pulseToRad(1500 + kIds[i]));
  }
}

TEST_CASE("two windows over the same servos both read bounded, and the "
          "read after a failure in one is the other's recovery read",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(1800);
  dxl::Port port(bus.path(), 3000000);
  dxl::SyncGroup group(port, kIds);
  REQUIRE(group.setupIndirect().ok());
  // The motion-only read (10 bytes) and the full feedback window (13).
  dxl::BoundedReadConfig config = positionRead();
  config.windows = {{reg::kIndirectDataBase, 13}, {reg::kIndirectDataBase, 10}};
  port.armBoundedSyncRead(config);
  std::vector<dxl::Feedback> fb;
  REQUIRE(group.readAll(fb).ok());
  REQUIRE(group.readMotionWindow(fb).ok());
  CHECK(port.boundedReadStats().reads == 2);
  for (std::size_t i = 0; i < kIds.size(); ++i) {
    CHECK(fb[i].position == dxl::pulseToRad(1800 + kIds[i]));
  }
  const std::size_t before = bus.pty().arrivals().size();

  // The full read's replies never come: it times out, and the motion-only
  // read that follows asks for 11 bytes, then 10 again.
  bus.pty().scriptSyncReadFaults({{emu::ReplyFault::Kind::kDrop,
                                   milliseconds(0), 0, {}}});
  CHECK_FALSE(group.readAll(fb).ok());
  CHECK(port.inRecovery());
  bus.setPositions(1900);
  REQUIRE(group.readMotionWindow(fb).ok());
  CHECK_FALSE(port.inRecovery());
  REQUIRE(group.readMotionWindow(fb).ok());
  for (std::size_t i = 0; i < kIds.size(); ++i) {
    CHECK(fb[i].position == dxl::pulseToRad(1900 + kIds[i]));
  }
  const auto arrivals = bus.waitArrivals(before + 3);
  const std::vector<emu::Arrival> after(
      arrivals.begin() + static_cast<std::ptrdiff_t>(before), arrivals.end());
  const std::vector<std::uint16_t> want{13, 11, 10};
  CHECK(syncReadLengths(after) == want);
  CHECK(port.boundedReadStats().timeouts == 1);
  CHECK(port.boundedReadStats().recoveries == 1);
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
  // instruction when it polls, at or after the reception, which is after
  // the transmission, so no allowance is due.
  const auto arrivals = bus.waitArrivals(before + 1);
  REQUIRE(arrivals.size() > before);
  for (std::size_t i = before; i < arrivals.size(); ++i) {
    CAPTURE(i, static_cast<int>(arrivals[i].instruction));
    CHECK(arrivals[i].at >= quiet_until);
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
  CHECK_THROWS_AS(
      port.armBoundedSyncRead(with([](auto& c) { c.windows = {}; })),
      std::invalid_argument);
  CHECK_THROWS_AS(
      port.armBoundedSyncRead(with([](auto& c) { c.windows[0].len = 0; })),
      std::invalid_argument);
  CHECK_THROWS_AS(port.armBoundedSyncRead(
                      with([](auto& c) { c.windows[0].len = 0xFFFF; })),
                  std::invalid_argument);
  // A reply carries no register address: windows whose lengths, or
  // recovery lengths, could be taken for each other are refused.
  for (const std::uint16_t near : {4, 5, 3}) {
    CHECK_THROWS_AS(port.armBoundedSyncRead(with([near](auto& c) {
                      c.windows.push_back({reg::kPresentVelocity.addr, near});
                    })),
                    std::invalid_argument);
  }
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

namespace {

std::size_t sentAfter(const std::vector<emu::Arrival>& arrivals,
                      Clock::time_point t) {
  std::size_t n = 0;
  for (const auto& a : arrivals) {
    if (a.at > t) ++n;
  }
  return n;
}

// The arm on the emulated bus, with the bounded read armed on its feedback
// window and a long guard, so a transmission waits in the guard for
// hundreds of milliseconds.
struct BoundedArm {
  FaultBus bus;
  dxl::Port port{bus.path(), 3000000};
  rtctrl::hw::CraneX7 arm{port,
                          rtctrl::hw::Config::load("config/crane_x7.toml")};

  explicit BoundedArm(std::chrono::nanoseconds guard) {
    REQUIRE(arm.activate());
    // SyncGroup's feedback window: 13 bytes at the start of Indirect Data.
    dxl::BoundedReadConfig config = positionRead();
    config.windows = {{reg::kIndirectDataBase, 13}};
    config.guard = guard;
    port.armBoundedSyncRead(config);
  }
};

}  // namespace

TEST_CASE("an inhibit cancels a transmission waiting in the guard and "
          "refuses every later one",
          "[pty][bounded]") {
  FaultBus bus;
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  std::atomic<bool> inhibit{false};
  port.setTransmitInhibit(&inhibit);
  dxl::BoundedReadConfig config = positionRead();
  config.guard = milliseconds(300);
  port.armBoundedSyncRead(config);
  bus.pty().scriptSyncReadFaults({{emu::ReplyFault::Kind::kDrop, {}, 0, {}}});
  std::vector<std::uint8_t> out;
  REQUIRE_FALSE(readPositions(port, out).ok());
  REQUIRE(port.quietGuardPending());
  const auto guard_end = port.quietUntil();
  const std::size_t before = bus.waitArrivals(1).size();

  // A fallback goal write waits in the guard; the inhibit lands 20 ms in.
  std::thread setter([&inhibit] {
    std::this_thread::sleep_for(milliseconds(20));
    inhibit.store(true);
  });
  const std::vector<std::uint8_t> goals(kIds.size() * 4, 0);
  const auto start = Clock::now();
  const dxl::IoResult cancelled =
      port.syncWrite(reg::kGoalPosition.addr, 4, kIds, goals);
  const auto waited = Clock::now() - start;
  setter.join();
  CHECK(cancelled.comm == dxl::kCommTransmitInhibited);
  CHECK(waited < milliseconds(100));  // the guard alone runs 330 ms
  CHECK(port.quietGuardPending());

  // Every other kind of transmission is refused as well.
  CHECK(port.write32(5, reg::kGoalPosition.addr, 2048).comm ==
        dxl::kCommTransmitInhibited);
  std::uint8_t model[2] = {};
  CHECK(port.read(5, reg::kModelNumber.addr, model, 2).comm ==
        dxl::kCommTransmitInhibited);
  CHECK(port.ping(5, nullptr).comm == dxl::kCommTransmitInhibited);
  CHECK(readPositions(port, out).comm == dxl::kCommTransmitInhibited);
  CHECK(port.boundedReadStats().inhibited == 5);

  // Past the guard's end, nothing was sent.
  std::this_thread::sleep_until(guard_end + milliseconds(20));
  CHECK(bus.pty().arrivals().size() == before);
  port.setTransmitInhibit(nullptr);
}

TEST_CASE("an inhibit refuses transmissions with no guard pending",
          "[pty][bounded]") {
  FaultBus bus;
  dxl::Port port(bus.path(), 3000000);
  std::atomic<bool> inhibit{false};
  port.setTransmitInhibit(&inhibit);
  REQUIRE(port.ping(5, nullptr).ok());
  const std::size_t before = bus.waitArrivals(1).size();
  inhibit.store(true);
  REQUIRE_FALSE(port.quietGuardPending());
  CHECK(port.write32(5, reg::kGoalPosition.addr, 2048).comm ==
        dxl::kCommTransmitInhibited);
  const std::vector<std::uint8_t> goals(kIds.size() * 4, 0);
  CHECK(port.syncWrite(reg::kGoalPosition.addr, 4, kIds, goals).comm ==
        dxl::kCommTransmitInhibited);
  std::vector<std::uint8_t> out;
  CHECK(readPositions(port, out).comm == dxl::kCommTransmitInhibited);
  std::this_thread::sleep_for(milliseconds(20));
  CHECK(bus.pty().arrivals().size() == before);
  port.setTransmitInhibit(nullptr);
  // Unregistered, the port sends again.
  CHECK(port.ping(5, nullptr).ok());
}

TEST_CASE("a quiesce stops the background fallback write waiting in the "
          "guard",
          "[pty][bounded]") {
  BoundedArm a(milliseconds(300));
  a.bus.pty().scriptSyncReadFaults(
      {{emu::ReplyFault::Kind::kDrop, {}, 0, {}}});
  const std::size_t before = a.bus.pty().arrivals().size();
  REQUIRE(a.arm.startThread());
  // The thread's first read gets no reply and times out after 30 ms; its
  // fallback goal write then waits in the guard until about 330 ms.
  a.bus.waitArrivals(before + 1);
  std::this_thread::sleep_for(milliseconds(60));
  const auto quiesce = Clock::now();
  a.arm.requestQuiesce();
  std::this_thread::sleep_for(milliseconds(350));
  a.arm.stopThread();
  CHECK(sentAfter(a.bus.pty().arrivals(), quiesce) == 0);
  CHECK(a.port.boundedReadStats().inhibited >= 1);
}

TEST_CASE("a quiesce stops the shutdown writes waiting in the guard",
          "[pty][bounded]") {
  BoundedArm a(milliseconds(300));
  a.bus.pty().scriptSyncReadFaults(
      {{emu::ReplyFault::Kind::kDrop, {}, 0, {}}});
  std::vector<dxl::Feedback> fb;
  REQUIRE_FALSE(a.arm.readAll(fb));
  REQUIRE(a.port.quietGuardPending());
  const auto guard_end = a.port.quietUntil();

  // deactivate()'s first stop write waits in the guard; a session
  // watchdog's quiesce lands 20 ms in, from another thread.
  Clock::time_point quiesce{};
  std::thread watchdog([&a, &quiesce] {
    std::this_thread::sleep_for(milliseconds(20));
    quiesce = Clock::now();
    a.arm.requestQuiesce();
  });
  const bool stopped = a.arm.deactivate();
  watchdog.join();
  CHECK_FALSE(stopped);
  std::this_thread::sleep_until(guard_end + milliseconds(20));
  CHECK(sentAfter(a.bus.pty().arrivals(), quiesce) == 0);
  CHECK(a.port.boundedReadStats().inhibited >= 1);
}

TEST_CASE("activation arms the port's bounded read only on a matching bus, "
          "and the background thread reads through it",
          "[pty][bounded]") {
  FaultBus bus;
  dxl::Port port(bus.path(), 3000000);
  rtctrl::hw::CraneX7::Options options;
  options.bounded_read.enabled = true;
  options.bounded_read.budget = milliseconds(20);
  options.bounded_read.guard = milliseconds(20);
  options.bounded_read.scheduling_allowance = milliseconds(10);
  options.bounded_read.transport_allowance = milliseconds(10);

  SECTION("Return Delay Time 0, as on the arm") {
    bus.withBus([](emu::MotorBus& b) {
      for (auto& motor : b.motors()) motor.poke(reg::kReturnDelayTime, 0);
    });
    rtctrl::hw::CraneX7 arm(
        port, rtctrl::hw::Config::load("config/crane_x7.toml"), options);
    REQUIRE(arm.activate());
    CHECK(port.boundedSyncReadArmed());
    // A pseudo-terminal exposes no latency timer: reported, unverified.
    CHECK_FALSE(arm.busDiscovery().profile.latency_timer_ms.has_value());
    REQUIRE(arm.startThread());
    std::this_thread::sleep_for(milliseconds(100));
    arm.stopThread();
    CHECK(port.boundedReadStats().reads > 0);
    CHECK(port.boundedReadStats().timeouts == 0);
    CHECK(port.boundedReadStats().malformed == 0);
    REQUIRE(arm.deactivate());
    CHECK_FALSE(port.boundedSyncReadArmed());
  }
  SECTION("the factory Return Delay Time (the emulator's default)") {
    rtctrl::hw::CraneX7 arm(
        port, rtctrl::hw::Config::load("config/crane_x7.toml"), options);
    REQUIRE_FALSE(arm.activate());
    CHECK(arm.lastError().find("Return Delay Time 250") != std::string::npos);
    CHECK_FALSE(port.boundedSyncReadArmed());
    CHECK(port.boundedReadStats().reads == 0);
  }
}

// Fast Sync Read (control-rate Phase B3) on the bounded path.

TEST_CASE("Fast Sync Read is sent only on an armed window", "[pty][fast]") {
  FaultBus bus(47);
  dxl::Port port(bus.path(), 3000000);
  std::vector<std::uint8_t> out;
  const std::size_t before = bus.pty().arrivals().size();
  CHECK(fastReadPositions(port, out).comm == dxl::kCommNotAvailable);
  // A Sync Read window does not arm the fast instruction.
  port.armBoundedSyncRead(positionRead());
  CHECK(fastReadPositions(port, out).comm == dxl::kCommNotAvailable);
  std::this_thread::sleep_for(milliseconds(20));
  CHECK(bus.pty().arrivals().size() == before);
}

TEST_CASE("an armed Fast Sync Read returns what Sync Read returns",
          "[pty][fast]") {
  FaultBus bus(47);
  bus.setPositions(2100);
  dxl::Port port(bus.path(), 3000000);
  std::vector<std::uint8_t> sdk;
  REQUIRE(readPositions(port, sdk).ok());
  // A sync and a fast window of one length may be armed together: their
  // replies differ in the first packet's ID.
  dxl::BoundedReadConfig config = fastPositionRead();
  config.windows.push_back(positionRead().windows[0]);
  port.armBoundedSyncRead(config);
  std::vector<std::uint8_t> fast;
  REQUIRE(fastReadPositions(port, fast).ok());
  CHECK(fast == sdk);
  CHECK(positions(fast) == expected(2100));
  const dxl::SyncReadTiming t = port.lastSyncReadTiming();
  CHECK(t.sent);
  CHECK(t.replies == 8);  // the servos' sections of the one packet
  std::vector<std::uint8_t> sync;
  REQUIRE(readPositions(port, sync).ok());
  CHECK(sync == sdk);
  CHECK(port.boundedReadStats().reads == 2);
  const auto arrivals = bus.waitArrivals(3);
  CHECK(syncReadLengths(arrivals, kFastSyncRead) ==
        std::vector<std::uint16_t>{4});
}

TEST_CASE("a lost Fast Sync Read reply is followed by a longer recovery "
          "read that returns fresh values",
          "[pty][fast]") {
  FaultBus bus(47);
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(fastPositionRead());
  bus.pty().scriptSyncReadFaults(
      {{emu::ReplyFault::Kind::kDrop, milliseconds(0), 0, {}}});
  std::vector<std::uint8_t> out;
  CHECK(fastReadPositions(port, out).comm != 0);
  CHECK(port.boundedReadStats().timeouts == 1);
  CHECK(port.inRecovery());
  bus.setPositions(2300);
  REQUIRE(fastReadPositions(port, out).ok());
  CHECK(positions(out) == expected(2300));
  CHECK_FALSE(port.inRecovery());
  REQUIRE(fastReadPositions(port, out).ok());
  const auto lengths = syncReadLengths(bus.waitArrivals(3), kFastSyncRead);
  CHECK(lengths == std::vector<std::uint16_t>{4, 5, 4});
  CHECK(port.boundedReadStats().recoveries == 1);
}

TEST_CASE("garbage before a Fast Sync Read reply fails it as malformed",
          "[pty][fast]") {
  FaultBus bus(47);
  bus.setPositions(2000);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(fastPositionRead());
  bus.pty().scriptSyncReadFaults(
      {{emu::ReplyFault::Kind::kGarbage, milliseconds(0), 0, {0x00, 0x13}}});
  std::vector<std::uint8_t> out;
  CHECK(fastReadPositions(port, out).comm != 0);
  CHECK(port.boundedReadStats().malformed == 1);
  CHECK(port.boundedReadStats().last_fault ==
        dxl::StatusReplyParser::Fault::kHeader);
}

TEST_CASE("servos older than Fast Sync Read leave it unanswered",
          "[pty][fast]") {
  FaultBus bus(44);
  dxl::Port port(bus.path(), 3000000);
  port.armBoundedSyncRead(fastPositionRead());
  std::vector<std::uint8_t> out;
  CHECK(fastReadPositions(port, out).comm != 0);
  CHECK(port.boundedReadStats().timeouts == 1);
}
