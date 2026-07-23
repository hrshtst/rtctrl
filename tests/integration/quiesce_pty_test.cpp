// Real-mechanism validation of the session-watchdog quiesce path over
// the UNMODIFIED DynamixelSDK stack and the pty emulator
// (docs/IDENTIFICATION_PLAN.md verification):
//  * deterministic SDK/configuration check — a FORCED full grouped-read
//    timeout (bus made non-responsive) stays under the B_io = 50 ms
//    budget the T_quiesce arithmetic relies on;
//  * loaded integration check — request-to-traffic-silence stays under
//    B_io + B_sched with concurrent CPU load, proven by the EMULATED
//    servos' own Bus Watchdogs tripping on the ensuing silence;
//  * a quiesce landing mid-deactivate() leaves the remaining writes
//    unsent while the blocked transaction ends on its bounded timeout.
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;
using Clock = std::chrono::steady_clock;

namespace {

constexpr double kBioS = 0.050;    // longest-single-transaction budget
constexpr double kBschedS = 0.050;  // scheduling margin

double secondsSince(Clock::time_point t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

// A pausable emulated bus: pause() stops answering (the SDK then runs
// its packet timeout to expiry) while leaving the pty open.
class PausableBus {
 public:
  PausableBus() {
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
        if (!paused_.load()) pty_->poll(0.001);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
    });
  }
  ~PausableBus() {
    stop_.store(true);
    thread_.join();
  }
  const std::string& path() const { return pty_->slavePath(); }
  void pause() { paused_.store(true); }
  void resume() { paused_.store(false); }
  // Only while paused (the poll thread owns the bus otherwise).
  emu::MotorBus& bus() { return bus_; }

 private:
  emu::MotorBus bus_;
  std::unique_ptr<emu::PtyBus> pty_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> paused_{false};
};

}  // namespace

TEST_CASE("forced full grouped-read timeout stays under the B_io budget",
          "[quiesce_pty]") {
  PausableBus fixture;
  dxl::Port port(fixture.path(), 3000000);
  dxl::SyncGroup group(port, {2, 3, 4, 5, 6, 7, 8, 9});
  REQUIRE(group.setupIndirect().ok());
  std::vector<dxl::Feedback> fb;
  REQUIRE(group.readAll(fb).ok());  // healthy baseline

  fixture.pause();  // the SDK must now run its timeout to expiry
  for (int trial = 0; trial < 5; ++trial) {
    const auto t0 = Clock::now();
    const auto r = group.readAll(fb);
    const double took = secondsSince(t0);
    INFO("forced-timeout grouped read took " << 1e3 * took << " ms");
    CHECK_FALSE(r.ok());
    CHECK(took <= kBioS);
  }
  fixture.resume();
}

TEST_CASE("quiesce silences the real bus fast enough for the servo "
          "watchdogs, under CPU load",
          "[quiesce_pty]") {
  PausableBus fixture;
  auto config = hw::Config::load("config/crane_x7.toml");
  config.port = fixture.path();
  dxl::Port port(config.port, config.baudrate);
  hw::CraneX7 arm(port, config);
  REQUIRE(arm.activate());  // torque on, servo Bus Watchdogs armed
  REQUIRE(arm.startThread());
  REQUIRE(arm.waitCycle(0) > 0);

  // concurrent CPU load while the silence deadline is measured
  std::atomic<bool> stop_load{false};
  std::vector<std::thread> load;
  for (unsigned i = 0; i < std::thread::hardware_concurrency(); ++i) {
    load.emplace_back([&stop_load] {
      volatile double x = 1.0;
      while (!stop_load.load()) x = x * 1.0000001 + 1e-9;
    });
  }

  const auto t0 = Clock::now();
  arm.requestQuiesce();
  // Bus silence must arrive within B_io + B_sched; the emulated
  // servos' own Bus Watchdogs (100 ms, armed by activate) then stop
  // them with no further host action. Wait past both bounds while the
  // emulator keeps ticking, then inspect the motors.
  std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<long>(1e3 * (kBioS + kBschedS)) + 250));
  stop_load.store(true);
  for (auto& th : load) th.join();
  fixture.pause();  // take the bus to inspect motor state race-free
  for (std::uint8_t id = 2; id <= 9; ++id) {
    CHECK(fixture.bus().find(id)->peek(reg::kBusWatchdog) ==
          dxl::kBusWatchdogTriggered);
  }
  INFO("quiesce to inspection " << secondsSince(t0) << " s");
  CHECK(arm.threadRunning());   // silenced, not joined
  CHECK_FALSE(arm.escalated());  // the deadman path never ran
  // quiesced cleanup produces zero further bus operations
  CHECK_FALSE(arm.deactivate());
  CHECK_FALSE(arm.threadRunning());
}

TEST_CASE("quiesce mid-deactivate over the real SDK: remaining writes "
          "unsent, blocked transaction ends on its bounded timeout",
          "[quiesce_pty]") {
  PausableBus fixture;
  auto config = hw::Config::load("config/crane_x7.toml");
  config.port = fixture.path();
  dxl::Port port(config.port, config.baudrate);
  hw::CraneX7 arm(port, config);
  REQUIRE(arm.activate());

  // The bus stops answering (a hung shutdown); a watchdog-like second
  // thread quiesces shortly after deactivate() starts blocking.
  fixture.pause();
  std::thread killer([&arm] {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    arm.requestQuiesce();
  });
  const auto t0 = Clock::now();
  const bool ok = arm.deactivate();
  const double took = secondsSince(t0);
  killer.join();
  INFO("deactivate returned after " << took << " s");
  CHECK_FALSE(ok);
  // one or two in-flight transactions at most: the quiesce re-check
  // between transactions suppressed the rest of the ~24-write sequence
  CHECK(took <= 0.060 + 2.0 * kBioS + kBschedS);
  CHECK_FALSE(arm.activated());
  // the torque-off writes were never sent: every servo still torqued
  // (on hardware the servo Bus Watchdog owns the stop)
  for (std::uint8_t id = 2; id <= 9; ++id) {
    CHECK(fixture.bus().find(id)->peek(reg::kTorqueEnable) == 1);
  }
}
