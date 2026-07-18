// Integration: the UNMODIFIED DynamixelSDK stack (dxl::Port) against
// the pty bus emulator at 3 Mbps.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <thread>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/dxl/sync_group.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"

using Catch::Approx;
namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace reg = rtctrl::dxl::reg;

namespace {

// Serves a CRANE-X7-shaped bus on a background thread for the duration
// of a test.
class BusFixture {
 public:
  BusFixture() {
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
        pty_->poll(0.001);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
      }
    });
  }

  ~BusFixture() {
    stop_.store(true);
    thread_.join();
  }

  const std::string& path() const { return pty_->slavePath(); }

 private:
  emu::MotorBus bus_;
  std::unique_ptr<emu::PtyBus> pty_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace

TEST_CASE("protocol helpers: crc and byte stuffing", "[pty]") {
  // CRC vector from a canonical ping frame: FF FF FD 00 01 03 00 01
  // has CRC16 0x4E19 (per the ROBOTIS protocol documentation example).
  const std::uint8_t ping[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00,
                               0x01};
  CHECK(emu::dxlCrc16(ping, sizeof(ping)) == 0x4E19);

  std::vector<std::uint8_t> payload = {0x01, 0xFF, 0xFF, 0xFD, 0x02};
  auto stuffed = payload;
  emu::dxlAddStuffing(stuffed);
  REQUIRE(stuffed.size() == payload.size() + 1);
  emu::dxlRemoveStuffing(stuffed);
  CHECK(stuffed == payload);
}

TEST_CASE("SDK port scans the emulated bus at 3 Mbps", "[pty]") {
  BusFixture fixture;
  dxl::Port port(fixture.path(), 3000000);

  int found = 0;
  for (std::uint8_t id = 1; id <= 12; ++id) {
    std::uint16_t model = 0;
    if (port.ping(id, &model).ok()) {
      ++found;
      if (id == 3) CHECK(model == dxl::kModelXm540W270);
    }
  }
  CHECK(found == 8);  // ids 2..9
}

TEST_CASE("single-register round trip through the real SDK", "[pty]") {
  BusFixture fixture;
  dxl::Port port(fixture.path(), 3000000);

  REQUIRE(port.write8(2, reg::kOperatingMode.addr, 0).ok());
  std::uint8_t mode = 0xFF;
  REQUIRE(port.read8(2, reg::kOperatingMode.addr, &mode).ok());
  CHECK(mode == 0);

  // EEPROM gating propagates through the wire protocol.
  REQUIRE(port.write8(2, reg::kTorqueEnable.addr, 1).ok());
  CHECK(port.write8(2, reg::kOperatingMode.addr, 3).error ==
        dxl::kErrAccess);
}

TEST_CASE("SyncGroup works end-to-end over the wire protocol", "[pty]") {
  BusFixture fixture;
  dxl::Port port(fixture.path(), 3000000);
  dxl::SyncGroup group(port, {2, 3, 4, 5, 6, 7, 8, 9});

  REQUIRE(group.setupIndirect().ok());
  for (const auto id : group.ids()) {
    REQUIRE(port.write8(id, reg::kTorqueEnable.addr, 1).ok());
  }

  std::vector<double> currents(8, 0.0), positions(8, 0.0);
  positions[3] = 0.7;
  REQUIRE(group.writeGoals(currents, positions).ok());
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  std::vector<dxl::Feedback> fb;
  REQUIRE(group.readAll(fb).ok());
  REQUIRE(fb.size() == 8);
  CHECK(fb[3].position == Approx(0.7).margin(5e-3));
  CHECK(fb[0].position == Approx(0.0).margin(5e-3));
  CHECK(fb[0].voltage == Approx(12.0));
}

TEST_CASE("bus watchdog fires on real bus silence", "[pty]") {
  BusFixture fixture;
  dxl::Port port(fixture.path(), 3000000);

  REQUIRE(port.write8(2, reg::kBusWatchdog.addr, 5).ok());  // 100 ms
  // stay silent past the timeout while the emulator keeps ticking
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  std::uint8_t wd = 0;
  REQUIRE(port.read8(2, reg::kBusWatchdog.addr, &wd).ok());
  CHECK(wd == dxl::kBusWatchdogTriggered);
  CHECK(port.write32(2, reg::kGoalPosition.addr, 2100).error ==
        dxl::kErrAccess);
  REQUIRE(port.write8(2, reg::kBusWatchdog.addr, 0).ok());
  CHECK(port.write32(2, reg::kGoalPosition.addr, 2100).ok());
}
