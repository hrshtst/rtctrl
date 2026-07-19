// Integration: the background read-write thread and the RealArm bridge
// against the pty emulator through the real SDK stack.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/emu/pty_bus.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"

using Catch::Approx;
namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;

namespace {

class BusFixture {
 public:
  explicit BusFixture(const hw::Config& config) {
    for (const auto& joint : config.joints) {
      emu::MotorEmulator::Config c;
      c.id = joint.id;
      c.model_number = joint.model_number;
      bus_.add(c);
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
  emu::MotorBus& bus() { return bus_; }

 private:
  emu::MotorBus bus_;
  std::unique_ptr<emu::PtyBus> pty_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
};

}  // namespace

TEST_CASE("background thread sustains its cycle over the wire",
          "[thread]") {
  auto config = hw::Config::load("config/crane_x7.toml");
  BusFixture fixture(config);
  config.port = fixture.path();
  dxl::Port port(config.port, config.baudrate);
  hw::CraneX7::Options options;
  options.control_cycle_s = 0.005;  // 200 Hz
  hw::CraneX7 arm(port, config, options);

  REQUIRE(arm.activate());
  REQUIRE(arm.startThread());
  std::this_thread::sleep_for(std::chrono::seconds(5));
  const auto stats = arm.cycleStats();
  arm.deactivate();

  // ~1000 cycles expected at 200 Hz over 5 s; allow generous slack but
  // require a sustained loop and a small overrun budget (<2%).
  CHECK(stats.cycles > 700);
  CHECK(stats.overruns * 50 < stats.cycles);
  INFO("cycles=" << stats.cycles << " overruns=" << stats.overruns);
}

TEST_CASE("RealArm drives the bridge through the wire protocol",
          "[thread]") {
  auto config = hw::Config::load("config/crane_x7.toml");
  BusFixture fixture(config);
  config.port = fixture.path();
  dxl::Port port(config.port, config.baudrate);
  hw::CraneX7 hw_arm(port, config);
  rtctrl::arm::RealArm arm(hw_arm);

  REQUIRE(arm.setMode(rtctrl::arm::ControlMode::Position));
  REQUIRE(arm.activate());

  // Command a small step on the wrist through the bridge and let the
  // emulator's servo model track it.
  struct StepController : rtctrl::arm::Controller {
    void update(const rtctrl::arm::JointState& state,
                rtctrl::arm::JointCommand& cmd, double t) override {
      (void)t;
      cmd.mode = rtctrl::arm::ControlMode::Position;
      zVecCopyNC(state.q.get(), cmd.q.get());
      zVecElemNC(cmd.q.get(), 6) = 0.4;
    }
  } controller;
  REQUIRE(rtctrl::arm::run(arm, controller, 1.5));

  rtctrl::arm::JointState state;
  REQUIRE(arm.readState(state));
  CHECK(state.q[6] == Approx(0.4).margin(0.01));
  REQUIRE(arm.deactivate());
}

TEST_CASE("stopThread zeroes velocity goals", "[thread]") {
  auto config = hw::Config::load("config/crane_x7.toml");
  for (auto& joint : config.joints) joint.operating_mode = 1;
  BusFixture fixture(config);
  config.port = fixture.path();
  dxl::Port port(config.port, config.baudrate);
  hw::CraneX7 arm(port, config);

  REQUIRE(arm.activate());
  REQUIRE(arm.startThread());
  arm.setTargetVelocities(std::vector<double>(8, 0.5));
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  arm.stopThread();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto goal = static_cast<std::int32_t>(
      fixture.bus().find(2)->peek(reg::kGoalVelocity));
  CHECK(goal == 0);
  arm.deactivate();
}
