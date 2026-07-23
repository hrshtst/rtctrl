#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fstream>
#include <sstream>

#include "rtctrl/dxl/control_table.hpp"
#include "rtctrl/dxl/conversions.hpp"
#include "rtctrl/emu/fake_packet_io.hpp"
#include "rtctrl/emu/motor_emulator.hpp"
#include "rtctrl/hw/config.hpp"
#include "rtctrl/hw/crane_x7.hpp"
#include "rtctrl/model/joint_map.hpp"

using Catch::Approx;
namespace dxl = rtctrl::dxl;
namespace emu = rtctrl::emu;
namespace hw = rtctrl::hw;
namespace reg = rtctrl::dxl::reg;

namespace {

hw::Config craneConfig() { return hw::Config::load("config/crane_x7.toml"); }

emu::MotorBus busFor(const hw::Config& config, std::uint8_t firmware = 44) {
  emu::MotorBus bus;
  for (const auto& joint : config.joints) {
    emu::MotorEmulator::Config c;
    c.id = joint.id;
    c.model_number = joint.model_number;
    c.firmware_version = firmware;
    bus.add(c);
  }
  return bus;
}

}  // namespace

TEST_CASE("config TOML loads with bus settings and canonical joints",
          "[hw]") {
  const auto config = craneConfig();
  CHECK(config.port == "/dev/ttyUSB0");
  CHECK(config.baudrate == 3000000);
  REQUIRE(config.joints.size() == 8);
  CHECK(config.joints[0].id == 2);
  CHECK(config.joints[1].model_number == dxl::kModelXm540W270);
  CHECK(config.joints[7].id == 9);
}

TEST_CASE("activation sequence arms safety and causes no motion", "[hw]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);

  // servo sitting away from home: activation must not move it
  bus.find(4)->poke(reg::kPresentPosition, 1500);

  REQUIRE(arm.activate());
  for (const auto& joint : config.joints) {
    auto* motor = bus.find(joint.id);
    CHECK(motor->peek(reg::kTorqueEnable) == 1);
    CHECK(motor->peek(reg::kBusWatchdog) == 5);  // 100 ms / 20 ms units
    CHECK(motor->peek(reg::kPositionPGain) == 800);
    CHECK(motor->peek(reg::kOperatingMode) == joint.operating_mode);
    CHECK(motor->peek(reg::kGoalPosition) ==
          motor->peek(reg::kPresentPosition));
  }
  // keep the cycle alive (reads feed the armed Bus Watchdog) while
  // checking that nothing moves
  std::vector<dxl::Feedback> fb;
  for (int i = 0; i < 50; ++i) {
    REQUIRE(arm.readAll(fb));
    bus.tick(0.01);
  }
  CHECK(bus.find(4)->peek(reg::kPresentPosition) == 1500);  // did not move

  REQUIRE(arm.deactivate());
  for (const auto& joint : config.joints) {
    auto* motor = bus.find(joint.id);
    CHECK(motor->peek(reg::kTorqueEnable) == 0);
    CHECK(motor->peek(reg::kBusWatchdog) == 0);
    CHECK(motor->peek(reg::kPositionPGain) == 5);  // limp
  }
}

TEST_CASE("activation fails on firmware without Bus Watchdog", "[hw]") {
  const auto config = craneConfig();
  auto bus = busFor(config, /*firmware=*/37);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  CHECK_FALSE(arm.activate());
  CHECK(arm.lastError().find("firmware") != std::string::npos);
  // and nothing was torqued on
  CHECK(bus.find(2)->peek(reg::kTorqueEnable) == 0);
}

TEST_CASE("activation fails on model mismatch", "[hw]") {
  auto config = craneConfig();
  auto bus = busFor(config);
  config.joints[2].model_number = dxl::kModelXm540W270;  // wrong on purpose
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  CHECK_FALSE(arm.activate());
  CHECK(arm.lastError().find("model") != std::string::npos);
}

TEST_CASE("position commands clamp to servo limits minus margin", "[hw]") {
  auto config = craneConfig();
  for (auto& joint : config.joints) joint.pos_limit_margin = 0.1;
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());

  // command far beyond the +pi position limit
  std::vector<double> target(8, 10.0);
  REQUIRE(arm.writePositions(target));
  const auto goal = bus.find(2)->peek(reg::kGoalPosition);
  const double goal_rad = dxl::pulseToRad(static_cast<std::int32_t>(goal));
  const double servo_hi = dxl::pulseToRad(4095);
  CHECK(goal_rad == Approx(servo_hi - 0.1).margin(2e-3));
}

TEST_CASE(
    "deadman escalation: stalled writes with healthy reads end with the "
    "motors actually stopped",
    "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.host_command_timeout_s = 0.05;
  hw::CraneX7 arm(io, config, options);

  double sim_time = 0.0;
  arm.setTimeSource([&sim_time] { return sim_time; });
  auto tick = [&](double dt) {
    bus.tick(dt);
    sim_time += dt;
  };

  bool bus_silenced = false;
  arm.onEscalate([&bus_silenced] { bus_silenced = true; });

  REQUIRE(arm.activate());
  // start a long move so the motor is visibly in motion
  std::vector<double> target(8, 0.0);
  target[0] = 2.0;
  REQUIRE(arm.writePositions(target));
  tick(0.01);
  REQUIRE(bus.find(2)->moving());

  // Now the write path dies while reads stay healthy — the servo-side
  // watchdog alone would never fire.
  io.setWriteFailure(-3001);
  std::vector<dxl::Feedback> fb;
  bool escalated = false;
  for (int cycle = 0; cycle < 100; ++cycle) {
    REQUIRE(arm.readAll(fb));               // reads keep succeeding
    arm.writePositions(target);             // fails silently now
    tick(0.01);
    if (!arm.checkDeadman()) {
      escalated = true;
      break;
    }
  }
  REQUIRE(escalated);
  CHECK(bus_silenced);  // the escalation hook silenced the bus

  // ACCEPTANCE IS THE MOTOR'S END STATE: after escalation the host is
  // silent; the servo Bus Watchdog fires and motion stops even though
  // the best-effort torque-off writes were failing.
  for (int i = 0; i < 30; ++i) tick(0.01);
  auto* motor = bus.find(2);
  CHECK(motor->watchdogTriggered());
  const auto pos_after_trigger = motor->peek(reg::kPresentPosition);
  for (int i = 0; i < 30; ++i) tick(0.01);
  CHECK(motor->peek(reg::kPresentPosition) == pos_after_trigger);  // stopped
  // and the arm object refuses further commands
  CHECK_FALSE(arm.writePositions(target));
}

TEST_CASE("activation recovers from a previously triggered bus watchdog",
          "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);

  // Trip every watchdog the way a real USB pull does: torqued, armed,
  // then silence past the timeout (the watchdog only counts while
  // torque is enabled, as on the real servos).
  for (const auto& joint : config.joints) {
    REQUIRE(io.write8(joint.id, reg::kTorqueEnable.addr, 1).ok());
    REQUIRE(io.write8(joint.id, reg::kBusWatchdog.addr, 5).ok());
  }
  bus.tick(0.2);
  REQUIRE(bus.find(2)->watchdogTriggered());

  // Goal writes are locked out — and activation must recover anyway
  // (verified on hardware after a real cable-pull drill).
  REQUIRE(arm.activate());
  CHECK_FALSE(bus.find(2)->watchdogTriggered());
  CHECK(bus.find(2)->peek(reg::kTorqueEnable) == 1);
  CHECK(bus.find(2)->peek(reg::kBusWatchdog) == 5);  // re-armed
  REQUIRE(arm.deactivate());
}

TEST_CASE("deadman does not fire while commands flow", "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.host_command_timeout_s = 0.05;
  hw::CraneX7 arm(io, config, options);
  double sim_time = 0.0;
  arm.setTimeSource([&sim_time] { return sim_time; });
  REQUIRE(arm.activate());

  std::vector<dxl::Feedback> fb;
  REQUIRE(arm.readAll(fb));
  std::vector<double> hold(8);
  for (int i = 0; i < 8; ++i) hold[i] = fb[i].position;
  for (int cycle = 0; cycle < 50; ++cycle) {
    REQUIRE(arm.writePositions(hold));
    bus.tick(0.001);
    sim_time += 0.001;
    REQUIRE(arm.checkDeadman());
  }
  CHECK_FALSE(arm.escalated());
}

TEST_CASE(
    "persistent read failure escalates instead of serving frozen feedback",
    "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.control_cycle_s = 0.001;  // fast loop, short test
  hw::CraneX7 arm(io, config, options);

  bool bus_silenced = false;
  arm.onEscalate([&bus_silenced] { bus_silenced = true; });

  REQUIRE(arm.activate());
  REQUIRE(arm.startThread());

  // healthy loop first: cycles advance, feedback flows
  const auto seq = arm.waitCycle(0);
  REQUIRE(seq > 0);
  REQUIRE(arm.lastFeedback().size() == config.joints.size());

  // Now the read path dies while writes stay "healthy" (sync writes are
  // broadcast — they succeed with nobody listening). Observed on
  // hardware 2026-07-21: feedback froze mid-run and the controller kept
  // commanding torques blind for 3 s while the app reported success.
  io.setReadFailure(-3001);

  // the thread must escalate and stop within the failure budget
  std::uint64_t last = seq;
  for (int i = 0; i < 1000 && arm.waitCycle(last) != 0; ++i) {
    last = arm.cycleStats().cycles;
  }
  CHECK(arm.escalated());
  CHECK(bus_silenced);
  CHECK(arm.cycleStats().read_failures >=
        static_cast<std::uint64_t>(options.max_read_failures));
  // waitCycle now reports the stopped thread: RealArm::step -> false,
  // so a running controller aborts instead of finishing on stale state.
  CHECK(arm.waitCycle(last) == 0);
  // and the arm refuses further commands
  std::vector<double> zeros(config.joints.size(), 0.0);
  CHECK_FALSE(arm.writePositions(zeros));
  arm.stopThread();  // join the exited loop
}

TEST_CASE("feedback wraps multi-turn present position to principal angle",
          "[hw]") {
  auto config = craneConfig();
  for (auto& joint : config.joints) joint.operating_mode = 0;  // current
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());

  // In current/velocity modes the servo reports multi-turn position:
  // hand-moving a limp joint across the encoder boundary leaves a 2*pi
  // offset (2026-07-21 hardware: twist read +6.54 rad and its soft
  // limit gate then blocked every positive current). 0.259 rad + one
  // full turn = 6313 pulses.
  const double physical = 0.2592;
  bus.find(4)->poke(reg::kPresentPosition,
                    static_cast<std::uint32_t>(dxl::radToPulse(physical) +
                                               4096));

  std::vector<dxl::Feedback> fb;
  REQUIRE(arm.readAll(fb));
  CHECK(fb[2].position == Approx(physical).margin(2e-3));
  // and the soft limit gate sees the true angle: positive current on
  // the twist (canonical 2, well inside its range) must pass through
  std::vector<double> amps(config.joints.size(), 0.0);
  amps[2] = 0.5;
  REQUIRE(arm.writeCurrents(amps));
  const auto goal = bus.find(4)->peek(reg::kGoalCurrent);
  CHECK(static_cast<std::int16_t>(goal) > 0);
}

TEST_CASE("deadman escalates when the controller stops submitting targets",
          "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7::Options options;
  options.control_cycle_s = 0.001;
  options.host_command_timeout_s = 0.05;
  hw::CraneX7 arm(io, config, options);

  bool bus_silenced = false;
  arm.onEscalate([&bus_silenced] { bus_silenced = true; });

  REQUIRE(arm.activate());
  REQUIRE(arm.startThread());

  // A live controller: fresh submissions keep the deadman fed well past
  // the timeout. (The thread's OWN retransmissions must not count —
  // that was the trap: a frozen controller left the last command active
  // forever while every write "succeeded".)
  std::vector<double> hold(config.joints.size(), 0.0);
  {
    const auto fb = arm.lastFeedback();
    for (std::size_t i = 0; i < fb.size(); ++i) hold[i] = fb[i].position;
  }
  for (int i = 0; i < 20; ++i) {
    REQUIRE(arm.setTargetPositions(hold));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  CHECK_FALSE(arm.escalated());

  // The controller dies; the thread keeps retransmitting successfully.
  std::uint64_t last = 0;
  for (int i = 0; i < 1000 && arm.waitCycle(last) != 0; ++i) {
    last = arm.cycleStats().cycles;
  }
  CHECK(arm.escalated());
  CHECK(bus_silenced);
  arm.stopThread();
}

TEST_CASE("activation failure mid-torque-on releases every servo",
          "[hw][safety]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  // the LAST servo refuses torque-on: ids 2..8 are already torqued when
  // the sequence fails
  io.setWriteFailureOn(9, reg::kTorqueEnable.addr, 1, -3001);
  hw::CraneX7 arm(io, config);

  CHECK_FALSE(arm.activate());
  CHECK(arm.lastError().find("torque-on") != std::string::npos);
  for (const auto& joint : config.joints) {
    auto* motor = bus.find(joint.id);
    CHECK(motor->peek(reg::kTorqueEnable) == 0);   // rolled back
    CHECK(motor->peek(reg::kBusWatchdog) == 0);    // disarmed
  }
  CHECK_FALSE(arm.activated());
}

TEST_CASE("destruction joins a still-running background thread", "[hw]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  {
    hw::CraneX7 arm(io, config);
    REQUIRE(arm.activate());
    REQUIRE(arm.startThread());
    // leaving scope without stopThread/deactivate must join, not
    // std::terminate
  }
  SUCCEED("destructor joined the thread");
}

TEST_CASE("command vectors must match the joint count", "[hw]") {
  auto config = craneConfig();
  for (auto& joint : config.joints) joint.operating_mode = 0;  // current
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  REQUIRE(arm.activate());

  std::vector<double> nine(9, 0.0), seven(7, 0.0);
  CHECK_FALSE(arm.writeCurrents(nine));   // would index limits OOB
  CHECK_FALSE(arm.writeCurrents(seven));
  CHECK_FALSE(arm.setTargetCurrents(nine));
  CHECK(arm.lastError().find("rejected") != std::string::npos);
  std::vector<double> eight(8, 0.0);
  CHECK(arm.writeCurrents(eight));
}

namespace {

// A canonical-order TOML with one deliberate deviation per parameter.
std::string configToml(std::size_t count, int bad_id_at = -1,
                       int bad_id_value = 42, bool swap_first_two = false,
                       const char* model_at_2 = nullptr, int mode = 3) {
  const auto& canonical = rtctrl::model::canonicalJoints();
  std::ostringstream out;
  out << "[bus]\nport = \"/dev/null\"\nbaudrate = 3000000\n";
  for (std::size_t i = 0; i < count; ++i) {
    std::size_t src = i;
    if (swap_first_two && i < 2) src = 1 - i;
    int id = canonical[i].dxl_id;
    if (static_cast<int>(i) == bad_id_at) id = bad_id_value;
    const char* model = (i == 1) ? "XM540-W270" : "XM430-W350";
    if (model_at_2 != nullptr && i == 2) model = model_at_2;
    out << "[[joint]]\nname = \"" << canonical[src].urdf_joint << "\"\n"
        << "id = " << id << "\nmodel = \"" << model << "\"\n"
        << "operating_mode = " << mode << "\nvelocity_limit = 4.8\n"
        << "effort_limit = 4.0\npos_limit_margin = 0.0\n"
        << "current_limit_margin = 0.5\n";
  }
  return out.str();
}

std::string writeTempConfig(const std::string& text) {
  const std::string path = "build/hw_test_config.toml";
  std::ofstream f(path);
  f << text;
  return path;
}

}  // namespace

TEST_CASE("config validation rejects non-canonical deployments", "[hw]") {
  // A misordered or mistyped config would silently map controller
  // outputs onto the wrong servos — every deviation must throw.
  CHECK_THROWS(hw::Config::load(writeTempConfig(configToml(7))));
  CHECK_THROWS(hw::Config::load(writeTempConfig(configToml(8, /*bad_id*/ 3))));
  CHECK_THROWS(hw::Config::load(
      writeTempConfig(configToml(8, -1, 42, /*swap*/ true))));
  CHECK_THROWS(hw::Config::load(
      writeTempConfig(configToml(8, -1, 42, false, "XM999-W000"))));
  CHECK_THROWS(hw::Config::load(
      writeTempConfig(configToml(8, -1, 42, false, nullptr, /*mode*/ 2))));
  // Out-of-range raw values must not wrap into valid ones during the
  // int -> uint8 narrowing: id 258 would otherwise become a legal 2,
  // mode 259 a legal 3.
  CHECK_THROWS(hw::Config::load(
      writeTempConfig(configToml(8, /*at*/ 0, /*id*/ 258))));
  CHECK_THROWS(hw::Config::load(
      writeTempConfig(configToml(8, -1, 42, false, nullptr, /*mode*/ 259))));
  CHECK_NOTHROW(hw::Config::load(writeTempConfig(configToml(8))));
}

TEST_CASE("CraneX7 re-validates a hand-built config", "[hw]") {
  // Config is a plain struct: a caller can bypass load() entirely, so
  // the constructor must enforce the same invariant.
  auto config = craneConfig();
  config.joints.pop_back();
  emu::MotorBus bus = busFor(config);
  emu::FakePacketIO io(bus);
  CHECK_THROWS(hw::CraneX7(io, config));

  // unknown model_number would silently get XM430 torque scaling in
  // torqueConstant() — must be rejected too
  auto config2 = craneConfig();
  config2.joints[4].model_number = 9999;
  emu::MotorBus bus2 = busFor(config2);
  emu::FakePacketIO io2(bus2);
  CHECK_THROWS(hw::CraneX7(io2, config2));
}

TEST_CASE("writers reject commands before activation", "[hw]") {
  // The soft-limit arrays are read from the servos during activation:
  // a pre-activation command would index them empty.
  auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  std::vector<double> eight(8, 0.0);
  CHECK_FALSE(arm.writePositions(eight));
  CHECK(arm.lastError().find("not activated") != std::string::npos);
}

TEST_CASE("feedback carries a coherent acquisition stamp and sequence",
          "[hw]") {
  const auto config = craneConfig();
  auto bus = busFor(config);
  emu::FakePacketIO io(bus);
  hw::CraneX7 arm(io, config);
  double sim_time = 100.0;
  arm.setTimeSource([&sim_time] { return sim_time; });
  REQUIRE(arm.activate());  // performs a readAll internally

  std::vector<dxl::Feedback> fb;
  sim_time = 101.5;
  REQUIRE(arm.readAll(fb));
  const auto s1 = arm.lastFeedbackStamped();
  CHECK(s1.time == Approx(101.5));  // follows the injected clock
  REQUIRE(s1.seq >= 1);

  sim_time = 101.51;
  REQUIRE(arm.readAll(fb));
  const auto s2 = arm.lastFeedbackStamped();
  CHECK(s2.seq == s1.seq + 1);
  CHECK(s2.time == Approx(101.51));

  // a failed read freezes stamp, sequence, and values alike
  io.setReadFailure(-3001);
  sim_time = 102.0;
  CHECK_FALSE(arm.readAll(fb));
  const auto s3 = arm.lastFeedbackStamped();
  CHECK(s3.seq == s2.seq);
  CHECK(s3.time == Approx(101.51));
}
