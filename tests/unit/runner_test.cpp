// The runner's measured-time base and stale-feedback policy
// (docs/REMEDIATION_PLAN.md D4), exercised with a scripted mock Arm.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <vector>

#include "rtctrl/arm/runner.hpp"

namespace arm = rtctrl::arm;
using Catch::Approx;

namespace {

// Emits a scripted sequence of (t, seq) feedback samples; the last
// entry repeats if the script runs out.
class ScriptedArm : public arm::Arm {
 public:
  struct Sample {
    double t;
    std::uint64_t seq;
  };
  explicit ScriptedArm(std::vector<Sample> script)
      : script_(std::move(script)) {}

  int dof() const override { return rtctrl::model::kCanonicalDof; }
  double dt() const override { return 0.01; }
  bool activate() override { return true; }
  bool deactivate() override { return true; }
  bool setMode(arm::ControlMode) override { return true; }
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* cmds = nullptr) override {
    const auto& s = script_[std::min(cursor_, script_.size() - 1)];
    state.t = s.t;
    state.seq = s.seq;
    if (cmds != nullptr) *cmds = arm::CommandSnapshot{};
    return true;
  }
  bool writeCommand(const arm::JointCommand&,
                    arm::CommandReceipt* receipt = nullptr) override {
    ++writes;
    if (receipt != nullptr) *receipt = {true, writes, script_[0].t};
    return true;
  }
  bool step() override {
    ++steps;
    ++cursor_;
    return true;
  }

  std::uint64_t writes = 0;
  std::uint64_t steps = 0;

 private:
  std::vector<Sample> script_;
  std::size_t cursor_ = 0;
};

struct CountingController : arm::Controller {
  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    ++updates;
    last_t = t;
    last_seq = state.seq;
    cmd.mode = arm::ControlMode::Current;
  }
  int updates = 0;
  double last_t = -1.0;
  std::uint64_t last_seq = 0;
};

}  // namespace

TEST_CASE("runner uses measured time and finishes on it", "[runner]") {
  // 10 ms nominal samples starting at an arbitrary absolute time
  std::vector<ScriptedArm::Sample> script;
  for (int k = 0; k < 20; ++k) {
    script.push_back({100.0 + 0.01 * k, static_cast<std::uint64_t>(k + 1)});
  }
  ScriptedArm robot(script);
  CountingController ctl;
  CHECK(arm::run(robot, ctl, 0.05));
  // t is relative to the first sample; the run ends when t >= 0.05
  CHECK(ctl.last_t == Approx(0.04));
  CHECK(ctl.updates == 5);
}

TEST_CASE("runner skips duplicate samples but still steps", "[runner]") {
  std::vector<ScriptedArm::Sample> script = {
      {100.00, 1}, {100.01, 2}, {100.01, 2}, {100.01, 2},
      {100.02, 3}, {100.03, 4}, {100.04, 5}, {100.05, 6}};
  ScriptedArm robot(script);
  CountingController ctl;
  CHECK(arm::run(robot, ctl, 0.045));
  CHECK(ctl.updates == 5);              // duplicates saw no update
  CHECK(robot.steps > robot.writes);    // but the arm still stepped
}

TEST_CASE("runner aborts on a backward timestamp", "[runner]") {
  std::vector<ScriptedArm::Sample> script = {
      {100.00, 1}, {100.01, 2}, {99.99, 3}};
  ScriptedArm robot(script);
  CountingController ctl;
  CHECK_FALSE(arm::run(robot, ctl, 1.0));
}

TEST_CASE("runner tolerates one missed sample, aborts on more",
          "[runner]") {
  // gap of 2 (one missed sample) is fine
  std::vector<ScriptedArm::Sample> ok = {
      {100.00, 1}, {100.01, 2}, {100.03, 4}, {100.04, 5}, {100.05, 6}};
  ScriptedArm robot_ok(ok);
  CountingController ctl_ok;
  CHECK(arm::run(robot_ok, ctl_ok, 0.045));

  // gap of 3 aborts
  std::vector<ScriptedArm::Sample> bad = {
      {100.00, 1}, {100.01, 2}, {100.04, 5}};
  ScriptedArm robot_bad(bad);
  CountingController ctl_bad;
  CHECK_FALSE(arm::run(robot_bad, ctl_bad, 1.0));
}

TEST_CASE("runner aborts on an over-age sample interval", "[runner]") {
  // seq gap of 1 but 30 ms of elapsed time: exceeds kMaxSampleInterval
  std::vector<ScriptedArm::Sample> script = {
      {100.00, 1}, {100.01, 2}, {100.04, 3}};
  ScriptedArm robot(script);
  CountingController ctl;
  CHECK_FALSE(arm::run(robot, ctl, 1.0));
}

TEST_CASE("observer veto aborts the run", "[runner]") {
  struct Veto : arm::CycleObserver {
    bool observe(double, const arm::JointState&,
                 const arm::CommandSnapshot&, const arm::JointCommand&,
                 const arm::CommandReceipt& receipt) override {
      CHECK(receipt.accepted);  // receipts flow into the observer
      return --allowed >= 0;
    }
    int allowed = 2;
  } veto;
  std::vector<ScriptedArm::Sample> script;
  for (int k = 0; k < 50; ++k) {
    script.push_back({100.0 + 0.01 * k, static_cast<std::uint64_t>(k + 1)});
  }
  ScriptedArm robot(script);
  CountingController ctl;
  CHECK_FALSE(arm::run(robot, ctl, 1.0, &veto));
  CHECK(ctl.updates == 3);  // aborted right after the veto
}
