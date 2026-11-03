// The runner's measured-time base and stale-feedback policy
// (docs/records/history.md (pass-1 record) D4), exercised with a scripted mock Arm.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <limits>
#include <string>
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
    if (cursor_ == fail_read_at) return false;
    const auto& s = script_[std::min(cursor_, script_.size() - 1)];
    state.t = s.t;
    state.seq = s.seq;
    if (cmds != nullptr) *cmds = arm::CommandSnapshot{};
    return true;
  }
  bool writeCommand(const arm::JointCommand&,
                    arm::CommandReceipt* receipt = nullptr) override {
    if (cursor_ == reject_write_at) return false;
    ++writes;
    if (receipt != nullptr) *receipt = {true, writes, script_[0].t};
    return true;
  }
  bool step() override {
    if (cursor_ == fail_step_at) return false;
    ++steps;
    ++cursor_;
    return true;
  }

  std::uint64_t writes = 0;
  std::uint64_t steps = 0;
  // Script positions (cycles, one step each) at which a call fails.
  static constexpr std::size_t kNever = static_cast<std::size_t>(-1);
  std::size_t fail_read_at = kNever;
  std::size_t reject_write_at = kNever;
  std::size_t fail_step_at = kNever;

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

TEST_CASE("runner rejects non-finite and nonpositive durations before "
          "touching the arm",
          "[runner]") {
  // NaN survives std::min and a sufficiently negative duration casts
  // out of long's range — both undefined (review finding). The reject
  // must precede ANY arm interaction.
  ScriptedArm robot({{100.0, 1}});
  CountingController ctl;
  for (const double bad :
       {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(), 0.0, -5.0}) {
    arm::RunReport report;
    CHECK_FALSE(arm::run(robot, ctl, bad, nullptr, &report));
    CHECK(report.end == arm::RunEnd::kInvalidDuration);
    CHECK(report.cycle == -1);
  }
  CHECK(robot.writes == 0);
  CHECK(robot.steps == 0);
  CHECK(ctl.updates == 0);
}

TEST_CASE("runner uses measured time and finishes on it", "[runner]") {
  // 10 ms nominal samples starting at an arbitrary absolute time
  std::vector<ScriptedArm::Sample> script;
  for (int k = 0; k < 20; ++k) {
    script.push_back({100.0 + 0.01 * k, static_cast<std::uint64_t>(k + 1)});
  }
  ScriptedArm robot(script);
  CountingController ctl;
  arm::RunReport report;
  CHECK(arm::run(robot, ctl, 0.05, nullptr, &report));
  // the reference read is controlled at t = 0; the origin latches on
  // the first FRESH sample, and the run ends when measured t >= 0.05
  CHECK(ctl.last_t == Approx(0.04));
  CHECK(ctl.updates == 6);  // reference cycle + five measured cycles
  CHECK(report.end == arm::RunEnd::kCompleted);
  CHECK(report.cycle == 6);
  CHECK(report.accepted_seq == 7);
  CHECK(report.accepted_sample_t == Approx(100.06));
  CHECK(report.run_t == Approx(0.05));
  CHECK(report.final_seq == 7);
  CHECK(report.duplicates == 0);
  // The report is optional: the same run without one returns the same.
  ScriptedArm again(script);
  CountingController ctl_again;
  CHECK(arm::run(again, ctl_again, 0.05));
}

TEST_CASE("runner skips duplicate samples but still steps", "[runner]") {
  std::vector<ScriptedArm::Sample> script = {
      {100.00, 1}, {100.01, 2}, {100.01, 2}, {100.01, 2}, {100.02, 3},
      {100.03, 4}, {100.04, 5}, {100.05, 6}, {100.06, 7}};
  ScriptedArm robot(script);
  CountingController ctl;
  arm::RunReport report;
  CHECK(arm::run(robot, ctl, 0.045, nullptr, &report));
  CHECK(ctl.updates == 6);              // duplicates saw no update
  CHECK(robot.steps > robot.writes);    // but the arm still stepped
  CHECK(report.duplicates == 2);
}

TEST_CASE("runner aborts on a backward timestamp", "[runner]") {
  std::vector<ScriptedArm::Sample> script = {
      {100.00, 1}, {100.01, 2}, {99.99, 3}};
  ScriptedArm robot(script);
  CountingController ctl;
  arm::RunReport report;
  CHECK_FALSE(arm::run(robot, ctl, 1.0, nullptr, &report));
  CHECK(report.end == arm::RunEnd::kBackwardClock);
  CHECK(report.cycle == 2);
  CHECK(report.accepted_seq == 2);
  CHECK(report.accepted_sample_t == Approx(100.01));
  CHECK(report.final_seq == 3);
  CHECK(report.final_sample_t == Approx(99.99));
}

TEST_CASE("runner tolerates one missed sample, aborts on more",
          "[runner]") {
  // gap of 2 (one missed sample) is fine
  std::vector<ScriptedArm::Sample> ok = {
      {100.00, 1}, {100.01, 2}, {100.03, 4},
      {100.04, 5}, {100.05, 6}, {100.06, 7}};
  ScriptedArm robot_ok(ok);
  CountingController ctl_ok;
  CHECK(arm::run(robot_ok, ctl_ok, 0.045));

  // gap of 3 aborts
  std::vector<ScriptedArm::Sample> bad = {
      {100.00, 1}, {100.01, 2}, {100.04, 5}};
  ScriptedArm robot_bad(bad);
  CountingController ctl_bad;
  arm::RunReport report;
  CHECK_FALSE(arm::run(robot_bad, ctl_bad, 1.0, nullptr, &report));
  CHECK(report.end == arm::RunEnd::kSequenceGap);
  CHECK(report.cycle == 2);
  CHECK(report.accepted_seq == 2);
  CHECK(report.final_seq == 5);
  CHECK(report.final_sample_t == Approx(100.04));
}

TEST_CASE("runner aborts on an over-age sample interval", "[runner]") {
  // seq gap of 1 but 30 ms of elapsed time: exceeds kMaxSampleInterval
  std::vector<ScriptedArm::Sample> script = {
      {100.00, 1}, {100.01, 2}, {100.04, 3}};
  ScriptedArm robot(script);
  CountingController ctl;
  arm::RunReport report;
  CHECK_FALSE(arm::run(robot, ctl, 1.0, nullptr, &report));
  CHECK(report.end == arm::RunEnd::kSampleInterval);
  CHECK(report.accepted_sample_t == Approx(100.01));
  CHECK(report.final_sample_t == Approx(100.04));
  CHECK(report.final_seq == 3);
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
  arm::RunReport report;
  CHECK_FALSE(arm::run(robot, ctl, 1.0, &veto, &report));
  CHECK(ctl.updates == 3);  // aborted right after the veto
  CHECK(report.end == arm::RunEnd::kObserverVeto);
  CHECK(report.cycle == 2);
  CHECK(report.accepted_seq == 3);
}

TEST_CASE("runner reports a failed read, a rejected command and a failed "
          "step",
          "[runner]") {
  std::vector<ScriptedArm::Sample> script;
  for (int k = 0; k < 50; ++k) {
    script.push_back({100.0 + 0.01 * k, static_cast<std::uint64_t>(k + 1)});
  }
  CountingController ctl;
  {
    ScriptedArm robot(script);
    robot.fail_read_at = 4;
    arm::RunReport report;
    CHECK_FALSE(arm::run(robot, ctl, 1.0, nullptr, &report));
    CHECK(report.end == arm::RunEnd::kReadFailed);
    CHECK(report.cycle == 4);
    // The last accepted sample is the one before the failed read.
    CHECK(report.accepted_seq == 4);
    CHECK(report.accepted_sample_t == Approx(100.03));
    CHECK(report.run_t == Approx(0.02));
  }
  {
    ScriptedArm robot(script);
    robot.reject_write_at = 3;
    arm::RunReport report;
    CHECK_FALSE(arm::run(robot, ctl, 1.0, nullptr, &report));
    CHECK(report.end == arm::RunEnd::kCommandRejected);
    CHECK(report.cycle == 3);
    CHECK(report.accepted_seq == 4);
  }
  {
    ScriptedArm robot(script);
    robot.fail_step_at = 5;
    arm::RunReport report;
    CHECK_FALSE(arm::run(robot, ctl, 1.0, nullptr, &report));
    CHECK(report.end == arm::RunEnd::kStepFailed);
    CHECK(report.cycle == 5);
    CHECK(report.accepted_seq == 6);
  }
  {
    // A failed step on a duplicate sample is reported the same way.
    ScriptedArm robot({{100.00, 1}, {100.01, 2}, {100.01, 2}});
    robot.fail_step_at = 2;
    arm::RunReport report;
    CHECK_FALSE(arm::run(robot, ctl, 1.0, nullptr, &report));
    CHECK(report.end == arm::RunEnd::kStepFailed);
    CHECK(report.duplicates == 1);
  }
}

TEST_CASE("runner reports the cycle cap", "[runner]") {
  // The sequence never advances, so measured time never reaches the
  // duration: the iteration backstop ends the run.
  ScriptedArm robot({{100.0, 1}});
  CountingController ctl;
  arm::RunReport report;
  CHECK_FALSE(arm::run(robot, ctl, 0.05, nullptr, &report));
  CHECK(report.end == arm::RunEnd::kCycleCap);
  CHECK(report.cycle == 28);  // 4 x 0.05 / 0.01 + 8
  CHECK(report.duplicates == 27);
  CHECK(report.run_t == 0.0);  // only the reference cycle was accepted
}

TEST_CASE("runner describes every end", "[runner]") {
  for (const auto end :
       {arm::RunEnd::kCompleted, arm::RunEnd::kInvalidDuration,
        arm::RunEnd::kReadFailed, arm::RunEnd::kBackwardClock,
        arm::RunEnd::kSequenceGap, arm::RunEnd::kSampleInterval,
        arm::RunEnd::kCommandRejected, arm::RunEnd::kObserverVeto,
        arm::RunEnd::kStepFailed, arm::RunEnd::kCycleCap}) {
    CHECK(std::string(arm::describe(end)) != "unknown");
  }
}
