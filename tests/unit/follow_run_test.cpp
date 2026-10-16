#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <thread>
#include <vector>

#include "follow/follow_run.hpp"

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace model = rtctrl::model;

namespace {

class IdealArm : public arm::Arm {
 public:
  explicit IdealArm(arm::ControlMode mode, double dt = 0.01)
      : mode_(mode), dt_(dt) {}
  int dof() const override { return model::kCanonicalDof; }
  double dt() const override { return dt_; }
  bool activate() override { active_ = true; return true; }
  bool deactivate() override { active_ = false; return true; }
  bool setMode(arm::ControlMode mode) override {
    if (active_) return false;
    mode_ = mode;
    return true;
  }
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* snapshot = nullptr) override {
    zVecCopyNC(state_.q.get(), state.q.get());
    zVecCopyNC(state_.dq.get(), state.dq.get());
    zVecCopyNC(state_.tau.get(), state.tau.get());
    state.t = state_.t;
    state.seq = state_.seq;
    if (snapshot != nullptr) *snapshot = snapshot_;
    return true;
  }
  bool writeCommand(const arm::JointCommand& command,
                    arm::CommandReceipt* receipt = nullptr) override {
    if (command.mode != mode_) return false;
    command_.mode = command.mode;
    zVecCopyNC(command.q.get(), command_.q.get());
    zVecCopyNC(command.dq.get(), command_.dq.get());
    zVecCopyNC(command.tau.get(), command_.tau.get());
    zVecCopyNC(command.effort_limit.get(), command_.effort_limit.get());
    ++target_seq_;
    if (receipt != nullptr) *receipt = {true, target_seq_, state_.t};
    snapshot_.applied.valid = true;
    snapshot_.applied.target_seq = target_seq_;
    snapshot_.applied.mode = static_cast<std::uint8_t>(mode_);
    return true;
  }
  bool step() override {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      state_.q[i] = command_.q[i];
      state_.dq[i] = 0.0;
    }
    state_.t += dt_;
    ++state_.seq;
    return true;
  }

 private:
  arm::ControlMode mode_;
  double dt_;
  bool active_ = false;
  arm::JointState state_;
  arm::JointCommand command_;
  arm::CommandSnapshot snapshot_;
  std::uint64_t target_seq_ = 0;
};

std::string referenceFile() {
  const std::string path = "build/follow_run_reference.zvs";
  std::ofstream out(path);
  out << "0.02 8 (0.1 0 0 0 0 0 0 0 )\n"
      << "0.02 8 (0.12 0 0 0 0 0 0 0 )\n"
      << "0.02 8 (0.14 0 0 0 0 0 0 0 )\n";
  return path;
}

follow::Config baseConfig(arm::ControlMode mode) {
  follow::Config config;
  config.mode = mode;
  config.home.motion_time = 0.02;
  config.home.velocity_limit = 2.0;
  config.home.settle_time_s = 0.01;
  config.finalization.wait_time_s = 0.01;
  config.effort_limit_set = true;
  config.effort_limit_nm.fill(1.0);
  return config;
}

}  // namespace

TEST_CASE("shared follow run completes every phase in supported servo modes",
          "[follow][run]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  for (const auto mode : {arm::ControlMode::Position,
                          arm::ControlMode::CurrentBasedPosition}) {
    IdealArm robot(mode);
    REQUIRE(robot.activate());
    auto config = baseConfig(mode);
    follow::FollowRun run(robot, reference, config, true);
    const auto result = run.run();
    CHECK(result.status == follow::RunStatus::Success);
    CHECK(result.cycles > 3);
  }
  std::remove(path.c_str());
}

TEST_CASE("strict home gate refuses a nonmoving arm", "[follow][run]") {
  class StuckArm : public IdealArm {
   public:
    StuckArm() : IdealArm(arm::ControlMode::Position) {}
    bool step() override { return true; }
  } robot;
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  auto config = baseConfig(arm::ControlMode::Position);
  config.home.correction_retries = 1;
  follow::FollowRun run(robot, reference, config, true);
  CHECK(run.run().status == follow::RunStatus::HomeNotConverged);
  std::remove(path.c_str());
}

namespace {

class CountingArm : public IdealArm {
 public:
  using IdealArm::IdealArm;
  bool writeCommand(const arm::JointCommand& command,
                    arm::CommandReceipt* receipt = nullptr) override {
    ++writes;
    return IdealArm::writeCommand(command, receipt);
  }
  bool step() override {
    ++steps;
    return IdealArm::step();
  }
  int writes = 0;
  int steps = 0;
};

struct PhaseRecorder : follow::CycleSink {
  bool record(const follow::CycleRecord& cycle) override {
    phases.push_back(cycle.phase);
    seqs.push_back(cycle.state->seq);
    return true;
  }
  std::vector<follow::Phase> phases;
  std::vector<std::uint64_t> seqs;
};

}  // namespace

TEST_CASE("follow tracking-start hook fires once before the first tracking "
          "command",
          "[follow][run][hook]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  CountingArm robot(arm::ControlMode::Position);
  REQUIRE(robot.activate());
  const auto config = baseConfig(arm::ControlMode::Position);
  PhaseRecorder sink;
  follow::FollowRun run(robot, reference, config, true, &sink);
  int calls = 0;
  std::uint64_t hook_seq = 0;
  int writes_at = -1, steps_at = -1;
  std::size_t records_at = 0;
  run.setTrackingStartHook([&](const arm::JointState& state) {
    ++calls;
    hook_seq = state.seq;
    writes_at = robot.writes;
    steps_at = robot.steps;
    records_at = sink.phases.size();
  });
  REQUIRE(run.run().status == follow::RunStatus::Success);
  CHECK(calls == 1);
  const auto first = static_cast<std::size_t>(
      std::find(sink.phases.begin(), sink.phases.end(),
                follow::Phase::Tracking) -
      sink.phases.begin());
  REQUIRE(first < sink.phases.size());
  REQUIRE(first > 0);  // homing came first
  CHECK(records_at == first);  // before the first tracking record,
  CHECK(sink.seqs[first] == hook_seq);  // with that cycle's feedback,
  CHECK(writes_at == static_cast<int>(first));  // before its command
  CHECK(steps_at == static_cast<int>(first));   // and before its step
  std::remove(path.c_str());
}

TEST_CASE("follow tracking-start hook does not fire when homing fails",
          "[follow][run][hook]") {
  class StuckArm : public IdealArm {
   public:
    StuckArm() : IdealArm(arm::ControlMode::Position) {}
    bool step() override { return true; }
  } robot;
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  auto config = baseConfig(arm::ControlMode::Position);
  config.home.correction_retries = 1;
  follow::FollowRun run(robot, reference, config, true);
  int calls = 0;
  run.setTrackingStartHook([&](const arm::JointState&) { ++calls; });
  CHECK(run.run().status == follow::RunStatus::HomeNotConverged);
  CHECK(calls == 0);
  std::remove(path.c_str());
}

TEST_CASE("follow CSV log records the run and reports an incomplete log",
          "[follow][run][log]") {
  SECTION("every cycle reaches the file") {
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);
    const auto path = referenceFile();
    model::ZvsTrajectory reference(path, map);
    const std::string log_path = "build/follow_run_log.csv";
    std::remove(log_path.c_str());
    IdealArm robot(arm::ControlMode::Position);
    REQUIRE(robot.activate());
    const auto config = baseConfig(arm::ControlMode::Position);
    follow::FollowCsvLog log(log_path);
    follow::FollowRun run(robot, reference, config, true, &log);
    const auto result = run.run();
    REQUIRE(result.status == follow::RunStatus::Success);
    REQUIRE(log.close());
    std::ifstream in(log_path);
    std::string line;
    REQUIRE(std::getline(in, line));
    CHECK(line.rfind("schema_version,time_s,phase,", 0) == 0);
    std::uint64_t rows = 0;
    while (std::getline(in, line)) {
      CHECK(line.rfind("1,", 0) == 0);
      ++rows;
    }
    CHECK(rows == result.cycles);
    std::remove(log_path.c_str());
    std::remove(path.c_str());
  }
  SECTION("a failed write fails a later record and the close") {
    std::FILE* full = std::fopen("/dev/full", "w");
    REQUIRE(full != nullptr);
    follow::FollowCsvLog log(full, rtctrl::util::AsyncWriter::AdoptStream{});
    arm::JointState state;
    arm::JointCommand command;
    model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
        ddq(model::kCanonicalDof);
    arm::CommandSnapshot snapshot;
    arm::CommandReceipt receipt;
    const follow::CycleRecord cycle{follow::Phase::Tracking, 0.0, 0.0,
                                    &state, &command, &q, &dq, &ddq,
                                    &snapshot, &receipt};
    bool refused = false;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!refused && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      refused = !log.record(cycle);
    }
    CHECK(refused);
    CHECK_FALSE(log.close());
  }
}
