#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
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

TEST_CASE("a posture-matched run refuses before any motion if the arm "
          "is away from the reference start",
          "[follow][run][posture]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();  // starts 0.1 rad away on joint 0
  model::ZvsTrajectory reference(path, map);
  IdealArm robot(arm::ControlMode::Position);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.posture = follow::PostureRunConfig{0.005, 0.05};
  follow::FollowRun run(robot, reference, config, true);
  const auto result = run.run();
  CHECK(result.status == follow::RunStatus::HomeRefused);
  CHECK(result.worst_home_error_rad == 0.1);
  // Only the hold at the arm's own position was ever commanded.
  arm::JointState state;
  REQUIRE(robot.readState(state));
  for (int i = 0; i < model::kCanonicalDof; ++i) CHECK(state.q[i] == 0.0);
  std::remove(path.c_str());
}

TEST_CASE("a posture-matched run proceeds without homing when the arm is "
          "at the reference start",
          "[follow][run][posture]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const std::string path = "build/follow_run_matched.zvs";
  {
    std::ofstream out(path);
    out << "0.02 8 (0 0 0 0 0 0 0 0 )\n"
        << "0.02 8 (0.002 0 0 0 0 0 0 0 )\n"
        << "0.02 8 (0.004 0 0 0 0 0 0 0 )\n";
  }
  model::ZvsTrajectory reference(path, map);
  IdealArm robot(arm::ControlMode::Position);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.posture = follow::PostureRunConfig{0.005, 0.05};
  follow::FollowRun run(robot, reference, config, true);
  const auto result = run.run();
  CHECK(result.status == follow::RunStatus::Success);
  CHECK(result.worst_home_error_rad <= 0.005);
  std::remove(path.c_str());
}

namespace {

std::string matchedReferenceFile(const std::string& path) {
  std::ofstream out(path);
  for (int frame = 0; frame < 20; ++frame) {
    out << "0.02 8 (0 0 0 0 0 0 0 0 )\n";
  }
  return path;
}

// Reports the commanded posture plus a displacement on joint 0 from the
// given step on: something moved the arm after the start check passed.
class DisplacedArm : public IdealArm {
 public:
  DisplacedArm(int from_step, double displacement)
      : IdealArm(arm::ControlMode::Position),
        from_step_(from_step),
        displacement_(displacement) {}
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* snapshot = nullptr) override {
    if (!IdealArm::readState(state, snapshot)) return false;
    if (steps_ >= from_step_) state.q[0] += displacement_;
    return true;
  }
  bool step() override {
    ++steps_;
    return IdealArm::step();
  }

 private:
  int from_step_;
  double displacement_;
  int steps_ = 0;
};

struct CommandRecorder : follow::CycleSink {
  bool record(const follow::CycleRecord& cycle) override {
    phases.push_back(cycle.phase);
    std::array<double, model::kCanonicalDof> c{}, r{}, q{};
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      c[i] = cycle.command->q[i];
      r[i] = (*cycle.reference_q)[i];
      q[i] = cycle.state->q[i];
    }
    commands.push_back(c);
    references.push_back(r);
    measured.push_back(q);
    return true;
  }
  std::vector<follow::Phase> phases;
  std::vector<std::array<double, model::kCanonicalDof>> commands,
      references, measured;
};

}  // namespace

TEST_CASE("a posture run commands only the reference, never a homing "
          "correction",
          "[follow][run][posture][safety]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = matchedReferenceFile("build/follow_run_posture.zvs");
  model::ZvsTrajectory reference(path, map);
  // Matched exactly at the start, then displaced 0.02 rad: a homing pass
  // would correct for it with a 0.02 rad command offset.
  DisplacedArm robot(2, 0.02);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.posture = follow::PostureRunConfig{0.005, 0.05};
  CommandRecorder sink;
  follow::FollowRun run(robot, reference, config, true, &sink);
  const auto result = run.run();
  CHECK(result.status == follow::RunStatus::Success);
  CHECK(result.worst_tracking_error_rad == 0.02);
  REQUIRE_FALSE(sink.phases.empty());
  CHECK(sink.phases.front() == follow::Phase::Tracking);
  for (std::size_t k = 0; k < sink.phases.size(); ++k) {
    CAPTURE(k);
    CHECK(sink.phases[k] != follow::Phase::Home);
    CHECK(sink.phases[k] != follow::Phase::HomeCorrection);
    // The command is the reference itself: no offset of any size.
    CHECK(sink.commands[k] == sink.references[k]);
  }
  std::remove(path.c_str());
}

TEST_CASE("a posture run aborts and holds where the arm is once it departs "
          "from the reference",
          "[follow][run][posture][safety]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = matchedReferenceFile("build/follow_run_departure.zvs");
  model::ZvsTrajectory reference(path, map);
  DisplacedArm robot(3, 0.06);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.posture = follow::PostureRunConfig{0.005, 0.05};
  CommandRecorder sink;
  follow::FollowRun run(robot, reference, config, true, &sink);
  const auto result = run.run();
  CHECK(result.status == follow::RunStatus::PostureDeparture);
  CHECK(std::string(follow::statusName(result.status)) ==
        "posture_departure");
  CHECK(result.worst_tracking_error_rad == 0.06);
  REQUIRE_FALSE(sink.phases.empty());
  CHECK(sink.phases.back() == follow::Phase::AbortHold);
  bool held = false;
  for (std::size_t k = 0; k < sink.phases.size(); ++k) {
    CAPTURE(k);
    CHECK(sink.phases[k] != follow::Phase::Home);
    CHECK(sink.phases[k] != follow::Phase::HomeCorrection);
    if (sink.phases[k] == follow::Phase::Tracking) {
      CHECK(sink.commands[k] == sink.references[k]);
    } else {
      // The abort holds the measured, displaced posture.
      REQUIRE(sink.phases[k] == follow::Phase::AbortHold);
      CHECK(sink.commands[k][0] == 0.06);
      held = true;
    }
  }
  CHECK(held);
  std::remove(path.c_str());
}

TEST_CASE("a posture run commands no reference from feedback already beyond "
          "the abort bound",
          "[follow][run][posture][safety]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = matchedReferenceFile("build/follow_run_fresh_gate.zvs");
  model::ZvsTrajectory reference(path, map);
  // The fourth tracking read is the first displaced one; a sample at the
  // bound itself is beyond it.
  for (const double displacement : {0.06, 0.05}) {
    CAPTURE(displacement);
    DisplacedArm robot(3, displacement);
    REQUIRE(robot.activate());
    auto config = baseConfig(arm::ControlMode::Position);
    config.posture = follow::PostureRunConfig{0.005, 0.05};
    config.finalization.wait_time_s = 0.05;  // five hold cycles in all
    CommandRecorder sink;
    follow::FollowRun run(robot, reference, config, true, &sink);
    const auto result = run.run();
    CHECK(result.status == follow::RunStatus::PostureDeparture);
    CHECK(result.worst_tracking_error_rad == displacement);
    std::size_t first_hold = sink.phases.size();
    for (std::size_t k = 0; k < sink.phases.size(); ++k) {
      CAPTURE(k);
      if (sink.phases[k] == follow::Phase::Tracking) {
        // Every reference command answered feedback inside the bound.
        REQUIRE(first_hold == sink.phases.size());
        double error = 0.0;
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          error = std::max(error, std::fabs(sink.measured[k][i] -
                                            sink.references[k][i]));
        }
        CHECK(error < 0.05);
        CHECK(sink.commands[k] == sink.references[k]);
        continue;
      }
      REQUIRE(sink.phases[k] == follow::Phase::AbortHold);
      if (first_hold == sink.phases.size()) first_hold = k;
      // The hold is the posture of the sample that triggered it.
      CHECK(sink.commands[k] == sink.measured[first_hold]);
    }
    // The departed sample's own cycle is answered with the hold.
    REQUIRE(first_hold == 3);
    CHECK(sink.measured[3][0] == displacement);
    CHECK(sink.phases.size() == 3 + 5);
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
