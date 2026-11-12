#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "follow/follow_preflight.hpp"
#include "follow/follow_run.hpp"
#include "rtctrl/hw/config.hpp"

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

// ------------------------------------------------- learned command offset

namespace {

namespace learn = rtctrl::learn;

using Joints = std::array<double, model::kCanonicalDof>;

const std::string kSha(64, 'a');  // any well-formed SHA-256 text

// An offset on the follower's tracking grid of `reference`, one that the
// frontends' preflight limits (the defaults) accept.
learn::CommandOffset learnedOffset(
    const model::ZvsTrajectory& reference, double dt,
    const std::function<double(std::size_t, int)>& value) {
  const auto grid = learn::trackingGrid(reference.duration(), dt);
  learn::CommandOffset offset;
  offset.method = "ilc";
  offset.trial = 1;
  offset.dt_s = grid.dt_s;
  offset.reference_duration_s = reference.duration();
  offset.reference_sha256 = kSha;
  offset.parent_sha256 = kSha;
  offset.source_log_sha256 = kSha;
  offset.delta_rad.resize(grid.samples());
  for (std::size_t i = 0; i < grid.samples(); ++i) {
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      offset.delta_rad[i][j] = value(i, j);
    }
  }
  learn::checkOffsetForRun(offset, reference.duration(), dt, kSha, {});
  return offset;
}

learn::CommandOffset constantOffset(const model::ZvsTrajectory& reference,
                                    double dt, const Joints& delta) {
  return learnedOffset(reference, dt,
                       [&delta](std::size_t, int j) { return delta[j]; });
}

struct Row {
  follow::Phase phase;
  double phase_time;
  Joints command, reference, reference_dq, reference_ddq, measured;
};

struct RowRecorder : follow::CycleSink {
  bool record(const follow::CycleRecord& cycle) override {
    Row row{cycle.phase, cycle.phase_time_s, {}, {}, {}, {}, {}};
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      row.command[i] = cycle.command->q[i];
      row.reference[i] = (*cycle.reference_q)[i];
      row.reference_dq[i] = (*cycle.reference_dq)[i];
      row.reference_ddq[i] = (*cycle.reference_ddq)[i];
      row.measured[i] = cycle.state->q[i];
    }
    rows.push_back(row);
    if (on_record) on_record(row);
    return true;
  }
  std::vector<Row> of(follow::Phase phase) const {
    std::vector<Row> selected;
    for (const auto& row : rows) {
      if (row.phase == phase) selected.push_back(row);
    }
    return selected;
  }
  // The phases in run order, one entry per contiguous block.
  std::vector<follow::Phase> blocks() const {
    std::vector<follow::Phase> sequence;
    for (const auto& row : rows) {
      if (sequence.empty() || sequence.back() != row.phase) {
        sequence.push_back(row.phase);
      }
    }
    return sequence;
  }
  std::function<void(const Row&)> on_record;
  std::vector<Row> rows;
};

// Reports the commanded posture less a constant per-joint sag (a servo
// with a steady-state error, which the home correction compensates), plus
// `displacement` on joint 0 from the given tracking cycle on (counted from
// startTracking(), wired to the run's tracking-start hook), plus `bump` on
// joint 1 whenever joint 1 was commanded strictly between low and high.
class SaggingArm : public IdealArm {
 public:
  explicit SaggingArm(const Joints& sag)
      : IdealArm(arm::ControlMode::Position), sag_(sag) {}
  void displaceFrom(long tracking_cycle, double displacement) {
    from_ = tracking_cycle;
    displacement_ = displacement;
  }
  void bumpBetween(double low, double high, double bump) {
    bump_low_ = low;
    bump_high_ = high;
    bump_ = bump;
  }
  void startTracking() { tracking_ = true; }
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* snapshot = nullptr) override {
    if (!IdealArm::readState(state, snapshot)) return false;
    const double commanded1 = state.q[1];
    for (int i = 0; i < model::kCanonicalDof; ++i) state.q[i] -= sag_[i];
    if (commanded1 > bump_low_ && commanded1 < bump_high_) {
      state.q[1] += bump_;
    }
    if (displacement_ != 0.0 && tracking_ && tracking_steps_ >= from_) {
      state.q[0] += displacement_;
    }
    return true;
  }
  bool step() override {
    if (tracking_) ++tracking_steps_;
    return IdealArm::step();
  }

 private:
  Joints sag_;
  long from_ = 0;
  double displacement_ = 0.0;
  double bump_low_ = 0.0, bump_high_ = 0.0, bump_ = 0.0;
  bool tracking_ = false;
  long tracking_steps_ = 0;
};

Joints joints(const model::ZVector& q) {
  Joints values{};
  for (int i = 0; i < model::kCanonicalDof; ++i) values[i] = q[i];
  return values;
}

}  // namespace

TEST_CASE("with an offset, tracking commands the reference plus the offset "
          "at every grid sample and the final hold keeps the last sample",
          "[follow][run][offset]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  IdealArm robot(arm::ControlMode::Position);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.finalization.wait_time_s = 0.03;  // three final-hold cycles
  const auto offset =
      learnedOffset(reference, robot.dt(), [](std::size_t i, int j) {
        return 0.001 * (j + 1) * std::sin(0.3 * static_cast<double>(i));
      });
  RowRecorder sink;
  follow::FollowRun run(robot, reference, config, true, &sink);
  run.setCommandOffset(&offset);
  const auto result = run.run();
  REQUIRE(result.status == follow::RunStatus::Success);

  // One grid for the follower and the offset.
  const auto grid = learn::trackingGrid(reference.duration(), robot.dt());
  const auto tracking = sink.of(follow::Phase::Tracking);
  REQUIRE(tracking.size() == grid.samples());
  REQUIRE(tracking.size() == offset.delta_rad.size());
  model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
      ddq(model::kCanonicalDof);
  for (std::size_t k = 0; k < tracking.size(); ++k) {
    CAPTURE(k);
    const double t =
        std::min(reference.duration(), static_cast<double>(k) * grid.dt_s);
    reference.sample(t, q.get(), dq.get(), ddq.get());
    CHECK(tracking[k].phase_time == t);
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      CAPTURE(j);
      // The logged reference is the reference; the command carries delta.
      CHECK(tracking[k].reference[j] == q[j]);
      CHECK(tracking[k].reference_dq[j] == dq[j]);
      CHECK(tracking[k].reference_ddq[j] == ddq[j]);
      CHECK(tracking[k].command[j] == q[j] + offset.delta_rad[k][j]);
    }
  }
  // The logged command is the one the arm received (IdealArm reports its
  // last command in the next cycle).
  for (std::size_t r = 0; r + 1 < sink.rows.size(); ++r) {
    CAPTURE(r);
    CHECK(sink.rows[r + 1].measured == sink.rows[r].command);
  }

  // The final hold logs q_d(T) at rest and commands q_d(T) + delta[N-1].
  const auto hold = sink.of(follow::Phase::Finalizing);
  REQUIRE(hold.size() == 3);
  reference.sample(reference.duration(), q.get());
  Joints held{};
  for (int j = 0; j < model::kCanonicalDof; ++j) {
    held[j] = q[j] + offset.delta_rad.back()[j];
  }
  CHECK(held != joints(q));  // the last sample is not zero
  for (const auto& row : hold) {
    CHECK(row.reference == joints(q));
    CHECK(row.reference_dq == Joints{});
    CHECK(row.reference_ddq == Joints{});
    CHECK(row.command == held);
  }
  std::remove(path.c_str());
}

TEST_CASE("an offset away from the reference trips no gate while the arm "
          "tracks the reference, and a real error trips the gates exactly "
          "as without an offset",
          "[follow][run][offset][safety]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = matchedReferenceFile("build/follow_run_offset_gates.zvs");
  model::ZvsTrajectory reference(path, map);  // 0 for 0.4 s
  // The sagging servo stays on the reference only with the offset that
  // compensates its sag: 0.1 rad on joint 0, beyond every gate below.
  const Joints sag{0.1, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0};
  auto config = baseConfig(arm::ControlMode::Position);
  config.safety.warning_error_rad = 0.05;
  config.safety.sustained_abort_error_rad = 0.06;
  config.safety.sustained_abort_time_s = 0.02;
  config.safety.immediate_abort_error_rad = 0.08;

  struct Outcome {
    follow::RunResult result;
    RowRecorder sink;
  };
  const auto runOnce = [&](bool with_offset, double displacement) {
    auto outcome = std::make_unique<Outcome>();
    SaggingArm robot(with_offset ? sag : Joints{});
    REQUIRE(robot.activate());
    robot.displaceFrom(3, displacement);
    const auto offset = constantOffset(reference, robot.dt(), sag);
    follow::FollowRun run(robot, reference, config, true, &outcome->sink);
    run.setTrackingStartHook(
        [&robot](const arm::JointState&) { robot.startTracking(); });
    if (with_offset) run.setCommandOffset(&offset);
    outcome->result = run.run();
    return outcome;
  };

  for (const double displacement : {0.0, 0.055, 0.07, 0.09}) {
    CAPTURE(displacement);
    const auto plain = runOnce(false, displacement);
    const auto learned = runOnce(true, displacement);
    const auto& a = plain->result;
    const auto& b = learned->result;
    CHECK(b.status == a.status);
    CHECK(b.tracking_warning == a.tracking_warning);
    CHECK(b.worst_tracking_error_rad == a.worst_tracking_error_rad);
    // worst_joint is shared with homing (only the offset run's corrected
    // one): it is the tracking error's joint once tracking has an error.
    if (displacement != 0.0) {
      CHECK(a.worst_joint == 0);
      CHECK(b.worst_joint == 0);
    }
    if (displacement == 0.0) {
      CHECK(b.status == follow::RunStatus::Success);
      CHECK_FALSE(b.tracking_warning);
      CHECK(b.worst_tracking_error_rad == 0.0);
    } else if (displacement < 0.06) {
      CHECK(b.status == follow::RunStatus::Success);
      CHECK(b.tracking_warning);
      CHECK(b.worst_tracking_error_rad == displacement);
    } else {
      CHECK(b.status == follow::RunStatus::TrackingError);
      CHECK(b.worst_tracking_error_rad == displacement);
    }
    // The offset run homed with a correction, so it compensated the sag.
    const auto corrected = learned->sink.of(follow::Phase::HomeCorrection);
    REQUIRE_FALSE(corrected.empty());
    CHECK(corrected.back().command[0] == 0.1);

    // Same tracking cycles, same measured motion; only the command moved.
    const auto ta = plain->sink.of(follow::Phase::Tracking);
    const auto tb = learned->sink.of(follow::Phase::Tracking);
    REQUIRE(tb.size() == ta.size());
    for (std::size_t k = 0; k < tb.size(); ++k) {
      CAPTURE(k);
      CHECK(tb[k].measured == ta[k].measured);
      CHECK(tb[k].reference == ta[k].reference);
      for (int j = 0; j < model::kCanonicalDof; ++j) {
        CHECK(tb[k].command[j] == ta[k].command[j] + sag[j]);
      }
    }
    // An abort holds the measured posture: no offset.
    const auto ha = plain->sink.of(follow::Phase::AbortHold);
    const auto hb = learned->sink.of(follow::Phase::AbortHold);
    REQUIRE(hb.size() == ha.size());
    CHECK(hb.empty() == (b.status == follow::RunStatus::Success));
    for (std::size_t k = 0; k < hb.size(); ++k) {
      CAPTURE(k);
      CHECK(hb[k].command == ha[k].command);
      CHECK(hb[k].command == tb.back().measured);
      CHECK(hb[k].command[0] == displacement);
    }
  }
  std::remove(path.c_str());
}

TEST_CASE("the offset ramp moves the command from the corrected home goal "
          "to the first offset command at the rate limit, then settles",
          "[follow][run][offset]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();  // q_d(0) = 0.1 on joint 0
  model::ZvsTrajectory reference(path, map);
  // A 0.1 rad sag on joint 0: homing ends with a home correction, its last
  // goal 0.2 rad. Joint 1 is bumped 2 rad (beyond every gate) whenever it
  // is commanded strictly inside the ramp, which only the ramp does.
  SaggingArm robot(Joints{0.1, 0, 0, 0, 0, 0, 0, 0});
  robot.bumpBetween(0.0, 0.05, 2.0);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.home.settle_time_s = 0.02;              // two cycles
  config.learning.max_offset_rate_rad_s = 0.4;   // lowered: it governs
  const Joints delta{-0.1, 0.05, 0, 0, -0.03, 0, 0, 0};
  const auto offset = constantOffset(reference, robot.dt(), delta);
  RowRecorder sink;
  follow::FollowRun run(robot, reference, config, true, &sink);
  run.setCommandOffset(&offset);
  const auto result = run.run();
  CHECK(result.status == follow::RunStatus::Success);
  // The ramp's bumped feedback reached no gate.
  CHECK(result.worst_tracking_error_rad < 0.5);
  CHECK(sink.blocks() ==
        std::vector<follow::Phase>{follow::Phase::Home,
                                   follow::Phase::HomeCorrection,
                                   follow::Phase::OffsetRamp,
                                   follow::Phase::Tracking,
                                   follow::Phase::Finalizing});

  model::ZVector q0(model::kCanonicalDof);
  reference.sample(0.0, q0.get());
  const auto corrected = sink.of(follow::Phase::HomeCorrection);
  REQUIRE_FALSE(corrected.empty());
  const Joints goal = corrected.back().command;  // the last homing command
  // A real correction, and a measured posture away from that goal: the
  // ramp starts from what was commanded.
  CHECK(std::fabs(goal[0] - q0[0] - 0.1) < 1e-12);
  CHECK(corrected.back().measured != goal);
  Joints target{};
  double widest = 0.0;
  for (int j = 0; j < model::kCanonicalDof; ++j) {
    target[j] = q0[j] + delta[j];
    widest = std::max(widest, std::fabs(target[j] - goal[j]));
  }
  CHECK(widest == std::fabs(target[0] - goal[0]));  // 0.2 rad on joint 0
  const double dt = robot.dt();
  const double rate = config.learning.max_offset_rate_rad_s;
  const auto steps = static_cast<std::size_t>(std::ceil(widest / rate / dt));
  CHECK(steps >= 50);  // 0.2 rad at 0.4 rad/s, not at the 0.5 default
  const auto ramp = sink.of(follow::Phase::OffsetRamp);
  REQUIRE(ramp.size() == steps + 2);  // then the two settle cycles

  Joints previous = goal;
  bool bumped = false;
  for (std::size_t k = 0; k < ramp.size(); ++k) {
    CAPTURE(k);
    const auto& row = ramp[k];
    CHECK(row.phase_time == static_cast<double>(k) * dt);
    // The logged reference is q_d(0) at rest.
    CHECK(row.reference == joints(q0));
    CHECK(row.reference_dq == Joints{});
    CHECK(row.reference_ddq == Joints{});
    if (k + 1 >= steps) {
      CHECK(row.command == target);  // the last step lands on it; settle
    } else {
      const double fraction =
          static_cast<double>(k + 1) / static_cast<double>(steps);
      for (int j = 0; j < model::kCanonicalDof; ++j) {
        CAPTURE(j);
        CHECK(std::fabs(row.command[j] -
                        (goal[j] + (target[j] - goal[j]) * fraction)) < 1e-12);
      }
    }
    for (int j = 0; j < model::kCanonicalDof; ++j) {
      CAPTURE(j);
      CHECK(std::fabs(row.command[j] - previous[j]) <= rate * dt + 1e-12);
    }
    bumped = bumped || std::fabs(row.measured[1] - row.reference[1]) > 1.0;
    previous = row.command;
  }
  CHECK(bumped);  // the gates would have seen it
  // Tracking starts where the ramp settled.
  const auto tracking = sink.of(follow::Phase::Tracking);
  REQUIRE_FALSE(tracking.empty());
  CHECK(tracking.front().command == target);
  std::remove(path.c_str());
}

TEST_CASE("an all-zero offset still runs the ramp phase, of settle cycles "
          "only",
          "[follow][run][offset]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  IdealArm robot(arm::ControlMode::Position);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.home.settle_time_s = 0.03;
  const auto zero =
      learn::zeroOffset(reference.duration(), robot.dt(), kSha);
  RowRecorder sink;
  follow::FollowRun run(robot, reference, config, true, &sink);
  run.setCommandOffset(&zero);
  REQUIRE(run.run().status == follow::RunStatus::Success);
  CHECK(sink.blocks() ==
        std::vector<follow::Phase>{follow::Phase::Home,
                                   follow::Phase::HomeCorrection,
                                   follow::Phase::OffsetRamp,
                                   follow::Phase::Tracking,
                                   follow::Phase::Finalizing});
  model::ZVector q0(model::kCanonicalDof);
  reference.sample(0.0, q0.get());
  const auto ramp = sink.of(follow::Phase::OffsetRamp);
  // The home goal is the first command already: zero ramp steps, then the
  // settle hold.
  REQUIRE(ramp.size() ==
          static_cast<std::size_t>(std::ceil(config.home.settle_time_s /
                                             robot.dt())));
  for (const auto& row : ramp) {
    CHECK(row.command == joints(q0));
    CHECK(row.reference == joints(q0));
  }
  for (const auto& row : sink.of(follow::Phase::Tracking)) {
    CHECK(row.command == row.reference);
  }
  std::remove(path.c_str());
}

TEST_CASE("without an offset the run has no ramp and commands the "
          "reference, as before",
          "[follow][run][offset]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  const auto config = baseConfig(arm::ControlMode::Position);
  const auto runOnce = [&](bool clear) {
    IdealArm robot(arm::ControlMode::Position);
    REQUIRE(robot.activate());
    auto sink = std::make_unique<RowRecorder>();
    follow::FollowRun run(robot, reference, config, true, sink.get());
    if (clear) run.setCommandOffset(nullptr);
    REQUIRE(run.run().status == follow::RunStatus::Success);
    return sink;
  };
  const auto plain = runOnce(false);
  CHECK(plain->blocks() ==
        std::vector<follow::Phase>{follow::Phase::Home,
                                   follow::Phase::HomeCorrection,
                                   follow::Phase::Tracking,
                                   follow::Phase::Finalizing});
  const auto tracking = plain->of(follow::Phase::Tracking);
  CHECK(tracking.size() ==
        learn::trackingGrid(reference.duration(), 0.01).samples());
  for (const auto& row : tracking) CHECK(row.command == row.reference);
  for (const auto& row : plain->of(follow::Phase::Finalizing)) {
    CHECK(row.command == row.reference);
  }
  // A null offset is no offset: the same cycles, bit for bit.
  const auto cleared = runOnce(true);
  REQUIRE(cleared->rows.size() == plain->rows.size());
  for (std::size_t r = 0; r < plain->rows.size(); ++r) {
    CAPTURE(r);
    CHECK(cleared->rows[r].phase == plain->rows[r].phase);
    CHECK(cleared->rows[r].phase_time == plain->rows[r].phase_time);
    CHECK(cleared->rows[r].command == plain->rows[r].command);
    CHECK(cleared->rows[r].reference == plain->rows[r].reference);
  }
  std::remove(path.c_str());
}

TEST_CASE("a follow run refuses an offset off its grid or with a posture "
          "reference",
          "[follow][run][offset][posture]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  CountingArm robot(arm::ControlMode::Position);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  follow::FollowRun run(robot, reference, config, true);
  // Built for a 200 Hz period: more samples than this 100 Hz grid.
  const auto fast = learn::zeroOffset(reference.duration(), 0.005, kSha);
  CHECK_THROWS_WITH(run.setCommandOffset(&fast),
                    Catch::Matchers::ContainsSubstring("tracking grid"));
  config.posture = follow::PostureRunConfig{0.005, 0.05};
  const auto zero = learn::zeroOffset(reference.duration(), 0.01, kSha);
  CHECK_THROWS_WITH(run.setCommandOffset(&zero),
                    Catch::Matchers::ContainsSubstring("measured posture"));
  CHECK(robot.writes == 0);
  std::remove(path.c_str());
}

TEST_CASE("an I/O failure in the offset ramp ends the run as in homing",
          "[follow][run][offset]") {
  class FlakyArm : public IdealArm {
   public:
    FlakyArm() : IdealArm(arm::ControlMode::Position) {}
    bool writeCommand(const arm::JointCommand& command,
                      arm::CommandReceipt* receipt = nullptr) override {
      if (fail_next) {
        fail_next = false;
        return false;
      }
      return IdealArm::writeCommand(command, receipt);
    }
    bool fail_next = false;
  } robot;
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  REQUIRE(robot.activate());
  auto config = baseConfig(arm::ControlMode::Position);
  config.home.settle_time_s = 0.03;  // a ramp of three cycles
  const auto offset =
      constantOffset(reference, robot.dt(), Joints{0.002, 0, 0, 0, 0, 0, 0, 0});
  RowRecorder sink;
  // The ramp's second write fails once.
  sink.on_record = [&](const Row& row) {
    if (row.phase == follow::Phase::OffsetRamp) robot.fail_next = true;
  };
  follow::FollowRun run(robot, reference, config, true, &sink);
  int hook_calls = 0;
  run.setTrackingStartHook([&](const arm::JointState&) { ++hook_calls; });
  run.setCommandOffset(&offset);
  const auto result = run.run();
  CHECK(result.status == follow::RunStatus::IoFailure);
  CHECK(hook_calls == 0);
  CHECK(sink.of(follow::Phase::Tracking).empty());
  CHECK(sink.of(follow::Phase::OffsetRamp).size() == 1);
  // As after a homing failure, the abort hold holds the last measured
  // posture (this arm's: the ramp's one command).
  const auto hold = sink.of(follow::Phase::AbortHold);
  REQUIRE_FALSE(hold.empty());
  for (const auto& row : hold) {
    CHECK(row.command == sink.of(follow::Phase::OffsetRamp).back().command);
  }
  CHECK(result.cycles == sink.rows.size());
  std::remove(path.c_str());
}

namespace {

bool sameBits(double a, double b) {
  std::uint64_t x = 0;
  std::uint64_t y = 0;
  std::memcpy(&x, &a, sizeof x);
  std::memcpy(&y, &b, sizeof y);
  return x == y;
}

// `frames` frames of `frame_s` each, joint 0 rising by 1 mrad per frame.
std::string rampReference(const std::string& name, double frame_s,
                          int frames) {
  const std::string path = "build/" + name;
  std::FILE* out = std::fopen(path.c_str(), "w");
  REQUIRE(out != nullptr);
  for (int k = 0; k < frames; ++k) {
    std::fprintf(out, "%.17g 8 ( %.17g 0 0 0 0 0 0 0 )\n", frame_s,
                 0.1 + 0.001 * k);
  }
  REQUIRE(std::fclose(out) == 0);
  return path;
}

}  // namespace

TEST_CASE("the preflight validates the tracking grid the follower commands",
          "[follow][run][preflight]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto hardware = rtctrl::hw::Config::load("config/crane_x7.toml");
  struct Case {
    double frame_s;
    int frames;
    double rate_hz;
  };
  // Durations summed from frames on which the preflight's former grid,
  // max(1, ceil(duration * rate)) intervals, was not the follower's: one
  // interval more (0.01 s x 6 is 0.060000000000000005 s: 7 intervals at
  // 100 Hz, the follower's 6) or one fewer (0.1 s at 70 Hz: 7, the
  // follower's 8).
  for (const Case c : {Case{0.01, 6, 100.0}, Case{0.1, 1, 70.0},
                       Case{0.07, 5, 20.0}, Case{0.02, 7, 150.0},
                       Case{0.03, 181, 200.0}}) {
    INFO(c.frames << " frames of " << c.frame_s << " s at " << c.rate_hz
                  << " Hz");
    const auto path = rampReference("follow_run_grid.zvs", c.frame_s, c.frames);
    model::ZvsTrajectory reference(path, map);
    auto config = baseConfig(arm::ControlMode::Position);
    config.control_rate_hz = c.rate_hz;
    const auto grid =
        follow::preflightTrackingGrid(reference.duration(), config);
    CHECK(std::max(1L, static_cast<long>(std::ceil(reference.duration() *
                                                   c.rate_hz))) !=
          grid.intervals);
    CHECK_NOTHROW(
        follow::validateReference(config, chain, map, reference, hardware));

    // Every frontend's arm has dt() = 1 / control.rate_hz.
    IdealArm robot(arm::ControlMode::Position, 1.0 / c.rate_hz);
    REQUIRE(robot.activate());
    RowRecorder sink;
    follow::FollowRun run(robot, reference, config, true, &sink);
    REQUIRE(run.run().status == follow::RunStatus::Success);
    const auto tracking = sink.of(follow::Phase::Tracking);
    REQUIRE(tracking.size() == grid.samples());
    for (std::size_t i = 0; i < tracking.size(); ++i) {
      const double t =
          follow::trackingSampleTime(grid, reference.duration(), i);
      if (!sameBits(tracking[i].phase_time, t)) {
        FAIL_CHECK("tracking sample " << i << " at " << tracking[i].phase_time
                                      << " s, the preflight's at " << t
                                      << " s");
      }
    }
    std::remove(path.c_str());
  }
}

TEST_CASE("the preflight refuses a reference the follower could not track",
          "[follow][run][preflight]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto hardware = rtctrl::hw::Config::load("config/crane_x7.toml");
  auto config = baseConfig(arm::ControlMode::Position);
  // The ZVS reader takes any finite positive frame time: two 1e17 s frames
  // are 2e19 control periods at 100 Hz, more than a long counts. The
  // follower would build this grid only after homing, with torque on.
  const auto path = rampReference("follow_run_huge.zvs", 1e17, 2);
  model::ZvsTrajectory reference(path, map);
  REQUIRE(std::isfinite(reference.duration()));
  CHECK_THROWS(rtctrl::learn::trackingGrid(reference.duration(), 0.01));
  CHECK_THROWS_WITH(
      follow::validateReference(config, chain, map, reference, hardware),
      "follow preflight: the reference cannot be tracked at control.rate_hz "
      "= 100: tracking grid: reference duration 2e+17 s over control period "
      "0.01 s is too many intervals");
  std::remove(path.c_str());
  // Durations no grid exists for.
  for (const double duration : {0.0, -1.0, std::nan(""), HUGE_VAL}) {
    CHECK_THROWS_WITH(
        follow::preflightTrackingGrid(duration, config),
        Catch::Matchers::StartsWith("follow preflight: the reference cannot "
                                    "be tracked at control.rate_hz = 100: "
                                    "tracking grid: reference duration") &&
            Catch::Matchers::EndsWith("s must be finite and > 0"));
  }
}
