#include "track/track_run.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
namespace track = x7::track;

namespace {

// Holds a fixed pose, accepts every command, and advances one control
// period and one feedback sequence per step; counts writes and steps.
class ClockArm : public arm::Arm {
 public:
  explicit ClockArm(int fail_reads_after = -1)
      : fail_reads_after_(fail_reads_after) {
    zVecElemNC(state_.q.get(), 0) = 0.1;
  }
  int dof() const override { return model::kCanonicalDof; }
  double dt() const override { return 0.01; }
  bool activate() override { return true; }
  bool deactivate() override { return true; }
  bool setMode(arm::ControlMode) override { return true; }
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* = nullptr) override {
    if (fail_reads_after_ >= 0 && reads_++ >= fail_reads_after_) return false;
    zVecCopyNC(state_.q.get(), state.q.get());
    zVecZero(state.dq.get());
    zVecZero(state.tau.get());
    state.t = state_.t;
    state.seq = state_.seq;
    return true;
  }
  bool writeCommand(const arm::JointCommand&,
                    arm::CommandReceipt* receipt = nullptr) override {
    ++writes;
    if (receipt != nullptr) {
      *receipt = {true, static_cast<std::uint64_t>(writes), state_.t};
    }
    return true;
  }
  bool step() override {
    ++steps;
    state_.t += 0.01;
    ++state_.seq;
    return true;
  }
  int writes = 0;
  int steps = 0;

 private:
  int fail_reads_after_;
  int reads_ = 0;
  arm::JointState state_;
};

struct PhaseRecorder : track::CycleSink {
  bool record(const track::CycleRecord& cycle) override {
    phases.push_back(cycle.phase);
    seqs.push_back(cycle.state.seq);
    return true;
  }
  std::vector<track::Phase> phases;
  std::vector<std::uint64_t> seqs;
};

class Still : public model::Trajectory {
 public:
  int size() const override { return model::kCanonicalDof; }
  double duration() const override { return 0.0; }
  void sample(double, zVec q, zVec dq, zVec ddq) const override {
    zVecZero(q);
    zVecElemNC(q, 0) = 0.1;
    if (dq != nullptr) zVecZero(dq);
    if (ddq != nullptr) zVecZero(ddq);
  }
};

std::string referenceFile() {
  const std::string path = "build/track_run_reference.zvs";
  std::ofstream out(path);
  out << "0.02 8 (0.1 0 0 0 0 0 0 0 )\n"
      << "0.02 8 (0.12 0 0 0 0 0 0 0 )\n"
      << "0.02 8 (0.14 0 0 0 0 0 0 0 )\n";
  return path;
}

track::Config shortConfig() {
  track::Config config;
  config.finalization.simulation_hold_time_s = 0.05;
  return config;
}

}  // namespace

TEST_CASE(
    "track tracking-start hook fires once before the first tracking "
    "command",
    "[track][run][hook]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  const auto config = shortConfig();
  ClockArm robot;
  PhaseRecorder sink;
  track::TrackingRun run(robot, chain, map, reference, config, true, &sink);
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
  REQUIRE(run.run().status == track::RunStatus::Success);
  CHECK(calls == 1);
  REQUIRE_FALSE(sink.phases.empty());
  CHECK(sink.phases.front() == track::Phase::Tracking);
  CHECK(records_at == 0);
  CHECK(sink.seqs.front() == hook_seq);
  CHECK(writes_at == 0);
  CHECK(steps_at == 0);
  std::remove(path.c_str());
}

TEST_CASE("track tracking-start hook waits for the completion check",
          "[track][run][hook]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const Still reference;  // complete before any tracking cycle
  const auto config = shortConfig();
  ClockArm robot;
  track::TrackingRun run(robot, chain, map, reference, config, true);
  int calls = 0;
  run.setTrackingStartHook([&](const arm::JointState&) { ++calls; });
  CHECK(run.run().status == track::RunStatus::Success);
  CHECK(calls == 0);
}

TEST_CASE(
    "track tracking-start hook does not fire when tracking never "
    "starts",
    "[track][run][hook]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  const auto config = shortConfig();
  ClockArm robot(1);  // the initial read succeeds, the first cycle's fails
  track::TrackingRun run(robot, chain, map, reference, config, true);
  int calls = 0;
  run.setTrackingStartHook([&](const arm::JointState&) { ++calls; });
  CHECK(run.run().status != track::RunStatus::Success);
  CHECK(calls == 0);
  CHECK(robot.steps == 0);
  std::remove(path.c_str());
}
