#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>

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
      if (mode_ == arm::ControlMode::Velocity) {
        state_.q[i] += command_.dq[i] * dt_;
        state_.dq[i] = command_.dq[i];
      } else {
        state_.q[i] = command_.q[i];
        state_.dq[i] = 0.0;
      }
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

TEST_CASE("shared follow run completes every phase in all servo modes",
          "[follow][run]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto path = referenceFile();
  model::ZvsTrajectory reference(path, map);
  for (const auto mode : {arm::ControlMode::Position,
                          arm::ControlMode::Velocity,
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
