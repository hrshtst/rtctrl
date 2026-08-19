// Characterization of the LaggedArm command-pipeline semantics — the
// extraction-equivalence pin for x7_track_sim's legacy behavior and
// the preregistered rules of the EFL study
// (docs/records/history.md (EFL frozen specification)).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "common/lagged_arm.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
using Catch::Matchers::WithinAbs;

namespace {

// Records the tau[0] of every command the wrapper actually applies.
struct RecorderArm : arm::Arm {
  int dof() const override { return model::kCanonicalDof; }
  double dt() const override { return 0.01; }
  bool activate() override { return true; }
  bool deactivate() override { return true; }
  bool setMode(arm::ControlMode) override { return true; }
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* cmds = nullptr) override {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      zVecElemNC(state.q.get(), i) = q_val;
      zVecElemNC(state.dq.get(), i) = dq_val;
      zVecElemNC(state.tau.get(), i) = 0.0;
    }
    state.t = time;
    state.seq = seq;
    if (cmds != nullptr) *cmds = arm::CommandSnapshot{};
    return true;
  }
  bool writeCommand(const arm::JointCommand& cmd,
                    arm::CommandReceipt* receipt = nullptr) override {
    applied.push_back(zVecElemNC(cmd.tau.get(), 0));
    if (receipt != nullptr) *receipt = {true, 0, time};
    return true;
  }
  bool step() override {
    time += dt();
    ++seq;
    return true;
  }
  std::vector<double> applied;
  double q_val = 0.0, dq_val = 0.0, time = 0.0;
  std::uint64_t seq = 0;
};

// One impulse sequence: command tau[0] = k at cycle k (1-based).
std::vector<double> appliedSchedule(x7::LaggedArm& lagged,
                                    RecorderArm& rec, int cycles) {
  arm::JointState state;
  arm::JointCommand cmd;
  cmd.mode = arm::ControlMode::Current;
  for (int k = 1; k <= cycles; ++k) {
    lagged.readState(state);
    zVecElemNC(cmd.tau.get(), 0) = static_cast<double>(k);
    lagged.writeCommand(cmd);
    lagged.step();
  }
  return rec.applied;
}

}  // namespace

TEST_CASE("legacy lag: first command passes through, then one cycle",
          "[lagged_arm]") {
  RecorderArm rec;
  x7::LaggedArm lagged(rec, 0.12);  // defaults = x7_track_sim semantics
  const auto applied = appliedSchedule(lagged, rec, 5);
  // cmd1 applies immediately AND as the first pending command; cmd k
  // (k >= 2) applies at cycle k+1. cmd5 is still pending at the end.
  REQUIRE(applied == std::vector<double>{1.0, 1.0, 2.0, 3.0, 4.0});
}

TEST_CASE("study lag: zero-preloaded queue applies cmd k at cycle k+2",
          "[lagged_arm]") {
  RecorderArm rec;
  x7::LaggedArmOptions opt;
  opt.delay_cycles = 2;
  opt.first_passthrough = false;
  x7::LaggedArm lagged(rec, 0.12, opt);
  const auto applied = appliedSchedule(lagged, rec, 6);
  // two preloaded zero-current commands first, then cmd k at k+2
  REQUIRE(applied == std::vector<double>{0.0, 0.0, 1.0, 2.0, 3.0, 4.0});
}

TEST_CASE("position and velocity quantization to the encoder LSBs",
          "[lagged_arm]") {
  RecorderArm rec;
  rec.q_val = 0.01;   // not a kPosLsb multiple
  rec.dq_val = 0.05;  // filtered then rounded to kVelLsb
  x7::LaggedArm lagged(rec, 0.12);
  arm::JointState state;
  lagged.readState(state);
  const double expect_q =
      std::round(0.01 / x7::LaggedArm::kPosLsb) * x7::LaggedArm::kPosLsb;
  REQUIRE_THAT(zVecElemNC(state.q.get(), 0), WithinAbs(expect_q, 1e-15));
  // first sample: filt = alpha*0.05 with alpha = 0.01/0.13, which
  // rounds to ZERO velocity LSBs — the lag visibly suppresses dq
  REQUIRE_THAT(zVecElemNC(state.dq.get(), 0), WithinAbs(0.0, 1e-15));
}

TEST_CASE("exact-velocity option passes dq through unfiltered",
          "[lagged_arm]") {
  RecorderArm rec;
  rec.q_val = 0.01;
  rec.dq_val = 0.05;
  x7::LaggedArmOptions opt;
  opt.exact_velocity = true;  // EFL-ideal's velocity source
  x7::LaggedArm lagged(rec, 0.12, opt);
  arm::JointState state;
  lagged.readState(state);
  REQUIRE_THAT(zVecElemNC(state.dq.get(), 0), WithinAbs(0.05, 1e-15));
  // position quantization is independent and stays on by default
  const double expect_q =
      std::round(0.01 / x7::LaggedArm::kPosLsb) * x7::LaggedArm::kPosLsb;
  REQUIRE_THAT(zVecElemNC(state.q.get(), 0), WithinAbs(expect_q, 1e-15));
}
