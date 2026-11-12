// Characterization of the LaggedArm command-pipeline semantics — the
// extraction-equivalence pin for x7_track_sim's legacy behavior and
// the preregistered rules of the EFL study
// (docs/records/history.md (EFL frozen specification)).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "common/lagged_arm.hpp"
#include "rtctrl/arm/sim_arm.hpp"

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

namespace {

// Records q[0] of every command written to it, and the cycle it came in.
struct PositionRecorderArm : RecorderArm {
  bool writeCommand(const arm::JointCommand& cmd,
                    arm::CommandReceipt* receipt = nullptr) override {
    written_q.push_back(zVecElemNC(cmd.q.get(), 0));
    written_cycle.push_back(seq + 1);  // cycles count from 1
    return RecorderArm::writeCommand(cmd, receipt);
  }
  std::vector<double> written_q;
  std::vector<std::uint64_t> written_cycle;
};

x7::LaggedArmOptions holdOptions(int delay_cycles) {
  x7::LaggedArmOptions opt;
  opt.delay_cycles = delay_cycles;
  opt.first_passthrough = false;
  opt.hold_preload = true;
  return opt;
}

// Position command q[0] = k at cycle k (1-based); returns the receipts.
std::vector<arm::CommandReceipt> positionSchedule(x7::LaggedArm& lagged,
                                                  int cycles) {
  std::vector<arm::CommandReceipt> receipts;
  arm::JointState state;
  arm::JointCommand cmd;
  cmd.mode = arm::ControlMode::Position;
  for (int k = 1; k <= cycles; ++k) {
    lagged.readState(state);
    zVecElemNC(cmd.q.get(), 0) = static_cast<double>(k);
    arm::CommandReceipt receipt;
    REQUIRE(lagged.writeCommand(cmd, &receipt));
    receipts.push_back(receipt);
    lagged.step();
  }
  return receipts;
}

rtctrl::arm::SimArm::Options simOptions() {
  rtctrl::arm::SimArm::Options opt;
  opt.model_path = "models/crane_x7/crane_x7.ztk";
  opt.initial_q8 = {0.0, 0.5, 0.0, -1.0, 0.0, -0.4, 0.0, 0.2};
  return opt;
}

void requireSameArmState(arm::Arm& a, arm::Arm& b) {
  arm::JointState sa, sb;
  REQUIRE(a.readState(sa));
  REQUIRE(b.readState(sb));
  CHECK(sa.t == sb.t);
  CHECK(sa.seq == sb.seq);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    CHECK(sa.q[i] == sb.q[i]);
    CHECK(sa.dq[i] == sb.dq[i]);
    CHECK(sa.tau[i] == sb.tau[i]);
  }
}

}  // namespace

TEST_CASE("hold lag: position command k applies at cycle k+2, none before",
          "[lagged_arm]") {
  PositionRecorderArm rec;
  x7::LaggedArm lagged(rec, 0.12, holdOptions(2));
  const auto receipts = positionSchedule(lagged, 6);
  // cycles 1 and 2 meet the hold slots and write nothing; cmd k follows
  // at cycle k+2; cmd5 and cmd6 are still in the queue at the end
  REQUIRE(rec.written_q == std::vector<double>{1.0, 2.0, 3.0, 4.0});
  REQUIRE(rec.written_cycle == std::vector<std::uint64_t>{3, 4, 5, 6});
  // every command is accepted as its own submission, at its read time
  for (std::size_t k = 0; k < receipts.size(); ++k) {
    CHECK(receipts[k].accepted);
    CHECK(receipts[k].submitted_seq == k + 1);
    CHECK(receipts[k].submission_time == 0.01 * static_cast<double>(k));
  }
}

TEST_CASE("hold lag without delay passes every command through",
          "[lagged_arm]") {
  PositionRecorderArm rec;
  x7::LaggedArm lagged(rec, 0.12, holdOptions(0));
  positionSchedule(lagged, 4);
  REQUIRE(rec.written_q == std::vector<double>{1.0, 2.0, 3.0, 4.0});
  REQUIRE(rec.written_cycle == std::vector<std::uint64_t>{1, 2, 3, 4});
}

TEST_CASE("hold lag refuses the legacy startup and a negative delay",
          "[lagged_arm]") {
  RecorderArm rec;
  auto legacy = holdOptions(1);
  legacy.first_passthrough = true;
  CHECK_THROWS_AS(x7::LaggedArm(rec, 0.12, legacy), std::invalid_argument);
  CHECK_THROWS_AS(x7::LaggedArm(rec, 0.12, holdOptions(-1)),
                  std::invalid_argument);
}

TEST_CASE("hold lag keeps a SimArm at its activation posture until cmd 1",
          "[lagged_arm][sim]") {
  constexpr int kDelay = 3;
  for (const auto mode : {arm::ControlMode::Position,
                          arm::ControlMode::CurrentBasedPosition}) {
    rtctrl::arm::SimArm sim(simOptions());
    rtctrl::arm::SimArm idle(simOptions());  // activated, never commanded
    for (auto* plant : {&sim, &idle}) {
      REQUIRE(plant->setMode(mode));
      REQUIRE(plant->activate());
    }
    auto opt = holdOptions(kDelay);
    opt.quantize_pos = false;
    opt.exact_velocity = true;
    x7::LaggedArm lagged(sim, 0.12, opt);

    arm::JointState start;
    REQUIRE(sim.readState(start));
    arm::JointCommand cmd;
    cmd.mode = mode;
    for (int k = 1; k <= kDelay + 4; ++k) {
      INFO("cycle " << k);
      arm::JointState state;
      arm::CommandSnapshot snap;
      REQUIRE(lagged.readState(state, &snap));
      if (k <= kDelay + 1) {
        // nothing applied yet: the hold slots wrote nothing, and the arm
        // is exactly where an activated, uncommanded arm would be
        CHECK_FALSE(snap.applied.valid);
        CHECK_FALSE(snap.last_attempt.valid);
        requireSameArmState(sim, idle);
      } else {
        // the record of cmd (k - kDelay - 1), applied during the last
        // step: the inner arm's values under the wrapper's sequence
        const auto seq = static_cast<std::uint64_t>(k - kDelay - 1);
        REQUIRE(snap.applied.valid);
        CHECK(snap.applied.target_seq == seq);
        CHECK(snap.applied.mode == static_cast<std::uint8_t>(mode));
        CHECK(snap.applied.first_cycle == seq + kDelay - 1);
        CHECK(snap.last_attempt.valid);
        CHECK(snap.last_attempt.target_seq == seq);
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          CHECK(snap.applied.applied[i] ==
                start.q[i] + 0.01 * static_cast<double>(seq));
        }
        if (mode == arm::ControlMode::CurrentBasedPosition) {
          // 20 Nm requested over the 4 to 10 Nm ceilings: clamp truth kept
          CHECK(snap.applied.effort_limit[0] == 10.0);
          CHECK(snap.applied.flags[0] == arm::kCmdClamped);
        }
      }
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        cmd.q[i] = start.q[i] + 0.01 * k;
        cmd.effort_limit[i] = 20.0;
      }
      arm::CommandReceipt receipt;
      REQUIRE(lagged.writeCommand(cmd, &receipt));
      CHECK(receipt.accepted);
      CHECK(receipt.submitted_seq == static_cast<std::uint64_t>(k));
      REQUIRE(lagged.step());
      REQUIRE(idle.step());
    }
    // cmd 1 reached the servo at cycle kDelay + 1: the arm has moved on
    arm::JointState moved, held;
    REQUIRE(sim.readState(moved));
    REQUIRE(idle.readState(held));
    CHECK(moved.q[0] > held.q[0] + 1e-3);
  }
}

TEST_CASE("hold lag reads exact or quantized feedback as configured",
          "[lagged_arm][sim]") {
  rtctrl::arm::SimArm sim(simOptions());
  REQUIRE(sim.setMode(arm::ControlMode::Position));
  REQUIRE(sim.activate());
  auto exact_opt = holdOptions(1);
  exact_opt.quantize_pos = false;
  exact_opt.exact_velocity = true;
  x7::LaggedArm exact(sim, 0.12, exact_opt);
  auto quantized_opt = holdOptions(1);  // quantize_pos, filtered velocity
  x7::LaggedArm quantized(sim, 0.12, quantized_opt);

  arm::JointState start;
  REQUIRE(sim.readState(start));
  arm::JointCommand cmd;
  cmd.mode = arm::ControlMode::Position;
  for (int i = 0; i < model::kCanonicalDof; ++i) cmd.q[i] = start.q[i] + 0.2;
  bool moving = false;
  for (int k = 1; k <= 30; ++k) {
    INFO("cycle " << k);
    arm::JointState truth, a, b;
    REQUIRE(sim.readState(truth));
    REQUIRE(exact.readState(a));
    REQUIRE(quantized.readState(b));
    CHECK(a.t == truth.t);
    CHECK(a.seq == truth.seq);
    CHECK(b.t == truth.t);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      // exact: the inner arm's feedback, bit for bit
      CHECK(a.q[i] == truth.q[i]);
      CHECK(a.dq[i] == truth.dq[i]);
      CHECK(a.tau[i] == truth.tau[i]);
      CHECK(b.tau[i] == truth.tau[i]);
      // quantized: whole encoder and velocity LSBs (a value that rounding
      // to its LSB leaves unchanged), the position within half an LSB of
      // the truth
      const auto lsbs = [](double value, double lsb) {
        return std::round(value / lsb) * lsb;
      };
      CHECK(b.q[i] == lsbs(b.q[i], x7::LaggedArm::kPosLsb));
      CHECK(std::fabs(b.q[i] - truth.q[i]) <=
            0.5 * x7::LaggedArm::kPosLsb + 1e-12);
      CHECK(b.dq[i] == lsbs(b.dq[i], x7::LaggedArm::kVelLsb));
    }
    if (std::fabs(truth.dq[1]) > 0.1) moving = true;
    exact.writeCommand(cmd);  // only the exact wrapper drives the arm
    REQUIRE(sim.step());
  }
  CHECK(moving);  // the readout was exercised on a moving arm
}
