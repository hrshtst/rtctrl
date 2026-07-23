// Application-orchestration regressions for apps/track_common.hpp:
// the settle quiescence gate and TrackingRun's sequence-keyed latency
// verification. These exist because the settle gate once reported
// quiescent on a timeout whose FINAL sample merely dipped below the
// threshold — a defect no library-level test could see.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <functional>

#include "track_common.hpp"

using Catch::Approx;
namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
using model::kCanonicalDof;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

// An Arm whose joint-1 position follows a caller-supplied q(t); every
// step advances 10 ms and one feedback sequence.
class PoseScriptArm : public arm::Arm {
 public:
  explicit PoseScriptArm(std::function<double(double)> q1)
      : q1_(std::move(q1)) {}

  int dof() const override { return kCanonicalDof; }
  double dt() const override { return 0.01; }
  bool activate() override { return true; }
  bool deactivate() override { return true; }
  bool setMode(arm::ControlMode) override { return true; }
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* cmds = nullptr) override {
    for (int i = 0; i < kCanonicalDof; ++i) {
      zVecElemNC(state.q.get(), i) = 0.0;
      zVecElemNC(state.dq.get(), i) = 0.0;
    }
    zVecElemNC(state.q.get(), 1) = q1_(time_);
    state.t = 100.0 + time_;
    state.seq = seq_;
    if (cmds != nullptr) *cmds = arm::CommandSnapshot{};
    return true;
  }
  bool writeCommand(const arm::JointCommand&,
                    arm::CommandReceipt* receipt = nullptr) override {
    if (receipt != nullptr) *receipt = {true, ++writes_, 100.0 + time_};
    return true;
  }
  bool step() override {
    time_ += 0.01;
    ++seq_;
    return true;
  }

 private:
  std::function<double(double)> q1_;
  double time_ = 0.0;
  std::uint64_t seq_ = 1;
  std::uint64_t writes_ = 0;
};

}  // namespace

TEST_CASE("settle gate: sustained stillness is accepted",
          "[track_common]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  x7::SettleController settle(chain, map, x7::tuning::kSettleKd);
  PoseScriptArm robot([](double) { return 0.1; });  // perfectly still
  const auto res = x7::settleArm(robot, settle, 6.0);
  CHECK(res.io_ok);
  CHECK(res.quiescent);
  CHECK(res.elapsed < 2.0);  // exits at the quiet window, not timeout
}

TEST_CASE("settle gate: a final low sample without the sustained window "
          "is REJECTED",
          "[track_common]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  x7::SettleController settle(chain, map, x7::tuning::kSettleKd);
  // moving at 0.5 rad/s until t = 1.45 s, then frozen: by the 2.0 s
  // timeout the slow-filtered residual has decayed below the 0.05
  // threshold (the metric crosses ~0.4 s after stillness begins), but
  // the sustained 0.3 s quiet window was NOT yet reached — the exact
  // near-miss that defeated the pre-fix gate (quiescent = last-sample
  // residual < 0.05).
  PoseScriptArm robot([](double t) { return 0.5 * std::min(t, 1.45); });
  const auto res = x7::settleArm(robot, settle, 2.0);
  CHECK(res.io_ok);
  INFO("residual " << res.residual);
  CHECK(res.residual < 0.05);  // the trap precondition really occurred
  CHECK_FALSE(res.quiescent);  // the pre-fix gate said true here
}

TEST_CASE("settle gate: timeout while moving is rejected",
          "[track_common]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  x7::SettleController settle(chain, map, x7::tuning::kSettleKd);
  PoseScriptArm robot([](double t) { return 0.5 * t; });  // never still
  const auto res = x7::settleArm(robot, settle, 1.5);
  CHECK(res.io_ok);
  CHECK_FALSE(res.quiescent);
  CHECK(res.residual > 0.05);
}

TEST_CASE("TrackingRun latency verification: baseline, match, deadline, "
          "skip",
          "[track_common]") {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  model::ZVector zero(kCanonicalDof);
  model::MinJerkTrajectory hold(zero.get(), zero.get(), 1.0);

  arm::JointState state;
  arm::JointCommand cmd;
  auto snapshotFor = [](bool valid, std::uint64_t seq, double first_time) {
    arm::CommandSnapshot s;
    s.applied.valid = valid;
    s.applied.target_seq = seq;
    s.applied.first_time = first_time;
    s.applied.latest_time = first_time;
    return s;
  };

  SECTION("pre-run baseline is not a violation; matched receipts pass") {
    x7::TrackingRun run(chain, map, hold, 6.0, 1.0, 0.5, nullptr);
    // snapshot already carries a settle-phase command unknown to the
    // receipt map: baseline, not a skipped submission
    state.t = 100.00;
    CHECK(run.observe(0.0, state, snapshotFor(true, 42, 99.99), cmd,
                      {true, 100, 100.00}));
    // next cycle: our submission (seq 100) applied 10 ms later — fine
    state.t = 100.01;
    CHECK(run.observe(0.01, state, snapshotFor(true, 100, 100.01), cmd,
                      {true, 101, 100.01}));
  }

  SECTION("a submission never applied within the deadline aborts") {
    x7::TrackingRun run(chain, map, hold, 6.0, 1.0, 0.5, nullptr);
    state.t = 100.00;
    CHECK(run.observe(0.0, state, snapshotFor(true, 42, 99.99), cmd,
                      {true, 100, 100.00}));
    // the applied record never advances past the baseline while our
    // submission ages beyond 2 nominal cycles
    state.t = 100.01;
    CHECK(run.observe(0.01, state, snapshotFor(true, 42, 99.99), cmd,
                      {true, 101, 100.01}));
    state.t = 100.03;
    CHECK_FALSE(run.observe(0.03, state, snapshotFor(true, 42, 99.99),
                            cmd, {true, 102, 100.03}));
  }

  SECTION("an applied sequence that skips a pending one aborts") {
    x7::TrackingRun run(chain, map, hold, 6.0, 1.0, 0.5, nullptr);
    state.t = 100.00;
    CHECK(run.observe(0.0, state, snapshotFor(false, 0, 0.0), cmd,
                      {true, 100, 100.00}));
    state.t = 100.01;
    CHECK(run.observe(0.01, state, snapshotFor(false, 0, 0.0), cmd,
                      {true, 101, 100.01}));
    // seq 101 applied while 100 is still pending: 100 was skipped
    state.t = 100.02;
    CHECK_FALSE(run.observe(0.02, state,
                            snapshotFor(true, 101, 100.02), cmd,
                            {true, 102, 100.02}));
  }

  SECTION("a delayed first application beyond two cycles aborts") {
    x7::TrackingRun run(chain, map, hold, 6.0, 1.0, 0.5, nullptr);
    state.t = 100.00;
    CHECK(run.observe(0.0, state, snapshotFor(false, 0, 0.0), cmd,
                      {true, 100, 100.00}));
    // seq 100 first applied 30 ms after submission: bound is 20 ms
    state.t = 100.03;
    CHECK_FALSE(run.observe(0.03, state,
                            snapshotFor(true, 100, 100.03), cmd,
                            {true, 101, 100.03}));
  }
}
