#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <fstream>

#include "rtctrl/arm/arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "track/track_config.hpp"
#include "track/track_run.hpp"

namespace track = x7::track;
namespace model = rtctrl::model;
namespace arm = rtctrl::arm;
using Catch::Approx;

namespace {

const char* writeConfig(const char* path, const std::string& extra = {}) {
  std::ofstream out(path);
  out << "format = \"rtctrl-x7-track\"\n"
      << "version = 1\n"
      << "model = \"../../models/crane_x7/crane_x7.ztk\"\n"
      << "reference = \"reference.zvs\"\n"
      << "hardware_config = \"../../config/crane_x7.toml\"\n"
      << extra;
  return path;
}

class FixedArm : public arm::Arm {
 public:
  explicit FixedArm(double error) { state_.q[1] = -error; }
  int dof() const override { return model::kCanonicalDof; }
  double dt() const override { return 0.01; }
  bool activate() override { return true; }
  bool deactivate() override { return true; }
  bool setMode(arm::ControlMode) override { return true; }
  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* snapshot = nullptr) override {
    zVecCopyNC(state_.q.get(), state.q.get());
    zVecCopyNC(state_.dq.get(), state.dq.get());
    zVecCopyNC(state_.tau.get(), state.tau.get());
    state.t = time_;
    state.seq = seq_;
    if (snapshot != nullptr) *snapshot = {};
    return true;
  }
  bool writeCommand(const arm::JointCommand&,
                    arm::CommandReceipt* receipt = nullptr) override {
    if (receipt != nullptr) *receipt = {true, ++target_seq_, time_};
    return true;
  }
  bool step() override {
    time_ += dt();
    ++seq_;
    return true;
  }

 private:
  arm::JointState state_;
  double time_ = 0.0;
  std::uint64_t seq_ = 0;
  std::uint64_t target_seq_ = 0;
};

}  // namespace

TEST_CASE("track config defaults to textbook current tracking",
          "[track][config]") {
  const char* path = "build/track_default.toml";
  writeConfig(path);
  const auto config = track::loadConfig(path);
  CHECK(config.control_rate_hz == 100.0);
  CHECK(config.kp == 20.0);
  CHECK(config.kd == 2.0);
  CHECK(config.playback_rate == 1.0);
  CHECK(config.assessment.rms_error_rad == 0.02);
  CHECK(config.assessment.peak_error_rad == 0.10);
  CHECK_FALSE(config.effort_limit_set);
  std::remove(path);
}

TEST_CASE("track config and CLI reject unsupported or unsafe controls",
          "[track][config]") {
  const char* slow = "build/track_slow.toml";
  writeConfig(slow,
              "[control]\nplayback_rate = 0.5\n"
              "effort_limit_nm = [2.5]\n");
  const auto config = track::loadConfig(slow);
  CHECK(config.playback_rate == 0.5);
  CHECK(config.effort_limit_set);
  CHECK(config.effort_limit_nm[7] == 2.5);
  std::remove(slow);

  const char* fast = "build/track_fast.toml";
  writeConfig(fast, "[control]\nplayback_rate = 1.1\n");
  CHECK_THROWS(track::loadConfig(fast));
  std::remove(fast);

  char app[] = "x7_track";
  char config_flag[] = "--config";
  char config_path[] = "run.toml";
  char mode[] = "--mode";
  char current[] = "current";
  char* unsupported[] = {app, config_flag, config_path, mode, current};
  CHECK_THROWS(track::parseCli(5, unsupported, false));
}

TEST_CASE("playback trajectory scales time derivatives exactly",
          "[track][trajectory]") {
  model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
  qf[2] = 0.8;
  model::MinJerkTrajectory source(q0.get(), qf.get(), 2.0);
  track::PlaybackTrajectory slow(source, 0.5);
  CHECK(slow.duration() == 4.0);
  model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
      ddq(model::kCanonicalDof), source_q(model::kCanonicalDof),
      source_dq(model::kCanonicalDof), source_ddq(model::kCanonicalDof);
  slow.sample(1.2, q.get(), dq.get(), ddq.get());
  source.sample(0.6, source_q.get(), source_dq.get(), source_ddq.get());
  CHECK(q[2] == Approx(source_q[2]));
  CHECK(dq[2] == Approx(0.5 * source_dq[2]));
  CHECK(ddq[2] == Approx(0.25 * source_ddq[2]));
}

TEST_CASE("tracking assessment failure completes while hard error aborts",
          "[track][run]") {
  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  model::ZVector zero(model::kCanonicalDof);
  model::MinJerkTrajectory hold(zero.get(), zero.get(), 0.05);

  track::Config config;
  config.finalization.simulation_hold_time_s = 0.0;
  config.assessment.rms_error_rad = 0.01;
  config.assessment.peak_error_rad = 0.02;
  FixedArm inaccurate(0.05);
  track::TrackingRun ordinary(inaccurate, chain, map, hold, config, true);
  const auto ordinary_result = ordinary.run();
  CHECK(ordinary_result.status == track::RunStatus::Success);
  CHECK_FALSE(ordinary_result.tracking_pass);
  CHECK(ordinary_result.worst_joint_rms_error_rad == Approx(0.05));

  config.safety.immediate_abort_error_rad = 0.4;
  config.safety.sustained_abort_error_rad = 0.3;
  config.safety.warning_error_rad = 0.2;
  FixedArm unsafe(0.5);
  track::TrackingRun guarded(unsafe, chain, map, hold, config, true);
  const auto guarded_result = guarded.run();
  CHECK(guarded_result.status == track::RunStatus::HardTrackingError);
}
