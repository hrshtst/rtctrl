// x7_controller_bench: off-hardware cost of one controller cycle.
//
// Measures, on this host, the computation a hardware cycle spends in the
// controller thread, without any bus or servo:
//   follow            one x7_follow cycle: reference sample, command
//                     submission and its CSV row (to /dev/null), run by
//                     FollowRun against an instant in-process arm
//   gravity_comp      GravityComp::update (one gravity-only inverse
//                     dynamics call; x7_float's controller)
//   computed_torque   ComputedTorque::update (reference sample plus full
//                     inverse dynamics; x7_track's controller)
// Percentiles are exact over the recorded calls. They describe this host
// under its current load; they bound nothing on another machine or under
// other load, and they exclude bus transactions and thread handoffs.
//
// Usage: x7_controller_bench [--iterations N] [--rt-priority P --rt-cpu C]
//                            [--fine-timer-slack]

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "common/x7_common.hpp"
#include "follow/follow_config.hpp"
#include "follow/follow_run.hpp"
#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/posture.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/util/realtime.hpp"

namespace arm = rtctrl::arm;
namespace follow = x7::follow;
namespace model = rtctrl::model;
using Clock = std::chrono::steady_clock;

namespace {

// Applies every command at once and stamps each step(), so the gap
// between consecutive steps is one controller cycle's host work.
class InstantArm : public arm::Arm {
 public:
  int dof() const override { return model::kCanonicalDof; }
  double dt() const override { return 0.001; }
  bool activate() override { return true; }
  bool deactivate() override { return true; }
  bool setMode(arm::ControlMode mode) override {
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
    zVecCopyNC(command.q.get(), command_.q.get());
    ++target_seq_;
    if (receipt != nullptr) *receipt = {true, target_seq_, state_.t};
    snapshot_.applied.valid = true;
    snapshot_.applied.target_seq = target_seq_;
    snapshot_.applied.mode = static_cast<std::uint8_t>(mode_);
    return true;
  }
  bool step() override {
    const auto now = Clock::now();
    if (have_last_) {
      cycles_s.push_back(std::chrono::duration<double>(now - last_).count());
    }
    last_ = now;
    have_last_ = true;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      state_.q[i] = command_.q[i];
    }
    state_.t += dt();
    ++state_.seq;
    return true;
  }

  std::vector<double> cycles_s;

 private:
  arm::ControlMode mode_ = arm::ControlMode::Position;
  arm::JointState state_;
  arm::JointCommand command_;
  arm::CommandSnapshot snapshot_;
  std::uint64_t target_seq_ = 0;
  Clock::time_point last_;
  bool have_last_ = false;
};

void report(const char* name, std::vector<double> samples) {
  if (samples.empty()) return;
  std::sort(samples.begin(), samples.end());
  const auto at = [&](double q) {
    const auto i = static_cast<std::size_t>(
        std::min<double>(samples.size() - 1,
                         std::ceil(q * samples.size()) - 1));
    return 1e6 * samples[i];
  };
  std::printf("%-16s n=%-7zu p50 %8.2f  p99 %8.2f  p99.9 %8.2f  max %8.2f us\n",
              name, samples.size(), at(0.5), at(0.99), at(0.999),
              1e6 * samples.back());
}

}  // namespace

int main(int argc, char* argv[]) {
  long iterations = 20000;
  rtctrl::util::RealtimePolicy realtime;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    long n = 0;
    if ((arg == "--iterations" || arg == "--rt-priority" ||
         arg == "--rt-cpu") &&
        i + 1 < argc && x7::parseStrictLong(argv[i + 1], &n)) {
      ++i;
      if (arg == "--iterations") iterations = n;
      if (arg == "--rt-priority") realtime.priority = static_cast<int>(n);
      if (arg == "--rt-cpu") realtime.cpu = static_cast<int>(n);
    } else if (arg == "--fine-timer-slack") {
      realtime.fine_timer_slack = true;
    } else {
      std::fprintf(stderr,
                   "usage: x7_controller_bench [--iterations N] "
                   "[--rt-priority P --rt-cpu C] [--fine-timer-slack]\n");
      return 2;
    }
  }
  if (iterations < 10 || iterations > 10000000) {
    std::fprintf(stderr, "--iterations must be within [10, 1e7]\n");
    return 2;
  }
  std::string why;
  if (!rtctrl::util::applyRealtimePolicy(realtime, &why)) {
    std::fprintf(stderr, "real-time policy refused: %s\n", why.c_str());
    return 1;
  }
  std::printf("scheduler: %s\n",
              rtctrl::util::describeRealtimeState(
                  rtctrl::util::currentRealtimeState())
                  .c_str());

  model::ChainModel chain("models/crane_x7/crane_x7.ztk");
  model::JointMap map(chain);
  const auto p1 = model::loadPostureToml("config/postures/p1.toml");
  const auto p2 = model::loadPostureToml("config/postures/p2.toml");
  model::ZVector q1(model::kCanonicalDof), q2(model::kCanonicalDof);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    q1[i] = p1.joint_positions[i];
    q2[i] = p2.joint_positions[i];
  }
  const double duration = iterations * 0.001;
  model::MinJerkTrajectory trajectory(q1.get(), q2.get(), duration);

  // follow: a reference file at 100 Hz frames, followed at 1 kHz.
  {
    const std::string path = "build/x7_controller_bench_reference.zvs";
    std::remove(path.c_str());
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (f == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", path.c_str());
      return 1;
    }
    model::ZVector q(model::kCanonicalDof);
    for (long k = 0; k * 0.01 <= duration; ++k) {
      trajectory.sample(k * 0.01, q.get());
      std::fprintf(f, "0.01 %d (", model::kCanonicalDof);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::fprintf(f, " %.17g", q[i]);
      }
      std::fprintf(f, " )\n");
    }
    std::fclose(f);
    model::ZvsTrajectory reference(path, map);
    follow::Config config;
    config.mode = arm::ControlMode::Position;
    config.home.motion_time = 0.01;
    config.home.velocity_limit = 100.0;
    config.home.settle_time_s = 0.0;
    config.finalization.simulation_hold_time_s = 0.01;
    InstantArm robot;
    std::FILE* sink = std::fopen("/dev/null", "w");
    follow::FollowCsvLog log(sink, rtctrl::util::AsyncWriter::AdoptStream{});
    follow::FollowRun run(robot, reference, config, true, &log);
    const auto result = run.run();
    log.close();
    std::remove(path.c_str());
    if (result.status != follow::RunStatus::Success) {
      std::fprintf(stderr, "follow run failed: %s\n",
                   follow::statusName(result.status));
      return 1;
    }
    report("follow", robot.cycles_s);
  }

  arm::JointState state;
  arm::JointCommand command;
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(iterations));
  {
    arm::GravityComp controller(chain, map);
    for (long k = 0; k < iterations; ++k) {
      trajectory.sample(k * 0.001, state.q.get());
      const auto start = Clock::now();
      controller.update(state, command, k * 0.001);
      samples.push_back(
          std::chrono::duration<double>(Clock::now() - start).count());
    }
    report("gravity_comp", samples);
  }
  samples.clear();
  {
    arm::ComputedTorque controller(chain, map, trajectory, 6.0, 1.0);
    for (long k = 0; k < iterations; ++k) {
      trajectory.sample(k * 0.001, state.q.get(), state.dq.get());
      const auto start = Clock::now();
      controller.update(state, command, k * 0.001);
      samples.push_back(
          std::chrono::duration<double>(Clock::now() - start).count());
    }
    report("computed_torque", samples);
  }
  return 0;
}
