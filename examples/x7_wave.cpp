// First multi-joint motion: a gentle synchronized wave on tilt, elbow
// and wrist through the RealArm bridge — the same Controller pattern
// that runs on SimArm. Works against dxl_emu or the real robot.
//
// Usage: x7_wave [--config path] [--port dev] [seconds]

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "../apps/x7_common.hpp"

namespace arm = rtctrl::arm;

namespace {

struct WaveController : arm::Controller {
  explicit WaveController(const arm::JointState& start) {
    zVecCopyNC(start.q.get(), home.q.get());
  }
  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    (void)state;
    cmd.mode = arm::ControlMode::Position;
    zVecCopyNC(home.q.get(), cmd.q.get());
    // ramp the amplitude in over 3 s, wave at 0.15 Hz
    const double ramp = std::min(1.0, t / 3.0);
    const double wave = ramp * std::sin(2.0 * M_PI * 0.15 * t);
    zVecElemNC(cmd.q.get(), 1) += 0.15 * wave;  // shoulder tilt
    zVecElemNC(cmd.q.get(), 3) += 0.25 * wave;  // elbow
    zVecElemNC(cmd.q.get(), 5) += 0.20 * wave;  // wrist pitch
  }
  arm::JointState home;
};

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  double duration_s = 15.0;
  if (cli.rest.size() > 1) {
    std::fprintf(stderr, "usage: x7_wave [--config path] [--port dev] "
                         "[seconds]\n");
    return 1;
  }
  if (!cli.rest.empty() &&
      !x7::parseStrictDouble(cli.rest[0], &duration_s)) {
    std::fprintf(stderr, "invalid duration: %s\n", cli.rest[0]);
    return 1;
  }

  try {
    auto session = x7::openSession(cli);
    arm::RealArm robot(*session.arm);

    if (!robot.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   session.arm->lastError().c_str());
      return 1;
    }
    x7::ShutdownGuard shutdown{*session.arm};
    arm::JointState start;
    if (!robot.readState(start)) {
      std::fprintf(stderr, "initial state read failed — aborting\n");
      shutdown.run();
      return 1;
    }
    WaveController controller(start);

    std::printf("waving for %.0f s (Ctrl-C safe: deadman + watchdogs)\n",
                duration_s);
    const bool ok = arm::run(robot, controller, duration_s);
    const bool clean = shutdown.run();
    if (!clean) {
      std::printf("SHUTDOWN FAULT (run %s)\n", ok ? "done" : "ABORTED");
      return 1;
    }
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
