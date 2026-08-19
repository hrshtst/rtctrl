// First multi-joint motion: a gentle synchronized wave on tilt, elbow
// and wrist through the RealArm bridge — the same Controller pattern
// that runs on SimArm. Works against dxl_emu or the real robot.
//
// Usage: x7_wave [--config path] [--port dev] [--zvs file.zvs] [seconds]
// --zvs exports the COMMANDED position reference, cycle by cycle, as a
// 9-coordinate joint-displacement sequence — so the identical motion
// the hardware was told to make replays in the roki viewer:
//   rk_anim models/crane_x7/crane_x7.ztk file.zvs

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"
#include "../apps/common/x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

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

// Taps the runner's per-cycle hook to export the commanded reference:
// exactly what was submitted to the arm, ramp included.
struct ZvsObserver : arm::CycleObserver {
  ZvsObserver(const model::JointMap& map, model::ZvsWriter& writer,
              double dt)
      : map_(map), writer_(writer), dt_(dt) {}
  bool observe(double, const arm::JointState&, const arm::CommandSnapshot&,
               const arm::JointCommand& cmd,
               const arm::CommandReceipt&) override {
    map_.expand(cmd.q.get(), q9_.get());
    writer_.frame(dt_, q9_.get());
    return true;
  }
  const model::JointMap& map_;
  model::ZvsWriter& writer_;
  double dt_;
  model::ZVector q9_{model::kModelDof};
};

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  std::string zvs_path;
  std::vector<const char*> positional;
  for (std::size_t i = 0; i < cli.rest.size(); ++i) {
    if (std::strcmp(cli.rest[i], "--zvs") == 0) {
      // An empty value would silently alias the "option absent" state.
      if (i + 1 >= cli.rest.size() || cli.rest[i + 1][0] == '\0') {
        std::fprintf(stderr, "--zvs requires a non-empty value\n");
        return 1;
      }
      zvs_path = cli.rest[++i];
    } else {
      positional.push_back(cli.rest[i]);
    }
  }
  double duration_s = 15.0;
  if (positional.size() > 1) {
    std::fprintf(stderr, "usage: x7_wave [--config path] [--port dev] "
                         "[--zvs file.zvs] [seconds]\n");
    return 1;
  }
  if (!positional.empty() &&
      !x7::parseStrictDouble(positional[0], &duration_s)) {
    std::fprintf(stderr, "invalid duration: %s\n", positional[0]);
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

    std::unique_ptr<model::ChainModel> chain;
    std::unique_ptr<model::JointMap> map;
    std::unique_ptr<model::ZvsWriter> zvs;
    std::unique_ptr<ZvsObserver> observer;
    if (!zvs_path.empty()) {
      chain = std::make_unique<model::ChainModel>(
          "models/crane_x7/crane_x7.ztk");
      map = std::make_unique<model::JointMap>(*chain);
      zvs = std::make_unique<model::ZvsWriter>(zvs_path);
      observer = std::make_unique<ZvsObserver>(*map, *zvs, robot.dt());
    }

    std::printf("waving for %.0f s (Ctrl-C safe: deadman + watchdogs)\n",
                duration_s);
    const bool ok = arm::run(robot, controller, duration_s, observer.get());
    const bool clean = shutdown.run();
    if (zvs != nullptr && zvs->frames() > 0) {
      std::printf("%d reference zvs frames -> %s\n  view with:  rk_anim "
                  "models/crane_x7/crane_x7.ztk %s\n",
                  zvs->frames(), zvs_path.c_str(), zvs_path.c_str());
    }
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
