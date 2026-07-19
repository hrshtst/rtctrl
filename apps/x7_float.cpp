// M7 hardware phase: gravity compensation on the real arm. The arm
// floats — held against gravity, freely back-drivable by hand — while
// the console compares measured torque (current × torque constant)
// against the rkChainID_G prediction each second.
//
// SAFETY: current mode. Keep the power cutoff in reach; support the
// arm lightly on the first run. Verified in sim first
// (gravity_sim_test): drift < 0.05 rad over 10 s.
//
// Usage: x7_float [--config path] [--port dev] [seconds]

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

namespace {

struct ReportingGravityComp : arm::GravityComp {
  ReportingGravityComp(model::ChainModel& chain,
                       const model::JointMap& map)
      : GravityComp(chain, map), chain_(chain), map_(map) {}

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    GravityComp::update(state, cmd, t);
    if (t - last_report_ >= 1.0) {
      last_report_ = t;
      chain_.gravityTorque(map_, state.q.get(), predicted_.get());
      std::printf("t=%4.0fs  measured vs predicted tau [Nm]:\n", t);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::printf("  j%d %+6.2f / %+6.2f", i,
                    zVecElemNC(state.tau.get(), i), predicted_[i]);
      }
      std::printf("\n");
    }
  }

  model::ChainModel& chain_;
  const model::JointMap& map_;
  model::ZVector predicted_{model::kCanonicalDof};
  double last_report_ = -1.0;
};

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  const double duration_s = cli.argi < argc ? std::atof(argv[cli.argi]) : 30.0;

  try {
    // gravity compensation runs in current (torque) mode
    auto session = x7::openSession(cli, /*operating_mode_override=*/0);

    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);
    arm::RealArm robot(*session.arm);

    if (!robot.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   session.arm->lastError().c_str());
      return 1;
    }
    ReportingGravityComp controller(chain, map);
    std::printf("floating for %.0f s — the arm is back-drivable; keep the "
                "power cutoff in reach\n",
                duration_s);
    const bool ok = arm::run(robot, controller, duration_s);
    robot.deactivate();
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
