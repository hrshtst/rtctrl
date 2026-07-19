// M7 acceptance in sim, through the bridge: under pure gravity
// compensation in current mode the arm floats — it neither falls nor
// runs away.
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "rtctrl/arm/gravity_comp.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"

using rtctrl::arm::ControlMode;
using rtctrl::arm::GravityComp;
using rtctrl::arm::JointState;
using rtctrl::arm::SimArm;
using rtctrl::model::ChainModel;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;

TEST_CASE("gravity compensation floats the arm in sim", "[gravity][sim]") {
  SimArm::Options opt;
  opt.model_path = "models/crane_x7/crane_x7.ztk";
  // a strongly gravity-loaded pose
  opt.initial_q8 = {0.0, 0.8, 0.0, -1.3, 0.0, -0.5, 0.0, 0.2};
  SimArm arm(opt);
  REQUIRE(arm.setMode(ControlMode::Current));
  REQUIRE(arm.activate());

  ChainModel model(opt.model_path);
  JointMap map(model);
  GravityComp controller(model, map);

  JointState initial;
  REQUIRE(arm.readState(initial));
  REQUIRE(rtctrl::arm::run(arm, controller, 10.0));

  JointState final;
  REQUIRE(arm.readState(final));
  for (int i = 0; i < kCanonicalDof; ++i) {
    INFO("joint " << i);
    CHECK(std::fabs(final.q[i] - initial.q[i]) < 0.05);
    CHECK(std::isfinite(final.dq[i]));
  }
}
