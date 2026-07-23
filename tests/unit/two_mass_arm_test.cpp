// Ground-truth checks on the TwoMassArm identification fixture: the
// planted transmission modes must ring at their designed frequency and
// damping, and the synchronous CommandSnapshot synthesis must satisfy
// the latency verifier the identification run reuses.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include "latency_verifier.hpp"
#include "two_mass_arm.hpp"

using Catch::Approx;
namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
using model::kCanonicalDof;

namespace {

// Ring the gear spring of joint `j` and fit the free decay of the
// relative coordinate δ = q_m − q_l (a damped SDOF at the transmission
// mode). Returns {frequency Hz, damping ratio} from zero crossings and
// the log decrement over the positive-peak envelope.
struct RingFit {
  double f_hz = 0.0;
  double zeta = 0.0;
};

RingFit ringDown(int j, double seconds) {
  x7::TwoMassArm::Options opt;
  opt.control_dt = 1e-3;  // fine sampling for the 13 Hz mode
  x7::TwoMassArm plant(opt);
  plant.deflectGear(j, 0.02);
  std::vector<double> t;
  std::vector<double> d;
  const long steps = std::lround(seconds / opt.control_dt);
  for (long k = 0; k < steps; ++k) {
    REQUIRE(plant.step());
    t.push_back(plant.time());
    d.push_back(plant.motorPos(j) - plant.linkPos(j));
  }
  // frequency: mean interval between upward zero crossings
  std::vector<double> crossings;
  for (std::size_t k = 1; k < d.size(); ++k) {
    if (d[k - 1] < 0.0 && d[k] >= 0.0) {
      const double frac = -d[k - 1] / (d[k] - d[k - 1]);
      crossings.push_back(t[k - 1] + frac * (t[k] - t[k - 1]));
    }
  }
  REQUIRE(crossings.size() >= 4);
  // damping: log decrement between the first and last positive peak
  std::vector<double> peaks;
  for (std::size_t k = 1; k + 1 < d.size(); ++k) {
    if (d[k] > d[k - 1] && d[k] >= d[k + 1] && d[k] > 0.0) {
      peaks.push_back(d[k]);
    }
  }
  REQUIRE(peaks.size() >= 3);
  RingFit fit;
  fit.f_hz = (crossings.size() - 1) / (crossings.back() - crossings.front());
  const double n = static_cast<double>(peaks.size() - 1);
  fit.zeta = std::log(peaks.front() / peaks.back()) / (2.0 * M_PI * n);
  return fit;
}

}  // namespace

TEST_CASE("two-mass fixture rings at the planted modes",
          "[two_mass_arm]") {
  SECTION("joint 1: 4.5 Hz, zeta 0.03") {
    const auto fit = ringDown(1, 2.0);
    INFO("f " << fit.f_hz << " Hz, zeta " << fit.zeta);
    CHECK(fit.f_hz == Approx(4.5).margin(0.1));
    CHECK(fit.zeta == Approx(0.03).epsilon(0.35));
  }
  SECTION("joint 5: 13 Hz, zeta 0.05") {
    const auto fit = ringDown(5, 1.0);
    INFO("f " << fit.f_hz << " Hz, zeta " << fit.zeta);
    CHECK(fit.f_hz == Approx(13.0).margin(0.3));
    CHECK(fit.zeta == Approx(0.05).epsilon(0.35));
  }
}

TEST_CASE("two-mass fixture planted stiffness matches the design math",
          "[two_mass_arm]") {
  const auto p1 = x7::TwoMassArm::plantMode(0.4, 0.05, 4.5, 0.03);
  CHECK(p1.k_g == Approx(35.5).margin(0.2));
  CHECK(p1.c_g == Approx(0.075).margin(0.002));
  const auto p5 = x7::TwoMassArm::plantMode(0.01, 0.05, 13.0, 0.05);
  CHECK(p5.k_g == Approx(55.6).margin(0.3));
  CHECK(p5.c_g == Approx(0.068).margin(0.002));
}

TEST_CASE("two-mass fixture snapshots satisfy the latency verifier",
          "[two_mass_arm]") {
  x7::TwoMassArm plant;
  REQUIRE(plant.setMode(arm::ControlMode::Current));
  REQUIRE(plant.activate());
  x7::LatencyVerifier verifier;
  arm::JointState state;
  arm::JointCommand cmd;
  cmd.mode = arm::ControlMode::Current;
  for (int cycle = 0; cycle < 50; ++cycle) {
    arm::CommandSnapshot snap;
    REQUIRE(plant.readState(state, &snap));
    for (int i = 0; i < kCanonicalDof; ++i) {
      zVecElemNC(cmd.tau.get(), i) = 0.01 * cycle;
    }
    arm::CommandReceipt receipt;
    REQUIRE(plant.writeCommand(cmd, &receipt));
    double delay = 0.0;
    REQUIRE(verifier.check(state.t, snap, receipt, &delay));
    REQUIRE(plant.step());
  }
}

TEST_CASE("two-mass fixture clamps torque and flags it",
          "[two_mass_arm]") {
  x7::TwoMassArm plant;
  REQUIRE(plant.activate());
  arm::JointCommand cmd;
  cmd.mode = arm::ControlMode::Current;
  zVecElemNC(cmd.tau.get(), 1) = 100.0;  // limit is 10 Nm
  REQUIRE(plant.writeCommand(cmd));
  arm::JointState state;
  arm::CommandSnapshot snap;
  REQUIRE(plant.readState(state, &snap));
  CHECK(snap.applied.applied[1] == Approx(10.0));
  CHECK((snap.applied.flags[1] & arm::kCmdClamped) != 0);
  CHECK(zVecElemNC(state.tau.get(), 1) == Approx(10.0));
  CHECK(snap.applied.flags[0] == 0);
}
