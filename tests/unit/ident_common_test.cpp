// Identification-run regressions: the dwell state machine's timing,
// the hard/soft fault taxonomy, the emergency kill ramp, adaptive-hold
// convergence, the session budget, health supervision, and the anchor
// gate — all on scripted mocks with known signals.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <functional>
#include <random>
#include <vector>

#include "ident_common.hpp"

using Catch::Approx;
namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
using model::kCanonicalDof;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

// An Arm whose joint positions follow a caller-supplied q(joint, t) and
// whose measured torque echoes the last written command (as a current-
// sensing servo would); synchronous snapshot synthesis like SimArm.
class ScriptArm : public arm::Arm {
 public:
  using Script = std::function<double(int, double)>;
  explicit ScriptArm(Script q) : q_(std::move(q)) {}

  int dof() const override { return kCanonicalDof; }
  double dt() const override { return 0.01; }
  bool activate() override { return true; }
  bool deactivate() override { return true; }
  bool setMode(arm::ControlMode) override { return true; }

  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* cmds = nullptr) override {
    for (int i = 0; i < kCanonicalDof; ++i) {
      zVecElemNC(state.q.get(), i) = q_(i, time_);
      zVecElemNC(state.dq.get(), i) = 0.0;
      zVecElemNC(state.tau.get(), i) = echo_[i];
    }
    state.t = 100.0 + time_;
    state.seq = seq_;
    if (cmds != nullptr) {
      cmds->applied = applied_rec_;
      cmds->last_attempt = attempt_rec_;
    }
    return true;
  }

  bool writeCommand(const arm::JointCommand& cmd,
                    arm::CommandReceipt* receipt = nullptr) override {
    ++target_seq_;
    attempt_rec_ = {true, target_seq_, 100.0 + time_, true};
    auto& rec = applied_rec_;
    rec.valid = true;
    rec.target_seq = target_seq_;
    rec.first_cycle = rec.latest_cycle = seq_;
    rec.first_time = rec.latest_time = 100.0 + time_;
    rec.mode = static_cast<std::uint8_t>(cmd.mode);
    for (int i = 0; i < kCanonicalDof; ++i) {
      echo_[i] = zVecElemNC(cmd.tau.get(), i);
      rec.applied[i] = echo_[i];
      rec.flags[i] = 0;
    }
    if (receipt != nullptr) *receipt = {true, target_seq_, 100.0 + time_};
    return true;
  }

  bool step() override {
    time_ += 0.01;
    ++seq_;
    return true;
  }

  double time() const { return time_; }

 private:
  Script q_;
  double time_ = 0.0;
  std::uint64_t seq_ = 1;
  std::uint64_t target_seq_ = 0;
  double echo_[kCanonicalDof] = {};
  arm::AppliedTargetRecord applied_rec_;
  arm::WriteAttemptRecord attempt_rec_;
};

x7::IdentRun::Options baseOptions(std::vector<x7::DwellSpec> dwells) {
  x7::IdentRun::Options o;
  o.probe_joint = 1;
  o.dwells = std::move(dwells);
  o.anchor.assign(kCanonicalDof, 0.0);
  o.tau_max.assign(kCanonicalDof, 10.0);
  return o;
}

struct Fixture {
  model::ChainModel chain{kModelPath};
  model::JointMap map{chain};
};

}  // namespace

TEST_CASE("ident schedule arithmetic matches the plan's budget",
          "[ident]") {
  std::vector<x7::DwellSpec> survey;
  for (const double f : {2.0, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0,
                         8.0, 10.0, 12.0, 13.0, 14.0, 16.0, 20.0}) {
    survey.push_back({f, x7::probeAmplitude(0.05, f), 1});
  }
  const double worst = x7::baseWorstSeconds(survey);
  INFO("survey worst-case base " << worst << " s");
  CHECK(worst == Approx(146.0).margin(2.0));
  CHECK(x7::scheduleFitsBudget(survey));

  // an over-long custom schedule is refused before activation
  std::vector<x7::DwellSpec> too_long(30, {2.0, 0.1, 1});
  CHECK_FALSE(x7::scheduleFitsBudget(too_long));

  // amplitude rule: floor, scaling, cap
  CHECK(x7::probeAmplitude(0.05, 2.0) ==
        Approx(0.05).margin(1e-9));  // floored
  CHECK(x7::probeAmplitude(0.05, 20.0) == Approx(0.15));  // capped
}

TEST_CASE("ident state machine: still arm runs hold to timeout and an "
          "integer-period window",
          "[ident]") {
  Fixture fx;
  auto opt = baseOptions({{5.0, 0.15, 1}});
  x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
  ScriptArm robot([](int, double) { return 0.0; });
  CHECK_FALSE(arm::run(robot, run, 60.0, &run));  // ends by Done veto
  REQUIRE(run.finishedCleanly());
  CHECK(run.outcome() == x7::IdentRun::Outcome::Completed);
  const auto& r = run.results()[0];
  CHECK(r.started);
  CHECK(r.completed);
  // a still arm never satisfies |Z| >= floor: hold must time out
  CHECK(r.low_confidence);
  CHECK(r.hold_s == Approx(x7::kHoldMaxS).margin(0.05));
  // measurement window: integer periods, minimum max(10 periods, 3 s)
  CHECK(r.window_periods == 15);
  CHECK(r.window_s == Approx(15.0 / 5.0).margin(0.05));
  CHECK(r.window_s * 5.0 == Approx(r.window_periods).margin(0.3));
}

TEST_CASE("ident adaptive hold accepts early on a settled response and "
          "recovers the planted amplitude",
          "[ident]") {
  Fixture fx;
  auto opt = baseOptions({{5.0, 0.15, 1}});
  x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
  // response appears only after the lead-in (the floors calibrate on
  // stillness), steady from before the dwell's hold phase
  ScriptArm robot([](int i, double t) {
    return (i == 1 && t > 2.4) ? 0.005 * std::sin(2.0 * M_PI * 5.0 * t)
                               : 0.0;
  });
  CHECK_FALSE(arm::run(robot, run, 60.0, &run));
  REQUIRE(run.finishedCleanly());
  const auto& r = run.results()[0];
  CHECK(r.completed);
  CHECK_FALSE(r.low_confidence);
  // convergence at the first agreeing block pair past the 1 s minimum
  CHECK(r.hold_s >= x7::kHoldMinS - 1e-6);
  CHECK(r.hold_s < 2.0);
  // the window-aggregate demod recovers the scripted 5 mrad response
  CHECK(r.resp[1].abs() == Approx(0.005).epsilon(0.1));
  // BOTH torque estimates demodulate the TOTAL actuator command —
  // probe PLUS the anchor PD's reaction to the scripted motion (the
  // actuator-transfer variant needs the actual command, and the
  // direct-ratio primary needs total measured torque; the probe alone
  // is only the excitation reference). With this echo mock the
  // measured signal IS last cycle's command, so the two agree.
  CHECK(r.tau_meas.abs() > 0.05);
  CHECK(r.tau_meas.abs() < 0.25);
  CHECK(r.tau_cmd.abs() == Approx(r.tau_meas.abs()).epsilon(0.05));
}

TEST_CASE("ident soft event on a NON-probe joint: kill ramp, re-settle, "
          "one retry, then abort on recurrence",
          "[ident]") {
  Fixture fx;
  auto opt = baseOptions({{5.0, 0.15, 1}});
  x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
  // joint 3 oscillates above the 0.03 rad response cap from just after
  // the lead-in and never stops: the retry must also fail
  ScriptArm robot([](int i, double t) {
    return (i == 3 && t > 2.4) ? 0.05 * std::sin(2.0 * M_PI * 5.0 * t)
                               : 0.0;
  });
  CHECK_FALSE(arm::run(robot, run, 60.0, &run));
  CHECK(run.outcome() == x7::IdentRun::Outcome::SoftAbort);
  CHECK_FALSE(run.finishedCleanly());
  const auto& r = run.results()[0];
  CHECK(r.soft_events == 2);
  CHECK(r.retried);
  CHECK_FALSE(r.completed);
  CHECK(r.note.find("response cap") != std::string::npos);
}

TEST_CASE("ident emergency kill ramp removes the probe within 50 ms",
          "[ident]") {
  Fixture fx;
  auto opt = baseOptions({{5.0, 0.15, 1}});
  x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
  ScriptArm robot([](int i, double t) {
    return (i == 3 && t > 2.4) ? 0.05 * std::sin(2.0 * M_PI * 5.0 * t)
                               : 0.0;
  });
  // drive the loop manually to watch the kill ramp cycle by cycle
  arm::JointState state;
  arm::JointCommand cmd;
  bool saw_kill = false;
  double kill_entry_t = 0.0;
  double kill_entry_env = 0.0;
  for (int k = 0; k < 2000; ++k) {
    REQUIRE(robot.readState(state));
    const double t = robot.time();  // scripted time tracks measured time
    run.update(state, cmd, t);
    REQUIRE(robot.writeCommand(cmd));
    REQUIRE(robot.step());
    if (run.phase() == x7::IdentRun::Phase::KillRamp && !saw_kill) {
      saw_kill = true;
      kill_entry_t = t;
      kill_entry_env = std::fabs(run.probeTau());
    } else if (saw_kill &&
               run.phase() == x7::IdentRun::Phase::ReSettle) {
      // the ramp completed: probe fully removed within 50 ms + 1 cycle
      CHECK(t - kill_entry_t <= x7::kKillRampS + 0.011);
      CHECK(run.probeTau() == 0.0);
      break;
    } else if (saw_kill) {
      // during the ramp the probe magnitude is bounded by the dwell
      // amplitude — a half-cosine removal, never a step or overshoot
      CHECK(std::fabs(run.probeTau()) <= 0.15 + 1e-9);
    }
  }
  CHECK(saw_kill);
  CHECK(kill_entry_env <= 0.15 + 1e-9);
}

TEST_CASE("ident hard faults abort with no retry", "[ident]") {
  Fixture fx;

  SECTION("anchor deviation") {
    auto opt = baseOptions({{5.0, 0.15, 1}});
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int i, double t) {
      return (i == 2 && t > 3.0) ? 0.1 : 0.0;  // 0.1 >= 0.08 abort
    });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    CHECK(run.outcome() == x7::IdentRun::Outcome::HardFault);
    CHECK(run.faultReason().find("deviation") != std::string::npos);
    CHECK(run.results()[0].soft_events == 0);  // no retry machinery
  }

  SECTION("thermal limit via the scripted HealthProvider") {
    auto opt = baseOptions({{5.0, 0.15, 1}});
    int calls = 0;
    opt.health = [&calls](std::array<x7::JointHealth,
                                     kCanonicalDof>& h) {
      ++calls;
      for (auto& j : h) {
        j.temp_c = calls > 250 ? 70.0 : 40.0;  // heats mid-run
        j.volt_v = 12.0;
      }
      return true;
    };
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int, double) { return 0.0; });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    CHECK(run.outcome() == x7::IdentRun::Outcome::HardFault);
    CHECK(run.faultReason().find("thermal") != std::string::npos);
  }

  SECTION("supply voltage out of range") {
    auto opt = baseOptions({{5.0, 0.15, 1}});
    opt.health = [](std::array<x7::JointHealth, kCanonicalDof>& h) {
      for (auto& j : h) {
        j.temp_c = 40.0;
        j.volt_v = 10.0;  // below the 10.5 V abort floor
      }
      return true;
    };
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int, double) { return 0.0; });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    CHECK(run.outcome() == x7::IdentRun::Outcome::HardFault);
    CHECK(run.faultReason().find("voltage") != std::string::npos);
  }
}

TEST_CASE("ident session budget at runtime", "[ident]") {
  Fixture fx;

  SECTION("post-settle admission: setup that ate the margin refuses "
          "every dwell and terminates gracefully") {
    auto opt = baseOptions({{5.0, 0.15, 1}, {5.0, 0.15, 1}});
    opt.setup_offset_s = 170.0;  // T_stop 177.5: nothing fits
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int, double) { return 0.0; });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    REQUIRE(run.finishedCleanly());
    CHECK(run.outcome() == x7::IdentRun::Outcome::DeadlineStop);
    for (const auto& r : run.results()) {
      CHECK(r.skipped);
      CHECK_FALSE(r.started);
    }
  }

  SECTION("window extension is denied when it would breach T_stop") {
    auto opt = baseOptions({{5.0, 0.15, 4}});  // x4 requested
    opt.t_stop_s = 12.0;  // base fits (~10.5), +9 s extension does not
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int, double) { return 0.0; });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    REQUIRE(run.finishedCleanly());
    const auto& r = run.results()[0];
    CHECK(r.completed);
    CHECK(r.low_confidence);
    CHECK(r.window_periods == 15);  // un-extended
    CHECK(r.note.find("extension denied") != std::string::npos);
  }

  SECTION("soft-event retry is denied when it would breach T_stop") {
    auto opt = baseOptions({{5.0, 0.15, 1}});
    opt.t_stop_s = 13.0;  // dwell fits; a post-re-settle retry does not
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int i, double t) {
      return (i == 3 && t > 2.4)
                 ? 0.05 * std::sin(2.0 * M_PI * 5.0 * t)
                 : 0.0;
    });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    REQUIRE(run.finishedCleanly());
    const auto& r = run.results()[0];
    CHECK(r.soft_events == 1);
    CHECK_FALSE(r.retried);
    CHECK(r.note.find("retry denied") != std::string::npos);
  }

  SECTION("window extension granted when the budget allows") {
    auto opt = baseOptions({{5.0, 0.15, 4}});
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);  // default T_stop
    ScriptArm robot([](int, double) { return 0.0; });
    CHECK_FALSE(arm::run(robot, run, 90.0, &run));
    REQUIRE(run.finishedCleanly());
    CHECK(run.results()[0].window_periods == 60);  // 15 x 4
  }
}

TEST_CASE("ident headroom precheck reduces or skips the dwell",
          "[ident]") {
  Fixture fx;

  SECTION("no headroom: dwell skipped below the amplitude floor") {
    auto opt = baseOptions({{5.0, 0.15, 1}});
    opt.tau_max.assign(kCanonicalDof, 10.0);
    opt.tau_max[4] = 0.2;  // any joint below A + 0.2 margin binds
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int, double) { return 0.0; });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    REQUIRE(run.finishedCleanly());
    const auto& r = run.results()[0];
    CHECK(r.skipped);
    CHECK(r.note.find("headroom") != std::string::npos);
  }

  SECTION("partial headroom: amplitude reduced into the margin") {
    auto opt = baseOptions({{5.0, 0.15, 1}});
    opt.tau_max[4] = 0.3;  // headroom ~0.1: amp reduced 0.15 -> ~0.1
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([](int, double) { return 0.0; });
    CHECK_FALSE(arm::run(robot, run, 60.0, &run));
    REQUIRE(run.finishedCleanly());
    const auto& r = run.results()[0];
    CHECK(r.completed);
    CHECK(r.spec.amp_nm == Approx(0.1).margin(0.02));
    CHECK(r.note.find("reduced") != std::string::npos);
  }
}

TEST_CASE("ident convergence rule: no premature acceptance on a "
          "decaying transient with noise",
          "[ident]") {
  // Direct rule test: probe-frequency component 0.005 rad plus an
  // onset-excited free decay (4.5 Hz, tau ~1.2 s) and encoder-scale
  // noise. Blocks must NOT converge while the transient dominates.
  const double f_probe = 5.0;
  const double f_free = 4.5;
  const double dt = 0.01;
  std::minstd_rand rng(12345);
  std::uniform_real_distribution<double> noise(-4.4e-4, 4.4e-4);
  x7::BlockDemod demod;
  double phi = 0.0;
  x7::BlockEstimate prev;
  bool have_prev = false;
  const double floor = 3.0 * 2.0e-4;  // representative q floor [rad]
  double accepted_at = -1.0;
  for (double t = 0.0; t < 6.0; t += dt) {
    phi += 2.0 * M_PI * f_probe * dt;
    const double transient = 0.02 * std::exp(-t / 1.2) *
                             std::sin(2.0 * M_PI * f_free * t);
    const double s = 0.005 * std::sin(phi - 0.7) + transient + noise(rng);
    if (demod.add(phi, dt, s)) {
      if (have_prev &&
          x7::blocksConverged(prev, demod.lastBlock(), floor) &&
          accepted_at < 0.0 && t >= x7::kHoldMinS) {
        accepted_at = t;
      }
      prev = demod.lastBlock();
      have_prev = true;
    }
  }
  REQUIRE(accepted_at > 0.0);  // it does converge eventually
  // At acceptance the free transient must have decayed to the same
  // order as the steady response: 0.02 e^(-t/1.2) <= ~40% of 5 mrad.
  INFO("accepted at " << accepted_at << " s");
  CHECK(0.02 * std::exp(-accepted_at / 1.2) < 0.4 * 0.005);
}

TEST_CASE("ident pre-run gates: health and anchor reference",
          "[ident]") {
  std::array<x7::JointHealth, kCanonicalDof> h;
  for (auto& j : h) {
    j.temp_c = 45.0;
    j.volt_v = 12.0;
  }
  CHECK(x7::preRunHealthOk(h));
  h[3].temp_c = 60.0;  // above the 55 C pre-run gate
  std::string why;
  CHECK_FALSE(x7::preRunHealthOk(h, &why));
  CHECK(why.find("joint 3") != std::string::npos);
  h[3].temp_c = 45.0;
  h[6].volt_v = 10.8;  // below the 11.0 V pre-run gate
  CHECK_FALSE(x7::preRunHealthOk(h));

  double settled[kCanonicalDof] = {};
  double ref[kCanonicalDof] = {};
  settled[2] = 0.019;
  CHECK(x7::anchorWithinTolerance(settled, ref, x7::kAnchorToleranceRad));
  settled[2] = 0.021;
  double deltas[kCanonicalDof] = {};
  CHECK_FALSE(x7::anchorWithinTolerance(settled, ref,
                                        x7::kAnchorToleranceRad, deltas));
  CHECK(deltas[2] == Approx(0.021));
}

TEST_CASE("ident block demod is immune to a large constant offset and "
          "drift",
          "[ident]") {
  // The review repro: at the P1 posture q4 = -2.456 rad, the plain
  // correlation's boundary quantization leaked ~0.096 rad into a
  // still joint's estimate — over three times the response cap. The
  // LS regressors [1, t, sin, cos] must absorb offset and drift.
  const double dt = 0.01;

  SECTION("still joint at the P1 offset reads ~zero") {
    x7::BlockDemod demod;
    double phi = 0.0;
    double worst = 0.0;
    int blocks = 0;
    for (double t = 0.0; t < 3.0; t += dt) {
      phi += 2.0 * M_PI * 2.0 * dt;  // the 2 Hz worst case
      if (demod.add(phi, dt, -2.456 + 0.001 * t)) {
        worst = std::max(worst, demod.lastBlock().abs());
        ++blocks;
      }
    }
    REQUIRE(blocks >= 4);
    INFO("worst still-joint block amplitude " << worst);
    CHECK(worst < 1e-9);
  }

  SECTION("a small sine riding the offset is recovered exactly") {
    x7::BlockDemod demod;
    double phi = 0.0;
    int blocks = 0;
    for (double t = 0.0; t < 3.0; t += dt) {
      phi += 2.0 * M_PI * 2.0 * dt;
      const double y =
          -2.456 + 0.001 * t + 0.004 * std::sin(phi - 1.0);
      if (demod.add(phi, dt, y)) ++blocks;
    }
    REQUIRE(blocks >= 4);
    CHECK(demod.lastBlock().abs() == Approx(0.004).epsilon(0.01));
  }
}

TEST_CASE("ident run at the P1 anchor: no false monitor trips",
          "[ident]") {
  Fixture fx;
  const std::vector<double> p1 = {-0.357, -0.831, 2.126, -1.572,
                                  -2.456, -0.106, 0.563, -0.014};

  SECTION("still arm at P1 completes with zero soft events") {
    auto opt = baseOptions({{2.0, 0.15, 1}});  // the 2 Hz worst case
    opt.anchor = p1;
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([p1](int i, double) { return p1[i]; });
    CHECK_FALSE(arm::run(robot, run, 90.0, &run));
    REQUIRE(run.finishedCleanly());
    CHECK(run.outcome() == x7::IdentRun::Outcome::Completed);
    const auto& r = run.results()[0];
    CHECK(r.completed);
    CHECK(r.soft_events == 0);  // the pre-fix demod tripped the cap here
    CHECK(r.resp[4].abs() < 1e-6);  // the offset joint reads still
  }

  SECTION("slow drift on the probe joint does not leak into the demod") {
    auto opt = baseOptions({{5.0, 0.15, 1}});
    opt.anchor = p1;
    x7::IdentRun run(fx.chain, fx.map, opt, nullptr);
    ScriptArm robot([p1](int i, double t) {
      return p1[i] + (i == 1 ? 0.0005 * t : 0.0);  // 0.5 mrad/s drift
    });
    CHECK_FALSE(arm::run(robot, run, 90.0, &run));
    REQUIRE(run.finishedCleanly());
    const auto& r = run.results()[0];
    CHECK(r.completed);
    CHECK(r.soft_events == 0);
    CHECK(r.resp[1].abs() < 1e-5);  // drift absorbed, not demodulated
  }
}

TEST_CASE("ident frequency list parsing is strict", "[ident]") {
  std::vector<x7::FreqSpec> out;
  CHECK(x7::parseFreqList("4.5,5.0@0.075", &out));
  REQUIRE(out.size() == 2);
  CHECK(out[0].freq_hz == Approx(4.5));
  CHECK(out[0].amp_override_nm == 0.0);
  CHECK(out[1].amp_override_nm == Approx(0.075));
  // an unreplaced campaign placeholder must refuse, not truncate
  CHECK_FALSE(x7::parseFreqList("4.5,<peak>@<half-amp>", &out));
  CHECK_FALSE(x7::parseFreqList("4.5,", &out));
  CHECK_FALSE(x7::parseFreqList("", &out));
  CHECK_FALSE(x7::parseFreqList("4.5x", &out));
  CHECK_FALSE(x7::parseFreqList("0.1", &out));   // below 0.5 Hz
  // the block estimator needs >= 5 samples/period at 100 Hz: anything
  // above 20 Hz would stall the measurement window, so it is refused
  CHECK_FALSE(x7::parseFreqList("25", &out));
  CHECK_FALSE(x7::parseFreqList("45", &out));
  CHECK(x7::parseFreqList("20", &out));  // the survey-grid ceiling
  CHECK_FALSE(x7::parseFreqList("nan", &out));
  CHECK_FALSE(x7::parseFreqList("4.5@0", &out));
  CHECK_FALSE(x7::parseFreqList("4.5@-1", &out));
  CHECK_FALSE(x7::parseFreqList("4.5@nan", &out));
}

TEST_CASE("ident amplitude cap parsing is strict and the hard cap is "
          "unbypassable",
          "[ident]") {
  double a = 0.0;
  CHECK(x7::parseAmpCap("0.15", &a));
  CHECK(a == Approx(0.15));
  CHECK(x7::parseAmpCap("0.3", &a));
  CHECK(x7::parseAmpCap("0.05", &a));
  // atof-plus-clamp let "nan" schedule 3.8 Nm (review finding)
  CHECK_FALSE(x7::parseAmpCap("nan", &a));
  CHECK_FALSE(x7::parseAmpCap("inf", &a));
  CHECK_FALSE(x7::parseAmpCap("0.31", &a));   // above the hard cap
  CHECK_FALSE(x7::parseAmpCap("0.01", &a));   // below the floor
  CHECK_FALSE(x7::parseAmpCap("0.15x", &a));  // trailing garbage
  CHECK_FALSE(x7::parseAmpCap("", &a));
  // defense in depth: even a NaN cap reaching the amplitude rule must
  // degrade to the hard cap, never bypass it
  const double nan_cap = std::nan("");
  CHECK(x7::probeAmplitude(0.05, 20.0, nan_cap) <= x7::kAmpCapHardNm);
  CHECK(x7::probeAmplitude(0.05, 20.0, 1e9) <= x7::kAmpCapHardNm);
}
