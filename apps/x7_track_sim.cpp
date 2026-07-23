// Sim twin of x7_track: the identical computed-torque excursion run on
// SimArm through the bridge — preview a hardware session here first.
//
// By default the loop also models the lags and quantizations the real
// loop has and the ideal sim loop lacks: one control cycle of command
// pipeline latency, position readout quantized to the encoder LSB, and
// a servo-style velocity estimate (first-order lag, --vel-tau, default
// 0.12 s + one-LSB quantization) in state.dq. Historical note: with
// the PRE-hardening controller (which damped on state.dq directly),
// sweeping --vel-tau against the first hardware run (Kp20/Kd2)
// reproduced its oscillation at 0.12 s (sim RMS 0.067 vs 0.065-0.070
// measured) —
// that sweep is what identified the servo velocity estimate as
// unusable for damping. The shipped controller estimates velocity
// host-side, so state.dq no longer reaches the Kd term; the lag model
// now matters mainly through the quantized positions. --ideal strips
// the lag model entirely (note: the hardware countermeasures in the
// controller remain active here; tracking_sim_test additionally
// disables those to certify the pure math).
//
// A rigid-joint simulation cannot show the gear-elasticity modes that
// dominated the hardware campaign — use this app to preview
// trajectories and replay logged poses/disturbances, not to certify
// gains (docs/theory/computed-torque.md).
//
// Usage: x7_track_sim [--kp v] [--kd v] [--ki v] [--vel-tau s]
//                     [--log out.csv] [--zvs out.zvs]
//                     [--start q0,..,q7] [--disturb j tau] [--ideal]
//                     [scale]
//   scale in (0,1] shrinks the excursion (default 1.0; the sim permits
//     scales past x7_track's 0.6 hardware cap, for exploration)
//   --start replays a hardware pose (e.g. the first q row of an
//     x7_track --log CSV) instead of the default test pose
//   --disturb j tau adds a constant unmodeled torque [Nm] on canonical
//     joint j (repeatable) — stands in for stiction/model error
//   --zvs writes the motion for: rk_anim models/crane_x7/crane_x7.ztk <file>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rtctrl/arm/runner.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"
#include "track_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

namespace {

// Adds the real control loop's lag to an ideal SimArm: commands land
// one cycle late, and reported velocity is low-passed (the servo's
// internal estimator) then quantized to the Dynamixel LSB.
struct LaggedArm : arm::Arm {
  static constexpr double kVelLsb = 0.229 * 2.0 * M_PI / 60.0;  // [rad/s]
  static constexpr double kPosLsb = 2.0 * M_PI / 4096.0;        // [rad]

  LaggedArm(arm::SimArm& inner, double vel_tau)
      : inner_(inner), vel_tau_(vel_tau) {}

  int dof() const override { return inner_.dof(); }
  double dt() const override { return inner_.dt(); }
  bool activate() override { return inner_.activate(); }
  bool deactivate() override { return inner_.deactivate(); }
  bool setMode(arm::ControlMode mode) override {
    return inner_.setMode(mode);
  }

  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* cmds = nullptr) override {
    if (!inner_.readState(state)) return false;
    const double alpha = dt() / (vel_tau_ + dt());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      dq_filt_[i] += alpha * (zVecElemNC(state.dq.get(), i) - dq_filt_[i]);
      zVecElemNC(state.dq.get(), i) =
          std::round(dq_filt_[i] / kVelLsb) * kVelLsb;
      zVecElemNC(state.q.get(), i) =
          std::round(zVecElemNC(state.q.get(), i) / kPosLsb) * kPosLsb;
    }
    // The wrapper owns the command records: its lag makes application
    // ASYNCHRONOUS, so the inner sim's synchronous records would
    // misattribute the current request to the previous command's
    // sequence.
    if (cmds != nullptr) {
      cmds->applied = applied_rec_;
      cmds->last_attempt = attempt_rec_;
    }
    return true;
  }

  bool writeCommand(const arm::JointCommand& cmd,
                    arm::CommandReceipt* receipt = nullptr) override {
    const std::uint64_t this_seq = ++wrapper_seq_;
    const double now = inner_.time();
    bool ok = true;
    if (!primed_) {  // first cycle passes through, as activation snaps
      primed_ = true;
      ok = inner_.writeCommand(cmd);
      if (ok) adoptInnerRecords(this_seq, now);
    } else {
      // the PENDING command (accepted one cycle ago) reaches the
      // actuator now — ITS sequence becomes the applied sequence
      ok = inner_.writeCommand(pending_);
      if (ok) adoptInnerRecords(pending_seq_, now);
    }
    copy(cmd, pending_);
    pending_seq_ = this_seq;
    if (receipt != nullptr) *receipt = {ok, this_seq, now};
    return ok;
  }

  bool step() override { return inner_.step(); }

 private:
  static void copy(const arm::JointCommand& from, arm::JointCommand& to) {
    to.mode = from.mode;
    zVecCopyNC(from.q.get(), to.q.get());
    zVecCopyNC(from.dq.get(), to.dq.get());
    zVecCopyNC(from.tau.get(), to.tau.get());
  }

  // Take the inner sim's freshly synthesized records (they carry the
  // clamp truth) but stamp them with the WRAPPER's sequence.
  void adoptInnerRecords(std::uint64_t wrapper_seq, double now) {
    arm::JointState dummy;
    arm::CommandSnapshot snap;
    inner_.readState(dummy, &snap);
    applied_rec_ = snap.applied;
    applied_rec_.target_seq = wrapper_seq;
    applied_rec_.first_time = applied_rec_.latest_time = now;
    attempt_rec_ = snap.last_attempt;
    attempt_rec_.target_seq = wrapper_seq;
    attempt_rec_.time = now;
  }

  arm::SimArm& inner_;
  double vel_tau_;
  arm::JointCommand pending_;
  std::uint64_t pending_seq_ = 0;
  std::uint64_t wrapper_seq_ = 0;
  bool primed_ = false;
  arm::AppliedTargetRecord applied_rec_;
  arm::WriteAttemptRecord attempt_rec_;
  double dq_filt_[model::kCanonicalDof] = {};
};

}  // namespace

int main(int argc, char* argv[]) {
  double scale = 1.0;
  double kp = x7::tuning::kKp, kd = x7::tuning::kKd,
         ki = x7::tuning::kKi;  // the shipped library tuning
  double vel_tau = 0.12;                // servo velocity-estimator lag [s]
  bool ideal = false;
  std::string log_path, zvs_path, start_csv;
  std::vector<int> disturb_joint;
  std::vector<double> disturb_tau;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--kp") == 0 && i + 1 < argc) {
      kp = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--kd") == 0 && i + 1 < argc) {
      kd = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--ki") == 0 && i + 1 < argc) {
      ki = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--vel-tau") == 0 && i + 1 < argc) {
      vel_tau = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
      log_path = argv[++i];
    } else if (std::strcmp(argv[i], "--zvs") == 0 && i + 1 < argc) {
      zvs_path = argv[++i];
    } else if (std::strcmp(argv[i], "--start") == 0 && i + 1 < argc) {
      start_csv = argv[++i];
    } else if (std::strcmp(argv[i], "--disturb") == 0 && i + 2 < argc) {
      // constant unmodeled torque on a canonical joint: --disturb j tau
      // (stands in for stiction / model error, the seed the ideal-model
      // sim otherwise lacks)
      disturb_joint.push_back(std::atoi(argv[++i]));
      disturb_tau.push_back(std::atof(argv[++i]));
    } else if (std::strcmp(argv[i], "--ideal") == 0) {
      ideal = true;
    } else {
      scale = std::atof(argv[i]);
    }
  }
  scale = std::clamp(scale, 0.05, 1.0);
  kp = std::clamp(kp, 0.0, 50.0);
  kd = std::clamp(kd, 0.0, 5.0);
  ki = std::clamp(ki, 0.0, 20.0);

  std::FILE* log = nullptr;
  if (!log_path.empty()) {
    log = x7::openCsvLog(log_path);
    if (!log) {
      std::fprintf(stderr, "cannot open log file %s\n", log_path.c_str());
      return 1;
    }
  }

  try {
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);

    // The gravity-loaded start pose of the sim acceptance test; the
    // hardware app starts from wherever the arm is (--start replays one).
    arm::SimArm::Options opt;
    opt.initial_q8 = {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0, 0.1};
    if (!start_csv.empty()) {
      opt.initial_q8.clear();
      const char* p = start_csv.c_str();
      char* end = nullptr;
      for (double v = std::strtod(p, &end); p != end;
           v = std::strtod(p, &end)) {
        opt.initial_q8.push_back(v);
        p = *end == ',' ? end + 1 : end;
      }
      if (opt.initial_q8.size() != model::kCanonicalDof) {
        std::fprintf(stderr, "--start needs %d comma-separated values\n",
                     model::kCanonicalDof);
        return 1;
      }
    }
    arm::SimArm sim(opt);
    sim.setMode(arm::ControlMode::Current);

    std::unique_ptr<model::ZvsWriter> zvs;
    if (!zvs_path.empty()) {
      zvs = std::make_unique<model::ZvsWriter>(zvs_path);
      sim.logTo(zvs.get());
    }

    for (std::size_t i = 0; i < disturb_joint.size(); ++i) {
      sim.setDisturbance(disturb_joint[i], disturb_tau[i]);
    }

    LaggedArm lagged(sim, vel_tau);
    arm::Arm& robot = ideal ? static_cast<arm::Arm&>(sim) : lagged;
    robot.activate();

    // same settle phase as x7_track, then anchor where the arm is
    // (with --disturb seeds it settles slightly off the start pose)
    x7::SettleController settle_ctl(chain, map, x7::tuning::kSettleKd);
    x7::settleArm(robot, settle_ctl, 6.0);
    arm::JointState settled;
    robot.readState(settled);

    model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      q0[i] = zVecElemNC(settled.q.get(), i);
      qf[i] = q0[i];
    }
    // gentle excursion on tilt / elbow / wrist pitch — as x7_track
    qf[1] += 0.20 * scale;
    qf[3] -= 0.30 * scale;
    qf[5] += 0.25 * scale;

    constexpr double kVel = 0.3;  // rad/s — reduced speed
    // duration rule as x7_track: hold peak accel at the proven level
    const double min_T = 2.0 * std::sqrt(std::max(scale, 0.5) / 0.5);
    // one continuous round trip, one controller — as x7_track
    const model::RoundTripTrajectory trip(
        model::MinJerkTrajectory::withVelocityLimit(q0, qf, kVel, min_T),
        model::MinJerkTrajectory::withVelocityLimit(qf, q0, kVel, min_T));

    std::printf("computed-torque tracking [sim, %s loop]: %.1f s out, "
                "%.1f s back (scale %.2f, Kp %.1f, Kd %.2f, Ki %.1f)\n",
                ideal ? "ideal" : "lagged", trip.outDuration(),
                trip.duration() - trip.outDuration(), scale, kp, kd, ki);

    x7::TrackingRun tracking(chain, map, trip, kp, kd,
                             trip.outDuration(), log);
    tracking.inner.setIntegral(ki, x7::tuning::kIntegralClampNm);
    tracking.inner.setGainScales(x7::kGainScale);
    tracking.inner.setNominalDt(x7::tuning::kNominalDt);
    {
      // sim-side torque-limit truth: SimArm's effort clamp
      double tau_max[model::kCanonicalDof];
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        tau_max[i] = opt.effort_limit8[i];
      }
      tracking.inner.setTorqueLimits(tau_max);
    }

    const bool ok = arm::run(robot, tracking, trip.duration(), &tracking);
    tracking.report();

    robot.deactivate();
    if (log) std::fclose(log);
    if (zvs) {
      std::printf("wrote %d frames — view with:  rk_anim "
                  "models/crane_x7/crane_x7.ztk %s\n",
                  zvs->frames(), zvs_path.c_str());
    }
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    if (log) std::fclose(log);
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
