// Sim twin of x7_track: the identical computed-torque excursion run on
// SimArm through the bridge — preview a hardware session here first.
//
// By default the loop also models the two lags the real loop has and
// the ideal sim loop lacks: one control cycle of command pipeline
// latency, and a servo-style velocity estimate (first-order lag +
// one-LSB quantization) feeding the Kd term. Sweeping the estimator
// lag against the 2026-07-21 hardware run showed the pipeline latency
// alone is harmless; the velocity lag is what destabilizes — at
// --vel-tau 0.12 the old gains (--kp 20 --kd 2) reproduce the observed
// oscillation (RMS 0.067 vs 0.065-0.070 measured), so 0.12 s is the
// default. --ideal strips the lag model back to the pristine loop of
// tracking_sim_test.
//
// Usage: x7_track_sim [--kp v] [--kd v] [--vel-tau s] [--log out.csv]
//                     [--zvs out.zvs] [--start q0,..,q7] [--ideal] [scale]
//   scale in (0,1] shrinks the excursion (default 1.0 ≈ ±0.3 rad max)
//   --start replays a hardware pose (e.g. the first q row of an
//     x7_track --log CSV) instead of the default test pose
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

  bool readState(arm::JointState& state) override {
    if (!inner_.readState(state)) return false;
    const double alpha = dt() / (vel_tau_ + dt());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      dq_filt_[i] += alpha * (zVecElemNC(state.dq.get(), i) - dq_filt_[i]);
      zVecElemNC(state.dq.get(), i) =
          std::round(dq_filt_[i] / kVelLsb) * kVelLsb;
      zVecElemNC(state.q.get(), i) =
          std::round(zVecElemNC(state.q.get(), i) / kPosLsb) * kPosLsb;
    }
    return true;
  }

  bool writeCommand(const arm::JointCommand& cmd) override {
    if (!primed_) {  // first cycle passes through, as activation snaps
      primed_ = true;
      copy(cmd, pending_);
      return inner_.writeCommand(cmd);
    }
    const bool ok = inner_.writeCommand(pending_);
    copy(cmd, pending_);
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

  arm::SimArm& inner_;
  double vel_tau_;
  arm::JointCommand pending_;
  bool primed_ = false;
  double dq_filt_[model::kCanonicalDof] = {};
};

}  // namespace

int main(int argc, char* argv[]) {
  double scale = 1.0;
  double kp = 12.0, kd = 1.6;  // x7_track's hardware defaults
  double vel_tau = 0.12;      // servo velocity-estimator lag [s]
  bool ideal = false;
  std::string log_path, zvs_path, start_csv;
  std::vector<int> disturb_joint;
  std::vector<double> disturb_tau;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--kp") == 0 && i + 1 < argc) {
      kp = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--kd") == 0 && i + 1 < argc) {
      kd = std::atof(argv[++i]);
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

    model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      q0[i] = opt.initial_q8[i];
      qf[i] = opt.initial_q8[i];
    }
    // gentle excursion on tilt / elbow / wrist pitch — as x7_track
    qf[1] += 0.20 * scale;
    qf[3] -= 0.30 * scale;
    qf[5] += 0.25 * scale;

    constexpr double kVel = 0.3;  // rad/s — reduced speed
    const auto out = model::MinJerkTrajectory::withVelocityLimit(
        q0, qf, kVel, 2.0);
    const auto back = model::MinJerkTrajectory::withVelocityLimit(
        qf, q0, kVel, 2.0);

    std::printf("computed-torque tracking [sim, %s loop]: %.1f s out, "
                "%.1f s back (scale %.2f, Kp %.1f, Kd %.2f)\n",
                ideal ? "ideal" : "lagged", out.duration(),
                back.duration(), scale, kp, kd);

    x7::TrackingRun leg1(chain, map, out, kp, kd, 0, log);
    bool ok = arm::run(robot, leg1, out.duration());
    leg1.report();

    if (ok) {
      x7::TrackingRun leg2(chain, map, back, kp, kd, 1, log);
      ok = arm::run(robot, leg2, back.duration());
      leg2.report();
    }

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
