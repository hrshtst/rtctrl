// M8 hardware phase: computed-torque trajectory tracking on the real
// arm — tau = ID(q_d, dq_d, ddq_d) + Kp e + Kd de in current mode,
// a small minimum-jerk excursion from the present pose and back, at
// reduced speed.
//
// SAFETY: current mode. Power cutoff in reach, workspace clear.
//
// Gains: the sim acceptance runs Kp=20/Kd=2 (tracking_sim_test, RMS
// 0.0050 rad), but the real loop has ~2 control cycles of command
// pipeline latency (read -> controller -> next cycle's write) plus the
// servo's filtered, quantized velocity estimate — lag the ideal sim
// loop does not have. At Kp=20 the distal joints sit on the resulting
// stability margin and oscillate, so the hardware defaults are softer;
// the inverse-dynamics feedforward still carries the trajectory.
//
// Usage: x7_track [--config path] [--port dev]
//                 [--kp v] [--kd v] [--log out.csv] [scale]
//   scale in (0,1] shrinks the excursion (default 1.0 ≈ ±0.3 rad max)
//   --log writes t, q_d, q, dq_d, dq, tau per joint each cycle

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "x7_common.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

namespace {

// Wraps ComputedTorque, accumulates per-joint tracking error, and
// optionally logs every cycle to CSV.
struct TrackingRun : arm::Controller {
  TrackingRun(model::ChainModel& chain, const model::JointMap& map,
              const model::MinJerkTrajectory& trajectory, double kp,
              double kd, int leg, std::FILE* log)
      : inner(chain, map, trajectory, kp, kd),
        trajectory_(trajectory),
        leg_(leg),
        log_(log) {}

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    inner.update(state, cmd, t);
    trajectory_.sample(t, q_d.get(), dq_d.get());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double e = q_d[i] - zVecElemNC(state.q.get(), i);
      sum_sq[i] += e * e;
      max_abs[i] = std::max(max_abs[i], std::fabs(e));
    }
    ++samples;
    if (log_) {
      std::fprintf(log_, "%d,%.4f", leg_, t);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::fprintf(log_, ",%.5f,%.5f,%.5f,%.5f,%.4f", q_d[i],
                     zVecElemNC(state.q.get(), i), dq_d[i],
                     zVecElemNC(state.dq.get(), i),
                     zVecElemNC(cmd.tau.get(), i));
      }
      std::fprintf(log_, "\n");
    }
  }

  double rms() const {
    if (!samples) return 0.0;
    double total = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) total += sum_sq[i];
    return std::sqrt(total / (samples * model::kCanonicalDof));
  }
  double rms(int i) const {
    return samples ? std::sqrt(sum_sq[i] / samples) : 0.0;
  }

  void report() const {
    std::printf("%s RMS: %.4f rad\n", leg_ == 0 ? "out " : "back", rms());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (max_abs[i] < 1e-4) continue;  // joint not commanded to move
      std::printf("  [%d] %-22s RMS %.4f  max %.4f rad\n", i,
                  model::canonicalJoints()[i].urdf_joint, rms(i),
                  max_abs[i]);
    }
  }

  arm::ComputedTorque inner;
  const model::MinJerkTrajectory& trajectory_;
  int leg_;
  std::FILE* log_;
  model::ZVector q_d{model::kCanonicalDof};
  model::ZVector dq_d{model::kCanonicalDof};
  double sum_sq[model::kCanonicalDof] = {};
  double max_abs[model::kCanonicalDof] = {};
  long samples = 0;
};

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  double scale = 1.0;
  // Hardware defaults, softer than the sim-verified 20/2 — see header.
  double kp = 8.0, kd = 1.2;
  std::string log_path;
  for (int i = cli.argi; i < argc; ++i) {
    if (std::strcmp(argv[i], "--kp") == 0 && i + 1 < argc) {
      kp = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--kd") == 0 && i + 1 < argc) {
      kd = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
      log_path = argv[++i];
    } else {
      scale = std::atof(argv[i]);
    }
  }
  scale = std::clamp(scale, 0.05, 1.0);
  kp = std::clamp(kp, 0.0, 50.0);
  kd = std::clamp(kd, 0.0, 5.0);

  std::FILE* log = nullptr;
  if (!log_path.empty()) {
    log = std::fopen(log_path.c_str(), "w");
    if (!log) {
      std::fprintf(stderr, "cannot open log file %s\n", log_path.c_str());
      return 1;
    }
    std::fprintf(log, "leg,t");
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(log, ",qd%d,q%d,dqd%d,dq%d,tau%d", i, i, i, i, i);
    }
    std::fprintf(log, "\n");
  }

  try {
    auto session = x7::openSession(cli, /*operating_mode_override=*/0);
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);
    arm::RealArm robot(*session.arm);

    if (!robot.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   session.arm->lastError().c_str());
      return 1;
    }
    arm::JointState start;
    if (!robot.readState(start)) return 1;

    model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
    zVecCopyNC(start.q.get(), q0.get());
    zVecCopyNC(start.q.get(), qf.get());
    // gentle excursion on tilt / elbow / wrist pitch
    qf[1] += 0.20 * scale;
    qf[3] -= 0.30 * scale;
    qf[5] += 0.25 * scale;

    constexpr double kVel = 0.3;  // rad/s — reduced speed
    const auto out = model::MinJerkTrajectory::withVelocityLimit(
        q0, qf, kVel, 2.0);
    const auto back = model::MinJerkTrajectory::withVelocityLimit(
        qf, q0, kVel, 2.0);

    std::printf("computed-torque tracking: %.1f s out, %.1f s back "
                "(scale %.2f, Kp %.1f, Kd %.2f)\n",
                out.duration(), back.duration(), scale, kp, kd);

    TrackingRun leg1(chain, map, out, kp, kd, 0, log);
    bool ok = arm::run(robot, leg1, out.duration());
    leg1.report();

    if (ok) {
      TrackingRun leg2(chain, map, back, kp, kd, 1, log);
      ok = arm::run(robot, leg2, back.duration());
      leg2.report();
    }

    robot.deactivate();
    if (log) std::fclose(log);
    std::printf("%s\n", ok ? "done" : "ABORTED");
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    if (log) std::fclose(log);
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
