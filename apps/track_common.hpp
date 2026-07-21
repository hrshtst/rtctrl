// Shared between x7_track (hardware) and x7_track_sim: the wrapped
// computed-torque controller with per-joint tracking statistics and
// optional per-cycle CSV logging.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7 {

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

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

inline std::FILE* openCsvLog(const std::string& path) {
  std::FILE* log = std::fopen(path.c_str(), "w");
  if (!log) return nullptr;
  std::fprintf(log, "leg,t");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log, ",qd%d,q%d,dqd%d,dq%d,tau%d", i, i, i, i, i);
  }
  std::fprintf(log, "\n");
  return log;
}

}  // namespace x7
