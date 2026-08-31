// Offline exact-feedback-linearization study runner — executes the
// COMPLETE preregistered study of docs/records/history.md (EFL study) with
// the frozen constants of docs/records/history.md (EFL frozen specification).
// Simulation only: no hardware, no x7_track involvement.
//
// Usage: x7_efl_study [--out dir] [--case ID]
//   --out   output directory (default build/efl_study). Holds
//           results.json plus one per-cycle CSV per run.
//   --case  run a single held-out case (smoke testing). --case output
//           is ALWAYS noncanonical; only the complete unfiltered
//           matrix from a clean worktree at the built HEAD may be
//           promoted to data/efl_study/results.json.
//   --zvs   additionally write one .zvs motion file per SimArm-based
//           run (the grid and the rigid held-out cells), playable
//           with: rk_anim models/crane_x7/crane_x7.ztk <file>.
//           Passive logging — no effect on any metric or on
//           canonicality. The TwoMassArm screens are not a roki
//           chain and produce none.
//
// There are deliberately NO tuning knobs: every number is frozen in
// the plan documents and pinned here as constants.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "study/exact_feedback_linearization.hpp"
#include "common/lagged_arm.hpp"
#include "rtctrl/arm/practical_computed_torque.hpp"
#include "rtctrl/arm/crane_x7_tuning.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/posture.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"
#include "study/study_metrics.hpp"
#include "common/two_mass_arm.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
namespace study = x7::study;
namespace tuning = rtctrl::arm::tuning;
using model::kCanonicalDof;

// Build-time provenance (cmake/GitRev.cmake, regenerated every build).
namespace x7::gitrev {
extern const char* const kBuildSha;
extern const bool kBuildDirty;
}  // namespace x7::gitrev

namespace {

// ------------------------- preregistered constants (frozen; see plan)

constexpr const char* kSchemaVersion = "2";
constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr const char* kP1Path = "config/postures/p1.toml";

// Torque limits [Nm]: max(0, min(effort, kt*I_servo) - margin*kt) with
// the frozen ASSUMED servo current limits below (joint 0 is
// current-limit bound: 1.783 * 1193*0.00269 A = 5.7220 Nm < 10 Nm).
constexpr long kAssumedCurrentLimitRaw[kCanonicalDof] = {
    1193, 2047, 1193, 1193, 1193, 1193, 1193, 1193};
constexpr double kTauMax[kCanonicalDof] = {
    4.8305, 8.7955, 3.1085, 3.1085, 3.1085, 3.1085, 3.1085, 3.1085};

constexpr double kDevPose[kCanonicalDof] = {0.0,  0.2, 0.0, -0.4,
                                            0.0, -0.2, 0.0, 0.1};
constexpr double kIncidentPose[kCanonicalDof] = {
    -0.10738, 0.50928, 0.19021, -1.87299,
    0.02608, -0.79153, -0.05983, -0.01841};

constexpr double kScale = 0.5;
constexpr double kOmegaGrid[] = {4.0, 6.0, 8.0};  // [rad/s]
constexpr double kZetaGrid[] = {0.7, 1.0};

constexpr int kDelayCycles = 2;       // zero-preloaded queue
constexpr double kVelTau = 0.12;      // servo-style dq lag (unused by
                                      // the host controllers)
constexpr double kDisturbNm = 0.8;    // D+/D− on canonical joint 1
constexpr double kHoldS = 3.0;        // D hold beyond the trajectory
constexpr double kBandRad = 0.01;     // settling band
constexpr double kDwellS = 0.5;       // settling dwell
constexpr double kSsWindowS = 1.0;    // steady-state window

constexpr double kDeflectionRad = 0.005;  // motor-side gear deflection
constexpr double kFlexDurationS = 3.0;
constexpr double kFloorEps = 1e-9;  // [rad] modal censoring floor
// modal windows: A_early [0,0.5), A1 [0.5,1.5), A2 [1.5,2.5)
constexpr double kEarly0 = 0.0, kEarly1 = 0.5;
constexpr double kWin1a = 0.5, kWin1b = 1.5;
constexpr double kWin2a = 1.5, kWin2b = 2.5;

// ------------------------------------------------------------ pieces

// The x7_track excursion and duration rule, mirrored (provenance:
// apps/track/x7_track.cpp — tilt/elbow/wrist-pitch excursion, peak accel
// held at the half-scale-proven level so T grows with
// sqrt(amplitude)). The hardware app additionally clamps against the
// servo soft-limit band, which does not exist offline.
model::RoundTripTrajectory makeTrip(const double* start) {
  model::ZVector q0(kCanonicalDof), qf(kCanonicalDof);
  for (int i = 0; i < kCanonicalDof; ++i) {
    q0[i] = start[i];
    qf[i] = start[i];
  }
  qf[1] += 0.20 * kScale;
  qf[3] -= 0.30 * kScale;
  qf[5] += 0.25 * kScale;
  constexpr double kVel = 0.3;  // rad/s
  const double min_t = 2.0 * std::sqrt(std::max(kScale, 0.5) / 0.5);
  return model::RoundTripTrajectory(
      model::MinJerkTrajectory::withVelocityLimit(q0, qf, kVel, min_t),
      model::MinJerkTrajectory::withVelocityLimit(qf, q0, kVel, min_t));
}

struct RunResult {
  bool completed = false;
  bool finite = false;
  std::string fail_reason;
  double rms_total = 0.0;
  double peak_err = 0.0;
  double rms_joint[kCanonicalDof] = {};
  double peak_err_joint[kCanonicalDof] = {};
  double rms_tau = 0.0;
  double peak_tau = 0.0;
  long sat_count = 0;
  bool has_settling = false;
  study::Settling settling;
  bool has_modal = false;
  study::ModalResult modal;
};

// One synchronous run: readState -> update -> writeCommand -> step,
// scoring against the reference (which clamps at its endpoints, so a
// post-trajectory hold is simply running past duration()).
template <typename Controller, typename SatFn>
RunResult runCase(arm::Arm& robot, Controller& ctl,
                  const model::Trajectory& traj, double duration,
                  SatFn cycle_sat, double settle_t_end, int score_joint,
                  double modal_f_hz, int probe_joint,
                  const std::string& trace_path) {
  RunResult r;
  std::FILE* trace = std::fopen(trace_path.c_str(), "w");
  if (trace != nullptr) {
    std::fprintf(trace, "t");
    for (int i = 0; i < kCanonicalDof; ++i) std::fprintf(trace, ",e%d", i);
    for (int i = 0; i < kCanonicalDof; ++i) {
      std::fprintf(trace, ",tau%d", i);
    }
    std::fprintf(trace, "\n");
  }
  robot.setMode(arm::ControlMode::Current);
  robot.activate();

  model::ZVector q_d(kCanonicalDof);
  arm::JointState state;
  arm::JointCommand cmd;
  std::vector<double> ts, e_scored, probe_q;
  double sum_sq = 0.0;
  double sum_sq_j[kCanonicalDof] = {};
  double sum_sq_tau = 0.0;
  long n_samples = 0;
  bool finite = true;
  bool io_ok = true;
  for (double t = 0.0; t < duration; t += robot.dt()) {
    if (!robot.readState(state)) {
      io_ok = false;
      break;
    }
    ctl.update(state, cmd, t);
    if (!robot.writeCommand(cmd)) {
      io_ok = false;
      break;
    }
    if (!robot.step()) {
      io_ok = false;
      break;
    }
    traj.sample(t, q_d.get());
    if (trace != nullptr) std::fprintf(trace, "%.4f", t);
    for (int i = 0; i < kCanonicalDof; ++i) {
      const double q = zVecElemNC(state.q.get(), i);
      const double tau = zVecElemNC(cmd.tau.get(), i);
      if (!std::isfinite(q) || !std::isfinite(tau)) finite = false;
      const double e = q_d[i] - q;
      sum_sq += e * e;
      sum_sq_j[i] += e * e;
      r.peak_err = std::max(r.peak_err, std::abs(e));
      r.peak_err_joint[i] = std::max(r.peak_err_joint[i], std::abs(e));
      sum_sq_tau += tau * tau;
      r.peak_tau = std::max(r.peak_tau, std::abs(tau));
      if (trace != nullptr) std::fprintf(trace, ",%.6e", e);
    }
    if (trace != nullptr) {
      for (int i = 0; i < kCanonicalDof; ++i) {
        std::fprintf(trace, ",%.6e", zVecElemNC(cmd.tau.get(), i));
      }
      std::fprintf(trace, "\n");
    }
    r.sat_count += cycle_sat();
    ts.push_back(t);
    e_scored.push_back(q_d[score_joint] -
                       zVecElemNC(state.q.get(), score_joint));
    probe_q.push_back(zVecElemNC(state.q.get(), probe_joint));
    ++n_samples;
    if (!finite) break;
  }
  if (trace != nullptr) std::fclose(trace);
  robot.deactivate();

  r.completed = io_ok && finite;
  r.finite = finite;
  if (!io_ok) r.fail_reason = "arm I/O failure";
  if (!finite) r.fail_reason = "non-finite state or command";
  if (n_samples == 0) return r;
  const double n_all = static_cast<double>(n_samples * kCanonicalDof);
  r.rms_total = std::sqrt(sum_sq / n_all);
  r.rms_tau = std::sqrt(sum_sq_tau / n_all);
  for (int i = 0; i < kCanonicalDof; ++i) {
    r.rms_joint[i] =
        std::sqrt(sum_sq_j[i] / static_cast<double>(n_samples));
  }
  if (settle_t_end >= 0.0) {
    r.has_settling = true;
    r.settling = study::settlingMetrics(ts, e_scored, settle_t_end,
                                        kBandRad, kDwellS, kSsWindowS);
  }
  if (modal_f_hz > 0.0) {
    auto window = [&](double a, double b) {
      std::vector<double> wt, wy;
      for (std::size_t k = 0; k < ts.size(); ++k) {
        if (ts[k] >= a && ts[k] < b) {
          wt.push_back(ts[k]);
          wy.push_back(probe_q[k]);
        }
      }
      return study::sineFitAmplitude(wt, wy, modal_f_hz);
    };
    r.has_modal = true;
    r.modal = study::classifyModal(window(kEarly0, kEarly1),
                                   window(kWin1a, kWin1b),
                                   window(kWin2a, kWin2b), kFloorEps,
                                   kWin2a - kWin1a);
  }
  return r;
}

// -------------------------------------------------- controller kinds

enum class Kind { Practical, PracticalGf, DesiredHost, EflHost, EflIdeal };

const char* kindName(Kind k) {
  switch (k) {
    case Kind::Practical: return "PRACTICAL";
    case Kind::PracticalGf: return "PRACTICAL-GF";
    case Kind::DesiredHost: return "DESIRED-host";
    case Kind::EflHost: return "EFL-host";
    case Kind::EflIdeal: return "EFL-ideal";
  }
  return "?";
}

const char* velocitySource(Kind k) {
  return k == Kind::EflIdeal ? "exact-state" : "host-estimate";
}

// Runs one (case, controller) cell on a freshly constructed plant.
struct CellSpec {
  Kind kind;
  double kp_prime = 0.0;  // acceleration-domain gains (grid-selected)
  double kd_prime = 0.0;
  const model::Trajectory* traj = nullptr;
  double duration = 0.0;
  bool gravity_free = false;
  double settle_t_end = -1.0;
  int score_joint = 0;
  double modal_f_hz = 0.0;
  int probe_joint = 0;
};

RunResult runCell(model::ChainModel& chain, const model::JointMap& map,
                  arm::Arm& robot, const CellSpec& s,
                  const std::string& trace_path) {
  switch (s.kind) {
    case Kind::Practical: {
      arm::PracticalComputedTorque ctl(chain, map, *s.traj, tuning::kKp,
                              tuning::kKd);
      ctl.setIntegral(tuning::kKi, tuning::kIntegralClampNm);
      ctl.setGainScales(tuning::kGainScale);
      ctl.setNominalDt(tuning::kNominalDt);
      ctl.setTorqueLimits(kTauMax);
      auto sat = [&ctl] {
        long n = 0;
        for (int i = 0; i < kCanonicalDof; ++i) {
          if (ctl.controllerSaturated(i)) ++n;
        }
        return n;
      };
      return runCase(robot, ctl, *s.traj, s.duration, sat,
                     s.settle_t_end, s.score_joint, s.modal_f_hz,
                     s.probe_joint, trace_path);
    }
    case Kind::PracticalGf: {
      x7::PracticalReplica ctl(chain, map, *s.traj, tuning::kKp,
                               tuning::kKd, true);
      ctl.setIntegral(tuning::kKi, tuning::kIntegralClampNm);
      ctl.setGainScales(tuning::kGainScale);
      ctl.setNominalDt(tuning::kNominalDt);
      ctl.setTorqueLimits(kTauMax);
      auto sat = [&ctl] {
        long n = 0;
        for (int i = 0; i < kCanonicalDof; ++i) {
          if (ctl.controllerSaturated(i)) ++n;
        }
        return n;
      };
      return runCase(robot, ctl, *s.traj, s.duration, sat,
                     s.settle_t_end, s.score_joint, s.modal_f_hz,
                     s.probe_joint, trace_path);
    }
    default: {
      x7::AccelDomainOptions opt;
      opt.eval = s.kind == Kind::DesiredHost
                     ? x7::AccelDomainOptions::Eval::Desired
                     : x7::AccelDomainOptions::Eval::Measured;
      opt.state_velocity = s.kind == Kind::EflIdeal;
      opt.gravity_free = s.gravity_free;
      opt.nominal_dt = tuning::kNominalDt;
      opt.tau_max = kTauMax;
      x7::AccelDomainController ctl(chain, map, *s.traj, s.kp_prime,
                                    s.kd_prime, opt);
      auto sat = [&ctl] {
        long n = 0;
        for (int i = 0; i < kCanonicalDof; ++i) {
          if (ctl.controllerSaturated(i)) ++n;
        }
        return n;
      };
      return runCase(robot, ctl, *s.traj, s.duration, sat,
                     s.settle_t_end, s.score_joint, s.modal_f_hz,
                     s.probe_joint, trace_path);
    }
  }
}

// ------------------------------------------- plant configurations

// ONE source for both plant construction and result-table emission —
// the canonical table must carry the actual plant parameters, not
// descriptive strings (review finding).
arm::SimArm::Options rigidOptions(const double* pose) {
  arm::SimArm::Options o;
  o.model_path = kModelPath;
  o.initial_q8.assign(pose, pose + kCanonicalDof);
  return o;
}

// EXPLICIT single-mode fixture (review round 4): every joint stiff
// and well damped, ONLY the probe joint carries the planted mode.
x7::TwoMassArm::Options flexOptions(int probe, double f_hz,
                                    double zeta_mode, double j_l) {
  x7::TwoMassArm::Options o;
  for (auto& p : o.joints) {
    p = x7::TwoMassArm::plantMode(0.1, 0.05, 40.0, 0.5);
  }
  o.joints[probe] = x7::TwoMassArm::plantMode(j_l, 0.05, f_hz,
                                              zeta_mode);
  return o;
}

x7::TwoMassArm::Options flexOptionsFor(const std::string& case_id) {
  return case_id == "F4" ? flexOptions(0, 4.5, 0.03, 0.4)
                         : flexOptions(5, 13.0, 0.05, 0.01);
}

// ------------------------------------------------------ provenance

std::string runGit(const char* args) {
  std::string cmd = std::string("git ") + args + " 2>/dev/null";
  std::FILE* p = popen(cmd.c_str(), "r");
  if (p == nullptr) return "<git unavailable>";
  std::string out;
  char buf[512];
  std::size_t n;
  while ((n = std::fread(buf, 1, sizeof buf, p)) > 0) out.append(buf, n);
  pclose(p);
  while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) {
    out.pop_back();
  }
  return out;
}

}  // namespace

int main(int argc, char* argv[]) {
  std::string out_dir = "build/efl_study";
  std::string only_case;
  bool want_zvs = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc) {
      out_dir = argv[++i];
    } else if (std::strcmp(argv[i], "--case") == 0 && i + 1 < argc) {
      only_case = argv[++i];
    } else if (std::strcmp(argv[i], "--zvs") == 0) {
      want_zvs = true;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 1;
    }
  }
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::fprintf(stderr, "cannot create %s\n", out_dir.c_str());
    return 1;
  }

  // Canonicality checks BEFORE creating any output content: embedded
  // build SHA equals HEAD; worktree clean INCLUDING untracked files;
  // complete unfiltered invocation. (Cell completeness is re-checked
  // after the run.)
  std::vector<std::string> reasons;
  const std::string head = runGit("rev-parse HEAD");
  const std::string status =
      runGit("status --porcelain=v1 --untracked-files=normal");
  if (head != x7::gitrev::kBuildSha) {
    reasons.push_back("built SHA does not match current HEAD");
  }
  if (x7::gitrev::kBuildDirty) {
    reasons.push_back("worktree was dirty at build time");
  }
  if (!status.empty()) {
    reasons.push_back("worktree is dirty (tracked or untracked files)");
  }
  if (!only_case.empty()) {
    reasons.push_back("--case filter: not a complete run");
  }

  try {
    model::ChainModel chain(kModelPath);
    model::JointMap map(chain);

    double p1[kCanonicalDof];
    try {
      const auto posture = model::loadPostureToml(kP1Path);
      std::copy(posture.joint_positions.begin(),
                posture.joint_positions.end(), p1);
    } catch (const std::exception& error) {
      std::fprintf(stderr, "cannot load %s: %s\n", kP1Path, error.what());
      return 1;
    }

    // ---------------- gain selection on the development scenario ----
    const auto dev_trip = makeTrip(kDevPose);
    struct GridRow {
      double omega, zeta;
      Kind kind;
      RunResult r;
    };
    std::vector<GridRow> grid;
    auto rigid_arm = [&](const double* pose) {
      return std::make_unique<arm::SimArm>(rigidOptions(pose));
    };
    // --zvs: one motion file per SimArm run, named after its trace CSV.
    auto zvs_for = [](std::string csv) {
      csv.replace(csv.size() - 4, 4, ".zvs");
      return csv;
    };
    int zvs_files = 0;
    // practical baseline on the identical scenario (the discard rule
    // compares against its peak error)
    CellSpec base;
    base.kind = Kind::Practical;
    base.traj = &dev_trip;
    base.duration = dev_trip.duration();
    auto base_arm = rigid_arm(kDevPose);
    std::unique_ptr<model::ZvsWriter> base_zvs;
    if (want_zvs) {
      base_zvs = std::make_unique<model::ZvsWriter>(
          zvs_for(out_dir + "/grid_practical.csv"));
      base_arm->logTo(base_zvs.get());
      ++zvs_files;
    }
    const auto base_r = runCell(chain, map, *base_arm, base,
                                out_dir + "/grid_practical.csv");
    for (const double omega : kOmegaGrid) {
      for (const double zeta : kZetaGrid) {
        for (const Kind kind : {Kind::DesiredHost, Kind::EflHost}) {
          CellSpec s;
          s.kind = kind;
          s.kp_prime = omega * omega;
          s.kd_prime = 2.0 * zeta * omega;
          s.traj = &dev_trip;
          s.duration = dev_trip.duration();
          auto plant = rigid_arm(kDevPose);
          char name[128];
          std::snprintf(name, sizeof name, "%s/grid_%s_w%g_z%g.csv",
                        out_dir.c_str(), kindName(kind), omega, zeta);
          std::unique_ptr<model::ZvsWriter> zvs;
          if (want_zvs) {
            zvs = std::make_unique<model::ZvsWriter>(zvs_for(name));
            plant->logTo(zvs.get());
            ++zvs_files;
          }
          grid.push_back({omega, zeta, kind,
                          runCell(chain, map, *plant, s, name)});
        }
      }
    }
    // Selection (EFL-host survivors): discard incomplete/non-finite,
    // any saturation, or peak error above the practical baseline's;
    // among survivors within 5% of the lowest RMS pick the lowest
    // omega, then the HIGHER zeta.
    double best_rms = -1.0;
    for (const auto& g : grid) {
      if (g.kind != Kind::EflHost) continue;
      if (!g.r.completed || g.r.sat_count > 0 ||
          g.r.peak_err > base_r.peak_err) {
        continue;
      }
      if (best_rms < 0.0 || g.r.rms_total < best_rms) {
        best_rms = g.r.rms_total;
      }
    }
    bool have_gains = false;
    double sel_omega = 0.0, sel_zeta = 0.0;
    for (const auto& g : grid) {
      if (g.kind != Kind::EflHost) continue;
      if (!g.r.completed || g.r.sat_count > 0 ||
          g.r.peak_err > base_r.peak_err) {
        continue;
      }
      if (best_rms < 0.0 || g.r.rms_total > 1.05 * best_rms) continue;
      if (!have_gains || g.omega < sel_omega ||
          (g.omega == sel_omega && g.zeta > sel_zeta)) {
        have_gains = true;
        sel_omega = g.omega;
        sel_zeta = g.zeta;
      }
    }
    const double kp_sel = sel_omega * sel_omega;
    const double kd_sel = 2.0 * sel_zeta * sel_omega;
    // No survivor is a STUDY RESULT (recorded per cell below), not a
    // provenance defect — it must not mark the run noncanonical.

    // ------------------------------------------ held-out case matrix
    const auto p1_trip = makeTrip(p1);
    const auto inc_trip = makeTrip(kIncidentPose);
    model::ZVector zero_pose(kCanonicalDof);
    const x7::ConstantTrajectory zero_ref(zero_pose);

    struct CaseDef {
      const char* id;
      bool in_s;
      bool delayed;   // member of the C3 completion set
      bool flexible;  // F4/F13
    };
    const std::vector<CaseDef> case_defs = {
        {"R1", true, false, false}, {"R2", true, false, false},
        {"L1", true, true, false},  {"L2", true, true, false},
        {"D+", true, true, false},  {"D-", true, true, false},
        {"F4", false, true, true},  {"F13", false, true, true}};

    struct CellResult {
      std::string case_id;
      Kind kind;
      RunResult r;
    };
    std::vector<CellResult> cells;

    auto flex_arm = [&](const std::string& case_id) {
      return std::make_unique<x7::TwoMassArm>(flexOptionsFor(case_id));
    };

    for (const auto& cd : case_defs) {
      if (!only_case.empty() && only_case != cd.id) continue;
      std::vector<Kind> kinds;
      if (cd.flexible) {
        kinds = {Kind::PracticalGf, Kind::DesiredHost, Kind::EflHost};
      } else if (std::strcmp(cd.id, "R1") == 0 ||
                 std::strcmp(cd.id, "R2") == 0 ||
                 std::strcmp(cd.id, "L1") == 0 ||
                 std::strcmp(cd.id, "L2") == 0) {
        kinds = {Kind::Practical, Kind::DesiredHost, Kind::EflHost,
                 Kind::EflIdeal};
      } else {
        kinds = {Kind::Practical, Kind::DesiredHost, Kind::EflHost};
      }
      for (const Kind kind : kinds) {
        const bool needs_grid_gains =
            kind != Kind::Practical && kind != Kind::PracticalGf;
        if (needs_grid_gains && !have_gains) {
          // Explicit, reproducible failure for every dependent cell:
          // the preregistered selection produced no gains, so these
          // runs cannot be executed as preregistered.
          CellResult cr;
          cr.case_id = cd.id;
          cr.kind = kind;
          cr.r.fail_reason =
              "not run: no surviving gain candidate on the "
              "development grid";
          cells.push_back(cr);
          std::printf("case %-3s %-12s NOT RUN (no surviving gain "
                      "candidate)\n",
                      cd.id, kindName(kind));
          continue;
        }
        CellSpec s;
        s.kind = kind;
        s.kp_prime = kp_sel;
        s.kd_prime = kd_sel;
        std::unique_ptr<arm::Arm> plant;
        std::unique_ptr<x7::LaggedArm> wrapper;
        std::unique_ptr<arm::SimArm> sim;
        std::unique_ptr<x7::TwoMassArm> tma;
        if (cd.flexible) {
          const bool f4 = std::strcmp(cd.id, "F4") == 0;
          s.traj = &zero_ref;
          s.duration = kFlexDurationS;
          s.gravity_free = true;
          s.modal_f_hz = f4 ? 4.5 : 13.0;
          s.probe_joint = f4 ? 0 : 5;
          s.score_joint = s.probe_joint;
          tma = flex_arm(cd.id);
          tma->deflectGear(s.probe_joint, kDeflectionRad);
          x7::LaggedArmOptions lo;
          lo.delay_cycles = kDelayCycles;
          lo.first_passthrough = false;
          lo.exact_velocity = true;   // dq untouched (host estimators
                                      // ignore it; no LSB model here)
          lo.quantize_pos = false;    // the F loop has delay only
          wrapper = std::make_unique<x7::LaggedArm>(*tma, kVelTau, lo);
        } else {
          const bool p1_start = std::strcmp(cd.id, "R2") != 0 &&
                                std::strcmp(cd.id, "L2") != 0;
          const double* pose = p1_start ? p1 : kIncidentPose;
          const auto* trip =
              std::strcmp(cd.id, "R2") == 0 || std::strcmp(cd.id, "L2") == 0
                  ? &inc_trip
                  : &p1_trip;
          s.traj = trip;
          s.duration = trip->duration();
          sim = rigid_arm(pose);
          if (std::strcmp(cd.id, "D+") == 0) {
            sim->setDisturbance(1, kDisturbNm);
          } else if (std::strcmp(cd.id, "D-") == 0) {
            sim->setDisturbance(1, -kDisturbNm);
          }
          if (std::strcmp(cd.id, "D+") == 0 ||
              std::strcmp(cd.id, "D-") == 0) {
            s.duration = trip->duration() + kHoldS;
            s.settle_t_end = trip->duration();
            s.score_joint = 1;
          }
          if (cd.delayed) {
            x7::LaggedArmOptions lo;
            lo.delay_cycles = kDelayCycles;
            lo.first_passthrough = false;
            lo.exact_velocity = kind == Kind::EflIdeal;
            lo.quantize_pos = true;  // encoder quantization per case
            wrapper = std::make_unique<x7::LaggedArm>(*sim, kVelTau, lo);
          }
        }
        arm::Arm& robot =
            wrapper ? static_cast<arm::Arm&>(*wrapper)
                    : (sim ? static_cast<arm::Arm&>(*sim)
                           : static_cast<arm::Arm&>(*tma));
        std::string trace = out_dir + "/case_" + cd.id + "_" +
                            kindName(kind) + ".csv";
        std::replace(trace.begin(), trace.end(), '+', 'p');
        std::unique_ptr<model::ZvsWriter> zvs;
        if (want_zvs && sim) {  // TwoMassArm is not a roki chain
          zvs = std::make_unique<model::ZvsWriter>(zvs_for(trace));
          sim->logTo(zvs.get());
          ++zvs_files;
        }
        cells.push_back(
            {cd.id, kind, runCell(chain, map, robot, s, trace)});
        std::printf("case %-3s %-12s rms %.5f peak %.5f tau_peak %.3f "
                    "sat %ld%s\n",
                    cd.id, kindName(kind), cells.back().r.rms_total,
                    cells.back().r.peak_err, cells.back().r.peak_tau,
                    cells.back().r.sat_count,
                    cells.back().r.completed ? "" : "  [INCOMPLETE]");
      }
    }

    // ---------------------------------------------- decision rule
    auto find = [&](const char* id, Kind k) -> const RunResult* {
      for (const auto& c : cells) {
        if (c.case_id == id && c.kind == k) return &c.r;
      }
      return nullptr;
    };
    const std::size_t expected_cells = 4 * 4 + 2 * 3 + 2 * 3;  // 28
    if (only_case.empty() && cells.size() != expected_cells) {
      reasons.push_back("incomplete matrix: expected 28 cells");
    }
    bool have_decision = false;
    bool c1 = false, c2 = false, c3 = false, c4 = false,
         promising = false;
    if (only_case.empty() && have_gains &&
        cells.size() == expected_cells) {
      std::vector<study::CasePair> pairs, delayed;
      for (const auto& cd : case_defs) {
        const Kind cmp_kind =
            cd.flexible ? Kind::PracticalGf : Kind::Practical;
        const RunResult* efl = find(cd.id, Kind::EflHost);
        const RunResult* cmp = find(cd.id, cmp_kind);
        if (efl == nullptr || cmp == nullptr) continue;
        study::CasePair p;
        p.id = cd.id;
        p.in_s = cd.in_s;
        p.rms_efl = efl->rms_total;
        p.rms_cmp = cmp->rms_total;
        p.peak_err_efl = efl->peak_err;
        p.peak_err_cmp = cmp->peak_err;
        p.peak_tau_efl = efl->peak_tau;
        p.peak_tau_cmp = cmp->peak_tau;
        p.sat_efl = efl->sat_count;
        p.sat_cmp = cmp->sat_count;
        p.efl_complete_finite = efl->completed && efl->finite;
        pairs.push_back(p);
        if (cd.delayed) delayed.push_back(p);
      }
      const RunResult* f4e = find("F4", Kind::EflHost);
      const RunResult* f4c = find("F4", Kind::PracticalGf);
      const RunResult* f13e = find("F13", Kind::EflHost);
      const RunResult* f13c = find("F13", Kind::PracticalGf);
      have_decision = true;
      c1 = study::criterionC1(pairs);
      c2 = study::criterionC2(pairs);
      c3 = study::criterionC3(delayed);
      c4 = study::criterionC4(f4e->modal, f4c->modal, f13e->modal,
                              f13c->modal);
      promising = c1 && c2 && c3 && c4;
    }

    // ------------------------------------------------------- JSON
    study::JsonWriter j;
    j.beginObject();
    j.key("schema_version");
    j.value(kSchemaVersion);
    j.key("study");
    j.value("offline exact-feedback-linearization study "
            "(docs/records/history.md (EFL study))");
    j.key("invocation");
    {
      std::string inv;
      for (int i = 0; i < argc; ++i) {
        if (i > 0) inv += ' ';
        inv += argv[i];
      }
      j.value(inv);
    }
    j.key("git");
    j.beginObject();
    j.key("build_sha");
    j.value(x7::gitrev::kBuildSha);
    j.key("build_dirty");
    j.value(x7::gitrev::kBuildDirty);
    j.key("head");
    j.value(head);
    j.key("worktree_clean");
    j.value(status.empty());
    j.endObject();
    j.key("build_type");
    j.value(RTCTRL_BUILD_TYPE);
    j.key("canonical");
    j.value(reasons.empty());
    j.key("noncanonical_reasons");
    j.beginArray();
    for (const auto& r : reasons) j.value(r);
    j.endArray();
    j.key("frozen");
    j.beginObject();
    j.key("tau_max_nm");
    j.beginArray();
    for (const double v : kTauMax) j.value(v);
    j.endArray();
    j.key("assumed_servo_current_limit_raw");
    j.beginArray();
    for (const long v : kAssumedCurrentLimitRaw) j.value(v);
    j.endArray();
    j.key("scale");
    j.value(kScale);
    j.key("dev_pose");
    j.beginArray();
    for (const double v : kDevPose) j.value(v);
    j.endArray();
    j.key("incident_pose");
    j.beginArray();
    for (const double v : kIncidentPose) j.value(v);
    j.endArray();
    j.key("delay_cycles");
    j.value(kDelayCycles);
    j.key("disturbance_nm");
    j.value(kDisturbNm);
    j.key("hold_s");
    j.value(kHoldS);
    j.key("settle_band_rad");
    j.value(kBandRad);
    j.key("settle_dwell_s");
    j.value(kDwellS);
    j.key("steady_state_window_s");
    j.value(kSsWindowS);
    j.key("gear_deflection_rad");
    j.value(kDeflectionRad);
    j.key("modal_floor_rad");
    j.value(kFloorEps);
    j.key("modal_windows_s");
    j.beginArray();
    for (const double v : {kEarly0, kEarly1, kWin1a, kWin1b, kWin2a,
                           kWin2b}) {
      j.value(v);
    }
    j.endArray();
    j.endObject();

    auto emitRun = [&](const RunResult& r) {
      j.key("completed");
      j.value(r.completed);
      j.key("finite");
      j.value(r.finite);
      if (!r.fail_reason.empty()) {
        j.key("fail_reason");
        j.value(r.fail_reason);
      }
      j.key("rms_total");
      j.value(r.rms_total);
      j.key("peak_err");
      j.value(r.peak_err);
      j.key("rms_joint");
      j.beginArray();
      for (const double v : r.rms_joint) j.value(v);
      j.endArray();
      j.key("peak_err_joint");
      j.beginArray();
      for (const double v : r.peak_err_joint) j.value(v);
      j.endArray();
      j.key("rms_tau");
      j.value(r.rms_tau);
      j.key("peak_tau");
      j.value(r.peak_tau);
      j.key("sat_count");
      j.value(r.sat_count);
      if (r.has_settling) {
        j.key("settled");
        j.value(r.settling.settled);
        j.key("settle_time_s");
        if (r.settling.settled) {
          j.value(r.settling.settle_time);
        } else {
          j.null();
        }
        j.key("steady_state_err");
        j.value(r.settling.steady_state_err);
      }
      if (r.has_modal) {
        j.key("modal_class");
        switch (r.modal.cls) {
          case study::ModalClass::Uncensored:
            j.value("uncensored");
            break;
          case study::ModalClass::CensoredDecayed:
            j.value("censored-decayed");
            break;
          case study::ModalClass::CensoredGrown:
            j.value("censored-grown");
            break;
          case study::ModalClass::Inconclusive:
            j.value("inconclusive");
            break;
        }
        j.key("modal_rate");
        if (r.modal.cls == study::ModalClass::Uncensored) {
          j.value(r.modal.rate);  // censored rates are JSON null
        } else {
          j.null();
        }
        j.key("modal_amplitudes");
        j.beginArray();
        j.value(r.modal.a_early);
        j.value(r.modal.a1);
        j.value(r.modal.a2);
        j.endArray();
      }
    };

    auto emitPose = [&](const double* pose) {
      j.beginArray();
      for (int i = 0; i < kCanonicalDof; ++i) j.value(pose[i]);
      j.endArray();
    };
    // Takes the SAME options object the plant was constructed from
    // (rigidOptions) — an independent default-construction here could
    // silently go stale against a future override (review finding).
    auto emitSimPlant = [&](const arm::SimArm::Options& o) {
      j.beginObject();
      j.key("type");
      j.value("SimArm");
      j.key("model_path");
      j.value(o.model_path);
      j.key("sim_dt");
      j.value(o.sim_dt);
      j.key("control_dt");
      j.value(o.control_dt);
      j.key("reflected_inertia_kgm2");
      j.value(o.reflected_inertia);
      j.key("numeric_damping_ratio");
      j.value(o.numeric_damping_ratio);
      j.key("finger_couple_k");
      j.value(o.couple_k);
      j.key("finger_couple_c");
      j.value(o.couple_c);
      j.key("effort_limit8");
      j.beginArray();
      for (const double v : o.effort_limit8) j.value(v);
      j.endArray();
      j.endObject();
    };
    auto emitFlexPlant = [&](const x7::TwoMassArm::Options& o) {
      j.beginObject();
      j.key("type");
      j.value("TwoMassArm");
      j.key("sim_dt");
      j.value(o.sim_dt);
      j.key("control_dt");
      j.value(o.control_dt);
      j.key("effort_limit8");
      j.beginArray();
      for (const double v : o.effort_limit8) j.value(v);
      j.endArray();
      j.key("joints");
      j.beginArray();
      for (const auto& p : o.joints) {
        j.beginObject();
        j.key("j_m");
        j.value(p.j_m);
        j.key("j_l");
        j.value(p.j_l);
        j.key("k_g");
        j.value(p.k_g);
        j.key("c_g");
        j.value(p.c_g);
        j.key("b_l");
        j.value(p.b_l);
        j.endObject();
      }
      j.endArray();
      j.endObject();
    };

    j.key("grid");
    j.beginArray();
    {
      j.beginObject();
      j.key("controller");
      j.value("PRACTICAL");
      j.key("scenario");
      j.value("development");
      j.key("start_pose");
      emitPose(kDevPose);
      j.key("plant_params");
      emitSimPlant(rigidOptions(kDevPose));
      emitRun(base_r);
      j.endObject();
    }
    for (const auto& g : grid) {
      j.beginObject();
      j.key("controller");
      j.value(kindName(g.kind));
      j.key("omega_n");
      j.value(g.omega);
      j.key("zeta");
      j.value(g.zeta);
      j.key("scenario");
      j.value("development");
      j.key("start_pose");
      emitPose(kDevPose);
      j.key("plant_params");
      emitSimPlant(rigidOptions(kDevPose));
      emitRun(g.r);
      j.endObject();
    }
    j.endArray();
    j.key("selected_gains");
    if (have_gains) {
      j.beginObject();
      j.key("omega_n");
      j.value(sel_omega);
      j.key("zeta");
      j.value(sel_zeta);
      j.key("kp_prime");
      j.value(kp_sel);
      j.key("kd_prime");
      j.value(kd_sel);
      j.endObject();
    } else {
      j.null();
    }
    j.key("cases");
    j.beginArray();
    for (const auto& c : cells) {
      const bool flexible = c.case_id == "F4" || c.case_id == "F13";
      j.beginObject();
      j.key("case");
      j.value(c.case_id);
      j.key("controller");
      j.value(kindName(c.kind));
      j.key("velocity_source");
      j.value(velocitySource(c.kind));
      j.key("gains");
      if (c.kind == Kind::Practical || c.kind == Kind::PracticalGf) {
        j.value("shipped tuning (crane_x7_tuning.hpp)");
      } else if (!have_gains) {
        j.null();
      } else {
        j.beginObject();
        j.key("kp_prime");
        j.value(kp_sel);
        j.key("kd_prime");
        j.value(kd_sel);
        j.endObject();
      }
      const double* rigid_pose =
          c.case_id == "R2" || c.case_id == "L2" ? kIncidentPose : p1;
      j.key("plant");
      j.value(flexible ? "TwoMassArm" : "SimArm");
      j.key("plant_params");
      if (flexible) {
        emitFlexPlant(flexOptionsFor(c.case_id));
      } else {
        emitSimPlant(rigidOptions(rigid_pose));
      }
      j.key("start");
      j.value(c.case_id == "R2" || c.case_id == "L2"
                  ? "incident pose"
                  : (flexible ? "zero pose" : "P1"));
      j.key("start_pose");
      if (flexible) {
        const double zeros[kCanonicalDof] = {};
        emitPose(zeros);
      } else {
        emitPose(rigid_pose);
      }
      j.key("scale");
      j.value(flexible ? 0.0 : kScale);
      j.key("delay_cycles");
      j.value(c.case_id == "R1" || c.case_id == "R2" ? 0 : kDelayCycles);
      j.key("disturbance_nm");
      if (c.case_id == "D+") {
        j.value(kDisturbNm);
      } else if (c.case_id == "D-") {
        j.value(-kDisturbNm);
      } else {
        j.value(0.0);
      }
      j.key("tau_max_nm");
      j.beginArray();
      for (const double v : kTauMax) j.value(v);
      j.endArray();
      emitRun(c.r);
      j.endObject();
    }
    j.endArray();
    j.key("decision");
    if (have_decision) {
      j.beginObject();
      j.key("c1_rms_improvement");
      j.value(c1);
      j.key("c2_no_regression");
      j.value(c2);
      j.key("c3_delayed_completion");
      j.value(c3);
      j.key("c4_flexible_screens");
      j.value(c4);
      j.key("promising");
      j.value(promising);
      j.endObject();
    } else {
      j.null();
    }
    if (only_case.empty() && !have_gains) {
      j.key("decision_note");
      j.value("not evaluable: the preregistered selection produced no "
              "surviving EFL-host gain candidate (a negative result "
              "for the study question)");
    }
    j.endObject();

    const std::string json_path = out_dir + "/results.json";
    std::FILE* f = std::fopen(json_path.c_str(), "w");
    if (f == nullptr) {
      std::fprintf(stderr, "cannot write %s\n", json_path.c_str());
      return 1;
    }
    std::fwrite(j.str().data(), 1, j.str().size(), f);
    std::fclose(f);

    std::printf("results: %s\n", json_path.c_str());
    if (want_zvs) {
      std::printf("zvs motion files: %d (play: rk_anim %s <file>)\n",
                  zvs_files, kModelPath);
    }
    if (have_decision) {
      std::printf("decision: C1 %d C2 %d C3 %d C4 %d -> %s\n", c1, c2,
                  c3, c4, promising ? "PROMISING" : "NOT promising");
    }
    if (reasons.empty()) {
      std::printf("CANONICAL result (clean tree at built HEAD, "
                  "complete matrix)\n");
    } else {
      std::printf("NONCANONICAL result:\n");
      for (const auto& r : reasons) {
        std::printf("  - %s\n", r.c_str());
      }
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
    return 1;
  }
}
