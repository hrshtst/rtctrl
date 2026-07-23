// Simulation twin of x7_ident over the TwoMassArm fixture with its
// PLANTED modes (joint 1: 4.5 Hz zeta 0.03; joint 5: 13 Hz zeta 0.05)
// — the CSV/JSON source for validating tools/ident_analysis.py against
// known ground truth before any hardware run. Same schedule pipeline,
// no bus/settle/watchdog: the fixture starts at rest and is
// gravity-free (the anchor feedforward is subtracted per C6b —
// docs/IDENTIFICATION_PLAN.md).
//
// Usage: x7_ident_sim [--joint N] [--freqs 4.1,4.25,...] [--amp cap]
//                     [--label name] [--log out.csv]

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ident_common.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "two_mass_arm.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

namespace {

std::vector<double> parseFreqList(const char* text) {
  std::vector<double> freqs;
  const char* p = text;
  while (*p != '\0') {
    char* end = nullptr;
    const double f = std::strtod(p, &end);
    if (end == p) break;
    if (f > 0.0) freqs.push_back(f);
    p = *end == ',' ? end + 1 : end;
  }
  return freqs;
}

}  // namespace

int main(int argc, char* argv[]) {
  int probe_joint = 1;
  std::vector<double> freqs = x7::surveyGridHz();
  double a_cap = 0.15;
  std::string label = "sim";
  std::string log_path = "ident_sim.csv";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--joint") == 0 && i + 1 < argc) {
      probe_joint = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--freqs") == 0 && i + 1 < argc) {
      freqs = parseFreqList(argv[++i]);
    } else if (std::strcmp(argv[i], "--amp") == 0 && i + 1 < argc) {
      a_cap = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--label") == 0 && i + 1 < argc) {
      label = argv[++i];
    } else if (std::strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
      log_path = argv[++i];
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 1;
    }
  }
  if (probe_joint < 0 || probe_joint >= model::kCanonicalDof) {
    std::fprintf(stderr, "--joint must be in [0, %d]\n",
                 model::kCanonicalDof - 1);
    return 1;
  }
  if (freqs.empty()) {
    std::fprintf(stderr, "--freqs parsed to an empty grid\n");
    return 1;
  }
  a_cap = std::clamp(a_cap, x7::kAmpFloorNm, x7::kAmpCapHardNm);

  try {
    model::ChainModel chain("models/crane_x7/crane_x7.ztk");
    model::JointMap map(chain);
    x7::TwoMassArm robot;
    if (!robot.setMode(arm::ControlMode::Current) || !robot.activate()) {
      std::fprintf(stderr, "fixture activation failed\n");
      return 1;
    }

    // amplitude rule against the fixture's TRUE low-frequency inertia
    const auto& p = robot.params(probe_joint);
    const double j_hat = p.j_l + p.j_m;
    auto dwells = x7::buildSchedule(freqs, j_hat, a_cap);
    if (!x7::scheduleFitsBudget(dwells)) {
      std::fprintf(stderr,
                   "schedule worst case %.1f s exceeds T_stop %.1f s — "
                   "split it (--freqs)\n",
                   x7::baseWorstSeconds(dwells), x7::kTStopS);
      return 1;
    }
    std::printf("probe joint %d (J_hat %.4f kg m^2), %zu dwells, worst "
                "case %.1f s\n",
                probe_joint, j_hat, dwells.size(),
                x7::baseWorstSeconds(dwells));

    std::string meta = "label=" + label +
                       " probe_joint=" + std::to_string(probe_joint) +
                       " sim=two_mass";
    std::FILE* log = x7::openIdentCsvLog(log_path, meta);
    if (!log) {
      std::fprintf(stderr, "cannot open log file %s\n", log_path.c_str());
      return 1;
    }

    x7::IdentRun::Options opt;
    opt.probe_joint = probe_joint;
    opt.dwells = dwells;
    opt.anchor.assign(model::kCanonicalDof, 0.0);
    opt.tau_max.assign(model::kCanonicalDof, 4.0);
    opt.gravity_free_plant = true;  // C6b
    opt.health = [](std::array<x7::JointHealth,
                               model::kCanonicalDof>& h) {
      for (auto& j : h) j = {45.0, 12.0};  // constant, healthy
      return true;
    };

    x7::IdentRun ident(chain, map, opt, log);
    arm::run(robot, ident, x7::kTStopS, &ident);
    robot.deactivate();
    std::fclose(log);
    x7::writeDwellJson(log_path + ".dwells.json", opt, ident);

    for (const auto& r : ident.results()) {
      const double resp = r.resp[probe_joint].abs();
      const double tau = r.tau_meas.abs();
      std::printf("  %6.2f Hz  amp %.3f Nm  %s  |q| %.5f rad  "
                  "|H| %.5f rad/Nm%s\n",
                  r.spec.freq_hz, r.spec.amp_nm,
                  r.completed ? "done" : "----", resp,
                  tau > 1e-9 ? resp / tau : 0.0,
                  r.low_confidence ? "  low-confidence" : "");
    }
    const bool ok =
        ident.outcome() == x7::IdentRun::Outcome::Completed;
    std::printf("%s — telemetry in %s, dwell summary in %s.dwells.json\n",
                ok ? "done" : "ABORTED", log_path.c_str(),
                log_path.c_str());
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
