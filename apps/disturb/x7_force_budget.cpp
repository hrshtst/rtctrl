// Offline force-budget SCREEN for a planned link-force disturbance. It
// computes the largest force amplitude satisfying user-supplied joint
// holding-torque allowances at sampled reference poses under a
// quasi-static model. It is not a hardware safety or fuse rating. Never
// opens the bus.
// Usage: x7_force_budget --config budget.toml [--reference FILE]
//                        [--output FILE] [--check]

#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "disturb/force_budget_config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/force_budget.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/link_force.hpp"
#include "rtctrl/model/zvs_trajectory.hpp"
#include "rtctrl/version.hpp"

namespace disturb = x7::disturb;
namespace fs = std::filesystem;
namespace model = rtctrl::model;

namespace {

void usage() {
  std::printf(
      "usage: x7_force_budget --config FILE [options]\n"
      "  --reference FILE\n"
      "  --output FILE\n"
      "  --check\n");
}

std::string vectorText(const std::array<double, 3>& v) {
  char buffer[96];
  std::snprintf(buffer, sizeof buffer, "[%.9g, %.9g, %.9g]", v[0], v[1], v[2]);
  return buffer;
}

std::string allowanceText(const model::ForceBudgetOptions& options) {
  std::ostringstream out;
  out.precision(9);
  out << '[';
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    out << (i == 0 ? "" : ", ") << options.joint_allowance_nm[i];
  }
  out << ']';
  return out.str();
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const auto cli = disturb::parseCli(argc, argv);
    if (cli.help) {
      usage();
      return 0;
    }
    for (const auto& line : model::forceBudgetPreamble()) {
      std::printf("%s\n", line.c_str());
    }
    auto config = disturb::loadConfig(cli.config_path);
    disturb::applyOverrides(cli, &config);

    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    model::ZvsTrajectory reference(config.reference_path.string(), map,
                                   config.reference);
    const auto spec = model::loadLinkForce(config.link_force_path.string());
    if (chain.linkIndex(spec.link) < 0) {
      throw std::runtime_error("link force: unknown link '" + spec.link + "'");
    }
    if (cli.check) {
      std::printf("configuration preflight passed\n");
      return 0;
    }
    if (fs::exists(fs::symlink_status(config.csv_path))) {
      throw std::runtime_error("refusing to overwrite '" +
                               config.csv_path.string() + "'");
    }

    const auto result =
        model::evaluateForceBudget(chain, map, reference, spec, config.budget);
    const bool anchor = spec.direction == model::ForceDirection::Anchor;
    std::vector<std::string> metadata = {
        "tool: x7_force_budget " + std::string(rtctrl::version()),
        "config: " + config.source_path.string(),
        "reference: " + config.reference_path.string(),
        "link_force: " + config.link_force_path.string(),
        "link: " + spec.link,
        "point_link_m: " + vectorText(spec.point_link_m),
        anchor ? "anchor_world_m: " + vectorText(spec.anchor_world_m)
               : "direction_world: " + vectorText(spec.direction_world),
        "planned_magnitude_n: " + std::to_string(spec.magnitude_n),
        "sample_rate_hz: " + std::to_string(config.budget.sample_rate_hz),
        "joint_allowance_nm: " + allowanceText(config.budget),
        std::string("status: ") + model::budgetStatusName(result.status),
    };
    if (fs::path(config.csv_path).has_parent_path()) {
      fs::create_directories(fs::path(config.csv_path).parent_path());
    }
    model::writeForceBudgetCsv(config.csv_path.string(), result, metadata);

    std::printf("samples: %zu over %.9g s at %.9g Hz\n", result.samples.size(),
                reference.duration(), config.budget.sample_rate_hz);
    std::printf("status: %s\n", model::budgetStatusName(result.status));
    const bool ok = result.status == model::BudgetStatus::Ok;
    if (ok) {
      if (std::isinf(result.amplitude_bound_n)) {
        std::printf(
            "amplitude bound: no bound imposed by the modeled joint-torque "
            "constraints\n");
      } else {
        std::printf("amplitude bound: %.6g N (binding joint %d at t=%.3f s)\n",
                    result.amplitude_bound_n, result.binding_joint,
                    result.binding_time_s);
      }
      if (std::isinf(result.disturbance_only_bound_n)) {
        std::printf(
            "disturbance-only bound: no bound imposed by the modeled "
            "joint-torque constraints\n");
      } else {
        std::printf("disturbance-only bound: %.6g N\n",
                    result.disturbance_only_bound_n);
      }
    }
    if (anchor) {
      std::printf("minimum anchor distance: %.6g m\n",
                  result.min_anchor_distance_m);
    }
    std::printf("planned magnitude: %.6g N\n", spec.magnitude_n);
    if (!ok) {
      std::printf("headroom: n/a (evaluation failed)\n");
    } else if (spec.magnitude_n == 0.0) {
      std::printf("headroom: n/a (planned magnitude is zero)\n");
    } else if (std::isinf(result.amplitude_bound_n)) {
      std::printf("headroom: n/a (no modeled bound)\n");
    } else {
      std::printf("headroom: %.4g x\n",
                  result.amplitude_bound_n / spec.magnitude_n);
    }

    int exit_code = 1;
    switch (result.status) {
      case model::BudgetStatus::Ok:
        if (spec.magnitude_n <= result.amplitude_bound_n) {
          std::printf(
              "verdict: within the supplied sampled model "
              "allowances\n");
          exit_code = 0;
        } else {
          std::printf(
              "verdict: EXCEEDS the supplied sampled model "
              "allowances\n");
        }
        break;
      case model::BudgetStatus::Infeasible:
        std::printf(
            "verdict: INFEASIBLE at t=%.3f s joint %d (its gravity load "
            "exceeds its allowance)\n",
            result.first_failure_time_s, result.first_failure_joint);
        break;
      case model::BudgetStatus::DegenerateAnchor:
        std::printf(
            "verdict: EVALUATION FAILED (degenerate anchor) at "
            "t=%.3f s\n",
            result.first_failure_time_s);
        break;
      case model::BudgetStatus::NonFiniteEvaluation:
        std::printf(
            "verdict: EVALUATION FAILED (non-finite evaluation) at "
            "t=%.3f s joint %d\n",
            result.first_failure_time_s, result.first_failure_joint);
        break;
    }
    std::printf("csv: %s\n", config.csv_path.string().c_str());
    return exit_code;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_force_budget: %s\n", error.what());
    return 1;
  }
}
