// Offline Cartesian point-to-point planner for CRANE-X7. Reads a world-frame
// TCP motion from TOML, solves continuation-seeded IK, and writes a model-space
// .zvs sequence for rk_anim. This app never opens the hardware bus.
//
// Usage: x7_plan_ptp --config PLAN.toml [overrides]
//   --output FILE
//   --bundle NEW_DIRECTORY
//   --motion-time SEC
//   --max-linear-velocity M/S --max-angular-velocity RAD/S
//   --sample-rate HZ
//   --profile linear|trapezoidal|minimum-jerk
//   --strict-ik | --no-strict-ik

#include <cstdio>
#include <exception>
#include <memory>

#include "plan/ptp_bundle.hpp"
#include "plan/ptp_config.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/ptp_planner.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"
#include "rtctrl/version.hpp"

namespace x7::gitrev {
extern const char* const kBuildSha;
extern const bool kBuildDirty;
}  // namespace x7::gitrev

namespace model = rtctrl::model;

namespace {

void usage() {
  std::printf(
      "usage: x7_plan_ptp --config PLAN.toml [options]\n"
      "  --output FILE\n"
      "  --bundle NEW_DIRECTORY  create a portable, immutable archive\n"
      "  --motion-time SEC\n"
      "  --max-linear-velocity M/S --max-angular-velocity RAD/S\n"
      "  --sample-rate HZ\n"
      "  --profile linear|trapezoidal|minimum-jerk\n"
      "  --strict-ik | --no-strict-ik\n");
}

void printIkResult(const char* prefix, std::size_t sample, double time,
                   const model::IkResult& result) {
  std::fprintf(stderr,
               "%s at sample %zu (t=%.6f s): residuals %.6g m / %.6g "
               "rad after %d iterations%s%s\n",
               prefix, sample, time, result.pos_residual,
               result.att_residual, result.iterations,
               result.within_limits ? "" : " (joint limits violated)",
               result.finite ? "" : " (non-finite solution)");
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::ptp::parseCli(argc, argv);
  if (!cli.ok) {
    std::fprintf(stderr, "%s\n", cli.error.c_str());
    usage();
    return 1;
  }
  if (cli.help) {
    usage();
    return 0;
  }

  try {
    // Bundle refusal precedes config loading by design: an existing archive
    // is never inspected, merged, or modified, even when the source config
    // is malformed or missing.
    std::unique_ptr<x7::ptp::BundleWorkspace> bundle;
    if (cli.bundle_path) {
      bundle =
          std::make_unique<x7::ptp::BundleWorkspace>(*cli.bundle_path);
    }

    auto config = x7::ptp::loadConfig(cli.config_path);
    x7::ptp::applyOverrides(cli, &config);
    if (bundle) {
      const auto prepared = x7::ptp::prepareBundle(
          bundle->staging(), cli.config_path, config);
      config = x7::ptp::loadConfig(prepared.config_path);
      // output remains invocation-relative for ordinary configs. Bundle
      // creation owns its destination explicitly inside the staging tree.
      config.output_path = prepared.output_path;
    }

    model::ChainModel chain(config.model_path.string());
    model::JointMap map(chain);
    model::ZVector q8(model::kCanonicalDof);
    model::ZVector q9(chain.jointSize());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      q8[i] = config.initial_joints[i];
    }
    map.expand(q8.get(), q9.get());

    model::CartesianPtpPlanner planner(chain, map, config.end_effector);
    const auto plan =
        planner.plan(config.start, config.end, q9.get(), config.options);

    for (const auto& warning : plan.ik_warnings) {
      printIkResult("warning: IK did not converge", warning.sample,
                    warning.time, warning.result);
    }

    int written_frames = 0;
    {
      model::ZvsWriter writer(config.output_path.string());
      model::ZVector displacement(chain.jointSize());
      for (const auto& sample : plan.samples) {
        for (int i = 0; i < chain.jointSize(); ++i) {
          displacement[i] = sample.displacement[i];
        }
        writer.frame(plan.interval, displacement.get());
      }
      written_frames = writer.frames();
    }

    std::string display_model = config.model_path.string();
    std::string display_output = config.output_path.string();
    if (bundle) {
      x7::ptp::writeBundleManifest(
          bundle->staging(), rtctrl::version(), x7::gitrev::kBuildSha,
          x7::gitrev::kBuildDirty);
      bundle->publish();
      display_model =
          (bundle->target() / "model" / config.model_path.filename()).string();
      display_output = (bundle->target() / "trajectory.zvs").string();
      std::printf("created bundle %s\n", bundle->target().string().c_str());
    }

    const double peak = model::ptpPeakSpeedFactor(
        config.options.profile,
        config.options.trapezoid_acceleration_fraction);
    const double linear_speed =
        plan.duration > 0.0
            ? peak * model::cartesianTranslationDistance(config.start,
                                                         config.end) /
                  plan.duration
            : 0.0;
    const double angular_speed =
        plan.duration > 0.0
            ? peak * model::cartesianRotationDistance(config.start,
                                                      config.end) /
                  plan.duration
            : 0.0;
    std::printf("wrote %d frames to %s\n", written_frames,
                display_output.c_str());
    std::printf(
        "profile: %s, duration: %.6g s, interval: %.6g s\n"
        "peak Cartesian speed: %.6g m/s, %.6g rad/s\n"
        "IK warnings: %zu, worst residuals: %.6g m / %.6g rad\n"
        "peak sampled joint speed: %.6g rad/s\n"
        "view: rk_anim %s %s\n",
        x7::ptp::profileName(config.options.profile), plan.duration,
        plan.interval, linear_speed, angular_speed, plan.ik_warnings.size(),
        plan.worst_position_residual, plan.worst_attitude_residual,
        plan.peak_joint_velocity, display_model.c_str(),
        display_output.c_str());
    return 0;
  } catch (const model::PtpPlanningError& error) {
    printIkResult("error: IK failed", error.sample(), error.time(),
                  error.result());
    return 1;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "error: %s\n", error.what());
    return 1;
  }
}
