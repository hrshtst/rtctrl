// Computed-torque robustness to mass-property model error, assessed in
// the roki forward-dynamics sim through the bridge — in the spirit of
// mi-lib-tutorial's roki008 tracking demo, with a simple joint-space
// round trip in place of the PUMA star. The PLANT (SimArm) integrates
// the true CRANE-X7 model; the CONTROLLER's inverse-dynamics
// feedforward uses a copy whose link masses and inertias are scaled by
// --mass-scale, so the feedforward is wrong by construction and the PD
// (plus optional integrator) must absorb the difference.
//
// Usage: x7_ct_mass_error [--mass-scale s] [--mass-error e]
//                         [--com-error m] [--seed n] [--ki gain]
//                         [--out file.csv]
// --mass-scale applies the uniform density-style factor; --mass-error /
// --com-error add the tutorial-style randomized per-link perturbation
// (symmetric mass factor, COM offset cube in meters), reproducible for
// a given --seed. Scale applies first; both may combine.
// CSV columns: t, qd0..qd7, q0..q7 (canonical order, rad).

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

namespace {

// Strict double parse: the whole token must convert.
bool parseDouble(const char* s, double* out) {
  char* end = nullptr;
  *out = std::strtod(s, &end);
  return end != s && *end == '\0';
}

// Strict uint64 parse — a double round trip would accept inf, hit UB
// on out-of-range conversion, and collapse seeds above 2^53.
bool parseSeed(const char* s, std::uint64_t* out) {
  const char* end = s + std::strlen(s);
  const auto res = std::from_chars(s, end, *out);
  return res.ec == std::errc() && res.ptr == end;
}

int usage() {
  std::fprintf(stderr,
               "usage: x7_ct_mass_error [--mass-scale s] [--mass-error e] "
               "[--com-error m] [--seed n] [--ki gain] [--out file.csv]\n");
  return 1;
}

}  // namespace

int main(int argc, char* argv[]) {
  constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
  double mass_scale = 1.2;
  double mass_error = 0.0;
  double com_error = 0.0;
  std::uint64_t seed = 1;
  double ki = 0.0;
  std::string out_path = "ct_mass_error.csv";
  for (int i = 1; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--mass-scale") == 0) {
      if (!has_value || !parseDouble(argv[++i], &mass_scale)) return usage();
    } else if (std::strcmp(argv[i], "--mass-error") == 0) {
      if (!has_value || !parseDouble(argv[++i], &mass_error)) return usage();
    } else if (std::strcmp(argv[i], "--com-error") == 0) {
      if (!has_value || !parseDouble(argv[++i], &com_error)) return usage();
    } else if (std::strcmp(argv[i], "--seed") == 0) {
      if (!has_value || !parseSeed(argv[++i], &seed)) return usage();
    } else if (std::strcmp(argv[i], "--ki") == 0) {
      if (!has_value || !parseDouble(argv[++i], &ki)) return usage();
    } else if (std::strcmp(argv[i], "--out") == 0) {
      if (!has_value) return usage();
      out_path = argv[++i];
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return usage();
    }
  }

  try {
    // The same start/goal postures and gains as the M8 sim acceptance
    // (tracking_sim_test), on a faster round trip (2 s minimum legs vs
    // the test's one-way 3 s minimum — the RMS numbers are therefore
    // NOT directly comparable to the test's). Hardware countermeasures
    // (PD low-pass, friction integrator) stay off by default: this
    // isolates the model-error effect in the ideal rigid sim.
    const double start[] = {0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0, 0.1};
    const double goal[] = {0.4, 0.7, -0.3, -1.2, 0.3, -0.6, 0.5, 0.3};
    model::ZVector q0(model::kCanonicalDof), qf(model::kCanonicalDof);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      q0[i] = start[i];
      qf[i] = goal[i];
    }
    const model::RoundTripTrajectory trajectory(
        model::MinJerkTrajectory::withVelocityLimit(q0, qf, 1.0, 2.0),
        model::MinJerkTrajectory::withVelocityLimit(qf, q0, 1.0, 2.0));
    const double duration = trajectory.duration() + 0.5;

    model::ChainModel ctrl_model(kModelPath);
    const double true_mass = ctrl_model.totalMass();
    ctrl_model.scaleMassProperties(mass_scale);
    if (mass_error > 0.0 || com_error > 0.0) {
      ctrl_model.perturbMassProperties(mass_error, com_error, seed);
    }
    model::JointMap map(ctrl_model);
    arm::ComputedTorque controller(ctrl_model, map, trajectory, 20.0, 2.0);
    controller.setPdFilterTau(0.0);
    controller.setIntegral(ki, ki > 0.0 ? 1.5 : 0.0);

    arm::SimArm::Options opt;
    opt.model_path = kModelPath;
    opt.initial_q8.assign(start, start + model::kCanonicalDof);
    arm::SimArm sim(opt);
    sim.setMode(arm::ControlMode::Current);
    sim.activate();

    std::FILE* out = std::fopen(out_path.c_str(), "w");
    if (out == nullptr) {
      std::perror(out_path.c_str());
      return 1;
    }
    std::fprintf(out, "t");
    for (int i = 0; i < model::kCanonicalDof; ++i) std::fprintf(out, ",qd%d", i);
    for (int i = 0; i < model::kCanonicalDof; ++i) std::fprintf(out, ",q%d", i);
    std::fprintf(out, "\n");

    model::ZVector q_d(model::kCanonicalDof);
    arm::JointState state;
    arm::JointCommand cmd;
    double sum_sq[model::kCanonicalDof] = {};
    int samples = 0;
    for (double t = 0.0; t < duration; t += sim.dt()) {
      sim.readState(state);
      controller.update(state, cmd, t);
      sim.writeCommand(cmd);
      if (!sim.step()) {
        std::fprintf(stderr, "sim step failed at t=%.3f\n", t);
        std::fclose(out);
        return 1;
      }
      trajectory.sample(t, q_d.get());
      std::fprintf(out, "%.4f", t);
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::fprintf(out, ",%.6f", q_d[i]);
      }
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        std::fprintf(out, ",%.6f", zVecElemNC(state.q.get(), i));
      }
      std::fprintf(out, "\n");
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double e = q_d[i] - zVecElemNC(state.q.get(), i);
        sum_sq[i] += e * e;
      }
      ++samples;
    }
    std::fclose(out);

    double sum_all = 0.0;
    std::printf("mass-scale %.2f, mass-error %.2f, com-error %.3f m, "
                "seed %llu (controller model %.3f kg, true %.3f kg), "
                "ki %.1f, %d cycles -> %s\n",
                mass_scale, mass_error, com_error,
                static_cast<unsigned long long>(seed),
                ctrl_model.totalMass(), true_mass, ki, samples,
                out_path.c_str());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double rms = std::sqrt(sum_sq[i] / samples);
      sum_all += sum_sq[i];
      std::printf("  j%d RMS %.4f rad\n", i, rms);
    }
    std::printf("  all-joint RMS %.4f rad\n",
                std::sqrt(sum_all / (samples * model::kCanonicalDof)));
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
