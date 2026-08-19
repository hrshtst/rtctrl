// Two-mass flexible-joint test fixture for the identification
// pipeline: every joint is an independent motor–gear–link oscillator
// with KNOWN planted modes, so the probe → log → analysis chain can be
// validated against ground truth before touching hardware.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "rtctrl/arm/arm.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace x7 {

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

// Per joint: J_m q̈_m = τ − K_g(q_m−q_l) − C_g(q̇_m−q̇_l);
//           J_l q̈_l = K_g(q_m−q_l) + C_g(q̇_m−q̇_l) − b_l q̇_l.
// Torque enters MOTOR-side; `state.q` reports the LINK side only
// (non-collocated, as on hardware). Gravity-free by design — the
// ident regression anchors at the zero pose where the real model's
// gravity vanishes, and this fixture must never be paired with a real
// gravity feedforward.
class TwoMassArm : public arm::Arm {
 public:
  struct JointParams {
    double j_m = 0.05;  // motor-side inertia [kg·m²]
    double j_l = 0.1;   // link-side inertia [kg·m²]
    double k_g = 0.0;   // gear stiffness [Nm/rad]
    double c_g = 0.0;   // gear damping [Nms/rad]
    double b_l = 0.01;  // link-side viscous damping [Nms/rad]
  };

  // Stiffness/damping from a target transmission mode: the free
  // two-mass resonance ω² = K_g(1/J_m + 1/J_l) with damping ratio ζ
  // against the reduced inertia J_eff = J_m·J_l/(J_m+J_l).
  static JointParams plantMode(double j_l, double j_m, double f_hz,
                               double zeta) {
    JointParams p;
    p.j_l = j_l;
    p.j_m = j_m;
    const double w = 2.0 * M_PI * f_hz;
    const double j_eff = j_m * j_l / (j_m + j_l);
    p.k_g = w * w * j_eff;
    p.c_g = 2.0 * zeta * std::sqrt(p.k_g * j_eff);
    return p;
  }

  struct Options {
    double sim_dt = 1e-4;      // integrator step [s]
    double control_dt = 0.01;  // one step() advances this much [s]
    std::array<JointParams, model::kCanonicalDof> joints;
    std::vector<double> effort_limit8 = {10.0, 10.0, 4.0, 4.0,
                                         4.0,  4.0,  4.0, 4.0};  // [Nm]
    std::vector<double> initial_q8;  // start pose (default zeros)

    Options() {
      // Uninteresting joints: stiff (40 Hz, far above the survey grid)
      // and well damped, so their transmission dynamics stay invisible.
      for (auto& p : joints) p = plantMode(0.1, 0.05, 40.0, 0.5);
      // Planted ground-truth modes (the regression targets): the
      // ~4.5 Hz structural analog on joint 1 and the ~13 Hz gear
      // analog on joint 5.
      joints[1] = plantMode(0.4, 0.05, 4.5, 0.03);
      joints[5] = plantMode(0.01, 0.05, 13.0, 0.05);
    }
  };

  explicit TwoMassArm(Options options = Options())
      : options_(std::move(options)) {
    if (!options_.initial_q8.empty()) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        q_m_[i] = q_l_[i] = options_.initial_q8[i];
      }
    }
  }

  int dof() const override { return model::kCanonicalDof; }
  double dt() const override { return options_.control_dt; }

  bool activate() override {
    zVecZero(cmd_tau_.get());
    active_ = true;
    return true;
  }
  bool deactivate() override {
    zVecZero(cmd_tau_.get());
    active_ = false;
    return true;
  }
  bool setMode(arm::ControlMode mode) override {
    if (active_) return false;
    return mode == arm::ControlMode::Current;  // torque fixture only
  }

  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* cmds = nullptr) override {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      zVecElemNC(state.q.get(), i) = q_l_[i];
      zVecElemNC(state.dq.get(), i) = v_l_[i];
      // measured torque = the clamped motor torque actually applied,
      // the τ_meas analog the FRF's primary variant depends on
      zVecElemNC(state.tau.get(), i) = applied_tau_[i];
    }
    state.t = time_;
    state.seq = seq_;
    if (cmds != nullptr) {
      cmds->applied = applied_rec_;
      cmds->last_attempt = attempt_rec_;
    }
    return true;
  }

  bool writeCommand(const arm::JointCommand& cmd,
                    arm::CommandReceipt* receipt = nullptr) override {
    if (cmd.mode != arm::ControlMode::Current) {
      if (receipt != nullptr) *receipt = {false, 0, time_};
      return false;
    }
    zVecCopyNC(cmd.tau.get(), cmd_tau_.get());
    // Synchronous application, as SimArm: the accepted command IS the
    // actuator goal from the next step (a default-constructed snapshot
    // would blow the latency verifier's deadline instantly).
    ++target_seq_;
    attempt_rec_ = {true, target_seq_, time_, true};
    auto& rec = applied_rec_;
    rec.valid = true;
    rec.target_seq = target_seq_;
    rec.first_cycle = rec.latest_cycle = seq_;
    rec.first_time = rec.latest_time = time_;
    rec.mode = static_cast<std::uint8_t>(cmd.mode);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double lim = options_.effort_limit8[i];
      const double raw = zVecElemNC(cmd.tau.get(), i);
      const double v = std::clamp(raw, -lim, lim);
      rec.applied[i] = v;
      rec.flags[i] = v != raw ? arm::kCmdClamped : 0;
      applied_tau_[i] = active_ ? v : 0.0;
    }
    if (receipt != nullptr) *receipt = {true, target_seq_, time_};
    return true;
  }

  bool step() override {
    const int substeps = std::max(
        1, static_cast<int>(
               std::lround(options_.control_dt / options_.sim_dt)));
    const double h = options_.sim_dt;
    for (int s = 0; s < substeps; ++s) {
      // Semi-implicit Euler (explicit Euler is unstable for lightly
      // damped oscillators; here ω₀h ≈ 0.008 with ~250× margin).
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const auto& p = options_.joints[i];
        const double tau = active_ ? applied_tau_[i] : 0.0;
        const double spring = p.k_g * (q_m_[i] - q_l_[i]) +
                              p.c_g * (v_m_[i] - v_l_[i]);
        const double a_m = (tau - spring) / p.j_m;
        const double a_l = (spring - p.b_l * v_l_[i]) / p.j_l;
        v_m_[i] += a_m * h;
        v_l_[i] += a_l * h;
        q_m_[i] += v_m_[i] * h;
        q_l_[i] += v_l_[i] * h;
      }
      time_ += h;
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (!std::isfinite(q_l_[i]) || !std::isfinite(v_l_[i])) return false;
    }
    ++seq_;  // one fresh feedback sample per control step
    return true;
  }

  const JointParams& params(int i) const { return options_.joints[i]; }

  // Test hooks: displace the motor side against a held link (a pure
  // gear-spring deflection) so release rings the transmission mode.
  void deflectGear(int i, double delta) { q_m_[i] += delta; }
  double motorPos(int i) const { return q_m_[i]; }
  double linkPos(int i) const { return q_l_[i]; }
  double linkVel(int i) const { return v_l_[i]; }
  double time() const { return time_; }

 private:
  Options options_;
  bool active_ = false;
  double time_ = 0.0;
  std::uint64_t seq_ = 0;
  std::uint64_t target_seq_ = 0;
  arm::AppliedTargetRecord applied_rec_;
  arm::WriteAttemptRecord attempt_rec_;
  model::ZVector cmd_tau_{model::kCanonicalDof};
  double applied_tau_[model::kCanonicalDof] = {};
  double q_m_[model::kCanonicalDof] = {};
  double v_m_[model::kCanonicalDof] = {};
  double q_l_[model::kCanonicalDof] = {};
  double v_l_[model::kCanonicalDof] = {};
};

}  // namespace x7
