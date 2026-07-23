// Shared between x7_ident (hardware) and x7_ident_sim: the stepped-sine
// identification run — a constant-anchor ComputedTorque with a torque
// probe superposed on ONE joint, online I/Q demodulation, all-joint
// safety monitors with the hard/soft fault taxonomy, and the session
// duration budget (docs/IDENTIFICATION_PLAN.md).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "latency_verifier.hpp"
#include "rtctrl/arm/computed_torque.hpp"
#include "rtctrl/arm/crane_x7_tuning.hpp"
#include "rtctrl/arm/runner.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace x7 {

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
namespace tuning = rtctrl::arm::tuning;

// ---------------------------------------------------------------------------
// Health supervision seam: ident_common stays hardware-free — the app
// injects a callable (hardware: adapted from CraneX7::lastFeedbackStamped;
// sim twin: constants; tests: scripted faults). Returns false on a read
// failure (treated as a hard I/O fault when supervision is enabled).

struct JointHealth {
  double temp_c = 0.0;
  double volt_v = 0.0;
};
using HealthProvider =
    std::function<bool(std::array<JointHealth, model::kCanonicalDof>&)>;

// Pre-run gate (also the between-run cooldown rule): every servo cool
// and the supply nominal BEFORE any probe.
inline constexpr double kPreRunMaxTempC = 55.0;
inline constexpr double kPreRunMinVolt = 11.0;
inline constexpr double kPreRunMaxVolt = 13.0;
// Per-cycle hard-fault limits.
inline constexpr double kAbortTempC = 65.0;
inline constexpr double kAbortMinVolt = 10.5;
inline constexpr double kAbortMaxVolt = 14.0;

inline bool preRunHealthOk(
    const std::array<JointHealth, model::kCanonicalDof>& h,
    std::string* why = nullptr) {
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    if (h[i].temp_c > kPreRunMaxTempC || h[i].volt_v < kPreRunMinVolt ||
        h[i].volt_v > kPreRunMaxVolt) {
      if (why != nullptr) {
        char buf[96];
        std::snprintf(buf, sizeof buf,
                      "joint %d: %.1f degC, %.2f V outside pre-run gate", i,
                      h[i].temp_c, h[i].volt_v);
        *why = buf;
      }
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// Anchor identity: canonical hand-placed postures are only comparable
// when the settled anchor matches the reference within tolerance.

inline constexpr double kAnchorToleranceRad = 0.02;  // per joint

inline bool anchorWithinTolerance(const double* settled, const double* ref,
                                  double tol, double* deltas = nullptr) {
  bool ok = true;
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    const double d = settled[i] - ref[i];
    if (deltas != nullptr) deltas[i] = d;
    if (std::fabs(d) > tol) ok = false;
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Schedule: dwell list with per-dwell amplitudes and the worst-case
// base arithmetic the session budget is checked against.

struct DwellSpec {
  double freq_hz = 0.0;
  double amp_nm = 0.0;
  int window_mult = 1;  // predicted-SNR gate request (x4), granted at runtime
};

// Session budget constants (docs/IDENTIFICATION_PLAN.md deadline ladder).
inline constexpr double kTStopS = 177.5;        // graceful-stop threshold
inline constexpr double kTQuiesceS = 179.5;     // watchdog expiry
inline constexpr double kSetupAllowanceS = 15.0;  // pre-activation only
inline constexpr double kRampS = 0.5;
inline constexpr double kHoldMinS = 1.0;
inline constexpr double kHoldMaxS = 4.0;
inline constexpr double kKillRampS = 0.05;
inline constexpr double kReSettleS = 2.0;
inline constexpr double kFinalHoldS = 0.5;
inline constexpr double kAmpFloorNm = 0.05;
inline constexpr double kAmpCapHardNm = 0.3;
inline constexpr double kRespCapRad = 0.03;
inline constexpr double kGrowthRatio = 1.5;
inline constexpr double kDeviationAbortRad = 0.08;
inline constexpr double kHeadroomMarginNm = 0.2;
inline constexpr int kClipSoftCycles = 5;

// A(f) = clamp(x_t * J_hat * omega^2, floor, cap): sized for a target
// link response x_t = 0.005 rad, floored at ~10 torque LSB.
inline double probeAmplitude(double j_hat, double freq_hz,
                             double a_cap_nm = 0.15,
                             double x_t_rad = 0.005) {
  const double w = 2.0 * M_PI * freq_hz;
  return std::clamp(x_t_rad * j_hat * w * w, kAmpFloorNm,
                    std::min(a_cap_nm, kAmpCapHardNm));
}

// Default survey grid: dense through the 4-6 Hz structural band, the
// gear band around 13 Hz covered, 0.5-octave elsewhere.
inline const std::vector<double>& surveyGridHz() {
  static const std::vector<double> grid = {2.0,  3.0,  3.5, 4.0,  4.5,
                                           5.0,  5.5,  6.0, 7.0,  8.0,
                                           10.0, 12.0, 13.0, 14.0, 16.0,
                                           20.0};
  return grid;
}

// Analytic position-demod noise floor for a >= 3 s window (encoder
// quantization sigma ~4.4e-4 rad, dither-linearized) — the predicted-
// SNR gate's yardstick at schedule-build time.
inline constexpr double kWindowFloorRad = 3.6e-5;

// Diagonal inertia J_jj at the anchor from two inverseDynamics calls
// (unit angular acceleration on joint j minus the gravity-only term).
inline double diagInertia(model::ChainModel& chain,
                          const model::JointMap& map,
                          const model::ZVector& q_anchor, int joint) {
  model::ZVector zero(model::kCanonicalDof), ddq(model::kCanonicalDof);
  model::ZVector tau_g(model::kCanonicalDof), tau_a(model::kCanonicalDof);
  zVecElemNC(ddq.get(), joint) = 1.0;
  chain.inverseDynamics(map, q_anchor.get(), zero.get(), zero.get(),
                        tau_g.get());
  chain.inverseDynamics(map, q_anchor.get(), zero.get(), ddq.get(),
                        tau_a.get());
  return tau_a[joint] - tau_g[joint];
}

// Dwell list from a frequency grid with the amplitude rule; cap-bound
// dwells whose predicted response falls under 3x the window noise
// floor request the x4 window extension (granted at runtime only under
// the T_stop admission).
inline std::vector<DwellSpec> buildSchedule(
    const std::vector<double>& freqs_hz, double j_hat, double a_cap_nm) {
  std::vector<DwellSpec> dwells;
  for (const double f : freqs_hz) {
    DwellSpec d;
    d.freq_hz = f;
    d.amp_nm = probeAmplitude(j_hat, f, a_cap_nm);
    const double w = 2.0 * M_PI * f;
    const double x_hat = d.amp_nm / (j_hat * w * w);
    if (d.amp_nm >= std::min(a_cap_nm, kAmpCapHardNm) - 1e-9 &&
        x_hat < 3.0 * kWindowFloorRad) {
      d.window_mult = 4;
    }
    dwells.push_back(d);
  }
  return dwells;
}

// Anchor reference loader: accepts the per-dwell JSON sidecar (takes
// the 8 numbers after the "anchor" key) or any plain text holding 8
// numbers. Returns false unless exactly kCanonicalDof values found.
inline bool loadAnchorRef(const std::string& path, double* out) {
  std::FILE* f = std::fopen(path.c_str(), "r");
  if (!f) return false;
  std::string text;
  char buf[4096];
  std::size_t n;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) text.append(buf, n);
  std::fclose(f);
  std::size_t pos = 0;
  const auto key = text.find("\"anchor\"");
  if (key != std::string::npos) pos = key + 8;
  int found = 0;
  while (pos < text.size() && found < model::kCanonicalDof) {
    const char c = text[pos];
    if ((c >= '0' && c <= '9') ||
        ((c == '-' || c == '+') && pos + 1 < text.size() &&
         text[pos + 1] >= '0' && text[pos + 1] <= '9')) {
      char* end = nullptr;
      out[found++] = std::strtod(text.c_str() + pos, &end);
      pos = end - text.c_str();
    } else {
      ++pos;
    }
  }
  return found == model::kCanonicalDof;
}

// Lead-in sized to guarantee >= 4 non-overlapping 1-period noise-floor
// calibration blocks at the LOWEST scheduled frequency.
inline double leadInSeconds(const std::vector<DwellSpec>& dwells) {
  double f_min = 1e9;
  for (const auto& d : dwells) f_min = std::min(f_min, d.freq_hz);
  return std::max(2.0, dwells.empty() ? 2.0 : 4.0 / f_min);
}

// Measurement window: an integer number of probe periods, minimum
// max(10 periods, 3 s), ended at a 2*pi*n crossing by construction.
inline int windowPeriods(const DwellSpec& d) {
  return static_cast<int>(
      std::ceil(std::max(10.0, 3.0 * d.freq_hz) - 1e-9));
}
inline double windowSeconds(const DwellSpec& d, int mult = 1) {
  return mult * windowPeriods(d) / d.freq_hz;
}

// Worst-case BASE duration of one dwell: ramp-in + maximum adaptive
// hold + minimum window (extensions/retries are NOT base) + ramp-out.
inline double dwellWorstSeconds(const DwellSpec& d) {
  return kRampS + kHoldMaxS + windowSeconds(d) + kRampS;
}

// Worst-case BASE schedule: lead-in + every dwell at maximum hold +
// the final anchor tail, plus 10% slack.
inline double baseWorstSeconds(const std::vector<DwellSpec>& dwells) {
  double total = leadInSeconds(dwells) + kFinalHoldS;
  for (const auto& d : dwells) total += dwellWorstSeconds(d);
  return 1.10 * total;
}

// Pre-activation refusal: base schedule + the setup allowance must fit
// the graceful-stop threshold. The POST-SETTLE admission check against
// the real activation timestamp is the authoritative one.
inline bool scheduleFitsBudget(const std::vector<DwellSpec>& dwells,
                               double t_stop_s = kTStopS) {
  return baseWorstSeconds(dwells) + kSetupAllowanceS <= t_stop_s;
}

// ---------------------------------------------------------------------------
// Online demodulation: per-signal I/Q integrated over 1-period blocks
// whose boundaries are the accumulated-phase 2*pi crossings. The block
// estimate is Z = (2/T) * integral s(t) e^{-i phi} dt — the complex
// amplitude of the probe-frequency component.

struct BlockEstimate {
  double re = 0.0;
  double im = 0.0;
  double abs() const { return std::hypot(re, im); }
};

class BlockDemod {
 public:
  void reset() { *this = BlockDemod(); }
  // Accumulate one sample; returns true when a block just completed
  // (phase advanced 2*pi past the first sample after reset — the
  // reference latches on that sample, so a demod reset mid-dwell
  // blocks correctly from wherever the phase happens to be).
  bool add(double phase, double dt, double sample) {
    if (!primed_) {
      block_start_phase_ = phase;
      primed_ = true;
    }
    i_ += sample * std::sin(phase) * dt;
    q_ += sample * std::cos(phase) * dt;
    span_ += dt;
    if (phase - block_start_phase_ >= 2.0 * M_PI) {
      if (span_ > 0.0) {
        last_ = {2.0 * i_ / span_, 2.0 * q_ / span_};
        ++blocks_;
      }
      i_ = q_ = span_ = 0.0;
      block_start_phase_ += 2.0 * M_PI;
      return true;
    }
    return false;
  }
  int blocks() const { return blocks_; }
  const BlockEstimate& lastBlock() const { return last_; }

 private:
  bool primed_ = false;
  double i_ = 0.0, q_ = 0.0, span_ = 0.0;
  double block_start_phase_ = 0.0;
  BlockEstimate last_;
  int blocks_ = 0;
};

// Adaptive-hold convergence (round 4): two consecutive 1-period block
// estimates agree, normalized against a signal-specific floor, AND both
// clear the floor — two near-zero noisy blocks agreeing accidentally is
// not convergence.
inline bool blocksConverged(const BlockEstimate& prev,
                            const BlockEstimate& cur, double floor) {
  const double f = std::max(floor, 1e-12);
  if (prev.abs() < f || cur.abs() < f) return false;
  const double denom = std::max({prev.abs(), cur.abs(), f});
  return std::hypot(cur.re - prev.re, cur.im - prev.im) / denom < 0.1;
}

// ---------------------------------------------------------------------------
// Per-dwell outcome record (the JSON sidecar's substance).

struct DwellResult {
  DwellSpec spec;
  bool started = false;
  bool completed = false;
  bool skipped = false;         // admission or headroom refusal
  bool low_confidence = false;  // hold timeout or unmet SNR
  bool retried = false;
  int soft_events = 0;
  std::string note;
  double hold_s = 0.0;
  double window_s = 0.0;
  int window_periods = 0;
  // window-aggregate probe-frequency estimates
  BlockEstimate resp[model::kCanonicalDof];  // q per joint [rad]
  BlockEstimate tau_meas;                    // probe-joint torque [Nm]
  BlockEstimate tau_cmd;                     // commanded probe torque [Nm]
};

// ---------------------------------------------------------------------------
// The identification run: Controller + CycleObserver on one object
// (the TrackingRun pattern — controller state must survive the whole
// arm::run, and every telemetry row pairs its own receipt with the
// pre-write snapshot).

class IdentRun : public arm::Controller, public arm::CycleObserver {
 public:
  enum class Phase {
    LeadIn,
    RampIn,
    Hold,
    Measure,
    RampOut,
    KillRamp,
    ReSettle,
    FinalHold,
    Done
  };
  enum class Outcome {
    Running,
    Completed,      // all dwells processed (some may be skipped)
    DeadlineStop,   // graceful T_stop termination, rest not-run
    SoftAbort,      // second soft event on one dwell
    HardFault       // abort-then-deactivate, no retry
  };

  struct Options {
    int probe_joint = 1;
    std::vector<DwellSpec> dwells;
    std::vector<double> anchor;    // canonical rad, the settled posture
    std::vector<double> tau_max;   // hardware clamp mirror [Nm]
    // Session budget: elapsed activation->run-start seconds and the
    // graceful-stop threshold. Admission uses setup_offset_s + t.
    double setup_offset_s = 0.0;
    double t_stop_s = kTStopS;
    HealthProvider health;         // optional; empty = unsupervised (sim)
    // Sim twin / TwoMassArm ONLY (C6b): the fixture is gravity-free, so
    // the real model's anchor feedforward is a fictitious torque that
    // would drive it into the deviation abort — subtract it. NEVER on
    // hardware.
    bool gravity_free_plant = false;
  };

  IdentRun(model::ChainModel& chain, const model::JointMap& map,
           Options options, std::FILE* log)
      : options_(validated(std::move(options))),
        anchor_(model::kCanonicalDof),
        hold_traj_(makeAnchorVec(options_, anchor_),
                   anchor_.get(), 1.0),
        inner_(chain, map, hold_traj_, tuning::kKp, tuning::kKd),
        log_(log) {
    inner_.setIntegral(tuning::kKi, tuning::kIntegralClampNm);
    inner_.setGainScales(tuning::kGainScale);
    inner_.setNominalDt(tuning::kNominalDt);
    inner_.setPdFilterTau(tuning::kPdFilterTau);
    inner_.setTorqueLimits(options_.tau_max.data());
    results_.resize(options_.dwells.size());
    for (std::size_t k = 0; k < options_.dwells.size(); ++k) {
      results_[k].spec = options_.dwells[k];
    }
    lead_in_s_ = leadInSeconds(options_.dwells);
    // one floor calibrator per scheduled frequency, per signal
    floor_q_.resize(options_.dwells.size());
    floor_tau_.resize(options_.dwells.size());
    floor_phase_.assign(options_.dwells.size(), 0.0);
    floor_mags_q_.resize(options_.dwells.size());
    floor_mags_tau_.resize(options_.dwells.size());
  }

  // -- Controller ------------------------------------------------------

  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    inner_.update(state, cmd, t);
    if (options_.gravity_free_plant) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        zVecElemNC(cmd.tau.get(), i) -= inner_.feedforward()[i];
      }
    }
    if (t_prev_ < 0.0) {  // first sample: anchor feedforward only
      t_prev_ = t;
      probe_tau_ = 0.0;
      return;
    }
    const double dt = t - t_prev_;
    if (dt <= 0.0) {  // duplicate sample: re-emit the held command
      zVecElemNC(cmd.tau.get(), options_.probe_joint) = last_cmd_tau_;
      return;
    }
    t_prev_ = t;

    if (outcome_ == Outcome::Running) {
      supervise(state, t);
    }
    if (outcome_ == Outcome::Running || outcome_ == Outcome::DeadlineStop ||
        outcome_ == Outcome::SoftAbort) {
      advance(state, t, dt);
    }

    // Superpose the probe and re-clamp to the hardware limit; a clipped
    // sum is the probe's own soft-event signal, distinct from the inner
    // controller's saturation (which is a hard fault).
    const int pj = options_.probe_joint;
    const double base = zVecElemNC(cmd.tau.get(), pj);
    const double raw = base + probe_tau_;
    const double lim = options_.tau_max[pj];
    const double out = std::clamp(raw, -lim, lim);
    probe_clipped_ = out != raw;
    if (probe_clipped_) {
      ++clip_run_;
    } else {
      clip_run_ = 0;
    }
    zVecElemNC(cmd.tau.get(), pj) = out;
    last_cmd_tau_ = out;
  }

  // -- CycleObserver ---------------------------------------------------

  bool observe(double t, const arm::JointState& state,
               const arm::CommandSnapshot& cmds,
               const arm::JointCommand& cmd,
               const arm::CommandReceipt& receipt) override {
    (void)cmd;
    double first_apply_delay = std::nan("");
    if (!verifier_.check(state.t, cmds, receipt, &first_apply_delay)) {
      fail(Outcome::HardFault, "latency violation");
      return false;
    }
    // Hardware saturation or position-gate engagement on ANY joint is a
    // hard fault — except the probe joint's own kCmdClamped, which the
    // controller already accounts for as probe clipping.
    if (cmds.applied.valid && outcome_ == Outcome::Running) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        if (cmds.applied.flags[i] & arm::kCmdGated) {
          fail(Outcome::HardFault, "position gate engaged on joint " +
                                       std::to_string(i));
          return false;
        }
        if ((cmds.applied.flags[i] & arm::kCmdClamped) &&
            i != options_.probe_joint) {
          fail(Outcome::HardFault, "hardware clamp on joint " +
                                       std::to_string(i));
          return false;
        }
      }
    }
    if (log_ != nullptr) {
      writeRow(t, state, cmds, receipt, first_apply_delay);
    }
    if (outcome_ == Outcome::HardFault) return false;
    if (phase_ == Phase::Done) return false;  // ends arm::run by veto
    return true;
  }

  // -- results / telemetry --------------------------------------------

  Outcome outcome() const { return outcome_; }
  const std::string& faultReason() const { return fault_reason_; }
  Phase phase() const { return phase_; }
  int currentDwell() const { return dwell_; }
  const std::vector<DwellResult>& results() const { return results_; }
  double probeTau() const { return probe_tau_; }
  double probePhase() const { return phi_; }
  bool probeClipped() const { return probe_clipped_; }
  double leadIn() const { return lead_in_s_; }
  // calibrated floors (valid after lead-in), 3x RMS of block magnitudes
  double floorQ(int dwell) const {
    return dwell >= 0 && dwell < static_cast<int>(floor_q_val_.size())
               ? floor_q_val_[dwell]
               : 0.0;
  }
  double floorTau(int dwell) const {
    return dwell >= 0 && dwell < static_cast<int>(floor_tau_val_.size())
               ? floor_tau_val_[dwell]
               : 0.0;
  }
  const arm::ComputedTorque& inner() const { return inner_; }

  // True when run() ended by this object's own Done veto rather than a
  // fault: Completed and DeadlineStop are successful terminations.
  bool finishedCleanly() const {
    return phase_ == Phase::Done && (outcome_ == Outcome::Completed ||
                                     outcome_ == Outcome::DeadlineStop);
  }

 private:
  static Options validated(Options o) {
    if (static_cast<int>(o.anchor.size()) != model::kCanonicalDof ||
        static_cast<int>(o.tau_max.size()) != model::kCanonicalDof ||
        o.probe_joint < 0 || o.probe_joint >= model::kCanonicalDof ||
        o.dwells.empty()) {
      throw std::invalid_argument("IdentRun: bad options");
    }
    return o;
  }

  // Trajectory-before-inner initialization helper: copies the anchor
  // into `out` and returns its raw zVec for MinJerkTrajectory.
  static zVec makeAnchorVec(const Options& o, model::ZVector& out) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      zVecElemNC(out.get(), i) = o.anchor[i];
    }
    return out.get();
  }

  static void addNote(DwellResult& r, const std::string& text) {
    if (!r.note.empty()) r.note += "; ";
    r.note += text;
  }

  void fail(Outcome outcome, const std::string& reason) {
    if (outcome_ == Outcome::HardFault) return;
    outcome_ = outcome;
    fault_reason_ = reason;
    if (outcome == Outcome::HardFault) probe_tau_ = 0.0;
  }

  // Hard-fault supervision, every cycle regardless of phase.
  void supervise(const arm::JointState& state, double t) {
    (void)t;
    if (options_.health) {
      std::array<JointHealth, model::kCanonicalDof> h;
      if (!options_.health(h)) {
        fail(Outcome::HardFault, "health read failed");
        return;
      }
      last_health_ = h;
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        if (h[i].temp_c >= kAbortTempC) {
          fail(Outcome::HardFault,
               "thermal limit on joint " + std::to_string(i));
          return;
        }
        if (h[i].volt_v < kAbortMinVolt || h[i].volt_v > kAbortMaxVolt) {
          fail(Outcome::HardFault,
               "supply voltage out of range at joint " + std::to_string(i));
          return;
        }
      }
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double dev =
          std::fabs(zVecElemNC(state.q.get(), i) - options_.anchor[i]);
      if (dev >= kDeviationAbortRad) {
        fail(Outcome::HardFault, "anchor deviation " + std::to_string(dev) +
                                     " rad on joint " + std::to_string(i));
        return;
      }
      if (inner_.controllerSaturated(i)) {
        fail(Outcome::HardFault,
             "controller saturation on joint " + std::to_string(i));
        return;
      }
    }
  }

  // Worst-case BASE seconds of the dwells from index `first` on, plus
  // the final anchor tail. Extensions and retries are NOT included —
  // they are exactly what these admission bounds gate.
  double worstFrom(std::size_t first) const {
    double rem = kFinalHoldS;
    for (std::size_t k = first; k < options_.dwells.size(); ++k) {
      rem += dwellWorstSeconds(options_.dwells[k]);
    }
    return rem;
  }

  // Runtime admission against the ORIGINAL activation timestamp: the
  // requested work plus the rest of the base schedule at maximum holds
  // must still fit the graceful-stop threshold.
  bool admits(double t, double requested) const {
    return options_.setup_offset_s + t + requested <= options_.t_stop_s;
  }

  // The dwell state machine (documented in docs/IDENTIFICATION_PLAN.md).
  void advance(const arm::JointState& state, double t, double dt) {
    const int pj = options_.probe_joint;
    const double q_probe = zVecElemNC(state.q.get(), pj);
    const double tau_probe = zVecElemNC(state.tau.get(), pj);

    // Graceful deadline backstop: reaching the point where even an
    // immediate ramp-out would overrun T_stop forces termination NOW.
    // Only in the probe-carrying phases — the wind-down phases are
    // already covered by the admission bounds that let them start.
    if (outcome_ == Outcome::Running &&
        (phase_ == Phase::LeadIn || phase_ == Phase::RampIn ||
         phase_ == Phase::Hold || phase_ == Phase::Measure) &&
        options_.setup_offset_s + t + kRampS + kFinalHoldS >
            options_.t_stop_s) {
      fail(Outcome::DeadlineStop, "graceful stop at T_stop");
      markRestNotRun("not run: T_stop reached");
      enterKill(t, /*abort_after=*/false, /*to_final=*/true);
    }

    switch (phase_) {
      case Phase::LeadIn: {
        // noise-floor calibration on the UNFORCED anchor hold: the same
        // 1-period estimator at every scheduled frequency, both signals
        for (std::size_t k = 0; k < options_.dwells.size(); ++k) {
          floor_phase_[k] += 2.0 * M_PI * options_.dwells[k].freq_hz * dt;
          if (floor_q_[k].add(floor_phase_[k], dt, q_probe)) {
            floor_mags_q_[k].push_back(floor_q_[k].lastBlock().abs());
          }
          if (floor_tau_[k].add(floor_phase_[k], dt, tau_probe)) {
            floor_mags_tau_[k].push_back(floor_tau_[k].lastBlock().abs());
          }
        }
        // anchor torque estimate for the headroom precheck (slow filter
        // of |tau_meas| per joint over the lead-in)
        const double a = dt / (0.5 + dt);
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          tau_anchor_[i] +=
              a * (std::fabs(zVecElemNC(state.tau.get(), i)) -
                   tau_anchor_[i]);
        }
        if (t >= lead_in_s_) {
          finishFloors();
          startDwell(0, t);
        }
        break;
      }
      case Phase::RampIn: {
        phi_ += 2.0 * M_PI * currentSpec().freq_hz * dt;
        const double u =
            std::clamp((t - phase_start_) / kRampS, 0.0, 1.0);
        emitProbe(0.5 * (1.0 - std::cos(M_PI * u)));
        if (softEventCheck(t)) break;
        if (t - phase_start_ >= kRampS) {
          phase_ = Phase::Hold;
          hold_start_ = t;
          hold_demod_q_.reset();
          hold_demod_tau_.reset();
          have_prev_block_ = false;
          resetJointMonitors();
        }
        break;
      }
      case Phase::Hold: {
        phi_ += 2.0 * M_PI * currentSpec().freq_hz * dt;
        emitProbe(1.0);
        demodJoints(state, dt, /*growth_check=*/false);
        const bool q_block = hold_demod_q_.add(phi_, dt, q_probe);
        const bool tau_block = hold_demod_tau_.add(phi_, dt, tau_probe);
        if (softEventCheck(t)) break;
        if (q_block && tau_block) {
          const bool conv =
              have_prev_block_ &&
              blocksConverged(prev_q_, hold_demod_q_.lastBlock(),
                              floor_q_val_[dwell_]) &&
              blocksConverged(prev_tau_, hold_demod_tau_.lastBlock(),
                              floor_tau_val_[dwell_]);
          prev_q_ = hold_demod_q_.lastBlock();
          prev_tau_ = hold_demod_tau_.lastBlock();
          have_prev_block_ = true;
          if (conv && t - hold_start_ >= kHoldMinS) {
            startMeasure(t);
            break;
          }
        }
        if (t - hold_start_ >= kHoldMaxS) {
          results_[dwell_].low_confidence = true;
          addNote(results_[dwell_],
                  "hold timeout: transient/SNR unresolved");
          startMeasure(t);
        }
        break;
      }
      case Phase::Measure: {
        phi_ += 2.0 * M_PI * currentSpec().freq_hz * dt;
        emitProbe(1.0);
        demodJoints(state, dt, /*growth_check=*/true);
        auto& res = results_[dwell_];
        // block counter (period boundaries) + window-aggregate I/Q
        win_q_.add(phi_, dt, q_probe);
        win_tau_i_ += tau_probe * std::sin(phi_) * dt;
        win_tau_q_ += tau_probe * std::cos(phi_) * dt;
        win_cmd_i_ += probe_tau_ * std::sin(phi_) * dt;
        win_cmd_q_ += probe_tau_ * std::cos(phi_) * dt;
        accumWindow(state, dt);
        if (softEventCheck(t)) break;
        if (win_q_.blocks() >= windowPeriods(res.spec) * window_mult_) {
          finishMeasure(t);
        }
        break;
      }
      case Phase::RampOut: {
        phi_ += 2.0 * M_PI * currentSpec().freq_hz * dt;
        const double u =
            std::clamp((t - phase_start_) / kRampS, 0.0, 1.0);
        emitProbe(0.5 * (1.0 + std::cos(M_PI * u)));
        if (t - phase_start_ >= kRampS) {
          probe_tau_ = 0.0;
          nextDwell(t);
        }
        break;
      }
      case Phase::KillRamp: {
        // Emergency probe removal: 50 ms half-cosine from the envelope
        // at the event — <= 0.23 cycles at 4.5 Hz — never the ordinary
        // 0.5 s ramp, which would keep exciting a growing mode.
        phi_ += 2.0 * M_PI * currentSpec().freq_hz * dt;
        const double u =
            std::clamp((t - phase_start_) / kKillRampS, 0.0, 1.0);
        emitProbe(kill_env_ * 0.5 * (1.0 + std::cos(M_PI * u)));
        if (t - phase_start_ >= kKillRampS) {
          probe_tau_ = 0.0;
          if (kill_to_final_) {
            phase_ = Phase::FinalHold;
            phase_start_ = t;
          } else if (abort_after_kill_) {
            fail(Outcome::SoftAbort, soft_reason_);
            phase_ = Phase::Done;
          } else {
            phase_ = Phase::ReSettle;
            phase_start_ = t;
          }
        }
        break;
      }
      case Phase::ReSettle: {
        probe_tau_ = 0.0;
        if (t - phase_start_ >= kReSettleS) {
          auto& res = results_[dwell_];
          if (!res.retried &&
              admits(t, dwellWorstSeconds(res.spec) +
                            worstFrom(dwell_ + 1))) {
            // one reduced-amplitude retry
            res.retried = true;
            retry_amp_ = 0.5 * res.spec.amp_nm;
            restartDwell(t);
          } else {
            if (!res.retried) {
              addNote(res, "retry denied: T_stop admission");
            }
            nextDwell(t);
          }
        }
        break;
      }
      case Phase::FinalHold: {
        probe_tau_ = 0.0;
        if (t - phase_start_ >= kFinalHoldS) {
          if (outcome_ == Outcome::Running) outcome_ = Outcome::Completed;
          phase_ = Phase::Done;
        }
        break;
      }
      case Phase::Done:
        probe_tau_ = 0.0;
        break;
    }
  }

  // The LIVE spec (results_ copy): headroom reductions and analysis
  // notes land here; options_.dwells stays pristine for the worst-case
  // budget arithmetic.
  const DwellSpec& currentSpec() const { return results_[dwell_].spec; }

  void emitProbe(double envelope) {
    envelope_ = envelope;
    const double amp = retry_amp_ > 0.0 ? retry_amp_ : currentSpec().amp_nm;
    probe_tau_ = envelope * amp * std::sin(phi_);
  }

  void resetJointMonitors() {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      joint_demod_[i].reset();
      first_block_amp_[i] = -1.0;
    }
    clip_run_ = 0;
  }

  // ALL-JOINT online demodulated response monitors (the target is a
  // coherent whole-arm mode: excitation at one joint can produce the
  // larger response elsewhere). Cap and growth are SOFT events. The
  // growth check runs in the MEASURE window only, against its first
  // block: the adaptive hold guarantees steady state by then, whereas
  // the normal resonant ring-up during Hold would false-trigger it.
  void demodJoints(const arm::JointState& state, double dt,
                   bool growth_check) {
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (joint_demod_[i].add(phi_, dt, zVecElemNC(state.q.get(), i))) {
        const double amp = joint_demod_[i].lastBlock().abs();
        if (i == options_.probe_joint) resp_amp_est_ = amp;
        if (amp > kRespCapRad) {
          pend_soft_ = "response cap " + std::to_string(amp) +
                       " rad on joint " + std::to_string(i);
        } else if (!growth_check) {
          continue;
        } else if (first_block_amp_[i] < 0.0) {
          first_block_amp_[i] = amp;
        } else if (first_block_amp_[i] > 0.002 &&
                   amp > kGrowthRatio * first_block_amp_[i]) {
          pend_soft_ = "response growth on joint " + std::to_string(i);
        }
      }
    }
  }

  void accumWindow(const arm::JointState& state, double dt) {
    // per-joint window-aggregate I/Q for the participation vectors
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      win_joint_i_[i] +=
          zVecElemNC(state.q.get(), i) * std::sin(phi_) * dt;
      win_joint_q_[i] +=
          zVecElemNC(state.q.get(), i) * std::cos(phi_) * dt;
    }
    win_span_ += dt;
  }

  // Returns true when a soft event fired and the phase switched.
  bool softEventCheck(double t) {
    if (clip_run_ >= kClipSoftCycles && pend_soft_.empty()) {
      pend_soft_ = "probe clipped " + std::to_string(clip_run_) +
                   " consecutive cycles";
    }
    if (pend_soft_.empty()) return false;
    auto& res = results_[dwell_];
    ++res.soft_events;
    soft_reason_ = pend_soft_;
    pend_soft_.clear();
    addNote(res, soft_reason_);
    enterKill(t, /*abort_after=*/res.soft_events >= 2,
              /*to_final=*/false);
    return true;
  }

  void enterKill(double t, bool abort_after, bool to_final) {
    if (dwell_ < 0 || probe_tau_ == 0.0) {
      // no probe active (lead-in / between dwells): nothing to remove
      probe_tau_ = 0.0;
      phase_ = to_final ? Phase::FinalHold : Phase::ReSettle;
      phase_start_ = t;
      return;
    }
    kill_env_ = envelope_;
    abort_after_kill_ = abort_after;
    kill_to_final_ = to_final;
    phase_ = Phase::KillRamp;
    phase_start_ = t;
  }

  void startDwell(int k, double t) {
    dwell_ = k;
    if (dwell_ >= static_cast<int>(options_.dwells.size())) {
      phase_ = Phase::FinalHold;
      phase_start_ = t;
      return;
    }
    auto& res = results_[dwell_];
    // runtime admission: starting this dwell must leave it plus the
    // rest of the base schedule inside T_stop
    if (!admits(t, dwellWorstSeconds(res.spec) + worstFrom(dwell_ + 1))) {
      res.skipped = true;
      addNote(res, "not run: T_stop admission");
      if (outcome_ == Outcome::Running) {
        fail(Outcome::DeadlineStop, "graceful stop at T_stop");
        markRestNotRun("not run: T_stop reached");
      }
      phase_ = Phase::FinalHold;
      phase_start_ = t;
      return;
    }
    // ALL-JOINT headroom precheck against the measured anchor torque:
    // reduce the amplitude into the margin or skip the dwell.
    double amp = res.spec.amp_nm;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double headroom =
          options_.tau_max[i] - tau_anchor_[i] - kHeadroomMarginNm;
      amp = std::min(amp, headroom);
    }
    if (amp < kAmpFloorNm) {
      res.skipped = true;
      addNote(res, "skipped: torque headroom below the amplitude floor");
      nextDwell(t);
      return;
    }
    if (amp < res.spec.amp_nm) {
      addNote(res, "amplitude reduced to headroom");
      res.spec.amp_nm = amp;
    }
    // predicted-SNR window extension is granted only under admission
    window_mult_ = 1;
    if (res.spec.window_mult > 1) {
      const double extra =
          windowSeconds(res.spec, res.spec.window_mult) -
          windowSeconds(res.spec, 1);
      if (admits(t, dwellWorstSeconds(res.spec) + extra +
                        worstFrom(dwell_ + 1))) {
        window_mult_ = res.spec.window_mult;
      } else {
        res.low_confidence = true;
        addNote(res, "window extension denied: T_stop admission");
      }
    }
    retry_amp_ = 0.0;
    restartDwell(t);
  }

  void restartDwell(double t) {
    auto& res = results_[dwell_];
    res.started = true;
    phi_ = 0.0;
    phase_ = Phase::RampIn;
    phase_start_ = t;
    resetJointMonitors();
  }

  void startMeasure(double t) {
    auto& res = results_[dwell_];
    res.hold_s = t - hold_start_;
    measure_start_ = t;
    phase_ = Phase::Measure;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      first_block_amp_[i] = -1.0;  // growth reference = first MEASURE block
    }
    win_q_.reset();
    win_tau_i_ = win_tau_q_ = win_cmd_i_ = win_cmd_q_ = 0.0;
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      win_joint_i_[i] = win_joint_q_[i] = 0.0;
    }
    win_span_ = 0.0;
  }

  void finishMeasure(double t) {
    auto& res = results_[dwell_];
    res.completed = true;
    res.window_s = t - measure_start_;
    res.window_periods = win_q_.blocks();
    if (win_span_ > 0.0) {
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        res.resp[i] = {2.0 * win_joint_i_[i] / win_span_,
                       2.0 * win_joint_q_[i] / win_span_};
      }
    }
    if (win_span_ > 0.0) {
      res.tau_meas = {2.0 * win_tau_i_ / win_span_,
                      2.0 * win_tau_q_ / win_span_};
      res.tau_cmd = {2.0 * win_cmd_i_ / win_span_,
                     2.0 * win_cmd_q_ / win_span_};
    }
    phase_ = Phase::RampOut;
    phase_start_ = t;
  }

  void nextDwell(double t) {
    retry_amp_ = 0.0;
    startDwell(dwell_ + 1, t);
  }

  void markRestNotRun(const std::string& why) {
    for (std::size_t k = dwell_ + 1; k < results_.size(); ++k) {
      if (!results_[k].started) {
        results_[k].skipped = true;
        results_[k].note = why;
      }
    }
  }

  void finishFloors() {
    floor_q_val_.assign(options_.dwells.size(), 0.0);
    floor_tau_val_.assign(options_.dwells.size(), 0.0);
    for (std::size_t k = 0; k < options_.dwells.size(); ++k) {
      floor_q_val_[k] = 3.0 * rms(floor_mags_q_[k]);
      floor_tau_val_[k] = 3.0 * rms(floor_mags_tau_[k]);
    }
  }

  static double rms(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s = 0.0;
    for (const double x : v) s += x * x;
    return std::sqrt(s / v.size());
  }

  void writeRow(double t, const arm::JointState& state,
                const arm::CommandSnapshot& cmds,
                const arm::CommandReceipt& receipt,
                double first_apply_delay) {
    const auto& ap = cmds.applied;
    const auto& at = cmds.last_attempt;
    std::fprintf(log_, "%d,%.4f,%.6f,%llu,%llu,%.6f",
                 static_cast<int>(phase_), t, state.t,
                 static_cast<unsigned long long>(state.seq),
                 static_cast<unsigned long long>(receipt.submitted_seq),
                 receipt.submission_time);
    std::fprintf(log_, ",%d,%llu,%.6f,%llu,%.6f,%llu,%.6f,%.6f",
                 ap.valid ? 1 : 0,
                 static_cast<unsigned long long>(ap.target_seq),
                 ap.first_time,
                 static_cast<unsigned long long>(ap.first_cycle),
                 ap.latest_time,
                 static_cast<unsigned long long>(ap.latest_cycle),
                 first_apply_delay, state.t - ap.latest_time);
    std::fprintf(log_, ",%d,%llu,%.6f,%d", at.valid ? 1 : 0,
                 static_cast<unsigned long long>(at.target_seq), at.time,
                 at.ok ? 1 : 0);
    const double amp =
        dwell_ >= 0 && dwell_ < static_cast<int>(options_.dwells.size())
            ? (retry_amp_ > 0.0 ? retry_amp_ : currentSpec().amp_nm)
            : 0.0;
    std::fprintf(log_, ",%d,%d,%d,%.4f,%.4f,%.6f,%.5f,%d,%.6f", dwell_,
                 static_cast<int>(phase_), options_.probe_joint,
                 dwell_ >= 0 &&
                         dwell_ < static_cast<int>(options_.dwells.size())
                     ? currentSpec().freq_hz
                     : 0.0,
                 amp, phi_, probe_tau_, probe_clipped_ ? 1 : 0,
                 resp_amp_est_);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(
          log_,
          ",%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%d,%d,"
          "%.4f,%.1f,%.2f",
          options_.anchor[i], zVecElemNC(state.q.get(), i),
          zVecElemNC(state.dq.get(), i), inner_.velocityEstimate()[i],
          inner_.feedforward()[i], inner_.pdTerm()[i],
          inner_.integralTerm()[i], inner_.tauRaw()[i],
          inner_.tauCommanded()[i], inner_.controllerSaturated(i) ? 1 : 0,
          ap.applied[i], (ap.flags[i] & arm::kCmdClamped) ? 1 : 0,
          (ap.flags[i] & arm::kCmdGated) ? 1 : 0,
          zVecElemNC(state.tau.get(), i), last_health_[i].temp_c,
          last_health_[i].volt_v);
    }
    std::fprintf(log_, "\n");
  }

  Options options_;
  model::ZVector anchor_;
  model::MinJerkTrajectory hold_traj_;
  arm::ComputedTorque inner_;
  std::FILE* log_;
  LatencyVerifier verifier_;

  Outcome outcome_ = Outcome::Running;
  std::string fault_reason_;
  Phase phase_ = Phase::LeadIn;
  int dwell_ = -1;
  double t_prev_ = -1.0;
  double phase_start_ = 0.0;
  double hold_start_ = 0.0;
  double measure_start_ = 0.0;
  double lead_in_s_ = 2.0;
  double phi_ = 0.0;
  double envelope_ = 0.0;
  double probe_tau_ = 0.0;
  double last_cmd_tau_ = 0.0;
  double retry_amp_ = 0.0;
  double kill_env_ = 0.0;
  bool abort_after_kill_ = false;
  bool kill_to_final_ = false;
  bool probe_clipped_ = false;
  int clip_run_ = 0;
  int window_mult_ = 1;
  double resp_amp_est_ = 0.0;
  std::string pend_soft_;
  std::string soft_reason_;

  // adaptive-hold convergence state
  BlockDemod hold_demod_q_, hold_demod_tau_;
  BlockEstimate prev_q_, prev_tau_;
  bool have_prev_block_ = false;

  // measurement-window state
  BlockDemod win_q_;
  double win_tau_i_ = 0.0, win_tau_q_ = 0.0;
  double win_cmd_i_ = 0.0, win_cmd_q_ = 0.0;
  double win_joint_i_[model::kCanonicalDof] = {};
  double win_joint_q_[model::kCanonicalDof] = {};
  double win_span_ = 0.0;

  // all-joint monitors
  BlockDemod joint_demod_[model::kCanonicalDof];
  double first_block_amp_[model::kCanonicalDof] = {};
  double tau_anchor_[model::kCanonicalDof] = {};

  // noise-floor calibration (lead-in)
  std::vector<BlockDemod> floor_q_, floor_tau_;
  std::vector<double> floor_phase_;
  std::vector<std::vector<double>> floor_mags_q_, floor_mags_tau_;
  std::vector<double> floor_q_val_, floor_tau_val_;

  std::array<JointHealth, model::kCanonicalDof> last_health_{};
  std::vector<DwellResult> results_;
};

// ---------------------------------------------------------------------------
// Ident CSV: the full D8 telemetry block (feedback / this cycle's
// request / latest applied) plus the probe columns and per-joint
// health. `meta` lines (label, anchor, planned schedule) are appended
// to the '#' comment by the caller before the header row.

inline std::FILE* openIdentCsvLog(const std::string& path,
                                  const std::string& meta) {
  std::FILE* log = std::fopen(path.c_str(), "w");
  if (!log) return nullptr;
  std::fprintf(
      log,
      "# events per row: FEEDBACK (feedback_seq/feedback_time, q, "
      "dq_servo, tau_meas) | THIS CYCLE'S REQUEST (submitted_seq/"
      "submission_time; applied LATER) | LATEST APPLIED (applied_*, "
      "tau_applied/hsat/gate from the latest successful transmission; "
      "first_apply_* preserved across retransmissions). "
      "first_apply_delay = first_apply_time - MATCHED submission_time "
      "(receipt-map join; NaN for the pre-run baseline). probe_* "
      "columns describe THIS cycle's excitation; resp_amp_est is the "
      "latest completed 1-period demod block on the probe joint. "
      "%s\n",
      meta.c_str());
  std::fprintf(log,
               "leg,control_t,feedback_time,feedback_seq,submitted_seq,"
               "submission_time,applied_valid,applied_seq,"
               "first_apply_time,first_apply_cycle,latest_apply_time,"
               "latest_apply_cycle,first_apply_delay,"
               "feedback_minus_latest_apply,attempt_valid,attempt_seq,"
               "attempt_time,attempt_ok,dwell_id,dwell_phase,"
               "probe_joint,probe_hz,probe_amp,probe_phase,probe_tau,"
               "probe_clipped,resp_amp_est");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(log,
                 ",qd%d,q%d,dq%d,dqest%d,ff%d,pd%d,i%d,tauraw%d,"
                 "taucmd%d,csat%d,tauapp%d,hsat%d,gate%d,taumeas%d,"
                 "temp%d,volt%d",
                 i, i, i, i, i, i, i, i, i, i, i, i, i, i, i, i);
  }
  std::fprintf(log, "\n");
  return log;
}

// Per-dwell JSON summary sidecar: online estimates, verdicts, and the
// anchor vector — what the analysis needs to merge runs legitimately.
inline bool writeDwellJson(const std::string& path,
                           const IdentRun::Options& options,
                           const IdentRun& run) {
  std::FILE* f = std::fopen(path.c_str(), "w");
  if (!f) return false;
  std::fprintf(f, "{\n  \"probe_joint\": %d,\n  \"anchor\": [",
               options.probe_joint);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(f, "%s%.6f", i ? ", " : "", options.anchor[i]);
  }
  std::fprintf(f, "],\n  \"outcome\": %d,\n  \"fault\": \"%s\",\n",
               static_cast<int>(run.outcome()),
               run.faultReason().c_str());
  std::fprintf(f, "  \"dwells\": [\n");
  const auto& results = run.results();
  for (std::size_t k = 0; k < results.size(); ++k) {
    const auto& r = results[k];
    std::fprintf(
        f,
        "    {\"freq_hz\": %.4f, \"amp_nm\": %.4f, \"started\": %s, "
        "\"completed\": %s, \"skipped\": %s, \"low_confidence\": %s, "
        "\"retried\": %s, \"soft_events\": %d, \"hold_s\": %.3f, "
        "\"window_s\": %.3f, \"window_periods\": %d, "
        "\"floor_q\": %.3e, \"floor_tau\": %.3e, "
        "\"note\": \"%s\",\n     \"resp\": [",
        r.spec.freq_hz, r.spec.amp_nm, r.started ? "true" : "false",
        r.completed ? "true" : "false", r.skipped ? "true" : "false",
        r.low_confidence ? "true" : "false", r.retried ? "true" : "false",
        r.soft_events, r.hold_s, r.window_s, r.window_periods,
        run.floorQ(static_cast<int>(k)), run.floorTau(static_cast<int>(k)),
        r.note.c_str());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(f, "%s[%.6e, %.6e]", i ? ", " : "", r.resp[i].re,
                   r.resp[i].im);
    }
    std::fprintf(f,
                 "],\n     \"tau_meas\": [%.6e, %.6e], "
                 "\"tau_cmd\": [%.6e, %.6e]}%s\n",
                 r.tau_meas.re, r.tau_meas.im, r.tau_cmd.re, r.tau_cmd.im,
                 k + 1 < results.size() ? "," : "");
  }
  std::fprintf(f, "  ]\n}\n");
  std::fclose(f);
  return true;
}

}  // namespace x7
