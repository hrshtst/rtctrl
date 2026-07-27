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
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "latency_verifier.hpp"
#include "quiescence_metric.hpp"
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
// link response x_t = 0.005 rad, floored at ~10 torque LSB. The
// 0.3 Nm hard cap is UNBYPASSABLE: a non-finite or out-of-range cap
// request degrades to the hard cap (std::clamp does not sanitize NaN
// — a "--amp nan" once scheduled 3.8 Nm; review finding).
inline double probeAmplitude(double j_hat, double freq_hz,
                             double a_cap_nm = 0.15,
                             double x_t_rad = 0.005) {
  const double cap =
      std::isfinite(a_cap_nm) && a_cap_nm > 0.0
          ? std::min(a_cap_nm, kAmpCapHardNm)
          : kAmpCapHardNm;
  const double w = 2.0 * M_PI * freq_hz;
  const double a = x_t_rad * j_hat * w * w;
  if (!std::isfinite(a)) return kAmpFloorNm;
  return std::clamp(a, kAmpFloorNm, cap);
}

// Strict "--amp" parsing: the whole token must be one finite number in
// [floor, hard cap] — atof-plus-clamp let NaN through (review finding).
inline bool parseAmpCap(const char* text, double* out) {
  char* end = nullptr;
  const double a = std::strtod(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(a) ||
      a < kAmpFloorNm || a > kAmpCapHardNm) {
    return false;
  }
  *out = a;
  return true;
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

// CLI frequency list: comma-separated "f" or "f@amp" entries — the
// optional per-dwell amplitude override is how the refinement up-sweep
// carries its HALF-amplitude peak repeat (the linearity spot-check)
// inside one invocation, e.g. "--freqs 3.9,4.2,4.5,4.8,4.5@0.075".
// STRICT: any malformed entry, non-finite value, or frequency outside
// [0.5, 30] Hz rejects the WHOLE list — a silently truncated schedule
// (e.g. an unreplaced "<peak>@<half-amp>" placeholder) must refuse to
// run, not proceed without its dwells (review finding).
struct FreqSpec {
  double freq_hz = 0.0;
  double amp_override_nm = 0.0;  // <= 0: the amplitude rule applies
};

inline constexpr double kFreqMinHz = 0.5;
// The drift-immune block estimator needs >= 5 samples per period at
// the 100 Hz loop, so 20 Hz (the survey-grid maximum) is the true
// ceiling — 30 Hz gave 3-4 samples per block and the measurement
// window never completed (review finding).
inline constexpr double kFreqMaxHz = 20.0;

inline bool parseFreqList(const char* text, std::vector<FreqSpec>* out) {
  out->clear();
  const char* p = text;
  while (*p != '\0') {
    char* end = nullptr;
    const double f = std::strtod(p, &end);
    if (end == p || !std::isfinite(f) || f < kFreqMinHz ||
        f > kFreqMaxHz) {
      return false;
    }
    FreqSpec spec;
    spec.freq_hz = f;
    p = end;
    if (*p == '@') {
      spec.amp_override_nm = std::strtod(p + 1, &end);
      if (end == p + 1 || !std::isfinite(spec.amp_override_nm) ||
          spec.amp_override_nm <= 0.0) {
        return false;
      }
      p = end;
    }
    out->push_back(spec);
    if (*p == ',') {
      ++p;
      if (*p == '\0') return false;  // trailing comma
    } else if (*p != '\0') {
      return false;  // trailing garbage
    }
  }
  return !out->empty();
}

// Schedule from CLI specs: the amplitude rule, then any per-dwell
// overrides (clamped to the same floor/caps).
inline std::vector<DwellSpec> buildScheduleFromSpecs(
    const std::vector<FreqSpec>& specs, double j_hat, double a_cap_nm) {
  std::vector<double> freqs;
  for (const auto& s : specs) freqs.push_back(s.freq_hz);
  auto dwells = buildSchedule(freqs, j_hat, a_cap_nm);
  for (std::size_t k = 0; k < specs.size(); ++k) {
    if (specs[k].amp_override_nm > 0.0) {
      dwells[k].amp_nm = std::clamp(
          specs[k].amp_override_nm, kAmpFloorNm,
          std::min(a_cap_nm, kAmpCapHardNm));
    }
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
// Online demodulation: per-signal dt-weighted least squares on the
// regressors [1, t, sin phi, cos phi] over 1-period blocks bounded by
// the accumulated-phase 2*pi crossings. The LS solve — not a plain
// correlation — is essential: the constant and drift regressors absorb
// the absolute joint position exactly, whereas correlating raw
// positions lets the offset leak through the sampled (non-exact)
// period boundary — at the P1 posture's q4 = -2.456 rad a still joint
// read ~0.096 rad, three times the response cap (review finding).

struct BlockEstimate {
  double re = 0.0;  // sin-phase coefficient
  double im = 0.0;  // cos-phase coefficient
  double abs() const { return std::hypot(re, im); }
};

// Accumulates the 4x4 normal equations; solve() extracts the complex
// probe-frequency amplitude with offset and linear drift removed.
class LsAccum {
 public:
  void reset() { *this = LsAccum(); }
  void add(double t, double phase, double dt, double y) {
    const double r[4] = {1.0, t, std::sin(phase), std::cos(phase)};
    for (int i = 0; i < 4; ++i) {
      for (int j = i; j < 4; ++j) m_[i][j] += r[i] * r[j] * dt;
      b_[i] += r[i] * y * dt;
    }
    ++n_;
  }
  int samples() const { return n_; }
  bool solve(BlockEstimate* z) const {
    double a[4][5];
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        a[i][j] = m_[std::min(i, j)][std::max(i, j)];
      }
      a[i][4] = b_[i];
    }
    for (int col = 0; col < 4; ++col) {  // gaussian, partial pivoting
      int piv = col;
      for (int r = col + 1; r < 4; ++r) {
        if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
      }
      if (std::fabs(a[piv][col]) < 1e-15) return false;
      for (int j = 0; j < 5; ++j) std::swap(a[col][j], a[piv][j]);
      for (int r = col + 1; r < 4; ++r) {
        const double f = a[r][col] / a[col][col];
        for (int j = col; j < 5; ++j) a[r][j] -= f * a[col][j];
      }
    }
    double x[4];
    for (int i = 3; i >= 0; --i) {
      double s = a[i][4];
      for (int j = i + 1; j < 4; ++j) s -= a[i][j] * x[j];
      x[i] = s / a[i][i];
    }
    *z = {x[2], x[3]};
    return true;
  }

 private:
  double m_[4][4] = {};
  double b_[4] = {};
  int n_ = 0;
};

class BlockDemod {
 public:
  void reset() { *this = BlockDemod(); }
  // Accumulate one sample; returns true when a block just completed.
  // Blocks are HALF-OPEN [start, next 2*pi crossing): the crossing
  // sample closes the old block and opens the new one — it is counted
  // exactly once (double-counting the boundary was part of the offset
  // leak). The phase reference latches on the first sample after
  // reset, so a demod reset mid-dwell blocks correctly from wherever
  // the phase happens to be.
  bool add(double phase, double dt, double sample) {
    if (!primed_) {
      block_start_phase_ = phase;
      primed_ = true;
    }
    bool completed = false;
    if (phase - block_start_phase_ >= 2.0 * M_PI) {
      if (accum_.samples() >= 5 && accum_.solve(&last_)) {
        ++blocks_;
        completed = true;
      }
      accum_.reset();
      t_rel_ = 0.0;
      block_start_phase_ += 2.0 * M_PI;
    }
    t_rel_ += dt;
    accum_.add(t_rel_, phase, dt, sample);
    return completed;
  }
  int blocks() const { return blocks_; }
  const BlockEstimate& lastBlock() const { return last_; }

 private:
  bool primed_ = false;
  LsAccum accum_;
  double t_rel_ = 0.0;
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
// Capture-phase constants (reviewer-approved integrated startup): the
// pre-lead-in capture pulls a SMALL post-transition displacement onto
// the canonical anchor with the shipped restoring controller; a
// displacement beyond the envelope aborts rather than being pulled
// through.
inline constexpr double kCaptureEnvelopeRad = 0.08;  // = deviation abort
inline constexpr double kCaptureSpeedRadS = 0.05;    // ramp peak speed
inline constexpr double kCaptureRampMinS = 1.0;
inline constexpr double kCaptureRampMaxS = 5.0;
inline constexpr double kCaptureHoldMaxS = 10.0;  // quiescence timeout
inline constexpr double kCaptureWorstS = 20.0;    // budget pre-check add-on
// Hold-diagnostic enforcement grace: capture acceptance means the
// metric sat just under 0.05 rad/s for 0.3 s, so a marginal entry can
// blip a fraction of a millirad-per-second over the threshold moments
// later while the last transient finishes decaying — an acceptance-
// boundary artifact, not instability. Continuous enforcement begins
// after this grace; the per-joint statistics still record the WHOLE
// hold, so a grace-period excursion remains visible in the report.
inline constexpr double kHoldDiagGraceS = 1.0;
// The integrated flow has NO settle phase — setup between current-mode
// activation and run start is the thread start, the confirmed hold and
// the gates (~1-2 s), not the hand-placed flow's 10 s settle + gates.
// With the 15 s allowance the DEFAULT survey + capture arithmetic
// rejected itself: 146.5 + 20 + 15 = 181.5 > 177.5 (review finding).
inline constexpr double kPoseFirstSetupAllowanceS = 5.0;

// The capture reference: a constant anchor until beginCapture()
// installs a min-jerk segment from the MEASURED start posture onto the
// anchor; reverts to the constant anchor when the segment ends. The
// identification controller tracks THIS trajectory for its whole life,
// so the capture, the gates, the lead-in and the probe all share one
// controller instance with continuous integrator state (reviewer
// correction: a controller swap after the gate would step the torque).
class AnchorCaptureTrajectory : public model::Trajectory {
 public:
  explicit AnchorCaptureTrajectory(const zVec anchor)
      : anchor_(model::kCanonicalDof) {
    zVecCopyNC(anchor, anchor_.get());
  }

  double duration() const override { return 1.0; }  // nominal; unused
  int size() const override { return model::kCanonicalDof; }

  void beginCapture(const zVec q0, double t0, double duration) {
    seg_.emplace(q0, anchor_.get(), duration);
    t0_ = t0;
  }
  double captureEnd() const {
    return seg_ ? t0_ + seg_->duration() : 0.0;
  }

  void sample(double t, zVec q, zVec dq = nullptr,
              zVec ddq = nullptr) const override {
    if (seg_ && t >= t0_ && t < t0_ + seg_->duration()) {
      seg_->sample(t - t0_, q, dq, ddq);
      return;
    }
    zVecCopyNC(anchor_.get(), q);
    if (dq != nullptr) zVecZero(dq);
    if (ddq != nullptr) zVecZero(ddq);
  }

 private:
  model::ZVector anchor_;
  std::optional<model::MinJerkTrajectory> seg_;
  double t0_ = 0.0;
};

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
  BlockEstimate tau_meas;  // measured probe-joint torque [Nm]
  BlockEstimate tau_cmd;   // SUBMITTED TOTAL probe-joint torque [Nm]
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
    Done,
    Capture,  // appended: the CSV's dwell_phase numbering is frozen
              // (Measure == 3 in tools/ident_analysis.py)
    HoldDiag  // unforced-hold diagnostic (stabilization procedure)
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
    // Integrated startup (reviewer-approved): when > 0 the run begins
    // with a CAPTURE phase — a bounded min-jerk reference from the
    // MEASURED first-sample posture onto the anchor under the shipped
    // restoring controller, then the quiescence metric and the
    // +/-0.02 rad anchor gate evaluated UNDER that hold — before any
    // calibration or probing. A first-sample displacement beyond this
    // envelope hard-faults (abort, never pull through). 0 disables:
    // the hand-placed flow starts at the anchor as before.
    double capture_envelope_rad = 0.0;
    // Unforced-hold DIAGNOSTIC mode (stabilization procedure): when
    // > 0 the run holds the anchor for this long after capture
    // acceptance — no calibration, no probing (dwells must be EMPTY;
    // requires the capture) — under CONTINUOUS-stability requirements:
    // every joint within the +/-0.02 anchor tolerance and the
    // quiescence metric below 0.05 rad/s at EVERY sample, all hard
    // monitors live; any violation fails. Per-joint maxima are
    // reported for the tuning decision.
    double hold_only_s = 0.0;
    // Diagnostic-scoped joint-0 PD gain-scale override (0 = shipped
    // kGainScale). Restricted to the hold diagnostic by the app: the
    // FRFs are CONTROLLER-SPECIFIC (a multivariable arm: other joints'
    // coherent feedback torques are coupled inputs the primary
    // estimator's denominator does not contain), so the campaign runs
    // one fixed, recorded tuning — never a per-run knob.
    double hold_scale0 = 0.0;
    // EXPERIMENTAL (reviewer-directed, diagnostic-only): freeze the
    // learned integral state at capture acceptance — without
    // resetting — preserving the captured friction/gravity bias and
    // torque continuity while preventing the integrator-stiction
    // hunting (a stuck in-tolerance error otherwise winds at Ki*e
    // until breakaway). Recorded in the sidecar tuning block.
    bool freeze_integral_at_capture = false;
  };

  IdentRun(model::ChainModel& chain, const model::JointMap& map,
           Options options, std::FILE* log)
      : options_(validated(std::move(options))),
        anchor_(model::kCanonicalDof),
        hold_traj_(makeAnchorVec(options_, anchor_)),
        inner_(chain, map, hold_traj_, tuning::kKp, tuning::kKd),
        log_(log) {
    inner_.setIntegral(tuning::kKi, tuning::kIntegralClampNm);
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      gain_scale_[i] = tuning::kGainScale[i];
    }
    if (options_.hold_scale0 > 0.0) {
      gain_scale_[0] = options_.hold_scale0;
    }
    inner_.setGainScales(gain_scale_);
    inner_.setNominalDt(tuning::kNominalDt);
    inner_.setPdFilterTau(tuning::kPdFilterTau);
    inner_.setTorqueLimits(options_.tau_max.data());
    results_.resize(options_.dwells.size());
    for (std::size_t k = 0; k < options_.dwells.size(); ++k) {
      results_[k].spec = options_.dwells[k];
    }
    if (options_.capture_envelope_rad > 0.0) phase_ = Phase::Capture;
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
    if (phase_ == Phase::Capture && !capture_begun_) {
      // First controlled sample: install the ramped reference FROM THE
      // MEASURED posture (so the very first feedforward is computed at
      // the real pose), then admit or abort. The envelope rule is the
      // reviewer's: capture closes a small expected residual — it must
      // never pull through an arbitrary displacement.
      capture_begun_ = true;
      double worst = 0.0;
      int worst_j = 0;
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        const double dev =
            std::fabs(zVecElemNC(state.q.get(), i) - options_.anchor[i]);
        if (dev > worst) {
          worst = dev;
          worst_j = i;
        }
      }
      capture_initial_dev_ = worst;
      const double ramp = std::clamp(
          worst * 1.875 / kCaptureSpeedRadS, kCaptureRampMinS,
          kCaptureRampMaxS);
      hold_traj_.beginCapture(state.q.get(), t, ramp);
      if (worst > options_.capture_envelope_rad) {
        fail(Outcome::HardFault,
             "capture admission: joint " + std::to_string(worst_j) +
                 " is " + std::to_string(worst) +
                 " rad from the anchor (envelope " +
                 std::to_string(options_.capture_envelope_rad) + ")");
      }
    }
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
    // the commanded-side window estimate demodulates the SUBMITTED
    // total, available only here after the probe sum and re-clamp
    if (phase_ == Phase::Measure) {
      win_cmd_.add(t - measure_start_, phi_, dt, out);
    }
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
    // Hardware saturation or position-gate engagement on ANY joint is
    // a hard fault. The probe joint's own kCmdClamped is exempt ONLY
    // while a probe can actually be active (the controller accounts
    // for it as probe clipping): in Capture, HoldDiag, LeadIn,
    // ReSettle and FinalHold there is no probe, so a clamp there is a
    // genuine anomaly on every joint (review finding).
    if (cmds.applied.valid && outcome_ == Outcome::Running) {
      const bool probe_active =
          phase_ == Phase::RampIn || phase_ == Phase::Hold ||
          phase_ == Phase::Measure || phase_ == Phase::RampOut ||
          phase_ == Phase::KillRamp;
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        if (cmds.applied.flags[i] & arm::kCmdGated) {
          fail(Outcome::HardFault, "position gate engaged on joint " +
                                       std::to_string(i));
          return false;
        }
        if ((cmds.applied.flags[i] & arm::kCmdClamped) &&
            !(probe_active && i == options_.probe_joint)) {
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
  // capture telemetry (integrated startup only)
  double captureInitialDev() const { return capture_initial_dev_; }
  double captureAcceptedAt() const { return capture_s_; }
  // hold-diagnostic telemetry: whole-hold vs ENFORCED-period maxima
  // (the whole-hold figure includes the acceptance-boundary grace;
  // pass/fail judgments belong to the enforced figure)
  double holdCompletedS() const { return hold_done_s_; }
  double holdMaxSpeed(int i) const { return hold_max_speed_[i]; }
  double holdMaxSpeedEnforced(int i) const {
    return hold_max_speed_enf_[i];
  }
  double holdRangeRad(int i) const {
    return hold_q_max_[i] - hold_q_min_[i];
  }
  // the EFFECTIVE gain-scale vector (shipped + any diagnostic
  // override) — recorded so the analysis can refuse to merge runs
  // under different controllers (the FRFs are controller-specific)
  const double* gainScales() const { return gain_scale_; }
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
        o.probe_joint < 0 || o.probe_joint >= model::kCanonicalDof) {
      throw std::invalid_argument("IdentRun: bad options");
    }
    if (o.hold_only_s > 0.0) {
      // the diagnostic is explicit: no dwells, and it needs the
      // capture phase (never faked with a dummy dwell)
      if (!o.dwells.empty() || o.capture_envelope_rad <= 0.0) {
        throw std::invalid_argument(
            "IdentRun: hold diagnostic requires EMPTY dwells and the "
            "capture phase");
      }
    } else if (o.dwells.empty()) {
      throw std::invalid_argument("IdentRun: no dwells");
    }
    if (o.hold_scale0 != 0.0 && o.hold_only_s <= 0.0) {
      throw std::invalid_argument(
          "IdentRun: the scale override is diagnostic-only");
    }
    if (o.freeze_integral_at_capture && o.hold_only_s <= 0.0) {
      throw std::invalid_argument(
          "IdentRun: the integral freeze is diagnostic-only");
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
    // During Capture the deviation bound applies to the TRACKING error
    // against the moving capture reference (the distance to the anchor
    // is legitimately up to the envelope there); everywhere else it is
    // the anchor deviation as before.
    if (phase_ == Phase::Capture) {
      hold_traj_.sample(t, ref_scratch_.get());
    }
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      const double ref = phase_ == Phase::Capture
                             ? zVecElemNC(ref_scratch_.get(), i)
                             : options_.anchor[i];
      const double dev = std::fabs(zVecElemNC(state.q.get(), i) - ref);
      if (dev >= kDeviationAbortRad) {
        fail(Outcome::HardFault,
             std::string(phase_ == Phase::Capture ? "capture tracking"
                                                  : "anchor") +
                 " deviation " + std::to_string(dev) + " rad on joint " +
                 std::to_string(i));
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
        (phase_ == Phase::Capture || phase_ == Phase::HoldDiag ||
         phase_ == Phase::LeadIn || phase_ == Phase::RampIn ||
         phase_ == Phase::Hold || phase_ == Phase::Measure) &&
        options_.setup_offset_s + t + kRampS + kFinalHoldS >
            options_.t_stop_s) {
      fail(Outcome::DeadlineStop, "graceful stop at T_stop");
      markRestNotRun("not run: T_stop reached");
      enterKill(t, /*abort_after=*/false, /*to_final=*/true);
    }

    switch (phase_) {
      case Phase::Capture: {
        probe_tau_ = 0.0;
        // quiescence is judged UNDER the restoring hold — never by
        // handing back to the float, which would lose position again
        // (reviewer correction 3)
        capture_metric_.update(t, state.q.get());
        if (t < hold_traj_.captureEnd()) {
          capture_quiet_ = 0.0;
          break;
        }
        const double held = t - hold_traj_.captureEnd();
        double worst = 0.0;
        int worst_j = 0;
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          const double dev = std::fabs(
              zVecElemNC(state.q.get(), i) - options_.anchor[i]);
          if (dev > worst) {
            worst = dev;
            worst_j = i;
          }
        }
        // Admission readiness accumulates ONLY while BOTH conditions
        // hold at THIS sample — position inside the tolerance AND the
        // metric quiet — and resets when either fails. A joint stuck
        // motionless OUTSIDE tolerance must not bank quiet time, and a
        // breakaway transit through the band must not be caught
        // mid-slip (review finding: joint 2 was admitted crossing the
        // band at ~0.19 rad/s host velocity — quiet time banked while
        // stuck off-anchor, position checked only at the final sample
        // — and overshot the far boundary 0.11 s later).
        const bool ready_now = worst <= kAnchorToleranceRad &&
                               capture_metric_.maxSpeed() < 0.05;
        capture_quiet_ = ready_now ? capture_quiet_ + dt : 0.0;
        if (held >= 0.5 && capture_quiet_ >= 0.3) {
          capture_s_ = t;  // both gates passed under the hold
          if (options_.freeze_integral_at_capture) {
            // the stored terms keep acting; only the winding stops
            inner_.freezeIntegral(true);
          }
          if (options_.hold_only_s > 0.0) {
            // diagnostic: reset the REPORTING statistics only — the
            // controller and metric state run on uninterrupted
            phase_ = Phase::HoldDiag;
            for (int i = 0; i < model::kCanonicalDof; ++i) {
              hold_max_speed_[i] = 0.0;
              hold_max_speed_enf_[i] = 0.0;
              hold_q_min_[i] = hold_q_max_[i] =
                  zVecElemNC(state.q.get(), i);
            }
          } else {
            phase_ = Phase::LeadIn;
          }
          phase_start_ = t;
        } else if (held >= kCaptureHoldMaxS) {
          // Only the timeout gives up: a quiet P-only equilibrium is
          // still being pulled onto the anchor by the integrator, so
          // an early quiet-but-out sample must keep holding, not fail
          // (review finding).
          fail(Outcome::HardFault,
               "capture timeout: joint " + std::to_string(worst_j) +
                   " still " + std::to_string(worst) +
                   " rad from the anchor (tolerance " +
                   std::to_string(kAnchorToleranceRad) +
                   ", residual speed " +
                   std::to_string(capture_metric_.maxSpeed()) +
                   " rad/s) after " + std::to_string(held) + " s");
        }
        break;
      }
      case Phase::HoldDiag: {
        // CONTINUOUS stability, not a passing final window (review
        // correction): every sample of the hold must keep every joint
        // inside the anchor tolerance AND the unchanged quiescence
        // metric below 0.05 rad/s; all hard monitors stay live. The
        // per-joint maxima feed the tuning decision.
        probe_tau_ = 0.0;
        capture_metric_.update(t, state.q.get());
        // elapsed is recorded continuously so an aborted summary
        // reports the real hold time, not 0.0 (review cosmetic)
        hold_done_s_ = t - phase_start_;
        const bool enforced = t - phase_start_ >= kHoldDiagGraceS;
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          const double q = zVecElemNC(state.q.get(), i);
          hold_max_speed_[i] =
              std::max(hold_max_speed_[i], capture_metric_.speed(i));
          if (enforced) {
            hold_max_speed_enf_[i] = std::max(
                hold_max_speed_enf_[i], capture_metric_.speed(i));
          }
          hold_q_min_[i] = std::min(hold_q_min_[i], q);
          hold_q_max_[i] = std::max(hold_q_max_[i], q);
          const double dev = std::fabs(q - options_.anchor[i]);
          if (dev > kAnchorToleranceRad) {
            fail(Outcome::HardFault,
                 "hold deviation: joint " + std::to_string(i) + " at " +
                     std::to_string(dev) + " rad after " +
                     std::to_string(t - phase_start_) + " s");
            break;
          }
        }
        if (outcome_ == Outcome::Running && enforced &&
            capture_metric_.maxSpeed() >= 0.05) {
          fail(Outcome::HardFault,
               "hold quiescence lost: " +
                   std::to_string(capture_metric_.maxSpeed()) +
                   " rad/s after " + std::to_string(t - phase_start_) +
                   " s");
        }
        if (outcome_ == Outcome::Running &&
            t - phase_start_ >= options_.hold_only_s) {
          outcome_ = Outcome::Completed;
          phase_ = Phase::Done;
        }
        break;
      }
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
        if (t - phase_start_ >= lead_in_s_) {  // phase_start_ = 0
          finishFloors();                      // without a capture phase
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
        // period counter + window-aggregate LS (offset/drift-immune)
        win_q_.add(phi_, dt, q_probe);
        const double t_win = t - measure_start_;
        for (int i = 0; i < model::kCanonicalDof; ++i) {
          win_joint_[i].add(t_win, phi_, dt,
                            zVecElemNC(state.q.get(), i));
        }
        win_tau_.add(t_win, phi_, dt, tau_probe);
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
      win_joint_[i].reset();
    }
    win_q_.reset();
    win_tau_.reset();
    win_cmd_.reset();
  }

  void finishMeasure(double t) {
    auto& res = results_[dwell_];
    res.completed = true;
    res.window_s = t - measure_start_;
    res.window_periods = win_q_.blocks();
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      win_joint_[i].solve(&res.resp[i]);
    }
    win_tau_.solve(&res.tau_meas);
    win_cmd_.solve(&res.tau_cmd);
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
    std::fprintf(log_, ",%d,%d,%d,%.4f,%.4f,%.6f,%.5f,%.5f,%d,%.6f",
                 dwell_, static_cast<int>(phase_), options_.probe_joint,
                 dwell_ >= 0 &&
                         dwell_ < static_cast<int>(options_.dwells.size())
                     ? currentSpec().freq_hz
                     : 0.0,
                 amp, phi_, probe_tau_, last_cmd_tau_,
                 probe_clipped_ ? 1 : 0, resp_amp_est_);
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
  AnchorCaptureTrajectory hold_traj_;
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

  // measurement-window state (LS aggregates, offset/drift-immune)
  BlockDemod win_q_;  // period counter + probe-joint blocks
  LsAccum win_joint_[model::kCanonicalDof];
  LsAccum win_tau_;  // measured probe-joint torque
  LsAccum win_cmd_;  // SUBMITTED TOTAL probe-joint torque (not the
                     // probe reference: the actuator-transfer variant
                     // needs the actual command — review finding)

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

  // capture phase (integrated startup)
  bool capture_begun_ = false;
  double capture_initial_dev_ = 0.0;
  double capture_quiet_ = 0.0;
  double capture_s_ = 0.0;
  QuiescenceMetric capture_metric_;
  model::ZVector ref_scratch_{model::kCanonicalDof};

  // hold diagnostic
  double hold_done_s_ = 0.0;
  double hold_max_speed_[model::kCanonicalDof] = {};
  double hold_max_speed_enf_[model::kCanonicalDof] = {};
  double hold_q_min_[model::kCanonicalDof] = {};
  double hold_q_max_[model::kCanonicalDof] = {};
  double gain_scale_[model::kCanonicalDof] = {};
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
      "columns describe THIS cycle's excitation; cmd_total is the "
      "SUBMITTED total probe-joint torque (anchor + probe, post-clamp "
      "— the actuator-transfer variant's input); resp_amp_est is the "
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
               "cmd_total,probe_clipped,resp_amp_est");
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
  if (options.capture_envelope_rad > 0.0) {
    std::fprintf(f,
                 "  \"capture\": {\"envelope_rad\": %.4f, "
                 "\"initial_dev_rad\": %.4f, \"accepted_at_s\": %.3f},\n",
                 options.capture_envelope_rad, run.captureInitialDev(),
                 run.captureAcceptedAt());
  }
  // The COMPLETE controller record (review correction: the FRFs are
  // controller-specific on a multivariable arm — the analysis merge
  // guard refuses to combine runs under different tunings).
  std::fprintf(f,
               "  \"tuning\": {\"kp\": %.4f, \"kd\": %.4f, \"ki\": %.4f, "
               "\"i_clamp_nm\": %.4f, \"pd_tau_s\": %.4f, "
               "\"nominal_dt_s\": %.4f, \"gain_scale\": [",
               tuning::kKp, tuning::kKd, tuning::kKi,
               tuning::kIntegralClampNm, tuning::kPdFilterTau,
               tuning::kNominalDt);
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(f, "%s%.4f", i ? ", " : "", run.gainScales()[i]);
  }
  std::fprintf(f, "], \"integral_frozen_at_capture\": %d},\n",
               options.freeze_integral_at_capture ? 1 : 0);
  if (options.hold_only_s > 0.0) {
    std::fprintf(f,
                 "  \"hold\": {\"requested_s\": %.1f, "
                 "\"completed_s\": %.2f,\n   \"max_speed\": [",
                 options.hold_only_s, run.holdCompletedS());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(f, "%s%.4f", i ? ", " : "", run.holdMaxSpeed(i));
    }
    std::fprintf(f, "],\n   \"max_speed_enforced\": [");
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(f, "%s%.4f", i ? ", " : "",
                   run.holdMaxSpeedEnforced(i));
    }
    std::fprintf(f, "],\n   \"range_rad\": [");
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      std::fprintf(f, "%s%.5f", i ? ", " : "", run.holdRangeRad(i));
    }
    std::fprintf(f, "]},\n");
  }
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
