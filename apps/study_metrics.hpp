// Pure-logic scoring for the offline EFL study — every number the
// preregistered decision rule consumes, plus a minimal JSON writer for
// the machine-readable result table. All constants and formulas are
// frozen in docs/EFL_STUDY_IMPLEMENTATION_PLAN.md; the unit tests in
// tests/unit/study_metrics_test.cpp pin them.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace x7::study {

// ---------------------------------------------------------------- RMS

inline double rms(const std::vector<double>& samples) {
  if (samples.empty()) return 0.0;
  double sum_sq = 0.0;
  for (const double s : samples) sum_sq += s * s;
  return std::sqrt(sum_sq / static_cast<double>(samples.size()));
}

inline double peakAbs(const std::vector<double>& samples) {
  double peak = 0.0;
  for (const double s : samples) peak = std::max(peak, std::abs(s));
  return peak;
}

// -------------------------------------- fixed-frequency amplitude fit

// Least-squares fit of a·cos(2πft) + b·sin(2πft) + c over the window;
// amplitude √(a²+b²) — exact for arbitrary phase and noninteger cycle
// counts (review round 3; a normalized Goertzel coefficient is not).
inline double sineFitAmplitude(const std::vector<double>& t,
                               const std::vector<double>& y,
                               double f_hz) {
  const std::size_t n = t.size();
  if (n < 3 || y.size() != n) return 0.0;
  // Normal equations for the 3-parameter model, solved by Cramer.
  double scc = 0, sss = 0, scs = 0, sc = 0, ss = 0;
  double syc = 0, sys = 0, sy = 0;
  const double w = 2.0 * M_PI * f_hz;
  for (std::size_t k = 0; k < n; ++k) {
    const double c = std::cos(w * t[k]);
    const double s = std::sin(w * t[k]);
    scc += c * c;
    sss += s * s;
    scs += c * s;
    sc += c;
    ss += s;
    syc += y[k] * c;
    sys += y[k] * s;
    sy += y[k];
  }
  const double nn = static_cast<double>(n);
  const double det = scc * (sss * nn - ss * ss) -
                     scs * (scs * nn - ss * sc) +
                     sc * (scs * ss - sss * sc);
  if (std::abs(det) < 1e-30) return 0.0;
  const double a = (syc * (sss * nn - ss * ss) -
                    scs * (sys * nn - ss * sy) +
                    sc * (sys * ss - sss * sy)) /
                   det;
  const double b = (scc * (sys * nn - ss * sy) -
                    syc * (scs * nn - ss * sc) +
                    sc * (scs * sy - sys * sc)) /
                   det;
  return std::sqrt(a * a + b * b);
}

// ------------------------------------------- modal censoring calculus

// The frozen classification (review rounds 3-4): a below-floor
// amplitude is a censored observation, never a zero.
enum class ModalClass {
  Uncensored,       // finite rate ln(A2/A1)/window
  CensoredDecayed,  // ring existed and vanished below resolution
  CensoredGrown,    // vanished, then reappeared — fails C4 outright
  Inconclusive,     // nothing measurable was excited — fails C4
};

struct ModalResult {
  ModalClass cls = ModalClass::Inconclusive;
  double rate = 0.0;    // valid ONLY when cls == Uncensored
  double a_early = 0.0;
  double a1 = 0.0;
  double a2 = 0.0;
};

inline ModalResult classifyModal(double a_early, double a1, double a2,
                                 double eps, double half_gap_s) {
  ModalResult r;
  r.a_early = a_early;
  r.a1 = a1;
  r.a2 = a2;
  if (a1 >= eps && a2 >= eps) {
    r.cls = ModalClass::Uncensored;
    r.rate = std::log(a2 / a1) / half_gap_s;
  } else if (a1 >= eps && a2 < eps) {
    r.cls = ModalClass::CensoredDecayed;
  } else if (a1 < eps && a2 >= eps) {
    r.cls = ModalClass::CensoredGrown;
  } else if (a_early >= eps) {  // a1 < eps && a2 < eps
    r.cls = ModalClass::CensoredDecayed;
  } else {
    r.cls = ModalClass::Inconclusive;
  }
  return r;
}

// EFL-host itself must be uncensored-negative or censored-decayed.
inline bool modalValidNegative(const ModalResult& efl) {
  if (efl.cls == ModalClass::CensoredDecayed) return true;
  return efl.cls == ModalClass::Uncensored && efl.rate < 0.0;
}

// The complete comparator ordering (review round 4).
inline bool modalNoWorse(const ModalResult& efl, const ModalResult& cmp) {
  if (!modalValidNegative(efl)) return false;
  switch (cmp.cls) {
    case ModalClass::CensoredDecayed:
      return efl.cls == ModalClass::CensoredDecayed;
    case ModalClass::Uncensored:
      if (efl.cls == ModalClass::CensoredDecayed) return true;
      return efl.rate <= cmp.rate;
    case ModalClass::CensoredGrown:
      return true;  // any valid negative/decayed result is better
    case ModalClass::Inconclusive:
      return false;  // the comparison itself is inconclusive
  }
  return false;
}

// ------------------------------------------------- settling (D+/D−)

struct Settling {
  bool settled = false;
  double settle_time = 0.0;      // [s] after the trajectory end
  double steady_state_err = 0.0;  // mean |e| over the final window
};

// Frozen definitions: settling = first time after t_end at which |e|
// stays within `band` continuously for `dwell_s`; steady-state = mean
// |e| over the final `ss_window_s`.
inline Settling settlingMetrics(const std::vector<double>& t,
                                const std::vector<double>& e,
                                double t_end, double band,
                                double dwell_s, double ss_window_s) {
  Settling out;
  const std::size_t n = t.size();
  double in_band_since = -1.0;
  for (std::size_t k = 0; k < n; ++k) {
    if (t[k] < t_end) continue;
    if (std::abs(e[k]) <= band) {
      if (in_band_since < 0.0) in_band_since = t[k];
      if (t[k] - in_band_since >= dwell_s) {
        // The FIRST qualifying dwell latches (frozen definition) — a
        // later excursion never voids it (review finding).
        out.settled = true;
        out.settle_time = in_band_since - t_end;
        break;
      }
    } else {
      in_band_since = -1.0;
    }
  }
  if (n > 0) {
    const double ss_start = t.back() - ss_window_s;
    double sum = 0.0;
    int count = 0;
    for (std::size_t k = 0; k < n; ++k) {
      if (t[k] >= ss_start) {
        sum += std::abs(e[k]);
        ++count;
      }
    }
    if (count > 0) out.steady_state_err = sum / count;
  }
  return out;
}

// ----------------------------------------- decision rule (C1..C4)

// One held-out case, EFL-host against that case's comparator
// (PRACTICAL in S, PRACTICAL-GF in F4/F13).
struct CasePair {
  std::string id;
  bool in_s = false;  // member of S = {R1, R2, L1, L2, D+, D−}
  double rms_efl = 0.0, rms_cmp = 0.0;
  double peak_err_efl = 0.0, peak_err_cmp = 0.0;
  double peak_tau_efl = 0.0, peak_tau_cmp = 0.0;
  long sat_efl = 0, sat_cmp = 0;
  bool efl_complete_finite = false;
};

// C1: RMS_EFL <= 0.8 * RMS_cmp in at least 4 of the 6 cases of S.
inline bool criterionC1(const std::vector<CasePair>& cases) {
  int improved = 0;
  for (const auto& c : cases) {
    if (c.in_s && c.rms_efl <= 0.8 * c.rms_cmp) ++improved;
  }
  return improved >= 4;
}

// C2: the no-regression bounds in EVERY case of T = S ∪ {F4, F13}.
inline bool criterionC2(const std::vector<CasePair>& cases) {
  for (const auto& c : cases) {
    if (c.peak_err_efl > 1.1 * c.peak_err_cmp) return false;
    if (c.peak_tau_efl > 1.1 * c.peak_tau_cmp) return false;
    if (c.sat_efl > c.sat_cmp) return false;
  }
  return true;
}

// C3: the six delayed cases complete with finite results. The caller
// passes exactly {L1, L2, D+, D−, F4, F13}.
inline bool criterionC3(const std::vector<CasePair>& delayed_cases) {
  for (const auto& c : delayed_cases) {
    if (!c.efl_complete_finite) return false;
  }
  return true;
}

// C4: both flexible screens valid-negative and no worse than
// PRACTICAL-GF under the comparator ordering.
inline bool criterionC4(const ModalResult& f4_efl,
                        const ModalResult& f4_cmp,
                        const ModalResult& f13_efl,
                        const ModalResult& f13_cmp) {
  return modalNoWorse(f4_efl, f4_cmp) && modalNoWorse(f13_efl, f13_cmp);
}

// ------------------------------------------------------- JSON writer

// Minimal, dependency-free writer — enough structure for the result
// table (objects, arrays, numbers, strings, booleans, null).
class JsonWriter {
 public:
  void beginObject() { open('{'); }
  void endObject() { close('}'); }
  void beginArray() { open('['); }
  void endArray() { close(']'); }
  void key(const std::string& k) {
    comma();
    out_ += '"' + escape(k) + "\":";
    just_keyed_ = true;
  }
  void value(double v) {
    comma();
    if (std::isfinite(v)) {
      char buf[32];
      std::snprintf(buf, sizeof buf, "%.17g", v);
      out_ += buf;
    } else {
      out_ += "null";  // non-finite metrics are encoded as null
    }
  }
  void value(long v) {
    comma();
    out_ += std::to_string(v);
  }
  void value(int v) { value(static_cast<long>(v)); }
  void value(bool v) {
    comma();
    out_ += v ? "true" : "false";
  }
  void value(const std::string& v) {
    comma();
    out_ += '"' + escape(v) + '"';
  }
  void value(const char* v) { value(std::string(v)); }
  void null() {
    comma();
    out_ += "null";
  }
  const std::string& str() const { return out_; }

 private:
  void open(char c) {
    comma();
    out_ += c;
    need_comma_ = false;
  }
  void close(char c) {
    out_ += c;
    need_comma_ = true;
    just_keyed_ = false;
  }
  void comma() {
    if (just_keyed_) {
      just_keyed_ = false;
      need_comma_ = true;
      return;
    }
    if (need_comma_) out_ += ',';
    need_comma_ = true;
  }
  static std::string escape(const std::string& s) {
    std::string r;
    for (const char c : s) {
      if (c == '"' || c == '\\') {
        r += '\\';
        r += c;
      } else if (c == '\n') {
        r += "\\n";
      } else {
        r += c;
      }
    }
    return r;
  }
  std::string out_;
  bool need_comma_ = false;
  bool just_keyed_ = false;
};

}  // namespace x7::study
