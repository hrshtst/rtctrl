#include "rtctrl/learn/ilc_update.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace rtctrl::learn {

namespace {

constexpr int kDof = model::kCanonicalDof;
constexpr double kPi = 3.14159265358979323846;
constexpr double kNyquistFraction = 0.8;
constexpr double kPadCycles = 1.5;   // padding, in periods of the cutoff
constexpr double kTailSlack = 1e-9;  // samples

template <typename... Parts>
[[noreturn]] void fail(const char* function, const Parts&... parts) {
  std::ostringstream out;
  out << function << ": ";
  (out << ... << parts);
  throw std::invalid_argument(out.str());
}

// Direct form I, a0 = 1.
struct Biquad {
  double b0 = 0.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
};

void checkFilterArguments(const char* function, double dt, double cutoff_hz) {
  if (!std::isfinite(dt) || !(dt > 0.0)) {
    fail(function, "dt must be finite and > 0 (got ", dt, ")");
  }
  const double limit = kNyquistFraction * (0.5 / dt);
  if (!std::isfinite(cutoff_hz) || !(cutoff_hz > 0.0) || cutoff_hz > limit) {
    fail(function,
         "cutoff_hz must be finite, > 0 and at most 0.8 of the Nyquist "
         "frequency 0.5/dt (got ",
         cutoff_hz, " Hz, limit ", limit, " Hz)");
  }
}

void checkFinite(const char* function, const char* name,
                 const std::vector<JointArray>& x) {
  for (std::size_t i = 0; i < x.size(); ++i) {
    for (int j = 0; j < kDof; ++j) {
      if (!std::isfinite(x[i][j])) {
        fail(function, name, "[", i, "][", j, "] is not finite (", x[i][j],
             ")");
      }
    }
  }
}

// 2nd-order Butterworth low-pass by the bilinear transform, prewarped at
// the design corner f0 = cutoff / (sqrt(2) - 1)^(1/4), so that the
// forward-backward pair's analog prototype 1 / (1 + (f/f0)^4) is 1/sqrt(2)
// at the cutoff.
Biquad design(double dt, double cutoff_hz) {
  const double sqrt2 = std::sqrt(2.0);
  const double f0 = cutoff_hz / std::pow(sqrt2 - 1.0, 0.25);
  const double k = std::tan(kPi * f0 * dt);
  const double norm = 1.0 / (1.0 + sqrt2 * k + k * k);
  Biquad q;
  q.a1 = 2.0 * (k * k - 1.0) * norm;
  q.a2 = (1.0 - sqrt2 * k + k * k) * norm;
  // b0 = k^2 * norm in exact arithmetic. Taken from the stored a1, a2, the
  // DC gain of the stored coefficients is one to rounding of the sum, and
  // exactly one for a low cutoff (k <= 0.29, cutoff_hz * dt <= 0.072),
  // where both sums are exact by Sterbenz's lemma. Computed as k^2 * norm,
  // it would scale a constant by a rounding error that grows as 1 / k^2
  // (the cancellation in 1 + a1 + a2).
  q.b0 = (1.0 + q.a1 + q.a2) / 4.0;
  q.b1 = 2.0 * q.b0;
  q.b2 = q.b0;
  return q;
}

std::size_t padLength(std::size_t n, double dt, double cutoff_hz) {
  const double wanted = std::ceil(kPadCycles / (cutoff_hz * dt));
  const double most = static_cast<double>(n - 1);
  return wanted >= most ? n - 1 : static_cast<std::size_t>(wanted);
}

// One pass in place, starting from the steady state for v[0] (unit DC
// gain: every past input and output equals v[0]).
void filterPass(const Biquad& q, std::vector<double>& v) {
  double x1 = v.front(), x2 = v.front(), y1 = v.front(), y2 = v.front();
  for (double& s : v) {
    const double x0 = s;
    const double y0 = q.b0 * x0 + q.b1 * x1 + q.b2 * x2 - q.a1 * y1 - q.a2 * y2;
    x2 = x1;
    x1 = x0;
    y2 = y1;
    y1 = y0;
    s = y0;
  }
}

// Odd-reflection padding of pad samples at both ends (pad <= n - 1),
// forward then backward pass, padding removed.
std::vector<double> lowPassColumn(const std::vector<double>& x, const Biquad& q,
                                  std::size_t pad) {
  const std::size_t n = x.size();
  std::vector<double> ext(n + 2 * pad);
  for (std::size_t k = 1; k <= pad; ++k) {
    ext[pad - k] = 2.0 * x.front() - x[k];
    ext[pad + n - 1 + k] = 2.0 * x.back() - x[n - 1 - k];
  }
  std::copy(x.begin(), x.end(), ext.begin() + static_cast<std::ptrdiff_t>(pad));
  filterPass(q, ext);
  std::reverse(ext.begin(), ext.end());
  filterPass(q, ext);
  std::reverse(ext.begin(), ext.end());
  const auto first = ext.begin() + static_cast<std::ptrdiff_t>(pad);
  return std::vector<double>(first, first + static_cast<std::ptrdiff_t>(n));
}

std::vector<double> column(const std::vector<JointArray>& x, int j) {
  std::vector<double> c(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) c[i] = x[i][j];
  return c;
}

// v at fractional sample position p >= 0, linearly interpolated. A
// position at or past the last sample reads the last sample; the callers'
// tail check bounds the excess by kTailSlack plus rounding.
double sampleAt(const std::vector<double>& v, double p) {
  const std::size_t last = v.size() - 1;
  if (p >= static_cast<double>(last)) return v[last];
  const double f = std::floor(p);
  const std::size_t k = static_cast<std::size_t>(f);
  const double w = p - f;
  return v[k] + w * (v[k + 1] - v[k]);
}

double rms(double sum_of_squares, std::size_t n) {
  return std::sqrt(sum_of_squares / static_cast<double>(n));
}

void checkIlcArguments(const std::vector<JointArray>& delta_k,
                       const std::vector<JointArray>& error, double dt,
                       const IlcOptions& options) {
  constexpr const char* kFn = "ilcUpdate";
  if (delta_k.size() < 2) {
    fail(kFn, "delta_k must have at least 2 samples (has ", delta_k.size(),
         ")");
  }
  checkFilterArguments(kFn, dt, options.cutoff_hz);
  if (!std::isfinite(options.advance_s) || !(options.advance_s >= 0.0)) {
    fail(kFn, "advance_s must be finite and >= 0 (got ", options.advance_s,
         ")");
  }
  if (!std::isfinite(options.max_abs_rad) || !(options.max_abs_rad > 0.0)) {
    fail(kFn, "max_abs_rad must be finite and > 0 (got ", options.max_abs_rad,
         ")");
  }
  if (!std::isfinite(options.max_trial_step_rad) ||
      !(options.max_trial_step_rad > 0.0)) {
    fail(kFn, "max_trial_step_rad must be finite and > 0 (got ",
         options.max_trial_step_rad, ")");
  }
  for (int j = 0; j < kDof; ++j) {
    if (!std::isfinite(options.phi[j]) || !(options.phi[j] >= 0.0)) {
      fail(kFn, "phi[", j, "] must be finite and >= 0 (got ", options.phi[j],
           ")");
    }
    if (!std::isfinite(options.psi_s[j]) || !(options.psi_s[j] >= 0.0)) {
      fail(kFn, "psi_s[", j, "] must be finite and >= 0 (got ",
           options.psi_s[j], ")");
    }
  }
  const std::size_t n = delta_k.size();
  const std::size_t m = error.size();
  if (m < n) {
    fail(kFn, "error must have at least the ", n,
         " samples of delta_k plus the hold tail (has ", m, ")");
  }
  const double needed = std::ceil(options.advance_s / dt - kTailSlack);
  if (static_cast<double>(m - n) < needed) {
    fail(kFn, "error needs ", needed, " samples past the ", n,
         " of delta_k for advance_s = ", options.advance_s, " s at dt = ", dt,
         " s (has ", m - n, ")");
  }
  checkFinite(kFn, "delta_k", delta_k);
  checkFinite(kFn, "error", error);
}

std::vector<JointArray> update(const std::vector<JointArray>& delta_k,
                               const std::vector<JointArray>& error, double dt,
                               const IlcOptions& options, IlcReport& report) {
  checkIlcArguments(delta_k, error, dt, options);
  const std::size_t n = delta_k.size();
  const std::size_t m = error.size();
  const Biquad q = design(dt, options.cutoff_hz);
  const std::size_t pad_error = padLength(m, dt, options.cutoff_hz);
  const std::size_t pad_delta = padLength(n, dt, options.cutoff_hz);
  const double shift = options.advance_s / dt;  // in samples

  std::vector<JointArray> next = delta_k;
  for (int j = 0; j < kDof; ++j) {
    IlcJointReport& r = report.joints[j];
    r = IlcJointReport{};
    double error_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) error_sq += error[i][j] * error[i][j];
    r.error_rms_rad = rms(error_sq, n);

    const double phi = options.phi[j];
    const double psi = options.psi_s[j];
    r.enabled = !(phi == 0.0 && psi == 0.0);
    if (r.enabled) {
      const std::vector<double> ef =
          lowPassColumn(column(error, j), q, pad_error);
      std::vector<double> edot(m);
      edot.front() = (ef[1] - ef[0]) / dt;
      edot.back() = (ef[m - 1] - ef[m - 2]) / dt;
      for (std::size_t i = 1; i + 1 < m; ++i) {
        edot[i] = (ef[i + 1] - ef[i - 1]) / (2.0 * dt);
      }
      std::vector<double> c(n);
      for (std::size_t i = 0; i < n; ++i) {
        const double p = static_cast<double>(i) + shift;
        c[i] = delta_k[i][j] + phi * sampleAt(ef, p) + psi * sampleAt(edot, p);
      }
      const std::vector<double> d = lowPassColumn(c, q, pad_delta);
      const double step = options.max_trial_step_rad;
      const double bound = options.max_abs_rad;
      // An unclamped sample keeps d[i] exactly (base + (d[i] - base) need
      // not round back to d[i]).
      for (std::size_t i = 0; i < n; ++i) {
        const double base = delta_k[i][j];
        double value = d[i];
        if (value - base > step) {
          value = base + step;
          ++r.step_clamped;
        } else if (value - base < -step) {
          value = base - step;
          ++r.step_clamped;
        }
        if (value > bound) {
          value = bound;
          ++r.abs_clamped;
        } else if (value < -bound) {
          value = -bound;
          ++r.abs_clamped;
        }
        next[i][j] = value;
      }
    }

    double delta_sq = 0.0, change_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      const double value = next[i][j];
      const double change = value - delta_k[i][j];
      delta_sq += value * value;
      change_sq += change * change;
      r.delta_max_abs_rad = std::max(r.delta_max_abs_rad, std::abs(value));
      r.change_max_abs_rad = std::max(r.change_max_abs_rad, std::abs(change));
    }
    r.delta_rms_rad = rms(delta_sq, n);
    r.change_rms_rad = rms(change_sq, n);
  }
  return next;
}

}  // namespace

std::vector<JointArray> zeroPhaseLowPass(const std::vector<JointArray>& x,
                                         double dt, double cutoff_hz) {
  constexpr const char* kFn = "zeroPhaseLowPass";
  if (x.size() < 2) {
    fail(kFn, "x must have at least 2 samples (has ", x.size(), ")");
  }
  checkFilterArguments(kFn, dt, cutoff_hz);
  checkFinite(kFn, "x", x);
  const Biquad q = design(dt, cutoff_hz);
  const std::size_t pad = padLength(x.size(), dt, cutoff_hz);
  std::vector<JointArray> y(x.size());
  for (int j = 0; j < kDof; ++j) {
    const std::vector<double> filtered = lowPassColumn(column(x, j), q, pad);
    for (std::size_t i = 0; i < x.size(); ++i) y[i][j] = filtered[i];
  }
  return y;
}

std::vector<JointArray> ilcUpdate(const std::vector<JointArray>& delta_k,
                                  const std::vector<JointArray>& error,
                                  double dt, const IlcOptions& options) {
  IlcReport unused;
  return update(delta_k, error, dt, options, unused);
}

std::vector<JointArray> ilcUpdate(const std::vector<JointArray>& delta_k,
                                  const std::vector<JointArray>& error,
                                  double dt, const IlcOptions& options,
                                  IlcReport& report) {
  IlcReport filled;
  std::vector<JointArray> next = update(delta_k, error, dt, options, filled);
  report = filled;
  return next;
}

}  // namespace rtctrl::learn
