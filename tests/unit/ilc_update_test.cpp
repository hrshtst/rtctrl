#include "rtctrl/learn/ilc_update.hpp"

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/model/joint_map.hpp"

using Catch::Matchers::ContainsSubstring;
using rtctrl::learn::IlcOptions;
using rtctrl::learn::IlcReport;
using rtctrl::learn::ilcUpdate;
using rtctrl::learn::JointArray;
using rtctrl::learn::zeroPhaseLowPass;
using rtctrl::model::kCanonicalDof;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

using Signal = std::vector<JointArray>;

template <typename F>
Signal makeSignal(std::size_t n, F&& value) {
  Signal s(n);
  for (std::size_t i = 0; i < n; ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) s[i][j] = value(i, j);
  }
  return s;
}

// The std::invalid_argument's message, or "" when fn throws nothing (any
// other exception propagates and fails the test).
template <typename F>
std::string invalidArgument(F&& fn) {
  try {
    fn();
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return {};
}

bool bitIdentical(double a, double b) {
  return std::memcmp(&a, &b, sizeof a) == 0;
}

// ---------------------------------------------------------------- filter

// The combined forward-backward magnitude of zeroPhaseLowPass's bilinear
// design: the analog prototype 1 / (1 + (w/w0)^4), with the cutoff
// prewarped and w0 = that prewarped cutoff * (sqrt(2) - 1)^(-1/4).
double digitalMagnitude(double f_hz, double dt, double cutoff_hz) {
  const double ratio =
      std::tan(kPi * f_hz * dt) / std::tan(kPi * cutoff_hz * dt);
  return 1.0 / (1.0 + (std::sqrt(2.0) - 1.0) * std::pow(ratio, 4.0));
}

struct SineFit {
  double amplitude = 0.0;
  double phase = 0.0;  // y ~ amplitude * sin(w t + phase)
  double offset = 0.0;
};

// Least-squares fit of y ~ a sin(w t) + b cos(w t) + c over rows
// [from, to) of column j.
SineFit fitSine(const Signal& y, int j, double dt, double f_hz,
                std::size_t from, std::size_t to) {
  double m[3][4] = {};
  for (std::size_t i = from; i < to; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double basis[3] = {std::sin(2.0 * kPi * f_hz * t),
                             std::cos(2.0 * kPi * f_hz * t), 1.0};
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) m[r][c] += basis[r] * basis[c];
      m[r][3] += basis[r] * y[i][j];
    }
  }
  for (int p = 0; p < 3; ++p) {  // Gauss-Jordan; the normal matrix is SPD
    for (int r = 0; r < 3; ++r) {
      if (r == p) continue;
      const double factor = m[r][p] / m[p][p];
      for (int c = p; c < 4; ++c) m[r][c] -= factor * m[p][c];
    }
  }
  const double a = m[0][3] / m[0][0];
  const double b = m[1][3] / m[1][1];
  SineFit fit;
  fit.amplitude = std::hypot(a, b);
  fit.phase = std::atan2(b, a);
  fit.offset = m[2][3] / m[2][2];
  return fit;
}

// -------------------------------------------------- plant and disturbance

// Per joint G(z) = z^-d (1 - a) / (1 - a z^-1): a first-order lag with a
// pure delay of d samples and unit DC gain, from rest (u = 0 before t = 0).
struct Plant {
  double a = 0.0;
  int delay = 0;
};

Plant lagPlant(double time_constant_s, double dt, int delay) {
  return {std::exp(-dt / time_constant_s), delay};
}

std::vector<double> respond(const std::vector<double>& u, const Plant& p) {
  std::vector<double> y(u.size());
  double state = 0.0;
  for (std::size_t i = 0; i < u.size(); ++i) {
    const double input =
        i >= static_cast<std::size_t>(p.delay) ? u[i - p.delay] : 0.0;
    state = p.a * state + (1.0 - p.a) * input;
    y[i] = state;
  }
  return y;
}

double minimumJerk(double s) {
  if (s <= 0.0) return 0.0;
  if (s >= 1.0) return 1.0;
  return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
}

// A rest-to-rest minimum-jerk move per joint, starting at 0.25 s. Peak
// reference speed 0.17 to 0.75 rad/s, so the ideal offset stays below the
// 0.05 rad trial step and the clamps never act in the convergence cases.
struct Motion {
  double amplitude_rad;
  double duration_s;
  double sag_per_rad;  // static offset that builds up with the posture change
};
constexpr std::array<Motion, kCanonicalDof> kMotions = {{
    {0.8, 2.0, 0.02},
    {-0.6, 2.5, -0.015},
    {0.5, 1.8, 0.03},
    {-0.7, 2.2, 0.01},
    {0.6, 2.0, -0.02},
    {-0.4, 2.4, 0.025},
    {0.5, 1.6, -0.01},
    {0.3, 1.8, 0.0},
}};
constexpr double kMotionStart_s = 0.25;

struct Scenario {
  double dt = 0.01;
  std::size_t n = 300;    // tracking grid, 3 s; the moves end by 2.75 s
  std::size_t tail = 50;  // the follower's final hold, 0.5 s
  Plant plant;
  Signal r;  // M = n + tail rows: the repeatable error without offset
};

// r = the lag error of the move through the plant itself (q~ - G q~, with
// q~ the move relative to its start) plus a static offset that builds up
// with the posture change (as gravity sag does). r(0) = 0 with zero
// derivative: the identical, at-rest initial state the papers assume.
Scenario makeScenario(const Plant& plant) {
  Scenario s;
  s.plant = plant;
  const std::size_t m = s.n + s.tail;
  s.r.assign(m, JointArray{});
  for (int j = 0; j < kCanonicalDof; ++j) {
    const Motion& mo = kMotions[j];
    std::vector<double> q(m);
    for (std::size_t i = 0; i < m; ++i) {
      const double t = static_cast<double>(i) * s.dt;
      q[i] =
          mo.amplitude_rad * minimumJerk((t - kMotionStart_s) / mo.duration_s);
    }
    const std::vector<double> y = respond(q, plant);
    for (std::size_t i = 0; i < m; ++i) {
      s.r[i][j] = (q[i] - y[i]) + mo.sag_per_rad * q[i];
    }
  }
  return s;
}

// One trial: e = r - G u + noise, u = delta on the grid and held at its
// last value during the tail; noise is a per-joint constant for the trial.
Signal runTrial(const Scenario& s, const Signal& delta,
                const JointArray& noise) {
  const std::size_t m = s.n + s.tail;
  Signal e(m);
  for (int j = 0; j < kCanonicalDof; ++j) {
    std::vector<double> u(m);
    for (std::size_t i = 0; i < m; ++i) {
      u[i] = delta[i < s.n ? i : s.n - 1][j];
    }
    const std::vector<double> y = respond(u, s.plant);
    for (std::size_t i = 0; i < m; ++i) {
      e[i][j] = s.r[i][j] - y[i] + noise[j];
    }
  }
  return e;
}

double trackingMeanSquare(const Signal& e, std::size_t n, int j) {
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) sum += e[i][j] * e[i][j];
  return sum / static_cast<double>(n);
}

struct History {
  std::vector<JointArray> rms;          // per trial: tracking-grid RMS of e
  std::vector<JointArray> mean_square;  // the same, squared
  std::vector<IlcReport> reports;       // per update
  Signal delta;                         // after the last update
};

// Trials 0..trials-1 from delta_0 = 0, an update after each. noise, when
// given, holds one per-joint constant per trial.
History learn(const Scenario& s, const IlcOptions& options, int trials,
              const std::vector<JointArray>* noise = nullptr) {
  History h;
  h.delta.assign(s.n, JointArray{});
  for (int k = 0; k < trials; ++k) {
    const JointArray offset = noise != nullptr ? (*noise)[k] : JointArray{};
    const Signal e = runTrial(s, h.delta, offset);
    JointArray ms{}, rms{};
    for (int j = 0; j < kCanonicalDof; ++j) {
      ms[j] = trackingMeanSquare(e, s.n, j);
      rms[j] = std::sqrt(ms[j]);
    }
    h.mean_square.push_back(ms);
    h.rms.push_back(rms);
    IlcReport report;
    h.delta = ilcUpdate(h.delta, e, s.dt, options, report);
    h.reports.push_back(report);
  }
  return h;
}

// Reproducible standard normals (Box-Muller on 53-bit uniforms from
// mt19937_64, whose output the standard fixes; std::normal_distribution's
// is implementation-defined).
class Normal {
 public:
  explicit Normal(std::uint64_t seed) : engine_(seed) {}
  double operator()() {
    const double u1 = uniform();
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
  }

 private:
  double uniform() {  // on (0, 1)
    return (static_cast<double>(engine_() >> 11) + 0.5) * 0x1p-53;
  }
  std::mt19937_64 engine_;
};

constexpr double kDt = 0.01;
constexpr double kLag_s = 0.03;
constexpr int kDelay = 2;

}  // namespace

// ================================================================ filter

TEST_CASE("zero-phase low-pass returns a constant unchanged",
          "[learn][ilc][filter]") {
  for (const double cutoff : {0.5, 2.0, 20.0, 40.0}) {  // 40 Hz: the limit
    for (const std::size_t n :
         {std::size_t{2}, std::size_t{3}, std::size_t{50}, std::size_t{1000}}) {
      INFO("cutoff " << cutoff << " Hz, n " << n);
      const Signal x =
          makeSignal(n, [](std::size_t, int j) { return 0.37 - 0.11 * j; });
      const Signal y = zeroPhaseLowPass(x, kDt, cutoff);
      REQUIRE(y.size() == n);
      for (std::size_t i = 0; i < n; ++i) {
        for (int j = 0; j < kCanonicalDof; ++j) {
          // steady-state start and unit DC gain: rounding only
          CHECK(std::abs(y[i][j] - x[i][j]) <= 1e-12);
        }
      }
    }
  }
}

TEST_CASE("zero-phase low-pass: exactly -3 dB at the cutoff, no phase shift",
          "[learn][ilc][filter]") {
  // 20 s at 100 Hz, fitted on [5 s, 15 s]. The edge transients decay at
  // 11 1/s or faster (the 2 Hz design's pole radius is 0.895 per sample), so
  // they are below 1e-20 after 5 s; what is left is rounding in the filter
  // and in the fit's sums over 1000 samples, well below 1e-12.
  const std::size_t n = 2001;
  const std::size_t from = 500, to = 1500;
  for (const double cutoff : {2.0, 10.0, 25.0, 40.0}) {  // 40 Hz: the limit
    INFO("cutoff " << cutoff << " Hz");
    const Signal x = makeSignal(n, [&](std::size_t i, int) {
      return std::sin(2.0 * kPi * cutoff * static_cast<double>(i) * kDt);
    });
    const Signal y = zeroPhaseLowPass(x, kDt, cutoff);
    const SineFit at = fitSine(y, 0, kDt, cutoff, from, to);
    CHECK(std::abs(at.amplitude - 1.0 / std::sqrt(2.0)) < 1e-12);
    CHECK(std::abs(at.phase) < 1e-12);  // forward-backward: exactly zero
  }

  const double cutoff = 2.0;
  const Signal x = makeSignal(n, [&](std::size_t i, int j) {
    const double t = static_cast<double>(i) * kDt;
    const double f = j == 0 ? cutoff / 10.0 : 5.0 * cutoff;
    return j < 2 ? std::sin(2.0 * kPi * f * t) : 0.0;
  });
  const Signal y = zeroPhaseLowPass(x, kDt, cutoff);

  const SineFit tenth = fitSine(y, 0, kDt, cutoff / 10.0, from, to);
  CHECK(std::abs(tenth.amplitude - 1.0) < 0.01);  // the spec's passband check
  // the design's own magnitude, 0.99996; the same rounding bound
  CHECK(std::abs(tenth.amplitude -
                 digitalMagnitude(cutoff / 10.0, kDt, cutoff)) < 1e-12);
  CHECK(std::abs(tenth.phase) < 1e-12);

  // 5x the cutoff: the bilinear design's 1 / (1 + (sqrt(2) - 1)
  // (tan(pi f dt) / tan(pi fc dt))^4) = 3.38e-3. The analog prototype
  // 1 / (1 + (sqrt(2) - 1) (f/fc)^4) would say 3.85e-3: at 10 Hz of a
  // 100 Hz rate the warping is 14 %.
  const SineFit five = fitSine(y, 1, kDt, 5.0 * cutoff, from, to);
  const double predicted = digitalMagnitude(5.0 * cutoff, kDt, cutoff);
  CHECK(predicted < 3.4e-3);
  // relative 1e-9: the rounding bound against a 3.4e-3 amplitude
  CHECK(std::abs(five.amplitude / predicted - 1.0) < 1e-9);
  CHECK(std::abs(five.phase) < 1e-9);
}

TEST_CASE("zero-phase low-pass preserves a ramp in the interior",
          "[learn][ilc][filter]") {
  // Each pass starts in the steady state of a constant, so a ramp leaves a
  // lag transient of slope * group delay (at most 0.24 rad/s * 0.09 s) that
  // decays at 11 1/s: below 1e-14 at 2 s from either end (beyond the 0.75 s
  // of padding), so 1e-9 is rounding with margin. Odd reflection continues
  // a ramp exactly.
  const std::size_t n = 1001;  // 10 s
  const Signal x = makeSignal(n, [](std::size_t i, int j) {
    return 0.2 - 0.05 * j + (0.1 + 0.02 * j) * static_cast<double>(i) * kDt;
  });
  const Signal y = zeroPhaseLowPass(x, kDt, 2.0);
  for (std::size_t i = 200; i <= 800; ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      CHECK(std::abs(y[i][j] - x[i][j]) < 1e-9);
    }
  }
}

TEST_CASE("zero-phase low-pass keeps a symmetric input symmetric",
          "[learn][ilc][filter]") {
  // A bump centred on the middle sample over a pedestal: flat at both ends
  // (the Gaussian tails are below 1e-60 there), so both passes start in
  // steady state and only the rounding of the two pass orders differs.
  const std::size_t n = 601;
  const std::size_t mid = 300;
  const Signal x = makeSignal(n, [&](std::size_t i, int j) {
    const double t = (static_cast<double>(i) - static_cast<double>(mid)) * kDt;
    const double width = 0.1 + 0.02 * j;
    return 0.1 * j + 0.3 * std::exp(-(t * t) / (width * width));
  });
  const Signal y = zeroPhaseLowPass(x, kDt, 2.0);
  for (int j = 0; j < kCanonicalDof; ++j) {
    std::size_t peak = 0;
    for (std::size_t i = 0; i < n; ++i) {
      CHECK(std::abs(y[i][j] - y[n - 1 - i][j]) < 1e-12);
      if (y[i][j] > y[peak][j]) peak = i;
    }
    CHECK(peak == mid);                   // a causal filter would delay it
    CHECK(y[mid][j] < x[mid][j] - 0.01);  // and it did filter
  }
}

TEST_CASE("zero-phase low-pass filters each joint column on its own",
          "[learn][ilc][filter]") {
  const std::size_t n = 300;
  const Signal x = makeSignal(n, [](std::size_t i, int j) {
    return std::sin(0.05 * static_cast<double>(i) * (j + 1)) + 0.1 * j;
  });
  const Signal y = zeroPhaseLowPass(x, kDt, 2.0);
  for (int j = 0; j < kCanonicalDof; ++j) {
    const Signal alone = makeSignal(
        n, [&](std::size_t i, int jj) { return jj == j ? x[i][j] : 0.0; });
    const Signal ya = zeroPhaseLowPass(alone, kDt, 2.0);
    for (std::size_t i = 0; i < n; ++i) CHECK(bitIdentical(ya[i][j], y[i][j]));
  }
}

TEST_CASE("zero-phase low-pass rejects invalid arguments",
          "[learn][ilc][filter]") {
  const Signal ok = makeSignal(
      10, [](std::size_t i, int) { return 0.01 * static_cast<double>(i); });
  CHECK_NOTHROW(zeroPhaseLowPass(ok, kDt, 2.0));
  CHECK_NOTHROW(zeroPhaseLowPass(ok, kDt, 40.0));  // 0.8 of 50 Hz, allowed

  CHECK_THAT(invalidArgument([&] { zeroPhaseLowPass(Signal{}, kDt, 2.0); }),
             ContainsSubstring("x must have at least 2 samples"));
  CHECK_THAT(invalidArgument([&] { zeroPhaseLowPass(Signal(1), kDt, 2.0); }),
             ContainsSubstring("x must have at least 2 samples"));
  for (const double dt : {0.0, -0.01, kNaN, kInf}) {
    INFO("dt " << dt);
    CHECK_THAT(invalidArgument([&] { zeroPhaseLowPass(ok, dt, 2.0); }),
               ContainsSubstring("dt must be finite and > 0"));
  }
  for (const double cutoff : {0.0, -1.0, kNaN, kInf, 40.001}) {
    INFO("cutoff " << cutoff);
    CHECK_THAT(invalidArgument([&] { zeroPhaseLowPass(ok, kDt, cutoff); }),
               ContainsSubstring("cutoff_hz must be finite, > 0 and at most "
                                 "0.8 of the Nyquist frequency"));
  }
  for (const double bad : {kNaN, kInf, -kInf}) {
    Signal x = ok;
    x[7][5] = bad;
    CHECK_THAT(invalidArgument([&] { zeroPhaseLowPass(x, kDt, 2.0); }),
               ContainsSubstring("x[7][5] is not finite"));
  }
}

// ============================================================ update law

TEST_CASE(
    "ILC: the tracking error falls monotonically on a lagged, delayed "
    "plant",
    "[learn][ilc]") {
  // Representative of the arm: 100 Hz, lag 30 ms, delay 2 samples, advance
  // 50 ms (delay + lag), cutoff 2 Hz, phi 0.5, psi 0. At low frequency
  // |1 - phi G| = 0.5 per trial, and |Q (1 - L G)| < 1 at every frequency
  // (its maximum is 0.50, near DC), so the error contracts each trial until
  // it reaches the floor Q leaves above its cutoff. (With phi = 1 that floor
  // is reached by the fourth trial; from there the error approaches Q's fixed
  // point, by a fraction of a percent and not necessarily from above, so the
  // monotone check uses phi = 0.5 and the next case covers phi = 1.)
  const Scenario s = makeScenario(lagPlant(kLag_s, kDt, kDelay));
  IlcOptions options;  // defaults: advance 0.05 s, cutoff 2 Hz, clamps
  options.phi.fill(0.5);
  const int trials = 12;
  const History h = learn(s, options, trials);
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("joint " << j);
    for (int k = 1; k < trials; ++k) {
      INFO("trial " << k);
      CHECK(h.rms[k][j] < h.rms[k - 1][j]);
    }
    CHECK(h.rms[trials - 1][j] < 0.1 * h.rms[0][j]);
  }
  for (const IlcReport& report : h.reports) {
    for (const auto& joint : report.joints) {
      CHECK(joint.enabled);
      CHECK(joint.step_clamped == 0);  // the convergence is the law's
      CHECK(joint.abs_clamped == 0);
    }
  }
  // the report's error RMS is the tracking-grid RMS of the logged error
  for (int j = 0; j < kCanonicalDof; ++j) {
    CHECK(std::abs(h.reports[3].joints[j].error_rms_rad - h.rms[3][j]) <=
          1e-15);
  }
}

TEST_CASE("ILC: a unit gain removes most of the error in one trial",
          "[learn][ilc]") {
  // phi = 1 cancels the quasi-static error in one trial (G = 1 at low
  // frequency); what is left is Q's and the lag's share at the moves'
  // frequencies (|1 - Q L G| is 0.06 at 1 Hz). |Q (1 - L G)| is at most
  // about 0.07 at any frequency, so it stays down.
  const Scenario s = makeScenario(lagPlant(kLag_s, kDt, kDelay));
  IlcOptions options;
  options.phi.fill(1.0);
  const int trials = 20;
  const History h = learn(s, options, trials);
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("joint " << j);
    for (int k = 1; k < trials; ++k) {
      INFO("trial " << k);
      CHECK(h.rms[k][j] < 0.1 * h.rms[0][j]);
    }
  }
  for (const IlcReport& report : h.reports) {
    for (const auto& joint : report.joints) {
      CHECK(joint.step_clamped == 0);
      CHECK(joint.abs_clamped == 0);
    }
  }
}

TEST_CASE("ILC: a velocity gain inside the note's bound still converges",
          "[learn][ilc]") {
  // The note's bound 0 < psi < 2 (T - theta) = 2 * 30 ms (the lag part, not
  // the delay): psi = 15 ms and 30 ms, a quarter and a half of it.
  const Scenario s = makeScenario(lagPlant(kLag_s, kDt, kDelay));
  IlcOptions options;
  options.phi.fill(0.5);
  for (int j = 0; j < kCanonicalDof; ++j) {
    options.psi_s[j] = j % 2 == 0 ? 0.015 : 0.03;
  }
  const int trials = 12;
  const History h = learn(s, options, trials);
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("joint " << j << ", psi " << options.psi_s[j]);
    for (int k = 1; k < trials; ++k) {
      INFO("trial " << k);
      CHECK(h.rms[k][j] < h.rms[k - 1][j]);
    }
    CHECK(h.rms[trials - 1][j] < 0.1 * h.rms[0][j]);
  }
}

TEST_CASE("ILC: non-repeatable error settles at the note's floor",
          "[learn][ilc]") {
  // The note's "Limits": with G = 1, a scalar gain gamma and a non-repeatable
  // part of variance sigma^2 independent between trials, the converged
  // error variance is 2 sigma^2 / (2 - gamma). Here the non-repeatable part
  // is a random static offset per trial and joint (sigma = 1 mrad), inside
  // the learning band, where G = 1, Q = 1 and L = phi hold exactly (DC).
  // Joints 0-3 learn with phi = 0.5 (floor 1.33 sigma^2), joints 4-7 with
  // phi = 1 (2 sigma^2); the repeatable residual each settles at is added.
  //
  // Tolerance 10 %: per joint a trial's noise error is nearly one Gaussian
  // variable z_k, correlated between trials as rho_m = -(phi/2)(1-phi)^(m-1),
  // so the mean square over K = 4 joints x 2000 trials has a relative
  // standard error of sqrt(2 (1 + 2 sum rho_m^2) / K) = 1.7 % (phi 0.5) and
  // 1.9 % (phi 1); the finite horizon moves the expectation by < 1 % (the
  // first ~6 samples of 300, before the plant responds, carry sigma^2
  // instead). 10 % is about 5 standard errors.
  const Scenario s = makeScenario(lagPlant(kLag_s, kDt, kDelay));
  IlcOptions options;
  for (int j = 0; j < kCanonicalDof; ++j) options.phi[j] = j < 4 ? 0.5 : 1.0;
  const double sigma = 1e-3;
  const int warmup = 40;
  const int measured = 2000;

  const History clean = learn(s, options, warmup + 1);
  std::vector<JointArray> noise(warmup + measured);
  Normal normal(20261112);
  for (auto& row : noise) {
    for (double& v : row) v = sigma * normal();
  }
  const History noisy = learn(s, options, warmup + measured, &noise);

  for (const int group : {0, 1}) {
    const double phi = group == 0 ? 0.5 : 1.0;
    double residual = 0.0, observed = 0.0;
    for (int j = 4 * group; j < 4 * group + 4; ++j) {
      residual += clean.mean_square[warmup][j] / 4.0;
      for (int k = warmup; k < warmup + measured; ++k) {
        observed += noisy.mean_square[k][j] / (4.0 * measured);
      }
    }
    const double predicted = residual + 2.0 * sigma * sigma / (2.0 - phi);
    INFO("phi " << phi << ": residual RMS " << std::sqrt(residual)
                << ", observed RMS " << std::sqrt(observed)
                << ", predicted RMS " << std::sqrt(predicted));
    CHECK(std::abs(observed / predicted - 1.0) < 0.10);
    // the floor is that of the formula, not the bare non-repeatable level
    CHECK(observed > 1.2 * (residual + sigma * sigma));
  }
}

TEST_CASE(
    "ILC: without advance and with Q above the phase crossover the "
    "error grows",
    "[learn][ilc]") {
  // Why the advance and Q exist. Plant delay 5 samples (50 ms) with the
  // 30 ms lag; no advance; cutoff 20 Hz; phi 1.9. The phase of L G leaves
  // +-90 degrees at 3.5 Hz, Q still passes 6 Hz, and there |Q (1 - L G)|
  // peaks at 2.17: the high-frequency content of the error, small at
  // first, roughly doubles every trial. The low-frequency part decays by
  // only |1 - phi| = 0.9 per trial meanwhile, so growth is checked from
  // trial 16 on.
  const Scenario s = makeScenario(lagPlant(kLag_s, kDt, 5));
  IlcOptions options;
  options.phi.fill(1.9);
  options.advance_s = 0.0;
  options.cutoff_hz = 20.0;

  SECTION("clamps out of the way: the law itself diverges") {
    options.max_abs_rad = 1e3;
    options.max_trial_step_rad = 1e3;
    const int trials = 24;
    const History h = learn(s, options, trials);
    for (int j = 0; j < kCanonicalDof; ++j) {
      INFO("joint " << j);
      for (int k = 16; k < trials; ++k) {
        INFO("trial " << k);
        CHECK(h.rms[k][j] > h.rms[k - 1][j]);
      }
      CHECK(h.rms[trials - 1][j] > 10.0 * h.rms[0][j]);
    }
  }
  SECTION("the default clamps bound the offset, not the divergence") {
    const int trials = 30;
    const History h = learn(s, options, trials);
    for (int j = 0; j < kCanonicalDof; ++j) {
      INFO("joint " << j);
      CHECK(h.rms[trials - 1][j] > h.rms[0][j]);
      std::size_t clamped = 0;
      for (const IlcReport& report : h.reports) {
        clamped += report.joints[j].step_clamped + report.joints[j].abs_clamped;
        CHECK(report.joints[j].delta_max_abs_rad <= options.max_abs_rad);
      }
      CHECK(clamped > 0);
    }
  }
}

// ======================================================= update details

TEST_CASE("ILC pairs delta(t) with the error at t + advance", "[learn][ilc]") {
  // A bump in the error centred on sample 300 shows in the offset centred
  // on 300 - advance / dt, both filters being zero-phase. The ends are 3 s
  // from the bump, where the filters' tails are below 1e-14.
  const std::size_t n = 600, m = 700;
  const Signal error = makeSignal(m, [](std::size_t i, int) {
    const double t = (static_cast<double>(i) - 300.0) * kDt;
    return 0.02 * std::exp(-(t * t) / (0.1 * 0.1));
  });
  const Signal zero(n, JointArray{});
  IlcOptions options;
  options.phi.fill(1.0);

  options.advance_s = 0.05;  // 5 samples
  const Signal a = ilcUpdate(zero, error, kDt, options);
  options.advance_s = 0.055;  // 5.5 samples, interpolated
  const Signal b = ilcUpdate(zero, error, kDt, options);
  for (int j = 0; j < kCanonicalDof; ++j) {
    std::size_t peak = 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (a[i][j] > a[peak][j]) peak = i;
    }
    CHECK(peak == 295);
    for (std::size_t k = 0; k <= 100; ++k) {  // symmetric about 295 and 294.5
      CHECK(std::abs(a[295 - k][j] - a[295 + k][j]) < 1e-12);
      CHECK(std::abs(b[294 - k][j] - b[295 + k][j]) < 1e-12);
    }
  }
}

TEST_CASE("ILC: the velocity term is psi_s times the error's rate",
          "[learn][ilc]") {
  // e = b + s t (rad, rad/s): ef = e and edot = s in the interior (a ramp
  // passes the low-pass, central differences are exact on it), so
  // delta_{k+1} = Q[phi (b + s (t + advance)) + psi s] = the same ramp.
  // 1e-9: the ends' transients have decayed by e^-11 over the 1 s to the
  // interior checked (see the ramp test), the rest is rounding.
  const std::size_t n = 300, m = 305;
  const double b = -0.02, slope = 0.05;
  const Signal error = makeSignal(m, [&](std::size_t i, int) {
    return b + slope * static_cast<double>(i) * kDt;
  });
  const Signal zero(n, JointArray{});
  IlcOptions options;
  options.phi.fill(0.5);
  options.psi_s.fill(0.02);
  options.max_abs_rad = 1.0;  // this case is about the law, not the clamps
  options.max_trial_step_rad = 1.0;
  const Signal next = ilcUpdate(zero, error, kDt, options);
  for (std::size_t i = 100; i <= 200; ++i) {
    const double t = static_cast<double>(i) * kDt;
    const double expected = 0.5 * (b + slope * (t + 0.05)) + 0.02 * slope;
    for (int j = 0; j < kCanonicalDof; ++j) {
      CHECK(std::abs(next[i][j] - expected) < 1e-9);
    }
  }
}

TEST_CASE("ILC: with zero error the update is Q applied to delta_k",
          "[learn][ilc]") {
  // Q acts on the whole bracket, delta_k included: content above the cutoff
  // in the offset is attenuated trial after trial.
  const std::size_t n = 300;
  const Signal delta = makeSignal(n, [](std::size_t i, int j) {
    const double t = static_cast<double>(i) * kDt;
    return 0.02 - 0.003 * j + 0.01 * std::sin(2.0 * kPi * 10.0 * t);
  });
  const Signal error(n + 5, JointArray{});
  IlcOptions options;
  options.phi.fill(0.5);
  const Signal next = ilcUpdate(delta, error, kDt, options);
  const Signal filtered = zeroPhaseLowPass(delta, kDt, options.cutoff_hz);
  for (std::size_t i = 0; i < n; ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      CHECK(bitIdentical(next[i][j], filtered[i][j]));
    }
  }
}

TEST_CASE("ILC clamps the change and the value and reports them",
          "[learn][ilc]") {
  // advance 0, so the error is read on the grid; phi 1 on every joint but 6
  const std::size_t n = 200, m = 210;
  auto ramp = [](std::size_t i) {  // -0.199 .. 0.199, never on a bound
    return -0.2 + 0.002 * (static_cast<double>(i) + 0.5);
  };
  auto inside = [](std::size_t i) {  // -0.14925 .. 0.14925, inside the bound
    return -0.15 + 0.0015 * (static_cast<double>(i) + 0.5);
  };
  Signal delta(n, JointArray{});
  Signal error(m, JointArray{});
  for (std::size_t i = 0; i < m; ++i) {
    if (i < n) {
      delta[i][1] = 0.14;
      delta[i][2] = -0.12;
      delta[i][3] = 0.01;
      delta[i][4] = inside(i);
      delta[i][6] = 0.3;  // outside the bound, but the joint is disabled
    }
    error[i][0] = i < n ? 0.3 : 0.7;  // the tail is not in the error RMS
    error[i][1] = 0.3;
    error[i][2] = -0.3;
    error[i][3] = 0.02;
    error[i][4] = 0.04;
    error[i][5] = ramp(i);  // continued through the tail
    error[i][6] = 0.3;
  }
  IlcOptions options;
  options.phi.fill(1.0);
  options.phi[6] = 0.0;
  options.advance_s = 0.0;
  IlcReport report;
  const Signal next = ilcUpdate(delta, error, kDt, options, report);
  const auto& r = report.joints;

  // 0: the step clamp everywhere, from 0 to +0.05
  CHECK(r[0].step_clamped == n);
  CHECK(r[0].abs_clamped == 0);
  for (std::size_t i = 0; i < n; ++i) CHECK(next[i][0] == 0.05);
  CHECK(std::abs(r[0].error_rms_rad - 0.3) < 1e-15);
  CHECK(std::abs(r[0].delta_rms_rad - 0.05) < 1e-15);
  CHECK(r[0].delta_max_abs_rad == 0.05);
  CHECK(std::abs(r[0].change_rms_rad - 0.05) < 1e-15);
  CHECK(r[0].change_max_abs_rad == 0.05);

  // 1 and 2: both clamps everywhere, 0.14 + 0.05 > 0.15, -0.12 - 0.05 < -0.15
  CHECK(r[1].step_clamped == n);
  CHECK(r[1].abs_clamped == n);
  CHECK(r[2].step_clamped == n);
  CHECK(r[2].abs_clamped == n);
  for (std::size_t i = 0; i < n; ++i) {
    CHECK(next[i][1] == 0.15);
    CHECK(next[i][2] == -0.15);
  }
  CHECK(std::abs(r[1].change_max_abs_rad - 0.01) < 1e-15);
  CHECK(std::abs(r[2].change_max_abs_rad - 0.03) < 1e-15);
  CHECK(std::abs(r[2].delta_rms_rad - 0.15) < 1e-15);

  // 3: inside both bounds, nothing clamped: 0.01 + 0.02
  CHECK(r[3].step_clamped == 0);
  CHECK(r[3].abs_clamped == 0);
  for (std::size_t i = 0; i < n; ++i) {
    CHECK(std::abs(next[i][3] - 0.03) < 1e-12);
  }

  // 4: the offset ramps inside the bound and the update adds 0.04 (below
  // the trial step), which crosses +0.15 where delta_k > 0.11: the last 27
  // samples (i = 173 is 2.5e-4 over, i = 172 1.25e-3 under; the filter's
  // ramp error is below 1e-5). There the absolute clamp shortens the
  // change, never lengthens it.
  CHECK(r[4].step_clamped == 0);
  CHECK(r[4].abs_clamped == 27);
  for (std::size_t i = 0; i < n; ++i) {
    if (i >= 173) {
      CHECK(next[i][4] == 0.15);
      CHECK(next[i][4] - delta[i][4] < 0.04);
    } else {
      CHECK(std::abs(next[i][4] - (delta[i][4] + 0.04)) < 1e-5);
    }
  }
  CHECK(r[4].delta_max_abs_rad == 0.15);
  CHECK(r[4].change_max_abs_rad < 0.04 + 1e-5);

  // 5: the error ramps past +-0.05 at 75 samples each end
  CHECK(r[5].step_clamped == 150);
  CHECK(r[5].abs_clamped == 0);
  for (std::size_t i = 0; i < n; ++i) {
    if (i < 75) CHECK(next[i][5] == -0.05);
    if (i >= 125) CHECK(next[i][5] == 0.05);
    if (i >= 75 && i < 125) CHECK(std::abs(next[i][5] - ramp(i)) < 1e-5);
  }

  // 6: disabled, returned as it was although outside the bound
  CHECK_FALSE(r[6].enabled);
  CHECK(r[6].step_clamped == 0);
  CHECK(r[6].abs_clamped == 0);
  for (std::size_t i = 0; i < n; ++i) CHECK(next[i][6] == 0.3);
  CHECK(r[6].change_max_abs_rad == 0.0);
  CHECK(std::abs(r[6].error_rms_rad - 0.3) < 1e-15);

  // 7: enabled, nothing to learn
  CHECK(r[7].enabled);
  CHECK(r[7].delta_max_abs_rad == 0.0);
  CHECK(r[7].change_rms_rad == 0.0);
  CHECK(r[7].error_rms_rad == 0.0);
}

TEST_CASE("ILC refuses an enabled joint's offset outside the clamp",
          "[learn][ilc]") {
  const std::size_t n = 20, m = 25;
  const Signal error(m, JointArray{});
  IlcOptions options;
  options.phi.fill(0.5);
  options.phi[3] = 0.0;  // disabled: not checked
  options.phi[6] = 0.0;  // psi alone: enabled, checked
  options.psi_s[6] = 0.01;
  auto run = [&](std::size_t i, int j, double value) {
    Signal delta(n, JointArray{});
    delta[i][j] = value;
    return invalidArgument([&] { ilcUpdate(delta, error, kDt, options); });
  };
  CHECK(run(4, 2, 0.15).empty());  // on the bound is inside
  CHECK(run(5, 2, -0.15).empty());
  CHECK_THAT(run(4, 2, std::nextafter(0.15, 1.0)),
             ContainsSubstring("delta_k[4][2] = 0.15000000000000002 rad "
                               "exceeds max_abs_rad = 0.15 rad on enabled "
                               "joint 2"));
  CHECK_THAT(run(7, 5, -0.2), ContainsSubstring("delta_k[7][5] = -0.2 rad"));
  CHECK_THAT(run(19, 6, 0.3), ContainsSubstring("delta_k[19][6] = 0.3 rad"));
  CHECK(run(4, 3, 0.5).empty());
}

TEST_CASE(
    "ILC: with delta_k inside the bound the change never exceeds the "
    "trial step",
    "[learn][ilc]") {
  // Offsets anywhere in [-0.15, 0.15], on the bounds included, and errors
  // large enough for both clamps to act often.
  const std::size_t n = 400, m = 405;
  const Signal delta = makeSignal(n, [](std::size_t i, int j) {
    const double v = 0.16 * std::sin(0.031 * static_cast<double>(i) + j);
    return std::max(-0.15, std::min(0.15, v));
  });
  const Signal error = makeSignal(m, [](std::size_t i, int j) {
    return 0.4 * std::sin(0.017 * static_cast<double>(i) * (j + 1) + 0.5 * j);
  });
  IlcOptions options;
  options.phi.fill(1.0);
  options.psi_s.fill(0.02);
  IlcReport report;
  const Signal next = ilcUpdate(delta, error, kDt, options, report);
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("joint " << j);
    CHECK(report.joints[j].step_clamped > 0);
    CHECK(report.joints[j].abs_clamped > 0);
    for (std::size_t i = 0; i < n; ++i) {
      CHECK(std::abs(next[i][j]) <= options.max_abs_rad);
      // 1e-16: the rounding of delta_k +- step
      CHECK(std::abs(next[i][j] - delta[i][j]) <=
            options.max_trial_step_rad + 1e-16);
    }
  }
}

TEST_CASE("ILC requires the hold tail the advance reads", "[learn][ilc]") {
  const std::size_t n = 100;
  const Signal delta(n, JointArray{});
  IlcOptions options;
  options.phi.fill(0.5);
  auto run = [&](double advance, std::size_t tail) {
    options.advance_s = advance;
    return invalidArgument([&] {
      ilcUpdate(delta, Signal(n + tail, JointArray{}), kDt, options);
    });
  };
  // 0.05 / 0.01 is 5.000000000000001 in binary: 5 samples, not 6
  CHECK(run(0.05, 5).empty());
  CHECK_THAT(run(0.05, 4),
             ContainsSubstring("error needs 5 samples past the 100 of "
                               "delta_k") &&
                 ContainsSubstring("(has 4)"));
  CHECK(run(0.055, 6).empty());  // 5.5 samples
  CHECK_THAT(run(0.055, 5), ContainsSubstring("error needs 6 samples"));
  CHECK(run(0.0, 0).empty());
  CHECK(run(0.3, 30).empty());
  CHECK_THAT(run(0.3, 29), ContainsSubstring("error needs 30 samples"));
  CHECK_THAT(invalidArgument([&] {
               options.advance_s = 0.0;
               ilcUpdate(delta, Signal(n - 1, JointArray{}), kDt, options);
             }),
             ContainsSubstring("error must have at least the 100 samples"));
}

TEST_CASE("ILC leaves a disabled joint bit-identical", "[learn][ilc]") {
  const std::size_t n = 120, m = 125;
  // includes -0.0 and a subnormal; the disabled joint 3 also values beyond
  // the clamp, which it is not checked against
  const Signal delta = makeSignal(n, [](std::size_t i, int j) {
    if (i == 3) return -0.0;
    if (i == 4) return 4.9e-324;
    return (j == 3 ? 0.4 : 0.12) * std::sin(0.7 * static_cast<double>(i) + j);
  });
  const Signal error = makeSignal(m, [](std::size_t i, int j) {
    return 0.05 * std::cos(0.3 * static_cast<double>(i) - j);
  });
  IlcOptions options;
  options.phi.fill(0.8);
  options.psi_s.fill(0.01);
  options.phi[3] = 0.0;
  options.psi_s[3] = 0.0;
  options.phi[6] = 0.0;  // psi alone keeps a joint enabled
  IlcReport report;
  const Signal next = ilcUpdate(delta, error, kDt, options, report);
  for (std::size_t i = 0; i < n; ++i) {
    CHECK(bitIdentical(next[i][3], delta[i][3]));
  }
  CHECK_FALSE(report.joints[3].enabled);
  CHECK(report.joints[6].enabled);
  for (int j = 0; j < kCanonicalDof; ++j) {
    if (j == 3) continue;
    // enabled joints were filtered and clamped: |delta| <= 0.15
    CHECK(report.joints[j].delta_max_abs_rad <= 0.15);
  }
}

TEST_CASE("ILC is deterministic and does not modify its inputs",
          "[learn][ilc]") {
  const std::size_t n = 150, m = 160;
  const Signal delta = makeSignal(n, [](std::size_t i, int j) {
    return 0.03 * std::sin(0.11 * static_cast<double>(i) * (j + 1));
  });
  const Signal error = makeSignal(m, [](std::size_t i, int j) {
    return 0.02 * std::cos(0.05 * static_cast<double>(i) + j);
  });
  IlcOptions options;
  options.phi.fill(0.7);
  options.psi_s.fill(0.02);
  const Signal delta_copy = delta;
  const Signal error_copy = error;
  const IlcOptions options_copy = options;
  IlcReport first_report, second_report;
  const Signal first = ilcUpdate(delta, error, kDt, options, first_report);
  const Signal second = ilcUpdate(delta, error, kDt, options, second_report);
  const Signal plain = ilcUpdate(delta, error, kDt, options);
  CHECK(delta == delta_copy);
  CHECK(error == error_copy);
  CHECK(options.phi == options_copy.phi);
  CHECK(options.psi_s == options_copy.psi_s);
  REQUIRE(first.size() == n);
  for (std::size_t i = 0; i < n; ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      CHECK(bitIdentical(first[i][j], second[i][j]));
      CHECK(bitIdentical(first[i][j], plain[i][j]));
    }
  }
  for (int j = 0; j < kCanonicalDof; ++j) {
    CHECK(first_report.joints[j].change_rms_rad ==
          second_report.joints[j].change_rms_rad);
  }
}

TEST_CASE("ILC rejects invalid arguments before computing anything",
          "[learn][ilc]") {
  const std::size_t n = 10, m = 15;
  const Signal delta(n, JointArray{});
  const Signal error(m, JointArray{});
  IlcOptions good;
  good.phi.fill(0.5);
  REQUIRE_NOTHROW(ilcUpdate(delta, error, kDt, good));

  auto withOptions = [&](auto&& change) {
    IlcOptions o = good;
    change(o);
    return invalidArgument([&] { ilcUpdate(delta, error, kDt, o); });
  };

  CHECK_THAT(invalidArgument(
                 [&] { ilcUpdate(Signal(1, JointArray{}), error, kDt, good); }),
             ContainsSubstring("delta_k must have at least 2 samples"));
  CHECK_THAT(invalidArgument([&] { ilcUpdate(Signal{}, error, kDt, good); }),
             ContainsSubstring("delta_k must have at least 2 samples"));
  for (const double bad : {kNaN, kInf, -kInf}) {
    Signal d = delta;
    d[4][2] = bad;
    CHECK_THAT(invalidArgument([&] { ilcUpdate(d, error, kDt, good); }),
               ContainsSubstring("delta_k[4][2] is not finite"));
    Signal e = error;
    e[12][7] = bad;  // in the tail
    CHECK_THAT(invalidArgument([&] { ilcUpdate(delta, e, kDt, good); }),
               ContainsSubstring("error[12][7] is not finite"));
  }
  for (const double dt : {0.0, -0.01, kNaN, kInf}) {
    INFO("dt " << dt);
    CHECK_THAT(invalidArgument([&] { ilcUpdate(delta, error, dt, good); }),
               ContainsSubstring("dt must be finite and > 0"));
  }
  for (const double bad : {-0.01, kNaN, kInf}) {
    INFO("value " << bad);
    CHECK_THAT(withOptions([&](IlcOptions& o) { o.advance_s = bad; }),
               ContainsSubstring("advance_s must be finite and >= 0"));
    CHECK_THAT(withOptions([&](IlcOptions& o) { o.phi[5] = bad; }),
               ContainsSubstring("phi[5] must be finite and >= 0"));
    CHECK_THAT(withOptions([&](IlcOptions& o) { o.psi_s[1] = bad; }),
               ContainsSubstring("psi_s[1] must be finite and >= 0"));
  }
  for (const double bad : {0.0, -0.1, kNaN, kInf}) {
    INFO("value " << bad);
    CHECK_THAT(withOptions([&](IlcOptions& o) { o.max_abs_rad = bad; }),
               ContainsSubstring("max_abs_rad must be finite and > 0"));
    CHECK_THAT(withOptions([&](IlcOptions& o) { o.max_trial_step_rad = bad; }),
               ContainsSubstring("max_trial_step_rad must be finite and > 0"));
  }
  for (const double bad : {0.0, -2.0, kNaN, kInf, 40.001}) {
    INFO("cutoff " << bad);
    CHECK_THAT(withOptions([&](IlcOptions& o) { o.cutoff_hz = bad; }),
               ContainsSubstring("cutoff_hz must be finite, > 0 and at most "
                                 "0.8 of the Nyquist frequency"));
  }
  // checked even when every joint is disabled
  CHECK_THAT(withOptions([&](IlcOptions& o) {
               o.phi.fill(0.0);
               o.cutoff_hz = 0.0;
             }),
             ContainsSubstring("cutoff_hz"));
  CHECK_THAT(invalidArgument([&] {
               IlcOptions o;  // all gains zero
               Signal e = error;
               e[0][0] = kNaN;
               ilcUpdate(delta, e, kDt, o);
             }),
             ContainsSubstring("error[0][0] is not finite"));
  // the report is left untouched on a throw
  IlcReport report;
  report.joints[0].step_clamped = 77;
  CHECK_THROWS_AS(ilcUpdate(delta, error, -1.0, good, report),
                  std::invalid_argument);
  CHECK(report.joints[0].step_clamped == 77);
}
