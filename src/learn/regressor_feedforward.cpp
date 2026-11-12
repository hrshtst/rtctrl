#include "rtctrl/learn/regressor_feedforward.hpp"

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <toml++/toml.hpp>
#include <utility>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/zvector.hpp"

namespace rtctrl::learn {

namespace fs = std::filesystem;

namespace {

constexpr int kDof = model::kCanonicalDof;
constexpr int kGripper = model::kCanonicalDof - 1;
// Slack on a whole number of samples: the Python tool's kernel half-width
// is ceil(4 sigma / dt - 1e-9), and the advance in samples uses the same.
constexpr double kSampleSlack = 1e-9;
constexpr int kMaxJacobiSweeps = 100;

using Matrix =
    std::array<std::array<double, kRegressorCount>, kRegressorCount>;
using Column = std::array<double, kRegressorCount>;

[[noreturn]] void invalid(const std::string& message) {
  throw std::invalid_argument(message);
}

std::string shortNumber(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof buffer, "%.6g", value);
  return buffer;
}

std::string sampleJoint(std::size_t i, int j) {
  return "sample " + std::to_string(i) + ", joint " + std::to_string(j);
}

void requireStep(const std::string& where, double dt) {
  if (!(std::isfinite(dt) && dt > 0.0)) {
    invalid(where + ": dt must be finite and > 0, got " + shortNumber(dt));
  }
}

void requireNonNegative(const std::string& where, const char* name,
                        double value) {
  if (!(std::isfinite(value) && value >= 0.0)) {
    invalid(where + ": " + name + " must be finite and >= 0, got " +
            shortNumber(value));
  }
}

// sigma finite and >= 0 with a kernel of at most kMaxSmoothingHalfWidth
// taps per side; dt already validated.
void requireSmoothing(const std::string& where, const char* name,
                      double sigma_s, double dt) {
  requireNonNegative(where, name, sigma_s);
  if (sigma_s > 0.0 &&
      !(std::ceil(4.0 * sigma_s / dt - kSampleSlack) <=
        kMaxSmoothingHalfWidth)) {
    invalid(where + ": " + name + " = " + shortNumber(sigma_s) +
            " s at dt = " + shortNumber(dt) +
            " s needs a Gaussian kernel wider than " +
            shortNumber(kMaxSmoothingHalfWidth) + " taps per side");
  }
}

void requireFiniteSeries(const std::string& where, const char* name,
                         const std::vector<JointArray>& series) {
  for (std::size_t i = 0; i < series.size(); ++i) {
    for (int j = 0; j < kDof; ++j) {
      if (!std::isfinite(series[i][j])) {
        invalid(where + ": " + name + " is not finite at " +
                sampleJoint(i, j));
      }
    }
  }
}

void requireFiniteRegressors(const std::string& where,
                             const std::vector<RegressorRow>& phi) {
  for (std::size_t i = 0; i < phi.size(); ++i) {
    for (int j = 0; j < kDof; ++j) {
      for (int l = 0; l < kRegressorCount; ++l) {
        if (!std::isfinite(phi[i][j][l])) {
          invalid(where + ": regressor " + std::string(kRegressorNames[l]) +
                  " is not finite at " + sampleJoint(i, j));
        }
      }
    }
  }
}

void checkRegressorOptions(const RegressorOptions& options,
                           const std::string& prefix) {
  if (!(std::isfinite(options.smooth_sigma_s) &&
        options.smooth_sigma_s >= 0.0)) {
    invalid(prefix + "smooth_sigma_s must be finite and >= 0, got " +
            shortNumber(options.smooth_sigma_s));
  }
  if (!(std::isfinite(options.coulomb_v0_rad_s) &&
        options.coulomb_v0_rad_s > 0.0)) {
    invalid(prefix + "coulomb_v0_rad_s must be finite and > 0, got " +
            shortNumber(options.coulomb_v0_rad_s));
  }
}

std::string weightKey(int j, int l) {
  return "weights.joint" + std::to_string(j) + "[" + std::to_string(l) +
         "] (" + std::string(kRegressorNames[l]) + ")";
}

// Every weight finite, the gripper's all zero.
void checkWeightValues(const FeedforwardWeights& weights,
                       const std::string& prefix) {
  for (int j = 0; j < kDof; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      if (!std::isfinite(weights.w[j][l])) {
        invalid(prefix + weightKey(j, l) + " must be finite");
      }
    }
  }
  for (int l = 0; l < kRegressorCount; ++l) {
    if (weights.w[kGripper][l] != 0.0) {
      invalid(prefix + "weights.joint" + std::to_string(kGripper) +
              " must be all zero: the gripper is never learned (" +
              weightKey(kGripper, l) + " is " +
              shortNumber(weights.w[kGripper][l]) + ")");
    }
  }
}

bool isSha256Hex(const std::string& text) {
  if (text.size() != 64) return false;
  return std::all_of(text.begin(), text.end(), [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}

void checkWeights(const FeedforwardWeights& weights,
                  const std::string& prefix) {
  if (weights.trial < 0) {
    invalid(prefix + "trial must be >= 0, got " +
            std::to_string(weights.trial));
  }
  checkRegressorOptions(weights.regressors, prefix + "regressors.");
  checkWeightValues(weights, prefix);
  const std::pair<const char*, const std::string*> hashes[] = {
      {"parent_sha256", &weights.parent_sha256},
      {"source_log_sha256", &weights.source_log_sha256},
      {"reference_sha256", &weights.reference_sha256}};
  for (const auto& [name, value] : hashes) {
    if (!value->empty() && !isSha256Hex(*value)) {
      invalid(prefix + name +
              " must be empty or 64 lowercase hexadecimal digits");
    }
  }
  if (weights.trial == 0) {
    for (int j = 0; j < kDof; ++j) {
      for (int l = 0; l < kRegressorCount; ++l) {
        if (weights.w[j][l] != 0.0) {
          invalid(prefix + "trial 0 requires all weights zero, but " +
                  weightKey(j, l) + " is " + shortNumber(weights.w[j][l]));
        }
      }
    }
    for (const auto& [name, value] : hashes) {
      if (!value->empty()) {
        invalid(prefix + name + " must be empty for trial 0");
      }
    }
  } else {
    for (const auto& [name, value] : hashes) {
      if (value->empty()) {
        invalid(prefix + name + " is required for trial >= 1");
      }
    }
  }
}

// ------------------------------------------------------- signal kernels

// Validated by the callers.
std::vector<double> smoothCore(const std::vector<double>& x, double sigma_s,
                               double dt) {
  if (sigma_s == 0.0) return x;
  const double width = std::ceil(4.0 * sigma_s / dt - kSampleSlack);
  const auto half =
      std::max<std::ptrdiff_t>(1, static_cast<std::ptrdiff_t>(width));
  std::vector<double> kernel(static_cast<std::size_t>(2 * half + 1));
  for (std::ptrdiff_t m = -half; m <= half; ++m) {
    const double u = static_cast<double>(m) * dt / sigma_s;
    kernel[static_cast<std::size_t>(m + half)] = std::exp(-0.5 * (u * u));
  }
  double sum = 0.0;
  for (const double k : kernel) sum += k;
  for (double& k : kernel) k /= sum;

  // The kernel is exactly symmetric (m and -m give the same u * u), so
  // this correlation equals the Python tool's convolution of the
  // edge-padded series.
  const auto n = static_cast<std::ptrdiff_t>(x.size());
  std::vector<double> out(x.size());
  for (std::ptrdiff_t i = 0; i < n; ++i) {
    double acc = 0.0;
    for (std::ptrdiff_t m = -half; m <= half; ++m) {
      const std::ptrdiff_t k = std::clamp<std::ptrdiff_t>(i + m, 0, n - 1);
      acc += kernel[static_cast<std::size_t>(m + half)] *
             x[static_cast<std::size_t>(k)];
    }
    out[static_cast<std::size_t>(i)] = acc;
  }
  return out;
}

// Validated by the callers: at least 2 samples.
std::vector<double> gradientCore(const std::vector<double>& f, double dt) {
  const std::size_t n = f.size();
  std::vector<double> out(n);
  out[0] = (f[1] - f[0]) / dt;
  out[n - 1] = (f[n - 1] - f[n - 2]) / dt;
  for (std::size_t i = 1; i + 1 < n; ++i) {
    out[i] = (f[i + 1] - f[i - 1]) / (2.0 * dt);
  }
  return out;
}

std::vector<double> jointSeries(const std::vector<JointArray>& series,
                                int j) {
  std::vector<double> out(series.size());
  for (std::size_t i = 0; i < series.size(); ++i) out[i] = series[i][j];
  return out;
}

double rms(const std::vector<double>& x) {
  double sum = 0.0;
  for (const double v : x) sum += v * v;
  return std::sqrt(sum / static_cast<double>(x.size()));
}

// The advance in samples: a whole number when within kSampleSlack of one.
struct Shift {
  std::size_t whole = 0;
  double fraction = 0.0;  // 0 exactly for a whole-sample shift
};

Shift advanceShift(double advance_s, double dt) {
  const double samples = advance_s / dt;
  const double nearest = std::round(samples);
  Shift shift;
  if (std::fabs(samples - nearest) <= kSampleSlack) {
    shift.whole = static_cast<std::size_t>(nearest);
  } else {
    const double whole = std::floor(samples);
    shift.whole = static_cast<std::size_t>(whole);
    shift.fraction = samples - whole;
  }
  return shift;
}

std::vector<JointArray> errorSignal(const std::string& where,
                                    const std::vector<JointArray>& error,
                                    std::size_t samples, double dt,
                                    const FelOptions& options) {
  requireStep(where, dt);
  requireNonNegative(where, "options.advance_s", options.advance_s);
  requireNonNegative(where, "options.error_lead_s", options.error_lead_s);
  requireSmoothing(where, "options.error_smooth_sigma_s",
                   options.error_smooth_sigma_s, dt);
  if (samples < 3) {
    invalid(where + ": the tracking grid needs at least 3 samples, got " +
            std::to_string(samples));
  }
  if (error.size() < samples) {
    invalid(where + ": error has " + std::to_string(error.size()) +
            " samples, fewer than the " + std::to_string(samples) +
            " tracking samples");
  }
  const double tail_needed =
      std::ceil(options.advance_s / dt - kSampleSlack);
  const std::size_t tail = error.size() - samples;
  if (static_cast<double>(tail) < tail_needed) {
    invalid(where + ": error needs at least " + shortNumber(tail_needed) +
            " samples after the " + std::to_string(samples) +
            " tracking samples (ceil(advance_s / dt - 1e-9) for advance_s = " +
            shortNumber(options.advance_s) + " s, dt = " + shortNumber(dt) +
            " s), got " + std::to_string(tail));
  }
  requireFiniteSeries(where, "error", error);

  const Shift shift = advanceShift(options.advance_s, dt);
  const auto at = [&shift](const std::vector<double>& x, std::size_t i) {
    const std::size_t k = i + shift.whole;
    if (shift.fraction == 0.0) return x[k];
    return (1.0 - shift.fraction) * x[k] + shift.fraction * x[k + 1];
  };
  std::vector<JointArray> s(samples);
  for (int j = 0; j < kDof; ++j) {
    const std::vector<double> smooth = smoothCore(
        jointSeries(error, j), options.error_smooth_sigma_s, dt);
    const std::vector<double> rate = gradientCore(smooth, dt);
    for (std::size_t i = 0; i < samples; ++i) {
      s[i][j] = at(smooth, i) + options.error_lead_s * at(rate, i);
    }
  }
  return s;
}

// ------------------------------------------------------ linear algebra

// Extreme eigenvalues {min, max} of the symmetric k x k matrix a (k <= 6)
// by cyclic Jacobi rotations, run until the off-diagonal Frobenius norm is
// at most eps times the matrix's. By Weyl's inequality the diagonal is then
// within that norm of the spectrum; with the rounding of at most
// kMaxJacobiSweeps sweeps of backward-stable rotations the absolute error
// stays below 1e-10 for |a|_F <= 6 (a unit-diagonal Gram matrix).
std::pair<double, double> extremeEigenvalues(Matrix a, int k) {
  double norm2 = 0.0;
  for (int p = 0; p < k; ++p) {
    for (int q = 0; q < k; ++q) norm2 += a[p][q] * a[p][q];
  }
  const double eps = std::numeric_limits<double>::epsilon();
  const double stop = eps * eps * norm2;
  bool converged = false;
  for (int sweep = 0; sweep < kMaxJacobiSweeps && !converged; ++sweep) {
    double off = 0.0;
    for (int p = 0; p < k; ++p) {
      for (int q = p + 1; q < k; ++q) off += 2.0 * a[p][q] * a[p][q];
    }
    if (off <= stop) {
      converged = true;
      break;
    }
    for (int p = 0; p < k; ++p) {
      for (int q = p + 1; q < k; ++q) {
        if (a[p][q] == 0.0) continue;
        // The rotation angle of magnitude <= pi/4 that zeroes a[p][q].
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        double t = 1.0 / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        if (theta < 0.0) t = -t;
        const double c = 1.0 / std::sqrt(t * t + 1.0);
        const double s = t * c;
        for (int r = 0; r < k; ++r) {  // a <- a J
          const double arp = a[r][p];
          const double arq = a[r][q];
          a[r][p] = c * arp - s * arq;
          a[r][q] = s * arp + c * arq;
        }
        for (int r = 0; r < k; ++r) {  // a <- J^T a
          const double apr = a[p][r];
          const double aqr = a[q][r];
          a[p][r] = c * apr - s * aqr;
          a[q][r] = s * apr + c * aqr;
        }
        a[p][q] = 0.0;
        a[q][p] = 0.0;
      }
    }
  }
  if (!converged) {
    throw std::runtime_error(
        "felUpdate: the Jacobi eigenvalue iteration did not converge");
  }
  double lo = a[0][0];
  double hi = a[0][0];
  for (int p = 1; p < k; ++p) {
    lo = std::min(lo, a[p][p]);
    hi = std::max(hi, a[p][p]);
  }
  return {lo, hi};
}

// Solves m x = b for the symmetric positive definite k x k matrix m
// (Cholesky); a non-positive pivot is an error.
Column choleskySolve(const Matrix& m, const Column& b, int k,
                     const std::string& where) {
  Matrix l{};
  for (int j = 0; j < k; ++j) {
    double d = m[j][j];
    for (int p = 0; p < j; ++p) d -= l[j][p] * l[j][p];
    if (!(d > 0.0)) {
      invalid(where + ": the normal matrix is not positive definite");
    }
    l[j][j] = std::sqrt(d);
    for (int i = j + 1; i < k; ++i) {
      double v = m[i][j];
      for (int p = 0; p < j; ++p) v -= l[i][p] * l[j][p];
      l[i][j] = v / l[j][j];
    }
  }
  Column y{};
  for (int i = 0; i < k; ++i) {
    double v = b[i];
    for (int p = 0; p < i; ++p) v -= l[i][p] * y[p];
    y[i] = v / l[i][i];
  }
  Column x{};
  for (int i = k - 1; i >= 0; --i) {
    double v = y[i];
    for (int p = i + 1; p < k; ++p) v -= l[p][i] * x[p];
    x[i] = v / l[i][i];
  }
  return x;
}

// ----------------------------------------------------------- TOML I/O

std::string tomlDouble(double value) {
  char buffer[64];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value,
                                    std::chars_format::general, 17);
  if (result.ec != std::errc()) {
    throw std::runtime_error("follow weights: cannot format a number");
  }
  std::string text(buffer, result.ptr);
  // %.17g prints integral values without a point: append one so TOML
  // reads a float (and -0.0 keeps its sign).
  if (text.find_first_of(".e") == std::string::npos) text += ".0";
  return text;
}

[[noreturn]] void fail(const std::string& source, const std::string& message) {
  throw std::runtime_error("follow weights '" + source + "': " + message);
}

void rejectUnknown(const toml::table& table,
                   std::initializer_list<std::string_view> allowed,
                   const std::string& where, const std::string& source) {
  for (const auto& [key, node] : table) {
    (void)node;
    if (std::find(allowed.begin(), allowed.end(), key.str()) ==
        allowed.end()) {
      fail(source, "unknown key '" + std::string(key.str()) + "' in " + where);
    }
  }
}

const toml::node& requiredNode(const toml::table& table, std::string_view key,
                               const std::string& name,
                               const std::string& source) {
  const toml::node* node = table.get(key);
  if (node == nullptr) fail(source, "missing key " + name);
  return *node;
}

std::string requiredString(const toml::table& table, std::string_view key,
                           const std::string& name,
                           const std::string& source) {
  const auto* value = requiredNode(table, key, name, source).as_string();
  if (value == nullptr) fail(source, name + " must be a string");
  return value->get();
}

std::int64_t requiredInteger(const toml::table& table, std::string_view key,
                             const std::string& name,
                             const std::string& source) {
  const auto* value = requiredNode(table, key, name, source).as_integer();
  if (value == nullptr) fail(source, name + " must be an integer");
  return value->get();
}

double numberValue(const toml::node& node, const std::string& name,
                   const std::string& source) {
  if (!node.is_floating_point() && !node.is_integer()) {
    fail(source, name + " must be a number");
  }
  const auto value = node.value<double>();
  if (!value) {
    fail(source, name + " is an integer a double cannot hold exactly");
  }
  if (!std::isfinite(*value)) fail(source, name + " must be finite");
  return *value;
}

// Pointers, never null (a reference return trips -Wdangling-reference on
// the temporary names).
const toml::table* requiredTable(const toml::table& table,
                                 std::string_view key, const std::string& name,
                                 const std::string& source) {
  const auto* value = requiredNode(table, key, name, source).as_table();
  if (value == nullptr) fail(source, name + " must be a table");
  return value;
}

const toml::array* requiredArray(const toml::table& table,
                                 std::string_view key, const std::string& name,
                                 std::size_t size, const std::string& source) {
  const auto* value = requiredNode(table, key, name, source).as_array();
  if (value == nullptr) fail(source, name + " must be an array");
  if (value->size() != size) {
    fail(source, name + " must have " + std::to_string(size) +
                     " elements, has " + std::to_string(value->size()));
  }
  return value;
}

std::string expectedNames() {
  std::string text = "[";
  for (int l = 0; l < kRegressorCount; ++l) {
    text += (l == 0 ? "\"" : ", \"") + std::string(kRegressorNames[l]) + "\"";
  }
  return text + "]";
}

}  // namespace

// ----------------------------------------------------------- regressors

void validateRegressorOptions(const RegressorOptions& options) {
  checkRegressorOptions(options, "regressor options: ");
}

std::vector<double> zeroPhaseGaussianSmooth(const std::vector<double>& x,
                                            double sigma_s, double dt) {
  const std::string where = "zeroPhaseGaussianSmooth";
  if (x.empty()) invalid(where + ": the series is empty");
  requireStep(where, dt);
  requireSmoothing(where, "sigma_s", sigma_s, dt);
  for (std::size_t i = 0; i < x.size(); ++i) {
    if (!std::isfinite(x[i])) {
      invalid(where + ": sample " + std::to_string(i) + " is not finite");
    }
  }
  return smoothCore(x, sigma_s, dt);
}

std::vector<double> numpyGradient(const std::vector<double>& x, double dt) {
  const std::string where = "numpyGradient";
  if (x.size() < 2) {
    invalid(where + ": needs at least 2 samples, got " +
            std::to_string(x.size()));
  }
  requireStep(where, dt);
  return gradientCore(x, dt);
}

SmoothedReference smoothReference(const std::vector<JointArray>& qref,
                                  double dt, double sigma_s) {
  const std::string where = "smoothReference";
  if (qref.size() < 3) {
    invalid(where + ": qref needs at least 3 samples, got " +
            std::to_string(qref.size()));
  }
  requireStep(where, dt);
  requireSmoothing(where, "sigma_s", sigma_s, dt);
  requireFiniteSeries(where, "qref", qref);
  SmoothedReference out;
  out.q.resize(qref.size());
  out.dq.resize(qref.size());
  out.ddq.resize(qref.size());
  for (int j = 0; j < kDof; ++j) {
    const std::vector<double> q = smoothCore(jointSeries(qref, j), sigma_s, dt);
    const std::vector<double> dq = gradientCore(q, dt);
    const std::vector<double> ddq = gradientCore(dq, dt);
    for (std::size_t i = 0; i < qref.size(); ++i) {
      out.q[i][j] = q[i];
      out.dq[i][j] = dq[i];
      out.ddq[i][j] = ddq[i];
    }
  }
  return out;
}

std::vector<RegressorRow> buildRegressors(model::ChainModel& chain,
                                          const model::JointMap& map,
                                          const std::vector<JointArray>& qref,
                                          double dt,
                                          const RegressorOptions& options) {
  checkRegressorOptions(options, "buildRegressors: options.");
  const SmoothedReference ref =
      smoothReference(qref, dt, options.smooth_sigma_s);
  model::ZVector q(kDof);
  model::ZVector dq(kDof);
  model::ZVector ddq(kDof);
  model::ZVector gravity(kDof);
  model::ZVector dynamics(kDof);
  std::vector<RegressorRow> rows(qref.size());
  for (std::size_t i = 0; i < qref.size(); ++i) {
    for (int j = 0; j < kDof; ++j) {
      q[j] = ref.q[i][j];
      dq[j] = ref.dq[i][j];
      ddq[j] = ref.ddq[i][j];
    }
    chain.gravityTorque(map, q, gravity);
    chain.inverseDynamics(map, q, dq, ddq, dynamics);
    for (int j = 0; j < kDof; ++j) {
      if (!std::isfinite(gravity[j]) || !std::isfinite(dynamics[j])) {
        throw std::runtime_error("buildRegressors: non-finite model torque at " +
                                 sampleJoint(i, j));
      }
    }
    RegressorRow& row = rows[i];
    for (int j = 0; j < kGripper; ++j) {
      row[j][kRegressorGravity] = gravity[j];
      row[j][kRegressorDynamics] = dynamics[j] - gravity[j];
      row[j][kRegressorAcceleration] = ddq[j];
      row[j][kRegressorVelocity] = dq[j];
      row[j][kRegressorCoulomb] = std::tanh(dq[j] / options.coulomb_v0_rad_s);
      row[j][kRegressorConstant] = 1.0;
    }
    row[kGripper].fill(0.0);
  }
  return rows;
}

// -------------------------------------------------------------- weights

void validateFeedforwardWeights(const FeedforwardWeights& weights) {
  checkWeights(weights, "follow weights: ");
}

std::string formatFeedforwardWeights(const FeedforwardWeights& weights) {
  validateFeedforwardWeights(weights);
  std::string out;
  out += "format = \"" + std::string(kFeedforwardWeightsFormat) + "\"\n";
  out += "version = " + std::to_string(kFeedforwardWeightsVersion) + "\n";
  out += "trial = " + std::to_string(weights.trial) + "\n";
  out += "parent_sha256 = \"" + weights.parent_sha256 + "\"\n";
  out += "source_log_sha256 = \"" + weights.source_log_sha256 + "\"\n";
  out += "reference_sha256 = \"" + weights.reference_sha256 + "\"\n";
  out += "\n[regressors]\nnames = " + expectedNames() + "\n";
  out += "smooth_sigma_s = " + tomlDouble(weights.regressors.smooth_sigma_s) +
         "\n";
  out += "coulomb_v0_rad_s = " +
         tomlDouble(weights.regressors.coulomb_v0_rad_s) + "\n";
  out +=
      "\n# delta_j = sum_l w_jl * phi_jl: rad per regressor unit, in names "
      "order;\n# joint7 (the gripper) is never learned.\n[weights]\n";
  for (int j = 0; j < kDof; ++j) {
    out += "joint" + std::to_string(j) + " = [";
    for (int l = 0; l < kRegressorCount; ++l) {
      out += (l == 0 ? "" : ", ") + tomlDouble(weights.w[j][l]);
    }
    out += "]\n";
  }
  return out;
}

FeedforwardWeights parseFeedforwardWeights(std::string_view text,
                                           const std::string& source) {
  toml::table root;
  try {
    root = toml::parse(text, source);
  } catch (const toml::parse_error& error) {
    fail(source, "cannot parse (line " +
                     std::to_string(error.source().begin.line) + "): " +
                     std::string(error.description()));
  }
  const std::string format = requiredString(root, "format", "format", source);
  if (format != kFeedforwardWeightsFormat) {
    fail(source, "format is '" + format + "', expected '" +
                     kFeedforwardWeightsFormat + "'");
  }
  const std::int64_t version =
      requiredInteger(root, "version", "version", source);
  if (version != kFeedforwardWeightsVersion) {
    fail(source, "version is " + std::to_string(version) + ", expected " +
                     std::to_string(kFeedforwardWeightsVersion));
  }
  rejectUnknown(root,
                {"format", "version", "trial", "parent_sha256",
                 "source_log_sha256", "reference_sha256", "regressors",
                 "weights"},
                "the root table", source);

  FeedforwardWeights weights;
  const std::int64_t trial = requiredInteger(root, "trial", "trial", source);
  if (trial < 0 || trial > std::numeric_limits<int>::max()) {
    fail(source, "trial must be an integer in [0, " +
                     std::to_string(std::numeric_limits<int>::max()) +
                     "], got " + std::to_string(trial));
  }
  weights.trial = static_cast<int>(trial);
  weights.parent_sha256 =
      requiredString(root, "parent_sha256", "parent_sha256", source);
  weights.source_log_sha256 =
      requiredString(root, "source_log_sha256", "source_log_sha256", source);
  weights.reference_sha256 =
      requiredString(root, "reference_sha256", "reference_sha256", source);

  const toml::table& regressors =
      *requiredTable(root, "regressors", "[regressors]", source);
  rejectUnknown(regressors, {"names", "smooth_sigma_s", "coulomb_v0_rad_s"},
                "[regressors]", source);
  const toml::array& names = *requiredArray(regressors, "names",
                                            "regressors.names",
                                            kRegressorCount, source);
  for (int l = 0; l < kRegressorCount; ++l) {
    const auto* name = names[static_cast<std::size_t>(l)].as_string();
    if (name == nullptr || name->get() != kRegressorNames[l]) {
      fail(source, "regressors.names must be " + expectedNames() +
                       " (the compiled order); element " + std::to_string(l) +
                       " differs");
    }
  }
  weights.regressors.smooth_sigma_s =
      numberValue(requiredNode(regressors, "smooth_sigma_s",
                               "regressors.smooth_sigma_s", source),
                  "regressors.smooth_sigma_s", source);
  weights.regressors.coulomb_v0_rad_s =
      numberValue(requiredNode(regressors, "coulomb_v0_rad_s",
                               "regressors.coulomb_v0_rad_s", source),
                  "regressors.coulomb_v0_rad_s", source);

  const toml::table& table =
      *requiredTable(root, "weights", "[weights]", source);
  std::array<std::string, kDof> keys;
  for (int j = 0; j < kDof; ++j) keys[j] = "joint" + std::to_string(j);
  for (const auto& [key, node] : table) {
    (void)node;
    if (std::find(keys.begin(), keys.end(), key.str()) == keys.end()) {
      fail(source,
           "unknown key '" + std::string(key.str()) + "' in [weights]");
    }
  }
  for (int j = 0; j < kDof; ++j) {
    const std::string name = "weights." + keys[j];
    const toml::array& row =
        *requiredArray(table, keys[j], name, kRegressorCount, source);
    for (int l = 0; l < kRegressorCount; ++l) {
      weights.w[j][l] = numberValue(row[static_cast<std::size_t>(l)],
                                    weightKey(j, l), source);
    }
  }

  try {
    checkWeights(weights, "");
  } catch (const std::invalid_argument& error) {
    fail(source, error.what());
  }
  return weights;
}

FeedforwardWeights readFeedforwardWeights(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) fail(path.string(), "cannot open");
  const std::string text((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  if (input.bad()) fail(path.string(), "read error");
  return parseFeedforwardWeights(text, path.string());
}

void writeFeedforwardWeights(const fs::path& path,
                             const FeedforwardWeights& weights) {
  const std::string text = formatFeedforwardWeights(weights);  // validates
  std::FILE* file = std::fopen(path.string().c_str(), "wx");
  if (file == nullptr) {
    const int error = errno;
    if (error == EEXIST) {
      throw std::runtime_error("follow weights: refusing to overwrite '" +
                               path.string() + "'");
    }
    throw std::runtime_error("follow weights: cannot create '" +
                             path.string() + "': " + std::strerror(error));
  }
  struct Created {
    std::FILE* file;
    const fs::path& path;
    bool keep = false;
    ~Created() {
      if (file != nullptr) std::fclose(file);
      if (!keep) {
        std::error_code ignored;
        fs::remove(path, ignored);
      }
    }
  } created{file, path};

  bool ok = std::fwrite(text.data(), 1, text.size(), file) == text.size();
  ok = ok && std::fflush(file) == 0 && std::ferror(file) == 0;
  const int closed = std::fclose(file);
  created.file = nullptr;
  if (!ok || closed != 0) {
    throw std::runtime_error("follow weights: cannot write '" +
                             path.string() + "'");
  }
  created.keep = true;
}

// ------------------------------------------------------------ synthesis

std::vector<JointArray> synthesizeOffset(
    const std::vector<RegressorRow>& regressors,
    const FeedforwardWeights& weights) {
  const std::string where = "synthesizeOffset";
  if (regressors.empty()) invalid(where + ": regressors are empty");
  checkWeightValues(weights, where + ": ");
  requireFiniteRegressors(where, regressors);
  std::vector<JointArray> delta(regressors.size());
  for (std::size_t i = 0; i < regressors.size(); ++i) {
    for (int j = 0; j < kGripper; ++j) {
      double sum = 0.0;
      for (int l = 0; l < kRegressorCount; ++l) {
        sum += weights.w[j][l] * regressors[i][j][l];
      }
      if (!std::isfinite(sum)) {
        invalid(where + ": the offset is not finite at " + sampleJoint(i, j));
      }
      delta[i][j] = sum;
    }
    delta[i][kGripper] = 0.0;
  }
  return delta;
}

// --------------------------------------------------------------- update

std::vector<JointArray> felErrorSignal(const std::vector<JointArray>& error,
                                       std::size_t samples, double dt,
                                       const FelOptions& options) {
  return errorSignal("felErrorSignal", error, samples, dt, options);
}

FeedforwardWeights felUpdate(const FeedforwardWeights& w_k,
                             const std::vector<RegressorRow>& phi,
                             const std::vector<JointArray>& error, double dt,
                             const FelOptions& options, FelReport* report) {
  const std::string where = "felUpdate";
  const bool gradient = options.step == FelOptions::Step::NormalisedGradient;
  if (!gradient && options.step != FelOptions::Step::LeastSquares) {
    invalid(where + ": options.step is not a known step");
  }
  if (!(std::isfinite(options.gain) && options.gain > 0.0 &&
        options.gain <= 1.0)) {
    invalid(where + ": options.gain must be in (0, 1], got " +
            shortNumber(options.gain));
  }
  requireNonNegative(where, "options.ridge", options.ridge);
  if (!(std::isfinite(options.negligible_rms) &&
        options.negligible_rms > 0.0)) {
    invalid(where + ": options.negligible_rms must be finite and > 0, got " +
            shortNumber(options.negligible_rms));
  }
  if (options.joint_enabled[kGripper]) {
    invalid(where + ": options.joint_enabled[" + std::to_string(kGripper) +
            "] must be false: the gripper is never learned");
  }
  if (w_k.trial < 0 || w_k.trial == std::numeric_limits<int>::max()) {
    invalid(where + ": w_k.trial must be in [0, " +
            std::to_string(std::numeric_limits<int>::max() - 1) + "], got " +
            std::to_string(w_k.trial));
  }
  checkRegressorOptions(w_k.regressors, where + ": w_k.regressors.");
  checkWeightValues(w_k, where + ": w_k.");
  if (phi.size() < 3) {
    invalid(where + ": phi needs at least 3 samples, got " +
            std::to_string(phi.size()));
  }
  requireFiniteRegressors(where, phi);
  const std::vector<JointArray> s =
      errorSignal(where, error, phi.size(), dt, options);

  const std::size_t n = phi.size();
  const double count = static_cast<double>(n);
  FeedforwardWeights next = w_k;
  next.trial = w_k.trial + 1;
  next.parent_sha256.clear();
  next.source_log_sha256.clear();
  next.reference_sha256.clear();
  FelReport local;

  for (int j = 0; j < kDof; ++j) {
    FelJointReport& joint = local.joints[j];
    joint.enabled = options.joint_enabled[j];
    const std::vector<double> sj = jointSeries(s, j);
    joint.s_rms = rms(sj);
    joint.residual_rms = joint.s_rms;
    std::array<int, kRegressorCount> kept{};
    int k = 0;
    for (int l = 0; l < kRegressorCount; ++l) {
      double sum = 0.0;
      for (std::size_t i = 0; i < n; ++i) sum += phi[i][j][l] * phi[i][j][l];
      joint.column_rms[l] = std::sqrt(sum / count);
      if (!joint.enabled) continue;
      if (joint.column_rms[l] < options.negligible_rms) {
        joint.skipped[l] = true;
      } else {
        kept[k++] = l;
      }
    }
    joint.kept_columns = k;
    if (!joint.enabled || k == 0) continue;

    // RMS-normalised kept columns a = phi / rms (as ridge_fit), their Gram
    // matrix g = a^T a, and C = g / N.
    std::vector<Column> a(n);
    for (std::size_t i = 0; i < n; ++i) {
      for (int c = 0; c < k; ++c) {
        a[i][c] = phi[i][j][kept[c]] / joint.column_rms[kept[c]];
      }
    }
    Matrix g{};
    Matrix gram{};
    Column rhs{};
    for (int p = 0; p < k; ++p) {
      for (int q = 0; q < k; ++q) {
        double sum = 0.0;
        for (std::size_t i = 0; i < n; ++i) sum += a[i][p] * a[i][q];
        g[p][q] = sum;
        gram[p][q] = sum / count;
      }
      double sum = 0.0;
      for (std::size_t i = 0; i < n; ++i) sum += a[i][p] * sj[i];
      rhs[p] = sum;
    }
    const auto [lambda_min, lambda_max] = extremeEigenvalues(gram, k);
    joint.lambda_min = lambda_min;
    joint.lambda_max = lambda_max;

    Column dw{};
    if (gradient) {
      if (options.gain * lambda_max >= 2.0) {
        invalid(where + ": joint " + std::to_string(j) + ": gain " +
                shortNumber(options.gain) + " times lambda_max " +
                shortNumber(lambda_max) +
                " of the normalised regressor Gram matrix is >= 2, so the "
                "normalised-gradient iteration diverges; the largest "
                "admissible gain is below 2 / lambda_max = " +
                shortNumber(2.0 / lambda_max));
      }
      for (int c = 0; c < k; ++c) {
        const int l = kept[c];
        double cross = 0.0;
        double square = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
          cross += phi[i][j][l] * sj[i];
          square += phi[i][j][l] * phi[i][j][l];
        }
        dw[c] = options.gain * (cross / square);
      }
    } else {
      if (lambda_min + options.ridge <=
          kLeastSquaresConditionLimit * (lambda_max + options.ridge)) {
        invalid(where + ": joint " + std::to_string(j) +
                ": the kept regressor columns are numerically dependent "
                "(lambda_min " +
                shortNumber(lambda_min) + ", lambda_max " +
                shortNumber(lambda_max) + ", ridge " +
                shortNumber(options.ridge) +
                "); a least-squares step needs ridge > 0 here");
      }
      Matrix normal = g;
      for (int p = 0; p < k; ++p) normal[p][p] += options.ridge * count;
      const Column x = choleskySolve(
          normal, rhs, k, where + ": joint " + std::to_string(j));
      for (int c = 0; c < k; ++c) {
        dw[c] = options.gain * (x[c] / joint.column_rms[kept[c]]);
      }
    }
    for (int c = 0; c < k; ++c) {
      const int l = kept[c];
      next.w[j][l] = w_k.w[j][l] + dw[c];
      if (!std::isfinite(next.w[j][l])) {
        invalid(where + ": the updated " + weightKey(j, l) + " is not finite");
      }
    }
    std::vector<double> residual = sj;
    for (std::size_t i = 0; i < n; ++i) {
      for (int c = 0; c < k; ++c) {
        const int l = kept[c];
        residual[i] -= phi[i][j][l] * (next.w[j][l] - w_k.w[j][l]);
      }
    }
    joint.residual_rms = rms(residual);
  }
  if (report != nullptr) *report = local;
  return next;
}

}  // namespace rtctrl::learn
