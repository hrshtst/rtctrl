#include "rtctrl/learn/regressor_feedforward.hpp"

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

using Catch::Matchers::ContainsSubstring;
using rtctrl::learn::FeedforwardWeights;
using rtctrl::learn::FelOptions;
using rtctrl::learn::FelReport;
using rtctrl::learn::JointArray;
using rtctrl::learn::kRegressorCount;
using rtctrl::learn::RegressorOptions;
using rtctrl::learn::RegressorRow;
using rtctrl::model::ChainModel;
using rtctrl::model::JointMap;
using rtctrl::model::kCanonicalDof;
using rtctrl::model::ZVector;
namespace learn = rtctrl::learn;

namespace {

namespace fs = std::filesystem;

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr int kArm = 7;  // arm joints 0..6
constexpr int kGripper = 7;
constexpr double kEps = std::numeric_limits<double>::epsilon();
constexpr double kPi = 3.14159265358979323846;
constexpr int kG = learn::kRegressorGravity;
constexpr int kDyn = learn::kRegressorDynamics;
constexpr int kDdq = learn::kRegressorAcceleration;
constexpr int kDq = learn::kRegressorVelocity;
constexpr int kCoulomb = learn::kRegressorCoulomb;
constexpr int kConstant = learn::kRegressorConstant;

using Weights =
    std::array<std::array<double, kRegressorCount>, kCanonicalDof>;
using Row = std::array<double, kRegressorCount>;

bool sameBits(double a, double b) {
  return std::memcmp(&a, &b, sizeof a) == 0;
}

bool sameBits(const Row& a, const Row& b) {
  return std::memcmp(a.data(), b.data(), sizeof a) == 0;
}

// Fixed-seed standard normal samples. std::mt19937_64 is specified bit for
// bit; the map to doubles and Box-Muller are written out because
// std::normal_distribution is implementation-defined.
class Gaussian {
 public:
  explicit Gaussian(std::uint64_t seed) : engine_(seed) {}
  double operator()() {
    const double u1 = uniform();
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
  }

 private:
  double uniform() {  // in (0, 1)
    return (static_cast<double>(engine_() >> 11) + 0.5) * 0x1.0p-53;
  }
  std::mt19937_64 engine_;
};

const JointArray kPosture = {0.3, -0.6, 0.2, -1.1, 0.4, 0.7, -0.5, 0.2};

// Every joint moves about a posture that loads the arm: two sines per
// joint, well inside the joint ranges (the gripper moves less).
std::vector<JointArray> movingReference(std::size_t n, double dt) {
  std::vector<JointArray> q(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * dt;
    for (int j = 0; j < kCanonicalDof; ++j) {
      const double scale = j == kGripper ? 0.15 : 1.0;
      q[i][j] = kPosture[j] +
                scale * (0.3 * std::sin(2.0 * kPi * (0.4 + 0.07 * j) * t +
                                        0.5 * j) +
                         0.1 * std::sin(2.0 * kPi * (1.1 + 0.05 * j) * t));
    }
  }
  return q;
}

// q(t) = a + b t + c t^2 on every joint.
struct Quadratic {
  JointArray a = kPosture;
  JointArray b{};
  JointArray c{};
  Quadratic() {
    for (int j = 0; j < kCanonicalDof; ++j) {
      b[j] = 0.2 - 0.07 * j;
      c[j] = 0.5 - 0.13 * j;
    }
  }
  double q(int j, double t) const { return a[j] + b[j] * t + c[j] * t * t; }
  double dq(int j, double t) const { return b[j] + 2.0 * c[j] * t; }
  double ddq(int j) const { return 2.0 * c[j]; }
};

std::vector<double> column(const std::vector<JointArray>& series, int j) {
  std::vector<double> out;
  for (const auto& sample : series) out.push_back(sample[j]);
  return out;
}

// The definition of zero-phase Gaussian smoothing, written independently of
// the library: explicit edge padding, unnormalised weights, normalisation at
// the end.
int halfWidth(double sigma, double dt) {
  return std::max(1, static_cast<int>(std::ceil(4.0 * sigma / dt - 1e-9)));
}

std::vector<double> plainSmooth(const std::vector<double>& x, double sigma,
                                double dt) {
  if (sigma == 0.0) return x;
  const int half = halfWidth(sigma, dt);
  std::vector<double> padded(static_cast<std::size_t>(half), x.front());
  padded.insert(padded.end(), x.begin(), x.end());
  padded.insert(padded.end(), static_cast<std::size_t>(half), x.back());
  std::vector<double> weight;
  for (int m = -half; m <= half; ++m) {
    weight.push_back(std::exp(-0.5 * std::pow(m * dt / sigma, 2)));
  }
  double total = 0.0;
  for (const double w : weight) total += w;
  std::vector<double> out(x.size());
  for (std::size_t i = 0; i < x.size(); ++i) {
    double acc = 0.0;
    for (std::size_t k = 0; k < weight.size(); ++k) {
      acc += weight[k] * padded[i + k];
    }
    out[i] = acc / total;
  }
  return out;
}

// numpy.gradient, uniform spacing, edge_order 1 (from its documentation).
std::vector<double> plainGradient(const std::vector<double>& f, double dt) {
  const std::size_t n = f.size();
  std::vector<double> out(n);
  for (std::size_t i = 1; i + 1 < n; ++i) {
    out[i] = (f[i + 1] - f[i - 1]) / (2.0 * dt);
  }
  out.front() = (f[1] - f[0]) / dt;
  out.back() = (f[n - 1] - f[n - 2]) / dt;
  return out;
}

double maxAbs(const std::vector<double>& x) {
  double out = 0.0;
  for (const double v : x) out = std::max(out, std::fabs(v));
  return out;
}

double rms(const std::vector<double>& x) {
  double sum = 0.0;
  for (const double v : x) sum += v * v;
  return std::sqrt(sum / static_cast<double>(x.size()));
}

// Two weighted means of the same 2h + 1 values, summed in different orders
// with the normalisation at a different place: each rounds to within about
// (2h + 1) eps max|x|; 10 times that bounds their difference.
double smoothTolerance(int half, double max_abs) {
  return 10.0 * (2 * half + 1) * kEps * max_abs;
}

struct Torques {
  JointArray gravity{};
  JointArray dynamics{};
};

Torques modelTorques(ChainModel& chain, const JointMap& map,
                     const JointArray& q, const JointArray& dq,
                     const JointArray& ddq) {
  ZVector zq(kCanonicalDof), zdq(kCanonicalDof), zddq(kCanonicalDof);
  ZVector gravity(kCanonicalDof), tau(kCanonicalDof);
  for (int j = 0; j < kCanonicalDof; ++j) {
    zq[j] = q[j];
    zdq[j] = dq[j];
    zddq[j] = ddq[j];
  }
  chain.gravityTorque(map, zq, gravity);
  chain.inverseDynamics(map, zq, zdq, zddq, tau);
  Torques out;
  for (int j = 0; j < kCanonicalDof; ++j) {
    out.gravity[j] = gravity[j];
    out.dynamics[j] = tau[j];
  }
  return out;
}

// ----------------------------------------------- synthetic regressors

constexpr std::array<double, kRegressorCount> kColumnScale = {2.0,  0.3, 15.0,
                                                              0.05, 1.0, 7.0};

double columnScale(int j, int l) { return kColumnScale[l] * (1.0 + 0.1 * j); }

// Column l of arm joint j is scale_jl * (alpha_j + beta_j * b_jl) with
// b_jl(i) = sqrt(2) cos(2 pi p_jl i / N) for the distinct integers
// p_jl = 2 + 3 l + j in (0, N/2): zero mean and orthonormal on the grid. So
// the column RMS is scale_jl (alpha^2 + beta^2 = 1) and the normalised Gram
// matrix is C = (1 - rho) I + rho 1 1^T with rho_j = alpha_j^2: eigenvalue
// 1 + 5 rho along 1 / sqrt(6) and 1 - rho on its complement (five-fold).
std::vector<RegressorRow> equicorrelated(std::size_t n,
                                         const std::array<double, kArm>& rho) {
  std::vector<RegressorRow> rows(n);
  for (std::size_t i = 0; i < n; ++i) {
    for (auto& joint : rows[i]) joint.fill(0.0);
    for (int j = 0; j < kArm; ++j) {
      for (int l = 0; l < kRegressorCount; ++l) {
        const double p = 2.0 + 3.0 * l + j;
        const double b = std::sqrt(2.0) *
                         std::cos(2.0 * kPi * p * static_cast<double>(i) /
                                  static_cast<double>(n));
        rows[i][j][l] = columnScale(j, l) *
                        (std::sqrt(rho[j]) + std::sqrt(1.0 - rho[j]) * b);
      }
    }
  }
  return rows;
}

// Planted weights of about 0.01 rad per unit-RMS column.
Weights plantedWeights(const std::array<Row, kArm>& column_rms) {
  Weights w{};
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      w[j][l] = 0.01 * std::sin(1.3 * j + 0.7 * l + 0.2) / column_rms[j][l];
    }
  }
  return w;
}

std::array<Row, kArm> syntheticScales() {
  std::array<Row, kArm> out{};
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) out[j][l] = columnScale(j, l);
  }
  return out;
}

// The tracking error of a unit-gain plant whose remaining error is exactly
// what the current weights miss, `shift` samples late:
// e[i + shift] = sum_l phi[i][j][l] (planted - current)[j][l], i in 0..N-1.
// Before the shift the error is an unrelated ramp; after the grid it holds
// the last value; the gripper error is a constant the update never uses.
std::vector<JointArray> plantError(const std::vector<RegressorRow>& phi,
                                   const Weights& planted,
                                   const Weights& current, std::size_t shift,
                                   std::size_t tail) {
  REQUIRE(tail >= shift);
  const std::size_t n = phi.size();
  std::vector<JointArray> e(n + tail);
  for (std::size_t m = 0; m < shift; ++m) {
    for (int j = 0; j < kArm; ++j) e[m][j] = 0.02 * (1.0 + j) * (m + 1.0);
    e[m][kGripper] = 0.01;
  }
  for (std::size_t i = 0; i < n; ++i) {
    for (int j = 0; j < kArm; ++j) {
      double sum = 0.0;
      for (int l = 0; l < kRegressorCount; ++l) {
        sum += phi[i][j][l] * (planted[j][l] - current[j][l]);
      }
      e[i + shift][j] = sum;
    }
    e[i + shift][kGripper] = 0.01;
  }
  for (std::size_t m = n + shift; m < e.size(); ++m) e[m] = e[n + shift - 1];
  return e;
}

// sqrt(sum_l (rms_l (a_l - b_l))^2): a weight difference in RMS-normalised
// units.
double normalisedDistance(const Row& a, const Row& b, const Row& column_rms) {
  double sum = 0.0;
  for (int l = 0; l < kRegressorCount; ++l) {
    const double d = column_rms[l] * (a[l] - b[l]);
    sum += d * d;
  }
  return std::sqrt(sum);
}

FelOptions exactPlantOptions(FelOptions::Step step, double gain) {
  FelOptions options;
  options.step = step;
  options.gain = gain;
  options.advance_s = 0.05;           // 5 samples at 10 ms
  options.error_smooth_sigma_s = 0.0;  // the plant error is exact
  options.error_lead_s = 0.0;
  return options;
}

constexpr double kDt = 0.01;
constexpr std::size_t kShift = 5;  // advance_s 0.05 at kDt
constexpr std::size_t kTail = 8;

// ------------------------------------------------------------ weights

const std::string kHashParent =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
const std::string kHashLog(64, 'f');
const std::string kHashReference(64, '0');

FeedforwardWeights sampleWeights() {
  FeedforwardWeights w;
  w.trial = 2;
  w.parent_sha256 = kHashParent;
  w.source_log_sha256 = kHashLog;
  w.reference_sha256 = kHashReference;
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      w.w[j][l] = 0.5 * (j + 1) + 0.25 * l;
    }
  }
  return w;
}

std::string replaced(std::string text, const std::string& from,
                     const std::string& to) {
  const auto at = text.find(from);
  REQUIRE(at != std::string::npos);
  REQUIRE(text.find(from, at + 1) == std::string::npos);
  return text.replace(at, from.size(), to);
}

void expectReadError(const std::string& text,
                     const std::vector<std::string>& phrases) {
  try {
    learn::parseFeedforwardWeights(text, "case.toml");
    FAIL("parse succeeded, expected an error");
  } catch (const std::runtime_error& error) {
    CHECK_THAT(error.what(), ContainsSubstring("follow weights 'case.toml'"));
    for (const auto& phrase : phrases) {
      CHECK_THAT(error.what(), ContainsSubstring(phrase));
    }
  }
}

std::string fileText(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), {});
}

fs::path scratchDir() {
  const fs::path dir = "build/regressor_feedforward_test";
  fs::create_directories(dir);
  return dir;
}

template <class F>
void expectInvalid(F&& call, const std::vector<std::string>& phrases) {
  try {
    call();
    FAIL("call succeeded, expected std::invalid_argument");
  } catch (const std::invalid_argument& error) {
    for (const auto& phrase : phrases) {
      CHECK_THAT(error.what(), ContainsSubstring(phrase));
    }
  }
}

// ------------------------------------- the update with the default smoothing

// Every joint moves inside a sin^4 window, so the reference, its rate and
// its acceleration vanish at both ends (at rest, as a planned reference is):
// the regressors are constant there and the plant's hold before and after
// the grid continues them smoothly.
std::vector<JointArray> restToRestReference(std::size_t n, double dt) {
  std::vector<JointArray> q(n);
  const double duration = static_cast<double>(n - 1) * dt;
  for (std::size_t i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * dt;
    const double s = std::sin(kPi * t / duration);
    const double window = s * s * s * s;
    for (int j = 0; j < kCanonicalDof; ++j) {
      const double scale = j == kGripper ? 0.15 : 1.0;
      q[i][j] = kPosture[j] +
                scale * window *
                    (0.5 * std::sin(2.0 * kPi * (0.4 + 0.07 * j) * t + 0.5 * j) +
                     0.15 * std::sin(2.0 * kPi * (1.1 + 0.05 * j) * t));
    }
  }
  return q;
}

std::array<Row, kArm> columnRms(const std::vector<RegressorRow>& phi) {
  std::array<Row, kArm> out{};
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      double sum = 0.0;
      for (const auto& row : phi) sum += row[j][l] * row[j][l];
      out[j][l] = std::sqrt(sum / static_cast<double>(phi.size()));
    }
  }
  return out;
}

// Planted weights of about 0.01 rad per unit-RMS column; zero on the
// columns the update skips (joint 0's gravity).
Weights plantedOnKept(const std::array<Row, kArm>& column_rms) {
  Weights w{};
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      if (column_rms[j][l] < 1e-6) continue;
      w[j][l] = 0.01 * std::sin(1.3 * j + 0.7 * l + 0.2) / column_rms[j][l];
    }
  }
  return w;
}

// The plant's error is exactly what the weights miss, `shift` samples late,
// held before the shift and after the grid:
// e[m] = sum_l phi[clamp(m - shift, 0, N - 1)][j][l] (planted - current)[j][l].
// It vanishes identically when the weights are the planted ones, so those
// stay the fixed point of the update even through the smoother.
std::vector<JointArray> heldPlantError(const std::vector<RegressorRow>& phi,
                                       const Weights& planted,
                                       const Weights& current,
                                       std::size_t shift, std::size_t tail) {
  const std::size_t n = phi.size();
  std::vector<JointArray> e(n + tail);
  for (std::size_t m = 0; m < e.size(); ++m) {
    const std::size_t i = m < shift ? 0 : std::min(m - shift, n - 1);
    for (int j = 0; j < kCanonicalDof; ++j) {
      double sum = 0.0;
      for (int l = 0; l < kRegressorCount; ++l) {
        sum += phi[i][j][l] * (planted[j][l] - current[j][l]);
      }
      e[m][j] = sum;
    }
  }
  return e;
}

// RMS over the grid of joint j's remaining error Phi (w* - w).
double remainingRms(const std::vector<RegressorRow>& phi,
                    const Weights& planted, const Weights& w, int j) {
  double sum = 0.0;
  for (const auto& row : phi) {
    double e = 0.0;
    for (int l = 0; l < kRegressorCount; ++l) {
      e += row[j][l] * (planted[j][l] - w[j][l]);
    }
    sum += e * e;
  }
  return std::sqrt(sum / static_cast<double>(phi.size()));
}

using Dense = std::vector<std::vector<double>>;

Dense denseZero(std::size_t k) {
  return Dense(k, std::vector<double>(k, 0.0));
}

Dense product(const Dense& a, const Dense& b) {
  Dense out = denseZero(a.size());
  for (std::size_t p = 0; p < a.size(); ++p) {
    for (std::size_t q = 0; q < a.size(); ++q) {
      for (std::size_t r = 0; r < a.size(); ++r) out[p][q] += a[p][r] * b[r][q];
    }
  }
  return out;
}

Dense transposed(const Dense& a) {
  Dense out = denseZero(a.size());
  for (std::size_t p = 0; p < a.size(); ++p) {
    for (std::size_t q = 0; q < a.size(); ++q) out[q][p] = a[p][q];
  }
  return out;
}

std::vector<double> times(const Dense& a, const std::vector<double>& x) {
  std::vector<double> out(a.size(), 0.0);
  for (std::size_t p = 0; p < a.size(); ++p) {
    for (std::size_t q = 0; q < a.size(); ++q) out[p] += a[p][q] * x[q];
  }
  return out;
}

double quadratic(const Dense& a, const std::vector<double>& x) {
  const auto ax = times(a, x);
  double sum = 0.0;
  for (std::size_t p = 0; p < x.size(); ++p) sum += x[p] * ax[p];
  return sum;
}

// Gaussian elimination with partial pivoting (a different algorithm from the
// library's Cholesky).
std::vector<double> solveDense(Dense a, std::vector<double> b) {
  const std::size_t k = b.size();
  for (std::size_t c = 0; c < k; ++c) {
    std::size_t pivot = c;
    for (std::size_t r = c + 1; r < k; ++r) {
      if (std::fabs(a[r][c]) > std::fabs(a[pivot][c])) pivot = r;
    }
    std::swap(a[c], a[pivot]);
    std::swap(b[c], b[pivot]);
    REQUIRE(a[c][c] != 0.0);
    for (std::size_t r = c + 1; r < k; ++r) {
      const double f = a[r][c] / a[c][c];
      for (std::size_t q = c; q < k; ++q) a[r][q] -= f * a[c][q];
      b[r] -= f * b[c];
    }
  }
  std::vector<double> x(k);
  for (std::size_t c = k; c-- > 0;) {
    double v = b[c];
    for (std::size_t q = c + 1; q < k; ++q) v -= a[c][q] * x[q];
    x[c] = v / a[c][c];
  }
  return x;
}

// Whether the symmetric part of q is positive definite (Cholesky succeeds).
bool positiveDefinite(const Dense& q) {
  const std::size_t k = q.size();
  Dense l = denseZero(k);
  for (std::size_t j = 0; j < k; ++j) {
    double d = q[j][j];
    for (std::size_t p = 0; p < j; ++p) d -= l[j][p] * l[j][p];
    if (!(d > 0.0)) return false;
    l[j][j] = std::sqrt(d);
    for (std::size_t i = j + 1; i < k; ++i) {
      double v = 0.5 * (q[i][j] + q[j][i]);
      for (std::size_t p = 0; p < j; ++p) v -= l[i][p] * l[j][p];
      l[i][j] = v / l[j][j];
    }
  }
  return true;
}

// One joint's update in RMS-normalised weight units u = rms * (w* - w), as
// the test derives it with its own smoother. The held plant makes the error
// record linear in u: e = sum_c u_c A_c, A_c the held record of normalised
// column a_c. The error signal is then s = sum_c u_c S_c, S_c the signal of
// A_c (smoothing, rate, advance, lead), so one update is
//   normalised gradient: u <- (I - gain M) u,
//   least squares:       u <- (I - gain (C + ridge I)^-1 M) u,
// with C = mean(a_p a_q) and M = mean(a_p S_q): the regressors as the error
// signal path sees them. Without smoothing or lead S_c = a_c and M = C; the
// smoother's attenuation of the regressors is M - C. RMS(Phi (w* - w))^2 is
// u^T C u.
struct SignalPath {
  std::vector<int> kept;
  std::vector<double> scale;
  std::vector<std::vector<double>> a;       // [column][sample]
  std::vector<std::vector<double>> signal;  // S_c, [column][sample]
  Dense c;
  Dense m;

  std::vector<double> u(const Weights& planted, const Weights& w,
                        int j) const {
    std::vector<double> out;
    for (std::size_t k = 0; k < kept.size(); ++k) {
      out.push_back(scale[k] * (planted[j][kept[k]] - w[j][kept[k]]));
    }
    return out;
  }
};

SignalPath signalPath(const std::vector<RegressorRow>& phi, int j,
                      const FelOptions& options, std::size_t shift,
                      std::size_t tail) {
  REQUIRE(options.advance_s / kDt == static_cast<double>(shift));
  const std::size_t n = phi.size();
  SignalPath path;
  const auto column_rms = columnRms(phi);
  for (int l = 0; l < kRegressorCount; ++l) {
    if (column_rms[j][l] < options.negligible_rms) continue;
    path.kept.push_back(l);
    path.scale.push_back(column_rms[j][l]);
  }
  const std::size_t k = path.kept.size();
  for (std::size_t c = 0; c < k; ++c) {
    std::vector<double> a(n);
    for (std::size_t i = 0; i < n; ++i) {
      a[i] = phi[i][j][path.kept[c]] / path.scale[c];
    }
    std::vector<double> record(n + tail);
    for (std::size_t m = 0; m < record.size(); ++m) {
      record[m] = a[m < shift ? 0 : std::min(m - shift, n - 1)];
    }
    const auto smooth =
        plainSmooth(record, options.error_smooth_sigma_s, kDt);
    const auto rate = plainGradient(smooth, kDt);
    std::vector<double> signal(n);
    for (std::size_t i = 0; i < n; ++i) {
      signal[i] = smooth[i + shift] + options.error_lead_s * rate[i + shift];
    }
    path.a.push_back(a);
    path.signal.push_back(signal);
  }
  path.c = denseZero(k);
  path.m = denseZero(k);
  for (std::size_t p = 0; p < k; ++p) {
    for (std::size_t q = 0; q < k; ++q) {
      double cc = 0.0;
      double mm = 0.0;
      for (std::size_t i = 0; i < n; ++i) {
        cc += path.a[p][i] * path.a[q][i];
        mm += path.a[p][i] * path.signal[q][i];
      }
      path.c[p][q] = cc / static_cast<double>(n);
      path.m[p][q] = mm / static_cast<double>(n);
    }
  }
  return path;
}

// The least-squares step at gain 1 in normalised units.
std::vector<double> leastSquaresStep(const SignalPath& path,
                                     const std::vector<double>& u,
                                     double ridge) {
  Dense normal = path.c;
  for (std::size_t p = 0; p < normal.size(); ++p) normal[p][p] += ridge;
  const auto du = solveDense(normal, times(path.m, u));
  std::vector<double> out(u.size());
  for (std::size_t p = 0; p < u.size(); ++p) out[p] = u[p] - du[p];
  return out;
}

// The smoother's relative distortion of the remaining error a u:
// RMS(s - a u) / RMS(a u), s = sum_c u_c S_c.
double signalDistortion(const SignalPath& path, const std::vector<double>& u) {
  const std::size_t n = path.a.front().size();
  double distortion = 0.0;
  double size = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    double e = 0.0;
    double s = 0.0;
    for (std::size_t c = 0; c < u.size(); ++c) {
      e += u[c] * path.a[c][i];
      s += u[c] * path.signal[c][i];
    }
    distortion += (s - e) * (s - e);
    size += e * e;
  }
  return std::sqrt(distortion / size);
}

constexpr std::size_t kModelSamples = 300;

// (a) for the given lead, all other options at their defaults.
void checkLeastSquaresContraction(double lead) {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto phi = learn::buildRegressors(
      chain, map, restToRestReference(kModelSamples, kDt), kDt, {});
  FelOptions options;  // the defaults: sigma 0.04 s, advance 0.05 s, ...
  REQUIRE(options.error_smooth_sigma_s == 0.04);
  REQUIRE(options.ridge == 1e-6);
  options.step = FelOptions::Step::LeastSquares;
  options.gain = 1.0;
  options.error_lead_s = lead;
  const Weights planted = plantedOnKept(columnRms(phi));

  // A priori: the factor of the first update on RMS(Phi (w* - w)) from the
  // derived operator, u1 = (I - (C + ridge I)^-1 M) u0.
  std::array<SignalPath, kArm> paths;
  std::array<double, kArm> predicted{};
  for (int j = 0; j < kArm; ++j) {
    paths[j] = signalPath(phi, j, options, kShift, kTail);
    const auto u0 = paths[j].u(planted, Weights{}, j);
    const auto u1 = leastSquaresStep(paths[j], u0, options.ridge);
    predicted[j] = std::sqrt(quadratic(paths[j].c, u1) /
                             quadratic(paths[j].c, u0));
    INFO("joint " << j << ": predicted first-update factor " << predicted[j]
                  << ", smoother distortion of the planted error "
                  << signalDistortion(paths[j], u0));
    // A large factor: at least 5 in one update.
    CHECK(predicted[j] <= 0.2);
  }

  FeedforwardWeights w;
  std::array<double, kArm> initial{};
  for (int j = 0; j < kArm; ++j) initial[j] = remainingRms(phi, planted, w.w, j);
  constexpr int kUpdates = 5;
  for (int k = 0; k < kUpdates; ++k) {
    FelReport report;
    const auto next = learn::felUpdate(
        w, phi, heldPlantError(phi, planted, w.w, kShift, kTail), kDt,
        options, &report);
    for (int j = 0; j < kArm; ++j) {
      const auto& r = report.joints[j];
      const double before = remainingRms(phi, planted, w.w, j);
      const double after = remainingRms(phi, planted, next.w, j);
      const double kappa = signalDistortion(paths[j], paths[j].u(planted, w.w, j));
      const double ridge_term = options.ridge / (r.lambda_min + options.ridge);
      INFO("update " << k << ", joint " << j << ": factor " << after / before
                     << ", bound " << kappa + ridge_term << " (distortion "
                     << kappa << ")");
      // The tolerances below allow for the rounding of two different
      // computations of the same operator (smoothing sums of 33 terms in
      // different orders, about 1e-14 relative), amplified by the solve
      // (cond(C + ridge I) < 2e4, checked) and by passing between u and
      // RMS(Phi (w* - w)) (sqrt(lambda_max / lambda_min) < 150): below 1e-8.
      REQUIRE(r.lambda_max / r.lambda_min < 2e4);
      if (k == 0) CHECK(std::fabs(after / before - predicted[j]) <= 1e-6);
      // Rigorous bound at every update: with H the ridge hat matrix
      // (eigenvalues in [0, 1)), the remaining error after a gain-1 step is
      // ridge a (C + ridge I)^-1 u - H (s - a u), so its RMS is at most
      // RMS(s - a u) + ridge / (lambda_min + ridge) RMS(a u).
      CHECK(after <= (kappa + ridge_term + 1e-6) * before);
    }
    w = next;
  }
  for (int j = 0; j < kArm; ++j) {
    const double fraction = remainingRms(phi, planted, w.w, j) / initial[j];
    INFO("joint " << j << ": remaining after " << kUpdates
                  << " updates: " << fraction);
    CHECK(fraction <= 1e-2);  // a small fraction within a few updates
  }
}

// (b) the normalised gradient at the default gain over a run of updates:
// returns, per joint, the RMS(Phi (w* - w)) after every update relative to
// the start, and checks the library against the derived iteration.
std::array<std::vector<double>, kArm> runNormalisedGradient(
    const std::vector<RegressorRow>& phi, const Weights& planted,
    const FelOptions& options, const std::array<SignalPath, kArm>& paths,
    int updates, std::array<std::vector<double>, kArm>* weight_error) {
  std::array<std::vector<double>, kArm> remaining;
  std::array<std::vector<double>, kArm> predicted_u;
  for (int j = 0; j < kArm; ++j) predicted_u[j] = paths[j].u(planted, Weights{}, j);
  std::array<double, kArm> initial{};
  FeedforwardWeights w;
  for (int j = 0; j < kArm; ++j) {
    initial[j] = remainingRms(phi, planted, w.w, j);
    remaining[j].push_back(1.0);
    const auto u = paths[j].u(planted, w.w, j);
    double norm = 0.0;
    for (const double v : u) norm += v * v;
    (*weight_error)[j].push_back(std::sqrt(norm));
  }
  for (int k = 0; k < updates; ++k) {
    w = learn::felUpdate(w, phi,
                         heldPlantError(phi, planted, w.w, kShift, kTail), kDt,
                         options);
    for (int j = 0; j < kArm; ++j) {
      const double rms_now = remainingRms(phi, planted, w.w, j);
      remaining[j].push_back(rms_now / initial[j]);
      const auto u = paths[j].u(planted, w.w, j);
      double norm = 0.0;
      for (const double v : u) norm += v * v;
      (*weight_error)[j].push_back(std::sqrt(norm));
      // The derived iteration u <- (I - gain M) u, run alongside. The two
      // differ by rounding: about 1e-14 |u| per update from the smoothing
      // sums, not amplified (the iteration contracts, checked by the
      // caller), over at most 100 updates, seen through
      // sqrt(lambda_max / lambda_min) < 150: below 1e-9 of the start.
      const auto mu = times(paths[j].m, predicted_u[j]);
      for (std::size_t c = 0; c < mu.size(); ++c) {
        predicted_u[j][c] -= options.gain * mu[c];
      }
      const double predicted =
          std::sqrt(quadratic(paths[j].c, predicted_u[j])) / initial[j];
      INFO("update " << k << ", joint " << j);
      CHECK(std::fabs(rms_now / initial[j] - predicted) <= 1e-9);
    }
  }
  return remaining;
}

}  // namespace

// =========================================================== regressors

TEST_CASE("regressor feedforward: the names follow the analysis tool",
          "[learn][regressor_feedforward]") {
  // tools/follow_learnability.py REGRESSORS, in order.
  const std::array<std::string, kRegressorCount> python = {
      "g", "tau_dyn", "ddq", "dq", "coulomb", "constant"};
  for (int l = 0; l < kRegressorCount; ++l) {
    CHECK(learn::kRegressorNames[l] == python[l]);
  }
  CHECK(kG == 0);
  CHECK(kDyn == 1);
  CHECK(kDdq == 2);
  CHECK(kDq == 3);
  CHECK(kCoulomb == 4);
  CHECK(kConstant == 5);
}

TEST_CASE("regressor feedforward: numpy.gradient semantics",
          "[learn][regressor_feedforward]") {
  // f = (i + 1)^2 at dt 0.5: every difference below is exact in binary.
  const auto g = learn::numpyGradient({1.0, 4.0, 9.0, 16.0}, 0.5);
  REQUIRE(g.size() == 4);
  CHECK(g[0] == 6.0);   // (4 - 1) / 0.5, one-sided
  CHECK(g[1] == 8.0);   // (9 - 1) / 1
  CHECK(g[2] == 12.0);  // (16 - 4) / 1
  CHECK(g[3] == 14.0);  // (16 - 9) / 0.5, one-sided
  const auto two = learn::numpyGradient({1.0, 3.0}, 0.5);
  CHECK(two == std::vector<double>{4.0, 4.0});
  expectInvalid([] { learn::numpyGradient({1.0}, 0.5); },
                {"at least 2 samples"});
  expectInvalid([] { learn::numpyGradient({1.0, 2.0}, 0.0); }, {"dt"});
}

TEST_CASE(
    "regressor feedforward: a quadratic reference gives the analytic interior "
    "rates and the model's torques",
    "[learn][regressor_feedforward]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const Quadratic quad;
  constexpr std::size_t n = 40;
  std::vector<JointArray> qref(n);
  for (std::size_t i = 0; i < n; ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      qref[i][j] = quad.q(j, static_cast<double>(i) * kDt);
    }
  }
  const auto ref = learn::smoothReference(qref, kDt, 0.0);
  REQUIRE(ref.q.size() == n);
  // sigma 0 is no smoothing.
  CHECK(ref.q == qref);
  // Central differences are exact for a quadratic: what is left is the
  // rounding of q (|q| < 2, a few eps each) divided by 2 dt, below 1e-13
  // rad/s, and once more for ddq, below 1e-11 rad/s^2. Tolerances 100x
  // that. dq is exact on 1..N-2; ddq on 2..N-3 (it uses the one-sided dq
  // at the ends).
  for (std::size_t i = 1; i + 1 < n; ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) {
      INFO("sample " << i << ", joint " << j);
      CHECK(std::fabs(ref.dq[i][j] - quad.dq(j, static_cast<double>(i) * kDt)) <
            1e-11);
      if (i >= 2 && i + 2 < n) {
        CHECK(std::fabs(ref.ddq[i][j] - quad.ddq(j)) < 1e-9);
      }
    }
  }

  RegressorOptions options;
  options.smooth_sigma_s = 0.0;
  options.coulomb_v0_rad_s = 0.05;
  const auto phi = learn::buildRegressors(chain, map, qref, kDt, options);
  REQUIRE(phi.size() == n);
  double largest_gravity = 0.0;
  double largest_dynamic = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    // The same function at the same state: exact.
    const Torques direct =
        modelTorques(chain, map, ref.q[i], ref.dq[i], ref.ddq[i]);
    for (int j = 0; j < kArm; ++j) {
      INFO("sample " << i << ", joint " << j);
      CHECK(phi[i][j][kG] == direct.gravity[j]);
      CHECK(phi[i][j][kDyn] == direct.dynamics[j] - direct.gravity[j]);
      CHECK(phi[i][j][kDdq] == ref.ddq[i][j]);
      CHECK(phi[i][j][kDq] == ref.dq[i][j]);
      CHECK(phi[i][j][kCoulomb] == std::tanh(ref.dq[i][j] / 0.05));
      CHECK(phi[i][j][kConstant] == 1.0);
      largest_gravity = std::max(largest_gravity, std::fabs(phi[i][j][kG]));
      largest_dynamic = std::max(largest_dynamic, std::fabs(phi[i][j][kDyn]));
    }
    CHECK(phi[i][kGripper] == Row{});  // the gripper is never learned
    if (i >= 2 && i + 2 < n) {
      // And at the analytic state: the inputs agree to 1e-11 rad/s and
      // 1e-9 rad/s^2 (above), and the torques' sensitivity to them is of
      // order the arm's inertia (< 1 kg m^2), so 1e-8 Nm is a wide bound.
      JointArray dq{}, ddq{};
      for (int j = 0; j < kCanonicalDof; ++j) {
        dq[j] = quad.dq(j, static_cast<double>(i) * kDt);
        ddq[j] = quad.ddq(j);
      }
      const Torques analytic = modelTorques(chain, map, qref[i], dq, ddq);
      for (int j = 0; j < kArm; ++j) {
        INFO("sample " << i << ", joint " << j);
        CHECK(std::fabs(phi[i][j][kG] - analytic.gravity[j]) == 0.0);
        CHECK(std::fabs(phi[i][j][kDyn] -
                        (analytic.dynamics[j] - analytic.gravity[j])) < 1e-8);
      }
    }
  }
  CHECK(largest_gravity > 0.1);   // the posture loads the arm
  CHECK(largest_dynamic > 1e-3);  // the rates and accelerations matter
}

TEST_CASE(
    "regressor feedforward: the smoothing matches a straightforward "
    "implementation",
    "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 200;
  constexpr double sigma = 0.04;
  std::vector<JointArray> qref = movingReference(n, kDt);
  Gaussian noise(20261112);
  for (auto& sample : qref) {
    for (double& v : sample) v += 1e-3 * noise();
  }
  const auto ref = learn::smoothReference(qref, kDt, sigma);
  const int half = halfWidth(sigma, kDt);
  REQUIRE(half == 16);
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("joint " << j);
    const std::vector<double> raw = column(qref, j);
    const std::vector<double> q = plainSmooth(raw, sigma, kDt);
    const std::vector<double> dq = plainGradient(q, kDt);
    const std::vector<double> ddq = plainGradient(dq, kDt);
    // The smoothing difference d propagates to at most d / dt in dq and
    // d / dt^2 in ddq (a difference of two values over 2 dt or dt).
    const double tol = smoothTolerance(half, maxAbs(raw));
    for (std::size_t i = 0; i < n; ++i) {
      INFO("sample " << i);
      CHECK(std::fabs(ref.q[i][j] - q[i]) <= tol);
      CHECK(std::fabs(ref.dq[i][j] - dq[i]) <= tol / kDt);
      CHECK(std::fabs(ref.ddq[i][j] - ddq[i]) <= tol / (kDt * kDt));
    }
    // The public smoother is the one smoothReference uses.
    CHECK(learn::zeroPhaseGaussianSmooth(raw, sigma, kDt) == column(ref.q, j));
    // Smoothing changed something: the noise is 1e-3 rad.
    CHECK(maxAbs(column(ref.q, j)) > 0.0);
    double changed = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
      changed = std::max(changed, std::fabs(ref.q[i][j] - raw[i]));
    }
    CHECK(changed > 1e-4);
  }

  // The regressor columns are that smoothed reference's rates.
  ChainModel chain(kModelPath);
  JointMap map(chain);
  RegressorOptions options;
  options.smooth_sigma_s = sigma;
  options.coulomb_v0_rad_s = 0.1;
  const auto phi = learn::buildRegressors(chain, map, qref, kDt, options);
  for (std::size_t i = 0; i < n; ++i) {
    for (int j = 0; j < kArm; ++j) {
      CHECK(phi[i][j][kDq] == ref.dq[i][j]);
      CHECK(phi[i][j][kDdq] == ref.ddq[i][j]);
      CHECK(phi[i][j][kCoulomb] == std::tanh(ref.dq[i][j] / 0.1));
    }
    CHECK(phi[i][kGripper] == Row{});
  }
}

TEST_CASE("regressor feedforward: the kernel width and the record's edges",
          "[learn][regressor_feedforward]") {
  // An impulse shows the half-width. 4 * 0.07 / 0.01 rounds to
  // 28.000000000000004: the -1e-9 slack keeps it at 28 taps, as in Python.
  REQUIRE(4.0 * 0.07 / 0.01 > 28.0);
  std::vector<double> impulse(101, 0.0);
  impulse[50] = 1.0;
  const auto response = learn::zeroPhaseGaussianSmooth(impulse, 0.07, kDt);
  CHECK(response[50 - 28] > 0.0);
  CHECK(response[50 + 28] > 0.0);
  CHECK(response[50 - 29] == 0.0);
  CHECK(response[50 + 29] == 0.0);
  CHECK(response[50 - 28] == response[50 + 28]);  // zero phase

  // A kernel wider than the record leans on the repeated end values.
  const std::vector<double> short_series = {0.3, -0.2, 0.9, 0.1, 0.5};
  const auto wide = learn::zeroPhaseGaussianSmooth(short_series, 0.04, kDt);
  const auto expected = plainSmooth(short_series, 0.04, kDt);
  for (std::size_t i = 0; i < short_series.size(); ++i) {
    CHECK(std::fabs(wide[i] - expected[i]) <= smoothTolerance(16, 0.9));
  }
  // A sigma below dt / 4 still smooths over one tap per side (Python's
  // max(1, ...)).
  const auto narrow = learn::zeroPhaseGaussianSmooth(short_series, 0.001, kDt);
  const auto narrow_expected = plainSmooth(short_series, 0.001, kDt);
  for (std::size_t i = 0; i < short_series.size(); ++i) {
    CHECK(std::fabs(narrow[i] - narrow_expected[i]) <=
          smoothTolerance(1, 0.9));
  }
  // A constant stays constant (normalised weights, edge padding).
  const auto flat =
      learn::zeroPhaseGaussianSmooth(std::vector<double>(30, 0.25), 0.05, kDt);
  for (const double v : flat) CHECK(std::fabs(v - 0.25) <= 1e-15);
}

TEST_CASE("regressor feedforward: invalid regressor inputs are rejected",
          "[learn][regressor_feedforward]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto good = movingReference(10, kDt);
  const RegressorOptions options;
  CHECK(learn::buildRegressors(chain, map, good, kDt, options).size() == 10);

  const auto build = [&](std::vector<JointArray> qref, double dt,
                         RegressorOptions o) {
    return [&chain, &map, qref, dt, o] {
      learn::buildRegressors(chain, map, qref, dt, o);
    };
  };
  expectInvalid(build({good[0], good[1]}, kDt, options),
                {"at least 3 samples", "got 2"});
  auto bad = good;
  bad[4][2] = std::numeric_limits<double>::quiet_NaN();
  expectInvalid(build(bad, kDt, options),
                {"qref is not finite at sample 4, joint 2"});
  bad = good;
  bad[0][7] = std::numeric_limits<double>::infinity();
  expectInvalid(build(bad, kDt, options), {"sample 0, joint 7"});
  for (const double dt : {0.0, -0.01, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()}) {
    expectInvalid(build(good, dt, options), {"dt must be finite and > 0"});
  }
  RegressorOptions o = options;
  o.smooth_sigma_s = -0.01;
  expectInvalid(build(good, kDt, o), {"smooth_sigma_s must be finite"});
  o.smooth_sigma_s = std::numeric_limits<double>::quiet_NaN();
  expectInvalid(build(good, kDt, o), {"smooth_sigma_s must be finite"});
  o.smooth_sigma_s = 1e5;  // 4e7 taps per side
  expectInvalid(build(good, kDt, o), {"taps per side"});
  o = options;
  for (const double v0 : {0.0, -0.05, std::numeric_limits<double>::infinity()}) {
    o.coulomb_v0_rad_s = v0;
    expectInvalid(build(good, kDt, o), {"coulomb_v0_rad_s must be finite"});
  }
  expectInvalid([] { learn::validateRegressorOptions({-1.0, 0.05}); },
                {"regressor options: smooth_sigma_s"});
  expectInvalid([] { learn::zeroPhaseGaussianSmooth({}, 0.04, kDt); },
                {"empty"});
}

// ============================================================== weights

TEST_CASE("regressor feedforward: the weights file layout",
          "[learn][regressor_feedforward]") {
  FeedforwardWeights w;  // trial 0
  const std::string zeros = "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
  std::string expected =
      "format = \"rtctrl-x7-follow-weights\"\n"
      "version = 1\n"
      "trial = 0\n"
      "parent_sha256 = \"\"\n"
      "source_log_sha256 = \"\"\n"
      "reference_sha256 = \"\"\n"
      "\n"
      "[regressors]\n"
      "names = [\"g\", \"tau_dyn\", \"ddq\", \"dq\", \"coulomb\", "
      "\"constant\"]\n"
      "smooth_sigma_s = 0.040000000000000001\n"
      "coulomb_v0_rad_s = 0.050000000000000003\n"
      "\n"
      "# delta_j = sum_l w_jl * phi_jl: rad per regressor unit, in names "
      "order;\n"
      "# joint7 (the gripper) is never learned.\n"
      "[weights]\n";
  for (int j = 0; j < kCanonicalDof; ++j) {
    expected += "joint" + std::to_string(j) + " = " + zeros;
  }
  CHECK(learn::formatFeedforwardWeights(w) == expected);

  const auto text = learn::formatFeedforwardWeights(sampleWeights());
  CHECK_THAT(text, ContainsSubstring("trial = 2\n"));
  CHECK_THAT(text, ContainsSubstring("parent_sha256 = \"" + kHashParent));
  CHECK_THAT(text,
             ContainsSubstring("joint3 = [2.0, 2.25, 2.5, 2.75, 3.0, 3.25]\n"));
}

TEST_CASE(
    "regressor feedforward: a weights round trip is bit-exact and "
    "byte-identical",
    "[learn][regressor_feedforward]") {
  const std::vector<double> awkward = {
      0.1,
      1.0 / 3.0,
      -0.0,
      4.9406564584124654e-324,   // smallest subnormal
      2.2250738585072009e-308,   // largest subnormal
      -2.2250738585072014e-308,  // smallest normal
      1.7976931348623157e308,    // largest finite
      1e22,                      // integral, printed with an exponent
      9007199254740992.0,        // 2^53, printed without a point
      123456789012345678.0,
      1e-5,
      -1.0,
      2.5e-310,
      123.0,
      -7.0 / 3.0,
      kPi,
      0.1 + 0.2,
  };
  FeedforwardWeights w = sampleWeights();
  w.trial = 17;
  w.regressors.smooth_sigma_s = 0.1 + 0.2;
  w.regressors.coulomb_v0_rad_s = 1.0 / 3.0;
  std::size_t k = 0;
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      w.w[j][l] = awkward[k++ % awkward.size()];
    }
  }
  const std::string text = learn::formatFeedforwardWeights(w);
  CHECK_THAT(text, ContainsSubstring("-0.0, "));
  CHECK_THAT(text, ContainsSubstring("9007199254740992.0"));
  CHECK_THAT(text, ContainsSubstring("4.9406564584124654e-324"));

  const FeedforwardWeights read = learn::parseFeedforwardWeights(text, "mem");
  CHECK(read.trial == 17);
  CHECK(read.parent_sha256 == w.parent_sha256);
  CHECK(read.source_log_sha256 == w.source_log_sha256);
  CHECK(read.reference_sha256 == w.reference_sha256);
  CHECK(sameBits(read.regressors.smooth_sigma_s, w.regressors.smooth_sigma_s));
  CHECK(sameBits(read.regressors.coulomb_v0_rad_s,
                 w.regressors.coulomb_v0_rad_s));
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("joint " << j);
    CHECK(sameBits(read.w[j], w.w[j]));
  }
  CHECK(std::signbit(read.w[0][2]));  // -0.0 kept its sign
  CHECK(learn::formatFeedforwardWeights(read) == text);

  // Through files: write -> read -> write gives identical bytes.
  const fs::path dir = scratchDir();
  const fs::path first = dir / "round_trip_1.toml";
  const fs::path second = dir / "round_trip_2.toml";
  fs::remove(first);
  fs::remove(second);
  learn::writeFeedforwardWeights(first, w);
  learn::writeFeedforwardWeights(second, learn::readFeedforwardWeights(first));
  CHECK(fileText(first) == text);
  CHECK(fileText(second) == text);
  fs::remove(first);
  fs::remove(second);

  // Integers are numbers too (exact up to 2^53).
  const std::string integers =
      replaced(learn::formatFeedforwardWeights(sampleWeights()),
               "joint3 = [2.0, 2.25, 2.5, 2.75, 3.0, 3.25]",
               "joint3 = [2, 9007199254740992, -3, 0, 1, 7]");
  const auto parsed = learn::parseFeedforwardWeights(integers, "mem");
  CHECK(parsed.w[3] == Row{2.0, 9007199254740992.0, -3.0, 0.0, 1.0, 7.0});
}

TEST_CASE("regressor feedforward: every weights read rejection names the key",
          "[learn][regressor_feedforward]") {
  const std::string good = learn::formatFeedforwardWeights(sampleWeights());
  REQUIRE_NOTHROW(learn::parseFeedforwardWeights(good, "case.toml"));
  const std::string joint3 = "joint3 = [2.0, 2.25, 2.5, 2.75, 3.0, 3.25]";
  const auto with = [&good](const std::string& from, const std::string& to) {
    return replaced(good, from, to);
  };

  expectReadError("format = [", {"cannot parse", "line 1"});
  expectReadError(with("format = \"rtctrl-x7-follow-weights\"\n", ""),
                  {"missing key format"});
  expectReadError(with("rtctrl-x7-follow-weights", "rtctrl-x7-follow"),
                  {"format is 'rtctrl-x7-follow'"});
  expectReadError(with("version = 1\n", "version = 2\n"),
                  {"version is 2, expected 1"});
  expectReadError(with("version = 1\n", "version = \"1\"\n"),
                  {"version must be an integer"});
  expectReadError(with("version = 1\n", ""), {"missing key version"});
  expectReadError(with("trial = 2\n", "trial = 2\nextra = 1\n"),
                  {"unknown key 'extra' in the root table"});
  expectReadError(with("trial = 2\n", ""), {"missing key trial"});
  expectReadError(with("trial = 2\n", "trial = 2.0\n"),
                  {"trial must be an integer"});
  expectReadError(with("trial = 2\n", "trial = true\n"),
                  {"trial must be an integer"});
  expectReadError(with("trial = 2\n", "trial = -1\n"),
                  {"trial must be an integer in [0, "});
  expectReadError(with("trial = 2\n", "trial = 3000000000\n"),
                  {"trial must be an integer in [0, ", "3000000000"});
  for (const std::string key :
       {"parent_sha256", "source_log_sha256", "reference_sha256"}) {
    INFO(key);
    const auto at = good.find(key + " = ");
    const auto end = good.find('\n', at);
    const std::string line = good.substr(at, end - at);
    expectReadError(with(line + "\n", ""), {"missing key " + key});
    expectReadError(with(line, key + " = 12"), {key + " must be a string"});
    std::string upper = line;  // the value's hex digits only, not the key
    std::transform(upper.begin() + static_cast<std::ptrdiff_t>(line.find('"')),
                   upper.end(),
                   upper.begin() + static_cast<std::ptrdiff_t>(line.find('"')),
                   [](char c) {
                     return c >= 'a' && c <= 'f'
                                ? static_cast<char>(c - 'a' + 'A')
                                : c;
                   });
    if (upper != line) {
      expectReadError(with(line, upper),
                      {key + " must be empty or 64 lowercase hexadecimal"});
    }
    expectReadError(with(line, key + " = \"" + std::string(63, 'a') + "\""),
                    {key + " must be empty or 64 lowercase hexadecimal"});
    expectReadError(with(line, key + " = \"" + std::string(63, 'a') + "g\""),
                    {key + " must be empty or 64 lowercase hexadecimal"});
    expectReadError(with(line, key + " = \"\""),
                    {key + " is required for trial >= 1"});
  }
  expectReadError(with("\n[regressors]\n", "\n[other]\n"),
                  {"unknown key 'other' in the root table"});
  {
    const auto begin = good.find("\n[regressors]\n");
    const auto end = good.find("\n# delta_j");
    REQUIRE(begin < end);
    std::string without = good;
    without.erase(begin, end - begin);
    expectReadError(without, {"missing key [regressors]"});
    expectReadError(replaced(without, "trial = 2\n", "trial = 2\nregressors = 1\n"),
                    {"[regressors] must be a table"});
  }
  expectReadError(with("smooth_sigma_s", "x = 1\nsmooth_sigma_s"),
                  {"unknown key 'x' in [regressors]"});
  expectReadError(with("\"tau_dyn\", \"ddq\"", "\"ddq\", \"tau_dyn\""),
                  {"regressors.names must be [\"g\", \"tau_dyn\"",
                   "element 1 differs"});
  expectReadError(with("names = [\"g\"", "names = [1"),
                  {"regressors.names must be", "element 0 differs"});
  expectReadError(with(", \"constant\"]", "]"),
                  {"regressors.names must have 6 elements, has 5"});
  expectReadError(with(", \"constant\"]", ", \"constant\", \"extra\"]"),
                  {"regressors.names must have 6 elements, has 7"});
  expectReadError(
      with("names = [\"g\", \"tau_dyn\", \"ddq\", \"dq\", \"coulomb\", "
           "\"constant\"]",
           "names = \"g\""),
      {"regressors.names must be an array"});
  expectReadError(
      with("names = [\"g\", \"tau_dyn\", \"ddq\", \"dq\", \"coulomb\", "
           "\"constant\"]\n",
           ""),
      {"missing key regressors.names"});
  expectReadError(with("smooth_sigma_s = 0.040000000000000001\n", ""),
                  {"missing key regressors.smooth_sigma_s"});
  expectReadError(with("coulomb_v0_rad_s = 0.050000000000000003\n", ""),
                  {"missing key regressors.coulomb_v0_rad_s"});
  expectReadError(with("smooth_sigma_s = 0.040000000000000001",
                       "smooth_sigma_s = \"0.04\""),
                  {"regressors.smooth_sigma_s must be a number"});
  expectReadError(with("smooth_sigma_s = 0.040000000000000001",
                       "smooth_sigma_s = -0.01"),
                  {"regressors.smooth_sigma_s must be finite and >= 0"});
  expectReadError(with("smooth_sigma_s = 0.040000000000000001",
                       "smooth_sigma_s = nan"),
                  {"regressors.smooth_sigma_s must be finite"});
  expectReadError(with("coulomb_v0_rad_s = 0.050000000000000003",
                       "coulomb_v0_rad_s = 0.0"),
                  {"regressors.coulomb_v0_rad_s must be finite and > 0"});
  expectReadError(with("coulomb_v0_rad_s = 0.050000000000000003",
                       "coulomb_v0_rad_s = inf"),
                  {"regressors.coulomb_v0_rad_s must be finite"});
  const auto weights_at = good.find("\n# delta_j");
  REQUIRE(weights_at != std::string::npos);
  expectReadError(good.substr(0, weights_at + 1), {"missing key [weights]"});
  expectReadError(with("joint7 = ", "joint8 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n"
                                    "joint7 = "),
                  {"unknown key 'joint8' in [weights]"});
  expectReadError(with(joint3 + "\n", ""), {"missing key weights.joint3"});
  expectReadError(with(joint3, "joint3 = 2.0"),
                  {"weights.joint3 must be an array"});
  expectReadError(with(joint3, "joint3 = [2.0, 2.25, 2.5, 2.75, 3.0]"),
                  {"weights.joint3 must have 6 elements, has 5"});
  expectReadError(with(joint3, "joint3 = [2.0, 2.25, 2.5, 2.75, 3.0, 3.25, 1.0]"),
                  {"weights.joint3 must have 6 elements, has 7"});
  expectReadError(with(joint3, "joint3 = [2.0, 2.25, \"2.5\", 2.75, 3.0, 3.25]"),
                  {"weights.joint3[2] (ddq) must be a number"});
  expectReadError(with(joint3, "joint3 = [2.0, 2.25, true, 2.75, 3.0, 3.25]"),
                  {"weights.joint3[2] (ddq) must be a number"});
  expectReadError(with(joint3, "joint3 = [2.0, 2.25, inf, 2.75, 3.0, 3.25]"),
                  {"weights.joint3[2] (ddq) must be finite"});
  expectReadError(with(joint3, "joint3 = [2.0, 2.25, 2.5, 2.75, -nan, 3.25]"),
                  {"weights.joint3[4] (coulomb) must be finite"});
  expectReadError(
      with(joint3, "joint3 = [2.0, 2.25, 9007199254740993, 2.75, 3.0, 3.25]"),
      {"weights.joint3[2] (ddq) is an integer a double cannot hold exactly"});
  expectReadError(with("joint7 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                       "joint7 = [0.0, 0.0, 0.0, 0.0, 0.0, 1e-300]"),
                  {"weights.joint7 must be all zero", "gripper"});

  // Trial 0: all weights zero and no hashes.
  FeedforwardWeights zero;
  const std::string fresh = learn::formatFeedforwardWeights(zero);
  REQUIRE_NOTHROW(learn::parseFeedforwardWeights(fresh, "case.toml"));
  expectReadError(replaced(fresh, "joint5 = [0.0, 0.0, 0.0",
                           "joint5 = [0.0, 0.0, 0.25"),
                  {"trial 0 requires all weights zero",
                   "weights.joint5[2] (ddq) is 0.25"});
  expectReadError(replaced(fresh, "parent_sha256 = \"\"",
                           "parent_sha256 = \"" + kHashParent + "\""),
                  {"parent_sha256 must be empty for trial 0"});
}

TEST_CASE("regressor feedforward: invalid weights are never written",
          "[learn][regressor_feedforward]") {
  const fs::path path = scratchDir() / "invalid.toml";
  fs::remove(path);
  FeedforwardWeights w = sampleWeights();
  w.w[kGripper][0] = 0.5;
  expectInvalid([&w] { learn::formatFeedforwardWeights(w); },
                {"follow weights: weights.joint7 must be all zero"});
  expectInvalid([&] { learn::writeFeedforwardWeights(path, w); },
                {"weights.joint7"});
  CHECK_FALSE(fs::exists(path));
  w = sampleWeights();
  w.w[2][4] = std::numeric_limits<double>::infinity();
  expectInvalid([&] { learn::writeFeedforwardWeights(path, w); },
                {"weights.joint2[4] (coulomb) must be finite"});
  CHECK_FALSE(fs::exists(path));
  w = sampleWeights();
  w.trial = -1;
  expectInvalid([&] { learn::validateFeedforwardWeights(w); },
                {"trial must be >= 0"});
  w = sampleWeights();
  w.regressors.coulomb_v0_rad_s = 0.0;
  expectInvalid([&] { learn::validateFeedforwardWeights(w); },
                {"regressors.coulomb_v0_rad_s"});
}

TEST_CASE("regressor feedforward: the weights file is created exclusively",
          "[learn][regressor_feedforward]") {
  const fs::path dir = scratchDir();
  const fs::path path = dir / "exclusive.toml";
  fs::remove(path);
  const FeedforwardWeights w = sampleWeights();
  learn::writeFeedforwardWeights(path, w);
  const std::string written = fileText(path);
  CHECK(written == learn::formatFeedforwardWeights(w));

  FeedforwardWeights other = w;
  other.trial = 3;
  CHECK_THROWS_WITH(learn::writeFeedforwardWeights(path, other),
                    ContainsSubstring("refusing to overwrite"));
  CHECK(fileText(path) == written);  // left untouched

  CHECK_THROWS_WITH(
      learn::writeFeedforwardWeights(dir / "no_such_dir" / "x.toml", w),
      ContainsSubstring("cannot create"));
  CHECK_THROWS_WITH(learn::readFeedforwardWeights(dir / "no_such_file.toml"),
                    ContainsSubstring("cannot open"));
  fs::remove(path);
}

// ============================================================ synthesis

TEST_CASE("regressor feedforward: the offset is the explicit weighted sum",
          "[learn][regressor_feedforward]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto phi =
      learn::buildRegressors(chain, map, movingReference(60, kDt), kDt, {});
  FeedforwardWeights w = sampleWeights();
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      w.w[j][l] = 1e-3 * std::cos(0.9 * j - 1.7 * l);
    }
  }
  const auto delta = learn::synthesizeOffset(phi, w);
  REQUIRE(delta.size() == phi.size());
  for (std::size_t i = 0; i < phi.size(); ++i) {
    for (int j = 0; j < kArm; ++j) {
      const auto& p = phi[i][j];
      const auto& v = w.w[j];
      const double sum = v[0] * p[0] + v[1] * p[1] + v[2] * p[2] +
                         v[3] * p[3] + v[4] * p[4] + v[5] * p[5];
      CHECK(delta[i][j] == sum);  // same products, same order
    }
    CHECK(delta[i][kGripper] == 0.0);
  }

  // The gripper offset is zero whatever its regressor row holds.
  auto odd = phi;
  odd[3][kGripper].fill(5.0);
  CHECK(learn::synthesizeOffset(odd, w)[3][kGripper] == 0.0);

  expectInvalid([&w] { learn::synthesizeOffset({}, w); }, {"empty"});
  auto bad = w;
  bad.w[kGripper][1] = 1.0;
  expectInvalid([&] { learn::synthesizeOffset(phi, bad); },
                {"weights.joint7 must be all zero"});
  bad = w;
  bad.w[4][0] = std::numeric_limits<double>::quiet_NaN();
  expectInvalid([&] { learn::synthesizeOffset(phi, bad); },
                {"weights.joint4[0] (g) must be finite"});
  auto bad_phi = phi;
  bad_phi[7][2][3] = std::numeric_limits<double>::infinity();
  expectInvalid([&] { learn::synthesizeOffset(bad_phi, w); },
                {"regressor dq is not finite at sample 7, joint 2"});
  bad = w;
  bad.w[1][kConstant] = 1.7e308;
  bad.w[1][kDq] = 1.7e308;
  bad_phi = phi;
  bad_phi[0][1][kDq] = 1e10;
  expectInvalid([&] { learn::synthesizeOffset(bad_phi, bad); },
                {"offset is not finite at sample 0, joint 1"});
}

// =============================================================== update

TEST_CASE(
    "regressor feedforward: the error signal matches a straightforward "
    "implementation",
    "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 150;
  constexpr std::size_t m = n + 9;
  std::vector<JointArray> e(m);
  Gaussian noise(7);
  for (std::size_t k = 0; k < m; ++k) {
    const double t = static_cast<double>(k) * kDt;
    for (int j = 0; j < kCanonicalDof; ++j) {
      e[k][j] = 0.01 * std::sin(2.0 * kPi * (0.5 + 0.1 * j) * t + j) +
                2e-4 * noise();
    }
  }
  FelOptions options;
  options.advance_s = 0.047;  // 4.7 samples: interpolated
  options.error_lead_s = 0.02;
  options.error_smooth_sigma_s = 0.03;
  const auto s = learn::felErrorSignal(e, n, kDt, options);
  REQUIRE(s.size() == n);
  const int half = halfWidth(options.error_smooth_sigma_s, kDt);
  for (int j = 0; j < kCanonicalDof; ++j) {
    INFO("joint " << j);
    const auto smooth = plainSmooth(column(e, j), options.error_smooth_sigma_s,
                                    kDt);
    const auto rate = plainGradient(smooth, kDt);
    // The smoothing difference d (smoothTolerance) reaches the rate as d /
    // dt; the interpolation position is computed from times here, which
    // moves it by a few eps samples on a signal whose sample-to-sample step
    // is below 0.01 rad, and the rate's by 1 rad/s per sample at most.
    const double d = smoothTolerance(half, maxAbs(column(e, j)));
    const double tol = d + options.error_lead_s * d / kDt + 1e-14;
    for (std::size_t i = 0; i < n; ++i) {
      const double position =
          (static_cast<double>(i) * kDt + options.advance_s) / kDt;
      const auto k = static_cast<std::size_t>(std::floor(position));
      const double f = position - static_cast<double>(k);
      const double value = smooth[k] + f * (smooth[k + 1] - smooth[k]);
      const double slope = rate[k] + f * (rate[k + 1] - rate[k]);
      INFO("sample " << i);
      CHECK(std::fabs(s[i][j] - (value + options.error_lead_s * slope)) <=
            tol);
    }
  }

  // An advance a hair above a whole number of samples (0.07 / 0.01 =
  // 7.000000000000001) is that whole number: the 7 tail samples
  // ceil(advance / dt - 1e-9) asks for suffice and the shift is exact.
  FelOptions whole;
  whole.advance_s = 0.07;
  whole.error_smooth_sigma_s = 0.0;
  whole.error_lead_s = 0.0;
  REQUIRE(0.07 / kDt > 7.0);
  std::vector<JointArray> e7(e.begin(), e.begin() + n + 7);
  const auto s7 = learn::felErrorSignal(e7, n, kDt, whole);
  for (std::size_t i = 0; i < n; ++i) {
    for (int j = 0; j < kCanonicalDof; ++j) CHECK(s7[i][j] == e7[i + 7][j]);
  }
}

TEST_CASE("regressor feedforward: the update needs the advance's tail",
          "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 100;
  const auto phi = equicorrelated(n, {0, 0, 0, 0, 0, 0, 0});
  const FeedforwardWeights w0;
  const Weights planted = plantedWeights(syntheticScales());
  FelOptions options = exactPlantOptions(FelOptions::Step::NormalisedGradient,
                                         0.2);
  // advance 0.05 s at 10 ms: 5 samples after the grid are needed.
  const auto e4 = plantError(phi, planted, w0.w, 4, 4);
  expectInvalid([&] { learn::felUpdate(w0, phi, e4, kDt, options); },
                {"felUpdate: error needs at least 5 samples after the 100 "
                 "tracking samples",
                 "got 4"});
  const auto e5 = plantError(phi, planted, w0.w, 5, 5);
  CHECK_NOTHROW(learn::felUpdate(w0, phi, e5, kDt, options));
  // 4.7 samples interpolate between the 4th and 5th: 5 are needed.
  options.advance_s = 0.047;
  expectInvalid([&] { learn::felUpdate(w0, phi, e4, kDt, options); },
                {"at least 5 samples", "got 4"});
  CHECK_NOTHROW(learn::felUpdate(w0, phi, e5, kDt, options));
  // No advance needs no tail.
  options.advance_s = 0.0;
  const auto e0 = plantError(phi, planted, w0.w, 0, 0);
  CHECK_NOTHROW(learn::felUpdate(w0, phi, e0, kDt, options));
}

TEST_CASE(
    "regressor feedforward: least squares at gain 1 recovers planted weights "
    "in one update",
    "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 400;
  constexpr double rho = 0.5;  // lambda_max = 3.5, lambda_min = 0.5
  const auto phi = equicorrelated(n, {rho, rho, rho, rho, rho, rho, rho});
  const auto scales = syntheticScales();
  const Weights planted = plantedWeights(scales);
  const FeedforwardWeights w0;
  const auto e = plantError(phi, planted, w0.w, kShift, kTail);
  FelOptions options = exactPlantOptions(FelOptions::Step::LeastSquares, 1.0);
  REQUIRE(options.ridge == 1e-6);
  FelReport report;
  const FeedforwardWeights w1 =
      learn::felUpdate(w0, phi, e, kDt, options, &report);
  const auto signal = learn::felErrorSignal(e, n, kDt, options);
  CHECK(w1.trial == 1);
  CHECK(w1.parent_sha256.empty());
  CHECK(w1.source_log_sha256.empty());
  CHECK(w1.reference_sha256.empty());
  CHECK(w1.w[kGripper] == Row{});
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    const auto& r = report.joints[j];
    CHECK(r.enabled);
    CHECK(r.kept_columns == 6);
    // The Gram matrix's spectrum is known (equicorrelated); 1e-10 is the
    // library's stated eigenvalue bound, far above the grid sums' rounding.
    CHECK(std::fabs(r.lambda_max - (1.0 + 5.0 * rho)) < 1e-10);
    CHECK(std::fabs(r.lambda_min - (1.0 - rho)) < 1e-10);
    for (int l = 0; l < kRegressorCount; ++l) {
      CHECK(std::fabs(r.column_rms[l] - scales[j][l]) < 1e-12 * scales[j][l]);
    }
    // The ridge solution in normalised units is (C + r I)^-1 C u, so it
    // misses u by r (C + r I)^-1 u: at most r / (lambda_min + r) |u|
    // = 2e-6 |u|, plus the solve's rounding (cond(C) = 7 times a few eps).
    const double u = normalisedDistance(planted[j], w0.w[j], scales[j]);
    const double ridge_bound = options.ridge / (0.5 + options.ridge) * u;
    const double miss = normalisedDistance(w1.w[j], planted[j], scales[j]);
    CHECK(miss <= ridge_bound + 1e-12 * u);
    CHECK(miss > 0.0);  // the ridge is applied
    CHECK(r.s_rms == rms(column(signal, j)));  // same sum, same order
    // The predicted remaining error is the miss seen through C:
    // at most sqrt(lambda_max) times it.
    CHECK(r.residual_rms <= std::sqrt(3.5) * (ridge_bound + 1e-12 * u));
  }

  // Without the advance the error does not line up with the regressors:
  // the update then misses the planted weights by far more.
  FelOptions unaligned = options;
  unaligned.advance_s = 0.0;
  FelReport unaligned_report;
  const auto w_unaligned =
      learn::felUpdate(w0, phi, e, kDt, unaligned, &unaligned_report);
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    const double u = normalisedDistance(planted[j], w0.w[j], scales[j]);
    CHECK(normalisedDistance(w_unaligned.w[j], planted[j], scales[j]) >
          1e-3 * u);
    CHECK(unaligned_report.joints[j].residual_rms >
          1e3 * report.joints[j].residual_rms);
  }
}

TEST_CASE(
    "regressor feedforward: the least-squares step is the analysis tool's "
    "ridge fit",
    "[learn][regressor_feedforward]") {
  // A large ridge makes the shrinkage visible. In normalised units the
  // ridge_fit solution is (C + r I)^-1 C u; with C = (1 - rho) I + rho 1 1^T
  // that is lambda / (lambda + r) times u along each eigenspace: the mean
  // direction (lambda 1 + 5 rho) and its complement (lambda 1 - rho).
  constexpr std::size_t n = 400;
  constexpr double rho = 0.5;
  constexpr double ridge = 0.1;
  const auto phi = equicorrelated(n, {rho, rho, rho, rho, rho, rho, rho});
  const auto scales = syntheticScales();
  const Weights planted = plantedWeights(scales);
  const FeedforwardWeights w0;
  const auto e = plantError(phi, planted, w0.w, kShift, kTail);
  FelOptions options = exactPlantOptions(FelOptions::Step::LeastSquares, 0.8);
  options.ridge = ridge;
  const auto w1 = learn::felUpdate(w0, phi, e, kDt, options);
  const double hi = 1.0 + 5.0 * rho;
  const double lo = 1.0 - rho;
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    Row u{};
    double mean = 0.0;
    for (int l = 0; l < kRegressorCount; ++l) {
      u[l] = scales[j][l] * planted[j][l];
      mean += u[l] / kRegressorCount;
    }
    double norm = 0.0;
    for (const double v : u) norm += v * v;
    norm = std::sqrt(norm);
    for (int l = 0; l < kRegressorCount; ++l) {
      const double expected =
          options.gain *
          (hi / (hi + ridge) * mean + lo / (lo + ridge) * (u[l] - mean)) /
          scales[j][l];
      // The grid reproduces C to about N eps; the solve adds cond(C) = 7
      // times a few eps: 1e-11 |u| is a wide bound.
      CHECK(std::fabs(w1.w[j][l] - expected) <= 1e-11 * norm / scales[j][l]);
    }
  }
}

TEST_CASE(
    "regressor feedforward: the normalised gradient converges to planted "
    "weights, the error signal shrinking monotonically",
    "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 400;
  constexpr double rho = 0.5;
  constexpr double gain = 0.2;
  const double hi = 1.0 + 5.0 * rho;  // 3.5: gain * hi = 0.7 < 2
  const double lo = 1.0 - rho;
  const auto phi = equicorrelated(n, {rho, rho, rho, rho, rho, rho, rho});
  const auto scales = syntheticScales();
  const Weights planted = plantedWeights(scales);
  const FelOptions options =
      exactPlantOptions(FelOptions::Step::NormalisedGradient, gain);

  // In normalised units the weight error obeys u <- (I - gain C) u, so the
  // RMS of s = Phi (w* - w) after k updates is exactly
  // sqrt(hi (1 - gain hi)^2k |P u0|^2 + lo (1 - gain lo)^2k |(I - P) u0|^2),
  // P the projection on the mean direction.
  std::array<double, kArm> along{}, across{};
  for (int j = 0; j < kArm; ++j) {
    double mean = 0.0;
    double norm2 = 0.0;
    for (int l = 0; l < kRegressorCount; ++l) {
      const double u = scales[j][l] * planted[j][l];
      mean += u / kRegressorCount;
      norm2 += u * u;
    }
    along[j] = kRegressorCount * mean * mean;
    across[j] = norm2 - along[j];
  }

  FeedforwardWeights w;
  std::array<double, kArm> previous{};
  previous.fill(std::numeric_limits<double>::infinity());
  std::array<double, kArm> s0{};
  constexpr int kTrials = 300;
  for (int k = 0; k < kTrials; ++k) {
    const auto e = plantError(phi, planted, w.w, kShift, kTail);
    FelReport report;
    w = learn::felUpdate(w, phi, e, kDt, options, &report);
    REQUIRE(w.trial == k + 1);
    for (int j = 0; j < kArm; ++j) {
      INFO("trial " << k << ", joint " << j);
      const double s = report.joints[j].s_rms;
      const double predicted =
          std::sqrt(hi * std::pow(1.0 - gain * hi, 2 * k) * along[j] +
                    lo * std::pow(1.0 - gain * lo, 2 * k) * across[j]);
      // It follows the prediction to the grid's reproduction of C (about N
      // eps per trial, compounding to well below 1e-8 relative), above an
      // absolute floor: the stored weights round by eps |w*| per update,
      // which the iteration contracts to at most eps |u0| / (gain lo) =
      // 10 eps |u0|, and the plant's sums round likewise; 1e-13 s_0 bounds
      // that floor. The slowest mode contracts by 0.9 per trial, so up to
      // trial 200 (0.9^200 = 7e-10) the signal stays far above the floor
      // and must shrink at every trial.
      if (k == 0) s0[j] = s;
      if (k <= 200) {
        CHECK(s < previous[j]);
        CHECK(std::fabs(s - predicted) <= 1e-8 * predicted + 1e-13 * s0[j]);
      }
      previous[j] = s;
    }
  }
  // After 300 trials the slowest mode is down to 0.9^300 = 2e-14 of the
  // initial error; rounding keeps an error of order eps |w*| / (gain lo)
  // = 1e-14 |u0|. 1e-12 |u0| bounds both.
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    const double u0 = normalisedDistance(planted[j], Row{}, scales[j]);
    CHECK(normalisedDistance(w.w[j], planted[j], scales[j]) <= 1e-12 * u0);
  }
}

TEST_CASE(
    "regressor feedforward: with noise the normalised gradient settles within "
    "its statistical spread",
    "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 400;
  constexpr double rho = 0.5;
  constexpr double gain = 0.2;
  constexpr double sigma_noise = 1e-3;  // rad, white, every sample and trial
  const double hi = 1.0 + 5.0 * rho;
  const auto phi = equicorrelated(n, {rho, rho, rho, rho, rho, rho, rho});
  const auto scales = syntheticScales();
  const Weights planted = plantedWeights(scales);
  const FelOptions options =
      exactPlantOptions(FelOptions::Step::NormalisedGradient, gain);
  Gaussian noise(42);
  FeedforwardWeights w;
  constexpr int kTrials = 400;
  for (int k = 0; k < kTrials; ++k) {
    auto e = plantError(phi, planted, w.w, kShift, kTail);
    for (auto& sample : e) {
      for (double& v : sample) v += sigma_noise * noise();
    }
    w = learn::felUpdate(w, phi, e, kDt, options);
  }
  // Each eigen-direction of C follows x <- (1 - gain lambda) x + gain xi,
  // xi = q^T a^T n / N of variance sigma^2 lambda / N, so its stationary
  // variance is gain sigma^2 / (N (2 - gain lambda)), largest at
  // lambda_max; a normalised weight is a unit combination of directions, so
  // its standard deviation is at most that. The transient is gone after 400
  // trials (0.9^400 = 5e-19). 5 standard deviations: a Gaussian exceeds it
  // with probability 6e-7 per weight (2e-5 for all 42); the seed is fixed.
  const double spread = sigma_noise * std::sqrt(gain / (n * (2.0 - gain * hi)));
  double largest = 0.0;
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      INFO("joint " << j << ", column " << l);
      const double deviation =
          std::fabs(scales[j][l] * (w.w[j][l] - planted[j][l]));
      CHECK(deviation <= 5.0 * spread);
      largest = std::max(largest, deviation / spread);
    }
  }
  INFO("largest deviation in standard deviations: " << largest);
  CHECK(largest > 0.0);
}

TEST_CASE(
    "regressor feedforward: the stability guard names the admissible gain",
    "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 200;
  // Joint 3 has equicorrelated columns (lambda_max 1 + 5 * 0.5 = 3.5), the
  // others orthogonal ones (lambda 1).
  const auto phi = equicorrelated(n, {0, 0, 0, 0.5, 0, 0, 0});
  const Weights planted = plantedWeights(syntheticScales());
  const FeedforwardWeights w0;
  const auto e = plantError(phi, planted, w0.w, kShift, kTail);
  FelOptions options =
      exactPlantOptions(FelOptions::Step::NormalisedGradient, 0.6);
  // 0.6 * 3.5 = 2.1 >= 2; the bound is 2 / 3.5 = 0.571429.
  expectInvalid([&] { learn::felUpdate(w0, phi, e, kDt, options); },
                {"felUpdate: joint 3:", "gain 0.6", "lambda_max 3.5",
                 "largest admissible gain is below 2 / lambda_max = 0.571429"});
  options.gain = 0.57;
  FelReport report;
  CHECK_NOTHROW(learn::felUpdate(w0, phi, e, kDt, options, &report));
  CHECK(std::fabs(report.joints[3].lambda_max - 3.5) < 1e-10);
  CHECK(std::fabs(report.joints[0].lambda_max - 1.0) < 1e-10);
  CHECK(std::fabs(report.joints[0].lambda_min - 1.0) < 1e-10);
  // With orthogonal columns any gain in (0, 1] is admissible.
  const auto orthogonal = equicorrelated(n, {0, 0, 0, 0, 0, 0, 0});
  options.gain = 1.0;
  CHECK_NOTHROW(learn::felUpdate(
      w0, orthogonal, plantError(orthogonal, planted, w0.w, kShift, kTail),
      kDt, options));
}

TEST_CASE(
    "regressor feedforward: disabled joints and the inputs are left untouched",
    "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 120;
  const auto phi = equicorrelated(n, {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2});
  const Weights planted = plantedWeights(syntheticScales());
  FeedforwardWeights w_k = sampleWeights();
  w_k.trial = 4;
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      w_k.w[j][l] = 0.3 * planted[j][l];
    }
  }
  w_k.w[2][2] = -0.0;  // a sign that w + 0.0 would lose
  const auto e = plantError(phi, planted, w_k.w, kShift, kTail);
  FelOptions options = exactPlantOptions(FelOptions::Step::LeastSquares, 0.5);
  options.joint_enabled[2] = false;
  options.joint_enabled[5] = false;

  const auto phi_copy = phi;
  const auto e_copy = e;
  const FeedforwardWeights w_copy = w_k;
  FelReport report;
  const auto next = learn::felUpdate(w_k, phi, e, kDt, options, &report);
  CHECK(next.trial == 5);
  CHECK(next.parent_sha256.empty());
  CHECK(sameBits(next.w[2], w_k.w[2]));
  CHECK(sameBits(next.w[5], w_k.w[5]));
  CHECK(sameBits(next.w[kGripper], w_k.w[kGripper]));
  CHECK_FALSE(report.joints[2].enabled);
  CHECK_FALSE(report.joints[5].enabled);
  CHECK_FALSE(report.joints[kGripper].enabled);
  for (const int j : {0, 1, 3, 4, 6}) {
    INFO("joint " << j);
    CHECK(report.joints[j].enabled);
    for (int l = 0; l < kRegressorCount; ++l) {
      CHECK(next.w[j][l] != w_k.w[j][l]);
    }
  }
  // Inputs unmodified.
  CHECK(phi == phi_copy);
  CHECK(e == e_copy);
  CHECK(w_k.trial == w_copy.trial);
  CHECK(w_k.parent_sha256 == w_copy.parent_sha256);
  for (int j = 0; j < kCanonicalDof; ++j) CHECK(sameBits(w_k.w[j], w_copy.w[j]));
}

TEST_CASE(
    "regressor feedforward: the model's joint-0 gravity column is skipped and "
    "reported",
    "[learn][regressor_feedforward]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  constexpr std::size_t n = 300;
  const RegressorOptions regressor_options;  // the defaults
  const auto phi = learn::buildRegressors(chain, map, movingReference(n, kDt),
                                          kDt, regressor_options);
  // Joint 0 turns about the vertical axis: the model's gravity torque on it
  // is zero up to rounding, whatever the posture.
  std::array<Row, kArm> column_rms{};
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      double sum = 0.0;
      for (const auto& row : phi) sum += row[j][l] * row[j][l];
      column_rms[j][l] = std::sqrt(sum / n);
    }
  }
  const double g0 = column_rms[0][kG];
  INFO("joint 0 gravity column RMS over a moving reference: " << g0 << " Nm");
  CHECK(g0 < 1e-6);  // FelOptions::negligible_rms
  for (int j = 1; j < kArm; ++j) {
    INFO("joint " << j << " gravity column RMS " << column_rms[j][kG]);
    CHECK(column_rms[j][kG] >= 1e-6);  // kept on every other joint
  }
  CHECK(column_rms[1][kG] > 0.1);  // the shoulder carries the arm

  Weights planted = plantedWeights([&column_rms] {
    auto safe = column_rms;
    safe[0][kG] = 1.0;  // the zero column gets a finite planted weight
    return safe;
  }());
  FeedforwardWeights w_k = sampleWeights();
  for (int j = 0; j < kArm; ++j) w_k.w[j].fill(0.0);
  w_k.w[0][kG] = 0.123;  // must survive bit-exactly
  const auto e = plantError(phi, planted, w_k.w, kShift, kTail);
  const FelOptions options =
      exactPlantOptions(FelOptions::Step::NormalisedGradient, 0.2);
  FelReport report;
  const auto next = learn::felUpdate(w_k, phi, e, kDt, options, &report);
  const auto& r0 = report.joints[0];
  CHECK(r0.skipped[kG]);
  CHECK(r0.kept_columns == 5);
  for (int l = 1; l < kRegressorCount; ++l) CHECK_FALSE(r0.skipped[l]);
  CHECK(sameBits(next.w[0][kG], 0.123));
  CHECK(r0.column_rms[kG] == g0);
  for (int j = 1; j < kArm; ++j) {
    INFO("joint " << j);
    CHECK(report.joints[j].kept_columns == 6);
    CHECK(report.joints[j].lambda_max < 2.0 / options.gain);
    INFO("lambda_min " << report.joints[j].lambda_min << ", lambda_max "
                       << report.joints[j].lambda_max);
    CHECK(report.joints[j].lambda_min > 0.0);
  }
}

TEST_CASE(
    "regressor feedforward: on the model's regressors the normalised gradient "
    "shrinks the error signal monotonically and least squares recovers the "
    "weights",
    "[learn][regressor_feedforward]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  constexpr std::size_t n = 300;
  const auto phi =
      learn::buildRegressors(chain, map, movingReference(n, kDt), kDt, {});
  std::array<Row, kArm> column_rms{};
  for (int j = 0; j < kArm; ++j) {
    for (int l = 0; l < kRegressorCount; ++l) {
      double sum = 0.0;
      for (const auto& row : phi) sum += row[j][l] * row[j][l];
      column_rms[j][l] = std::sqrt(sum / n);
    }
  }
  auto safe = column_rms;
  safe[0][kG] = 1.0;
  Weights planted = plantedWeights(safe);
  planted[0][kG] = 0.0;  // the skipped column is never learned

  // Normalised gradient: u <- (I - gain C) u shrinks u^T C u = RMS(s)^2 at
  // every trial while gain * lambda_max < 2, whatever C's conditioning.
  const FelOptions gradient =
      exactPlantOptions(FelOptions::Step::NormalisedGradient, 0.2);
  FeedforwardWeights w;
  std::array<double, kArm> previous{};
  previous.fill(std::numeric_limits<double>::infinity());
  for (int k = 0; k < 30; ++k) {
    FelReport report;
    w = learn::felUpdate(w, phi, plantError(phi, planted, w.w, kShift, kTail),
                         kDt, gradient, &report);
    for (int j = 0; j < kArm; ++j) {
      INFO("trial " << k << ", joint " << j);
      CHECK(report.joints[j].s_rms < previous[j]);
      previous[j] = report.joints[j].s_rms;
    }
  }

  // Least squares without a ridge recovers the planted weights to the
  // normal equations' accuracy: relative error at most about cond(C) N eps
  // (Gram sums over N samples, then a Cholesky solve); 10 times that.
  FelOptions exact = exactPlantOptions(FelOptions::Step::LeastSquares, 1.0);
  exact.ridge = 0.0;
  const FeedforwardWeights w0;
  FelReport report;
  const auto w1 = learn::felUpdate(
      w0, phi, plantError(phi, planted, w0.w, kShift, kTail), kDt, exact,
      &report);
  for (int j = 0; j < kArm; ++j) {
    const auto& r = report.joints[j];
    const double cond = r.lambda_max / r.lambda_min;
    INFO("joint " << j << ": lambda_min " << r.lambda_min << ", lambda_max "
                  << r.lambda_max << ", cond " << cond);
    Row rms_kept = column_rms[j];
    if (j == 0) rms_kept[kG] = 0.0;  // skipped: compare the kept columns
    const double u = normalisedDistance(planted[j], w0.w[j], rms_kept);
    CHECK(normalisedDistance(w1.w[j], planted[j], rms_kept) <=
          10.0 * cond * n * kEps * u);
  }
}

TEST_CASE("regressor feedforward: the error lead uses the error's rate",
          "[learn][regressor_feedforward]") {
  // e(t) = alpha + beta t + gamma t^2. At the advanced time tau = t_i +
  // advance, s = e + T_s edot = (alpha + T_s beta) + (beta + 2 T_s gamma) tau
  // + gamma tau^2: with the columns 1, tau, tau^2 a least-squares step at
  // gain 1 and no ridge returns exactly those coefficients. The central
  // difference is exact for a quadratic, and every sample used is interior
  // (two samples of tail beyond the advance).
  constexpr std::size_t n = 200;
  constexpr double alpha = 0.01;
  constexpr double beta = -0.02;
  constexpr double gamma = 0.015;
  constexpr double lead = 0.03;
  std::vector<RegressorRow> phi(n);
  for (std::size_t i = 0; i < n; ++i) {
    for (auto& joint : phi[i]) joint.fill(0.0);
    const double tau = static_cast<double>(i + kShift) * kDt;
    for (int j = 0; j < kArm; ++j) {
      phi[i][j][0] = 1.0;
      phi[i][j][1] = tau;
      phi[i][j][2] = tau * tau;
    }
  }
  std::vector<JointArray> e(n + kShift + 2);
  for (std::size_t m = 0; m < e.size(); ++m) {
    const double t = static_cast<double>(m) * kDt;
    for (int j = 0; j < kCanonicalDof; ++j) {
      e[m][j] = alpha + beta * t + gamma * t * t;
    }
  }
  FelOptions options = exactPlantOptions(FelOptions::Step::LeastSquares, 1.0);
  options.ridge = 0.0;
  options.error_lead_s = lead;
  const FeedforwardWeights w0;
  FelReport report;
  const auto w1 = learn::felUpdate(w0, phi, e, kDt, options, &report);
  options.error_lead_s = 0.0;
  const auto w_plain = learn::felUpdate(w0, phi, e, kDt, options);
  const Row with_lead = {alpha + lead * beta, beta + 2.0 * lead * gamma, gamma,
                         0.0, 0.0, 0.0};
  const Row without = {alpha, beta, gamma, 0.0, 0.0, 0.0};
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    const auto& r = report.joints[j];
    CHECK(r.kept_columns == 3);
    CHECK(r.skipped[3]);
    CHECK(r.skipped[4]);
    CHECK(r.skipped[5]);
    // Normal equations: relative error about cond(C) N eps (10 times that).
    const double tol = 10.0 * (r.lambda_max / r.lambda_min) * n * kEps;
    const Row scale = {r.column_rms[0], r.column_rms[1], r.column_rms[2],
                       0.0, 0.0, 0.0};
    const double u = normalisedDistance(with_lead, Row{}, scale);
    CHECK(normalisedDistance(w1.w[j], with_lead, scale) <= tol * u);
    CHECK(normalisedDistance(w_plain.w[j], without, scale) <=
          tol * normalisedDistance(without, Row{}, scale));
    CHECK(r.residual_rms <= tol * r.s_rms);
  }
}

TEST_CASE("regressor feedforward: invalid update inputs are rejected",
          "[learn][regressor_feedforward]") {
  constexpr std::size_t n = 50;
  const auto phi = equicorrelated(n, {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
  const Weights planted = plantedWeights(syntheticScales());
  const FeedforwardWeights w0;
  const auto e = plantError(phi, planted, w0.w, kShift, kTail);
  const FelOptions good;
  REQUIRE_NOTHROW(learn::felUpdate(w0, phi, e, kDt, good));

  const auto update = [&](FeedforwardWeights w, std::vector<RegressorRow> p,
                          std::vector<JointArray> err, double dt,
                          FelOptions o) {
    return [w, p, err, dt, o] { learn::felUpdate(w, p, err, dt, o); };
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const double gain : {0.0, -0.1, 1.5, nan}) {
    FelOptions o = good;
    o.gain = gain;
    expectInvalid(update(w0, phi, e, kDt, o),
                  {"felUpdate: options.gain must be in (0, 1]"});
  }
  FelOptions o = good;
  o.advance_s = -0.01;
  expectInvalid(update(w0, phi, e, kDt, o), {"felUpdate: options.advance_s"});
  o = good;
  o.error_lead_s = -0.01;
  expectInvalid(update(w0, phi, e, kDt, o), {"felUpdate: options.error_lead_s"});
  o = good;
  o.error_smooth_sigma_s = nan;
  expectInvalid(update(w0, phi, e, kDt, o),
                {"felUpdate: options.error_smooth_sigma_s"});
  o = good;
  o.ridge = -1e-6;
  expectInvalid(update(w0, phi, e, kDt, o), {"felUpdate: options.ridge"});
  for (const double negligible : {0.0, -1.0, nan}) {
    o = good;
    o.negligible_rms = negligible;
    expectInvalid(update(w0, phi, e, kDt, o),
                  {"felUpdate: options.negligible_rms must be finite and > 0"});
  }
  o = good;
  o.joint_enabled[kGripper] = true;
  expectInvalid(update(w0, phi, e, kDt, o),
                {"felUpdate: options.joint_enabled[7]", "gripper"});
  o = good;
  o.step = static_cast<FelOptions::Step>(7);
  expectInvalid(update(w0, phi, e, kDt, o), {"felUpdate: options.step"});
  expectInvalid(update(w0, phi, e, 0.0, good), {"felUpdate: dt must be"});
  expectInvalid(update(w0, {phi[0], phi[1]}, e, kDt, good),
                {"felUpdate: phi needs at least 3 samples, got 2"});
  expectInvalid(update(w0, phi, {e.begin(), e.begin() + n - 1}, kDt, good),
                {"felUpdate: error has 49 samples, fewer than the 50"});
  auto bad_phi = phi;
  bad_phi[3][2][kDdq] = nan;
  expectInvalid(update(w0, bad_phi, e, kDt, good),
                {"felUpdate: regressor ddq is not finite at sample 3, joint 2"});
  auto bad_e = e;
  bad_e[n + 2][6] = std::numeric_limits<double>::infinity();
  expectInvalid(update(w0, phi, bad_e, kDt, good),
                {"felUpdate: error is not finite at sample 52, joint 6"});
  FeedforwardWeights bad_w = w0;
  bad_w.w[1][kG] = nan;
  expectInvalid(update(bad_w, phi, e, kDt, good),
                {"felUpdate: w_k.weights.joint1[0] (g) must be finite"});
  bad_w = w0;
  bad_w.w[kGripper][kConstant] = 0.1;
  expectInvalid(update(bad_w, phi, e, kDt, good),
                {"felUpdate: w_k.weights.joint7 must be all zero"});
  bad_w = w0;
  bad_w.trial = -1;
  expectInvalid(update(bad_w, phi, e, kDt, good), {"felUpdate: w_k.trial"});
  bad_w.trial = INT_MAX;
  expectInvalid(update(bad_w, phi, e, kDt, good), {"felUpdate: w_k.trial"});
  bad_w = w0;
  bad_w.regressors.coulomb_v0_rad_s = -1.0;
  expectInvalid(update(bad_w, phi, e, kDt, good),
                {"felUpdate: w_k.regressors.coulomb_v0_rad_s"});

  // Without a ridge, dependent columns are refused, not solved.
  auto dependent = phi;
  for (auto& row : dependent) row[4][kConstant] = 2.0 * row[4][kCoulomb];
  o = good;
  o.step = FelOptions::Step::LeastSquares;
  o.ridge = 0.0;
  expectInvalid(update(w0, dependent, e, kDt, o),
                {"felUpdate: joint 4: the kept regressor columns are "
                 "numerically dependent"});
  o.ridge = 1e-6;  // the ridge makes it well posed
  CHECK_NOTHROW(update(w0, dependent, e, kDt, o)());
}

// ======================================== the update with default smoothing
//
// The model's regressors on a rest-to-rest reference, the planted-weights
// plant held at both ends (heldPlantError), and the default options: the
// error smoothed with sigma 0.04 s and advanced by 0.05 s. The smoothed
// error vanishes only when the remaining error does (C^-1 M nonsingular), so
// the planted weights stay the fixed point; SignalPath derives each update.

TEST_CASE(
    "regressor feedforward: with the default error smoothing least squares at "
    "gain 1 contracts the remaining error as derived",
    "[learn][regressor_feedforward]") {
  checkLeastSquaresContraction(0.0);
}

TEST_CASE(
    "regressor feedforward: with an error lead and the default smoothing least "
    "squares at gain 1 contracts the remaining error as derived",
    "[learn][regressor_feedforward]") {
  checkLeastSquaresContraction(0.02);
}

TEST_CASE(
    "regressor feedforward: with the default error smoothing the normalised "
    "gradient shrinks the remaining error at every update",
    "[learn][regressor_feedforward]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto phi = learn::buildRegressors(
      chain, map, restToRestReference(kModelSamples, kDt), kDt, {});
  const FelOptions options;  // all defaults: gain 0.2, no lead
  REQUIRE(options.step == FelOptions::Step::NormalisedGradient);
  const Weights planted = plantedOnKept(columnRms(phi));
  std::array<SignalPath, kArm> paths;
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    paths[j] = signalPath(phi, j, options, kShift, kTail);
    // RMS(Phi (w* - w))^2 = u^T C u drops at every update, from every start,
    // exactly when Q = C - T^T C T is positive definite, T = I - gain M.
    Dense t = paths[j].m;
    for (std::size_t p = 0; p < t.size(); ++p) {
      for (std::size_t q = 0; q < t.size(); ++q) {
        t[p][q] = (p == q ? 1.0 : 0.0) - options.gain * t[p][q];
      }
    }
    Dense q = paths[j].c;
    const Dense tct = product(transposed(t), product(paths[j].c, t));
    for (std::size_t p = 0; p < q.size(); ++p) {
      for (std::size_t r = 0; r < q.size(); ++r) q[p][r] -= tct[p][r];
    }
    CHECK(positiveDefinite(q));
  }
  constexpr int kUpdates = 100;
  std::array<std::vector<double>, kArm> weight_error;
  const auto remaining = runNormalisedGradient(phi, planted, options, paths,
                                               kUpdates, &weight_error);
  for (int j = 0; j < kArm; ++j) {
    for (int k = 0; k < kUpdates; ++k) {
      INFO("update " << k << ", joint " << j);
      CHECK(remaining[j][k + 1] < remaining[j][k]);
    }
    INFO("joint " << j << ": remaining after " << kUpdates
                  << " updates: " << remaining[j].back()
                  << ", normalised weight error "
                  << weight_error[j].back() / weight_error[j].front());
    CHECK(remaining[j].back() <= 0.1);  // reduced by 10 within the run
  }
}

TEST_CASE(
    "regressor feedforward: with an error lead and the default smoothing the "
    "normalised gradient contracts the weight error at every update",
    "[learn][regressor_feedforward]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto phi = learn::buildRegressors(
      chain, map, restToRestReference(kModelSamples, kDt), kDt, {});
  FelOptions options;  // defaults but for the lead
  options.error_lead_s = 0.02;
  const Weights planted = plantedOnKept(columnRms(phi));
  std::array<SignalPath, kArm> paths;
  for (int j = 0; j < kArm; ++j) {
    INFO("joint " << j);
    paths[j] = signalPath(phi, j, options, kShift, kTail);
    // The lead adds lead * mean(a_p d/dt S_q) to M, nearly skew (the rate
    // couples columns: dq to ddq). That can break the drop of RMS(Phi (w* -
    // w)) at every update (the Q of the test above is not positive definite
    // on every joint here), but leaves the symmetric part of M: the
    // normalised weight error |u| drops at every update, from every start,
    // exactly when Q2 = gain (M + M^T) - gain^2 M^T M is positive definite.
    const Dense& m = paths[j].m;
    const Dense mtm = product(transposed(m), m);
    Dense q2 = denseZero(m.size());
    for (std::size_t p = 0; p < m.size(); ++p) {
      for (std::size_t q = 0; q < m.size(); ++q) {
        q2[p][q] = options.gain * (m[p][q] + m[q][p]) -
                   options.gain * options.gain * mtm[p][q];
      }
    }
    CHECK(positiveDefinite(q2));
  }
  constexpr int kUpdates = 100;
  std::array<std::vector<double>, kArm> weight_error;
  const auto remaining = runNormalisedGradient(phi, planted, options, paths,
                                               kUpdates, &weight_error);
  for (int j = 0; j < kArm; ++j) {
    for (int k = 0; k < kUpdates; ++k) {
      INFO("update " << k << ", joint " << j);
      CHECK(weight_error[j][k + 1] < weight_error[j][k]);
    }
    INFO("joint " << j << ": remaining after " << kUpdates
                  << " updates: " << remaining[j].back()
                  << ", normalised weight error "
                  << weight_error[j].back() / weight_error[j].front());
    CHECK(remaining[j].back() <= 0.1);  // reduced by 10 within the run
  }
}
