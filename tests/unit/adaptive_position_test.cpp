// The pure logic of the simulation-only Slotine and Li study
// (apps/study/adaptive_position.hpp): the Christoffel-consistent model
// products and the properties the Slotine and Li proof needs, the
// proper-form regressors against the library's desired-form ones, the
// in-loop update's rate and convergence, the advance pairing, the clamp,
// the negligible-column rule and the band split.
#include "study/adaptive_position.hpp"

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "rtctrl/learn/regressor_feedforward.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"

namespace learn = rtctrl::learn;
namespace study = x7::study;
using learn::JointArray;
using learn::RegressorRow;
using rtctrl::model::ChainModel;
using rtctrl::model::JointMap;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";
constexpr int kDof = study::kAdaptiveDof;
constexpr int kArm = study::kAdaptiveArmJoints;
constexpr double kPi = 3.14159265358979323846;
constexpr int kG = learn::kRegressorGravity;
constexpr int kDyn = learn::kRegressorDynamics;
constexpr int kDdq = learn::kRegressorAcceleration;
constexpr int kDq = learn::kRegressorVelocity;
constexpr int kCoulomb = learn::kRegressorCoulomb;
constexpr int kConstant = learn::kRegressorConstant;

// Fixed-seed uniform samples in [-1, 1): std::mt19937_64 is specified bit
// for bit; the map to doubles is written out (the distributions are
// implementation-defined).
class Uniform {
 public:
  explicit Uniform(std::uint64_t seed) : engine_(seed) {}
  double operator()() {
    return 2.0 * (static_cast<double>(engine_() >> 11) * 0x1.0p-53) - 1.0;
  }
  JointArray joints(double scale) {
    JointArray out{};
    for (auto& v : out) v = scale * (*this)();
    return out;
  }

 private:
  std::mt19937_64 engine_;
};

const JointArray kPosture = {0.3, -0.6, 0.2, -1.1, 0.4, 0.7, -0.5, 0.2};
// Inside the joint limits with room for +-0.5 rad on the arm joints and
// +-0.2 rad on the gripper (its range is -5 to 90 degrees). roki holds a
// displacement outside the limits at the limit, so dM/dq vanishes there
// and a finite difference across it cannot match the Christoffel product.
const JointArray kMidRange = {0.3, -0.6, 0.2, -1.1, 0.4, 0.7, -0.5, 0.6};

JointArray around(const JointArray& centre, const JointArray& offset) {
  JointArray out{};
  for (int i = 0; i < kDof; ++i) out[i] = centre[i] + offset[i];
  return out;
}

double dot(const JointArray& a, const JointArray& b) {
  double sum = 0.0;
  for (int i = 0; i < kDof; ++i) sum += a[i] * b[i];
  return sum;
}

double maxAbs(const JointArray& a) {
  double out = 0.0;
  for (const double v : a) out = std::max(out, std::fabs(v));
  return out;
}

double maxAbsDiff(const JointArray& a, const JointArray& b) {
  double out = 0.0;
  for (int i = 0; i < kDof; ++i) out = std::max(out, std::fabs(a[i] - b[i]));
  return out;
}

JointArray combine(double a, const JointArray& x, double b,
                   const JointArray& y) {
  JointArray out{};
  for (int i = 0; i < kDof; ++i) out[i] = a * x[i] + b * y[i];
  return out;
}

// Every joint moves about a posture that loads the arm (the gripper less).
std::vector<JointArray> movingReference(std::size_t n, double dt) {
  std::vector<JointArray> q(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double t = static_cast<double>(i) * dt;
    for (int j = 0; j < kDof; ++j) {
      const double scale = j == kDof - 1 ? 0.15 : 1.0;
      q[i][j] =
          kPosture[j] +
          scale * (0.3 * std::sin(2.0 * kPi * (0.4 + 0.07 * j) * t + 0.5 * j) +
                   0.1 * std::sin(2.0 * kPi * (1.1 + 0.05 * j) * t));
    }
  }
  return q;
}

// N desired rows with one nonzero column (joint, regressor) of the given
// values, everything else zero.
std::vector<RegressorRow> singleColumn(int joint, int regressor,
                                       const std::vector<double>& values) {
  std::vector<RegressorRow> rows(values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    rows[i][joint][regressor] = values[i];
  }
  return rows;
}

RegressorRow rowWith(int joint, int regressor, double value) {
  RegressorRow row{};
  row[joint][regressor] = value;
  return row;
}

bool sameBits(double a, double b) { return std::memcmp(&a, &b, sizeof a) == 0; }

}  // namespace

// ------------------------------------------------------- model products

TEST_CASE(
    "adaptive position: the velocity torque is a quadratic form and "
    "b its polarisation",
    "[adaptive]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  study::ModelProducts model(chain, map);
  Uniform random(20261112);
  // Tolerances [Nm]: the torques are of order 1 Nm (gravity) and 0.01 to
  // 0.1 Nm (velocity terms at 1 rad/s); every identity below holds exactly
  // in real arithmetic, so the bound only has to cover rounding.
  constexpr double kTol = 1e-10;
  for (int trial = 0; trial < 6; ++trial) {
    JointArray offset = random.joints(0.4);
    offset[kDof - 1] *= 0.5;
    const JointArray q = around(kMidRange, offset);
    const JointArray u = random.joints(1.0);
    const JointArray v = random.joints(1.0);
    const JointArray w = random.joints(1.0);
    CAPTURE(trial);
    // ID(q, 0, 0) is the gravity torque.
    REQUIRE(maxAbsDiff(model.inverseDynamics(q, JointArray{}, JointArray{}),
                       model.gravity(q)) < kTol);
    const JointArray c_v = model.velocityTorque(q, v);
    REQUIRE(maxAbs(c_v) > 1e-3);  // the test has something to check
    // No term linear in v (no joint friction or damping in ID):
    // c(-v) = c(v) and c(2 v) = 4 c(v).
    JointArray minus_v{}, two_v{};
    for (int i = 0; i < kDof; ++i) {
      minus_v[i] = -v[i];
      two_v[i] = 2.0 * v[i];
    }
    REQUIRE(maxAbsDiff(model.velocityTorque(q, minus_v), c_v) < kTol);
    REQUIRE(maxAbsDiff(model.velocityTorque(q, two_v),
                       combine(4.0, c_v, 0, c_v)) < kTol);
    // b is symmetric, b(v, v) = c(v), and bilinear.
    const JointArray b_uv = model.coriolis(q, u, v);
    const JointArray b_vu = model.coriolis(q, v, u);
    REQUIRE(maxAbsDiff(b_uv, b_vu) < kTol);
    REQUIRE(maxAbsDiff(model.coriolis(q, v, v), c_v) < kTol);
    const double alpha = 0.7;
    const double beta = -1.3;
    const JointArray b_u_mix = model.coriolis(q, u, combine(alpha, v, beta, w));
    const JointArray mix = combine(alpha, b_uv, beta, model.coriolis(q, u, w));
    REQUIRE(maxAbsDiff(b_u_mix, mix) < kTol);
    // M(q) a is linear in a and M symmetric with a positive diagonal.
    const JointArray a1 = random.joints(1.0);
    const JointArray a2 = random.joints(1.0);
    REQUIRE(maxAbsDiff(model.mass(q, combine(alpha, a1, beta, a2)),
                       combine(alpha, model.mass(q, a1), beta,
                               model.mass(q, a2))) < kTol);
    for (int r = 0; r < kDof; ++r) {
      JointArray e_r{};
      e_r[r] = 1.0;
      const JointArray column_r = model.mass(q, e_r);
      REQUIRE(column_r[r] > 0.0);
      for (int k = 0; k < kDof; ++k) {
        JointArray e_k{};
        e_k[k] = 1.0;
        REQUIRE(std::fabs(column_r[k] - model.mass(q, e_k)[r]) < kTol);
      }
    }
  }
}

TEST_CASE(
    "adaptive position: dM/dt - 2 C is skew-symmetric along a "
    "trajectory for the Christoffel product, not for any C with "
    "C(q, dq) dq = c(dq)",
    "[adaptive]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  study::ModelProducts model(chain, map);
  Uniform random(7);
  // q(t) = q0 + A sin(omega t + phase) per joint; dM/dt v by a central
  // difference of M(q(t)) v with step h. Truncation is O(h^2) (about 1e-9
  // of the scale here), rounding about eps |M v| / h (1e-12 Nm).
  JointArray amplitude = random.joints(0.5);
  amplitude[kDof - 1] *= 0.4;
  const JointArray omega =
      around(JointArray{3, 3, 3, 3, 3, 3, 3, 3}, random.joints(1.5));
  const JointArray phase = random.joints(3.0);
  const auto at = [&](double t, JointArray* q, JointArray* dq) {
    for (int i = 0; i < kDof; ++i) {
      (*q)[i] = kMidRange[i] + amplitude[i] * std::sin(omega[i] * t + phase[i]);
      (*dq)[i] = amplitude[i] * omega[i] * std::cos(omega[i] * t + phase[i]);
    }
  };
  constexpr double kH = 1e-5;
  // Stated tolerance: |x' (dM/dt - 2C) y + y' (dM/dt - 2C) x| at most 1e-6
  // of the scale |x' dM/dt y| + 2 |x' C y| + 2 |y' C x|.
  constexpr double kRelTol = 1e-6;
  for (int sample = 0; sample < 5; ++sample) {
    const double t = 0.37 * sample;
    JointArray q{}, dq{}, q_plus{}, q_minus{}, unused{};
    at(t, &q, &dq);
    at(t + kH, &q_plus, &unused);
    at(t - kH, &q_minus, &unused);
    const auto mdot = [&](const JointArray& v) {
      return combine(1.0 / (2.0 * kH), model.mass(q_plus, v), -1.0 / (2.0 * kH),
                     model.mass(q_minus, v));
    };
    for (int pair = 0; pair < 3; ++pair) {
      const JointArray x = random.joints(1.0);
      const JointArray y = random.joints(1.0);
      CAPTURE(sample, pair);
      // v' (dM/dt - 2 C) v = 0 with C v = b(dq, v)
      const double x_mdot_x = dot(x, mdot(x));
      const double x_c_x = dot(x, model.coriolis(q, dq, x));
      const double scale_xx = std::fabs(x_mdot_x) + 2.0 * std::fabs(x_c_x);
      CAPTURE(x_mdot_x, x_c_x);
      REQUIRE(scale_xx > 1e-4);
      REQUIRE(std::fabs(x_mdot_x - 2.0 * x_c_x) <= kRelTol * scale_xx);
      // and the full skew-symmetry: x' N y = -y' N x
      const double x_mdot_y = dot(x, mdot(y));
      const double y_mdot_x = dot(y, mdot(x));
      const double x_c_y = dot(x, model.coriolis(q, dq, y));
      const double y_c_x = dot(y, model.coriolis(q, dq, x));
      const double sum = (x_mdot_y - 2.0 * x_c_y) + (y_mdot_x - 2.0 * y_c_x);
      const double scale_xy =
          std::fabs(x_mdot_y) + 2.0 * std::fabs(x_c_y) + 2.0 * std::fabs(y_c_x);
      REQUIRE(std::fabs(sum) <= kRelTol * scale_xy);
      // The negative control: the rank-one factorisation C1 v =
      // c(dq) (dq' v) / |dq|^2 also gives C1 dq = c(dq) but fails the
      // property, so the check has power.
      const JointArray c_dq = model.velocityTorque(q, dq);
      const double x_c1_x = dot(x, c_dq) * dot(dq, x) / dot(dq, dq);
      REQUIRE(std::fabs(x_mdot_x - 2.0 * x_c1_x) > 1e3 * kRelTol * scale_xx);
    }
    // b(dq, dq) is the velocity torque of the trajectory itself.
    REQUIRE(maxAbsDiff(model.coriolis(q, dq, dq), model.velocityTorque(q, dq)) <
            1e-10);
  }
}

// ------------------------------------------------- proper-form regressor

TEST_CASE(
    "adaptive position: on the desired state with lambda 0 the proper "
    "regressors are the library's desired regressors",
    "[adaptive]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  study::ModelProducts model(chain, map);
  constexpr double kDt = 0.01;
  const auto qref = movingReference(300, kDt);
  const learn::RegressorOptions options;  // sigma 0.04 s, v0 0.05 rad/s
  const auto desired = learn::buildRegressors(chain, map, qref, kDt, options);
  // buildRegressors evaluates the SMOOTHED reference (a zero-phase Gaussian
  // of sigma 0.04 s, then numpy.gradient). The proper form is handed the
  // same smoothed q, dq and ddq (the study binary evaluates its reference
  // velocity and acceleration on this smoothed reference too); on the raw
  // samples the posture would differ by the smoothing (about sigma^2 / 2
  // times ddq) and the rows would not agree to rounding.
  const auto smooth = learn::smoothReference(qref, kDt, options.smooth_sigma_s);
  // tau_dyn: one inverse dynamics call minus gravity in the library, the
  // mass product plus the Christoffel product here; equal in real
  // arithmetic, so the bound covers rounding on torques of order 1 Nm.
  constexpr double kTol = 1e-10;
  for (std::size_t i = 0; i < qref.size(); i += 7) {
    CAPTURE(i);
    for (const double lambda : {0.0, 5.0}) {
      // With the measured state on the reference, e = edot = 0 and lambda
      // drops out.
      const RegressorRow proper = study::properRegressorRow(
          model, smooth.q[i], smooth.dq[i], smooth.q[i], smooth.dq[i],
          smooth.ddq[i], lambda, options.coulomb_v0_rad_s);
      for (int j = 0; j < kDof; ++j) {
        for (int l = 0; l < learn::kRegressorCount; ++l) {
          CAPTURE(lambda, j, l);
          REQUIRE(std::fabs(proper[j][l] - desired[i][j][l]) < kTol);
        }
      }
    }
  }
}

TEST_CASE(
    "adaptive position: the proper regressors use the measured state "
    "and the reference velocity",
    "[adaptive]") {
  ChainModel chain(kModelPath);
  JointMap map(chain);
  study::ModelProducts model(chain, map);
  Uniform random(99);
  const JointArray q_d = kPosture;
  const JointArray dq_d = random.joints(0.3);
  const JointArray ddq_d = random.joints(0.5);
  const JointArray q = around(q_d, random.joints(0.05));
  const JointArray dq = around(dq_d, random.joints(0.1));
  constexpr double kLambda = 7.0;
  constexpr double kV0 = 0.05;
  const RegressorRow row =
      study::properRegressorRow(model, q, dq, q_d, dq_d, ddq_d, kLambda, kV0);
  JointArray dq_r{}, ddq_r{};
  for (int i = 0; i < kDof; ++i) {
    dq_r[i] = dq_d[i] + kLambda * (q_d[i] - q[i]);
    ddq_r[i] = ddq_d[i] + kLambda * (dq_d[i] - dq[i]);
  }
  const JointArray g = model.gravity(q);
  const JointArray dynamics =
      combine(1.0, model.mass(q, ddq_r), 1.0, model.coriolis(q, dq, dq_r));
  for (int j = 0; j < kArm; ++j) {
    CAPTURE(j);
    REQUIRE(row[j][kG] == g[j]);
    REQUIRE(std::fabs(row[j][kDyn] - dynamics[j]) < 1e-12);
    REQUIRE(row[j][kDdq] == ddq_r[j]);
    REQUIRE(row[j][kDq] == dq_r[j]);
    REQUIRE(row[j][kCoulomb] == std::tanh(dq_r[j] / kV0));
    REQUIRE(row[j][kConstant] == 1.0);
  }
  for (int l = 0; l < learn::kRegressorCount; ++l) {
    REQUIRE(row[kDof - 1][l] == 0.0);  // the gripper row
  }
  REQUIRE_THROWS_AS(
      study::properRegressorRow(model, q, dq, q_d, dq_d, ddq_d, -1.0, kV0),
      std::invalid_argument);
  REQUIRE_THROWS_AS(
      study::properRegressorRow(model, q, dq, q_d, dq_d, ddq_d, kLambda, 0.0),
      std::invalid_argument);
  JointArray bad = q;
  bad[2] = std::numeric_limits<double>::quiet_NaN();
  REQUIRE_THROWS_AS(
      study::properRegressorRow(model, bad, dq, q_d, dq_d, ddq_d, kLambda, kV0),
      std::invalid_argument);
}

// ------------------------------------------------------- in-loop update

TEST_CASE(
    "adaptive position: the in-loop update reproduces the "
    "continuous-time rate gamma for small dt",
    "[adaptive]") {
  // Scalar synthetic plant on joint 3 with only one column nonzero: the
  // error is what the regressor expresses with the planted weight, e =
  // phi (w* - w), with no lag and no advance. The weight error then obeys
  // dw/dt = -gamma phi^2 / mean(phi^2) (w - w*), whose average rate is
  // gamma.
  constexpr int kJoint = 3;
  constexpr double kGamma = 2.0;
  constexpr double kDt = 1e-4;
  constexpr double kPlanted = 0.04;
  const auto run = [&](int regressor, const std::vector<double>& phi,
                       std::size_t cycles) {
    const auto gains =
        study::normalisedGains(singleColumn(kJoint, regressor, phi), kGamma);
    study::AdaptiveOffset controller(gains, kDt, 0, 10.0);
    for (std::size_t k = 0; k < cycles; ++k) {
      const double value = phi[k % phi.size()];
      JointArray s{};
      s[kJoint] = value * (kPlanted - controller.weights()[kJoint][regressor]);
      controller.cycle(rowWith(kJoint, regressor, value), s, true);
    }
    return controller.weights()[kJoint][regressor];
  };
  SECTION("a constant regressor") {
    // After 1 s the weight error is (1 - gamma dt)^(1 / dt) of the initial
    // one: exp(-gamma) to O(gamma^2 dt).
    const std::vector<double> phi(1, 1.0);
    const double ratio = (kPlanted - run(kConstant, phi, 10000)) / kPlanted;
    REQUIRE(std::fabs(ratio / std::exp(-kGamma) - 1.0) < 1e-3);
    // and it converges to the planted weight
    REQUIRE(std::fabs(run(kConstant, phi, 100000) - kPlanted) < 1e-9);
  }
  SECTION("a sinusoidal regressor, normalised by its mean square") {
    // 5 Hz over whole periods: mean(phi^2) = 0.5, the instantaneous rate
    // 2 gamma sin^2 averages to gamma.
    std::vector<double> phi(2000);  // one period at 5 Hz
    for (std::size_t k = 0; k < phi.size(); ++k) {
      phi[k] = std::sin(2.0 * kPi * 5.0 * static_cast<double>(k) * kDt);
    }
    const double ratio = (kPlanted - run(kDq, phi, 10000)) / kPlanted;
    REQUIRE(std::fabs(ratio / std::exp(-kGamma) - 1.0) < 1e-3);
  }
}

TEST_CASE(
    "adaptive position: with a delayed plant and the matching advance "
    "the update converges to the planted weight",
    "[adaptive]") {
  // The error at cycle k answers the offset sent `delay` cycles before:
  // e_k = phi_{k-D} (w* - w_{k-D}); the advance pairs phi_{k-D} with e_k.
  // At gamma 2 1/s and dt 0.01 s the loop gain per cycle is 0.02, well
  // inside the delayed integrator's bound for D = 5.
  constexpr int kJoint = 1;
  constexpr double kDt = 0.01;
  constexpr std::size_t kDelay = 5;
  constexpr double kPlanted = -0.03;
  std::vector<double> phi(400);
  for (std::size_t k = 0; k < phi.size(); ++k) {
    phi[k] = 1.0 + 0.5 * std::sin(2.0 * kPi * static_cast<double>(k) / 400.0);
  }
  const auto gains = study::normalisedGains(singleColumn(kJoint, kG, phi), 2.0);
  study::AdaptiveOffset controller(gains, kDt, kDelay, 10.0);
  std::vector<double> sent;  // phi_k (w* - w_k) of every cycle
  for (std::size_t k = 0; k < 3000; ++k) {
    const double value = phi[k % phi.size()];
    JointArray s{};
    if (k >= kDelay) s[kJoint] = sent[k - kDelay];
    const auto cycle = controller.cycle(rowWith(kJoint, kG, value), s, true);
    REQUIRE(cycle.updated == (k >= kDelay));
    sent.push_back(value * kPlanted - cycle.delta[kJoint]);
  }
  REQUIRE(std::fabs(controller.weights()[kJoint][kG] - kPlanted) <
          1e-6 * std::fabs(kPlanted));
}

// ------------------------------------------------------ advance pairing

TEST_CASE(
    "adaptive position: the advance pairs the row of Delta ago with "
    "the present error",
    "[adaptive]") {
  REQUIRE(study::advanceCycles(0.05, 0.01) == 5);
  REQUIRE(study::advanceCycles(0.0, 0.01) == 0);
  REQUIRE_THROWS_AS(study::advanceCycles(0.055, 0.01), std::invalid_argument);
  REQUIRE_THROWS_AS(study::advanceCycles(-0.01, 0.01), std::invalid_argument);

  SECTION("the ring buffer") {
    study::AdvanceBuffer buffer(3);
    for (int k = 0; k < 20; ++k) {
      buffer.push(rowWith(0, 0, k), true);
      const RegressorRow* paired = buffer.paired();
      if (k < 3) {
        REQUIRE(paired == nullptr);
      } else {
        REQUIRE(paired != nullptr);
        REQUIRE((*paired)[0][0] == k - 3);
      }
    }
    buffer.clear();
    for (int k = 0; k < 3; ++k) {
      buffer.push(rowWith(0, 0, 100 + k), true);
      REQUIRE(buffer.paired() == nullptr);
    }
    // a row pushed as not adaptable is never paired
    buffer.push(rowWith(0, 0, 103), true);
    REQUIRE((*buffer.paired())[0][0] == 100);
    buffer.push(rowWith(0, 0, 104), false);
    REQUIRE((*buffer.paired())[0][0] == 101);
    buffer.push(rowWith(0, 0, 105), false);
    buffer.push(rowWith(0, 0, 106), false);
    buffer.push(rowWith(0, 0, 107), false);
    REQUIRE(buffer.paired() == nullptr);  // 104 is paired, not adaptable
    study::AdvanceBuffer none(0);
    none.push(rowWith(0, 0, 7), true);
    REQUIRE((*none.paired())[0][0] == 7);
  }

  SECTION(
      "the update uses the row of Delta ago, the offset the present "
      "row") {
    constexpr double kDt = 0.01;
    constexpr std::size_t kDelay = 4;
    constexpr int kJoint = 5;
    const std::vector<double> desired = {1.0, -1.0};  // mean square 1
    const auto gains =
        study::normalisedGains(singleColumn(kJoint, kDq, desired), 3.0);
    study::AdaptiveOffset controller(gains, kDt, kDelay, 10.0);
    double expected = 0.0;
    std::vector<double> rows;
    for (int k = 0; k < 12; ++k) {
      const double value = 0.1 * (k + 1);
      rows.push_back(value);
      JointArray s{};
      s[kJoint] = 0.5 + k;
      const auto cycle = controller.cycle(rowWith(kJoint, kDq, value), s, true);
      if (k >= static_cast<int>(kDelay)) {
        expected += kDt * 3.0 * rows[k - kDelay] * s[kJoint];
      }
      REQUIRE(std::fabs(controller.weights()[kJoint][kDq] - expected) < 1e-15);
      REQUIRE(std::fabs(cycle.delta[kJoint] - expected * value) < 1e-15);
    }
    // a new pass drops the history, not the weights
    controller.startPass();
    JointArray s{};
    s[kJoint] = 1.0;
    const auto cycle = controller.cycle(rowWith(kJoint, kDq, 1.0), s, true);
    REQUIRE_FALSE(cycle.updated);
    REQUIRE(controller.weights()[kJoint][kDq] == expected);
  }
}

// --------------------------------------------------------------- clamp

TEST_CASE(
    "adaptive position: the offset is clamped to the limit and "
    "counted",
    "[adaptive]") {
  std::vector<RegressorRow> desired(2);
  for (auto& row : desired) {
    for (int j = 0; j < kArm; ++j) row[j][kConstant] = 1.0;
  }
  const auto gains = study::normalisedGains(desired, 1.0);
  study::AdaptiveOffset controller(gains, 0.01, 0, 0.15);
  study::RegressorMatrix w{};
  w[0][kConstant] = 0.2;     // above the limit
  w[1][kConstant] = -0.4;    // below minus the limit
  w[2][kConstant] = 0.1499;  // inside
  w[3][kConstant] = 0.15;    // on the limit: not clamped
  controller.setWeights(w);
  RegressorRow phi{};
  for (int j = 0; j < kDof; ++j) phi[j][kConstant] = 1.0;  // gripper too
  const auto cycle = controller.offset(phi);
  REQUIRE(cycle.delta[0] == 0.15);
  REQUIRE(cycle.delta[1] == -0.15);
  REQUIRE(cycle.delta[2] == 0.1499);
  REQUIRE(cycle.delta[3] == 0.15);
  REQUIRE(cycle.unclamped[0] == 0.2);
  REQUIRE(cycle.unclamped[1] == -0.4);
  REQUIRE(cycle.clamped == std::array<bool, kDof>{true, true, false, false,
                                                  false, false, false, false});
  REQUIRE(cycle.any_clamped);
  REQUIRE(cycle.delta[kDof - 1] == 0.0);  // the gripper is never offset
  // inside the limit nothing is clamped
  w[0][kConstant] = 0.0;
  w[1][kConstant] = 0.0;
  controller.setWeights(w);
  REQUIRE_FALSE(controller.offset(phi).any_clamped);
  // a non-finite sum is not hidden by the clamp
  phi[2][kConstant] = std::numeric_limits<double>::infinity();
  REQUIRE(std::isinf(controller.offset(phi).delta[2]));
  REQUIRE_FALSE(controller.offset(phi).clamped[2]);
  phi[2][kDq] = std::numeric_limits<double>::quiet_NaN();
  w[2][kDq] = 1.0;
  controller.setWeights(w);
  REQUIRE(std::isnan(controller.offset(phi).delta[2]));
  // the gripper's weights must stay zero
  w[kDof - 1][kConstant] = 1.0;
  REQUIRE_THROWS_AS(controller.setWeights(w), std::invalid_argument);
}

// --------------------------------------------------- negligible columns

TEST_CASE("adaptive position: a negligible regressor is not adapted",
          "[adaptive]") {
  std::vector<RegressorRow> desired(100);
  for (std::size_t i = 0; i < desired.size(); ++i) {
    const double sign = i % 2 == 0 ? 1.0 : -1.0;
    desired[i][2][kG] = 1e-7 * sign;   // RMS 1e-7: negligible
    desired[i][2][kDq] = 1e-5 * sign;  // RMS 1e-5: adapted
    desired[i][kDof - 1][kG] = 1.0;    // the gripper: never adapted
  }
  const auto gains = study::normalisedGains(desired, 4.0);
  REQUIRE_FALSE(gains.adapted[2][kG]);
  REQUIRE(gains.gain[2][kG] == 0.0);
  REQUIRE(gains.adapted[2][kDq]);
  REQUIRE(std::fabs(gains.gain[2][kDq] / (4.0 / 1e-10) - 1.0) < 1e-12);
  REQUIRE(std::fabs(gains.rms[2][kDq] - 1e-5) < 1e-18);
  REQUIRE_FALSE(gains.adapted[2][kConstant]);  // all-zero column
  REQUIRE_FALSE(gains.adapted[kDof - 1][kG]);
  REQUIRE(gains.gain[kDof - 1][kG] == 0.0);
  study::AdaptiveOffset controller(gains, 0.01, 0, 10.0);
  RegressorRow phi{};
  phi[2][kG] = 1e6;  // whatever the measured-state value
  phi[2][kDq] = 1e-5;
  phi[kDof - 1][kG] = 1.0;
  JointArray s{};
  s[2] = 1.0;
  s[kDof - 1] = 1.0;
  for (int k = 0; k < 10; ++k) controller.cycle(phi, s, true);
  REQUIRE(sameBits(controller.weights()[2][kG], 0.0));
  REQUIRE(controller.weights()[2][kDq] != 0.0);
  REQUIRE(sameBits(controller.weights()[kDof - 1][kG], 0.0));

  // On the arm model, joint 0 turns about the vertical axis: its gravity
  // column is negligible on any reference and is not adapted.
  ChainModel chain(kModelPath);
  JointMap map(chain);
  const auto rows = learn::buildRegressors(
      chain, map, movingReference(200, 0.01), 0.01, learn::RegressorOptions{});
  const auto model_gains = study::normalisedGains(rows, 1.0);
  REQUIRE_FALSE(model_gains.adapted[0][kG]);
  REQUIRE(model_gains.adapted[1][kG]);
  REQUIRE_THROWS_AS(study::normalisedGains({}, 1.0), std::invalid_argument);
  REQUIRE_THROWS_AS(study::normalisedGains(rows, -1.0), std::invalid_argument);
}

// ------------------------------------------------------------ band split

TEST_CASE("adaptive position: the band split is follow_learnability's",
          "[adaptive]") {
  // On exact DFT bins: 400 samples at 0.01 s put 1, 3 and 5 Hz on bins 4,
  // 12 and 20. The mean and everything up to and including 3 Hz stay in
  // the lower band; the 5 Hz component is the upper band.
  constexpr double kDt = 0.01;
  for (const std::size_t n : {std::size_t{400}, std::size_t{401}}) {
    std::vector<double> x(n), high(n);
    for (std::size_t i = 0; i < n; ++i) {
      const double t = static_cast<double>(i) * kDt;
      high[i] = 0.2 * std::sin(2.0 * kPi * 5.0 * t + 0.3);
      x[i] = 0.7 + std::sin(2.0 * kPi * 1.0 * t) +
             0.5 * std::cos(2.0 * kPi * 3.0 * t) + high[i];
    }
    const auto above = study::aboveBand(x, kDt, 3.0);
    CAPTURE(n);
    if (n == 400) {
      for (std::size_t i = 0; i < n; ++i) {
        REQUIRE(std::fabs(above[i] - high[i]) < 1e-12);
      }
    } else {
      // Off the bins the split leaks (as numpy's does); the upper band
      // still carries about the 5 Hz power and little of the rest.
      const double rms = study::seriesRms(above);
      REQUIRE(std::fabs(rms - study::seriesRms(high)) <
              0.2 * study::seriesRms(high));
    }
  }
  // A constant has no upper band.
  const auto flat = study::aboveBand(std::vector<double>(50, 3.0), kDt, 3.0);
  for (const double v : flat) REQUIRE(std::fabs(v) < 1e-12);
  REQUIRE_THROWS_AS(study::aboveBand({}, kDt, 3.0), std::invalid_argument);
}
