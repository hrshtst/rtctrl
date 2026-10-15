#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/link_force.hpp"

namespace rtctrl::model {

class ChainModel;
class Trajectory;

// Quasi-static force-budget SCREEN. Contract: compute the largest force
// amplitude satisfying user-supplied joint holding-torque allowances at
// sampled reference poses under a quasi-static model. It is a model-based
// screen, not a hardware safety or fuse rating: it excludes motion
// transients, off-path yielding, fuse release, friction, bearing and
// bracket loads, and individual finger-linkage loads, and denser sampling
// does not make it a continuous-path guarantee.

// Per-joint bound on the amplitude F >= 0 of a force whose joint torque
// per newton is a, against gravity load g and allowance allow: the
// largest F_max such that EVERY 0 <= F <= F_max satisfies
// |g - a F| <= allow.
//  * Infeasible: |g| > allow, even if some F > 0 would compensate it.
//  * Unbounded:  a == 0; this modeled constraint imposes no bound (value
//    +inf). It never means unlimited physical loading.
//  * Bounded:    (allow + sgn(a) g) / |a|, evaluated without avoidable
//    overflow.
//  * Overflow:   the true bound is not representable (value NaN).
struct JointBound {
  enum class Kind { Bounded, Unbounded, Infeasible, Overflow };
  Kind kind = Kind::Bounded;
  double value = 0.0;
};
// Throws std::invalid_argument unless g and a are finite and allow is
// finite and > 0.
JointBound jointForceBound(double gravity_nm, double torque_per_newton,
                           double allowance_nm);

inline constexpr double kMaxBudgetSamples = 1e7;

struct ForceBudgetOptions {
  std::array<double, kCanonicalDof> joint_allowance_nm{};  // finite, > 0
  double sample_rate_hz = 1000.0;                          // finite, > 0
};

enum class BudgetStatus {
  Ok,
  Infeasible,
  DegenerateAnchor,
  NonFiniteEvaluation
};
const char* budgetStatusName(BudgetStatus status);

struct ForceBudgetSample {
  double time_s = 0.0;
  BudgetStatus status = BudgetStatus::Ok;
  std::array<double, 3> point_world{};      // application point
  std::array<double, 3> direction_world{};  // unit force direction
  double anchor_distance_m = 0.0;           // Anchor mode; NaN for World
  std::array<double, kCanonicalDof> gravity_nm{};
  std::array<double, kCanonicalDof> torque_per_newton{};  // (Jᵀu)_i
  std::array<double, kCanonicalDof> joint_bound_n{};      // inf: unbounded
  double amplitude_bound_n = 0.0;         // min over joints; NaN unless Ok
  int binding_joint = -1;                 // -1 when every joint is unbounded
  double disturbance_only_bound_n = 0.0;  // the same with gravity = 0
  int disturbance_only_binding_joint = -1;
  int failure_joint = -1;  // first failing joint in canonical order
};

struct ForceBudgetResult {
  // First failure in time; Ok only if every sample is Ok.
  BudgetStatus status = BudgetStatus::Ok;
  std::vector<ForceBudgetSample> samples;
  // Defined (not NaN) only when status is Ok; +inf when no sampled joint
  // constraint imposes a bound. The binding time is the earliest sample
  // attaining the minimum.
  double amplitude_bound_n = 0.0;
  double binding_time_s = 0.0;
  int binding_joint = -1;
  double disturbance_only_bound_n = 0.0;
  double first_failure_time_s = 0.0;  // NaN when Ok
  int first_failure_joint = -1;
  double min_anchor_distance_m = 0.0;  // NaN for World mode
};

// Samples the reference at N + 1 points, N = ceil(duration * rate)
// (one sample at t = 0 for a zero duration), evaluating at each pose the
// gravity load and the torque per newton of the spec's force at its
// application point and direction (the profile and magnitude are not
// used). Invalid inputs throw std::invalid_argument before sampling: the
// spec (validateLinkForce, and the link must exist in the model), a
// trajectory size other than kCanonicalDof, a non-finite or negative
// duration, a non-finite or non-positive rate or allowance, or more than
// kMaxBudgetSamples intervals. Evaluation outcomes are returned as status,
// never thrown; any non-finite pose, gravity, point, direction, or torque
// coefficient, and any joint Overflow, is NonFiniteEvaluation.
ForceBudgetResult evaluateForceBudget(ChainModel& model, const JointMap& map,
                                      const Trajectory& reference,
                                      const LinkForceSpec& spec,
                                      const ForceBudgetOptions& options);

// The contract, qualification, and exclusion lines every report carries.
std::vector<std::string> forceBudgetPreamble();

// Writes a schema-versioned CSV, refusing an existing path ("wx"). The
// leading '#' lines hold forceBudgetPreamble() followed by the caller's
// metadata lines. Throws std::runtime_error on any I/O failure.
void writeForceBudgetCsv(const std::string& path,
                         const ForceBudgetResult& result,
                         const std::vector<std::string>& metadata);

}  // namespace rtctrl::model
