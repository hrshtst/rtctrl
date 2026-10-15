#include "rtctrl/model/force_budget.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>

#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"

namespace rtctrl::model {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

bool finite3(const std::array<double, 3>& v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument("force budget: " + message);
  }
}

// Minimum over joints of the per-joint bounds; returns the first failing
// joint's kind instead when any joint is Infeasible or Overflow.
struct Combined {
  JointBound::Kind failure = JointBound::Kind::Bounded;
  int failure_joint = -1;
  double bound = kInf;
  int binding_joint = -1;
};

Combined combine(const std::array<double, kCanonicalDof>& gravity,
                 const std::array<double, kCanonicalDof>& per_newton,
                 const std::array<double, kCanonicalDof>& allowance,
                 std::array<double, kCanonicalDof>* joint_bound) {
  Combined out;
  for (int i = 0; i < kCanonicalDof; ++i) {
    const JointBound jb =
        jointForceBound(gravity[i], per_newton[i], allowance[i]);
    if (joint_bound != nullptr) (*joint_bound)[i] = jb.value;
    if (jb.kind == JointBound::Kind::Infeasible ||
        jb.kind == JointBound::Kind::Overflow) {
      if (out.failure_joint < 0) {
        out.failure = jb.kind;
        out.failure_joint = i;
      }
      continue;
    }
    if (jb.value < out.bound) {
      out.bound = jb.value;
      out.binding_joint = i;
    }
  }
  return out;
}

// Fields a failure prevented from being evaluated read NaN, never zero.
void clearEvaluation(ForceBudgetSample* sample) {
  sample->direction_world.fill(kNaN);
  sample->gravity_nm.fill(kNaN);
  sample->torque_per_newton.fill(kNaN);
  sample->joint_bound_n.fill(kNaN);
}

void markFailure(ForceBudgetSample* sample, BudgetStatus status, int joint) {
  sample->status = status;
  sample->failure_joint = joint;
  sample->amplitude_bound_n = kNaN;
  sample->binding_joint = -1;
  sample->disturbance_only_bound_n = kNaN;
  sample->disturbance_only_binding_joint = -1;
}

}  // namespace

JointBound jointForceBound(double gravity_nm, double torque_per_newton,
                           double allowance_nm) {
  require(std::isfinite(gravity_nm) && std::isfinite(torque_per_newton),
          "joint bound needs finite gravity and torque per newton");
  require(std::isfinite(allowance_nm) && allowance_nm > 0.0,
          "joint allowance must be finite and > 0");
  if (std::abs(gravity_nm) > allowance_nm) {
    return {JointBound::Kind::Infeasible, kNaN};
  }
  if (torque_per_newton == 0.0) return {JointBound::Kind::Unbounded, kInf};
  const double denominator = std::abs(torque_per_newton);
  const double signed_g = torque_per_newton > 0.0 ? gravity_nm : -gravity_nm;
  // Subtraction: |g| <= allow, so the numerator cannot overflow. Addition:
  // divide first so the numerator cannot overflow either.
  const double bound =
      signed_g < 0.0 ? (allowance_nm + signed_g) / denominator
                     : allowance_nm / denominator + signed_g / denominator;
  if (!std::isfinite(bound)) return {JointBound::Kind::Overflow, kNaN};
  return {JointBound::Kind::Bounded, bound};
}

const char* budgetStatusName(BudgetStatus status) {
  switch (status) {
    case BudgetStatus::Ok:
      return "ok";
    case BudgetStatus::Infeasible:
      return "infeasible";
    case BudgetStatus::DegenerateAnchor:
      return "degenerate_anchor";
    case BudgetStatus::NonFiniteEvaluation:
      return "non_finite_evaluation";
  }
  return "unknown";
}

ForceBudgetResult evaluateForceBudget(ChainModel& model, const JointMap& map,
                                      const Trajectory& reference,
                                      const LinkForceSpec& spec,
                                      const ForceBudgetOptions& options) {
  validateLinkForce(spec);
  const int link = model.linkIndex(spec.link);
  require(link >= 0, "unknown link '" + spec.link + "'");
  require(reference.size() == kCanonicalDof,
          "reference must have " + std::to_string(kCanonicalDof) + " joints");
  const double duration = reference.duration();
  require(std::isfinite(duration) && duration >= 0.0,
          "reference duration must be finite and >= 0");
  require(std::isfinite(options.sample_rate_hz) && options.sample_rate_hz > 0.0,
          "sample rate must be finite and > 0");
  for (int i = 0; i < kCanonicalDof; ++i) {
    require(std::isfinite(options.joint_allowance_nm[i]) &&
                options.joint_allowance_nm[i] > 0.0,
            "joint allowances must be finite and > 0");
  }
  const double intervals_real = std::ceil(duration * options.sample_rate_hz);
  require(intervals_real <= kMaxBudgetSamples,
          "too many samples (duration x rate exceeds 1e7)");
  const auto intervals = static_cast<std::size_t>(intervals_real);

  ForceBudgetResult result;
  result.samples.reserve(intervals + 1);
  result.amplitude_bound_n = kInf;
  result.disturbance_only_bound_n = kInf;
  result.first_failure_time_s = kNaN;
  result.min_anchor_distance_m =
      spec.direction == ForceDirection::Anchor ? kInf : kNaN;

  ZVector q8(kCanonicalDof), q9(kModelDof), g8(kCanonicalDof),
      a8(kCanonicalDof);
  const std::array<double, kCanonicalDof> zero_gravity{};

  for (std::size_t k = 0; k <= intervals; ++k) {
    ForceBudgetSample sample;
    sample.time_s = intervals == 0 ? 0.0
                                   : duration * static_cast<double>(k) /
                                         static_cast<double>(intervals);
    sample.anchor_distance_m = kNaN;
    reference.sample(sample.time_s, q8.get());

    bool finite = true;
    for (int i = 0; i < kCanonicalDof; ++i)
      finite = finite && std::isfinite(q8[i]);
    if (finite) {
      map.expand(q8.get(), q9.get());
      model.fk(q9.get());
      sample.point_world = model.linkPointWorldPos(link, spec.point_link_m);
      finite = finite3(sample.point_world);
    }
    if (finite && spec.direction == ForceDirection::Anchor) {
      sample.anchor_distance_m =
          std::hypot(spec.anchor_world_m[0] - sample.point_world[0],
                     spec.anchor_world_m[1] - sample.point_world[1],
                     spec.anchor_world_m[2] - sample.point_world[2]);
      if (sample.anchor_distance_m < result.min_anchor_distance_m) {
        result.min_anchor_distance_m = sample.anchor_distance_m;
      }
    }

    if (!finite) {
      clearEvaluation(&sample);
      markFailure(&sample, BudgetStatus::NonFiniteEvaluation, -1);
    } else if (!linkForceDirection(spec, sample.point_world,
                                   &sample.direction_world)) {
      clearEvaluation(&sample);
      markFailure(&sample, BudgetStatus::DegenerateAnchor, -1);
    } else {
      model.forceJointTorque(map, q8.get(), link, spec.point_link_m,
                             sample.direction_world, a8.get());
      model.gravityTorque(map, q8.get(), g8.get());
      int nonfinite_joint = -1;
      for (int i = 0; i < kCanonicalDof; ++i) {
        sample.torque_per_newton[i] = a8[i];
        sample.gravity_nm[i] = g8[i];
        if (nonfinite_joint < 0 &&
            (!std::isfinite(a8[i]) || !std::isfinite(g8[i]))) {
          nonfinite_joint = i;
        }
      }
      if (nonfinite_joint >= 0) {
        sample.joint_bound_n.fill(kNaN);
        markFailure(&sample, BudgetStatus::NonFiniteEvaluation,
                    nonfinite_joint);
      } else {
        const Combined full =
            combine(sample.gravity_nm, sample.torque_per_newton,
                    options.joint_allowance_nm, &sample.joint_bound_n);
        const Combined disturbance =
            combine(zero_gravity, sample.torque_per_newton,
                    options.joint_allowance_nm, nullptr);
        if (full.failure_joint >= 0) {
          markFailure(&sample,
                      full.failure == JointBound::Kind::Infeasible
                          ? BudgetStatus::Infeasible
                          : BudgetStatus::NonFiniteEvaluation,
                      full.failure_joint);
        } else if (disturbance.failure_joint >= 0) {
          // only Overflow is possible with zero gravity
          markFailure(&sample, BudgetStatus::NonFiniteEvaluation,
                      disturbance.failure_joint);
        } else {
          sample.amplitude_bound_n = full.bound;
          sample.binding_joint = full.binding_joint;
          sample.disturbance_only_bound_n = disturbance.bound;
          sample.disturbance_only_binding_joint = disturbance.binding_joint;
        }
      }
    }

    if (sample.status != BudgetStatus::Ok) {
      if (result.status == BudgetStatus::Ok) {
        result.status = sample.status;
        result.first_failure_time_s = sample.time_s;
        result.first_failure_joint = sample.failure_joint;
      }
    } else {
      // strict comparisons keep the earliest sample on ties
      if (sample.amplitude_bound_n < result.amplitude_bound_n) {
        result.amplitude_bound_n = sample.amplitude_bound_n;
        result.binding_time_s = sample.time_s;
        result.binding_joint = sample.binding_joint;
      }
      if (sample.disturbance_only_bound_n < result.disturbance_only_bound_n) {
        result.disturbance_only_bound_n = sample.disturbance_only_bound_n;
      }
    }
    result.samples.push_back(sample);
  }

  if (result.status != BudgetStatus::Ok) {
    result.amplitude_bound_n = kNaN;
    result.binding_time_s = kNaN;
    result.binding_joint = -1;
    result.disturbance_only_bound_n = kNaN;
  }
  if (spec.direction == ForceDirection::Anchor &&
      std::isinf(result.min_anchor_distance_m)) {
    result.min_anchor_distance_m = kNaN;  // no finite sample
  }
  return result;
}

std::vector<std::string> forceBudgetPreamble() {
  return {
      "contract: compute the largest force amplitude satisfying "
      "user-supplied joint holding-torque allowances at sampled reference "
      "poses under a quasi-static model",
      "meaning: every amplitude 0 <= F <= bound satisfies |gravity - "
      "torque_per_newton * F| <= allowance on every joint at every sample",
      "qualification: model-based screen; not a hardware safety or fuse "
      "rating",
      "excludes: motion transients, off-path yielding, fuse release, "
      "friction, bearing and bracket loads, individual finger-linkage loads",
      "sampling: denser sampling is not a continuous-path guarantee",
      "inf: the modeled joint-torque constraints impose no bound (not "
      "unlimited physical loading)",
  };
}

void writeForceBudgetCsv(const std::string& path,
                         const ForceBudgetResult& result,
                         const std::vector<std::string>& metadata) {
  std::FILE* file = std::fopen(path.c_str(), "wx");
  if (file == nullptr) {
    throw std::runtime_error(
        "force budget: cannot create '" + path +
        "' (it may already exist): " + std::strerror(errno));
  }
  for (const auto& line : forceBudgetPreamble()) {
    std::fprintf(file, "# %s\n", line.c_str());
  }
  for (const auto& line : metadata) std::fprintf(file, "# %s\n", line.c_str());
  std::fprintf(file,
               "schema_version,time_s,status,point_x_m,point_y_m,point_z_m,"
               "direction_x,direction_y,direction_z,anchor_distance_m,"
               "amplitude_bound_n,binding_joint,disturbance_only_bound_n,"
               "disturbance_only_binding_joint,failure_joint");
  for (int i = 0; i < kCanonicalDof; ++i) {
    std::fprintf(file,
                 ",gravity%d_nm,torque_per_newton%d_nm_per_n,joint_bound%d_n",
                 i, i, i);
  }
  std::fprintf(file, "\n");
  for (const auto& s : result.samples) {
    std::fprintf(file,
                 "1,%.9f,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,"
                 "%.9g,%d,%d",
                 s.time_s, budgetStatusName(s.status), s.point_world[0],
                 s.point_world[1], s.point_world[2], s.direction_world[0],
                 s.direction_world[1], s.direction_world[2],
                 s.anchor_distance_m, s.amplitude_bound_n, s.binding_joint,
                 s.disturbance_only_bound_n, s.disturbance_only_binding_joint,
                 s.failure_joint);
    for (int i = 0; i < kCanonicalDof; ++i) {
      std::fprintf(file, ",%.9g,%.9g,%.9g", s.gravity_nm[i],
                   s.torque_per_newton[i], s.joint_bound_n[i]);
    }
    std::fprintf(file, "\n");
  }
  const bool write_error = std::ferror(file) != 0;
  const bool close_error = std::fclose(file) != 0;
  if (write_error || close_error) {
    throw std::runtime_error("force budget: failed writing '" + path + "'");
  }
}

}  // namespace rtctrl::model
