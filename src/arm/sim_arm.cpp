#include "rtctrl/arm/sim_arm.hpp"

#include "rtctrl/arm/link_force_log.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace rtctrl::arm {

using model::kCanonicalDof;
using model::kModelDof;
using model::ZVector;

namespace {

double axisInertia(rkChain* chain, int link_id) {
  // The link's moment of inertia about its own joint axis (z in the
  // link frame, roki's revolute convention): I_zz about the COM plus
  // the parallel-axis term. Distal links are not included, which only
  // underestimates — safe for a stability-bounded damping coefficient.
  rkLink* link = rkChainLink(chain, link_id);
  const zMat3D* inertia = rkLinkInertia(link);
  const zVec3D* com = rkLinkCOM(link);
  const double mass = rkLinkMass(link);
  return inertia->c.zz + mass * (com->c.x * com->c.x + com->c.y * com->c.y);
}

}  // namespace

SimArm::SimArm(Options options)
    : options_(std::move(options)),
      model_(options_.model_path),
      map_(model_) {
  if (static_cast<int>(options_.effort_limit8.size()) != kCanonicalDof) {
    throw std::invalid_argument("SimArm: effort_limit8 must have 8 entries");
  }

  if (!options_.initial_q8.empty()) {
    if (static_cast<int>(options_.initial_q8.size()) != kCanonicalDof) {
      throw std::invalid_argument("SimArm: initial_q8 must have 8 entries");
    }
    ZVector q8(kCanonicalDof);
    for (int i = 0; i < kCanonicalDof; ++i) q8[i] = options_.initial_q8[i];
    map_.expand(q8, q9_);
  }

  mass_mat_ = zMatAllocSqr(kModelDof);
  if (mass_mat_ == nullptr) throw std::bad_alloc();

  auto perJoint = [&](int canonical, int offset, int link_id) {
    limit_lo_[offset] = model_.jointMin(link_id);
    limit_hi_[offset] = model_.jointMax(link_id);
    effort9_[offset] =
        options_.effort_limit8[std::min(canonical, kCanonicalDof - 1)];
    damping9_[offset] = options_.numeric_damping_ratio *
                        axisInertia(model_.chain(), link_id) /
                        options_.sim_dt;
  };
  for (int i = 0; i < kCanonicalDof; ++i) {
    perJoint(i, map_.rokiOffset(i), map_.linkId(i));
  }
  perJoint(kCanonicalDof - 1, map_.rokiOffsetFingerB(), map_.linkIdFingerB());
}

SimArm::~SimArm() { zMatFree(mass_mat_); }

double SimArm::fingerBDis() const { return q9_[map_.rokiOffsetFingerB()]; }

bool SimArm::activate() {
  // No motion is COMMANDED on activation: position-mode goals snap to
  // the present posture (held); current-mode goals initialize to zero
  // torque, so the simulated arm falls under gravity once stepped —
  // same contract as the hardware.
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(cmd_.q.get(), i) = q9_[map_.rokiOffset(i)];
    zVecElemNC(cmd_.dq.get(), i) = 0.0;
    zVecElemNC(cmd_.tau.get(), i) = 0.0;
    zVecElemNC(cmd_.effort_limit.get(), i) = options_.effort_limit8[i];
  }
  cmd_.mode = mode_;
  active_ = true;
  return true;
}

bool SimArm::deactivate() {
  zVecZero(cmd_.q.get());
  zVecZero(cmd_.dq.get());
  zVecZero(cmd_.tau.get());
  zVecZero(cmd_.effort_limit.get());
  active_ = false;
  return true;
}

bool SimArm::setMode(ControlMode mode) {
  if (active_) return false;  // mode changes only while deactivated
  mode_ = mode;
  return true;
}

bool SimArm::readState(JointState& state, CommandSnapshot* cmds) {
  if (lf_faulted_) return false;  // never expose a rejected state
  for (int i = 0; i < kCanonicalDof; ++i) {
    const int offset = map_.rokiOffset(i);
    zVecElemNC(state.q.get(), i) = q9_[offset];
    zVecElemNC(state.dq.get(), i) = v9_[offset];
    zVecElemNC(state.tau.get(), i) = applied_tau8_[i];
  }
  state.t = time_;
  state.seq = seq_;
  if (cmds != nullptr) {
    cmds->applied = applied_rec_;
    cmds->last_attempt = attempt_rec_;
  }
  return true;
}

bool SimArm::writeCommand(const JointCommand& cmd, CommandReceipt* receipt) {
  if (lf_faulted_ || cmd.mode != mode_) {  // mode is fixed while active
    if (receipt != nullptr) *receipt = {false, 0, time_};
    return false;
  }
  zVecCopyNC(cmd.q.get(), cmd_.q.get());
  zVecCopyNC(cmd.dq.get(), cmd_.dq.get());
  zVecCopyNC(cmd.tau.get(), cmd_.tau.get());
  zVecCopyNC(cmd.effort_limit.get(), cmd_.effort_limit.get());
  cmd_.mode = cmd.mode;
  // Synchronous application: the accepted command IS the actuator goal
  // from the next step, so submit/attempt/apply coincide.
  ++target_seq_;
  attempt_rec_ = {true, target_seq_, time_, true};
  auto& rec = applied_rec_;
  rec.valid = true;
  rec.target_seq = target_seq_;
  rec.first_cycle = rec.latest_cycle = seq_;
  rec.first_time = rec.latest_time = time_;
  rec.mode = static_cast<std::uint8_t>(cmd.mode);
  for (int i = 0; i < kCanonicalDof; ++i) {
    double v = 0.0;
    std::uint8_t flag = 0;
    rec.effort_limit[i] = 0.0;
    switch (cmd.mode) {
      case ControlMode::Position: v = zVecElemNC(cmd.q.get(), i); break;
      case ControlMode::CurrentBasedPosition:
        v = zVecElemNC(cmd.q.get(), i);
        rec.effort_limit[i] = std::clamp(
            std::fabs(zVecElemNC(cmd.effort_limit.get(), i)), 0.0,
            options_.effort_limit8[i]);
        if (rec.effort_limit[i] !=
            std::fabs(zVecElemNC(cmd.effort_limit.get(), i))) {
          flag |= kCmdClamped;
        }
        break;
      case ControlMode::Velocity: v = zVecElemNC(cmd.dq.get(), i); break;
      case ControlMode::Current: {
        const double lim = options_.effort_limit8[i];
        const double raw = zVecElemNC(cmd.tau.get(), i);
        v = std::clamp(raw, -lim, lim);
        if (v != raw) flag |= kCmdClamped;
        break;
      }
    }
    rec.applied[i] = v;
    rec.flags[i] = flag;
  }
  if (receipt != nullptr) *receipt = {true, target_seq_, time_};
  return true;
}

void SimArm::computeTorques() {
  // Canonical torque for each servo-equivalent joint.
  double tau8[kCanonicalDof] = {};
  if (active_) {
    for (int i = 0; i < kCanonicalDof; ++i) {
      const int offset = map_.rokiOffset(i);
      const double q = q9_[offset];
      const double dq = v9_[offset];
      switch (mode_) {
        case ControlMode::Position:
          tau8[i] = options_.kp * (zVecElemNC(cmd_.q.get(), i) - q) -
                    options_.kd * dq;
          break;
        case ControlMode::CurrentBasedPosition: {
          const double limit = std::clamp(
              std::fabs(zVecElemNC(cmd_.effort_limit.get(), i)), 0.0,
              options_.effort_limit8[i]);
          tau8[i] = std::clamp(
              options_.kp * (zVecElemNC(cmd_.q.get(), i) - q) -
                  options_.kd * dq,
              -limit, limit);
          break;
        }
        case ControlMode::Velocity:
          tau8[i] = options_.kv * (zVecElemNC(cmd_.dq.get(), i) - dq);
          break;
        case ControlMode::Current:
          tau8[i] = zVecElemNC(cmd_.tau.get(), i);
          break;
      }
      applied_tau8_[i] = tau8[i];
    }
  } else {
    zVecZero(applied_tau8_.get());
  }

  // Fingers: split the commanded gripper effort evenly, then enforce the
  // mimic linkage as a penalty constraint, equal and opposite.
  const int off_a = map_.rokiOffset(kCanonicalDof - 1);
  const int off_b = map_.rokiOffsetFingerB();
  const double couple = -options_.couple_k * (q9_[off_b] - q9_[off_a]) -
                        options_.couple_c * (v9_[off_b] - v9_[off_a]);
  const double grip = 0.5 * tau8[kCanonicalDof - 1];

  for (int i = 0; i < kCanonicalDof - 1; ++i) {
    const int offset = map_.rokiOffset(i);
    tau9_[offset] = tau8[i] + zVecElemNC(disturbance_.get(), i) -
                    damping9_[offset] * v9_[offset];
  }
  tau9_[off_a] = grip - couple +
                 zVecElemNC(disturbance_.get(), kCanonicalDof - 1) -
                 damping9_[off_a] * v9_[off_a];
  tau9_[off_b] = grip + couple + zVecElemNC(disturbance_.get(), kCanonicalDof) -
                 damping9_[off_b] * v9_[off_b];

  for (int i = 0; i < kModelDof; ++i) {
    tau9_[i] = std::clamp(tau9_[i], -effort9_[i], effort9_[i]);
  }
}

void SimArm::substep(double dt) {
  computeTorques();

  // Forward dynamics: (M + diag(J_reflected)) qdd = tau - bias.
  // Same equation rkChainFD solves, built manually so the reflected
  // motor inertia can be added to the diagonal.
  rkChain* chain = model_.chain();
  // Keep this a pure state commit even if a diagnostic registered IK state
  // through the public chain() hook: rkChainFK() would silently solve IK.
  rkChainSetJointDisAll(chain, q9_.get());
  rkChainUpdateFK(chain);
  rkChainSetJointVelAll(chain, v9_.get());
  rkChainInertiaMatBiasVecG(chain, mass_mat_, rhs9_.get(), RK_GRAVITY6D);
  for (int i = 0; i < kModelDof; ++i) {
    zMatElemNC(mass_mat_, i, i) += options_.reflected_inertia;
  }
  zVecSubDRC(rhs9_.get(), tau9_.get());   // rhs = bias - tau
  zVecRevDRC(rhs9_.get());                // rhs = tau - bias
  if (zLESolveGauss(mass_mat_, rhs9_.get(), acc9_.get()) == nullptr) {
    zVecZero(acc9_.get());
  }

  // Semi-implicit Euler with inelastic joint stops.
  for (int i = 0; i < kModelDof; ++i) {
    v9_[i] += acc9_[i] * dt;
    q9_[i] += v9_[i] * dt;
    if (q9_[i] < limit_lo_[i]) {
      q9_[i] = limit_lo_[i];
      if (v9_[i] < 0.0) v9_[i] = 0.0;
    } else if (q9_[i] > limit_hi_[i]) {
      q9_[i] = limit_hi_[i];
      if (v9_[i] > 0.0) v9_[i] = 0.0;
    }
  }
  time_ += dt;
}

bool SimArm::step() {
  if (!std::isfinite(options_.control_dt) || options_.control_dt <= 0.0 ||
      !std::isfinite(options_.sim_dt) || options_.sim_dt <= 0.0) {
    return false;
  }
  const bool advanced =
      lf_prepared_ || lf_faulted_ ? stepWithLinkForce() : stepPlain();
  // Recording failures never reject physics, but they fail the step.
  if (lf_log_ != nullptr && !lf_log_->write(lf_interval_)) {
    lf_log_failed_ = true;
  }
  return advanced && !lf_log_failed_;
}

bool SimArm::stepPlain() {
  // Without a link-force spec: the original arithmetic, plus interval
  // bookkeeping that reads but never feeds back into the state.
  LinkForceInterval& interval = lf_interval_;
  interval = LinkForceInterval{};
  interval.attempt = lf_attempts_++;
  interval.start_seq = interval.end_seq = seq_;
  interval.start_s = interval.end_s = time_;
  double remaining = options_.control_dt;
  while (remaining > 0.0) {
    const double dt = std::min(options_.sim_dt, remaining);
    const double eval = time_;
    substep(dt);
    ++interval.substeps_completed;
    interval.last_eval_s = eval;
    interval.last_substep_dt = dt;
    interval.end_s = time_;
    for (int i = 0; i < kModelDof; ++i) {
      if (!std::isfinite(q9_[i]) || !std::isfinite(v9_[i])) {
        interval.status = LinkForceStatus::NonFiniteState;
        return false;
      }
    }
    remaining -= dt;
    if (remaining < 1e-15 * options_.control_dt) remaining = 0.0;
  }
  if (log_ != nullptr) {
    log_->frame(options_.control_dt, q9_.get());
  }
  ++seq_;  // one fresh feedback sample per control step
  interval.complete = true;
  interval.end_seq = seq_;
  return true;
}

bool SimArm::stepWithLinkForce() {
  LinkForceInterval& interval = lf_interval_;
  interval = LinkForceInterval{};
  interval.attempt = lf_attempts_++;
  interval.start_seq = interval.end_seq = seq_;
  interval.start_s = interval.end_s = time_;
  interval.prepared = lf_prepared_;
  interval.started = lf_started_;
  if (lf_faulted_) {
    interval.status = LinkForceStatus::FaultLatched;
    return false;
  }
  double remaining = options_.control_dt;
  while (remaining > 0.0) {
    const double dt = std::min(options_.sim_dt, remaining);
    const LinkForceStatus status = substepWithLinkForce(dt);
    if (status != LinkForceStatus::Ok) {
      // Nothing of the rejected substep was committed: the clock and the
      // state stay at the last completed substep, and the arm latches.
      interval.status = status;
      interval.rejected = true;
      interval.rejected_eval_s = time_;
      interval.end_s = time_;
      lf_faulted_ = true;
      return false;
    }
    remaining -= dt;
    if (remaining < 1e-15 * options_.control_dt) remaining = 0.0;
  }
  if (log_ != nullptr) {
    log_->frame(options_.control_dt, q9_.get());
  }
  ++seq_;
  interval.complete = true;
  interval.end_seq = seq_;
  interval.end_s = time_;
  return true;
}

LinkForceStatus SimArm::substepWithLinkForce(double dt) {
  computeTorques();

  rkChain* chain = model_.chain();
  rkChainSetJointDisAll(chain, q9_.get());
  rkChainUpdateFK(chain);
  rkChainSetJointVelAll(chain, v9_.get());

  const double eval = time_;
  const auto point =
      model_.linkPointWorldPos(lf_link_, lf_spec_.point_link_m);
  detail::SubstepFacts facts;
  double scale = 0.0;
  std::array<double, 3> force{};
  std::array<double, 3> force_impulse = lf_interval_.force_impulse_ns;
  std::array<double, kCanonicalDof> joint_impulse =
      lf_interval_.joint_impulse_nms;
  if (lf_started_) {
    scale = lf_spec_.profile.scale(eval - lf_start_time_);
    facts.force_requested = scale * lf_spec_.magnitude_n > 0.0;
  }
  if (facts.force_requested) {
    std::array<double, 3> unit{};
    facts.direction_defined = model::linkForceDirection(lf_spec_, point, &unit);
    if (facts.direction_defined) {
      const double magnitude = scale * lf_spec_.magnitude_n;
      for (int k = 0; k < 3; ++k) force[k] = magnitude * unit[k];
      model_.forceJointTorqueModel(lf_link_, lf_spec_.point_link_m, force,
                                   ext9_.get());
      map_.reduceTorque(ext9_.get(), ext8_.get());
      bool finite = true;
      for (int k = 0; k < 3; ++k) {
        force_impulse[k] += force[k] * dt;
        finite = finite && std::isfinite(point[k]) &&
                 std::isfinite(force[k]) && std::isfinite(force_impulse[k]);
      }
      for (int i = 0; i < kModelDof; ++i) {
        finite = finite && std::isfinite(ext9_[i]);
      }
      for (int i = 0; i < kCanonicalDof; ++i) {
        joint_impulse[i] += ext8_[i] * dt;
        finite = finite && std::isfinite(joint_impulse[i]);
      }
      facts.force_finite = finite;
    }
  }
  LinkForceStatus status = detail::classifySubstep(facts);
  if (status != LinkForceStatus::Ok) return status;

  // The same equation as substep(), with J^T f added after the actuator
  // effort clamp (tau9_ is already clamped).
  rkChainInertiaMatBiasVecG(chain, mass_mat_, rhs9_.get(), RK_GRAVITY6D);
  for (int i = 0; i < kModelDof; ++i) {
    zMatElemNC(mass_mat_, i, i) += options_.reflected_inertia;
  }
  zVecSubDRC(rhs9_.get(), tau9_.get());  // rhs = bias - tau
  zVecRevDRC(rhs9_.get());               // rhs = tau - bias
  if (facts.force_requested) zVecAddDRC(rhs9_.get(), ext9_.get());
  facts.solve_ok =
      zLESolveGauss(mass_mat_, rhs9_.get(), acc9_.get()) != nullptr;
  status = detail::classifySubstep(facts);
  if (status != LinkForceStatus::Ok) return status;

  // Staged semi-implicit Euler with inelastic joint stops: the same
  // operations as substep(), committed only if finite.
  bool finite = true;
  for (int i = 0; i < kModelDof; ++i) {
    double v = v9_[i] + acc9_[i] * dt;
    double q = q9_[i] + v * dt;
    if (q < limit_lo_[i]) {
      q = limit_lo_[i];
      if (v < 0.0) v = 0.0;
    } else if (q > limit_hi_[i]) {
      q = limit_hi_[i];
      if (v > 0.0) v = 0.0;
    }
    q_next_[i] = q;
    v_next_[i] = v;
    finite = finite && std::isfinite(q) && std::isfinite(v);
  }
  facts.state_finite = finite;
  status = detail::classifySubstep(facts);
  if (status != LinkForceStatus::Ok) return status;

  for (int i = 0; i < kModelDof; ++i) {
    q9_[i] = q_next_[i];
    v9_[i] = v_next_[i];
  }
  time_ += dt;

  LinkForceInterval& interval = lf_interval_;
  ++interval.substeps_completed;
  interval.last_eval_s = eval;
  interval.last_substep_dt = dt;
  interval.last_scale = scale;
  interval.last_point_world = point;
  interval.end_s = time_;
  if (facts.force_requested) {
    interval.last_force_world = force;
    for (int i = 0; i < kCanonicalDof; ++i) {
      interval.last_joint_torque_nm[i] = ext8_[i];
    }
    interval.force_impulse_ns = force_impulse;
    interval.joint_impulse_nms = joint_impulse;
    interval.active = true;
    if (!lf_applied_) lf_first_applied_ = eval;
    lf_applied_ = true;
    lf_last_applied_ = eval;
  } else {
    interval.last_force_world = {};
    interval.last_joint_torque_nm = {};
  }
  return LinkForceStatus::Ok;
}

void SimArm::setLinkForce(const model::LinkForceSpec& spec) {
  model::validateLinkForce(spec);
  const int link = model_.linkIndex(spec.link);
  if (link < 0) {
    throw std::invalid_argument("SimArm::setLinkForce: unknown link '" +
                                spec.link + "'");
  }
  lf_spec_ = spec;
  lf_link_ = link;
  lf_prepared_ = true;
  lf_started_ = false;
}

void SimArm::startLinkForce(double start_time) {
  if (!lf_prepared_) {
    throw std::logic_error("SimArm::startLinkForce: no link force is set");
  }
  if (!std::isfinite(start_time)) {
    throw std::invalid_argument(
        "SimArm::startLinkForce: start time must be finite");
  }
  lf_start_time_ = start_time;
  lf_started_ = true;
}

void SimArm::clearLinkForce() {
  lf_prepared_ = false;
  lf_started_ = false;
  lf_link_ = -1;
}

void SimArm::setDisturbance(int canonical_or_finger_b, double torque) {
  const int index = canonical_or_finger_b < 0 ? kCanonicalDof  // finger_b
                                              : canonical_or_finger_b;
  zVecElemNC(disturbance_.get(), index) = torque;
}

}  // namespace rtctrl::arm
