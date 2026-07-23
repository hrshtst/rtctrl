#include "rtctrl/arm/sim_arm.hpp"

#include <algorithm>
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
  // Hold the present posture: no motion on activation.
  for (int i = 0; i < kCanonicalDof; ++i) {
    zVecElemNC(cmd_.q.get(), i) = q9_[map_.rokiOffset(i)];
    zVecElemNC(cmd_.dq.get(), i) = 0.0;
    zVecElemNC(cmd_.tau.get(), i) = 0.0;
  }
  cmd_.mode = mode_;
  active_ = true;
  return true;
}

bool SimArm::deactivate() {
  zVecZero(cmd_.q.get());
  zVecZero(cmd_.dq.get());
  zVecZero(cmd_.tau.get());
  active_ = false;
  return true;
}

bool SimArm::setMode(ControlMode mode) {
  if (active_) return false;  // mode changes only while deactivated
  mode_ = mode;
  return true;
}

bool SimArm::readState(JointState& state) {
  for (int i = 0; i < kCanonicalDof; ++i) {
    const int offset = map_.rokiOffset(i);
    zVecElemNC(state.q.get(), i) = q9_[offset];
    zVecElemNC(state.dq.get(), i) = v9_[offset];
    zVecElemNC(state.tau.get(), i) = zVecElemNC(cmd_.tau.get(), i);
  }
  state.t = time_;
  state.seq = seq_;
  return true;
}

bool SimArm::writeCommand(const JointCommand& cmd) {
  if (cmd.mode != mode_) return false;  // mode is fixed while active
  zVecCopyNC(cmd.q.get(), cmd_.q.get());
  zVecCopyNC(cmd.dq.get(), cmd_.dq.get());
  zVecCopyNC(cmd.tau.get(), cmd_.tau.get());
  cmd_.mode = cmd.mode;
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
        case ControlMode::Velocity:
          tau8[i] = options_.kv * (zVecElemNC(cmd_.dq.get(), i) - dq);
          break;
        case ControlMode::Current:
          tau8[i] = zVecElemNC(cmd_.tau.get(), i);
          break;
      }
    }
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

void SimArm::substep() {
  computeTorques();

  // Forward dynamics: (M + diag(J_reflected)) qdd = tau - bias.
  // Same equation rkChainFD solves, built manually so the reflected
  // motor inertia can be added to the diagonal.
  rkChain* chain = model_.chain();
  rkChainFK(chain, q9_.get());
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
  const double dt = options_.sim_dt;
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
  const int substeps = std::max(
      1, static_cast<int>(std::lround(options_.control_dt / options_.sim_dt)));
  for (int s = 0; s < substeps; ++s) {
    substep();
    for (int i = 0; i < kModelDof; ++i) {
      if (!std::isfinite(q9_[i]) || !std::isfinite(v9_[i])) return false;
    }
  }
  if (log_ != nullptr) {
    log_->frame(options_.control_dt, q9_.get());
  }
  ++seq_;  // one fresh feedback sample per control step
  return true;
}

void SimArm::setDisturbance(int canonical_or_finger_b, double torque) {
  const int index = canonical_or_finger_b < 0 ? kCanonicalDof  // finger_b
                                              : canonical_or_finger_b;
  zVecElemNC(disturbance_.get(), index) = torque;
}

}  // namespace rtctrl::arm
