// Reusable sim-loop lag model (extracted from x7_track_sim): wraps an
// ideal Arm with the real control loop's command pipeline latency,
// position readout quantized to the encoder LSB, and a servo-style
// velocity estimate (first-order lag + one-LSB quantization) in
// state.dq.
//
// Two startup semantics, both pinned by tests/unit/lagged_arm_test.cpp:
//  * legacy (default; the x7_track_sim behavior): the FIRST command
//    passes through immediately AND becomes the pending command, so it
//    is applied twice; every later command is applied one cycle after
//    acceptance.
//  * preloaded (first_passthrough = false; the EFL study's
//    preregistered rule): the queue begins FULL of `delay_cycles`
//    zero-current commands, so the command accepted at cycle k is
//    applied at cycle k + delay_cycles from the first cycle on
//    (docs/HISTORY.md (EFL frozen specification), preregistered constants).
#pragma once

#include <cmath>
#include <cstdint>
#include <deque>

#include "rtctrl/arm/arm.hpp"

namespace x7 {

namespace arm = rtctrl::arm;
namespace model = rtctrl::model;

struct LaggedArmOptions {
  int delay_cycles = 1;           // pipeline depth (>= 1)
  bool first_passthrough = true;  // legacy startup; false = zero-preload
  bool exact_velocity = false;    // pass state.dq through unfiltered
  bool quantize_pos = true;       // round positions to the encoder LSB
};

struct LaggedArm : arm::Arm {
  static constexpr double kVelLsb = 0.229 * 2.0 * M_PI / 60.0;  // [rad/s]
  static constexpr double kPosLsb = 2.0 * M_PI / 4096.0;        // [rad]

  LaggedArm(arm::Arm& inner, double vel_tau, LaggedArmOptions options = {})
      : inner_(inner), vel_tau_(vel_tau), opt_(options) {
    if (!opt_.first_passthrough) {
      arm::JointCommand zero;
      zero.mode = arm::ControlMode::Current;
      for (int k = 0; k < opt_.delay_cycles; ++k) {
        pending_.push_back({zero, 0});  // preload seq 0: never accepted
      }
    }
  }

  int dof() const override { return inner_.dof(); }
  double dt() const override { return inner_.dt(); }
  bool activate() override { return inner_.activate(); }
  bool deactivate() override { return inner_.deactivate(); }
  bool setMode(arm::ControlMode mode) override {
    return inner_.setMode(mode);
  }

  bool readState(arm::JointState& state,
                 arm::CommandSnapshot* cmds = nullptr) override {
    if (!inner_.readState(state)) return false;
    last_time_ = state.t;
    const double alpha = dt() / (vel_tau_ + dt());
    for (int i = 0; i < model::kCanonicalDof; ++i) {
      if (!opt_.exact_velocity) {
        dq_filt_[i] +=
            alpha * (zVecElemNC(state.dq.get(), i) - dq_filt_[i]);
        zVecElemNC(state.dq.get(), i) =
            std::round(dq_filt_[i] / kVelLsb) * kVelLsb;
      }
      if (opt_.quantize_pos) {
        zVecElemNC(state.q.get(), i) =
            std::round(zVecElemNC(state.q.get(), i) / kPosLsb) * kPosLsb;
      }
    }
    // The wrapper owns the command records: its lag makes application
    // ASYNCHRONOUS, so the inner sim's synchronous records would
    // misattribute the current request to the previous command's
    // sequence.
    if (cmds != nullptr) {
      cmds->applied = applied_rec_;
      cmds->last_attempt = attempt_rec_;
    }
    return true;
  }

  bool writeCommand(const arm::JointCommand& cmd,
                    arm::CommandReceipt* receipt = nullptr) override {
    const std::uint64_t this_seq = ++wrapper_seq_;
    const double now = last_time_;
    bool ok = true;
    if (opt_.first_passthrough && !primed_) {
      // first cycle passes through, as activation snaps
      primed_ = true;
      ok = inner_.writeCommand(cmd);
      if (ok) adoptInnerRecords(this_seq, now);
      pending_.clear();
    } else {
      // the queue front (accepted delay_cycles ago) reaches the
      // actuator now — ITS sequence becomes the applied sequence
      ok = inner_.writeCommand(pending_.front().cmd);
      if (ok) adoptInnerRecords(pending_.front().seq, now);
      pending_.pop_front();
    }
    pending_.push_back({cmd, this_seq});
    if (receipt != nullptr) *receipt = {ok, this_seq, now};
    return ok;
  }

  bool step() override { return inner_.step(); }

 private:
  struct Pending {
    arm::JointCommand cmd;
    std::uint64_t seq;
  };

  // Take the inner sim's freshly synthesized records (they carry the
  // clamp truth) but stamp them with the WRAPPER's sequence.
  void adoptInnerRecords(std::uint64_t wrapper_seq, double now) {
    arm::JointState dummy;
    arm::CommandSnapshot snap;
    inner_.readState(dummy, &snap);
    applied_rec_ = snap.applied;
    applied_rec_.target_seq = wrapper_seq;
    applied_rec_.first_time = applied_rec_.latest_time = now;
    attempt_rec_ = snap.last_attempt;
    attempt_rec_.target_seq = wrapper_seq;
    attempt_rec_.time = now;
  }

  arm::Arm& inner_;
  double vel_tau_;
  LaggedArmOptions opt_;
  std::deque<Pending> pending_;
  std::uint64_t wrapper_seq_ = 0;
  bool primed_ = false;
  double last_time_ = 0.0;
  arm::AppliedTargetRecord applied_rec_;
  arm::WriteAttemptRecord attempt_rec_;
  double dq_filt_[model::kCanonicalDof] = {};
};

}  // namespace x7
