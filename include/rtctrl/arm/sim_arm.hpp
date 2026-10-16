#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rtctrl/arm/arm.hpp"
#include "rtctrl/arm/link_force_interval.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/link_force.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

namespace rtctrl::arm {

class LinkForceCsvLog;

// Simulated Arm: plain roki forward dynamics (rkChainFD) integrated
// with semi-implicit Euler, no contact machinery.
//
// Why not roki-fd: for this contact-free single chain its solver left
// the chain's per-joint dis/vel as intermediate-stage workspace and the
// cell state showed inconsistent velocities on the low-inertia finger
// joints; plain rkChainFD keeps the state committed by construction.
// roki-fd returns when contact-rich scenes (grasping, tables) do.
//
// Current mode feeds commanded torques straight to the joints;
// Position/Velocity modes emulate the servo's own loop with a PD/P law.
// All applied joint torques are clamped to the URDF effort bounds. The
// mimic finger_b is driven by the penalty coupling torque, equal and
// opposite on both fingers, plus half of the commanded gripper effort.
// Joint limits are enforced inelastically (position clamped, velocity
// zeroed at the stop).
//
// JointState::tau reports the servo-equivalent canonical torque BEFORE
// the model-coordinate effort clamp (and without the setDisturbance and
// numeric-damping terms), so under saturation it can differ from the
// effort the forward dynamics used. Only the link-force telemetry below
// is exact about what was applied.
//
// Link-force disturbance (setLinkForce): a prescribed external force on
// one link, added to the forward dynamics as J^T f AFTER the actuator
// effort clamp. The controller never sees it except through motion. The
// force is constant over each substep at its value at the substep start,
// and a substep ends early at the profile's rise end, hold end, or release
// so that every phase is sampled. Once a spec is set, every substep is
// accounted strictly (see LinkForceStatus; finiteness is checked before
// the joint stops); the first rejected substep commits nothing and
// latches the arm: readState and writeCommand then fail, every later
// step() fails without advancing, deactivate() still succeeds, and only
// a fresh SimArm clears the latch. Without a spec the arithmetic path is
// unchanged.
class SimArm : public Arm {
 public:
  struct Options {
    std::string model_path = "models/crane_x7/crane_x7.ztk";
    double sim_dt = 1e-4;      // integrator step [s]
    double control_dt = 0.01;  // one step() advances this much [s]
    // Servo-loop gains. The explicit integrator bounds usable gains by
    // the smallest joint inertia: sqrt(kp/I)*dt and (kd/I)*dt well
    // under ~2. Defaults fit the wrist links (~5e-4 kg·m²) at 1e-4 s.
    double kp = 1000.0;  // position-mode servo stiffness [Nm/rad]
    double kd = 5.0;     // position-mode damping [Nms/rad]
    double kv = 5.0;     // velocity-mode gain [Nms/rad]
    // Finger coupling: overdamped — relative-velocity damping never
    // resists common-mode motion and is what pulls the pair together
    // through the effort clamp.
    double couple_k = 30.0;  // [Nm/rad]
    double couple_c = 0.2;   // [Nms/rad]
    // Reflected motor inertia added to each joint's mass-matrix
    // diagonal: gear^2 * J_rotor, the dominant inertia a geared servo
    // joint actually presents (~350^2 gearing on the XM series). The
    // value is an engineering estimate pending M7 identification; it is
    // what keeps low-inertia links (fingers: ~3e-5 kg·m² bare) from
    // clamp-driven chatter that no real Dynamixel joint exhibits.
    double reflected_inertia = 0.05;  // [kg·m²]
    // Small inertia-scaled viscous term standing in for the real joints'
    // (unmodeled) friction; keeps a fully limp arm from whipping
    // through the joint limits.
    double numeric_damping_ratio = 0.05;
    std::vector<double> initial_q8;  // canonical start pose (default zeros)
    std::vector<double> effort_limit8 = {10.0, 10.0, 4.0, 4.0,
                                         4.0,  4.0,  4.0, 4.0};  // [Nm]
  };

  explicit SimArm(Options options);
  ~SimArm() override;

  SimArm(const SimArm&) = delete;
  SimArm& operator=(const SimArm&) = delete;

  int dof() const override { return model::kCanonicalDof; }
  double dt() const override { return options_.control_dt; }
  bool activate() override;
  bool deactivate() override;
  bool setMode(ControlMode mode) override;
  bool readState(JointState& state,
                 CommandSnapshot* cmds = nullptr) override;
  bool writeCommand(const JointCommand& cmd,
                    CommandReceipt* receipt = nullptr) override;
  bool step() override;

  // Test/diagnostic hooks.
  // Constant external torque on a model joint (by canonical index; -1 =
  // finger_b), e.g. to emulate contact on one finger.
  void setDisturbance(int canonical_or_finger_b, double torque);
  double time() const { return time_; }
  double fingerBDis() const;
  rkChain* chain() const { return model_.chain(); }
  void logTo(model::ZvsWriter* writer) { log_ = writer; }  // per control step

  // Link-force lifecycle. setLinkForce validates the spec and resolves
  // its link (std::invalid_argument otherwise) and enables strict
  // accounting; the force stays zero until startLinkForce starts the
  // profile clock at start_time (sim time; finite; std::logic_error
  // without a spec). Zero requested force (before onset, after release,
  // magnitude 0) skips the direction evaluation entirely, so a
  // degenerate anchor faults only while force is requested.
  // clearLinkForce removes the spec but never clears a fault latch.
  void setLinkForce(const model::LinkForceSpec& spec);
  void startLinkForce(double start_time);
  void clearLinkForce();
  const LinkForceInterval& lastLinkForceInterval() const {
    return lf_interval_;
  }
  bool linkForceFaulted() const { return lf_faulted_; }
  // Whether a completed substep applied nonzero force, and the first and
  // last such substep start times (NaN before any).
  bool linkForceApplied() const { return lf_applied_; }
  double firstLinkForceTime() const { return lf_first_applied_; }
  double lastLinkForceTime() const { return lf_last_applied_; }
  // Writes one LinkForceInterval row at the end of every step() attempt
  // (rejected and latched ones included). A row that cannot be written
  // does not undo or reject the physics: the interval stays as computed
  // and feedback stays valid, but linkForceLogFailed() latches true and
  // that step() and every later one return false.
  void linkForceLogTo(LinkForceCsvLog* log) { lf_log_ = log; }
  bool linkForceLogFailed() const { return lf_log_failed_; }

 private:
  void computeTorques();
  void substep(double dt);
  double shortenToProfileBreakpoint(double dt) const;
  LinkForceStatus substepWithLinkForce(double dt);
  bool stepPlain();
  bool stepWithLinkForce();

  Options options_;
  model::ChainModel model_;
  model::JointMap map_;
  bool active_ = false;
  ControlMode mode_ = ControlMode::Position;
  JointCommand cmd_;
  double time_ = 0.0;
  std::uint64_t seq_ = 0;  // control-step counter -> JointState::seq
  std::uint64_t target_seq_ = 0;         // bumps per accepted command
  AppliedTargetRecord applied_rec_;      // synchronous application
  WriteAttemptRecord attempt_rec_;

  model::ZVector q9_{model::kModelDof};    // committed joint positions
  model::ZVector v9_{model::kModelDof};    // committed joint velocities
  model::ZVector tau9_{model::kModelDof};  // torques applied this substep
  model::ZVector applied_tau8_{model::kCanonicalDof};
  model::ZVector acc9_{model::kModelDof};
  model::ZVector rhs9_{model::kModelDof};
  zMat mass_mat_ = nullptr;  // owned; freed in the destructor
  model::ZVector limit_lo_{model::kModelDof};
  model::ZVector limit_hi_{model::kModelDof};
  model::ZVector effort9_{model::kModelDof};
  model::ZVector damping9_{model::kModelDof};
  model::ZVector disturbance_{model::kCanonicalDof + 1};  // [8] = finger_b
  model::ZvsWriter* log_ = nullptr;

  // Link-force state (see setLinkForce).
  model::LinkForceSpec lf_spec_;
  int lf_link_ = -1;
  bool lf_prepared_ = false;
  bool lf_started_ = false;
  double lf_start_time_ = 0.0;
  bool lf_faulted_ = false;
  bool lf_applied_ = false;
  double lf_first_applied_ = LinkForceInterval::kNaN;
  double lf_last_applied_ = LinkForceInterval::kNaN;
  std::uint64_t lf_attempts_ = 0;
  LinkForceInterval lf_interval_;
  LinkForceCsvLog* lf_log_ = nullptr;
  bool lf_log_failed_ = false;
  model::ZVector ext9_{model::kModelDof};    // J^T f of the current substep
  model::ZVector ext8_{model::kCanonicalDof};
  model::ZVector q_next_{model::kModelDof};  // staged integration
  model::ZVector v_next_{model::kModelDof};
};

}  // namespace rtctrl::arm
