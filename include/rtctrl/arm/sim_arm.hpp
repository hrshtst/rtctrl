#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rtctrl/arm/arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"
#include "rtctrl/model/zvs_writer.hpp"

namespace rtctrl::arm {

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
  bool readState(JointState& state) override;
  bool writeCommand(const JointCommand& cmd) override;
  bool step() override;

  // Test/diagnostic hooks.
  // Constant external torque on a model joint (by canonical index; -1 =
  // finger_b), e.g. to emulate contact on one finger.
  void setDisturbance(int canonical_or_finger_b, double torque);
  double time() const { return time_; }
  double fingerBDis() const;
  rkChain* chain() const { return model_.chain(); }
  void logTo(model::ZvsWriter* writer) { log_ = writer; }  // per control step

 private:
  void computeTorques();
  void substep();

  Options options_;
  model::ChainModel model_;
  model::JointMap map_;
  bool active_ = false;
  ControlMode mode_ = ControlMode::Position;
  JointCommand cmd_;
  double time_ = 0.0;

  model::ZVector q9_{model::kModelDof};    // committed joint positions
  model::ZVector v9_{model::kModelDof};    // committed joint velocities
  model::ZVector tau9_{model::kModelDof};  // torques applied this substep
  model::ZVector acc9_{model::kModelDof};
  model::ZVector rhs9_{model::kModelDof};
  zMat mass_mat_ = nullptr;  // owned; freed in the destructor
  model::ZVector limit_lo_{model::kModelDof};
  model::ZVector limit_hi_{model::kModelDof};
  model::ZVector effort9_{model::kModelDof};
  model::ZVector damping9_{model::kModelDof};
  model::ZVector disturbance_{model::kCanonicalDof + 1};  // [8] = finger_b
  model::ZvsWriter* log_ = nullptr;
};

}  // namespace rtctrl::arm
