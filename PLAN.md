# PLAN

## Overview

This repository implements a control library of a 7-DOF robot arm
CRANE-X7 with high-level kinematics/dynamics computation.

This directory has already cloned some repositories: some are remained
as dependencies, some are just for your reference so will be deleted.

## Objective

CRANE-X7 <https://rt-net.jp/products/crane-x7/> is a 7DOF robotic arm
produced by RT Corporation. They provide a software to control the
robot, but it is not well structured and robotic computation such as
an inverse kinematics solver has numerical instability. Also, it
doesn't provide current-based torque control, so capabilities of
Dynamixel motors are not fully utilized.

The objective of this project is to implement a control library of
CRANE-X7 capable of robust inverse kinematics, dynamics-based control
using Jacobian, and beyond. @rt_manipulators_cpp/ is software provided by RT Corporation for reference.
This project should cover all features in @rt_manipulators_cpp/ .
Extendability should be considered. High-level robotics computation and
simulation are delegated to mi-lib <https://github.com/mi-lib>.
Communication with motors should be implemented using Dynamixel SDK.
Hence, this project should focus on specific to CRANE-X7.

## Cloned repositories

### Reference

The following repositories are just for your reference, which will be
deleted.
  * @rt_manipulators_cpp/
  * @crane_x7_ros/
  * @mi-lib-tutorial/
@rt_manipulators_cpp/ is reference for features that this project must
have. @crane_x7_ros/ is another reference for how to use the robot model
in URDF and hints for additional features. @mi-lib-tutorial/ is a
tutorial of mi-lib in Japanese. Basic usage of mi-lib should follow
this tutorial.

### Dependencies

The following repositories are dependencies of this project. Other
dependencies can be added if needed.
  * @third_party/crane_x7_description/
  * @third_party/DynamixelSDK/
  * @third_party/zeda/
  * @third_party/zm/
  * @third_party/zeo/
  * @third_party/roki/
  * @third_party/roki-fd/
  * @third_party/zx11/
  * @third_party/roki-gl/
  * @third_party/dzco/
  * @third_party/liw/
@third_party/crane_x7_description/ has a CRANE-X7 model in URDF, which
should be ported to a model file in mi-lib. @third_party/DynamixelSDK/
is SDK provided by the motor manufacturer. Installed motors in
CRANE-X7 are listed in
<https://docs.google.com/spreadsheets/d/1aDFqHm1ITyZzfuScvNEFtuxv-zPM3NvJoxa1wEGHN7g/edit?gid=1289058805#gid=1289058805>.

Other repositories are developed in Motor Intelligence Lab
<https://github.com/mi-lib>. I am used to using and developing
robotics software with mi-lib, so robotics computation should be
implemented with this library. This project should handle CRANE-X7
model for mi-lib. Later, we will research and develop a robot
controller. Hence, this project should make a bridge between the
developed controller and mi-lib simulator, so that we can verify if
the developed controller works well in the simulator before execution
on the real robot.


## What I want you to plan

At least, I want you to plan the followings.
  * Project structure (directories)
  * Testing framework
  * Verification steps on simulator side
    * Port URDF to .ztk file
    * Load it in rk_pen
    * Make a simple motion and visualize it in rk_anim
    * Implement a dynamics simulation using roki-fd
  * Verification steps on hardware side
    * Implement all features in `rt_manipulators_cpp`
    * Inspection tool for motor parameters
    * Implement a mock (emulator) of a motor to allow test driven
      development
    * Testing steps on hardware
      * ON/OFF
      * Parameter modification
      * Read sensory data
      * Make a simple motion
    * Sub-goals
      * Position based control
      * Current based torque estimation
      * Torque control


I treated `PLAN.md` as the source specification. The plan is well structured, but several issues should be resolved before implementation.

### Findings

- **[P0] USB-loss safety cannot work as described.** The plan says the communication watchdog will “zero & torque-off” and that USB unplugging will test this behavior ([IMPLEMENTATION_PLAN.md:183](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:183), [IMPLEMENTATION_PLAN.md:206](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:206)). Once USB communication is lost, the host cannot transmit either command. Configure the servo-side Bus Watchdog—available at register 98 ([xm430_w350.model:51](/home/atsuta/develop/rtctrl/third_party/DynamixelSDK/control_table/xm430_w350.model:51))—and test command-stream loss. A real emergency stop must independently remove actuator power; `deactivate()` is not an e-stop.

- **[P1] The proposed gravity calculation dereferences null vectors.** `rkChainID_G(chain, q, 0, 0, ...)` in [IMPLEMENTATION_PLAN.md:201](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:201) is invalid for this roki version. `rkChainID_G` calls `rkChainSetJointRateAll`, which unconditionally indexes both vectors ([rk_chain.c:478](/home/atsuta/develop/rtctrl/third_party/roki/src/rk_chain.c:478), [rk_chain.c:334](/home/atsuta/develop/rtctrl/third_party/roki/src/rk_chain.c:334)). M7 should require correctly sized zero velocity and acceleration vectors and include a regression test.

- **[P1] Controller, actuator, and model dimensions are undefined.** The system has seven arm joints, eight servos including the gripper, and nine revolute model joints including the mimic finger. Yet `Arm::dof()`, `JointState`, and `JointCommand` expose only undifferentiated vectors ([IMPLEMENTATION_PLAN.md:64](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:64)). Define a canonical joint order and explicit mappings between controller coordinates, hardware actuators, and roki generalized coordinates before M2/M3.

- **[P1] Mirroring the mimic finger in software is not a valid dynamics constraint.** Keeping `finger_b` as an independent revolute DOF and merely driving or copying it ([IMPLEMENTATION_PLAN.md:97](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:97)) allows the two fingers to diverge under contact because they have different inertias and collision forces. The plan needs a real coupling strategy—constraint enforcement, state projection with documented nonphysical effects, or a reduced-coordinate gripper model—and an acceptance test under asymmetric contact.

- **[P1] M0 is not reproducible from a clean clone.** Its acceptance command assumes mi-lib has already been installed under `$HOME/usr`, while no bootstrap or prerequisite verification is included ([IMPLEMENTATION_PLAN.md:103](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:103), [IMPLEMENTATION_PLAN.md:145](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:145)). Catch2 and TOML dependencies also have no pinned versions, and the TOML library remains undecided. Select and pin exact revisions, document/install the mi-lib submodules in dependency order, and exercise M0 in a clean CI environment.

- **[P2] The robust-IK acceptance criterion is too weak.** “No NaN” for unreachable targets ([IMPLEMENTATION_PLAN.md:160](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:160)) can still accept a finite but meaningless result. `IkSolver` should return a structured status based on final position/orientation residuals, iteration count, and joint-limit satisfaction. Tests should require success for reachable poses and explicit non-convergence for unreachable ones.

- **[P2] Model-port fidelity omits limits that the converter explicitly drops.** roki ignores URDF effort and velocity limits ([rk_chain_urdf.c:398](/home/atsuta/develop/rtctrl/third_party/roki/src/rk_chain_urdf.c:398)), but M1’s patch only discusses motors and friction ([IMPLEMENTATION_PLAN.md:149](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:149)). Add per-joint velocity and effort/torque limits to the patch or runtime model and verify them. Also clarify the friction source: the cited CSV contains inertia, limits, motor constants, and gains—not friction parameters ([manipulator_links.csv:1](/home/atsuta/develop/rtctrl/crane_x7_ros/crane_x7_control/config/manipulator_links.csv:1)).

Overall, the milestone ordering and simulator/hardware bridge are sound. I would approve the plan after correcting the two safety/API defects and specifying coordinate mapping, dependency reproducibility, and mimic-joint behavior.

The update plausibly resolves most earlier findings, but four issues remain.

### Findings

- **[P1] Bus Watchdog semantics are still modeled incorrectly.** The plan triggers it on “command-stream silence” ([IMPLEMENTATION_PLAN.md:173](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:173)), but the servo monitors all instruction packets. Continuing sync reads while writes/controller updates stall keeps the watchdog alive. The emulator must timestamp every instruction, while the host watchdog separately detects stale commands even when reads succeed. Activation should also reject firmware older than v38, where Bus Watchdog is unavailable. Both behaviors are specified in the [official XM430 manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#bus-watchdog98).

- **[P1] Gravity torque still has an 8-vs-9 dimension contradiction.** Canonical `q` is always eight elements ([IMPLEMENTATION_PLAN.md:109](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:109)), but the proposed call passes it directly to a nine-joint roki chain ([IMPLEMENTATION_PLAN.md:254](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:254)). `gravityTorque` must explicitly expand canonical coordinates to nine model coordinates, call roki, then reduce the nine generalized torques back to eight. For the mimic coordinate, reduction should use the constraint Jacobian—normally summing the two finger generalized torques—not merely returning `finger_a`.

- **[P2] Reachable singular poses should not automatically fail IK.** The updated acceptance test groups singular targets with unreachable targets and expects non-convergence ([IMPLEMENTATION_PLAN.md:204](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:204)). A reachable pose at or near a singular configuration is precisely where the damped solver should demonstrate robustness. Require convergence for selected reachable singular/near-singular poses; reserve explicit failure for unreachable or incompatible constraints.

- **[P2] The friction-source statement is factually wrong.** M1 now says the URDF carries no friction parameters ([IMPLEMENTATION_PLAN.md:190](/home/atsuta/develop/rtctrl/docs/IMPLEMENTATION_PLAN.md:190)), but the arm URDF contains damping and friction values ([crane_x7_arm.xacro:110](/home/atsuta/develop/rtctrl/third_party/crane_x7_description/urdf/crane_x7_arm.xacro:110)) and the gripper does too. Starting with zero friction pending identification is reasonable; say that the converter drops or does not preserve those nominal values, rather than claiming they do not exist.

The previous clean-build, null-vector, coordinate-order, model-limit, emergency-stop, and mimic-dynamics concerns are otherwise plausibly addressed. The penalty coupling is acceptable for a first simulation model, though implementation should state explicitly that its constraint torques are equal and opposite. No files were changed.
