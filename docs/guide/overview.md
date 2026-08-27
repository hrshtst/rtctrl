# Overview and philosophy

**rtctrl** is a C++17 control library for the
[CRANE-X7](https://rt-net.jp/products/crane-x7/), a 7-DOF arm with a
two-finger gripper built from Dynamixel XM servos. It provides robust
inverse kinematics, dynamics-based control (gravity compensation and
computed-torque tracking), and a simulator bridge, so the same
controller code runs in simulation and on the real arm.

## Design philosophy

**Thin, robot-specific glue.** rtctrl contains as little robotics
code as possible. The general math (kinematics, Jacobians, inverse
dynamics, forward-dynamics simulation) comes from
[mi-lib](https://github.com/mi-lib); all servo communication goes
through [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK).
What lives here is only what is specific to the CRANE-X7: its model,
its coordinate conventions, its safety envelope, and the seams that
join the pieces.

**Verify in simulation before touching hardware.** Controllers are
written once against a single interface, the `Arm` bridge, and run
unchanged on the physics simulator (`SimArm`) and on the real robot
(`RealArm`). Every dynamics controller in this repository passed its
acceptance test in simulation before its first hardware run. A
simulation pass is necessary, never sufficient: the simulated joints
are rigid while the real gearboxes flex, so some failures only ever
appear on the arm (see
[what the hardware taught us](../theory/computed-torque.md#what-the-hardware-taught-us)).

**Testable without a robot, down to the wire.** A motor emulator
behaves like the arm's servos, in two forms: an in-process fake for
fast unit tests, and a virtual serial port (a pseudo-terminal
speaking the servos' real byte protocol) for integration tests. CI
therefore exercises the same *unmodified* DynamixelSDK code path
that talks to hardware, and every hardware app can be rehearsed
risk-free by pointing it at the emulator (`--port /tmp/ttyDXL`).

**Safety is layered and assumes failure.** Torque can only come on
through an activation sequence: it checks every servo's identity and
firmware, aligns all goals with the arm's present posture so that
activation itself never commands motion (position mode then holds
the pose; current mode starts at zero current, so the app must
support the arm immediately or stage a gravity preload), and arms a
watchdog inside each servo that halts it when the bus goes silent.
Because ordinary reads keep that watchdog fed, the host adds its own
deadman timer on the command stream: if commands stall, it silences
the bus completely, which is precisely the condition that forces the
servo watchdogs to stop the arm. None of this replaces the physical
power cutoff, which must stay in reach during hardware sessions.

**Explicit coordinates.** The same arm counts differently depending
on where you look: seven arm joints, eight servos (the gripper adds
one), and nine joints in the dynamics model (the gripper's two
fingers mirror each other). The library never lets these blur: one
canonical 8-DOF order is fixed project-wide, and `JointMap` owns
every mapping between the three, including how the mimic finger's
torque folds back onto its servo (see
[dynamics foundations](../theory/dynamics-foundations.md)).

## What works where

Infrastructure (verified on the physical arm): the URDF-derived
mi-lib model, robust IK (`IkSolver`), the deterministic `SimArm`
dynamics simulator with reflected motor inertia, the Dynamixel comm
layer with its wire-level emulator, and the `CraneX7`/`RealArm`
hardware layer with layered watchdog safety.

Control, by evidence tier (the honest ledger of this project):

| Capability | Theory | Simulation | Hardware |
|---|---|---|---|
| Gravity compensation (`x7_float`) | [notes](../theory/gravity-compensation.md) | ✓ sim float | **✓ WORKS** — requires `config/crane_x7_vendor_scale.toml` (gate before bus contact) and a fresh `--log`; the failed subjective back-drive criterion was owner-waived ([history](../records/history.md#gate-outcomes-and-the-waived-back-drive-criterion)) |
| Manual motion teaching (`x7_teach`) | passive acquisition or gravity compensation | ✓ wire-level emulator | **⚠ NOT YET ACCEPTED**: torque-off and gravity-compensated recording, raw logs, resampled ZVS output, and portable bundles are implemented; first physical-arm use remains a new bring-up step ([usage](usage.md#manual-motion-teaching)) |
| Position-mode trajectory tracking (`x7_wave`, `x7_move_simple`, `x7_pose`) | — (servo-internal loops) | ✓ emulator | **✓ WORKS** — the M6 parity milestone and the supported route for large fast motions; bring-up moves report dropped writes, `x7_move_simple` verifies its return, and `x7_pose` requires measured convergence ([history](../records/history.md#position-mode-tracking-on-hardware)) |
| Servo-side trajectory following (`x7_follow`, sim twin `x7_follow_sim`) | servo-internal loops | ✓ full phase simulation | **⚠ NOT YET ACCEPTED**: position and current-based-position frontends are implemented with preflight, home gate, tracking-error abort, final support hold, and watchdog timing, but the physical-arm acceptance run is pending ([usage](usage.md#servo-side-trajectory-following)) |
| Computed-torque tracking (`x7_track`, sim twin `x7_track_sim`) | [notes](../theory/computed-torque.md) | ✓ RMS 0.005 rad, 3.1× bare PD | **⚠ CAPPED & PARKED** — accepted at RMS ≈ 0.02 rad only within scale ≤ 0.6 (**cap FINAL**, [history](../records/history.md#hardware-campaign-and-the-final-06-scale-cap)); the app is parked behind an unimplemented settle-phase fix ([status](../hardware/bringup.md)) |
| Exact feedback linearization (`x7_efl_study`) | [notes](../theory/computed-torque.md) | **✗ CLOSED-NEGATIVE** at the preregistered gain gate ([history](../records/history.md#results-and-interpretation)) | never operated (offline-only by charter) |
| Flexible-mode identification (`x7_ident_sim` + `tools/ident_analysis.py`) | [notes](../theory/identification.md) | ✓ method validated (planted modes recovered) | **✗ CLOSED-NULL** — joint 1 at P1 stiction-locked below encoder resolution; hardware app removed 2026-08-18 ([history](../records/history.md#closure-decision-and-precise-scope)) |

The consolidated record of every decision and hardware finding is
[history.md](../records/history.md).

## Reading order

1. [Getting started](getting-started.md): build it, run the emulator
   demo, see the model.
2. [Architecture](architecture.md): layers, coordinates, the bridge,
   safety.
3. [Usage](usage.md): the library as an API, with code.
4. Theory: [foundations](../theory/dynamics-foundations.md) ·
   [gravity compensation](../theory/gravity-compensation.md) ·
   [computed torque](../theory/computed-torque.md).
5. Hardware: [bring-up checklist](../hardware/bringup.md).

## Acknowledgments

The CRANE-X7 and its official control software,
[`rt_manipulators_cpp`](https://github.com/rt-net/rt_manipulators_cpp),
are products of [RT Corporation](https://rt-net.jp/). The official
software was the functional reference for this library; the
[parity checklist](../records/parity.md) records how its capabilities
map to rtctrl equivalents, including the vendor surface rtctrl
consciously simplifies.
