# Overview and philosophy

**rtctrl** is a C++17 control library for the
[CRANE-X7](https://rt-net.jp/products/crane-x7/), a 7-DOF arm with a
two-finger gripper built from Dynamixel XM servos. It replaces the
vendor software with a structured stack offering robust inverse
kinematics, dynamics-based control (gravity compensation,
computed-torque tracking), and a simulator bridge — while functionally
covering the capabilities of the original `rt_manipulators_cpp` for
whole-arm control (see the [parity checklist](../PARITY.md), including
its list of consciously simplified vendor surface).

## Design philosophy

**Thin, robot-specific glue.** All generic robotics computation —
kinematics, Jacobians, inverse dynamics, forward-dynamics simulation —
is delegated to [mi-lib](https://github.com/mi-lib) (roki and
friends); all wire communication to the vendor's DynamixelSDK. rtctrl
contains only what is specific to the CRANE-X7: its model, its
coordinate conventions, its safety envelope, and the seams that join
the pieces.

**Verify in simulation before touching hardware.** The central
abstraction is the `Arm` bridge: a controller written once against it
runs unchanged on the roki-dynamics simulator (`SimArm`) and on the
real robot (`RealArm`). Every dynamics controller in this repository
passed its acceptance test in simulation before its first hardware
run. Simulation is necessary, not sufficient: the rigid-joint sim
cannot certify gains against the real arm's gear elasticity — see
[what the hardware taught us](../theory/computed-torque.md#what-the-hardware-taught-us).

**Testable without a robot, down to the wire.** A motor emulator
implements the XM control-table state machine twice over: as an
in-process fake behind the `PacketIO` seam (fast unit tests) and as a
pseudo-terminal speaking wire-level Protocol 2.0, so the *unmodified*
DynamixelSDK — the exact bytes-on-the-wire path — is exercised in CI.
Every hardware app runs identically against the emulator
(`--port /tmp/ttyDXL`) and the robot.

**Safety is layered and assumes failure.** Torque can only be enabled
through an activation sequence that verifies servo identity and
firmware, snaps goals to the present posture (no motion on power-up),
and arms the servo-side Bus Watchdog. A host-side deadman
independently watches the *command* stream — because reads alone keep
the servo watchdog fed — and escalates to full bus silence, which
provably forces the servos to halt themselves. None of this replaces
the physical power cutoff, which must stay in reach during hardware
sessions.

**Explicit coordinates.** Seven arm joints, eight servos, nine model
joints: the library never lets these dimensions blur. A single
canonical 8-DOF order is fixed project-wide, and `JointMap` owns the
mappings — including the virtual-work torque reduction for the mimic
finger (see [dynamics foundations](../theory/dynamics-foundations.md)).

## What exists today

| Area | State |
|---|---|
| CRANE-X7 model for mi-lib (`models/crane_x7/crane_x7.ztk`) | generated from the URDF, verified against it numerically and in `rk_pen` |
| Robust IK (`IkSolver`) | error-damped LM, structured results; converges at reachable singular poses, reports unreachable ones explicitly |
| Dynamics simulator (`SimArm`) | plain-roki FD with reflected motor inertia; deterministic |
| Motor comm layer + emulator | full XM register map, indirect-address sync IO, wire-level emulator |
| Hardware layer (`CraneX7`, `RealArm`) | vendor parity + background RW thread + layered watchdog safety (servo bus watchdog; host deadman on stale commands or frozen feedback), verified on the physical arm |
| Gravity compensation | proven by gradient tests, sim float, and hardware float |
| Computed-torque tracking | sim RMS 0.005 rad (3× better than bare PD); hardware-accepted at RMS ≈ 0.02 rad within the reduced-speed envelope — see the [theory notes](../theory/computed-torque.md) for the hardware-hardened law and the scale cap |

The development history, including every design decision and hardware
finding, lives in the [implementation plan](../IMPLEMENTATION_PLAN.md);
the original specification is [PLAN.md](../PLAN.md).

## Reading order

1. [Getting started](getting-started.md) — build it, run the emulator
   demo, see the model.
2. [Architecture](architecture.md) — layers, coordinates, the bridge,
   safety.
3. [Usage](usage.md) — the library as an API, with code.
4. Theory: [foundations](../theory/dynamics-foundations.md) ·
   [gravity compensation](../theory/gravity-compensation.md) ·
   [computed torque](../theory/computed-torque.md).
5. Hardware: [bring-up checklist](../HARDWARE_BRINGUP.md).
