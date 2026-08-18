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
firmware, snaps goals to the present values (no motion is commanded;
position mode holds the posture, while current-mode activation is
zero-current — the app must support the arm immediately or stage a
gravity preload), and arms the servo-side Bus Watchdog. A host-side deadman
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

## What works where

Infrastructure (verified on the physical arm): the URDF-derived
mi-lib model, robust IK (`IkSolver`), the deterministic `SimArm`
dynamics simulator with reflected motor inertia, the Dynamixel comm
layer with its wire-level emulator, and the `CraneX7`/`RealArm`
hardware layer with layered watchdog safety.

Control, by evidence tier — the honest ledger of this project:

| Capability | Theory | Simulation | Hardware |
|---|---|---|---|
| Gravity compensation (`x7_float`) | [notes](../theory/gravity-compensation.md) | ✓ sim float | **✓ WORKS** — requires `config/crane_x7_vendor_scale.toml` (gate before bus contact) and a fresh `--log`; the failed subjective back-drive criterion was owner-waived ([history](../HISTORY.md#gate-outcomes-and-the-waived-back-drive-criterion)) |
| Position-mode trajectory tracking (`x7_wave`, `x7_move_simple`, `x7_pose`) | — (servo-internal loops) | ✓ emulator | **✓ WORKS** — the M6 parity milestone and the supported route for large fast motions ([history](../HISTORY.md#position-mode-tracking-on-hardware)) |
| Computed-torque tracking (`x7_track`, sim twin `x7_track_sim`) | [notes](../theory/computed-torque.md) | ✓ RMS 0.005 rad, 3.1× bare PD | **⚠ CAPPED & PARKED** — accepted at RMS ≈ 0.02 rad only within scale ≤ 0.6 (**cap FINAL**, [history](../HISTORY.md#hardware-campaign-and-the-final-06-scale-cap)); the app is parked behind an unimplemented settle-phase fix ([status](../HARDWARE_BRINGUP.md)) |
| Exact feedback linearization (`x7_efl_study`) | [notes](../theory/computed-torque.md) | **✗ CLOSED-NEGATIVE** at the preregistered gain gate ([history](../HISTORY.md#results-and-interpretation)) | never operated (offline-only by charter) |
| Flexible-mode identification (`x7_ident_sim` + `tools/ident_analysis.py`) | [notes](../theory/identification.md) | ✓ method validated (planted modes recovered) | **✗ CLOSED-NULL** — joint 1 at P1 stiction-locked below encoder resolution; hardware app removed 2026-08-18 ([history](../HISTORY.md#closure-decision-and-precise-scope)) |

The consolidated record of every decision and hardware finding is
[HISTORY.md](../HISTORY.md).

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
