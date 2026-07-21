# Architecture

## Layers

```
        controllers (yours)
              │ Arm / Controller / Runner
   ┌──────────┴──────────┐
   │      arm/  L3       │   the sim⇄real BRIDGE
   │  SimArm    RealArm  │
   └───┬──────────┬──────┘
       │          │
 model/ L2      hw/ L4          emu/  (test double)
 roki wrappers  CraneX7,        MotorEmulator
 ChainModel,    safety,   ──►   ├─ FakePacketIO (in-process)
 JointMap, IK,  config          └─ PtyBus (wire-level Protocol 2.0)
 trajectories      │                     ▲
       │        dxl/ L1  ────────────────┘
   mi-lib       PacketIO seam, Port(SDK), SyncGroup,
 (roki, zm,…)   control table, conversions
```

- **`dxl/` — motor communication.** `PacketIO` is the abstract
  transaction seam (ping/read/write/syncRead/syncWrite). `Port`
  implements it over DynamixelSDK; the emulator implements it twice
  for tests. `SyncGroup` programs the servos' indirect-address banks
  so one sync-read fetches position/velocity/current/voltage/
  temperature for the whole group and one sync-write sets the goals.
  `control_table.hpp`/`conversions.hpp` fix the XM register map and
  raw-unit conversions (incl. per-model torque constants).
- **`model/` — kinematics and dynamics.** RAII wrappers over mi-lib's
  C API: `ChainModel` (load `.ztk`, FK, gravity torque, inverse
  dynamics), `JointMap` (coordinate/force mappings), `IkSolver`
  (error-damped LM with structured `IkResult`), trajectories, `.zvs`
  motion logging for `rk_anim`.
- **`arm/` — the bridge.** `Arm` (activate/deactivate/setMode/
  readState/writeCommand/step), `Controller` + `run()`, the two
  implementations `SimArm`/`RealArm`, and the shipped controllers
  `GravityComp` and `ComputedTorque`.
- **`hw/` — the real robot.** `Config` (TOML), `CraneX7` (activation
  sequence, mode-checked limited writers, background RW thread,
  deadman), consumed by `RealArm`.
- **`emu/` — the test double.** One control-table state machine, two
  transports. Not linked into controllers; linked into tests and the
  standalone `dxl_emu`.

Dependency rule: `dxl` and `model` are independent of each other;
`arm` depends on `model`; `hw` depends on `dxl` + `model` + `arm`;
`emu` depends only on `dxl` types. Controllers depend on `arm` (and
`model` for dynamics) — never on `dxl`/`hw` directly.

## Directory map

```
include/rtctrl/{dxl,model,arm,hw,emu}/   public headers (src/ mirrors)
apps/         dxl_inspect, dxl_emu, x7_onoff/read/set_param/move_simple/float/track/track_sim
examples/     make_motion (kinematic .zvs), x7_wave (bridge demo)
models/crane_x7/   crane_x7.ztk + meshes + contactinfo.ztk  (generated, committed)
config/crane_x7.toml   deployment config: bus, joints, limits, margins
tools/        port_model.py (uv), bootstrap_milib.sh, milib_config/
tests/{unit,integration}/   Catch2; integration = pty emulator + sim
docs/         this documentation
third_party/  pinned submodules (mi-lib stack, DynamixelSDK, URDF source)
```

## Canonical coordinates

One joint order is fixed project-wide (`model::canonicalJoints()`):

| canonical | joint | DXL id | servo |
|---|---|---|---|
| 0 | shoulder pan | 2 | XM430-W350 |
| 1 | shoulder tilt | 3 | XM540-W270 |
| 2 | upper-arm twist | 4 | XM430-W350 |
| 3 | elbow | 5 | XM430-W350 |
| 4 | forearm twist | 6 | XM430-W350 |
| 5 | wrist pitch | 7 | XM430-W350 |
| 6 | wrist rotate | 8 | XM430-W350 |
| 7 | gripper (finger A) | 9 | XM430-W350 |

The roki model has a ninth joint (finger B, mimicking A).
`JointMap` resolves model indices **by link name at load time** and
provides `expand` ($q_9 = E q_8$), `reduce`, and `reduceTorque`
($\tau_8 = E^\mathsf{T}\tau_9$) — see
[dynamics foundations](../theory/dynamics-foundations.md). Every
`JointState`/`JointCommand` on the bridge is canonical 8-DOF; only
`SimArm`/`RealArm` translate at their boundaries.

## The bridge contract

```cpp
class Arm {
  int dof();                    // 8
  double dt();                  // control period
  bool activate();              // torque on, safety armed, NO motion
  bool deactivate();            // gentle release — NOT an e-stop
  bool setMode(ControlMode);    // Position=3 / Velocity=1 / Current=0
  bool readState(JointState&);  // q, dq, tau (est.), t
  bool writeCommand(const JointCommand&);
  bool step();                  // sim: integrate dt; real: wait cycle
};
```

One cycle is `readState → Controller::update → writeCommand → step`
(`arm::run`). `SimArm` integrates plain-roki forward dynamics,
$(M + \mathrm{diag}\,J_r)\,\ddot q = \tau - b$, with reflected motor
inertia $J_r$, semi-implicit Euler, inelastic joint stops, and an
overdamped equal-and-opposite finger coupling. `RealArm` rides `CraneX7`'s
background thread: `step()` blocks on the next completed
read→limit→write cycle.

## Safety architecture (hardware)

Two independent watchdog layers, because each alone has a blind spot:

1. **Servo-side Bus Watchdog** (register 98, armed at activation,
   firmware ≥ v38 enforced): the servo freezes itself — motion halted,
   goal writes locked out, **torque stays on** — when no instruction
   packet arrives in time. Covers host death and cable loss. Blind
   spot: *any* packet feeds it, so a host whose reads continue while
   command writes stall never trips it.
2. **Host-side deadman**: tracks the time since the last *successful
   command write*; on staleness it best-effort zeroes/torque-offs and
   then **silences the bus entirely** (closes the port) — which
   guarantees layer 1 fires even if the safety writes themselves were
   failing. The same escalation fires after a few consecutive *failed
   feedback reads*: frozen feedback would otherwise leave the
   controller commanding into a robot it can no longer see, while
   broadcast sync writes keep "succeeding" and never trip the
   staleness check (`CycleStats::read_failures` counts these).

The activation sequence additionally verifies servo models and
firmware, clears previously tripped watchdogs, snaps goals to the
present posture, and applies soft-start gains. Command writers clamp
against position/velocity/effort limits and reject mode-mismatched or
feedback-less commands. All of it is asserted in tests whose
acceptance criterion is the *emulated motor's end state*, and was
drilled on the physical arm (USB pull → freeze within 100 ms).

`deactivate()` is not an emergency stop. Hardware sessions require an
independent actuator-power cutoff within reach
([bring-up checklist](../HARDWARE_BRINGUP.md)).

## Verification tiers

| Tier | Command | Covers |
|---|---|---|
| unit | `ctest -L unit` | pure logic + in-process emulator |
| integration | `ctest -L integration` | wire-level emulator through the real SDK; roki dynamics sim |
| hardware | staged apps | the physical checklist + controller phases |

CI runs the first two from a clean clone, including the mi-lib
bootstrap.
