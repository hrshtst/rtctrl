# CRANE-X7 Control Library (rtctrl) — Project Plan

## Context

This repo will host a new control library for the CRANE-X7 7-DOF arm (RT Corporation).
The vendor lib `rt_manipulators_cpp` is poorly structured, its IK is numerically unstable,
and it offers no current-based torque control. The new library covers all vendor features,
delegates kinematics/dynamics/simulation to mi-lib (roki/roki-fd), uses DynamixelSDK for
motor comms, and provides a **bridge** so a controller runs unchanged against the roki-fd
simulator and the real robot. Reference repos (`rt_manipulators_cpp/`, `crane_x7_ros/`,
`mi-lib-tutorial/`) will be deleted later; `third_party/*` submodules stay.

**User decisions (2026-07-17):** C++17, CMake, Catch2. mi-lib's zeda-makefile-gen/autotest
explicitly rejected as too complicated. Config files in **TOML** (not YAML). Third-party
retrieval: submodules and FetchContent **combined** by rule (see Build design). Python
utilities run with **uv** and carry a `pyproject.toml`.

**Key facts verified during exploration:**
- roki ships a URDF converter: `urdf2ztk` CLI (`third_party/roki/app/urdf2ztk.c`) and
  `rkChainReadURDF`/`rkURDFWriteZTK` (`rk_chain_urdf.h`); roki built with `CONFIG_USE_URDF=y`.
  Mesh `import` paths pass through verbatim (`rk_chain_urdf.c:644-651`) and zeo opens them
  CWD-relative with `fopen`; `package://` URIs must be rewritten. zeo reads ASCII+binary STL
  (`zeo/include/zeo/zeo_ph3d_stl.h`).
- mi-lib is installed under `$HOME/usr` with `<lib>-config` scripts (no CMake/pkg-config);
  `urdf2ztk`, `rk_pen`, `rk_anim` are in `~/usr/bin`.
- DynamixelSDK `c++/CMakeLists.txt` is a modern package: target **`dynamixel_sdk`** (shared,
  C++17, PIC), `install(EXPORT dynamixel_sdkTargets)` — usable via `add_subdirectory`.
  Its `dynamixel_easy_sdk` has NO indirect-address support → build on the low-level API
  (PortHandler/PacketHandler 2.0/GroupSync(Fast)Read/Write).
- `PortHandlerLinux` supports 3 Mbps via standard `B3000000` termios constant → a pty-based
  bus emulator works with the unmodified SDK.
- XM servos have a servo-side **Bus Watchdog** (register 98, 1 byte, 20 ms units; requires
  firmware ≥ v38; `third_party/DynamixelSDK/control_table/xm430_w350.model:51` + official
  XM430 e-manual): it monitors ALL instruction packets — if none arrives within the timeout
  the servo stops itself and rejects goal writes until cleared. The only mechanism that
  still works once USB/host communication is lost. Caveat: ongoing sync reads keep it fed
  even when the write path stalls, so a host-side stale-command check is still required.
- roki's `rkChainID_G` requires non-null, correctly sized velocity/acceleration vectors:
  `rkChainSetJointRateAll` unconditionally indexes both (`rk_chain.c:478`, `:334`).
- `rk_chain_urdf.c:398`: the converter drops URDF effort/velocity limits — they must be
  carried separately (see M1).
- Robust IK requirement met by roki: `rkChainIK(chain, dis, tol, iter)` with error-damped
  solver (`rkChainIKSetEquationSolverED`), prioritized/weighted constraint cells.
- roki-fd sim template: `third_party/roki-fd/example/chain/arm_box_trq_test.c`
  (rkFDCreate → rkFDChainRegFile → rkFDContactInfoScanFile → ODE2Assign/PrpSetDT/SetSolver
  → rkFDUpdate loop; torque via `rkJointMotorSetInput`; writes .zvs each step).
- Catch2 NOT installed as system package → FetchContent. (yaml-cpp 0.8 is installed but
  unused — config format is TOML per user decision.)
- Hardware: DXL IDs 2–8 (arm) + 9 (gripper finger_a; finger_b mimics, no servo). Joint 2 =
  XM540-W270 (2.409 Nm/A), others XM430-W350 (1.783 Nm/A). 3 Mbps, /dev/ttyUSB0. Joint 4
  range -161°..0°, velocity limit 4.817 rad/s. roki has no mimic joints.

## Architecture

Library name/namespace **`rtctrl`**, CMake target `rtctrl::rtctrl`. Four layers +
emulator; `dxl` and `model` are independent, `arm` depends on `model`, `hw` on all.

```
rtctrl/
├── CMakeLists.txt
├── cmake/FindMiLib.cmake          # <lib>-config scripts → imported INTERFACE targets
├── include/rtctrl/ + src/         # mirrored layout
│   ├── dxl/                       # L1: motor comms (DynamixelSDK low-level)
│   │   ├── packet_io.hpp          #   abstract seam: ping/read/write/syncRead/syncWrite
│   │   ├── port.hpp               #   RAII PortHandler+PacketHandler(2.0); implements PacketIO
│   │   ├── control_table.hpp      #   XM430-W350/XM540-W270 register maps + unit conversions
│   │   ├── sync_group.hpp         #   indirect-address programming + GroupSync(Fast)Read/Write
│   │   └── error.hpp
│   ├── model/                     # L2: roki wrappers (RAII over C API)
│   │   ├── chain_model.hpp        #   rkChain RAII: load .ztk, FK, Jacobians, gravityTorque
│   │   ├── joint_map.hpp          #   canonical 8-DOF ↔ DXL IDs ↔ roki joint indices
│   │   ├── ik_solver.hpp          #   rkChainIK w/ ED solver, priorities, weights
│   │   ├── trajectory.hpp         #   min-jerk / sinusoid joint trajectories, limit clamp
│   │   └── zvs_writer.hpp         #   motion logging for rk_anim
│   ├── arm/                       # L3: the sim⇄real BRIDGE
│   │   ├── types.hpp              #   ControlMode{Position,Velocity,Current}=DXL{3,1,0};
│   │   │                          #   JointState{q,dq,tau,t}; JointCommand{mode,q,dq,tau}
│   │   │                          #   — all vectors in canonical 8-DOF order (see below)
│   │   ├── arm.hpp                #   abstract Arm: dof/dt/activate/deactivate/setMode/
│   │   │                          #   readState/writeCommand/step
│   │   ├── sim_arm.hpp            #   roki-fd backed Arm (Current→rkJointMotorSetInput;
│   │   │                          #   Position/Velocity→internal PD emulating servo loop)
│   │   └── runner.hpp             #   run(Arm&, Controller&, T): read→update→write→step
│   ├── hw/                        # L4: CRANE-X7 hardware
│   │   ├── config.hpp             #   TOML config model (groups, ids, modes, margins)
│   │   ├── crane_x7.hpp           #   CraneX7Hardware: rt_manipulators parity + Arm impl
│   │   ├── safety.hpp             #   soft gains, limit clamps, servo Bus Watchdog(98)
│   │   │                          #   + host-side watchdog, zero-on-stop
│   │   └── conversions.hpp        #   pulse↔rad, LSB↔A, current↔torque per model
│   └── emu/motor_emulator.hpp     # control-table state machine (shared by both transports)
├── apps/                          # dxl_inspect, dxl_emu, x7_onoff, x7_read,
│                                  # x7_set_param, x7_move_simple
├── tools/                         # uv project: pyproject.toml (deps: xacro, rospkg)
│   ├── port_model.py              # xacro→URDF→urdf2ztk→path fix→motor patch merge
│   │                              #   (run: `uv run port_model`)
│   ├── bootstrap_milib.sh         # build/install mi-lib submodules in dependency order
│   └── crane_x7_motor.patch.ztk   # hand-written [roki::motor] + (later) friction params
├── models/crane_x7/               # crane_x7.ztk (committed), meshes/, contactinfo.ztk
├── config/crane_x7.toml           # port/baud/joint groups + authored joint limits
├── examples/                      # ports of vendor samples01–03 onto rtctrl API
├── tests/{unit,integration}/      # Catch2; integration = pty emulator + sim tests
└── third_party/                   # submodules (unchanged)
```

**Bridge interface (the load-bearing design):** controllers implement
`Controller::update(const JointState&, JointCommand&, double t)`; `Runner` drives any `Arm`.
`SimArm` wraps roki-fd (pattern from `arm_box_trq_test.c`); `CraneX7Hardware` implements the
same interface on real hardware (`step()` blocks on the RW-thread cycle tick). Same
controller binary verifies in sim, then runs on the robot — PLAN.md's bridge requirement.

**Canonical coordinates (defined in M1, before any controller code):** the system spans
three spaces — 7 arm joints + 1 gripper at the controller, 8 servos (DXL IDs 2–9) on the
bus, 9 revolute joints (7 arm + finger_a + finger_b) in the roki model. Canonical order:
indices 0–6 = arm joints 1–7 (IDs 2–8), index 7 = gripper (ID 9 / finger_a).
`JointState`/`JointCommand`/`Arm::dof()` are always canonical 8-DOF. `model/joint_map.hpp`
owns the explicit mappings — canonical↔DXL ID, canonical↔roki joint index resolved by link
NAME via `rkChainFindLink` at load time (never hardcoded ordinals) — plus the gripper
expansion to finger_a/finger_b. `SimArm`/`RealArm` translate at their boundaries only.

**Mimic finger_b (coupling, not copying):** finger_b stays a real revolute joint in the
.ztk (correct mass/collision for sim). `SimArm` enforces the mechanical linkage as a
penalty constraint: coupling torque τ_c = −K(q_b−q_a) − C(q̇_b−q̇_a) applied **equal and
opposite** (+τ_c on finger_b, −τ_c on finger_a) each step, with the commanded gripper
effort split symmetrically. Reported
gripper state = finger_a. Acceptance test (M3): under asymmetric contact (one finger
blocked), |q_b−q_a| stays under a documented threshold; the residual divergence is
documented as a model limitation. Finger joints get zero IK weight.

## Build design

- **mi-lib:** `cmake/FindMiLib.cmake` with components
  (`find_package(MiLib REQUIRED COMPONENTS roki roki-fd dzco liw)`): per component,
  `find_program(<lib>-config HINTS $ENV{HOME}/usr/bin)`, run `--cflags`/`-l` via
  `execute_process`, split into INTERFACE include dirs/definitions/link dirs/libs, create
  `MiLib::<lib>` INTERFACE IMPORTED target. The -config scripts are transitive already.
- **DynamixelSDK:** `add_subdirectory(third_party/DynamixelSDK/c++ EXCLUDE_FROM_ALL)`, link
  target `dynamixel_sdk`. Ignore its `CONTROL_TABLE_PATH` define (invalid un-installed);
  define our own path to `third_party/DynamixelSDK/control_table/` for .model cross-checks.
  Avoid `dynamixel_easy_sdk` headers entirely.
- **Config format: TOML** via **toml++ (tomlplusplus), pinned GIT_TAG v3.4.0** —
  decided, header-only C++17, FetchContent. Schema mirrors rt_manipulators' shape (joint groups,
  sync signals, per-joint id/model/mode/margins) as `[[...]]` tables. ZTK stays for what
  mi-lib owns (robot model, contact info, .zvs). No yaml-cpp dependency.
- **Catch2 via FetchContent, pinned GIT_TAG v3.7.1** + CTest; labels `unit` (fast,
  in-process) and `integration` (pty/sim). C++17 everywhere.
- **Third-party retrieval policy (submodules + FetchContent combined, by rule):**
  *submodule* = pinned sources needed in-tree — mi-lib libs (built/installed to $HOME/usr
  outside our CMake), DynamixelSDK (built via add_subdirectory; its `control_table/*.model`
  data files are consumed at runtime by dxl_inspect, so a stable in-source path matters),
  crane_x7_description (model source data for the port pipeline, not a build dep).
  *FetchContent* = pure build/test-time deps our CMake builds itself — Catch2, toml++.
  All FetchContent deps pinned to exact tags; mi-lib pinned by submodule SHAs.
- **Clean-clone reproducibility:** `tools/bootstrap_milib.sh` builds/installs the mi-lib
  submodules in dependency order (zeda → zm → zeo → dzco → roki → roki-fd → liw → zx11 →
  roki-gl) into `MILIB_PREFIX` (default `$HOME/usr`). `FindMiLib.cmake` fails with an
  actionable message pointing at the script when a `<lib>-config` is missing. CI exercises
  bootstrap + build + `ctest -L unit` from a clean clone.
- **Python utilities:** `uv`-managed; `tools/pyproject.toml` declares xacro + rospkg;
  scripts run via `uv run` (no global pip installs, no manual venvs).

## Motor mock/emulator (TDD)

One state-machine core `emu::MotorEmulator`, two transports:
1. **FakePacketIO** (implements `dxl::PacketIO`) — in-process, for fast unit tests of
   SyncGroup/CraneX7Hardware logic.
2. **Pty bus emulator** (`apps/dxl_emu.cpp`) — `posix_openpt`, parses Protocol 2.0 frames
   (Ping/Read/Write/SyncRead 0x82/SyncWrite 0x83/FastSyncRead 0x8A, CRC-16 per
   `protocol2_packet_handler.cpp`), so the *unmodified* SDK PortHandler connects to it.
   Integration tests + offline demos of all apps.

Emulator rules: EEPROM & OperatingMode writes rejected while TorqueEnable=1 (error 0x07);
mode-dependent goal register validity; `tick(dt)` motion sim (mode 3: first-order toward
goal clamped by Profile Velocity/Acceleration; mode 1: integrate; mode 0: PresentCurrent =
GoalCurrent + optional 1-DOF inertia); indirect data reads redirect through IndirectAddress
(168+/224+); position/current/velocity limits clamp goals; HardwareErrorStatus injectable;
**Bus Watchdog (98)**: armed value counts 20 ms units; the emulator timestamps EVERY
instruction packet (reads included, per the XM430 e-manual) — silence past the timeout
stops motion and rejects goal writes (error status) until cleared by writing 0. Tests must
cover the trap case: continuous sync reads with stalled writes must NOT trip it.

## Milestones (each PR-sized, independently verifiable)

**M0 — Scaffolding.** Repo layout, root CMakeLists, FindMiLib.cmake (actionable
missing-prerequisite errors), `tools/bootstrap_milib.sh`, DynamixelSDK add_subdirectory,
pinned Catch2/toml++ FetchContent, smoke test (`rkChainInit` + PacketHandler ctor), CI
workflow.
✓ Clean-clone CI: bootstrap → `cmake -B build && cmake --build build && ctest --test-dir
build` green; same sequence green locally.

**M1 — Model port (URDF → .ztk → rk_pen).** `tools/` uv project (`pyproject.toml` with
xacro + rospkg) providing `port_model.py`, run as `uv run port_model`:
`ROS_PACKAGE_PATH=third_party xacro .../crane_x7.urdf.xacro use_gazebo:=false
use_d435:=false use_mock_components:=true` → flat URDF; rewrite
`package://crane_x7_description/meshes` → `models/crane_x7/meshes`; `urdf2ztk`; merge
`crane_x7_motor.patch.ztk` ([roki::motor] type `trq` per driven joint — direct-drive,
torque constants live in `hw/conversions.hpp` not the ztk). The URDF DOES carry nominal
per-joint damping/friction (`crane_x7_arm.xacro:110`; gripper too) but urdf2ztk does not
preserve them — ztk friction starts at ZERO (optionally seeded from those URDF nominals),
refined by M7 identification.
urdf2ztk drops URDF velocity/effort limits (`rk_chain_urdf.c:398`), so author them into
`config/crane_x7.toml` (velocity 4.817 rad/s all joints; effort 10/10/4/4/4/4/4/4 Nm) —
enforced by trajectory clamp, `hw/safety.hpp`, and `SimArm`; a test verifies the TOML
values against the URDF source. Also in M1: **canonical joint order +
`model/joint_map.hpp`** (name-resolved via `rkChainFindLink`), with round-trip tests.
✓ Manual: `rk_pen models/crane_x7/crane_x7.ztk` from repo root. Catch2: load succeeds,
joint count, joint-4 limits ≈[-161°,0°], total mass = URDF sum, FK zero-pose wrist height,
joint-map round-trips (canonical↔ID↔roki index), limits TOML = URDF values.

**M2 — Simple motion → .zvs → rk_anim.** `trajectory.hpp` (min-jerk + sinusoid, 4.817 rad/s
clamp), `zvs_writer.hpp` (`fprintf` dt + `zVecFPrint` per line), `examples/make_motion.cpp`.
`IkSolver` returns a structured `IkResult` (converged flag derived from position/
orientation residuals vs tolerance, iteration count, joint-limit satisfaction) — never a
bare vector. IK acceptance (the "robust IK" criterion): reachable poses — including
selected reachable singular/near-singular configurations, exactly where the error-damped
solver must prove its robustness — → converged, residual < 1e-4 m / 1e-3 rad, within
limits; unreachable targets or incompatible constraints → explicit non-convergence status
(finite result, no NaN, flagged as failure — a finite-but-wrong answer must not pass).
✓ Manual: `rk_anim models/crane_x7/crane_x7.ztk build/motion.zvs`. Catch2: trajectory
boundary conditions, velocity-limit property, zvs re-parse.

**M3 — roki-fd sim + sim side of bridge.** `arm/` layer (types, Arm, SimArm, Runner),
`contactinfo.ztk`.
✓ Catch2 integration: zero-torque gravity drop stays finite (no NaN, 5 s); PD-hold via
Runner holds q₀ within 0.05 rad; determinism (two runs identical); finger-coupling
acceptance — asymmetric contact on one finger keeps |q_b−q_a| under the documented
threshold. Manual: SimArm .zvs → rk_anim.

**M4 — DXL comm layer + emulator + inspection tool.** `dxl/` layer (PacketIO, Port,
ControlTable, SyncGroup with indirect addressing: one FastSyncRead covering
pos/vel/current/voltage/temp, one SyncWrite); `MotorEmulator` + FakePacketIO; pty
`dxl_emu`; `dxl_inspect` (bus scan, read/write named or raw registers, dump-all,
cross-check against SDK `.model` files).
✓ Catch2 unit (Fake): torque gating, mode rules, indirect redirect, unit round-trips.
Integration (pty): real PortHandler @3 Mbps scan finds IDs 2–9, one-transaction multi-signal
read, fault injection → SDK timeout surfaced. Manual: `dxl_emu & dxl_inspect --port
/tmp/ttyDXL scan|dump` offline.

**M5 — Hardware bring-up (ON/OFF → params → sensory read → simple motion).**
`hw/config.hpp` + `config/crane_x7.toml`, `hw/safety.hpp`, minimal `crane_x7.cpp`; apps
in order: `x7_onoff` (soft P=800 on activate / P=5 on deactivate, clamp goal to present),
`x7_read` (stream all present values), `x7_set_param` (gains, profiles, return-delay),
`x7_move_simple` (one-joint min-jerk with limit clamps + the two-layer watchdog below).
**Loss-of-comms safety — two layers, because the host cannot transmit anything once the
link is dead:** (1) *servo-side* Bus Watchdog (register 98) armed on activate (e.g. 100 ms
= value 5); `activate()` reads Firmware Version (register 6) and treats firmware < v38
(no Bus Watchdog support) as activation failure. The watchdog counts ALL instruction
packets, so ongoing sync reads keep it fed even when writes stall; therefore (2) the
*host-side* watchdog independently tracks time since the last successful COMMAND write /
controller update — even while reads succeed — and on staleness ESCALATES: best-effort
zero commands + deactivate, then, whether or not those writes succeeded, STOP ALL BUS
TRAFFIC and close the port so the servo Bus Watchdog necessarily fires (continuing reads
would otherwise keep feeding it while failed safety writes leave torque on). Plus
missed-cycle/stale-read handling. `deactivate()` is NOT an e-stop: hardware sessions
require an independent actuator-power cutoff within arm's reach, stated in the runbook.
✓ Each app against pty emulator in CI first, incl. emulated Bus Watchdog timeout AND the
reads-alive/writes-stalled trap case — acceptance is the MOTOR's end state: host watchdog
fires, escalation goes bus-silent, and the emulated motor verifiably stops / enters Bus
Watchdog error (not merely "host watchdog triggered"); then physical checklist on
/dev/ttyUSB0, wrist joint (ID 8) before all joints. No motion on activate; halting all
bus traffic (USB unplug) → servo stops autonomously via Bus Watchdog, confirmed by state
read-back after reconnect.

**M6 — Position-control parity with rt_manipulators.** Full `CraneX7Hardware`: TOML joint
groups, background read→limit→write thread (liwPAction or timerfd), all getters/setters,
gain/profile writers, validation rule (writing velocity/current requires reading position),
zero-on-stop, `Arm` adapter. Port vendor samples01–03 to `examples/`.
✓ Catch2 (pty): 60 s thread loop with missed-deadline budget, limit-clamp properties,
watchdog, validation rejections. Parity checklist mapping every vendor feature to a
test/example. Manual: examples reproduce vendor sample behavior on the real arm.

**M7 — Current-based torque estimation + gravity compensation.** `conversions.hpp`
torque↔current (2.409 / 1.783 Nm/A); `JointState.tau` from PresentCurrent; gravity term via
`ChainModel::gravityTorque(q8)`: expand canonical 8 → 9 model coordinates via JointMap
(finger_b = finger_a), call `rkChainID_G(chain, q9, zeroVel9, zeroAcc9, RK_GRAVITY6D,
trq9)` with member-owned zero vectors sized `rkChainJointSize` — NEVER null
(`rkChainSetJointRateAll` dereferences both unconditionally, `rk_chain.c:478`, `:334`) —
then reduce 9 → 8 through the constraint Jacobian transpose: with mimic multiplier 1,
canonical gripper torque = τ_finger_a + τ_finger_b (summed, not finger_a alone).
Regression tests cover the null-vector call path, coordinate projection (8→9→8) and the
force mapping via the virtual-work identity τ₈ᵀδq₈ = τ₉ᵀδq₉ as separate checks. `GravityComp` controller in Current mode. Friction identification starts here
(dzco `dz_ident_lag`), feeding the ztk friction params.
✓ Sim first via bridge: drift <0.05 rad over 10 s under gravity comp; same binary on
hardware — arm floats, back-drivable. Static-pose tau vs `rkChainID_G` prediction, document
per-joint error (~20–30 % expected, friction unmodeled).

**M8 — Torque control via inverse dynamics.** Computed-torque tracking
`tau = ID_G(q_d, dq_d, ddq_d) + Kp·e + Kd·ė`, current clamped to CurrentLimit − margin,
optional dzco PID/filter blocks; runs through Runner unchanged sim→real.
✓ Sim: tracking RMS below threshold on the M2 trajectory, ID+PD vs PD-only comparison.
Hardware: reduced speed, conservative clamps; safety = M5's two-layer watchdog plus the
independent actuator-power cutoff (deactivate() is not an e-stop).

## Git workflow

**Conventional Commits** (https://www.conventionalcommits.org/en/v1.0.0/), one commit per
coherent, appropriately-sized change — never one commit per milestone. Types: `feat`,
`fix`, `build`, `test`, `docs`, `refactor`, `chore`; scope = layer or area, e.g.
`feat(dxl): add indirect-address sync group`, `build(cmake): add FindMiLib module`,
`feat(model): port CRANE-X7 URDF to ztk`, `test(emu): cover torque-enable gating`,
`feat(arm): define sim/real bridge interface`, `chore(tools): set up uv project`.
Guideline per milestone: M0 ≈ 3–4 commits (scaffold, FindMiLib, DynamixelSDK wiring,
Catch2/CTest); larger milestones commit per module + its tests together (test code lands
with the feature it tests, `test:` type reserved for test-only changes).

## Risks / open questions

1. ~~Mesh paths CWD-relative~~ **Resolved at M1:** mi-lib viewers (rk_pen/rk_anim/rk_view)
   chdir into the model's directory before reading, so imports are written relative to the
   .ztk itself (`meshes/...`) and `ChainModel` mirrors the same chdir convention on load.
2. **urdf2ztk inertia fidelity** (URDF inertial origin rpy → roki tensor) — numeric
   comparison test at M1. Mimic tag silently dropped — handled by penalty coupling (M3).
3. **xacro standalone**: if pip xacro rejects `$(find …)` + ROS_PACKAGE_PATH, fallback is a
   thin top-level xacro including `crane_x7.xacro` by relative path (skips ros2_control/
   gazebo blocks — cleaner output anyway).
4. **Realtime**: 9-servo FastSyncRead + SyncWrite ≪2 ms/cycle → 100–200 Hz safe; overrun
   counters in M6; document `latency_timer=1` udev rule for ftdi_sio.
5. **Current ≠ torque** (friction, gear efficiency): M7 measures the gap; per-joint
   friction feedforward later (params identified in M7, then stored in the ztk).
6. **Pty emulator replies instantly** — inject artificial delays for watchdog tests; final
   latency validation stays on hardware (M5).
7. **XM540 vs XM430 deltas** (CurrentLimit defaults, stall ratings) — encode per-model in
   ControlTable, cross-check `.model` files at M4.
8. **Finger penalty coupling is a stiff term** — K/C must be tuned against the sim dt to
   avoid integrator instability; the M3 determinism/finite checks and the coupling
   acceptance test bound it, thresholds documented with the chosen gains.

## Key reference files (for implementation)

- `third_party/roki/app/urdf2ztk.c`, `third_party/roki/src/rk_chain_urdf.c` — converter
- `third_party/roki/include/roki/{rk_chain.h,rk_ik.h,rk_jacobi.h}` — FK/IK/ID API
- `third_party/roki-fd/example/chain/arm_box_trq_test.c` — sim loop template
- `third_party/roki-fd/example/model/*.ztk`, `mi-lib-tutorial/roki/example/puma.ztk` — ztk style
- `third_party/DynamixelSDK/c++/example/protocol2.0/{sync_read_write,indirect_address}/` — SDK usage
- `third_party/DynamixelSDK/c++/src/dynamixel_sdk/protocol2_packet_handler.cpp` — frame/CRC spec for emulator
- `third_party/DynamixelSDK/control_table/xm430_w350.model` — register data incl. Bus Watchdog (98)
- `third_party/roki/src/rk_chain.c:334,478` — why rkChainID_G needs non-null rate vectors
- `rt_manipulators_cpp/rt_manipulators_lib/{include,src}/` — feature parity reference (before deletion)
- `crane_x7_ros/crane_x7_control/config/{manipulator_config.yaml,manipulator_links.csv}` — IDs, gains, torque constants, inertias (copy needed values before deletion)
- `third_party/crane_x7_description/urdf/*.xacro` — model source

## Verification summary

Per milestone above; overall: `ctest -L unit` (fast, no devices), `ctest -L integration`
(pty emulator + roki-fd sim), manual visual checks with `rk_pen`/`rk_anim`, and the staged
hardware checklist (M5→M8) with safety gates (soft gains, clamps, Bus Watchdog + host
watchdog, independent power cutoff) proven against the emulator before each hardware step.
