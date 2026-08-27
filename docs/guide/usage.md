# Using the library

All code below is real API; headers live under `include/rtctrl/`.
Link against the `rtctrl::rtctrl` CMake target.

## Model and coordinates

```cpp
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

using namespace rtctrl;

model::ChainModel chain("models/crane_x7/crane_x7.ztk");
model::JointMap map(chain);   // canonical 8-DOF ↔ DXL ids ↔ roki indices

model::ZVector q8(model::kCanonicalDof);   // RAII zVec, starts zeroed
model::ZVector q9(model::kModelDof);
q8[1] = 0.6;                    // shoulder tilt
map.expand(q8, q9);             // finger B mimics finger A
chain.fk(q9);                   // forward kinematics
```

`ChainModel::chain()` exposes the underlying `rkChain*` for any roki
API not wrapped yet: the wrappers are conveniences, not a wall.
Model loading resolves mesh paths relative to the `.ztk` file (the
mi-lib convention), so it works from any working directory.

## Inverse kinematics

```cpp
#include "rtctrl/model/ik_solver.hpp"

model::IkSolver ik(chain, map);          // arm joints only; fingers excluded
zVec3D target_pos; zMat3D target_att;    // world-frame gripper-base pose
// ... fill target ...

model::ZVector init(model::kModelDof), solution(model::kModelDof);
const auto result = ik.solve(target_pos, target_att, init, solution);
if (!result.converged) {
  // unreachable or constrained: residuals/iterations say why.
  // A finite-but-wrong answer never comes back as success.
}
```

`IkResult` carries position/attitude residuals, the iteration count,
and joint-limit/finiteness flags. The solver uses roki's
Levenberg–Marquardt iteration with the error-damped equation solver;
it converges at reachable singular poses (that robustness is pinned by
tests).

The effector defaults to the gripper base; pass a link name to target
another frame. The model ships a virtual **tool-center point**,
`crane_x7_tcp_link`: a fixed, massless frame at the midpoint of the
closed fingertips whose axes coincide with the world axes when the
gripper points forward (+x), so TCP x is the approach direction and
TCP z is up when the gripper is level. World-frame pose targets then
read naturally (identity attitude = level, forward-pointing gripper):

```cpp
model::IkSolver tcp_ik(chain, map, "crane_x7_tcp_link");
```

On hardware,
`x7_pose --tcp X Y Z ROLL_RAD PITCH_RAD YAW_RAD` uses exactly this
solver (seeded from the measured posture; a non-converged solve is
refused before any motion), and adding `--preview <basename>` writes
an `.init.ztk` for `rk_pen` without touching the bus. For both `--tcp`
and `--posture`, the subsequent hardware placement must bring every
measured joint within 0.01 rad of its resolved target. A stalled or
exhausted placement correction, or any dropped position command,
deactivates the arm and returns a nonzero exit instead of entering the
normal hold phase.

### Authored postures

Authored joint postures use strict, versioned TOML files under
`config/postures/`:

```toml
format_version = 1
name = "Zeros posture"
joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

`joint_positions` contains the eight canonical joint displacements in
radians. Unknown keys, incompatible versions, non-finite values, and a
joint count other than eight are errors. Use a posture without hardware
by adding `--preview`:

```sh
./build/apps/x7_pose --posture config/postures/zeros.toml \
  --preview /tmp/zeros
```

Identification `.dwells.json` files are generated experiment sidecars,
not authored posture files. Archived sidecars remain usable through the
explicit `--legacy-anchor-sidecar <file.dwells.json>` compatibility
option. Arbitrary text or JSON is not accepted by `--posture`.

## Trajectories and motion files

```cpp
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvs_writer.hpp"

auto traj = model::MinJerkTrajectory::withVelocityLimit(q_start, q_goal,
                                                        /*vel limit*/ 1.0);
model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
    ddq(model::kCanonicalDof);
traj.sample(t, q, dq, ddq);     // C² quintic; dq/ddq optional

model::ZvsWriter log("motion.zvs");   // one "<dt> <zVec>" line per frame
log.frame(0.01, q9);                  // view: rk_anim <model.ztk> motion.zvs

chain.writeInitZtk("pose.init.ztk", q9);  // static posture for
                                          // rk_pen -model <model.ztk> -init pose.init.ztk
```

`writeInitZtk` delegates to roki's own `rkChainInitWriteZTK`: the
`[roki::chain::init]` format stores revolute displacements in degrees,
so a hand-written radian file loads as a near-zero posture.

### Cartesian PTP planning

`x7_plan_ptp` plans a straight world-frame TCP translation with a
shortest-path spherical attitude interpolation. It solves IK at every
sample, writes the model's nine joint coordinates to a `.zvs` file, and
writes an analysis-ready trajectory diagnostics CSV. The app is entirely
offline and never opens the hardware bus.

Start with the checked-in configuration:

```sh
./build/apps/x7_plan_ptp --config config/ptp_example.toml
rk_anim models/crane_x7/crane_x7.ztk ptp.zvs
```

For a top-down approach toward a nominal tabletop object, use the separate
pick example:

```sh
./build/apps/x7_plan_ptp --config config/ptp_pick_example.toml
rk_anim models/crane_x7/crane_x7.ztk ptp_pick_approach.zvs
uv run --project tools tools/plot/ptp_trajectory.py \
  ptp_pick_approach.csv --output /tmp/ptp-pick-review.png
```

It holds the TCP at world RPY `[0, pi/2, 0]`, which points the TCP approach
axis downward, and descends vertically from 0.22 m to 0.09 m at a horizontal
reach of 0.28 m. Those coordinates are illustrative. Measure the actual table
and object frame, check collisions, and update both endpoints before using the
trajectory around hardware. The plan does not command the gripper to close.

The configuration schema is:

```toml
model = "../models/crane_x7/crane_x7.ztk"
output = "ptp.zvs"
end_effector = "crane_x7_tcp_link"

[diagnostics]
enabled = true
output = "ptp.csv"

[trajectory]
profile = "minimum-jerk" # linear, trapezoidal, or minimum-jerk
sample_rate = 100.0
motion_time = 2.0
max_linear_velocity = 0.05
max_angular_velocity = 0.5
trapezoid_acceleration_fraction = 0.2

[start]
position = [0.20, 0.0, 0.25] # m, world frame
rpy_rad = [0.0, 0.0, 0.0]   # roll, pitch, yaw in radians

[end]
position = [0.22, 0.0, 0.25]
rpy_rad = [0.0, 0.0, 0.0]

[ik]
strict = true
position_tolerance = 1.0e-4
attitude_tolerance = 1.0e-3
max_iterations = 200
initial_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

The attitude convention is
$R=R_z(\mathit{yaw})R_y(\mathit{pitch})R_x(\mathit{roll})$, matching
`x7_pose --tcp`. Both apps convert these world-frame RPY radians to an
attitude matrix before calling `IkSolver`; the solver receives the
matrix, not an angle-axis vector. RoKi's degree-valued angle-axis syntax
applies to textual ZTK input and is not the runtime IK reference
convention. The model path is relative to the TOML file. Trajectory and
diagnostics output paths are relative to the directory where the app is
invoked; `.zvs` and `.csv` are appended when absent. Unknown keys and
malformed values are errors, which prevents misspelled options from
silently using defaults.

`model`, `[start]`, and `[end]` are required. Optional defaults are
`ptp.zvs`, a same-basename `ptp.csv` diagnostics file,
`crane_x7_tcp_link`, minimum jerk, 100 Hz, strict IK, position tolerance
$10^{-4}$ m, attitude tolerance $10^{-3}$ rad, 200 IK iterations, and
an all-zero canonical initial joint seed.

The three profiles apply to one scalar path-progress variable. The
geometric translation remains a line and attitude remains SLERP for
all profiles:

- `linear` has constant path speed and an instantaneous velocity
  change at each endpoint.
- `trapezoidal` has symmetric acceleration and deceleration ramps.
  Each ramp occupies 20 percent of the motion by default.
- `minimum-jerk` uses the quintic
  $10u^3-15u^4+6u^5$ and is the default.

Motion time and Cartesian velocity limits are compatible constraints,
not mutually exclusive choices. The selected duration is

```math
T=\max\left(T_{requested},\;c\frac{\lVert\Delta p\rVert}{v_{max}},\;
 c\frac{\theta}{\omega_{max}}\right),
```

where $\theta$ is the shortest attitude change and $c$ is the
profile's peak progress speed: 1 for linear, $1/(1-a)$ for a
trapezoid with ramp fraction $a$, and $15/8$ for minimum jerk. Linear
and angular velocity limits must be specified together. If no timing
constraint is present, the duration defaults to 5 seconds.

The planner creates enough equal intervals to meet or exceed the
requested sample rate, adjusts the interval so the endpoint occurs at
the exact duration, and includes both endpoints. The first IK solve
uses `initial_joints` in canonical order; each later solve uses the
previous solution. The gripper stays at its initial value, and finger
B is emitted as the mimic of finger A.

Strict IK is the default. A non-converged sample aborts planning before
the output file is opened. With `strict = false`, a finite, in-limit
best result is written with a warning containing its time and
residuals; a non-finite or limit-violating result remains fatal. The
app also reports the worst IK residuals, peak sampled joint speed and
acceleration, and minimum joint-limit margin.

#### Trajectory diagnostics and plots

The diagnostics CSV has one schema-versioned row per `.zvs` sample. It
records:

- analytic target TCP position, quaternion attitude, world-frame linear
  and angular velocity, and acceleration;
- FK position, quaternion attitude, velocity, and acceleration evaluated
  from the IK joint solution;
- world-frame position and rotation-vector errors plus the complete IK
  result;
- canonical eight-joint position, sampled velocity and acceleration, and
  joint-limit margin.

Target Cartesian derivatives come directly from the selected scalar
profile. Joint velocity and acceleration are second-order finite-difference
estimates of the sampled IK path; the FK rates use those estimates. They do
not yet define a continuous joint interpolant for a hardware tracker.
`profile_derivative_discontinuity` marks samples where an ideal linear or
trapezoidal profile has no unique two-sided derivative.

Quaternion attitude is authoritative. Angular velocity and acceleration
are world-frame vectors, not derivatives of RPY angles. The plotter derives
and unwraps RPY only for display:

```sh
uv run --project tools tools/plot/ptp_trajectory.py ptp.csv \
  --output /tmp/ptp-review.png
```

The plot covers target versus FK TCP position and attitude, Cartesian
speed and acceleration, joint position, joint velocity and acceleration,
IK/FK errors, and minimum joint-limit margin. Use `--show` for an
interactive window. Without `--show`, `--output` is required so plotting a
completed bundle cannot modify it accidentally.

Use `--diagnostics FILE` to select another CSV path or `--no-diagnostics`
for ordinary non-bundle planning. A bundle always contains diagnostics.

Common settings can be overridden without editing the TOML file:

```sh
./build/apps/x7_plan_ptp --config plan.toml \
  --output trial.zvs --diagnostics trial.csv --motion-time 4 \
  --max-linear-velocity 0.04 --max-angular-velocity 0.4 \
  --sample-rate 200 --profile trapezoidal --no-strict-ik
```

#### Portable planning bundles

Create an immutable, self-contained archive by giving a directory that
does not yet exist:

```sh
./build/apps/x7_plan_ptp --config plan.toml \
  --bundle backups/pick-20260826 --motion-time 4
```

`--bundle` owns the trajectory and diagnostics locations and is therefore
exclusive with `--output`, `--diagnostics`, and `--no-diagnostics`. The
target is checked before the source configuration is loaded. An existing
directory, regular file, or symbolic link is refused without inspection
or modification, even if the source configuration is missing or malformed.
Failed creation removes its private staging directory and does not publish
a partial bundle.

A successful bundle contains:

```text
pick-20260826/
  source.toml
  plan.toml
  trajectory.zvs
  trajectory.csv
  manifest.toml
  model/
    crane_x7.ztk
    meshes/visual/...
```

`source.toml` is a byte-for-byte copy of the input. `plan.toml` records
the effective configuration after CLI overrides, with portable paths
`model/crane_x7.ztk`, `trajectory.zvs`, and `trajectory.csv`. The planner
reloads this copied configuration and generates the initial trajectory and
diagnostics from the copied model. Every relative `import:` used by the
model is copied with its directory layout; absolute imports and imports
that escape the model directory are refused.

`manifest.toml` records the bundle format version, rtctrl version, build
Git commit and dirty state, and the size and SHA-256 of every archived
file. The bundle is staged beside its destination and published with a
no-replace rename only after planning and hashing succeed.

Reproduce into a new output file without modifying the archive:

```sh
./build/apps/x7_plan_ptp \
  --config backups/pick-20260826/plan.toml \
  --output /tmp/pick-reproduced.zvs \
  --diagnostics /tmp/pick-reproduced.csv
cmp backups/pick-20260826/trajectory.zvs /tmp/pick-reproduced.zvs
cmp backups/pick-20260826/trajectory.csv /tmp/pick-reproduced.csv
```

For strict reproduction, build the Git commit named in `manifest.toml`.
Loading `plan.toml` resolves the model relative to the archive, so the
repository's current model is not used. `--bundle` is creation-only and
is not used for replay.

The planner checks kinematics and model joint limits, but it does not
perform collision checking or certify a trajectory for hardware. A
hardware tracking app must independently validate the file, starting
posture, joint rates, operating modes, and collision constraints
before enabling torque.

### Servo-side trajectory following

`x7_follow_sim` and `x7_follow` run the same phase controller against the
dynamics simulator and the real arm. They delegate the feedback loop to each
Dynamixel servo. The host sends position, velocity, or current-based-position
commands without a host-side tracking correction during the reference phase.
This is separate from the parked computed-torque `x7_track` app.

Generate the example reference, validate the follow configuration, and run the
simulation first:

```sh
./build/apps/x7_plan_ptp --config config/ptp_example.toml
./build/apps/x7_follow_sim --config config/follow_example.toml --check
./build/apps/x7_follow_sim --config config/follow_example.toml \
  --motion /tmp/follow-sim.zvs --log /tmp/follow-sim.csv
rk_anim models/crane_x7/crane_x7.ztk /tmp/follow-sim.zvs
uv run --project tools tools/plot/follow_tracking.py /tmp/follow-sim.csv \
  --output /tmp/follow-review.png
```

Both output files cover the complete session: home positioning, home
correction and settling, reference tracking, and the final hold. The CSV also
records the reference and measured joint states, mode-native command,
submission receipt, applied target sequence, effort ceiling, and clamp/gate
flags. The plotter validates this schema and places its legends above the
traces so they do not obscure the data.

The checked-in `config/follow_example.toml` documents every option. Input paths
are relative to that TOML file. Output
paths are invocation-relative and are created exclusively; an existing file
is refused. The main processing controls are:

- `control.rate_hz`: one 20 to 200 Hz cycle rate for every phase. Hardware
  timing has been validated at 100 Hz; other rates print an experimental-use
  warning.
- `control.mode`: `position`, `velocity`, or `current-based-position`.
  Current-based position requires one or eight positive
  `current_based_position.effort_limit_nm` values, each no greater than the
  deployment limit.
- `home`: linear, trapezoidal, or minimum-jerk PTP motion to the first
  reference frame. Duration is the greater of `motion_time`, when present,
  and the duration required by `velocity_limit`. Strict mode refuses tracking
  if the measured home error remains above `tolerance_rad` after bounded
  correction retries. Non-strict mode records and warns about that error.
- `reference_processing`: no filter, first-order low-pass, moving average, or
  Savitzky-Golay filtering, followed by linear or shape-preserving cubic
  interpolation at the control-cycle times. Filtering preserves the authored
  first and final frames. The interpolator provides joint position, velocity,
  and acceleration at arbitrary times even when the `.zvs` and control rates
  differ.
- `safety`: a one-time tracking warning, a sustained-error abort threshold and
  duration, and a larger immediate-error abort threshold. There is no strict
  endpoint convergence gate during tracking. These thresholds detect gross
  disagreement without turning ordinary servo following error into a phase
  transition.
- `finalization`: a positive `wait_time_s` holds for that duration. Zero keeps
  the final posture commanded until the operator supports the arm from below
  and presses Enter, bounded by `operator_timeout_s`. The nonblocking Enter
  check cannot starve the hardware command cycle.

This first version deliberately uses one homogeneous motor mode for home,
tracking, and finalization. Changing a Dynamixel operating mode requires
torque to be disabled. A mode change at the home-to-tracking boundary would
therefore introduce an unsupported gravity-drop interval. Supporting different
phase modes requires a separately reviewed in-place transition protocol.

The simulation converts position commands to bounded PD torque and velocity
commands to bounded velocity-error torque, then integrates the robot dynamics.
Current-based position uses the same position adapter with the requested
effort ceiling. These adapters verify phase logic, interpolation, safety gates,
and command saturation. They are not a high-fidelity model of the Dynamixel
firmware, current loop, friction, backlash, or gear compliance.

#### Motor parameters and hardware run

An optional `motor_parameters` input accepts the versioned TOML produced by
`dxl_inspect dump-params`. Keep only the parameters reviewed for this run and
remove `operating_mode`: `control.mode` owns that register. Unknown motor IDs
are refused before bus contact. During activation the batch is applied and
read back transactionally while every servo is torque-off; incomplete rollback
aborts activation. The bundle copies this exact parameter input.

After completing the hardware bring-up checklist and reviewing the simulation
log, use position mode for the first hardware trial:

```sh
./build/apps/x7_follow --config config/follow_example.toml --check
run_stamp=$(date +%Y%m%d-%H%M%S)
./build/apps/x7_follow --config config/follow_example.toml \
  --log "follow-hw-${run_stamp}.csv"
```

`--check` performs configuration, model, reference, joint-limit, velocity,
effort, and motor-parameter-file validation without opening the bus. A real run
uses the feedback-synchronized hardware cycle, servo and host watchdogs, stale
command rejection, and verified shutdown. Keep the actuator power cutoff in
reach. At completion, support the arm from below before Enter; torque-off makes
the arm fall limp. Any home refusal, tracking abort, I/O failure, operator
timeout, stale submission, or unclean shutdown is a failed run.

#### Portable follow bundles

Create a new immutable simulation archive:

```sh
./build/apps/x7_follow_sim --config config/follow_example.toml \
  --bundle backups/follow-sim-20260827
```

For hardware, replace `x7_follow_sim` with `x7_follow`. The requested bundle
path must not exist. It is refused before the source configuration is loaded,
and publishing uses a no-replace rename. `--check` and `--bundle` are mutually
exclusive. A completed archive contains `source.toml`, portable `follow.toml`,
the reference, deployment and optional motor parameter files, model and meshes,
the frontend outputs, `result.toml`, and a SHA-256 `manifest.toml`. Normal
controller refusals are archived with their result and full log; setup failures
remove the private staging directory.

Replay a simulation bundle into new files, leaving the archive unchanged:

```sh
./build/apps/x7_follow_sim \
  --config backups/follow-sim-20260827/follow.toml \
  --motion /tmp/follow-reproduced.zvs \
  --log /tmp/follow-reproduced.csv
cmp backups/follow-sim-20260827/simulation.zvs /tmp/follow-reproduced.zvs
cmp backups/follow-sim-20260827/simulation.csv /tmp/follow-reproduced.csv
```

For strict reproduction, build the Git commit recorded by the manifest. A
hardware replay is intentionally not expected to be byte-identical because
measured state and bus timing are physical inputs.

## Writing a controller (the bridge)

A controller is one function, written once, run anywhere:

```cpp
#include "rtctrl/arm/runner.hpp"

struct MyController : arm::Controller {
  explicit MyController(const arm::JointState& start) {
    zVecCopyNC(start.q.get(), home.q.get());  // capture home ONCE
  }
  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    (void)state;
    cmd.mode = arm::ControlMode::Position;      // or Velocity / Current
    zVecCopyNC(home.q.get(), cmd.q.get());      // hold home + offset
    const double ramp = std::min(1.0, t / 3.0); // no step at t = 0
    zVecElemNC(cmd.q.get(), 6) += 0.3 * ramp * std::sin(t);
  }
  arm::JointState home;
};
```

Two safety habits are baked into that shape (they are what
`examples/x7_wave.cpp` does): the reference is anchored to a
*captured* posture, never an absolute value that would command a jump
from wherever the arm actually is, and the offset ramps in from zero
so the first command equals the measured posture.

**In simulation** (roki forward dynamics, deterministic, no hardware):

```cpp
#include "rtctrl/arm/sim_arm.hpp"

arm::SimArm::Options opt;
opt.initial_q8 = {0, 0.6, 0, -1.2, 0, -0.5, 0, 0.2};
arm::SimArm sim(opt);
sim.setMode(arm::ControlMode::Position);
sim.activate();                       // no motion commanded
arm::JointState start;
sim.readState(start);
MyController c(start);
arm::run(sim, c, /*seconds=*/10.0);   // read → update → write → step
```

**On the robot** (identical controller code; this listing is written
as an in-repo program: the verified-shutdown guard it uses is the
apps' shared plumbing, not part of the installed library API):

```cpp
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/hw/crane_x7.hpp"
// Repository-internal: the x7_* apps' shared plumbing lives under
// apps/, NOT in the installed rtctrl:: API — in-repo programs include
// it relatively (examples/ does exactly this):
#include "../apps/common/x7_common.hpp"

auto config = hw::Config::load("config/crane_x7.toml");
dxl::Port port(config.port, config.baudrate);
hw::CraneX7 hardware(port, config);
hardware.onEscalate([&port] { port.close(); });  // deadman → bus silence

arm::RealArm robot(hardware);
if (!robot.setMode(arm::ControlMode::Position)) return 1;  // must match config
if (!robot.activate()) return 1;  // verifies servos, arms watchdogs, snaps goals
// Verified shutdown from here on: CraneX7's DESTRUCTOR deliberately
// does not torque off, so every exit after activation must deactivate
// and CHECK the result — the x7_* apps share x7::ShutdownGuard
// (apps/common/x7_common.hpp), which also silences the bus on an unclean
// deactivation so the servo watchdogs halt the arm.
x7::ShutdownGuard shutdown{hardware};
arm::JointState start;
if (!robot.readState(start)) { shutdown.run(); return 1; }
MyController c(start);
const bool ok = arm::run(robot, c, 10.0);
const bool clean = shutdown.run();  // deactivate + verify; quiesce on failure
return ok && clean ? 0 : 1;
```

(`examples/x7_wave.cpp` is a complete example using this pattern,
including the `apps/` header the same relative way, runnable against
`dxl_emu` or the robot.)

`readState` gives positions, velocities, and torque estimates
($\hat\tau = k_t\,i$). On hardware it returns a fresh sample while that
sample's bounded command window is open; `step()` blocks until the next
successful feedback read and returns `false` after a safety escalation. The
shipped controllers `arm::GravityComp` and `arm::ComputedTorque` follow
exactly this pattern; see the
[theory documents](../theory/gravity-compensation.md) and their
sources for worked examples.

## Talking to servos directly

For tools and experiments below the bridge:

```cpp
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/dxl/sync_group.hpp"

dxl::Port port("/dev/ttyUSB0", 3000000);
dxl::SyncGroup group(port, {2, 3, 4, 5, 6, 7, 8, 9});
group.setupIndirect();                 // once, torque OFF (EEPROM-gated)

std::vector<dxl::Feedback> fb;
group.readAll(fb);                     // ONE bus transaction, SI units
port.write8(8, dxl::reg::kLed.addr, 1);
```

The `dxl_inspect` app is this API as a CLI. In tests, substitute
`emu::FakePacketIO` for `Port` behind the same `PacketIO` interface,
or run against `dxl_emu`'s pseudo-terminal with `Port` itself.

### Backing up and restoring motor parameters

`dxl_inspect dump-params` captures the configuration and tuning
XM430-W350/XM540-W270 registers in a versioned TOML document. With no ids it
scans the complete Protocol 2.0 id range; trailing ids select only those
motors. Use `-` instead of a filename for machine-readable standard output.

```sh
# All motors found on the bus
./build/apps/dxl_inspect --port /dev/ttyUSB0 dump-params x7-params.toml

# Only ids 2, 3, and 8
./build/apps/dxl_inspect --port /dev/ttyUSB0 \
  dump-params selected-params.toml 2 3 8
```

Each `[[motor]]` records its id, model number, firmware version, and a
`[motor.parameters]` table. Values are raw control-table integers. The dump
includes communication settings for audit completeness, but the loader treats
`baud_rate`, `secondary_id`, `protocol_type`, `status_return_level`, and
`startup_configuration` as dump-only. The communication fields can remove the
response path needed to verify and roll back the transaction. Startup
configuration can enable torque automatically after power-on, which is outside
this loader's torque-off contract.

To apply a file, leave only the parameter keys that should be enforced, edit
their values, and run:

```sh
# Apply entries for every motor in the file
./build/apps/dxl_load_params --port /dev/ttyUSB0 x7-params.toml

# Apply only the id 2 entry
./build/apps/dxl_load_params --port /dev/ttyUSB0 x7-params.toml 2
```

The loader parses and validates the whole file before opening the bus. It then
requires every selected motor to match the recorded model and report torque
off, and completes all preflight reads before its first write. Unchanged values
are skipped to avoid EEPROM wear. Changed values are read back exactly. A
failure triggers best-effort rollback and reports whether rollback was fully
verified. Operating-mode changes reset gains and profiles in XM firmware, so
the loader snapshots and restores any such fields omitted from the file.

The document is intentionally strict: unknown keys, duplicate ids, invalid
register widths, and unsupported operating-mode values are errors. Runtime
state such as torque enable, Bus Watchdog, goals, telemetry, LEDs, and indirect
address mappings is not part of a parameter dump.

## Testing your additions

- Pure logic → Catch2 unit test (`tests/unit/`), label `unit`.
- Anything touching the bus → drive it against `emu::FakePacketIO`
  (fast) and, if it exercises new wire behavior, the pty fixture
  (`tests/integration/`).
- Anything producing motion → acceptance test on `SimArm` first;
  hardware only after (that ordering is the project's core rule,
  though sim passage is necessary, not sufficient: the rigid-joint sim
  cannot certify gains against gear elasticity; see the
  [computed-torque theory notes](../theory/computed-torque.md#what-the-hardware-taught-us)).
- Line-speed/serial-timing behavior is invisible to pty tests
  (a lesson learned the hard way; see the hardware findings in
  [history.md](../records/history.md)): budget a hardware check for it.

## Conventions

- Conventional Commits; a commit is a module plus its tests.
- Canonical joint order everywhere above the `dxl` layer.
- SI units (rad, rad/s, Nm, A, V) outside `dxl/conversions.hpp`;
  raw servo units never leak upward; the deliberate exceptions are
  the servo-parameter passthroughs (operating-mode codes and the raw
  profile registers on `CraneX7`), which stay in register units by
  design.
- New mi-lib quirks belong in the repo guidance (CLAUDE.md gotchas)
  and the project memory: several cost hours to discover.
