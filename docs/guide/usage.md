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

Hardware placement defaults to normal servo position mode. Select
current-based-position explicitly when that mode is part of the experiment:

```sh
./build/apps/x7_pose --posture config/postures/zeros.toml \
  --mode current-based-position --effort-limit-nm 2.5
```

The CBP path stages the requested all-joint effort ceiling before torque-on
and carries it through placement and the final hold. The ceiling defaults to
2.5 Nm when omitted and must not exceed any joint's configured limit.
`--effort-limit-nm` is rejected in position mode so a safety-relevant value is
never silently ignored. CBP changes the servo operating mode; it does not add
host-side feedback or computed torque. Keep the power cutoff in reach and
support the arm before pressing Enter to disable torque.

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
if (!log.close()) { /* a write failed; the file is incomplete */ }

chain.writeInitZtk("pose.init.ztk", q9);  // static posture for
                                          // rk_pen -model <model.ztk> -init pose.init.ztk
```

`ZvsWriter` buffers its frames, so a write error (a full disk, for example)
often surfaces only when the buffer is flushed. The first failure latches:
`frame()` returns false for it and every later frame, and `close()` returns
false. Only `close()` returning true means the file holds every counted frame.
Every app and example that writes through `ZvsWriter` checks it: the planner
fails with `cannot finish PTP output` before publishing a bundle, and the
follow and track simulators report `motion: writing <file> failed; the motion
is incomplete` and exit 1.

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

It starts at `[0.22, -0.10, 0.26]` m with the TCP pointing forward, then reaches
forward and across the workspace to `[0.28, 0.10, 0.08]` m while pitching by
`pi/2` so the TCP approach axis finishes downward. Those coordinates are
illustrative. Measure the actual table and object frame, check collisions, and
update both endpoints before using the trajectory around hardware. The plan
does not command the gripper to close.

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

### Host-side computed-torque tracking

`x7_track_sim` and `x7_track` replay a ZVS reference with the same host-side
current controller:

```math
tau = ID(qref, dqref, ddqref) + Kp (qref - q) + Kd (dqref - dq)
```

The controller uses measured velocity directly. It has no host velocity
estimator, low-pass filter, integral term, per-joint gain scaling, friction
term, or current-mode settling controller. Those historical countermeasures
remain available in `arm::PracticalComputedTorque`, but the renewed app does
not select them. For the complete timestamped bundle workflow, hardware
procedure, and analysis commands, see
[Host-side controller comparison](host-side-controller-comparison.md).

Generate a synthetic reference, check the complete configuration, and run the
dynamics simulation first:

```sh
./build/apps/x7_plan_ptp --config config/ptp_example.toml
./build/apps/x7_track_sim --config config/track_example.toml --check
./build/apps/x7_track_sim --config config/track_example.toml \
  --motion /tmp/track-sim.zvs --log /tmp/track-sim.csv
```

A manually taught ZVS can be selected with `--reference`. Use the same
filtering decision made during the teaching review, commonly `--filter
low-pass` for raw taught motion. Synthetic planner output normally uses
`--filter none`.

Deliberately slow the reference without changing its path:

```sh
./build/apps/x7_track_sim --config config/track_example.toml \
  --playback-rate 0.5 --bundle /tmp/track-half-speed
```

A playback rate in `(0, 1]` multiplies reference velocity by the rate and
acceleration by its square. The total tracking duration is divided by the
rate. `--kp`, `--kd`, `--effort-limit-nm`, `--filter`, and `--interpolation`
are also shared by both frontends.

Both frontends first move to reference frame zero in servo position mode. The
hardware frontend settles there, stops the command thread, calculates gravity
current at that stationary posture, and performs the reviewed position to
current transition with the preload already staged. Tracking begins on fresh
current-mode feedback. No mode change occurs during motion.

Tracking error is assessed separately from execution status. The result stores
aggregate, per-joint RMS, and peak errors plus `tracking_pass`. Missing the
assessment thresholds does not abort or make an otherwise clean invocation
fail. The larger hard-error envelope, position gates, command rejection,
watchdogs, I/O checks, and verified shutdown remain active.

The hardware app is intentionally experimental and has not been validated on
the renewed path. The plain law was unstable in the historical campaign. Keep
the actuator cutoff in reach, use a short slow reference, inspect the complete
simulation first, and support the arm before ending the final hold.

```sh
./build/apps/x7_track --config config/track_example.toml \
  --playback-rate 0.5 --check
./build/apps/x7_track --config config/track_example.toml \
  --playback-rate 0.5 --bundle /tmp/track-campaign/hardware
```

Create the matching simulation at `/tmp/track-campaign/simulation` with the
same configuration and overrides. Do not run these commands back-to-back
without inspecting the simulation and preparing the hardware workspace.

Archive simulation and hardware runs beneath a common campaign directory, then
compare them:

```sh
uv run --project tools tools/plot/track_comparison.py /tmp/track-campaign \
  --output /tmp/track-campaign/comparison.png \
  --summary-csv /tmp/track-campaign/comparison.csv
```

The directory form expects `simulation/simulation.csv` and
`hardware/hardware.csv`. Two explicit CSV paths can be supplied instead.

`x7_track_sim` also accepts an optional `[simulation] link_force` spec, a
prescribed external force started at the first tracking cycle, after the
current-mode switch. See
[link-force disturbances](#link-force-disturbances-and-the-force-budget-screen).

### Manual motion teaching

`x7_teach` records a trajectory while an operator guides the arm. It supports
two acquisition modes:

- `torque-off` leaves every servo disabled and reads position, velocity, and
  current directly. This is the default and should be tried first.
- `gravity-compensation` uses the reviewed `x7_float` activation, calibrated
  gravity preload, soft-limit start gate, current-mode control cycle, servo
  watchdog, and host deadman. It requires the approved vendor-scale hardware
  configuration.

Both modes completed successfully on the physical arm 2026-08-28. The
recorded workflow was then exercised through dynamics simulation and
position/CBP hardware playback. This establishes the app workflow, not the
safety of an arbitrary taught path: every new recording still requires visual
inspection, simulation, preflight, and the normal hardware precautions.

Validate the configuration without opening the bus:

```sh
./build/apps/x7_teach --config config/teach_example.toml --check
```

For a torque-off recording, support the arm throughout the run. The first
Enter starts recording and the second Enter stops it:

```sh
run_stamp=$(date +%Y%m%d-%H%M%S)
./build/apps/x7_teach --config config/teach_example.toml \
  --mode torque-off --bundle "/tmp/teach-torque-off-${run_stamp}"
```

Gravity compensation adds a separate support confirmation. Keep supporting
the arm during activation, press Enter to start, guide the arm and press Enter
to stop, then support it from below and press Enter once more to disable
torque:

```sh
run_stamp=$(date +%Y%m%d-%H%M%S)
./build/apps/x7_teach --config config/teach_example.toml \
  --mode gravity-compensation \
  --bundle "/tmp/teach-gravity-${run_stamp}"
```

The recording stops automatically at `recording.max_duration_s` if the second
Enter is not received. In gravity-compensation mode, torque remains active
after that stop until the separate support Enter or
`finalization.operator_timeout_s`. Activation, start wait, recording, and
support wait share a hard 60 second torqued-session ceiling. A timeout or
unclean shutdown makes the command fail.

The hardware acquisition loop always reads at 100 Hz. The CSV retains those
raw samples across the complete session, including the start wait and the
gravity-mode support wait. It records phase transitions, measured state, and,
when gravity compensation is active, submitted and applied torque command
metadata. `recording.sample_rate_hz`, or `--sample-rate`, selects the uniform
1 to 100 Hz ZVS output rate. The app linearly resamples only the interval
between the start and stop events and does not smooth it. Use the filtering and
interpolation settings in `x7_follow` when preparing a manually taught
reference for replay.

Standalone output files are created exclusively and never overwritten:

```sh
./build/apps/x7_teach --config config/teach_example.toml \
  --output /tmp/pick-taught.zvs --log /tmp/pick-taught.csv \
  --sample-rate 50 --max-duration 20
rk_anim models/crane_x7/crane_x7.ztk /tmp/pick-taught.zvs
```

`--bundle` is mutually exclusive with `--output`, `--log`, and `--check`.
Its target must not exist and is refused before the source configuration is
loaded. A successful bundle contains the source and effective TOML files, the
hardware configuration, model and meshes, `trajectory.zvs`, `recording.csv`,
`result.toml`, and a SHA-256 manifest. The effective `teach.toml` uses paths
inside the bundle, so its configuration and model can be checked later with:

```sh
./build/apps/x7_teach --config /tmp/teach-torque-off-STAMP/teach.toml --check
rk_anim /tmp/teach-torque-off-STAMP/model/crane_x7.ztk \
  /tmp/teach-torque-off-STAMP/trajectory.zvs
```

To replay a taught motion, point `reference` in a copy of
`config/follow_example.toml` at the new ZVS. Run `x7_follow_sim` and inspect its
full-session log before considering hardware. A taught path records what the
operator did; it does not provide collision, speed, acceleration, or endpoint
safety certification.

The complete two-mode procedure, including immutable campaign storage, is in
[teaching playback and servo-mode comparison](teaching-playback.md).

### Servo-side trajectory following

`x7_follow_sim` and `x7_follow` run the same phase controller against the
dynamics simulator and the real arm. They delegate the feedback loop to each
Dynamixel servo. The host sends position or current-based-position commands
without a host-side tracking correction during the reference phase. This is
separate from the parked computed-torque `x7_track` app. Velocity mode is not
offered because replaying a position reference through Goal Velocity requires
a host-side position loop.

For a command-by-command position-versus-CBP campaign using a planned or
manually taught reference, see
[teaching playback and servo-mode comparison](teaching-playback.md).

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

To compare archived position and current-based-position hardware runs, place
the bundles under `position/` and `cbp/` in one directory, then run:

```sh
run_dir=/tmp/follow-mode-hw-20260827-155637
uv run --project tools tools/plot/follow_mode_comparison.py "$run_dir" \
  --output "$run_dir/comparison.png" \
  --summary-csv "$run_dir/comparison.csv"
```

The comparison uses only tracking-phase samples, excluding home positioning
and the final operator hold. It prints aggregate and per-joint RMS and peak
errors, cycle-period statistics, rejected command receipts, clamp/gate events,
and command-application lag. The figure shows the shared reference with both
measured responses, together with per-joint error traces and RMS and peak bars.
Two CSV paths can be supplied instead of the common bundle directory. A
confirmed CBP `abort_hold` can be examined with the explicit
`--failure-analysis` shared-prefix mode documented in
[Teaching playback and servo-mode comparison](teaching-playback.md#simulate-both-playback-modes).

The checked-in `config/follow_example.toml` documents every option except the
servo-like `[simulation]` keys, which `config/follow_sim_servo_like.toml`
shows (see [servo-like simulation](#servo-like-simulation)). Input paths are
relative to the TOML file that names them. Output paths are
invocation-relative and are created exclusively; an existing file is refused.
The main processing controls are:

- `control.rate_hz`: one 20 to 200 Hz cycle rate for every phase. Hardware
  timing has been validated at 100 Hz; other rates print an experimental-use
  warning.
- `control.mode`: `position` or `current-based-position`. Current-based
  position requires one or eight positive
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
- `simulation.link_force`: an optional prescribed link-force disturbance for
  `x7_follow_sim`; the hardware frontend ignores it. See
  [link-force disturbances](#link-force-disturbances-and-the-force-budget-screen).
- `simulation` servo keys: the simulated servo's gains, and optionally a
  command delay, encoder quantization and joint friction. See
  [servo-like simulation](#servo-like-simulation).
- `learning`: an optional learned command offset added to the tracking
  command. See [learning between follow trials](#learning-between-follow-trials).

This first version deliberately uses one homogeneous motor mode for home,
tracking, and finalization. Changing a Dynamixel operating mode requires
torque to be disabled. A mode change at the home-to-tracking boundary would
therefore introduce an unsupported gravity-drop interval. Supporting different
phase modes requires a separately reviewed in-place transition protocol.

The simulation converts position commands to bounded PD torque, then integrates
the robot dynamics. Current-based position uses the same position adapter with
the requested effort ceiling. These adapters verify phase logic,
interpolation, safety gates, and command saturation. They are not a
high-fidelity model of the Dynamixel firmware, current loop, friction,
backlash, or gear compliance, and the servo-like settings below do not make
them one.

#### Servo-like simulation

By default every simulated servo is the same stiff spring: `position_kp =
1000.0` Nm/rad and `position_kd = 5.0` Nm·s/rad on every joint, each command
applied as soon as it is written, and exact feedback. That servo leaves almost
no tracking error. Optional `[simulation]` keys make the simulated servo
softer, slower and coarser, and can add joint friction:

| Key | Default | Accepted values |
|---|---|---|
| `position_kp` | `1000.0` | one number for every joint, or an array of 8 in canonical order; each > 0 [Nm/rad] |
| `position_kd` | `5.0` | the same two forms; each >= 0 [Nm·s/rad] |
| `coulomb_friction_nm` | absent (off) | one number or 8; each >= 0 [Nm] |
| `viscous_friction_nm_s` | absent (off) | one number or 8; each >= 0 [Nm·s/rad] |
| `friction_v0_rad_s` | `0.05` | a number > 0 [rad/s] |
| `command_delay_cycles` | `0` | a TOML integer >= 0 |
| `quantize_encoder` | `false` | a TOML boolean |

Every value must be finite, an array needs exactly eight entries, and each key
takes only its own TOML type: `quantize_encoder = 1` and
`command_delay_cycles = 1.0` are refused. When `position_kp` or `position_kd`
is an array it replaces the scalar for every joint; entry 7 is the gripper
servo, whose torque is split over both fingers.

- **Friction** adds `-fc * tanh(v / v0) - fv * v` to each joint's torque
  before the effort clamp, with `fc` from `coulomb_friction_nm`, `fv` from
  `viscous_friction_nm_s` and `v0` from `friction_v0_rad_s`.
- **`command_delay_cycles`**: the command accepted at cycle `k` reaches the
  simulated servo at cycle `k + command_delay_cycles`. Until the first one
  arrives, the arm holds its activation posture.
- **`quantize_encoder`**: the feedback position is rounded to the 4096-count
  encoder, and the feedback velocity is a servo-style estimate (a first-order
  lag of 0.12 s, rounded to the servo's velocity unit of 0.229 rpm). The
  simulated servo itself still acts on the exact state.

The integrator is explicit, so the integration step bounds the usable gains:
per joint, keep `sqrt(kp / I) * integration_step_s` and
`(kd / I) * integration_step_s` well under 2, with `I` the 0.05 kg·m² reflected
inertia every simulated joint carries, and with friction also
`(fc / v0 + fv) / I * integration_step_s`.

`config/follow_sim_servo_like.toml` sets these keys for the learning study.
Its purpose is a tracking error of realistic size and structure (a spring
deflection under the required torque plus a lag behind the reference) in
simulation, where the default servo leaves almost nothing to learn. It is a
servo-LIKE setting derived from the stiffness and lag measured on the arm (see
the theory note's [evidence](../theory/learning-control.md#evidence)), not a
calibrated twin of the arm: one round stiffness per servo model (20 Nm/rad on
the XM430 joints, 60 Nm/rad on joint 1, the XM540), damping fixed by
`kd / kp = 0.04 s`, a one-cycle command delay, encoder quantization, and no
friction. A learned offset does not transfer from this simulation to the arm.
It replays the PTP example, which must be generated first:

```sh
./build/apps/x7_plan_ptp --config config/ptp_example.toml
./build/apps/x7_follow_sim --config config/follow_sim_servo_like.toml \
  --log servo-like.csv --motion servo-like.zvs
```

The following are properties of the simulator, measured on its own logs; they
say nothing about the arm:

- The lag a fit recovers from a servo-like log (the lag-stiffness fit of
  [learnability analysis](#learnability-analysis-of-follow-logs), for example)
  is `kd / kp` plus `command_delay_cycles + 1` control cycles: about 60 ms for
  the shipped file.
- At startup the arm holds its activation posture until the first delayed
  command arrives. In the log, `applied_seq` trails `submitted_seq` by
  `command_delay_cycles + 1`, since each row's feedback is read before that
  cycle's command is written.
- `quantize_encoder` changes the logged position and velocity. The logged
  torque stays the exact servo torque, `kp * (u - q) - kd * dq` on the exact
  state, without friction, disturbance, or the simulator's numeric damping.
- The simulator adds a numeric damping of 0.05 times each link's inertia
  divided by `integration_step_s`. At the default step it is comparable to the
  configured `kd` on some joints, so changing `integration_step_s` changes the
  effective damping; keep the step fixed across runs that are compared.
- With soft gains the arm sags under gravity: homing corrects the sag by
  moving the home goal (home correction), and plain tracking then commands
  `q_d(0)` and drops that correction, so the elbow sags by tens of mrad in the
  first tracking cycles. In a learning run the `offset_ramp` phase handles
  this (see [learning between follow trials](#learning-between-follow-trials)).
- A short, small reference identifies the stiffness poorly: the fit needs a
  reference that varies the joint torques, and the 2 cm PTP example barely
  does.

A bundle (`--bundle`) of a servo-like run records these keys in its portable
`follow.toml`: the gains in the form given (one number or eight), and the
friction, delay and quantization keys whenever they are set. Replaying that
`follow.toml` reproduces the bundle's log and motion byte for byte. The hardware frontend `x7_follow` ignores
`[simulation]`.

#### Motor parameters and hardware run

An optional `motor_parameters` input accepts the versioned TOML produced by
`dxl_inspect dump-params`. Keep only the parameters reviewed for this run and
remove `operating_mode`: `control.mode` owns that register. Unknown motor IDs
are refused before bus contact. During activation the batch is applied and
read back transactionally while every servo is torque-off; incomplete rollback
aborts activation. Activation then checks Drive Mode again and refuses a
`drive_mode` with bit 3 (Torque On by Goal Update) set, since its goal snap
would switch torque on early. The bundle copies this exact parameter input.

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

The run ends with the background thread's cycle statistics: the cycle,
overrun, stale and deadline-miss counts, a `hardware faults:` line (read and
write failures, maximum lateness, maximum feedback age at submission), and
`host-measured phases:` quantiles of each phase of the cycle. `--timing-log
FILE` (hardware only, not with `--bundle`; `x7_track` accepts it too) also
writes one CSV row of host stamps per background cycle, drained off the control
threads; an incomplete timing log fails the run. The `cycle` column numbers the
rows 1, 2, ... over the whole run, and `segment` counts background-thread runs:
`x7_track` restarts its thread at the mode switch, so its log has two.
`deadline_miss` marks a window that closed with no submission while a
controller was armed. After the arm is released the app also writes
`FILE.summary.toml`, which holds the run status, whether the log is complete,
and the cycle counters both when the controller finished and after the thread
stopped (an existing file is refused before the bus is touched). The
stamps are host
observations: `read_start_ns` is when the host began the feedback transaction,
not when the servos sampled, and `write_return_ns` is when the host's write
returned, not when the bytes left the adapter.

`--hold-current SECONDS` and `--sweep-current JOINT:AMPLITUDE:DURATION`
(hardware only; not with `--reference`, `--check`, or `--bundle`) replace the
configured reference with one built from the posture the arm is resting in.
The posture is read torque-off before activation. The hold keeps it; the sweep
moves one canonical arm joint (0 to 6) out by AMPLITUDE radians and back along
two minimum-jerk halves of DURATION/2 seconds each. Both references start
exactly at the measured posture, so the run never homes or corrects: there is
no homing or home-correction cycle, and every command is the reference itself.
It refuses before any motion command if the arm has moved more than 0.005 rad
from that posture by the time tracking would begin (status `home_refused`),
and it aborts tracking once any joint is 0.05 rad from the reference (status
`posture_departure`; a provisional bound, tighter than the general `[safety]`
gates). That check runs on each feedback sample before the reference is
commanded from it: the cycle whose feedback departed is answered with a hold at
that sample's posture, never with the reference. The generated
reference is written to `<log>.reference.zvs` (an existing file is refused) and
validated like any reference before torque-on. It requires `filter = none`,
since a filtered reference would no longer start at the posture. These modes
let a new control rate be exercised without a multi-joint homing motion: hold
first, then a wrist-only sweep (`--sweep-current 6:0.3:6`).

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
the optional link-force spec and learned offset, the frontend outputs,
`result.toml`, and a SHA-256 `manifest.toml`. Normal
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

### Learnability analysis of follow logs

Two offline tools turn follow logs into the evidence used by the
[position-based learning control](../theory/learning-control.md) note:
how the tracking error is structured, how repeatable it is, and how much
of it a feedforward built only from the reference predicts. Neither opens
the bus. The numbers they produced on archived hardware logs are in the
note's [evidence](../theory/learning-control.md#evidence) section.

`x7_model_torque` evaluates the rigid-body model along a table of joint
trajectories. It selects the reference columns by name, in any order:
`qrefN_rad`, `dqrefN_rad_s` and `ddqrefN_rad_s2` for all eight canonical
joints. Other columns, numeric or not, are ignored, so a follow log can be
passed as it is:

```sh
./build/apps/x7_model_torque --model models/crane_x7/crane_x7.ztk \
  --input follow-hw.csv --output follow-hw-torque.csv
```

For every input data row the output holds `schema_version`, `row` (the
0-based input row), the model gravity torques `gravity0_nm` to
`gravity7_nm`, and the inverse-dynamics torques `inverse_dynamics0_nm` to
`inverse_dynamics7_nm` (which include gravity). The output must not exist:
it is created exclusively, an existing file is refused, and a partial file
is removed on failure. Every input row needs the header's field count and
finite numbers in the selected columns; an error names the row and column.

`tools/follow_learnability.py` analyses the tracking rows of follow logs
(schema version 1; one contiguous block of `tracking` rows). It uses the
log's sign, `e = q - qref` from the `errorN_rad` columns, which is the
opposite of the theory note's (see its
[sign convention](../theory/learning-control.md#the-servo-as-a-spring-with-lag)).
It has three subcommands; each one's `--help` documents its fits and
units.

`anatomy` analyses each log independently: per joint, the RMS, peak and
mean error, the error above a band split (`--split-hz`, default 3 Hz), the
reference speed and acceleration at the first and last tracking sample (a
rest-to-rest check), a lag-stiffness fit `e = -tau/K - T*dqref + c` on the
log's torque, and a servo-law fit `tau = Ks*(applied - q) - Ds*dq + b`:

```sh
uv run --project tools tools/follow_learnability.py anatomy \
  follow-hw-a.csv follow-hw-b.csv
```

`repeat` takes at least two runs of the same reference (tracking lengths
within one sample, identical references; otherwise it exits naming the
logs) and separates the repeatable error (the mean over runs, raw and
debiased) from the non-repeatable rest, below and above the band split,
together with the spread of the initial state:

```sh
uv run --project tools tools/follow_learnability.py repeat \
  follow-rep-1.csv follow-rep-2.csv follow-rep-3.csv
```

`predict` tests a feedforward built only from the reference, on the arm
joints, by leaving one run out: weights fitted on the other runs predict
the held-out run's error. The runs are named logs, at least two, intended
to have different references. The tool smooths each reference
(`--smooth-sigma-s`, default 0.04 s), differentiates it, and calls
`x7_model_torque` for the model terms; the regressors are those of the
note's feedback-error-learning table (Coulomb scale `--coulomb-v0`,
default 0.05 rad/s), with two reduced sets reported alongside. Read the
held-out residual, not the individual weights:

```sh
uv run --project tools tools/follow_learnability.py predict \
  --model-torque ./build/apps/x7_model_torque \
  --model models/crane_x7/crane_x7.ztk \
  --run a=follow-hw-a.csv --run b=follow-hw-b.csv --run c=follow-hw-c.csv
```

With `--json FILE`, any subcommand also writes its results as JSON in SI
units, with null for every value that is not identifiable; the tables on
standard output show errors in mrad and times in ms. The exit status is 0
on success, 1 on invalid input (the reason on standard error), and 2 on a
usage error.

### Learning between follow trials

The follower can command the reference plus a learned position offset,
`q_d + delta`, read from an offset file. The offset is computed offline between
trials, from the previous trial's log, by `x7_follow_learn`; the follower only
reads and checks it and never computes or changes one. The schemes, their sign
convention and their limits are in the
[position-based learning control](../theory/learning-control.md) note.

This section is the workflow in simulation, with `x7_follow_sim`. Applying a
learned offset on the physical arm is not approved yet (see the note's
[comparison](../theory/learning-control.md#comparison)).

#### The follower with an offset

Name the offset in the follow configuration:

```toml
[learning]
offset = "offset_1.csv"      # relative to this config
max_offset_rad = 0.15        # |delta| on every joint and sample
max_offset_rate_rad_s = 0.5  # |delta[i+1] - delta[i]| / dt
```

or on the command line, which overrides `[learning] offset`:

```sh
./build/apps/x7_follow_sim --config config/follow_sim_servo_like.toml \
  --offset offset_1.csv --check
```

`[learning] offset` is resolved against the configuration file's directory,
like the other inputs; `--offset FILE` is resolved against the invocation
directory, like every command-line path. The two limits default to the values
shown (0.15 rad is also the home-correction clamp) and can only be lowered: a
value that is not positive or exceeds the default is refused, and so is a rate
limit below 0.01 rad/s. The `offset_ramp` phase below moves at the rate limit,
and the floor bounds its length (at 0.01 rad/s the widest ramp it can be asked
for, the 0.15 rad home-correction clamp plus a 0.15 rad offset, takes 30 s).

The preflight reads files only and runs before any simulation step or bus
contact, `--check` included. It reads the offset file with the strict reader
described [below](#the-offset-file), hashing it before and after the read (a
file that changed in between is refused), and accepts it only for this run:

- its grid must be the tracking grid of the loaded reference at
  `control.rate_hz` (the same sample count, the step within 1e-12 s);
- its `reference_sha256` must be this run's
  [reference signature](#the-reference-signature), computed from the
  configuration after the command-line overrides;
- every value must be within `max_offset_rad`, and every change from one
  sample to the next within `max_offset_rate_rad_s` times the step;
- every command it produces, `q_d(t_i) + delta[i]` at each tracking sample and
  `q_d(T) + delta[N-1]` for the final hold, must lie inside the model's joint
  limits.

A refusal names the problem; a limit violation names the joint, sample, time
and value, and a signature mismatch lists what the signature covers. The
reference check and this one validate exactly the samples the follower will
command (one grid definition serves the follower, its preflight and
`x7_follow_learn`), and a reference whose grid cannot be built at
`control.rate_hz` is refused before any bus contact, as is a posture run's.
An accepted offset prints one line, before the run or before `--check`'s
`configuration and reference preflight passed`:

```text
command offset accepted: method METHOD, trial K, max |delta| X rad, sha256 HASH
```

With an offset, the run gains one phase and two commands change:

- **`offset_ramp`**, between homing and tracking: the command moves from the
  last homing command (the home goal, any home correction included) to
  `q_d(0) + delta[0]`, the first tracking command, in equal steps of at most
  `max_offset_rate_rad_s` times the control period on every joint, and then
  holds there for `home.settle_time_s`. It lasts the largest joint distance
  divided by the rate limit, rounded up to whole cycles, plus that hold (only
  the hold when the distance is zero). Its logged reference is `q_d(0)` at
  rest, and, as in homing, no tracking gate is evaluated. The phase runs for
  every offset, an all-zero one included, so that every trial of a series
  follows the same procedure. A run with a zero offset therefore differs from
  a run without one: it has this phase, and it removes a home correction
  gradually instead of in the first tracking command.
- **Tracking** sample `i` commands `q_d(t_i) + delta[i]`.
- **The final hold** commands `q_d(T) + delta[N-1]`: the last offset sample
  (the learned static compensation) stays applied while the logged reference
  is `q_d(T)`. Abort holds, a failure during the ramp included, hold the
  measured posture and carry no offset.

The log keeps its schema. `qrefN_rad` stays the reference, `qcmdN_rad` is the
command sent (reference plus offset; in `offset_ramp`, the ramp command), and
`errorN_rad` stays `q - qref`. The safety gates are unchanged and are judged on
the reference, never on the command. A reference built from the measured
posture (`--hold-current`, `--sweep-current`, both hardware-only) takes no
offset: the combination is refused before the bus is opened, since an offset
belongs to one reference and this one is made from the posture at run time.

A [bundle](#portable-follow-bundles) of a run with an offset archives the file
as `offset.csv`, writes a `[learning]` table (`offset = "offset.csv"` and both
limits) into the portable `follow.toml`, and adds three lines to
`result.toml`: `offset_method`, `offset_trial`, and `offset_sha256` (of the
file's bytes). The portable `follow.toml` keeps every setting the signature
covers, and the archived reference keeps its bytes, so replaying the bundle
accepts its offset.

#### The offset file

An offset is one self-contained text file: a `# key: value` preamble, a column
header, and one row per tracking-grid sample. `x7_follow_learn` writes it; a
hand-edited file is no longer what its lineage says produced it. The layout,
with each value's meaning in place of the value:

```text
# format: rtctrl-x7-follow-offset
# version: 1
# method: ilc
# trial: 1
# dt_s: <grid step>
# samples: <N>
# reference_duration_s: <T>
# reference_sha256: <reference signature of the run the offset is for>
# parent_sha256: <SHA-256 of the file this offset was updated from>
# source_log_sha256: <SHA-256 of the trial log the update read>
time_s,delta0_rad,delta1_rad,delta2_rad,delta3_rad,delta4_rad,delta5_rad,delta6_rad,delta7_rad
0.000000000,<delta0>,<delta1>,<delta2>,<delta3>,<delta4>,<delta5>,<delta6>,<delta7>
```

An empty value is written as `# key:` (no space after the colon), as in the
trial-0 file's `parent_sha256` and `source_log_sha256`. `method` is `zero`,
`ilc` or `fel`, and `trial` counts the learning updates applied. Sample `i` is
at `t = i * dt_s`; the grid is the follower's tracking grid, `max(1,
ceil(T * rate_hz))` intervals of `T / intervals`, so `dt_s` can differ from
`1 / rate_hz` in the last digits. `time_s` is written with `%.9f` and every
other number with `%.17g`, so a file reads back bit for bit.

The reader is strict. It requires the format and version; the ten preamble
keys in this order, each once; the exact column header; nine fields per row,
each a finite number; as many rows as `samples`; each `time_s` within 1e-6 s of
`i * dt_s`; and a final newline. It then checks the structure: at least two
samples, spanning `reference_duration_s` within 1e-9 s; every hash empty or 64
lowercase hex digits (the reference signature never empty); trial 0 if and only if
method `zero`, whose values must all be exactly zero and which has no parent
or source; and both a parent and a source for every later trial. Offset files
are created exclusively: an existing file or symlink is refused and left
untouched, and a partially written file is removed.

The three hashes are the file's lineage:

- `reference_sha256` names the run the offset belongs to: its
  [reference signature](#the-reference-signature), not the reference file's
  hash;
- `parent_sha256` names the file this offset was computed from: the previous
  offset for `ilc`, the weights file for `fel`;
- `source_log_sha256` names the trial log the update read.

A hash identifies a file but cannot recreate it. Keep every offset, weights
file, log and report of a series: the chain can be checked only by hashing the
kept files, and the next update needs the previous offset (for `fel`, also its
weights file) byte for byte.

#### The reference signature

The `reference_sha256` field of an offset file, and of a weights file, holds
the reference signature: the SHA-256 of a canonical text covering the
reference file's SHA-256, `control.rate_hz`, `control.mode` and every
`[reference_processing]` setting (interpolation, filter, cut-off, window and
Savitzky-Golay order, each whether or not the filter uses it). The follower
tracks the reference after its processing, on the grid of its rate, in its
mode; an offset learned under one of these settings is the wrong offset under
another, although the reference file is the same. The follower's preflight
and `x7_follow_learn` therefore refuse an offset whose signature is not the
run's.

- `x7_follow` and `x7_follow_sim` compute the signature from the
  configuration after their command-line overrides, so `--reference`,
  `--mode`, `--filter` and `--interpolation` change it whenever they change a
  covered setting, and the follower then refuses an offset learned without
  them. `x7_follow_learn` computes it from the follow configuration file
  alone. A learning series therefore keeps its settings in the follow
  configuration and keeps them fixed; changing any covered setting, an unused
  filter parameter included, starts a new series.
- Offset files written before the signature was introduced hold the
  reference file's hash in this field and are refused: restart such a series
  with `init`. A weights file's field only records the reference it was
  trained on last and is never compared, so `synth` still evaluates older
  weights, while continuing an older `fel` series fails at its offset.
- Not covered, by decision: the servo gains (the servos' registers, a
  `motor_parameters` file, or the simulation's `[simulation]` keys), the
  model, the current-based position effort limits, and the homing, safety and
  finalization settings, which do not change the tracked trajectory. Running
  an offset with the servo and model it was learned with is the operator's
  responsibility.

#### The learning tool

`x7_follow_learn` computes one update and never opens the bus. Each command
names its inputs and outputs explicitly (`x7_follow_learn COMMAND --help`
describes one); every output must not exist yet. The follow configuration
given with `--follow-config` supplies the reference exactly as the follower
loads it (model, reference file, `reference_processing`, `control.rate_hz`)
and, with `control.mode`, the reference signature; the tool has no override
options. `--offset` names the offset the trial ran with. When the follow
configuration also names a `[learning] offset`, `ilc` and `fel` refuse unless
that file has the same content (SHA-256) as `--offset`, since it would
otherwise be in doubt which of the two the trial ran with; a configuration
without one names nothing to compare (the trial may have used `--offset`).

`init` writes the trial-0 offset, all zeros on the follow configuration's
tracking grid:

```sh
./build/apps/x7_follow_learn init --follow-config follow.toml \
  --output offset_0.csv
```

`ilc` reads a trial's log and the offset it ran with and writes the next
offset by iterative learning control (method `ilc`, trial plus one):

```sh
./build/apps/x7_follow_learn ilc --config learn.toml \
  --follow-config follow.toml --log trial_0.csv --offset offset_0.csv \
  --output offset_1.csv --report report_1.toml
```

`init-weights` writes trial-0 feedforward weights (all zero) with the learn
configuration's regressor options, the starting point of feedback-error
learning:

```sh
./build/apps/x7_follow_learn init-weights --config learn.toml \
  --output weights_0.toml
```

`fel` reads a trial's log, the offset it ran with and the weights that offset
was made from, and writes the next weights and the offset they give on the
same reference (method `fel`):

```sh
./build/apps/x7_follow_learn fel --config learn.toml \
  --follow-config follow.toml --log trial_0.csv --offset offset_0.csv \
  --weights weights_0.toml --output-weights weights_1.toml \
  --output offset_1.csv --report report_1.toml
```

`synth` evaluates trained weights (trial 1 or later) on the reference of
another follow configuration and writes that offset:

```sh
./build/apps/x7_follow_learn synth --config learn.toml \
  --weights weights_5.toml --follow-config other_follow.toml \
  --output other_offset.csv
```

The learn configuration (format `rtctrl-x7-follow-learn`, version 1) holds the
offset limits and the gains of both update laws.
`config/follow_learn_example.toml` lists every key with its default; an absent
key keeps the default, and unknown keys, wrong types and values out of range
are errors that name the key:

| Key | Default | Meaning |
|---|---|---|
| `limits.max_offset_rad` | `0.15` | bound on every value the tool writes [rad]; may only be lowered |
| `limits.max_offset_rate_rad_s` | `0.5` | bound on `abs(delta[i+1] - delta[i]) / dt` [rad/s]; may only be lowered, not below 0.01 |
| `ilc.phi` | `0.0` | position-error gain, dimensionless, in [0, 2) |
| `ilc.psi_s` | `0.0` | velocity-error gain [s], in [0, 0.1] |
| `ilc.advance_s` | `0.05` | time advance: the error at `t + advance_s` updates `delta(t)` [s] |
| `ilc.cutoff_hz` | `2.0` | -3 dB point of the zero-phase low-pass; at most 0.8 of the Nyquist frequency |
| `ilc.max_trial_step_rad` | `0.05` | clamp on the change of each sample per update [rad] |
| `fel.step` | `"normalised-gradient"` | or `"least-squares"` |
| `fel.gain` | `0.2` | learning gain, in (0, 1] |
| `fel.advance_s` | `0.05` | pairs the regressors at `t` with the error at `t + advance_s` [s] |
| `fel.error_lead_s` | `0.0` | error signal `e + error_lead_s * de/dt` [s], >= 0 |
| `fel.error_smooth_sigma_s` | `0.04` | zero-phase Gaussian smoothing of the error [s] (0 = none) |
| `fel.ridge` | `1e-6` | least-squares penalty on the RMS-normalised columns |
| `fel.regressors.smooth_sigma_s` | `0.04` | zero-phase Gaussian smoothing of the reference [s] (0 = none) |
| `fel.regressors.coulomb_v0_rad_s` | `0.05` | speed scale of the regressor `tanh(dq / v0)` [rad/s], > 0 |

`phi` and `psi_s` are one number for the seven arm joints, or an array of all
eight canonical joints whose gripper entry (the last) is 0: the gripper is
never learned. ILC is off until they are set: with both 0 on every joint (the
default), `ilc` refuses to run, and a joint with both 0 is passed through
unchanged. The ceilings come from the theory note: `phi < 2` is the
low-frequency contraction condition `0 < Φ < 2` of the
[sampled law](../theory/learning-control.md#what-100-hz-sampling-adds), and
`psi_s <= 0.1 s` caps the velocity gain, which the note's
[gain condition](../theory/learning-control.md#conditions-the-papers-assume)
bounds by `2 K^-1 D`, less than twice the measured lag. Within the ceilings
the usable gains are still to be found in simulation. The ILC update clamps each sample's change to `max_trial_step_rad`
and its value to `limits.max_offset_rad`, and the summary reports how many
samples each clamp changed. Feedback-error learning always updates arm joints
0 to 6; a regressor column with an RMS below 1e-6 over the trial is skipped
and its weight kept (the gravity column of joint 0, which turns about the
vertical axis, is one). The weights file records the regressor options, and
`fel` and `synth` refuse weights learned with options other than the learn
configuration's `[fel.regressors]`. The learn configuration's `[limits]` are
independent of the follow configuration's `[learning]` limits: keep the two
equal, or the tool can write an offset that the follower then refuses.

Before it writes anything, the tool checks:

- that every output path is non-empty, distinct from the others, and absent;
- that no input changes while it is read (each input is hashed before and
  after reading);
- that the input offset belongs to this run (the follower's run check, the
  reference signature included, under the learn configuration's limits) and
  leaves the gripper at zero;
- for `ilc` and `fel`, that a `[learning] offset` named by the follow
  configuration has the content of `--offset`;
- for `fel`, that the input offset was made from the input weights: the same
  trial, and from trial 1 on method `fel`, the weights file's hash as parent,
  the same source log, and values equal to the weights' synthesis on this
  reference within 1e-9 rad;
- that the log belongs to this reference and this offset: in every tracking
  row, `qref` equals the reference sample and `qcmd - qref` equals the offset
  sample, each within 1e-6 rad, at the grid time within 1e-6 s;
- that the trial completed: no `abort_hold` row, exactly one contiguous block
  of as many `tracking` rows as the grid has samples, preceded only by `home`,
  `home_correction` and `offset_ramp` rows and followed only by `finalizing`
  rows, every one of them with `receipt_accepted = 1`;
- that the final hold is long enough: the tool reads at most the first 200
  `finalizing` rows (checked like the tracking rows, against `q_d(T)` and the
  last offset sample), and the update needs at least `advance_s / dt`,
  rounded up, of them (5 at the default advance at 100 Hz). The error is
  filtered over the tracking and hold rows together, so keep `[finalization]`
  the same throughout a series.

An update whose result violates the limits (its value or its rate) is refused
with a message naming the joint, sample and value, and nothing is written: the
offset is never clipped to fit. (The ILC clamps above belong to the update law
and act before this check; an ILC result can still fail the rate limit, and
the message then suggests a lower `cutoff_hz` or lower gains.) The written
offset is read back and must match bit for bit; any failure after the first
output removes every output already written. A run without an offset passes
as the trial of a zero offset (its commands are the reference), but run trial
0 with the zero offset file, so that every trial of the series has the same
phases.

On success the tool prints each output with its SHA-256, a per-joint table
(tracking RMS of the measured trial, offset RMS and peak, and the ILC clamp
counts or the feedback-error eigenvalue and skipped columns), and the
aggregate tracking RMS of the trial it read:

```text
tracking RMS of trial K: X rad over all 8 joints, Y rad over arm joints 0-6
```

`K` is the input offset's trial number. Over a series, these lines are the
learning curve. `--report FILE` (for `ilc` and `fel`) writes the same numbers
as TOML (format `rtctrl-x7-follow-learn-report`; the `[tracking]` table's
`aggregate_rms_rad` and `arm_rms_rad`, the per-joint tracking and offset
values, the `[ilc]` or `[fel]` diagnostics), together with the path and SHA-256
of every input and of the outputs written before it.

The tool's error is `e = qref - q`, the negative of the log's `errorN_rad`
column: the theory note's sign (see its
[sign convention](../theory/learning-control.md#the-servo-as-a-spring-with-lag)).
With positive gains, the next offset rises where `q < qref` (`e > 0`).

#### A learning loop in simulation

Each trial runs with the offset of the previous update and logs to a name of
its own; the update then reads that log and that offset:

```sh
follow=config/follow_sim_servo_like.toml  # the reference and the servo
learn=learn.toml        # a copy of config/follow_learn_example.toml, phi set
run=learning-series     # one new directory per series
mkdir "$run"
./build/apps/x7_follow_learn init --follow-config "$follow" \
  --output "$run/offset_0.csv"
for k in 0 1 2 3 4 5 6 7; do
  ./build/apps/x7_follow_sim --config "$follow" \
    --offset "$run/offset_$k.csv" \
    --log "$run/trial_$k.csv" --motion "$run/trial_$k.zvs" || break
  ./build/apps/x7_follow_learn ilc --config "$learn" \
    --follow-config "$follow" --log "$run/trial_$k.csv" \
    --offset "$run/offset_$k.csv" --output "$run/offset_$((k + 1)).csv" \
    --report "$run/report_$((k + 1)).toml" || break
done
grep -H '^aggregate_rms_rad' "$run"/report_*.toml
```

Give the trial and the update the same follow configuration and keep it fixed
through the series. The follower refuses the trial's offset when
`--reference`, `--filter`, `--interpolation` or `--mode` changes a setting the
[reference signature](#the-reference-signature) covers, so such settings
belong in the follow configuration. For feedback-error learning, start from
`init-weights` as well and replace the `ilc` line by `fel` with
`--weights "$run/weights_$k.toml"` and
`--output-weights "$run/weights_$((k + 1)).toml"`.

The integration test `x7_follow_learn_loop_smoke`
(`tests/integration/follow_learn_loop_smoke.sh`) runs this loop end to end:
four ILC trials of the pick example in the servo-like simulation, checking
that the follower accepts each offset the tool wrote, that each offset's
lineage and signature chain to the previous trial's, and that the aggregate
tracking RMS falls at every trial. Run it with
`ctest --test-dir build -R x7_follow_learn_loop_smoke`.

#### Two schemes

- **ILC** (`ilc`) learns the offset signal itself, sample by sample, for one
  reference. Each offset's parent is the previous offset.
- **Feedback-error learning** (`fel`) learns weights of six regressors per arm
  joint evaluated on the reference (`g`, `tau_dyn`, `ddq`, `dq`, `coulomb`,
  `constant`; the note's
  [regressor table](../theory/learning-control.md#feedback-error-learning-in-position-form)).
  Each update writes the next weights and realises them as an offset on the
  same reference; that offset's parent is the weights file. `synth` applies
  trained weights to another reference: the transfer test. The scheme removes
  only what the regressors can express.

#### Limitations

- An offset is valid only for the run it was learned for: the reference file,
  its processing, the control rate and mode (together, the
  [reference signature](#the-reference-signature)) and the tracking grid. The
  follower and the tool refuse it for any other run.
- Nothing binds an offset to the servo gains, the model, the current-based
  position effort limits, or the homing, safety and finalization settings.
  Keeping those as they were when the offset was learned is the operator's
  responsibility.
- The weights record their regressor options and lineage, not the model or the
  servo gains: `synth` evaluates them with the model of the follow
  configuration it is given, and nothing checks that the servos are the ones
  the weights were learned with. The report records the hashes of the follow
  configuration and the model each update used.
- The tool does not check feedback freshness. It reads `phase`,
  `phase_time_s`, `receipt_accepted`, `qref`, `q` and `qcmd`, so a stale
  feedback sample enters the update as it is.

### Link-force disturbances and the force-budget screen

These tools support disturbance experiments in which a force acts on the arm
during motion and is then removed, so that the baselines here and controllers
in dependent projects can be compared under one loading rule. They provide a
reusable simulated disturbance and an offline load screen. Physical apparatus
operation and fuse qualification are outside this repository.

#### Link-force specification

One `rtctrl-x7-link-force` TOML file describes the disturbance for the
simulators, the force-budget screen, and external code
(`model::loadLinkForce`). `config/link_force_example.toml`:

```toml
format = "rtctrl-x7-link-force"
version = 1
link = "crane_x7_tcp_link"
point_link_m = [0.0, 0.0, 0.0] # in the link's own frame
magnitude_n = 3.0
direction_world = [0.0, 1.0, 0.0] # or anchor_world_m = [x, y, z]

[profile]
onset_s = 0.4
rise_s = 0.2
hold_s = 0.6
fall_s = 0.2
```

- `direction_world` is a fixed world direction, normalized on use.
  `anchor_world_m` instead pulls the application point toward a fixed world
  point, which models a cable over a pulley. Exactly one is required.
- The amplitude is `magnitude_n` times a profile that is zero before onset,
  rises over `rise_s` with a raised cosine, holds, and falls over `fall_s` with
  a raised cosine. Both ramps have zero slope at their ends, so the force has
  no steps. `rise_s` and `fall_s` must be positive and all four profile keys
  are required.
- The profile clock starts at the first tracking cycle (the feedback time of
  that cycle's state) and runs in elapsed tracking time. It is not scaled by
  `playback_rate`, so compare runs only at the same playback rate and
  reference processing.

Every controller receives the same scalar force profile under the same loading
rule. In anchor mode the realized world vector and joint torques depend on
where each controller holds the attachment point, so they are logged rather
than assumed equal. A fixed world direction is the simpler first comparison.

#### Simulating a disturbance

Point the simulation table of a follow or track configuration at a spec:

```toml
[simulation]
link_force = "link_force_example.toml" # relative to this config
```

`x7_follow_sim` and `x7_track_sim` load the spec during `--check`, reject an
unknown link and an onset at or after the end of the run (tracking plus final
hold), and warn when the profile outlasts the run. The force is added to the
forward dynamics as `J^T f` after the actuator effort clamp, so it is never
truncated by the effort limits and the controller sees it only through motion.
`SimArm::setDisturbance`, the older constant joint torque, is applied before
that clamp instead.

Besides the main log, a run writes `<log stem>_disturbance.csv` (in a bundle,
`simulation_disturbance.csv`) with one row per simulator step attempt. Each row
records the attempt index, the step's `start_seq`/`end_seq` in the main log's
`feedback_seq` numbering, its sim-time bounds, the last completed substep's
force, application point and joint torques, and the force and joint impulses
of the completed substeps. Within a substep the force is constant at its value
at the substep start, so the impulses are exactly what the integrator applied.
A substep that contains the end of the rise, the end of the hold, or the
release ends there, so every phase of the profile is sampled even when it is
shorter than the integration step (`integration_step_s`). A pulse shorter than
one control period, or than one integration step, therefore still appears in
its row's impulse. A phase shorter than the integration step is applied as one
constant sample, though: the shape, and with unequal ramps the impulse, is
resolved only to the integration step.

Join the two logs by seq, never by timestamp. The main row with feedback seq
`s` joins the complete row with `end_seq = s` (the step that produced it) and
the row with `start_seq = s` (the step that followed it). Some rows are
unmatched by design:

- the initial main row has no preceding step;
- the final successful step has no later main row;
- a rejected step, and every `fault_latched` row after it, has no later main
  row;
- a track main row recorded without a step (an assessment abort) has no
  following step.

Zero-substep rows repeat their start time and seq. Keep the unmatched rows in
any analysis.

Once a spec is set, every substep is accounted strictly. When a nonzero force
is requested, an undefined direction (an anchor within 1 mm of the point) is
`anchor_fault` and a non-finite force is `non_finite_force`; for every
substep, a failed dynamics solve is `solve_failed` and a non-finite
acceleration or state update is `non_finite_state`. The update is checked
before the joint stops, which could otherwise clip an infinite position to a
finite limit. A zero requested force (before onset, after
release, or a zero magnitude) never evaluates the direction. A rejected
substep commits nothing: the state and the clock stay at the last completed
substep, and the simulator latches. It then refuses feedback and commands,
records `fault_latched` for further steps, and still deactivates. The run
fails with the status printed. Failing to write a disturbance row is
different: the physics stays complete and feedback stays valid, but the step
and the run fail because the log is incomplete.

`JointState::tau` in the main log keeps its meaning: the servo-equivalent
torque before the model-coordinate effort clamp. Under saturation it can
differ from the effort the dynamics used, so only the disturbance log is exact
about what was applied. The tracking-error columns measure error against the
time-indexed reference. Analysis of rejoining the path after a delay belongs
to the controller under study and must separate path deviation from delayed
progress.

Bundles archive the spec as `link_force.toml`, write
`link_force = "link_force.toml"` into the portable configuration, and add
provenance to `result.toml`: `link_force_enabled`, `link_force_started` (the
tracking start was reached), and `link_force_applied` (a completed substep
applied a nonzero force) with the first and last application times. A replay
reproduces all three outputs byte for byte. Hardware frontends ignore the key,
print a notice that it is simulation-only, and record
`link_force_enabled = false` with a note: an archived simulation spec is not
evidence that any apparatus applied a force.

#### Force-budget screen

`x7_force_budget` is a model-based screen with this contract: compute the
largest force amplitude satisfying user-supplied joint holding-torque
allowances at sampled reference poses under a quasi-static model. It is not a
hardware safety or fuse rating. It never opens the bus.

```sh
./build/apps/x7_plan_ptp --config config/ptp_example.toml
./build/apps/x7_force_budget --config config/force_budget_example.toml
```

At every sample (`N = ceil(duration * sample_rate_hz)` intervals, endpoints
included, one sample for a zero duration) it evaluates the gravity load `g`
at the reference pose and the joint torque per newton `a = J^T u` of the
spec's application point and direction. The bound requires feasibility for
every amplitude from zero up to it, `|g_i - a_i F| <= allow_i` for all
`0 <= F <= F_max`:

- a joint whose gravity load already exceeds its allowance is infeasible,
  even if some positive force would compensate it;
- otherwise the joint bound is `(allow_i + sgn(a_i) g_i) / |a_i|`, computed
  without avoidable overflow;
- `a_i = 0` imposes no bound from that modeled constraint (`inf`), which never
  means unlimited physical loading;
- a bound that cannot be represented, a non-finite pose, gravity, point, or
  torque coefficient, and a degenerate anchor are failed evaluations.

The amplitude bound is the minimum over joints and samples, reported with the
binding joint and the earliest binding time, plus a disturbance-only bound
that ignores gravity. The whole reference path is screened, and denser
sampling is not a continuous-path guarantee. The tool prints the bound, the
planned `magnitude_n`, and a headroom ratio. The ratio reads `n/a` when the
planned magnitude is zero, the bound is unbounded, or the evaluation failed.
Only a successful evaluation whose planned magnitude is within the bound
prints `within the supplied sampled model allowances` and exits 0. Exceeding,
infeasible, and failed evaluations exit 1 and still write the CSV. Invalid
input exits 1 without one. The CSV begins with `#` lines that carry the
contract and qualification, then one schema-versioned row per sample.

`budget.joint_allowance_nm` (one or eight values) is an engineering input
that this screen does not establish. Actuator effort limits, such as the
torque constant times the current limit or the ceilings derived for the EFL
study, describe actuator and model effort, not allowable external mechanical
loads. The screen excludes motion transients, off-path yielding of compliant
controllers, fuse release, friction, bearing and bracket loads, and the
individual finger-linkage loads behind the gripper's reduced torque. Selecting
a physical fuse, including its release tolerance and transient loading, is a
separate engineering decision.

#### Using a link force from your own controller

Dependent projects drive the same disturbance through the library. Set the
spec before the run, start its clock when tracking starts, and attach the
interval log:

```cpp
#include "rtctrl/arm/link_force_log.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/link_force.hpp"

const auto spec = model::loadLinkForce("config/link_force_example.toml");
arm::LinkForceCsvLog log("run_disturbance.csv");  // exclusive; outlives sim
arm::SimArm sim(options);
sim.setLinkForce(spec);    // strict accounting from here on, force still zero
sim.linkForceLogTo(&log);  // one row per step() attempt
sim.setMode(arm::ControlMode::Position);
sim.activate();
// ... at the first tracking cycle, with that cycle's feedback state:
sim.startLinkForce(state.t);
// ... after the run:
const bool clean = !sim.linkForceFaulted() && !sim.linkForceLogFailed() &&
                   log.close();
```

`model::evaluateForceBudget` and `model::writeForceBudgetCsv` expose the
screen in the same way.

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

`arm::run` returns false for several distinct reasons. Pass an
`arm::RunReport` as its last argument to learn which: it names the end
(`arm::describe(report.end)`: a failed read, a feedback gap or over-long
interval, a rejected command, an observer veto, a failed step, and so on),
the cycle, and the sequence numbers and timestamps of the last accepted and
the final sample. The report is filled during the run and read at its end,
so nothing is printed per cycle; `x7_wave` prints it on an abort.

`readState` gives positions, velocities, and torque estimates
($\hat\tau = k_t\,i$). On hardware it returns a fresh sample while that
sample's bounded command window is open; `step()` blocks until the next
successful feedback read and returns `false` after a safety escalation. The
shipped controllers `arm::GravityComp`, `arm::ComputedTorque`, and the retained
historical `arm::PracticalComputedTorque` follow
exactly this pattern; see the
[theory documents](../theory/gravity-compensation.md) and their
sources for worked examples.

### Logging from a control loop

Do not write files from `update()`, a `CycleObserver`, or anything else on the
control thread. A buffered write to a regular file can block in the kernel for
tens to hundreds of milliseconds (journal commits, device power states, other
processes' `fsync` calls; a 162 ms append was measured on the development
workstation). On hardware a 25 ms stall makes `arm::run` abort on the feedback
gap, and a 250 ms stall trips the submission deadman. Queue the rows through
`util::AsyncWriter` instead; every app run log uses it:

```cpp
#include "rtctrl/util/async_writer.hpp"

util::AsyncWriter log("run.csv");  // exclusive creation; starts its writer thread
log.appendf("t,q0\n");
log.commitRow();
// per cycle: format in memory and queue; the file is never touched here
log.appendf("%.4f,%.6f\n", t, zVecElemNC(state.q.get(), 0));
if (!log.commitRow()) { /* a row was dropped or a write failed */ }
// after the shutdown:
const bool log_ok = log.close();  // drains, flushes, closes
```

The queue is bounded (4 MiB by default, about 30 s of a 100 Hz log with
1.2 kB rows), so a commit never allocates or waits for the file. The writer
thread hands the queue to the file every 20 ms; that interval is nominal,
since a blocked write delays the next one. A row that does not fit, and any
failed write or flush, latches the writer: later rows are refused and
`close()` returns false, so a log is either complete or reported incomplete,
never silently gapped.

Failure detection is asynchronous and has no time bound. A write's failure is
known only when the write returns: while a write is blocked in the kernel,
commits keep succeeding and `ok()` stays true. The failure then surfaces at the
next `commitRow`, and `close()` always reports it. A queue overflow is reported
by the commit that hits it. A killed process loses the unwritten part of the
batch being written (up to the queue capacity) and every row queued behind it
(up to the capacity again), so up to about 8 MiB with the default; without a
blocked write that is about the last 20 ms of rows.

Whether a failed log aborts the run is the application's decision. `x7_follow`
and `x7_track` abort once a failure is known, as they always did on a failed
write. `x7_float`, `x7_gravity_demo`, and `x7_teach` finish the session,
because an abort would de-torque an arm the operator may already have
released, and then void it.

A per-cycle `.zvs` export (the joint sequence `rk_anim` replays) follows the
same rule: `model::AsyncZvsWriter` writes `model::ZvsWriter`'s format byte for
byte, with the file I/O on a writer thread, and `x7_wave --zvs` uses it.
`ZvsWriter` itself writes on the calling thread, so it is for offline tools
only.

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

### Measuring bus and cycle timing

`x7_bus_timing` measures the bus with every servo torque-off and never writes
Torque Enable or Drive Mode. A timing run refuses, before any write, a servo
with torque on or with Drive Mode bit 3 (Torque On by Goal Update, firmware
v45 and later) set, since on such a servo a goal write switches torque on.
`--inventory` reports each servo's bus-timing registers (Return Delay Time,
Status Return Level, baud register, firmware), its Drive Mode and the USB
adapter.
A timing run replays one mode's exact per-cycle traffic at a fixed rate and
writes one CSV row of host stamps per cycle plus a summary:

```sh
./build/apps/x7_bus_timing --inventory --out inventory.toml
./build/apps/x7_bus_timing --rate-hz 500 --profile current --duration 60 \
  --log bus-current-500hz.csv
```

`--confirm-writes` (each unchanged production write followed by a read of the
last servo's Model Number) and `--drain` turn it into a calibration run that
observes transport timing. `--read-path bounded` receives the grouped
read through the bounded path of control-rate Phase R (35 ms budget and an
equal guard by default, `--budget-ms` and `--guard-ms`), after a read-only
check of the bus profile. `dxl/wire_budget.hpp` gives the analytic wire bytes of each
cycle, and `x7_controller_bench` measures controller computation without a bus.
`tools/timing_summary.py` reduces probe logs and app `--timing-log` files to
measured, estimated and envelope quantities. The procedure that uses them is
[control-rate qualification](../hardware/control-rate.md);
`tools/e1_prod.sh` and `tools/e1_cal.sh` run its torque-off E1 step.

On the library side, `hw::CraneX7::Options::timing_ring_capacity` enables the
per-cycle `hw::CycleRecord` ring that `popCycleRecord()` drains, and
`CycleStats::timing` always holds host-measured histograms of every phase of
the background cycle. `util::RealtimePolicy` applies a SCHED_FIFO priority,
CPU affinity, memory lock or fine timer slack to the calling thread, all or
nothing.

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
