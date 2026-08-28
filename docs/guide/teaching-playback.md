# Teaching playback and servo-mode comparison

This workflow compares the CRANE-X7's normal position mode with the selected
current-based-position (CBP) deployment. A reference may come from Cartesian
PTP planning or from manual teaching. In either case, first inspect the
reference, then run both modes in dynamics simulation, and only then consider
the two hardware runs.

The comparison is between the normal position baseline and the currently
selected tuned CBP setup. It is not a pure operating-mode-only experiment:
the CBP run uses an explicit effort ceiling and the checked-in 75 percent D
gain configuration with P=900 on motor ID 5. Those gains are provisional and
specific to the tested CRANE-X7 with firmware 47.

## Campaign storage and invariants

Keep every campaign outside the repository. Set `run_stamp` once so all
artifacts from one comparison share a storage root:

```sh
run_stamp=$(date +%Y%m%d-%H%M%S)
storage="/external/some-label-${run_stamp}"
mkdir -p "$storage/simulation" "$storage/analysis"
```

The commands below create this layout:

```text
some-label-TIMESTAMP/
  reference/ or teaching/       source trajectory bundle
  simulation/
    position/                   position simulation bundle
    cbp/                        CBP simulation bundle
  position/                     position hardware bundle
  cbp/                          CBP hardware bundle
  analysis/                     comparison figures and summaries
```

Each bundle target must not exist before its command. The apps refuse an
existing file, directory, or symbolic link instead of overwriting it. The
campaign directory itself is only a container; each child bundle has its own
effective configuration, inputs, result, Git revision, hashes, and manifest.

For a fair comparison, hold these values identical between position and CBP:

- reference trajectory;
- control rate, home motion, safety thresholds, and finalization policy;
- filter, interpolation, and all detailed processing parameters;
- simulation initial posture and integration settings.

Only the servo mode, CBP effort ceiling, and CBP motor parameters should
differ. Run position first, then CBP. Do not chain hardware commands: inspect
each result and allow the arm and servos to settle before continuing.

## Option A: plan a synthetic reference

Plan and archive a Cartesian PTP trajectory:

```sh
./build/apps/x7_plan_ptp --config config/ptp_pick_example.toml \
  --bundle "$storage/reference"
reference="$storage/reference/trajectory.zvs"
```

Inspect the joint motion and the planner diagnostics:

```sh
rk_anim "$storage/reference/model/crane_x7.ztk" "$reference"
uv run --project tools tools/plot/ptp_trajectory.py \
  "$storage/reference/trajectory.csv" \
  --output "$storage/analysis/planned-trajectory.png"
```

`rk_anim` replays model configurations only. It does not simulate dynamics,
servo behavior, effort limits, tracking error, or safety responses. Continue
to the dynamics simulations below even when the animation looks correct.

Synthetic planner output is already smooth, so use no additional filter:

```sh
reference_filter=none
```

## Option B: teach a reference manually

Choose exactly one teaching mode. Torque-off mode is the first choice:

```sh
./build/apps/x7_teach --config config/teach_example.toml --check
./build/apps/x7_teach --config config/teach_example.toml \
  --mode torque-off --bundle "$storage/teaching"
```

Gravity compensation is an alternative only after the accepted `x7_float`
workflow and its hazards are understood:

```sh
./build/apps/x7_teach --config config/teach_example.toml \
  --mode gravity-compensation --check
./build/apps/x7_teach --config config/teach_example.toml \
  --mode gravity-compensation --bundle "$storage/teaching"
```

In gravity mode, Enter starts recording, a second Enter stops recording, and
a third Enter confirms that the operator is supporting the arm before torque
is disabled. Keep the actuator power cutoff in reach throughout the session.

Inspect the taught motion before playback:

```sh
reference="$storage/teaching/trajectory.zvs"
rk_anim "$storage/teaching/model/crane_x7.ztk" "$reference"
```

Teaching records what the operator did; it does not certify collision
clearance, joint speed, acceleration, or replay safety. The ZVS is an
unsmoothed linear resampling of the raw 100 Hz acquisition. For the initial
playback workflow, use the base configuration's 5 Hz low-pass setting:

```sh
reference_filter=low-pass
```

If that filtering changes the intended path materially, stop and review the
raw `recording.csv` and the processed simulation reference. Do not compensate
by changing filtering between servo modes.

## Simulate both playback modes

Both reference-acquisition options set `reference` and `reference_filter`.
Use one shared follow configuration and override only the campaign inputs and
comparison dimensions:

```sh
follow_config=config/follow_example.toml
cbp_parameters=config/follow_cbp_75pct_d_id5_p900_params.toml
common=(
  --config "$follow_config"
  --reference "$reference"
  --filter "$reference_filter"
  --interpolation shape-preserving-cubic
)
```

Run the complete phase controller in dynamics simulation for position mode:

```sh
./build/apps/x7_follow_sim "${common[@]}" --mode position \
  --bundle "$storage/simulation/position"
```

Then run CBP simulation with the selected hardware input and the reviewed
2.5 Nm all-joint effort ceiling:

```sh
./build/apps/x7_follow_sim "${common[@]}" \
  --mode current-based-position \
  --motor-parameters "$cbp_parameters" --effort-limit-nm 2.5 \
  --bundle "$storage/simulation/cbp"
```

The simulation frontend validates and archives the motor-parameter file, but
does not model the Dynamixel firmware gains. Its CBP adapter applies the same
position adapter with the requested torque ceiling. Simulation verifies phase
transitions, reference processing, dynamics response, tracking guards, and
effort limiting. It cannot predict current-loop behavior, friction, backlash,
gear compliance, audible vibration, or the hardware benefit of the tuned
gains.

Replay both full simulated sessions and compare their tracking phases:

```sh
rk_anim "$storage/simulation/position/model/crane_x7.ztk" \
  "$storage/simulation/position/simulation.zvs"
rk_anim "$storage/simulation/cbp/model/crane_x7.ztk" \
  "$storage/simulation/cbp/simulation.zvs"

uv run --project tools tools/plot/follow_mode_comparison.py \
  "$storage/simulation/position/simulation.csv" \
  "$storage/simulation/cbp/simulation.csv" \
  --output "$storage/analysis/simulation-comparison.png" \
  --summary-csv "$storage/analysis/simulation-comparison.csv"
```

The comparison tool refuses logs with different tracking sample times or
joint references. Do not proceed if either simulation fails, triggers a safety
abort, or produces a reference mismatch.

## Preflight and execute on hardware

Complete the [hardware bring-up checklist](../hardware/bringup.md) first. Run
both preflight checks separately. `--check` performs configuration, reference,
limit, effort, and parameter validation without opening the bus:

```sh
./build/apps/x7_follow "${common[@]}" --mode position --check

./build/apps/x7_follow "${common[@]}" \
  --mode current-based-position \
  --motor-parameters "$cbp_parameters" --effort-limit-nm 2.5 --check
```

Review the simulations and preflight output before enabling torque. Keep the
power cutoff in reach and the workspace clear. At the end of each run, support
the arm from below before pressing Enter to disable torque.

Run and archive the position baseline:

```sh
./build/apps/x7_follow "${common[@]}" --mode position \
  --bundle "$storage/position"
```

Inspect its result and hardware timing before continuing. Then run the tuned
CBP deployment:

```sh
./build/apps/x7_follow "${common[@]}" \
  --mode current-based-position \
  --motor-parameters "$cbp_parameters" --effort-limit-nm 2.5 \
  --bundle "$storage/cbp"
```

Visible jerk, audible vibration, an unexpected posture, a tracking abort,
stale command, deadline miss, clamp or gate event, or incomplete shutdown is a
reason to stop the campaign. Do not continue merely to obtain the second log.

## Compare the hardware runs

The hardware bundle names above match the comparison tool's directory
convention. Generate the figure and machine-readable summary directly from
the campaign root:

```sh
uv run --project tools tools/plot/follow_mode_comparison.py "$storage" \
  --output "$storage/analysis/hardware-comparison.png" \
  --summary-csv "$storage/analysis/hardware-comparison.csv"
```

The report covers tracking-phase RMS and peak error for every joint, cycle
period statistics, rejected command receipts, clamp and gate events, and
command-application lag. Preserve the complete campaign directory: the plots
are summaries, while the child bundles contain the inputs and evidence needed
to interpret or reproduce each run.

## Follow override reference

`x7_follow_sim` and `x7_follow` share these comparison overrides:

| Option | Effect |
|---|---|
| `--reference FILE` | Replace the TOML reference with an invocation-relative input path. |
| `--mode MODE` | Select `position` or `current-based-position`. |
| `--motor-parameters FILE` | Replace or add the reviewed parameter input. |
| `--effort-limit-nm VALUE` | Apply one positive CBP ceiling to all eight joints. |
| `--filter FILTER` | Select none, low-pass, moving-average, or Savitzky-Golay processing. |
| `--interpolation METHOD` | Select linear or shape-preserving-cubic interpolation. |

CLI overrides take precedence over the source TOML and are written into each
bundle's effective `follow.toml`. Detailed processing parameters remain in the
source TOML. A bundle owns its output paths, so `--bundle` cannot be combined
with `--log` or the simulation-only `--motion` option.
