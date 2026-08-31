# Host-side controller comparison

This workflow compares the same host-side computed-torque controller in
dynamics simulation and on the CRANE-X7 hardware. Both `x7_track_sim` and
`x7_track` evaluate the same law at the host control rate:

```math
tau = ID(q_{ref}, \dot q_{ref}, \ddot q_{ref})
    + K_p(q_{ref} - q) + K_d(\dot q_{ref} - \dot q)
```

The purpose is to preserve evidence of the expected contrast: the plain
controller can track successfully in simulation while the same logic can
track poorly or fail safely on hardware. Do not tune away that contrast during
the first campaign. Record the baseline first, then introduce additional
terms in separate, timestamped campaigns.

The renewed hardware path is experimental. It commands current directly,
and a successful simulation does not establish hardware safety. Complete the
[hardware bring-up checklist](../hardware/bringup.md), keep the actuator power
cutoff in reach, and begin with a short, slow reference.

## Campaign storage and invariants

Keep raw campaign artifacts outside the repository. Set `run_stamp` once so
the reference, both runs, and their analysis share one storage root:

```sh
run_stamp=$(date +%Y%m%d-%H%M%S)
storage="/external/x7-track-${run_stamp}"
mkdir -p "$storage/analysis"
```

Replace `/external` with the mounted storage location for the experiment. The
commands below create this layout:

```text
x7-track-TIMESTAMP/
  reference/ or teaching/       source trajectory bundle
  simulation/                   x7_track_sim bundle
  hardware/                     x7_track hardware bundle
  analysis/
    simulation-hardware.png
    simulation-hardware-error.png
    simulation-hardware-torque.png
    simulation-hardware.csv
```

Do not create `simulation/`, `hardware/`, `reference/`, or `teaching/` in
advance. Every `--bundle` target must be new. The apps reject an existing
file, directory, or symbolic link instead of overwriting it. The `analysis/`
directory is only a container and may exist before the runs.

For a valid simulation and hardware comparison, hold these values identical:

- source reference and reference processing;
- playback rate, control rate, interpolation, and controller gains;
- effort ceiling, home motion, safety thresholds, and finalization policy;
- model and hardware configuration inputs.

Simulation-only integration and initial-posture settings are still recorded
in the effective configuration. Do not compare runs from different builds or
silently edit either bundle. Use a new timestamp for every changed condition.

## Acquire and inspect the reference

Choose one reference source. A synthetic trajectory is the most controlled
starting point:

```sh
./build/apps/x7_plan_ptp --config config/ptp_example.toml \
  --bundle "$storage/reference"
reference="$storage/reference/trajectory.zvs"
reference_filter=none
```

A manually taught trajectory can be used instead:

```sh
./build/apps/x7_teach --config config/teach_example.toml --check
./build/apps/x7_teach --config config/teach_example.toml \
  --mode torque-off --bundle "$storage/teaching"
reference="$storage/teaching/trajectory.zvs"
reference_filter=low-pass
```

Use gravity-compensation teaching only after its separate bring-up workflow
has been accepted. Follow the complete acquisition and storage instructions
in [Teaching playback and servo-mode comparison](teaching-playback.md).

Inspect the reference before applying any torque:

```sh
if [ -d "$storage/reference" ]; then
  rk_anim "$storage/reference/model/crane_x7.ztk" "$reference"
else
  rk_anim "$storage/teaching/model/crane_x7.ztk" "$reference"
fi
```

`rk_anim` checks the model configurations visually. It does not simulate
dynamics, tracking error, effort limits, control timing, or safety responses.
For taught data, review the raw recording and the effect of filtering as
described in the teaching guide.

## Define one comparison condition

Use one shell array for every override shared by simulation and hardware. The
first hardware attempt should normally use a playback rate below 1.0. For
example, `0.5` follows the same geometric path over twice the original time:

```sh
track_config=config/track_example.toml
playback_rate=0.5
effort_limit_nm=2.5
common=(
  --config "$track_config"
  --reference "$reference"
  --filter "$reference_filter"
  --interpolation shape-preserving-cubic
  --playback-rate "$playback_rate"
  --effort-limit-nm "$effort_limit_nm"
)
```

The 2.5 Nm value is an example of an explicit all-joint ceiling, not a
universal safe limit. Review it against the robot configuration, payload, and
planned posture. To change the rate, gains, effort ceiling, or processing,
start a new campaign with a new `run_stamp`; do not change `common` between
the simulation and hardware runs.

Slower replay scales reference velocity by `playback_rate`, reference
acceleration by its square, and duration by its reciprocal. It therefore
changes the timed dynamics experiment even though the geometric path is
unchanged. Compare only bundles recorded at the same rate.

## Record the simulation bundle

Preflight does not create a bundle and does not run dynamics:

```sh
./build/apps/x7_track_sim "${common[@]}" --check
```

Then run and archive the complete simulated session:

```sh
./build/apps/x7_track_sim "${common[@]}" \
  --bundle "$storage/simulation"
```

Inspect the machine-readable result and replay the simulated motion:

```sh
sed -n '1,80p' "$storage/simulation/result.toml"
rk_anim "$storage/simulation/model/crane_x7.ztk" \
  "$storage/simulation/simulation.zvs"
```

Do not proceed if the simulation reports an execution failure, a hard safety
abort, clamp or gate events that were not expected, or a reference that is
unsafe to replay. `tracking_pass = false` is an assessment outcome rather
than an execution failure, but it also requires investigation before hardware
use.

## Preflight and record the hardware bundle

Complete hardware preflight with exactly the same shared overrides:

```sh
./build/apps/x7_track "${common[@]}" --check
```

Before starting, clear the workspace, support the arm as needed, and keep the
power cutoff in reach. The app moves to reference frame zero in servo position
mode. After the arm settles, it stops motion, stages gravity current, and
switches to current mode while stationary. Tracking does not begin until fresh
current-mode feedback is available. There is no operating-mode switch during
the reference motion.

Run the hardware experiment by itself, not as part of a chained shell command:

```sh
./build/apps/x7_track "${common[@]}" \
  --bundle "$storage/hardware"
```

Watch for unexpected posture, visible jerk, audible vibration, rapidly growing
error, clamp or gate events, stale commands, and timing faults. Use the power
cutoff when continued torque is unsafe. If the app reaches its final hold,
support the arm from below before pressing Enter to disable torque.

An assessment failure on hardware is a useful result for this experiment. A
hard-error abort is also evidence, provided the shutdown was controlled and
the resulting bundle was published. Do not repeat a failed run merely to
obtain a passing bundle. First preserve and review the evidence.

## Verify and compare the bundles

Review both results before plotting:

```sh
sed -n '1,80p' "$storage/simulation/result.toml"
sed -n '1,80p' "$storage/hardware/result.toml"
cmp "$storage/simulation/track.toml" "$storage/hardware/track.toml"
sha256sum "$storage/simulation/reference.zvs" \
  "$storage/hardware/reference.zvs"
```

`cmp` should report no difference, and both reference hashes should match. If
they do not, the runs are not one controlled comparison. Preserve them as
separate trials and start a new campaign.

Generate trajectory, error, and controller-torque figures plus a summary CSV
from the campaign root:

```sh
uv run --project tools tools/plot/track_comparison.py "$storage" \
  --output "$storage/analysis/simulation-hardware.png" \
  --error-output "$storage/analysis/simulation-hardware-error.png" \
  --torque-output "$storage/analysis/simulation-hardware-torque.png" \
  --summary-csv "$storage/analysis/simulation-hardware.csv"
```

Directory mode reads `simulation/simulation.csv` and
`hardware/hardware.csv`. It also refuses to compare the campaign when the two
bundled reference files have different SHA-256 hashes. The report includes
per-joint and aggregate RMS and peak tracking error, cycle timing, rejected
receipts, clamp and gate events, and the feedforward, feedback, and commanded
torques.

Interpret the result as a controller and plant comparison, not as proof that
the simulator is generally accurate or that one unmodeled hardware effect has
been identified. Useful follow-up evidence includes the first joint and time
at which errors diverge, whether PD torque grows against the divergence,
whether commanded torque reaches its ceiling, and whether hardware timing or
command receipt faults occurred. Introduce friction compensation, filtering,
integral action, gain scaling, or other practical terms only in later,
separately timestamped campaigns.

## Preserve the complete campaign

Keep the complete campaign directory, not only the plots. Each simulation and
hardware bundle contains the source and effective TOML files, reference,
hardware configuration, model dependencies, telemetry, result, Git revision,
hashes, and manifest. These inputs distinguish a genuine controller comparison
from two unrelated logs.

Raw telemetry remains external to the repository. When a campaign becomes
project evidence, archive it according to
[Data archive](../records/data-archive.md) and add only the tracked summary or
evidence sidecar required by that policy.
