#!/usr/bin/env bash
# Create and replay a self-contained textbook computed-torque simulation.
set -eu

TRACK_APP="$1"
PLAN_APP="$2"
PLAN_CONFIG="$3"
MODEL="$4"
HARDWARE="$5"
ROOT="$6"
BUNDLE="$ROOT/archive"

rm -rf "$ROOT"
mkdir -p "$ROOT"

"$PLAN_APP" --config "$PLAN_CONFIG" --output "$ROOT/reference.zvs" \
  --diagnostics "$ROOT/reference.csv" --motion-time 0.2 \
  >"$ROOT/plan.log" 2>&1

cat >"$ROOT/track.toml" <<EOF
format = "rtctrl-x7-track"
version = 1
model = "$MODEL"
reference = "$ROOT/reference.zvs"
hardware_config = "$HARDWARE"

[control]
rate_hz = 100
kp = 20
kd = 2
playback_rate = 0.5

[home]
velocity_limit = 4.0
strict = false
tolerance_rad = 0.02
settle_time_s = 0.02

[simulation]
initial_posture = "zeros"
integration_step_s = 0.0001

[safety]
warning_error_rad = 10.0
sustained_abort_error_rad = 20.0
sustained_abort_time_s = 0.2
immediate_abort_error_rad = 30.0

[assessment]
rms_error_rad = 1.0
peak_error_rad = 2.0

[finalization]
simulation_hold_time_s = 0.02
EOF

"$TRACK_APP" --config "$ROOT/track.toml" --bundle "$BUNDLE" \
  >"$ROOT/create.log" 2>&1

for path in source.toml track.toml reference.zvs hardware.toml \
    simulation.zvs simulation.csv result.toml manifest.toml \
    model/crane_x7.ztk model/meshes/visual/mounting_plate.stl; do
  test -f "$BUNDLE/$path"
done
grep -q 'format = "rtctrl-x7-track"' "$BUNDLE/track.toml"
grep -q 'playback_rate = 0.5' "$BUNDLE/track.toml"
grep -q 'status = "success"' "$BUNDLE/result.toml"
grep -q 'tracking_pass = true' "$BUNDLE/result.toml"

"$TRACK_APP" --config "$BUNDLE/track.toml" \
  --motion "$ROOT/reproduced.zvs" --log "$ROOT/reproduced.csv" \
  >"$ROOT/reproduce.log" 2>&1
cmp "$BUNDLE/simulation.zvs" "$ROOT/reproduced.zvs"
cmp "$BUNDLE/simulation.csv" "$ROOT/reproduced.csv"

echo "track bundle creation and replay OK"
