#!/usr/bin/env bash
# Create and replay a self-contained follow simulation archive, then verify
# immediate no-clobber refusal and its SHA-256 manifest.
set -eu

FOLLOW_APP="$1"
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

cat >"$ROOT/follow.toml" <<EOF
format = "rtctrl-x7-follow"
version = 1
model = "$MODEL"
reference = "$ROOT/reference.zvs"
hardware_config = "$HARDWARE"

[control]
rate_hz = 100
mode = "position"

[home]
profile = "minimum-jerk"
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

[finalization]
simulation_hold_time_s = 0.02
EOF

"$FOLLOW_APP" --config "$ROOT/follow.toml" --bundle "$BUNDLE" \
  >"$ROOT/create.log" 2>&1

for path in source.toml follow.toml reference.zvs hardware.toml \
    simulation.zvs simulation.csv result.toml manifest.toml \
    model/crane_x7.ztk model/meshes/visual/mounting_plate.stl; do
  test -f "$BUNDLE/$path"
done
grep -q 'model = "model/crane_x7.ztk"' "$BUNDLE/follow.toml"
grep -q 'reference = "reference.zvs"' "$BUNDLE/follow.toml"
grep -q 'status = "success"' "$BUNDLE/result.toml"

SIM_HASH=$(sha256sum "$BUNDLE/simulation.zvs" | cut -d ' ' -f 1)
grep -q "$SIM_HASH" "$BUNDLE/manifest.toml"

BEFORE_HASH=$(sha256sum "$BUNDLE/manifest.toml" | cut -d ' ' -f 1)
if "$FOLLOW_APP" --config "$ROOT/missing.toml" --bundle "$BUNDLE" \
    >"$ROOT/refuse.log" 2>&1; then
  echo "existing follow bundle was unexpectedly accepted"
  exit 1
fi
grep -q "bundle directory already exists" "$ROOT/refuse.log"
AFTER_HASH=$(sha256sum "$BUNDLE/manifest.toml" | cut -d ' ' -f 1)
test "$BEFORE_HASH" = "$AFTER_HASH"

"$FOLLOW_APP" --config "$BUNDLE/follow.toml" \
  --motion "$ROOT/reproduced.zvs" --log "$ROOT/reproduced.csv" \
  >"$ROOT/reproduce.log" 2>&1
cmp "$BUNDLE/simulation.zvs" "$ROOT/reproduced.zvs"
cmp "$BUNDLE/simulation.csv" "$ROOT/reproduced.csv"

echo "follow bundle creation and replay OK"
