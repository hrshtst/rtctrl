#!/usr/bin/env bash
# Portable PTP archive creation, immediate no-clobber refusal, and replay from
# the archived config/model while invoked outside the bundle directory.
set -eu

APP="$1"
CONFIG="$2"
ROOT="$3"
BUNDLE="$ROOT/archive"
REPRODUCED="$ROOT/reproduced.zvs"
REPRODUCED_CSV="$ROOT/reproduced.csv"

rm -rf "$ROOT"
mkdir -p "$ROOT"

"$APP" --config "$CONFIG" --bundle "$BUNDLE" --motion-time 0.2 \
  >"$ROOT/create.log" 2>&1

test -f "$BUNDLE/source.toml"
test -f "$BUNDLE/plan.toml"
test -f "$BUNDLE/trajectory.zvs"
test -f "$BUNDLE/trajectory.csv"
test -f "$BUNDLE/manifest.toml"
test -f "$BUNDLE/model/crane_x7.ztk"
test -f "$BUNDLE/model/meshes/visual/mounting_plate.stl"
cmp "$CONFIG" "$BUNDLE/source.toml"
grep -q 'model = "model/crane_x7.ztk"' "$BUNDLE/plan.toml"
grep -q 'output = "trajectory.zvs"' "$BUNDLE/plan.toml"
grep -q 'output = "trajectory.csv"' "$BUNDLE/plan.toml"
grep -q 'motion_time = 0.20000000000000001' "$BUNDLE/plan.toml"

# The manifest uses a locally implemented SHA-256. Pin it against coreutils.
TRAJECTORY_HASH=$(sha256sum "$BUNDLE/trajectory.zvs" | cut -d ' ' -f 1)
grep -q "$TRAJECTORY_HASH" "$BUNDLE/manifest.toml"
DIAGNOSTICS_HASH=$(sha256sum "$BUNDLE/trajectory.csv" | cut -d ' ' -f 1)
grep -q "$DIAGNOSTICS_HASH" "$BUNDLE/manifest.toml"

python3 - "$BUNDLE/trajectory.csv" <<'PY'
import csv
import math
import sys

with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) > 2
required = {
    "target_quat_w",
    "fk_pos_x_m",
    "target_vel_x_m_s",
    "target_acc_x_m_s2",
    "dq0_rad_s",
    "ddq7_rad_s2",
    "position_error_norm_m",
}
assert required <= rows[0].keys()
for row in rows:
    assert all(math.isfinite(float(row[key])) for key in required)
assert max(float(row["position_error_norm_m"]) for row in rows) < 1e-4
PY

# An existing target wins over even a missing source config and remains intact.
BEFORE_HASH=$(sha256sum "$BUNDLE/manifest.toml" | cut -d ' ' -f 1)
if "$APP" --config "$ROOT/does-not-exist.toml" --bundle "$BUNDLE" \
    >"$ROOT/refuse.log" 2>&1; then
  echo "existing bundle was unexpectedly accepted"
  exit 1
fi
grep -q "bundle directory already exists" "$ROOT/refuse.log"
AFTER_HASH=$(sha256sum "$BUNDLE/manifest.toml" | cut -d ' ' -f 1)
test "$BEFORE_HASH" = "$AFTER_HASH"

# A failed new archive is cleaned up rather than published partially.
FAILED_BUNDLE="$ROOT/failed-archive"
if "$APP" --config "$ROOT/does-not-exist.toml" --bundle "$FAILED_BUNDLE" \
    >"$ROOT/failed-create.log" 2>&1; then
  echo "missing config was unexpectedly accepted"
  exit 1
fi
test ! -e "$FAILED_BUNDLE"
if find "$ROOT" -maxdepth 1 -name '.failed-archive.tmp.*' | grep -q .; then
  echo "failed bundle left a staging directory"
  exit 1
fi

# Replay uses the archived model. An explicit output keeps the archive immutable.
(cd "$ROOT" && "$APP" --config "$BUNDLE/plan.toml" \
  --output "$REPRODUCED" --diagnostics "$REPRODUCED_CSV" \
  >"$ROOT/reproduce.log" 2>&1)
cmp "$BUNDLE/trajectory.zvs" "$REPRODUCED"
cmp "$BUNDLE/trajectory.csv" "$REPRODUCED_CSV"

echo "PTP bundle creation and replay OK"
