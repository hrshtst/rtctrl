#!/usr/bin/env bash
# Portable PTP archive creation, immediate no-clobber refusal, and replay from
# the archived config/model while invoked outside the bundle directory.
set -eu

APP="$1"
CONFIG="$2"
ROOT="$3"
BUNDLE="$ROOT/archive"
REPRODUCED="$ROOT/reproduced.zvs"

rm -rf "$ROOT"
mkdir -p "$ROOT"

"$APP" --config "$CONFIG" --bundle "$BUNDLE" --motion-time 0.2 \
  >"$ROOT/create.log" 2>&1

test -f "$BUNDLE/source.toml"
test -f "$BUNDLE/plan.toml"
test -f "$BUNDLE/trajectory.zvs"
test -f "$BUNDLE/manifest.toml"
test -f "$BUNDLE/model/crane_x7.ztk"
test -f "$BUNDLE/model/meshes/visual/mounting_plate.stl"
cmp "$CONFIG" "$BUNDLE/source.toml"
grep -q 'model = "model/crane_x7.ztk"' "$BUNDLE/plan.toml"
grep -q 'output = "trajectory.zvs"' "$BUNDLE/plan.toml"
grep -q 'motion_time = 0.20000000000000001' "$BUNDLE/plan.toml"

# The manifest uses a locally implemented SHA-256. Pin it against coreutils.
TRAJECTORY_HASH=$(sha256sum "$BUNDLE/trajectory.zvs" | cut -d ' ' -f 1)
grep -q "$TRAJECTORY_HASH" "$BUNDLE/manifest.toml"

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
  --output "$REPRODUCED" >"$ROOT/reproduce.log" 2>&1)
cmp "$BUNDLE/trajectory.zvs" "$REPRODUCED"

echo "PTP bundle creation and replay OK"
