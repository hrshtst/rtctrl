#!/usr/bin/env bash
# Generate authoritative planner diagnostics, render them headlessly, and
# verify that malformed input is rejected rather than partially interpreted.
set -eu

APP="$1"
PLOTTER="$2"
CONFIG="$3"
ROOT="$4"

rm -rf "$ROOT"
mkdir -p "$ROOT"

"$APP" --config "$CONFIG" --output "$ROOT/trajectory.zvs" \
  --diagnostics "$ROOT/trajectory.csv" --motion-time 0.2 \
  >"$ROOT/plan.log" 2>&1
python "$PLOTTER" "$ROOT/trajectory.csv" \
  --output "$ROOT/trajectory.png" >"$ROOT/plot.log" 2>&1

test -s "$ROOT/trajectory.png"
grep -q "max TCP errors" "$ROOT/plot.log"
grep -q "minimum joint-limit margin" "$ROOT/plot.log"

printf 'time_s\n0\n' >"$ROOT/malformed.csv"
if python "$PLOTTER" "$ROOT/malformed.csv" \
    --output "$ROOT/malformed.png" >"$ROOT/malformed.log" 2>&1; then
  echo "malformed diagnostics were unexpectedly accepted"
  exit 1
fi
grep -q "missing columns" "$ROOT/malformed.log"
test ! -e "$ROOT/malformed.png"

echo "PTP diagnostics plotting OK"
