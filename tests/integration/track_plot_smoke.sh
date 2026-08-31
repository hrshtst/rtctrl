#!/usr/bin/env bash
set -eu

BUNDLE_SMOKE="$1"
TRACK_APP="$2"
PLAN_APP="$3"
PLAN_CONFIG="$4"
MODEL="$5"
HARDWARE="$6"
PLOT="$7"
ROOT="$8"

bash "$BUNDLE_SMOKE" "$TRACK_APP" "$PLAN_APP" "$PLAN_CONFIG" \
  "$MODEL" "$HARDWARE" "$ROOT"

python "$PLOT" "$ROOT/archive/simulation.csv" "$ROOT/archive/simulation.csv" \
  --output "$ROOT/comparison.png" \
  --summary-csv "$ROOT/comparison.csv"
test -s "$ROOT/comparison.png"
test -s "$ROOT/comparison-error.png"
test -s "$ROOT/comparison-torque.png"
test -s "$ROOT/comparison.csv"

echo "track telemetry plotting OK"
