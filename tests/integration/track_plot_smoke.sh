#!/usr/bin/env bash
set -eu

BUNDLE_SMOKE="$1"
TRACK_APP="$2"
PLAN_APP="$3"
PLAN_CONFIG="$4"
MODEL="$5"
HARDWARE="$6"
PLOT="$7"
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init track_plot_smoke "${8:-}" || exit 2  # a new child of an approved base: the only place this test removes
ROOT=$RTCTRL_SCRATCH

bash "$BUNDLE_SMOKE" "$TRACK_APP" "$PLAN_APP" "$PLAN_CONFIG" \
  "$MODEL" "$HARDWARE" "$ROOT/track_bundle_smoke"
RUN=$(scratch_nested track_bundle_smoke)  # the bundle test's own child, left in this one's

python "$PLOT" "$RUN/archive/simulation.csv" "$RUN/archive/simulation.csv" \
  --output "$ROOT/comparison.png" \
  --summary-csv "$ROOT/comparison.csv"
test -s "$ROOT/comparison.png"
test -s "$ROOT/comparison-error.png"
test -s "$ROOT/comparison-torque.png"
test -s "$ROOT/comparison.csv"

scratch_release
echo "track telemetry plotting OK"
