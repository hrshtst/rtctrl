#!/usr/bin/env bash
# Generate phase-complete simulation telemetry, render it headlessly, and
# ensure malformed logs are refused.
set -eu

BUNDLE_SMOKE="$1"
FOLLOW_APP="$2"
PLAN_APP="$3"
PLAN_CONFIG="$4"
MODEL="$5"
HARDWARE="$6"
PLOTTER="$7"
COMPARATOR="$8"
ROOT="$9"

bash "$BUNDLE_SMOKE" "$FOLLOW_APP" "$PLAN_APP" "$PLAN_CONFIG" \
  "$MODEL" "$HARDWARE" "$ROOT/run" >"$ROOT-bundle.log" 2>&1
python "$PLOTTER" "$ROOT/run/archive/simulation.csv" \
  --output "$ROOT/follow.png" >"$ROOT/plot.log" 2>&1

test -s "$ROOT/follow.png"
grep -q "maximum joint tracking error" "$ROOT/plot.log"
grep -q "phase cycles" "$ROOT/plot.log"

BUNDLE_PLOTTER="$(dirname "$PLOTTER")/follow_bundle_tracking.py"
python "$BUNDLE_PLOTTER" "$ROOT/run/archive" \
  >"$ROOT/bundle-plot.log" 2>&1
test -s "$ROOT/run/archive/tracking-analysis.png"
grep -q "maximum joint tracking error" "$ROOT/bundle-plot.log"

printf 'time_s,phase\n0,tracking\n' >"$ROOT/malformed.csv"
if python "$PLOTTER" "$ROOT/malformed.csv" \
    --output "$ROOT/malformed.png" >"$ROOT/malformed.log" 2>&1; then
  echo "malformed follow telemetry was unexpectedly accepted"
  exit 1
fi
grep -q "missing columns" "$ROOT/malformed.log"
test ! -e "$ROOT/malformed.png"

mkdir -p "$ROOT/comparison"
cp "$ROOT/run/archive/simulation.csv" "$ROOT/comparison/position.csv"
awk 'BEGIN { FS=OFS="," } NR > 1 { $4=5 } { print }' \
  "$ROOT/run/archive/simulation.csv" >"$ROOT/comparison/cbp.csv"
python "$COMPARATOR" "$ROOT/comparison/position.csv" \
  "$ROOT/comparison/cbp.csv" --output "$ROOT/comparison.png" \
  --summary-csv "$ROOT/comparison.csv" >"$ROOT/comparison.log" 2>&1

test -s "$ROOT/comparison.png"
test -s "$ROOT/comparison.csv"
grep -q "aggregate RMS" "$ROOT/comparison.log"
grep -q "current_based_position_rms_rad" "$ROOT/comparison.csv"

if python "$COMPARATOR" "$ROOT/comparison/position.csv" \
    "$ROOT/comparison/cbp.csv" --failure-analysis \
    --output "$ROOT/not-aborted.png" >"$ROOT/not-aborted.log" 2>&1; then
  echo "failure analysis accepted a completed CBP run"
  exit 1
fi
grep -q "requires a CBP log with an abort_hold phase" \
  "$ROOT/not-aborted.log"
test ! -e "$ROOT/not-aborted.png"

if python "$COMPARATOR" "$ROOT/comparison/position.csv" \
    "$ROOT/comparison/position.csv" --output "$ROOT/wrong-mode.png" \
    >"$ROOT/wrong-mode.log" 2>&1; then
  echo "mode comparison accepted two position logs"
  exit 1
fi
grep -q "expected current-based-position command mode" "$ROOT/wrong-mode.log"
test ! -e "$ROOT/wrong-mode.png"

# A CBP safety abort is comparable only through its shared reference prefix,
# and only when failure analysis is explicitly requested. Preserve the home
# phases, truncate tracking, and turn the normal final hold into abort_hold.
awk 'BEGIN { FS=OFS="," }
  NR == 1 { print; next }
  $3 == "tracking" { if (++tracking <= 10) print; next }
  $3 == "finalizing" { $3="abort_hold" }
  { print }' "$ROOT/comparison/cbp.csv" >"$ROOT/comparison/cbp-abort.csv"
if python "$COMPARATOR" "$ROOT/comparison/position.csv" \
    "$ROOT/comparison/cbp-abort.csv" \
    --output "$ROOT/failure-strict.png" >"$ROOT/failure-strict.log" 2>&1; then
  echo "mode comparison accepted a truncated CBP run without failure analysis"
  exit 1
fi
grep -q "use --failure-analysis" "$ROOT/failure-strict.log"
test ! -e "$ROOT/failure-strict.png"

python "$COMPARATOR" "$ROOT/comparison/position.csv" \
  "$ROOT/comparison/cbp-abort.csv" --failure-analysis \
  --output "$ROOT/failure-analysis.png" \
  --summary-csv "$ROOT/failure-analysis.csv" \
  >"$ROOT/failure-analysis.log" 2>&1
test -s "$ROOT/failure-analysis.png"
test -s "$ROOT/failure-analysis.csv"
grep -q "FAILURE ANALYSIS: CBP tracking ended in abort_hold" \
  "$ROOT/failure-analysis.log"
grep -q "cbp_abort_common_prefix" "$ROOT/failure-analysis.csv"

echo "follow telemetry plotting OK"
