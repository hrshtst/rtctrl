#!/usr/bin/env bash
# End-to-end x7_gravity_demo over the dxl_emu pty: the pose-first
# startup and log contract with the DEFAULT (vendor) constants, the
# custom-kt header derivation, and the ENTER early-stop. Run from the
# repo root (models/ and config/ by relative path).
set -eu
EMU="$1"
DEMO="$2"
OUT="$3"
mkdir -p "$OUT"
LINK="$OUT/ttyDXL"
# Logs are created EXCLUSIVELY (never overwritten): clear previous runs.
rm -f "$LINK" "$OUT/demo_default.csv" "$OUT/demo_custom.csv" \
  "$OUT/demo_stop.csv"
"$EMU" --link "$LINK" &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

# Phase 1: defaults — a flagless run IS the approved vendor
# calibration; EOF on the piped stdin must never read as the stop key,
# so the 2 s session runs to completion.
PHASE1=$("$DEMO" --port "$LINK" --log "$OUT/demo_default.csv" 2 \
  < /dev/null)
echo "$PHASE1" | grep -q "demonstration complete" || {
  echo "phase 1 did not complete:"; echo "$PHASE1"; exit 1;
}
python3 - "$OUT/demo_default.csv" <<'EOF'
import csv
import math
import sys
lines = open(sys.argv[1]).read().splitlines()
head = [ln for ln in lines if ln.startswith("#")]
assert any("run_mode: demonstration" in ln for ln in head), head
assert any("calibration: vendor-approved" in ln for ln in head), head
scales = next(
    ln for ln in head if "command_torque_scale:" in ln).split(":")[1].split()
assert scales == ["0.810455", "0.669167"] + ["0.810455"] * 6, scales
rows = list(csv.DictReader(
    ln for ln in lines if not ln.startswith("#")))
assert len(rows) > 50, len(rows)
for r in rows[:: max(1, len(rows) // 20)]:
    for k, v in r.items():
        assert math.isfinite(float(v)), (k, v)
print(f"default log OK ({len(rows)} rows)")
EOF

# Phase 2: custom constants need the unmistakable experimental
# opt-in, land in the header as scale = kt_nominal / kt_effective,
# and the log self-labels the calibration EXPERIMENTAL.
PHASE2=$("$DEMO" --port "$LINK" --experimental-calibration \
  --kt-xm430 2.0 --kt-xm540 3.0 --log "$OUT/demo_custom.csv" 2 \
  < /dev/null)
echo "$PHASE2" | grep -q "EXPERIMENTAL calibration session" || {
  echo "phase 2 missing the experimental banner:"; echo "$PHASE2"; exit 1;
}
python3 - "$OUT/demo_custom.csv" <<'EOF'
import sys
head = [ln for ln in open(sys.argv[1]) if ln.startswith("#")]
assert any("calibration: EXPERIMENTAL" in ln for ln in head), head
scales = next(
    ln for ln in head if "command_torque_scale:" in ln).split(":")[1].split()
assert scales == ["0.891500", "0.803000"] + ["0.891500"] * 6, scales
kts = next(
    ln for ln in head if "kt_effective" in ln).split(":")[1].split()
assert kts == ["2.000000", "3.000000"] + ["2.000000"] * 6, kts
print("custom-kt header OK")
EOF

# Phase 3: ENTER ends the session early through the verified shutdown
# (exit 0, the explicit early-stop verdict).
PHASE3=$( (sleep 1; echo) | "$DEMO" --port "$LINK" \
  --log "$OUT/demo_stop.csv" 30 )
echo "$PHASE3" | grep -q "stopped by operator" || {
  echo "phase 3 missing the early-stop verdict:"; echo "$PHASE3"; exit 1;
}
echo "gravity demo smoke OK"
