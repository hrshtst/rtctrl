#!/usr/bin/env bash
# End-to-end x7_float --log validation over the dxl_emu pty, two
# phases (GRAVITY_CALIBRATION_PLAN M-GC1/M-GC2):
#  1. default config, 2 s, no marker: the position-hold -> preloaded
#     switch startup and the log contract at scale 1.0 (ends normally
#     before the 8 s marker deadline);
#  2. vendor-scale config with a piped release marker at ~1 s: the run
#     must self-terminate 5 s after the marker, and every applied
#     torque must equal scale * tau_request — the end-to-end proof the
#     calibration is applied exactly once through the RealArm path.
# Run from the repo root (models/ and config/ by relative path).
set -eu
EMU="$1"
FLOAT="$2"
OUT="$3"
mkdir -p "$OUT"
LINK="$OUT/ttyDXL"
rm -f "$LINK" "$OUT/float.csv" "$OUT/float_vendor.csv"
"$EMU" --link "$LINK" &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

"$FLOAT" --port "$LINK" --log "$OUT/float.csv" 2 < /dev/null
python3 "$(dirname "$0")/check_float_log.py" "$OUT/float.csv"

(sleep 1; echo) | "$FLOAT" --config config/crane_x7_vendor_scale.toml \
  --port "$LINK" --log "$OUT/float_vendor.csv" 15
python3 "$(dirname "$0")/check_float_log.py" --vendor \
  "$OUT/float_vendor.csv"
