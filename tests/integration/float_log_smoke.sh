#!/usr/bin/env bash
# End-to-end x7_float --log validation over the dxl_emu pty: start the
# emulator, float for 2 s with logging, then validate the CSV's
# structure and event semantics (check_float_log.py). Run from the
# repo root (models/ and config/ are loaded by relative path).
set -eu
EMU="$1"
FLOAT="$2"
OUT="$3"
mkdir -p "$OUT"
LINK="$OUT/ttyDXL"
rm -f "$LINK" "$OUT/float.csv"
"$EMU" --link "$LINK" &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }
"$FLOAT" --port "$LINK" --log "$OUT/float.csv" 2
python3 "$(dirname "$0")/check_float_log.py" "$OUT/float.csv"
