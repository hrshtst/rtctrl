#!/usr/bin/env bash
# x7_wave --zvs exports the commanded position reference over the
# dxl_emu pty as a 9-coordinate sequence for rk_anim replay — the
# same-motion bridge between a hardware wave session and the viewer.
set -eu
EMU="$1"
WAVE="$2"
OUT="$3"
mkdir -p "$OUT"
LINK="$OUT/ttyDXL"
rm -f "$LINK" "$OUT/wave.zvs"
"$EMU" --link "$LINK" &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

# --zvs requires a non-empty value.
if "$WAVE" --port "$LINK" --zvs "" 2 >/dev/null 2>&1; then
  echo "accepted an empty --zvs value"
  exit 1
fi
if "$WAVE" --port "$LINK" 2 --zvs >/dev/null 2>&1; then
  echo "accepted --zvs without a value"
  exit 1
fi

"$WAVE" --port "$LINK" --zvs "$OUT/wave.zvs" 2 > /dev/null

# ~200 frames for a 2 s run at the 10 ms cycle; a generous band keeps
# parallel-suite load jitter from tipping the count (cf. the float
# smoke's margin note).
frames=$(wc -l < "$OUT/wave.zvs")
if [ "$frames" -lt 150 ] || [ "$frames" -gt 260 ]; then
  echo "unexpected frame count: $frames"
  exit 1
fi
# Every frame is "<dt> 9 ( ... )" — the finger-mimic-expanded vector.
if grep -vqE '^[0-9.eE+-]+ 9 \(' "$OUT/wave.zvs"; then
  echo "malformed zvs frame:"
  grep -vE '^[0-9.eE+-]+ 9 \(' "$OUT/wave.zvs" | head -2
  exit 1
fi
echo "wave zvs export holds ($frames frames)"
