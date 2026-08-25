#!/usr/bin/env bash
# End-to-end TCP placement over the PTY bus. The solve happens after the
# servo watchdog is armed and must keep the measured position stream alive.
set -eu

EMU="$1"
POSE="$2"
OUT="$3"
mkdir -p "$OUT"
LINK="$OUT/ttyDXL"
rm -f "$LINK"

"$EMU" --link "$LINK" > "$OUT/emu.log" 2>&1 &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

# The zero-seeded target takes about 9 s at 0.5 rad/s. Deliver ENTER after
# placement so the app exercises its verified deactivation path as well.
OUTPUT=$( (sleep 12; echo) | "$POSE" --port "$LINK" \
  --tcp 0.2 0 0.25 0 0 0 --vel 0.5 2>&1)

echo "$OUTPUT" | grep -q "moving to the posture" || {
  echo "TCP placement did not start:"
  echo "$OUTPUT"
  exit 1
}
echo "$OUTPUT" | grep -q "the arm is limp" || {
  echo "TCP placement did not shut down cleanly:"
  echo "$OUTPUT"
  exit 1
}
if echo "$OUTPUT" | grep -Eq "IK did not converge|SHUTDOWN FAULT|DEADMAN"; then
  echo "TCP placement hit an IK/watchdog fault:"
  echo "$OUTPUT"
  exit 1
fi

echo "x7_pose TCP smoke OK"
