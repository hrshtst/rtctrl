#!/usr/bin/env bash
# Hardware x7_follow over dxl_emu with references built from the measured
# posture: a hold and a wrist sweep complete with no homing or correction
# cycle at all, the generated
# reference is archived next to the log, and the refusals happen before
# the bus is touched.
set -eu

EMU="$1"
FOLLOW="$2"
PLAN="$3"
SRC="$4"
ROOT="$5"
LINK="$ROOT/tty"

rm -rf "$ROOT"
mkdir -p "$ROOT"
"$PLAN" --config "$SRC/config/ptp_example.toml" --output "$ROOT/ptp.zvs" \
  --no-diagnostics --motion-time 0.5 >"$ROOT/plan.log" 2>&1
sed -e "s|^model = .*|model = \"$SRC/models/crane_x7/crane_x7.ztk\"|" \
    -e "s|^reference = .*|reference = \"$ROOT/ptp.zvs\"|" \
    -e "s|^hardware_config = .*|hardware_config = \"$SRC/config/crane_x7.toml\"|" \
    -e "s|^wait_time_s = .*|wait_time_s = 0.2|" \
    "$SRC/config/follow_example.toml" >"$ROOT/follow.toml"

"$EMU" --link "$LINK" >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

run() {
  local name="$1"
  shift
  if ! "$FOLLOW" --config "$ROOT/follow.toml" --port "$LINK" \
      --log "$ROOT/$name.csv" "$@" </dev/null >"$ROOT/$name.log" 2>&1; then
    echo "x7_follow $name failed"
    cat "$ROOT/$name.log"
    exit 1
  fi
  grep -q '^follow hardware: success' "$ROOT/$name.log"
  test -f "$ROOT/$name.csv.reference.zvs"
  # Never homed: the first command is a tracking one, and no row is a
  # homing or home-correction cycle.
  test "$(sed -n 2p "$ROOT/$name.csv" | cut -d, -f3)" = tracking
  if cut -d, -f3 "$ROOT/$name.csv" | grep -qx 'home\|home_correction'; then
    echo "x7_follow $name homed"
    exit 1
  fi
}

run hold --hold-current 1
grep -q '^posture reference: hold the measured posture for 1 s' \
  "$ROOT/hold.log"
test "$(wc -l <"$ROOT/hold.csv.reference.zvs")" = 101

run sweep --sweep-current 6:0.2:2
grep -q '^posture reference: sweep joint 6 by +0.2 rad and back over 2 s' \
  "$ROOT/sweep.log"
test "$(wc -l <"$ROOT/sweep.csv.reference.zvs")" = 201

# Refusals before the bus: an existing posture reference, a filtered
# reference, and a reference file alongside.
printf 'keep\n' >"$ROOT/again.csv.reference.zvs"
if "$FOLLOW" --config "$ROOT/follow.toml" --port "$LINK" \
    --log "$ROOT/again.csv" --hold-current 1 </dev/null \
    >"$ROOT/again.log" 2>&1; then
  echo "an existing posture reference was overwritten"
  exit 1
fi
test "$(cat "$ROOT/again.csv.reference.zvs")" = keep
test ! -e "$ROOT/again.csv"
if "$FOLLOW" --config "$ROOT/follow.toml" --port "$LINK" \
    --log "$ROOT/filtered.csv" --hold-current 1 --filter low-pass \
    </dev/null >"$ROOT/filtered.log" 2>&1; then
  echo "a filtered posture reference was accepted"
  exit 1
fi
grep -q 'filter = none' "$ROOT/filtered.log"
test ! -e "$ROOT/filtered.csv"

echo "follow posture references OK"
