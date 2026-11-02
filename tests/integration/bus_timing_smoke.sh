#!/usr/bin/env bash
# x7_bus_timing over dxl_emu: inventory, production-traffic and
# calibration runs, exclusive outputs, and the torque-off contract (it
# refuses a torqued servo or one whose Drive Mode would switch torque on
# at a goal write, and never changes either). The emulated servos run
# firmware v45, where Torque On by Goal Update exists. Functional only: a
# pty answers instantly, so no timing value here means anything.
set -eu

EMU="$1"
PROBE="$2"
INSPECT="$3"
ROOT="$4"
LINK="$ROOT/tty"

rm -rf "$ROOT"
mkdir -p "$ROOT"
"$EMU" --link "$LINK" --firmware 45 >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

fail() { echo "$1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }

# Inventory: every servo with its bus-timing registers.
"$PROBE" --port "$LINK" --inventory --out "$ROOT/inventory.toml" \
  >"$ROOT/inventory.log" 2>&1 || fail "inventory failed" "$ROOT/inventory.log"
test "$(grep -c '^\[\[servo\]\]' "$ROOT/inventory.toml")" = 8
grep -q '^return_delay_time = 250' "$ROOT/inventory.toml"
grep -q '^status_return_level = 2' "$ROOT/inventory.toml"
grep -q '^firmware = 45' "$ROOT/inventory.toml"
test "$(grep -c '^torque_on_by_goal_update = false' "$ROOT/inventory.toml")" = 8
BEFORE=$(sha256sum "$ROOT/inventory.toml" | cut -d ' ' -f 1)
if "$PROBE" --port "$LINK" --inventory --out "$ROOT/inventory.toml" \
    >"$ROOT/inventory-again.log" 2>&1; then
  fail "an existing inventory was overwritten"
fi
test "$BEFORE" = "$(sha256sum "$ROOT/inventory.toml" | cut -d ' ' -f 1)"

# Production traffic: one row per cycle, complete outputs, torque off.
"$PROBE" --port "$LINK" --rate-hz 100 --profile position --duration 1 \
  --log "$ROOT/run.csv" >"$ROOT/run.log" 2>&1 ||
  fail "production-traffic run failed" "$ROOT/run.log"
test "$(wc -l <"$ROOT/run.csv")" = 101
head -1 "$ROOT/run.csv" | grep -q '^cycle,scheduled_wake_ns,wake_ns,read_start_ns'
SUMMARY="$ROOT/run.csv.summary.toml"
grep -q '^kind = "production-traffic"' "$SUMMARY"
grep -q '^log_complete = true' "$SUMMARY"
grep -q '^cycles = 100' "$SUMMARY"
grep -q '^read_failures = 0' "$SUMMARY"
grep -q '^torque_off_after = true' "$SUMMARY"
grep -q '^\[budget\]' "$SUMMARY"
grep -q '^\[histogram.host_transaction\]' "$SUMMARY"

# Calibration: every acknowledged write matched what was written.
"$PROBE" --port "$LINK" --rate-hz 100 --profile cbp --duration 1 \
  --confirm-writes --drain --log "$ROOT/cal.csv" >"$ROOT/cal.log" 2>&1 ||
  fail "calibration run failed" "$ROOT/cal.log"
grep -q '^kind = "calibration"' "$ROOT/cal.csv.summary.toml"
grep -q '^ack_mismatches = 0' "$ROOT/cal.csv.summary.toml"
grep -A1 '^\[histogram.completion_offset\]' "$ROOT/cal.csv.summary.toml" |
  grep -q '^count = 100'
"$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.5 \
  --confirm-writes --log "$ROOT/cal-current.csv" >"$ROOT/cal-current.log" \
  2>&1 || fail "current calibration failed" "$ROOT/cal-current.log"
grep -q '^ack_mismatches = 0' "$ROOT/cal-current.csv.summary.toml"

# An existing log is never reused.
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --log "$ROOT/run.csv" >"$ROOT/reuse.log" 2>&1; then
  fail "an existing log was accepted"
fi
test "$(wc -l <"$ROOT/run.csv")" = 101

# A torqued servo is refused, and the probe leaves its torque alone.
"$INSPECT" --port "$LINK" write 5 torque_enable 1 >/dev/null
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --log "$ROOT/torqued.csv" >"$ROOT/torqued.log" 2>&1; then
  fail "a torqued servo was accepted"
fi
grep -q 'id 5 has torque enabled' "$ROOT/torqued.log"
test ! -e "$ROOT/torqued.csv"
test "$("$INSPECT" --port "$LINK" read 5 torque_enable)" = "1"
"$INSPECT" --port "$LINK" write 5 torque_enable 0 >/dev/null
for id in 2 3 4 5 6 7 8 9; do
  test "$("$INSPECT" --port "$LINK" read "$id" torque_enable)" = "0"
done

# Torque On by Goal Update is refused before any write: on this firmware
# the probe's first goal write would switch torque on.
"$INSPECT" --port "$LINK" write 7 drive_mode 8 >/dev/null
"$PROBE" --port "$LINK" --inventory >"$ROOT/inventory-drive.log" 2>&1 ||
  fail "inventory with drive mode 8 failed" "$ROOT/inventory-drive.log"
grep -q 'warning: id 7 has Torque On by Goal Update' "$ROOT/inventory-drive.log"
for profile in current position cbp; do
  if "$PROBE" --port "$LINK" --rate-hz 100 --profile "$profile" \
      --duration 0.2 --log "$ROOT/drive-$profile.csv" \
      >"$ROOT/drive-$profile.log" 2>&1; then
    fail "Torque On by Goal Update was accepted ($profile)"
  fi
  grep -q 'id 7 has Drive Mode 8 with bit 3' "$ROOT/drive-$profile.log"
  test ! -e "$ROOT/drive-$profile.csv"
done
for id in 2 3 4 5 6 7 8 9; do
  test "$("$INSPECT" --port "$LINK" read "$id" torque_enable)" = "0"
done
# The hazard is real on the emulated bus: one goal write torques id 7.
"$INSPECT" --port "$LINK" write 7 goal_current 0 >/dev/null
test "$("$INSPECT" --port "$LINK" read 7 torque_enable)" = "1"
"$INSPECT" --port "$LINK" write 7 torque_enable 0 >/dev/null
"$INSPECT" --port "$LINK" write 7 drive_mode 0 >/dev/null
test "$("$INSPECT" --port "$LINK" read 7 drive_mode)" = "0"

# A real-time priority without a dedicated cpu is refused before bus use.
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --rt-priority 10 --log "$ROOT/rt.csv" >"$ROOT/rt.log" 2>&1; then
  fail "a priority without a cpu was accepted"
fi
grep -q 'dedicated cpu' "$ROOT/rt.log"

echo "bus timing probe OK"
