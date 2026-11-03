#!/usr/bin/env bash
# x7_bus_timing over dxl_emu: inventory, production-traffic and
# calibration runs, exclusive outputs, and the torque-off contract (it
# refuses a torqued servo or one whose Drive Mode would switch torque on
# at a goal write, and never changes either). The emulated servos run
# firmware v45, where Torque On by Goal Update exists; a second bus on
# firmware v47 in operating mode 0 checks calibration against the goal
# overwrite observed there. Functional only: a pty answers instantly, so
# no timing value here means anything.
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
PROBE_PID=
EMU47_PID=
# A probe left stopped by an interrupted stall check is resumed and ended.
trap 'kill "$EMU_PID" 2>/dev/null || true
  if [ -n "$EMU47_PID" ]; then kill "$EMU47_PID" 2>/dev/null || true; fi
  if [ -n "$PROBE_PID" ]; then
    kill -CONT "$PROBE_PID" 2>/dev/null || true
    kill "$PROBE_PID" 2>/dev/null || true
  fi' EXIT
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

# Summary value of KEY (the first match) in a TOML file.
value() { sed -n "s/^$2 = //p" "$1" | head -1; }

# Production traffic: one row per cycle, complete outputs, torque off. The
# 100 period slots before the 1 s deadline are either run or skipped (an
# overrun under test-suite load skips some), never more cycles than slots.
"$PROBE" --port "$LINK" --rate-hz 100 --profile position --duration 1 \
  --log "$ROOT/run.csv" >"$ROOT/run.log" 2>&1 ||
  fail "production-traffic run failed" "$ROOT/run.log"
head -1 "$ROOT/run.csv" | grep -q '^cycle,scheduled_wake_ns,wake_ns,read_start_ns'
SUMMARY="$ROOT/run.csv.summary.toml"
CYCLES=$(value "$SUMMARY" cycles)
test "$(wc -l <"$ROOT/run.csv")" = $((CYCLES + 1))
test "$CYCLES" -le 100
test $((CYCLES + $(value "$SUMMARY" skipped_periods))) -ge 100
grep -q '^kind = "production-traffic"' "$SUMMARY"
grep -q '^log_complete = true' "$SUMMARY"
grep -q '^elapsed_s = ' "$SUMMARY"
grep -q '^read_failures = 0' "$SUMMARY"
grep -q '^torque_off_after = true' "$SUMMARY"
grep -q '^\[budget\]' "$SUMMARY"
grep -q '^\[histogram.host_transaction\]' "$SUMMARY"

# --duration is a monotonic deadline, not a cycle count: at a rate the pty
# may not sustain, the run ends once the deadline passes, never later than
# one late wake plus one cycle's work after it. The bound comes from the
# run's own measurements, not from a wall-clock threshold.
"$PROBE" --port "$LINK" --rate-hz 1000 --profile cbp --duration 0.3 \
  --log "$ROOT/fast.csv" >"$ROOT/fast.log" 2>&1 ||
  fail "1 kHz production-traffic run failed" "$ROOT/fast.log"
grep -q '^ran [0-9.]* s (deadline 0.300 s)' "$ROOT/fast.log"
python3 - "$ROOT/fast.csv.summary.toml" "$ROOT/fast.csv" <<'EOF_PY'
import sys
import tomllib

with open(sys.argv[1], "rb") as stream:
    s = tomllib.load(stream)
r, h = s["result"], s["histogram"]
with open(sys.argv[2]) as stream:
    rows = sum(1 for _ in stream) - 1
assert rows == r["cycles"], (rows, r["cycles"])
late, work = h["wake_lateness"]["max_s"], h["host_work"]["max_s"]
bound = s["duration_s"] + late + work + 1e-3
assert r["elapsed_s"] <= bound, (r["elapsed_s"], bound)
assert r["last_wake_s"] <= s["duration_s"] + late + 1e-3, r["last_wake_s"]
assert r["elapsed_s"] >= s["duration_s"] - 2.0 / s["rate_hz"], r["elapsed_s"]
EOF_PY

# The deadline also holds against the clock after a wake: a probe stopped
# in its sleep until past the deadline starts no further transaction. The
# stop lands mid-run (rows are being logged) and lasts past the deadline;
# at 20 Hz it almost always lands in the sleep. Either way no cycle wakes
# at or after the deadline, and the stall is on record: the late wake in
# the summary, or a cycle that ended past the deadline.
"$PROBE" --port "$LINK" --rate-hz 20 --profile position --duration 2 \
  --log "$ROOT/stall.csv" >"$ROOT/stall.log" 2>&1 &
PROBE_PID=$!
for _ in $(seq 1 200); do
  [ -e "$ROOT/stall.csv" ] && [ "$(wc -l <"$ROOT/stall.csv")" -ge 4 ] && break
  sleep 0.02
done
kill -STOP "$PROBE_PID"
sleep 2.5
kill -CONT "$PROBE_PID"
wait "$PROBE_PID" || fail "stalled run failed" "$ROOT/stall.log"
PROBE_PID=
python3 - "$ROOT/stall.csv.summary.toml" "$ROOT/stall.csv" <<'EOF_PY'
import csv
import sys
import tomllib

with open(sys.argv[1], "rb") as stream:
    s = tomllib.load(stream)
r = s["result"]
with open(sys.argv[2], newline="") as stream:
    rows = list(csv.DictReader(stream))
deadline_ns = s["duration_s"] * 1e9
assert len(rows) == r["cycles"] >= 3, (len(rows), r["cycles"])
late = [row["cycle"] for row in rows if int(row["wake_ns"]) >= deadline_ns]
assert not late, f"cycles {late} started after the deadline"
assert r["last_wake_s"] < s["duration_s"], r["last_wake_s"]
in_sleep = r["late_wake_stop"] and r["late_wake_lateness_s"] > 0.5
in_cycle = int(rows[-1]["cycle_end_ns"]) >= deadline_ns
assert in_sleep or in_cycle, "the stall left no record"
print("stopped in its sleep" if in_sleep else "stopped mid-cycle")
EOF_PY
if grep -q '^late_wake_stop = true' "$ROOT/stall.csv.summary.toml"; then
  grep -q 'past the deadline: no further cycle started' "$ROOT/stall.log"
fi

# Calibration: every write is acknowledged by a valid Model Number read.
# (The completion-offset histogram is not counted: the emulator replies
# without the 500 us Return Delay Time the bound subtracts, so an offset
# can come out negative here, which the histogram ignores.)
"$PROBE" --port "$LINK" --rate-hz 100 --profile cbp --duration 1 \
  --confirm-writes --drain --log "$ROOT/cal.csv" >"$ROOT/cal.log" 2>&1 ||
  fail "calibration run failed" "$ROOT/cal.log"
grep -q '^kind = "calibration"' "$ROOT/cal.csv.summary.toml"
grep -q '^version = 2' "$ROOT/cal.csv.summary.toml"
grep -q '^ack_register = "model_number"' "$ROOT/cal.csv.summary.toml"
grep -q '^ack_invalid = 0' "$ROOT/cal.csv.summary.toml"
test "$(cut -d, -f18 "$ROOT/cal.csv" | sed 1d | sort -u)" = 1
test "$(wc -l <"$ROOT/cal.csv")" = \
  $(($(value "$ROOT/cal.csv.summary.toml" cycles) + 1))
"$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.5 \
  --confirm-writes --log "$ROOT/cal-current.csv" >"$ROOT/cal-current.log" \
  2>&1 || fail "current calibration failed" "$ROOT/cal-current.log"
grep -q '^ack_invalid = 0' "$ROOT/cal-current.csv.summary.toml"
# A production-traffic run makes no acknowledging read and names none.
if grep -q '^ack_register' "$ROOT/run.csv.summary.toml"; then
  fail "a production-traffic run named an acknowledgement register"
fi

# An existing log is never reused.
RUN_SHA=$(sha256sum "$ROOT/run.csv" | cut -d ' ' -f 1)
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --log "$ROOT/run.csv" >"$ROOT/reuse.log" 2>&1; then
  fail "an existing log was accepted"
fi
test "$RUN_SHA" = "$(sha256sum "$ROOT/run.csv" | cut -d ' ' -f 1)"

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

# Firmware v47 in operating mode 0 overwrites a torque-off goal write with
# present values (observed on the arm), so a goal read-back cannot
# acknowledge a write. The Model Number read still does, for every write.
LINK47="$ROOT/tty47"
"$EMU" --link "$LINK47" --firmware 47 >"$ROOT/emu47.log" 2>&1 &
EMU47_PID=$!
for _ in $(seq 1 50); do
  [ -e "$LINK47" ] && break
  sleep 0.1
done
[ -e "$LINK47" ] || { echo "v47 emulator link never appeared"; exit 1; }
for id in 2 3 4 5 6 7 8 9; do
  "$INSPECT" --port "$LINK47" write "$id" operating_mode 0 >/dev/null
done
PRESENT=$("$INSPECT" --port "$LINK47" read 9 present_position)
"$INSPECT" --port "$LINK47" write 9 goal_position $((PRESENT + 20)) >/dev/null
test "$("$INSPECT" --port "$LINK47" read 9 goal_position)" = "$PRESENT"
"$INSPECT" --port "$LINK47" write 9 goal_current 1 >/dev/null
test "$("$INSPECT" --port "$LINK47" read 9 goal_current)" = "0"
for profile in current position; do
  "$PROBE" --port "$LINK47" --rate-hz 100 --profile "$profile" \
    --duration 0.5 --confirm-writes --log "$ROOT/cal47-$profile.csv" \
    >"$ROOT/cal47-$profile.log" 2>&1 ||
    fail "v47 $profile calibration failed" "$ROOT/cal47-$profile.log"
  grep -q '^ack_invalid = 0' "$ROOT/cal47-$profile.csv.summary.toml"
  test "$(cut -d, -f18 "$ROOT/cal47-$profile.csv" | sed 1d | sort -u)" = 1
done

echo "bus timing probe OK"
