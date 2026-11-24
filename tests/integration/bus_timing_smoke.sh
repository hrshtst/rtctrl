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
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "bus_timing_smoke" "${4:-}" || exit 2  # a new child of an approved base: the only place this test removes
ROOT=$RTCTRL_SCRATCH
LINK="$ROOT/tty"

mkdir -p "$ROOT"
"$EMU" --link "$LINK" --firmware 45 >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
PROBE_PID=
EMU47_PID=
EMUS_PID=
# A probe left stopped by an interrupted stall check is resumed and ended.
trap 'kill "$EMU_PID" 2>/dev/null || true
  if [ -n "$EMU47_PID" ]; then kill "$EMU47_PID" 2>/dev/null || true; fi
  if [ -n "$EMUS_PID" ]; then kill "$EMUS_PID" 2>/dev/null || true; fi
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
# Grid timing is the default, recorded as such, with no adapter section
# (tests/integration/bus_timing_adapter_smoke.sh covers --timing adapter).
grep -q '^timing = "grid"$' "$SUMMARY"
if grep -q '^\[adapter' "$SUMMARY"; then
  fail "a grid run wrote an [adapter] section" "$SUMMARY"
fi
grep -q '^timing: grid (the default)$' "$ROOT/run.log"

# --duration is a monotonic deadline, not a cycle count: at a rate the pty
# may not sustain, the run ends once the deadline passes, never later than
# one late wake plus one cycle's work after it. The bound comes from the
# run's own measurements, not from a wall-clock threshold.
MONO_BEFORE=$(python3 -c 'import time; print(time.monotonic_ns())')
"$PROBE" --port "$LINK" --rate-hz 1000 --profile cbp --duration 0.3 \
  --log "$ROOT/fast.csv" >"$ROOT/fast.log" 2>&1 ||
  fail "1 kHz production-traffic run failed" "$ROOT/fast.log"
MONO_AFTER=$(python3 -c 'import time; print(time.monotonic_ns())')
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

# The log on the system clock: the summary's monotonic_origin_ns places
# every stamp on CLOCK_MONOTONIC (Python's time.monotonic_ns reads it), so
# each falls between the clock read before the probe started and the one
# after it exited, in the log's order. The log itself is unchanged: the
# same columns, the stamps still relative to the epoch.
python3 - "$ROOT/fast.csv.summary.toml" "$ROOT/fast.csv" "$ROOT/fast.log" \
  "$MONO_BEFORE" "$MONO_AFTER" <<'EOF_PY'
import csv
import sys
import tomllib

with open(sys.argv[1], "rb") as stream:
    r = tomllib.load(stream)["result"]
with open(sys.argv[2], newline="") as stream:
    reader = csv.DictReader(stream)
    header = reader.fieldnames
    rows = list(reader)
assert header == (
    "cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,"
    "first_reply_ns,last_reply_ns,read_end_ns,limit_start_ns,"
    "bus_write_start_ns,write_return_ns,drain_return_ns,ack_return_ns,"
    "completion_upper_ns,cycle_end_ns,read_ok,write_ok,ack_ok,"
    "skipped_periods,read_len"
).split(","), header
before, after = int(sys.argv[4]), int(sys.argv[5])
origin = r["monotonic_origin_ns"]
assert before <= origin <= after, (before, origin, after)
assert int(rows[0]["scheduled_wake_ns"]) == 0, rows[0]["scheduled_wake_ns"]
columns = [c for c in header if c.endswith("_ns")]
last = before
for row in rows:
    for c in ("wake_ns", "read_start_ns", "read_end_ns", "cycle_end_ns"):
        t = origin + int(row[c])
        assert last <= t <= after, (row["cycle"], c, last, t, after)
        last = t
    for c in columns:
        if int(row[c]) >= 0:
            assert before <= origin + int(row[c]) <= after, (row["cycle"], c)
first_wake = round(r["first_wake_s"] * 1e9)
assert abs(first_wake - int(rows[0]["wake_ns"])) <= 1, first_wake
assert r["worker_tid"] > 0, r["worker_tid"]
with open(sys.argv[3]) as stream:
    log = stream.read()
line = (f"worker thread {r['worker_tid']}; log epoch at CLOCK_MONOTONIC "
        f"{origin} ns")
assert line in log, line
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
STALL_PID=$PROBE_PID
PROBE_PID=
python3 - "$ROOT/stall.csv.summary.toml" "$ROOT/stall.csv" "$STALL_PID" <<'EOF_PY'
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
# The cycles ran on the probe thread, not the process's main thread.
assert 0 < r["worker_tid"] != int(sys.argv[3]), (r["worker_tid"], sys.argv[3])
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

# The bounded read path (Phase R). Its limits need the path.
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --budget-ms 10 --log "$ROOT/limits.csv" >"$ROOT/limits.log" 2>&1; then
  fail "bounded-read limits were accepted on the legacy path"
fi
grep -q 'need --read-path bounded' "$ROOT/limits.log"
# The emulated servos keep the factory Return Delay Time (250): the
# read-only profile check refuses before the probe's first write.
if "$PROBE" --port "$LINK" --read-path bounded --rate-hz 100 \
    --profile current --duration 0.2 --log "$ROOT/bounded-rdt.csv" \
    >"$ROOT/bounded-rdt.log" 2>&1; then
  fail "the bounded read accepted Return Delay Time 250"
fi
grep -q 'refusing: bounded read: id 2 has Return Delay Time 250' \
  "$ROOT/bounded-rdt.log"
test ! -e "$ROOT/bounded-rdt.csv"
for id in 2 3 4 5 6 7 8 9; do
  test "$("$INSPECT" --port "$LINK" read "$id" torque_enable)" = "0"
  "$INSPECT" --port "$LINK" write "$id" return_delay_time 0 >/dev/null
done
# With Return Delay Time 0 every cycle's grouped read is bounded (the
# initial read before the run is not), with the conservative defaults.
for kind in production calibration; do
  extra=()
  [ "$kind" = calibration ] && extra=(--confirm-writes)
  "$PROBE" --port "$LINK" --read-path bounded --rate-hz 100 \
    --profile position --duration 0.5 "${extra[@]}" \
    --log "$ROOT/bounded-$kind.csv" >"$ROOT/bounded-$kind.log" 2>&1 ||
    fail "bounded $kind run failed" "$ROOT/bounded-$kind.log"
  S="$ROOT/bounded-$kind.csv.summary.toml"
  grep -q '^read_path = "bounded"' "$S"
  grep -q '^\[bounded_read\]' "$S"
  grep -q '^budget_s = 0.035' "$S"
  grep -q '^guard_s = 0.035' "$S"
  test "$(value "$S" reads)" = "$(value "$S" cycles)"
  grep -q '^timeouts = 0' "$S"
  grep -q '^malformed = 0' "$S"
  grep -q '^latency_timer_verified = false' "$S"  # a pty exposes none
  grep -q '^read_failures = 0' "$S"
  grep -q '^torque_off_after = true' "$S"
  grep -q 'the device exposes no latency timer' "$ROOT/bounded-$kind.log"
done
grep -q '^ack_invalid = 0' "$ROOT/bounded-calibration.csv.summary.toml"
grep -q '^expected_latency_timer_ms = 1' "$ROOT/bounded-production.csv.summary.toml"

# The adapter experiment's expected latency timer: 0 to 255, on the
# bounded path only, and recorded (a pty exposes no timer to compare).
for bad in 256 -1 1.5; do
  if "$PROBE" --port "$LINK" --read-path bounded --rate-hz 100 \
      --profile current --duration 0.2 --expect-latency-timer-ms "$bad" \
      --log "$ROOT/latency-bad.csv" >"$ROOT/latency-bad.log" 2>&1; then
    fail "--expect-latency-timer-ms $bad was accepted"
  fi
  grep -q 'an integer from 0 to 255' "$ROOT/latency-bad.log"
done
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --expect-latency-timer-ms 0 --log "$ROOT/latency-legacy.csv" \
    >"$ROOT/latency-legacy.log" 2>&1; then
  fail "--expect-latency-timer-ms was accepted on the legacy path"
fi
grep -q 'expect-latency-timer-ms needs --read-path bounded' "$ROOT/latency-legacy.log"
test ! -e "$ROOT/latency-legacy.csv"
"$PROBE" --port "$LINK" --read-path bounded --rate-hz 100 --profile current \
  --duration 0.3 --expect-latency-timer-ms 0 --log "$ROOT/latency-0.csv" \
  >"$ROOT/latency-0.log" 2>&1 || fail "the run expecting 0 ms failed" "$ROOT/latency-0.log"
grep -q 'expecting an adapter latency timer of 0 ms' "$ROOT/latency-0.log"
grep -q '^expected_latency_timer_ms = 0' "$ROOT/latency-0.csv.summary.toml"
grep -q '^latency_timer_verified = false' "$ROOT/latency-0.csv.summary.toml"
grep -q '^read_path = "legacy"' "$ROOT/run.csv.summary.toml"
grep -q '^feedback = "full"' "$ROOT/run.csv.summary.toml"

# The motion-only read (Phase B2) needs the bounded path; its interval
# needs the motion-only read.
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --feedback motion --log "$ROOT/motion-legacy.csv" \
    >"$ROOT/motion-legacy.log" 2>&1; then
  fail "the motion-only read was accepted on the legacy path"
fi
grep -q 'needs --read-path bounded' "$ROOT/motion-legacy.log"
if "$PROBE" --port "$LINK" --read-path bounded --rate-hz 100 \
    --profile current --duration 0.2 --aux-interval-ms 50 \
    --log "$ROOT/aux-full.csv" >"$ROOT/aux-full.log" 2>&1; then
  fail "an auxiliary interval was accepted with the full read"
fi
grep -q 'needs --feedback motion' "$ROOT/aux-full.log"
# 50 ms at 100 Hz: the full window every fifth cycle, both windows armed
# and every read bounded.
"$PROBE" --port "$LINK" --read-path bounded --rate-hz 100 \
  --profile position --duration 0.5 --feedback motion --aux-interval-ms 50 \
  --log "$ROOT/motion.csv" >"$ROOT/motion.log" 2>&1 ||
  fail "motion-only run failed" "$ROOT/motion.log"
S="$ROOT/motion.csv.summary.toml"
grep -q '^feedback = "motion"' "$S"
grep -q '^full_read_every = 5' "$S"
grep -q '^window_bytes = \[13, 10\]' "$S"
test "$(value "$S" reads)" = "$(value "$S" cycles)"
grep -q '^timeouts = 0' "$S"
grep -q '^read_failures = 0' "$S"
grep -q '^motion_wire_time_worst_s = ' "$S"
# The budget is the longest cycle, the full read's.
grep -q '^read_len = 13' "$S"
grep -q 'motion-only read: .* full read every 5 cycles' "$ROOT/motion.log"
python3 - "$ROOT/motion.csv" "$(value "$S" full_reads)" <<'EOF'
import csv
import sys
rows = list(csv.DictReader(open(sys.argv[1])))
lengths = [int(r["read_len"]) for r in rows]
assert lengths, "no cycles"
want = [13 if (int(r["cycle"]) - 1) % 5 == 0 else 10 for r in rows]
assert lengths == want, lengths
assert lengths.count(13) == int(sys.argv[2]), (lengths.count(13), sys.argv[2])
EOF

# Fast Sync Read (Phase B3): bounded only; the emulated servos (v45) have
# it. With the motion-only read, both windows by Fast Sync Read.
if "$PROBE" --port "$LINK" --rate-hz 100 --profile current --duration 0.2 \
    --read-instruction fast --log "$ROOT/fastsync-legacy.csv" \
    >"$ROOT/fastsync-legacy.log" 2>&1; then
  fail "Fast Sync Read was accepted on the legacy path"
fi
grep -q 'needs --read-path bounded' "$ROOT/fastsync-legacy.log"
"$PROBE" --port "$LINK" --read-path bounded --read-instruction fast \
  --rate-hz 100 --profile cbp --duration 0.5 --feedback motion \
  --aux-interval-ms 50 --log "$ROOT/fastsync.csv" >"$ROOT/fastsync.log" 2>&1 ||
  fail "Fast Sync Read run failed" "$ROOT/fastsync.log"
S="$ROOT/fastsync.csv.summary.toml"
grep -q '^read_instruction = "fast_sync"' "$S"
test "$(value "$S" reads)" = "$(value "$S" cycles)"
grep -q '^timeouts = 0' "$S"
grep -q '^malformed = 0' "$S"
grep -q '^read_failures = 0' "$S"
grep -q '^read_stuffing_max_bytes = 0' "$S"
grep -q 'Fast Sync Read, never stuffed' "$ROOT/fastsync.log"
grep -q '^read_instruction = "sync"' "$ROOT/motion.csv.summary.toml"

# With no auxiliary reads every timed cycle reads the motion window, so
# that cycle is the budget (Fast Sync Read of 10 bytes and a position
# write: 196 bytes); the initial full read is startup traffic, budgeted
# on its own (a legacy Sync Read of 13 bytes: 246 bytes worst case).
"$PROBE" --port "$LINK" --read-path bounded --read-instruction fast \
  --rate-hz 100 --profile position --duration 0.3 --feedback motion \
  --aux-interval-ms 0 --log "$ROOT/fastsync-noaux.csv" \
  >"$ROOT/fastsync-noaux.log" 2>&1 ||
  fail "the run without auxiliary reads failed" "$ROOT/fastsync-noaux.log"
S="$ROOT/fastsync-noaux.csv.summary.toml"
grep -q '^full_read_every = 0' "$S"
grep -q '^full_reads = 0' "$S"
grep -q '^read_len = 10' "$S"
grep -q '^startup_read_len = 13' "$S"
if grep -q '^motion_' "$S"; then
  fail "a run with one read length reported a second budget" "$S"
fi
grep -q 'motion-only read on every cycle' "$ROOT/fastsync-noaux.log"
grep -q '^startup traffic, before the run' "$ROOT/fastsync-noaux.log"
python3 - "$ROOT/fastsync-noaux.csv" "$(value "$S" baudrate)" \
  "$(value "$S" wire_time_worst_s)" \
  "$(value "$S" startup_read_wire_time_worst_s)" <<'EOF'
import csv
import sys
rows = list(csv.DictReader(open(sys.argv[1])))
assert rows, "no cycles"
assert {r["read_len"] for r in rows} == {"10"}, "a full read in the run"
byte_s = 10.0 / int(sys.argv[2])
assert abs(float(sys.argv[3]) - 196 * byte_s) < 1e-12, sys.argv[3]
assert abs(float(sys.argv[4]) - 246 * byte_s) < 1e-12, sys.argv[4]
EOF

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

# --read-ids (the traffic-reduction diagnostic's subset read). Every
# refusal comes before the port is opened: the port named here does not
# exist, so a refusal after opening it would be a different error (exit 1).
NOPORT="$ROOT/no-such-tty"
refused() {  # the expected message, then the probe's arguments
  want=$1
  shift
  st=0
  "$PROBE" --port "$NOPORT" --duration 5 --log "$ROOT/subset-refused.csv" \
    "$@" >"$ROOT/subset-refused.log" 2>&1 || st=$?
  test "$st" = 2 || fail "--read-ids refusal exited $st: $*" "$ROOT/subset-refused.log"
  grep -qF -- "$want" "$ROOT/subset-refused.log" ||
    fail "--read-ids refusal without '$want': $*" "$ROOT/subset-refused.log"
  test ! -e "$ROOT/subset-refused.csv"
}
FAST=(--rate-hz 100 --read-path bounded --read-instruction fast --profile current)
refused "needs --read-path bounded" --rate-hz 100 --profile current --read-ids 2,3
refused "needs --read-instruction fast" --rate-hz 100 --read-path bounded \
  --profile current --read-ids 2,3
for profile in position cbp; do
  refused "needs --profile current" --rate-hz 100 --read-path bounded \
    --read-instruction fast --profile "$profile" --read-ids 2,3
  # A list naming every servo is held to the same rules.
  refused "needs --profile current" --rate-hz 100 --read-path bounded \
    --read-instruction fast --profile "$profile" --read-ids 2,3,4,5,6,7,8,9
done
refused "grid timing only" --rate-hz 1000 --read-path bounded \
  --read-instruction fast --feedback motion --profile current \
  --timing adapter --read-ids 2,3,4,5
refused "grid timing only" --rate-hz 1000 --read-path bounded \
  --read-instruction fast --feedback motion --profile current \
  --timing adapter --read-ids 2,3,4,5,6,7,8,9
refused "production traffic only" "${FAST[@]}" --confirm-writes --read-ids 2,3
refused "production traffic only" "${FAST[@]}" --drain --read-ids 2,3
for bad in "" "3,2" "2,2" "2,,3" ",2" "2," "+2" " 2" "2.0" "x" "0x2" "253" "1000"; do
  refused "strictly increasing" "${FAST[@]}" --read-ids "$bad"
done
refused "id 10 is not a configured servo" "${FAST[@]}" --read-ids 2,10
refused "id 1 is not a configured servo" "${FAST[@]}" --read-ids 1,2
st=0
"$PROBE" --port "$NOPORT" --inventory --read-ids 2 >"$ROOT/subset-inventory.log" 2>&1 || st=$?
test "$st" = 2 || fail "--read-ids with --inventory exited $st" "$ROOT/subset-inventory.log"
grep -q 'for a timing run, not --inventory' "$ROOT/subset-inventory.log"

# The runs, on a v45 bus whose emulator records every instruction it
# received (--arrival-log, written when it stops).
LINKS="$ROOT/ttys"
"$EMU" --link "$LINKS" --firmware 45 --arrival-log "$ROOT/arrivals.csv" \
  >"$ROOT/emus.log" 2>&1 &
EMUS_PID=$!
for _ in $(seq 1 50); do
  [ -e "$LINKS" ] && break
  sleep 0.1
done
[ -e "$LINKS" ] || { echo "arrival-log emulator link never appeared"; exit 1; }
for id in 2 3 4 5 6 7 8 9; do
  "$INSPECT" --port "$LINKS" write "$id" return_delay_time 0 >/dev/null
done

# An unread servo's torque and Drive Mode are still checked before any write.
"$INSPECT" --port "$LINKS" write 8 torque_enable 1 >/dev/null
if "$PROBE" --port "$LINKS" "${FAST[@]}" --duration 0.2 --read-ids 2,3,4,5 \
    --log "$ROOT/subset-torqued.csv" >"$ROOT/subset-torqued.log" 2>&1; then
  fail "a torqued unread servo was accepted"
fi
grep -q 'id 8 has torque enabled' "$ROOT/subset-torqued.log"
test ! -e "$ROOT/subset-torqued.csv"
"$INSPECT" --port "$LINKS" write 8 torque_enable 0 >/dev/null
"$INSPECT" --port "$LINKS" write 9 drive_mode 8 >/dev/null
if "$PROBE" --port "$LINKS" "${FAST[@]}" --duration 0.2 --read-ids 2,3,4,5 \
    --log "$ROOT/subset-drive.csv" >"$ROOT/subset-drive.log" 2>&1; then
  fail "an unread servo's Torque On by Goal Update was accepted"
fi
grep -q 'id 9 has Drive Mode 8 with bit 3' "$ROOT/subset-drive.log"
test ! -e "$ROOT/subset-drive.csv"
"$INSPECT" --port "$LINKS" write 9 drive_mode 0 >/dev/null

# Each run's span on CLOCK_MONOTONIC, for its part of the arrival log.
mono() { python3 -c 'import time; print(time.monotonic_ns())'; }
SPANS="$ROOT/spans.txt"
: >"$SPANS"
subset_run() {  # name, then the probe's arguments
  name=$1
  shift
  before=$(mono)
  "$PROBE" --port "$LINKS" --duration 0.5 --log "$ROOT/$name.csv" "$@" \
    >"$ROOT/$name.log" 2>&1 || fail "the $name run failed" "$ROOT/$name.log"
  echo "$name $before $(mono)" >>"$SPANS"
}
MOTION=("${FAST[@]}" --feedback motion --aux-interval-ms 0)
subset_run r2345 "${MOTION[@]}" --read-ids 2,3,4,5
subset_run r2 "${MOTION[@]}" --read-ids 2
subset_run r2345-full "${FAST[@]}" --read-ids 2,3,4,5
subset_run r6789 "${MOTION[@]}" --read-ids 6,7,8,9
subset_run all-listed "${MOTION[@]}" --read-ids 2,3,4,5,6,7,8,9
subset_run default "${MOTION[@]}"
grep -q '^read set (--read-ids): ids 2,3,4,5, 4 of 8 servos; the goal write (zero current) and every check: all 8$' \
  "$ROOT/r2345.log"
if grep -q '^read set' "$ROOT/default.log"; then
  fail "a run without --read-ids reported a read set" "$ROOT/default.log"
fi
kill -TERM "$EMUS_PID"
wait "$EMUS_PID" || fail "the arrival-log emulator failed" "$ROOT/emus.log"
EMUS_PID=

# Per run, in the emulator's order:
# - before the initial Sync Read, Torque Enable and Drive Mode of every servo
#   are read, and the indirect setup writes every servo;
# - the initial read is a Sync Read of every servo's full window;
# - then each cycle is one Fast Sync Read of the read set and one Sync Write
#   of zero goal current (addr 237) to all eight, as many as the log's rows;
# - after it, Torque Enable and Drive Mode of every servo are read again;
# - nothing writes Torque Enable, Drive Mode or the Bus Watchdog.
# The summary records the read set and counts the read's servos apart from
# the write's; the log's header and every row's read_len are the run's.
python3 - "$ROOT/arrivals.csv" "$SPANS" "$ROOT" <<'EOF_PY'
import csv
import sys
import tomllib

FAST_SYNC_READ, SYNC_READ, SYNC_WRITE, READ, WRITE = 0x8A, 0x82, 0x83, 0x02, 0x03
ALL = [2, 3, 4, 5, 6, 7, 8, 9]
TORQUE, DRIVE, WATCHDOG, GOAL = 64, 10, 98, 237
HEADER = ("cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,"
          "first_reply_ns,last_reply_ns,read_end_ns,limit_start_ns,"
          "bus_write_start_ns,write_return_ns,drain_return_ns,ack_return_ns,"
          "completion_upper_ns,cycle_end_ns,read_ok,write_ok,ack_ok,"
          "skipped_periods,read_len").split(",")
WANT = {  # name: (the read set, read_len, summary keys present)
    "r2345": ([2, 3, 4, 5], 10, True),
    "r2": ([2], 10, True),
    "r2345-full": ([2, 3, 4, 5], 13, True),
    "r6789": ([6, 7, 8, 9], 10, True),
    "all-listed": (ALL, 10, True),
    "default": (ALL, 10, False),
}
with open(sys.argv[1], newline="") as f:
    arrivals = list(csv.DictReader(f))
for a in arrivals:
    a["at"], a["instruction"], a["id"] = int(a["arrival_ns"]), int(a["instruction"]), int(a["id"])
    a["addr"], a["data_length"] = int(a["addr"]), int(a["data_length"])
    a["ids"] = [int(x) for x in a["ids"].split()]



spans = [line.split() for line in open(sys.argv[2])]
assert [s[0] for s in spans] == list(WANT), spans
root = sys.argv[3]
for name, before, after in spans:
    read_set, read_len, keyed = WANT[name]
    part = [a for a in arrivals if int(before) <= a["at"] <= int(after)]
    with open(f"{root}/{name}.csv.summary.toml", "rb") as f:
        s = tomllib.load(f)
    with open(f"{root}/{name}.csv", newline="") as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames == HEADER, (name, reader.fieldnames)
        rows = list(reader)
    cycles = s["result"]["cycles"]
    assert len(rows) == cycles > 0, (name, len(rows), cycles)
    assert {int(r["read_len"]) for r in rows} == {read_len}, name
    assert all(r["read_ok"] == "1" and r["write_ok"] == "1" for r in rows), name
    assert s["result"]["torque_off_before"] is True and s["result"]["torque_off_after"] is True, name
    initial = [i for i, a in enumerate(part) if a["instruction"] == SYNC_READ]
    assert len(initial) == 1, (name, initial)
    k = initial[0]
    assert part[k]["ids"] == ALL and part[k]["data_length"] == 13, (name, part[k])
    cyc = part[k + 1:k + 1 + 2 * cycles]
    for j in range(cycles):
        r, w = cyc[2 * j], cyc[2 * j + 1]
        assert r["instruction"] == FAST_SYNC_READ and r["ids"] == read_set, (name, j, r)
        assert r["data_length"] == read_len and r["addr"] == 224, (name, j, r)
        assert w["instruction"] == SYNC_WRITE and w["ids"] == ALL, (name, j, w)
        assert w["addr"] == GOAL and w["data"] == "00" * 16, (name, j, w)
    pre, post = part[:k], part[k + 1 + 2 * cycles:]
    assert post and all(a["instruction"] == READ for a in post), (name, post[:3])
    for where, seq in (("before", pre), ("after", post)):
        for addr in (TORQUE, DRIVE):
            read = {a["id"] for a in seq if a["instruction"] == READ and a["addr"] == addr}
            assert read == set(ALL), (name, where, addr, read)
    # The indirect setup writes every servo's indirect-address slots (168 on).
    assert {a["id"] for a in pre if a["instruction"] == WRITE} == set(ALL), name
    assert all(a["addr"] >= 168 for a in pre if a["instruction"] == WRITE), name
    writes = [a for a in part if a["instruction"] in (WRITE, SYNC_WRITE)]
    assert not [a for a in writes if a["addr"] in (TORQUE, DRIVE, WATCHDOG)], name
    assert {a["addr"] for a in writes if a["instruction"] == SYNC_WRITE} == {GOAL}, name
    b = s["budget"]
    n = len(read_set)
    assert b["read_instruction_bytes"] == 14 + n, (name, b)
    assert b["read_reply_bytes"] == 8 + n * (read_len + 4), (name, b)
    assert b["write_bytes"] == 38, (name, b)
    if keyed:
        assert s["read_ids"] == read_set, (name, s.get("read_ids"))
        assert (b["read_servos"], b["write_servos"]) == (n, 8), (name, b)
    else:
        assert "read_ids" not in s and "read_servos" not in b and "write_servos" not in b, name
    assert len(s["servo"]) == 8, name
    print(f"{name}: {cycles} cycles, each a read of {read_set} and a zero write to all eight")
EOF_PY

scratch_release
echo "bus timing probe OK"
