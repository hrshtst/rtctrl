#!/usr/bin/env bash
# Hardware x7_follow over dxl_emu with --timing-log: the run completes,
# the timing log has one complete row per background cycle, its run
# summary's counters agree with the per-cycle flags, and the
# end-of-run statistics include the fault counts. Functional only: a pty
# answers instantly, so the timing values themselves mean nothing here.
set -eu

EMU="$1"
FOLLOW="$2"
PLAN="$3"
SRC="$4"
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "follow_timing_smoke" "${5:-}" || exit 2  # a new child of an approved base: the only place this test removes
ROOT=$RTCTRL_SCRATCH
LINK="$ROOT/tty"

mkdir -p "$ROOT"
"$PLAN" --config "$SRC/config/ptp_example.toml" --output "$ROOT/ptp.zvs" \
  --no-diagnostics --motion-time 0.5 >"$ROOT/plan.log" 2>&1
sed -e "s|^model = .*|model = \"$SRC/models/crane_x7/crane_x7.ztk\"|" \
    -e "s|^reference = .*|reference = \"$ROOT/ptp.zvs\"|" \
    -e "s|^hardware_config = .*|hardware_config = \"$SRC/config/crane_x7.toml\"|" \
    -e "s|^velocity_limit = .*|velocity_limit = 1.0|" \
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

if ! "$FOLLOW" --config "$ROOT/follow.toml" --port "$LINK" \
    --log "$ROOT/hardware.csv" --timing-log "$ROOT/timing.csv" \
    </dev/null >"$ROOT/follow.log" 2>&1; then
  echo "x7_follow over the emulator failed"
  cat "$ROOT/follow.log"
  exit 1
fi
grep -q '^follow hardware: success' "$ROOT/follow.log"
grep -q '^hardware faults: 0 read failures, 0 write failures' \
  "$ROOT/follow.log"
grep -q ', 0 timing records dropped$' "$ROOT/follow.log"
grep -q '^host-measured phases:' "$ROOT/follow.log"

python3 - "$ROOT/timing.csv" <<'PY'
import csv
import sys

with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) > 50, len(rows)
for i, row in enumerate(rows, start=1):
    assert int(row["cycle"]) == i, (i, row["cycle"])
    assert row["segment"] == "1", row  # follow never restarts its thread
    assert row["command_kind"] in {"fresh", "fallback", "none"}, row
fresh = [row for row in rows if row["command_kind"] == "fresh"]
assert len(fresh) > 0.9 * len(rows), (len(fresh), len(rows))
for row in fresh:
    # A fresh command carries this cycle's acquisition, and its stamps
    # run in cycle order.
    assert row["applied_source_feedback_seq"] == row["feedback_seq"], row
    stamps = [int(row[k]) for k in (
        "read_start_ns", "read_end_ns", "window_open_ns",
        "controller_wake_ns", "submit_ns", "limit_start_ns",
        "bus_write_start_ns", "write_return_ns", "cycle_end_ns")]
    assert stamps == sorted(stamps), row
PY

# The run summary: the verdict, and controller counters that agree
# exactly with the per-cycle flags of the cycles they cover.
python3 - "$ROOT/timing.csv" "$ROOT/timing.csv.summary.toml" <<'PY'
import csv
import sys
import tomllib

with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))
with open(sys.argv[2], "rb") as stream:
    run = tomllib.load(stream)
assert run["format"] == "rtctrl-cycle-timing-run", run
assert run["app"] == "x7_follow" and run["status"] == "success", run
assert run["successful"] and run["log_complete"] and not run["escalated"]
end, final = run["controller_end"], run["final"]
assert final["cycles"] == len(rows), (final["cycles"], len(rows))
assert 0 < end["cycles"] <= final["cycles"], end
flags = [int(r["deadline_miss"]) for r in rows]
assert sum(flags[:end["cycles"]]) == end["controller_deadline_misses"], end
assert sum(flags) == final["controller_deadline_misses"], final
assert end["stale_submissions"] == 0, end
PY

# An existing timing log, or its summary, is refused before the bus is
# touched.
if "$FOLLOW" --config "$ROOT/follow.toml" --port "$LINK" \
    --log "$ROOT/second.csv" --timing-log "$ROOT/timing.csv" \
    </dev/null >"$ROOT/refuse.log" 2>&1; then
  echo "an existing timing log was accepted"
  exit 1
fi
test ! -e "$ROOT/second.csv"
printf 'keep\n' >"$ROOT/third.csv.summary.toml"
if "$FOLLOW" --config "$ROOT/follow.toml" --port "$LINK" \
    --log "$ROOT/third-hw.csv" --timing-log "$ROOT/third.csv" \
    </dev/null >"$ROOT/refuse-summary.log" 2>&1; then
  echo "an existing timing summary was accepted"
  exit 1
fi
grep -q 'timing summary already exists' "$ROOT/refuse-summary.log"
test ! -e "$ROOT/third.csv"
test "$(cat "$ROOT/third.csv.summary.toml")" = keep

scratch_release
echo "follow timing log OK"
