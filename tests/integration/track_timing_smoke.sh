#!/usr/bin/env bash
# Hardware x7_track over dxl_emu with --timing-log. The run stops and
# restarts the background thread at its position-to-current switch with
# one timing log open: the log must number its cycles 1, 2, ... across the
# restart and mark the two thread runs as segments 1 and 2. Functional
# only: a pty answers instantly, so the timing values mean nothing here.
set -eu

EMU="$1"
TRACK="$2"
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
    "$SRC/config/track_example.toml" >"$ROOT/track.toml"

"$EMU" --link "$LINK" >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

if ! "$TRACK" --config "$ROOT/track.toml" --port "$LINK" \
    --log "$ROOT/hardware.csv" --timing-log "$ROOT/timing.csv" \
    </dev/null >"$ROOT/track.log" 2>&1; then
  echo "x7_track over the emulator failed"
  cat "$ROOT/track.log"
  exit 1
fi
grep -q 'switching stationary to current mode' "$ROOT/track.log"
grep -q ', 0 timing records dropped$' "$ROOT/track.log"

python3 - "$ROOT/timing.csv" <<'PY'
import csv
import sys

with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) > 50, len(rows)
segments = [int(row["segment"]) for row in rows]
for i, row in enumerate(rows, start=1):
    assert int(row["cycle"]) == i, (i, row["cycle"])
# Position home, then the current-mode run: two segments, in order.
assert segments[0] == 1 and segments[-1] == 2, (segments[0], segments[-1])
assert segments == sorted(segments), "segments out of order"
switch = segments.index(2)
assert switch > 10 and len(rows) - switch > 10, switch
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
assert run["app"] == "x7_track" and run["status"] == "success", run
assert run["successful"] and run["log_complete"] and not run["escalated"]
end, final = run["controller_end"], run["final"]
assert final["cycles"] == len(rows), (final["cycles"], len(rows))
assert 0 < end["cycles"] <= final["cycles"], end
flags = [int(r["deadline_miss"]) for r in rows]
assert sum(flags[:end["cycles"]]) == end["controller_deadline_misses"], end
assert sum(flags) == final["controller_deadline_misses"], final
assert end["stale_submissions"] == 0, end
PY

echo "track timing log OK"
