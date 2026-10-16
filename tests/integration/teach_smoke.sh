#!/usr/bin/env bash
# Exercise torque-off and gravity-compensated teaching over dxl_emu.
set -eu

EMU="$1"
TEACH="$2"
INSPECT="$3"
CONFIG="$4"
ROOT="$5"
LINK="$ROOT/ttyDXL"
BUNDLE="$ROOT/torque-bundle"

rm -rf "$ROOT"
mkdir -p "$ROOT"
"$EMU" --link "$LINK" >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

# Passive setup takes finite bus time. Delay the first marker until after the
# start prompt, then stop the taught motion independently.
(sleep 1.2; echo; sleep 0.3; echo) | \
  "$TEACH" --config "$CONFIG" --port "$LINK" --max-duration 1 \
    --sample-rate 50 --bundle "$BUNDLE" >"$ROOT/torque.log" 2>&1

for path in source.toml teach.toml hardware.toml trajectory.zvs recording.csv \
    result.toml manifest.toml model/crane_x7.ztk \
    model/meshes/visual/mounting_plate.stl; do
  test -f "$BUNDLE/$path"
done
grep -q 'status = "success"' "$BUNDLE/result.toml"
grep -q ',start,torque-off,' "$BUNDLE/recording.csv"
grep -q ',stop,torque-off,' "$BUNDLE/recording.csv"
grep -q 'output_sample_rate_hz: 50' "$BUNDLE/recording.csv"
test "$("$INSPECT" --port "$LINK" read 2 torque_enable)" = "0"

HASH=$(sha256sum "$BUNDLE/trajectory.zvs" | cut -d ' ' -f 1)
grep -q "$HASH" "$BUNDLE/manifest.toml"
BEFORE=$(sha256sum "$BUNDLE/manifest.toml" | cut -d ' ' -f 1)
if "$TEACH" --config "$ROOT/missing.toml" --bundle "$BUNDLE" \
    >"$ROOT/refuse.log" 2>&1; then
  echo "existing teach bundle was unexpectedly accepted"
  exit 1
fi
grep -q "bundle directory already exists" "$ROOT/refuse.log"
AFTER=$(sha256sum "$BUNDLE/manifest.toml" | cut -d ' ' -f 1)
test "$BEFORE" = "$AFTER"
"$TEACH" --config "$BUNDLE/teach.toml" --check \
  >"$ROOT/bundle-check.log" 2>&1

printf 'keep me\n' >"$ROOT/existing.zvs"
if "$TEACH" --config "$CONFIG" --port "$LINK" \
    --output "$ROOT/existing.zvs" --log "$ROOT/unused.csv" \
    >"$ROOT/output-refuse.log" 2>&1; then
  echo "existing teach output was unexpectedly accepted"
  exit 1
fi
grep -q "motion output already exists" "$ROOT/output-refuse.log"
grep -q "keep me" "$ROOT/existing.zvs"
test ! -e "$ROOT/unused.csv"

# Log failure: a file-size cap makes the kernel refuse the queued log
# writes (EFBIG; SIGXFSZ ignored). The session is not aborted for it; the
# final flush reports the incomplete log and the run exits 1. The 1 s
# duration limit ends the recording, and a 1 Hz output keeps the motion
# file under the cap, so the log is the only failure.
set +e
( (sleep 1.2; echo; sleep 2) | (
  ulimit -f 4
  trap '' XFSZ
  exec "$TEACH" --config "$CONFIG" --port "$LINK" --max-duration 1 \
    --sample-rate 1 --output "$ROOT/capped.zvs" --log "$ROOT/capped.csv"
) ) >"$ROOT/capped.log" 2>&1
CAPPED_RC=$?
set -e
if [ "$CAPPED_RC" -eq 0 ]; then
  echo "capped teach log: expected a failing exit"; cat "$ROOT/capped.log"
  exit 1
fi
grep -q "x7_teach: cannot finish teach log" "$ROOT/capped.log" || {
  echo "capped teach log: missing the incomplete-log error"
  cat "$ROOT/capped.log"; exit 1;
}
if grep -q "recorded motion rejected" "$ROOT/capped.log"; then
  echo "capped teach log: the motion failed too"; cat "$ROOT/capped.log"
  exit 1
fi
test -s "$ROOT/capped.zvs"
[ "$(stat -c %s "$ROOT/capped.csv")" -le 4096 ]
test "$("$INSPECT" --port "$LINK" read 2 torque_enable)" = "0"

(sleep 1.5; echo; sleep 0.4; echo; sleep 0.4; echo) | \
  "$TEACH" --config "$CONFIG" --mode gravity-compensation \
    --port "$LINK" --max-duration 2 --output "$ROOT/gravity.zvs" \
    --log "$ROOT/gravity.csv" >"$ROOT/gravity.log" 2>&1

test -s "$ROOT/gravity.zvs"
test -s "$ROOT/gravity.csv"
grep -q ',start,gravity-compensation,' "$ROOT/gravity.csv"
grep -q ',stop,gravity-compensation,' "$ROOT/gravity.csv"
grep -q ',support,gravity-compensation,' "$ROOT/gravity.csv"
grep -q '# status: success' "$ROOT/gravity.csv"
test "$("$INSPECT" --port "$LINK" read 2 torque_enable)" = "0"

python3 - "$BUNDLE/recording.csv" "$ROOT/gravity.csv" <<'EOF'
import csv
import sys

for path, expected_valid in ((sys.argv[1], "0"), (sys.argv[2], "1")):
    rows = list(csv.DictReader(
        line for line in open(path) if not line.startswith("#")))
    assert len(rows) >= 2, path
    assert all(row["schema_version"] == "1" for row in rows)
    assert all(row["command_valid"] == expected_valid for row in rows)
    times = [float(row["session_time_s"]) for row in rows]
    assert all(b >= a for a, b in zip(times, times[1:])), path
EOF

echo "x7_teach emulator workflow OK"
