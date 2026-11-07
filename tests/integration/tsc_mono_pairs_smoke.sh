#!/usr/bin/env bash
# Smoke test of apps/bus/tsc_mono_pairs: pinned to CPU 0 for 200 ms at
# 10 ms, it appends its header and about 20 bracketed pairs, each read on
# CPU 0, with nondecreasing monotonic times and TSC counts and a bracket
# under 1 ms; SIGTERM ends an open-ended run with status 0; bad arguments
# exit 2. No timing bound beyond that: the test suite runs in parallel.
#
# usage: tsc_mono_pairs_smoke.sh <tsc_mono_pairs> <scratch dir>
set -u
BIN=$1
ROOT=$2
rm -rf "$ROOT"
mkdir -p "$ROOT"
fail() { echo "FAILED: $1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }

"$BIN" --output "$ROOT/pairs.csv" --cpu 0 --interval-ms 10 --duration-ms 200 ||
  fail "the timed run exited $?"
grep -q '^# tsc_mono_pairs cpu=0 interval_ms=10 tries=16 pid=' "$ROOT/pairs.csv" || fail "no header" "$ROOT/pairs.csv"
awk -F, '
  /^#/ { next }
  NF != 4 || $4 != 0 { bad = "a malformed row or another CPU: " $0; exit }
  $3 < $1 || $3 - $1 > 1000000 { bad = "a bracket out of range: " $0; exit }
  n && ($1 < a || $2 < t) { bad = "time going backwards: " $0; exit }
  { a = $1; t = $2; n++ }
  END { if (bad) { print bad; exit 1 } if (n < 10 || n > 40) { print n " rows"; exit 1 } }
' "$ROOT/pairs.csv" || fail "the pairs" "$ROOT/pairs.csv"

"$BIN" --output "$ROOT/open.csv" --interval-ms 5 &
pid=$!
sleep 0.3
kill -TERM "$pid"
wait "$pid" || fail "SIGTERM did not end the run with status 0"
[ "$(grep -vc '^#' "$ROOT/open.csv")" -ge 5 ] || fail "the open-ended run wrote too little" "$ROOT/open.csv"

for bad in "--cpu" "--output $ROOT/x.csv --interval-ms 0" "--output $ROOT/x.csv --tries 1001" "--interval-ms 5"; do
  # shellcheck disable=SC2086
  "$BIN" $bad >/dev/null 2>&1
  [ $? -eq 2 ] || fail "'$bad' was not refused with status 2"
done
echo "tsc_mono_pairs smoke OK"
