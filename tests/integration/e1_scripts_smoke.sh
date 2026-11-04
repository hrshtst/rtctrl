#!/usr/bin/env bash
# tools/e1_prod.sh and tools/e1_cal.sh over dxl_emu: the refusal without a
# confirmation, the rate climb and its per-run records, resuming without a
# retry, the stop on a probe failure, and calibration runs. Functional only:
# a pty answers instantly, so no verdict here means anything and none is
# asserted. Every session is detached from the terminal (setsid), so no
# prompt can wait on the developer's terminal: a script with no terminal
# ends a climb at its first failed check instead of asking. The loaded and
# real-time conditions need stress-ng and a real-time priority limit, so
# they stay out of this test.
set -eu

EMU="$1"
PROBE="$2"
ROOT="$3"
INSPECT="$4"
LINK="$ROOT/tty"
OUT="$ROOT/out"
DATE=20000101

rm -rf "$ROOT"
mkdir -p "$OUT"
"$EMU" --link "$LINK" --firmware 47 >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

fail() { echo "$1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }

# The quiet-machine gate is lifted: the test suite runs in parallel.
COMMON=(--probe "$PROBE" --port "$LINK" --outdir "$OUT" --duration 1
        --date "$DATE" --max-runnable 100000)
prod() { echo "rate-e1-prod-$1-${2}hz-idle-$DATE-1.csv"; }
cal() { echo "rate-e1-cal-$1-${2}hz-$DATE-1.csv"; }

# Without a confirmation and without a terminal, nothing starts.
if setsid tools/e1_prod.sh "${COMMON[@]}" --conditions idle \
    --profiles current --rates 100 </dev/null >"$ROOT/refuse.log" 2>&1; then
  fail "an unconfirmed production session started" "$ROOT/refuse.log"
fi
grep -q 'no terminal to confirm the session' "$ROOT/refuse.log"
if setsid tools/e1_cal.sh "${COMMON[@]}" --runs current:100 \
    </dev/null >"$ROOT/refuse-cal.log" 2>&1; then
  fail "an unconfirmed calibration session started" "$ROOT/refuse-cal.log"
fi
grep -q 'no terminal to confirm the session' "$ROOT/refuse-cal.log"
[ -z "$(ls -A "$OUT")" ] || fail "a refused session left files"

# The climb: each run leaves its log, summary and record, made through the
# given port; a run whose climb ended has no higher rate after it.
setsid tools/e1_prod.sh "${COMMON[@]}" --yes --conditions idle \
  --profiles current,cbp --rates 100,200 </dev/null >"$ROOT/prod.log" 2>&1 ||
  fail "the production session failed" "$ROOT/prod.log"
grep -q '^E1 production-traffic results' "$ROOT/prod.log"
for profile in current cbp; do
  first="$OUT/$(prod "$profile" 100)"
  for f in "$first" "$first.summary.toml" "$first.run.txt"; do
    [ -f "$f" ] || fail "missing $f" "$ROOT/prod.log"
  done
  grep -q '^kind = "production-traffic"' "$first.summary.toml"
  grep -Eq '^verdict: (pass|fail)' "$first.run.txt" ||
    fail "no verdict in $first.run.txt" "$first.run.txt"
  second="$OUT/$(prod "$profile" 200)"
  if grep -qx 'climb: continue' "$first.run.txt"; then
    grep -Eq '^verdict: (pass|fail)' "$second.run.txt" ||
      fail "the climb went on but $second.run.txt has no verdict"
  else
    grep -q '^climb: ended' "$first.run.txt"
    if [ -e "$second" ] || [ -e "$second.run.txt" ]; then
      fail "a run after the end of the $profile climb"
    fi
    grep -Eq "^  idle +$profile +200 Hz  not run" "$ROOT/prod.log"
  fi
done
for note in "$OUT"/*.run.txt; do
  grep -q "^command: .* --port $LINK " "$note" ||
    fail "$note: a run not made through the given port" "$note"
done

# Resuming re-runs nothing: every attempted run is skipped.
BEFORE=$(ls -l --time-style=+%s.%N "$OUT" | sha256sum)
setsid tools/e1_prod.sh "${COMMON[@]}" --yes --conditions idle \
  --profiles current,cbp --rates 100,200 </dev/null >"$ROOT/resume.log" 2>&1 ||
  fail "the resumed session failed" "$ROOT/resume.log"
[ "$BEFORE" = "$(ls -l --time-style=+%s.%N "$OUT" | sha256sum)" ] ||
  fail "resuming changed the outputs" "$ROOT/resume.log"
grep -q 'skipped (attempted earlier' "$ROOT/resume.log"
if grep -Eq '^  idle .* Hz  (PASS|FAIL)' "$ROOT/resume.log"; then
  fail "resuming re-ran a run" "$ROOT/resume.log"
fi

# A probe that exits nonzero stops the whole session.
if setsid tools/e1_prod.sh "${COMMON[@]}" --port "$ROOT/no_such_tty" --yes \
    --conditions idle --profiles position --rates 100,200 \
    </dev/null >"$ROOT/stop.log" 2>&1; then
  fail "a probe failure did not stop the session" "$ROOT/stop.log"
fi
grep -q 'session stopped (stop rule)' "$ROOT/stop.log"
grep -q '^verdict: probe exit' "$OUT/$(prod position 100).run.txt"
[ ! -e "$OUT/$(prod position 200).run.txt" ] ||
  fail "a run started after the session stopped" "$ROOT/stop.log"

# Calibration: acknowledged writes and drain stamps, summarized, never
# judged.
setsid tools/e1_cal.sh "${COMMON[@]}" --yes --runs current:100 \
  --runs cbp:200 </dev/null >"$ROOT/cal.log" 2>&1 ||
  fail "the calibration session failed" "$ROOT/cal.log"
for run in "current 100" "cbp 200"; do
  # shellcheck disable=SC2086
  base="$OUT/$(cal $run)"
  grep -q '^kind = "calibration"' "$base.summary.toml"
  grep -q '^confirm_writes = true' "$base.summary.toml"
  grep -q '^drain = true' "$base.summary.toml"
  grep -q '^verdict: done' "$base.run.txt" ||
    fail "no verdict in $base.run.txt" "$base.run.txt"
  grep -q 'envelope candidate (1.25 x max)' "$base.run.txt"
  if grep -q -- '--check' "$base.run.txt"; then
    fail "a calibration run was judged" "$base.run.txt"
  fi
  # The record states what was asked for and what the probe was granted.
  grep -q '^scheduling requested: default' "$base.run.txt"
  grep -q '^scheduling granted: SCHED_OTHER' "$base.run.txt"
done

# Real-time calibration (planned only: CI cannot grant SCHED_FIFO): the
# probe gets the policy and the CPU, and the names say rt-idle. Half of
# the pair is refused.
setsid tools/e1_cal.sh "${COMMON[@]}" --dry-run --runs current:100 \
  --rt-priority 80 --rt-cpu 0 </dev/null >"$ROOT/cal-rt.log" 2>&1 ||
  fail "the real-time calibration plan failed" "$ROOT/cal-rt.log"
grep -q -- "--rt-priority 80 --rt-cpu 0" "$ROOT/cal-rt.log"
grep -q "rate-e1-cal-current-100hz-rt-idle-$DATE-1.csv" "$ROOT/cal-rt.log"
grep -q 'scheduling SCHED_FIFO 80 pinned to CPU 0' "$ROOT/cal-rt.log"
if setsid tools/e1_cal.sh "${COMMON[@]}" --dry-run --runs current:100 \
    --rt-priority 80 </dev/null >"$ROOT/cal-rt-half.log" 2>&1; then
  fail "a priority without a cpu was accepted" "$ROOT/cal-rt-half.log"
fi
grep -q -- '--rt-priority and --rt-cpu go together' "$ROOT/cal-rt-half.log"
# A SCHED_FIFO priority is 1 to 99: the probe reads 0 as ordinary
# scheduling, which a run labelled rt-idle must never get.
for priority in 0 100; do
  if setsid tools/e1_cal.sh "${COMMON[@]}" --dry-run --runs current:100 \
      --rt-priority "$priority" --rt-cpu 0 </dev/null \
      >"$ROOT/cal-rt-$priority.log" 2>&1; then
    fail "calibration accepted priority $priority" "$ROOT/cal-rt-$priority.log"
  fi
  grep -q -- '--rt-priority must be 1 to 99' "$ROOT/cal-rt-$priority.log"
  if setsid tools/e1_prod.sh "${COMMON[@]}" --dry-run --conditions rt-idle \
      --rt-priority "$priority" --rt-cpu 0 </dev/null \
      >"$ROOT/prod-rt-$priority.log" 2>&1; then
    fail "the matrix accepted priority $priority" "$ROOT/prod-rt-$priority.log"
  fi
  grep -q -- '--rt-priority must be 1 to 99' "$ROOT/prod-rt-$priority.log"
done

# --no-climb asks nothing and runs every listed rate, even after a failed
# check: 1000 Hz fails on the pty (its replies alone exceed the period),
# and 1001 Hz still runs.
setsid tools/e1_prod.sh "${COMMON[@]}" --yes --no-climb --attempt 5 \
  --conditions idle --profiles current --rates 1000,1001 </dev/null \
  >"$ROOT/noclimb.log" 2>&1 || fail "the no-climb session failed" \
  "$ROOT/noclimb.log"
for rate in 1000 1001; do
  note="$OUT/rate-e1-prod-current-${rate}hz-idle-$DATE-5.csv.run.txt"
  grep -q '^verdict: fail' "$note" || fail "no failed verdict at $rate Hz" "$note"
  grep -q '^operator: none asked (--no-climb)' "$note"
  grep -qx 'climb: continue' "$note"
  grep -q '^climb rule: none: every listed rate runs' "$note"
done
grep -q 'no climb' "$ROOT/noclimb.log"
if grep -q 'not run\|Did this rate' "$ROOT/noclimb.log"; then
  fail "a no-climb session ended a climb or asked" "$ROOT/noclimb.log"
fi
if setsid tools/e1_prod.sh "${COMMON[@]}" --dry-run --no-climb --auto-stop \
    </dev/null >"$ROOT/noclimb-auto.log" 2>&1; then
  fail "--no-climb was accepted with --auto-stop" "$ROOT/noclimb-auto.log"
fi
grep -q 'exclude each other' "$ROOT/noclimb-auto.log"

# The bounded read path. On the emulator's factory Return Delay Time (250)
# the probe refuses before its first write, and the refusal stops the
# session (stop rule).
if setsid tools/e1_prod.sh "${COMMON[@]}" --yes --read-path bounded \
    --conditions idle --profiles current --rates 100,200 </dev/null \
    >"$ROOT/bounded-refused.log" 2>&1; then
  fail "the bounded session ran on Return Delay Time 250" \
    "$ROOT/bounded-refused.log"
fi
grep -q 'session stopped (stop rule)' "$ROOT/bounded-refused.log"
grep -q 'refusing: bounded read: id 2 has Return Delay Time 250' \
  "$ROOT/bounded-refused.log"
BREF="$OUT/rate-e1-prod-current-100hz-idle-bounded-$DATE-1.csv"
grep -q '^read path: bounded' "$BREF.run.txt"
test ! -e "$BREF"
test ! -e "$OUT/rate-e1-prod-current-200hz-idle-bounded-$DATE-1.csv.run.txt"
# With Return Delay Time 0 (as on the arm), a new series runs bounded.
for id in 2 3 4 5 6 7 8 9; do
  "$INSPECT" --port "$LINK" write "$id" return_delay_time 0 >/dev/null
done
setsid tools/e1_prod.sh "${COMMON[@]}" --yes --read-path bounded \
  --attempt 2 --conditions idle --profiles current --rates 100 </dev/null \
  >"$ROOT/bounded-prod.log" 2>&1 ||
  fail "the bounded production session failed" "$ROOT/bounded-prod.log"
setsid tools/e1_cal.sh "${COMMON[@]}" --yes --read-path bounded \
  --attempt 2 --runs cbp:200 </dev/null >"$ROOT/bounded-cal.log" 2>&1 ||
  fail "the bounded calibration session failed" "$ROOT/bounded-cal.log"
for base in "$OUT/rate-e1-prod-current-100hz-idle-bounded-$DATE-2.csv" \
            "$OUT/rate-e1-cal-cbp-200hz-bounded-$DATE-2.csv"; do
  grep -q '^read_path = "bounded"' "$base.summary.toml"
  grep -q '^timeouts = 0' "$base.summary.toml"
  grep -q '^read path: bounded' "$base.run.txt"
  grep -q 'read path: bounded (budget 35.000 ms, guard 35.000 ms)' \
    "$base.run.txt"
  grep -q -- '--read-path bounded' "$base.run.txt"
done

# The motion-only read (Phase B2): bounded only, its own log names, and
# the summary tool's report per read length.
if tools/e1_prod.sh "${COMMON[@]}" --dry-run --feedback motion \
    >"$ROOT/motion-legacy.log" 2>&1; then
  fail "a motion-only session was planned on the legacy path" \
    "$ROOT/motion-legacy.log"
fi
grep -q 'needs --read-path bounded' "$ROOT/motion-legacy.log"
setsid tools/e1_prod.sh "${COMMON[@]}" --yes --read-path bounded \
  --feedback motion --aux-interval-ms 30 --attempt 3 --conditions idle \
  --profiles position --rates 100 </dev/null >"$ROOT/motion-prod.log" 2>&1 ||
  fail "the motion-only production session failed" "$ROOT/motion-prod.log"
MBASE="$OUT/rate-e1-prod-position-100hz-idle-bounded-motion-aux30ms-$DATE-3.csv"
grep -q '^feedback = "motion"' "$MBASE.summary.toml"
grep -q '^full_read_every = 3' "$MBASE.summary.toml"
grep -q '^timeouts = 0' "$MBASE.summary.toml"
grep -q 'read path: bounded .*, feedback motion (full read every 30 ms)' \
  "$MBASE.run.txt"
grep -q -- '--feedback motion --aux-interval-ms 30' "$MBASE.run.txt"
grep -q '10-byte reads, .* cycles: host_transaction' "$MBASE.run.txt"
grep -q '13-byte reads, .* cycles: host_transaction' "$MBASE.run.txt"

echo "e1 session scripts OK"
