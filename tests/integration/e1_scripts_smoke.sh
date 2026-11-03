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
done

echo "e1 session scripts OK"
