#!/usr/bin/env bash
# x7_bus_timing --timing adapter over dxl_emu's opt-in adapter cadence
# (--cadence; emu::CompletionCadence), whose record says which completion
# carried each reply: deterministic outcomes, asserted, where the default
# emulator's depend on whether this host's pty happens to lock
# (bus_timing_adapter_smoke.sh). First the emulator's refusals of invalid
# cadence options, before it serves (unrepresentable numbers among them);
# then a client that stops reading: the full pty must fail the run
# explicitly, with a truthful record (cadence_backpressure.py); then one
# emulator per scenario:
# - an independent cadence (-57.5 ppm): one true lock, held to the end, and
#   no coupling detected (full-window reads on the lattice at the base lead);
# - delayed and missed completions (from 2.5 s on, after the lock): every
#   locked read on one observed late, the true lock held;
# - a phase step 200 us earlier: early replies, the loss, the grid, and a
#   true lock on the moved completions (whichever of supervision's two rules
#   ends the lock);
# - a drift beyond the fit's range: every fit refused, the grid throughout;
# - request-tied replies (the negative controls): 500 us after the request,
#   false locks lost after 64 cycles with their replies early; 790 us, a
#   false lock near the prediction held at least 256 cycles and declared
#   request-tied.
# The probe runs with --fine-timer-slack: it wakes within a few us, as the
# deterministic cases model it (with the default 50 us slack it wakes about
# 51 us late, which moves request-tied replies against its predictions).
# Each run's log is checked against its summary (check_adapter_log.py) and
# against the emulator's record (check_cadence_log.py). The margins are the
# scenarios' (300 us steps, 120 us delays, 1 ms misses against a 50 us
# gate), not the pty's; the pty's delivery delays are printed with each run.
set -eu

EMU="$1"
PROBE="$2"
INSPECT="$3"
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "bus_timing_cadence_smoke" "${4:-}" || exit 2
ROOT=$RTCTRL_SCRATCH
LINK="$ROOT/tty"
CHECK="$(dirname "$0")/check_adapter_log.py"
CADENCE_CHECK="$(dirname "$0")/check_cadence_log.py"

fail() { echo "$1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }

# The emulator refuses invalid cadence options before it serves: status 2,
# the reason on stderr, no link. One that serves instead is stopped after 5 s
# (status 124) rather than waited on.
refuse_emu() {  # refuse_emu WHY ARGS...
  local why="$1"
  shift
  local status=0
  timeout 5 "$EMU" --link "$ROOT/refused-tty" --firmware 45 "$@" >"$ROOT/refused-emu.log" 2>&1 || status=$?
  [ "$status" = 2 ] || fail "dxl_emu did not refuse (status $status): $*" "$ROOT/refused-emu.log"
  grep -qF -- "$why" "$ROOT/refused-emu.log" || fail "dxl_emu refused for another reason: $*" "$ROOT/refused-emu.log"
  [ ! -e "$ROOT/refused-tty" ] || fail "a refused emulator served: $*"
}
mkdir -p "$ROOT"
refuse_emu "independent or request-tied" --cadence bogus
refuse_emu "need --cadence" --cadence-ppm 10
refuse_emu "needs a value" --cadence independent --cadence-ppm
refuse_emu "not a number" --cadence independent --cadence-ppm abc
refuse_emu "unknown" --cadence independent --cadence-bogus 1
refuse_emu "[-1000, 1000] ppm" --cadence independent --cadence-ppm 2000
refuse_emu "N >= 2" --cadence independent --cadence-miss-every 1
refuse_emu "needs delayed completions" --cadence independent --cadence-delay-us 10
refuse_emu "below half a period" --cadence independent --cadence-delay-every 3 --cadence-delay-us 600
refuse_emu "below half a period" --cadence independent --cadence-step 1:600
refuse_emu "two periods apart" --cadence independent --cadence-step 1:100 --cadence-step 1.001:100
refuse_emu "T:S" --cadence independent --cadence-step 1
refuse_emu "independent cadence only" --cadence request-tied --cadence-tie-us 150 --cadence-ppm 5
refuse_emu "request-tied cadence only" --cadence independent --cadence-tie-us 150
# Numbers checked before they are scaled (the review of e40875f): an
# unrepresentable shift, an infinite time, a count beyond 64 bits, a time
# finer than a nanosecond, and a delay the request-tied control would ignore.
refuse_emu "out of range" --cadence independent --cadence-step 1:1e300
refuse_emu "out of range" --cadence independent --cadence-miss-every 2 --cadence-impair-from-s inf
refuse_emu "out of range" --cadence independent --cadence-miss-every 18446744073709551616
refuse_emu "finer than one nanosecond" --cadence independent --cadence-phase-us 0.0004
refuse_emu "independent cadence only" --cadence request-tied --cadence-tie-us 790 --cadence-delay-us 120
: >"$ROOT/exists.csv"
refuse_emu "must not exist" --cadence independent --cadence-log "$ROOT/exists.csv"

# A client that never reads its replies: the run fails at the first reply
# the full pty does not accept in full, and says so.
mkdir -p "$ROOT/backpressure"
python3 "$(dirname "$0")/cadence_backpressure.py" "$EMU" "$ROOT/backpressure" ||
  fail "backpressure: not an explicit failure with a truthful record" "$ROOT/backpressure/backpressure.emu.log"

GRID_HEADER="cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,first_reply_ns,last_reply_ns,read_end_ns,limit_start_ns,bus_write_start_ns,write_return_ns,drain_return_ns,ack_return_ns,completion_upper_ns,cycle_end_ns,read_ok,write_ok,ack_ok,skipped_periods,read_len"
MOTION=(--read-instruction fast --feedback motion --aux-interval-ms 100)
EMU_PID=
trap '[ -n "$EMU_PID" ] && kill "$EMU_PID" 2>/dev/null || true' EXIT

# run NAME DURATION_S "EMULATOR CADENCE OPTIONS" PROBE OPTIONS...: one
# emulator (its cadence starts with it) and one adapter run, both checked
# for consistency; the outcome is the caller's to check.
run() {
  local name="$1" duration="$2" emu_opts="$3"
  shift 3
  local -a cadence
  read -r -d '' -a cadence <<<"$emu_opts" || true  # every line
  "$EMU" --link "$LINK" --firmware 45 "${cadence[@]}" --cadence-log "$ROOT/$name.emu.csv" \
    >"$ROOT/$name.emu.log" 2>&1 &
  EMU_PID=$!
  for _ in $(seq 1 50); do
    [ -e "$LINK" ] && break
    sleep 0.1
  done
  [ -e "$LINK" ] || fail "emulator link never appeared ($name)" "$ROOT/$name.emu.log"
  for id in 2 3 4 5 6 7 8 9; do  # the bounded read's bus profile
    "$INSPECT" --port "$LINK" write "$id" return_delay_time 0 >/dev/null
  done
  "$PROBE" --port "$LINK" --profile current --log "$ROOT/$name.csv" --rate-hz 1000 --duration "$duration" \
    --read-path bounded --timing adapter --fine-timer-slack "${MOTION[@]}" "$@" >"$ROOT/$name.log" 2>&1 ||
    fail "$name: the probe failed" "$ROOT/$name.log"
  for id in 2 3 4 5 6 7 8 9; do
    test "$("$INSPECT" --port "$LINK" read "$id" torque_enable)" = "0" || fail "$name: torque on"
  done
  kill "$EMU_PID"
  local status=0
  wait "$EMU_PID" || status=$?
  EMU_PID=
  [ "$status" = 0 ] || fail "$name: the emulator exited $status" "$ROOT/$name.emu.log"
  grep -q '^dxl_emu: cadence: ' "$ROOT/$name.emu.log" || fail "$name: no cadence summary" "$ROOT/$name.emu.log"
  python3 "$CHECK" "$ROOT/$name.csv" "$GRID_HEADER" "$duration" 1 || fail "$name: log and summary disagree"
}
cadence_check() {  # cadence_check NAME MODE EXPECTATIONS...
  local name="$1" mode="$2"
  shift 2
  python3 "$CADENCE_CHECK" "$ROOT/$name.csv" "$ROOT/$name.emu.csv" --mode "$mode" "$@" ||
    fail "$name: not the expected outcome" "$ROOT/$name.log"
}
# The windows and durations of the deterministic cases that pin these
# outcomes at every phase of the cadence to the probe's grid
# (tests/unit/bus_timing_adapter_test.cpp, "[cadence]"): a run's phase is
# whatever its emulator's start gives it.
# A reply is ready 500 us after its request: on this host the pty delivers a
# reply sent sooner after its request slowly (p50 25 to 35 us, against 5 to
# 9 us), which at some phases left the fit's share under its floor.
ALT=(--lead-alternation-us 20)
INDEPENDENT="--cadence independent --cadence-ppm -57.5 --cadence-ready-us 500"

# An independent completion cadence, 57.5 ppm fast on the host clock.
run independent 4 "$INDEPENDENT --cadence-phase-us 900" --acquisition-s 2 --settling-s 0.5 "${ALT[@]}"
grep -q '^cadence: independent, period 999942.500 ns (-57.500 ppm), first completion 900.0 us' \
  "$ROOT/independent.emu.log" || fail "the cadence is not as asked" "$ROOT/independent.emu.log"
cadence_check independent independent --locks 1 --losses 0 --held-at-end --true-lock \
  --first-declaration no_coupling_detected --coupling-end no_coupling_detected --observed early=0

# Delayed (every 10th, 120 us) and missed (every 17th) completions, from
# 2.5 s on: the acquisition fits the plain cadence (a tenth of its stamps
# delayed would leave the fit's share near its 0.75 floor once the pty's own
# spread is added), the lock then meets them.
run delayed-missed 4.5 "$INDEPENDENT --cadence-delay-every 10 --cadence-delay-us 120 --cadence-miss-every 17
  --cadence-impair-from-s 2.5" --acquisition-s 2 --settling-s 0.5 "${ALT[@]}"
cadence_check delayed-missed independent --locks 1 --losses 0 --held-at-end --true-lock --late-on-cadence \
  --min-on-delayed 100 --min-on-missed 50 --min-observed late=150

# A phase step 200 us earlier at 3 s (to just before the request's ready
# time a step would pass the moved completion, its replies then a period
# late): the replies come early until
# supervision ends the lock: 64 cycles on (no gated stamp), or sooner when the
# pty's late stamps before the step bring the last 256 cycles past a quarter
# outside the gate (checked against the rows either way); the grid, then a
# new true lock.
run step 6 "$INDEPENDENT --cadence-step 3:-200" --acquisition-s 2 --settling-s 0.5 "${ALT[@]}"
cadence_check step independent --locks 2 --losses 1 --loss-cause no_gated_stamp,outside_gate --held-at-end \
  --true-lock --min-observed early=48

# A drift beyond the fit's range: every fit refused, the grid throughout.
run drift-beyond 3 "--cadence independent --cadence-ppm 350 --cadence-ready-us 500" \
  --acquisition-s 1 --settling-s 0.5
cadence_check drift-beyond independent --no-lock

# The negative controls: request-tied replies, no lattice. 500 us: each
# false lock's replies about 280 us early, lost after exactly 64 cycles.
# (Replies 150 or 300 us after their requests came with a wider delivery
# spread on this host, p50 28 to 35 us, and most fits refused them; the
# deterministic cases pin 150 us.) A fit can still refuse these, so the run
# allows five attempts and asserts every lock that forms.
run tied-500 6 "--cadence request-tied --cadence-tie-us 500" --acquisition-s 1 --settling-s 0.5 "${ALT[@]}"
cadence_check tied-500 request-tied --min-locks 1 --loss-cause no_gated_stamp --first-declaration none
python3 - "$ROOT/tied-500.csv" <<'EOF_PY' || fail "tied-500: a false lock not ended by its early replies"
# Every false lock lost (no gated stamp: check_cadence_log.py checks each
# loss against its rows) once its replies came early: at least 64 cycles
# (65 or more when the pty delayed one of its replies into the gate), its
# locked replies early but for at most 1% (such delays).
import csv
import sys
import tomllib
s = tomllib.load(open(sys.argv[1] + ".summary.toml", "rb"))
ended = [lk for lk in s["adapter"]["lock"] if lk["end"] != "run_end"]
assert ended and all(lk["referenced_cycles"] >= 64 for lk in ended), ended
rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(open(sys.argv[1]))]
lattice = [r for r in rows if r["schedule"] == 1]
early = sum(1 for r in lattice if r["observation"] == 5)
assert early >= 64 * len(ended) and (len(lattice) - early) * 100 <= len(lattice), (early, len(lattice), len(ended))
EOF_PY
# 790 us: a false lock near the prediction, held at least 256 cycles,
# declared request-tied.
run tied-790 4 "--cadence request-tied --cadence-tie-us 790" --acquisition-s 1 --settling-s 0.5 "${ALT[@]}"
cadence_check tied-790 request-tied --min-locks 1 --first-declaration request_tied_detected \
  --min-referenced 256

scratch_release
echo "bus timing cadence smoke OK"
