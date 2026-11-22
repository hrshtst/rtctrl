#!/usr/bin/env bash
# The torque-off adapter-timing diagnostic's two arms over dxl_emu's opt-in
# adapter cadence, unprivileged (the diagnostic plan, revision 2):
# - tools/trace_session.sh's plan (--dry-run) for both arms at the sitting's
#   condition (rt-loaded-excl): the grid arm `fast-motion`, the adapter arm
#   `fast-motion-adapter` with its six timing flags; and its refusals of the
#   adapter variant off 1000 Hz or with too short a run;
# - each arm's planned command (the idle condition's, since a real-time
#   priority needs a limit this test cannot have; --fine-timer-slack stands
#   for the zero slack the sitting's SCHED_FIFO worker has), 20 s over the
#   rehearsal's cadence: independent, -57.5 ppm, the first completion 900 us
#   after the start, replies ready 500 us after their requests (the
#   settings the cadence smoke's true locks were shown under);
# - tools/adapter_diagnostic.py: both runs valid with the plan's settings;
#   the adapter run analysed with the emulator's record as its carriers (the
#   PTY stand-in for the trace's byte association), the grid run against it.
# Asserted: validity, the first fit applied with a lock held through the
# locked window (A1), and the logged plans aligned with the emulator's
# completions (A3 on its record). Every other prediction is printed, not
# asserted: their bands are the hardware's, and a PTY is not the adapter.
set -eu

EMU="$1"
PROBE="$2"
INSPECT="$3"
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "bus_timing_diagnostic_smoke" "${4:-}" || exit 2
ROOT=$RTCTRL_SCRATCH
LINK="$ROOT/tty"
TOOL=tools/adapter_diagnostic.py
CADENCE=(--cadence independent --cadence-ppm -57.5 --cadence-phase-us 900 --cadence-ready-us 500)

fail() { echo "$1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }
: >"$ROOT/manifest.md"
plan() {  # plan RUN-SPEC [extra session options...] -> the dry run's output in $ROOT/plan.txt
  local spec="$1"
  shift
  TRACE_SESSION_RTPRIO_LIMIT=99 tools/trace_session.sh --dry-run --modes untraced --runs "$spec" --duration 20 \
    --attempt 1 --date 20000101 --outdir "$ROOT" --manifest "$ROOT/manifest.md" --port "$LINK" --no-usbmon "$@" \
    </dev/null >"$ROOT/plan.txt" 2>&1
}

# The session's plan for the sitting's condition, both arms.
ADAPTER_FLAGS="--read-path bounded --read-instruction fast --feedback motion --timing adapter --acquisition-s 2 --settling-s 1 --lead-us 780 --wake-to-read-us 10 --lead-alternation-us 20"
plan current:1000:rt-loaded-excl:fast-motion-adapter || fail "the adapter arm's dry run failed" "$ROOT/plan.txt"
grep -q -- "--rt-priority 80 --rt-cpu 3 $ADAPTER_FLAGS \$" "$ROOT/plan.txt" ||
  fail "the adapter arm's planned command" "$ROOT/plan.txt"
grep -q -- "-bounded-fast-motion-adapter-untraced-20000101-1.csv" "$ROOT/plan.txt" || fail "the adapter arm's log name" "$ROOT/plan.txt"
plan current:1000:rt-loaded-excl:fast-motion || fail "the grid arm's dry run failed" "$ROOT/plan.txt"
grep -q -- "--rt-priority 80 --rt-cpu 3 --read-path bounded --read-instruction fast --feedback motion \$" "$ROOT/plan.txt" ||
  fail "the grid arm's planned command" "$ROOT/plan.txt"
if plan current:500:rt-loaded-excl:fast-motion-adapter; then fail "the adapter variant planned at 500 Hz" "$ROOT/plan.txt"; fi
grep -q "the adapter variant runs at 1000 Hz only" "$ROOT/plan.txt" || fail "no refusal off 1000 Hz" "$ROOT/plan.txt"
if TRACE_SESSION_RTPRIO_LIMIT=99 tools/trace_session.sh --dry-run --modes untraced --runs current:1000:idle:fast-motion-adapter \
    --duration 3 --outdir "$ROOT" --manifest "$ROOT/manifest.md" --port "$LINK" --no-usbmon </dev/null >"$ROOT/plan.txt" 2>&1; then
  fail "the adapter variant planned for 3 s" "$ROOT/plan.txt"
fi
grep -q "need at least 4 s" "$ROOT/plan.txt" || fail "no refusal of a 3 s adapter run" "$ROOT/plan.txt"

EMU_PID=
trap '[ -n "$EMU_PID" ] && kill "$EMU_PID" 2>/dev/null || true' EXIT
# arm VARIANT: one emulator with the rehearsal's cadence and its record, the session's planned idle command.
arm() {
  local variant="$1" cmd
  rm -f "$LINK"
  "$EMU" --link "$LINK" --firmware 45 "${CADENCE[@]}" --cadence-log "$ROOT/$variant.emu.csv" \
    >"$ROOT/$variant.emu.log" 2>&1 &
  EMU_PID=$!
  for _ in $(seq 1 50); do
    [ -e "$LINK" ] && break
    sleep 0.1
  done
  [ -e "$LINK" ] || fail "emulator link never appeared ($variant)" "$ROOT/$variant.emu.log"
  for id in 2 3 4 5 6 7 8 9; do
    "$INSPECT" --port "$LINK" write "$id" return_delay_time 0 >/dev/null
  done
  plan "current:1000:idle:$variant" --probe "$PROBE" || fail "$variant: the dry run failed" "$ROOT/plan.txt"
  cmd=$(grep -o -- "$PROBE --port .*" "$ROOT/plan.txt") || fail "$variant: no planned command" "$ROOT/plan.txt"
  read -r -a cmd <<<"$cmd"
  "${cmd[@]}" --fine-timer-slack >"$ROOT/$variant.probe.log" 2>&1 || fail "$variant: the probe failed" "$ROOT/$variant.probe.log"
  kill "$EMU_PID"
  local status=0
  wait "$EMU_PID" || status=$?
  EMU_PID=
  [ "$status" = 0 ] || fail "$variant: the emulator exited $status" "$ROOT/$variant.emu.log"
  LOG=$(grep -o -- "--log [^ ]*" <<<"${cmd[*]}" | cut -d' ' -f2)
}

arm fast-motion-adapter
ALOG=$LOG
arm fast-motion
GLOG=$LOG
uv run --project tools python "$TOOL" validate --plan adapter "$ALOG" >"$ROOT/a.validate.txt" 2>&1 ||
  fail "the adapter run is not valid" "$ROOT/a.validate.txt"
uv run --project tools python "$TOOL" validate --plan grid "$GLOG" >"$ROOT/g.validate.txt" 2>&1 ||
  fail "the grid run is not valid" "$ROOT/g.validate.txt"
uv run --project tools python "$TOOL" analyze "$ALOG" --emulator-record "$ROOT/fast-motion-adapter.emu.csv" \
  --json "$ROOT/a.json" >"$ROOT/a.analysis.txt" 2>&1 || fail "the adapter run's analysis failed" "$ROOT/a.analysis.txt"
uv run --project tools python "$TOOL" analyze "$GLOG" --compare-adapter "$ALOG" >"$ROOT/g.analysis.txt" 2>&1 ||
  fail "the grid run's analysis failed" "$ROOT/g.analysis.txt"
cat "$ROOT/a.analysis.txt" "$ROOT/g.analysis.txt"
python3 - "$ROOT/a.json" <<'EOF' || fail "the adapter run's lock or its alignment" "$ROOT/a.analysis.txt"
import json, sys
rep = json.load(open(sys.argv[1]))
p = {x["prediction"]: x for x in rep["predictions"]}
assert p["A1"]["verdict"] == "held", p["A1"]
assert p["A3"]["verdict"] == "held", p["A3"]
EOF
echo "bus timing diagnostic smoke OK"
