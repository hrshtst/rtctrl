#!/usr/bin/env bash
# E1 calibration runs (docs/hardware/control-rate.md), torque off.
#
# Run it from the rtctrl repo root, after a Release build, with every servo
# torque-off and the arm at rest and supported. x7_bus_timing refuses to
# measure otherwise, and it never writes Torque Enable or Drive Mode.
#
# Each run adds --confirm-writes --drain, idle machine, default scheduling.
# Default runs: current and position at 100, 200, 500 and 1000 Hz, cbp at
# 100, 200 and 500 Hz. Calibration traffic is not production traffic, so its
# overruns are expected and never judged: there is no --check and no climb.
# After each run timing_summary.py prints the completion-offset distribution
# and the envelope candidate.
#
# Each write is the unchanged production write; the probe acknowledges it
# by reading the last servo's Model Number, so a valid response bounds the
# write's completion whatever the servo does with the goal.
#
# Stops: a probe that exits nonzero (a refusal, torque found on, incomplete
# outputs) or an analysis error stops the session, per the stop rule. A run
# with invalid acknowledgements (a failed read, or a wrong model number) is
# shown, and the script asks whether to go on.
#
# Each run leaves three gitignored files in the output directory:
#   <log>.csv               per-cycle log (probe)
#   <log>.csv.summary.toml  run summary (probe)
#   <log>.csv.run.txt       command, host state, probe output, summary
# with <log> = rate-e1-cal-<profile>-<rate>hz-<date>-<attempt>. A run whose
# .csv or .run.txt already exists was attempted: it is skipped, never
# retried. Use --attempt N for a new series.
set -u

PROBE=./build/apps/x7_bus_timing  # --probe overrides it (the tests do)
SUMMARY_TOOL=tools/timing_summary.py

runs=()
duration=60
port=
date_tag=$(date +%Y%m%d)
attempt=1
outdir=.
max_runnable=3
dry_run=0
assume_yes=0

usage() {
  cat <<EOF
usage: $0 [options]
  --runs PROFILE:RATES  one profile and its rates, e.g. cbp:100,200,500;
                        repeat it to choose the runs (default
                        current:100,200,500,1000 position:100,200,500,1000
                        cbp:100,200,500)
  --duration S          seconds per run (default $duration)
  --probe PATH          x7_bus_timing binary (default $PROBE)
  --port P              serial port (default: the probe's configuration)
  --date YYYYMMDD       date in the log names (default today, $date_tag)
  --attempt N           attempt number in the log names (default $attempt)
  --outdir DIR          where the logs go (default the repo root)
  --max-runnable N      quiet-machine threshold, mean runnable tasks
                        including this script (default $max_runnable)
  --dry-run             check the prerequisites and print the plan only
  --yes                 confirm the session in advance (no prompt)
EOF
}

die() { echo "e1_cal: $*" >&2; exit 2; }

while [ $# -gt 0 ]; do
  case "$1" in
    --runs|--duration|--probe|--port|--date|--attempt|--outdir|--max-runnable)
      [ $# -ge 2 ] || die "$1 needs a value"
      case "$1" in
        --runs) runs+=("$2") ;;
        --duration) duration=$2 ;;
        --probe) PROBE=$2 ;;
        --port) port=$2 ;;
        --date) date_tag=$2 ;;
        --attempt) attempt=$2 ;;
        --outdir) outdir=$2 ;;
        --max-runnable) max_runnable=$2 ;;
      esac
      shift 2 ;;
    --dry-run) dry_run=1; shift ;;
    --yes) assume_yes=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
done
[ ${#runs[@]} -gt 0 ] ||
  runs=("current:100,200,500,1000" "position:100,200,500,1000" "cbp:100,200,500")

is_uint() { [[ $1 =~ ^[0-9]+$ ]]; }
plan=()  # "profile rate"
for spec in "${runs[@]}"; do
  profile=${spec%%:*}
  [ "$profile" != "$spec" ] || die "--runs '$spec': expected PROFILE:RATES"
  case "$profile" in current|position|cbp) ;; *) die "unknown profile '$profile'" ;; esac
  IFS=, read -r -a rs <<<"${spec#*:}"
  for r in "${rs[@]}"; do
    is_uint "$r" && [ "$r" -gt 0 ] || die "--runs '$spec': rates must be positive integers"
    plan+=("$profile $r")
  done
done
[[ $duration =~ ^[0-9]+$ ]] && [ "$duration" -gt 0 ] || die "--duration must be a positive integer"
is_uint "$attempt" && [ "$attempt" -gt 0 ] || die "--attempt must be a positive integer"
is_uint "$max_runnable" || die "--max-runnable must be an integer"
[[ $date_tag =~ ^[0-9]{8}$ ]] || die "--date must be YYYYMMDD"

problems=()
[ -x "$PROBE" ] && [ -f "$SUMMARY_TOOL" ] ||
  problems+=("run from the repo root after building ($PROBE, $SUMMARY_TOOL)")
[ -d "$outdir" ] && [ -w "$outdir" ] || problems+=("--outdir $outdir is not a writable directory")
if [ -f "$SUMMARY_TOOL" ] &&
   ! uv run --project tools "$SUMMARY_TOOL" --help >/dev/null 2>&1; then
  problems+=("uv run --project tools $SUMMARY_TOOL --help failed")
fi
if [ ${#problems[@]} -gt 0 ]; then
  for p in "${problems[@]}"; do echo "e1_cal: $p" >&2; done
  [ "$dry_run" -eq 1 ] || exit 2
fi

log_base() { echo "rate-e1-cal-$1-${2}hz-$date_tag-$attempt"; }

probe_cmd() {  # profile rate -> the probe command in cmd[]
  cmd=("$PROBE")
  [ -n "$port" ] && cmd+=(--port "$port")
  cmd+=(--rate-hz "$2" --profile "$1" --duration "$duration"
        --confirm-writes --drain --log "$outdir/$(log_base "$1" "$2").csv")
}

total=${#plan[@]}

if [ "$dry_run" -eq 1 ]; then
  echo "plan: $total runs, in this order (date $date_tag, attempt $attempt):"
  for entry in "${plan[@]}"; do
    read -r profile rate <<<"$entry"
    probe_cmd "$profile" "$rate"
    printf '  '; printf '%q ' "${cmd[@]}"; echo
  done
  echo "about $(( (total * (duration + 10) + 59) / 60 )) minutes"
  exit 0
fi

# Nothing touches the bus before the operator confirms the session.
if [ "$assume_yes" -ne 1 ]; then
  if ! { : </dev/tty; } 2>/dev/null; then
    die "no terminal to confirm the session on (pass --yes to confirm in advance)"
  fi
  target=${port:-"the probe's configured port"}
  echo "This session talks to the servos on $target. Check that:"
  echo "  - every servo is torque-off and the arm is at rest and supported;"
  echo "  - the power cutoff is within reach;"
  echo "  - nothing else uses the port or loads the machine."
  printf 'Type yes to start: '
  read -r answer </dev/tty || answer=
  [ "$answer" = yes ] || die "not confirmed; nothing was sent"
fi

current=()
results=()
add_result() { results+=("$(printf '%-9s %5s Hz  %s' "$1" "$2" "$3")"); }
print_table() {
  echo
  echo "E1 calibration results (date $date_tag, attempt $attempt):"
  for line in "${results[@]}"; do echo "  $line"; done
}
on_interrupt() {
  echo
  echo "e1_cal: interrupted; the run in progress counts as attempted"
  [ ${#current[@]} -gt 0 ] && add_result "${current[@]}" "INTERRUPTED (counts as attempted)"
  print_table
  exit 130
}
trap on_interrupt INT TERM

runnable_mean() {  # mean runnable tasks over 2 s, this script included
  local sum=0 r
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    read -r _ _ _ r _ </proc/loadavg
    sum=$((sum + ${r%%/*}))
    sleep 0.2
  done
  echo $(( (sum + 5) / 10 ))
}
wait_quiet() {
  local n polls=0
  while :; do
    n=$(runnable_mean)
    if [ "$n" -le "$max_runnable" ]; then quiet=$n; return; fi
    if [ $((polls % 15)) -eq 0 ]; then
      echo "waiting for a quiet machine: $n runnable tasks (at most $max_runnable);$(uptime | sed 's/.*load/ load/')"
      ps -eo pcpu,comm --sort=-pcpu | sed -n '2,4p' | sed 's/^/    /'
    fi
    polls=$((polls + 1))
  done
}

governors=$(sort /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | uniq -c | xargs)
commit="$(git rev-parse --short HEAD 2>/dev/null) ($(git status --porcelain --untracked-files=no 2>/dev/null | wc -l) tracked files modified)"
k=0

for entry in "${plan[@]}"; do
  current=()
  read -r profile rate <<<"$entry"
  base=$(log_base "$profile" "$rate")
  csv="$outdir/$base.csv"
  note="$csv.run.txt"
  k=$((k + 1))
  echo
  echo "=== [$k/$total] $profile, $rate Hz: $base"
  if [ -e "$csv" ] || [ -e "$note" ]; then
    echo "already attempted, skipped"
    add_result "$profile" "$rate" "skipped (attempted earlier)"
    continue
  fi
  current=("$profile" "$rate")
  wait_quiet
  probe_cmd "$profile" "$rate"
  {
    echo "rtctrl E1 calibration run"
    echo "started: $(date -Is)"
    echo "commit: $commit"
    echo "probe binary: $(stat -c %y "$PROBE")"
    echo "condition: idle, default scheduling"
    echo "cpu governors: $governors"
    echo "runnable before start: $quiet (this script included)"
    echo "uptime:$(uptime)"
    printf 'command:'; printf ' %q' "${cmd[@]}"; echo
    echo
  } >"$note"
  "${cmd[@]}" 2>&1 | tee -a "$note"
  probe_status=${PIPESTATUS[0]}
  if [ "$probe_status" -ne 0 ]; then
    { echo; echo "verdict: probe exit $probe_status (session stopped)"; } >>"$note"
    echo "e1_cal: the probe exited $probe_status: session stopped (stop rule). See $note"
    add_result "$profile" "$rate" "STOPPED the session (see the .run.txt)"
    print_table
    exit 1
  fi
  { echo; echo "--- timing_summary.py --rate-hz $rate ---"; } >>"$note"
  uv run --project tools "$SUMMARY_TOOL" "$csv" --rate-hz "$rate" 2>&1 | tee -a "$note"
  check_status=${PIPESTATUS[0]}
  if [ "$check_status" -ne 0 ]; then
    { echo; echo "verdict: analysis error (exit $check_status, session stopped)"; } >>"$note"
    echo "e1_cal: timing_summary.py failed (exit $check_status): session stopped. See $note"
    add_result "$profile" "$rate" "STOPPED the session (analysis error)"
    print_table
    exit 1
  fi
  invalid=$(sed -n 's/^ack_invalid = //p' "$csv.summary.toml" | head -1)
  candidate=$(sed -n 's/.*envelope candidate (1.25 x max) *//p' "$note" | tail -1)
  if [ "${invalid:-0}" != 0 ]; then
    echo "verdict: done, $invalid invalid acknowledgements" >>"$note"
    add_result "$profile" "$rate" "done, $invalid INVALID ACKNOWLEDGEMENTS; envelope candidate ${candidate:-?} ms"
    if ! { : </dev/tty; } 2>/dev/null; then
      echo "e1_cal: invalid acknowledgements and no terminal to ask: session stopped"
      print_table
      exit 1
    fi
    echo
    echo ">>> $profile at $rate Hz: $invalid acknowledging reads failed or returned the wrong model number."
    while :; do
      printf '>>> c = continue with the next run; q = quit: '
      read -r answer </dev/tty || answer=q
      case "$answer" in
        c|C) echo "operator: continued" >>"$note"; break ;;
        q|Q) echo "operator: quit the session" >>"$note"; print_table; exit 1 ;;
      esac
    done
  else
    echo "verdict: done" >>"$note"
    add_result "$profile" "$rate" "done; envelope candidate ${candidate:-?} ms"
  fi
done
print_table
