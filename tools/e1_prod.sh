#!/usr/bin/env bash
# E1 production-traffic matrix (docs/hardware/control-rate.md), torque off.
#
# Run it from the rtctrl repo root, after a Release build, with every servo
# torque-off and the arm at rest and supported. x7_bus_timing refuses to
# measure otherwise, and it never writes Torque Enable or Drive Mode.
#
# For each condition, then each profile, it climbs the rates with one run
# each and evaluates the criteria with timing_summary.py --check:
#   conditions  idle, loaded, rt-idle, rt-loaded
#   profiles    current, position, cbp
#   rates       100, 200, 500, 1000 Hz
# A failed criterion is data, not a stop: the script shows the verdicts and
# asks whether that rate clearly failed. "Yes" ends the climb for that
# profile under that condition. A probe that exits nonzero (a refusal,
# torque found on, incomplete outputs) or an analysis error stops the whole
# session, per the stop rule.
#
# Before each run it waits for a quiet machine (the runnable-task count in
# /proc/loadavg). A loaded run starts its own stress-ng window, which ends
# 15 s after the run's duration.
#
# Each run leaves three gitignored files in the output directory:
#   <log>.csv               per-cycle log (probe)
#   <log>.csv.summary.toml  run summary (probe)
#   <log>.csv.run.txt       command, host state, probe output, verdicts
# with <log> = rate-e1-prod-<profile>-<rate>hz-<condition>[-bounded]-<date>-
# <attempt> (-bounded with --read-path bounded).
# A run whose .csv or .run.txt already exists was attempted: it is skipped,
# never retried, and its recorded climb decision is kept. Use --attempt N
# for a new series.
set -u

PROBE=./build/apps/x7_bus_timing  # --probe overrides it (the tests do)
SUMMARY_TOOL=tools/timing_summary.py

conditions=idle,loaded,rt-idle,rt-loaded
profiles=current,position,cbp
rates=100,200,500,1000
duration=60
rt_priority=80
rt_cpu=3
port=
date_tag=$(date +%Y%m%d)
attempt=1
outdir=.
max_runnable=3
stress_dir="$HOME/.cache/rtctrl-stress"
read_path=legacy
auto_stop=0
dry_run=0
assume_yes=0

usage() {
  cat <<EOF
usage: $0 [options]
  --conditions LIST   comma-separated, from idle,loaded,rt-idle,rt-loaded
                      (default $conditions)
  --profiles LIST     from current,position,cbp (default $profiles)
  --rates LIST        Hz, ascending (default $rates)
  --duration S        seconds per run (default $duration)
  --rt-priority N     SCHED_FIFO priority 1 to 99 for rt-* (default $rt_priority)
  --rt-cpu C          CPU for rt-* (default $rt_cpu)
  --probe PATH        x7_bus_timing binary (default $PROBE)
  --port P            serial port (default: the probe's configuration)
  --date YYYYMMDD     date in the log names (default today, $date_tag)
  --attempt N         attempt number in the log names (default $attempt)
  --outdir DIR        where the logs go (default the repo root)
  --max-runnable N    quiet-machine threshold, mean runnable tasks
                      including this script (default $max_runnable)
  --read-path P       legacy (default) or bounded: the probe's grouped read
                      (bounded: the probe's conservative 35 ms budget and
                      equal guard; log names gain "-bounded")
  --stress-dir DIR    stress-ng --hdd directory, on a disk, not tmpfs
                      (default $stress_dir)
  --auto-stop         end a climb at its first failed check without asking
  --dry-run           check the prerequisites and print the plan only
  --yes               confirm the session in advance (no prompt)
EOF
}

die() { echo "e1_prod: $*" >&2; exit 2; }

while [ $# -gt 0 ]; do
  case "$1" in
    --conditions|--profiles|--rates|--duration|--rt-priority|--rt-cpu|\
    --probe|--port|--date|--attempt|--outdir|--max-runnable|--stress-dir|\
    --read-path)
      [ $# -ge 2 ] || die "$1 needs a value"
      case "$1" in
        --conditions) conditions=$2 ;;
        --profiles) profiles=$2 ;;
        --rates) rates=$2 ;;
        --duration) duration=$2 ;;
        --rt-priority) rt_priority=$2 ;;
        --rt-cpu) rt_cpu=$2 ;;
        --probe) PROBE=$2 ;;
        --port) port=$2 ;;
        --date) date_tag=$2 ;;
        --attempt) attempt=$2 ;;
        --outdir) outdir=$2 ;;
        --max-runnable) max_runnable=$2 ;;
        --stress-dir) stress_dir=$2 ;;
        --read-path) read_path=$2 ;;
      esac
      shift 2 ;;
    --auto-stop) auto_stop=1; shift ;;
    --dry-run) dry_run=1; shift ;;
    --yes) assume_yes=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
done

IFS=, read -r -a cond_list <<<"$conditions"
IFS=, read -r -a profile_list <<<"$profiles"
IFS=, read -r -a rate_list <<<"$rates"

is_uint() { [[ $1 =~ ^[0-9]+$ ]]; }
for c in "${cond_list[@]}"; do
  case "$c" in idle|loaded|rt-idle|rt-loaded) ;; *) die "unknown condition '$c'" ;; esac
done
for p in "${profile_list[@]}"; do
  case "$p" in current|position|cbp) ;; *) die "unknown profile '$p'" ;; esac
done
prev=0
for r in "${rate_list[@]}"; do
  is_uint "$r" && [ "$r" -gt "$prev" ] || die "rates must be ascending integers (got '$rates')"
  prev=$r
done
[[ $duration =~ ^[0-9]+$ ]] && [ "$duration" -gt 0 ] || die "--duration must be a positive integer"
is_uint "$attempt" && [ "$attempt" -gt 0 ] || die "--attempt must be a positive integer"
if ! is_uint "$rt_priority" || ! is_uint "$rt_cpu"; then
  die "--rt-priority and --rt-cpu must be integers"
fi
# The probe reads priority 0 as ordinary scheduling, never SCHED_FIFO.
if [ "$rt_priority" -lt 1 ] || [ "$rt_priority" -gt 99 ]; then
  die "--rt-priority must be 1 to 99 (0 would mean ordinary scheduling)"
fi
is_uint "$max_runnable" || die "--max-runnable must be an integer"
case "$read_path" in
  legacy) path_tag=; read_desc="legacy (the SDK's receive)" ;;
  bounded) path_tag=-bounded
    read_desc="bounded (the probe's defaults: 35 ms budget, equal guard)" ;;
  *) die "--read-path must be legacy or bounded" ;;
esac
[[ $date_tag =~ ^[0-9]{8}$ ]] || die "--date must be YYYYMMDD"
stress_timeout=$((duration + 15))

any_rt=0; any_loaded=0
for c in "${cond_list[@]}"; do
  case "$c" in rt-*) any_rt=1 ;; esac
  case "$c" in *loaded) any_loaded=1 ;; esac
done

# Prerequisites: every problem is listed, then the script refuses to start
# (a dry run lists them and goes on).
problems=()
[ -x "$PROBE" ] && [ -f "$SUMMARY_TOOL" ] ||
  problems+=("run from the repo root after building ($PROBE, $SUMMARY_TOOL)")
[ -d "$outdir" ] && [ -w "$outdir" ] || problems+=("--outdir $outdir is not a writable directory")
if [ "$any_rt" -eq 1 ]; then
  lim=$(ulimit -r)
  if [ "$lim" != unlimited ] && [ "$lim" -lt "$rt_priority" ]; then
    problems+=("real-time priority limit is $lim, below $rt_priority: in the shell that starts this script run
      sudo prlimit --pid \$\$ --rtprio=90:90
    or leave out the rt-* conditions (--conditions idle,loaded)")
  fi
  [ "$rt_cpu" -lt "$(nproc)" ] || problems+=("--rt-cpu $rt_cpu: this machine has $(nproc) CPUs")
fi
if [ "$any_loaded" -eq 1 ]; then
  command -v stress-ng >/dev/null ||
    problems+=("stress-ng is not installed (sudo apt install stress-ng), or leave out the loaded conditions")
  probe_dir=$stress_dir
  while [ ! -d "$probe_dir" ]; do probe_dir=$(dirname "$probe_dir"); done
  [ "$(stat -f -c %T "$probe_dir")" != tmpfs ] ||
    problems+=("--stress-dir $stress_dir is on tmpfs, so --hdd would load memory, not a disk")
fi
if [ -f "$SUMMARY_TOOL" ] &&
   ! uv run --project tools "$SUMMARY_TOOL" --help >/dev/null 2>&1; then
  problems+=("uv run --project tools $SUMMARY_TOOL --help failed")
fi
if [ ${#problems[@]} -gt 0 ]; then
  for p in "${problems[@]}"; do echo "e1_prod: $p" >&2; done
  [ "$dry_run" -eq 1 ] || exit 2
fi

stress_cmd=(stress-ng --cpu "$(nproc)" --hdd 1 --timeout "${stress_timeout}s"
            --temp-path "$stress_dir")

log_base() { echo "rate-e1-prod-$2-${3}hz-$1$path_tag-$date_tag-$attempt"; }

probe_cmd() {  # condition profile rate -> the probe command in cmd[]
  local base; base=$(log_base "$1" "$2" "$3")
  cmd=("$PROBE")
  [ -n "$port" ] && cmd+=(--port "$port")
  cmd+=(--rate-hz "$3" --profile "$2" --duration "$duration"
        --log "$outdir/$base.csv")
  case "$1" in rt-*) cmd+=(--rt-priority "$rt_priority" --rt-cpu "$rt_cpu") ;; esac
  [ "$read_path" = bounded ] && cmd+=(--read-path bounded)
}

total=$(( ${#cond_list[@]} * ${#profile_list[@]} * ${#rate_list[@]} ))

if [ "$dry_run" -eq 1 ]; then
  echo "plan: $total runs at most, in this order (date $date_tag, attempt $attempt):"
  for c in "${cond_list[@]}"; do
    for p in "${profile_list[@]}"; do
      for r in "${rate_list[@]}"; do
        probe_cmd "$c" "$p" "$r"
        case "$c" in *loaded) printf '  [load: %s] ' "${stress_cmd[*]}" ;; *) printf '  ' ;; esac
        printf '%q ' "${cmd[@]}"; echo
      done
    done
  done
  echo "at most about $(( (total * (duration + 20) + 59) / 60 )) minutes if every climb reaches the top rate"
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

stress_pid=
current=()
results=()
add_result() { results+=("$(printf '%-10s %-9s %5s Hz  %s' "$1" "$2" "$3" "$4")"); }
print_table() {
  echo
  echo "E1 production-traffic results (date $date_tag, attempt $attempt, read path $read_path):"
  for line in "${results[@]}"; do echo "  $line"; done
}
stop_stress() {
  if [ -n "$stress_pid" ]; then
    kill -TERM -- "-$stress_pid" 2>/dev/null
    wait "$stress_pid" 2>/dev/null
    stress_pid=
  fi
}
on_interrupt() {
  echo
  echo "e1_prod: interrupted; the run in progress counts as attempted"
  stop_stress
  [ ${#current[@]} -gt 0 ] && add_result "${current[@]}" "INTERRUPTED (counts as attempted; climb ended)"
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

ask() {  # condition profile rate failed -> decision: end, continue or quit
  local answer
  if [ "$auto_stop" -eq 1 ] || ! { : </dev/tty; } 2>/dev/null; then
    decision="end"
    return
  fi
  echo
  echo ">>> $2 at $3 Hz ($1): $4 did not pass."
  while :; do
    printf '>>> Did this rate clearly fail? y = yes, end this climb; n = no, go on to the next rate; q = quit: '
    read -r answer </dev/tty || answer=q
    case "$answer" in
      y|Y) decision="end"; return ;;
      n|N) decision="continue"; return ;;
      q|Q) decision="quit"; return ;;
    esac
  done
}

governors=$(sort /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | uniq -c | xargs)
commit="$(git rev-parse --short HEAD 2>/dev/null) ($(git status --porcelain --untracked-files=no 2>/dev/null | wc -l) tracked files modified)"
k=0

# One run: sets outcome to pass, fail, skip-continue, skip-ended or stop,
# and failed to the criteria that did not pass.
run_one() {
  local cond=$1 profile=$2 rate=$3 base csv note loaded=0 probe_status
  local check_status seen stress_out stress_status
  base=$(log_base "$cond" "$profile" "$rate")
  csv="$outdir/$base.csv"
  note="$csv.run.txt"
  failed=
  k=$((k + 1))
  echo
  echo "=== [$k/$total] $cond, $profile, $rate Hz: $base"
  if [ -e "$csv" ] || [ -e "$note" ]; then
    if [ -f "$note" ] && grep -qx 'climb: continue' "$note"; then
      outcome=skip-continue
    else
      outcome=skip-ended
    fi
    echo "already attempted, skipped"
    return
  fi
  case "$cond" in *loaded) loaded=1 ;; esac
  wait_quiet
  probe_cmd "$cond" "$profile" "$rate"
  {
    echo "rtctrl E1 production-traffic run"
    echo "started: $(date -Is)"
    echo "commit: $commit"
    echo "probe binary: $(stat -c %y "$PROBE")"
    echo "condition: $cond"
    echo "read path: $read_desc"
    if [ "$loaded" -eq 1 ]; then echo "load: ${stress_cmd[*]}"; else echo "load: none"; fi
    case "$cond" in
      rt-*) echo "scheduling: --rt-priority $rt_priority --rt-cpu $rt_cpu (rtprio limit $(ulimit -r))" ;;
      *) echo "scheduling: default" ;;
    esac
    echo "cpu governors: $governors"
    echo "runnable before start: $quiet (this script included)"
    echo "uptime:$(uptime)"
    printf 'command:'; printf ' %q' "${cmd[@]}"; echo
    echo
  } >"$note"
  if [ "$loaded" -eq 1 ]; then
    mkdir -p "$stress_dir"
    stress_out=$(mktemp)
    setsid "${stress_cmd[@]}" >"$stress_out" 2>&1 &
    stress_pid=$!
    sleep 2
    if ! kill -0 "$stress_pid" 2>/dev/null; then
      wait "$stress_pid"
      echo "stress-ng ended before the run (exit $?):" | tee -a "$note"
      tee -a "$note" <"$stress_out"
      rm -f "$stress_out"
      stress_pid=
      echo "climb: ended (no load)" >>"$note"
      outcome=stop
      return
    fi
  fi
  "${cmd[@]}" 2>&1 | tee -a "$note"
  probe_status=${PIPESTATUS[0]}
  if [ "$loaded" -eq 1 ]; then
    [ "$probe_status" -eq 0 ] || kill -TERM -- "-$stress_pid" 2>/dev/null
    wait "$stress_pid"
    stress_status=$?
    stress_pid=
    { echo; echo "--- stress-ng (exit $stress_status) ---"; cat "$stress_out"; } >>"$note"
    rm -f "$stress_out"
  fi
  if [ "$probe_status" -ne 0 ]; then
    { echo; echo "verdict: probe exit $probe_status"; echo "climb: ended (session stopped)"; } >>"$note"
    echo "e1_prod: the probe exited $probe_status: session stopped (stop rule). See $note"
    outcome=stop
    return
  fi
  local check_args=(--rate-hz "$rate" --check)
  [ "$loaded" -eq 1 ] && check_args+=(--loaded)
  { echo; echo "--- timing_summary.py ${check_args[*]} ---"; } >>"$note"
  uv run --project tools "$SUMMARY_TOOL" "$csv" "${check_args[@]}" 2>&1 | tee -a "$note"
  check_status=${PIPESTATUS[0]}
  seen=$(awk '/^--- timing_summary/ {s = 1} s && /^criteria/ {print "yes"; exit}' "$note")
  failed=$(awk '/^--- timing_summary/ {s = 1} s && /^criteria/ {c = 1; next}
    c && $1 ~ /^T[0-9]/ && ($3 == "FAIL" || $3 == "INDETERMINATE") {
      printf "%s%s", sep, $1; sep = ","}' "$note")
  if [ "$check_status" -eq 0 ] && [ "$seen" = yes ] && [ -z "$failed" ]; then
    { echo; echo "verdict: pass"; echo "climb: continue"; } >>"$note"
    outcome=pass
  elif [ "$check_status" -eq 1 ] && [ "$seen" = yes ] && [ -n "$failed" ]; then
    echo >>"$note"
    echo "verdict: fail ($failed)" >>"$note"
    outcome=fail
  else
    { echo; echo "verdict: analysis error (exit $check_status)"; echo "climb: ended (session stopped)"; } >>"$note"
    echo "e1_prod: timing_summary.py failed (exit $check_status): session stopped. See $note"
    outcome=stop
  fi
}

for cond in "${cond_list[@]}"; do
  for profile in "${profile_list[@]}"; do
    ended_at=
    for i in "${!rate_list[@]}"; do
      rate=${rate_list[$i]}
      if [ -n "$ended_at" ]; then
        k=$((k + 1))
        add_result "$cond" "$profile" "$rate" "not run (climb ended at $ended_at Hz)"
        continue
      fi
      current=("$cond" "$profile" "$rate")
      run_one "$cond" "$profile" "$rate"
      note="$outdir/$(log_base "$cond" "$profile" "$rate").csv.run.txt"
      case "$outcome" in
        pass) add_result "$cond" "$profile" "$rate" "PASS" ;;
        skip-continue) add_result "$cond" "$profile" "$rate" "skipped (attempted earlier; climb went on)" ;;
        skip-ended)
          add_result "$cond" "$profile" "$rate" "skipped (attempted earlier; climb ended)"
          ended_at=$rate ;;
        stop)
          add_result "$cond" "$profile" "$rate" "STOPPED the session (see the .run.txt)"
          print_table
          exit 1 ;;
        fail)
          if [ "$i" -eq $(( ${#rate_list[@]} - 1 )) ]; then
            echo "climb: ended (top rate)" >>"$note"
            add_result "$cond" "$profile" "$rate" "FAIL $failed (top rate)"
            current=()
            continue
          fi
          ask "$cond" "$profile" "$rate" "$failed"
          case "$decision" in
            end)
              echo "climb: ended (operator: clear failure)" >>"$note"
              add_result "$cond" "$profile" "$rate" "FAIL $failed; climb ended"
              ended_at=$rate ;;
            continue)
              echo "operator: not a clear failure" >>"$note"
              echo "climb: continue" >>"$note"
              add_result "$cond" "$profile" "$rate" "FAIL $failed; climb went on" ;;
            quit)
              echo "climb: ended (operator quit the session)" >>"$note"
              add_result "$cond" "$profile" "$rate" "FAIL $failed; session quit"
              print_table
              exit 1 ;;
          esac ;;
      esac
      current=()
    done
  done
done
print_table
