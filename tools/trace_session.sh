#!/usr/bin/env bash
# Traced torque-off diagnostics of the bounded read
# (docs/hardware/control-rate.md, after the B2 and B3 runs).
#
# Run it from the rtctrl repo root, after a Release build, with every servo
# torque-off and the arm at rest and supported. x7_bus_timing refuses to
# measure otherwise, and it never writes Torque Enable or Drive Mode.
#
# Each listed run is made twice, untraced and then traced, under the same
# conditions, so the pair shows the tracing overhead. A traced run records,
# around the probe:
#   - scheduler, interrupt, timer, workqueue and xHCI URB events with
#     trace-cmd on ftrace's mono clock (CLOCK_MONOTONIC, the clock of the
#     probe's monotonic_origin_ns); the session stops if the clock is not
#     mono;
#   - the adapter's usbmon traffic with tcpdump (usbmon stamps are
#     CLOCK_REALTIME, so the record also takes CLOCK_MONOTONIC and
#     CLOCK_REALTIME pairs before and after the run).
# The tracers run on the CPUs outside the excluded set (the real-time CPU
# and its SMT siblings by default), and read nothing on the bus.
#
# The session first records the host state once: the fair server's
# settings (read with bpftrace, since the kernel's lockdown refuses the
# debugfs files), lockdown, boot line, topology and the adapter. Every run
# record repeats it, with the run's IRQ placement before and after and a
# task and affinity snapshot halfway through the run. The session changes
# no scheduler setting; it loads the usbmon module if needed and unloads it
# at the end.
#
# Conditions: idle, loaded, loaded-excl, rt-idle, rt-loaded, rt-loaded-excl.
# A loaded run starts stress-ng --cpu <n> --hdd 1; an -excl run pins it to
# the CPUs outside the excluded set, with one --cpu worker per CPU there.
#
# The load starts once the recorders record, and the load and the
# recorders must last the whole run: one that ends before the probe does
# voids the run, as does a load that does not report success.
#
# A failed criterion is data, not a stop. A probe that exits nonzero, a
# tracer that does not start, stop cleanly or last the run, a load that
# does not last the run or succeed, or an analysis error stops the session
# (the stop rule).
#
# Each run leaves gitignored files in the output directory:
#   <log>.csv               per-cycle log (probe)
#   <log>.csv.summary.toml  run summary (probe)
#   <log>.csv.run.txt       command, host state, probe output, verdicts
#   <log>.csv.tasks.txt     tasks, CPUs and affinities halfway through
#   <log>.csv.trace.dat     trace-cmd record (traced runs)
#   <log>.csv.usbmon.pcap   usbmon capture of the adapter (traced runs)
# with <log> = rate-trace-<profile>-<rate>hz-<condition>-bounded<variant
# tags>-<untraced|traced>-<date>-<attempt>. A run whose .csv or .run.txt
# already exists was attempted: it is skipped, never retried.
#
# Privileged steps (trace-cmd, tcpdump, bpftrace, modprobe) go through sudo;
# the probe runs as the invoking user. For tests, TRACE_SESSION_SUDO=none
# runs them directly and TRACE_SESSION_SUDO=<command> uses that command in
# sudo's place, and TRACE_CMD, TCPDUMP, BPFTRACE, MODPROBE, STRESS_NG and
# TRACEFS replace the tools and the tracefs path.
set -u

PROBE=./build/apps/x7_bus_timing  # --probe overrides it (the tests do)
SUMMARY_TOOL=tools/timing_summary.py
CONFIG=config/crane_x7.toml       # the probe's default configuration
TRACE_CMD=${TRACE_CMD:-trace-cmd}
TCPDUMP=${TCPDUMP:-tcpdump}
BPFTRACE=${BPFTRACE:-bpftrace}
MODPROBE=${MODPROBE:-modprobe}
STRESS_NG=${STRESS_NG:-stress-ng}
TRACEFS=${TRACEFS:-/sys/kernel/tracing}
case "${TRACE_SESSION_SUDO:-sudo}" in
  none) sudo_cmd=() ;;
  *) sudo_cmd=("${TRACE_SESSION_SUDO:-sudo}") ;;
esac

# The targeted set decided on 2026-11-05: Fast Sync Read with the
# motion-only read at 500 and 1000 Hz, quiet and loaded, with and without
# the load on the probe's core; and the cbp 1000 Hz idle anomaly.
runs=current:500:rt-idle:fast-motion,current:500:rt-loaded:fast-motion
runs+=,current:500:rt-loaded-excl:fast-motion,current:1000:rt-idle:fast-motion
runs+=,current:1000:rt-loaded:fast-motion,current:1000:rt-loaded-excl:fast-motion
runs+=,cbp:1000:rt-idle:fast
modes=untraced,traced
duration=60
rt_priority=80
rt_cpu=3
exclude_cpus=
port=
usb_device=
usbmon=1
buffer_kb=16384
date_tag=$(date +%Y%m%d)
attempt=1
outdir=.
max_runnable=3
load_tail=15
stress_dir="$HOME/.cache/rtctrl-stress"
dry_run=0
assume_yes=0

usage() {
  cat <<EOF
usage: $0 [options]
  --runs LIST         comma-separated profile:rate:condition:variant
                      (default: the targeted set, see the script header)
                      profile   current, position or cbp
                      condition idle, loaded, loaded-excl, rt-idle,
                                rt-loaded or rt-loaded-excl
                      variant   full, motion, motion-aux0ms, fast,
                                fast-motion or fast-motion-aux0ms (all on
                                the bounded read path)
  --modes LIST        untraced,traced (default), or one of them
  --duration S        seconds per run (default $duration)
  --rt-priority N     SCHED_FIFO priority 1 to 99 for rt-* (default $rt_priority)
  --rt-cpu C          CPU for rt-* (default $rt_cpu)
  --exclude-cpus L    CPUs kept free of load in -excl runs and of the
                      tracers (default: the real-time CPU's SMT siblings)
  --probe PATH        x7_bus_timing binary (default $PROBE)
  --port P            serial port (default: the probe's configuration)
  --usb-device DIR    the adapter's sysfs USB device directory (default:
                      found from the port)
  --no-usbmon         trace without usbmon (an emulator rehearsal)
  --buffer-kb N       trace-cmd buffer per CPU (default $buffer_kb)
  --date YYYYMMDD     date in the log names (default today, $date_tag)
  --attempt N         attempt number in the log names (default $attempt)
  --outdir DIR        where the logs go (default the repo root)
  --max-runnable N    quiet-machine threshold, mean runnable tasks
                      including this script (default $max_runnable)
  --stress-dir DIR    stress-ng --hdd directory, on a disk, not tmpfs
                      (default $stress_dir)
  --load-tail S       seconds the load's timeout exceeds the run's duration
                      (default $load_tail)
  --dry-run           check the prerequisites and print the plan only
  --yes               confirm the session in advance (no prompt)
EOF
}

die() { echo "trace_session: $*" >&2; exit 2; }

while [ $# -gt 0 ]; do
  case "$1" in
    --runs|--modes|--duration|--rt-priority|--rt-cpu|--exclude-cpus|--probe|\
    --port|--usb-device|--buffer-kb|--date|--attempt|--outdir|--max-runnable|\
    --stress-dir|--load-tail)
      [ $# -ge 2 ] || die "$1 needs a value"
      case "$1" in
        --runs) runs=$2 ;;
        --modes) modes=$2 ;;
        --duration) duration=$2 ;;
        --rt-priority) rt_priority=$2 ;;
        --rt-cpu) rt_cpu=$2 ;;
        --exclude-cpus) exclude_cpus=$2 ;;
        --probe) PROBE=$2 ;;
        --port) port=$2 ;;
        --usb-device) usb_device=$2 ;;
        --buffer-kb) buffer_kb=$2 ;;
        --date) date_tag=$2 ;;
        --attempt) attempt=$2 ;;
        --outdir) outdir=$2 ;;
        --max-runnable) max_runnable=$2 ;;
        --stress-dir) stress_dir=$2 ;;
        --load-tail) load_tail=$2 ;;
      esac
      shift 2 ;;
    --no-usbmon) usbmon=0; shift ;;
    --dry-run) dry_run=1; shift ;;
    --yes) assume_yes=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
done

is_uint() { [[ $1 =~ ^[0-9]+$ ]]; }

# "0-3,8" -> "0 1 2 3 8"
expand_cpus() {
  local part parts out=() a
  IFS=, read -r -a parts <<<"$1"
  for part in "${parts[@]}"; do
    if [[ $part =~ ^([0-9]+)-([0-9]+)$ ]]; then
      for ((a = BASH_REMATCH[1]; a <= BASH_REMATCH[2]; a++)); do out+=("$a"); done
    elif [[ $part =~ ^[0-9]+$ ]]; then
      out+=("$part")
    else
      return 1
    fi
  done
  echo "${out[*]}"
}

# The variant's probe flags (vflags) and log-name tag (vtag).
variant_flags() {
  vflags=(--read-path bounded)
  vtag=-bounded
  case "$1" in
    full) ;;
    motion) vflags+=(--feedback motion); vtag+=-motion ;;
    motion-aux0ms) vflags+=(--feedback motion --aux-interval-ms 0); vtag+=-motion-aux0ms ;;
    fast) vflags+=(--read-instruction fast); vtag+=-fast ;;
    fast-motion) vflags+=(--read-instruction fast --feedback motion); vtag+=-fast-motion ;;
    fast-motion-aux0ms)
      vflags+=(--read-instruction fast --feedback motion --aux-interval-ms 0)
      vtag+=-fast-motion-aux0ms ;;
    *) return 1 ;;
  esac
}

IFS=, read -r -a run_list <<<"$runs"
IFS=, read -r -a mode_list <<<"$modes"
[ ${#run_list[@]} -gt 0 ] || die "--runs is empty"
any_rt=0; any_loaded=0; any_traced=0
for spec in "${run_list[@]}"; do
  IFS=: read -r p r c v extra <<<"$spec"
  [ -z "${extra:-}" ] && [ -n "${v:-}" ] || die "run '$spec' is not profile:rate:condition:variant"
  case "$p" in current|position|cbp) ;; *) die "run '$spec': unknown profile '$p'" ;; esac
  is_uint "$r" && [ "$r" -gt 0 ] || die "run '$spec': the rate must be a positive integer"
  case "$c" in
    idle|loaded|loaded-excl|rt-idle|rt-loaded|rt-loaded-excl) ;;
    *) die "run '$spec': unknown condition '$c'" ;;
  esac
  variant_flags "$v" || die "run '$spec': unknown variant '$v'"
  case "$c" in rt-*) any_rt=1 ;; esac
  case "$c" in *loaded*) any_loaded=1 ;; esac
done
for m in "${mode_list[@]}"; do
  case "$m" in untraced) ;; traced) any_traced=1 ;; *) die "unknown mode '$m'" ;; esac
done
[[ $duration =~ ^[0-9]+$ ]] && [ "$duration" -gt 0 ] || die "--duration must be a positive integer"
is_uint "$attempt" && [ "$attempt" -gt 0 ] || die "--attempt must be a positive integer"
is_uint "$rt_priority" && is_uint "$rt_cpu" || die "--rt-priority and --rt-cpu must be integers"
# The probe reads priority 0 as ordinary scheduling, never SCHED_FIFO.
if [ "$rt_priority" -lt 1 ] || [ "$rt_priority" -gt 99 ]; then
  die "--rt-priority must be 1 to 99 (0 would mean ordinary scheduling)"
fi
is_uint "$max_runnable" || die "--max-runnable must be an integer"
is_uint "$load_tail" && [ "$load_tail" -gt 0 ] || die "--load-tail must be a positive integer"
is_uint "$buffer_kb" && [ "$buffer_kb" -gt 0 ] || die "--buffer-kb must be a positive integer"
[[ $date_tag =~ ^[0-9]{8}$ ]] || die "--date must be YYYYMMDD"

online=$(expand_cpus "$(cat /sys/devices/system/cpu/online)") || die "cannot read the online CPUs"
if [ -z "$exclude_cpus" ]; then
  exclude_cpus=$(cat "/sys/devices/system/cpu/cpu$rt_cpu/topology/thread_siblings_list" 2>/dev/null) ||
    die "cannot read CPU $rt_cpu's SMT siblings (pass --exclude-cpus)"
fi
excluded=$(expand_cpus "$exclude_cpus") || die "--exclude-cpus '$exclude_cpus' is not a CPU list"
others=()
for c in $online; do
  case " $excluded " in *" $c "*) ;; *) others+=("$c") ;; esac
done
[ ${#others[@]} -gt 0 ] || die "--exclude-cpus leaves no CPU for the load and the tracers"
other_cpus=$(IFS=,; echo "${others[*]}")
stress_timeout=$((duration + load_tail))

# The adapter to capture: the probe's port, unless --usb-device names it.
if [ "$any_traced" -eq 1 ] && [ "$usbmon" -eq 1 ] && [ -z "$usb_device" ]; then
  tty_path=$port
  [ -n "$tty_path" ] || tty_path=$(sed -n 's/^port *= *"\(.*\)"/\1/p' "$CONFIG" | head -n 1)
  d=$(readlink -f "/sys/class/tty/$(basename "$(readlink -f "$tty_path")")/device" 2>/dev/null)
  while [ -n "$d" ] && [ "$d" != / ] && [ ! -f "$d/devnum" ]; do d=$(dirname "$d"); done
  [ -n "$d" ] && [ -f "$d/devnum" ] && usb_device=$d
fi

# Prerequisites: every problem is listed, then the script refuses to start
# (a dry run lists them and goes on).
problems=()
[ -x "$PROBE" ] && [ -f "$SUMMARY_TOOL" ] ||
  problems+=("run from the repo root after building ($PROBE, $SUMMARY_TOOL)")
[ -d "$outdir" ] && [ -w "$outdir" ] || problems+=("--outdir $outdir is not a writable directory")
command -v taskset >/dev/null || problems+=("taskset is not installed (util-linux)")
command -v stdbuf >/dev/null || problems+=("stdbuf is not installed (coreutils)")
command -v python3 >/dev/null || problems+=("python3 is not installed")
command -v "$BPFTRACE" >/dev/null ||
  problems+=("$BPFTRACE is not installed: it reads the fair server's settings")
if [ "$any_traced" -eq 1 ]; then
  command -v "$TRACE_CMD" >/dev/null || problems+=("$TRACE_CMD is not installed")
  if [ "$usbmon" -eq 1 ]; then
    command -v "$TCPDUMP" >/dev/null || problems+=("$TCPDUMP is not installed")
    if [ -z "$usb_device" ] || [ ! -f "$usb_device/busnum" ] || [ ! -f "$usb_device/devnum" ]; then
      problems+=("no USB device to capture: the port is not a USB serial adapter (pass --usb-device, or --no-usbmon for a rehearsal)")
    fi
  fi
fi
if [ "$any_rt" -eq 1 ]; then
  lim=$(ulimit -r)
  if [ "$lim" != unlimited ] && [ "$lim" -lt "$rt_priority" ]; then
    problems+=("real-time priority limit is $lim, below $rt_priority: in the shell that starts this script run
      sudo prlimit --pid \$\$ --rtprio=90:90")
  fi
  case " $online " in *" $rt_cpu "*) ;; *) problems+=("--rt-cpu $rt_cpu is not an online CPU") ;; esac
  case " $excluded " in *" $rt_cpu "*) ;; *) problems+=("--exclude-cpus $exclude_cpus leaves out the real-time CPU $rt_cpu") ;; esac
fi
if [ "$any_loaded" -eq 1 ]; then
  command -v "$STRESS_NG" >/dev/null ||
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
  for p in "${problems[@]}"; do echo "trace_session: $p" >&2; done
  [ "$dry_run" -eq 1 ] || exit 2
fi

usb_bus=; usb_devnum=
if [ -n "$usb_device" ] && [ -f "$usb_device/busnum" ]; then
  usb_bus=$(cat "$usb_device/busnum")
  usb_devnum=$(cat "$usb_device/devnum")
fi

trace_events=(
  sched:sched_switch sched:sched_waking sched:sched_wakeup sched:sched_migrate_task
  irq:irq_handler_entry irq:irq_handler_exit irq:softirq_entry irq:softirq_exit
  irq_vectors:local_timer_entry irq_vectors:local_timer_exit
  irq_vectors:reschedule_entry irq_vectors:reschedule_exit
  irq_vectors:call_function_entry irq_vectors:call_function_exit
  irq_vectors:call_function_single_entry irq_vectors:call_function_single_exit
  timer:hrtimer_expire_entry timer:hrtimer_expire_exit
  workqueue:workqueue_queue_work workqueue:workqueue_execute_start
  workqueue:workqueue_execute_end
  xhci-hcd:xhci_urb_enqueue xhci-hcd:xhci_urb_giveback
)
trace_args=(record -C mono -b "$buffer_kb")
for e in "${trace_events[@]}"; do trace_args+=(-e "$e"); done
# usbmon's mmapped header carries the device address at byte 11.
usbmon_opts=(-i "usbmon$usb_bus" -Z root -s 0)
usbmon_filter="link[11] == $usb_devnum"

stress_cmd() {  # condition -> scmd[]
  scmd=()
  case "$1" in
    *loaded-excl)
      scmd=(taskset -c "$other_cpus" "$STRESS_NG" --cpu "${#others[@]}" --hdd 1
            --timeout "${stress_timeout}s" --temp-path "$stress_dir") ;;
    *loaded)
      scmd=("$STRESS_NG" --cpu "$(nproc)" --hdd 1 --timeout "${stress_timeout}s"
            --temp-path "$stress_dir") ;;
  esac
}

log_base() {  # profile rate condition variant mode
  variant_flags "$4"
  echo "rate-trace-$1-${2}hz-$3$vtag-$5-$date_tag-$attempt"
}

probe_cmd() {  # profile rate condition variant mode -> cmd[]
  local base; base=$(log_base "$@")
  variant_flags "$4"
  cmd=("$PROBE")
  [ -n "$port" ] && cmd+=(--port "$port")
  cmd+=(--rate-hz "$2" --profile "$1" --duration "$duration"
        --log "$outdir/$base.csv")
  case "$3" in rt-*) cmd+=(--rt-priority "$rt_priority" --rt-cpu "$rt_cpu") ;; esac
  cmd+=("${vflags[@]}")
}

total=$(( ${#run_list[@]} * ${#mode_list[@]} ))

if [ "$dry_run" -eq 1 ]; then
  echo "plan: $total runs (date $date_tag, attempt $attempt), tracers and -excl load on CPUs $other_cpus:"
  for spec in "${run_list[@]}"; do
    IFS=: read -r p r c v <<<"$spec"
    for m in "${mode_list[@]}"; do
      probe_cmd "$p" "$r" "$c" "$v" "$m"
      stress_cmd "$c"
      printf '  [%s' "$m"
      [ ${#scmd[@]} -gt 0 ] && printf '; load: %s' "${scmd[*]}"
      printf '] '; printf '%q ' "${cmd[@]}"; echo
    done
  done
  if [ "$any_traced" -eq 1 ]; then
    printf 'traced runs add: stdbuf -oL -eL %s %s -o <log>.csv.trace.dat\n' "$TRACE_CMD" "${trace_args[*]}"
    if [ "$usbmon" -eq 1 ]; then
      printf "                 %s %s -w <log>.csv.usbmon.pcap '%s' (device %s)\n" \
        "$TCPDUMP" "${usbmon_opts[*]}" "$usbmon_filter" "${usb_device:-unknown}"
    fi
  fi
  echo "about $(( (total * (duration + 30) + 59) / 60 )) minutes"
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

tmp=$(mktemp -d)
stress_pid=
keepalive_pid=
trace_job=; usbmon_job=
usbmon_loaded_here=0
run_outputs=()  # the tracer outputs of the run in progress, for own_outputs
results=()
add_result() { results+=("$(printf '%-28s %-9s %s' "$1" "$2" "$3")"); }
print_table() {
  echo
  echo "Traced torque-off results (date $date_tag, attempt $attempt):"
  for line in "${results[@]}"; do echo "  $line"; done
}

priv() { "${sudo_cmd[@]}" "$@"; }

# Starts a privileged tracer in the background, on the tracer CPUs. A
# script's background commands start with SIGINT ignored, so the wrapper
# restores its default before the tracer runs (the tracer stops on SIGINT)
# and records the tracer's PID (the wrapper execs the command, so the PID
# stays the tracer's). The output goes through a pipe: AppArmor would
# replace an inherited file descriptor that tcpdump's profile does not
# allow with /dev/null.
start_tracer() {  # name -> <name>_job; the rest is the command
  local name=$1; shift
  rm -f "$tmp/$name.pid" "$tmp/$name.out"
  taskset -c "$other_cpus" "${sudo_cmd[@]}" python3 -c '
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
with open(sys.argv[1], "w") as f:
    f.write(str(os.getpid()))
os.execvp(sys.argv[2], sys.argv[2:])' "$tmp/$name.pid" "$@" \
    </dev/null > >(cat >"$tmp/$name.out") 2>&1 &
  printf -v "${name}_job" %s "$!"
}
# Whether a process is running (not a zombie). A user's kill -0 on a job
# that runs under sudo can fail with EPERM, so this reads /proc.
alive() {
  local state
  state=$(sed 's/.*) //' "/proc/$1/stat" 2>/dev/null | cut -d ' ' -f 1)
  [ -n "$state" ] && [ "$state" != Z ]
}
wait_line() {  # name job pattern seconds -> 0 once the tracer printed it
  local i
  for ((i = 0; i < $4 * 10; i++)); do
    grep -qF -- "$3" "$tmp/$1.out" 2>/dev/null && return 0
    alive "$2" || break
    sleep 0.1
  done
  grep -qF -- "$3" "$tmp/$1.out" 2>/dev/null
}
stop_tracer() {  # name job -> its exit status (SIGINT, then SIGKILL after 120 s)
  local pid status i
  pid=$(cat "$tmp/$1.pid" 2>/dev/null)
  [ -n "$pid" ] && priv kill -INT "$pid" 2>/dev/null
  for ((i = 0; i < 1200; i++)); do alive "$2" || break; sleep 0.1; done
  if alive "$2"; then
    echo "trace_session: $1 did not stop on SIGINT; killing it" >&2
    priv kill -KILL "${pid:-$2}" 2>/dev/null
  fi
  wait "$2"
  status=$?
  sleep 0.2  # let the pipe's reader finish the output file
  return "$status"
}
recorders_ended() {  # the run's recorders that are no longer running
  local gone=''
  alive "$trace_job" || gone="trace-cmd ended"
  if [ "$usbmon" -eq 1 ]; then alive "$usbmon_job" || gone+="${gone:+, }tcpdump ended"; fi
  echo "$gone"
}
# Gives the run's tracer outputs that exist to the invoking user. Every
# path that stops the recorders calls it (stop_all does, and so does the
# cleanup of a stopped or interrupted session through finish).
own_outputs() {
  local f
  [ ${#sudo_cmd[@]} -gt 0 ] || return 0
  for f in "${run_outputs[@]}"; do
    [ -e "$f" ] && priv chown "$(id -u):$(id -g)" "$f" 2>/dev/null
  done
  return 0
}
stop_all() {
  if [ -n "$trace_job" ]; then stop_tracer trace "$trace_job"; trace_job=; fi
  if [ -n "$usbmon_job" ]; then stop_tracer usbmon "$usbmon_job"; usbmon_job=; fi
  if [ -n "$stress_pid" ]; then
    kill -TERM -- "-$stress_pid" 2>/dev/null
    wait "$stress_pid" 2>/dev/null
    stress_pid=
  fi
  own_outputs
}
finish() {
  stop_all
  if [ "$usbmon_loaded_here" -eq 1 ]; then priv "$MODPROBE" -r usbmon; fi
  [ -n "$keepalive_pid" ] && kill "$keepalive_pid" 2>/dev/null
  rm -rf "$tmp"
}
on_interrupt() {
  echo
  echo "trace_session: interrupted; the run in progress counts as attempted"
  [ -n "${current:-}" ] && add_result "$current" "${current_mode:-}" "INTERRUPTED (counts as attempted)"
  finish
  print_table
  exit 130
}
trap on_interrupt INT TERM

if [ ${#sudo_cmd[@]} -gt 0 ]; then
  "${sudo_cmd[@]}" -v || die "sudo is needed for the tracers and the fair-server read"
  # Keep sudo's timestamp fresh for the whole session.
  ( while sleep 60; do "${sudo_cmd[@]}" -n -v </dev/null 2>/dev/null || exit; done ) &
  keepalive_pid=$!
fi

# Host state, once per session.
cat >"$tmp/dl_servers.bt" <<'EOF'
kprobe:dl_server_start,
kprobe:dl_server_update
{
  $se = (struct sched_dl_entity *)arg0;
  if (@seen[(uint64)$se] == 0) {
    @seen[(uint64)$se] = 1;
    printf("cpu %d %s runtime_ns %llu period_ns %llu deadline_ns %llu dl_server %d dl_defer %d\n",
           $se->rq->cpu, ksym((uint64)$se->server_pick_task),
           $se->dl_runtime, $se->dl_period, $se->dl_deadline,
           $se->dl_server, $se->dl_defer);
  }
}
interval:s:5 { exit(); }
END { clear(@seen); }
EOF
priv "$BPFTRACE" "$tmp/dl_servers.bt" >"$tmp/dl_servers.out" 2>&1
bpf_status=$?
sort -n -k 2 "$tmp/dl_servers.out" | grep '^cpu ' >"$tmp/dl_servers.txt"
if [ "$bpf_status" -ne 0 ] || ! grep -q "^cpu $rt_cpu fair_server_pick_task " "$tmp/dl_servers.txt"; then
  cat "$tmp/dl_servers.out" >&2
  finish
  die "the fair server's settings for CPU $rt_cpu were not read (bpftrace exit $bpf_status): nothing was sent"
fi
if [ "$any_traced" -eq 1 ] && [ "$usbmon" -eq 1 ]; then
  if [ ! -d /sys/module/usbmon ]; then
    priv "$MODPROBE" usbmon || { finish; die "modprobe usbmon failed: nothing was sent"; }
    usbmon_loaded_here=1
  fi
fi
{
  echo "--- host state (session) ---"
  echo "kernel: $(uname -srvm)"
  echo "lockdown: $(cat /sys/kernel/security/lockdown 2>&1)"
  echo "secure boot: $(mokutil --sb-state 2>&1 | head -n 1)"
  echo "boot line: $(cat /proc/cmdline)"
  echo "sched_rt_runtime_us: $(cat /proc/sys/kernel/sched_rt_runtime_us) of $(cat /proc/sys/kernel/sched_rt_period_us)"
  echo "fair and ext servers (bpftrace, read-only; per CPU):"
  sed 's/^/  /' "$tmp/dl_servers.txt"
  echo "online CPUs: $(cat /sys/devices/system/cpu/online); CPU $rt_cpu SMT siblings: $(cat "/sys/devices/system/cpu/cpu$rt_cpu/topology/thread_siblings_list" 2>&1)"
  echo "excluded CPUs (no -excl load, no tracers): $exclude_cpus; tracer and -excl load CPUs: $other_cpus"
  echo "topology (lscpu -e):"
  lscpu -e=CPU,CORE,SOCKET,NODE,ONLINE 2>&1 | sed 's/^/  /'
  echo "cpu governors: $(sort /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2>/dev/null | uniq -c | xargs)"
  echo "irqbalance: $(systemctl is-active irqbalance 2>&1)"
  if [ -n "$usb_device" ]; then
    echo "adapter: $usb_device, $(cat "$usb_device/idVendor" 2>&1):$(cat "$usb_device/idProduct" 2>&1), $(cat "$usb_device/speed" 2>&1) Mbps, bus $usb_bus device $usb_devnum"
  fi
  for lt in /sys/bus/usb-serial/devices/*/latency_timer; do
    [ -f "$lt" ] && echo "latency_timer $(basename "$(dirname "$lt")"): $(cat "$lt")"
  done
  if [ "$usbmon" -eq 1 ] && [ "$any_traced" -eq 1 ]; then
    echo "usbmon: $([ "$usbmon_loaded_here" -eq 1 ] && echo "loaded by this session" || echo "already loaded")"
  else
    echo "usbmon: not used"
  fi
  echo "tracers: $("$TRACE_CMD" -h 2>&1 | grep -m 1 -i version); $("$TCPDUMP" --version 2>&1 | head -n 1); $("$BPFTRACE" --version 2>&1 | head -n 1)"
} >"$tmp/session.txt"

irq_state() {
  local i
  grep -E '^ *CPU0|xhci' /proc/interrupts
  for i in $(awk -F: '/xhci/ {gsub(/ /, "", $1); print $1}' /proc/interrupts); do
    echo "irq $i: smp_affinity_list $(cat "/proc/irq/$i/smp_affinity_list" 2>&1), effective $(cat "/proc/irq/$i/effective_affinity_list" 2>&1)"
  done
}
tasks_snapshot() {
  echo "taken: $(date -Is)"
  echo "--- ps -eLo pid,tid,psr,cls,rtprio,ni,stat,comm ---"
  ps -eLo pid,tid,psr,cls,rtprio,ni,stat,comm
  echo "--- Cpus_allowed_list per task (/proc/<pid>/task/<tid>/status) ---"
  awk '/^Cpus_allowed_list/ {print FILENAME, $2}' /proc/[0-9]*/task/[0-9]*/status 2>/dev/null
}
clock_pair() {  # the tightest of 2000 CLOCK_MONOTONIC/REALTIME/MONOTONIC reads
  python3 - <<'EOF'
import time
M, R = time.CLOCK_MONOTONIC, time.CLOCK_REALTIME
best = None
for _ in range(2000):
    a = time.clock_gettime_ns(M)
    r = time.clock_gettime_ns(R)
    b = time.clock_gettime_ns(M)
    if best is None or b - a < best[2] - best[0]:
        best = (a, r, b)
a, r, b = best
print(f"monotonic_before_ns {a} realtime_ns {r} monotonic_after_ns {b} "
      f"realtime_minus_monotonic_ns {r - (a + b) // 2} bracket_ns {b - a}")
EOF
}

commit="$(git rev-parse --short HEAD 2>/dev/null) ($(git status --porcelain --untracked-files=no 2>/dev/null | wc -l) tracked files modified)"

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

# One run: sets outcome to pass, fail, skip or stop, and failed to the
# criteria that did not pass.
k=0
run_one() {
  local profile=$1 rate=$2 cond=$3 variant=$4 mode=$5 base csv note
  local loaded=0 probe_status check_status seen stress_out stress_status snap_pid
  local dat pcap
  base=$(log_base "$profile" "$rate" "$cond" "$variant" "$mode")
  csv="$outdir/$base.csv"
  note="$csv.run.txt"
  dat="$csv.trace.dat"
  pcap="$csv.usbmon.pcap"
  run_outputs=()
  failed=
  k=$((k + 1))
  echo
  echo "=== [$k/$total] $profile, $rate Hz, $cond, $variant, $mode: $base"
  if [ -e "$csv" ] || [ -e "$note" ]; then
    echo "already attempted, skipped"
    outcome=skip
    return
  fi
  case "$cond" in *loaded*) loaded=1 ;; esac
  wait_quiet
  probe_cmd "$profile" "$rate" "$cond" "$variant" "$mode"
  stress_cmd "$cond"
  {
    echo "rtctrl traced torque-off run"
    echo "started: $(date -Is)"
    echo "commit: $commit"
    echo "probe binary: $(stat -c %y "$PROBE")"
    echo "condition: $cond"
    echo "variant: $variant (bounded read path: the probe's 35 ms budget and equal guard)"
    echo "mode: $mode"
    if [ "$loaded" -eq 1 ]; then echo "load: ${scmd[*]}"; else echo "load: none"; fi
    case "$cond" in
      rt-*) echo "scheduling: --rt-priority $rt_priority --rt-cpu $rt_cpu (rtprio limit $(ulimit -r))" ;;
      *) echo "scheduling: default" ;;
    esac
    echo "runnable before start: $quiet (this script included)"
    echo "uptime:$(uptime)"
    printf 'command:'; printf ' %q' "${cmd[@]}"; echo
    if [ "$mode" = traced ]; then
      printf 'trace: stdbuf -oL -eL %s' "$TRACE_CMD"; printf ' %q' "${trace_args[@]}"; printf ' -o %q\n' "$dat"
      if [ "$usbmon" -eq 1 ]; then
        printf 'usbmon: %s' "$TCPDUMP"; printf ' %q' "${usbmon_opts[@]}" -w "$pcap" "$usbmon_filter"; echo
      else
        echo "usbmon: not used (--no-usbmon)"
      fi
    fi
    cat "$tmp/session.txt"
    echo "--- IRQ placement before ---"
    irq_state
    echo
  } >"$note"
  if [ "$mode" = traced ]; then
    run_outputs=("$dat")
    if [ "$usbmon" -eq 1 ]; then run_outputs+=("$pcap"); fi
    # On a pipe, the C library would hold trace-cmd's "Hit Ctrl^C to stop
    # recording" until it exits; stdbuf (which execs it, keeping the PID)
    # makes its output line-buffered. tcpdump reports on stderr, which is
    # unbuffered.
    start_tracer trace stdbuf -oL -eL "$TRACE_CMD" "${trace_args[@]}" -o "$dat"
    if ! wait_line trace "$trace_job" "Hit Ctrl^C to stop recording" 60; then
      stop_all
      { echo "--- trace-cmd did not start ---"; cat "$tmp/trace.out"; } | tee -a "$note"
      echo "verdict: not run (tracer)" >>"$note"
      outcome=stop
      return
    fi
    clock_line=$(priv cat "$TRACEFS/trace_clock" 2>&1)
    echo "trace clock while recording: $clock_line" >>"$note"
    case "$clock_line" in
      *"[mono]"*) ;;
      *)
        echo "trace_session: the trace clock is not mono ($clock_line)" | tee -a "$note"
        echo "verdict: not run (trace clock)" >>"$note"
        stop_all
        outcome=stop
        return ;;
    esac
    if [ "$usbmon" -eq 1 ]; then
      start_tracer usbmon "$TCPDUMP" "${usbmon_opts[@]}" -w "$pcap" "$usbmon_filter"
      if ! wait_line usbmon "$usbmon_job" "listening on usbmon$usb_bus" 30; then
        stop_all
        { echo "--- tcpdump did not start ---"; cat "$tmp/usbmon.out"; } | tee -a "$note"
        echo "verdict: not run (tracer)" >>"$note"
        outcome=stop
        return
      fi
    fi
  fi
  # The load starts once the recorders are recording, so their startup
  # takes none of its time, and it must outlast the probe.
  if [ "$loaded" -eq 1 ]; then
    mkdir -p "$stress_dir"
    stress_out=$(mktemp)
    setsid "${scmd[@]}" >"$stress_out" 2>&1 &
    stress_pid=$!
    sleep 2
    if ! alive "$stress_pid"; then
      wait "$stress_pid"
      echo "stress-ng ended before the run (exit $?):" | tee -a "$note"
      tee -a "$note" <"$stress_out"
      rm -f "$stress_out"
      stress_pid=
      echo "verdict: not run (no load)" >>"$note"
      stop_all
      outcome=stop
      return
    fi
  fi
  if [ "$mode" = traced ]; then
    local ended
    ended=$(recorders_ended)
    if [ -n "$ended" ]; then
      echo "trace_session: $ended before the run: session stopped" | tee -a "$note"
      echo "verdict: not run (tracer)" >>"$note"
      stop_all
      outcome=stop
      return
    fi
    echo "clock pair before: $(clock_pair)" >>"$note"
  fi
  ( taskset -cp "$other_cpus" "$BASHPID" >/dev/null
    sleep $((duration / 2)); tasks_snapshot >"$csv.tasks.txt" ) &
  snap_pid=$!
  "${cmd[@]}" 2>&1 | tee -a "$note"
  probe_status=${PIPESTATUS[0]}
  # The load and the recorders must have lasted the whole run: checked as
  # the probe ends, before anything is stopped.
  local load_problem='' tracer_problem=''
  if [ "$loaded" -eq 1 ] && ! alive "$stress_pid"; then
    load_problem="the load ended before the probe did"
  fi
  if [ "$mode" = traced ]; then
    tracer_problem=$(recorders_ended)
    [ -n "$tracer_problem" ] && tracer_problem+=" before the probe did"
  fi
  wait "$snap_pid" 2>/dev/null
  if [ "$mode" = traced ]; then
    echo "clock pair after: $(clock_pair)" >>"$note"
    local trace_status usbmon_status=0
    stop_tracer trace "$trace_job"; trace_status=$?; trace_job=
    { echo; echo "--- trace-cmd (exit $trace_status) ---"; cat "$tmp/trace.out"; } >>"$note"
    [ "$trace_status" -eq 0 ] && [ -s "$dat" ] ||
      tracer_problem+="${tracer_problem:+, }trace-cmd exit $trace_status"
    if [ "$usbmon" -eq 1 ]; then
      stop_tracer usbmon "$usbmon_job"; usbmon_status=$?; usbmon_job=
      { echo; echo "--- tcpdump (exit $usbmon_status) ---"; cat "$tmp/usbmon.out"; } >>"$note"
      [ "$usbmon_status" -eq 0 ] && [ -s "$pcap" ] ||
        tracer_problem+="${tracer_problem:+, }tcpdump exit $usbmon_status"
    fi
    own_outputs
  fi
  if [ "$loaded" -eq 1 ]; then
    # A failed probe ends the load at once; otherwise it runs out its
    # timeout and must report success.
    [ "$probe_status" -eq 0 ] || kill -TERM -- "-$stress_pid" 2>/dev/null
    wait "$stress_pid"
    stress_status=$?
    stress_pid=
    { echo; echo "--- stress-ng (exit $stress_status) ---"; cat "$stress_out"; } >>"$note"
    rm -f "$stress_out"
    if [ -n "$load_problem" ]; then
      load_problem+=" (stress-ng exit $stress_status)"
    elif [ "$stress_status" -ne 0 ]; then
      load_problem="stress-ng exit $stress_status"
    fi
  fi
  { echo; echo "--- IRQ placement after ---"; irq_state; } >>"$note"
  if [ "$probe_status" -ne 0 ]; then
    { echo; echo "verdict: probe exit $probe_status"; } >>"$note"
    echo "trace_session: the probe exited $probe_status: session stopped (stop rule). See $note"
    outcome=stop
    return
  fi
  if [ -n "$tracer_problem" ]; then
    { echo; echo "verdict: tracer failure ($tracer_problem)"; } >>"$note"
    echo "trace_session: $tracer_problem: session stopped. See $note"
    outcome=stop
    return
  fi
  if [ -n "$load_problem" ]; then
    { echo; echo "verdict: load failure ($load_problem)"; } >>"$note"
    echo "trace_session: $load_problem: session stopped. See $note"
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
    { echo; echo "verdict: pass"; } >>"$note"
    outcome=pass
  elif [ "$check_status" -eq 1 ] && [ "$seen" = yes ] && [ -n "$failed" ]; then
    { echo; echo "verdict: fail ($failed)"; } >>"$note"
    outcome=fail
  else
    { echo; echo "verdict: analysis error (exit $check_status)"; } >>"$note"
    echo "trace_session: timing_summary.py failed (exit $check_status): session stopped. See $note"
    outcome=stop
  fi
}

for spec in "${run_list[@]}"; do
  IFS=: read -r p r c v <<<"$spec"
  for m in "${mode_list[@]}"; do
    current=$spec; current_mode=$m
    run_one "$p" "$r" "$c" "$v" "$m"
    case "$outcome" in
      pass) add_result "$spec" "$m" "PASS" ;;
      fail) add_result "$spec" "$m" "FAIL $failed" ;;
      skip) add_result "$spec" "$m" "skipped (attempted earlier)" ;;
      stop)
        add_result "$spec" "$m" "STOPPED the session (see the .run.txt)"
        current=
        finish
        print_table
        exit 1 ;;
    esac
    current=
  done
done
finish
print_table
