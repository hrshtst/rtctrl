#!/usr/bin/env bash
# Traced torque-off diagnostics of the bounded read
# (docs/hardware/control-rate.md, after the B2 and B3 runs).
#
# Run it from the rtctrl repo root, after a Release build, with every servo
# torque-off and the arm at rest and supported. x7_bus_timing refuses to
# measure otherwise, and it never writes Torque Enable or Drive Mode.
#
# Each listed run is made in each mode, by default untraced and then traced,
# under the same conditions, so the runs show the tracing overhead. A traced
# run records, around the probe:
#   - scheduler, interrupt, timer, workqueue and xHCI URB events with
#     trace-cmd on ftrace's mono clock (CLOCK_MONOTONIC, the clock of the
#     probe's monotonic_origin_ns); the session stops if the clock is not
#     mono;
#   - the adapter's usbmon traffic with tcpdump (usbmon stamps are
#     CLOCK_REALTIME, so the record also takes CLOCK_MONOTONIC and
#     CLOCK_REALTIME pairs before and after the run).
# A profiled run (--modes ...,profiled; rt-* conditions only) records what a
# traced run does, plus the probe threads' timer arming (hrtimer_start,
# hrtimer_cancel), sleep calls (clock_nanosleep and nanosleep entry and
# exit) and page faults, each filtered to the probe's comm, the workqueue's
# activations, and perf's samples of the real-time CPU with call chains on
# the mono clock (perf record -k mono -C <cpu> -F <hz> -g). Afterwards the
# worker's samples are written as text (perf script --tid). The traced mode's
# events stay as they were, so its runs compare with earlier ones.
# The tracers run on the CPUs outside the excluded set (the real-time CPU
# and its SMT siblings by default), and read nothing on the bus.
#
# The session first records the host state once: the fair server's
# settings (read with bpftrace, since the kernel's lockdown refuses the
# debugfs files), lockdown, boot line, topology and the adapter. Every run
# record repeats it, with the run's IRQ placement before and after, the
# xHCI interrupts around the probe, and a task and affinity snapshot
# halfway through the run. The session changes no scheduler setting; it
# loads the usbmon module if needed and unloads it at the end.
#
# The adapter experiment: --latency-steps L1,L2,... (for example 1,0,1)
# makes each listed run once per step, in order, each untraced and traced,
# with the adapter's USB latency timer at the step's value in ms, set
# through tools/latency_timer.py. Before the session sends anything it reads
# the driver's sysfs value, the adapter's own value and the port's
# low-latency flag (with which the driver writes 1 ms whatever is stored),
# and refuses if the flag is set or the two values differ. Before and after
# each run it reads all three again: a run whose adapter does not hold the
# step's value, with the flag clear, is voided and stops the session. The
# probe expects the step's value (--expect-latency-timer-ms). Once the
# first stepped run begins, the session owes the value it found back: on
# every exit of the script (its EXIT trap: the end, a stop, Ctrl-C,
# SIGTERM, SIGHUP or any other exit; only a SIGKILL or a crash skips it)
# it reads the adapter, verifies that the starting value is in effect, and
# sets it again if not. A cleanup that leaves another value fails the
# session: an otherwise successful one exits 3, and a stop or an
# interruption keeps its own status. The bounded read's
# profile check requires 1 ms (only the probe's option relaxes it), but the
# 100 Hz path does not read the timer: the restoration is what keeps the
# host's setting. Without --latency-steps the timer is left alone.
#
# Exit status: 0 the session completed; 1 a run stopped it (the stop rule);
# 2 it was refused before anything was sent; 3 it completed but its cleanup
# did not restore the adapter's latency timer; 130 it was interrupted.
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
#   <log>.csv.perf.data     perf samples of the real-time CPU (profiled runs)
#   <log>.csv.perf.txt      the worker's samples as text (profiled runs)
# with <log> = rate-trace-<profile>-<rate>hz-<condition>-bounded<variant
# tags>[-lt<ms>-s<step>]-<untraced|traced|profiled>-<date>-<attempt>. A run whose
# .csv or .run.txt already exists was attempted: it is skipped, never
# retried.
#
# Privileged steps (trace-cmd, tcpdump, bpftrace, modprobe) go through sudo;
# the probe runs as the invoking user. For tests, TRACE_SESSION_SUDO=none
# runs them directly and TRACE_SESSION_SUDO=<command> uses that command in
# sudo's place, TRACE_CMD, TCPDUMP, BPFTRACE, MODPROBE, STRESS_NG,
# LATENCY_TOOL, PERF and TRACEFS replace the tools and the tracefs path, and
# TRACE_SESSION_RTPRIO_LIMIT replaces the real-time priority limit checked
# (with a stand-in probe, which needs none).
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
LATENCY_TOOL=${LATENCY_TOOL:-tools/latency_timer.py}
PERF=${PERF:-perf}
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
latency_steps=
perf_hz=4999
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
  --modes LIST        untraced,traced (default), any of untraced, traced and
                      profiled (traced, plus timer arming, sleep calls, page
                      faults and perf samples of the real-time CPU; rt-*
                      conditions only)
  --perf-hz N         a profiled run's perf sampling frequency (default $perf_hz)
  --latency-steps L   the adapter experiment: each run once per step of the
                      adapter's USB latency timer in ms, 0 to 255 (for
                      example 1,0,1), restored on every exit (default: the
                      timer is left alone)
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
    --stress-dir|--load-tail|--latency-steps|--perf-hz)
      [ $# -ge 2 ] || die "$1 needs a value"
      case "$1" in
        --runs) runs=$2 ;;
        --modes) modes=$2 ;;
        --latency-steps) latency_steps=$2 ;;
        --perf-hz) perf_hz=$2 ;;
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
any_rt=0; any_loaded=0; any_traced=0; any_profiled=0; all_rt=1
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
  case "$c" in rt-*) any_rt=1 ;; *) all_rt=0 ;; esac
  case "$c" in *loaded*) any_loaded=1 ;; esac
done
for m in "${mode_list[@]}"; do
  case "$m" in
    untraced) ;;
    traced) any_traced=1 ;;
    profiled) any_traced=1; any_profiled=1 ;;
    *) die "unknown mode '$m'" ;;
  esac
done
# The latency steps, as ms:index (one empty step without the experiment).
lt_list=()
step_list=("")
if [ -n "$latency_steps" ]; then
  IFS=, read -r -a lt_list <<<"$latency_steps"
  [ ${#lt_list[@]} -gt 0 ] || die "--latency-steps is empty"
  step_list=()
  for i in "${!lt_list[@]}"; do
    l=${lt_list[$i]}
    is_uint "$l" && [ "$l" -le 255 ] || die "--latency-steps: '$l' is not a latency timer of 0 to 255 ms"
    step_list+=("$((10#$l)):$((i + 1))")
  done
fi
[[ $duration =~ ^[0-9]+$ ]] && [ "$duration" -gt 0 ] || die "--duration must be a positive integer"
is_uint "$perf_hz" && [ "$perf_hz" -ge 100 ] && [ "$perf_hz" -le 20000 ] ||
  die "--perf-hz must be 100 to 20000"
if [ "$any_profiled" -eq 1 ] && [ "$all_rt" -ne 1 ]; then
  die "the profiled mode samples the real-time CPU: use it with rt-* conditions only"
fi
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

# The port whose latency timer the experiment sets.
lt_tty=
if [ ${#lt_list[@]} -gt 0 ]; then
  lt_path=$port
  [ -n "$lt_path" ] || lt_path=$(sed -n 's/^port *= *"\(.*\)"/\1/p' "$CONFIG" | head -n 1)
  lt_tty=$(basename "$(readlink -f "$lt_path")")
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
if [ ${#lt_list[@]} -gt 0 ]; then
  [ -x "$LATENCY_TOOL" ] || problems+=("$LATENCY_TOOL is not executable")
  [ -n "$lt_tty" ] || problems+=("no port to set the latency timer of")
fi
command -v "$BPFTRACE" >/dev/null ||
  problems+=("$BPFTRACE is not installed: it reads the fair server's settings")
if [ "$any_profiled" -eq 1 ]; then
  command -v "$PERF" >/dev/null || problems+=("$PERF is not installed (linux-tools)")
fi
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
  lim=${TRACE_SESSION_RTPRIO_LIMIT:-$(ulimit -r)}
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
# A profiled run adds the probe threads' timer arming, sleep calls and page
# faults (filtered to the probe's comm: the kernel truncates it to 15
# characters) and the workqueue's activations.
probe_comm=$(basename "$PROBE" | cut -c 1-15)
profile_events=(
  timer:hrtimer_start timer:hrtimer_cancel
  syscalls:sys_enter_clock_nanosleep syscalls:sys_exit_clock_nanosleep
  syscalls:sys_enter_nanosleep syscalls:sys_exit_nanosleep
  exceptions:page_fault_user exceptions:page_fault_kernel
)
profile_args=("${trace_args[@]}")
for e in "${profile_events[@]}"; do profile_args+=(-e "$e" -f "comm == \"$probe_comm\""); done
profile_args+=(-e workqueue:workqueue_activate_work)
perf_args=(record -k mono -C "$rt_cpu" -F "$perf_hz" -g)
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

log_base() {  # profile rate condition variant mode [ms:step]
  local lt=
  variant_flags "$4"
  [ -n "${6:-}" ] && lt="-lt${6%%:*}-s${6##*:}"
  echo "rate-trace-$1-${2}hz-$3$vtag$lt-$5-$date_tag-$attempt"
}

probe_cmd() {  # profile rate condition variant mode [ms:step] -> cmd[]
  local base; base=$(log_base "$@")
  variant_flags "$4"
  cmd=("$PROBE")
  [ -n "$port" ] && cmd+=(--port "$port")
  cmd+=(--rate-hz "$2" --profile "$1" --duration "$duration"
        --log "$outdir/$base.csv")
  case "$3" in rt-*) cmd+=(--rt-priority "$rt_priority" --rt-cpu "$rt_cpu") ;; esac
  cmd+=("${vflags[@]}")
  [ -n "${6:-}" ] && cmd+=(--expect-latency-timer-ms "${6%%:*}")
  return 0
}

total=$(( ${#run_list[@]} * ${#step_list[@]} * ${#mode_list[@]} ))

if [ "$dry_run" -eq 1 ]; then
  echo "plan: $total runs (date $date_tag, attempt $attempt), tracers and -excl load on CPUs $other_cpus:"
  for spec in "${run_list[@]}"; do
    IFS=: read -r p r c v <<<"$spec"
    for step in "${step_list[@]}"; do
      for m in "${mode_list[@]}"; do
        probe_cmd "$p" "$r" "$c" "$v" "$m" "$step"
        stress_cmd "$c"
        printf '  [%s' "$m"
        [ "$m" = profiled ] && printf '; profiled'
        [ -n "$step" ] && printf '; latency timer %s ms (step %s)' "${step%%:*}" "${step##*:}"
        [ ${#scmd[@]} -gt 0 ] && printf '; load: %s' "${scmd[*]}"
        printf '] '; printf '%q ' "${cmd[@]}"; echo
      done
    done
  done
  if [ ${#lt_list[@]} -gt 0 ]; then
    echo "latency steps $latency_steps on $lt_tty through $LATENCY_TOOL: read before anything is sent, set before each step's runs, read after each run, and restored on every exit"
  fi
  if [ "$any_traced" -eq 1 ]; then
    printf 'traced runs add: stdbuf -oL -eL %s %s -o <log>.csv.trace.dat\n' "$TRACE_CMD" "${trace_args[*]}"
    if [ "$any_profiled" -eq 1 ]; then
      printf 'profiled runs record: stdbuf -oL -eL %s %s -o <log>.csv.trace.dat\n' "$TRACE_CMD" "${profile_args[*]}"
      printf '                      %s %s -o <log>.csv.perf.data\n' "$PERF" "${perf_args[*]}"
    fi
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
trace_job=; usbmon_job=; perf_job=
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
  if [ -n "$perf_job" ]; then alive "$perf_job" || gone+="${gone:+, }perf ended"; fi
  echo "$gone"
}
wait_file() {  # name job file seconds -> 0 once the recorder wrote its file
  local i
  for ((i = 0; i < $4 * 10; i++)); do
    [ -s "$3" ] && return 0
    alive "$2" || break
    sleep 0.1
  done
  [ -s "$3" ]
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
  if [ -n "$perf_job" ]; then stop_tracer perf "$perf_job"; perf_job=; fi
  if [ -n "$stress_pid" ]; then
    kill -TERM -- "-$stress_pid" 2>/dev/null
    wait "$stress_pid" 2>/dev/null
    stress_pid=
  fi
  own_outputs
}
# The adapter experiment's latency timer: the value found at the start,
# the value set last, whether the session owes the starting value back (from
# the first stepped run on, set or not), and the record of the last run,
# where the cleanup is written too.
lt_orig=; lt_now=; lt_owed=0; last_note=
lt_parse() {  # a latency_timer.py line -> lt_sysfs lt_device lt_low
  local line
  line=$(grep -m 1 '^tty ' <<<"$1")
  lt_sysfs=$(sed -n 's/.* sysfs \([^ ]*\) .*/\1/p' <<<"$line")
  lt_device=$(sed -n 's/.* device \([^ ]*\) .*/\1/p' <<<"$line")
  lt_low=$(sed -n 's/.* low_latency \([^ ]*\) .*/\1/p' <<<"$line")
}
# The cleanup: once owed, verify the starting value on the adapter and set it
# again if another is in effect. 0 when the adapter holds it (verified or
# restored), 1 when it does not; written to the session's output and to the
# last run record either way.
restore_latency() {
  local out
  [ "$lt_owed" -eq 1 ] || return 0
  out=$(priv "$LATENCY_TOOL" read "$lt_tty" 2>&1)
  lt_parse "$out"
  if [ "$lt_sysfs" = "$lt_orig" ] && [ "$lt_device" = "$lt_orig" ] && [ "$lt_low" = 0 ]; then
    lt_owed=0
    cleanup_note "verified at the starting $lt_orig ms" "$out"
    return 0
  fi
  if out=$(priv "$LATENCY_TOOL" set "$lt_tty" "$lt_orig" 2>&1); then
    lt_owed=0
    cleanup_note "restored to $lt_orig ms" "$out"
    return 0
  fi
  cleanup_note "NOT restored to $lt_orig ms" "$out"
  echo "  restore it by hand: echo $lt_orig | sudo tee /sys/bus/usb-serial/devices/$lt_tty/latency_timer" >&2
  return 1
}
cleanup_note() {  # what happened, the tool's output
  local line
  line=$(grep -m 1 '^tty ' <<<"$2")
  if [ "${1#NOT}" = "$1" ]; then
    echo "trace_session: cleanup: the adapter's latency timer is $1 (${line:-$2})"
  else
    echo "trace_session: CLEANUP FAILED: the adapter's latency timer is $1: $2" >&2
  fi
  if [ -n "$last_note" ] && [ -f "$last_note" ]; then
    printf '\n--- session end ---\ncleanup: latency timer %s: %s\n' "$1" "${line:-$2}" >>"$last_note"
  fi
}
# Every exit ends here. A second interrupt cannot cut the cleanup short,
# and a cleanup failure turns an otherwise successful exit into status 3.
on_exit() {
  local status=$?
  trap '' INT TERM HUP
  if ! restore_latency; then
    echo "trace_session: the session fails: its cleanup did not restore the adapter's latency timer" >&2
    [ "$status" -eq 0 ] && exit 3
  fi
  return 0
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
trap on_interrupt INT TERM HUP
trap on_exit EXIT

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
# The experiment's starting point: the value to restore, and a refusal if
# the driver would not apply the steps.
if [ ${#lt_list[@]} -gt 0 ]; then
  if ! lt_line=$(priv "$LATENCY_TOOL" read "$lt_tty" 2>&1); then
    finish
    die "the adapter's latency timer on $lt_tty was not read ($lt_line): nothing was sent"
  fi
  lt_parse "$lt_line"
  if [ "$lt_low" != 0 ]; then
    finish
    die "the port's low-latency flag is set ($lt_line): the driver would write 1 ms whatever the step; nothing was sent"
  fi
  if [ "$lt_sysfs" != "$lt_device" ]; then
    finish
    die "the adapter's latency timer reads $lt_device but sysfs $lt_sysfs ($lt_line): nothing was sent"
  fi
  lt_orig=$lt_sysfs; lt_now=$lt_orig
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
  echo "unbound workqueues: cpumask $(cat /sys/devices/virtual/workqueue/cpumask 2>&1), affinity scope $(cat /sys/module/workqueue/parameters/default_affinity_scope 2>&1)"
  if [ -n "$usb_device" ]; then
    echo "adapter: $usb_device, $(cat "$usb_device/idVendor" 2>&1):$(cat "$usb_device/idProduct" 2>&1), $(cat "$usb_device/speed" 2>&1) Mbps, bus $usb_bus device $usb_devnum"
  fi
  for lt in /sys/bus/usb-serial/devices/*/latency_timer; do
    [ -f "$lt" ] && echo "latency_timer $(basename "$(dirname "$lt")"): $(cat "$lt")"
  done
  if [ ${#lt_list[@]} -gt 0 ]; then
    echo "latency steps: $latency_steps; at the session's start: $(grep -m 1 '^tty ' <<<"$lt_line")"
  fi
  if [ "$usbmon" -eq 1 ] && [ "$any_traced" -eq 1 ]; then
    echo "usbmon: $([ "$usbmon_loaded_here" -eq 1 ] && echo "loaded by this session" || echo "already loaded")"
  else
    echo "usbmon: not used"
  fi
  echo "tracers: $("$TRACE_CMD" -h 2>&1 | grep -m 1 -i version); $("$TCPDUMP" --version 2>&1 | head -n 1); $("$BPFTRACE" --version 2>&1 | head -n 1)"
} >"$tmp/session.txt"

xhci_interrupts() {  # every CPU's count of the xHCI interrupt lines
  awk '/xhci/ {for (i = 2; i <= NF && $i ~ /^[0-9]+$/; i++) s += $i} END {print s + 0}' /proc/interrupts
}
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
  local profile=$1 rate=$2 cond=$3 variant=$4 mode=$5 step=${6:-} base csv note
  local loaded=0 probe_status check_status seen stress_out stress_status snap_pid
  local dat pcap lt_ms=${step%%:*} lt_out lt_status irq_before irq_after t_before t_after
  local traced=0 profiled=0 perfdat perftxt targs
  case "$mode" in traced) traced=1 ;; profiled) traced=1; profiled=1 ;; esac
  base=$(log_base "$profile" "$rate" "$cond" "$variant" "$mode" "$step")
  csv="$outdir/$base.csv"
  note="$csv.run.txt"
  dat="$csv.trace.dat"
  pcap="$csv.usbmon.pcap"
  perfdat="$csv.perf.data"
  perftxt="$csv.perf.txt"
  targs=("${trace_args[@]}")
  [ "$profiled" -eq 1 ] && targs=("${profile_args[@]}")
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
  probe_cmd "$profile" "$rate" "$cond" "$variant" "$mode" "$step"
  stress_cmd "$cond"
  {
    echo "rtctrl traced torque-off run"
    echo "started: $(date -Is)"
    echo "commit: $commit"
    echo "probe binary: $(stat -c %y "$PROBE")"
    echo "condition: $cond"
    echo "variant: $variant (bounded read path: the probe's 35 ms budget and equal guard)"
    echo "mode: $mode"
    if [ -n "$step" ]; then
      echo "latency step: $lt_ms ms (step ${step##*:} of ${#step_list[@]}: $latency_steps)"
    fi
    if [ "$loaded" -eq 1 ]; then echo "load: ${scmd[*]}"; else echo "load: none"; fi
    case "$cond" in
      rt-*) echo "scheduling: --rt-priority $rt_priority --rt-cpu $rt_cpu (rtprio limit ${TRACE_SESSION_RTPRIO_LIMIT:-$(ulimit -r)})" ;;
      *) echo "scheduling: default" ;;
    esac
    echo "runnable before start: $quiet (this script included)"
    echo "uptime:$(uptime)"
    printf 'command:'; printf ' %q' "${cmd[@]}"; echo
    if [ "$traced" -eq 1 ]; then
      printf 'trace: stdbuf -oL -eL %s' "$TRACE_CMD"; printf ' %q' "${targs[@]}"; printf ' -o %q\n' "$dat"
      if [ "$profiled" -eq 1 ]; then
        printf 'perf: %s' "$PERF"; printf ' %q' "${perf_args[@]}"; printf ' -o %q\n' "$perfdat"
      fi
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
  last_note=$note
  # The step's latency timer, set if it changes, and checked in effect. From
  # here the session owes the starting value back, set or not: the run opens
  # the port, and a run can change the adapter's value.
  if [ -n "$step" ]; then
    lt_owed=1
    if [ "$lt_ms" != "$lt_now" ]; then
      lt_out=$(priv "$LATENCY_TOOL" set "$lt_tty" "$lt_ms" 2>&1); lt_status=$?
      lt_now=$lt_ms
    else
      lt_out=$(priv "$LATENCY_TOOL" read "$lt_tty" 2>&1); lt_status=$?
    fi
    echo "latency timer before the run (requested $lt_ms ms): $lt_out" >>"$note"
    lt_parse "$lt_out"
    if [ "$lt_status" -ne 0 ] || [ "$lt_sysfs" != "$lt_ms" ] || [ "$lt_device" != "$lt_ms" ] ||
       [ "$lt_low" != 0 ]; then
      echo "trace_session: the adapter does not hold the requested $lt_ms ms ($lt_out): session stopped" | tee -a "$note"
      echo "verdict: not run (latency timer)" >>"$note"
      outcome=stop
      return
    fi
  fi
  if [ "$traced" -eq 1 ]; then
    run_outputs=("$dat")
    if [ "$usbmon" -eq 1 ]; then run_outputs+=("$pcap"); fi
    if [ "$profiled" -eq 1 ]; then run_outputs+=("$perfdat"); fi
    # On a pipe, the C library would hold trace-cmd's "Hit Ctrl^C to stop
    # recording" until it exits; stdbuf (which execs it, keeping the PID)
    # makes its output line-buffered. tcpdump reports on stderr, which is
    # unbuffered.
    start_tracer trace stdbuf -oL -eL "$TRACE_CMD" "${targs[@]}" -o "$dat"
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
    if [ "$profiled" -eq 1 ]; then
      # perf says nothing until it stops; it is recording once its file has
      # its header.
      start_tracer perf "$PERF" "${perf_args[@]}" -o "$perfdat"
      if ! wait_file perf "$perf_job" "$perfdat" 30; then
        stop_all
        { echo "--- perf did not start ---"; cat "$tmp/perf.out"; } | tee -a "$note"
        echo "verdict: not run (tracer)" >>"$note"
        outcome=stop
        return
      fi
      sleep 1
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
  if [ "$traced" -eq 1 ]; then
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
  irq_before=$(xhci_interrupts); t_before=$(date +%s.%N)
  "${cmd[@]}" 2>&1 | tee -a "$note"
  probe_status=${PIPESTATUS[0]}
  irq_after=$(xhci_interrupts); t_after=$(date +%s.%N)
  # The load and the recorders must have lasted the whole run: checked as
  # the probe ends, before anything is stopped.
  local load_problem='' tracer_problem=''
  if [ "$loaded" -eq 1 ] && ! alive "$stress_pid"; then
    load_problem="the load ended before the probe did"
  fi
  if [ "$traced" -eq 1 ]; then
    tracer_problem=$(recorders_ended)
    [ -n "$tracer_problem" ] && tracer_problem+=" before the probe did"
  fi
  wait "$snap_pid" 2>/dev/null
  if [ "$traced" -eq 1 ]; then
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
    if [ "$profiled" -eq 1 ]; then
      local perf_status script_status wtid perf_clean=0
      stop_tracer perf "$perf_job"; perf_status=$?; perf_job=
      # perf record finishes its file, then dies of the SIGINT that stopped
      # it: a clean stop is that status (or 0) with its closing report.
      if { [ "$perf_status" -eq 0 ] || [ "$perf_status" -eq 130 ]; } && [ -s "$perfdat" ] &&
         grep -q 'Captured and wrote' "$tmp/perf.out"; then
        perf_clean=1
      else
        tracer_problem+="${tracer_problem:+, }perf exit $perf_status"
      fi
      { echo; echo "--- perf (exit $perf_status, $([ "$perf_clean" -eq 1 ] && echo "a clean stop" || echo "NOT a clean stop")) ---"
        cat "$tmp/perf.out"; } >>"$note"
      # The worker's samples as text, symbols resolved as root.
      wtid=$(sed -n 's/^worker_tid = //p' "$csv.summary.toml" 2>/dev/null)
      if [ -n "$wtid" ] && [ "$perf_clean" -eq 1 ]; then
        priv "$PERF" script -i "$perfdat" --ns -G -F comm,tid,cpu,time,ip,sym,dso --tid "$wtid" \
          >"$perftxt" 2>"$tmp/perf-script.err"
        script_status=$?
        echo "perf script --tid $wtid (exit $script_status): $(wc -l <"$perftxt") samples" >>"$note"
        [ "$script_status" -eq 0 ] && [ -s "$perftxt" ] ||
          tracer_problem+="${tracer_problem:+, }perf script exit $script_status"
      elif [ "$probe_status" -eq 0 ]; then
        tracer_problem+="${tracer_problem:+, }no worker thread for perf script"
      fi
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
  awk -v n=$((irq_after - irq_before)) -v a="$t_before" -v b="$t_after" 'BEGIN {
    printf "xhci interrupts around the probe: %d in %.3f s (%.0f per s; every device on the controller)\n", n, b - a, n / (b - a)}' >>"$note"
  local latency_problem=''
  if [ -n "$step" ]; then
    lt_out=$(priv "$LATENCY_TOOL" read "$lt_tty" 2>&1); lt_status=$?
    echo "latency timer after the run: $lt_out" >>"$note"
    lt_parse "$lt_out"
    if [ "$lt_status" -ne 0 ] || [ "$lt_sysfs" != "$lt_ms" ] || [ "$lt_device" != "$lt_ms" ] ||
       [ "$lt_low" != 0 ]; then
      latency_problem="the adapter's latency timer read $(grep -m 1 '^tty ' <<<"$lt_out") after the run, not $lt_ms ms"
    fi
  fi
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
  if [ -n "$latency_problem" ]; then
    { echo; echo "verdict: latency timer failure ($latency_problem)"; } >>"$note"
    echo "trace_session: $latency_problem: session stopped. See $note"
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
  for step in "${step_list[@]}"; do
    label=$spec
    [ -n "$step" ] && label="$spec@${step%%:*}ms#${step##*:}"
    for m in "${mode_list[@]}"; do
      current=$label; current_mode=$m
      run_one "$p" "$r" "$c" "$v" "$m" "$step"
      case "$outcome" in
        pass) add_result "$label" "$m" "PASS" ;;
        fail) add_result "$label" "$m" "FAIL $failed" ;;
        skip) add_result "$label" "$m" "skipped (attempted earlier)" ;;
        stop)
          add_result "$label" "$m" "STOPPED the session (see the .run.txt)"
          current=
          finish
          print_table
          exit 1 ;;
      esac
      current=
    done
  done
done
finish
print_table
