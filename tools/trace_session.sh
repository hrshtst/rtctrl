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
# activations, and perf's samples of the real-time CPU on the mono clock,
# with call chains unwound from a copy of each sample's user stack (perf
# record -k mono -C <cpu> -F <hz> --call-graph dwarf,<bytes>: the Release
# build keeps no frame pointers). Afterwards, as root, the worker's samples
# are written as text twice: their sampled instruction pointers
# (<log>.csv.perf.txt) and their call chains (<log>.csv.perf.stacks.txt), and
# perf's sample, lost and throttle counts go to the record. The traced mode's
# events stay as they were, so its runs compare with earlier ones.
# The tracers run on the CPUs outside the excluded set (the real-time CPU
# and its SMT siblings by default), and read nothing on the bus.
#
# A timer run also records the fair server's probe events (Stage 1;
# tools/fair_server.py, tools/fair_server_probes.sh): kprobe events in the
# session's own group (rtctrl_fs) on the deadline class's pick, the fair
# server's pick and its start, stop and timer, with the server's state read
# from offsets in the kernel's BTF. The session defines them once, before
# anything is sent (refusing a group left by an earlier session), records
# the layout, the definitions and each event's format, filters them to the
# real-time CPU in every timer run (the picks by the CPU they run on, the
# server's calls by the server's CPU), records their hit and miss counts
# right before and after the probe, and removes them on every exit; a
# failed removal fails the session (status 3), and a group whose definition
# had an unknown outcome is left in place and reported, never removed on a
# guess.
#
# The probes' controls (--fs-controls CPU; the rehearsal only, over the
# emulator): before the runs, tools/fair_server_control.py runs a positive
# test (a SCHED_FIFO busy loop with a self-imposed deadline and a normal task
# on the spare CPU, its supervisor on another CPU) and negative ones (the
# normal task alone, then the idle CPU), traced with the same probe
# definitions filtered to that CPU and the tracers off it. Its record holds
# the probes' counts, the buffers' losses and the analysis
# (tools/fair_server.py controls); a failed control, a loss or a tracer
# failure stops the session, and an inconclusive negative control is
# reported. Files: fs-controls-<date>-<attempt>.{run.txt,trace.dat,controls.txt}.
# Controls are never reused: if any of those files exists (a passed, failed,
# partial or interrupted attempt), the session refuses before anything is
# sent, and a fresh --attempt runs them again.
#
# The observer placement comparison (--placements open,confined with
# --observers PID,...): each listed run is made once per placement, its log
# name carrying the label (-obs-open, -obs-confined). For a confined run,
# tools/observer_affinity.py moves every thread of the operator's named
# process trees off the excluded CPUs right after the record is opened
# (writing <log>.csv.observers.jsonl, every thread's identity, start time and
# mask, before any change), the probe starts with its mask outside them (only
# its worker pins itself to the real-time CPU), and the observers are
# restored right after the probe, and on every exit (a failed restoration is
# status 3). The run's halfway task snapshot is checked: an observer thread
# or a probe helper allowed on an excluded CPU stops the session as a
# placement failure, as does a thread confine cannot move. The session's own
# process tree is exempt, and kernel workers are outside this mechanism. An
# open run leaves the observers as they are.
#
# Recorder staging (every run; tools/trace_stage.py, approved 2026-11-09):
# each run's capture streams (the probe's log and summary, the trace with
# trace-cmd's per-CPU temporary files (--temp), usbmon, the SMI counts, the
# TSC instance and sampler pairs, perf, the exerciser's log and the
# snapshots) are written to a private, size-bounded noswap tmpfs mounted for
# the run under the stage root (/run/rtctrl-stage), never to the output
# directory, so no recorder writes to the output filesystem during the run;
# the run record and the observer record stay in the output directory, so
# every recovery command names a file that exists. Before the mount, the
# run's budget (by mode: the raw trace, its assembly and the other enabled
# streams, plus the kernel buffers and a headroom; provisional constants)
# must fit in MemAvailable; the mount is verified (tmpfs, noswap, its size,
# private) and marked as the session's; both trace arrays'
# buffer_total_size_kb are read back against the budget before the probe;
# and a guard on the tracer CPUs stops the recorders when the stage's free
# space or MemAvailable falls below its bounds (a guard, not a
# reservation). Once every writer has stopped, the stage is inventoried,
# published to the output directory (each copy hashed as read back, linked
# without overwriting and fsynced; the ledger <log>.csv.staging.sha256 last)
# and only then released (emptied, unmounted, removed); failed captures are
# published the same way, and the capture is judged from the published
# copies. A publication or release that fails keeps the stage, names it with
# the retry, and fails the session (status 3, an interruption keeping 130),
# the capture's own verdict recorded apart. A stage left by an earlier session
# refuses the session; a session never touches another's stage. Read-only,
# system-wide memory and writeback counters (tools/vm_counters.py) bracket
# every probe, beside the idle counters. These are diagnostic runs: they do
# not qualify the production logging to disk.
#
# The session first records the host state once: the fair server's
# settings (read with bpftrace, since the kernel's lockdown refuses the
# debugfs files), lockdown, boot line, topology and the adapter. Every run
# record repeats it, with the run's IRQ placement before and after, the
# xHCI interrupts around the probe, a task and affinity snapshot halfway
# through the run, and read-only snapshots of the excluded CPUs' idle-state
# counters (tools/idle_counters.py: identities, raw values and deltas, with
# their monotonic sample times) right before and after the probe, untraced
# runs included. The session changes no scheduler setting; it loads the
# usbmon module if needed and unloads it at the end.
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
# failed (the adapter's latency timer, the trace instance, the observers, the
# probes, or a stage not released), or, whatever the runs' own outcomes, a
# run's staged evidence was not published and verified; 130 it was
# interrupted (kept whatever the cleanup).
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
# tracer that does not start, stop cleanly or last the run, a trace buffer
# that lost events or whose losses cannot be counted (the capture is then
# invalid), a load that does not last the run or succeed, or an analysis
# error stops the session (the stop rule).
#
# Each run leaves gitignored files in the output directory:
#   <log>.csv               per-cycle log (probe)
#   <log>.csv.summary.toml  run summary (probe)
#   <log>.csv.run.txt       command, host state, probe output, verdicts
#   <log>.csv.tasks.txt     tasks, CPUs and affinities halfway through
#   <log>.csv.trace.dat     trace-cmd record (traced runs)
#   <log>.csv.usbmon.pcap   usbmon capture of the adapter (traced runs)
#   <log>.csv.staging.sha256  the published files' SHA-256 (sha256sum -c)
#   <log>.csv.staging.inventory.json  the staged files, taken once the
#                           writers stopped (written before the copies)
#   <log>.csv.perf.data     perf samples of the real-time CPU (profiled runs)
#   <log>.csv.perf.txt      the worker's sampled instruction pointers (profiled)
#   <log>.csv.perf.stacks.txt  the worker's samples' call chains (profiled)
# with <log> = rate-trace-<profile>-<rate>hz-<condition>-bounded<variant
# tags>[-lt<ms>-s<step>]-<untraced|traced|profiled>-<date>-<attempt>. A run whose
# .csv or .run.txt already exists was attempted: it is skipped, never
# retried. Since the output directory forgets the runs moved to the archive,
# the session refuses to start if any planned <log>.csv already appears in
# the archive manifest (docs/records/data-archive.md, or --manifest), bare or
# directory-qualified, itself or as a sidecar's prefix: a free --attempt
# avoids the name.
#
# Privileged steps (trace-cmd, tcpdump, bpftrace, modprobe) go through sudo;
# the probe runs as the invoking user. For tests, TRACE_SESSION_SUDO=none
# runs them directly and TRACE_SESSION_SUDO=<command> uses that command in
# sudo's place, TRACE_CMD, TCPDUMP, BPFTRACE, MODPROBE, STRESS_NG,
# LATENCY_TOOL, PERF, FS_PROBE_TOOL, OBS_TOOL, MOUNT, FINDMNT, UMOUNT and
# TRACEFS replace the tools and the tracefs path,
# TRACE_SESSION_STAGE_ROOT, TRACE_SESSION_MEMINFO and TRACE_SESSION_VMSTAT
# the stage root and the /proc files read, and
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
TSC_INSTANCE_TOOL=${TSC_INSTANCE_TOOL:-tools/tsc_instance.sh}
TSC_SAMPLER=${TSC_SAMPLER:-./build/apps/tsc_mono_pairs}
EXERCISER=tools/timer_exerciser.py
IDLE_TOOL=tools/idle_counters.py
FS_TOOL=tools/fair_server.py
FS_PROBE_TOOL=${FS_PROBE_TOOL:-tools/fair_server_probes.sh}
FS_GROUP=rtctrl_fs
FS_CONTROL_TOOL=tools/fair_server_control.py
# The controls' phase and settle seconds (tests shorten them).
FS_CONTROL_PHASE_S=${FS_CONTROL_PHASE_S:-3}
FS_CONTROL_SETTLE_S=${FS_CONTROL_SETTLE_S:-2}
fs_controls_cpu=
OBS_TOOL=${OBS_TOOL:-tools/observer_affinity.py}  # tests replace it
STAGE_TOOL=tools/trace_stage.py
VM_TOOL=tools/vm_counters.py
MOUNT=${MOUNT:-mount}
STAGE_ROOT=${TRACE_SESSION_STAGE_ROOT:-/run/rtctrl-stage}
MEMINFO=${TRACE_SESSION_MEMINFO:-/proc/meminfo}
VMSTAT=${TRACE_SESSION_VMSTAT:-/proc/vmstat}
placements=; observers=
obs_owed=0; obs_record=  # a confined run's placement record, owed a restoration while obs_owed is 1
placement=  # the placement of the run being named or made
fs_defs=; fs_layout=  # the probes' definitions and layout (timer mode), read before anything is sent
CPU_ROOT=${TRACE_SESSION_CPU_ROOT:-/sys/devices/system/cpu}  # the idle counters' sysfs (tests replace it)
TSC_INSTANCE=rtctrl_tsc
# Unrelated batch workloads that refuse the session (tools/batch_workloads.sh:
# process names, as the kernel truncates them to 15 characters, less the
# positively identified upgrade shutdown helper; TRACE_SESSION_BATCH_LIST
# and TRACE_SESSION_BATCH_EXEMPT replace them for tests). The session never
# stops a process; tools/quiet_host.sh is the operator's reversible stop of
# the resident ones, and its record goes into every run record.
# shellcheck source=tools/batch_workloads.sh
. "$(dirname "$0")/batch_workloads.sh"
batch_list=$BATCH_LIST
case "${TRACE_SESSION_SUDO:-sudo}" in
  none) sudo_cmd=() ;;
  *) sudo_cmd=("${TRACE_SESSION_SUDO:-sudo}") ;;
esac
# The staging tool unmounts and removes a stage through the same sudo.
export TRACE_STAGE_SUDO="${TRACE_SESSION_SUDO:-sudo}"

# The targeted set decided on 2026-11-05: Fast Sync Read with the
# motion-only read at 500 and 1000 Hz, quiet and loaded, with and without
# the load on the probe's core; and the cbp 1000 Hz idle anomaly.
runs=current:500:rt-idle:fast-motion,current:500:rt-loaded:fast-motion
runs+=,current:500:rt-loaded-excl:fast-motion,current:1000:rt-idle:fast-motion
runs+=,current:1000:rt-loaded:fast-motion,current:1000:rt-loaded-excl:fast-motion
runs+=,cbp:1000:rt-idle:fast
modes=untraced,traced
latency_steps=
perf_hz=999
perf_stack=4096
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
manifest=docs/records/data-archive.md
outdir=.
max_runnable=3
load_tail=15
stress_dir="$HOME/.cache/rtctrl-stress"
dry_run=0
assume_yes=0
exerciser=0

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
  --modes LIST        untraced,traced (default), any of untraced, traced,
                      profiled (traced, plus timer arming, sleep calls, page
                      faults and perf samples of the real-time CPU; rt-*
                      conditions only) and timer (traced, plus every timer's
                      arming and cancellation, the real-time CPU's TSC
                      deadline writes, idle transitions, NMIs, system calls
                      and page faults, the worker's sleep calls, process
                      lifecycle, a TSC-clock trace instance of its timer
                      interrupts, bracketed TSC and monotonic pairs, its
                      SMI count and the fair server's probe events; rt-*
                      only)
  --exerciser         the timer mode's rehearsal: the timer exerciser runs on
                      the real-time CPU during each timer run (refused
                      without --port and on a serial device)
  --placements L      the observer placement comparison: each run once per
                      placement, open and/or confined, its log name labelled
                      (default: no placement, no label)
  --observers PIDS    the observer applications' processes (explicit PIDs,
                      their trees confined in confined runs)
  --fs-controls C     the timer mode's rehearsal: the fair server's probes'
                      positive and negative controls on spare CPU C before
                      the runs (refused without --port, on a serial device,
                      and on an excluded CPU)
  --perf-hz N         a profiled run's perf sampling frequency (default $perf_hz)
  --perf-stack B      bytes of user stack perf copies per sample for its DWARF
                      unwinding, a multiple of 8 (default $perf_stack)
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
  --manifest FILE     the archive manifest the log names must not repeat
                      (default $manifest)
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
    --port|--usb-device|--buffer-kb|--date|--attempt|--manifest|--outdir|--max-runnable|\
    --stress-dir|--load-tail|--latency-steps|--perf-hz|--perf-stack|--fs-controls|--placements|--observers)
      [ $# -ge 2 ] || die "$1 needs a value"
      case "$1" in
        --runs) runs=$2 ;;
        --modes) modes=$2 ;;
        --latency-steps) latency_steps=$2 ;;
        --perf-hz) perf_hz=$2 ;;
        --perf-stack) perf_stack=$2 ;;
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
        --manifest) manifest=$2 ;;
        --outdir) outdir=$2 ;;
        --max-runnable) max_runnable=$2 ;;
        --stress-dir) stress_dir=$2 ;;
        --load-tail) load_tail=$2 ;;
        --fs-controls) fs_controls_cpu=$2 ;;
        --placements) placements=$2 ;;
        --observers) observers=$2 ;;
      esac
      shift 2 ;;
    --no-usbmon) usbmon=0; shift ;;
    --exerciser) exerciser=1; shift ;;
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
any_rt=0; any_loaded=0; any_traced=0; any_profiled=0; any_timer=0; all_rt=1; max_rate=0
for spec in "${run_list[@]}"; do
  IFS=: read -r p r c v extra <<<"$spec"
  [ -z "${extra:-}" ] && [ -n "${v:-}" ] || die "run '$spec' is not profile:rate:condition:variant"
  case "$p" in current|position|cbp) ;; *) die "run '$spec': unknown profile '$p'" ;; esac
  is_uint "$r" && [ "$r" -gt 0 ] || die "run '$spec': the rate must be a positive integer"
  [ "$r" -gt "$max_rate" ] && max_rate=$r
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
    timer) any_traced=1; any_timer=1 ;;
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
is_uint "$perf_stack" && [ "$perf_stack" -ge 1024 ] && [ "$perf_stack" -le 65528 ] &&
  [ $((perf_stack % 8)) -eq 0 ] || die "--perf-stack must be a multiple of 8 from 1024 to 65528"
if [ "$any_profiled" -eq 1 ] && [ "$all_rt" -ne 1 ]; then
  die "the profiled mode samples the real-time CPU: use it with rt-* conditions only"
fi
if [ "$any_timer" -eq 1 ] && [ "$all_rt" -ne 1 ]; then
  die "the timer mode traces the real-time CPU: use it with rt-* conditions only"
fi
# The exerciser loads the real-time CPU: a rehearsal over the emulator only.
if [ "$exerciser" -eq 1 ]; then
  [ "$any_timer" -eq 1 ] || die "--exerciser needs the timer mode"
  [ -n "$port" ] || die "--exerciser is for the rehearsal: pass the emulator's --port"
  ex_tty=$(basename "$(readlink -f "$port")")
  [ ! -e "/sys/class/tty/$ex_tty/device" ] || die "--exerciser is for the rehearsal: $port is a serial device ($ex_tty)"
fi
# The placements, as labels (one empty placement without the comparison).
pl_list=("")
if [ -n "$placements" ]; then
  IFS=, read -r -a pl_list <<<"$placements"
  [ ${#pl_list[@]} -gt 0 ] || die "--placements is empty"
  for pl in "${pl_list[@]}"; do
    case "$pl" in open|confined) ;; *) die "--placements: unknown placement '$pl' (open or confined)" ;; esac
  done
fi
if [ -n "$observers" ]; then
  [[ $observers =~ ^[0-9]+(,[0-9]+)*$ ]] || die "--observers must be a list of PIDs"
  case ",$placements," in *,confined,*) ;; *) die "--observers needs --placements with confined" ;; esac
fi
case ",$placements," in
  *,confined,*) [ -n "$observers" ] || die "a confined placement needs --observers (the PIDs to confine)" ;;
esac
# The probes' controls load a spare CPU: a rehearsal over the emulator only.
if [ -n "$fs_controls_cpu" ]; then
  [ "$any_timer" -eq 1 ] || die "--fs-controls needs the timer mode (its probes)"
  is_uint "$fs_controls_cpu" || die "--fs-controls must be a CPU number"
  [ -n "$port" ] || die "--fs-controls is for the rehearsal: pass the emulator's --port"
  fc_tty=$(basename "$(readlink -f "$port")")
  [ ! -e "/sys/class/tty/$fc_tty/device" ] || die "--fs-controls is for the rehearsal: $port is a serial device ($fc_tty)"
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
# The kernel sizes a trace array's buffers for every possible CPU.
possible_cpus=$(expand_cpus "$(cat /sys/devices/system/cpu/possible)" | wc -w) ||
  die "cannot read the possible CPUs"
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
idle_cpus=$(tr ' ' , <<<"$excluded")  # the idle counters' CPUs: the excluded set
tracer_cpus=$other_cpus  # where start_tracer runs a tracer (the controls keep theirs off their CPU)
ctl_sup=; ctl_tracer_cpus=
if [ -n "$fs_controls_cpu" ]; then
  case " $online " in *" $fs_controls_cpu "*) ;; *) die "--fs-controls $fs_controls_cpu is not an online CPU" ;; esac
  case " $excluded " in
    *" $fs_controls_cpu "*) die "--fs-controls $fs_controls_cpu is in the excluded set ($exclude_cpus): pick a spare CPU" ;;
  esac
  ctl_cpus=()
  for c in "${others[@]}"; do [ "$c" != "$fs_controls_cpu" ] && ctl_cpus+=("$c"); done
  [ ${#ctl_cpus[@]} -gt 0 ] || die "--fs-controls $fs_controls_cpu leaves no CPU for the supervisor and the tracers"
  ctl_sup=${ctl_cpus[0]}
  ctl_tracer_cpus=$(IFS=,; echo "${ctl_cpus[*]}")
fi
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
[ -f "$IDLE_TOOL" ] || problems+=("$IDLE_TOOL is missing (run from the repo root)")
[ -f "$STAGE_TOOL" ] && [ -f "$VM_TOOL" ] || problems+=("$STAGE_TOOL or $VM_TOOL is missing (run from the repo root)")
for c in "$MOUNT" "${FINDMNT:-findmnt}" "${UMOUNT:-umount}"; do
  command -v "$c" >/dev/null || problems+=("$c is not installed (util-linux): the runs' staging needs it")
done
command -v timeout >/dev/null || problems+=("timeout is not installed (coreutils)")
# A stage left by an earlier session may hold the only copy of a run's
# evidence: it refuses the session, which never touches it.
if [ -f "$STAGE_TOOL" ] && ! left=$(python3 "$STAGE_TOOL" leftovers --root "$STAGE_ROOT" 2>&1); then
  problems+=("a stage remains under $STAGE_ROOT from an earlier session (it may hold the only copy of a run's evidence; this session never touches it):
$(sed 's/^/      /' <<<"$left")
      publish and release it (python3 $STAGE_TOOL publish --stage DIR --outdir OUTDIR, then release with the same arguments), or, if it holds no data, python3 $STAGE_TOOL abort --stage DIR")
fi
# The probes' controls run afresh in each session: an earlier attempt's
# files, whatever their verdict or completeness, refuse it.
if [ -n "$fs_controls_cpu" ]; then
  for f in "$outdir/fs-controls-$date_tag-$attempt".{run.txt,trace.dat,controls.txt}; do
    [ -e "$f" ] && problems+=("the fair-server controls of this attempt were attempted earlier ($f): controls are never reused; pass a free --attempt")
  done
fi
if [ -n "$observers" ]; then
  [ -f "$OBS_TOOL" ] || problems+=("$OBS_TOOL is missing (run from the repo root)")
  IFS=, read -r -a obs_pids <<<"$observers"
  for op in "${obs_pids[@]}"; do
    [ -d "/proc/$op" ] || problems+=("--observers: no process $op")
  done
fi
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
if [ "$any_timer" -eq 1 ]; then
  command -v "$PERF" >/dev/null || problems+=("$PERF is not installed (linux-tools): it counts SMIs")
  [ -x "$TSC_INSTANCE_TOOL" ] || problems+=("$TSC_INSTANCE_TOOL is not executable")
  [ -x "$TSC_SAMPLER" ] || problems+=("$TSC_SAMPLER is not built (cmake --build build --target tsc_mono_pairs)")
  [ -f "$EXERCISER" ] || [ "$exerciser" -eq 0 ] || problems+=("$EXERCISER is missing")
  [ -e /sys/bus/event_source/devices/msr/events/smi ] || problems+=("no msr/smi/ perf event: the SMI count needs it")
  [ -x "$FS_PROBE_TOOL" ] || problems+=("$FS_PROBE_TOOL is not executable")
  # The fair server's probes: their offsets from the kernel's BTF, read as
  # the invoking user (bpftool; FAIR_SERVER_BTF_RAW replaces the dump in tests).
  if ! fs_layout=$(python3 "$FS_TOOL" layout 2>&1) ||
     ! fs_defs=$(python3 "$FS_TOOL" definitions --group "$FS_GROUP" 2>&1); then
    problems+=("the fair server's probe layout was not read: ${fs_layout:-}${fs_defs:+ $fs_defs}")
  fi
fi
# Known unrelated batch workloads refuse the session; it never stops one.
running_batch=$(batch_running "$batch_list")
[ -z "$running_batch" ] || problems+=("unrelated batch workloads are running ($running_batch): let them finish or stop them yourself (the session never stops a process)$(batch_hint "$running_batch")")
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
sleep_events=(
  syscalls:sys_enter_clock_nanosleep syscalls:sys_exit_clock_nanosleep
  syscalls:sys_enter_nanosleep syscalls:sys_exit_nanosleep
)
profile_events=(
  timer:hrtimer_start timer:hrtimer_cancel "${sleep_events[@]}"
  exceptions:page_fault_user exceptions:page_fault_kernel
)
profile_args=("${trace_args[@]}")
for e in "${profile_events[@]}"; do profile_args+=(-e "$e" -f "comm == \"$probe_comm\""); done
profile_args+=(-e workqueue:workqueue_activate_work)
perf_args=(record -k mono -C "$rt_cpu" -F "$perf_hz" --call-graph "dwarf,$perf_stack")
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

log_base() {  # profile rate condition variant mode [ms:step]; the placement from $placement
  local lt= pl=
  variant_flags "$4"
  [ -n "${6:-}" ] && lt="-lt${6%%:*}-s${6##*:}"
  [ -n "$placement" ] && pl="-obs-$placement"
  echo "rate-trace-$1-${2}hz-$3$vtag$lt$pl-$5-$date_tag-$attempt"
}

probe_cmd() {  # profile rate condition variant mode [ms:step] -> cmd[]
  local base; base=$(log_base "$@")
  variant_flags "$4"
  cmd=("$PROBE")
  [ -n "$port" ] && cmd+=(--port "$port")
  cmd+=(--rate-hz "$2" --profile "$1" --duration "$duration"
        --log "${logdir:-$outdir}/$base.csv")
  case "$3" in rt-*) cmd+=(--rt-priority "$rt_priority" --rt-cpu "$rt_cpu") ;; esac
  cmd+=("${vflags[@]}")
  [ -n "${6:-}" ] && cmd+=(--expect-latency-timer-ms "${6%%:*}")
  return 0
}

total=$(( ${#run_list[@]} * ${#step_list[@]} * ${#pl_list[@]} * ${#mode_list[@]} ))

# A planned log name already in the archive manifest would give a second
# dataset that name: refused, as the prerequisites are (a dry run lists it).
taken=()
if [ -f "$manifest" ]; then
  # Every name the manifest holds, reduced to its basename: a table row's,
  # a directory-qualified entry's or citation's, a sidecar's. A log is taken
  # if its <log>.csv is one of them or the start of a sidecar's.
  archived=$(grep -oE '[A-Za-z0-9._/+-]+' "$manifest" | sed 's#.*/##')
  for spec in "${run_list[@]}"; do
    IFS=: read -r p r c v <<<"$spec"
    for step in "${step_list[@]}"; do
      for placement in "${pl_list[@]}"; do
        for m in "${mode_list[@]}"; do
          b=$(log_base "$p" "$r" "$c" "$v" "$m" "$step")
          awk -v n="$b.csv" '$0 == n || index($0, n ".") == 1 { found = 1; exit }
                             END { exit !found }' <<<"$archived" && taken+=("$b")
        done
      done
    done
  done
  if [ -n "$fs_controls_cpu" ]; then
    b=fs-controls-$date_tag-$attempt
    awk -v n="$b." 'index($0, n) == 1 { found = 1; exit } END { exit !found }' <<<"$archived" && taken+=("$b")
  fi
  if [ ${#taken[@]} -gt 0 ]; then
    for b in "${taken[@]}"; do
      echo "trace_session: $b.csv is already in the archive manifest $manifest" >&2
    done
    echo "trace_session: pass an --attempt whose log names are free" >&2
  fi
else
  echo "trace_session: no archive manifest at $manifest to check the log names against (run from the repo root, or pass --manifest)" >&2
  taken+=(none)
fi
placement=
[ ${#taken[@]} -eq 0 ] || [ "$dry_run" -eq 1 ] || exit 2

if [ "$dry_run" -eq 1 ]; then
  echo "plan: $total runs (date $date_tag, attempt $attempt), tracers and -excl load on CPUs $other_cpus:"
  for spec in "${run_list[@]}"; do
    IFS=: read -r p r c v <<<"$spec"
    for step in "${step_list[@]}"; do
      for placement in "${pl_list[@]}"; do
      for m in "${mode_list[@]}"; do
        probe_cmd "$p" "$r" "$c" "$v" "$m" "$step"
        stress_cmd "$c"
        printf '  [%s' "$m"
        [ -n "$placement" ] && printf '; observers %s' "$placement"
        [ "$m" = profiled ] && printf '; profiled'
        [ -n "$step" ] && printf '; latency timer %s ms (step %s)' "${step%%:*}" "${step##*:}"
        [ ${#scmd[@]} -gt 0 ] && printf '; load: %s' "${scmd[*]}"
        printf '] '; printf '%q ' "${cmd[@]}"; echo
      done
      done
    done
  done
  placement=
  if [ -n "$placements" ]; then
    echo "placements $placements: a confined run moves the trees of $observers off CPUs $exclude_cpus through $OBS_TOOL (recorded first, restored right after the probe and on every exit) and starts the probe on CPUs $other_cpus"
  fi
  echo "staging: each run's capture streams on a private noswap tmpfs under $STAGE_ROOT ($STAGE_TOOL), published to $outdir, verified and released after the run; the budgets (provisional) at today's MemAvailable:"
  for m in "${mode_list[@]}"; do
    python3 "$STAGE_TOOL" budget --mode "$m" --duration-s "$duration" --rate-hz "$max_rate" --buffer-kb "$buffer_kb" \
      --cpus "$possible_cpus" --usbmon "$usbmon" --perf-hz "$perf_hz" --perf-stack "$perf_stack" \
      --exerciser "$exerciser" --meminfo "$MEMINFO" 2>&1 | grep -v = | sed 's/^/  /'
  done
  if [ ${#lt_list[@]} -gt 0 ]; then
    echo "latency steps $latency_steps on $lt_tty through $LATENCY_TOOL: read before anything is sent, set before each step's runs, read after each run, and restored on every exit"
  fi
  if [ "$any_traced" -eq 1 ]; then
    printf 'traced runs add: stdbuf -oL -eL %s %s --temp <stage>/<log>.csv.trace-tmp -o <stage>/<log>.csv.trace.dat\n' "$TRACE_CMD" "${trace_args[*]}"
    if [ "$any_timer" -eq 1 ]; then
      printf 'timer runs record: the traced events, plus (after the availability check) every timer arming and cancellation, CPU %s deadline writes, idle, NMIs, system calls and page faults, the worker'"'"'s sleep calls by name, and process lifecycle;\n' "$rt_cpu"
      printf '                   %s start %s %s %s %s (x86-tsc instance), %s on CPU %s and %s, %s stat -e msr/smi/ -C %s -I 10%s\n' \
        "$TSC_INSTANCE_TOOL" "$TRACEFS" "$TSC_INSTANCE" "$rt_cpu" "$buffer_kb" "$TSC_SAMPLER" "${others[0]}" "$rt_cpu" \
        "$PERF" "$rt_cpu" "$([ "$exerciser" -eq 1 ] && echo "; the exerciser on CPU $rt_cpu")"
    fi
    if [ -n "$fs_controls_cpu" ]; then
      printf 'before the runs: the fair server'"'"'s probe controls on CPU %s (supervisor CPU %s, tracers on CPUs %s): python3 %s run --cpu %s --supervisor-cpu %s --output fs-controls-%s-%s.controls.txt\n' \
        "$fs_controls_cpu" "$ctl_sup" "$ctl_tracer_cpus" "$FS_CONTROL_TOOL" "$fs_controls_cpu" "$ctl_sup" "$date_tag" "$attempt"
    fi
    if [ "$any_timer" -eq 1 ]; then
      printf '                   the fair server'"'"'s probe events (group %s, %s definitions; defined before the first run, removed at the end), filtered to CPU %s\n' \
        "$FS_GROUP" "$FS_TOOL" "$rt_cpu"
    fi
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
trace_job=; usbmon_job=; perf_job=; guard_job=; snap_pid=
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
  taskset -c "$tracer_cpus" "${sudo_cmd[@]}" python3 -c '
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
  if [ -n "$smi_job" ]; then alive "$smi_job" || gone+="${gone:+, }perf stat ended"; fi
  if [ -n "$sampler_pid" ]; then alive "$sampler_pid" || gone+="${gone:+, }the TSC sampler ended"; fi
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
  if [ -n "$snap_pid" ]; then kill "$snap_pid" 2>/dev/null; wait "$snap_pid" 2>/dev/null; snap_pid=; fi
  if [ -n "$ex_pid" ]; then kill -TERM "$ex_pid" 2>/dev/null; wait "$ex_pid" 2>/dev/null; ex_pid=; fi
  if [ -n "$sampler_pid" ]; then kill -TERM "$sampler_pid" 2>/dev/null; wait "$sampler_pid" 2>/dev/null; sampler_pid=; fi
  if [ -n "$smi_job" ]; then stop_tracer smi "$smi_job"; smi_job=; fi
  remove_tsc_instance
  if [ -n "$trace_job" ]; then stop_tracer trace "$trace_job"; trace_job=; fi
  if [ -n "$usbmon_job" ]; then stop_tracer usbmon "$usbmon_job"; usbmon_job=; fi
  if [ -n "$perf_job" ]; then stop_tracer perf "$perf_job"; perf_job=; fi
  if [ -n "$guard_job" ]; then  # the staging guard's record, on every path that stops it
    stop_tracer guard "$guard_job"; guard_job=
    [ -n "$last_note" ] && [ -f "$last_note" ] && { echo; echo "--- staging guard ---"; cat "$tmp/guard.out"; } >>"$last_note"
  fi
  if [ -n "$stress_pid" ]; then
    kill -TERM -- "-$stress_pid" 2>/dev/null
    wait "$stress_pid" 2>/dev/null
    stress_pid=
  fi
  own_outputs
}
# The timer mode's TSC instance: tsc_owed is 1 once this session's start
# created it, 2 while a start's outcome is unknown (during the start, or
# after a status that does not say), else 0. An owed instance is removed
# (and its removal verified) on every path that stops the recorders and on
# every exit; a failed removal is a cleanup failure (status 3). An instance
# of unknown ownership is left in place, since it may be another session's,
# and reported: never removed on a guess.
tsc_owed=0; ex_pid=; sampler_pid=; smi_job=
remove_tsc_instance() {
  local out
  [ "$tsc_owed" -ne 0 ] || return 0
  if [ "$tsc_owed" -eq 2 ]; then
    tsc_owed=0
    priv "$TSC_INSTANCE_TOOL" exists "$TRACEFS" "$TSC_INSTANCE" || return 0
    echo "trace_session: the trace instance $TSC_INSTANCE exists after a start whose outcome is unknown; it was left in place (it may be another session's)" >&2
    echo "  if it is this session's, remove it by hand: sudo rmdir $TRACEFS/instances/$TSC_INSTANCE" >&2
    if [ -n "$last_note" ] && [ -f "$last_note" ]; then
      printf '\n--- session end ---\ncleanup: trace instance %s left in place (the outcome of its start unknown)\n' "$TSC_INSTANCE" >>"$last_note"
    fi
    return 1
  fi
  if out=$(priv "$TSC_INSTANCE_TOOL" remove "$TRACEFS" "$TSC_INSTANCE" 2>&1); then
    tsc_owed=0
    return 0
  fi
  echo "trace_session: CLEANUP FAILED: the trace instance $TSC_INSTANCE was not removed: $out" >&2
  echo "  remove it by hand: sudo rmdir $TRACEFS/instances/$TSC_INSTANCE" >&2
  if [ -n "$last_note" ] && [ -f "$last_note" ]; then
    printf '\n--- session end ---\ncleanup: trace instance %s NOT removed: %s\n' "$TSC_INSTANCE" "$out" >>"$last_note"
  fi
  return 1
}

# The fair server's probe events: fs_owed is 1 once this session's
# definition created any, 2 while a definition's outcome is unknown, else 0.
# They are removed (verified) at the session's end and on every exit, after
# the recorders stopped; a failed removal is a cleanup failure (status 3). A
# group of unknown ownership is left in place and reported.
fs_owed=0
fs_hint() {
  local n
  echo "  remove them by hand (as root), one line per event:" >&2
  for n in $(sed -n 's#^[pr]:[A-Za-z0-9_]*/\([A-Za-z0-9_]*\) .*#\1#p' <<<"$fs_defs"); do
    echo "    echo -:$FS_GROUP/$n | sudo tee -a $TRACEFS/kprobe_events" >&2
  done
}
remove_fs_probes() {
  local out f st=0
  [ "$fs_owed" -ne 0 ] || return 0
  f=$(mktemp)
  printf '%s\n' "$fs_defs" >"$f"
  if [ "$fs_owed" -eq 2 ]; then
    fs_owed=0
    if priv "$FS_PROBE_TOOL" exists "$TRACEFS" "$FS_GROUP"; then
      echo "trace_session: the probe group $FS_GROUP exists after a definition whose outcome is unknown; it was left in place (it may be another session's)" >&2
      fs_hint
      if [ -n "$last_note" ] && [ -f "$last_note" ]; then
        printf '\n--- session end ---\ncleanup: probe group %s left in place (the outcome of its definition unknown)\n' "$FS_GROUP" >>"$last_note"
      fi
      st=1
    fi
  elif out=$(priv "$FS_PROBE_TOOL" remove "$TRACEFS" "$FS_GROUP" "$f" 2>&1); then
    fs_owed=0
  else
    echo "trace_session: CLEANUP FAILED: the fair server's probes ($FS_GROUP) were not removed: $out" >&2
    fs_hint
    if [ -n "$last_note" ] && [ -f "$last_note" ]; then
      printf '\n--- session end ---\ncleanup: probe group %s NOT removed: %s\n' "$FS_GROUP" "$out" >>"$last_note"
    fi
    st=1
  fi
  rm -f "$f"
  return "$st"
}

# The observers' placement: restored from its record (verified by the tool);
# a failure is a cleanup failure (status 3), with the command to retry.
restore_observers() {
  local out
  [ "$obs_owed" -ne 0 ] || return 0
  if out=$(python3 "$OBS_TOOL" restore --record "$obs_record" 2>&1); then
    obs_owed=0
    [ -n "$last_note" ] && [ -f "$last_note" ] && echo "$out" >>"$last_note"
    return 0
  fi
  echo "trace_session: CLEANUP FAILED: the observers' placement was not restored: $out" >&2
  echo "  retry: python3 $OBS_TOOL restore --record $obs_record" >&2
  if [ -n "$last_note" ] && [ -f "$last_note" ]; then
    printf '\n--- session end ---\ncleanup: observer placement NOT restored: %s\n' "$out" >>"$last_note"
  fi
  return 1
}

# The run's stage (tools/trace_stage.py): stage_owed is 0 without one; 1
# once its mount point exists; 2 once a mount was attempted (mounted or
# not); 3 once it is verified and marked, the run's streams going there; 4
# when it is kept after a failed publication, release or abort (reported,
# never retried by this session). A stage without data (1, 2, or 3 before
# any writer) is undone by abort; otherwise it is published and released.
# stage_failed makes the session fail (status 3, 130 kept).
stage_dir=; stage_owed=0; stage_note=; stage_failed=0; stage_seq=0; stage_published=0
stage_kernel=0; stage_gfree=0; stage_gavail=0; guard_stop=
stage_name() {  # -> stage_dir, a fresh name (nothing is created)
  stage_seq=$((stage_seq + 1))
  stage_dir="$STAGE_ROOT/$session_id-$attempt-$stage_seq-$(od -An -N4 -tx1 /dev/urandom | tr -d ' \n')"
}
stage_open() {  # mode duration_s rate_hz base note -> 0 once the stage is ready, 1 when the run must not start
  local mode=$1 dur=$2 rate=$3 base=$4 note=$5 out st size_mib
  stage_note=$note; stage_published=0
  out=$(python3 "$STAGE_TOOL" budget --mode "$mode" --duration-s "$dur" --rate-hz "$rate" --buffer-kb "$buffer_kb" \
        --cpus "$possible_cpus" --usbmon "$usbmon" --perf-hz "$perf_hz" --perf-stack "$perf_stack" \
        --exerciser "$exerciser" --meminfo "$MEMINFO" 2>&1); st=$?
  { echo; echo "--- staging ($STAGE_TOOL; provisional budget; the guard is not a reservation) ---"
    grep -v = <<<"$out"; } >>"$note"
  size_mib=$(sed -n 's/^size_mib=//p' <<<"$out")
  stage_kernel=$(sed -n 's/^kernel=//p' <<<"$out")
  stage_gfree=$(sed -n 's/^guard_min_free=//p' <<<"$out")
  stage_gavail=$(sed -n 's/^guard_min_available=//p' <<<"$out")
  if [ "$st" -ne 0 ] || [ -z "$size_mib" ] || [ -z "$stage_kernel" ]; then
    echo "staging: refused: the run's budget does not fit (exit $st)" >>"$note"
    return 1
  fi
  if ! out=$(priv mkdir -p -m 0755 "$STAGE_ROOT" 2>&1) || ! out=$(priv mkdir -m 0700 "$stage_dir" 2>&1); then
    echo "staging: the mount point $stage_dir was not created: $out" >>"$note"
    return 1
  fi
  stage_owed=1
  stage_owed=2
  if ! out=$(priv "$MOUNT" -t tmpfs -o "size=${size_mib}m,noswap,mode=0700,uid=$(id -u),gid=$(id -g),nosuid,nodev,noexec" \
             rtctrl-stage "$stage_dir" 2>&1); then
    echo "staging: the tmpfs was not mounted on $stage_dir: $out" >>"$note"
    stage_abort
    return 1
  fi
  if ! out=$(python3 "$STAGE_TOOL" verify --stage "$stage_dir" --size-mib "$size_mib" 2>&1); then
    echo "staging: the mount was not verified: $out" >>"$note"
    stage_abort
    return 1
  fi
  echo "staging: $out" >>"$note"
  if ! out=$(python3 "$STAGE_TOOL" mark --stage "$stage_dir" --session "$session_id" --base "$base" \
             --outdir "$outdir" 2>&1); then
    echo "staging: the stage was not marked: $out" >>"$note"
    stage_abort
    return 1
  fi
  stage_owed=3
  echo "staging: $out" >>"$note"
  echo "logging: the probe's log and every capture stream go to the stage $stage_dir (tmpfs), then are published to $outdir; a diagnostic arrangement, not a qualification of production logging to disk" >>"$note"
}
stage_abort() {  # undoes a stage that holds no data: 0 when undone, 1 when kept
  local out
  if out=$(python3 "$STAGE_TOOL" abort --stage "$stage_dir" 2>&1); then
    stage_owed=0
    [ -n "$stage_note" ] && echo "staging: $out" >>"$stage_note"
    return 0
  fi
  stage_owed=4; stage_failed=1
  echo "trace_session: CLEANUP FAILED: the stage $stage_dir was not undone: $out" >&2
  [ -n "$stage_note" ] && echo "staging: NOT undone (kept): $out" >>"$stage_note"
  return 1
}
# Publishes the run's stage once every writer has stopped, then releases it:
# 0 when published, verified and released (or nothing was owed); 1 when the
# stage is kept (stage_published says whether its copies were verified).
stage_close() {
  local out st
  case "$stage_owed" in
    0|4) return 0 ;;
    1|2) stage_abort; return $? ;;
  esac
  [ ${#sudo_cmd[@]} -gt 0 ] && priv chown -R "$(id -u):$(id -g)" "$stage_dir" 2>/dev/null
  out=$(timeout -k 10 600 python3 "$STAGE_TOOL" publish --stage "$stage_dir" --outdir "$outdir" --meminfo "$MEMINFO" 2>&1)
  st=$?
  { echo; echo "--- staging: publication (exit $st) ---"; echo "$out"; } >>"$stage_note"
  if [ "$st" -ne 0 ]; then
    stage_owed=4; stage_failed=1
    echo "preservation: NOT published: the staged evidence is kept at $stage_dir" >>"$stage_note"
    echo "trace_session: PRESERVATION FAILED: the run's staged evidence is kept at $stage_dir (see $stage_note)" >&2
    echo "  retry: python3 $STAGE_TOOL publish --stage $stage_dir --outdir $outdir" >&2
    echo "  then:  python3 $STAGE_TOOL release --stage $stage_dir --outdir $outdir" >&2
    return 1
  fi
  stage_published=1
  out=$(python3 "$STAGE_TOOL" release --stage "$stage_dir" --outdir "$outdir" 2>&1); st=$?
  { echo "--- staging: release (exit $st) ---"; echo "$out"; } >>"$stage_note"
  if [ "$st" -ne 0 ]; then
    stage_owed=4; stage_failed=1
    echo "preservation: published and verified; the stage was NOT released ($stage_dir)" >>"$stage_note"
    echo "trace_session: CLEANUP FAILED: the stage $stage_dir was not released: $out" >&2
    echo "  retry: python3 $STAGE_TOOL release --stage $stage_dir --outdir $outdir" >&2
    return 1
  fi
  stage_owed=0
  echo "preservation: published and verified, the stage released" >>"$stage_note"
  return 0
}
stage_usage() {  # -> "used free" bytes of the stage
  python3 "$STAGE_TOOL" usage --stage "$stage_dir" 2>/dev/null | awk '{print $2, $4}'
}
# trace-cmd's per-CPU temporary files: on the stage, in its temporary
# directory, and nowhere else (the output directory, beside the output).
stage_temp_check() {  # temp dir, output basename, note -> 0 when they are
  local i n=0 stray
  for ((i = 0; i < 50; i++)); do
    n=$(find "$1" -maxdepth 1 -name "$2.cpu*" 2>/dev/null | wc -l)
    [ "$n" -gt 0 ] && break
    sleep 0.1
  done
  stray=$(find "$outdir" "$stage_dir" -maxdepth 1 -name "$2.cpu*" 2>/dev/null | head -n 3 | tr '\n' ' ')
  if [ "$n" -eq 0 ] || [ -n "$stray" ] || [ "$(stat -c %d "$1")" != "$(stat -c %d "$stage_dir")" ]; then
    echo "trace_session: trace-cmd's per-CPU temporary files are not on the stage only ($n in $1; elsewhere: ${stray:-none}): session stopped" | tee -a "$3"
    return 1
  fi
  echo "staging: trace-cmd's per-CPU temporary files: $n in $1 (the stage's filesystem), none in $outdir or beside the output" >>"$3"
}
# Both trace arrays' buffers, read back before the probe against the
# budget's kernel allowance.
stage_kernel_check() {  # 1 with the TSC instance, note -> 0 when they fit
  local m i='' total
  m=$(priv cat "$TRACEFS/buffer_total_size_kb" 2>&1 | awk 'NR == 1 {print $1}')
  [ "$1" -eq 1 ] && i=$(priv cat "$TRACEFS/instances/$TSC_INSTANCE/buffer_total_size_kb" 2>&1 | awk 'NR == 1 {print $1}')
  echo "kernel buffers read back (buffer_total_size_kb): main ${m:-none}${i:+, instance $TSC_INSTANCE $i} kB; the budget allows $((stage_kernel / 1024)) kB" >>"$2"
  if ! is_uint "$m" || { [ "$1" -eq 1 ] && ! is_uint "$i"; }; then
    echo "trace_session: the trace buffers' sizes were not read back: session stopped" | tee -a "$2"
    return 1
  fi
  total=$((m + ${i:-0}))
  if [ $((total * 1024)) -gt "$stage_kernel" ]; then
    echo "trace_session: the trace buffers ($total kB) exceed the staging budget's $((stage_kernel / 1024)) kB: session stopped" | tee -a "$2"
    return 1
  fi
}
stage_guard_start() {  # note -> 0 once the guard samples the stage
  local pids=()
  [ -n "$trace_job" ] && [ -s "$tmp/trace.pid" ] && pids+=(--stop-pid "$(cat "$tmp/trace.pid")")
  [ -n "$perf_job" ] && [ -s "$tmp/perf.pid" ] && pids+=(--stop-pid "$(cat "$tmp/perf.pid")")
  start_tracer guard python3 "$STAGE_TOOL" guard --stage "$stage_dir" --min-free "$stage_gfree" \
    --min-available "$stage_gavail" --meminfo "$MEMINFO" "${pids[@]}"
  if ! wait_line guard "$guard_job" "guard: every" 30; then
    { echo "--- the staging guard did not start ---"; cat "$tmp/guard.out"; } | tee -a "$1"
    return 1
  fi
}
stage_guard_stop() {  # note -> guard_stop, the guard's stop if it made one (not in a subshell: it ends the job)
  guard_stop=
  [ -n "$guard_job" ] || return 0
  stop_tracer guard "$guard_job"; guard_job=
  { echo; echo "--- staging guard ---"; cat "$tmp/guard.out"; } >>"$1"
  guard_stop=$(sed -n 's/^guard stop: \(.*\) at monotonic_ns.*/the staging guard stopped the recorders (\1)/p' "$tmp/guard.out" | head -n 1)
}
stage_assembly_note() {  # "used free" before trace-cmd's stop, the trace, note -> "the staging filled ..." if it did
  local after size free
  after=$(stage_usage)
  size=$(stat -c %s "$2" 2>/dev/null || echo 0)
  echo "staging usage: ${1%% *} bytes just before trace-cmd's stop (its raw files), ${after%% *} after its exit; the assembled trace $size bytes; conservative peak bound $(( ${1%% *} + size )) bytes (the raw files and the output together)" >>"$3"
  free=${after##* }
  if [ "${free:-0}" -lt 1048576 ] || grep -qi 'no space left' "$tmp/trace.out" 2>/dev/null; then
    echo "the staging filled (${free:-unknown} bytes free at the end)"
  fi
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
  local cleanup_ok=1
  if ! restore_latency; then
    echo "trace_session: the session fails: its cleanup did not restore the adapter's latency timer" >&2
    cleanup_ok=0
  fi
  if [ "$tsc_owed" -ne 0 ] && ! remove_tsc_instance; then
    echo "trace_session: the session fails: its cleanup did not remove the trace instance" >&2
    cleanup_ok=0
  fi
  if [ "$obs_owed" -ne 0 ] && ! restore_observers; then
    echo "trace_session: the session fails: its cleanup did not restore the observers' placement" >&2
    cleanup_ok=0
  fi
  if [ "$fs_owed" -ne 0 ] && ! remove_fs_probes; then
    echo "trace_session: the session fails: its cleanup did not remove the fair server's probes" >&2
    cleanup_ok=0
  fi
  # The stage last, independently of the cleanups above: published and
  # released, or kept and named.
  if [ "$stage_owed" -eq 4 ]; then
    echo "trace_session: the stage $stage_dir is kept (see $stage_note)" >&2
  elif [ "$stage_owed" -ne 0 ]; then
    stage_close || :
  fi
  if [ "$stage_failed" -eq 1 ]; then
    echo "trace_session: the session fails: a run's staged evidence was not published and verified, or its stage not released" >&2
    [ "$status" -eq 130 ] || exit 3
  fi
  [ "$cleanup_ok" -eq 1 ] || { [ "$status" -eq 0 ] && exit 3; }
  return 0
}
finish() {
  stop_all
  remove_fs_probes || :  # a failure is reported, and retried and counted at exit
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
# The timer mode's events, its trace clock and a free instance name, checked
# before anything is sent; the trace arguments follow from what the kernel
# offers (hrtimer_setup or hrtimer_init; cpu_idle_miss if present).
timer_args=()
if [ "$any_timer" -eq 1 ]; then
  priv "$TRACE_CMD" list -e >"$tmp/events.txt" 2>&1 || { finish; die "trace-cmd list -e failed: nothing was sent"; }
  missing=()
  for e in timer:hrtimer_start timer:hrtimer_cancel msr:write_msr power:cpu_idle nmi:nmi_handler \
           raw_syscalls:sys_enter raw_syscalls:sys_exit exceptions:page_fault_user exceptions:page_fault_kernel \
           sched:sched_process_fork sched:sched_process_exec sched:sched_process_exit irq_vectors:local_timer_entry \
           "${sleep_events[@]}"; do
    grep -qx -- "$e" "$tmp/events.txt" || missing+=("$e")
  done
  init_ev=
  for e in timer:hrtimer_setup timer:hrtimer_init; do
    if grep -qx -- "$e" "$tmp/events.txt"; then init_ev=$e; break; fi
  done
  [ -n "$init_ev" ] || missing+=("timer:hrtimer_setup or timer:hrtimer_init")
  [ ${#missing[@]} -eq 0 ] || { finish; die "the timer mode's events are not available (${missing[*]}): nothing was sent"; }
  case "$(priv cat "$TRACEFS/trace_clock" 2>&1)" in
    *x86-tsc*) ;;
    *) finish; die "the kernel offers no x86-tsc trace clock: nothing was sent" ;;
  esac
  if priv "$TSC_INSTANCE_TOOL" exists "$TRACEFS" "$TSC_INSTANCE"; then
    finish
    die "the trace instance $TSC_INSTANCE remains from an earlier session (sudo rmdir $TRACEFS/instances/$TSC_INSTANCE): nothing was sent"
  fi
  # The fair server's probes, defined once for the session, before anything
  # is sent; a group left by an earlier session refuses it.
  if priv "$FS_PROBE_TOOL" exists "$TRACEFS" "$FS_GROUP"; then
    finish
    die "the probe group $FS_GROUP remains from an earlier session (see $FS_PROBE_TOOL remove): nothing was sent"
  fi
  printf '%s\n' "$fs_defs" >"$tmp/fs_defs.txt"
  fs_owed=2
  fs_out=$(priv "$FS_PROBE_TOOL" define "$TRACEFS" "$FS_GROUP" "$tmp/fs_defs.txt" 2>&1)
  fs_status=$?
  case "$fs_status" in 0|4) fs_owed=1 ;; 2) fs_owed=0 ;; *) fs_owed=2 ;; esac
  if [ "$fs_status" -ne 0 ]; then
    echo "$fs_out" >&2
    finish
    die "the fair server's probes were not defined (exit $fs_status): nothing was sent"
  fi
  priv "$TRACE_CMD" list -e >"$tmp/events-fs.txt" 2>&1
  fs_missing=()
  for e in $(sed -n 's#^[pr]:[A-Za-z0-9_]*/\([A-Za-z0-9_]*\) .*#\1#p' "$tmp/fs_defs.txt"); do
    grep -qx -- "$FS_GROUP:$e" "$tmp/events-fs.txt" || fs_missing+=("$FS_GROUP:$e")
  done
  if [ ${#fs_missing[@]} -gt 0 ]; then
    finish
    die "the fair server's probe events are not listed by trace-cmd (${fs_missing[*]}): nothing was sent"
  fi
  timer_args=("${trace_args[@]}" -e timer:hrtimer_start -e timer:hrtimer_cancel -e "$init_ev"
              -e msr:write_msr -f "msr == 0x6e0 && CPU == $rt_cpu"
              -e power:cpu_idle -f "cpu_id == $rt_cpu")
  if grep -qx power:cpu_idle_miss "$tmp/events.txt"; then
    timer_args+=(-e power:cpu_idle_miss -f "cpu_id == $rt_cpu")
  fi
  for e in nmi:nmi_handler raw_syscalls:sys_enter raw_syscalls:sys_exit exceptions:page_fault_user \
           exceptions:page_fault_kernel; do
    timer_args+=(-e "$e" -f "CPU == $rt_cpu")
  done
  timer_args+=(-e sched:sched_process_fork -e sched:sched_process_exec -e sched:sched_process_exit)
  # The worker's sleep calls by name, as a profiled run records them (the
  # sleep checker's evidence; the raw system calls carry only numbers).
  for e in "${sleep_events[@]}"; do timer_args+=(-e "$e" -f "comm == \"$probe_comm\""); done
  # The fair server's probes: the picks by the CPU they run on, the server's
  # calls by the server's CPU.
  while read -r e f; do
    timer_args+=(-e "$FS_GROUP:$e" -f "$f")
  done < <(python3 - "$rt_cpu" <<'EOF'
import sys
sys.path.insert(0, "tools")
import fair_server
for e, f in fair_server.filters(int(sys.argv[1])):
    print(e, f)
EOF
)
  {
    echo "--- fair-server probes (group $FS_GROUP, defined by this session; $FS_TOOL) ---"
    echo "$fs_layout"
    echo "$fs_out"
    sed 's/^/  /' "$tmp/fs_defs.txt"
    priv "$TRACE_CMD" list -e "^$FS_GROUP:" -F 2>&1
    echo "--- timer mode events (trace-cmd list -e ... -F) ---"
    for e in msr:write_msr power:cpu_idle nmi:nmi_handler timer:hrtimer_start timer:hrtimer_cancel "$init_ev" \
             sched:sched_process_fork sched:sched_process_exec; do
      priv "$TRACE_CMD" list -e "^$e\$" -F 2>&1
    done
    echo "--- CPU $rt_cpu idle states (name, latency us, target residency us, disabled) ---"
    for s in /sys/devices/system/cpu/cpu"$rt_cpu"/cpuidle/state*; do
      [ -d "$s" ] && echo "$(basename "$s"): $(cat "$s/name" 2>&1) $(cat "$s/latency" 2>&1) $(cat "$s/residency" 2>&1) $(cat "$s/disable" 2>&1)"
    done
    echo "clock event device of CPU $rt_cpu: $(cat "/sys/devices/system/clockevents/clockevent$rt_cpu/current_device" 2>&1)"
  } >"$tmp/timer_events.txt"
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
  echo "session pid: $$"
  echo "batch workloads: $batch_list (exempt: $BATCH_EXEMPT)"
  if [ -e "$QUIET_HOST_STATE" ]; then
    echo "temporary host changes (tools/quiet_host.sh, $QUIET_HOST_STATE): $(tr '\n' ' ' <"$QUIET_HOST_STATE")"
  else
    echo "temporary host changes: none recorded by tools/quiet_host.sh"
  fi
  [ "$any_timer" -eq 1 ] && cat "$tmp/timer_events.txt"
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
clock_offsets() {  # BOOTTIME and TAI minus MONOTONIC, for timers on those clocks (timer mode)
  python3 - <<'EOF'
import time
M, B, T = time.CLOCK_MONOTONIC, time.CLOCK_BOOTTIME, time.CLOCK_TAI
best = None
for _ in range(2000):
    a = time.clock_gettime_ns(M)
    b = time.clock_gettime_ns(B)
    t = time.clock_gettime_ns(T)
    c = time.clock_gettime_ns(M)
    if best is None or c - a < best[3] - best[0]:
        best = (a, b, t, c)
a, b, t, c = best
m = (a + c) // 2
print(f"boottime_minus_monotonic_ns {b - m} tai_minus_monotonic_ns {t - m} bracket_ns {c - a}")
EOF
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

mono_ns() { python3 -c 'import time; print(time.clock_gettime_ns(time.CLOCK_MONOTONIC))'; }
session_id="$$-$(date +%Y%m%dT%H%M%S)"

# A buffer's losses from per-CPU stats in "CPU: N" blocks (as trace-cmd
# report --stat prints them, and tsc_instance.sh stats): the sum of the
# overrun, commit overrun and dropped events counters over the CPUs given,
# or status 1 with the reason when one of those CPUs, or one of its
# counters, is missing or not a number. A failed or empty read is never a
# count of zero.
loss_sum() {  # stats text, CPUs (space-separated)
  awk -v want="$2" '
    BEGIN {
      n = split(want, w, " "); for (i = 1; i <= n; i++) need[w[i]] = 1
      nk = split("overrun|commit overrun|dropped events", keys, "|")
    }
    /^[[:space:]]*CPU:[[:space:]]*[0-9]+[[:space:]]*$/ { cpu = $2; seen[cpu] = 1; next }
    cpu != "" {
      for (k = 1; k <= nk; k++) {
        p = "^[[:space:]]*" keys[k] ":"
        if ($0 ~ p) {
          v = $0; sub(p "[[:space:]]*", "", v); sub(/[[:space:]]+$/, "", v)
          got[cpu, k] = 1
          if (v ~ /^[0-9]+$/) s += v; else bad = bad sprintf("CPU %s %s is \"%s\"; ", cpu, keys[k], v)
        }
      }
    }
    END {
      for (c in need) {
        if (!(c in seen)) { bad = bad "no stats for CPU " c "; "; continue }
        for (k = 1; k <= nk; k++) if (!((c, k) in got)) bad = bad sprintf("CPU %s has no %s count; ", c, keys[k])
      }
      if (bad != "") { sub(/; $/, "", bad); print bad; exit 1 }
      printf "%d\n", s
    }' <<<"$1"
}

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
  local loaded=0 probe_status check_status seen stress_out stress_status sb pub filled
  local dat pcap lt_ms=${step%%:*} lt_out lt_status irq_before irq_after t_before t_after
  local traced=0 profiled=0 timered=0 perfdat perftxt perfstacks targs tsc_txt tsc_pairs
  case "$mode" in traced) traced=1 ;; profiled) traced=1; profiled=1 ;; timer) traced=1; timered=1 ;; esac
  base=$(log_base "$profile" "$rate" "$cond" "$variant" "$mode" "$step")
  csv="$outdir/$base.csv"
  note="$csv.run.txt"
  # The capture streams go to the run's stage (sb), the records stay here.
  stage_name
  sb="$stage_dir/$base.csv"
  logdir=$stage_dir
  dat="$sb.trace.dat"
  pcap="$sb.usbmon.pcap"
  perfdat="$sb.perf.data"
  perftxt="$sb.perf.txt"
  perfstacks="$sb.perf.stacks.txt"
  tsc_txt="$sb.tsc.txt"
  tsc_pairs="$sb.tsc-pairs.csv"
  targs=("${trace_args[@]}")
  [ "$profiled" -eq 1 ] && targs=("${profile_args[@]}")
  [ "$timered" -eq 1 ] && targs=("${timer_args[@]}")
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
  local batch
  batch=$(batch_running "$batch_list")
  if [ -n "$batch" ]; then
    echo "trace_session: unrelated batch workloads are running ($batch): session stopped before the run$(batch_hint "$batch")"
    outcome=stop
    return
  fi
  case "$cond" in *loaded*) loaded=1 ;; esac
  wait_quiet
  probe_cmd "$profile" "$rate" "$cond" "$variant" "$mode" "$step"
  # A confined run starts the probe outside the excluded CPUs: its helper
  # threads keep that mask, and only its worker pins itself to the real-time CPU.
  [ "$placement" = confined ] && cmd=(taskset -c "$other_cpus" "${cmd[@]}")
  stress_cmd "$cond"
  {
    echo "rtctrl traced torque-off run"
    echo "started: $(date -Is)"
    echo "commit: $commit"
    echo "probe binary: $(stat -c %y "$PROBE")"
    echo "condition: $cond"
    echo "variant: $variant (bounded read path: the probe's 35 ms budget and equal guard)"
    echo "mode: $mode"
    case "$placement" in
      confined) echo "placement: confined (observers $observers moved off CPUs $exclude_cpus, the probe started on CPUs $other_cpus)" ;;
      open) echo "placement: open (observers $observers left as they are)" ;;
    esac
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
      printf 'trace: stdbuf -oL -eL %s' "$TRACE_CMD"; printf ' %q' "${targs[@]}"; printf ' --temp %q -o %q\n' "$sb.trace-tmp" "$dat"
      if [ "$profiled" -eq 1 ]; then
        printf 'perf: %s' "$PERF"; printf ' %q' "${perf_args[@]}"; printf ' -o %q\n' "$perfdat"
      fi
      if [ "$timered" -eq 1 ]; then
        echo "tsc instance: $TSC_INSTANCE_TOOL start $TRACEFS $TSC_INSTANCE $rt_cpu $buffer_kb (x86-tsc, local_timer_entry on CPU $rt_cpu)"
        echo "tsc sampler: $TSC_SAMPLER on CPU ${others[0]} throughout, and on CPU $rt_cpu for 0.5 s before and after the probe"
        echo "smi: $PERF stat -e msr/smi/ -C $rt_cpu -I 10 -x , -o $sb.smi.csv"
        if [ "$exerciser" -eq 1 ]; then
          echo "exerciser: python3 $EXERCISER --cpu $rt_cpu --duration $((duration + 60)) --seed $k (rehearsal; stopped after the probe)"
        fi
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
  # The run's stage, before anything else runs (tools/trace_stage.py): a
  # run whose stage is refused or fails does not start.
  if ! stage_open "$mode" "$duration" "$rate" "$base.csv" "$note"; then
    echo "trace_session: the run's staging failed: session stopped. See $note" | tee -a "$note"
    echo "verdict: not run (staging)" >>"$note"
    outcome=stop
    return
  fi
  [ "$traced" -eq 1 ] && mkdir "$sb.trace-tmp"
  # A confined run: the observers off the excluded CPUs, recorded first.
  if [ "$placement" = confined ]; then
    local obs_out obs_status
    obs_record="$csv.observers.jsonl"
    obs_owed=1
    obs_out=$(python3 "$OBS_TOOL" confine --pids "$observers" --exclude-cpus "$exclude_cpus" \
      --record "$obs_record" --exempt-pid "$$" 2>&1)
    obs_status=$?
    { echo "--- observer placement: python3 $OBS_TOOL confine (exit $obs_status) ---"; echo "$obs_out"; } >>"$note"
    [ -s "$obs_record" ] || obs_owed=0  # refused before writing anything
    if [ "$obs_status" -ne 0 ]; then
      echo "trace_session: the observers were not all confined (exit $obs_status): session stopped. See $note" | tee -a "$note"
      restore_observers
      echo "verdict: not run (placement)" >>"$note"
      outcome=stop
      return
    fi
  fi
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
  if [ "$timered" -eq 1 ]; then
    ps -eo pid,ppid,comm >"$sb.procs.txt"
  fi
  if [ "$traced" -eq 1 ]; then
    run_outputs=("$dat")
    if [ "$usbmon" -eq 1 ]; then run_outputs+=("$pcap"); fi
    if [ "$profiled" -eq 1 ]; then run_outputs+=("$perfdat"); fi
    # On a pipe, the C library would hold trace-cmd's "Hit Ctrl^C to stop
    # recording" until it exits; stdbuf (which execs it, keeping the PID)
    # makes its output line-buffered. tcpdump reports on stderr, which is
    # unbuffered.
    start_tracer trace stdbuf -oL -eL "$TRACE_CMD" "${targs[@]}" --temp "$sb.trace-tmp" -o "$dat"
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
    if ! stage_temp_check "$sb.trace-tmp" "$(basename "$dat")" "$note"; then
      stop_all
      echo "verdict: not run (staging)" >>"$note"
      outcome=stop
      return
    fi
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
    if [ "$timered" -eq 1 ]; then
      # The TSC instance, inside the main trace's span; the sampler,
      # throughout on a tracer CPU and briefly on the real-time CPU; the SMI
      # count, anchored before its start and when its first interval is seen.
      local inst_line inst_start smi_before smi_seen i
      # Its start's status says whether it created the instance: 0 and 4
      # (created, then a step failed) make it this session's to remove; 2
      # (not created: it exists, or cannot be made) leaves it alone, so
      # another session's is never removed; any other status (sudo's own
      # failure, an interruption) leaves the ownership unknown.
      tsc_owed=2
      inst_line=$(priv "$TSC_INSTANCE_TOOL" start "$TRACEFS" "$TSC_INSTANCE" "$rt_cpu" "$buffer_kb" 2>&1)
      inst_start=$?
      case "$inst_start" in 0|4) tsc_owed=1 ;; 2) tsc_owed=0 ;; *) tsc_owed=2 ;; esac
      if [ "$inst_start" -ne 0 ]; then
        { echo "--- the TSC instance did not start (exit $inst_start) ---"; echo "$inst_line"; } | tee -a "$note"
        echo "verdict: not run (TSC instance)" >>"$note"
        stop_all
        outcome=stop
        return
      fi
      echo "$inst_line" >>"$note"
      "$TSC_SAMPLER" --cpu "${others[0]}" --output "$tsc_pairs" &
      sampler_pid=$!
      if ! "$TSC_SAMPLER" --cpu "$rt_cpu" --duration-ms 500 --output "$tsc_pairs"; then
        echo "trace_session: the TSC sampler failed on CPU $rt_cpu: session stopped" | tee -a "$note"
        echo "verdict: not run (TSC sampler)" >>"$note"
        stop_all
        outcome=stop
        return
      fi
      run_outputs+=("$sb.smi.csv")
      smi_before=$(mono_ns)
      start_tracer smi "$PERF" stat -e msr/smi/ -C "$rt_cpu" -I 10 -x , -o "$sb.smi.csv"
      for ((i = 0; i < 6000; i++)); do
        grep -qv '^#' "$sb.smi.csv" 2>/dev/null && break
        alive "$smi_job" || break
        sleep 0.005
      done
      smi_seen=$(mono_ns)
      if ! grep -qv '^#' "$sb.smi.csv" 2>/dev/null; then
        stop_all
        { echo "--- perf stat did not start ---"; cat "$tmp/smi.out"; } | tee -a "$note"
        echo "verdict: not run (tracer)" >>"$note"
        outcome=stop
        return
      fi
      echo "smi anchor monotonic_ns: $smi_before $smi_seen (before perf stat started; when its first interval was seen)" >>"$note"
    fi
  fi
  # Both trace arrays' buffers, read back against the budget, and the
  # staging guard on the tracer CPUs, through the run and the assembly.
  if [ "$traced" -eq 1 ] && ! stage_kernel_check "$timered" "$note"; then
    stop_all
    echo "verdict: not run (staging)" >>"$note"
    outcome=stop
    return
  fi
  if ! stage_guard_start "$note"; then
    stop_all
    echo "verdict: not run (staging)" >>"$note"
    outcome=stop
    return
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
    [ "$timered" -eq 1 ] && echo "clock offsets: $(clock_offsets)" >>"$note"
    if [ "$timered" -eq 1 ] && [ "$exerciser" -eq 1 ]; then
      # It outlasts the probe, so the stop (SIGTERM, status 0) always finds
      # it running: a signal during its own exit would kill it.
      python3 "$EXERCISER" --cpu "$rt_cpu" --duration $((duration + 60)) --seed "$k" --output "$sb.exerciser.csv" &
      ex_pid=$!
    fi
  fi
  ( taskset -cp "$other_cpus" "$BASHPID" >/dev/null
    sleep $((duration / 2)); tasks_snapshot >"$sb.tasks.txt" ) &
  snap_pid=$!
  local fs_before='' fs_after='' fs_b_status=0 fs_a_status=0
  if [ "$timered" -eq 1 ]; then
    fs_before=$(priv "$FS_PROBE_TOOL" profile "$TRACEFS" "$FS_GROUP" "$tmp/fs_defs.txt" 2>&1); fs_b_status=$?
  fi
  python3 "$IDLE_TOOL" snapshot --cpus "$idle_cpus" --label before --root "$CPU_ROOT" >"$tmp/idle_before.txt" 2>&1
  python3 "$VM_TOOL" snapshot --label before --vmstat "$VMSTAT" --meminfo "$MEMINFO" >"$tmp/vm_before.txt" 2>&1
  irq_before=$(xhci_interrupts); t_before=$(date +%s.%N)
  "${cmd[@]}" 2>&1 | tee -a "$note"
  probe_status=${PIPESTATUS[0]}
  irq_after=$(xhci_interrupts); t_after=$(date +%s.%N)
  python3 "$IDLE_TOOL" snapshot --cpus "$idle_cpus" --label after --root "$CPU_ROOT" >"$tmp/idle_after.txt" 2>&1
  python3 "$VM_TOOL" snapshot --label after --vmstat "$VMSTAT" --meminfo "$MEMINFO" >"$tmp/vm_after.txt" 2>&1
  local placement_problem=''
  if [ "$placement" = confined ]; then
    { echo; echo "--- observer placement: restore ---"; } >>"$note"
    restore_observers || placement_problem="the observers' placement was not restored"
  fi
  if [ "$timered" -eq 1 ]; then
    fs_after=$(priv "$FS_PROBE_TOOL" profile "$TRACEFS" "$FS_GROUP" "$tmp/fs_defs.txt" 2>&1); fs_a_status=$?
  fi
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
  alive "$guard_job" || tracer_problem+="${tracer_problem:+, }the staging guard ended before the probe did"
  wait "$snap_pid" 2>/dev/null
  snap_pid=
  if [ "$placement" = confined ] && [ -z "$placement_problem" ]; then
    local chk wtid_c
    wtid_c=$(sed -n 's/^worker_tid = //p' "$sb.summary.toml" 2>/dev/null)
    chk=$(python3 "$OBS_TOOL" check --record "$obs_record" --tasks "$sb.tasks.txt" --worker-tid "${wtid_c:-0}" 2>&1) ||
      placement_problem="the placement was not in effect halfway through the run"
    { echo "--- observer placement: check of the halfway snapshot ---"; echo "$chk"; } >>"$note"
  fi
  if [ "$traced" -eq 1 ]; then
    echo "clock pair after: $(clock_pair)" >>"$note"
    if [ "$timered" -eq 1 ]; then
      local ex_status samp_status smi_status inst_out inst_status n_over
      if [ -n "$ex_pid" ]; then
        kill -TERM "$ex_pid" 2>/dev/null; wait "$ex_pid"; ex_status=$?; ex_pid=
        echo "exerciser: exit $ex_status, $(grep -cvE '^(#|round)' "$sb.exerciser.csv" 2>/dev/null) rows" >>"$note"
        [ "$ex_status" -eq 0 ] || tracer_problem+="${tracer_problem:+, }exerciser exit $ex_status"
      fi
      "$TSC_SAMPLER" --cpu "$rt_cpu" --duration-ms 500 --output "$tsc_pairs" ||
        tracer_problem+="${tracer_problem:+, }the TSC sampler failed on CPU $rt_cpu after the probe"
      kill -TERM "$sampler_pid" 2>/dev/null; wait "$sampler_pid"; samp_status=$?; sampler_pid=
      [ "$samp_status" -eq 0 ] || tracer_problem+="${tracer_problem:+, }TSC sampler exit $samp_status"
      echo "tsc sampler pairs: $(grep -cv '^#' "$tsc_pairs" 2>/dev/null)" >>"$note"
      stop_tracer smi "$smi_job"; smi_status=$?; smi_job=
      { echo; echo "--- perf stat (exit $smi_status) ---"; cat "$tmp/smi.out"; } >>"$note"
      { [ "$smi_status" -eq 0 ] || [ "$smi_status" -eq 130 ]; } && grep -qv '^#' "$sb.smi.csv" 2>/dev/null ||
        tracer_problem+="${tracer_problem:+, }perf stat exit $smi_status"
      priv "$TSC_INSTANCE_TOOL" stop "$TRACEFS" "$TSC_INSTANCE" >"$tsc_txt" 2>"$tmp/tsc_stop.err" ||
        tracer_problem+="${tracer_problem:+, }the TSC instance was not read ($(cat "$tmp/tsc_stop.err"))"
      inst_out=$(priv "$TSC_INSTANCE_TOOL" stats "$TRACEFS" "$TSC_INSTANCE" "$rt_cpu" 2>&1); inst_status=$?
      { echo; echo "--- TSC instance stats (CPU $rt_cpu, exit $inst_status) ---"; echo "$inst_out"; } >>"$note"
      if [ "$inst_status" -ne 0 ]; then
        n_over="unknown (the stats read exited $inst_status)"
      elif ! n_over=$(loss_sum "$inst_out" "$rt_cpu"); then
        n_over="unknown ($n_over)"
      fi
      echo "tsc instance overruns: $n_over (overrun, commit overrun and dropped events)" >>"$note"
      if ! [[ $n_over =~ ^[0-9]+$ ]]; then
        tracer_problem+="${tracer_problem:+, }the TSC instance's loss counts: $n_over"
      elif [ "$n_over" -gt 0 ]; then
        tracer_problem+="${tracer_problem:+, }the TSC instance lost $n_over events: the capture is invalid"
      fi
      # Its evidence is read; a failed removal is a cleanup failure, which
      # ends the session after this run (status 3 if it would succeed).
      remove_tsc_instance
      # The fair server's probes: their hits and misses (kprobe_profile)
      # right before and after the probe; a read that fails, or an event
      # missing from it, leaves the probes' coverage unknown.
      { echo; echo "--- fair-server probes (hits misses) right before the probe (exit $fs_b_status) ---"
        echo "$fs_before"
        echo "--- fair-server probes (hits misses) right after the probe (exit $fs_a_status) ---"
        echo "$fs_after"; } >>"$note"
      if [ "$fs_b_status" -ne 0 ] || [ "$fs_a_status" -ne 0 ]; then
        tracer_problem+="${tracer_problem:+, }the fair server's probe counts were not read"
      else
        awk 'NR == FNR { h[$1] = $2; m[$1] = $3; next }
             { dh = $2 - h[$1]; dm = $3 - m[$1]; hits = hits sep $1 " " dh; sep = ", "; miss += dm }
             END { printf "fair-server probe hits during the run: %s; misses during the run: %d\n", hits, miss }' \
          <(echo "$fs_before") <(echo "$fs_after") >>"$note"
      fi
    fi
    local trace_status usbmon_status=0 used_before
    used_before=$(stage_usage)
    stop_tracer trace "$trace_job"; trace_status=$?; trace_job=
    { echo; echo "--- trace-cmd (exit $trace_status) ---"; cat "$tmp/trace.out"; } >>"$note"
    [ "$trace_status" -eq 0 ] && [ -s "$dat" ] ||
      tracer_problem+="${tracer_problem:+, }trace-cmd exit $trace_status"
    filled=$(stage_assembly_note "$used_before" "$dat" "$note")
    [ -n "$filled" ] && tracer_problem+="${tracer_problem:+, }$filled"
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
      # The worker's samples as text, symbols resolved as root: the sampled
      # instruction pointers alone, then with their call chains; and perf's
      # own counts of samples, lost samples and throttling.
      wtid=$(sed -n 's/^worker_tid = //p' "$sb.summary.toml" 2>/dev/null)
      if [ -n "$wtid" ] && [ "$perf_clean" -eq 1 ]; then
        local stacks_status
        priv "$PERF" script -i "$perfdat" --ns -G -F comm,tid,cpu,time,ip,sym,dso --tid "$wtid" \
          >"$perftxt" 2>"$tmp/perf-script.err"
        script_status=$?
        priv "$PERF" script -i "$perfdat" --ns -F comm,tid,cpu,time,ip,sym,dso --tid "$wtid" \
          >"$perfstacks" 2>>"$tmp/perf-script.err"
        stacks_status=$?
        echo "perf script --tid $wtid (exit $script_status): $(wc -l <"$perftxt") samples; with call chains (exit $stacks_status): $(grep -cE '\[[0-9]+\] +[0-9]+\.[0-9]+:' "$perfstacks") samples" >>"$note"
        # perf report lists only nonzero counts unless --no-skip-empty: the
        # aggregated counts, each named, and "not reported" for one it omits.
        local stats_status key n counts=
        priv "$PERF" report -i "$perfdat" --stats --no-skip-empty >"$tmp/perf-stats.out" 2>&1
        stats_status=$?
        for key in SAMPLE LOST LOST_SAMPLES THROTTLE; do
          n=$(awk -v k="$key" '/^Aggregated stats:/ {a = 1; next} a && /stats:$/ {exit}
                a && $1 == k && $2 == "events:" {print $3; exit}' "$tmp/perf-stats.out")
          counts+="${counts:+, }$key ${n:-not reported}"
        done
        echo "perf stats: $counts (perf report --stats --no-skip-empty, exit $stats_status)" >>"$note"
        [ "$script_status" -eq 0 ] && [ -s "$perftxt" ] && [ "$stacks_status" -eq 0 ] && [ -s "$perfstacks" ] ||
          tracer_problem+="${tracer_problem:+, }perf script exit $script_status/$stacks_status"
      elif [ "$probe_status" -eq 0 ]; then
        tracer_problem+="${tracer_problem:+, }no worker thread for perf script"
      fi
    fi
    own_outputs
  fi
  # The guard, last: its sampled peak, and any stop it made.
  stage_guard_stop "$note"
  [ -n "$guard_stop" ] && tracer_problem+="${tracer_problem:+, }$guard_stop"
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
  # Every writer has stopped: the stage is published, verified and released
  # before the capture is judged from the published copies (from the kept
  # stage when publication failed, so the run's own verdict is not lost).
  stage_close || :
  pub=$csv
  [ "$stage_published" -eq 1 ] || pub=$sb
  if [ "$traced" -eq 1 ]; then
    local stat_out stat_status n_trace
    # The buffers' losses, as trace-cmd recorded them at the end.
    stat_out=$(priv "$TRACE_CMD" report --stat -i "$pub.trace.dat" 2>&1); stat_status=$?
    { echo "--- trace-cmd report --stat (exit $stat_status) ---"; echo "$stat_out"; } >>"$note"
    if [ "$stat_status" -ne 0 ]; then
      n_trace="unknown (trace-cmd report --stat exited $stat_status)"
    elif ! n_trace=$(loss_sum "$stat_out" "$online"); then
      n_trace="unknown ($n_trace)"
    fi
    echo "trace overruns: $n_trace (overrun, commit overrun and dropped events over all CPUs)" >>"$note"
    if ! [[ $n_trace =~ ^[0-9]+$ ]]; then
      tracer_problem+="${tracer_problem:+, }the trace's loss counts: $n_trace"
    elif [ "$n_trace" -gt 0 ]; then
      tracer_problem+="${tracer_problem:+, }the trace lost $n_trace events: the capture is invalid"
    fi
  fi
  { echo; echo "--- IRQ placement after ---"; irq_state; } >>"$note"
  { echo; echo "--- idle counters, CPUs $idle_cpus (read-only, right before and after the probe) ---"
    cat "$tmp/idle_before.txt" "$tmp/idle_after.txt"
    python3 "$IDLE_TOOL" delta "$tmp/idle_before.txt" "$tmp/idle_after.txt" 2>&1; } >>"$note"
  { echo; echo "--- memory and writeback counters (system-wide, read-only; $VM_TOOL), right before and after the probe ---"
    cat "$tmp/vm_before.txt" "$tmp/vm_after.txt"
    python3 "$VM_TOOL" delta "$tmp/vm_before.txt" "$tmp/vm_after.txt" 2>&1; } >>"$note"
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
  if [ -n "$placement_problem" ]; then
    { echo; echo "verdict: placement failure ($placement_problem)"; } >>"$note"
    echo "trace_session: $placement_problem: session stopped. See $note"
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
  uv run --project tools "$SUMMARY_TOOL" "$pub" "${check_args[@]}" 2>&1 | tee -a "$note"
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
  if [ "$stage_failed" -eq 1 ]; then
    echo "trace_session: the run's staged evidence was not published and verified, or its stage not released: session stopped. See $note"
    outcome=stop
  fi
}

# The probes' controls (--fs-controls): 0 when they pass (an inconclusive
# negative control is reported, not a stop), 1 when the session must stop.
run_controls() {
  local base="$outdir/fs-controls-$date_tag-$attempt" note dat ctl ctl_args prof_b prof_a pb pa ctl_status
  local st stat_out stat_status n_loss problem='' an_status e f sbase pubb ctl_dur used_before filled
  note=$base.run.txt; dat=$base.trace.dat; ctl=$base.controls.txt
  echo
  echo "=== fair-server controls on CPU $fs_controls_cpu (supervisor on CPU $ctl_sup): $(basename "$base")"
  if [ -e "$note" ] || [ -e "$dat" ] || [ -e "$ctl" ]; then
    echo "trace_session: the controls of this attempt were attempted earlier: controls are never reused; session stopped"
    return 1
  fi
  # The controls' streams go to their own stage (their record stays here);
  # their budget's duration is theirs: three phases, two settles and 10 s.
  stage_name
  sbase="$stage_dir/fs-controls-$date_tag-$attempt"
  dat=$sbase.trace.dat; ctl=$sbase.controls.txt
  ctl_dur=$(awk -v p="$FS_CONTROL_PHASE_S" -v s="$FS_CONTROL_SETTLE_S" 'BEGIN {print 3 * p + 2 * s + 10}')
  ctl_args=("${trace_args[@]}" -e timer:hrtimer_start)
  while read -r e f; do
    ctl_args+=(-e "$FS_GROUP:$e" -f "$f")
  done < <(python3 - "$fs_controls_cpu" <<'EOF'
import sys
sys.path.insert(0, "tools")
import fair_server
for e, f in fair_server.filters(int(sys.argv[1])):
    print(e, f)
EOF
)
  {
    echo "rtctrl fair-server probe controls (rehearsal)"
    echo "started: $(date -Is)"
    echo "commit: $commit"
    echo "control CPU: $fs_controls_cpu; supervisor CPU: $ctl_sup; tracers on CPUs $ctl_tracer_cpus"
    printf 'controls: python3 %s run --cpu %s --supervisor-cpu %s --phase-s %s --settle-s %s --output %q\n' \
      "$FS_CONTROL_TOOL" "$fs_controls_cpu" "$ctl_sup" "$FS_CONTROL_PHASE_S" "$FS_CONTROL_SETTLE_S" "$ctl"
    printf 'trace: stdbuf -oL -eL %s' "$TRACE_CMD"; printf ' %q' "${ctl_args[@]}"; printf ' --temp %q -o %q\n' "$sbase.trace-tmp" "$dat"
    cat "$tmp/session.txt"
  } >"$note"
  last_note=$note
  if ! stage_open controls "$ctl_dur" 0 "fs-controls-$date_tag-$attempt" "$note"; then
    echo "trace_session: the controls' staging failed: session stopped. See $note" | tee -a "$note"
    echo "verdict: not run (staging)" >>"$note"
    return 1
  fi
  mkdir "$sbase.trace-tmp"
  run_outputs=("$dat")
  tracer_cpus=$ctl_tracer_cpus
  start_tracer trace stdbuf -oL -eL "$TRACE_CMD" "${ctl_args[@]}" --temp "$sbase.trace-tmp" -o "$dat"
  tracer_cpus=$other_cpus
  if ! wait_line trace "$trace_job" "Hit Ctrl^C to stop recording" 60; then
    stop_all
    { echo "--- trace-cmd did not start ---"; cat "$tmp/trace.out"; } | tee -a "$note"
    echo "verdict: not run (tracer)" >>"$note"
    return 1
  fi
  if ! stage_temp_check "$sbase.trace-tmp" "$(basename "$dat")" "$note" || ! stage_kernel_check 0 "$note"; then
    stop_all
    echo "verdict: not run (staging)" >>"$note"
    return 1
  fi
  tracer_cpus=$ctl_tracer_cpus
  if ! stage_guard_start "$note"; then
    tracer_cpus=$other_cpus
    stop_all
    echo "verdict: not run (staging)" >>"$note"
    return 1
  fi
  tracer_cpus=$other_cpus
  prof_b=$(priv "$FS_PROBE_TOOL" profile "$TRACEFS" "$FS_GROUP" "$tmp/fs_defs.txt" 2>&1); pb=$?
  python3 "$FS_CONTROL_TOOL" run --cpu "$fs_controls_cpu" --supervisor-cpu "$ctl_sup" \
    --phase-s "$FS_CONTROL_PHASE_S" --settle-s "$FS_CONTROL_SETTLE_S" --output "$ctl" 2>&1 | tee -a "$note"
  ctl_status=${PIPESTATUS[0]}
  prof_a=$(priv "$FS_PROBE_TOOL" profile "$TRACEFS" "$FS_GROUP" "$tmp/fs_defs.txt" 2>&1); pa=$?
  used_before=$(stage_usage)
  stop_tracer trace "$trace_job"; st=$?; trace_job=
  { echo; echo "--- trace-cmd (exit $st) ---"; cat "$tmp/trace.out"; } >>"$note"
  [ "$st" -eq 0 ] && [ -s "$dat" ] || problem+="${problem:+, }trace-cmd exit $st"
  filled=$(stage_assembly_note "$used_before" "$dat" "$note")
  [ -n "$filled" ] && problem+="${problem:+, }$filled"
  stage_guard_stop "$note"
  [ -n "$guard_stop" ] && problem+="${problem:+, }$guard_stop"
  own_outputs
  # Every writer has stopped: publish, verify and release, then judge the
  # published copies (the kept stage's if publication failed).
  stage_close || :
  pubb=$base
  [ "$stage_published" -eq 1 ] || pubb=$sbase
  stat_out=$(priv "$TRACE_CMD" report --stat -i "$pubb.trace.dat" 2>&1); stat_status=$?
  { echo "--- trace-cmd report --stat (exit $stat_status) ---"; echo "$stat_out"; } >>"$note"
  if [ "$stat_status" -ne 0 ]; then
    n_loss="unknown (trace-cmd report --stat exited $stat_status)"
  elif ! n_loss=$(loss_sum "$stat_out" "$online"); then
    n_loss="unknown ($n_loss)"
  fi
  echo "trace overruns: $n_loss (overrun, commit overrun and dropped events over all CPUs)" >>"$note"
  if ! [[ $n_loss =~ ^[0-9]+$ ]]; then
    problem+="${problem:+, }the trace's loss counts: $n_loss"
  elif [ "$n_loss" -gt 0 ]; then
    problem+="${problem:+, }the trace lost $n_loss events: the capture is invalid"
  fi
  { echo; echo "--- fair-server probes (hits misses) before the controls (exit $pb) ---"; echo "$prof_b"
    echo "--- fair-server probes (hits misses) after the controls (exit $pa) ---"; echo "$prof_a"; } >>"$note"
  if [ "$pb" -ne 0 ] || [ "$pa" -ne 0 ]; then
    problem+="${problem:+, }the fair server's probe counts were not read"
  else
    awk 'NR == FNR { h[$1] = $2; m[$1] = $3; next }
         { dh = $2 - h[$1]; dm = $3 - m[$1]; hits = hits sep $1 " " dh; sep = ", "; miss += dm }
         END { printf "fair-server probe hits during the controls: %s; misses during the controls: %d\n", hits, miss }' \
      <(echo "$prof_b") <(echo "$prof_a") >>"$note"
  fi
  [ "$ctl_status" -eq 0 ] || problem+="${problem:+, }the controls tool exited $ctl_status"
  [ "$stage_failed" -eq 0 ] || problem+="${problem:+, }the controls' staged evidence was not published and verified, or their stage not released"
  if [ -n "$problem" ]; then
    { echo; echo "verdict: tracer failure ($problem)"; } >>"$note"
    echo "trace_session: fair-server controls: $problem: session stopped. See $note"
    return 1
  fi
  { echo; echo "--- $FS_TOOL controls ---"; } >>"$note"
  TRACE_CMD="$TRACE_CMD" python3 "$FS_TOOL" controls --controls "$pubb.controls.txt" --record "$note" --trace-dat "$pubb.trace.dat" 2>&1 |
    tee -a "$note"
  an_status=${PIPESTATUS[0]}
  case "$an_status" in
    0) { echo; echo "verdict: controls pass"; } >>"$note"; return 0 ;;
    1) { echo; echo "verdict: controls fail"; } >>"$note"
       echo "trace_session: the fair-server controls failed: session stopped. See $note"
       return 1 ;;
    *) { echo; echo "verdict: analysis error (exit $an_status)"; } >>"$note"
       echo "trace_session: the controls' analysis failed (exit $an_status): session stopped. See $note"
       return 1 ;;
  esac
}

if [ -n "$fs_controls_cpu" ]; then
  current="fair-server controls"; current_mode=controls
  if ! run_controls; then
    add_result "fair-server controls" controls "STOPPED the session (see its .run.txt)"
    current=
    finish
    print_table
    exit 1
  fi
  add_result "fair-server controls" controls "$(grep -m 1 '^verdict:' "$outdir/fs-controls-$date_tag-$attempt.run.txt")"
  current=
fi

for spec in "${run_list[@]}"; do
  IFS=: read -r p r c v <<<"$spec"
  for step in "${step_list[@]}"; do
    for placement in "${pl_list[@]}"; do
    label=$spec
    [ -n "$step" ] && label="$spec@${step%%:*}ms#${step##*:}"
    [ -n "$placement" ] && label+=" obs-$placement"
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
      if [ "$tsc_owed" -eq 1 ]; then
        echo "trace_session: the trace instance was not removed: no further run (the cleanup is retried at exit)" >&2
        finish
        print_table
        exit 0
      fi
    done
    done
  done
done
placement=
finish
print_table
