#!/usr/bin/env bash
# tools/trace_session.sh over dxl_emu, with stand-in tracers (trace-cmd,
# tcpdump, bpftrace, modprobe) and no sudo: the refusals, the
# untraced/traced pairs and their records, the mono trace clock, the
# usbmon device filter, the clock pairs, resuming without a retry, the
# load (a stand-in stress-ng) starting after the recorders and pinned off
# the excluded CPUs, the stops when the fair server is not read, a
# tracer does not start, the trace clock is not mono, a recorder or the
# load ends before the probe does, or the load fails, an interrupted
# session (Ctrl-C or SIGTERM) giving its outputs back, and the adapter
# experiment's latency steps (a stand-in latency tool): the steps in order,
# their checks before and after each run, the stops when the adapter does
# not hold a step's value, and the restoration on every exit; and the
# profiled mode (a stand-in perf and a stand-in probe replaying a real run,
# since rt-* conditions need a real-time priority limit the test cannot
# have): its recorders, the worker's samples, and the stops when perf does
# not start or ends early; the refusal while an unrelated batch workload
# runs; and the timer mode (a stand-in for the TSC instance's tool, the real
# TSC sampler): its events with their CPU filters, the instance inside the
# trace, the sampler's pairs, the SMI count's anchors, the losses counted in
# the record, the refusals for a missing event and a stale instance, the
# instance's removal on every exit (status 3 when it fails), the timekeeping
# recorder's order around everything it covers, its files on the stage, its
# failures and interruption (a stand-in for its tool, and tools/tk_recorder.sh
# itself in tools/test_tk_record.py), and the exerciser of the rehearsal; and
# every run's staging (stand-in mount,
# findmnt and umount, a stand-in meminfo): each run's records (budget,
# verification, marker, temporary files, kernel buffers, guard, publication,
# release, ledger and inventory, memory counters), the refusals of a stage
# that is not a private noswap tmpfs or cannot be mounted, and of a budget
# MemAvailable cannot hold; a stage that fills, the guard's memory stop,
# trace-cmd's temporary files off the stage and buffers above the budget; a
# failed publication (status 3, and 130 when interrupted) and its retry; a
# failed unmount (status 3), the next session's refusal while that stage
# remains, and its retried release. Functional only: a
# pty answers instantly and the tracers record nothing real, so no verdict
# is asserted. Real-time conditions need a real-time priority limit, so
# they stay out of this test (the plan is checked with --dry-run).
set -eu

EMU="$1"
PROBE="$2"
ROOT="$3"
INSPECT="$4"
SAMPLER="${5:-}"  # tsc_mono_pairs (x86-64 builds); without it the timer mode is not run
LINK="$ROOT/tty"
OUT="$ROOT/out"
FAKE="$ROOT/fake"
DATE=20000101

rm -rf "$ROOT"
mkdir -p "$OUT" "$FAKE" "$ROOT/tracefs" "$ROOT/usbdev"
"$EMU" --link "$LINK" --firmware 47 >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }
# The bounded read refuses the factory Return Delay Time (250); the arm's
# servos have 0.
for id in 2 3 4 5 6 7 8 9; do
  "$INSPECT" --port "$LINK" write "$id" return_delay_time 0 >/dev/null
done

fail() { echo "$1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }

# Stand-ins. Each logs its arguments; the recorders run until SIGINT and
# then write their file, as the real ones do. Like the real trace-cmd on a
# pipe, the trace-cmd stand-in holds its readiness line until it exits
# unless stdbuf made its output line-buffered.
cat >"$FAKE/trace-cmd" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.trace-cmd"
[ "$1" = -h ] && { echo "trace-cmd version 0.0 (stand-in)"; exit 1; }
# report --stat: the buffers' losses on every online CPU
# (FAKE_TRACE_OVERRUN on the first; FAKE_STATS=fail fails the read, nocpu
# leaves the last CPU out, bad prints a counter that is not a number); list
# -e: the events (all the timer mode needs, less FAKE_MISSING_EVENT), or a
# format.
# extract -B: the timekeeping recorder's record (FAKE_TK_EXTRACT=fail, none
# written); report -t -i of it: its rows.
if [ "$1" = extract ]; then
  [ "${FAKE_TK_EXTRACT:-}" = fail ] && { echo "stand-in: cannot extract" >&2; exit 1; }
  while [ $# -gt 0 ]; do case "$1" in -o) out=$2; shift ;; esac; shift; done
  echo "stand-in record" >"$out"
  exit 0
fi
if [ "$1" = report ] && [ "$2" = -t ] && [[ ${4:-} == *.tk.dat ]]; then
  echo "cpus=4"
  echo "   tk_record-1   [001] d..1.  1.000000000: ntp_adjtimex: (x) tkid=0 modes=0"
  exit 0
fi
if [ "$1" = report ]; then
  [ "${FAKE_STATS:-}" = fail ] && { echo "stand-in: cannot read the buffer statistics" >&2; exit 1; }
  cpus=()
  IFS=, read -r -a parts </sys/devices/system/cpu/online
  for part in "${parts[@]}"; do
    if [[ $part == *-* ]]; then for ((c = ${part%-*}; c <= ${part#*-}; c++)); do cpus+=("$c"); done
    else cpus+=("$part"); fi
  done
  [ "${FAKE_STATS:-}" = nocpu ] && unset 'cpus[-1]'
  first=1
  for c in "${cpus[@]}"; do
    over=0
    [ "$first" = 1 ] && over=${FAKE_TRACE_OVERRUN:-0}
    [ "$first" = 1 ] && [ "${FAKE_STATS:-}" = bad ] && over="n/a"
    printf 'CPU: %s\nentries: 10\noverrun: %s\ncommit overrun: 0\nbytes: 4096\ndropped events: 0\nread events: 10\n\n' "$c" "$over"
    first=0
  done
  exit 0
fi
if [ "$1" = list ]; then
  if [ $# -ge 3 ]; then echo "name: $3 (stand-in format)"; exit 0; fi
  for e in timer:hrtimer_start timer:hrtimer_cancel timer:hrtimer_setup timer:hrtimer_expire_entry \
           msr:write_msr power:cpu_idle power:cpu_idle_miss nmi:nmi_handler raw_syscalls:sys_enter \
           raw_syscalls:sys_exit exceptions:page_fault_user exceptions:page_fault_kernel \
           sched:sched_process_fork sched:sched_process_exec sched:sched_process_exit \
           irq_vectors:local_timer_entry sched:sched_switch syscalls:sys_enter_clock_nanosleep \
           syscalls:sys_exit_clock_nanosleep syscalls:sys_enter_nanosleep syscalls:sys_exit_nanosleep; do
    [ "$e" = "${FAKE_MISSING_EVENT:-}" ] || echo "$e"
  done
  # The fair server's probe events, once the stand-in group exists.
  if [ -e "$FAKE_LOG.fs_group_exists" ]; then
    for e in rtfs_dl_pick rtfs_dl_pick_ret rtfs_fs_pick rtfs_fs_pick_ret rtfs_set_next rtfs_srv_start rtfs_srv_stop rtfs_srv_timer; do
      echo "rtctrl_fs:$e"
    done
  fi
  exit 0
fi
[ "${FAKE_TRACE_FAIL:-0}" = 1 ] && { echo "stand-in trace-cmd: failing as asked"; exit 1; }
python3 -c 'import os; print("affinity", ",".join(map(str, sorted(os.sched_getaffinity(0)))))' >>"$FAKE_LOG.trace-affinity"
out=; clock=local; temp=; bkb=0
while [ $# -gt 0 ]; do
  case "$1" in -o) out=$2; shift ;; -C) clock=$2; shift ;; --temp) temp=$2; shift ;; -b) bkb=$2; shift ;; esac
  shift
done
# Its per-CPU temporary file in --temp's directory (FAKE_IGNORE_TEMP: beside
# the output instead), and its buffers sized as the kernel sizes them (the
# stand-in ring, below). FAKE_DROP_MEM: MemAvailable falls once it
# records (the staging guard's memory stop); FAKE_TRACE_FILL: its temporary
# file fills the stage's modeled capacity but 1 MiB (the guard's space stop;
# sparse, removed at its stop as trace-cmd removes its temporary files).
tdir=${temp:-$(dirname "$out")}
[ "${FAKE_IGNORE_TEMP:-0}" = 1 ] && tdir=$(dirname "$out")
: >"$tdir/$(basename "$out").cpu0"
if [ "${FAKE_TRACE_FILL:-0}" = 1 ]; then
  cap=$("$FINDMNT" -n -b -o FSTYPE,SIZE,OPTIONS --mountpoint "$(dirname "$tdir")" | awk '{print $2}')
  truncate -s $((cap - 1048576)) "$tdir/$(basename "$out").cpu0"
fi
"$(dirname "$0")/ring" "$TRACEFS" "$bkb"
[ -n "${FAKE_DROP_MEM:-}" ] && sed -i 's/^MemAvailable:.*/MemAvailable:       1000 kB/' "$TRACE_SESSION_MEMINFO"
[ -n "${FAKE_TRACE_CLOCK:-}" ] && clock=$FAKE_TRACE_CLOCK
case "$clock" in
  mono) echo "local global counter uptime perf [mono] mono_raw boot tai x86-tsc" >"$TRACEFS/trace_clock" ;;
  *) echo "[local] global counter uptime perf mono mono_raw boot tai x86-tsc" >"$TRACEFS/trace_clock" ;;
esac
stop=0
trap 'stop=1' INT
held=
if [ "${_STDBUF_O:-}" = L ]; then
  echo "Hit Ctrl^C to stop recording"
else
  held="Hit Ctrl^C to stop recording"
fi
echo "trace-ready" >>"$FAKE_LOG.events"
# FAKE_TRACE_LIFE: end on its own after that many seconds, successfully.
end=$(( $(date +%s) + ${FAKE_TRACE_LIFE:-100000} ))
while [ "$stop" -eq 0 ] && [ "$(date +%s)" -lt "$end" ]; do sleep 0.05; done
rm -f "$tdir/$(basename "$out").cpu0"
if [ "${FAKE_TRACE_ENOSPC:-0}" = 1 ]; then
  echo "partial stand-in trace" >"$out"
  echo "trace-cmd: No space left on device"
  exit 1
fi
echo "stand-in trace data" >"$out"
[ -n "$held" ] && echo "$held"
echo "CPU0 data recorded at offset=0x0"
echo "overrun: 0"
exit 0
EOF
cat >"$FAKE/tcpdump" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.tcpdump"
[ "$1" = --version ] && { echo "tcpdump version 0.0 (stand-in)"; exit 0; }
out=; iface=
while [ $# -gt 0 ]; do
  case "$1" in -w) out=$2; shift ;; -i) iface=$2; shift ;; -Z|-s) shift ;; esac
  shift
done
stop=0
trap 'stop=1' INT
echo "tcpdump: listening on $iface, link-type USB_LINUX_MMAPPED (stand-in)" >&2
echo "usbmon-ready" >>"$FAKE_LOG.events"
end=$(( $(date +%s) + ${FAKE_USBMON_LIFE:-100000} ))
while [ "$stop" -eq 0 ] && [ "$(date +%s)" -lt "$end" ]; do sleep 0.05; done
python3 -c 'import struct, sys; open(sys.argv[1], "wb").write(struct.pack("<IHHiIII", 0xa1b2c3d4, 2, 4, 0, 0, 262144, 220))' "$out"
echo "0 packets captured" >&2
exit 0
EOF
cat >"$FAKE/bpftrace" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.bpftrace"
[ "$1" = --version ] && { echo "bpftrace v0.0 (stand-in)"; exit 0; }
[ "${FAKE_BPF_FAIL:-0}" = 1 ] && { echo "stand-in bpftrace: failing as asked"; exit 1; }
echo "Attached 3 probes"
for ((c = $(nproc) - 1; c >= 0; c--)); do
  echo "cpu $c fair_server_pick_task runtime_ns 50000000 period_ns 1000000000 deadline_ns 1000000000 dl_server 1 dl_defer 1"
done
EOF
cat >"$FAKE/stress-ng" <<'EOF'
#!/usr/bin/env bash
# Runs out its --timeout (or FAKE_STRESS_LIFE seconds) and exits
# FAKE_STRESS_EXIT (default 0), logging its start and its CPUs.
echo "load" >>"$FAKE_LOG.events"
python3 -c 'import os; print("affinity", ",".join(map(str, sorted(os.sched_getaffinity(0)))))' >>"$FAKE_LOG.stress-ng"
life=
while [ $# -gt 0 ]; do
  case "$1" in --timeout) life=${2%s}; shift ;; esac
  shift
done
sleep "${FAKE_STRESS_LIFE:-$life}"
exit "${FAKE_STRESS_EXIT:-0}"
EOF
cat >"$FAKE/modprobe" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.modprobe"
EOF
# The adapter's latency timer, kept in $FAKE_LOG.latency-state as "sysfs
# device low_latency". set stores the value; the adapter keeps 1 with the
# low-latency flag set, and its previous value for a request of
# FAKE_LATENCY_IGNORE; a set of FAKE_LATENCY_FAIL_SET fails and changes
# nothing.
cat >"$FAKE/latency" <<'EOF'
#!/usr/bin/env bash
state="$FAKE_LOG.latency-state"
echo "$*" >>"$FAKE_LOG.latency"
read -r s d l <"$state"
if [ "$1" = set ] && [ "$3" = "${FAKE_LATENCY_FAIL_SET:-}" ]; then
  echo "tty $2 sysfs $s device $d low_latency $l adapter 0403:6014 bcdDevice 0900 channel 1"
  echo "latency_timer: cannot write the latency timer (stand-in failing as asked)" >&2
  exit 7
fi
if [ "$1" = set ]; then
  s=$3
  if [ "$l" = 1 ]; then d=1; elif [ "$3" != "${FAKE_LATENCY_IGNORE:-}" ]; then d=$3; fi
  echo "$s $d $l" >"$state"
fi
echo "tty $2 sysfs $s device $d low_latency $l adapter 0403:6014 bcdDevice 0900 channel 1"
if [ "$1" = set ] && { [ "$s" != "$3" ] || [ "$d" != "$3" ] || [ "$l" != 0 ]; }; then
  echo "latency_timer: requested $3 ms, but sysfs reads $s, the adapter $d" >&2
  exit 1
fi
exit 0
EOF
# perf: record writes its file and runs until SIGINT, then reports and dies
# of that signal, as perf does; script prints one sample of the --tid given
# (with its call chain unless -G), and report --stats its counts as perf 7
# does: the zero counts only with --no-skip-empty, and the sample count again
# per event after the aggregated ones.
cat >"$FAKE/perf" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.perf"
case "$1" in
  record)
    out=
    while [ $# -gt 0 ]; do case "$1" in -o) out=$2; shift ;; esac; shift; done
    [ "${FAKE_PERF_FAIL:-0}" = 1 ] && { echo "stand-in perf: failing as asked" >&2; exit 1; }
    echo "PERFILE2 stand-in header" >"$out"
    stop=0
    trap 'stop=1' INT
    end=$(( $(date +%s) + ${FAKE_PERF_LIFE:-100000} ))
    while [ "$stop" -eq 0 ] && [ "$(date +%s)" -lt "$end" ]; do sleep 0.05; done
    echo "stand-in samples" >>"$out"
    echo "[ perf record: Captured and wrote 0.010 MB $out (12 samples) ]" >&2
    if [ "$stop" -eq 1 ]; then trap - INT; kill -INT $$; fi
    exit 0 ;;
  script)
    tid=; flat=0
    while [ $# -gt 0 ]; do case "$1" in --tid) tid=$2; shift ;; -G) flat=1 ;; esac; shift; done
    if [ "$flat" -eq 1 ]; then
      echo "   probe $tid [000]  1000.000100000:  ffffffff81000000 do_nanosleep ([kernel.kallsyms])"
    else
      printf '   probe %s [000]  1000.000100000:\n\tffffffff81000000 do_nanosleep ([kernel.kallsyms])\n\t55d000001234 Probe::run (/x7_bus_timing)\n\n' "$tid"
    fi
    exit 0 ;;
  stat)  # -o FILE: a header, then an interval line every 10 ms until SIGINT
    out=
    while [ $# -gt 0 ]; do case "$1" in -o) out=$2; shift ;; esac; shift; done
    echo "# started on (stand-in)" >"$out"
    stop=0; trap 'stop=1' INT TERM
    t=0
    while [ "$stop" -eq 0 ]; do
      sleep 0.01; t=$((t + 1))
      printf '%d.%02d0000000,0,,msr/smi/,10000000,100.00,,\n' $((t / 100)) $((t % 100)) >>"$out"
    done
    exit 0 ;;
  report)
    empty=0
    for a in "$@"; do [ "$a" = --no-skip-empty ] && empty=1; done
    echo
    echo "Aggregated stats:"
    echo "               TOTAL events:         40"
    [ "$empty" -eq 1 ] && echo "                LOST events:          0  ( 0.0%)"
    [ "$empty" -eq 1 ] && echo "            THROTTLE events:          0  ( 0.0%)"
    echo "              SAMPLE events:         12  (30.0%)"
    [ "$empty" -eq 1 ] && echo "        LOST_SAMPLES events:          0  ( 0.0%)"
    echo "cpu_core/cycles/P stats:"
    echo "              SAMPLE events:         12"
    exit 0 ;;
esac
EOF
# A stand-in probe for the profiled runs: it replays a real run's log
# ($FAKE_PROBE_TEMPLATE, from the pairs below) for --duration seconds
# (FAKE_PROBE_OWN_TID: its summary names its own process as the worker, for
# a confined run's halfway check).
cat >"$FAKE/probe" <<'EOF'
#!/usr/bin/env bash
log=; dur=1
while [ $# -gt 0 ]; do
  case "$1" in --log) log=$2; shift ;; --duration) dur=$2; shift ;; esac
  shift
done
cp "$FAKE_PROBE_TEMPLATE" "$log"
cp "$FAKE_PROBE_TEMPLATE.summary.toml" "$log.summary.toml"
[ -z "${FAKE_PROBE_OWN_TID:-}" ] || sed -i "s/^worker_tid = .*/worker_tid = $$/" "$log.summary.toml"
echo "worker thread $(sed -n 's/^worker_tid = //p' "$log.summary.toml") (stand-in probe)"
sleep "$dur"
EOF
# In sudo's place (TRACE_SESSION_SUDO): logs each privileged command and
# runs it as the invoking user; its timestamp checks succeed.
cat >"$FAKE/sudo" <<'EOF'
#!/usr/bin/env bash
case "$1" in -v|-n) exit 0 ;; esac
echo "$*" >>"$FAKE_LOG.sudo"
exec "$@"
EOF
# The TSC instance's tool: a marker file stands for the instance
# (FAKE_TSC_REMOVE_FAIL: its removal fails; FAKE_TSC_STATS=fail: its stats
# read fails; FAKE_TSC_OVERRUN: its overrun count; FAKE_TSC_START: race, another session creates it first and
# the start exits 2; partial, the start creates it and fails, 4; unknown,
# it creates it and exits 1, a status that does not say;
# FAKE_TSC_EXISTS_FAIL: the existence checks that fail, see exists_answer).
# The timekeeping recorder's tool: a marker file stands for its instance and
# group, and FAKE_LOG.order orders its steps against the TSC instance's
# (FAKE_TK_START: race, partial, unknown, as FAKE_TSC_START, and
# unknown-none, a status that does not say after creating nothing;
# FAKE_TK_STATES=off: read-backs report a probe off, offend at the end only;
# FAKE_TK_OVERRUN; FAKE_TK_STATS=fail; FAKE_TK_REMOVE_FAIL;
# FAKE_TK_EXISTS_FAIL, as FAKE_TSC_EXISTS_FAIL).
cat >"$FAKE/tk_recorder" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.tk_recorder"
echo "tk $1" >>"$FAKE_LOG.order"
mark="$FAKE_LOG.tk_recorder_exists"
PROBES="ntp_adjtimex ntp_clear update_return update_entry"
case "$1" in
  start) [ -e "$mark" ] && exit 2
         case "${FAKE_TK_START:-}" in
           race) : >"$mark"; echo "tk_recorder: it exists (another session's)" >&2; exit 2 ;;
           partial) : >"$mark"; echo "tk_recorder: cannot enable" >&2; exit 4 ;;
           unknown) : >"$mark"; echo "sudo: a stand-in failure" >&2; exit 1 ;;
           unknown-none) echo "sudo: a stand-in failure" >&2; exit 1 ;;
         esac
         : >"$mark"
         mkdir -p "$TRACEFS/instances/$3/events"
         cp "$TRACEFS/events/header_page" "$TRACEFS/instances/$3/events/header_page"
         echo 4 >"$TRACEFS/instances/$3/buffer_subbuf_size_kb"
         "$(dirname "$0")/ring" "$TRACEFS/instances/$3" "$4"
         echo "timekeeping recorder $3: clock [mono], buffer $4 kB per CPU, events $PROBES" ;;
  states) n=$(grep -c . "$FAKE_LOG.tk_states" 2>/dev/null); echo x >>"$FAKE_LOG.tk_states"
         for p in $PROBES; do
           if [ "${FAKE_TK_STATES:-}" = off ] && [ "${n:-0}" -ge 1 ] && [ "$p" = update_entry ]; then echo "$p 0"; else echo "$p 1"; fi
         done ;;
  disable) ;;
  profile) for p in $PROBES; do echo "$p 10 0"; done ;;
  stats) [ "${FAKE_TK_STATS:-}" = fail ] && { echo "stand-in: cannot read the stats" >&2; exit 1; }
         n=0
         for c in "$TRACEFS/instances/$3"/per_cpu/cpu*; do
           ov=0; [ "$n" -eq 0 ] && ov=${FAKE_TK_OVERRUN:-0}
           printf 'CPU: %s\nentries: 1\noverrun: %s\ncommit overrun: 0\nbytes: 1\ndropped events: 0\n' "${c##*/cpu}" "$ov"
           n=$((n + 1))
         done
         echo "cpus: $n" ;;
  remove) [ "${FAKE_TK_REMOVE_FAIL:-0}" = 1 ] && { echo "stand-in: it remains" >&2; exit 1; }
         rm -f "$mark"; rm -rf "$TRACEFS/instances/$3" ;;
  exists) "$(dirname "$0")/exists_answer" "$mark" "$FAKE_LOG.tk_exists_calls" "${FAKE_TK_EXISTS_FAIL:-}" ;;
esac
EOF
# The stand-ins' checked existence: "present" or "absent", status 0. A call
# counted in the range FAIL (N, or N-M; calls from 1) fails as sudo does,
# status 1 and no answer, the status the helpers once gave for absence.
cat >"$FAKE/exists_answer" <<'EOF'
#!/usr/bin/env bash
mark=$1 calls=$2 f=$3
n=$(( $(cat "$calls" 2>/dev/null || echo 0) + 1 )); echo "$n" >"$calls"
if [ -n "$f" ] && [ "$n" -ge "${f%-*}" ] && { [ "$f" = "${f%-*}" ] || [ "$n" -le "${f#*-}" ]; }; then
  echo "sudo: a stand-in failure" >&2; exit 1
fi
if [ -e "$mark" ]; then echo present; else echo absent; fi
EOF
# A stand-in for dpkg-query, answering as dpkg does: the stand-in trace-cmd
# is the package trace-cmd at FAKE_TRACE_CMD_VERSION (the audited version by
# default), with its libraries; any other file is no package's. Its calls
# are counted (FAKE_LOG.dpkg_calls), and from call FAKE_DPKG_FAIL_FROM on it
# answers for trace-cmd too as for a file no package installed.
cat >"$FAKE/dpkg-query" <<'EOF'
#!/usr/bin/env bash
n=$(( $(cat "$FAKE_LOG.dpkg_calls" 2>/dev/null || echo 0) + 1 )); echo "$n" >"$FAKE_LOG.dpkg_calls"
case "$1" in
  -S) if { [ -n "${FAKE_DPKG_FAIL_FROM:-}" ] && [ "$n" -ge "$FAKE_DPKG_FAIL_FROM" ]; } ||
         [ "$2" != "$(realpath "$(dirname "$0")/trace-cmd")" ]; then
        echo "dpkg-query: no path found matching pattern $2" >&2; exit 1
      fi
      echo "trace-cmd: $2" ;;
  -W) shift 3; rc=0
      for p in "$@"; do
        case "$p" in
          trace-cmd) printf 'trace-cmd\t%s\tamd64\tii \n' "${FAKE_TRACE_CMD_VERSION:-3.3.3-1ubuntu2}" ;;
          libtraceevent1|libtraceevent1-plugin) printf '%s\t1:1.8.7-1\tii \n' "$p" ;;
          libtracefs1) printf '%s\t1.8.2-1ubuntu1\tii \n' "$p" ;;
          *) echo "dpkg-query: no packages found matching $p" >&2; rc=1 ;;
        esac
      done
      exit $rc ;;
  *) exit 2 ;;
esac
EOF
cat >"$FAKE/journalctl" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.journalctl"
[ "${FAKE_JOURNAL:-}" = fail ] && { echo "stand-in: no journal" >&2; exit 1; }
echo "[    1.000000] host kernel: tsc: Detected 3187.200 MHz TSC"
EOF
cat >"$FAKE/tsc_instance" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.tsc_instance"
echo "tsc $1" >>"$FAKE_LOG.order"
mark="$FAKE_LOG.tsc_instance_exists"
case "$1" in
  start) [ -e "$mark" ] && exit 2
         case "${FAKE_TSC_START:-}" in
           race) : >"$mark"; echo "tsc_instance: it exists (another session's)" >&2; exit 2 ;;
           partial) : >"$mark"; echo "tsc_instance: cannot set the clock" >&2; exit 4 ;;
           unknown) : >"$mark"; echo "sudo: a stand-in failure" >&2; exit 1 ;;
         esac
         : >"$mark"
         # Its own page header (FAKE_INSTANCE_PAYLOAD: another subbuffer
         # payload than the main array's) and buffers, sized as the kernel does.
         mkdir -p "$TRACEFS/instances/$3/events"
         ip=${FAKE_INSTANCE_PAYLOAD:-4080}
         sed "s/size:4080;/size:$ip;/" "$TRACEFS/events/header_page" >"$TRACEFS/instances/$3/events/header_page"
         echo $(( (ip + 16) / 1024 )) >"$TRACEFS/instances/$3/buffer_subbuf_size_kb"
         "$(dirname "$0")/ring" "$TRACEFS/instances/$3" "$5"
         echo "tsc instance $3: clock [x86-tsc], cpumask 1, buffer $5 kB per CPU, event irq_vectors:local_timer_entry" ;;
  stop) echo "# entries-in-buffer/entries-written: 1/1   #P:2"
        echo "          <idle>-0       [000] d.h1. 123456789: local_timer_entry: vector=236" ;;
  stats) [ "${FAKE_TSC_STATS:-}" = fail ] && { echo "stand-in: cannot read the stats" >&2; exit 1; }
         printf 'CPU: %s\nentries: 1\noverrun: %s\ncommit overrun: 0\ndropped events: 0\n' "$4" "${FAKE_TSC_OVERRUN:-0}" ;;
  remove) [ "${FAKE_TSC_REMOVE_FAIL:-0}" = 1 ] && { echo "stand-in: cannot remove" >&2; exit 1; }
          rm -f "$mark" ;;
  exists) "$(dirname "$0")/exists_answer" "$mark" "$FAKE_LOG.tsc_exists_calls" "${FAKE_TSC_EXISTS_FAIL:-}" ;;
esac
EOF
# The fair server's probe tool: a marker file stands for its group
# (FAKE_FS_DEFINE: fail, nothing created, 2; partial, created then failed,
# 4; unknown, created and a status that does not say, 1; race, another
# session's group made first, 2. FAKE_FS_REMOVE_FAIL: its removal fails;
# FAKE_FS_PROFILE=fail: its counts read fails; FAKE_FS_MISSES: misses on
# the first event after a run; FAKE_FS_EXISTS_FAIL, as FAKE_TSC_EXISTS_FAIL).
cat >"$FAKE/fs_probes" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.fs_probes"
mark="$FAKE_LOG.fs_group_exists"
names() { sed -n 's#^[pr]:rtctrl_fs/\([a-z_]*\) .*#\1#p' "$1"; }
case "$1" in
  define) [ -e "$mark" ] && exit 2
          case "${FAKE_FS_DEFINE:-}" in
            fail) echo "fair_server_probes: the kernel refused: (stand-in)" >&2; exit 2 ;;
            partial) : >"$mark"; echo "fair_server_probes: the kernel refused the third (stand-in)" >&2; exit 4 ;;
            unknown) : >"$mark"; echo "sudo: a stand-in failure" >&2; exit 1 ;;
            race) : >"$mark"; echo "fair_server_probes: it exists (another session's)" >&2; exit 2 ;;
          esac
          : >"$mark"
          echo "fair-server probes $3: $(names "$4" | wc -l) events defined ($(names "$4" | xargs))" ;;
  remove) [ "${FAKE_FS_REMOVE_FAIL:-0}" = 1 ] && { echo "stand-in: cannot remove" >&2; exit 1; }
          rm -f "$mark" ;;
  exists) "$(dirname "$0")/exists_answer" "$mark" "$FAKE_LOG.fs_exists_calls" "${FAKE_FS_EXISTS_FAIL:-}" ;;
  profile) [ "${FAKE_FS_PROFILE:-}" = fail ] && { echo "stand-in: cannot read kprobe_profile" >&2; exit 1; }
           n=$(( $(cat "$FAKE_LOG.fs_profile_calls" 2>/dev/null || echo 0) + 1 )); echo "$n" >"$FAKE_LOG.fs_profile_calls"
           first=1
           for e in $(names "$4"); do
             miss=0; [ "$first" = 1 ] && [ $((n % 2)) -eq 0 ] && miss=${FAKE_FS_MISSES:-0}
             echo "$e $((n * 10)) $miss"; first=0
           done ;;
esac
EOF
# The runs' staging: mount adds a line to a stand-in mount table that
# findmnt answers from, with the mount point's own owner and mode (logged)
# kept for its unmount; umount emulates a real one: the tmpfs's contents
# vanish and the mount point reappears with its own mode. FAKE_MOUNT_FAIL:
# the mount fails; FAKE_MOUNT_TYPE replaces tmpfs; FAKE_MOUNT_DROP leaves an
# option out; $FAKE_UMOUNT_FAIL or $FAKE_RMDIR_FAIL existing: the unmount or
# the mount point's removal fails. The guard and the usage see each stage
# with its bounded capacity (TRACE_STAGE_CAPACITY=findmnt).
cat >"$FAKE/mount" <<'EOF'
#!/usr/bin/env bash
target=${*: -1}
echo "$* (mount point $(stat -c %u:%g:%a "$target"))" >>"$FAKE_LOG.mount"
[ "${FAKE_MOUNT_FAIL:-0}" = 1 ] && { echo "mount: stand-in failing as asked" >&2; exit 32; }
opts=$4
size=$(sed -n 's/.*size=\([0-9]*\)m.*/\1/p' <<<"$opts")
[ -n "${FAKE_MOUNT_DROP:-}" ] && opts=$(tr , '\n' <<<"$opts" | grep -vx "$FAKE_MOUNT_DROP" | paste -sd,)
under=$(stat -c %a "$target")
chmod 700 "$target"
echo "$target ${FAKE_MOUNT_TYPE:-tmpfs} $((size * 1048576)) $opts $under" >>"$FAKE_MOUNTS"
EOF
cat >"$FAKE/findmnt" <<'EOF'
#!/usr/bin/env bash
target=${*: -1}
while read -r path fstype size opts under; do
  [ "$path" = "$target" ] && { echo "$fstype $size $opts"; exit 0; }
done <"$FAKE_MOUNTS"
exit 1
EOF
cat >"$FAKE/umount" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.umount"
[ -e "$FAKE_UMOUNT_FAIL" ] && { echo "umount: $1: target is busy" >&2; exit 32; }
line=$(grep "^$1 " "$FAKE_MOUNTS") || { echo "umount: $1: not mounted" >&2; exit 32; }
find "$1" -mindepth 1 -delete
chmod "${line##* }" "$1"
grep -v "^$1 " "$FAKE_MOUNTS" >"$FAKE_MOUNTS.new"; mv "$FAKE_MOUNTS.new" "$FAKE_MOUNTS"
EOF
cat >"$FAKE/rmdir" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.rmdir"
[ -e "$FAKE_RMDIR_FAIL" ] && { echo "rmdir: failed to remove '$1': Device or resource busy" >&2; exit 1; }
exec rmdir "$@"
EOF
# The stand-in kernel's ring-buffer sizing for one trace array ($1 its
# directory, $2 the KiB requested per CPU), as Linux 7.0 sizes it: n = max(2,
# ceil(B / p)) subbuffers of the array's page-header payload p, n p bytes
# shown per CPU (KiB, truncated), the total their sum. FAKE_CPU_SKEW: cpu0
# shows 3 KiB more and the next CPU 3 KiB less, the total unchanged;
# FAKE_BUFFER_TOTAL replaces the total.
cat >"$FAKE/ring" <<'EOF'
#!/usr/bin/env bash
exec python3 - "$1" "$2" <<'PY'
import os, pathlib, re, sys
d, kb = pathlib.Path(sys.argv[1]), int(sys.argv[2])
p = int(re.search(r"field: char data;\s*offset:\d+;\s*size:(\d+);", (d / "events" / "header_page").read_text())[1])
n = max(2, -(-kb * 1024 // p))
cpus = []
for part in open("/sys/devices/system/cpu/possible").read().strip().split(","):
    a, _, b = part.partition("-")
    cpus += range(int(a), int(b or a) + 1)
vals = {c: (n * p) >> 10 for c in cpus}
if os.environ.get("FAKE_CPU_SKEW") and len(cpus) > 1:
    vals[cpus[0]] += 3
    vals[cpus[1]] -= 3
for c, v in vals.items():
    (d / "per_cpu" / f"cpu{c}").mkdir(parents=True, exist_ok=True)
    (d / "per_cpu" / f"cpu{c}" / "buffer_size_kb").write_text(f"{v}\n")
(d / "buffer_total_size_kb").write_text(os.environ.get("FAKE_BUFFER_TOTAL", str(sum(vals.values()))) + "\n")
PY
EOF
chmod +x "$FAKE"/*
echo "[local] global counter uptime perf mono mono_raw boot tai x86-tsc" >"$ROOT/tracefs/trace_clock"
# The main array's page header as this kernel prints it (4 KiB subbuffers,
# 4,080 bytes of payload at offset 16), and its subbuffer size.
mkdir -p "$ROOT/tracefs/events" "$ROOT/tracefs/events/irq_vectors/local_timer_entry"
echo 1 >"$ROOT/tracefs/events/irq_vectors/local_timer_entry/enable"
printf '\tfield: u64 timestamp;\toffset:0;\tsize:8;\tsigned:0;\n\tfield: local_t commit;\toffset:8;\tsize:8;\tsigned:1;\n\tfield: int overwrite;\toffset:8;\tsize:1;\tsigned:1;\n\tfield: char data;\toffset:16;\tsize:4080;\tsigned:0;\n' \
  >"$ROOT/tracefs/events/header_page"
cp "$ROOT/tracefs/events/header_page" "$ROOT/header_page.good"
echo 4 >"$ROOT/tracefs/buffer_subbuf_size_kb"
: >"$ROOT/mounts"
# 64 GB of RAM, 44 GB available (this host on 2026-11-09).
meminfo() {  # MemAvailable in kB
  printf 'MemTotal:       64625280 kB\nMemFree:        1000000 kB\nMemAvailable:   %s kB\nDirty:          100 kB\nWriteback:      0 kB\n' "$1"
}
meminfo 43557304 >"$ROOT/meminfo"
# The excluded CPU 0's idle states (the idle counters' stand-in sysfs): state
# 2 without the optional above and below counters.
for st in 0:POLL:0:0 1:C1_ACPI:1:1 2:C2_ACPI:127:381; do
  IFS=: read -r i name lat res <<<"$st"
  d="$ROOT/cpu/cpu0/cpuidle/state$i"
  mkdir -p "$d"
  for kv in name:$name desc:stand-in latency:$lat residency:$res disable:0 usage:$((i + 10)) time:$((i * 100)) rejected:0; do
    echo "${kv#*:}" >"$d/${kv%%:*}"
  done
  [ "$i" = 2 ] || { echo 0 >"$d/above"; echo 0 >"$d/below"; }
done
printf '1\n' >"$ROOT/usbdev/busnum"
printf '9\n' >"$ROOT/usbdev/devnum"
printf '0403\n' >"$ROOT/usbdev/idVendor"
printf '6014\n' >"$ROOT/usbdev/idProduct"
printf '480\n' >"$ROOT/usbdev/speed"

export TRACE_SESSION_SUDO=none TRACEFS="$ROOT/tracefs" FAKE_LOG="$ROOT/fake"
export TRACE_CMD="$FAKE/trace-cmd" TCPDUMP="$FAKE/tcpdump"
export BPFTRACE="$FAKE/bpftrace" MODPROBE="$FAKE/modprobe" STRESS_NG="$FAKE/stress-ng"
export LATENCY_TOOL="$FAKE/latency" PERF="$FAKE/perf" TSC_INSTANCE_TOOL="$FAKE/tsc_instance"
export TK_RECORDER_TOOL="$FAKE/tk_recorder" JOURNALCTL="$FAKE/journalctl" TK_SETTLE_S=0.01 DPKG_QUERY="$FAKE/dpkg-query"
export TRACE_SESSION_BATCH_LIST=rtctrl-nobatch QUIET_HOST_STATE="$ROOT/qh.state"
export TRACE_SESSION_CPU_ROOT="$ROOT/cpu"
export MOUNT="$FAKE/mount" FINDMNT="$FAKE/findmnt" UMOUNT="$FAKE/umount" RMDIR="$FAKE/rmdir" FAKE_MOUNTS="$ROOT/mounts"
export FAKE_UMOUNT_FAIL="$ROOT/umount-fail" FAKE_RMDIR_FAIL="$ROOT/rmdir-fail" TRACE_STAGE_CAPACITY=findmnt
export TRACE_SESSION_STAGE_ROOT="$ROOT/stage" TRACE_SESSION_MEMINFO="$ROOT/meminfo"
export FS_PROBE_TOOL="$FAKE/fs_probes" FAIR_SERVER_BTF_RAW="$PWD/tests/data/btf_raw_fair_server.txt"
if [ -n "$SAMPLER" ]; then export TSC_SAMPLER="$SAMPLER"; fi
echo "4 4 0" >"$ROOT/fake.latency-state"

# The quiet-machine gate is lifted: the test suite runs in parallel. The
# tracers need a CPU outside the excluded set, so CPU 0 alone is excluded
# (a one-CPU machine cannot run this test); these runs use ordinary
# scheduling.
if [ "$(nproc)" -lt 2 ]; then
  echo "trace session script OK (not run: one CPU)"
  exit 0
fi
COMMON=(--probe "$PROBE" --port "$LINK" --outdir "$OUT" --duration 1
        --date "$DATE" --max-runnable 100000 --usb-device "$ROOT/usbdev"
        --rt-cpu 0 --exclude-cpus 0)
RUNS=current:100:idle:fast-motion,position:100:idle:full
base() { echo "$OUT/rate-trace-$1-$DATE-$2.csv"; }

# Without a confirmation and without a terminal, nothing starts.
if setsid tools/trace_session.sh "${COMMON[@]}" --runs "$RUNS" \
    </dev/null >"$ROOT/refuse.log" 2>&1; then
  fail "an unconfirmed session started" "$ROOT/refuse.log"
fi
grep -q 'no terminal to confirm the session' "$ROOT/refuse.log"
[ -z "$(ls -A "$OUT")" ] || fail "a refused session left files"

# Without the fair server's settings, nothing is sent.
if FAKE_BPF_FAIL=1 setsid tools/trace_session.sh "${COMMON[@]}" --yes \
    --runs "$RUNS" </dev/null >"$ROOT/nobpf.log" 2>&1; then
  fail "a session without the fair-server read started" "$ROOT/nobpf.log"
fi
grep -q "fair server's settings for CPU 0 were not read" "$ROOT/nobpf.log"
[ -z "$(ls -A "$OUT")" ] || fail "a session without the fair-server read left files"

# A log name already in the archive manifest is refused before anything is
# sent, since the output directory forgets the archived runs: through a
# sidecar's bare row, and through a directory-qualified entry and citation.
# Another attempt's name (10, not 1) is no collision. A missing manifest is
# refused too, and a dry run lists the name and goes on.
archived=rate-trace-current-100hz-idle-bounded-fast-motion-untraced-$DATE-1
other=rate-trace-current-100hz-idle-bounded-fast-motion-traced-$DATE-10
printf '| `%s.csv.run.txt` | archived run, session record | `0` |\n| `%s.csv` | another attempt | `0` |\n' \
  "$archived" "$other" >"$ROOT/manifest.md"
printf '| `profiled_torque_off/%s.csv.run.txt` | archived | `0` |\ncited as profiled_torque_off/%s.csv.trace.dat and %s.csv\n' \
  "$archived" "$archived" "$other" >"$ROOT/manifest-dirs.md"
for mf in "$ROOT/manifest.md" "$ROOT/manifest-dirs.md"; do
  if setsid tools/trace_session.sh "${COMMON[@]}" --yes --runs "$RUNS" --manifest "$mf" \
      </dev/null >"$ROOT/archived.log" 2>&1; then
    fail "a session reusing an archived log name ($mf) started" "$ROOT/archived.log"
  fi
  grep -q "^trace_session: $archived.csv is already in the archive manifest $mf" "$ROOT/archived.log" &&
    grep -q 'pass an --attempt whose log names are free' "$ROOT/archived.log" ||
    fail "no refusal of the archived log name ($mf)" "$ROOT/archived.log"
  [ "$(grep -c 'is already in the archive manifest' "$ROOT/archived.log")" -eq 1 ] ||
    fail "a free log name was refused ($mf)" "$ROOT/archived.log"
  [ -z "$(ls -A "$OUT")" ] || fail "a session reusing an archived log name ($mf) left files"
done
if setsid tools/trace_session.sh "${COMMON[@]}" --yes --runs "$RUNS" --manifest "$ROOT/no-manifest.md" \
    </dev/null >"$ROOT/nomanifest.log" 2>&1; then
  fail "a session without the archive manifest started" "$ROOT/nomanifest.log"
fi
grep -q "no archive manifest at $ROOT/no-manifest.md" "$ROOT/nomanifest.log" ||
  fail "no refusal without the manifest" "$ROOT/nomanifest.log"
[ -z "$(ls -A "$OUT")" ] || fail "a session without the archive manifest left files"
setsid tools/trace_session.sh "${COMMON[@]}" --dry-run --runs "$RUNS" --manifest "$ROOT/manifest.md" \
  </dev/null >"$ROOT/archived-plan.log" 2>&1 || fail "the dry run with an archived name failed" "$ROOT/archived-plan.log"
grep -q 'is already in the archive manifest' "$ROOT/archived-plan.log" && grep -q '^plan: 4 runs' "$ROOT/archived-plan.log" ||
  fail "the dry run does not list the archived name with its plan" "$ROOT/archived-plan.log"

# An unrelated batch workload refuses the session; it is not stopped.
python3 -c 'import time; open("/proc/self/comm", "w").write("rtctrl-batchjob"); time.sleep(30)' &
batch_pid=$!
for _ in $(seq 1 50); do pgrep -x rtctrl-batchjob >/dev/null && break; sleep 0.1; done
if TRACE_SESSION_BATCH_LIST=dpkg,rtctrl-batchjob setsid tools/trace_session.sh "${COMMON[@]}" --yes --runs "$RUNS" \
    </dev/null >"$ROOT/batch.log" 2>&1; then
  fail "a session started beside a batch workload" "$ROOT/batch.log"
fi
grep -q 'unrelated batch workloads are running (rtctrl-batchjob): let them finish or stop them yourself (the session never stops a process)$' "$ROOT/batch.log" ||
  fail "no refusal of the batch workload" "$ROOT/batch.log"
kill -0 "$batch_pid" 2>/dev/null || fail "the session stopped the batch workload"
kill "$batch_pid"; wait "$batch_pid" 2>/dev/null || true
[ -z "$(ls -A "$OUT")" ] || fail "a session beside a batch workload left files"

# The pairs: each run leaves its log, summary, record and task snapshot;
# only a traced run has a trace and a usbmon capture.
setsid tools/trace_session.sh "${COMMON[@]}" --yes --runs "$RUNS" \
  </dev/null >"$ROOT/session.log" 2>&1 ||
  fail "the session failed" "$ROOT/session.log"
grep -q '^Traced torque-off results' "$ROOT/session.log"
for run in current-100hz-idle-bounded-fast-motion position-100hz-idle-bounded; do
  for mode in untraced traced; do
    b=$(base "$run-$mode" 1)
    for f in "$b" "$b.summary.toml" "$b.run.txt" "$b.tasks.txt"; do
      [ -f "$f" ] || fail "missing $f" "$ROOT/session.log"
    done
    note="$b.run.txt"
    grep -q "^mode: $mode$" "$note" || fail "$note: no mode $mode" "$note"
    grep -q "^command: .* --port $LINK " "$note" || fail "$note: not made through the given port" "$note"
    grep -q -- '--read-path bounded' "$note"
    grep -Eq '^verdict: (pass|fail)' "$note" || fail "no verdict in $note" "$note"
    grep -q '^fair and ext servers (bpftrace, read-only; per CPU):' "$note"
    grep -q '^  cpu 0 fair_server_pick_task runtime_ns 50000000 period_ns 1000000000' "$note"
    grep -q '^--- IRQ placement before ---' "$note"
    grep -q '^--- IRQ placement after ---' "$note"
    # The excluded CPU's idle counters, before and after the probe, in
    # untraced and traced runs alike.
    grep -q '^--- idle counters, CPUs 0 (read-only, right before and after the probe) ---$' "$note" &&
      grep -Eq '^idle counters \(before\): monotonic_ns [0-9]+ [0-9]+ \(read between them\), root '"$ROOT/cpu"'$' "$note" &&
      grep -Eq '^idle counters \(after\): monotonic_ns [0-9]+ [0-9]+ ' "$note" &&
      grep -q '^cpu0 state2 name=C2_ACPI desc=stand-in latency=127 residency=381 disable=0 usage=12 time=200 above=unavailable below=unavailable rejected=0$' "$note" &&
      grep -q '^cpu0 state2 C2_ACPI: usage +0, time +0, above unavailable, below unavailable, rejected +0$' "$note" ||
      fail "$note: no idle counters around the probe" "$note"
    grep -q '^--- Cpus_allowed_list per task' "$b.tasks.txt"
    if [ "$mode" = traced ]; then
      [ -s "$b.trace.dat" ] || fail "missing $b.trace.dat" "$note"
      [ -s "$b.usbmon.pcap" ] || fail "missing $b.usbmon.pcap" "$note"
      grep -q '^trace clock while recording: .*\[mono\]' "$note" || fail "$note: no mono clock" "$note"
      grep -Eq '^clock pair before: monotonic_before_ns [0-9]+ realtime_ns [0-9]+ monotonic_after_ns [0-9]+ realtime_minus_monotonic_ns -?[0-9]+ bracket_ns [0-9]+$' "$note" ||
        fail "$note: no clock pair before the run" "$note"
      grep -q '^clock pair after: monotonic_before_ns' "$note"
      grep -q '^--- trace-cmd (exit 0) ---' "$note"
      grep -q '^--- tcpdump (exit 0) ---' "$note"
      grep -q '^0 packets captured' "$note"
    else
      [ ! -e "$b.trace.dat" ] && [ ! -e "$b.usbmon.pcap" ] ||
        fail "an untraced run was traced" "$note"
      if grep -q '^clock pair' "$note"; then fail "an untraced run has clock pairs" "$note"; fi
    fi
  done
done
# The recorder ran on the mono clock with the scheduler and xHCI events;
# tcpdump captured only the adapter (bus 1, device 9).
grep -q "^record -C mono -b 16384 .*-e sched:sched_switch .*-e xhci-hcd:xhci_urb_giveback --temp $ROOT/stage/[^ ]*\\.csv\\.trace-tmp -o $ROOT/stage/" "$ROOT/fake.trace-cmd" ||
  fail "trace-cmd was not asked for the mono clock and the events" "$ROOT/fake.trace-cmd"
grep -q '^-i usbmon1 -Z root -s 0 -w .*\.csv\.usbmon\.pcap link\[11\] == 9$' "$ROOT/fake.tcpdump" ||
  fail "tcpdump was not limited to the adapter" "$ROOT/fake.tcpdump"
# modprobe: a session that loaded usbmon unloads it.
if [ -f "$ROOT/fake.modprobe" ]; then
  grep -qx 'usbmon' "$ROOT/fake.modprobe"
  grep -qx -- '-r usbmon' "$ROOT/fake.modprobe"
fi

# Resuming re-runs nothing: every attempted run is skipped.
BEFORE=$(ls -l --time-style=+%s.%N "$OUT" | sha256sum)
setsid tools/trace_session.sh "${COMMON[@]}" --yes --runs "$RUNS" \
  </dev/null >"$ROOT/resume.log" 2>&1 ||
  fail "the resumed session failed" "$ROOT/resume.log"
[ "$BEFORE" = "$(ls -l --time-style=+%s.%N "$OUT" | sha256sum)" ] ||
  fail "resuming changed the outputs" "$ROOT/resume.log"
[ "$(grep -c 'skipped (attempted earlier)' "$ROOT/resume.log")" -eq 4 ] ||
  fail "resuming did not skip all four runs" "$ROOT/resume.log"

# A tracer that does not start stops the session before the probe runs.
if FAKE_TRACE_FAIL=1 setsid tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt 2 --modes traced --runs current:100:idle:full,cbp:100:idle:full \
    </dev/null >"$ROOT/notrace.log" 2>&1; then
  fail "a session without its tracer went on" "$ROOT/notrace.log"
fi
b=$(base current-100hz-idle-bounded-traced 2)
[ ! -e "$b" ] || fail "the probe ran without its tracer" "$ROOT/notrace.log"
grep -q '^verdict: not run (tracer)' "$b.run.txt"
[ ! -e "$(base cbp-100hz-idle-bounded-traced 2).run.txt" ] ||
  fail "a run started after the session stopped" "$ROOT/notrace.log"

# A trace clock other than mono stops the session before the probe runs.
if FAKE_TRACE_CLOCK=local setsid tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt 3 --modes traced --runs current:100:idle:full \
    </dev/null >"$ROOT/clock.log" 2>&1; then
  fail "a session on the local trace clock went on" "$ROOT/clock.log"
fi
b=$(base current-100hz-idle-bounded-traced 3)
[ ! -e "$b" ] || fail "the probe ran on the local trace clock" "$ROOT/clock.log"
grep -q 'the trace clock is not mono' "$ROOT/clock.log"
grep -q '^verdict: not run (trace clock)' "$b.run.txt"

# Events lost from the trace invalidate the capture: a tracer failure that
# stops the session, never a timing verdict, and no further run starts.
if FAKE_TRACE_OVERRUN=7 setsid tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt 28 --modes traced --runs current:100:idle:full,cbp:100:idle:full \
    </dev/null >"$ROOT/lost.log" 2>&1; then
  fail "a session whose trace lost events went on" "$ROOT/lost.log"
fi
b=$(base current-100hz-idle-bounded-traced 28)
grep -q '^trace overruns: 7 (' "$b.run.txt" &&
  grep -q '^verdict: tracer failure (the trace lost 7 events: the capture is invalid)' "$b.run.txt" ||
  fail "the record lacks the lost events and the tracer failure" "$b.run.txt"
if grep -qE '^verdict: (pass|fail)' "$b.run.txt"; then fail "a capture that lost events got a timing verdict" "$b.run.txt"; fi
grep -q 'the trace lost 7 events: the capture is invalid: session stopped' "$ROOT/lost.log" ||
  fail "the session did not say why it stopped" "$ROOT/lost.log"
[ ! -e "$(base cbp-100hz-idle-bounded-traced 28).run.txt" ] ||
  fail "a run started after the lost events" "$ROOT/lost.log"

# A loaded traced run: the load starts after both recorders record, is
# pinned off the excluded CPU (0), runs out its timeout and succeeds.
: >"$ROOT/fake.events"
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 5 --load-tail 5 \
  --modes traced --runs current:100:loaded-excl:full </dev/null >"$ROOT/load.log" 2>&1 ||
  fail "the loaded session failed" "$ROOT/load.log"
b=$(base current-100hz-loaded-excl-bounded-traced 5)
grep -Eq '^verdict: (pass|fail)' "$b.run.txt" || fail "no verdict in $b.run.txt" "$b.run.txt"
grep -q '^--- stress-ng (exit 0) ---' "$b.run.txt"
[ "$(tr '\n' ' ' <"$ROOT/fake.events")" = "trace-ready usbmon-ready load " ] ||
  fail "the load did not start after the recorders" "$ROOT/fake.events"
python3 - "$ROOT/fake.stress-ng" <<'PY' || fail "the -excl load was not pinned off CPU 0" "$ROOT/fake.stress-ng"
import sys
cpus = [int(c) for c in open(sys.argv[1]).read().split()[-1].split(",")]
sys.exit(0 if cpus and 0 not in cpus else 1)
PY

# A load that ends before the probe does voids the run and stops the
# session, even with the probe's own run intact.
if FAKE_STRESS_LIFE=4 FAKE_STRESS_EXIT=7 setsid tools/trace_session.sh "${COMMON[@]}" \
    --yes --attempt 6 --duration 4 --modes untraced \
    --runs current:100:loaded:full,cbp:100:loaded:full </dev/null >"$ROOT/shortload.log" 2>&1; then
  fail "a load that ended early was accepted" "$ROOT/shortload.log"
fi
b=$(base current-100hz-loaded-bounded-untraced 6)
grep -q '^verdict: load failure (the load ended before the probe did (stress-ng exit 7))' "$b.run.txt" ||
  fail "no load failure in $b.run.txt" "$b.run.txt"
[ ! -e "$(base cbp-100hz-loaded-bounded-untraced 6).run.txt" ] ||
  fail "a run started after the load failed" "$ROOT/shortload.log"

# A load that lasts but does not succeed is rejected too.
if FAKE_STRESS_EXIT=3 setsid tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt 7 --load-tail 5 --modes untraced --runs current:100:loaded:full \
    </dev/null >"$ROOT/badload.log" 2>&1; then
  fail "a failed load was accepted" "$ROOT/badload.log"
fi
grep -q '^verdict: load failure (stress-ng exit 3)' \
  "$(base current-100hz-loaded-bounded-untraced 7).run.txt"

# A recorder that ends before the probe does, even successfully, voids the
# run and stops the session.
for tracer in trace usbmon; do
  case "$tracer" in
    trace) attempt=8; env_life=FAKE_TRACE_LIFE; name=trace-cmd ;;
    usbmon) attempt=9; env_life=FAKE_USBMON_LIFE; name=tcpdump ;;
  esac
  if env "$env_life=2" setsid tools/trace_session.sh "${COMMON[@]}" --yes \
      --attempt "$attempt" --duration 4 --modes traced \
      --runs current:100:idle:full,cbp:100:idle:full </dev/null >"$ROOT/short-$tracer.log" 2>&1; then
    fail "a $name that ended early was accepted" "$ROOT/short-$tracer.log"
  fi
  b=$(base current-100hz-idle-bounded-traced "$attempt")
  grep -q "^verdict: tracer failure ($name ended before the probe did)" "$b.run.txt" ||
    fail "no tracer failure in $b.run.txt" "$b.run.txt"
  [ ! -e "$(base cbp-100hz-idle-bounded-traced "$attempt").run.txt" ] ||
    fail "a run started after $name ended" "$ROOT/short-$tracer.log"
done

# A recorder that ends while the load settles stops the session before the
# probe runs.
if FAKE_TRACE_LIFE=1 setsid tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt 10 --modes traced --runs current:100:loaded:full \
    </dev/null >"$ROOT/settle.log" 2>&1; then
  fail "a recorder that ended before the run was accepted" "$ROOT/settle.log"
fi
b=$(base current-100hz-loaded-bounded-traced 10)
[ ! -e "$b" ] || fail "the probe ran without its recorder" "$ROOT/settle.log"
grep -q 'trace-cmd ended before the run' "$ROOT/settle.log"
grep -q '^verdict: not run (tracer)' "$b.run.txt"

# An interrupted session stops its recorders and gives the run's outputs to
# the invoking user, as every other path does: Ctrl-C reaches the whole
# process group (the probe ends at once), SIGTERM only the script (its trap
# runs once the probe's run ends). Through the stand-in sudo, so the
# ownership handback is visible. Started in the background, the session
# would ignore SIGINT (and bash cannot trap a signal ignored at entry); like
# one started at a terminal, this one does not, and it leads its own
# process group.
# The sudo refresh (with the stand-in sudo): its loop and every process it
# starts are on the tracers' CPUs, never the excluded one, read from /proc
# while the session runs; none of it outlives the session.
cpu_set() {  # CPU list -> the CPUs, one per word
  local part a out=()
  IFS=, read -r -a parts <<<"$1"
  for part in "${parts[@]}"; do
    if [[ $part =~ ^([0-9]+)-([0-9]+)$ ]]; then
      for ((a = BASH_REMATCH[1]; a <= BASH_REMATCH[2]; a++)); do out+=("$a"); done
    else out+=("$part"); fi
  done
  echo "${out[*]}"
}
allowed() { sed -n 's/^Cpus_allowed_list:[[:space:]]*//p' "/proc/$1/status" 2>/dev/null; }
refresh_pinned() {  # RECORD: the loop's pid, once it and its children are on the tracers' CPUs
  local line loop want c kids
  line=$(grep -m1 '^sudo refresh: loop ' "$1") || fail "the record has no sudo refresh line" "$1"
  loop=$(sed -E 's/^sudo refresh: loop ([0-9]+) .*/\1/' <<<"$line")
  want=$(cpu_set "$(sed -nE 's/^excluded CPUs .*tracer and -excl load CPUs: ([0-9,]+)$/\1/p' "$1")")
  [ -n "$want" ] || fail "the record names no tracer CPUs" "$1"
  [[ $line =~ "on CPUs "([0-9,-]+)" (read back at the session's start), now "([0-9,-]+)$ ]] ||
    fail "the sudo refresh line reads back no CPUs" "$1"
  [ "$(cpu_set "${BASH_REMATCH[1]}")" = "$want" ] && [ "$(cpu_set "${BASH_REMATCH[2]}")" = "$want" ] ||
    fail "the sudo refresh is not on the tracers' CPUs ($want): $line" "$1"
  [ "$(cpu_set "$(allowed "$loop")")" = "$want" ] || fail "the refresh loop $loop is on $(allowed "$loop"), not $want" "$1"
  kids=$(pgrep -P "$loop") || fail "the refresh loop $loop has no sleep" "$1"
  for c in $kids; do
    [ "$(cpu_set "$(allowed "$c")")" = "$want" ] || fail "the refresh loop's child $c is on $(allowed "$c"), not $want" "$1"
  done
  echo "$loop $kids"
}
gone() {  # PID...: none of them alive
  local q
  for q in "$@"; do ! kill -0 "$q" 2>/dev/null || return 1; done
}

for sig in INT TERM; do
  case "$sig" in INT) attempt=11; target=group ;; TERM) attempt=12; target=script ;; esac
  : >"$ROOT/fake.sudo"
  TRACE_SESSION_SUDO="$FAKE/sudo" python3 -c '
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.setsid()
os.execvp(sys.argv[1], sys.argv[1:])' tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt "$attempt" --duration 4 --modes traced --runs current:100:idle:full,cbp:100:idle:full \
    </dev/null >"$ROOT/int-$sig.log" 2>&1 &
  pid=$!
  b=$(base current-100hz-idle-bounded-traced "$attempt")
  for _ in $(seq 1 300); do
    grep -q '^clock pair before' "$b.run.txt" 2>/dev/null && break
    sleep 0.1
  done
  grep -q '^clock pair before' "$b.run.txt" || fail "the probe never started" "$ROOT/int-$sig.log"
  [ "$(ps -o pgid= -p "$pid" | tr -d ' ')" = "$pid" ] || fail "the session does not lead its process group"
  refresh=$(refresh_pinned "$b.run.txt")
  sleep 1
  if [ "$target" = group ]; then kill -INT -- "-$pid"; else kill -TERM "$pid"; fi
  status=0
  wait "$pid" || status=$?
  [ "$status" -eq 130 ] || fail "the interrupted session exited $status" "$ROOT/int-$sig.log"
  # shellcheck disable=SC2086  # the pids, one per word
  gone $refresh || fail "the sudo refresh ($refresh) outlived the session after SIG$sig" "$ROOT/int-$sig.log"
  grep -q 'INTERRUPTED (counts as attempted)' "$ROOT/int-$sig.log" ||
    fail "the interrupt was not reported" "$ROOT/int-$sig.log"
  [ -s "$b.trace.dat" ] && [ -s "$b.usbmon.pcap" ] ||
    fail "the interrupted run's recorders did not write their files" "$ROOT/int-$sig.log"
  # Their stage was given back, published (the ledger checks) and released.
  grep -q "^chown -R $(id -u):$(id -g) $ROOT/stage/" "$ROOT/fake.sudo" ||
    fail "the stage was not given back after SIG$sig" "$ROOT/fake.sudo"
  (cd "$OUT" && sha256sum --quiet -c "$(basename "$b").staging.sha256") ||
    fail "the interrupted run's ledger does not check after SIG$sig" "$b.run.txt"
  grep -q '^preservation: published and verified, the stage released$' "$b.run.txt" ||
    fail "the interrupted run's preservation verdict after SIG$sig" "$b.run.txt"
  [ -z "$(ls -A "$ROOT/stage")" ] || fail "a stage remains after SIG$sig" "$ROOT/int-$sig.log"
  [ ! -e "$(base cbp-100hz-idle-bounded-traced "$attempt").run.txt" ] ||
    fail "a run started after SIG$sig" "$ROOT/int-$sig.log"
done

# A session that ends normally: the refresh was on the tracers' CPUs when the
# run's record was written, and none of it remains.
b=$(base current-100hz-idle-bounded-traced 65)
TRACE_SESSION_SUDO="$FAKE/sudo" setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 65 \
  --modes traced --runs current:100:idle:full </dev/null >"$ROOT/refresh-end.log" 2>&1 ||
  fail "the session with the sudo refresh failed" "$ROOT/refresh-end.log"
line=$(grep -m1 '^sudo refresh: loop ' "$b.run.txt") || fail "the record has no sudo refresh line" "$b.run.txt"
want=$(cpu_set "$(sed -nE 's/^excluded CPUs .*tracer and -excl load CPUs: ([0-9,]+)$/\1/p' "$b.run.txt")")
[[ $line =~ "on CPUs "([0-9,-]+)" (read back at the session's start), now "([0-9,-]+)$ ]] &&
  [ "$(cpu_set "${BASH_REMATCH[1]}")" = "$want" ] && [ "$(cpu_set "${BASH_REMATCH[2]}")" = "$want" ] ||
  fail "the sudo refresh was not on the tracers' CPUs ($want): $line" "$b.run.txt"
loop=$(sed -E 's/^sudo refresh: loop ([0-9]+) .*/\1/' <<<"$line")
gone "$loop" && [ -z "$(pgrep -P "$loop")" ] || fail "the sudo refresh loop $loop outlived the session" "$ROOT/refresh-end.log"
# Without the sudo refresh's pin (taskset fails), or with a pin that does not
# read back (taskset does nothing), nothing is sent, and the loop is ended.
real_taskset=$(command -v taskset)
mkdir -p "$FAKE/tsbin"
cat >"$FAKE/tsbin/taskset" <<EOF
#!/usr/bin/env bash
if [ "\$1" = -cp ]; then
  case "\${FAKE_TASKSET_PIN:-}" in fail) echo "stand-in: taskset failed" >&2; exit 1 ;; noop) exit 0 ;; esac
fi
exec $real_taskset "\$@"
EOF
chmod +x "$FAKE/tsbin/taskset"
for pin in fail noop; do
  case "$pin" in fail) attempt=66; seen='\(taskset failed\)' ;; noop) attempt=67; seen='0[-,]' ;; esac
  status=0
  FAKE_TASKSET_PIN=$pin PATH="$FAKE/tsbin:$PATH" TRACE_SESSION_SUDO="$FAKE/sudo" setsid tools/trace_session.sh \
    "${COMMON[@]}" --yes --attempt "$attempt" --modes traced --runs current:100:idle:full \
    </dev/null >"$ROOT/pin-$pin.log" 2>&1 || status=$?
  [ "$status" -eq 2 ] || fail "a session whose refresh pin $pin exited $status" "$ROOT/pin-$pin.log"
  grep -Eq "the sudo refresh \(loop [0-9]+\) is not on CPUs [0-9,]+ \(read back: $seen.*\): nothing was sent" "$ROOT/pin-$pin.log" ||
    fail "the refresh pin's failure ($pin) was not reported" "$ROOT/pin-$pin.log"
  [ ! -e "$(base current-100hz-idle-bounded-traced "$attempt").run.txt" ] && [ ! -e "$(base current-100hz-idle-bounded-traced "$attempt")" ] ||
    fail "a run started without the refresh's pin ($pin)" "$ROOT/pin-$pin.log"
  loop=$(sed -nE 's/.*the sudo refresh \(loop ([0-9]+)\).*/\1/p' "$ROOT/pin-$pin.log")
  gone "$loop" && [ -z "$(pgrep -P "$loop")" ] || fail "the unpinned refresh loop $loop outlived the session" "$ROOT/pin-$pin.log"
done

# The adapter experiment. No session above touched the latency timer.
[ ! -e "$ROOT/fake.latency" ] || fail "a session without --latency-steps used the latency tool" "$ROOT/fake.latency"
LT_TTY=$(basename "$(readlink -f "$LINK")")
latency_state() { cat "$ROOT/fake.latency-state"; }

# Steps 1, 0, 1, each untraced and traced: the timer is set when the step
# changes, checked before and after every run, the probe expects the step's
# value, and the value found (4 ms) comes back at the end.
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 13 --latency-steps 1,0,1 \
  --runs current:100:idle:full </dev/null >"$ROOT/latency.log" 2>&1 ||
  fail "the latency session failed" "$ROOT/latency.log"
expected_calls="read $LT_TTY
set $LT_TTY 1
read $LT_TTY
read $LT_TTY
read $LT_TTY
set $LT_TTY 0
read $LT_TTY
read $LT_TTY
read $LT_TTY
set $LT_TTY 1
read $LT_TTY
read $LT_TTY
read $LT_TTY
read $LT_TTY
set $LT_TTY 4"
[ "$(cat "$ROOT/fake.latency")" = "$expected_calls" ] ||
  fail "the latency timer was not set and read as planned" "$ROOT/fake.latency"
[ "$(latency_state)" = "4 4 0" ] || fail "the latency timer was not restored" "$ROOT/latency.log"
grep -q "the adapter's latency timer is restored to 4 ms" "$ROOT/latency.log"
for s in 1:1 0:2 1:3; do
  ms=${s%%:*}; k=${s##*:}
  for mode in untraced traced; do
    b=$(base "current-100hz-idle-bounded-lt$ms-s$k-$mode" 13)
    note="$b.run.txt"
    [ -f "$b" ] && [ -f "$note" ] || fail "missing the step $k $mode run" "$ROOT/latency.log"
    grep -Eq '^verdict: (pass|fail)' "$note" || fail "no verdict in $note" "$note"
    grep -q "^latency step: $ms ms (step $k of 3: 1,0,1)" "$note"
    grep -q "^latency timer before the run (requested $ms ms): tty $LT_TTY sysfs $ms device $ms low_latency 0 " "$note" ||
      fail "$note: no latency check before the run" "$note"
    grep -q "^latency timer after the run: tty $LT_TTY sysfs $ms device $ms low_latency 0 " "$note" ||
      fail "$note: no latency check after the run" "$note"
    grep -q "^command: .* --expect-latency-timer-ms $ms" "$note" || fail "$note: the probe did not expect $ms ms" "$note"
    grep -Eq '^xhci interrupts around the probe: [0-9]+ in [0-9.]+ s' "$note" || fail "$note: no xHCI count" "$note"
    grep -q "^latency steps: 1,0,1; at the session's start: tty $LT_TTY sysfs 4 device 4 low_latency 0" "$note"
  done
done
grep -q '^cleanup: latency timer restored to 4 ms' "$(base current-100hz-idle-bounded-lt1-s3-traced 13).run.txt" ||
  fail "the restoration is not in the last record" "$(base current-100hz-idle-bounded-lt1-s3-traced 13).run.txt"
grep -Eq 'current:100:idle:full@0ms#2 +traced' "$ROOT/latency.log"

# Ctrl-C and SIGTERM during a 0 ms step restore the timer too.
for sig in INT TERM; do
  case "$sig" in INT) attempt=14; target=group ;; TERM) attempt=15; target=script ;; esac
  python3 -c '
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.setsid()
os.execvp(sys.argv[1], sys.argv[1:])' tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt "$attempt" --duration 4 --modes traced --latency-steps 0 \
    --runs current:100:idle:full </dev/null >"$ROOT/latency-$sig.log" 2>&1 &
  pid=$!
  b=$(base current-100hz-idle-bounded-lt0-s1-traced "$attempt")
  for _ in $(seq 1 300); do
    grep -q '^clock pair before' "$b.run.txt" 2>/dev/null && break
    sleep 0.1
  done
  grep -q '^clock pair before' "$b.run.txt" || fail "the probe never started" "$ROOT/latency-$sig.log"
  [ "$(latency_state)" = "0 0 0" ] || fail "the 0 ms step was not set" "$ROOT/latency-$sig.log"
  sleep 1
  if [ "$target" = group ]; then kill -INT -- "-$pid"; else kill -TERM "$pid"; fi
  status=0
  wait "$pid" || status=$?
  [ "$status" -eq 130 ] || fail "the interrupted latency session exited $status" "$ROOT/latency-$sig.log"
  [ "$(latency_state)" = "4 4 0" ] || fail "SIG$sig left the latency timer at $(latency_state)" "$ROOT/latency-$sig.log"
  grep -q "the adapter's latency timer is restored to 4 ms" "$ROOT/latency-$sig.log"
done

# An adapter that keeps its value for a 0 ms request voids the step before
# its probe runs, stops the session and is restored.
: >"$ROOT/fake.latency"
if FAKE_LATENCY_IGNORE=0 setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 16 \
    --modes untraced --latency-steps 1,0 --runs current:100:idle:full,cbp:100:idle:full \
    </dev/null >"$ROOT/latency-ignored.log" 2>&1; then
  fail "a step the adapter did not hold was accepted" "$ROOT/latency-ignored.log"
fi
b=$(base current-100hz-idle-bounded-lt0-s2-untraced 16)
[ ! -e "$b" ] || fail "the probe ran with the adapter at another latency" "$ROOT/latency-ignored.log"
grep -q '^verdict: not run (latency timer)' "$b.run.txt"
[ -f "$(base current-100hz-idle-bounded-lt1-s1-untraced 16)" ] || fail "the 1 ms step did not run" "$ROOT/latency-ignored.log"
[ ! -e "$(base cbp-100hz-idle-bounded-lt1-s1-untraced 16).run.txt" ] ||
  fail "a run started after the latency stop" "$ROOT/latency-ignored.log"
[ "$(latency_state)" = "4 4 0" ] || fail "the stopped session did not restore the timer" "$ROOT/latency-ignored.log"
tail -n 1 "$ROOT/fake.latency" | grep -qx "set $LT_TTY 4"

# The adapter's value changing during a run voids it, stops the session and
# is restored.
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 18 --duration 3 \
  --modes untraced --latency-steps 0 --runs current:100:idle:full,cbp:100:idle:full \
  </dev/null >"$ROOT/latency-drift.log" 2>&1 &
pid=$!
b=$(base current-100hz-idle-bounded-lt0-s1-untraced 18)
for _ in $(seq 1 300); do [ -e "$b" ] && break; sleep 0.1; done
[ -e "$b" ] || fail "the probe never started" "$ROOT/latency-drift.log"
echo "0 16 0" >"$ROOT/fake.latency-state"
status=0
wait "$pid" || status=$?
[ "$status" -eq 1 ] || fail "a run whose latency timer changed was accepted (exit $status)" "$ROOT/latency-drift.log"
grep -q '^verdict: latency timer failure (the adapter.s latency timer read tty .* device 16 .* after the run, not 0 ms)' "$b.run.txt" ||
  fail "no latency failure in $b.run.txt" "$b.run.txt"
[ "$(latency_state)" = "4 4 0" ] || fail "the drifted timer was not restored" "$ROOT/latency-drift.log"

# Starting at the first step's own value, the session sets nothing, yet it
# owes that value back once a run begins: a run that changes the adapter is
# voided, and the cleanup sets the starting value again; an undisturbed
# session's cleanup verifies it.
echo "1 1 0" >"$ROOT/fake.latency-state"
: >"$ROOT/fake.latency"
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 20 --duration 3 \
  --modes untraced --latency-steps 1,0,1 --runs current:100:idle:full </dev/null >"$ROOT/latency-drift1.log" 2>&1 &
pid=$!
b=$(base current-100hz-idle-bounded-lt1-s1-untraced 20)
for _ in $(seq 1 300); do [ -e "$b" ] && break; sleep 0.1; done
[ -e "$b" ] || fail "the probe never started" "$ROOT/latency-drift1.log"
echo "1 16 0" >"$ROOT/fake.latency-state"
status=0
wait "$pid" || status=$?
[ "$status" -eq 1 ] || fail "a run whose latency timer changed was accepted (exit $status)" "$ROOT/latency-drift1.log"
grep -q '^verdict: latency timer failure' "$b.run.txt" || fail "no latency failure in $b.run.txt" "$b.run.txt"
[ "$(latency_state)" = "1 1 0" ] || fail "the starting 1 ms was not restored: $(latency_state)" "$ROOT/latency-drift1.log"
[ "$(grep -c '^set ' "$ROOT/fake.latency")" -eq 1 ] && tail -n 1 "$ROOT/fake.latency" | grep -qx "set $LT_TTY 1" ||
  fail "the cleanup did not set the starting value (and only that)" "$ROOT/fake.latency"
grep -q '^cleanup: latency timer restored to 1 ms' "$b.run.txt"
: >"$ROOT/fake.latency"
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 21 --modes untraced \
  --latency-steps 1 --runs current:100:idle:full </dev/null >"$ROOT/latency-same.log" 2>&1 ||
  fail "the session at its starting value failed" "$ROOT/latency-same.log"
! grep -q '^set ' "$ROOT/fake.latency" || fail "a session at its starting value set the timer" "$ROOT/fake.latency"
[ "$(grep -c '^read ' "$ROOT/fake.latency")" -eq 4 ] || fail "the cleanup did not verify the timer" "$ROOT/fake.latency"
grep -q "cleanup: the adapter's latency timer is verified at the starting 1 ms" "$ROOT/latency-same.log"
grep -q '^cleanup: latency timer verified at the starting 1 ms' "$(base current-100hz-idle-bounded-lt1-s1-untraced 21).run.txt"

# A cleanup that cannot restore the starting value fails the session: an
# otherwise successful one exits 3; a stop and an interruption keep their
# status. The adapter is left at the step's value, and the records say so.
echo "4 4 0" >"$ROOT/fake.latency-state"
status=0
FAKE_LATENCY_FAIL_SET=4 setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 22 --modes untraced \
  --latency-steps 0 --runs current:100:idle:full </dev/null >"$ROOT/latency-restore-fail.log" 2>&1 || status=$?
[ "$status" -eq 3 ] || fail "a failed restoration exited $status, not 3" "$ROOT/latency-restore-fail.log"
b=$(base current-100hz-idle-bounded-lt0-s1-untraced 22)
grep -Eq '^verdict: (pass|fail)' "$b.run.txt" || fail "the run itself did not complete" "$b.run.txt"
grep -q "CLEANUP FAILED: the adapter's latency timer is NOT restored to 4 ms" "$ROOT/latency-restore-fail.log"
grep -q 'the session fails: its cleanup did not restore' "$ROOT/latency-restore-fail.log"
grep -q '^cleanup: latency timer NOT restored to 4 ms' "$b.run.txt"
[ "$(latency_state)" = "0 0 0" ] || fail "unexpected adapter state $(latency_state)"
echo "4 4 0" >"$ROOT/fake.latency-state"
status=0
FAKE_LATENCY_IGNORE=0 FAKE_LATENCY_FAIL_SET=4 setsid tools/trace_session.sh "${COMMON[@]}" --yes \
  --attempt 23 --modes untraced --latency-steps 0 --runs current:100:idle:full \
  </dev/null >"$ROOT/latency-stop-fail.log" 2>&1 || status=$?
[ "$status" -eq 1 ] || fail "a stop with a failed restoration exited $status, not 1" "$ROOT/latency-stop-fail.log"
grep -q 'CLEANUP FAILED' "$ROOT/latency-stop-fail.log"
echo "4 4 0" >"$ROOT/fake.latency-state"
FAKE_LATENCY_FAIL_SET=4 python3 -c '
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.setsid()
os.execvp(sys.argv[1], sys.argv[1:])' tools/trace_session.sh "${COMMON[@]}" --yes \
  --attempt 24 --duration 4 --modes traced --latency-steps 0 \
  --runs current:100:idle:full </dev/null >"$ROOT/latency-int-fail.log" 2>&1 &
pid=$!
b=$(base current-100hz-idle-bounded-lt0-s1-traced 24)
for _ in $(seq 1 300); do
  grep -q '^clock pair before' "$b.run.txt" 2>/dev/null && break
  sleep 0.1
done
grep -q '^clock pair before' "$b.run.txt" || fail "the probe never started" "$ROOT/latency-int-fail.log"
kill -TERM "$pid"
status=0
wait "$pid" || status=$?
[ "$status" -eq 130 ] || fail "an interruption with a failed restoration exited $status" "$ROOT/latency-int-fail.log"
grep -q 'CLEANUP FAILED' "$ROOT/latency-int-fail.log"
echo "4 4 0" >"$ROOT/fake.latency-state"

# The low-latency flag at the start, or values that differ, refuse the
# session before anything is sent; a startup failure before the read
# changes nothing.
for st in "1 1 1" "4 1 0"; do
  echo "$st" >"$ROOT/fake.latency-state"
  : >"$ROOT/fake.latency"
  if setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 17 --modes untraced \
      --latency-steps 1,0 --runs current:100:idle:full </dev/null >"$ROOT/latency-refused.log" 2>&1; then
    fail "a session started with the adapter at $st" "$ROOT/latency-refused.log"
  fi
  grep -Eq 'low-latency flag is set|reads 1 but sysfs 4' "$ROOT/latency-refused.log" ||
    fail "no refusal for $st" "$ROOT/latency-refused.log"
  [ "$(cat "$ROOT/fake.latency")" = "read $LT_TTY" ] || fail "the refused session set the timer" "$ROOT/fake.latency"
  [ ! -e "$(base current-100hz-idle-bounded-lt1-s1-untraced 17).run.txt" ] || fail "a refused session ran"
  [ "$(latency_state)" = "$st" ] || fail "the refused session changed the timer"
done
echo "4 4 0" >"$ROOT/fake.latency-state"
: >"$ROOT/fake.latency"
if FAKE_BPF_FAIL=1 setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 19 --modes untraced \
    --latency-steps 0 --runs current:100:idle:full </dev/null >"$ROOT/latency-startup.log" 2>&1; then
  fail "a session without the fair-server read started" "$ROOT/latency-startup.log"
fi
[ ! -s "$ROOT/fake.latency" ] || fail "a session that failed at startup used the latency tool" "$ROOT/fake.latency"

# The plan names the steps and touches nothing; bad steps are refused.
setsid tools/trace_session.sh "${COMMON[@]}" --dry-run --latency-steps 1,0,1 \
  --runs current:1000:rt-idle:fast-motion </dev/null >"$ROOT/latency-plan.log" 2>&1 ||
  fail "the latency plan failed" "$ROOT/latency-plan.log"
grep -q '^plan: 6 runs' "$ROOT/latency-plan.log"
grep -q 'latency timer 0 ms (step 2).*rate-trace-current-1000hz-rt-idle-bounded-fast-motion-lt0-s2-traced-.*--expect-latency-timer-ms 0' "$ROOT/latency-plan.log" ||
  fail "the plan does not show the 0 ms step" "$ROOT/latency-plan.log"
grep -q "^latency steps 1,0,1 on $LT_TTY through" "$ROOT/latency-plan.log"
[ ! -s "$ROOT/fake.latency" ] || fail "the plan used the latency tool" "$ROOT/fake.latency"
for bad in 1,256 1,x 1,,0; do
  if setsid tools/trace_session.sh "${COMMON[@]}" --dry-run --latency-steps "$bad" \
      </dev/null >"$ROOT/latency-bad.log" 2>&1; then
    fail "--latency-steps $bad was accepted" "$ROOT/latency-bad.log"
  fi
  grep -q 'is not a latency timer of 0 to 255 ms' "$ROOT/latency-bad.log"
done

# The profiled mode: a traced run plus the probe threads' timer arming,
# sleep calls and page faults (filtered to the probe's comm) and perf
# samples of the real-time CPU, written as text for the worker. It needs
# rt-* conditions: the stand-in probe replays the first pair's traced run.
export FAKE_PROBE_TEMPLATE
FAKE_PROBE_TEMPLATE=$(base current-100hz-idle-bounded-fast-motion-traced 1)
PROFILE=(--probe "$FAKE/probe" --port "$LINK" --outdir "$OUT" --date "$DATE" --max-runnable 100000
         --usb-device "$ROOT/usbdev" --rt-cpu 0 --exclude-cpus 0)
wtid=$(sed -n 's/^worker_tid = //p' "$FAKE_PROBE_TEMPLATE.summary.toml")
: >"$ROOT/fake.trace-cmd"
TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 25 \
  --duration 2 --modes untraced,profiled --runs current:100:rt-idle:fast-motion \
  </dev/null >"$ROOT/profiled.log" 2>&1 || fail "the profiled session failed" "$ROOT/profiled.log"
b=$(base current-100hz-rt-idle-bounded-fast-motion-profiled 25)
for f in "$b" "$b.run.txt" "$b.trace.dat" "$b.usbmon.pcap" "$b.perf.data" "$b.perf.txt" "$b.perf.stacks.txt"; do
  [ -s "$f" ] || fail "missing $f" "$ROOT/profiled.log"
done
grep -Eq '^verdict: (pass|fail)' "$b.run.txt" || fail "no verdict in $b.run.txt" "$b.run.txt"
# (printf %q may quote the comma)
grep -Eq "^perf: $FAKE/perf record -k mono -C 0 -F 999 --call-graph dwarf\\\\?,4096 -o $ROOT/stage/[^ ]*/$(basename "$b").perf.data" "$b.run.txt" ||
  fail "the record does not name the perf command" "$b.run.txt"
grep -q '^--- perf (exit 130, a clean stop) ---' "$b.run.txt" || fail "perf's SIGINT stop not accepted" "$b.run.txt"
grep -q "^perf script --tid $wtid (exit 0): 1 samples; with call chains (exit 0): 1 samples" "$b.run.txt" ||
  fail "the record does not count the worker's samples" "$b.run.txt"
grep -qx 'perf stats: SAMPLE 12, LOST 0, LOST_SAMPLES 0, THROTTLE 0 (perf report --stats --no-skip-empty, exit 0)' "$b.run.txt" &&
  [ "$(grep -c '^perf stats:' "$b.run.txt")" -eq 1 ] || fail "the record lacks perf's counts" "$b.run.txt"
grep -q "probe $wtid \[000\]" "$b.perf.txt" || fail "the worker's samples are not written" "$b.perf.txt"
grep -q 'Probe::run' "$b.perf.stacks.txt" || fail "the worker's call chains are not written" "$b.perf.stacks.txt"
grep -q -- "-e timer:hrtimer_start -f comm == \"probe\" .*-e syscalls:sys_enter_clock_nanosleep -f comm == \"probe\" .*-e exceptions:page_fault_user -f comm == \"probe\" .*-e workqueue:workqueue_activate_work" "$ROOT/fake.trace-cmd" ||
  fail "trace-cmd was not given the profiled events" "$ROOT/fake.trace-cmd"
grep -q '^unbound workqueues: cpumask ' "$b.run.txt"
u=$(base current-100hz-rt-idle-bounded-fast-motion-untraced 25)
[ -f "$u" ] && [ ! -e "$u.perf.data" ] && [ ! -e "$u.trace.dat" ] || fail "the untraced run was profiled" "$u.run.txt"
# perf that does not start, or ends before the probe, stops the session.
if FAKE_PERF_FAIL=1 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 26 --duration 2 --modes profiled --runs current:100:rt-idle:fast-motion,cbp:100:rt-idle:fast \
    </dev/null >"$ROOT/profiled-nostart.log" 2>&1; then
  fail "a profiled run without perf went on" "$ROOT/profiled-nostart.log"
fi
b=$(base current-100hz-rt-idle-bounded-fast-motion-profiled 26)
[ ! -e "$b" ] || fail "the probe ran without perf" "$ROOT/profiled-nostart.log"
grep -q '^--- perf did not start ---' "$b.run.txt" && grep -q '^verdict: not run (tracer)' "$b.run.txt"
[ ! -e "$(base cbp-100hz-rt-idle-bounded-fast-profiled 26).run.txt" ] || fail "a run started after perf failed"
if FAKE_PERF_LIFE=4 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 27 --duration 8 --modes profiled --runs current:100:rt-idle:fast-motion \
    </dev/null >"$ROOT/profiled-short.log" 2>&1; then
  fail "a perf that ended early was accepted" "$ROOT/profiled-short.log"
fi
b=$(base current-100hz-rt-idle-bounded-fast-motion-profiled 27)
grep -q '^verdict: tracer failure (perf ended before the probe did' "$b.run.txt" ||
  fail "no tracer failure for perf in $b.run.txt" "$b.run.txt"
# Only rt-* conditions pin the probe to the sampled CPU.
if setsid tools/trace_session.sh "${PROFILE[@]}" --dry-run --modes profiled \
    --runs current:100:idle:full </dev/null >"$ROOT/profiled-idle.log" 2>&1; then
  fail "a profiled run without real-time scheduling was planned" "$ROOT/profiled-idle.log"
fi
grep -q 'use it with rt-\* conditions only' "$ROOT/profiled-idle.log"
for bad in 1000 7 70000; do
  if setsid tools/trace_session.sh "${PROFILE[@]}" --dry-run --modes profiled --perf-stack "$bad" \
      --runs current:100:rt-idle:full </dev/null >"$ROOT/profiled-stack.log" 2>&1; then
    fail "--perf-stack $bad was accepted" "$ROOT/profiled-stack.log"
  fi
done
TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --dry-run --modes untraced,traced,profiled \
  --runs current:1000:rt-loaded:fast-motion </dev/null >"$ROOT/profiled-plan.log" 2>&1 ||
  fail "the profiled plan failed" "$ROOT/profiled-plan.log"
grep -q '^plan: 3 runs' "$ROOT/profiled-plan.log"
grep -q "^                      $FAKE/perf record -k mono -C 0 -F 999 --call-graph dwarf,4096 -o <log>.csv.perf.data" "$ROOT/profiled-plan.log" ||
  fail "the plan does not name the perf command" "$ROOT/profiled-plan.log"

# tools/tsc_instance.sh on a scratch tree, plain directories standing for
# tracefs: an instance that exists, and one mkdir cannot make, are status 2
# (not created: never the caller's to remove); one created whose clock then
# cannot be set is status 4 (created: the caller's); its stats name the CPU
# first.
T="$ROOT/tsc-tree"
rm -rf "$T"
mkdir -p "$T/instances/held" "$T/instances/s/per_cpu/cpu3"
printf 'entries: 1\noverrun: 0\n' >"$T/instances/s/per_cpu/cpu3/stats"
set +e
tools/tsc_instance.sh start "$T" held 0 64 2>/dev/null; st_held=$?
tools/tsc_instance.sh start "$T/absent" x 0 64 2>/dev/null; st_mkdir=$?
tools/tsc_instance.sh start "$T" fresh 0 64 2>/dev/null; st_fresh=$?
set -e
[ "$st_held" -eq 2 ] && [ -d "$T/instances/held" ] || fail "an existing instance: status $st_held"
[ "$st_mkdir" -eq 2 ] || fail "an instance mkdir cannot make: status $st_mkdir"
[ "$st_fresh" -eq 4 ] && [ -d "$T/instances/fresh" ] || fail "a created instance whose clock failed: status $st_fresh"
stats=$(tools/tsc_instance.sh stats "$T" s 3) && [ "${stats%%$'\n'*}" = "CPU: 3" ] ||
  fail "the instance's stats do not name the CPU"
# Its checked existence: an answer only from a listing read in full; an
# instances directory that is missing or cannot be listed answers nothing,
# status 1, and so does a removal that cannot verify.
set +e
ex_held=$(tools/tsc_instance.sh exists "$T" held); st_eh=$?
ex_none=$(tools/tsc_instance.sh exists "$T" none); st_en=$?
ex_root=$(tools/tsc_instance.sh exists "$T/absent" held 2>/dev/null); st_er=$?
chmod 000 "$T/instances"
ex_shut=$(tools/tsc_instance.sh exists "$T" held 2>/dev/null); st_es=$?
tools/tsc_instance.sh remove "$T" held 2>/dev/null; st_rs=$?
chmod 755 "$T/instances"
tools/tsc_instance.sh remove "$T" none; st_rn=$?
set -e
[ "$st_eh" -eq 0 ] && [ "$ex_held" = present ] && [ "$st_en" -eq 0 ] && [ "$ex_none" = absent ] ||
  fail "the instance's existence: '$ex_held' $st_eh, '$ex_none' $st_en"
[ "$st_er" -eq 1 ] && [ -z "$ex_root" ] || fail "no instances directory answered '$ex_root', status $st_er"
if [ "$(id -u)" -ne 0 ]; then  # root lists any directory
  [ "$st_es" -eq 1 ] && [ -z "$ex_shut" ] && [ "$st_rs" -eq 1 ] ||
    fail "an unlistable instances directory answered '$ex_shut', status $st_es; removal status $st_rs"
fi
[ "$st_rn" -eq 0 ] || fail "removing an absent instance: status $st_rn"

# tools/fair_server_probes.sh on a scratch tree: a group that exists, and a
# first definition the kernel refuses (kprobe_events cannot be written), are
# status 2 (nothing created: never the caller's to remove); definitions
# written whose event directories do not appear are status 4 (the caller's
# to remove); a definition outside the group is refused before anything is
# written; the counts name every event, and a missing one fails; removing
# events that are absent succeeds.
T="$ROOT/fs-tree"
rm -rf "$T"
mkdir -p "$T/events/held" "$T/nowrite/kprobe_events"
printf 'p:held/rtfs_a f x=$arg1\n' >"$T/held.defs"
printf 'p:fresh/rtfs_a f x=$arg1\nr:fresh/rtfs_b f y=$retval\n' >"$T/fresh.defs"
printf 'p:fresh/rtfs_a f x=$arg1\np:other/rtfs_b f y=$arg1\n' >"$T/mixed.defs"
: >"$T/kprobe_events"
printf '  rtfs_a          12            0\n  other_x  5  1\n' >"$T/kprobe_profile"
set +e
tools/fair_server_probes.sh define "$T" held "$T/held.defs" 2>/dev/null; st_held=$?
tools/fair_server_probes.sh define "$T/nowrite" fresh "$T/fresh.defs" 2>/dev/null; st_nowrite=$?
tools/fair_server_probes.sh define "$T" fresh "$T/fresh.defs" 2>/dev/null; st_nodir=$?
tools/fair_server_probes.sh define "$T" fresh "$T/mixed.defs" 2>"$T/mixed.err"; st_mixed=$?
prof=$(tools/fair_server_probes.sh profile "$T" fresh "$T/fresh.defs" 2>/dev/null); st_prof=$?
tools/fair_server_probes.sh remove "$T" fresh "$T/fresh.defs"; st_rm=$?
set -e
[ "$st_held" -eq 2 ] || fail "an existing probe group: status $st_held"
[ "$st_nowrite" -eq 2 ] || fail "a refused first definition: status $st_nowrite"
[ "$st_nodir" -eq 4 ] && [ "$(cat "$T/kprobe_events")" = "$(cat "$T/fresh.defs")" ] ||
  fail "definitions written without their event directories: status $st_nodir, or not appended as written"
[ "$st_mixed" -eq 1 ] && grep -q 'a definition outside the group fresh' "$T/mixed.err" ||
  fail "a definition outside the group: status $st_mixed"
[ "$(cat "$T/kprobe_events")" = "$(cat "$T/fresh.defs")" ] || fail "the refused definitions were written"
[ "$st_prof" -eq 1 ] && [ "$prof" = $'rtfs_a 12 0\nrtfs_b missing' ] || fail "the probe counts: status $st_prof, '$prof'"
[ "$st_rm" -eq 0 ] || fail "removing absent events: status $st_rm"
# Its checked existence, as tools/tsc_instance.sh's.
set +e
ex_held=$(tools/fair_server_probes.sh exists "$T" held); st_eh=$?
ex_none=$(tools/fair_server_probes.sh exists "$T" fresh); st_en=$?
ex_root=$(tools/fair_server_probes.sh exists "$T/nowrite" held 2>/dev/null); st_er=$?
chmod 000 "$T/events"
ex_shut=$(tools/fair_server_probes.sh exists "$T" held 2>/dev/null); st_es=$?
tools/fair_server_probes.sh remove "$T" held "$T/held.defs" 2>/dev/null; st_rs=$?
chmod 755 "$T/events"
set -e
[ "$st_eh" -eq 0 ] && [ "$ex_held" = present ] && [ "$st_en" -eq 0 ] && [ "$ex_none" = absent ] ||
  fail "the group's existence: '$ex_held' $st_eh, '$ex_none' $st_en"
[ "$st_er" -eq 1 ] && [ -z "$ex_root" ] || fail "no events directory answered '$ex_root', status $st_er"
if [ "$(id -u)" -ne 0 ]; then
  [ "$st_es" -eq 1 ] && [ -z "$ex_shut" ] && [ "$st_rs" -eq 1 ] ||
    fail "an unlistable events directory answered '$ex_shut', status $st_es; removal status $st_rs"
fi

# The timer mode (with the real TSC sampler): its events with the CPU
# filters, the TSC instance started inside the trace and removed, the
# sampler's pairs on both CPUs, the SMI count with its anchors, the losses in
# the record, and the lineage listing.
if [ -n "$SAMPLER" ]; then
  : >"$ROOT/fake.trace-cmd"
  TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 30 \
    --duration 2 --modes timer --runs current:100:rt-idle:fast-motion \
    </dev/null >"$ROOT/timer.log" 2>&1 || fail "the timer session failed" "$ROOT/timer.log"
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 30)
  for f in "$b" "$b.run.txt" "$b.trace.dat" "$b.tsc.txt" "$b.tsc-pairs.csv" "$b.smi.csv" "$b.procs.txt"; do
    [ -s "$f" ] || fail "missing $f" "$ROOT/timer.log"
  done
  grep -q '^mode: timer' "$b.run.txt" && grep -q '^tsc instance rtctrl_tsc: clock \[x86-tsc\]' "$b.run.txt" &&
    grep -q '^tsc instance overruns: 0 ' "$b.run.txt" && grep -q '^trace overruns: 0 ' "$b.run.txt" &&
    grep -Eq '^smi anchor monotonic_ns: [0-9]+ [0-9]+ ' "$b.run.txt" && grep -q "^session pid: " "$b.run.txt" &&
    grep -q '^kernel buffers, array main: requested 16384 KiB per CPU .*4113 subbuffers of 4096 bytes (payload 4080); expected 16387 KiB per CPU .*: read back exactly' "$b.run.txt" &&
    grep -q '^kernel buffers, array rtctrl_tsc: requested 16384 KiB per CPU .*: read back exactly' "$b.run.txt" &&
    grep -q '^kernel buffers, array rtctrl_tk: requested [0-9]* KiB per CPU .*: read back exactly' "$b.run.txt" &&
    grep -q "^kernel buffers' physical budget: per CPU main 16456 KiB, rtctrl_tsc 16456 KiB, rtctrl_tk [0-9]* KiB, .* for these 3 arrays; bookkeeping and transient allocations excluded" "$b.run.txt" ||
    fail "the timer record" "$b.run.txt"
  # The timekeeping recorder: armed before the TSC instance, confirmed and
  # finished after it, the entry probe's disable before the counts, removed
  # after; every file on the stage, published and in the ledger.
  awk '{ print $1 "_" $2 }' "$ROOT/fake.order" | tr '\n' ' ' | grep -q 'tk_start tk_states tsc_start .*tsc_stop .*tk_states tk_disable tk_profile tk_stats tk_remove' ||
    fail "the timekeeping recorder's order around the TSC instance" "$ROOT/fake.order"
  [ ! -e "$ROOT/fake.tk_recorder_exists" ] || fail "the timekeeping recorder remains"
  for f in tk-before.json tk-after.json tk-log.json tk-times.txt tk-enable-before.txt tk-enable-after.txt tk-profile.txt tk-stats.txt tk.dat tk.txt tk-journal.txt \
           tk-tool-before.json tk-tool-after.json; do
    [ -s "$b.$f" ] && grep -q "$f" "$b.staging.sha256" || fail "the recorder's $f is not published and in the ledger" "$b.staging.sha256"
  done
  # trace-cmd's identity, read through the stand-in dpkg-query as the real one would be: before the main trace and
  # after the recorder's report, the file the session runs, its SHA-256 and its package.
  tc_real=$(realpath "$FAKE/trace-cmd") tc_sha=$(sha256sum "$FAKE/trace-cmd" | cut -d' ' -f1)
  for when in "before the main trace" "after the recorder's report"; do
    grep -qxF "trace-cmd identity, $when: trace-cmd 3.3.3-1ubuntu2 amd64, sha256 $tc_sha, $tc_real" "$b.run.txt" ||
      fail "the record lacks trace-cmd's identity $when" "$b.run.txt"
  done
  python3 - "$b.tk-tool-before.json" "$tc_real" "$tc_sha" <<'EOF' || fail "the identity file" "$b.tk-tool-before.json"
import json, sys
v = json.load(open(sys.argv[1]))
sys.exit(not (v["read_status"] == "ok" and v["path"] == sys.argv[2] and v["sha256"] == sys.argv[3] and v["package"] == "trace-cmd"
              and v["libraries"] == {"libtraceevent1": "1:1.8.7-1", "libtraceevent1-plugin": "1:1.8.7-1", "libtracefs1": "1.8.2-1ubuntu1"}))
EOF
  [ ! -e "$b.tk-tmp" ] || fail "the recorder's temporary directory remains" "$b.run.txt"
  grep -Eq '^armed_t [0-9]+$' "$b.tk-times.txt" && grep -Eq '^confirmed_t [0-9]+$' "$b.tk-times.txt" || fail "the recorder's times" "$b.tk-times.txt"
  grep -q '^timekeeping recorder overruns: 0 ' "$b.run.txt" && grep -q '^main timer event enable before the TSC instance: 1$' "$b.run.txt" &&
    grep -q '^timekeeping recorder rtctrl_tk: clock' "$b.run.txt" && grep -q '^--- timekeeping recorder: problems: none ---$' "$b.run.txt" ||
    fail "the recorder in the record" "$b.run.txt"
  grep -q -- '^extract -B rtctrl_tk --temp .*\.tk-tmp -o .*\.tk\.dat$' "$ROOT/fake.trace-cmd" && grep -q -- '^report -t -i .*\.tk\.dat$' "$ROOT/fake.trace-cmd" ||
    fail "the recorder's extraction" "$ROOT/fake.trace-cmd"
  grep -q -- '^-k -b -o short-monotonic --no-pager$' "$ROOT/fake.journalctl" || fail "the kernel log's read" "$ROOT/fake.journalctl"
  grep -q -- '-e timer:hrtimer_start -e timer:hrtimer_cancel -e timer:hrtimer_setup -e msr:write_msr -f msr == 0x6e0 && CPU == 0 -e power:cpu_idle -f cpu_id == 0 -e power:cpu_idle_miss -f cpu_id == 0 -e nmi:nmi_handler -f CPU == 0 .*-e sched:sched_process_fork -e sched:sched_process_exec -e sched:sched_process_exit -e syscalls:sys_enter_clock_nanosleep -f comm == "probe" .*-e syscalls:sys_exit_nanosleep -f comm == "probe"' "$ROOT/fake.trace-cmd" ||
    fail "trace-cmd was not given the timer events" "$ROOT/fake.trace-cmd"
  grep -Eq '^clock offsets: boottime_minus_monotonic_ns -?[0-9]+ tai_minus_monotonic_ns -?[0-9]+ bracket_ns [0-9]+$' "$b.run.txt" ||
    fail "the timer record has no clock offsets" "$b.run.txt"
  # The analysis's timer-mode fixture uses only events the session records.
  rec_events=$(grep -m 1 -- '-e msr:write_msr' "$ROOT/fake.trace-cmd" | grep -oE -- '-e [a-z_-]+:[a-z_0-9]+' | sed 's/.*://' | sort -u)
  fixture_events=$(python3 tools/test_trace_correlate.py --timer-events | tr ' ' '\n')
  [ -n "$fixture_events" ] || fail "the fixture listed no events"
  for e in $fixture_events; do
    grep -qx -- "$e" <<<"$rec_events" || fail "the timer fixture uses $e, which the timer mode does not record" "$ROOT/fake.trace-cmd"
  done
  # The fair server's probes: defined once with the BTF layout, enabled in
  # the timer run with their filters, their counts around the probe in the
  # record, removed at the end.
  grep -q '^--- fair-server probes (group rtctrl_fs, defined by this session; tools/fair_server.py) ---$' "$b.run.txt" &&
    grep -q '^fair-server probe layout: rq.ext_server=2952 rq.fair_server=3192 rq.cpu=3688 ' "$b.run.txt" &&
    grep -q '^fair-server probes rtctrl_fs: 8 events defined (rtfs_dl_pick rtfs_dl_pick_ret rtfs_fs_pick rtfs_fs_pick_ret rtfs_set_next ' "$b.run.txt" &&
    grep -q '^  p:rtctrl_fs/rtfs_dl_pick pick_task_dl rq=\$arg1:x64 rq_cpu=+3688(\$arg1):s32 fs_bits=+3276(\$arg1):x32 ' "$b.run.txt" &&
    grep -q '^--- fair-server probes (hits misses) right before the probe (exit 0) ---$' "$b.run.txt" &&
    grep -q '^fair-server probe hits during the run: rtfs_dl_pick 10, .*; misses during the run: 0$' "$b.run.txt" ||
    fail "the fair server's probes in the record" "$b.run.txt"
  grep -q -- '-e rtctrl_fs:rtfs_dl_pick -f CPU == 0 -e rtctrl_fs:rtfs_dl_pick_ret -f CPU == 0 .*-e rtctrl_fs:rtfs_srv_start -f srv_cpu == 0 ' "$ROOT/fake.trace-cmd" ||
    fail "trace-cmd was not given the fair server's probe events" "$ROOT/fake.trace-cmd"
  grep -q "^define $ROOT/tracefs rtctrl_fs " "$ROOT/fake.fs_probes" && grep -q "^remove $ROOT/tracefs rtctrl_fs " "$ROOT/fake.fs_probes" &&
    [ ! -e "$ROOT/fake.fs_group_exists" ] || fail "the probes' definition and removal" "$ROOT/fake.fs_probes"
  grep -q "^start $ROOT/tracefs rtctrl_tsc 0 16384" "$ROOT/fake.tsc_instance" &&
    grep -q '^stop ' "$ROOT/fake.tsc_instance" && grep -q '^remove ' "$ROOT/fake.tsc_instance" ||
    fail "the TSC instance's start, stop and removal" "$ROOT/fake.tsc_instance"
  [ ! -e "$ROOT/fake.tsc_instance_exists" ] || fail "the TSC instance remains"
  awk -F, '!/^#/ {seen[$4] = 1} END {exit !(seen[0] && length(seen) >= 1)}' "$b.tsc-pairs.csv" ||
    fail "no sampler pairs on the real-time CPU" "$b.tsc-pairs.csv"
  [ "$(grep -c '^# tsc_mono_pairs' "$b.tsc-pairs.csv")" -eq 3 ] || fail "three sampler runs expected" "$b.tsc-pairs.csv"
  grep -q 'local_timer_entry' "$b.tsc.txt" || fail "the TSC instance's trace" "$b.tsc.txt"
  # A missing event refuses the session; so does a stale instance.
  if FAKE_MISSING_EVENT=msr:write_msr TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" \
      --yes --attempt 31 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion \
      </dev/null >"$ROOT/timer-missing.log" 2>&1; then
    fail "a timer session without msr:write_msr started" "$ROOT/timer-missing.log"
  fi
  grep -q "the timer mode's events are not available (msr:write_msr): nothing was sent" "$ROOT/timer-missing.log" ||
    fail "no refusal for the missing event" "$ROOT/timer-missing.log"
  [ ! -e "$(base current-100hz-rt-idle-bounded-fast-motion-timer 31)" ] || fail "a refused timer session ran"
  : >"$ROOT/fake.tsc_instance_exists"
  if TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 32 --duration 2 \
      --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/timer-stale.log" 2>&1; then
    fail "a timer session beside a stale instance started" "$ROOT/timer-stale.log"
  fi
  grep -q 'the trace instance rtctrl_tsc remains from an earlier session' "$ROOT/timer-stale.log" ||
    fail "no refusal for the stale instance" "$ROOT/timer-stale.log"
  rm -f "$ROOT/fake.tsc_instance_exists"
  # A failed removal fails an otherwise successful session (status 3).
  set +e
  FAKE_TSC_REMOVE_FAIL=1 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 33 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/timer-rmfail.log" 2>&1
  st=$?
  set -e
  [ "$st" -eq 3 ] && grep -q 'CLEANUP FAILED: the trace instance rtctrl_tsc was not removed' "$ROOT/timer-rmfail.log" ||
    fail "a failed removal gave status $st" "$ROOT/timer-rmfail.log"
  grep -q 'cleanup: trace instance rtctrl_tsc NOT removed' "$(base current-100hz-rt-idle-bounded-fast-motion-timer 33).run.txt" ||
    fail "the record lacks the failed removal" "$ROOT/timer-rmfail.log"
  rm -f "$ROOT/fake.tsc_instance_exists"
  # Loss counts that cannot be read are not counts of zero: a failed read
  # of each buffer's stats, a missing CPU and a malformed counter stop the
  # session with the count unknown.
  for c in "35 FAKE_STATS=fail trace overruns: unknown (trace-cmd report --stat exited 1)" \
           "36 FAKE_STATS=nocpu trace overruns: unknown (no stats for CPU" \
           "37 FAKE_STATS=bad trace overruns: unknown (CPU 0 overrun is \"n/a\")" \
           "38 FAKE_TSC_STATS=fail tsc instance overruns: unknown (the stats read exited 1)"; do
    read -r att envv want <<<"$c"
    set +e
    env "$envv" TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt "$att" \
      --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/timer-stats-$att.log" 2>&1
    st=$?
    set -e
    b=$(base current-100hz-rt-idle-bounded-fast-motion-timer "$att")
    [ "$st" -eq 1 ] && grep -qF -- "$want" "$b.run.txt" && grep -q '^verdict: tracer failure (.*loss counts' "$b.run.txt" ||
      fail "$envv: status $st, or the record lacks '$want' and the tracer failure" "$b.run.txt"
    # The buffer whose read failed has no count of zero.
    if grep -q "^${want%%: *}: 0 " "$b.run.txt"; then fail "$envv: the record still claims zero losses" "$b.run.txt"; fi
  done
  # A positive loss count in either buffer invalidates the capture: a tracer
  # failure that stops the session, never a timing verdict, and the next
  # run does not start.
  for c in "42 FAKE_TRACE_OVERRUN=7 trace overruns: 7 |the trace lost 7 events: the capture is invalid" \
           "43 FAKE_TSC_OVERRUN=5 tsc instance overruns: 5 |the TSC instance lost 5 events: the capture is invalid"; do
    read -r att envv rest <<<"$c"
    want=${rest%%|*} why=${rest#*|}
    set +e
    env "$envv" TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt "$att" \
      --duration 2 --modes timer --runs current:100:rt-idle:fast-motion,current:100:rt-idle:full \
      </dev/null >"$ROOT/timer-lost-$att.log" 2>&1
    st=$?
    set -e
    b=$(base current-100hz-rt-idle-bounded-fast-motion-timer "$att")
    [ "$st" -eq 1 ] && grep -qF -- "$want(" "$b.run.txt" && grep -qF -- "verdict: tracer failure ($why)" "$b.run.txt" ||
      fail "$envv: status $st, or the record lacks '$want' and the tracer failure" "$b.run.txt"
    if grep -qE '^verdict: (pass|fail)' "$b.run.txt"; then fail "$envv: a capture that lost events got a timing verdict" "$b.run.txt"; fi
    [ ! -e "$(base current-100hz-rt-idle-bounded-timer "$att").run.txt" ] ||
      fail "$envv: a run started after the lost events" "$ROOT/timer-lost-$att.log"
    [ ! -e "$ROOT/fake.tsc_instance_exists" ] || fail "$envv: the TSC instance remains" "$ROOT/timer-lost-$att.log"
  done
  # The instance's ownership: another session's instance, made after the
  # session's check, is left alone; one this session's start made and then
  # failed is removed; one after a status that does not say is left and
  # reported.
  for c in "39 race 1 left" "40 partial 1 removed" "41 unknown 1 left"; do
    read -r att how want_st want_mark <<<"$c"
    rm -f "$ROOT/fake.tsc_instance_exists"
    set +e
    FAKE_TSC_START=$how TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
      --attempt "$att" --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/timer-own-$how.log" 2>&1
    st=$?
    set -e
    [ "$st" -eq "$want_st" ] || fail "start $how: status $st" "$ROOT/timer-own-$how.log"
    if [ "$want_mark" = left ]; then
      [ -e "$ROOT/fake.tsc_instance_exists" ] || fail "start $how: the instance was removed" "$ROOT/timer-own-$how.log"
    else
      [ ! -e "$ROOT/fake.tsc_instance_exists" ] || fail "start $how: the instance remains" "$ROOT/timer-own-$how.log"
    fi
  done
  grep -q 'exists after a start whose outcome is unknown; it was left in place' "$ROOT/timer-own-unknown.log" ||
    fail "the unknown start's instance was not reported" "$ROOT/timer-own-unknown.log"
  rm -f "$ROOT/fake.tsc_instance_exists"
  # The timekeeping recorder: one left by an earlier session refuses the
  # session; ownership follows its start's status; a failed removal is
  # status 3; each failed step is a tracer failure, never an empty success;
  # an interruption collects what it can and still removes it.
  : >"$ROOT/fake.tk_recorder_exists"
  if TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 60 --duration 2 \
      --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/tk-stale.log" 2>&1; then
    fail "a timer session beside a stale recorder started" "$ROOT/tk-stale.log"
  fi
  grep -q 'the timekeeping recorder rtctrl_tk remains from an earlier session' "$ROOT/tk-stale.log" && [ -e "$ROOT/fake.tk_recorder_exists" ] ||
    fail "no refusal for the stale recorder, or it was removed" "$ROOT/tk-stale.log"
  rm -f "$ROOT/fake.tk_recorder_exists"
  for c in "61 race left" "62 partial removed" "63 unknown left"; do
    read -r att how want_mark <<<"$c"
    rm -f "$ROOT/fake.tk_recorder_exists"
    set +e
    FAKE_TK_START=$how TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
      --attempt "$att" --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/tk-own-$how.log" 2>&1
    st=$?
    set -e
    [ "$st" -eq 1 ] && grep -q '^verdict: not run (timekeeping recorder)' "$(base current-100hz-rt-idle-bounded-fast-motion-timer "$att").run.txt" ||
      fail "recorder start $how: status $st" "$ROOT/tk-own-$how.log"
    [ ! -e "$ROOT/fake.tsc_instance_exists" ] || fail "recorder start $how: a TSC instance was left" "$ROOT/tk-own-$how.log"
    if [ "$want_mark" = left ]; then
      [ -e "$ROOT/fake.tk_recorder_exists" ] || fail "recorder start $how: it was removed" "$ROOT/tk-own-$how.log"
    else
      [ ! -e "$ROOT/fake.tk_recorder_exists" ] || fail "recorder start $how: it remains" "$ROOT/tk-own-$how.log"
    fi
  done
  grep -q 'exists after a start whose outcome is unknown; it was left in place' "$ROOT/tk-own-unknown.log" ||
    fail "the unknown recorder start was not reported" "$ROOT/tk-own-unknown.log"
  rm -f "$ROOT/fake.tk_recorder_exists"
  # An unknown start, then existence checks that fail as sudo does (status
  # 1, which the helpers once answered for absence): nothing is removed, the
  # recorder stays owed and is checked again at every cleanup and at exit,
  # the record names it once, and the exit counts the unresolved cleanup.
  # The same for the TSC instance; the fair server's group is the next block's.
  for c in "72 tk tk_recorder FAKE_TK_START=unknown FAKE_TK_EXISTS_FAIL=2 the timekeeping recorder rtctrl_tk|instance $ROOT/tracefs/instances/rtctrl_tk, probe group $ROOT/tracefs/events/rtctrl_tk|the timekeeping recorder" \
           "73 tsc tsc_instance FAKE_TSC_START=unknown FAKE_TSC_EXISTS_FAIL=2 the trace instance rtctrl_tsc|$ROOT/tracefs/instances/rtctrl_tsc|the trace instance"; do
    read -r att who tool start_env fail_env rest <<<"$c"
    what=${rest%%|*} rest=${rest#*|}
    ident=${rest%%|*} exit_what=${rest#*|}
    tool_log="$ROOT/fake.$tool" mark="$ROOT/fake.${tool}_exists"
    rm -f "$ROOT/fake.tk_recorder_exists" "$ROOT/fake.tsc_instance_exists" "$ROOT/fake.tk_exists_calls" "$ROOT/fake.tsc_exists_calls"
    n_rm=$(grep -c '^remove ' "$tool_log" || true)
    set +e
    env "$start_env" "$fail_env" TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
      --attempt "$att" --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/unk-fail-$who.log" 2>&1
    st=$?
    set -e
    b=$(base current-100hz-rt-idle-bounded-fast-motion-timer "$att")
    [ "$st" -eq 1 ] && grep -q '^verdict: not run' "$b.run.txt" || fail "$who: unknown start, failed check: status $st" "$ROOT/unk-fail-$who.log"
    grep -qxF "cleanup: $what UNRESOLVED (the outcome of its start unknown; its existence not confirmed, unknown (exit 1: sudo: a stand-in failure)): $ident" "$b.run.txt" ||
      fail "$who: the record lacks the unresolved cleanup with its identity" "$b.run.txt"
    [ "$(grep -c 'UNRESOLVED' "$b.run.txt")" -eq 1 ] || fail "$who: the unresolved cleanup recorded more than once" "$b.run.txt"
    grep -qF "CLEANUP UNRESOLVED: whether $what exists after a start whose outcome is unknown was not confirmed" "$ROOT/unk-fail-$who.log" &&
      grep -qF "the session fails: its cleanup did not remove $exit_what" "$ROOT/unk-fail-$who.log" ||
      fail "$who: the unresolved cleanup was not reported, or not counted at exit" "$ROOT/unk-fail-$who.log"
    [ "$(cat "$ROOT/fake.${who}_exists_calls")" -ge 4 ] || fail "$who: the obligation was not checked again at exit" "$ROOT/unk-fail-$who.log"
    [ -e "$mark" ] || fail "$who: what was not confirmed was removed"
    [ "$(grep -c '^remove ' "$tool_log" || true)" -eq "$n_rm" ] || fail "$who: a removal was sent for what may be another session's" "$tool_log"
  done
  rm -f "$ROOT/fake.tk_recorder_exists" "$ROOT/fake.tsc_instance_exists"
  # A check that fails once and then answers absent ends the obligation, and says so.
  rm -f "$ROOT/fake.tk_exists_calls"
  set +e
  FAKE_TK_START=unknown-none FAKE_TK_EXISTS_FAIL=2-2 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 74 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/unk-later.log" 2>&1
  st=$?
  set -e
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 74)
  [ "$st" -eq 1 ] && grep -q 'cleanup: the timekeeping recorder rtctrl_tk UNRESOLVED' "$b.run.txt" &&
    grep -qxF "cleanup: the timekeeping recorder rtctrl_tk (instance $ROOT/tracefs/instances/rtctrl_tk, probe group $ROOT/tracefs/events/rtctrl_tk) confirmed absent" "$b.run.txt" ||
    fail "a later answer of absence: status $st" "$b.run.txt"
  if grep -q 'its cleanup did not remove the timekeeping recorder' "$ROOT/unk-later.log"; then
    fail "a recorder confirmed absent still failed the cleanup" "$ROOT/unk-later.log"
  fi
  # Before anything is sent, a check that does not answer refuses the session.
  for c in "75 FAKE_TSC_EXISTS_FAIL=1 the trace instance rtctrl_tsc" "76 FAKE_TK_EXISTS_FAIL=1 the timekeeping recorder rtctrl_tk" \
           "77 FAKE_FS_EXISTS_FAIL=1 the probe group rtctrl_fs"; do
    read -r att envv what <<<"$c"
    rm -f "$ROOT/fake.tsc_exists_calls" "$ROOT/fake.tk_exists_calls" "$ROOT/fake.fs_exists_calls"
    set +e
    env "$envv" TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt "$att" --duration 2 \
      --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/pre-unk-$att.log" 2>&1
    st=$?
    set -e
    [ "$st" -eq 2 ] && grep -qF "$what was not confirmed absent, unknown (exit 1: sudo: a stand-in failure): nothing was sent" "$ROOT/pre-unk-$att.log" &&
      [ ! -e "$(base current-100hz-rt-idle-bounded-fast-motion-timer "$att").run.txt" ] ||
      fail "$envv: an unanswered check before the session: status $st" "$ROOT/pre-unk-$att.log"
  done
  rm -f "$ROOT/fake.tsc_exists_calls" "$ROOT/fake.tk_exists_calls" "$ROOT/fake.fs_exists_calls"
  # A failed removal is status 3, and no further run starts while the recorder is owed.
  set +e
  FAKE_TK_REMOVE_FAIL=1 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 64 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion,current:100:rt-idle:full \
    </dev/null >"$ROOT/tk-rmfail.log" 2>&1
  st=$?
  set -e
  [ "$st" -eq 3 ] && grep -q 'CLEANUP FAILED: the timekeeping recorder rtctrl_tk was not removed' "$ROOT/tk-rmfail.log" &&
    grep -q 'cleanup: timekeeping recorder rtctrl_tk NOT removed' "$(base current-100hz-rt-idle-bounded-fast-motion-timer 64).run.txt" &&
    grep -q 'the trace instance or the timekeeping recorder was not removed: no further run' "$ROOT/tk-rmfail.log" &&
    [ ! -e "$(base current-100hz-rt-idle-bounded-timer 64).run.txt" ] ||
    fail "a failed recorder removal gave status $st, or another run started" "$ROOT/tk-rmfail.log"
  rm -f "$ROOT/fake.tk_recorder_exists" "$ROOT/fake.tk_states"
  for c in "65 FAKE_TK_STATES=off the probes did not read back enabled at the end" \
           "66 FAKE_TK_EXTRACT=fail the record was not extracted" \
           "67 FAKE_TK_STATS=fail the recorder's buffer stats were not read" \
           "68 FAKE_TK_OVERRUN=3 the timekeeping recorder lost 3 events: the capture is invalid" \
           "69 FAKE_JOURNAL=fail the kernel log was not read" \
           "79 FAKE_DPKG_FAIL_FROM=4 trace-cmd's identity was not read after the recorder's report"; do
    read -r att envv why <<<"$c"
    rm -f "$ROOT/fake.tk_states" "$ROOT/fake.dpkg_calls"
    set +e
    env "$envv" TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt "$att" \
      --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/tk-fail-$att.log" 2>&1
    st=$?
    set -e
    b=$(base current-100hz-rt-idle-bounded-fast-motion-timer "$att")
    [ "$st" -eq 1 ] && grep -qF -- "$why" "$b.run.txt" && grep -q '^verdict: tracer failure' "$b.run.txt" ||
      fail "$envv: status $st, or the record lacks '$why' and the tracer failure" "$b.run.txt"
    if grep -qE '^verdict: (pass|fail)' "$b.run.txt"; then fail "$envv: a timing verdict" "$b.run.txt"; fi
    [ ! -e "$ROOT/fake.tk_recorder_exists" ] || fail "$envv: the recorder remains" "$b.run.txt"
    [ ! -e "$ROOT/fake.tsc_instance_exists" ] || fail "$envv: the TSC instance remains" "$b.run.txt"
  done
  rm -f "$ROOT/fake.tk_states" "$ROOT/fake.dpkg_calls"
  grep -qF "trace-cmd identity, after the recorder's report: not read (the package that installed $(realpath "$FAKE/trace-cmd") is not known (dpkg-query: no path found matching pattern" \
    "$(base current-100hz-rt-idle-bounded-fast-motion-timer 79).run.txt" || fail "the unread identity is not in the record" "$(base current-100hz-rt-idle-bounded-fast-motion-timer 79).run.txt"
  # trace-cmd's identity unread before the run: nothing is started, neither the trace nor the recorder.
  n_rec=$(grep -c '^record ' "$ROOT/fake.trace-cmd" || true) n_tk=$(grep -c '^start ' "$ROOT/fake.tk_recorder" || true)
  set +e
  FAKE_DPKG_FAIL_FROM=1 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 80 --duration 2 \
    --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/tk-tool.log" 2>&1
  st=$?
  set -e
  rm -f "$ROOT/fake.dpkg_calls"
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 80)
  [ "$st" -eq 1 ] && grep -q '^verdict: not run (trace-cmd identity)$' "$b.run.txt" &&
    grep -q '^trace-cmd identity, before the main trace: not read (the package that installed ' "$b.run.txt" &&
    [ "$(grep -c '^record ' "$ROOT/fake.trace-cmd" || true)" -eq "$n_rec" ] && [ "$(grep -c '^start ' "$ROOT/fake.tk_recorder" || true)" -eq "$n_tk" ] ||
    fail "an unread identity before the run: status $st, or something was started" "$b.run.txt"
  # The end's read-back failed: no confirmation is written.
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 65)
  if grep -q '^confirmed_t' "$b.tk-times.txt"; then fail "a recorder that read back off was confirmed" "$b.tk-times.txt"; fi
  # The main trace's event read as off before the TSC instance: not run.
  echo 0 >"$ROOT/tracefs/events/irq_vectors/local_timer_entry/enable"
  set +e
  TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 70 --duration 2 \
    --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/tk-main.log" 2>&1
  st=$?
  set -e
  echo 1 >"$ROOT/tracefs/events/irq_vectors/local_timer_entry/enable"
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 70)
  [ "$st" -eq 1 ] && grep -q '^main timer event enable before the TSC instance: 0$' "$b.run.txt" && grep -q '^verdict: not run (timekeeping recorder)' "$b.run.txt" &&
    [ ! -e "$ROOT/fake.tk_recorder_exists" ] || fail "a main event that was off: status $st" "$b.run.txt"
  # An interruption during the run: the recorder's evidence is collected, and it is removed.
  rm -f "$ROOT/fake.tk_states"
  TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 71 --duration 30 \
    --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/tk-int.log" 2>&1 &
  pid=$!
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 71)
  for _ in $(seq 1 300); do grep -q '^tsc sampler\|^clock pair before' "$b.run.txt" 2>/dev/null && [ -e "$ROOT/fake.tsc_instance_exists" ] && break; sleep 0.1; done
  [ -e "$ROOT/fake.tk_recorder_exists" ] || fail "the recorder was not armed for the interruption" "$ROOT/tk-int.log"
  sleep 1
  kill -TERM "$pid"
  st=0
  wait "$pid" || st=$?
  [ "$st" -eq 130 ] || fail "the interrupted recorder session exited $st" "$ROOT/tk-int.log"
  [ ! -e "$ROOT/fake.tk_recorder_exists" ] || fail "the recorder remains after the interruption" "$ROOT/tk-int.log"
  for f in tk-before.json tk-after.json tk-enable-before.txt tk-enable-after.txt tk-profile.txt tk-stats.txt tk.dat tk.txt \
           tk-tool-before.json tk-tool-after.json; do
    [ -s "$b.$f" ] || fail "the interrupted run lost the recorder's $f" "$b.run.txt"
  done
  (cd "$OUT" && sha256sum --quiet -c "$(basename "$b").staging.sha256") || fail "the interrupted recorder run's ledger" "$b.run.txt"
  # Its one reading of the fair server's counts would flip the parity that stand-in's misses go by.
  rm -f "$ROOT/fake.fs_profile_calls"
  # The fair server's probes: a group left by an earlier session, and a
  # definition the kernel refuses, refuse the session before anything is
  # sent; a partly made group is removed; one of unknown ownership (an
  # unclear status, or another session's made first) is left alone; a
  # failed removal is status 3; counts that cannot be read stop the run.
  : >"$ROOT/fake.fs_group_exists"
  if TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 44 --duration 2 \
      --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fs-stale.log" 2>&1; then
    fail "a timer session beside a stale probe group started" "$ROOT/fs-stale.log"
  fi
  grep -q 'the probe group rtctrl_fs remains from an earlier session' "$ROOT/fs-stale.log" &&
    [ -e "$ROOT/fake.fs_group_exists" ] && [ ! -e "$(base current-100hz-rt-idle-bounded-fast-motion-timer 44).run.txt" ] ||
    fail "no refusal for the stale probe group, or it was removed" "$ROOT/fs-stale.log"
  rm -f "$ROOT/fake.fs_group_exists"
  for c in "45 fail 2 absent" "46 partial 2 absent" "47 unknown 2 left" "48 race 2 left"; do
    read -r att how want_st want_mark <<<"$c"
    rm -f "$ROOT/fake.fs_group_exists"
    set +e
    FAKE_FS_DEFINE=$how TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
      --attempt "$att" --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fs-def-$how.log" 2>&1
    st=$?
    set -e
    [ "$st" -eq "$want_st" ] && grep -q "the fair server's probes were not defined" "$ROOT/fs-def-$how.log" ||
      fail "definition $how: status $st" "$ROOT/fs-def-$how.log"
    [ ! -e "$(base current-100hz-rt-idle-bounded-fast-motion-timer "$att").run.txt" ] ||
      fail "definition $how: a run started" "$ROOT/fs-def-$how.log"
    if [ "$want_mark" = left ]; then
      [ -e "$ROOT/fake.fs_group_exists" ] || fail "definition $how: the group was removed" "$ROOT/fs-def-$how.log"
    else
      [ ! -e "$ROOT/fake.fs_group_exists" ] || fail "definition $how: the group remains" "$ROOT/fs-def-$how.log"
    fi
  done
  grep -q 'exists after a definition whose outcome is unknown; it was left in place' "$ROOT/fs-def-unknown.log" &&
    grep -q 'echo -:rtctrl_fs/rtfs_dl_pick | sudo tee -a ' "$ROOT/fs-def-unknown.log" ||
    fail "the unknown definition's group was not reported with its manual removal" "$ROOT/fs-def-unknown.log"
  rm -f "$ROOT/fake.fs_group_exists"
  # An unknown definition, then existence checks that fail as sudo does:
  # the group is not removed, stays owed through the exit, and is reported
  # with its identity and its manual removal.
  rm -f "$ROOT/fake.fs_exists_calls"
  n_rm=$(grep -c '^remove ' "$ROOT/fake.fs_probes" || true)
  set +e
  FAKE_FS_DEFINE=unknown FAKE_FS_EXISTS_FAIL=2 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 78 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fs-unk-fail.log" 2>&1
  st=$?
  set -e
  [ "$st" -eq 2 ] && grep -qF "CLEANUP UNRESOLVED: whether the probe group rtctrl_fs exists after a definition whose outcome is unknown was not confirmed, unknown (exit 1: sudo: a stand-in failure); nothing was removed: $ROOT/tracefs/events/rtctrl_fs" "$ROOT/fs-unk-fail.log" &&
    [ "$(grep -c 'CLEANUP UNRESOLVED' "$ROOT/fs-unk-fail.log")" -eq 1 ] &&
    grep -q 'echo -:rtctrl_fs/rtfs_dl_pick | sudo tee -a ' "$ROOT/fs-unk-fail.log" &&
    grep -q "the session fails: its cleanup did not remove the fair server's probes" "$ROOT/fs-unk-fail.log" ||
    fail "an unknown definition and a failed check: status $st, or not reported and counted" "$ROOT/fs-unk-fail.log"
  [ -e "$ROOT/fake.fs_group_exists" ] && [ "$(cat "$ROOT/fake.fs_exists_calls")" -ge 3 ] &&
    [ "$(grep -c '^remove ' "$ROOT/fake.fs_probes" || true)" -eq "$n_rm" ] ||
    fail "an unconfirmed group was removed, or not checked again at exit" "$ROOT/fs-unk-fail.log"
  rm -f "$ROOT/fake.fs_group_exists" "$ROOT/fake.fs_exists_calls"
  set +e
  FAKE_FS_REMOVE_FAIL=1 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 49 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fs-rmfail.log" 2>&1
  st=$?
  set -e
  [ "$st" -eq 3 ] && grep -q "CLEANUP FAILED: the fair server's probes (rtctrl_fs) were not removed" "$ROOT/fs-rmfail.log" &&
    grep -q 'cleanup: probe group rtctrl_fs NOT removed' "$(base current-100hz-rt-idle-bounded-fast-motion-timer 49).run.txt" ||
    fail "a failed probe removal gave status $st" "$ROOT/fs-rmfail.log"
  rm -f "$ROOT/fake.fs_group_exists"
  set +e
  FAKE_FS_PROFILE=fail TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 51 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion,current:100:rt-idle:full \
    </dev/null >"$ROOT/fs-prof.log" 2>&1
  st=$?
  set -e
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 51)
  [ "$st" -eq 1 ] && grep -qF "verdict: tracer failure (the fair server's probe counts were not read)" "$b.run.txt" &&
    [ ! -e "$(base current-100hz-rt-idle-bounded-timer 51).run.txt" ] && [ ! -e "$ROOT/fake.fs_group_exists" ] ||
    fail "unread probe counts: status $st, or the run was not stopped, or the group remains" "$b.run.txt"
  # Misses are counted in the record.
  FAKE_FS_MISSES=3 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 52 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fs-miss.log" 2>&1 ||
    fail "the session with probe misses failed" "$ROOT/fs-miss.log"
  grep -q '; misses during the run: 3$' "$(base current-100hz-rt-idle-bounded-fast-motion-timer 52).run.txt" ||
    fail "the record lacks the probe misses" "$(base current-100hz-rt-idle-bounded-fast-motion-timer 52).run.txt"
  # The probes' controls: refused without the timer mode, without --port and
  # on an excluded CPU; planned by a dry run; over the emulator they run
  # before the timer runs, their tracer off the control CPU. Without a real
  # trace (the stand-in records nothing) and without a real-time loop (the
  # tool's no-rt test mode), the positive control fails: the session stops
  # before any timer run and removes the probes.
  for c in "--modes traced|needs the timer mode" "--exclude-cpus 0,1|is in the excluded set"; do
    extra=${c%%|*} words=${c#*|}
    # shellcheck disable=SC2086
    if TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 53 --duration 2 \
        --modes timer $extra --fs-controls 1 --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fc-refused.log" 2>&1; then
      fail "--fs-controls with $extra was accepted" "$ROOT/fc-refused.log"
    fi
    grep -q "$words" "$ROOT/fc-refused.log" || fail "no refusal ($words)" "$ROOT/fc-refused.log"
  done
  if TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh --probe "$FAKE/probe" --outdir "$OUT" --date "$DATE" \
      --rt-cpu 0 --exclude-cpus 0 --dry-run --fs-controls 1 --modes timer --runs current:100:rt-idle:full \
      </dev/null >"$ROOT/fc-noport.log" 2>&1; then
    fail "--fs-controls without --port was accepted" "$ROOT/fc-noport.log"
  fi
  grep -q -- '--fs-controls is for the rehearsal: pass the emulator' "$ROOT/fc-noport.log" ||
    fail "no --fs-controls refusal without --port" "$ROOT/fc-noport.log"
  if [ "$(nproc)" -ge 3 ]; then
    TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --dry-run --attempt 53 --duration 2 \
      --fs-controls 1 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fc-plan.log" 2>&1 ||
      fail "the controls' dry run failed" "$ROOT/fc-plan.log"
    grep -q "^before the runs: the fair server's probe controls on CPU 1 (supervisor CPU 2, tracers on CPUs 2," "$ROOT/fc-plan.log" ||
      fail "the dry run does not plan the controls" "$ROOT/fc-plan.log"
    : >"$ROOT/fake.trace-affinity"
    set +e
    FAIR_SERVER_CONTROL_TEST=no-rt FS_CONTROL_PHASE_S=0.3 FS_CONTROL_SETTLE_S=0.1 TRACE_SESSION_RTPRIO_LIMIT=99 \
      setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 53 --duration 2 --fs-controls 1 \
      --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fc.log" 2>&1
    st=$?
    set -e
    fb="$OUT/fs-controls-$DATE-53"
    [ "$st" -eq 1 ] && [ -s "$fb.controls.txt" ] && [ -s "$fb.trace.dat" ] || fail "the controls' session: status $st" "$ROOT/fc.log"
    grep -q '^control CPU: 1; supervisor CPU: 2; tracers on CPUs 2,' "$fb.run.txt" &&
      grep -q -- '-e rtctrl_fs:rtfs_dl_pick -f CPU\\ ==\\ 1 ' "$fb.run.txt" &&
      grep -q -- '-e rtctrl_fs:rtfs_srv_stop -f srv_cpu\\ ==\\ 1 ' "$fb.run.txt" &&
      grep -q '^fair-server probe hits during the controls: rtfs_dl_pick 10, .*; misses during the controls: 0$' "$fb.run.txt" &&
      grep -q '^trace overruns: 0 ' "$fb.run.txt" &&
      grep -q '^  positive: FAIL (the loop (pid [0-9]*) did not run at a real-time priority on CPU 1 in the trace)$' "$fb.run.txt" &&
      grep -q "^  negative-normal: INCONCLUSIVE (preconditions not established: no state of the server was read at a deadline-class pick after its last start, stop or timer call" "$fb.run.txt" &&
      grep -q '^verdict: controls fail$' "$fb.run.txt" ||
      fail "the controls' record" "$fb.run.txt"
    grep -q '^phase negative-idle ' "$fb.controls.txt" && grep -q '^# fifo: test mode no-rt, no SCHED_FIFO$' "$fb.controls.txt" ||
      fail "the controls' output" "$fb.controls.txt"
    if grep -q '^affinity .*\b1\b' "$ROOT/fake.trace-affinity"; then
      fail "the controls' tracer may run on the control CPU" "$ROOT/fake.trace-affinity"
    fi
    [ ! -e "$(base current-100hz-rt-idle-bounded-fast-motion-timer 53).run.txt" ] && [ ! -e "$ROOT/fake.fs_group_exists" ] ||
      fail "a timer run followed failed controls, or the probes remain" "$ROOT/fc.log"
    # An earlier attempt's controls are never reused: a failed record, a
    # partial output and an interrupted trace each refuse the session before
    # anything is sent, and no probe runs (reviewer, 2026-11-08).
    for c in "55 run.txt verdict: controls fail" "56 controls.txt phase positive 1 2" "57 trace.dat stand-in trace data"; do
      read -r att suffix content <<<"$c"
      echo "$content" >"$OUT/fs-controls-$DATE-$att.$suffix"
      set +e
      FAIR_SERVER_CONTROL_TEST=no-rt FS_CONTROL_PHASE_S=0.3 FS_CONTROL_SETTLE_S=0.1 TRACE_SESSION_RTPRIO_LIMIT=99 \
        setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt "$att" --duration 2 --fs-controls 1 \
        --modes untraced,timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/fc-retry-$att.log" 2>&1
      st=$?
      set -e
      [ "$st" -eq 2 ] && grep -q "the fair-server controls of this attempt were attempted earlier ($OUT/fs-controls-$DATE-$att.$suffix): controls are never reused; pass a free --attempt" "$ROOT/fc-retry-$att.log" ||
        fail "an earlier attempt's controls ($suffix): status $st" "$ROOT/fc-retry-$att.log"
      [ -z "$(ls "$OUT" | grep -E -- "-$DATE-$att\.csv")" ] && [ "$(ls "$OUT"/fs-controls-$DATE-$att.* | wc -l)" -eq 1 ] ||
        fail "a session with an earlier attempt's controls ($suffix) ran something" "$ROOT/fc-retry-$att.log"
    done
  fi
  # The exerciser: refused without --port; over the emulator it runs.
  if TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh --probe "$FAKE/probe" --outdir "$OUT" --date "$DATE" \
      --rt-cpu 0 --exclude-cpus 0 --dry-run --exerciser --modes timer --runs current:100:rt-idle:full \
      </dev/null >"$ROOT/ex-noport.log" 2>&1; then
    fail "--exerciser without --port was accepted" "$ROOT/ex-noport.log"
  fi
  grep -q 'is for the rehearsal: pass the emulator' "$ROOT/ex-noport.log" || fail "no exerciser refusal" "$ROOT/ex-noport.log"
  TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes --attempt 34 --duration 2 \
    --exerciser --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/ex.log" 2>&1 ||
    fail "the exerciser session failed" "$ROOT/ex.log"
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 34)
  grep -q '^exerciser: exit 0, ' "$b.run.txt" && grep -q '^# timer_exerciser tid=' "$b.exerciser.csv" ||
    fail "the exerciser's run" "$b.run.txt"
fi

# The observer placement: refusals (a confined placement without observers,
# observers without a confined placement, an unknown placement, a missing
# process); then an open and a confined run of a stand-in observer (a
# process of this test's with two threads and a child): labelled names, the
# confined run's record of the tool's confinement, restoration and halfway
# check, the probe started off the excluded CPU, the open run left alone,
# and the observer's masks restored afterwards.
for c in "--placements confined|a confined placement needs --observers" \
         "--placements confined --observers 1 --modes untraced,traced|a confined placement runs untraced or in the timer mode" \
         "--placements open --observers 1|--observers needs --placements with confined" \
         "--placements shut --observers 1|unknown placement 'shut'" \
         "--placements open,confined --observers 999999999|--observers: no process 999999999"; do
  extra=${c%%|*} words=${c#*|}
  # shellcheck disable=SC2086
  if setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 54 --modes untraced \
      --runs current:100:idle:fast-motion $extra </dev/null >"$ROOT/pl-refused.log" 2>&1; then
    fail "the placement $extra was accepted" "$ROOT/pl-refused.log"
  fi
  grep -qF -- "$words" "$ROOT/pl-refused.log" || fail "no refusal ($words)" "$ROOT/pl-refused.log"
done
python3 -c '
import subprocess, sys, threading, time
threading.Thread(target=lambda: time.sleep(600), daemon=True).start()
subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
time.sleep(600)' &
obs_pid=$!
# A failed check must not leave the observer holding the test's output.
trap 'kill "$EMU_PID" 2>/dev/null || true; pkill -P "$obs_pid" 2>/dev/null || true; kill "$obs_pid" 2>/dev/null || true' EXIT
sleep 0.5
all_cpus=$(python3 -c 'import os; print(",".join(map(str, sorted(os.sched_getaffinity(0)))))')
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 54 --duration 4 --modes untraced \
  --runs current:100:idle:fast-motion --placements open,confined --observers "$obs_pid" \
  </dev/null >"$ROOT/pl.log" 2>&1 || fail "the placement session failed" "$ROOT/pl.log"
bo=$(base current-100hz-idle-bounded-fast-motion-obs-open-untraced 54)
bc=$(base current-100hz-idle-bounded-fast-motion-obs-confined-untraced 54)
[ -f "$bo.run.txt" ] && [ -f "$bc.run.txt" ] && [ ! -e "$bo.observers.jsonl" ] && [ -s "$bc.observers.jsonl" ] ||
  fail "the placement runs' files" "$ROOT/pl.log"
grep -q "^placement: open (observers $obs_pid left as they are)$" "$bo.run.txt" ||
  fail "the open run's record" "$bo.run.txt"
grep -q "^placement: confined (observers $obs_pid moved off CPUs 0, the probe started on CPUs " "$bc.run.txt" &&
  grep -q '^command: taskset -c [0-9,\]* .*x7_bus_timing ' "$bc.run.txt" &&
  grep -q '^--- observer placement: python3 tools/observer_affinity.py confine (exit 0) ---$' "$bc.run.txt" &&
  grep -Eq '^observer placement: 2 processes, [0-9]+ threads in [0-9]+ passes: [0-9]+ confined to CPUs ' "$bc.run.txt" &&
  grep -q '^observer placement restored: restored [0-9]*, exited 0, reused 0, changed 0' "$bc.run.txt" &&
  grep -Eq '^observer placement check \(halfway snapshot\): [0-9]+ observer threads, 0 allowed on CPUs 0; [0-9]+ probe helper threads, 0 allowed there$' "$bc.run.txt" &&
  grep -Eq '^  lineage: 2 observer processes \(the recorded ones by ID and start time' "$bc.run.txt" &&
  grep -q "^halfway snapshot: the task listing's exit 0$" "$bc.run.txt" &&
  grep -Eq '^lineage complete: [0-9]+ tasks$' "$bc.tasks.txt" &&
  grep -q '^--- task lineage (observer_affinity.py tasks: pid tid ppid pstart start allowed) ---$' "$bc.tasks.txt" &&
  grep -q '"kind": "state", "passes": [0-9]*, "state": "confined"' "$bc.observers.jsonl" &&
  grep -Eq '^verdict: (pass|fail)' "$bc.run.txt" ||
  fail "the confined run's record" "$bc.run.txt"
python3 - "$obs_pid" "$all_cpus" <<'EOF' || fail "the observer's masks were not restored" "$bc.run.txt"
import os, sys
pid, want = int(sys.argv[1]), set(map(int, sys.argv[2].split(",")))
kids = [int(p) for p in os.listdir("/proc") if p.isdigit() and open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()[1] == str(pid)]
for p in [pid] + kids:
    for t in os.listdir(f"/proc/{p}/task"):
        assert os.sched_getaffinity(int(t)) == want, (p, t, os.sched_getaffinity(int(t)))
EOF
# A confined timer run: the lineage listed right after the trace started,
# published with the run, beside the halfway check's (reviewer, 2026-11-09).
if [ -n "$SAMPLER" ]; then
  FAKE_PROBE_OWN_TID=1 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
    --attempt 89 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion --placements confined \
    --observers "$obs_pid" </dev/null >"$ROOT/pl-timer.log" 2>&1 || fail "the confined timer session failed" "$ROOT/pl-timer.log"
  b=$(base current-100hz-rt-idle-bounded-fast-motion-obs-confined-timer 89)
  grep -q '^--- task lineage (observer_affinity.py tasks: pid tid ppid pstart start allowed) ---$' "$b.lineage.txt" &&
    grep -Eq "^$obs_pid $obs_pid [0-9]+ [0-9]+ [0-9]+ [0-9,-]+$" "$b.lineage.txt" &&
    grep -Eq '^task lineage after the trace started: .*\.lineage\.txt, [0-9]+ tasks$' "$b.run.txt" &&
    grep -Eq '^observer placement check \(halfway snapshot\): [0-9]+ observer threads, 0 allowed on CPUs 0; ' "$b.run.txt" &&
    grep -q 'lineage.txt' "$b.staging.sha256" ||
    fail "the confined timer run's lineage" "$b.run.txt"
fi
pkill -P "$obs_pid" 2>/dev/null || true
kill "$obs_pid" 2>/dev/null || true
wait "$obs_pid" 2>/dev/null || true
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT

# A confine that fails (here one that does not settle) stops the session
# before the probe starts, and the observers are restored (reviewer,
# 2026-11-09).
cat >"$FAKE/obs_unsettled.py" <<'EOF'
import os, sys
with open(os.environ["FAKE_LOG"] + ".obs_unsettled", "a") as log:
    log.write(" ".join(sys.argv[1:]) + "\n")
if sys.argv[1] == "confine":
    with open(sys.argv[sys.argv.index("--record") + 1], "w") as rec:
        rec.write('{"kind": "header", "version": 2, "exclude": [0], "allowed": [1]}\n')
    print("observer placement: 1 processes, 10 threads in 10 passes: 10 confined (stand-in); NOT SETTLED")
    sys.exit(1)
elif sys.argv[1] == "restore":
    print("observer placement restored: restored 10 (stand-in)")
EOF
set +e
OBS_TOOL="$FAKE/obs_unsettled.py" setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 59 --duration 2 \
  --modes untraced --runs current:100:idle:fast-motion --placements confined --observers $$ </dev/null \
  >"$ROOT/pl-unsettled.log" 2>&1
st=$?
set -e
b=$(base current-100hz-idle-bounded-fast-motion-obs-confined-untraced 59)
[ "$st" -ne 0 ] && grep -q 'the observers were not all confined (exit 1): session stopped' "$ROOT/pl-unsettled.log" &&
  grep -q '^verdict: not run (placement)$' "$b.run.txt" && [ ! -e "$b" ] && [ ! -e "$b.summary.toml" ] &&
  ! grep -q '^worker thread ' "$b.run.txt" && grep -q '^restore ' "$ROOT/fake.obs_unsettled" ||
  fail "an unsettled confine: status $st" "$ROOT/pl-unsettled.log"

# A task listing that cannot read a live task exits 1 (an incomplete
# lineage): the halfway snapshot's status makes the confined run a placement
# failure even when the check passes, and right after a timer run's trace
# started it stops the session before the probe (reviewer, 2026-11-09).
cat >"$FAKE/obs_unreadable.py" <<'EOF'
import os, sys
with open(os.environ["FAKE_LOG"] + ".obs_unreadable", "a") as log:
    log.write(" ".join(sys.argv[1:]) + "\n")
if sys.argv[1] == "confine":
    with open(sys.argv[sys.argv.index("--record") + 1], "w") as rec:
        rec.write('{"kind": "header", "version": 2, "exclude": [0], "allowed": [1]}\n')
    print("observer placement: 1 processes, 1 threads in 2 passes: 1 confined (stand-in)")
elif sys.argv[1] == "restore":
    print("observer placement restored: restored 1 (stand-in)")
elif sys.argv[1] == "tasks":
    print("--- task lineage (observer_affinity.py tasks: pid tid ppid pstart start allowed) ---")
    print("lineage INCOMPLETE: 1 unreadable: /proc/1/task (stand-in)")
    sys.exit(1)
else:
    print("observer placement check (stand-in)")
EOF
OBS_TOOL="$FAKE/obs_unreadable.py" setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 60 --duration 2 \
  --modes untraced --runs current:100:idle:fast-motion --placements confined --observers $$ </dev/null \
  >"$ROOT/pl-unreadable.log" 2>&1 || true
b=$(base current-100hz-idle-bounded-fast-motion-obs-confined-untraced 60)
grep -q "^halfway snapshot: the task listing's exit 1$" "$b.run.txt" &&
  grep -q "^verdict: placement failure (the halfway snapshot was not complete (its task listing's exit 1))$" "$b.run.txt" ||
  fail "an incomplete halfway snapshot" "$b.run.txt"
if [ -n "$SAMPLER" ]; then
  set +e
  OBS_TOOL="$FAKE/obs_unreadable.py" FAKE_PROBE_OWN_TID=1 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh \
    "${PROFILE[@]}" --yes --attempt 90 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion \
    --placements confined --observers $$ </dev/null >"$ROOT/pl-unreadable-timer.log" 2>&1
  st=$?
  set -e
  b=$(base current-100hz-rt-idle-bounded-fast-motion-obs-confined-timer 90)
  [ "$st" -ne 0 ] && grep -q 'the task lineage was not listed after the trace started: session stopped' "$b.run.txt" &&
    grep -q '^verdict: not run (placement)$' "$b.run.txt" && ! grep -q '^worker thread ' "$b.run.txt" &&
    [ "$(grep -c '^restore ' "$ROOT/fake.obs_unreadable")" -ge 2 ] ||
    fail "an incomplete lineage after the trace started: status $st" "$b.run.txt"
fi

# A restoration that stays unresolved (a stand-in placement tool whose
# restore fails) reaches the session's cleanup-failure handling: the run
# stops as a placement failure, the cleanup is retried at exit and reported,
# and the record says the placement was not restored (reviewer, 2026-11-08).
# (The session runs the tool with python3.)
cat >"$FAKE/obs_tool.py" <<'EOF'
import os, sys
with open(os.environ["FAKE_LOG"] + ".obs_tool", "a") as log:
    log.write(" ".join(sys.argv[1:]) + "\n")
if sys.argv[1] == "confine":
    with open(sys.argv[sys.argv.index("--record") + 1], "w") as rec:
        rec.write('{"kind": "header", "exclude": [0], "allowed": [1]}\n')
    print("observer placement: 1 processes, 1 threads: 1 confined (stand-in)")
elif sys.argv[1] == "restore":
    print("observer placement restored: failed 1; INCOMPLETE: the record stays, retry the restore")
    print("observer_affinity: thread 1 not restored: stand-in EPERM", file=sys.stderr)
    sys.exit(1)
else:
    print("observer placement check (stand-in)")
EOF
set +e
OBS_TOOL="$FAKE/obs_tool.py" setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 58 --duration 2 --modes untraced \
  --runs current:100:idle:fast-motion --placements confined --observers $$ </dev/null >"$ROOT/pl-rmfail.log" 2>&1
st=$?
set -e
b=$(base current-100hz-idle-bounded-fast-motion-obs-confined-untraced 58)
[ "$st" -ne 0 ] && grep -q "CLEANUP FAILED: the observers' placement was not restored" "$ROOT/pl-rmfail.log" &&
  grep -q "retry: python3 $FAKE/obs_tool.py restore --record $b.observers.jsonl" "$ROOT/pl-rmfail.log" &&
  grep -q "^verdict: placement failure (the observers' placement was not restored)$" "$b.run.txt" &&
  grep -q '^cleanup: observer placement NOT restored: ' "$b.run.txt" &&
  [ "$(grep -c '^restore ' "$ROOT/fake.obs_tool")" -ge 2 ] ||
  fail "an unresolved restoration: status $st" "$ROOT/pl-rmfail.log"

# An emulator rehearsal traces without usbmon.
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 4 --no-usbmon \
  --modes traced --runs current:100:idle:fast </dev/null >"$ROOT/nousb.log" 2>&1 ||
  fail "the session without usbmon failed" "$ROOT/nousb.log"
b=$(base current-100hz-idle-bounded-fast-traced 4)
[ -s "$b.trace.dat" ] && [ ! -e "$b.usbmon.pcap" ] || fail "--no-usbmon captured usbmon" "$b.run.txt"
grep -q '^usbmon: not used (--no-usbmon)' "$b.run.txt"
grep -q '^temporary host changes: none recorded by tools/quiet_host.sh$' "$b.run.txt" ||
  fail "the record does not say that no host changes were recorded" "$b.run.txt"
# tools/quiet_host.sh's record of its temporary host changes goes into the
# run record.
printf 'started=2000-01-01T00:00:00+0000\nunattended-upgrades.service_stopped=1\n' >"$QUIET_HOST_STATE"
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 50 --no-usbmon \
  --modes traced --runs current:100:idle:fast </dev/null >"$ROOT/qh-run.log" 2>&1 ||
  fail "the session with a quiet_host record failed" "$ROOT/qh-run.log"
rm -f "$QUIET_HOST_STATE"
grep -q "^temporary host changes (tools/quiet_host.sh, $QUIET_HOST_STATE): started=2000-01-01T00:00:00+0000 unattended-upgrades.service_stopped=1" \
  "$(base current-100hz-idle-bounded-fast-traced 50).run.txt" || fail "the record lacks the host changes" "$(base current-100hz-idle-bounded-fast-traced 50).run.txt"

# Without a USB adapter behind the port, a traced plan names the problem.
setsid tools/trace_session.sh --probe "$PROBE" --port "$LINK" --outdir "$OUT" \
  --rt-cpu 0 --exclude-cpus 0 --runs current:100:idle:full --dry-run \
  </dev/null >"$ROOT/nodev.log" 2>&1 ||
  fail "the plan without an adapter failed" "$ROOT/nodev.log"
grep -q 'no USB device to capture' "$ROOT/nodev.log"

# The default plan: seven pairs, the -excl load pinned off the excluded
# CPUs, the probe on its real-time CPU.
setsid tools/trace_session.sh "${COMMON[@]}" --dry-run </dev/null >"$ROOT/plan.log" 2>&1 ||
  fail "the default plan failed" "$ROOT/plan.log"
grep -q '^plan: 14 runs' "$ROOT/plan.log"
grep -q 'load: taskset -c 1[0-9,]* [^ ]*stress-ng --cpu [0-9]* --hdd 1' "$ROOT/plan.log"
grep -q -- "rate-trace-current-1000hz-rt-loaded-excl-bounded-fast-motion-traced-$DATE-1.csv --rt-priority 80 --rt-cpu 0" "$ROOT/plan.log"
grep -q "rate-trace-cbp-1000hz-rt-idle-bounded-fast-untraced-$DATE-1.csv" "$ROOT/plan.log"


# The runs' staging (tools/trace_stage.py). Every session above ran its runs
# on a stand-in stage with its bounded capacity, untraced ones included,
# without a guard stop; here, the records of an untraced and a traced run,
# and the paths that refuse, stop or keep a stage.
stage_left() { ls -A "$ROOT/stage" 2>/dev/null; }
[ -z "$(stage_left)" ] && [ ! -s "$ROOT/mounts" ] || fail "a stage remains after the sessions above" "$ROOT/mounts"
for mode in untraced traced; do
  b=$(base "current-100hz-idle-bounded-fast-motion-$mode" 1)
  note="$b.run.txt"
  grep -q '^staging budget (' "$note" && grep -q 'guard reserve 64 MiB' "$note" &&
    grep -q "^staging: the mount point $ROOT/stage/[^ ]*, the operator's with mode 0700 ($(id -u):$(id -g):700)$" "$note" &&
    grep -q "^staging: verified: $ROOT/stage/" "$note" &&
    grep -q 'noswap' "$note" && grep -q "^staging: marked: $ROOT/stage/" "$note" &&
    grep -q "^logging: the probe's log and every capture stream go to the stage $ROOT/stage/" "$note" &&
    grep -q '^--- staging guard ---' "$note" && grep -q 'stop: none$' "$note" &&
    grep -q '^published: ' "$note" && grep -q '^preservation: published and verified, the stage released$' "$note" &&
    grep -q '^--- memory and writeback counters (system-wide, read-only' "$note" &&
    grep -q '^vm counter changes (after - before; ' "$note" ||
    fail "the $mode run's staging record" "$note"
  (cd "$OUT" && sha256sum --quiet -c "$(basename "$b").staging.sha256") || fail "the $mode run's ledger" "$note"
  [ -f "$b.staging.inventory.json" ] || fail "the $mode run's inventory" "$note"
done
grep -q "^kernel buffers' geometry, array main: subbuffer 4096 bytes, payload 4080 at offset 16 (page header, buffer_subbuf_size_kb) (read before any run)" "$(base current-100hz-idle-bounded-fast-motion-traced 1).run.txt" &&
  grep -q '^staging budget (traced, .*kernel buffers 0.54 GB (1 arrays x [0-9]* CPUs x 16456 KiB, 4113 subbuffers plus the reader.s of 4096 bytes; .*expected readback 16387 KiB per CPU for 16384 KiB requested)' "$(base current-100hz-idle-bounded-fast-motion-traced 1).run.txt" &&
  grep -q '^kernel buffers, array main: .*: read back exactly' "$(base current-100hz-idle-bounded-fast-motion-traced 1).run.txt" &&
  grep -q "^staging: trace-cmd's per-CPU temporary files: 1 in $ROOT/stage/" "$(base current-100hz-idle-bounded-fast-motion-traced 1).run.txt" &&
  grep -q '^staging usage: ' "$(base current-100hz-idle-bounded-fast-motion-traced 1).run.txt" ||
  fail "the traced run's kernel buffers, temporary files and usage" "$(base current-100hz-idle-bounded-fast-motion-traced 1).run.txt"
# The guard's record once per run, the session's last run included.
for f in "$(base current-100hz-idle-bounded-fast-motion-traced 1).run.txt" "$(base position-100hz-idle-bounded-traced 1).run.txt"; do
  [ "$(grep -c '^--- staging guard ---$' "$f")" -eq 1 ] || fail "the guard's record is not once in $f" "$f"
done
# Every mount found its mount point the operator's, mode 0700; no session
# above had a guard stop (the stages modeled with their bounded capacity).
grep -q '(mount point ' "$FAKE_LOG.mount" && ! grep -v "(mount point $(id -u):$(id -g):700)$" "$FAKE_LOG.mount" | grep -q . ||
  fail "a mount point was not the operator's with mode 0700" "$FAKE_LOG.mount"
if grep -l '^guard stop:' "$OUT"/*.run.txt 2>/dev/null | grep -q .; then
  fail "a guard stopped a run on a stage of its budgeted size" "$(grep -l '^guard stop:' "$OUT"/*.run.txt | head -n 1)"
fi
if grep -q '^kernel buffers' "$(base current-100hz-idle-bounded-fast-motion-untraced 1).run.txt"; then
  fail "an untraced run read back trace buffers" "$(base current-100hz-idle-bounded-fast-motion-untraced 1).run.txt"
fi

# A stage the session cannot use: the run does not start, the probe never
# runs, and the mount point is undone (unmounted when mounted, removed).
for case in "71:ext4:FAKE_MOUNT_TYPE=ext4:is a ext4 mount, not tmpfs" \
            "72:swap:FAKE_MOUNT_DROP=noswap:a tmpfs without noswap" \
            "73:nomount:FAKE_MOUNT_FAIL=1:the tmpfs was not mounted"; do
  IFS=: read -r attempt name envset words <<<"$case"
  if env "$envset" setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt "$attempt" --modes traced \
      --runs current:100:idle:full </dev/null >"$ROOT/stage-$name.log" 2>&1; then
    fail "a session on a bad stage ($name) went on" "$ROOT/stage-$name.log"
  fi
  b=$(base current-100hz-idle-bounded-traced "$attempt")
  grep -q "$words" "$b.run.txt" && grep -q '^verdict: not run (staging)' "$b.run.txt" &&
    grep -q '^staging: aborted: ' "$b.run.txt" || fail "the bad stage ($name) was not refused and undone" "$b.run.txt"
  [ ! -e "$b" ] || fail "the probe ran on a bad stage ($name)" "$b.run.txt"
  [ -z "$(stage_left)" ] && [ ! -s "$ROOT/mounts" ] || fail "the bad stage ($name) was left" "$ROOT/mounts"
done

# Too little MemAvailable for the run's budget: refused before any mount.
meminfo 4000000 >"$ROOT/meminfo-low"
: >"$ROOT/fake.mount"
if TRACE_SESSION_MEMINFO="$ROOT/meminfo-low" setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 80 \
    --modes traced --runs current:100:idle:full </dev/null >"$ROOT/stage-mem.log" 2>&1; then
  fail "a session without the memory for its stage went on" "$ROOT/stage-mem.log"
fi
b=$(base current-100hz-idle-bounded-traced 80)
grep -q 'NOT ENOUGH' "$b.run.txt" && grep -q '^staging: refused: ' "$b.run.txt" && [ ! -s "$ROOT/fake.mount" ] ||
  fail "the memory refusal" "$b.run.txt"

# A stage that fills: trace-cmd fails on ENOSPC; its partial trace is
# published and verified all the same, and the capture is a tracer failure.
if FAKE_TRACE_ENOSPC=1 setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 81 --modes traced \
    --runs current:100:idle:full </dev/null >"$ROOT/stage-full.log" 2>&1; then
  fail "a session whose stage filled went on" "$ROOT/stage-full.log"
fi
b=$(base current-100hz-idle-bounded-traced 81)
grep -q '^verdict: tracer failure (.*the staging filled' "$b.run.txt" &&
  grep -q '^preservation: published and verified, the stage released$' "$b.run.txt" &&
  grep -q '^partial stand-in trace$' "$b.trace.dat" || fail "the filled stage" "$b.run.txt"
[ -z "$(stage_left)" ] || fail "the filled stage was left" "$ROOT/stage-full.log"

# The guard's memory stop: MemAvailable falls once trace-cmd records; the
# guard stops it and says why, and the run fails.
meminfo 43557304 >"$ROOT/meminfo-drop"
if FAKE_DROP_MEM=1 TRACE_SESSION_MEMINFO="$ROOT/meminfo-drop" setsid tools/trace_session.sh "${COMMON[@]}" --yes \
    --attempt 82 --duration 3 --modes traced --runs current:100:idle:full </dev/null >"$ROOT/stage-guard.log" 2>&1; then
  fail "a session whose guard stopped the recorders went on" "$ROOT/stage-guard.log"
fi
b=$(base current-100hz-idle-bounded-traced 82)
# (Stopped before the probe started, or during its run: either way the
# guard's stop is recorded and the run fails.)
grep -q '^guard stop: MemAvailable is 1024000 bytes' "$b.run.txt" &&
  grep -Eq '^verdict: (not run \(tracer\)|tracer failure \(.*the staging guard stopped the recorders \(MemAvailable is 1024000 bytes)' "$b.run.txt" ||
  fail "the guard's memory stop" "$b.run.txt"
[ -z "$(stage_left)" ] || fail "the guard's stage was left" "$ROOT/stage-guard.log"
# The guard's space stop on the stage's bounded capacity: trace-cmd's
# temporary file fills it but 1 MiB, under the bound; the guard stops it and
# says why, and the run fails.
if FAKE_TRACE_FILL=1 setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 85 --duration 3 --modes traced \
    --runs current:100:idle:full </dev/null >"$ROOT/stage-space.log" 2>&1; then
  fail "a session whose guard stopped the recorders on space went on" "$ROOT/stage-space.log"
fi
b=$(base current-100hz-idle-bounded-traced 85)
grep -Eq '^guard stop: the stage has [0-9]+ bytes free \(bound [0-9]+\)' "$b.run.txt" &&
  grep -Eq '^verdict: (not run \(tracer\)|tracer failure \(.*the staging guard stopped the recorders \(the stage has)' "$b.run.txt" &&
  grep -q '^preservation: published and verified, the stage released$' "$b.run.txt" ||
  fail "the guard's space stop" "$b.run.txt"
[ -z "$(stage_left)" ] || fail "the guard's space stage was left" "$ROOT/stage-space.log"

# trace-cmd's temporary files beside its output, not in --temp's directory:
# the run stops before the probe; and buffers above the budget's allowance.
for case in "83:temp:FAKE_IGNORE_TEMP=1:temporary files are not on the stage only" \
            "84:buffers:FAKE_BUFFER_TOTAL=999999999:buffer_total_size_kb reads 999999999, not " \
            "86:skew:FAKE_CPU_SKEW=1:2 CPUs not at 16387 KiB: cpu0 16390 KiB"; do
  IFS=: read -r attempt name envset words <<<"$case"
  if env "$envset" setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt "$attempt" --modes traced \
      --runs current:100:idle:full </dev/null >"$ROOT/stage-$name.log" 2>&1; then
    fail "a session with $name went on" "$ROOT/stage-$name.log"
  fi
  b=$(base current-100hz-idle-bounded-traced "$attempt")
  grep -q "$words" "$b.run.txt" && grep -q '^verdict: not run (staging)' "$b.run.txt" ||
    fail "the $name stop" "$b.run.txt"
  [ ! -e "$b" ] && [ -z "$(stage_left)" ] || fail "the $name stop ran the probe or left its stage" "$b.run.txt"
done
grep -q 'do not read back as expected (per array and per CPU' "$ROOT/stage-skew.log" ||
  fail "the per-CPU skew's stop message" "$ROOT/stage-skew.log"
# The TSC instance with another subbuffer geometry than the main array's:
# stopped before the probe.
if [ -n "$SAMPLER" ]; then
  if FAKE_INSTANCE_PAYLOAD=8176 TRACE_SESSION_RTPRIO_LIMIT=99 setsid tools/trace_session.sh "${PROFILE[@]}" --yes \
      --attempt 87 --duration 2 --modes timer --runs current:100:rt-idle:fast-motion </dev/null >"$ROOT/stage-geo.log" 2>&1; then
    fail "a session whose arrays disagree in geometry went on" "$ROOT/stage-geo.log"
  fi
  b=$(base current-100hz-rt-idle-bounded-fast-motion-timer 87)
  grep -q '^kernel buffers, array main: .*: read back exactly' "$b.run.txt" &&
    grep -q '^kernel buffers, array rtctrl_tsc: .*NOT AS EXPECTED: its geometry (subbuffer 8192 bytes, payload 8176 at offset 16 .*) is not the budget.s (subbuffer 4096 bytes, payload 4080)' "$b.run.txt" &&
    grep -q '^verdict: not run (staging)' "$b.run.txt" && [ -z "$(stage_left)" ] ||
    fail "the instance's geometry stop" "$b.run.txt"
fi
# A page header without its data field: the session refuses before anything
# is sent, and no run starts.
printf '\tfield: u64 timestamp;\toffset:0;\tsize:8;\tsigned:0;\n' >"$ROOT/tracefs/events/header_page"
status=0
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 88 --modes traced --runs current:100:idle:full \
  </dev/null >"$ROOT/stage-header.log" 2>&1 || status=$?
cp "$ROOT/header_page.good" "$ROOT/tracefs/events/header_page"
[ "$status" -eq 2 ] && grep -q 'the page header has 0 data fields, not one' "$ROOT/stage-header.log" &&
  grep -q "the trace buffers' geometry was not read or is not supported: nothing was sent" "$ROOT/stage-header.log" &&
  [ ! -e "$(base current-100hz-idle-bounded-traced 88).run.txt" ] && [ -z "$(stage_left)" ] ||
  fail "the malformed page header's refusal (status $status)" "$ROOT/stage-header.log"

# A publication that fails keeps the stage and fails the session (status
# 3), the capture judged from the kept stage; the retry publishes and
# releases it. Interrupted, the session keeps 130.
retry_stage() {
  local st
  st="$ROOT/stage/$(stage_left)"
  [ -d "$st" ] || fail "no kept stage to retry" "$ROOT/mounts"
  TRACE_STAGE_SUDO=none python3 tools/trace_stage.py publish --stage "$st" --outdir "$OUT" --settle-s 1 \
    --meminfo "$ROOT/meminfo" >"$ROOT/retry.log" 2>&1 || fail "the retried publication failed" "$ROOT/retry.log"
  TRACE_STAGE_SUDO=none python3 tools/trace_stage.py release --stage "$st" --outdir "$OUT" \
    >>"$ROOT/retry.log" 2>&1 || fail "the retried release failed" "$ROOT/retry.log"
  [ -z "$(stage_left)" ] && [ ! -s "$ROOT/mounts" ] || fail "the retry left the stage" "$ROOT/retry.log"
}
status=0
TRACE_STAGE_FAIL=copy setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 90 --modes traced \
  --runs current:100:idle:full </dev/null >"$ROOT/stage-copy.log" 2>&1 || status=$?
b=$(base current-100hz-idle-bounded-traced 90)
[ "$status" -eq 3 ] && grep -q 'PRESERVATION FAILED' "$ROOT/stage-copy.log" &&
  grep -q "retry: python3 tools/trace_stage.py publish --stage $ROOT/stage/" "$ROOT/stage-copy.log" &&
  grep -q '^preservation: NOT published: the staged evidence is kept at ' "$b.run.txt" &&
  grep -Eq '^verdict: (pass|fail)' "$b.run.txt" && [ ! -e "$b" ] && [ -n "$(stage_left)" ] ||
  fail "the failed publication (status $status)" "$ROOT/stage-copy.log"
retry_stage
(cd "$OUT" && sha256sum --quiet -c "$(basename "$b").staging.sha256") && [ -s "$b.trace.dat" ] ||
  fail "the retry's ledger" "$ROOT/retry.log"
TRACE_STAGE_FAIL=copy python3 -c '
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL)
os.setsid()
os.execvp(sys.argv[1], sys.argv[1:])' tools/trace_session.sh "${COMMON[@]}" --yes --attempt 91 --duration 4 \
  --modes traced --runs current:100:idle:full </dev/null >"$ROOT/stage-int.log" 2>&1 &
pid=$!
b=$(base current-100hz-idle-bounded-traced 91)
for _ in $(seq 1 300); do grep -q '^clock pair before' "$b.run.txt" 2>/dev/null && break; sleep 0.1; done
sleep 1
kill -TERM "$pid"
status=0
wait "$pid" || status=$?
[ "$status" -eq 130 ] && grep -q 'PRESERVATION FAILED' "$ROOT/stage-int.log" && [ -n "$(stage_left)" ] ||
  fail "the interrupted, failed publication (status $status)" "$ROOT/stage-int.log"
retry_stage

# A release whose unmount fails: published and verified, the stage kept,
# status 3; the next session refuses while it remains and never touches it;
# the retried release then removes it.
: >"$ROOT/umount-fail"
status=0
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 92 --modes traced \
  --runs current:100:idle:full </dev/null >"$ROOT/stage-umount.log" 2>&1 || status=$?
b=$(base current-100hz-idle-bounded-traced 92)
[ "$status" -eq 3 ] && grep -q '^preservation: published and verified; the stage was NOT released' "$b.run.txt" &&
  grep -q 'CLEANUP FAILED: the stage .* was not released' "$ROOT/stage-umount.log" && [ -n "$(stage_left)" ] ||
  fail "the failed unmount (status $status)" "$ROOT/stage-umount.log"
kept="$ROOT/stage/$(stage_left)"
status=0
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 93 --modes traced \
  --runs current:100:idle:full </dev/null >"$ROOT/stage-left.log" 2>&1 || status=$?
[ "$status" -eq 2 ] && grep -q "a stage remains under $ROOT/stage" "$ROOT/stage-left.log" &&
  grep -q "$kept: run rate-trace-current-100hz-idle-bounded-traced-$DATE-92.csv" "$ROOT/stage-left.log" &&
  [ -d "$kept" ] && [ -e "$kept/.rtctrl-stage" ] || fail "the leftover stage's refusal (status $status)" "$ROOT/stage-left.log"
[ ! -e "$(base current-100hz-idle-bounded-traced 93).run.txt" ] || fail "a session ran beside a leftover stage" "$ROOT/stage-left.log"
rm -f "$ROOT/umount-fail"
TRACE_STAGE_SUDO=none python3 tools/trace_stage.py release --stage "$kept" --outdir "$OUT" >"$ROOT/retry.log" 2>&1 &&
  [ -z "$(stage_left)" ] && [ ! -s "$ROOT/mounts" ] || fail "the retried release" "$ROOT/retry.log"

# A release that unmounts and then fails to remove the mount point: status
# 3; the bare mount point (its marker gone with the tmpfs) refuses the next
# session, listed as one; the retried release removes it without a marker.
: >"$ROOT/rmdir-fail"
status=0
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 94 --modes traced \
  --runs current:100:idle:full </dev/null >"$ROOT/stage-rmdir.log" 2>&1 || status=$?
rm -f "$ROOT/rmdir-fail"
b=$(base current-100hz-idle-bounded-traced 94)
kept="$ROOT/stage/$(stage_left)"
[ "$status" -eq 3 ] && grep -q '^preservation: published and verified; the stage was NOT released' "$b.run.txt" &&
  grep -q 'unmounted but was not removed' "$b.run.txt" && [ -d "$kept" ] && [ ! -s "$ROOT/mounts" ] &&
  [ -z "$(ls -A "$kept")" ] || fail "the failed removal after the unmount (status $status)" "$ROOT/stage-rmdir.log"
status=0
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 95 --modes traced \
  --runs current:100:idle:full </dev/null >"$ROOT/stage-bare.log" 2>&1 || status=$?
[ "$status" -eq 2 ] && grep -q "$kept: not mounted, a bare mount point" "$ROOT/stage-bare.log" && [ -d "$kept" ] ||
  fail "the bare mount point's refusal (status $status)" "$ROOT/stage-bare.log"
TRACE_STAGE_SUDO=none python3 tools/trace_stage.py release --stage "$kept" --outdir "$OUT" >"$ROOT/retry.log" 2>&1 &&
  grep -q 'was no longer mounted' "$ROOT/retry.log" && [ -z "$(stage_left)" ] ||
  fail "the retried release of the bare mount point" "$ROOT/retry.log"

echo "trace session script OK"
