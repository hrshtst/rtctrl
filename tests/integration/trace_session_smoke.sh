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
# instance's removal on every exit (status 3 when it fails), and the
# exerciser of the rehearsal. Functional only: a
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
  exit 0
fi
[ "${FAKE_TRACE_FAIL:-0}" = 1 ] && { echo "stand-in trace-cmd: failing as asked"; exit 1; }
out=; clock=local
while [ $# -gt 0 ]; do
  case "$1" in -o) out=$2; shift ;; -C) clock=$2; shift ;; esac
  shift
done
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
# ($FAKE_PROBE_TEMPLATE, from the pairs below) for --duration seconds.
cat >"$FAKE/probe" <<'EOF'
#!/usr/bin/env bash
log=; dur=1
while [ $# -gt 0 ]; do
  case "$1" in --log) log=$2; shift ;; --duration) dur=$2; shift ;; esac
  shift
done
cp "$FAKE_PROBE_TEMPLATE" "$log"
cp "$FAKE_PROBE_TEMPLATE.summary.toml" "$log.summary.toml"
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
# it creates it and exits 1, a status that does not say).
cat >"$FAKE/tsc_instance" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.tsc_instance"
mark="$FAKE_LOG.tsc_instance_exists"
case "$1" in
  start) [ -e "$mark" ] && exit 2
         case "${FAKE_TSC_START:-}" in
           race) : >"$mark"; echo "tsc_instance: it exists (another session's)" >&2; exit 2 ;;
           partial) : >"$mark"; echo "tsc_instance: cannot set the clock" >&2; exit 4 ;;
           unknown) : >"$mark"; echo "sudo: a stand-in failure" >&2; exit 1 ;;
         esac
         : >"$mark"
         echo "tsc instance $3: clock [x86-tsc], cpumask 1, buffer $5 kB per CPU, event irq_vectors:local_timer_entry" ;;
  stop) echo "# entries-in-buffer/entries-written: 1/1   #P:2"
        echo "          <idle>-0       [000] d.h1. 123456789: local_timer_entry: vector=236" ;;
  stats) [ "${FAKE_TSC_STATS:-}" = fail ] && { echo "stand-in: cannot read the stats" >&2; exit 1; }
         printf 'CPU: %s\nentries: 1\noverrun: %s\ncommit overrun: 0\ndropped events: 0\n' "$4" "${FAKE_TSC_OVERRUN:-0}" ;;
  remove) [ "${FAKE_TSC_REMOVE_FAIL:-0}" = 1 ] && { echo "stand-in: cannot remove" >&2; exit 1; }
          rm -f "$mark" ;;
  exists) [ -e "$mark" ] ;;
esac
EOF
chmod +x "$FAKE"/*
echo "[local] global counter uptime perf mono mono_raw boot tai x86-tsc" >"$ROOT/tracefs/trace_clock"
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
export TRACE_SESSION_BATCH_LIST=rtctrl-nobatch QUIET_HOST_STATE="$ROOT/qh.state"
export TRACE_SESSION_CPU_ROOT="$ROOT/cpu"
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
grep -q '^record -C mono -b 16384 .*-e sched:sched_switch .*-e xhci-hcd:xhci_urb_giveback -o ' "$ROOT/fake.trace-cmd" ||
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
  sleep 1
  if [ "$target" = group ]; then kill -INT -- "-$pid"; else kill -TERM "$pid"; fi
  status=0
  wait "$pid" || status=$?
  [ "$status" -eq 130 ] || fail "the interrupted session exited $status" "$ROOT/int-$sig.log"
  grep -q 'INTERRUPTED (counts as attempted)' "$ROOT/int-$sig.log" ||
    fail "the interrupt was not reported" "$ROOT/int-$sig.log"
  [ -s "$b.trace.dat" ] && [ -s "$b.usbmon.pcap" ] ||
    fail "the interrupted run's recorders did not write their files" "$ROOT/int-$sig.log"
  for f in "$b.trace.dat" "$b.usbmon.pcap"; do
    grep -qxF "chown $(id -u):$(id -g) $f" "$ROOT/fake.sudo" ||
      fail "$f was not given back after SIG$sig" "$ROOT/fake.sudo"
  done
  [ ! -e "$(base cbp-100hz-idle-bounded-traced "$attempt").run.txt" ] ||
    fail "a run started after SIG$sig" "$ROOT/int-$sig.log"
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
grep -Eq "^perf: $FAKE/perf record -k mono -C 0 -F 999 --call-graph dwarf\\\\?,4096 -o $b.perf.data" "$b.run.txt" ||
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
    grep -Eq '^smi anchor monotonic_ns: [0-9]+ [0-9]+ ' "$b.run.txt" && grep -q "^session pid: " "$b.run.txt" ||
    fail "the timer record" "$b.run.txt"
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

echo "trace session script OK"
