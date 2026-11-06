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
# not hold a step's value, and the restoration on every exit. Functional only: a
# pty answers instantly and the tracers record nothing real, so no verdict
# is asserted. Real-time conditions need a real-time priority limit, so
# they stay out of this test (the plan is checked with --dry-run).
set -eu

EMU="$1"
PROBE="$2"
ROOT="$3"
INSPECT="$4"
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
[ "${FAKE_TRACE_FAIL:-0}" = 1 ] && { echo "stand-in trace-cmd: failing as asked"; exit 1; }
out=; clock=local
while [ $# -gt 0 ]; do
  case "$1" in -o) out=$2; shift ;; -C) clock=$2; shift ;; esac
  shift
done
[ -n "${FAKE_TRACE_CLOCK:-}" ] && clock=$FAKE_TRACE_CLOCK
case "$clock" in
  mono) echo "local global counter uptime perf [mono] mono_raw boot" >"$TRACEFS/trace_clock" ;;
  *) echo "[local] global counter uptime perf mono mono_raw boot" >"$TRACEFS/trace_clock" ;;
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
# FAKE_LATENCY_IGNORE.
cat >"$FAKE/latency" <<'EOF'
#!/usr/bin/env bash
state="$FAKE_LOG.latency-state"
echo "$*" >>"$FAKE_LOG.latency"
read -r s d l <"$state"
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
# In sudo's place (TRACE_SESSION_SUDO): logs each privileged command and
# runs it as the invoking user; its timestamp checks succeed.
cat >"$FAKE/sudo" <<'EOF'
#!/usr/bin/env bash
case "$1" in -v|-n) exit 0 ;; esac
echo "$*" >>"$FAKE_LOG.sudo"
exec "$@"
EOF
chmod +x "$FAKE"/*
echo "[local] global counter uptime perf mono mono_raw boot" >"$ROOT/tracefs/trace_clock"
printf '1\n' >"$ROOT/usbdev/busnum"
printf '9\n' >"$ROOT/usbdev/devnum"
printf '0403\n' >"$ROOT/usbdev/idVendor"
printf '6014\n' >"$ROOT/usbdev/idProduct"
printf '480\n' >"$ROOT/usbdev/speed"

export TRACE_SESSION_SUDO=none TRACEFS="$ROOT/tracefs" FAKE_LOG="$ROOT/fake"
export TRACE_CMD="$FAKE/trace-cmd" TCPDUMP="$FAKE/tcpdump"
export BPFTRACE="$FAKE/bpftrace" MODPROBE="$FAKE/modprobe" STRESS_NG="$FAKE/stress-ng"
export LATENCY_TOOL="$FAKE/latency"
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
grep -q '^latency timer restored to 4 ms' "$(base current-100hz-idle-bounded-lt1-s3-traced 13).run.txt" ||
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

# An emulator rehearsal traces without usbmon.
setsid tools/trace_session.sh "${COMMON[@]}" --yes --attempt 4 --no-usbmon \
  --modes traced --runs current:100:idle:fast </dev/null >"$ROOT/nousb.log" 2>&1 ||
  fail "the session without usbmon failed" "$ROOT/nousb.log"
b=$(base current-100hz-idle-bounded-fast-traced 4)
[ -s "$b.trace.dat" ] && [ ! -e "$b.usbmon.pcap" ] || fail "--no-usbmon captured usbmon" "$b.run.txt"
grep -q '^usbmon: not used (--no-usbmon)' "$b.run.txt"

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
