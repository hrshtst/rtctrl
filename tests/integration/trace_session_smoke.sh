#!/usr/bin/env bash
# tools/trace_session.sh over dxl_emu, with stand-in tracers (trace-cmd,
# tcpdump, bpftrace, modprobe) and no sudo: the refusals, the
# untraced/traced pairs and their records, the mono trace clock, the
# usbmon device filter, the clock pairs, resuming without a retry, and the
# stops when the fair server is not read, a tracer does not start or the
# trace clock is not mono. Functional only: a pty answers instantly and the
# tracers record nothing real, so no verdict is asserted. The loaded and
# real-time conditions need stress-ng and a real-time priority limit, so
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
# then write their file, as the real ones do.
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
echo "Hit Ctrl^C to stop recording"
while [ "$stop" -eq 0 ]; do sleep 0.05; done
echo "stand-in trace data" >"$out"
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
while [ "$stop" -eq 0 ]; do sleep 0.05; done
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
cat >"$FAKE/modprobe" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$FAKE_LOG.modprobe"
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
export BPFTRACE="$FAKE/bpftrace" MODPROBE="$FAKE/modprobe"

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
grep -q 'load: taskset -c 1[0-9,]* stress-ng --cpu [0-9]* --hdd 1' "$ROOT/plan.log"
grep -q -- "rate-trace-current-1000hz-rt-loaded-excl-bounded-fast-motion-traced-$DATE-1.csv --rt-priority 80 --rt-cpu 0" "$ROOT/plan.log"
grep -q "rate-trace-cbp-1000hz-rt-idle-bounded-fast-untraced-$DATE-1.csv" "$ROOT/plan.log"

echo "trace session script OK"
