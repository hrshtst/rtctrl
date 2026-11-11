#!/usr/bin/env bash
# Smoke test of tools/option_a.sh and tools/root_keeper.sh: the real
# supervisor, keeper (its unprivileged test mode) and tools/wq_mask.sh, with
# stand-ins for the kernel's mask files and workers, systemctl, the session,
# the gate and tools/quiet_host.sh.
#
# A normal experiment runs, in order, the quieting, the mask's clear, the
# untraced session (without the controls), the check and gate, the controls
# with the timer session, the check and gate, then restores and verifies
# everything and ends the keeper (exit 0); the real gate passes on its
# records, open and closed. Refusals change nothing (fresh directories, a
# failing dry run, an open record, a low real-time limit). A failed session,
# a failed gate, a hangup or an interrupt reaching the process group during
# a session, a TERM to the supervisor alone during a session, and a TERM
# latched just before a session starts each stop it with no further step
# (the latched one before that session) and every restoration verified
# (exit 1). A record that cannot be written before a session starts stops
# it before that session, and a final record that cannot be written makes
# the outcome unresolved, never exit 0 (exit 3). A restoration that fails is
# unresolved (exit 3, its record open). A process still running when the
# experiment ends (a lingering one, or a fork-exit chain, whose generations
# a /proc scan can miss, in the group or in a session of its own) leaves
# everything unrestored (exit 3): the supervisor, the child subreaper of all
# it starts, finds it among its children and releases the keeper, which ends
# without restoring. A supervisor killed with SIGKILL leaves the keeper to
# restore the mask once the kernel finds no process left in the group, a
# fork-exit chain in it included. The keeper: its list; its modes under
# umask 077 (the files the client reads stay 0644); a directory of its own,
# created exclusively; links planted in its replies (temporary or final
# names, or in place of the directory) redirect nothing; an owner that
# cannot be read is not taken as ended; a group member that a /proc scan
# cannot read is still found (the keeper ends without restoring once its
# wait is over), and one seen to end lets it restore. Its group test, on the
# re-review's race (a member forking and exiting just as a scan reaches it)
# and on a free-running fork-exit chain, reports "present", and "ended" once
# the chain has stopped.
#
# The scratch directory comes through tests/integration/scratch_dir.sh: never
# empty, never a system or home directory; every removal is under ${ROOT:?}.
#
# usage: option_a_smoke.sh <scratch dir>
set -u
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "option_a_smoke" "${1:-}" || exit 2  # a new child of an approved base: the only place this test removes
ROOT=$RTCTRL_SCRATCH
fail() { echo "FAILED: $1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }
ORIG=ffffffff
TARGET=fffffff7
OPTION_A_KERNEL=$(uname -r)
export ROOT OPTION_A_OUT=$ROOT/out OPTION_A_PORT=$ROOT/pty OPTION_A_KERNEL OPTION_A_SUDO=none OPTION_A_YES=1
export OPTION_A_RTLIMIT=95 OPTION_A_TRACE_SESSION=$ROOT/session OPTION_A_GATE=$ROOT/gate OPTION_A_QUIET=$ROOT/quiet
export ROOT_KEEPER_TEST=1 KEEPER_SYSFS=$ROOT/sys KEEPER_WRITE=$ROOT/kernel KEEPER_SYSTEMCTL=$ROOT/systemctl
export WQ_MASK_SYSFS=$ROOT/sys WQ_MASK_PROC=$ROOT/proc WQ_MASK_RECORD=$ROOT/state/wq.rec WQ_MASK_WAIT=1
export QUIET_HOST_STATE=$ROOT/state/quiet.state
export KEEPER_RUNDIR=$ROOT/run OPTION_A_GROUP_WAIT=2
REAL_DATE=$(command -v date)
REAL_PY=$(command -v python3)
E=$ROOT/out/clock-validation-option-a

cat >"$ROOT/cpus" <<'EOF'
import sys
v = 0
for g in sys.argv[1].split(","):
    v = (v << 32) | int(g, 16)
on = [i for i in range(v.bit_length()) if v >> i & 1]
runs, s = [], None
for i, c in enumerate(on):
    if s is None:
        s = c
    if i + 1 == len(on) or on[i + 1] != c + 1:
        runs.append(f"{s}-{c}" if c > s else f"{s}")
        s = None
print(",".join(runs))
EOF
# The kernel's write (KEEPER_WRITE): both masks and the workers' affinities, unless $ROOT/kernel.mode refuses the original.
cat >"$ROOT/kernel" <<'EOF'
#!/usr/bin/env bash
[ "$(cat "$ROOT/kernel.mode" 2>/dev/null)" = refuse-original ] && [ "$1" = ffffffff ] && exit 1
printf '%s\n' "$1" >"$ROOT/sys/cpumask"; printf '%s\n' "$1" >"$ROOT/sys/cpumask_requested"
list=$(python3 "$ROOT/cpus" "$1")
for d in "$ROOT"/proc/*/; do printf 'Name:\tx\nCpus_allowed_list:\t%s\n' "$list" >"$d/status"; done
EOF
cat >"$ROOT/systemctl" <<'EOF'
#!/usr/bin/env bash
echo "$1 $2" >>"$ROOT/uu.log"
EOF
# tools/quiet_host.sh's stand-in: its record, and the system service's stop and start through its sudo (the keeper).
cat >"$ROOT/quiet" <<'EOF'
#!/usr/bin/env bash
case "$1" in
  stop) echo "started=x" >"$QUIET_HOST_STATE"; "$QUIET_HOST_SUDO" systemctl stop unattended-upgrades.service ;;
  restore) [ -e "$QUIET_HOST_STATE" ] || exit 0
           "$QUIET_HOST_SUDO" systemctl start unattended-upgrades.service && { . "$RTCTRL_SCRATCH_LIB"; scratch_rm "$QUIET_HOST_STATE"; } ;;
esac
EOF
# The session's stand-in: the dry run, or a run that writes its probe log (and the controls' record in the timer mode),
# notes the mask it ran under, lasts $ROOT/session.secs (0.6 s) and exits with $ROOT/session.exit.<attempt> (0);
# INT, TERM or HUP run its "cleanup" (a marker) and exit 130.
cat >"$ROOT/session" <<'EOF'
#!/usr/bin/env bash
mode=; attempt=; outdir=; dry=0; ctl=0
args="$*"
while [ $# -gt 0 ]; do
  case "$1" in --modes) mode=$2; shift ;; --attempt) attempt=$2; shift ;; --outdir) outdir=$2; shift ;;
               --dry-run) dry=1 ;; --fs-controls) ctl=1; shift ;; esac
  shift
done
if [ "$dry" -eq 1 ]; then echo "dry $args" >>"$ROOT/calls"; exit "$(cat "$ROOT/dry.exit" 2>/dev/null || echo 0)"; fi
echo "session $mode $attempt controls=$ctl" >>"$ROOT/calls"
trap 'echo cleaned >"$ROOT/cleaned-$attempt"; exit 130' INT TERM HUP
cat "$ROOT/sys/cpumask" >"$ROOT/mask-during-$attempt"
: >"$outdir/rate-trace-current-1000hz-rt-loaded-excl-bounded-fast-motion-obs-open-$mode-20261112-$attempt.csv"
: >"$outdir/rate-trace-current-1000hz-rt-loaded-excl-bounded-fast-motion-obs-open-$mode-20261112-$attempt.csv.run.txt"
[ "$ctl" -eq 1 ] && : >"$outdir/fs-controls-20261112-$attempt.run.txt"
if [ -e "$ROOT/real-gate" ]; then
  uv run --project tools python -c 'import pathlib, sys
sys.path.insert(0, "tools")
import test_validation_gate as t
d = pathlib.Path(sys.argv[1])
t.write_run(d, sys.argv[2], mask=sys.argv[3], mode=sys.argv[4])
if sys.argv[5] == "1":
    (d / f"fs-controls-20261112-{sys.argv[6]}.run.txt").write_text("verdict: controls pass\n")' \
    "$outdir" "rate-trace-current-1000hz-rt-loaded-excl-bounded-fast-motion-obs-open-$mode-20261112-$attempt" \
    "$(cat "$ROOT/sys/cpumask")" "$mode" "$ctl" "$attempt" || exit 9
fi
[ -e "$ROOT/session.linger.$attempt" ] && { sleep 30 & echo $! >"$ROOT/linger.pid"; }
case "$(cat "$ROOT/session.chain.$attempt" 2>/dev/null)" in
  group) "$ROOT/chain" & ;;
  setsid) setsid "$ROOT/chain" & ;;
esac
: >"$ROOT/running-$attempt"
end=$(( $(date +%s%N) + $(python3 -c "print(int(float('$(cat "$ROOT/session.secs" 2>/dev/null || echo 0.6)') * 1e9))") ))
while [ "$(date +%s%N)" -lt "$end" ]; do sleep 0.05; done
exit "$(cat "$ROOT/session.exit.$attempt" 2>/dev/null || echo 0)"
EOF
cat >"$ROOT/gate" <<'EOF'
#!/usr/bin/env bash
s=; for a in "$@"; do [ "${prev:-}" = --session ] && s=$a; prev=$a; done
echo "gate $*" >>"$ROOT/calls"
exit "$(cat "$ROOT/gate.exit.$s" 2>/dev/null || echo 0)"
EOF
# A fork-exit chain: each generation starts the next and ends at once, until $ROOT/chain.stop exists; $ROOT/chain.pid
# holds the latest generation, $ROOT/chain.first the first (started with setsid: its own session and group).
cat >"$ROOT/chain" <<'EOF'
#!/usr/bin/env bash
[ -e "$ROOT/chain.first" ] || echo $$ >"$ROOT/chain.first"
echo $$ >"$ROOT/chain.pid"
[ -e "$ROOT/chain.stop" ] && exit 0
sleep 0.02
"$0" &
exit 0
EOF
chmod +x "$ROOT/kernel" "$ROOT/systemctl" "$ROOT/quiet" "$ROOT/session" "$ROOT/gate" "$ROOT/chain"

chain_stop() {  # stops a running fork-exit chain: status 0 once its latest generation has ended
  local i p q
  : >"$ROOT/chain.stop"
  for ((i = 0; i < 100; i++)); do
    p=$(cat "$ROOT/chain.pid" 2>/dev/null); sleep 0.05; q=$(cat "$ROOT/chain.pid" 2>/dev/null)
    [ "$p" = "$q" ] && { [ -z "$p" ] || ! kill -0 "$p" 2>/dev/null; } && return 0
  done
  return 1
}
reset() {  # the host as it starts
  [ -e "$ROOT/linger.pid" ] && kill "$(cat "$ROOT/linger.pid")" 2>/dev/null
  [ -e "$ROOT/chain.pid" ] && chain_stop
  scratch_rm "$ROOT/out" "$ROOT/state" "$ROOT/proc" "$ROOT/sys" "$ROOT/run" "$ROOT/bin" "$ROOT"/calls \
         "$ROOT"/cleaned-* "$ROOT"/mask-during-* "$ROOT"/running-* "$ROOT"/session.* "$ROOT"/gate.exit.* \
         "$ROOT/dry.exit" "$ROOT/kernel.mode" "$ROOT/uu.log" "$ROOT/sup.out" "$ROOT"/trigger.* "$ROOT/linger.pid" \
         "$ROOT/hook.bash" "$ROOT"/hide.* "$ROOT"/chain.*
  mkdir -p "$ROOT/sys" "$ROOT/state" "$ROOT/proc" "$ROOT/run"
  printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask"; printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask_requested"
  printf '00000000\n' >"$ROOT/sys/cpumask_isolated"
  local p
  for p in 101:kworker/u128:3 102:kworker/R-writeback 103:bash; do
    mkdir -p "$ROOT/proc/${p%%:*}"; printf '%s\n' "${p#*:}" >"$ROOT/proc/${p%%:*}/comm"
    printf 'Name:\tx\nCpus_allowed_list:\t0-31\n' >"$ROOT/proc/${p%%:*}/status"
  done
  : >"$ROOT/pty"
}
masks() { echo "$(cat "$ROOT/sys/cpumask") $(cat "$ROOT/sys/cpumask_requested")"; }
keeper_dir() { sed -n 's/^started=.* keeper_dir=//p' "$E/experiment.record" "$E/experiment.record.saved" 2>/dev/null | head -n 1; }
keeper_gone() {  # waits up to 5 s for the experiment's keeper to end
  local pid i
  pid=$(sed -n 's/^pid=//p' "$(keeper_dir)/keeper.ready" 2>/dev/null)
  [ -n "$pid" ] || return 1
  for ((i = 0; i < 50; i++)); do [ -e "/proc/$pid" ] || return 0; sleep 0.1; done
  return 1
}
restored() {  # every restoration verified: masks, the closed mask record, the quiet record, the service, the keeper
  [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$WQ_MASK_RECORD" ] && ls "$WQ_MASK_RECORD".restored-* >/dev/null 2>&1 &&
    [ ! -e "$QUIET_HOST_STATE" ] && [ "$(tail -n 1 "$ROOT/uu.log")" = "start unattended-upgrades.service" ] && keeper_gone
}
start() {  # the supervisor in its own session and process group, INT and QUIT at their defaults (a non-interactive
           # shell starts background commands with both ignored, which no trap could undo); its pid in sup
  python3 -c 'import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_DFL); signal.signal(signal.SIGQUIT, signal.SIG_DFL)
os.setsid(); os.execvp(sys.argv[1], sys.argv[1:])' tools/option_a.sh >"$ROOT/sup.out" 2>&1 &
  sup=$!
}
wait_file() { local i; for ((i = 0; i < 100; i++)); do [ -e "$1" ] && return 0; sleep 0.1; done; return 1; }

# A normal experiment.
reset
start; wait "$sup"; st=$?
[ "$st" -eq 0 ] || fail "the normal experiment exited $st" "$ROOT/sup.out"
grep -v '^dry ' "$ROOT/calls" >"$ROOT/calls.run"
[ "$(sed -n 1p "$ROOT/calls.run")" = "session untraced 14 controls=0" ] && grep -q -- '--session A --record-state open' <(sed -n 2p "$ROOT/calls.run") &&
  grep -q -- '--mode untraced' <(sed -n 2p "$ROOT/calls.run") && [ "$(sed -n 3p "$ROOT/calls.run")" = "session timer 15 controls=1" ] &&
  grep -q -- '--mode timer --controls .*fs-controls-20261112-15.run.txt' <(sed -n 4p "$ROOT/calls.run") && [ "$(wc -l <"$ROOT/calls.run")" -eq 4 ] ||
  fail "the steps' order" "$ROOT/calls"
[ "$(grep -c '^dry ' "$ROOT/calls")" -eq 2 ] && grep -q '^dry .*--modes untraced' "$ROOT/calls" && grep -q '^dry .*--fs-controls 5 --modes timer' "$ROOT/calls" ||
  fail "the dry runs" "$ROOT/calls"
[ "$(cat "$ROOT/mask-during-14")" = "$TARGET" ] && [ "$(cat "$ROOT/mask-during-15")" = "$TARGET" ] || fail "the sessions did not run under the target"
restored || fail "the normal experiment's restorations" "$ROOT/sup.out"
keys=$(sed 's/=.*//' "$E/experiment.record" | tr '\n' ' ')
want="started directories quiet_owed quiet_stop_start quiet_stop_end mask_owed mask_clear_start mask_clear_end session_A_start session_A_end check_A_start check_A_end gate_A_start gate_A_end session_B_start session_B_end check_B_start check_B_end gate_B_start gate_B_end mask_restore_start mask_restore_end quiet_restore_start quiet_restore_end keeper_finish_start keeper_finish_end finished "
[ "$keys" = "$want" ] && grep -q '^finished=.* outcome=ACQUIRED reason=none$' "$E/experiment.record" || fail "the experiment record: $keys" "$E/experiment.record"
grep -q 'request .*: mask fffffff7: status 0' "$E/keeper.log" && grep -q 'finish: the original is in effect' "$E/keeper.log" ||
  fail "the keeper's log (the supervisor's copy)" "$E/keeper.log"
[ "$(stat -c %a "$(keeper_dir)")" = 755 ] && [ "$(stat -c %a "$(keeper_dir)/replies")" = 755 ] && [ ! -e "$(keeper_dir)/requests" ] ||
  fail "the keeper's directory: its modes, or the FIFO left"

# The same with the real gate, on realistic run files: the supervisor's and the mask tool's records are what it reads.
# After the experiment, the gate on both runs passes against the closed record too.
reset; : >"$ROOT/real-gate"
(unset OPTION_A_GATE; start; wait "$sup"; echo $? >"$ROOT/real.st")
st=$(cat "$ROOT/real.st"); scratch_rm "$ROOT/real-gate"
[ "$st" -eq 0 ] && restored && grep -q 'gate_A_end=.* status=0' "$E/experiment.record" && grep -q 'gate_B_end=.* status=0' "$E/experiment.record" ||
  fail "the experiment with the real gate ($st)" "$E/supervisor.log"
closed=$(ls "$WQ_MASK_RECORD".restored-* | head -n 1)
for s in A:untraced:14 B:timer:15; do
  IFS=: read -r ses m a <<<"$s"
  csv=$(ls "$ROOT/out/clock-validation-$a"/*-"$m"-*-"$a".csv)
  extra=(); [ "$m" = timer ] && extra=(--controls "$ROOT/out/clock-validation-$a/fs-controls-20261112-$a.run.txt")
  uv run --project tools tools/validation_gate.py "$csv" --mode "$m" "${extra[@]}" --wq-record "$closed" --experiment "$E/experiment.record" \
    --session "$ses" --record-state closed >"$ROOT/closed-$ses.txt" 2>&1 || fail "the closed-record gate on session $ses" "$ROOT/closed-$ses.txt"
done

# Refusals: nothing changed.
reset; mkdir -p "$ROOT/out/clock-validation-14"
start; wait "$sup"; st=$?
[ "$st" -eq 2 ] && [ ! -e "$ROOT/calls" ] && [ ! -e "$E" ] && [ "$(masks)" = "$ORIG $ORIG" ] || fail "an existing directory did not refuse ($st)" "$ROOT/sup.out"
reset; echo 2 >"$ROOT/dry.exit"
start; wait "$sup"; st=$?
[ "$st" -eq 2 ] && [ ! -e "$E" ] && [ ! -e "$ROOT/out/clock-validation-14" ] && [ ! -e "$QUIET_HOST_STATE" ] && [ ! -e "$WQ_MASK_RECORD" ] ||
  fail "a failing dry run did not refuse, or left directories ($st)" "$ROOT/sup.out"
reset; echo "started=x" >"$WQ_MASK_RECORD"
start; wait "$sup"; st=$?
[ "$st" -eq 2 ] && [ ! -e "$E" ] || fail "an open mask record did not refuse ($st)" "$ROOT/sup.out"
reset; OPTION_A_RTLIMIT=0 tools/option_a.sh >"$ROOT/sup.out" 2>&1; st=$?
[ "$st" -eq 2 ] && grep -q 'real-time priority limit is 0' "$ROOT/sup.out" || fail "a low real-time limit did not refuse ($st)" "$ROOT/sup.out"
reset; tools/option_a.sh --dry-run >"$ROOT/sup.out" 2>&1; st=$?
[ "$st" -eq 0 ] && [ ! -e "$E" ] && [ "$(grep -c '^dry ' "$ROOT/calls")" -eq 2 ] && [ "$(masks)" = "$ORIG $ORIG" ] || fail "the dry run ($st)" "$ROOT/sup.out"

# A failed session, a failed gate: no further step, everything restored.
reset; echo 1 >"$ROOT/session.exit.14"
start; wait "$sup"; st=$?
[ "$st" -eq 1 ] && ! grep -q '^session timer' "$ROOT/calls" && ! grep -q '^gate' "$ROOT/calls" && restored &&
  grep -q '^finished=.* outcome=STOPPED reason=the untraced session failed (status 1)$' "$E/experiment.record" || fail "a failed session ($st)" "$ROOT/sup.out"
reset; echo 1 >"$ROOT/gate.exit.A"
start; wait "$sup"; st=$?
[ "$st" -eq 1 ] && ! grep -q '^session timer' "$ROOT/calls" && restored &&
  grep -q 'outcome=STOPPED reason=the gate on the untraced run failed: no timer run' "$E/experiment.record" || fail "a failed gate ($st)" "$ROOT/sup.out"

# A hangup, and an interrupt, reaching the process group during a session: the session cleans up, then the supervisor
# restores everything.
for case in HUP:14 INT:15; do
  sig=${case%%:*}; at=${case#*:}
  reset; echo 3 >"$ROOT/session.secs"
  start
  wait_file "$ROOT/running-$at" || fail "session $at did not start" "$ROOT/sup.out"
  kill -"$sig" -- -"$sup"; wait "$sup"; st=$?
  [ "$st" -eq 1 ] && [ -e "$ROOT/cleaned-$at" ] && restored && grep -q "^signal=.* $sig$" "$E/experiment.record" &&
    grep -q "outcome=STOPPED reason=signal $sig" "$E/experiment.record" || fail "$sig during session $at ($st)" "$ROOT/sup.out"
  [ "$at" = 15 ] || ! grep -q '^session timer' "$ROOT/calls" || fail "a session after the hangup" "$ROOT/calls"
done
# A TERM to the supervisor alone: the running session finishes, then nothing else runs.
reset; echo 1.5 >"$ROOT/session.secs"
start
wait_file "$ROOT/running-14" || fail "session 14 did not start" "$ROOT/sup.out"
kill -TERM "$sup"; wait "$sup"; st=$?
[ "$st" -eq 1 ] && [ ! -e "$ROOT/cleaned-14" ] && ! grep -q '^gate' "$ROOT/calls" && ! grep -q '^session timer' "$ROOT/calls" && restored &&
  grep -q 'session_A_end=.* status=0' "$E/experiment.record" || fail "a TERM to the supervisor alone ($st)" "$ROOT/sup.out"

# A restoration that fails: unresolved, the record open.
reset; echo refuse-original >"$ROOT/kernel.mode"
start; wait "$sup"; st=$?
[ "$st" -eq 3 ] && [ -e "$WQ_MASK_RECORD" ] && [ "$(masks)" = "$TARGET $TARGET" ] &&
  grep -q '^finished=.* outcome=UNRESOLVED reason=none; unresolved: mask keeper$' "$E/experiment.record" || fail "a failed restoration ($st)" "$ROOT/sup.out"
keeper_gone || fail "the keeper stayed after a failed restoration"

# The supervisor killed (SIGKILL) during the untraced session: the keeper waits for the group to end, then restores the mask.
reset; echo 2 >"$ROOT/session.secs"
start
wait_file "$ROOT/running-14" || fail "session 14 did not start" "$ROOT/sup.out"
kill -KILL "$sup"; wait "$sup" 2>/dev/null
keeper_log=$(keeper_dir)/keeper.log
for ((i = 0; i < 100; i++)); do grep -q 'is in effect; ending' "$keeper_log" 2>/dev/null && break; sleep 0.1; done
grep -q 'ended without a finish' "$keeper_log" && grep -q 'is in effect; ending' "$keeper_log" && [ "$(masks)" = "$ORIG $ORIG" ] &&
  [ -e "$WQ_MASK_RECORD" ] && keeper_gone || fail "the keeper after a killed supervisor" "$keeper_log"
# The session (still in the group) had ended before the keeper wrote the original back.
[ "$(stat -c %Y "$ROOT/sys/cpumask")" -ge "$(stat -c %Y "$ROOT/mask-during-14")" ] || fail "the keeper restored before the session ended"
WQ_MASK_SUDO=none tools/wq_mask.sh restore >/dev/null 2>&1 || :
# The same with a fork-exit chain left in the group by the untraced session: the keeper's kernel test finds the group
# populated after the session has ended (where a /proc scan could miss a generation), and it restores only once the
# chain has stopped.
reset; echo group >"$ROOT/session.chain.14"
start
wait_file "$ROOT/running-14" && wait_file "$ROOT/chain.pid" || fail "session 14 or its chain did not start" "$ROOT/sup.out"
kill -KILL "$sup"; wait "$sup" 2>/dev/null
keeper_log=$(keeper_dir)/keeper.log
for ((i = 0; i < 50; i++)); do grep -q 'ended without a finish' "$keeper_log" 2>/dev/null && break; sleep 0.1; done
sleep 2
[ "$(masks)" = "$TARGET $TARGET" ] && grep -q 'ended without a finish' "$keeper_log" && ! grep -q 'is in effect; ending' "$keeper_log" ||
  fail "the keeper restored while a chain ran in the group" "$keeper_log"
chain_stop || fail "the chain (supervisor killed) did not stop"
for ((i = 0; i < 100; i++)); do grep -q 'is in effect; ending' "$keeper_log" && break; sleep 0.1; done
[ "$(masks)" = "$ORIG $ORIG" ] && keeper_gone || fail "the keeper after the chain stopped" "$keeper_log"
WQ_MASK_SUDO=none tools/wq_mask.sh restore >/dev/null 2>&1 || :

# A TERM latched just before the untraced session starts (a debug hook sends it as that step is about to run): that
# session never starts.
reset
printf '%s\n' 'if [[ $0 == *option_a.sh ]]; then trap '"'"'if [[ $BASH_COMMAND == "step session_A "* && -z ${HOOK_SENT:-} ]]; then HOOK_SENT=1; kill -TERM $$; fi'"'"' DEBUG; fi' >"$ROOT/hook.bash"
(export BASH_ENV=$ROOT/hook.bash; start; wait "$sup"; echo $? >"$ROOT/term.st")
st=$(cat "$ROOT/term.st")
[ "$st" -eq 1 ] && ! grep -q '^session ' "$ROOT/calls" && ! grep -q '^session_A_start=' "$E/experiment.record" && grep -q '^signal=.* TERM$' "$E/experiment.record" &&
  restored || fail "a TERM latched before the untraced session ($st)" "$ROOT/sup.out"

# Records that cannot be written (a date stand-in turns the experiment record into a directory at the chosen moment):
# before the untraced session starts, the session does not start; at the final record, the outcome is unresolved.
mkdate() {  # the chosen moment: before-session-A or final
  mkdir -p "$ROOT/bin"; echo "$1" >"$ROOT/trigger.mode"
  cat >"$ROOT/bin/date" <<EOF
#!/usr/bin/env bash
rec=$E/experiment.record
if [ -f "\$rec" ] && [ ! -e "$ROOT/trigger.done" ]; then
  case "\$(cat "$ROOT/trigger.mode")" in
    before-session-A) grep -q '^mask_clear_end=' "\$rec" && ! grep -q '^session_A_start=' "\$rec" && { : >"$ROOT/trigger.done"; mv "\$rec" "\$rec.saved"; mkdir "\$rec"; } ;;
    final) grep -q '^keeper_finish_end=' "\$rec" && { : >"$ROOT/trigger.done"; mv "\$rec" "\$rec.saved"; mkdir "\$rec"; } ;;
  esac
fi
exec $REAL_DATE "\$@"
EOF
  chmod +x "$ROOT/bin/date"
}
reset; mkdate before-session-A
(export PATH=$ROOT/bin:$PATH; start; wait "$sup"; echo $? >"$ROOT/rec.st")
st=$(cat "$ROOT/rec.st")
[ "$st" -eq 3 ] && ! grep -q '^session ' "$ROOT/calls" && [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$WQ_MASK_RECORD" ] && [ ! -e "$QUIET_HOST_STATE" ] &&
  grep -q 'the start of session_A could not be recorded' "$E/supervisor.log" &&
  grep -q 'UNRESOLVED (stop: the start of session_A could not be recorded; unresolved: record)' "$E/supervisor.log" &&
  grep -q '  record: .*experiment.record is incomplete: the outcome is not ACQUIRED' "$E/supervisor.log" &&
  ! grep -q '  mask: \|  quiet_host: ' "$E/supervisor.log" || fail "a record failing before the untraced session ($st)" "$E/supervisor.log"
reset; mkdate final
(export PATH=$ROOT/bin:$PATH; start; wait "$sup"; echo $? >"$ROOT/rec.st")
st=$(cat "$ROOT/rec.st")
[ "$st" -eq 3 ] && grep -q '^session timer' "$ROOT/calls" && [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$WQ_MASK_RECORD" ] && [ ! -e "$QUIET_HOST_STATE" ] &&
  grep -q 'the final record could not be written (the outcome would have been ACQUIRED)' "$E/supervisor.log" &&
  grep -q 'UNRESOLVED (stop: none; unresolved: record)' "$E/supervisor.log" && ! grep -q 'both runs acquired' "$E/supervisor.log" ||
  fail "a final record that cannot be written ($st)" "$E/supervisor.log"

# A process the session left running when the experiment ends (here the session's own background sleep): nothing is
# restored (exit 3), and the released keeper ends without restoring, even once that process has ended.
reset; : >"$ROOT/session.linger.14"
start; wait "$sup"; st=$?
[ "$st" -eq 3 ] && [ "$(masks)" = "$TARGET $TARGET" ] && [ -e "$WQ_MASK_RECORD" ] && [ -e "$QUIET_HOST_STATE" ] &&
  ! grep -q '^mask_restore_start=' "$E/experiment.record" && grep -q '^descendants=.* children=1 unknown=0$' "$E/experiment.record" &&
  grep -q '^keeper_release=.* status=0$' "$E/experiment.record" &&
  grep -q 'outcome=UNRESOLVED reason=none; unresolved: descendants' "$E/experiment.record" &&
  keeper_gone && grep -q 'release: ending WITHOUT restoring' "$(keeper_dir)/keeper.log" || fail "a lingering process ($st)" "$E/supervisor.log"
kill "$(cat "$ROOT/linger.pid")"; sleep 1
[ "$(masks)" = "$TARGET $TARGET" ] || fail "the mask was restored after the release"
# A fork-exit chain, in the group and in a session of its own (outside the group): the supervisor, their subreaper,
# always has a generation among its children.
for how in group setsid; do
  reset; echo "$how" >"$ROOT/session.chain.14"
  start; wait "$sup"; st=$?
  [ "$st" -eq 3 ] && [ "$(masks)" = "$TARGET $TARGET" ] && ! grep -q '^mask_restore_start=' "$E/experiment.record" &&
    grep -Eq '^descendants=.* children=[1-9][0-9]* unknown=0$' "$E/experiment.record" && grep -q '^keeper_release=.* status=0$' "$E/experiment.record" &&
    keeper_gone || fail "a fork-exit chain ($how, $st)" "$E/supervisor.log"
  chain_stop || fail "the chain ($how) did not stop"
  [ "$(masks)" = "$TARGET $TARGET" ] || fail "the mask was restored after the release (chain, $how)"
done

# The keeper alone: its list, its own directory, links planted in it, an owner or a group member that cannot be read.
owner_start() { sed 's/.*) //' "/proc/$1/stat" | cut -d ' ' -f 20; }
keeper() {  # OWNER PGID [env...]: starts a keeper for OWNER, its directory in K
  local o=$1 g=$2
  shift 2
  K=$ROOT/run/rtctrl-keeper-$o-$(owner_start "$o")
  scratch_within "$K" || return 1  # the keeper directory the cases below remove from
  env "$@" tools/root_keeper.sh serve "$o" "$(owner_start "$o")" "$g" 3 || return 1
  wait_file "$K/keeper.ready"
}
c() { ROOT_KEEPER_DIR=$K tools/root_keeper.sh "$@"; }
reset
sleep 60 &
owner=$!
old_umask=$(umask); umask 077  # the keeper's modes must not follow the caller's umask (sudo may pass it on)
keeper "$owner" 999999 || fail "the keeper did not start"
umask "$old_umask"
[ "$(stat -c %a "$K")" = 755 ] && [ "$(stat -c %a "$K/replies")" = 755 ] && [ "$(stat -c %a "$K/requests")" = 600 ] && [ -p "$K/requests" ] &&
  [ "$(stat -c %a "$K/keeper.ready")" = 644 ] && [ "$(stat -c %a "$K/keeper.log")" = 644 ] || fail "the keeper's modes under umask 077"
echo 0000ffff | c tee "$ROOT/sys/cpumask"; st1=$?
echo "$TARGET" | c tee /etc/passwd; st2=$?
c systemctl restart unattended-upgrades.service; st3=$?
out=$(c status); st4=$?
[ "$st1" -eq 99 ] && [ "$st2" -eq 99 ] && [ "$st3" -eq 99 ] && [ "$st4" -eq 0 ] && [ "$out" = "cpumask $ORIG requested $ORIG" ] && [ "$(masks)" = "$ORIG $ORIG" ] ||
  fail "the keeper's list ($st1 $st2 $st3 $st4: $out)" "$K/keeper.log"
for f in "$K"/replies/*; do [ "$(stat -c %a "$f")" = 644 ] || fail "a reply's mode under umask 077: $(stat -c %a "$f") $f"; done
# Links planted where the keeper writes its replies (possible only in the test mode: in /run the directory is root's)
# redirect nothing: a temporary name taken, a final name that is a link, the replies directory itself a link.
printf 'PRESERVE\n' >"$ROOT/victim"; mkdir -p "$ROOT/elsewhere"
ln -s "$ROOT/victim" "$K/replies/.p1.part"
echo "p1 status" >"$K/requests"
ln -s "$ROOT/victim" "$K/replies/p2"
echo "p2 status" >"$K/requests"
for ((i = 0; i < 50; i++)); do grep -q 'request p1: the reply could not be written' "$K/keeper.log" && [ -f "$K/replies/p2" ] && [ ! -L "$K/replies/p2" ] && break; sleep 0.1; done
[ "$(cat "$ROOT/victim")" = PRESERVE ] && grep -q 'request p1: the reply could not be written' "$K/keeper.log" && [ ! -L "$K/replies/p2" ] &&
  grep -q '^status=0$' "$K/replies/p2" || fail "links planted in the replies" "$K/keeper.log"
mv "$K/replies" "$K/replies.real"; ln -s "$ROOT/elsewhere" "$K/replies"
echo "p3 status" >"$K/requests"
for ((i = 0; i < 50; i++)); do grep -q 'request p3: the reply could not be written' "$K/keeper.log" && break; sleep 0.1; done
[ -z "$(ls -A "$ROOT/elsewhere")" ] && grep -q 'request p3: the reply could not be written' "$K/keeper.log" || fail "a replies directory replaced by a link" "$K/keeper.log"
scratch_rm "$K/replies"; mv "$K/replies.real" "$K/replies"
echo "$TARGET" | c tee "$ROOT/sys/cpumask" && [ "$(masks)" = "$TARGET $TARGET" ] || fail "the keeper's target write" "$K/keeper.log"
c finish && [ "$(masks)" = "$ORIG $ORIG" ] || fail "the keeper's finish" "$K/keeper.log"
# Its directory exists already (an earlier keeper's, for this owner): refused.
tools/root_keeper.sh serve "$owner" "$(owner_start "$owner")" 999999 3 2>/dev/null; st=$?
[ "$st" -eq 2 ] || fail "a keeper whose directory exists ($st)"
kill "$owner"

# Fault injection for the keeper alone: a cat that cannot read one owner's stat, a python3 that cannot read one
# process's stat in the group census; everything else unchanged.
mkdir -p "$ROOT/hidebin"
cat >"$ROOT/hidebin/cat" <<EOF
#!/usr/bin/env bash
[ -e "$ROOT/hide.owner" ] && [ "\$1" = "/proc/\$($(command -v cat) "$ROOT/hide.owner")/stat" ] && exit 1
exec $(command -v cat) "\$@"
EOF
cat >"$ROOT/hidebin/python3" <<EOF
#!$REAL_PY
import builtins, os, sys
args = sys.argv[1:]
if "-c" not in args:
    os.execv("$REAL_PY", ["$REAL_PY"] + args)
code = args[args.index("-c") + 1]
hide = "$ROOT/hide.member"
if os.path.exists(hide) and 'os.listdir("/proc")' in code:
    pid = open(hide).read().strip()
    real_open = builtins.open
    def guarded(path, *a, **kw):
        if str(path) == f"/proc/{pid}/stat":
            raise PermissionError("hidden by the test")
        return real_open(path, *a, **kw)
    builtins.open = guarded
sys.argv = ["-c"] + args[args.index("-c") + 2:]
exec(compile(code, "<string>", "exec"), {"__name__": "__main__"})
EOF
chmod +x "$ROOT/hidebin/cat" "$ROOT/hidebin/python3"
# An owner whose /proc entry cannot be read is not taken as ended: the keeper serves on.
reset
sleep 60 &
owner=$!
keeper "$owner" 999999 PATH="$ROOT/hidebin:$PATH" || fail "the keeper (an owner hidden) did not start"
echo "$owner" >"$ROOT/hide.owner"
for ((i = 0; i < 50; i++)); do grep -q 'cannot be read: not taken as ended' "$K/keeper.log" && break; sleep 0.1; done
out=$(c status); st=$?
[ "$st" -eq 0 ] && grep -q 'cannot be read: not taken as ended' "$K/keeper.log" && ! grep -q 'ended without a finish' "$K/keeper.log" ||
  fail "an owner that cannot be read" "$K/keeper.log"
scratch_rm "$ROOT/hide.owner"; c finish >/dev/null; kill "$owner"
# A member of the group that cannot be read is not taken as ended: once the owner has ended, the keeper waits, and when
# its wait is over it ends WITHOUT restoring.
group_owner() {  # an owner, in a group of its own, with one child: their pids in owner and child
  python3 -c 'import os, sys
os.setsid(); os.execvp("bash", ["bash", "-c", "sleep 60 & echo $! >\"$1\"; wait", "owner", sys.argv[1]])' "$ROOT/child.pid" &
  owner=$!
  wait_file "$ROOT/child.pid" || fail "no child"
  child=$(cat "$ROOT/child.pid"); scratch_rm "$ROOT/child.pid"
}
reset
group_owner
keeper "$owner" "$owner" PATH="$ROOT/hidebin:$PATH" KEEPER_GROUP_WAIT=2 || fail "the keeper (a member hidden) did not start"
echo "$TARGET" | c tee "$ROOT/sys/cpumask" || fail "the target write (a member hidden)"
echo "$child" >"$ROOT/hide.member"
kill -KILL "$owner"; wait "$owner" 2>/dev/null
for ((i = 0; i < 80; i++)); do grep -q 'UNRESOLVED' "$K/keeper.log" && break; sleep 0.1; done
[ "$(masks)" = "$TARGET $TARGET" ] && grep -q 'UNRESOLVED: the end of process group .* is not confirmed .*; the mask is NOT restored' "$K/keeper.log" &&
  grep -q "unknown $child" "$K/keeper.log" && [ -e "/proc/$child" ] || fail "a hidden member of the group" "$K/keeper.log"
scratch_rm "$ROOT/hide.member"; kill "$child"
# The same member seen to end while the keeper waits: the keeper restores.
reset
group_owner
keeper "$owner" "$owner" KEEPER_GROUP_WAIT=20 || fail "the keeper (a member running) did not start"
echo "$TARGET" | c tee "$ROOT/sys/cpumask" || fail "the target write (a member running)"
kill -KILL "$owner"; wait "$owner" 2>/dev/null
for ((i = 0; i < 30; i++)); do grep -q 'waiting for a confirmed end' "$K/keeper.log" && break; sleep 0.1; done
sleep 1
[ "$(masks)" = "$TARGET $TARGET" ] || fail "the keeper restored while a member of the group ran" "$K/keeper.log"
kill "$child"
for ((i = 0; i < 50; i++)); do [ "$(masks)" = "$ORIG $ORIG" ] && break; sleep 0.1; done
[ "$(masks)" = "$ORIG $ORIG" ] && grep -q 'the group has ended; the original ffffffff is in effect' "$K/keeper.log" || fail "the keeper after the member ended" "$K/keeper.log"
# The keeper's exact group test (its embedded body), on the re-review's race: a member forks and exits just as a /proc
# scan reaches it (a hook times it), so the scan sees only its zombie and misses the child born after the listing; and
# on a free-running fork-exit chain in a group of its own, sampled 200 times. The kernel's test reports "present"
# throughout, and the chain "ended" once it has stopped.
reset
python3 - "$ROOT" <<'EOF' >"$ROOT/group-race.txt" 2>&1 || fail "the keeper's group test against forks" "$ROOT/group-race.txt"
import os, pathlib, signal, subprocess, sys, time
root = pathlib.Path(sys.argv[1])
src = pathlib.Path("tools/root_keeper.sh").read_text()
body = src.split("group_state() {", 1)[1].split("python3 -I -c '\n", 1)[1].split("\n' \"$1\"", 1)[0]
def wait_for(pred, s=10):
    end = time.monotonic() + s
    while not pred():
        assert time.monotonic() < end, "timeout"
        time.sleep(0.01)
# 1. The race, timed by a hook on the scan's read of the member's stat.
member = r"""import os, pathlib, signal, sys, time
p = pathlib.Path(sys.argv[1])
def fork_exit(s, f):
    c = os.fork()
    if c == 0:
        time.sleep(30); os._exit(0)
    (p / "race.child").write_text(str(c)); os._exit(0)
signal.signal(signal.SIGUSR1, fork_exit)
(p / "race.ready").write_text("ready")
while True: time.sleep(0.1)
"""
m = subprocess.Popen([sys.executable, "-c", member, str(root)], start_new_session=True)
child = None
try:
    wait_for((root / "race.ready").exists)
    hook = r"""import builtins, os, pathlib, signal, sys, time
original = builtins.open
pid, root = int(sys.argv[1]), pathlib.Path(sys.argv[2]); target = f"/proc/{pid}/stat"; fired = False
def guarded(path, *a, **kw):
    global fired
    if str(path) == target and not fired:
        fired = True; os.kill(pid, signal.SIGUSR1)
        end = time.monotonic() + 10
        while not ((root / "race.child").exists() and original(target).read().rsplit(")", 1)[1].split()[0] == "Z"):
            assert time.monotonic() < end; time.sleep(0.001)
    return original(path, *a, **kw)
builtins.open = guarded
sys.argv = ["-c", str(pid)]
exec(BODY)
print("hook fired:", fired)
""".replace("BODY", repr(body))
    r = subprocess.run([sys.executable, "-I", "-c", hook, str(m.pid), str(root)], capture_output=True, text=True, timeout=20)
    out = r.stdout.splitlines()
    child = int((root / "race.child").read_text())
    alive = pathlib.Path(f"/proc/{child}/stat").read_text().rsplit(")", 1)[1].split()[0]
    print("race:", out[0], "| hook fired:", out[-1].endswith("True"), "| the new child:", alive)
    assert r.returncode == 0 and out[0] == "present" and out[-1] == "hook fired: True" and alive in ("S", "R"), (r.stdout, r.stderr)
finally:
    if child is not None:
        os.kill(child, signal.SIGKILL)
    if m.poll() is None:
        m.kill()
    m.wait()
# 2. A free-running fork-exit chain in a group of its own.
first = subprocess.Popen([str(root / "chain")], start_new_session=True, env=os.environ | {"ROOT": str(root)})
first.wait()  # the first generation (the group's id) is reaped at once: what remains is the chain itself
pgid = first.pid
states = [subprocess.run([sys.executable, "-I", "-c", body, str(pgid)], capture_output=True, text=True).stdout.split("\n", 1)[0]
          for _ in range(200)]
print("chain running:", {s: states.count(s) for s in set(states)})
assert states == ["present"] * 200, states
(root / "chain.stop").write_text("")
end = time.monotonic() + 10
while True:
    s = subprocess.run([sys.executable, "-I", "-c", body, str(pgid)], capture_output=True, text=True).stdout.split("\n", 1)[0]
    if s == "ended":
        break
    assert time.monotonic() < end, "the chain's group never ended"
    time.sleep(0.05)
print("chain stopped:", s)
EOF
cat "$ROOT/group-race.txt"
scratch_release
echo "option_a smoke OK"
