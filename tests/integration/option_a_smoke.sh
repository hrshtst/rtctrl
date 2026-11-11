#!/usr/bin/env bash
# Smoke test of tools/option_a.sh and tools/root_keeper.sh: the real
# supervisor, keeper (its unprivileged test mode) and tools/wq_mask.sh, with
# stand-ins for the kernel's mask files and workers, systemctl, the session,
# the gate and tools/quiet_host.sh.
#
# A normal experiment runs, in order, the quieting, the mask's clear, the
# untraced session (without the controls), the check and gate, the controls
# with the timer session, the check and gate, then restores and verifies
# everything and ends the keeper (exit 0). Refusals change nothing (fresh
# directories, a failing dry run, an open record). A failed session, a
# failed gate, a hangup or an interrupt reaching the process group during a
# session, and a TERM to the supervisor alone each stop it with no further
# step and every restoration verified, after the session's own cleanup
# (exit 1). A restoration that fails is unresolved (exit 3, its record
# open). A supervisor killed with SIGKILL leaves the keeper to restore the
# mask once the group's processes have ended. The keeper refuses anything
# outside its list, and a directory that is not the user's private one.
#
# usage: option_a_smoke.sh <scratch dir>
set -u
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "option_a_smoke" "${1:-}" || exit 2  # a new child of an approved base: the only place this test removes
ROOT=$RTCTRL_SCRATCH
mkdir -p "$ROOT"
fail() { echo "FAILED: $1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }
ORIG=ffffffff
TARGET=fffffff7
OPTION_A_KERNEL=$(uname -r)
export ROOT OPTION_A_OUT=$ROOT/out OPTION_A_PORT=$ROOT/pty OPTION_A_KERNEL OPTION_A_SUDO=none OPTION_A_YES=1
export OPTION_A_RTLIMIT=95 OPTION_A_TRACE_SESSION=$ROOT/session OPTION_A_GATE=$ROOT/gate OPTION_A_QUIET=$ROOT/quiet
export ROOT_KEEPER_TEST=1 KEEPER_SYSFS=$ROOT/sys KEEPER_WRITE=$ROOT/kernel KEEPER_SYSTEMCTL=$ROOT/systemctl
export WQ_MASK_SYSFS=$ROOT/sys WQ_MASK_PROC=$ROOT/proc WQ_MASK_RECORD=$ROOT/state/wq.rec WQ_MASK_WAIT=1
export QUIET_HOST_STATE=$ROOT/state/quiet.state
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
chmod +x "$ROOT/kernel" "$ROOT/systemctl" "$ROOT/quiet" "$ROOT/session" "$ROOT/gate"

reset() {  # the host as it starts
  scratch_rm "$ROOT/out" "$ROOT/state" "$ROOT/proc" "$ROOT/sys" "$ROOT"/calls "$ROOT"/cleaned-* "$ROOT"/mask-during-* "$ROOT"/running-* \
         "$ROOT"/session.* "$ROOT"/gate.exit.* "$ROOT/dry.exit" "$ROOT/kernel.mode" "$ROOT/uu.log" "$ROOT/sup.out"
  mkdir -p "$ROOT/sys" "$ROOT/state" "$ROOT/proc"
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
keeper_gone() {  # waits up to 5 s for the experiment's keeper to end
  local pid i
  pid=$(sed -n 's/^pid=//p' "$E/keeper/keeper.ready" 2>/dev/null)
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
grep -q 'request .*: mask fffffff7: status 0' "$E/keeper/keeper.log" && grep -q 'finish: the original is in effect' "$E/keeper/keeper.log" ||
  fail "the keeper's log" "$E/keeper/keeper.log"

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
  grep -q '^finished=.* outcome=UNRESOLVED reason=none unresolved: mask keeper$' "$E/experiment.record" || fail "a failed restoration ($st)" "$ROOT/sup.out"
keeper_gone || fail "the keeper stayed after a failed restoration"

# The supervisor killed (SIGKILL) during the untraced session: the keeper waits for the group to end, then restores the mask.
reset; echo 2 >"$ROOT/session.secs"
start
wait_file "$ROOT/running-14" || fail "session 14 did not start" "$ROOT/sup.out"
kill -KILL "$sup"; wait "$sup" 2>/dev/null
keeper_log=$E/keeper/keeper.log
for ((i = 0; i < 100; i++)); do grep -q 'is in effect; ending' "$keeper_log" 2>/dev/null && break; sleep 0.1; done
grep -q 'ended without a finish' "$keeper_log" && grep -q 'is in effect; ending' "$keeper_log" && [ "$(masks)" = "$ORIG $ORIG" ] &&
  [ -e "$WQ_MASK_RECORD" ] && keeper_gone || fail "the keeper after a killed supervisor" "$keeper_log"
# The session (still in the group) had ended before the keeper wrote the original back.
[ "$(stat -c %Y "$ROOT/sys/cpumask")" -ge "$(stat -c %Y "$ROOT/mask-during-14")" ] || fail "the keeper restored before the session ended"
WQ_MASK_SUDO=none tools/wq_mask.sh restore >/dev/null 2>&1 || :

# The keeper alone: its list, its directory.
reset
K=$ROOT/k; scratch_within "$K" || exit 2; mkdir -m 700 "$K"; mkfifo -m 600 "$K/requests"; mkdir -m 700 "$K/replies"
sleep 30 &
owner=$!
ostart=$(sed 's/.*) //' "/proc/$owner/stat" | cut -d ' ' -f 20)
tools/root_keeper.sh serve "$K" "$owner" "$ostart" 999999 3 || fail "the keeper did not start"
wait_file "$K/keeper.ready" || fail "the keeper is not ready" "$K/keeper.log"
c() { ROOT_KEEPER_DIR=$K tools/root_keeper.sh "$@"; }
echo 0000ffff | c tee "$ROOT/sys/cpumask"; st1=$?
echo "$TARGET" | c tee /etc/passwd; st2=$?
c systemctl restart unattended-upgrades.service; st3=$?
out=$(c status); st4=$?
[ "$st1" -eq 99 ] && [ "$st2" -eq 99 ] && [ "$st3" -eq 99 ] && [ "$st4" -eq 0 ] && [ "$out" = "cpumask $ORIG requested $ORIG" ] && [ "$(masks)" = "$ORIG $ORIG" ] ||
  fail "the keeper's list ($st1 $st2 $st3 $st4: $out)" "$K/keeper.log"
echo "$TARGET" | c tee "$ROOT/sys/cpumask" && [ "$(masks)" = "$TARGET $TARGET" ] || fail "the keeper's target write" "$K/keeper.log"
c finish && [ "$(masks)" = "$ORIG $ORIG" ] || fail "the keeper's finish" "$K/keeper.log"
kill "$owner" 2>/dev/null
chmod 755 "$K"; scratch_rm "$K/keeper.ready"
tools/root_keeper.sh serve "$K" "$$" "$(sed 's/.*) //' "/proc/$$/stat" | cut -d ' ' -f 20)" 999999 3 2>/dev/null; st=$?
[ "$st" -eq 2 ] && [ ! -e "$K/keeper.ready" ] || fail "a keeper in a directory that is not private ($st)"
scratch_release
echo "option_a smoke OK"
