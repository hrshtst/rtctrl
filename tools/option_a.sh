#!/usr/bin/env bash
# The supervisor of the loaded emulator validation's Option A
# (docs/hardware/control-rate.md): one process that owns the experiment from
# its first host change to the last verified restoration, run by the
# operator from the repository root.
#
# usage: tools/option_a.sh [--dry-run]
#
# Before any host change it refuses (exit 2) unless: the kernel is the
# audited one; the emulator's pty exists and is no serial device; the
# real-time priority limit allows the probe's 80; tools/wq_mask.sh and
# tools/quiet_host.sh have no open record; the experiment's three
# directories do not exist (it then creates them, fresh); both session
# commands pass trace_session.sh's dry run; the operator confirms; sudo
# authenticates; and the root keeper (tools/root_keeper.sh) is ready in its
# own root-owned directory. With --dry-run it stops after the session dry
# runs, with scratch directories and nothing changed.
#
# Then the experiment's steps, in order:
#   1. tools/quiet_host.sh stop;
#   2. tools/wq_mask.sh clear 3;
#   3. the untraced session (attempt 14), without the controls;
#   4. tools/wq_mask.sh check, then tools/validation_gate.py on the untraced
#      run (its T3 above all): the timer session starts only if it passes;
#   5. the controls and the timer session (attempt 15);
#   6. tools/wq_mask.sh check, then the gate on the timer run, with the
#      controls.
# No step starts unless no stop is latched, the obligation it creates (steps
# 1 and 2) is recorded, and its start is recorded; each step's end and status
# are recorded once it returns. Any nonzero status, a failed record or a
# signal stops it: no further step. The privileged steps go through the
# keeper (passed as the tools' sudo), which needs no sudo after its start;
# the sessions use their own sudo as before.
#
# Signals: INT, TERM and HUP are latched. The sessions run in the
# foreground, in this script's process group, so a Ctrl-C or a hangup of the
# terminal reaches the running session too, which runs its own cleanup; the
# supervisor acts once the session has returned (a signal sent to the
# supervisor alone therefore lets the running step finish, and one that
# arrives after a step's last check, before its command starts, is acted on
# when that command returns).
#
# The supervisor is the child subreaper of everything it starts
# (PR_SET_CHILD_SUBREAPER, set through perl, which leaves every signal's
# disposition as it was, before it execs this script again): a descendant
# whose parent ends is reparented to it, not to init, whatever its process
# group or session. Nothing else about the steps changes.
#
# The cleanup ignores INT, TERM and HUP. It first needs a confirmed end of
# every other process it started: the kernel's list of its children holds
# nothing but the keeper (and the check's own process). Every running
# descendant has an ancestor among those children (a zombie counts until it
# is reaped), the list is read from the kernel in one pass, and no new
# descendant can appear once it is empty (only this script and the keeper
# start processes), so a fork during the check cannot escape it, as it can
# escape a /proc scan. A list that cannot be read is no end. It waits up to
# 120 s for that; without it, it changes nothing on the host: it names what
# remained, releases the keeper (which then ends without restoring: its own
# test reaches only the process group), and ends unresolved (exit 3), for
# the operator to restore by hand once those processes have ended. With it,
# it restores and verifies: tools/wq_mask.sh restore, tools/quiet_host.sh
# restore, then the keeper's finish (which verifies the original mask), and
# copies the keeper's log. An owed restoration that is not verified, or a
# record that could not be written, the final one included, leaves it
# unresolved (exit 3): the records stay open, and no session may start until
# they are restored. If the supervisor itself is killed (SIGKILL) or crashes,
# only the keeper acts: it restores the mask once the kernel finds no process
# left in the supervisor's process group (the limit of this crash path: a
# descendant that has left the group is beyond that test, and after the
# supervisor's death nothing tracks it); nothing restores the quieted services then,
# and a crash of the keeper too, or a power loss, restores nothing (a reboot
# resets the kernel's mask). Its messages go to the experiment's log and,
# while it can, to the terminal.
#
# The experiment record (<dir>/experiment.record): started=TIME, then
# NAME_start=TIME and NAME_end=TIME status=N for each step (quiet_stop,
# mask_clear, session_A, check_A, gate_A, session_B, check_B, gate_B,
# mask_restore, quiet_restore, keeper_finish), quiet_owed=TIME and
# mask_owed=TIME before those changes, signal=TIME NAME,
# descendants=TIME children=N unknown=0|1 and keeper_release=TIME status=N
# for an unconfirmed end, and last
# finished=TIME outcome=ACQUIRED|STOPPED|UNRESOLVED reason=TEXT.
#
# Exit: 0 both runs acquired and both gates passed, every restoration
# verified and every record written (the acceptance is then judged on the
# captures); 1 stopped, every restoration verified and every record written;
# 2 refused, nothing changed; 3 unresolved: the group's end not confirmed, an
# owed restoration not verified, or a record not written.
#
# For tests: OPTION_A_OUT replaces ~/rtctrl-hw, OPTION_A_PORT the emulator's
# pty, OPTION_A_KERNEL the audited kernel, OPTION_A_TRACE_SESSION,
# OPTION_A_GATE, OPTION_A_QUIET, OPTION_A_WQ and OPTION_A_KEEPER the tools,
# OPTION_A_SUDO sudo (none: directly), OPTION_A_YES=1 the confirmation,
# OPTION_A_RTLIMIT the real-time priority limit read, OPTION_A_GROUP_WAIT the
# group's wait in seconds, QUIET_HOST_STATE and WQ_MASK_RECORD the two
# records, and with ROOT_KEEPER_TEST=1 KEEPER_RUNDIR the keeper's /run.
set -u
if [ "${OPTION_A_SUBREAPER:-}" != "$$" ]; then  # the same process, now a child subreaper (an exec keeps the pid)
  shopt -s execfail
  OPTION_A_SUBREAPER=$$ exec perl -e '
    eval { require "syscall.ph"; 1 } or do { print STDERR "option_a: no syscall.ph for perl: nothing changed\n"; exit 2 };
    syscall(&SYS_prctl, 36, 1, 0, 0, 0) == 0 or do { print STDERR "option_a: cannot become a child subreaper: $!: nothing changed\n"; exit 2 };
    exec { $ARGV[0] } @ARGV or do { print STDERR "option_a: cannot run $ARGV[0]: $!: nothing changed\n"; exit 2 };' -- "$BASH" "$0" "$@"
  echo "option_a: cannot run perl: nothing changed" >&2
  exit 2
fi
unset OPTION_A_SUBREAPER
[ -f tools/trace_session.sh ] || { echo "option_a: run from the repository root" >&2; exit 2; }
OUT=${OPTION_A_OUT:-$HOME/rtctrl-hw}
PORT=${OPTION_A_PORT:-/tmp/rtctrl-rehearsal-tty}
KERNEL=${OPTION_A_KERNEL:-7.0.0-42-generic}
TRACE_SESSION=${OPTION_A_TRACE_SESSION:-tools/trace_session.sh}
read -ra GATE <<<"${OPTION_A_GATE:-uv run --project tools tools/validation_gate.py}"
QUIET=${OPTION_A_QUIET:-tools/quiet_host.sh}
WQ=${OPTION_A_WQ:-tools/wq_mask.sh}
KEEPER=$(readlink -f "${OPTION_A_KEEPER:-tools/root_keeper.sh}")
case "${OPTION_A_SUDO:-sudo}" in none) SUDO=() ;; *) SUDO=("${OPTION_A_SUDO:-sudo}") ;; esac
QSTATE=${QUIET_HOST_STATE:-${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/rtctrl-quiet-host.state}
WREC=${WQ_MASK_RECORD:-${XDG_STATE_HOME:-$HOME/.local/state}/rtctrl-wq-mask.record}
KRUN=/run
[ "${ROOT_KEEPER_TEST:-0}" = 1 ] && KRUN=${KEEPER_RUNDIR:-/run}
GROUP_WAIT=${OPTION_A_GROUP_WAIT:-120}
RT_CPU=3
D14=$OUT/clock-validation-14
D15=$OUT/clock-validation-15
E=$OUT/clock-validation-option-a
COMMON=(--port "$PORT" --usb-device /sys/bus/usb/devices/1-9 --placements open
        --runs current:1000:rt-loaded-excl:fast-motion --duration 20)
SESSION_A=(--modes untraced --attempt 14 --outdir "$D14")
SESSION_B=(--fs-controls 5 --modes timer --attempt 15 --outdir "$D15")

now() { date +%Y-%m-%dT%H:%M:%S.%N%z; }
LOG=
log() {  # to the log (once it exists) and, while it can, the terminal
  local l
  l="[$(now)] $*"
  [ -n "$LOG" ] && printf '%s\n' "$l" >>"$LOG" 2>/dev/null
  printf '%s\n' "$l" 2>/dev/null || :
}
refuse() { log "option_a: $*: nothing changed"; exit 2; }
record_failed=0
rec() { printf '%s\n' "$1" >>"$E/experiment.record" 2>/dev/null || { record_failed=1; log "cannot append to $E/experiment.record"; return 1; }; }
one() {  # DIR PATTERN: the one file matching, or status 1
  local f
  mapfile -t f < <(find "$1" -maxdepth 1 -type f -name "$2" 2>/dev/null)
  [ ${#f[@]} -eq 1 ] || return 1
  printf '%s\n' "${f[0]}"
}

dry=0
case "${1:-}" in --dry-run) dry=1 ;; '') ;; *) echo "usage: $0 [--dry-run]" >&2; exit 2 ;; esac
[ $# -le 1 ] || { echo "usage: $0 [--dry-run]" >&2; exit 2; }

# --- Before any host change: refusals only. ---
[ "$(uname -r)" = "$KERNEL" ] || refuse "the kernel is $(uname -r), not $KERNEL"
[ -e "$PORT" ] || refuse "the emulator's pty $PORT does not exist (is dxl_emu running?)"
pty=$(basename "$(readlink -f "$PORT")")
[ ! -e "/sys/class/tty/$pty/device" ] || refuse "$PORT is a serial device ($pty), not the emulator's pty"
rtl=${OPTION_A_RTLIMIT:-$(ulimit -r)}
[ "$rtl" = unlimited ] || [ "$rtl" -ge 80 ] || refuse "the real-time priority limit is $rtl, below the probe's 80 (sudo prlimit --pid \$\$ --rtprio=90:90)"
[ ! -e "$WREC" ] || refuse "a workqueue mask record is open ($WREC): tools/wq_mask.sh restore first"
[ ! -e "$QSTATE" ] || refuse "a quiet_host record is open ($QSTATE): tools/quiet_host.sh restore first"
for d in "$D14" "$D15" "$E"; do [ ! -e "$d" ] || refuse "$d exists: the experiment needs fresh directories"; done
if [ "$dry" -eq 1 ]; then
  s=$(mktemp -d) || refuse "no scratch directory"
  mkdir "$s/14" "$s/15"
  st=0
  "$TRACE_SESSION" --dry-run --yes "${COMMON[@]}" --modes untraced --attempt 14 --outdir "$s/14" || st=$?
  [ "$st" -eq 0 ] && { "$TRACE_SESSION" --dry-run --yes "${COMMON[@]}" --fs-controls 5 --modes timer --attempt 15 --outdir "$s/15" || st=$?; }
  rm -rf "$s"
  [ "$st" -eq 0 ] || refuse "a session's dry run failed (status $st)"
  log "option_a: dry run: both session commands are accepted; nothing changed"
  exit 0
fi
mkdir "$OUT" 2>/dev/null
mkdir "$D14" && mkdir "$D15" && mkdir "$E" || { rmdir "$D14" "$D15" "$E" 2>/dev/null; refuse "cannot create the experiment's directories"; }
LOG=$E/supervisor.log
undo_dirs() { rmdir "$D14" "$D15" 2>/dev/null; rm -f "$LOG" "$E/experiment.record"; rmdir "$E" 2>/dev/null; }
for spec in "untraced 14 $D14" "timer 15 $D15"; do
  read -r m a d <<<"$spec"
  extra=(); [ "$m" = timer ] && extra=(--fs-controls 5)
  if ! "$TRACE_SESSION" --dry-run --yes "${COMMON[@]}" "${extra[@]}" --modes "$m" --attempt "$a" --outdir "$d" >>"$LOG" 2>&1; then
    cat "$LOG" 2>/dev/null; undo_dirs; refuse "the $m session's dry run failed"
  fi
done
if [ "${OPTION_A_YES:-0}" != 1 ]; then
  cat <<EOF
The loaded emulator validation, Option A: an untraced and a timer run on $PORT (attempts 14 and 15),
with CPU $RT_CPU removed from the kernel's mask for unbound workqueues (a changed host condition, restored at the end).
Check that:
  - the arm is powered off, or torque-off and supported; nothing is sent to it;
  - the power cutoff is within reach;
  - nothing else uses the port or loads the machine.
Type yes to start:
EOF
  read -r answer </dev/tty || answer=
  [ "$answer" = yes ] || { undo_dirs; refuse "not confirmed"; }
fi
if [ ${#SUDO[@]} -gt 0 ]; then "${SUDO[@]}" -v || { undo_dirs; refuse "sudo did not authenticate"; }; fi
me_start=$(sed 's/.*) //' "/proc/$$/stat" | cut -d ' ' -f 20)
me_pgid=$(sed 's/.*) //' "/proc/$$/stat" | cut -d ' ' -f 3)
K=$KRUN/rtctrl-keeper-$$-$me_start
[ ! -e "$K" ] || { undo_dirs; refuse "the keeper's directory $K exists"; }
"${SUDO[@]}" "$KEEPER" serve "$$" "$me_start" "$me_pgid" "$RT_CPU" || { undo_dirs; refuse "the root keeper did not start"; }
for ((i = 0; i < 100; i++)); do [ -e "$K/keeper.ready" ] && break; sleep 0.1; done
kpid=$(sed -n 's/^pid=//p' "$K/keeper.ready" 2>/dev/null)
kstart=$(sed 's/.*) //' "/proc/$kpid/stat" 2>/dev/null | cut -d ' ' -f 20)
[ -n "$kpid" ] && [[ $kstart =~ ^[0-9]+$ ]] || { undo_dirs; refuse "the root keeper is not ready (it ends by itself when this script does)"; }
KENV=(WQ_MASK_SUDO="$KEEPER" QUIET_HOST_SUDO="$KEEPER" ROOT_KEEPER_DIR="$K")

# --- The experiment: owned from here on. ---
rec "started=$(now) kernel=$(uname -r) supervisor=$$ group=$me_pgid keeper=$kpid keeper_start=$kstart keeper_dir=$K" ||
  { env ROOT_KEEPER_DIR="$K" "$KEEPER" finish >/dev/null 2>&1; refuse "cannot write the experiment record"; }
rec "directories=$D14 $D15 $E" || { env ROOT_KEEPER_DIR="$K" "$KEEPER" finish >/dev/null 2>&1; refuse "cannot write the experiment record"; }
quiet_owed=0; mask_owed=0; keeper_up=1; stopped=; finished=0
on_signal() { rec "signal=$(now) $1"; [ -n "$stopped" ] || stopped="signal $1"; log "option_a: $1: no further step; the restoration follows once the running step returns"; }
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM
trap 'on_signal HUP' HUP

# descendants: status 0 and "ended" only if the kernel's list of this script's children holds nothing but the keeper
# (its pid with its start time) and this check's own processes (see the cleanup above); otherwise status 1, with
# "present N" and the other children, or "unknown" and why the list cannot be read.
descendants() {
  python3 -I -c '
import os, sys
sup, kpid, kstart = sys.argv[1], sys.argv[2], sys.argv[3]
def stat(p):
    s = open(f"/proc/{p}/stat").read()
    return s[s.index("(") + 1:s.rindex(")")], s[s.rindex(")") + 2:].split()
try:
    own, p = set(), str(os.getpid())
    while p != sup:  # this check, and any subshell between it and the supervisor
        own.add(p)
        p = stat(p)[1][1]
        if int(p) <= 1:
            raise ValueError("this check does not descend from the supervisor")
    with open(f"/proc/{sup}/task/{sup}/children") as f:
        kids = f.read().split()
except (OSError, ValueError, IndexError) as e:
    print("unknown")
    print(f"  the list of its children cannot be read: {e}")
    sys.exit(1)
others = []
for k in kids:
    if k in own:
        continue
    try:
        comm, f = stat(k)
        if k == kpid and f[19] == kstart:
            continue
        others.append(f"  child {k} {comm} {f[0]}")
    except (OSError, ValueError, IndexError):
        others.append(f"  child {k} (cannot be read)")
if others:
    print("present", len(others))
    print("\n".join(others))
    sys.exit(1)
print("ended")
' "$$" "$kpid" "$kstart"
}
finish() {
  [ "$finished" -eq 1 ] && return
  finished=1
  trap '' INT TERM HUP
  local i g='' unresolved='' outcome code reason
  # A confirmed end of every other process it started comes first: nothing changes on the host before it.
  for ((i = 0; i < GROUP_WAIT * 10; i++)); do
    g=$(descendants) && break
    sleep 0.1
  done
  if [ "$i" -ge $((GROUP_WAIT * 10)) ]; then
    case "$(head -n 1 <<<"${g:-unknown}")" in
      present\ *) rec "descendants=$(now) children=$(head -n 1 <<<"$g" | cut -d ' ' -f 2) unknown=0" ;;
      *) rec "descendants=$(now) children=? unknown=1" ;;
    esac
    log "option_a: UNRESOLVED: the end of the processes it started is not confirmed after $GROUP_WAIT s; nothing is restored:"
    log "${g:-the list of its children could not be read}"
    # The keeper's own test reaches only the process group, not a descendant that has left it: released, it restores
    # nothing either.
    local rst
    env ROOT_KEEPER_DIR="$K" "$KEEPER" release >>"$LOG" 2>&1
    rst=$?
    rec "keeper_release=$(now) status=$rst"
    if [ "$rst" -eq 0 ]; then
      log "  the keeper has ended WITHOUT restoring: only the operator (or a reboot) restores the mask now"
    else
      log "  the keeper did not confirm its release (status $rst): it may restore the mask once the kernel finds the group empty"
    fi
    log "  once the processes above have ended, restore by hand: tools/wq_mask.sh restore, tools/quiet_host.sh restore"
    log "  (the keeper's log: $K/keeper.log)"
    cp "$K/keeper.log" "$E/keeper.log" 2>/dev/null
    rec "finished=$(now) outcome=UNRESOLVED reason=${stopped:-none}; unresolved: descendants"
    exit 3
  fi
  cleanup_step() {  # NAME CMD...: a restoration runs whatever the record does
    local name=$1 st
    shift
    rec "${name}_start=$(now)"
    "$@"
    st=$?
    rec "${name}_end=$(now) status=$st"
    return "$st"
  }
  if [ "$mask_owed" -eq 1 ]; then
    if cleanup_step mask_restore env "${KENV[@]}" "$WQ" restore >>"$LOG" 2>&1; then mask_owed=0; else unresolved+=" mask"; fi
  fi
  if [ "$quiet_owed" -eq 1 ]; then
    if cleanup_step quiet_restore env "${KENV[@]}" "$QUIET" restore >>"$LOG" 2>&1; then quiet_owed=0; else unresolved+=" quiet_host"; fi
  fi
  if [ "$keeper_up" -eq 1 ]; then
    if cleanup_step keeper_finish env ROOT_KEEPER_DIR="$K" "$KEEPER" finish >>"$LOG" 2>&1; then keeper_up=0; else unresolved+=" keeper"; fi
  fi
  cp "$K/keeper.log" "$E/keeper.log" 2>/dev/null || { record_failed=1; log "option_a: cannot copy the keeper's log $K/keeper.log"; }
  [ "$record_failed" -eq 0 ] || unresolved+=" record"
  reason=${stopped:-none}${unresolved:+; unresolved:$unresolved}
  if [ -n "$unresolved" ]; then outcome=UNRESOLVED; code=3
  elif [ -n "$stopped" ]; then outcome=STOPPED; code=1
  else outcome=ACQUIRED; code=0
  fi
  if ! rec "finished=$(now) outcome=$outcome reason=$reason"; then
    log "option_a: UNRESOLVED: the final record could not be written (the outcome would have been $outcome)"
    case " $unresolved " in *" record "*) ;; *) unresolved+=" record" ;; esac
    code=3
  fi
  case "$code" in
    0) log "option_a: both runs acquired and both gates passed; every restoration verified" ;;
    1) log "option_a: STOPPED ($stopped); every restoration verified" ;;
    *) log "option_a: UNRESOLVED (stop: ${stopped:-none}; unresolved:$unresolved); no session may start until each is resolved:"
       local u
       for u in $unresolved; do
         case "$u" in
           mask) log "  mask: its restoration is not verified: tools/wq_mask.sh restore" ;;
           quiet_host) log "  quiet_host: the services' restoration is not verified: tools/quiet_host.sh restore" ;;
           keeper) log "  keeper: its finish is not verified (it may still serve): read $K/keeper.log, check the masks" ;;
           record) log "  record: $E/experiment.record is incomplete: the outcome is not ACQUIRED; the log $LOG holds the steps" ;;
         esac
       done ;;
  esac
  exit "$code"
}
trap finish EXIT
halt() { [ -n "$stopped" ] || stopped=$1; log "option_a: stop: $stopped"; finish; }
ready() {  # no new work after a latched stop or a failed record
  [ -z "$stopped" ] || halt "$stopped"
  [ "$record_failed" -eq 0 ] || halt "the experiment record could not be written"
}
owe() {  # NAME: the obligation recorded before the change that creates it
  ready
  rec "${1}_owed=$(now)" || halt "the obligation $1 could not be recorded"
  ready
}
step() {  # NAME CMD...: an experiment step, started only once ready and its start recorded; its status
  local name=$1 st
  shift
  ready
  rec "${name}_start=$(now)" || halt "the start of $name could not be recorded"
  ready
  "$@"
  st=$?
  rec "${name}_end=$(now) status=$st"
  return "$st"
}
ok() {  # STATUS WHAT: halts unless the step succeeded, no signal came and the record holds
  [ "$1" -eq 0 ] || halt "$2 (status $1)"
  ready
}

owe quiet; quiet_owed=1
step quiet_stop env "${KENV[@]}" "$QUIET" stop; ok $? "tools/quiet_host.sh stop failed"
owe mask; mask_owed=1
step mask_clear env "${KENV[@]}" "$WQ" clear "$RT_CPU"; ok $? "tools/wq_mask.sh clear $RT_CPU failed"
step session_A "$TRACE_SESSION" --yes "${COMMON[@]}" "${SESSION_A[@]}"; ok $? "the untraced session failed"
step check_A "$WQ" check; ok $? "the mask was not in effect after the untraced session"
csv_a=$(one "$D14" '*-untraced-*-14.csv') || halt "no single untraced probe log in $D14"
step gate_A "${GATE[@]}" "$csv_a" --mode untraced --wq-record "$WREC" --experiment "$E/experiment.record" --session A --record-state open
ok $? "the gate on the untraced run failed: no timer run"
step session_B "$TRACE_SESSION" --yes "${COMMON[@]}" "${SESSION_B[@]}"; ok $? "the controls or the timer session failed"
step check_B "$WQ" check; ok $? "the mask was not in effect after the timer session"
csv_b=$(one "$D15" '*-timer-*-15.csv') || halt "no single timer probe log in $D15"
ctl=$(one "$D15" 'fs-controls-*-15.run.txt') || halt "no single controls record in $D15"
step gate_B "${GATE[@]}" "$csv_b" --mode timer --controls "$ctl" --wq-record "$WREC" --experiment "$E/experiment.record" --session B --record-state open
ok $? "the gate on the timer run failed"
finish
