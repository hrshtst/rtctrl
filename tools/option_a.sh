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
# authenticates; and the root keeper (tools/root_keeper.sh) is ready. With
# --dry-run it stops after the session dry runs, with scratch directories
# and nothing changed.
#
# Then, each step's start and end (with its status) in the experiment record,
# and each obligation recorded before the change that creates it:
#   1. tools/quiet_host.sh stop;
#   2. tools/wq_mask.sh clear 3;
#   3. the untraced session (attempt 14), without the controls;
#   4. tools/wq_mask.sh check, then tools/validation_gate.py on the untraced
#      run (its T3 above all): the timer session starts only if it passes;
#   5. the controls and the timer session (attempt 15);
#   6. tools/wq_mask.sh check, then the gate on the timer run, with the
#      controls.
# Any nonzero status, or a signal, stops it there: no further step. The
# privileged steps go through the keeper (passed as the tools' sudo), which
# needs no sudo after the start; the sessions use their own sudo as before.
#
# Signals: INT, TERM and HUP are latched. The sessions run in the
# foreground, in this script's process group, so a Ctrl-C or a hangup of the
# terminal reaches the running session too, which runs its own cleanup; the
# supervisor acts once the session has returned (a signal sent to the
# supervisor alone therefore lets the running session finish). Its cleanup
# then ignores INT, TERM and HUP, waits (up to 120 s) for any other process
# of its group to end, and restores, verifying each: tools/wq_mask.sh
# restore, tools/quiet_host.sh restore, then the keeper's finish (which
# verifies the original mask). An owed restoration that is not verified is
# unresolved (exit 3): its record stays open, and no session may start until
# it is restored. If the supervisor itself is killed (SIGKILL) or crashes,
# the keeper waits for the group's processes to end and restores the mask,
# leaving the records open; nothing restores the quieted services then, and
# a crash of the keeper too, or a power loss, restores nothing (a reboot
# resets the kernel's mask). Its messages go to the experiment's log and,
# while it can, to the terminal.
#
# The experiment record (<dir>/experiment.record): started=TIME, then
# NAME_start=TIME and NAME_end=TIME status=N for each step (quiet_stop,
# mask_clear, session_A, check_A, gate_A, session_B, check_B, gate_B,
# mask_restore, quiet_restore, keeper_finish), quiet_owed=TIME and
# mask_owed=TIME before those changes, signal=TIME NAME, and last
# finished=TIME outcome=ACQUIRED|STOPPED|UNRESOLVED reason=TEXT.
#
# Exit: 0 both runs acquired and both gates passed, every restoration
# verified (the acceptance is then judged on the captures); 1 stopped, every
# restoration verified; 2 refused, nothing changed; 3 an owed restoration
# not verified.
#
# For tests: OPTION_A_OUT replaces ~/rtctrl-hw, OPTION_A_PORT the emulator's
# pty, OPTION_A_KERNEL the audited kernel, OPTION_A_TRACE_SESSION,
# OPTION_A_GATE, OPTION_A_QUIET, OPTION_A_WQ and OPTION_A_KEEPER the tools,
# OPTION_A_SUDO sudo (none: directly), OPTION_A_YES=1 the confirmation,
# OPTION_A_RTLIMIT the real-time priority limit read, and
# QUIET_HOST_STATE and WQ_MASK_RECORD the two records.
set -u
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
RT_CPU=3
D14=$OUT/clock-validation-14
D15=$OUT/clock-validation-15
E=$OUT/clock-validation-option-a
K=$E/keeper
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
step() {  # NAME CMD...: the command's status; its start and end recorded (the command runs even if a record fails)
  local name=$1 st
  shift
  rec "${name}_start=$(now)"
  "$@"
  st=$?
  rec "${name}_end=$(now) status=$st"
  return "$st"
}
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
undo_dirs() { rmdir "$K/replies" 2>/dev/null; rm -f "$K/requests"; rmdir "$K" "$D14" "$D15" 2>/dev/null; rm -f "$LOG" "$E/experiment.record"; rmdir "$E" 2>/dev/null; }
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
mkdir -m 700 "$K" && mkfifo -m 600 "$K/requests" && mkdir -m 700 "$K/replies" || { undo_dirs; refuse "cannot create the keeper's directory"; }
me_start=$(sed 's/.*) //' "/proc/$$/stat" | cut -d ' ' -f 20)
me_pgid=$(sed 's/.*) //' "/proc/$$/stat" | cut -d ' ' -f 3)
"${SUDO[@]}" "$KEEPER" serve "$K" "$$" "$me_start" "$me_pgid" "$RT_CPU" || { undo_dirs; refuse "the root keeper did not start"; }
for ((i = 0; i < 100; i++)); do [ -e "$K/keeper.ready" ] && break; sleep 0.1; done
kpid=$(sed -n 's/^pid=//p' "$K/keeper.ready" 2>/dev/null)
[ -n "$kpid" ] && [ -e "/proc/$kpid" ] || { undo_dirs; refuse "the root keeper is not ready (it ends by itself when this script does)"; }
KENV=(WQ_MASK_SUDO="$KEEPER" QUIET_HOST_SUDO="$KEEPER" ROOT_KEEPER_DIR="$K")

# --- The experiment: owned from here on. ---
rec "started=$(now) kernel=$(uname -r) supervisor=$$ group=$me_pgid keeper=$kpid" || { env ROOT_KEEPER_DIR="$K" "$KEEPER" finish >/dev/null 2>&1; refuse "cannot write the experiment record"; }
rec "directories=$D14 $D15 $E"
quiet_owed=0; mask_owed=0; keeper_up=1; stopped=; finished=0
on_signal() { rec "signal=$(now) $1"; [ -n "$stopped" ] || stopped="signal $1"; log "option_a: $1: no further step; the restoration follows once the running step returns"; }
trap 'on_signal INT' INT
trap 'on_signal TERM' TERM
trap 'on_signal HUP' HUP
finish() {
  [ "$finished" -eq 1 ] && return
  finished=1
  trap '' INT TERM HUP
  local i unresolved='' outcome code
  # The sessions ran in the foreground and have returned; anything else left in the group is waited for.
  for ((i = 0; i < 1200; i++)); do
    python3 -I -c '
import os, sys
g, me = sys.argv[1], {sys.argv[2], str(os.getpid())}
for p in os.listdir("/proc"):
    if p.isdigit() and p not in me:
        try:
            if open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()[2] == g:
                sys.exit(1)
        except (OSError, IndexError):
            pass
' "$me_pgid" "$$" && break
    sleep 0.1
  done
  [ "$i" -lt 1200 ] || log "option_a: processes of the group remain after 120 s; restoring anyway"
  if [ "$mask_owed" -eq 1 ]; then
    if step mask_restore env "${KENV[@]}" "$WQ" restore >>"$LOG" 2>&1; then mask_owed=0; else unresolved+=" mask"; fi
  fi
  if [ "$quiet_owed" -eq 1 ]; then
    if step quiet_restore env "${KENV[@]}" "$QUIET" restore >>"$LOG" 2>&1; then quiet_owed=0; else unresolved+=" quiet_host"; fi
  fi
  if [ "$keeper_up" -eq 1 ]; then
    if step keeper_finish env ROOT_KEEPER_DIR="$K" "$KEEPER" finish >>"$LOG" 2>&1; then keeper_up=0; else unresolved+=" keeper"; fi
  fi
  if [ -n "$unresolved" ]; then
    outcome=UNRESOLVED; code=3
    log "option_a: UNRESOLVED:$unresolved not verified; no session may start until it is restored (tools/wq_mask.sh restore, tools/quiet_host.sh restore)"
  elif [ -n "$stopped" ]; then outcome=STOPPED; code=1; log "option_a: STOPPED ($stopped); every restoration verified"
  else outcome=ACQUIRED; code=0; log "option_a: both runs acquired and both gates passed; every restoration verified"
  fi
  rec "finished=$(now) outcome=$outcome reason=${stopped:-none}${unresolved:+ unresolved:$unresolved}"
  exit "$code"
}
trap finish EXIT
halt() { [ -n "$stopped" ] || stopped=$1; log "option_a: stop: $stopped"; finish; }
ok() {  # STATUS WHAT: halts unless the step succeeded, no signal came and the record holds
  [ "$1" -eq 0 ] || halt "$2 (status $1)"
  [ -z "$stopped" ] || halt "$stopped"
  [ "$record_failed" -eq 0 ] || halt "the experiment record could not be written"
}

rec "quiet_owed=$(now)"; quiet_owed=1
step quiet_stop env "${KENV[@]}" "$QUIET" stop; ok $? "tools/quiet_host.sh stop failed"
rec "mask_owed=$(now)"; mask_owed=1
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
