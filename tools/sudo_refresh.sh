# shellcheck shell=bash
# tools/sudo_refresh.sh: the traced session's sudo refresh, sourced by
# tools/trace_session.sh and by its test (tests/integration/sudo_refresh_test.sh).
#
# The refresh keeps sudo's timestamp fresh for the whole session, its
# cleanup included, off the excluded CPUs, and its end is confirmed:
#
# refresh_start DIR CPUS PERIOD_S SUDO...
#   Creates the empty ledger DIR/children (status 1, nothing started, when
#   it cannot), so that a loop that starts nothing still leaves a valid one,
#   then starts the loop in the background (its pid in refresh_pid). The loop
#   pins itself to CPUS before anything else, so every process it starts
#   inherits that placement, and writes the CPUs it then reads back to
#   DIR/cpus; when the pin fails it writes "(taskset failed)" and ends
#   (status 1) without starting anything. It then alternates a sleep of
#   PERIOD_S and a refresh (SUDO -n -v) while the session ($$) lives,
#   recording each process it starts and reaps in DIR/children ("spawned
#   PID KIND", "reaped PID STATUS"); a refresh that fails ends it (status 2),
#   and so does a ledger line it cannot write, once its child is reaped
#   (status 3). The group's signals (INT, TERM, HUP, QUIT) do not stop it, so the
#   refresh lasts through the session's cleanup; refresh_stop's USR1 does.
#   Its own messages (the shell's notices of the sleeps it kills, taskset's)
#   go to DIR/stderr.
#
# refresh_ended PID
#   The process's state as /proc shows it: status 0 when it has ended (no
#   such process, or a zombie), 1 when it is running, 2 when that is not
#   known (its status could not be read or understood). Only 0 is an end.
#
# refresh_stop DIR TIMEOUT_S
#   Stops the loop and confirms, within TIMEOUT_S, that it ended by itself
#   (status 0, 1 or 2) and that its ledger shows it reaped every process it
#   started. It waits for the loop only once the loop is known to have
#   ended, so an unknown state never blocks it. Status 1, with refresh_pid
#   kept and the ledger left in place, when any of that cannot be confirmed:
#   the loop still running, its state unknown, its status not collected, its
#   ledger missing, unreadable or malformed, or a process not reaped; the
#   reason goes to stderr. A later call goes on from what is already known.

refresh_pid=
refresh_status=  # the loop's collected exit status, while refresh_pid is unresolved

# The loop (in the background). On USR1 it starts nothing more: a sleep it
# is waiting for is killed, a refresh in progress is waited for (sudo runs
# as root and is not signalled).
refresh_loop() {  # DIR CPUS PERIOD_S SESSION_PID SUDO...
  local dir=$1 cpus=$2 period=$3 session=$4 child='' kind='' stop=0 ledger=1 got st
  shift 4
  trap '' INT TERM HUP QUIT
  trap 'stop=1; if [ "$kind" = sleep ] && [ -n "$child" ]; then kill -KILL "$child" 2>/dev/null; fi' USR1
  if ! taskset -cp "$cpus" "$BASHPID" >/dev/null; then
    echo "(taskset failed)" >"$dir/cpus.part"
    mv "$dir/cpus.part" "$dir/cpus"
    exit 1
  fi
  sed -n 's/^Cpus_allowed_list:[[:space:]]*//p' "/proc/$BASHPID/status" >"$dir/cpus.part"
  mv "$dir/cpus.part" "$dir/cpus"
  while :; do
    for kind in sleep refresh; do
      if [ "$stop" -ne 0 ] || ! kill -0 "$session" 2>/dev/null; then exit 0; fi
      if [ "$kind" = sleep ]; then
        sleep "$period" &
      else
        "$@" -n -v </dev/null >/dev/null 2>&1 &
      fi
      child=$!
      echo "spawned $child $kind" >>"$dir/children" || ledger=0
      # A stop that came between the check above and the start.
      if [ "$stop" -ne 0 ] && [ "$kind" = sleep ]; then kill -KILL "$child" 2>/dev/null; fi
      while :; do  # until its status is collected (a USR1 interrupts the wait)
        got=''
        wait -p got "$child"
        st=$?
        [ "${got-}" = "$child" ] && break
        [ "$st" -gt 128 ] || { st=127; break; }  # not a child: never expected
      done
      echo "reaped $child $st" >>"$dir/children" || ledger=0
      child=''
      [ "$ledger" -eq 1 ] || exit 3
      if [ "$kind" = refresh ] && [ "$st" -ne 0 ]; then exit 2; fi
    done
  done
}

refresh_start() {  # DIR CPUS PERIOD_S SUDO...
  local dir=$1
  mkdir -p "$dir" || return 1
  : >>"$dir/children" || return 1
  refresh_status=
  refresh_loop "$dir" "$2" "$3" "$$" "${@:4}" 2>>"$dir/stderr" &
  refresh_pid=$!
}

refresh_ended() {  # PID: 0 ended, 1 running, 2 not known
  local s
  if s=$(cat "/proc/$1/stat" 2>/dev/null) && [[ $s == *") "* ]]; then
    s=${s##*) }
    case ${s%% *} in
      Z|X|x) return 0 ;;
      R|S|D|T|t|W|K|P|I) return 1 ;;
      *) return 2 ;;
    esac
  fi
  # Not read, or not a status line: an end only when the kernel, which shows
  # this shell, shows no such process.
  if [ -e "/proc/$$" ] && [ ! -e "/proc/$1" ]; then return 0; fi
  return 2
}

# The ledger as evidence, line by line: the loop runs one process at a time,
# so each "spawned PID KIND" is followed by its "reaped PID STATUS". Status 0
# with refresh_unreaped naming a process started and never reaped, if any;
# status 1 with the reason in refresh_problem when it is no evidence.
refresh_ledger() {  # FILE
  local text line word pid value extra n=0
  refresh_unreaped='' refresh_problem=''
  if ! text=$(cat "$1" 2>/dev/null && printf x); then
    if [ -e "$1" ]; then refresh_problem='could not be read'; else refresh_problem='is missing'; fi
    return 1
  fi
  text=${text%x}
  [ -n "$text" ] || return 0
  if [ "${text: -1}" != $'\n' ]; then
    refresh_problem='ends with an unfinished line'
    return 1
  fi
  while IFS= read -r line; do
    n=$((n + 1))
    read -r word pid value extra <<<"$line"
    if [ -n "${extra-}" ] || ! [[ ${pid-} =~ ^[0-9]+$ ]]; then
      refresh_problem="has a line it cannot read (line $n: $line)"
      return 1
    fi
    case $word in
      spawned)
        if ! [[ ${value-} =~ ^(sleep|refresh)$ ]]; then
          refresh_problem="has a line it cannot read (line $n: $line)"
          return 1
        fi
        if [ -n "$refresh_unreaped" ]; then
          refresh_problem="starts a process before reaping $refresh_unreaped (line $n: $line)"
          return 1
        fi
        refresh_unreaped=$pid
        ;;
      reaped)
        if ! [[ ${value-} =~ ^[0-9]+$ ]]; then
          refresh_problem="has a line it cannot read (line $n: $line)"
          return 1
        fi
        if [ "$pid" != "$refresh_unreaped" ]; then
          refresh_problem="reaps a process it did not start (line $n: $line)"
          return 1
        fi
        refresh_unreaped=''
        ;;
      *)
        refresh_problem="has a line it cannot read (line $n: $line)"
        return 1
        ;;
    esac
  done <<<"${text%$'\n'}"
  return 0
}

refresh_stop() {  # DIR TIMEOUT_S
  local dir=$1 timeout=$2 i state got
  [ -n "$refresh_pid" ] || return 0
  if [ -z "$refresh_status" ]; then
    kill -USR1 "$refresh_pid" 2>/dev/null
    for ((i = 0; i < timeout * 20; i++)); do
      refresh_ended "$refresh_pid" && break
      sleep 0.05
    done
    refresh_ended "$refresh_pid"
    state=$?
    if [ "$state" -eq 1 ]; then
      echo "the sudo refresh (loop $refresh_pid) did not stop within $timeout s: it is left running, its end not confirmed (its processes: $dir/children)" >&2
      return 1
    fi
    if [ "$state" -ne 0 ]; then
      echo "the sudo refresh (loop $refresh_pid): its state could not be read within $timeout s, so it was not waited for: its end is not confirmed (its processes: $dir/children)" >&2
      return 1
    fi
    got=''
    wait -p got "$refresh_pid"
    refresh_status=$?
    if [ "${got-}" != "$refresh_pid" ]; then
      refresh_status=
      echo "the sudo refresh (loop $refresh_pid) ended, but its status could not be collected (its processes: $dir/children)" >&2
      return 1
    fi
  fi
  if ! refresh_ledger "$dir/children"; then
    echo "the sudo refresh (loop $refresh_pid, status $refresh_status) ended, but its ledger $dir/children $refresh_problem: what it started and reaped is not known" >&2
    return 1
  fi
  if [ -n "$refresh_unreaped" ]; then
    echo "the sudo refresh (loop $refresh_pid, status $refresh_status) did not reap every process it started: started $refresh_unreaped, never reaped ($dir/children)" >&2
    return 1
  fi
  case $refresh_status in
    0|1|2) ;;
    3) echo "the sudo refresh (loop $refresh_pid) ended because a line of its ledger could not be written ($dir/children)" >&2
       return 1 ;;
    *) echo "the sudo refresh (loop $refresh_pid) ended with status $refresh_status, not by itself ($dir/children)" >&2
       return 1 ;;
  esac
  refresh_pid=
  refresh_status=
  return 0
}
