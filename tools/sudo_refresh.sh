# shellcheck shell=bash
# tools/sudo_refresh.sh: the traced session's sudo refresh, sourced by
# tools/trace_session.sh and by its test (tests/integration/sudo_refresh_test.sh).
#
# The refresh keeps sudo's timestamp fresh for the whole session, its
# cleanup included, off the excluded CPUs, and its end is confirmed:
#
# refresh_start DIR CPUS PERIOD_S SUDO...
#   Starts the loop in the background (its pid in refresh_pid). The loop
#   pins itself to CPUS before anything else, so every process it starts
#   inherits that placement, and writes the CPUs it then reads back to
#   DIR/cpus; when the pin fails it writes "(taskset failed)" and ends
#   (status 1) without starting anything. It then alternates a sleep of
#   PERIOD_S and a refresh (SUDO -n -v) while the session ($$) lives,
#   recording each process it starts and reaps in DIR/children ("spawned
#   PID KIND", "reaped PID STATUS"); a refresh that fails ends it (status 2).
#   The group's signals (INT, TERM, HUP, QUIT) do not stop it, so the
#   refresh lasts through the session's cleanup; refresh_stop's USR1 does.
#   Its own messages (the shell's notices of the sleeps it kills, taskset's)
#   go to DIR/stderr.
#
# refresh_stop DIR TIMEOUT_S
#   Stops the loop and confirms, within TIMEOUT_S, that it ended by itself
#   (status 0, 1 or 2) and reaped every process it started. Status 1, with
#   refresh_pid kept and the ledger DIR/children left in place, when that
#   cannot be confirmed; the reason goes to stderr.

refresh_pid=

# The loop (in the background). On USR1 it starts nothing more: a sleep it
# is waiting for is killed, a refresh in progress is waited for (sudo runs
# as root and is not signalled).
refresh_loop() {  # DIR CPUS PERIOD_S SESSION_PID SUDO...
  local dir=$1 cpus=$2 period=$3 session=$4 child='' kind='' stop=0 got st
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
      echo "spawned $child $kind" >>"$dir/children"
      # A stop that came between the check above and the start.
      if [ "$stop" -ne 0 ] && [ "$kind" = sleep ]; then kill -KILL "$child" 2>/dev/null; fi
      while :; do  # until its status is collected (a USR1 interrupts the wait)
        got=''
        wait -p got "$child"
        st=$?
        [ "${got-}" = "$child" ] && break
        [ "$st" -gt 128 ] || { st=127; break; }  # not a child: never expected
      done
      echo "reaped $child $st" >>"$dir/children"
      child=''
      if [ "$kind" = refresh ] && [ "$st" -ne 0 ]; then exit 2; fi
    done
  done
}

refresh_start() {  # DIR CPUS PERIOD_S SUDO...
  local dir=$1
  mkdir -p "$dir" || return 1
  refresh_loop "$dir" "$2" "$3" "$$" "${@:4}" 2>>"$dir/stderr" &
  refresh_pid=$!
}

refresh_ended() {  # PID: gone or a zombie
  local s
  s=$(cat "/proc/$1/stat" 2>/dev/null) || return 0
  [ -n "$s" ] || return 0
  s=${s##*) }
  [ "${s%% *}" = Z ]
}

refresh_stop() {  # DIR TIMEOUT_S
  local dir=$1 timeout=$2 i st got spawned reaped
  [ -n "$refresh_pid" ] || return 0
  kill -USR1 "$refresh_pid" 2>/dev/null
  for ((i = 0; i < timeout * 20; i++)); do
    refresh_ended "$refresh_pid" && break
    sleep 0.05
  done
  if ! refresh_ended "$refresh_pid"; then
    echo "the sudo refresh (loop $refresh_pid) did not stop within $timeout s: it is left running, its end not confirmed (its processes: $dir/children)" >&2
    return 1
  fi
  got=''
  wait -p got "$refresh_pid"
  st=$?
  if [ "${got-}" != "$refresh_pid" ]; then
    echo "the sudo refresh (loop $refresh_pid) ended, but its status could not be collected (its processes: $dir/children)" >&2
    return 1
  fi
  spawned=$(awk '$1 == "spawned" {print $2}' "$dir/children" 2>/dev/null | sort)
  reaped=$(awk '$1 == "reaped" {print $2}' "$dir/children" 2>/dev/null | sort)
  if [ "$spawned" != "$reaped" ]; then
    echo "the sudo refresh (loop $refresh_pid, status $st) did not reap every process it started: started $(tr '\n' ' ' <<<"$spawned"), reaped $(tr '\n' ' ' <<<"$reaped") ($dir/children)" >&2
    return 1
  fi
  case $st in
    0|1|2) ;;
    *) echo "the sudo refresh (loop $refresh_pid) ended with status $st, not by itself ($dir/children)" >&2
       return 1 ;;
  esac
  refresh_pid=
  return 0
}
