#!/usr/bin/env bash
# tools/sudo_refresh.sh, the traced session's sudo refresh, with a stand-in
# sudo (it logs its own pid when it starts and ends, and can take a while or
# fail): the pin and its read-back; stops at random moments of many short
# cycles, between a sleep and a refresh among them, each confirmed and
# leaving none of its processes (each checked by its own pid, from the loop's
# ledger and from the stand-in's log); a stop during a refresh, which is
# waited for; a refresh that outlasts the stop's timeout, reported as
# unresolved and confirmed later; the group's signals not stopping it; the
# session's death ending it; a pin that fails; a refresh that fails; a loop
# killed outright, whose orphan is reported; a state that cannot be read,
# never taken for an end or waited on; and a ledger that cannot be created,
# written or read, is missing or malformed, never taken for an empty one.
set -u

ROOT=$1
rm -rf "$ROOT"
mkdir -p "$ROOT"
# shellcheck source=tools/sudo_refresh.sh
. tools/sudo_refresh.sh

fail() { echo "FAIL: $1"; [ -n "${2:-}" ] && [ -e "$2" ] && cat "$2"; exit 1; }

cat >"$ROOT/fsudo" <<'EOF'
#!/usr/bin/env bash
echo "start $$" >>"$FAKE_SUDO_LOG"
[ -z "${FAKE_SUDO_DELAY:-}" ] || sleep "$FAKE_SUDO_DELAY"
echo "end $$" >>"$FAKE_SUDO_LOG"
exit "${FAKE_SUDO_STATUS:-0}"
EOF
chmod +x "$ROOT/fsudo"
export FAKE_SUDO_LOG="$ROOT/sudo.log"

# The pin: one CPU this process may use, or all of them on a one-CPU machine.
allowed=$(sed -n 's/^Cpus_allowed_list:[[:space:]]*//p' /proc/self/status)
pin=${allowed%%[,-]*}
gone() {  # PID...: none alive
  local q
  for q in "$@"; do ! kill -0 "$q" 2>/dev/null || return 1; done
}
pids() {  # DIR: every pid the loop started, from its ledger and the stand-in's log
  { awk '$1 == "spawned" {print $2}' "$1/children" 2>/dev/null; awk '{print $2}' "$FAKE_SUDO_LOG" 2>/dev/null; } | sort -u
}
fresh() {  # NAME: a new ledger directory and an empty stand-in log
  dir="$ROOT/$1"
  : >"$FAKE_SUDO_LOG"
}

# The pin and its read-back; refreshes happen; the stop is confirmed.
fresh pin
refresh_start "$dir" "$pin" 0.05 "$ROOT/fsudo" || fail "the refresh did not start"
for _ in $(seq 1 100); do [ -e "$dir/cpus" ] && break; sleep 0.02; done
[ "$(cat "$dir/cpus")" = "$pin" ] || fail "the loop read back $(cat "$dir/cpus" 2>&1), not $pin"
[ "$(sed -n 's/^Cpus_allowed_list:[[:space:]]*//p' "/proc/$refresh_pid/status")" = "$pin" ] ||
  fail "the loop is not on CPU $pin"
for _ in $(seq 1 100); do [ "$(grep -c '^end' "$FAKE_SUDO_LOG")" -ge 3 ] && break; sleep 0.02; done
[ "$(grep -c '^end' "$FAKE_SUDO_LOG")" -ge 3 ] || fail "no refreshes" "$FAKE_SUDO_LOG"
loop=$refresh_pid
refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "a plain stop was not confirmed" "$ROOT/stop.err"
[ -z "$refresh_pid" ] || fail "refresh_pid kept after a confirmed stop"
# shellcheck disable=SC2046  # the pids, one per word
gone "$loop" $(pids "$dir") || fail "a process of the refresh outlived its stop" "$dir/children"

# Stops at random moments of short cycles: every one confirmed, nothing left.
for i in $(seq 1 120); do
  fresh "race-$i"
  refresh_start "$dir" "$pin" 0.002 "$ROOT/fsudo" || fail "the refresh did not start ($i)"
  sleep "0.0$((RANDOM % 40))"
  loop=$refresh_pid
  refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "stop $i was not confirmed" "$ROOT/stop.err"
  # shellcheck disable=SC2046
  gone "$loop" $(pids "$dir") || fail "stop $i left a process of the refresh" "$dir/children"
  [ "$(awk '$1 == "spawned"' "$dir/children" 2>/dev/null | wc -l)" -eq "$(awk '$1 == "reaped"' "$dir/children" 2>/dev/null | wc -l)" ] ||
    fail "stop $i: not every process was reaped" "$dir/children"
done

# A stop during a refresh: the refresh is waited for, not killed.
fresh during
FAKE_SUDO_DELAY=0.6 refresh_start "$dir" "$pin" 0.01 "$ROOT/fsudo"
for _ in $(seq 1 200); do grep -q '^start' "$FAKE_SUDO_LOG" && break; sleep 0.01; done
grep -q '^start' "$FAKE_SUDO_LOG" || fail "no refresh started"
loop=$refresh_pid
t0=$(date +%s%N)
refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "the stop during a refresh was not confirmed" "$ROOT/stop.err"
t1=$(date +%s%N)
[ $(((t1 - t0) / 1000000)) -ge 300 ] || fail "the stop did not wait for the refresh ($(((t1 - t0) / 1000000)) ms)"
[ "$(grep -c '^start' "$FAKE_SUDO_LOG")" -eq "$(grep -c '^end' "$FAKE_SUDO_LOG")" ] ||
  fail "a refresh was cut short" "$FAKE_SUDO_LOG"
# shellcheck disable=SC2046
gone "$loop" $(pids "$dir") || fail "the refresh outlived its stop" "$dir/children"

# A refresh that outlasts the stop's timeout: unresolved, reported, the loop
# kept; a later stop confirms it.
fresh slow
FAKE_SUDO_DELAY=3 refresh_start "$dir" "$pin" 0.01 "$ROOT/fsudo"
for _ in $(seq 1 200); do grep -q '^start' "$FAKE_SUDO_LOG" && break; sleep 0.01; done
loop=$refresh_pid
if refresh_stop "$dir" 1 2>"$ROOT/stop.err"; then fail "a stop that timed out was confirmed"; fi
grep -q "the sudo refresh (loop $loop) did not stop within 1 s" "$ROOT/stop.err" || fail "the timeout was not reported" "$ROOT/stop.err"
[ "$refresh_pid" = "$loop" ] || fail "an unconfirmed stop dropped the loop's pid"
refresh_stop "$dir" 10 2>"$ROOT/stop.err" || fail "the later stop was not confirmed" "$ROOT/stop.err"
# shellcheck disable=SC2046
gone "$loop" $(pids "$dir") || fail "the slow refresh outlived its stop" "$dir/children"

# The group's signals do not stop it.
fresh signals
refresh_start "$dir" "$pin" 0.02 "$ROOT/fsudo"
for _ in $(seq 1 100); do [ -e "$dir/cpus" ] && break; sleep 0.02; done
loop=$refresh_pid
for sig in INT TERM HUP QUIT; do kill -"$sig" "$loop"; done
before=$(grep -c '^end' "$FAKE_SUDO_LOG")
sleep 0.3
kill -0 "$loop" 2>/dev/null || fail "a group signal stopped the refresh"
[ "$(grep -c '^end' "$FAKE_SUDO_LOG")" -gt "$before" ] || fail "the refresh stopped refreshing after a group signal"
refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "the stop after group signals was not confirmed" "$ROOT/stop.err"
# shellcheck disable=SC2046
gone "$loop" $(pids "$dir") || fail "the refresh outlived its stop after group signals" "$dir/children"

# The session's death ends it, with at most the refresh or sleep then running.
fresh orphan
mkdir -p "$dir"
sleep 30 &
session=$!
refresh_loop "$dir" "$pin" 0.02 "$session" "$ROOT/fsudo" &
refresh_pid=$!
loop=$refresh_pid
sleep 0.2
kill "$session"
wait "$session" 2>/dev/null
n0=$(awk '$1 == "spawned"' "$dir/children" | wc -l)
for _ in $(seq 1 100); do refresh_ended "$loop" && break; sleep 0.02; done
refresh_ended "$loop" || fail "the refresh outlived its session"
[ "$(awk '$1 == "spawned"' "$dir/children" | wc -l)" -le $((n0 + 1)) ] || fail "the refresh went on after its session died" "$dir/children"
refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "the orphan's end was not confirmed" "$ROOT/stop.err"
# shellcheck disable=SC2046
gone "$loop" $(pids "$dir") || fail "a process outlived the orphaned refresh" "$dir/children"

# A pin that fails: nothing starts, the loop ends (status 1), confirmed.
fresh nopin
refresh_start "$dir" 99999 0.02 "$ROOT/fsudo"
for _ in $(seq 1 100); do [ -e "$dir/cpus" ] && break; sleep 0.02; done
[ "$(cat "$dir/cpus")" = "(taskset failed)" ] || fail "a failed pin read back $(cat "$dir/cpus" 2>&1)"
refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "the unpinned loop's end was not confirmed" "$ROOT/stop.err"
[ -e "$dir/children" ] && [ ! -s "$dir/children" ] && [ ! -s "$FAKE_SUDO_LOG" ] || fail "an unpinned loop started something, or left no empty ledger" "$dir/children"

# A refresh that fails ends the loop (status 2), confirmed.
fresh refail
FAKE_SUDO_STATUS=1 refresh_start "$dir" "$pin" 0.01 "$ROOT/fsudo"
loop=$refresh_pid
for _ in $(seq 1 100); do refresh_ended "$loop" && break; sleep 0.02; done
refresh_ended "$loop" || fail "a failed refresh did not end the loop"
refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "the end after a failed refresh was not confirmed" "$ROOT/stop.err"
[ "$(grep -c '^start' "$FAKE_SUDO_LOG")" -eq 1 ] || fail "the loop refreshed again after a failure" "$FAKE_SUDO_LOG"

# A loop killed outright: its sleep is an orphan, which is reported.
fresh killed
refresh_start "$dir" "$pin" 30 "$ROOT/fsudo"
for _ in $(seq 1 100); do grep -q 'spawned [0-9]* sleep' "$dir/children" 2>/dev/null && break; sleep 0.02; done
orphan=$(awk '$1 == "spawned" {print $2}' "$dir/children")
kill -KILL "$refresh_pid"
if refresh_stop "$dir" 5 2>"$ROOT/stop.err"; then fail "a killed loop's end was confirmed"; fi
grep -q "did not reap every process it started: started $orphan" "$ROOT/stop.err" || fail "the orphan was not reported" "$ROOT/stop.err"
kill -KILL "$orphan" 2>/dev/null  # it ignores TERM, as the loop did
refresh_pid=
refresh_status=

# Stand-ins for what the helper reads: its state and its ledger.
stat_standin=
ledger_denied=
cat() {
  if [ "$1" = "/proc/${loop-}/stat" ]; then
    case $stat_standin in
      denied) return 1 ;;
      empty) return 0 ;;
      garbled) echo "not a stat line"; return 0 ;;
      garbled-zombie) echo "Z not a stat line"; return 0 ;;
      unknown-letter) echo "${loop} (bash) ? 1 2 3"; return 0 ;;
    esac
  fi
  if [ -n "$ledger_denied" ] && [ "$1" = "$ledger_denied" ]; then return 1; fi
  command cat "$@"
}

# The three states, and nothing but a known end is an end.
fresh state
FAKE_SUDO_DELAY=3 refresh_start "$dir" "$pin" 0.01 "$ROOT/fsudo"
for _ in $(seq 1 200); do grep -q '^start' "$FAKE_SUDO_LOG" && break; sleep 0.01; done
loop=$refresh_pid
refresh_ended "$loop"
st=$?
[ "$st" -eq 1 ] || fail "a running loop read as state $st"
for m in denied empty garbled garbled-zombie unknown-letter; do
  stat_standin=$m
  refresh_ended "$loop"
  st=$?
  stat_standin=
  [ "$st" -eq 2 ] || fail "a status that is $m read as state $st, not as unknown"
done

# A state that cannot be read, with a refresh that outlasts the timeout: the
# stop ends within its timeout, while the refresh still runs (it never entered
# the wait), reports the stop unresolved and keeps the loop; a later stop,
# with the state readable again, confirms it.
stat_standin=denied
if refresh_stop "$dir" 1 2>"$ROOT/stop.err"; then fail "a stop whose state could not be read was confirmed"; fi
stat_standin=
[ "$(grep -c '^end' "$FAKE_SUDO_LOG")" -eq 0 ] || fail "the stop waited for the refresh though its state was unknown" "$FAKE_SUDO_LOG"
grep -q "the sudo refresh (loop $loop): its state could not be read within 1 s, so it was not waited for" "$ROOT/stop.err" ||
  fail "the unknown state was not reported" "$ROOT/stop.err"
[ "$refresh_pid" = "$loop" ] || fail "a stop of unknown state dropped the loop's pid"
kill -0 "$loop" 2>/dev/null || fail "the loop was not running after the unresolved stop"
refresh_stop "$dir" 10 2>"$ROOT/stop.err" || fail "the stop once the state was readable was not confirmed" "$ROOT/stop.err"
[ -z "$refresh_pid" ] || fail "refresh_pid kept after the confirmed stop"
# shellcheck disable=SC2046
gone "$loop" $(pids "$dir") || fail "the refresh outlived its stop after an unknown state" "$dir/children"

# A loop that ended by itself (a refresh that failed), its ledger written.
ended() {  # NAME
  fresh "$1"
  FAKE_SUDO_STATUS=1 refresh_start "$dir" "$pin" 0.01 "$ROOT/fsudo"
  loop=$refresh_pid
  for _ in $(seq 1 200); do refresh_ended "$loop" && break; sleep 0.02; done
  refresh_ended "$loop" || fail "the loop for $1 did not end"
}
unconfirmed() {  # WHAT EXPECTED: an unresolved stop, reported, the loop kept
  if refresh_stop "$dir" 5 2>"$ROOT/stop.err"; then fail "$1: the stop was confirmed"; fi
  grep -qF "$2" "$ROOT/stop.err" || fail "$1: not reported as expected ($2)" "$ROOT/stop.err"
  [ "$refresh_pid" = "$loop" ] || fail "$1: the loop's pid was dropped"
}
confirmed() {  # WHAT: a later stop, the evidence back, confirms it
  refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "$1: not confirmed with the evidence back" "$ROOT/stop.err"
  [ -z "$refresh_pid" ] && [ -z "$refresh_status" ] || fail "$1: the loop's state was kept after a confirmed stop"
}

# A missing ledger is not an empty one, and the recovery state is kept.
ended ledger-missing
mv "$dir/children" "$dir/children.kept"
unconfirmed "a missing ledger" "(loop $loop, status 2) ended, but its ledger $dir/children is missing"
mv "$dir/children.kept" "$dir/children"
confirmed "a missing ledger"

# Unreadable: refused by a stand-in, and a directory in its place.
ended ledger-unreadable
ledger_denied=$dir/children
unconfirmed "an unreadable ledger" "its ledger $dir/children could not be read"
ledger_denied=
confirmed "an unreadable ledger"
ended ledger-directory
mv "$dir/children" "$dir/children.kept"
mkdir "$dir/children"
unconfirmed "a directory for a ledger" "its ledger $dir/children could not be read"
rmdir "$dir/children"
mv "$dir/children.kept" "$dir/children"
confirmed "a directory for a ledger"

# Malformed: every line must be one the loop writes, in the loop's order.
n=0
malformed() {  # WHAT CONTENT EXPECTED
  n=$((n + 1))
  ended "ledger-malformed-$n"
  cp "$dir/children" "$dir/children.kept"
  printf '%b' "$2" >"$dir/children"
  unconfirmed "$1" "$3"
  mv "$dir/children.kept" "$dir/children"
  confirmed "$1"
}
malformed "a garbage line" 'garbage row\n' "has a line it cannot read (line 1"
malformed "a line after good ones" 'spawned 12 sleep\nreaped 12 0\nbroken\n' "has a line it cannot read (line 3"
malformed "a pid that is not a number" 'spawned abc sleep\nreaped abc 0\n' "has a line it cannot read (line 1"
malformed "an unknown kind" 'spawned 12 other\nreaped 12 0\n' "has a line it cannot read (line 1"
malformed "a status that is not a number" 'spawned 12 sleep\nreaped 12 zero\n' "has a line it cannot read (line 2"
malformed "a line with an extra field" 'spawned 12 sleep extra\nreaped 12 0\n' "has a line it cannot read (line 1"
malformed "a line cut short" 'spawned 12 sleep\nreaped 12\n' "has a line it cannot read (line 2"
malformed "an unfinished last line" 'spawned 12 sleep\nreaped 12 0' "ends with an unfinished line"
malformed "a reap with no start" 'reaped 12 0\n' "reaps a process it did not start (line 1"
malformed "another process reaped" 'spawned 12 sleep\nreaped 13 0\n' "reaps a process it did not start (line 2"
malformed "a start before the last reap" 'spawned 12 sleep\nspawned 13 refresh\nreaped 13 0\nreaped 12 0\n' "starts a process before reaping 12 (line 2"
malformed "a process never reaped" 'spawned 12 sleep\nreaped 12 0\nspawned 14 refresh\n' "did not reap every process it started: started 14, never reaped"

# A ledger that cannot be created: nothing starts.
fresh ledger-uncreated
mkdir -p "$dir/children"
if refresh_start "$dir" "$pin" 0.01 "$ROOT/fsudo" 2>/dev/null; then fail "a refresh started without its ledger"; fi
[ -z "$refresh_pid" ] || fail "a refresh without a ledger has a pid"
[ ! -s "$FAKE_SUDO_LOG" ] || fail "a refresh without a ledger ran" "$FAKE_SUDO_LOG"

# A ledger line that cannot be written ends the loop (status 3), reported.
fresh ledger-write
refresh_start "$dir" "$pin" 0.5 "$ROOT/fsudo"
loop=$refresh_pid
for _ in $(seq 1 200); do grep -q 'spawned [0-9]* sleep' "$dir/children" 2>/dev/null && break; sleep 0.01; done
grep -q 'spawned [0-9]* sleep' "$dir/children" || fail "the loop started no sleep"
mv "$dir/children" "$dir/children.kept"
mkdir "$dir/children"
for _ in $(seq 1 300); do refresh_ended "$loop" && break; sleep 0.02; done
refresh_ended "$loop" || fail "a ledger that cannot be written did not end the loop"
unconfirmed "a ledger line that was not written" "(loop $loop, status 3) ended, but its ledger $dir/children could not be read"
rmdir "$dir/children"
mv "$dir/children.kept" "$dir/children"
unconfirmed "a ledger missing its last line" "did not reap every process it started"
refresh_pid=
refresh_status=

# A stop request that goes unanswered: it is repeated while the stop waits.
fresh unanswered
refresh_start "$dir" "$pin" 30 "$ROOT/fsudo"
for _ in $(seq 1 200); do grep -q 'spawned [0-9]* sleep' "$dir/children" 2>/dev/null && break; sleep 0.01; done
grep -q 'spawned [0-9]* sleep' "$dir/children" || fail "the loop started no sleep"
loop=$refresh_pid
requests=0
kill() {
  if [ "$1" = -USR1 ]; then
    requests=$((requests + 1))
    [ "$requests" -gt 1 ] || return 0
  fi
  builtin kill "$@"
}
refresh_stop "$dir" 5 2>"$ROOT/stop.err" || fail "a stop whose first request went unanswered was not confirmed" "$ROOT/stop.err"
unset -f kill
[ "$requests" -ge 2 ] || fail "the stop request was not repeated"
[ -z "$refresh_pid" ] || fail "refresh_pid kept after the confirmed stop"
# shellcheck disable=SC2046
gone "$loop" $(pids "$dir") || fail "a process outlived the stop whose first request went unanswered" "$dir/children"

echo "sudo refresh OK"
