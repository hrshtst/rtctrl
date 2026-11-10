#!/usr/bin/env bash
# tools/sudo_refresh.sh, the traced session's sudo refresh, with a stand-in
# sudo (it logs its own pid when it starts and ends, and can take a while or
# fail): the pin and its read-back; stops at random moments of many short
# cycles, between a sleep and a refresh among them, each confirmed and
# leaving none of its processes (each checked by its own pid, from the loop's
# ledger and from the stand-in's log); a stop during a refresh, which is
# waited for; a refresh that outlasts the stop's timeout, reported as
# unresolved and confirmed later; the group's signals not stopping it; the
# session's death ending it; a pin that fails; a refresh that fails; and a
# loop killed outright, whose orphan is reported.
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
[ ! -s "$dir/children" ] && [ ! -s "$FAKE_SUDO_LOG" ] || fail "an unpinned loop started something" "$dir/children"

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

echo "sudo refresh OK"
