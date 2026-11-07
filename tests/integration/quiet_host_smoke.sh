#!/usr/bin/env bash
# Smoke test of tools/batch_workloads.sh and tools/quiet_host.sh, with
# stand-in processes under test-only names (never the host's own daemons)
# and a stand-in systemctl that keeps each unit's state in files.
#
# The library: a positively identified shutdown helper does not count, but
# an upgrader beside it does; a helper with other arguments and a zombie
# (unreadable arguments) count; the defaults list localsearch-ext and exempt
# exactly the real helper. quiet_host.sh: stop records the states, stops the
# active services and masks the indexer for this boot, restore undoes only
# that; already stopped or masked services are left alone; a running
# upgrade (its units, or a process) refuses the stop with nothing changed; an
# indexer still running fails the stop with the record kept for restore; a
# second stop refuses; a failed restore step keeps the record. Interrupted
# around each change (after it, or before it with its intent recorded) and
# with the record unwritable before a change, restore returns every unit to
# its recorded state; an unreadable state refuses the stop before any
# change, or fails a verification with the record kept.
#
# usage: quiet_host_smoke.sh <scratch dir>
set -u
ROOT=$1
rm -rf "$ROOT"
mkdir -p "$ROOT/sd"
fail() { echo "FAILED: $1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }
pids=()
cleanup() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT

# Stand-in processes: python3 <script> [args], each setting its own name.
cat >"$ROOT/named" <<'EOF'
import os, sys, time
open("/proc/self/comm", "w").write(os.environ["NAME"])
time.sleep(float(os.environ.get("LIFE", "60")))
EOF
cp "$ROOT/named" "$ROOT/uu-helper"
cp "$ROOT/named" "$ROOT/uu-run"
start() {  # name life script args...: starts it, waits until that process has its name
  local name=$1 life=$2 pid
  shift 2
  NAME=$name LIFE=$life python3 "$@" &
  pid=$!
  pids+=("$pid")
  for _ in $(seq 1 50); do [ "$(cat "/proc/$pid/comm" 2>/dev/null)" = "$name" ] && return 0; sleep 0.1; done
  fail "the stand-in $name ($pid) did not start"
}
stop_all() { for p in "${pids[@]}"; do kill "$p" 2>/dev/null; wait "$p" 2>/dev/null; done; pids=(); }

export TRACE_SESSION_BATCH_EXEMPT="rtctrl-uu|^python3\$|$ROOT/uu-helper --wait-for-signal"
# shellcheck source=tools/batch_workloads.sh
. tools/batch_workloads.sh

start rtctrl-uu 60 "$ROOT/uu-helper" --wait-for-signal
[ -z "$(batch_running rtctrl-uu)" ] || fail "the positively identified helper counted"
start rtctrl-uu 60 "$ROOT/uu-run"
[ "$(batch_running rtctrl-uu)" = rtctrl-uu ] || fail "an upgrader beside the helper did not count"
stop_all
start rtctrl-uu 60 "$ROOT/uu-helper" --other
[ "$(batch_running rtctrl-uu)" = rtctrl-uu ] || fail "a helper with other arguments did not count"
stop_all
# A zombie keeps its name but has no arguments to read.
python3 -c 'import os, time
pid = os.fork()
if pid == 0:
    open("/proc/self/comm", "w").write("rtctrl-uu")
    os._exit(0)
time.sleep(60)' &
zparent=$!
pids+=("$zparent")
for _ in $(seq 1 50); do z=$(pgrep -P "$zparent" -x rtctrl-uu) && break; sleep 0.1; done
[ -n "$z" ] && grep -q '^State:.*Z' "/proc/$z/status" || fail "no zombie stand-in"
[ "$(batch_running rtctrl-uu)" = rtctrl-uu ] || fail "a zombie (unreadable arguments) did not count"
stop_all
[ -n "$(batch_hint "localsearch-3")" ] && [ -z "$(batch_hint "exiftool")" ] || fail "the hints"
# The defaults.
defaults=$(env -u TRACE_SESSION_BATCH_LIST -u TRACE_SESSION_BATCH_EXEMPT bash -c '. tools/batch_workloads.sh; echo "$BATCH_LIST"; echo "$BATCH_EXEMPT"')
grep -q '^\(.*,\)\?localsearch-ext\(,.*\)\?$' <<<"$(head -n 1 <<<"$defaults")" || fail "localsearch-ext is not listed"
[ "$(tail -n 1 <<<"$defaults")" = 'unattended-upgr|^/usr/bin/python3$|/usr/share/unattended-upgrades/unattended-upgrade-shutdown --wait-for-signal' ] ||
  fail "the default exemption is not exactly the shutdown helper: $(tail -n 1 <<<"$defaults")"
# On a host running the real helper, it is exempt under the defaults.
real=$(pgrep -f '^/usr/bin/python3 /usr/share/unattended-upgrades/unattended-upgrade-shutdown --wait-for-signal$' | head -n 1)
if [ -n "$real" ]; then
  env -u TRACE_SESSION_BATCH_EXEMPT bash -c ". tools/batch_workloads.sh; batch_exempt $real unattended-upgr" ||
    fail "the host's real shutdown helper (pid $real) is not exempt"
fi

# The stand-in systemctl: <scope>.<unit>.active and .enabled files; mask
# --runtime keeps the earlier state to restore. Faults, each named
# <verb>:<unit>: FAKE_SYSTEMCTL_FAIL=<verb> fails that verb; UNKNOWN prints
# no state and exits 1 (from its UNKNOWN_FROM-th query on, default the
# first); KILL makes the change and then sends SIGTERM to its caller,
# KILL_BEFORE sends it without the change; RO makes the record's directory
# read-only after the change.
cat >"$ROOT/systemctl" <<'EOF'
#!/usr/bin/env bash
echo "$*" >>"$SD/log"
scope=system
[ "$1" = --user ] && { scope=user; shift; }
verb=$1; shift
[ "$verb" = "${FAKE_SYSTEMCTL_FAIL:-}" ] && { echo "stand-in: $verb fails" >&2; exit 1; }
[ "${1:-}" = --runtime ] && shift
f="$SD/$scope.$1"
if [ "$verb:$1" = "${FAKE_UNKNOWN:-}" ]; then
  n=$(($(cat "$SD/unknown.count" 2>/dev/null || echo 0) + 1)); echo "$n" >"$SD/unknown.count"
  [ "$n" -ge "${FAKE_UNKNOWN_FROM:-1}" ] && { echo "Failed to connect to bus" >&2; exit 1; }
fi
[ "$verb:$1" = "${FAKE_KILL_BEFORE:-}" ] && { kill -TERM "$PPID"; exit 1; }
trap 'if [ "$verb:$1" = "${FAKE_KILL:-}" ]; then kill -TERM "$PPID"; fi
      if [ "$verb:$1" = "${FAKE_RO:-}" ]; then chmod a-w "$(dirname "$QUIET_HOST_STATE")"; fi' EXIT
case "$verb" in
  is-active) s=$(cat "$f.active" 2>/dev/null || echo inactive); echo "$s"; [ "$s" = active ] ;;
  is-enabled) [ -e "$f.enabled" ] || { echo "Failed to get unit file state" >&2; exit 1; }
              s=$(cat "$f.enabled"); echo "$s"; [ "$s" = enabled ] ;;
  stop) echo inactive >"$f.active" ;;
  start) case "$(cat "$f.enabled" 2>/dev/null)" in masked*) echo "Unit is masked" >&2; exit 1 ;; esac
         echo active >"$f.active" ;;
  mask) cp "$f.enabled" "$f.before"; echo masked-runtime >"$f.enabled" ;;
  unmask) mv "$f.before" "$f.enabled" ;;
esac
EOF
chmod +x "$ROOT/systemctl"
mkdir -p "$ROOT/rec"
export SD="$ROOT/sd" QUIET_HOST_SYSTEMCTL="$ROOT/systemctl" QUIET_HOST_SUDO=none QUIET_HOST_STATE="$ROOT/rec/qh.state"
export QUIET_HOST_UPGRADE_NAMES=rtctrl-uu,rtctrl-apt QUIET_HOST_INDEXER_NAMES=rtctrl-ix QUIET_HOST_WAIT=1
units() {  # uu active, ls active, ls enabled
  echo "$1" >"$SD/system.unattended-upgrades.service.active"
  echo "$2" >"$SD/user.localsearch-3.service.active"
  echo "$3" >"$SD/user.localsearch-3.service.enabled"
  rm -f "$SD/system.apt-daily.service.active" "$SD/system.apt-daily-upgrade.service.active" "$SD/log" "$SD/unknown.count"
}
now() { echo "$(cat "$SD/system.unattended-upgrades.service.active") $(cat "$SD/user.localsearch-3.service.active") $(cat "$SD/user.localsearch-3.service.enabled")"; }
QH=tools/quiet_host.sh

# Both running: stopped and masked, then restored exactly; the extractor
# exits on its own within the wait.
units active active enabled
start rtctrl-ix 0.5 "$ROOT/named"
"$QH" stop >"$ROOT/stop1.log" 2>&1 || fail "stop with both running" "$ROOT/stop1.log"
[ "$(now)" = "inactive inactive masked-runtime" ] || fail "after the stop: $(now)" "$SD/log"
grep -qx 'unattended-upgrades.service_was=active' "$QUIET_HOST_STATE" &&
  grep -qx 'localsearch-3.service_runtime_mask_intended=1' "$QUIET_HOST_STATE" &&
  grep -qx 'localsearch-3.service_stop_intended=1' "$QUIET_HOST_STATE" || fail "the record" "$QUIET_HOST_STATE"
stop_all
"$QH" stop >"$ROOT/stop2.log" 2>&1
[ $? -eq 2 ] && grep -q 'a previous stop is not restored' "$ROOT/stop2.log" || fail "a second stop" "$ROOT/stop2.log"
"$QH" status | grep -q '^record ' || fail "status without the record"
"$QH" restore >"$ROOT/restore1.log" 2>&1 || fail "restore" "$ROOT/restore1.log"
[ "$(now)" = "active active enabled" ] && [ ! -e "$QUIET_HOST_STATE" ] || fail "after the restore: $(now)" "$ROOT/restore1.log"
"$QH" restore | grep -q 'nothing to restore' || fail "a restore without a record"

# Already stopped and masked (not by the tool): left as they are, both ways.
units inactive inactive masked-runtime
"$QH" stop >"$ROOT/stop3.log" 2>&1 || fail "stop with nothing to do" "$ROOT/stop3.log"
"$QH" restore >"$ROOT/restore3.log" 2>&1 || fail "restore with nothing to do" "$ROOT/restore3.log"
if grep -qE '^(stop|start|--user (stop|start|mask|unmask))' "$SD/log"; then fail "a service left alone was changed" "$SD/log"; fi
[ "$(now)" = "inactive inactive masked-runtime" ] || fail "the untouched services: $(now)"

# A running upgrade refuses, with nothing changed: its unit, or a process
# (but the identified helper alone does not).
units active active enabled
echo active >"$SD/system.apt-daily-upgrade.service.active"
"$QH" stop >"$ROOT/stop4.log" 2>&1
[ $? -eq 2 ] && grep -q 'apt-daily-upgrade.service is active: an upgrade may be running: nothing changed' "$ROOT/stop4.log" ||
  fail "a running upgrade unit" "$ROOT/stop4.log"
units active active enabled
start rtctrl-uu 60 "$ROOT/uu-helper" --wait-for-signal
start rtctrl-uu 60 "$ROOT/uu-run"
"$QH" stop >"$ROOT/stop5.log" 2>&1
[ $? -eq 2 ] && grep -q 'a package operation is running (rtctrl-uu)' "$ROOT/stop5.log" || fail "a running upgrader" "$ROOT/stop5.log"
[ "$(now)" = "active active enabled" ] && [ ! -e "$QUIET_HOST_STATE" ] || fail "a refused stop changed something: $(now)"
if grep -qE '^(stop|--user (stop|mask))' "$SD/log"; then fail "a refused stop changed a unit" "$SD/log"; fi
stop_all

# The indexer still running after the wait: the stop fails, keeping the
# record of what it did, and restore undoes it.
units active active enabled
start rtctrl-ix 60 "$ROOT/named"
"$QH" stop >"$ROOT/stop6.log" 2>&1
[ $? -eq 1 ] && grep -q 'the indexer is still running (rtctrl-ix)' "$ROOT/stop6.log" && [ -e "$QUIET_HOST_STATE" ] ||
  fail "an indexer still running" "$ROOT/stop6.log"
stop_all
"$QH" restore >"$ROOT/restore6.log" 2>&1 || fail "restore after a failed stop" "$ROOT/restore6.log"
[ "$(now)" = "active active enabled" ] || fail "after restoring a failed stop: $(now)"

# A failed restore step keeps the record.
units active active enabled
"$QH" stop >/dev/null 2>&1 || fail "stop before a failed restore"
FAKE_SYSTEMCTL_FAIL=start "$QH" restore >"$ROOT/restore7.log" 2>&1
[ $? -eq 1 ] && [ -e "$QUIET_HOST_STATE" ] || fail "a failed restore step" "$ROOT/restore7.log"
"$QH" restore >/dev/null 2>&1 || fail "the restore retried"
[ "$(now)" = "active active enabled" ] || fail "after the retried restore: $(now)"
# Interrupted after each change, or before one with its intent recorded:
# the stop fails with the record kept, and restore returns every unit.
for k in "KILL stop:unattended-upgrades.service" "KILL mask:localsearch-3.service" "KILL stop:localsearch-3.service" \
         "KILL_BEFORE stop:unattended-upgrades.service" "KILL_BEFORE mask:localsearch-3.service"; do
  read -r kind at <<<"$k"
  units active active enabled
  env "FAKE_$kind=$at" "$QH" stop >"$ROOT/int.log" 2>&1
  [ $? -eq 1 ] && grep -q 'interrupted' "$ROOT/int.log" && [ -e "$QUIET_HOST_STATE" ] ||
    fail "$kind at $at: the interrupted stop" "$ROOT/int.log"
  "$QH" restore >"$ROOT/int-restore.log" 2>&1 || fail "$kind at $at: restore" "$ROOT/int-restore.log"
  [ "$(now)" = "active active enabled" ] && [ ! -e "$QUIET_HOST_STATE" ] ||
    fail "$kind at $at: after the restore: $(now)" "$ROOT/int-restore.log"
done
# The record unwritable: before the first change, the stop refuses with
# nothing changed; before a later one, that change is not made and restore
# undoes the earlier.
units active active enabled
chmod a-w "$ROOT/rec"
"$QH" stop >"$ROOT/ro1.log" 2>&1
st=$?
chmod u+w "$ROOT/rec"
[ "$st" -eq 2 ] && grep -q 'cannot write' "$ROOT/ro1.log" && [ "$(now)" = "active active enabled" ] ||
  fail "an unwritable record before any change (status $st): $(now)" "$ROOT/ro1.log"
units active active enabled
FAKE_RO=stop:unattended-upgrades.service "$QH" stop >"$ROOT/ro2.log" 2>&1
st=$?
chmod u+w "$ROOT/rec"
[ "$st" -eq 1 ] && grep -q "cannot write $QUIET_HOST_STATE before masking" "$ROOT/ro2.log" &&
  [ "$(now)" = "inactive active enabled" ] || fail "an unwritable record before masking (status $st): $(now)" "$ROOT/ro2.log"
"$QH" restore >/dev/null 2>&1 || fail "restore after an unwritable record"
[ "$(now)" = "active active enabled" ] || fail "after restoring an unwritable record: $(now)"
# An unreadable state: before the changes, the stop refuses (the
# reviewer's case: apt-daily.service active, its query empty); in the
# verification after them, the stop fails and the record stays, and so
# does a restore's verification until the state reads again.
for at in is-active:apt-daily.service is-enabled:localsearch-3.service; do
  units active active enabled
  echo active >"$SD/system.apt-daily.service.active"
  [ "$at" = is-enabled:localsearch-3.service ] && rm -f "$SD/system.apt-daily.service.active"
  FAKE_UNKNOWN=$at "$QH" stop >"$ROOT/unk.log" 2>&1
  [ $? -eq 2 ] && grep -q "cannot read ${at#*:}'s state" "$ROOT/unk.log" && [ "$(now)" = "active active enabled" ] &&
    [ ! -e "$QUIET_HOST_STATE" ] || fail "an unreadable $at before the changes: $(now)" "$ROOT/unk.log"
done
units active active enabled
FAKE_UNKNOWN=is-active:unattended-upgrades.service FAKE_UNKNOWN_FROM=2 "$QH" stop >"$ROOT/unk2.log" 2>&1
[ $? -eq 1 ] && grep -q "cannot read unattended-upgrades.service's state" "$ROOT/unk2.log" && [ -e "$QUIET_HOST_STATE" ] ||
  fail "an unreadable state in the stop's verification" "$ROOT/unk2.log"
rm -f "$SD/unknown.count"
FAKE_UNKNOWN=is-active:unattended-upgrades.service "$QH" restore >"$ROOT/unk3.log" 2>&1
[ $? -eq 1 ] && [ -e "$QUIET_HOST_STATE" ] || fail "an unreadable state in a restore" "$ROOT/unk3.log"
"$QH" restore >/dev/null 2>&1 || fail "restore once the state reads again"
[ "$(now)" = "active active enabled" ] && [ ! -e "$QUIET_HOST_STATE" ] || fail "after the last restore: $(now)"
echo "quiet host smoke OK"
