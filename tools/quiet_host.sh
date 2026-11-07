#!/usr/bin/env bash
# The operator's reversible quieting of the host's resident services before
# a traced session (docs/hardware/control-rate.md); tools/trace_session.sh
# refuses to start beside them and never stops a process itself.
#
# usage: tools/quiet_host.sh stop|restore|status
#
# stop records each service's state, then:
#   - unattended-upgrades.service (system, through sudo) is stopped if it is
#     active, but only once no upgrade is running: apt-daily.service and
#     apt-daily-upgrade.service inactive, and no unattended-upgr, apt,
#     apt-get or dpkg process other than the positively identified shutdown
#     helper (tools/batch_workloads.sh). Otherwise it refuses and changes
#     nothing: the helper asks a running upgrade to stop when it is stopped.
#   - localsearch-3.service (user) is masked for this boot (--runtime)
#     unless it is masked already, so that nothing starts it again, and is
#     stopped if it is active. Masking and stopping are separate steps, and
#     each is recorded only once done. The indexer and its extractor
#     (localsearch-3, localsearch-extractor-3) must then have exited, within
#     10 s.
# The record, $XDG_RUNTIME_DIR/rtctrl-quiet-host.state (cleared at reboot,
# like the runtime mask), is rewritten after each step, and
# tools/trace_session.sh copies it into its run records as the session's
# temporary host changes. A second stop refuses while it exists.
# restore undoes only what stop did: it removes the runtime mask only if
# stop added it, and starts only the services that stop stopped; then it
# removes the record. status prints the record and the services' states.
#
# Exit: 0 done; 1 a step failed (the record is kept, so restore can follow
# a failed stop); 2 refused, nothing changed.
#
# For tests, QUIET_HOST_SYSTEMCTL and QUIET_HOST_SUDO replace systemctl and
# sudo (QUIET_HOST_SUDO=none runs directly), QUIET_HOST_UPGRADE_NAMES and
# QUIET_HOST_INDEXER_NAMES the process names checked, QUIET_HOST_WAIT the
# wait for the indexer, and QUIET_HOST_STATE the record's path.
set -u
here=$(dirname "$0")
# shellcheck source=tools/batch_workloads.sh
. "$here/batch_workloads.sh"
SYSTEMCTL=${QUIET_HOST_SYSTEMCTL:-systemctl}
case "${QUIET_HOST_SUDO:-sudo}" in none) SUDO=() ;; *) SUDO=("${QUIET_HOST_SUDO:-sudo}") ;; esac
UPGRADE_NAMES=${QUIET_HOST_UPGRADE_NAMES:-unattended-upgr,apt,apt-get,dpkg}
INDEXER_NAMES=${QUIET_HOST_INDEXER_NAMES:-localsearch-3,localsearch-ext}
WAIT_S=${QUIET_HOST_WAIT:-10}
UU=unattended-upgrades.service
LS=localsearch-3.service
state=$QUIET_HOST_STATE

usage() { echo "usage: $0 stop|restore|status" >&2; exit 2; }
refuse() { echo "quiet_host: $*: nothing changed" >&2; exit 2; }
fail() { echo "quiet_host: $*" >&2; echo "  the record of what was changed is $state; tools/quiet_host.sh restore undoes it" >&2; exit 1; }
unit_state() { "$SYSTEMCTL" "$@" 2>/dev/null | head -n 1; }  # is-active/is-enabled print even on a nonzero status
ls_enabled() {  # localsearch-3.service's unit file state; absent when the host has none
  local s
  s=$(unit_state --user is-enabled "$LS")
  case "$s" in ""|not-found) echo absent ;; *) echo "$s" ;; esac
}

# The record: one key=value per line, rewritten whole.
qh_started=; qh_uu_was=; qh_uu_stopped=0; qh_ls_was=; qh_ls_enabled=; qh_ls_masked=0; qh_ls_stopped=0
save() {
  printf '%s\n' "started=$qh_started" "unattended-upgrades.service_was=$qh_uu_was" \
    "unattended-upgrades.service_stopped=$qh_uu_stopped" "localsearch-3.service_was=$qh_ls_was" \
    "localsearch-3.service_enabled=$qh_ls_enabled" "localsearch-3.service_masked_runtime=$qh_ls_masked" \
    "localsearch-3.service_stopped=$qh_ls_stopped" >"$state.tmp" && mv "$state.tmp" "$state"
}
load() {
  local k v
  while IFS='=' read -r k v; do
    [[ $v =~ ^[A-Za-z0-9:+.-]*$ ]] || fail "unreadable record line: $k=$v"
    case "$k" in
      started) qh_started=$v ;;
      unattended-upgrades.service_was) qh_uu_was=$v ;;
      unattended-upgrades.service_stopped) qh_uu_stopped=$v ;;
      localsearch-3.service_was) qh_ls_was=$v ;;
      localsearch-3.service_enabled) qh_ls_enabled=$v ;;
      localsearch-3.service_masked_runtime) qh_ls_masked=$v ;;
      localsearch-3.service_stopped) qh_ls_stopped=$v ;;
      *) fail "unknown record line: $k" ;;
    esac
  done <"$state"
}

[ $# -eq 1 ] || usage
case "$1" in
  stop)
    [ ! -e "$state" ] || refuse "a previous stop is not restored ($state): run tools/quiet_host.sh restore first"
    for u in apt-daily.service apt-daily-upgrade.service; do
      s=$(unit_state is-active "$u")
      case "$s" in active|activating|deactivating|reloading|refreshing) refuse "$u is $s: an upgrade may be running" ;; esac
    done
    up=$(batch_running "$UPGRADE_NAMES")
    [ -z "$up" ] || refuse "a package operation is running ($up): let it finish"
    qh_started=$(date +%Y-%m-%dT%H:%M:%S%z)
    qh_uu_was=$(unit_state is-active "$UU")
    qh_ls_was=$(unit_state --user is-active "$LS")
    qh_ls_enabled=$(ls_enabled)
    save || refuse "cannot write $state"
    if [ "$qh_uu_was" = active ]; then
      "${SUDO[@]}" "$SYSTEMCTL" stop "$UU" || fail "cannot stop $UU"
      qh_uu_stopped=1; save
    fi
    case "$qh_ls_enabled" in
      masked* | absent) ;;  # already masked (not by this tool), or no such unit: left as it is
      *) "$SYSTEMCTL" --user mask --runtime "$LS" >/dev/null || fail "cannot mask $LS for this boot"
         qh_ls_masked=1; save ;;
    esac
    if [ "$qh_ls_was" = active ]; then
      "$SYSTEMCTL" --user stop "$LS" || fail "cannot stop $LS"
      qh_ls_stopped=1; save
    fi
    for ((i = 0; i < WAIT_S * 10; i++)); do
      left=$(batch_running "$INDEXER_NAMES")
      [ -z "$left" ] && break
      sleep 0.1
    done
    [ -z "$left" ] || fail "the indexer is still running ($left) after ${WAIT_S} s"
    s=$(unit_state is-active "$UU"); case "$s" in active|activating) fail "$UU is $s after the stop" ;; esac
    s=$(unit_state --user is-active "$LS"); case "$s" in active|activating) fail "$LS is $s after the stop" ;; esac
    echo "quiet_host: stopped (record $state):"
    sed 's/^/  /' "$state"
    echo "  restore with: tools/quiet_host.sh restore"
    ;;
  restore)
    [ -e "$state" ] || { echo "quiet_host: nothing to restore (no $state)"; exit 0; }
    load
    bad=0
    if [ "$qh_ls_masked" = 1 ]; then
      if "$SYSTEMCTL" --user unmask --runtime "$LS" >/dev/null; then qh_ls_masked=0; save
      else echo "quiet_host: cannot remove the runtime mask of $LS" >&2; bad=1; fi
    fi
    if [ "$qh_ls_stopped" = 1 ] && [ "$bad" -eq 0 ]; then
      if "$SYSTEMCTL" --user start "$LS"; then qh_ls_stopped=0; save
      else echo "quiet_host: cannot start $LS" >&2; bad=1; fi
    fi
    if [ "$qh_uu_stopped" = 1 ]; then
      if "${SUDO[@]}" "$SYSTEMCTL" start "$UU"; then qh_uu_stopped=0; save
      else echo "quiet_host: cannot start $UU" >&2; bad=1; fi
    fi
    [ "$bad" -eq 0 ] || fail "restore incomplete"
    # Verified against the states recorded before the stop.
    [ "$(unit_state is-active "$UU")" = "$qh_uu_was" ] || fail "$UU is $(unit_state is-active "$UU"), not $qh_uu_was"
    [ "$(unit_state --user is-active "$LS")" = "$qh_ls_was" ] || fail "$LS is $(unit_state --user is-active "$LS"), not $qh_ls_was"
    [ "$(ls_enabled)" = "$qh_ls_enabled" ] || fail "$LS is $(ls_enabled), not $qh_ls_enabled"
    rm -f "$state"
    echo "quiet_host: restored: $UU $qh_uu_was, $LS $qh_ls_was ($qh_ls_enabled)"
    ;;
  status)
    if [ -e "$state" ]; then echo "record $state:"; sed 's/^/  /' "$state"; else echo "no record ($state)"; fi
    echo "now: $UU $(unit_state is-active "$UU"); $LS $(unit_state --user is-active "$LS") ($(ls_enabled))"
    ;;
  *) usage ;;
esac
