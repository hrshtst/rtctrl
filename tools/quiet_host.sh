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
#     stopped if it is active; masking and stopping are separate steps. The
#     indexer and its extractor (localsearch-3, localsearch-extractor-3)
#     must then have exited, within 10 s.
# Every state is read strictly: only a state word systemctl prints (with a
# consistent status for is-active) counts, so a failed or unreadable query
# refuses before any change, or fails a verification.
#
# The record, $XDG_RUNTIME_DIR/rtctrl-quiet-host.state (cleared at reboot,
# like the runtime mask), is written ahead: the states found, and each
# intended change before it is made, every write checked (a failed write
# stops the tool before that change). tools/trace_session.sh copies it into
# its run records as the session's temporary host changes. A second stop
# refuses while it exists. An interrupted stop or restore leaves the record,
# and restore can always follow.
# restore undoes what the record intended, checking each unit's actual
# state: it removes a runtime mask only if stop intended to add one (never
# one that existed before) and it is in place, and starts only a service
# that was active and that stop intended to stop, if it is not running. It
# then verifies the recorded states and only then removes the record; any
# failure keeps it. restore can be repeated. status prints the record and
# the services' states.
#
# Exit: 0 done; 1 a step or a verification failed, or interrupted (the
# record is kept for restore); 2 refused, nothing changed.
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
fail() {
  echo "quiet_host: $*" >&2
  [ -e "$state" ] && echo "  the record of the intended changes is $state; tools/quiet_host.sh restore undoes them" >&2
  exit 1
}
trap 'fail "interrupted"' INT TERM HUP

# state VERB UNIT [--user]: the unit's state word (is-active or is-enabled),
# status 1 if systemctl gave none it is known to print.
state() {
  local verb=$1 unit=$2 out st word
  local -a scope=()
  [ "${3:-}" = --user ] && scope=(--user)
  out=$("$SYSTEMCTL" "${scope[@]}" "$verb" "$unit" 2>/dev/null)
  st=$?
  word=$(head -n 1 <<<"$out")
  case "$verb:$word" in
    is-active:active) [ "$st" -eq 0 ] || return 1 ;;
    is-active:inactive | is-active:failed | is-active:activating | is-active:deactivating | is-active:reloading | \
      is-active:refreshing | is-active:maintenance) [ "$st" -ne 0 ] || [ "$word" != inactive ] || return 1 ;;
    is-enabled:enabled | is-enabled:enabled-runtime | is-enabled:linked | is-enabled:linked-runtime | is-enabled:alias | \
      is-enabled:masked | is-enabled:masked-runtime | is-enabled:static | is-enabled:indirect | is-enabled:disabled | \
      is-enabled:generated | is-enabled:transient | is-enabled:not-found) ;;
    *) return 1 ;;
  esac
  echo "$word"
}
need() {  # VAR VERB UNIT [--user]: the state into VAR, else refuse or fail by $mode
  local __got
  if ! __got=$(state "$2" "$3" "${4:-}"); then
    if [ "$mode" = refuse ]; then refuse "cannot read $3's state ($2)"; else fail "cannot read $3's state ($2)"; fi
  fi
  printf -v "$1" '%s' "$__got"
}

# The record: one key=value per line, rewritten whole and checked.
qh_started=; qh_uu_was=; qh_uu_stop=0; qh_ls_was=; qh_ls_enabled=; qh_ls_mask=0; qh_ls_stop=0
s=  # a state read by need
save() {
  { printf '%s\n' "started=$qh_started" "unattended-upgrades.service_was=$qh_uu_was" \
      "unattended-upgrades.service_stop_intended=$qh_uu_stop" "localsearch-3.service_was=$qh_ls_was" \
      "localsearch-3.service_enabled_was=$qh_ls_enabled" "localsearch-3.service_runtime_mask_intended=$qh_ls_mask" \
      "localsearch-3.service_stop_intended=$qh_ls_stop" >"$state.tmp" && mv -f "$state.tmp" "$state"; } 2>/dev/null
}
load() {
  local k v
  while IFS='=' read -r k v; do
    [[ $v =~ ^[A-Za-z0-9:+.-]*$ ]] || fail "unreadable record line: $k=$v"
    case "$k" in
      started) qh_started=$v ;;
      unattended-upgrades.service_was) qh_uu_was=$v ;;
      unattended-upgrades.service_stop_intended) qh_uu_stop=$v ;;
      localsearch-3.service_was) qh_ls_was=$v ;;
      localsearch-3.service_enabled_was) qh_ls_enabled=$v ;;
      localsearch-3.service_runtime_mask_intended) qh_ls_mask=$v ;;
      localsearch-3.service_stop_intended) qh_ls_stop=$v ;;
      *) fail "unknown record line: $k" ;;
    esac
  done <"$state" || fail "cannot read $state"
  [ -n "$qh_uu_was" ] && [ -n "$qh_ls_was" ] && [ -n "$qh_ls_enabled" ] || fail "incomplete record $state"
}
intend() {  # save the record with an intended change before making it
  save || fail "cannot write $state before $1: not done"
}

[ $# -eq 1 ] || usage
case "$1" in
  stop)
    mode=refuse
    [ ! -e "$state" ] || refuse "a previous stop is not restored ($state): run tools/quiet_host.sh restore first"
    for u in apt-daily.service apt-daily-upgrade.service; do
      need s is-active "$u"
      case "$s" in inactive | failed) ;; *) refuse "$u is $s: an upgrade may be running" ;; esac
    done
    up=$(batch_running "$UPGRADE_NAMES")
    [ -z "$up" ] || refuse "a package operation is running ($up): let it finish"
    need qh_uu_was is-active "$UU"
    need qh_ls_was is-active "$LS" --user
    need qh_ls_enabled is-enabled "$LS" --user
    qh_started=$(date +%Y-%m-%dT%H:%M:%S%z)
    if ! save; then rm -f "$state" "$state.tmp"; refuse "cannot write $state"; fi
    mode=fail
    if [ "$qh_uu_was" = active ]; then
      qh_uu_stop=1; intend "stopping $UU"
      "${SUDO[@]}" "$SYSTEMCTL" stop "$UU" || fail "cannot stop $UU"
    fi
    case "$qh_ls_enabled" in
      masked* | not-found) ;;  # already masked (not by this tool), or no such unit: left as it is
      *) qh_ls_mask=1; intend "masking $LS"
         "$SYSTEMCTL" --user mask --runtime "$LS" >/dev/null || fail "cannot mask $LS for this boot" ;;
    esac
    if [ "$qh_ls_was" = active ]; then
      qh_ls_stop=1; intend "stopping $LS"
      "$SYSTEMCTL" --user stop "$LS" || fail "cannot stop $LS"
    fi
    for ((i = 0; i < WAIT_S * 10; i++)); do
      left=$(batch_running "$INDEXER_NAMES")
      [ -z "$left" ] && break
      sleep 0.1
    done
    [ -z "$left" ] || fail "the indexer is still running ($left) after ${WAIT_S} s"
    need s is-active "$UU"; case "$s" in inactive | failed) ;; *) fail "$UU is $s after the stop" ;; esac
    need s is-active "$LS" --user; case "$s" in inactive | failed) ;; *) fail "$LS is $s after the stop" ;; esac
    if [ "$qh_ls_mask" = 1 ]; then
      need s is-enabled "$LS" --user; [ "$s" = masked-runtime ] || fail "$LS is $s after masking it"
    fi
    echo "quiet_host: stopped (record $state):"
    sed 's/^/  /' "$state"
    echo "  restore with: tools/quiet_host.sh restore"
    ;;
  restore)
    mode=fail
    [ -e "$state" ] || { echo "quiet_host: nothing to restore (no $state)"; exit 0; }
    load
    # Each intent against the unit's actual state: an interrupted stop may
    # have made the change or not.
    if [ "$qh_ls_mask" = 1 ]; then
      need s is-enabled "$LS" --user
      if [ "$s" = masked-runtime ]; then
        "$SYSTEMCTL" --user unmask --runtime "$LS" >/dev/null || fail "cannot remove the runtime mask of $LS"
      fi
      qh_ls_mask=0; save || fail "cannot write $state after removing the runtime mask"
    fi
    if [ "$qh_ls_stop" = 1 ]; then
      need s is-active "$LS" --user
      if [ "$qh_ls_was" = active ] && [ "$s" != active ]; then
        "$SYSTEMCTL" --user start "$LS" || fail "cannot start $LS"
      fi
      qh_ls_stop=0; save || fail "cannot write $state after starting $LS"
    fi
    if [ "$qh_uu_stop" = 1 ]; then
      need s is-active "$UU"
      if [ "$qh_uu_was" = active ] && [ "$s" != active ]; then
        "${SUDO[@]}" "$SYSTEMCTL" start "$UU" || fail "cannot start $UU"
      fi
      qh_uu_stop=0; save || fail "cannot write $state after starting $UU"
    fi
    # Verified against the states recorded before the stop.
    need s is-active "$UU"; [ "$s" = "$qh_uu_was" ] || fail "$UU is $s, not $qh_uu_was"
    need s is-active "$LS" --user; [ "$s" = "$qh_ls_was" ] || fail "$LS is $s, not $qh_ls_was"
    need s is-enabled "$LS" --user; [ "$s" = "$qh_ls_enabled" ] || fail "$LS is $s, not $qh_ls_enabled"
    rm "$state" || fail "cannot remove $state"
    echo "quiet_host: restored: $UU $qh_uu_was, $LS $qh_ls_was ($qh_ls_enabled)"
    ;;
  status)
    if [ -e "$state" ]; then echo "record $state:"; sed 's/^/  /' "$state"; else echo "no record ($state)"; fi
    echo "now: $UU $(state is-active "$UU" || echo unknown); $LS $(state is-active "$LS" --user || echo unknown)" \
      "($(state is-enabled "$LS" --user || echo unknown))"
    ;;
  *) usage ;;
esac
