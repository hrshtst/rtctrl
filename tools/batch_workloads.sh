# shellcheck shell=bash
# Known unrelated batch workloads, for tools/trace_session.sh (which refuses
# a session beside one, and never stops a process) and tools/quiet_host.sh
# (the operator's reversible stop of the resident services among them).
# Sourced, not run. This is a check for known workloads, not a guarantee
# against all background activity.
#
# A listed name (the kernel's process name, truncated to 15 characters)
# counts while a process of that name exists, unless that process is
# positively identified as a known resident helper that does no work: its
# NUL-separated arguments must match an exemption exactly. A process whose
# arguments cannot be read while it still exists (a zombie, a refused read)
# counts. The only exemption is unattended-upgrades' shutdown helper,
# `/usr/bin/python3 /usr/share/unattended-upgrades/unattended-upgrade-shutdown
# --wait-for-signal`, which only holds shutdown while an upgrade runs (the
# upgrade itself, `/usr/bin/unattended-upgrade`, has the same process name
# and counts, as do apt, apt-get and dpkg).
#
# For tests, TRACE_SESSION_BATCH_LIST replaces the list and
# TRACE_SESSION_BATCH_EXEMPT the exemptions; QUIET_HOST_STATE replaces the
# path of tools/quiet_host.sh's record of its temporary host changes.

BATCH_LIST=${TRACE_SESSION_BATCH_LIST-exiftool,updatedb,updatedb.plocat,apt,apt-get,dpkg,unattended-upgr,cc1,cc1plus,ninja,make,rustc,cargo,localsearch-3,localsearch-ext,tracker-extract,tracker-miner-f,baloo_file,baloo_file_extr}
# Exemptions, separated by ';': name|argv[0] (an extended regular
# expression)|the arguments after it (separated by single spaces, compared
# one by one and all of them).
BATCH_EXEMPT=${TRACE_SESSION_BATCH_EXEMPT-'unattended-upgr|^/usr/bin/python3$|/usr/share/unattended-upgrades/unattended-upgrade-shutdown --wait-for-signal'}
QUIET_HOST_STATE=${QUIET_HOST_STATE:-${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/rtctrl-quiet-host.state}

# batch_exempt PID NAME: status 0 when the process is positively identified
# as an exemption of that name.
batch_exempt() {
  local pid=$1 name=$2 e ename re args i ok
  local -a argv=() want=() entries=()
  mapfile -d '' -t argv <"/proc/$pid/cmdline" 2>/dev/null || return 1
  [ ${#argv[@]} -gt 0 ] || return 1
  IFS=';' read -r -a entries <<<"$BATCH_EXEMPT"
  for e in "${entries[@]}"; do
    IFS='|' read -r ename re args <<<"$e"
    [ -n "$ename" ] && [ "$ename" = "$name" ] || continue
    [[ ${argv[0]} =~ $re ]] || continue
    read -r -a want <<<"$args"
    [ ${#argv[@]} -eq $((${#want[@]} + 1)) ] || continue
    ok=1
    for ((i = 0; i < ${#want[@]}; i++)); do
      [ "${argv[i + 1]}" = "${want[i]}" ] || { ok=0; break; }
    done
    [ "$ok" -eq 1 ] && return 0
  done
  return 1
}

# batch_running [LIST]: the listed names (comma-separated; default
# BATCH_LIST) with a counted process, space-separated.
batch_running() {
  local n pid counted
  local -a names=() found=()
  IFS=, read -r -a names <<<"${1-$BATCH_LIST}"
  for n in "${names[@]}"; do
    [ -n "$n" ] || continue
    counted=0
    for pid in $(pgrep -x -- "${n:0:15}"); do
      batch_exempt "$pid" "${n:0:15}" && continue
      [ -e "/proc/$pid" ] || continue  # it has exited since
      counted=1
      break
    done
    [ "$counted" -eq 1 ] && found+=("$n")
  done
  echo "${found[*]}"
}

# batch_hint NAMES: what the operator can do about them.
batch_hint() {
  case " $1 " in
    *" unattended-upgr "* | *" apt "* | *" apt-get "* | *" dpkg "*)
      echo -n "; a package operation is running: let it finish" ;;
  esac
  case " $1 " in
    *" localsearch-3 "* | *" localsearch-ext "*)
      echo -n "; the indexer: tools/quiet_host.sh stop stops it reversibly (tools/quiet_host.sh restore afterwards)" ;;
  esac
}
