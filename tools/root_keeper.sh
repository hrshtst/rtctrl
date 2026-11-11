#!/usr/bin/env bash
# The privileged half of the loaded emulator validation's supervisor
# (tools/option_a.sh): a root process, started through sudo while the
# operator's terminal is alive, that performs the experiment's few
# privileged steps on request and owns the mask's restoration if the
# supervisor ends without finishing. With sudo-rs the operator's cached
# credentials are tied to the terminal session, so after a hangup the
# supervisor may no longer run sudo; the keeper does not need it.
#
# usage: sudo tools/root_keeper.sh serve DIR OWNER_PID OWNER_START PGID CPU
#        ROOT_KEEPER_DIR=DIR tools/root_keeper.sh REQUEST...   (the client)
#
# serve checks DIR (a directory, not a link, owned by the invoking user,
# mode 0700, holding the request FIFO "requests" and the directory
# "replies", both the user's), reads the unbound workqueues' masks (the
# effective and the requested one equal, CPU in them) and fixes the only two
# values it will ever write: that mask (the original) and that mask less CPU
# (the target). It then detaches (a new session, its input and output on
# DIR/keeper.log), ignores HUP, INT, TERM and QUIT, writes DIR/keeper.ready
# and serves requests, one line each, "ID VERB [VALUE]":
#   mask VALUE   writes VALUE to the mask if it is the original or the target
#   uu-stop      systemctl stop unattended-upgrades.service
#   uu-start     systemctl start unattended-upgrades.service
#   status       the masks now
#   finish       writes the original back unless it is in effect, verifies
#                it and ends
# and answers each in DIR/replies/ID ("status=N", then any detail); any
# other request is refused (99). Every action goes to DIR/keeper.log.
#
# If the owner (OWNER_PID with its /proc start time OWNER_START) ends
# without a finish, the keeper waits until no process of the experiment's
# process group PGID remains (the session's own cleanup, at most 30
# minutes), then writes the original back unless it is in effect, verifies
# it and ends. It restores nothing else (tools/quiet_host.sh's record and
# tools/wq_mask.sh's record stay open for their tools to finish). A keeper
# killed with SIGKILL, or a crash or power loss, restores nothing; a reboot
# resets the kernel's mask.
#
# The client maps exactly these invocations, so that the supervisor can pass
# this script as the tools' sudo (WQ_MASK_SUDO, QUIET_HOST_SUDO):
#   tee <the keeper's mask file>, the value on its input  -> mask VALUE
#   systemctl stop|start unattended-upgrades.service      -> uu-stop|uu-start
#   status | finish                                       -> status | finish
# Anything else is refused (99); a keeper that is not running, 98; no
# answer within ROOT_KEEPER_WAIT seconds (default 60), 97. The client
# ignores HUP, so that a request made during the cleanup after a hangup is
# answered.
#
# For tests only, with ROOT_KEEPER_TEST=1 and never as root: the keeper runs
# unprivileged, KEEPER_SYSFS replaces /sys/devices/virtual/workqueue,
# KEEPER_WRITE a program that receives each value written (in place of the
# write to the mask file) and KEEPER_SYSTEMCTL systemctl.
set -u
SYSFS=/sys/devices/virtual/workqueue
WRITE=
SYSTEMCTL=systemctl
if [ "${ROOT_KEEPER_TEST:-0}" = 1 ]; then
  [ "$(id -u)" -ne 0 ] || { echo "root_keeper: the test mode never runs as root" >&2; exit 2; }
  SYSFS=${KEEPER_SYSFS:-$SYSFS}; WRITE=${KEEPER_WRITE:-}; SYSTEMCTL=${KEEPER_SYSTEMCTL:-systemctl}
fi
UU=unattended-upgrades.service
now() { date +%Y-%m-%dT%H:%M:%S.%N%z; }

read_mask() {
  local v
  v=$(cat "$SYSFS/$1" 2>/dev/null) || return 1
  [[ $v =~ ^[0-9a-f]+(,[0-9a-f]+)*$ ]] || return 1
  printf '%s\n' "$v"
}
mask_clear() {  # MASK CPU: the mask less the CPU in MASK's layout; status 1 if the CPU is not in it or nothing is left
  python3 -I -c '
import sys
a, cpu = sys.argv[1], int(sys.argv[2])
groups = a.split(",")
v = 0
for g in groups:
    v = (v << 32) | int(g, 16)
if not v >> cpu & 1 or v == 1 << cpu:
    sys.exit(1)
v &= ~(1 << cpu)
print(",".join(reversed([format((v >> (32 * i)) & 0xFFFFFFFF, "0%dx" % len(g)) for i, g in enumerate(reversed(groups))])))
' "$1" "$2"
}
start_time() {  # PID: its start time (field 22 of /proc/PID/stat), or nothing
  sed 's/.*) //' "/proc/$1/stat" 2>/dev/null | cut -d ' ' -f 20
}

serve() {
  local dir=$1 owner=$2 owner_start=$3 pgid=$4 cpu=$5 uid
  uid=${SUDO_UID:-$(id -u)}
  [ "${ROOT_KEEPER_TEST:-0}" = 1 ] || [ "$(id -u)" -eq 0 ] || { echo "root_keeper: serve runs as root (through sudo)" >&2; exit 2; }
  [[ $owner =~ ^[0-9]+$ && $owner_start =~ ^[0-9]+$ && $pgid =~ ^[0-9]+$ && $cpu =~ ^[0-9]+$ ]] || { echo "root_keeper: bad arguments" >&2; exit 2; }
  [ -d "$dir" ] && [ ! -L "$dir" ] && [ "$(stat -c '%u %a' "$dir")" = "$uid 700" ] ||
    { echo "root_keeper: $dir is not the invoking user's private directory (mode 0700)" >&2; exit 2; }
  [ -p "$dir/requests" ] && [ ! -L "$dir/requests" ] && [ "$(stat -c %u "$dir/requests")" = "$uid" ] &&
    [ -d "$dir/replies" ] && [ ! -L "$dir/replies" ] && [ "$(stat -c %u "$dir/replies")" = "$uid" ] ||
    { echo "root_keeper: $dir lacks the user's request FIFO or replies directory" >&2; exit 2; }
  [ "$(start_time "$owner")" = "$owner_start" ] || { echo "root_keeper: the owner $owner is not running" >&2; exit 2; }
  local orig rq target
  orig=$(read_mask cpumask) && rq=$(read_mask cpumask_requested) && [ "$orig" = "$rq" ] ||
    { echo "root_keeper: the masks cannot be read, or the requested one differs" >&2; exit 2; }
  target=$(mask_clear "$orig" "$cpu") || { echo "root_keeper: CPU $cpu cannot be cleared from $orig" >&2; exit 2; }
  if [ -z "${ROOT_KEEPER_DETACHED:-}" ]; then
    # Detach: a new session, nothing on the caller's terminal; the caller returns at once.
    ROOT_KEEPER_DETACHED=1 setsid -f "$(readlink -f "$0")" serve "$dir" "$owner" "$owner_start" "$pgid" "$cpu" </dev/null >>"$dir/keeper.log" 2>&1
    return $?
  fi
  trap '' HUP INT TERM QUIT
  log() { echo "[$(now)] $*"; }
  write() {  # VALUE: the original or the target only; the write's status
    case "$1" in "$orig" | "$target") ;; *) return 99 ;; esac
    if [ -n "$WRITE" ]; then "$WRITE" "$1"; else printf '%s\n' "$1" >"$SYSFS/cpumask"; fi
  }
  in_effect() {  # VALUE: status 0 if both masks read back as it
    local c r
    c=$(read_mask cpumask) && r=$(read_mask cpumask_requested) && [ "$c" = "$1" ] && [ "$r" = "$1" ]
  }
  restore_original() {  # writes the original back unless it is in effect; status 0 once it is
    in_effect "$orig" && return 0
    write "$orig"; local st=$?
    log "wrote the original $orig: status $st"
    in_effect "$orig"
  }
  reply() {  # ID STATUS [DETAIL]
    printf 'status=%s\n%s\n' "$2" "${3:-}" >"$dir/replies/.$1.part" && chown "$uid" "$dir/replies/.$1.part" 2>/dev/null
    mv -f "$dir/replies/.$1.part" "$dir/replies/$1"
  }
  exec 3<>"$dir/requests" || { log "cannot open the request FIFO"; exit 1; }
  printf 'pid=%s\noriginal=%s\ntarget=%s\n' "$$" "$orig" "$target" >"$dir/keeper.ready.part" && mv -f "$dir/keeper.ready.part" "$dir/keeper.ready"
  log "keeper $$ for owner $owner (group $pgid): original $orig, target $target"
  local id verb value st
  while :; do
    if IFS=' ' read -r -t 0.5 id verb value <&3; then
      if ! [[ $id =~ ^[A-Za-z0-9._-]{1,64}$ ]]; then log "refused a request with a bad id"; continue; fi
      case "$verb" in
        mask) write "$value"; st=$?; log "request $id: mask $value: status $st"; reply "$id" "$st" ;;
        uu-stop | uu-start)
          "$SYSTEMCTL" "${verb#uu-}" "$UU" >/dev/null 2>&1; st=$?
          log "request $id: systemctl ${verb#uu-} $UU: status $st"; reply "$id" "$st" ;;
        status) reply "$id" 0 "cpumask $(read_mask cpumask || echo unreadable) requested $(read_mask cpumask_requested || echo unreadable)" ;;
        finish)
          if restore_original; then st=0; else st=1; fi
          log "request $id: finish: the original is $([ "$st" -eq 0 ] && echo in effect || echo NOT in effect); ending"
          reply "$id" "$st"; exit "$st" ;;
        *) log "request $id: refused: $verb"; reply "$id" 99 ;;
      esac
      continue
    fi
    if [ "$(start_time "$owner")" != "$owner_start" ]; then
      log "the owner $owner ended without a finish: waiting for its process group $pgid to end"
      local i
      for ((i = 0; i < 3600; i++)); do
        python3 -I -c '
import os, sys
g = sys.argv[1]
for p in os.listdir("/proc"):
    if p.isdigit():
        try:
            if open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()[2] == g:
                sys.exit(1)
        except (OSError, IndexError):
            pass
' "$pgid" && break
        sleep 0.5
      done
      [ "$i" -lt 3600 ] || log "the process group $pgid still has processes after 30 minutes"
      if restore_original; then log "the original $orig is in effect; ending"; exit 0; fi
      log "the original $orig is NOT in effect; ending (unresolved)"; exit 1
    fi
  done
}

client() {
  trap '' HUP
  local dir=${ROOT_KEEPER_DIR:-} verb value='' id t pid
  [ -n "$dir" ] || { echo "root_keeper: no ROOT_KEEPER_DIR" >&2; exit 98; }
  case "$*" in
    "tee $SYSFS/cpumask") verb=mask; IFS= read -r value || exit 99; [[ $value =~ ^[0-9a-f]+(,[0-9a-f]+)*$ ]] || exit 99 ;;
    "systemctl stop $UU") verb=uu-stop ;;
    "systemctl start $UU") verb=uu-start ;;
    status | finish) verb=$1 ;;
    *) echo "root_keeper: refused: $*" >&2; exit 99 ;;
  esac
  pid=$(sed -n 's/^pid=//p' "$dir/keeper.ready" 2>/dev/null)
  [ -n "$pid" ] && [ -e "/proc/$pid" ] || { echo "root_keeper: no keeper running for $dir" >&2; exit 98; }
  id="$$.$(date +%s%N)"
  timeout 10 bash -c 'printf "%s\n" "$2" >"$1"' _ "$dir/requests" "$id $verb $value" || { echo "root_keeper: cannot send the request" >&2; exit 98; }
  for ((t = 0; t < ${ROOT_KEEPER_WAIT:-60} * 20; t++)); do
    if [ -e "$dir/replies/$id" ]; then
      local st
      st=$(sed -n '1s/^status=//p' "$dir/replies/$id")
      sed -n '2,$p' "$dir/replies/$id" | sed '/^$/d'
      rm -f "$dir/replies/$id"
      [[ $st =~ ^[0-9]+$ ]] || exit 97
      exit "$st"
    fi
    sleep 0.05
  done
  echo "root_keeper: no answer to $verb" >&2
  exit 97
}

case "${1:-}" in
  serve) [ $# -eq 6 ] || { echo "usage: $0 serve DIR OWNER_PID OWNER_START PGID CPU" >&2; exit 2; }; shift; serve "$@" ;;
  *) client "$@" ;;
esac
