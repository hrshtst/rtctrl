#!/usr/bin/env bash
# The privileged half of the loaded emulator validation's supervisor
# (tools/option_a.sh): a root process, started through sudo while the
# operator's terminal is alive, that performs the experiment's few
# privileged steps on request and owns the mask's restoration if the
# supervisor ends without finishing. With sudo-rs the operator's cached
# credentials are tied to the terminal session, so after a hangup the
# supervisor may no longer run sudo; the keeper does not need it.
#
# usage: sudo tools/root_keeper.sh serve OWNER_PID OWNER_START PGID CPU
#        ROOT_KEEPER_DIR=DIR tools/root_keeper.sh REQUEST...   (the client)
#
# serve reads the unbound workqueues' masks (the effective and the requested
# one equal, CPU in them) and fixes the only two values it will ever write:
# that mask (the original) and that mask less CPU (the target). It then
# creates its own directory, /run/rtctrl-keeper-OWNER_PID-OWNER_START,
# exclusively (it refuses if the name exists): a root-owned directory (mode
# 0755) in root-owned /run, so that no other user, the invoking one included,
# can create, replace or redirect anything in it. In it: the request FIFO
# "requests" (the invoking user's, mode 0600: the only thing the user
# writes), the directory "replies", keeper.log and keeper.ready. Every file
# the keeper writes there is created exclusively, without following a link,
# relative to a descriptor of its directory, then renamed into place; it
# changes the owner of nothing but the FIFO it has just created. It then
# detaches (a new session, its input and output on keeper.log), ignores HUP,
# INT, TERM and QUIT, writes keeper.ready and serves requests, one line each,
# "ID VERB [VALUE]":
#   mask VALUE   writes VALUE to the mask if it is the original or the target
#   uu-stop      systemctl stop unattended-upgrades.service
#   uu-start     systemctl start unattended-upgrades.service
#   status       the masks now
#   finish       writes the original back unless it is in effect, verifies
#                it and ends
# and answers each in replies/ID ("status=N", then any detail); any other
# request is refused (99). Every action goes to keeper.log. When it ends it
# removes the FIFO; the directory, with its replies, log and ready file,
# stays until a reboot (tmpfs), for the supervisor to copy.
#
# The owner (OWNER_PID with its /proc start time OWNER_START) has ended only
# when its /proc entry no longer exists or holds another start time; one that
# exists but cannot be read is unknown, and the keeper goes on serving. If the
# owner ends without a finish, the keeper waits for a confirmed end of the
# experiment's process group PGID: no process of the group left, a zombie
# counting as ended, and no /proc entry that exists but cannot be read (an
# unknown entry is not an ended one). Only then does it write the original
# back (unless it is in effect), verify it and end. If that end is not
# confirmed within 30 minutes it ends WITHOUT restoring, the mask unresolved,
# its log naming what remained. It restores nothing else
# (tools/quiet_host.sh's and tools/wq_mask.sh's records stay open). A keeper
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
# unprivileged, KEEPER_RUNDIR replaces /run, KEEPER_SYSFS
# /sys/devices/virtual/workqueue, KEEPER_WRITE a program that receives each
# value written (in place of the write to the mask file), KEEPER_SYSTEMCTL
# systemctl and KEEPER_GROUP_WAIT the 30 minutes' wait in seconds.
set -u
RUNDIR=/run
SYSFS=/sys/devices/virtual/workqueue
WRITE=
SYSTEMCTL=systemctl
GROUP_POLLS=3600  # 30 minutes of 0.5 s polls
if [ "${ROOT_KEEPER_TEST:-0}" = 1 ]; then
  [ "$(id -u)" -ne 0 ] || { echo "root_keeper: the test mode never runs as root" >&2; exit 2; }
  RUNDIR=${KEEPER_RUNDIR:-$RUNDIR}; SYSFS=${KEEPER_SYSFS:-$SYSFS}; WRITE=${KEEPER_WRITE:-}; SYSTEMCTL=${KEEPER_SYSTEMCTL:-systemctl}
  GROUP_POLLS=$(( ${KEEPER_GROUP_WAIT:-1800} * 2 ))
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
# owner_state PID START: "alive", "ended" (no /proc entry, or another start time) or "unknown" (it cannot be read).
owner_state() {
  local s
  if ! s=$(cat "/proc/$1/stat" 2>/dev/null); then
    if [ -e "/proc/$1" ]; then echo unknown; else echo ended; fi
    return
  fi
  s=$(printf '%s\n' "${s##*) }" | cut -d ' ' -f 20)
  if ! [[ $s =~ ^[0-9]+$ ]]; then echo unknown
  elif [ "$s" = "$2" ]; then echo alive
  else echo ended
  fi
}
# group_state PGID: "<members> <unknown>" of the process group (a zombie has ended; an entry that exists but cannot be
# read or understood is unknown), then one line per member or unknown entry, for the log.
group_state() {
  python3 -I -c '
import os, sys
g, me = sys.argv[1], str(os.getpid())
members, unknown = [], []
for p in os.listdir("/proc"):
    if not p.isdigit() or p == me:
        continue
    try:
        s = open(f"/proc/{p}/stat").read()
    except OSError:
        if os.path.exists(f"/proc/{p}"):
            unknown.append(p)
        continue
    try:
        comm = s[s.index("(") + 1:s.rindex(")")]
        f = s[s.rindex(")") + 2:].split()
        state, pgrp = f[0], f[2]
    except (ValueError, IndexError):
        unknown.append(p)
        continue
    if pgrp == g and state not in ("Z", "X"):
        members.append(f"{p} {comm} {state}")
print(len(members), len(unknown))
for m in members:
    print("  member", m)
for u in unknown:
    print("  unknown", u)
' "$1"
}
# put DIR NAME CONTENT: NAME in DIR, created exclusively without following a link (relative to a descriptor of DIR,
# itself opened without following one), then renamed into place (a rename replaces a name, it follows nothing).
put() {
  python3 -I -c '
import os, sys
d, name, data = sys.argv[1], sys.argv[2], sys.argv[3]
dfd = os.open(d, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
try:
    tmp = "." + name + ".part"
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644, dir_fd=dfd)
    try:
        os.write(fd, data.encode())
    finally:
        os.close(fd)
    os.rename(tmp, name, src_dir_fd=dfd, dst_dir_fd=dfd)
finally:
    os.close(dfd)
' "$1" "$2" "$3"
}

serve() {
  local owner=$1 owner_start=$2 pgid=$3 cpu=$4 uid dir orig rq target
  uid=${SUDO_UID:-$(id -u)}
  [ "${ROOT_KEEPER_TEST:-0}" = 1 ] || [ "$(id -u)" -eq 0 ] || { echo "root_keeper: serve runs as root (through sudo)" >&2; exit 2; }
  [[ $owner =~ ^[0-9]+$ && $owner_start =~ ^[0-9]+$ && $pgid =~ ^[0-9]+$ && $cpu =~ ^[0-9]+$ && $uid =~ ^[0-9]+$ ]] ||
    { echo "root_keeper: bad arguments" >&2; exit 2; }
  [ "$(owner_state "$owner" "$owner_start")" = alive ] || { echo "root_keeper: the owner $owner is not running" >&2; exit 2; }
  orig=$(read_mask cpumask) && rq=$(read_mask cpumask_requested) && [ "$orig" = "$rq" ] ||
    { echo "root_keeper: the masks cannot be read, or the requested one differs" >&2; exit 2; }
  target=$(mask_clear "$orig" "$cpu") || { echo "root_keeper: CPU $cpu cannot be cleared from $orig" >&2; exit 2; }
  dir=$RUNDIR/rtctrl-keeper-$owner-$owner_start
  mkdir -m 0755 "$dir" 2>/dev/null || { echo "root_keeper: $dir exists or cannot be created" >&2; exit 2; }
  mkdir -m 0755 "$dir/replies" && mkfifo -m 0600 "$dir/requests" && put "$dir" keeper.log "" ||
    { echo "root_keeper: cannot lay out $dir" >&2; exit 2; }
  if [ "$(id -u)" -eq 0 ]; then
    chown "$uid" "$dir/requests" || { echo "root_keeper: cannot give the request FIFO to user $uid" >&2; exit 2; }
  fi
  # Detach: a new session, nothing on the caller's terminal; the caller returns at once.
  ROOT_KEEPER_DETACHED=1 setsid -f "$(readlink -f "$0")" __run "$dir" "$owner" "$owner_start" "$pgid" "$orig" "$target" \
    </dev/null >>"$dir/keeper.log" 2>&1
}

run() {
  local dir=$1 owner=$2 owner_start=$3 pgid=$4 orig=$5 target=$6
  [ "${ROOT_KEEPER_DETACHED:-}" = 1 ] || { echo "root_keeper: __run is the detached keeper's own" >&2; exit 2; }
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
    put "$dir/replies" "$1" "$(printf 'status=%s\n%s\n' "$2" "${3:-}")" || log "request $1: the reply could not be written"
  }
  ending() {  # the FIFO goes; the replies, the log and the ready file stay (a client may still read its reply)
    exec 3<&-
    rm -f "$dir/requests"
  }
  exec 3<>"$dir/requests" || { log "cannot open the request FIFO"; exit 1; }
  put "$dir" keeper.ready "$(printf 'pid=%s\noriginal=%s\ntarget=%s\n' "$$" "$orig" "$target")" || { log "cannot write keeper.ready"; ending; exit 1; }
  log "keeper $$ for owner $owner (group $pgid): original $orig, target $target"
  local id verb value st told_unknown=0 i g=''
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
          reply "$id" "$st"; ending; exit "$st" ;;
        *) log "request $id: refused: $verb"; reply "$id" 99 ;;
      esac
      continue
    fi
    case "$(owner_state "$owner" "$owner_start")" in
      alive) told_unknown=0; continue ;;
      unknown) [ "$told_unknown" -eq 1 ] || log "the owner $owner cannot be read: not taken as ended; serving on"; told_unknown=1; continue ;;
    esac
    log "the owner $owner ended without a finish: waiting for a confirmed end of its process group $pgid"
    for ((i = 0; i < GROUP_POLLS; i++)); do
      g=$(group_state "$pgid") && [ "$(head -n 1 <<<"$g")" = "0 0" ] && break
      sleep 0.5
    done
    if [ "$i" -ge "$GROUP_POLLS" ]; then
      log "UNRESOLVED: the end of process group $pgid is not confirmed after 30 minutes; the mask is NOT restored:"
      printf '%s\n' "${g:-the group could not be read}"
      ending; exit 1
    fi
    if restore_original; then log "the group has ended; the original $orig is in effect; ending"; ending; exit 0; fi
    log "the group has ended; the original $orig is NOT in effect; ending (unresolved)"; ending; exit 1
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
      [[ $st =~ ^[0-9]+$ ]] || exit 97
      exit "$st"
    fi
    sleep 0.05
  done
  echo "root_keeper: no answer to $verb" >&2
  exit 97
}

case "${1:-}" in
  serve) [ $# -eq 5 ] || { echo "usage: $0 serve OWNER_PID OWNER_START PGID CPU" >&2; exit 2; }; shift; serve "$@" ;;
  __run) [ $# -eq 7 ] || exit 2; shift; run "$@" ;;
  *) client "$@" ;;
esac
