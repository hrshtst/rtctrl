#!/usr/bin/env bash
# The reversible removal of one CPU from the kernel's global mask for unbound
# workqueues (/sys/devices/virtual/workqueue/cpumask), for the loaded
# emulator validation's Option A (docs/hardware/control-rate.md), run by its
# supervisor (tools/option_a.sh). The mask limits every unbound workqueue,
# not only the tty layer's flush work: a changed host condition, recorded as
# such. tools/validation_gate.py checks a run against this tool's record.
#
# usage: tools/wq_mask.sh clear CPU | check | restore | status
#
# clear CPU reads the effective mask (cpumask), the requested one
#   (cpumask_requested) and the isolated one strictly, and refuses with
#   nothing changed if a record is open, a value cannot be read, the
#   requested and effective masks differ, the CPU is not in the mask, the
#   target would be empty, or the workers' census is incomplete. It records
#   the originals and the target, then that the restoration is owed, before
#   it writes; it writes the target (the mask less the CPU, every other bit
#   kept) through sudo and verifies that both masks read back as the target
#   and that, within the wait, a complete census finds no unbound kernel
#   worker or rescuer (kworker/u*, kworker/R-*) allowed on the CPU. A failed
#   or unconfirmed write, a failed verification or an interruption restores
#   at once, as restore does; a restoration that is not verified keeps the
#   record open and the tool fails (unresolved: run restore).
# check compares both masks with the record's target, appends the
#   observation and succeeds only if both equal it.
# restore writes the original requested mask and verifies that both masks
#   read back as their originals and that, within the wait, a complete
#   census finds no more of those workers barred from the CPU than at the
#   start. It appends the result and only then closes the record (renamed to
#   <record>.restored-<time> and kept); any failure keeps it open. restore
#   can be repeated, and it is right after a reboot too (the kernel's
#   default is then in effect).
# status prints the record and the masks now.
#
# The census: every /proc entry whose name is a number. One whose comm or
# status cannot be read is gone only if its /proc entry no longer exists;
# while it exists it is unknown, as is a worker whose Cpus_allowed_list is
# missing or malformed. An unknown entry fails the census: it may be a
# worker allowed on the CPU.
#
# The record, ${XDG_STATE_HOME:-~/.local/state}/rtctrl-wq-mask.record
# (persistent: the kernel keeps the mask through a logout, until a reboot),
# is append-only, one line per step, every append checked (a failed append
# stops the tool before the next change):
#   started=TIME, cpu=N, original_cpumask=M, original_requested=M,
#   original_isolated=M, original_census=workers=N allowed=N unknown=0,
#   target=M (each once, in this order), then restore_owed=TIME,
#   set_write=TIME status=N, set_verified=TIME cpumask=M requested=M
#   workers=N allowed=0 unknown=0 or set_failed=TIME reason=TEXT;
#   observed=TIME cpumask=M requested=M result=matches|differs (or
#   result=unreadable); restore_write=TIME status=N, restore_failed=TIME
#   reason=TEXT, restored=TIME cpumask=M requested=M workers=N allowed=N
#   unknown=0 context=restore|failed-clear. TIME is
#   YYYY-MM-DDTHH:MM:SS.NNNNNNNNN+ZZZZ.
#
# Exit: 0 done; 1 a step or a verification failed, or interrupted (an open
# record is kept for restore); 2 refused, nothing changed.
#
# For tests: WQ_MASK_SYSFS replaces /sys/devices/virtual/workqueue,
# WQ_MASK_PROC /proc, WQ_MASK_SUDO sudo (none: directly; the supervisor
# passes tools/root_keeper.sh), WQ_MASK_RECORD the record and WQ_MASK_WAIT
# the workers' wait in seconds (default 5).
set -u
SYSFS=${WQ_MASK_SYSFS:-/sys/devices/virtual/workqueue}
PROC=${WQ_MASK_PROC:-/proc}
case "${WQ_MASK_SUDO:-sudo}" in none) SUDO=() ;; *) SUDO=("${WQ_MASK_SUDO:-sudo}") ;; esac
rec=${WQ_MASK_RECORD:-${XDG_STATE_HOME:-$HOME/.local/state}/rtctrl-wq-mask.record}
WAIT_S=${WQ_MASK_WAIT:-5}

usage() { echo "usage: $0 clear CPU | check | restore | status" >&2; exit 2; }
refuse() { echo "wq_mask: $*: nothing changed" >&2; exit 2; }
now() { date +%Y-%m-%dT%H:%M:%S.%N%z; }
note() {  # append one record line, checked
  printf '%s\n' "$1" >>"$rec" 2>/dev/null
}

# read_mask NAME: the mask file's value, strictly (hex groups, comma-separated), status 1 otherwise.
read_mask() {
  local v
  v=$(cat "$SYSFS/$1" 2>/dev/null) || return 1
  [[ $v =~ ^[0-9a-f]+(,[0-9a-f]+)*$ ]] || return 1
  printf '%s\n' "$v"
}
# mask_math OP ARGS: mask arithmetic, in the kernel's own format (32-bit groups).
#   has MASK CPU: status 0 if the CPU is in it; clear MASK CPU: the mask less the CPU, in MASK's layout;
#   same A B: status 0 if A and B hold the same CPUs.
mask_math() {
  python3 -I -c '
import sys
op, a = sys.argv[1], sys.argv[2]
def val(m):
    v = 0
    for g in m.split(","):
        v = (v << 32) | int(g, 16)
    return v
if op == "has":
    sys.exit(0 if val(a) >> int(sys.argv[3]) & 1 else 1)
if op == "same":
    sys.exit(0 if val(a) == val(sys.argv[3]) else 1)
groups = a.split(",")
v = val(a) & ~(1 << int(sys.argv[3]))
if v == 0:
    sys.exit(1)
out = []
for i, g in enumerate(reversed(groups)):
    out.append(format((v >> (32 * i)) & 0xFFFFFFFF, "0%dx" % len(g)))
print(",".join(reversed(out)))
' "$@"
}
# census CPU: "<workers> <allowed on CPU> <unknown>" over the unbound kernel workers and rescuers (see above).
census() {
  python3 -I -c '
import os, sys
proc, cpu = sys.argv[1], int(sys.argv[2])
def cpus(spec):
    out = set()
    for part in spec.split(","):
        a, sep, b = part.partition("-")
        if not a.isdigit() or (sep and not b.isdigit()):
            raise ValueError(spec)
        out.update(range(int(a), int(b or a) + 1))
    return out
n = on = unknown = 0
for pid in os.listdir(proc):
    if not pid.isdigit():
        continue
    d = f"{proc}/{pid}"
    try:
        comm = open(f"{d}/comm").read().strip()
    except OSError:
        unknown += os.path.exists(d)  # gone: it runs nowhere; still there: perhaps a worker
        continue
    if not comm.startswith(("kworker/u", "kworker/R-")):
        continue
    try:
        status = open(f"{d}/status").read().splitlines()
    except OSError:
        unknown += os.path.exists(d)
        continue
    spec = [ln.split(":", 1)[1].strip() for ln in status if ln.startswith("Cpus_allowed_list:")]
    try:
        if len(spec) != 1:
            raise ValueError(spec)
        allowed = cpus(spec[0])
    except ValueError:
        unknown += 1
        continue
    n += 1
    on += cpu in allowed
print(n, on, unknown)
' "$PROC" "$1"
}
write_mask() {  # VALUE: through sudo; the status of the write itself
  printf '%s\n' "$1" | "${SUDO[@]}" tee "$SYSFS/cpumask" >/dev/null 2>&1
  return "${PIPESTATUS[1]}"
}

r_cpu=; r_cpumask=; r_requested=; r_isolated=; r_target=; r_workers=; r_allowed=
load() {  # the record's fixed values, each exactly once and well formed
  local k v seen=" "
  [ -r "$rec" ] || return 1
  local mask='^[0-9a-f]+(,[0-9a-f]+)*$'
  while IFS='=' read -r k v; do
    [[ $v =~ ^[[:print:]]*$ ]] || { echo "wq_mask: unreadable record line: $k" >&2; return 1; }
    case "$k" in
      # The steps' lines are checked by tools/validation_gate.py; restore and check use the fixed values below.
      started | restore_owed | set_write | set_verified | set_failed | observed | restore_write | restore_failed | restored) continue ;;
      cpu | original_cpumask | original_requested | original_isolated | original_census | target)
        case "$seen" in *" $k "*) echo "wq_mask: $k twice in $rec" >&2; return 1 ;; esac
        seen+="$k " ;;
      *) echo "wq_mask: unknown record line: $k" >&2; return 1 ;;
    esac
    case "$k" in
      cpu) r_cpu=$v ;;
      original_cpumask) r_cpumask=$v ;;
      original_requested) r_requested=$v ;;
      original_isolated) r_isolated=$v ;;
      original_census) [[ $v =~ ^workers=([0-9]+)\ allowed=([0-9]+)\ unknown=0$ ]] || { echo "wq_mask: original_census: $v" >&2; return 1; }
                       r_workers=${BASH_REMATCH[1]}; r_allowed=${BASH_REMATCH[2]} ;;
      target) r_target=$v ;;
    esac
  done <"$rec" || return 1
  [[ $r_cpu =~ ^[0-9]+$ ]] && [[ $r_cpumask =~ $mask ]] && [[ $r_requested =~ $mask ]] && [[ $r_isolated =~ $mask ]] &&
    [[ $r_target =~ $mask ]] && [ -n "$r_workers" ] || { echo "wq_mask: incomplete record $rec" >&2; return 1; }
}

# undo CONTEXT: write the original requested mask back and verify it; closes the record on success. Status 0 if restored.
undo() {
  local st cm rq c='' n on unk barred0 i
  write_mask "$r_requested"
  st=$?
  note "restore_write=$(now) status=$st" || { echo "wq_mask: cannot append to $rec" >&2; return 1; }
  if ! cm=$(read_mask cpumask) || ! rq=$(read_mask cpumask_requested); then
    note "restore_failed=$(now) reason=a mask cannot be read back"; echo "wq_mask: a mask cannot be read back" >&2; return 1
  fi
  if ! mask_math same "$cm" "$r_cpumask" || ! mask_math same "$rq" "$r_requested"; then
    note "restore_failed=$(now) reason=the masks read back as $cm (requested $rq)"
    echo "wq_mask: the masks read back as $cm (requested $rq), not $r_cpumask ($r_requested)" >&2; return 1
  fi
  barred0=$((r_workers - r_allowed))
  for ((i = 0; i <= WAIT_S * 10; i++)); do
    if c=$(census "$r_cpu"); then
      read -r n on unk <<<"$c"
      [ "$unk" -eq 0 ] && [ "$n" -ge 1 ] && [ $((n - on)) -le "$barred0" ] && break
    fi
    c=
    [ "$i" -lt $((WAIT_S * 10)) ] && sleep 0.1
  done
  if [ -z "$c" ]; then
    note "restore_failed=$(now) reason=the masks are restored but the workers' census is incomplete or shows workers barred from CPU $r_cpu"
    echo "wq_mask: the masks are restored but the workers are not shown restored (a complete census with at most $barred0 barred from CPU $r_cpu)" >&2
    return 1
  fi
  note "restored=$(now) cpumask=$cm requested=$rq workers=$n allowed=$on unknown=$unk context=$1" ||
    { echo "wq_mask: cannot append to $rec" >&2; return 1; }
  mv "$rec" "$rec.restored-$(date +%Y%m%dT%H%M%S)" || { echo "wq_mask: restored, but cannot close $rec" >&2; return 1; }
  echo "wq_mask: restored: cpumask $cm (requested $rq), workers $n, $on allowed on CPU $r_cpu (record closed)"
}

[ $# -ge 1 ] || usage
cmd=$1
case "$cmd" in
  clear)
    [ $# -eq 2 ] && [[ $2 =~ ^[0-9]+$ ]] || usage
    cpu=$2
    [ ! -e "$rec" ] || refuse "a record is open ($rec): run tools/wq_mask.sh restore first"
    cm=$(read_mask cpumask) || refuse "cannot read $SYSFS/cpumask"
    rq=$(read_mask cpumask_requested) || refuse "cannot read $SYSFS/cpumask_requested"
    iso=$(read_mask cpumask_isolated) || refuse "cannot read $SYSFS/cpumask_isolated"
    mask_math same "$cm" "$rq" || refuse "the requested mask ($rq) differs from the effective one ($cm)"
    mask_math has "$cm" "$cpu" || refuse "CPU $cpu is not in the mask ($cm)"
    target=$(mask_math clear "$cm" "$cpu") || refuse "the mask less CPU $cpu would be empty"
    c0=$(census "$cpu") || refuse "cannot take the unbound kernel workers' census under $PROC"
    read -r n0 on0 unk0 <<<"$c0"
    [ "$unk0" -eq 0 ] || refuse "the workers' census is incomplete ($unk0 entries under $PROC cannot be read)"
    [ "$n0" -ge 1 ] || refuse "no unbound kernel worker or rescuer under $PROC"
    mkdir -p "$(dirname "$rec")" 2>/dev/null
    { printf '%s\n' "started=$(now)" "cpu=$cpu" "original_cpumask=$cm" "original_requested=$rq" \
        "original_isolated=$iso" "original_census=workers=$n0 allowed=$on0 unknown=0" "target=$target" >"$rec"; } 2>/dev/null ||
      { rm -f "$rec"; refuse "cannot write $rec"; }
    r_cpu=$cpu; r_cpumask=$cm; r_requested=$rq; r_isolated=$iso; r_workers=$n0; r_allowed=$on0; r_target=$target
    note "restore_owed=$(now)" || { rm -f "$rec"; refuse "cannot write $rec"; }
    # From here the restoration is owed: every failure, and an interruption, undoes the change.
    failed() {
      trap '' INT TERM HUP
      note "set_failed=$(now) reason=$1"
      echo "wq_mask: $1; restoring" >&2
      if undo failed-clear; then exit 1; fi
      echo "wq_mask: the restoration is not verified: the record $rec stays open; run tools/wq_mask.sh restore" >&2
      exit 1
    }
    trap 'failed interrupted' INT TERM HUP
    write_mask "$target"
    st=$?
    note "set_write=$(now) status=$st" || failed "cannot append to $rec"
    [ "$st" -eq 0 ] || failed "the write of $target failed (status $st)"
    c_m=$(read_mask cpumask) || failed "cpumask cannot be read back"
    c_r=$(read_mask cpumask_requested) || failed "cpumask_requested cannot be read back"
    mask_math same "$c_m" "$target" && mask_math same "$c_r" "$target" ||
      failed "the masks read back as $c_m (requested $c_r), not $target"
    ok=0
    for ((i = 0; i <= WAIT_S * 10; i++)); do
      if c=$(census "$cpu"); then
        read -r n on unk <<<"$c"
        [ "$unk" -eq 0 ] && [ "$n" -ge 1 ] && [ "$on" -eq 0 ] && { ok=1; break; }
      fi
      [ "$i" -lt $((WAIT_S * 10)) ] && sleep 0.1
    done
    [ "$ok" -eq 1 ] || failed "after ${WAIT_S} s, no complete census shows every unbound kernel worker off CPU $cpu (last: ${c:-none})"
    note "set_verified=$(now) cpumask=$c_m requested=$c_r workers=$n allowed=0 unknown=0" || failed "cannot append to $rec"
    trap - INT TERM HUP
    echo "wq_mask: CPU $cpu cleared: cpumask $cm -> $c_m (requested $c_r); $n unbound workers and rescuers, none allowed on CPU $cpu"
    echo "  record: $rec (open: restore with tools/wq_mask.sh restore)"
    ;;
  check)
    [ $# -eq 1 ] || usage
    load || { echo "wq_mask: no readable open record ($rec)" >&2; exit 1; }
    if ! c_m=$(read_mask cpumask) || ! c_r=$(read_mask cpumask_requested); then
      note "observed=$(now) result=unreadable"; echo "wq_mask: a mask cannot be read" >&2; exit 1
    fi
    if mask_math same "$c_m" "$r_target" && mask_math same "$c_r" "$r_target"; then
      note "observed=$(now) cpumask=$c_m requested=$c_r result=matches" || { echo "wq_mask: cannot append to $rec" >&2; exit 1; }
      echo "wq_mask: the target $r_target is in effect (cpumask $c_m, requested $c_r)"
    else
      note "observed=$(now) cpumask=$c_m requested=$c_r result=differs"
      echo "wq_mask: the masks are $c_m (requested $c_r), not the target $r_target" >&2
      exit 1
    fi
    ;;
  restore)
    [ $# -eq 1 ] || usage
    [ -e "$rec" ] || { echo "wq_mask: nothing to restore (no open record $rec)"; exit 0; }
    load || exit 1
    trap 'note "restore_failed=$(now) reason=interrupted"; echo "wq_mask: interrupted: the record $rec stays open" >&2; exit 1' INT TERM HUP
    undo restore || { echo "wq_mask: the record $rec stays open; repeat tools/wq_mask.sh restore" >&2; exit 1; }
    ;;
  status)
    [ $# -eq 1 ] || usage
    if [ -e "$rec" ]; then echo "open record $rec:"; sed 's/^/  /' "$rec"; else echo "no open record ($rec)"; fi
    echo "now: cpumask $(read_mask cpumask || echo unreadable), requested $(read_mask cpumask_requested || echo unreadable)," \
      "isolated $(read_mask cpumask_isolated || echo unreadable)"
    ;;
  *) usage ;;
esac
