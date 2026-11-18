#!/usr/bin/env bash
# Smoke test of tools/wq_mask.sh with a stand-in sysfs directory, a stand-in
# /proc (unbound kernel workers, rescuers and one other thread) and a
# stand-in sudo that applies a write the way the kernel does (both masks and
# the workers' affinities), or fails in a chosen way.
#
# Refusals change nothing: an open record, an unreadable mask, requested and
# effective masks that differ, a CPU not in the mask, an empty target, no
# readable workers, an unwritable record, and an incomplete census (an entry
# that exists but cannot be read, an unreadable comm, a malformed or missing
# affinity list, a descending range such as 3-1 included); a worker that is gone does not count. clear, check and restore succeed
# and leave a closed record with every step; check fails on a changed mask.
# A failed write, a write that applied but failed, a mask read back wrong,
# workers that stay on the CPU, a worker whose affinity list turns malformed
# (3-1) after the write, and an interruption each restore at once and
# close the record (exit 1). A restoration that fails keeps the record open
# (exit 1) until a later restore verifies; an interrupted clear whose
# restoration fails likewise. A corrupt record is refused by check and
# restore. After a "reboot" (the original masks back, the record open),
# restore verifies and closes.
#
# set LIST --rt-cpu N (plan L1), with a stand-in CPU sysfs (CPU 3's siblings
# 2-3): set 4-5, check and restore succeed with a set record (mode, the
# real-time CPU and its siblings, the list, within and full counts); each
# bound refuses with nothing changed (an empty, descending or non-canonical
# list, the real-time CPU or its sibling, offline or isolated CPUs, CPUs
# outside the original, a usage error without --rt-cpu), as do an open record
# and an incomplete census; a failed write, a mask read back wrong, workers
# that stay outside the list, a malformed worker list and an interruption
# restore at once (context failed-set); a restoration whose write fails, or
# whose workers stay narrowed, keeps the record open until one verifies; a
# set record with a cpu line or clear's census is refused; after a reboot
# restore verifies and closes.
#
# usage: wq_mask_smoke.sh <scratch dir>
set -u
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "wq_mask_smoke" "${1:-}" || exit 2  # a new child of an approved base: the only place this test removes
ROOT=$RTCTRL_SCRATCH
mkdir -p "$ROOT/sys" "$ROOT/proc" "$ROOT/state" "$ROOT/cpusys/cpu3/topology"
export ROOT
fail() { echo "FAILED: $1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }
TOOL=tools/wq_mask.sh
export WQ_MASK_SYSFS=$ROOT/sys WQ_MASK_PROC=$ROOT/proc WQ_MASK_SUDO=$ROOT/sudo WQ_MASK_RECORD=$ROOT/state/rec WQ_MASK_WAIT=1
export WQ_MASK_CPUSYS=$ROOT/cpusys
ORIG=ffffffff
TARGET=fffffff7

cat >"$ROOT/cpus" <<'EOF'
import sys
v = 0
for g in sys.argv[1].split(","):
    v = (v << 32) | int(g, 16)
on = [i for i in range(v.bit_length()) if v >> i & 1]
runs, s = [], None
for i, c in enumerate(on):
    if s is None:
        s = c
    if i + 1 == len(on) or on[i + 1] != c + 1:
        runs.append(f"{s}-{c}" if c > s else f"{s}")
        s = None
print(",".join(runs))
EOF
workers() {  # MASK: every stand-in worker's affinity follows it
  local list d
  list=$(python3 "$ROOT/cpus" "$1")
  for d in "$ROOT"/proc/*/; do
    case "$(cat "$d/comm")" in kworker/*) printf 'Name:\tx\nCpus_allowed_list:\t%s\n' "$list" >"$d/status" ;; esac
  done
}
reset() {  # the host as it starts: the original masks, the workers on every CPU, no record
  printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask"
  printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask_requested"
  printf '%s\n' 00000000 >"$ROOT/sys/cpumask_isolated"
  scratch_rm "$ROOT/proc"/* "$ROOT/state"/*
  local p c
  for p in 101:kworker/u128:3 102:kworker/u128:4-events_unbound 103:kworker/R-writeback 104:kworker/R-kblockd 105:bash; do
    mkdir -p "$ROOT/proc/${p%%:*}"; printf '%s\n' "${p#*:}" >"$ROOT/proc/${p%%:*}/comm"
    printf 'Name:\tx\nCpus_allowed_list:\t0-31\n' >"$ROOT/proc/${p%%:*}/status"
  done
  printf 'normal\n' >"$ROOT/mode"
}
cat >"$ROOT/sudo" <<'EOF'
#!/usr/bin/env bash
# The stand-in sudo: only "tee <sysfs>/cpumask", applied as $ROOT/mode says.
[ "$1" = tee ] && [ "$2" = "$WQ_MASK_SYSFS/cpumask" ] || exit 99
read -r value
both() { printf '%s\n' "$1" >"$WQ_MASK_SYSFS/cpumask"; printf '%s\n' "$1" >"$WQ_MASK_SYSFS/cpumask_requested"; }
follow() {
  list=$(python3 "$ROOT/cpus" "$1")
  for d in "$WQ_MASK_PROC"/*/; do
    case "$(cat "$d/comm")" in kworker/*) printf 'Name:\tx\nCpus_allowed_list:\t%s\n' "$list" >"$d/status" ;; esac
  done
}
case "$(cat "$ROOT/mode")" in
  normal) both "$value"; follow "$value" ;;
  fail) exit 1 ;;
  fail_after_write) both "$value"; follow "$value"; exit 1 ;;
  wrong) printf '%s\n' "$value" >"$WQ_MASK_SYSFS/cpumask_requested" ;;
  stuck) both "$value" ;;
  restore_fail) [ "$value" = ffffffff ] && exit 1; both "$value"; follow "$value" ;;
  stuck_restore_fail) [ "$value" = ffffffff ] && exit 1; both "$value" ;;
  hide_worker) both "$value"; follow "$value"; [ "$value" = ffffffff ] || chmod 000 "$WQ_MASK_PROC/103/status" ;;
  descend_worker) both "$value"; follow "$value"; [ "$value" = ffffffff ] || printf 'Name:\tx\nCpus_allowed_list:\t3-1\n' >"$WQ_MASK_PROC/103/status" ;;
esac
EOF
chmod +x "$ROOT/sudo"
mode() { printf '%s\n' "$1" >"$ROOT/mode"; }
masks() { echo "$(cat "$ROOT/sys/cpumask") $(cat "$ROOT/sys/cpumask_requested")"; }
closed() { ls "$ROOT/state"/rec.restored-* 2>/dev/null | head -n 1; }
run() { "$TOOL" "$@" >"$ROOT/out" 2>&1; }

# Refusals, nothing changed.
reset; printf 'started=x\n' >"$ROOT/state/rec"
run clear 3; st=$?
[ "$st" -eq 2 ] && [ "$(masks)" = "$ORIG $ORIG" ] && grep -q 'a record is open' "$ROOT/out" || fail "an open record did not refuse ($st)" "$ROOT/out"
reset; scratch_rm "$ROOT/sys/cpumask_requested"
run clear 3; st=$?
[ "$st" -eq 2 ] && [ ! -e "$ROOT/state/rec" ] || fail "an unreadable mask did not refuse ($st)" "$ROOT/out"
reset; printf 'ffff,zz\n' >"$ROOT/sys/cpumask"
run clear 3; st=$?
[ "$st" -eq 2 ] && [ ! -e "$ROOT/state/rec" ] || fail "a malformed mask did not refuse ($st)" "$ROOT/out"
reset; printf 'fffffff0\n' >"$ROOT/sys/cpumask"
run clear 3; st=$?
[ "$st" -eq 2 ] && grep -q 'differs from the effective' "$ROOT/out" || fail "differing masks did not refuse ($st)" "$ROOT/out"
reset; printf '%s\n' "$TARGET" >"$ROOT/sys/cpumask"; printf '%s\n' "$TARGET" >"$ROOT/sys/cpumask_requested"
run clear 3; st=$?
[ "$st" -eq 2 ] && grep -q 'CPU 3 is not in the mask' "$ROOT/out" || fail "a CPU outside the mask did not refuse ($st)" "$ROOT/out"
reset; printf '00000008\n' >"$ROOT/sys/cpumask"; printf '00000008\n' >"$ROOT/sys/cpumask_requested"
run clear 3; st=$?
[ "$st" -eq 2 ] && grep -q 'would be empty' "$ROOT/out" || fail "an empty target did not refuse ($st)" "$ROOT/out"
reset; scratch_rm "$ROOT/proc"/*
run clear 3; st=$?
[ "$st" -eq 2 ] && [ ! -e "$ROOT/state/rec" ] || fail "no readable workers did not refuse ($st)" "$ROOT/out"
reset; chmod 500 "$ROOT/state"
run clear 3; st=$?
chmod 700 "$ROOT/state"
[ "$st" -eq 2 ] && [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] || fail "an unwritable record did not refuse ($st)" "$ROOT/out"

# The census: an entry that still exists but cannot be read is unknown, never gone; so is a worker without a
# well-formed affinity list. Each refuses clear with nothing changed (the start's census must be complete).
reset; chmod 000 "$ROOT/proc/103/status"
run clear 3; st=$?
chmod 600 "$ROOT/proc/103/status"
[ "$st" -eq 2 ] && [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] && grep -q 'census is incomplete (1 entries' "$ROOT/out" ||
  fail "an unreadable worker among readable ones did not refuse ($st)" "$ROOT/out"
reset; chmod 000 "$ROOT/proc/105/comm"
run clear 3; st=$?
chmod 600 "$ROOT/proc/105/comm"
[ "$st" -eq 2 ] && [ ! -e "$ROOT/state/rec" ] || fail "an entry with an unreadable comm did not refuse ($st)" "$ROOT/out"
for bad in 'Cpus_allowed_list:\t0-x' 'Cpus_allowed_list:\t3-1' 'Name:\tno list'; do  # 3-1: the re-review's descending range (finding 4)
  reset; printf "Name:\tx\n$bad\n" >"$ROOT/proc/102/status"
  run clear 3; st=$?
  [ "$st" -eq 2 ] && [ ! -e "$ROOT/state/rec" ] || fail "a worker status '$bad' did not refuse ($st)" "$ROOT/out"
done
# A worker that has gone (its entry removed) is no worker: clear succeeds without it.
reset; scratch_rm "$ROOT/proc/101"
run clear 3 && grep -q '^set_verified=.* workers=3 allowed=0 unknown=0$' "$ROOT/state/rec" || fail "a gone worker" "$ROOT/out"
run restore || fail "restore after a gone worker" "$ROOT/out"
# A worker that becomes unreadable once the target is written: the verification is incomplete, so clear restores, but the
# restoration's census is incomplete too: the record stays open until the worker can be read and restore verifies.
reset; mode hide_worker
run clear 3; st=$?
[ "$st" -eq 1 ] && [ -e "$ROOT/state/rec" ] && grep -q '^set_failed=.* reason=after 1 s, no complete census' "$ROOT/state/rec" &&
  grep -q '^restore_failed=.* census is incomplete' "$ROOT/state/rec" && [ "$(masks)" = "$ORIG $ORIG" ] ||
  fail "a worker unreadable during the verification ($st)" "$ROOT/out"
chmod 600 "$ROOT/proc/103/status"; mode normal
run restore || fail "restore once the worker is readable" "$ROOT/out"
[ ! -e "$ROOT/state/rec" ] || fail "the record stayed open once the census was complete"

# The whole cycle.
reset
run clear 3 || fail "clear failed" "$ROOT/out"
[ "$(masks)" = "$TARGET $TARGET" ] && grep -q '^target=fffffff7$' "$ROOT/state/rec" && grep -q '^original_census=workers=4 allowed=4 unknown=0$' "$ROOT/state/rec" &&
  grep -q '^restore_owed=' "$ROOT/state/rec" && grep -q '^set_write=.* status=0$' "$ROOT/state/rec" &&
  grep -q '^set_verified=.* cpumask=fffffff7 requested=fffffff7 workers=4 allowed=0 unknown=0$' "$ROOT/state/rec" || fail "clear's record or masks" "$ROOT/state/rec"
[ "$(grep -n '^restore_owed=' "$ROOT/state/rec" | cut -d: -f1)" -lt "$(grep -n '^set_write=' "$ROOT/state/rec" | cut -d: -f1)" ] ||
  fail "the restoration was not owed before the write" "$ROOT/state/rec"
grep -q 'Cpus_allowed_list:.0-31' "$ROOT/proc/105/status" || fail "the census changed a thread that is not a worker"
run check || fail "check failed with the target in effect" "$ROOT/out"
grep -q '^observed=.* cpumask=fffffff7 requested=fffffff7 result=matches$' "$ROOT/state/rec" || fail "check's observation" "$ROOT/state/rec"
printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask"
run check; st=$?
[ "$st" -eq 1 ] && grep -q '^observed=.* result=differs$' "$ROOT/state/rec" || fail "check passed a changed mask ($st)" "$ROOT/out"
printf '%s\n' "$TARGET" >"$ROOT/sys/cpumask"
run restore || fail "restore failed" "$ROOT/out"
c=$(closed)
[ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] && [ -n "$c" ] && grep -q '^restored=.* cpumask=ffffffff requested=ffffffff workers=4 allowed=4 unknown=0 context=restore$' "$c" ||
  fail "restore's masks or closed record" "$ROOT/out"
run restore && grep -q 'nothing to restore' "$ROOT/out" || fail "a second restore" "$ROOT/out"

# Each failure of clear restores at once and closes the record.
for case in fail:"the write of fffffff7 failed" fail_after_write:"the write of fffffff7 failed" \
            wrong:"the masks read back as" stuck:"no complete census shows every unbound kernel worker off CPU 3" \
            descend_worker:"no complete census shows every unbound kernel worker off CPU 3"; do
  reset; mode "${case%%:*}"
  run clear 3; st=$?
  c=$(closed)
  [ "$st" -eq 1 ] && [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] && [ -n "$c" ] &&
    grep -q "^set_failed=.* reason=.*${case#*:}" "$c" && grep -q '^restored=.* context=failed-clear$' "$c" || fail "clear, ${case%%:*}: not restored and closed ($st)" "$ROOT/out"
done
# Interrupted while it waits for the workers: restored and closed.
reset; mode stuck
WQ_MASK_WAIT=30 "$TOOL" clear 3 >"$ROOT/out" 2>&1 &
pid=$!
for _ in $(seq 1 50); do grep -q '^set_write=' "$ROOT/state/rec" 2>/dev/null && break; sleep 0.1; done
sleep 0.3; kill -TERM "$pid"; wait "$pid"; st=$?
c=$(closed)
[ "$st" -eq 1 ] && [ "$(masks)" = "$ORIG $ORIG" ] && [ -n "$c" ] && grep -q '^set_failed=.* reason=interrupted$' "$c" ||
  fail "an interrupted clear was not restored ($st)" "$ROOT/out"

# A restoration that fails keeps the record open until one verifies.
reset
run clear 3 || fail "clear before the failed restore" "$ROOT/out"
mode restore_fail
run restore; st=$?
[ "$st" -eq 1 ] && [ -e "$ROOT/state/rec" ] && grep -q '^restore_failed=.* reason=' "$ROOT/state/rec" && [ "$(masks)" = "$TARGET $TARGET" ] ||
  fail "a failed restore did not keep the record ($st)" "$ROOT/out"
run clear 3; st=$?
[ "$st" -eq 2 ] || fail "clear did not refuse beside an unrestored record ($st)" "$ROOT/out"
mode normal
run restore || fail "the repeated restore failed" "$ROOT/out"
[ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] || fail "the repeated restore did not close the record"
# An interrupted clear whose restoration fails: the record stays open, a later restore closes it.
reset; mode stuck_restore_fail
WQ_MASK_WAIT=30 "$TOOL" clear 3 >"$ROOT/out" 2>&1 &
pid=$!
for _ in $(seq 1 50); do grep -q '^set_write=' "$ROOT/state/rec" 2>/dev/null && break; sleep 0.1; done
sleep 0.3; kill -TERM "$pid"; wait "$pid"; st=$?
[ "$st" -eq 1 ] && [ -e "$ROOT/state/rec" ] && grep -q 'stays open' "$ROOT/out" && [ "$(masks)" = "$TARGET $TARGET" ] ||
  fail "an interrupted clear with a failed restoration ($st)" "$ROOT/out"
mode normal
run restore || fail "restore after the interrupted clear" "$ROOT/out"
[ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] || fail "not restored after the interrupted clear"

# set LIST --rt-cpu N (plan L1): the mask set to exactly LIST, the real-time CPU named, its siblings from the stand-in
# CPU sysfs (CPU 3's are 2-3).
SET_TARGET=00000030
cpusys() {  # ONLINE: the stand-in CPU sysfs
  printf '%s\n' "$1" >"$ROOT/cpusys/online"
  printf '2-3\n' >"$ROOT/cpusys/cpu3/topology/thread_siblings_list"
}
reset; cpusys 0-31
run set 4-5 --rt-cpu 3 || fail "set failed" "$ROOT/out"
[ "$(masks)" = "$SET_TARGET $SET_TARGET" ] && grep -q '^mode=set$' "$ROOT/state/rec" && grep -q '^rt_cpu=3$' "$ROOT/state/rec" &&
  grep -q '^rt_siblings=2-3$' "$ROOT/state/rec" && grep -q '^target_list=4-5$' "$ROOT/state/rec" && grep -q '^target=00000030$' "$ROOT/state/rec" &&
  grep -q '^original_census=workers=4 within=0 full=4 unknown=0$' "$ROOT/state/rec" &&
  grep -q '^set_verified=.* cpumask=00000030 requested=00000030 workers=4 within=4 unknown=0$' "$ROOT/state/rec" &&
  ! grep -q '^cpu=' "$ROOT/state/rec" || fail "set's record or masks" "$ROOT/state/rec"
[ "$(sed -n 2p "$ROOT/state/rec")" = mode=set ] && [ "$(grep -n '^restore_owed=' "$ROOT/state/rec" | cut -d: -f1)" -lt "$(grep -n '^set_write=' "$ROOT/state/rec" | cut -d: -f1)" ] ||
  fail "set's record order" "$ROOT/state/rec"
grep -q 'Cpus_allowed_list:.0-31' "$ROOT/proc/105/status" || fail "set changed a thread that is not a worker"
run check && grep -q '^observed=.* cpumask=00000030 requested=00000030 result=matches$' "$ROOT/state/rec" || fail "check after set" "$ROOT/out"
run restore || fail "restore after set" "$ROOT/out"
c=$(closed)
[ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] && [ -n "$c" ] &&
  grep -q '^restored=.* cpumask=ffffffff requested=ffffffff workers=4 full=4 unknown=0 context=restore$' "$c" || fail "restore after set: masks or record" "$ROOT/out"
# Refusals, nothing changed: each bound of tools/wq_target.py, an open record, an incomplete census, a usage error.
set_refused() {  # WHAT WANT ARGS...: exit 2, the masks and the record untouched, WANT in the output
  local what=$1 want=$2 st
  shift 2
  run "$@"; st=$?
  [ "$st" -eq 2 ] && [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] && grep -q -- "$want" "$ROOT/out" ||
    fail "set, $what: not refused as wanted ($st)" "$ROOT/out"
}
reset; cpusys 0-31
set_refused "an empty list" "an empty CPU list" set "" --rt-cpu 3
set_refused "a descending range" "not a canonical CPU list" set 5-4 --rt-cpu 3
set_refused "a list not in canonical form" "not a canonical CPU list" set 4,5 --rt-cpu 3
set_refused "the real-time CPU" "the real-time CPU 3 or its SMT siblings" set 3-5 --rt-cpu 3
set_refused "its SMT sibling" "the real-time CPU 3 or its SMT siblings" set 2 --rt-cpu 3
set_refused "no real-time CPU" "usage" set 4-5
set_refused "the real-time CPU not named as an option" "usage" set 4-5 3
cpusys 0-3,6-31
set_refused "offline CPUs" "offline: 4,5" set 4-5 --rt-cpu 3
cpusys 0-31
printf '00000010\n' >"$ROOT/sys/cpumask_isolated"
set_refused "an isolated CPU" "isolated: 4" set 4-5 --rt-cpu 3
reset; cpusys 0-31; chmod 000 "$ROOT/proc/103/status"
set_refused "an incomplete census" "census is incomplete" set 4-5 --rt-cpu 3
chmod 600 "$ROOT/proc/103/status"
reset; cpusys 0-31; printf 'started=x\n' >"$ROOT/state/rec"
run set 4-5 --rt-cpu 3; st=$?
[ "$st" -eq 2 ] && [ "$(masks)" = "$ORIG $ORIG" ] && grep -q 'a record is open' "$ROOT/out" || fail "set beside an open record ($st)" "$ROOT/out"
reset; cpusys 0-31; printf 'ffffffcf\n' >"$ROOT/sys/cpumask"; printf 'ffffffcf\n' >"$ROOT/sys/cpumask_requested"
run set 4-5 --rt-cpu 3; st=$?
[ "$st" -eq 2 ] && [ ! -e "$ROOT/state/rec" ] && grep -q 'outside the original mask: 4,5' "$ROOT/out" || fail "set outside the original mask ($st)" "$ROOT/out"
# Each failure of set restores at once and closes the record.
for case in fail:"the write of 00000030 failed" wrong:"the masks read back as" stuck:"allowed only within 4-5" \
            descend_worker:"allowed only within 4-5"; do
  reset; cpusys 0-31; mode "${case%%:*}"
  run set 4-5 --rt-cpu 3; st=$?
  c=$(closed)
  [ "$st" -eq 1 ] && [ "$(masks)" = "$ORIG $ORIG" ] && [ ! -e "$ROOT/state/rec" ] && [ -n "$c" ] &&
    grep -q "^set_failed=.* reason=.*${case#*:}" "$c" && grep -q '^restored=.* full=4 unknown=0 context=failed-set$' "$c" ||
    fail "set, ${case%%:*}: not restored and closed ($st)" "$ROOT/out"
done
# Interrupted while it waits for the workers: restored and closed.
reset; cpusys 0-31; mode stuck
WQ_MASK_WAIT=30 "$TOOL" set 4-5 --rt-cpu 3 >"$ROOT/out" 2>&1 &
pid=$!
for _ in $(seq 1 50); do grep -q '^set_write=' "$ROOT/state/rec" 2>/dev/null && break; sleep 0.1; done
sleep 0.3; kill -TERM "$pid"; wait "$pid"; st=$?
c=$(closed)
[ "$st" -eq 1 ] && [ "$(masks)" = "$ORIG $ORIG" ] && [ -n "$c" ] && grep -q '^set_failed=.* reason=interrupted$' "$c" &&
  grep -q 'context=failed-set$' "$c" || fail "an interrupted set was not restored ($st)" "$ROOT/out"
# A restoration whose write fails, and one whose workers stay narrowed: the record stays open until one verifies.
reset; cpusys 0-31
run set 4-5 --rt-cpu 3 || fail "set before the failed restore" "$ROOT/out"
mode restore_fail
run restore; st=$?
[ "$st" -eq 1 ] && [ -e "$ROOT/state/rec" ] && [ "$(masks)" = "$SET_TARGET $SET_TARGET" ] || fail "a failed restore after set ($st)" "$ROOT/out"
mode stuck
run restore; st=$?
[ "$st" -eq 1 ] && [ -e "$ROOT/state/rec" ] && [ "$(masks)" = "$ORIG $ORIG" ] &&
  grep -q '^restore_failed=.* reason=.*more workers narrowed than at the start' "$ROOT/state/rec" ||
  fail "a restoration with the workers still within 4-5 ($st)" "$ROOT/out"
mode normal; workers "$ORIG"
run restore && [ ! -e "$ROOT/state/rec" ] || fail "the repeated restore after set" "$ROOT/out"
# A set record that is not one (a cpu line in it, or clear's census) is refused by check and restore; after a reboot
# restore verifies and closes.
reset; cpusys 0-31
run set 4-5 --rt-cpu 3 || fail "set before the corrupt records" "$ROOT/out"
cp "$ROOT/state/rec" "$ROOT/rec.good"
for bad in 'cpu=3' 'original_census=workers=4 allowed=4 unknown=0'; do
  cp "$ROOT/rec.good" "$ROOT/state/rec"
  case "$bad" in cpu=*) printf '%s\n' "$bad" >>"$ROOT/state/rec" ;; *) sed -i "s/^original_census=.*/$bad/" "$ROOT/state/rec" ;; esac
  run check; st1=$?
  run restore; st2=$?
  [ "$st1" -eq 1 ] && [ "$st2" -eq 1 ] && [ "$(masks)" = "$SET_TARGET $SET_TARGET" ] || fail "a corrupt set record ($bad) was used ($st1 $st2)" "$ROOT/out"
done
cp "$ROOT/rec.good" "$ROOT/state/rec"
printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask"; printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask_requested"; workers "$ORIG"
run restore && [ ! -e "$ROOT/state/rec" ] || fail "restore of a set record after a reboot" "$ROOT/out"

# A corrupt record is refused; after a reboot (the originals back) restore verifies and closes.
reset
run clear 3 || fail "clear before the corrupt record" "$ROOT/out"
cp "$ROOT/state/rec" "$ROOT/rec.good"
printf 'unknown=1\n' >>"$ROOT/state/rec"
run check; st1=$?
run restore; st2=$?
[ "$st1" -eq 1 ] && [ "$st2" -eq 1 ] && [ "$(masks)" = "$TARGET $TARGET" ] || fail "a corrupt record was used ($st1 $st2)" "$ROOT/out"
cp "$ROOT/rec.good" "$ROOT/state/rec"
printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask"; printf '%s\n' "$ORIG" >"$ROOT/sys/cpumask_requested"; workers "$ORIG"
run restore || fail "restore after a reboot" "$ROOT/out"
[ ! -e "$ROOT/state/rec" ] || fail "the record stayed open after a reboot's restore"
scratch_release
echo "wq_mask smoke OK"
