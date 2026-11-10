#!/usr/bin/env bash
# The timer mode's timekeeping recorder (tools/trace_session.sh,
# docs/hardware/control-rate.md): four kprobe events in a private trace
# instance, the evidence tools/timer_clock.py's certificate rests on.
# ntp_adjtimex (under the timekeeping lock, with the timekeeper's id and the
# modes) and ntp_clear record every change to the NTP state; the entry and
# return of update_wall_time time every timekeeping update on every CPU. Run
# it as root; it changes nothing but its own instance and its own group of
# events, and only ever appends to kprobe_events (truncating that file would
# remove every probe on the system).
#
# usage: tk_recorder.sh start   TRACEFS NAME BUFFER_KB
#        tk_recorder.sh states  TRACEFS NAME   each probe's enable state, one "event state" per line
#        tk_recorder.sh disable TRACEFS NAME   entry first, then the rest, then the instance off
#        tk_recorder.sh profile TRACEFS NAME   each probe's "event hits misses" (kprobe_profile)
#        tk_recorder.sh stats   TRACEFS NAME   each CPU's buffer stats ("CPU: n", then its stats), then "cpus: N"
#        tk_recorder.sh remove  TRACEFS NAME   removes the instance, then the probes (verified; 0 if absent)
#        tk_recorder.sh exists  TRACEFS NAME   status 0 if the instance or the group exists
#
# NAME names both the instance and the event group. start registers the
# probes, creates the instance, sets and verifies its clock ([mono]) and
# buffer, turns it on, enables the probes (the return probe before the entry
# probe, so that the start of the record can hold only a return with no
# entry, which the certificate allows) and reads every enable state back. Its
# status says what it created: 2 when nothing (they exist, including another
# session's, or the first definition was refused), so the caller must leave
# them alone; 4 when a step after the first definition failed, leaving what
# it made to the caller's remove; 0 when it is armed. Other commands exit 1
# on a failure. disable stops the entry probe first, so that the end of the
# record can hold only a return with no entry too.
set -u
usage() { echo "usage: $0 start|states|disable|profile|stats|remove|exists TRACEFS NAME [BUFFER_KB]" >&2; exit 2; }
[ $# -ge 3 ] || usage
cmd=$1; tfs=$2; name=$3
[[ $name =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || usage
inst="$tfs/instances/$name"
events="$tfs/events/$name"
# In the order start enables them: the return probe before the entry probe.
PROBES=(ntp_adjtimex ntp_clear update_return update_entry)
fail() { echo "tk_recorder: $*" >&2; exit 1; }
defs() {
  printf '%s\n' \
    "p:$name/ntp_adjtimex ntp_adjtimex tkid=\$arg1:u32 modes=+0(\$arg2):u32" \
    "p:$name/ntp_clear ntp_clear tkid=\$arg1:u32" \
    "r:$name/update_return update_wall_time" \
    "p:$name/update_entry update_wall_time"
}
case "$cmd" in
  start)
    [ $# -eq 4 ] || usage
    kb=$4
    [[ $kb =~ ^[0-9]+$ ]] || usage
    if [ -e "$inst" ] || [ -e "$events" ]; then echo "tk_recorder: $inst or $events already exists" >&2; exit 2; fi
    made=0
    while IFS= read -r line; do
      if ! printf '%s\n' "$line" >>"$tfs/kprobe_events"; then
        echo "tk_recorder: the kernel refused: $line" >&2
        [ -r "$tfs/error_log" ] && tail -n 4 "$tfs/error_log" >&2
        [ "$made" -eq 0 ] && exit 2
        exit 4
      fi
      made=$((made + 1))
    done < <(defs)
    fail() { echo "tk_recorder: $*" >&2; exit 4; }  # from here on, what it made is the caller's
    for p in "${PROBES[@]}"; do
      [ -d "$events/$p" ] || fail "no event directory $events/$p"
    done
    mkdir "$inst" || fail "the instance $inst was not created"
    echo mono >"$inst/trace_clock" || fail "cannot set the clock"
    grep -q '\[mono\]' "$inst/trace_clock" || fail "the clock is not mono: $(cat "$inst/trace_clock")"
    echo "$kb" >"$inst/buffer_size_kb" || fail "cannot set the buffer to $kb kB"
    echo 1 >"$inst/tracing_on" || fail "cannot turn the instance on"
    for p in "${PROBES[@]}"; do
      echo 1 >"$inst/events/$name/$p/enable" || fail "cannot enable $p"
    done
    for p in "${PROBES[@]}"; do
      state=$(cat "$inst/events/$name/$p/enable") || fail "cannot read $p back"
      [ "$state" = 1 ] || fail "$p reads back as $state, not enabled"
    done
    echo "timekeeping recorder $name: clock $(grep -o '\[[^]]*\]' "$inst/trace_clock"), buffer $(cat "$inst/buffer_size_kb") kB per CPU, events ${PROBES[*]}"
    ;;
  states)
    [ $# -eq 3 ] || usage
    for p in "${PROBES[@]}"; do
      state=$(cat "$inst/events/$name/$p/enable") || fail "cannot read $p"
      echo "$p $state"
    done
    ;;
  disable)
    [ $# -eq 3 ] || usage
    for p in update_entry update_return ntp_clear ntp_adjtimex; do
      echo 0 >"$inst/events/$name/$p/enable" || fail "cannot disable $p"
    done
    echo 0 >"$inst/tracing_on" || fail "cannot turn the instance off"
    ;;
  profile)
    [ $# -eq 3 ] || usage
    # kprobe_profile: "  <event> <hits> <misses>"
    awk -v want="${PROBES[*]}" '
      BEGIN { n = split(want, w, " "); for (i = 1; i <= n; i++) need[w[i]] = 1 }
      ($1 in need) && NF == 3 { if ($1 in got) dup = 1; got[$1] = $2 " " $3 }
      END {
        if (dup) { print "duplicate names in kprobe_profile"; exit 1 }  # the profile names no group: never a guess
        for (i = 1; i <= n; i++) {
          if (!(w[i] in got)) { print w[i] " missing"; bad = 1 } else print w[i] " " got[w[i]]
        }
        exit bad
      }' "$tfs/kprobe_profile" || fail "a probe is missing from $tfs/kprobe_profile"
    ;;
  stats)
    [ $# -eq 3 ] || usage
    found=0
    for c in "$inst"/per_cpu/cpu*; do
      [ -r "$c/stats" ] || fail "cannot read $c/stats"
      echo "CPU: ${c##*/cpu}"
      cat "$c/stats" || fail "cannot read $c/stats"
      found=$((found + 1))
    done
    [ "$found" -gt 0 ] || fail "no per-CPU stats under $inst"
    echo "cpus: $found"  # last: a truncated copy has no count to agree with
    ;;
  remove)
    [ $# -eq 3 ] || usage
    if [ -e "$inst" ]; then
      echo 0 >"$inst/tracing_on" 2>/dev/null
      # A probe still enabled here would make its removal below fail: the
      # instance goes first, and its removal disables them.
      rmdir "$inst" || fail "cannot remove $inst"
      [ ! -e "$inst" ] || fail "$inst remains"
    fi
    for p in "${PROBES[@]}"; do
      [ -e "$events/$p" ] || continue
      printf -- '-:%s/%s\n' "$name" "$p" >>"$tfs/kprobe_events" || fail "cannot remove $name/$p"
    done
    for p in "${PROBES[@]}"; do
      [ ! -e "$events/$p" ] || fail "$events/$p remains"
    done
    ;;
  exists)
    [ $# -eq 3 ] || usage
    [ -e "$inst" ] || [ -e "$events" ]
    ;;
  *) usage ;;
esac
