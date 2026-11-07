#!/usr/bin/env bash
# The timer mode's second trace instance (tools/trace_session.sh,
# docs/hardware/control-rate.md, timer service diagnostics): the real-time
# CPU's local timer interrupts on the x86-tsc clock, the anchors that
# convert the TSC deadline MSR's values to the main trace's monotonic time.
# Run it as root; it changes nothing but its own instance.
#
# usage: tsc_instance.sh start  TRACEFS NAME CPU BUFFER_KB
#        tsc_instance.sh stop   TRACEFS NAME      stops recording, prints the trace
#        tsc_instance.sh stats  TRACEFS NAME CPU  prints the CPU's buffer stats ("CPU: N" first)
#        tsc_instance.sh remove TRACEFS NAME      removes it (verified; 0 if absent)
#        tsc_instance.sh exists TRACEFS NAME      status 0 if it exists
#
# start creates the instance, sets and verifies its clock ([x86-tsc]) and
# CPU mask, sets its buffer, enables irq_vectors:local_timer_entry and turns
# it on, and prints what it set. Its status says whether it created the
# instance: 2 when it did not (the instance exists, including one another
# session created first, or cannot be made), so the caller must leave it
# alone; 4 when a step after creating it failed, leaving it to the caller's
# remove; 0 when it is ready. Other commands exit 1 on a failure.
set -u
usage() { echo "usage: $0 start|stop|stats|remove|exists TRACEFS NAME [CPU [BUFFER_KB]]" >&2; exit 2; }
[ $# -ge 3 ] || usage
cmd=$1; tfs=$2; name=$3
case "$name" in ""|*/*|.*) usage ;; esac
inst="$tfs/instances/$name"
fail() { echo "tsc_instance: $*" >&2; exit 1; }
case "$cmd" in
  start)
    [ $# -eq 5 ] || usage
    cpu=$4; kb=$5
    [[ $cpu =~ ^[0-9]+$ && $kb =~ ^[0-9]+$ ]] || usage
    if [ -e "$inst" ]; then echo "tsc_instance: $inst already exists" >&2; exit 2; fi
    # mkdir is the test that matters: it fails if another session made the
    # instance after the check above.
    if ! mkdir "$inst"; then
      echo "tsc_instance: $inst was not created ($( [ -e "$inst" ] && echo "it exists" || echo "mkdir failed"))" >&2
      exit 2
    fi
    fail() { echo "tsc_instance: $*" >&2; exit 4; }  # from here on, the instance is the caller's
    echo x86-tsc >"$inst/trace_clock" || fail "cannot set the clock"
    grep -q '\[x86-tsc\]' "$inst/trace_clock" || fail "the clock is not x86-tsc: $(cat "$inst/trace_clock")"
    # The CPU's bit, in the kernel's comma-separated 32-bit groups.
    mask=$(printf '%x' $((1 << (cpu % 32))))
    for ((i = 0; i < cpu / 32; i++)); do mask+=",00000000"; done
    echo "$mask" >"$inst/tracing_cpumask" || fail "cannot set the CPU mask $mask"
    got=$(tr -d ',\n' <"$inst/tracing_cpumask" | sed 's/^0*//')
    [ "$got" = "$(tr -d ',' <<<"$mask" | sed 's/^0*//')" ] || fail "the CPU mask reads $(cat "$inst/tracing_cpumask"), not $mask"
    echo "$kb" >"$inst/buffer_size_kb" || fail "cannot set the buffer to $kb kB"
    echo 1 >"$inst/events/irq_vectors/local_timer_entry/enable" || fail "cannot enable local_timer_entry"
    echo 1 >"$inst/tracing_on" || fail "cannot turn the instance on"
    echo "tsc instance $name: clock $(grep -o '\[[^]]*\]' "$inst/trace_clock"), cpumask $(cat "$inst/tracing_cpumask"), buffer $(cat "$inst/buffer_size_kb") kB per CPU, event irq_vectors:local_timer_entry"
    ;;
  stop)
    echo 0 >"$inst/tracing_on" || fail "cannot turn the instance off"
    cat "$inst/trace" || fail "cannot read the trace"
    ;;
  stats)
    [ $# -eq 4 ] || usage
    echo "CPU: $4"
    cat "$inst/per_cpu/cpu$4/stats" || fail "cannot read CPU $4's stats"
    ;;
  remove)
    [ -e "$inst" ] || exit 0
    echo 0 >"$inst/tracing_on" 2>/dev/null
    rmdir "$inst" || fail "cannot remove $inst"
    [ ! -e "$inst" ] || fail "$inst remains"
    ;;
  exists)
    [ -e "$inst" ]
    ;;
  *) usage ;;
esac
