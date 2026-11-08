#!/usr/bin/env bash
# The fair server's probe events for the timer mode of tools/trace_session.sh
# (docs/hardware/control-rate.md, Stage 1): kprobe events in one group of the
# session's own, defined by tools/fair_server.py. Run it as root; it changes
# nothing but its group's events, and only ever appends to kprobe_events
# (truncating that file would remove every probe on the system).
#
# usage: fair_server_probes.sh define  TRACEFS GROUP DEFFILE
#        fair_server_probes.sh remove  TRACEFS GROUP DEFFILE   removes each defined event (verified; 0 if absent)
#        fair_server_probes.sh exists  TRACEFS GROUP           status 0 if the group has events
#        fair_server_probes.sh profile TRACEFS GROUP DEFFILE   each event's hits and misses (kprobe_profile)
#
# DEFFILE holds one kprobe_events line per event, every one in GROUP. define
# writes them in order and checks each event's directory; its status says
# whether it created any: 2 when it did not (the group exists, including one
# another session created first, or the first definition was refused), so
# the caller must leave the group alone; 4 when a definition after the first
# failed, leaving the created ones to the caller's remove; 0 when all are
# ready. A refused definition prints the kernel's error log. Other commands
# exit 1 on a failure.
set -u
usage() { echo "usage: $0 define|remove|exists|profile TRACEFS GROUP [DEFFILE]" >&2; exit 2; }
[ $# -ge 3 ] || usage
cmd=$1; tfs=$2; group=$3; deffile=${4:-}
[[ $group =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]] || usage
fail() { echo "fair_server_probes: $*" >&2; exit 1; }
# The event names of DEFFILE, each checked to be in GROUP.
names=()
if [ -n "$deffile" ]; then
  [ -f "$deffile" ] || fail "no definitions file $deffile"
  while IFS= read -r line; do
    [ -n "$line" ] || continue
    if [[ $line =~ ^[pr]:([A-Za-z0-9_]+)/([A-Za-z0-9_]+)[[:space:]] ]] && [ "${BASH_REMATCH[1]}" = "$group" ]; then
      names+=("${BASH_REMATCH[2]}")
    else
      fail "a definition outside the group $group: $line"
    fi
  done <"$deffile"
  [ ${#names[@]} -gt 0 ] || fail "$deffile defines no event"
fi
events="$tfs/events/$group"
case "$cmd" in
  define)
    [ $# -eq 4 ] || usage
    if [ -e "$events" ]; then echo "fair_server_probes: $events already exists" >&2; exit 2; fi
    made=0
    while IFS= read -r line; do
      [ -n "$line" ] || continue
      if ! printf '%s\n' "$line" >>"$tfs/kprobe_events"; then
        echo "fair_server_probes: the kernel refused: $line" >&2
        [ -r "$tfs/error_log" ] && tail -n 4 "$tfs/error_log" >&2
        [ "$made" -eq 0 ] && exit 2
        exit 4
      fi
      made=$((made + 1))
    done <"$deffile"
    for n in "${names[@]}"; do
      [ -d "$events/$n" ] || { echo "fair_server_probes: no event directory $events/$n" >&2; exit 4; }
    done
    echo "fair-server probes $group: ${#names[@]} events defined (${names[*]})"
    ;;
  remove)
    [ $# -eq 4 ] || usage
    for n in "${names[@]}"; do
      [ -e "$events/$n" ] || continue
      printf -- '-:%s/%s\n' "$group" "$n" >>"$tfs/kprobe_events" || fail "cannot remove $group/$n"
    done
    for n in "${names[@]}"; do
      [ ! -e "$events/$n" ] || fail "$events/$n remains"
    done
    ;;
  exists)
    [ -e "$events" ]
    ;;
  profile)
    [ $# -eq 4 ] || usage
    # kprobe_profile: "  <event> <hits> <misses>"
    awk -v want="${names[*]}" '
      BEGIN { n = split(want, w, " "); for (i = 1; i <= n; i++) need[w[i]] = 1 }
      ($1 in need) && NF == 3 { got[$1] = $2 " " $3 }
      END {
        for (i = 1; i <= n; i++) {
          if (!(w[i] in got)) { print w[i] " missing"; bad = 1 } else print w[i] " " got[w[i]]
        }
        exit bad
      }' "$tfs/kprobe_profile" || fail "an event is missing from $tfs/kprobe_profile"
    ;;
  *) usage ;;
esac
