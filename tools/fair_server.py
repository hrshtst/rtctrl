"""The fair server's probes for the timer mode of tools/trace_session.sh
(docs/hardware/control-rate.md, Stage 1: fair-server observation).

The fair server (rq->fair_server) is a deadline-class entity that runs
normal tasks ahead of real-time ones while it is active. Its entry into a
CPU's schedule is observed, not inferred from function entry:

- a selection: fair_server_pick_task returned a task (a return probe), and
  the deadline class's pick (pick_task_dl, entry and return probes) returned
  that same task in the same pick; a pick returning no task is counted apart;
- the server's state at the selection: the fair server's flags word (active,
  throttled, deferred, armed, running...), runtime and deadline, read at the
  fair server pick's entry and at the deadline class's pick entry;
- dispatch: the selection is followed, on that CPU, by the sched_switch to
  the selected task (or, when it was already running, by that task staying
  on until its next switch).

dl_server_start, dl_server_stop and dl_server_timer are probed with the
server's state before each call, as context only: a start can return without
enqueueing the server, so its entry is never counted as activation.

The probes are kprobe events whose arguments are memory offsets read from the
kernel's BTF at each session (they change with the kernel's build), so a
session records the layout it used. This module prints the layout and the
definitions (layout, definitions); tools/fair_server_probes.sh writes them to
tracefs as root.

usage: fair_server.py layout [--btf-raw FILE]
       fair_server.py definitions --group GROUP [--btf-raw FILE]
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

GROUP = "rtctrl_fs"
EVENTS = ("rtfs_dl_pick", "rtfs_dl_pick_ret", "rtfs_fs_pick", "rtfs_fs_pick_ret",
          "rtfs_srv_start", "rtfs_srv_stop", "rtfs_srv_timer")
# Events filtered by the CPU they run on (picks are local to their CPU), and
# those filtered by the server's own CPU (a start or stop can run on another).
CPU_FILTERED = ("rtfs_dl_pick", "rtfs_dl_pick_ret", "rtfs_fs_pick", "rtfs_fs_pick_ret")
SERVER_FILTERED = ("rtfs_srv_start", "rtfs_srv_stop", "rtfs_srv_timer")
FLAGS = ("dl_throttled", "dl_non_contending", "dl_server", "dl_server_active", "dl_defer",
         "dl_defer_armed", "dl_defer_running", "dl_defer_idle")
NEEDED = {
    "rq": ("fair_server", "ext_server", "cpu"),
    "sched_dl_entity": ("runtime", "deadline", "rq") + FLAGS,
    "task_struct": ("pid", "policy", "prio"),
}
TYPE = re.compile(r"^\[(\d+)\] (\w+) '([^']*)'")
MEMBER = re.compile(r"^\t'(\w+)' type_id=\d+ bits_offset=(\d+)(?: bitfield_size=(\d+))?")
LAYOUT_LINE = re.compile(r"^fair-server probe layout: (.*)$")


class LayoutError(Exception):
    pass


def layout_from_btf_raw(lines) -> dict[str, int]:
    """The offsets the probes read, from `bpftool btf dump ... format raw`:
    byte offsets of the members, and each flag's bit in the 32-bit word that
    holds them all."""
    found: dict[str, list[dict[str, tuple[int, int]]]] = {}
    current: dict[str, tuple[int, int]] | None = None
    for line in lines:
        t = TYPE.match(line)
        if t:
            current = None
            if t[2] == "STRUCT" and t[3] in NEEDED:
                current = {}
                found.setdefault(t[3], []).append(current)
            continue
        if current is not None:
            m = MEMBER.match(line)
            if m:
                current[m[1]] = (int(m[2]), int(m[3] or 0))
    # The first struct of each name with every needed member: another struct
    # of the name without them (a module's, say) does not count.
    members: dict[str, dict[str, tuple[int, int]]] = {}
    for name, need in NEEDED.items():
        whole = [s for s in found.get(name, []) if all(m in s for m in need)]
        if not whole:
            raise LayoutError(f"no BTF struct {name} with {', '.join(need)}")
        members[name] = whole[0]
    lay: dict[str, int] = {}
    for name, short in (("rq", "rq"), ("sched_dl_entity", "se"), ("task_struct", "task")):
        for m, (bits, size) in members[name].items():
            if m in NEEDED[name] and m not in FLAGS:
                if size or bits % 8:
                    raise LayoutError(f"{name}.{m} is not byte-aligned")
                lay[f"{short}.{m}"] = bits // 8
    words = {members["sched_dl_entity"][f][0] // 32 for f in FLAGS}
    if len(words) != 1:
        raise LayoutError("the server's flags are not in one 32-bit word")
    word = words.pop()
    lay["se.flags_word"] = word * 4
    for f in FLAGS:
        bits, size = members["sched_dl_entity"][f]
        if size != 1:
            raise LayoutError(f"sched_dl_entity.{f} is not a one-bit field")
        lay[f"bit.{f}"] = bits - word * 32
    return lay


def layout_text(lay: dict[str, int]) -> str:
    return "fair-server probe layout: " + " ".join(f"{k}={v}" for k, v in lay.items())


def parse_layout(record_lines) -> dict[str, int] | None:
    """The layout a session recorded, or None."""
    for line in record_lines:
        m = LAYOUT_LINE.match(line.strip())
        if m:
            return {k: int(v) for k, v in (p.split("=", 1) for p in m[1].split())}
    return None


def definitions(lay: dict[str, int], group: str = GROUP) -> list[str]:
    """The kprobe_events lines, one per event."""
    fs = lay["rq.fair_server"]
    se_state = (f"bits=+{lay['se.flags_word']}($A):x32 runtime=+{lay['se.runtime']}($A):s64 "
                f"deadline=+{lay['se.deadline']}($A):u64")
    se_where = f"srv_rq=+{lay['se.rq']}($A):x64 srv_cpu=+{lay['rq.cpu']}(+{lay['se.rq']}($A)):s32"
    task = f"task=$retval:x64 pid=+{lay['task.pid']}($retval):s32 policy=+{lay['task.policy']}($retval):u32"

    def server(event: str, func: str, arg: str) -> str:
        return (f"p:{group}/{event} {func} se={arg}:x64 {se_where.replace('$A', arg)} "
                f"{se_state.replace('$A', arg)}")

    return [
        (f"p:{group}/rtfs_dl_pick pick_task_dl rq=$arg1:x64 rq_cpu=+{lay['rq.cpu']}($arg1):s32 "
         f"fs_bits=+{fs + lay['se.flags_word']}($arg1):x32 fs_runtime=+{fs + lay['se.runtime']}($arg1):s64 "
         f"fs_deadline=+{fs + lay['se.deadline']}($arg1):u64"),
        f"r:{group}/rtfs_dl_pick_ret pick_task_dl {task} prio=+{lay['task.prio']}($retval):s32",
        server("rtfs_fs_pick", "fair_server_pick_task", "$arg1"),
        f"r:{group}/rtfs_fs_pick_ret fair_server_pick_task {task}",
        server("rtfs_srv_start", "dl_server_start", "$arg1"),
        server("rtfs_srv_stop", "dl_server_stop", "$arg1"),
        server("rtfs_srv_timer", "dl_server_timer", "$arg2"),
    ]


def filters(cpu: int) -> list[tuple[str, str]]:
    """Each event with its trace-cmd filter for one CPU."""
    return [(e, f"CPU == {cpu}" if e in CPU_FILTERED else f"srv_cpu == {cpu}") for e in EVENTS]


def btf_raw(path: str | None) -> list[str]:
    if path:
        with open(path) as f:
            return f.read().splitlines()
    tool = os.environ.get("BPFTOOL", "bpftool")
    r = subprocess.run([tool, "btf", "dump", "file", "/sys/kernel/btf/vmlinux", "format", "raw"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise LayoutError(f"{tool} btf dump failed: {r.stderr.strip()}")
    return r.stdout.splitlines()


def main() -> int:
    p = argparse.ArgumentParser(description="The fair server's probes: layout and definitions.")
    sub = p.add_subparsers(dest="cmd", required=True)
    lp = sub.add_parser("layout")
    lp.add_argument("--btf-raw", help="a bpftool raw BTF dump (default: read the kernel's)")
    dp = sub.add_parser("definitions")
    dp.add_argument("--group", default=GROUP)
    dp.add_argument("--btf-raw", help="a bpftool raw BTF dump (default: read the kernel's)")
    args = p.parse_args()
    try:
        lay = layout_from_btf_raw(btf_raw(args.btf_raw or os.environ.get("FAIR_SERVER_BTF_RAW")))
    except (OSError, LayoutError) as e:
        print(f"fair_server: {e}", file=sys.stderr)
        return 2
    if args.cmd == "layout":
        print(layout_text(lay))
    else:
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", args.group):
            print(f"fair_server: bad group {args.group!r}", file=sys.stderr)
            return 2
        print("\n".join(definitions(lay, args.group)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
