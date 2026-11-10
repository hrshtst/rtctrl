#!/usr/bin/env python3
"""The sudo refresh's placement in a timer run: read-only.

For a timer run recorded with the pinned refresh (tools/sudo_refresh.sh),
this checks the refresh loop and every process of its tree against the
tracers' CPUs, alongside the run's shared validity checks
(tools/trace_correlate.py), which it does not replace:

- the run record's "sudo refresh:" line: the loop, and the CPUs read back at
  the session's start and at the record, both the tracers' CPUs;
- the tree: the loop, its descendants in the pre-run process listing
  (<run>.csv.procs.txt) and in the halfway snapshot's task lineage
  (<run>.csv.tasks.txt), then every fork the trace records from a member;
- the halfway snapshot: the loop listed, and every member it lists allowed
  only on the tracers' CPUs;
- the trace (trace-cmd report, streamed; its exit must be 0): every CPU a
  member is seen on (its own events, its switch-ins and switch-outs,
  migrations and wake-up targets) among the tracers' CPUs;
- a refresh inside the probe's run window (a sudo a member executes, as
  tools/trace_correlate.py's Run takes the window: the summary's monotonic
  origin plus the first cycle's wake and the last cycle's end);
- no kernel page fault at a NULL-page address in that window, on the CPUs
  the trace records them for (its event filter, CPU 3).

Missing, incomplete or unreadable evidence is a failure. The verdict:
PASS (exit 0), FAIL (exit 1), or INCONCLUSIVE (exit 2) when every check
holds but no refresh fell inside the window.

Usage: refresh_check.py <timer run>.csv   (TRACE_CMD overrides trace-cmd)
"""

from __future__ import annotations

import csv
import os
import re
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path

LINE = re.compile(r"^\s*(.+?)-(\d+)\s+\[(\d+)\]\s+\S+\s+([\d.]+):\s+(\w+):\s*(.*)$")


def cpus(text: str) -> set[int]:
    """A CPU list (0-1,4) as its CPUs; ValueError when it is not one."""
    out: set[int] = set()
    for part in text.split(","):
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return out


def window(run_csv: Path) -> tuple[float, float]:
    """The probe's run window in monotonic seconds, as tools/trace_correlate.py's Run takes it."""
    with open(str(run_csv) + ".summary.toml", "rb") as f:
        origin = int(tomllib.load(f)["result"]["monotonic_origin_ns"])
    first = last = None
    with open(run_csv, newline="") as f:
        for row in csv.DictReader(f):
            if first is None:
                first = row
            last = row
    if first is None or last is None:
        raise ValueError("the probe log has no cycles")
    return (origin + int(first["wake_ns"])) / 1e9, (origin + int(last["cycle_end_ns"])) / 1e9


def procs_children(path: Path) -> dict[int, list[int]]:
    """The pre-run process listing (PID PPID COMMAND): each parent's children."""
    kids: dict[int, list[int]] = {}
    lines = path.read_text(errors="replace").splitlines()
    if not lines or not lines[0].split()[:2] == ["PID", "PPID"]:
        raise ValueError(f"{path.name} is not a PID PPID listing")
    for ln in lines[1:]:
        f = ln.split()
        if len(f) >= 2 and f[0].isdigit() and f[1].isdigit():
            kids.setdefault(int(f[1]), []).append(int(f[0]))
    return kids


def lineage(path: Path) -> list[tuple[int, int, int, str]]:
    """The halfway snapshot's task lineage: (pid, tid, ppid, allowed) per task."""
    rows, on = [], False
    for ln in path.read_text(errors="replace").splitlines():
        if ln.startswith("--- "):
            on = ln.startswith("--- task lineage")
            continue
        f = ln.split()
        if on and len(f) == 6 and all(x.isdigit() for x in f[:5]):
            rows.append((int(f[0]), int(f[1]), int(f[2]), f[5]))
    if not rows:
        raise ValueError(f"{path.name} has no task lineage")
    return rows


def descendants(root: int, kids: dict[int, list[int]]) -> set[int]:
    out, todo = {root}, [root]
    while todo:
        for c in kids.get(todo.pop(), []):
            if c not in out:
                out.add(c)
                todo.append(c)
    return out


def check(run_csv: Path, out=sys.stdout) -> str:
    fails: list[str] = []

    def need(ok: bool, what: str) -> bool:
        if not ok:
            fails.append(what)
        return ok

    rec_path = Path(str(run_csv) + ".run.txt")
    rec = rec_path.read_text(errors="replace") if rec_path.exists() else ""
    m = re.search(r"^sudo refresh: loop (\d+) on CPUs (\S+) \(read back at the session's start\), now (\S+)$", rec, re.M)
    c = re.search(r"^excluded CPUs \(no -excl load, no tracers\): (\S+); tracer and -excl load CPUs: (\S+)$", rec, re.M)
    if not need(m is not None, "the run record has no sudo refresh line") or not need(c is not None, "the run record names no tracer CPUs"):
        return report(fails, None, out)
    assert m is not None and c is not None
    loop = int(m.group(1))
    tracers, excluded = cpus(c.group(2)), cpus(c.group(1))
    try:
        need(cpus(m.group(2)) == tracers, f"the loop read back CPUs {m.group(2)} at the session's start, not {c.group(2)}")
        need(cpus(m.group(3)) == tracers, f"the loop was on CPUs {m.group(3)} at the record, not {c.group(2)}")
    except ValueError:
        need(False, f"the record's read-back is not a CPU list: {m.group(2)}, {m.group(3)}")
    print(f"loop {loop}; tracers' CPUs {c.group(2)}, excluded {c.group(1)}", file=out)
    f = re.search(r"-e exceptions:page_fault_kernel -f CPU\\ ==\\ (\d+)", rec)
    if not need(f is not None, "the record traces no kernel page faults with a CPU filter"):
        return report(fails, None, out)
    assert f is not None
    fault_cpu = int(f.group(1))

    try:
        win = window(run_csv)
    except (OSError, KeyError, ValueError) as e:
        need(False, f"the run window cannot be read ({e})")
        return report(fails, None, out)
    print(f"run window {win[0]:.6f}..{win[1]:.6f} s", file=out)

    # The tree, from the pre-run listing and the halfway lineage, then forks.
    tree = {loop}
    try:
        kids = procs_children(Path(str(run_csv) + ".procs.txt"))
        need(any(loop in v for v in kids.values()), "the pre-run process listing does not list the loop")
        tree |= descendants(loop, kids)
    except (OSError, ValueError) as e:
        need(False, f"no usable pre-run process listing ({e})")
    allowed: dict[int, str] = {}
    try:
        rows = lineage(Path(str(run_csv) + ".tasks.txt"))
        lkids: dict[int, list[int]] = {}
        for pid, tid, ppid, mask in rows:
            if pid == tid:
                lkids.setdefault(ppid, []).append(pid)
        need(any(r[0] == loop for r in rows), "the halfway snapshot does not list the loop")
        tree |= descendants(loop, lkids)
        for pid, _, _, mask in rows:
            if pid in tree:
                allowed[pid] = mask
    except (OSError, ValueError) as e:
        need(False, f"no usable halfway snapshot ({e})")
    for pid, mask in sorted(allowed.items()):
        try:
            need(cpus(mask) == tracers, f"pid {pid} of the refresh is allowed on CPUs {mask} halfway")
        except ValueError:
            need(False, f"pid {pid}'s halfway mask is not a CPU list: {mask}")
    print(f"tree before the trace: {sorted(tree)}; halfway masks: {allowed or 'none'}", file=out)

    # The trace, streamed.
    seen: dict[int, set[int]] = {}
    execs: list[tuple[float, int, str, int]] = []
    faults: list[str] = []
    lines = 0
    with tempfile.TemporaryFile() as err:
        p = subprocess.Popen([os.environ.get("TRACE_CMD", "trace-cmd"), "report", "-t", "-i", str(run_csv) + ".trace.dat"],
                             stdout=subprocess.PIPE, stderr=err, text=True, errors="replace")
        assert p.stdout is not None
        for ln in p.stdout:
            mm = LINE.match(ln)
            if not mm:
                continue
            lines += 1
            pid, cpu, ts, ev, rest = int(mm.group(2)), int(mm.group(3)), float(mm.group(4)), mm.group(5), mm.group(6)
            if ev == "sched_process_fork":
                k = re.search(r"\bpid=(\d+) .*child_pid=(\d+)", rest)
                if k and int(k.group(1)) in tree:
                    tree.add(int(k.group(2)))
            if pid in tree:
                seen.setdefault(pid, set()).add(cpu)
                if ev == "sched_process_exec":
                    fn = re.search(r"filename=(\S+)", rest)
                    execs.append((ts, pid, fn.group(1) if fn else "?", cpu))
            if ev == "sched_switch":
                s = re.search(r":(\d+) \[\d+\] \S+ ==> .*?:(\d+) \[", rest)
                if s:
                    for q in (int(s.group(1)), int(s.group(2))):
                        if q in tree:
                            seen.setdefault(q, set()).add(cpu)
            elif ev == "sched_migrate_task":
                g = re.search(r"\bpid=(\d+) .*dest_cpu=(\d+)", rest)
                if g and int(g.group(1)) in tree:
                    seen.setdefault(int(g.group(1)), set()).add(int(g.group(2)))
            elif ev == "sched_waking":
                g = re.search(r"\bpid=(\d+) .*target_cpu=(\d+)", rest)
                if g and int(g.group(1)) in tree:
                    seen.setdefault(int(g.group(1)), set()).add(int(g.group(2)))
            elif ev == "sched_wakeup":
                g = re.search(r":(\d+) \[\d+\] CPU:(\d+)", rest)
                if g and int(g.group(1)) in tree:
                    seen.setdefault(int(g.group(1)), set()).add(int(g.group(2)))
            elif ev == "page_fault_kernel" and win[0] <= ts <= win[1]:
                a = re.search(r"address=(0x[0-9a-f]+|\d+)", rest)
                if a and int(a.group(1), 0) < 4096:
                    faults.append(f"{ts:.6f} s, CPU {cpu}, {mm.group(1)}-{pid}: {rest.strip()}")
        status = p.wait()
        err.seek(0)
        diag = err.read().decode(errors="replace").strip()
    need(status == 0, f"trace-cmd report exited {status}{': ' + diag[-300:] if diag else ''}")
    need(lines > 0, "the trace has no events")
    print(f"trace: {lines} events read; the tree, {len(tree)} processes:", file=out)
    for pid in sorted(tree):
        print(f"  pid {pid}: seen on CPUs {sorted(seen.get(pid, set())) or 'none'}", file=out)
        bad = seen.get(pid, set()) - tracers
        need(not bad, f"pid {pid} of the refresh was on CPUs {sorted(bad)}, outside the tracers' (excluded {sorted(excluded)})")
    for ts, pid, fn, cpu in execs:
        inside = win[0] <= ts <= win[1]
        print(f"  exec {fn}, pid {pid}, at {ts:.6f} s on CPU {cpu}{' (in the run window)' if inside else ''}", file=out)
    refreshes = [e for e in execs if e[2].endswith("/sudo") and win[0] <= e[0] <= win[1]]
    print(f"refreshes in the run window: {len(refreshes)}", file=out)
    print(f"kernel page faults at NULL-page addresses in the run window (recorded on CPU {fault_cpu} only): {len(faults)}", file=out)
    for x in faults:
        print(f"  {x}", file=out)
    need(not faults, f"{len(faults)} kernel page faults at NULL-page addresses on CPU {fault_cpu} in the run window")
    return report(fails, bool(refreshes), out)


def report(fails: list[str], refreshed: bool | None, out) -> str:
    for x in fails:
        print(f"failed: {x}", file=out)
    if fails:
        verdict = "FAIL"
    elif not refreshed:
        verdict = "INCONCLUSIVE"
        print("no refresh fell inside the run window: the placement during a refresh is not shown", file=out)
    else:
        verdict = "PASS"
    print(f"verdict: {verdict}", file=out)
    return verdict


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 1
    return {"PASS": 0, "FAIL": 1, "INCONCLUSIVE": 2}[check(Path(sys.argv[1]))]


if __name__ == "__main__":
    sys.exit(main())
