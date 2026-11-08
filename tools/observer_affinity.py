"""Reversible placement of observer applications off the real-time CPUs, for
tools/trace_session.sh --observers (docs/hardware/control-rate.md, the
observer placement comparison).

confine moves every thread of each named process tree (the operator's
explicit PIDs and their descendants) off the excluded CPUs: each thread's
mask becomes its own mask less those CPUs. Before it changes anything it
writes a record of every thread's identity (process and thread IDs and
start times, which guard against PID reuse) and mask, flushed to disk; the
changes and the final state follow in the same record. Each change is
written ahead too: the intended mask is recorded, flushed, before the
thread's mask is set, and the mask read back after it. A thread whose mask
holds only excluded CPUs is left alone and reported; a live thread whose
change or read-back fails is a failure (status 1), never counted as gone. Exempt process trees
(the session's own, with the probe and the tracers) are never touched.
Kernel threads are never in a named tree.

restore reconciles every recorded intent, completed or not (an interruption
can fall between a change and its completion record), against the thread's
identity and current mask: a thread that still exists with its recorded
start time and holds the intended mask goes back to its recorded mask (read
back); one already at its recorded mask needs nothing; a thread that exited,
a reused ID and a mask changed during the session are reported and left as
found. Any other error on a live thread, or a read-back that does not match,
is unresolved: restore says so, leaves the record retryable and exits 1. A thread created during the session inherited
its creator's mask, and a multithreaded process has no single original
mask: a new thread (or a new process in a named tree) is given its process's
(or nearest recorded ancestor process's) original mask only when every
recorded thread of that process had the same one and the new thread still
holds its confined form; otherwise it is left at its current mask and
reported. restore can be repeated; it acts on what the record and the
system show.

check reads a run's task snapshot (tools/trace_session.sh's <log>.csv.tasks.txt,
taken halfway through the run) and fails when a thread of a recorded process,
or a helper thread of the probe (its process's threads but the worker), was
allowed on an excluded CPU. A process created after confine does not appear
in the snapshot's list of recorded processes; the trace's check covers it.

usage: observer_affinity.py confine --pids P[,P...] --exclude-cpus L --record FILE [--exempt-pid P]...
       observer_affinity.py restore --record FILE
       observer_affinity.py check --record FILE --tasks FILE --worker-tid TID
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys
import time

PROC = pathlib.Path("/proc")


def cpus(text: str) -> set[int]:
    out: set[int] = set()
    for part in text.split(","):
        if not part:
            continue
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return out


def stat_fields(path: pathlib.Path) -> list[str] | None:
    """A /proc stat file's fields after the command (field 3 on)."""
    try:
        text = path.read_text()
    except OSError:
        return None
    return text[text.rfind(")") + 2:].split()


def start_time(pid: int, tid: int | None = None) -> int | None:
    f = stat_fields(PROC / str(pid) / "task" / str(tid) / "stat" if tid else PROC / str(pid) / "stat")
    return int(f[19]) if f else None  # field 22: starttime


def comm(pid: int) -> str:
    try:
        return (PROC / str(pid) / "comm").read_text().strip()
    except OSError:
        return "?"


def processes() -> dict[int, int]:
    """Every process's parent."""
    out: dict[int, int] = {}
    for d in PROC.iterdir():
        if d.name.isdigit():
            f = stat_fields(d / "stat")
            if f:
                out[int(d.name)] = int(f[1])
    return out


def tree(roots: set[int], parents: dict[int, int]) -> set[int]:
    kids: dict[int, list[int]] = {}
    for p, pp in parents.items():
        kids.setdefault(pp, []).append(p)
    out, todo = set(), [r for r in roots if r in parents]
    while todo:
        p = todo.pop()
        if p not in out:
            out.add(p)
            todo.extend(kids.get(p, []))
    return out


def threads(pid: int) -> list[int]:
    try:
        return sorted(int(t.name) for t in (PROC / str(pid) / "task").iterdir() if t.name.isdigit())
    except OSError:
        return []


class Record:
    """The write-ahead record: one JSON object per line, flushed and synced."""

    def __init__(self, path: pathlib.Path, mode: str) -> None:
        self.f = open(path, mode)

    def write(self, **obj) -> None:
        self.f.write(json.dumps(obj, sort_keys=True) + "\n")
        self.f.flush()
        os.fsync(self.f.fileno())


def confine(pids: list[int], exclude: set[int], record: pathlib.Path, exempt: list[int]) -> int:
    online = cpus((pathlib.Path("/sys/devices/system/cpu/online")).read_text().strip())
    allowed = online - exclude
    if not allowed:
        print("observer_affinity: the exclusion leaves no CPU", file=sys.stderr)
        return 2
    if record.exists():
        print(f"observer_affinity: the record {record} exists (restore it first)", file=sys.stderr)
        return 2
    parents = processes()
    missing = [p for p in pids if p not in parents]
    if missing:
        print(f"observer_affinity: no process {', '.join(map(str, missing))}", file=sys.stderr)
        return 2
    spared = tree(set(exempt), parents)
    named = [p for p in pids if p in spared]
    if named:
        print(f"observer_affinity: {', '.join(map(str, named))} in an exempt tree (the session's own)",
              file=sys.stderr)
        return 2
    procs = sorted(tree(set(pids), parents) - spared)
    rec = Record(record, "x")
    rec.write(kind="header", version=1, created_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC),
              exclude=sorted(exclude), allowed=sorted(allowed), exempt=sorted(set(exempt)),
              roots=[{"pid": p, "start": start_time(p), "comm": comm(p)} for p in pids])
    planned: list[tuple[int, int, int | None, list[int]]] = []
    for p in procs:
        pstart = start_time(p)
        for t in threads(p):
            try:
                mask = sorted(os.sched_getaffinity(t))
            except OSError:
                continue  # exited meanwhile
            s = start_time(p, t)
            planned.append((p, t, s, mask))
            rec.write(kind="thread", pid=p, pstart=pstart, ppid=parents.get(p), tid=t, start=s,
                      comm=comm(t), mask=mask)
    rec.write(kind="state", state="confining")
    moved = skipped = gone = failed = 0
    for p, t, s, mask in planned:
        new = sorted(set(mask) & allowed)
        if not new:
            rec.write(kind="skipped", tid=t, why="its mask holds only excluded CPUs")
            skipped += 1
            continue
        if new == mask:
            rec.write(kind="unchanged", tid=t)
            continue
        if start_time(p, t) != s:
            rec.write(kind="gone", tid=t)
            gone += 1
            continue
        rec.write(kind="intent", tid=t, mask=new)  # written ahead of the change
        try:
            os.sched_setaffinity(t, new)
            got = sorted(os.sched_getaffinity(t))
        except OSError as e:
            if start_time(p, t) != s:
                rec.write(kind="gone", tid=t, why=str(e))
                gone += 1
            else:
                rec.write(kind="failed", tid=t, why=str(e))
                failed += 1
            continue
        if got != new:
            rec.write(kind="failed", tid=t, why=f"its mask reads {got}, not {new}")
            failed += 1
            continue
        rec.write(kind="confined", tid=t, mask=got)
        moved += 1
    rec.write(kind="state", state="confined" if not failed else "confined with failures")
    print(f"observer placement: {len(procs)} processes, {len(planned)} threads: {moved} confined to "
          f"CPUs {','.join(map(str, sorted(allowed)))}, {skipped} left (only excluded CPUs), {gone} gone, "
          f"{failed} failed")
    return 1 if skipped or failed else 0


def restore(record: pathlib.Path) -> int:
    try:
        lines = [json.loads(ln) for ln in record.read_text().splitlines() if ln.strip()]
    except (OSError, ValueError) as e:
        print(f"observer_affinity: {e}", file=sys.stderr)
        return 2
    head = next((x for x in lines if x["kind"] == "header"), None)
    if head is None:
        print("observer_affinity: the record has no header", file=sys.stderr)
        return 2
    allowed = set(head["allowed"])
    recorded = {x["tid"]: x for x in lines if x["kind"] == "thread"}
    # Every change intended, completed or not: the intent records, and the
    # completion records of a record written before intents existed.
    intended = {x["tid"]: x["mask"] for x in lines if x["kind"] == "confined"}
    intended.update({x["tid"]: x["mask"] for x in lines if x["kind"] == "intent"})
    # A process's single original mask, when all its recorded threads shared one.
    by_proc: dict[int, set[tuple]] = {}
    pstart: dict[int, int | None] = {}
    for x in recorded.values():
        by_proc.setdefault(x["pid"], set()).add(tuple(x["mask"]))
        pstart[x["pid"]] = x["pstart"]
    single = {p: list(next(iter(m))) for p, m in by_proc.items() if len(m) == 1}
    rec = Record(record, "a")
    n = {"restored": 0, "exited": 0, "reused": 0, "changed": 0, "new restored": 0, "new left": 0,
         "already": 0, "failed": 0}

    def fail(tid: int, why: str) -> None:
        rec.write(kind="restore failed", tid=tid, why=why)
        print(f"observer_affinity: thread {tid} not restored: {why}", file=sys.stderr)
        n["failed"] += 1

    def put_back(pid: int, tid: int, start: int | None, mask: list[int], key: str) -> None:
        """Sets and reads back the mask; an error is an exit only when the
        thread is gone."""
        try:
            os.sched_setaffinity(tid, mask)
            got = sorted(os.sched_getaffinity(tid))
        except OSError as e:
            if start_time(pid, tid) != start:
                n["exited"] += 1
            else:
                fail(tid, str(e))
            return
        if got != sorted(mask):
            fail(tid, f"its mask reads {got}, not {sorted(mask)}")
            return
        rec.write(kind=key, tid=tid, mask=got)
        n[key] += 1

    for t, want in intended.items():
        x = recorded.get(t)
        if x is None:
            fail(t, "an intent without its thread's record")
            continue
        s = start_time(x["pid"], t)
        if s is None:
            n["exited"] += 1
            continue
        if s != x["start"]:
            rec.write(kind="left", tid=t, why="the ID was reused")
            n["reused"] += 1
            continue
        try:
            cur = sorted(os.sched_getaffinity(t))
        except OSError as e:
            if start_time(x["pid"], t) is None:
                n["exited"] += 1
            else:
                fail(t, str(e))
            continue
        if cur == sorted(x["mask"]):
            n["already"] += 1  # never changed, or restored before
        elif cur == sorted(want):
            put_back(x["pid"], t, s, x["mask"], "restored")
        else:
            rec.write(kind="left", tid=t, why=f"its mask changed during the session ({cur})")
            n["changed"] += 1
    # New threads in recorded processes, and new processes in their trees.
    parents = processes()
    live = {p for p in pstart if p in parents and start_time(p) == pstart[p]}
    newp = tree(live, parents) - set(pstart)
    for p in sorted(live | newp):
        owner = p
        while owner not in live and owner in parents:
            owner = parents[owner]
        mask = single.get(owner)
        for t in threads(p):
            if t in recorded:
                continue
            s = start_time(p, t)
            try:
                cur = sorted(os.sched_getaffinity(t))
            except OSError as e:
                if start_time(p, t) is None:
                    continue  # gone
                fail(t, str(e))
                continue
            if mask is not None and cur == sorted(set(mask) & allowed):
                put_back(p, t, s, mask, "new restored")
            elif cur != sorted(mask or []):
                why = "its process has no single original mask" if mask is None else f"its mask is {cur}"
                rec.write(kind="new left", pid=p, tid=t, why=why)
                n["new left"] += 1
    done = n["failed"] == 0
    rec.write(kind="state", state="restored" if done else "restore incomplete")
    print("observer placement restored: " + ", ".join(f"{k} {v}" for k, v in n.items())
          + ("" if done else "; INCOMPLETE: the record stays, retry the restore"))
    return 0 if done else 1


def check(record: pathlib.Path, tasks: pathlib.Path, worker: int) -> int:
    try:
        lines = [json.loads(ln) for ln in record.read_text().splitlines() if ln.strip()]
        snap = tasks.read_text().splitlines()
    except (OSError, ValueError) as e:
        print(f"observer_affinity: {e}", file=sys.stderr)
        return 2
    head = next((x for x in lines if x["kind"] == "header"), None)
    if head is None:
        print("observer_affinity: the record has no header", file=sys.stderr)
        return 2
    exclude = set(head["exclude"])
    pids = {x["pid"] for x in lines if x["kind"] == "thread"}
    probe = None
    allowed: dict[tuple[int, int], set[int]] = {}
    for ln in snap:
        w = ln.split()
        if len(w) >= 2 and w[0].startswith("/proc/") and w[0].endswith("/status"):
            parts = w[0].split("/")
            try:
                allowed[(int(parts[2]), int(parts[4]))] = cpus(w[1])
            except (ValueError, IndexError):
                continue
    for (pid, tid) in allowed:
        if tid == worker:
            probe = pid
    if probe is None:
        print(f"observer placement check: the worker thread {worker} is not in the snapshot", file=sys.stderr)
        return 1
    bad_obs = sorted((p, t) for (p, t), m in allowed.items() if p in pids and m & exclude)
    bad_probe = sorted((p, t) for (p, t), m in allowed.items() if p == probe and t != worker and m & exclude)
    seen = sum(1 for (p, _) in allowed if p in pids)
    helpers = sum(1 for (p, t) in allowed if p == probe and t != worker)
    print(f"observer placement check (halfway snapshot): {seen} observer threads, {len(bad_obs)} allowed on "
          f"CPUs {','.join(map(str, sorted(exclude)))}; {helpers} probe helper threads, {len(bad_probe)} allowed "
          "there")
    for p, t in (bad_obs + bad_probe)[:20]:
        print(f"  process {p} thread {t}: allowed CPUs {sorted(allowed[(p, t)])}")
    return 1 if bad_obs or bad_probe else 0


def main() -> int:
    p = argparse.ArgumentParser(description="Reversible placement of observer applications off excluded CPUs.")
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("confine")
    c.add_argument("--pids", required=True)
    c.add_argument("--exclude-cpus", required=True)
    c.add_argument("--record", type=pathlib.Path, required=True)
    c.add_argument("--exempt-pid", type=int, action="append", default=[])
    r = sub.add_parser("restore")
    r.add_argument("--record", type=pathlib.Path, required=True)
    k = sub.add_parser("check")
    k.add_argument("--record", type=pathlib.Path, required=True)
    k.add_argument("--tasks", type=pathlib.Path, required=True)
    k.add_argument("--worker-tid", type=int, required=True)
    a = p.parse_args()
    if a.cmd == "restore":
        return restore(a.record)
    if a.cmd == "check":
        return check(a.record, a.tasks, a.worker_tid)
    try:
        pids = [int(x) for x in a.pids.split(",") if x]
        exclude = cpus(a.exclude_cpus)
    except ValueError:
        print("observer_affinity: --pids and --exclude-cpus are lists of numbers", file=sys.stderr)
        return 2
    if not pids or not exclude:
        print("observer_affinity: --pids and --exclude-cpus are empty", file=sys.stderr)
        return 2
    return confine(pids, exclude, a.record, a.exempt_pid)


if __name__ == "__main__":
    sys.exit(main())
