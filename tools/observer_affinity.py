"""Reversible placement of observer applications off the real-time CPUs, for
tools/trace_session.sh --observers (docs/hardware/control-rate.md, the
observer placement comparison).

confine moves every thread of each named process tree (the operator's
explicit PIDs and their descendants) off the excluded CPUs: each thread's
mask becomes its own mask less those CPUs. A new thread inherits its
creator's mask, so one created between a listing and its creator's change
keeps the excluded CPUs; confine therefore repeats its pass (reviewer,
2026-11-09). Each pass lists the trees afresh: the named roots and every
process found before (by process ID and start time, even if reparented since)
and their descendants, less the exempt trees, also listed afresh; then every
live thread in them. A process or thread not seen before is recorded, with its
identity (process and thread IDs and start times, which guard against ID
reuse) and mask, flushed to disk, before any change to it; every known
thread's mask is read again. Each change is written ahead too: the intended
mask is recorded, flushed, before the thread's mask is set, and the mask read
back after it. confine ends after a pass that finds no new process or thread
and every live one off the excluded CPUs: every thread of the trees alive at
that pass's start held such a mask, so a thread created later by one of them
inherits one too, unless an application sets its own mask (the run's checks
remain; a placement is not a freeze of the applications). It fails (status 1)
when MAX_PASSES passes do not settle, when a thread confine confined or found
off the excluded CPUs holds them again (an application setting its own mask),
for a thread whose mask holds only excluded CPUs, a process or thread whose
identity (its stat), task directory or mask cannot be read, and a change or
read-back that fails on a live thread. Only a missing /proc entry (ENOENT,
ESRCH) says a task exited; any other read failure is never counted as gone.
Repeated passes are no atomic view of the trees: a process forked by a thread
not yet confined, whose parent exits before the next pass, is reparented out
of the trees unseen (the trace's fork events attribute it only if it was
forked after the trace started). Exempt process trees (the session's own, with the
probe and the tracers) are never touched. Kernel threads are never in a named
tree.

Each thread's record keeps the mask it was found with and that mask's origin:
"inventory", the first pass's listing before any change, the thread's
original mask; "unconfined", a thread found by a later pass still holding
excluded CPUs (it inherited a creator's mask before the creator was changed),
its mask before confine changed it; "inherited", a thread found by a later
pass already off them, left as found: that mask is no evidence of the
thread's original placement.

restore reconciles every recorded intent, completed or not (an interruption
can fall between a change and its completion record), against the thread's
identity and current mask: a thread that still exists with its recorded
start time and holds the intended mask goes back to its recorded mask (read
back); one already at its recorded mask needs nothing; a thread that exited,
a reused ID and a mask changed during the session are reported and left as
found. An inherited thread, and a thread or process created after the last
pass in a recorded tree, is given an original mask only where one is
justified: its process's, when every inventory and unconfined thread of that
process had the same one (a process with neither takes its recorded parent's,
a new process its nearest recorded ancestor's), and only while the thread
holds that mask's confined form. Otherwise it is left at its current mask and
reported as left, never given an invented wider mask. Any other error on a
live thread, an identity or task directory that cannot be read, or a
read-back that does not match, is unresolved: restore says so, leaves the
record retryable and exits 1. restore can be repeated; it acts
on what the record and the system show. A record of the earlier single-pass
confine (version 1) is restored as before: its threads are inventory threads.

tasks lists every task's process and thread IDs, its process's parent, both
start times and its allowed CPUs, closed by a completeness line (status 1 and
"lineage INCOMPLETE" when a live task cannot be read): the lineage
tools/trace_session.sh keeps in a run's halfway snapshot and, for a traced
run, right after its trace started.

check reads a run's task snapshot (tools/trace_session.sh's <log>.csv.tasks.txt,
taken halfway through the run) and fails when an observer task, or a helper
thread of the probe (its process's threads but the worker), was allowed on an
excluded CPU. The observers are the recorded processes, matched by process ID
and start time, and their descendants by the snapshot's parents, less the
exempt trees. A record of this confine requires the snapshot's lineage, and
a lineage without its completeness line, or an incomplete one, is no
evidence (status 1); a snapshot without one (before 2026-11-09) is checked
against the recorded processes alone, and the check says so. A snapshot is one moment, not the
whole run.

usage: observer_affinity.py confine --pids P[,P...] --exclude-cpus L --record FILE [--exempt-pid P]...
       observer_affinity.py restore --record FILE
       observer_affinity.py tasks
       observer_affinity.py check --record FILE --tasks FILE --worker-tid TID
"""

from __future__ import annotations

import argparse
import errno
import json
import os
import pathlib
import sys
import time

PROC = pathlib.Path("/proc")
VERSION = 2
MAX_PASSES = 10  # the reviewer's bound (2026-11-09)
LINEAGE = "--- task lineage (observer_affinity.py tasks: pid tid ppid pstart start allowed) ---"
COMPLETE = "lineage complete:"
INCOMPLETE = "lineage INCOMPLETE:"


def cpus(text: str) -> set[int]:
    out: set[int] = set()
    for part in text.split(","):
        if not part:
            continue
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return out


def cpu_list(mask) -> str:
    """A CPU set as ranges, as the kernel lists it (0-1,4-31)."""
    s, out = sorted(mask), []
    i = 0
    while i < len(s):
        j = i
        while j + 1 < len(s) and s[j + 1] == s[j] + 1:
            j += 1
        out.append(str(s[i]) if i == j else f"{s[i]}-{s[j]}")
        i = j + 1
    return ",".join(out)


class Unreadable(Exception):
    """A live task's /proc entry (or mask) that cannot be read. Never
    evidence of its exit: only a missing entry (ENOENT, ESRCH) is that."""


def gone(e: OSError) -> bool:
    """An error that says the task exited (its /proc entry is missing)."""
    return isinstance(e, (FileNotFoundError, ProcessLookupError)) or e.errno in (errno.ENOENT, errno.ESRCH)


def stat_fields(path: pathlib.Path) -> list[str] | None:
    """A /proc stat file's fields after the command (field 3 on); None when
    the task is gone; Unreadable when it cannot be read or parsed."""
    try:
        text = path.read_text()
    except OSError as e:
        if gone(e):
            return None
        raise Unreadable(f"{path}: {e.strerror or e}") from e
    i = text.rfind(")")
    f = text[i + 2:].split() if i >= 0 else []
    if len(f) < 20:
        raise Unreadable(f"{path}: malformed ({text[:60]!r})")
    return f


def start_time(pid: int, tid: int | None = None) -> int | None:
    """A task's start time (its identity, with its ID); None when it is gone;
    Unreadable otherwise."""
    f = stat_fields(PROC / str(pid) / "task" / str(tid) / "stat" if tid else PROC / str(pid) / "stat")
    return int(f[19]) if f else None  # field 22: starttime


def comm(pid: int) -> str:
    try:
        return (PROC / str(pid) / "comm").read_text().strip()
    except OSError:
        return "?"


def processes() -> dict[int, int]:
    """Every process's parent. Unreadable when the listing, or any listed
    process's identity, cannot be read: its place in the trees is unknown."""
    try:
        listed = [d for d in PROC.iterdir() if d.name.isdigit()]
    except OSError as e:
        raise Unreadable(f"{PROC}: {e.strerror or e}") from e
    out: dict[int, int] = {}
    bad: list[str] = []
    for d in listed:
        try:
            f = stat_fields(d / "stat")
        except Unreadable as e:
            bad.append(str(e))
            continue
        if f:
            out[int(d.name)] = int(f[1])
    if bad:
        raise Unreadable(f"{len(bad)} processes: " + "; ".join(bad[:5]))
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


def threads(pid: int) -> list[int] | None:
    """A process's thread IDs; None when the process is gone; Unreadable when
    its task directory cannot be listed."""
    d = PROC / str(pid) / "task"
    try:
        return sorted(int(t.name) for t in d.iterdir() if t.name.isdigit())
    except OSError as e:
        if gone(e):
            return None
        raise Unreadable(f"{d}: {e.strerror or e}") from e


def mask_of(tid: int) -> list[int] | None:
    """A thread's allowed CPUs; None when it is gone; Unreadable otherwise."""
    try:
        return sorted(os.sched_getaffinity(tid))
    except OSError as e:
        if gone(e):
            return None
        raise Unreadable(f"thread {tid}'s mask: {e.strerror or e}") from e


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
    try:
        parents = processes()
    except Unreadable as e:
        print(f"observer_affinity: the processes cannot be listed: {e}", file=sys.stderr)
        return 2
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
    try:
        roots = {p: start_time(p) for p in pids}
        exempt_ids = {p: start_time(p) for p in sorted(set(exempt))}
    except Unreadable as e:
        print(f"observer_affinity: a named or exempt process cannot be read: {e}", file=sys.stderr)
        return 2
    rec = Record(record, "x")
    rec.write(kind="header", version=VERSION, created_ns=time.clock_gettime_ns(time.CLOCK_MONOTONIC),
              exclude=sorted(exclude), allowed=sorted(allowed), exempt=sorted(set(exempt)),
              exempt_ids=[{"pid": p, "start": s} for p, s in exempt_ids.items()],
              roots=[{"pid": p, "start": roots[p], "comm": comm(p)} for p in pids], max_passes=MAX_PASSES)
    found_procs: set[tuple[int, int]] = set()  # (pid, start)
    known: dict[tuple[int, int], dict] = {}   # (tid, start) -> pid, mask found, off (confined or found off)
    moved = skipped = n_gone = failed = 0
    late = {"unconfined": 0, "inherited": 0}
    settled = False
    n = 0
    for n in range(1, MAX_PASSES + 1):
        # The trees afresh: the roots and every process found before, by
        # identity, with their descendants, less the exempt trees. A process
        # or thread that cannot be read is a failure, never an exit.
        try:
            parents = processes()
            spared = tree({p for p, s in exempt_ids.items() if start_time(p) == s}, parents)
            live = {p for p, s in roots.items() if p in parents and start_time(p) == s}
            live |= {p for p, s in found_procs if p in parents and start_time(p) == s}
        except Unreadable as e:
            rec.write(kind="failed", why=f"the trees cannot be listed: {e}")
            failed += 1
            break
        new_found = False
        planned: list[tuple[int, int]] = []
        for p in sorted(tree(live, parents) - spared):
            try:
                ps = start_time(p)
                ts = threads(p) if ps is not None else None
            except Unreadable as e:
                rec.write(kind="failed", pid=p, why=f"it cannot be read: {e}")
                failed += 1
                continue
            if ps is None or ts is None:
                continue  # exited meanwhile
            if (p, ps) not in found_procs:
                found_procs.add((p, ps))
                rec.write(kind="process", pid=p, pstart=ps, ppid=parents.get(p), comm=comm(p), n=n)
                new_found = True
            for t in ts:
                try:
                    s = start_time(p, t)
                    mask = mask_of(t) if s is not None else None
                except Unreadable as e:
                    rec.write(kind="failed", pid=p, tid=t, why=f"it cannot be read: {e}")
                    failed += 1
                    continue
                if s is None or mask is None:
                    continue  # exited meanwhile
                x = known.get((t, s))
                if x is None:
                    origin = "inventory" if n == 1 else ("unconfined" if set(mask) & exclude else "inherited")
                    if origin != "inventory":
                        late[origin] += 1
                    rec.write(kind="thread", pid=p, pstart=ps, ppid=parents.get(p), tid=t, start=s, comm=comm(t),
                              mask=mask, n=n, origin=origin)
                    known[(t, s)] = {"pid": p, "mask": mask, "off": False}
                    planned.append((t, s))
                    new_found = True
                elif x["off"] and set(mask) & exclude:
                    rec.write(kind="failed", tid=t, start=s,
                              why=f"its mask holds excluded CPUs again ({mask}): set by the application")
                    failed += 1
        if failed:
            break
        if not new_found:
            settled = True  # this pass found nothing new, and every live thread off the excluded CPUs
            break
        rec.write(kind="state", state="confining", n=n)
        for t, s in planned:
            x = known[(t, s)]
            p, mask = x["pid"], x["mask"]
            new = sorted(set(mask) & allowed)
            if not new:
                rec.write(kind="skipped", tid=t, start=s, why="its mask holds only excluded CPUs")
                skipped += 1
                continue
            if new == mask:
                rec.write(kind="unchanged", tid=t, start=s)
                x["off"] = True
                continue
            try:
                same = start_time(p, t) == s
            except Unreadable as e:
                rec.write(kind="failed", tid=t, start=s, why=f"it cannot be read: {e}")
                failed += 1
                continue
            if not same:
                rec.write(kind="gone", tid=t, start=s)
                n_gone += 1
                continue
            rec.write(kind="intent", tid=t, start=s, mask=new)  # written ahead of the change
            try:
                os.sched_setaffinity(t, new)
                got = sorted(os.sched_getaffinity(t))
            except OSError as e:
                try:
                    same = start_time(p, t) == s
                except Unreadable:
                    same = True  # not shown to have exited: a failure
                if not same:
                    rec.write(kind="gone", tid=t, start=s, why=str(e))
                    n_gone += 1
                else:
                    rec.write(kind="failed", tid=t, start=s, why=str(e))
                    failed += 1
                continue
            if got != new:
                rec.write(kind="failed", tid=t, start=s, why=f"its mask reads {got}, not {new}")
                failed += 1
                continue
            rec.write(kind="confined", tid=t, start=s, mask=got)
            x["off"] = True
            moved += 1
        if skipped or failed:
            break
    bad = skipped or failed
    state = "confined with failures" if bad else ("confined" if settled else "not settled")
    rec.write(kind="state", state=state, passes=n)
    tail = ""
    if not bad and not settled:
        tail = f"; NOT SETTLED: new processes or threads in each of {MAX_PASSES} passes"
    print(f"observer placement: {len(found_procs)} processes, {len(known)} threads in {n} passes: {moved} confined "
          f"to CPUs {','.join(map(str, sorted(allowed)))}, {skipped} left (only excluded CPUs), {n_gone} gone, "
          f"{failed} failed; found after the first pass: {late['unconfined']} on the excluded CPUs (confined), "
          f"{late['inherited']} already off them{tail}")
    return 1 if bad or not settled else 0


def load(record: pathlib.Path) -> list[dict]:
    return [json.loads(ln) for ln in record.read_text().splitlines() if ln.strip()]


def restore(record: pathlib.Path) -> int:
    try:
        lines = load(record)
    except (OSError, ValueError) as e:
        print(f"observer_affinity: {e}", file=sys.stderr)
        return 2
    head = next((x for x in lines if x["kind"] == "header"), None)
    if head is None:
        print("observer_affinity: the record has no header", file=sys.stderr)
        return 2
    allowed = set(head["allowed"])
    recs = [x for x in lines if x["kind"] == "thread"]
    recorded = {(x["tid"], x["start"]): x for x in recs}
    by_tid: dict[int, list[dict]] = {}
    for x in recs:
        by_tid.setdefault(x["tid"], []).append(x)

    def ident(x: dict) -> tuple[int, int] | None:
        """A change record's thread: by ID and start time, or (version 1,
        no start time) the one thread recorded with its ID."""
        if "start" in x:
            return (x["tid"], x["start"]) if (x["tid"], x["start"]) in recorded else None
        c = by_tid.get(x["tid"], [])
        return (c[0]["tid"], c[0]["start"]) if len(c) == 1 else None

    # Every change intended, completed or not: the intent records, and the
    # completion records of a record written before intents existed.
    intended: dict[tuple[int, int], list[int]] = {}
    orphans: list[int] = []
    for kind in ("confined", "intent"):
        for x in lines:
            if x["kind"] == kind:
                k = ident(x)
                if k is None:
                    orphans.append(x["tid"])
                else:
                    intended[k] = x["mask"]
    # The processes and their justified originals: the single mask of their
    # inventory and unconfined threads (inherited masks are no evidence).
    procs: dict[tuple[int, int], int | None] = {}
    for x in lines:
        if x["kind"] == "process":
            procs[(x["pid"], x["pstart"])] = x.get("ppid")
    for x in recs:
        procs.setdefault((x["pid"], x["pstart"]), x.get("ppid"))
    evidence: dict[tuple[int, int], set[tuple]] = {}
    for x in recs:
        if x.get("origin", "inventory") in ("inventory", "unconfined"):
            evidence.setdefault((x["pid"], x["pstart"]), set()).add(tuple(x["mask"]))

    def original(pk: tuple[int, int], depth: int = 0) -> list[int] | None:
        m = evidence.get(pk)
        if m:
            return list(next(iter(m))) if len(m) == 1 else None
        ppid = procs.get(pk)
        up = [q for q in procs if q[0] == ppid and (q[1] or 0) <= (pk[1] or 0)]
        if not up or depth > 64:
            return None
        return original(max(up, key=lambda q: q[1] or 0), depth + 1)

    rec = Record(record, "a")
    n = {"restored": 0, "exited": 0, "reused": 0, "changed": 0, "inherited restored": 0, "inherited left": 0,
         "new restored": 0, "new left": 0, "already": 0, "failed": 0}

    unresolved: set[int] = set()

    def fail(tid: int | None, why: str) -> None:
        if tid is not None:
            unresolved.add(tid)
        rec.write(kind="restore failed", tid=tid, why=why)
        what = f"thread {tid}" if tid is not None else "the trees"
        print(f"observer_affinity: {what} not restored: {why}", file=sys.stderr)
        n["failed"] += 1

    def identity(pid: int, tid: int) -> tuple[bool, int | None]:
        """(read, start time or None when gone); an unreadable identity
        fails (unresolved, retryable), never counted as an exit."""
        try:
            return True, start_time(pid, tid)
        except Unreadable as e:
            fail(tid, f"its identity cannot be read: {e}")
            return False, None

    def put_back(pid: int, tid: int, start: int | None, mask: list[int], key: str) -> None:
        """Sets and reads back the mask; an error is an exit only when the
        thread is gone."""
        try:
            os.sched_setaffinity(tid, mask)
            got = sorted(os.sched_getaffinity(tid))
        except OSError as e:
            if gone(e):  # the kernel has no such task
                n["exited"] += 1
                return
            ok, s = identity(pid, tid)
            if ok and s != start:
                n["exited"] += 1
            elif ok:
                fail(tid, str(e))
            return
        if got != sorted(mask):
            fail(tid, f"its mask reads {got}, not {sorted(mask)}")
            return
        rec.write(kind=key, tid=tid, start=start, mask=got)
        n[key] += 1

    def current(pid: int, tid: int) -> list[int] | None:
        """The thread's mask, or None when it exited (counted); a live or
        unreadable thread's error fails."""
        try:
            return sorted(os.sched_getaffinity(tid))
        except OSError as e:
            if gone(e):  # the kernel has no such task
                n["exited"] += 1
                return None
            ok, s = identity(pid, tid)
            if ok and s is None:  # its identity says it exited
                n["exited"] += 1
            elif ok:
                fail(tid, str(e))
            return None

    for t in orphans:
        fail(t, "an intent without its thread's record")
    for (t, start), want in intended.items():
        x = recorded[(t, start)]
        ok, s = identity(x["pid"], t)
        if not ok:
            continue
        if s is None:
            n["exited"] += 1
            continue
        if s != start:
            rec.write(kind="left", tid=t, why="the ID was reused")
            n["reused"] += 1
            continue
        cur = current(x["pid"], t)
        if cur is None:
            continue
        if cur == sorted(x["mask"]):
            n["already"] += 1  # never changed, or restored before
        elif cur == sorted(want):
            put_back(x["pid"], t, s, x["mask"], "restored")
        else:
            rec.write(kind="left", tid=t, why=f"its mask changed during the session ({cur})")
            n["changed"] += 1
    # Threads found already confined: their process's original, if justified.
    for x in recs:
        k = (x["tid"], x["start"])
        if x.get("origin") != "inherited" or k in intended:
            continue
        t = x["tid"]
        ok, s = identity(x["pid"], t)
        if not ok:
            continue
        if s is None:
            n["exited"] += 1
            continue
        if s != x["start"]:
            rec.write(kind="left", tid=t, why="the ID was reused")
            n["reused"] += 1
            continue
        cur = current(x["pid"], t)
        if cur is None:
            continue
        mask = original((x["pid"], x["pstart"]))
        if mask is not None and cur == sorted(mask):
            n["already"] += 1
        elif mask is not None and cur == sorted(set(mask) & allowed):
            put_back(x["pid"], t, s, mask, "inherited restored")
        else:
            why = "its process has no single original mask" if mask is None else f"its mask is {cur}"
            rec.write(kind="inherited left", pid=x["pid"], tid=t, why=why)
            n["inherited left"] += 1
    # Threads created after the last pass, in recorded processes and new
    # processes in their trees (the exempt trees left out).
    try:
        parents = processes()
        ex = head.get("exempt_ids") or [{"pid": p, "start": None} for p in head.get("exempt", [])]
        spared = tree({e["pid"] for e in ex if e["pid"] in parents and e["start"] in (None, start_time(e["pid"]))},
                      parents)
        live = {pk for pk in procs if pk[0] in parents and start_time(pk[0]) == pk[1]}
    except Unreadable as e:
        fail(None, f"the trees cannot be listed for the threads created during the session: {e}")
        parents, spared, live = {}, set(), set()
    live_pid = {p: pk for pk in live for p in [pk[0]]}
    newp = tree(set(live_pid), parents) - set(live_pid)
    for p in sorted((set(live_pid) | newp) - spared):
        owner, hops = p, 0
        while owner not in live_pid and owner in parents and hops < 4096:
            owner, hops = parents[owner], hops + 1
        mask = original(live_pid[owner]) if owner in live_pid else None
        try:
            ts = threads(p) or []
        except Unreadable as e:
            fail(None, f"process {p}'s threads cannot be listed: {e}")
            continue
        for t in ts:
            if t in unresolved:
                continue  # already failed above
            ok, s = identity(p, t)
            if not ok or s is None or (t, s) in recorded:
                continue
            cur = current(p, t)
            if cur is None:
                continue
            if mask is not None and cur == sorted(set(mask) & allowed):
                put_back(p, t, s, mask, "new restored")
            elif cur != sorted(mask or []):
                why = "its process has no single original mask" if mask is None else f"its mask is {cur}"
                rec.write(kind="new left", pid=p, tid=t, why=why)
                n["new left"] += 1
    done = n["failed"] == 0
    left = n["changed"] + n["inherited left"] + n["new left"]
    rec.write(kind="state", state=("restored" if not left else "restored, threads left") if done
              else "restore incomplete")
    print("observer placement restored: " + ", ".join(f"{k} {v}" for k, v in n.items())
          + (f"; {left} threads left at their current masks (no justified original, or changed during the "
             "session; see the record)" if left else "")
          + ("" if done else "; INCOMPLETE: the record stays, retry the restore"))
    return 0 if done else 1


def tasks(out=sys.stdout) -> int:
    """Every task's lineage and allowed CPUs, in one /proc pass, then a
    completeness line: "lineage complete: N tasks", or, when a live process
    or thread (its identity, its task directory or its mask) cannot be read,
    "lineage INCOMPLETE" with the reasons and status 1. A task that exits
    during the pass is left out; an unreadable one is never."""
    t0 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    rows: list[str] = []
    problems: list[str] = []
    try:
        pids = sorted(int(d.name) for d in PROC.iterdir() if d.name.isdigit())
    except OSError as e:
        problems.append(f"{PROC}: {e.strerror or e}")
        pids = []
    for pid in pids:
        try:
            f = stat_fields(PROC / str(pid) / "stat")
            ts = threads(pid) if f else None
        except Unreadable as e:
            problems.append(str(e))
            continue
        if not f or ts is None:
            continue  # exited during the pass
        ppid, pstart = int(f[1]), int(f[19])
        for t in ts:
            try:
                s = start_time(pid, t)
                m = mask_of(t) if s is not None else None
            except Unreadable as e:
                problems.append(str(e))
                continue
            if s is None or m is None:
                continue  # exited during the pass
            rows.append(f"{pid} {t} {ppid} {pstart} {s} {cpu_list(m)}")
    t1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    print(LINEAGE, file=out)
    print(f"taken monotonic_ns: {t0} {t1}", file=out)
    if rows:
        print("\n".join(rows), file=out)
    if problems:
        print(f"{INCOMPLETE} {len(problems)} unreadable: " + "; ".join(problems[:10]), file=out)
        print("observer_affinity: the task lineage is incomplete: " + "; ".join(problems[:20]), file=sys.stderr)
        return 1
    print(f"{COMPLETE} {len(rows)} tasks", file=out)
    return 0


def parse_lineage(lines: list[str]) -> tuple[dict | None, tuple[int, int] | None, str | None]:
    """A snapshot's lineage: (pid, tid) -> (ppid, pstart, start, allowed
    CPUs), when it was taken, and why it is not complete evidence (None when
    its completeness line closes it); (None, None, None) without a lineage."""
    try:
        i = lines.index(LINEAGE)
    except ValueError:
        return None, None, None
    out: dict[tuple[int, int], tuple] = {}
    taken = None
    problem = "it has no completeness line (a listing that did not finish)"
    for ln in lines[i + 1:]:
        if ln.startswith("---"):
            break
        w = ln.split()
        if ln.startswith(COMPLETE):
            problem = None
            break
        if ln.startswith(INCOMPLETE):
            problem = ln
            break
        if ln.startswith("taken monotonic_ns:") and len(w) == 4:
            taken = (int(w[2]), int(w[3]))
        elif len(w) == 6 and all(x.isdigit() for x in w[:5]):
            p, t, pp, ps, s = map(int, w[:5])
            out[(p, t)] = (pp, ps, s, None if w[5] == "-" else cpus(w[5]))
        else:
            problem = f"a line it cannot read: {ln[:60]!r}"
            break
    return out, taken, problem


def observers_in(entries: list[dict], lineage: dict) -> tuple[set[int], set[int]]:
    """The observer processes in a lineage: the recorded processes (by ID and
    start time) and their descendants by its parents, less the exempt trees;
    and those of them absent from the record."""
    head = next((x for x in entries if x["kind"] == "header"), {})
    recorded = {(x["pid"], x["pstart"]) for x in entries if x["kind"] in ("thread", "process")}
    procs = {p: (pp, ps) for (p, _), (pp, ps, _, _) in lineage.items()}
    parents = {p: v[0] for p, v in procs.items()}
    ex = head.get("exempt_ids") or [{"pid": p, "start": None} for p in head.get("exempt", [])]
    spared = tree({e["pid"] for e in ex if e["pid"] in procs and e["start"] in (None, procs[e["pid"]][1])}, parents)
    roots = {p for p, (_, ps) in procs.items() if (p, ps) in recorded}
    obs = tree(roots, parents) - spared
    return obs, obs - roots


def check(record: pathlib.Path, tasks_file: pathlib.Path, worker: int) -> int:
    try:
        lines = load(record)
        snap = tasks_file.read_text().splitlines()
    except (OSError, ValueError) as e:
        print(f"observer_affinity: {e}", file=sys.stderr)
        return 2
    head = next((x for x in lines if x["kind"] == "header"), None)
    if head is None:
        print("observer_affinity: the record has no header", file=sys.stderr)
        return 2
    exclude = set(head["exclude"])
    lineage, _, problem = parse_lineage(snap)
    if lineage is None and head.get("version", 1) >= 2:
        print("observer placement check: the snapshot has no task lineage, which this placement record requires",
              file=sys.stderr)
        return 1
    if problem:
        print(f"observer placement check: the snapshot's task lineage is incomplete ({problem}): no evidence",
              file=sys.stderr)
        return 1
    allowed: dict[tuple[int, int], set[int] | None] = {}
    if lineage is not None:
        allowed = {k: v[3] for k, v in lineage.items()}
        obs, new = observers_in(lineage=lineage, entries=lines)
    else:  # a snapshot before the lineage: the recorded processes alone
        for ln in snap:
            w = ln.split()
            if len(w) >= 2 and w[0].startswith("/proc/") and w[0].endswith("/status"):
                parts = w[0].split("/")
                try:
                    allowed[(int(parts[2]), int(parts[4]))] = cpus(w[1])
                except (ValueError, IndexError):
                    continue
        obs, new = {x["pid"] for x in lines if x["kind"] == "thread"}, set()
    probe = next((p for (p, t) in allowed if t == worker), None)
    if probe is None:
        print(f"observer placement check: the worker thread {worker} is not in the snapshot", file=sys.stderr)
        return 1

    def on_excluded(m: set[int] | None) -> bool:
        return m is None or bool(m & exclude)  # an unreadable mask is no evidence of placement

    bad_obs = sorted((p, t) for (p, t), m in allowed.items() if p in obs and on_excluded(m))
    bad_probe = sorted((p, t) for (p, t), m in allowed.items() if p == probe and t != worker and on_excluded(m))
    seen = sum(1 for (p, _) in allowed if p in obs)
    helpers = sum(1 for (p, t) in allowed if p == probe and t != worker)
    print(f"observer placement check (halfway snapshot): {seen} observer threads, {len(bad_obs)} allowed on "
          f"CPUs {','.join(map(str, sorted(exclude)))}; {helpers} probe helper threads, {len(bad_probe)} allowed "
          "there")
    if lineage is not None:
        print(f"  lineage: {len(obs)} observer processes (the recorded ones by ID and start time, and their "
              f"descendants by parent), {len(new)} of them not in the record; the exempt trees left out")
    else:
        print("  lineage: none in this snapshot (taken before 2026-11-09): only the recorded processes were "
              "checked, not processes created after the placement")
    for p, t in (bad_obs + bad_probe)[:20]:
        m = allowed[(p, t)]
        print(f"  process {p} thread {t}: allowed CPUs {'unreadable' if m is None else sorted(m)}")
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
    sub.add_parser("tasks")
    k = sub.add_parser("check")
    k.add_argument("--record", type=pathlib.Path, required=True)
    k.add_argument("--tasks", type=pathlib.Path, required=True)
    k.add_argument("--worker-tid", type=int, required=True)
    a = p.parse_args()
    if a.cmd == "restore":
        return restore(a.record)
    if a.cmd == "tasks":
        return tasks()
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
