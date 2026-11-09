#!/usr/bin/env python3
"""Regressions for observer_affinity.py, run under ctest, on real processes
of the test's own (no privilege needed: they are the invoking user's).

A stand-in observer (a process with two threads and a child) is confined
off one CPU and restored. Pinned: the write-ahead record (every thread's
identity, start time and mask before any change); every thread confined to
its mask less the excluded CPUs; a thread whose mask holds only excluded
CPUs left and reported (status 1); the refusals (an existing record, a
missing process, a process in an exempt tree); restore putting back the
recorded masks, leaving a mask changed during the session and a reused ID;
new threads and processes given their process's single original mask, and
left where the process had none; a repeated restore; and the check of a
run's task snapshot (observer threads and the probe's helpers, not its
worker, kept off the excluded CPU).

And the recovery cases the reviewer found (2026-11-08), injected into the
module in this process on a child it creates: an interruption between the
mask change and its completion record, and between the intent and the
change, restored correctly; a failed read-back after the change, a failure
of confinement that restore then repairs; a permission error on a live
thread during restore, and a read-back that does not match, unresolved
(status 1, the record retryable, a later restore completing it); and a
permission error on a thread created during the run.

And the repeated pass (reviewer, 2026-11-09): a thread and a child process
created in the first pass's gap (between its listing and its creator's
change) found and confined by the second, which records them as unconfined;
a thread created after its creator's change, found already confined
(inherited), left as found and restored to its process's original, or, with
no single original, left and reported (and the reviewer's counterexample on
fake identities); an application widening a confined thread again, passes
that never settle, a live thread unreadable in a later pass (failures, then
restored); a reused thread ID recorded as another thread; and an
interruption in a later pass. The task listing's lineage, and the halfway
check's: a process created after the placement covered by its parent, an
exempt tree and a reused process ID left out, a snapshot without the
lineage refused for this record and a legacy one checked as before, said
so.

And read failures (reviewer, 2026-11-09), on a fake /proc with injected
permission errors and stand-in masks: a live process's stat, a thread's
stat, a task directory and a mask that cannot be read fail confinement
(before any change, or in a later pass), leave restoration unresolved and
retryable, and make the task listing incomplete (status 1), which the
check refuses; a task that exits during a pass is still left out.
"""

import contextlib
import errno
import importlib.util
import io
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import time

spec = importlib.util.spec_from_file_location("observer_affinity", "tools/observer_affinity.py")
assert spec is not None and spec.loader is not None
oa = importlib.util.module_from_spec(spec)
sys.modules["observer_affinity"] = oa
spec.loader.exec_module(oa)

CHILD = r"""
import os, subprocess, sys, threading, time
def idle():
    while True:
        time.sleep(1)
threading.Thread(target=idle, daemon=True).start()
kid = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
print(os.getpid(), kid.pid, flush=True)
for line in sys.stdin:
    w = line.split()
    if w and w[0] == "thread":
        threading.Thread(target=idle, daemon=True).start()
        time.sleep(0.1)
        print("ok", flush=True)
    elif w and w[0] == "child":
        k = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
        time.sleep(0.2)
        print(k.pid, flush=True)
"""


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def tool(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "tools/observer_affinity.py", *args], capture_output=True, text=True)


def tids(pid: int) -> list[int]:
    return sorted(int(t) for t in os.listdir(f"/proc/{pid}/task"))


def masks(pid: int) -> dict[int, set[int]]:
    return {t: os.sched_getaffinity(t) for t in tids(pid)}


def start(child: subprocess.Popen) -> tuple[int, int]:
    assert child.stdout is not None
    a, b = child.stdout.readline().split()
    return int(a), int(b)


def say(child: subprocess.Popen, what: str) -> str:
    assert child.stdin is not None and child.stdout is not None
    child.stdin.write(what + "\n")
    child.stdin.flush()
    return child.stdout.readline().strip()


class Injected(Exception):
    pass


def quiet(fn, *args):
    """Runs fn with its prints silenced, returning its status."""
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return fn(*args)


def cpu_list(m) -> str:
    return oa.cpu_list(m)


def lineage_snapshot(root: pathlib.Path, pids: set[int], extra: list[tuple], override: dict | None = None,
                     name: str = "tasks.txt") -> pathlib.Path:
    """A halfway snapshot as the session takes it, its lineage section from
    the tool (the given processes' rows), with extra rows (pid, tid, CPUs)
    and rows' CPUs overridden ((pid, tid) -> CPUs, None: unreadable)."""
    listing = tool("tasks").stdout.splitlines()
    rows = []
    for ln in listing[2:]:
        w = ln.split()
        if len(w) == 6 and int(w[0]) in pids:
            key = (int(w[0]), int(w[1]))
            if override and key in override:
                w[5] = "-" if override[key] is None else cpu_list(override[key])
            rows.append(" ".join(w))
    rows += [f"{p_} {t_} 1 1 1 {cpu_list(m)}" for p_, t_, m in extra]
    path = root / name
    path.write_text("taken: x\n" + "\n".join(listing[:2] + rows + [f"{oa.COMPLETE} {len(rows)} tasks"]) + "\n")
    return path


def fault_checks(root: pathlib.Path, mine: list[int]) -> None:
    ex = {mine[-1]}
    orig = set(mine)
    p = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
    real_write, real_set, real_get = oa.Record.write, oa.os.sched_setaffinity, oa.os.sched_getaffinity
    try:
        time.sleep(0.2)

        def run_case(name: str, write=None, setaff=None, getaff=None) -> pathlib.Path:
            rec = root / f"{name}.jsonl"
            if write:
                oa.Record.write = write
            if setaff:
                oa.os.sched_setaffinity = setaff
            if getaff:
                oa.os.sched_getaffinity = getaff
            try:
                try:
                    st = quiet(oa.confine, [p.pid], ex, rec, [])
                except Injected:
                    st = "interrupted"
            finally:
                oa.Record.write, oa.os.sched_setaffinity, oa.os.sched_getaffinity = real_write, real_set, real_get
            run_case.status = st
            return rec

        # An interruption between the change and its completion record.
        def no_completion(self, **obj):
            if obj.get("kind") == "confined":
                raise Injected
            return real_write(self, **obj)
        rec = run_case("after-change", write=no_completion)
        expect(run_case.status == "interrupted" and os.sched_getaffinity(p.pid) == orig - ex,
               "the change was made before the interruption")
        expect(quiet(oa.restore, rec) == 0 and os.sched_getaffinity(p.pid) == orig,
               "an interrupted change is restored from its intent")
        # An interruption between the intent and the change: nothing to undo.
        def no_change(tid, mask):
            raise Injected
        rec = run_case("before-change", setaff=no_change)
        expect(run_case.status == "interrupted" and os.sched_getaffinity(p.pid) == orig, "no change was made")
        expect(quiet(oa.restore, rec) == 0 and os.sched_getaffinity(p.pid) == orig and
               '"kind": "state", "state": "restored"' in rec.read_text(), "an intent never carried out")
        # A read-back failure after the change: confinement fails, restore repairs.
        calls = {"n": 0}
        def flaky_get(tid):
            if tid == p.pid:
                calls["n"] += 1
                if calls["n"] == 2:  # the read-back right after the change
                    raise PermissionError("injected read-back failure")
            return real_get(tid)
        rec = run_case("readback", getaff=flaky_get)
        expect(run_case.status == 1 and '"kind": "failed"' in rec.read_text() and
               os.sched_getaffinity(p.pid) == orig - ex, "a failed read-back fails confinement")
        expect(quiet(oa.restore, rec) == 0 and os.sched_getaffinity(p.pid) == orig, "and restore repairs it")

        # A permission error on a live thread during restore: unresolved, retryable.
        rec = run_case("eperm")
        expect(run_case.status == 0 and os.sched_getaffinity(p.pid) == orig - ex, "a clean confinement")
        def denied(tid, mask):
            if tid == p.pid:
                raise PermissionError("injected EPERM on a live thread")
            return real_set(tid, mask)
        oa.os.sched_setaffinity = denied
        try:
            st = quiet(oa.restore, rec)
        finally:
            oa.os.sched_setaffinity = real_set
        expect(st == 1 and os.sched_getaffinity(p.pid) == orig - ex and
               '"kind": "state", "state": "restore incomplete"' in rec.read_text(),
               f"a live thread's permission error is unresolved: {st}")
        r = tool("restore", "--record", str(rec))
        expect(r.returncode == 0 and os.sched_getaffinity(p.pid) == orig, f"a retried restore completes: {r.stdout}")
        # A restore whose read-back does not match.
        rec = run_case("mismatch")
        oa.os.sched_setaffinity = lambda tid, mask: None if tid == p.pid else real_set(tid, mask)
        try:
            st = quiet(oa.restore, rec)
        finally:
            oa.os.sched_setaffinity = real_set
        expect(st == 1, "a restore that does not take is unresolved")
        expect(tool("restore", "--record", str(rec)).returncode == 0 and os.sched_getaffinity(p.pid) == orig,
               "and a retry completes it")
    finally:
        oa.Record.write, oa.os.sched_setaffinity, oa.os.sched_getaffinity = real_write, real_set, real_get
        p.kill()
        p.wait()
    # A permission error on a thread created during the run.
    obs = subprocess.Popen([sys.executable, "-c", CHILD], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
    pid, kid = start(obs)
    try:
        time.sleep(0.2)
        rec = root / "new-thread.jsonl"
        expect(quiet(oa.confine, [pid], ex, rec, []) == 0, "a clean confinement of a threaded process")
        expect(say(obs, "thread") == "ok", "a new thread")
        recorded = {json.loads(x)["tid"] for x in rec.read_text().splitlines() if '"kind": "thread"' in x}
        new = [t for t in tids(pid) if t not in recorded]
        oa.os.sched_setaffinity = lambda tid, mask: (_ for _ in ()).throw(PermissionError("injected")) \
            if tid in new else real_set(tid, mask)
        try:
            st = quiet(oa.restore, rec)
        finally:
            oa.os.sched_setaffinity = real_set
        expect(st == 1 and os.sched_getaffinity(new[0]) == orig - ex, "a new thread's permission error is unresolved")
        expect(tool("restore", "--record", str(rec)).returncode == 0 and os.sched_getaffinity(new[0]) == orig,
               "and a retry restores the new thread")
    finally:
        obs.kill()
        obs.wait()
        try:
            os.kill(kid, 9)
        except OSError:
            pass


class Observer:
    """A stand-in observer (CHILD) of the test's: its process, its children."""

    def __init__(self) -> None:
        self.p = subprocess.Popen([sys.executable, "-c", CHILD], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  text=True)
        self.pid, kid = start(self.p)
        self.kids = [kid]
        time.sleep(0.2)
        self.idle = next(t for t in tids(self.pid) if t != self.pid)

    def thread(self) -> int:
        before = set(tids(self.pid))
        expect(say(self.p, "thread") == "ok", "a new thread")
        return (set(tids(self.pid)) - before).pop()

    def child(self) -> int:
        k = int(say(self.p, "child"))
        self.kids.append(k)
        return k

    def masks(self) -> dict[int, set[int]]:
        return {t: os.sched_getaffinity(t) for p in [self.pid] + self.kids for t in tids(p)}

    def close(self) -> None:
        self.p.kill()
        self.p.wait()
        for k in self.kids:
            try:
                os.kill(k, 9)
            except OSError:
                pass


@contextlib.contextmanager
def patched(**fns):
    """The module's functions (and os's, as "os_<name>", and Record.write as
    "record_write") replaced for the block."""
    saved = []
    for name, fn in fns.items():
        if name.startswith("os_"):
            saved.append((oa.os, name[3:], getattr(oa.os, name[3:])))
            setattr(oa.os, name[3:], fn)
        elif name == "record_write":
            saved.append((oa.Record, "write", oa.Record.write))
            oa.Record.write = fn
        else:
            saved.append((oa, name, getattr(oa, name)))
            setattr(oa, name, fn)
    try:
        yield
    finally:
        for obj, name, fn in reversed(saved):
            setattr(obj, name, fn)


def capture(fn, *args) -> tuple:
    """Runs fn, returning its status and what it printed."""
    o, e = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(o), contextlib.redirect_stderr(e):
        st = fn(*args)
    return st, o.getvalue() + e.getvalue()


def entries(rec: pathlib.Path) -> list[dict]:
    return [json.loads(x) for x in rec.read_text().splitlines() if x.strip()]


def repeated_scan_checks(root: pathlib.Path, mine: list[int]) -> None:
    """The repeated pass (reviewer, 2026-11-09), injected into the module in
    this process on stand-in observers it creates."""
    ex = {mine[-1]}
    full = set(mine)
    allowed = full - ex
    real_threads, real_processes, real_start = oa.threads, oa.processes, oa.start_time
    real_set, real_get, real_write = oa.os.sched_setaffinity, oa.os.sched_getaffinity, oa.Record.write
    observers: list[Observer] = []

    def observer() -> Observer:
        o = Observer()
        observers.append(o)
        return o

    def restored(o: Observer, rec: pathlib.Path, what: str) -> str:
        st, out = capture(oa.restore, rec)
        expect(st == 0 and all(m == full for m in o.masks().values()), f"{what}: restored ({st}) {o.masks()}\n{out}")
        return out

    try:
        # The creation gap: a thread and a child process created by the
        # observer's main thread between the first pass's listing and its
        # change keep the excluded CPU; the second pass records them
        # (unconfined) and confines them; the third finds nothing new.
        o = observer()
        fired: list[int] = []
        made: list[int] = []

        def gap(p: int):
            ts = real_threads(p)
            if p == o.pid and not fired:
                fired.append(1)
                made.extend([o.thread(), o.child()])
            return ts
        rec = root / "gap.jsonl"
        with patched(threads=gap):
            st = quiet(oa.confine, [o.pid], ex, rec, [])
        e = entries(rec)
        late = [x for x in e if x["kind"] == "thread" and x.get("origin") == "unconfined"]
        expect(st == 0 and {x["tid"] for x in late} == set(made) and all(x["n"] == 2 for x in late)
               and any(x["kind"] == "process" and x["pid"] == made[1] and x["n"] == 2 for x in e)
               and e[-1] == {"kind": "state", "state": "confined", "passes": 3},
               f"the gap's thread and process found and confined by the second pass: {st} {e[-1]} {late}")
        expect(all(m == allowed for m in o.masks().values()), f"every thread confined: {o.masks()}")
        restored(o, rec, "the gap's tasks")

        # A thread created after its creator's change inherits the confined
        # mask: found by the second pass (inherited), left as found, and
        # restored to its process's original (the reviewer's case).
        o = observer()
        fired.clear()
        made.clear()

        def set_then_spawn(tid, mask):
            real_set(tid, mask)
            if tid == o.pid and not fired:
                fired.append(1)
                made.append(o.thread())
        rec = root / "inherited.jsonl"
        with patched(os_sched_setaffinity=set_then_spawn):
            st = quiet(oa.confine, [o.pid], ex, rec, [])
        e = entries(rec)
        inh = [x for x in e if x["kind"] == "thread" and x.get("origin") == "inherited"]
        expect(st == 0 and [x["tid"] for x in inh] == made and set(inh[0]["mask"]) == allowed and inh[0]["n"] == 2
               and not any(x["kind"] == "intent" and x["tid"] == made[0] for x in e),
               f"an inherited thread recorded as found, unchanged: {st} {inh}")
        out = restored(o, rec, "an inherited thread")
        expect("inherited restored 1" in out, f"the inherited thread given its process's original: {out}")
        # The same where the process has no single original: left, reported.
        o = observer()
        os.sched_setaffinity(o.idle, full - {mine[0]})
        fired.clear()
        made.clear()
        rec = root / "inherited-ambiguous.jsonl"
        with patched(os_sched_setaffinity=set_then_spawn):
            st = quiet(oa.confine, [o.pid], ex, rec, [])
        st, out = capture(oa.restore, rec)
        expect(st == 0 and "inherited left 1" in out and "1 threads left at their current masks" in out
               and os.sched_getaffinity(made[0]) == allowed and os.sched_getaffinity(o.pid) == full
               and '"state": "restored, threads left"' in rec.read_text(),
               f"an inherited thread without a justified original left and reported: {out}")

        # An application widening a confined thread again: a failure.
        o = observer()
        calls: list[int] = []

        def count_then_widen():
            calls.append(1)
            if len(calls) == 3:  # the second pass's listing (the first call precedes the header)
                real_set(o.idle, full)
            return real_processes()
        rec = root / "widened.jsonl"
        with patched(processes=count_then_widen):
            st, out = capture(oa.confine, [o.pid], ex, rec, [])
        text = rec.read_text()
        expect(st == 1 and "holds excluded CPUs again" in text and '"state": "confined with failures"' in text,
               f"a thread widened by the application fails confine: {st} {out}")
        restored(o, rec, "after a widened thread")

        # Passes that never settle (a new thread in each): a failure after
        # MAX_PASSES, then restored, the threads created meanwhile included.
        o = observer()

        def every(p: int):
            ts = real_threads(p)
            if p == o.pid:
                o.thread()
            return ts
        rec = root / "unsettled.jsonl"
        with patched(threads=every):
            st, out = capture(oa.confine, [o.pid], ex, rec, [])
        e = entries(rec)
        expect(st == 1 and "NOT SETTLED" in out and e[-1] == {"kind": "state", "state": "not settled",
                                                               "passes": oa.MAX_PASSES},
               f"unsettled passes fail: {st} {out} {e[-1]}")
        restored(o, rec, "after unsettled passes")

        # A reused thread ID (another start time) is another thread.
        o = observer()
        calls.clear()

        def count():
            calls.append(1)
            return real_processes()

        def reused_start(pid: int, tid: int | None = None):
            v = real_start(pid, tid)
            return v + 1000 if tid == o.idle and len(calls) >= 3 and v is not None else v
        rec = root / "reused.jsonl"
        with patched(processes=count, start_time=reused_start):
            st = quiet(oa.confine, [o.pid], ex, rec, [])
        ids = [(x["start"], x.get("origin")) for x in entries(rec) if x["kind"] == "thread" and x["tid"] == o.idle]
        expect(st == 0 and len(ids) == 2 and ids[1] == (ids[0][0] + 1000, "inherited"),
               f"a reused ID recorded as another thread: {st} {ids}")
        out = restored(o, rec, "a reused ID")
        expect("reused 1" in out, f"the reused ID's record left: {out}")

        # An interruption in a later pass, between a change and its record.
        o = observer()
        fired.clear()
        made.clear()

        def gap_thread(p: int):
            ts = real_threads(p)
            if p == o.pid and not fired:
                fired.append(1)
                made.append(o.thread())
            return ts
        armed: list[int] = []

        def interrupted(self, **obj):
            if obj.get("kind") == "state" and obj.get("n") == 2:
                armed.append(1)
            if armed and obj.get("kind") == "confined":
                raise Injected
            return real_write(self, **obj)
        rec = root / "later-interrupted.jsonl"
        with patched(threads=gap_thread, record_write=interrupted):
            try:
                quiet(oa.confine, [o.pid], ex, rec, [])
                st = "returned"
            except Injected:
                st = "interrupted"
        expect(st == "interrupted" and os.sched_getaffinity(made[0]) == allowed,
               f"the second pass's change made, then interrupted: {st}")
        restored(o, rec, "an interrupted later pass")

        # A live thread whose mask cannot be read in a later pass: a failure.
        o = observer()
        calls.clear()

        def unreadable(tid: int):
            if tid == o.idle and len(calls) >= 3:
                raise PermissionError("injected")
            return real_get(tid)
        rec = root / "unreadable.jsonl"
        with patched(processes=count, os_sched_getaffinity=unreadable):
            st = quiet(oa.confine, [o.pid], ex, rec, [])
        expect(st == 1 and "mask: injected" in rec.read_text(), f"an unreadable live thread fails: {st}")
        restored(o, rec, "after an unreadable thread")
    finally:
        oa.threads, oa.processes, oa.start_time = real_threads, real_processes, real_start
        oa.os.sched_setaffinity, oa.os.sched_getaffinity, oa.Record.write = real_set, real_get, real_write
        for o in observers:
            o.close()


def counterexample_checks(root: pathlib.Path) -> None:
    """The reviewer's restoration counterexample (2026-11-09) in this
    record's form, on fake identities and masks only: a thread found by a
    later pass with its creator's confined mask is restored to the process's
    original; with no single original it is left and reported."""
    full, confined = [0, 1, 2, 3], [0, 1]

    def run(extra: list[dict], masks: dict[int, set[int]]) -> tuple:
        records = [
            {"kind": "header", "version": 2, "allowed": confined, "exclude": [2, 3], "exempt": [], "exempt_ids": [],
             "roots": [{"pid": 101, "start": 1}]},
            {"kind": "process", "pid": 101, "pstart": 1, "ppid": 1, "n": 1},
            {"kind": "thread", "pid": 101, "pstart": 1, "ppid": 1, "tid": 101, "start": 11, "mask": full, "n": 1,
             "origin": "inventory"},
            *extra,
            {"kind": "state", "state": "confining", "n": 1},
            {"kind": "intent", "tid": 101, "start": 11, "mask": confined},
            {"kind": "confined", "tid": 101, "start": 11, "mask": confined},
            {"kind": "thread", "pid": 101, "pstart": 1, "ppid": 1, "tid": 102, "start": 12, "mask": confined, "n": 2,
             "origin": "inherited"},
            {"kind": "state", "state": "confining", "n": 2},
            {"kind": "unchanged", "tid": 102, "start": 12},
            {"kind": "state", "state": "confined", "passes": 3},
        ]
        path = root / "counterexample.jsonl"
        path.write_text("".join(json.dumps(x) + "\n" for x in records))
        with patched(start_time=lambda pid, tid=None: {None: 1, 101: 11, 102: 12, 103: 13}.get(tid),
                     processes=lambda: {101: 1}, threads=lambda pid: sorted(masks) if pid == 101 else [],
                     os_sched_getaffinity=lambda tid: set(masks[tid]),
                     os_sched_setaffinity=lambda tid, mask: masks.__setitem__(tid, set(mask))):
            return capture(oa.restore, path)

    masks = {101: set(confined), 102: set(confined)}
    st, out = run([], masks)
    expect(st == 0 and masks == {101: set(full), 102: set(full)} and "inherited restored 1" in out,
           f"the inherited thread restored with its creator: {st} {masks} {out}")
    masks = {101: set(confined), 102: set(confined), 103: set(confined)}
    st, out = run([{"kind": "thread", "pid": 101, "pstart": 1, "ppid": 1, "tid": 103, "start": 13, "mask": [0, 1, 3],
                    "n": 1, "origin": "inventory"},
                   {"kind": "intent", "tid": 103, "start": 13, "mask": confined}], masks)
    expect(st == 0 and masks == {101: set(full), 102: set(confined), 103: {0, 1, 3}} and "inherited left 1" in out
           and "threads left at their current masks" in out,
           f"no single original: the inherited thread left and reported: {st} {masks} {out}")


def lineage_checks(root: pathlib.Path, mine: list[int]) -> None:
    """The task listing and the halfway check's lineage (reviewer,
    2026-11-09)."""
    ex = {mine[-1]}
    full = set(mine)
    allowed = full - ex
    # The listing: this process's row with its parent, start time and CPUs.
    r = tool("tasks")
    lin, taken, problem = oa.parse_lineage(r.stdout.splitlines())
    me = (lin or {}).get((os.getpid(), os.getpid()))
    expect(r.returncode == 0 and problem is None and r.stdout.splitlines()[-1] == f"{oa.COMPLETE} {len(lin)} tasks"
           and taken is not None and taken[0] <= taken[1] and me is not None
           and me[0] == os.getppid() and me[1] == oa.start_time(os.getpid()) and me[3] == os.sched_getaffinity(0),
           f"the listing's row of this process: {me} {taken}")
    # A process created after the placement is an observer by its parent:
    # confined (it inherited the mask) it passes; widened it fails.
    o = Observer()
    try:
        rec = root / "lineage.jsonl"
        expect(tool("confine", "--pids", str(o.pid), "--exclude-cpus", cpu_list(ex), "--record",
                    str(rec)).returncode == 0, "a clean confinement")
        late = o.child()
        probe = [(4242, 4243, ex), (4242, 4242, allowed)]
        snap = lineage_snapshot(root, {o.pid, *o.kids}, probe)
        r = tool("check", "--record", str(rec), "--tasks", str(snap), "--worker-tid", "4243")
        expect(r.returncode == 0 and "lineage: 3 observer processes" in r.stdout
               and "1 of them not in the record" in r.stdout, f"a new descendant, confined, passes: {r.stdout}")
        os.sched_setaffinity(late, full)
        snap = lineage_snapshot(root, {o.pid, *o.kids}, probe)
        r = tool("check", "--record", str(rec), "--tasks", str(snap), "--worker-tid", "4243")
        expect(r.returncode == 1 and "1 allowed on" in r.stdout and f"process {late} thread {late}" in r.stdout,
               f"a new descendant on the excluded CPU fails: {r.stdout}")
        expect(tool("restore", "--record", str(rec)).returncode == 0, "restored")
    finally:
        o.close()

    # Identities and the exempt trees, on a synthetic record and snapshot.
    def check(record: list[dict], rows: list[str], lineage: bool = True) -> subprocess.CompletedProcess:
        rp, tp = root / "syn.jsonl", root / "syn-tasks.txt"
        rp.write_text("".join(json.dumps(x) + "\n" for x in record))
        if lineage:
            tp.write_text("taken: x\n" + oa.LINEAGE + "\ntaken monotonic_ns: 1 2\n" + "\n".join(rows)
                          + f"\n{oa.COMPLETE} {len(rows)} tasks\n")
        else:
            tp.write_text("taken: x\n" + "\n".join(rows) + "\n")
        return tool("check", "--record", str(rp), "--tasks", str(tp), "--worker-tid", "4243")
    record = [{"kind": "header", "version": 2, "exclude": [3], "allowed": [0, 1, 2], "exempt": [6000],
               "exempt_ids": [{"pid": 6000, "start": 20}]},
              {"kind": "process", "pid": 5000, "pstart": 10, "ppid": 1, "n": 1},
              {"kind": "thread", "pid": 5000, "pstart": 10, "ppid": 1, "tid": 5000, "start": 10, "mask": [0, 1, 2, 3],
               "n": 1, "origin": "inventory"}]
    base = ["5000 5000 1 10 10 0-2",
            "6000 6000 5000 20 20 0-3",  # the session, run from an observer's tree: exempt
            "6001 6001 6000 40 40 0-3",  # its descendant: exempt too
            "4242 4243 6000 50 50 3"]    # the probe's worker
    r = check(record, base)
    expect(r.returncode == 0 and "1 observer threads, 0 allowed" in r.stdout, f"the exempt trees left out: {r.stdout}")
    r = check(record, base + ["5001 5001 5000 30 30 0-3"])
    expect(r.returncode == 1 and "process 5001 thread 5001" in r.stdout, f"a new descendant there fails: {r.stdout}")
    r = check(record, ["5000 5000 1 99 99 0-3", "5002 5002 5000 100 100 0-3", "4242 4243 1 50 50 3"])
    expect(r.returncode == 0 and "0 observer threads" in r.stdout,
           f"a reused process ID (another start time) and its child are not observers: {r.stdout}")
    # A record of the single-pass confine (version 1) and a snapshot before
    # the lineage: the recorded processes alone, said so.
    v1 = [{"kind": "header", "exclude": [3], "allowed": [0, 1, 2], "exempt": [6000]}, record[2]]
    r = check(v1, ["/proc/5000/task/5000/status 0-2", "/proc/5001/task/5001/status 0-3",
                   "/proc/4242/task/4243/status 3"], lineage=False)
    expect(r.returncode == 0 and "lineage: none in this snapshot" in r.stdout, f"a legacy snapshot: {r.stdout}")


class FakeProc:
    """A fake /proc tree with stand-in masks (no real mask changes): tasks
    {pid: (ppid, start, {tid: start})}; `denied` holds paths whose read or
    listing raises EACCES, and ("mask", tid) entries whose mask read does."""

    def __init__(self, root: pathlib.Path, tasks: dict) -> None:
        self.proc = root / "proc"
        self.masks: dict[int, set[int]] = {}
        self.sets: list[tuple] = []
        self.denied: set = set()
        for pid, (ppid, start, threads) in tasks.items():
            for tid, s in threads.items():
                self.thread(pid, ppid, tid, s)
            (self.proc / str(pid) / "stat").write_text(self.stat(pid, ppid, start))
            (self.proc / str(pid) / "comm").write_text("task\n")

    @staticmethod
    def stat(pid: int, ppid: int, start: int) -> str:
        return f"{pid} (task) " + " ".join(["S", str(ppid)] + ["0"] * 17 + [str(start)]) + "\n"

    def thread(self, pid: int, ppid: int, tid: int, start: int, mask: set[int] | None = None) -> None:
        d = self.proc / str(pid) / "task" / str(tid)
        d.mkdir(parents=True, exist_ok=True)
        (d / "stat").write_text(self.stat(tid, ppid, start))
        self.masks[tid] = set(mask if mask is not None else {0, 1, 2, 3})

    @contextlib.contextmanager
    def active(self):
        real_read, real_iter = pathlib.Path.read_text, pathlib.Path.iterdir
        fake = self

        def read(path, *a, **k):
            if path in fake.denied:
                raise PermissionError(errno.EACCES, "injected denial", str(path))
            return real_read(path, *a, **k)

        def iterdir(path):
            if path in fake.denied:
                raise PermissionError(errno.EACCES, "injected denial", str(path))
            return real_iter(path)

        def getaff(tid):
            if ("mask", tid) in fake.denied:
                raise PermissionError(errno.EACCES, "injected denial")
            if tid not in fake.masks:
                raise ProcessLookupError(errno.ESRCH, "No such process")
            return set(fake.masks[tid])

        def setaff(tid, mask):
            fake.sets.append((tid, set(mask)))
            fake.masks[tid] = set(mask)
        with patched(PROC=self.proc, os_sched_getaffinity=getaff, os_sched_setaffinity=setaff):
            pathlib.Path.read_text, pathlib.Path.iterdir = read, iterdir
            try:
                yield self
            finally:
                pathlib.Path.read_text, pathlib.Path.iterdir = real_read, real_iter


def read_failure_checks(root: pathlib.Path) -> None:
    """Unreadable is never gone (reviewer, 2026-11-09): an observer 100
    (threads 100, 101) and a probe 200 (its worker 200) on a fake /proc."""
    ex = {2, 3}
    full, confined = {0, 1, 2, 3}, {0, 1}
    case = {"n": 0}

    def fresh() -> FakeProc:
        case["n"] += 1
        d = root / f"case{case['n']}"
        d.mkdir()
        f = FakeProc(d, {100: (1, 10, {100: 10, 101: 11}), 200: (1, 20, {200: 20})})
        f.masks[200] = {3}
        return f

    def rec_path() -> pathlib.Path:
        return root / f"case{case['n']}" / "observers.jsonl"

    def confine(f: FakeProc) -> tuple:
        return capture(oa.confine, [100], ex, rec_path(), [])

    # A live thread's stat that cannot be read: confine fails before any
    # change (the reviewer's first case), and a restore has nothing to undo.
    f = fresh()
    f.denied.add(f.proc / "100" / "task" / "101" / "stat")
    with f.active():
        st, out = confine(f)
        expect(st == 1 and not f.sets and "1 failed" in out and "it cannot be read" in rec_path().read_text()
               and '"state": "confined with failures"' in rec_path().read_text(),
               f"an unreadable thread's identity fails confine, nothing changed: {st} {out} {f.sets}")
        f.denied.clear()
        expect(capture(oa.restore, rec_path())[0] == 0 and f.masks[100] == full, "nothing to restore")
    # A live process's stat: refused before the record (the trees unknown).
    f = fresh()
    f.denied.add(f.proc / "100" / "stat")
    with f.active():
        st, out = confine(f)
        expect(st == 2 and not rec_path().exists() and not f.sets and "cannot be listed" in out,
               f"an unreadable process refuses confine: {st} {out}")
    # A task directory that cannot be listed.
    f = fresh()
    f.denied.add(f.proc / "100" / "task")
    with f.active():
        st, out = confine(f)
        expect(st == 1 and not f.sets and "it cannot be read" in rec_path().read_text(),
               f"an unreadable task directory fails confine: {st} {out}")
    # A mask that cannot be read (the existing rule, on this fake).
    f = fresh()
    f.denied.add(("mask", 101))
    with f.active():
        st, out = confine(f)
        expect(st == 1 and not f.sets, f"an unreadable mask fails confine: {st} {out}")
    # A process's stat unreadable in a later pass: a failure after the first
    # pass's changes; restore then fails (the trees cannot be listed) and is
    # retryable, and its retry restores everything.
    f = fresh()
    calls: list[int] = []
    real_processes = oa.processes

    def deny_later():
        calls.append(1)
        if len(calls) == 3:  # the second pass's listing
            f.denied.add(f.proc / "100" / "stat")
        return real_processes()
    with f.active():
        with patched(processes=deny_later):
            st, out = confine(f)
        expect(st == 1 and f.masks[100] == confined and f.masks[101] == confined
               and "the trees cannot be listed" in rec_path().read_text(),
               f"a later pass's unreadable process fails confine: {st} {out}")
        st, out = capture(oa.restore, rec_path())
        expect(st == 1 and "INCOMPLETE" in out and "restore incomplete" in rec_path().read_text(),
               f"restore with an unreadable process is unresolved: {st} {out}")
        f.denied.clear()
        st, out = capture(oa.restore, rec_path())
        expect(st == 0 and f.masks[100] == full and f.masks[101] == full, f"and its retry restores: {st} {out}")
    # Restore with an intended thread's identity unreadable (the reviewer's
    # second case): unresolved, never "exited"; the retry restores it.
    f = fresh()
    with f.active():
        expect(confine(f)[0] == 0 and f.masks[101] == confined, "a clean confinement on the fake")
        f.denied.add(f.proc / "100" / "task" / "101" / "stat")
        st, out = capture(oa.restore, rec_path())
        expect(st == 1 and "exited 0" in out and f.masks[101] == confined and f.masks[100] == full
               and '"state": "restore incomplete"' in rec_path().read_text(),
               f"an unreadable intended identity is unresolved: {st} {out}")
        f.denied.clear()
        st, out = capture(oa.restore, rec_path())
        expect(st == 0 and f.masks[101] == full, f"and its retry restores it: {st} {out}")
    # Restore with a recorded process's task directory unreadable (threads
    # created during the session cannot be listed): unresolved, retryable.
    f = fresh()
    with f.active():
        expect(confine(f)[0] == 0, "a clean confinement on the fake")
        f.thread(100, 1, 102, 12, confined)  # created during the session, inherited the confined mask
        f.denied.add(f.proc / "100" / "task")
        st, out = capture(oa.restore, rec_path())
        expect(st == 1 and f.masks[102] == confined, f"an unreadable task directory leaves restore unresolved: {out}")
        f.denied.clear()
        st, out = capture(oa.restore, rec_path())
        expect(st == 0 and f.masks[102] == full and "new restored 1" in out, f"and its retry restores: {st} {out}")

    # The listing: complete, then each denial makes it incomplete (status 1),
    # and the check refuses it (the reviewer's third case).
    f = fresh()
    with f.active():
        expect(confine(f)[0] == 0, "a clean confinement on the fake")
        snap = root / f"case{case['n']}" / "tasks.txt"

        def listing() -> tuple:
            o = io.StringIO()
            with contextlib.redirect_stderr(io.StringIO()):
                st = oa.tasks(o)
            snap.write_text("taken: x\n" + o.getvalue())
            return st, o.getvalue()
        st, text = listing()
        expect(st == 0 and text.splitlines()[-1] == f"{oa.COMPLETE} 3 tasks" and "100 101 1 10 11 0-1" in text,
               f"a complete listing: {st} {text}")
        st, out = capture(oa.check, rec_path(), snap, 200)
        expect(st == 0 and "2 observer threads, 0 allowed" in out, f"which the check takes: {st} {out}")
        for denial in (f.proc / "100" / "stat", f.proc / "100" / "task", f.proc / "100" / "task" / "101" / "stat",
                       ("mask", 101)):
            f.denied = {denial}
            st, text = listing()
            expect(st == 1 and text.splitlines()[-1].startswith(f"{oa.INCOMPLETE} 1 unreadable"),
                   f"an unreadable {denial} makes the listing incomplete: {st} {text}")
            f.denied = set()
            st, out = capture(oa.check, rec_path(), snap, 200)
            expect(st == 1 and "task lineage is incomplete" in out, f"and the check refuses it: {st} {out}")
        # A listing that did not finish (no completeness line) is refused too.
        st, text = listing()
        snap.write_text("taken: x\n" + "\n".join(text.splitlines()[:-1]) + "\n")
        st, out = capture(oa.check, rec_path(), snap, 200)
        expect(st == 1 and "no completeness line" in out, f"a truncated listing is refused: {st} {out}")

    # Tasks that exit during a pass stay benign: a thread whose entry is gone
    # (its directory listed, its stat missing), one whose mask read finds it
    # gone (ESRCH), and a process directory without its stat.
    f = fresh()
    (f.proc / "100" / "task" / "103").mkdir()
    f.thread(100, 1, 104, 14)
    del f.masks[104]
    (f.proc / "300").mkdir()
    with f.active():
        st, out = confine(f)
        e = entries(rec_path())
        expect(st == 0 and {x["tid"] for x in e if x["kind"] == "thread"} == {100, 101}
               and f.masks[101] == confined, f"tasks gone during a pass are left out: {st} {out}")
        o = io.StringIO()
        expect(oa.tasks(o) == 0 and o.getvalue().splitlines()[-1] == f"{oa.COMPLETE} 3 tasks",
               f"and from the listing: {o.getvalue()}")
        expect(capture(oa.restore, rec_path())[0] == 0 and f.masks[101] == full, "restored")


def main() -> int:
    mine = sorted(os.sched_getaffinity(0))
    if len(mine) < 3:
        print("observer affinity regressions OK (not run: fewer than three CPUs)")
        return 0
    ex = mine[-1]
    allowed = set(mine) - {ex}
    procs: list[subprocess.Popen] = []
    kids: list[int] = []  # the observers' children, killed at the end
    try:
        with tempfile.TemporaryDirectory() as t:
            root = pathlib.Path(t)
            obs = subprocess.Popen([sys.executable, "-c", CHILD], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   text=True)
            procs.append(obs)
            pid, kid = start(obs)
            kids.append(kid)
            time.sleep(0.2)
            before = {**masks(pid), **masks(kid)}
            expect(all(m == set(mine) for m in before.values()), "the observer starts unconfined")
            # One thread pinned to the excluded CPU alone, another changed later.
            ts = tids(pid)
            os.sched_setaffinity(ts[1], {ex})
            rec = root / "r.jsonl"
            r = tool("confine", "--pids", str(pid), "--exclude-cpus", str(ex), "--record", str(rec))
            expect(r.returncode == 1 and "1 left (only excluded CPUs)" in r.stdout,
                   f"a thread on the excluded CPU only is left: {r.stdout} {r.stderr}")
            lines = [json.loads(x) for x in rec.read_text().splitlines()]
            kinds = [x["kind"] for x in lines]
            expect(kinds[0] == "header" and kinds.index("state") > max(i for i, k in enumerate(kinds) if k == "thread")
                   and kinds[-1] == "state" and lines[-1]["state"] == "confined with failures",
                   f"the record: header, every thread, then the changes: {kinds}")
            th = [x for x in lines if x["kind"] == "thread"]
            expect({x["tid"] for x in th} == set(tids(pid)) | set(tids(kid)) and all(x["start"] for x in th),
                   "every thread of the tree is recorded with its start time")
            now = {**masks(pid), **masks(kid)}
            expect(all(m == allowed for t_, m in now.items() if t_ != ts[1]) and now[ts[1]] == {ex},
                   f"the threads confined: {now}")
            # Refusals.
            expect(tool("confine", "--pids", str(pid), "--exclude-cpus", str(ex), "--record", str(rec)).returncode == 2,
                   "an existing record is refused")
            expect(tool("confine", "--pids", "999999999", "--exclude-cpus", str(ex),
                        "--record", str(root / "x.jsonl")).returncode == 2, "a missing process is refused")
            r = tool("confine", "--pids", str(kid), "--exclude-cpus", str(ex), "--record", str(root / "y.jsonl"),
                     "--exempt-pid", str(pid))
            expect(r.returncode == 2 and "exempt tree" in r.stderr, "a process in an exempt tree is refused")
            # During the session: a new thread, a new child, a mask changed by the application.
            expect(say(obs, "thread") == "ok", "a new thread")
            grandkid = int(say(obs, "child"))
            kids.append(grandkid)
            os.sched_setaffinity(ts[0], {mine[0]})
            new_tids = set(tids(pid)) - {x["tid"] for x in th}
            expect(len(new_tids) == 1 and os.sched_getaffinity(new_tids.pop()) == allowed,
                   "the new thread inherited the confined mask")
            r = tool("restore", "--record", str(rec))
            after = masks(pid)
            expect(r.returncode == 0 and after[ts[0]] == {mine[0]} and "changed 1" in r.stdout,
                   f"a mask changed during the session is left: {r.stdout}")
            # The observer's recorded threads had two masks (one was on the
            # excluded CPU alone), so its new thread and its new child (whose
            # nearest recorded process is the observer) are left; the existing
            # child's recorded threads are restored.
            expect("new left 2" in r.stdout and "new restored 0" in r.stdout, f"no single original mask: {r.stdout}")
            expect(all(after[t_] == set(mine) for t_ in tids(pid) if t_ in {x["tid"] for x in th} and t_ != ts[0]
                       and t_ != ts[1]), f"the recorded threads restored: {after}")
            expect(masks(kid) == {t_: set(mine) for t_ in tids(kid)}, "the child restored")
            expect(os.sched_getaffinity(grandkid) == allowed, "the new process left at its mask")
            r = tool("restore", "--record", str(rec))
            expect(r.returncode == 0, "a repeated restore")

            # A process whose threads shared one mask: its new thread and new
            # child get it back; a reused ID is left.
            obs2 = subprocess.Popen([sys.executable, "-c", CHILD], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    text=True)
            procs.append(obs2)
            pid2, kid2 = start(obs2)
            kids.append(kid2)
            time.sleep(0.2)
            rec2 = root / "r2.jsonl"
            r = tool("confine", "--pids", str(pid2), "--exclude-cpus", str(ex), "--record", str(rec2))
            expect(r.returncode == 0, f"a clean confinement: {r.stdout} {r.stderr}")
            # The run's task snapshot: the observer's threads as confined, the
            # probe's worker on the excluded CPU and a helper off it pass; a
            # helper or an observer thread allowed there fails.
            # (The snapshot's lineage, as the session lists it, with a stand-in
            # probe: its worker 4243 and a helper 4242.)
            def snapshot(helper: set[int], override: dict | None = None) -> pathlib.Path:
                return lineage_snapshot(root, {pid2, kid2}, [(4242, 4243, {ex}), (4242, 4242, helper)], override)
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(allowed)), "--worker-tid", "4243")
            expect(r.returncode == 0 and "0 allowed on" in r.stdout and "1 probe helper threads, 0 allowed" in r.stdout
                   and "lineage: 2 observer processes" in r.stdout and "0 of them not in the record" in r.stdout,
                   f"a confined run's snapshot passes: {r.stdout} {r.stderr}")
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(set(mine))), "--worker-tid", "4243")
            expect(r.returncode == 1 and "1 probe helper threads, 1 allowed there" in r.stdout,
                   f"a helper on the excluded CPU fails: {r.stdout}")
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(allowed, {(pid2, pid2): set(mine)})),
                     "--worker-tid", "4243")
            expect(r.returncode == 1 and "1 allowed on" in r.stdout, f"an observer thread there fails: {r.stdout}")
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(allowed, {(pid2, pid2): None})),
                     "--worker-tid", "4243")
            expect(r.returncode == 1 and "allowed CPUs unreadable" in r.stdout, f"an unreadable mask fails: {r.stdout}")
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(allowed)), "--worker-tid", "1")
            expect(r.returncode == 1 and "not in the snapshot" in r.stderr, "a missing worker fails")
            # This confine's record requires the lineage; a snapshot without it fails.
            legacy = root / "legacy-tasks.txt"
            legacy.write_text("taken: x\n" + "".join(f"/proc/{p_}/task/{t_}/status {cpu_list(os.sched_getaffinity(t_))}\n"
                                                     for p_ in (pid2, kid2) for t_ in tids(p_))
                              + f"/proc/4242/task/4243/status {ex}\n")
            r = tool("check", "--record", str(rec2), "--tasks", str(legacy), "--worker-tid", "4243")
            expect(r.returncode == 1 and "no task lineage, which this placement record requires" in r.stderr,
                   f"a snapshot without the lineage fails: {r.stdout} {r.stderr}")
            expect(say(obs2, "thread") == "ok", "a new thread")
            gk2 = int(say(obs2, "child"))
            kids.append(gk2)
            # Fake a reused ID for the child's main thread in the record (its
            # thread and change records, keyed by ID and start time): the
            # recorded thread is reported reused, and the thread holding the
            # ID now is another, a new thread of its process, given the
            # process's original (it holds the confined form).
            entries = [json.loads(x) for x in rec2.read_text().splitlines()]
            for x in entries:
                if x.get("tid") == kid2 and "start" in x:
                    x["start"] = 1
            rec2.write_text("".join(json.dumps(x, sort_keys=True) + "\n" for x in entries))
            r = tool("restore", "--record", str(rec2))
            expect(r.returncode == 0 and "new restored 3" in r.stdout and "reused 1" in r.stdout,
                   f"new threads with a single original mask, and a reused ID: {r.stdout}")
            expect(all(m == set(mine) for m in masks(pid2).values()) and os.sched_getaffinity(gk2) == set(mine),
                   f"the new thread and process restored: {masks(pid2)} {os.sched_getaffinity(gk2)}\n{r.stdout}\n"
                   f"{rec2.read_text()}")
            expect(os.sched_getaffinity(kid2) == set(mine), "the ID's present thread treated as a new one")
        with tempfile.TemporaryDirectory() as t:
            fault_checks(pathlib.Path(t), mine)
        with tempfile.TemporaryDirectory() as t:
            repeated_scan_checks(pathlib.Path(t), mine)
        with tempfile.TemporaryDirectory() as t:
            lineage_checks(pathlib.Path(t), mine)
        with tempfile.TemporaryDirectory() as t:
            counterexample_checks(pathlib.Path(t))
        with tempfile.TemporaryDirectory() as t:
            read_failure_checks(pathlib.Path(t))
    finally:
        for p in procs:
            p.kill()
            p.wait()
        for k in kids:
            try:
                os.kill(k, 9)
            except OSError:
                pass
    print("observer affinity regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
