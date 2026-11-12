#!/usr/bin/env python3
"""Whether the emulator stayed off the real-time CPU during a timer capture, for the loaded emulator validation's Option A
second run (docs/hardware/control-rate.md): launched with an affinity without that CPU, and checked so by the supervisor
(tools/option_a.sh), none of its tasks may run there or wait on its run queue. Read-only.

    emulator_residency.py <run>.csv --experiment FILE [--lineage FILE] [--report FILE] [--rt-cpu N]

The emulator's identity comes from the supervisor's experiment record: its emulator_start=, emulator_A= and emulator_B=
lines, each "TIME pid=P start=S comm=C tids=T,... allowed=L;..." (every thread's Cpus_allowed_list). All three must be
there, name the same pid and start time, and give every thread a well-formed list without the CPU; otherwise the record
is unreadable (an emulator check that failed is the supervisor's stop, not this tool's question). They are affinity
checks at three moments, not a census of the emulator's tasks over the run.

Its tasks come from the session's task lineage right after the trace started (<run>.csv.lineage.txt, from
tools/observer_affinity.py tasks; or --lineage FILE): the emulator's process (its pid and start time) and its
descendants by parent, every thread of each. The lineage is read strictly (read_lineage): its header, one taken line
(began, ended; not ending before it began), rows each listed once, with one process per thread and one parent and
start time per process, and a completeness line, last, whose count is the rows'; it must list the emulator, begin after
the trace's first event and end before the window began. Anything else makes the evidence unreadable. To the lineage's
tasks are added the threads the record names and every task the trace shows exiting (sched_process_exit) before the
lineage ended, which the listing cannot have seen; then every task any of these created, through any number of
generations, from all the trace's forks (sched_process_fork) up to the window's end, whatever their order: a parent
known only by its exit still brings the children it made before it, even a child the lineage shows reparented. Every
task's events are gathered first and judged against that final set. The set only grows: a reused ID stays the
emulator's (at worst a false failure, never a false pass). A task created before the trace and gone before the lineage
was taken cannot run in the window. Not covered: a descendant process created before the trace whose parent process
(itself a descendant) ended before the lineage, so that the lineage shows it reparented and no fork in the trace
connects it (the supervisor's records name the leader's threads only).

Inside the probe's window (its first wake stamp to its last cycle-end stamp, as tools/unbound_work.py takes it), from the
main trace (trace-cmd report -t under tools/tk_record.py's launch, or a saved --report):

- occupancy: the real-time CPU's tasks from its sched_switch chain, from the one running when the window began (the
  last switch onto the CPU at or before it; none recorded leaves it unknown, and the evidence unreadable) to the window's
  end; no stretch of an emulator task there may overlap the window (one carried into the window counts);
- placement: each emulator task's place from its own events since the trace's start (woken onto a CPU, migrated to
  one, switched in or preempted there: on that CPU, queued or running; switched out asleep: on none); no time on the
  real-time CPU may overlap the window (a task queued there when the window began counts), and no migration of one off
  that CPU may happen inside the window (its CPU was the real-time one, whatever the trace showed of it before). A task
  with no event from the trace's start to the window's end is placed nowhere by the trace;
- lineage affinity: no emulator thread in the lineage allowed on the real-time CPU;
- seen: at least one switch to an emulator task in the window, on any CPU (else nothing establishes where it ran).

The evidence must be complete: every sched_switch, sched_wakeup, sched_migrate_task, sched_process_fork and
sched_process_exit must parse, the real-time CPU's switches must follow on from each other (each switch's previous task
is the one the last switch put there), an emulator task's events must follow from its known place (switched out where
it was switched in; switched in, woken or migrated from where it was queued), every event line after the first must
be an event line, the report must reach
past the window's end, and it is read to its end: a decoder that exits nonzero, at any point, makes the evidence
unreadable. Descriptive: the CPUs the emulator's tasks were switched to, and their wake-to-run waits.

Exit: 0 every check passed; 1 a check failed; 2 the evidence could not be read.
"""

from __future__ import annotations

import argparse
import collections
import os
import pathlib
import re
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import observer_affinity  # noqa: E402
import tk_cost  # noqa: E402
import tk_record  # noqa: E402
import trace_correlate  # noqa: E402

WAKEUP = re.compile(r"^(?P<comm>.*):(?P<pid>\d+) \[-?\d+\] CPU:(?P<cpu>\d+)$")
MIGRATE = re.compile(r"\bpid=(?P<pid>\d+) prio=-?\d+ orig_cpu=(?P<orig>\d+) dest_cpu=(?P<dest>\d+)$")
FORK = re.compile(r"\bpid=(?P<pid>\d+) child_comm=.* child_pid=(?P<child>\d+)$")
EXIT = re.compile(r"\bpid=(?P<pid>\d+) prio=-?\d+(?: group_dead=\w+)?$")
RECORD = re.compile(r"^(?P<t>\S+) pid=(?P<pid>\d+) start=(?P<start>\d+) comm=(?P<comm>\S+) tids=(?P<tids>[\d,]+) "
                    r"allowed=(?P<allowed>[\d,;-]+)$")
EVENTS = {"sched_switch", "sched_wakeup", "sched_migrate_task", "sched_process_fork", "sched_process_exit"}
PREEMPTED = re.compile(r"^R\+?$")  # a switch-out that leaves the task runnable where it was


class Unreadable(Exception):
    pass


def cpus(spec: str) -> set:
    """A Cpus_allowed_list as a set; a malformed list (a descending range included) raises ValueError."""
    out: set = set()
    for part in spec.split(","):
        a, sep, b = part.partition("-")
        if not a.isdigit() or (sep and not b.isdigit()):
            raise ValueError(spec)
        lo, hi = int(a), (int(b) if sep else int(a))
        if hi < lo:
            raise ValueError(spec)
        out.update(range(lo, hi + 1))
    return out


def emulator(record: pathlib.Path, rt: int) -> dict:
    """The emulator's pid, start time and recorded threads from the experiment record; raises Unreadable."""
    try:
        lines = record.read_text().splitlines()
    except OSError as e:
        raise Unreadable(f"the experiment record {record}: {e}") from None
    seen: dict = {}
    for ln in lines:
        k, sep, v = ln.partition("=")
        if k not in ("emulator_start", "emulator_A", "emulator_B"):
            continue
        if k in seen:
            raise Unreadable(f"{record}: {k} twice")
        m = RECORD.match(v)
        if not m:
            raise Unreadable(f"{record}: {k} is no emulator state (a failed check?): {v!r}")
        tids, lists = m["tids"].split(","), m["allowed"].split(";")
        if len(tids) != len(lists):
            raise Unreadable(f"{record}: {k}: {len(tids)} threads, {len(lists)} affinity lists")
        try:
            sets = [cpus(x) for x in lists]
        except ValueError as e:
            raise Unreadable(f"{record}: {k}: a malformed affinity list {e}") from None
        if any(rt in s or not s for s in sets):
            raise Unreadable(f"{record}: {k}: a thread may run on CPU {rt} (or on no CPU): {m['allowed']}")
        seen[k] = (int(m["pid"]), m["start"], {int(t) for t in tids})
    missing = [k for k in ("emulator_start", "emulator_A", "emulator_B") if k not in seen]
    if missing:
        raise Unreadable(f"{record}: no {', '.join(missing)}")
    ids = {(pid, start) for pid, start, _ in seen.values()}
    if len(ids) != 1:
        raise Unreadable(f"{record}: the emulator's identity changes: {sorted(ids)}")
    pid, start = ids.pop()
    return dict(pid=pid, start=start, tids=set().union(*(t for _, _, t in seen.values())))


TAKEN = re.compile(r"^taken monotonic_ns: (\d+) (\d+)$")
ROW = re.compile(r"^(\d+) (\d+) (\d+) (\d+) (\d+) (\S+)$")
FOOTER = re.compile(r"^lineage complete: (\d+) tasks$")


def read_lineage(path: pathlib.Path) -> tuple[dict, tuple[int, int]]:
    """A task lineage file as tools/observer_affinity.py tasks writes it, read strictly: its header, its taken line
    (two monotonic stamps, the second not before the first), its rows ("pid tid ppid pstart start allowed", each
    (pid, tid) once, each thread in one process, each process with one parent and start time, its leader's start its
    start, no thread started before its process, a well-formed nonempty affinity), and its completeness line, whose
    count is the rows', last. Anything else (an incomplete listing's line among it) raises Unreadable. Returns
    {(pid, tid): (ppid, pstart, start, allowed)} and (began, ended)."""
    try:
        lines = path.read_text().split("\n")
    except (OSError, UnicodeDecodeError) as e:
        raise Unreadable(f"no task lineage after the trace started: {path}: {e}") from None
    if lines and lines[-1] == "":
        lines.pop()  # the final newline
    if len(lines) < 3 or lines[0] != observer_affinity.LINEAGE:
        raise Unreadable(f"{path}: no task lineage (its header, taken line and completeness line)")
    m = TAKEN.match(lines[1])
    if not m:
        raise Unreadable(f"{path}: the task lineage's second line is no taken line: {lines[1][:80]!r}")
    taken = (int(m[1]), int(m[2]))
    if taken[1] < taken[0]:
        raise Unreadable(f"{path}: the task lineage ended before it began: {taken}")
    f = FOOTER.match(lines[-1])
    if not f:
        raise Unreadable(f"{path}: the task lineage's last line is no completeness line: {lines[-1][:80]!r}")
    rows: dict = {}
    procs: dict = {}            # pid -> (ppid, pstart)
    owner: dict = {}            # tid -> pid
    for n, ln in enumerate(lines[2:-1], start=3):
        r = ROW.match(ln)
        if not r:
            raise Unreadable(f"{path}: line {n} is no task row: {ln[:80]!r}")
        pid, tid, ppid, pstart, start = (int(r[i]) for i in range(1, 6))
        try:
            allowed = cpus(r[6])
        except ValueError:
            raise Unreadable(f"{path}: line {n}: a malformed affinity list {r[6]!r}") from None
        if (pid, tid) in rows:
            raise Unreadable(f"{path}: line {n}: task {pid}/{tid} listed twice")
        if owner.setdefault(tid, pid) != pid:
            raise Unreadable(f"{path}: line {n}: thread {tid} listed in processes {owner[tid]} and {pid}")
        if procs.setdefault(pid, (ppid, pstart)) != (ppid, pstart):
            raise Unreadable(f"{path}: line {n}: process {pid} listed with two parents or start times")
        if (tid == pid and start != pstart) or start < pstart:
            raise Unreadable(f"{path}: line {n}: thread {tid}'s start {start} contradicts its process's {pstart}")
        rows[(pid, tid)] = (ppid, pstart, start, allowed)
    if int(f[1]) != len(rows):
        raise Unreadable(f"{path}: the completeness line counts {f[1]} tasks; the lineage lists {len(rows)}")
    return rows, taken


def lineage(path: pathlib.Path, emu: dict, rt: int) -> dict:
    """The emulator's tasks in the session's lineage after the trace started (read_lineage): its process (by pid and
    start time) and its descendants by parent, every thread of each; when the lineage was taken (monotonic ns, its
    start and end); and those threads allowed on the real-time CPU. Raises Unreadable for a lineage that is missing,
    malformed or without the emulator."""
    rows, taken = read_lineage(path)
    parents = {p: pp for (p, _), (pp, _, _, _) in rows.items()}
    starts = {p: ps for (p, _), (_, ps, _, _) in rows.items()}
    if starts.get(emu["pid"]) != int(emu["start"]):
        raise Unreadable(f"{path}: the emulator (pid {emu['pid']}, start {emu['start']}) is not in the task lineage"
                         + (f" (pid {emu['pid']} has start {starts[emu['pid']]})" if emu["pid"] in starts else ""))
    procs = observer_affinity.tree({emu["pid"]}, parents)
    tasks = {t: allowed for (p, t), (_, _, _, allowed) in rows.items() if p in procs}
    return dict(seeds=set(tasks), procs=procs, taken=taken, allowed_rt=sorted(t for t, a in tasks.items() if rt in a))


def closure(base: set, forks: list) -> set:
    """The tasks of base and every task one of them created, through any number of generations, whatever the order in
    which the trace showed the forks and the parents became known (a parent known only by its exit still brings the
    children it made before it)."""
    kids = collections.defaultdict(set)
    for _, parent, child in forks:
        kids[parent].add(child)
    out, todo = set(base), list(base)
    while todo:
        for child in kids.get(todo.pop(), ()):
            if child not in out:
                out.add(child)
                todo.append(child)
    return out


def analyse(lines, cap, emu: dict, rt: int, out) -> dict:
    """The checks above on report lines (read to their end), for the emulator of emulator() with its lineage();
    raises Unreadable on incomplete evidence. Every task's evidence is gathered (the idle tasks, ID 0, aside) and judged
    once the emulator's tasks are known: the lineage's, the record's and the tasks that exited before the lineage
    ended, with the closure of every fork in the trace up to the window's end."""
    if "seeds" not in emu or "taken" not in emu:
        raise Unreadable("no task lineage after the trace started")
    lo, hi = cap.window
    t0, t1 = emu["taken"]
    first = last = None
    unparsed = malformed = inconsistent = 0
    occupant = None             # (comm, pid, since) on the real-time CPU
    occupant_at_lo = None
    stretches: list = []        # (tid, from, to) every task's stretch on the real-time CPU overlapping the window
    spans: list = []            # (tid, from, to) every task's time on it, queued or running, overlapping the window
    moved_off: list = []        # (ts, tid) every task's migration off it inside the window
    place: dict = {}            # tid -> (cpu, since) while queued or running there, None while asleep; absent: unknown
    contradicted = collections.Counter()  # tid -> its events that do not follow from its known place
    woken_rt = collections.Counter()      # tid -> its wakeups onto the real-time CPU inside the window (descriptive)
    moved_rt = collections.Counter()      # tid -> its migrations onto it inside the window (descriptive)
    ran = collections.defaultdict(collections.Counter)  # tid -> CPU -> its switch-ins inside the window
    forks: list = []            # (ts, parent, child) every fork up to the window's end
    exited: set = set()         # every task seen exiting before the lineage ended
    inside_set = None           # the emulator's tasks, once the window began (for the descriptive waits)
    waits: list = []
    woken: dict = {}            # tid -> wakeup ts inside the window, awaiting its switch-in

    def on(tid: int) -> int | None:
        """The CPU a task is known to be queued or running on (None: asleep or unknown)."""
        p = place.get(tid)
        return p[0] if p else None

    def leave(tid: int, ts: int) -> None:
        p = place.get(tid)
        if p and p[0] == rt and ts >= lo and p[1] <= hi:
            spans.append((tid, p[1], ts))

    def put(tid: int, cpu: int | None, ts: int) -> None:
        if cpu is not None and on(tid) == cpu:
            return
        leave(tid, ts)
        place[tid] = (cpu, ts) if cpu is not None else None

    for ln in lines:
        ln = ln.rstrip("\n")
        m = trace_correlate.LINE.match(ln)
        if not m:
            if first is not None and ln.strip():
                unparsed += 1
            continue
        ts = int(m["s"]) * 1_000_000_000 + int(m["ns"])
        if first is None:
            first = ts
        last = ts
        ev, rest, cpu = m["ev"], m["rest"], int(m["cpu"])
        if ev not in EVENTS or ts > hi:
            continue
        inside = ts >= lo
        if inside and inside_set is None:
            inside_set = closure(set(emu["tids"]) | set(emu["seeds"]) | exited, forks)
        if ev == "sched_process_fork":
            f = FORK.search(rest)
            if not f:
                malformed += 1
                continue
            parent, child = int(f["pid"]), int(f["child"])
            forks.append((ts, parent, child))
            place.pop(child, None)  # a new task: a reused ID starts afresh
            woken.pop(child, None)
            if inside_set is not None and parent in inside_set:
                inside_set.add(child)
        elif ev == "sched_process_exit":
            x = EXIT.search(rest)
            if not x:
                malformed += 1
                continue
            if ts <= t1:
                exited.add(int(x["pid"]))
        elif ev == "sched_switch":
            s = trace_correlate.SWITCH.match(rest)
            if not s:
                malformed += 1
                continue
            ppid, npid = int(s["ppid"]), int(s["npid"])
            if cpu == rt:
                if occupant is not None and ppid != occupant[1]:
                    inconsistent += 1
                if occupant is not None and occupant[1] != 0 and ts >= lo:
                    stretches.append((occupant[1], occupant[2], ts))
                occupant = (s["next"], npid, ts)
                if ts <= lo:
                    occupant_at_lo = occupant
            if ppid != 0:  # it ran here: it was switched in here, unless its place is unknown
                if ppid in place and on(ppid) != cpu:
                    contradicted[ppid] += 1
                put(ppid, cpu if PREEMPTED.match(s["state"]) else None, ts)
            if npid != 0:  # queued here, or (a wakeup's place is its only source) asleep or unknown
                if on(npid) not in (None, cpu):
                    contradicted[npid] += 1
                put(npid, cpu, ts)
                if inside:
                    ran[npid][cpu] += 1
                    if npid in woken:
                        w0 = woken.pop(npid)
                        if inside_set is not None and npid in inside_set:
                            waits.append(ts - w0)
        elif ev == "sched_wakeup":
            w = WAKEUP.match(rest)
            if not w:
                malformed += 1
                continue
            pid, to = int(w["pid"]), int(w["cpu"])
            if pid != 0:
                if on(pid) not in (None, to):  # a queued or running task is woken where it is
                    contradicted[pid] += 1
                put(pid, to, ts)
                if inside:
                    woken[pid] = ts
                    woken_rt[pid] += to == rt
        else:
            g = MIGRATE.search(rest)
            if not g:
                malformed += 1
                continue
            pid, orig, dest = int(g["pid"]), int(g["orig"]), int(g["dest"])
            if pid != 0:
                if on(pid) not in (None, orig):  # a queued task moves from where it is
                    contradicted[pid] += 1
                if inside and orig == rt:  # its CPU was the real-time one, whatever the trace showed of it before
                    moved_off.append((ts, pid))
                if on(pid) is not None or dest == rt:  # a queued task moves; a sleeper's wakeup that follows places it
                    put(pid, dest, ts)
                moved_rt[pid] += inside and dest == rt
    if last is None or last < hi:
        raise Unreadable(f"the report ends at {last} ns, before the window's end ({hi} ns)")
    if not first < t0:
        raise Unreadable(f"the task lineage began at {t0} ns, not after the trace's first event ({first} ns)")
    if not t1 < lo:
        raise Unreadable(f"the task lineage ended at {t1} ns, not before the window began ({lo} ns)")
    if occupant is not None and occupant[1] != 0:
        stretches.append((occupant[1], occupant[2], hi))
    for tid in list(place):
        leave(tid, hi)
    # The emulator's tasks: the lineage's, the record's and those that exited before the lineage ended, with every
    # task any of them created (the closure of the trace's forks); the set only grows.
    base = set(emu["tids"]) | set(emu["seeds"]) | exited
    tids = closure(base, forks)
    created = tids - base
    contra = sum(contradicted[t] for t in tids)
    if unparsed or malformed or inconsistent or contra:
        raise Unreadable(f"{malformed} malformed required events, {unparsed} lines that are no events, {inconsistent} switches of CPU {rt} "
                         f"that do not follow on from the previous one, {contra} events of the emulator's tasks that do not "
                         "follow from where they were")
    if occupant_at_lo is None:
        raise Unreadable(f"the task on CPU {rt} when the window began is unknown (no switch onto it recorded before the window)")
    ran_rt = [x for x in stretches if x[0] in tids]
    placed_rt = [x for x in spans if x[0] in tids]
    off = [x for x in moved_off if x[1] in tids]
    cpus_ran = collections.Counter()
    for t in tids:
        cpus_ran.update(ran.get(t, {}))
    seen = sum(cpus_ran.values())
    n_woken = sum(woken_rt[t] for t in tids)
    n_moved = sum(moved_rt[t] for t in tids)
    only_exited = exited - set(emu["tids"]) - set(emu["seeds"])
    print(f"window: {lo} to {hi} ns ({(hi - lo) / 1e9:.6f} s); real-time CPU {rt}", file=out)
    print(f"  the emulator: pid {emu['pid']} (start {emu['start']}); its tasks: {len(emu['seeds'])} in the lineage after the trace started "
          f"({(t0 - lo) / 1e9:+.3f} to {(t1 - lo) / 1e9:+.3f} s), {len(set(emu['tids']) - set(emu['seeds']))} more in the record, "
          f"{len(only_exited)} other tasks counted as its, exited before the lineage ended, "
          + (f"{len(created)} created by these through the trace's forks ({', '.join(map(str, sorted(created)[:5]))})" if created
             else "none created by these through the trace's forks"), file=out)
    print(f"  on CPU {rt} when the window began: {occupant_at_lo[0]} ({occupant_at_lo[1]}), since {occupant_at_lo[2]} ns", file=out)
    print(f"  switches to it in the window: {seen}" + (" (CPUs: " + ", ".join(f"{c} {n}" for c, n in sorted(cpus_ran.items())) + ")" if seen else ""), file=out)
    print(f"  its stretches on CPU {rt} overlapping the window: {len(ran_rt)}" + (" (" + ", ".join(
        f"{t} {(a - lo) / 1e6:+.3f}..{(b - lo) / 1e6:+.3f} ms" for t, a, b in ran_rt[:5]) + ")" if ran_rt else ""), file=out)
    print(f"  its time on CPU {rt} (queued or running) overlapping the window: {len(placed_rt)}" + (" (" + ", ".join(
        f"{t} {(a - lo) / 1e6:+.3f}..{(b - lo) / 1e6:+.3f} ms" for t, a, b in placed_rt[:5]) + ")" if placed_rt else "")
          + f"; inside the window, wakeups of it onto CPU {rt} {n_woken}, migrations of it onto CPU {rt} {n_moved}", file=out)
    print(f"  migrations of it off CPU {rt} inside the window: {len(off)}" + (" (" + ", ".join(
        f"{t} {(a - lo) / 1e6:+.3f} ms" for a, t in off[:5]) + ")" if off else ""), file=out)
    print(f"  its threads in the lineage allowed on CPU {rt}: {len(emu['allowed_rt'])}"
          + (f" ({', '.join(map(str, emu['allowed_rt'][:10]))})" if emu["allowed_rt"] else ""), file=out)
    if waits:
        print(f"  wake to run (descriptive): {tk_cost.dist(sorted(waits))}", file=out)
    if not seen:
        print("  the emulator was not seen to run in the window: nothing establishes where it ran", file=out)
    ok = seen > 0 and not ran_rt and not placed_rt and not off and not emu["allowed_rt"]
    print(f"emulator residency: {'PASS' if ok else 'FAIL'}", file=out)
    return dict(ok=ok, seen=seen, occupant=occupant_at_lo[1], ran_rt=len(ran_rt), placed_rt=len(placed_rt), moved_off=len(off),
                woken_rt=n_woken, moved_rt=n_moved, allowed_rt=len(emu["allowed_rt"]), forks=len(created), exited=len(only_exited),
                tasks=len(tids), cpus=dict(cpus_ran), waits=waits)


def run_report(csv_path: pathlib.Path, cap, emu: dict, rt: int, out) -> dict:
    """analyse() over trace-cmd report -t under the launch, read to its end; the decoder's exit is always collected."""
    decoder = os.environ.get("EMULATOR_RESIDENCY_DECODER") if os.environ.get("EMULATOR_RESIDENCY_TEST") == "1" else None
    if decoder is None:
        tool = tk_record.tool_identity("trace-cmd")
        if tool.get("read_status") != "ok" or not tool.get("path"):
            raise Unreadable(f"trace-cmd is not identified ({tool.get('read_status')})")
        decoder = tool["path"]
    proc = subprocess.Popen([decoder, "report", "-t", "-i", f"{csv_path}.trace.dat"], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, errors="replace", env=tk_record.launch_env())
    assert proc.stdout is not None
    try:
        r = analyse(proc.stdout, cap, emu, rt, out)
    finally:
        if proc.poll() is None:
            for _ in proc.stdout:  # drain to the end, so that its exit is its own
                pass
        proc.stdout.close()
        status = proc.wait()
    if status != 0:
        raise Unreadable(f"the decoder exited {status}")
    return r


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("csv", type=pathlib.Path, help="the timer capture's probe log (<run>.csv)")
    p.add_argument("--experiment", type=pathlib.Path, required=True, help="the supervisor's experiment.record")
    p.add_argument("--lineage", type=pathlib.Path, help="the task lineage after the trace started (default: <run>.csv.lineage.txt)")
    p.add_argument("--report", type=pathlib.Path, help="a saved trace-cmd report -t of the capture (default: run trace-cmd)")
    p.add_argument("--rt-cpu", type=int, help="the real-time CPU (default: the run record's, else 3)")
    a = p.parse_args()
    try:
        cap = tk_cost.Capture(a.csv)
    except (OSError, KeyError, ValueError, SystemExit) as e:
        print(f"emulator_residency: {a.csv}: unreadable ({e})", file=sys.stderr)
        return 2
    rt = a.rt_cpu if a.rt_cpu is not None else tk_cost.rt_cpu_of(a.csv)
    print(f"{a.csv.name}: tools/emulator_residency.py")
    try:
        emu = emulator(a.experiment, rt)
        emu.update(lineage(a.lineage or pathlib.Path(f"{a.csv}.lineage.txt"), emu, rt))
        if a.report:
            with open(a.report, errors="replace") as f:
                r = analyse(f, cap, emu, rt, sys.stdout)
        else:
            r = run_report(a.csv, cap, emu, rt, sys.stdout)
    except (OSError, Unreadable) as e:
        print(f"emulator_residency: UNREADABLE: {e}", file=sys.stderr)
        return 2
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
