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
descendants by parent, every thread of each. The lineage must close with its completeness line, list the emulator,
begin after the trace's first event and end before the window began; otherwise the evidence is unreadable. A task
created before the trace and gone before the lineage was taken cannot run in the window. To the lineage's tasks are
added the threads the record names, every task one of them is seen to create (sched_process_fork, from the trace's
start), and every task the trace shows exiting (sched_process_exit) before the lineage ended, which the listing cannot
have seen. The set only grows: a reused ID stays the emulator's (at worst a false failure, never a false pass).

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


def lineage(path: pathlib.Path, emu: dict, rt: int) -> dict:
    """The emulator's tasks in the session's lineage after the trace started: its process (by pid and start time) and
    its descendants by parent, every thread of each; when the lineage was taken (monotonic ns, its start and end); and
    those threads allowed on the real-time CPU. Raises Unreadable for a lineage that is missing, incomplete, untimed or
    without the emulator."""
    try:
        text = path.read_text(errors="replace").splitlines()
    except OSError as e:
        raise Unreadable(f"no task lineage after the trace started: {path}: {e}") from None
    rows, taken, problem = observer_affinity.parse_lineage(text)
    if rows is None:
        raise Unreadable(f"{path}: no task lineage")
    if problem:
        raise Unreadable(f"{path}: the task lineage is incomplete ({problem})")
    if taken is None:
        raise Unreadable(f"{path}: the task lineage does not say when it was taken")
    parents = {p: pp for (p, _), (pp, _, _, _) in rows.items()}
    starts = {p: ps for (p, _), (_, ps, _, _) in rows.items()}
    if starts.get(emu["pid"]) != int(emu["start"]):
        raise Unreadable(f"{path}: the emulator (pid {emu['pid']}, start {emu['start']}) is not in the task lineage"
                         + (f" (pid {emu['pid']} has start {starts[emu['pid']]})" if emu["pid"] in starts else ""))
    procs = observer_affinity.tree({emu["pid"]}, parents)
    tasks = {t: allowed for (p, t), (_, _, _, allowed) in rows.items() if p in procs}
    if any(not a for a in tasks.values()):
        raise Unreadable(f"{path}: an emulator thread without its allowed CPUs")
    return dict(seeds=set(tasks), procs=procs, taken=taken, allowed_rt=sorted(t for t, a in tasks.items() if rt in a))


def analyse(lines, cap, emu: dict, rt: int, out) -> dict:
    """The checks above on report lines (read to their end), for the emulator of emulator() with its lineage();
    raises Unreadable on incomplete evidence."""
    if "seeds" not in emu or "taken" not in emu:
        raise Unreadable("no task lineage after the trace started")
    lo, hi = cap.window
    t0, t1 = emu["taken"]
    tids = set(emu["tids"]) | set(emu["seeds"])
    first = last = None
    unparsed = malformed = inconsistent = 0
    occupant = None             # (comm, pid, since) on the real-time CPU
    occupant_at_lo = None
    ran_rt: list = []           # (tid, from, to) emulator stretches on the real-time CPU overlapping the window
    placed_rt: list = []        # (tid, from, to) emulator time on the real-time CPU, queued or running, overlapping the window
    moved_off: list = []        # (ts, tid) migrations of one off the real-time CPU inside the window
    place: dict = {}            # tid -> (cpu, since) while queued or running there, None while asleep; absent: unknown
    contradicted = 0            # a task's event that does not follow from its known place
    woken_rt = moved_rt = 0     # events onto the real-time CPU inside the window (descriptive)
    forks: list = []            # (ts, parent, child)
    exited: set = set()         # tasks seen exiting before the lineage ended
    ran = collections.Counter()
    waits: list = []
    woken: dict = {}            # tid -> wakeup ts, awaiting its switch-in

    def on(tid: int) -> int | None:
        """The CPU a task is known to be queued or running on (None: asleep or unknown)."""
        p = place.get(tid)
        return p[0] if p else None

    def leave(tid: int, ts: int) -> None:
        p = place.get(tid)
        if p and p[0] == rt and ts >= lo and p[1] <= hi:
            placed_rt.append((tid, p[1], ts))

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
        if ev == "sched_process_fork":
            f = FORK.search(rest)
            if not f:
                malformed += 1
                continue
            if int(f["pid"]) in tids:
                tids.add(int(f["child"]))
                forks.append((ts, int(f["pid"]), int(f["child"])))
        elif ev == "sched_process_exit":
            x = EXIT.search(rest)
            if not x:
                malformed += 1
                continue
            if ts <= t1 and int(x["pid"]) not in tids:
                exited.add(int(x["pid"]))
                tids.add(int(x["pid"]))
        elif ev == "sched_switch":
            s = trace_correlate.SWITCH.match(rest)
            if not s:
                malformed += 1
                continue
            ppid, npid = int(s["ppid"]), int(s["npid"])
            if cpu == rt:
                if occupant is not None and ppid != occupant[1]:
                    inconsistent += 1
                if occupant is not None and occupant[1] in tids and ts >= lo:
                    ran_rt.append((occupant[1], occupant[2], ts))
                occupant = (s["next"], npid, ts)
                if ts <= lo:
                    occupant_at_lo = occupant
            if ppid in tids:  # it ran here: it was switched in here, unless its place is unknown
                if ppid in place and on(ppid) != cpu:
                    contradicted += 1
                put(ppid, cpu if PREEMPTED.match(s["state"]) else None, ts)
            if npid in tids:  # queued here, or (a wakeup's place is its only source) asleep or unknown
                if on(npid) not in (None, cpu):
                    contradicted += 1
                put(npid, cpu, ts)
                if inside:
                    ran[cpu] += 1
                    if npid in woken:
                        waits.append(ts - woken.pop(npid))
        elif ev == "sched_wakeup":
            w = WAKEUP.match(rest)
            if not w:
                malformed += 1
                continue
            pid, to = int(w["pid"]), int(w["cpu"])
            if pid in tids:
                if on(pid) not in (None, to):  # a queued or running task is woken where it is
                    contradicted += 1
                put(pid, to, ts)
                if inside:
                    woken[pid] = ts
                    woken_rt += to == rt
        else:
            g = MIGRATE.search(rest)
            if not g:
                malformed += 1
                continue
            pid, orig, dest = int(g["pid"]), int(g["orig"]), int(g["dest"])
            if pid in tids:
                if on(pid) not in (None, orig):  # a queued task moves from where it is
                    contradicted += 1
                if inside and orig == rt:  # its CPU was the real-time one, whatever the trace showed of it before
                    moved_off.append((ts, pid))
                if on(pid) is not None or dest == rt:  # a queued task moves; a sleeper's wakeup that follows places it
                    put(pid, dest, ts)
                moved_rt += inside and dest == rt
    if last is None or last < hi:
        raise Unreadable(f"the report ends at {last} ns, before the window's end ({hi} ns)")
    if unparsed or malformed or inconsistent or contradicted:
        raise Unreadable(f"{malformed} malformed required events, {unparsed} lines that are no events, {inconsistent} switches of CPU {rt} "
                         f"that do not follow on from the previous one, {contradicted} events of the emulator's tasks that do not "
                         "follow from where they were")
    if not first < t0:
        raise Unreadable(f"the task lineage began at {t0} ns, not after the trace's first event ({first} ns)")
    if not t1 < lo:
        raise Unreadable(f"the task lineage ended at {t1} ns, not before the window began ({lo} ns)")
    if occupant_at_lo is None:
        raise Unreadable(f"the task on CPU {rt} when the window began is unknown (no switch onto it recorded before the window)")
    if occupant is not None and occupant[1] in tids:
        ran_rt.append((occupant[1], occupant[2], hi))
    for tid in list(place):
        leave(tid, hi)
    seen = sum(ran.values())
    print(f"window: {lo} to {hi} ns ({(hi - lo) / 1e9:.6f} s); real-time CPU {rt}", file=out)
    print(f"  the emulator: pid {emu['pid']} (start {emu['start']}); its tasks: {len(emu['seeds'])} in the lineage after the trace started "
          f"({(t0 - lo) / 1e9:+.3f} to {(t1 - lo) / 1e9:+.3f} s), {len(set(emu['tids']) - set(emu['seeds']))} more in the record, "
          + (f"{len(forks)} created in the trace ({', '.join(f'{c} by {p}' for _, p, c in forks[:5])})" if forks else "none created in the trace")
          + f"; {len(exited)} other tasks counted as its, exited before the lineage ended", file=out)
    print(f"  on CPU {rt} when the window began: {occupant_at_lo[0]} ({occupant_at_lo[1]}), since {occupant_at_lo[2]} ns", file=out)
    print(f"  switches to it in the window: {seen}" + (" (CPUs: " + ", ".join(f"{c} {n}" for c, n in sorted(ran.items())) + ")" if seen else ""), file=out)
    print(f"  its stretches on CPU {rt} overlapping the window: {len(ran_rt)}" + (" (" + ", ".join(
        f"{t} {(a - lo) / 1e6:+.3f}..{(b - lo) / 1e6:+.3f} ms" for t, a, b in ran_rt[:5]) + ")" if ran_rt else ""), file=out)
    print(f"  its time on CPU {rt} (queued or running) overlapping the window: {len(placed_rt)}" + (" (" + ", ".join(
        f"{t} {(a - lo) / 1e6:+.3f}..{(b - lo) / 1e6:+.3f} ms" for t, a, b in placed_rt[:5]) + ")" if placed_rt else "")
          + f"; inside the window, wakeups of it onto CPU {rt} {woken_rt}, migrations of it onto CPU {rt} {moved_rt}", file=out)
    print(f"  migrations of it off CPU {rt} inside the window: {len(moved_off)}" + (" (" + ", ".join(
        f"{t} {(a - lo) / 1e6:+.3f} ms" for a, t in moved_off[:5]) + ")" if moved_off else ""), file=out)
    print(f"  its threads in the lineage allowed on CPU {rt}: {len(emu['allowed_rt'])}"
          + (f" ({', '.join(map(str, emu['allowed_rt'][:10]))})" if emu["allowed_rt"] else ""), file=out)
    if waits:
        print(f"  wake to run (descriptive): {tk_cost.dist(sorted(waits))}", file=out)
    if not seen:
        print("  the emulator was not seen to run in the window: nothing establishes where it ran", file=out)
    ok = seen > 0 and not ran_rt and not placed_rt and not moved_off and not emu["allowed_rt"]
    print(f"emulator residency: {'PASS' if ok else 'FAIL'}", file=out)
    return dict(ok=ok, seen=seen, occupant=occupant_at_lo[1], ran_rt=len(ran_rt), placed_rt=len(placed_rt), moved_off=len(moved_off),
                woken_rt=woken_rt, moved_rt=moved_rt, allowed_rt=len(emu["allowed_rt"]), forks=len(forks), exited=len(exited),
                cpus=dict(ran), waits=waits)


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
