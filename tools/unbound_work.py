#!/usr/bin/env python3
"""Whether the unbound workqueues' workers stayed off the real-time CPU during a timer capture, for the loaded emulator
validation's Option A (docs/hardware/control-rate.md): with that CPU out of the unbound workqueues' mask
(tools/wq_mask.sh), none of their workers may run there. Read-only.

    unbound_work.py <run>.csv [--report FILE] [--rt-cpu N] [--experiment FILE [--lineage FILE] [--allowed LIST]]

Inside the probe's window (its first wake stamp to its last cycle-end stamp, as tools/tk_cost.py takes it), from the
main trace (trace-cmd report -t under tools/tk_record.py's launch, or a saved --report):

- residency: the tasks the real-time CPU ran, from its sched_switch events, from the one running when the window began
  (the last switch onto the CPU before it; none recorded leaves it unknown, which fails) to the window's end. Any
  stretch of an unbound worker (kworker/u*) or a rescuer (kworker/R-*) on the CPU that overlaps the window fails, as
  does a per-CPU worker of another CPU (kworker/N:M with N not the real-time CPU) and any other kworker/ name (an
  unclassified worker). The real-time CPU's own per-CPU workers are counted, not judged: the mask does not govern them;
- executions: each work item's execution (workqueue_execute_start to its workqueue_execute_end) is followed across its
  task's switches on every CPU, so that it is known on every CPU it ran on from its start to its end, not only where
  it began and ended; one begun before the report (an end with no start) is followed from the report's first event,
  through its task's switches before then, and one still open when the report ends, or 100 ms after the window, to
  there. Every check below judges an execution on all of those CPUs. Work executed on the real-time CPU by an unbound
  worker or a rescuer, and any tty flush (flush_to_ldisc) there, by an execution overlapping the window (one open when
  the window began counts) fail, as does work executed there by a task that is no kernel worker (unclassified);
- the worker's flushes (its instructions to the emulator, workqueue_queue_work of flush_to_ldisc by the probe's
  worker inside the window), each matched to that work's next execution: one never seen to run, or run on the
  real-time CPU at any point of its execution, fails; the CPUs they began on, how many moved while executing and their
  queue-to-run delays are descriptive;
- with --experiment FILE (the supervisor's record) the emulator's flushes too, the other tty direction (its replies,
  and any flush of its tty its reads restart): workqueue_queue_work of flush_to_ldisc inside the window by one of the
  emulator's tasks, identified as tools/emulator_residency.py identifies them (the record's threads, its process tree
  in the task lineage after the trace started, --lineage FILE or <run>.csv.lineage.txt, the tasks the trace shows
  exiting before the lineage ended, and every task these created through the trace's forks up to the window's end),
  each matched to its next execution: none queued, one never seen to run, or one run on the real-time CPU at any
  point of its execution, fails;
- with --allowed LIST (plan L1: the unbound mask set to exactly those CPUs; it needs --experiment, so that both tty
  directions are judged), besides every check above: any execution by an unbound worker or a rescuer that overlaps
  the window and ran on a CPU outside LIST at any point of it (a move within LIST passes), and any flush of either
  direction, by whatever task, that ran outside LIST at any point of its execution, fails. LIST is a canonical CPU
  list without the real-time CPU. Per-CPU workers' executions other than the flushes stay outside these checks (the
  mask does not govern them).

The evidence must be complete: every sched_switch (of every CPU) and every workqueue event must parse (with
--experiment, every sched_process_fork and sched_process_exit too, and the lineage must begin after the trace's first
event and end before the window began), every event line after the first must be an event line, the real-time CPU's
switches must follow on from each other (each switch's previous task is the one the last switch put there), an
executing task's events must follow from where it ran (its execution ends where it is running; it is not switched in
while running, nor out of a CPU it is not on; no start while its last execution is open; no end without a start after
its last end; and, for one begun before the report, its earlier switches follow on too), the report must reach past
the window's end, and it is read to its end: a decoder that exits nonzero, at any point, makes the evidence
unreadable.

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
import emulator_residency  # noqa: E402
import tk_cost  # noqa: E402
import tk_record  # noqa: E402
import trace_correlate  # noqa: E402
import wq_target  # noqa: E402

AFTER_NS = 100_000_000  # executions are looked for this long after the window, for flushes queued near its end
QUEUE = re.compile(r"work struct=(\S+) function=(\S+) workqueue=(\S+)")
EXEC = re.compile(r"work struct (\S+?): function (\S+)")
WORK_EVENTS = {"workqueue_queue_work", "workqueue_execute_start", "workqueue_execute_end"}
PERCPU = re.compile(r"kworker/(\d+):\d+H?(?:-.*)?")


class Unreadable(Exception):
    pass


def classify(comm: str, rt: int) -> str:
    """unbound, rescuer, own (a per-CPU worker of the real-time CPU), other-cpu, unclassified (another kworker/ name), or
    task (no kernel worker)."""
    if comm.startswith("kworker/u"):
        return "unbound"
    if comm.startswith("kworker/R-"):
        return "rescuer"
    m = PERCPU.fullmatch(comm)
    if m:
        return "own" if int(m[1]) == rt else "other-cpu"
    return "unclassified" if comm.startswith("kworker/") else "task"


FORBIDDEN = {"unbound", "rescuer", "other-cpu", "unclassified"}
ABSENT = object()  # no switch of the task seen yet


class Execution:
    """One work item's execution by one task, followed across that task's switches: the CPUs it ran on, and for how
    long, from its start (None: begun before the report) to its end."""

    __slots__ = ("ts", "cpu", "comm", "fn", "on", "since", "ran", "end", "end_cpu")

    def __init__(self, ts: int | None, cpu: int | None, comm: str, fn: str) -> None:
        self.ts, self.cpu, self.comm, self.fn = ts, cpu, comm, fn
        self.on, self.since = cpu, ts            # the CPU it runs on now (None while switched out), and since when
        self.ran: collections.Counter = collections.Counter()  # cpu -> its running time in the execution
        self.end: int | None = None
        self.end_cpu: int | None = None

    def where(self) -> set:
        """Every CPU the execution ran on: its start and end CPUs and every stretch between them."""
        return {c for c, w in self.ran.items() if w > 0} | ({self.cpu, self.end_cpu} - {None})


def analyse(lines, cap, rt: int, out, allowed: str | None = None, emu: dict | None = None) -> dict:
    """The checks above on report lines (read to their end); raises Unreadable on incomplete evidence. emu is the
    emulator of tools/emulator_residency.py's emulator() with its lineage() (the emulator's flushes are judged only
    with it); allowed a canonical CPU list (it needs emu)."""
    lo, hi = cap.window
    allowed_set = set(wq_target.canonical_list(allowed)) if allowed is not None else None
    if allowed_set is not None and (emu is None or rt in allowed_set):
        raise Unreadable(f"the allowed CPUs {allowed} need the emulator's record, and must not hold the real-time CPU {rt}")
    if emu is not None and ("seeds" not in emu or "taken" not in emu):
        raise Unreadable("no task lineage after the trace started")
    first = None
    last = None
    started = False
    unparsed = malformed = inconsistent = 0
    occupant = None             # (comm, pid, since) on the real-time CPU
    occupant_at_lo = None
    resident: list = []         # (comm, pid, class, from, to) forbidden stretches overlapping the window
    own = 0                     # the real-time CPU's own per-CPU workers that ran in the window
    open_exec: dict = {}        # pid -> its open Execution
    seen_exec: set = set()      # tasks with an execution event in the report
    before: dict = {}           # pid -> (cpu, since) while running, None while switched out: tasks with no execution event yet
    before_ran: dict = {}       # pid -> Counter (cpu -> running time) on the judged CPUs, while it had no execution event
    spoiled: set = set()        # tasks with no execution event yet whose switches did not follow on
    contra = 0                  # execution events, and switches of executing tasks, that do not follow on
    executed: list = []         # (comm, class, function, start, end) forbidden executions overlapping the window
    queued: list = []           # (ts, work, workqueue) of the worker's flushes inside the window
    pending: dict = {}          # work -> index into queued, awaiting its execution
    ran: dict = {}              # index -> its Execution
    outside_n = 0               # unbound or rescuer executions overlapping the window that ran outside the allowed CPUs
    outside_ex: list = []       # the first five of them: (comm, class, function, cpus)
    moved = 0                   # executions overlapping the window that ran on more than one CPU (descriptive)
    other_q: list = []          # (ts, work, workqueue, pid) of flushes queued inside the window by other tasks (with emu)
    other_pending: dict = {}    # work -> index into other_q
    other_ran: dict = {}        # index -> its Execution
    forks: list = []            # (ts, parent, child) every fork up to the window's end (with emu)
    exited: set = set()         # every task seen exiting before the lineage ended (with emu)
    now = None                  # the last event judged

    def stretch(comm: str, pid: int, a: int, b: int) -> None:
        nonlocal own
        if b < lo or a > hi:
            return
        c = classify(comm, rt)
        if c in FORBIDDEN:
            resident.append((comm, pid, c, a, b))
        elif c == "own":
            own += 1

    def judged(cpu: int) -> bool:
        return cpu == rt or (allowed_set is not None and cpu not in allowed_set)

    def credit(e: Execution, cpu: int, a: int, b: int) -> None:
        if b > a:
            e.ran[cpu] += b - a

    def leave(pid: int, cpu: int, ts: int) -> None:
        """pid switched out of cpu at ts."""
        nonlocal contra
        if pid == 0:
            return
        e = open_exec.get(pid)
        if e is not None:
            if e.on != cpu:
                contra += 1
            else:
                credit(e, cpu, e.since, ts)
            e.on = None
        elif pid not in seen_exec:
            u = before.get(pid, ABSENT)
            if u is None or (u is not ABSENT and u[0] != cpu):
                spoiled.add(pid)
            elif judged(cpu):
                since = first if u is ABSENT else u[1]  # no switch of it seen before: running there since the report began
                before_ran.setdefault(pid, collections.Counter())[cpu] += ts - since
            before[pid] = None

    def enter(pid: int, cpu: int, ts: int) -> None:
        """pid switched onto cpu at ts."""
        nonlocal contra
        if pid == 0:
            return
        e = open_exec.get(pid)
        if e is not None:
            if e.on is not None:
                contra += 1
            e.on, e.since = cpu, ts
        elif pid not in seen_exec:
            if before.get(pid, ABSENT) not in (ABSENT, None):
                spoiled.add(pid)
            before[pid] = (cpu, ts)

    def judge(e: Execution, end: int) -> None:
        """An execution's general checks, once it ended (or the report did): every CPU it ran on counts once it
        overlaps the window."""
        nonlocal outside_n, moved
        a = e.ts if e.ts is not None else first
        if end < lo or a > hi:
            return
        c = classify(e.comm, rt)
        where = e.where()
        moved += len(where) > 1
        if rt in where and (c in FORBIDDEN or c == "task" or e.fn == "flush_to_ldisc"):
            executed.append((e.comm, c, e.fn, a, end))
        off = sorted(k for k in where if allowed_set is not None and k not in allowed_set and k != rt)
        if off and c in ("unbound", "rescuer"):
            outside_n += 1
            if len(outside_ex) < 5:
                outside_ex.append((e.comm, c, e.fn, off))

    for ln in lines:
        ln = ln.rstrip("\n")
        m = trace_correlate.LINE.match(ln)
        if not m:
            if started and ln.strip():
                unparsed += 1
            continue
        started = True
        ts = int(m["s"]) * 1_000_000_000 + int(m["ns"])
        if first is None:
            first = ts
        last = ts
        if ts > hi + AFTER_NS:
            continue  # read to the end, judged no more
        now = ts
        ev, rest, cpu, pid, comm = m["ev"], m["rest"], int(m["cpu"]), int(m["pid"]), m["comm"].strip()
        if ev == "sched_switch":
            s = trace_correlate.SWITCH.match(rest)
            if not s:
                malformed += 1
                continue
            if cpu == rt:
                if occupant is not None and int(s["ppid"]) != occupant[1]:
                    inconsistent += 1
                if occupant is not None:
                    stretch(occupant[0], occupant[1], occupant[2], ts)
                occupant = (s["next"], int(s["npid"]), ts)
                if ts <= lo:
                    occupant_at_lo = occupant
            leave(int(s["ppid"]), cpu, ts)
            enter(int(s["npid"]), cpu, ts)
        elif emu is not None and ev == "sched_process_fork":
            f = emulator_residency.FORK.search(rest)
            if not f:
                malformed += 1
                continue
            if ts <= hi:
                forks.append((ts, int(f["pid"]), int(f["child"])))
        elif emu is not None and ev == "sched_process_exit":
            x = emulator_residency.EXIT.search(rest)
            if not x:
                malformed += 1
                continue
            if ts <= emu["taken"][1]:
                exited.add(int(x["pid"]))
        elif ev in WORK_EVENTS:
            if ev == "workqueue_queue_work":
                q = QUEUE.search(rest)
                if not q:
                    malformed += 1
                    continue
                if pid == cap.tid and lo <= ts <= hi and q[2] == "flush_to_ldisc":
                    pending[q[1]] = len(queued)
                    queued.append((ts, q[1], q[3]))
                elif emu is not None and lo <= ts <= hi and q[2] == "flush_to_ldisc":
                    other_pending[q[1]] = len(other_q)
                    other_q.append((ts, q[1], q[3], pid))
                continue
            x = EXEC.search(rest)
            if not x:
                malformed += 1
                continue
            if ev == "workqueue_execute_start":
                e = Execution(ts, cpu, comm, x[2])
                k = pending.pop(x[1], None)
                if k is not None:
                    ran[k] = e
                k = other_pending.pop(x[1], None)
                if k is not None:
                    other_ran[k] = e
                if pid in open_exec:
                    contra += 1  # a start while its task's last execution is open
                seen_exec.add(pid)
                before.pop(pid, None)
                before_ran.pop(pid, None)
                spoiled.discard(pid)
                open_exec[pid] = e
            else:
                e = open_exec.pop(pid, None)
                if e is not None:
                    if e.on != cpu:
                        contra += 1  # it ends where it was not running
                    else:
                        credit(e, cpu, e.since, ts)
                elif pid in seen_exec:
                    contra += 1  # an end with no start since its task's last end
                else:
                    # Begun before the report: it ran wherever its task ran from the report's first event.
                    e = Execution(None, None, comm, x[2])
                    e.ran = before_ran.pop(pid, collections.Counter())
                    u = before.pop(pid, ABSENT)
                    if pid in spoiled or u is None or (u is not ABSENT and u[0] != cpu):
                        contra += 1
                    else:
                        credit(e, cpu, first if u is ABSENT else u[1], ts)
                    spoiled.discard(pid)
                    seen_exec.add(pid)
                if e is not None:
                    e.end, e.end_cpu = ts, cpu
                    judge(e, ts)
    if last is None or last < hi:
        raise Unreadable(f"the report ends at {last} ns, before the window's end ({hi} ns)")
    if unparsed or malformed or inconsistent or contra:
        raise Unreadable(f"{malformed} malformed required events, {unparsed} lines that are no events, {inconsistent} switches of CPU {rt} "
                         f"that do not follow on from the previous one, {contra} execution events or switches of executing tasks "
                         "that do not follow from where those tasks ran")
    if emu is not None and not (first is not None and first < emu["taken"][0] and emu["taken"][1] < lo):
        raise Unreadable(f"the task lineage ({emu['taken'][0]} to {emu['taken'][1]} ns) does not begin after the trace's first "
                         f"event ({first} ns) and end before the window began ({lo} ns)")
    if occupant is not None:
        stretch(occupant[0], occupant[1], occupant[2], hi)
    stop = hi if now is None else now
    for e in open_exec.values():  # still open at the report's end (or the horizon): followed to there
        if e.on is not None:
            credit(e, e.on, e.since, stop)
        judge(e, stop)
    delays = sorted(ran[k].ts - queued[k][0] for k in ran)
    cpus = collections.Counter(ran[k].cpu for k in ran)  # where each began (descriptive)
    on_rt = sum(1 for k in ran if rt in ran[k].where())
    unmatched = len(queued) - len(ran)
    wqs = collections.Counter(q[2] for q in queued)
    known = occupant_at_lo is not None
    off_instr = sum(1 for k in ran if allowed_set is not None and ran[k].where() - allowed_set)
    instr_moved = sum(1 for k in ran if len(ran[k].where()) > 1)
    emu_tids: set = set()
    emu_idx: list = []
    if emu is not None:
        emu_tids = emulator_residency.closure(set(emu["tids"]) | set(emu["seeds"]) | exited, forks)
        emu_idx = [i for i, q in enumerate(other_q) if q[3] in emu_tids]
    emu_ran = [other_ran[i] for i in emu_idx if i in other_ran]
    emu_unmatched = len(emu_idx) - len(emu_ran)
    emu_on_rt = sum(1 for e in emu_ran if rt in e.where())
    emu_off = sum(1 for e in emu_ran if allowed_set is not None and e.where() - allowed_set)
    emu_moved = sum(1 for e in emu_ran if len(e.where()) > 1)
    print(f"window: {lo} to {hi} ns ({(hi - lo) / 1e9:.6f} s); real-time CPU {rt}", file=out)
    print(f"  on CPU {rt} when the window began: " + (f"{occupant_at_lo[0]} ({occupant_at_lo[1]}), since {occupant_at_lo[2]} ns" if known
                                                    else "unknown (no switch onto it recorded before the window)"), file=out)
    print(f"  forbidden workers resident on CPU {rt} in the window: {len(resident)}" + (" (" + ", ".join(
        f"{c} {k} {(a - lo) / 1e6:+.3f}..{(b - lo) / 1e6:+.3f} ms" for c, _, k, a, b in resident[:5]) + (", ..." if len(resident) > 5 else "") + ")"
        if resident else ""), file=out)
    print(f"  forbidden executions on CPU {rt} in the window: {len(executed)}" + (" (" + ", ".join(
        f"{f} by {c} ({k})" for c, k, f, _, _ in executed[:5]) + (", ..." if len(executed) > 5 else "") + ")" if executed else ""), file=out)
    print(f"  CPU {rt}'s own per-CPU workers that ran in the window (not governed by the mask; counted, not judged): {own}", file=out)
    print(f"  executions overlapping the window that ran on more than one CPU (followed across their task's switches): {moved}",
          file=out)
    print(f"  the worker's flushes (its instructions to the emulator): {len(queued)} queued ("
          + (", ".join(f"{w} {n}" for w, n in wqs.items()) or "none") + f"), {len(ran)} seen to run, {unmatched} not", file=out)
    if delays:
        print("    ran on CPUs: " + ", ".join(f"{c} {n}" for c, n in sorted(cpus.items())) + f"; on CPU {rt}: {on_rt}", file=out)
        print(f"    queued to executed (descriptive): {tk_cost.dist(delays)}", file=out)
        print(f"    moved between CPUs while executing (each judged on every CPU it ran on): {instr_moved}", file=out)
    if not queued:
        print("  no flush queued by the worker: nothing establishes where the instructions' flushes ran", file=out)
    if emu is not None:
        ecpus = collections.Counter(e.cpu for e in emu_ran)
        edelays = sorted(other_ran[i].ts - other_q[i][0] for i in emu_idx if i in other_ran)
        print(f"  the emulator's flushes (its replies, the other direction): {len(emu_idx)} queued by its {len(emu_tids)} tasks, "
              f"{len(emu_ran)} seen to run, {emu_unmatched} not", file=out)
        if emu_ran:
            print("    ran on CPUs: " + ", ".join(f"{c} {n}" for c, n in sorted(ecpus.items())) + f"; on CPU {rt}: {emu_on_rt}", file=out)
            print(f"    queued to executed (descriptive): {tk_cost.dist(edelays)}", file=out)
            print(f"    moved between CPUs while executing (each judged on every CPU it ran on): {emu_moved}", file=out)
        if not emu_idx:
            print("  no flush queued by the emulator: nothing establishes where the replies' flushes ran", file=out)
    if allowed_set is not None:
        print(f"  outside the allowed CPUs {allowed}: unbound or rescuer executions in the window {outside_n}" + (" (" + ", ".join(
            f"{f} by {c} ({k}) on CPU {','.join(map(str, u))}" for c, k, f, u in outside_ex) + (", ..." if outside_n > 5 else "") + ")" if outside_n else "")
              + f"; the worker's flushes run there {off_instr}; the emulator's {emu_off}", file=out)
    ok = (known and not resident and not executed and on_rt == 0 and unmatched == 0 and bool(queued)
          and (emu is None or (bool(emu_idx) and emu_unmatched == 0 and emu_on_rt == 0))
          and (allowed_set is None or (outside_n == 0 and off_instr == 0 and emu_off == 0)))
    print(f"unbound work: {'PASS' if ok else 'FAIL'}", file=out)
    return dict(ok=ok, known=known, resident=len(resident), executed=len(executed), own=own, queued=len(queued), ran=len(ran),
                on_rt=on_rt, unmatched=unmatched, cpus=dict(cpus), delays=delays, emu_queued=len(emu_idx), emu_ran=len(emu_ran),
                emu_unmatched=emu_unmatched, emu_on_rt=emu_on_rt, outside=outside_n, off_instr=off_instr, emu_off=emu_off,
                moved=moved, instr_moved=instr_moved, emu_moved=emu_moved)


def run_report(csv_path: pathlib.Path, cap, rt: int, out, allowed: str | None = None, emu: dict | None = None) -> dict:
    """analyse() over trace-cmd report -t under the launch, read to its end; the decoder's exit is always collected."""
    decoder = os.environ.get("UNBOUND_WORK_DECODER") if os.environ.get("UNBOUND_WORK_TEST") == "1" else None
    if decoder is None:
        tool = tk_record.tool_identity("trace-cmd")
        if tool.get("read_status") != "ok" or not tool.get("path"):
            raise Unreadable(f"trace-cmd is not identified ({tool.get('read_status')})")
        decoder = tool["path"]
    proc = subprocess.Popen([decoder, "report", "-t", "-i", f"{csv_path}.trace.dat"], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, errors="replace", env=tk_record.launch_env())
    assert proc.stdout is not None
    try:
        r = analyse(proc.stdout, cap, rt, out, allowed, emu)
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
    p.add_argument("--report", type=pathlib.Path, help="a saved trace-cmd report -t of the capture (default: run trace-cmd)")
    p.add_argument("--rt-cpu", type=int, help="the real-time CPU (default: the run record's, else 3)")
    p.add_argument("--experiment", type=pathlib.Path, help="the supervisor's experiment.record: the emulator's flushes judged too")
    p.add_argument("--lineage", type=pathlib.Path, help="the task lineage after the trace started (default: <run>.csv.lineage.txt)")
    p.add_argument("--allowed", help="the unbound mask's CPUs, a canonical list (plan L1: 4-5; needs --experiment)")
    a = p.parse_args()
    if a.allowed is not None and a.experiment is None:
        p.error("--allowed needs --experiment (both tty directions are judged)")
    if a.lineage is not None and a.experiment is None:
        p.error("--lineage goes with --experiment")
    try:
        cap = tk_cost.Capture(a.csv)
    except (OSError, KeyError, ValueError, SystemExit) as e:
        print(f"unbound_work: {a.csv}: unreadable ({e})", file=sys.stderr)
        return 2
    if cap.tid is None:
        print(f"unbound_work: {a.csv}: no worker_tid in the summary", file=sys.stderr)
        return 2
    rt = a.rt_cpu if a.rt_cpu is not None else tk_cost.rt_cpu_of(a.csv)
    allowed = None
    if a.allowed is not None:
        try:
            allowed = set(wq_target.canonical_list(a.allowed))
        except wq_target.Refused as e:
            print(f"unbound_work: --allowed: {e}", file=sys.stderr)
            return 2
        if rt in allowed:
            print(f"unbound_work: --allowed {a.allowed} holds the real-time CPU {rt}", file=sys.stderr)
            return 2
    print(f"{a.csv.name}: tools/unbound_work.py")
    try:
        emu = None
        if a.experiment is not None:
            emu = emulator_residency.emulator(a.experiment, rt, allowed)
            emu.update(emulator_residency.lineage(a.lineage or pathlib.Path(f"{a.csv}.lineage.txt"), emu, rt, allowed))
        if a.report:
            with open(a.report, errors="replace") as f:
                r = analyse(f, cap, rt, sys.stdout, a.allowed, emu)
        else:
            r = run_report(a.csv, cap, rt, sys.stdout, a.allowed, emu)
    except emulator_residency.Unreadable as e:
        print(f"unbound_work: UNREADABLE: {e}", file=sys.stderr)
        return 2
    except (OSError, Unreadable) as e:
        print(f"unbound_work: UNREADABLE: {e}", file=sys.stderr)
        return 2
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
