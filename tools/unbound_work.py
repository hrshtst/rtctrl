#!/usr/bin/env python3
"""Whether the unbound workqueues' workers stayed off the real-time CPU during a timer capture, for the loaded emulator
validation's Option A (docs/hardware/control-rate.md): with that CPU out of the unbound workqueues' mask
(tools/wq_mask.sh), none of their workers may run there. Read-only.

    unbound_work.py <run>.csv [--report FILE] [--rt-cpu N]

Inside the probe's window (its first wake stamp to its last cycle-end stamp, as tools/tk_cost.py takes it), from the
main trace (trace-cmd report -t under tools/tk_record.py's launch, or a saved --report):

- residency: the tasks the real-time CPU ran, from its sched_switch events, from the one running when the window began
  (the last switch onto the CPU before it; none recorded leaves it unknown, which fails) to the window's end. Any
  stretch of an unbound worker (kworker/u*) or a rescuer (kworker/R-*) on the CPU that overlaps the window fails, as
  does a per-CPU worker of another CPU (kworker/N:M with N not the real-time CPU) and any other kworker/ name (an
  unclassified worker). The real-time CPU's own per-CPU workers are counted, not judged: the mask does not govern them;
- executions: work executed on the real-time CPU (workqueue_execute_start to its workqueue_execute_end) by an unbound
  worker or a rescuer, and any tty flush (flush_to_ldisc) there, overlapping the window (an execution open when the
  window began counts) fail, as does work executed by a task that is no kernel worker (unclassified);
- the worker's flushes (its instructions to the emulator, workqueue_queue_work of flush_to_ldisc by the probe's
  worker inside the window), each matched to that work's next execution: one never seen to run, or run on the
  real-time CPU, fails; their CPUs and queue-to-run delays are descriptive.

The evidence must be complete: every sched_switch of the real-time CPU and every workqueue event must parse, every
event line after the first must be an event line, the real-time CPU's switches must follow on from each other (each
switch's previous task is the one the last switch put there), the report must reach past the window's end, and it is
read to its end: a decoder that exits nonzero, at any point, makes the evidence unreadable.

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
import tk_cost  # noqa: E402
import tk_record  # noqa: E402
import trace_correlate  # noqa: E402

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


def analyse(lines, cap, rt: int, out) -> dict:
    """The checks above on report lines (read to their end); raises Unreadable on incomplete evidence."""
    lo, hi = cap.window
    last = None
    started = False
    unparsed = malformed = inconsistent = 0
    occupant = None             # (comm, pid, since) on the real-time CPU
    occupant_at_lo = None
    resident: list = []         # (comm, pid, class, from, to) forbidden stretches overlapping the window
    own = 0                     # the real-time CPU's own per-CPU workers that ran in the window
    open_exec: dict = {}        # pid -> (ts, cpu, comm, work, function)
    executed: list = []         # (comm, class, function, start, end) forbidden executions overlapping the window
    queued: list = []           # (ts, work, workqueue) of the worker's flushes inside the window
    pending: dict = {}          # work -> index into queued, awaiting its execution
    ran: dict = {}              # index -> (ts, cpu, comm)

    def stretch(comm: str, pid: int, a: int, b: int) -> None:
        nonlocal own
        if b < lo or a > hi:
            return
        c = classify(comm, rt)
        if c in FORBIDDEN:
            resident.append((comm, pid, c, a, b))
        elif c == "own":
            own += 1

    def execution(rec: tuple, end: int) -> None:
        ts, cpu, comm, _, fn = rec
        if cpu != rt or end < lo or ts > hi:
            return
        c = classify(comm, rt)
        if c in FORBIDDEN or c == "task" or fn == "flush_to_ldisc":
            executed.append((comm, c, fn, ts, end))

    for ln in lines:
        ln = ln.rstrip("\n")
        m = trace_correlate.LINE.match(ln)
        if not m:
            if started and ln.strip():
                unparsed += 1
            continue
        started = True
        ts = int(m["s"]) * 1_000_000_000 + int(m["ns"])
        last = ts
        if ts > hi + AFTER_NS:
            continue  # read to the end, judged no more
        ev, rest, cpu, pid, comm = m["ev"], m["rest"], int(m["cpu"]), int(m["pid"]), m["comm"].strip()
        if ev == "sched_switch" and cpu == rt:
            s = trace_correlate.SWITCH.match(rest)
            if not s:
                malformed += 1
                continue
            if occupant is not None and int(s["ppid"]) != occupant[1]:
                inconsistent += 1
            if occupant is not None:
                stretch(occupant[0], occupant[1], occupant[2], ts)
            occupant = (s["next"], int(s["npid"]), ts)
            if ts <= lo:
                occupant_at_lo = occupant
        elif ev in WORK_EVENTS:
            if ev == "workqueue_queue_work":
                q = QUEUE.search(rest)
                if not q:
                    malformed += 1
                    continue
                if pid == cap.tid and lo <= ts <= hi and q[2] == "flush_to_ldisc":
                    pending[q[1]] = len(queued)
                    queued.append((ts, q[1], q[3]))
                continue
            x = EXEC.search(rest)
            if not x:
                malformed += 1
                continue
            if ev == "workqueue_execute_start":
                k = pending.pop(x[1], None)
                if k is not None:
                    ran[k] = (ts, cpu, comm)
                open_exec[pid] = (ts, cpu, comm, x[1], x[2])
            else:
                rec = open_exec.pop(pid, None)
                if rec is not None:
                    execution(rec, ts)
                    if rec[1] != cpu:  # the execution moved: its end CPU judged too
                        execution((ts, cpu, comm, rec[3], rec[4]), ts)
                else:
                    execution((ts, cpu, comm, x[1], x[2]), ts)  # begun before the report: judged where it ends
    if last is None or last < hi:
        raise Unreadable(f"the report ends at {last} ns, before the window's end ({hi} ns)")
    if unparsed or malformed or inconsistent:
        raise Unreadable(f"{malformed} malformed required events, {unparsed} lines that are no events, {inconsistent} switches of CPU {rt} "
                         "that do not follow on from the previous one")
    if occupant is not None:
        stretch(occupant[0], occupant[1], occupant[2], hi)
    for rec in open_exec.values():
        execution(rec, hi)
    delays = sorted(ran[k][0] - queued[k][0] for k in ran)
    cpus = collections.Counter(ran[k][1] for k in ran)
    on_rt = sum(1 for k in ran if ran[k][1] == rt)
    unmatched = len(queued) - len(ran)
    wqs = collections.Counter(q[2] for q in queued)
    known = occupant_at_lo is not None
    print(f"window: {lo} to {hi} ns ({(hi - lo) / 1e9:.6f} s); real-time CPU {rt}", file=out)
    print(f"  on CPU {rt} when the window began: " + (f"{occupant_at_lo[0]} ({occupant_at_lo[1]}), since {occupant_at_lo[2]} ns" if known
                                                    else "unknown (no switch onto it recorded before the window)"), file=out)
    print(f"  forbidden workers resident on CPU {rt} in the window: {len(resident)}" + (" (" + ", ".join(
        f"{c} {k} {(a - lo) / 1e6:+.3f}..{(b - lo) / 1e6:+.3f} ms" for c, _, k, a, b in resident[:5]) + (", ..." if len(resident) > 5 else "") + ")"
        if resident else ""), file=out)
    print(f"  forbidden executions on CPU {rt} in the window: {len(executed)}" + (" (" + ", ".join(
        f"{f} by {c} ({k})" for c, k, f, _, _ in executed[:5]) + (", ..." if len(executed) > 5 else "") + ")" if executed else ""), file=out)
    print(f"  CPU {rt}'s own per-CPU workers that ran in the window (not governed by the mask; counted, not judged): {own}", file=out)
    print(f"  the worker's flushes (its instructions to the emulator): {len(queued)} queued ("
          + (", ".join(f"{w} {n}" for w, n in wqs.items()) or "none") + f"), {len(ran)} seen to run, {unmatched} not", file=out)
    if delays:
        print("    ran on CPUs: " + ", ".join(f"{c} {n}" for c, n in sorted(cpus.items())) + f"; on CPU {rt}: {on_rt}", file=out)
        print(f"    queued to executed (descriptive): {tk_cost.dist(delays)}", file=out)
    if not queued:
        print("  no flush queued by the worker: nothing establishes where the instructions' flushes ran", file=out)
    ok = known and not resident and not executed and on_rt == 0 and unmatched == 0 and bool(queued)
    print(f"unbound work: {'PASS' if ok else 'FAIL'}", file=out)
    return dict(ok=ok, known=known, resident=len(resident), executed=len(executed), own=own, queued=len(queued), ran=len(ran),
                on_rt=on_rt, unmatched=unmatched, cpus=dict(cpus), delays=delays)


def run_report(csv_path: pathlib.Path, cap, rt: int, out) -> dict:
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
        r = analyse(proc.stdout, cap, rt, out)
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
    a = p.parse_args()
    try:
        cap = tk_cost.Capture(a.csv)
    except (OSError, KeyError, ValueError, SystemExit) as e:
        print(f"unbound_work: {a.csv}: unreadable ({e})", file=sys.stderr)
        return 2
    if cap.tid is None:
        print(f"unbound_work: {a.csv}: no worker_tid in the summary", file=sys.stderr)
        return 2
    rt = a.rt_cpu if a.rt_cpu is not None else tk_cost.rt_cpu_of(a.csv)
    print(f"{a.csv.name}: tools/unbound_work.py")
    try:
        if a.report:
            with open(a.report, errors="replace") as f:
                r = analyse(f, cap, rt, sys.stdout)
        else:
            r = run_report(a.csv, cap, rt, sys.stdout)
    except (OSError, Unreadable) as e:
        print(f"unbound_work: UNREADABLE: {e}", file=sys.stderr)
        return 2
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
