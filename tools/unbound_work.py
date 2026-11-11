#!/usr/bin/env python3
"""Where the unbound workqueues' work ran during a timer capture, for the loaded emulator validation's Option A
(docs/hardware/control-rate.md): with the real-time CPU out of the unbound workqueues' mask (tools/wq_mask.sh), none
of their work may run on it. Read-only.

    unbound_work.py <run>.csv [--report FILE] [--rt-cpu N]

Inside the probe's window (its first wake stamp to its last cycle-end stamp, as tools/tk_cost.py takes it), from the
main trace (trace-cmd report -t under tools/tk_record.py's launch, or a saved --report):

- every work execution (workqueue_execute_start) on the real-time CPU by an unbound worker (kworker/u*), and every
  execution of the tty layer's flush work (flush_to_ldisc) there by any task: both must be none;
- each flush the probe's worker queued (its instruction to the emulator: workqueue_queue_work, flush_to_ldisc), matched
  to that work's next execution: the CPUs they ran on, the delay from queueing to execution (descriptive), and those
  that ran on the real-time CPU or were never seen to run: both must be none.

Bound work (a per-CPU worker, kworker/N:M) on the real-time CPU is counted apart and does not fail: the mask does not
govern it. Exit: 0 none of the above; 1 some; 2 the evidence could not be read.
"""

from __future__ import annotations

import argparse
import collections
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tk_cost  # noqa: E402
import trace_correlate  # noqa: E402

AFTER_NS = 100_000_000  # executions are looked for this long after the window, for flushes queued near its end
QUEUE = re.compile(r"work struct=(\S+) function=(\S+) workqueue=(\S+)")
EXEC = re.compile(r"work struct (\S+?): function (\S+)")


class Unreadable(Exception):
    pass


def analyse(lines, cap: tk_cost.Capture, rt: int, out) -> dict:
    """The checks above on report lines; raises Unreadable if the report ends before the window does."""
    lo, hi = cap.window
    last = None
    rt_unbound: list = []
    rt_flush: list = []
    rt_bound: collections.Counter = collections.Counter()
    queued: list = []           # (ts, work, workqueue) of the worker's flushes, inside the window
    pending: dict = {}          # work -> index into queued, awaiting its execution
    ran: dict = {}              # index -> (ts, cpu, comm)
    for ln in lines:
        m = trace_correlate.LINE.match(ln.rstrip("\n"))
        if not m:
            continue
        ts = int(m["s"]) * 1_000_000_000 + int(m["ns"])
        last = ts
        if ts > hi + AFTER_NS:
            break
        ev, rest, cpu, comm = m["ev"], m["rest"], int(m["cpu"]), m["comm"].strip()
        if ev == "workqueue_queue_work" and int(m["pid"]) == cap.tid and lo <= ts <= hi:
            q = QUEUE.search(rest)
            if q and q[2] == "flush_to_ldisc":
                pending[q[1]] = len(queued)
                queued.append((ts, q[1], q[3]))
        elif ev == "workqueue_execute_start":
            x = EXEC.search(rest)
            if not x:
                continue
            k = pending.pop(x[1], None)
            if k is not None and ts >= queued[k][0]:
                ran[k] = (ts, cpu, comm)
            if cpu == rt and lo <= ts <= hi:
                if comm.startswith("kworker/u"):
                    rt_unbound.append((ts, comm, x[2]))
                elif x[2] != "flush_to_ldisc":
                    rt_bound[x[2]] += 1
                if x[2] == "flush_to_ldisc":
                    rt_flush.append((ts, comm))
    if last is None or last < hi:
        raise Unreadable(f"the report ends at {last} ns, before the window's end ({hi} ns)")
    delays = sorted(ran[k][0] - queued[k][0] for k in ran)
    cpus = collections.Counter(ran[k][1] for k in ran)
    on_rt = sum(1 for k in ran if ran[k][1] == rt)
    unmatched = len(queued) - len(ran)
    wqs = collections.Counter(q[2] for q in queued)
    print(f"window: {lo} to {hi} ns ({(hi - lo) / 1e9:.6f} s); real-time CPU {rt}", file=out)
    print(f"  unbound work executed on CPU {rt}: {len(rt_unbound)}" + (" (" + ", ".join(
        f"{f} by {c}" for _, c, f in rt_unbound[:5]) + (", ..." if len(rt_unbound) > 5 else "") + ")" if rt_unbound else ""), file=out)
    print(f"  flush_to_ldisc executed on CPU {rt}: {len(rt_flush)}", file=out)
    print(f"  bound work on CPU {rt} (not governed by the mask; counted, not judged): {sum(rt_bound.values())}"
          + (" (" + ", ".join(f"{f} {n}" for f, n in rt_bound.most_common(5)) + ")" if rt_bound else ""), file=out)
    print(f"  the worker's flushes (its instructions to the emulator): {len(queued)} queued ("
          + (", ".join(f"{w} {n}" for w, n in wqs.items()) or "none") + f"), {len(ran)} seen to run, {unmatched} not", file=out)
    if delays:
        print("    ran on CPUs: " + ", ".join(f"{c} {n}" for c, n in sorted(cpus.items())) + f"; on CPU {rt}: {on_rt}", file=out)
        print(f"    queued to executed (descriptive): {tk_cost.dist(delays)}", file=out)
    ok = not rt_unbound and not rt_flush and on_rt == 0 and unmatched == 0 and bool(queued)
    if not queued:
        print("  no flush queued by the worker: nothing establishes where the instructions' flushes ran", file=out)
    print(f"unbound work: {'PASS' if ok else 'FAIL'}", file=out)
    return dict(ok=ok, rt_unbound=len(rt_unbound), rt_flush=len(rt_flush), rt_bound=sum(rt_bound.values()), queued=len(queued),
                ran=len(ran), on_rt=on_rt, unmatched=unmatched, cpus=dict(cpus), delays=delays)


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("csv", type=pathlib.Path, help="the timer capture's probe log (<run>.csv)")
    p.add_argument("--report", type=pathlib.Path, help="a saved trace-cmd report -t of the capture (default: run trace-cmd)")
    p.add_argument("--rt-cpu", type=int, help="the real-time CPU (default: the run record's, else 3)")
    a = p.parse_args()
    try:
        cap = tk_cost.Capture(a.csv)
    except (OSError, KeyError, ValueError) as e:
        print(f"unbound_work: {a.csv}: unreadable ({e})", file=sys.stderr)
        return 2
    if cap.tid is None:
        print(f"unbound_work: {a.csv}: no worker_tid in the summary", file=sys.stderr)
        return 2
    rt = a.rt_cpu if a.rt_cpu is not None else tk_cost.rt_cpu_of(a.csv)
    print(f"{a.csv.name}: tools/unbound_work.py")
    try:
        r = analyse(tk_cost.report_lines(a.csv, a.report), cap, rt, sys.stdout)
    except (OSError, Unreadable, SystemExit) as e:  # tk_cost.report_lines reports a failed trace-cmd as SystemExit
        print(f"unbound_work: the report: {e}", file=sys.stderr)
        return 2
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
