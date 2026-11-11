#!/usr/bin/env python3
"""Regressions for tools/unbound_work.py on synthetic report lines: CPU 3 running only the probe's worker, idle and its
own per-CPU worker, with the worker's flushes run on CPU 2, passes; an unbound worker or a rescuer resident on CPU 3 in
the window, a per-CPU worker of another CPU there, work there by a task that is no worker, a flush there, a job begun
before the window that resumes inside it (the Option A review's case), a flush never seen to run, no flush, and an
unknown occupant at the window's start each fail; unbound work there outside the window does not count; a malformed
required event, a line that is no event, switches that do not follow on, and a report that ends inside the window are
unreadable; the CLI reads a stand-in decoder to its end and fails on its late nonzero exit."""

from __future__ import annotations

import io
import os
import pathlib
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import unbound_work as uw  # noqa: E402

FAILURES: list[str] = []
S = 1_000_000_000
T = 100 * S
TID = 4242
MS = 1_000_000
WORK = "0xffff8de1cb764408"
WORKER = ("x7_bus_timing", TID)
IDLE = ("swapper/3", 0)


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def line(t: int, cpu: int, comm: str, pid: int, ev: str, rest: str) -> str:
    return f"  {comm}-{pid}  [{cpu:03d}] d.... {t // S}.{t % S:09d}: {ev}: {rest}"


def switch(t: int, prev: tuple, nxt: tuple, cpu: int = 3) -> str:
    return line(t, cpu, prev[0], prev[1], "sched_switch", f"{prev[0]}:{prev[1]} [120] S ==> {nxt[0]}:{nxt[1]} [120]")


def queue(t: int, work: str = WORK) -> str:
    return line(t, 3, "x7_bus_timing", TID, "workqueue_queue_work",
                f"work struct={work} function=flush_to_ldisc workqueue=events_unbound req_cpu=8192 cpu=-1")


def run(t: int, cpu: int, comm: str = "kworker/u128:3", fn: str = "flush_to_ldisc", work: str = WORK, pid: int = 77, end: int | None = None) -> list[str]:
    out = [line(t, cpu, comm, pid, "workqueue_execute_start", f"work struct {work}: function {fn}")]
    if end is not None:
        out.append(line(end, cpu, comm, pid, "workqueue_execute_end", f"work struct {work}: function {fn}"))
    return out


def stamp(ln: str) -> int:
    s, ns = ln.split("] ")[1].split()[1].rstrip(":").split(".")
    return int(s) * S + int(ns)


def clean() -> list[str]:
    """CPU 3 runs the worker from before the window (10 ms long), sleeps to idle and back each millisecond, and runs its
    own per-CPU worker once; three instructions, each flushed 4 us later on CPU 2; the trace runs past the window."""
    out = [switch(T - MS, IDLE, WORKER)]
    for k in range(3):
        t = T + (k + 1) * MS
        out += [queue(t)] + run(t + 4_000, 2, end=t + 6_000)
        out += [switch(t + 600_000, WORKER, IDLE), switch(t + 1 * MS - 10_000, IDLE, WORKER)]
    own = ("kworker/3:1", 55)
    out += [switch(T + 5 * MS, WORKER, own)] + run(T + 5 * MS + 1_000, 3, "kworker/3:1", "vmstat_update", "0xffff0001", 55, T + 5 * MS + 2_000)
    out += [switch(T + 5 * MS + 3_000, own, WORKER), line(T + 11 * MS, 0, "swapper/0", 0, "sched_switch", "swapper/0:0 [120] R ==> x:1 [120]")]
    return out


class Cap:
    def __init__(self) -> None:
        self.window, self.tid = (T, T + 10 * MS), TID


def check(lines: list[str]) -> dict:
    return uw.analyse(sorted(lines, key=stamp), Cap(), 3, io.StringIO())


def visit(t0: int, t1: int, who: tuple, work: list[str]) -> list[str]:
    """who runs on CPU 3 from t0 to t1 (from the worker and back to it), doing work."""
    return [switch(t0, WORKER, who)] + work + [switch(t1, who, WORKER)]


def unreadable(lines: list[str], what: str) -> None:
    try:
        check(lines)
        expect(False, f"{what}: read")
    except uw.Unreadable:
        pass


def units() -> None:
    r = check(clean())
    expect(r["ok"] and r["known"] and r["queued"] == 3 and r["ran"] == 3 and r["cpus"] == {2: 3} and r["delays"] == [4000] * 3
           and r["own"] == 1 and r["resident"] == 0 and r["executed"] == 0, f"clean: {r}")
    u9 = ("kworker/u128:9", 88)
    t = T + 6 * MS + 100_000
    r = check(clean() + [queue(t)] + visit(t + 5_000, t + 20_000, ("kworker/u128:3", 77), run(t + 9_000, 3, end=t + 12_000)))
    expect(not r["ok"] and r["on_rt"] == 1 and r["resident"] == 1 and r["executed"] == 1, f"a flush on CPU 3: {r}")
    r = check(clean() + visit(t, t + 20_000, u9, run(t + 1_000, 3, "kworker/u128:9", "wb_update_bandwidth_workfn", "0xffff0002", 88, t + 5_000)))
    expect(not r["ok"] and r["resident"] == 1 and r["executed"] == 1, f"other unbound work on CPU 3: {r}")
    # The Option A review's case: an unbound job begun on CPU 3 before the window, preempted, resumed inside it.
    job = [switch(T - 2 * MS, WORKER, u9)] + run(T - MS, 3, "kworker/u128:9", "long_job", "0xffff000a", 88) + [
        switch(T - 100, u9, WORKER), switch(T + 500_000, WORKER, u9),
        line(T + 800_000, 3, "kworker/u128:9", 88, "workqueue_execute_end", "work struct 0xffff000a: function long_job"),
        switch(T + 900_000, u9, WORKER)]
    r = check([ln for ln in clean() if stamp(ln) != T - MS] + [switch(T - 3 * MS, IDLE, WORKER)] + job)
    expect(not r["ok"] and r["resident"] == 1 and r["executed"] == 1, f"a job begun before the window, resumed inside it: {r}")
    r = check(clean() + visit(t, t + 20_000, ("kworker/R-writeback", 66), run(t + 1_000, 3, "kworker/R-writeback", "wb_workfn", "0xffff0003", 66, t + 5_000)))
    expect(not r["ok"] and r["resident"] == 1 and r["executed"] == 1, f"a rescuer executing writeback work on CPU 3: {r}")
    r = check(clean() + visit(t, t + 20_000, ("kworker/5:1", 99), []))
    expect(not r["ok"] and r["resident"] == 1, f"another CPU's per-CPU worker on CPU 3: {r}")
    r = check(clean() + visit(t, t + 20_000, ("kworker/x", 98), []))
    expect(not r["ok"] and r["resident"] == 1, f"an unclassified worker on CPU 3: {r}")
    r = check(clean() + run(t + 1_000, 3, "bash", "odd_fn", "0xffff0004", TID, t + 2_000))
    expect(not r["ok"] and r["executed"] == 1, f"work executed by a task that is no worker: {r}")
    out = [ln for ln in clean() if stamp(ln) != T - MS] + [switch(T - 3 * MS, IDLE, u9), switch(T - 2 * MS, u9, WORKER),
                                                           switch(T + 10 * MS + 500_000, WORKER, u9), switch(T + 10 * MS + 600_000, u9, WORKER)]
    r = check(out)
    expect(r["ok"] and r["resident"] == 0, f"an unbound worker on CPU 3 only outside the window: {r}")
    r = check(clean() + [queue(T + 8 * MS, "0xffff0005")])
    expect(not r["ok"] and r["unmatched"] == 1, f"a flush never seen to run: {r}")
    r = check([ln for ln in clean() if "workqueue_queue_work" not in ln])
    expect(not r["ok"] and r["queued"] == 0, f"no flush queued: {r}")
    r = check([ln for ln in clean() if stamp(ln) != T - MS])
    expect(not r["ok"] and not r["known"], f"no switch onto CPU 3 before the window: {r}")
    unreadable(clean() + [line(T + 6 * MS, 3, "kworker/u128:9", 88, "workqueue_execute_start", "MALFORMED")], "a malformed execution (the review's case)")
    unreadable(clean() + [line(T + 6 * MS, 3, "x", 1, "sched_switch", "MALFORMED")], "a malformed switch")
    rows = sorted(clean(), key=stamp)
    rows.insert(len(rows) // 2, "  garbage that is no event")
    try:
        uw.analyse(rows, Cap(), 3, io.StringIO())
        expect(False, "a line that is no event: read")
    except uw.Unreadable:
        pass
    unreadable(clean() + [switch(T + 7 * MS, ("bash", 5), WORKER)], "a switch that does not follow on")
    unreadable(clean()[:6], "a report that ends inside the window")


def cli() -> None:
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d, "run.csv")
        base.write_text("cycle,scheduled_wake_ns,wake_ns,cycle_end_ns\n1,0,0,1000\n2,1000000,1000000,10000000\n")
        pathlib.Path(str(base) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {T}\nworker_tid = {TID}\n")
        rep = pathlib.Path(d, "report.txt")
        flush_on_3 = clean() + [queue(T + 6 * MS)] + visit(T + 6 * MS + 5_000, T + 6 * MS + 20_000, ("kworker/u128:3", 77),
                                                             run(T + 6 * MS + 9_000, 3, end=T + 6 * MS + 12_000))
        for lines, want in ((clean(), 0), (flush_on_3, 1), (clean()[:6], 2)):
            rep.write_text("\n".join(sorted(lines, key=stamp)) + "\n")
            p = subprocess.run([sys.executable, "tools/unbound_work.py", str(base), "--report", str(rep), "--rt-cpu", "3"],
                               capture_output=True, text=True)
            expect(p.returncode == want, f"the CLI with a saved report, expecting {want}: {p.returncode} {p.stdout} {p.stderr}")
        # A stand-in decoder: the whole clean report, then its exit status; a late nonzero exit makes the evidence unreadable.
        rep.write_text("\n".join(sorted(clean(), key=stamp)) + "\n")
        dec = pathlib.Path(d, "decoder")
        for status, want in ((0, 0), (7, 2)):
            dec.write_text(f"#!/bin/sh\ncat {rep}\nexit {status}\n")
            dec.chmod(0o755)
            p = subprocess.run([sys.executable, "tools/unbound_work.py", str(base), "--rt-cpu", "3"], capture_output=True, text=True,
                               env={**os.environ, "UNBOUND_WORK_TEST": "1", "UNBOUND_WORK_DECODER": str(dec)})
            expect(p.returncode == want, f"a decoder exiting {status} after its whole report: {p.returncode} {p.stdout} {p.stderr}")


def main() -> int:
    units()
    cli()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print("unbound work regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
