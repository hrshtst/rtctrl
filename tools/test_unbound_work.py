#!/usr/bin/env python3
"""Regressions for tools/unbound_work.py on synthetic report lines: CPU 3 running only the probe's worker, idle and its
own per-CPU worker, with the worker's flushes run on CPU 2, passes; an unbound worker or a rescuer resident on CPU 3 in
the window, a per-CPU worker of another CPU there, work there by a task that is no worker, a flush there, a job begun
before the window that resumes inside it (the Option A review's case), a flush never seen to run, no flush, and an
unknown occupant at the window's start each fail; unbound work there outside the window does not count; a malformed
required event, a line that is no event, switches that do not follow on, and a report that ends inside the window are
unreadable; the CLI reads a stand-in decoder to its end and fails on its late nonzero exit.

With --experiment and --allowed 4-5 (plan L1): both directions' flushes run on 4-5 pass (and the emulator's alone without
the option); the worker's flushes on CPU 2 with no reply flush, one of them run on CPU 2 (by a per-CPU worker: the
flush's own check; by an unbound worker: both checks), a reply flush run on CPU 6, one never seen to run, one run on
CPU 3 (also without --allowed), one by a task the emulator forked during the trace run on CPU 6, and unbound or rescuer
work on CPU 0 in the window each fail; unbound work there after the window, a per-CPU worker there, and another task's
tty flush do not count; --allowed without the emulator, a set holding CPU 3, a lineage ending after the window began or
beginning before the trace, and a malformed fork or exit (required only with --experiment) are unreadable; the CLI needs
--experiment for --allowed, refuses a set holding CPU 3, a list not in canonical form, a set narrower than the run's,
and a record allowing the emulator outside the set.

Executions followed across their task's switches (the L1 code review's finding): an excursion to CPU 6 between a start
and an end on CPU 4 fails, for other unbound work, for the worker's flush (also run by a per-CPU worker's name: the
flush's own check) and for an emulator's reply flush; moves within 4-5 and a preemption resumed on CPU 4 pass; an
execution carried into the window fails on CPU 6 across the window's start or before it (the whole execution is
judged once it overlaps the window) and passes on 5; one begun before the report fails on CPU 6 in the window and
passes on 5 then 4; one still running on CPU 6 when the report ends fails; without --allowed, a flush that moves onto
CPU 3 mid-execution counts as run there; an end where its task was not running, an executing task switched in twice,
a start while its last execution is open, an end with no start after its last end, and an end with no start whose
earlier switches did not follow on are unreadable."""

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


EMU = 4100
REPLY = "0xffff8de1cb7a0010"
EMU_REC = dict(pid=EMU, start="1234", tids={EMU}, seeds={EMU}, taken=(T - 50 * MS, T - 40 * MS), allowed_rt=[], allowed_outside=[])


def emu_queue(t: int, pid: int = EMU, work: str = REPLY, cpu: int = 4) -> str:
    return line(t, cpu, "dxl_emu", pid, "workqueue_queue_work",
                f"work struct={work} function=flush_to_ldisc workqueue=events_unbound req_cpu=8192 cpu=-1")


def l1() -> list[str]:
    """Plan L1's layout: the trace's first event before the lineage; the worker's three flushes run on CPU 4, and after
    each the emulator's reply queued on CPU 4 and run on CPU 5."""
    out = [line(T - 100 * MS, 0, "swapper/0", 0, "sched_switch", "swapper/0:0 [120] R ==> x:1 [120]"), switch(T - MS, IDLE, WORKER)]
    for k in range(3):
        t = T + (k + 1) * MS
        out += [queue(t)] + run(t + 4_000, 4, end=t + 6_000)
        out += [emu_queue(t + 50_000)] + run(t + 55_000, 5, "kworker/u128:4", work=REPLY, pid=78, end=t + 57_000)
        out += [switch(t + 600_000, WORKER, IDLE), switch(t + 1 * MS - 10_000, IDLE, WORKER)]
    return out + [line(T + 11 * MS, 0, "swapper/0", 0, "sched_switch", "swapper/0:0 [120] R ==> x:1 [120]")]


def allowed_units() -> None:
    """--experiment (the emulator's flushes) and --allowed 4-5 (plan L1)."""
    def chk(lines: list[str], allowed: str | None = "4-5", emu: dict | None = EMU_REC) -> dict:
        return uw.analyse(sorted(lines, key=stamp), Cap(), 3, io.StringIO(), allowed, emu)
    r = chk(l1())
    expect(r["ok"] and r["emu_queued"] == 3 and r["emu_ran"] == 3 and r["cpus"] == {4: 3} and r["outside"] == 0,
           f"L1: both directions on 4-5: {r}")
    r = chk(l1(), None)
    expect(r["ok"] and r["emu_queued"] == 3, f"the emulator's flushes without --allowed: {r}")
    r = check(l1())
    expect(r["ok"] and "emu_queued" in r and r["emu_queued"] == 0, f"L1's trace without --experiment: {r}")
    r = chk([line(T - 100 * MS, 0, "swapper/0", 0, "sched_switch", "swapper/0:0 [120] R ==> x:1 [120]")] + clean())
    expect(not r["ok"] and r["off_instr"] == 3 and r["emu_queued"] == 0, f"L1: the worker's flushes on CPU 2, no reply flush: {r}")
    t = T + 7 * MS
    r = chk(l1() + [emu_queue(t, work="0xffff00e1")] + run(t + 5_000, 6, "kworker/u128:5", work="0xffff00e1", pid=79, end=t + 7_000))
    expect(not r["ok"] and r["emu_off"] == 1 and r["emu_on_rt"] == 0, f"L1: a reply flush run on CPU 6: {r}")
    # One of the worker's flushes run on CPU 2 by a per-CPU worker (outside the unbound executions' check): the flush's
    # own check fails it; run by an unbound worker, both checks do.
    r = chk(l1() + [queue(t, "0xffff00ea")] + run(t + 4_000, 2, "kworker/2:1", work="0xffff00ea", pid=57, end=t + 6_000))
    expect(not r["ok"] and r["off_instr"] == 1 and r["emu_off"] == 0 and r["outside"] == 0,
           f"L1: one of the worker's flushes run on CPU 2 by a per-CPU worker: {r}")
    r = chk(l1() + [queue(t, "0xffff00eb")] + run(t + 4_000, 2, work="0xffff00eb", end=t + 6_000))
    expect(not r["ok"] and r["off_instr"] == 1 and r["outside"] == 1, f"L1: one of the worker's flushes run on CPU 2: {r}")
    r = chk(l1() + [emu_queue(t, work="0xffff00e2")])
    expect(not r["ok"] and r["emu_unmatched"] == 1, f"L1: a reply flush never seen to run: {r}")
    r = chk(l1() + [emu_queue(t, work="0xffff00e3")] + visit(t + 5_000, t + 20_000, ("kworker/u128:3", 77),
                                                              run(t + 9_000, 3, work="0xffff00e3", end=t + 12_000)), None)
    expect(not r["ok"] and r["emu_on_rt"] == 1, f"a reply flush run on CPU 3, without --allowed: {r}")
    u9 = ("kworker/u128:9", 88)
    r = chk(l1() + run(t, 0, u9[0], "wb_workfn", "0xffff00e4", u9[1], t + 3_000))
    expect(not r["ok"] and r["outside"] == 1, f"L1: other unbound work on CPU 0 in the window: {r}")
    r = chk(l1() + run(T + 10 * MS + 500_000, 0, u9[0], "wb_workfn", "0xffff00e5", u9[1], T + 10 * MS + 600_000))
    expect(r["ok"] and r["outside"] == 0, f"L1: unbound work on CPU 0 only after the window: {r}")
    r = chk(l1() + run(t, 0, "kworker/R-writeback", "wb_workfn", "0xffff00e6", 66, t + 3_000))
    expect(not r["ok"] and r["outside"] == 1, f"L1: a rescuer on CPU 0 in the window: {r}")
    r = chk(l1() + run(t, 0, "kworker/0:1", "vmstat_update", "0xffff00e7", 56, t + 3_000))
    expect(r["ok"], f"L1: a per-CPU worker on CPU 0 (not governed by the mask): {r}")
    r = chk(l1() + [line(t, 1, "ghostty", 5555, "workqueue_queue_work",
                         "work struct=0xffff00e8 function=flush_to_ldisc workqueue=events_unbound req_cpu=8192 cpu=-1")]
            + run(t + 5_000, 4, "kworker/u128:5", work="0xffff00e8", pid=79, end=t + 7_000))
    expect(r["ok"] and r["emu_queued"] == 3, f"another task's tty flush is not the emulator's: {r}")
    # A task the emulator forked during the trace queues a reply: it is the emulator's, and judged.
    fork = line(T - 20 * MS, 4, "dxl_emu", EMU, "sched_process_fork", f"comm=dxl_emu pid={EMU} child_comm=dxl_emu child_pid={EMU + 1}")
    r = chk(l1() + [fork, emu_queue(t, EMU + 1, "0xffff00e9")] + run(t + 5_000, 6, "kworker/u128:5", work="0xffff00e9", pid=79, end=t + 7_000))
    expect(not r["ok"] and r["emu_queued"] == 4 and r["emu_off"] == 1, f"L1: a reply flush by the emulator's child, on CPU 6: {r}")
    for what, kw, lines in (("--allowed without the emulator", dict(emu=None), l1()),
                            ("a set holding the real-time CPU", dict(allowed="3-5"), l1()),
                            ("a lineage ended after the window began", dict(emu=dict(EMU_REC, taken=(T - 50 * MS, T + 1))), l1()),
                            ("a lineage begun before the trace's first event", dict(emu=dict(EMU_REC, taken=(T - 200 * MS, T - 40 * MS))), l1()),
                            ("a malformed fork, with the emulator", {}, l1() + [line(t, 2, "x", 1, "sched_process_fork", "MALFORMED")]),
                            ("a malformed exit, with the emulator", {}, l1() + [line(t, 2, "x", 1, "sched_process_exit", "MALFORMED")])):
        try:
            chk(lines, **kw)
            expect(False, f"L1: {what}: read")
        except uw.Unreadable:
            pass
    r = check(clean() + [line(T + 6 * MS, 2, "x", 1, "sched_process_fork", "MALFORMED")])
    expect(r["ok"], f"a malformed fork without --experiment (not required): {r}")


def swc(t: int, cpu: int, prev: tuple, nxt: tuple, state: str = "S") -> str:
    """A switch on any CPU."""
    return line(t, cpu, prev[0], prev[1], "sched_switch", f"{prev[0]}:{prev[1]} [120] {state} ==> {nxt[0]}:{nxt[1]} [120]")


def legs(who: tuple, fn: str, work: str, t0: int, path: list, start: bool = True, end: bool = True) -> list[str]:
    """who executes work across path [(cpu, ns)]: switched in on each leg's CPU from its idle task and out (preempted,
    except after the last) to it, 20 us between legs; the execution starts 10 us into the first leg and ends 10 us before
    the last one's end (start or end False: begun before the report, or still open)."""
    out, t = [], t0
    for i, (cpu, d) in enumerate(path):
        idle = (f"swapper/{cpu}", 0)
        out.append(swc(t, cpu, idle, who))
        if i == 0 and start:
            out.append(line(t + 10_000, cpu, who[0], who[1], "workqueue_execute_start", f"work struct {work}: function {fn}"))
        if i == len(path) - 1 and end:
            out.append(line(t + d - 10_000, cpu, who[0], who[1], "workqueue_execute_end", f"work struct {work}: function {fn}"))
        out.append(swc(t + d, cpu, who, idle, "R" if i < len(path) - 1 else "S"))
        t += d + 20_000
    return out


def followed_units() -> None:
    """Executions followed across their task's switches (the L1 code review's finding): every CPU an execution ran on is
    judged, not only where it began and ended."""
    def chk(lines: list[str], allowed: str | None = "4-5") -> dict:
        return uw.analyse(sorted(lines, key=stamp), Cap(), 3, io.StringIO(), allowed, EMU_REC)
    u9, u6, pc = ("kworker/u128:9", 88), ("kworker/u128:6", 90), ("kworker/4:1", 91)
    t = T + 6 * MS
    away, within, still = [(4, 100_000), (6, 880_000), (4, 100_000)], [(4, 100_000), (5, 880_000), (4, 100_000)], \
        [(4, 100_000), (4, 300_000)]
    # Other unbound work: an excursion to CPU 6 between a start and an end on CPU 4 fails; a move within 4-5 and a
    # preemption resumed on CPU 4 pass.
    r = chk(l1() + legs(u9, "wb_workfn", "0xffff00f1", t, away))
    expect(not r["ok"] and r["outside"] == 1 and r["moved"] == 1, f"followed: unbound work 4 -> 6 -> 4: {r}")
    r = chk(l1() + legs(u9, "wb_workfn", "0xffff00f1", t, within))
    expect(r["ok"] and r["outside"] == 0 and r["moved"] == 1, f"followed: unbound work 4 -> 5 -> 4 (within the set): {r}")
    r = chk(l1() + legs(u9, "wb_workfn", "0xffff00f1", t, still))
    expect(r["ok"] and r["outside"] == 0 and r["moved"] == 0, f"followed: unbound work preempted, resumed on CPU 4: {r}")
    # The worker's flushes (instructions): the same excursion fails the flush's own check, by an unbound worker and by a
    # worker outside that class (whose executions the unbound check leaves aside); a move within 4-5 passes.
    r = chk(l1() + [queue(t - 5_000, "0xffff00f2")] + legs(u6, "flush_to_ldisc", "0xffff00f2", t, away))
    expect(not r["ok"] and r["off_instr"] == 1 and r["outside"] == 1 and r["instr_moved"] == 1,
           f"followed: an instruction flush 4 -> 6 -> 4: {r}")
    r = chk(l1() + [queue(t - 5_000, "0xffff00f2")] + legs(pc, "flush_to_ldisc", "0xffff00f2", t, away))
    expect(not r["ok"] and r["off_instr"] == 1 and r["outside"] == 0,
           f"followed: an instruction flush 4 -> 6 -> 4 by a per-CPU worker's name: {r}")
    r = chk(l1() + [queue(t - 5_000, "0xffff00f2")] + legs(u6, "flush_to_ldisc", "0xffff00f2", t, within))
    expect(r["ok"] and r["off_instr"] == 0 and r["instr_moved"] == 1, f"followed: an instruction flush 4 -> 5 -> 4: {r}")
    # The emulator's flushes (replies), the other direction: likewise.
    r = chk(l1() + [emu_queue(t - 5_000, work="0xffff00f3")] + legs(u6, "flush_to_ldisc", "0xffff00f3", t, [(5, 100_000), (6, 880_000), (5, 100_000)]))
    expect(not r["ok"] and r["emu_off"] == 1 and r["emu_moved"] == 1, f"followed: a reply flush 5 -> 6 -> 5: {r}")
    r = chk(l1() + [emu_queue(t - 5_000, work="0xffff00f3")] + legs(u6, "flush_to_ldisc", "0xffff00f3", t, [(5, 100_000), (4, 880_000)]))
    expect(r["ok"] and r["emu_off"] == 0 and r["emu_moved"] == 1 and r["emu_ran"] == 4, f"followed: a reply flush 5 -> 4: {r}")
    # Carried into the window: begun on CPU 4 before it, on CPU 6 across its start, back on 4 inside it, fails; on 5
    # instead passes; an excursion before the window also fails (the whole execution is judged once it overlaps the
    # window, as its start CPU always was).
    c0 = T - 2 * MS
    r = chk(l1() + legs(u9, "long_job", "0xffff00f4", c0, [(4, 1_500_000), (6, 880_000), (4, 100_000)]))
    expect(not r["ok"] and r["outside"] == 1, f"followed: carried in, 4 -> 6 (across the window's start) -> 4: {r}")
    r = chk(l1() + legs(u9, "long_job", "0xffff00f4", c0, [(4, 1_500_000), (5, 880_000), (4, 100_000)]))
    expect(r["ok"] and r["outside"] == 0 and r["moved"] == 1, f"followed: carried in, 4 -> 5 -> 4: {r}")
    r = chk(l1() + legs(u9, "long_job", "0xffff00f4", c0, [(4, 1_000_000), (6, 300_000), (4, 1_000_000)]))
    expect(not r["ok"] and r["outside"] == 1, f"followed: carried in, its excursion to CPU 6 before the window: {r}")
    # Begun before the report (an end with no start): its task followed from the report's first event.
    r = chk(l1() + legs(u9, "long_job", "0xffff00f5", T + 100_000, [(6, 300_000), (4, 100_000)], start=False))
    expect(not r["ok"] and r["outside"] == 1, f"followed: begun before the report, on CPU 6 in the window: {r}")
    r = chk(l1() + legs(u9, "long_job", "0xffff00f5", T + 100_000, [(5, 300_000), (4, 100_000)], start=False))
    expect(r["ok"] and r["outside"] == 0, f"followed: begun before the report, on 5 then 4: {r}")
    # Still open, and running on CPU 6, when the report ends: followed to its end.
    r = chk(l1() + legs(u9, "long_job", "0xffff00f6", t, [(4, 100_000)], end=False) + [swc(t + 120_000, 6, ("swapper/6", 0), u9)])
    expect(not r["ok"] and r["outside"] == 1, f"followed: still open at the report's end, running on CPU 6: {r}")
    # Without --allowed (Option A): a flush that moves onto the real-time CPU mid-execution counts as run there.
    w = T + 6 * MS + 100_000
    via3 = ([queue(w - 5_000, "0xffff00f7"), swc(w, 2, ("swapper/2", 0), u6),
             line(w + 10_000, 2, u6[0], u6[1], "workqueue_execute_start", "work struct 0xffff00f7: function flush_to_ldisc"),
             swc(w + 20_000, 2, u6, ("swapper/2", 0), "R")] + visit(w + 30_000, w + 60_000, u6, [])
            + [swc(w + 70_000, 2, ("swapper/2", 0), u6),
               line(w + 80_000, 2, u6[0], u6[1], "workqueue_execute_end", "work struct 0xffff00f7: function flush_to_ldisc"),
               swc(w + 90_000, 2, u6, ("swapper/2", 0))])
    r = check(clean() + via3)
    expect(not r["ok"] and r["on_rt"] == 1 and r["executed"] == 1 and r["resident"] == 1,
           f"followed: an instruction flush 2 -> 3 -> 2, without --allowed: {r}")
    # Evidence that does not follow on: unreadable.
    for what, extra in (("an end where its task was not running",
                         [swc(t, 6, ("swapper/6", 0), u9),
                          line(t + 10_000, 6, u9[0], u9[1], "workqueue_execute_start", "work struct 0xffff00f8: function x"),
                          line(t + 20_000, 4, u9[0], u9[1], "workqueue_execute_end", "work struct 0xffff00f8: function x")]),
                        ("an executing task switched in twice",
                         [line(t, 6, u9[0], u9[1], "workqueue_execute_start", "work struct 0xffff00f8: function x"),
                          swc(t + 10_000, 4, ("swapper/4", 0), u9)]),
                        ("a start while its task's execution is open",
                         [line(t, 6, u9[0], u9[1], "workqueue_execute_start", "work struct 0xffff00f8: function x"),
                          line(t + 10_000, 6, u9[0], u9[1], "workqueue_execute_start", "work struct 0xffff00f9: function x")]),
                        ("an end with no start after its task's last end",
                         legs(u9, "x", "0xffff00f8", t, still)
                         + [line(t + 500_000, 4, u9[0], u9[1], "workqueue_execute_end", "work struct 0xffff00f9: function x")]),
                        ("an end with no start whose task's switches did not follow on",
                         [swc(t, 6, ("swapper/6", 0), u9), swc(t + 10_000, 4, ("swapper/4", 0), u9),
                          line(t + 20_000, 4, u9[0], u9[1], "workqueue_execute_end", "work struct 0xffff00f8: function x")])):
        try:
            chk(l1() + extra)
            expect(False, f"followed: {what}: read")
        except uw.Unreadable:
            pass


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
        # --allowed needs --experiment; with both, a record and a lineage on disk.
        rep.write_text("\n".join(sorted(l1(), key=stamp)) + "\n")
        p = subprocess.run([sys.executable, "tools/unbound_work.py", str(base), "--report", str(rep), "--rt-cpu", "3", "--allowed", "4-5"],
                           capture_output=True, text=True)
        expect(p.returncode == 2, f"the CLI with --allowed and no --experiment: {p.returncode} {p.stderr}")
        rec = pathlib.Path(d, "experiment.record")
        good = "pid=4100 start=1234 comm=dxl_emu tids=4100 allowed=4-5"
        rec.write_text(f"started=x\nemulator_start=t0 {good}\nemulator_A=t1 {good}\nemulator_B=t2 {good}\n")
        pathlib.Path(str(base) + ".lineage.txt").write_text(
            "--- task lineage (observer_affinity.py tasks: pid tid ppid pstart start allowed) ---\n"
            f"taken monotonic_ns: {T - 50 * MS} {T - 40 * MS}\n4100 4100 1 1234 1234 4-5\nlineage complete: 1 tasks\n")
        for extra, want in ((["--allowed", "4-5"], 0), (["--allowed", "5"], 2), ([], 0), (["--allowed", "3-5"], 2), (["--allowed", "4,5"], 2)):
            p = subprocess.run([sys.executable, "tools/unbound_work.py", str(base), "--report", str(rep), "--rt-cpu", "3",
                                "--experiment", str(rec)] + extra, capture_output=True, text=True)
            expect(p.returncode == want, f"the CLI with --experiment {extra}: {p.returncode} {p.stdout} {p.stderr}")
        rec.write_text(f"started=x\nemulator_start=t0 {good.replace('4-5', '0-2,4-31')}\nemulator_A=t1 {good}\nemulator_B=t2 {good}\n")
        p = subprocess.run([sys.executable, "tools/unbound_work.py", str(base), "--report", str(rep), "--rt-cpu", "3",
                            "--experiment", str(rec), "--allowed", "4-5"], capture_output=True, text=True)
        expect(p.returncode == 2, f"the CLI with an emulator record allowed outside 4-5: {p.returncode} {p.stderr}")


def main() -> int:
    units()
    allowed_units()
    followed_units()
    cli()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print("unbound work regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
