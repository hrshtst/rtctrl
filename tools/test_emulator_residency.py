#!/usr/bin/env python3
"""Regressions for tools/emulator_residency.py on synthetic report lines, lineages and records.

Passes: the emulator woken and run on CPU 2 throughout, with the worker (a known task other than the emulator's) on
CPU 3 when the window began, or another task carried into the window there; the emulator on CPU 3 only before the
window; another task on CPU 3 inside it.

Fails: a switch to the emulator on CPU 3 inside the window; the emulator carried into the window on CPU 3 (a consistent
switch chain, switched out just after the window began: the reviewer's first counterexample); a wakeup of it onto CPU 3
and a migration of it onto CPU 3 (each left there to the window's end); the emulator queued on CPU 3 when the window
began and moved off it inside the window; attempt 15's sequence; a thread it creates that runs on CPU 3; a thread
created before the trace, absent from the supervisor's records, listed by the lineage after the trace started, that
runs on CPU 3 (the reviewer's second counterexample); a task the trace shows exiting before the lineage ended, seen on
CPU 3 in the window; a lineage thread allowed on CPU 3; an emulator never seen to run.

Unreadable: no switch onto CPU 3 before the window (its task then unknown); a switch chain that does not follow on; an
emulator task's event that does not follow from its known place (woken on CPU 2 while queued on CPU 3); no lineage; a
lineage begun before the trace's first event, or ended after the window began; a malformed required event (the exit
event's among them); a line that is no event; a report that ends inside the window. The lineage must exist, have its
header, its completeness line and its taken line, and list the emulator by pid and start time (its descendant
processes join, other processes do not); the record must hold all three emulator lines, the same pid and start time,
and affinities without CPU 3 (a failed check, a changed identity, a list with CPU 3, a malformed or descending list, a
missing line, a line twice: unreadable); the CLI reads the default or a given lineage, a stand-in decoder to its end,
and fails on its late nonzero exit."""

from __future__ import annotations

import io
import os
import pathlib
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import emulator_residency as er  # noqa: E402

FAILURES: list[str] = []
S = 1_000_000_000
T = 100 * S
MS = 1_000_000
EMU = 4100
WORKER = ("x7_bus_timing", 4242)
IDLE3 = ("swapper/3", 0)
TAKEN = (T - 50 * MS, T - 40 * MS)


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def line(t: int, cpu: int, comm: str, pid: int, ev: str, rest: str) -> str:
    return f"  {comm}-{pid}  [{cpu:03d}] d.... {t // S}.{t % S:09d}: {ev}: {rest}"


def switch(t: int, cpu: int, prev: tuple, nxt: tuple, state: str = "S") -> str:
    return line(t, cpu, prev[0], prev[1], "sched_switch", f"{prev[0]}:{prev[1]} [120] {state} ==> {nxt[0]}:{nxt[1]} [120]")


def wakeup(t: int, cpu: int, pid: int = EMU, comm: str = "dxl_emu", by: tuple = ("swapper/2", 0)) -> str:
    return line(t, cpu, by[0], by[1], "sched_wakeup", f"{comm}:{pid} [120] CPU:{cpu:03d}")


def migrate(t: int, orig: int, dest: int, pid: int = EMU, comm: str = "dxl_emu") -> str:
    return line(t, dest, "swapper/%d" % dest, 0, "sched_migrate_task", f"comm={comm} pid={pid} prio=120 orig_cpu={orig} dest_cpu={dest}")


def exits(t: int, pid: int, comm: str = "gone") -> str:
    return line(t, 5, comm, pid, "sched_process_exit", f"comm={comm} pid={pid} prio=120 group_dead=true")


def stamp(ln: str) -> int:
    s, ns = ln.split("] ")[1].split()[1].rstrip(":").split(".")
    return int(s) * S + int(ns)


def cycle(t: int, cpu: int = 2, pid: int = EMU, comm: str = "dxl_emu") -> list[str]:
    """The emulator's poll: woken on CPU cpu, run there 20 us later, asleep 50 us after that."""
    idle = (f"swapper/{cpu}", 0)
    return [wakeup(t, cpu, pid, comm), switch(t + 20_000, cpu, idle, (comm, pid)), switch(t + 70_000, cpu, (comm, pid), idle)]


def start() -> list[str]:
    """The trace's first event, before the lineage."""
    return [line(T - 100 * MS, 0, "swapper/0", 0, "sched_switch", "swapper/0:0 [120] R ==> x:1 [120]")]


def polls() -> list[str]:
    """Ten polls on CPU 2 in the 10 ms window; the trace runs past the window."""
    out = []
    for k in range(10):
        out += cycle(T + k * MS + 100_000)
    return out + [line(T + 11 * MS, 0, "swapper/0", 0, "sched_switch", "swapper/0:0 [120] R ==> x:1 [120]")]


def clean() -> list[str]:
    """The polls, with the worker switched onto CPU 3 before the window."""
    return start() + [switch(T - MS, 3, IDLE3, WORKER)] + polls()


class Cap:
    def __init__(self) -> None:
        self.window = (T, T + 10 * MS)


EMU_REC = dict(pid=EMU, start="1234", tids={EMU}, seeds={EMU}, taken=TAKEN, allowed_rt=[])


def check(lines: list[str], emu: dict | None = None) -> dict:
    return er.analyse(sorted(lines, key=stamp), Cap(), emu or EMU_REC, 3, io.StringIO())


def unreadable(lines: list[str], what: str, emu: dict | None = None) -> None:
    try:
        r = check(lines, emu)
        expect(False, f"{what}: read ({r})")
    except er.Unreadable:
        pass


def units() -> None:
    r = check(clean())
    expect(r["ok"] and r["seen"] == 10 and r["cpus"] == {2: 10} and r["occupant"] == WORKER[1]
           and r["ran_rt"] == r["placed_rt"] == r["moved_off"] == r["woken_rt"] == r["moved_rt"] == 0
           and r["waits"] == [20_000] * 10, f"clean, the worker on CPU 3 when the window began: {r}")
    other = ("other", 77)
    r = check(start() + [switch(T - MS, 3, IDLE3, other), switch(T + 10_000, 3, other, WORKER)] + polls())
    expect(r["ok"] and r["occupant"] == 77, f"another task carried into the window on CPU 3: {r}")
    t = T + 5 * MS + 300_000
    r = check(clean() + [switch(t, 3, WORKER, ("dxl_emu", EMU)), switch(t + 10_000, 3, ("dxl_emu", EMU), WORKER)])
    expect(not r["ok"] and r["ran_rt"] == 1, f"a switch to the emulator on CPU 3: {r}")
    # The reviewer's first counterexample: a consistent chain carries the emulator into the window on CPU 3.
    r = check(start() + [switch(T - MS, 3, IDLE3, ("dxl_emu", EMU)), switch(T + 10_000, 3, ("dxl_emu", EMU), WORKER)] + polls())
    expect(not r["ok"] and r["ran_rt"] == 1 and r["occupant"] == EMU, f"the emulator carried into the window on CPU 3: {r}")
    # The window's initial task must be known, and the chain must follow on.
    unreadable(start() + polls(), "no switch onto CPU 3 at all")
    unreadable(start() + polls() + [switch(T + MS, 3, IDLE3, WORKER)], "no switch onto CPU 3 before the window")
    unreadable(clean() + [switch(t, 3, ("other", 77), WORKER)], "a switch on CPU 3 that does not follow on")
    late = T + 9 * MS + 500_000  # after the last poll: nothing of the emulator follows to the window's end
    r = check(clean() + [wakeup(late, 3, by=WORKER)])
    expect(not r["ok"] and r["woken_rt"] == 1 and r["placed_rt"] == 1, f"a wakeup of the emulator onto CPU 3: {r}")
    r = check(clean() + [migrate(late, 2, 3)])
    expect(not r["ok"] and r["moved_rt"] == 1 and r["placed_rt"] == 1, f"a migration of the emulator onto CPU 3: {r}")
    # Queued on CPU 3 when the window began (woken there before it), moved off inside it.
    r = check(clean() + [wakeup(T - 500_000, 3, by=WORKER), migrate(T + 50_000, 3, 2)])
    expect(not r["ok"] and r["placed_rt"] == 1 and r["moved_off"] == 1 and r["woken_rt"] == 0,
           f"the emulator queued on CPU 3 when the window began: {r}")
    r = check(clean() + [migrate(T + 50_000, 3, 2)])
    expect(not r["ok"] and r["moved_off"] == 1 and r["placed_rt"] == 0, f"a migration of it off CPU 3, its first event: {r}")
    unreadable(clean() + [wakeup(T - 500_000, 3, by=WORKER)], "woken on CPU 2 while queued on CPU 3")
    # Attempt 15's sequence: pulled onto an idle CPU 3, asleep there, woken there, run there in the worker's place.
    t15 = T + 9 * MS + 200_000
    seq = [switch(t15, 3, WORKER, IDLE3), migrate(t15 + 2_000, 2, 3), switch(t15 + 6_000, 3, IDLE3, ("dxl_emu", EMU)),
           switch(t15 + 8_000, 3, ("dxl_emu", EMU), IDLE3), switch(t15 + 500_000, 3, IDLE3, WORKER),
           wakeup(t15 + 600_000, 3, by=WORKER), switch(t15 + 700_000, 3, WORKER, ("dxl_emu", EMU), "R+")]
    r = check(clean() + seq)
    expect(not r["ok"] and r["moved_rt"] == 1 and r["woken_rt"] == 1 and r["ran_rt"] == 2 and r["placed_rt"] == 2,
           f"attempt 15's sequence: {r}")
    # A thread the emulator creates, run on CPU 3.
    fork = line(T + 2 * MS, 2, "dxl_emu", EMU, "sched_process_fork", f"comm=dxl_emu pid={EMU} child_comm=dxl_emu child_pid={EMU + 1}")
    r = check(clean() + [fork, switch(t, 3, WORKER, ("dxl_emu", EMU + 1)), switch(t + 10_000, 3, ("dxl_emu", EMU + 1), WORKER)])
    expect(not r["ok"] and r["forks"] == 1 and r["ran_rt"] == 1, f"a thread it creates, run on CPU 3: {r}")
    # The reviewer's second counterexample: a thread created before the trace and absent from the records (here its
    # fork predates the trace), run on CPU 3 in the window. The lineage after the trace started lists it.
    pre = [switch(T + 2 * MS, 3, WORKER, ("dxl_emu", EMU + 1)), switch(T + 2 * MS + 50_000, 3, ("dxl_emu", EMU + 1), WORKER)]
    r = check(clean() + pre)
    expect(r["ok"], f"the records and the trace alone do not see it: {r}")
    r = check(clean() + pre, dict(EMU_REC, seeds={EMU, EMU + 1}))
    expect(not r["ok"] and r["ran_rt"] == 1, f"a thread in the lineage, absent from the records, run on CPU 3: {r}")
    # A task seen exiting before the lineage ended counts as the emulator's; one exiting later does not.
    r = check(clean() + pre + [exits(T - 45 * MS, EMU + 1)])
    expect(not r["ok"] and r["exited"] == 1 and r["ran_rt"] == 1, f"a task exited before the lineage ended, on CPU 3: {r}")
    r = check(clean() + pre + [exits(T - 30 * MS, EMU + 1)])
    expect(r["ok"] and r["exited"] == 0, f"a task exited after the lineage ended: {r}")
    r = check(clean(), dict(EMU_REC, allowed_rt=[EMU]))
    expect(not r["ok"] and r["allowed_rt"] == 1, f"a lineage thread allowed on CPU 3: {r}")
    # Outside the window (pulled to CPU 3, run there and asleep before it; woken there after it), and another task on
    # CPU 3 inside it: no count.
    r = check(clean() + [migrate(T - 500_000, 2, 3), switch(T - 400_000, 3, WORKER, ("dxl_emu", EMU), "R+"),
                         switch(T - 300_000, 3, ("dxl_emu", EMU), WORKER), wakeup(T + 10 * MS + 500_000, 3, by=WORKER),
                         switch(t, 3, WORKER, ("other", 77)), switch(t + 10_000, 3, ("other", 77), WORKER), wakeup(t, 3, 77, "other")])
    expect(r["ok"], f"the emulator on CPU 3 only outside the window, another task there inside it: {r}")
    r = check([ln for ln in clean() if "dxl_emu" not in ln])
    expect(not r["ok"] and r["seen"] == 0, f"the emulator never seen to run: {r}")
    # The lineage's timing.
    unreadable(clean(), "a lineage begun before the trace's first event", dict(EMU_REC, taken=(T - 200 * MS, T - 40 * MS)))
    unreadable(clean(), "a lineage ended after the window began", dict(EMU_REC, taken=(T - 50 * MS, T + 1)))
    unreadable(clean(), "no lineage", dict(pid=EMU, start="1234", tids={EMU}))
    for ev in ("sched_switch", "sched_wakeup", "sched_migrate_task", "sched_process_fork", "sched_process_exit"):
        unreadable(clean() + [line(T + 6 * MS, 3 if ev != "sched_switch" else 2, "x", 1, ev, "MALFORMED")], f"a malformed {ev}")
    rows = sorted(clean(), key=stamp)
    rows.insert(len(rows) // 2, "  garbage that is no event")
    try:
        er.analyse(rows, Cap(), EMU_REC, 3, io.StringIO())
        expect(False, "a line that is no event: read")
    except er.Unreadable:
        pass
    unreadable(sorted(clean(), key=stamp)[:14], "a report that ends inside the window")


GOOD = "pid=4100 start=1234 comm=dxl_emu tids=4100 allowed=0-2,4-31"
RECORD = ["started=x", f"emulator_start=t0 {GOOD}", f"emulator_A=t1 {GOOD}", f"emulator_B=t2 {GOOD}"]
HEAD = "--- task lineage (observer_affinity.py tasks: pid tid ppid pstart start allowed) ---"
ROWS = ["4000 4000 1 900 900 0-31", "4100 4100 4000 1234 1234 0-2,4-31", "4100 4101 4000 1234 1300 0-2,4-31",
        "4200 4200 4100 1400 1400 0-2,4-31", "4242 4242 4000 1500 1500 0-31", "4242 4243 4000 1500 1501 3"]


def lineage_text(rows: list[str] | None = None, taken: tuple = TAKEN, close: str | None = None) -> str:
    rows = ROWS if rows is None else rows
    return "\n".join([HEAD, f"taken monotonic_ns: {taken[0]} {taken[1]}"] + rows
                     + [close if close is not None else f"lineage complete: {len(rows)} tasks"]) + "\n"


def records() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = pathlib.Path(d, "experiment.record")
        p.write_text("\n".join(RECORD) + "\n")
        e = er.emulator(p, 3)
        expect(e == dict(pid=4100, start="1234", tids={4100}), f"a good record: {e}")
        two = RECORD[:3] + ["emulator_B=t2 pid=4100 start=1234 comm=dxl_emu tids=4100,4101 allowed=0-2,4-31;0-2"]
        p.write_text("\n".join(two) + "\n")
        e = er.emulator(p, 3)
        expect(e["tids"] == {4100, 4101}, f"a thread recorded later joins the set: {e}")
        for what, lines in (("a failed check", RECORD[:3] + ["emulator_B=t2 status=1 reason=x"]),
                            ("another process", RECORD[:3] + ["emulator_B=t2 pid=4200 start=999 comm=dxl_emu tids=4200 allowed=0-2,4-31"]),
                            ("a list with CPU 3", RECORD[:3] + ["emulator_B=t2 pid=4100 start=1234 comm=dxl_emu tids=4100 allowed=0-31"]),
                            ("a descending list", RECORD[:3] + ["emulator_B=t2 pid=4100 start=1234 comm=dxl_emu tids=4100 allowed=2-0"]),
                            ("lists not matching the threads", RECORD[:3] + ["emulator_B=t2 pid=4100 start=1234 comm=dxl_emu tids=4100,4101 allowed=0-2"]),
                            ("a missing line", RECORD[:3]),
                            ("a line twice", RECORD + [f"emulator_A=t3 {GOOD}"])):
            p.write_text("\n".join(lines) + "\n")
            try:
                er.emulator(p, 3)
                expect(False, f"{what}: read")
            except er.Unreadable:
                pass
        # The lineage: the emulator's process by pid and start time, its threads and its descendants' (4200), not
        # its parent's (4000) or the worker's (4242, whose thread 4243 is allowed only on CPU 3).
        emu = dict(pid=4100, start="1234", tids={4100})
        lf = pathlib.Path(d, "lineage.txt")
        lf.write_text(lineage_text())
        g = er.lineage(lf, emu, 3)
        expect(g == dict(seeds={4100, 4101, 4200}, procs={4100, 4200}, taken=TAKEN, allowed_rt=[]), f"a good lineage: {g}")
        lf.write_text(lineage_text(ROWS[:2] + ["4100 4101 4000 1234 1300 0-31"] + ROWS[3:]))
        g = er.lineage(lf, emu, 3)
        expect(g["allowed_rt"] == [4101], f"an emulator thread allowed on CPU 3: {g}")
        for what, text in (("no task lineage", "taken: x\n"),
                           ("an incomplete lineage", lineage_text(close="lineage INCOMPLETE: 1 unreadable: /proc/9 (x)")),
                           ("no completeness line", lineage_text(close="")),
                           ("an unreadable line", lineage_text(ROWS + ["4300 x"])),
                           ("no taken line", lineage_text().replace(f"taken monotonic_ns: {TAKEN[0]} {TAKEN[1]}\n", "")),
                           ("the emulator absent", lineage_text([r for r in ROWS if not r.startswith("4100 ")])),
                           ("another process with the emulator's pid", lineage_text([r.replace(" 1234 ", " 1999 ") for r in ROWS]))):
            lf.write_text(text)
            try:
                g = er.lineage(lf, emu, 3)
                expect(False, f"{what}: read ({g})")
            except er.Unreadable:
                pass
        try:
            er.lineage(pathlib.Path(d, "none"), emu, 3)
            expect(False, "a missing lineage: read")
        except er.Unreadable:
            pass


def cli() -> None:
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d, "run.csv")
        base.write_text("cycle,scheduled_wake_ns,wake_ns,cycle_end_ns\n1,0,0,1000\n2,1000000,1000000,10000000\n")
        pathlib.Path(str(base) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {T}\nworker_tid = 4242\n")
        rec = pathlib.Path(d, "experiment.record")
        rec.write_text("\n".join(RECORD) + "\n")
        lin = pathlib.Path(str(base) + ".lineage.txt")
        lin.write_text(lineage_text())
        rep = pathlib.Path(d, "report.txt")
        for lines, want in ((clean(), 0), (clean() + [wakeup(T + 9 * MS + 500_000, 3, by=WORKER)], 1), (sorted(clean(), key=stamp)[:14], 2)):
            rep.write_text("\n".join(sorted(lines, key=stamp)) + "\n")
            p = subprocess.run([sys.executable, "tools/emulator_residency.py", str(base), "--experiment", str(rec), "--report", str(rep),
                                "--rt-cpu", "3"], capture_output=True, text=True)
            expect(p.returncode == want, f"the CLI with a saved report, expecting {want}: {p.returncode} {p.stdout} {p.stderr}")
        rep.write_text("\n".join(sorted(clean(), key=stamp)) + "\n")
        dec = pathlib.Path(d, "decoder")
        for status, want in ((0, 0), (7, 2)):
            dec.write_text(f"#!/bin/sh\ncat {rep}\nexit {status}\n")
            dec.chmod(0o755)
            p = subprocess.run([sys.executable, "tools/emulator_residency.py", str(base), "--experiment", str(rec), "--rt-cpu", "3"],
                               capture_output=True, text=True,
                               env={**os.environ, "EMULATOR_RESIDENCY_TEST": "1", "EMULATOR_RESIDENCY_DECODER": str(dec)})
            expect(p.returncode == want, f"a decoder exiting {status} after its whole report: {p.returncode} {p.stdout} {p.stderr}")
        p = subprocess.run([sys.executable, "tools/emulator_residency.py", str(base), "--experiment", str(pathlib.Path(d, "none")),
                            "--report", str(rep), "--rt-cpu", "3"], capture_output=True, text=True)
        expect(p.returncode == 2, f"the CLI without a readable record: {p.returncode}")
        # A given lineage with a thread that runs on CPU 3 (the default one lacks it), then no lineage at all.
        other = pathlib.Path(d, "other-lineage.txt")
        other.write_text(lineage_text(ROWS + ["4100 4102 4000 1234 1310 0-2,4-31"]))
        rep.write_text("\n".join(sorted(clean() + [switch(T + 2 * MS, 3, WORKER, ("dxl_emu", 4102)),
                                                   switch(T + 2 * MS + 50_000, 3, ("dxl_emu", 4102), WORKER)], key=stamp)) + "\n")
        for extra, want in (([], 0), (["--lineage", str(other)], 1)):
            p = subprocess.run([sys.executable, "tools/emulator_residency.py", str(base), "--experiment", str(rec), "--report", str(rep),
                                "--rt-cpu", "3", *extra], capture_output=True, text=True)
            expect(p.returncode == want, f"the CLI with lineage {extra or 'by default'}, expecting {want}: {p.returncode} {p.stdout} {p.stderr}")
        lin.unlink()
        p = subprocess.run([sys.executable, "tools/emulator_residency.py", str(base), "--experiment", str(rec), "--report", str(rep),
                            "--rt-cpu", "3"], capture_output=True, text=True)
        expect(p.returncode == 2 and "no task lineage after the trace started" in p.stderr, f"the CLI without a lineage: {p.returncode} {p.stderr}")


def main() -> int:
    units()
    records()
    cli()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print("emulator residency regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
