#!/usr/bin/env python3
"""Regressions for tools/emulator_residency.py on synthetic report lines and records: the emulator woken and run on CPU 2
throughout passes; a switch to it on CPU 3, a wakeup of it onto CPU 3 and a migration of it onto CPU 3 inside the window
each fail (attempt 15's sequence among them), as does a thread it creates that runs on CPU 3, and an emulator never seen
to run; its events on CPU 3 outside the window, and another task's on CPU 3, do not count; a malformed required event, a
line that is no event and a report that ends inside the window are unreadable; the record must hold all three emulator
lines, the same pid and start time, and affinities without CPU 3 (a failed check, a changed identity, a list with CPU 3,
a malformed or descending list, a missing line, a line twice: unreadable); the CLI reads a stand-in decoder to its end
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


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def line(t: int, cpu: int, comm: str, pid: int, ev: str, rest: str) -> str:
    return f"  {comm}-{pid}  [{cpu:03d}] d.... {t // S}.{t % S:09d}: {ev}: {rest}"


def switch(t: int, cpu: int, prev: tuple, nxt: tuple, state: str = "S") -> str:
    return line(t, cpu, prev[0], prev[1], "sched_switch", f"{prev[0]}:{prev[1]} [120] {state} ==> {nxt[0]}:{nxt[1]} [120]")


def wakeup(t: int, cpu: int, pid: int = EMU, comm: str = "dxl_emu", by: tuple = ("swapper/2", 0)) -> str:
    return line(t, cpu if cpu != 3 else 3, by[0], by[1], "sched_wakeup", f"{comm}:{pid} [120] CPU:{cpu:03d}")


def migrate(t: int, orig: int, dest: int, pid: int = EMU, comm: str = "dxl_emu") -> str:
    return line(t, dest, "swapper/%d" % dest, 0, "sched_migrate_task", f"comm={comm} pid={pid} prio=120 orig_cpu={orig} dest_cpu={dest}")


def stamp(ln: str) -> int:
    s, ns = ln.split("] ")[1].split()[1].rstrip(":").split(".")
    return int(s) * S + int(ns)


def cycle(t: int, cpu: int = 2, pid: int = EMU, comm: str = "dxl_emu") -> list[str]:
    """The emulator's poll: woken on CPU cpu, run there 20 us later, asleep 50 us after that."""
    idle = (f"swapper/{cpu}", 0)
    return [wakeup(t, cpu, pid, comm), switch(t + 20_000, cpu, idle, (comm, pid)), switch(t + 70_000, cpu, (comm, pid), idle)]


def clean() -> list[str]:
    """Ten polls on CPU 2 in the 10 ms window; the worker on CPU 3; the trace runs past the window."""
    out = [switch(T - MS, 3, ("swapper/3", 0), WORKER)]
    for k in range(10):
        out += cycle(T + k * MS + 100_000)
    out += [line(T + 11 * MS, 0, "swapper/0", 0, "sched_switch", "swapper/0:0 [120] R ==> x:1 [120]")]
    return out


class Cap:
    def __init__(self) -> None:
        self.window = (T, T + 10 * MS)


EMU_REC = dict(pid=EMU, start="1234", tids={EMU})


def check(lines: list[str], emu: dict | None = None) -> dict:
    return er.analyse(sorted(lines, key=stamp), Cap(), emu or EMU_REC, 3, io.StringIO())


def unreadable(lines: list[str], what: str) -> None:
    try:
        check(lines)
        expect(False, f"{what}: read")
    except er.Unreadable:
        pass


def units() -> None:
    r = check(clean())
    expect(r["ok"] and r["seen"] == 10 and r["cpus"] == {2: 10} and r["runs_rt"] == r["woken_rt"] == r["moved_rt"] == 0
           and r["waits"] == [20_000] * 10, f"clean: {r}")
    t = T + 5 * MS + 300_000
    r = check(clean() + [switch(t, 3, WORKER, ("dxl_emu", EMU)), switch(t + 10_000, 3, ("dxl_emu", EMU), WORKER)])
    expect(not r["ok"] and r["runs_rt"] == 1, f"a switch to the emulator on CPU 3: {r}")
    r = check(clean() + [wakeup(t, 3, by=WORKER)])
    expect(not r["ok"] and r["woken_rt"] == 1, f"a wakeup of the emulator onto CPU 3: {r}")
    r = check(clean() + [migrate(t, 2, 3)])
    expect(not r["ok"] and r["moved_rt"] == 1, f"a migration of the emulator onto CPU 3: {r}")
    # Attempt 15's sequence: pulled onto an idle CPU 3, asleep there, woken there, run there once the worker sleeps.
    seq = [migrate(t, 2, 3), switch(t + 6_000, 3, ("swapper/3", 0), ("dxl_emu", EMU)), switch(t + 8_000, 3, ("dxl_emu", EMU), ("swapper/3", 0)),
           wakeup(t + 1 * MS, 3, by=WORKER), switch(t + 3 * MS, 3, WORKER, ("dxl_emu", EMU))]
    r = check(clean() + seq)
    expect(not r["ok"] and r["moved_rt"] == 1 and r["woken_rt"] == 1 and r["runs_rt"] == 2, f"attempt 15's sequence: {r}")
    # A thread the emulator creates, run on CPU 3.
    fork = line(T + 2 * MS, 2, "dxl_emu", EMU, "sched_process_fork", f"comm=dxl_emu pid={EMU} child_comm=dxl_emu child_pid={EMU + 1}")
    r = check(clean() + [fork, switch(t, 3, WORKER, ("dxl_emu", EMU + 1)), switch(t + 10_000, 3, ("dxl_emu", EMU + 1), WORKER)])
    expect(not r["ok"] and r["forks"] == 1 and r["runs_rt"] == 1, f"a thread it creates, run on CPU 3: {r}")
    # Outside the window, and another task on CPU 3: no count.
    r = check(clean() + [migrate(T - 500_000, 2, 3), wakeup(T + 10 * MS + 500_000, 3, by=WORKER),
                         switch(t, 3, WORKER, ("other", 77)), switch(t + 10_000, 3, ("other", 77), WORKER), wakeup(t, 3, 77, "other")])
    expect(r["ok"], f"the emulator on CPU 3 only outside the window, another task there inside it: {r}")
    r = check([ln for ln in clean() if "dxl_emu" not in ln])
    expect(not r["ok"] and r["seen"] == 0, f"the emulator never seen to run: {r}")
    unreadable(clean() + [line(T + 6 * MS, 3, "x", 1, "sched_switch", "MALFORMED")], "a malformed switch")
    unreadable(clean() + [line(T + 6 * MS, 3, "x", 1, "sched_wakeup", "MALFORMED")], "a malformed wakeup")
    unreadable(clean() + [line(T + 6 * MS, 3, "x", 1, "sched_migrate_task", "MALFORMED")], "a malformed migration")
    unreadable(clean() + [line(T + 6 * MS, 3, "x", 1, "sched_process_fork", "MALFORMED")], "a malformed fork")
    rows = sorted(clean(), key=stamp)
    rows.insert(len(rows) // 2, "  garbage that is no event")
    try:
        er.analyse(rows, Cap(), EMU_REC, 3, io.StringIO())
        expect(False, "a line that is no event: read")
    except er.Unreadable:
        pass
    unreadable(sorted(clean(), key=stamp)[:12], "a report that ends inside the window")


GOOD = "pid=4100 start=1234 comm=dxl_emu tids=4100 allowed=0-2,4-31"
RECORD = ["started=x", f"emulator_start=t0 {GOOD}", f"emulator_A=t1 {GOOD}", f"emulator_B=t2 {GOOD}"]


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


def cli() -> None:
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d, "run.csv")
        base.write_text("cycle,scheduled_wake_ns,wake_ns,cycle_end_ns\n1,0,0,1000\n2,1000000,1000000,10000000\n")
        pathlib.Path(str(base) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {T}\nworker_tid = 4242\n")
        rec = pathlib.Path(d, "experiment.record")
        rec.write_text("\n".join(RECORD) + "\n")
        rep = pathlib.Path(d, "report.txt")
        for lines, want in ((clean(), 0), (clean() + [wakeup(T + 5 * MS, 3, by=WORKER)], 1), (sorted(clean(), key=stamp)[:12], 2)):
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
