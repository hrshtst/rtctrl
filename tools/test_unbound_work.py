#!/usr/bin/env python3
"""Regressions for tools/unbound_work.py on synthetic report lines: the worker's flushes run off the real-time CPU and
bound work on it is only counted (PASS); a flush on it, other unbound work on it, a flush never seen to run, no flush
at all, each FAIL; unbound work there outside the window does not count; a report that ends before the window is
unreadable; the CLI's exit codes."""

from __future__ import annotations

import io
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


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def line(t: int, cpu: int, comm: str, pid: int, ev: str, rest: str) -> str:
    return f"  {comm}-{pid}  [{cpu:03d}] d.... {t // S}.{t % S:09d}: {ev}: {rest}"


def queue(t: int, work: str = WORK) -> str:
    return line(t, 3, "x7_bus_timing", TID, "workqueue_queue_work",
                f"work struct={work} function=flush_to_ldisc workqueue=events_unbound req_cpu=8192 cpu=-1")


def run(t: int, cpu: int, comm: str = "kworker/u128:3", fn: str = "flush_to_ldisc", work: str = WORK) -> str:
    return line(t, cpu, comm, 77, "workqueue_execute_start", f"work struct {work}: function {fn}")


def clean() -> list[str]:
    """Three instructions 1 ms apart, each flushed 4 us later on CPU 2; bound work on CPU 3; the trace runs past the
    window (10 ms)."""
    out = []
    for k in range(3):
        t = T + (k + 1) * MS
        out += [queue(t), run(t + 4_000, 2)]
    out += [run(T + 5 * MS, 3, "kworker/3:1", "vmstat_update", "0xffff0001"), line(T + 11 * MS, 0, "swapper/0", 0, "sched_switch", "x")]
    return out


def stamp(ln: str) -> int:
    s, ns = ln.split("] ")[1].split()[1].rstrip(":").split(".")
    return int(s) * S + int(ns)


class Cap:
    def __init__(self) -> None:
        self.window, self.tid = (T, T + 10 * MS), TID


def check(lines: list[str]) -> dict:
    return uw.analyse(sorted(lines, key=stamp), Cap(), 3, io.StringIO())


def units() -> None:
    r = check(clean())
    expect(r["ok"] and r["queued"] == 3 and r["ran"] == 3 and r["cpus"] == {2: 3} and r["delays"] == [4000, 4000, 4000]
           and r["rt_bound"] == 1, f"clean: {r}")
    r = check(clean() + [queue(T + 6 * MS), run(T + 6 * MS + 9_000, 3)])
    expect(not r["ok"] and r["on_rt"] == 1 and r["rt_flush"] == 1 and r["rt_unbound"] == 1, f"a flush on CPU 3: {r}")
    r = check(clean() + [run(T + 7 * MS, 3, fn="wb_update_bandwidth_workfn", work="0xffff0002")])
    expect(not r["ok"] and r["rt_unbound"] == 1 and r["rt_flush"] == 0, f"other unbound work on CPU 3: {r}")
    r = check(clean() + [run(T - MS, 3, fn="wb_update_bandwidth_workfn", work="0xffff0002"),
                         run(T + 10 * MS + 500_000, 3, fn="wb_update_bandwidth_workfn", work="0xffff0002")])
    expect(r["ok"] and r["rt_unbound"] == 0, f"unbound work on CPU 3 outside the window: {r}")
    r = check(clean() + [queue(T + 8 * MS, "0xffff0003")])
    expect(not r["ok"] and r["unmatched"] == 1, f"a flush never seen to run: {r}")
    out = io.StringIO()
    r = uw.analyse([run(T + MS, 2), line(T + 11 * MS, 0, "swapper/0", 0, "sched_switch", "x")], Cap(), 3, out)
    expect(not r["ok"] and r["queued"] == 0 and "nothing establishes" in out.getvalue(), f"no flush queued: {out.getvalue()}")
    try:
        check(clean()[:4])
        expect(False, "a report that ends inside the window was read")
    except uw.Unreadable:
        pass


def cli() -> None:
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d, "run.csv")
        base.write_text("cycle,scheduled_wake_ns,wake_ns,cycle_end_ns\n1,0,0,1000\n2,1000000,1000000,10000000\n")
        pathlib.Path(str(base) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {T}\nworker_tid = {TID}\n")
        rep = pathlib.Path(d, "report.txt")
        for lines, want in ((clean(), 0), (clean() + [queue(T + 6 * MS), run(T + 6 * MS + 9_000, 3)], 1), (clean()[:4], 2)):
            rep.write_text("\n".join(sorted(lines, key=stamp)) + "\n")
            p = subprocess.run([sys.executable, "tools/unbound_work.py", str(base), "--report", str(rep), "--rt-cpu", "3"],
                               capture_output=True, text=True)
            expect(p.returncode == want, f"the CLI, expecting {want}: {p.returncode} {p.stdout} {p.stderr}")


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
