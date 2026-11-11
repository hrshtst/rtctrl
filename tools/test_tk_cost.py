#!/usr/bin/env python3
"""Regressions for tools/tk_cost.py: interrupts paired per CPU (device ones by number, nested), the worker's waits,
everything restricted to the recorded window with what lies outside counted apart, the median difference reported as a
descriptive figure that may be negative (the validation review's counterexample), cycle wakes only for the cycles that
slept, and the CLI with a saved report and a baseline cropped to its own window."""

from __future__ import annotations

import io
import pathlib
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tk_cost  # noqa: E402

FAILURES: list[str] = []
S = 1_000_000_000
T = 100 * S       # the probe's epoch
TID = 4242
MS = 1_000_000


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def stamp(t: int) -> str:
    return f"{t // S}.{t % S:09d}"


def row(t: int, cpu: int, event: str, rest: str = "vector=236", comm: str = "<idle>-0") -> str:
    return f"          {comm}     [{cpu:03d}] d.h1. {stamp(t)}: {event}:    {rest}"


def irq(t0: int, t1: int, cpu: int, kind: str = "local_timer", rest: tuple = ("vector=236", "vector=236")) -> list[str]:
    return [row(t0, cpu, f"{kind}_entry", rest[0]), row(t1, cpu, f"{kind}_exit", rest[1])]


def waking(t: int) -> str:
    return row(t, 5, "sched_waking", f"comm=x7_bus_timing pid={TID} prio=19 target_cpu=003", "kworker/u128:3-77")


def switch(t: int, prev: tuple, nxt: tuple) -> str:
    return row(t, 3, "sched_switch", f"{prev[0]}:{prev[1]} [19] {prev[2]} ==> {nxt[0]}:{nxt[1]} [19]")


def by_time(ln: str) -> int:
    s, n = ln.split("]")[1].split()[1].rstrip(":").split(".")
    return int(s) * S + int(n)


def capture() -> tuple[list[str], list[dict], list[dict]]:
    """Rows: ten 1 ms cycles from T, the third waking 60 us late, the sixth beginning without sleeping (its scheduled wake
    already past at the fifth's end). CPU 3: ten timer interrupts of 2 us inside the window, two of 5 us holding an
    update, one before the window and one after it (each holding an update), and an update across the window's end.
    CPU 5: a timer interrupt and, inside a VMD interrupt, an NVMe queue's interrupt each holding an update, and an update
    in no interrupt; plus an exit with no entry and an entry with no exit. The worker: a 70 us wait overlapped by an
    update on CPU 3, a 2 us one, and a waking while it was still running (no wait)."""
    rows = []
    for k in range(10):
        sched = k * MS
        wake = sched + (60_000 if k == 2 else 2_000)
        end = wake + (1_200_000 if k == 4 else 500_000)  # the fifth runs past the sixth's scheduled wake
        if k == 5:
            wake = rows[-1]["cycle_end_ns"] + 1_000
            end = wake + 300_000
        rows.append({"cycle": k + 1, "scheduled_wake_ns": sched, "wake_ns": wake, "cycle_end_ns": end})
    lines = [row(T - 10, 3, "local_timer_exit"), row(T + 1, 0, "sched_switch", "prev_comm=x")]
    ups = []
    for i in range(10):
        a = T + i * MS + 100_000
        long_ = i in (2, 7)
        lines += irq(a, a + (5000 if long_ else 2000), 3)
        if long_:
            ups.append({"entry": a + 1000, "ret": a + 2500, "cpu": 3})
    lines += irq(T - 5 * MS, T - 5 * MS + 5000, 3) + irq(T + 20 * MS, T + 20 * MS + 5000, 3)
    end = T + rows[-1]["cycle_end_ns"]
    ups += [{"entry": T - 5 * MS + 1000, "ret": T - 5 * MS + 2000, "cpu": 3}, {"entry": T + 20 * MS + 1000, "ret": T + 20 * MS + 2000, "cpu": 3},
            {"entry": end - 1000, "ret": end + 1000, "cpu": 3}]  # across the window's end
    lines += irq(T + 500_000, T + 503_000, 5) + [row(T + 599_000, 5, "irq_handler_entry", "irq=136 name=vmd0")]
    lines += irq(T + 600_000, T + 604_000, 5, "irq_handler", ("irq=157 name=nvme1q0", "irq=157 ret=handled"))
    lines += [row(T + 605_000, 5, "irq_handler_exit", "irq=136 ret=handled")]
    ups += [{"entry": T + 501_000, "ret": T + 502_000, "cpu": 5}, {"entry": T + 601_000, "ret": T + 602_000, "cpu": 5},
            {"entry": T + 700_000, "ret": T + 701_000, "cpu": 5}]
    worker, idle = ("x7_bus_timing", TID, "D"), ("swapper/3", 0, "R")
    lines += [switch(T + 2 * MS + 90_000, worker, idle), waking(T + 2 * MS + 99_000), switch(T + 2 * MS + 169_000, idle, ("x7_bus_timing", TID, "R")),
              switch(T + 4 * MS + 800_000, worker, idle), waking(T + 4 * MS + 801_000), switch(T + 4 * MS + 803_000, idle, ("x7_bus_timing", TID, "R")),
              waking(T + 6 * MS + 10_000)]  # still running: no wait
    ups.append({"entry": T + 2 * MS + 120_000, "ret": T + 2 * MS + 121_000, "cpu": 3})  # inside the 70 us wait, in no interrupt
    lines += [row(T + 9 * S, 5, "local_timer_entry")]
    return sorted(lines, key=by_time), ups, rows


class Cap(tk_cost.Capture):
    def __init__(self, rows: list[dict]) -> None:  # the probe's rows and summary, in memory
        self.origin, self.tid, self.cycles = T, TID, len(rows)
        self.window = (T + rows[0]["wake_ns"], T + rows[-1]["cycle_end_ns"])
        self.wakes = [(T + b["scheduled_wake_ns"], T + b["wake_ns"]) for a, b in zip(rows, rows[1:]) if b["scheduled_wake_ns"] > a["cycle_end_ns"]]


def units() -> None:
    lines, ups, rows = capture()
    s = tk_cost.scan(lines, TID, 3)
    irqs, unpaired = s["irqs"], s["unpaired"]
    expect(unpaired == 2 and len(irqs[3]) == 12 and len(irqs[5]) == 3, f"pairing: {unpaired} unpaired, {len(irqs[3])} and {len(irqs[5])}")
    expect(s["waits"] == [(T + 2 * MS + 99_000, T + 2 * MS + 169_000), (T + 4 * MS + 801_000, T + 4 * MS + 803_000)], f"the worker's waits: {s['waits']}")
    starts = [x[0] for x in irqs[5]]
    expect(tk_cost.enclosing(irqs[5], starts, T + 601_000, T + 602_000) == (T + 600_000, T + 604_000, "device"), "the innermost device interrupt")
    expect(tk_cost.enclosing(irqs[5], starts, T + 700_000, T + 701_000) is None, "an update in no recorded interrupt")
    expect(tk_cost.quantile([1, 2, 3, 4], .5) == 2 and tk_cost.quantile([], .5) is None, "the nearest-rank quantile")
    out = io.StringIO()
    r = tk_cost.analyse(ups, s, Cap(rows), 3, out)
    c3, c5 = r["cpus"][3], r["cpus"][5]
    expect(c3["updates"] == 3 and c3["none_recorded"] == 1 and c3["with_"] == [5000, 5000] and len(c3["without"]) == 8
           and c3["median_difference"] == 3000, f"CPU 3 inside the window only: {c3}")
    expect(r["outside"][3] == [1, 1, 1], f"the updates before, after and across the window's end: {r['outside']}")
    expect(c5["updates"] == 3 and c5["none_recorded"] == 1 and c5["kinds"] == {"local timer": 1, "device": 1}, f"CPU 5: {c5}")
    expect(r["wakes"] == 8 and r["late"] == 1 and r["late_overlapped"] == 0, f"cycle wakes, the cycles that slept: {r['wakes']} {r['late']}")
    expect(r["waits"] == 2 and r["long_waits"] == 1 and r["long_overlapped"] == 1, f"the worker's waits: {r['waits']} {r['long_waits']} {r['long_overlapped']}")
    text = out.getvalue()
    expect("analysis window:" in text and "CPU 3 1/1/1" in text and "8 of 10 cycles slept" in text
           and "difference of the medians, holding an update minus holding none: 3.00 us (descriptive" in text
           and "upper bound" not in text and "1 of 50 us or more, 1 of them overlapped by an update on CPU 3" in text
           and "not proof that the instrumentation had no effect" in text, text)
    # The review's counterexample: an update interrupt of 2 us beside an interrupt of 10 us without one. The difference
    # is -8 us, reported as it is and labelled descriptive, never as a bound and never clamped.
    cap = Cap([{"scheduled_wake_ns": 0, "wake_ns": 0, "cycle_end_ns": 30_000}])
    out = io.StringIO()
    r = tk_cost.analyse([{"cpu": 3, "entry": T + 1100, "ret": T + 1400}],
                        {"irqs": {3: [(T + 1000, T + 3000, "local timer"), (T + 10000, T + 20000, "local timer")]}, "unpaired": 0, "waits": []}, cap, 3, out)
    expect(r["cpus"][3]["median_difference"] == -8000 and "-8.00 us (descriptive" in out.getvalue(), f"a negative difference: {out.getvalue()}")
    # No update on the real-time CPU inside the window: zero, said so, and no difference.
    out = io.StringIO()
    r = tk_cost.analyse([{"cpu": 3, "entry": T - 5000, "ret": T - 4000}], {"irqs": {}, "unpaired": 0, "waits": []}, cap, 3, out)
    expect(r["cpus"][3]["updates"] == 0 and r["outside"][3] == [1, 0, 0] and "CPU 3 (real-time): 0 updates" in out.getvalue()
           and "difference of the medians" not in out.getvalue(), f"no update inside the window: {out.getvalue()}")


def cli() -> None:
    with tempfile.TemporaryDirectory() as d:
        lines, ups, rows = capture()
        base = pathlib.Path(d, "run.csv")
        header = "cycle,scheduled_wake_ns,wake_ns,cycle_end_ns"
        base.write_text("\n".join([header] + [f"{r['cycle']},{r['scheduled_wake_ns']},{r['wake_ns']},{r['cycle_end_ns']}" for r in rows]) + "\n")
        pathlib.Path(str(base) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {T}\nworker_tid = {TID}\n")
        pathlib.Path(str(base) + ".run.txt").write_text("command: x --rt-cpu 3 --rt-priority 80\n")
        rec = ["cpus=32"] + [f"rtctrl_tk:   <idle>-0  [{u['cpu']:03d}] d.h1. {stamp(u['entry'])}: update_entry: (x)\n"
                             f"rtctrl_tk:   <idle>-0  [{u['cpu']:03d}] d.h1. {stamp(u['ret'])}: update_return: (x <- y)"
                             for u in sorted(ups, key=lambda u: u["entry"])]
        pathlib.Path(str(base) + ".tk.txt").write_text("\n".join(rec) + "\n")
        rep = pathlib.Path(d, "run.report.txt")
        rep.write_text("\n".join(lines) + "\n")
        bl = pathlib.Path(d, "base.csv")
        bl.write_text(base.read_text())
        pathlib.Path(str(bl) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {T}\nworker_tid = {TID}\n")
        brep = pathlib.Path(d, "base.report.txt")
        inside = sum((irq(T + i * MS + 100_000, T + i * MS + 101_500, 3) for i in range(10)), [])
        brep.write_text("\n".join(inside + irq(T + 30 * MS, T + 30 * MS + 90_000, 3)) + "\n")  # one outside its window
        p = subprocess.run([sys.executable, "tools/tk_cost.py", str(base), "--report", str(rep), "--baseline", str(bl), "--baseline-report", str(brep)],
                           capture_output=True, text=True)
        expect(p.returncode == 0 and "judges and bounds nothing" in p.stdout and "CPU 3 (real-time): 3 updates" in p.stdout
               and "baseline:     n 10, p50 1.50" in p.stdout and "this capture: n 10, p50 2.00" in p.stdout and "not a controlled experiment" in p.stdout,
               f"the CLI: {p.returncode} {p.stdout} {p.stderr}")
        pathlib.Path(str(base) + ".tk.txt").write_text("garbage\n")
        p = subprocess.run([sys.executable, "tools/tk_cost.py", str(base), "--report", str(rep)], capture_output=True, text=True)
        expect(p.returncode == 2 and "no readable timekeeping record" in p.stderr, f"an unreadable record: {p.returncode} {p.stderr}")
        p = subprocess.run([sys.executable, "tools/tk_cost.py", str(base), "--baseline-report", str(brep)], capture_output=True, text=True)
        expect(p.returncode == 2, "--baseline-report alone")


def main() -> int:
    units()
    cli()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print("timekeeping cost regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
