#!/usr/bin/env python3
"""Regressions for tools/tk_cost.py: interrupts paired per CPU from report lines, each update placed in its innermost
enclosing interrupt, the with and without durations and their bound, the late sleeps an update overlaps, and the CLI
with a saved report and a baseline."""

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


T = 100 * S


def capture() -> tuple[list[str], list[dict]]:
    """CPU 3: ten timer interrupts of 2 us, two of them 5 us and holding an update; CPU 5: a timer interrupt holding an
    update, a device interrupt holding one, and an update in no interrupt; plus an exit with no entry, and an entry with no
    exit, at the trace's ends."""
    lines = [row(T - 10, 3, "local_timer_exit"), row(T, 0, "sched_switch", "prev_comm=x")]
    ups = []
    for i in range(10):
        a = T + i * 1_000_000
        long_ = i in (2, 7)
        lines += irq(a, a + (5000 if long_ else 2000), 3)
        if long_:
            ups.append({"entry": a + 1000, "ret": a + 2500, "cpu": 3})
    # A device interrupt whose handler runs another's inside it (VMD and its NVMe queue): paired by number, the inner
    # one holds the update.
    lines += irq(T + 500_000, T + 503_000, 5) + [row(T + 599_000, 5, "irq_handler_entry", "irq=136 name=vmd0")]
    lines += irq(T + 600_000, T + 604_000, 5, "irq_handler", ("irq=157 name=nvme1q0", "irq=157 ret=handled"))
    lines += [row(T + 605_000, 5, "irq_handler_exit", "irq=136 ret=handled")]
    ups += [{"entry": T + 501_000, "ret": T + 502_000, "cpu": 5}, {"entry": T + 601_000, "ret": T + 602_000, "cpu": 5},
            {"entry": T + 700_000, "ret": T + 701_000, "cpu": 5}]
    lines += [row(T + 9 * S, 5, "local_timer_entry")]
    return lines, ups


def units() -> None:
    lines, ups = capture()
    irqs, unpaired = tk_cost.interrupts(lines)
    expect(unpaired == 2 and len(irqs[3]) == 10 and len(irqs[5]) == 3, f"pairing: {unpaired} unpaired, {len(irqs[3])} and {len(irqs[5])}")
    starts = [x[0] for x in irqs[5]]
    expect(tk_cost.enclosing(irqs[5], starts, T + 601_000, T + 602_000) == (T + 600_000, T + 604_000, "device"), "the enclosing device interrupt")
    expect(tk_cost.enclosing(irqs[5], starts, T + 700_000, T + 701_000) is None, "an update in no recorded interrupt")
    expect(tk_cost.enclosing(irqs[5], starts, T + 502_500, T + 503_500) is None, "an update that outlasts its interrupt is not inside it")
    expect(tk_cost.quantile([1, 2, 3, 4], .5) == 2 and tk_cost.quantile([1, 2, 3, 4], 1.0) == 4 and tk_cost.quantile([], .5) is None, "the nearest-rank quantile")
    # Sleeps: one woken 60 us late with an update on CPU 3 inside it, one woken 60 us late without, and ordinary ones.
    sl = [(T + 2 * 1_000_000 + 900, T + 2 * 1_000_000 + 60_900), (T + 4 * 1_000_000, T + 4 * 1_000_000 + 60_000)]
    sl += [(T + k * 1_000_000 + 500, T + k * 1_000_000 + 1500) for k in range(5, 9)]  # the one at 7 overlaps that update, on time
    out = io.StringIO()
    r = tk_cost.analyse(ups, irqs, sl, 3, out)
    c3, c5 = r["cpus"][3], r["cpus"][5]
    expect(c3["updates"] == 2 and c3["with_"] == [5000, 5000] and len(c3["without"]) == 8 and c3["bound"] == 3000 and c3["outside"] == 0,
           f"CPU 3: {c3}")
    expect(c5["updates"] == 3 and c5["outside"] == 1 and c5["kinds"] == {"local timer": 1, "device": 1} and c5["bound"] is None, f"CPU 5: {c5}")
    expect(r["late"] == 2 and r["late_overlapped"] == 1 and r["sleeps"] == 6 and r["sleeps_overlapped"] == 2,
           f"the late sleeps an update overlaps: {r['late']} {r['late_overlapped']} {r['sleeps']} {r['sleeps_overlapped']}")
    text = out.getvalue()
    expect("CPU 3 (real-time): 2 updates" in text and "difference of the medians (an upper bound on the recorder's addition per update): 3.00 us" in text
           and "none recorded 1" in text and "2 sleeps woke 50 us or more late, 1 of them overlapped by an update on CPU 3" in text, text)


def cli() -> None:
    with tempfile.TemporaryDirectory() as d:
        lines, ups = capture()
        base = pathlib.Path(d, "run.csv")
        origin = T
        rows = ["cycle,scheduled_wake_ns,wake_ns"] + [f"{k},{k * 1_000_000},{k * 1_000_000 + (60_000 if k == 2 else 2000)}" for k in range(1, 9)]
        base.write_text("\n".join(rows) + "\n")
        pathlib.Path(str(base) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {origin}\n")
        pathlib.Path(str(base) + ".run.txt").write_text("command: x --rt-cpu 3 --rt-priority 80\n")
        rec = ["cpus=32"] + [f"rtctrl_tk:   <idle>-0  [{u['cpu']:03d}] d.h1. {stamp(u['entry'])}: update_entry: (x)\n"
                             f"rtctrl_tk:   <idle>-0  [{u['cpu']:03d}] d.h1. {stamp(u['ret'])}: update_return: (x <- y)" for u in ups]
        pathlib.Path(str(base) + ".tk.txt").write_text("\n".join(rec) + "\n")
        rep = pathlib.Path(d, "run.report.txt")
        rep.write_text("\n".join(lines) + "\n")
        bl = pathlib.Path(d, "base.csv")
        bl.write_text("\n".join(rows) + "\n")
        pathlib.Path(str(bl) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {origin}\n")
        brep = pathlib.Path(d, "base.report.txt")
        brep.write_text("\n".join(sum((irq(T + i * 1_000_000, T + i * 1_000_000 + 1500, 3) for i in range(10)), [])) + "\n")
        p = subprocess.run([sys.executable, "tools/tk_cost.py", str(base), "--report", str(rep), "--baseline", str(bl), "--baseline-report", str(brep)],
                           capture_output=True, text=True)
        expect(p.returncode == 0 and "judges nothing" in p.stdout and "CPU 3 (real-time): 2 updates" in p.stdout
               and "baseline:     n 10, p50 1.50" in p.stdout and "this capture: n 10, p50 2.00" in p.stdout, f"the CLI: {p.returncode} {p.stdout} {p.stderr}")
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
