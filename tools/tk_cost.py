#!/usr/bin/env python3
"""The timekeeping recorder's cost on the CPUs that perform the updates: a read-only analysis of one timer capture
(docs/hardware/control-rate.md, the clock alignment's validation). It changes nothing and judges nothing.

    tk_cost.py <run>.csv [--report FILE] [--baseline <run>.csv [--baseline-report FILE]] [--rt-cpu N]

The recorder (tools/tk_recorder.sh) puts a probe on the entry and the return of update_wall_time, which run on whichever
CPU performs the timekeeping update. Its records (<run>.csv.tk.txt) give each update's CPU and its entry and return
stamps; the main trace (<run>.csv.trace.dat, through trace-cmd report -t under tools/tk_record.py's launch, or a saved
--report) gives every CPU's interrupts on the same clock. For each CPU that performed updates:

- the updates, their rate and durations (return stamp minus entry stamp, which already holds part of the probes' work);
- the interrupt that encloses each update (the innermost of the recorded local timer, device, reschedule and
  function-call interrupts), and the updates no recorded interrupt encloses;
- the local timer interrupts that hold an update against those that hold none: their durations' quantiles, and the
  difference of the medians. That difference is an upper bound on what the recorder adds to an update's interrupt,
  since it also holds the update's own work, which runs without the recorder too.

On the real-time CPU, each sleep whose wake came 50 us or more after its scheduled time (the probe's log) is checked for
an update on that CPU overlapping it, beside the share of all sleeps an update overlaps. With --baseline, a capture
without the recorder at the same condition, the real-time CPU's local timer interrupt durations and the wake lateness are
set side by side; the difference between two sessions also holds anything else that differed between them.
"""

from __future__ import annotations

import argparse
import bisect
import collections
import csv
import math
import pathlib
import re
import subprocess
import sys

try:
    import tomllib
except ImportError:  # Python < 3.11
    tomllib = None  # type: ignore[assignment]

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tk_record  # noqa: E402

# The interrupts whose entry and exit the main trace records on every CPU, by the kind their events name.
IRQ_EVENTS = {"local_timer_entry": ("local timer", 1), "local_timer_exit": ("local timer", -1),
              "irq_handler_entry": ("device", 1), "irq_handler_exit": ("device", -1),
              "reschedule_entry": ("reschedule", 1), "reschedule_exit": ("reschedule", -1),
              "call_function_entry": ("function call", 1), "call_function_exit": ("function call", -1),
              "call_function_single_entry": ("function call", 1), "call_function_single_exit": ("function call", -1)}
EVENT = re.compile(r"^\s*(?:[A-Za-z_][A-Za-z0-9_]*:\s+)?.*?-\d+\s+\[(\d+)\]\s+(?:\S+\s+)?(\d+)\.(\d{9}):\s+(\w+):\s*(.*)$")
IRQ_NUMBER = re.compile(r"\birq=(\d+)")
LATE_NS = 50_000  # the timer service classification's threshold for a late wake


def quantile(xs: list, p: float):
    """The nearest-rank p-quantile of sorted xs, or None for none."""
    if not xs:
        return None
    return xs[max(0, min(len(xs) - 1, math.ceil(p * len(xs)) - 1))]


def interrupts(lines) -> tuple[dict, int]:
    """Every CPU's interrupts as (entry, exit, kind), sorted by entry, from report lines; and the count of entries or
    exits that found no partner (at the trace's ends, or interleaved). Device interrupts pair by their number: one
    handler can run another's inside it (Intel VMD's demultiplexes its NVMe queues' interrupts)."""
    open_: dict = {}
    out: dict = collections.defaultdict(list)
    unpaired = 0
    for ln in lines:
        m = EVENT.match(ln)
        if not m or m[4] not in IRQ_EVENTS:
            continue
        cpu, t = int(m[1]), int(m[2]) * 1_000_000_000 + int(m[3])
        kind, way = IRQ_EVENTS[m[4]]
        n = IRQ_NUMBER.search(m[5]) if kind == "device" else None
        key = (cpu, kind, n[1] if n else None)
        if way == 1:
            unpaired += key in open_
            open_[key] = t
        elif key in open_:
            out[cpu].append((open_.pop(key), t, kind))
        else:
            unpaired += 1
    unpaired += len(open_)
    for v in out.values():
        v.sort()
    return out, unpaired


def enclosing(irqs: list, starts: list, entry: int, ret: int):
    """The innermost recorded interrupt on the CPU that holds [entry, ret], or None."""
    i = bisect.bisect_right(starts, entry) - 1
    for j in range(i, max(-1, i - 16), -1):
        a, b, _ = irqs[j]
        if a <= entry and ret <= b:
            return irqs[j]
    return None


def sleeps(csv_path: pathlib.Path) -> list:
    """The probe's sleeps as (scheduled wake, wake) on CLOCK_MONOTONIC, from its log and summary."""
    if tomllib is None:
        raise SystemExit("tk_cost: needs Python 3.11 or later (tomllib)")
    origin = int(tomllib.loads(pathlib.Path(str(csv_path) + ".summary.toml").read_text())["result"]["monotonic_origin_ns"])
    with open(csv_path, newline="") as f:
        return [(origin + int(r["scheduled_wake_ns"]), origin + int(r["wake_ns"])) for r in csv.DictReader(f) if int(r["wake_ns"]) >= 0]


def rt_cpu_of(csv_path: pathlib.Path, default: int = 3) -> int:
    try:
        m = re.search(r"--rt-cpu (\d+)", pathlib.Path(str(csv_path) + ".run.txt").read_text(errors="replace"))
    except OSError:
        return default
    return int(m[1]) if m else default


def report_lines(csv_path: pathlib.Path, saved: pathlib.Path | None):
    """The main trace's report lines: a saved report, or trace-cmd report -t under the launch."""
    if saved:
        with open(saved, errors="replace") as f:
            yield from f
        return
    tool = tk_record.tool_identity("trace-cmd")
    proc = subprocess.Popen([tool.get("path") or "trace-cmd", "report", "-t", "-i", f"{csv_path}.trace.dat"], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, errors="replace", env=tk_record.launch_env())
    assert proc.stdout is not None
    yield from proc.stdout
    if proc.wait() != 0:
        raise SystemExit(f"tk_cost: trace-cmd report -i {csv_path}.trace.dat failed")


def us(ns) -> str:
    return "-" if ns is None else f"{ns / 1000:.2f}"


def dist(xs: list) -> str:
    xs = sorted(xs)
    return f"n {len(xs)}, p50 {us(quantile(xs, .5))} p99 {us(quantile(xs, .99))} p99.9 {us(quantile(xs, .999))} max {us(xs[-1] if xs else None)} us"


def analyse(updates: list, irqs: dict, sleeps_: list, rt: int, out) -> dict:
    """The per-CPU cost summary of a capture's updates (tk_record.parse_record's) against its interrupts."""
    per: dict = collections.defaultdict(list)
    for u in updates:
        per[u["cpu"]].append(u)
    span = (max(u["ret"] for u in updates) - min(u["entry"] for u in updates)) / 1e9 if updates else 0.0
    result: dict = {"cpus": {}}
    print(f"timekeeping updates: {len(updates)} over {span:.1f} s on {len(per)} CPUs (the real-time CPU is {rt})", file=out)
    for cpu in sorted(per, key=lambda c: (c != rt, -len(per[c]))):
        ups = per[cpu]
        irq = irqs.get(cpu, [])
        starts = [x[0] for x in irq]
        held: dict = {}
        outside = 0
        kinds: collections.Counter = collections.Counter()
        for u in ups:
            e = enclosing(irq, starts, u["entry"], u["ret"])
            if e is None:
                outside += 1
            else:
                held[e] = True
                kinds[e[2]] += 1
        timers = [x for x in irq if x[2] == "local timer"]
        with_ = sorted(b - a for (a, b, k) in timers if (a, b, k) in held)
        without = sorted(b - a for (a, b, k) in timers if (a, b, k) not in held)
        mw, mo = quantile(with_, .5), quantile(without, .5)
        bound = mw - mo if mw is not None and mo is not None else None
        dur = sorted(u["ret"] - u["entry"] for u in ups)
        result["cpus"][cpu] = dict(updates=len(ups), outside=outside, kinds=dict(kinds), with_=with_, without=without, bound=bound, durations=dur)
        print(f"  CPU {cpu}{' (real-time)' if cpu == rt else ''}: {len(ups)} updates ({len(ups) / span if span else 0:.0f}/s); durations {dist(dur)}", file=out)
        print("    enclosing interrupt: " + ", ".join(f"{k} {n}" for k, n in kinds.most_common()) + f", none recorded {outside}", file=out)
        print(f"    local timer interrupts holding an update: {dist(with_)}", file=out)
        print(f"    local timer interrupts holding none:      {dist(without)}", file=out)
        print(f"    difference of the medians (an upper bound on the recorder's addition per update): {us(bound)} us", file=out)
    rt_ups = sorted((u["entry"], u["ret"]) for u in per.get(rt, []))
    starts = [a for a, _ in rt_ups]

    def overlapped(lo: int, hi: int) -> bool:
        i = bisect.bisect_right(starts, hi) - 1
        return any(rt_ups[j][1] >= lo for j in range(i, max(-1, i - 4), -1))

    late = [(s, w) for s, w in sleeps_ if w - s >= LATE_NS]
    late_hit = sum(overlapped(s, w) for s, w in late)
    all_hit = sum(overlapped(s, w) for s, w in sleeps_)
    lateness = sorted(w - s for s, w in sleeps_)
    result.update(late=len(late), late_overlapped=late_hit, sleeps=len(sleeps_), sleeps_overlapped=all_hit, lateness=lateness)
    print(f"  the worker's wakes on CPU {rt}: lateness {dist(lateness)}", file=out)
    print(f"    {len(late)} sleeps woke {LATE_NS // 1000} us or more late, {late_hit} of them overlapped by an update on CPU {rt}; "
          f"of all {len(sleeps_)} sleeps, {all_hit} overlapped by one", file=out)
    return result


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("csv", type=pathlib.Path, help="the timer capture's probe log (<run>.csv)")
    p.add_argument("--report", type=pathlib.Path, help="a saved trace-cmd report -t of the capture (default: run trace-cmd)")
    p.add_argument("--baseline", type=pathlib.Path, help="a capture without the recorder at the same condition (<run>.csv)")
    p.add_argument("--baseline-report", type=pathlib.Path, help="a saved report of the baseline")
    p.add_argument("--rt-cpu", type=int, help="the real-time CPU (default: the run record's, else 3)")
    a = p.parse_args()
    if a.baseline_report and not a.baseline:
        p.error("--baseline-report needs --baseline")
    rt = a.rt_cpu if a.rt_cpu is not None else rt_cpu_of(a.csv)
    rec = tk_record.parse_record(pathlib.Path(str(a.csv) + ".tk.txt").read_text(errors="replace")) if pathlib.Path(str(a.csv) + ".tk.txt").exists() else None
    if rec is None or rec["read_status"] != "ok":
        print(f"tk_cost: {a.csv}: no readable timekeeping record ({rec['read_status'] if rec else 'none'})", file=sys.stderr)
        return 2
    irqs, unpaired = interrupts(report_lines(a.csv, a.report))
    print(f"{a.csv.name}: tools/tk_cost.py (diagnostic, judges nothing); {unpaired} interrupt entries or exits unpaired", file=sys.stdout)
    analyse(rec["updates"], irqs, sleeps(a.csv), rt, sys.stdout)
    if a.baseline:
        b_irqs, b_unpaired = interrupts(report_lines(a.baseline, a.baseline_report))
        here = sorted(b - a_ for a_, b, k in irqs.get(rt, []) if k == "local timer")
        there = sorted(b - a_ for a_, b, k in b_irqs.get(rt, []) if k == "local timer")
        mine, theirs = sleeps(a.csv), sleeps(a.baseline)
        print(f"against {a.baseline.name} (no recorder; {b_unpaired} unpaired): CPU {rt}'s local timer interrupts", file=sys.stdout)
        print(f"  this capture: {dist(here)}", file=sys.stdout)
        print(f"  baseline:     {dist(there)}", file=sys.stdout)
        print(f"  the worker's wake lateness, this capture: {dist(sorted(w - s for s, w in mine))}", file=sys.stdout)
        print(f"  the worker's wake lateness, baseline:     {dist(sorted(w - s for s, w in theirs))}", file=sys.stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
