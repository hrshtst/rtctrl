#!/usr/bin/env python3
"""Observations of the timekeeping recorder's surroundings on the CPUs that perform the updates: a read-only, descriptive
analysis of one timer capture (docs/hardware/control-rate.md, the clock alignment's validation). It changes nothing,
judges nothing and bounds nothing.

    tk_cost.py <run>.csv [--report FILE] [--baseline <run>.csv [--baseline-report FILE]] [--rt-cpu N]

The recorder (tools/tk_recorder.sh) puts a probe on the entry and the return of update_wall_time, which run on whichever
CPU performs the timekeeping update. Its records (<run>.csv.tk.txt) give each update's CPU and its entry and return
stamps; the main trace (<run>.csv.trace.dat, through trace-cmd report -t under tools/tk_record.py's launch, or a saved
--report) gives every CPU's interrupts and the worker's wakings and switches on the same clock.

Everything is restricted to one recorded analysis window: the probe's first wake stamp to its last cycle-end stamp (the
condition's measured run). An update, interrupt or wait counts only if it lies entirely inside the window; updates
before, after or across its ends are counted apart, and rates use the window's elapsed time. The recorder's longer
dependency interval, which its certificate uses, is not trimmed by this; only this analysis is.

For each CPU that performed updates inside the window:

- the updates, their rate and their durations (the return stamp minus the entry stamp: the probes' own work is partly
  inside that interval and partly outside it, so it is not a complete measure of the instrumentation either);
- the innermost recorded interrupt enclosing each update (local timer, device, reschedule, function call), and the
  updates no recorded interrupt encloses;
- the local timer interrupts holding an update, and those holding none: the two distributions, and the difference of
  their medians. That difference is descriptive. The two groups are different interrupts doing different work, so it
  can understate or overstate what the recorder adds, and it can be negative; it is neither a bound on the recorder's
  cost nor an estimate of it.

On the real-time CPU, two separate populations: the probe's cycle wakes, and the worker's waits in the trace (a
sched_waking of the worker while it was off the CPU, to its next switch-in). The cycle wakes are the sleeps the trace
establishes, never inferred from the probe's deadlines: tools/trace_correlate.py's pairing (periodic_sleeps), one timer
wake of the worker inside the transition from a cycle's end stamp to the next cycle's wake stamp. A scheduled wake still
ahead at the end stamp does not show a sleep, since the probe's bookkeeping after that stamp can use up the rest of the
period; the transitions that began awake, lack a wake or hold several are counted, not taken as sleeps. Each sleep's
lateness is its wake stamp minus its scheduled wake. Wakes and waits of 50 us or longer are checked for an update on
that CPU overlapping them. An overlap count of zero describes the recorded update bodies only; it is not proof that the
instrumentation had no effect.

With --baseline, a capture without the recorder at nominally the same condition: the real-time CPU's local timer
interrupts, the established sleeps' lateness and the worker's waits, each cropped to its own capture's window, side by
side. It is a comparison between sessions, not a controlled recorder-on and recorder-off experiment: any difference also
holds whatever else differed between them, and it bounds nothing.
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
import types

try:
    import tomllib
except ImportError:  # Python < 3.11
    tomllib = None  # type: ignore[assignment]

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tk_record  # noqa: E402
import trace_correlate  # noqa: E402

# The events trace_correlate's sleep pairing reads: the worker's timer wakes and its switches.
SLEEP_EVENTS = {"hrtimer_expire_entry", "hrtimer_expire_exit", "sched_waking", "sched_switch"}

# The interrupts whose entry and exit the main trace records on every CPU, by the kind their events name.
IRQ_EVENTS = {"local_timer_entry": ("local timer", 1), "local_timer_exit": ("local timer", -1),
              "irq_handler_entry": ("device", 1), "irq_handler_exit": ("device", -1),
              "reschedule_entry": ("reschedule", 1), "reschedule_exit": ("reschedule", -1),
              "call_function_entry": ("function call", 1), "call_function_exit": ("function call", -1),
              "call_function_single_entry": ("function call", 1), "call_function_single_exit": ("function call", -1)}
EVENT = re.compile(r"^\s*(?:[A-Za-z_][A-Za-z0-9_]*:\s+)?.*?-\d+\s+\[(\d+)\]\s+(?:\S+\s+)?(\d+)\.(\d{9}):\s+(\w+):\s*(.*)$")
IRQ_NUMBER = re.compile(r"\birq=(\d+)")
WAKING_PID = re.compile(r"\bpid=(\d+)")
SWITCH = re.compile(r":(\d+) \[-?\d+\] \S+ ==> .*:(\d+) \[-?\d+\]\s*$")
LATE_NS = 50_000  # the timer service classification's threshold for a late wake


def quantile(xs: list, p: float):
    """The nearest-rank p-quantile of sorted xs, or None for none."""
    if not xs:
        return None
    return xs[max(0, min(len(xs) - 1, math.ceil(p * len(xs)) - 1))]


def scan(lines, tid: int | None = None, rt: int = 3) -> dict:
    """From report lines: every CPU's interrupts as (entry, exit, kind), sorted by entry, and the count of entries or
    exits with no partner; and, given the worker's tid, its waits on the real-time CPU as (waking, switch-in) and the
    trace_correlate.Trace its sleeps are paired from (else None). Device interrupts pair by their number: one handler can
    run another's inside it (Intel VMD's runs its NVMe queues')."""
    open_: dict = {}
    irqs: dict = collections.defaultdict(list)
    unpaired = 0
    waits: list = []
    off, waking = True, None
    trace = trace_correlate.Trace(tid, "") if tid is not None else None
    for ln in lines:
        m = EVENT.match(ln)
        if not m:
            continue
        cpu, t, ev, rest = int(m[1]), int(m[2]) * 1_000_000_000 + int(m[3]), m[4], m[5]
        if trace is not None and ev in SLEEP_EVENTS:
            trace.feed(ln.rstrip("\n"))
        if ev in IRQ_EVENTS:
            kind, way = IRQ_EVENTS[ev]
            n = IRQ_NUMBER.search(rest) if kind == "device" else None
            key = (cpu, kind, n[1] if n else None)
            if way == 1:
                unpaired += key in open_
                open_[key] = t
            elif key in open_:
                irqs[cpu].append((open_.pop(key), t, kind))
            else:
                unpaired += 1
        elif tid is not None and ev == "sched_waking":
            p = WAKING_PID.search(rest)
            if p and int(p[1]) == tid and off and waking is None:
                waking = t
        elif tid is not None and ev == "sched_switch" and cpu == rt:
            s = SWITCH.search(rest)
            if s and int(s[1]) == tid:
                off = True
            if s and int(s[2]) == tid:
                if waking is not None:
                    waits.append((waking, t))
                off, waking = False, None
    unpaired += len(open_)
    for v in irqs.values():
        v.sort()
    return {"irqs": irqs, "unpaired": unpaired, "waits": waits, "trace": trace}


def interrupts(lines) -> tuple[dict, int]:
    r = scan(lines)
    return r["irqs"], r["unpaired"]


def enclosing(irqs: list, starts: list, entry: int, ret: int):
    """The innermost recorded interrupt on the CPU that holds [entry, ret], or None."""
    i = bisect.bisect_right(starts, entry) - 1
    for j in range(i, max(-1, i - 16), -1):
        a, b, _ = irqs[j]
        if a <= entry and ret <= b:
            return irqs[j]
    return None


class Capture:
    """A capture's probe log and summary: its analysis window, the worker and the rows (the transitions between them are
    the candidates for its sleeps; the trace establishes which slept)."""

    def __init__(self, csv_path: pathlib.Path) -> None:
        if tomllib is None:
            raise SystemExit("tk_cost: needs Python 3.11 or later (tomllib)")
        self.summary = tomllib.loads(pathlib.Path(str(csv_path) + ".summary.toml").read_text())
        res = self.summary["result"]
        self.origin = int(res["monotonic_origin_ns"])
        self.tid = int(res["worker_tid"]) if "worker_tid" in res else None
        with open(csv_path, newline="") as f:
            self.rows = [r for r in csv.DictReader(f)]
        if not self.rows:
            raise SystemExit(f"tk_cost: {csv_path}: no cycles")
        o = self.origin
        self.window = (o + int(self.rows[0]["wake_ns"]), o + int(self.rows[-1]["cycle_end_ns"]))
        self.cycles = len(self.rows)

    def inside(self, a: int, b: int) -> bool:
        return self.window[0] <= a and b <= self.window[1]


def sleeps(cap: Capture, trace) -> dict | None:
    """The capture's sleeps as its trace establishes them (trace_correlate.periodic_sleeps): the slept cycles' (scheduled
    wake, wake stamp) inside the window, and the counts of the transitions that did not establish a sleep. None without
    the worker's trace."""
    if trace is None:
        return None
    # Only the transitions between rows: a late wake that ended the run lies after the window.
    res = {k: v for k, v in cap.summary["result"].items() if k != "late_wake_stop"}
    p = trace_correlate.periodic_sleeps(types.SimpleNamespace(origin=cap.origin, rows=cap.rows, summary={"result": res}), trace)
    slept = [(sched, wake) for _, (i, _, sched, wake) in p["pairs"] if i is not None and cap.inside(sched, wake)]
    return dict(slept=slept, transitions=cap.cycles - 1, began_awake=p["began_awake"], missing=p["missing"], duplicate=p["duplicate"])


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


def analyse(updates: list, scanned: dict, cap: Capture, rt: int, out) -> dict:
    """The per-CPU observations of a capture's updates (tk_record.parse_record's) and the real-time CPU's wakes and
    waits, inside the capture's window."""
    lo, hi = cap.window
    span = (hi - lo) / 1e9
    irqs = {c: [x for x in v if cap.inside(x[0], x[1])] for c, v in scanned["irqs"].items()}
    per: dict = collections.defaultdict(list)
    outside: dict = collections.defaultdict(lambda: [0, 0, 0])  # before, after, across an end
    for u in updates:
        if cap.inside(u["entry"], u["ret"]):
            per[u["cpu"]].append(u)
        else:
            outside[u["cpu"]][0 if u["ret"] < lo else 1 if u["entry"] > hi else 2] += 1
    result: dict = {"window": cap.window, "span": span, "cpus": {}, "outside": dict(outside)}
    print(f"analysis window: {lo} to {hi} ns ({span:.6f} s, the probe's first wake to its last cycle end); an update, "
          "interrupt or wait counts only if it lies entirely inside", file=out)
    print(f"timekeeping updates inside the window: {sum(len(v) for v in per.values())} on {len(per)} CPUs; outside it "
          "(before, after, across an end): " + (", ".join(f"CPU {c} {v[0]}/{v[1]}/{v[2]}" for c, v in sorted(outside.items())) or "none"), file=out)
    cpus = sorted(set(per) | {rt}, key=lambda c: (c != rt, -len(per.get(c, []))))
    for cpu in cpus:
        ups = per.get(cpu, [])
        irq = irqs.get(cpu, [])
        starts = [x[0] for x in irq]
        held: set = set()
        none_recorded = 0
        kinds: collections.Counter = collections.Counter()
        for u in ups:
            e = enclosing(irq, starts, u["entry"], u["ret"])
            if e is None:
                none_recorded += 1
            else:
                held.add(e)
                kinds[e[2]] += 1
        timers = [x for x in irq if x[2] == "local timer"]
        with_ = sorted(b - a for (a, b, k) in timers if (a, b, k) in held)
        without = sorted(b - a for (a, b, k) in timers if (a, b, k) not in held)
        mw, mo = quantile(with_, .5), quantile(without, .5)
        diff = mw - mo if mw is not None and mo is not None else None
        dur = sorted(u["ret"] - u["entry"] for u in ups)
        result["cpus"][cpu] = dict(updates=len(ups), none_recorded=none_recorded, kinds=dict(kinds), with_=with_, without=without,
                                   median_difference=diff, durations=dur)
        print(f"  CPU {cpu}{' (real-time)' if cpu == rt else ''}: {len(ups)} updates ({len(ups) / span if span else 0:.1f}/s); "
              f"durations {dist(dur)}", file=out)
        if not ups:
            continue
        print("    enclosing interrupt: " + ", ".join(f"{k} {n}" for k, n in kinds.most_common()) + f", none recorded {none_recorded}", file=out)
        print(f"    local timer interrupts holding an update: {dist(with_)}", file=out)
        print(f"    local timer interrupts holding none:      {dist(without)}", file=out)
        print(f"    difference of the medians, holding an update minus holding none: {us(diff)} us (descriptive: different "
              "interrupts, neither a bound on nor an estimate of the recorder's cost)", file=out)
    rt_ups = sorted((u["entry"], u["ret"]) for u in per.get(rt, []))
    ustarts = [a for a, _ in rt_ups]

    def overlapped(a: int, b: int) -> bool:
        i = bisect.bisect_right(ustarts, b) - 1
        return any(rt_ups[j][1] >= a for j in range(i, max(-1, i - 4), -1))

    sl = sleeps(cap, scanned.get("trace"))
    wakes = sl["slept"] if sl else []
    late = [(s, w) for s, w in wakes if w - s >= LATE_NS]
    waits = [(a, b) for a, b in scanned["waits"] if cap.inside(a, b)]
    long_ = [(a, b) for a, b in waits if b - a >= LATE_NS]
    result.update(sleeps=sl, wakes=len(wakes), late=len(late), late_overlapped=sum(overlapped(s, w) for s, w in late),
                  wakes_overlapped=sum(overlapped(s, w) for s, w in wakes), lateness=sorted(w - s for s, w in wakes),
                  waits=len(waits), long_waits=len(long_), long_overlapped=sum(overlapped(a, b) for a, b in long_),
                  wait_durations=sorted(b - a for a, b in waits))
    if sl is None:
        print(f"  cycle wakes on CPU {rt}: no worker_tid in the summary, so no sleep is established from the trace", file=out)
    else:
        print(f"  cycle wakes on CPU {rt}, the sleeps the trace establishes (one timer wake of the worker in the transition "
              f"from a cycle's end stamp to the next wake stamp; tools/trace_correlate.py's pairing): {len(wakes)} of "
              f"{sl['transitions']} transitions; {sl['began_awake']} began awake, {sl['missing']} without a wake, "
              f"{sl['duplicate']} with more than one", file=out)
        print(f"    lateness (wake stamp minus scheduled wake): {dist(result['lateness'])}", file=out)
        print(f"    {len(late)} woke {LATE_NS // 1000} us or more late, {result['late_overlapped']} of them overlapped by an update "
              f"on CPU {rt}; of all {len(wakes)}, {result['wakes_overlapped']} overlapped by one", file=out)
    if cap.tid is None:
        print("  the worker's waits: no worker_tid in the summary", file=out)
    else:
        print(f"  the worker's waits on CPU {rt} (sched_waking while off the CPU, to its switch-in): {dist(result['wait_durations'])}", file=out)
        print(f"    {len(long_)} of {LATE_NS // 1000} us or more, {result['long_overlapped']} of them overlapped by an update on CPU {rt}", file=out)
    print(f"  an overlap count describes the recorded update bodies on CPU {rt}; it is not proof that the instrumentation had "
          "no effect", file=out)
    return result


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("csv", type=pathlib.Path, help="the timer capture's probe log (<run>.csv)")
    p.add_argument("--report", type=pathlib.Path, help="a saved trace-cmd report -t of the capture (default: run trace-cmd)")
    p.add_argument("--baseline", type=pathlib.Path, help="a capture without the recorder at nominally the same condition (<run>.csv)")
    p.add_argument("--baseline-report", type=pathlib.Path, help="a saved report of the baseline")
    p.add_argument("--rt-cpu", type=int, help="the real-time CPU (default: the run record's, else 3)")
    a = p.parse_args()
    if a.baseline_report and not a.baseline:
        p.error("--baseline-report needs --baseline")
    rt = a.rt_cpu if a.rt_cpu is not None else rt_cpu_of(a.csv)
    tk_txt = pathlib.Path(str(a.csv) + ".tk.txt")
    rec = tk_record.parse_record(tk_txt.read_text(errors="replace")) if tk_txt.exists() else None
    if rec is None or rec["read_status"] != "ok":
        print(f"tk_cost: {a.csv}: no readable timekeeping record ({rec['read_status'] if rec else 'none'})", file=sys.stderr)
        return 2
    cap = Capture(a.csv)
    scanned = scan(report_lines(a.csv, a.report), cap.tid, rt)
    print(f"{a.csv.name}: tools/tk_cost.py (descriptive; it judges and bounds nothing); {scanned['unpaired']} interrupt entries "
          "or exits unpaired")
    analyse(rec["updates"], scanned, cap, rt, sys.stdout)
    if a.baseline:
        bcap = Capture(a.baseline)
        bscan = scan(report_lines(a.baseline, a.baseline_report), bcap.tid, rt)

        def timers(s: dict, c: Capture) -> list:
            return sorted(y - x for x, y, k in s["irqs"].get(rt, []) if k == "local timer" and c.inside(x, y))

        def lateness(s: dict, c: Capture) -> list:
            sl = sleeps(c, s["trace"])
            return sorted(w - x for x, w in sl["slept"]) if sl else []

        def waits(s: dict, c: Capture) -> list:
            return sorted(y - x for x, y in s["waits"] if c.inside(x, y))

        print(f"against {a.baseline.name} (no recorder; {bscan['unpaired']} unpaired; its window {(bcap.window[1] - bcap.window[0]) / 1e9:.6f} s): "
              "a comparison between sessions at nominally the same condition, not a controlled experiment; a difference also "
              "holds whatever else differed, and bounds nothing")
        print(f"  CPU {rt}'s local timer interrupts, this capture: {dist(timers(scanned, cap))}")
        print(f"  CPU {rt}'s local timer interrupts, baseline:     {dist(timers(bscan, bcap))}")
        print(f"  the established sleeps' lateness, this capture: {dist(lateness(scanned, cap))}")
        print(f"  the established sleeps' lateness, baseline:     {dist(lateness(bscan, bcap))}")
        print(f"  the worker's waits, this capture: {dist(waits(scanned, cap))}")
        print(f"  the worker's waits, baseline:     {dist(waits(bscan, bcap))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
