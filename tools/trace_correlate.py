#!/usr/bin/env python3
"""Correlate a traced torque-off run's probe log with its kernel trace.

usage: trace_correlate.py <run>.csv [--report FILE] [--adapter DEVICE]
                          [--top N] [--segments-csv FILE]

Reads the probe's per-cycle log <run>.csv, its summary
<run>.csv.summary.toml (worker_tid, monotonic_origin_ns, the budget's
transfer sizes) and the run's trace: <run>.csv.trace.dat through
`trace-cmd report -t`, or a saved report given with --report. The trace
must be recorded on the mono clock (tools/trace_session.sh checks it), so
its timestamps and the probe's stamps (monotonic_origin_ns + *_ns) share
CLOCK_MONOTONIC.

Association and alignment checks (exit 1 if any fails):

- Timer wakes. A wake of the worker by its sleep timer is a sched_waking
  of the worker inside an hrtimer_wakeup callback: between that expiry's
  hrtimer_expire_entry and hrtimer_expire_exit on the same CPU (no time
  window is involved). The wakes in the run are matched one-to-one to the
  sleeps before the cycles: before every cycle after the first (the first
  begins at once, its scheduled wake being the epoch) and, when the summary
  records a late wake stop, before the final wake, which ends the run
  without a row (the end-of-run case). A sleep is the interval the probe
  stamps, from the previous cycle's end to the cycle's wake stamp, and a
  wake belongs to the sleep that contains its waking; the association uses
  neither the scheduled wake nor the switch-in that B and C test. A sleep
  with two wakes fails, and so does a wake in no sleep. A sleep with no
  wake fails unless the cycle began without sleeping: its scheduled wake
  had passed when the probe was to sleep, so it set no timer, the worker
  never left the CPU in that interval, and the cycle began at or after its
  scheduled wake. Each pair gives three differences that must be
  nonnegative: A, the expiry's trace
  time minus the timer's own now= (its CLOCK_MONOTONIC base, read when the
  interrupt began); B, now= minus the cycle's scheduled wake (the probe
  sleeps for the remaining time, so its timer cannot expire before the
  scheduled wake); C, the cycle's wake stamp minus the worker's next
  switch-in, or minus the waking itself when the timer fired before the
  worker left the CPU. Sign checks show consistency, not a calibrated
  accuracy bound.
- Bus transfers. The adapter's bulk OUT submissions in the run are matched
  one-to-one, in order, to the probe's transfers (each cycle's read
  instruction, then its write), whose lengths the summary gives. A
  submission from the worker must lie between the probe's stamps around the
  call (read_start and instr_return; bus_write_start and write_return).
- Replies. For a Fast Sync Read, the data of the adapter's bulk IN
  completions between a read's instruction submission and the probe's
  last-reply stamp must add up to the reply's length, 8 + N(L + 4) bytes
  (the FTDI's two status bytes per USB packet removed), and no data may
  arrive outside a read.

Receive path (measured, not gated). For each read, from the trace:
instruction submission and completion; the first and last data
completions; the USB completion work (usb_giveback_urb_bh) on the CPU
that took the interrupt; the TTY work (flush_to_ldisc) queued from it,
started and ended by a kernel worker; the worker's resumption (its wake by
that kernel worker and switch-in, when it was blocked) and the probe's
last-reply stamp. Work items are followed by context and work identity
through workqueue_execute_end: a task's work by its thread ID, softirq
work by its CPU. When a read's completion work queued no TTY work (it was
already pending), the TTY work's queue time is not in the trace: that
read's wait is reported separately, from the completion work's start
(bh_to_tty_start_no_queue), never as a queue wait. A read's time is
located before the first USB completion (instruction transport, the wire,
the servos' turnaround, the adapter's buffering and the host's USB
processing), between the completions, or after the last completion (the
data is in the host, not yet with the worker). For a cycle that overran,
the largest measured of these parts, its wake lateness and its write is
reported: where the time went, not what caused it. The adapter's bulk IN
completions within the run (from the first cycle's wake to the last
cycle's end) give the cadence at which it delivered data or status, and
the xHCI interrupt handlers and USB completion work in the run (for every
device on the controller) the host's USB work.

Placement: for each read's last completion, the xHCI interrupt's CPU, the
USB completion work's context (at interrupt exit or in ksoftirqd), the TTY
work's kernel worker and CPU, and that worker's waking after the work was
queued (or that it was already awake) and its switch-in: the queue wait
splits into before the waking, from the waking to the switch-in (the run
queue) and from the switch-in to the work. Summaries by CPU, and the parts
of the queue waits of 100 us or more.

Sleep timing (profiled runs, tools/trace_session.sh --modes profiled): from
each cycle's end stamp to the worker's sleep call (clock_nanosleep or
nanosleep), and for the latest wakes the call, the arming of its timer, the
page faults and the worker's perf samples (<run>.csv.perf.txt) in between.
The cycle-end stamp precedes the cycle's bookkeeping and the sleep call,
and not being descheduled does not show that the probe's code ran.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import pathlib
import re
import subprocess
import sys

try:
    import tomllib
except ImportError:  # Python < 3.11
    tomllib = None

LINE = re.compile(
    r"^\s*(?P<comm>.*?)-(?P<pid>\d+)\s+\[(?P<cpu>\d+)\]\s+(?:(?P<flags>\S+)\s+)?"
    r"(?P<s>\d+)\.(?P<ns>\d{9}):\s+(?P<ev>\w+):\s+(?P<rest>.*)$")
SWITCH = re.compile(r"^(?P<prev>.*):(?P<ppid>\d+) \[-?\d+\] (?P<state>\S+) ==> "
                    r"(?P<next>.*):(?P<npid>\d+) \[-?\d+\]")
URB = re.compile(r"^(?P<dev>\S+) ep(?P<ep>\d+)(?P<dir>in|out)-(?P<type>\w+): urb (?P<urb>\S+)"
                 r".* length (?P<actual>\d+)/(?P<length>\d+)")
QUEUE = re.compile(r"work struct=(?P<work>\S+) function=(?P<fn>\S+)")
EXEC = re.compile(r"work struct (?P<work>[^:\s]+): function (?P<fn>\S+)")
NOW = re.compile(r"now=(\d+)")
PID = re.compile(r"\bpid=(\d+)")

PACKET = 512        # the FTDI's high-speed bulk packet, two status bytes each
STATUS_BYTES = 2
BH_WORK = "usb_giveback_urb_bh"
TTY_WORK = "flush_to_ldisc"
BARRIER_WORK = "wq_barrier_func"


class Trace:
    """The events of one trace that the checks and the receive path use."""

    def __init__(self, tid: int, adapter: str) -> None:
        self.tid = tid
        self.adapter = adapter
        self.expiring: dict[int, list] = {}   # cpu -> stack of (entry ts, now, wakeup)
        self.timer_wakes: list[tuple] = []    # (entry ts, now, cpu, waking ts, running)
        self.worker_switch: list[tuple] = []  # (ts, cpu, "in"/"out", state)
        self.running = False
        self.other_wakes: list[tuple] = []    # (ts, cpu, waker comm, waker pid, work)
        self.out_submit: list[tuple] = []     # (ts, pid, urb, length)
        self.out_done: dict[str, list] = {}   # urb -> completion times
        self.in_done: list[tuple] = []        # (ts, cpu, actual length)
        self.work_stack: dict[tuple, list] = {}  # context -> [(work, fn, start, pid, cpu)]
        self.executions: dict[str, list] = {BH_WORK: [], TTY_WORK: [], BARRIER_WORK: []}
        self.tty_queued: list[tuple] = []     # (ts, cpu, work)
        self.work_anomalies = 0               # execute_end without its start
        self.kworker_in: dict[int, list] = {}  # pid -> [(ts, cpu, prev comm, prev state)]
        self.irq_open: dict[int, tuple] = {}  # cpu -> (entry ts, irq) of an xHCI handler
        self.xhci_irq: list[tuple] = []        # (entry ts, handler duration)
        self.kworker_wake: dict[int, list] = {}  # pid -> [(ts, waker comm, waker cpu, target cpu)]
        # A profiled run's worker events: sleep-call entries, timer arming
        # (ts, expires) of its hrtimer_wakeup timers, and page faults.
        self.sleep_calls: list[int] = []
        self.timer_arms: list[tuple] = []
        self.faults: list[tuple] = []
        self.lines = 0

    def feed(self, line: str) -> None:
        m = LINE.match(line)
        if not m:
            return
        self.lines += 1
        ev, rest = m["ev"], m["rest"]
        cpu, pid = int(m["cpu"]), int(m["pid"])
        ts = int(m["s"]) * 1_000_000_000 + int(m["ns"])
        flags = m["flags"] or ""
        if ev == "hrtimer_expire_entry":
            n = NOW.search(rest)
            self.expiring.setdefault(cpu, []).append(
                (ts, int(n[1]) if n else None, "function=hrtimer_wakeup" in rest))
        elif ev == "hrtimer_expire_exit":
            stack = self.expiring.get(cpu)
            if stack:
                stack.pop()
        elif ev == "sched_waking":
            p = PID.search(rest)
            if not p:
                return
            if int(p[1]) != self.tid:
                if rest.startswith("comm=kworker/"):
                    tc = re.search(r"target_cpu=(\d+)", rest)
                    self.kworker_wake.setdefault(int(p[1]), []).append(
                        (ts, m["comm"], cpu, int(tc[1]) if tc else -1))
                return
            stack = self.expiring.get(cpu)
            if stack and stack[-1][2] and stack[-1][1] is not None:
                entry, now, _ = stack[-1]
                self.timer_wakes.append((entry, now, cpu, ts, self.running))
            else:
                work = self.active_work(flags, cpu, pid)
                self.other_wakes.append((ts, cpu, m["comm"], pid, work))
        elif ev == "sched_switch":
            s = SWITCH.match(rest)
            if not s:
                return
            npid, ppid = int(s["npid"]), int(s["ppid"])
            if npid == self.tid:
                self.worker_switch.append((ts, cpu, "in", ""))
                self.running = True
            elif ppid == self.tid:
                self.worker_switch.append((ts, cpu, "out", s["state"]))
                self.running = False
            if s["next"].startswith("kworker/"):
                self.kworker_in.setdefault(npid, []).append((ts, cpu, s["prev"], s["state"]))
        elif ev in ("xhci_urb_enqueue", "xhci_urb_giveback"):
            u = URB.match(rest)
            if not u or u["dev"] != self.adapter or u["type"] != "bulk":
                return
            if u["dir"] == "out":
                if ev == "xhci_urb_enqueue":
                    self.out_submit.append((ts, pid, u["urb"], int(u["length"])))
                else:
                    self.out_done.setdefault(u["urb"], []).append(ts)
            elif ev == "xhci_urb_giveback":
                self.in_done.append((ts, cpu, int(u["actual"])))
        elif ev in ("sys_enter_clock_nanosleep", "sys_enter_nanosleep"):
            if pid == self.tid:
                self.sleep_calls.append(ts)
        elif ev == "hrtimer_start":
            if pid == self.tid and "function=hrtimer_wakeup" in rest:
                x = re.search(r"\bexpires=(\d+)", rest)
                self.timer_arms.append((ts, int(x[1]) if x else None))
        elif ev in ("page_fault_user", "page_fault_kernel"):
            if pid == self.tid:
                self.faults.append((ts, ev))
        elif ev == "irq_handler_entry":
            if "name=xhci" in rest:
                n = re.search(r"irq=(\d+)", rest)
                if n:
                    self.irq_open[cpu] = (ts, n[1])
        elif ev == "irq_handler_exit":
            n = re.search(r"irq=(\d+)", rest)
            o = self.irq_open.get(cpu)
            if o and n and n[1] == o[1]:
                self.xhci_irq.append((o[0], ts - o[0]))
                del self.irq_open[cpu]
        elif ev == "workqueue_queue_work":
            q = QUEUE.search(rest)
            if q and q["fn"] == TTY_WORK:
                self.tty_queued.append((ts, cpu, q["work"]))
        elif ev == "workqueue_execute_start":
            e = EXEC.search(rest)
            if e:
                key = self.context(flags, cpu, pid)
                self.work_stack.setdefault(key, []).append((e["work"], e["fn"], ts, pid, cpu, m["comm"]))
        elif ev == "workqueue_execute_end":
            e = EXEC.search(rest)
            if not e:
                return
            stack = self.work_stack.get(self.context(flags, cpu, pid), [])
            for i in range(len(stack) - 1, -1, -1):
                if stack[i][0] == e["work"]:
                    work, fn, start, wpid, wcpu, wcomm = stack.pop(i)
                    if fn in self.executions:
                        self.executions[fn].append((start, ts, wpid, wcpu, work, wcomm))
                    break
            else:
                self.work_anomalies += 1

    @staticmethod
    def context(flags: str, cpu: int, pid: int) -> tuple:
        # Softirq work (the USB completion work runs as a BH) belongs to its
        # CPU; a task's work to its thread, wherever it runs.
        if len(flags) > 2 and flags[2] in "sH":
            return ("softirq", cpu)
        return ("task", pid)

    def active_work(self, flags: str, cpu: int, pid: int) -> str:
        stack = self.work_stack.get(self.context(flags, cpu, pid))
        return stack[-1][1] if stack else "-"


def stamp(row: dict, key: str, origin: int) -> int | None:
    v = int(row[key])
    return None if v < 0 else origin + v


def data_bytes(actual: int) -> int:
    packets = max(1, math.ceil(actual / PACKET))
    return max(0, actual - STATUS_BYTES * packets)


def pct(xs: list, q: float) -> float:
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * (len(xs) - 1) + 0.5))]


def show(name: str, xs: list, out) -> None:
    if not xs:
        print(f"    {name:<34} no samples", file=out)
        return
    print(f"    {name:<34} n={len(xs):<6} min {min(xs) / 1e3:9.3f}  p50 {pct(xs, 0.5) / 1e3:9.3f}"
          f"  p99 {pct(xs, 0.99) / 1e3:9.3f}  p99.9 {pct(xs, 0.999) / 1e3:9.3f}"
          f"  max {max(xs) / 1e3:9.3f}", file=out)


class Run:
    def __init__(self, csv_path: pathlib.Path) -> None:
        assert tomllib is not None
        with open(str(csv_path) + ".summary.toml", "rb") as f:
            self.summary = tomllib.load(f)
        res = self.summary["result"]
        self.tid = int(res["worker_tid"])
        self.origin = int(res["monotonic_origin_ns"])
        with open(csv_path, newline="") as f:
            self.rows = list(csv.DictReader(f))
        if not self.rows:
            raise ValueError(f"{csv_path}: no cycles")
        self.kind = self.summary.get("kind", "")
        self.read_instruction = self.summary.get("read_instruction", "")
        budget = self.summary.get("budget", {})
        self.instr_bytes = int(budget.get("read_instruction_bytes", 0))
        self.write_bytes = int(budget.get("write_bytes", 0))
        self.period_ns = round(1e9 / float(self.summary["rate_hz"]))

    def reply_bytes(self, read_len: int) -> int | None:
        if self.read_instruction != "fast_sync" or self.instr_bytes <= 14:
            return None
        return 8 + (self.instr_bytes - 14) * (read_len + 4)


def check_timer_wakes(run: Run, tr: Trace, failures: list, out) -> dict:
    o, rows = run.origin, run.rows
    res = run.summary["result"]
    lo = o + int(rows[0]["wake_ns"])
    late = bool(res.get("late_wake_stop", False))
    hi = o + int(rows[-1]["cycle_end_ns"])
    if late:
        hi = o + round(float(res["late_wake_s"]) * 1e9)
    wakes = [w for w in tr.timer_wakes if lo < w[3] <= hi]
    # The sleeps, as the probe stamps them: from the previous cycle's end to
    # the cycle's wake stamp, for every row after the first (which begins at
    # once), and the late wake that ended the run without a row.
    sleeps = []
    for i in range(1, len(rows)):
        sleeps.append((i, o + int(rows[i - 1]["cycle_end_ns"]),
                       o + int(rows[i]["scheduled_wake_ns"]), o + int(rows[i]["wake_ns"])))
    if late:
        lw = round(float(res["late_wake_s"]) * 1e9)
        lat = round(float(res["late_wake_lateness_s"]) * 1e9)
        sleeps.append((None, o + int(rows[-1]["cycle_end_ns"]), o + lw - lat, o + lw))
    # Each wake belongs to the sleep that contains its waking: a causal,
    # one-to-one association that uses neither the scheduled wake (B) nor
    # the switch-in (C).
    waking_ts = [w[3] for w in wakes]
    outs = [s[0] for s in tr.worker_switch if s[2] == "out"]
    pairs, used = [], 0
    duplicate = missing = began_awake = 0
    for sleep in sleeps:
        _, prev_end, sched, wake = sleep
        k0 = bisect.bisect_right(waking_ts, prev_end)
        k1 = bisect.bisect_right(waking_ts, wake)
        n = k1 - k0
        used += n
        if n == 1:
            pairs.append((wakes[k0], sleep))
        elif n > 1:
            duplicate += 1
        else:
            # Without a wake, the cycle must have begun without sleeping:
            # its scheduled wake had passed when it was to sleep, so the
            # worker set no timer and never left the CPU.
            left = bisect.bisect_right(outs, wake) - bisect.bisect_right(outs, prev_end)
            if left == 0 and wake >= sched:
                began_awake += 1
            else:
                missing += 1
    extra = len(wakes) - used
    if duplicate:
        failures.append(f"timer wakes: {duplicate} sleeps with more than one wake")
    if missing:
        failures.append(f"timer wakes: {missing} sleeps without a wake")
    if extra:
        failures.append(f"timer wakes: {extra} wakes outside every sleep")
    ins = [s[0] for s in tr.worker_switch if s[2] == "in"]
    a, b, c, c_on = [], [], [], []
    for (entry, now, _, waking, running), (_, _, sched, wake) in pairs:
        a.append(entry - now)
        b.append(now - sched)
        if running:
            c_on.append(wake - waking)
        else:
            j = bisect.bisect_left(ins, waking)
            c.append(wake - ins[j] if j < len(ins) else -1)
    for name, xs in (("A", a), ("B", b), ("C", c), ("C (still on the CPU)", c_on)):
        neg = sum(1 for x in xs if x < 0)
        if neg:
            failures.append(f"timer wakes: {neg} negative {name}")
    print(f"  timer wakes: {len(wakes)} in the run for {len(sleeps)} sleeps"
          f"{' (with the late wake that ended the run)' if late else ''}: {len(pairs)} paired, "
          f"{began_awake} cycles began without sleeping, {missing} missing, {duplicate} duplicate, "
          f"{extra} extra", file=out)
    show("A expiry trace time - timer now=", a, out)
    show("B timer now= - scheduled wake", b, out)
    show("C wake stamp - switch-in", c, out)
    show("C wake stamp - waking (on the CPU)", c_on, out)
    return {"pairs": pairs, "switch_in": [(s[0], s[1]) for s in tr.worker_switch if s[2] == "in"],
            "switch_out": outs}


def check_transfers(run: Run, tr: Trace, failures: list, out) -> dict:
    o, rows = run.origin, run.rows
    lo, hi = o + int(rows[0]["wake_ns"]), o + int(rows[-1]["cycle_end_ns"])
    expected = []  # (kind, row index, start stamp, return stamp, length)
    for i, r in enumerate(rows):
        rs, ir = stamp(r, "read_start_ns", o), stamp(r, "instr_return_ns", o)
        if rs is not None and ir is not None:
            expected.append(("read", i, rs, ir, run.instr_bytes))
        ws, wr = stamp(r, "bus_write_start_ns", o), stamp(r, "write_return_ns", o)
        if ws is not None and wr is not None:
            expected.append(("write", i, ws, wr, run.write_bytes))
    subs = [s for s in tr.out_submit if lo <= s[0] <= hi]
    if len(subs) != len(expected):
        failures.append(f"bus transfers: {len(subs)} OUT submissions for {len(expected)} probe transfers")
    bad_len = outside = deferred = 0
    paired = []
    for (ts, pid, urb, length), (kind, i, start, ret, want) in zip(subs, expected):
        if want and length != want:
            bad_len += 1
        if pid == run.tid:
            if not (start <= ts <= ret):
                outside += 1
        else:
            deferred += 1
            if ts < start:
                outside += 1
        done = tr.out_done.get(urb, [])
        k = bisect.bisect_left(done, ts)
        paired.append((kind, i, ts, done[k] if k < len(done) else None))
    if bad_len:
        failures.append(f"bus transfers: {bad_len} submissions with a length other than the probe's")
    if outside:
        failures.append(f"bus transfers: {outside} submissions outside the probe's stamps")
    print(f"  bus transfers: {len(subs)} OUT submissions for {len(expected)} probe transfers"
          f" ({deferred} submitted outside the worker)", file=out)
    return {"paired": paired}


def receive_path(run: Run, tr: Trace, transfers: dict, failures: list, out) -> list[dict]:
    o, rows = run.origin, run.rows
    reads = [(i, ts, done) for kind, i, ts, done in transfers["paired"] if kind == "read"]
    data = [(ts, cpu, data_bytes(a)) for ts, cpu, a in tr.in_done]
    data = [d for d in data if d[2] > 0]
    data_ts = [d[0] for d in data]
    bh = {}
    for start, end, _, cpu, _, comm in tr.executions[BH_WORK]:
        bh.setdefault(cpu, []).append((start, end, comm))
    for v in bh.values():
        v.sort()
    bh_starts = {cpu: [s for s, *_ in v] for cpu, v in bh.items()}
    queued = sorted(tr.tty_queued)
    queued_ts = [q[0] for q in queued]
    # Other TTYs (terminals, ptys) run flush_to_ldisc too: the adapter's
    # port is the work item that its USB completion work queues.
    port_work = port_work_item(data, bh, queued)
    tty = sorted(e for e in tr.executions[TTY_WORK] if e[4] == port_work)
    tty_starts = [t[0] for t in tty]
    switches = tr.worker_switch
    switch_ts = [s[0] for s in switches]
    other = tr.other_wakes
    other_ts = [w[0] for w in other]
    assigned = set()
    checked = short = 0
    rec = []
    for i, submit, done in reads:
        r = rows[i]
        last_reply = stamp(r, "last_reply_ns", o)
        if int(r["read_ok"]) != 1 or last_reply is None:
            continue
        j0 = bisect.bisect_right(data_ts, submit)
        j1 = bisect.bisect_right(data_ts, last_reply)
        mine = data[j0:j1]
        assigned.update(range(j0, j1))
        want = run.reply_bytes(int(r["read_len"]))
        got = sum(d[2] for d in mine)
        if want is not None:
            checked += 1
            if got != want:
                short += 1
        if not mine:
            continue
        first, (last, last_cpu, _) = mine[0][0], mine[-1]
        x = {"cycle": int(r["cycle"]), "row": i, "read_len": int(r["read_len"]),
             "completions": len(mine), "irq_cpu": last_cpu, "read_start": o + int(r["read_start_ns"]),
             "submit": submit, "instr_done": done, "first": first, "last": last,
             "last_reply": last_reply, "read_end": stamp(r, "read_end_ns", o)}
        # The USB completion work on the interrupt's CPU: running then, or next.
        st = bh_starts.get(last_cpu, [])
        k = bisect.bisect_right(st, last) - 1
        if k >= 0 and bh[last_cpu][k][1] >= last:
            x["bh_start"], x["bh_end"], x["bh_comm"] = bh[last_cpu][k]
        elif k + 1 < len(st):
            x["bh_start"], x["bh_end"], x["bh_comm"] = bh[last_cpu][k + 1]
        # The TTY work queued from it (none if it was already pending).
        q = bisect.bisect_left(queued_ts, last)
        while "bh_end" in x and q < len(queued) and queued[q][0] <= x["bh_end"]:
            if queued[q][1] == last_cpu and queued[q][2] == port_work:
                x["tty_queue"] = queued[q][0]
                break
            q += 1
        after = x.get("tty_queue", last)
        t = bisect.bisect_left(tty_starts, after)
        if t < len(tty) and tty[t][0] <= last_reply:
            x["tty_start"], x["tty_end"], x["tty_pid"], x["tty_cpu"] = tty[t][:4]
            # Was the worker blocked when the TTY work ended?
            s = bisect.bisect_right(switch_ts, x["tty_end"]) - 1
            if s >= 0 and switches[s][2] == "out":
                x["blocked"] = switches[s][3]
                # Its waker: the kernel worker that ran the port's TTY work.
                w = bisect.bisect_left(other_ts, x["tty_start"])
                while w < len(other) and other[w][0] <= last_reply:
                    if other[w][3] == x["tty_pid"]:
                        x["woken"], x["waker_work"] = other[w][0], other[w][4]
                        break
                    w += 1
                n = s + 1
                if n < len(switches) and switches[n][0] <= last_reply:
                    x["switch_in"] = switches[n][0]
            # Where the kernel worker came from before it ran the work.
            kin = tr.kworker_in.get(x["tty_pid"], [])
            kk = bisect.bisect_right([e[0] for e in kin], x["tty_start"]) - 1
            if kk >= 0:
                x["kworker_in"], x["kworker_in_cpu"], x["kworker_prev"], _ = kin[kk]
            # Its waking after the work was queued, if it slept: else it was
            # already awake (running or runnable).
            since = x.get("tty_queue", x.get("bh_start", last))
            kw = tr.kworker_wake.get(x["tty_pid"], [])
            kj = bisect.bisect_right([e[0] for e in kw], x["tty_start"]) - 1
            if kj >= 0 and kw[kj][0] >= since:
                x["kworker_woken"], x["kworker_waker"], _, x["kworker_target"] = kw[kj]
            else:
                x["kworker_awake"] = True
        rec.append(x)
    lo, hi = o + int(rows[0]["wake_ns"]), o + int(rows[-1]["cycle_end_ns"])
    stray = [k for k, d in enumerate(data) if lo <= d[0] <= hi and k not in assigned]
    if short:
        failures.append(f"replies: {short} of {checked} reads whose completions do not add up to the reply")
    if stray:
        failures.append(f"replies: {len(stray)} data completions outside any read "
                        f"({sum(data[k][2] for k in stray)} bytes)")
    print(f"  replies: {checked} reads checked, {short} not adding up, "
          f"{len(stray)} data completions outside a read", file=out)
    return rec


def usb_cadence(run: Run, tr: Trace, out) -> None:
    """The adapter's bulk IN completions (data or status only) in the run."""
    o, rows = run.origin, run.rows
    lo, hi = o + int(rows[0]["wake_ns"]), o + int(rows[-1]["cycle_end_ns"])
    ts = [c[0] for c in tr.in_done if lo <= c[0] <= hi]
    if len(ts) < 2:
        print(f"  adapter bulk IN completions in the run: {len(ts)}", file=out)
        return
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    print(f"  adapter bulk IN completions in the run: {len(ts)} over {(ts[-1] - ts[0]) / 1e9:.6f} s "
          "(first to last); consecutive intervals:", file=out)
    show("in_completion_interval", gaps, out)
    # The host's USB work, for every device on the controller.
    span = (hi - lo) / 1e9
    irqs = [d for s, d in tr.xhci_irq if lo <= s <= hi]
    works = [e - s for s, e, *_ in tr.executions[BH_WORK] if lo <= s <= hi]
    print(f"  host USB work in the run (every device on the controller): {len(irqs)} xHCI interrupts "
          f"({len(irqs) / span:.0f} per s, handlers {sum(irqs) / 1e6:.3f} ms in all), {len(works)} USB "
          f"completion works ({sum(works) / 1e6:.3f} ms in all)", file=out)
    show("xhci_interrupt_handler", irqs, out)
    show("usb_completion_work", works, out)


def port_work_item(data: list, bh: dict, queued: list) -> str | None:
    """The TTY work item queued most often by the USB completion work that
    follows the adapter's data completions."""
    counts: dict[str, int] = {}
    starts = {cpu: [s for s, *_ in v] for cpu, v in bh.items()}
    queued_ts = [q[0] for q in queued]
    for ts, cpu, _ in data:
        st = starts.get(cpu, [])
        k = bisect.bisect_right(st, ts) - 1
        if not (k >= 0 and bh[cpu][k][1] >= ts):
            k += 1
        if k >= len(st):
            continue
        start, end = bh[cpu][k][:2]
        q = bisect.bisect_left(queued_ts, max(ts, start))
        while q < len(queued) and queued[q][0] <= end:
            if queued[q][1] == cpu:
                counts[queued[q][2]] = counts.get(queued[q][2], 0) + 1
                break
            q += 1
    return max(counts.items(), key=lambda kv: kv[1])[0] if counts else None


def segments(x: dict) -> dict:
    s = {}

    def put(name, a, b):
        if a in x and b in x and x[a] is not None and x[b] is not None:
            s[name] = x[b] - x[a]
    put("before_send", "read_start", "submit")
    put("instruction_usb", "submit", "instr_done")
    put("to_first_completion", "instr_done", "first")
    put("between_completions", "first", "last")
    put("completion_to_bh", "last", "bh_start")
    put("bh_to_tty_queue", "bh_start", "tty_queue")
    put("tty_queue_to_start", "tty_queue", "tty_start")
    put("tty_run", "tty_start", "tty_end")
    put("tty_end_to_waking", "tty_end", "woken")
    put("waking_to_switch_in", "woken", "switch_in")
    put("tty_end_to_last_reply", "tty_end", "last_reply")
    put("after_last_completion", "last", "last_reply")
    put("host_transaction", "read_start", "read_end")
    if "tty_queue" not in x:
        # No queue event: the work was already pending, so its queue time is
        # unknown. Not a queue wait; reported apart from tty_queue_to_start.
        put("bh_to_tty_start_no_queue", "bh_start", "tty_start")
    # The TTY work's kernel worker: before its waking, from its waking to
    # its switch-in (the run queue), from its switch-in to the work.
    if "tty_queue" in x and "kworker_woken" in x:
        put("tty_queue_to_kworker_waking", "tty_queue", "kworker_woken")
        if x.get("kworker_in", -1) >= x["kworker_woken"]:
            put("kworker_waking_to_switch_in", "kworker_woken", "kworker_in")
            put("kworker_switch_in_to_tty_start", "kworker_in", "tty_start")
    return s


def report(run: Run, rec: list[dict], timer: dict, top: int, out) -> None:
    segs = [segments(x) for x in rec]
    print("  receive path per read (us): from the trace, with the probe's stamps", file=out)
    for name in ("before_send", "instruction_usb", "to_first_completion", "between_completions",
                 "after_last_completion", "completion_to_bh", "bh_to_tty_queue",
                 "tty_queue_to_start", "bh_to_tty_start_no_queue", "tty_run",
                 "tty_end_to_last_reply", "tty_end_to_waking", "waking_to_switch_in",
                 "host_transaction"):
        show(name, [s[name] for s in segs if name in s], out)
    no_queue = sum(1 for x in rec if "tty_start" in x and "tty_queue" not in x)
    print(f"  {no_queue} reads whose USB completion work queued no TTY work (already pending): "
          "their wait is bh_to_tty_start_no_queue, not a queue wait", file=out)
    split = [x for x in rec if x["completions"] > 1]
    blocked = sum(1 for x in rec if "blocked" in x)
    print(f"  {len(split)} of {len(rec)} replies arrived in more than one completion; "
          f"the worker was blocked when the TTY work ended in {blocked}", file=out)
    wakers = {}
    for x in rec:
        if "waker_work" in x:
            wakers[x["waker_work"]] = wakers.get(x["waker_work"], 0) + 1
    if wakers:
        print("  the worker's wakers' work: " + ", ".join(f"{k} {v}" for k, v in sorted(wakers.items())), file=out)

    # Overruns: the cycle's largest measured part, a location, not a cause.
    o, rows = run.origin, run.rows
    by_row = {x["row"]: s for x, s in zip(rec, segs)}
    largest: dict[str, int] = {}
    overran = [i for i, r in enumerate(rows) if int(r["skipped_periods"]) > 0]
    for i in overran:
        r = rows[i]
        s = by_row.get(i, {})
        parts = {
            "wake lateness": int(r["wake_ns"]) - int(r["scheduled_wake_ns"]),
            "before the first USB completion": s.get("to_first_completion", 0) + s.get("instruction_usb", 0),
            "between USB completions": s.get("between_completions", 0),
            "after the last USB completion": s.get("after_last_completion", 0),
        }
        ws, wr = int(r["bus_write_start_ns"]), int(r["write_return_ns"])
        if ws >= 0 and wr >= 0:
            parts["write"] = wr - ws
        part = max(parts.items(), key=lambda kv: kv[1])[0]
        largest[part] = largest.get(part, 0) + 1
    print(f"  overran cycles: {len(overran)}; largest measured part: "
          + (", ".join(f"{k} {v}" for k, v in sorted(largest.items(), key=lambda kv: -kv[1])) or "-"),
          file=out)
    del o

    if top > 0 and rec:
        print(f"  slowest {min(top, len(rec))} reads (us from the instruction's submission):", file=out)
        for x, s in sorted(zip(rec, segs), key=lambda xs: -xs[1].get("host_transaction", 0))[:top]:
            def at(k, x=x):
                return f"{(x[k] - x['submit']) / 1e3:8.1f}" if k in x and x[k] is not None else "       -"
            extra = ""
            if "tty_cpu" in x:
                extra = f" tty work pid {x['tty_pid']} cpu {x['tty_cpu']}"
                if "kworker_prev" in x:
                    extra += f" (switched in after {x['kworker_prev']})"
            print(f"    cycle {x['cycle']:>6} txn {s.get('host_transaction', 0) / 1e3:7.1f}: "
                  f"instr done {at('instr_done')} first {at('first')} last {at('last')} "
                  f"bh {at('bh_start')} tty {at('tty_start')}-{at('tty_end')} "
                  f"reply {at('last_reply')} ({x['completions']} completions){extra}", file=out)
        wl = sorted(((int(r["wake_ns"]) - int(r["scheduled_wake_ns"]), i) for i, r in enumerate(rows)),
                    reverse=True)[:top]
        ins = [(s[0], s[1]) for s in tr_switch_in(timer)]
        ins_ts = [s[0] for s in ins]
        outs_ts = timer.get("switch_out", [])
        by_cycle = {}
        for (entry, now, cpu, waking, running), (i, prev_end, sched, wake) in timer["pairs"]:
            if i is not None:
                by_cycle[i] = (entry, now, cpu, waking, running, prev_end, sched, wake)
        print(f"  latest {len(wl)} wakes (us from the scheduled wake):", file=out)
        for lat, i in wl:
            if i not in by_cycle:
                why = "the first cycle (begins at once)" if i == 0 else "no timer wake (began without sleeping)"
                print(f"    cycle {rows[i]['cycle']:>6} wake {lat / 1e3:7.1f}: {why}", file=out)
                continue
            entry, now, cpu, waking, running, prev_end, sched, wake = by_cycle[i]
            j = bisect.bisect_left(ins_ts, waking)
            sw = "still on the CPU" if running else (
                f"switch-in {(ins[j][0] - sched) / 1e3:7.1f}" if j < len(ins) else "no switch-in")
            k = bisect.bisect_right(outs_ts, waking) - 1
            slept = (f"left the CPU {(outs_ts[k] - sched) / 1e3:7.1f}"
                     if not running and k >= 0 and outs_ts[k] >= prev_end else "did not leave the CPU")
            print(f"    cycle {rows[i]['cycle']:>6} wake {lat / 1e3:7.1f}: previous cycle ended "
                  f"{(prev_end - sched) / 1e3:7.1f}, {slept}, timer interrupt {(now - sched) / 1e3:7.1f} "
                  f"(CPU {cpu}), callback {(entry - sched) / 1e3:7.1f}, waking "
                  f"{(waking - sched) / 1e3:7.1f}, {sw}", file=out)


def tr_switch_in(timer: dict) -> list:
    return timer.get("switch_in", [])


def placement(rec: list[dict], out) -> None:
    """Where each read's last completion was handled: the xHCI interrupt's
    CPU, the USB completion work's context, the TTY work's kernel worker
    and its CPU, and that worker's waking and switch-in."""
    if not rec:
        return
    print("  placement (each read's last completion):", file=out)

    def counts(values) -> str:
        c: dict = {}
        for v in values:
            c[v] = c.get(v, 0) + 1
        return ", ".join(f"{k}: {v}" for k, v in sorted(c.items(), key=lambda kv: (-kv[1], str(kv[0]))))
    print(f"    xHCI interrupt CPUs: {counts(x['irq_cpu'] for x in rec)}", file=out)
    print("    USB completion work: " + counts(
        "in ksoftirqd" if x.get("bh_comm", "").startswith("ksoftirqd") else "at interrupt exit"
        for x in rec if "bh_comm" in x), file=out)
    segs = [(x, segments(x)) for x in rec]
    print("    TTY work by CPU (queue wait, us):", file=out)
    by_cpu: dict[int, list] = {}
    for x, s in segs:
        if "tty_cpu" in x and "tty_queue_to_start" in s:
            by_cpu.setdefault(x["tty_cpu"], []).append(s["tty_queue_to_start"])
    for cpu in sorted(by_cpu, key=lambda c: -len(by_cpu[c])):
        w = by_cpu[cpu]
        print(f"      CPU {cpu:>2}: {len(w):>6} reads, p50 {pct(w, 0.5) / 1e3:8.1f}  p99 {pct(w, 0.99) / 1e3:8.1f}"
              f"  max {max(w) / 1e3:8.1f}", file=out)
    woken = [x for x in rec if "kworker_woken" in x]
    awake = [x for x in rec if x.get("kworker_awake")]
    print(f"    its kernel worker was woken after the queue in {len(woken)} reads and already awake "
          f"in {len(awake)}", file=out)
    for name in ("tty_queue_to_kworker_waking", "kworker_waking_to_switch_in", "kworker_switch_in_to_tty_start"):
        show(name, [s[name] for _, s in segs if name in s], out)
    tail = [(x, s) for x, s in segs if s.get("tty_queue_to_start", 0) >= 100_000]
    if tail:
        largest: dict[str, int] = {}
        prev: dict[str, int] = {}
        for x, s in tail:
            parts = {k: s[k] for k in ("tty_queue_to_kworker_waking", "kworker_waking_to_switch_in",
                                       "kworker_switch_in_to_tty_start") if k in s}
            key = max(parts.items(), key=lambda kv: kv[1])[0] if parts else (
                "already awake" if x.get("kworker_awake") else "-")
            largest[key] = largest.get(key, 0) + 1
            p = re.sub(r"[/:]?\d+$", "", x.get("kworker_prev", "-"))
            prev[p] = prev.get(p, 0) + 1
        print(f"    queue waits of 100 us or more: {len(tail)} reads; largest part: "
              + ", ".join(f"{k} {v}" for k, v in sorted(largest.items(), key=lambda kv: -kv[1])), file=out)
        print("      the kernel worker was switched in after: "
              + ", ".join(f"{k} {v}" for k, v in sorted(prev.items(), key=lambda kv: -kv[1])[:6]), file=out)


PERF_LINE = re.compile(r"^\s*(?P<comm>.*?)\s+(?P<tid>\d+)\s+\[(?P<cpu>\d+)\]\s+(?P<s>\d+)\.(?P<f>\d+):"
                       r"\s+(?P<ip>[0-9a-f]+)\s+(?P<sym>.*?)\s+\((?P<dso>[^()]*)\)\s*$")


def load_perf(path: pathlib.Path, tid: int) -> list[tuple]:
    """The worker's perf samples, (ts, symbol, dso), from perf script text."""
    samples = []
    for line in path.read_text(errors="replace").splitlines():
        m = PERF_LINE.match(line)
        if m and int(m["tid"]) == tid:
            f = (m["f"] + "000000000")[:9]
            samples.append((int(m["s"]) * 1_000_000_000 + int(f), m["sym"], m["dso"]))
    return sorted(samples)


def sleep_timing(run: Run, tr: Trace, timer: dict, perf: list | None, top: int, out) -> None:
    """A profiled run's sleeps: from each cycle's end stamp to the worker's
    sleep call, and for the latest wakes the call, the timer's arming, page
    faults and the worker's perf samples in between."""
    if not tr.sleep_calls:
        print("  sleep timing: no sleep-call events (not a profiled run)", file=out)
        return
    calls = sorted(tr.sleep_calls)
    arms = sorted(tr.timer_arms)
    arm_ts = [a[0] for a in arms]
    faults = sorted(f[0] for f in tr.faults)
    perf_ts = [p[0] for p in perf] if perf else []
    gaps, by_cycle = [], {}
    for (_, now, _, waking, _), (i, prev_end, sched, wake) in timer["pairs"]:
        k = bisect.bisect_right(calls, prev_end)
        if k < len(calls) and calls[k] <= waking:
            gaps.append(calls[k] - prev_end)
            if i is not None:
                by_cycle[i] = (prev_end, calls[k], sched, now)
    print(f"  sleep timing: {len(calls)} sleep calls of the worker, {len(arms)} timers armed, "
          f"{len(faults)} page faults{f', {len(perf)} perf samples' if perf is not None else ''}", file=out)
    show("cycle_end_to_sleep_call", gaps, out)
    rows = run.rows
    late = sorted(((int(r["wake_ns"]) - int(r["scheduled_wake_ns"]), i) for i, r in enumerate(rows)
                   if i in by_cycle), reverse=True)[:top]
    for lat, i in late:
        prev_end, call, sched, now = by_cycle[i]
        a = bisect.bisect_left(arm_ts, call)
        armed = arms[a] if a < len(arms) else None
        nf = bisect.bisect_right(faults, call) - bisect.bisect_right(faults, prev_end)
        line = (f"    cycle {rows[i]['cycle']:>6} wake {lat / 1e3:7.1f}: sleep call "
                f"{(call - prev_end) / 1e3:+8.1f} after the cycle-end stamp")
        if armed is not None:
            line += f", timer armed {(armed[0] - prev_end) / 1e3:+8.1f}"
            if armed[1] is not None:
                line += f" for {(armed[1] - sched) / 1e3:+.1f} from the scheduled wake"
        line += f", expired {(now - sched) / 1e3:+.1f}; {nf} page faults"
        if perf is not None:
            p0, p1 = bisect.bisect_right(perf_ts, prev_end), bisect.bisect_right(perf_ts, call)
            syms: dict[str, int] = {}
            for _, sym, _ in perf[p0:p1]:
                syms[sym] = syms.get(sym, 0) + 1
            line += f"; perf {p1 - p0} samples" + (": " + ", ".join(
                f"{k} {v}" for k, v in sorted(syms.items(), key=lambda kv: -kv[1])[:3]) if syms else "")
        print(line, file=out)


def write_segments(path: pathlib.Path, rec: list[dict]) -> None:
    names = ["cycle", "read_len", "completions", "irq_cpu", "bh_comm", "tty_pid", "tty_cpu", "blocked",
             "kworker_prev", "kworker_waker", "kworker_target"]
    keys = ["before_send", "instruction_usb", "to_first_completion", "between_completions",
            "after_last_completion", "completion_to_bh", "bh_to_tty_queue", "tty_queue_to_start",
            "bh_to_tty_start_no_queue", "tty_queue_to_kworker_waking", "kworker_waking_to_switch_in",
            "kworker_switch_in_to_tty_start", "tty_run", "tty_end_to_last_reply", "tty_end_to_waking",
            "waking_to_switch_in", "host_transaction"]
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(names + [k + "_ns" for k in keys])
        for x in rec:
            s = segments(x)
            w.writerow([x.get(n, "") for n in names] + [s.get(k, "") for k in keys])


def adapter_from_record(csv_path: pathlib.Path) -> str | None:
    note = pathlib.Path(str(csv_path) + ".run.txt")
    if not note.exists():
        return None
    for line in note.read_text(errors="replace").splitlines():
        m = re.match(r"^adapter: (\S+),", line)
        if m:
            return pathlib.Path(m[1]).name
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    parser.add_argument("csv", type=pathlib.Path, help="the probe's per-cycle log (<run>.csv)")
    parser.add_argument("--report", type=pathlib.Path,
                        help="a saved `trace-cmd report -t` of the run (default: run trace-cmd)")
    parser.add_argument("--adapter", help="the adapter's USB device path in xHCI events, e.g. 1-9 "
                        "(default: the run record's adapter line)")
    parser.add_argument("--top", type=int, default=10, help="slowest reads and latest wakes to list")
    parser.add_argument("--segments-csv", type=pathlib.Path, help="write each read's receive path here")
    parser.add_argument("--perf-txt", type=pathlib.Path,
                        help="a profiled run's worker samples (default: <run>.csv.perf.txt if it exists)")
    args = parser.parse_args()
    if tomllib is None:
        print("trace_correlate: needs Python 3.11 or later (tomllib)", file=sys.stderr)
        return 2
    try:
        run = Run(args.csv)
    except (OSError, KeyError, ValueError) as e:
        print(f"trace_correlate: {e}", file=sys.stderr)
        return 2
    if run.kind != "production-traffic":
        print(f"trace_correlate: {args.csv}: a {run.kind or 'unknown'} run; only production-traffic "
              "runs are handled", file=sys.stderr)
        return 2
    adapter = args.adapter or adapter_from_record(args.csv)
    if not adapter:
        print("trace_correlate: no adapter (give --adapter)", file=sys.stderr)
        return 2
    tr = Trace(run.tid, adapter)
    if args.report:
        with open(args.report, errors="replace") as f:
            for line in f:
                tr.feed(line)
    else:
        dat = str(args.csv) + ".trace.dat"
        proc = subprocess.Popen(["trace-cmd", "report", "-t", "-i", dat], stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True, errors="replace")
        assert proc.stdout is not None
        for line in proc.stdout:
            tr.feed(line)
        if proc.wait() != 0:
            print(f"trace_correlate: trace-cmd report -i {dat} failed", file=sys.stderr)
            return 2
    out = sys.stdout
    failures: list[str] = []
    print(f"{args.csv.name}: worker {run.tid}, {len(run.rows)} cycles at {run.summary['rate_hz']} Hz, "
          f"adapter {adapter}, {tr.lines} trace lines", file=out)
    timer = check_timer_wakes(run, tr, failures, out)
    transfers = check_transfers(run, tr, failures, out)
    rec = receive_path(run, tr, transfers, failures, out)
    usb_cadence(run, tr, out)
    if tr.work_anomalies:
        print(f"  {tr.work_anomalies} workqueue executions ended without a recorded start "
              "(begun before the trace)", file=out)
    report(run, rec, timer, args.top, out)
    placement(rec, out)
    perf_txt = args.perf_txt or pathlib.Path(str(args.csv) + ".perf.txt")
    perf = load_perf(perf_txt, run.tid) if perf_txt.exists() else None
    sleep_timing(run, tr, timer, perf, args.top, out)
    if args.segments_csv:
        write_segments(args.segments_csv, rec)
    if failures:
        print("association and alignment: FAIL", file=out)
        for f in failures:
            print(f"  {f}", file=out)
        return 1
    print("association and alignment: PASS", file=out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
