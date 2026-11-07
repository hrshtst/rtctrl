"""Timer service analysis of a timer-mode traced torque-off run, for
tools/trace_correlate.py (docs/hardware/control-rate.md, timer service
diagnostics).

For each of the worker's sleeps it lays out, on the real-time CPU, the
deadline the clock event device was programmed with (the TSC deadline
MSR's writes, the observed timeline), the timers armed and cancelled, the
CPU's idle transitions, its interrupts and NMIs, and the worker's service.

The deadlines are TSC counts. They are converted to CLOCK_MONOTONIC with
anchors: the real-time CPU's local timer interrupts, recorded both in the
main trace (mono clock) and in a second trace instance on the x86-tsc
clock. The anchors are matched one-to-one by their interval pattern,
which uses only the TSC frequency (from the independent sampler), never
the conversion under test, and a missing or duplicate anchor fails. A
deadline is converted by linear interpolation between the anchors around
it, never extrapolated. The conversion's uncertainty U is validated
independently: the residuals of held-out anchors (every second anchor,
predicted from the others), and the residuals of a bracketed TSC and
CLOCK_MONOTONIC sampler (tools' tsc_mono_pairs), each bounded by its
bracket. The classification uses U in its boundaries, and a U above 5 us
fails the run.

A sleep whose timer was served 50 us or more late (the expiry's now= minus
the timer's expiry, both monotonic) is described by its recorded evidence,
not by a cause: a late sleep call; a deadline late (the deadline in effect
at the expiry later than it by more than U, or disarmed); an interrupt late
from idle or while busy (the deadline on time); and INDETERMINATE when the
deadline cannot be converted. A late service is a result, not a failure.

A rehearsal's exerciser (tools/timer_exerciser.py) arms, replaces and
cancels timerfd timers (its "expired" rows are its own observations, not
actions); each action must match its trace events, and the
deadline in effect after it must cover the earliest known pending expiry
on the real-time CPU (a missing write is legitimate when an earlier
deadline already covers it). A timer whose destination CPU is not shown by
its expiry or by a matching write is INDETERMINATE; an action its events
or the observed state do not explain fails.
"""

from __future__ import annotations

import bisect
import csv
import pathlib
import re
import statistics

U_LIMIT_NS = 5_000       # the provisional bound on the validated uncertainty
LATE_NS = 50_000         # a timer service this late is classified
RES_NS = 1               # timestamp resolution added to U (mono 1 ns; the TSC finer)
MSR_TSC_DEADLINE = "6e0"
IDLE_EXIT = 4294967295   # power:cpu_idle's state on leaving idle
ANCHOR_CHECK = 50        # intervals compared when aligning the anchors
TIMER_EVENTS = {"write_msr", "local_timer_entry", "cpu_idle", "cpu_idle_miss", "nmi_handler",
                "sys_enter", "sys_exit", "page_fault_user", "page_fault_kernel", "hrtimer_start",
                "hrtimer_cancel", "hrtimer_expire_entry", "hrtimer_init", "hrtimer_setup",
                "sched_process_fork", "sched_process_exec", "sched_process_exit",
                "irq_handler_entry", "irq_handler_exit"}

WRITE = re.compile(r"^([0-9a-f]+), value ([0-9a-f]+)(\s+#GP)?")
ADDR = re.compile(r"hrtimer=(\S+)")
FUNC = re.compile(r"function=([^\s/]+)")
EXPIRES = re.compile(r"\bexpires=(-?\d+)")
SOFT = re.compile(r"\bsoftexpires=(-?\d+)")
MODE = re.compile(r"\bmode=(\S+)")
CLOCKID = re.compile(r"\bclockid=(\S+)")
NOW = re.compile(r"\bnow=(-?\d+)")
STATE = re.compile(r"\bstate=(\d+)")
CPUID = re.compile(r"\bcpu_id=(\d+)")
NMI = re.compile(r"^(\S+?)(?:\(\))?\s+delta_ns:\s*(\d+)")
FORK = re.compile(r"comm=(.*?) pid=(\d+) child_comm=(.*?) child_pid=(\d+)")
EXECV = re.compile(r"filename=(\S+) pid=(\d+)")
EXITV = re.compile(r"comm=(.*?) pid=(\d+)")
INST_LINE = re.compile(r"^\s*(?P<comm>.*?)-(?P<pid>\d+)\s+\[(?P<cpu>\d+)\]\s+(?:\S+\s+)?"
                       r"(?P<ts>\d+(?:\.\d+)?):\s+(?P<ev>\w+):")
KERNEL_COMM = re.compile(r"^(kworker/|ksoftirqd/|migration/|rcu|cpuhp/|irq/|idle_inject/|watchdog|"
                         r"kthreadd$|swapper|kcompactd|kswapd|khugepaged|oom_reaper|jbd2/|nvidia|"
                         r"card\d|scsi_|kblockd|writeback|ext4-|btrfs|kauditd|khungtaskd)")


class TimerEvents:
    """The timer mode's events of one trace: the real-time CPU's deadline
    writes, anchors, interrupts, NMIs, idle transitions, system calls and
    page faults and current task; every CPU's timer arming, cancellation
    and expiry, process lifecycle and per-task CPU time within the run's
    window."""

    def __init__(self, tid: int, rt_cpu: int, window: tuple[int, int] | None = None) -> None:
        self.tid, self.rt_cpu, self.window = tid, rt_cpu, window
        self.writes: list[tuple] = []     # (ts, value, failed)
        self.anchors: list[int] = []      # local_timer_entry on the real-time CPU
        self.irqs: list[tuple] = []       # (entry, exit) of device interrupts there
        self._irq_open: int | None = None
        self.idle: list[tuple] = []       # (ts, state): IDLE_EXIT on leaving idle
        self.idle_miss: list[tuple] = []  # (ts, state, type)
        self.nmis: list[tuple] = []       # (start, end, handler)
        self.syscalls: list[tuple] = []   # (ts, pid, entering)
        self.faults: list[tuple] = []     # (ts, pid)
        self.rt_switches: list[tuple] = []  # (ts, next pid, next comm)
        self.starts: list[tuple] = []     # (ts, cpu, pid, comm, addr, function, expires, soft, mode)
        self.cancels: list[tuple] = []    # (ts, cpu, pid, addr, in an interrupt)
        self.expiries: list[tuple] = []   # (ts, cpu, addr, now, function)
        self.clockid: dict[str, str] = {}  # hrtimer -> its clock, from hrtimer_init/setup
        self.forks: list[tuple] = []      # (ts, parent pid, parent comm, child pid, child comm)
        self.execs: list[tuple] = []      # (ts, pid, filename)
        self.exits: list[tuple] = []      # (ts, pid, comm)
        self.cpu_time: dict[int, int] = {}  # pid -> ns on a CPU inside the window
        self.comm: dict[int, str] = {}
        self._cur: dict[int, tuple] = {}  # cpu -> (pid, since)
        self.seen = set()

    def feed(self, ts: int, cpu: int, pid: int, comm: str, ev: str, rest: str, flags: str) -> None:
        rt = cpu == self.rt_cpu
        if ev == "write_msr":
            w = WRITE.match(rest)
            if rt and w and w[1] == MSR_TSC_DEADLINE:
                self.writes.append((ts, int(w[2], 16), bool(w[3])))
                self.seen.add(ev)
        elif ev == "local_timer_entry":
            if rt:
                self.anchors.append(ts)
        elif ev in ("cpu_idle", "cpu_idle_miss"):
            c, s = CPUID.search(rest), STATE.search(rest)
            if c and s and int(c[1]) == self.rt_cpu:
                self.seen.add(ev)
                if ev == "cpu_idle":
                    self.idle.append((ts, int(s[1])))
                else:
                    t = re.search(r"type=(\S+)", rest)
                    self.idle_miss.append((ts, int(s[1]), t[1] if t else "?"))
        elif ev == "nmi_handler":
            n = NMI.match(rest)
            if rt and n:
                self.nmis.append((ts - int(n[2]), ts, n[1]))
                self.seen.add(ev)
        elif ev in ("sys_enter", "sys_exit"):
            if rt:
                self.syscalls.append((ts, pid, ev == "sys_enter"))
                self.seen.add(ev)
        elif ev in ("page_fault_user", "page_fault_kernel"):
            if rt:
                self.faults.append((ts, pid))
        elif ev == "hrtimer_start":
            a, x = ADDR.search(rest), EXPIRES.search(rest)
            if a and x:
                f, s, m = FUNC.search(rest), SOFT.search(rest), MODE.search(rest)
                self.starts.append((ts, cpu, pid, comm, a[1], f[1] if f else "?", int(x[1]),
                                    int(s[1]) if s else int(x[1]), m[1] if m else "?"))
                self.seen.add(ev)
        elif ev == "hrtimer_cancel":
            a = ADDR.search(rest)
            if a:
                self.cancels.append((ts, cpu, pid, a[1], len(flags) > 2 and flags[2] in "hH"))
        elif ev == "hrtimer_expire_entry":
            a, n, f = ADDR.search(rest), NOW.search(rest), FUNC.search(rest)
            if a and n:
                self.expiries.append((ts, cpu, a[1], int(n[1]), f[1] if f else "?"))
        elif ev in ("hrtimer_init", "hrtimer_setup"):
            a, c = ADDR.search(rest), CLOCKID.search(rest)
            if a and c:
                self.clockid[a[1]] = c[1]
        elif ev == "sched_process_fork":
            f = FORK.search(rest)
            if f:
                self.forks.append((ts, int(f[2]), f[1], int(f[4]), f[3]))
                self.comm.setdefault(int(f[4]), f[3])
                self.seen.add(ev)
        elif ev == "sched_process_exec":
            e = EXECV.search(rest)
            if e:
                self.execs.append((ts, int(e[2]), e[1]))
                self.comm[int(e[2])] = pathlib.PurePosixPath(e[1]).name[:15]
        elif ev == "sched_process_exit":
            e = EXITV.search(rest)
            if e:
                self.exits.append((ts, int(e[2]), e[1]))
        elif ev == "irq_handler_entry":
            if rt:
                self._irq_open = ts
        elif ev == "irq_handler_exit":
            if rt and self._irq_open is not None:
                self.irqs.append((self._irq_open, ts))
                self._irq_open = None

    def switch(self, ts: int, cpu: int, prev_pid: int, prev_comm: str, next_pid: int, next_comm: str) -> None:
        """A sched_switch: the per-task CPU time inside the window, and the
        real-time CPU's current task."""
        cur = self._cur.get(cpu)
        if cur and cur[0] == prev_pid:
            self._account(prev_pid, cur[1], ts)
        if prev_pid:
            self.comm[prev_pid] = prev_comm
        if next_pid:
            self.comm[next_pid] = next_comm
        self._cur[cpu] = (next_pid, ts)
        if cpu == self.rt_cpu:
            self.rt_switches.append((ts, next_pid, next_comm))

    def _account(self, pid: int, start: int, end: int) -> None:
        if not pid:
            return
        if self.window:
            start, end = max(start, self.window[0]), min(end, self.window[1])
        if end > start:
            self.cpu_time[pid] = self.cpu_time.get(pid, 0) + end - start

    def finish(self, end: int) -> None:
        """Accounts the tasks still running at the trace's end."""
        for pid, since in self._cur.values():
            self._account(pid, since, end)
        self._cur.clear()

    def present(self) -> bool:
        return bool(self.writes or "cpu_idle" in self.seen or "sched_process_fork" in self.seen)


# --- The TSC to CLOCK_MONOTONIC conversion -------------------------------------------------


def load_instance(path: pathlib.Path, rt_cpu: int) -> tuple[list[int], list[str]]:
    """The x86-tsc instance's anchors (local_timer_entry on the real-time
    CPU, raw TSC counts) and its problems (a clock other than raw counts,
    overwritten entries)."""
    tscs, problems = [], []
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("#"):
            e = re.search(r"entries-in-buffer/entries-written:\s*(\d+)/(\d+)", line)
            if e and int(e[2]) > int(e[1]):
                problems.append(f"the TSC instance lost {int(e[2]) - int(e[1])} entries (overwritten)")
            continue
        m = INST_LINE.match(line)
        if not m or m["ev"] != "local_timer_entry" or int(m["cpu"]) != rt_cpu:
            continue
        if "." in m["ts"]:
            problems.append("the TSC instance's timestamps are not raw counts (its clock is not x86-tsc)")
            break
        tscs.append(int(m["ts"]))
    if any(b < a for a, b in zip(tscs, tscs[1:])):
        problems.append("the TSC instance's anchors go backwards")
    return tscs, problems


def load_pairs(path: pathlib.Path) -> list[tuple]:
    """A tsc_mono_pairs file: (mono before, tsc, mono after, cpu)."""
    pairs = []
    if not path.exists():
        return pairs
    for row in csv.reader(line for line in path.read_text().splitlines() if line and not line.startswith("#")):
        if len(row) >= 4 and row[0].isdigit():
            pairs.append((int(row[0]), int(row[1]), int(row[2]), int(row[3])))
    return pairs


def fit_rate(pairs: list[tuple]) -> tuple[float, float] | None:
    """mono ~ a + tsc / hz from the sampler's pairs (least squares on the
    brackets' midpoints): (a, ns per count). Used only for the anchors'
    interval pattern and their coarse alignment."""
    if len(pairs) < 2:
        return None
    xs = [p[1] for p in pairs]
    ys = [(p[0] + p[2]) / 2 for p in pairs]
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return None
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - slope * mx, slope


def match_anchors(mono: list[int], tsc: list[int], rate: tuple[float, float]) -> tuple[list[tuple], list[str]]:
    """Pairs the instance's anchors one-to-one with the main trace's, in
    order, by their interval pattern: the instance's first anchors' intervals
    (TSC counts times the sampler's slope) must match one place in the main
    trace, and every following interval must match the next main interval.
    A mismatch is a missing or duplicate anchor. The offset of the
    conversion is never used."""
    slope = rate[1]
    problems: list[str] = []
    if len(tsc) < 3 or len(mono) < 3:
        return [], ["fewer than three anchors in an instance"]

    def close(dm: float, dt: float) -> bool:
        return abs(dm - dt) <= 2_000 + 1e-4 * abs(dt)

    k = min(ANCHOR_CHECK, len(tsc) - 1)
    want = [(tsc[i + 1] - tsc[i]) * slope for i in range(k)]
    starts = [j for j in range(len(mono) - k)
              if all(close(mono[j + i + 1] - mono[j + i], want[i]) for i in range(k))]
    if len(starts) != 1:
        return [], [f"the anchors' interval pattern matched {len(starts)} places in the main trace (one required)"]
    j0 = starts[0]
    pairs = []
    for i in range(len(tsc)):
        j = j0 + i
        if j >= len(mono):
            problems.append(f"{len(tsc) - i} TSC anchors after the main trace's last")
            break
        if i and not close(mono[j] - mono[j - 1], (tsc[i] - tsc[i - 1]) * slope):
            problems.append(f"anchor {i}: the intervals disagree (a missing or duplicate anchor near "
                            f"monotonic {mono[j - 1]})")
            break
        pairs.append((tsc[i], mono[j]))
    return pairs, problems


class TscMap:
    """Linear interpolation between (tsc, mono) anchors; None outside them."""

    def __init__(self, pairs: list[tuple]) -> None:
        self.pairs = sorted(pairs)
        self.t = [p[0] for p in self.pairs]

    def span(self) -> tuple[int, int] | None:
        return (self.t[0], self.t[-1]) if self.t else None

    def mono(self, tsc: int) -> float | None:
        t = self.t
        if not t or tsc < t[0] or tsc > t[-1]:
            return None
        k = bisect.bisect_right(t, tsc)
        if k >= len(t):
            return float(self.pairs[-1][1])
        (t0, m0), (t1, m1) = self.pairs[k - 1], self.pairs[k]
        return m0 + (m1 - m0) * (tsc - t0) / (t1 - t0) if t1 != t0 else float(m0)


def validate(pairs: list[tuple], samples: list[tuple]) -> dict:
    """The conversion's independently validated uncertainty: held-out
    anchors (the odd ones predicted from the even ones) and the bracketed
    sampler pairs inside the anchors' span (residual plus half the
    bracket)."""
    full = TscMap(pairs)
    even = TscMap(pairs[0::2])
    held = [abs(even.mono(t) - m) for t, m in pairs[1::2] if even.mono(t) is not None]
    by_cpu: dict[int, list] = {}
    for m0, t, m1, cpu in samples:
        v = full.mono(t)
        if v is not None:
            by_cpu.setdefault(cpu, []).append((v - (m0 + m1) / 2, (m1 - m0) / 2))
    bounds = [abs(r) + h for rs in by_cpu.values() for r, h in rs]
    u = None
    if held and bounds:
        u = max(max(held), max(bounds)) + RES_NS
    return {"anchors": len(pairs), "held": held, "by_cpu": by_cpu, "bounds": bounds, "u": u}


# --- Timers and deadlines ----------------------------------------------------------------


def domain(ev: TimerEvents, addr: str, expires: int) -> str:
    """A timer's clock: from its hrtimer_init/setup, else by magnitude (an
    expiry past 1e18 ns is on a calendar clock)."""
    c = ev.clockid.get(addr)
    if c:
        return c
    return "CLOCK_REALTIME?" if expires > 10**18 else "CLOCK_MONOTONIC?"


def deadline_at(ev: TimerEvents, t: float) -> tuple | None:
    """The deadline write in effect at t (the last before it)."""
    k = bisect.bisect_left([w[0] for w in ev.writes], t) - 1
    return ev.writes[k] if k >= 0 else None


def state_at(ev: TimerEvents, t: float) -> tuple:
    """The real-time CPU at t: (idle state or None, its entry time, task
    pid, task comm)."""
    idle_ts = [i[0] for i in ev.idle]
    k = bisect.bisect_right(idle_ts, t) - 1
    idle = ev.idle[k] if k >= 0 and ev.idle[k][1] != IDLE_EXIT else None
    sw_ts = [s[0] for s in ev.rt_switches]
    j = bisect.bisect_right(sw_ts, t) - 1
    task = ev.rt_switches[j] if j >= 0 else (None, None, "?")
    return (idle[1] if idle else None, idle[0] if idle else None, task[1], task[2])


def in_kernel(ev: TimerEvents, pid: int | None, t: float) -> bool | None:
    """Whether the real-time CPU's task was inside a system call at t: its
    last system call event there before t, or else its first after t (a call
    entered on another CPU exits here); None if it has none."""
    calls = [c for c in ev.syscalls if c[1] == pid]
    k = bisect.bisect_right([c[0] for c in calls], t) - 1
    if k >= 0:
        return calls[k][2]
    if calls:
        return not calls[0][2]
    return None


def overlapping(spans: list[tuple], a: float, b: float) -> list[tuple]:
    return [s for s in spans if s[0] <= b and s[1] >= a]


def classify(ev: TimerEvents, conv: TscMap, u: float | None, arm: tuple, expiry: tuple,
             smi: list[tuple]) -> dict:
    """One worker sleep: arm (ts, expires), expiry (entry ts, now). The
    class of a late service, with its evidence."""
    arm_ts, e = arm
    entry, now = expiry
    late = now - e
    out = {"late": late, "e": e}
    anchors_before = [a for a in ev.anchors if a <= entry]
    irq_entry = anchors_before[-1] if anchors_before else entry
    out["irq_entry"] = irq_entry
    if late < LATE_NS:
        out["class"] = "on time"
        return out
    if arm_ts > e:
        out["class"] = "late sleep call"
        out["detail"] = f"armed {(arm_ts - e) / 1e3:.1f} us after its expiry"
        return out
    w = deadline_at(ev, e)
    if w is None or u is None or u > U_LIMIT_NS:
        out["class"] = "INDETERMINATE"
        out["detail"] = "no deadline write before the expiry" if w is None else "conversion not validated"
        return out
    if w[1] == 0:
        out["class"] = "deadline late"
        out["detail"] = f"the device was disarmed (written {(e - w[0]) / 1e3:.1f} us before the expiry)"
        return out
    d = conv.mono(w[1])
    if d is None:
        out["class"] = "INDETERMINATE"
        out["detail"] = "the deadline lies outside the anchors (no extrapolation)"
        return out
    out["deadline"] = d
    if d - e > u:
        out["class"] = "deadline late"
        out["detail"] = f"the deadline in effect was {(d - e) / 1e3:+.1f} us from the expiry (U {u / 1e3:.1f} us)"
        return out
    due = max(d, e) if d >= e - u else d
    idle, idle_since, pid, comm = state_at(ev, due)
    others = [a for a in ev.anchors if due + u < a < irq_entry]
    nmis = overlapping(ev.nmis, due, irq_entry)
    irqs = overlapping(ev.irqs, due, irq_entry)
    smis = [s for s in smi if s[0] <= irq_entry and s[1] >= due and s[2] > 0]
    ev_detail = []
    if others:
        ev_detail.append(f"{len(others)} timer interrupts in between")
    if nmis:
        ev_detail.append(f"{len(nmis)} NMIs ({', '.join(sorted({n[2] for n in nmis}))})")
    if irqs:
        ev_detail.append(f"{len(irqs)} device interrupts")
    if smis:
        ev_detail.append(f"SMIs in {len(smis)} overlapping count intervals")
    if idle is not None:
        out["class"] = "interrupt late from idle"
        out["detail"] = (f"deadline {(d - e) / 1e3:+.1f} us, interrupt {(irq_entry - due) / 1e3:.1f} us after it; "
                         f"idle state {idle} since {(due - idle_since) / 1e3:.1f} us before")
    else:
        k = in_kernel(ev, pid, due)
        if KERNEL_COMM.match(comm or ""):
            mode = "a kernel thread"
        else:
            mode = "in a system call" if k else ("in user mode or a fault" if k is False else "mode unknown")
        out["class"] = "interrupt late while busy"
        out["detail"] = (f"deadline {(d - e) / 1e3:+.1f} us, interrupt {(irq_entry - due) / 1e3:.1f} us after it; "
                         f"{comm} ({pid}) running, {mode}")
    if ev_detail:
        out["detail"] += "; " + ", ".join(ev_detail)
    return out


def attribute(ev: TimerEvents, conv: TscMap, u: float, rt_offset: int | None) -> list[tuple]:
    """Each deadline write: (ts, converted deadline or None, attribution).
    The attribution is the one pending timer whose expiry (in monotonic
    time) is within U of the deadline, else INDETERMINATE (none, several,
    or a clock that cannot be converted)."""
    events = sorted([(s[0], 0, s) for s in ev.starts] + [(c[0], 1, c) for c in ev.cancels] +
                    [(x[0], 2, x) for x in ev.expiries])
    pending: dict[str, tuple] = {}
    k = 0
    out = []
    for ts, value, _ in ev.writes:
        while k < len(events) and events[k][0] <= ts:
            _, kind, e = events[k]
            if kind == 0:
                pending[e[4]] = e
            else:
                pending.pop(e[3] if kind == 1 else e[2], None)
            k += 1
        if value == 0:
            out.append((ts, None, "disarmed"))
            continue
        d = conv.mono(value)
        if d is None:
            out.append((ts, None, "INDETERMINATE (outside the anchors)"))
            continue
        cands = []
        for s in pending.values():
            dom = domain(ev, s[4], s[6])
            x = s[6]
            if "REALTIME" in dom or "TAI" in dom:
                if rt_offset is None:
                    continue
                x -= rt_offset
            elif "BOOTTIME" in dom:
                continue
            if abs(x - d) <= u:
                cands.append(s)
        if len(cands) == 1:
            s = cands[0]
            out.append((ts, d, f"{s[5]} of {s[3]} ({s[2]})"))
        elif not cands:
            out.append((ts, d, "INDETERMINATE (no pending timer at that expiry)"))
        else:
            out.append((ts, d, f"INDETERMINATE ({len(cands)} pending timers at that expiry)"))
    return out


# --- The rehearsal's exerciser ---------------------------------------------------------------


def load_exerciser(path: pathlib.Path) -> tuple[dict, list[dict]]:
    head: dict = {}
    rows = []
    for line in path.read_text().splitlines():
        if line.startswith("#"):
            head.update(dict(kv.split("=", 1) for kv in line[1:].split() if "=" in kv))
        elif line and not line.startswith("round"):
            r = line.split(",")
            rows.append({"round": int(r[0]), "kind": r[1], "action": r[2], "before": int(r[3]),
                         "after": int(r[4]), "expiry": int(r[5])})
    return head, rows


def explain_exerciser(ev: TimerEvents, conv: TscMap, u: float, head: dict, rows: list[dict]) -> dict:
    """Each exerciser action: its trace events (an arm one hrtimer_start of
    timerfd_tmrproc, a re-arm a cancellation then a start, a cancellation
    one hrtimer_cancel unless the timer had expired), its timer's
    destination (shown by an expiry or a matching write), and whether the
    deadline in effect after it covers the earliest known pending expiry on
    the real-time CPU. OK, legitimate no-write, INDETERMINATE or
    UNEXPLAINED."""
    tid = int(head.get("tid", -1))
    starts = [s for s in ev.starts if s[2] == tid and s[5] == "timerfd_tmrproc"]
    addrs = {s[4] for s in starts}
    cancels = [c for c in ev.cancels if c[2] == tid and c[3] in addrs]
    expiries = [x for x in ev.expiries if x[2] in addrs]
    result = {"OK": 0, "no write needed": 0, "INDETERMINATE": 0, "UNEXPLAINED": 0}
    notes = []

    def within(lst, r):
        return [x for x in lst if r["before"] <= x[0] <= r["after"]]

    def destination(start: tuple) -> int | None:
        """An arming's CPU: its expiry's (before the timer's next arming), or
        the real-time CPU if a deadline matching its expiry was written there
        during the arming call (within 50 us of its start event)."""
        nxt = min([s[0] for s in starts if s[4] == start[4] and s[0] > start[0]] + [float("inf")])
        exp = [x for x in expiries if x[2] == start[4] and start[0] < x[0] < nxt]
        if exp:
            return exp[0][1]
        for w in ev.writes:
            if start[0] <= w[0] <= start[0] + 50_000 and w[1]:
                d = conv.mono(w[1])
                if d is not None and abs(d - start[6]) <= u:
                    return ev.rt_cpu
        return None

    rows = [r for r in rows if r["action"] != "expired"]  # the exerciser's own observations
    for r in rows:
        st, cn = within(starts, r), within(cancels, r)
        if r["action"] == "cancel":
            if len(st) or len(cn) > 1:
                result["UNEXPLAINED"] += 1
                notes.append(f"round {r['round']} cancel: {len(st)} starts, {len(cn)} cancellations")
                continue
            if not cn:
                gone = [x for x in expiries if x[0] < r["before"]]
                if gone:
                    result["OK"] += 1  # the timer had expired before the cancellation
                    continue
                result["UNEXPLAINED"] += 1
                notes.append(f"round {r['round']} cancel: no cancellation and no earlier expiry")
                continue
            ts, addr = cn[0][0], cn[0][3]
        else:
            if len(st) != 1 or (r["action"] == "arm" and cn) or (r["action"] == "rearm" and len(cn) > 1):
                result["UNEXPLAINED"] += 1
                notes.append(f"round {r['round']} {r['action']}: {len(st)} starts, {len(cn)} cancellations")
                continue
            if st[0][6] != r["expiry"]:
                result["UNEXPLAINED"] += 1
                notes.append(f"round {r['round']} {r['action']}: armed for {st[0][6]}, not {r['expiry']}")
                continue
            ts, addr = st[0][0], st[0][4]
        # The destination: that of the arming (this one, or for a
        # cancellation the one it cancels).
        if r["action"] == "cancel":
            armed = [s for s in starts if s[4] == addr and s[0] < ts]
            dest = destination(armed[-1]) if armed else None
        else:
            dest = destination(st[0])
        after = deadline_at(ev, r["after"] + 1)
        before = deadline_at(ev, ts)
        if dest is None or dest != ev.rt_cpu:
            result["INDETERMINATE"] += 1
            if dest is not None:
                notes.append(f"round {r['round']} {r['action']}: the timer expired on CPU {dest}")
            continue
        if after is None or (after[1] and conv.mono(after[1]) is None):
            result["INDETERMINATE"] += 1
            continue
        d_after = conv.mono(after[1]) if after[1] else None
        if r["action"] == "cancel":
            # Removing a timer may leave the device armed for it (a spurious
            # interrupt follows) or reprogram it later: either is explained
            # unless the device was disarmed while timers stayed pending.
            result["OK" if d_after is not None else "UNEXPLAINED"] += 1
            if d_after is None:
                notes.append(f"round {r['round']} cancel: the device was disarmed")
            continue
        if d_after is not None and d_after <= r["expiry"] + u:
            wrote = before is None or after[0] != before[0]
            result["OK" if wrote else "no write needed"] += 1
        else:
            result["UNEXPLAINED"] += 1
            where = "disarmed" if d_after is None else f"{(d_after - r['expiry']) / 1e3:+.1f} us from its expiry"
            notes.append(f"round {r['round']} {r['action']}: the deadline after it is {where}")
    return {**result, "notes": notes, "actions": len(rows)}


# --- Background load -------------------------------------------------------------------------


def load_procs(path: pathlib.Path) -> dict[int, tuple]:
    """A `ps -eo pid,ppid,comm` listing: pid -> (ppid, comm)."""
    procs = {}
    if path.exists():
        for line in path.read_text(errors="replace").splitlines()[1:]:
            f = line.split(None, 2)
            if len(f) == 3 and f[0].isdigit() and f[1].isdigit():
                procs[int(f[0])] = (int(f[1]), f[2].strip())
    return procs


def background(ev: TimerEvents, session_pid: int | None, procs: dict[int, tuple], batch: set[str]) -> dict:
    """The CPU time inside the run's window of the tasks outside the
    session (its descendants, from the listing and the trace's forks), the
    kernel's threads and the idle task, by command; the batch workloads
    among them, and the tasks that started or exited during the run."""
    parent = {pid: pp for pid, (pp, _) in procs.items()}
    for _, pp, _, child, _ in ev.forks:
        parent[child] = pp
    comm = dict(ev.comm)
    for pid, (_, c) in procs.items():
        comm.setdefault(pid, c)

    def session(pid: int) -> bool:
        seen = 0
        while pid and seen < 64:
            if pid == session_pid:
                return True
            pid = parent.get(pid, 0)
            seen += 1
        return False

    def kernel(pid: int) -> bool:
        return pid == 0 or parent.get(pid) == 2 or pid == 2 or bool(KERNEL_COMM.match(comm.get(pid, "")))

    by_comm: dict[str, list] = {}
    for pid, ns in ev.cpu_time.items():
        if kernel(pid) or (session_pid and session(pid)) or pid == ev.tid:
            continue
        c = comm.get(pid, "?")
        e = by_comm.setdefault(c, [0, 0])
        e[0] += ns
        e[1] += 1
    started = {c for _, _, _, child, c in ev.forks if not kernel(child) and not session(child)}
    started |= {pathlib.PurePosixPath(f).name[:15] for _, pid, f in ev.execs if not session(pid)}
    found = sorted(c for c in by_comm if c in batch) + sorted(c for c in started if c in batch and c not in by_comm)
    return {"by_comm": by_comm, "batch": sorted(set(found)), "started": started}


# --- SMI counts --------------------------------------------------------------------------------


def load_smi(path: pathlib.Path, anchors: tuple[int, int] | None) -> list[tuple]:
    """perf stat -I intervals: (start, end, count) in monotonic time. perf's
    times are relative to its own start s, which lies between the anchor
    taken before it started and the time its first interval (at s + t1) was
    seen, minus t1; each interval is widened to cover both ends."""
    rows = []
    if anchors is None or not path.exists():
        return rows
    for line in path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        f = line.split(",")
        try:
            rows.append((float(f[0]), int(float(f[1]))))
        except (ValueError, IndexError):
            continue
    if not rows:
        return rows
    before, seen = anchors
    lo, hi = before, max(before, seen - int(rows[0][0] * 1e9))
    out, prev = [], 0.0
    for t, n in rows:
        out.append((lo + int(prev * 1e9), hi + int(t * 1e9), n))
        prev = t
    return out


# --- The report --------------------------------------------------------------------------------


def pct(xs: list[float], q: float) -> float:
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))] if s else float("nan")


def record_value(record: list[str], key: str) -> str | None:
    for line in record:
        if line.startswith(key):
            return line[len(key):].strip()
    return None


def report(csv_path: pathlib.Path, ev: TimerEvents, pairs_of_sleeps: list[tuple], record: list[str],
           failures: list, top: int, out) -> None:
    """The timer service section of trace_correlate's report."""
    mode_timer = any(line.strip() == "mode: timer" for line in record)
    if not ev.present():
        if mode_timer:
            failures.append("timer service: a timer-mode run without its deadline, idle or lifecycle events")
            print("  timer service: no timer-mode events, though the run is in the timer mode", file=out)
        return
    print("  timer service (the real-time CPU's deadlines, idle transitions and interrupts):", file=out)
    for key, what in (("trace overruns:", "the trace"), ("tsc instance overruns:", "the TSC instance")):
        over = record_value(record, key)
        if over is None:
            failures.append(f"timer service: the record has no count of {what}'s losses")
        elif over.split()[0] != "0":
            failures.append(f"timer service: {what} lost events ({over})")
        print(f"    {key} {over}", file=out)
    # The conversion.
    inst = pathlib.Path(str(csv_path) + ".tsc.txt")
    samp = load_pairs(pathlib.Path(str(csv_path) + ".tsc-pairs.csv"))
    rate = fit_rate(samp)
    u = None
    conv = TscMap([])
    if not inst.exists() or rate is None:
        failures.append("timer service: no TSC instance or sampler pairs")
    else:
        tsc, problems = load_instance(inst, ev.rt_cpu)
        pairs, mp = match_anchors(ev.anchors, tsc, rate)
        problems += mp
        for p in problems:
            failures.append(f"timer service: {p}")
        conv = TscMap(pairs)
        v = validate(pairs, samp)
        u = v["u"]
        print(f"    anchors: {len(tsc)} in the TSC instance, {len(ev.anchors)} in the main trace, {len(pairs)} "
              f"matched one-to-one; TSC {1e3 / rate[1]:.4f} MHz from {len(samp)} sampler pairs", file=out)
        held = v["held"]
        if held:
            print(f"    held-out anchors: n={len(held)} p50 {pct(held, .5):.0f} p99 {pct(held, .99):.0f} "
                  f"max {max(held):.0f} ns", file=out)
        for cpu, rs in sorted(v["by_cpu"].items()):
            res = [r for r, _ in rs]
            print(f"    sampler on CPU {cpu}: n={len(rs)} residual median {statistics.median(res):+.0f} ns, "
                  f"|residual| + half bracket max {max(abs(r) + h for r, h in rs):.0f} ns", file=out)
        if u is None:
            failures.append("timer service: the conversion's uncertainty is not validated (no held-out anchors "
                            "or sampler pairs inside the anchors)")
            print("    conversion uncertainty: not validated", file=out)
        else:
            print(f"    conversion uncertainty U: {u:.0f} ns (limit {U_LIMIT_NS} ns)", file=out)
            if u > U_LIMIT_NS:
                failures.append(f"timer service: the conversion's uncertainty {u:.0f} ns exceeds {U_LIMIT_NS} ns")
    # The worker's sleeps.
    offset = None
    for line in record:
        m = re.search(r"realtime_minus_monotonic_ns (-?\d+)", line)
        if m:
            offset = int(m[1])
            break
    anchor = record_value(record, "smi anchor monotonic_ns:")
    smi_anchors = None
    parts = anchor.split() if anchor else []
    if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit():
        smi_anchors = (int(parts[0]), int(parts[1]))
    smi = load_smi(pathlib.Path(str(csv_path) + ".smi.csv"), smi_anchors)
    arms = sorted((s[0], s[4], s[6]) for s in ev.starts if s[2] == ev.tid and s[5] == "hrtimer_wakeup")
    arm_ts = [a[0] for a in arms]
    results = []
    for (entry, now, _, _, _, addr), (i, prev_end, sched, _) in pairs_of_sleeps:
        a0, a1 = bisect.bisect_right(arm_ts, prev_end), bisect.bisect_right(arm_ts, entry)
        mine = [a for a in arms[a0:a1] if a[1] == addr]
        if not mine:
            results.append({"class": "INDETERMINATE", "late": now - sched, "detail": "no arming recorded", "i": i})
            continue
        r = classify(ev, conv, u, (mine[-1][0], mine[-1][2]), (entry, now), smi)
        r["i"] = i
        results.append(r)
    counts: dict[str, int] = {}
    for r in results:
        counts[r["class"]] = counts.get(r["class"], 0) + 1
    print(f"    sleeps: {len(results)}; " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())), file=out)
    late = sorted((r for r in results if r["class"] != "on time"), key=lambda r: -r["late"])
    for r in late[:top]:
        print(f"      sleep before cycle {r['i']}: served {r['late'] / 1e3:.1f} us late: {r['class']}"
              + (f" ({r['detail']})" if r.get("detail") else ""), file=out)
    # The deadline writes and their attribution.
    if u is not None and u <= U_LIMIT_NS:
        att = attribute(ev, conv, u, offset)
        ind = sum(1 for a in att if a[2].startswith("INDETERMINATE"))
        print(f"    deadline writes: {len(att)}, {sum(1 for a in att if a[2] == 'disarmed')} disarming, "
              f"{ind} INDETERMINATE in attribution", file=out)
    # Idle states and SMIs.
    states: dict[int, int] = {}
    for _, s in ev.idle:
        if s != IDLE_EXIT:
            states[s] = states.get(s, 0) + 1
    print(f"    idle entries by state: {dict(sorted(states.items()))}; idle misses {len(ev.idle_miss)}; "
          f"NMIs {len(ev.nmis)}; SMIs {sum(s[2] for s in smi) if smi else 'not counted'}", file=out)
    # The exerciser.
    exf = pathlib.Path(str(csv_path) + ".exerciser.csv")
    if exf.exists():
        if u is None or u > U_LIMIT_NS:
            failures.append("timer service: the exerciser cannot be checked without a validated conversion")
        else:
            head, rows = load_exerciser(exf)
            x = explain_exerciser(ev, conv, u, head, rows)
            print(f"    exerciser: {x['actions']} actions: OK {x['OK']}, no write needed {x['no write needed']}, "
                  f"INDETERMINATE {x['INDETERMINATE']}, UNEXPLAINED {x['UNEXPLAINED']}", file=out)
            for n in x["notes"][:top]:
                print(f"      {n}", file=out)
            if x["UNEXPLAINED"]:
                failures.append(f"timer service: {x['UNEXPLAINED']} exerciser actions unexplained")
    # Background load.
    batch = set((record_value(record, "batch workloads:") or "").split(",")) - {""}
    sp = record_value(record, "session pid:")
    bg = background(ev, int(sp) if sp and sp.isdigit() else None,
                    load_procs(pathlib.Path(str(csv_path) + ".procs.txt")), batch)
    topc = sorted(bg["by_comm"].items(), key=lambda kv: -kv[1][0])[:6]
    print("    foreign CPU time in the run: " + (", ".join(f"{c} {v[0] / 1e6:.0f} ms" for c, v in topc) or "none"),
          file=out)
    if bg["batch"]:
        print(f"    UNCONTROLLED background load: {', '.join(bg['batch'])} ran", file=out)
