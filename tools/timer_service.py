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
clock. tools/timer_clock.py pairs them by the deadline written before each
interrupt, which is in TSC units and needs no clock mapping; a bracketed
TSC and CLOCK_MONOTONIC sampler (tsc_mono_pairs) checks the assignment. A
deadline is converted by linear interpolation between the retained anchors
around it, never extrapolated. U is a bound on the error of every converted
value, from each anchor's measured stamp difference, the sampler's bound, the
chord over the conversion's interval and the clock-read difference. It holds
under a clock model that tools/timer_clock.py certifies from the acquisition's
timekeeping record (tools/tk_record.py); without a certificate the run fails,
and a U computed under an exploratory model is labelled as such. The
classification uses U in its boundaries, and a U above 5 us fails the run. The
residuals of held-out anchors and of the sampler are printed as diagnostics.

The deadline in effect is followed as a timeline: a write arms the device
for its deadline (0 disarms it; a failed write leaves it unknown), and a
TSC deadline disarms itself when it fires, so the first local timer
interrupt at or after it serves it. A sleep whose timer was served 50 us or
more late (the expiry's now= minus the timer's expiry, both monotonic) is
described by its recorded evidence, not by a cause: a late sleep call (the
call entered after the expiry); a timer armed late (the call entered before
the expiry, its timer armed after it); a deadline late (the deadline that
raised the serving interrupt was due, at the later of its value and its
write, more than U after the expiry); an interrupt late from idle or while
busy (that deadline on time); and INDETERMINATE when the deadline cannot be
converted, the state at the expiry is unknown, or the serving interrupt
follows no armed deadline or may be an overwritten deadline's pending one.
A write's time is its tracepoint's, which follows the WRMSR instruction. A
late service is a result, not a failure.

Each timer arming is followed to its end (its cancellation, expiry or next
arming), and its residency (the CPU it is queued on) is taken only from the
trace's evidence: its expiry's CPU, or a pinned arming's CPU (a matching
write after an arming is a hint only). A deadline write is attributed to the one pending
timer resident on the real-time CPU at its value; a matching timer of
unknown residency leaves it INDETERMINATE. A rehearsal's exerciser
(tools/timer_exerciser.py) arms, replaces and cancels timerfd timers (its
"expired" rows are its own observations, not actions); each action must
match its timer's lifetime, and while the arming is pending the device must
cover it (a missing write is legitimate when an earlier deadline already
covers it). An arming of unknown or foreign residency is INDETERMINATE; an
action its events or the device's state do not explain fails.

A timer-mode run must hold the evidence the mode records (report()); its
absence fails the run.
"""

from __future__ import annotations

import bisect
import pathlib
import re
import statistics
from typing import NamedTuple

import timer_clock
import tk_record

ANALYSIS_REVISION = "clock alignment revision 53"
U_LIMIT_NS = int(timer_clock.U_LIMIT)  # the provisional bound on the validated uncertainty
LATE_NS = 50_000         # a timer service this late is classified
RES_NS = 1               # timestamp resolution added to U (mono 1 ns; the TSC finer)
MSR_TSC_DEADLINE = "6e0"
IDLE_EXIT = 4294967295   # power:cpu_idle's state on leaving idle
TIMER_EVENTS = {"write_msr", "local_timer_entry", "local_timer_exit", "cpu_idle", "cpu_idle_miss", "nmi_handler",
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
        self.ledger = timer_clock.Ledger()  # the real-time CPU's deadline writes, entries and exits, strictly

    def unparsed(self, line: str) -> None:
        """A report row that is not in the event format. Other events' odd rows are not this analysis's evidence,
        but a record of the ledger's events that cannot be read must not be lost silently."""
        if any(name in line for name in timer_clock.KIND):
            self.ledger.problems.append(f"an unreadable row of the report: {line.strip()[:120]}")

    def feed(self, ts: int, cpu: int, pid: int, comm: str, ev: str, rest: str, flags: str) -> None:
        rt = cpu == self.rt_cpu
        if rt and ev in timer_clock.KIND:
            self.ledger.feed(ts, ev, rest)
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


# --- The TSC to CLOCK_MONOTONIC conversion -------------------------------------------------


def fit_rate(pairs: list[tuple]) -> tuple[float, float] | None:
    """mono ~ a + tsc / hz from the sampler's pairs (least squares on the
    brackets' midpoints): (a, ns per count), a figure the report prints."""
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
    """The conversion's residuals, a diagnostic and not a bound: held-out
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


# --- The clock event device ------------------------------------------------------------------


class Device(NamedTuple):
    """The real-time CPU's TSC deadline timer at some time."""

    kind: str                  # "unknown", "disarmed", "armed" or "served"
    write: tuple | None        # the deadline write that set it
    d: float | None = None     # its deadline in monotonic time (None outside the anchors)
    served: int | None = None  # the local timer interrupt that served it
    why: str = ""              # why the state is unknown
    pending: tuple | None = None  # (write, deadline) of an earlier generation whose interrupt may be pending


BEFORE_WRITES = Device("unknown", None, why="before the first deadline write")


class Timeline:
    """The real-time CPU's clock event device, from its deadline writes and
    local timer interrupts. A write of 0 disarms it; a failed write leaves
    its state unknown; any other write arms it for that deadline. A TSC
    deadline disarms itself when it fires (Intel SDM vol. 3, the TSC-deadline
    mode of the local APIC timer): the first local timer interrupt at or
    after the deadline (less U) serves it, and the device stays unarmed until
    the next write. An interrupt before the armed deadline leaves it armed;
    one after a deadline outside the anchors leaves the state unknown, as is
    the state before the first write.

    Firing and service are separate observations: a deadline may fire while
    interrupts are masked, its interrupt pending. So a write that replaces
    an armed deadline already due (within U of the write's tracepoint, which
    follows the WRMSR instruction), one outside the anchors, or an unknown
    state, with no interrupt seen since, carries that generation as possibly
    pending; the next local timer interrupt may be its, and clears it."""

    def __init__(self, ev: TimerEvents, conv: TscMap, u: float | None) -> None:
        self.conv, self.u = conv, u
        marks = sorted([(a, 0, None) for a in ev.anchors] + [(w[0], 1, w) for w in ev.writes],
                       key=lambda m: (m[0], m[1]))
        st = BEFORE_WRITES
        self.ts: list[int] = []
        self.states: list[Device] = []
        for ts, kind, w in marks:
            if kind == 1:
                pend = st.pending  # the most recent candidate is named
                if st.kind == "armed" and (st.d is None or u is None or st.d - u <= ts):
                    pend = (st.write, st.d)  # it may have fired, its interrupt not yet seen
                elif pend is None and st.kind == "unknown" and st.served is None:
                    pend = (st.write, None)
                if w[2]:
                    st = Device("unknown", w, why="a failed deadline write", pending=pend)
                elif w[1] == 0:
                    st = Device("disarmed", w, pending=pend)
                else:
                    st = Device("armed", w, conv.mono(w[1]), pending=pend)
            elif st.kind == "armed" and st.d is None:
                st = Device("unknown", st.write, served=ts, why="an interrupt after a deadline outside the anchors")
            elif st.kind == "armed" and u is not None and ts >= st.d - u:
                st = Device("served", st.write, st.d, ts)
            elif st.pending is not None:
                st = st._replace(pending=None)  # the pending interrupt, if any, was this one
            else:
                continue
            self.ts.append(ts)
            self.states.append(st)

    def at(self, t: float) -> Device:
        """The device just before t (the events at t excluded)."""
        k = bisect.bisect_left(self.ts, t) - 1
        return self.states[k] if k >= 0 else BEFORE_WRITES


def describe(s: Device, e: float) -> str:
    """The device's state, its times relative to e. A write's time is its
    tracepoint's, which follows the WRMSR instruction."""
    if s.kind == "armed":
        what = "a deadline outside the anchors" if s.d is None else f"{(s.d - e) / 1e3:+.1f} us"
        return f"armed for {what} (its write traced at {(s.write[0] - e) / 1e3:+.1f} us)"
    if s.kind == "served":
        return (f"not armed: its deadline ({(s.d - e) / 1e3:+.1f} us) was served by the interrupt at "
                f"{(s.served - e) / 1e3:+.1f} us")
    if s.kind == "disarmed":
        return f"disarmed (its write traced at {(s.write[0] - e) / 1e3:+.1f} us)"
    return f"in an unknown state ({s.why})"


def describe_pending(s: Device, e: float) -> str:
    """The earlier generation whose interrupt may be pending in s."""
    w, d = s.pending
    if w is None:
        return "the device's state before the trace's first deadline write is unknown"
    if d is None:
        return (f"an earlier deadline (its write traced at {(w[0] - e) / 1e3:+.1f} us, its state unknown or outside "
                "the anchors) was overwritten with no interrupt seen after it")
    return (f"an earlier deadline ({(d - e) / 1e3:+.1f} us, its write traced at {(w[0] - e) / 1e3:+.1f} us) was due "
            "when it was overwritten, with no interrupt seen after it")


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


def classify(ev: TimerEvents, tl: Timeline, arm: tuple, expiry: tuple, smi: list[tuple],
             call: int | None = None) -> dict:
    """One worker sleep: arm (the arming's ts, its expiry E), expiry (the
    expiry's entry ts, its now=), call (its sleep call's entry, if
    recorded). The class of a late service, with its evidence. The
    interrupt that served the expiry (the last local timer interrupt before
    it) was raised by the deadline armed just before it, which was due at
    the later of its value and its write."""
    arm_ts, e = arm
    entry, now = expiry
    u = tl.u
    late = now - e
    out = {"late": late, "e": e}
    k = bisect.bisect_right(ev.anchors, entry) - 1
    irq_entry = ev.anchors[k] if k >= 0 else None
    out["irq_entry"] = irq_entry if irq_entry is not None else entry
    if late < LATE_NS:
        out["class"] = "on time"
        return out
    if call is not None and call > e:
        out["class"] = "late sleep call"
        out["detail"] = f"its sleep call entered {(call - e) / 1e3:.1f} us after its expiry"
        return out
    if arm_ts > e:
        out["class"] = "timer armed late"
        out["detail"] = ((f"its sleep call entered {(e - call) / 1e3:.1f} us before its expiry, " if call is not None
                          else "its sleep call's entry is not recorded; ")
                         + f"its timer was armed {(arm_ts - e) / 1e3:.1f} us after it")
        return out
    if u is None or u > U_LIMIT_NS:
        out["class"] = "INDETERMINATE"
        out["detail"] = "conversion not validated"
        return out
    if irq_entry is None:
        out["class"] = "INDETERMINATE"
        out["detail"] = "no local timer interrupt before the expiry"
        return out
    s = tl.at(irq_entry)
    if s.kind != "armed" or s.d is None:
        out["class"] = "INDETERMINATE"
        out["detail"] = f"the interrupt that served it followed no convertible armed deadline: the device was {describe(s, e)}"
        return out
    if s.pending is not None:
        out["class"] = "INDETERMINATE"
        out["detail"] = (f"the interrupt that served it may be an earlier deadline's pending one: "
                         f"{describe_pending(s, e)}; the trace cannot tell which raised it")
        return out
    d, w = s.d, s.write[0]
    if irq_entry < d - u:
        out["class"] = "INDETERMINATE"
        out["detail"] = (f"the interrupt that served it came {(d - irq_entry) / 1e3:.1f} us before the deadline armed "
                         "(an interrupt pending from before its write)")
        return out
    out["deadline"] = d
    due = max(d, w)
    if due - e > u:
        se = tl.at(e)
        if se.kind == "unknown":
            out["class"] = "INDETERMINATE"
            out["detail"] = f"the device was {describe(se, e)} at the expiry"
            return out
        out["class"] = "deadline late"
        if se.write == s.write:
            out["detail"] = f"the deadline in effect was {(d - e) / 1e3:+.1f} us from the expiry (U {u / 1e3:.1f} us)"
        else:
            out["detail"] = (f"at the expiry the device was {describe(se, e)}; the deadline that served it, "
                             f"{(d - e) / 1e3:+.1f} us from the expiry, had its write traced at {(w - e) / 1e3:+.1f} us "
                             f"(U {u / 1e3:.1f} us)")
        return out
    due = max(due, e) if d >= e - u else due
    idle, idle_since, pid, comm = state_at(ev, due)
    others = [a for a in ev.anchors if due + u < a < irq_entry]
    nmis = overlapping(ev.nmis, due, irq_entry)
    irqs = overlapping(ev.irqs, due, irq_entry)
    smis = [x for x in smi if x[0] <= irq_entry and x[1] >= due and x[2] > 0]
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


# --- Timer lifetimes ---------------------------------------------------------------------------


def mono_values(ev: TimerEvents, addr: str, x: int, offsets: dict) -> list[float] | None:
    """A timer's expiry in CLOCK_MONOTONIC: one value for a clock known from
    its hrtimer_init/setup, the values of both possible clocks for one
    inferred from the expiry's magnitude (past 1e18 ns: REALTIME or TAI;
    else MONOTONIC or BOOTTIME), None when an offset needed is not recorded.
    offsets: REALTIME, BOOTTIME and TAI minus MONOTONIC, in ns."""
    c = ev.clockid.get(addr)
    names = [c] if c else (["REALTIME", "TAI"] if x > 10**18 else ["MONOTONIC", "BOOTTIME"])
    out = []
    for n in names:
        if "MONOTONIC" in n:
            out.append(float(x))
            continue
        key = next((k for k in ("REALTIME", "BOOTTIME", "TAI") if k in n), None)
        if key is None or offsets.get(key) is None:
            return None
        out.append(float(x - offsets[key]))
    return out


class Lifetimes:
    """Every timer arming (an hrtimer_start) to its end: the timer's next
    cancellation, expiry or arming ("cancel", "expiry", "rearm"), or None
    while it stays pending to the trace's end. Each arming's residency, the
    CPU whose clock event device it is queued on, where the trace shows one:
    the CPU of its expiry, or the arming CPU of a pinned arming. Otherwise
    unknown: the CPU that recorded an unpinned arming is not its residency
    (the timer may be queued on another CPU). An arming on the real-time CPU
    followed there by a matching deadline write (no other timer event or
    local timer interrupt between, within 50 us) is marked with a hint for
    the reader, which leaves its residency unknown: the trace does not show
    that nothing else (a device interrupt, an NMI, another context) ran
    between them."""

    def __init__(self, ev: TimerEvents, conv: TscMap, u: float | None, offsets: dict) -> None:
        self.ev = ev
        by_addr: dict[str, list] = {}
        for x in ev.expiries:
            by_addr.setdefault(x[2], []).append((x[0], 0, "expiry", x))
        for c in ev.cancels:
            by_addr.setdefault(c[3], []).append((c[0], 1, "cancel", c))
        for s in ev.starts:
            by_addr.setdefault(s[4], []).append((s[0], 2, "start", s))
        rt = ev.rt_cpu
        seq = sorted([(a, 0, "anchor", None) for a in ev.anchors]
                     + [(x[0], 1, "expiry", x) for x in ev.expiries if x[1] == rt]
                     + [(c[0], 2, "cancel", c) for c in ev.cancels if c[1] == rt]
                     + [(s[0], 3, "start", s) for s in ev.starts if s[1] == rt]
                     + [(w[0], 4, "write", w) for w in ev.writes], key=lambda m: (m[0], m[1]))
        following = {id(seq[i][3]): seq[i + 1] for i in range(len(seq) - 1) if seq[i][2] == "start"}
        self.gens: list[dict] = []
        for addr, lst in by_addr.items():
            lst.sort(key=lambda m: (m[0], m[1]))
            for i, (_, _, kind, s) in enumerate(lst):
                if kind != "start":
                    continue
                nxt = lst[i + 1] if i + 1 < len(lst) else None
                g = {"start": s, "end": None, "end_ts": None, "end_cpu": None,
                     "x": mono_values(ev, addr, s[6], offsets)}
                if nxt:
                    g["end"] = "rearm" if nxt[2] == "start" else nxt[2]
                    g["end_ts"], g["end_cpu"] = nxt[0], nxt[3][1]
                g["cpu"], g["evidence"] = self._residency(g)
                g["hint"] = self._hint(g, following.get(id(s)), conv, u)
                self.gens.append(g)
        self.gens.sort(key=lambda g: g["start"][0])

    def _residency(self, g: dict) -> tuple:
        s = g["start"]
        if g["end"] == "expiry":
            return g["end_cpu"], "its expiry"
        if "PINNED" in s[8]:
            return s[1], "a pinned arming"
        return None, "unknown"

    def _hint(self, g: dict, nxt: tuple | None, conv: TscMap, u: float | None) -> bool:
        """A matching deadline write followed the arming on the real-time CPU
        (a hint only; never residency)."""
        s = g["start"]
        if g["cpu"] is not None or s[1] != self.ev.rt_cpu or u is None or not g["x"]:
            return False
        if not (nxt and nxt[2] == "write" and nxt[0] - s[0] <= 50_000 and nxt[3][1] and not nxt[3][2]):
            return False
        d = conv.mono(nxt[3][1])
        return d is not None and any(abs(d - x) <= u for x in g["x"])


def attribute(ev: TimerEvents, lt: Lifetimes, conv: TscMap, u: float) -> list[tuple]:
    """Each deadline write: (ts, converted deadline or None, attribution).
    The attribution is the one pending timer resident on the real-time CPU
    whose expiry (in monotonic time) is within U of the deadline. A timer
    resident on another CPU cannot be it; a matching timer of unknown
    residency, or a pending one whose clock cannot be converted, leaves the
    write INDETERMINATE, as do no match and several. Timers armed before
    the trace began are not seen."""
    rt = ev.rt_cpu
    marks = sorted([(g["start"][0], 0, i) for i, g in enumerate(lt.gens)]
                   + [(g["end_ts"], 1, i) for i, g in enumerate(lt.gens) if g["end_ts"] is not None])
    pending: set[int] = set()
    k = 0
    out = []
    for ts, value, failed in ev.writes:
        while k < len(marks) and marks[k][0] <= ts:
            _, kind, i = marks[k]
            if kind == 0:
                pending.add(i)
            else:
                pending.discard(i)
            k += 1
        if failed:
            out.append((ts, None, "INDETERMINATE (a failed write)"))
            continue
        if value == 0:
            out.append((ts, None, "disarmed"))
            continue
        d = conv.mono(value)
        if d is None:
            out.append((ts, None, "INDETERMINATE (outside the anchors)"))
            continue
        here, unknown, hinted, unconverted = [], 0, 0, 0
        for i in pending:
            g = lt.gens[i]
            if g["cpu"] is not None and g["cpu"] != rt:
                continue
            if g["x"] is None:
                unconverted += 1
            elif any(abs(x - d) <= u for x in g["x"]):
                if g["cpu"] == rt:
                    here.append(g)
                else:
                    unknown += 1
                    hinted += g["hint"]
        if unknown:
            hint = f"; {hinted} followed by a matching write on this CPU, a hint only" if hinted else ""
            out.append((ts, d, f"INDETERMINATE ({unknown} matching timers of unknown residency{hint})"))
        elif unconverted:
            out.append((ts, d, f"INDETERMINATE ({unconverted} pending timers whose clock cannot be converted)"))
        elif len(here) == 1:
            s = here[0]["start"]
            out.append((ts, d, f"{s[5]} of {s[3]} ({s[2]})"))
        elif not here:
            out.append((ts, d, "INDETERMINATE (no pending timer on this CPU at that expiry)"))
        else:
            out.append((ts, d, f"INDETERMINATE ({len(here)} pending timers at that expiry)"))
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


def explain_exerciser(ev: TimerEvents, tl: Timeline, lt: Lifetimes, head: dict, rows: list[dict]) -> dict:
    """Each exerciser action, followed through its timer's armings
    (Lifetimes). An arm is one hrtimer_start of timerfd_tmrproc for the
    requested expiry, no earlier arming pending; a re-arm is one, ending the
    round's pending arming by one cancellation in its call, or none if that
    arming had expired; a cancellation ends the pending arming of its expiry
    by one hrtimer_cancel in its call, or finds it expired (before or in the
    call). The arming's residency is its destination: another CPU, or
    unknown, is INDETERMINATE. The device must then cover the arming, at the
    call's end or at the arming's expiry if that came first: armed (or, at
    its expiry, served) for a deadline no later than its expiry plus U; OK
    when written after the arming, "no write needed" when an earlier write
    covers it. An arming made after its own expiry (its hrtimer_start at or
    after the expiry, less U) is counted apart, "armed after its expiry",
    when a successful deadline write after it and before the evaluation point
    has a deadline already due at that write (within U): the kernel's
    programming of an expired timer. After a cancellation the device must
    not be disarmed. Anything else is UNEXPLAINED; the exerciser's "expired"
    rows are its own observations, not actions."""
    u = tl.u
    rt = ev.rt_cpu
    tid = int(head.get("tid", -1))
    gens = [g for g in lt.gens if g["start"][2] == tid and g["start"][5] == "timerfd_tmrproc"]
    addrs = {g["start"][4] for g in gens}
    cancels = [c for c in ev.cancels if c[2] == tid and c[3] in addrs]
    result = {"OK": 0, "no write needed": 0, "armed after its expiry": 0, "INDETERMINATE": 0, "UNEXPLAINED": 0}
    why_indet: dict[str, int] = {}
    notes: list[str] = []
    write_ts = [w[0] for w in ev.writes]

    def indeterminate(why: str) -> None:
        result["INDETERMINATE"] += 1
        why_indet[why] = why_indet.get(why, 0) + 1

    def unexplained(r: dict, why: str) -> None:
        result["UNEXPLAINED"] += 1
        notes.append(f"round {r['round']} {r['action']}: {why}")

    def rel(t: float, r: dict) -> str:
        return f"{(t - r['before']) / 1e3:+.1f} us from the call"

    rows = [r for r in rows if r["action"] != "expired"]
    for r in rows:
        b, a = r["before"], r["after"]
        st = [g for g in gens if b <= g["start"][0] <= a]
        cn = [c for c in cancels if b <= c[0] <= a]
        if r["action"] == "cancel":
            prior = [g for g in gens if g["start"][0] < b]
            g = prior[-1] if prior else None
            if st or len(cn) > 1:
                unexplained(r, f"{len(st)} armings, {len(cn)} cancellations")
                continue
            if g is None or g["start"][6] != r["expiry"]:
                unexplained(r, "the last arming before it is not of its expiry")
                continue
            if not cn and g["end"] == "expiry" and g["end_ts"] <= a:
                result["OK"] += 1  # it had expired: there was nothing to cancel
                continue
            if not (cn and g["end"] == "cancel" and g["end_ts"] == cn[0][0]):
                unexplained(r, "its arming stayed pending through it" if g["end_ts"] is None or g["end_ts"] > a
                            else f"its arming ended by {g['end']} {rel(g['end_ts'], r)}")
                continue
            target, t_check = g, a + 1
        else:
            if len(st) != 1:
                unexplained(r, f"{len(st)} armings, {len(cn)} cancellations")
                continue
            g = st[0]
            if g["start"][6] != r["expiry"]:
                unexplained(r, f"armed for {g['start'][6]}, not {r['expiry']}")
                continue
            prev = [p for p in gens if p["start"][4] == g["start"][4] and p["start"][0] < g["start"][0]]
            p = prev[-1] if prev else None
            pending = p is not None and (p["end_ts"] is None or p["end_ts"] >= b)
            by_cancel = pending and p["end"] == "cancel" and p["end_ts"] <= g["start"][0]
            want = [p["end_ts"]] if by_cancel else []
            if r["action"] == "arm" and pending:
                unexplained(r, f"an earlier arming was pending (it ended by {p['end']})")
                continue
            if [c[0] for c in cn] != want or (pending and not by_cancel and p["end"] != "expiry"):
                unexplained(r, f"{len(cn)} cancellations; the arming before it "
                            + ("was pending" if pending else "had ended") + f" (it ended by {p['end'] if p else '-'})")
                continue
            target = g
            t_check = g["end_ts"] if g["end"] == "expiry" and g["end_ts"] <= a else a + 1
        if target["cpu"] is None:
            indeterminate("residency unknown, its events matched" + (" (a write hint)" if target["hint"] else ""))
            continue
        if target["cpu"] != rt:
            indeterminate("on another CPU")
            notes.append(f"round {r['round']} {r['action']}: its timer was on CPU {target['cpu']} "
                         f"({target['evidence']})")
            continue
        x = r["expiry"]
        if r["action"] != "cancel" and target["start"][0] >= x - u:
            # Armed after its own expiry: the kernel programs a deadline that
            # is already due. A successful write after the arming, before the
            # evaluation point, whose deadline is due at its write (within U)
            # shows it; a stale write or a future deadline does not.
            lo, hi = bisect.bisect_right(write_ts, target["start"][0]), bisect.bisect_left(write_ts, t_check)
            cands = [w for w in ev.writes[lo:hi] if w[1] and not w[2]]
            due = [w for w in cands if tl.conv.mono(w[1]) is not None and tl.conv.mono(w[1]) <= w[0] + u]
            if due:
                result["armed after its expiry"] += 1
            elif any(tl.conv.mono(w[1]) is None for w in cands):
                indeterminate("the device's state unknown")
            else:
                unexplained(r, f"armed {(target['start'][0] - x) / 1e3:+.1f} us from its expiry, with no deadline "
                            "write after it already due at its write")
            continue
        s = tl.at(t_check)
        if s.kind == "unknown" or (s.kind in ("armed", "served") and s.d is None):
            indeterminate("the device's state unknown")
            continue
        if r["action"] == "cancel":
            if s.kind == "disarmed":
                unexplained(r, "the device was disarmed after it")
            else:
                result["OK"] += 1
            continue
        at_expiry = t_check != a + 1
        if s.kind == "armed" or (s.kind == "served" and at_expiry):
            if s.d <= x + u:
                result["OK" if s.write[0] >= target["start"][0] else "no write needed"] += 1
            else:
                unexplained(r, f"the deadline covering it is {(s.d - x) / 1e3:+.1f} us from its expiry")
        else:
            unexplained(r, f"while it was pending the device was {describe(s, x)}")
    return {**result, "notes": notes, "actions": len(rows), "indeterminate": why_indet}


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


def load_smi(path: pathlib.Path, anchors: tuple[int, int] | None,
             window: tuple[int, int] | None = None) -> tuple[list[tuple], list[str]]:
    """perf stat -I intervals: (start, end, count) in monotonic time, and the
    series' problems. perf's times are relative to its own start s, which
    lies between the anchor taken before it started and the time its first
    interval (at s + t1) was seen, minus t1; each interval is widened to
    cover both ends. A count that is not a number (perf's "<not counted>"),
    times that do not advance, a gap over three median intervals and a
    series not covering the run's window are problems, as are a missing
    anchor or file."""
    if anchors is None:
        return [], ["the record has no valid SMI anchor"]
    if not path.exists():
        return [], ["no SMI count file"]
    rows, problems = [], []
    for line in path.read_text(errors="replace").splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        f = line.split(",")
        try:
            t = float(f[0])
        except (ValueError, IndexError):
            problems.append(f"an SMI count line without a time: {line[:60]}")
            continue
        n = f[1].strip() if len(f) > 1 else ""
        if not n.isdigit():
            problems.append(f"an SMI count that is not a number ({n or 'none'}) at {t:.3f} s")
            continue
        rows.append((t, int(n)))
    if not rows:
        return [], problems + ["no SMI counts"]
    before, seen = anchors
    if seen < before:
        problems.append("the SMI anchor's times go backwards")
    steps = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    if any(d <= 0 for d in steps):
        problems.append("the SMI intervals' times do not advance")
    elif steps and max(steps) > 3 * statistics.median(steps):
        problems.append(f"a gap of {max(steps):.3f} s in the SMI counts")
    lo, hi = before, max(before, seen - int(rows[0][0] * 1e9))
    out, prev = [], 0.0
    for t, n in rows:
        out.append((lo + int(prev * 1e9), hi + int(t * 1e9), n))
        prev = t
    if window and (out[0][0] > window[0] or out[-1][1] < window[1]):
        problems.append("the SMI counts do not cover the run")
    return out, problems


# --- The report --------------------------------------------------------------------------------


def pct(xs: list[float], q: float) -> float:
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))] if s else float("nan")


def record_value(record: list[str], key: str) -> str | None:
    for line in record:
        if line.startswith(key):
            return line[len(key):].strip()
    return None


def record_batch(record: list[str]) -> set[str]:
    """The batch workloads' names from the run record ("a,b,c", followed by
    " (exempt: ...)" since the exemption was added)."""
    v = (record_value(record, "batch workloads:") or "").split(" (exempt:")[0]
    return {n.strip() for n in v.split(",")} - {""}


def record_offsets(record: list[str]) -> dict:
    """REALTIME, BOOTTIME and TAI minus MONOTONIC (ns) from the run record's
    clock pair and clock offsets lines; None where not recorded."""
    text = "\n".join(record)
    out = {}
    for key, name in (("REALTIME", "realtime"), ("BOOTTIME", "boottime"), ("TAI", "tai")):
        m = re.search(rf"\b{name}_minus_monotonic_ns (-?\d+)", text)
        out[key] = int(m[1]) if m else None
    return out


def reference_pairs(samp: list[tuple], rt_cpu: int) -> list[tuple]:
    """The sampler pairs of the tracer CPU that sampled the most (the one sampling throughout the run)."""
    by: dict[int, list] = {}
    for s in samp:
        if s[3] != rt_cpu:
            by.setdefault(s[3], []).append(s)
    return max(by.values(), key=len) if by else []


def report(csv_path: pathlib.Path, ev: TimerEvents, pairs_of_sleeps: list[tuple], record: list[str],
           failures: list, top: int, out, sleep_calls: list[int] | None = None,
           exploratory_ppm: float | None = None, report_tool: dict | None = None) -> None:
    """The timer service section of trace_correlate's report, for a run
    recorded in the timer mode. The evidence the mode records must be
    there: the deadline writes, local timer interrupts and system calls on
    the real-time CPU, the worker's sleep calls and timer armings, both
    buffers' loss counts (zero), the TSC instance, sampler pairs on a
    tracer CPU and on the real-time CPU before and after the run, a valid
    SMI count series, the timekeeping record, and the exerciser's file when
    the record says it ran. A late service is a result, not a failure.
    exploratory_ppm supplies a slope range for a capture with no certificate:
    its U is labelled exploratory and the run still fails. report_tool is the
    identity of the trace-cmd that reported the main trace for this analysis
    (tools/tk_record.py tool_identity); the certificate requires the audited
    build there too."""
    rt = ev.rt_cpu
    calls = sorted(sleep_calls or [])
    print("  timer service (the real-time CPU's deadlines, idle transitions and interrupts):", file=out)
    for what, ok in ((f"deadline writes on CPU {rt}", ev.writes),
                     (f"local timer interrupts on CPU {rt}", len(ev.anchors) >= 3),
                     (f"system calls on CPU {rt}", ev.syscalls),
                     ("sleep calls of the worker", calls),
                     ("timer armings of the worker",
                      any(s[2] == ev.tid and s[5] == "hrtimer_wakeup" for s in ev.starts))):
        if not ok:
            failures.append(f"timer service: the timer-mode run has no {what}")
            print(f"    MISSING: {what}", file=out)
    for key, what in (("trace overruns:", "the trace"), ("tsc instance overruns:", "the TSC instance")):
        over = record_value(record, key)
        tok = over.split()[0] if over else ""
        if over is None:
            failures.append(f"timer service: the record has no count of {what}'s losses")
        elif not tok.isdigit():
            failures.append(f"timer service: the record has no valid count of {what}'s losses ({over})")
        elif tok != "0":
            failures.append(f"timer service: {what} lost events ({over})")
        print(f"    {key} {over}", file=out)
    # The conversion, assigned by deadline slot, bounded and certified.
    print(f"    analysis: {ANALYSIS_REVISION}", file=out)
    if "main timer event enable before the TSC instance: 1" not in [ln.strip() for ln in record]:
        failures.append("timer service: the main trace's timer event was not confirmed enabled before the TSC instance")
    inst = pathlib.Path(str(csv_path) + ".tsc.txt")
    pairs_file = pathlib.Path(str(csv_path) + ".tsc-pairs.csv")
    u = None
    conv = TscMap([])
    if not inst.exists() or not pairs_file.exists():
        failures.append("timer service: no TSC instance or sampler pairs")
    else:
        tsc, inst_problems = timer_clock.parse_instance(inst.read_text(errors="replace"), rt)
        samp, pair_problems = timer_clock.parse_pairs(pairs_file.read_text(errors="replace"))
        model = timer_clock.Model(exploratory_ppm * 1e-6) if exploratory_ppm is not None else None
        r = timer_clock.analyze((ev.ledger.events, ev.ledger.problems), (tsc, inst_problems + pair_problems),
                                reference_pairs(samp, rt), window=ev.window,
                                evidence=tk_record.load(csv_path, report_tool=report_tool), model=model)
        for why in r["why"]:
            failures.append(f"timer service: {why}")
        print(f"    conversion certificate: {r['cert']}", file=out)
        for why in r["cert_reasons"]:
            print(f"      {why}", file=out)
        if not r["accepted"] and r["status"] == "PASS":
            failures.append("timer service: the conversion's bound is not certified: "
                            + "; ".join(r["cert_reasons"][:3]))
        elif r["status"] != "PASS" and r["status"] != "FAIL":
            failures.append("timer service: the conversion was not analysed: " + "; ".join(r["cert_reasons"][:3]))
        if r.get("U") is not None:
            m = r["model"]
            print(f"    anchors: {len(tsc)} in the TSC instance, {len(ev.ledger.events)} ledger records, {r['paired']} paired by "
                  f"deadline slot, {len(r['left_out'])} left out of the conversion, {r['nomap']} with no independent "
                  f"check; {len(samp)} sampler pairs", file=out)
            print(f"    clock model ({m['source']}): slope range {m['ds'] * 1e6:.3f} ppm, read allowance {m['W_ns'] / 1e6:.1f} ms, "
                  f"R {m['R_ns']:.1f} ns", file=out)
            if r.get("provenance"):
                prov = r["provenance"]
                print(f"    certificate provenance: {prov['audit']}, {prov['trace_cmd']} ({prov['trace_runtime']}), mult {prov['mult']} shift {prov['shift']}, tick "
                      f"{prov['tick_us']} us, deviation {prov['deviation_ppm']:.1f} ppm, longest update "
                      f"{prov['observed_max_update_ns']} ns over {prov['updates_in_interval']} updates", file=out)
            print(f"    conversion bound U: {r['U']:.0f} ns (limit {U_LIMIT_NS} ns), {r['U_label']}; longest conversion "
                  f"interval {r['g_conv_ms']:.1f} ms", file=out)
            if r["status"] == "PASS":
                conv = TscMap(r["pairs"])
                u = r["U"]
                # Diagnostics: the residuals are not the bound.
                v = validate(r["pairs"], samp)
                held = v["held"]
                if held:
                    print(f"    held-out anchors (diagnostic): n={len(held)} p50 {pct(held, .5):.0f} p99 "
                          f"{pct(held, .99):.0f} max {max(held):.0f} ns", file=out)
                for cpu, rs in sorted(v["by_cpu"].items()):
                    res = [x for x, _ in rs]
                    print(f"    sampler on CPU {cpu} (diagnostic): n={len(rs)} residual median {statistics.median(res):+.0f} ns, "
                          f"|residual| + half bracket max {max(abs(x) + h for x, h in rs):.0f} ns", file=out)
        # The sampler's promised windows, inside the anchors: a tracer CPU,
        # and the real-time CPU before and after the run.
        lo, hi = (min(p[1] for p in r["pairs"]), max(p[1] for p in r["pairs"])) if r.get("pairs") else (0, -1)
        inside = [s for s in samp if lo <= s[0] and s[2] <= hi]
        if not any(s[3] != rt for s in inside):
            failures.append("timer service: no sampler pairs on a tracer CPU inside the anchors")
        if ev.window and not (any(s[3] == rt and s[2] < ev.window[0] for s in inside)
                              and any(s[3] == rt and s[0] > ev.window[1] for s in inside)):
            failures.append(f"timer service: no sampler pairs on CPU {rt} inside the anchors both before and after "
                            "the run")
    tl = Timeline(ev, conv, u)
    offsets = record_offsets(record)
    lt = Lifetimes(ev, conv, u, offsets)
    # SMIs.
    anchor = record_value(record, "smi anchor monotonic_ns:")
    parts = anchor.split() if anchor else []
    smi_anchors = (int(parts[0]), int(parts[1])) if len(parts) >= 2 and parts[0].isdigit() and parts[1].isdigit() \
        else None
    smi, smi_problems = load_smi(pathlib.Path(str(csv_path) + ".smi.csv"), smi_anchors, ev.window)
    for p in smi_problems:
        failures.append(f"timer service: {p}")
    # The worker's sleeps.
    arms = sorted((s[0], s[4], s[6]) for s in ev.starts if s[2] == ev.tid and s[5] == "hrtimer_wakeup")
    arm_ts = [a[0] for a in arms]
    results = []
    for (entry, now, _, _, _, addr), (i, prev_end, sched, _) in pairs_of_sleeps:
        a0, a1 = bisect.bisect_right(arm_ts, prev_end), bisect.bisect_right(arm_ts, entry)
        mine = [a for a in arms[a0:a1] if a[1] == addr]
        if not mine:
            results.append({"class": "INDETERMINATE", "late": now - sched, "detail": "no arming recorded", "i": i})
            continue
        c0, c1 = bisect.bisect_right(calls, prev_end), bisect.bisect_right(calls, mine[-1][0])
        call = calls[c1 - 1] if c1 > c0 else None
        r = classify(ev, tl, (mine[-1][0], mine[-1][2]), (entry, now), smi, call)
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
        att = attribute(ev, lt, conv, u)
        ind = sum(1 for a in att if a[2].startswith("INDETERMINATE"))
        print(f"    deadline writes: {len(att)}, {sum(1 for a in att if a[2] == 'disarmed')} disarming, "
              f"{ind} INDETERMINATE in attribution", file=out)
    # Idle states, NMIs and SMIs.
    states: dict[int, int] = {}
    for _, s in ev.idle:
        if s != IDLE_EXIT:
            states[s] = states.get(s, 0) + 1
    print(f"    idle entries by state: {dict(sorted(states.items()))}; idle misses {len(ev.idle_miss)}; "
          f"NMIs {len(ev.nmis)}; SMIs {sum(x[2] for x in smi) if smi else 'not counted'}", file=out)
    # The exerciser.
    exf = pathlib.Path(str(csv_path) + ".exerciser.csv")
    if record_value(record, "exerciser:") is not None and not exf.exists():
        failures.append("timer service: the record says the exerciser ran, but its file is missing")
    if exf.exists():
        if u is None or u > U_LIMIT_NS:
            failures.append("timer service: the exerciser cannot be checked without a validated conversion")
        else:
            head, rows = load_exerciser(exf)
            x = explain_exerciser(ev, tl, lt, head, rows)
            print(f"    exerciser: {x['actions']} actions: OK {x['OK']}, no write needed {x['no write needed']}, "
                  f"armed after its expiry {x['armed after its expiry']}, INDETERMINATE {x['INDETERMINATE']}, "
                  f"UNEXPLAINED {x['UNEXPLAINED']}", file=out)
            if x["indeterminate"]:
                print("      INDETERMINATE: " + ", ".join(f"{k} {v}" for k, v in sorted(x["indeterminate"].items())),
                      file=out)
            for n in x["notes"][:top]:
                print(f"      {n}", file=out)
            if x["UNEXPLAINED"]:
                failures.append(f"timer service: {x['UNEXPLAINED']} exerciser actions unexplained")
            if not x["actions"]:
                failures.append("timer service: the exerciser's file has no actions")
    # Background load.
    batch = record_batch(record)
    sp = record_value(record, "session pid:")
    bg = background(ev, int(sp) if sp and sp.isdigit() else None,
                    load_procs(pathlib.Path(str(csv_path) + ".procs.txt")), batch)
    topc = sorted(bg["by_comm"].items(), key=lambda kv: -kv[1][0])[:6]
    print("    foreign CPU time in the run: " + (", ".join(f"{c} {v[0] / 1e6:.0f} ms" for c, v in topc) or "none"),
          file=out)
    if bg["batch"]:
        print(f"    UNCONTROLLED background load: {', '.join(bg['batch'])} ran", file=out)
