"""The fair server's probes for the timer mode of tools/trace_session.sh
(docs/hardware/control-rate.md, Stage 1: fair-server observation).

The fair server (rq->fair_server) is a deadline-class entity that runs
normal tasks ahead of real-time ones while it is active. Its entry into a
CPU's schedule is observed, not inferred from function entry:

- a selection: fair_server_pick_task returned a task (a return probe), and
  the deadline class's pick (pick_task_dl, entry and return probes) returned
  that same task pointer in the same pick; a pick returning no task is
  counted apart. Neither return probe reads anything through the pointer it
  records, since either may return NULL: a read through NULL faults in the
  kernel (the rehearsal of 2026-11-08 counted 56,712 kernel page faults of
  the worker within the probe's run, at the NULL-page offsets of the fields
  then read through the deadline class's return);
- the task's identity: set_next_task_fair(rq, p, first) runs for every fair
  task about to run on the CPU, a server-dispatched one included, with p
  never NULL; its probe records the pointer and the pid read through it, so
  each pointer's pid is known once the task has been set next there. A
  selected task whose pid is not known is counted as unidentified, never as
  server-backed;
- the server's state at the selection: the fair server's flags word (active,
  throttled, deferred, armed, running...), runtime and deadline, read at the
  fair server pick's entry and at the deadline class's pick entry;
- dispatch: the selection is followed, on that CPU, by the sched_switch to
  the selected task, identified by its pid (or, when it was already running,
  by that task staying on until its next switch).

dl_server_start, dl_server_stop and dl_server_timer are probed with the
server's state before each call, as context only: a start can return without
enqueueing the server, so its entry is never counted as activation, and any
of the three may change the state just recorded (the timer can set
deferred-running and enqueue the server), so their entry states never
establish the state after them. The state read at the deadline class's pick
entry is the state at that moment.

A kernel page fault at a NULL-page address on the CPU during the run (a read
through a NULL pointer: in practice a probe's) is counted, and in a capture
with the probes it fails the run.

The probes are kprobe events whose arguments are memory offsets read from the
kernel's BTF at each session (they change with the kernel's build), so a
session records the layout it used. This module prints the layout and the
definitions (layout, definitions); tools/fair_server_probes.sh writes them to
tracefs as root.

The analysis (FairServerEvents, report; through tools/trace_correlate.py)
follows each wake of the sleeping worker on the real-time CPU, from its
sched_waking to its switch-in, and splits that CPU's time inside the wait
into overlapping execution intervals, each labelled by what occupied the
CPU: a normal task in validated server-backed dispatch (which may have
begun before the wake); a normal task already running at the wake and left
running with no selection (preemption delay, only when the probes are in the
capture and missed nothing); a normal task without validated dispatch
otherwise; a task the server selected but that was never identified
(unidentified: perhaps server-backed, never preemption delay); a
contradictory server state; a higher-class task (real-time,
stopper, or deadline-class not selected by the fair server); interrupts and
softirqs; or idle and switching. A wait of 50 us or more is server-backed
only when server-backed intervals cover it apart from interrupt and
switching time; it is mixed when they cover part of it beside another
mechanism, other mechanism when none of it, and INDETERMINATE when
normal-task time without validated dispatch, unidentified time, or a
contradictory server state, covers more than the tolerance. The result says
what occupied the CPU, never a sole cause. A capture without the probes (an
earlier one) gets the same waits, its normal-task time unattributed; normal
tasks running without a recorded dl_server_start never invalidate a
capture.

The controls (controls; tools/fair_server_control.py's output and its
trace) validate the probes on a spare rehearsal CPU, with the same
definitions filtered to that CPU. The positive control passes when the
trace shows the FIFO loop at a real-time priority there, and the normal
task both progressing and dispatched by the fair server while the loop ran;
no fixed share is required. A negative control's preconditions must be
established first, from the server's state as read after the last call
that could change it (this kernel never calls dl_server_stop in these runs:
the rehearsal of 2026-11-08 recorded none, the server going back to
deferred, armed and throttled, or inactive): no probe misses; no
fair-server selection between the previous test and the control; and a
state read at a deadline-class pick in that gap after the last
dl_server_start, dl_server_stop or dl_server_timer entry there (an entry
state precedes a call that may change it), the latest such state showing
the server not in its running mode (a deferred server not deferred-running,
or an inactive one). The controls tool runs a brief SCHED_FIFO task on the
CPU at the end of each settle, so that such a pick reads the state. Without
them the control is INCONCLUSIVE; with them it passes when the server
dispatched nothing (the normal task alone, which must have run) or
selected nothing (the idle CPU), and fails otherwise.

usage: fair_server.py layout [--btf-raw FILE]
       fair_server.py definitions --group GROUP [--btf-raw FILE]
       fair_server.py controls --controls FILE --record FILE (--trace-dat FILE | --report FILE)
"""

from __future__ import annotations

import argparse
import bisect
import os
import re
import statistics
import subprocess
import sys

GROUP = "rtctrl_fs"
EVENTS = ("rtfs_dl_pick", "rtfs_dl_pick_ret", "rtfs_fs_pick", "rtfs_fs_pick_ret", "rtfs_set_next",
          "rtfs_srv_start", "rtfs_srv_stop", "rtfs_srv_timer")
# Events filtered by the CPU they run on (picks are local to their CPU), and
# those filtered by the server's own CPU (a start or stop can run on another).
CPU_FILTERED = ("rtfs_dl_pick", "rtfs_dl_pick_ret", "rtfs_fs_pick", "rtfs_fs_pick_ret", "rtfs_set_next")
SERVER_FILTERED = ("rtfs_srv_start", "rtfs_srv_stop", "rtfs_srv_timer")
FLAGS = ("dl_throttled", "dl_non_contending", "dl_server", "dl_server_active", "dl_defer",
         "dl_defer_armed", "dl_defer_running", "dl_defer_idle")
NEEDED = {
    "rq": ("fair_server", "ext_server", "cpu"),
    "sched_dl_entity": ("runtime", "deadline", "rq") + FLAGS,
    "task_struct": ("pid", "policy", "prio"),
}
TYPE = re.compile(r"^\[(\d+)\] (\w+) '([^']*)'")
MEMBER = re.compile(r"^\t'(\w+)' type_id=\d+ bits_offset=(\d+)(?: bitfield_size=(\d+))?")
LAYOUT_LINE = re.compile(r"^fair-server probe layout: (.*)$")


WAIT_CLASS_NS = 50_000   # a wait this long is classified (the timer mode's class threshold)
TOLERANCE_NS = 2_000     # time a label may hold without changing a wait's class
SWITCH_FULL = re.compile(r"^(?P<prev>.*):(?P<ppid>\d+) \[(?P<pprio>-?\d+)\] (?P<state>\S+) ==> "
                         r"(?P<next>.*):(?P<npid>\d+) \[(?P<nprio>-?\d+)\]")
KV = re.compile(r"(\w+)=(\S+)")
WAKE = re.compile(r"\bpid=(\d+)\b.*?\btarget_cpu=(\d+)")
WAKEUP = re.compile(r":(\d+) \[-?\d+\](?: success=\d+)? CPU:(\d+)")
IRQ_EVENTS = {"irq_handler", "softirq", "local_timer", "reschedule", "call_function", "call_function_single"}
DL_TIMERS = ("dl_task_timer", "inactive_task_timer")
LABELS = ("server-backed", "preemption delay", "normal unvalidated", "unidentified", "contradictory",
          "higher class", "interrupt", "idle/switching")
FEED_EVENTS = {"sched_switch", "sched_waking", "sched_wakeup", "hrtimer_start", "hrtimer_expire_entry",
               "page_fault_kernel",
               *(f"{e}_{x}" for e in ("irq_handler", "softirq", "local_timer", "reschedule", "call_function",
                                      "call_function_single") for x in ("entry", "exit"))}


class LayoutError(Exception):
    pass


def layout_from_btf_raw(lines) -> dict[str, int]:
    """The offsets the probes read, from `bpftool btf dump ... format raw`:
    byte offsets of the members, and each flag's bit in the 32-bit word that
    holds them all."""
    found: dict[str, list[dict[str, tuple[int, int]]]] = {}
    current: dict[str, tuple[int, int]] | None = None
    for line in lines:
        t = TYPE.match(line)
        if t:
            current = None
            if t[2] == "STRUCT" and t[3] in NEEDED:
                current = {}
                found.setdefault(t[3], []).append(current)
            continue
        if current is not None:
            m = MEMBER.match(line)
            if m:
                current[m[1]] = (int(m[2]), int(m[3] or 0))
    # The first struct of each name with every needed member: another struct
    # of the name without them (a module's, say) does not count.
    members: dict[str, dict[str, tuple[int, int]]] = {}
    for name, need in NEEDED.items():
        whole = [s for s in found.get(name, []) if all(m in s for m in need)]
        if not whole:
            raise LayoutError(f"no BTF struct {name} with {', '.join(need)}")
        members[name] = whole[0]
    lay: dict[str, int] = {}
    for name, short in (("rq", "rq"), ("sched_dl_entity", "se"), ("task_struct", "task")):
        for m, (bits, size) in members[name].items():
            if m in NEEDED[name] and m not in FLAGS:
                if size or bits % 8:
                    raise LayoutError(f"{name}.{m} is not byte-aligned")
                lay[f"{short}.{m}"] = bits // 8
    words = {members["sched_dl_entity"][f][0] // 32 for f in FLAGS}
    if len(words) != 1:
        raise LayoutError("the server's flags are not in one 32-bit word")
    word = words.pop()
    lay["se.flags_word"] = word * 4
    for f in FLAGS:
        bits, size = members["sched_dl_entity"][f]
        if size != 1:
            raise LayoutError(f"sched_dl_entity.{f} is not a one-bit field")
        lay[f"bit.{f}"] = bits - word * 32
    return lay


def layout_text(lay: dict[str, int]) -> str:
    return "fair-server probe layout: " + " ".join(f"{k}={v}" for k, v in lay.items())


def parse_layout(record_lines) -> dict[str, int] | None:
    """The layout a session recorded, or None."""
    for line in record_lines:
        m = LAYOUT_LINE.match(line.strip())
        if m:
            return {k: int(v) for k, v in (p.split("=", 1) for p in m[1].split())}
    return None


def definitions(lay: dict[str, int], group: str = GROUP) -> list[str]:
    """The kprobe_events lines, one per event."""
    fs = lay["rq.fair_server"]
    se_state = (f"bits=+{lay['se.flags_word']}($A):x32 runtime=+{lay['se.runtime']}($A):s64 "
                f"deadline=+{lay['se.deadline']}($A):u64")
    se_where = f"srv_rq=+{lay['se.rq']}($A):x64 srv_cpu=+{lay['rq.cpu']}(+{lay['se.rq']}($A)):s32"

    def server(event: str, func: str, arg: str) -> str:
        return (f"p:{group}/{event} {func} se={arg}:x64 {se_where.replace('$A', arg)} "
                f"{se_state.replace('$A', arg)}")

    return [
        (f"p:{group}/rtfs_dl_pick pick_task_dl rq=$arg1:x64 rq_cpu=+{lay['rq.cpu']}($arg1):s32 "
         f"fs_bits=+{fs + lay['se.flags_word']}($arg1):x32 fs_runtime=+{fs + lay['se.runtime']}($arg1):s64 "
         f"fs_deadline=+{fs + lay['se.deadline']}($arg1):u64"),
        f"r:{group}/rtfs_dl_pick_ret pick_task_dl task=$retval:x64",
        server("rtfs_fs_pick", "fair_server_pick_task", "$arg1"),
        f"r:{group}/rtfs_fs_pick_ret fair_server_pick_task task=$retval:x64",
        (f"p:{group}/rtfs_set_next set_next_task_fair task=$arg2:x64 pid=+{lay['task.pid']}($arg2):s32 "
         f"policy=+{lay['task.policy']}($arg2):u32"),
        server("rtfs_srv_start", "dl_server_start", "$arg1"),
        server("rtfs_srv_stop", "dl_server_stop", "$arg1"),
        server("rtfs_srv_timer", "dl_server_timer", "$arg2"),
    ]


def filters(cpu: int) -> list[tuple[str, str]]:
    """Each event with its trace-cmd filter for one CPU."""
    return [(e, f"CPU == {cpu}" if e in CPU_FILTERED else f"srv_cpu == {cpu}") for e in EVENTS]


def num(v: str) -> int | None:
    try:
        return int(v, 16) if v.startswith(("0x", "-0x")) else int(v)
    except ValueError:
        return None


def fields(rest: str) -> dict[str, int]:
    out: dict[str, int] = {}
    for k, v in KV.findall(rest):
        n = num(v)
        if n is not None:
            out[k] = n
    return out


class Selection:
    """A fair-server selection on the CPU: the task pointer, its pid (None
    until known), its pick's time and the server's state read at the fair
    server pick's entry."""

    __slots__ = ("ts", "task", "pid", "bits", "valid")

    def __init__(self, ts: int, task: int, pid: int | None, bits: int | None, valid: bool) -> None:
        self.ts, self.task, self.pid, self.bits, self.valid = ts, task, pid, bits, valid


class FairServerEvents:
    """One CPU's execution timeline and the fair server's probe events on it,
    from trace-cmd report lines (fed every line by tools/trace_correlate.py):
    the segments between switches, each labelled by what ran and how it was
    dispatched, the interrupt intervals, the worker's wakings and wakeups,
    and the counts of the server's selections, dispatches and calls."""

    def __init__(self, cpu: int, layout: dict[str, int] | None, worker: int | None = None,
                 window: tuple[int, int] | None = None) -> None:
        self.cpu, self.layout, self.worker, self.window = cpu, layout, worker, window
        self.segments: list[list] = []   # [start, end, pid, comm, label]
        self._open: list | None = None    # the running segment
        self.irqs: list[tuple] = []       # (start, end) of interrupt time on the CPU
        self._irq_depth = 0
        self._irq_start = 0
        self.wakings: list[tuple] = []    # (ts, waker comm, waker cpu)
        self.wakeups: list[int] = []      # completed sched_wakeup of the worker on the CPU
        self._pick: dict | None = None     # the deadline-class pick in progress
        self._fs_ret: tuple | None = None  # (task, pid) the fair server's pick returned in it
        self._fs_bits: int | None = None
        self.pending: Selection | None = None
        self.counts = {k: 0 for k in ("dl picks", "fair selections", "dispatched", "re-selected",
                                      "not dispatched", "unidentified", "contradictory", "no task",
                                      "other deadline-class selections", "set next")}
        self.ptr_pid: dict[int, int] = {}   # task pointer -> pid, from set_next_task_fair on the CPU
        self.null_faults: list[int] = []    # kernel page faults at NULL-page addresses on the CPU
        self.server_calls = {"rtfs_srv_start": 0, "rtfs_srv_stop": 0, "rtfs_srv_timer": 0}
        self.server_states: list[tuple] = []  # (ts, event, bits) of the CPU's fair server
        self.selection_log: list[tuple] = []  # (ts, pid, valid) of each fair-server selection
        self.dispatch_log: list[tuple] = []   # (ts, pid, valid) of each dispatch or re-selection
        self.dl_timers = {f"{f} {k}": 0 for f in DL_TIMERS for k in ("start", "expiry")}
        self.probe_events = 0

    # -- feeding ---------------------------------------------------------------

    def feed(self, ts: int, cpu: int, pid: int, comm: str, ev: str, rest: str) -> None:
        if ev.startswith("rtfs_"):
            self.probe(ts, cpu, ev, rest)
            return
        if ev == "sched_waking":
            if self.worker is not None and f"pid={self.worker} " in rest + " ":
                w = WAKE.search(rest)
                if w and int(w[1]) == self.worker and int(w[2]) == self.cpu:
                    self.wakings.append((ts, comm, cpu))
            return
        if ev == "sched_wakeup":
            if self.worker is not None and f":{self.worker} " in rest:
                w = WAKEUP.search(rest)
                if w and int(w[1]) == self.worker and int(w[2]) == self.cpu:
                    self.wakeups.append(ts)
            return
        if cpu != self.cpu:
            return
        if ev == "page_fault_kernel":
            a = re.search(r"\baddress=(0x[0-9a-f]+|\d+)", rest)
            if a and num(a[1]) is not None and num(a[1]) < 4096:
                self.null_faults.append(ts)
            return
        if ev == "sched_switch":
            self.switch(ts, rest)
        elif ev.endswith("_entry") and ev[:-6] in IRQ_EVENTS:
            if self._irq_depth == 0:
                self._irq_start = ts
            self._irq_depth += 1
        elif ev.endswith("_exit") and ev[:-5] in IRQ_EVENTS:
            if self._irq_depth > 0:
                self._irq_depth -= 1
                if self._irq_depth == 0:
                    self.irqs.append((self._irq_start, ts))
        elif ev in ("hrtimer_start", "hrtimer_expire_entry"):
            for f in DL_TIMERS:
                if f"function={f}" in rest:
                    self.dl_timers[f"{f} {'start' if ev == 'hrtimer_start' else 'expiry'}"] += 1

    def label_of(self, pid: int, prio: int) -> str:
        if pid == 0:
            return "idle/switching"
        if pid == self.worker:
            return "worker"
        return "normal unvalidated" if prio >= 100 else "higher class"

    def switch(self, ts: int, rest: str) -> None:
        s = SWITCH_FULL.match(rest)
        if not s:
            return
        npid, nprio = int(s["npid"]), int(s["nprio"])
        if self._open is not None:
            self._open[1] = ts
            self.segments.append(self._open)
        label = self.label_of(npid, nprio)
        sel = self.pending
        if sel is not None:
            self.pending = None
            if sel.pid is None:
                self.counts["unidentified"] += 1
                label = "unidentified"  # perhaps server-backed: never preemption delay
            elif sel.pid == npid:
                self.counts["dispatched"] += 1
                self.dispatch_log.append((ts, npid, sel.valid))
                label = "server-backed" if sel.valid else "contradictory"
            else:
                self.counts["not dispatched"] += 1
        self._open = [ts, None, npid, s["next"], label]

    def probe(self, ts: int, cpu: int, ev: str, rest: str) -> None:
        f = fields(rest)
        if ev in ("rtfs_srv_start", "rtfs_srv_stop", "rtfs_srv_timer"):
            if f.get("srv_cpu") != self.cpu or self.layout is None:
                return
            if f.get("se", -1) != f.get("srv_rq", -2) + self.layout["rq.fair_server"]:
                return  # the ext server's
            self.probe_events += 1
            self.server_calls[ev] += 1
            self.server_states.append((ts, ev, f.get("bits")))
            return
        if cpu != self.cpu:
            return
        self.probe_events += 1
        if ev == "rtfs_dl_pick":
            self.server_states.append((ts, ev, f.get("fs_bits")))  # the CPU's fair server at the pick
            # A selection not followed by a switch: its task stayed on (a
            # re-selection), or another pick replaced it.
            sel = self.pending
            if sel is not None:
                self.pending = None
                if sel.pid is None:
                    self.counts["unidentified"] += 1
                    if self._open is not None and self._open[3] != "swapper" and self._open[2] != 0:
                        seg = self._open
                        if sel.ts > seg[0]:
                            seg[1] = sel.ts
                            self.segments.append(seg)
                            self._open = [sel.ts, None, seg[2], seg[3], seg[4]]
                        self._open[4] = "unidentified"
                elif self._open is not None and self._open[2] == sel.pid:
                    self.counts["re-selected"] += 1
                    self.dispatch_log.append((sel.ts, sel.pid, sel.valid))
                    seg = self._open
                    if sel.ts > seg[0]:
                        seg[1] = sel.ts
                        self.segments.append(seg)
                        self._open = [sel.ts, None, seg[2], seg[3], seg[4]]
                    self._open[4] = "server-backed" if sel.valid else "contradictory"
                else:
                    self.counts["not dispatched"] += 1
            self.counts["dl picks"] += 1
            self._pick = f
            self._fs_ret = None
            self._fs_bits = None
        elif ev == "rtfs_fs_pick":
            self._fs_bits = f.get("bits")
            self.server_states.append((ts, ev, f.get("bits")))
        elif ev == "rtfs_fs_pick_ret":
            task = f.get("task", 0)
            if task == 0:
                self.counts["no task"] += 1
                self._fs_ret = None
            else:
                self._fs_ret = task
        elif ev == "rtfs_set_next":
            task, pid = f.get("task", 0), f.get("pid")
            if task and pid is not None:
                self.counts["set next"] += 1
                self.ptr_pid[task] = pid
                if self.pending is not None and self.pending.task == task:
                    self.pending.pid = pid
        elif ev == "rtfs_dl_pick_ret":
            task = f.get("task", 0)
            if task and self._fs_ret is not None and self._fs_ret == task:
                bits = self._fs_bits
                self.counts["fair selections"] += 1
                valid = self.state_valid(bits)
                if not valid:
                    self.counts["contradictory"] += 1
                pid = self.ptr_pid.get(task)
                self.pending = Selection(ts, task, pid, bits, valid)
                self.selection_log.append((ts, pid, valid))
            elif task:
                self.counts["other deadline-class selections"] += 1
            self._pick = None
            self._fs_ret = None

    def flag(self, bits: int | None, name: str) -> int | None:
        if bits is None or self.layout is None:
            return None
        return (bits >> self.layout[f"bit.{name}"]) & 1

    def state_valid(self, bits: int | None) -> bool:
        """The server picked must be active and not throttled."""
        return self.flag(bits, "dl_server_active") == 1 and self.flag(bits, "dl_throttled") == 0

    def finish(self, end: int) -> None:
        if self._open is not None:
            self._open[1] = end
            self.segments.append(self._open)
            self._open = None
        if self._irq_depth > 0:
            self.irqs.append((self._irq_start, end))
            self._irq_depth = 0

    # -- the waits ---------------------------------------------------------------

    def waits(self, complete: bool = False) -> list[dict]:
        """Each wake of the sleeping worker on the CPU: from its sched_waking
        to its switch-in, with the CPU's time inside by label. complete says
        that the probes are in the capture and missed nothing, so a normal
        task running at the wake with no selection was left running."""
        segs = sorted(self.segments, key=lambda g: g[0])
        starts = [g[0] for g in segs]
        ins = [g[0] for g in segs if g[2] == self.worker]
        irq_starts = [a for a, _ in self.irqs]
        out: list[dict] = []
        last_end = -1
        for tw, waker, wcpu in self.wakings:
            if self.window and not (self.window[0] <= tw <= self.window[1]):
                continue
            if tw < last_end:
                continue  # the worker was already runnable
            i = bisect.bisect_right(starts, tw) - 1
            if i >= 0 and segs[i][2] == self.worker and segs[i][1] > tw:
                continue  # it was running: the waking found it on the CPU
            j = bisect.bisect_left(ins, tw)
            if j >= len(ins):
                continue
            t_in = ins[j]
            last_end = t_in
            by = {k: 0 for k in LABELS}
            ran: list[str] = []
            k = max(i, 0)
            while k < len(segs) and segs[k][0] < t_in:
                a, b = max(segs[k][0], tw), min(segs[k][1], t_in)
                if b > a and segs[k][2] != self.worker:
                    by[self.wait_label(segs[k], tw, complete)] += b - a
                    if segs[k][2] != 0 and (not ran or ran[-1] != segs[k][3]):
                        ran.append(segs[k][3])
                k += 1
            # Interrupt time comes out of whatever label it overlapped.
            irq = 0
            m = max(bisect.bisect_right(irq_starts, tw) - 1, 0)
            while m < len(self.irqs) and self.irqs[m][0] < t_in:
                a, b = max(self.irqs[m][0], tw), min(self.irqs[m][1], t_in)
                if b > a:
                    irq += b - a
                    self.remove_overlap(by, segs, starts, a, b, tw, complete)
                m += 1
            by["interrupt"] = irq
            wu = bisect.bisect_left(self.wakeups, tw)
            wakeup = self.wakeups[wu] if wu < len(self.wakeups) and self.wakeups[wu] <= t_in else None
            w = {"ts": tw, "in": t_in, "ns": t_in - tw, "by": by, "ran": ran, "waker": waker,
                 "waker_cpu": wcpu, "wakeup": wakeup}
            w["class"] = classify(w) if w["ns"] >= WAIT_CLASS_NS else None
            out.append(w)
        return out

    def wait_label(self, seg: list, tw: int, complete: bool) -> str:
        if seg[4] == "normal unvalidated" and complete and seg[0] < tw:
            return "preemption delay"
        return seg[4]

    def remove_overlap(self, by: dict, segs: list, starts: list[int], a: int, b: int, tw: int,
                       complete: bool) -> None:
        k = max(bisect.bisect_right(starts, a) - 1, 0)
        while k < len(segs) and segs[k][0] < b:
            x, y = max(segs[k][0], a), min(segs[k][1], b)
            if y > x and segs[k][2] != self.worker:
                by[self.wait_label(segs[k], tw, complete)] -= y - x
            k += 1


def classify(w: dict) -> str:
    by = w["by"]
    sb = by["server-backed"]
    unknown = by["normal unvalidated"] + by["unidentified"] + by["contradictory"]
    other = by["higher class"] + by["idle/switching"] + by["preemption delay"]
    if unknown > TOLERANCE_NS:
        return "INDETERMINATE"
    if sb > TOLERANCE_NS:
        return "server-backed" if other <= TOLERANCE_NS else "mixed"
    return "other mechanism"


def percentile(xs: list[int], q: float) -> float:
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def report(ev: FairServerEvents, misses: int | None, top: int, out, failures: list[str]) -> None:
    """The waits after waking and the fair server's evidence."""
    ws = ev.waits(complete=ev.layout is not None and misses == 0)
    print(f"  waits after waking (CPU {ev.cpu}: the worker's sched_waking to its switch-in, "
          "while it was off the CPU):", file=out)
    if not ws:
        print("    none", file=out)
    else:
        ns = sorted(w["ns"] for w in ws)
        print(f"    n={len(ns)} p50 {statistics.median(ns) / 1e3:.1f} p99 {percentile(ns, 0.99) / 1e3:.1f} "
              f"p99.9 {percentile(ns, 0.999) / 1e3:.1f} max {ns[-1] / 1e3:.1f} us; "
              f"20 us or more: {sum(1 for x in ns if x >= 20_000)}, "
              f"50 us or more: {sum(1 for x in ns if x >= WAIT_CLASS_NS)}", file=out)
    if ev.layout is None:
        print("    fair-server probes: not in this capture; normal-task time in a wait is unattributed "
              "(INDETERMINATE)", file=out)
    else:
        c = ev.counts
        print(f"    fair-server probes: {ev.probe_events} events on CPU {ev.cpu}; deadline-class picks "
              f"{c['dl picks']}; fair-server selections {c['fair selections']} (dispatched {c['dispatched']}, "
              f"re-selected while running {c['re-selected']}, not dispatched {c['not dispatched']}, "
              f"unidentified {c['unidentified']}, contradictory state {c['contradictory']}); fair-server picks "
              f"returning no task {c['no task']}; other deadline-class selections "
              f"{c['other deadline-class selections']}; fair tasks set next {c['set next']}", file=out)
        sc = ev.server_calls
        print(f"    CPU {ev.cpu}'s fair server: start {sc['rtfs_srv_start']}, stop {sc['rtfs_srv_stop']}, "
              f"timer {sc['rtfs_srv_timer']} (entries; a start is not an activation)", file=out)
        print(f"    probe misses during the run: {'unknown' if misses is None else misses}", file=out)
        if misses:
            print("      a missed selection leaves its dispatch unvalidated: such time is INDETERMINATE, "
                  "never server-backed", file=out)
        if c["dl picks"] == 0:
            failures.append(f"fair server: the probes recorded no pick on CPU {ev.cpu}: their evidence is missing")
    t = ev.dl_timers
    print("    deadline-class timers on CPU " + str(ev.cpu) + ": " + ", ".join(f"{k} {v}" for k, v in t.items()),
          file=out)
    nf = [x for x in ev.null_faults if not ev.window or ev.window[0] <= x <= ev.window[1]]
    print(f"    kernel page faults at NULL-page addresses on CPU {ev.cpu} during the run: {len(nf)}", file=out)
    if nf and ev.layout is not None:
        failures.append(f"fair server: {len(nf)} kernel page faults at NULL-page addresses on CPU {ev.cpu}: "
                        "a read through a NULL pointer (a probe's) perturbed the run")
    long = [w for w in ws if w["class"] is not None]
    if long:
        n = {k: sum(1 for w in long if w["class"] == k)
             for k in ("server-backed", "mixed", "other mechanism", "INDETERMINATE")}
        print(f"    waits of 50 us or more: {len(long)}: " + ", ".join(f"{k} {v}" for k, v in n.items()),
              file=out)
        for w in sorted(long, key=lambda x: -x["ns"])[:top]:
            by = ", ".join(f"{k} {v / 1e3:.1f}" for k, v in w["by"].items() if v)
            wu = f", sched_wakeup +{(w['wakeup'] - w['ts']) / 1e3:.1f}" if w["wakeup"] is not None else ""
            print(f"      {w['ts'] / 1e9:.6f} s: {w['ns'] / 1e3:.1f} us from sched_waking (by {w['waker']} on "
                  f"CPU {w['waker_cpu']}{wu}) to switch-in: {by} us -> {w['class']}; ran: "
                  f"{', '.join(w['ran']) or 'none'}", file=out)


# --- the controls ------------------------------------------------------------------

REPORT_LINE = re.compile(r"^\s*(?P<comm>.*?)-(?P<pid>\d+)\s+\[(?P<cpu>\d+)\]\s+(?:\S+\s+)?"
                         r"(?P<s>\d+)\.(?P<ns>\d{9}):\s+(?P<ev>\w+):\s+(?P<rest>.*)$")
CTL_HEAD = re.compile(r"^# fair_server_control cpu=(\d+) supervisor_cpu=(\d+)")
CTL_PHASE = re.compile(r"^phase (\S+) (\d+) (\d+)$")
CTL_FIFO = re.compile(r"^fifo start=(\S+) end=(\S+) exit=(-?\d+) killed=(\w+) pid=(\d+)$")
CTL_NORMAL = re.compile(r"^normal (\S+) pid=(\d+) exit=(-?\d+) samples=(\d+)$")
CTL_SAMPLES = re.compile(r"^samples (\S+)(?: (.*))?$")


def parse_controls(text: str) -> dict:
    ctl: dict = {"cpu": None, "supervisor": None, "phases": {}, "fifo": None, "normal": {}}
    for line in text.splitlines():
        if m := CTL_HEAD.match(line):
            ctl["cpu"], ctl["supervisor"] = int(m[1]), int(m[2])
        elif m := CTL_PHASE.match(line):
            ctl["phases"][m[1]] = (int(m[2]), int(m[3]))
        elif m := CTL_FIFO.match(line):
            ctl["fifo"] = {"start": num(m[1]), "end": num(m[2]), "exit": int(m[3]), "killed": m[4] == "yes",
                           "pid": int(m[5])}
        elif m := CTL_NORMAL.match(line):
            ctl["normal"].setdefault(m[1], {})
            ctl["normal"][m[1]].update(pid=int(m[2]), exit=int(m[3]))
        elif m := CTL_SAMPLES.match(line):
            ctl["normal"].setdefault(m[1], {})["samples"] = [int(t) for t in (m[2] or "").split()]
    if ctl["cpu"] is None:
        raise ValueError("not a fair_server_control output")
    return ctl


def flags_text(ev: FairServerEvents, bits: int | None) -> str:
    if bits is None or ev.layout is None:
        return "unknown"
    on = [f.removeprefix("dl_") for f in FLAGS if ev.flag(bits, f)]
    return ",".join(on) or "none"


def running_mode(ev: FairServerEvents, bits: int) -> bool:
    """The server in the mode in which it can dispatch: active and, when
    deferred, deferred-running (throttled or not: a throttled running
    server dispatches again once replenished)."""
    if not ev.flag(bits, "dl_server_active"):
        return False
    return bool(ev.flag(bits, "dl_defer_running")) if ev.flag(bits, "dl_defer") else not ev.flag(bits, "dl_throttled")


CHANGING_CALLS = ("rtfs_srv_start", "rtfs_srv_stop", "rtfs_srv_timer")  # entry states, before a possible change


def preconditions(ev: FairServerEvents, after: int, before: int, misses: int | None) -> tuple[bool, str]:
    """A negative control's preconditions: the CPU's fair server's state as
    read at a deadline-class pick between the previous test and the control,
    after the last call there that could change it."""
    if misses is None:
        return False, "the probes' misses are unknown: a selection could be missing"
    if misses:
        return False, f"the probes missed {misses} hits: a selection could be missing"
    picked = [t for t, _, _ in ev.selection_log if after <= t <= before]
    if picked:
        return False, f"the server selected a task {len(picked)} times between the previous test and this control"
    gap = [(t, e, b) for t, e, b in ev.server_states if after <= t <= before]
    last_change = max((t for t, e, _ in gap if e in CHANGING_CALLS), default=after)
    read = [(t, b) for t, e, b in gap if e == "rtfs_dl_pick" and t > last_change and b is not None]
    if not read:
        return False, ("no state of the server was read at a deadline-class pick after its last start, stop or "
                       "timer call before this control")
    t, bits = read[-1]
    if running_mode(ev, bits):
        return False, f"the server was last read in its running mode ({flags_text(ev, bits)})"
    return True, (f"the server was read {(before - t) / 1e6:.1f} ms before this control, after its last start, stop "
                  f"or timer call ({flags_text(ev, bits)}): not running, and it selected nothing since the "
                  "previous test")


def judge_controls(ev: FairServerEvents, ctl: dict, misses: int | None) -> list[tuple[str, str, str]]:
    out: list[tuple[str, str, str]] = []
    f = ctl["fifo"]
    pos = ctl["normal"].get("positive", {})
    if "positive" not in ctl["phases"] or not f or f["start"] is None or f["end"] is None or f["exit"] != 0 \
            or f["killed"]:
        out.append(("positive", "FAIL", f"the FIFO loop did not run to its deadline ({f})"))
    else:
        a, b = f["start"], f["end"]
        rt = any(g[2] == f["pid"] and g[4] == "higher class" for g in ev.segments)
        prog = sum(1 for t in pos.get("samples", []) if a <= t <= b)
        disp = sum(1 for t, pid, valid in ev.dispatch_log if valid and pid == pos.get("pid") and a <= t <= b)
        if not rt:
            out.append(("positive", "FAIL", f"the loop (pid {f['pid']}) did not run at a real-time priority on "
                                            f"CPU {ev.cpu} in the trace"))
        elif prog == 0:
            out.append(("positive", "FAIL", "the normal task made no progress while the loop ran"))
        elif disp == 0:
            out.append(("positive", "FAIL", "no server-backed dispatch of the normal task while the loop ran"))
        else:
            out.append(("positive", "PASS", f"while the loop ran ({(b - a) / 1e9:.2f} s) the normal task progressed "
                                            f"({prog} samples) and was dispatched by the fair server {disp} times"))
    for name, prev in (("negative-normal", "positive"), ("negative-idle", "negative-normal")):
        if name not in ctl["phases"] or prev not in ctl["phases"]:
            out.append((name, "INCONCLUSIVE", "the phase is missing from the controls' output"))
            continue
        xs, xe = ctl["phases"][name]
        ok, why = preconditions(ev, ctl["phases"][prev][1], xs, misses)
        sel_in = [s for s in ev.selection_log if xs <= s[0] <= xe]
        disp_in = [d for d in ev.dispatch_log if xs <= d[0] <= xe and d[2]]
        if not ok:
            out.append((name, "INCONCLUSIVE", f"preconditions not established: {why}"))
        elif name == "negative-normal":
            n = ctl["normal"].get(name, {})
            prog = sum(1 for t in n.get("samples", []) if xs <= t <= xe)
            if disp_in:
                out.append((name, "FAIL", f"{len(disp_in)} server-backed dispatches while the normal task ran alone"))
            elif prog == 0:
                out.append((name, "INCONCLUSIVE", "the normal task did not run"))
            else:
                out.append((name, "PASS", f"{why}; no server-backed dispatch while the normal task ran alone "
                                          f"({prog} samples)"))
        elif sel_in:
            out.append((name, "FAIL", f"{len(sel_in)} fair-server selections on the idle CPU"))
        else:
            out.append((name, "PASS", f"{why}; no fair-server selection on the idle CPU"))
    return out


def controls_main(args) -> int:
    try:
        ctl = parse_controls(open(args.controls).read())
        record = open(args.record).read().splitlines()
    except (OSError, ValueError) as e:
        print(f"fair_server: {e}", file=sys.stderr)
        return 2
    lay = parse_layout(record)
    if lay is None:
        print("fair_server: the record has no probe layout", file=sys.stderr)
        return 2
    misses = None
    for line in record:
        m = re.match(r"^fair-server probe hits during the controls: .*; misses during the controls: (\d+)$", line)
        if m:
            misses = int(m[1])
    ev = FairServerEvents(ctl["cpu"], lay, None)
    proc = None
    if args.report:
        lines = open(args.report, errors="replace")
    else:
        proc = subprocess.Popen([os.environ.get("TRACE_CMD", "trace-cmd"), "report", "-t", "-i", args.trace_dat],
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, errors="replace")
        lines = proc.stdout
    last = 0
    for line in lines:
        m = REPORT_LINE.match(line)
        if not m:
            continue
        ts = int(m["s"]) * 1_000_000_000 + int(m["ns"])
        last = ts
        e = m["ev"]
        if e in FEED_EVENTS or e.startswith("rtfs_"):
            ev.feed(ts, int(m["cpu"]), int(m["pid"]), m["comm"], e, m["rest"])
    if proc is not None and proc.wait() != 0:
        print(f"fair_server: trace-cmd report -i {args.trace_dat} failed", file=sys.stderr)
        return 2
    ev.finish(last)
    verdicts = judge_controls(ev, ctl, misses)
    print(f"fair-server controls on CPU {ctl['cpu']} (supervisor on CPU {ctl['supervisor']}; probe misses "
          f"{'unknown' if misses is None else misses}; {ev.probe_events} probe events, "
          f"{len(ev.selection_log)} fair-server selections):")
    for name, verdict, why in verdicts:
        print(f"  {name}: {verdict} ({why})")
    failed = any(v == "FAIL" for _, v, _ in verdicts)
    print(f"fair-server controls: {'FAIL' if failed else 'PASS'}")
    return 1 if failed else 0


def btf_raw(path: str | None) -> list[str]:
    if path:
        with open(path) as f:
            return f.read().splitlines()
    tool = os.environ.get("BPFTOOL", "bpftool")
    r = subprocess.run([tool, "btf", "dump", "file", "/sys/kernel/btf/vmlinux", "format", "raw"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise LayoutError(f"{tool} btf dump failed: {r.stderr.strip()}")
    return r.stdout.splitlines()


def main() -> int:
    p = argparse.ArgumentParser(description="The fair server's probes: layout and definitions.")
    sub = p.add_subparsers(dest="cmd", required=True)
    lp = sub.add_parser("layout")
    lp.add_argument("--btf-raw", help="a bpftool raw BTF dump (default: read the kernel's)")
    dp = sub.add_parser("definitions")
    dp.add_argument("--group", default=GROUP)
    dp.add_argument("--btf-raw", help="a bpftool raw BTF dump (default: read the kernel's)")
    cp = sub.add_parser("controls")
    cp.add_argument("--controls", required=True, help="tools/fair_server_control.py's output")
    cp.add_argument("--record", required=True, help="the controls' run record (layout, misses)")
    src = cp.add_mutually_exclusive_group(required=True)
    src.add_argument("--trace-dat")
    src.add_argument("--report", help="trace-cmd report -t text instead of the trace")
    args = p.parse_args()
    if args.cmd == "controls":
        return controls_main(args)
    try:
        lay = layout_from_btf_raw(btf_raw(args.btf_raw or os.environ.get("FAIR_SERVER_BTF_RAW")))
    except (OSError, LayoutError) as e:
        print(f"fair_server: {e}", file=sys.stderr)
        return 2
    if args.cmd == "layout":
        print(layout_text(lay))
    else:
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", args.group):
            print(f"fair_server: bad group {args.group!r}", file=sys.stderr)
            return 2
        print("\n".join(definitions(lay, args.group)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
