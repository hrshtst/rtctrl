#!/usr/bin/env python3
"""Regressions for trace_correlate.py, run under ctest.

A synthetic three-cycle probe run at 500 Hz and its trace, written in
`trace-cmd report -t` form: the timer wakes, the adapter's bulk transfers
and the replies' USB completions with the completion work, the TTY work
and the worker's resumption. The clean run passes and its receive path is
pinned (a reply split across two completions; another TTY's
flush_to_ldisc starting first; a waking after the waker's work ended), and
so is the adapter's completion cadence. A read whose completion work
queued no TTY work (already pending) reports its wait apart, never as a
queue wait. Placement: the interrupt's CPU, the completion work's context,
the TTY worker's CPU, its waking, its first run after it, and its time
running, preempted and asleep until the work (a variant runs 900 us and is
preempted for 100 us first). A profiled variant adds the worker's sleep
calls and timer arming, a late sleep with a page fault and perf samples
(sampled instruction pointers and call chains; another thread's sample is
ignored) before it; missing, duplicate and inconsistent sleep evidence
fails the run, while a re-armed timer and another timer do not. A
timer-mode variant uses exactly the events the session's timer mode
records (--timer-events lists them for its smoke test, which checks them
against the session's own), and fails without any of its evidence.
Every association and alignment failure is then provoked once and must
fail the run, and the documented exception (a cycle that began without
sleeping) must not.
"""

import csv
import pathlib
import random
import subprocess
import sys
import tempfile

ORIGIN = 1_000_000_000_000   # the probe's epoch on CLOCK_MONOTONIC, ns
TID = 4242
PERIOD_US = 2000.0
HEADER = ("cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,first_reply_ns,"
          "last_reply_ns,read_end_ns,limit_start_ns,bus_write_start_ns,write_return_ns,"
          "drain_return_ns,ack_return_ns,completion_upper_ns,cycle_end_ns,read_ok,write_ok,"
          "ack_ok,skipped_periods,read_len").split(",")
# Data completions per cycle, us after its read_start: (time, URB length).
# 120 reply bytes (8 + 8 * (10 + 4)) plus two status bytes per packet.
REPLIES = [[(600.0, 122)], [(300.0, 66), (1300.0, 58)], [(700.0, 122)]]


# A session record's fair-server probe layout (this kernel's, as
# tools/fair_server.py reads it from the BTF).
FS_LAYOUT = ("fair-server probe layout: rq.ext_server=2952 rq.fair_server=3192 rq.cpu=3688 se.runtime=64 "
             "se.deadline=72 se.rq=216 task.prio=108 task.policy=1356 task.pid=2800 se.flags_word=84 "
             "bit.dl_throttled=0 bit.dl_non_contending=2 bit.dl_server=4 bit.dl_server_active=5 bit.dl_defer=6 "
             "bit.dl_defer_armed=7 bit.dl_defer_running=8 bit.dl_defer_idle=9")


def ns(us: float) -> int:
    return round(us * 1000)


TSC_HZ, TSC0 = 3_187_200_000, 9_000_000_000_000  # the timer variant's TSC at ORIGIN


def tsc_of(mono: int) -> int:
    return TSC0 + round((mono - ORIGIN) * TSC_HZ / 1e9)


class Run:
    def __init__(self, **opt) -> None:
        self.opt = opt
        self.perf: list[tuple] = []
        self.stacks: list[tuple] = []
        self.events: list[tuple[int, int, str]] = []
        self.rows: list[dict] = []
        self.names: set[str] = set()
        self.build()

    def ev(self, us: float, comm: str, pid: int, cpu: int, flags: str, event: str, rest: str) -> None:
        self.names.add(event)
        t = ORIGIN + ns(us)
        s, n = divmod(t, 1_000_000_000)
        line = f"{comm:>16}-{pid:<7} [{cpu:03d}] {flags} {s}.{n:09d}: {event}: {rest}"
        self.events.append((t, len(self.events), line))

    def worker_out(self, us: float, state: str) -> None:
        self.ev(us, "x7_bus_timing", TID, 3, "d..2.", "sched_switch",
                f"x7_bus_timing:{TID} [19] {state} ==> swapper/3:0 [120]")

    def worker_in(self, us: float) -> None:
        self.ev(us, "<idle>", 0, 3, "d..2.", "sched_switch",
                f"swapper/3:0 [120] R ==> x7_bus_timing:{TID} [19]")

    def timer_wake(self, sched: float, now_offset: float = 0.4) -> None:
        if self.opt.get("timer"):
            self.ev(sched + 0.45, "<idle>", 0, 3, "d.h1.", "local_timer_entry", "vector=236")
        self.ev(sched + 0.5, "<idle>", 0, 3, "d.h1.", "hrtimer_expire_entry",
                f"hrtimer=0xffffcdd1a193fcd8 now={ORIGIN + ns(sched + now_offset)} "
                "function=hrtimer_wakeup/0x0")
        self.ev(sched + 0.6, "<idle>", 0, 3, "d.h3.", "sched_waking",
                f"comm=x7_bus_timing pid={TID} prio=19 target_cpu=003")
        self.ev(sched + 0.7, "<idle>", 0, 3, "d.h1.", "hrtimer_expire_exit", "hrtimer=0xffffcdd1a193fcd8")

    def urb(self, us: float, pid: int, comm: str, cpu: int, kind: str, ep: str, urb: str,
            actual: int, length: int) -> None:
        if kind == "giveback":  # in the controller's interrupt handler
            self.ev(us - 0.5, comm, pid, cpu, "d.h1.", "irq_handler_entry", "irq=125 name=xhci_hcd")
            self.ev(us + 0.5, comm, pid, cpu, "d.h1.", "irq_handler_exit", "irq=125 ret=handled")
        self.ev(us, comm, pid, cpu, "....." if kind == "enqueue" else "d.h2.", f"xhci_urb_{kind}",
                f"1-9 {ep}-bulk: urb {urb} pipe 3221292032 slot 5 length {actual}/{length} "
                "sgs 0/0 stream 0 flags 0")

    def completion(self, c: float, actual: int, waking_in_barrier: bool, queue: bool = True,
                   busy: tuple | None = None) -> float:
        """One data completion and its path to the worker; returns the
        probe's stamp after the worker read the data."""
        self.urb(c, 0, "<idle>", 7, "giveback", "ep1in", "0xin", actual, 512)
        self.ev(c + 0.1, "<idle>", 0, 7, "d.h3.", "workqueue_queue_work",
                "work struct=0xbh function=usb_giveback_urb_bh workqueue=events_bh req_cpu=8192 cpu=7")
        self.ev(c + 2.0, "<idle>", 0, 7, "..s1.", "workqueue_execute_start",
                "work struct 0xbh: function usb_giveback_urb_bh")
        if queue:
            self.ev(c + 3.0, "<idle>", 0, 7, "dNs2.", "workqueue_queue_work",
                    "work struct=0xtty function=flush_to_ldisc workqueue=events_unbound req_cpu=8192 cpu=-1")
        self.ev(c + 3.5, "<idle>", 0, 7, ".Ns1.", "workqueue_execute_end",
                "work struct 0xbh: function usb_giveback_urb_bh")
        self.worker_out(c + 3.8, "D")
        # Another TTY's work starts first, on another kernel worker.
        self.ev(c + 4.2, "kworker/u128:2", 888, 9, ".....", "workqueue_execute_start",
                "work struct 0xother: function flush_to_ldisc")
        self.ev(c + 4.8, "kworker/u128:2", 888, 9, ".....", "workqueue_execute_end",
                "work struct 0xother: function flush_to_ldisc")
        self.ev(c + 3.6, "<idle>", 0, 7, "dNs2.", "sched_waking",
                "comm=kworker/u128:1 pid=777 prio=120 target_cpu=008")
        self.ev(c + 4.0, "<idle>", 0, 8, "d..2.", "sched_switch",
                "swapper/8:0 [120] R ==> kworker/u128:1:777 [120]")
        d = 0.0
        if busy:  # the worker runs, is preempted, and runs again first
            run_us, preempted_us = busy
            self.ev(c + 4.0 + run_us, "kworker/u128:1", 777, 8, "d..2.", "sched_switch",
                    "kworker/u128:1:777 [120] R ==> competing:999 [120]")
            self.ev(c + 4.0 + run_us + preempted_us, "competing", 999, 8, "d..2.", "sched_switch",
                    "competing:999 [120] R ==> kworker/u128:1:777 [120]")
            d = run_us + preempted_us
        self.ev(c + d + 4.5, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_start",
                "work struct 0xtty: function flush_to_ldisc")
        self.ev(c + d + 5.0, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_end",
                "work struct 0xtty: function flush_to_ldisc")
        self.ev(c + d + 5.1, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_start",
                "work struct 0xbar: function wq_barrier_func")
        if waking_in_barrier:
            self.ev(c + d + 5.2, "kworker/u128:1", 777, 8, "d..3.", "sched_waking",
                    f"comm=x7_bus_timing pid={TID} prio=19 target_cpu=003")
            self.ev(c + d + 5.3, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_end",
                    "work struct 0xbar: function wq_barrier_func")
        else:
            self.ev(c + d + 5.15, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_end",
                    "work struct 0xbar: function wq_barrier_func")
            self.ev(c + d + 5.2, "kworker/u128:1", 777, 8, "d..3.", "sched_waking",
                    f"comm=x7_bus_timing pid={TID} prio=19 target_cpu=003")
        self.worker_in(c + d + 6.0)
        return c + d + 6.5

    def build(self) -> None:
        o = self.opt
        prev_end = None
        if o.get("timer"):  # the real-time CPU's other timer interrupts, irregular
            rnd = random.Random(7)
            us = -400.0
            while us < len(REPLIES) * PERIOD_US + 400.0:
                self.ev(us, "<idle>", 0, 3, "d.h1.", "local_timer_entry", "vector=236")
                us += rnd.uniform(60.0, 140.0)
        for i, replies in enumerate(REPLIES):
            sched = i * PERIOD_US
            if i == 2 and o.get("began_awake"):
                sched = (prev_end or 0.0) + 0.05
            wake = sched + 1.3
            if i > 0:
                if not (i == 2 and (o.get("began_awake") or o.get("missing"))):
                    self.timer_wake(sched, now_offset=-1.0 if (i == 1 and o.get("negative_b")) else 0.4)
                    if i == 1 and o.get("duplicate"):
                        self.timer_wake(sched - 50.0)
                    self.worker_in(sched + 1.0)
            read_start = wake + 1.0
            submit = read_start + 1.0
            self.urb(submit, TID, "x7_bus_timing", 3, "enqueue", "ep2out", "0xout", 0, 22)
            self.urb(read_start + 12.0, 0, "<idle>", 7, "giveback", "ep2out", "0xout", 22, 22)
            self.urb(read_start + 100.0, 0, "<idle>", 7, "giveback", "ep1in", "0xin", 2, 512)
            stamps = []
            for t, actual in replies:
                if i == 0 and o.get("short"):
                    actual -= 20
                stamps.append(self.completion(read_start + t, actual, waking_in_barrier=(i != 2),
                                              queue=not (i == 0 and o.get("no_queue")),
                                              busy=(900.0, 100.0) if (i == 0 and o.get("preempted")) else None))
            read_end = stamps[-1] + 0.5
            if i == 0 and o.get("stray"):
                self.urb(read_end + 200.0, 0, "<idle>", 7, "giveback", "ep1in", "0xin", 12, 512)
            if i == 1 and o.get("extra"):
                self.timer_wake(read_start + 400.0)
            bws = read_end + 0.1
            ws = bws + (5.0 if (i == 0 and o.get("late_submit")) else 1.0)
            self.urb(ws, TID, "x7_bus_timing", 3, "enqueue", "ep2out", "0xout", 0, 38)
            self.urb(bws + 12.0, 0, "<idle>", 7, "giveback", "ep2out", "0xout", 38, 38)
            write_return = bws + 3.0
            cycle_end = write_return + 0.1
            # The sleep call (late after cycle 2 in the late_sleep variant).
            call = cycle_end + (500.0 if (i == 1 and o.get("late_sleep")) else 0.3)
            if (o.get("profiled") or o.get("timer")) and i < len(REPLIES) - 1:
                first = i == 0  # the sleep before cycle 2: the variants' target
                nxt_us = (i + 1) * PERIOD_US
                if o.get("timer"):  # raw_syscalls, filtered to CPU 3 (every task's)
                    self.ev(call - 0.01, "x7_bus_timing", TID, 3, ".....", "sys_enter",
                            "NR 230 (1, 0, 7ffd0000, 0, 0, 0)")
                    self.ev(nxt_us + 1.05, "x7_bus_timing", TID, 3, ".....", "sys_exit", "NR 230 = 0")
                if not (first and o.get("drop_call")) and not o.get("drop_named"):
                    self.ev(call, "x7_bus_timing", TID, 3, ".....", "sys_enter_clock_nanosleep",
                            "which_clock: 0x00000001, flags: 0x00000000, rqtp: 0x7ffd0000, rmtp: 0x00000000")
                if first and o.get("dup_call"):
                    self.ev(call + 0.05, "x7_bus_timing", TID, 3, ".....", "sys_enter_clock_nanosleep",
                            "which_clock: 0x00000001, flags: 0x00000000, rqtp: 0x7ffd0000, rmtp: 0x00000000")
                nxt = ORIGIN + ns((i + 1) * PERIOD_US)

                def arm(us, expires, addr="0xffffcdd1a193fcd8"):
                    self.ev(us, "x7_bus_timing", TID, 3, "d..1.", "hrtimer_start",
                            f"hrtimer={addr} function=hrtimer_wakeup expires={expires} softexpires={expires} mode=REL")
                if first and o.get("rearm"):
                    arm(call + 0.06, nxt + ns(2000.0))
                    self.ev(call + 0.08, "x7_bus_timing", TID, 3, "d..1.", "hrtimer_cancel",
                            "hrtimer=0xffffcdd1a193fcd8")
                if first and o.get("other_timer"):
                    arm(call + 0.07, nxt + ns(500.0), addr="0xffffcdd1a193f000")
                if not (first and o.get("drop_arm")):
                    arm(call + 0.1, nxt + (ns(3000.0) if (first and o.get("far_expiry")) else 0))
                if o.get("timer") and not o.get("no_writes"):  # the deadline written for the worker's expiry
                    self.ev(call + 0.12, "x7_bus_timing", TID, 3, "d..1.", "write_msr", f"6e0, value {tsc_of(nxt):x}")
                if i == 1 and o.get("late_sleep"):
                    self.ev(cycle_end + 100.0, "x7_bus_timing", TID, 3, ".....", "page_fault_user",
                            "address=0x7f0000001000 ip=0x55d000001234 error_code=0x6")
                    self.perf = [(cycle_end + 200.0, TID, "hw::accumulateCycleTiming", "x7_bus_timing"),
                                 (cycle_end + 250.0, 999, "other_thread_fn", "x7_bus_timing"),
                                 (cycle_end + 300.0, TID, "memcpy", "libc.so.6")]
                    self.stacks = [(cycle_end + 200.0, TID, ["hw::accumulateCycleTiming", "Probe::run", "start_thread"]),
                                   (cycle_end + 250.0, 999, ["other_thread_fn"]),
                                   (cycle_end + 300.0, TID, ["memcpy", "hw::accumulateCycleTiming", "Probe::run"])]
            if i < len(REPLIES) - 1 and not (i == 1 and o.get("began_awake")):
                self.worker_out(call + 0.2, "S")
            prev_end = cycle_end
            row = {
                "cycle": i + 1, "scheduled_wake_ns": ns(sched), "wake_ns": ns(wake),
                "read_start_ns": ns(read_start), "instr_return_ns": ns(read_start + 8.0),
                "first_reply_ns": ns(stamps[0]), "last_reply_ns": ns(stamps[-1]),
                "read_end_ns": ns(read_end), "limit_start_ns": ns(read_end),
                "bus_write_start_ns": ns(bws), "write_return_ns": ns(write_return),
                "drain_return_ns": -1, "ack_return_ns": -1, "completion_upper_ns": -1,
                "cycle_end_ns": ns(cycle_end), "read_ok": 1, "write_ok": 1, "ack_ok": 0,
                "skipped_periods": 1 if (i == 1 and o.get("overran")) else 0, "read_len": 10,
            }
            self.rows.append(row)

    def write(self, root: pathlib.Path, name: str) -> pathlib.Path:
        path = root / f"{name}.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=HEADER)
            w.writeheader()
            w.writerows(self.rows)
        kind = self.opt.get("kind", "production-traffic")
        pathlib.Path(str(path) + ".summary.toml").write_text(
            f'format = "rtctrl-bus-timing"\nkind = "{kind}"\nrate_hz = 500\n'
            'read_instruction = "fast_sync"\n\n[result]\n'
            f"worker_tid = {TID}\nmonotonic_origin_ns = {ORIGIN}\nlate_wake_stop = false\n"
            "late_wake_s = 0\nlate_wake_lateness_s = 0\n\n[budget]\n"
            "read_instruction_bytes = 22\nwrite_bytes = 38\n")
        if self.perf:
            lines = []
            for us, tid, sym, dso in self.perf:
                s, n = divmod(ORIGIN + ns(us), 1_000_000_000)
                lines.append(f"   x7_bus_timing {tid} [003] {s}.{n:09d}:  55d000001234 {sym} ({dso})")
            pathlib.Path(str(path) + ".perf.txt").write_text("\n".join(lines) + "\n")
        if self.stacks:
            lines = []
            for us, tid, frames in self.stacks:
                s, n = divmod(ORIGIN + ns(us), 1_000_000_000)
                lines.append(f"   x7_bus_timing {tid} [003] {s}.{n:09d}:")
                lines += [f"\t55d000001234 {f} (x7_bus_timing)" for f in frames]
                lines.append("")
            pathlib.Path(str(path) + ".perf.stacks.txt").write_text("\n".join(lines) + "\n")
        if self.opt.get("timer") or self.opt.get("timer_record"):
            smi_before = ORIGIN - 2_000_000  # perf stat started 2 ms before the origin
            pathlib.Path(str(path) + ".run.txt").write_text(
                "mode: timer\nscheduling: --rt-priority 80 --rt-cpu 3 (rtprio limit 90)\n"
                f"trace overruns: {self.opt.get('trace_overruns', '0')} (trace-cmd report --stat)\n"
                "tsc instance overruns: 0\nsession pid: 1\n"
                "clock pair before: monotonic_before_ns 1 realtime_ns 2 monotonic_after_ns 3 "
                "realtime_minus_monotonic_ns 1700000000000000000 bracket_ns 40\n"
                "clock offsets: boottime_minus_monotonic_ns 0 tai_minus_monotonic_ns 1700000037000000000 "
                "bracket_ns 40\n"
                f"smi anchor monotonic_ns: {smi_before} {smi_before + 10_100_000} (before perf stat started; "
                "when its first interval was seen)\n"
                + (FS_LAYOUT + "\nfair-server probe hits during the run: rtfs_dl_pick 0; misses during the run: 0\n"
                   if self.opt.get("fs_layout") else ""))
            if not self.opt.get("no_smi"):
                pathlib.Path(str(path) + ".smi.csv").write_text(
                    "# started on Sat Nov  7 12:00:00 2026\n\n"
                    "     0.010000000,0,,msr/smi/,10000000,100.00,,\n"
                    "     0.020000000,0,,msr/smi/,10000000,100.00,,\n")
        if self.opt.get("timer") and not self.opt.get("no_instance"):
            marks = sorted(e[0] for e in self.events if ": local_timer_entry:" in e[2])
            if self.opt.get("short_instance"):  # an instance that stopped halfway through the run
                marks = [m for m in marks if m < ORIGIN + ns(len(REPLIES) * PERIOD_US / 2)]
            lines = [f"# entries-in-buffer/entries-written: {len(marks)}/{len(marks)}   #P:32"]
            lines += [f"          <idle>-0       [003] d.h1. {tsc_of(m)}: local_timer_entry: vector=236"
                      for m in marks]
            pathlib.Path(str(path) + ".tsc.txt").write_text("\n".join(lines) + "\n")
            pairs = ["# tsc_mono_pairs cpu=1 interval_ms=10"]
            for k in range(40):
                m = marks[0] + (marks[-1] - marks[0]) * (k + 0.5) / 40
                pairs.append(f"{round(m) - 20},{tsc_of(round(m))},{round(m) + 20},1")
            # The real-time CPU's windows, before and after the run.
            for m in ([] if self.opt.get("no_rt_pairs") else [ORIGIN - ns(200.0), ORIGIN + ns(6200.0)]):
                pairs.append(f"{m - 20},{tsc_of(m)},{m + 20},3")
            pathlib.Path(str(path) + ".tsc-pairs.csv").write_text("\n".join(pairs) + "\n")
        if self.opt.get("profiled_record"):
            pathlib.Path(str(path) + ".run.txt").write_text(
                "mode: profiled\nperf stats: SAMPLE 0, LOST 0, LOST_SAMPLES 0, THROTTLE 0 "
                "(perf report --stats --no-skip-empty, exit 0)\n")
        report = root / f"{name}.report.txt"
        report.write_text("cpus=32\n" + "\n".join(line for _, _, line in sorted(self.events)) + "\n")
        return path


def correlate(root: pathlib.Path, name: str, **opt) -> tuple[int, str, pathlib.Path]:
    path = Run(**opt).write(root, name)
    seg = root / f"{name}.segments.csv"
    p = subprocess.run([sys.executable, "tools/trace_correlate.py", str(path), "--report",
                        str(root / f"{name}.report.txt"), "--adapter", "1-9",
                        "--segments-csv", str(seg)], capture_output=True, text=True)
    return p.returncode, p.stdout + p.stderr, seg


def expect(cond: bool, what: str, out: str) -> None:
    if not cond:
        print(f"FAILED: {what}\n{out}")
        sys.exit(1)


def main() -> int:
    if sys.argv[1:] == ["--timer-events"]:
        # The event names a timer-mode run's fixture uses, for the session's
        # smoke test: each must be one the timer mode records.
        print(" ".join(sorted(Run(timer=True).names)))
        return 0
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)

        code, out, seg = correlate(root, "clean")
        expect(code == 0, "the clean run passes", out)
        # Each read's instruction against the adapter's delivery cadence: the
        # split reply's instruction came late in its interval.
        expect("host OUT-completion phase (each instruction's OUT completion between the host's bulk IN completions "
               "around it, a retrospective association; 2 reads, 1 split):" in out and "    0.9-1.0       1   100%" in out
               and "    0.8-0.9       1     0%" in out, "the host OUT-completion phase", out)
        expect("cycles whose host work exceeds 0.7 T (1400 us): 0;" in out, "no cycle over 0.7 T", out)
        expect("timer wakes: 2 in the run for 2 sleeps: 2 paired, 0 cycles began without sleeping, "
               "0 missing, 0 duplicate, 0 extra" in out, "both sleeps paired", out)
        expect("bus transfers: 6 OUT submissions for 6 probe transfers (0 submitted outside the worker)"
               in out, "six transfers paired", out)
        expect("replies: 3 reads checked, 0 not adding up, 0 data completions outside a read" in out,
               "the replies add up", out)
        expect("1 of 3 replies arrived in more than one completion" in out, "one split reply", out)
        # Attribution through execute_end: the waking after the barrier's end
        # belongs to no work.
        expect("the worker's wakers' work: - 1, wq_barrier_func 2" in out, "the wakers' work", out)
        expect("association and alignment: PASS" in out, "PASS reported", out)
        # Three status-only completions and four with data, within the run.
        expect("adapter bulk IN completions in the run: 7 over 0.004600 s" in out, "the cadence", out)
        expect("latest 3 wakes" in out and "cycle      1 wake     1.3: the first cycle (begins at once)" in out,
               "the first cycle listed as such", out)
        expect("0 reads whose USB completion work queued no TTY work" in out, "every TTY work queued", out)
        # Placement: the interrupt's CPU, the completion work's context, the
        # TTY worker's CPU and its waking.
        expect("xHCI interrupt CPUs: 7: 3" in out and "USB completion work: at interrupt exit: 3" in out,
               "the interrupt and its completion work", out)
        expect("CPU  8:      3 reads" in out, "the TTY work's CPU", out)
        expect("its kernel worker was woken after the queue in 3 reads (first run on another CPU than the "
               "waking's target in 0) and already awake in 0" in out, "the TTY worker's wakings", out)
        expect("sleep timing: no sleep-call events (not a profiled run)" in out, "no sleep timing traced", out)
        # Every giveback but the last write's (after the run) in the run;
        # four data completions, each with its completion work.
        expect("12 xHCI interrupts (" in out and "handlers 0.012 ms in all), 4 USB completion works "
               "(0.006 ms in all)" in out, "the host's USB work", out)
        rows = list(csv.DictReader(open(seg)))
        expect(len(rows) == 3, "a segment row per read", out)
        r = rows[1]
        want = {"completions": "2", "tty_pid": "777", "tty_cpu": "8", "blocked": "D",
                "before_send_ns": "1000", "instruction_usb_ns": "11000",
                "to_first_completion_ns": "288000", "between_completions_ns": "1000000",
                "after_last_completion_ns": "6500", "completion_to_bh_ns": "2000",
                "bh_to_tty_queue_ns": "1000", "tty_queue_to_start_ns": "1500", "tty_run_ns": "500",
                "tty_end_to_last_reply_ns": "1500", "tty_end_to_waking_ns": "200",
                "waking_to_switch_in_ns": "800", "host_transaction_ns": "1307000",
                "kworker_prev": "swapper/8", "irq_cpu": "7", "bh_comm": "<idle>",
                "kworker_waker": "<idle>", "kworker_target": "8",
                "tty_queue_to_kworker_waking_ns": "600", "kworker_waking_to_first_run_ns": "400",
                "kworker_ran_before_tty_ns": "500", "kworker_preempted_before_tty_ns": "0",
                "kworker_slept_before_tty_ns": "0", "kworker_first_cpu": "8", "kworker_migrated": "0"}
        for k, v in want.items():
            expect(r[k] == v, f"split reply's {k} is {v} (got {r[k]})", out)

        # A profiled run: every sleep's call and timer arming, and a late
        # sleep with a page fault and the worker's perf samples before it.
        code, out, _ = correlate(root, "profiled", profiled=True, late_sleep=True)
        expect(code == 0, "the profiled run passes", out)
        expect("sleep timing: 2 sleeps; 2 with their sleep call and timer, 0 without a sleep call, 0 with "
               "several, 0 without an arming of the expiring timer, 0 whose arming does not fit the expiry; "
               "0 re-armed, 0 with another timer armed; 1 page faults" in out, "the profiled evidence", out)
        expect("cycle_end_to_sleep_call            n=2      min     0.300" in out and "max   500.000" in out,
               "from the cycle-end stamp to the sleep call", out)
        expect("perf: 2 samples of the worker (sampled instruction pointers), 2 inside its on-CPU time in the "
               "trace; 2 with call chains" in out, "the samples' alignment and chains", out)
        expect("+500.0 after the cycle-end stamp, timer armed   +500.1 for +0.0 from the scheduled wake, "
               "expired +0.4; 1 page faults; sampled IPs 2: hw::accumulateCycleTiming 1, memcpy 1" in out,
               "the late sleep's call, arming, fault and sampled IPs", out)
        expect("call context (1 of 2 samples): hw::accumulateCycleTiming < Probe::run < start_thread" in out,
               "the late sleep's call context", out)
        # Missing, duplicate and inconsistent evidence fails; a re-armed
        # timer and another timer do not, and nothing is borrowed from the
        # next sleep.
        for opt, words in (("drop_arm", "1 sleeps without an arming of the expiring timer"),
                           ("drop_call", "1 sleeps without a sleep call"),
                           ("dup_call", "1 sleeps with several sleep calls"),
                           ("far_expiry", "1 sleeps whose timer arming does not fit the expiry")):
            code, out, _ = correlate(root, opt, profiled=True, **{opt: True})
            expect(code == 1 and f"sleep timing: {words}" in out, f"{opt} fails", out)
        code, out, _ = correlate(root, "rearm", profiled=True, rearm=True, other_timer=True)
        expect(code == 0 and "1 re-armed, 1 with another timer armed" in out
               and "for +0.0 from the scheduled wake" in out, "a re-armed timer and another timer", out)
        code, out, _ = correlate(root, "profiled_empty", profiled_record=True)
        expect(code == 1 and "a profiled run without sleep-call or timer-arming events" in out,
               "a profiled run without its events fails", out)
        # A timer-mode run, with exactly the events the session's timer mode
        # records (TIMER_EVENTS_RUN): the deadline timeline through the TSC
        # instance's anchors, validated by the sampler. Its missing evidence
        # fails the run: the events, the named sleep calls (raw system calls
        # alone do not show them), the deadline writes, the instance, the
        # sampler's windows on the real-time CPU, the SMI series and a valid
        # loss count.
        code, out, _ = correlate(root, "timer", timer=True)
        expect(code == 0 and "timer service (the real-time CPU's deadlines" in out
               and "sleep timing: 2 sleeps; 2 with their sleep call and timer" in out
               and "matched one-to-one" in out and "conversion uncertainty U:" in out
               and "sleeps: 2; on time 2" in out and "deadline writes: 2, 0 disarming" in out
               and "SMIs 0" in out and "MISSING" not in out, "the timer-mode run", out)
        # Its waits after waking (tools/fair_server.py), unattributed without
        # the probes; with the probes' layout in the record but no pick in the
        # trace, their evidence is missing.
        expect("waits after waking (CPU 3: the worker's sched_waking to its switch-in" in out
               and "n=2 " in out and "fair-server probes: not in this capture" in out,
               "the timer-mode run's waits", out)
        # A confined run's placement: observer and probe helper threads kept
        # off the excluded CPUs pass; one switched in on CPU 2 during the run
        # fails, and so does a probe helper there.
        for name, extra, ok in (("placed", [], True),
                                ("misplaced", [(1000.0, "obs-thread", 7771), (1050.0, None, 0)], False),
                                ("helper", [(1000.0, "x7_bus_timing", 4243), (1050.0, None, 0)], False)):
            r = Run(timer=True)
            prev = ("swapper/2", 0)
            for us, comm, pid in extra:
                nxt = (comm, pid) if comm else ("swapper/2", 0)
                r.ev(us, prev[0], prev[1], 2, "d..2.", "sched_switch",
                     f"{prev[0]}:{prev[1]} [120] S ==> {nxt[0]}:{nxt[1]} [120]")
                prev = nxt
            path = r.write(root, name)
            pathlib.Path(str(path) + ".observers.jsonl").write_text(
                '{"kind": "header", "exclude": [2, 3], "allowed": [0, 1], "version": 1}\n'
                '{"kind": "thread", "pid": 7770, "tid": 7771, "start": 5, "pstart": 5, "mask": [0, 1, 2, 3]}\n')
            p = subprocess.run([sys.executable, "tools/trace_correlate.py", str(path), "--report",
                                str(root / f"{name}.report.txt"), "--adapter", "1-9"], capture_output=True, text=True)
            out = p.stdout + p.stderr
            expect(("observer placement (confined, " in out) and ((p.returncode == 0) == ok)
                   and (ok or "ran on CPUs 2,3" in out), f"the placement check ({name})", out)
        code, out, _ = correlate(root, "timer_fs_empty", timer=True, fs_layout=True)
        expect(code == 1 and "fair server: the probes recorded no pick on CPU 3: their evidence is missing" in out
               and "probe misses during the run: 0" in out, "probes without a pick fail", out)
        for name, opt, words in (
                ("timer_bare", {"timer_record": True}, "the timer-mode run has no deadline writes on CPU 3"),
                ("timer_raw_only", {"timer": True, "drop_named": True}, "sleep timing: 2 sleeps without a sleep call"),
                ("timer_raw_only2", {"timer": True, "drop_named": True}, "the timer-mode run has no sleep calls"),
                ("timer_no_writes", {"timer": True, "no_writes": True},
                 "the timer-mode run has no deadline writes on CPU 3"),
                ("timer_noinst", {"timer": True, "no_instance": True}, "no TSC instance or sampler pairs"),
                ("timer_no_rt_pairs", {"timer": True, "no_rt_pairs": True},
                 "no sampler pairs on CPU 3 inside the anchors both before and after the run"),
                ("timer_no_smi", {"timer": True, "no_smi": True}, "timer service: no SMI count file"),
                ("timer_short_instance", {"timer": True, "short_instance": True},
                 "timer service: the conversion's anchors do not cover the run"),
                ("timer_bad_losses", {"timer": True, "trace_overruns": "unknown (stats unreadable)"},
                 "the record has no valid count of the trace's losses")):
            code, out, _ = correlate(root, name, **opt)
            expect(code == 1 and words in out, f"{name} fails ({words})", out)
        # A kernel worker that runs and is preempted before the TTY work:
        # its run-queue wait is up to its first run only.
        code, out, seg = correlate(root, "preempted", preempted=True)
        expect(code == 0, "the preempted worker's run passes", out)
        r = list(csv.DictReader(open(seg)))[0]
        want = {"kworker_waking_to_first_run_ns": "400", "kworker_ran_before_tty_ns": "900500",
                "kworker_preempted_before_tty_ns": "100000", "kworker_slept_before_tty_ns": "0"}
        for k, v in want.items():
            expect(r[k] == v, f"preempted: {k} is {v} (got {r[k]})", out)
        expect("largest part: kworker_ran_before_tty 1" in out, "the tail's largest part: running", out)

        code, out, _ = correlate(root, "awake", began_awake=True)
        expect(code == 0, "a cycle that began without sleeping is accepted", out)
        expect("1 paired, 1 cycles began without sleeping, 0 missing" in out, "counted as such", out)

        code, out, _ = correlate(root, "overran", overran=True)
        expect(code == 0, "an overrun is not an association failure", out)
        expect("overran cycles: 1; largest measured part: between USB completions 1" in out,
               "the overrun located", out)

        # Without a queue event the TTY work's queue time is unknown: its
        # wait from the completion work's start is reported apart.
        code, out, seg = correlate(root, "no_queue", no_queue=True)
        expect(code == 0, "a TTY work that was already pending is no failure", out)
        expect("1 reads whose USB completion work queued no TTY work" in out, "counted", out)
        r = list(csv.DictReader(open(seg)))[0]
        expect(r["tty_queue_to_start_ns"] == "" and r["bh_to_tty_start_no_queue_ns"] == "2500"
               and r["bh_to_tty_queue_ns"] == "", "reported apart, not as a queue wait", out)
        r = list(csv.DictReader(open(seg)))[1]
        expect(r["tty_queue_to_start_ns"] == "1500" and r["bh_to_tty_start_no_queue_ns"] == "",
               "the other reads keep their queue wait", out)

        failures = {
            "missing": "timer wakes: 1 sleeps without a wake",
            "duplicate": "timer wakes: 1 sleeps with more than one wake",
            "extra": "timer wakes: 1 wakes outside every sleep",
            "negative_b": "timer wakes: 1 negative B",
            "late_submit": "bus transfers: 1 submissions outside the probe's stamps",
            "short": "replies: 1 of 3 reads whose completions do not add up to the reply",
            "stray": "replies: 1 data completions outside any read (10 bytes)",
        }
        for opt, message in failures.items():
            code, out, _ = correlate(root, opt, **{opt: True})
            expect(code == 1, f"{opt} fails the run", out)
            expect("association and alignment: FAIL" in out and message in out, f"{opt}: {message}", out)

        code, out, _ = correlate(root, "calibration", kind="calibration")
        expect(code == 2, "a calibration run is refused", out)
    print("trace correlate regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
