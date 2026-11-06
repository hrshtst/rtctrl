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
queue wait.
Every association and alignment failure is then provoked once and must
fail the run, and the documented exception (a cycle that began without
sleeping) must not.
"""

import csv
import pathlib
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


def ns(us: float) -> int:
    return round(us * 1000)


class Run:
    def __init__(self, **opt) -> None:
        self.opt = opt
        self.events: list[tuple[int, int, str]] = []
        self.rows: list[dict] = []
        self.build()

    def ev(self, us: float, comm: str, pid: int, cpu: int, flags: str, event: str, rest: str) -> None:
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
        self.ev(sched + 0.5, "<idle>", 0, 3, "d.h1.", "hrtimer_expire_entry",
                f"hrtimer=0xffffcdd1a193fcd8 now={ORIGIN + ns(sched + now_offset)} "
                "function=hrtimer_wakeup/0x0")
        self.ev(sched + 0.6, "<idle>", 0, 3, "d.h3.", "sched_waking",
                f"comm=x7_bus_timing pid={TID} prio=19 target_cpu=003")
        self.ev(sched + 0.7, "<idle>", 0, 3, "d.h1.", "hrtimer_expire_exit", "hrtimer=0xffffcdd1a193fcd8")

    def urb(self, us: float, pid: int, comm: str, cpu: int, kind: str, ep: str, urb: str,
            actual: int, length: int) -> None:
        self.ev(us, comm, pid, cpu, "....." if kind == "enqueue" else "d.h2.", f"xhci_urb_{kind}",
                f"1-9 {ep}-bulk: urb {urb} pipe 3221292032 slot 5 length {actual}/{length} "
                "sgs 0/0 stream 0 flags 0")

    def completion(self, c: float, actual: int, waking_in_barrier: bool, queue: bool = True) -> float:
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
        self.ev(c + 4.0, "<idle>", 0, 8, "d..2.", "sched_switch",
                "swapper/8:0 [120] R ==> kworker/u128:1:777 [120]")
        self.ev(c + 4.5, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_start",
                "work struct 0xtty: function flush_to_ldisc")
        self.ev(c + 5.0, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_end",
                "work struct 0xtty: function flush_to_ldisc")
        self.ev(c + 5.1, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_start",
                "work struct 0xbar: function wq_barrier_func")
        if waking_in_barrier:
            self.ev(c + 5.2, "kworker/u128:1", 777, 8, "d..3.", "sched_waking",
                    f"comm=x7_bus_timing pid={TID} prio=19 target_cpu=003")
            self.ev(c + 5.3, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_end",
                    "work struct 0xbar: function wq_barrier_func")
        else:
            self.ev(c + 5.15, "kworker/u128:1", 777, 8, ".....", "workqueue_execute_end",
                    "work struct 0xbar: function wq_barrier_func")
            self.ev(c + 5.2, "kworker/u128:1", 777, 8, "d..3.", "sched_waking",
                    f"comm=x7_bus_timing pid={TID} prio=19 target_cpu=003")
        self.worker_in(c + 6.0)
        return c + 6.5

    def build(self) -> None:
        o = self.opt
        prev_end = None
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
                                              queue=not (i == 0 and o.get("no_queue"))))
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
            if i < len(REPLIES) - 1 and not (i == 1 and o.get("began_awake")):
                self.worker_out(cycle_end + 0.5, "S")
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
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)

        code, out, seg = correlate(root, "clean")
        expect(code == 0, "the clean run passes", out)
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
                "kworker_prev": "swapper/8"}
        for k, v in want.items():
            expect(r[k] == v, f"split reply's {k} is {v} (got {r[k]})", out)

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
