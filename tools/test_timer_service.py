#!/usr/bin/env python3
"""Regressions for tools/timer_service.py on synthetic events.

The TSC and CLOCK_MONOTONIC follow a known mapping (3187.2 MHz from an
origin), so every conversion has a true value. (The pairing of the anchors,
the bound and the certificate are tools/test_timer_clock.py's.) Covered:
interpolation without extrapolation; the residuals of held-out anchors and
the bracketed sampler, of which only the sampler's detect a systematic
offset of the instance's timestamps; a strict ledger of the real-time CPU's
records, with an unreadable record reported; each class of a
late service (a deadline written late or disarmed, an interrupt late from
idle and while busy in user mode and in a system call, a late sleep call,
a timer armed late, and INDETERMINATE deadlines), on a deadline timeline
that a firing disarms (an already-served deadline is not in effect) and
that keeps an overwritten due deadline's interrupt as possibly pending; the
deadline writes' attribution, which needs the timer's residency on the
real-time CPU (its expiry there or a pinned arming; a matching write after
an arming is a hint only), never a matching expiry alone, and converts
other clocks by their recorded offsets; the exerciser's actions followed through
each arming's lifetime (a needed write, a legitimate no-write, re-arms
earlier and after an expiry, cancellations with and without a write and
after an expiry, an expiry inside the arming call, a missing write, a timer
on another CPU or of unknown residency, a pinned arming, an action with no
event, and a cancellation without its event while its arming is pending);
the background load by lineage, with a batch workload; and the SMI file,
with the SMI series' problems.
"""

from __future__ import annotations

import pathlib
import random
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import timer_service as ts  # noqa: E402

HZ = 3_187_200_000
T0, M0 = 9_000_000_000_000, 300_000_000_000_000  # TSC and monotonic at the origin
RT = 3
TID = 4242


def tsc(mono: float) -> int:
    return T0 + round((mono - M0) * HZ / 1e9)


def fail(msg: str) -> None:
    print(f"FAILED: {msg}")
    sys.exit(1)


def expect(cond: bool, msg: str) -> None:
    if not cond:
        fail(msg)


def anchors(n: int = 400, jitter: int = 100, seed: int = 1) -> tuple[list[int], list[int]]:
    """Main-trace anchor times and their instance TSC counts (the main
    stamps jittered, as two callbacks' stamps are)."""
    rnd = random.Random(seed)
    mono, t = [], []
    m = M0 + 1_000_000
    for _ in range(n):
        m += rnd.randint(200_000, 2_000_000)
        mono.append(m + rnd.randint(-jitter, jitter))
        t.append(tsc(m))
    return mono, t


def sampler(span: tuple[int, int], n: int = 60, offset_ns: float = 0, cpu: int = 1) -> list[tuple]:
    out = []
    for k in range(n):
        m = span[0] + (span[1] - span[0]) * (k + 0.5) / n
        out.append((int(m - 20), tsc(m + offset_ns), int(m + 20), cpu))
    return out


def conversion() -> None:
    mono, t = anchors()
    samp = sampler((mono[0], mono[-1]))
    rate = ts.fit_rate(samp)
    expect(rate is not None and abs(1e9 / rate[1] - HZ) < 1_000, f"sampler rate {rate}")
    pairs = list(zip(t, mono))
    m = ts.TscMap(pairs)
    mid = (mono[100] + mono[101]) / 2
    expect(abs(m.mono(tsc(mid)) - mid) < 300, "interpolation")
    expect(m.mono(t[0] - 1) is None and m.mono(t[-1] + 1) is None, "no extrapolation")
    v = ts.validate(pairs, samp)
    expect(v["u"] is not None and v["u"] < 1_000, f"U {v['u']}")
    # A constant skew of the instance's stamps: held-out anchors cannot see
    # it, the bracketed sampler does.
    skew = [x + round(10_000 * HZ / 1e9) for x in t]
    v3 = ts.validate(list(zip(skew, mono)), samp)
    expect(max(v3["held"]) < 1_000 and v3["u"] > ts.U_LIMIT_NS, f"skew: held {max(v3['held'])}, U {v3['u']}")


def ledger() -> None:
    """The real-time CPU's records are taken strictly; another CPU's are not its business; an unreadable one is reported."""
    ev = ts.TimerEvents(TID, RT)
    ev.feed(100, RT, 0, "<idle>", "local_timer_entry", "vector=236", "d.h1.")
    ev.feed(200, RT, 0, "<idle>", "write_msr", "6e0, value 2723d67a9bc2", "d.h1.")
    ev.feed(300, RT, 0, "<idle>", "local_timer_exit", "vector=236", "d.h1.")
    ev.feed(250, 5, 0, "<idle>", "local_timer_entry", "vector=236", "d.h1.")
    expect(ev.ledger.events == [(100, "E", None), (200, "W", 0x2723d67a9bc2), (300, "X", None)] and not ev.ledger.problems,
           f"ledger {ev.ledger.events} {ev.ledger.problems}")
    ev.feed(290, RT, 0, "<idle>", "local_timer_entry", "vector=236", "d.h1.")
    expect(any("out of order" in x for x in ev.ledger.problems), f"out of order: {ev.ledger.problems}")
    ev = ts.TimerEvents(TID, RT)
    ev.unparsed("cpus=32\n")
    ev.unparsed("some other event's odd line\n")
    expect(not ev.ledger.problems, f"harmless rows: {ev.ledger.problems}")
    ev.unparsed("  <idle>-0 [003] d.h1. 100.000100: local_timer_entry: vector=236\n")
    expect(any("unreadable" in x for x in ev.ledger.problems), f"a microsecond ledger row: {ev.ledger.problems}")


def base_events() -> tuple[ts.TimerEvents, ts.TscMap, float]:
    mono, t = anchors(n=200, jitter=0)
    ev = ts.TimerEvents(TID, RT)
    ev.anchors = list(mono)
    return ev, ts.TscMap(list(zip(t, mono))), 500.0


def classes() -> None:
    ev, conv, u = base_events()
    a = ev.anchors
    e = a[50] + 100_000  # an expiry between anchors

    def case(writes, served, setup=None, arm_at=None, call=None, extra=(), uu=None):
        """writes: (ts, deadline or 0[, failed]) in order; served: the local
        timer interrupt that served the expiry; extra: other interrupts."""
        # A known state first, as in a trace long before the run: an earlier
        # deadline, served by its interrupt.
        first = writes[0][0]
        ev.writes = [(first - 100_000, tsc(first - 90_000), False)]
        ev.writes += [(w[0], tsc(w[1]) if w[1] else 0, len(w) > 2 and w[2]) for w in writes]
        ev.idle, ev.rt_switches, ev.syscalls, ev.nmis = [], [], [], []
        ev.anchors = sorted([x for x in a if x < first - 100_000] + [first - 90_000, served, *extra])
        if setup:
            setup()
        tl = ts.Timeline(ev, conv, u if uu is None else uu)
        return ts.classify(ev, tl, (arm_at if arm_at is not None else e - 500_000, e), (served + 300, served + 200),
                           [], call)

    r = case([(e - 400_000, e + 900_000)], e + 941_000)
    expect(r["class"] == "deadline late" and "the deadline in effect was +900.0 us" in r["detail"], f"deadline late: {r}")
    r = case([(e - 400_000, 0), (e + 900_000, e + 940_000)], e + 941_000)
    expect(r["class"] == "deadline late" and "disarmed (its write traced at -400.0 us)" in r["detail"], f"disarmed: {r}")
    # A deadline already served is not still armed (the reviewer's case):
    # the next one was written at E+90 us for E+100 us.
    r = case([(e - 400_000, e - 300_000), (e + 90_000, e + 100_000)], e + 100_000, extra=[e - 300_000])
    expect(r["class"] == "deadline late" and "was served by the interrupt at -300.0 us" in r["detail"]
           and "had its write traced at +90.0 us" in r["detail"], f"already served: {r}")
    # A deadline due and overwritten with no interrupt seen after it may
    # still have its interrupt pending (the reviewer's case): the serving
    # interrupt cannot be told apart, so INDETERMINATE; overwritten before it
    # was due, or with an interrupt seen in between, it is not pending.
    r = case([(e - 400_000, e - 300_000), (e + 90_000, e + 100_000)], e + 150_000)
    expect(r["class"] == "INDETERMINATE" and "pending one" in r["detail"] and "was due when it was overwritten"
           in r["detail"], f"overwritten, pending: {r}")
    r = case([(e - 400_000, e + 50_000), (e - 100_000, e + 100_000)], e + 100_000)
    expect(r["class"] == "deadline late" and "the deadline in effect was +100.0 us" in r["detail"],
           f"overwritten before it was due: {r}")
    r = case([(e - 400_000, e - 300_000), (e + 90_000, e + 100_000)], e + 150_000, extra=[e + 95_000])
    expect(r["class"] == "deadline late" and "had its write traced at +90.0 us" in r["detail"],
           f"the pending interrupt seen first: {r}")
    # The state before the trace's first write is unknown: its first serving
    # interrupt may be a pending one.
    ev.writes = [(e - 400_000, tsc(e), False)]
    ev.anchors = [e + 100_000]
    r = ts.classify(ev, ts.Timeline(ev, conv, u), (e - 500_000, e), (e + 100_300, e + 100_200), [])
    expect(r["class"] == "INDETERMINATE" and "first deadline write" in r["detail"], f"before the first write: {r}")
    # An interrupt after a served deadline with no write, one before the
    # deadline armed (pending from before its write), and one after a failed
    # write are INDETERMINATE.
    r = case([(e - 400_000, e - 300_000)], e + 100_000, extra=[e - 300_000])
    expect(r["class"] == "INDETERMINATE" and "not armed" in r["detail"], f"served, then an interrupt: {r}")
    r = case([(e - 400_000, e + 300_000)], e + 100_000)
    expect(r["class"] == "INDETERMINATE" and "pending from before" in r["detail"], f"interrupt before: {r}")
    r = case([(e - 400_000, e), (e - 200_000, e + 300_000, True)], e + 300_000)
    expect(r["class"] == "INDETERMINATE" and "failed" in r["detail"], f"a failed write: {r}")

    def idle():
        ev.idle = [(e - 300_000, 2)]
    r = case([(e - 400_000, e)], e + 150_000, idle)
    expect(r["class"] == "interrupt late from idle" and "idle state 2" in r["detail"], f"idle: {r}")

    def busy():
        ev.rt_switches = [(e - 300_000, 999, "stress-ng-cpu")]
        ev.syscalls = [(e - 250_000, 999, False)]
        ev.nmis = [(e + 10_000, e + 14_000, "perf_event_nmi_handler")]
    r = case([(e - 400_000, e)], e + 600_000, busy)
    expect(r["class"] == "interrupt late while busy" and "stress-ng-cpu" in r["detail"]
           and "in user mode or a fault" in r["detail"] and "1 NMIs" in r["detail"], f"busy user: {r}")

    def kernel():
        ev.rt_switches = [(e - 300_000, 777, "KMS thread")]
        ev.syscalls = [(e - 200_000, 777, True)]
    r = case([(e - 400_000, e)], e + 300_000, kernel)
    expect(r["class"] == "interrupt late while busy" and "in a system call" in r["detail"], f"busy kernel: {r}")

    def entered_elsewhere():
        ev.rt_switches = [(e - 300_000, 777, "KMS thread")]
        ev.syscalls = [(e + 200_000, 777, False)]
    r = case([(e - 400_000, e)], e + 300_000, entered_elsewhere)
    expect("in a system call" in r["detail"], f"a call entered on another CPU: {r}")

    def unknown():
        ev.rt_switches = [(e - 300_000, 999, "stress-ng-cpu")]
    r = case([(e - 400_000, e)], e + 300_000, unknown)
    expect("mode unknown" in r["detail"], f"no system call recorded: {r}")
    # A sleep call entered after the expiry, and one entered before it whose
    # timer was armed after it (the reviewer's case), are told apart.
    r = case([(e - 400_000, e)], e + 300_000, arm_at=e + 50_000, call=e + 20_000)
    expect(r["class"] == "late sleep call" and "entered 20.0 us after" in r["detail"], f"late sleep call: {r}")
    r = case([(e - 400_000, e)], e + 300_000, arm_at=e + 50_000, call=e - 100_000)
    expect(r["class"] == "timer armed late" and "entered 100.0 us before its expiry" in r["detail"]
           and "armed 50.0 us after" in r["detail"], f"timer armed late: {r}")
    r = case([(e - 400_000, e)], e + 300_000, arm_at=e + 50_000)
    expect(r["class"] == "timer armed late" and "not recorded" in r["detail"], f"armed late, no call: {r}")
    r = case([(e - 400_000, e)], e + 2_000)
    expect(r["class"] == "on time", f"on time: {r}")
    ev.writes = [(e - 500_000, tsc(e - 490_000), False), (e - 400_000, conv.t[-1] + 10, False)]
    ev.anchors = sorted([x for x in a if x < e - 500_000] + [e - 490_000, e + 100_000])
    r = ts.classify(ev, ts.Timeline(ev, conv, u), (e - 500_000, e), (e + 100_300, e + 100_200), [])
    expect(r["class"] == "INDETERMINATE", f"outside the anchors: {r}")
    r = case([(e - 400_000, e)], e + 100_000, uu=9_000.0)
    expect(r["class"] == "INDETERMINATE" and "not validated" in r["detail"], f"U above the limit: {r}")
    # The boundary includes U: a deadline 400 ns after the expiry is on time.
    r = case([(e - 400_000, e + 400)], e + 100_000, busy)
    expect(r["class"] == "interrupt late while busy", f"within U: {r}")
    r = case([(e - 400_000, e + 600)], e + 100_000, busy)
    expect(r["class"] == "deadline late", f"beyond U: {r}")


def residency() -> None:
    """Deadline attribution needs the timer's residency on the real-time
    CPU, not only a matching expiry."""
    ev, conv, u = base_events()
    t = ev.anchors[20] + 100_000
    e = t + 400_000
    other, worker, tick = "0xffff0000bbbb", "0xffff0000cccc", "0xffff0000dddd"
    ev.writes = [(t + 10_000, tsc(e), False)]

    # The run record's clock offsets (no suspend: BOOTTIME is MONOTONIC).
    recorded = {"REALTIME": 1_700_000_000_000_000_000, "BOOTTIME": 0, "TAI": 1_700_000_037_000_000_000}

    def att(offsets=None):
        return ts.attribute(ev, ts.Lifetimes(ev, conv, u, {**recorded, **(offsets or {})}), conv, u)[0][2]

    # The reviewer's case: armed and expired on CPU 7 at the deadline's value.
    ev.starts = [(t, 7, 777, "other-task", other, "timerfd_tmrproc", e, e, "ABS")]
    ev.expiries = [(e + 1_000, 7, other, e + 900, "timerfd_tmrproc")]
    r = att()
    expect(r == "INDETERMINATE (no pending timer on this CPU at that expiry)", f"expired on CPU 7: {r}")
    # Armed on CPU 7 and cancelled: its residency is unknown.
    ev.expiries, ev.cancels = [], [(e - 50_000, 7, 777, other, False)]
    r = att()
    expect(r == "INDETERMINATE (1 matching timers of unknown residency)", f"unknown residency: {r}")
    # The worker's timer, expiring on CPU 3, with another CPU's pinned tick at
    # the same expiry: the worker's.
    ev.cancels = []
    ev.starts = [(t, RT, TID, "x7_bus_timing", worker, "hrtimer_wakeup", e, e, "REL"),
                 (t, 5, 0, "swapper/5", tick, "tick_nohz_handler", e, e, "ABS|PINNED|HARD")]
    ev.expiries = [(e + 1_000, RT, worker, e + 900, "hrtimer_wakeup")]
    r = att()
    expect(r == f"hrtimer_wakeup of x7_bus_timing ({TID})", f"the worker's: {r}")
    # A pinned arming on CPU 3 is resident there: two matching timers.
    ev.starts.append((t, RT, 0, "swapper/3", tick + "3", "tick_nohz_handler", e, e, "ABS|PINNED|HARD"))
    r = att()
    expect(r == "INDETERMINATE (2 pending timers at that expiry)", f"two on CPU 3: {r}")
    ev.starts.pop()
    # The reviewer's case: an unpinned arming on CPU 3, cancelled, with a
    # matching CPU-3 write 10 us later and a switch, a device interrupt and
    # an NMI in between: a hint, never residency.
    ev2, _, _ = base_events()
    ev2.anchors = []
    near = "0xffff00001111"
    ev2.starts = [(t, RT, 111, "exerciser", near, "timerfd_tmrproc", e, e, "ABS")]
    ev2.cancels = [(t + 100_000, RT, 111, near, False)]
    ev2.writes = [(t + 10_000, tsc(e), False)]
    ev2.rt_switches = [(t + 500, 222, "other-task")]
    ev2.irqs = [(t + 1_000, t + 3_000)]
    ev2.nmis = [(t + 4_000, t + 5_000, "perf_event_nmi_handler")]
    lt2 = ts.Lifetimes(ev2, conv, u, recorded)
    g = lt2.gens[0]
    expect(g["cpu"] is None and g["evidence"] == "unknown" and g["hint"], f"the write hint: {g}")
    r = ts.attribute(ev2, lt2, conv, u)[0][2]
    expect(r == "INDETERMINATE (1 matching timers of unknown residency; 1 followed by a matching write on this CPU, "
           "a hint only)", f"the hint does not attribute: {r}")
    # A CLOCK_BOOTTIME timer of unknown residency: unconvertible without its
    # offset; with it, matching or not.
    boot = "0xffff0000eeee"
    ev.clockid[boot] = "CLOCK_BOOTTIME"
    ev.clockid[worker] = "CLOCK_MONOTONIC"  # its hrtimer_setup was traced
    ev.starts.append((t, 9, 55, "systemd", boot, "hrtimer_wakeup", e + 5_000_000_000, e + 5_000_000_000, "ABS"))
    r = att({"BOOTTIME": None})
    expect(r == "INDETERMINATE (1 pending timers whose clock cannot be converted)", f"boottime, no offset: {r}")
    r = att({"BOOTTIME": 5_000_000_000})
    expect(r == "INDETERMINATE (1 matching timers of unknown residency)", f"boottime matching: {r}")
    r = att({"BOOTTIME": 4_000_000_000})
    expect(r == f"hrtimer_wakeup of x7_bus_timing ({TID})", f"boottime elsewhere in time: {r}")
    # A clock inferred from the expiry's magnitude takes both readings.
    expect(ts.mono_values(ev, "0xnone", 1_000, {"BOOTTIME": 7}) == [1_000.0, 993.0], "magnitude: monotonic or boottime")
    expect(ts.mono_values(ev, "0xnone", 2 * 10**18, {"REALTIME": 10**18, "TAI": None}) is None, "TAI offset missing")


def exerciser() -> None:
    ev, conv, u = base_events()
    base = ev.anchors[20]
    ev.anchors = []
    x = 777
    head = {"tid": str(x)}
    addr = "0xffff0000aaaa"
    rows: list[dict] = []
    starts, cancels, expiries, writes = [], [], [], []

    def row(rnd, kind, action, t, expiry, dur=10_000):
        rows.append({"round": rnd, "kind": kind, "action": action, "before": t, "after": t + dur, "expiry": expiry})

    def start(t, expiry, mode="ABS"):
        starts.append((t, RT, x, "timer_exerci", addr, "timerfd_tmrproc", expiry, expiry, mode))

    def cancel(t):
        cancels.append((t, RT, x, addr, False))

    def write(t, d):
        writes.append((t, tsc(d) if d else 0, False))

    def expire(at, cpu=RT):
        if cpu == RT:
            ev.anchors.append(at - 500)  # the interrupt it runs in
        expiries.append((at, cpu, addr, at - 100, "timerfd_tmrproc"))

    t = base + 100_000
    # 1: a write needed.
    row(1, "expire", "arm", t, t + 400_000)
    start(t + 3_000, t + 400_000)
    write(t + 4_000, t + 400_000)
    expire(t + 401_000)
    t += 1_000_000
    # 2: an earlier timer's deadline in effect: no write needed (its
    # interrupt then reprograms for this one).
    write(t - 1_000, t + 200_000)
    row(2, "expire", "arm", t, t + 600_000)
    start(t + 3_000, t + 600_000)
    ev.anchors.append(t + 200_000)
    write(t + 201_000, t + 600_000)
    expire(t + 601_000)
    t += 1_000_000
    # 3: armed, then re-armed earlier (a cancellation, then the arming).
    row(3, "earlier", "arm", t, t + 800_000)
    start(t + 3_000, t + 800_000)
    write(t + 4_000, t + 800_000)
    row(3, "earlier", "rearm", t + 60_000, t + 400_000)
    cancel(t + 62_000)
    start(t + 63_000, t + 400_000)
    write(t + 64_000, t + 400_000)
    expire(t + 401_000)
    t += 1_000_000
    # 4: cancelled; the device stays armed for it (a spurious interrupt follows).
    row(4, "cancel", "arm", t, t + 500_000)
    start(t + 3_000, t + 500_000)
    write(t + 4_000, t + 500_000)
    row(4, "cancel", "cancel", t + 100_000, t + 500_000)
    cancel(t + 102_000)
    t += 1_000_000
    # 5: cancelled; the device reprogrammed.
    row(5, "cancel", "arm", t, t + 500_000)
    start(t + 3_000, t + 500_000)
    write(t + 4_000, t + 500_000)
    row(5, "cancel", "cancel", t + 100_000, t + 500_000)
    cancel(t + 102_000)
    write(t + 103_000, t + 5_000_000)
    t += 1_000_000
    # 6: a later deadline in effect and no write: unexplained.
    write(t - 1_000, t + 900_000)
    row(6, "expire", "arm", t, t + 400_000)
    start(t + 3_000, t + 400_000)
    expire(t + 901_000)
    t += 1_000_000
    # 7: expired on another CPU: indeterminate.
    row(7, "expire", "arm", t, t + 400_000)
    start(t + 3_000, t + 400_000)
    expire(t + 401_000, cpu=5)
    t += 1_000_000
    # 8: no event at all.
    row(8, "expire", "arm", t, t + 400_000)
    t += 1_000_000
    # 9: the reviewer's case: it expires and is served inside its arming call,
    # and the next deadline is written before the call returns.
    row(9, "expire", "arm", t, t + 5_000)
    start(t + 1_000, t + 5_000)
    write(t + 2_000, t + 5_000)
    expire(t + 6_000)
    write(t + 7_000, t + 1_000_000)
    t += 1_000_000
    # 11: a cancellation after the timer had expired: nothing to cancel.
    row(11, "cancel", "arm", t, t + 300_000)
    start(t + 3_000, t + 300_000)
    write(t + 4_000, t + 300_000)
    expire(t + 301_000)
    row(11, "cancel", "cancel", t + 400_000, t + 300_000)
    t += 1_000_000
    # 12: re-armed later after the first arming had expired (an overrun
    # sleep): no cancellation.
    row(12, "later", "arm", t, t + 200_000)
    start(t + 3_000, t + 200_000)
    write(t + 4_000, t + 200_000)
    expire(t + 201_000)
    row(12, "later", "rearm", t + 250_000, t + 700_000)
    start(t + 253_000, t + 700_000)
    write(t + 254_000, t + 700_000)
    expire(t + 701_000)
    t += 1_000_000
    # 13: an arming covered by an earlier deadline and then cancelled: its
    # residency is unknown (no expiry, no write after it, not pinned).
    write(t - 1_000, t + 100_000)
    row(13, "cancel", "arm", t, t + 500_000)
    start(t + 3_000, t + 500_000)
    ev.anchors.append(t + 100_000)
    write(t + 101_000, t + 500_000)
    row(13, "cancel", "cancel", t + 200_000, t + 500_000)
    cancel(t + 202_000)
    t += 1_000_000
    # 14: a pinned arming is resident on its CPU: no write needed.
    write(t - 1_000, t + 300_000)
    row(14, "cancel", "arm", t, t + 500_000)
    start(t + 3_000, t + 500_000, mode="ABS|PINNED")
    row(14, "cancel", "cancel", t + 100_000, t + 500_000)
    cancel(t + 102_000)
    t += 1_000_000
    # 16: armed after its own expiry (its call came late): the kernel writes
    # a deadline already due, and the timer expires at once.
    row(16, "expire", "arm", t, t - 50_000)
    start(t + 3_000, t - 50_000)
    write(t + 4_000, t + 4_000 - 300)
    expire(t + 5_000)
    t += 1_000_000
    # 17: the same with only a stale write before the arming: unexplained.
    row(17, "expire", "arm", t, t - 50_000)
    write(t + 1_000, t - 40_000)
    start(t + 3_000, t - 50_000)
    expire(t + 5_000)
    t += 1_000_000
    # 18: the same with a future deadline written after it: unexplained.
    row(18, "expire", "arm", t, t - 50_000)
    start(t + 3_000, t - 50_000)
    write(t + 4_000, t + 400_000)
    expire(t + 401_000)
    t += 1_000_000
    # 15: the reviewer's case: a cancellation without its event while the
    # arming is pending (an earlier round's expiry does not excuse it).
    row(15, "cancel", "arm", t, t + 500_000)
    start(t + 3_000, t + 500_000)
    write(t + 4_000, t + 500_000)
    row(15, "cancel", "cancel", t + 100_000, t + 500_000)
    rows.append({"round": 1, "kind": "expire", "action": "expired", "before": base + 600_000,
                 "after": base + 600_000, "expiry": base + 500_000})  # an observation, not an action
    ev.starts, ev.cancels, ev.expiries = starts, cancels, expiries
    ev.writes = sorted(writes)
    ev.anchors.sort()
    tl = ts.Timeline(ev, conv, u)
    r = ts.explain_exerciser(ev, tl, ts.Lifetimes(ev, conv, u, {"BOOTTIME": 0}), head, rows)
    expect(r["actions"] == 25, f"actions {r}")
    # The armings that never expire and are not pinned (rounds 3, 4, 5, 13
    # and 15's arms, and the cancellations of 4, 5 and 13) are of unknown
    # residency, the write after an arming a hint only.
    expect(r["OK"] == 8 and r["no write needed"] == 2 and r["armed after its expiry"] == 1
           and r["INDETERMINATE"] == 9 and r["UNEXPLAINED"] == 5, f"exerciser classes: {r}")
    for n in (17, 18):
        expect(any(f"round {n} arm: armed +53.0 us from its expiry, with no deadline write after it already due" in m
                   for m in r["notes"]), f"round {n}: {r['notes']}")
    expect(r["indeterminate"] == {"residency unknown, its events matched (a write hint)": 6,
                                  "residency unknown, its events matched": 2, "on another CPU": 1},
           f"exerciser INDETERMINATE reasons: {r['indeterminate']}")
    for n in (6, 8, 15):
        expect(any(f"round {n} " in m for m in r["notes"]), f"round {n} noted: {r['notes']}")
    expect(any("round 15 cancel: its arming stayed pending" in m for m in r["notes"]), f"round 15: {r['notes']}")
    expect(any("round 7 arm: its timer was on CPU 5 (its expiry)" in m for m in r["notes"]), f"round 7: {r['notes']}")


def load() -> None:
    ev = ts.TimerEvents(TID, RT, window=(1_000, 10_000))
    ev.forks = [(2_000, 100, "bash", 101, "bash"), (2_500, 50, "gnome-shell", 300, "exiftool")]
    ev.cpu_time = {101: 4_000, 300: 6_000, 400: 2_000, 9: 3_000, TID: 8_000}
    ev.comm = {101: "bash", 300: "exiftool", 400: "renderer", 9: "kworker/u128:1"}
    procs = {100: (1, "bash"), 50: (1, "gnome-shell"), 400: (50, "renderer"), 9: (2, "kworker/u128:1")}
    b = ts.background(ev, 100, procs, {"exiftool"})
    expect(set(b["by_comm"]) == {"exiftool", "renderer"} and b["batch"] == ["exiftool"], f"background {b}")
    # The record's list, with and without its exemption.
    for line in ("batch workloads: exiftool,localsearch-ext (exempt: unattended-upgr|^/usr/bin/python3$|/x --y)",
                 "batch workloads: exiftool,localsearch-ext"):
        got = ts.record_batch([line])
        expect(got == {"exiftool", "localsearch-ext"}, f"the record's batch list: {got}")
    # Accounting inside the window only.
    ev2 = ts.TimerEvents(TID, RT, window=(1_000, 10_000))
    ev2.switch(500, 0, 0, "swapper/0", 300, "exiftool")
    ev2.switch(3_000, 0, 300, "exiftool", 0, "swapper/0")
    ev2.switch(9_000, 1, 0, "swapper/1", 400, "renderer")
    ev2.finish(12_000)
    expect(ev2.cpu_time == {300: 2_000, 400: 1_000}, f"window accounting {ev2.cpu_time}")


def files() -> None:
    with tempfile.TemporaryDirectory() as d:
        p = pathlib.Path(d)
        sf = p / "smi.csv"
        sf.write_text("# started\n0.010000000,0,,msr/smi/,1,100.00,,\n0.020000000,2,,msr/smi/,1,100.00,,\n")
        # perf started between 1.000 s (before) and 1.015 s - 0.010 s (its
        # first interval, at +0.010 s, was seen at 1.015 s).
        s, pr = ts.load_smi(sf, (1_000_000_000, 1_015_000_000), (1_002_000_000, 1_020_000_000))
        expect(s == [(1_000_000_000, 1_015_000_000, 0), (1_010_000_000, 1_025_000_000, 2)] and not pr, f"smi {s} {pr}")
        _, pr = ts.load_smi(sf, (1_000_000_000, 1_015_000_000), (1_002_000_000, 1_030_000_000))
        expect(pr == ["the SMI counts do not cover the run"], f"smi coverage {pr}")
        _, pr = ts.load_smi(sf, None)
        expect(pr == ["the record has no valid SMI anchor"], f"smi anchor {pr}")
        _, pr = ts.load_smi(p / "absent.csv", (1, 2))
        expect(pr == ["no SMI count file"], f"smi file {pr}")
        sf.write_text("0.010000000,<not counted>,,msr/smi/,0,0.00,,\n0.020000000,1,,msr/smi/,1,100.00,,\n"
                      "0.030000000,0,,msr/smi/,1,100.00,,\n0.040000000,0,,msr/smi/,1,100.00,,\n"
                      "0.050000000,0,,msr/smi/,1,100.00,,\n0.110000000,0,,msr/smi/,1,100.00,,\n")
        _, pr = ts.load_smi(sf, (1_000_000_000, 1_015_000_000))
        expect(any("not a number (<not counted>)" in x for x in pr) and any("a gap of 0.060 s" in x for x in pr),
               f"smi problems {pr}")

def main() -> int:
    conversion()
    ledger()
    classes()
    residency()
    exerciser()
    load()
    files()
    print("timer service regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
