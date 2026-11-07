#!/usr/bin/env python3
"""Regressions for tools/timer_service.py on synthetic events.

The TSC and CLOCK_MONOTONIC follow a known mapping (3187.2 MHz from an
origin), so every conversion has a true value. Covered: the anchors'
one-to-one matching by their interval pattern, and its refusal of a missing
and a duplicate anchor; interpolation without extrapolation; the
uncertainty from held-out anchors and the bracketed sampler, which alone
detects a systematic offset of the instance's timestamps; each class of a
late service (a deadline written late or disarmed, an interrupt late from
idle and while busy in user mode and in a system call, a late sleep call,
and an INDETERMINATE deadline); the exerciser's actions (a needed write,
a legitimate no-write, a re-arm, cancellations with and without a write, a
missing write, a timer that expired on another CPU, an action with no
event); the background load by lineage, with a batch workload; and the
instance, sampler and SMI files.
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
    pairs, problems = ts.match_anchors(mono, t, rate)
    expect(not problems and len(pairs) == len(t), f"matching: {problems}, {len(pairs)}")
    # Extra main anchors before and after the instance's span still match.
    pre = [mono[0] - 700_000, mono[0] - 300_000]
    pairs2, problems2 = ts.match_anchors(pre + mono + [mono[-1] + 900_000], t, rate)
    expect(not problems2 and pairs2 == pairs, f"matching inside a longer main trace: {problems2}")
    m = ts.TscMap(pairs)
    mid = (mono[100] + mono[101]) / 2
    expect(abs(m.mono(tsc(mid)) - mid) < 300, "interpolation")
    expect(m.mono(t[0] - 1) is None and m.mono(t[-1] + 1) is None, "no extrapolation")
    v = ts.validate(pairs, samp)
    expect(v["u"] is not None and v["u"] < 1_000, f"U {v['u']}")
    # A missing instance anchor, and a duplicate main anchor, are refused.
    _, p = ts.match_anchors(mono, t[:200] + t[201:], rate)
    expect(any("missing or duplicate" in x for x in p), f"missing anchor: {p}")
    _, p = ts.match_anchors(mono[:150] + [mono[150] - 40_000] + mono[150:], t, rate)
    expect(any("missing or duplicate" in x for x in p), f"duplicate anchor: {p}")
    # A constant skew of the instance's stamps: held-out anchors cannot see
    # it, the bracketed sampler does.
    skew = [x + round(10_000 * HZ / 1e9) for x in t]
    pairs3, _ = ts.match_anchors(mono, skew, rate)
    v3 = ts.validate(pairs3, samp)
    expect(max(v3["held"]) < 1_000 and v3["u"] > ts.U_LIMIT_NS, f"skew: held {max(v3['held'])}, U {v3['u']}")


def base_events() -> tuple[ts.TimerEvents, ts.TscMap, float]:
    mono, t = anchors(n=200, jitter=0)
    ev = ts.TimerEvents(TID, RT)
    ev.anchors = list(mono)
    pairs, _ = ts.match_anchors(mono, t, ts.fit_rate(sampler((mono[0], mono[-1]))))
    return ev, ts.TscMap(pairs), 500.0


def classes() -> None:
    ev, conv, u = base_events()
    a = ev.anchors
    e = a[50] + 100_000  # an expiry between anchors

    def case(write_at, deadline, served, setup=None, arm_at=None):
        ev.writes = [(write_at, tsc(deadline) if deadline else 0, False)]
        ev.idle, ev.rt_switches, ev.syscalls, ev.nmis = [], [], [], []
        ev.anchors = sorted(set(a) | {served})
        if setup:
            setup()
        return ts.classify(ev, conv, u, (arm_at if arm_at is not None else e - 500_000, e), (served + 300, served + 200), [])

    r = case(e - 400_000, e + 900_000, e + 941_000)
    expect(r["class"] == "deadline late", f"deadline late: {r}")
    r = case(e - 400_000, None, e + 941_000)
    expect(r["class"] == "deadline late" and "disarmed" in r["detail"], f"disarmed: {r}")

    def idle():
        ev.idle = [(e - 300_000, 2)]
    r = case(e - 400_000, e, e + 150_000, idle)
    expect(r["class"] == "interrupt late from idle" and "idle state 2" in r["detail"], f"idle: {r}")

    def busy():
        ev.rt_switches = [(e - 300_000, 999, "stress-ng-cpu")]
        ev.syscalls = [(e - 250_000, 999, False)]
        ev.nmis = [(e + 10_000, e + 14_000, "perf_event_nmi_handler")]
    r = case(e - 400_000, e, e + 600_000, busy)
    expect(r["class"] == "interrupt late while busy" and "stress-ng-cpu" in r["detail"]
           and "in user mode or a fault" in r["detail"] and "1 NMIs" in r["detail"], f"busy user: {r}")

    def kernel():
        ev.rt_switches = [(e - 300_000, 777, "KMS thread")]
        ev.syscalls = [(e - 200_000, 777, True)]
    r = case(e - 400_000, e, e + 300_000, kernel)
    expect(r["class"] == "interrupt late while busy" and "in a system call" in r["detail"], f"busy kernel: {r}")

    def entered_elsewhere():
        ev.rt_switches = [(e - 300_000, 777, "KMS thread")]
        ev.syscalls = [(e + 200_000, 777, False)]
    r = case(e - 400_000, e, e + 300_000, entered_elsewhere)
    expect("in a system call" in r["detail"], f"a call entered on another CPU: {r}")

    def unknown():
        ev.rt_switches = [(e - 300_000, 999, "stress-ng-cpu")]
    r = case(e - 400_000, e, e + 300_000, unknown)
    expect("mode unknown" in r["detail"], f"no system call recorded: {r}")
    r = case(e - 400_000, e, e + 300_000, arm_at=e + 50_000)
    expect(r["class"] == "late sleep call", f"late sleep call: {r}")
    r = case(e - 400_000, e, e + 2_000)
    expect(r["class"] == "on time", f"on time: {r}")
    ev.writes = [(e - 400_000, conv.t[-1] + 10, False)]
    r = ts.classify(ev, conv, u, (e - 500_000, e), (e + 100_300, e + 100_200), [])
    expect(r["class"] == "INDETERMINATE", f"outside the anchors: {r}")
    r = ts.classify(ev, conv, 9_000.0, (e - 500_000, e), (e + 100_300, e + 100_200), [])
    expect(r["class"] == "INDETERMINATE", f"U above the limit: {r}")
    # The boundary includes U: a deadline 400 ns after the expiry is on time.
    r = case(e - 400_000, e + 400, e + 100_000, busy)
    expect(r["class"] == "interrupt late while busy", f"within U: {r}")
    r = case(e - 400_000, e + 600, e + 100_000, busy)
    expect(r["class"] == "deadline late", f"beyond U: {r}")


def exerciser() -> None:
    ev, conv, u = base_events()
    a = ev.anchors
    x = 777
    head = {"tid": str(x)}
    addr = "0xffff0000aaaa"
    base = a[20]
    ev.writes = [(base, tsc(base + 5_000_000), False)]  # a far deadline in effect
    rows, starts, cancels, expiries = [], [], [], []

    def add(rnd, kind, action, t, expiry, write=None, start=True, cancel=False, exp_cpu: int | None = RT):
        rows.append({"round": rnd, "kind": kind, "action": action, "before": t, "after": t + 10_000,
                     "expiry": expiry})
        if cancel:
            cancels.append((t + 2_000, RT, x, addr, False))
        if start:
            starts.append((t + 3_000, RT, x, "timer_exerci", addr, "timerfd_tmrproc", expiry, expiry, "ABS"))
        if write is not None:
            ev.writes.append((t + 4_000, tsc(write) if write else 0, False))
        if exp_cpu is not None and action != "cancel":
            expiries.append((expiry + 1_000, exp_cpu, addr, expiry + 900, "timerfd_tmrproc"))

    t = base + 100_000
    add(1, "expire", "arm", t, t + 400_000, write=t + 400_000)          # earlier: written
    t += 1_000_000
    ev.writes.append((t - 1_000, tsc(t + 200_000), False))              # an earlier timer armed
    add(2, "expire", "arm", t, t + 600_000)                             # later: no write needed
    t += 1_000_000
    add(3, "earlier", "rearm", t, t + 300_000, write=t + 300_000, cancel=True)
    t += 1_000_000
    add(4, "cancel", "arm", t, t + 500_000, write=t + 500_000, exp_cpu=None)
    add(4, "cancel", "cancel", t + 100_000, t + 500_000, start=False, cancel=True, write=t + 3_000_000)
    t += 1_000_000
    add(5, "cancel", "arm", t, t + 500_000, write=t + 500_000, exp_cpu=None)
    add(5, "cancel", "cancel", t + 100_000, t + 500_000, start=False, cancel=True)  # no write: spurious path
    t += 1_000_000
    ev.writes.append((t - 1_000, tsc(t + 900_000), False))
    add(6, "expire", "arm", t, t + 400_000)                             # earlier, but no write: unexplained
    t += 1_000_000
    add(7, "expire", "arm", t, t + 400_000, exp_cpu=5)                  # expired elsewhere: indeterminate
    t += 1_000_000
    rows.append({"round": 8, "kind": "expire", "action": "arm", "before": t, "after": t + 10_000,
                 "expiry": t + 400_000})                                # no event at all
    rows.append({"round": 1, "kind": "expire", "action": "expired", "before": base + 600_000,
                 "after": base + 600_000, "expiry": base + 500_000})  # an observation, not an action
    ev.starts, ev.cancels, ev.expiries = starts, cancels, expiries
    ev.writes.sort()
    r = ts.explain_exerciser(ev, conv, u, head, rows)
    expect(r["actions"] == 10, f"actions {r}")
    expect(r["OK"] == 6 and r["no write needed"] == 1 and r["INDETERMINATE"] == 1 and r["UNEXPLAINED"] == 2,
           f"exerciser classes: {r}")
    expect(any("round 6" in n for n in r["notes"]) and any("round 8" in n for n in r["notes"]), f"notes {r['notes']}")


def load() -> None:
    ev = ts.TimerEvents(TID, RT, window=(1_000, 10_000))
    ev.forks = [(2_000, 100, "bash", 101, "bash"), (2_500, 50, "gnome-shell", 300, "exiftool")]
    ev.cpu_time = {101: 4_000, 300: 6_000, 400: 2_000, 9: 3_000, TID: 8_000}
    ev.comm = {101: "bash", 300: "exiftool", 400: "renderer", 9: "kworker/u128:1"}
    procs = {100: (1, "bash"), 50: (1, "gnome-shell"), 400: (50, "renderer"), 9: (2, "kworker/u128:1")}
    b = ts.background(ev, 100, procs, {"exiftool"})
    expect(set(b["by_comm"]) == {"exiftool", "renderer"} and b["batch"] == ["exiftool"], f"background {b}")
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
        inst = p / "i.txt"
        inst.write_text("# entries-in-buffer/entries-written: 3/3   #P:32\n"
                        "          <idle>-0       [003] d.h1. 9000000001000: local_timer_entry: vector=236\n"
                        "          <idle>-0       [005] d.h1. 9000000002000: local_timer_entry: vector=236\n"
                        "   stress-ng-cpu-12      [003] d.h1. 9000000003000: local_timer_entry: vector=236\n")
        t, pr = ts.load_instance(inst, RT)
        expect(t == [9000000001000, 9000000003000] and not pr, f"instance {t} {pr}")
        inst.write_text("# entries-in-buffer/entries-written: 3/5   #P:32\n"
                        "          <idle>-0       [003] d.h1.  1234.567890: local_timer_entry: vector=236\n")
        _, pr = ts.load_instance(inst, RT)
        expect(any("lost 2" in x for x in pr) and any("not raw" in x for x in pr), f"instance problems {pr}")
        pf = p / "pairs.csv"
        pf.write_text("# tsc_mono_pairs cpu=1\n100,5000,140,1\n200,5300,230,3\n")
        expect(ts.load_pairs(pf) == [(100, 5000, 140, 1), (200, 5300, 230, 3)], "pairs")
        sf = p / "smi.csv"
        sf.write_text("# started\n0.010000000,0,,msr/smi/,1,100.00,,\n0.020000000,2,,msr/smi/,1,100.00,,\n")
        # perf started between 1.000 s (before) and 1.015 s - 0.010 s (its
        # first interval, at +0.010 s, was seen at 1.015 s).
        s = ts.load_smi(sf, (1_000_000_000, 1_015_000_000))
        expect(s == [(1_000_000_000, 1_015_000_000, 0), (1_010_000_000, 1_025_000_000, 2)], f"smi {s}")


def main() -> int:
    conversion()
    classes()
    exerciser()
    load()
    files()
    print("timer service regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
