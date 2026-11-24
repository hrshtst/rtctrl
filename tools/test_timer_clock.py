#!/usr/bin/env python3
"""Regressions for tools/timer_clock.py on synthetic captures. Exit 0 if every case behaves as stated.

Besides the outcome of each case, the ground-truth groups assert that the actual error of every
converted value is within the U that classification would consume."""
import bisect
import copy
import math
import random
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import timer_clock as R  # noqa: E402

BASE = 10**12
LAT, DELAY, EXIT, H = 350, 60, 320, 20
FAILS, COUNT = [], 0
M2 = R.Model(2e-6)      # the slope premise used by the structural cases; these cases are not about the slope


def gen(n=4000, step=500_000, cluster=None):
    E = [BASE + i * step for i in range(n)]
    for k, v in (cluster or {}).items():
        E[k] = v(E)
    E = sorted(E)
    D = [e - LAT for e in E]
    W = [max(D[i] - 200_000, (E[i - 1] + EXIT + 50) if i else D[i] - 200_000) for i in range(n)]
    X = [e + EXIT for e in E]
    tau = [e + DELAY for e in E]
    span = (E[0] - 2_000_000, E[-1] + 20_000_000)
    return E, D, W, X, tau, [(c - H, c, c + H, 0) for c in range(span[0], span[1], 10_000_000)]


def events(E, D, W, X, extra=()):
    ev = [(w, "W", d) for w, d in zip(W, D)] + [(e, "E", None) for e in E] + [(x, "X", None) for x in X] + list(extra)
    return sorted(ev, key=lambda r: (r[0], {"W": 0, "E": 1, "X": 2}[r[1]]))


def evidence_for(lo, hi, armed_before=3 * 10**9, pre=10**9, post=10**9, tc=7, o0=-37693, update_gap=1_000_000):
    """A complete, well-formed timekeeping record around [lo, hi]: arm, settle, read, capture, read, confirm."""
    armed, s0, s1 = lo - armed_before, lo - pre, hi + post
    o = float(o0)
    for _ in range(round((s1 + 50_000 - s0) / 1e9)):
        o -= o / 2 ** (2 + tc)
    def state(t, off):
        return dict(read_status="ok", t_start=t, t_end=t + 50_000, tid=777, ret=0, offset=off, freq=1646056, status=0x2001, constant=tc, tick=10000, time_adjust=0)
    calls = [dict(t=s0 + 20_000, name="ntp_adjtimex", tkid=0, modes=0, tid=777), dict(t=s1 + 20_000, name="ntp_adjtimex", tkid=0, modes=0, tid=777)]
    updates = [dict(entry=t, ret=t + 3000, cpu=0) for t in range(armed + 500_000, s1 + 5_000_000, update_gap)]
    probes = {n: dict(enabled_before=True, enabled_after=True, hits=h, missed=0) for n, h in (("ntp_adjtimex", len(calls)), ("ntp_clear", 0), ("update_entry", len(updates)), ("update_return", len(updates)))}
    return dict(schema=1, kernel=dict(read_status="ok", release="7.0.0-42-generic", config_sha256=R.AUDITED["config_sha256"], tsc_khz=3187200, cmdline="BOOT_IMAGE=/boot/vmlinuz-7.0.0-42-generic root=UUID=x ro quiet splash"), clocksource=dict(read_status="ok", before="tsc", after="tsc"),
                log=dict(read_status="ok", covers=[armed - 10**9, s1 + 3 * 10**9], clock_events=[]),
                recorder=dict(read_status="ok", armed_t=armed, confirmed_t=s1 + 2 * 10**9, probes=probes, buffer=dict(read_status="ok", overrun=0, dropped=0), calls=calls, updates=updates, unmatched=dict(entries=[], returns=[])),
                state=dict(before=state(s0, o0), after=state(s1, round(o))),
                trace_cmd={use: dict(read_status="ok", path="/usr/bin/trace-cmd", **R.AUDITED_TRACE_CMD,
                                     runtime={**copy.deepcopy(R.AUDITED_TRACE_RUNTIME), "loaded": {"libtraceevent.so.1": "/usr/lib/x86_64-linux-gnu/libtraceevent.so.1"}})
                           for use in R.TRACE_CMD_USES})


def without(lst, k):
    return lst[:k] + lst[k + 1:]


def check(name, want, ev, tau, samp, must=None, model=M2, **kw):
    global COUNT
    COUNT += 1
    r = R.rule(ev, tau, samp, model, **kw)
    ok = r["status"] == want and (must is None or any(must in w for w in r["why"]))
    print(f"  {'ok ' if ok else 'BAD'} {name:70} {r['status']:4} {'; '.join(r['why'])[:90]}")
    if not ok:
        FAILS.append(name)
    return r


def claim(name, ok, note=""):
    global COUNT
    COUNT += 1
    print(f"  {'ok ' if ok else 'BAD'} {name:70} {note}")
    if not ok:
        FAILS.append(name)


K = 2000
CL = {K: lambda E: E[K - 1] + 500, K + 1: lambda E: E[K - 1] + 1000}
print("baselines:")
E, D, W, X, tau, samp = gen()
check("regular anchors", "PASS", events(E, D, W, X), tau, samp)
E, D, W, X, tau, samp = gen(cluster=CL)
check("a 500 ns cluster", "PASS", events(E, D, W, X), tau, samp)

print("the review of revision 10, in structured form:")
E1, X1 = without(E, K), without(X, K)
check("missing entry in the cluster, no duplicate", "FAIL", events(E1, D, W, X1), tau, samp, "two instance anchors")
dup = E[K - 1] + 300
E2, X2 = sorted(E1 + [dup]), sorted(X1 + [dup + EXIT])
check("missing entry and write, duplicate at +300 ns, no deadline", "FAIL", events(E2, without(D, K), without(W, K), X2), tau, samp, "no deadline written")
check("duplicate at +300 ns inside the previous interrupt", "FAIL", events(E2, D, [(w if k != K else E[K - 1] + 100) for k, w in enumerate(W)], X2), tau, samp, "inside the previous")
dup3 = E[K - 1] + 400
E3 = sorted(E1 + [dup3])
X3 = sorted(without(X, K)[:K - 1] + [E[K - 1] + 350] + [dup3 + 100] + X[K + 1:])
W3 = [(w if k != K else E[K - 1] + 360) for k, w in enumerate(W)]
r = check("LIMIT: substitution keeping slot, structure and a plausible stamp", "PASS", events(E3, D, W3, X3), tau, samp)
Eb, Db, Wb, Xb, tb, sb = gen(cluster={K: lambda E: E[K - 1] + 20_000})
E4, X4 = sorted(without(Eb, K) + [Eb[K - 1] + 400]), sorted(without(Xb, K) + [Eb[K - 1] + 720])
check("20 us variant: duplicate stamped before the armed deadline", "FAIL", events(E4, Db, [(w if k != K else Eb[K - 1] + 360) for k, w in enumerate(Wb)], X4), tb, sb, "before their deadline")

print("defects on both sides, separated, and at the boundaries:")
E, D, W, X, tau, samp = gen()
check("missing instance anchor", "FAIL", events(E, D, W, X), without(tau, K), samp, "no instance anchor")
check("duplicate instance anchor in the same slot", "FAIL", events(E, D, W, X), sorted(tau + [tau[K] + 200]), samp, "two instance anchors")
check("instance anchor moved into the next slot", "FAIL", events(E, D, W, X), sorted(without(tau, K) + [D[K + 1] + 10]), samp, "two instance anchors")
for sep in (1, 3, 10, 100, 1000):
    check(f"compensating: missing entry and extra entry {sep} apart", "FAIL", events(sorted(without(E, K) + [E[K + sep] + 1000]), D, W, sorted(without(X, K) + [E[K + sep] + 1200])), tau, samp)
check("missing entry near the start", "FAIL", events(without(E, 5), D, W, without(X, 5)), tau, samp)
check("missing entry near the end", "FAIL", events(without(E, len(E) - 5), D, W, without(X, len(X) - 5)), tau, samp)
check("first instance anchors missing (overlap only shortens)", "PASS", events(E, D, W, X), tau[3:], samp)
for s in (500, 1000, 5000, 20000):
    Ec, Dc, Wc, Xc, tc, sc = gen(cluster={K: lambda E, s=s: E[K - 1] + s, K + 1: lambda E, s=s: E[K - 1] + 2 * s})
    d = Ec[K - 1] + 300
    check(f"cluster {s} ns: missing entry and write, duplicate at +300 ns", "FAIL", events(sorted(without(Ec, K) + [d]), without(Dc, K), without(Wc, K), sorted(without(Xc, K) + [d + EXIT])), tc, sc, "no deadline written")
    check(f"LIMIT: cluster {s} ns: instance anchor replaced inside its slot", "PASS", events(Ec, Dc, Wc, Xc), sorted(without(tc, K) + [Ec[K] + 100]), sc)
    check(f"cluster {s} ns: instance anchor replaced before its main stamp", "FAIL", events(Ec, Dc, Wc, Xc), sorted(without(tc, K) + [Dc[K] + 10]), sc, "main stamp after")

print("periodic aliases:")
alt = [BASE]
for i in range(1, 4000):
    alt.append(alt[-1] + (538_500 if i % 2 else 461_500))
Da, Wa, Xa, ta = [e - LAT for e in alt], [alt[i - 1] + EXIT + 50 if i else alt[0] - LAT - 200_000 for i in range(4000)], [e + EXIT for e in alt], [e + DELAY for e in alt]
sa = [(c - H, c, c + H, 0) for c in range(alt[0] - 2_000_000, alt[-1] + 20_000_000, 10_000_000)]
check("alternating 538.5/461.5 us", "PASS", events(alt, Da, Wa, Xa), ta, sa)
t2 = list(ta)
t2[2000] += 2300
check("alternating, one 2.3 us stamp disagreement (as H2 attempt 7)", "PASS", events(alt, Da, Wa, Xa), t2, sa)

print("sampler coverage:")
E, D, W, X, tau, samp = gen()
gap = [s for s in samp if not (E[1500] < s[1] < E[1500] + 200_000_000)]
check("a long sampler gap inside the run: coverage fails", "FAIL", events(E, D, W, X), tau, gap, "overlap the run", window=(E[100], E[3900]))
check("the same gap outside the run window passes", "PASS", events(E, D, W, X), tau, gap, window=(E[3000], E[3900]))
check("irregular sampling within the allowed gap", "PASS", events(E, D, W, X), tau, [s for k, s in enumerate(samp) if k % 3], window=(E[100], E[3900]))
check("bracket extremes (+/-400 ns) only widen the bound", "PASS", events(E, D, W, X), tau, [(c - 400, c, c + 400, 0) for _, c, _, _ in samp])
fake = [(a + 5_000_000_000, c + 5_000_000_000, b + 5_000_000_000, x) for a, c, b, x in samp]
check("no sampler data anywhere in the run: an explicit coverage failure", "FAIL", events(E, D, W, X), tau, fake, "no usable conversion pairs", window=(E[100], E[-1]))


def bump(lst, lo, hi, d):
    return [v + d if lo <= v <= hi else v for v in lst]


print("local stamp and map errors (the sign checks are necessary conditions, not numeric bounds):")
lo, hi = E[1000], E[1200]
check("excursion +1 us in the main stamps, 200 anchors", "FAIL", events(bump(E, lo, hi, 1000), D, W, bump(X, lo, hi, 1000)), tau, samp, "main stamp after")
check("excursion -1 us in the main stamps, 200 anchors", "FAIL", events(bump(E, lo, hi, -1000), D, W, bump(X, lo, hi, -1000)), tau, samp, "before their deadline")
check("LIMIT: excursion -100 ns, inside the bound and the latency floor", "PASS", events(bump(E, lo, hi, -100), D, W, bump(X, lo, hi, -100)), tau, samp)
check("LIMIT: stamp delay +0.9 us masking a map shift over 200 anchors", "PASS", events(E, D, W, X), [t + 900 if lo <= t <= hi else t for t in tau], samp)
def loc(d):
    return [(a + d, c, b + d, x) if lo <= c <= hi else (a, c, b, x) for a, c, b, x in samp]
check("the review's local wrong map: +1 us over 200 anchors", "FAIL", events(E, D, W, X), tau, loc(1000), "before their deadline")
check("the review's local wrong map: -1 us over 200 anchors", "FAIL", events(E, D, W, X), tau, loc(-1000), "main stamp after")

print("deadline disarm and pending interrupts (zero writes), and trace boundaries:")
ee = events(E, D, W, X)
check("cancel before expiry, then the interrupt with no later write (the review's case)", "FAIL", sorted(ee + [(E[2000] - 1000, "W", 0)], key=lambda r: r[0]), tau, samp, "disarmed before")
Dp = list(D)
Dp[2000] = E[2000] - 5000
Wp = list(W)
Wp[2000] = E[2000] - 100_000
check("deadline expired, then a zero write, then the pending interrupt", "PASS", sorted(events(E, Dp, Wp, X) + [(E[2000] - 1000, "W", 0)], key=lambda r: r[0]), tau, samp)
check("zero write within the bound of expiry: cannot tell, so it fails", "FAIL", sorted(events(E, Dp, Wp, X) + [(E[2000] - 5000 + 10, "W", 0)], key=lambda r: r[0]), tau, samp, "disarmed before")
check("cancelled, re-armed: the later deadline delimits the slot", "PASS", sorted(ee + [(W[2000] + 10, "W", 0), (W[2000] + 20, "W", D[2000])], key=lambda r: (r[0], r[2] is None)) , tau, samp)
check("a zero write with nothing armed changes nothing", "PASS", sorted(ee + [(E[2000] + EXIT + 10, "W", 0)], key=lambda r: r[0]), tau, samp)
noexit = [r for r in ee if r != (E[-1] + EXIT, "X", None)]
r = check("final entry without its exit: it leaves the span", "PASS", noexit, tau, samp)
claim("  ... and is counted, not claimed as closed", r.get("unclosed_tail") == 1 and r["used"] == len(E) - 1, f"unclosed_tail {r.get('unclosed_tail')}, used {r.get('used')}")
check("final entry without its exit, window needs it: coverage fails", "FAIL", noexit, tau, samp, "do not cover", window=(E[100], E[-1]))
check("an exit missing inside the span fails", "FAIL", [r for r in ee if r != (E[2000] + EXIT, "X", None)], tau, samp, "no exit")
check("an exit before any entry (the trace started inside one) is allowed", "PASS", [(E[0] - 100_000, "X", None)] + ee, tau, samp)
check("an exit with no entry inside the span fails", "FAIL", sorted(ee + [(E[2000] + EXIT + 100, "X", None)], key=lambda r: r[0]), tau, samp, "exits with no entry")

print("strict parsing (every record is taken or reported):")


def led(rows, back_ns=0):
    ledger = R.Ledger(back_ns=back_ns)
    for t, name, body in rows:
        ledger.feed(t, name, body)
    return ledger


GOOD_ROWS = [(100, "local_timer_entry", "vector=236"), (200, "write_msr", "6e0, value 2723d67a9bc2"), (300, "local_timer_exit", "vector=236")]
ledger_ok = led(GOOD_ROWS)
claim("well-formed records are taken completely", len(ledger_ok.events) == 3 and not ledger_ok.problems)
ENTRY = GOOD_ROWS[0]
for name, rows, word in (("another MSR", [ENTRY, (200, "write_msr", "6e1, value 10")], "malformed deadline"), ("a failed write", [ENTRY, (200, "write_msr", "6e0, value 10 #GP")], "malformed deadline"), ("a truncated value", [ENTRY, (200, "write_msr", "6e0, value")], "malformed deadline"),
                         ("an unexpected vector", [(100, "local_timer_entry", "vector=235")], "unexpected vector"), ("a duplicate record", [ENTRY, ENTRY], "duplicate"), ("an out-of-order record", [(300, "local_timer_exit", "vector=236"), ENTRY], "out of order")):
    claim(f"the ledger reports {name}", any(word in x for x in led(rows).problems), "; ".join(led(rows).problems)[:70])
ledger_b = led([(300, "local_timer_exit", "vector=236"), ENTRY], back_ns=500)
claim("a backward step within an allowance is counted, not hidden", ledger_b.backward_steps == 1 and not ledger_b.problems)
inst_ok = "# entries-in-buffer/entries-written: 2/2\n <idle>-0 [003] d.h1. 5000: local_timer_entry: vector=236\n <idle>-0 [003] d.h1. 6000: local_timer_entry: vector=236"
t, p = R.parse_instance(inst_ok)
claim("the instance parses completely", t == [5000, 6000] and not p)
for name, txt, word in (("lost entries", inst_ok.replace("2/2", "2/3"), "lost"), ("a malformed row", inst_ok + "\nxx", "malformed"), ("a duplicate", inst_ok + "\n <idle>-0 [003] d.h1. 6000: local_timer_entry: vector=236", "duplicate"), ("an out-of-order row", inst_ok + "\n <idle>-0 [003] d.h1. 5500: local_timer_entry: vector=236", "out of order"), ("another CPU", inst_ok + "\n <idle>-0 [002] d.h1. 7000: local_timer_entry: vector=236", "unexpected")):
    t, p = R.parse_instance(txt)
    claim(f"instance parser reports {name}", any(word in x for x in p), "; ".join(p)[:70])
pairs_ok = "# tsc_mono_pairs cpu=1\n100,5000,140,1\n200,5300,230,3\n"
q, p = R.parse_pairs(pairs_ok)
claim("the sampler file parses completely", q == [(100, 5000, 140, 1), (200, 5300, 230, 3)] and not p)
for name, txt, word in (("a malformed row", pairs_ok + "x,y\n", "malformed"), ("a short row", pairs_ok + "1,2,3\n", "malformed"), ("a negative bracket", pairs_ok + "500,1,400,1\n", "ends before")):
    q, p = R.parse_pairs(txt)
    claim(f"sampler parser reports {name}", any(word in x for x in p), "; ".join(p)[:70])

print("time-varying CPU offsets (the main stamps, deadlines and instance stamps all read CPU 3's TSC):")
SPAN = 2_000_000_000


def gen_cpu(delta, n=4000, step=500_000):
    P = [BASE + i * step for i in range(n)]
    def T3(p):
        return int(round(p + delta(p)))
    return [T3(p) for p in P], [T3(p - LAT) for p in P], [T3(p - 200_000) for p in P], [T3(p + EXIT) for p in P], [T3(p + DELAY) for p in P], [(c - H, c, c + H, 0) for c in range(P[0] - 2_000_000, P[-1] + 20_000_000, 10_000_000)]


for name, d in (("ramp to +2 us", lambda p: (p - BASE) / SPAN * 2000), ("ramp to -2 us", lambda p: -(p - BASE) / SPAN * 2000), ("step +1 us between interrupts", lambda p: 1000 if p >= BASE + SPAN / 2 + 250_000 else 0), ("+/-1 us sine, 100 ms period", lambda p: 1000 * math.sin(2 * math.pi * (p - BASE) / 1e8))):
    E, D, W, X, tau, samp = gen_cpu(d)
    r = check(f"CPU 3 offset, {name}", "PASS", events(E, D, W, X), tau, samp)
    claim("  ... leaves e unchanged", abs(r["emin"] + DELAY) <= 30, f"emin {r['emin']:.0f}")
Pk = BASE + 2000 * 500_000
E, D, W, X, tau, samp = gen_cpu(lambda p: 1000 if p >= Pk + 30 else 0)
check("LIMIT: CPU 3 offset steps +1 us between one interrupt's two stamps", "PASS", events(E, D, W, X), tau, samp)
E, D, W, X, tau, samp = gen_cpu(lambda p: -200 if p >= Pk + 30 else 0)
check("CPU 3 offset steps -200 ns between one interrupt's two stamps", "FAIL", events(E, D, W, X), tau, samp, "main stamp after")
E, D, W, X, tau, samp = gen_cpu(lambda p: 0)
check("map drifts from CPU 3's clock, ramp to +2 us", "FAIL", events(E, D, W, X), tau, [(a + (c - BASE) / SPAN * 2000, c, b + (c - BASE) / SPAN * 2000, x) for a, c, b, x in samp], "before their deadline")
check("map drifts from CPU 3's clock, ramp to -2 us", "FAIL", events(E, D, W, X), tau, [(a - (c - BASE) / SPAN * 2000, c, b - (c - BASE) / SPAN * 2000, x) for a, c, b, x in samp], "main stamp after")


# --- the bound itself ------------------------------------------------------------------------------------------------

def actual_within(name, r, truth, queries, also_local=True):
    """Every query's actual conversion error against the scalar U classification would consume, and the local bound."""
    worst, worst_r, n = 0.0, 0.0, 0
    for q in queries:
        c = r["conv"].at(q)
        if c is None:
            continue
        n += 1
        err = abs(c[0] - truth(q))
        worst = max(worst, err)
        worst_r = max(worst_r, err / c[1])
        if err > r["U"] + 1e-6 or (also_local and err > c[1] + 1e-6):
            claim(name, False, f"error {err:.1f} at {q}, U {r['U']:.1f}, local {c[1]:.1f}")
            return worst
    claim(name, n > 0, f"{n} queries, largest error {worst:.1f} ns, U {r['U']:.1f}; largest error/local bound {worst_r:.2f}")
    return worst


def with_calls(ev, extra):
    ev = copy.deepcopy(ev)
    ev["recorder"]["calls"].extend(copy.deepcopy(extra))
    for c in extra:
        ev["recorder"]["probes"][c["name"]]["hits"] += 1
    return ev


print("the bound: the review of revision 29's counterexamples, asserting actual error against U:")
N = 4000
ent = [BASE + i * 500_000 for i in range(N)]
keep = [i for i in range(N) if not 1500 <= i < 1901]
ev0 = sorted(sum(([(ent[i] - 200_350, "W", ent[i] - 350), (ent[i], "E", None), (ent[i] + 320, "X", None)] for i in keep), []))
inst0 = [ent[i] + 60 for i in keep]
samp0 = [(t - 20, t, t + 20, 0) for t in range(BASE - 2_000_000, ent[-1] + 80_000_000, 40_000_000)]
lo5, hi5 = BASE + 798_000_000, BASE + 838_000_000
q = (lo5 + hi5) // 2
def tri(x, height):
    return 0 if not lo5 <= x <= hi5 else height * (1 - abs(x - q) / (q - lo5))
M5 = R.Model(5e-6)
ev1 = sorted(ev0 + [(q - 100_000 + round(tri(q - 100_000, 50)), "W", q)])
r = check("model-consistent hidden excursion, 5 ppm: 201 ms interval", "PASS", ev1, inst0, samp0, model=M5, window=(BASE + 100_000_000, BASE + 1_900_000_000))
def truth(x):
    return x + tri(x, 50)
w = actual_within("  actual error at the recorded deadline is within U (the old U was 81 ns)", r, truth, [q])
claim("  ... and that error is the review's 110 ns", abs(w - 110) < 1, f"{w:.1f}")
actual_within("  and at every deadline and anchor", r, truth, [ent[i] - 350 for i in keep] + inst0 + [q])
r5 = R.analyze((ev1, []), (inst0, []), samp0, window=(BASE + 100_000_000, BASE + 1_900_000_000), model=R.Model(5e-6))
claim("  without evidence the analysis is exploratory and not accepted", r5["cert"] == "UNSUPPORTED" and r5["U_label"] == "exploratory" and r5["accepted"] is False, f"{r5['cert']}, {r5['U_label']}, {r5['cert_reasons'][0]}")
for height in (5000, 10000):
    ev2 = sorted(ev0 + [(q - 100_000 + round(tri(q - 100_000, height)), "W", q)])
    r2 = R.rule(ev2, inst0, samp0, M5, window=(BASE + 100_000_000, BASE + 1_900_000_000))
    c = r2["conv"].at(q)
    claim(f"OUTSIDE THE MODEL: {height} ns excursion ({4 * height / 40e6 * 1e6:.0f} ppm): passes, and exceeds U", r2["status"] == "PASS" and abs(c[0] - (q + height)) > r2["U"], f"error {abs(c[0] - (q + height)):.0f}, U {r2['U']:.0f}; realizable only with a recorded timekeeping call")
    claim("  ... so a certificate requires that record: one such call in the interval is UNSUPPORTED", R.certify(with_calls(evidence_for(BASE, BASE + 2 * 10**9), [dict(t=BASE + 800_000_000, name="ntp_adjtimex", tkid=0, modes=0x2001, tid=99)]))["status"] == "UNSUPPORTED")
truth2 = [BASE + i * 500_000 for i in range(4000)]
truth2[K], truth2[K + 1] = truth2[K - 1] + 500, truth2[K - 1] + 1000
main2 = list(truth2)
main2[K] = truth2[K - 1] + 400
X2 = [e + 320 for e in truth2]
X2[K - 1], X2[K] = truth2[K - 1] + 350, main2[K] + 100
W2 = [max(truth2[i] - 350 - 200_000, truth2[i - 1] + 370 if i else truth2[i] - 350 - 200_000) for i in range(4000)]
W2[K] = truth2[K - 1] + 360
D2 = [e - 350 for e in truth2]
ev3 = sorted([(w, "W", d) for w, d in zip(W2, D2)] + [(e, "E", None) for e in main2] + [(x, "X", None) for x in X2])
inst2 = [e + 60 for e in truth2]
s2 = [(t - 20, t, t + 20, 0) for t in range(BASE - 1_000_000, truth2[-1] + 20_000_000, 10_000_000)]
r = check("the named in-slot substitution, no pair left out", "PASS", ev3, inst2, s2, model=M5, window=(truth2[100], truth2[3900]), cap=0.0)
wq = actual_within("  actual error at the substituted anchor is within U (the old U was 101 ns)", r, lambda x: x, [inst2[K]])
claim("  ... and that error is the review's 160 ns", abs(wq - 160) < 1, f"{wq:.1f}")
r = check("the same substitution under the default ceiling", "PASS", ev3, inst2, s2, model=M5, window=(truth2[100], truth2[3900]))
actual_within("  the bound still holds at the substituted anchor, left out or not", r, lambda x: x, [inst2[K]])

print("the bound: the realizable class (source audit), random captures, every converted value:")


def realizable(seed):
    """f is what the audited timekeeping code can produce with no timekeeping call: updates each ms; slope m or m+1 units; the tick length moves once a second."""
    rng = random.Random(seed)
    unit = 0.19e-6
    tc, o0 = rng.choice([4, 5, 6, 7]), rng.uniform(-700e3, 700e3)
    x0, secs = BASE, 6
    ks = [x0 + i * 1_000_000 for i in range(secs * 1000 + 1)]
    fs, f, o, slopes, stale = [], float(x0), o0, [], []
    mode = rng.choice(["dither", "adversarial", "steady"])
    for i in range(len(ks) - 1):
        if i % 1000 == 0 and i:
            o -= o / 2 ** (2 + tc)
        chunk = o / 2 ** (2 + tc) / 1e9
        bit = {"dither": rng.random() < 0.5, "adversarial": (i // 100) % 2 == 0, "steady": False}[mode]
        s = 1 + chunk + bit * unit
        fs.append(f)
        slopes.append(s)
        stale.append((rng.uniform(0, 30_000) if rng.random() < 0.3 else 0) if rng.random() > 0.01 else rng.uniform(1e6, 10e6))  # how long a reader may use the old snapshot, up to the operational maximum
        f += s * 1_000_000
    fs.append(f)
    chunk_range = abs(o0 - o) / 2 ** (2 + tc) / 1e9
    ds = 2 * unit + chunk_range

    def S(x):
        k = min(max(bisect.bisect_right(ks, x) - 1, 0), len(fs) - 2)
        return fs[k] + slopes[k] * (x - ks[k])

    def M(x):
        k = min(max(bisect.bisect_right(ks, x) - 1, 0), len(fs) - 2)
        return fs[k] + (slopes[k - 1] if k and x - ks[k] < stale[k] else slopes[k]) * (x - ks[k])

    x, end = x0 + 50_000_000, x0 + secs * 1e9 - 60_000_000
    ev, tau, dls = [], [], []
    px, d_prev = None, 0.0
    while x < end:
        lat = rng.uniform(100, 1500)
        d = rng.uniform(40, 300) if rng.random() > 0.002 else rng.uniform(1500, 2100)
        if px is not None:  # A2: the kernel arms the next deadline after this interrupt's stamps, so it cannot precede them
            x = max(x, px + d_prev + 400 + lat)
        deadline = int(x - lat)
        tw = deadline - int(rng.uniform(20, 200))
        if px is not None:
            tw = max(tw, int(px + d_prev + 350))
        ev.append((int(M(tw)), "W", deadline))
        ev.append((int(M(x)), "E", None))
        ev.append((int(M(x)) + 300, "X", None))
        tau.append(int(x + d))
        dls.append(deadline)
        px, d_prev = x, d
        x += rng.choice([rng.uniform(300e3, 3e6), rng.uniform(300e3, 3e6), rng.uniform(300e3, 3e6), 800.0 if rng.random() < 0.05 else rng.uniform(300e3, 3e6)])
        if rng.random() < 0.002:
            x += rng.uniform(100e6, 800e6)
    ev.sort(key=lambda r: (r[0], {"W": 0, "E": 1, "X": 2}[r[1]]))
    samp, c = [], x0 + 10_000_000
    while c < x0 + secs * 1e9 - 2_000_000:
        h1, h2 = rng.uniform(10, 50), rng.uniform(10, 50)
        cc = int(c)
        samp.append((int(S(cc - h1)), cc, int(S(cc + h2)), 0))
        c += rng.choice([rng.uniform(5e6, 15e6), rng.uniform(5e6, 80e6)])
    return ev, tau, samp, M, dls, ds, mode


bad = 0
wr = []
for seed in range(60):
    ev, tau, samp, M_, dls, ds, mode = realizable(seed)
    m = R.Model(ds)
    r = R.rule(ev, tau, samp, m)
    if r["status"] != "PASS":
        print(f"  seed {seed} mode {mode}: {r['status']} {r['why'][:2]}")
        bad += 1
        continue
    n = 0
    for q in dls + tau:
        c = r["conv"].at(q)
        if c is None:
            continue
        n += 1
        err = abs(c[0] - M_(q))
        wr.append(err / c[1])
        if err > r["U"] or err > c[1]:
            print(f"  seed {seed} mode {mode}: error {err:.1f} > bound {c[1]:.1f} (U {r['U']:.1f})")
            bad += 1
            break
claim("60 random realizable captures: every error within U and the local bound", bad == 0 and len(wr) > 1000, f"{len(wr)} queries; largest error/bound {max(wr):.2f}")

print("the bound: selection, self-consistency and failure:")
E, D, W, X, tau, samp = gen()
ee = events(E, D, W, X)
t3 = list(tau)
for k in (800, 1800, 2800):
    t3[k] += 4000
r = check("three 4 us stamp disagreements are left out and U falls", "PASS", ee, t3, samp)
claim("  U is the largest bound over the final retained set", r["U"] == max(r["conv"].b), f"U {r['U']:.1f}")
claim("  the left-out pairs stay counted", len(r["left_out"]) == 3 and r["used"] == 3997, f"left out {r['left_out']}")
t4 = list(tau)
for k in range(1000, 3500, 400):
    t4[k] += 5500
t4[1000] += 0
r = R.rule(ee, t4, samp, M2)
claim("  more large disagreements than the 0.1% ceiling: U over the limit fails, with the budget named", r["status"] == "FAIL" and any("exceeds" in w for w in r["why"]), "; ".join(r["why"])[:80])
t5 = list(tau)
t5[2000] = E[2000] - 200  # still in its slot, stamped before the main stamp by enough to break the sign check
r = R.rule(ee, t5, samp, M2)
claim("  a sign failure is not concealed by leaving that pair out", r["status"] == "FAIL", "; ".join(r["why"])[:80])
ok_steps = True
for _ in range(1):
    items = [dict(tsc=i * 1000, t=i * 1000, e=-10.0 - i, B=1.0, E=11.0 + i) for i in range(5000)]
    kept, out, b = R.select(items, M2, 5)
    ok_steps = len(out) <= 5 and len(kept) + len(out) == 5000 and abs(max(b) - max(R.interval_bound(a, c, M2) for a, c in zip(kept, kept[1:]))) < 1e-9
claim("  selection is finite, within its ceiling, and returns its own final U", ok_steps)
# A model violation is caught where a sampler pair can see it.
E, D, W, X, tau, samp = gen()
hole = [i for i in range(len(E)) if not 1500 <= i < 1901]
def pick(v):
    return [v[i] for i in hole]
lo6, hi6 = E[1500] + 30_000_000, E[1900] - 30_000_000
sv = [(a + 1000, c, b + 1000, x) if lo6 <= c <= hi6 else (a, c, b, x) for a, c, b, x in samp]
r = R.rule(events(pick(E), pick(D), pick(W), pick(X)), pick(tau), sv, M2)
claim("  a sampler pair outside the bound, where no anchor can absorb it: a model violation", r["status"] == "FAIL" and any("outside the bound" in w for w in r["why"]), "; ".join(r["why"])[:80])
r = R.rule(events(pick(E), pick(D), pick(W), pick(X)), pick(tau), samp, M2)
claim("  a 201 ms hole in the anchors is charged to U, with no separate limit", r["status"] == "PASS" and abs(r["Imax"] - 2e-6 * 201e6 / 4) < 2, f"Imax {r['Imax']:.1f}, U {r['U']:.1f}")

print("end to end: the certificate binds the calculation:")
E, D, W, X, tau, samp = gen()
EE, MP, IP, WIN = events(E, D, W, X), None, (tau, []), (E[100], E[3900])
MP = (EE, [], {})
SLO, SHI = samp[0][0], samp[-1][2]
GOOD = evidence_for(SLO, SHI)


def run(ev=GOOD, model=None, mp=None, ip=None, s=None, win=WIN):
    return R.analyze(mp if mp is not None else MP, ip if ip is not None else IP, s if s is not None else samp, window=win, evidence=ev, model=model)


r = run()
c = R.certify(GOOD)
claim("complete evidence: accepted, with the model from the certificate", r["accepted"] is True and r["U_label"] == "validated" and r["model"]["source"] == "certificate" and r["model"]["ds"] == c["ds"] and r["model"]["W_ns"] == R.W_OP_NS, f"ds {r['model']['ds'] * 1e6:.4f} ppm, R {r['model']['R_ns']:.2f}, U {r['U']:.1f}")
claim("the parameters and their provenance are returned with U", set(r["provenance"]) >= {"audit", "mult", "shift", "observed_max_update_ns"} and r["provenance"]["mult"] == 5263936, str(r["provenance"]))
claim("explicit units and rounding: 2 units of mult plus the offset term", abs(c["ds"] - (2 * 1.001 / 5263936 + c["provenance"]["chunk_ppm"] / 1e6)) < 1e-15, f"unit {c['provenance']['unit_ppm']:.4f} ppm, chunk {c['provenance']['chunk_ppm']:.5f} ppm")
for name, m, why in (("ds 1e-12, the review's case", R.Model(1e-12), "smaller"), ("0.4 ppm with W = 0, the review's case", R.Model(0.4e-6, W_ns=0), "smaller"), ("a slope just below the certificate's", R.Model(c["ds"] * 0.999), "smaller")):
    r = run(model=m)
    claim(f"a smaller supplied model is not validated: {name}", r["accepted"] is False and r["U_label"] == "exploratory" and r["cert"] == "UNSUPPORTED" and any(why in x for x in r["cert_reasons"]), f"U {r['U']:.1f}, {r['model']['source']}")
r = run(model=R.Model(1e-6, W_ns=20_000_000))
claim("a larger supplied model stays validated, and says so", r["accepted"] is True and r["U_label"] == "validated" and "not smaller" in r["model"]["source"], r["model"]["source"])
r = R.analyze(MP, IP, samp, window=WIN, evidence=None, model=None)
claim("no certificate and no model: not analysed", r["status"] == "NOT ANALYSED" and r["accepted"] is False, r["why"][0])
r = R.analyze(MP, IP, samp, window=WIN, evidence=None, model=R.Model(5e-6, label="certified"))
claim("a caller's label cannot override an unsupported result", r["accepted"] is False and r["U_label"] == "exploratory" and r["model"]["source"].startswith("exploratory"), f"{r['U_label']}, {r['model']['source']}")
r = R.analyze((EE, ["line 9: malformed row"], {}), IP, samp, window=WIN, evidence=GOOD)
claim("a parse problem fails the analysis and is not merely kept", r["status"] == "FAIL" and r["accepted"] is False and r["why"][0].startswith("line 9"), r["why"][0])
r = R.analyze(MP, (tau, ["the instance lost 1 entries"]), samp, window=WIN, evidence=GOOD)
claim("an instance parse problem fails the analysis", r["status"] == "FAIL" and r["accepted"] is False, r["why"][0])
# A certified analysis whose deadline-slot accounting fails (attempt 13's case): its U is printed as a candidate with
# the conversion not accepted, never as validated, and the certificate stays CERTIFIED apart; the numbers are the rule's.
r = run(ip=(sorted(tau + [tau[K] + 200]), []))
r0 = R.rule(EE, sorted(tau + [tau[K] + 200]), samp, R.Model(c["ds"], c["W_ns"], label="certified"), window=WIN)
claim("a certified U whose slot accounting failed is a candidate, not accepted", r["status"] == "FAIL" and r["cert"] == "CERTIFIED"
      and r["accepted"] is False and r["U"] is not None and r["U"] == r0["U"] and r["U_label"] == "candidate U; conversion not accepted"
      and any("two instance anchors" in w for w in r["why"]), f"{r['status']}, {r['U_label']}, U {r['U']}")


def drop(ev, path):
    ev = copy.deepcopy(ev)
    node = ev
    keys = path.split(".")
    for k in keys[:-1]:
        node = node[k]
    del node[keys[-1]]
    return ev


def put(ev, path, value):
    ev = copy.deepcopy(ev)
    node = ev
    keys = path.split(".")
    for k in keys[:-1]:
        node = node[k]
    node[keys[-1]] = value
    return ev


print("end to end: strict evidence (missing, unreadable or malformed is UNSUPPORTED, never empty):")
PF = ("enabled_before", "enabled_after", "hits", "missed")
SF = ("read_status", "t_start", "t_end", "tid", "ret", "offset", "freq", "status", "constant", "tick", "time_adjust")
missing = ["schema", "kernel", "kernel.read_status", "kernel.release", "kernel.config_sha256", "kernel.tsc_khz", "kernel.cmdline", "clocksource", "clocksource.read_status", "clocksource.before", "clocksource.after", "log", "log.read_status", "log.covers", "log.clock_events",
           "recorder", "recorder.read_status", "recorder.armed_t", "recorder.confirmed_t", "recorder.probes", "recorder.buffer", "recorder.buffer.read_status", "recorder.buffer.overrun", "recorder.buffer.dropped", "recorder.calls", "recorder.updates", "recorder.unmatched", "recorder.unmatched.entries", "recorder.unmatched.returns", "state", "state.before", "state.after"]
missing += [f"recorder.probes.{n}" for n in R.PROBES] + [f"recorder.probes.{n}.{f}" for n in R.PROBES for f in PF] + [f"state.{side}.{f}" for side in ("before", "after") for f in SF]
missing += ["trace_cmd"] + [f"trace_cmd.{u}" for u in R.TRACE_CMD_USES] + [f"trace_cmd.{u}.{f}" for u in R.TRACE_CMD_USES for f in ("read_status", "runtime", *R.AUDITED_TRACE_CMD)]
bad = [k for k in missing if (lambda r: not (r["accepted"] is False and r["cert"] == "UNSUPPORTED" and r["U_label"] != "validated"))(run(drop(GOOD, k)))]
claim(f"{len(missing)} nested fields each missing: never validated", not bad, "; ".join(bad)[:90])
for name, path, value, word in (("a failed read", "log.read_status", "failed", "not read"), ("recorder calls unreadable", "recorder.read_status", "error", "not read"), ("calls is not a list", "recorder.calls", "none", "wrong type"), ("calls is null", "recorder.calls", None, "wrong type"),
                                ("clock events is not a list", "log.clock_events", "", "wrong type"), ("log range reversed", "log.covers", [5, 1], "time range"), ("log range in the wrong shape", "log.covers", [1, 2, 3], "time range"), ("a string in place of a time", "recorder.armed_t", "12", "wrong type"),
                                ("a float in place of a count", "recorder.probes.update_entry.missed", 0.0, "wrong type"), ("a bool in place of a count", "recorder.buffer.overrun", False, "wrong type"), ("an out-of-range constant", "state.before.constant", 11, "out of range"), ("an absurd TSC rate", "kernel.tsc_khz", 5, "out of range"),
                                ("a malformed call", "recorder.calls", [dict(t=1, name="ntp_adjtimex")], "malformed"), ("an unknown call name", "recorder.calls", [dict(t=1, name="x", tkid=0, modes=0, tid=1)], "malformed"), ("an out-of-range tkid", "recorder.calls", [dict(t=1, name="ntp_clear", tkid=99, modes=0, tid=1)], "malformed"),
                                ("a malformed update", "recorder.updates", [dict(entry=5, ret=1, cpu=0)], "malformed"), ("non-numeric unmatched times", "recorder.unmatched.entries", ["x"], "other than times"), ("non-text log events", "log.clock_events", [1], "other than messages"), ("a mapping missing altogether", "state", "none", "missing"), ("a command line that is not text", "kernel.cmdline", 7, "wrong type"), ("a frequency past the kernel's limit", "state.before.freq", 32_768_001, "out of range")):
    c = R.certify(put(GOOD, path, value))
    claim(f"unsupported: {name}", c["status"] == "UNSUPPORTED" and any(word in x for x in c["reasons"]), "; ".join(c["reasons"])[:80])
c = R.certify(put(GOOD, "schema", 2))
claim("unsupported: another schema", c["status"] == "UNSUPPORTED", c["reasons"][0])
for name, patch, word in (("a different kernel", ("kernel.release", "7.0.0-43-generic"), "audited"), ("another configuration", ("kernel.config_sha256", "0"), "audited"), ("clocksource changed", ("clocksource.after", "hpet"), "clocksource"),
                          ("a clocksource event in the log", ("log.clock_events", ["Switched to clocksource hpet"]), "kernel log"), ("log does not cover the interval", ("log.covers", [GOOD["state"]["before"]["t_start"] + 1, GOOD["state"]["after"]["t_end"]]), "does not cover"),
                          ("a pending slew", ("state.before.time_adjust", 10), "slew"), ("PPS status", ("state.before.status", 0x2101), "status"), ("a leap pending", ("state.after.status", 0x2011), "status"), ("microsecond mode", ("state.after.status", 0x0001), "status"),
                          ("frequency changed", ("state.after.freq", 1), "freq"), ("constant changed", ("state.after.constant", 6), "constant"), ("tick changed", ("state.after.tick", 9999), "tick"), ("offset that the recurrence does not produce", ("state.after.offset", 5000), "recurrence"),
                          ("probe disabled after", ("recorder.probes.ntp_adjtimex.enabled_after", False), "not confirmed"), ("probe never confirmed before", ("recorder.probes.update_return.enabled_before", False), "not confirmed"), ("a probe miss", ("recorder.probes.update_return.missed", 1), "missed"), ("buffer overrun", ("recorder.buffer.overrun", 1), "overrun"), ("dropped events", ("recorder.buffer.dropped", 3), "dropped"),
                          ("hit count disagrees", ("recorder.probes.ntp_adjtimex.hits", 99), "records are present"), ("recorder not confirmed after", ("recorder.confirmed_t", GOOD["state"]["after"]["t_end"]), "confirmed after"), ("armed after the first read", ("recorder.armed_t", GOOD["state"]["before"]["t_start"]), "armed before")):
    c = R.certify(put(GOOD, *patch))
    claim(f"unsupported: {name}", c["status"] == "UNSUPPORTED" and any(word in x for x in c["reasons"]), "; ".join(c["reasons"])[:80])
c = R.certify(put(GOOD, "recorder.calls", GOOD["recorder"]["calls"][1:]))
claim("unsupported: the first state read is not in the record", c["status"] == "UNSUPPORTED" and any("before state read is not" in x for x in c["reasons"]), "; ".join(c["reasons"])[:90])

print("end to end: the audited trace-cmd, for every use the certificate rests on:")
claim("the audited build is named in the provenance", R.certify(GOOD)["provenance"]["trace_cmd"] == "trace-cmd 3.3.3-1ubuntu2", str(R.certify(GOOD)["provenance"].get("trace_cmd")))
for use, desc in R.TRACE_CMD_USES.items():
    for name, patch, word in (("another version", (f"trace_cmd.{use}.version", "3.3.4-1"), "not the audited build"), ("another build of the same version", (f"trace_cmd.{use}.sha256", "0" * 64), "not the audited build"),
                              ("another architecture", (f"trace_cmd.{use}.architecture", "arm64"), "not the audited build"), ("another package", (f"trace_cmd.{use}.package", "trace-cmd-git"), "not the audited build"),
                              ("an identity that could not be read", (f"trace_cmd.{use}.read_status", "the package that installed /usr/local/bin/trace-cmd is not known"), "not identified"),
                              ("no identity at all", (f"trace_cmd.{use}", {"read_status": "the trace-cmd that reported the main trace is not identified"}), "not identified"),
                              ("a version in the wrong form", (f"trace_cmd.{use}.version", 333), "wrong type")):
        r = run(put(GOOD, *patch))
        claim(f"unsupported: the trace-cmd {desc}: {name}", r["accepted"] is False and r["cert"] == "UNSUPPORTED" and any(word in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:80])
r = run(put(GOOD, "trace_cmd", {"read_status": "ok", "version": "unaudited-new-version"}))
claim("unsupported: an identity in the implementation review's form, naming no use", r["accepted"] is False and r["cert"] == "UNSUPPORTED", "; ".join(r["cert_reasons"])[:80])
r = run(put(GOOD, "trace_cmd.report", dict(GOOD["trace_cmd"]["report"], path="/opt/replay/bin/trace-cmd")))
claim("the same build at another path is the audited build", r["accepted"] is True, str(r["cert_reasons"]))
claim("the audited runtime is named in the provenance", R.certify(GOOD)["provenance"]["trace_runtime"] == "libtraceevent1 1:1.8.7-1, libtracefs1 1.8.2-1ubuntu1, 13 plugins",
      str(R.certify(GOOD)["provenance"].get("trace_runtime")))
TE, TF, HR = "libtraceevent.so.1", "libtracefs.so.1", f"{R._TE_PLUGINS}/plugin_hrtimer.so"
EXTRA = dict(path="/opt/plugins/plugin_extra.so", sha256="1" * 64, package=None, version=None, architecture=None)


def at(ev, keys, value=None, delete=False):
    """A copy of EV with the field at KEYS (a list: sonames and paths contain dots) set to VALUE, or deleted."""
    ev = copy.deepcopy(ev)
    node = ev
    for k in keys[:-1]:
        node = node[k]
    if delete:
        del node[keys[-1]]
    else:
        node[keys[-1]] = value
    return ev


for use, desc in R.TRACE_CMD_USES.items():
    rt = ["trace_cmd", use, "runtime"]
    for name, keys, value, word in (("libtraceevent's file", [*rt, "libraries", TE, "sha256"], "0" * 64, f"{TE}:"), ("libtracefs's version", [*rt, "libraries", TF, "version"], "1.8.3-1", f"{TF}:"),
                                    ("libtracefs resolved elsewhere", [*rt, "libraries", TF, "path"], "/usr/local/lib/libtracefs.so.1.8.2", f"{TF}:"),
                                    ("an unreviewed library", [*rt, "libraries", "libextra.so.1"], dict(path="/x", sha256="2" * 64), "unreviewed library"),
                                    ("a plugin's file", [*rt, "plugins", HR, "sha256"], "3" * 64, "plugin_hrtimer.so:"),
                                    ("a plugin added", [*rt, "plugins", EXTRA["path"]], EXTRA, "unreviewed plugin"),
                                    ("a plugin directory from the environment", [*rt, "plugin_dirs"], [R._TE_PLUGINS, "/opt/plugins"], "plugin directories"),
                                    ("HOME in the launch", [*rt, "inputs", "HOME"], "/root", "search inputs"), ("an LD_ variable in the launch", [*rt, "inputs", "LD"], ["LD_PRELOAD"], "search inputs"),
                                    ("a preload", [*rt, "preload"], "/usr/lib/libsomething.so", "a preload"), ("a runtime not a mapping", rt, "none", "wrong type"),
                                    ("a runtime field not understood", [*rt, "extra"], 1, "other fields"), ("an identity field not understood", ["trace_cmd", use, "libraries"], {}, "does not know")):
        r = run(at(GOOD, keys, value))
        claim(f"unsupported: the trace-cmd {desc}, executable unchanged: {name}", r["accepted"] is False and r["cert"] == "UNSUPPORTED" and any(word in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:80])
    for name, keys, word in (("libtracefs absent", [*rt, "libraries", TF], f"{TF}: absent"), ("a plugin absent", [*rt, "plugins", HR], "not loaded")):
        r = run(at(GOOD, keys, delete=True))
        claim(f"unsupported: the trace-cmd {desc}, executable unchanged: {name}", r["accepted"] is False and any(word in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:80])
c = R.certify(put(GOOD, "state.after.tid", 5))
claim("unsupported: the last state read is not this process's", c["status"] == "UNSUPPORTED" and any("after state read is not" in x for x in c["reasons"]), "; ".join(c["reasons"])[:90])

print("end to end: the dependency interval and its fences:")
S0, S1, ARM = GOOD["state"]["before"]["t_start"], GOOD["state"]["after"]["t_end"], GOOD["recorder"]["armed_t"]
def chg(t, name="ntp_adjtimex", modes=8193, tkid=0):
    return dict(t=t, name=name, tkid=tkid, modes=modes, tid=99)
for name, t in (("just after arming", ARM + 1), ("between arming and the first state read", ARM + 10**8), ("between the first state read and the anchors", S0 + 10**8), ("inside a supporting sampler interval before the first anchor", samp[0][1] + 1), ("immediately before the first anchor", E[0] - 1), ("mid-run", E[2000]), ("immediately after the last anchor", E[-1] + 1), ("inside a supporting sampler interval after the last anchor", samp[-1][1] - 1), ("between the last anchor and the last state read", S1 - 10**8), ("during the last state read", S1 - 1)):
    r = run(with_calls(GOOD, [chg(t)]))
    claim(f"a change {name}", r["accepted"] is False and r["cert"] == "UNSUPPORTED" and any("timekeeping changes" in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:70])
r = run(with_calls(GOOD, [chg(E[1000]), chg(E[1000] + 10**7)]))
claim("a change and its restoration, so the end states agree", r["accepted"] is False and any("2 timekeeping changes" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:70])
r = run(with_calls(GOOD, [chg(E[1000], name="ntp_clear", modes=0)]))
claim("an NTP clear inside the interval", r["accepted"] is False and any("ntp_clear" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:70])
r = run(with_calls(GOOD, [chg(E[1000], modes=0), chg(E[1100], modes=0xa001), chg(E[1200], tkid=1)]))
claim("read-only calls and an auxiliary clock's change do not unsupport", r["accepted"] is True, str(r["cert_reasons"]))
r = run(with_calls(GOOD, [chg(S1 + 10**9)]))
claim("a change after the last state read is outside the interval", r["accepted"] is True, str(r["cert_reasons"]))
ev2 = evidence_for(SLO + 5 * 10**7, SHI, pre=10**6)
r = run(ev2)
claim("a sampler bracket used by the bound before the first state read", r["accepted"] is False and any("sampler brackets" in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:90])
ev2 = evidence_for(E[0] + 10**8, SHI, armed_before=2 * 10**9, pre=10**6)
r = run(ev2)
claim("the conversion span starting before the first state read", r["accepted"] is False and any("not inside" in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:90])
ev2 = evidence_for(SLO, SHI, post=10**6)
ev2 = put(ev2, "state.after.t_start", E[-1] - 10**6)
ev2 = put(ev2, "state.after.t_end", E[-1] - 5 * 10**5)
ev2 = put(ev2, "recorder.calls", [ev2["recorder"]["calls"][0], dict(t=E[-1] - 7 * 10**5, name="ntp_adjtimex", tkid=0, modes=0, tid=777)])
r = run(ev2)
claim("the conversion span ending after the last state read", r["accepted"] is False and any("not inside" in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:90])

print("end to end: the arming fence (a change before arming is serialised against the first state read by the timekeeping lock):")
ev2 = copy.deepcopy(GOOD)
ev2["recorder"]["updates"] = [u for u in ev2["recorder"]["updates"] if u["entry"] > S0]
n = len(ev2["recorder"]["updates"])
ev2["recorder"]["probes"]["update_entry"]["hits"] = ev2["recorder"]["probes"]["update_return"]["hits"] = n
r = run(ev2)
claim("no update completed between arming and the first read: a straddling change may not be absorbed", r["accepted"] is False and any("accumulating" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:90])
ev2 = copy.deepcopy(GOOD)
ev2["recorder"]["updates"] = [u for u in ev2["recorder"]["updates"] if u["entry"] > S0 or u["entry"] < ARM + 1_500_000]
n = len(ev2["recorder"]["updates"])
ev2["recorder"]["probes"]["update_entry"]["hits"] = ev2["recorder"]["probes"]["update_return"]["hits"] = n
r = run(ev2)
claim("updates too close together to prove one accumulated: unsupported", r["accepted"] is False and any("accumulating" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:90])
ev2 = copy.deepcopy(GOOD)
ev2["recorder"]["unmatched"]["returns"] = [ARM + 100]
ev2["recorder"]["probes"]["update_return"]["hits"] += 1
r = run(ev2)
claim("a return with no entry at arming (an update in flight) is accepted before the settling updates", r["accepted"] is True, str(r["cert_reasons"]))
ev2 = copy.deepcopy(GOOD)
ev2["recorder"]["unmatched"]["entries"] = [E[2000]]
ev2["recorder"]["probes"]["update_entry"]["hits"] += 1
r = run(ev2)
claim("an entry with no return inside the interval", r["accepted"] is False and any("unmatched" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:70])

print("end to end: the per-acquisition update-duration guard (W):")


def slow(ev, entry, age):
    ev = copy.deepcopy(ev)
    u = min(ev["recorder"]["updates"], key=lambda x: abs(x["entry"] - entry))
    u["ret"] = u["entry"] + age
    return ev


r = run(slow(GOOD, E[2000], R.W_OP_NS))
claim("an update of exactly the maximum is accepted, and the observation is returned", r["accepted"] is True and r["provenance"]["observed_max_update_ns"] == R.W_OP_NS, str(r["provenance"]["observed_max_update_ns"]))
r = run(slow(GOOD, E[2000], R.W_OP_NS + 1))
claim("one update over the maximum: unsupported, fails closed", r["accepted"] is False and any("over the" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:70])
r = run(slow(GOOD, S0 - 3 * 10**6, R.W_OP_NS + 5 * 10**6))
claim("an update straddling the start of the interval is covered", r["accepted"] is False and any("over the" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:70])
r = run(slow(GOOD, ARM + 5 * 10**7, 50 * 10**6))
claim("a long update well before the interval does not count", r["accepted"] is True, str(r["cert_reasons"]))
ev2 = copy.deepcopy(GOOD)
ev2["recorder"]["updates"] = [u for u in ev2["recorder"]["updates"] if u["entry"] < S0 - 10**8]
n = len(ev2["recorder"]["updates"])
ev2["recorder"]["probes"]["update_entry"]["hits"] = ev2["recorder"]["probes"]["update_return"]["hits"] = n
r = run(ev2)
claim("no update recorded inside the interval: unsupported", r["accepted"] is False and any("no update is recorded" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:70])

print("end to end: unmatched update records are not symmetric:")
FIRST = min(u["entry"] for u in GOOD["recorder"]["updates"])


def unmatched(kind, t):
    ev = copy.deepcopy(GOOD)
    ev["recorder"]["unmatched"]["entries" if kind == "entry" else "returns"].append(t)
    ev["recorder"]["probes"]["update_" + kind]["hits"] += 1
    return ev


assert ARM < ARM + 100_000 < FIRST
for name, t in (("just after arming, before the first complete update (the review's case)", ARM + 100_000), ("exactly at arming", ARM), ("just before arming", ARM - 1), ("just before the first complete entry", FIRST - 1), ("at the first complete entry", FIRST), ("mid-interval", E[2000]), ("exactly at the end of the interval", S1)):
    r = run(unmatched("entry", t))
    claim(f"an entry with no return: {name}", r["accepted"] is False and r["cert"] == "UNSUPPORTED" and any("no recorded return" in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:60])
r = run(unmatched("entry", S1 + 1))
claim("an entry with no return after the interval (the record's end) is accepted", r["accepted"] is True, str(r["cert_reasons"]))
for name, t in (("well before the first complete update", ARM + 100), ("just before the first complete entry", FIRST - 1)):
    r = run(unmatched("return", t))
    claim(f"a return with no entry {name} (an update already running at arming) is accepted", r["accepted"] is True, str(r["cert_reasons"]))
for name, t in (("at the first complete entry", FIRST), ("mid-interval", E[2000]), ("exactly at the end of the interval", S1)):
    r = run(unmatched("return", t))
    claim(f"a return with no entry {name}", r["accepted"] is False and any("unmatched update return" in x for x in r["cert_reasons"]), "; ".join(r["cert_reasons"])[:60])
r = run(unmatched("return", S1 + 1))
claim("a return with no entry after the interval (the record's end) is accepted", r["accepted"] is True, str(r["cert_reasons"]))

print("end to end: the initial state the multiplier margin rests on:")


def both(ev, field, value):
    return put(put(ev, f"state.before.{field}", value), f"state.after.{field}", value)


for tick in (9000, 9999, 10001, 11000):
    r = run(both(GOOD, "tick", tick))
    claim(f"a legal, unchanging, non-nominal tick of {tick} us", r["accepted"] is False and r["cert"] == "UNSUPPORTED" and any("not the audited" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:60])
for name, line in (("ntp_tick_adj=1000", "ro quiet ntp_tick_adj=1000"), ("an explicit zero", "ntp_tick_adj=0 ro"), ("the dashed spelling the kernel also accepts", "ro ntp-tick-adj=1000")):
    r = run(put(GOOD, "kernel.cmdline", line))
    claim(f"a boot override: {name}", r["accepted"] is False and r["cert"] == "UNSUPPORTED" and any("ntp_tick_adj" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:60])
r = run(put(GOOD, "kernel.cmdline", "ro quiet xntp_tick_adj=1"))
claim("a different parameter that merely contains the name is not an override", r["accepted"] is True, str(r["cert_reasons"]))
r = run()
claim("the baseline is returned in the provenance", r["provenance"]["tick_us"] == 10000 and "not on" in r["provenance"]["ntp_tick_adj"] and r["provenance"]["deviation_ppm"] < 30, f"{r['provenance']['tick_us']} us, {r['provenance']['deviation_ppm']:.1f} ppm")
r = run(both(GOOD, "freq", 32_768_000))
claim("the kernel's largest frequency is inside the margin", r["accepted"] is True, str(r["cert_reasons"]))
ev2 = evidence_for(SLO, SHI, o0=-500_000_000)
r = run(ev2)
claim("a frequency deviation past the margin", r["accepted"] is False and any("multiplier margin" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:80])
ev2 = evidence_for(SLO, SHI, o0=-100_000_000)
r = run(ev2)
claim("a large but covered deviation stays certified, and widens ds", r["accepted"] is True and r["model"]["ds"] > 1e-6, f"ds {r['model']['ds'] * 1e6:.2f} ppm, deviation {r['provenance']['deviation_ppm']:.0f} ppm")

print("end to end: the boot command line, read as the kernel reads it:")
KERNEL_TOKENS = (  # lib/cmdline.c next_arg, compiled from the audited source and run on these lines
    ('ntp_tick_adj=1000', [('ntp_tick_adj', '1000')]),
    ('"ntp_tick_adj=1000"', [('ntp_tick_adj', '1000')]),
    ('"ntp-tick-adj=1000"', [('ntp-tick-adj', '1000')]),
    ('ntp_tick_adj="1000"', [('ntp_tick_adj', '1000')]),
    ('"ntp_tick_adj="1000""', [('ntp_tick_adj', '1000"')]),
    ('ro  quiet\tntp_tick_adj=1', [('ro', None), ('quiet', None), ('ntp_tick_adj', '1')]),
    ('a="b c" d', [('a', 'b c'), ('d', None)]),
    ('"a b"=c e', [('a b"', 'c'), ('e', None)]),
    ('"', [('', None)]),
    ('""', [('', None)]),
    ('="x"', [('="x"', None)]),
    ('a="', [('a', '')]),
    ('"a=', [('a', '')]),
    ('a=b"', [('a', 'b"')]),
    ('-- ntp_tick_adj=1', [('--', None), ('ntp_tick_adj', '1')]),
    ('  ntp-tick_adj=7  ', [('ntp-tick_adj', '7')]),
    ('ntp_tick_adj', [('ntp_tick_adj', None)]),
    ('"ntp_tick_adj"', [('ntp_tick_adj', None)]),
)
for line, tokens in KERNEL_TOKENS:
    got = R.boot_params(line)
    claim(f"tokenized as the kernel does: {line!r}", got == tokens, str(got)[:70])
for name, line in (("fully quoted", 'ro quiet "ntp_tick_adj=1000"'), ("fully quoted, dashed", 'ro quiet "ntp-tick-adj=1000"'), ("a quoted value", 'ro quiet ntp_tick_adj="1000"'), ("quoted twice over", 'ro "ntp_tick_adj="1000"" x'), ("after a tab", 'ro\tntp_tick_adj=1'), ("after the end-of-options mark", 'ro -- ntp_tick_adj=1'), ("mixed spelling", 'ro ntp-tick_adj=1'), ("first on the line", 'ntp_tick_adj=1 ro'), ("named with no value", 'ro ntp_tick_adj')):
    r = run(put(GOOD, "kernel.cmdline", line))
    claim(f"a boot override, {name}", r["accepted"] is False and r["cert"] == "UNSUPPORTED" and any("ntp_tick_adj" in x for x in r["cert_reasons"]), r["cert_reasons"][0][:60])
for name, line in (("no such parameter", "ro quiet splash"), ("a longer name", "ro ntp_tick_adjX=1"), ("a prefixed name", "ro xntp_tick_adj=1"), ("the name inside a value", "ro foo=ntp_tick_adj=1"), ("the name quoted inside a value", 'ro foo="ntp_tick_adj=1"')):
    r = run(put(GOOD, "kernel.cmdline", line))
    claim(f"not an override: {name}", r["accepted"] is True, str(r["cert_reasons"]))

print("end to end: every converted value within the certified U, on the realizable class:")
bad, wr = 0, []
for seed in range(20):
    ev_, tau_, samp_, M_, dls, ds_, mode = realizable(seed)
    lo_, hi_ = min(a for a, _, _, _ in samp_), max(b for _, _, b, _ in samp_)
    rng_tc = None
    c0 = evidence_for(lo_, hi_, tc=7, o0=0)
    r = R.analyze((ev_, [], {}), (tau_, []), samp_, evidence=c0, model=R.Model(max(ds_, R.certify(c0)["ds"])))
    if not (r["status"] == "PASS" and r["accepted"]):
        bad += 1
        print(f"  seed {seed}: {r['status']} {r['why'][:2]} {r['cert_reasons'][:2]}")
        continue
    for q in dls + tau_:
        c = r["conv"].at(q)
        if c is None:
            continue
        err = abs(c[0] - M_(q))
        wr.append(err / c[1])
        if err > r["U"] or err > c[1]:
            bad += 1
            print(f"  seed {seed}: error {err:.1f} over bound {c[1]:.1f}")
            break
claim("20 realizable captures through analyze(), stale reads up to the maximum", bad == 0 and len(wr) > 1000, f"{len(wr)} queries; largest error/bound {max(wr):.2f}")
print("entries no deadline delimits (revision 54; attempts 13 and 23):")
EXP = 10  # an hrtimer expiry 10 ns after its entry
ORDER = {"W": 0, "E": 1, "H": 2, "X": 3}


def served(ev, skip=()):
    """The ledger with an hrtimer expiry inside every entry except those at the times in skip."""
    return sorted(list(ev) + [(t + EXP, "H", "0x1 tick_nohz_handler/0x0") for t, k, _ in ev if k == "E" and t not in skip],
                  key=lambda r: (r[0], ORDER[r[1]]))


def und(r):
    return r.get("undelimited") or []


E, D, W, X, tau, samp = gen()
xe = E[K] + 18_500  # attempt 23: 18.5 us after the previous entry
FRESH = (E[K] + EXIT + 60, "W", D[K + 1] - 1)  # the next deadline (one count lower, as attempt 23's), not due at the extra entry
BASE_EV = served(events(E, D, W, X))
r0 = check("baseline with expiries recorded", "PASS", BASE_EV, tau, samp)


def extra(fresh=True, inst=True, shift=0, expire=False, more=(), more_tau=()):
    add = [(xe, "E", None), (xe + EXIT, "X", None)] + ([FRESH] if fresh else []) + list(more)
    ev = served(events(E, D, W, X, extra=add), skip=() if expire else (xe,) + tuple(t for t, k, _ in more if k == "E"))
    return ev, sorted(tau + ([xe + DELAY + shift] if inst else []) + list(more_tau))


ev, t2 = extra()
r = check("attempt 23's shape: extra entry, nothing expired, a fresh deadline not due", "PASS", ev, t2, samp)
claim("  ... identified by order, within range, left out of the conversion", len(und(r)) == 1 and und(r)[0]["ok"] and r["used"] == r0["used"]
      and r["U"] == r0["U"] and r["paired"] == r0["paired"], f"undelimited {[(x['j'], round(x['e']), x['ok']) for x in und(r)]}, used {r['used']}/{r0['used']}, U {r['U']:.1f}/{r0['U']:.1f}")
ev, t2 = extra(fresh=False)
r = check("an extra entry, nothing expired, no fresh deadline written", "PASS", ev, t2, samp)
claim("  ... identified by order, left out of the conversion", len(und(r)) == 1 and r["used"] == r0["used"] and r["U"] == r0["U"], f"used {r['used']}, U {r['U']:.1f}")
F13 = (E[K] + EXIT + 60, "W", D[K + 1] + 10)  # attempt 13: the next deadline written early, then rewritten 10 counts lower


def extra13(expire=False):
    add = [(xe, "E", None), (xe + EXIT, "X", None), F13]
    return served(events(E, D, W, X, extra=add), skip=() if expire else (xe,)), sorted(tau + [xe + DELAY])


ev, t2 = extra13()
check("attempt 13's shape with no expiries recorded: fails, as frozen", "FAIL", [x for x in ev if x[1] != "H"], t2, samp, "ladder decreases")
r = check("attempt 13's shape: the early deadline rewritten 10 counts lower; the ladder is the delimiting slots'", "PASS", ev, t2, samp)
claim("  ... identified by order, left out of the conversion", len(und(r)) == 1 and r["used"] == r0["used"] and r["U"] == r0["U"], f"used {r['used']}, U {r['U']:.1f}")
ev, t2 = extra13(expire=True)
check("attempt 13's shape, but a timer expired in the extra entry: the ladder failure stands", "FAIL", ev, t2, samp, "ladder decreases")
Dl = list(D)
Dl[K + 1] = D[K] - 10
check("a ladder decrease between delimiting slots still fails", "FAIL", served(events(E, Dl, W, X)), tau, samp, "ladder decreases")
ev, t2 = extra()
plain = [x for x in ev if x[1] != "H"]
check("attempt 23's shape with no expiries recorded: fails, as frozen", "FAIL", plain, t2, samp, "two instance anchors")
ev, t2 = extra(fresh=False)
check("no fresh deadline, with no expiries recorded: fails, as frozen", "FAIL", [x for x in ev if x[1] != "H"], t2, samp, "no deadline written")
ev, t2 = extra(expire=True)
check("an extra entry in which a timer expired", "FAIL", ev, t2, samp, "no instance anchor")
ev, t2 = extra(inst=False)
check("an extra main entry with no instance anchor", "FAIL", ev, t2, samp)
check("an extra instance anchor with no main entry", "FAIL", BASE_EV, sorted(tau + [xe + DELAY]), samp, "two instance anchors")
for sh in (1000, -1000, 200, -200):
    ev, t2 = extra(shift=sh)
    check(f"the extra entry's anchor {sh:+d} ns from its stamp", "FAIL", ev, t2, samp, "disagree")
x2 = xe + 5_000
ev, t2 = extra(more=[(x2, "E", None), (x2 + EXIT, "X", None)], more_tau=[x2 + DELAY])
check("two extra entries in a row", "FAIL", ev, t2, samp)
ev, t2 = extra()
ev = sorted([x for x in ev if not (x[1] == "H" and x[0] == E[K] + EXP)], key=lambda r: (r[0], ORDER[r[1]]))
check("the previous neighbour served nothing (deadline-raised, as 112 of attempt 13's)", "PASS", ev, t2, samp)
xl = E[-1] + 18_500
ev = served(events(E, D, W, X, extra=[(xl, "E", None), (xl + EXIT, "X", None)]), skip=(xl,))
check("an extra entry after the last one", "FAIL", ev, sorted(tau + [xl + DELAY]), samp)
xf = E[0] - 18_500
ev = served(events(E, D, W, X, extra=[(xf, "E", None), (xf + EXIT, "X", None)]), skip=(xf,))
check("an extra entry before the first one", "FAIL", ev, sorted(tau + [xf + DELAY]), samp)
Es = sorted(without(E, K) + [E[K] - 30])
ev = served(events(Es, without(D, K), without(W, K), sorted(without(X, K) + [E[K] - 30 + EXIT])), skip=(E[K] - 30,))
r = check("LIMIT: a lost entry and its write replaced by an undelimited stamp within range", "PASS", ev, tau, samp)
claim("  ... the substitute is not used in the conversion", len(und(r)) == 1 and r["used"] == r0["used"] - 1, f"used {r['used']}")

print("the identified pair's own bound, independent of other pairs' brackets (revision 54's review):")
MIXED = [(c - 1200, c, c + 1200, z) if E[400] < c < E[1200] else (a, c, b, z) for a, c, b, z in samp]
ev, _ = extra()


def at_e(e, sampler):
    """The rule on attempt 23's shape with the extra entry's anchor placed for a main-minus-anchor e (identity map)."""
    return R.rule(ev, sorted(tau + [round(xe - e)]), sampler, M2)


r = check("mixed brackets elsewhere, the extra entry intact", "PASS", ev, sorted(tau + [xe + DELAY]), MIXED)
claim("  ... its own bound met, and within the (wide) range", len(und(r)) == 1 and und(r)[0]["own"] and und(r)[0]["in_range"],
      f"e {und(r)[0]['e']:.0f}, B {und(r)[0]['B']:.1f}, range [{und(r)[0]['lo']:.0f}, {und(r)[0]['hi']:.0f}]")
check("mixed brackets elsewhere: main stamp 240 ns after the anchor (the review's case)", "FAIL", ev, sorted(tau + [xe - 240]), MIXED, "beyond its own bound")
B0 = und(at_e(0, MIXED))[0]["B"]
r1 = at_e(math.floor(B0), MIXED)
x1 = und(r1)[0]
claim("boundary: e at the pair's own bound passes (mixed brackets)", r1["status"] == "PASS" and x1["e"] <= x1["B"], f"e {x1['e']:.1f}, B {x1['B']:.2f}, {r1['status']}")
r2 = at_e(math.floor(x1["B"]) + 1, MIXED)
x2 = und(r2)[0]
claim("boundary: e 1 ns beyond the pair's own bound fails, the wide range notwithstanding",
      r2["status"] == "FAIL" and x2["e"] > x2["B"] and x2["in_range"] and any("own bound" in w for w in r2["why"]),
      f"e {x2['e']:.1f}, B {x2['B']:.2f}, in range {x2['in_range']}, {r2['status']}")
r3 = at_e(40, samp)
x3 = und(r3)[0]
claim("the range check still applies when the own bound holds (tight brackets)",
      r3["status"] == "FAIL" and x3["own"] and not x3["in_range"] and any("retained pairs' range" in w for w in r3["why"]),
      f"e {x3['e']:.1f}, B {x3['B']:.1f}, range [{x3['lo']:.0f}, {x3['hi']:.0f}]")
WIN_X = (E[100], E[3900])
EV_X = evidence_for(*WIN_X)
r = R.analyze((ev, []), (sorted(tau + [xe + DELAY]), []), samp, window=WIN_X, evidence=EV_X)
claim("certified analyze(): the identified entry, intact: accepted, validated, the entry reported",
      r["status"] == "PASS" and r["accepted"] is True and r["U_label"] == "validated" and len(und(r)) == 1 and und(r)[0]["ok"],
      f"{r['status']}, {r['U_label']}, U {r['U']:.1f}")
r = R.analyze((ev, []), (sorted(tau + [xe - 240]), []), MIXED, window=WIN_X, evidence=EV_X)
claim("certified analyze(), mixed brackets, main stamp 240 ns after the anchor: not accepted, a candidate",
      r["status"] == "FAIL" and r["accepted"] is False and r["cert"] == "CERTIFIED" and r["U_label"] == "candidate U; conversion not accepted"
      and any("own bound" in w for w in r["why"]), f"{r['status']}, {r['U_label']}, e {und(r)[0]['e']:.0f}, B {und(r)[0]['B']:.1f}")
r = R.analyze((ev, []), (sorted(tau + [xe + DELAY]), []), MIXED, window=WIN_X, evidence=EV_X)
claim("certified analyze(), mixed brackets, the extra entry intact: accepted", r["status"] == "PASS" and r["accepted"] is True,
      f"{r['status']}, {r['U_label']}, U {r['U']:.1f}")

print("the review of revision 10's substitutions, now with expiries recorded and nothing expired in the substitute:")
E, D, W, X, tau, samp = gen(cluster=CL)
E1, X1 = without(E, K), without(X, K)
check("missing entry in the cluster, no duplicate", "FAIL", served(events(E1, D, W, X1)), tau, samp, "two instance anchors")
dup = E[K - 1] + 300
E2, X2 = sorted(E1 + [dup]), sorted(X1 + [dup + EXIT])
check("missing entry and write, duplicate at +300 ns, no deadline", "FAIL", served(events(E2, without(D, K), without(W, K), X2), skip=(dup,)), tau, samp)
check("duplicate at +300 ns inside the previous interrupt", "FAIL", served(events(E2, D, [(w if k != K else E[K - 1] + 100) for k, w in enumerate(W)], X2), skip=(dup,)), tau, samp, "inside the previous")
Eb, Db, Wb, Xb, tb, sb = gen(cluster={K: lambda E: E[K - 1] + 20_000})
E4, X4 = sorted(without(Eb, K) + [Eb[K - 1] + 400]), sorted(without(Xb, K) + [Eb[K - 1] + 720])
check("20 us variant: duplicate stamped before the armed deadline", "FAIL", served(events(E4, Db, [(w if k != K else Eb[K - 1] + 360) for k, w in enumerate(Wb)], X4), skip=(Eb[K - 1] + 400,)), tb, sb, "before their deadline")
E, D, W, X, tau, samp = gen()
for sep in (1, 3, 10, 100, 1000):
    xs = E[K + sep] + 1000
    check(f"compensating: missing entry and extra entry {sep} apart", "FAIL", served(events(sorted(without(E, K) + [xs]), D, W, sorted(without(X, K) + [xs + 200])), skip=(xs,)), tau, samp)
for sc_ in (500, 1000, 5000, 20000):
    Ec, Dc, Wc, Xc, tc, sc = gen(cluster={K: lambda E, s=sc_: E[K - 1] + s, K + 1: lambda E, s=sc_: E[K - 1] + 2 * s})
    d = Ec[K - 1] + 300
    check(f"cluster {sc_} ns: missing entry and write, duplicate at +300 ns", "FAIL", served(events(sorted(without(Ec, K) + [d]), without(Dc, K), without(Wc, K), sorted(without(Xc, K) + [d + EXIT])), skip=(d,)), tc, sc)
print("the ledger takes hrtimer expiries strictly:")
lg = led([(100, "local_timer_entry", "vector=236"), (150, "hrtimer_expire_entry", "hrtimer=0xffff8c577efa2098 now=358688588000915 function=tick_nohz_handler/0x0"), (300, "local_timer_exit", "vector=236")])
claim("a well-formed expiry is taken", [k for _, k, _ in lg.events] == ["E", "H", "X"] and not lg.problems)
claim("a malformed expiry is reported", any("malformed hrtimer expiry" in x for x in led([(150, "hrtimer_expire_entry", "hrtimer=0x1 function=f")]).problems))
print("revision 55: an entry stamped before its own delimiter (attempt 28's invocation 3):")
E, D, W, X, tau, samp = gen()
r0 = check("baseline with expiries recorded (again)", "PASS", served(events(E, D, W, X)), tau, samp)
xe = E[K] + 40_000                  # the extra interrupt, 40 us after the tick's (attempt 28: +39.951 us)
EARLY = (E[K] + EXIT + 60, "W", D[K + 1] - 1)  # the next tick deadline written in entry K: the extra entry's delimiter, not due
pe = E[K] + 220_000                 # the probe's own timer interrupt, 220 us after the tick's
P = (xe + 7_600, "W", pe - LAT)     # its sleep, armed after the extra entry, earlier than the extra entry's delimiter
IDLE, NEED, PROBE = ("<idle>", 0, "d.h1."), ("<idle>", 0, "dNh1."), ("x7_bus_timing", 3134348, "d.h..")


def shape28(anchor_x=True, anchor_p=True, more=(), more_tau=(), shift=0, extra_writes=(), Dv=None):
    """Attempt 28's sequence on the synthetic ledger: entry K (the tick), the extra entry (nothing expired), the
    probe's timer (expired), then entry K+1. Returns the ledger, the anchors, the main contexts and the anchors'."""
    Dd = D if Dv is None else Dv
    add = [EARLY, (xe, "E", None), (xe + EXIT, "X", None), P, (pe, "E", None), (pe + EXIT, "X", None)] + list(extra_writes) + list(more)
    ev = served(events(E, Dd, W, X, extra=add), skip=(xe,) + tuple(x for x, k, _ in more if k == "E"))
    t2 = sorted(tau + ([xe + DELAY + shift] if anchor_x else []) + ([pe + DELAY] if anchor_p else []) + list(more_tau))
    mctx = {x: (NEED if x == xe else PROBE if x == E[K] else IDLE) for x, k, _ in ev if k == "E"}
    by_tsc = {x + DELAY: c for x, c in mctx.items()}
    if anchor_x and shift:
        by_tsc[xe + DELAY + shift] = NEED
    ictx = [by_tsc.get(a) for a in t2]
    return ev, t2, mctx, ictx


def rep28(name, want, ev, t2, mctx, ictx, must=None, sampler=None):
    return check(name, want, ev, t2, samp if sampler is None else sampler, must, main_ctx=mctx, inst_ctx=ictx)


ev, t2, mc, ic = shape28()
check("attempt 28's shape without contexts: the frozen failure stands, the repair refused", "FAIL", ev, t2, samp, "contexts")
r = check("  ... and its frozen counts kept", "FAIL", ev, t2, samp, "ladder decreases")
claim("  ... nothing repaired", r["repaired"] == [] and r["repair_refused"], str(r["repair_refused"])[:80])
r = rep28("attempt 28's shape, contexts agreeing: identified once its delimiter is set aside", "PASS", ev, t2, mc, ic)
x = und(r)
kept_tsc = {k for k, _ in r["pairs"]}
claim("  ... one entry, repaired, own bound met, within range, out of the conversion",
      len(x) == 1 and x[0]["repaired"] and x[0]["own"] and x[0]["in_range"] and x[0]["tsc"] not in kept_tsc and r["repaired"] == [x[0]["j"]],
      f"{[(y['j'], round(y['e']), y['own'], y['in_range']) for y in x]}, repaired {r['repaired']}")
claim("  ... every other entry deadline-paired: the probe's timer a pair, one index offset",
      r["paired"] == r0["paired"] + 1 and len(r["offsets"]) == 1, f"paired {r['paired']}/{r0['paired']}, offsets {r['offsets']}")
bad = list(ic)
bad[t2.index(xe + DELAY)] = IDLE
rep28("contexts differ at the extra entry (flags)", "FAIL", ev, t2, mc, bad, "contexts differ")
bad = list(ic)
bad[t2.index(pe + DELAY)] = ("x7_bus_timing", 3134348, "d.h1.")
rep28("contexts differ at the following neighbour", "FAIL", ev, t2, mc, bad, "contexts differ")
bad = list(ic)
bad[t2.index(E[K] + DELAY)] = None
rep28("an anchor's context missing (the preceding neighbour)", "FAIL", ev, t2, mc, bad, "missing")
mc2 = dict(mc)
del mc2[pe]
rep28("an entry's context missing in the main trace", "FAIL", ev, t2, mc2, ic, "missing")
ev, t2, mc, ic = shape28(anchor_p=False)
rep28("the probe timer's anchor missing", "FAIL", ev, t2, mc, ic, "not identified by order")
ev, t2, mc, ic = shape28(more_tau=[xe + 100_000])
rep28("an extra anchor in the region", "FAIL", ev, t2, mc, ic, "not identified by order")
ev, t2, mc, ic = shape28(anchor_x=False)
rep28("the extra entry's anchor missing", "FAIL", ev, t2, mc, ic, "not identified by order")
x2 = xe + 5_000
ev, t2, mc, ic = shape28(more=[(x2, "E", None), (x2 + EXIT, "X", None)], more_tau=[x2 + DELAY])
rep28("two extra entries in a row, the second with no deadline written", "FAIL", ev, t2, mc, ic, "not identified by order")
ev, t2, mc, ic = shape28(more=[(xe + EXIT + 10, "W", D[K + 1] - 2), (x2, "E", None), (x2 + EXIT, "X", None)], more_tau=[x2 + DELAY])
rep28("two adjacent candidates, each stamped before its own delimiter", "FAIL", ev, t2, mc, ic, "neighbour another candidate")
Dl = list(D)
Dl[K + 50] = D[K + 49] - 10
ev, t2, mc, ic = shape28(Dv=Dl)
rep28("a residual decrease elsewhere: refused, the decrease reported", "FAIL", ev, t2, mc, ic, "ladder decreases")
ev, t2, mc, ic = shape28(extra_writes=[(xe + 7_700, "W", 0)])
rep28("the following neighbour pending (its deadline disarmed after it was written)", "FAIL", ev, t2, mc, ic, "not identified by order")
xn = E[K] + 100  # inside entry K's interrupt (its exit at +320 ns)
evn = served(events(E, D, W, X, extra=[EARLY, (xn, "E", None), (xn + EXIT, "X", None), P, (pe, "E", None), (pe + EXIT, "X", None)]), skip=(xn,))
t2n = sorted(tau + [xn + DELAY, pe + DELAY])
mcn = {x: (NEED if x == xn else IDLE) for x, k, _ in evn if k == "E"}
byn = {x + DELAY: c for x, c in mcn.items()}
r = check("the extra entry nested inside the previous interrupt: no candidate, fails", "FAIL", evn, t2n, samp, "inside the previous",
          main_ctx=mcn, inst_ctx=[byn.get(a) for a in t2n])
claim("  ... nothing repaired", r.get("repaired") == [], str(r.get("repair_refused"))[:80])
for sh in (1000, -1000, 200, -200):
    ev, t2, mc, ic = shape28(shift=sh)
    rep28(f"the extra entry's anchor {sh:+d} ns from its stamp", "FAIL", ev, t2, mc, ic)

print("revision 55: ordinary timer-free entries stay ordinary (the next-step review's counterexample):")
ev = served(events(E, D, W, X), skip=(E[K],))
r = check("an ordinary timer-free entry, deadline-raised, holding its anchor (1, 1, 1)", "PASS", ev, tau, samp)
claim("  ... not a candidate: nothing repaired, no entry identified by order, the pairs unchanged",
      r["repaired"] == [] and not r["repair_refused"] and und(r) == [] and r["paired"] == r0["paired"] and r["used"] == r0["used"] and r["U"] == r0["U"],
      f"repaired {r['repaired']}, und {len(und(r))}, paired {r['paired']}/{r0['paired']}, U {r['U']:.1f}/{r0['U']:.1f}")
mc = {x: IDLE for x, k, _ in ev if k == "E"}
r = check("  ... the same with contexts given: unchanged", "PASS", ev, tau, samp, main_ctx=mc, inst_ctx=[IDLE] * len(tau))
claim("  ... nothing repaired", r["repaired"] == [] and und(r) == [] and r["U"] == r0["U"])
ev = served(events(E, D, W, X), skip=(E[K],))
r = check("an ordinary timer-free entry whose anchor is lost (its delimiter was due): not a candidate, fails", "FAIL", ev, without(tau, K), samp,
          main_ctx={x: IDLE for x, k, _ in ev if k == "E"}, inst_ctx=[IDLE] * (len(tau) - 1))
claim("  ... nothing repaired, nothing refused (no candidate)", r.get("repaired") == [] and not r.get("repair_refused"), str(r.get("repair_refused")))
sl = [dict(A=v, t=v, pending=False, nested=False, closed=True, served=0 if i == 2 else 1) for i, v in enumerate((100, 200, 300, 400, 500))]
an = [110, 210, 310, 410, 510]
fr = R.assign(sl, an)
out = R.repair_undelimited(sl, an, fr, {}, R.SamplerMap([(c - 1, c, c + 1, 0) for c in range(0, 1000, 50)], M2))
claim("the review's own example: the (1, 1, 1) entry is no candidate; the frozen slots come back unchanged",
      out[3] == [] and out[4] == [] and out[0] is sl and out[1] is fr, f"repaired {out[3]}, refused {out[4]}")

print("revision 55: the identified pair's own bound and the range, as revision 54's:")
MIXED = [(c - 1200, c, c + 1200, z) if E[400] < c < E[1200] else (a, c, b, z) for a, c, b, z in samp]
ev, t2, mc, ic = shape28()
r = rep28("mixed brackets elsewhere, attempt 28's shape intact", "PASS", ev, t2, mc, ic, sampler=MIXED)
claim("  ... repaired, own bound met", r["repaired"] and und(r)[0]["own"], f"e {und(r)[0]['e']:.0f}, B {und(r)[0]['B']:.1f}")


def at_e28(e, sampler):
    t2b = sorted(tau + [round(xe - e), pe + DELAY])
    byt = {x + DELAY: c for x, c in mc.items()}
    byt[round(xe - e)] = NEED
    return R.rule(ev, t2b, sampler, M2, main_ctx=mc, inst_ctx=[byt.get(a) for a in t2b])


r = at_e28(240, MIXED)
claim("mixed brackets: the extra entry's main stamp 240 ns after its anchor fails on its own bound",
      r["status"] == "FAIL" and any("own bound" in w for w in r["why"]), f"{r['status']}; {r['why'][:1]}")
B0 = und(at_e28(0, MIXED))[0]["B"]
r1 = at_e28(math.floor(B0), MIXED)
claim("boundary: e at the pair's own bound passes", r1["status"] == "PASS", f"e {und(r1)[0]['e']:.1f}, B {und(r1)[0]['B']:.2f}")
r2 = at_e28(math.floor(und(r1)[0]["B"]) + 1, MIXED)
claim("boundary: e 1 ns beyond its own bound fails, in range or not", r2["status"] == "FAIL" and any("own bound" in w for w in r2["why"]),
      f"e {und(r2)[0]['e']:.1f}, B {und(r2)[0]['B']:.2f}, in range {und(r2)[0]['in_range']}")
r3 = at_e28(40, samp)
claim("the range check still applies when the own bound holds (tight brackets)",
      r3["status"] == "FAIL" and und(r3)[0]["own"] and not und(r3)[0]["in_range"], f"e {und(r3)[0]['e']:.1f}, B {und(r3)[0]['B']:.1f}")
WIN_X = (E[100], E[3900])
EV_X = evidence_for(*WIN_X)
r = R.analyze((ev, [], mc), (t2, [], ic), samp, window=WIN_X, evidence=EV_X)
claim("certified analyze(): attempt 28's shape, contexts agreeing: accepted, the repaired entry reported",
      r["status"] == "PASS" and r["accepted"] is True and r["U_label"] == "validated" and r["repaired"] and und(r)[0]["ok"], f"{r['status']}, {r['U_label']}")
bad = list(ic)
bad[t2.index(xe + DELAY)] = IDLE
r = R.analyze((ev, [], mc), (t2, [], bad), samp, window=WIN_X, evidence=EV_X)
claim("certified analyze(): contexts differing: not accepted, a candidate U", r["status"] == "FAIL" and r["accepted"] is False
      and r["cert"] == "CERTIFIED" and r["U_label"] == "candidate U; conversion not accepted", f"{r['status']}, {r['U_label']}")
r = R.analyze((ev, []), (t2, []), samp, window=WIN_X, evidence=EV_X)
claim("certified analyze(): no contexts given (revision 54's call): not accepted", r["status"] == "FAIL" and r["accepted"] is False, r["status"])

print("revision 55: the contexts as the reports give them:")
rows = "\n".join([
    "# tracer: nop",
    "   Isolated Web Co-2711452 [003] d.h1. 100: local_timer_entry: vector=236",
    "   kworker/u128:2-3133692 [003] dNh1. 200: local_timer_entry: vector=236",
    "   a-1-5 [003] d.h.. 300: local_timer_entry: vector=236",
    "          <idle>-0       [003] d.h1. 400: local_timer_entry: vector=236",
    "   [003] d.h1. 500: local_timer_entry: vector=236",
])
tsc_c, prob_c, ctx_c = R.parse_instance_contexts(rows)
claim("task names with spaces, dashes and digits; the idle task; a row with no task field (its context missing)",
      tsc_c == [100, 200, 300, 400, 500] and not prob_c and ctx_c == [("Isolated Web Co", 2711452, "d.h1."), ("kworker/u128:2", 3133692, "dNh1."),
                                                                   ("a-1", 5, "d.h.."), ("<idle>", 0, "d.h1."), None], str(ctx_c))
claim("parse_instance still returns its two lists, the same", R.parse_instance(rows) == (tsc_c, prob_c))
lg = R.Ledger()
lg.feed(100, "local_timer_entry", "vector=236", ("<idle>", 0, "dNh1."))
lg.feed(150, "write_msr", "6e0, value 2723d67a9bc2", ("<idle>", 0, "dNh1."))
lg.feed(300, "local_timer_exit", "vector=236", ("<idle>", 0, "dNh1."))
claim("the ledger keeps an entry's context, and only an entry's", lg.contexts == {100: ("<idle>", 0, "dNh1.")} and [k for _, k, _ in lg.events] == ["E", "W", "X"])
print(f"\n{COUNT} cases. {'ALL OK' if not FAILS else 'FAILED: ' + '; '.join(FAILS)}")
sys.exit(1 if FAILS else 0)
