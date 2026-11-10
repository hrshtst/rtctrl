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
                state=dict(before=state(s0, o0), after=state(s1, round(o))))


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
print(f"\n{COUNT} cases. {'ALL OK' if not FAILS else 'FAILED: ' + '; '.join(FAILS)}")
sys.exit(1 if FAILS else 0)
