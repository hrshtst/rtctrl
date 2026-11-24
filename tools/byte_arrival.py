#!/usr/bin/env python3
"""The Fast Sync Read reply's byte arrival as the host observes it, from one timer run's exports (offline).

usage: byte_arrival.py LOG --completions CSV --segments CSV [--len 13|10] [--json FILE]

LOG is the probe's per-cycle log: every executed read of the chosen length is in C5's denominator. The completions are
trace_correlate.py --completions-csv (every bulk IN completion of the run: its data bytes, FTDI status bytes excluded,
and the read whose last reply bytes it carried, with whether that read's data completions add up to its reply); the
segments are --segments-csv (each read's OUT completion and data completions, as durations).

Every time is a host stamp: a USB IN completion's trace time less the read's instruction OUT completion. Neither is a
UART event, and the adapter's cutoff and the USB transfer add an unknown offset and jitter.

Observations, per read of the chosen length (len 13: the full window, N = 144 bytes; len 10: motion-only, N = 120):
- a partial prefix d (0 < d < N): bytes 0..d-1 were in the adapter's buffer at that completion's cutoff, byte d not;
- a whole reply (d = N): censored, complete by then, its last byte's time unknown;
- a status-only completion after the OUT completion and before the first data: a prefix of 0 (a status-only completion
  between a read's data completions is not an observation, as in the byte-arrival study);
- the read's first completion after its OUT completion, and whether it carried the whole reply.

The protocol map (include/rtctrl/dxl/wire_budget.hpp, fastSyncReadReplyBytes): a Fast Sync Read reply to 8 servos of
len bytes each is 8 + 8 (len + 4) bytes, the 8-byte header then per servo its error byte, ID, data and a running
2-byte CRC; servo s's section starts at byte 8 + s (len + 4), s = 1..7. That a step happens there is the model's.

Coverage (the full-feedback diagnostic's plan r1 and its addendum), evaluated before any fit, each with its denominator:
- C1: every prefix position 1..N-1 holds at least 20 partial observations;
- C2: at each section start b, each position b-3..b+3 holds at least 20 partial observations, and b itself at least 40;
- C3: each half-open 10 us bin [650, 660) ... [840, 850) of the first completion's time after the OUT completion holds
  at least 30 first-completion reads (reconstructed reads);
- C4: descriptive and non-gating: the first completions' times, their range and largest gap;
- C5: the excluded reads (read failed, no carrier, data completions not adding up, completeness unknown, segments
  missing, completions not reconstructed exactly) are at most 1 % of every executed read of the chosen length.
A requirement that fails makes the results it supports INSUFFICIENT (C5: every result): insufficient evidence, never a
capture-validity failure or a reason to retry.

Results:
- primary, host-observed: per prefix the partial observations' count and time distribution; per transition bin the
  first-completion reads, the whole ones and their share; the latest partial and the earliest whole;
- secondary, conditional on the model: the pooled common-step fit, t = T0' + w d + g kbar(d), kbar(d) = (k(d-1) +
  k(d)) / 2, with rank, the standardized design's condition number, standard errors (independent errors assumed) and
  residuals; the boundaries shifted -2..+2 bytes; w fixed at the wire's 3.333 us; dwell counts (a uniform phase
  assumed); at each section start, a local step fitted on the positions within 5 bytes of it alone (descriptive: not
  P1's evidence); the whole-reply endpoint only when the partial prefixes reach N - 1;
- the predictions, as the addendum defines them:
  - P1 (narrowed to the common-step model): the pooled common step g within [12, 25] us, with the unshifted
    boundaries' residual sum the least of the five shifts; needs C1, C2 and C5;
  - P2: the pooled slope w within [3.0, 3.4] us; needs C1 and C5;
  - P3: the crossing rule: the share of whole first completions per bin, over the bins with reads in order; a crossing
    is a pair of consecutive such bins with the share below 0.5 then at least 0.5 (rising), or at least 0.5 then below
    (falling). Every crossing is reported, none smoothed. As predicted only if there is exactly one crossing, rising,
    both its bins within [700, 780) us; needs C3 and C5;
  - P4: at least one read whose first completion after its OUT completion is later than 750 us and does not carry the
    whole reply (partial data or status only, each counted); needs C5, and C3 in every bin reaching past 750 us;
  - P5 is the timing summary's (T1 to T7), not this tool's.
  Each is AS PREDICTED, NOT AS PREDICTED or INSUFFICIENT; a prediction not as predicted is an observation.

Exit status: 0 with the report written (whatever coverage holds), 2 on unreadable or inconsistent input.
"""

from __future__ import annotations

import argparse
import bisect
import collections
import csv
import json
import math
import pathlib
import sys
from typing import Any

import numpy as np

N_SERVOS = 8
WIRE_US = 10 / 3.0  # 10 bits per byte at 3 Mbps
C1_MIN, C2_NEIGH_MIN, C2_START_MIN, C3_MIN, C5_MAX = 20, 20, 40, 30, 0.01
BINS = tuple(range(650, 850, 10))  # [b, b + 10) us
LOCAL_SPAN = 5
P1_G, P2_W, P3_RANGE, P4_AFTER = (12.0, 25.0), (3.0, 3.4), (700, 780), 750.0


def reply_bytes(length: int) -> int:
    return 8 + N_SERVOS * (length + 4)


def section_starts(length: int) -> list[int]:
    return [8 + s * (length + 4) for s in range(1, N_SERVOS)]


def handoffs(j: int, length: int, shift: int = 0) -> int:
    """The section starts at or before byte j (each shifted by shift bytes, for the shift check)."""
    return sum(1 for b in section_starts(length) if b + shift <= j)


def kbar(d: int, length: int, shift: int = 0) -> float:
    return (handoffs(d - 1, length, shift) + handoffs(d, length, shift)) / 2


def bin_of(t_us: float) -> int | None:
    """The half-open 10 us bin [b, b + 10) holding t, or None outside [650, 850)."""
    b = math.floor(t_us / 10) * 10
    return b if b in BINS else None


class InputError(Exception):
    pass


def read_csv(path: pathlib.Path) -> list[dict]:
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except OSError as e:
        raise InputError(f"{path}: {e}") from e


def reconstruct(log_rows: list[dict], comps: list[dict], segs: dict[int, dict], length: int) -> dict:
    """Every executed read of the chosen length, each observed or excluded with its reason."""
    N = reply_bytes(length)
    try:
        ts = [int(c["ts_ns"]) for c in comps]
        db = [int(c["data_bytes"]) for c in comps]
        reads = [r for r in log_rows if int(r["read_len"]) == length]
    except (KeyError, ValueError) as e:
        raise InputError(f"a completion or log row: {e!r}") from e
    if ts != sorted(ts):
        raise InputError("the completions are not in time order")
    carrier = {}
    for k, c in enumerate(comps):
        if c.get("last_of_cycle"):
            carrier[int(c["last_of_cycle"])] = (k, c.get("reply_complete", ""))
    obs: list[dict] = []
    first: list[dict] = []
    excluded: collections.Counter = collections.Counter()
    for r in reads:
        cyc = int(r["cycle"])
        if r.get("read_ok") != "1":
            excluded["read failed"] += 1
            continue
        got = carrier.get(cyc)
        if got is None:
            excluded["no carrier (unassociated)"] += 1
            continue
        k, complete = got
        if complete == "0":
            excluded["data completions do not add up to the reply"] += 1
            continue
        if complete != "1":
            excluded["reply completeness unknown"] += 1
            continue
        s = segs.get(cyc)
        if s is None or s.get("to_first_completion_ns") in (None, "") or s.get("between_completions_ns") in (None, ""):
            excluded["segments missing"] += 1
            continue
        n = int(s["completions"])
        t_first = ts[k] - int(s["between_completions_ns"])
        t_out = t_first - int(s["to_first_completion_ns"])
        idxs, j = [k], k - 1
        while len(idxs) < n and j >= 0 and ts[j] >= t_first:
            if db[j] > 0:
                idxs.insert(0, j)
            j -= 1
        if len(idxs) != n or ts[idxs[0]] != t_first or sum(db[i] for i in idxs) != N:
            excluded["completions not reconstructed exactly"] += 1
            continue
        cum = 0
        for i in idxs:
            cum += db[i]
            obs.append({"cycle": cyc, "t_us": (ts[i] - t_out) / 1e3, "d": cum, "kind": "whole" if cum == N else "partial"})
        j0 = bisect.bisect_right(ts, t_out)
        zero = [jj for jj in range(j0, idxs[0]) if db[jj] == 0]
        for jj in zero:
            obs.append({"cycle": cyc, "t_us": (ts[jj] - t_out) / 1e3, "d": 0, "kind": "zero"})
        f0 = zero[0] if zero else idxs[0]
        whole = f0 == idxs[0] and n == 1
        first.append({"cycle": cyc, "t_us": (ts[f0] - t_out) / 1e3, "whole": whole,
                      "kind": "zero" if zero else ("whole" if whole else "partial")})
    return {"length": length, "N": N, "reads": len(reads), "observations": obs, "first": first,
            "excluded": dict(excluded)}


def coverage(rec: dict) -> dict:
    """C1 to C5 against their thresholds (C4 descriptive), each with its counts and denominator."""
    N, length = rec["N"], rec["length"]
    counts = collections.Counter(o["d"] for o in rec["observations"] if o["kind"] == "partial")
    c1_short = [d for d in range(1, N) if counts[d] < C1_MIN]
    c2 = {}
    for b in section_starts(length):
        short = [x for x in range(b - 3, b + 4) if counts[x] < C2_NEIGH_MIN]
        c2[b] = {"start": counts[b], "neighbourhood_short": short, "ok": not short and counts[b] >= C2_START_MIN}
    bins: collections.Counter = collections.Counter()
    whole: collections.Counter = collections.Counter()
    for f in rec["first"]:
        b = bin_of(f["t_us"])
        if b is not None:
            bins[b] += 1
            whole[b] += f["whole"]
    c3_short = [b for b in BINS if bins[b] < C3_MIN]
    times = sorted(f["t_us"] for f in rec["first"])
    gaps = [y - x for x, y in zip(times, times[1:])]
    n_excl = sum(rec["excluded"].values())
    share = n_excl / rec["reads"] if rec["reads"] else 1.0
    return {
        "partial_counts": {d: counts[d] for d in range(1, N)},
        "C1": {"ok": not c1_short, "min": C1_MIN, "short": c1_short, "denominator": "partial observations"},
        "C2": {"ok": all(v["ok"] for v in c2.values()), "start_min": C2_START_MIN, "neighbourhood_min": C2_NEIGH_MIN,
               "starts": c2, "denominator": "partial observations"},
        "C3": {"ok": not c3_short, "min": C3_MIN, "bins": {b: bins[b] for b in BINS}, "whole": {b: whole[b] for b in BINS},
               "short": c3_short, "denominator": f"first-completion reads ({len(rec['first'])})"},
        "C4": {"gating": False, "first_reads": len(times), "range_us": [times[0], times[-1]] if times else None,
               "largest_gap_us": max(gaps) if gaps else None},
        "C5": {"ok": rec["reads"] > 0 and share <= C5_MAX, "max_share": C5_MAX, "excluded": n_excl, "reads": rec["reads"],
               "share": share, "denominator": "every executed read of the chosen length"},
    }


def distributions(rec: dict) -> dict:
    by = collections.defaultdict(list)
    for o in rec["observations"]:
        if o["kind"] == "partial":
            by[o["d"]].append(o["t_us"])

    def q(v: list[float], p: float) -> float:
        return v[min(len(v) - 1, int(p * len(v)))]
    out = {}
    for d, v in sorted(by.items()):
        v.sort()
        out[d] = {"n": len(v), "min": v[0], "p5": q(v, .05), "p50": q(v, .5), "p95": q(v, .95), "max": v[-1]}
    return out


def crossings(shares: list[tuple[int, float]]) -> list[dict]:
    """Every 50 % crossing between consecutive bins (those with reads, in order), rising or falling, none smoothed."""
    out = []
    for (b0, s0), (b1, s1) in zip(shares, shares[1:]):
        if s0 < 0.5 <= s1:
            out.append({"from_bin": b0, "to_bin": b1, "direction": "rising"})
        elif s0 >= 0.5 > s1:
            out.append({"from_bin": b0, "to_bin": b1, "direction": "falling"})
    return out


def transition(cov: dict, rec: dict) -> dict:
    seq = [(b, cov["C3"]["whole"][b] / cov["C3"]["bins"][b]) for b in BINS if cov["C3"]["bins"][b]]
    part = [f["t_us"] for f in rec["first"] if not f["whole"]]
    whole = [f["t_us"] for f in rec["first"] if f["whole"]]
    return {"shares": dict(seq), "crossings": crossings(seq), "latest_partial_us": max(part) if part else None,
            "earliest_whole_us": min(whole) if whole else None,
            "partial_after_p4_us": sum(1 for t in part if t > P4_AFTER),
            "after_p4_by_kind": dict(collections.Counter(f.get("kind", "partial") for f in rec["first"]
                                                         if not f["whole"] and f["t_us"] > P4_AFTER))}


def fit(rows: list[tuple[int, float]], cols: list) -> dict[str, Any]:
    """Least squares of t on the given columns of d; refused (rank short) rather than guessed."""
    d = np.array([r[0] for r in rows], float)
    t = np.array([r[1] for r in rows], float)
    X = np.column_stack([c(d) for c in cols]) if rows else np.zeros((0, len(cols)))
    rank = int(np.linalg.matrix_rank(X)) if rows else 0
    out: dict[str, Any] = {"n": len(rows), "rank": rank, "columns": len(cols)}
    if rank < len(cols) or len(rows) <= len(cols):
        out["refused"] = f"rank {rank} of {len(cols)} columns, {len(rows)} observations: no estimate"
        return out
    beta, *_ = np.linalg.lstsq(X, t, rcond=None)
    res = t - X @ beta
    s2 = float(res @ res) / (len(t) - len(cols))
    se = np.sqrt(np.diag(s2 * np.linalg.inv(X.T @ X)))
    sd = X.std(axis=0)
    Xs = np.column_stack([np.ones(len(t))] + [(X[:, i] - X[:, i].mean()) / sd[i] for i in range(len(cols)) if sd[i] > 0])
    out.update(beta=[float(x) for x in beta], se=[float(x) for x in se], rss=float(res @ res), resid_sd=math.sqrt(s2),
               cond=float(np.linalg.cond(Xs)))
    return out


def model(rec: dict) -> dict[str, Any]:
    length, N = rec["length"], rec["N"]
    part = [(o["d"], o["t_us"]) for o in rec["observations"] if o["kind"] == "partial"]
    starts = section_starts(length)

    def one(d):
        return np.ones_like(d)

    def ident(d):
        return d

    def kb(shift: int):
        return lambda d: np.array([kbar(int(x), length, shift) for x in d])
    out: dict[str, Any] = {"pooled": fit(part, [one, ident, kb(0)])}
    out["shifts"] = {sh: fit(part, [one, ident, kb(sh)]).get("rss") for sh in (-2, -1, 0, 1, 2)}
    out["w_fixed"] = fit([(d, t - WIRE_US * d) for d, t in part], [one, kb(0)])
    counts = collections.Counter(d for d, _ in part)
    out["dwell"] = None
    if part:
        lo, hi = min(counts), max(counts)
        inner = [counts[x] for x in range(lo + 1, hi) if x not in starts]
        at = [counts[b] for b in starts if lo < b < hi]
        if inner and at:
            out["dwell"] = {"inner_mean": sum(inner) / len(inner), "start_mean": sum(at) / len(at), "starts": len(at)}
    local = {}
    for b in starts:
        near = [(d, t) for d, t in part if abs(d - b) <= LOCAL_SPAN]
        local[b] = fit(near, [one, ident, lambda d, b=b: np.array([(int(x - 1 >= b) + int(x >= b)) / 2 for x in d])])
    out["local_steps"] = local
    p = out["pooled"]
    if "beta" in p and part and max(counts) >= N - 1:
        T0, w, g = p["beta"]
        out["endpoint_us"] = T0 + w * N + g * (N_SERVOS - 1)
    else:
        out["endpoint_us"] = None
        out["endpoint_note"] = "the partial prefixes do not reach N - 1 or the fit is refused: no endpoint"
    return out


def predictions(cov: dict, tr: dict, m: dict[str, Any] | None) -> dict:
    def need(*keys: str) -> list[str]:
        return [k for k in keys if not cov[k]["ok"]]
    out = {}
    p: dict[str, Any] = m["pooled"] if m else {}
    short = need("C1", "C2", "C5")
    if short:
        out["P1"] = {"outcome": "INSUFFICIENT", "why": f"{', '.join(short)} insufficient"}
    elif m is None or "beta" not in p:
        out["P1"] = {"outcome": "INSUFFICIENT", "why": p.get("refused", "no fit")}
    else:
        g = p["beta"][2]
        shifts = m["shifts"]
        least = min(shifts, key=lambda s: shifts[s])
        within = P1_G[0] <= g <= P1_G[1]
        out["P1"] = {"outcome": "AS PREDICTED" if within and least == 0 else "NOT AS PREDICTED",
                     "why": f"the common step {g:.2f} us (within [{P1_G[0]:g}, {P1_G[1]:g}]: {within}); the least "
                            f"residual sum at shift {least:+d}"}
    short = need("C1", "C5")
    if short or "beta" not in p:
        out["P2"] = {"outcome": "INSUFFICIENT", "why": f"{', '.join(short)} insufficient" if short else p.get("refused")}
    else:
        w = p["beta"][1]
        out["P2"] = {"outcome": "AS PREDICTED" if P2_W[0] <= w <= P2_W[1] else "NOT AS PREDICTED",
                     "why": f"the slope {w:.3f} us per byte (within [{P2_W[0]:g}, {P2_W[1]:g}])"}
    short = need("C3", "C5")
    if short:
        out["P3"] = {"outcome": "INSUFFICIENT", "why": f"{', '.join(short)} insufficient", "crossings": tr["crossings"]}
    else:
        cr = tr["crossings"]
        ok = (len(cr) == 1 and cr[0]["direction"] == "rising" and cr[0]["from_bin"] >= P3_RANGE[0]
              and cr[0]["to_bin"] + 10 <= P3_RANGE[1])
        out["P3"] = {"outcome": "AS PREDICTED" if ok else "NOT AS PREDICTED",
                     "why": f"{len(cr)} crossing(s), every one kept", "crossings": cr}
    late = [b for b in cov["C3"]["short"] if b + 10 > P4_AFTER]
    if not cov["C5"]["ok"] or late:
        out["P4"] = {"outcome": "INSUFFICIENT", "why": "C5 insufficient" if not cov["C5"]["ok"] else f"C3 short at {late}"}
    else:
        n, k = tr["partial_after_p4_us"], tr["after_p4_by_kind"]
        out["P4"] = {"outcome": "AS PREDICTED" if n else "NOT AS PREDICTED",
                     "why": f"{n} first completion(s) later than {P4_AFTER:g} us not carrying the whole reply "
                            f"({k.get('partial', 0)} partial data, {k.get('zero', 0)} status only)"}
    return out


def analyze(log_rows: list[dict], comps: list[dict], segs: dict[int, dict], length: int) -> dict:
    rec = reconstruct(log_rows, comps, segs, length)
    cov = coverage(rec)
    tr = transition(cov, rec)
    m = model(rec) if cov["C5"]["ok"] else None
    return {"length": length, "N": rec["N"], "section_starts": section_starts(length), "reads": rec["reads"],
            "excluded": rec["excluded"], "observations": dict(collections.Counter(o["kind"] for o in rec["observations"])),
            "coverage": cov, "insufficient": [k for k in ("C1", "C2", "C3", "C5") if not cov[k]["ok"]],
            "distributions": distributions(rec), "transition": tr, "model": m, "predictions": predictions(cov, tr, m)}


def suff(ok: bool) -> str:
    return "SUFFICIENT" if ok else "INSUFFICIENT"


def print_report(rep: dict, out=sys.stdout) -> None:
    def say(*a: object) -> None:
        print(*a, file=out)
    cov, N = rep["coverage"], rep["N"]
    say(f"# byte arrival, len {rep['length']} (N {N}), section starts {rep['section_starts']}; host stamps after the "
        "OUT completion, not UART events")
    say(f"executed reads {rep['reads']}; excluded {cov['C5']['excluded']} ({cov['C5']['share']:.4%}): "
        f"{rep['excluded'] or 'none'}; observations: {rep['observations']}")
    sh = cov["C1"]["short"]
    say(f"C1 (every position 1..{N - 1} >= {C1_MIN} partial observations): {suff(cov['C1']['ok'])}; short: "
        f"{sh[:20]}{' ...' if len(sh) > 20 else ''}")
    for b, v in cov["C2"]["starts"].items():
        say(f"C2 at {b}: the start {v['start']} (>= {C2_START_MIN}), neighbours short {v['neighbourhood_short']}: "
            f"{suff(v['ok'])}")
    say(f"C3 (each bin [650, 850) >= {C3_MIN} of {cov['C3']['denominator']}): {suff(cov['C3']['ok'])}; "
        + " ".join(f"{b}:{cov['C3']['bins'][b]}" for b in BINS))
    say(f"C4 (descriptive, non-gating): {cov['C4']}")
    say(f"C5 (excluded <= {C5_MAX:.0%} of every executed read): {suff(cov['C5']['ok'])}")
    say(f"insufficient: {', '.join(rep['insufficient']) or 'none'}")
    tr = rep["transition"]
    say("the share of whole first completions per bin: " + (" ".join(f"{b}:{s:.2f}" for b, s in tr["shares"].items())
                                                           or "no bin with reads"))
    say(f"50 % crossings (every one): {tr['crossings'] or 'none'}; the latest partial {tr['latest_partial_us']}, the "
        f"earliest whole {tr['earliest_whole_us']}")
    say("per prefix (n, p5, p50, p95 us): " + " ".join(f"{d}:{v['n']},{v['p5']:.0f},{v['p50']:.0f},{v['p95']:.0f}"
                                                     for d, v in rep["distributions"].items()))
    m = rep["model"]
    if m is None:
        say("model: not fitted (C5 insufficient)")
    else:
        p = m["pooled"]
        say("model (secondary, conditional), the pooled common step: " + (p.get("refused") or (
            f"T0' {p['beta'][0]:.1f} +- {p['se'][0]:.1f}, w {p['beta'][1]:.3f} +- {p['se'][1]:.3f}, g {p['beta'][2]:.1f} "
            f"+- {p['se'][2]:.1f} us; rank {p['rank']}, condition {p['cond']:.1f}, residual sd {p['resid_sd']:.1f} us, "
            f"n {p['n']}")))
        say(f"  shifted boundaries, residual sums: {m['shifts']}")
        wf = m["w_fixed"]
        say("  w fixed at the wire's 3.333 us: " + (wf.get("refused") or f"g {wf['beta'][1]:.1f} +- {wf['se'][1]:.1f} us"))
        say(f"  dwell counts (a uniform phase assumed): {m['dwell']}")
        for b, f in m["local_steps"].items():
            say(f"  the local step at {b} (descriptive; positions within {LOCAL_SPAN} bytes): " + (f.get("refused") or (
                f"{f['beta'][2]:.1f} +- {f['se'][2]:.1f} us (n {f['n']}, rank {f['rank']})")))
        say(f"  the whole-reply endpoint: {m['endpoint_us'] if m['endpoint_us'] is not None else m['endpoint_note']}")
    for k, v in rep["predictions"].items():
        say(f"{k}: {v['outcome']}: {v['why']}")
    say("P5: the timing summary's (T1 to T7), not this tool's")


def main() -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    ap.add_argument("log", type=pathlib.Path)
    ap.add_argument("--completions", type=pathlib.Path, required=True)
    ap.add_argument("--segments", type=pathlib.Path, required=True)
    ap.add_argument("--len", dest="length", type=int, choices=(10, 13), default=13)
    ap.add_argument("--json", type=pathlib.Path)
    a = ap.parse_args()
    try:
        log_rows = read_csv(a.log)
        comps = read_csv(a.completions)
        segs = {int(r["cycle"]): r for r in read_csv(a.segments)}
        rep = analyze(log_rows, comps, segs, a.length)
    except (InputError, KeyError, ValueError) as e:
        print(f"byte_arrival: {e!r}", file=sys.stderr)
        return 2
    print_report(rep)
    if a.json:
        a.json.write_text(json.dumps(rep, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
