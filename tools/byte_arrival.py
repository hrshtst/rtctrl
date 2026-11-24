#!/usr/bin/env python3
"""The Fast Sync Read reply's byte arrival as the host observes it, from one timer run's exports (offline).

usage: byte_arrival.py LOG --completions CSV --segments CSV [--len 13|10 | --condition C] [--json FILE]
       byte_arrival.py contrast Q4|Q5 A.json B.json

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

The traffic-reduction diagnostic's Stage A (proposal r2 and its review's scoring amendment, 2026-11-24), with
--condition R8-10, R4a-10, R1-10, R4a-13, R4b-10 or R8-10' (one grid-timed timer run of x7_bus_timing --read-ids):
- the run's read set (the summary's read_ids, else its every servo) and every row's window must be the condition's;
- the reconstruction, coverage and model are for its n servos (N = 8 + n (len + 4), starts 8 + s (len + 4) for s =
  1..n-1). C3's bins are the condition's declared 200 us window; C2 is not applicable to one servo, whose model is
  the rank-2 fit t = T0' + w d (no step term by construction);
- Q1 (the crossing rule above in the condition's band, both bins inside it: from_bin at or above its lower limit,
  to_bin + 10 at or below its upper), Q2 (the pooled common step g within [12, 25] us with the least residual at the
  protocol's positions; not applicable to one servo) and Q3 (the pooled slope w within [3.0, 3.4] us). Each required
  coverage item must be explicitly established (True): missing, not applicable or False makes the prediction
  INSUFFICIENT. Q1 needs C3 and C5; Q2 C1, C2 and C5; Q3 C1 and C5;
- `contrast Q4 R4a-10.json R4a-13.json` (R4a-13's single rising crossing later by 25 to 55 us) and `contrast Q5
  R4a-10.json R4b-10.json` (within 15 us), each needing C3 and C5 and exactly one rising crossing in both runs. The
  local fits at each start and the model's differences are reported, never scored.
Without --condition, a run whose summary records a read set of other than eight servos is refused.

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
import tomllib
from typing import Any

import numpy as np

N_SERVOS = 8
WIRE_US = 10 / 3.0  # 10 bits per byte at 3 Mbps
C1_MIN, C2_NEIGH_MIN, C2_START_MIN, C3_MIN, C5_MAX = 20, 20, 40, 30, 0.01
BINS = tuple(range(650, 850, 10))  # [b, b + 10) us
LOCAL_SPAN = 5
P1_G, P2_W, P3_RANGE, P4_AFTER = (12.0, 25.0), (3.0, 3.4), (700, 780), 750.0


def reply_bytes(length: int, n: int = N_SERVOS) -> int:
    return 8 + n * (length + 4)


def section_starts(length: int, n: int = N_SERVOS) -> list[int]:
    return [8 + s * (length + 4) for s in range(1, n)]


def handoffs(j: int, length: int, shift: int = 0, n: int = N_SERVOS) -> int:
    """The section starts at or before byte j (each shifted by shift bytes, for the shift check)."""
    return sum(1 for b in section_starts(length, n) if b + shift <= j)


def kbar(d: int, length: int, shift: int = 0, n: int = N_SERVOS) -> float:
    return (handoffs(d - 1, length, shift, n) + handoffs(d, length, shift, n)) / 2


def bin_of(t_us: float, bins: tuple[int, ...] = BINS) -> int | None:
    """The half-open 10 us bin [b, b + 10) holding t, or None outside the bins (by default [650, 850))."""
    b = math.floor(t_us / 10) * 10
    return b if b in bins else None


class InputError(Exception):
    pass


def read_csv(path: pathlib.Path) -> list[dict]:
    try:
        with open(path, newline="") as f:
            return list(csv.DictReader(f))
    except OSError as e:
        raise InputError(f"{path}: {e}") from e


def reconstruct(log_rows: list[dict], comps: list[dict], segs: dict[int, dict], length: int,
                servos: int = N_SERVOS) -> dict:
    """Every executed read of the chosen length (a reply to that many servos), each observed or excluded with its
    reason."""
    N = reply_bytes(length, servos)
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
    return {"length": length, "N": N, "n": servos, "reads": len(reads), "observations": obs, "first": first,
            "excluded": dict(excluded)}


def coverage(rec: dict, window: tuple[int, ...] = BINS) -> dict:
    """C1 to C5 against their thresholds (C4 descriptive), each with its counts and denominator. C3's bins are the
    window's (by default [650, 850)); C2 is not applicable (ok None) to a reply with no section start."""
    N, length, n = rec["N"], rec["length"], rec.get("n", N_SERVOS)
    counts = collections.Counter(o["d"] for o in rec["observations"] if o["kind"] == "partial")
    c1_short = [d for d in range(1, N) if counts[d] < C1_MIN]
    c2 = {}
    for b in section_starts(length, n):
        short = [x for x in range(b - 3, b + 4) if counts[x] < C2_NEIGH_MIN]
        c2[b] = {"start": counts[b], "neighbourhood_short": short, "ok": not short and counts[b] >= C2_START_MIN}
    bins: collections.Counter = collections.Counter()
    whole: collections.Counter = collections.Counter()
    for f in rec["first"]:
        b = bin_of(f["t_us"], window)
        if b is not None:
            bins[b] += 1
            whole[b] += f["whole"]
    c3_short = [b for b in window if bins[b] < C3_MIN]
    times = sorted(f["t_us"] for f in rec["first"])
    gaps = [y - x for x, y in zip(times, times[1:])]
    n_excl = sum(rec["excluded"].values())
    share = n_excl / rec["reads"] if rec["reads"] else 1.0
    return {
        "partial_counts": {d: counts[d] for d in range(1, N)},
        "C1": {"ok": not c1_short, "min": C1_MIN, "short": c1_short, "denominator": "partial observations"},
        "C2": {"ok": all(v["ok"] for v in c2.values()) if c2 else None, "start_min": C2_START_MIN,
               "neighbourhood_min": C2_NEIGH_MIN, "starts": c2, "denominator": "partial observations"},
        "C3": {"ok": not c3_short, "min": C3_MIN, "bins": {b: bins[b] for b in window},
               "whole": {b: whole[b] for b in window}, "short": c3_short, "denominator": f"first-completion reads ({len(rec['first'])})"},
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
    seq = [(b, cov["C3"]["whole"][b] / cov["C3"]["bins"][b]) for b in cov["C3"]["bins"] if cov["C3"]["bins"][b]]
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
    length, N, n = rec["length"], rec["N"], rec.get("n", N_SERVOS)
    part = [(o["d"], o["t_us"]) for o in rec["observations"] if o["kind"] == "partial"]
    starts = section_starts(length, n)

    def one(d):
        return np.ones_like(d)

    def ident(d):
        return d

    def kb(shift: int):
        return lambda d: np.array([kbar(int(x), length, shift, n) for x in d])
    if not starts:
        return model_no_start(part, N)
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
        out["endpoint_us"] = T0 + w * N + g * (n - 1)
    else:
        out["endpoint_us"] = None
        out["endpoint_note"] = "the partial prefixes do not reach N - 1 or the fit is refused: no endpoint"
    return out


def model_no_start(part: list[tuple[int, float]], N: int) -> dict[str, Any]:
    """A reply to one servo has no section start: the rank-2 fit t = T0' + w d, with no step term by construction
    (so it cannot show the multi-servo steps' mechanism), w fixed at the wire's for its intercept, and no shift check,
    dwell count or local step."""
    def one(d):
        return np.ones_like(d)

    def ident(d):
        return d
    out: dict[str, Any] = {"pooled": fit(part, [one, ident]), "shifts": None,
                           "w_fixed": fit([(d, t - WIRE_US * d) for d, t in part], [one]), "dwell": None,
                           "local_steps": {}, "no_section_start": True}
    p = out["pooled"]
    if "beta" in p and part and max(d for d, _ in part) >= N - 1:
        out["endpoint_us"] = p["beta"][0] + p["beta"][1] * N
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


# ------------------------------------------------------------------------------------------------------------------
# The traffic-reduction diagnostic's Stage A (proposal r2, accepted 2026-11-24 with its scoring amendment;
# review-notes/2026-11-24-traffic-reduction-proposal-r2/ and -r2-review/): one grid-timed timer run per condition, its
# grouped read listing a fixed read set (x7_bus_timing --read-ids) while the write lists every servo.
# label: (the read set, the window L, Q1's band [lo, hi) us, C3's window [lo, lo + 200) us)
STAGE_A = {
    "R8-10": ((2, 3, 4, 5, 6, 7, 8, 9), 10, (620, 670), 540),
    "R4a-10": ((2, 3, 4, 5), 10, (370, 420), 290),
    "R1-10": ((2,), 10, (170, 240), 110),
    "R4a-13": ((2, 3, 4, 5), 13, (400, 460), 330),
    "R4b-10": ((6, 7, 8, 9), 10, (370, 420), 290),
    "R8-10'": ((2, 3, 4, 5, 6, 7, 8, 9), 10, (620, 670), 540),
}
Q2_G, Q3_W = P1_G, P2_W  # the common step [12, 25] us and the slope [3.0, 3.4] us per byte
Q4_DIFF, Q5_DIFF = (25, 55), 15  # the contrasts, on the crossings' from_bin edges (us)
Q4_PAIR, Q5_PAIR = ("R4a-10", "R4a-13"), ("R4a-10", "R4b-10")


def missing(coverage: dict, *keys: str) -> list[str]:
    """The required coverage items not established: each must be explicitly True. Missing, None (not applicable or
    unknown) and False all count as not established (the r2 review's scoring amendment)."""
    return [k for k in keys if coverage.get(k) is not True]


def score_q1(crossings: list[dict], band: tuple[int, int], coverage: dict) -> str:
    """Q1: AS PREDICTED only with exactly one crossing, rising, its from_bin at or above the band's lower limit and its
    to_bin + 10 at or below the upper limit; every crossing kept. Needs C3 and C5."""
    if missing(coverage, "C3", "C5"):
        return "INSUFFICIENT"
    ok = (len(crossings) == 1 and crossings[0]["direction"] == "rising" and crossings[0]["from_bin"] >= band[0]
          and crossings[0]["to_bin"] + 10 <= band[1])
    return "AS PREDICTED" if ok else "NOT AS PREDICTED"


def score_q2(g: float | None, least_shift: int | None, coverage: dict) -> str:
    """Q2 (a run with a section start; never dispatched for one without): the pooled common step g within [12, 25]
    us, with the unshifted boundaries' residual sum the least of the five shifts. Needs C1, C2 and C5."""
    if missing(coverage, "C1", "C2", "C5") or g is None or least_shift is None:
        return "INSUFFICIENT"
    return "AS PREDICTED" if Q2_G[0] <= g <= Q2_G[1] and least_shift == 0 else "NOT AS PREDICTED"


def score_q3(w: float | None, coverage: dict) -> str:
    """Q3: the pooled slope w (one servo: its rank-2 fit's) within [3.0, 3.4] us per byte. Needs C1 and C5."""
    if missing(coverage, "C1", "C5") or w is None:
        return "INSUFFICIENT"
    return "AS PREDICTED" if Q3_W[0] <= w <= Q3_W[1] else "NOT AS PREDICTED"


def single_rising(crossings: list[dict]) -> int | None:
    return crossings[0]["from_bin"] if len(crossings) == 1 and crossings[0]["direction"] == "rising" else None


def score_contrast(cr_a: list[dict], cov_a: dict, cr_b: list[dict], cov_b: dict, lo: int, hi: int) -> str:
    """Runs a and b each need C3 and C5 and exactly one rising crossing; b's from_bin less a's within [lo, hi]."""
    if missing(cov_a, "C3", "C5") or missing(cov_b, "C3", "C5"):
        return "INSUFFICIENT"
    a, b = single_rising(cr_a), single_rising(cr_b)
    if a is None or b is None:
        return "NOT AS PREDICTED"
    return "AS PREDICTED" if lo <= b - a <= hi else "NOT AS PREDICTED"


def score_q4(cr_10: list[dict], cov_10: dict, cr_13: list[dict], cov_13: dict) -> str:
    """Q4: R4a-13's crossing later than R4a-10's by 25 to 55 us (on the 10 us lattice: 30, 40 or 50)."""
    return score_contrast(cr_10, cov_10, cr_13, cov_13, *Q4_DIFF)


def score_q5(cr_a: list[dict], cov_a: dict, cr_b: list[dict], cov_b: dict) -> str:
    """Q5 (descriptive): R4b-10's crossing within 15 us of R4a-10's (on the lattice: 0 or one bin either way)."""
    return score_contrast(cr_a, cov_a, cr_b, cov_b, -Q5_DIFF, Q5_DIFF)


def read_set(summary: dict) -> list[int]:
    """The servos each cycle's grouped read listed: the summary's read_ids (x7_bus_timing --read-ids), else every
    servo of its inventory (a run without the flag reads them all)."""
    if "read_ids" in summary:
        return [int(x) for x in summary["read_ids"]]
    return [int(s["id"]) for s in summary.get("servo", [])]


def analyze_condition(log_rows: list[dict], comps: list[dict], segs: dict[int, dict], summary: dict,
                      label: str) -> dict:
    """One Stage A run against its condition: the read set and window it must have, then the reconstruction for its
    n servos, coverage with its C3 window, the transition, the model (rank 2 for one servo) and Q1 to Q3."""
    ids, length, band, c3 = STAGE_A[label]
    got = read_set(summary)
    if got != list(ids):
        raise InputError(f"the run's read set is {got}, condition {label}'s {list(ids)}")
    lens = {int(r["read_len"]) for r in log_rows}
    if lens != {length}:
        raise InputError(f"the run's reads are {sorted(lens)} bytes, condition {label}'s {length} only")
    n, window = len(ids), tuple(range(c3, c3 + 200, 10))
    rec = reconstruct(log_rows, comps, segs, length, n)
    cov = coverage(rec, window)
    tr = transition(cov, rec)
    m = model(rec) if cov["C5"]["ok"] else None
    scored = {k: cov[k]["ok"] for k in ("C1", "C2", "C3", "C5")}
    p: dict[str, Any] = m["pooled"] if m else {}
    beta = p.get("beta")
    q: dict[str, dict] = {"Q1": {"outcome": score_q1(tr["crossings"], band, scored),
                                 "why": f"{len(tr['crossings'])} crossing(s), every one kept; band [{band[0]}, {band[1]})",
                                 "crossings": tr["crossings"]}}
    if n > 1:
        shifts = m["shifts"] if m else {}
        rss = {s: v for s, v in shifts.items() if v is not None}
        least = min(rss, key=lambda s: rss[s]) if len(rss) == 5 else None
        g = beta[2] if beta else None
        q["Q2"] = {"outcome": score_q2(g, least, scored),
                   "why": f"the common step {g} us; the least residual at shift {least}"}
    else:
        q["Q2"] = {"outcome": "NOT APPLICABLE", "why": "one servo: no section start (a rank-2 fit, no step term)"}
    w = beta[1] if beta else None
    q["Q3"] = {"outcome": score_q3(w, scored), "why": f"the slope {w} us per byte"}
    return {"condition": label, "read_ids": list(ids), "n": n, "length": length, "N": rec["N"],
            "section_starts": section_starts(length, n), "band": list(band), "reads": rec["reads"],
            "excluded": rec["excluded"], "observations": dict(collections.Counter(o["kind"] for o in rec["observations"])),
            "coverage": cov, "coverage_scored": scored,
            "insufficient": [k for k in ("C1", "C2", "C3", "C5") if cov[k]["ok"] is False],
            "distributions": distributions(rec), "transition": tr, "model": m, "predictions": q}


def contrast(which: str, rep_a: dict, rep_b: dict) -> dict:
    """Q4 or Q5 from two condition reports (their JSON): the conditions must be the contrast's pair."""
    pair = Q4_PAIR if which == "Q4" else Q5_PAIR
    if (rep_a.get("condition"), rep_b.get("condition")) != pair:
        raise InputError(f"{which} compares {pair[0]} with {pair[1]}, not {rep_a.get('condition')} with "
                         f"{rep_b.get('condition')}")
    args = (rep_a["transition"]["crossings"], rep_a["coverage_scored"], rep_b["transition"]["crossings"],
            rep_b["coverage_scored"])
    outcome = score_q4(*args) if which == "Q4" else score_q5(*args)
    return {"prediction": which, "pair": list(pair), "outcome": outcome,
            "from_bins": [single_rising(rep_a["transition"]["crossings"]),
                          single_rising(rep_b["transition"]["crossings"])]}


def suff(ok: bool) -> str:
    return "SUFFICIENT" if ok else "INSUFFICIENT"


def print_report(rep: dict, out=sys.stdout) -> None:
    def say(*a: object) -> None:
        print(*a, file=out)
    cov, N = rep["coverage"], rep["N"]
    say(f"# byte arrival, len {rep['length']} (N {N}), section starts {rep['section_starts']}; host stamps after the "
        "OUT completion, not UART events")
    if "condition" in rep:
        say(f"condition {rep['condition']} (the traffic-reduction diagnostic's Stage A): read set {rep['read_ids']} "
            f"({rep['n']} servos; every servo written); Q1's band [{rep['band'][0]}, {rep['band'][1]}) us")
    say(f"executed reads {rep['reads']}; excluded {cov['C5']['excluded']} ({cov['C5']['share']:.4%}): "
        f"{rep['excluded'] or 'none'}; observations: {rep['observations']}")
    sh = cov["C1"]["short"]
    say(f"C1 (every position 1..{N - 1} >= {C1_MIN} partial observations): {suff(cov['C1']['ok'])}; short: "
        f"{sh[:20]}{' ...' if len(sh) > 20 else ''}")
    for b, v in cov["C2"]["starts"].items():
        say(f"C2 at {b}: the start {v['start']} (>= {C2_START_MIN}), neighbours short {v['neighbourhood_short']}: "
            f"{suff(v['ok'])}")
    bins = list(cov["C3"]["bins"])
    say(f"C3 (each bin [{bins[0]}, {bins[-1] + 10}) >= {C3_MIN} of {cov['C3']['denominator']}): "
        f"{suff(cov['C3']['ok'])}; " + " ".join(f"{b}:{cov['C3']['bins'][b]}" for b in bins))
    say(f"C4 (descriptive, non-gating): {cov['C4']}")
    if not cov["C2"]["starts"]:
        say("C2: not applicable (no section start)")
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
    elif m.get("no_section_start"):
        p = m["pooled"]
        say("model (secondary, conditional), one servo, no section start (rank 2, no step term by construction): " + (
            p.get("refused") or (f"T0' {p['beta'][0]:.1f} +- {p['se'][0]:.1f}, w {p['beta'][1]:.3f} +- {p['se'][1]:.3f} "
                                 f"us; rank {p['rank']}, condition {p['cond']:.1f}, residual sd {p['resid_sd']:.1f} us, "
                                 f"n {p['n']}")))
        wf = m["w_fixed"]
        say("  w fixed at the wire's 3.333 us: " + (wf.get("refused") or f"T0' {wf['beta'][0]:.1f} +- {wf['se'][0]:.1f} us"))
        say(f"  the whole-reply endpoint: {m['endpoint_us'] if m['endpoint_us'] is not None else m['endpoint_note']}")
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
    if "condition" in rep:
        say("Q4 and Q5: byte_arrival.py contrast, from two conditions' reports; Q6: the timing summary's (T1 to T7)")
    else:
        say("P5: the timing summary's (T1 to T7), not this tool's")


def read_summary(log: pathlib.Path) -> dict | None:
    """The probe's summary beside the log (LOG.summary.toml), or None if there is none."""
    path = pathlib.Path(str(log) + ".summary.toml")
    if not path.exists():
        return None
    try:
        with open(path, "rb") as f:
            return tomllib.load(f)
    except (OSError, tomllib.TOMLDecodeError) as e:
        raise InputError(f"{path}: {e}") from e


def main_contrast(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(prog="byte_arrival.py contrast",
                                 description="Q4 (R4a-10, R4a-13) or Q5 (R4a-10, R4b-10) from two conditions' --json")
    ap.add_argument("prediction", choices=("Q4", "Q5"))
    ap.add_argument("a", type=pathlib.Path)
    ap.add_argument("b", type=pathlib.Path)
    a = ap.parse_args(argv)
    try:
        out = contrast(a.prediction, json.loads(a.a.read_text()), json.loads(a.b.read_text()))
    except (InputError, KeyError, ValueError, OSError) as e:
        print(f"byte_arrival: {e!r}", file=sys.stderr)
        return 2
    print(f"{out['prediction']} ({out['pair'][0]} against {out['pair'][1]}): {out['outcome']}; the single rising "
          f"crossings' from_bins {out['from_bins']}")
    return 0


def main() -> int:
    if sys.argv[1:2] == ["contrast"]:
        return main_contrast(sys.argv[2:])
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    ap.add_argument("log", type=pathlib.Path)
    ap.add_argument("--completions", type=pathlib.Path, required=True)
    ap.add_argument("--segments", type=pathlib.Path, required=True)
    ap.add_argument("--len", dest="length", type=int, choices=(10, 13), default=None)
    ap.add_argument("--condition", choices=tuple(STAGE_A), default=None,
                    help="a traffic-reduction Stage A run: its read set and window (from the condition, checked "
                         "against the run's summary), its C3 window and Q1 to Q3")
    ap.add_argument("--json", type=pathlib.Path)
    a = ap.parse_args()
    try:
        log_rows = read_csv(a.log)
        comps = read_csv(a.completions)
        segs = {int(r["cycle"]): r for r in read_csv(a.segments)}
        summary = read_summary(a.log)
        if a.condition is not None:
            if a.length is not None:
                raise InputError("--condition sets the window: no --len")
            if summary is None:
                raise InputError(f"{a.log}.summary.toml: a condition needs the run's summary (its read set)")
            rep = analyze_condition(log_rows, comps, segs, summary, a.condition)
        else:
            # A reply to fewer servos than eight is never analysed as eight's: every read would fail reconstruction.
            if summary is not None and "read_ids" in summary and len(summary["read_ids"]) != N_SERVOS:
                raise InputError(f"the run read {summary['read_ids']}: a subset read needs --condition")
            rep = analyze(log_rows, comps, segs, a.length if a.length is not None else 13)
    except (InputError, KeyError, ValueError) as e:
        print(f"byte_arrival: {e!r}", file=sys.stderr)
        return 2
    print_report(rep)
    if a.json:
        a.json.write_text(json.dumps(rep, indent=1, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
