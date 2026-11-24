#!/usr/bin/env python3
"""Regressions for tools/byte_arrival.py, on synthetic exports with known answers.

- the protocol map against include/rtctrl/dxl/wire_budget.hpp (fastSyncReadReplyBytes, kFeedbackBytes): 144 and 120
  bytes, sections of 17 bytes from 25 and of 14 bytes from 22;
- the coverage predicates at their exact thresholds: C1 20 vs 19, C2's start 40 vs 39 and its neighbours 20 vs 19, C3
  30 vs 29 in half-open bins (650.0 in, 849.999 in, 850.0 and 649.999 out), C5 at exactly 1 % vs above;
- reconstruction: a three-completion read after a status-only one, a censored whole reply, and every exclusion in the
  denominator (failed, unassociated, incomplete, completeness unknown, segments missing, not reconstructed);
- the model: a known 18 us step at every section start (pooled and local), no step, a rank-deficient fit refused;
- P3's crossing rule: one rising crossing inside [700, 780), one outside, a nonmonotonic share with three crossings;
- the command line: a report on good input, exit 2 on inconsistent input;
- the traffic-reduction diagnostic's Stage A (--condition): the protocol map for 4 and 1 servos; a subset run's
  reconstruction and common-step model; one servo's rank-2 fit with Q2 not dispatched; the read set and window
  checked against the condition; the Q1 to Q5 scorers at their edges, every required coverage item explicitly
  established (missing, None and False all INSUFFICIENT; the r2 review's required-coverage cases), R1's not
  applicable C2 irrelevant to Q1 and Q3; the contrast's pair; the command line, and the refusal of a subset run
  without --condition.

The generator builds each read from the model's arrival times and a flush lattice, independently of the tool."""

from __future__ import annotations

import csv
import pathlib
import random
import re
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import byte_arrival as ba  # noqa: E402

FAILED = 0


def check(cond: bool, what: str, detail: object = "") -> None:
    global FAILED
    if not cond:
        FAILED += 1
        print(f"FAILED: {what}\n{detail}")


def arrival(j: int, length: int, T0: float, w: float, g: float, servos: int = 8) -> float:
    return T0 + w * (j + 1) + g * sum(1 for b in ba.section_starts(length, servos) if b <= j)


class Run:
    """Synthetic exports: the log rows, the completions and the segments, read by read."""

    def __init__(self) -> None:
        self.log: list[dict] = []
        self.comps: list[dict] = []  # ts_ns, data_bytes, last_of_cycle, reply_complete
        self.segs: dict[int, dict] = {}
        self.t = 1_000_000_000  # ns

    def read(self, deliveries: list[tuple[float, int]], length: int = 13, ok: bool = True, carrier: bool = True,
             complete: str = "1", segments: bool = True, wrong_count: int = 0) -> int:
        """One read: deliveries = [(us after the OUT completion, data bytes)], 0 bytes for a status-only completion."""
        cyc = len(self.log) + 1
        self.log.append({"cycle": str(cyc), "read_len": str(length), "read_ok": "1" if ok else "0"})
        t_out = self.t
        data = []
        for us, n in deliveries:
            ts = t_out + round(us * 1000)
            self.comps.append({"ts_ns": str(ts), "data_bytes": str(n), "last_of_cycle": "", "reply_complete": ""})
            if n:
                data.append((len(self.comps) - 1, ts))
        if data and carrier:
            self.comps[data[-1][0]].update(last_of_cycle=str(cyc), reply_complete=complete)
        if data and segments:
            self.segs[cyc] = {"cycle": str(cyc), "read_len": str(length), "completions": str(len(data) + wrong_count),
                              "to_first_completion_ns": str(data[0][1] - t_out),
                              "between_completions_ns": str(data[-1][1] - data[0][1])}
        self.t += 1_001_001
        return cyc

    def modelled(self, n: int, length: int = 13, T0: float = 130.0, w: float = 10 / 3, g: float = 18.0,
                 flush_us: float = 37.0, seed: int = 3, servos: int = 8) -> None:
        """n reads from the model: a flush lattice of the given period at a random phase, each flush delivering the
        bytes that arrived since the last (a status-only completion before the first byte)."""
        rng = random.Random(seed)
        N = ba.reply_bytes(length, servos)
        for _ in range(n):
            jit = rng.uniform(-3, 3)
            arr = [arrival(j, length, T0, w, g, servos) + jit for j in range(N)]
            f = rng.uniform(0, flush_us)
            got, dl = 0, []
            while got < N:
                now = sum(1 for a in arr if a <= f)
                if now > got or (got == 0 and not dl):
                    dl.append((f, now - got))
                got = now
                f += flush_us
            self.read(dl, length)

    def write(self, root: pathlib.Path, name: str) -> list[pathlib.Path]:
        out = []
        for suffix, rows, cols in ((".csv", self.log, ["cycle", "read_len", "read_ok"]),
                                   (".completions.csv", self.comps, ["ts_ns", "data_bytes", "last_of_cycle", "reply_complete"]),
                                   (".segments.csv", list(self.segs.values()),
                                    ["cycle", "read_len", "completions", "to_first_completion_ns", "between_completions_ns"])):
            p = root / f"{name}{suffix}"
            with open(p, "w", newline="") as f:
                wr = csv.DictWriter(f, fieldnames=cols)
                wr.writeheader()
                wr.writerows(rows)
            out.append(p)
        return out

    def analyze(self, length: int = 13) -> dict:
        return ba.analyze(self.log, self.comps, self.segs, length)


def synthetic_rec(counts: dict[int, int], first_bins: dict[float, int], reads: int, excluded: int = 0,
                  length: int = 13) -> dict:
    """A reconstruction with exact partial counts per position and first-completion times."""
    obs = [{"cycle": 0, "t_us": 0.0, "d": d, "kind": "partial"} for d, k in counts.items() for _ in range(k)]
    first = [{"cycle": 0, "t_us": t, "whole": False} for t, k in first_bins.items() for _ in range(k)]
    return {"length": length, "N": ba.reply_bytes(length), "reads": reads, "observations": obs, "first": first,
            "excluded": {"read failed": excluded} if excluded else {}}


def base_counts(length: int = 13) -> dict[int, int]:
    c = {d: 20 for d in range(1, ba.reply_bytes(length))}
    for b in ba.section_starts(length):
        c[b] = 40
    return c


def base_bins() -> dict[float, int]:
    return {b + 5.0: 30 for b in ba.BINS}


def main() -> int:
    # The protocol map against the wire budget's own formula and window.
    hpp = (HERE.parent / "include/rtctrl/dxl/wire_budget.hpp").read_text()
    check(re.search(r"fastSyncReadReplyBytes\(int n, int len\) \{\s*return 8 \+ n \* \(len \+ 4\);", hpp) is not None,
          "wire_budget.hpp: a Fast Sync Read reply is 8 + n (len + 4) bytes")
    check(re.search(r"constexpr int kFeedbackBytes = 13;", hpp) is not None, "wire_budget.hpp: the full window is 13 bytes")
    check(ba.reply_bytes(13) == 144 and ba.reply_bytes(10) == 120, "144 and 120 bytes for 8 servos")
    check(ba.section_starts(13) == [25, 42, 59, 76, 93, 110, 127], "17-byte sections from 25", ba.section_starts(13))
    check(ba.section_starts(10) == [22, 36, 50, 64, 78, 92, 106], "14-byte sections from 22", ba.section_starts(10))
    check(ba.kbar(25, 13) == 0.5 and ba.kbar(26, 13) == 1 and ba.kbar(24, 13) == 0 and ba.kbar(143, 13) == 7,
          "kbar: half a step at a section start")
    check(ba.kbar(25, 13, shift=1) == 0 and ba.kbar(26, 13, shift=1) == 0.5, "kbar's shifted boundaries")

    # Coverage at the exact thresholds.
    cov = ba.coverage(synthetic_rec(base_counts(), base_bins(), reads=100))
    check(all(cov[k]["ok"] for k in ("C1", "C2", "C3", "C5")), "every threshold met exactly: C1, C2, C3, C5 sufficient", cov)
    check(cov["C4"]["gating"] is False, "C4 is non-gating")
    for b in ba.section_starts(13):
        c = base_counts()
        c[b] = 39
        cov = ba.coverage(synthetic_rec(c, base_bins(), reads=100))
        check(not cov["C2"]["ok"] and not cov["C2"]["starts"][b]["ok"] and cov["C1"]["ok"],
              f"the start {b} at 39: C2 insufficient there, C1 sufficient", cov["C2"]["starts"][b])
    c = base_counts()
    c[42 - 3] = 19
    cov = ba.coverage(synthetic_rec(c, base_bins(), reads=100))
    check(not cov["C1"]["ok"] and cov["C1"]["short"] == [39] and cov["C2"]["starts"][42]["neighbourhood_short"] == [39],
          "a neighbour (b-3) at 19: C1 and C2 insufficient, the position named", (cov["C1"], cov["C2"]["starts"][42]))
    c = base_counts()
    del c[1]
    cov = ba.coverage(synthetic_rec(c, base_bins(), reads=100))
    check(cov["C1"]["short"] == [1] and cov["C2"]["ok"], "a gap at position 1: C1 insufficient, C2 sufficient", cov["C1"])
    c = base_counts()
    c[143] = 19
    check(ba.coverage(synthetic_rec(c, base_bins(), reads=100))["C1"]["short"] == [143], "position 143 at 19: named")
    bins = base_bins()
    bins[845.0] = 29
    cov = ba.coverage(synthetic_rec(base_counts(), bins, reads=100))
    check(cov["C3"]["short"] == [840], "the bin [840, 850) at 29: C3 insufficient there", cov["C3"]["short"])
    bins = {b + 5.0: 30 for b in ba.BINS if b != 650} | {650.0: 30, 849.999: 1, 850.0: 50, 649.999: 50}
    cov = ba.coverage(synthetic_rec(base_counts(), bins, reads=100))
    check(cov["C3"]["bins"][650] == 30 and cov["C3"]["bins"][840] == 31 and cov["C3"]["ok"],
          "half-open bins: 650.0 in [650, 660), 849.999 in [840, 850), 850.0 and 649.999 outside", cov["C3"]["bins"])
    bins = {b + 5.0: 30 for b in ba.BINS if b != 650} | {649.999: 30}
    check(ba.coverage(synthetic_rec(base_counts(), bins, reads=100))["C3"]["short"] == [650],
          "first completions just below 650 do not fill [650, 660)")
    cov = ba.coverage(synthetic_rec(base_counts(), base_bins(), reads=1000, excluded=10))
    check(cov["C5"]["ok"] and cov["C5"]["share"] == 0.01, "C5: 10 of 1000 excluded (exactly 1 %) sufficient", cov["C5"])
    cov = ba.coverage(synthetic_rec(base_counts(), base_bins(), reads=999, excluded=10))
    check(not cov["C5"]["ok"], "C5: 10 of 999 excluded (above 1 %) insufficient", cov["C5"])
    check(not ba.coverage(synthetic_rec({}, {}, reads=0))["C5"]["ok"], "C5: no executed read is insufficient")

    # Reconstruction: three data completions after a status-only one; a censored whole reply; the exclusions.
    r = Run()
    r.read([(400.0, 0), (650.0, 60), (720.0, 70), (790.0, 14)])  # 1: status, 60, 130, 144
    r.read([(700.0, 144)])  # 2: whole in one
    r.read([(700.0, 144)], ok=False)  # 3: failed
    r.read([(700.0, 144)], carrier=False)  # 4: unassociated
    r.read([(700.0, 100)], complete="0")  # 5: incomplete
    r.read([(700.0, 144)], complete="")  # 6: completeness unknown
    r.read([(700.0, 144)], segments=False)  # 7: no segments
    r.read([(650.0, 60), (720.0, 84)], wrong_count=1)  # 8: the segments name three data completions, two found
    r.read([(700.0, 120)], length=10)  # 9: another length, not this analysis's
    rec = ba.reconstruct(r.log, r.comps, r.segs, 13)
    o1 = [(o["d"], o["t_us"], o["kind"]) for o in rec["observations"] if o["cycle"] == 1]
    check(sorted(o1) == [(0, 400.0, "zero"), (60, 650.0, "partial"), (130, 720.0, "partial"), (144, 790.0, "whole")],
          "a three-completion read: a status-only prefix 0, partials 60 and 130, the whole reply", o1)
    f = {x["cycle"]: x for x in rec["first"]}
    check(f[1]["t_us"] == 400.0 and f[1]["whole"] is False and f[1]["kind"] == "zero",
          "its first completion is the status-only one, not whole", f[1])
    check(f[2]["whole"] is True and [o["kind"] for o in rec["observations"] if o["cycle"] == 2] == ["whole"],
          "a whole reply in one completion: censored, first and whole, no partial", f[2])
    check(rec["reads"] == 8 and set(f) == {1, 2}, "the denominator: every executed read of len 13 (8), two reconstructed",
          (rec["reads"], sorted(f)))
    check(rec["excluded"] == {"read failed": 1, "no carrier (unassociated)": 1,
                              "data completions do not add up to the reply": 1, "reply completeness unknown": 1,
                              "segments missing": 1, "completions not reconstructed exactly": 1},
          "each exclusion counted under its reason", rec["excluded"])
    cov = ba.coverage(rec)
    check(cov["C5"]["excluded"] == 6 and cov["C5"]["reads"] == 8 and not cov["C5"]["ok"],
          "C5 counts the six exclusions over all eight executed reads", cov["C5"])
    rep = ba.analyze(r.log, r.comps, r.segs, 13)
    check(rep["model"] is None and all(v["outcome"] == "INSUFFICIENT" for v in rep["predictions"].values()),
          "C5 insufficient: no model, every prediction INSUFFICIENT", rep["predictions"])

    # The model: a known step, no step, rank deficiency.
    r = Run()
    r.modelled(700, g=18.0)
    rep = r.analyze()
    p = rep["model"]["pooled"]
    check(rep["coverage"]["C1"]["ok"] and rep["coverage"]["C2"]["ok"] and rep["coverage"]["C5"]["ok"],
          "the step run's coverage: C1, C2, C5 sufficient", rep["coverage"]["C1"]["short"])
    check(abs(p["beta"][2] - 18.0) < 1.0 and abs(p["beta"][1] - 10 / 3) < 0.05 and p["rank"] == 3,
          "a known 18 us step: the pooled fit recovers g and w", p)
    sh = rep["model"]["shifts"]
    check(min(sh, key=lambda s: sh[s]) == 0, "the unshifted boundaries fit best", sh)
    loc = rep["model"]["local_steps"]
    check(all("beta" in loc[b] and abs(loc[b]["beta"][2] - 18.0) < 3.0 for b in ba.section_starts(13)),
          "the local step at each start near 18 us", {b: loc[b].get("beta") for b in loc})
    wf = rep["model"]["w_fixed"]
    check(abs(wf["beta"][1] - 18.0) < 1.0, "w fixed at 3.333 us: g near 18", wf)
    d = rep["model"]["dwell"]
    check(d is not None and d["start_mean"] > 3 * d["inner_mean"], "dwell: the starts' counts well above the others", d)
    check(rep["predictions"]["P1"]["outcome"] == "AS PREDICTED" and rep["predictions"]["P2"]["outcome"] == "AS PREDICTED",
          "P1 (common step) and P2 as predicted for the 18 us step", rep["predictions"])
    check(rep["predictions"]["P3"]["outcome"] == "INSUFFICIENT", "P3 insufficient: no first completion in [650, 850)",
          rep["predictions"]["P3"])
    r = Run()
    r.modelled(700, g=0.0, seed=5)
    rep = r.analyze()
    p = rep["model"]["pooled"]
    check(abs(p["beta"][2]) < 1.0, "no step: the pooled g near 0", p)
    check(rep["predictions"]["P1"]["outcome"] == "NOT AS PREDICTED", "P1 not as predicted without a step",
          rep["predictions"]["P1"])
    check(all("beta" in v and abs(v["beta"][2]) < 3.0 for v in rep["model"]["local_steps"].values()),
          "no step: each local step near 0")
    one = ba.fit([(30, 100.0)] * 50, [lambda d: d * 0 + 1, lambda d: d, lambda d: d * 0])
    check("refused" in one and one["rank"] < 3, "a rank-deficient design (one position, a null column) refused", one)
    few = ba.fit([(30, 100.0), (31, 103.0), (32, 106.0)], [lambda d: d * 0 + 1, lambda d: d, lambda d: (d > 31) * 1.0])
    check("refused" in few, "as many observations as columns: refused", few)
    loc = ba.model({"length": 13, "N": 144, "observations": [{"d": d, "t_us": 100.0 + d, "kind": "partial"}
                                                            for d in range(1, 20) for _ in range(5)]})
    check(all("refused" in v for v in loc["local_steps"].values()) and loc["endpoint_us"] is None,
          "no observation near any section start: every local step refused; no endpoint short of N - 1", loc["local_steps"])

    # P3's crossing rule on exact shares.
    def p3(shares: dict[int, tuple[int, int]]) -> dict:
        rec = synthetic_rec(base_counts(), {}, reads=10000)
        rec["first"] = [{"cycle": 0, "t_us": b + 5.0, "whole": k < w} for b, (n, w) in shares.items() for k in range(n)]
        cov = ba.coverage(rec)
        return ba.predictions(cov, ba.transition(cov, rec), None)["P3"]
    rising = {b: (40, 0 if b < 740 else 40) for b in ba.BINS}
    out = p3(rising)
    check(out["outcome"] == "AS PREDICTED" and out["crossings"] == [{"from_bin": 730, "to_bin": 740, "direction": "rising"}],
          "one rising crossing between [730, 740) and [740, 750): as predicted", out)
    out = p3({b: (40, 0 if b < 780 else 40) for b in ba.BINS})
    check(out["outcome"] == "NOT AS PREDICTED" and out["crossings"][0]["to_bin"] == 780,
          "a crossing into [780, 790): outside [700, 780), not as predicted", out)
    out = p3({b: (40, 40 if b == 730 else (0 if b < 750 else 40)) for b in ba.BINS})
    check(out["outcome"] == "NOT AS PREDICTED" and out["crossings"] == [
        {"from_bin": 720, "to_bin": 730, "direction": "rising"}, {"from_bin": 730, "to_bin": 740, "direction": "falling"},
        {"from_bin": 740, "to_bin": 750, "direction": "rising"}],
          "a nonmonotonic share (whole in [730, 740), none in [740, 750)): three crossings kept, none smoothed", out)
    out = p3({b: (40, 20 if b == 740 else (0 if b < 740 else 40)) for b in ba.BINS})
    check(out["outcome"] == "AS PREDICTED" and out["crossings"] == [{"from_bin": 730, "to_bin": 740, "direction": "rising"}],
          "a share of exactly 0.5 counts as reached (rising into [740, 750)); staying at or above is no crossing", out)
    out = p3({b: (40, 40) for b in ba.BINS})
    check(out["outcome"] == "NOT AS PREDICTED" and out["crossings"] == [], "whole from the first bin: no crossing", out)
    short = dict(rising)
    short[700] = (29, 0)
    check(p3(short)["outcome"] == "INSUFFICIENT", "a bin at 29 first-completion reads: P3 insufficient")

    # P4: a partial first completion after 750 us.
    rec = synthetic_rec(base_counts(), {}, reads=10000)
    rec["first"] = [{"cycle": 0, "t_us": b + 5.0, "whole": b >= 740} for b in ba.BINS for _ in range(40)]
    rec["first"].append({"cycle": 0, "t_us": 770.0, "whole": False})
    cov = ba.coverage(rec)
    out = ba.predictions(cov, ba.transition(cov, rec), None)["P4"]
    check(out["outcome"] == "AS PREDICTED" and out["why"].endswith("(1 partial data, 0 status only)"),
          "P4: one partial first completion at 770 us", out)
    rec["first"].append({"cycle": 0, "t_us": 790.0, "whole": False, "kind": "zero"})
    cov = ba.coverage(rec)
    out = ba.predictions(cov, ba.transition(cov, rec), None)["P4"]
    check(out["why"].startswith("2 first completion(s)") and out["why"].endswith("(1 partial data, 1 status only)"),
          "P4: a status-only first completion at 790 us counted apart", out)
    rec["first"].pop()
    rec["first"].pop()
    cov = ba.coverage(rec)
    check(ba.predictions(cov, ba.transition(cov, rec), None)["P4"]["outcome"] == "NOT AS PREDICTED", "P4: none after 750 us")

    # The command line.
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        r = Run()
        r.modelled(300, g=18.0)
        log, comps, segs = r.write(root, "run")
        js = root / "run.json"
        p = subprocess.run([sys.executable, str(HERE / "byte_arrival.py"), str(log), "--completions", str(comps),
                            "--segments", str(segs), "--json", str(js)], capture_output=True, text=True)
        check(p.returncode == 0 and "C5 (excluded <= 1% of every executed read): SUFFICIENT" in p.stdout
              and "C3 (each bin [650, 850) >= 30 of first-completion reads (300)): INSUFFICIENT" in p.stdout
              and "P5: the timing summary's" in p.stdout and js.exists(),
              "the command line: a report, coverage named, the JSON written", p.stdout + p.stderr)
        rows = comps.read_text().splitlines()
        comps.write_text("\n".join([rows[0], rows[2], rows[1], *rows[3:]]) + "\n")
        p = subprocess.run([sys.executable, str(HERE / "byte_arrival.py"), str(log), "--completions", str(comps),
                            "--segments", str(segs)], capture_output=True, text=True)
        check(p.returncode == 2 and "not in time order" in p.stderr, "completions out of time order: exit 2", p.stderr)
        p = subprocess.run([sys.executable, str(HERE / "byte_arrival.py"), str(root / "absent.csv"), "--completions",
                            str(comps), "--segments", str(segs)], capture_output=True, text=True)
        check(p.returncode == 2, "an absent log: exit 2", p.stderr)

    stage_a()
    print(f"byte arrival regressions {'OK' if FAILED == 0 else f'{FAILED} FAILED'}")
    return 1 if FAILED else 0


def cr(*items: tuple[int, int, str]) -> list[dict]:
    return [{"from_bin": f, "to_bin": t, "direction": d} for f, t, d in items]


def stage_a() -> None:
    """The traffic-reduction diagnostic's Stage A (--condition)."""
    # The protocol map for subsets (wire_budget.hpp's 8 + n (len + 4)).
    check(ba.reply_bytes(10, 4) == 64 and ba.reply_bytes(13, 4) == 76 and ba.reply_bytes(10, 1) == 22,
          "64, 76 and 22 bytes for 4 servos (10, 13) and 1 servo (10)")
    check(ba.section_starts(10, 4) == [22, 36, 50] and ba.section_starts(13, 4) == [25, 42, 59]
          and ba.section_starts(10, 1) == [], "the starts of 4 servos, and none for 1")
    check(ba.STAGE_A["R1-10"][0] == (2,) and ba.STAGE_A["R4b-10"][0] == (6, 7, 8, 9)
          and ba.STAGE_A["R4a-13"][1] == 13 and ba.STAGE_A["R8-10"][3] == 540 and ba.STAGE_A["R1-10"][3] == 110,
          "the conditions' read sets, windows and C3 windows as r2 declares them")
    eight = {"servo": [{"id": i} for i in range(2, 10)]}

    # A subset run's reconstruction and common-step model: a known 18 us step at 22, 36 and 50.
    r = Run()
    r.modelled(2500, length=10, g=18.0, servos=4)
    rep = ba.analyze_condition(r.log, r.comps, r.segs, {"read_ids": [2, 3, 4, 5]}, "R4a-10")
    check(rep["N"] == 64 and rep["excluded"] == {} and rep["section_starts"] == [22, 36, 50],
          "R4a-10: every read reconstructed exactly as a 64-byte reply", rep["excluded"])
    p = rep["model"]["pooled"]
    check(abs(p["beta"][2] - 18.0) < 1.0 and abs(p["beta"][1] - 10 / 3) < 0.05, "R4a-10: the step 18 us recovered",
          p.get("beta"))
    check(min(rep["model"]["shifts"], key=lambda s: rep["model"]["shifts"][s]) == 0,
          "R4a-10: the residual least at the protocol's starts", rep["model"]["shifts"])
    check(rep["predictions"]["Q2"]["outcome"] == "AS PREDICTED" and rep["predictions"]["Q3"]["outcome"] == "AS PREDICTED",
          "R4a-10: Q2 and Q3 as predicted on covered model data", rep["predictions"])
    check(rep["predictions"]["Q1"]["outcome"] == "INSUFFICIENT" and rep["coverage_scored"]["C3"] is False,
          "R4a-10: Q1 insufficient when C3's window holds no first completion", rep["coverage_scored"])

    # One servo: no section start, a rank-2 fit, Q2 not dispatched.
    r1 = Run()
    r1.modelled(2500, length=10, g=18.0, servos=1)
    rep1 = ba.analyze_condition(r1.log, r1.comps, r1.segs, {"read_ids": [2]}, "R1-10")
    m1 = rep1["model"]
    check(rep1["N"] == 22 and m1["no_section_start"] and m1["pooled"]["rank"] == 2 and m1["shifts"] is None,
          "R1-10: a 22-byte reply, the rank-2 fit", m1["pooled"])
    check(abs(m1["pooled"]["beta"][1] - 10 / 3) < 0.05, "R1-10: the slope recovered", m1["pooled"].get("beta"))
    check(rep1["coverage"]["C2"]["ok"] is None and rep1["coverage_scored"]["C2"] is None, "R1-10: C2 not applicable")
    check(rep1["predictions"]["Q2"]["outcome"] == "NOT APPLICABLE", "R1-10: Q2 not dispatched", rep1["predictions"]["Q2"])
    check(rep1["predictions"]["Q3"]["outcome"] == "AS PREDICTED", "R1-10: Q3 scored without C2", rep1["predictions"]["Q3"])
    out = io_report(rep1)
    check("C2: not applicable (no section start)" in out and "one servo, no section start" in out
          and "C3 (each bin [110, 310)" in out, "R1-10's report", out)

    # The read set and the window are the condition's.
    for summary, label, why in (({"read_ids": [2, 3, 4]}, "R4a-10", "read set"),
                                (eight, "R4a-10", "read set"),
                                ({"read_ids": [2, 3, 4, 5]}, "R4a-13", "13 only")):
        try:
            ba.analyze_condition(r.log, r.comps, r.segs, summary, label)
            check(False, f"{label} accepted a mismatched run ({why})")
        except ba.InputError as e:
            check(why in str(e), f"{label}: the mismatch named ({why})", e)
    check(ba.read_set(eight) == list(range(2, 10)) and ba.read_set({"read_ids": [6, 7, 8, 9], **eight}) == [6, 7, 8, 9],
          "the read set: read_ids, else every inventoried servo")

    # The scorers: every required coverage item explicitly True (the r2 review's required-coverage cases).
    OK = {"C1": True, "C2": True, "C3": True, "C5": True}
    a, b = cr((380, 390, "rising")), cr((420, 430, "rising"))
    cases = [("Q1", ("C3", "C5"), lambda c: ba.score_q1(a, (370, 420), c)),
             ("Q2", ("C1", "C2", "C5"), lambda c: ba.score_q2(18.0, 0, c)),
             ("Q3", ("C1", "C5"), lambda c: ba.score_q3(3.2, c)),
             ("Q4 left", ("C3", "C5"), lambda c: ba.score_q4(a, c, b, OK)),
             ("Q4 right", ("C3", "C5"), lambda c: ba.score_q4(a, OK, b, c)),
             ("Q5 left", ("C3", "C5"), lambda c: ba.score_q5(a, c, a, OK)),
             ("Q5 right", ("C3", "C5"), lambda c: ba.score_q5(a, OK, a, c))]
    for name, keys, call in cases:
        check(call(OK) == "AS PREDICTED", f"{name}: established prerequisites", call(OK))
        for key in keys:
            for state in ("missing", None, False):
                cov = dict(OK)
                if state == "missing":
                    cov.pop(key)
                else:
                    cov[key] = state
                check(call(cov) == "INSUFFICIENT", f"{name}, {key}={state!r}: INSUFFICIENT", call(cov))
    check(ba.score_q1(a, (370, 420), {**OK, "C2": None}) == "AS PREDICTED", "Q1: R1's not applicable C2 is not required")
    check(ba.score_q3(3.2, {**OK, "C2": None}) == "AS PREDICTED", "Q3: R1's not applicable C2 is not required")
    check(ba.score_q1(a, (370, 420), {**OK, "C1": False}) == "AS PREDICTED", "Q1: C1 is not required")

    # The scorers' edges (r2's check_predictions.py cases).
    band = (370, 420)
    for crs, want, what in ((cr((370, 380, "rising")), "AS PREDICTED", "from_bin at the lower limit"),
                            (cr((400, 410, "rising")), "AS PREDICTED", "to_bin + 10 at the upper limit"),
                            (cr((360, 370, "rising")), "NOT AS PREDICTED", "from_bin below the limit"),
                            (cr((410, 420, "rising")), "NOT AS PREDICTED", "to_bin + 10 past the limit"),
                            (cr((380, 390, "falling")), "NOT AS PREDICTED", "one falling crossing"),
                            (cr((380, 390, "rising"), (390, 400, "falling"), (400, 410, "rising")), "NOT AS PREDICTED",
                             "three crossings, every one kept"),
                            ([], "NOT AS PREDICTED", "no crossing")):
        check(ba.score_q1(crs, band, OK) == want, f"Q1: {what}: {want}")
    for g, least, want in ((12.0, 0, "AS PREDICTED"), (25.0, 0, "AS PREDICTED"), (11.99, 0, "NOT AS PREDICTED"),
                           (25.01, 0, "NOT AS PREDICTED"), (18.0, 1, "NOT AS PREDICTED"), (None, 0, "INSUFFICIENT"),
                           (18.0, None, "INSUFFICIENT")):
        check(ba.score_q2(g, least, OK) == want, f"Q2: g {g}, least {least}: {want}")
    for w, want in ((3.0, "AS PREDICTED"), (3.4, "AS PREDICTED"), (3.41, "NOT AS PREDICTED"),
                    (2.99, "NOT AS PREDICTED"), (None, "INSUFFICIENT")):
        check(ba.score_q3(w, OK) == want, f"Q3: w {w}: {want}")
    base = cr((390, 400, "rising"))
    for d, want in ((20, "NOT AS PREDICTED"), (30, "AS PREDICTED"), (40, "AS PREDICTED"), (50, "AS PREDICTED"),
                    (60, "NOT AS PREDICTED")):
        check(ba.score_q4(base, OK, cr((390 + d, 400 + d, "rising")), OK) == want, f"Q4: later by {d}: {want}")
    for d, want in ((-20, "NOT AS PREDICTED"), (-10, "AS PREDICTED"), (0, "AS PREDICTED"), (10, "AS PREDICTED"),
                    (20, "NOT AS PREDICTED")):
        check(ba.score_q5(base, OK, cr((390 + d, 400 + d, "rising")), OK) == want, f"Q5: differs by {d:+d}: {want}")
    check(ba.score_q4(base, OK, cr((420, 430, "rising"), (430, 440, "falling")), OK) == "NOT AS PREDICTED",
          "Q4: two crossings in one run")

    # The contrast's pair, from two reports.
    def fake(label: str, crs: list[dict]) -> dict:
        return {"condition": label, "transition": {"crossings": crs}, "coverage_scored": dict(OK)}
    out = ba.contrast("Q4", fake("R4a-10", base), fake("R4a-13", cr((430, 440, "rising"))))
    check(out["outcome"] == "AS PREDICTED" and out["from_bins"] == [390, 430], "Q4 from two reports", out)
    for which, x, y in (("Q4", "R4a-10", "R4b-10"), ("Q5", "R4a-13", "R4a-10"), ("Q5", "R4b-10", "R4a-10")):
        try:
            ba.contrast(which, fake(x, base), fake(y, base))
            check(False, f"{which} accepted {x} against {y}")
        except ba.InputError:
            pass

    # The command line.
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        log, comps, segs = r.write(root, "subset")
        pathlib.Path(str(log) + ".summary.toml").write_text("read_ids = [2, 3, 4, 5]\n")
        exe = [sys.executable, str(HERE / "byte_arrival.py")]
        p = subprocess.run([*exe, str(log), "--completions", str(comps), "--segments", str(segs)],
                           capture_output=True, text=True)
        check(p.returncode == 2 and "a subset read needs --condition" in p.stderr,
              "a subset run without --condition: refused, exit 2", p.stderr)
        p = subprocess.run([*exe, str(log), "--completions", str(comps), "--segments", str(segs), "--condition",
                            "R4a-10", "--len", "10"], capture_output=True, text=True)
        check(p.returncode == 2 and "no --len" in p.stderr, "--condition with --len: exit 2", p.stderr)
        js = root / "R4a-10.json"
        p = subprocess.run([*exe, str(log), "--completions", str(comps), "--segments", str(segs), "--condition",
                            "R4a-10", "--json", str(js)], capture_output=True, text=True)
        check(p.returncode == 0 and "condition R4a-10 (the traffic-reduction diagnostic's Stage A): read set [2, 3, 4, 5]"
              in p.stdout and "Q2: AS PREDICTED" in p.stdout and "C3 (each bin [290, 490)" in p.stdout and js.exists(),
              "--condition R4a-10: the report and its JSON", p.stdout + p.stderr)
        p = subprocess.run([*exe, "contrast", "Q5", str(js), str(js)], capture_output=True, text=True)
        check(p.returncode == 2 and "Q5 compares R4a-10 with R4b-10" in p.stderr, "contrast: a wrong pair, exit 2",
              p.stderr)
        bare = root / "bare.csv"
        bare.write_text(log.read_text())
        p = subprocess.run([*exe, str(bare), "--completions", str(comps), "--segments", str(segs), "--condition",
                            "R4a-10"], capture_output=True, text=True)
        check(p.returncode == 2 and "needs the run's summary" in p.stderr, "--condition without a summary: exit 2",
              p.stderr)


def io_report(rep: dict) -> str:
    import io
    buf = io.StringIO()
    ba.print_report(rep, out=buf)
    return buf.getvalue()


if __name__ == "__main__":
    sys.exit(main())
