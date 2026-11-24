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
- the command line: a report on good input, exit 2 on inconsistent input.

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


def arrival(j: int, length: int, T0: float, w: float, g: float) -> float:
    return T0 + w * (j + 1) + g * sum(1 for b in ba.section_starts(length) if b <= j)


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
                 flush_us: float = 37.0, seed: int = 3) -> None:
        """n reads from the model: a flush lattice of the given period at a random phase, each flush delivering the
        bytes that arrived since the last (a status-only completion before the first byte)."""
        rng = random.Random(seed)
        N = ba.reply_bytes(length)
        for _ in range(n):
            jit = rng.uniform(-3, 3)
            arr = [arrival(j, length, T0, w, g) + jit for j in range(N)]
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

    print(f"byte arrival regressions {'OK' if FAILED == 0 else f'{FAILED} FAILED'}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
