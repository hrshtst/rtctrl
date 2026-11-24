#!/usr/bin/env python3
"""Regressions for tools/adapter_diagnostic.py on synthetic runs: the fact-only validation accepts every legitimate
diagnostic outcome (no lock, every fit refused, stale and abandoned results, a lock applied on the final close, a loss
and a relock) and rejects inconsistent records; the analysis's windows, rates, denominators and the true-lock
classification (A3, from byte-associated trace carriers and from an emulator record) are as the plan defines them.

The builder makes each run from its own rules (the probe's, as the plan states them), independently of the tool."""

from __future__ import annotations

import copy
import csv
import json
import pathlib
import subprocess
import sys
import tempfile
from collections.abc import Collection

TOOL = pathlib.Path(__file__).resolve().parent / "adapter_diagnostic.py"
PERIOD, L, W, D, DELAY, GATE = 1_000_000, 780_000, 10_000, 20_000, 10_000, 50_000
ORIGIN = 7_000_000_000_000
GRID = ["cycle", "scheduled_wake_ns", "wake_ns", "read_start_ns", "instr_return_ns", "first_reply_ns",
        "last_reply_ns", "read_end_ns", "limit_start_ns", "bus_write_start_ns", "write_return_ns", "drain_return_ns",
        "ack_return_ns", "completion_upper_ns", "cycle_end_ns", "read_ok", "write_ok", "ack_ok", "skipped_periods",
        "read_len"]
ADAPTER = ["target_index", "schedule", "lock_state", "transition", "predicted_completion_ns", "lead_offset_ns",
           "coupling_state", "observation", "recheck_skipped"]
FAILED = 0


def check(cond: bool, what: str, detail: str = "") -> None:
    global FAILED
    if not cond:
        FAILED += 1
        print(f"FAILED: {what}\n{detail}")


class Builder:
    """A run, cycle by cycle: grid cycles on the epoch's periods, lattice cycles from a predicted completion lattice."""

    def __init__(self, duration_s: float = 5, acq_s: float = 2, settle_s: float = 1, alternation: int = D,
                 timing: str = "adapter") -> None:
        self.duration_s, self.acq_s, self.settle_s, self.alt, self.timing = duration_s, acq_s, settle_s, alternation, timing
        self.rows: list[dict] = []
        self.slot = 0  # the next grid wake
        self.locks: list[dict] = []
        self.fits: list[dict] = []
        self.services: list[dict] = []
        self.stored = 0
        self.lattice_c0 = 0
        self.period = 999_942.5
        self.sign = 1

    def _row(self, sched: int, reply: int | None, skipped: int, extra: dict) -> dict:
        cycle = len(self.rows) + 1
        wake = sched + 1_200
        rs = wake + 16
        lr = -1 if reply is None else reply
        re = (rs + 900_000) if reply is None else reply + 300
        wr = re + 1_030
        row = {"cycle": cycle, "scheduled_wake_ns": sched, "wake_ns": wake, "read_start_ns": rs,
               "instr_return_ns": rs + 1_000, "first_reply_ns": lr, "last_reply_ns": lr, "read_end_ns": re,
               "limit_start_ns": re + 20, "bus_write_start_ns": re + 30, "write_return_ns": wr, "drain_return_ns": -1,
               "ack_return_ns": -1, "completion_upper_ns": -1, "cycle_end_ns": wr + 10,
               "read_ok": int(reply is not None), "write_ok": 1, "ack_ok": 0, "skipped_periods": skipped,
               "read_len": 13 if cycle % 100 == 1 else 10}
        if self.timing == "adapter":
            row.update(extra)
        self.rows.append(row)
        return row

    def full(self) -> bool:
        return (len(self.rows) + 1) % 100 == 1

    def grid(self, n: int, state: int = 0, store: bool = False, skips: Collection[int] = (),
             fail: Collection[int] = ()) -> None:
        for j in range(n):
            sched = self.slot
            s = 1 if j in skips else 0
            reply = None if j in fail else sched + 850_000
            obs = 1 if store and reply is not None else 0
            self.stored += obs
            self._row(sched, reply, s, {"target_index": -1, "schedule": 0, "lock_state": state, "transition": 0,
                                        "predicted_completion_ns": -1, "lead_offset_ns": 0, "coupling_state": 0,
                                        "observation": obs, "recheck_skipped": 0})
            self.slot = sched + (1 + s) * PERIOD

    def lock(self, n: int, shift_ns: int = 0, skips: Collection[int] = (), late: Collection[int] = (),
             fail: Collection[int] = (), end: str = "run_end") -> list[dict]:
        """A lock applied on the last row's close, then n lattice cycles (and its end by `end` on the n-th close)."""
        prev = self.rows[-1]
        c0 = self.slot + 2 * PERIOD + shift_ns  # the first target, comfortably after the last cycle
        self.locks.append({"locked_cycle": prev["cycle"], "locked_s": prev["cycle_end_ns"] / 1e9, "end": end,
                           "referenced_cycles": n, "coupling_at_end": "no_coupling_detected"})
        i, rows = 0, []
        self.sign = 1
        for j in range(n):
            pred = c0 + round(i * self.period)
            off = 0 if self.full() else self.sign * self.alt
            if not self.full() and self.alt:
                self.sign = -self.sign
            reply = None if j in fail else pred + DELAY + (GATE + 30_000 if j in late else 2_000)
            e = None if reply is None else reply - pred - DELAY
            obs = 6 if e is None else 3 if abs(e) <= GATE else 4 if e > 0 else 5
            s = 1 if j in skips else 0
            rows.append(self._row(pred - (L + off + W), reply, s, {
                "target_index": i, "schedule": 1, "lock_state": 2, "transition": int(j == 0),
                "predicted_completion_ns": pred, "lead_offset_ns": off, "coupling_state": 1, "observation": obs,
                "recheck_skipped": 0}))
            i += 1 + s
        if end != "run_end":
            self.locks[-1].update(ended_cycle=self.rows[-1]["cycle"], ended_s=self.rows[-1]["cycle_end_ns"] / 1e9)
        last = self.rows[-1]["cycle_end_ns"]
        self.slot = (last // PERIOD + 1) * PERIOD
        return rows

    def fit(self, outcome: str, age: float = 0.015, service: bool = True) -> None:
        cycle = len(self.rows)
        g = sum(f["outcome"] == "requested" for f in self.fits)
        self.fits.append({"cycle": cycle, "outcome": "requested", "generation": g + 1, "window_end_s": 2,
                          "at_s": 2.0})
        rec = {"cycle": cycle, "outcome": outcome, "generation": g + 1, "window_end_s": 2, "at_s": 2.0 + age}
        if outcome in ("applied", "refused", "stale"):
            rec.update(request_to_result_s=age - 0.001, age_s=age, stamps=self.stored, slope_ppm=-57.5,
                       phase0_us=900, share=0.93 if outcome != "refused" else 0.41)
        self.fits.append(rec)
        if service:
            self.services.append({"generation": g + 1, "window_end_s": 2, "started_s": 2.001, "finished_s": 2.012,
                                  "fit_s": 0.011, "stamps": self.stored})

    def window(self, x: dict) -> int:
        t = x["scheduled_wake_ns"]
        return 0 if t < self.acq_s * 1e9 else 1 if t < (self.acq_s + self.settle_s) * 1e9 else 2

    def counts(self, sel: int | None) -> dict:
        c = dict.fromkeys(("cycles", "skipped_periods", "overruns", "consecutive_skips", "referenced_cycles",
                           "grid_cycles"), 0)
        prev, prev_w = False, None
        for x in self.rows:
            w, over = self.window(x), x["skipped_periods"] > 0
            if sel is None or w == sel:
                c["cycles"] += 1
                c["skipped_periods"] += x["skipped_periods"]
                c["overruns"] += over
                c["consecutive_skips"] += (x["skipped_periods"] >= 2) + (over and prev and (sel is None or prev_w == w))
                c["referenced_cycles" if x.get("schedule") == 1 else "grid_cycles"] += 1
            prev, prev_w = over, w
        return c

    def summary(self, state_at_end: int | None = None) -> dict:
        rows = self.rows
        s = {"format": "rtctrl-bus-timing", "version": 2, "kind": "production-traffic", "profile": "current",
             "rate_hz": 1000, "duration_s": self.duration_s, "confirm_writes": False, "drain": False,
             "read_path": "bounded", "feedback": "motion", "read_instruction": "fast_sync", "timing": self.timing,
             "aux_interval_s": 0.1, "full_read_every": 100, "log_complete": True,
             "result": {"started": True, "error": "", "worker_tid": 4242, "monotonic_origin_ns": ORIGIN,
                        "cycles": len(rows), "elapsed_s": rows[-1]["cycle_end_ns"] / 1e9, "late_wake_stop": False,
                        "late_wake_s": 0, "late_wake_lateness_s": 0,
                        "overruns": sum(x["skipped_periods"] > 0 for x in rows),
                        "skipped_periods": sum(x["skipped_periods"] for x in rows),
                        "read_failures": sum(not x["read_ok"] for x in rows), "write_failures": 0, "ack_invalid": 0,
                        "ring_drops": 0, "torque_off_before": True, "torque_off_after": True},
             "bounded_read": {"budget_s": 0.035, "guard_s": 0.035, "expected_latency_timer_ms": 1,
                              "latency_timer_verified": False}}
        if self.timing != "adapter":
            return s
        held = bool(self.locks) and self.locks[-1]["end"] == "run_end"
        state = state_at_end if state_at_end is not None else (2 if held else 0)
        outcomes = [f["outcome"] for f in self.fits]
        ended = [lk for lk in self.locks if lk["end"] != "run_end"]
        a = {"lead_us": L / 1000, "wake_to_read_us": W / 1000, "lead_alternation_us": self.alt / 1000,
             "acquisition_s": self.acq_s, "settling_s": self.settle_s, "publication_margin_us": 20,
             "fit_max_age_s": 0.5, "stamp": "last_reply_ns", "fitter": "main_thread", "reply_delay_us": DELAY / 1000,
             "gate_us": GATE / 1000, "lock_state_at_end": state, "fit_pending_at_end": state == 1,
             "protocol_errors": 0, "exchange_appended": self.stored, "exchange_dropped": 0,
             "gated": sum(x.get("observation") == 3 for x in rows), "late": sum(x.get("observation") == 4 for x in rows),
             "early": sum(x.get("observation") == 5 for x in rows),
             "invalid": sum(x.get("observation") == 6 for x in rows),
             "locks": len(self.locks), "losses": len(ended),
             "lost_outside_gate": 0, "lost_no_gated_stamp": len(ended), "lost_period_clamp": 0, "lost_non_finite": 0,
             "failed_acquisitions": outcomes.count("refused"), "fits_requested": outcomes.count("requested"),
             "fits_applied": outcomes.count("applied"), "fits_refused": outcomes.count("refused"),
             "stale_results": outcomes.count("stale"), "abandoned_requests": outcomes.count("abandoned"),
             "old_generation_results": 0, "queue_failures": 0, "fits_completed": len(self.services),
             "lock_records_dropped": 0, "fit_records_dropped": 0,
             "acquisition": self.counts(0), "settling": self.counts(1), "locked": self.counts(2),
             "whole_run": self.counts(None), "lock": self.locks, "fit": self.fits, "fit_service": self.services}
        if self.alt:
            a["coupling"] = {"delta_us": self.alt / 1000, "window_pairs": 32, "confirm": 8,
                             "state_at_end": "no_coupling_detected", "locks_request_tied": 0,
                             "locks_no_coupling": len(self.locks), "locks_insufficient": 0}
        s["adapter"] = a
        return s

    def write(self, root: pathlib.Path, name: str, summary: dict | None = None) -> pathlib.Path:
        path = root / f"{name}.csv"
        cols = GRID + (ADAPTER if self.timing == "adapter" else [])
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(self.rows)
        (root / f"{name}.csv.summary.toml").write_text(toml(summary or self.summary()))
        return path


def toml(d: dict) -> str:
    def val(v):
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, str):
            return json.dumps(v)
        if isinstance(v, list):
            return "[" + ", ".join(val(x) for x in v) + "]"
        return repr(v)

    def tables(v) -> bool:  # an array of tables, not a list of values
        return isinstance(v, list) and bool(v) and isinstance(v[0], dict)

    def table(prefix: str, t: dict) -> list[str]:
        out = [f"{k} = {val(v)}" for k, v in t.items() if not isinstance(v, dict) and not tables(v)]
        for k, v in t.items():
            name = f"{prefix}.{k}" if prefix else k
            if isinstance(v, dict):
                out += ["", f"[{name}]", *table(name, v)]
            elif tables(v):
                for item in v:
                    out += ["", f"[[{name}]]", *table(name, item)]
        return out
    return "\n".join(table("", d)) + "\n"


def run(*args: str) -> tuple[int, str]:
    p = subprocess.run([sys.executable, str(TOOL), *args], capture_output=True, text=True)
    return p.returncode, p.stdout + p.stderr


def locked_run(duration_s: float = 5) -> Builder:
    """Acquisition on the grid [0, 2 s), a fit applied, a lock from about 2.02 s to the end."""
    b = Builder(duration_s=duration_s)
    b.grid(1998, store=True)
    b.fit("applied")
    b.grid(15, state=1)
    b.lock(round(duration_s * 1000) - 2100)
    return b


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)

        # Validation: every legitimate outcome is valid.
        cases: dict[str, Builder] = {}
        b = Builder()
        b.grid(5000, store=False)
        cases["no lock (no fit requested)"] = b
        b = Builder()
        b.grid(1998, store=True)
        b.fit("refused")
        b.grid(10, state=1)
        b.grid(1500, store=True)
        b.fit("refused")
        b.grid(10, state=1)
        b.grid(1480)
        cases["every fit refused"] = b
        b = Builder()
        b.grid(1998, store=True)
        b.fit("stale", age=0.7)
        b.grid(600, state=1)
        b.grid(1000, store=True)
        b.fit("abandoned", service=False)
        b.grid(1400, state=1)
        cases["a stale result and an abandoned request"] = b
        b = Builder(duration_s=5)
        b.grid(4980, store=True)
        b.fit("applied")
        b.grid(19, state=1)
        cases["a fit applied on the final close"] = b
        b = Builder()
        b.grid(1998, store=True)
        b.fit("applied")
        b.grid(15, state=1)
        b.lock(1000, end="no_gated_stamp")
        b.grid(300, store=True)
        b.fit("applied")
        b.grid(12, state=1)
        b.lock(1500)
        cases["a loss and a relock"] = b
        cases["a held lock"] = locked_run()
        for name, b in cases.items():
            summary = b.summary()
            if name == "a fit applied on the final close":  # the record of the reviewer's endpoint case
                last = b.rows[-1]
                summary["adapter"].update(locks=1, lock_state_at_end=2, fit_pending_at_end=False,
                                          lock=[{"locked_cycle": last["cycle"], "locked_s": last["cycle_end_ns"] / 1e9,
                                                 "end": "run_end", "referenced_cycles": 0,
                                                 "coupling_at_end": "insufficient_evidence"}])
            elif name == "a stale result and an abandoned request":
                summary["adapter"].update(lock_state_at_end=1, fit_pending_at_end=True)
                summary["adapter"]["fits_requested"] = 2  # the abandoned request, still accounted: pending at the end
                summary["adapter"]["abandoned_requests"] = 0
                summary["adapter"]["fit"] = [f for f in summary["adapter"]["fit"] if f["outcome"] != "abandoned"]
            path = b.write(root, name.replace(" ", "-"), summary)
            code, out = run("validate", str(path))
            check(code == 0, f"valid: {name}", out)
        # A grid run is valid as a grid run.
        g = Builder(timing="grid")
        g.grid(4990, skips={7, 8, 900})
        gpath = g.write(root, "grid")
        code, out = run("validate", str(gpath))
        check(code == 0, "valid: a grid run with skips", out)

        # Validation: each inconsistency is refused, and named.
        base = locked_run()
        good = base.summary()
        mutants = {
            "the locked window's counts": lambda s, r: s["adapter"]["locked"].update(cycles=s["adapter"]["locked"]["cycles"] + 1),
            "a lattice wake off its prediction": lambda s, r: r[2500].update(scheduled_wake_ns=r[2500]["scheduled_wake_ns"] + 2),
            "the alternation out of turn": lambda s, r: r[2502].update(lead_offset_ns=-r[2502]["lead_offset_ns"],
                                                                       scheduled_wake_ns=r[2502]["scheduled_wake_ns"] + 2 * r[2502]["lead_offset_ns"]),
            "an observation against its stamp": lambda s, r: r[2600].update(observation=4),
            "the lock's start": lambda s, r: s["adapter"]["lock"][0].update(locked_cycle=s["adapter"]["lock"][0]["locked_cycle"] + 1),
            "the lock's referenced cycles": lambda s, r: s["adapter"]["lock"][0].update(referenced_cycles=17),
            "a loss not recorded": lambda s, r: s["adapter"].update(losses=1),
            "a fit outcome not counted": lambda s, r: s["adapter"].update(fits_applied=0),
            "the skipped periods": lambda s, r: s["result"].update(skipped_periods=3),
            "a ring drop": lambda s, r: s["result"].update(ring_drops=1),
            "torque on after": lambda s, r: s["result"].update(torque_off_after=False),
            "a grid wake off the epoch's periods": lambda s, r: r[100].update(scheduled_wake_ns=r[100]["scheduled_wake_ns"] + 5),
            "a stored stamp without its count": lambda s, r: s["adapter"].update(exchange_appended=s["adapter"]["exchange_appended"] - 1),
            "the lock state at the end": lambda s, r: s["adapter"].update(lock_state_at_end=0),
            "a final-close lock with referenced cycles": lambda s, r: s["adapter"]["lock"].append(
                {"locked_cycle": r[-1]["cycle"], "locked_s": 5.0, "end": "run_end", "referenced_cycles": 3}),
        }
        for name, mutate in mutants.items():
            s, rows = copy.deepcopy(good), copy.deepcopy(base.rows)
            mutate(s, rows)
            m = copy.copy(base)
            m.rows = rows
            path = m.write(root, "mutant", s)
            code, out = run("validate", str(path))
            check(code == 1 and "NOT valid" in out, f"refused: {name}", out)
        # The plan's settings: 20 s at the plan's flags; a wrong lead is named.
        s = copy.deepcopy(good)
        s["duration_s"] = 20
        s["adapter"]["lead_us"] = 800
        path = base.write(root, "plan", s)
        code, out = run("validate", "--plan", "adapter", str(path))
        check(code == 1 and "lead_us is 800" in out and "duration_s is 5" not in out, "the plan's lead is checked", out)

        # The hardware latency check, on the probe's own [bounded_read] shapes (x7_bus_timing: latency_timer_ms, the
        # measured value, when the device's timer was read; latency_timer_verified = false only when it exposes none;
        # no other form of that field). Forged shapes are refused, never accepted.
        lt_cases = [
            ("read and measured at 1 ms (the producer's success shape)", {"latency_timer_ms": 1}, True, True, ""),
            ("measured at 2 ms", {"latency_timer_ms": 2}, True, False, "the measured latency timer is 2 ms, not 1"),
            ("no timer exposed (the producer's false flag)", {"latency_timer_verified": False}, True, False,
             "latency_timer_verified = False"),
            ("neither field", {}, True, False, "no measured latency timer"),
            ("a forged true flag beside a measured 2 ms", {"latency_timer_ms": 2, "latency_timer_verified": True}, True,
             False, "the measured latency timer is 2 ms, not 1"),
            ("a forged true flag beside a measured 1 ms", {"latency_timer_ms": 1, "latency_timer_verified": True}, True,
             False, "latency_timer_verified = True"),
            ("a PTY (no timer exposed), not hardware", {"latency_timer_verified": False}, False, True, ""),
            ("another expected timer", {"latency_timer_ms": 1, "expected_latency_timer_ms": 2}, True, False,
             "the expected latency timer is 2 ms, not 1"),
        ]
        for name, extra, hw, ok_wanted, text in lt_cases:
            s = copy.deepcopy(good)
            s["duration_s"] = 20
            s["bounded_read"] = {"budget_s": 0.035, "guard_s": 0.035, "window_bytes": [13, 10], "reads": len(base.rows),
                                 "timeouts": 0, "malformed": 0, "servo_errors": 0, "recoveries": 0, "guard_waits": 0,
                                 "inhibited": 0, "expected_latency_timer_ms": 1} | extra
            path = base.write(root, "plan-latency", s)
            code, out = run("validate", "--plan", "adapter", *(["--hardware"] if hw else []), str(path))
            check((code == 0) == ok_wanted and text in out, f"the latency timer: {name}", out)

        # The full-feedback diagnostic's run (--plan full-grid): grid timing at 999 Hz, whose probe period is the
        # truncated 1,001,001 ns; 30 s; the session's `fast` variant, every row the full 13-byte window; no motion-only
        # cadence recorded. Each setting off the plan is refused and named; the hardware latency check is the same.
        def full_grid(n: int = 3000, period: int = 1_001_001) -> tuple[Builder, dict]:
            fb = Builder(duration_s=30, timing="grid")
            for j in range(n):
                sched = j * period
                fb._row(sched, sched + 850_000, 0, {})
                fb.rows[-1]["read_len"] = 13
            fs = fb.summary()
            fs.update(rate_hz=999, duration_s=30, feedback="full")
            del fs["aux_interval_s"], fs["full_read_every"]
            fs["bounded_read"] = {"budget_s": 0.035, "guard_s": 0.035, "window_bytes": [13], "reads": n, "timeouts": 0,
                                  "malformed": 0, "servo_errors": 0, "recoveries": 0, "guard_waits": 0, "inhibited": 0,
                                  "expected_latency_timer_ms": 1, "latency_timer_ms": 1}
            return fb, fs

        fb, fs = full_grid()
        path = fb.write(root, "full-grid", fs)
        code, out = run("validate", "--plan", "full-grid", "--hardware", str(path))
        check(code == 0 and "valid (grid timing, 3000 cycles)" in out, "a full-grid run at the plan's settings is valid", out)
        code, out = run("validate", "--plan", "full-grid", str(path))
        check(code == 0, "a full-grid run, without --hardware", out)
        code, out = run("validate", "--plan", "grid", str(path))
        check(code == 1 and "feedback is 'full', the plan's 'motion'" in out and "rate_hz is 999, the plan's 1000" in out,
              "the adapter diagnostic's grid plan still refuses a full-feedback run", out)
        fg_cases = [
            ("1000 Hz", lambda s, r: s.update(rate_hz=1000), "rate_hz is 1000, the plan's 999"),
            ("20 s", lambda s, r: s.update(duration_s=20), "duration_s is 20, the plan's 30"),
            ("motion-only feedback", lambda s, r: s.update(feedback="motion"), "feedback is 'motion', the plan's 'full'"),
            ("Sync Read", lambda s, r: s.update(read_instruction="sync"), "read_instruction is 'sync', the plan's 'fast_sync'"),
            ("the position profile", lambda s, r: s.update(profile="position"), "profile is 'position', the plan's 'current'"),
            ("the legacy read path", lambda s, r: s.update(read_path="legacy"), "read_path is 'legacy', the plan's 'bounded'"),
            ("writes confirmed", lambda s, r: s.update(confirm_writes=True), "confirm_writes is True, the plan's False"),
            ("a drain", lambda s, r: s.update(drain=True), "drain is True, the plan's False"),
            ("a motion-only cadence recorded", lambda s, r: s.update(aux_interval_s=0.1),
             "aux_interval_s is recorded (0.1): a full-feedback run records no motion-only cadence"),
            ("a full-read cadence recorded", lambda s, r: s.update(full_read_every=100),
             "full_read_every is recorded (100): a full-feedback run records no motion-only cadence"),
            ("a motion-only row", lambda s, r: r[1500].update(read_len=10),
             "1 rows read other than the full 13-byte window (the first, cycle 1501)"),
        ]
        for name, mutate, text in fg_cases:
            fb, fs = full_grid()
            mutate(fs, fb.rows)
            path = fb.write(root, "full-grid-mutant", fs)
            code, out = run("validate", "--plan", "full-grid", str(path))
            check(code == 1 and "NOT valid" in out and text in out, f"full-grid refuses: {name}", out)
        # A summary that cannot be read as a run (no rate, or adapter timing without its [adapter] section) is refused
        # before any plan check: exit 2, the missing key named.
        for name, mutate, key in [("no rate recorded", lambda s: s.pop("rate_hz"), "'rate_hz'"),
                                  ("adapter timing without its section", lambda s: s.update(timing="adapter"),
                                   "'acquisition_s'")]:
            fb, fs = full_grid()
            mutate(fs)
            path = fb.write(root, "full-grid-unreadable", fs)
            code, out = run("validate", "--plan", "full-grid", str(path))
            check(code == 2 and key in out and "valid (" not in out, f"full-grid refuses an unreadable summary: {name}", out)
        for name, extra, text in [("measured at 2 ms", {"latency_timer_ms": 2}, "the measured latency timer is 2 ms, not 1"),
                                  ("no timer exposed", {"latency_timer_verified": False, "latency_timer_ms": None},
                                   "latency_timer_verified = False")]:
            fb, fs = full_grid()
            fs["bounded_read"] = {k: v for k, v in (fs["bounded_read"] | extra).items() if v is not None}
            path = fb.write(root, "full-grid-latency", fs)
            code, out = run("validate", "--plan", "full-grid", "--hardware", str(path))
            check(code == 1 and text in out, f"full-grid's hardware latency check: {name}", out)

        # The traffic-reduction diagnostic's Stage A (--plan R8-10 ... R8-10'): grid timing at 999 Hz for 40 s, the
        # condition's read set (the summary's read_ids, or every inventoried servo for an eight-servo run without it),
        # one window every cycle, and the byte counts of a read of the set and a zero write to all eight.
        def stage_a(label: str, n_rows: int = 3000) -> tuple[Builder, dict]:
            ids, length = ad.STAGE_A_PLANS[label]
            sb = Builder(duration_s=40, timing="grid")
            for j in range(n_rows):
                sched = j * 1_001_001
                sb._row(sched, sched + 450_000, 0, {})
                sb.rows[-1]["read_len"] = length
            ss = sb.summary()
            ss.update(rate_hz=999, duration_s=40, feedback="full" if length == 13 else "motion")
            if length == 13:
                del ss["aux_interval_s"], ss["full_read_every"]
            else:
                ss.update(aux_interval_s=0, full_read_every=0)
            ss["read_ids"] = list(ids)
            ss["bounded_read"] = {"budget_s": 0.035, "guard_s": 0.035, "window_bytes": [length], "reads": n_rows,
                                  "timeouts": 0, "malformed": 0, "servo_errors": 0, "recoveries": 0, "guard_waits": 0,
                                  "inhibited": 0, "expected_latency_timer_ms": 1, "latency_timer_ms": 1}
            ss["budget"] = {"read_len": length, "read_servos": len(ids), "write_servos": 8,
                            "read_instruction_bytes": 14 + len(ids), "read_reply_bytes": 8 + len(ids) * (length + 4),
                            "read_stuffing_max_bytes": 0, "write_bytes": 38}
            ss["servo"] = [{"id": i} for i in range(2, 10)]
            return sb, ss

        sys.path.insert(0, str(TOOL.parent))
        import adapter_diagnostic as ad  # noqa: PLC0415
        import byte_arrival as ba  # noqa: PLC0415
        check({k: (v[0], v[1]) for k, v in ba.STAGE_A.items()} == ad.STAGE_A_PLANS,
              "the validator's Stage A read sets and windows are byte_arrival's", ad.STAGE_A_PLANS)
        for label in ad.STAGE_A_PLANS:
            sb, ss = stage_a(label)
            path = sb.write(root, f"stage-a-{label}", ss)
            code, out = run("validate", "--plan", label, "--hardware", str(path))
            check(code == 0 and "valid (grid timing, 3000 cycles)" in out, f"{label} at its condition is valid", out)
        for label in ("R8-10", "R8-10'"):
            sb, ss = stage_a(label)
            del ss["read_ids"], ss["budget"]["read_servos"], ss["budget"]["write_servos"]
            path = sb.write(root, "stage-a-eight-default", ss)
            code, out = run("validate", "--plan", label, str(path))
            check(code == 0, f"{label} made without --read-ids (every inventoried servo read) is valid", out)
        sa_cases = [
            ("R4a-10", "another read set", lambda s, r: s.update(read_ids=[2, 3, 4]),
             "the read set is [2, 3, 4], condition R4a-10's [2, 3, 4, 5]"),
            ("R4b-10", "the first group's servos", lambda s, r: s.update(read_ids=[2, 3, 4, 5]),
             "condition R4b-10's [6, 7, 8, 9]"),
            ("R1-10", "no read_ids for a subset", lambda s, r: s.pop("read_ids"),
             "the read set is not recorded (no read_ids), condition R1-10's [2]"),
            ("R4a-10", "the write's count taken for the read's", lambda s, r: s["budget"].update(write_servos=4),
             "write_servos 4, not 4 and 8"),
            ("R4a-10", "a write to the read set only", lambda s, r: s["budget"].update(write_bytes=26),
             "write_bytes is 26, not 38 (the zero write to all eight)"),
            ("R4a-10", "an instruction for eight", lambda s, r: s["budget"].update(read_instruction_bytes=22),
             "read_instruction_bytes is 22, not 18"),
            ("R4a-13", "a 10-byte reply budget", lambda s, r: s["budget"].update(read_reply_bytes=64),
             "read_reply_bytes is 64, not 76"),
            ("R4a-10", "an inventory of four", lambda s, r: s.update(servo=[{"id": i} for i in (2, 3, 4, 5)]),
             "the inventory lists [2, 3, 4, 5], not every servo"),
            ("R4a-10", "full reads every 100th", lambda s, r: s.update(aux_interval_s=0.1, full_read_every=100),
             "aux_interval_s is 0.1, the plan's 0"),
            ("R4a-10", "a full-window row", lambda s, r: r[700].update(read_len=13),
             "1 rows read other than the 10-byte window (the first, cycle 701)"),
            ("R4a-13", "a motion-only cadence recorded", lambda s, r: s.update(full_read_every=1),
             "full_read_every is recorded (1)"),
            ("R4a-13", "the motion-only feedback", lambda s, r: s.update(feedback="motion"),
             "feedback is 'motion', the plan's 'full'"),
            ("R1-10", "30 s", lambda s, r: s.update(duration_s=30), "duration_s is 30, the plan's 40"),
            ("R1-10", "1000 Hz", lambda s, r: s.update(rate_hz=1000), "rate_hz is 1000, the plan's 999"),
            ("R8-10", "the position profile", lambda s, r: s.update(profile="position"),
             "profile is 'position', the plan's 'current'"),
            ("R8-10", "Sync Read", lambda s, r: s.update(read_instruction="sync"),
             "read_instruction is 'sync', the plan's 'fast_sync'"),
        ]
        for label, name, mutate, text in sa_cases:
            sb, ss = stage_a(label)
            mutate(ss, sb.rows)
            path = sb.write(root, "stage-a-mutant", ss)
            code, out = run("validate", "--plan", label, str(path))
            check(code == 1 and "NOT valid" in out and text in out, f"{label} refuses: {name}", out)
        sb, ss = stage_a("R4a-10")
        ss["bounded_read"]["latency_timer_ms"] = 2
        path = sb.write(root, "stage-a-latency", ss)
        code, out = run("validate", "--plan", "R4a-10", "--hardware", str(path))
        check(code == 1 and "the measured latency timer is 2 ms, not 1" in out, "Stage A's hardware latency check", out)

        # Analysis: windows, rates and the predictions' denominators (an 8 s run: 50 full-window reads in its window).
        b = locked_run(8)
        lattice = [x for x in b.rows if x.get("schedule") == 1]
        in_window = [x for x in lattice if b.window(x) == 2]
        fulls = [i for i, x in enumerate(lattice) if x["read_len"] == 13 and b.window(x) == 2][:10]
        minus = [i for i, x in enumerate(lattice) if x["lead_offset_ns"] < 0 and b.window(x) == 2][:30]
        b = Builder(duration_s=8)
        b.grid(1998, store=True)
        b.fit("applied")
        b.grid(15, state=1)
        b.lock(5900, skips=set(fulls) | set(minus))
        path = b.write(root, "skips")
        js = root / "skips.json"
        code, out = run("analyze", str(path), "--json", str(js))
        check(code == 0, "the analysis runs", out)
        rep = json.loads(js.read_text())
        w = rep["windows"]["locked"]
        check(w["span_s"] == 5 and abs(w["rate_hz"] - w["cycles"] / 5) < 1e-9, "the window's rate is cycles over span", out)
        p = {x["prediction"]: x for x in rep["predictions"]}
        motion = [x for x in b.rows if x.get("schedule") == 1 and b.window(x) == 2 and x["read_len"] == 10]
        check(f"no skip {len(motion) - 30} of {len(motion)}" in p["A6"]["value"],
              "A6 divides the zero-skip motion-only lattice reads by the motion-only lattice reads", p["A6"]["value"])
        n_minus = sum(x["lead_offset_ns"] < 0 for x in motion)
        n_plus = sum(x["lead_offset_ns"] > 0 for x in motion)
        check(f"+20 us: no skip {n_plus} of {n_plus}" in p["A9"]["value"]
              and f"-20 us: no skip {n_minus - 30} of {n_minus}" in p["A9"]["value"],
              "A9 uses each sign's own motion-only reads", p["A9"]["value"])
        nfull = sum(x["read_len"] == 13 for x in b.rows if x.get("schedule") == 1 and b.window(x) == 2)
        check(f"no skip {nfull - 10} of {nfull}" in p["A8"]["value"], "A8's full-window reads", p["A8"]["value"])
        check(p["A3"]["verdict"] == "not testable" and "untraced" in p["A3"]["value"], "A3 untraced: not testable",
              p["A3"]["value"])
        t1 = {c["criterion"]: c for c in rep["criteria"]}["T1"]
        check(t1["detail"].startswith("mean period error"), "T1 is timing_summary's", t1["detail"])
        check(p["A2"]["verdict"] == "held" and "-57.50" in p["A2"]["value"], "A2: the tracked period per lock",
              p["A2"]["value"])
        check(len(in_window) == w["referenced_cycles"], "the window's referenced cycles", out)

        # A3 from byte-associated trace carriers: every class counted in the denominator; the mapping fixed from the
        # first prediction (a lock whose predictions fall between completions is ambiguous, not re-matched).
        b = Builder()
        b.grid(1998, store=True)
        b.fit("applied")
        b.grid(15, state=1)
        # The special reads at rows 1005 to 1100 of the first lock: inside the locked window [3 s, 5 s).
        first = b.lock(1200, fail={1100}, end="no_gated_stamp")
        b.grid(200, store=True)
        b.fit("applied")
        b.grid(12, state=1)
        second = b.lock(1400, shift_ns=500_000)
        path = b.write(root, "carriers")
        # The measured lattice: the first lock's completions 4 us before its predictions, every period, from well
        # before the run to after it; the second lock's predictions then fall midway between completions.
        c0 = first[0]["predicted_completion_ns"] - 4_000
        start = -3000
        comps = [c0 + round(k * b.period) for k in range(start, 6000)]
        carried = {}
        for j, x in enumerate(first):
            if x["read_ok"]:
                k = x["target_index"] - start
                if j == 1005:
                    k += 1  # carried by the completion after its target (the next read's, which has none)
                if j in (1006, 1015):
                    continue  # no carrier: unassociated
                carried[k] = (x["cycle"], 0 if j == 1025 else 1)  # 1025: data not adding up
        for x in second:
            k = round((x["predicted_completion_ns"] - c0) / b.period) - start
            carried[k] = (x["cycle"], 1)
        cpath = root / "completions.csv"
        with open(cpath, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["index", "ts_ns", "data_bytes", "last_of_cycle", "reply_complete"])
            for k, ts in enumerate(comps):
                cy, ok = carried.get(k, ("", ""))
                wr.writerow([k, ORIGIN + ts, 122 if cy != "" else 0, cy, ok])
        js = root / "carriers.json"
        code, out = run("analyze", str(path), "--completions", str(cpath), "--json", str(js))
        check(code == 0, "the analysis with trace carriers runs", out)
        a3 = json.loads(js.read_text())["true_lock"]
        win = [x for x in b.rows if x.get("schedule") == 1 and b.window(x) == 2]
        first_w = [x for x in first if b.window(x) == 2]
        second_w = [x for x in second if b.window(x) == 2]
        cls = a3["classes"]
        check(a3["denominator"] == len(win) == sum(cls.values()), "every lattice read of the window is in A3's "
              "denominator", json.dumps(a3))
        idx = {x["cycle"]: j for j, x in enumerate(first)}
        check(all(b.window(first[j]) == 2 for j in (1005, 1006, 1015, 1025, 1100)), "the special reads are in the window")
        expect_later = 1
        check(cls["carried_later"] == 1 and cls["unassociated"] == 3 and cls["incomplete"] == 1,
              "carried later, unassociated (no carrier, data not adding up) and incomplete", json.dumps(cls))
        check(idx[first_w[0]["cycle"]] > 0, "the first lock starts before the window", json.dumps(cls))
        check(cls["ambiguous"] == len(second_w), "a lock between completions: every read ambiguous", json.dumps(cls))
        check(cls["carried_by_target"] == len(first_w) - expect_later - cls["unassociated"] - cls["incomplete"],
              "the rest carried by their targets", json.dumps(cls))
        check(abs(a3["median_prediction_less_carrier_us"] - 4.0) < 0.01, "the median prediction less carrier",
              json.dumps(a3))
        # Every carrier one completion late: all carried later; nothing re-matches them to look successful.
        late = {k + 1: v for k, v in carried.items()}
        with open(cpath, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["index", "ts_ns", "data_bytes", "last_of_cycle", "reply_complete"])
            for k, ts in enumerate(comps):
                cy, ok = late.get(k, ("", ""))
                wr.writerow([k, ORIGIN + ts, 122 if cy != "" else 0, cy, ok])
        code, out = run("analyze", str(path), "--completions", str(cpath), "--json", str(js))
        cls = json.loads(js.read_text())["true_lock"]["classes"]
        check(cls["carried_by_target"] == 0 and cls["carried_later"] > 0, "late carriers stay late", json.dumps(cls))

        # A3 from an emulator record: one held reply per read, its completion's index and instant.
        b = locked_run()
        path = b.write(root, "emu")
        lat = [x for x in b.rows if x.get("schedule") == 1]
        e0 = lat[0]["predicted_completion_ns"] + 3_000
        recs = []
        for j, x in enumerate(b.rows):
            if not x["read_ok"]:
                continue
            arrival = x["read_start_ns"] + 60_000
            if x.get("schedule") == 1:
                k = x["target_index"] + 9000
                comp = e0 + round(x["target_index"] * b.period)
            else:
                k, comp = 1, 1  # the grid's replies: their completions do not matter here
            if j == len(b.rows) - 5:
                continue  # unassociated
            recs.append((arrival, k, comp))
            if j == len(b.rows) - 9:
                recs.append((arrival + 1_000, k, comp))  # two held replies in one read: ambiguous
        epath = root / "emu.emu.csv"
        with open(epath, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(["arrival_ns", "instruction", "id", "data_length", "bytes", "ready_ns", "index",
                         "completion_ns", "release_ns", "delayed", "missed", "due_ns", "written_ns", "written_bytes",
                         "write_error"])
            for arrival, k, comp in recs:
                wr.writerow([ORIGIN + arrival, 0x8A, 254, 10, 120, ORIGIN + arrival, k, ORIGIN + comp, ORIGIN + comp,
                             0, 0, ORIGIN + comp, ORIGIN + comp + 5_000, 120, 0])
        code, out = run("analyze", str(path), "--emulator-record", str(epath), "--json", str(js))
        check(code == 0, "the analysis with an emulator record runs", out)
        a3 = json.loads(js.read_text())["true_lock"]
        cls = a3["classes"]
        check(cls["unassociated"] == 1 and cls["ambiguous"] == 1 and
              cls["carried_by_target"] == a3["denominator"] - 2, "emulator carriers classified", json.dumps(a3))
        check(abs(a3["median_prediction_less_carrier_us"] + 3.0) < 0.01, "the emulator median", json.dumps(a3))

        # Small windows: insufficient evidence, never a verdict.
        b = Builder(duration_s=3.5)
        b.grid(1998, store=True)
        b.fit("applied")
        b.grid(15, state=1)
        b.lock(1400)
        path = b.write(root, "short")
        code, out = run("analyze", str(path), "--json", str(js))
        p = {x["prediction"]: x for x in json.loads(js.read_text())["predictions"]}
        check(all(p[k]["verdict"] == "insufficient evidence" for k in ("A5", "A6", "A9", "A10")),
              "half a second's window: insufficient evidence", json.dumps(p))

        # The endpoint lock's analysis: no lattice row, so no locked read to judge.
        b = cases["a fit applied on the final close"]
        path = root / "a-fit-applied-on-the-final-close.csv"
        code, out = run("analyze", str(path), "--json", str(js))
        p = {x["prediction"]: x for x in json.loads(js.read_text())["predictions"]}
        check(code == 0 and p["A1"]["verdict"] == "failed" and p["A2"]["verdict"] == "not testable",
              "an endpoint lock: A1 failed, A2 not testable", json.dumps(p))

        # A grid run against its mode's adapter run (G).
        code, out = run("analyze", str(gpath), "--compare-adapter", str(root / "a-held-lock.csv"))
        check(code == 0 and "below the adapter run's locked window" in out, "G compares the matched windows", out)

    if FAILED:
        print(f"adapter diagnostic regressions: {FAILED} FAILED")
        return 1
    print("adapter diagnostic regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
