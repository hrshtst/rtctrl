#!/usr/bin/env python3
"""The torque-off adapter-timing diagnostic: fact-only validation of an x7_bus_timing run's records, and the
analysis of its declared windows and exploratory predictions (the diagnostic plan, revision 2).

usage: adapter_diagnostic.py validate LOG [--plan grid|adapter|full-grid [--hardware]]
       adapter_diagnostic.py analyze LOG [--completions CSV | --emulator-record CSV] [--loaded]
                                         [--compare-adapter LOG] [--json FILE]

LOG is the probe's per-cycle log (its summary at LOG.summary.toml), a grid run (`--timing grid`) or an adapter run
(`--timing adapter`).

validate checks only facts: that the log and its summary agree, and that each row is consistent with the run's
declared configuration and the probe's rules. It never requires an outcome (a fit applied, a lock, a minimum count of
fits): a run with no lock, with every fit refused, with stale or abandoned results, or with a fit applied on the final
close (a lock recorded with no executed lattice row) is valid when its records agree. Exit 0 valid, 1 not (each failed
check listed), 2 unreadable.

analyze reports the run over four fixed spans by each cycle's scheduled wake (scheduled_wake_ns, from the log's
epoch): acquisition [0, A), settling [A, A + S), the locked measurement [A + S, D), and the whole run; A and S are the
adapter summary's (2 s and 1 s for a grid run, the plan's), D the run's duration. A window's rate is its cycles over
its span (descriptive); the whole run's T1 to T7 are timing_summary.py's, its wake-interval mean period included. It
then judges the plan's exploratory predictions: A1 to A10 and W for an adapter run, G for a grid run. Each is held,
failed, insufficient evidence (its denominator below the declared minimum) or not testable (with the reason); counts
are given beside every share. A failed prediction is a valid observation, never a validity failure.

The true-lock statistics (A3) need the carriers: for a hardware timer run, --completions (trace_correlate.py
--completions-csv: every bulk IN completion of the run, and the read whose last reply bytes each carried, by byte
association); for a PTY rehearsal, --emulator-record (dxl_emu --cadence-log, the completion that carried each held
reply). An untraced run has neither, and A3 is not testable. The mapping of each read's targeted completion to the
measured lattice never looks at carriers: per lock, its first lattice row's predicted completion is mapped to the
nearest measured completion (within a quarter period, or the lock's reads are ambiguous), which fixes the offset
between the probe's target_index and the measured index for the whole lock; each later read's mapped completion must
again lie within a quarter period of its prediction, or that read is ambiguous. Every lattice read of the locked
window is in A3's denominator: carried by its target, carried later, carried earlier, incomplete (no reply),
unassociated (no carrier, or a hardware reply whose data completions do not add up to it) or ambiguous.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import itertools
import json
import pathlib
import sys
import tomllib
from collections.abc import Sequence

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import timing_summary  # noqa: E402  (the criteria, as the session computes them)

GRID_COLUMNS = ["cycle", "scheduled_wake_ns", "wake_ns", "read_start_ns", "instr_return_ns", "first_reply_ns",
                "last_reply_ns", "read_end_ns", "limit_start_ns", "bus_write_start_ns", "write_return_ns",
                "drain_return_ns", "ack_return_ns", "completion_upper_ns", "cycle_end_ns", "read_ok", "write_ok",
                "ack_ok", "skipped_periods", "read_len"]
ADAPTER_COLUMNS = ["target_index", "schedule", "lock_state", "transition", "predicted_completion_ns",
                   "lead_offset_ns", "coupling_state", "observation", "recheck_skipped"]
FULL_LEN, MOTION_LEN = 13, 10
GROUPED_READS = {0x82, 0x8A}  # Sync Read, Fast Sync Read (the emulator record's instruction)
# The observation codes (rtctrl/hw/adapter_phase.hpp, Observation).
STORED, GATED, LATE, EARLY, INVALID = 1, 3, 4, 5, 6
# The plan's fixed spans for a grid run (an adapter run's come from its summary).
PLAN_ACQUISITION_S, PLAN_SETTLING_S = 2.0, 1.0
# The probe's fixed values, as its summary records them (provisional diagnostic values, not demonstrated bounds).
PUBLICATION_MARGIN_US, FIT_MAX_AGE_S = 20, 0.5
QUARTER_PERIOD_NS = 250_000  # the mapping's tolerance: a prediction within it names one completion of a 1 ms lattice

# The exploratory predictions' thresholds and minimum denominators (the plan, revision 2), declared before any run.
A1_LOCK_BEFORE_S = 2.5
A1_REFERENCED_SHARE = 0.99
A2_PPM = (-70.0, -15.0)
A3_SHARE, A3_MEDIAN_US, A3_MIN = 0.99, 15.0, 1000
A5_GATED, A5_EARLY, A5_MIN = 0.99, 0.001, 1000
A6_SHARE, A6_MIN = 0.995, 1000
A7_RATE_HZ, A7_HALVES_HZ = 990.0, 5.0
A8_SHARE, A8_MAX_SKIP, A8_MIN = (0.60, 0.95), 1, 50
A9_SHARE, A9_MIN = 0.99, 500
A10_US, A10_MIN = (780.0, 900.0), 1000
W_RATE_HZ = (940.0, 1000.0)
G_RATE_HZ, G_BELOW_HZ = (550.0, 800.0), 150.0


# ------------------------------------------------------------------------------------------------ loading

class Run:
    def __init__(self, path: pathlib.Path) -> None:
        self.path = path
        with open(str(path) + ".summary.toml", "rb") as f:
            self.s = tomllib.load(f)
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            self.header = reader.fieldnames or []
            self.rows = [{k: int(v) for k, v in r.items()} for r in reader]
        self.timing = self.s.get("timing", "grid")
        self.adapter = self.s.get("adapter", {})
        self.duration_ns = round(float(self.s["duration_s"]) * 1e9)
        self.period_ns = round(1e9 / float(self.s["rate_hz"]))
        if self.timing == "adapter":
            self.acq_ns = round(float(self.adapter["acquisition_s"]) * 1e9)
            self.settle_ns = round(float(self.adapter["settling_s"]) * 1e9)
        else:
            self.acq_ns, self.settle_ns = round(PLAN_ACQUISITION_S * 1e9), round(PLAN_SETTLING_S * 1e9)
        self.origin = int(self.s["result"]["monotonic_origin_ns"])

    def window_of(self, row: dict) -> int:
        """0 acquisition, 1 settling, 2 the locked measurement, by the cycle's scheduled wake."""
        t = row["scheduled_wake_ns"]
        return 0 if t < self.acq_ns else 1 if t < self.acq_ns + self.settle_ns else 2

    def spans_s(self) -> dict[str, float]:
        a, s, d = self.acq_ns / 1e9, self.settle_ns / 1e9, self.duration_ns / 1e9
        return {"acquisition": a, "settling": s, "locked": d - a - s, "whole_run": d}


WINDOWS = (("acquisition", 0), ("settling", 1), ("locked", 2), ("whole_run", None))


def skip_counts(run: Run, sel: int | None) -> dict:
    """The probe's SkipAccounting, recomputed: a row's skips are those its close produced; a run of consecutive skips
    counts once per extra period, and across adjacent overrunning rows of the same window (or of the run)."""
    c = {"cycles": 0, "skipped_periods": 0, "overruns": 0, "consecutive_skips": 0, "referenced_cycles": 0,
         "grid_cycles": 0}
    prev, prev_w = False, None
    for row in run.rows:
        w = run.window_of(row)
        over = row["skipped_periods"] > 0
        if sel is None or w == sel:
            c["cycles"] += 1
            c["skipped_periods"] += row["skipped_periods"]
            c["overruns"] += over
            c["consecutive_skips"] += row["skipped_periods"] >= 2
            c["consecutive_skips"] += over and prev and (sel is None or prev_w == w)
            if run.timing == "adapter":
                c["referenced_cycles" if row["schedule"] == 1 else "grid_cycles"] += 1
            else:
                c["grid_cycles"] += 1
        prev, prev_w = over, w
    return c


# --------------------------------------------------------------------------------------------- validation

def validate(run: Run) -> list[str]:
    """The failed facts (empty: the records are consistent)."""
    bad: list[str] = []

    def need(ok: bool, what: str) -> None:
        if not ok:
            bad.append(what)

    s, r, rows = run.s, run.s.get("result", {}), run.rows
    need(s.get("format") == "rtctrl-bus-timing" and s.get("kind") == "production-traffic",
         "the summary is not a production-traffic probe summary")
    need(run.timing in ("grid", "adapter"), f"unknown timing {run.timing!r}")
    want = GRID_COLUMNS + (ADAPTER_COLUMNS if run.timing == "adapter" else [])
    need(run.header == want, "the log's header is not the probe's for its timing")
    need(bool(s.get("log_complete")), "the summary does not record the log complete")
    need(r.get("ring_drops") == 0, f"ring drops {r.get('ring_drops')}")
    need(bool(r.get("torque_off_before")) and bool(r.get("torque_off_after")), "torque off not recorded before and after")
    need(len(rows) == r.get("cycles"), f"{len(rows)} rows against the summary's {r.get('cycles')} cycles")
    need([x["cycle"] for x in rows] == list(range(1, len(rows) + 1)), "the cycles are not numbered 1, 2, ... in order")
    need(sum(x["skipped_periods"] for x in rows) == r.get("skipped_periods"), "the skipped periods differ from the summary's")
    need(sum(x["skipped_periods"] > 0 for x in rows) == r.get("overruns"), "the overruns differ from the summary's")
    need(all(x["scheduled_wake_ns"] < run.duration_ns for x in rows), "a cycle scheduled at or after the deadline")
    need(all(a["scheduled_wake_ns"] < b["scheduled_wake_ns"] for a, b in itertools.pairwise(rows)),
         "the scheduled wakes are not increasing")
    if run.timing == "grid":
        need("adapter" not in s, "a grid run's summary has an [adapter] section")
        need(all(x["scheduled_wake_ns"] % run.period_ns == 0 for x in rows), "a grid wake off the epoch's periods")
        return bad
    return bad + validate_adapter(run)


# The plan's settings, as the summary must record them (validity item 8 of the plan, revision 2).
PLAN_COMMON = {"rate_hz": 1000, "duration_s": 20, "profile": "current", "read_path": "bounded",
               "feedback": "motion", "read_instruction": "fast_sync", "aux_interval_s": 0.1, "confirm_writes": False,
               "drain": False}
PLAN_ADAPTER = {"lead_us": 780, "wake_to_read_us": 10, "lead_alternation_us": 20, "acquisition_s": 2, "settling_s": 1}
# The full-feedback, grid-timed diagnostic's settings (its plan r1 and addendum, 2026-11-24): the session's variant
# `fast` (Fast Sync Read of the full 13-byte window every cycle), 999 Hz for 30 s. A full-feedback run records no
# motion-only cadence (aux_interval_s, full_read_every), and every row reads the full window.
PLAN_FULL_GRID = {"rate_hz": 999, "duration_s": 30, "profile": "current", "read_path": "bounded", "feedback": "full",
                  "read_instruction": "fast_sync", "confirm_writes": False, "drain": False}
FULL_WINDOW_LEN = 13


def check_plan(run: Run, arm: str, hardware: bool) -> list[str]:
    """The run's recorded settings against the plan's for its arm: the adapter-timing diagnostic's grid or adapter
    arm, or the full-feedback diagnostic's full-grid run."""
    plan = PLAN_FULL_GRID if arm == "full-grid" else PLAN_COMMON
    bad = [f"{k} is {run.s.get(k)!r}, the plan's {v!r}" for k, v in plan.items() if run.s.get(k) != v]
    timing = "grid" if arm == "full-grid" else arm
    if run.timing != timing:
        bad.append(f"timing is {run.timing!r}, the plan's {timing!r}" + (" (the full-feedback run)" if arm == "full-grid"
                                                                         else " arm"))
    if arm == "adapter":
        bad += [f"[adapter] {k} is {run.adapter.get(k)!r}, the plan's {v!r}" for k, v in PLAN_ADAPTER.items()
                if run.adapter.get(k) != v]
    if arm == "full-grid":
        bad += [f"{k} is recorded ({run.s[k]!r}): a full-feedback run records no motion-only cadence"
                for k in ("aux_interval_s", "full_read_every") if k in run.s]
        other = [x["cycle"] for x in run.rows if x["read_len"] != FULL_WINDOW_LEN]
        if other:
            bad.append(f"{len(other)} rows read other than the full {FULL_WINDOW_LEN}-byte window (the first, cycle "
                       f"{other[0]})")
    br = run.s.get("bounded_read", {})
    if br.get("expected_latency_timer_ms") != 1:
        bad.append(f"the expected latency timer is {br.get('expected_latency_timer_ms')!r} ms, not 1")
    if hardware:
        # The probe's schema (x7_bus_timing's [bounded_read]): a device whose latency timer was read records the
        # measured value, latency_timer_ms; one exposing none records latency_timer_verified = false, and the probe
        # writes that field in no other form. A hardware run needs the measured 1 ms and no such field.
        measured = br.get("latency_timer_ms")
        if "latency_timer_verified" in br:
            bad.append(f"latency_timer_verified = {br['latency_timer_verified']!r}: the adapter's latency timer was "
                       "not read (the probe writes the field only when the device exposes none)")
        if measured is None:
            bad.append("no measured latency timer (latency_timer_ms) in the summary (a hardware run)")
        elif measured != 1 or isinstance(measured, bool):
            bad.append(f"the measured latency timer is {measured!r} ms, not 1")
    return bad


def validate_adapter(run: Run) -> list[str]:
    bad: list[str] = []

    def need(ok: bool, what: str) -> None:
        if not ok:
            bad.append(what)

    a, rows = run.adapter, run.rows
    need(a.get("protocol_errors") == 0, f"protocol errors {a.get('protocol_errors')}")
    need(a.get("fitter") == "main_thread" and a.get("stamp") == "last_reply_ns", "not the main thread's fit of last_reply_ns")
    need(a.get("publication_margin_us") == PUBLICATION_MARGIN_US and a.get("fit_max_age_s") == FIT_MAX_AGE_S,
         "the publication margin or fit age differs from the probe's fixed values")
    need(a.get("lock_records_dropped") == 0 and a.get("fit_records_dropped") == 0, "lock or fit records dropped")
    L, W = round(a["lead_us"] * 1000), round(a["wake_to_read_us"] * 1000)
    D = round(a["lead_alternation_us"] * 1000)

    # The declared windows and the whole run.
    for name, sel in WINDOWS:
        need(skip_counts(run, sel) == a.get(name), f"the {name} window's counts differ from the summary's")

    # Each row: its plan on the grid (the epoch's whole periods, no target) or on the lattice (its wake from its
    # predicted completion, one rounding); full-window reads at the base lead; motion-only reads alternating from +
    # over each run of lattice rows when the alternation is on.
    expect = 1
    for x in rows:
        need(0 <= x["recheck_skipped"] <= x["skipped_periods"], f"cycle {x['cycle']}: recheck skips beyond its skips")
        if x["schedule"] == 0:
            need(x["scheduled_wake_ns"] % run.period_ns == 0, f"cycle {x['cycle']}: a grid plan off the epoch's periods")
            need(x["target_index"] == -1 and x["predicted_completion_ns"] == -1 and x["lead_offset_ns"] == 0,
                 f"cycle {x['cycle']}: a grid plan with a target")
            expect = 1
            continue
        need(x["schedule"] == 1, f"cycle {x['cycle']}: unknown schedule {x['schedule']}")
        want = x["predicted_completion_ns"] - (L + x["lead_offset_ns"] + W)
        need(abs(x["scheduled_wake_ns"] - want) <= 1, f"cycle {x['cycle']}: its wake is not its prediction less its lead")
        if x["read_len"] == FULL_LEN:
            need(x["lead_offset_ns"] == 0, f"cycle {x['cycle']}: a full-window read off the base lead")
        elif D:
            need(x["lead_offset_ns"] == expect * D, f"cycle {x['cycle']}: the alternation out of turn")
            expect = -expect
        else:
            need(x["lead_offset_ns"] == 0, f"cycle {x['cycle']}: a lead offset without the alternation")

    # Each locked lattice row's observation, from its own stamp against its plan's prediction (within 1 ns of the
    # gate's edge, not judged; an out-of-order stamp is recorded invalid).
    delay, gate = a["reply_delay_us"] * 1000, a["gate_us"] * 1000
    for x in rows:
        if x["schedule"] != 1 or x["lock_state"] != 2:
            continue
        if not x["read_ok"] or x["last_reply_ns"] < 0:
            need(x["observation"] == INVALID, f"cycle {x['cycle']}: no reply, not observed invalid")
            continue
        e = x["last_reply_ns"] - x["predicted_completion_ns"] - delay
        if abs(abs(e) - gate) <= 1 or x["observation"] == INVALID:
            continue
        want = GATED if abs(e) <= gate else LATE if e > 0 else EARLY
        need(x["observation"] == want, f"cycle {x['cycle']}: observed {x['observation']}, its stamp says {want}")

    # The fits: each request accounted for once (or pending at the end); the results' ages; the main thread's services.
    fits = a.get("fit", [])
    outcomes = [f["outcome"] for f in fits]
    need(outcomes.count("requested") == a.get("fits_requested"), "the fit requests differ from the summary's")
    for key, name in (("fits_applied", "applied"), ("fits_refused", "refused"), ("stale_results", "stale"),
                      ("abandoned_requests", "abandoned"), ("old_generation_results", "old_generation"),
                      ("queue_failures", "queue_failed")):
        need(outcomes.count(name) == a.get(key), f"the {name} fit records differ from the summary's {key}")
    pending = 1 if a.get("fit_pending_at_end") else 0
    need(a.get("fits_requested") == a.get("fits_applied", 0) + a.get("fits_refused", 0) + a.get("stale_results", 0)
         + a.get("abandoned_requests", 0) + pending, "a fit request not accounted for")
    for f in fits:
        if f["outcome"] in ("applied", "refused"):
            need(0 <= f["age_s"] <= FIT_MAX_AGE_S and f["request_to_result_s"] >= 0, f"a {f['outcome']} fit's age")
        if f["outcome"] == "stale":
            need(f["age_s"] > FIT_MAX_AGE_S, "a stale fit within the age limit")
    services = a.get("fit_service", [])
    need(len(services) == a.get("fits_completed"), "the fit services differ from the fits completed")
    for sv in services:
        need(sv["finished_s"] >= sv["window_end_s"] and sv["finished_s"] >= sv["started_s"], "a fit service's times")
    stored = [x for x in rows if x["observation"] == STORED]
    need(len(stored) == a.get("exchange_appended"), "the stored stamps differ from those handed to the fitter")
    need(all(x["last_reply_ns"] >= 0 and x["read_ok"] for x in stored), "a stored stamp without a reply")

    # The lock states the published plans went through, and the locks against them. A lock applied on the final
    # close has no executed row: it holds no referenced cycle and the run ended it.
    states = [x["lock_state"] for x in rows]
    need(not states or states[0] == 0, "the first plan is not acquiring")
    for s0, s1 in itertools.pairwise(states):
        need((s0, s1) in {(0, 0), (0, 1), (1, 1), (1, 0), (1, 2), (2, 2), (2, 0)}, f"a lock-state change {s0} to {s1}")
    starts = [rows[i]["cycle"] for i in range(1, len(rows)) if (states[i - 1], states[i]) == (1, 2)]
    locks = a.get("lock", [])
    need(len(locks) == a.get("locks"), "the lock records differ from the summary's locks")
    last = rows[-1]["cycle"] if rows else 0
    executed = [lk for lk in locks if lk["locked_cycle"] < last]
    need([lk["locked_cycle"] + 1 for lk in executed] == starts, "the locks do not start where the rows enter the lattice")
    for lk in locks:
        if lk["locked_cycle"] >= last:
            need(lk["locked_cycle"] == last and lk["referenced_cycles"] == 0 and lk["end"] == "run_end",
                 "a lock applied on the final close with referenced cycles or an end")
            continue
        end = lk.get("ended_cycle", last)
        n = sum(1 for x in rows if lk["locked_cycle"] < x["cycle"] <= end and x["schedule"] == 1)
        need(n == lk["referenced_cycles"], f"the lock at cycle {lk['locked_cycle']}: its referenced cycles")
    ended = [lk for lk in locks if lk["end"] != "run_end"]
    need(len(ended) == a.get("losses") == sum(a.get(k, 0) for k in ("lost_outside_gate", "lost_no_gated_stamp",
                                                                     "lost_period_clamp", "lost_non_finite")),
         "the ended locks differ from the losses")
    held_at_end = bool(locks) and locks[-1]["end"] == "run_end"
    need((a.get("lock_state_at_end") == 2) == held_at_end, "the lock state at the end against the last lock's end")
    need(bool(a.get("fit_pending_at_end")) == (a.get("lock_state_at_end") == 1), "the fit pending at the end")
    need(("coupling" in a) == bool(D), "the coupling record present without the alternation, or absent with it")
    return bad


# ----------------------------------------------------------------------------------------------- carriers

class Carriers:
    """The measured completion lattice and each read's carrier: index_of(t) maps a time (the probe's epoch) to the
    nearest completion's index with its distance; time_of(k) is completion k's instant; carrier(row) gives the
    row's carrier index, or a reason it has none."""

    def index_of(self, t: int) -> tuple[int, int]:
        raise NotImplementedError

    def time_of(self, k: int) -> int | None:
        raise NotImplementedError

    def carrier(self, row: dict) -> tuple[int | None, str]:
        raise NotImplementedError


class TraceCarriers(Carriers):
    """trace_correlate.py --completions-csv: every bulk IN completion of the run, the byte-associated carriers."""

    def __init__(self, path: pathlib.Path, run: Run) -> None:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
        self.ts = [int(c["ts_ns"]) - run.origin for c in rows]  # the probe's epoch
        if [int(c["index"]) for c in rows] != list(range(len(rows))) or self.ts != sorted(self.ts):
            raise SystemExit(f"{path}: the completions are not indexed in time order")
        self.by_cycle: dict[int, tuple[int, bool]] = {}
        for k, c in enumerate(rows):
            if c["last_of_cycle"]:
                self.by_cycle[int(c["last_of_cycle"])] = (k, c["reply_complete"] == "1")

    def index_of(self, t: int) -> tuple[int, int]:
        j = bisect.bisect_left(self.ts, t)
        best = min((k for k in (j - 1, j) if 0 <= k < len(self.ts)), key=lambda k: abs(self.ts[k] - t))
        return best, self.ts[best] - t

    def time_of(self, k: int) -> int | None:
        return self.ts[k] if 0 <= k < len(self.ts) else None

    def carrier(self, row: dict) -> tuple[int | None, str]:
        got = self.by_cycle.get(row["cycle"])
        if got is None:
            return None, "unassociated"
        k, complete = got
        return (k, "") if complete else (None, "unassociated")


class EmulatorCarriers(Carriers):
    """dxl_emu --cadence-log: the completion (index, instant) that carried each held reply; each grouped read is the
    one held reply whose request arrived while the read was open."""

    def __init__(self, path: pathlib.Path, run: Run) -> None:
        with open(path, newline="") as f:
            recs = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(f)]
        reads = [r for r in recs if r["instruction"] in GROUPED_READS]
        self.arrivals = [r["arrival_ns"] - run.origin for r in reads]
        if self.arrivals != sorted(self.arrivals):
            raise SystemExit(f"{path}: the record's requests are not in order")
        self.reads = reads
        pts = sorted({(r["index"], r["completion_ns"] - run.origin) for r in reads if r["index"] >= 0})
        self.idx = [p[0] for p in pts]
        self.t = [p[1] for p in pts]
        if len(pts) < 2:
            raise SystemExit(f"{path}: fewer than two recorded completions")

    def _period(self, j: int) -> float:
        a, b = (j, j + 1) if j + 1 < len(self.t) else (j - 1, j)
        return (self.t[b] - self.t[a]) / (self.idx[b] - self.idx[a])

    def index_of(self, t: int) -> tuple[int, int]:
        j = bisect.bisect_left(self.t, t)
        j = min((k for k in (j - 1, j) if 0 <= k < len(self.t)), key=lambda k: abs(self.t[k] - t))
        k = self.idx[j] + round((t - self.t[j]) / self._period(j))
        return k, self._time(k) - t

    def time_of(self, k: int) -> int | None:
        return self._time(k)

    def _time(self, k: int) -> int:
        j = bisect.bisect_left(self.idx, k)
        if j < len(self.idx) and self.idx[j] == k:
            return self.t[j]
        j = min(max(j - 1, 0), len(self.idx) - 1)  # the recorded completion before k (or the first), and its period
        return round(self.t[j] + (k - self.idx[j]) * self._period(j))

    def carrier(self, row: dict) -> tuple[int | None, str]:
        j0 = bisect.bisect_left(self.arrivals, row["read_start_ns"])
        j1 = bisect.bisect_right(self.arrivals, row["last_reply_ns"])
        if j1 - j0 == 0:
            return None, "unassociated"
        if j1 - j0 > 1:
            return None, "ambiguous"
        k = self.reads[j0]["index"]
        return (k, "") if k >= 0 else (None, "unassociated")


def true_lock(run: Run, carriers: Carriers | None) -> dict:
    """A3: every lattice read of the locked window, classified; the per-lock offsets from the first row's
    prediction alone."""
    window = [x for x in run.rows if run.window_of(x) == 2 and x["schedule"] == 1]
    out: dict = {"denominator": len(window)}
    if carriers is None:
        out["testable"] = "not testable: no carriers (an untraced run)"
        return out
    classes: dict[str, int] = dict.fromkeys(("carried_by_target", "carried_later", "carried_earlier", "incomplete", "unassociated",
                             "ambiguous"), 0)
    diffs: list[int] = []
    offsets: list[dict] = []
    offset: int | str | None = None
    for x in run.rows:
        if x["schedule"] != 1:
            offset = None
            continue
        if offset is None or x["transition"]:
            k0, d0 = carriers.index_of(x["predicted_completion_ns"])
            offset = (k0 - x["target_index"]) if abs(d0) <= QUARTER_PERIOD_NS else "ambiguous"
            offsets.append({"cycle": x["cycle"], "offset": offset, "distance_ns": d0})
        if run.window_of(x) != 2:
            continue
        if offset == "ambiguous":
            classes["ambiguous"] += 1
            continue
        target = x["target_index"] + offset
        tt = carriers.time_of(target)
        if tt is None or abs(tt - x["predicted_completion_ns"]) > QUARTER_PERIOD_NS:
            classes["ambiguous"] += 1
            continue
        if not x["read_ok"] or x["last_reply_ns"] < 0:
            classes["incomplete"] += 1
            continue
        k, why = carriers.carrier(x)
        if k is None:
            classes[why] += 1
            continue
        if k == target:
            classes["carried_by_target"] += 1
            diffs.append(x["predicted_completion_ns"] - tt)
        else:
            classes["carried_later" if k > target else "carried_earlier"] += 1
    out.update(classes=classes, offsets=offsets)
    out["median_prediction_less_carrier_us"] = (timing_summary.nearest_rank(np.array(diffs), 0.5) / 1e3
                                                if diffs else None)
    return out


# ------------------------------------------------------------------------------------------------ analysis

def lock_segments(run: Run, window: int | None = None) -> list[list[dict]]:
    """Each lock's lattice rows (a lock starts at its first lattice row, transition 1, and ends at the next grid row),
    restricted to one window when asked."""
    segs: list[list[dict]] = []
    cur: list[dict] = []
    for x in run.rows:
        if x["schedule"] != 1 or x["transition"]:
            if cur:
                segs.append(cur)
            cur = []
        if x["schedule"] == 1:
            cur.append(x)
    if cur:
        segs.append(cur)
    if window is not None:
        segs = [[x for x in s if run.window_of(x) == window] for s in segs]
    return [s for s in segs if s]


def q(values: Sequence[float], p: float) -> float | None:
    """The nearest-rank p-quantile (timing_summary's), or None for no values."""
    return timing_summary.nearest_rank(np.array(values), p) if len(values) else None


def window_report(run: Run, sel: int | None, span_s: float) -> dict:
    rows = [x for x in run.rows if sel is None or run.window_of(x) == sel]
    c = skip_counts(run, sel)
    lateness = [x["wake_ns"] - x["scheduled_wake_ns"] for x in rows]
    work = [x["cycle_end_ns"] - x["read_start_ns"] for x in rows if x["read_start_ns"] >= 0 and x["cycle_end_ns"] >= 0]
    motion = [x for x in rows if x["read_len"] == MOTION_LEN]
    full = [x for x in rows if x["read_len"] == FULL_LEN]
    run_len = longest = 0
    for x in rows:  # the longest run of adjacent cycles whose closes each skipped
        run_len = run_len + 1 if x["skipped_periods"] > 0 else 0
        longest = max(longest, run_len)
    w = dict(c, span_s=span_s, rate_hz=c["cycles"] / span_s if span_s > 0 else None,
             most_skipped_at_one_close=max((x["skipped_periods"] for x in rows), default=0),
             longest_overrun_run=longest,
             motion_reads=len(motion), motion_no_skip=sum(x["skipped_periods"] == 0 for x in motion),
             full_reads=len(full), full_no_skip=sum(x["skipped_periods"] == 0 for x in full),
             recheck_skipped_rows=sum(x.get("recheck_skipped", 0) > 0 for x in rows),
             recheck_skipped=sum(x.get("recheck_skipped", 0) for x in rows),
             read_failures=sum(not x["read_ok"] for x in rows),
             wake_lateness_us={k: (q(lateness, p) or 0) / 1e3 for k, p in (("p50", .5), ("p99", .99), ("p999", .999))}
             | {"max": max(lateness, default=0) / 1e3},
             host_work_us={k: (q(work, p) or 0) / 1e3 for k, p in (("p50", .5), ("p999", .999))}
             | {"max": max(work, default=0) / 1e3})
    if run.timing == "adapter":
        lattice = [x for x in rows if x["schedule"] == 1 and x["lock_state"] == 2]
        w["observations"] = {name: sum(x["observation"] == code for x in lattice)
                             for name, code in (("gated", GATED), ("late", LATE), ("early", EARLY),
                                                ("invalid", INVALID))}
        for sign, name in ((1, "plus"), (-1, "minus")):
            m = [x for x in lattice if x["read_len"] == MOTION_LEN and x["lead_offset_ns"] * sign > 0]
            w[f"motion_{name}_reads"] = len(m)
            w[f"motion_{name}_no_skip"] = sum(x["skipped_periods"] == 0 for x in m)
    return w


def share(n: int, d: int) -> str:
    return f"{n} of {d} ({100 * n / d:.2f} %)" if d else f"{n} of 0"


def criteria(run: Run, loaded: bool) -> list[tuple]:
    kind, data = timing_summary.load(run.path)
    s = timing_summary.summarize(kind, data, float(run.s["rate_hz"]), None, None, run.s)
    args = argparse.Namespace(period_tolerance=0.001, loaded_skips_per_10k=1.0, lateness_p999=0.10,
                              lateness_max=0.5, host_work_p999=0.70, estimate_p999=0.90)
    return timing_summary.check(s, loaded, args)


def predictions(run: Run, windows: dict, a3: dict, crit: list[tuple], compare: dict | None) -> list[dict]:
    out: list[dict] = []

    def put(name: str, verdict: str, value: str) -> None:
        out.append({"prediction": name, "verdict": verdict, "value": value})

    def held(ok: bool) -> str:
        return "held" if ok else "failed"

    lw, whole = windows["locked"], windows["whole_run"]
    t = {name: v for name, _, v, _ in crit}
    if run.timing == "grid":
        rates = (whole["rate_hz"], lw["rate_hz"])
        ok = all(G_RATE_HZ[0] <= r <= G_RATE_HZ[1] for r in rates) and all(
            t.get(k) == "FAIL" for k in ("T1", "T2", "T6a"))
        value = (f"whole run {rates[0]:.1f} Hz, matched window {rates[1]:.1f} Hz; T1 {t.get('T1')}, T2 {t.get('T2')}, "
                 f"T6a {t.get('T6a')}")
        if compare is not None:
            below = compare["locked"]["rate_hz"] - rates[1]
            ok = ok and below >= G_BELOW_HZ
            value += f"; {below:.1f} Hz below the adapter run's locked window"
        put("G", held(ok), value)
        return out

    a = run.adapter
    fits = a.get("fit", [])
    results = [f for f in fits if f["outcome"] != "requested"]
    first = results[0] if results else None
    locks = a.get("lock", [])
    D = a["lead_alternation_us"]
    end_s = run.duration_ns / 1e9
    lw_start = (run.acq_ns + run.settle_ns) / 1e9
    lost_in_window = [lk for lk in locks if lk["end"] != "run_end" and lw_start <= lk.get("ended_s", 0) < end_s]
    ref = lw["referenced_cycles"]
    ok = (first is not None and first["outcome"] == "applied" and bool(locks)
          and locks[0]["locked_s"] < A1_LOCK_BEFORE_S and not lost_in_window
          and lw["cycles"] > 0 and ref >= A1_REFERENCED_SHARE * lw["cycles"])
    fit_text = f"first fit {first['outcome'] if first else 'none'}"
    if first is not None and first["outcome"] == "applied":
        fit_text += f" (share {first['share']:.3f}, slope {first['slope_ppm']:+.2f} ppm)"
    lock_text = f"first lock at {locks[0]['locked_s']:.3f} s" if locks else "no lock"
    put("A1", held(ok), f"{fit_text}; {lock_text}; {len(lost_in_window)} losses in the window; "
        f"referenced {share(ref, lw['cycles'])}")
    applied = [f for f in fits if f["outcome"] == "applied"]
    lattice = [x for x in run.rows if run.window_of(x) == 2 and x["schedule"] == 1]
    if not applied or not lattice:
        put("A2", "not testable", "no applied fit" if not applied else "no lattice row in the window")
    else:
        slope = applied[0]["slope_ppm"]
        tracked = [(seg[-1]["predicted_completion_ns"] - seg[0]["predicted_completion_ns"])
                   / (seg[-1]["target_index"] - seg[0]["target_index"]) / run.period_ns * 1e6 - 1e6
                   for seg in lock_segments(run, window=2) if seg[-1]["target_index"] != seg[0]["target_index"]]
        ok = bool(tracked) and all(A2_PPM[0] <= v <= A2_PPM[1] for v in (slope, *tracked))
        put("A2", held(ok), f"fitted slope {slope:+.2f} ppm; tracked period over each lock's window rows "
            + (", ".join(f"{v:+.2f}" for v in tracked) or "none") + " ppm")
    if "classes" not in a3:
        put("A3", "not testable", a3["testable"].removeprefix("not testable: "))
    elif a3["denominator"] < A3_MIN:
        put("A3", "insufficient evidence", f"{a3['denominator']} lattice reads (minimum {A3_MIN})")
    else:
        c, m = a3["classes"], a3["median_prediction_less_carrier_us"]
        ok = c["carried_by_target"] >= A3_SHARE * a3["denominator"] and m is not None and abs(m) <= A3_MEDIAN_US
        put("A3", held(ok), f"carried by the target {share(c['carried_by_target'], a3['denominator'])}; "
            + ", ".join(f"{k.replace('_', ' ')} {v}" for k, v in c.items() if k != "carried_by_target")
            + (f"; median prediction less carrier {m:+.1f} us" if m is not None else ""))
    if not D:
        put("A4", "not testable", "no lead alternation")
    else:
        cp = a.get("coupling", {})
        tied = [lk for lk in locks if "request_tied_detected" in (lk.get("coupling_at_end"), lk.get("first_declaration"))]
        ok = cp.get("state_at_end") in ("no_coupling_detected", "insufficient_evidence") and not tied \
            and cp.get("locks_request_tied", 0) == 0
        put("A4", held(ok), f"at the end {cp.get('state_at_end')}; locks declared request-tied {len(tied)}")
    obs = lw.get("observations", {})
    n = sum(obs.values())
    if n < A5_MIN:
        put("A5", "insufficient evidence", f"{n} observations (minimum {A5_MIN})")
    else:
        ok = obs["gated"] >= A5_GATED * n and obs["early"] <= A5_EARLY * n
        put("A5", held(ok), f"gated {share(obs['gated'], n)}, early {share(obs['early'], n)}, late {obs['late']}, "
            f"invalid {obs['invalid']}")
    mo = [x for x in lattice if x["read_len"] == MOTION_LEN]
    mo_ok = sum(x["skipped_periods"] == 0 for x in mo)
    rech = sum(x["recheck_skipped"] > 0 for x in mo)
    if len(mo) < A6_MIN:
        put("A6", "insufficient evidence", f"{len(mo)} motion-only lattice reads (minimum {A6_MIN})")
    else:
        put("A6", held(mo_ok >= A6_SHARE * len(mo)),
            f"no skip {share(mo_ok, len(mo))} of the window's motion-only lattice reads; of them with publication "
            f"recheck skips {rech}; grid cycles in the window {lw['grid_cycles']}")
    half = (run.acq_ns + run.settle_ns + run.duration_ns) / 2
    h1 = sum(1 for x in run.rows if run.window_of(x) == 2 and x["scheduled_wake_ns"] < half)
    h2 = lw["cycles"] - h1
    span_half = (run.duration_ns - run.acq_ns - run.settle_ns) / 2e9
    r1, r2 = h1 / span_half, h2 / span_half
    ok = lw["rate_hz"] is not None and lw["rate_hz"] >= A7_RATE_HZ and abs(r1 - r2) <= A7_HALVES_HZ
    put("A7", held(ok), f"window {lw['rate_hz']:.1f} Hz ({lw['cycles']} cycles over {lw['span_s']:g} s); "
        f"halves {r1:.1f} and {r2:.1f} Hz")
    fr = [x for x in lattice if x["read_len"] == FULL_LEN]
    fr_ok = sum(x["skipped_periods"] == 0 for x in fr)
    if len(fr) < A8_MIN:
        put("A8", "insufficient evidence", f"{len(fr)} full-window lattice reads (minimum {A8_MIN})")
    else:
        worst = max(x["skipped_periods"] for x in fr)
        ok = A8_SHARE[0] * len(fr) <= fr_ok <= A8_SHARE[1] * len(fr) and worst <= A8_MAX_SKIP
        put("A8", held(ok), f"no skip {share(fr_ok, len(fr))}; the most periods one skipped {worst}")
    if not D:
        put("A9", "not testable", "no lead alternation")
    else:
        parts, ok, enough = [], True, True
        for name, sign in (("+", 1), ("-", -1)):
            m = [x for x in mo if x["lead_offset_ns"] * sign > 0]
            k = sum(x["skipped_periods"] == 0 for x in m)
            parts.append(f"{name}{D:g} us: no skip {share(k, len(m))}")
            enough = enough and len(m) >= A9_MIN
            ok = ok and k >= A9_SHARE * len(m)
        put("A9", held(ok) if enough else "insufficient evidence",
            "; ".join(parts) + ("" if enough else f" (minimum {A9_MIN} per sign)"))
    work = [x["cycle_end_ns"] - x["read_start_ns"] for x in run.rows if run.window_of(x) == 2]
    if len(work) < A10_MIN:
        put("A10", "insufficient evidence", f"{len(work)} cycles (minimum {A10_MIN})")
    else:
        p999 = (q(work, 0.999) or 0.0) / 1e3
        put("A10", held(A10_US[0] <= p999 <= A10_US[1]), f"host work p99.9 {p999:.1f} us over {len(work)} cycles")
    ok = W_RATE_HZ[0] <= whole["rate_hz"] <= W_RATE_HZ[1] and t.get("T2") == "FAIL"
    put("W", held(ok), f"whole run {whole['rate_hz']:.1f} Hz ({whole['cycles']} cycles over {whole['span_s']:g} s); "
        f"T2 {t.get('T2')}")
    return out


def analyze(run: Run, carriers: Carriers | None, loaded: bool, compare: Run | None) -> dict:
    spans = run.spans_s()
    windows = {name: window_report(run, sel, spans[name]) for name, sel in WINDOWS}
    a3 = true_lock(run, carriers) if run.timing == "adapter" else {}
    crit = criteria(run, loaded)
    cmp = None
    if compare is not None:
        cs = compare.spans_s()
        cmp = {"locked": window_report(compare, 2, cs["locked"])}
    return {"log": str(run.path), "timing": run.timing, "windows": windows, "true_lock": a3,
            "criteria": [{"criterion": n, "class": c, "verdict": v, "detail": d} for n, c, v, d in crit],
            "predictions": predictions(run, windows, a3, crit, cmp)}


def print_report(rep: dict) -> None:
    print(f"{rep['log']}: timing {rep['timing']}")
    print("whole run, the criteria as timing_summary.py computes them (thresholds unchanged):")
    for c in rep["criteria"]:
        print(f"  {c['criterion']:<4} {c['class']:<9} {c['verdict']:<13} {c['detail']}")
    print("windows by scheduled wake (descriptive; a window's rate is its cycles over its span):")
    for name, w in rep["windows"].items():
        line = (f"  {name:<11} [{w['span_s']:g} s] cycles {w['cycles']}, {w['rate_hz']:.1f} Hz, skipped "
                f"{w['skipped_periods']} ({w['overruns']} overruns, {w['consecutive_skips']} consecutive, at most "
                f"{w['most_skipped_at_one_close']} at one close, overruns in a row at most {w['longest_overrun_run']}), "
                f"recheck skips {w['recheck_skipped']} in "
                f"{w['recheck_skipped_rows']} rows; motion-only no skip {share(w['motion_no_skip'], w['motion_reads'])}, "
                f"full-window no skip {share(w['full_no_skip'], w['full_reads'])}; read failures {w['read_failures']}")
        if rep["timing"] == "adapter":
            line += f"; referenced {w['referenced_cycles']}, grid {w['grid_cycles']}"
        print(line)
        lat, work = w["wake_lateness_us"], w["host_work_us"]
        print(f"              wake lateness p50/p99/p99.9/max {lat['p50']:.1f}/{lat['p99']:.1f}/{lat['p999']:.1f}/"
              f"{lat['max']:.1f} us; host work p50/p99.9/max {work['p50']:.1f}/{work['p999']:.1f}/{work['max']:.1f} us"
              + (f"; observations {w['observations']}" if "observations" in w else ""))
    if rep["true_lock"].get("offsets"):
        print("true lock: per-lock offsets (target_index to the measured index, from each lock's first prediction): "
              + ", ".join(f"cycle {o['cycle']}: {o['offset']} ({o['distance_ns'] / 1e3:+.1f} us)"
                          for o in rep["true_lock"]["offsets"]))
    print("exploratory predictions (prospective hypotheses, not criteria or validity gates):")
    for p in rep["predictions"]:
        print(f"  {p['prediction']:<4} {p['verdict']:<22} {p['value']}")


def main() -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate")
    v.add_argument("log", type=pathlib.Path)
    v.add_argument("--plan", choices=("grid", "adapter", "full-grid"),
                   help="also check the plan's settings for this arm (full-grid: the full-feedback diagnostic's run)")
    v.add_argument("--hardware", action="store_true",
                   help="with --plan: the adapter's latency timer must have been read, and measured at 1 ms")
    a = sub.add_parser("analyze")
    a.add_argument("log", type=pathlib.Path)
    src = a.add_mutually_exclusive_group()
    src.add_argument("--completions", type=pathlib.Path, help="trace_correlate.py --completions-csv (a timer run)")
    src.add_argument("--emulator-record", type=pathlib.Path, help="dxl_emu --cadence-log (a PTY rehearsal)")
    a.add_argument("--loaded", action="store_true", help="the run was loaded (T2's allowance, as timing_summary's)")
    a.add_argument("--compare-adapter", type=pathlib.Path, help="a grid run's same-mode adapter run (prediction G)")
    a.add_argument("--json", type=pathlib.Path, help="also write the analysis as JSON")
    args = ap.parse_args()
    try:
        run = Run(args.log)
    except (OSError, KeyError, ValueError, tomllib.TOMLDecodeError) as e:
        print(f"adapter_diagnostic: {args.log}: {e}", file=sys.stderr)
        return 2
    bad = validate(run)
    if args.cmd == "validate" and args.plan:
        bad += check_plan(run, args.plan, args.hardware)
    if args.cmd == "validate":
        if bad:
            print(f"{args.log}: NOT valid ({len(bad)} facts failed):")
            for b in bad:
                print(f"  {b}")
            return 1
        print(f"{args.log}: valid ({run.timing} timing, {len(run.rows)} cycles)")
        return 0
    if bad:
        print(f"adapter_diagnostic: {args.log} is not valid ({bad[0]}): validate it first", file=sys.stderr)
        return 1
    carriers: Carriers | None = None
    if args.completions:
        carriers = TraceCarriers(args.completions, run)
    elif args.emulator_record:
        carriers = EmulatorCarriers(args.emulator_record, run)
    compare = Run(args.compare_adapter) if args.compare_adapter else None
    if compare is not None and (compare.timing != "adapter" or run.timing != "grid" or validate(compare)):
        print("adapter_diagnostic: --compare-adapter needs a grid LOG and a valid adapter run", file=sys.stderr)
        return 1
    rep = analyze(run, carriers, args.loaded, compare)
    print_report(rep)
    if args.json:
        args.json.write_text(json.dumps(rep, indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
