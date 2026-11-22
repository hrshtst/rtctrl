"""An x7_bus_timing adapter log against its summary, row by row.

Usage: check_adapter_log.py LOG GRID_HEADER DEADLINE_S MIN_FITS. Used by
bus_timing_adapter_smoke.sh and bus_timing_cadence_smoke.sh; it was the
former's inline check, moved here (e40875f) and since changed only in
style (ruff's import order, a dict literal, itertools.pairwise).
"""
import csv
import itertools
import sys

import tomllib

log, grid_header, deadline_s = sys.argv[1], sys.argv[2].split(","), float(sys.argv[3])
min_fits = int(sys.argv[4])  # fits the main thread must have completed (its requests come early enough)
with open(log + ".summary.toml", "rb") as f:
    s = tomllib.load(f)
with open(log, newline="") as f:
    reader = csv.DictReader(f)
    header = reader.fieldnames
    rows = [{k: int(v) for k, v in r.items()} for r in reader]
adapter_cols = ["target_index", "schedule", "lock_state", "transition", "predicted_completion_ns",
                "lead_offset_ns", "coupling_state", "observation", "recheck_skipped"]
assert header == grid_header + adapter_cols, header
r, a = s["result"], s["adapter"]
assert s["timing"] == "adapter", s["timing"]
assert s["log_complete"] and r["ring_drops"] == 0 and r["torque_off_after"]
assert len(rows) == r["cycles"], (len(rows), r["cycles"])
assert a["protocol_errors"] == 0
assert a["fitter"] == "main_thread" and a["stamp"] == "last_reply_ns"
assert a["publication_margin_us"] == 20 and a["fit_max_age_s"] == 0.5
L, W, D = a["lead_us"] * 1000, a["wake_to_read_us"] * 1000, a["lead_alternation_us"] * 1000
acq, settle = a["acquisition_s"] * 1e9, a["settling_s"] * 1e9

# The declared windows and the whole run, recomputed from the rows.
def counts(sel):
    c = {"cycles": 0, "skipped_periods": 0, "overruns": 0, "consecutive_skips": 0, "referenced_cycles": 0,
         "grid_cycles": 0}
    prev, prev_w = False, None
    for row in rows:
        t = row["scheduled_wake_ns"]
        w = 0 if t < acq else 1 if t < acq + settle else 2
        over = row["skipped_periods"] > 0
        if sel is None or w == sel:
            c["cycles"] += 1
            c["skipped_periods"] += row["skipped_periods"]
            c["overruns"] += over
            c["consecutive_skips"] += row["skipped_periods"] >= 2
            c["consecutive_skips"] += over and prev and (sel is None or prev_w == w)
            c["referenced_cycles" if row["schedule"] == 1 else "grid_cycles"] += 1
        prev, prev_w = over, w
    return c
for name, sel in (("acquisition", 0), ("settling", 1), ("locked", 2), ("whole_run", None)):
    assert counts(sel) == a[name], (name, counts(sel), a[name])
assert a["whole_run"]["skipped_periods"] == r["skipped_periods"] == sum(x["skipped_periods"] for x in rows)

# Each row: the plan its read executed (on the grid, the epoch's whole periods;
# on the lattice, its own wake from its predicted completion, one rounding),
# its close's skips with the recheck's among them, full-window reads at the
# base lead, motion-only reads alternating from + over each lock's reads.
expect, alternations, lattice = 1, 0, 0
for row in rows:
    assert row["recheck_skipped"] <= row["skipped_periods"], row
    if row["schedule"] == 0:
        assert row["scheduled_wake_ns"] % 1000000 == 0, row
        assert row["target_index"] == -1 and row["predicted_completion_ns"] == -1 and row["lead_offset_ns"] == 0
        expect = 1
        continue
    lattice += 1
    want = row["predicted_completion_ns"] - (L + row["lead_offset_ns"] + W)
    assert abs(row["scheduled_wake_ns"] - want) <= 1, row
    if row["read_len"] == 13:
        assert row["lead_offset_ns"] == 0, row
    elif D:
        assert row["lead_offset_ns"] == expect * D, row
        expect, alternations = -expect, alternations + 1
assert all(x["scheduled_wake_ns"] < deadline_s * 1e9 for x in rows)

# Each lattice row's observation, from its own stamp (last_reply_ns, from a
# complete read) against the completion its plan predicted: invalid without
# one, gated within the gate, else late or early (the prediction is rounded to
# the ns, so a row within 1 ns of the gate's edge is not judged).
D_REPLY, GATE = a["reply_delay_us"] * 1000, a["gate_us"] * 1000
judged = 0
for row in rows:
    if row["schedule"] != 1 or row["lock_state"] != 2:
        continue
    if not row["read_ok"] or row["last_reply_ns"] < 0:
        assert row["observation"] == 6, row
        continue
    e = row["last_reply_ns"] - row["predicted_completion_ns"] - D_REPLY
    if abs(abs(e) - GATE) <= 1:
        continue
    want = 3 if abs(e) <= GATE else 4 if e > 0 else 5
    if row["observation"] != 6:  # an out-of-order stamp is invalid
        assert row["observation"] == want, (row, e)
        judged += 1

# The fits: every request accounted for once; the main thread's services.
fits = s["adapter"].get("fit", [])
outcomes = [f["outcome"] for f in fits]
assert outcomes.count("requested") == a["fits_requested"]
for key, name in (("fits_applied", "applied"), ("fits_refused", "refused"), ("stale_results", "stale"),
                  ("abandoned_requests", "abandoned"), ("old_generation_results", "old_generation"),
                  ("queue_failures", "queue_failed")):
    assert outcomes.count(name) == a[key], (key, outcomes)
pending = 1 if a["fit_pending_at_end"] else 0
assert a["fits_requested"] == (a["fits_applied"] + a["fits_refused"] + a["stale_results"]
                               + a["abandoned_requests"] + pending), a
for f in fits:
    if f["outcome"] in ("applied", "refused"):
        assert 0 <= f["age_s"] <= 0.5 and f["request_to_result_s"] >= 0, f
    if f["outcome"] == "stale":
        assert f["age_s"] > 0.5, f
services = s["adapter"].get("fit_service", [])
assert len(services) == a["fits_completed"] >= min_fits, (len(services), a["fits_completed"], min_fits)
for sv in services:  # started_s is the call's start, which may precede its claim of the request
    assert sv["finished_s"] >= sv["window_end_s"] and sv["finished_s"] >= sv["started_s"], sv
# The acquisition's stamps: each stored row's last_reply_ns went to the fit.
stored = sum(1 for x in rows if x["observation"] == 1)
assert stored == a["exchange_appended"], (stored, a["exchange_appended"])
assert all(x["last_reply_ns"] >= 0 and x["read_ok"] for x in rows if x["observation"] == 1)
# A lock state the published plans went through, in order.
states = [x["lock_state"] for x in rows]
assert states[0] == 0
for p, q in itertools.pairwise(states):
    assert (p, q) in {(0, 0), (0, 1), (1, 1), (1, 0), (1, 2), (2, 2), (2, 0)}, (p, q)
assert a["locks"] == sum(1 for p, q in itertools.pairwise(states) if (p, q) == (1, 2))
coupling = s["adapter"].get("coupling")
assert (coupling is not None) == bool(D)
print(f"{log}: {len(rows)} cycles, {r['skipped_periods']} skipped; fits {outcomes}; "
      f"locks {a['locks']}, losses {a['losses']}, {lattice} lattice cycles ({judged} observations checked), "
      f"{alternations} alternated reads; "
      f"coupling {coupling['state_at_end'] if coupling else 'off'}")
