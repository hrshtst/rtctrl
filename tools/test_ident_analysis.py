#!/usr/bin/env python3
"""Dataset-integrity regressions for ident_analysis.py, run under
ctest.

Pins the reviewer-required rules: a frozen-integrator run and a
live-integrator run must never merge (the effective mode is part of
the tuning record); a sidecar claiming a frozen integral without the
actual 8-element bias vector is invalid; a retried dwell contributes
only its final completed attempt; and mode fitting refuses on
insufficient usable data instead of fitting noise.
"""

import argparse
import importlib.util
import sys

import numpy as np

spec = importlib.util.spec_from_file_location("ia", "tools/ident_analysis.py")
assert spec is not None and spec.loader is not None
ia = importlib.util.module_from_spec(spec)
sys.modules["ia"] = ia
spec.loader.exec_module(ia)

frozen = {
    "kp": 6.0, "kd": 1.0, "ki": 6.0, "i_clamp_nm": 1.5,
    "pd_tau_s": 0.05, "nominal_dt_s": 0.01,
    "gain_scale": [0.5, 1.0, 0.7, 0.7, 0.1, 0.3, 0.2, 0.2],
    "integral_frozen_at_capture": 1,
}
live = dict(frozen)
live["integral_frozen_at_capture"] = 0

# frozen vs live integrator = different controllers: never one dataset
assert not ia.same_tuning(frozen, live), \
    "frozen and live-integrator runs must not merge"
assert ia.same_tuning(frozen, dict(frozen))
assert ia.same_tuning(live, dict(live))

# a different identification scale is likewise a different controller
rescaled = dict(frozen)
rescaled["gain_scale"] = [1.0] + frozen["gain_scale"][1:]
assert not ia.same_tuning(frozen, rescaled)

# a frozen claim is only valid with the actual 8-element bias vector
assert not ia.valid_frozen_record(frozen, None)
assert not ia.valid_frozen_record(frozen, [0.0] * 7)
assert not ia.valid_frozen_record(frozen, "not-a-list")
assert ia.valid_frozen_record(frozen, [0.0] * 8)
assert ia.valid_frozen_record(live, None)   # live: no vector expected
assert ia.valid_frozen_record(None, None)   # legacy: handled upstream

print("ident_analysis merge-guard regressions: ok")

# policy versioning: the retired all-at-admission global freeze
# (policy 1) and legacy sidecars without the field must never merge
# with per-joint-latch (policy 2) datasets
per_joint = dict(frozen)
per_joint["integral_policy"] = 2
legacy_global = dict(frozen)
legacy_global["integral_policy"] = 1
legacy_no_field = dict(frozen)  # pre-policy sidecar: field absent
assert not ia.same_tuning(per_joint, legacy_global), \
    "global-freeze and per-joint-latch runs must not merge"
assert not ia.same_tuning(per_joint, legacy_no_field), \
    "pre-policy legacy sidecars must not merge with versioned ones"
assert ia.same_tuning(per_joint, dict(per_joint))

print("integral-policy versioning regressions: ok")

# ---------------------------------------------------------------------------
# Retry handling: a retried dwell logs BOTH attempts under one
# dwell_id — only the final completed attempt is a valid window
# (review finding: the analyzer merged a soft-aborted 2 Hz attempt
# with its retry and reported a 13.1 s window for a 5.02 s
# measurement).

# legacy CSV (no dwell_attempt column): split on the re-settle gap
t1 = np.arange(0.0, 1.0, 0.01)          # aborted first attempt
t2 = np.arange(4.0, 9.0, 0.01)          # accepted retry window
legacy = np.zeros(t1.size + t2.size, dtype=[("feedback_time", float)])
legacy["feedback_time"] = np.concatenate([t1, t2])
kept = ia.final_attempt_rows(legacy)
assert kept.size == t2.size, "legacy fallback must keep the last segment"
assert kept["feedback_time"][0] == t2[0]

# with the dwell_attempt column, the marker decides
marked = np.zeros(t1.size + t2.size,
                  dtype=[("feedback_time", float),
                         ("dwell_attempt", float)])
marked["feedback_time"] = np.concatenate([t1, t2])
marked["dwell_attempt"] = np.concatenate(
    [np.zeros(t1.size), np.ones(t2.size)])
kept = ia.final_attempt_rows(marked)
assert kept.size == t2.size, "dwell_attempt must select the retry"
assert np.all(kept["dwell_attempt"] == 1)

# an unretried dwell passes through unchanged
kept = ia.final_attempt_rows(legacy[t1.size:])
assert kept.size == t2.size

print("final-attempt selection regressions: ok")

# ---------------------------------------------------------------------------
# Mode-fit admission gate: fewer than five confident+observable
# dwells is a REFUSAL, never a fall-back to fitting all points
# (review finding: the 2026-07-27 null survey — probe joint
# tick-frozen, |resp| ~ 1e-16 rad — produced 5.04/12.06 Hz "modes"
# fitted from numerical noise).


def make_dwell(freq, resp_rad, low_confidence=False):
    z_q = np.zeros(ia.DOF, dtype=complex)
    z_q[1] = resp_rad
    return ia.Dwell(
        source="t.csv", order=0, freq_hz=freq, amp_nm=0.15,
        n_rows=300, window_s=3.0, z_q=z_q, z_tau_meas=0.15 + 0j,
        z_cmd=0.15 + 0j, delay_s=0.01, probe_joint=1,
        fit_residual_rms=0.0, clipped_cycles=0,
        low_confidence=low_confidence)


# all low-confidence (the null survey shape): refuse
null_survey = [make_dwell(f, 1e-16, low_confidence=True)
               for f in (2, 3, 4, 5, 6, 8, 10, 13, 16, 20)]
ia.apply_observability_floor(null_survey, ia.OBS_FLOOR_RAD)
usable, reasons = ia.fit_gate(null_survey)
assert not usable and reasons, "null survey must refuse mode fitting"

# confident but tick-frozen (below one encoder count): still refuse
frozen_resp = [make_dwell(f, 1e-16) for f in (2, 3, 4, 5, 6, 8)]
ia.apply_observability_floor(frozen_resp, ia.OBS_FLOOR_RAD)
usable, reasons = ia.fit_gate(frozen_resp)
assert not usable and reasons, \
    "sub-floor responses must not count as usable"
assert all(d.below_floor for d in frozen_resp)

# five confident, observable dwells: fitting proceeds
good = [make_dwell(f, 0.005) for f in (3, 4, 4.5, 5, 6)]
ia.apply_observability_floor(good, ia.OBS_FLOOR_RAD)
usable, reasons = ia.fit_gate(good)
assert len(usable) == 5 and not reasons

print("mode-fit admission gate regressions: ok")

# ---------------------------------------------------------------------------
# Floor validation (review finding): --obs-floor-rad accepted NaN,
# infinity, zero, and negatives — a NaN bypasses every `resp < floor`
# comparison, admitting tick-frozen dwells as usable. Both the CLI
# parser and the gate itself must reject unusable floors.

for bad in ("nan", "inf", "-inf", "0", "0.0", "-1e-3"):
    try:
        ia.parse_obs_floor(bad)
        raise AssertionError(f"parse_obs_floor must reject {bad!r}")
    except argparse.ArgumentTypeError:
        pass
try:
    ia.parse_obs_floor("not-a-number")
    raise AssertionError("parse_obs_floor must reject non-numbers")
except argparse.ArgumentTypeError:
    pass
assert ia.parse_obs_floor("3.6e-5") == 3.6e-5

# defense in depth: the gate refuses directly too — the reviewer's
# reproduction (five tick-frozen dwells admitted under a NaN floor)
# must be impossible at every layer
for bad_floor in (float("nan"), float("inf"), 0.0, -1e-3):
    frozen_again = [make_dwell(f, 1e-16) for f in (2, 3, 4, 5, 6)]
    try:
        ia.apply_observability_floor(frozen_again, bad_floor)
        raise AssertionError(
            f"apply_observability_floor must reject {bad_floor}")
    except ValueError:
        pass

print("observability-floor validation regressions: ok")
