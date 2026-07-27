#!/usr/bin/env python3
"""Merge-guard regressions for ident_analysis.py, run under ctest.

Pins the dataset-integrity rules the reviewer required: a frozen-
integrator run and a live-integrator run must never merge (the
effective mode is part of the tuning record), and a sidecar claiming a
frozen integral without the actual 8-element bias vector is invalid.
"""

import importlib.util
import sys

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
