"""Validates an x7_float --log CSV: the event-semantics and scale
headers, the column contract (11 shared + 7 per joint), numeric/
Boolean well-formedness with finiteness for every declared column,
acceptance on every submission, applied telemetry becoming and staying
valid after startup, the STRICT pre-submission sequencing invariant,
the signed feedback-to-apply offset's self-consistency, and the
release-marker column (monotone 0 -> 1).

With --vendor (GRAVITY_CALIBRATION_PLAN M-GC1/M-GC2): additionally
requires the vendor-equivalent scales in the header, a released
transition, marker-anchored termination (last row ~5 s after the
marker), and tau_applied == scale * tau_request on every applied row —
the end-to-end proof that the calibration applies exactly once through
the RealArm command path."""
import csv
import math
import sys

DOF = 8
SHARED = ("t", "feedback_time", "feedback_seq", "submitted_seq",
          "submission_time", "accepted", "applied_valid", "applied_seq",
          "latest_apply_time", "feedback_minus_latest_apply", "released")
PER_JOINT = ("q", "dqservo", "tau_meas", "tau_request", "tau_applied",
             "clamped", "gated")
BOOLEAN = {"accepted", "applied_valid", "released"} | {
    f"{c}{i}" for i in range(DOF) for c in ("clamped", "gated")}
INTEGER = {"feedback_seq", "submitted_seq", "applied_seq"}

VENDOR_SCALES = [0.810455] * DOF
VENDOR_SCALES[1] = 0.669167

args = sys.argv[1:]
vendor = "--vendor" in args
feel = "--feel" in args
legacy = "--legacy" in args  # pre-run_mode archived evidence ONLY
path = [a for a in args if not a.startswith("--")][0]

lines = open(path).read().splitlines()
comments = []
while lines and lines[0].startswith("#"):
    comments.append(lines.pop(0))
assert any(c.startswith("# events per row:") for c in comments), \
    "semantics header missing"
scale_lines = [c for c in comments
               if c.startswith("# command_torque_scale:")]
assert scale_lines, "scale header missing"
scales = [float(v) for v in scale_lines[0].split(":", 1)[1].split()]
assert len(scales) == DOF
mode_lines = [c for c in comments if c.startswith("# run_mode:")]
if mode_lines:
    run_mode = mode_lines[0].split(":", 1)[1].strip()
    # a feel-check log must never pass as acceptance evidence
    assert run_mode == ("feel-check" if feel else "acceptance"), run_mode
else:
    # --legacy admits ONLY the pre-run_mode archived evidence
    # (float2/float3, 2026-07-30); new logs always carry the header,
    # so classification is never weakened for them.
    assert legacy, "run_mode header missing (use --legacy only for "                    "pre-run_mode archived evidence)"
dur_lines = [c for c in comments if c.startswith("# duration_s:")]
requested_s = float(dur_lines[0].split(":", 1)[1]) if dur_lines else None
if not legacy:
    assert requested_s is not None, "duration_s header missing"

rows = list(csv.DictReader(lines))
expected = list(SHARED) + [f"{c}{i}" for i in range(DOF) for c in PER_JOINT]
assert list(rows[0].keys()) == expected, "column contract violated"
assert len(rows) >= 100, f"only {len(rows)} rows"

STARTUP_ROWS = 2  # applied telemetry may lag activation briefly
valid_seen = 0
prev_released = 0
t_marker = None
for k, r in enumerate(rows):
    for col in expected:
        if col in BOOLEAN:
            assert r[col] in ("0", "1"), (k, col, r[col])
        elif col in INTEGER:
            int(r[col])
        else:
            assert math.isfinite(float(r[col])), (
                f"row {k}: non-finite {col} = {r[col]}")
    assert r["accepted"] == "1", "unaccepted submission in a clean run"
    released = int(r["released"])
    assert released >= prev_released, f"row {k}: released went backward"
    if released == 1 and t_marker is None:
        t_marker = float(r["t"])
    prev_released = released
    if k >= STARTUP_ROWS:
        assert r["applied_valid"] == "1", (
            f"row {k}: applied telemetry still invalid after startup")
    if r["applied_valid"] == "1":
        valid_seen += 1
        assert int(r["applied_seq"]) < int(r["submitted_seq"]), (
            f"row {k}: applied record not strictly older than its own "
            "cycle's submission")
        recomputed = float(r["feedback_time"]) - float(
            r["latest_apply_time"])
        assert abs(recomputed - float(r["feedback_minus_latest_apply"])) \
            < 1e-5, f"row {k}: offset inconsistent with its parts"

assert valid_seen >= len(rows) - STARTUP_ROWS, "too few valid applied rows"

if vendor:
    for i, s in enumerate(scales):
        assert abs(s - VENDOR_SCALES[i]) < 1e-9, (
            f"scale j{i} = {s}, expected {VENDOR_SCALES[i]}")
    assert t_marker is not None, "no release marker recorded"
    t_end = float(rows[-1]["t"])
    assert 4.8 <= t_end - t_marker <= 5.3, (
        f"run did not terminate ~5 s after the marker "
        f"(marker {t_marker}, end {t_end})")
    # the calibration applies EXACTLY once: applied torque (reconverted
    # through the nominal constant) equals scale * requested torque.
    # The applied record lags a cycle; gravity varies slowly at hold,
    # so a small margin covers it plus the current LSB.
    checked = 0
    for r in rows:
        if r["applied_valid"] != "1":
            continue
        for i in range(DOF):
            tau_req = float(r[f"tau_request{i}"])
            if abs(tau_req) < 0.05:
                continue
            tau_app = float(r[f"tau_applied{i}"])
            assert abs(tau_app - scales[i] * tau_req) < 0.02, (
                f"j{i}: tau_applied {tau_app} != {scales[i]} * "
                f"{tau_req}")
            checked += 1
    assert checked > 100, "too few scaled-torque comparisons"
elif feel:
    # feel mode is M-GC3-specific: the vendor scales are mandatory
    # (the all-1.0 default is the known-failed configuration)
    for i, s in enumerate(scales):
        assert abs(s - VENDOR_SCALES[i]) < 1e-9, (
            f"feel run without vendor scales (j{i} = {s})")
    assert t_marker is not None, "feel-check run without a marker"
    t_end = float(rows[-1]["t"])
    # the session must have reached its REQUESTED deadline — a
    # truncated session must not pass (review finding)
    assert requested_s is not None
    assert t_end >= requested_s - 0.5, (
        f"feel session ended at {t_end} s, requested {requested_s} s")
else:
    # phase-1 smoke expectation only — legacy archived evidence may
    # legitimately carry a marker (float3's back-drive session)
    assert legacy or t_marker is None, \
        "unexpected release marker in phase 1"

print("float log ok: %d rows, %d columns, %d applied-valid%s" %
      (len(rows), len(expected), valid_seen,
       ", vendor-scale verified" if vendor
       else (", feel-check mode" if feel else "")))
