"""Validates an x7_float --log CSV: the event-semantics and scale
headers, the column contract, numeric/Boolean well-formedness with
finiteness for every declared column, acceptance on every submission,
applied telemetry becoming and staying valid after startup, the STRICT
pre-submission sequencing invariant, the signed feedback-to-apply
offset's self-consistency, and the release-marker column (monotone
0 -> 1).

Two column contracts are admitted: v1 (11 shared + 7 per joint — the
pre-instrumentation format of every archived 2026-07-30 session) and
v2 (v1 plus operator_event and per-joint goal_cnt/present_cnt — the
reviewer-directed notch-correlation instrumentation; every log the
current binary writes). On v2 the added invariants are enforced:
operator_event STRICTLY after the release-marker row (the transition
row itself is rejected), and both raw-count columns consistent with
their SI torque columns through the nominal torque constant and the
2.69 mA LSB (half a count plus print-rounding tolerance).

With --gate-free (docs/records/history.md gravity calibration, disposition revision
2026-07-31): any gate event on ANY axis fails validation — the next
instrumented j1 notch-correlation session is VOID on one, because a
cross-axis gate discontinuity (j3 zeroing ~1 Nm during the archived
j1 session) is a plausible confounder for the felt notch.

With --vendor (docs/records/history.md gravity calibration M-GC1/M-GC2), the current
10-second acceptance protocol is required. With --demo, a non-default
5..50-second demonstration protocol is required and can never validate
as acceptance. Both additionally require the vendor-equivalent scales,
a released transition, marker-anchored termination at the declared
evaluation_window_s, and tau_applied == scale * tau_request on every
applied row. The archived five-second acceptance protocol remains valid
only through --legacy --vendor."""
import csv
import hashlib
import math
import sys

DOF = 8
SHARED = ("t", "feedback_time", "feedback_seq", "submitted_seq",
          "submission_time", "accepted", "applied_valid", "applied_seq",
          "latest_apply_time", "feedback_minus_latest_apply", "released")
PER_JOINT = ("q", "dqservo", "tau_meas", "tau_request", "tau_applied",
             "clamped", "gated")
SHARED_V2 = SHARED + ("operator_event",)
PER_JOINT_V2 = PER_JOINT + ("goal_cnt", "present_cnt")
BOOLEAN = {"accepted", "applied_valid", "released"} | {
    f"{c}{i}" for i in range(DOF) for c in ("clamped", "gated")}
INTEGER = {"feedback_seq", "submitted_seq", "applied_seq"}

# raw-count consistency (v2): nominal torque constants and the
# goal/present-current LSB mirror dxl/conversions.hpp; tolerance is
# half a count (lround's boundary) plus the %.4f torque print rounding
KT_NOMINAL = [1.783, 2.409] + [1.783] * 6
CURRENT_LSB_A = 0.00269
CNT_TOL = 0.52

VENDOR_SCALES = [0.810455] * DOF
VENDOR_SCALES[1] = 0.669167

# The ONLY files --legacy admits, bound by SHA-256 with a fixed role
# (review finding: an unbound --legacy let the archived back-drive log
# validate as acceptance evidence, and admitted arbitrary headerless
# files).
LEGACY_EVIDENCE = {
    # float2.csv — the M-GC3 acceptance run: legacy acceptance-vendor
    "de1f1a3b333b26534b5617d5494a9c194e6e9c4555e32fcbeb81c7b02141fbf1":
        "acceptance-vendor",
    # float3.csv — the back-drive session: NEVER acceptance evidence
    "f09057f289066be6d60a9154c97f71ba141e7daccd8b993670ba0df3da7726fd":
        "backdrive",
}

args = sys.argv[1:]
vendor = "--vendor" in args
demo = "--demo" in args
feel = "--feel" in args
legacy = "--legacy" in args  # pre-run_mode archived evidence ONLY
gate_free = "--gate-free" in args  # instrumented j1 sessions: VOID on gates
assert sum((vendor, demo, feel)) <= 1, (
    "--vendor, --demo and --feel are mutually exclusive")
assert not (legacy and demo), "--legacy never validates as a demonstration"
path = next(a for a in args if not a.startswith("--"))

with open(path, "rb") as f:
    raw = f.read()
lines = raw.decode().splitlines()
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
    # a header-bearing log is NEVER legacy — --legacy must not weaken
    # classification for any new log (review finding)
    assert not legacy, "--legacy rejected: this log carries run_mode"
    run_mode = mode_lines[0].split(":", 1)[1].strip()
    # Feel-check and demonstration logs must never pass as acceptance.
    expected_mode = (
        "feel-check" if feel else ("demonstration" if demo else "acceptance"))
    assert run_mode == expected_mode, run_mode
else:
    # --legacy admits ONLY the pre-run_mode archived evidence
    # (float2/float3, 2026-07-30); new logs always carry the header,
    # so classification is never weakened for them.
    assert legacy, "run_mode header missing (use --legacy only for " \
                   "pre-run_mode archived evidence)"
    role = LEGACY_EVIDENCE.get(hashlib.sha256(raw).hexdigest())
    assert role is not None, (
        "--legacy admits ONLY the archived float2/float3 evidence "
        "(unknown SHA-256)")
    assert not feel, "--legacy never validates as a feel log"
    if role == "acceptance-vendor":
        assert vendor, "float2 must be validated with --vendor"
    else:
        assert not vendor, (
            "the archived back-drive log must NEVER validate as "
            "acceptance evidence")
dur_lines = [c for c in comments if c.startswith("# duration_s:")]
requested_s = float(dur_lines[0].split(":", 1)[1]) if dur_lines else None
window_lines = [
    c for c in comments if c.startswith("# evaluation_window_s:")]
evaluation_window_s = (
    float(window_lines[0].split(":", 1)[1]) if window_lines else None)
if not legacy:
    assert requested_s is not None, "duration_s header missing"
    if feel:
        assert evaluation_window_s is None, (
            "feel-check log must not declare an evaluation window")
    else:
        assert evaluation_window_s is not None, (
            "evaluation_window_s header missing")
        assert 5.0 <= evaluation_window_s <= 50.0, (
            f"evaluation window out of range: {evaluation_window_s}")
        if demo:
            assert evaluation_window_s != 10.0, (
                "demonstration log uses the acceptance window")
        else:
            assert evaluation_window_s == 10.0, (
                "acceptance log uses a non-default evaluation window")
        assert requested_s >= 8.0 + evaluation_window_s + 2.0, (
            "outer duration does not cover marker deadline + evaluation "
            "window + margin")

rows = list(csv.DictReader(lines))
expected_v1 = list(SHARED) + [
    f"{c}{i}" for i in range(DOF) for c in PER_JOINT]
expected_v2 = list(SHARED_V2) + [
    f"{c}{i}" for i in range(DOF) for c in PER_JOINT_V2]
header_row = list(rows[0].keys())
v2 = header_row == expected_v2
assert v2 or header_row == expected_v1, "column contract violated"
expected = expected_v2 if v2 else expected_v1
booleans = BOOLEAN | ({"operator_event"} if v2 else set())
integers = INTEGER | (
    {f"{c}{i}" for i in range(DOF) for c in ("goal_cnt", "present_cnt")}
    if v2 else set())
assert len(rows) >= 100, f"only {len(rows)} rows"

STARTUP_ROWS = 2  # applied telemetry may lag activation briefly
valid_seen = 0
prev_released = 0
t_marker = None
event_rows = 0
for k, r in enumerate(rows):
    for col in expected:
        if col in booleans:
            assert r[col] in ("0", "1"), (k, col, r[col])
        elif col in integers:
            int(r[col])
        else:
            assert math.isfinite(float(r[col])), (
                f"row {k}: non-finite {col} = {r[col]}")
    assert r["accepted"] == "1", "unaccepted submission in a clean run"
    if gate_free:
        for i in range(DOF):
            assert r[f"gated{i}"] == "0", (
                f"row {k}: gate event on axis {i} — a gate-free session "
                "was required, this attempt is VOID")
    released = int(r["released"])
    assert released >= prev_released, f"row {k}: released went backward"
    if released == 1 and t_marker is None:
        t_marker = float(r["t"])
    prev_released = released
    if v2:
        # the marker protocol orders every operator event STRICTLY
        # after the release marker — the release-transition row itself
        # is not admissible (review finding: released == 1 alone let
        # a transition-row event pass); the raw counts must be the SI
        # torque columns re-expressed through the nominal constant
        # and the LSB
        if r["operator_event"] == "1":
            assert t_marker is not None and float(r["t"]) > t_marker, (
                f"row {k}: operator_event not strictly after the "
                "release marker")
            event_rows += 1
        for i in range(DOF):
            per_count = KT_NOMINAL[i] * CURRENT_LSB_A
            present = float(r[f"tau_meas{i}"]) / per_count
            assert abs(present - int(r[f"present_cnt{i}"])) <= CNT_TOL, (
                f"row {k}: present_cnt{i} inconsistent with tau_meas")
            if r["applied_valid"] == "1":
                goal = float(r[f"tau_applied{i}"]) / per_count
                assert abs(goal - int(r[f"goal_cnt{i}"])) <= CNT_TOL, (
                    f"row {k}: goal_cnt{i} inconsistent with tau_applied")
            else:
                assert r[f"goal_cnt{i}"] == "0", (
                    f"row {k}: goal_cnt{i} nonzero before applied_valid")
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

if vendor or demo:
    for i, s in enumerate(scales):
        assert abs(s - VENDOR_SCALES[i]) < 1e-9, (
            f"scale j{i} = {s}, expected {VENDOR_SCALES[i]}")
    assert t_marker is not None, "no release marker recorded"
    t_end = float(rows[-1]["t"])
    expected_window = 5.0 if legacy else evaluation_window_s
    assert expected_window - 0.2 <= t_end - t_marker \
        <= expected_window + 0.3, (
        f"run did not terminate ~{expected_window:g} s after the marker "
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

event_summary = f", {event_rows} operator events" if v2 else ""
mode_summary = (
    ", vendor-scale verified"
    if vendor
    else (
        ", demonstration verified"
        if demo
        else (", feel-check mode" if feel else "")
    )
)
print(
    f"float log ok: {len(rows)} rows, {len(expected)} columns, "
    f"{valid_seen} applied-valid{event_summary}{mode_summary}"
)
