"""Validates an x7_float --log CSV: the event-semantics header, the
column contract (10 shared + 7 per joint), a plausible row count for a
2 s float at 100 Hz, full numeric/Boolean well-formedness of EVERY
declared column, acceptance on every submission, applied telemetry
becoming and staying valid after startup, the STRICT pre-submission
sequencing invariant (the snapshot is read before this cycle's write,
so its applied record must be strictly older than this cycle's own
submission), and the internal consistency of the signed
feedback-to-apply offset."""
import csv
import sys

DOF = 8
SHARED = ("t", "feedback_time", "feedback_seq", "submitted_seq",
          "submission_time", "accepted", "applied_valid", "applied_seq",
          "latest_apply_time", "feedback_minus_latest_apply")
PER_JOINT = ("q", "dqservo", "tau_meas", "tau_request", "tau_applied",
             "clamped", "gated")
BOOLEAN = {"accepted", "applied_valid"} | {
    f"{c}{i}" for i in range(DOF) for c in ("clamped", "gated")}
INTEGER = {"feedback_seq", "submitted_seq", "applied_seq"}

lines = open(sys.argv[1]).read().splitlines()
assert lines[0].startswith("# events per row:"), "semantics header missing"
rows = list(csv.DictReader(lines[1:]))

expected = list(SHARED) + [f"{c}{i}" for i in range(DOF) for c in PER_JOINT]
assert list(rows[0].keys()) == expected, "column contract violated"
assert len(rows) >= 100, f"only {len(rows)} rows for a 2 s float"

STARTUP_ROWS = 2  # applied telemetry may lag activation briefly
valid_seen = 0
for k, r in enumerate(rows):
    # every declared column parses as its declared kind
    for col in expected:
        if col in BOOLEAN:
            assert r[col] in ("0", "1"), (k, col, r[col])
        elif col in INTEGER:
            int(r[col])
        else:
            float(r[col])
    assert r["accepted"] == "1", "unaccepted submission in a clean run"
    if k >= STARTUP_ROWS:
        # a log whose applied telemetry never materializes must FAIL
        assert r["applied_valid"] == "1", (
            f"row {k}: applied telemetry still invalid after startup")
    if r["applied_valid"] == "1":
        valid_seen += 1
        # pre-submission snapshot: STRICTLY older than this submission
        assert int(r["applied_seq"]) < int(r["submitted_seq"]), (
            f"row {k}: applied record not strictly older than its own "
            "cycle's submission")
        # the signed offset must be self-consistent with its parts
        recomputed = float(r["feedback_time"]) - float(
            r["latest_apply_time"])
        assert abs(recomputed - float(r["feedback_minus_latest_apply"])) \
            < 1e-5, f"row {k}: offset inconsistent with its parts"

assert valid_seen >= len(rows) - STARTUP_ROWS, "too few valid applied rows"
print("float log ok: %d rows, %d columns, %d applied-valid" %
      (len(rows), len(expected), valid_seen))
