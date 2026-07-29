"""Validates an x7_float --log CSV: the event-semantics header, the
column contract (8 shared + 7 per joint), a plausible row count for a
2 s float at 100 Hz, numeric well-formedness, acceptance on every
submission, and the requested-versus-applied sequencing (the applied
record visible at a cycle's feedback can never be newer than that
cycle's own submission)."""
import csv
import sys

DOF = 8
SHARED = ("t", "feedback_time", "feedback_seq", "submitted_seq",
          "submission_time", "accepted", "applied_valid", "applied_seq")
PER_JOINT = ("q", "dqservo", "tau_meas", "tau_request", "tau_applied",
             "clamped", "gated")

lines = open(sys.argv[1]).read().splitlines()
assert lines[0].startswith("# events per row:"), "semantics header missing"
rows = list(csv.DictReader(lines[1:]))

expected = list(SHARED) + [f"{c}{i}" for i in range(DOF) for c in PER_JOINT]
assert list(rows[0].keys()) == expected, "column contract violated"
assert len(rows) >= 100, f"only {len(rows)} rows for a 2 s float"

for r in rows:
    float(r["t"])
    for i in range(DOF):
        float(r[f"q{i}"])
        float(r[f"tau_request{i}"])
        float(r[f"tau_applied{i}"])
    assert r["accepted"] == "1", "unaccepted submission in a clean run"
    if r["applied_valid"] == "1":
        assert int(r["applied_seq"]) <= int(r["submitted_seq"]), (
            "applied record newer than this cycle's own submission")

print("float log ok: %d rows, %d columns" % (len(rows), len(expected)))
