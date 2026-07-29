"""Validates the EFL smoke run's results.json (review finding: the
stdout regex alone would pass on malformed JSON or on a --case run
wrongly marked canonical). Asserts well-formedness, the explicit
noncanonical flag with reasons, the schema version, and the
plant/pose evidence fields on every result row."""
import json
import sys

path = sys.argv[1]
d = json.load(open(path))

assert d["schema_version"] == "2", d["schema_version"]
assert d["canonical"] is False, "--case output must be noncanonical"
assert d["noncanonical_reasons"], "noncanonical must carry reasons"

rows = d["grid"] + d["cases"]
assert rows, "no result rows"
for row in rows:
    for key in ("controller", "start_pose", "plant_params",
                "completed"):
        assert key in row, (row.get("case", "grid"), key)
    assert len(row["start_pose"]) == 8
    assert "type" in row["plant_params"]
for case in d["cases"]:
    for key in ("case", "tau_max_nm", "velocity_source", "gains"):
        assert key in case, (case["case"], key)

print("smoke json ok: %d rows validated" % len(rows))
