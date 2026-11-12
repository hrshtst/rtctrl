#!/usr/bin/env bash
# x7_model_torque end to end: the output header and row count, a
# zero-rate row's gravity equal to its inverse dynamics, no-clobber, CLI
# errors before any model work, and invalid input writing no output.
# Works in a run directory of its own under ROOT, removed after a pass and
# kept after a failure; nothing else is ever deleted.
set -eu

TOOL="$1"
MODEL="$2"
ROOT="$3"

mkdir -p "$ROOT"
WORK=$(mktemp -d "$ROOT/run.XXXXXX")
report_kept() {
  local status=$?
  if [ "$status" -ne 0 ]; then
    echo "model torque smoke: files kept in $WORK" >&2
  fi
}
trap report_kept EXIT

# expect EXIT NAME [ARGS...]: run the tool, capture NAME.out and NAME.err
expect() {
  local want=$1 name=$2
  shift 2
  set +e
  "$TOOL" "$@" >"$WORK/$name.out" 2>"$WORK/$name.err"
  local got=$?
  set -e
  if [ "$got" -ne "$want" ]; then
    echo "$name: exit $got, expected $want" >&2
    cat "$WORK/$name.out" "$WORK/$name.err" >&2
    exit 1
  fi
}

# Inputs in the x7_follow log layout (text and NaN columns around the
# references). Row 0 rests (zero rates and accelerations), rows 1 and 2
# move; bad.csv has a NaN reference in data row 2, empty.csv no data rows.
python3 - "$WORK" <<'PY'
import os
import sys

root = sys.argv[1]
columns = ["schema_version", "time_s", "phase"]
for j in range(8):
    columns += [f"qref{j}_rad", f"dqref{j}_rad_s", f"ddqref{j}_rad_s2",
                f"q{j}_rad", f"flags{j}"]


def row(index, q, dq, ddq):
    cells = ["1", f"{0.01 * index:.9f}", "tracking"]
    for j in range(8):
        cells += [repr(q[j]), repr(dq[j]), repr(ddq[j]), "nan", "0"]
    return cells


q = [0.3, -0.6, 0.2, -1.1, 0.4, 0.7, -0.5, 0.2]
rows = [
    row(0, q, [0.0] * 8, [0.0] * 8),
    row(1, q, [0.4 - 0.09 * j for j in range(8)],
        [-0.7 + 0.15 * j for j in range(8)]),
    row(2, [v + 0.1 for v in q], [0.2] * 8, [0.5] * 8),
]


def write(name, data):
    with open(os.path.join(root, name), "w") as stream:
        for cells in [columns] + data:
            stream.write(",".join(cells) + "\n")


write("in.csv", rows)
write("long.csv", [row(i, q, [0.0] * 8, [0.0] * 8) for i in range(400)])
bad = [list(cells) for cells in rows]
bad[1][columns.index("dqref3_rad_s")] = "nan"
write("bad.csv", bad)
write("empty.csv", [])
PY

# good input: one summary line, the table
expect 0 good --model "$MODEL" --input "$WORK/in.csv" --output "$WORK/out.csv"
test "$(cat "$WORK/good.out")" = "model torque: 3 rows"
python3 - "$WORK/out.csv" <<'PY'
import csv
import math
import sys

with open(sys.argv[1], newline="") as stream:
    reader = csv.reader(stream)
    header = next(reader)
    rows = list(reader)
expected = ["schema_version", "row"]
expected += [f"gravity{j}_nm" for j in range(8)]
expected += [f"inverse_dynamics{j}_nm" for j in range(8)]
assert header == expected, header
assert len(rows) == 3, len(rows)
for index, cells in enumerate(rows):
    assert len(cells) == len(header), cells
    assert cells[0] == "1" and cells[1] == str(index), cells[:2]
    assert all(math.isfinite(float(value)) for value in cells[2:]), cells
gravity = slice(2, 10)
dynamics = slice(10, 18)
# the resting row: inverse dynamics is the gravity torque, as printed
assert rows[0][gravity] == rows[0][dynamics], rows[0]
assert max(abs(float(value)) for value in rows[0][gravity]) > 0.1, rows[0]
# the moving rows: rates and accelerations change the torque
for cells in rows[1:]:
    assert cells[gravity] != cells[dynamics], cells
# rows 0 and 1 share the posture, so the gravity torque
assert rows[1][gravity] == rows[0][gravity]
PY

# no-clobber: an existing output is refused and left untouched
BEFORE=$(sha256sum "$WORK/out.csv")
expect 1 clobber --model "$MODEL" --input "$WORK/in.csv" \
  --output "$WORK/out.csv"
grep -q "refusing to overwrite" "$WORK/clobber.err"
test "$(sha256sum "$WORK/out.csv")" = "$BEFORE"
test ! -s "$WORK/clobber.out"

# invalid input: the row and column named, no output
expect 1 bad --model "$MODEL" --input "$WORK/bad.csv" \
  --output "$WORK/bad_out.csv"
grep -q "data row 2 (line 3): column 'dqref3_rad_s': not a finite number" \
  "$WORK/bad.err"
test ! -e "$WORK/bad_out.csv"
expect 1 empty --model "$MODEL" --input "$WORK/empty.csv" \
  --output "$WORK/empty_out.csv"
grep -q "no data rows" "$WORK/empty.err"
test ! -e "$WORK/empty_out.csv"

# a failing write: a file-size cap makes the kernel refuse the output
# (EFBIG; SIGXFSZ ignored) after it was created; the partial file is
# removed
set +e
(
  ulimit -f 8
  trap '' XFSZ
  exec "$TOOL" --model "$MODEL" --input "$WORK/long.csv" \
    --output "$WORK/capped.csv"
) >"$WORK/capped.out" 2>"$WORK/capped.err"
CAPPED_RC=$?
set -e
if [ "$CAPPED_RC" -eq 0 ]; then
  echo "capped: expected a failing exit" >&2
  cat "$WORK/capped.out" "$WORK/capped.err" >&2
  exit 1
fi
grep -q "output: cannot write" "$WORK/capped.err"
test ! -e "$WORK/capped.csv"
test ! -s "$WORK/capped.out"

# a model that fails to load: no output
expect 1 nomodel --model "$WORK/no_such.ztk" --input "$WORK/in.csv" \
  --output "$WORK/nomodel_out.csv"
grep -q "failed to load" "$WORK/nomodel.err"
test ! -e "$WORK/nomodel_out.csv"

# CLI errors reject before any model or input work (nothing here exists)
expect 1 unknown --model "$WORK/no_such.ztk" --input "$WORK/no_such.csv" \
  --output "$WORK/unknown_out.csv" --port x
grep -q "unknown option: --port" "$WORK/unknown.err"
test "$(wc -l <"$WORK/unknown.err")" -eq 1
test ! -e "$WORK/unknown_out.csv"
expect 1 nooutput --model "$WORK/no_such.ztk" --input "$WORK/no_such.csv"
grep -q -- "--output is required" "$WORK/nooutput.err"
expect 0 help --help
grep -q "usage: x7_model_torque --model ZTK --input IN.csv --output OUT.csv" \
  "$WORK/help.out"

rm -rf "$WORK"
echo "model torque smoke OK"
