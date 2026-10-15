#!/usr/bin/env bash
# x7_force_budget end to end: the conditional verdict and its exit codes,
# the CSV preamble and sample count, failure statuses, no-clobber, and
# invalid input writing no CSV.
set -eu

PLAN="$1"
BUDGET="$2"
PTP_CONFIG="$3"
MODEL="$4"
ROOT="$5"

rm -rf "$ROOT"
mkdir -p "$ROOT"
MODEL=$(realpath "$MODEL")

"$PLAN" --config "$PTP_CONFIG" --output "$ROOT/ptp.zvs" --no-diagnostics \
  >"$ROOT/plan.log" 2>&1

write_spec() {  # name magnitude link direction_line
  cat >"$ROOT/spec_$1.toml" <<EOF
format = "rtctrl-x7-link-force"
version = 1
link = "$3"
point_link_m = [0.0, 0.0, 0.0]
magnitude_n = $2
$4

[profile]
onset_s = 0.4
rise_s = 0.2
hold_s = 0.6
fall_s = 0.2
EOF
}

write_config() {  # name spec allowance
  cat >"$ROOT/$1.toml" <<EOF
format = "rtctrl-x7-force-budget"
version = 1
model = "$MODEL"
reference = "ptp.zvs"
link_force = "spec_$2.toml"

[budget]
joint_allowance_nm = [$3]
sample_rate_hz = 200.0
EOF
}

# expect EXIT NAME [ARGS...]: run with the config NAME.toml, capture stdout
expect() {
  local want=$1 name=$2
  shift 2
  set +e
  "$BUDGET" --config "$ROOT/$name.toml" "$@" >"$ROOT/$name.out" 2>"$ROOT/$name.err"
  local got=$?
  set -e
  if [ "$got" -ne "$want" ]; then
    echo "$name: exit $got, expected $want" >&2
    cat "$ROOT/$name.out" "$ROOT/$name.err" >&2
    exit 1
  fi
}

TCP="crane_x7_tcp_link"
write_spec push 3.0 "$TCP" 'direction_world = [0.0, 1.0, 0.0]'
write_spec heavy 1000.0 "$TCP" 'direction_world = [0.0, 1.0, 0.0]'
write_spec zero 0.0 "$TCP" 'direction_world = [0.0, 1.0, 0.0]'
write_spec pinned 3.0 world 'anchor_world_m = [0.0, 0.0, 0.0]'
write_spec unknown 3.0 no_such_link 'direction_world = [0.0, 1.0, 0.0]'
write_config within push 2.0
write_config exceeds heavy 2.0
write_config zero zero 2.0
write_config infeasible push 1e-4
write_config degenerate pinned 2.0
write_config invalid unknown 2.0

PHRASE="within the supplied sampled model allowances"

# within: exit 0, the contract first, the verdict phrase, a CSV
expect 0 within --output "$ROOT/within.csv"
head -n 1 "$ROOT/within.out" | grep -q "contract: compute the largest force amplitude"
grep -q "not a hardware safety or fuse rating" "$ROOT/within.out"
grep -q "verdict: $PHRASE" "$ROOT/within.out"
grep -q "^headroom: [0-9.]* x" "$ROOT/within.out"
SAMPLES=$(sed -n 's/^samples: \([0-9]*\) over \([0-9.]*\) s.*/\1/p' "$ROOT/within.out")
DURATION=$(sed -n 's/^samples: \([0-9]*\) over \([0-9.]*\) s.*/\2/p' "$ROOT/within.out")
python3 - "$ROOT/within.csv" "$SAMPLES" "$DURATION" <<'PY'
import csv
import math
import sys

path, samples, duration = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
with open(path) as stream:
    lines = stream.read().splitlines()
preamble = [line for line in lines if line.startswith("# ")]
assert lines[: len(preamble)] == preamble, "preamble must come first"
text = "\n".join(preamble)
assert "# contract: compute the largest force amplitude" in text
assert "not a hardware safety or fuse rating" in text
assert "# status: ok" in text
rows = list(csv.DictReader(lines[len(preamble):]))
# N + 1 evenly spaced samples, endpoints included, spacing <= 1 / rate
# (N = ceil(duration * rate) on the exact double, which the printed
# duration only approximates)
assert len(rows) == samples, (len(rows), samples)
assert abs(len(rows) - 1 - duration * 200.0) <= 1.0 + 1e-6, len(rows)
times = [float(row["time_s"]) for row in rows]
assert times[0] == 0.0
assert abs(times[-1] - duration) < 1e-6
steps = [b - a for a, b in zip(times, times[1:])]
assert max(steps) <= 1.0 / 200.0 + 1e-9 and min(steps) > 0.0
for row in rows:
    assert row["status"] == "ok"
    assert row["schema_version"] == "1"
    for key, value in row.items():
        # status is text, an unbounded joint reads inf, and the anchor
        # distance is NaN in world-direction mode by definition
        if key in ("status", "anchor_distance_m") or key.startswith(
            "joint_bound"
        ):
            continue
        assert math.isfinite(float(value)), (key, value)
PY

# no-clobber: an existing CSV is refused and left untouched
BEFORE=$(sha256sum "$ROOT/within.csv")
expect 1 within --output "$ROOT/within.csv"
grep -q "refusing to overwrite" "$ROOT/within.err"
test "$(sha256sum "$ROOT/within.csv")" = "$BEFORE"

# exceeds: exit 1, no verdict phrase, CSV still written
expect 1 exceeds --output "$ROOT/exceeds.csv"
grep -q "verdict: EXCEEDS the supplied sampled model allowances" "$ROOT/exceeds.out"
test "$(grep -c "verdict: $PHRASE" "$ROOT/exceeds.out" || true)" = 0
test -f "$ROOT/exceeds.csv"

# zero magnitude: within, headroom undefined
expect 0 zero --output "$ROOT/zero.csv"
grep -q "headroom: n/a (planned magnitude is zero)" "$ROOT/zero.out"
grep -q "verdict: $PHRASE" "$ROOT/zero.out"

# infeasible: a gravity load above its allowance
expect 1 infeasible --output "$ROOT/infeasible.csv"
grep -q "verdict: INFEASIBLE at t=" "$ROOT/infeasible.out"
grep -q "headroom: n/a (evaluation failed)" "$ROOT/infeasible.out"
test "$(grep -c "verdict: $PHRASE" "$ROOT/infeasible.out" || true)" = 0
grep -q "# status: infeasible" "$ROOT/infeasible.csv"

# degenerate anchor: an anchor on the application point
expect 1 degenerate --output "$ROOT/degenerate.csv"
grep -q "verdict: EVALUATION FAILED (degenerate anchor)" "$ROOT/degenerate.out"
grep -q "headroom: n/a (evaluation failed)" "$ROOT/degenerate.out"
grep -q "minimum anchor distance: 0 m" "$ROOT/degenerate.out"

# invalid input: no CSV
expect 1 invalid --output "$ROOT/invalid.csv"
grep -q "unknown link 'no_such_link'" "$ROOT/invalid.err"
test ! -e "$ROOT/invalid.csv"

# --check touches nothing
expect 0 within --check --output "$ROOT/check.csv"
grep -q "configuration preflight passed" "$ROOT/within.out"
test ! -e "$ROOT/check.csv"

echo "force budget smoke OK"
