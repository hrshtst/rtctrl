#!/usr/bin/env bash
# Link-force disturbance through a simulation frontend, end to end:
# bundle contents and provenance, the seq-keyed partial join between the
# main log and the per-attempt disturbance log, release followed by a
# force-free recovery, a byte-identical prefix against an undisturbed
# run, bundle replay, a run that fails before tracking, zero magnitude,
# and no disturbance output without a configured force.
set -eu

FRONTEND="$1"  # follow or track
APP="$2"
PLAN_APP="$3"
PLAN_CONFIG="$4"
MODEL="$5"
HARDWARE="$6"
EXAMPLE_SPEC="$7"
ROOT="$8"

rm -rf "$ROOT"
mkdir -p "$ROOT"
ROOT=$(realpath "$ROOT")
MODEL=$(realpath "$MODEL")
HARDWARE=$(realpath "$HARDWARE")

"$PLAN_APP" --config "$PLAN_CONFIG" --output "$ROOT/reference.zvs" \
  --no-diagnostics >"$ROOT/plan.log" 2>&1
cp "$EXAMPLE_SPEC" "$ROOT/spec_push.toml"
sed 's/^magnitude_n = .*/magnitude_n = 0.0/' "$EXAMPLE_SPEC" \
  >"$ROOT/spec_zero.toml"
grep -q '^magnitude_n = 0.0$' "$ROOT/spec_zero.toml"

# write_config NAME LINK_FORCE_LINE HOME_STRICT HOME_TOLERANCE
write_config() {
  case "$FRONTEND" in
    follow)
      cat >"$ROOT/$1.toml" <<EOF
format = "rtctrl-x7-follow"
version = 1
model = "$MODEL"
reference = "$ROOT/reference.zvs"
hardware_config = "$HARDWARE"

[control]
rate_hz = 100
mode = "position"

[home]
profile = "minimum-jerk"
velocity_limit = 4.0
strict = $3
tolerance_rad = $4
correction_retries = 0
settle_time_s = 0.02

[simulation]
initial_posture = "zeros"
integration_step_s = 0.0001
$2

[safety]
warning_error_rad = 10.0
sustained_abort_error_rad = 20.0
sustained_abort_time_s = 0.2
immediate_abort_error_rad = 30.0

[finalization]
simulation_hold_time_s = 1.5
EOF
      ;;
    track)
      # playback_rate 0.5: the profile clock is elapsed tracking time
      cat >"$ROOT/$1.toml" <<EOF
format = "rtctrl-x7-track"
version = 1
model = "$MODEL"
reference = "$ROOT/reference.zvs"
hardware_config = "$HARDWARE"

[control]
rate_hz = 100
kp = 20
kd = 2
playback_rate = 0.5

[home]
velocity_limit = 4.0
strict = $3
tolerance_rad = $4
settle_time_s = 0.02

[simulation]
initial_posture = "zeros"
integration_step_s = 0.0001
$2

[safety]
warning_error_rad = 10.0
sustained_abort_error_rad = 20.0
sustained_abort_time_s = 0.2
immediate_abort_error_rad = 30.0

[assessment]
rms_error_rad = 1.0
peak_error_rad = 2.0

[finalization]
simulation_hold_time_s = 1.5
EOF
      ;;
    *)
      echo "unknown frontend $FRONTEND" >&2
      exit 2
      ;;
  esac
}

write_config force 'link_force = "spec_push.toml"' false 0.02
write_config plain '' false 0.02
write_config zero 'link_force = "spec_zero.toml"' false 0.02
write_config early 'link_force = "spec_push.toml"' true 1e-9

run() {  # EXPECTED_EXIT NAME ARGS...
  local want=$1 name=$2
  shift 2
  set +e
  "$APP" "$@" >"$ROOT/$name.log" 2>&1
  local got=$?
  set -e
  if [ "$got" -ne "$want" ]; then
    echo "$name: exit $got, expected $want" >&2
    cat "$ROOT/$name.log" >&2
    exit 1
  fi
}

run 0 force --config "$ROOT/force.toml" --bundle "$ROOT/force"
run 0 plain --config "$ROOT/plain.toml" --bundle "$ROOT/plain"
B="$ROOT/force"
CONFIG_NAME="$FRONTEND.toml"

for path in link_force.toml simulation_disturbance.csv simulation.csv \
    simulation.zvs result.toml manifest.toml; do
  test -f "$B/$path"
done
cmp "$EXAMPLE_SPEC" "$B/link_force.toml"
grep -q '^link_force = "link_force.toml"$' "$B/$CONFIG_NAME"
for path in link_force.toml simulation_disturbance.csv; do
  grep -q "$(sha256sum "$B/$path" | cut -d ' ' -f 1)" "$B/manifest.toml"
done
grep -q '^link_force_enabled = true$' "$B/result.toml"
grep -q '^link_force_started = true$' "$B/result.toml"
grep -q '^link_force_applied = true$' "$B/result.toml"
grep -q '^link_force_first_applied_s = ' "$B/result.toml"
grep -q "link force: started, applied" "$ROOT/force.log"

# no configured force: no disturbance output, no provenance
test ! -e "$ROOT/plain/simulation_disturbance.csv"
test ! -e "$ROOT/plain/link_force.toml"
if grep -q link_force "$ROOT/plain/result.toml"; then exit 1; fi

python3 - "$B" "$ROOT/plain" "$EXAMPLE_SPEC" "$FRONTEND" <<'PY'
import csv
import sys
import tomllib

force, plain, spec_path, frontend = sys.argv[1:5]
# the main log's feedback-time column
time_key = "feedback_time_s" if frontend == "follow" else "time_s"
with open(spec_path, "rb") as stream:
    spec = tomllib.load(stream)
profile = spec["profile"]
end_s = (profile["onset_s"] + profile["rise_s"] + profile["hold_s"]
         + profile["fall_s"])

with open(f"{force}/simulation.csv") as stream:
    main_lines = stream.read().splitlines()
with open(f"{plain}/simulation.csv") as stream:
    plain_lines = stream.read().splitlines()
main = list(csv.DictReader(main_lines))
with open(f"{force}/simulation_disturbance.csv") as stream:
    rows = list(csv.DictReader(stream))

seqs = [int(r["feedback_seq"]) for r in main]
assert [int(r["attempt"]) for r in rows] == list(range(len(rows)))
assert all(r["complete"] == "1" and r["status"] == "ok" for r in rows)
# a successful run records every cycle before stepping it
assert len(rows) == len(main), (len(rows), len(main))

# partial join by seq, never by timestamp
assert all(int(r["end_seq"]) != seqs[0] for r in rows)  # initial: none
for r in rows[:-1]:
    assert seqs.count(int(r["end_seq"])) == 1
assert seqs.count(int(rows[-1]["end_seq"])) == 0  # terminal: no later row
for s in seqs:
    assert sum(int(r["start_seq"]) == s for r in rows) == 1

# the profile clock starts at the first tracking state
first_tracking = next(r for r in main if r["phase"] == "tracking")
t0 = float(first_tracking[time_key])
s0 = int(first_tracking["feedback_seq"])
before = next(r for r in rows if int(r["end_seq"]) == s0)
after = next(r for r in rows if int(r["start_seq"]) == s0)
assert before["started"] == "0" and float(before["impulse_y_ns"]) == 0.0
assert after["started"] == "1"

# release, then a force-free recovery interval of at least 1 s
release = t0 + end_s
assert float(rows[-1]["end_s"]) >= release + 1.0
recovery = [r for r in rows if float(r["start_s"]) >= release - 1e-9]
assert len(recovery) >= 100
for r in recovery:
    assert r["active"] == "0", r
    assert all(float(r[f"impulse_{a}_ns"]) == 0.0 for a in "xyz")
    assert all(float(r[f"joint_impulse{i}_nms"]) == 0.0 for i in range(8))

# identical to the undisturbed run until the first forced step
first_active = next(r for r in rows if r["active"] == "1")
cut = int(first_active["start_seq"])
assert len(plain_lines) == len(main_lines)
for mine, theirs, row in zip(main_lines[1:], plain_lines[1:], main):
    if int(row["feedback_seq"]) > cut:
        break
    assert mine == theirs, row["feedback_seq"]
assert any(
    mine != theirs
    for mine, theirs, row in zip(main_lines[1:], plain_lines[1:], main)
    if int(row["feedback_seq"]) > cut
)
PY

# replay from the bundle reproduces all three outputs
run 0 replay --config "$B/$CONFIG_NAME" --log "$ROOT/replay.csv" \
  --motion "$ROOT/replay.zvs"
cmp "$B/simulation.csv" "$ROOT/replay.csv"
cmp "$B/simulation.zvs" "$ROOT/replay.zvs"
cmp "$B/simulation_disturbance.csv" "$ROOT/replay_disturbance.csv"
# the derived disturbance log is refused like the others, before any run
touch "$ROOT/fresh_disturbance.csv"
run 1 clobber --config "$B/$CONFIG_NAME" --log "$ROOT/fresh.csv" \
  --motion "$ROOT/fresh.zvs"
grep -q "simulation disturbance log already exists" "$ROOT/clobber.log"
test ! -s "$ROOT/fresh_disturbance.csv"

# a configured run that fails before tracking: enabled, never started
run 1 early --config "$ROOT/early.toml" --bundle "$ROOT/early"
grep -q '^link_force_enabled = true$' "$ROOT/early/result.toml"
grep -q '^link_force_started = false$' "$ROOT/early/result.toml"
grep -q '^link_force_applied = false$' "$ROOT/early/result.toml"
if grep -q '^link_force_first_applied_s' "$ROOT/early/result.toml"; then
  exit 1
fi

# zero magnitude: started, never applied, no anchor evaluation, success
run 0 zero --config "$ROOT/zero.toml" --bundle "$ROOT/zero"
grep -q '^link_force_started = true$' "$ROOT/zero/result.toml"
grep -q '^link_force_applied = false$' "$ROOT/zero/result.toml"

if [ "$FRONTEND" = track ]; then
  # an assessment abort records its cycle without stepping it: that main
  # row has no following interval, and every interval still joins
  sed -e 's/^warning_error_rad = .*/warning_error_rad = 1e-8/' \
    -e 's/^sustained_abort_error_rad = .*/sustained_abort_error_rad = 1e-7/' \
    -e 's/^immediate_abort_error_rad = .*/immediate_abort_error_rad = 1e-6/' \
    "$ROOT/force.toml" >"$ROOT/abort.toml"
  run 1 abort --config "$ROOT/abort.toml" --log "$ROOT/abort.csv" \
    --motion "$ROOT/abort.zvs"
  python3 - "$ROOT/abort.csv" "$ROOT/abort_disturbance.csv" <<'PY'
import csv
import sys

with open(sys.argv[1]) as stream:
    main = list(csv.DictReader(stream))
with open(sys.argv[2]) as stream:
    rows = list(csv.DictReader(stream))
seqs = [int(r["feedback_seq"]) for r in main]
last = main[-1]
assert last["phase"] == "tracking", last["phase"]
assert all(int(r["start_seq"]) != seqs[-1] for r in rows)
assert len(rows) == len(main) - 1
for r in rows:
    assert r["complete"] == "1"
    assert seqs.count(int(r["end_seq"])) == 1
PY
fi

echo "link force smoke OK ($FRONTEND)"
