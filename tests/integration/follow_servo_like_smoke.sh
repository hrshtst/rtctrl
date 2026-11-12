#!/usr/bin/env bash
# x7_follow_sim with config/follow_sim_servo_like.toml on the PTP example
# reference, against the ideal-servo example configuration: both succeed;
# the servo-like log shows a tracking error of the measured arm's order
# and of the spring law's structure, far above the ideal servo's; its
# commands reach the servo one cycle later and its positions sit on the
# encoder grid. A bundle records the servo-like keys (the example's, and
# friction, a longer delay and no quantization in a variant), and its
# follow.toml replayed through x7_follow_sim reproduces the bundle's log
# and motion byte for byte; a bundle of the ideal servo has none of them.
# Works in a run directory of its own under ROOT, removed after a pass and
# kept after a failure; nothing else is ever deleted.
set -eu

FOLLOW_APP="$1"
PLAN_APP="$2"
PLAN_CONFIG="$3"
SERVO_CONFIG="$4"
IDEAL_CONFIG="$5"
ROOT="$6"

mkdir -p "$ROOT"
WORK=$(mktemp -d "$ROOT/run.XXXXXX")
report_kept() {
  local status=$?
  if [ "$status" -ne 0 ]; then
    echo "servo-like smoke: files kept in $WORK" >&2
  fi
}
trap report_kept EXIT

# expect EXIT NAME [ARGS...]: run the simulator, capture NAME.out and NAME.err
expect() {
  local want=$1 name=$2
  shift 2
  set +e
  "$FOLLOW_APP" "$@" >"$WORK/$name.out" 2>"$WORK/$name.err"
  local got=$?
  set -e
  if [ "$got" -ne "$want" ]; then
    echo "$name: exit $got, expected $want" >&2
    cat "$WORK/$name.out" "$WORK/$name.err" >&2
    exit 1
  fi
}

"$PLAN_APP" --config "$PLAN_CONFIG" --output "$WORK/reference.zvs" \
  --no-diagnostics >"$WORK/plan.log" 2>&1

expect 0 servo --config "$SERVO_CONFIG" --reference "$WORK/reference.zvs" \
  --log "$WORK/servo.csv" --motion "$WORK/servo.zvs"
grep -q "^follow simulation: success" "$WORK/servo.out"
expect 0 ideal --config "$IDEAL_CONFIG" --reference "$WORK/reference.zvs" \
  --log "$WORK/ideal.csv" --motion "$WORK/ideal.zvs"
grep -q "^follow simulation: success" "$WORK/ideal.out"

python3 - "$WORK/servo.csv" "$WORK/ideal.csv" <<'PY'
import csv
import math
import sys

LSB = 2.0 * math.pi / 4096.0  # the encoder grid quantize_encoder rounds to
ELBOW = 3                     # canonical joint 3, kp = 20 Nm/rad
ELBOW_KP = 20.0


def load(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def tracking(rows):
    return [row for row in rows if row["phase"] == "tracking"]


def aggregate_rms(rows):
    """RMS over the 8 joints of each joint's tracking-error RMS [rad]."""
    per_joint = []
    for j in range(8):
        e = [float(row[f"error{j}_rad"]) for row in rows]
        per_joint.append(math.sqrt(sum(x * x for x in e) / len(e)))
    return math.sqrt(sum(x * x for x in per_joint) / 8.0)


servo_all, ideal_all = load(sys.argv[1]), load(sys.argv[2])
servo, ideal = tracking(servo_all), tracking(ideal_all)
assert len(servo) > 150 and len(servo) == len(ideal), (len(servo), len(ideal))

# The size: the elbow's gravity sag at 20 Nm/rad (about 1.3 Nm / 20 Nm/rad
# = 64 mrad, every other joint far less) puts the 8-joint aggregate near
# 64 / sqrt(8) = 23 mrad, the order of the 20 to 31 mrad measured on the
# arm (docs/theory/learning-control.md, Evidence); at 1000 Nm/rad the same
# sag is 1.3 mrad and the aggregate well under 2 mrad.
servo_rms, ideal_rms = aggregate_rms(servo), aggregate_rms(ideal)
print(f"aggregate tracking RMS: servo-like {servo_rms * 1e3:.2f} mrad, "
      f"ideal {ideal_rms * 1e3:.2f} mrad")
assert 0.010 <= servo_rms <= 0.050, servo_rms
assert ideal_rms < 0.002, ideal_rms
assert servo_rms > 10.0 * ideal_rms, (servo_rms, ideal_rms)

# The structure: the elbow's mean error is the spring deflection under its
# mean logged torque, e = q - qref = -tau / kp, up to the lag term
# (T |dqref| <= 0.06 s * 0.05 rad/s = 3 mrad) and the encoder LSB (1.5 mrad).
e3 = sum(float(row[f"error{ELBOW}_rad"]) for row in servo) / len(servo)
tau3 = sum(float(row[f"tau{ELBOW}_nm"]) for row in servo) / len(servo)
print(f"elbow: mean error {e3 * 1e3:.2f} mrad, -mean tau / kp "
      f"{-tau3 / ELBOW_KP * 1e3:.2f} mrad")
assert abs(e3) > 0.03, e3
assert abs(e3 + tau3 / ELBOW_KP) < 0.005, (e3, tau3)

# The delay: a command reaches the servo one cycle after acceptance, so a
# row's applied record (the command applied during the last step) trails
# its submission by two; the ideal servo's by one. Before the first
# delayed command the servo-like arm holds and nothing is applied.
for rows, lag in ((servo, 2), (ideal, 1)):
    gaps = {int(row["submitted_seq"]) - int(row["applied_seq"]) for row in rows}
    assert gaps == {lag}, (lag, gaps)
assert [int(row["applied_seq"]) for row in servo_all[:3]] == [0, 0, 1]
assert [int(row["applied_seq"]) for row in ideal_all[:3]] == [0, 1, 2]

# The encoder: servo-like positions sit on the grid (to the log's 9
# significant digits), the ideal servo's do not.
def on_grid(value):
    counts = value / LSB
    return abs(counts - round(counts)) < 1e-5


assert all(on_grid(float(row[f"q{j}_rad"])) for row in servo_all
           for j in range(8))
assert not all(on_grid(float(row[f"q{j}_rad"])) for row in ideal_all
               for j in range(8))
PY

# bundle_replays NAME CONFIG: a bundle of CONFIG on the reference, and its
# follow.toml replayed, reproduce the same log and motion byte for byte
bundle_replays() {
  local name=$1 config=$2
  expect 0 "$name" --config "$config" --reference "$WORK/reference.zvs" \
    --bundle "$WORK/$name"
  grep -q "^follow simulation: success" "$WORK/$name.out"
  grep -qx 'status = "success"' "$WORK/$name/result.toml"
  expect 0 "$name-replay" --config "$WORK/$name/follow.toml" \
    --log "$WORK/$name-replay.csv" --motion "$WORK/$name-replay.zvs"
  cmp "$WORK/$name/simulation.csv" "$WORK/$name-replay.csv"
  cmp "$WORK/$name/simulation.zvs" "$WORK/$name-replay.zvs"
}

# same_keys SOURCE BUNDLED [ABSENT...]: the bundled follow.toml has the
# source config's servo-like [simulation] keys with the same values (the
# gains as given, a friction number for every joint as eight values), and
# none of ABSENT (keys the source may set to their defaults)
same_keys() {
  python3 - "$@" <<'PY'
import sys
import tomllib

source_path, bundled_path, absent = sys.argv[1], sys.argv[2], sys.argv[3:]
with open(source_path, "rb") as stream:
    source = tomllib.load(stream)["simulation"]
with open(bundled_path, "rb") as stream:
    bundled = tomllib.load(stream)["simulation"]
GAINS = ("position_kp", "position_kd")
FRICTION = ("coulomb_friction_nm", "viscous_friction_nm_s")
SCALARS = ("friction_v0_rad_s", "command_delay_cycles", "quantize_encoder")
for key in GAINS + FRICTION + SCALARS:
    if key not in source or key in absent:
        continue
    want = source[key]
    if key in FRICTION and not isinstance(want, list):
        want = [want] * 8
    assert bundled.get(key) == want, (key, bundled.get(key), want)
    if key not in GAINS or isinstance(want, list):
        assert type(bundled[key]) is type(want), key
for key in absent:
    assert key not in bundled, (key, bundled[key])
PY
}

# the example's servo-like keys are recorded and replay, and bundling does
# not change the run
bundle_replays archive "$SERVO_CONFIG"
cmp "$WORK/archive/simulation.csv" "$WORK/servo.csv"
same_keys "$SERVO_CONFIG" "$WORK/archive/follow.toml" \
  coulomb_friction_nm viscous_friction_nm_s friction_v0_rad_s

# the other keys: friction (eight values and one number), its v0, a
# two-cycle delay; quantization off is its default and not written
sed -e 's/^command_delay_cycles = 1$/command_delay_cycles = 2/' \
  -e 's/^quantize_encoder = true$/quantize_encoder = false/' \
  -e 's/^# \(coulomb_friction_nm\) = 0.0 .*/\1 = [0.3, 0.5, 0.3, 0.3, 0.2, 0.2, 0.2, 0.1]/' \
  -e 's/^# \(viscous_friction_nm_s\) = 0.0 .*/\1 = 0.05/' \
  -e 's/^# \(friction_v0_rad_s\) = 0.05 .*/\1 = 0.1/' \
  -e "s|^model = \"|model = \"$(dirname "$SERVO_CONFIG")/|" \
  -e "s|^hardware_config = \"|hardware_config = \"$(dirname "$SERVO_CONFIG")/|" \
  "$SERVO_CONFIG" >"$WORK/friction.toml"
grep -q '^friction_v0_rad_s = 0.1$' "$WORK/friction.toml"
grep -q '^viscous_friction_nm_s = 0.05$' "$WORK/friction.toml"
bundle_replays friction "$WORK/friction.toml"
same_keys "$WORK/friction.toml" "$WORK/friction/follow.toml" \
  quantize_encoder
# the friction changes the run: the keys are not only written
if cmp -s "$WORK/friction/simulation.csv" "$WORK/archive/simulation.csv"; then
  echo "friction: the run is the example's" >&2
  exit 1
fi

# a bundle of the ideal servo writes none of the servo-like keys
expect 0 ideal-bundle --config "$IDEAL_CONFIG" \
  --reference "$WORK/reference.zvs" --bundle "$WORK/ideal-bundle"
same_keys "$IDEAL_CONFIG" "$WORK/ideal-bundle/follow.toml" \
  coulomb_friction_nm viscous_friction_nm_s friction_v0_rad_s \
  command_delay_cycles quantize_encoder
grep -qx 'position_kp = 1000' "$WORK/ideal-bundle/follow.toml"
grep -qx 'position_kd = 5' "$WORK/ideal-bundle/follow.toml"

rm -rf "$WORK"
echo "follow servo-like simulation smoke OK"
