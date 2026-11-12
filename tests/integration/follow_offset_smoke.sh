#!/usr/bin/env bash
# A learned command offset through the follow frontends end to end. A zero
# offset (trial 0, made by x7_follow_learn init, so it carries the reference
# signature the follower checks; the learned ones are edited from it by
# hand) passes --check, runs with an offset_ramp phase between homing and
# tracking, and is archived in a bundle that records its provenance and
# replays bit for bit; a small learned offset moves the command and only
# the command. Tampered or over-limit offsets, a raised limit, another
# control rate, the same reference under other processing or another
# control mode (from the config or the command line) and an offset with a
# posture reference are refused before any run output, and so is a
# reference whose tracking grid cannot be built. x7_follow only ever gets
# --check or a refusal, and a --port that does not exist. Works in a run
# directory of its own under ROOT, removed after a pass and kept after a
# failure; nothing else is ever deleted.
set -eu

FOLLOW_SIM="$1"
FOLLOW_HW="$2"
PLAN_APP="$3"
LEARN_APP="$4"
PLAN_CONFIG="$5"
MODEL="$6"
HARDWARE="$7"
ROOT="$8"

mkdir -p "$ROOT"
WORK=$(mktemp -d "$ROOT/run.XXXXXX")
report_kept() {
  local status=$?
  if [ "$status" -ne 0 ]; then
    echo "follow offset smoke: files kept in $WORK" >&2
  fi
}
trap report_kept EXIT
NO_PORT="$WORK/no-such-port"

# expect EXIT NAME APP [ARGS...]: run APP, capture NAME.out and NAME.err
expect() {
  local want=$1 name=$2 app=$3
  shift 3
  set +e
  "$app" "$@" </dev/null >"$WORK/$name.out" 2>"$WORK/$name.err"
  local got=$?
  set -e
  if [ "$got" -ne "$want" ]; then
    echo "$name: exit $got, expected $want" >&2
    cat "$WORK/$name.out" "$WORK/$name.err" >&2
    exit 1
  fi
}

# refused NAME MESSAGE APP [ARGS...]: exit 1 naming MESSAGE, before any run
# output and without touching a bus
refused() {
  local name=$1 message=$2
  shift 2
  expect 1 "$name" "$@"
  if ! grep -qF -- "$message" "$WORK/$name.err"; then
    echo "$name: expected '$message' on stderr" >&2
    cat "$WORK/$name.out" "$WORK/$name.err" >&2
    exit 1
  fi
  if grep -q '^bus:' "$WORK/$name.out"; then
    echo "$name: reached the bus" >&2
    exit 1
  fi
  test ! -e "$WORK/$name.csv"
  test ! -e "$WORK/$name.zvs"
}

"$PLAN_APP" --config "$PLAN_CONFIG" --output "$WORK/reference.zvs" \
  --no-diagnostics --motion-time 0.2 >"$WORK/plan.log" 2>&1

# follow.toml plus EXTRA TOML lines
write_config() {
  local path=$1 extra=$2
  cat >"$path" <<EOF
format = "rtctrl-x7-follow"
version = 1
model = "$MODEL"
reference = "$WORK/reference.zvs"
hardware_config = "$HARDWARE"

[control]
rate_hz = 100
mode = "position"

[home]
velocity_limit = 4.0
strict = false
tolerance_rad = 0.02
settle_time_s = 0.05

[simulation]
initial_posture = "zeros"
integration_step_s = 0.0001

[finalization]
simulation_hold_time_s = 0.05
$extra
EOF
}
write_config "$WORK/follow.toml" ""

# The zero offset of follow.toml's reference, from the offline tool.
"$LEARN_APP" init --follow-config "$WORK/follow.toml" \
  --output "$WORK/zero.offset.csv" >"$WORK/init.log" 2>&1

# derive_offset OUTPUT METHOD DELTA3 [REFERENCE_SHA256]: zero.offset.csv
# with DELTA3 rad on joint 3 at every sample (trial 1 with placeholder
# lineage unless the method is zero), and another reference_sha256 if
# given; the grid and the signature are the tool's
derive_offset() {
  python3 - "$WORK/zero.offset.csv" "$@" <<'PY'
import sys

source, output, method, delta3 = sys.argv[1:5]
signature = sys.argv[5] if len(sys.argv) > 5 else None
with open(source) as stream:
    lines = stream.read().split("\n")
assert lines[-1] == "", "no final newline"
zero = method == "zero"
lineage = "" if zero else " " + "e" * 64
derived = []
for line in lines[:-1]:
    if line.startswith("# "):
        key = line[2:].split(":", 1)[0]
        if key == "method":
            line = "# method: " + method
        elif key == "trial":
            line = "# trial: " + ("0" if zero else "1")
        elif key in ("parent_sha256", "source_log_sha256"):
            line = "# " + key + ":" + lineage
        elif key == "reference_sha256" and signature:
            line = "# reference_sha256: " + signature
    elif not line.startswith("time_s,"):
        fields = line.split(",")
        fields[4] = delta3  # time_s, delta0_rad, ..., delta3_rad
        line = ",".join(fields)
    derived.append(line)
with open(output, "x") as stream:
    stream.write("\n".join(derived) + "\n")
PY
}
derive_offset "$WORK/small.offset.csv" ilc 0.01
derive_offset "$WORK/large.offset.csv" ilc 0.2
derive_offset "$WORK/wrong-hash.offset.csv" zero 0 \
  "$(printf 'f%.0s' $(seq 64))"
# The signature is not the reference file's hash.
SIGNATURE=$(sed -n 's/^# reference_sha256: //p' "$WORK/zero.offset.csv")
test "${#SIGNATURE}" -eq 64
test "$SIGNATURE" != "$(sha256sum "$WORK/reference.zvs" | cut -d ' ' -f 1)"
ZERO_SHA=$(sha256sum "$WORK/zero.offset.csv" | cut -d ' ' -f 1)
SMALL_SHA=$(sha256sum "$WORK/small.offset.csv" | cut -d ' ' -f 1)

# --check covers the offset, in both frontends
ACCEPTED="command offset accepted: method zero, trial 0, max |delta| 0 rad,"
ACCEPTED="$ACCEPTED sha256 $ZERO_SHA"
expect 0 check "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/zero.offset.csv" --check
grep -qxF "$ACCEPTED" "$WORK/check.out"
grep -qx "configuration and reference preflight passed" "$WORK/check.out"
expect 0 check-hw "$FOLLOW_HW" --config "$WORK/follow.toml" \
  --offset "$WORK/zero.offset.csv" --port "$NO_PORT" --check
grep -qxF "$ACCEPTED" "$WORK/check-hw.out"
grep -qx "configuration and reference preflight passed" "$WORK/check-hw.out"
if grep -q '^bus:' "$WORK/check-hw.out"; then
  echo "check-hw: reached the bus" >&2
  exit 1
fi
# [learning] offset resolves against the config's directory
write_config "$WORK/configured.toml" '[learning]
offset = "zero.offset.csv"'
expect 0 configured "$FOLLOW_SIM" --config "$WORK/configured.toml" --check
grep -qxF "$ACCEPTED" "$WORK/configured.out"
# without one, no offset line
expect 0 plain-check "$FOLLOW_SIM" --config "$WORK/follow.toml" --check
if grep -q "command offset" "$WORK/plain-check.out"; then
  echo "plain-check: an offset was reported without one" >&2
  exit 1
fi

# a run: the ramp sits between homing and tracking; a zero offset commands
# the reference itself
expect 0 zero "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/zero.offset.csv" --motion "$WORK/zero-run.zvs" \
  --log "$WORK/zero-run.csv"
grep -q "^follow simulation: success" "$WORK/zero.out"
PHASES=$(cut -d, -f3 "$WORK/zero-run.csv" | uniq | tr '\n' ' ')
test "$PHASES" = "phase home home_correction offset_ramp tracking finalizing "

# check_log LOG DELTA3 steps|any: tracking commands are the reference plus
# DELTA3 on joint 3 and nothing elsewhere; the ramp logs q_d(0) at rest,
# moves from the last homing command toward q_d(0) + delta at most 0.5
# rad/s (with "steps": in at least one step), and ends there; the final
# hold keeps the offset
check_log() {
  python3 - "$1" "$2" "$3" <<'PY'
import csv
import sys

path, delta3, steps = sys.argv[1], float(sys.argv[2]), sys.argv[3]
with open(path, newline="") as stream:
    rows = list(csv.DictReader(stream))


def joint(row, prefix, j, unit):
    return float(row[f"{prefix}{j}_{unit}"])


tracking = [row for row in rows if row["phase"] == "tracking"]
ramp = [row for row in rows if row["phase"] == "offset_ramp"]
final = [row for row in rows if row["phase"] == "finalizing"]
homing = [row for row in rows if row["phase"] == "home_correction"]
assert tracking and ramp and final and homing, path
for row in tracking + final:
    for j in range(8):
        offset = joint(row, "qcmd", j, "rad") - joint(row, "qref", j, "rad")
        want = delta3 if j == 3 else 0.0
        assert abs(offset - want) < 1e-8, (row["time_s"], j, offset)
start = [joint(homing[-1], "qcmd", j, "rad") for j in range(8)]
target = [joint(tracking[0], "qcmd", j, "rad") for j in range(8)]
previous = start
moved = 0
for row in ramp:
    for j in range(8):
        assert joint(row, "qref", j, "rad") == joint(tracking[0], "qref", j,
                                                     "rad")
        assert joint(row, "dqref", j, "rad_s") == 0.0
        assert joint(row, "ddqref", j, "rad_s2") == 0.0
        command = joint(row, "qcmd", j, "rad")
        assert abs(command - previous[j]) <= 0.5 * 0.01 + 1e-8, (j, command)
    current = [joint(row, "qcmd", j, "rad") for j in range(8)]
    moved += current != previous
    previous = current
assert all(abs(p - t) < 1e-8 for p, t in zip(previous, target))
assert steps == "any" or moved >= 1, moved
PY
}
check_log "$WORK/zero-run.csv" 0 any

# a learned offset moves the command, and only the command
expect 0 small "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/small.offset.csv" --motion "$WORK/small-run.zvs" \
  --log "$WORK/small-run.csv"
grep -q "^follow simulation: success" "$WORK/small.out"
grep -qF "method ilc, trial 1, max |delta| 0.01 rad, sha256 $SMALL_SHA" \
  "$WORK/small.out"
check_log "$WORK/small-run.csv" 0.01 steps

# a bundle archives the offset and its provenance, and replays bit for bit
BUNDLE="$WORK/bundle"
expect 0 bundle "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/small.offset.csv" --bundle "$BUNDLE"
cmp "$WORK/small.offset.csv" "$BUNDLE/offset.csv"
grep -qx '\[learning\]' "$BUNDLE/follow.toml"
grep -qx 'offset = "offset.csv"' "$BUNDLE/follow.toml"
grep -qx 'max_offset_rad = 0.14999999999999999' "$BUNDLE/follow.toml"
grep -qx 'max_offset_rate_rad_s = 0.5' "$BUNDLE/follow.toml"
grep -qx 'status = "success"' "$BUNDLE/result.toml"
grep -qx 'offset_method = "ilc"' "$BUNDLE/result.toml"
grep -qx 'offset_trial = 1' "$BUNDLE/result.toml"
grep -qx "offset_sha256 = \"$SMALL_SHA\"" "$BUNDLE/result.toml"
grep -q "\"offset.csv\"" "$BUNDLE/manifest.toml"
grep -q "$SMALL_SHA" "$BUNDLE/manifest.toml"
expect 0 replay "$FOLLOW_SIM" --config "$BUNDLE/follow.toml" \
  --motion "$WORK/replay.zvs" --log "$WORK/replay.csv"
grep -qF "sha256 $SMALL_SHA" "$WORK/replay.out"
cmp "$BUNDLE/simulation.zvs" "$WORK/replay.zvs"
cmp "$BUNDLE/simulation.csv" "$WORK/replay.csv"
# without an offset the bundle has none
expect 0 plain-bundle "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --bundle "$WORK/plain-bundle"
test ! -e "$WORK/plain-bundle/offset.csv"
if grep -q "learning\|offset" "$WORK/plain-bundle/follow.toml" \
    "$WORK/plain-bundle/result.toml"; then
  echo "plain-bundle: offset provenance without an offset" >&2
  exit 1
fi

# refusals, before any output
run_args() {  # NAME: the output arguments of a refused simulation run
  echo --motion "$WORK/$1.zvs" --log "$WORK/$1.csv"
}
# shellcheck disable=SC2046
refused wrong-hash "does not match the reference being run" "$FOLLOW_SIM" \
  --config "$WORK/follow.toml" --offset "$WORK/wrong-hash.offset.csv" \
  $(run_args wrong-hash)
refused wrong-hash-check "does not match the reference being run" \
  "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/wrong-hash.offset.csv" --check
refused wrong-hash-hw "does not match the reference being run" \
  "$FOLLOW_HW" --config "$WORK/follow.toml" \
  --offset "$WORK/wrong-hash.offset.csv" --port "$NO_PORT" --check
# shellcheck disable=SC2046
refused large "delta 0.2 rad exceeds max_abs_rad 0.15" "$FOLLOW_SIM" \
  --config "$WORK/follow.toml" --offset "$WORK/large.offset.csv" \
  $(run_args large)
write_config "$WORK/lowered.toml" '[learning]
max_offset_rad = 0.005'
# shellcheck disable=SC2046
refused lowered "delta 0.01 rad exceeds max_abs_rad 0.005" "$FOLLOW_SIM" \
  --config "$WORK/lowered.toml" --offset "$WORK/small.offset.csv" \
  $(run_args lowered)
write_config "$WORK/raised.toml" '[learning]
max_offset_rad = 0.2'
refused raised "the limit can only be lowered" "$FOLLOW_SIM" \
  --config "$WORK/raised.toml" --offset "$WORK/zero.offset.csv" --check
sed -e 's/^rate_hz = 100$/rate_hz = 50/' "$WORK/follow.toml" \
  >"$WORK/slow.toml"
refused slow "samples, the tracking grid" "$FOLLOW_SIM" \
  --config "$WORK/slow.toml" --offset "$WORK/zero.offset.csv" --check
# the offset is bound to the reference signature: the same reference file
# under another filter, cut-off, interpolation or control mode is another
# trajectory, whether the setting comes from the config or the command line
SIG="is not this run's reference signature, which covers the reference"
refused filtered "$SIG" "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/zero.offset.csv" --filter low-pass --check
refused linear "$SIG" "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/zero.offset.csv" --interpolation linear --check
write_config "$WORK/cutoff.toml" '[reference_processing]
low_pass_cutoff_hz = 4.0'
refused cutoff "$SIG" "$FOLLOW_SIM" --config "$WORK/cutoff.toml" \
  --offset "$WORK/zero.offset.csv" --check
refused cbp "$SIG" "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/zero.offset.csv" --mode current-based-position \
  --effort-limit-nm 1 --check
refused cbp-hw "$SIG" "$FOLLOW_HW" --config "$WORK/follow.toml" \
  --offset "$WORK/zero.offset.csv" --mode current-based-position \
  --effort-limit-nm 1 --port "$NO_PORT" --log "$WORK/cbp-hw.csv"
head -c -1 "$WORK/zero.offset.csv" >"$WORK/truncated.offset.csv"
refused truncated "missing final newline" "$FOLLOW_SIM" \
  --config "$WORK/follow.toml" --offset "$WORK/truncated.offset.csv" --check
refused missing "cannot read the offset file" "$FOLLOW_SIM" \
  --config "$WORK/follow.toml" --offset "$WORK/no-such.offset.csv" --check
refused refused-bundle "does not match the reference being run" \
  "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --offset "$WORK/wrong-hash.offset.csv" --bundle "$WORK/refused-bundle"
test ! -e "$WORK/refused-bundle"
for staging in "$WORK"/.refused-bundle*; do
  if [ -e "$staging" ]; then
    echo "refused-bundle: its staging directory was left behind" >&2
    exit 1
  fi
done
# a reference whose tracking grid cannot be built (two 1e17 s frames: more
# control periods than a long counts) is refused by the reference
# preflight, offset or not, before any bus contact; the follower would
# only build that grid after homing
HUGE="$WORK/huge-reference.zvs"
printf '1e17 8 ( 0 0 0 0 0 0 0 0 )\n1e17 8 ( 0 0 0 0 0 0 0 0 )\n' >"$HUGE"
GRID="the reference cannot be tracked at control.rate_hz = 100"
# shellcheck disable=SC2046
refused huge "$GRID" "$FOLLOW_SIM" --config "$WORK/follow.toml" \
  --reference "$HUGE" $(run_args huge)
refused huge-hw "$GRID" "$FOLLOW_HW" --config "$WORK/follow.toml" \
  --reference "$HUGE" --port "$NO_PORT" --log "$WORK/huge-hw.csv"
# a reference built from the measured posture takes no offset
POSTURE="cannot be applied to a reference built from the measured posture"
refused posture "$POSTURE" \
  "$FOLLOW_HW" --config "$WORK/configured.toml" --hold-current 1 \
  --port "$NO_PORT" --log "$WORK/posture.csv"
refused posture-cli "$POSTURE" \
  "$FOLLOW_HW" --config "$WORK/follow.toml" --offset "$WORK/zero.offset.csv" \
  --sweep-current 6:0.1:1 --port "$NO_PORT" --log "$WORK/posture-cli.csv"
test ! -e "$NO_PORT"

rm -rf "$WORK"
echo "follow offset smoke OK"
