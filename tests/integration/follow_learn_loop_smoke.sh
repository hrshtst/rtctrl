#!/usr/bin/env bash
# The learning loop end to end in simulation: x7_plan_ptp makes the pick
# example's reference, a copy of config/follow_sim_servo_like.toml follows
# it, x7_follow_learn init writes the trial-0 offset, then four trials of
# x7_follow_sim --offset, each followed by x7_follow_learn ilc --report
# (phi = 0.5). Every run must succeed with the follower's preflight
# accepting the offset the tool wrote (its sha256 printed, its lineage and
# reference signature chained to the previous trial's), and the aggregate
# tracking RMS over all 8 joints must fall at every trial, trial 3 below
# 25 % of trial 0: with gain 0.5 the repeatable error roughly halves per
# update, so three updates leave about an eighth plus the encoder floor.
# Works in a run directory of its own under ROOT, removed after a pass and
# kept after a failure; nothing else is ever deleted. Never opens the bus.
set -eu

TOOL="$1"
PLAN="$2"
FOLLOW_SIM="$3"
PICK_CONFIG="$4"
SERVO_CONFIG="$5"
MODEL="$6"
HARDWARE="$7"
ROOT="$8"

TRIALS=4

mkdir -p "$ROOT"
WORK=$(mktemp -d "$ROOT/run.XXXXXX")
report_kept() {
  local status=$?
  if [ "$status" -ne 0 ]; then
    echo "follow learn loop smoke: files kept in $WORK" >&2
  fi
}
trap report_kept EXIT

fail() {
  echo "follow learn loop smoke: $*" >&2
  exit 1
}

# run NAME APP [ARGS...]: APP must exit 0; NAME.out and NAME.err kept
run() {
  local name=$1 app=$2
  shift 2
  if ! "$app" "$@" </dev/null >"$WORK/$name.out" 2>"$WORK/$name.err"; then
    cat "$WORK/$name.out" "$WORK/$name.err" >&2
    fail "$name failed"
  fi
}

sha() { sha256sum "$1" | cut -d' ' -f1; }

# header KEY FILE: the value of an offset file's "# KEY: value" line
header() {
  sed -n "s/^# $1: *//p" "$2"
}

check_equal() {  # check_equal WHAT GOT WANT
  if [ "$2" != "$3" ]; then
    fail "$1 is '$2', expected '$3'"
  fi
}

run plan "$PLAN" --config "$PICK_CONFIG" --output "$WORK/pick.zvs" \
  --no-diagnostics

# The servo-like example pointed at this run's reference.
sed -e "s|^model = .*|model = \"$MODEL\"|" \
  -e "s|^reference = .*|reference = \"$WORK/pick.zvs\"|" \
  -e "s|^hardware_config = .*|hardware_config = \"$HARDWARE\"|" \
  "$SERVO_CONFIG" >"$WORK/follow.toml"
grep -q "^reference = \"$WORK/pick.zvs\"$" "$WORK/follow.toml" ||
  fail "follow config not written"
cat >"$WORK/learn.toml" <<'EOF'
format = "rtctrl-x7-follow-learn"
version = 1

[ilc]
phi = 0.5
EOF

run init "$TOOL" init --follow-config "$WORK/follow.toml" \
  --output "$WORK/offset0.csv"
SIGNATURE=$(header reference_sha256 "$WORK/offset0.csv")

RMS=()
for ((k = 0; k < TRIALS; ++k)); do
  OFFSET="$WORK/offset$k.csv"
  LOG="$WORK/trial$k.csv"
  run "sim$k" "$FOLLOW_SIM" --config "$WORK/follow.toml" --offset "$OFFSET" \
    --log "$LOG" --motion "$WORK/trial$k.zvs"
  grep -q "^follow simulation: success," "$WORK/sim$k.out" ||
    fail "trial $k did not succeed"
  # the follower's preflight accepted exactly the file the tool wrote
  ACCEPTED="^command offset accepted: method (zero|ilc), trial $k, "
  ACCEPTED="$ACCEPTED.*, sha256 $(sha "$OFFSET")$"
  grep -qE "$ACCEPTED" "$WORK/sim$k.out" ||
    fail "trial $k: offset$k.csv not accepted as written"

  NEXT="$WORK/offset$((k + 1)).csv"
  run "ilc$k" "$TOOL" ilc --config "$WORK/learn.toml" \
    --follow-config "$WORK/follow.toml" --log "$LOG" --offset "$OFFSET" \
    --output "$NEXT" --report "$WORK/report$k.toml"
  check_equal "offset$((k + 1)) method" "$(header method "$NEXT")" ilc
  check_equal "offset$((k + 1)) trial" "$(header trial "$NEXT")" $((k + 1))
  check_equal "offset$((k + 1)) parent" "$(header parent_sha256 "$NEXT")" \
    "$(sha "$OFFSET")"
  check_equal "offset$((k + 1)) source log" \
    "$(header source_log_sha256 "$NEXT")" "$(sha "$LOG")"
  check_equal "offset$((k + 1)) reference signature" \
    "$(header reference_sha256 "$NEXT")" "$SIGNATURE"
  RMS+=("$(python3 - "$WORK/report$k.toml" "$k" <<'PY'
import sys
import tomllib

with open(sys.argv[1], "rb") as stream:
    report = tomllib.load(stream)
assert report["command"] == "ilc", report["command"]
assert report["measured_trial"] == int(sys.argv[2]), report["measured_trial"]
print(repr(report["tracking"]["aggregate_rms_rad"]))
PY
)")
done

python3 - "${RMS[@]}" <<'PY'
import sys

rms = [float(value) for value in sys.argv[1:]]
print("aggregate tracking RMS by trial [mrad]: " +
      ", ".join(f"{value * 1e3:.2f}" for value in rms))
for k in range(1, len(rms)):
    assert rms[k] < rms[k - 1], f"trial {k} did not improve: {rms}"
assert rms[-1] < 0.25 * rms[0], f"trial 3 not below 25 % of trial 0: {rms}"
PY

rm -rf "$WORK"
echo "follow learn loop smoke OK"
