#!/usr/bin/env bash
# The hardware frontend x7_follow with a learned command offset, through
# complete runs against dxl_emu (the pty bus emulator, its wire protocol
# and servo model): a run with the zero offset of x7_follow_learn init;
# x7_follow_learn ilc learning from that hardware log (trial 1); a run with
# a nonzero offset, whose commands carry the offset (ramp, tracking, final
# hold), whose writes the bus layer records, and whose measured positions at
# rest sit on the goal pulse of each command; and an over-limit offset
# refused before the bus is opened. Functional only: a pty answers
# instantly, so no timing is asserted beyond the run summary's counters.
#
# Safety: x7_follow defaults to the real arm's port. Here it only ever runs
# through follow(), which first checks that this script's emulator is alive
# and that its link is a symlink to the /dev/pts device the emulator serves,
# refuses a --port among the arguments, and appends --port with that link
# last (the last --port wins). The hardware config it is given is a copy
# whose [bus] port is a path in the run directory that does not exist, so a
# run that lost its --port would fail to open, not reach a serial adapter.
#
# Works in a run directory of its own under ROOT, removed after a pass and
# kept after a failure; nothing else is ever deleted. The emulator is
# stopped on every exit.
set -eu

EMU="$1"
FOLLOW="$2"
PLAN="$3"
LEARN="$4"
PLAN_CONFIG="$5"
MODEL="$6"
HARDWARE="$7"
ROOT="${8:?the eighth argument, ROOT, is required}"

mkdir -p "$ROOT"
WORK=$(mktemp -d "$ROOT/run.XXXXXX")
LINK="$WORK/tty"
EMU_PID=""
stop_emulator() {
  if [ -n "$EMU_PID" ]; then
    kill "$EMU_PID" 2>/dev/null || true
    wait "$EMU_PID" 2>/dev/null || true
    EMU_PID=""
  fi
}
finish() {
  local status=$?
  stop_emulator
  if [ "$status" -ne 0 ]; then
    echo "follow offset emulator smoke: files kept in $WORK" >&2
  fi
}
trap finish EXIT

fail() {
  echo "follow offset emulator smoke: $*" >&2
  exit 1
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

# The DynamixelSDK refuses device paths over 100 characters.
if [ "${#LINK}" -gt 100 ]; then
  fail "the emulator link $LINK is longer than 100 characters"
fi

# The deployment config with a [bus] port that does not exist.
NO_PORT="$WORK/no-such-port"
sed -e "s|^port = .*|port = \"$NO_PORT\"|" "$HARDWARE" >"$WORK/crane_x7.toml"
grep -qxF "port = \"$NO_PORT\"" "$WORK/crane_x7.toml" ||
  fail "the hardware config copy has no port line to replace"
if grep -q 'ttyUSB\|/dev/' "$WORK/crane_x7.toml"; then
  fail "the hardware config copy still names a device"
fi

"$EMU" --config "$HARDWARE" --link "$LINK" >"$WORK/emu.log" 2>&1 &
EMU_PID=$!
for _ in $(seq 1 50); do
  grep -qxF "linked as $LINK" "$WORK/emu.log" && break
  sleep 0.1
done

# This script's emulator is running and LINK is a symlink to the pty it
# serves; anything else stops the script before x7_follow is started.
require_emulator() {
  kill -0 "$EMU_PID" 2>/dev/null || fail "the emulator is not running"
  [ -L "$LINK" ] || fail "$LINK is not a symlink"
  local target
  target=$(readlink -- "$LINK")
  [[ $target =~ ^/dev/pts/[0-9]+$ ]] || fail "$LINK points at '$target'"
  [ -c "$target" ] || fail "$target is not a character device"
  grep -qxF "serving on $target" "$WORK/emu.log" ||
    fail "the emulator does not serve $target"
  grep -qxF "linked as $LINK" "$WORK/emu.log" ||
    fail "the emulator did not link $LINK"
}
require_emulator

# follow WANT NAME [ARGS...]: x7_follow ARGS on the emulator, exit WANT;
# NAME.out and NAME.err kept
follow() {
  local want=$1 name=$2
  shift 2
  local arg
  for arg in "$@"; do
    [ "$arg" != --port ] || fail "$name: --port is the emulator's alone"
  done
  require_emulator
  set +e
  "$FOLLOW" "$@" --port "$LINK" </dev/null >"$WORK/$name.out" \
    2>"$WORK/$name.err"
  local got=$?
  set -e
  if [ "$got" -ne "$want" ]; then
    cat "$WORK/$name.out" "$WORK/$name.err" >&2
    fail "$name: x7_follow exited $got, expected $want"
  fi
}

# run NAME APP [ARGS...]: an offline tool (never opens a bus), exit 0
run() {
  local name=$1 app=$2
  shift 2
  if ! "$app" "$@" </dev/null >"$WORK/$name.out" 2>"$WORK/$name.err"; then
    cat "$WORK/$name.out" "$WORK/$name.err" >&2
    fail "$name failed"
  fi
}

# follow_run NAME OFFSET: a complete run with OFFSET, logged to NAME.csv;
# the preflight line names OFFSET before the bus line, which names the link
follow_run() {
  local name=$1 offset=$2 accepted bus
  follow 0 "$name" --config "$WORK/follow.toml" --offset "$offset" \
    --log "$WORK/$name.csv"
  grep -q '^follow hardware: success,' "$WORK/$name.out" ||
    fail "$name: no success line"
  accepted=$(grep -n '^command offset accepted: ' "$WORK/$name.out" |
    cut -d: -f1)
  bus=$(grep -n '^bus: ' "$WORK/$name.out" | cut -d: -f1)
  [ -n "$accepted" ] && [ -n "$bus" ] && [ "$accepted" -lt "$bus" ] ||
    fail "$name: the offset line does not precede the bus line"
  check_equal "$name bus" \
    "$(sed -n 's/^bus: \(.*\) @ .*$/\1/p' "$WORK/$name.out")" "$LINK"
}

run plan "$PLAN" --config "$PLAN_CONFIG" --output "$WORK/reference.zvs" \
  --no-diagnostics --motion-time 0.5

cat >"$WORK/follow.toml" <<EOF
format = "rtctrl-x7-follow"
version = 1
model = "$MODEL"
reference = "$WORK/reference.zvs"
hardware_config = "$WORK/crane_x7.toml"

[control]
rate_hz = 100
mode = "position"

[home]
velocity_limit = 2.0
strict = true
tolerance_rad = 0.01
settle_time_s = 0.3

[finalization]
wait_time_s = 0.2
EOF
cat >"$WORK/learn.toml" <<'EOF'
format = "rtctrl-x7-follow-learn"
version = 1

[ilc]
phi = 0.5
EOF

# check_run LOG OFFSET [ZERO_LOG]: the commands of a completed run carry
# OFFSET (tracking sample i: qref + delta[i]; the ramp from the last homing
# command to qref(0) + delta[0] at no more than 0.5 rad/s, then held; the
# final hold qref(T) + delta[N-1]); each feedback records the previous
# cycle's command as the last one written to the bus; and the emulated
# servo at rest sits on the goal pulse of its command. With ZERO_LOG (the
# zero-offset run), the measured rest positions of the two runs differ by
# the offset within one pulse.
check_run() {
  python3 - "$@" <<'PY'
import csv
import math
import sys

# One encoder pulse: the emulated servo's position is a whole pulse count,
# its goal the command rounded to the nearest pulse (dxl/conversions.hpp).
PULSE = 2.0 * math.pi / 4096.0
# The log prints positions with %.9g.
PRINT = 1e-6
RATE = 0.5 * 0.01  # max_offset_rate_rad_s per 10 ms control period
SETTLED = 10       # cycles a command is held before its rest is judged


def read_offset(path):
    header, delta = {}, []
    with open(path) as stream:
        for line in stream.read().split("\n")[:-1]:
            if line.startswith("# "):
                key, _, value = line[2:].partition(":")
                header[key] = value.strip()
            elif not line.startswith("time_s,"):
                delta.append([float(v) for v in line.split(",")[1:]])
    return header, delta


def read_log(path):
    with open(path, newline="") as stream:
        return list(csv.DictReader(stream))


def vec(row, prefix, suffix="_rad"):
    return [float(row[f"{prefix}{j}{suffix}"]) for j in range(8)]


def close(a, b, tolerance):
    return all(abs(x - y) <= tolerance for x, y in zip(a, b))


log_path, offset_path = sys.argv[1], sys.argv[2]
zero_path = sys.argv[3] if len(sys.argv) > 3 else None
_, delta = read_offset(offset_path)
rows = read_log(log_path)
assert all(row["receipt_accepted"] == "1" for row in rows), log_path
phase = {name: [row for row in rows if row["phase"] == name]
         for name in ("home_correction", "offset_ramp", "tracking",
                      "finalizing")}
tracking, ramp, final = (phase["tracking"], phase["offset_ramp"],
                         phase["finalizing"])
assert len(tracking) == len(delta), (len(tracking), len(delta))
assert ramp and final, log_path

# Tracking: the command is the reference plus the offset's sample.
for i, row in enumerate(tracking):
    offset = [c - r for c, r in zip(vec(row, "qcmd"), vec(row, "qref"))]
    assert close(offset, delta[i], PRINT), (i, offset, delta[i])
# The ramp: q_d(0) logged at rest, the command moving from the last
# homing command at most RATE per cycle, ending on q_d(0) + delta[0].
qref0, qref_final = vec(tracking[0], "qref"), vec(tracking[-1], "qref")
start = vec(rows[rows.index(ramp[0]) - 1], "qcmd")
assert rows[rows.index(ramp[0]) - 1]["phase"] in ("home",
                                                  "home_correction")
previous = start
steps = 0
for row in ramp:
    assert vec(row, "qref") == qref0, row["time_s"]
    assert all(float(row[f"dqref{j}_rad_s"]) == 0.0 for j in range(8))
    command = vec(row, "qcmd")
    assert close(command, previous, RATE + PRINT), (row["time_s"], command)
    steps += command != previous
    previous = command
target = [q + d for q, d in zip(qref0, delta[0])]
assert close(previous, target, PRINT), (previous, target)
# The final hold: q_d(T) logged, q_d(T) + delta[N-1] commanded.
for row in final:
    assert close(vec(row, "qref"), qref_final, PRINT), row["time_s"]
    hold = [q + d for q, d in zip(qref_final, delta[-1])]
    assert close(vec(row, "qcmd"), hold, PRINT), row["time_s"]

# The bus layer's record of the last successful write, taken with each
# feedback: every cycle's feedback names the previous cycle's submission
# (applied_seq) with exactly its command (applied, none clamped). A
# submission is accepted only inside its own cycle's command window and
# written in that cycle, so in a run whose every receipt was accepted this
# holds row for row.
for before, row in zip(rows, rows[1:]):
    assert row["applied_seq"] == before["submitted_seq"], (
        row["time_s"], row["applied_seq"], before["submitted_seq"])
    assert vec(row, "applied", "") == vec(before, "qcmd"), row["time_s"]
    assert all(row[f"flags{j}"] == "0" for j in range(8)), row["time_s"]

# The emulated servo moves toward its goal pulse at up to its velocity
# limit and stops on it exactly: after a command held for SETTLED cycles
# the measured position is a whole pulse within half a pulse of the
# command (its goal pulse, lround of the command).
settled = {name: 0 for name in phase}
for r in range(SETTLED, len(rows)):
    window = [rows[k][f"qcmd{j}_rad"] for k in range(r - SETTLED, r + 1)
              for j in range(8)]
    if window != window[:8] * (SETTLED + 1):
        continue
    q, command = vec(rows[r], "q"), vec(rows[r], "qcmd")
    for j in range(8):
        pulses = q[j] / PULSE
        assert abs(pulses - round(pulses)) < 1e-4, (
            rows[r]["time_s"], j, q[j])
        assert abs(q[j] - command[j]) <= PULSE / 2 + PRINT, (
            rows[r]["time_s"], j, q[j], command[j])
    if rows[r]["phase"] in settled:
        settled[rows[r]["phase"]] += 1
assert settled["offset_ramp"] > 0 and settled["finalizing"] > 0, settled
print(f"{log_path}: ramp {len(ramp)} rows ({steps} moving), tracking "
      f"{len(tracking)} rows, final hold {len(final)} rows; settled rows "
      f"{settled}")

if zero_path is not None:
    zero = read_log(zero_path)
    for name, sample in (("offset_ramp", 0), ("finalizing", -1)):
        mine = vec([row for row in rows if row["phase"] == name][-1], "q")
        base = vec([row for row in zero if row["phase"] == name][-1], "q")
        shift = [a - b for a, b in zip(mine, base)]
        for j in range(8):
            if delta[sample][j] == 0.0:
                assert abs(shift[j]) <= PRINT, (name, j, shift[j])
            else:
                assert abs(shift[j] - delta[sample][j]) <= PULSE + PRINT, (
                    name, j, shift[j], delta[sample][j])
        print(f"{name}: measured shift [mrad] "
              + " ".join(f"{1e3 * s:+.3f}" for s in shift)
              + " for delta "
              + " ".join(f"{1e3 * d:+.3f}" for d in delta[sample]))
PY
}

# ---------------------------------------------------- 1. the zero offset
run init "$LEARN" init --follow-config "$WORK/follow.toml" \
  --output "$WORK/zero.offset.csv"
ZERO_SHA=$(sha "$WORK/zero.offset.csv")
SIGNATURE=$(header reference_sha256 "$WORK/zero.offset.csv")
follow_run zero "$WORK/zero.offset.csv"
ACCEPTED="command offset accepted: method zero, trial 0, max |delta| 0 rad,"
grep -qxF "$ACCEPTED sha256 $ZERO_SHA" "$WORK/zero.out" ||
  fail "zero: the accepted offset is not reported"
PHASES=$(cut -d, -f3 "$WORK/zero.csv" | uniq | tr '\n' ' ')
case "$PHASES" in
  "phase home offset_ramp tracking finalizing " | \
    "phase home home_correction offset_ramp tracking finalizing ") ;;
  *) fail "zero: phases '$PHASES'" ;;
esac
check_run "$WORK/zero.csv" "$WORK/zero.offset.csv"
# The zero offset commands the reference itself, exactly, in tracking and
# in the final hold.
python3 - "$WORK/zero.csv" <<'PY'
import csv
import sys

with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))
for row in rows:
    if row["phase"] in ("tracking", "finalizing"):
        for j in range(8):
            assert row[f"qcmd{j}_rad"] == row[f"qref{j}_rad"], (
                row["time_s"], j)
corrected = any(row[f"qcmd{j}_rad"] != row[f"qref{j}_rad"]
                for row in rows if row["phase"] == "home_correction"
                for j in range(8))
print(f"zero run: home correction applied: {corrected}")
PY

# ------------------------------- 2. a learning step on the hardware log
# The emulated servo has no dynamics, gravity or deadband: it moves toward
# its goal pulse at up to its velocity limit (about 5 rad/s) and stops on
# it, so it reaches each tracking goal before the next read. Its error is
# the lag of one cycle (feedback is read before that cycle's command is
# written) plus up to half a pulse (0.77 mrad) of quantization, and the
# trial-1 offset is small (about a pulse during the motion), near zero at
# rest: only its lineage and grid are asserted.
run ilc1 "$LEARN" ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/zero.csv" \
  --offset "$WORK/zero.offset.csv" --output "$WORK/trial1.offset.csv" \
  --report "$WORK/report1.toml"
check_equal "trial-1 method" "$(header method "$WORK/trial1.offset.csv")" ilc
check_equal "trial-1 trial" "$(header trial "$WORK/trial1.offset.csv")" 1
check_equal "trial-1 parent" \
  "$(header parent_sha256 "$WORK/trial1.offset.csv")" "$ZERO_SHA"
check_equal "trial-1 source log" \
  "$(header source_log_sha256 "$WORK/trial1.offset.csv")" \
  "$(sha "$WORK/zero.csv")"
check_equal "trial-1 reference signature" \
  "$(header reference_sha256 "$WORK/trial1.offset.csv")" "$SIGNATURE"
for key in dt_s samples reference_duration_s; do
  check_equal "trial-1 $key" "$(header "$key" "$WORK/trial1.offset.csv")" \
    "$(header "$key" "$WORK/zero.offset.csv")"
done
python3 - "$WORK/report1.toml" <<'PY'
import sys
import tomllib

with open(sys.argv[1], "rb") as stream:
    report = tomllib.load(stream)
assert report["command"] == "ilc", report["command"]
assert report["measured_trial"] == 0, report["measured_trial"]
print("trial 0 on the emulator: aggregate tracking RMS "
      f"{1e3 * report['tracking']['aggregate_rms_rad']:.3f} mrad")
PY

# ------------------------------------------------ 3. a nonzero offset
# The trial-1 file with its values replaced by smooth bumps on joints 1
# and 5 (6 to 10 mrad, -5 to -8 mrad; nonzero at both ends, so the ramp
# moves and the final hold carries an offset): a valid offset (format,
# grid, signature and trial-1 lineage untouched, within 0.15 rad and 0.5
# rad/s). The follower checks the lineage's form, not the parent's content,
# so a hand-edited trial-1 offset is a legitimate rehearsal input.
python3 - "$WORK/trial1.offset.csv" "$WORK/bump.offset.csv" <<'PY'
import math
import sys

source, output = sys.argv[1:3]
with open(source) as stream:
    lines = stream.read().split("\n")
assert lines[-1] == "", "no final newline"
header = {}
for line in lines:
    if line.startswith("# "):
        key, _, value = line[2:].partition(":")
        header[key] = value.strip()
dt = float(header["dt_s"])
duration = float(header["reference_duration_s"])
bump = {1: (0.006, 0.004), 5: (-0.005, -0.003)}  # joint: (ends, rise)
out, sample = [], 0
for line in lines[:-1]:
    if not line.startswith("# ") and not line.startswith("time_s,"):
        fields = line.split(",")
        shape = math.sin(math.pi * sample * dt / duration) ** 2
        for j in range(8):
            ends, rise = bump.get(j, (0.0, 0.0))
            fields[1 + j] = repr(ends + rise * shape)
        line = ",".join(fields)
        sample += 1
    out.append(line)
assert sample == int(header["samples"]), sample
with open(output, "x") as stream:
    stream.write("\n".join(out) + "\n")
PY
BUMP_SHA=$(sha "$WORK/bump.offset.csv")
MAX_DELTA=$(python3 - "$WORK/bump.offset.csv" <<'PY'
import sys

values = [abs(float(v)) for line in open(sys.argv[1])
          if not line.startswith(("#", "time_s,"))
          for v in line.rstrip("\n").split(",")[1:]]
print("%.6g" % max(values))
PY
)
follow_run bump "$WORK/bump.offset.csv"
ACCEPTED="command offset accepted: method ilc, trial 1, max |delta|"
grep -qxF "$ACCEPTED $MAX_DELTA rad, sha256 $BUMP_SHA" "$WORK/bump.out" ||
  fail "bump: the accepted offset is not reported"
PHASES=$(cut -d, -f3 "$WORK/bump.csv" | uniq | tr '\n' ' ')
case "$PHASES" in
  "phase home offset_ramp tracking finalizing " | \
    "phase home home_correction offset_ramp tracking finalizing ") ;;
  *) fail "bump: phases '$PHASES'" ;;
esac
check_run "$WORK/bump.csv" "$WORK/bump.offset.csv" "$WORK/zero.csv"
# The learning tool reads this log as a run with exactly this offset
# (qcmd - qref = delta, sample for sample) and chains trial 2 to it.
run ilc2 "$LEARN" ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/bump.csv" \
  --offset "$WORK/bump.offset.csv" --output "$WORK/trial2.offset.csv"
check_equal "trial-2 trial" "$(header trial "$WORK/trial2.offset.csv")" 2
check_equal "trial-2 parent" \
  "$(header parent_sha256 "$WORK/trial2.offset.csv")" "$BUMP_SHA"
check_equal "trial-2 source log" \
  "$(header source_log_sha256 "$WORK/trial2.offset.csv")" \
  "$(sha "$WORK/bump.csv")"

# ------------------------------------------- 4. the run summary lines
# No write or read failures in either run, and no stale submissions or
# skipped periods in the offset run beyond the zero run's.
python3 - "$WORK/zero.out" "$WORK/bump.out" <<'PY'
import re
import sys


def summary(path):
    text = open(path).read()
    timing = re.search(r"^hardware timing: (\d+) cycles, (\d+) overruns, "
                       r"(\d+) skipped, (\d+) stale, (\d+) deadline misses",
                       text, re.M)
    faults = re.search(r"^hardware faults: (\d+) read failures, (\d+) write "
                       r"failures", text, re.M)
    assert timing and faults, path
    names = ("cycles", "overruns", "skipped", "stale", "misses")
    counts = dict(zip(names, map(int, timing.groups())))
    counts.update(zip(("read_failures", "write_failures"),
                      map(int, faults.groups())))
    return counts


zero, bump = summary(sys.argv[1]), summary(sys.argv[2])
print(f"zero run: {zero}\noffset run: {bump}")
for run in (zero, bump):
    assert run["write_failures"] == 0 and run["read_failures"] == 0, run
for key in ("stale", "skipped", "write_failures"):
    assert bump[key] <= zero[key], (key, bump[key], zero[key])
PY

# ---------------------- 5. an over-limit offset, refused before the bus
# One value of the nonzero offset past 0.15 rad. dxl_emu logs no traffic,
# so what is established is that x7_follow exits before its bus line
# (printed just before the port is opened) and before creating its log.
python3 - "$WORK/bump.offset.csv" "$WORK/tampered.offset.csv" <<'PY'
import sys

source, output = sys.argv[1:3]
lines = open(source).read().split("\n")
rows = [i for i, line in enumerate(lines)
        if line and not line.startswith(("#", "time_s,"))]
fields = lines[rows[len(rows) // 2]].split(",")
fields[2] = "0.16"  # delta1_rad
lines[rows[len(rows) // 2]] = ",".join(fields)
with open(output, "x") as stream:
    stream.write("\n".join(lines))
PY
follow 1 tampered --config "$WORK/follow.toml" \
  --offset "$WORK/tampered.offset.csv" --log "$WORK/tampered.csv"
grep -qF "delta 0.16 rad exceeds max_abs_rad 0.15" "$WORK/tampered.err" ||
  fail "tampered: refused for another reason"
if grep -q '^bus:\|^command offset accepted' "$WORK/tampered.out"; then
  fail "tampered: passed the preflight"
fi
test ! -e "$WORK/tampered.csv" || fail "tampered: a log was created"
require_emulator

# No run reached any other device.
if grep -l 'ttyUSB' "$WORK"/*.out "$WORK"/*.err; then
  fail "a run names ttyUSB"
fi

stop_emulator
rm -rf -- "${WORK:?}"
echo "follow offset emulator smoke OK"
