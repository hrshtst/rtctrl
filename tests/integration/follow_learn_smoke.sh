#!/usr/bin/env bash
# x7_follow_learn end to end on a simulated trial: init, one plain
# x7_follow_sim run (trial 0, the zero offset), ilc, init-weights, fel and
# synth, each output's header fields and lineage hashes checked against
# sha256sum, and every output bound to its follow config's reference
# signature (init's, which is not the reference file's hash); then refusals (an existing output, a log of another reference,
# tampered offsets) that exit nonzero and write nothing. Works in a run
# directory of its own under ROOT, removed after a pass and kept after a
# failure; nothing else is ever deleted. Never opens the bus.
set -eu

TOOL="$1"
PLAN="$2"
FOLLOW_SIM="$3"
PTP_CONFIG="$4"
FOLLOW_EXAMPLE="$5"
LEARN_EXAMPLE="$6"
MODEL="$7"
HARDWARE="$8"
ROOT="$9"

mkdir -p "$ROOT"
WORK=$(mktemp -d "$ROOT/run.XXXXXX")
report_kept() {
  local status=$?
  if [ "$status" -ne 0 ]; then
    echo "follow learn smoke: files kept in $WORK" >&2
  fi
}
trap report_kept EXIT

fail() {
  echo "follow learn smoke: $*" >&2
  exit 1
}

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

sha() { sha256sum "$1" | cut -d' ' -f1; }

# header KEY FILE: the value of an offset file's "# KEY: value" line
header() {
  sed -n "s/^# $1: *//p" "$2"
}

# toml KEY FILE: the string value of a top-level "KEY = "..."" line
toml() {
  sed -n "s/^$1 = \"\(.*\)\"$/\1/p" "$2"
}

check_equal() {  # check_equal WHAT GOT WANT
  if [ "$2" != "$3" ]; then
    fail "$1 is '$2', expected '$3'"
  fi
}

# Two references from the PTP example (the follow bundle smoke's plan), the
# second longer.
"$PLAN" --config "$PTP_CONFIG" --output "$WORK/reference.zvs" \
  --diagnostics "$WORK/reference.csv" --motion-time 0.2 \
  >"$WORK/plan.log" 2>&1
"$PLAN" --config "$PTP_CONFIG" --output "$WORK/other.zvs" \
  --diagnostics "$WORK/other.csv" --motion-time 1.0 \
  >"$WORK/plan_other.log" 2>&1

# The example follow config with its input paths pointed at this run.
follow_config() {  # follow_config REFERENCE OUT
  sed -e "s|^model = .*|model = \"$MODEL\"|" \
    -e "s|^reference = .*|reference = \"$1\"|" \
    -e "s|^hardware_config = .*|hardware_config = \"$HARDWARE\"|" \
    "$FOLLOW_EXAMPLE" >"$2"
  grep -q "^reference = \"$1\"$" "$2" || fail "follow config $2 not written"
}
follow_config "$WORK/reference.zvs" "$WORK/follow.toml"
follow_config "$WORK/other.zvs" "$WORK/other.toml"

# The example learn config with ILC switched on.
sed -e 's/^phi = 0\.0 /phi = 0.5 /' "$LEARN_EXAMPLE" >"$WORK/learn.toml"
grep -q '^phi = 0\.5 ' "$WORK/learn.toml" || fail "learn config phi not set"

# --- init: the zero offset ---------------------------------------------
expect 0 init init --follow-config "$WORK/follow.toml" \
  --output "$WORK/offset0.csv"
check_equal "offset0 format" "$(header format "$WORK/offset0.csv")" \
  rtctrl-x7-follow-offset
check_equal "offset0 method" "$(header method "$WORK/offset0.csv")" zero
check_equal "offset0 trial" "$(header trial "$WORK/offset0.csv")" 0
# The reference signature (the reference file, control rate and mode, and
# the reference processing), not the file's hash; the other reference's
# differs.
REF_SIG=$(header reference_sha256 "$WORK/offset0.csv")
test "${#REF_SIG}" -eq 64 || fail "offset0 reference signature '$REF_SIG'"
test "$REF_SIG" != "$(sha "$WORK/reference.zvs")" ||
  fail "offset0 carries the reference file's hash"
expect 0 init_other init --follow-config "$WORK/other.toml" \
  --output "$WORK/other0.csv"
OTHER_SIG=$(header reference_sha256 "$WORK/other0.csv")
test "$OTHER_SIG" != "$REF_SIG" || fail "both references sign alike"
check_equal "offset0 parent" "$(header parent_sha256 "$WORK/offset0.csv")" ""
SAMPLES=$(header samples "$WORK/offset0.csv")
grep -q "wrote offset $WORK/offset0.csv" "$WORK/init.out"

# --- trial 0: one plain simulated run (its command is the reference) -----
"$FOLLOW_SIM" --config "$WORK/follow.toml" --log "$WORK/trial0.csv" \
  --motion "$WORK/trial0.zvs" >"$WORK/sim.log" 2>&1 ||
  { cat "$WORK/sim.log" >&2; fail "x7_follow_sim failed"; }
TRACKING=$(cut -d, -f3 "$WORK/trial0.csv" | grep -c '^tracking$')
check_equal "tracking rows" "$TRACKING" "$SAMPLES"
LOG_SHA=$(sha "$WORK/trial0.csv")

# --- ilc ------------------------------------------------------------------
expect 0 ilc ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/trial0.csv" \
  --offset "$WORK/offset0.csv" --output "$WORK/offset1.csv" \
  --report "$WORK/report1.toml"
O1="$WORK/offset1.csv"
check_equal "offset1 method" "$(header method "$O1")" ilc
check_equal "offset1 trial" "$(header trial "$O1")" 1
check_equal "offset1 samples" "$(header samples "$O1")" "$SAMPLES"
check_equal "offset1 reference" "$(header reference_sha256 "$O1")" "$REF_SIG"
check_equal "offset1 parent" "$(header parent_sha256 "$O1")" \
  "$(sha "$WORK/offset0.csv")"
check_equal "offset1 source log" "$(header source_log_sha256 "$O1")" "$LOG_SHA"
grep -q "tracking RMS of trial 0: .* rad over all 8 joints" "$WORK/ilc.out"
check_equal "report1 command" "$(toml command "$WORK/report1.toml")" ilc
grep -q "^sha256 = \"$LOG_SHA\"$" "$WORK/report1.toml" ||
  fail "report1 lacks the log hash"
grep -q "^sha256 = \"$(sha "$O1")\"$" "$WORK/report1.toml" ||
  fail "report1 lacks the offset hash"

# --- fel: from zero weights -----------------------------------------------
expect 0 init_weights init-weights --config "$WORK/learn.toml" \
  --output "$WORK/weights0.toml"
check_equal "weights0 trial" \
  "$(sed -n 's/^trial = //p' "$WORK/weights0.toml")" 0
expect 0 fel fel --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/trial0.csv" \
  --offset "$WORK/offset0.csv" --weights "$WORK/weights0.toml" \
  --output-weights "$WORK/weights1.toml" --output "$WORK/fel1.csv" \
  --report "$WORK/report_fel1.toml"
W1="$WORK/weights1.toml"
check_equal "weights1 trial" "$(sed -n 's/^trial = //p' "$W1")" 1
check_equal "weights1 parent" "$(toml parent_sha256 "$W1")" \
  "$(sha "$WORK/weights0.toml")"
check_equal "weights1 source log" "$(toml source_log_sha256 "$W1")" "$LOG_SHA"
check_equal "weights1 reference" "$(toml reference_sha256 "$W1")" "$REF_SIG"
F1="$WORK/fel1.csv"
check_equal "fel1 method" "$(header method "$F1")" fel
check_equal "fel1 trial" "$(header trial "$F1")" 1
check_equal "fel1 parent" "$(header parent_sha256 "$F1")" "$(sha "$W1")"
check_equal "fel1 source log" "$(header source_log_sha256 "$F1")" "$LOG_SHA"
check_equal "fel1 reference" "$(header reference_sha256 "$F1")" "$REF_SIG"
grep -q "lambda_max" "$WORK/fel.out"

# --- synth: the trained weights on the other reference --------------------
expect 0 synth synth --config "$WORK/learn.toml" --weights "$W1" \
  --follow-config "$WORK/other.toml" --output "$WORK/transfer.csv"
T="$WORK/transfer.csv"
check_equal "transfer method" "$(header method "$T")" fel
check_equal "transfer trial" "$(header trial "$T")" 1
check_equal "transfer parent" "$(header parent_sha256 "$T")" "$(sha "$W1")"
check_equal "transfer source log" "$(header source_log_sha256 "$T")" "$LOG_SHA"
check_equal "transfer reference" "$(header reference_sha256 "$T")" "$OTHER_SIG"

# --- refusals: exit 1, nothing written --------------------------------------
# an existing output is left untouched, and the report is never written
BEFORE=$(sha "$O1")
expect 1 clobber ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/trial0.csv" \
  --offset "$WORK/offset0.csv" --output "$O1" --report "$WORK/clobber.toml"
grep -q "refusing to overwrite" "$WORK/clobber.err"
check_equal "offset1 after the clobber attempt" "$(sha "$O1")" "$BEFORE"
test ! -e "$WORK/clobber.toml"
test ! -s "$WORK/clobber.out"

# a log of another reference: the other reference's zero offset is fine,
# the log is not a run of it
expect 1 other_log ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/other.toml" --log "$WORK/trial0.csv" \
  --offset "$WORK/other0.csv" --output "$WORK/other1.csv"
grep -q "follow log '$WORK/trial0.csv'" "$WORK/other_log.err"
grep -q "is not a run of this reference" "$WORK/other_log.err"
test ! -e "$WORK/other1.csv"

# the same reference file under another filter is another trajectory: the
# trial's offset is not its offset
sed -e 's/^filter = "none"/filter = "low-pass"/' "$WORK/follow.toml" \
  >"$WORK/filtered.toml"
grep -q '^filter = "low-pass"' "$WORK/filtered.toml" ||
  fail "filtered follow config not written"
expect 1 filtered ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/filtered.toml" --log "$WORK/trial0.csv" \
  --offset "$WORK/offset0.csv" --output "$WORK/filtered1.csv"
grep -q "does not match the reference being run" "$WORK/filtered.err"
grep -q "is not the reference signature of this follow config" \
  "$WORK/filtered.err"
test ! -e "$WORK/filtered1.csv"

# tamper replace FILE OUT: OUT is FILE with delta1 of data row 10 changed
tamper() {
  python3 - "$1" "$2" <<'PY'
import sys

with open(sys.argv[1]) as stream:
    lines = stream.read().split("\n")
header = next(i for i, line in enumerate(lines) if line.startswith("time_s,"))
fields = lines[header + 10].split(",")
fields[2] = repr(float(fields[2]) + 1e-3)
lines[header + 10] = ",".join(fields)
with open(sys.argv[2], "w") as stream:
    stream.write("\n".join(lines))
PY
}

# a tampered zero offset is no zero offset
tamper "$WORK/offset0.csv" "$WORK/tampered0.csv"
expect 1 tampered_zero ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/trial0.csv" \
  --offset "$WORK/tampered0.csv" --output "$WORK/tampered_next.csv"
grep -q "a zero offset must be all zeros" "$WORK/tampered_zero.err"
test ! -e "$WORK/tampered_next.csv"

# a tampered fel offset is not the synthesis of its weights
tamper "$F1" "$WORK/tampered_fel1.csv"
expect 1 tampered_fel fel --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/trial0.csv" \
  --offset "$WORK/tampered_fel1.csv" --weights "$W1" \
  --output-weights "$WORK/tampered_w2.toml" --output "$WORK/tampered_f2.csv"
grep -q "differs from the synthesis" "$WORK/tampered_fel.err"
test ! -e "$WORK/tampered_w2.toml"
test ! -e "$WORK/tampered_f2.csv"

# the trial-1 offset with the trial-0 log: not the offset that trial ran
expect 1 wrong_offset ilc --config "$WORK/learn.toml" \
  --follow-config "$WORK/follow.toml" --log "$WORK/trial0.csv" \
  --offset "$O1" --output "$WORK/wrong2.csv"
grep -q "the log is not a run with this offset" "$WORK/wrong_offset.err"
test ! -e "$WORK/wrong2.csv"

# the CLI
expect 0 help --help
grep -q "usage: x7_follow_learn COMMAND" "$WORK/help.out"
expect 0 ilc_help ilc --help
grep -q "usage: x7_follow_learn ilc" "$WORK/ilc_help.out"
expect 1 unknown ilc --port x
grep -q "unknown option: --port" "$WORK/unknown.err"

rm -rf "$WORK"
echo "follow learn smoke OK"
