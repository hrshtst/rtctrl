#!/usr/bin/env bash
# End-to-end x7_float --log validation over the dxl_emu pty, two
# phases (GRAVITY_CALIBRATION_PLAN M-GC1/M-GC2):
#  1. default config, no marker: the position-hold -> preloaded switch
#     startup, the log contract at scale 1.0, AND the marker-timeout
#     abort path (exit nonzero at ~8 s with the void-attempt message —
#     short durations are rejected outright, so this IS the no-marker
#     behavior);
#  2. vendor-scale config with a piped release marker at ~1 s: the run
#     must self-terminate 5 s after the marker, and every applied
#     torque must equal scale * tau_request — the end-to-end proof the
#     calibration is applied exactly once through the RealArm path.
# Run from the repo root (models/ and config/ by relative path).
set -eu
EMU="$1"
FLOAT="$2"
OUT="$3"
mkdir -p "$OUT"
LINK="$OUT/ttyDXL"
rm -f "$LINK" "$OUT/float.csv" "$OUT/float_vendor.csv"
"$EMU" --link "$LINK" &
EMU_PID=$!
trap 'kill "$EMU_PID" 2>/dev/null || true' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || { echo "emulator link never appeared"; exit 1; }

# Phase 1: durations below deadline+window are now rejected, so the
# no-marker path IS the marker-timeout abort — exercise it end-to-end:
# exit nonzero, the void-attempt message, and a valid log up to ~8 s.
set +e
PHASE1=$("$FLOAT" --port "$LINK" --log "$OUT/float.csv" 15 \
  < /dev/null 2>&1)
RC=$?
set -e
if [ "$RC" -eq 0 ]; then
  echo "expected the marker-timeout abort, got success"
  exit 1
fi
echo "$PHASE1" | grep -q "NO release marker" || {
  echo "missing the marker-timeout message:"; echo "$PHASE1"; exit 1;
}
python3 "$(dirname "$0")/check_float_log.py" "$OUT/float.csv"

# Phase 2 pipes the marker LATE (~5.5-6 s of run time; a full 2 s
# clear of the 8 s deadline so parallel-suite load jitter cannot tip
# it into a timeout) — the late-marker regression: the evaluation
# window must still complete in full before the outer deadline (the
# enforced 15 s minimum guarantees the margin; review finding).
(sleep 6; echo) | "$FLOAT" --config config/crane_x7_vendor_scale.toml \
  --port "$LINK" --log "$OUT/float_vendor.csv" 15
python3 "$(dirname "$0")/check_float_log.py" --vendor \
  "$OUT/float_vendor.csv"

# Phase 3: feel-check mode — marker required as ever, but the session
# runs to the OUTER deadline; the log must self-mark run_mode:
# feel-check (never acceptance evidence). The two extra ENTERs after
# the marker are OPERATOR EVENT MARKS (reviewer-directed notch
# instrumentation) — they must land as operator_event rows strictly
# after the release marker (the checker enforces ordering; the count
# is asserted below).
(sleep 1; echo; sleep 3; echo; sleep 2; echo) | \
  "$FLOAT" --config config/crane_x7_vendor_scale.toml \
  --port "$LINK" --log "$OUT/float_feel.csv" --feel 15
python3 "$(dirname "$0")/check_float_log.py" --feel \
  "$OUT/float_feel.csv"
python3 - "$OUT/float_feel.csv" <<'EOF'
import csv
import sys
rows = list(csv.DictReader(
    ln for ln in open(sys.argv[1]) if not ln.startswith("#")))
events = [float(r["t"]) for r in rows if r["operator_event"] == "1"]
assert len(events) == 2, f"expected 2 operator event marks: {events}"
t_marker = next(float(r["t"]) for r in rows if r["released"] == "1")
assert all(t > t_marker for t in events), (t_marker, events)
print(f"operator event marks at {events} (marker {t_marker})")
EOF

# Negative: an event mark ON the release-transition row must be
# REJECTED (review finding: released == 1 alone admitted it — the
# ordering is strict, marker first, events on later rows only).
python3 - "$OUT/float_feel.csv" "$OUT/float_feel_bad.csv" <<'EOF'
import csv
import sys
lines = open(sys.argv[1]).read().splitlines()
head = [ln for ln in lines if ln.startswith("#")]
rows = list(csv.DictReader(
    ln for ln in lines if not ln.startswith("#")))
next(r for r in rows if r["released"] == "1")["operator_event"] = "1"
with open(sys.argv[2], "w", newline="") as f:
    f.write("\n".join(head) + "\n")
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)
EOF
if python3 "$(dirname "$0")/check_float_log.py" --feel \
    "$OUT/float_feel_bad.csv" 2>/dev/null; then
  echo "checker admitted an event mark on the release-transition row"
  exit 1
fi
echo "transition-row event mark correctly rejected"
