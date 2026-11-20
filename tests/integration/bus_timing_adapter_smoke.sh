#!/usr/bin/env bash
# x7_bus_timing --timing adapter over dxl_emu, the pty emulator as it is: the
# refusals before any bus access, grid timing still the default, and adapter
# runs whose log and summary agree with each other row by row, through a
# shutdown just after a fit request and a stall mid-run; then timing_summary
# on a real adapter log against its grid twin. Functional only: the emulator
# answers each request after the pty's own delays, with no adapter cadence, so
# whether a run locks (falsely: its replies are tied to the requests) depends
# on this host's pty timing. The checks hold either way, and the outcome is
# printed, not asserted; the outcomes themselves are pinned by the unit tests
# (tests/unit/bus_timing_adapter_test.cpp) on synthetic replies.
set -eu

EMU="$1"
PROBE="$2"
INSPECT="$3"
# shellcheck source=tests/integration/scratch_dir.sh
. "$(dirname "$0")/scratch_dir.sh"
scratch_init "bus_timing_adapter_smoke" "${4:-}" || exit 2
ROOT=$RTCTRL_SCRATCH
LINK="$ROOT/tty"

fail() { echo "$1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }

# Refused before the port is opened: the port does not exist, so a choice
# that got past the checks would fail to open it (status 1), not be refused
# (status 2); no output file is created.
NOPORT="$ROOT/no-such-tty"
refuse() {  # refuse WHY ARGS...: status 2, WHY on stderr, no log
  local why="$1"
  shift
  local status=0
  "$PROBE" --port "$NOPORT" --profile current --log "$ROOT/refused.csv" "$@" \
    >"$ROOT/refused.log" 2>&1 || status=$?
  [ "$status" = 2 ] || fail "not refused before the bus (status $status): $*" "$ROOT/refused.log"
  grep -qF -- "$why" "$ROOT/refused.log" || fail "refused for another reason: $*" "$ROOT/refused.log"
  [ ! -e "$ROOT/refused.csv" ] || fail "a refused run created its log: $*"
}
ADAPTER=(--rate-hz 1000 --duration 4 --read-path bounded --timing adapter)
MOTION=(--read-instruction fast --feedback motion --aux-interval-ms 100)
refuse "--timing: grid or adapter" --rate-hz 1000 --duration 1 --timing bogus
refuse "need --timing adapter" --rate-hz 1000 --duration 1 --lead-alternation-us 20
refuse "need --timing adapter" --rate-hz 1000 --duration 1 --lead-us 700
refuse "need --timing adapter" --rate-hz 1000 --duration 1 --timing grid --acquisition-s 1
refuse "--rate-hz 1000 only" --rate-hz 500 --duration 4 --read-path bounded --timing adapter
refuse "production traffic only" "${ADAPTER[@]}" --confirm-writes
refuse "production traffic only" "${ADAPTER[@]}" --drain
refuse "needs --read-path bounded" --rate-hz 1000 --duration 4 --timing adapter
refuse "needs motion-only reads" "${ADAPTER[@]}" --lead-alternation-us 20
refuse "needs motion-only reads" "${ADAPTER[@]}" --read-instruction fast --feedback motion \
  --aux-interval-ms 1 --lead-alternation-us 20
refuse "--lead-alternation-us must be within" "${ADAPTER[@]}" "${MOTION[@]}" --lead-alternation-us 101
refuse "the lead less the alternation" "${ADAPTER[@]}" "${MOTION[@]}" --lead-us 990
refuse "--acquisition-s must be within" "${ADAPTER[@]}" --acquisition-s 5
refuse "before --duration" "${ADAPTER[@]}" --acquisition-s 3 --settling-s 1
refuse "--lead-us: not a number" "${ADAPTER[@]}" --lead-us abc
# The checks are the probe's, before anything else: the same choices with a
# supported combination get as far as the missing port.
status=0
"$PROBE" --port "$NOPORT" --profile current --log "$ROOT/open.csv" "${ADAPTER[@]}" "${MOTION[@]}" \
  --lead-alternation-us 20 >"$ROOT/open.log" 2>&1 || status=$?
[ "$status" = 1 ] || fail "a supported choice did not reach the port (status $status)" "$ROOT/open.log"
[ ! -e "$ROOT/open.csv" ] || fail "a run that could not open the port created its log"

mkdir -p "$ROOT"
"$EMU" --link "$LINK" --firmware 45 >"$ROOT/emu.log" 2>&1 &
EMU_PID=$!
PROBE_PID=
trap 'kill "$EMU_PID" 2>/dev/null || true
  if [ -n "$PROBE_PID" ]; then
    kill -CONT "$PROBE_PID" 2>/dev/null || true
    kill "$PROBE_PID" 2>/dev/null || true
  fi' EXIT
for _ in $(seq 1 50); do
  [ -e "$LINK" ] && break
  sleep 0.1
done
[ -e "$LINK" ] || fail "emulator link never appeared"
for id in 2 3 4 5 6 7 8 9; do  # the bounded read's bus profile
  "$INSPECT" --port "$LINK" write "$id" return_delay_time 0 >/dev/null
done

# Grid timing is the default: the grid log's header and no [adapter] section,
# the choice recorded.
"$PROBE" --port "$LINK" --rate-hz 1000 --profile current --duration 0.5 --read-path bounded \
  "${MOTION[@]}" --log "$ROOT/grid.csv" >"$ROOT/grid.log" 2>&1 || fail "grid run failed" "$ROOT/grid.log"
GRID_HEADER="cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,first_reply_ns,last_reply_ns,read_end_ns,limit_start_ns,bus_write_start_ns,write_return_ns,drain_return_ns,ack_return_ns,completion_upper_ns,cycle_end_ns,read_ok,write_ok,ack_ok,skipped_periods,read_len"
[ "$(head -1 "$ROOT/grid.csv")" = "$GRID_HEADER" ] || fail "the grid log's header changed"
grep -q '^timing = "grid"$' "$ROOT/grid.csv.summary.toml" || fail "the grid choice is not recorded"
if grep -q '^\[adapter' "$ROOT/grid.csv.summary.toml"; then fail "a grid run wrote an [adapter] section"; fi
grep -q '^timing: grid (the default)$' "$ROOT/grid.log" || fail "the grid choice is not printed" "$ROOT/grid.log"

# The log and the summary of an adapter run, against each other: every row's
# plan columns, its close's skips and the declared windows' counts; the fits
# and their hand-off. Prints what the run did.
CHECK="$ROOT/check_adapter.py"
cat >"$CHECK" <<'EOF_PY'
import csv
import sys
import tomllib

log, grid_header, deadline_s = sys.argv[1], sys.argv[2].split(","), float(sys.argv[3])
min_fits = int(sys.argv[4])  # fits the main thread must have completed (its requests come early enough)
with open(log + ".summary.toml", "rb") as f:
    s = tomllib.load(f)
with open(log, newline="") as f:
    reader = csv.DictReader(f)
    header = reader.fieldnames
    rows = [{k: int(v) for k, v in r.items()} for r in reader]
adapter_cols = ["target_index", "schedule", "lock_state", "transition", "predicted_completion_ns",
                "lead_offset_ns", "coupling_state", "observation", "recheck_skipped"]
assert header == grid_header + adapter_cols, header
r, a = s["result"], s["adapter"]
assert s["timing"] == "adapter", s["timing"]
assert s["log_complete"] and r["ring_drops"] == 0 and r["torque_off_after"]
assert len(rows) == r["cycles"], (len(rows), r["cycles"])
assert a["protocol_errors"] == 0
assert a["fitter"] == "main_thread" and a["stamp"] == "last_reply_ns"
assert a["publication_margin_us"] == 20 and a["fit_max_age_s"] == 0.5
L, W, D = a["lead_us"] * 1000, a["wake_to_read_us"] * 1000, a["lead_alternation_us"] * 1000
acq, settle = a["acquisition_s"] * 1e9, a["settling_s"] * 1e9

# The declared windows and the whole run, recomputed from the rows.
def counts(sel):
    c = dict(cycles=0, skipped_periods=0, overruns=0, consecutive_skips=0, referenced_cycles=0, grid_cycles=0)
    prev, prev_w = False, None
    for row in rows:
        t = row["scheduled_wake_ns"]
        w = 0 if t < acq else 1 if t < acq + settle else 2
        over = row["skipped_periods"] > 0
        if sel is None or w == sel:
            c["cycles"] += 1
            c["skipped_periods"] += row["skipped_periods"]
            c["overruns"] += over
            c["consecutive_skips"] += row["skipped_periods"] >= 2
            c["consecutive_skips"] += over and prev and (sel is None or prev_w == w)
            c["referenced_cycles" if row["schedule"] == 1 else "grid_cycles"] += 1
        prev, prev_w = over, w
    return c
for name, sel in (("acquisition", 0), ("settling", 1), ("locked", 2), ("whole_run", None)):
    assert counts(sel) == a[name], (name, counts(sel), a[name])
assert a["whole_run"]["skipped_periods"] == r["skipped_periods"] == sum(x["skipped_periods"] for x in rows)

# Each row: the plan its read executed (on the grid, the epoch's whole periods;
# on the lattice, its own wake from its predicted completion, one rounding),
# its close's skips with the recheck's among them, full-window reads at the
# base lead, motion-only reads alternating from + over each lock's reads.
expect, alternations, lattice = 1, 0, 0
for row in rows:
    assert row["recheck_skipped"] <= row["skipped_periods"], row
    if row["schedule"] == 0:
        assert row["scheduled_wake_ns"] % 1000000 == 0, row
        assert row["target_index"] == -1 and row["predicted_completion_ns"] == -1 and row["lead_offset_ns"] == 0
        expect = 1
        continue
    lattice += 1
    want = row["predicted_completion_ns"] - (L + row["lead_offset_ns"] + W)
    assert abs(row["scheduled_wake_ns"] - want) <= 1, row
    if row["read_len"] == 13:
        assert row["lead_offset_ns"] == 0, row
    elif D:
        assert row["lead_offset_ns"] == expect * D, row
        expect, alternations = -expect, alternations + 1
assert all(x["scheduled_wake_ns"] < deadline_s * 1e9 for x in rows)

# Each lattice row's observation, from its own stamp (last_reply_ns, from a
# complete read) against the completion its plan predicted: invalid without
# one, gated within the gate, else late or early (the prediction is rounded to
# the ns, so a row within 1 ns of the gate's edge is not judged).
D_REPLY, GATE = a["reply_delay_us"] * 1000, a["gate_us"] * 1000
judged = 0
for row in rows:
    if row["schedule"] != 1 or row["lock_state"] != 2:
        continue
    if not row["read_ok"] or row["last_reply_ns"] < 0:
        assert row["observation"] == 6, row
        continue
    e = row["last_reply_ns"] - row["predicted_completion_ns"] - D_REPLY
    if abs(abs(e) - GATE) <= 1:
        continue
    want = 3 if abs(e) <= GATE else 4 if e > 0 else 5
    if row["observation"] != 6:  # an out-of-order stamp is invalid
        assert row["observation"] == want, (row, e)
        judged += 1

# The fits: every request accounted for once; the main thread's services.
fits = s["adapter"].get("fit", [])
outcomes = [f["outcome"] for f in fits]
assert outcomes.count("requested") == a["fits_requested"]
for key, name in (("fits_applied", "applied"), ("fits_refused", "refused"), ("stale_results", "stale"),
                  ("abandoned_requests", "abandoned"), ("old_generation_results", "old_generation"),
                  ("queue_failures", "queue_failed")):
    assert outcomes.count(name) == a[key], (key, outcomes)
pending = 1 if a["fit_pending_at_end"] else 0
assert a["fits_requested"] == (a["fits_applied"] + a["fits_refused"] + a["stale_results"]
                               + a["abandoned_requests"] + pending), a
for f in fits:
    if f["outcome"] in ("applied", "refused"):
        assert 0 <= f["age_s"] <= 0.5 and f["request_to_result_s"] >= 0, f
    if f["outcome"] == "stale":
        assert f["age_s"] > 0.5, f
services = s["adapter"].get("fit_service", [])
assert len(services) == a["fits_completed"] >= min_fits, (len(services), a["fits_completed"], min_fits)
for sv in services:  # started_s is the call's start, which may precede its claim of the request
    assert sv["finished_s"] >= sv["window_end_s"] and sv["finished_s"] >= sv["started_s"], sv
# The acquisition's stamps: each stored row's last_reply_ns went to the fit.
stored = sum(1 for x in rows if x["observation"] == 1)
assert stored == a["exchange_appended"], (stored, a["exchange_appended"])
assert all(x["last_reply_ns"] >= 0 and x["read_ok"] for x in rows if x["observation"] == 1)
# A lock state the published plans went through, in order.
states = [x["lock_state"] for x in rows]
assert states[0] == 0
for p, q in zip(states, states[1:]):
    assert (p, q) in {(0, 0), (0, 1), (1, 1), (1, 0), (1, 2), (2, 2), (2, 0)}, (p, q)
assert a["locks"] == sum(1 for p, q in zip(states, states[1:]) if (p, q) == (1, 2))
coupling = s["adapter"].get("coupling")
assert (coupling is not None) == bool(D)
print(f"{log}: {len(rows)} cycles, {r['skipped_periods']} skipped; fits {outcomes}; "
      f"locks {a['locks']}, losses {a['losses']}, {lattice} lattice cycles ({judged} observations checked), "
      f"{alternations} alternated reads; "
      f"coupling {coupling['state_at_end'] if coupling else 'off'}")
EOF_PY

# Adapter timing with the alternation: 1 s acquisition, 0.5 s settling.
"$PROBE" --port "$LINK" --profile current --log "$ROOT/adapter.csv" "${ADAPTER[@]}" "${MOTION[@]}" \
  --acquisition-s 1 --settling-s 0.5 --lead-alternation-us 20 >"$ROOT/adapter.log" 2>&1 ||
  fail "adapter run failed" "$ROOT/adapter.log"
python3 "$CHECK" "$ROOT/adapter.csv" "$GRID_HEADER" 4 1 || fail "adapter run inconsistent" "$ROOT/adapter.log"
grep -q '^adapter timing: acquisition 1.000 s on the 1 ms grid' "$ROOT/adapter.log"
grep -q '^lead alternation (observational): coupling at the end ' "$ROOT/adapter.log"

# Without the alternation, and with the full window on every read.
"$PROBE" --port "$LINK" --profile current --log "$ROOT/adapter-full.csv" "${ADAPTER[@]}" \
  --acquisition-s 1 --settling-s 0.5 >"$ROOT/adapter-full.log" 2>&1 ||
  fail "adapter run (full reads) failed" "$ROOT/adapter-full.log"
python3 "$CHECK" "$ROOT/adapter-full.csv" "$GRID_HEADER" 4 1 || fail "adapter run (full reads) inconsistent"
grep -q '^lead_alternation_us = 0$' "$ROOT/adapter-full.csv.summary.toml"

# Shutdown just after the request: the deadline 5 ms after the acquisition
# window's end. Whatever the fit's state then (queued, being fitted, or
# taken), the run ends with complete records, every request accounted for.
"$PROBE" --port "$LINK" --profile current --log "$ROOT/shutdown.csv" --rate-hz 1000 --duration 1.005 \
  --read-path bounded --timing adapter --acquisition-s 1 --settling-s 0 "${MOTION[@]}" \
  >"$ROOT/shutdown.log" 2>&1 || fail "shutdown run failed" "$ROOT/shutdown.log"
python3 "$CHECK" "$ROOT/shutdown.csv" "$GRID_HEADER" 1.005 0 || fail "shutdown run inconsistent"
grep -q '^fits_requested = 1$' "$ROOT/shutdown.csv.summary.toml" ||
  fail "the shutdown run made no request" "$ROOT/shutdown.csv.summary.toml"

# A stall mid-run (the process stopped for 0.3 s during the acquisition): the
# close after it counts the slots passed once, and everything still agrees.
"$PROBE" --port "$LINK" --profile current --log "$ROOT/stall.csv" --rate-hz 1000 --duration 2.5 \
  --read-path bounded --timing adapter --acquisition-s 1.5 --settling-s 0.5 "${MOTION[@]}" \
  --lead-alternation-us 20 >"$ROOT/stall.log" 2>&1 &
PROBE_PID=$!
for _ in $(seq 1 200); do
  [ -e "$ROOT/stall.csv" ] && [ "$(wc -l <"$ROOT/stall.csv")" -ge 300 ] && break
  sleep 0.02
done
kill -STOP "$PROBE_PID"
sleep 0.3
kill -CONT "$PROBE_PID"
wait "$PROBE_PID" || fail "stalled adapter run failed" "$ROOT/stall.log"
PROBE_PID=
python3 "$CHECK" "$ROOT/stall.csv" "$GRID_HEADER" 2.5 1 || fail "stalled adapter run inconsistent"
python3 - "$ROOT/stall.csv" <<'EOF_PY'
import csv
import sys
rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(open(sys.argv[1]))]
assert max(r["skipped_periods"] for r in rows) >= 200, "the stall left no skip"
EOF_PY

# The readers: timing_summary reads the real adapter log as it reads the same
# rows without the adapter columns (the reader regressions cover the others).
cut -d, -f1-20 "$ROOT/adapter.csv" >"$ROOT/adapter-grid-twin.csv"
cp "$ROOT/adapter.csv.summary.toml" "$ROOT/adapter-grid-twin.csv.summary.toml"
for log in adapter adapter-grid-twin; do
  status=0
  uv run --project tools python tools/timing_summary.py "$ROOT/$log.csv" --rate-hz 1000 --loaded --check \
    >"$ROOT/$log.summary.txt" 2>&1 || status=$?
  echo "status $status" >>"$ROOT/$log.summary.txt"
  sed -i "s|$ROOT/$log.csv|LOG|g" "$ROOT/$log.summary.txt"
done
grep -q 'T1' "$ROOT/adapter.summary.txt" || fail "timing_summary did not run" "$ROOT/adapter.summary.txt"
cmp "$ROOT/adapter.summary.txt" "$ROOT/adapter-grid-twin.summary.txt" ||
  fail "timing_summary read the adapter log differently" "$ROOT/adapter.summary.txt"

for id in 2 3 4 5 6 7 8 9; do
  test "$("$INSPECT" --port "$LINK" read "$id" torque_enable)" = "0"
done
scratch_release
echo "bus timing adapter smoke OK"
