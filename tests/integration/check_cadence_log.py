"""An x7_bus_timing adapter run against the emulator's own cadence record.

Usage: check_cadence_log.py LOG EMU_CSV [expectations]

LOG is the probe's log (its summary next to it); EMU_CSV is dxl_emu
--cadence-log's record of every reply it held. Times there are
CLOCK_MONOTONIC ns; the probe's are its epoch's, at monotonic_origin_ns.

Always checked:
- the record itself: each reply ready at or after its request, carried by a
  completion at or after that (independent) or a fixed delay after it
  (request-tied), due at or after its release, written in full (all its bytes
  accepted by the pty, no write error) at or after due (every reply: each
  was awaited before the emulator stopped), the replies due and
  written in order, and sent on time: their median lateness at the write
  (written less due, the emulator's own wake) within 30 us (2.6 to 19 us
  with its 1 ns timer slack on the test host, about 54 us with the default
  50 us);
- each probe read with a reply: exactly one grouped-read record whose
  request arrived while the read was open, and the read's last bytes seen no
  earlier than that reply was due (no reply is seen before it was sent);
- each lock supervision ended, against its own rows' observations: lost for
  no gated stamp after exactly 64 ungated cycles in a row, or outside the
  gate when more than a quarter of its last 256 observed cycles fell outside
  (at the 256th, or as the count reached 65), neither rule met before.

The expectations (flags) are the scenario's outcome: the locks, their
losses and causes, the coupling result, the observation counts, and on an
independent cadence whether every lock was a true lock: each locked read
carried by the completion its plan targeted (one constant offset per lock,
its most common, less the missed completions passed over), and each lock's
predictions at its carriers' instants. A read carried later than its target
is a late request, not an off-target one, when its request was ready only
after the targeted completion's instant (the pty can hold a request for a
millisecond or more): it must be observed late, and late requests must stay
under 1% of the locked reads.
"""

import argparse
import bisect
import csv
import itertools
import sys

import tomllib

GROUPED_READS = {0x82, 0x8A}  # Sync Read, Fast Sync Read
OBSERVATIONS = {"stored": 1, "dropped": 2, "gated": 3, "late": 4, "early": 5, "invalid": 6}


def quantile(values, p):
    if not values:
        return float("nan")
    v = sorted(values)
    return v[int(p * (len(v) - 1))]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("emu_csv")
    ap.add_argument("--mode", choices=["independent", "request-tied"], required=True)
    ap.add_argument("--locks", type=int)
    ap.add_argument("--min-locks", type=int)
    ap.add_argument("--losses", type=int)
    ap.add_argument("--loss-cause", help="every ended lock's cause, or one of a comma-separated list")
    ap.add_argument("--held-at-end", action="store_true", help="the last lock held at the run's end")
    ap.add_argument("--true-lock", action="store_true", help="every lock's reads carried by its targets")
    ap.add_argument("--coupling-end", help="[adapter.coupling].state_at_end")
    ap.add_argument("--first-declaration", help="the first lock's first coupling declaration (none: no declaration)")
    ap.add_argument("--min-referenced", type=int, help="the first lock's referenced cycles, at least")
    ap.add_argument("--late-on-cadence", action="store_true",
                    help="every locked read on a delayed or missed completion observed late")
    ap.add_argument("--min-on-delayed", type=int, default=0, help="locked reads on delayed completions, at least")
    ap.add_argument("--min-on-missed", type=int, default=0, help="locked reads past missed completions, at least")
    ap.add_argument("--observed", action="append", default=[], metavar="KIND=N",
                    help="exactly N lattice rows observed KIND")
    ap.add_argument("--min-observed", action="append", default=[], metavar="KIND=N")
    ap.add_argument("--no-lock", action="store_true", help="no fit applied, every row on the grid")
    args = ap.parse_args()

    with open(args.log + ".summary.toml", "rb") as f:
        s = tomllib.load(f)
    with open(args.log, newline="") as f:
        rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(f)]
    with open(args.emu_csv, newline="") as f:
        recs = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(f)]
    origin = s["result"]["monotonic_origin_ns"]
    a = s["adapter"]

    # The record itself.
    last_due = last_written = None
    for r in recs:
        assert r["ready_ns"] >= r["arrival_ns"], r
        if args.mode == "independent":
            assert r["index"] >= 0 and r["completion_ns"] >= r["ready_ns"], r
            assert r["release_ns"] >= r["completion_ns"], r
        else:
            assert r["index"] == -1 and r["completion_ns"] == -1, r
            assert r["release_ns"] > r["arrival_ns"], r
        assert r["due_ns"] >= r["release_ns"], r
        assert r["written_bytes"] == r["bytes"] and r["write_error"] == 0, ("a reply not written in full", r)
        assert r["written_ns"] >= r["due_ns"], ("a reply not written, or before it was due", r)
        if last_due is not None:
            assert r["due_ns"] >= last_due, ("replies due out of order", r)
            assert r["written_ns"] >= last_written, ("replies written out of order", r)
        last_due, last_written = r["due_ns"], r["written_ns"]
    reads = [r for r in recs if r["instruction"] in GROUPED_READS]
    arrivals = [r["arrival_ns"] for r in reads]
    assert arrivals == sorted(arrivals)

    # Each probe read with a reply against the one reply its request got.
    delivery, matched = [], {}
    for i, row in enumerate(rows):
        if not row["read_ok"] or row["last_reply_ns"] < 0:
            continue
        lo = origin + row["read_start_ns"]
        hi = origin + row["last_reply_ns"]
        j0, j1 = bisect.bisect_left(arrivals, lo), bisect.bisect_right(arrivals, hi)
        assert j1 - j0 == 1, ("not one request in the read", row["cycle"], j1 - j0)
        rec = reads[j0]
        assert hi >= rec["due_ns"], ("a reply seen before it was due", row["cycle"], hi - rec["due_ns"])
        delivery.append(hi - rec["due_ns"])
        matched[i] = rec
    assert matched, "no read matched"
    written_late = [r["written_ns"] - r["due_ns"] for r in reads]
    assert quantile(written_late, 0.5) <= 30000, ("the emulator sends late", quantile(written_late, 0.5))

    # The adapter's period on the host clock, from the record (independent).
    steps = [(b["completion_ns"] - a_["completion_ns"]) / (b["index"] - a_["index"])
             for a_, b in itertools.pairwise(reads) if b["index"] > a_["index"]]
    period = quantile(steps, 0.5) if steps else float("nan")

    # The locks' reads against their carriers: the index each plan targeted,
    # and (independent) the time each plan predicted against the carrier's
    # instant, for the reads on plain completions.
    lock_reads, current = [], None
    for i, row in enumerate(rows):
        if row["schedule"] != 1 or row["lock_state"] != 2:
            current = None
            continue
        if current is None:
            current = []
            lock_reads.append(current)
        if i in matched:
            current.append((row, matched[i]))
    locks_seen = sum(1 for lr in lock_reads if lr)
    off_target = locked_reads = late_wrong = on_cadence = on_delayed = on_missed = late_requests = 0
    predicted_minus_instant = []  # per lock
    for lr in lock_reads:
        if not lr:
            continue
        offsets = [rec["index"] - rec["missed"] - row["target_index"] for row, rec in lr]
        offset = max(set(offsets), key=offsets.count)
        predicted_minus_instant.append([])
        for (row, rec), o in zip(lr, offsets):
            locked_reads += 1
            on_delayed += rec["delayed"]
            on_missed += rec["missed"] > 0
            late = row["observation"] in (OBSERVATIONS["late"], OBSERVATIONS["invalid"])
            if o != offset:
                targeted = rec["completion_ns"] - (o - offset) * period  # the targeted completion's instant
                if args.mode == "independent" and o > offset and rec["ready_ns"] > targeted and late:
                    late_requests += 1
                else:
                    off_target += 1
                continue
            if args.mode == "independent" and (rec["delayed"] or rec["missed"] > 0):
                on_cadence += 1
                if not late:
                    late_wrong += 1
            elif args.mode == "independent":
                predicted_minus_instant[-1].append(origin + row["predicted_completion_ns"] - rec["completion_ns"])
    medians = [quantile(d, 0.5) for d in predicted_minus_instant if d]

    lattice_obs = {k: sum(1 for x in rows if x["schedule"] == 1 and x["observation"] == v)
                   for k, v in OBSERVATIONS.items()}
    lock_list = a.get("lock", [])

    # Each lock's supervised observations (its locked lattice rows, in order),
    # against the rule that ended it.
    lock_obs, current = [], None
    for row in rows:
        if row["schedule"] == 1 and row["lock_state"] == 2:
            if current is None:
                current = []
                lock_obs.append(current)
            current.append(row["observation"])
        else:
            current = None
    assert len(lock_obs) == len(lock_list), (len(lock_obs), len(lock_list))
    gated, window, outside_max, run_max = OBSERVATIONS["gated"], 256, 64, 64
    for lk, obs in zip(lock_list, lock_obs):
        assert OBSERVATIONS["invalid"] not in obs, ("an invalid stamp in a lock", lk)
        run, best_run, outside_at_end = 0, 0, None
        for j, o in enumerate(obs):
            run = 0 if o == gated else run + 1
            last = obs[max(0, j - window + 1): j + 1]
            outside = sum(1 for x in last if x != gated) if len(last) == window else None
            if j < len(obs) - 1:  # no rule met before the lock's last cycle
                assert run < run_max and (outside is None or outside <= outside_max), ("held past a rule", lk, j)
            else:
                best_run, outside_at_end = run, outside
        if lk["end"] == "no_gated_stamp":
            assert best_run == run_max, ("no-gated-stamp loss after", best_run, lk)
        elif lk["end"] == "outside_gate":
            # The rule applies once the window is full: at that cycle any count
            # above a quarter ends the lock; later, the count crosses by one.
            assert best_run < run_max and outside_at_end is not None and outside_at_end > outside_max and (
                len(obs) == window or outside_at_end == outside_max + 1), ("outside-gate loss", lk, outside_at_end)
        elif lk["end"] == "run_end":
            assert best_run < run_max and (outside_at_end is None or outside_at_end <= outside_max), lk
    coupling = a.get("coupling")
    print(f"{args.log}: {len(rows)} cycles, {len(matched)} reads matched to {len(reads)} held replies; "
          f"locks {a['locks']}, losses {a['losses']}; {locked_reads} locked reads in {locks_seen} locks, "
          f"{off_target} off target, {late_requests} late requests, {on_cadence} on delayed or missed "
          f"completions ({on_delayed} delayed, "
          f"{on_missed} past missed); predicted less the "
          f"carrier's instant, each lock's median {[round(m / 1e3, 1) for m in medians]} us; observed "
          f"{ {k: v for k, v in lattice_obs.items() if v} }; coupling "
          f"{coupling['state_at_end'] if coupling else 'off'}; delivery after due p50 "
          f"{quantile(delivery, 0.5) / 1e3:.1f} p99 {quantile(delivery, 0.99) / 1e3:.1f} max "
          f"{max(delivery) / 1e3:.1f} us, emulator written after due p50 {quantile(written_late, 0.5) / 1e3:.1f} "
          f"p99 {quantile(written_late, 0.99) / 1e3:.1f} us")

    def fail(why):
        print(f"{args.log}: {why}", file=sys.stderr)
        sys.exit(1)

    if args.locks is not None and a["locks"] != args.locks:
        fail(f"locks {a['locks']}, expected {args.locks}")
    if args.min_locks is not None and a["locks"] < args.min_locks:
        fail(f"locks {a['locks']}, expected at least {args.min_locks}")
    if args.losses is not None and a["losses"] != args.losses:
        fail(f"losses {a['losses']}, expected {args.losses}")
    if args.loss_cause:
        causes = [lk["end"] for lk in lock_list if lk["end"] != "run_end"]
        if not causes or any(c not in args.loss_cause.split(",") for c in causes):
            fail(f"loss causes {causes}, expected {args.loss_cause}")
    if args.held_at_end and (not lock_list or lock_list[-1]["end"] != "run_end"):
        fail("the last lock did not hold to the run's end")
    if args.true_lock:
        if args.mode != "independent" or locks_seen == 0 or locks_seen != a["locks"]:
            fail(f"true locks need an independent cadence and locks on the lattice ({locks_seen} seen)")
        if off_target:
            fail(f"{off_target} locked reads not carried by their targeted completion")
        if late_requests * 100 > locked_reads:
            fail(f"{late_requests} late requests in {locked_reads} locked reads, more than 1%")
        gate = a["gate_us"] * 1000
        if len(medians) != locks_seen or any(abs(m) > gate for m in medians):
            fail(f"a lock's predictions not at its carriers' instants (medians {medians} ns, gate {gate} ns)")
    if args.coupling_end and (coupling is None or coupling["state_at_end"] != args.coupling_end):
        fail(f"coupling at the end {coupling and coupling['state_at_end']}, expected {args.coupling_end}")
    first = lock_list[0].get("first_declaration", "none") if lock_list else None
    if args.first_declaration and first != args.first_declaration:
        fail(f"the first lock's first declaration {first}, expected {args.first_declaration}")
    if args.min_referenced is not None and (not lock_list or lock_list[0]["referenced_cycles"] < args.min_referenced):
        fail(f"the first lock's referenced cycles {lock_list and lock_list[0]['referenced_cycles']}")
    if on_delayed < args.min_on_delayed or on_missed < args.min_on_missed:
        fail(f"{on_delayed} locked reads on delayed and {on_missed} past missed completions, expected at least "
             f"{args.min_on_delayed} and {args.min_on_missed}")
    if args.late_on_cadence and (on_cadence == 0 or late_wrong):
        fail(f"{late_wrong} of {on_cadence} reads on delayed or missed completions not observed late")
    for spec, exact in [(x, True) for x in args.observed] + [(x, False) for x in args.min_observed]:
        kind, n = spec.split("=")
        got = lattice_obs[kind]
        if (exact and got != int(n)) or (not exact and got < int(n)):
            fail(f"{got} lattice rows observed {kind}, expected {'' if exact else 'at least '}{n}")
    if args.no_lock:
        if a["locks"] != 0 or a["fits_applied"] != 0 or any(x["schedule"] != 0 for x in rows):
            fail("a lock or a lattice row")
        if a["fits_refused"] < 1:
            fail("no fit was refused")
    return 0


if __name__ == "__main__":
    sys.exit(main())
