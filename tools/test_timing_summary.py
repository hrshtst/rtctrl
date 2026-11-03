#!/usr/bin/env python3
"""Regressions for timing_summary.py, run under ctest.

Synthetic probe and app timing logs with known stamps: the measured
intervals, the calibration distribution, the envelope and estimate
checks, and every criterion's pass and fail are pinned, including the
classes the report attaches to them. T3 and T4 also need the run summary:
without it, or when it cannot support a verdict, they are INDETERMINATE,
never PASS.
"""

import argparse
import importlib.util
import pathlib
import sys
import tempfile

import numpy as np

spec = importlib.util.spec_from_file_location("ts", "tools/timing_summary.py")
assert spec is not None and spec.loader is not None
ts = importlib.util.module_from_spec(spec)
sys.modules["ts"] = ts
spec.loader.exec_module(ts)

PERIOD_NS = 2_000_000  # 500 Hz

PROBE_HEADER = (
    "cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,"
    "first_reply_ns,last_reply_ns,read_end_ns,limit_start_ns,"
    "bus_write_start_ns,write_return_ns,drain_return_ns,ack_return_ns,"
    "completion_upper_ns,cycle_end_ns,read_ok,write_ok,ack_ok,"
    "skipped_periods")
APP_HEADER = (
    "cycle,segment,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,"
    "first_reply_ns,last_reply_ns,read_end_ns,window_open_ns,"
    "controller_wake_ns,submit_ns,limit_start_ns,bus_write_start_ns,"
    "write_return_ns,cycle_end_ns,feedback_seq,applied_target_seq,"
    "applied_source_feedback_seq,skipped_periods,read_ok,write_ok,"
    "deadline_miss,command_kind")


def probe_rows(n, lateness_ns=20_000, skip_at=None, read_fail_at=None):
    rows = []
    for k in range(1, n + 1):
        sched = (k - 1) * PERIOD_NS
        wake = sched + lateness_ns
        read_start = wake + 1_000
        read_end = read_start + 700_000
        limit = read_end + 10_000
        bus = limit + 2_000
        ret = bus + 30_000
        ack = ret + 300_000
        upper = ack - 100_000
        end = ack + 5_000
        rows.append([k, sched, wake, read_start, read_start + 20_000,
                     read_start + 500_000, read_end - 3_000, read_end, limit,
                     bus, ret, -1, ack, upper, end,
                     0 if k == read_fail_at else 1, 1, 1,
                     1 if k == skip_at else 0])
    return PROBE_HEADER, rows


def app_rows(n, fallback_at=(), restart_at=None, restart_gap_ns=0,
             all_fallbacks=False):
    """fallback_at: cycles whose window closed unanswered while the
    controller was armed (deadline misses). Cycle 1 is a fallback before
    the controller's first submission, which is no miss. restart_at: the
    first cycle of a second thread run (segment 2), which wakes
    restart_gap_ns later than the period grid."""
    rows = []
    for k in range(1, n + 1):
        second = restart_at is not None and k >= restart_at
        sched = (k - 1) * PERIOD_NS + (restart_gap_ns if second else 0)
        wake = sched + 10_000
        read_start = wake + 1_000
        read_end = read_start + 600_000
        window = read_end + 1_000
        ctrl = window + 50_000
        fallback = k in fallback_at or all_fallbacks
        submit = -1 if fallback else ctrl + 10_000
        limit = window + 1_400_000 if fallback else submit + 20_000
        bus = limit + 2_000
        ret = bus + 30_000
        end = ret + 5_000
        kind = "fallback" if fallback or k == 1 else "fresh"
        miss = 1 if k in fallback_at else 0
        rows.append([k, 2 if second else 1, sched, wake, read_start,
                     read_start + 20_000,
                     read_start + 400_000, read_end - 3_000, read_end, window,
                     ctrl, submit, limit, bus, ret, end, k + 1, k, k + 1, 0, 1,
                     1, miss, kind])
    return APP_HEADER, rows


def write(header, rows):
    handle = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False)
    with handle:
        handle.write(header + "\n")
        for row in rows:
            handle.write(",".join(str(v) for v in row) + "\n")
    return pathlib.Path(handle.name)


def probe_run(cycles, log_complete=True, ring_drops=0, late_wake_s=None,
              version=2):
    """late_wake_s: the lateness of a wake past the deadline that ended
    the run (no row), or None when the schedule ended it. version 1 is the
    probe whose acknowledgement compared goal values; version 2 reads
    Model Number."""
    acks = {"ack_register": "model_number"} if version >= 2 else {}
    return {"format": "rtctrl-bus-timing", "version": version, **acks,
            "kind": "production-traffic", "log_complete": log_complete,
            "result": {"started": True, "error": "", "cycles": cycles,
                       "ring_drops": ring_drops, "elapsed_s": 2e-3 * cycles,
                       "late_wake_stop": late_wake_s is not None,
                       "late_wake_s": 0.0 if late_wake_s is None else 2.0,
                       "late_wake_lateness_s": late_wake_s or 0.0}}


def app_run(cycles, end=None, misses=0, stale=0, status="success",
            log_complete=True, timing_drops=0):
    """end: the cycle count when the controller finished (default: all)."""
    def counters(n, deadline_misses):
        return {"cycles": n, "overruns": 0, "skipped_periods": 0,
                "read_failures": 0, "write_failures": 0,
                "stale_submissions": stale,
                "controller_deadline_misses": deadline_misses,
                "timing_drops": timing_drops}
    return {"format": "rtctrl-cycle-timing-run", "app": "x7_follow",
            "status": status, "successful": status == "success",
            "log_complete": log_complete, "escalated": False,
            "rate_hz": 500.0,
            "controller_end": counters(cycles if end is None else end, misses),
            "final": counters(cycles, misses)}


def defaults(**overrides):
    args = argparse.Namespace(period_tolerance=0.001, loaded_skips_per_10k=1.0,
                              lateness_p999=0.10, lateness_max=0.5,
                              host_work_p999=0.70, estimate_p999=0.90)
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def verdicts(results):
    return {name: (cls, verdict) for name, cls, verdict, _ in results}


# A clean probe log: exact intervals, calibration and criteria.
path = write(*probe_rows(1000))
kind, data = ts.load(path)
assert kind == "probe"
s = ts.summarize(kind, data, 500.0, delta_env_s=0.4e-3, delta_q_s=0.3e-3,
                 run=probe_run(1000))
assert s["cycles"] == 1000 and s["record_gaps"] == 0
assert abs(s["mean_period_s"] - 2e-3) < 1e-12
m = s["measured"]
assert abs(m["host_transaction"]["max"] - 700e-6) < 1e-12
assert abs(m["wake_lateness"]["p999"] - 20e-6) < 1e-12
# host work runs read_start -> cycle_end: read, limit, write, ack, tail.
assert abs(m["host_work"]["max"] - (700e-6 + 10e-6 + 2e-6 + 30e-6 + 300e-6
                                    + 5e-6)) < 1e-12
# C+ - bus_write_start = (ack - 100 us) - bus = 30 us + 300 us - 100 us.
assert abs(s["calibration"]["max"] - 230e-6) < 1e-12
assert abs(s["calibration"]["delta_env_candidate"] - 1.25 * 230e-6) < 1e-12
# read_start -> bus_write_start = 712 us; + delta_env 400 us.
assert abs(s["envelope_completion_max_s"] - 1.112e-3) < 1e-12
assert abs(s["estimated_completion_p999_s"] - 1.012e-3) < 1e-12
v = verdicts(ts.check(s, False, defaults()))
assert v["T1"] == ("measured", "PASS")
assert v["T2"] == ("measured", "PASS")
assert v["T3"] == ("measured", "PASS")
assert v["T4"] == ("measured", "n/a")  # a probe has no controller
assert v["T5"] == ("measured", "PASS")
assert v["T6a"] == ("measured", "PASS")
assert v["T6b"] == ("envelope", "PASS")
assert v["T7"] == ("estimated", "PASS")
assert ts.exit_status(ts.check(s, False, defaults())) == 0
# Completeness needs the probe's summary: absent, T3 cannot pass.
s = ts.summarize(kind, data, 500.0, None, None)
results = ts.check(s, False, defaults())
assert verdicts(results)["T3"] == ("measured", "INDETERMINATE")
# Goal-comparing acknowledgements (version 1), or none known: each kept
# row is still described, but no envelope candidate is offered.
for run in (probe_run(1000, version=1), None):
    s = ts.summarize(kind, data, 500.0, None, None, run=run)
    assert abs(s["calibration"]["max"] - 230e-6) < 1e-12
    assert "delta_env_candidate" not in s["calibration"]
assert ts.exit_status(results) == 1
for run in (probe_run(1000, log_complete=False), probe_run(1000, ring_drops=3),
            probe_run(999)):
    s = ts.summarize(kind, data, 500.0, None, None, run=run)
    assert verdicts(ts.check(s, False, defaults()))["T3"][1] == "FAIL", run
# A wake past the deadline ends the run with no row, but its lateness is
# still judged: 0.17 s late fails T5; a punctual one changes nothing.
s = ts.summarize(kind, data, 500.0, None, None,
                 run=probe_run(1000, late_wake_s=0.17))
assert s["late_wake_lateness_s"] == 0.17
assert verdicts(ts.check(s, False, defaults()))["T5"] == ("measured", "FAIL")
s = ts.summarize(kind, data, 500.0, None, None,
                 run=probe_run(1000, late_wake_s=20e-6))
assert verdicts(ts.check(s, False, defaults()))["T5"] == ("measured", "PASS")
path.unlink()

# Failures: a skip, a read failure, late wakes and a tight envelope.
path = write(*probe_rows(1000, lateness_ns=600_000, skip_at=500,
                         read_fail_at=10))
kind, data = ts.load(path)
s = ts.summarize(kind, data, 500.0, delta_env_s=1.5e-3, delta_q_s=None)
v = verdicts(ts.check(s, False, defaults()))
assert v["T2"][1] == "FAIL"  # idle allows no skip
assert v["T3"][1] == "FAIL"  # an observed failure fails, summary or not
assert v["T5"][1] == "FAIL"  # 0.6 ms > 0.1 T and > 0.5 T
assert v["T6b"][1] == "FAIL"
assert v["T7"] == ("estimated", "n/a")
# Loaded runs allow 1 skip per 1e4 cycles: 1000 cycles allow 0.1, so the
# single skip still fails; 20000 would allow it.
assert ts.check(s, True, defaults())[1][2] == "FAIL"
path.unlink()


def app_t4(rows, run):
    path = write(*rows)
    kind, data = ts.load(path)
    path.unlink()
    s = ts.summarize(kind, data, 500.0, None, None, run=run)
    return s, verdicts(ts.check(s, False, defaults()))


# App logs: fresh commands and the start-of-run fallback (no miss).
s, v = app_t4(app_rows(200), app_run(200))
assert s["fresh"] == 199 and s["fallbacks"] == 1
assert abs(s["measured"]["controller_time"]["max"] - 10e-6) < 1e-12
assert abs(s["measured"]["write_handoff"]["max"] - 20e-6) < 1e-12
assert v["T3"][1] == "PASS" and v["T4"] == ("measured", "PASS")
# An in-run miss, counted and flagged, fails.
s, v = app_t4(app_rows(200, fallback_at=(100,)), app_run(200, misses=1))
assert v["T4"][1] == "FAIL"
# A stale rejection fails.
s, v = app_t4(app_rows(200), app_run(200, stale=1))
assert v["T4"][1] == "FAIL"
# Ending with a fallback: a miss before the controller finished fails...
s, v = app_t4(app_rows(200, fallback_at=(200,)), app_run(200, misses=1))
assert v["T4"][1] == "FAIL"
# ...while the shutdown windows after it are reported, not counted.
s, v = app_t4(app_rows(200, fallback_at=(199, 200)),
              app_run(200, end=198, misses=0))
assert v["T4"][1] == "PASS" and s["misses_after_end"] == 2
# Only fallbacks: no evidence of a controller, never PASS.
s, v = app_t4(app_rows(200, all_fallbacks=True), app_run(200))
assert v["T4"][1] == "INDETERMINATE"
# No run summary: T3 and T4 cannot be decided.
s, v = app_t4(app_rows(200), None)
assert v["T3"][1] == "INDETERMINATE" and v["T4"][1] == "INDETERMINATE"
# An unsuccessful run, an incomplete log, or counters the flags
# contradict: T4 is undecidable (T3 fails for the incomplete log).
s, v = app_t4(app_rows(200), app_run(200, status="tracking_error"))
assert v["T4"][1] == "INDETERMINATE"
s, v = app_t4(app_rows(200), app_run(200, log_complete=False))
assert v["T3"][1] == "FAIL"
s, v = app_t4(app_rows(150), app_run(200))
assert v["T3"][1] == "FAIL" and v["T4"][1] == "INDETERMINATE"
s, v = app_t4(app_rows(200, fallback_at=(100,)), app_run(200, misses=0))
assert v["T4"][1] == "INDETERMINATE"
# A log without the deadline_miss column cannot support T4.
header, rows = app_rows(200)
columns = header.split(",")
drop = columns.index("deadline_miss")
s, v = app_t4((",".join(columns[:drop] + columns[drop + 1:]),
               [row[:drop] + row[drop + 1:] for row in rows]), app_run(200))
assert v["T4"][1] == "INDETERMINATE"

# Two thread runs (x7_track's mode switch): one numbering, two segments.
# The 0.5 s restart gap is not a period, and nothing is missing.
path = write(*app_rows(200, restart_at=120, restart_gap_ns=500_000_000))
kind, data = ts.load(path)
s = ts.summarize(kind, data, 500.0, None, None)
assert s["segments"] == 2 and s["record_gaps"] == 0, s
assert abs(s["mean_period_s"] - 2e-3) < 1e-12
assert verdicts(ts.check(s, False, defaults()))["T1"][1] == "PASS"
path.unlink()
# Numbering that restarts with the thread, or a skipped segment, is a gap.
header, rows = app_rows(200, restart_at=120, restart_gap_ns=500_000_000)
for k, row in enumerate(rows[119:], start=1):
    row[0] = k
path = write(header, rows)
kind, data = ts.load(path)
assert ts.summarize(kind, data, 500.0, None, None)["record_gaps"] > 0
path.unlink()
header, rows = app_rows(200, restart_at=120, restart_gap_ns=500_000_000)
for row in rows[119:]:
    row[1] = 3
path = write(header, rows)
kind, data = ts.load(path)
assert ts.summarize(kind, data, 500.0, None, None)["record_gaps"] > 0
path.unlink()

# A missing cycle number is a record gap (a dropped record).
header, rows = probe_rows(50)
del rows[20]
path = write(header, rows)
kind, data = ts.load(path)
assert ts.summarize(kind, data, 500.0, None, None)["record_gaps"] > 0
path.unlink()

# Nearest rank is exact on small samples.
assert ts.nearest_rank(np.array([3.0, 1.0, 2.0]), 0.5) == 2.0
assert ts.nearest_rank(np.array([3.0, 1.0, 2.0]), 1.0) == 3.0

print("timing summary regressions OK")
