#!/usr/bin/env python3
"""Regressions for timing_summary.py, run under ctest.

Synthetic probe and app timing logs with known stamps: the measured
intervals, the calibration distribution, the envelope and estimate
checks, and every criterion's pass and fail are pinned, including the
classes the report attaches to them.
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
    "cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,"
    "first_reply_ns,last_reply_ns,read_end_ns,window_open_ns,"
    "controller_wake_ns,submit_ns,limit_start_ns,bus_write_start_ns,"
    "write_return_ns,cycle_end_ns,feedback_seq,applied_target_seq,"
    "applied_source_feedback_seq,skipped_periods,read_ok,write_ok,"
    "command_kind")


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


def app_rows(n, fallback_at=None):
    rows = []
    for k in range(1, n + 1):
        sched = (k - 1) * PERIOD_NS
        wake = sched + 10_000
        read_start = wake + 1_000
        read_end = read_start + 600_000
        window = read_end + 1_000
        ctrl = window + 50_000
        fallback = k == fallback_at
        submit = -1 if fallback else ctrl + 10_000
        limit = window + 1_400_000 if fallback else submit + 20_000
        bus = limit + 2_000
        ret = bus + 30_000
        end = ret + 5_000
        kind = "fallback" if fallback or k == 1 else "fresh"
        rows.append([k, sched, wake, read_start, read_start + 20_000,
                     read_start + 400_000, read_end - 3_000, read_end, window,
                     ctrl, submit, limit, bus, ret, end, k + 1, k, k + 1, 0, 1,
                     1, kind])
    return APP_HEADER, rows


def write(header, rows):
    handle = tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False)
    with handle:
        handle.write(header + "\n")
        for row in rows:
            handle.write(",".join(str(v) for v in row) + "\n")
    return pathlib.Path(handle.name)


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
s = ts.summarize(kind, data, 500.0, delta_env_s=0.4e-3, delta_q_s=0.3e-3)
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
path.unlink()

# Failures: a skip, a read failure, late wakes and a tight envelope.
path = write(*probe_rows(1000, lateness_ns=600_000, skip_at=500,
                         read_fail_at=10))
kind, data = ts.load(path)
s = ts.summarize(kind, data, 500.0, delta_env_s=1.5e-3, delta_q_s=None)
v = verdicts(ts.check(s, False, defaults()))
assert v["T2"][1] == "FAIL"  # idle allows no skip
assert v["T3"][1] == "FAIL"
assert v["T5"][1] == "FAIL"  # 0.6 ms > 0.1 T and > 0.5 T
assert v["T6b"][1] == "FAIL"
assert v["T7"] == ("estimated", "n/a")
# Loaded runs allow 1 skip per 1e4 cycles: 1000 cycles allow 0.1, so the
# single skip still fails; 20000 would allow it.
assert ts.check(s, True, defaults())[1][2] == "FAIL"
path.unlink()

# App logs: fresh commands, the start-of-run fallback, and an in-run miss.
path = write(*app_rows(200))
kind, data = ts.load(path)
assert kind == "app"
s = ts.summarize(kind, data, 500.0, None, None)
assert s["fresh"] == 199 and s["in_run_fallbacks"] == 0
assert abs(s["measured"]["controller_time"]["max"] - 10e-6) < 1e-12
assert abs(s["measured"]["write_handoff"]["max"] - 20e-6) < 1e-12
assert verdicts(ts.check(s, False, defaults()))["T4"][1] == "PASS"
path.unlink()
path = write(*app_rows(200, fallback_at=100))
kind, data = ts.load(path)
s = ts.summarize(kind, data, 500.0, None, None)
assert s["in_run_fallbacks"] == 1
assert verdicts(ts.check(s, False, defaults()))["T4"][1] == "FAIL"
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
