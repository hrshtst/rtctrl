#!/usr/bin/env python3
"""Summarize a per-cycle timing CSV and check the control-rate criteria.

Reads either an x7_bus_timing probe log or an x7_follow / x7_track
--timing-log file (detected from the header), reduces it to the
quantities the control-rate qualification uses, and with --check
evaluates the timing criteria T1..T7, exiting 1 if any fails.

Every quantity is labelled with its class:
  measured   host-clock intervals taken from the log itself;
  estimated  a statistic of calibration data applied to this log (for
             example a completion-offset quantile); it estimates a
             quantile and bounds no individual cycle;
  envelope   a check that relies on an assumed worst case (for example
             delta_env, the assumed maximum completion offset).
Host stamps are host observations: read_start is when the host began the
feedback transaction, not when the servos sampled, and the age of a
servo's internal measurement is unknown.

App logs number their cycles 1, 2, ... over the whole file and mark each
background-thread run with a segment (x7_track restarts its thread at the
mode switch). Periods and consecutive skips are taken within a segment;
the restart gap between two segments is not a period.

Percentiles are exact nearest-rank values over the recorded cycles.
Threshold defaults are the plan's PROPOSED values; the frozen values are
whatever the qualification record's command line states.

  uv run --project tools tools/timing_summary.py run.csv --rate-hz 500 \\
      [--loaded] [--delta-env-s 1.2e-3] [--delta-q-s 0.9e-3] [--check]
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib

import numpy as np

PROBE_COLUMNS = {"ack_return_ns", "completion_upper_ns"}
APP_COLUMNS = {"controller_wake_ns", "command_kind"}


def load(path: pathlib.Path) -> tuple[str, dict[str, np.ndarray]]:
    with open(path, newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise SystemExit(f"{path}: empty timing log")
        fields = set(reader.fieldnames)
        rows = list(reader)
    if PROBE_COLUMNS <= fields:
        kind = "probe"
    elif APP_COLUMNS <= fields:
        kind = "app"
    else:
        raise SystemExit(f"{path}: not an x7 timing log (unknown header)")
    if not rows:
        raise SystemExit(f"{path}: no cycles recorded")
    data: dict[str, np.ndarray] = {}
    for name in reader.fieldnames:
        if name == "command_kind":
            data[name] = np.array([row[name] for row in rows])
        else:
            data[name] = np.array([int(row[name]) for row in rows],
                                  dtype=np.int64)
    return kind, data


def nearest_rank(values: np.ndarray, q: float) -> float:
    ordered = np.sort(values)
    index = max(0, min(len(ordered) - 1,
                       math.ceil(q * len(ordered)) - 1))
    return float(ordered[index])


def interval(data: dict[str, np.ndarray], start: str, end: str) -> np.ndarray:
    """Seconds between two stamps, over the cycles that reached both."""
    a, b = data[start], data[end]
    keep = (a >= 0) & (b >= 0)
    return (b[keep] - a[keep]).astype(np.float64) * 1e-9


def describe(values: np.ndarray) -> dict[str, float]:
    if len(values) == 0:
        return {}
    return {"n": float(len(values)),
            "p50": nearest_rank(values, 0.5),
            "p99": nearest_rank(values, 0.99),
            "p999": nearest_rank(values, 0.999),
            "max": float(values.max())}


def summarize(kind: str, data: dict[str, np.ndarray], rate_hz: float,
              delta_env_s: float | None,
              delta_q_s: float | None) -> dict:
    period = 1.0 / rate_hz
    cycles = data["cycle"]
    wake = data["wake_ns"]
    skipped = data["skipped_periods"]
    # Probe logs are one segment and carry no column.
    segment = data.get("segment", np.ones(len(cycles), dtype=np.int64))
    s: dict = {"kind": kind, "rate_hz": rate_hz, "period_s": period,
               "cycles": int(len(cycles))}
    # A complete log numbers its cycles 1, 2, ... without gaps over the
    # whole file, and its segments 1, 2, ... in order.
    expected = np.arange(1, len(cycles) + 1)
    steps = np.diff(segment)
    s["record_gaps"] = int(np.count_nonzero(cycles != expected) +
                           np.count_nonzero((steps != 0) & (steps != 1)) +
                           (segment[0] != 1))
    s["segments"] = int(np.count_nonzero(steps == 1) + 1)
    within = steps == 0  # consecutive cycles of one thread run
    periods = np.diff(wake)[within].astype(np.float64) * 1e-9
    s["mean_period_s"] = float(periods.mean()) if len(periods) else math.nan
    s["skipped_periods"] = int(skipped.sum())
    flagged = skipped > 0
    s["consecutive_skips"] = int(np.count_nonzero(skipped >= 2) +
                                 np.count_nonzero(flagged[1:] & flagged[:-1]
                                                  & within))
    s["read_failures"] = int(np.count_nonzero(data["read_ok"] == 0))
    s["write_failures"] = int(np.count_nonzero(
        (data["write_ok"] == 0) & (data["write_return_ns"] >= 0)))
    measured = {
        "wake_lateness": interval(data, "scheduled_wake_ns", "wake_ns"),
        "host_transaction": interval(data, "read_start_ns", "read_end_ns"),
        "reply_span": interval(data, "first_reply_ns", "last_reply_ns"),
        "limiter": interval(data, "limit_start_ns", "bus_write_start_ns"),
        "host_write": interval(data, "bus_write_start_ns", "write_return_ns"),
        "host_work": interval(data, "read_start_ns", "cycle_end_ns"),
    }
    if kind == "app":
        measured["controller_handoff"] = interval(
            data, "window_open_ns", "controller_wake_ns")
        measured["controller_time"] = interval(
            data, "controller_wake_ns", "submit_ns")
        measured["write_handoff"] = interval(data, "submit_ns",
                                             "limit_start_ns")
        kinds = data["command_kind"]
        fresh = np.flatnonzero(kinds == "fresh")
        s["fresh"] = int(len(fresh))
        if len(fresh):
            inside = kinds[fresh[0]:fresh[-1] + 1]
            s["in_run_fallbacks"] = int(np.count_nonzero(
                (inside == "fallback") | (inside == "zero-velocity-fallback")))
        else:
            s["in_run_fallbacks"] = 0
    s["measured"] = {name: describe(v) for name, v in measured.items()}

    to_write = interval(data, "read_start_ns", "bus_write_start_ns")
    if kind == "probe":
        ok = data["ack_ok"] == 1
        if np.any(ok):
            delta = (data["completion_upper_ns"][ok] -
                     data["bus_write_start_ns"][ok]).astype(np.float64) * 1e-9
            s["calibration"] = describe(delta)
            s["calibration"]["delta_env_candidate"] = float(
                1.25 * delta.max())
    if delta_env_s is not None and len(to_write):
        s["envelope_completion_max_s"] = float(to_write.max() + delta_env_s)
    if delta_q_s is not None and len(to_write):
        s["estimated_completion_p999_s"] = nearest_rank(
            to_write + delta_q_s, 0.999)
    return s


def check(s: dict, loaded: bool, args: argparse.Namespace) -> list[tuple]:
    """(criterion, class, verdict, detail) for each timing criterion."""
    period = s["period_s"]
    out = []

    def verdict(name, cls, ok, detail):
        out.append((name, cls, "n/a" if ok is None else
                    ("PASS" if ok else "FAIL"), detail))

    err = abs(s["mean_period_s"] - period) / period
    verdict("T1", "measured", err <= args.period_tolerance,
            f"mean period error {100 * err:.4f}%")
    allowed = (args.loaded_skips_per_10k * s["cycles"] / 1e4) if loaded else 0
    verdict("T2", "measured",
            s["skipped_periods"] <= allowed and s["consecutive_skips"] == 0,
            f"{s['skipped_periods']} skipped (allowed {allowed:g}), "
            f"{s['consecutive_skips']} consecutive")
    faults = s["read_failures"] + s["write_failures"] + s["record_gaps"]
    verdict("T3", "measured", faults == 0,
            f"{s['read_failures']} read / {s['write_failures']} write "
            f"failures, {s['record_gaps']} record gaps")
    if s["kind"] == "app":
        verdict("T4", "measured", s["in_run_fallbacks"] == 0,
                f"{s['in_run_fallbacks']} in-run fallbacks (stale rejections:"
                " read the app's hardware timing line)")
    else:
        verdict("T4", "measured", None, "probe log has no controller")
    lateness = s["measured"]["wake_lateness"]
    verdict("T5", "measured",
            lateness["p999"] <= args.lateness_p999 * period and
            lateness["max"] <= args.lateness_max * period,
            f"p99.9 {1e3 * lateness['p999']:.3f} ms, "
            f"max {1e3 * lateness['max']:.3f} ms")
    work = s["measured"]["host_work"]
    verdict("T6a", "measured",
            work["p999"] <= args.host_work_p999 * period and
            work["max"] < period,
            f"p99.9 {1e3 * work['p999']:.3f} ms, max {1e3 * work['max']:.3f} ms")
    if "envelope_completion_max_s" in s:
        value = s["envelope_completion_max_s"]
        verdict("T6b", "envelope", value <= period,
                f"max (bus_write_start - read_start) + delta_env = "
                f"{1e3 * value:.3f} ms")
    else:
        verdict("T6b", "envelope", None, "needs --delta-env-s")
    if "estimated_completion_p999_s" in s:
        value = s["estimated_completion_p999_s"]
        verdict("T7", "estimated", value <= args.estimate_p999 * period,
                f"p99.9 (bus_write_start - read_start) + delta_q = "
                f"{1e3 * value:.3f} ms")
    else:
        verdict("T7", "estimated", None, "needs --delta-q-s")
    return out


def print_summary(s: dict) -> None:
    print(f"{s['kind']} log: {s['cycles']} cycles in {s['segments']} "
          f"segment(s) at {s['rate_hz']:g} Hz "
          f"(period {1e3 * s['period_s']:.3f} ms)")
    print(f"  mean period {1e3 * s['mean_period_s']:.4f} ms, "
          f"{s['skipped_periods']} skipped, {s['read_failures']} read / "
          f"{s['write_failures']} write failures, "
          f"{s['record_gaps']} record gaps")
    if s["kind"] == "app":
        print(f"  {s['fresh']} fresh commands, "
              f"{s['in_run_fallbacks']} in-run fallbacks")
    print("  measured (host clock; ms):")
    for name, d in s["measured"].items():
        if d:
            print(f"    {name:<18} n={int(d['n']):<8} p50 {1e3 * d['p50']:8.3f}"
                  f"  p99 {1e3 * d['p99']:8.3f}  p99.9 {1e3 * d['p999']:8.3f}"
                  f"  max {1e3 * d['max']:8.3f}")
    if "calibration" in s:
        c = s["calibration"]
        print("  calibration, completion offset C+ - bus_write_start (ms):")
        print(f"    n={int(c['n'])} p50 {1e3 * c['p50']:.3f} p99 "
              f"{1e3 * c['p99']:.3f} p99.9 {1e3 * c['p999']:.3f} max "
              f"{1e3 * c['max']:.3f}; envelope candidate (1.25 x max) "
              f"{1e3 * c['delta_env_candidate']:.3f}")
    print("  read_start is a host transaction stamp; servo measurement age "
          "is unknown")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarize a per-cycle timing CSV and check the "
        "control-rate criteria.")
    parser.add_argument("csv", type=pathlib.Path)
    parser.add_argument("--rate-hz", type=float, required=True)
    parser.add_argument("--loaded", action="store_true",
                        help="the run was under the loaded condition (T2)")
    parser.add_argument("--delta-env-s", type=float,
                        help="assumed maximum completion offset (T6b)")
    parser.add_argument("--delta-q-s", type=float,
                        help="calibration completion-offset quantile (T7)")
    parser.add_argument("--check", action="store_true",
                        help="evaluate T1..T7; exit 1 on a failure")
    parser.add_argument("--period-tolerance", type=float, default=0.001)
    parser.add_argument("--loaded-skips-per-10k", type=float, default=1.0)
    parser.add_argument("--lateness-p999", type=float, default=0.10)
    parser.add_argument("--lateness-max", type=float, default=0.5)
    parser.add_argument("--host-work-p999", type=float, default=0.70)
    parser.add_argument("--estimate-p999", type=float, default=0.90)
    args = parser.parse_args()
    if not args.rate_hz > 0:
        raise SystemExit("--rate-hz must be positive")
    kind, data = load(args.csv)
    s = summarize(kind, data, args.rate_hz, args.delta_env_s, args.delta_q_s)
    print_summary(s)
    if not args.check:
        return 0
    print("criteria (thresholds as given; defaults are the plan's proposed "
          "values):")
    print(f"  T1 {args.period_tolerance:g}, T2 {args.loaded_skips_per_10k:g}"
          f"/1e4 loaded, T5 {args.lateness_p999:g}/{args.lateness_max:g} T, "
          f"T6a {args.host_work_p999:g} T, T7 {args.estimate_p999:g} T")
    results = check(s, args.loaded, args)
    for name, cls, verdict, detail in results:
        print(f"  {name:<4} {cls:<9} {verdict:<4} {detail}")
    return 1 if any(v == "FAIL" for _, _, v, _ in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
