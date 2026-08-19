#!/usr/bin/env python3
"""Replay recorded float/feel-log postures through the vendor gravity
model and compare per-joint current commands against rtctrl's.

Reviewer-directed simulation step for the M-GC3 back-drive exceptions
(docs/records/history.md (gravity calibration)): rtctrl's side of the comparison is
already in the log — tau_request at the measured posture each cycle —
and is converted to amps here exactly as the command boundary does
(i = command_torque_scale * tau / kt_nominal, hw::commandCurrentFromTorque).
The vendor side is produced by the in-tree fixture generator in posture
stream mode:

  g++ ... tests/fixtures/vendor_gravity_dump.cpp ... -o /tmp/vendor_gravity_dump
  uv run --project tools tools/replay_compare.py \
      --replay-bin /tmp/vendor_gravity_dump feel_j1.csv feel_j4.csv

(build line in the fixture's header; run from the repo root — the
fixture loads the vendor link CSV by relative path).

Per log and joint it reports the two commands' ranges, the difference
(mean and worst) in amps / goal-current counts / Nm at the vendor's
tuned torque constant, and a least-squares proportional fit
i_rtctrl ~ ratio * i_vendor with its worst residual — separating a
pure amplitude disagreement between the two mass models from a shaped
one.
"""

import argparse
import csv
import re
import subprocess
import sys

DOF = 8  # canonical joint order; the vendor chain covers j0..j6
KT_NOMINAL = [1.783, 2.409, 1.783, 1.783, 1.783, 1.783, 1.783]
KT_VENDOR = [2.20, 3.60, 2.20, 2.20, 2.20, 2.20, 2.20]
CURRENT_LSB_A = 0.00269  # goal/present-current LSB (dxl/conversions.hpp)

NUMERIC_LINE = re.compile(r"[-\d.eE+ ]+")


def read_log(path):
    """Header scales + data rows of an x7_float --log CSV."""
    lines = open(path).read().splitlines()
    comments = []
    while lines and lines[0].startswith("#"):
        comments.append(lines.pop(0))
    scale_lines = [c for c in comments
                   if c.startswith("# command_torque_scale:")]
    if not scale_lines:
        sys.exit(f"{path}: no command_torque_scale header")
    scales = [float(v) for v in scale_lines[0].split(":", 1)[1].split()]
    return scales, list(csv.DictReader(lines))


def vendor_currents(replay_bin, rows):
    """Vendor per-joint currents [A] at every row's measured posture."""
    postures = "\n".join(
        " ".join(r[f"q{i}"] for i in range(7)) for r in rows) + "\n"
    out = subprocess.run([replay_bin, "--replay"], input=postures,
                         capture_output=True, text=True, check=True)
    # the vendor link loader prints an info line to stdout; keep only
    # strictly numeric lines
    vend = [[float(x) for x in ln.split()]
            for ln in out.stdout.splitlines()
            if ln.strip() and NUMERIC_LINE.fullmatch(ln.strip())]
    if len(vend) != len(rows):
        sys.exit(f"replay returned {len(vend)} lines for {len(rows)} rows")
    return vend


def compare(path, scales, rows, vend, joints):
    print(f"\n== {path}  ({len(rows)} rows) ==")
    for j in joints:
        i_rt = [scales[j] * float(r[f"tau_request{j}"]) / KT_NOMINAL[j]
                for r in rows]
        i_v = [v[j] for v in vend]
        delta = [a - b for a, b in zip(i_rt, i_v)]
        mean_d = sum(delta) / len(delta)
        worst_d = max(delta, key=abs)
        print(f" j{j}: i_rtctrl [{min(i_rt):+.4f}, {max(i_rt):+.4f}] A"
              f"   i_vendor [{min(i_v):+.4f}, {max(i_v):+.4f}] A")
        print(f"     delta mean {mean_d:+.5f} A"
              f" ({mean_d / CURRENT_LSB_A:+.2f} counts,"
              f" {mean_d * KT_VENDOR[j]:+.4f} Nm@vendor-kt)"
              f"   worst {worst_d:+.5f} A"
              f" ({worst_d / CURRENT_LSB_A:+.2f} counts,"
              f" {worst_d * KT_VENDOR[j]:+.4f} Nm@vendor-kt)")
        sxy = sum(a * b for a, b in zip(i_rt, i_v))
        sxx = sum(b * b for b in i_v)
        if sxx > 1e-12:
            ratio = sxy / sxx
            resid = max(abs(a - ratio * b) for a, b in zip(i_rt, i_v))
            print(f"     LS fit: i_rtctrl = {ratio:.3f} * i_vendor,"
                  f" worst residual {resid:.4f} A"
                  f" ({resid / CURRENT_LSB_A:.1f} counts)")
        else:
            print("     LS fit: vendor command ~0 throughout; no ratio")


def main():
    ap = argparse.ArgumentParser(
        description="rtctrl-vs-vendor gravity current replay over "
                    "recorded float/feel logs")
    ap.add_argument("--replay-bin", required=True,
                    help="vendor_gravity_dump binary (invoked --replay)")
    ap.add_argument("--joints", default="1,4",
                    help="comma-separated canonical joints (default 1,4)")
    ap.add_argument("logs", nargs="+", help="x7_float --log CSV files")
    args = ap.parse_args()
    joints = [int(t) for t in args.joints.split(",")]
    if any(j < 0 or j > 6 for j in joints):
        sys.exit("joints must be canonical 0..6 (vendor chain coverage)")
    for path in args.logs:
        scales, rows = read_log(path)
        compare(path, scales, rows, vendor_currents(args.replay_bin, rows),
                joints)


if __name__ == "__main__":
    main()
