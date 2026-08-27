#!/usr/bin/env python3
"""Render the phase-complete tracking plot stored in a follow bundle."""

from __future__ import annotations

import argparse
import pathlib
import sys

from follow_tracking import main as plot_tracking


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create tracking-analysis.png from hardware.csv or simulation.csv "
            "in an x7_follow bundle"
        )
    )
    parser.add_argument("bundle", type=pathlib.Path, help="follow bundle directory")
    parser.add_argument("-o", "--output", type=pathlib.Path, help="output image")
    parser.add_argument("--title", help="figure title")
    parser.add_argument(
        "--dpi", type=int, default=160, help="raster resolution (default: 160)"
    )
    return parser.parse_args(argv)


def find_log(bundle: pathlib.Path) -> pathlib.Path:
    if not bundle.is_dir():
        raise SystemExit(f"{bundle}: not a follow bundle directory")
    candidates = [
        path
        for name in ("hardware.csv", "simulation.csv")
        if (path := bundle / name).is_file()
    ]
    if not candidates:
        raise SystemExit(f"{bundle}: no hardware.csv or simulation.csv")
    if len(candidates) > 1:
        names = ", ".join(path.name for path in candidates)
        raise SystemExit(f"{bundle}: multiple follow logs found: {names}")
    return candidates[0]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    log = find_log(args.bundle)
    output = args.output or args.bundle / "tracking-analysis.png"
    plot_args = [str(log), "--output", str(output), "--dpi", str(args.dpi)]
    if args.title:
        plot_args.extend(("--title", args.title))
    return plot_tracking(plot_args)


if __name__ == "__main__":
    sys.exit(main())
