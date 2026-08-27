#!/usr/bin/env python3
"""Compare position and current-based-position x7_follow telemetry."""

from __future__ import annotations

import argparse
import csv
import pathlib

import numpy as np

from follow_tracking import DOF, JOINT_LABELS, load_log

POSITION_MODE = 3
CBP_MODE = 5


def _joints(data: np.ndarray, prefix: str, unit: str = "") -> np.ndarray:
    suffix = f"_{unit}" if unit else ""
    return np.column_stack([data[f"{prefix}{joint}{suffix}"] for joint in range(DOF)])


def _tracking(data: np.ndarray, expected_mode: int, path: pathlib.Path) -> np.ndarray:
    modes = set(np.asarray(data["command_mode"], dtype=int))
    if modes != {expected_mode}:
        expected = (
            "position" if expected_mode == POSITION_MODE else "current-based-position"
        )
        found = ", ".join(str(mode) for mode in sorted(modes))
        raise SystemExit(
            f"{path}: expected {expected} command mode {expected_mode}, found {found}"
        )
    tracking = data[data["phase"] == "tracking"]
    if tracking.size == 0:
        raise SystemExit(f"{path}: no tracking-phase rows")
    return np.atleast_1d(tracking)


def _errors(data: np.ndarray) -> np.ndarray:
    return _joints(data, "error", "rad")


def _validate_comparable(position: np.ndarray, cbp: np.ndarray) -> None:
    if len(position) != len(cbp) or not np.allclose(
        position["phase_time_s"], cbp["phase_time_s"], rtol=1e-9, atol=1e-9
    ):
        raise SystemExit("tracking logs use different reference sample times")
    if not np.allclose(
        _joints(position, "qref", "rad"),
        _joints(cbp, "qref", "rad"),
        rtol=1e-9,
        atol=1e-9,
    ):
        raise SystemExit("tracking logs contain different joint references")


def _rms(values: np.ndarray, axis=None):
    return np.sqrt(np.mean(np.square(values), axis=axis))


def _period_ms(data: np.ndarray) -> np.ndarray:
    return np.diff(np.asarray(data["time_s"], dtype=float)) * 1000.0


def _print_run_summary(label: str, data: np.ndarray) -> None:
    error = _errors(data)
    periods = _period_ms(data)
    rejected = np.count_nonzero(data["receipt_accepted"] == 0)
    flags = np.count_nonzero(_joints(data, "flags"))
    max_lag = np.max(np.maximum(data["submitted_seq"] - data["applied_seq"], 0))
    duration = data["phase_time_s"][-1] - data["phase_time_s"][0]
    timing = "n/a"
    if periods.size:
        timing = (
            f"{np.median(periods):.3f}/{np.percentile(periods, 95):.3f}/"
            f"{np.max(periods):.3f} ms"
        )
    print(
        f"{label}: {len(data)} tracking cycles, {duration:.6g} s\n"
        f"  aggregate RMS {float(_rms(error)):.6g} rad; "
        f"peak {np.max(np.abs(error)):.6g} rad\n"
        f"  cycle period median/p95/max {timing}\n"
        f"  rejected receipts {rejected}; joint clamp/gate events {flags}; "
        f"maximum submitted-to-applied lag {max_lag}"
    )


def print_summary(position: np.ndarray, cbp: np.ndarray) -> None:
    print("tracking-phase comparison")
    _print_run_summary("position", position)
    _print_run_summary("current-based-position", cbp)
    print("\nper-joint RMS / peak absolute error [rad]")
    print("joint                    position             current-based-position")
    position_error = _errors(position)
    cbp_error = _errors(cbp)
    for joint, label in enumerate(JOINT_LABELS):
        pos_rms = _rms(position_error[:, joint])
        pos_peak = np.max(np.abs(position_error[:, joint]))
        cbp_rms = _rms(cbp_error[:, joint])
        cbp_peak = np.max(np.abs(cbp_error[:, joint]))
        print(
            f"{label:<24} {pos_rms:.6g} / {pos_peak:.6g}    "
            f"{cbp_rms:.6g} / {cbp_peak:.6g}"
        )


def write_summary_csv(
    path: pathlib.Path, position: np.ndarray, cbp: np.ndarray
) -> None:
    position_error = _errors(position)
    cbp_error = _errors(cbp)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "joint",
                "position_rms_rad",
                "position_peak_abs_rad",
                "current_based_position_rms_rad",
                "current_based_position_peak_abs_rad",
            )
        )
        for joint, label in enumerate(JOINT_LABELS):
            writer.writerow(
                (
                    label,
                    _rms(position_error[:, joint]),
                    np.max(np.abs(position_error[:, joint])),
                    _rms(cbp_error[:, joint]),
                    np.max(np.abs(cbp_error[:, joint])),
                )
            )
        writer.writerow(
            (
                "all joints",
                _rms(position_error),
                np.max(np.abs(position_error)),
                _rms(cbp_error),
                np.max(np.abs(cbp_error)),
            )
        )


def _legend_above(axis) -> None:
    axis.legend(
        bbox_to_anchor=(0.5, 1.02),
        borderaxespad=0.0,
        framealpha=1.0,
        loc="lower center",
        ncol=2,
        fontsize="x-small",
    )


def plot_comparison(position: np.ndarray, cbp: np.ndarray, title: str | None):
    import matplotlib.pyplot as plt

    position_error = _errors(position)
    cbp_error = _errors(cbp)
    figure, axes = plt.subplots(
        5, 2, figsize=(15, 17), sharex=False, constrained_layout=True
    )
    for joint, axis in enumerate(axes[:4].flat):
        axis.plot(
            position["phase_time_s"],
            position_error[:, joint],
            label="position",
        )
        axis.plot(
            cbp["phase_time_s"],
            cbp_error[:, joint],
            label="current-based-position",
        )
        axis.axhline(0.0, color="black", linewidth=0.7, alpha=0.5)
        axis.set_ylabel(f"{JOINT_LABELS[joint]}\nq - qref [rad]")
        axis.grid(True, alpha=0.25)
        _legend_above(axis)
    for axis in axes[3]:
        axis.set_xlabel("tracking phase time [s]")

    joint_indices = np.arange(DOF)
    width = 0.38
    labels = [f"J{joint}" for joint in range(DOF)]
    rms_axis, peak_axis = axes[4]
    rms_axis.bar(
        joint_indices - width / 2,
        _rms(position_error, axis=0),
        width,
        label="position",
    )
    rms_axis.bar(
        joint_indices + width / 2,
        _rms(cbp_error, axis=0),
        width,
        label="current-based-position",
    )
    peak_axis.bar(
        joint_indices - width / 2,
        np.max(np.abs(position_error), axis=0),
        width,
        label="position",
    )
    peak_axis.bar(
        joint_indices + width / 2,
        np.max(np.abs(cbp_error), axis=0),
        width,
        label="current-based-position",
    )
    for axis, ylabel in (
        (rms_axis, "tracking RMS error [rad]"),
        (peak_axis, "peak tracking |error| [rad]"),
    ):
        axis.set_xticks(joint_indices, labels)
        axis.set_xlabel("joint")
        axis.set_ylabel(ylabel)
        axis.grid(True, axis="y", alpha=0.25)
        _legend_above(axis)
    if title:
        figure.suptitle(title)
    return figure


def _resolve_logs(
    first: pathlib.Path, second: pathlib.Path | None
) -> tuple[pathlib.Path, pathlib.Path]:
    if second is not None:
        return first, second
    return first / "position" / "hardware.csv", first / "cbp" / "hardware.csv"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare position and current-based-position x7_follow logs"
    )
    parser.add_argument(
        "input",
        type=pathlib.Path,
        help=(
            "run directory containing position/hardware.csv and cbp/hardware.csv, "
            "or the position CSV when cbp_log is also given"
        ),
    )
    parser.add_argument(
        "cbp_log",
        nargs="?",
        type=pathlib.Path,
        help="current-based-position CSV when input is the position CSV",
    )
    parser.add_argument(
        "-o", "--output", type=pathlib.Path, help="PNG, PDF, or SVG output"
    )
    parser.add_argument(
        "--summary-csv", type=pathlib.Path, help="write per-joint metrics as CSV"
    )
    parser.add_argument(
        "--show", action="store_true", help="open an interactive plot window"
    )
    parser.add_argument("--title", help="figure title")
    parser.add_argument(
        "--dpi", type=int, default=160, help="raster resolution (default: 160)"
    )
    args = parser.parse_args(argv)
    if args.output is None and not args.show:
        parser.error("--output is required unless --show is selected")
    if args.dpi <= 0:
        parser.error("--dpi must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if not args.show:
        import matplotlib

        matplotlib.use("Agg")
    position_path, cbp_path = _resolve_logs(args.input, args.cbp_log)
    position = _tracking(load_log(position_path), POSITION_MODE, position_path)
    cbp = _tracking(load_log(cbp_path), CBP_MODE, cbp_path)
    _validate_comparable(position, cbp)
    logs = {position_path.resolve(), cbp_path.resolve()}
    outputs = [path.resolve() for path in (args.output, args.summary_csv) if path]
    if any(path in logs for path in outputs):
        raise SystemExit("refusing to overwrite an input CSV log")
    if len(outputs) != len(set(outputs)):
        raise SystemExit("plot and summary outputs must be different files")
    print_summary(position, cbp)
    figure = plot_comparison(
        position, cbp, args.title or "x7_follow hardware mode comparison"
    )
    if args.summary_csv:
        write_summary_csv(args.summary_csv, position, cbp)
        print(f"wrote {args.summary_csv}")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output, dpi=args.dpi)
        print(f"wrote {args.output}")
    if args.show:
        import matplotlib.pyplot as plt

        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
