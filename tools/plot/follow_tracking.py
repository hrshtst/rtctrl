#!/usr/bin/env python3
"""Plot phase-complete CSV telemetry from x7_follow[_sim]."""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

DOF = 8
SCHEMA_VERSION = 1
JOINT_LABELS = (
    "J0 shoulder pan",
    "J1 shoulder tilt",
    "J2 upper twist",
    "J3 elbow",
    "J4 forearm twist",
    "J5 wrist pitch",
    "J6 wrist rotate",
    "J7 gripper",
)
PHASES = ("home", "home_correction", "tracking", "finalizing", "abort_hold")
PHASE_COLORS = ("#dbeafe", "#ede9fe", "#dcfce7", "#fef3c7", "#fee2e2")


def _required_columns() -> set[str]:
    columns = {
        "schema_version",
        "time_s",
        "phase",
        "command_mode",
        "feedback_time_s",
        "feedback_seq",
        "receipt_accepted",
        "submitted_seq",
        "applied_seq",
    }
    for joint in range(DOF):
        for prefix, unit in (
            ("qref", "rad"),
            ("dqref", "rad_s"),
            ("ddqref", "rad_s2"),
            ("q", "rad"),
            ("dq", "rad_s"),
            ("tau", "nm"),
            ("qcmd", "rad"),
            ("dqcmd", "rad_s"),
            ("effort_limit", "nm"),
            ("error", "rad"),
            ("applied", ""),
            ("applied_effort_limit", "nm"),
            ("flags", ""),
        ):
            suffix = f"_{unit}" if unit else ""
            columns.add(f"{prefix}{joint}{suffix}")
    return columns


def load_log(path: pathlib.Path) -> np.ndarray:
    try:
        data = np.genfromtxt(
            path, delimiter=",", names=True, dtype=None, encoding="utf-8"
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"{path}: cannot read follow log: {error}") from error
    if data.dtype.names is None or data.size == 0:
        raise SystemExit(f"{path}: no log rows")
    data = np.atleast_1d(data)
    missing = sorted(_required_columns() - set(data.dtype.names))
    if missing:
        raise SystemExit(f"{path}: missing columns: {', '.join(missing)}")
    if np.any(data["schema_version"] != SCHEMA_VERSION):
        raise SystemExit(f"{path}: unsupported schema version")
    if np.any(np.diff(data["time_s"]) < 0.0):
        raise SystemExit(f"{path}: time must be nondecreasing")
    unknown = sorted(set(data["phase"]) - set(PHASES))
    if unknown:
        raise SystemExit(f"{path}: unknown phases: {', '.join(unknown)}")
    for name in _required_columns() - {"phase"}:
        if not np.all(np.isfinite(data[name])):
            raise SystemExit(f"{path}: non-finite values in {name}")
    return data


def _joints(data: np.ndarray, prefix: str, unit: str = "") -> np.ndarray:
    suffix = f"_{unit}" if unit else ""
    return np.column_stack([data[f"{prefix}{joint}{suffix}"] for joint in range(DOF)])


def _decorate(axis, ylabel: str) -> None:
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.25)


def _legend_above(axis, handles=None, labels=None, *, ncol: int = 4) -> None:
    options = {
        "bbox_to_anchor": (0.5, 1.02),
        "borderaxespad": 0.0,
        "framealpha": 1.0,
        "loc": "lower center",
        "ncol": ncol,
        "fontsize": "x-small",
    }
    if handles is None:
        axis.legend(**options)
    else:
        axis.legend(handles, labels, **options)


def _shade_phases(axis, data: np.ndarray) -> None:
    time = data["time_s"]
    if len(time) == 1:
        return
    boundaries = np.flatnonzero(data["phase"][1:] != data["phase"][:-1]) + 1
    starts = np.r_[0, boundaries]
    ends = np.r_[boundaries, len(time)]
    color = dict(zip(PHASES, PHASE_COLORS))
    for start, end in zip(starts, ends):
        right = time[end] if end < len(time) else time[-1]
        axis.axvspan(time[start], right, color=color[data["phase"][start]], alpha=0.24)


def _plot_joint_pair(axis, time, first, second, ylabel: str) -> None:
    for joint, label in enumerate(JOINT_LABELS):
        (line,) = axis.plot(time, first[:, joint], label=f"ref {label}")
        axis.plot(
            time, second[:, joint], "--", color=line.get_color(), label=f"meas {label}"
        )
    _decorate(axis, ylabel)
    _legend_above(axis, ncol=4)


def _plot_joints(axis, time, values, ylabel: str) -> None:
    for joint, label in enumerate(JOINT_LABELS):
        axis.plot(time, values[:, joint], label=label)
    _decorate(axis, ylabel)
    _legend_above(axis)


def plot_log(data: np.ndarray, title: str | None = None):
    import matplotlib.pyplot as plt

    time = data["time_s"]
    figure, axes = plt.subplots(
        4, 2, figsize=(16, 18), sharex=True, constrained_layout=True
    )
    q_axis, error_axis = axes[0]
    dq_axis, command_axis = axes[1]
    torque_axis, effort_axis = axes[2]
    event_axis, phase_axis = axes[3]

    _plot_joint_pair(
        q_axis, time, _joints(data, "qref", "rad"), _joints(data, "q", "rad"), "q [rad]"
    )
    _plot_joints(error_axis, time, _joints(data, "error", "rad"), "q - qref [rad]")
    _plot_joint_pair(
        dq_axis,
        time,
        _joints(data, "dqref", "rad_s"),
        _joints(data, "dq", "rad_s"),
        "dq [rad/s]",
    )
    _plot_joints(
        command_axis,
        time,
        _joints(data, "qcmd", "rad"),
        "position command [rad]",
    )
    _plot_joints(torque_axis, time, _joints(data, "tau", "nm"), "measured torque [Nm]")
    _plot_joints(
        effort_axis,
        time,
        _joints(data, "applied_effort_limit", "nm"),
        "applied effort ceiling [Nm]",
    )

    accepted = data["receipt_accepted"].astype(float)
    applied_lag = np.maximum(data["submitted_seq"] - data["applied_seq"], 0)
    flags = np.count_nonzero(_joints(data, "flags"), axis=1)
    event_axis.step(time, 1.0 - accepted, where="post", label="rejected receipt")
    event_axis.step(time, applied_lag, where="post", label="submitted - applied seq")
    event_axis.step(time, flags, where="post", label="joints clamped/gated")
    _decorate(event_axis, "command event count")
    _legend_above(event_axis, ncol=3)

    max_error = np.max(np.abs(_joints(data, "error", "rad")), axis=1)
    phase_axis.plot(time, max_error, color="black", label="max |joint error|")
    _decorate(phase_axis, "worst joint error [rad]")
    _legend_above(phase_axis, ncol=1)

    for axis in axes.flat:
        _shade_phases(axis, data)
    for axis in axes[-1]:
        axis.set_xlabel("time [s]")
    if title:
        figure.suptitle(title)
    return figure


def print_summary(data: np.ndarray) -> None:
    error = np.abs(_joints(data, "error", "rad"))
    flags = np.count_nonzero(_joints(data, "flags"))
    rejected = np.count_nonzero(data["receipt_accepted"] == 0)
    phases = ", ".join(
        f"{phase}={np.count_nonzero(data['phase'] == phase)}"
        for phase in PHASES
        if phase in data["phase"]
    )
    print(
        f"{len(data)} cycles, duration {data['time_s'][-1]:.6g} s\n"
        f"maximum joint tracking error: {np.max(error):.6g} rad\n"
        f"rejected command receipts: {rejected}; clamped/gated joint events: {flags}\n"
        f"phase cycles: {phases}"
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot x7_follow CSV telemetry")
    parser.add_argument("log", type=pathlib.Path, help="follow CSV log")
    parser.add_argument(
        "-o", "--output", type=pathlib.Path, help="PNG, PDF, or SVG output"
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
    data = load_log(args.log)
    figure = plot_log(data, args.title or f"Trajectory following: {args.log.name}")
    print_summary(data)
    if args.output:
        if args.output.resolve() == args.log.resolve():
            raise SystemExit("refusing to overwrite the CSV log")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output, dpi=args.dpi)
        print(f"wrote {args.output}")
    if args.show:
        import matplotlib.pyplot as plt

        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
