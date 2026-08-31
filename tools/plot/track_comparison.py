#!/usr/bin/env python3
"""Compare x7_track simulation and hardware telemetry."""

from __future__ import annotations

import argparse
import csv
import hashlib
import pathlib

import numpy as np

DOF = 8
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


def _columns(prefix: str, unit: str = "") -> list[str]:
    suffix = f"_{unit}" if unit else ""
    return [f"{prefix}{joint}{suffix}" for joint in range(DOF)]


def _joints(data: np.ndarray, prefix: str, unit: str = "") -> np.ndarray:
    return np.column_stack([data[name] for name in _columns(prefix, unit)])


def load_tracking(path: pathlib.Path) -> np.ndarray:
    try:
        data = np.genfromtxt(
            path, delimiter=",", names=True, dtype=None, encoding="utf-8"
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"{path}: cannot read track log: {error}") from error
    if data.dtype.names is None or data.size == 0:
        raise SystemExit(f"{path}: no log rows")
    data = np.atleast_1d(data)
    required = {
        "schema_version",
        "time_s",
        "phase",
        "phase_time_s",
        "receipt_accepted",
        "submitted_seq",
        "applied_seq",
        *_columns("qref", "rad"),
        *_columns("q", "rad"),
        *_columns("error", "rad"),
        *_columns("tau_ff", "nm"),
        *_columns("tau_pd", "nm"),
        *_columns("tau_command", "nm"),
        *_columns("flags"),
    }
    missing = sorted(required - set(data.dtype.names))
    if missing:
        raise SystemExit(f"{path}: missing columns: {', '.join(missing)}")
    tracking = np.atleast_1d(data[data["phase"] == "tracking"])
    if tracking.size == 0:
        raise SystemExit(f"{path}: no tracking-phase rows")
    if np.any(tracking["schema_version"] != 1):
        raise SystemExit(f"{path}: unsupported schema version")
    return tracking


def _rms(values: np.ndarray, axis=None):
    return np.sqrt(np.mean(np.square(values), axis=axis))


def print_summary(label: str, data: np.ndarray) -> None:
    error = _joints(data, "error", "rad")
    periods = np.diff(np.asarray(data["time_s"], dtype=float)) * 1000.0
    timing = "n/a"
    if periods.size:
        timing = (
            f"{np.median(periods):.3f}/{np.percentile(periods, 95):.3f}/"
            f"{np.max(periods):.3f} ms"
        )
    rejected = np.count_nonzero(data["receipt_accepted"] == 0)
    events = np.count_nonzero(_joints(data, "flags"))
    print(
        f"{label}: {len(data)} cycles; aggregate RMS {float(_rms(error)):.6g} "
        f"rad; peak {np.max(np.abs(error)):.6g} rad\n"
        f"  cycle period median/p95/max {timing}; rejected receipts "
        f"{rejected}; clamp/gate events {events}"
    )


def write_summary(path: pathlib.Path, sim: np.ndarray, hardware: np.ndarray) -> None:
    sim_error = _joints(sim, "error", "rad")
    hw_error = _joints(hardware, "error", "rad")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "joint",
                "simulation_rms_rad",
                "simulation_peak_abs_rad",
                "hardware_rms_rad",
                "hardware_peak_abs_rad",
            )
        )
        for joint, label in enumerate(JOINT_LABELS):
            writer.writerow(
                (
                    label,
                    _rms(sim_error[:, joint]),
                    np.max(np.abs(sim_error[:, joint])),
                    _rms(hw_error[:, joint]),
                    np.max(np.abs(hw_error[:, joint])),
                )
            )
        writer.writerow(
            (
                "all joints",
                _rms(sim_error),
                np.max(np.abs(sim_error)),
                _rms(hw_error),
                np.max(np.abs(hw_error)),
            )
        )


def plot(sim: np.ndarray, hardware: np.ndarray, title: str):
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(4, 2, figsize=(17, 18), constrained_layout=True)
    sim_t = sim["phase_time_s"]
    hw_t = hardware["phase_time_s"]
    sim_q = _joints(sim, "q", "rad")
    hw_q = _joints(hardware, "q", "rad")
    sim_ref = _joints(sim, "qref", "rad")
    hw_ref = _joints(hardware, "qref", "rad")
    sim_error = _joints(sim, "error", "rad")
    hw_error = _joints(hardware, "error", "rad")

    for joint, label in enumerate(JOINT_LABELS):
        row, col = divmod(joint, 2)
        axis = axes[row, col]
        axis.plot(sim_t, sim_ref[:, joint], color="black", label="reference")
        axis.plot(sim_t, sim_q[:, joint], label="simulation")
        axis.plot(hw_t, hw_ref[:, joint], color="black", alpha=0.35)
        axis.plot(hw_t, hw_q[:, joint], label="hardware")
        axis.set_title(label)
        axis.set_xlabel("tracking time [s]")
        axis.set_ylabel("q [rad]")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize="x-small")

    error_figure, error_axes = plt.subplots(
        4, 2, figsize=(17, 18), constrained_layout=True
    )
    for joint, label in enumerate(JOINT_LABELS):
        row, col = divmod(joint, 2)
        axis = error_axes[row, col]
        axis.plot(sim_t, sim_error[:, joint], label="simulation")
        axis.plot(hw_t, hw_error[:, joint], label="hardware")
        axis.axhline(0.0, color="black", linewidth=0.7)
        axis.set_title(label)
        axis.set_xlabel("tracking time [s]")
        axis.set_ylabel("qref - q [rad]")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize="x-small")
    torque_figure, torque_axes = plt.subplots(
        4, 2, figsize=(17, 18), constrained_layout=True
    )
    sim_command = _joints(sim, "tau_command", "nm")
    hw_command = _joints(hardware, "tau_command", "nm")
    hw_ff = _joints(hardware, "tau_ff", "nm")
    hw_pd = _joints(hardware, "tau_pd", "nm")
    for joint, label in enumerate(JOINT_LABELS):
        row, col = divmod(joint, 2)
        axis = torque_axes[row, col]
        axis.plot(sim_t, sim_command[:, joint], label="simulation command")
        axis.plot(hw_t, hw_ff[:, joint], label="hardware feedforward")
        axis.plot(hw_t, hw_pd[:, joint], label="hardware PD")
        axis.plot(hw_t, hw_command[:, joint], label="hardware command")
        axis.axhline(0.0, color="black", linewidth=0.7)
        axis.set_title(label)
        axis.set_xlabel("tracking time [s]")
        axis.set_ylabel("torque [Nm]")
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize="x-small")
    figure.suptitle(title)
    error_figure.suptitle(f"{title}: tracking error")
    torque_figure.suptitle(f"{title}: controller torque")
    return figure, error_figure, torque_figure


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare x7_track simulation and hardware logs"
    )
    parser.add_argument("input", type=pathlib.Path)
    parser.add_argument("hardware_log", nargs="?", type=pathlib.Path)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--error-output", type=pathlib.Path)
    parser.add_argument("--torque-output", type=pathlib.Path)
    parser.add_argument("--summary-csv", type=pathlib.Path)
    parser.add_argument("--title", default="x7_track simulation versus hardware")
    parser.add_argument("--dpi", type=int, default=160)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    import matplotlib

    matplotlib.use("Agg")
    if args.hardware_log is None:
        sim_path = args.input / "simulation" / "simulation.csv"
        hardware_path = args.input / "hardware" / "hardware.csv"
        sim_reference = args.input / "simulation" / "reference.zvs"
        hardware_reference = args.input / "hardware" / "reference.zvs"
        if sim_reference.is_file() and hardware_reference.is_file():
            sim_hash = hashlib.sha256(sim_reference.read_bytes()).digest()
            hardware_hash = hashlib.sha256(hardware_reference.read_bytes()).digest()
            if sim_hash != hardware_hash:
                raise SystemExit(
                    "simulation and hardware bundles use different references"
                )
    else:
        sim_path = args.input
        hardware_path = args.hardware_log
    sim = load_tracking(sim_path)
    hardware = load_tracking(hardware_path)
    print_summary("simulation", sim)
    print_summary("hardware", hardware)
    if args.summary_csv:
        write_summary(args.summary_csv, sim, hardware)
        print(f"wrote {args.summary_csv}")
    figure, error_figure, torque_figure = plot(sim, hardware, args.title)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=args.dpi)
    print(f"wrote {args.output}")
    error_output = args.error_output or args.output.with_name(
        f"{args.output.stem}-error{args.output.suffix}"
    )
    error_output.parent.mkdir(parents=True, exist_ok=True)
    error_figure.savefig(error_output, dpi=args.dpi)
    print(f"wrote {error_output}")
    torque_output = args.torque_output or args.output.with_name(
        f"{args.output.stem}-torque{args.output.suffix}"
    )
    torque_output.parent.mkdir(parents=True, exist_ok=True)
    torque_figure.savefig(torque_output, dpi=args.dpi)
    print(f"wrote {torque_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
