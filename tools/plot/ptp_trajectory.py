#!/usr/bin/env python3
"""Plot Cartesian PTP diagnostics produced by x7_plan_ptp.

The input CSV is authoritative: this tool does not load a robot model or
recompute forward kinematics. An explicit output path is required unless
--show is selected, so inspecting an archived bundle does not modify it.
"""

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
AXES = ("x", "y", "z")


def _required_columns() -> set[str]:
    columns = {
        "schema_version",
        "time_s",
        "profile_derivative_discontinuity",
        "position_error_norm_m",
        "attitude_error_norm_rad",
        "ik_converged",
    }
    for prefix in (
        "target_pos",
        "target_vel",
        "target_acc",
        "target_omega",
        "target_alpha",
        "fk_pos",
        "fk_vel",
        "fk_acc",
        "fk_omega",
        "fk_alpha",
    ):
        unit = {
            "target_pos": "m",
            "target_vel": "m_s",
            "target_acc": "m_s2",
            "target_omega": "rad_s",
            "target_alpha": "rad_s2",
            "fk_pos": "m",
            "fk_vel": "m_s",
            "fk_acc": "m_s2",
            "fk_omega": "rad_s",
            "fk_alpha": "rad_s2",
        }[prefix]
        columns.update(f"{prefix}_{axis}_{unit}" for axis in AXES)
    for prefix in ("target", "fk"):
        columns.update(f"{prefix}_quat_{component}" for component in "wxyz")
    for joint in range(DOF):
        columns.update(
            {
                f"q{joint}_rad",
                f"dq{joint}_rad_s",
                f"ddq{joint}_rad_s2",
                f"joint_limit_margin{joint}_rad",
            }
        )
    return columns


def load_diagnostics(path: pathlib.Path) -> np.ndarray:
    try:
        data = np.genfromtxt(
            path, delimiter=",", names=True, dtype=float, encoding="utf-8"
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"{path}: cannot read PTP diagnostics: {error}") from error
    if data.dtype.names is None or data.size == 0:
        raise SystemExit(f"{path}: no diagnostics rows")
    data = np.atleast_1d(data)
    missing = sorted(_required_columns() - set(data.dtype.names))
    if missing:
        raise SystemExit(f"{path}: missing columns: {', '.join(missing)}")
    versions = np.unique(data["schema_version"])
    if versions.size != 1 or versions[0] != SCHEMA_VERSION:
        raise SystemExit(
            f"{path}: unsupported schema version(s): "
            + ", ".join(f"{value:g}" for value in versions)
        )
    for name in _required_columns():
        if not np.all(np.isfinite(data[name])):
            raise SystemExit(f"{path}: non-finite values in {name}")
    time = data["time_s"]
    if np.any(np.diff(time) <= 0.0):
        raise SystemExit(f"{path}: time must be strictly increasing")
    return data


def _vector(data: np.ndarray, prefix: str, unit: str) -> np.ndarray:
    return np.column_stack([data[f"{prefix}_{axis}_{unit}"] for axis in AXES])


def _joints(data: np.ndarray, prefix: str, unit: str) -> np.ndarray:
    return np.column_stack([data[f"{prefix}{joint}_{unit}"] for joint in range(DOF)])


def quaternion_to_rpy(quaternion: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quaternion, axis=1)
    if np.any(norm <= 0.0):
        raise ValueError("zero-norm quaternion")
    q = quaternion / norm[:, None]
    w, x, y, z = q.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.unwrap(np.column_stack((roll, pitch, yaw)), axis=0)


def _quaternions(data: np.ndarray, prefix: str) -> np.ndarray:
    return np.column_stack([data[f"{prefix}_quat_{component}"] for component in "wxyz"])


def _decorate(axis, ylabel: str) -> None:
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.25)


def _mark_discontinuities(axis, data: np.ndarray) -> None:
    time = data["time_s"]
    marked = data["profile_derivative_discontinuity"] != 0.0
    for value in time[marked & (time > time[0]) & (time < time[-1])]:
        axis.axvline(value, color="0.45", linestyle=":", linewidth=1.0)


def _plot_vector_pair(
    axis, time, target, achieved, ylabel: str, components=AXES
) -> None:
    colors = ("tab:red", "tab:green", "tab:blue")
    for index, (name, color) in enumerate(zip(components, colors)):
        axis.plot(time, target[:, index], color=color, label=f"target {name}")
        axis.plot(
            time,
            achieved[:, index],
            color=color,
            linestyle="--",
            label=f"FK {name}",
        )
    _decorate(axis, ylabel)
    axis.legend(ncol=2, fontsize="small")


def _plot_norm_pair(
    axis,
    time,
    target_linear,
    achieved_linear,
    target_angular,
    achieved_angular,
    linear_label,
    angular_label,
) -> None:
    axis.plot(
        time,
        np.linalg.norm(target_linear, axis=1),
        color="tab:blue",
        label="target linear",
    )
    axis.plot(
        time,
        np.linalg.norm(achieved_linear, axis=1),
        color="tab:blue",
        linestyle="--",
        label="FK linear",
    )
    _decorate(axis, linear_label)
    angular_axis = axis.twinx()
    angular_axis.plot(
        time,
        np.linalg.norm(target_angular, axis=1),
        color="tab:orange",
        label="target angular",
    )
    angular_axis.plot(
        time,
        np.linalg.norm(achieved_angular, axis=1),
        color="tab:orange",
        linestyle="--",
        label="FK angular",
    )
    angular_axis.set_ylabel(angular_label)
    handles, labels = axis.get_legend_handles_labels()
    other_handles, other_labels = angular_axis.get_legend_handles_labels()
    axis.legend(
        handles + other_handles, labels + other_labels, ncol=2, fontsize="small"
    )


def _plot_joints(axis, time, values, ylabel: str) -> None:
    for joint, label in enumerate(JOINT_LABELS):
        axis.plot(time, values[:, joint], label=label)
    _decorate(axis, ylabel)
    axis.legend(ncol=2, fontsize="x-small")


def plot_diagnostics(data: np.ndarray, title: str | None = None):
    import matplotlib.pyplot as plt

    time = data["time_s"]
    figure, axes = plt.subplots(4, 2, figsize=(15, 15), sharex=True)
    position_axis, attitude_axis = axes[0]
    velocity_axis, acceleration_axis = axes[1]
    joint_axis, joint_velocity_axis = axes[2]
    joint_acceleration_axis, quality_axis = axes[3]

    _plot_vector_pair(
        position_axis,
        time,
        _vector(data, "target_pos", "m"),
        _vector(data, "fk_pos", "m"),
        "TCP position [m]",
    )
    target_rpy = quaternion_to_rpy(_quaternions(data, "target"))
    achieved_rpy = quaternion_to_rpy(_quaternions(data, "fk"))
    _plot_vector_pair(
        attitude_axis,
        time,
        target_rpy,
        achieved_rpy,
        "TCP RPY [rad]",
        ("roll", "pitch", "yaw"),
    )
    attitude_axis.set_title("RPY is unwrapped for display only")

    _plot_norm_pair(
        velocity_axis,
        time,
        _vector(data, "target_vel", "m_s"),
        _vector(data, "fk_vel", "m_s"),
        _vector(data, "target_omega", "rad_s"),
        _vector(data, "fk_omega", "rad_s"),
        "linear speed [m/s]",
        "angular speed [rad/s]",
    )
    _mark_discontinuities(velocity_axis, data)
    _plot_norm_pair(
        acceleration_axis,
        time,
        _vector(data, "target_acc", "m_s2"),
        _vector(data, "fk_acc", "m_s2"),
        _vector(data, "target_alpha", "rad_s2"),
        _vector(data, "fk_alpha", "rad_s2"),
        "linear acceleration [m/s²]",
        "angular acceleration [rad/s²]",
    )
    _mark_discontinuities(acceleration_axis, data)

    _plot_joints(joint_axis, time, _joints(data, "q", "rad"), "q [rad]")
    _plot_joints(
        joint_velocity_axis,
        time,
        _joints(data, "dq", "rad_s"),
        "sampled dq/dt [rad/s]",
    )
    _plot_joints(
        joint_acceleration_axis,
        time,
        _joints(data, "ddq", "rad_s2"),
        "sampled d²q/dt² [rad/s²]",
    )

    floor = np.finfo(float).eps
    quality_axis.semilogy(
        time,
        np.maximum(data["position_error_norm_m"], floor),
        label="position error [m]",
    )
    quality_axis.semilogy(
        time,
        np.maximum(data["attitude_error_norm_rad"], floor),
        label="attitude error [rad]",
    )
    _decorate(quality_axis, "IK/FK error [log scale]")
    margin_axis = quality_axis.twinx()
    margins = np.column_stack(
        [data[f"joint_limit_margin{joint}_rad"] for joint in range(DOF)]
    )
    minimum_margin = np.min(margins, axis=1)
    margin_axis.plot(
        time, minimum_margin, color="tab:green", label="minimum joint margin"
    )
    failed = data["ik_converged"] == 0.0
    if np.any(failed):
        margin_axis.scatter(
            time[failed],
            minimum_margin[failed],
            color="tab:red",
            marker="x",
            label="IK warning",
        )
    margin_axis.set_ylabel("joint-limit margin [rad]")
    handles, labels = quality_axis.get_legend_handles_labels()
    other_handles, other_labels = margin_axis.get_legend_handles_labels()
    quality_axis.legend(
        handles + other_handles, labels + other_labels, fontsize="small"
    )

    for axis in axes[-1]:
        axis.set_xlabel("time [s]")
    if title:
        figure.suptitle(title)
    figure.tight_layout(rect=(0, 0, 1, 0.98 if title else 1))
    return figure


def print_summary(data: np.ndarray) -> None:
    velocities = _joints(data, "dq", "rad_s")
    accelerations = _joints(data, "ddq", "rad_s2")
    margins = np.column_stack(
        [data[f"joint_limit_margin{joint}_rad"] for joint in range(DOF)]
    )
    print(
        f"{len(data)} samples, duration {data['time_s'][-1]:.6g} s\n"
        f"max TCP errors: {np.max(data['position_error_norm_m']):.6g} m / "
        f"{np.max(data['attitude_error_norm_rad']):.6g} rad\n"
        f"max sampled joint rate/acceleration: "
        f"{np.max(np.abs(velocities)):.6g} rad/s / "
        f"{np.max(np.abs(accelerations)):.6g} rad/s²\n"
        f"minimum joint-limit margin: {np.min(margins):.6g} rad"
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot x7_plan_ptp trajectory diagnostics"
    )
    parser.add_argument(
        "diagnostics", type=pathlib.Path, help="trajectory diagnostics CSV"
    )
    parser.add_argument(
        "-o", "--output", type=pathlib.Path, help="PNG, PDF, or SVG output path"
    )
    parser.add_argument(
        "--show", action="store_true", help="open an interactive plot window"
    )
    parser.add_argument("--title", help="figure title")
    parser.add_argument(
        "--dpi", type=int, default=160, help="raster output resolution (default: 160)"
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
    data = load_diagnostics(args.diagnostics)
    title = args.title or f"PTP trajectory: {args.diagnostics.name}"
    figure = plot_diagnostics(data, title)
    print_summary(data)
    if args.output:
        if args.output.resolve() == args.diagnostics.resolve():
            raise SystemExit("refusing to overwrite the diagnostics CSV")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.output, dpi=args.dpi)
        print(f"wrote {args.output}")
    if args.show:
        import matplotlib.pyplot as plt

        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
