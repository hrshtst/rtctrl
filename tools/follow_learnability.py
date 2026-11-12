#!/usr/bin/env python3
"""Learnability evidence from x7_follow logs.

Turns follow logs (FollowCsvLog, schema_version 1) into the evidence a
learning controller acting through the position command needs: how
structured the tracking error is (anatomy), how repeatable it is over runs
of one reference (repeat), and how well a parametric feedforward built only
from the desired trajectory predicts it on a run it was not fitted on
(predict).

Sign convention: e = q - qref (measured minus reference), the log's own
error{j}_rad column. Only rows with phase == tracking are analysed (they
must form one contiguous block); dt is the median step of time_s over them.

Units: --json writes SI units (rad, rad/s, rad/s^2, s, Nm, Nm/rad,
Nm s/rad, kg m^2) and null for every value that is not identifiable; the
tables on stdout show errors in mrad and times in ms, as their headers
state.

  uv run --project tools python tools/follow_learnability.py anatomy LOG [LOG ...]
  uv run --project tools python tools/follow_learnability.py repeat LOG LOG [LOG ...]
  uv run --project tools python tools/follow_learnability.py predict \\
      --model-torque BIN --model ZTK --run NAME=LOG --run NAME=LOG [...]

Exit status: 0 on success, 1 on invalid input (the reason on stderr), 2 on
a usage error.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from typing import Callable, Iterable

import numpy as np

DOF = 8                      # canonical joints 0..7 (7 = gripper)
ARM = 7                      # arm joints 0..6
SCHEMA_VERSION = 1
PHASES = ("home", "home_correction", "offset_ramp", "tracking", "finalizing",
          "abort_hold")
TRACKING = "tracking"
MIN_TRACKING_SAMPLES = 10
REFERENCE_TOLERANCE_RAD = 1e-9   # repeat: identical references
LENGTH_TOLERANCE_SAMPLES = 1     # repeat: tracking lengths
TAU_RMS_MIN_NM = 0.02            # anatomy: tau identifies a compliance
COMPLIANCE_ZERO_RAD = 1e-12      # anatomy: |1/K| * rms(tau) below this is zero
NEGLIGIBLE_RMS = 1e-6            # any other regressor, in its own units
RIDGE_PER_SAMPLE = 1e-6          # predict: penalty = this * fitting samples
FORMAT = "rtctrl-follow-learnability"
FORMAT_VERSION = 1
SIGN_CONVENTION = ("e = q - qref (measured minus reference), "
                   "the log's error{j}_rad column")

UNIT_SUFFIX = {"qref": "_rad", "dqref": "_rad_s", "ddqref": "_rad_s2",
               "q": "_rad", "dq": "_rad_s", "tau": "_nm", "error": "_rad",
               "applied": ""}

REGRESSORS = ("g", "tau_dyn", "ddq", "dq", "coulomb", "constant")
REGRESSOR_SETS = {"full": REGRESSORS,
                  "gravity_lag": ("g", "dq", "constant"),
                  "lag_only": ("dq", "constant")}
WEIGHT_UNITS = {"g": "rad/Nm", "tau_dyn": "rad/Nm", "ddq": "s^2",
                "dq": "s", "coulomb": "rad", "constant": "rad"}
TORQUE_TERMS = {"g": "a", "tau_dyn": "b", "ddq": "J_kg_m2",
                "dq": "fv_nm_s_per_rad", "coulomb": "fc_nm",
                "constant": "c_nm"}
WEIGHTS_NOTE = ("individual weights are not identified when regressors are "
                "collinear or a model term is small; the held-out residual, "
                "not the weights, is the result to read")


class InputError(Exception):
    """Invalid input; the message names the file and the problem."""


def column(quantity: str, joint: int) -> str:
    """The follow-log column of `quantity` for canonical joint `joint`."""
    return f"{quantity}{joint}{UNIT_SUFFIX[quantity]}"


def joint_columns(quantities: Iterable[str], joints: Iterable[int]) -> list:
    return [column(quantity, j) for j in joints for quantity in quantities]


ANATOMY_COLUMNS = ["command_mode", "applied_seq"] + joint_columns(
    ("qref", "dqref", "ddqref", "q", "dq", "tau", "error", "applied"),
    range(DOF))
REPEAT_COLUMNS = joint_columns(("qref", "q", "error"), range(DOF))
PREDICT_COLUMNS = (joint_columns(("qref",), range(DOF))
                   + joint_columns(("error", "tau"), range(ARM)))


@dataclass
class Run:
    """The tracking rows of one follow log, numeric columns as arrays."""

    path: str
    time: np.ndarray
    dt: float
    data: dict

    @property
    def samples(self) -> int:
        return len(self.time)

    @property
    def duration(self) -> float:
        return float(self.time[-1] - self.time[0])

    def joint(self, quantity: str, j: int) -> np.ndarray:
        return self.data[column(quantity, j)]


def load_tracking(path, required: Iterable[str]) -> Run:
    """Reads a follow log and keeps its tracking rows.

    Every row must have the header's field count, schema_version 1 and a
    known phase; the tracking rows must be contiguous, at least
    MIN_TRACKING_SAMPLES long, strictly increasing in time_s and finite in
    every required column. Columns of the other rows are not parsed.
    """
    path = str(path)
    numeric = list(dict.fromkeys(["time_s", *required]))
    try:
        with open(path, newline="") as stream:
            reader = csv.reader(stream)
            header = next(reader, None)
            if not header:
                raise InputError(f"{path}: empty file (no header)")
            index = {name: i for i, name in enumerate(header)}
            missing = [name for name in ["schema_version", "phase", *numeric]
                       if name not in index]
            if missing:
                raise InputError(f"{path}: not a follow log with the columns "
                                 f"this analysis needs; missing: "
                                 f"{', '.join(missing)}")
            width = len(header)
            raw = {name: [] for name in numeric}
            state = "before"  # before, inside, after the tracking block
            for row in reader:
                line = reader.line_num
                if len(row) != width:
                    raise InputError(f"{path}:{line}: expected {width} "
                                     f"fields, found {len(row)}")
                version = row[index["schema_version"]].strip()
                if version != str(SCHEMA_VERSION):
                    raise InputError(f"{path}:{line}: unsupported "
                                     f"schema_version '{version}' (expected "
                                     f"{SCHEMA_VERSION})")
                phase = row[index["phase"]].strip()
                if phase not in PHASES:
                    raise InputError(f"{path}:{line}: unknown phase "
                                     f"'{phase}'")
                if phase != TRACKING:
                    if state == "inside":
                        state = "after"
                    continue
                if state == "after":
                    raise InputError(f"{path}:{line}: the tracking rows are "
                                     f"not one contiguous block")
                state = "inside"
                for name in numeric:
                    raw[name].append(row[index[name]])
    except OSError as error:
        raise InputError(f"{path}: cannot read: {error.strerror}") from error
    except (csv.Error, UnicodeDecodeError) as error:
        raise InputError(f"{path}: not a readable CSV file: {error}") \
            from error
    count = len(raw["time_s"])
    if count < MIN_TRACKING_SAMPLES:
        raise InputError(f"{path}: {count} tracking samples; at least "
                         f"{MIN_TRACKING_SAMPLES} are needed")
    data = {}
    for name in numeric:
        try:
            values = np.array(raw[name], dtype=float)
        except ValueError as error:
            raise InputError(f"{path}: column {name}: a tracking row is not "
                             f"numeric") from error
        if not np.all(np.isfinite(values)):
            raise InputError(f"{path}: column {name}: a tracking row is not "
                             f"finite")
        data[name] = values
    time = data["time_s"]
    if not np.all(np.diff(time) > 0.0):
        raise InputError(f"{path}: time_s does not increase strictly over "
                         f"the tracking rows")
    dt = float(np.median(np.diff(time)))
    return Run(path=path, time=time, dt=dt, data=data)


# ---------------------------------------------------------------- numerics

def rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x))))


def aggregate_rms(values: Iterable[float]) -> float:
    """RMS over joints of equal sample count, from their per-joint RMS."""
    values = np.asarray(list(values), dtype=float)
    return float(np.sqrt(np.mean(np.square(values))))


def band_split(x: np.ndarray, dt: float, split_hz: float):
    """Zero-phase split of x into (below, above) split_hz by FFT masking of
    the mean-removed signal; the mean stays in the lower band, frequencies
    equal to split_hz too (within 1e-9 relative, so that the rounding of a
    median dt cannot move a bin that lies on the split). below + above ==
    x."""
    mean = float(np.mean(x))
    spectrum = np.fft.rfft(x - mean)
    freq = np.fft.rfftfreq(len(x), dt)
    below = np.fft.irfft(
        np.where(freq <= split_hz * (1.0 + 1e-9), spectrum, 0.0),
        len(x)) + mean
    return below, x - below


def r_squared(y: np.ndarray, residual: np.ndarray) -> float | None:
    total = float(np.sum(np.square(y - np.mean(y))))
    if total == 0.0:
        return None
    return 1.0 - float(np.sum(np.square(residual))) / total


def num(x) -> float | None:
    """A JSON number, or null for an absent or non-finite value."""
    if x is None:
        return None
    x = float(x)
    return x if math.isfinite(x) else None


@dataclass
class Fit:
    """Linear fit y ~ sum(weights[name] * columns[name]).

    A column whose RMS over the fitting rows is below its threshold, or
    that the caller excluded, is dropped: its weight is None, never fitted.
    `identifiable` is False when the plain least-squares problem has no
    unique solution (too few rows, or linearly dependent kept columns);
    every weight is then None.
    """

    names: tuple
    weights: dict
    dropped: list
    rows: int
    identifiable: bool = True
    kept: list = field(default_factory=list)

    def evaluate(self, columns: dict) -> np.ndarray:
        length = len(next(iter(columns.values())))
        out = np.zeros(length)
        for name in self.kept:
            out = out + self.weights[name] * columns[name]
        return out


def fit_columns(columns: dict, y: np.ndarray, names: Iterable[str],
                ridge: float = 0.0, thresholds: dict | None = None,
                exclude: Iterable[str] = ()) -> Fit:
    """Least squares (ridge == 0) or ridge least squares on RMS-normalised
    columns with penalty ridge * rows. Negligible columns are dropped."""
    names = tuple(names)
    thresholds = thresholds or {}
    exclude = set(exclude)
    rows = len(y)
    fit = Fit(names=names, weights=dict.fromkeys(names), dropped=[],
              rows=rows)
    if rows == 0:
        fit.identifiable = False
        return fit
    for name in names:
        if (name in exclude
                or rms(columns[name]) < thresholds.get(name, NEGLIGIBLE_RMS)):
            fit.dropped.append(name)
        else:
            fit.kept.append(name)
    if not fit.kept:
        return fit
    phi = np.column_stack([columns[name] for name in fit.kept])
    scale = np.sqrt(np.mean(np.square(phi), axis=0))
    a = phi / scale
    if ridge > 0.0:
        normal = a.T @ a + ridge * rows * np.eye(len(fit.kept))
        w = np.linalg.solve(normal, a.T @ y)
    else:
        if rows <= len(fit.kept):
            fit.identifiable = False
            fit.kept = []
            return fit
        w, _, rank, _ = np.linalg.lstsq(a, y, rcond=None)
        if rank < len(fit.kept):
            fit.identifiable = False
            fit.kept = []
            return fit
    for name, value in zip(fit.kept, w / scale):
        fit.weights[name] = float(value)
    return fit


def fit_quality(fit: Fit, columns: dict, y: np.ndarray):
    """(R2, residual RMS) of an identifiable fit, else (None, None)."""
    if not fit.identifiable:
        return None, None
    residual = y - fit.evaluate(columns)
    return r_squared(y, residual), rms(residual)


# ----------------------------------------------------------------- anatomy

def lag_stiffness_fit(e: np.ndarray, tau: np.ndarray,
                      dqref: np.ndarray) -> dict:
    """e = -tau/K - T*dqref + c by least squares.

    K is null when tau's RMS is below TAU_RMS_MIN_NM or the fitted
    compliance 1/K is numerically zero (its contribution |1/K| * rms(tau)
    below COMPLIANCE_ZERO_RAD); T and c are then fitted without it."""
    columns = {"tau": -tau, "dqref": -dqref, "constant": np.ones_like(e)}
    names = ("tau", "dqref", "constant")
    thresholds = {"tau": TAU_RMS_MIN_NM}
    reason = None
    if rms(tau) < TAU_RMS_MIN_NM:
        reason = "tau_rms_below_threshold"
    fit = fit_columns(columns, e, names, thresholds=thresholds)
    compliance = fit.weights["tau"]
    if (compliance is not None
            and abs(compliance) * rms(tau) < COMPLIANCE_ZERO_RAD):
        reason = "compliance_numerically_zero"
        fit = fit_columns(columns, e, names, thresholds=thresholds,
                          exclude=("tau",))
    if not fit.identifiable and reason is None:
        reason = "fit_not_identifiable"
    r2, residual = fit_quality(fit, columns, e)
    compliance = fit.weights["tau"]
    return {"K_nm_per_rad": num(1.0 / compliance) if compliance else None,
            "T_s": num(fit.weights["dqref"]),
            "c_rad": num(fit.weights["constant"]),
            "r2": num(r2), "residual_rms_rad": num(residual),
            "rows": fit.rows, "dropped": list(fit.dropped),
            "identifiable": fit.identifiable, "K_null_reason": reason}


def servo_law_fit(tau: np.ndarray, applied: np.ndarray, q: np.ndarray,
                  dq: np.ndarray) -> dict:
    """tau = Ks*(applied - q) - Ds*dq + b by least squares."""
    columns = {"applied_minus_q": applied - q, "dq": -dq,
               "constant": np.ones_like(tau)}
    fit = fit_columns(columns, tau, ("applied_minus_q", "dq", "constant"))
    r2, residual = fit_quality(fit, columns, tau)
    return {"Ks_nm_per_rad": num(fit.weights["applied_minus_q"]),
            "Ds_nm_s_per_rad": num(fit.weights["dq"]),
            "b_nm": num(fit.weights["constant"]),
            "r2": num(r2), "residual_rms_nm": num(residual),
            "rows": fit.rows, "dropped": list(fit.dropped),
            "identifiable": fit.identifiable}


def anatomy(run: Run, split_hz: float) -> dict:
    """The per-joint error anatomy of one log's tracking rows."""
    modes = np.unique(run.data["command_mode"])
    if len(modes) != 1:
        raise InputError(f"{run.path}: command_mode changes during tracking "
                         f"({', '.join(str(int(m)) for m in modes)})")
    applied_valid = run.data["applied_seq"] > 0
    joints = []
    for j in range(DOF):
        e = run.joint("error", j)
        dqref = run.joint("dqref", j)
        ddqref = run.joint("ddqref", j)
        tau = run.joint("tau", j)
        _, above = band_split(e, run.dt, split_hz)
        joints.append({
            "joint": j,
            "e_rms_rad": rms(e),
            "e_max_abs_rad": float(np.max(np.abs(e))),
            "e_mean_rad": float(np.mean(e)),
            "e_rms_above_split_rad": rms(above),
            "dqref_peak_abs_rad_s": float(np.max(np.abs(dqref))),
            "dqref_first_abs_rad_s": float(abs(dqref[0])),
            "dqref_last_abs_rad_s": float(abs(dqref[-1])),
            "ddqref_first_abs_rad_s2": float(abs(ddqref[0])),
            "ddqref_last_abs_rad_s2": float(abs(ddqref[-1])),
            "tau_rms_nm": rms(tau),
            "lag_stiffness": lag_stiffness_fit(e, tau, dqref),
            "servo_law": servo_law_fit(
                tau[applied_valid], run.joint("applied", j)[applied_valid],
                run.joint("q", j)[applied_valid],
                run.joint("dq", j)[applied_valid]),
        })
    return {
        "path": run.path,
        "samples": run.samples,
        "duration_s": run.duration,
        "dt_s": run.dt,
        "command_mode": int(modes[0]),
        # Over all 8 joints and over the 7 arm joints: the RMS over joints
        # of each per-joint RMS (aggregate_rms); follow_learn_study.py
        # reads e_rms_above_split_rad as its hf_rms_rad and hf_arm_rms_rad.
        "aggregate": {
            label: {key: aggregate_rms(joint[key] for joint in selected)
                    for key in ("e_rms_rad", "e_rms_above_split_rad")}
            for label, selected in (("all", joints), ("arm", joints[:ARM]))
        },
        "joints": joints,
    }


# ------------------------------------------------------------------ repeat

REPEAT_RMS_KEYS = ("repeatable_rms_rad", "nonrepeatable_rms_rad",
                   "repeatable_below_split_rms_rad",
                   "nonrepeatable_below_split_rms_rad",
                   "repeatable_above_split_rms_rad",
                   "nonrepeatable_above_split_rms_rad")


def debiased(repeatable: float, nonrepeatable: float, runs: int) -> float:
    """rms(m) with the noise that survives averaging N runs removed."""
    return math.sqrt(max(0.0, repeatable ** 2 - nonrepeatable ** 2 / runs))


def check_same_reference(runs: list) -> tuple[int, float]:
    """(common sample count, max |qref difference|) of runs of one
    reference, or InputError naming the logs that differ."""
    lengths = [run.samples for run in runs]
    if max(lengths) - min(lengths) > LENGTH_TOLERANCE_SAMPLES:
        listing = ", ".join(f"{run.path} ({run.samples})" for run in runs)
        raise InputError(f"tracking lengths differ by more than "
                         f"{LENGTH_TOLERANCE_SAMPLES} sample: {listing}")
    n = min(lengths)
    first = runs[0]
    worst = 0.0
    differing = []
    for run in runs[1:]:
        diff = max(float(np.max(np.abs(run.joint("qref", j)[:n]
                                       - first.joint("qref", j)[:n])))
                   for j in range(DOF))
        worst = max(worst, diff)
        if diff > REFERENCE_TOLERANCE_RAD:
            differing.append(f"{run.path} (max |qref difference| "
                             f"{diff:.3g} rad)")
    if differing:
        raise InputError(f"references differ from {first.path} over the "
                         f"common {n} tracking samples (limit "
                         f"{REFERENCE_TOLERANCE_RAD:g} rad): "
                         f"{'; '.join(differing)}")
    return n, worst


def repeat(runs: list, split_hz: float) -> dict:
    """Repeatable and non-repeatable error over N >= 2 runs of one
    reference: m = mean of e over runs, r_i = (e_i - m) * sqrt(N/(N-1))."""
    count = len(runs)
    if count < 2:
        raise InputError(f"repeat needs at least 2 logs, got {count}")
    n, reference_difference = check_same_reference(runs)
    dt = runs[0].dt
    scale = math.sqrt(count / (count - 1))
    joints = []
    for j in range(DOF):
        errors = np.array([run.joint("error", j)[:n] for run in runs])
        mean = errors.mean(axis=0)
        residuals = (errors - mean) * scale
        mean_below, mean_above = band_split(mean, dt, split_hz)
        split = [band_split(r, dt, split_hz) for r in residuals]
        residual_below = np.array([below for below, _ in split])
        residual_above = np.array([above for _, above in split])
        first_q = np.array([run.joint("q", j)[0] for run in runs])
        entry = {
            "joint": j,
            "repeatable_rms_rad": rms(mean),
            "nonrepeatable_rms_rad": rms(residuals),
            "repeatable_below_split_rms_rad": rms(mean_below),
            "nonrepeatable_below_split_rms_rad": rms(residual_below),
            "repeatable_above_split_rms_rad": rms(mean_above),
            "nonrepeatable_above_split_rms_rad": rms(residual_above),
            "initial_spread_rad": float(np.max(first_q) - np.min(first_q)),
        }
        entry["repeatable_debiased_rms_rad"] = debiased(
            entry["repeatable_rms_rad"], entry["nonrepeatable_rms_rad"],
            count)
        joints.append(entry)

    def aggregate(selected):
        out = {key: aggregate_rms(joint[key] for joint in selected)
               for key in REPEAT_RMS_KEYS}
        out["repeatable_debiased_rms_rad"] = debiased(
            out["repeatable_rms_rad"], out["nonrepeatable_rms_rad"], count)
        out["initial_spread_max_rad"] = max(joint["initial_spread_rad"]
                                            for joint in selected)
        return out

    return {
        "runs": [{"path": run.path, "samples": run.samples, "dt_s": run.dt}
                 for run in runs],
        "run_count": count,
        "common_samples": n,
        "dt_s": dt,
        "max_reference_difference_rad": reference_difference,
        "joints": joints,
        "aggregate": {"all": aggregate(joints), "arm": aggregate(joints[:ARM])},
    }


# ----------------------------------------------------------------- predict

TorqueProvider = Callable[[np.ndarray, np.ndarray, np.ndarray], tuple]


def gaussian_smooth(x: np.ndarray, sigma_s: float, dt: float) -> np.ndarray:
    """Zero-phase Gaussian smoothing, kernel truncated at 4 sigma; the ends
    are padded with the edge values, so the length is preserved. sigma_s 0
    returns a copy."""
    x = np.asarray(x, dtype=float)
    if sigma_s == 0.0:
        return x.copy()
    half = max(1, int(math.ceil(4.0 * sigma_s / dt - 1e-9)))
    kernel = np.exp(-0.5 * (np.arange(-half, half + 1) * dt / sigma_s) ** 2)
    kernel /= kernel.sum()
    return np.convolve(np.pad(x, half, mode="edge"), kernel, mode="valid")


def reference_derivatives(qref: np.ndarray, dt: float, sigma_s: float):
    """(q, dq, ddq), each (n, DOF): the smoothed reference and its central
    differences (numpy.gradient, applied twice)."""
    q = np.column_stack([gaussian_smooth(qref[:, j], sigma_s, dt)
                         for j in range(qref.shape[1])])
    dq = np.gradient(q, dt, axis=0)
    ddq = np.gradient(dq, dt, axis=0)
    return q, dq, ddq


def write_reference_csv(path: str, q: np.ndarray, dq: np.ndarray,
                        ddq: np.ndarray) -> None:
    names = joint_columns(("qref", "dqref", "ddqref"), range(DOF))
    with open(path, "w", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(names)
        for k in range(q.shape[0]):
            writer.writerow([f"{value:.17g}" for j in range(DOF)
                             for value in (q[k, j], dq[k, j], ddq[k, j])])


TORQUE_SCHEMA_VERSION = 1
TORQUE_HEADER = (["schema_version", "row"]
                 + [f"gravity{j}_nm" for j in range(DOF)]
                 + [f"inverse_dynamics{j}_nm" for j in range(DOF)])


def header_mismatch(header: list, expected: list) -> str | None:
    """How header differs from expected (first difference), or None."""
    for k, name in enumerate(expected):
        if k >= len(header):
            return f"column {name} is missing"
        if header[k] != name:
            return f"column {k + 1} is '{header[k]}', expected '{name}'"
    if len(header) > len(expected):
        return f"unexpected extra column '{header[len(expected)]}'"
    return None


def read_torque_csv(path: str, rows: int, source: str):
    """(gravity, inverse_dynamics), each (rows, DOF), from the model-torque
    output: exactly TORQUE_HEADER, schema_version 1 on every row, one row
    per input row in order."""
    width = len(TORQUE_HEADER)
    table = []
    try:
        with open(path, newline="") as stream:
            reader = csv.reader(stream)
            header = next(reader, None)
            problem = ("no header" if header is None
                       else header_mismatch(header, TORQUE_HEADER))
            if problem:
                raise InputError(f"{source}: unexpected output header: "
                                 f"{problem}")
            for row in reader:
                line = reader.line_num
                if len(row) != width:
                    raise InputError(f"{source}: output line {line}: "
                                     f"expected {width} fields, found "
                                     f"{len(row)}")
                if row[0] != str(TORQUE_SCHEMA_VERSION):
                    raise InputError(f"{source}: output line {line}: "
                                     f"schema_version '{row[0]}', expected "
                                     f"{TORQUE_SCHEMA_VERSION}")
                table.append(row)
    except OSError as error:
        raise InputError(f"{source}: cannot read its output: "
                         f"{error.strerror}") from error
    except (csv.Error, UnicodeDecodeError) as error:
        raise InputError(f"{source}: output is not a readable CSV file: "
                         f"{error}") from error
    if len(table) != rows:
        raise InputError(f"{source}: {len(table)} output rows for {rows} "
                         f"input rows")
    try:
        index = np.array([int(row[1]) for row in table])
        gravity = np.array([[float(v) for v in row[2:2 + DOF]]
                            for row in table]).reshape(rows, DOF)
        dynamics = np.array([[float(v) for v in row[2 + DOF:]]
                             for row in table]).reshape(rows, DOF)
    except ValueError as error:
        raise InputError(f"{source}: output is not numeric") from error
    if rows and not np.array_equal(index, index[0] + np.arange(rows)):
        raise InputError(f"{source}: output rows are not consecutive")
    return gravity, dynamics


def binary_torque_provider(binary: str, model: str) -> TorqueProvider:
    """Model torques from the x7_model_torque binary, through temp files."""
    def provider(q, dq, ddq):
        with tempfile.TemporaryDirectory(prefix="follow_learnability-") as tmp:
            source = os.path.join(tmp, "reference.csv")
            target = os.path.join(tmp, "torque.csv")  # must not exist yet
            write_reference_csv(source, q, dq, ddq)
            try:
                done = subprocess.run(
                    [binary, "--model", model, "--input", source,
                     "--output", target],
                    capture_output=True, text=True, check=False)
            except OSError as error:
                raise InputError(f"cannot run the model-torque binary "
                                 f"{binary}: {error.strerror}") from error
            if done.returncode != 0:
                detail = done.stderr.strip() or done.stdout.strip()
                raise InputError(f"{binary} exited with status "
                                 f"{done.returncode}: {detail}")
            return read_torque_csv(target, q.shape[0], binary)
    return provider


@dataclass
class PredictRun:
    """One run's regressors, error and measured torque per arm joint."""

    name: str
    run: Run
    columns: list   # per arm joint: {regressor name: array}
    error: list     # per arm joint: e
    tau: list       # per arm joint: measured tau


def prepare_predict_run(name: str, run: Run, provider: TorqueProvider,
                        sigma_s: float, v0: float) -> PredictRun:
    qref = np.column_stack([run.joint("qref", j) for j in range(DOF)])
    q, dq, ddq = reference_derivatives(qref, run.dt, sigma_s)
    result = provider(q, dq, ddq)
    try:
        gravity, dynamics = (np.asarray(part, dtype=float) for part in result)
    except (TypeError, ValueError) as error:
        raise InputError(f"run {name}: the torque provider returned no "
                         f"(gravity, inverse_dynamics) pair") from error
    for label, part in (("gravity", gravity), ("inverse dynamics", dynamics)):
        if part.shape != (run.samples, DOF):
            raise InputError(f"run {name}: {label} torques have shape "
                             f"{part.shape}, expected "
                             f"{(run.samples, DOF)}")
        if not np.all(np.isfinite(part)):
            raise InputError(f"run {name}: {label} torques are not finite")
    columns = []
    for j in range(ARM):
        columns.append({"g": gravity[:, j],
                        "tau_dyn": dynamics[:, j] - gravity[:, j],
                        "ddq": ddq[:, j], "dq": dq[:, j],
                        "coulomb": np.tanh(dq[:, j] / v0),
                        "constant": np.ones(run.samples)})
    return PredictRun(name=name, run=run, columns=columns,
                      error=[run.joint("error", j) for j in range(ARM)],
                      tau=[run.joint("tau", j) for j in range(ARM)])


def pooled(runs: list, j: int, target: str):
    """Joint j's regressor columns and target stacked over runs."""
    columns = {name: np.concatenate([run.columns[j][name] for run in runs])
               for name in REGRESSORS}
    y = np.concatenate([getattr(run, target)[j] for run in runs])
    return columns, y


def ridge_fit(columns: dict, y: np.ndarray, names) -> Fit:
    return fit_columns(columns, y, names, ridge=RIDGE_PER_SAMPLE)


def evaluation(run: PredictRun, fits: list) -> dict:
    """Baseline and residual RMS of run's e under per-joint fits."""
    joints = []
    for j, fit in enumerate(fits):
        e = run.error[j]
        residual = e - fit.evaluate(run.columns[j])
        joints.append({"joint": j, "baseline_rms_rad": rms(e),
                       "residual_rms_rad": rms(residual),
                       "dropped": list(fit.dropped)})
    return {"run": run.name,
            "baseline_rms_rad": aggregate_rms(
                joint["baseline_rms_rad"] for joint in joints),
            "residual_rms_rad": aggregate_rms(
                joint["residual_rms_rad"] for joint in joints),
            "joints": joints}


def regressor_set(prepared: list, names) -> dict:
    held_out = []
    for k, run in enumerate(prepared):
        others = prepared[:k] + prepared[k + 1:]
        fits = [ridge_fit(*pooled(others, j, "error"), names)
                for j in range(ARM)]
        held_out.append(evaluation(run, fits))
    ratios = [entry["baseline_rms_rad"] / entry["residual_rms_rad"]
              if entry["residual_rms_rad"] > 0.0 else None
              for entry in held_out]
    mean_ratio = (None if any(ratio is None for ratio in ratios)
                  else float(np.mean(ratios)))
    in_sample = [evaluation(run, [ridge_fit(*pooled([run], j, "error"), names)
                                  for j in range(ARM)])
                 for run in prepared]
    return {"columns": list(names),
            "leave_one_out": {"held_out": held_out,
                              "mean_baseline_over_residual": mean_ratio},
            "in_sample": in_sample}


def pooled_full_weights(prepared: list) -> list:
    out = []
    for j in range(ARM):
        columns, e = pooled(prepared, j, "error")
        fit = ridge_fit(columns, e, REGRESSORS)
        r2, residual = fit_quality(fit, columns, e)
        w = fit.weights
        out.append({
            "joint": j,
            "weights": {name: num(w[name]) for name in REGRESSORS},
            "dropped": list(fit.dropped),
            "stiffness_nm_per_rad": num(-1.0 / w["g"]) if w["g"] else None,
            "lag_s": num(-w["dq"]) if w["dq"] is not None else None,
            "coulomb_rad": num(w["coulomb"]),
            "constant_rad": num(w["constant"]),
            "r2": num(r2), "residual_rms_rad": num(residual),
            "rows": fit.rows,
        })
    return out


def measured_torque_check(prepared: list) -> list:
    out = []
    for j in range(ARM):
        columns, tau = pooled(prepared, j, "tau")
        fit = ridge_fit(columns, tau, REGRESSORS)
        r2, residual = fit_quality(fit, columns, tau)
        entry = {"joint": j}
        entry.update({TORQUE_TERMS[name]: num(fit.weights[name])
                      for name in REGRESSORS})
        entry.update({"dropped": list(fit.dropped), "r2": num(r2),
                      "residual_rms_nm": num(residual),
                      "tau_rms_nm": rms(tau), "rows": fit.rows})
        out.append(entry)
    return out


def predict(runs: list, provider: TorqueProvider, sigma_s: float = 0.04,
            v0: float = 0.05) -> dict:
    """Leave-one-run-out test of a desired-trajectory feedforward.

    runs: (name, Run) pairs, at least 2, names unique. provider(q, dq, ddq)
    returns (gravity, inverse_dynamics), each (n, DOF) in Nm, for the
    smoothed reference."""
    if len(runs) < 2:
        raise InputError(f"predict needs at least 2 runs, got {len(runs)}")
    names = [name for name, _ in runs]
    duplicates = sorted({name for name in names if names.count(name) > 1})
    if duplicates:
        raise InputError(f"run names are not unique: {', '.join(duplicates)}")
    if not (math.isfinite(sigma_s) and sigma_s >= 0.0):
        raise InputError("--smooth-sigma-s must be finite and nonnegative")
    if not (math.isfinite(v0) and v0 > 0.0):
        raise InputError("--coulomb-v0 must be finite and positive")
    prepared = [prepare_predict_run(name, run, provider, sigma_s, v0)
                for name, run in runs]
    return {
        "smooth_sigma_s": sigma_s,
        "coulomb_v0_rad_s": v0,
        "ridge_penalty_per_sample": RIDGE_PER_SAMPLE,
        "negligible_column_rms": NEGLIGIBLE_RMS,
        "weight_units": dict(WEIGHT_UNITS),
        "runs": [{"name": item.name, "path": item.run.path,
                  "samples": item.run.samples, "dt_s": item.run.dt,
                  "duration_s": item.run.duration} for item in prepared],
        "sets": {label: regressor_set(prepared, names)
                 for label, names in REGRESSOR_SETS.items()},
        "pooled_full": pooled_full_weights(prepared),
        "pooled_full_note": WEIGHTS_NOTE,
        "measured_torque": measured_torque_check(prepared),
        "measured_torque_note": WEIGHTS_NOTE,
    }


# ------------------------------------------------------------------ output

def fmt(value, scale: float = 1.0, spec: str = "8.3f") -> str:
    width = int(spec.split(".")[0].lstrip("+") or 0)
    if value is None:
        return "null".rjust(width)
    return format(value * scale, spec)


def print_anatomy(result: dict) -> None:
    print(f"error anatomy; {result['sign_convention']}; tracking rows only")
    for log in result["logs"]:
        print(f"\n{log['path']}")
        print(f"  tracking samples {log['samples']}, duration "
              f"{log['duration_s']:.3f} s, dt {log['dt_s'] * 1e3:.3f} ms, "
              f"command mode {log['command_mode']}")
        agg = log["aggregate"]
        print(f"  aggregate RMS e: 8 joints {agg['all']['e_rms_rad'] * 1e3:.3f}"
              f" mrad, arm 0-6 {agg['arm']['e_rms_rad'] * 1e3:.3f} mrad;"
              f" above {result['split_hz']:g} Hz: 8 joints"
              f" {agg['all']['e_rms_above_split_rad'] * 1e3:.3f} mrad,"
              f" arm 0-6 {agg['arm']['e_rms_above_split_rad'] * 1e3:.3f}"
              f" mrad")
        print(f"   j  rms[mrad] max|e|  mean  >{result['split_hz']:g}Hz | "
              f"|dqref| peak first last [rad/s] | |ddqref| first last "
              f"[rad/s^2]")
        for joint in log["joints"]:
            print(f"   {joint['joint']} {fmt(joint['e_rms_rad'], 1e3)}"
                  f" {fmt(joint['e_max_abs_rad'], 1e3, '7.3f')}"
                  f" {fmt(joint['e_mean_rad'], 1e3, '+7.3f')}"
                  f" {fmt(joint['e_rms_above_split_rad'], 1e3, '6.3f')} |"
                  f" {fmt(joint['dqref_peak_abs_rad_s'], 1, '6.3f')}"
                  f" {fmt(joint['dqref_first_abs_rad_s'], 1, '6.3f')}"
                  f" {fmt(joint['dqref_last_abs_rad_s'], 1, '6.3f')} |"
                  f" {fmt(joint['ddqref_first_abs_rad_s2'], 1, '7.3f')}"
                  f" {fmt(joint['ddqref_last_abs_rad_s2'], 1, '7.3f')}")
        print("  lag-stiffness fit e = -tau/K - T*dqref + c")
        print("   j  K[Nm/rad]    T[ms]  c[mrad]     R2  resid[mrad]"
              "  tau rms[Nm]  note")
        for joint in log["joints"]:
            fit = joint["lag_stiffness"]
            note = []
            if fit["K_null_reason"]:
                note.append(f"K null: {fit['K_null_reason']}")
            if fit["dropped"]:
                note.append(f"dropped {','.join(fit['dropped'])}")
            print(f"   {joint['joint']} {fmt(fit['K_nm_per_rad'], 1, '10.2f')}"
                  f" {fmt(fit['T_s'], 1e3, '8.2f')}"
                  f" {fmt(fit['c_rad'], 1e3, '+8.3f')}"
                  f" {fmt(fit['r2'], 1, '6.3f')}"
                  f" {fmt(fit['residual_rms_rad'], 1e3, '12.3f')}"
                  f" {fmt(joint['tau_rms_nm'], 1, '12.3f')}  "
                  f"{'; '.join(note)}")
        print("  servo-law fit tau = Ks*(applied - q) - Ds*dq + b "
              "(rows with applied_seq > 0)")
        print("   j  Ks[Nm/rad] Ds[Nm s/rad]   b[Nm]     R2  resid[Nm]"
              "   rows  note")
        for joint in log["joints"]:
            fit = joint["servo_law"]
            note = []
            if not fit["identifiable"]:
                note.append("not identifiable")
            if fit["dropped"]:
                note.append(f"dropped {','.join(fit['dropped'])}")
            print(f"   {joint['joint']} {fmt(fit['Ks_nm_per_rad'], 1, '11.2f')}"
                  f" {fmt(fit['Ds_nm_s_per_rad'], 1, '12.3f')}"
                  f" {fmt(fit['b_nm'], 1, '+7.3f')}"
                  f" {fmt(fit['r2'], 1, '6.3f')}"
                  f" {fmt(fit['residual_rms_nm'], 1, '10.4f')}"
                  f" {fit['rows']:6d}  {'; '.join(note)}")


def print_repeat(result: dict) -> None:
    n = result["run_count"]
    print(f"repeatability; {result['sign_convention']}; tracking rows only")
    print(f"  {n} runs, common tracking samples {result['common_samples']}, "
          f"dt {result['dt_s'] * 1e3:.3f} ms, split {result['split_hz']:g} Hz,"
          f" max |qref difference| "
          f"{result['max_reference_difference_rad']:.3g} rad")
    for run in result["runs"]:
        print(f"  {run['path']} ({run['samples']} samples)")
    print("  m = mean of e over runs (repeatable); r_i = (e_i - m)*sqrt(N/(N-1))"
          " (non-repeatable); debiased = sqrt(rms(m)^2 - rms(r)^2/N)")
    split = f"{result['split_hz']:g}Hz"
    print(f"     j  rms(m)  rms(r) debiased | <={split} m      r | "
          f">{split} m      r | q0 spread   [mrad]")

    def line(label, entry, spread):
        print(f"  {label:>4} {fmt(entry['repeatable_rms_rad'], 1e3, '7.3f')}"
              f" {fmt(entry['nonrepeatable_rms_rad'], 1e3, '7.3f')}"
              f" {fmt(entry['repeatable_debiased_rms_rad'], 1e3, '8.3f')} |"
              f" {fmt(entry['repeatable_below_split_rms_rad'], 1e3, '7.3f')}"
              f" {fmt(entry['nonrepeatable_below_split_rms_rad'], 1e3, '6.3f')}"
              f" | {fmt(entry['repeatable_above_split_rms_rad'], 1e3, '6.3f')}"
              f" {fmt(entry['nonrepeatable_above_split_rms_rad'], 1e3, '6.3f')}"
              f" | {fmt(spread, 1e3, '9.3f')}")

    for joint in result["joints"]:
        line(str(joint["joint"]), joint, joint["initial_spread_rad"])
    for label in ("arm", "all"):
        entry = result["aggregate"][label]
        line(label, entry, entry["initial_spread_max_rad"])
    print("  (arm = joints 0-6, all = 0-7; the aggregate q0 spread is the "
          "largest joint's)")


def print_predict(result: dict) -> None:
    print(f"desired-trajectory feedforward, arm joints 0-6; "
          f"{result['sign_convention']}")
    print(f"  smoothing sigma {result['smooth_sigma_s'] * 1e3:.1f} ms, "
          f"Coulomb v0 {result['coulomb_v0_rad_s']:g} rad/s, ridge "
          f"{result['ridge_penalty_per_sample']:g} * samples")
    for run in result["runs"]:
        print(f"  run {run['name']}: {run['samples']} samples, dt "
              f"{run['dt_s'] * 1e3:.3f} ms, {run['duration_s']:.2f} s, "
              f"{run['path']}")

    def dropped_lines(entries, label):
        for entry in entries:
            parts = [f"j{joint['joint']} {','.join(joint['dropped'])}"
                     for joint in entry["joints"] if joint["dropped"]]
            if parts:
                print(f"    dropped ({label} {entry['run']}): "
                      f"{'; '.join(parts)}")

    for label, block in result["sets"].items():
        print(f"\nregressor set {label}: {', '.join(block['columns'])}")
        print("  leave-one-run-out: the other runs' weights predict the "
              "held-out e [mrad]")
        print("    held-out   baseline -> residual | per joint "
              "baseline/residual")
        loo = block["leave_one_out"]
        for entry in loo["held_out"]:
            per = " ".join(f"{joint['baseline_rms_rad'] * 1e3:.2f}/"
                           f"{joint['residual_rms_rad'] * 1e3:.2f}"
                           for joint in entry["joints"])
            print(f"    {entry['run']:<10} {entry['baseline_rms_rad'] * 1e3:8.3f}"
                  f" -> {entry['residual_rms_rad'] * 1e3:8.3f} | {per}")
        print(f"    mean over held-out runs of baseline/residual: "
              f"{fmt(loo['mean_baseline_over_residual'], 1, '.2f')}")
        dropped_lines(loo["held_out"], "held out")
        print("  in-sample (fitted and evaluated on the same run) [mrad]")
        for entry in block["in_sample"]:
            print(f"    {entry['run']:<10} {entry['baseline_rms_rad'] * 1e3:8.3f}"
                  f" -> {entry['residual_rms_rad'] * 1e3:8.3f}")
        dropped_lines(block["in_sample"], "in-sample")

    print("\npooled fit of the full set over all runs, e = sum(w * phi)")
    print("   j  -1/w_g[Nm/rad]  lag -w_dq[ms]  w_coulomb[mrad]  "
          "constant[mrad]  w_tau_dyn[rad/Nm]  w_ddq[s^2]     R2  "
          "resid[mrad]  dropped")
    for joint in result["pooled_full"]:
        w = joint["weights"]
        print(f"   {joint['joint']} {fmt(joint['stiffness_nm_per_rad'], 1, '15.2f')}"
              f" {fmt(joint['lag_s'], 1e3, '14.2f')}"
              f" {fmt(joint['coulomb_rad'], 1e3, '16.3f')}"
              f" {fmt(joint['constant_rad'], 1e3, '15.3f')}"
              f" {fmt(w['tau_dyn'], 1, '18.3e')}"
              f" {fmt(w['ddq'], 1, '11.3e')}"
              f" {fmt(joint['r2'], 1, '6.3f')}"
              f" {fmt(joint['residual_rms_rad'], 1e3, '12.3f')}  "
              f"{','.join(joint['dropped'])}")
    print(f"  note: {result['pooled_full_note']}")
    print("\nmeasured torque check (pooled): "
          "tau = a*g + b*tau_dyn + J*ddq + fv*dq + fc*coulomb + c")
    print("   j       a       b  J[kg m^2]  fv[Nm s/rad]  fc[Nm]   c[Nm]"
          "     R2  resid[Nm]  tau rms[Nm]  dropped")
    for joint in result["measured_torque"]:
        print(f"   {joint['joint']} {fmt(joint['a'], 1, '7.3f')}"
              f" {fmt(joint['b'], 1, '7.3f')}"
              f" {fmt(joint['J_kg_m2'], 1, '10.4f')}"
              f" {fmt(joint['fv_nm_s_per_rad'], 1, '13.3f')}"
              f" {fmt(joint['fc_nm'], 1, '7.3f')}"
              f" {fmt(joint['c_nm'], 1, '+7.3f')}"
              f" {fmt(joint['r2'], 1, '6.3f')}"
              f" {fmt(joint['residual_rms_nm'], 1, '10.4f')}"
              f" {fmt(joint['tau_rms_nm'], 1, '12.4f')}  "
              f"{','.join(joint['dropped'])}")
    print(f"  note: {result['measured_torque_note']}")


# --------------------------------------------------------------------- CLI

def _g(x: float) -> str:
    return f"{x:g}".replace("e-0", "e-")


COMMON_HELP = f"""
Sign convention: e = q - qref (measured minus reference), read from the
log's error{{j}}_rad column. Only rows with phase == tracking are analysed
(one contiguous block of at least {MIN_TRACKING_SAMPLES} rows); dt is the
median step of time_s over them.
"""

ANATOMY_HELP = f"""Per-log error anatomy; each LOG is analysed independently.
{COMMON_HELP}
Per joint j = 0..7: rms, max|e| and mean of e; rms of e above --split-hz
(zero-phase FFT split of the mean-removed e); peak |dqref|, and |dqref| and
|ddqref| at the first and last tracking sample (rest-to-rest check).
  lag-stiffness fit (least squares)   e = -tau/K - T*dqref + c
    K is null when tau's RMS is below {TAU_RMS_MIN_NM} Nm or the fitted
    compliance 1/K is numerically zero (|1/K| * rms(tau) below
    {_g(COMPLIANCE_ZERO_RAD)} rad); T and c are then fitted without it.
  servo-law fit (least squares, rows with applied_seq > 0)
                                      tau = Ks*(applied - q) - Ds*dq + b
Any other regressor whose RMS is below {_g(NEGLIGIBLE_RMS)} (its own units) is
dropped and its term reported null; a fit whose remaining columns are
linearly dependent reports every term null. Per log: aggregate RMS of e, and
of e above --split-hz, over all 8 joints and over the 7 arm joints (the RMS
over joints of the per-joint RMS), samples, duration, dt, command mode.

Units: --json is SI: e in rad, dqref rad/s, ddqref rad/s^2, K and Ks Nm/rad,
T s, c rad, Ds Nm s/rad, tau and b Nm, dt and duration s. The table shows e,
c and residuals in mrad and T in ms.
"""

REPEAT_HELP = f"""Repeatable versus non-repeatable error over N >= 2 runs of the
SAME reference.
{COMMON_HELP}
The references must match: tracking lengths within
{LENGTH_TOLERANCE_SAMPLES} sample and max |qref difference| <=
{_g(REFERENCE_TOLERANCE_RAD)} rad over the common length (else exit 1, naming
the logs). Per joint, over the common length:
  m(t) = mean of e over runs                 repeatable, rms(m) raw
  r_i  = (e_i - m) * sqrt(N/(N-1))           non-repeatable, rms over all runs
  debiased repeatable = sqrt(max(0, rms(m)^2 - rms(r)^2/N))
  the raw rms(m), rms(r) pair below (including the mean) and above
  --split-hz (zero-phase FFT split), and the initial-state spread: max - min
  over runs of q at the first tracking sample.
Aggregates over all 8 joints and over the 7 arm joints (RMS over joints;
the aggregate spread is the largest joint's).

Units: --json is SI (rad, s, Hz); the table shows mrad.
"""

PREDICT_HELP = f"""Leave-one-run-out test of a parametric feedforward built ONLY
from the desired trajectory, arm joints 0..6; runs are intended to have
different references.
{COMMON_HELP}
Regressors per run: each qref{{j}} (all 8 joints) is smoothed by a zero-phase
Gaussian (--smooth-sigma-s, edge-padded), then dq and ddq are its central
differences (numpy.gradient). BIN is called as
  BIN --model ZTK --input IN.csv --output OUT.csv
for gravity and inverse-dynamics torques along the smoothed reference.
Per joint columns: g = gravity, tau_dyn = inverse_dynamics - gravity, ddq,
dq, coulomb = tanh(dq / --coulomb-v0), constant. Sets: full = all six,
gravity_lag = g, dq, constant; lag_only = dq, constant. Fits are ridge least
squares on RMS-normalised columns, penalty {_g(RIDGE_PER_SAMPLE)} * samples; a
column whose RMS over the fitting rows is below {_g(NEGLIGIBLE_RMS)} (its own
units) is dropped and listed, never fitted.
Reports per set: leave-one-run-out baseline (rms e) and residual RMS per
held-out run (7-joint aggregate and per joint), the mean over held-out runs
of baseline/residual, and in-sample baseline and residual per run. Then the
pooled full-set weights in physical terms (stiffness -1/w_g, lag -w_dq,
Coulomb w_coulomb, constant) and the measured-torque check
  tau = a*g + b*tau_dyn + J*ddq + fv*dq + fc*coulomb + c (pooled, R2).
Individual weights in those two tables are not identified when regressors
are collinear or a model term is small; the held-out residual, not the
weights, is the result to read.

Units: --json is SI: e and residuals in rad, stiffness Nm/rad, lag s, Coulomb
and constant rad, raw weights per weight_units (g, tau_dyn rad/Nm; ddq s^2;
dq s; coulomb, constant rad), J kg m^2, fv Nm s/rad, fc and c Nm. The tables
show mrad and ms where their headers say so.
"""


def parse_run(text: str) -> tuple[str, str]:
    name, sep, path = text.partition("=")
    if not sep or not name or not path:
        raise InputError(f"--run expects NAME=LOG, got '{text}'")
    return name, path


def build_parser() -> argparse.ArgumentParser:
    raw = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        prog="follow_learnability.py", description=__doc__,
        formatter_class=raw)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--json", metavar="FILE",
                        help="also write the results to FILE as JSON "
                             "(SI units, null when unidentifiable)")
    sub = parser.add_subparsers(dest="command", required=True)
    anatomy_parser = sub.add_parser(
        "anatomy", parents=[common], description=ANATOMY_HELP,
        formatter_class=raw,
        help="per-log error anatomy (e = q - qref; SI in JSON)")
    anatomy_parser.add_argument("logs", nargs="+", metavar="LOG")
    anatomy_parser.add_argument("--split-hz", type=float, default=3.0,
                                help="band split frequency [Hz] "
                                     "(default 3.0)")
    repeat_parser = sub.add_parser(
        "repeat", parents=[common], description=REPEAT_HELP,
        formatter_class=raw,
        help="repeatable vs non-repeatable error over runs of one "
             "reference (e = q - qref; SI in JSON)")
    repeat_parser.add_argument("logs", nargs="+", metavar="LOG",
                               help="at least two logs")
    repeat_parser.add_argument("--split-hz", type=float, default=3.0,
                               help="band split frequency [Hz] "
                                    "(default 3.0)")
    predict_parser = sub.add_parser(
        "predict", parents=[common], description=PREDICT_HELP,
        formatter_class=raw,
        help="leave-one-run-out test of a desired-trajectory feedforward "
             "(e = q - qref; SI in JSON)")
    predict_parser.add_argument("--model-torque", required=True,
                                metavar="BIN",
                                help="the x7_model_torque binary")
    predict_parser.add_argument("--model", required=True, metavar="ZTK",
                                help="the robot model passed to BIN")
    predict_parser.add_argument("--run", action="append", required=True,
                                metavar="NAME=LOG", dest="runs",
                                help="a named follow log (give at least "
                                     "two)")
    predict_parser.add_argument("--smooth-sigma-s", type=float, default=0.04,
                                help="Gaussian smoothing sigma of qref [s] "
                                     "(default 0.04; 0 disables)")
    predict_parser.add_argument("--coulomb-v0", type=float, default=0.05,
                                help="Coulomb regressor velocity scale "
                                     "[rad/s] (default 0.05)")
    return parser


def run_command(args: argparse.Namespace) -> tuple[dict, Callable]:
    head = {"format": FORMAT, "version": FORMAT_VERSION,
            "command": args.command, "sign_convention": SIGN_CONVENTION}
    if args.command in ("anatomy", "repeat"):
        if not (math.isfinite(args.split_hz) and args.split_hz > 0.0):
            raise InputError("--split-hz must be finite and positive")
        head["split_hz"] = args.split_hz
    if args.command == "anatomy":
        logs = [anatomy(load_tracking(path, ANATOMY_COLUMNS), args.split_hz)
                for path in args.logs]
        head.update({"tau_rms_threshold_nm": TAU_RMS_MIN_NM,
                     "compliance_zero_rad": COMPLIANCE_ZERO_RAD,
                     "negligible_column_rms": NEGLIGIBLE_RMS, "logs": logs})
        return head, print_anatomy
    if args.command == "repeat":
        if len(args.logs) < 2:
            raise InputError(f"repeat needs at least 2 logs, got "
                             f"{len(args.logs)}")
        runs = [load_tracking(path, REPEAT_COLUMNS) for path in args.logs]
        head.update(repeat(runs, args.split_hz))
        return head, print_repeat
    pairs = [parse_run(text) for text in args.runs]
    if len(pairs) < 2:
        raise InputError(f"predict needs at least 2 runs, got {len(pairs)}")
    if not os.path.isfile(args.model):
        raise InputError(f"{args.model}: no such model file")
    runs = [(name, load_tracking(path, PREDICT_COLUMNS))
            for name, path in pairs]
    provider = binary_torque_provider(args.model_torque, args.model)
    head.update(predict(runs, provider, args.smooth_sigma_s,
                        args.coulomb_v0))
    return head, print_predict


def main(argv: list | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        result, printer = run_command(args)
        if args.json:
            text = json.dumps(result, indent=2, allow_nan=False)
            try:
                with open(args.json, "w") as stream:
                    stream.write(text + "\n")
            except OSError as error:
                raise InputError(f"{args.json}: cannot write: "
                                 f"{error.strerror}") from error
    except InputError as error:
        print(f"follow_learnability: error: {error}", file=sys.stderr)
        return 1
    printer(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
