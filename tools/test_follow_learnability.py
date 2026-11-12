#!/usr/bin/env python3
"""Regressions for follow_learnability.py, run under ctest.

Synthetic x7_follow logs (the FollowCsvLog header, transcribed) carry
planted parameters. anatomy recovers a planted lag-stiffness law and servo
law, reports null for what the data cannot identify, and ignores every row
outside the tracking phase; repeat separates a planted repeatable trace
from a planted noise level and refuses runs of different references;
predict, with an injected torque provider, recovers planted feedforward
weights, drops a zero column, and predicts a held-out run to its noise
level. Parameter tolerances are six standard errors of the planted white
noise, computed from the planted design itself (plus, for the ridge fits,
a bound on the ridge bias); exact reductions are compared at the
precision of the logger's %.9g fields.
"""

import contextlib
import importlib.util
import io
import json
import math
import os
import pathlib
import sys
import tempfile

# The fits here are small: BLAS threads would only spin (about 8x the CPU
# time for no gain in wall time), so the regressions run single-threaded.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import numpy as np  # noqa: E402

spec = importlib.util.spec_from_file_location("fl",
                                              "tools/follow_learnability.py")
assert spec is not None and spec.loader is not None
fl = importlib.util.module_from_spec(spec)
sys.modules["fl"] = fl
spec.loader.exec_module(fl)

DOF = 8
ARM = 7
DT = 0.01
SCRATCH = tempfile.TemporaryDirectory(prefix="follow_learnability_test-")
ROOT = pathlib.Path(SCRATCH.name)
rng = np.random.default_rng(20261112)

# FollowCsvLog::writeHeader (apps/follow/follow_run.hpp), transcribed.
HEADER = ["schema_version", "time_s", "phase", "command_mode",
          "phase_time_s", "feedback_time_s", "feedback_seq",
          "receipt_accepted", "submitted_seq", "applied_seq"]
for _j in range(DOF):
    HEADER += [f"qref{_j}_rad", f"dqref{_j}_rad_s", f"ddqref{_j}_rad_s2",
               f"q{_j}_rad", f"dq{_j}_rad_s", f"tau{_j}_nm",
               f"qcmd{_j}_rad", f"dqcmd{_j}_rad_s", f"effort_limit{_j}_nm",
               f"error{_j}_rad", f"applied{_j}",
               f"applied_effort_limit{_j}_nm", f"flags{_j}"]
QUANTITIES = ("qref", "dqref", "ddqref", "q", "dq", "tau", "applied")


def g9(value):
    """The logger's %.9g."""
    return f"{value:.9g}"


def logged(x):
    """x as the tool reads it back from %.9g fields."""
    return np.vectorize(lambda v: float(g9(v)))(np.asarray(x, dtype=float))


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def multisine(t, terms):
    """(q, dq, ddq) of a sum of amp * sin(2 pi f t + phase)."""
    q, dq, ddq = np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)
    for amp, freq, phase in terms:
        w = 2.0 * math.pi * freq
        q += amp * np.sin(w * t + phase)
        dq += amp * w * np.cos(w * t + phase)
        ddq -= amp * w * w * np.sin(w * t + phase)
    return q, dq, ddq


def blank_track(n):
    track = {name: np.zeros((n, DOF)) for name in QUANTITIES}
    track["t"] = np.arange(n) * DT
    track["applied_seq"] = np.arange(1, n + 1)
    return track


def head(track, n):
    return {name: value[:n].copy() for name, value in track.items()}


def row_fields(t, t0, phase, mode, seq, applied_seq, values, nan_dq=False):
    fields = ["1", f"{t:.9f}", phase, str(mode), f"{t - t0:.9f}",
              f"{t:.9f}", str(seq), "1", str(seq), str(applied_seq)]
    for j in range(DOF):
        qref, dqref, ddqref, q, dq, tau, applied = (values[name][j]
                                                    for name in QUANTITIES)
        fields += [g9(qref), g9(dqref), g9(ddqref), g9(q),
                   "nan" if nan_dq else g9(dq), g9(tau), g9(qref), g9(dqref),
                   g9(0.0), g9(q - qref),
                   g9(applied if applied_seq > 0 else 0.0), g9(0.0), "0"]
    return fields


def write_log(name, track, mode=5, pre=0, post=0, drop=(), phase_at=None,
              truncate=False, pre_phase="home"):
    """A follow log whose tracking rows carry `track`. The pre (pre_phase)
    and post (finalizing) rows are far from them: q off by 0.5 rad, tau by
    50 Nm, another command mode, dq not even numeric."""
    n = len(track["t"])
    t0 = track["t"][0]

    def at(k):
        return {name: track[name][k] for name in QUANTITIES}

    rows, seq = [], 0
    for i in range(pre):
        values = at(0)
        values["q"] = values["q"] + 0.5
        values["tau"] = values["tau"] + 50.0
        seq += 1
        rows.append(row_fields(t0 - (pre - i) * DT, t0 - pre * DT, pre_phase,
                               3, seq, 0, values, nan_dq=True))
    for k in range(n):
        seq += 1
        rows.append(row_fields(track["t"][k], t0,
                               (phase_at or {}).get(k, "tracking"), mode, seq,
                               int(track["applied_seq"][k]), at(k)))
    for i in range(post):
        values = at(n - 1)
        values["q"] = values["q"] - 0.5
        values["tau"] = values["tau"] - 50.0
        seq += 1
        rows.append(row_fields(track["t"][-1] + (i + 1) * DT, track["t"][-1],
                               "finalizing", 3, seq, seq, values,
                               nan_dq=True))
    keep = [i for i, column in enumerate(HEADER) if column not in drop]
    path = ROOT / name
    with open(path, "w") as stream:
        stream.write(",".join(HEADER[i] for i in keep) + "\n")
        for fields in rows:
            stream.write(",".join(fields[i] for i in keep) + "\n")
        if truncate:
            stream.write(",".join(rows[-1][:20]) + "\n")
    return str(path)


def run_main(*argv):
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        try:
            status = fl.main([str(arg) for arg in argv])
        except SystemExit as stop:
            status = stop.code
    return status, out.getvalue(), err.getvalue()


def raises(fn, *args, contains=()):
    try:
        fn(*args)
    except fl.InputError as error:
        for text in contains:
            assert text in str(error), (text, str(error))
        return str(error)
    raise AssertionError(f"{fn.__name__} accepted invalid input")


def ols_standard_errors(x, sigma):
    """Standard errors of least-squares weights under white noise sigma."""
    return sigma * np.sqrt(np.diag(np.linalg.inv(x.T @ x)))


def ridge_bias_bound(x, w):
    """Per-weight bound on ridge minus least squares for noise-free data:
    with A = x / s (RMS-normalised) and G = A'A, the bias in normalised
    weights is -lambda (G + lambda I)^-1 (w s), at most lambda / eig_min(G)
    times |w s|. (The ridge noise covariance is below the least-squares
    one, so its standard errors bound the ridge's.)"""
    s = np.sqrt(np.mean(np.square(x), axis=0))
    gram = (x / s).T @ (x / s)
    penalty = fl.RIDGE_PER_SAMPLE * len(x)
    return penalty / np.linalg.eigvalsh(gram)[0] * np.linalg.norm(w * s) / s


# ---------------------------------------------------------------- numerics

# The band split is exact on whole periods: the 3 Hz component (equal to
# the split) and the mean stay below, the 10 Hz one goes above, also with
# the median dt of %.9f stamps (0.01 less 2e-16), which puts the 3 Hz bin
# an ulp above the split.
t = np.arange(1000) * DT
slow = 0.2 + np.sin(2 * np.pi * 1.0 * t) + 0.3 * np.sin(2 * np.pi * 3.0 * t)
fast = 0.5 * np.sin(2 * np.pi * 10.0 * t)
for dt in (DT, 0.009999999999999787):
    below, above = fl.band_split(slow + fast, dt, 3.0)
    assert np.max(np.abs(below - slow)) < 1e-12
    assert np.max(np.abs(above - fast)) < 1e-12
    assert abs(rms(above) - 0.5 / math.sqrt(2)) < 1e-12

# Gaussian smoothing: length kept, constants and ramps kept (the kernel is
# symmetric and normalised; edge padding keeps a constant at the ends), and
# a sinusoid keeps its phase and is attenuated by the Gaussian's transfer
# exp(-2 pi^2 f^2 sigma^2), up to the 4-sigma truncation (~3e-4).
SIGMA_S = 0.04
HALF = 16  # 4 sigma / dt
x = fl.gaussian_smooth(np.full(300, 0.7), SIGMA_S, DT)
assert x.shape == (300,) and np.max(np.abs(x - 0.7)) < 1e-15
ramp = 0.3 * np.arange(300) * DT
x = fl.gaussian_smooth(ramp, SIGMA_S, DT)
assert np.max(np.abs(x - ramp)[HALF:-HALF]) < 1e-12
q, dq, ddq = fl.reference_derivatives(np.column_stack([ramp, ramp]), DT,
                                      SIGMA_S)
assert np.max(np.abs(dq[HALF + 1:-HALF - 1] - 0.3)) < 1e-9
assert np.max(np.abs(ddq[HALF + 2:-HALF - 2])) < 1e-6
wave = np.sin(2 * np.pi * 2.0 * np.arange(600) * DT)
gain = math.exp(-2 * math.pi ** 2 * 2.0 ** 2 * SIGMA_S ** 2)
x = fl.gaussian_smooth(wave, SIGMA_S, DT)
assert np.max(np.abs(x - gain * wave)[HALF:-HALF]) < 1e-3
assert np.array_equal(fl.gaussian_smooth(wave, 0.0, DT), wave)

# The lag-stiffness fit: a compliance that is numerically zero is no
# stiffness (K null, tau dropped) and the other terms are fitted without it.
t = np.arange(500) * DT
dq = np.cos(2 * np.pi * 0.4 * t)
tau = 0.8 * np.sin(2 * np.pi * 1.1 * t)
fit = fl.lag_stiffness_fit(-0.03 * dq + 1e-3, tau, dq)
assert fit["K_nm_per_rad"] is None, fit
assert fit["K_null_reason"] == "compliance_numerically_zero"
assert fit["dropped"] == ["tau"]
assert abs(fit["T_s"] - 0.03) < 1e-12 and abs(fit["c_rad"] - 1e-3) < 1e-12
# The servo-law fit of an idle joint: dq is zero (dropped) and a constant
# applied - q cannot be told from b, so nothing is identified.
fit = fl.servo_law_fit(0.4 + 0.01 * np.sin(t), np.full(500, 0.31),
                       np.full(500, 0.3), np.zeros(500))
assert not fit["identifiable"] and fit["dropped"] == ["dq"]
assert fit["Ks_nm_per_rad"] is None and fit["b_nm"] is None
assert fit["r2"] is None

# ----------------------------------------------------------------- anatomy

N_ANATOMY = 2000
K = np.array([40, 60, 80, 100, 120, 150, 30, 20.0])          # Nm/rad
T = np.array([20, 30, 25, 40, 15, 35, 50, 10]) * 1e-3          # s
C = np.array([1.0, -2.0, 0.5, 0.0, -1.0, 3.0, 2.0, -0.5]) * 1e-3  # rad
KS = np.array([50, 70, 90, 110, 130, 160, 40, 25.0])          # Nm/rad
DS = np.array([0.3, 0.5, 0.4, 0.2, 0.6, 0.1, 0.3, 0.2])        # Nm s/rad
B = np.array([0.1, -0.2, 0.05, 0.0, 0.3, -0.1, 0.02, -0.05])   # Nm
SIGMA_E = 2e-5     # rad, white noise on e
SIGMA_TAU = 0.01   # Nm, white noise on the servo law
NO_TORQUE = 6      # tau == 0: K cannot be identified
AT_REST = 7        # the gripper's reference is constant: T cannot be
INVALID = 5        # the first tracking rows have no applied goal yet

track = blank_track(N_ANATOMY)
t = track["t"]
for j in range(DOF):
    if j == AT_REST:
        track["qref"][:, j] = 0.4
    else:
        track["qref"][:, j], track["dqref"][:, j], track["ddqref"][:, j] = (
            multisine(t, [(0.3, 0.2 + 0.03 * j, j),
                          (0.05, 0.9 + 0.1 * j, 2.0 * j)]))
    u = (0.02 * np.sin(2 * np.pi * (0.37 + 0.02 * j) * t + j)
         + 0.008 * np.cos(2 * np.pi * 1.3 * t + 0.5 * j))  # applied - q
    dq = track["dqref"][:, j] + 0.3 * np.sin(2 * np.pi * 2.1 * t + j)
    tau = (np.zeros(N_ANATOMY) if j == NO_TORQUE else
           KS[j] * u - DS[j] * dq + B[j]
           + SIGMA_TAU * rng.standard_normal(N_ANATOMY))
    e = (-tau / K[j] - T[j] * track["dqref"][:, j] + C[j]
         + SIGMA_E * rng.standard_normal(N_ANATOMY))
    track["q"][:, j] = track["qref"][:, j] + e
    track["dq"][:, j] = dq
    track["tau"][:, j] = tau
    track["applied"][:, j] = track["q"][:, j] + u
track["applied_seq"][:INVALID] = 0

plain = write_log("anatomy.csv", track)
framed = write_log("anatomy_framed.csv", track, pre=20, post=15)
result = fl.anatomy(fl.load_tracking(plain, fl.ANATOMY_COLUMNS), 3.0)
# Rows outside the tracking phase change nothing, not even when they hold
# other values, another command mode, or no numbers at all.
other = fl.anatomy(fl.load_tracking(framed, fl.ANATOMY_COLUMNS), 3.0)
assert other.pop("path") == framed and result["path"] == plain
assert json.dumps(other) == json.dumps({k: v for k, v in result.items()
                                        if k != "path"})
# A learned command offset adds an offset_ramp phase between homing and
# tracking: a known phase, and its rows change nothing either.
ramped = write_log("anatomy_ramped.csv", track, pre=20, post=15,
                   pre_phase="offset_ramp")
other = fl.anatomy(fl.load_tracking(ramped, fl.ANATOMY_COLUMNS), 3.0)
assert other.pop("path") == ramped
assert json.dumps(other) == json.dumps({k: v for k, v in result.items()
                                        if k != "path"})
assert result["samples"] == N_ANATOMY and result["command_mode"] == 5
assert abs(result["dt_s"] - DT) < 1e-12
assert abs(result["duration_s"] - (N_ANATOMY - 1) * DT) < 1e-9

error = logged(track["q"] - track["qref"])  # the error{j}_rad column
assert abs(result["aggregate"]["all"]["e_rms_rad"] - rms(error)) < 1e-15
assert abs(result["aggregate"]["arm"]["e_rms_rad"]
           - rms(error[:, :ARM])) < 1e-15
dqref, ddqref = logged(track["dqref"]), logged(track["ddqref"])
tau = logged(track["tau"])
u = logged(track["applied"]) - logged(track["q"])
ones = np.ones(N_ANATOMY)
valid = track["applied_seq"] > 0
for j, joint in enumerate(result["joints"]):
    e = error[:, j]
    assert joint["joint"] == j
    assert abs(joint["e_rms_rad"] - rms(e)) < 1e-15
    assert joint["e_max_abs_rad"] == np.max(np.abs(e))
    assert abs(joint["e_mean_rad"] - np.mean(e)) < 1e-15
    assert joint["dqref_peak_abs_rad_s"] == np.max(np.abs(dqref[:, j]))
    assert joint["dqref_first_abs_rad_s"] == abs(dqref[0, j])
    assert joint["dqref_last_abs_rad_s"] == abs(dqref[-1, j])
    assert joint["ddqref_first_abs_rad_s2"] == abs(ddqref[0, j])
    assert joint["ddqref_last_abs_rad_s2"] == abs(ddqref[-1, j])
    assert abs(joint["e_rms_above_split_rad"]
               - rms(fl.band_split(e, DT, 3.0)[1])) < 1e-15

    # e = -tau/K - T*dqref + c: compliance 1/K, T and c each within six
    # standard errors of SIGMA_E over this design.
    fit = joint["lag_stiffness"]
    columns, planted, fitted = [], [], []
    if j == NO_TORQUE:
        assert fit["K_nm_per_rad"] is None, fit
        assert fit["K_null_reason"] == "tau_rms_below_threshold"
        assert fit["dropped"] == ["tau"]
    else:
        assert fit["K_null_reason"] is None and fit["K_nm_per_rad"] > 0
        columns.append(-tau[:, j])
        planted.append(1.0 / K[j])
        fitted.append(1.0 / fit["K_nm_per_rad"])
    if j == AT_REST:
        assert fit["T_s"] is None and fit["dropped"] == ["dqref"], fit
    else:
        columns.append(-dqref[:, j])
        planted.append(T[j])
        fitted.append(fit["T_s"])
    columns.append(ones)
    planted.append(C[j])
    fitted.append(fit["c_rad"])
    se = ols_standard_errors(np.column_stack(columns), SIGMA_E)
    assert np.all(np.abs(np.array(fitted) - planted) <= 6 * se), (
        j, fitted, planted, se)
    # The residual is the planted noise: rms within six standard errors.
    assert abs(fit["residual_rms_rad"] / SIGMA_E - 1) <= 6 / math.sqrt(
        2 * N_ANATOMY), fit
    assert fit["rows"] == N_ANATOMY and fit["identifiable"]

    # tau = Ks*(applied - q) - Ds*dq + b, only on rows with a valid goal
    # (the logger writes applied 0 before one exists).
    fit = joint["servo_law"]
    assert fit["rows"] == N_ANATOMY - INVALID and fit["dropped"] == []
    if j == NO_TORQUE:
        assert abs(fit["Ks_nm_per_rad"]) < 1e-9 and fit["r2"] is None
        continue
    x = np.column_stack([u[valid, j], -logged(track["dq"])[valid, j],
                         ones[valid]])
    se = ols_standard_errors(x, SIGMA_TAU)
    fitted = [fit["Ks_nm_per_rad"], fit["Ds_nm_s_per_rad"], fit["b_nm"]]
    assert np.all(np.abs(np.array(fitted) - [KS[j], DS[j], B[j]])
                  <= 6 * se), (j, fitted, se)

# The CLI: the table prints (nulls included) and --json holds the same.
out_json = ROOT / "anatomy.json"
status, out, err = run_main("anatomy", plain, framed, "--json", out_json)
assert status == 0 and err == "", err
assert "e = q - qref" in out and "null" in out
loaded = json.loads(out_json.read_text())
assert list(loaded) == ["format", "version", "command", "sign_convention",
                        "split_hz", "tau_rms_threshold_nm",
                        "compliance_zero_rad", "negligible_column_rms",
                        "logs"]
assert loaded["format"] == "rtctrl-follow-learnability"
assert loaded["command"] == "anatomy" and loaded["split_hz"] == 3.0
assert "e = q - qref" in loaded["sign_convention"]
assert len(loaded["logs"]) == 2 and loaded["logs"][0] == result
assert set(loaded["logs"][0]) == {"path", "samples", "duration_s", "dt_s",
                                  "command_mode", "aggregate", "joints"}
assert set(loaded["logs"][0]["joints"][0]) == {
    "joint", "e_rms_rad", "e_max_abs_rad", "e_mean_rad",
    "e_rms_above_split_rad", "dqref_peak_abs_rad_s", "dqref_first_abs_rad_s",
    "dqref_last_abs_rad_s", "ddqref_first_abs_rad_s2",
    "ddqref_last_abs_rad_s2", "tau_rms_nm", "lag_stiffness", "servo_law"}
assert set(loaded["logs"][0]["joints"][0]["lag_stiffness"]) == {
    "K_nm_per_rad", "T_s", "c_rad", "r2", "residual_rms_rad", "rows",
    "dropped", "identifiable", "K_null_reason"}
assert set(loaded["logs"][0]["joints"][0]["servo_law"]) == {
    "Ks_nm_per_rad", "Ds_nm_s_per_rad", "b_nm", "r2", "residual_rms_nm",
    "rows", "dropped", "identifiable"}
assert loaded["logs"][0]["joints"][NO_TORQUE]["lag_stiffness"][
    "K_nm_per_rad"] is None

# Invalid input exits nonzero and says why.
for path, text in (
        (write_log("no_tau3.csv", track, drop=("tau3_nm",)), "tau3_nm"),
        (write_log("split.csv", track, phase_at={900: "home_correction"}),
         "contiguous"),
        (write_log("ramp_typo.csv", track, pre=3, pre_phase="offset_rampx"),
         "unknown phase 'offset_rampx'"),
        (write_log("truncated.csv", track, truncate=True), "fields"),
        (write_log("short.csv", head(track, 5)), "5 tracking samples"),
        (ROOT / "absent.csv", "cannot read")):
    status, out, err = run_main("anatomy", path)
    assert status == 1 and text in err and out == "", (path, status, err)
status, _, err = run_main("anatomy", plain, "--split-hz", "0")
assert status == 1 and "--split-hz" in err

# ------------------------------------------------------------------ repeat

N_REPEAT = 3000
RUNS = 4
SIGMA_R = 8e-4  # rad, run-to-run white noise
base = blank_track(N_REPEAT + 1)  # one run keeps one more sample
t = base["t"]
signal = np.zeros((N_REPEAT + 1, DOF))
for j in range(DOF):
    base["qref"][:, j], base["dqref"][:, j], base["ddqref"][:, j] = (
        multisine(t, [(0.3, 0.2 + 0.03 * j, j)]))
    # whole periods over the common 30 s: 0.5 Hz below the split, 7 Hz above
    signal[:, j] = (1e-3 * (1 + 0.1 * j) * np.sin(2 * np.pi * 0.5 * t + j)
                    + 4e-4 * np.sin(2 * np.pi * 7.0 * t + 0.3 * j))
paths, errors, first_q = [], [], []
for i in range(RUNS):
    run = {name: value.copy() for name, value in base.items()}
    run["q"] = run["qref"] + signal + SIGMA_R * rng.standard_normal(
        signal.shape)
    run["dq"] = run["dqref"].copy()
    run["applied"] = run["q"].copy()
    run = head(run, N_REPEAT + 1 if i == 2 else N_REPEAT)
    paths.append(write_log(f"repeat{i}.csv", run))
    errors.append(logged(run["q"] - run["qref"])[:N_REPEAT])
    first_q.append(logged(run["q"][0]))
runs = [fl.load_tracking(path, fl.REPEAT_COLUMNS) for path in paths]
result = fl.repeat(runs, 3.0)
assert result["run_count"] == RUNS and result["common_samples"] == N_REPEAT
assert result["max_reference_difference_rad"] == 0.0
errors = np.array(errors)
first_q = np.array(first_q)
scale = math.sqrt(RUNS / (RUNS - 1))
for j, joint in enumerate(result["joints"]):
    # The reductions themselves, recomputed from the logged columns.
    m = errors[:, :, j].mean(axis=0)
    r = (errors[:, :, j] - m) * scale
    for key, value in (("repeatable_rms_rad", rms(m)),
                       ("nonrepeatable_rms_rad", rms(r)),
                       ("repeatable_below_split_rms_rad",
                        rms(fl.band_split(m, DT, 3.0)[0])),
                       ("repeatable_above_split_rms_rad",
                        rms(fl.band_split(m, DT, 3.0)[1])),
                       ("nonrepeatable_below_split_rms_rad",
                        rms([fl.band_split(x, DT, 3.0)[0] for x in r])),
                       ("nonrepeatable_above_split_rms_rad",
                        rms([fl.band_split(x, DT, 3.0)[1] for x in r]))):
        assert abs(joint[key] - value) <= 1e-12 * value, (j, key)
    for kind in ("repeatable", "nonrepeatable"):  # the bands partition
        total = joint[f"{kind}_rms_rad"] ** 2
        parts = (joint[f"{kind}_below_split_rms_rad"] ** 2
                 + joint[f"{kind}_above_split_rms_rad"] ** 2)
        assert abs(parts - total) <= 1e-12 * total
    assert joint["initial_spread_rad"] == np.max(first_q[:, j]) - np.min(
        first_q[:, j])

    # The planted levels, within six standard errors: rms(r) has
    # (N-1)*n degrees of freedom; the debiased value's error is the
    # signal-noise cross term plus the two noise-power fluctuations.
    s = rms(signal[:N_REPEAT, j])
    noise_m = SIGMA_R ** 2 / RUNS
    se_r = SIGMA_R / math.sqrt(2 * (RUNS - 1) * N_REPEAT)
    assert abs(joint["nonrepeatable_rms_rad"] - SIGMA_R) <= 6 * se_r
    sd_d2 = math.sqrt(4 * s * s * noise_m / N_REPEAT
                      + 2 * noise_m ** 2 / N_REPEAT
                      + 2 * noise_m ** 2 / ((RUNS - 1) * N_REPEAT))
    se_d = sd_d2 / (2 * s)
    assert abs(joint["repeatable_debiased_rms_rad"] - s) <= 6 * se_d, j
    raw = math.sqrt(s * s + noise_m)
    assert abs(joint["repeatable_rms_rad"] - raw) <= 6 * sd_d2 / (2 * raw)
    # ...and the raw value is visibly biased by the averaged noise.
    assert joint["repeatable_rms_rad"] - s > 6 * se_d
for label, joints in (("all", result["joints"]),
                      ("arm", result["joints"][:ARM])):
    agg = result["aggregate"][label]
    for key in fl.REPEAT_RMS_KEYS:
        expected = math.sqrt(np.mean([joint[key] ** 2 for joint in joints]))
        assert abs(agg[key] - expected) <= 1e-12 * expected, (label, key)
    assert agg["repeatable_debiased_rms_rad"] == fl.debiased(
        agg["repeatable_rms_rad"], agg["nonrepeatable_rms_rad"], RUNS)
    assert agg["initial_spread_max_rad"] == max(
        joint["initial_spread_rad"] for joint in joints)

out_json = ROOT / "repeat.json"
status, out, err = run_main("repeat", *paths, "--json", out_json)
assert status == 0 and err == "" and "e = q - qref" in out, err
loaded = json.loads(out_json.read_text())
assert list(loaded) == ["format", "version", "command", "sign_convention",
                        "split_hz", "runs", "run_count", "common_samples",
                        "dt_s", "max_reference_difference_rad", "joints",
                        "aggregate"]
assert loaded["command"] == "repeat"
assert loaded["joints"] == result["joints"]
assert set(loaded["joints"][0]) == {"joint", *fl.REPEAT_RMS_KEYS,
                                    "repeatable_debiased_rms_rad",
                                    "initial_spread_rad"}
assert set(loaded["aggregate"]) == {"all", "arm"}
assert set(loaded["aggregate"]["arm"]) == {*fl.REPEAT_RMS_KEYS,
                                           "repeatable_debiased_rms_rad",
                                           "initial_spread_max_rad"}

# Runs of different references are refused, naming the log that differs.
moved = {name: value[:N_REPEAT].copy() for name, value in base.items()}
moved["qref"][:, 3] += 1e-6
moved["q"] = moved["qref"].copy()
moved["applied"] = moved["q"].copy()
moved_path = write_log("repeat_moved.csv", moved)
message = raises(fl.repeat,
                 runs[:2] + [fl.load_tracking(moved_path, fl.REPEAT_COLUMNS)],
                 3.0, contains=(moved_path, "differ"))
assert paths[1] not in message, message
status, _, err = run_main("repeat", paths[0], moved_path)
assert status == 1 and moved_path in err
long_path = write_log("repeat_long.csv", {name: value.copy() for name, value
                                          in blank_track(N_REPEAT + 2).items()})
raises(fl.repeat, [runs[0], fl.load_tracking(long_path, fl.REPEAT_COLUMNS)],
       3.0, contains=("tracking lengths differ", long_path))
raises(fl.repeat, runs[:1], 3.0, contains=("at least 2",))
status, out, err = run_main("repeat", paths[0])
assert status == 1 and "at least 2 logs" in err and out == ""

# ----------------------------------------------------------------- predict

TORQUE_MODULE = '''import numpy as np

GRAVITY_NM = np.array([0.0, 3.0, 1.5, 2.0, 0.5, 0.8, 0.2, 0.0])
INERTIA = np.array([0.05, 0.08, 0.03, 0.04, 0.01, 0.02, 0.005, 0.001])


def torques(q, dq, ddq):
    """Synthetic (gravity, inverse_dynamics); joint 0 has no gravity."""
    gravity = GRAVITY_NM * np.sin(q + 0.3)
    coupled = (0.5 * dq * np.roll(dq, -1, axis=1)
               + 0.3 * np.cos(q) * np.roll(ddq, -1, axis=1))
    return gravity, gravity + INERTIA * ddq + coupled
'''
FAKE_BINARY = '''#!{python}
"""Stands in for x7_model_torque: same arguments, same files."""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from synthetic_torque import torques  # noqa: E402

args = sys.argv[1:]
model, source, target = (args[args.index(flag) + 1]
                         for flag in ("--model", "--input", "--output"))
mode = os.environ.get("FAKE_TORQUE_MODE", "ok")
if mode == "fail":
    sys.exit("fake model torque: failed on purpose")
if os.path.exists(target) or not os.path.isfile(model):
    sys.exit(4)
with open(source, newline="") as stream:
    rows = list(csv.DictReader(stream))


def joints(pattern):
    return np.array([[float(row[pattern.format(j)]) for j in range(8)]
                     for row in rows])


gravity, dynamics = torques(joints("qref{{}}_rad"), joints("dqref{{}}_rad_s"),
                            joints("ddqref{{}}_rad_s2"))
count = len(rows) - (1 if mode == "short" else 0)
header = (["schema_version", "row"] + [f"gravity{{j}}_nm" for j in range(8)]
          + [f"inverse_dynamics{{j}}_nm" for j in range(8)])
extra = []
if mode == "renamed":    # a unit suffix lost
    header[-1] = "inverse_dynamics7"
elif mode == "reordered":  # the two torque blocks swapped
    header = header[:2] + header[10:] + header[2:10]
elif mode == "extra":    # one column more than the interface has
    header.append("note")
    extra = ["0"]
with open(target, "w") as stream:
    stream.write(",".join(header) + "\\n")
    for k in range(count):
        version = "2" if mode == "schema" and k == count - 1 else "1"
        stream.write(",".join([version, str(k)]
                              + [repr(float(v)) for v in gravity[k]]
                              + [repr(float(v)) for v in dynamics[k]]
                              + extra)
                     + "\\n")
'''
(ROOT / "synthetic_torque.py").write_text(TORQUE_MODULE)
spec = importlib.util.spec_from_file_location("synthetic_torque",
                                              ROOT / "synthetic_torque.py")
assert spec is not None and spec.loader is not None
synthetic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(synthetic)
fake = ROOT / "fake_model_torque"
fake.write_text(FAKE_BINARY.format(python=sys.executable))
fake.chmod(0o755)
model = ROOT / "model.ztk"
model.write_text("synthetic\n")

N_PREDICT = 1500
PREDICT_RUNS = 4
V0 = 0.05
SIGMA_P = 1e-5     # rad, white noise on e
SIGMA_PT = 0.01    # Nm, white noise on the measured torque
W = {"g": -1.0 / np.array([80, 90, 100, 110, 120, 130, 140.0]),
     "tau_dyn": -1.0 / np.array([150, 160, 170, 180, 190, 200, 210.0]),
     "ddq": np.array([2, 1.5, 1, 2.5, 3, 1, 2]) * 1e-4,
     "dq": -np.array([30, 25, 35, 20, 40, 30, 15]) * 1e-3,
     "coulomb": np.array([4, -3, 2, 5, 1, -2, 3]) * 1e-4,
     "constant": np.array([1, -1, 2, 0.5, -0.5, 1.5, -2]) * 1e-4}
TAU_W = {"g": 0.9, "tau_dyn": 1.1, "ddq": 0.02, "dq": 0.5, "coulomb": 0.3,
         "constant": 0.05}
design, noise, predict_paths = [], [], []
for k in range(PREDICT_RUNS):  # a different reference per run
    run = blank_track(N_PREDICT)
    t = run["t"]
    for j in range(DOF):
        run["qref"][:, j], run["dqref"][:, j], run["ddqref"][:, j] = (
            multisine(t, [(0.4, 0.15 + 0.05 * k + 0.02 * j, 0.7 * j + k),
                          (0.1, 0.6 + 0.1 * k + 0.03 * j, 1.3 * j)]))
    q, dq, ddq = fl.reference_derivatives(logged(run["qref"]), DT, SIGMA_S)
    gravity, dynamics = synthetic.torques(q, dq, ddq)
    run_design, run_noise = [], []
    for j in range(ARM):
        columns = {"g": gravity[:, j], "tau_dyn": dynamics[:, j] - gravity[:, j],
                   "ddq": ddq[:, j], "dq": dq[:, j],
                   "coulomb": np.tanh(dq[:, j] / V0),
                   "constant": np.ones(N_PREDICT)}
        n_e = SIGMA_P * rng.standard_normal(N_PREDICT)
        e = sum(W[name][j] * columns[name] for name in columns) + n_e
        run["q"][:, j] = run["qref"][:, j] + e
        run["tau"][:, j] = (sum(TAU_W[name] * columns[name] for name in columns)
                            + SIGMA_PT * rng.standard_normal(N_PREDICT))
        run_design.append(columns)
        run_noise.append(n_e)
    run["q"][:, 7] = run["qref"][:, 7]
    run["applied"] = run["q"].copy()
    design.append(run_design)
    noise.append(np.array(run_noise))
    predict_paths.append(write_log(f"predict{k}.csv", run))
names = [f"run{k}" for k in range(PREDICT_RUNS)]
runs = [(name, fl.load_tracking(path, fl.PREDICT_COLUMNS))
        for name, path in zip(names, predict_paths)]
result = fl.predict(runs, synthetic.torques, SIGMA_S, V0)
assert [run["name"] for run in result["runs"]] == names
assert list(result["sets"]) == ["full", "gravity_lag", "lag_only"]
assert result["sets"]["gravity_lag"]["columns"] == ["g", "dq", "constant"]
assert result["sets"]["lag_only"]["columns"] == ["dq", "constant"]


def pooled_design(j, kept):
    return np.column_stack([np.concatenate([design[k][j][name]
                                            for k in range(PREDICT_RUNS)])
                            for name in kept])


# Pooled weights: within six standard errors plus the ridge-bias bound.
# Joint 0 has no gravity torque: g is dropped, listed, never fitted.
for j, joint in enumerate(result["pooled_full"]):
    kept = [name for name in fl.REGRESSORS if not (j == 0 and name == "g")]
    assert joint["dropped"] == ([] if j else ["g"]), joint
    x = pooled_design(j, kept)
    planted = np.array([W[name][j] for name in kept])
    fitted = np.array([joint["weights"][name] for name in kept])
    tolerance = 6 * ols_standard_errors(x, SIGMA_P) + ridge_bias_bound(
        x, planted)
    assert np.all(np.abs(fitted - planted) <= tolerance), (j, fitted - planted,
                                                           tolerance)
    if j == 0:
        assert joint["weights"]["g"] is None
        assert joint["stiffness_nm_per_rad"] is None
    else:
        assert joint["stiffness_nm_per_rad"] == -1.0 / joint["weights"]["g"]
    assert joint["lag_s"] == -joint["weights"]["dq"]
    assert joint["coulomb_rad"] == joint["weights"]["coulomb"]
    assert joint["constant_rad"] == joint["weights"]["constant"]
    assert joint["rows"] == PREDICT_RUNS * N_PREDICT

    # The measured-torque check recovers its own planted law likewise.
    check = result["measured_torque"][j]
    assert check["dropped"] == joint["dropped"]
    fitted = np.array([check[fl.TORQUE_TERMS[name]] for name in kept])
    planted = np.array([TAU_W[name] for name in kept])
    tolerance = 6 * ols_standard_errors(x, SIGMA_PT) + ridge_bias_bound(
        x, planted)
    assert np.all(np.abs(fitted - planted) <= tolerance), (j, fitted, planted)
    assert (check["a"] is None) == (j == 0) and check["r2"] > 0.99

# Leave-one-run-out with the full set: a held-out run is predicted to its
# own noise. The estimation error of 6 weights from 4500 rows adds about
# 6/4500 to the residual's power and the ridge bias less than 0.1 %, so 2 %
# is a wide margin; the baseline is hundreds of noise levels.
full = result["sets"]["full"]
for k, entry in enumerate(full["leave_one_out"]["held_out"]):
    assert entry["run"] == names[k]
    realized = rms(noise[k])
    assert abs(entry["residual_rms_rad"] / realized - 1) < 0.02, entry
    assert entry["baseline_rms_rad"] > 300 * SIGMA_P
    assert entry["joints"][0]["dropped"] == ["g"]
    assert all(joint["dropped"] == [] for joint in entry["joints"][1:])
    for j, joint in enumerate(entry["joints"]):
        assert abs(joint["residual_rms_rad"] / rms(noise[k][j]) - 1) < 0.05
    in_sample = full["in_sample"][k]
    assert in_sample["run"] == names[k]
    assert abs(in_sample["residual_rms_rad"] / realized - 1) < 0.02
ratios = [entry["baseline_rms_rad"] / entry["residual_rms_rad"]
          for entry in full["leave_one_out"]["held_out"]]
assert full["leave_one_out"]["mean_baseline_over_residual"] == float(
    np.mean(ratios))
# Without the model terms the held-out error stays far above the noise.
for entry in result["sets"]["lag_only"]["leave_one_out"]["held_out"]:
    assert entry["residual_rms_rad"] > 30 * SIGMA_P

# The CLI through the model-torque binary's interface (a stand-in that
# reads the reference columns by name and refuses an existing output)
# gives the same results as the injected provider.
run_args = [f"--run={name}={path}" for name, path in zip(names, predict_paths)]
out_json = ROOT / "predict.json"
status, out, err = run_main("predict", "--model-torque", fake, "--model",
                            model, *run_args, "--json", out_json)
assert status == 0 and err == "", err
assert "e = q - qref" in out and "dropped" in out
# Both weight tables carry the caption that their weights are not the
# result (the held-out residual is), in the table and in the JSON.
assert out.count(f"  note: {fl.WEIGHTS_NOTE}\n") == 2, out
assert "not identified" in fl.WEIGHTS_NOTE and "held-out" in fl.WEIGHTS_NOTE
loaded = json.loads(out_json.read_text())
assert list(loaded) == ["format", "version", "command", "sign_convention",
                        "smooth_sigma_s", "coulomb_v0_rad_s",
                        "ridge_penalty_per_sample", "negligible_column_rms",
                        "weight_units", "runs", "sets", "pooled_full",
                        "pooled_full_note", "measured_torque",
                        "measured_torque_note"]
for key in ("runs", "sets", "pooled_full", "pooled_full_note",
            "measured_torque", "measured_torque_note"):
    assert loaded[key] == result[key], key
assert loaded["pooled_full_note"] == fl.WEIGHTS_NOTE
assert loaded["measured_torque_note"] == fl.WEIGHTS_NOTE
assert set(loaded["sets"]["full"]) == {"columns", "leave_one_out",
                                       "in_sample"}
assert set(loaded["sets"]["full"]["leave_one_out"]) == {
    "held_out", "mean_baseline_over_residual"}
assert set(loaded["sets"]["full"]["leave_one_out"]["held_out"][0]) == {
    "run", "baseline_rms_rad", "residual_rms_rad", "joints"}
assert set(loaded["sets"]["full"]["in_sample"][0]["joints"][0]) == {
    "joint", "baseline_rms_rad", "residual_rms_rad", "dropped"}
assert set(loaded["pooled_full"][0]) == {
    "joint", "weights", "dropped", "stiffness_nm_per_rad", "lag_s",
    "coulomb_rad", "constant_rad", "r2", "residual_rms_rad", "rows"}
assert set(loaded["measured_torque"][0]) == {
    "joint", "a", "b", "J_kg_m2", "fv_nm_s_per_rad", "fc_nm", "c_nm",
    "dropped", "r2", "residual_rms_nm", "tau_rms_nm", "rows"}
assert loaded["pooled_full"][0]["stiffness_nm_per_rad"] is None
assert loaded["weight_units"]["dq"] == "s"

# A failing or short model-torque output, one whose header is not exactly
# the interface's or whose schema_version is not 1 on every row, too few
# runs, a missing column, or a missing model file all exit nonzero and say
# why, naming the binary for its output.
for mode, text in (
        ("fail", "failed on purpose"), ("short", "output rows"),
        ("schema", "schema_version '2', expected 1"),
        ("renamed", "column 18 is 'inverse_dynamics7', expected "
                    "'inverse_dynamics7_nm'"),
        ("reordered", "column 3 is 'inverse_dynamics0_nm', expected "
                      "'gravity0_nm'"),
        ("extra", "unexpected extra column 'note'")):
    os.environ["FAKE_TORQUE_MODE"] = mode
    status, out, err = run_main("predict", "--model-torque", fake, "--model",
                                model, *run_args)
    del os.environ["FAKE_TORQUE_MODE"]
    assert status == 1 and text in err and out == "", (mode, err)
    if mode != "fail":
        assert f"{fake}: " in err, (mode, err)
    if mode in ("renamed", "reordered", "extra"):
        assert "unexpected output header" in err, (mode, err)
status, out, err = run_main("predict", "--model-torque", fake, "--model",
                            model, run_args[0])
assert status == 1 and "at least 2 runs" in err and out == ""
raises(fl.predict, runs[:1], synthetic.torques, contains=("at least 2",))
raises(fl.predict, [runs[0], ("run0", runs[1][1])], synthetic.torques,
       contains=("not unique",))
no_error2 = write_log("predict_no_error2.csv", blank_track(50),
                      drop=("error2_rad",))
status, _, err = run_main("predict", "--model-torque", fake, "--model", model,
                          run_args[0], f"--run=bad={no_error2}")
assert status == 1 and "error2_rad" in err
status, _, err = run_main("predict", "--model-torque", fake, "--model",
                          ROOT / "absent.ztk", *run_args)
assert status == 1 and "absent.ztk" in err
status, _, err = run_main("predict", "--model-torque", ROOT / "absent_bin",
                          "--model", model, *run_args)
assert status == 1 and "absent_bin" in err

# Every subcommand's help states the sign convention and the units.
for command in ("anatomy", "repeat", "predict"):
    status, out, _ = run_main(command, "--help")
    assert status == 0 and "e = q - qref" in out and "SI" in out, command
    assert "mrad" in out, command
    if command == "predict":
        assert "weights, is the result to read" in out, out

SCRATCH.cleanup()
print("follow learnability regressions OK")
