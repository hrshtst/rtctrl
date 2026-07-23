#!/usr/bin/env python3
"""Offline analysis for x7_ident / x7_ident_sim telemetry.

Per docs/IDENTIFICATION_PLAN.md: per-dwell dt-weighted least-squares
demodulation on the regressors [1, t, sin(phi), cos(phi)] over the
MEASURE window, the direct-ratio FRF (tau_meas primary; commanded +
delay-corrected secondary; their ratio = the actuator transfer),
all-joint participation vectors, a complex SDOF fit with residuals and
grid confidence intervals, repeat/half-amplitude consistency flags, the
anchor merge guard (+/-0.02 rad), and a per-posture mode table.

Usage:
    uv run --project tools tools/ident_analysis.py run1.csv [run2.csv ...]
        [--out prefix] [--joint-count 8]

Multiple CSVs (e.g. the survey and the up/down refinement invocations)
are combined ONLY when their recorded anchors agree within the
tolerance — cross-invocation combination is legitimate by design
through the anchor-reference gate, and illegitimate otherwise.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field

import numpy as np

MEASURE_PHASE = 3  # IdentRun::Phase::Measure
ANCHOR_TOL_RAD = 0.02
DOF = 8


@dataclass
class Dwell:
    source: str
    freq_hz: float
    amp_nm: float
    n_rows: int
    window_s: float
    z_q: np.ndarray  # complex, per joint [rad]
    z_tau_meas: complex  # probe-joint measured torque [Nm]
    z_cmd: complex  # commanded probe torque [Nm]
    delay_s: float  # median matched first-apply delay
    probe_joint: int
    fit_residual_rms: float  # probe-joint q LS residual [rad]
    clipped_cycles: int
    flags: list[str] = field(default_factory=list)

    @property
    def h_primary(self) -> complex:
        return self.z_q[self.probe_joint] / self.z_tau_meas

    @property
    def h_secondary(self) -> complex:
        # commanded, phase-corrected to the plant by the measured
        # first-apply delay
        corr = np.exp(-1j * 2 * np.pi * self.freq_hz * self.delay_s)
        return self.z_q[self.probe_joint] / (self.z_cmd * corr)

    @property
    def actuator_transfer(self) -> complex:
        return self.z_tau_meas / (
            self.z_cmd
            * np.exp(-1j * 2 * np.pi * self.freq_hz * self.delay_s)
        )


def load_run(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Returns (data, anchor). skip_header=1 skips the '#' semantics
    line — genfromtxt(names=True) would otherwise take IT as the column
    names."""
    data = np.genfromtxt(path, delimiter=",", names=True, skip_header=1)
    if data.size == 0:
        raise SystemExit(f"{path}: no data rows")
    anchor = np.array([data[f"qd{i}"][0] for i in range(DOF)])
    return data, anchor


def demod_dwells(path: str, data: np.ndarray) -> list[Dwell]:
    dwells: list[Dwell] = []
    ids = np.unique(data["dwell_id"].astype(int))
    for d in ids[ids >= 0]:
        rows = data[
            (data["dwell_id"].astype(int) == d)
            & (data["dwell_phase"].astype(int) == MEASURE_PHASE)
        ]
        if rows.size < 16:
            continue  # skipped or aborted dwell: no measure window
        t = rows["control_t"]
        phi = rows["probe_phase"]
        freq = float(rows["probe_hz"][0])
        pj = int(rows["probe_joint"][0])
        # dt weights from the measured feedback stamps
        dt = np.diff(rows["feedback_time"], prepend=rows["feedback_time"][0])
        dt[0] = np.median(dt[1:]) if rows.size > 1 else 0.01
        w = np.sqrt(dt / np.sum(dt))
        # regressors [1, t, sin phi, cos phi]: offset and drift are
        # absorbed explicitly; exact for any window end
        basis = np.column_stack(
            [np.ones_like(t), t - t[0], np.sin(phi), np.cos(phi)]
        )
        bw = basis * w[:, None]

        def fit(signal: np.ndarray) -> tuple[complex, float]:
            coef, *_ = np.linalg.lstsq(bw, signal * w, rcond=None)
            resid = signal - basis @ coef
            return complex(coef[2], coef[3]), float(
                np.sqrt(np.mean(resid**2))
            )

        z_q = np.zeros(DOF, dtype=complex)
        resid_pj = 0.0
        for i in range(DOF):
            z_q[i], r = fit(rows[f"q{i}"])
            if i == pj:
                resid_pj = r
        z_tau, _ = fit(rows[f"taumeas{pj}"])
        z_cmd, _ = fit(rows["probe_tau"])
        delays = rows["first_apply_delay"]
        delays = delays[np.isfinite(delays)]
        dwells.append(
            Dwell(
                source=path,
                freq_hz=freq,
                amp_nm=float(rows["probe_amp"][0]),
                n_rows=int(rows.size),
                window_s=float(t[-1] - t[0]),
                z_q=z_q,
                z_tau_meas=z_tau,
                z_cmd=z_cmd,
                delay_s=float(np.median(delays)) if delays.size else 0.0,
                probe_joint=pj,
                fit_residual_rms=resid_pj,
                clipped_cycles=int(np.sum(rows["probe_clipped"] > 0)),
            )
        )
    return dwells


def sdof_fit(
    freqs: np.ndarray, h: np.ndarray
) -> tuple[float, float, float, tuple[float, float], tuple[float, float]]:
    """Complex SDOF grid fit H ~ c*G + d, G = 1/(fn^2 - f^2 + 2j z fn f).
    Returns (fn, zeta, residual_rms, fn_ci, zeta_ci) where the CIs are
    the grid extent of the Delta-chi^2 <= chi^2_min * 2/(N-4) band —
    a coarse but honest interval from the residual surface."""
    fn_grid = np.arange(max(0.5, freqs.min() * 0.8), freqs.max() * 1.2, 0.01)
    z_grid = np.geomspace(0.005, 0.3, 120)
    best = (0.0, 0.0)
    best_res = np.inf
    res_map = np.full((fn_grid.size, z_grid.size), np.inf)
    for a, fn in enumerate(fn_grid):
        g = 1.0 / (fn**2 - freqs**2 + 2j * z_grid[:, None] * fn * freqs)
        # per zeta row: linear LS for complex c, d
        n = float(freqs.size)
        sgg = np.sum(g * np.conj(g), axis=1).real
        sg = np.sum(g, axis=1)
        shg = np.sum(h[None, :] * np.conj(g), axis=1)
        sh = np.sum(h)
        det = sgg * n - np.abs(sg) ** 2
        ok = det > 1e-18
        c = np.where(ok, (shg * n - sh * np.conj(sg)) / det, 0.0)
        dd = np.where(ok, (sgg * sh - sg * shg) / det, 0.0)
        res = np.sum(
            np.abs(h[None, :] - c[:, None] * g - dd[:, None]) ** 2, axis=1
        )
        res_map[a] = res
        b = int(np.argmin(res))
        if res[b] < best_res:
            best_res = float(res[b])
            best = (float(fn), float(z_grid[b]))
    dof = max(1, freqs.size - 4)
    band = res_map <= best_res * (1.0 + 2.0 / dof)
    fn_in = fn_grid[np.any(band, axis=1)]
    z_in = z_grid[np.any(band, axis=0)]
    return (
        best[0],
        best[1],
        float(np.sqrt(best_res / freqs.size)),
        (float(fn_in.min()), float(fn_in.max())),
        (float(z_in.min()), float(z_in.max())),
    )


def consistency_flags(dwells: list[Dwell]) -> None:
    """Repeat-visit and half-amplitude checks: same frequency measured
    more than once (up/down sweeps arrive as separate files) must agree;
    a half-amplitude repeat that moves the FRF point flags amplitude
    dependence (gear compliance/backlash)."""
    by_freq: dict[float, list[Dwell]] = {}
    for d in dwells:
        by_freq.setdefault(round(d.freq_hz, 3), []).append(d)
    for freq, group in by_freq.items():
        if len(group) < 2:
            continue
        amps = np.array([g.amp_nm for g in group])
        h = np.array([g.h_primary for g in group])
        same_amp = np.isclose(amps, amps[0], rtol=0.1).all()
        spread = np.max(np.abs(h - np.mean(h))) / max(
            np.mean(np.abs(h)), 1e-12
        )
        if same_amp and spread > 0.2:
            for g in group:
                g.flags.append(
                    f"repeat mismatch at {freq} Hz: |dH|/|H| {spread:.2f}"
                )
        if not same_amp and spread > 0.2:
            for g in group:
                g.flags.append(
                    f"amplitude-dependent FRF at {freq} Hz "
                    f"(half-amplitude check): |dH|/|H| {spread:.2f}"
                )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", nargs="+", help="ident telemetry CSV(s)")
    ap.add_argument("--out", default="ident_analysis", help="output prefix")
    args = ap.parse_args()

    # anchor merge guard: refuse to combine runs from different postures
    runs = []
    anchor0 = None
    for path in args.csv:
        data, anchor = load_run(path)
        if anchor0 is None:
            anchor0 = anchor
        else:
            delta = np.abs(anchor - anchor0)
            if np.any(delta > ANCHOR_TOL_RAD):
                bad = int(np.argmax(delta))
                raise SystemExit(
                    f"ANCHOR MERGE GUARD: {path} anchor differs by "
                    f"{delta[bad]:.4f} rad on joint {bad} (> "
                    f"{ANCHOR_TOL_RAD}) — separate postures cannot form "
                    "one FRF dataset"
                )
        runs.append((path, data))

    assert anchor0 is not None
    dwells: list[Dwell] = []
    for path, data in runs:
        dwells.extend(demod_dwells(path, data))
    if not dwells:
        raise SystemExit("no completed measurement windows found")
    dwells.sort(key=lambda d: d.freq_hz)
    consistency_flags(dwells)

    pj = dwells[0].probe_joint
    if any(d.probe_joint != pj for d in dwells):
        raise SystemExit(
            "runs probe DIFFERENT joints — one FRF dataset per probe "
            "joint; analyze them separately"
        )
    freqs = np.array([d.freq_hz for d in dwells])
    h = np.array([d.h_primary for d in dwells])
    # refinement evidence: the fit is survey-confidence only unless the
    # local grid near the |H| peak is fine enough to resolve zeta~0.03
    peak = int(np.argmax(np.abs(h)))
    local = np.abs(freqs - freqs[peak]) <= max(1.0, 0.15 * freqs[peak])
    local_step = (
        np.min(np.diff(np.unique(freqs[local])))
        if np.sum(local) > 2
        else np.inf
    )
    refined = local_step <= 0.35
    fn, zeta, resid, fn_ci, z_ci = sdof_fit(freqs, h)

    # all-joint participation at the peak dwell, normalized to the probe
    part = np.abs(dwells[peak].z_q) / max(
        np.abs(dwells[peak].z_q[pj]), 1e-15
    )

    table = {
        "probe_joint": pj,
        "anchor": [float(a) for a in anchor0],
        "sources": [p for p, _ in runs],
        "mode": {
            "f_n_hz": fn,
            "zeta": zeta,
            "f_n_ci_hz": list(fn_ci),
            "zeta_ci": list(z_ci),
            "fit_residual_rad_per_nm": resid,
            "confidence": "refined" if refined else "survey-only",
            "peak_abs_h_rad_per_nm": float(np.abs(h[peak])),
            "peak_freq_hz": float(freqs[peak]),
            "participation": [float(p) for p in part],
        },
        "actuator_transfer": [
            {
                "freq_hz": d.freq_hz,
                "mag": float(np.abs(d.actuator_transfer)),
                "phase_deg": float(
                    np.degrees(np.angle(d.actuator_transfer))
                ),
            }
            for d in dwells
        ],
        "dwells": [
            {
                "source": d.source,
                "freq_hz": d.freq_hz,
                "amp_nm": d.amp_nm,
                "h_primary": [float(d.h_primary.real), float(d.h_primary.imag)],
                "h_secondary": [
                    float(d.h_secondary.real),
                    float(d.h_secondary.imag),
                ],
                "window_s": d.window_s,
                "delay_s": d.delay_s,
                "fit_residual_rad": d.fit_residual_rms,
                "clipped_cycles": d.clipped_cycles,
                "flags": d.flags,
            }
            for d in dwells
        ],
    }
    with open(f"{args.out}.mode_table.json", "w") as f:
        json.dump(table, f, indent=1)

    lines = [
        f"# Mode table — probe joint {pj}",
        "",
        f"- fitted mode: **{fn:.2f} Hz**, zeta **{zeta:.4f}** "
        f"(CI {fn_ci[0]:.2f}-{fn_ci[1]:.2f} Hz, "
        f"{z_ci[0]:.4f}-{z_ci[1]:.4f})",
        f"- confidence: **{'refined' if refined else 'SURVEY-ONLY'}**"
        + ("" if refined else " — run the refinement sweeps"),
        f"- peak |H| {np.abs(h[peak]):.4f} rad/Nm at "
        f"{freqs[peak]:.2f} Hz; fit residual {resid:.5f} rad/Nm",
        f"- participation (|q_i|/|q_probe| at the peak): "
        + ", ".join(f"j{i}={part[i]:.2f}" for i in range(DOF) if part[i] > 0.05),
        "",
        "| f [Hz] | A [Nm] | \\|H\\| [rad/Nm] | phase [deg] | window [s] | flags |",
        "|---|---|---|---|---|---|",
    ]
    for d, hp in zip(dwells, h):
        lines.append(
            f"| {d.freq_hz:.2f} | {d.amp_nm:.3f} | {np.abs(hp):.5f} | "
            f"{np.degrees(np.angle(hp)):.1f} | {d.window_s:.2f} | "
            f"{'; '.join(d.flags)} |"
        )
    with open(f"{args.out}.mode_table.md", "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"fitted mode: {fn:.2f} Hz, zeta {zeta:.4f} "
          f"({'refined' if refined else 'SURVEY-ONLY'})")
    print(f"  CI: {fn_ci[0]:.2f}-{fn_ci[1]:.2f} Hz, "
          f"zeta {z_ci[0]:.4f}-{z_ci[1]:.4f}")
    for d in dwells:
        if d.flags:
            print(f"  FLAG {d.freq_hz:.2f} Hz: {'; '.join(d.flags)}")
    print(f"wrote {args.out}.mode_table.json / .md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
