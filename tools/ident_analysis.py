#!/usr/bin/env python3
"""Offline analysis for x7_ident / x7_ident_sim telemetry.

Per docs/IDENTIFICATION_PLAN.md: per-dwell dt-weighted least-squares
demodulation on the regressors [1, t, sin(phi), cos(phi)] over the
MEASURE windows, the direct-ratio FRF (tau_meas primary; the SUBMITTED
TOTAL commanded torque, delay-corrected, secondary; their ratio = the
actuator transfer), all-joint participation vectors, complex SDOF fits
per detected mode band with residuals and grid confidence intervals,
repeat-visit and half-amplitude consistency checks, the anchor merge
guard (+/-0.02 rad), and per-posture mode tables.

Dwell verdicts come from each run's `<csv>.dwells.json` sidecar:
skipped/incomplete dwells are dropped, low-confidence dwells are
excluded from the mode fits and flagged. "Refined" confidence requires
all three pieces of evidence near a peak: a grid fine enough for
zeta ~ 0.03, visits from at least two invocations (the up/down
sweeps), and the half-amplitude linearity point.

Usage:
    uv run --project tools tools/ident_analysis.py run1.csv [run2.csv ...]
        [--out prefix]

Multiple CSVs are combined ONLY when their recorded anchors agree
within the tolerance and they probe the same joint.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field

import numpy as np

MEASURE_PHASE = 3  # IdentRun::Phase::Measure
ANCHOR_TOL_RAD = 0.02
DOF = 8
REFINE_STEP_HZ = 0.35  # local spacing needed to resolve zeta ~ 0.03


@dataclass
class Dwell:
    source: str
    freq_hz: float
    amp_nm: float
    n_rows: int
    window_s: float
    z_q: np.ndarray  # complex, per joint [rad]
    z_tau_meas: complex  # probe-joint measured torque [Nm]
    z_cmd: complex  # SUBMITTED TOTAL probe-joint torque [Nm]
    delay_s: float  # median matched first-apply delay
    probe_joint: int
    fit_residual_rms: float  # probe-joint q LS residual [rad]
    clipped_cycles: int
    low_confidence: bool = False
    flags: list[str] = field(default_factory=list)

    @property
    def h_primary(self) -> complex:
        return self.z_q[self.probe_joint] / self.z_tau_meas

    @property
    def h_secondary(self) -> complex:
        # total commanded, phase-corrected to the plant by the measured
        # first-apply delay
        corr = np.exp(-1j * 2 * np.pi * self.freq_hz * self.delay_s)
        return self.z_q[self.probe_joint] / (self.z_cmd * corr)

    @property
    def actuator_transfer(self) -> complex:
        # commanded-to-measured: tau_meas over the delay-corrected
        # TOTAL command (using only the injected probe here would fold
        # the feedback controller's reaction into the actuator model)
        return self.z_tau_meas / (
            self.z_cmd
            * np.exp(-1j * 2 * np.pi * self.freq_hz * self.delay_s)
        )


def load_run(path: str) -> tuple[np.ndarray, np.ndarray, list | None]:
    """Returns (data, anchor, sidecar dwell verdicts). skip_header=1
    skips the '#' semantics line — genfromtxt(names=True) would
    otherwise take IT as the column names."""
    data = np.genfromtxt(path, delimiter=",", names=True, skip_header=1)
    if data.size == 0:
        raise SystemExit(f"{path}: no data rows")
    anchor = np.array([data[f"qd{i}"][0] for i in range(DOF)])
    sidecar = None
    sc_path = path + ".dwells.json"
    if os.path.exists(sc_path):
        with open(sc_path) as f:
            sidecar = json.load(f).get("dwells")
    else:
        print(f"WARNING: {sc_path} missing — dwell verdicts unknown",
              file=sys.stderr)
    return data, anchor, sidecar


def demod_dwells(path: str, data: np.ndarray,
                 sidecar: list | None) -> list[Dwell]:
    dwells: list[Dwell] = []
    ids = np.unique(data["dwell_id"].astype(int))
    for d in ids[ids >= 0]:
        verdict = (sidecar[d] if sidecar is not None and
                   d < len(sidecar) else None)
        if verdict is not None and (
                verdict.get("skipped") or not verdict.get("completed")):
            continue  # no legitimate measurement window
        rows = data[
            (data["dwell_id"].astype(int) == d)
            & (data["dwell_phase"].astype(int) == MEASURE_PHASE)
        ]
        if rows.size < 16:
            continue
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
        z_cmd, _ = fit(rows["cmd_total"])
        delays = rows["first_apply_delay"]
        delays = delays[np.isfinite(delays)]
        dw = Dwell(
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
        if verdict is None:
            dw.flags.append("no sidecar: verdict unknown")
        elif verdict.get("low_confidence"):
            dw.low_confidence = True
            dw.flags.append(
                "low-confidence (" + verdict.get("note", "") + ")"
            )
        dwells.append(dw)
    return dwells


def sdof_fit(
    freqs: np.ndarray, h: np.ndarray
) -> tuple[float, float, float, tuple[float, float], tuple[float, float]]:
    """Complex SDOF fit on a fine local grid: H(f) ~ c*G(f) + d with
    G = 1/(fn^2 - f^2 + 2 j zeta fn f); c, d solved by linear least
    squares per (f_n, zeta) candidate, best residual wins. CIs are the
    grid extent of the Delta-chi^2 <= chi^2_min * 2/(N-4) band."""
    fn_grid = np.arange(max(0.5, freqs.min() * 0.8), freqs.max() * 1.2, 0.01)
    z_grid = np.geomspace(0.005, 0.3, 120)
    best = (0.0, 0.0)
    best_res = np.inf
    res_map = np.full((fn_grid.size, z_grid.size), np.inf)
    for a, fn in enumerate(fn_grid):
        g = 1.0 / (fn**2 - freqs**2 + 2j * z_grid[:, None] * fn * freqs)
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


def amp_classes(group: list[Dwell]) -> tuple[list[Dwell], list[Dwell]]:
    """Split same-frequency dwells into full-amplitude and reduced
    (half-amplitude linearity) classes by the largest amplitude."""
    a_max = max(g.amp_nm for g in group)
    full = [g for g in group if g.amp_nm > 0.75 * a_max]
    reduced = [g for g in group if g.amp_nm <= 0.75 * a_max]
    return full, reduced


def consistency_flags(dwells: list[Dwell]) -> None:
    """Repeat visits (same frequency, FULL amplitude — the up/down
    sweep evidence) must agree with each other; the half-amplitude
    point is validated separately against the full-amplitude mean
    (amplitude dependence = gear compliance/backlash)."""
    by_freq: dict[float, list[Dwell]] = {}
    for d in dwells:
        by_freq.setdefault(round(d.freq_hz, 3), []).append(d)
    for freq, group in by_freq.items():
        full, reduced = amp_classes(group)
        if len(full) >= 2:
            h = np.array([g.h_primary for g in full])
            spread = np.max(np.abs(h - np.mean(h))) / max(
                np.mean(np.abs(h)), 1e-12
            )
            if spread > 0.2:
                for g in full:
                    g.flags.append(
                        f"repeat mismatch at {freq} Hz: "
                        f"|dH|/|H| {spread:.2f}"
                    )
        if full and reduced:
            h_full = np.mean([g.h_primary for g in full])
            for g in reduced:
                shift = abs(g.h_primary - h_full) / max(
                    abs(h_full), 1e-12
                )
                note = (
                    f"half-amplitude linearity at {freq} Hz: "
                    f"|dH|/|H| {shift:.3f}"
                )
                g.flags.append(
                    note + (" — AMPLITUDE-DEPENDENT" if shift > 0.2
                            else " (linear)")
                )


def find_peaks(freqs: np.ndarray, mags: np.ndarray) -> list[int]:
    """Interior local maxima of |H| over the unique-frequency profile,
    with a mild prominence rule; the global maximum always counts. One
    mode band per peak (the protocol expects BOTH the 4-5 Hz and the
    ~13 Hz bands from one survey)."""
    peaks = []
    floor = 2.0 * float(np.median(mags))
    for i in range(1, len(freqs) - 1):
        if mags[i] > mags[i - 1] and mags[i] >= mags[i + 1] and (
                mags[i] >= floor):
            peaks.append(i)
    gmax = int(np.argmax(mags))
    if gmax not in peaks and 0 < gmax < len(freqs) - 1:
        peaks.append(gmax)
    peaks.sort()
    # merge peaks that fall inside one another's band
    merged: list[int] = []
    for p in peaks:
        if merged and freqs[p] <= 1.4 * freqs[merged[-1]]:
            if mags[p] > mags[merged[-1]]:
                merged[-1] = p
        else:
            merged.append(p)
    return merged


def mode_entry(band: list[Dwell], all_dwells: list[Dwell],
               f_pk: float) -> dict:
    freqs = np.array([d.freq_hz for d in band])
    h = np.array([d.h_primary for d in band])
    fn, zeta, resid, fn_ci, z_ci = sdof_fit(freqs, h)
    peak = band[int(np.argmax(np.abs(h)))]
    part = np.abs(peak.z_q) / max(np.abs(peak.z_q[peak.probe_joint]),
                                  1e-15)
    # refinement evidence, all three required (review finding):
    missing = []
    local = np.abs(freqs - f_pk) <= max(1.0, 0.15 * f_pk)
    uf = np.unique(np.round(freqs[local], 3))
    if uf.size < 3 or np.min(np.diff(uf)) > REFINE_STEP_HZ:
        missing.append("grid too coarse for zeta~0.03")
    if len({d.source for d in band}) < 2:
        missing.append("needs visits from a second (up/down) invocation")
    near_pk = [d for d in all_dwells
               if abs(d.freq_hz - f_pk) <= max(0.2, 0.05 * f_pk)]
    full, reduced = amp_classes(near_pk) if near_pk else ([], [])
    if not (full and reduced):
        missing.append("no half-amplitude linearity point at the peak")
    return {
        "f_n_hz": fn,
        "zeta": zeta,
        "f_n_ci_hz": list(fn_ci),
        "zeta_ci": list(z_ci),
        "fit_residual_rad_per_nm": resid,
        "confidence": "refined" if not missing else "survey-only",
        "missing_evidence": missing,
        "band_hz": [float(freqs.min()), float(freqs.max())],
        "peak_abs_h_rad_per_nm": float(np.max(np.abs(h))),
        "peak_freq_hz": float(peak.freq_hz),
        "participation": [float(p) for p in part],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", nargs="+", help="ident telemetry CSV(s)")
    ap.add_argument("--out", default="ident_analysis", help="output prefix")
    args = ap.parse_args()

    # anchor merge guard: refuse to combine runs from different postures
    runs = []
    anchor0 = None
    for path in args.csv:
        data, anchor, sidecar = load_run(path)
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
        runs.append((path, data, sidecar))

    assert anchor0 is not None
    dwells: list[Dwell] = []
    for path, data, sidecar in runs:
        dwells.extend(demod_dwells(path, data, sidecar))
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

    # low-confidence windows are excluded from the mode fits
    fit_dwells = [d for d in dwells if not d.low_confidence]
    if len(fit_dwells) < 5:
        print("WARNING: fewer than 5 confident dwells — fitting all, "
              "treat the result as indicative only", file=sys.stderr)
        fit_dwells = dwells

    uf = np.unique(np.round([d.freq_hz for d in fit_dwells], 3))
    prof = np.array([
        np.mean([abs(d.h_primary) for d in fit_dwells
                 if round(d.freq_hz, 3) == f]) for f in uf
    ])
    modes = []
    for p in find_peaks(uf, prof):
        f_pk = float(uf[p])
        band = [d for d in fit_dwells
                if 0.7 * f_pk <= d.freq_hz <= 1.4 * f_pk]
        if len(band) < 5:
            modes.append({
                "peak_freq_hz": f_pk,
                "confidence": "insufficient",
                "missing_evidence": [
                    f"only {len(band)} dwells in the band"],
            })
            continue
        modes.append(mode_entry(band, dwells, f_pk))
    if not modes:
        raise SystemExit("no mode peak detected in |H|")

    table = {
        "probe_joint": pj,
        "anchor": [float(a) for a in anchor0],
        "sources": [p for p, _, _ in runs],
        "modes": modes,
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
                "low_confidence": d.low_confidence,
                "flags": d.flags,
            }
            for d in dwells
        ],
    }
    with open(f"{args.out}.mode_table.json", "w") as f:
        json.dump(table, f, indent=1)

    lines = [f"# Mode table — probe joint {pj}", ""]
    for m in modes:
        if m["confidence"] == "insufficient":
            lines.append(
                f"- peak near {m['peak_freq_hz']:.2f} Hz: "
                f"INSUFFICIENT DATA ({'; '.join(m['missing_evidence'])})"
            )
            continue
        lines.append(
            f"- mode **{m['f_n_hz']:.2f} Hz**, zeta **{m['zeta']:.4f}** "
            f"(CI {m['f_n_ci_hz'][0]:.2f}-{m['f_n_ci_hz'][1]:.2f} Hz, "
            f"{m['zeta_ci'][0]:.4f}-{m['zeta_ci'][1]:.4f}) — "
            f"**{m['confidence'].upper()}**"
            + (": " + "; ".join(m["missing_evidence"])
               if m["missing_evidence"] else "")
        )
        part = m["participation"]
        lines.append(
            "  participation: "
            + ", ".join(f"j{i}={part[i]:.2f}" for i in range(DOF)
                        if part[i] > 0.05)
        )
    lines += [
        "",
        "| f [Hz] | A [Nm] | \\|H\\| [rad/Nm] | phase [deg] | window [s] | flags |",
        "|---|---|---|---|---|---|",
    ]
    for d in dwells:
        hp = d.h_primary
        lines.append(
            f"| {d.freq_hz:.2f} | {d.amp_nm:.3f} | {np.abs(hp):.5f} | "
            f"{np.degrees(np.angle(hp)):.1f} | {d.window_s:.2f} | "
            f"{'; '.join(d.flags)} |"
        )
    with open(f"{args.out}.mode_table.md", "w") as f:
        f.write("\n".join(lines) + "\n")

    for m in modes:
        if m["confidence"] == "insufficient":
            print(f"peak near {m['peak_freq_hz']:.2f} Hz: insufficient "
                  f"data ({'; '.join(m['missing_evidence'])})")
        else:
            print(f"mode: {m['f_n_hz']:.2f} Hz, zeta {m['zeta']:.4f} "
                  f"({m['confidence'].upper()}"
                  + (": " + "; ".join(m["missing_evidence"])
                     if m["missing_evidence"] else "") + ")")
    for d in dwells:
        if d.flags:
            print(f"  FLAG {d.freq_hz:.2f} Hz: {'; '.join(d.flags)}")
    print(f"wrote {args.out}.mode_table.json / .md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
