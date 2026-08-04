"""Monte-Carlo study of computed-torque tracking under randomized
mass/COM model error, driving examples/x7_ct_mass_error.

Runs the seeded per-link perturbation sweep plus the correlated
uniform-scale reference runs, records provenance (git commit, model
and binary hashes) alongside the results, and renders the summary
figure. Outputs land in --out (default: build/ct_mass_error_study,
gitignored): results.csv, meta.json, study.png, and the per-run
trajectory CSVs under runs/.

Run from the repo root:  uv run --project tools tools/ct_mass_error_study.py
The binary must be built first (cmake --build build) and the mi-lib
shared libraries resolvable (the repo .envrc provides this).
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import re
import statistics
import subprocess
import sys
from datetime import datetime, timezone

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402  (backend must precede)

REPO = pathlib.Path(__file__).resolve().parents[1]
MODEL = REPO / "models/crane_x7/crane_x7.ztk"
JOINTS = [f"j{i}" for i in range(8)]

# (name, mass_error, com_error, ki); 20 seeds each by default.
CONDITIONS = [
    ("M10", 0.1, 0.0, 0.0),
    ("M20", 0.2, 0.0, 0.0),
    ("M30", 0.3, 0.0, 0.0),
    ("C05", 0.0, 0.005, 0.0),
    ("C10", 0.0, 0.010, 0.0),
    ("C20", 0.0, 0.020, 0.0),
    ("M20C10", 0.2, 0.010, 0.0),
    ("M20C10k", 0.2, 0.010, 6.0),
]
# Correlated uniform-scale references, run live rather than quoted.
SCALES = [0.7, 0.8, 1.2, 1.3]
LABELS = {
    "M10": "mass\n±10%", "M20": "mass\n±20%", "M30": "mass\n±30%",
    "C05": "COM\n±5 mm", "C10": "COM\n±10 mm", "C20": "COM\n±20 mm",
    "M20C10": "both\n20%+10mm", "M20C10k": "both\n+integrator",
}
FAMILY = {
    "M10": "#2a78d6", "M20": "#2a78d6", "M30": "#2a78d6",
    "C05": "#eb6834", "C10": "#eb6834", "C20": "#eb6834",
    "M20C10": "#1baf7a", "M20C10k": "#eda100",
}
INK, MUTED, GRID, SURFACE = "#333331", "#6b6b69", "#e5e5e2", "#fcfcfb"


def sha256(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout.strip()


def run_case(binary: pathlib.Path, out_csv: pathlib.Path, *,
             scale: float = 1.0, em: float = 0.0, ec: float = 0.0,
             ki: float = 0.0, seed: int = 1) -> dict[str, float]:
    cmd = [str(binary), "--mass-scale", str(scale), "--mass-error", str(em),
           "--com-error", str(ec), "--ki", str(ki), "--seed", str(seed),
           "--out", str(out_csv)]
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True,
                          check=True)
    rms = dict(re.findall(r"(j\d|all)(?:-joint)? RMS (\d+\.\d+)",
                          proc.stdout))
    missing = [k for k in [*JOINTS, "all"] if k not in rms]
    if missing:
        raise RuntimeError(f"unparsed RMS fields {missing}:\n{proc.stdout}")
    return {k: float(v) for k, v in rms.items()}


def sweep(binary: pathlib.Path, out: pathlib.Path, seeds: int) -> None:
    runs = out / "runs"
    runs.mkdir(parents=True, exist_ok=True)
    rows = []

    def record(cond: str, em: float, ec: float, ki: float, seed: int,
               **case: float) -> None:
        rms = run_case(binary, runs / f"{cond}_s{seed}.csv", em=em, ec=ec,
                       ki=ki, seed=seed, **case)
        rows.append({"cond": cond, "em": em, "ec": ec, "ki": ki,
                     "seed": seed, **rms})

    record("base", 0.0, 0.0, 0.0, 1)
    record("base_ki", 0.0, 0.0, 6.0, 1)
    for scale in SCALES:
        record(f"U{scale}", 0.0, 0.0, 0.0, 1, scale=scale)
    for seed in range(1, seeds + 1):
        for cond, em, ec, ki in CONDITIONS:
            record(cond, em, ec, ki, seed)
    with open(out / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"{len(rows)} runs -> {out / 'results.csv'}")


def provenance(binary: pathlib.Path, out: pathlib.Path, seeds: int) -> None:
    meta = {
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "argv": sys.argv,
        "git_commit": git("rev-parse", "HEAD"),
        "git_dirty": bool(git("status", "--porcelain")),
        "model": {"path": str(MODEL.relative_to(REPO)),
                  "sha256": sha256(MODEL)},
        "binary": {"path": str(binary), "sha256": sha256(binary)},
        "seeds": list(range(1, seeds + 1)),
        "conditions": CONDITIONS,
        "correlated_scales": SCALES,
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")


def report(out: pathlib.Path) -> None:
    rows = list(csv.DictReader(open(out / "results.csv")))
    by = {c: [r for r in rows if r["cond"] == c] for c, *_ in CONDITIONS}
    single = {r["cond"]: r for r in rows if r["cond"].startswith(("base", "U"))}

    print(f"{'cond':>8} {'j1 med [min-max] mrad':>24}"
          f" {'all med [min-max] mrad':>24}")
    for name, r in single.items():
        print(f"{name:>8} {float(r['j1']) * 1e3:8.1f}"
              f"              {float(r['all']) * 1e3:8.1f}")
    for c in by:
        j1 = sorted(float(r["j1"]) * 1e3 for r in by[c])
        al = sorted(float(r["all"]) * 1e3 for r in by[c])
        print(f"{c:>8} {statistics.median(j1):6.1f} [{j1[0]:4.1f}-{j1[-1]:4.1f}]"
              f"        {statistics.median(al):6.1f}"
              f" [{al[0]:4.1f}-{al[-1]:4.1f}]")
    off = {r["seed"]: float(r["j1"]) for r in by["M20C10"]}
    on = {r["seed"]: float(r["j1"]) for r in by["M20C10k"]}
    wins = sum(1 for s in off if on[s] < off[s])
    print(f"integrator improved j1 in {wins}/{len(off)} paired seeds")

    # Correlated references: the WORSE endpoint of each ± magnitude.
    refs = {"correlated ±20%": ("U0.8", "U1.2"),
            "correlated ±30%": ("U0.7", "U1.3")}
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 4.8))
    fig.patch.set_facecolor(SURFACE)
    for ax, key, ylabel in ((axes[0], "j1", "j1 shoulder tilt RMS [mrad]"),
                            (axes[1], "all", "all-joint RMS [mrad]")):
        ax.set_facecolor(SURFACE)
        ax.grid(True, axis="y", color=GRID, linewidth=0.7)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(MUTED)
        ax.tick_params(colors=MUTED, labelsize=8.5)
        conds = list(by)
        for x, c in enumerate(conds):
            vals = [float(r[key]) * 1e3 for r in by[c]]
            ax.scatter([x] * len(vals), vals, s=14, color=FAMILY[c],
                       alpha=0.55, linewidths=0)
            ax.hlines(statistics.median(vals), x - 0.28, x + 0.28,
                      color=FAMILY[c], linewidth=2.2)
        base_y = float(single["base"][key]) * 1e3
        ax.axhline(base_y, color=MUTED, linewidth=1.1, linestyle=(0, (4, 3)))
        ax.annotate("no error", (len(conds) - 0.55, base_y), color=MUTED,
                    fontsize=8, va="bottom", ha="right")
        for name, pair in refs.items():
            y = max(float(single[u][key]) for u in pair) * 1e3
            ax.axhline(y, color=MUTED, linewidth=0.8, linestyle=(0, (1, 2)))
            ax.annotate(f"{name} (worse endpoint)", (len(conds) - 0.55, y),
                        color=MUTED, fontsize=8, va="bottom", ha="right")
        ax.set_xticks(range(len(conds)))
        ax.set_xticklabels([LABELS[c] for c in conds], color=INK,
                           fontsize=8.5)
        ax.set_ylabel(ylabel, color=INK, fontsize=10)
        ax.set_ylim(bottom=0)
    fig.suptitle(
        "Computed-torque tracking vs randomized model error — CRANE-X7 "
        "roki sim (round trip, Kp 20, Kd 2)",
        color=INK, fontsize=11, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out / "study.png", dpi=140)
    print(f"wrote {out / 'study.png'}")


def trajectories_figure(out: pathlib.Path) -> None:
    """Reference vs actual (and error) for the correlated endpoints,
    rebuilt from the retained per-run trajectory CSVs."""
    series = [("U0.7", "mass ×0.7", "#2a78d6"),
              ("base", "true model", "#eb6834"),
              ("U1.3", "mass ×1.3", "#1baf7a")]
    data = {c: list(csv.DictReader(open(out / "runs" / f"{c}_s1.csv")))
            for c, _, _ in series}
    fig, axes = plt.subplots(2, 2, figsize=(11, 6.4), sharex=True)
    fig.patch.set_facecolor(SURFACE)
    for ax in axes.flat:
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.7)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(MUTED)
        ax.tick_params(colors=MUTED, labelsize=9)
    for col, (j, name) in enumerate(((1, "j1 shoulder tilt"),
                                     (3, "j3 elbow"))):
        top, bot = axes[0][col], axes[1][col]
        t = [float(r["t"]) for r in data["base"]]
        top.plot(t, [float(r[f"qd{j}"]) for r in data["base"]], color=MUTED,
                 linewidth=1.6, linestyle=(0, (4, 3)),
                 label="reference" if col == 0 else None)
        for cond, label, color in series:
            q = [float(r[f"q{j}"]) for r in data[cond]]
            top.plot(t, q, color=color, linewidth=1.8,
                     label=label if col == 0 else None)
            err = [(q[k] - float(data[cond][k][f"qd{j}"])) * 1e3
                   for k in range(len(t))]
            bot.plot(t, err, color=color, linewidth=1.8)
        bot.axhline(0.0, color=MUTED, linewidth=0.8)
        top.set_title(name, color=INK, fontsize=11, loc="left")
        bot.set_xlabel("t [s]", color=INK, fontsize=10)
    axes[0][0].set_ylabel("q [rad]", color=INK, fontsize=10)
    axes[1][0].set_ylabel("tracking error [mrad]", color=INK, fontsize=10)
    fig.legend(loc="upper right", bbox_to_anchor=(0.99, 0.97),
               frameon=False, fontsize=9, labelcolor=INK)
    fig.suptitle(
        "Correlated mass-error endpoints — reference vs actual, per joint",
        color=INK, fontsize=11, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out / "trajectories.png", dpi=140)
    print(f"wrote {out / 'trajectories.png'}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary",
                        default=REPO / "build/examples/x7_ct_mass_error",
                        type=pathlib.Path)
    parser.add_argument("--out", default=REPO / "build/ct_mass_error_study",
                        type=pathlib.Path)
    parser.add_argument("--seeds", default=20, type=int)
    parser.add_argument("--replot", action="store_true",
                        help="skip the sweep; re-render from results.csv")
    args = parser.parse_args()
    if not args.replot:
        if not args.binary.exists():
            sys.exit(f"binary not found: {args.binary} (build first)")
        args.out.mkdir(parents=True, exist_ok=True)
        sweep(args.binary, args.out, args.seeds)
        provenance(args.binary, args.out, args.seeds)
    report(args.out)
    trajectories_figure(args.out)


if __name__ == "__main__":
    main()
