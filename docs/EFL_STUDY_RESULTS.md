# Offline EFL study — results record

> **Closed 2026-07-29 with a NEGATIVE result at the preregistered gain
> gate.** The study was offline-only by charter: no hardware was
> operated, no hardware flag follows, `x7_track` and the FINAL 0.6
> excursion-scale cap are unchanged.

This is the results record of the preregistered offline
exact-feedback-linearization (EFL) study. The study definition
([ENVELOPE_STABILITY_PLAN.md](ENVELOPE_STABILITY_PLAN.md)) is the
authority on the question and the rules; the frozen constants and the
implementation mapping are in
[EFL_STUDY_IMPLEMENTATION_PLAN.md](EFL_STUDY_IMPLEMENTATION_PLAN.md);
the interpretive report lives in the theory chapter
([theory/computed-torque.md](theory/computed-torque.md), "Offline
exact-feedback-linearization study"). The canonical machine-readable
evidence is `data/efl_study/results.json` (schema 2).

## Question

Does exact feedback linearization — the rigid model evaluated at the
**measured** state, feedback in the acceleration domain
($v = \ddot q_d + K_d'(\dot q_d - \hat{\dot q}) + K_p'(q_d - q)$,
$\tau = \mathrm{ID}(q, \hat{\dot q}, v)$) — offer a useful advantage
over the shipped practical controller (desired-state feedforward,
filtered torque-space PD, clamped integrator)? Three sub-questions:
posture-independent error dynamics on a rigid plant; survival under
host-loop delay, encoder quantization, and constant disturbance; and
behavior at the two non-collocated flexible-mode bands.

## Headline result

**The preregistered gain selection produced no surviving EFL-host
candidate, terminating the study at its first gate.** On the frozen
development scenario (rigid `SimArm`, pose
`[0, 0.2, 0, −0.4, 0, −0.2, 0, 0.1]`, scale-0.5 round trip,
synchronous exact-position loop):

| controller | ω_n [rad/s] | ζ | RMS [rad] | peak error [rad] |
|---|---|---|---|---|
| PRACTICAL (baseline) | — | — | 0.0092 | **0.0426** |
| EFL-host | 4 | 0.7 | 0.0445 | 0.1402 |
| EFL-host | 4 | 1.0 | 0.0444 | 0.1388 |
| EFL-host | 6 | 0.7 | 0.0439 | 0.1412 |
| EFL-host | 6 | 1.0 | 0.0437 | 0.1422 |
| EFL-host | 8 | 0.7 | 0.0430 | 0.1484 |
| EFL-host | 8 | 1.0 | 0.0426 | 0.1486 |
| DESIRED-host | 4 | 0.7 | 0.1913 | 1.3222 |
| DESIRED-host | 4 | 1.0 | 0.1565 | 0.8868 |
| DESIRED-host | 6 | 0.7 | 0.1183 | 0.4186 |
| DESIRED-host | 6 | 1.0 | 0.1164 | 0.4119 |
| DESIRED-host | 8 | 0.7 | 0.1031 | 0.3669 |
| DESIRED-host | 8 | 1.0 | 0.1017 | 0.3718 |

Every EFL-host pair completed without saturation but exceeded the
baseline's peak error — the preregistered discard rule eliminated the
whole grid. Per preregistration, the held-out acceleration-domain
cells record explicit not-run failures, and the C1–C4 decision is
null with a note. All 28 cells are present.

## Attribution

The comparison design attributes the failure cleanly:

- **Measured-state evaluation genuinely helps.** At identical gains,
  EFL-host beats DESIRED-host by 3–10× in peak error — evaluating the
  model at the measured state is the better half of the idea.
- **The advantage does not survive the unmodeled reflected rotor
  inertia.** Both acceleration-domain forms emit
  $\tau = M_{\text{model}}(q)\,v$; the plant adds ~0.05 kg·m² of
  reflected rotor inertia per joint that the ID model omits. At the
  wrist (link-side inertia ~5·10⁻⁴ kg·m²) the feedback's effective
  gain collapses by roughly that ~100× ratio — the errors concentrate
  on the moving joints, worst at the wrist, nearly independent of the
  gain choice. The practical law is immune: its torque-space gains
  were tuned against the true plant, and its integrator absorbs the
  residual.

## Baseline byproducts (recorded for the record)

- The two-cycle-delay, quantized loop from P1 carries a 0.376 rad
  transient peak (L1) that the incident-pose start does not
  (L2: 0.029 rad).
- Under ±0.8 Nm constant disturbance on joint 1, the practical law
  reaches its first qualifying dwell at 1.89 s (D+) and 0.53 s (D−),
  settling to ~4 mrad steady error.
- Flexible screens (PRACTICAL-GF): the planted 4.5 Hz ζ 0.03 mode
  decays strongly (fitted modal rate −1.56 s⁻¹); the planted 13 Hz
  ζ 0.05 mode shows slight GROWTH (+0.10 s⁻¹) — a miniature of the
  delayed-D-path mechanism documented in the theory chapter.

## Decision and non-consequences

Further **offline** EFL work is not justified (preregistered stop
condition: competitiveness would require at least a
reflected-rotor-inertia model term, plus the M7 friction
feedforward — new modeling that belongs to a separate proposal). No
hardware flag, no `x7_track` change, no cap change. The
settle-phase pan instability observed on hardware 2026-07-28 remains
OPEN and unaddressed by this study; `x7_track` stays parked pending
its own reviewer-gated fix.

## Provenance and reproducibility

- Canonical table generated at commit `afff1ab` from a worktree clean
  including untracked files, complete 28-cell matrix, embedded
  build-SHA verified against `HEAD` at run time; schema 2 rows carry
  the numeric start poses, full plant parameters, and the frozen
  torque-limit vector.
- Independently verified (external reviewer and locally): a fresh
  full run reproduces the committed table byte-exactly after removing
  the invocation/git provenance fields.
- Implementation commits: `620b980` (plans adopted) → `a4f53f5` (lag
  model extracted) → `62b3e4d` (controllers, estimator parity,
  metrics) → `da1aa9e` (runner) → `1260f03`/`045c96e` (results home,
  first canonical table) → `3c29f8c` (report) → `973d206`/`afff1ab`
  (review fixes: first-dwell settling, evidence schema) → `57fa63a`
  (canonical regeneration, schema 2) → `c42807d` (settling numbers) →
  `5900da7` (single-source plant emission) → `14693f1` (`--zvs`
  motion output).
- Review history: four pre-implementation rounds on the
  implementation plan (frozen constants, executable verdict formulas,
  modal censoring calculus, single-mode fixtures), two
  post-implementation rounds (all findings fixed; final verification
  clean).

## What survives the closure

Reusable infrastructure, independent of the negative verdict:
`apps/lagged_arm.hpp` (configurable command-pipeline lag with pinned
schedules), the estimator-parity and practical-replica test harness,
`apps/study_metrics.hpp` (least-squares band amplitudes, the modal
censoring calculus, first-dwell settling), the clean-worktree
build-SHA provenance pattern (`cmake/GitRev.cmake`), the preregistered
runner with its canonicality gates, and `--zvs` motion output for
`rk_anim` inspection of any simulated run.
