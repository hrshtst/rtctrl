# Computed-torque mass-error study (sim)

*A descriptive simulation study, 2026-08-04. No controller, protocol,
or gate was changed. Tooling: `examples/x7_ct_mass_error` and
`tools/ct_mass_error_study.py`; figures and tables below are the
outputs of the committed study runner (provenance in each run's
`meta.json`: git commit + dirty flag, model and binary SHA-256,
seed list).*

## Background

The hardware disposition of computed torque is settled and is NOT what
this study examines: the 4–5 Hz full-amplitude oscillation was a
sampled-feedback phase problem (~117° of D-path lag at the mode — see
[computed torque](theory/computed-torque.md) and the remediation
records), the identification campaign that would have enabled a
redesign closed with a null result, and the 0.6 excursion-scale cap is
final. What remained unmeasured is the *other* axis of robustness: how
sensitive the inverse-dynamics feedforward is to wrong inertial
parameters — the axis people usually blame first, and the one the
incident record shows was NOT the hardware killer.

The study answers that question in the ideal rigid simulation, where
the phase axis is absent by construction: plant and controller run the
same cycle with no bus pipeline, no filters (PD low-pass off), and no
gear elasticity. Method and error model follow mi-lib-tutorial
roki008's `add_mp_error`, adapted as described below.

## Method

**Plant vs controller split.** The plant (`SimArm`) integrates the
true `crane_x7.ztk` model (plus its plant-only terms: reflected motor
inertia 0.05 kg·m², numeric damping, finger coupling). The
controller (`arm::ComputedTorque`, Kp 20, Kd 2, PD filter off,
integrator off unless stated) computes its feedforward on a *separate*
`ChainModel` copy, corrupted in one of two ways:

- **Correlated (uniform) scaling** — `ChainModel::scaleMassProperties`:
  every link's mass and rotational inertia × one factor; COMs and
  geometry untouched. Newton–Euler torques are linear in the inertial
  parameters, so the whole feedforward scales by exactly that factor
  (asserted as a unit-test identity).
- **Randomized per-link perturbation** —
  `ChainModel::perturbMassProperties(e_m, e_c, seed)`: each
  non-massless link draws an independent mass factor
  1 + U(−e_m, +e_m) (inertia density-scaled by the same factor) and a
  COM offset uniform in the ±e_c cube, with the tensor transferred to
  the displaced COM by the parallel-axis term. Draws come from a
  seeded `std::mt19937_64` mapped to doubles directly, so one seed
  reproduces bit-identically (runs are byte-identical CSVs).

**Task.** A joint-space minimum-jerk round trip between the M8
acceptance postures (velocity limit 1.0 rad/s, 2 s minimum legs,
0.5 s hold; ~4.5 s total). Note this is a *faster* trajectory than
the integration test's one-way 3 s minimum, so RMS values are not
comparable between the two.

**Conditions.** Mass-only ±10/20/30 %, COM-only ±5/10/20 mm,
combined (±20 % + ±10 mm) without and with the shipped integrator
(Ki 6, clamp 1.5 Nm); 20 seeds per condition, plus the four
correlated endpoints ×0.7/×0.8/×1.2/×1.3 and the two baselines —
166 runs. Reproduce with:

```
uv run --project tools tools/ct_mass_error_study.py
```

## Evaluation

### Correlated endpoints

Per-joint tracking RMS (mrad) under uniform scaling; j1 (shoulder
tilt) is gravity-dominated, j3 (elbow) acceleration-dominated:

| scale | j1 | j3 | all-joint |
|---|---|---|---|
| ×0.7 | 10.6 | 23.6 | 9.2 |
| ×0.8 | 7.2 | 23.0 | 8.6 |
| 1.0 (true) | 0.7 | 22.6 | 8.0 |
| ×1.2 | 6.8 | 23.1 | 8.5 |
| ×1.3 | 10.3 | 23.6 | 9.2 |

![Correlated endpoints: reference vs actual and tracking error for j1 and j3](img/ct_mass_error_trajectories.png)

j1's error splits symmetrically with the scale sign and its magnitude
matches the static-offset arithmetic Δg/Kp (≈0.3–0.5 Nm of gravity
error over Kp = 20 → the observed ~16 mrad peak). j3's error barely
moves across the whole sweep — its ~23 mrad, acceleration-correlated
component is *insensitive to every inertial perturbation tested*. That
insensitivity is the measured fact; attributing the component to the
plant's unmodeled reflected motor inertia is a plausible hypothesis
that has NOT been isolated (no zero-inertia or matched-controller
ablation has been run — same scoping as the EFL study's).

### Randomized perturbations (20 seeds per condition)

j1 and all-joint RMS, median [min–max] in mrad:

| condition | j1 | all-joint |
|---|---|---|
| no error | 0.7 | 8.0 |
| mass ±10 % | 1.0 [0.7–2.0] | 8.0 [8.0–8.0] |
| mass ±20 % | 1.6 [0.7–3.9] | 8.1 [8.0–8.1] |
| mass ±30 % | 2.2 [0.7–5.9] | 8.1 [8.0–8.3] |
| COM ±5 mm | 1.1 [0.7–1.6] | 8.0 [8.0–8.0] |
| COM ±10 mm | 1.7 [0.8–3.0] | 8.0 [8.0–8.1] |
| COM ±20 mm | 3.1 [0.9–5.8] | 8.1 [8.0–8.4] |
| both 20 % + 10 mm | 2.5 [0.8–5.6] | 8.1 [8.0–8.3] |
| both + integrator | 1.6 [0.7–3.5] | 8.0 [8.0–8.1] |

![Randomized model error: per-seed RMS by condition with correlated references](img/ct_mass_error_mc.png)

### Findings

1. **Every sampled randomized case was milder than the sampled
   correlated references** (worse endpoints: ×0.8 → 7.2 mrad,
   ×0.7 → 10.6 mrad on j1). Independent link errors partially cancel
   in the summed torque where the uniform scale cannot. The
   correlated runs are *references*, not a proven worst-case
   envelope; nothing here establishes a global extremum.
2. **No numerical failure or gross divergence occurred in any of the
   166 runs, and every aggregate RMS stayed within 8.0–8.4 mrad** —
   within 5 % of the no-error baseline. This is a finite-horizon
   (~4.5 s) observation; it makes no asymptotic stability claim.
3. **COM error is the stronger class, millimeter for percent**:
   ±10 mm ≈ ±20 % mass in median j1 effect, and ±20 mm posts the
   worst randomized medians.
4. **The shipped integrator improved j1 in 19/20 paired seeds** of
   the combined condition (median 2.5 → 1.6 mrad). Why it helps
   despite COM error reshaping the gravity field (rather than only
   biasing it) is untested — plausibly the reshaped field is nearly
   constant over this trajectory's configuration span, which a wider
   excursion or torque-residual analysis would probe.

## Limitations

The rigid, lag-free simulation excludes precisely the mechanisms that
parked computed torque on hardware (bus pipeline, filter phase, gear
elasticity), so this study bounds model-error sensitivity only —
per the testing ladder, a sim pass is necessary, never sufficient.
One trajectory, 20 seeds per condition, synthetic error classes
(density-style scaling; independent uniform draws). The all-joint
aggregate is dominated by j3's invariant component, which compresses
between-condition differences in that column.
