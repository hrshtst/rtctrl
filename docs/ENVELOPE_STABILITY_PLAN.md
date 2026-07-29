# Offline exact-feedback-linearization study

> This is a small, offline-only study answering one question: does exact
> feedback linearization offer a useful advantage over rtctrl's current
> inverse-dynamics feedforward controller? It does not authorize hardware
> operation, change `x7_track`, reopen pass-2 identification, or alter the
> final 0.6 excursion-scale cap.

*Status: PROPOSED 2026-07-28. No study milestone has been executed.*

## Question and scope

The current controller evaluates inverse dynamics on the desired trajectory,

```math
\tau =
\operatorname{ID}(q_d,\dot q_d,\ddot q_d)
+ \tau_{\mathrm{feedback}},
```

where the shipped feedback includes host-derived velocity, per-joint gain
scales, a broad low-pass, and a clamped integrator. The experimental exact
feedback-linearization (EFL) controller instead evaluates the rigid model at
the measured state:

```math
v =
\ddot q_d + K_d'(\dot q_d-\hat{\dot q}) + K_p'(q_d-q),
```

```math
\tau_{\mathrm{EFL}} = \operatorname{ID}(q,\hat{\dot q},v).
```

One existing RNEA call therefore supplies
`M(q)v + C(q,qdot_hat)qdot_hat + g(q)`; no explicit mass-matrix inversion is
needed.

The study asks:

1. Does EFL produce the expected posture-independent error dynamics on a
   rigid plant?
2. Does any advantage survive model mismatch, host-loop delay, encoder
   quantization, and constant disturbance torque?
3. Does EFL worsen the 4–5 Hz or 13 Hz non-collocated flexible modes?

The study does **not** design a notch/lead compensator, choose new hardware
gains, or claim hardware readiness.

## Compared controllers

- **PRACTICAL baseline:** the unmodified shipped `arm::ComputedTorque`
  configuration from `crane_x7_tuning.hpp`.
- **DESIRED-host control:** a matched acceleration-domain controller,
  using the same host velocity estimate and gains as EFL-host but evaluating
  the model on the desired state:

  ```math
  \tau_{\mathrm{DESIRED}} =
  \operatorname{ID}\!\left(
    q_d,\dot q_d,
    \ddot q_d + K_d'(\dot q_d-\hat{\dot q}) + K_p'(q_d-q)
  \right).
  ```

  Comparing EFL-host with DESIRED-host isolates measured-state versus
  desired-state model evaluation without changing gain units, velocity
  source, integral policy, or torque limiting.
- **EFL-host:** experimental EFL using the same host-side position-derived
  velocity estimator and the same controller/hardware torque limits. No
  integrator or new frequency compensator is added; the purpose is to isolate
  measured-state feedback linearization.
- **EFL-ideal diagnostic:** the same EFL law using exact simulated velocity.
  This is an upper bound, never a deployable candidate.

The existing unfiltered desired-state feedforward test remains the textbook
reference, but it is not another tuning candidate in the comparison matrix.

Experimental code stays outside the public API, for example in
`apps/exact_feedback_linearization.hpp`, and is consumed only by a simulation
study executable and tests. `x7_track`, its CLI, and its defaults remain
untouched. The experimental estimator must match `ComputedTorque`'s existing
host estimator on a deterministic position/timestamp trace; this parity is
unit-tested rather than assumed.

## Gain selection without test-set tuning

EFL gains have acceleration-domain units:

```math
K_p' = \omega_n^2,\qquad K_d' = 2\zeta\omega_n.
```

Use one small preregistered grid,
`omega_n = {4, 6, 8} rad/s` and `zeta = {0.7, 1.0}`, on one development
scenario only: the existing rigid-sim start pose, scale 0.5, with no delay or
disturbance. Apply every gain pair to both DESIRED-host and EFL-host. For
EFL-host, discard candidates that:

- do not complete or produce a non-finite state;
- enter torque saturation; or
- exceed the practical baseline's peak error.

Among the remaining EFL-host candidates within 5% of the lowest RMS, select
the lowest `omega_n` (then the higher `zeta` on a tie). Freeze that pair before
running the held-out cases. Do not retune per posture, plant, disturbance, or
flexible mode. Report the full grid, not only the selected point.

## Bounded offline cases

This is deliberately not a Cartesian product. Use the following fixed cases
and no others:

| ID | plant and start | loop | reference / disturbance |
|---|---|---|---|
| R1 | ideal `SimArm`, checked-in P1 | synchronous, exact position | scale-0.5 round trip, none |
| R2 | ideal `SimArm`, incident start vector below | synchronous, exact position | scale-0.5 round trip, none |
| L1 | `SimArm`, P1 | two-cycle delay and encoder quantization | scale-0.5 round trip, none |
| L2 | `SimArm`, incident start | two-cycle delay and encoder quantization | scale-0.5 round trip, none |
| D+ | `SimArm`, P1 | two-cycle delay and encoder quantization | scale-0.5 round trip, +0.8 Nm constant on canonical joint 1 |
| D− | `SimArm`, P1 | two-cycle delay and encoder quantization | scale-0.5 round trip, −0.8 Nm constant on canonical joint 1 |
| F4 | gravity-free `TwoMassArm`, zero pose, 4.5 Hz mode on joint 0 | two-cycle delay | zero reference, fixed small gear deflection |
| F13 | gravity-free `TwoMassArm`, zero pose, 13 Hz mode on joint 5 | two-cycle delay | zero reference, fixed small gear deflection |

The preliminary incident start vector recorded during the investigation is:

```text
[-0.10738, 0.50928, 0.19021, -1.87299,
  0.02608, -0.79153, -0.05983, -0.01841]
```

Run PRACTICAL, DESIRED-host, and EFL-host once in every case. Run EFL-ideal
only in R1, R2, L1, and L2 to show the estimator ceiling with and without
delay. Together with the six-point development grid, this is the complete
study.

For R1–D− use the identical round-trip implementation, duration rule, torque
limits, and scoring code for every controller. These cases ask whether EFL
reduces posture dependence and whether any benefit survives the measured
host-loop effects.

For F4/F13, because `TwoMassArm` is synchronous and gravity-free by contract:

- wrap it in the same two-cycle delay model used in L1–D−;
- use zero-pose, gravity-free controller scenarios for every compared
  controller: subtract `gravityTorque(q_d)` from desired-state ID and
  `gravityTorque(q)` from measured-state ID, so the gravity-free fixture is
  never driven by fictitious model gravity; and
- seed the same small motor-side deflection for each controller.

Do not infer hardware stability from a pass. This screen only rejects a
controller that increases modal growth in a known non-collocated fixture.

## Metrics and decision rule

Record per run:

- total and per-joint RMS/max tracking error;
- RMS/peak commanded torque and saturation counts;
- settling time and steady-state error under constant disturbance;
- 4–5 Hz and 13 Hz position-spectrum amplitude; and
- fitted modal growth/decay rate over a fixed, preregistered window.

All results include controller name, gains, velocity source, delay, plant
parameters, start posture, scale, and disturbance vector in a machine-readable
CSV or JSON record.

First attribute any benefit:

- EFL-host versus DESIRED-host measures the effect of measured-state model
  evaluation.
- EFL-host versus PRACTICAL measures relevance to the shipped controller.
- If DESIRED-host improves but EFL-host does not improve further, do not
  attribute the result to exact feedback linearization.

Call EFL **promising for further offline work** only if EFL-host:

1. improves held-out RMS by at least 20% in a majority of rigid/disturbed
   cases;
2. increases neither peak error nor peak torque by more than 10%, and
   introduces no new saturation;
3. remains finite and completes every delayed case; and
4. has a negative fitted modal rate in both flexible-mode screens, with a
   point estimate no greater than the practical baseline's (more negative is
   better).

Otherwise record the result as negative or inconclusive. Even a positive
result does not authorize a hardware flag or default change; that would
require a separate safety-reviewed plan.

## Milestones

### EFL-0 — harness and mathematical checks

- Add the experimental controller and reusable delay wrapper.
- Unit-test zero-error inverse dynamics, the acceleration-domain feedback
  construction, first-sample velocity-estimator behavior, duplicate timestamps,
  estimator parity with `ComputedTorque`, gravity subtraction for the
  gravity-free fixture, and torque limiting.
- Pin the practical baseline's existing regression results before comparison.

**Exit:** unit tests pass; no production app or default changes.

### EFL-1 — preregistered simulation run

- Run the gain grid on the single development scenario and freeze the selected
  gains.
- Execute the complete held-out matrix without tuning changes.
- Store the invocation, build revision, and machine-readable result table.

**Exit:** every planned cell has a result or an explicit, reproducible failure.

### EFL-2 — short report

- Add an offline-results section to
  `docs/theory/computed-torque.md`, including negative results.
- State separately what was learned in the rigid, delayed/disturbed, and
  flexible fixtures.
- Decide only whether further **offline** controller work is justified.

**Exit:** `ctest --test-dir build --output-on-failure` and
`uv run mkdocs build --strict` pass.

## Stop conditions and non-goals

Stop the study without expanding scope if EFL-host:

- loses its benefit when exact velocity is replaced by the host estimate;
- increases flexible-mode growth or torque saturation;
- requires posture-specific retuning; or
- needs a notch, lead network, friction model, or new observer to become
  competitive.

Those outcomes may motivate a separate proposal, but they are not added to
this study. There are no hardware runs, no archive campaign, no safety-gate
changes, no exact-controller option in `x7_track`, and no change to the
supported excursion cap.
