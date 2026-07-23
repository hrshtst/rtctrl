# Computed-torque trajectory control

*Implemented in `arm::ComputedTorque`
(`include/rtctrl/arm/computed_torque.hpp`) on top of
`model::ChainModel::inverseDynamics`. Prerequisite reading:
[dynamics foundations](dynamics-foundations.md).*

## Goal

Track a smooth reference trajectory
$t \mapsto \big(q_d(t), \dot q_d(t), \ddot q_d(t)\big)$ with joint
torques, so that the arm's *dynamics* — not just its servo positions —
follow the plan. This is what position mode cannot give: there, each
servo's internal loop fights the arm's coupled inertia on its own.

## Control law

rtctrl implements the **inverse-dynamics feedforward** form of
computed torque: the full inverse dynamics evaluated **along the
desired trajectory**, plus PD feedback on the tracking error
$e = q_d - q$ (this textbook form is what the theory below analyzes;
the shipped controller hardens it — see
[what the hardware taught us](#what-the-hardware-taught-us)):

```math
\boxed{\;
\tau \;=\; \underbrace{\mathrm{ID}\big(q_d,\, \dot q_d,\, \ddot q_d\big)}_{\text{feedforward}}
\;+\; \underbrace{K_p\, e + K_d\, \dot e}_{\text{feedback}}
\;}
```

with $\mathrm{ID}(q,\dot q,\ddot q) = M(q)\ddot q + C(q,\dot q)\dot q + g(q)$
computed by roki's recursive Newton–Euler (`rkChainID_G`) through the
same canonical expansion/reduction as gravity compensation:

```math
\tau_8^{ff} = E^\mathsf{T}\, \mathrm{ID}\big(E q_{d},\, E \dot q_{d},\, E \ddot q_{d}\big).
```

### Why this works

If the model were exact and the arm exactly on the trajectory
($e=\dot e=0$), the feedforward alone would realize
$\ddot q = \ddot q_d$ — by the very definition of inverse dynamics.
The PD term only has to absorb what is left. Substituting the law into
the equations of motion and expanding around the trajectory gives the
error dynamics

```math
M(q)\,\ddot e + \big(C(q,\dot q) + K_d\big)\,\dot e + K_p\, e
  \;=\; \delta(t),
```

where $\delta(t)$ collects the model mismatch (parameter errors,
friction, actuator effects, and the difference between evaluating
$M, C, g$ at the desired rather than the actual state). For
$K_p, K_d > 0$ the homogeneous part is (locally) exponentially stable,
and the steady tracking error scales with $\delta$ instead of with the
full gravity/inertia load that a bare PD must carry.

### Relation to exact feedback linearization

The textbook computed-torque controller evaluates the model at the
**measured** state and places the feedback inside the inertia matrix,

```math
\tau = M(q)\big(\ddot q_d + K_d' \dot e + K_p' e\big) + C(q,\dot q)\dot q + g(q),
```

which cancels the dynamics exactly and yields the linear error equation
$\ddot e + K_d'\dot e + K_p' e = 0$. rtctrl deliberately uses the
feedforward variant instead:

- it evaluates the model on the **noise-free desired** states — no
  measured velocities inside the dynamics terms, which matters with
  the Dynamixels' quantized velocity feedback;
- the feedback gains live directly in torque space (Nm/rad, Nm·s/rad),
  making saturation against the effort limits transparent;
- it is one RNEA call per cycle, cheap at any rate.

The price is that the error dynamics are only approximately linear;
for the CRANE-X7's speeds this is far below other error sources.

## Reference trajectories

`model::MinJerkTrajectory` supplies $C^2$ quintic (minimum-jerk)
point-to-point references. With normalized time $s = t/T$:

```math
q_d(t) = q_0 + \Delta q\,\big(10 s^3 - 15 s^4 + 6 s^5\big),
```
```math
\dot q_d(t) = \frac{\Delta q}{T}\,\big(30 s^2 - 60 s^3 + 30 s^4\big),
\qquad
\ddot q_d(t) = \frac{\Delta q}{T^2}\,\big(60 s - 180 s^2 + 120 s^3\big),
```

with zero velocity **and** acceleration at both ends — so the
feedforward starts and ends at pure $g(q)$, handing over smoothly to a
static hold. The duration factory bounds the peak velocity
($\max|\dot q_d| = \tfrac{15}{8}\,|\Delta q|/T$) by the configured
joint limit.

## Safeguards in the implementation

- Commanded torques convert to currents and are clamped per joint to
  $\min\!\big(\tau_{\max}/k_t,\; i_{\text{servo limit}}\big) - i_{\text{margin}}$
  in the hardware write path.
- Position-limit gating zeroes any current command that drives a joint
  past its (margin-reduced) limit.
- Both loss-of-comms watchdog layers stay active; a deadman
  escalation aborts the run.

## Verification

- **Sim acceptance (through the bridge, exact plant).** Tracking a
  multi-joint minimum-jerk sweep on `SimArm` in current mode:
  RMS error $5.0\times10^{-3}$ rad with the feedforward, versus
  $1.59\times10^{-2}$ rad for the identical PD without it — asserted
  as both an absolute bound (< 0.02 rad) and a relative one
  (< 0.5 × PD-only) in `tests/integration/tracking_sim_test.cpp`.
  The test runs with the hardware countermeasures below switched off —
  it certifies the feedforward math and the coordinate mapping, not
  the hardened loop.
  Note the sim adds reflected rotor inertia that the ID model does not
  include, so even in simulation the feedback carries a real residual.
- **Hardware.** `apps/x7_track` runs the same controller on the robot:
  a reduced-speed (0.3 rad/s) excursion on tilt/elbow/wrist and back,
  printing per-leg and per-joint RMS. Accepted 2026-07-21 at
  RMS 0.019/0.022 rad over the two legs — with the hardware-hardened
  control law below, not the textbook one.

## What the hardware taught us

The boxed law above is **not stable on the real arm**. A nine-run
campaign (2026-07-21, eight runs logged as per-cycle CSVs and
diagnosed offline) reshaped the shipped controller into

```math
\tau \;=\; \mathrm{ID}\big(q_d, \dot q_d, \ddot q_d\big)
\;+\; F_{lp}\!\Big( s_i \,\big(K_p e + K_d\, \dot e_{\text{host}}\big) \Big)
\;+\; \operatorname{clamp}\!\Big(K_i \!\int\! e \, dt\Big),
```

where every non-textbook term answers a specific measured failure:

- **Host-side velocity** $\dot e_{\text{host}}$: the servo's
  PresentVelocity estimate lags ~50 ms with ~2× attenuation (measured
  by cross-correlating it against the position derivative in the run
  logs). A $K_d$ term fed with it stops damping and starts *driving*
  near the arm's resonant modes. Velocity is instead estimated from the
  fresh position feedback (backward difference + 20 ms low-pass).
- **PD low-pass** $F_{lp}$ (50 ms): the gear trains resonate at
  ~13 Hz — a *non-collocated* loop (output-shaft encoder, motor-side
  torque, elastic gearing between) whose phase, after the 100 Hz bus
  loop's ~2 cycles of delay, makes every feedback term pump that mode.
  The filter removes loop gain there; the smooth feedforward passes
  unfiltered. The filter pole also caps usable stiffness
  ($\sqrt{K_p/J}$ must stay below it): $K_p \approx 6$, not the 20 an
  ideal rigid sim happily tolerates.
- **Clamped integrator**: ~1 Nm of unidentified friction sags the arm
  by $\tau_f / K_p$, which the modest $K_p$ cannot hide. A slow
  integrator absorbs it at DC while adding nothing at the resonances;
  the clamp (1.5 Nm) bounds windup against the torque limits.
- **Per-joint scales** $s_i$: the distal joints carry a small fraction
  of the shoulder's link-side inertia and limit-cycle through their
  gear backlash at gains the proximal joints need. The forearm twist is
  the extreme case — the hand's mass sits nearly *on* its axis
  ($J \sim 10^{-3}\,\mathrm{kg\,m^2}$) — and takes the smallest scale.
- **Soft starts and settling**: the filter applies no PD on its first
  cycle (a controller constructed at a leg boundary otherwise steps the
  residual error straight into the torque), and tracking only starts
  after a gravity-comp-plus-damping settle phase reports the arm
  quiescent — current-mode torque-on begins in free fall, and pure
  gravity compensation never damps the resulting swing.
- **Position hygiene** (hardware layer): in current/velocity modes the
  servos report *multi-turn* position, so hand-repositioning a limp
  joint across the encoder boundary leaves a $\pm 2\pi$ offset that
  silently turns the soft-limit gates into one-way walls. Feedback
  wraps to the principal angle, and the app refuses to start with any
  joint parked inside its limit-margin band.

**The stability envelope.** Beyond excursion scale ≈ 0.6 the arm
extends into configurations whose first structural mode (~4–5 Hz,
shoulder gear compliance against the extended arm's inertia) is
*actively pumped* by this loop — it grew even after the trajectory
rates were held at proven levels, so it is a loop property, not input
excitation. The mechanism is phase, not gain: the mode sits ABOVE the
PD filter's 3.2 Hz corner (1/(2π·0.05 s)) — inside its cut region —
yet the D path accumulates roughly 117° of lag there (≈55° from the
50 ms PD low-pass, ≈30° from the 20 ms velocity filter, ≈32° from the
~2-cycle bus pipeline), so past 90° the nominal damping *injects*
energy into the mode. Broad low-passing bought 13 Hz protection at the
price of the 4–5 Hz phase margin; more of it makes this worse, not
better. `x7_track` caps its scale accordingly.

**Pass-1 remediation (instrumentation & hardening).** Following the
post-completion review, the loop was made observable and its timing
monitored and bounded — see [REMEDIATION_PLAN.md](../REMEDIATION_PLAN.md):
measured-time control with an explicit stale-feedback abort policy,
feedback and command sequencing with first-vs-latest application
records and receipt-matched latency verification, one continuous
round-trip controller (no turnaround state reset), a strict quiescence
gate, direction-aware anti-windup against the exact actuator limits,
and full-loop CSV telemetry. Pass 1 does NOT lift the scale cap; it
gives pass 2 (mode identification, then a notch/phase-compensated
D-path design) trustworthy data to work from.

## Limitations and outlook

- **Unmodeled effects** — joint friction, reflected rotor inertia,
  current-loop bandwidth — land in $\delta(t)$ and set the tracking
  floor (the smooth tilt/elbow error humps in the accepted runs are
  the integrator chasing kinetic friction). Friction identification
  (M7 follow-up) can move friction from $\delta$ into the feedforward.
- **Large fast motions**: lifting the scale cap needs identification
  of the ~4–5 Hz structural mode followed by a notch or
  phase-compensated redesign of the D path — or position-mode tracking
  (servo-internal kHz loops) for that regime, keeping current mode for
  what it does best: gravity compensation, hand-guiding, and
  moderate-envelope tracking. Input shaping can only reduce REFERENCE
  excitation; it cannot stabilize an internally unstable feedback
  loop, so it is a supplement to the redesign, never a substitute.
- The feedback is diagonal PID; model-based designs
  (operational-space control, impedance control) can reuse the same
  `ChainModel::inverseDynamics` building block unchanged — that is
  precisely the extension surface the bridge was built for.
- **Rigid-joint simulation cannot certify gains for an elastic-geared
  arm.** Every sim-approved gain set failed on hardware until the
  elastic-mode countermeasures existed; the sim twin
  (`x7_track_sim`) earns its keep by *replaying logged failures*
  (`--start`, `--disturb`, lag models), not by proving stability.
