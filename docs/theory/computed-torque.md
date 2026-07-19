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
$e = q_d - q$:

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
  Note the sim adds reflected rotor inertia that the ID model does not
  include, so even in simulation the feedback carries a real residual.
- **Hardware.** `apps/x7_track` runs the same controller and gains on
  the robot: a reduced-speed (0.3 rad/s) excursion on tilt/elbow/wrist
  and back, printing per-leg RMS.

## Limitations and outlook

- **Unmodeled effects** — joint friction, reflected rotor inertia,
  current-loop bandwidth — land in $\delta(t)$ and set the tracking
  floor. Friction identification (M7 follow-up) can move friction from
  $\delta$ into the feedforward.
- The feedback is diagonal PD; model-based designs (operational-space
  control, impedance control) can reuse the same
  `ChainModel::inverseDynamics` building block unchanged — that is
  precisely the extension surface the bridge was built for.
