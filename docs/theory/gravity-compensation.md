# Gravity compensation

*Implemented in `arm::GravityComp`
(`include/rtctrl/arm/gravity_comp.hpp`) on top of
`model::ChainModel::gravityTorque`. Prerequisite reading:
[dynamics foundations](dynamics-foundations.md).*

## Control law

Gravity compensation applies, at every control cycle, exactly the
torque that gravity exerts on the arm in its **current** configuration:

```math
\tau = g(q).
```

Substituting into the equations of motion,

```math
M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + g(q) = g(q)
\quad\Longrightarrow\quad
M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} = 0 .
```

Gravity vanishes from the closed loop: **every** configuration with
$\dot q = 0$ is an equilibrium. The arm neither falls nor moves on its
own, yet offers no resistance to external pushes beyond its own
inertia: in the ideal model it *floats* and is freely back-drivable
(the hardware reality is bounded by friction and the waived back-drive
criterion below). This is the
foundation for hand-guiding, teaching, and as the $g(q)$ term inside
more advanced torque controllers.

## Stability

The closed loop conserves kinetic energy
$T = \tfrac12 \dot q^\mathsf{T} M(q) \dot q$ exactly. Differentiating
$T$ produces two terms. The $\tfrac12\dot q^\mathsf{T}\dot M\dot q$
term must not be dropped; it is what the skew-symmetry of
$\dot M - 2C$ cancels against:

```math
\dot T
= \dot q^\mathsf{T} M \ddot q + \tfrac12\,\dot q^\mathsf{T}\dot M\dot q
= \dot q^\mathsf{T}\!\left(\tau - C\dot q - g\right)
  + \tfrac12\,\dot q^\mathsf{T}\dot M\dot q
= \dot q^\mathsf{T}(\tau - g)
  + \tfrac12\,\dot q^\mathsf{T}\big(\dot M - 2C\big)\dot q
= \dot q^\mathsf{T}(\tau - g),
```

which vanishes along the compensated dynamics $\tau = g(q)$.

With any physical dissipation $D\dot q$ (joint friction, motor
damping; always present on the real robot, and represented by the
inertia-scaled viscous term in the simulator), $\dot T = -\dot
q^\mathsf{T} D \dot q \le 0$: motion decays and the arm settles
wherever it was left. Friction therefore *helps* stability while
slightly *hurting* back-drivability (the hand feels the uncompensated
friction, not gravity).

## Computing $g(q)$

Evaluating inverse dynamics at zero velocity and acceleration isolates
the gravity term:

```math
\mathrm{ID}(q, 0, 0) = M(q)\cdot 0 + C(q,0)\cdot 0 + g(q) = g(q).
```

`ChainModel::gravityTorque(map, q8, tau8)` does precisely this, with
the canonical↔model bookkeeping made explicit:

```math
\tau_8 \;=\; E^\mathsf{T}\, \mathrm{ID}\!\left(E q_8,\; 0_9,\; 0_9\right),
```

1. expand the canonical configuration, $q_9 = E q_8$ (finger B mimics
   finger A);
2. call `rkChainID_G` with **properly sized, member-owned zero
   vectors** for $\dot q_9$ and $\ddot q_9$ (roki dereferences both
   unconditionally: passing null is a crash, see the mi-lib notes);
3. reduce the nine generalized torques through the constraint
   transpose, $\tau_8 = E^\mathsf{T}\tau_9$, so the gripper receives
   the sum of both finger torques.

On the actuator side the bridge converts to current commands,
$i_k = \tau_k / k_{t,k}$, and the servos run in current mode
(operating mode 0).

## Verification

- **Gradient identity (unit test).** For a random gravity-loaded pose,
    each component of the computed $\tau_8$ is compared against a
    central finite difference of the potential energy,

    ```math
    \tau_i \overset{!}{=}
    \frac{U(q + \varepsilon e_i) - U(q - \varepsilon e_i)}{2\varepsilon},
    ```

    with $U$ evaluated independently from forward kinematics
    (link masses × COM heights). Agreement to $10^{-5}$ validates the
    RNEA path *and* the $E/E^\mathsf{T}$ mapping in one shot
    (`tests/unit/gravity_test.cpp`).

- **Structural checks.** At the upright zero pose, joints with
  vertical axes (pan/twist chain) must see exactly zero gravity
  torque.

- **Float acceptance (sim, through the bridge).** Under
  `GravityComp` in current mode the simulated arm holds a strongly
  gravity-loaded pose within 0.05 rad over 10 s
  (`tests/integration/gravity_sim_test.cpp`).

- **Hardware.** `apps/x7_float` runs the same controller on the robot
  and prints the current-derived torque estimate vs. the prediction
  ($k_t i_{\text{meas}}$ vs. $g(q)$) each second. In the 2026-07 M7
  acceptance sessions the arm floated and was back-drivable, with
  static agreement within a few hundredths of a Nm on all joints.
  Read that agreement for what it is: both sides share the same
  nominal $k_t$ (commands divide by it, the estimate multiplies by
  it), so it verifies the servo's *current loop* tracking the
  commanded current, not output-shaft torque; friction, gearbox
  efficiency, and any $k_t$ scale error sit outside the comparison.
  **Status: `x7_float` WORKS on hardware (un-parked 2026-07-31).**
  The 2026-07-29 incident, in which a float session accelerated the
  untouched arm toward the upright (peak ~2.37 rad/s), showed the
  signature of gravity over-compensation (a failure class the
  agreement print is structurally blind to) and led to the
  torque-constant calibration arc: the vendor-equivalent scales (hypotheses, not measurements)
  plus a position-held/preloaded startup passed the objective M-GC3
  acceptance with wide margins; the subjective back-drive criterion
  FAILED and the owner explicitly WAIVED it (a risk/quality decision,
  not a test pass; the characterized j1 notch and j4 tendency
  remain). Every `x7_float` session therefore refuses the bus without
  `config/crane_x7_vendor_scale.toml` and a fresh exclusive `--log`.
  Full arc and decisions:
  [history.md](../records/history.md#gravity-compensation-and-torque-constant-calibration).

## Limitations

- **Friction is uncompensated.** The float feels friction (a feature
  for stability, a bug for transparency). Compensating it requires the
  identification planned as a follow-up to M7.
- **Payload changes the model.** Grasping an object shifts $g(q)$; a
  payload estimate must be added to the model (a rigidly held mass can
  simply be attached to the gripper link of the ztk model).
- **Current tracking is assumed ideal.** The servo's current loop has
  finite bandwidth; at the slow motions typical for hand-guiding this
  is negligible.
