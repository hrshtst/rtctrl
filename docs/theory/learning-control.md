# Position-based learning control

*A theory note for the position-based learning study. It models the
servo-side follower as a spring with lag, shows that a position offset
acts as a feedforward torque, and restates three classical learning
schemes in that position form; the hardware status of each is in the
[comparison](#comparison). Prerequisite reading:
[dynamics foundations](dynamics-foundations.md) and
[computed torque](computed-torque.md).*

## Scope

On the CRANE-X7 the host sends joint position goals at 100 Hz, and each
Dynamixel servo closes its own position loop. Two other routes are not
available at present: torque (current) control from the host (see
[the history record's computed-torque chapter](../records/history.md#computed-torque))
and host loop rates above 100 Hz (see
[control-rate qualification](../hardware/control-rate.md)).

The question of this note: can three classical learning schemes, all
formulated for a torque input, act through the position command, on top
of the servo-side follower `x7_follow` (see
[servo-side trajectory following](../guide/usage.md#servo-side-trajectory-following))?
The schemes are iterative learning control (Arimoto), feedback-error
learning (Kawato), and adaptive control (Slotine and Li).

## The servo as a spring with lag

Per joint and in canonical order, $q_d(t)$ is the reference, $u(t)$ the
position command sent to the servo, $q(t)$ the measured position, and
$e = q_d - q$ the tracking error.

!!! warning "Sign convention"
    This note uses $e = q_d - q$ throughout. The follower's CSV log
    stores the opposite sign: its `errorN_rad` columns hold
    $q - q_{\mathrm{ref}} = -e$. Arimoto's papers ($\Delta x = x - x_d$)
    and the Slotine and Li law ($\tilde q = q - q_d$) also use the
    opposite sign; every law below is rewritten for $e$.

Model each servo's position loop as a stiffness $K$ and a damping $D$
(diagonal, in Nm/rad and Nm·s/rad), and the arm by its rigid-body
dynamics with a friction term $f$:

```math
\tau = K\,(u - q) - D\,\dot q,
\qquad
M(q)\,\ddot q + C(q,\dot q)\,\dot q + g(q) + f(\dot q) = \tau .
```

The servo applies its command after a transport delay $\theta$ (bus
pipeline and command application), so with the plain follower, $u =
q_d$, it acts on $q_d(t-\theta) \approx q_d - \theta\,\dot q_d$. With a
small error, $\dot q \approx \dot q_d$, and the loop equation gives the
quasi-static relation

```math
e \;\approx\; K^{-1}\tau + T\,\dot q_d,
\qquad
T = K^{-1}D + \theta ,
```

where $\tau$ is the torque the motion requires along the reference: a
spring deflection under that torque, plus a lag $T$ that lumps the
servo's $K^{-1}D$ and the transport delay. It holds under three
assumptions and fails where they fail:

- **No saturation.** In position mode the servo's loop drives PWM, a
  voltage, so speed and torque saturate; past saturation $K$ is no longer
  a stiffness. (Current-based position mode drives current and saturates
  at its effort ceiling.) Below saturation the motor's back-EMF adds
  damping, which the model lumps into $D$.
- **No integral action.** The follower's servos run with position I gain
  0 in the configurations used so far (see the
  [gain decision record](../records/history.md#current-based-position-gain-decision-2026-08-27)).
  An integrator would drive the static error to zero and break the
  proportionality between error and torque.
- **Slow motion against the loop.** The delay and the servo's own
  dynamics must be small against the motion's time scale; otherwise the
  full error equation of the next section applies.

### Evidence

The numbers below come from `tools/follow_learnability.py`, with
`x7_model_torque` for the model terms, run on archived `x7_follow`
hardware logs (see
[learnability analysis of follow logs](../guide/usage.md#learnability-analysis-of-follow-logs)).
The tool reports $q - q_{\mathrm{ref}}$; everything here is stated in
this note's sign, $e = q_d - q$, with angles in mrad and canonical
0-based joint indices (servo models per joint:
[canonical coordinates](../guide/architecture.md#canonical-coordinates)).

**Provenance A** (fit and prediction): five position-mode runs of the
plain follower (no offset), each replaying a different hand-taught
reference of 11.5 to 14.8 s, recorded at 100 Hz on 2026-08-28 and
2026-09-01 (the 2026-08-28 campaign is indexed in the
[data archive manifest](../records/data-archive.md#teaching-playback-20260828-143834-manual-teaching-and-playback-2026-08-28)).
Aggregate tracking RMS per run: 19.65 to 31.40 mrad over all 8 joints,
21.01 to 33.56 mrad over the 7 arm joints.

**Fit of the quasi-static relation** (the tool's `anatomy`). Per joint
and run, the tool fits $e = K^{-1}\tau + T\dot q_d + c$ by least squares.
The torque is the log's estimate (motor current times the nominal torque
constant), so $K$ is in those units and inherits that constant's
uncertainty. Ranges over the runs and the listed joints:

| Quantity | Joints | Value |
|---|---|---|
| Stiffness $K$, XM430-W350 | 0, 2, 3 | 17.2 to 24.0 Nm/rad |
| Stiffness $K$, XM430-W350 | 5 | 13.8 to 23.1 Nm/rad |
| Stiffness $K$, XM430-W350 | 4, 6 | not identifiable in most runs (logged torque RMS below the tool's 0.02 Nm threshold) |
| Stiffness $K$, XM540-W270 | 1 | 58.7 to 63.5 Nm/rad in four runs, 96.7 in the fifth |
| Lag $T$ | 0 to 5 | 39 to 77 ms (joint 1, the slowest: 61 to 77 ms; XM430 joints: 39 to 66 ms) |
| Residual of the fit | 0 to 5 | 0.9 to 6.9 mrad RMS, against per-joint tracking RMS of 3 to 61 mrad |

The taught references do not start exactly at rest: for the reference
chosen for the learning demonstration, the first sample has a reference
speed of up to 0.09 rad/s and a reference acceleration of up to
10.5 rad/s² (the end is at rest).

**Provenance B** (repeatability, the tool's `repeat`): three runs of one
planned (not taught) reference in current-based position mode,
2026-08-27, the runs of the
[gain decision record](../records/history.md#current-based-position-gain-decision-2026-08-27)
(manifest:
[gain sweep](../records/data-archive.md#follow-cbp-gain-sweep-20260827-servo-side-gain-sweep-2026-08-27)).
There is no repeated position-mode run yet; measuring one is the first
hardware step. Over all 8 joints, with the tool's 3 Hz band split:

| Band | Repeatable (trial mean) | Non-repeatable |
|---|---|---|
| all frequencies | 18.06 mrad RMS (debiased) | 0.98 mrad RMS |
| below 3 Hz | 18.04 mrad | 0.46 mrad |
| above 3 Hz | 1.01 mrad | 0.86 mrad |

The band rows are the tool's raw trial-mean RMS (not debiased). The
initial state differed between the runs by up to 9.2 mrad (6 encoder
counts) on one joint. In that mode and for that reference only, this
bounds what ILC can remove (the repeatable part) and where its floor lies
(the non-repeatable part; see the formula under [limits](#limits)).

**Prediction from the reference alone** (the tool's `predict`, on the
provenance A runs). The regressors of the
[feedback-error learning table](#feedback-error-learning-in-position-form),
evaluated on the reference only, are fitted on four runs and tested on
the fifth, over the arm joints. With the full regressor set the held-out
residual is 7.4 to 21.0 mrad, from baselines of 21.0 to 33.6 mrad, and
the mean over held-out runs of baseline/residual is 2.47. With gravity
and reference velocity only it is 2.31; with reference velocity only,
2.22 (both reduced sets keep the constant). For the reference chosen for
the learning demonstration: 21.01 mrad baseline, 7.36 mrad held-out,
4.92 mrad when fitted on that run itself. Two of the five references stay
poorly predicted (15.6 and 21.0 mrad residual), driven by joint 0 (in
one of them 52.6 mrad baseline, 46.0 mrad predicted residual); the cause
is not established (saturation is suspected, not shown).

**Measured torque against the model terms** (pooled over the five runs):
$R^2$ between 0.03 and 0.59 per joint. Consequently the fitted weights of
the model terms are not physically interpretable; most of the
predictable error comes from the reference-velocity term (compare the
reduced-set ratios above); and friction rather than rigid-body dynamics
dominates the logged torque at these speeds (a reading of the low $R^2$,
not a separate measurement).

## Command offset as torque

With $u = q_d + \delta$ the loop equation becomes
$K e - D\dot q + K\delta = M\ddot q + C\dot q + g + f$: the offset acts
as a feedforward torque $K\delta$. Setting $e \equiv 0$ gives the ideal
offset, and linearising about the reference (dominant terms kept) gives
the error equation:

```math
\delta^* = K^{-1}\Big[\, M(q_d)\,\ddot q_d + C(q_d,\dot q_d)\,\dot q_d
  + g(q_d) + f(\dot q_d) + D\,\dot q_d \,\Big],
\qquad
M\,\ddot e + (C + D)\,\dot e + K\,e = K\,(\delta^* - \delta).
```

The servo's damping acts on the measured velocity; because $\delta^*$
carries $D\dot q_d$, the same damping acts on $\dot e$ in the error
equation. At low frequency the transport delay adds $\theta\dot q_d$ to
the ideal command, so quasi-statically the ideal offset equals the error
the plain follower shows, $K^{-1}\tau + T\dot q_d$.

All three schemes below are ways of learning $\delta$. Consequently
neither the torque constant nor the servo gain has to be known in
absolute terms: the error is measured and the offset sent in radians,
and $K$ only scales gains and weights.

## Iterative learning control in position form

### Arimoto's law as stated

Iterative learning control began with the "betterment" scheme of
Arimoto, Kawamura and Miyazaki (1984). The two papers used here
(Arimoto, Sekimoto and Kawamura 2007, 2008) state it in task space, with
torque input, in continuous time. With task
coordinates $x$, Jacobian $J$, trial index $n$ and $\Delta x = x - x_d$,
the joint torque is a task-space PD law plus a joint damping $C_A$
(written $C$ in the papers) plus a learned input $v_n$ (2007, eq. 2),
and $v_n$ is updated between trials from the velocity error alone
(2007, eq. 3):

```math
\tau = -C_A\dot q - J^\mathsf{T}(q)\big\{k\,\Delta x + \zeta_1\sqrt{k}\,\Delta\dot x - v_n\big\},
\qquad
v_n = v_{n-1} - \Phi_A\,\Delta\dot x_{n-1},\quad v_1 = 0 .
```

A variant adds a position-error term (2007, eq. 23),
$v_n = v_{n-1} - \{\Phi_A\Delta\dot x_{n-1} + \Psi_A\Delta x_{n-1}\}$,
which the authors expect to work efficiently when Coulomb friction is
large; neither paper analyses it. The 2007 paper proves convergence on
dynamics linearised about the desired motion, through passivity (its
appendix A). The 2008 paper restates the law as its eq. 6 and proves
convergence on the nonlinear dynamics: the pair (learned-input error,
velocity error) is passive and output-dissipative (2008, eqs. 47 and
48).

### Joint-space reduction

Take the task variable to be the joint variable ($J = I$), replace
$\Delta x$ by $-e$, and divide the learned torque by $K$
($\delta = K^{-1}v$). With the two additions that 100 Hz sampling
requires (below), the law between trials $k$ reads

```math
\delta_{k+1}(t) = Q\Big[\,\delta_k(t) + \Phi\,e_k(t+\Delta) + \Psi\,\dot e_k(t+\Delta)\,\Big],
\qquad \delta_0 = 0 ,
```

with dimensionless $\Phi = K^{-1}\Psi_A$ and $\Psi = K^{-1}\Phi_A$ in
seconds. Mind the letters: here $\Phi$ multiplies the position error and
$\Psi$ the velocity error, the reverse of the papers' lettering. With
$\Delta = 0$ and $Q = 1$ this is the papers' law, sign included.

On the follower the offset is a sequence $\delta[0], \dots, \delta[N-1]$ on
the tracking grid, and the follower adds two phases around tracking (see
[learning between follow trials](../guide/usage.md#learning-between-follow-trials)).
Before tracking, after homing, it ramps the command from the last homing
command to $q_d(0) + \delta[0]$ at a bounded rate and holds it there; after
tracking, it holds $q_d(T) + \delta[N-1]$. Both remove a command step: once
the offset carries a static compensation such as $K^{-1}g(q_d)$, $\delta[0]$
and $\delta[N-1]$ are not zero, and without the ramp the first tracking
command would jump by $\delta[0]$, while dropping the offset at the end would
release the compensation in one step. The final hold also supplies the error
record that the advance $\Delta$ reads past the end of the reference, measured
against $q_d(T)$ with the last offset sample still applied.

### Conditions the papers assume

- **Identical initial state** at every trial, on the trajectory and at
  rest (2007, section III; 2008, section 1 and Proposition 1). The
  repeated runs of the [evidence](#evidence) started up to 9.2 mrad
  apart on one joint.
- **A twice continuously differentiable reference** with zero velocity
  and acceleration at both ends (2007, section III; the 2008 analysis
  needs them at the start). Minimum-jerk references meet this. The
  taught reference chosen for the demonstration does not: it starts with
  a reference speed of up to 0.09 rad/s and an acceleration of up to
  10.5 rad/s² (it ends at rest), a known departure from the rest-to-rest
  assumption.
- **A short horizon:** the terminal time is "at most a few seconds"
  (2008, section 4).
- **Enough damping:** of the order of the square root of the inertia
  (2007, eq. 21), and large against bounds on the nonlinear terms (2008,
  eq. 47).
- **No gravity, or gravity compensated directly.** The 2008 paper shows
  that the ideal input exists with uncompensated gravity but leaves
  convergence open (2008, sections 6 and 7). In position mode $g(q_d)/K$
  is part of $\delta^*$, so the arm is in the open case.

The 2007 gain condition (appendix A) is $0 < \Phi_A < 2\zeta_1\sqrt{k}$,
twice the velocity-error feedback gain. The servo has no separate
error-velocity gain: its $D$ acts on the measured velocity, like the
papers' $C_A$, and damps $\dot e$ in the error equation because
$\delta^*$ carries $D\dot q_d$. With $J = I$ that joint-damping term is
the one the 2007 proof drops as nonnegative; read this way (an inference
of this note) the condition maps to

```math
0 < \Psi < 2\,K^{-1}D = 2\,(T - \theta) .
```

Part of the measured lag $T$ is transport delay, not damping, so the
bound is at most of order $2T$. It also comes from a delay-free
argument, and the 2008 inequality (48) keeps only half of the task-space
damping, so it is optimistic. The usable range of $\Psi$ has to be
established in simulation with the command lag included, not assumed.
The learn configuration of `x7_follow_learn` refuses $\Psi$ above 0.1 s
and $\Phi$ of 2 or more (the low-frequency condition of the next
section); these are ceilings, and the usable range inside them is still
to be established.

### What 100 Hz sampling adds

The papers treat continuous time without dead time. The position form
needs two elements that are standard in the general ILC literature
(Longman 2000; Bristow, Tharayil and Alleyne 2006):

- **A time advance $\Delta$.** The offset sent at $t$ shows in the error
  only after the dead time and the servo's lag, so the update pairs
  $\delta_k(t)$ with the error at $t + \Delta$; the measured lag $T$
  (39 to 77 ms on joints 0 to 5, see the [evidence](#evidence)) is the
  natural starting value. The trial log must extend $\Delta$ past
  the end of the reference, which the follower's final hold provides.
- **A zero-phase low-pass $Q$.** The update runs between trials on a
  finished record, so non-causal operations (the advance, a forward and
  backward filter) cost nothing. The implemented $Q$ is a second-order
  Butterworth low-pass, discretised by the bilinear transform and applied
  forward, then backward. Its design puts the combined response at -3 dB
  exactly at the cut-off $f_c$: the cut-off is prewarped, and the analog
  corner is placed a factor $(\sqrt2 - 1)^{-1/4}$ above it, so that with $h$
  the grid step
  ```math
  |Q(f)| = \frac{1}{1 + (\sqrt2 - 1)\,\big(\tan(\pi f h) / \tan(\pi f_c h)\big)^4},
  \qquad \arg Q(f) = 0 ,
  ```
  which is $1/\sqrt2$ at $f = f_c$; $f_c$ may be at most 0.8 of the Nyquist
  frequency. Each record is padded at both ends by odd reflection
  ($2x_0 - x_m$ before the start, likewise after the end) over
  $\lceil 1.5/(f_c h)\rceil$ samples (1.5 periods of the cut-off), or the
  record's length less one if that is shorter. Each pass starts from the
  steady state of its first sample, so a constant passes unchanged; the
  padding is removed afterwards.

Let $G$ be the response from offset to position (a damped spring with
the transport delay, $G(0) = 1$) and $L(s) = (\Phi + \Psi s)\,e^{s\Delta}$
the learning operator. Splitting the error into the repeatable error
without offset, $r$, and a non-repeatable part $n_k$, so that
$e_k = r - G\delta_k + n_k$, the update gives

```math
\delta_{k+1} = Q\,(1 - LG)\,\delta_k + QL\,(r + n_k).
```

The offset, and with it the error, contracts at the frequencies where
$|Q(1 - LG)| < 1$. Where $Q = 1$ this needs the phase of $LG$ within ±90
degrees, since $|1 - LG| < 1$ is impossible outside that band whatever
the gain; $Q$ must cut learning off above the frequency where the phase
leaves it. Without $Q$, wherever $|1 - LG|$ is close to one (above the
servo's bandwidth $G$ is small), $n_k$ is summed trial after trial and
accumulates in $\delta$. At low frequency $G \approx 1$, so contraction
needs $0 < \Phi < 2$, and $\Phi = 1$ removes the quasi-static error in
one trial (inferences of this note from the same condition).

**The update as implemented** (`rtctrl/learn/ilc_update.hpp`, called by
`x7_follow_learn ilc`). The trial log supplies the error $e_k$ on the $N$
tracking samples and on the final-hold samples after them. Per joint, with
$\Phi$ and $\Psi$ not both zero (a joint with both zero is passed through
unchanged), one update runs in this order:

1. low-pass the error: $\bar e = Q e_k$, over the tracking and hold samples
   together;
2. differentiate: $\dot{\bar e}$ by central differences, one-sided at the
   two ends;
3. advance: read $\bar e$ and $\dot{\bar e}$ at $t_i + \Delta$, linearly
   interpolated between samples (this needs $\lceil\Delta/h\rceil$ hold
   samples);
4. form the bracket
   $c_i = \delta_k[i] + \Phi\,\bar e(t_i + \Delta) + \Psi\,\dot{\bar e}(t_i + \Delta)$;
5. low-pass the bracket over the $N$ tracking samples: $d = Q c$;
6. clamp the change per trial: $d_i - \delta_k[i]$ to $\pm$
   `max_trial_step_rad`;
7. clamp the value to $\pm$ `max_offset_rad`, giving $\delta_{k+1}[i]$.

The previous offset passes $Q$ once and the error twice, before
differentiation and in the bracket; ignoring the record's edges and the
clamps, the update reads
$\delta_{k+1} = Q(1 - QLG)\,\delta_k + Q^2L\,(r + n_k)$ (an inference of
this note), the same as above where $Q \approx 1$. An input offset outside the
value clamp is refused, so step 7 only moves a value toward $\delta_k[i]$ and
the change never exceeds the step clamp. The result must then pass the
offset limits on value and rate (the learn configuration's, which can only
be lower than the follower's ceilings); one that does not is refused, never
clipped.

### Limits

- Only the repeatable part of the error can be removed. With $G = 1$, a
  scalar gain $\gamma$, and a non-repeatable part of variance $\sigma^2$
  independent between trials, a one-line calculation gives a converged
  error variance of $2\sigma^2/(2-\gamma)$: a small gain keeps the floor
  near the non-repeatable level, $\gamma = 1$ doubles its variance.
- The learned signal belongs to one trajectory and one start posture.

## Feedback-error learning in position form

### Kawato's scheme as stated

Kawato, Furukawa and Suzuki (1987) feed the manipulator an inverse-model
torque plus a feedback torque, $T = T_i + T_f$ (their eq. 5). The
inverse model is a weighted sum of fixed nonlinear subsystems evaluated
on the desired trajectory, linear in the weights (eq. 7); the subsystems
are the expanded terms of the Lagrangian dynamics (eq. 6, Table 2). With
$x(t)$ the subsystem outputs and $\tau_L$ a learning time constant, the
weights follow $\tau_L\,\dot w = x(t)\,s(t)$ (eq. 2), with the feedback
torque $T_f$ as the error signal $s$.

The paper's convergence theorem (Theorem 1, via the averaged equation,
eq. 4) covers the open-loop identifier: with a learning time constant
long against the signals' mixing time, the weights converge in mean to
the least-mean-square value. For the closed loop it gives no proof and
says the feedback torque is not guaranteed to vanish. Its simulations
report the weights diverging when a fast pattern was learned with weak
(proportional-only) feedback, and the discussion (section 5) names the
remedies: slower training movements, a longer learning time constant, or
a better teaching signal.

### Position form

The offset is a weighted sum of regressors evaluated on the reference,
$\delta_i(t) = \sum_l w_{il}\,\phi_{il}(q_d, \dot q_d, \ddot q_d)$ for
joint $i$, with $K_i^{-1}$ absorbed into the weights. The error signal is
the servo's own feedback effort in response to the tracking error, in
position units $e + T_s\dot e$, with $T_s$ playing the role of $K^{-1}D$.
The paper's appendix gives a delayed-input form of rule (2), there to
make the identifier a predictor; this note uses the same pairing because
the offset sent at $t - \Delta$ is what the error at $t$ responds to:

```math
\tau_L\,\dot w_i = \phi_i(t - \Delta)\,\big[\,e_i(t) + T_s\,\dot e_i(t)\,\big].
```

Candidate regressors for joint $i$, with the weight each would take
under the quasi-static relation (this note's reading; on the logs of the
[evidence](#evidence) the fitted weights of the model terms are not
physically interpretable, and most of the predictable error comes from
the reference-velocity term):

| Regressor $\phi_{il}$ | Absorbs | Ideal weight |
|---|---|---|
| model gravity torque $[g(q_d)]_i$ | gravity | $1/K_i$ |
| model inertial and Coriolis torque $[M(q_d)\ddot q_d + C(q_d,\dot q_d)\dot q_d]_i$ | rigid-body dynamics | $1/K_i$ |
| reference acceleration $\ddot q_{d,i}$ | rotor inertia (absent from the rigid model) | rotor inertia over $K_i$ |
| reference velocity $\dot q_{d,i}$ | viscous friction, servo lag | $T_i$ plus viscous coefficient over $K_i$ |
| smooth sign $\tanh(\dot q_{d,i}/v_0)$ | Coulomb friction | Coulomb level over $K_i$ |
| constant | static bias | |

**Project decision.** On hardware the weights are first updated between
trials from the logged trial, by the batch form of the same gradient
($t_f$ is the end of the reference):

```math
w_{i,k+1} = w_{i,k} + \frac{1}{\tau_L}\int_{\Delta}^{t_f+\Delta}
  \phi_i(t-\Delta)\,\big[\,e_{i,k}(t) + T_s\,\dot e_{i,k}(t)\,\big]\,dt .
```

For slow learning this is justified by the paper's averaged equation
(eq. 4): when $\tau_L$ is long against a trial, the weights barely move
within it. In-loop updating is a later step.

Against ILC, the weights transfer to other trajectories (the paper's
network, trained on one pattern, controlled a different and faster
movement). The price: the scheme can only remove what the regressors can
express (the paper's appendix makes the same point for the identifier).

The [evidence](#evidence) supports a matching expectation for this arm:
a trajectory-specific ILC can approach the non-repeatable floor, while a
frozen desired-trajectory feedforward removed about 60 % of the RMS
error on a held-out trajectory (mean baseline/residual ratio 2.47). That
figure is an offline prediction from logs recorded without offset, not a
closed-loop result.

### The two update steps as implemented

`rtctrl/learn/regressor_feedforward.hpp` (called by `x7_follow_learn fel`)
evaluates the regressors of the table on the reference, smoothed by a
zero-phase Gaussian and differentiated, at the $N$ tracking samples $t_n$.
The error signal of joint $i$ is the trial's error $\bar e_i$, smoothed by a
zero-phase Gaussian of its own width over the tracking and hold samples and
differentiated, then read at the advance:
$s_i(t_n) = \bar e_i(t_n + \Delta) + T_s\,\dot{\bar e}_i(t_n + \Delta)$, which
is the pairing of the batch rule above. A regressor column whose RMS over the
trial is below $10^{-6}$ is skipped and its weight kept (the gravity column of
joint 0, which turns about the vertical axis, is one). Per arm joint there are
two steps, each scaled by a gain $\eta \in (0, 1]$:

- **Normalised gradient**, the batch form of Kawato's rule with one learning
  rate per weight:
  ```math
  w_{il,k+1} = w_{il,k} + \eta\,
    \frac{\sum_n \phi_{il}(t_n)\,s_i(t_n)}{\sum_n \phi_{il}(t_n)^2},
  ```
  that is, $1/\tau_L = \eta \big/ \big(h\sum_n \phi_{il}(t_n)^2\big)$ for
  weight $l$, with $h$ the grid step. With $C$ the Gram matrix of the
  RMS-normalised columns,
  $C_{ab} = \overline{\phi_a\phi_b}\big/\sqrt{\overline{\phi_a^2}\;\overline{\phi_b^2}}$
  (bars are means over the trial), the update is refused when
  $\eta\,\lambda_{\max}(C) \ge 2$.
- **Least squares**: the step $\Delta w_i$ minimises
  $\sum_n \big(s_i(t_n) - \sum_l \phi_{il}(t_n)\,\Delta w_{il}\big)^2$ plus a
  ridge penalty $\rho N$ (with $\rho$ = `fel.ridge`) times the squared norm
  of the step in RMS-normalised units, and
  $w_{i,k+1} = w_{i,k} + \eta\,\Delta w_i$.

The library's unit tests established the following, on synthetic plants (the
error of a trial is exactly what the regressors express with the planted
weights, delayed by the advance), not on the arm:

- The normalised gradient reduces the remaining error quickly, at every update
  with the default smoothing and no lead, but it learns weakly excited weight
  directions (small eigenvalues of $C$) slowly: the RMS of the remaining error
  converges, not the weights.
- Least squares at $\eta = 1$ recovers the planted weights in one update,
  within the ridge's shrinkage, when the error enters unsmoothed.
- The guard $\eta\,\lambda_{\max} < 2$ is the convergence condition for an
  aligned, unit-gain plant whose error enters without smoothing or lead, where
  the normalised weight error iterates $u \leftarrow (I - \eta C)\,u$. It is
  not a proof for the real servo, nor for the smoothed or led error signal.
- With a derivative lead $T_s$ of 0.02 s (the value the tests examine), the
  normalised gradient still contracts the weight error at every update, but
  the remaining error is no longer guaranteed to fall at every update. The
  lead (`fel.error_lead_s`) therefore defaults to zero, and the error signal
  is then the smoothed, advanced position error alone.

## Slotine and Li adaptive control in position form

### The textbook law

The original paper (Slotine and Li 1987) was not available for this
note; the law is quoted in its standard textbook form (Slotine and Li
1991, chapter 9, section 9.2). With $\tilde q = q - q_d$, a gain
$\Lambda > 0$, $\dot q_r = \dot q_d - \Lambda\tilde q$,
$s = \dot q - \dot q_r = \dot{\tilde q} + \Lambda\tilde q$, and the
dynamics linear in a parameter vector $a$,
$M\ddot q_r + C\dot q_r + g = Y(q,\dot q,\dot q_r,\ddot q_r)\,a$:

```math
\tau = Y(q,\dot q,\dot q_r,\ddot q_r)\,\hat a - K_D\,s,
\qquad
\dot{\hat a} = -\Gamma\,Y^\mathsf{T} s .
```

With $\tilde a = \hat a - a$ and the skew-symmetry of $\dot M - 2C$,
$V = \tfrac12 s^\mathsf{T}Ms + \tfrac12\tilde a^\mathsf{T}\Gamma^{-1}\tilde a$
gives $\dot V = -s^\mathsf{T}K_D s$.

### Position form

The servo's PD term supplies the sliding-variable feedback: with
$\tilde q = -e$, $K e + D\dot e = -Ds$ for $\Lambda = D^{-1}K$ and
$K_D = D$. If the servo's derivative acts on the measured velocity only,
it supplies $Ke - D\dot q = -Ds - D\dot q_d$; the missing $D\dot q_d$ is
one more feedforward term, absorbed by the reference-velocity regressor.
The model term is sent as the offset $\delta = K^{-1}Y\hat a$.

**Desired-compensation form.** With the regressor evaluated on the
desired trajectory, $Y(q_d,\dot q_d,\dot q_d,\ddot q_d)$, this is the
modified scheme of Sadegh and Horowitz (1990), which uses
desired-trajectory quantities in both the model compensation and the
adaptation law. In position units its adaptation law reads

```math
\dot{\hat a} = -\Gamma\,Y^\mathsf{T}s = \Gamma\,Y^\mathsf{T}\Lambda\,\big(e + T_s\dot e\big),
\qquad T_s = K^{-1}D ,
```

which is feedback-error learning up to the gain on the error signal (and
the parametrisation of the regressors): Kawato's feedback torque is
$-Ds$ here. The literature analyses feedback-error learning as adaptive
control: Nakanishi and Schaal (2004) interpret it as nonlinear adaptive
control and derive a condition on the feedback gains for its stability,
tied to strict positive realness of the tracking-error dynamics;
Miyamura and Kimura (2002) analyse its stability in an adaptive-control
frame, and Miyamura Ideta (2006) treats plants with time delay. Sadegh
and Horowitz keep their stability properties provided the feedback gains
are large enough and an auxiliary nonlinear feedback term is added. In
the position form the servo gains are fixed by its configuration and no
such term exists; this note has not mapped any of these conditions onto
the servo model, so none of the guarantees is assumed to carry over.

**Proper form.** With the regressor on measured state and
$\dot q_r = \dot q_d + \Lambda e$ containing the tracking error, the
offset carries measured signals back into the command at 100 Hz, through
a loop with tens of milliseconds of lag. That is host-side feedback
added around the servo, in the frequency range where the arm's
structural mode and the earlier host-side tracking oscillation are
recorded (see the [computed-torque chapter](../records/history.md#computed-torque)).
Project position: this form is studied in simulation only, until
stability margins are demonstrated there with command lag and joint
flexibility included.

## Comparison

| Scheme | What is learned | Persists between trials | Transfers to a new trajectory | What the cited papers prove | What the position form adds | Hardware status here |
|---|---|---|---|---|---|---|
| ILC (Arimoto) | offset signal $\delta(t)$ | $\delta(t)$ | no | convergence without gravity or with direct compensation; no delay | advance $\Delta$, zero-phase $Q$, identical start | planned after simulation acceptance |
| FEL (Kawato), trial-wise | regressor weights $w$ | $w$ | yes, within the regressors' span | open-loop identifier converges in mean; no closed-loop proof | position-unit regressors, delayed pairing, batch update | planned after simulation acceptance |
| Slotine and Li, desired-compensation form | parameters $\hat a$ | $\hat a$ | yes | stability with large gains and an auxiliary term (Sadegh and Horowitz) | servo PD as $-K_D s$, $D\dot q_d$ regressor | as trial-wise FEL (same update) |
| Slotine and Li, proper form | parameters $\hat a$ | $\hat a$ | yes | Lyapunov stability for torque input, rigid model, no delay | host-side feedback through the 100 Hz lag | simulation only |

## What a demonstration must show

- **The testing ladder applies:** simulation acceptance first, hardware
  only after; a simulation pass is necessary, never sufficient (see the
  [design philosophy](../guide/overview.md#design-philosophy) and the
  [bring-up checklist](../hardware/bringup.md)). The follower's
  simulator (`SimArm`) replaces each servo by the spring law of this
  note, $\tau = k_p(u - q) - k_d\dot q$ with an effort clamp. Its default
  stiffness, 1000 Nm/rad on every joint, is far above the measured $K$
  (see the [evidence](#evidence)) and leaves almost no learnable error.
  The servo-like mode of `x7_follow_sim` (see
  [servo-like simulation](../guide/usage.md#servo-like-simulation)) sets
  per-joint gains, a transport delay of whole control cycles, encoder
  quantization of the feedback, and optional joint friction;
  `config/follow_sim_servo_like.toml` sets the gains from the measured $K$
  and $T$, with part of $T$ booked as a one-cycle delay rather than as
  damping. It is still not a calibrated twin of the arm: one round
  stiffness per servo model does not match the measured per-joint spread,
  the split of $T$ into $k_d/k_p$ and $\theta$ is chosen rather than
  measured (so the simulated bound on $\Psi$ is only as good as that
  split), friction is off unless configured, the servo's voltage
  saturation is represented only by the effort clamp, and the simulator
  adds a numeric damping of its own. A learned offset still does not
  transfer from simulation to hardware; the simulation certifies the update
  machinery and its stability with the modeled lag.
- **Success for ILC** is a monotone fall of the aggregate tracking RMS
  over trials toward the non-repeatable floor.
- **The follower's safety gates stay unchanged** and stay evaluated
  against the reference $q_d$, not against the command $q_d + \delta$.

## References

- Arimoto, S., Kawamura, S., and Miyazaki, F. (1984). Bettering operation
  of robots by learning. *Journal of Robotic Systems*, 1(2), 123–140.
- Arimoto, S., Sekimoto, M., and Kawamura, S. (2007). Iterative learning
  of specified motions in task-space for redundant multi-joint hand-arm
  robots. *Proc. IEEE ICRA 2007*, 2867–2873.
- Arimoto, S., Sekimoto, M., and Kawamura, S. (2008). Task-space
  iterative learning for redundant robotic systems: existence of a
  task-space control and convergence of learning. *SICE Journal of
  Control, Measurement, and System Integration*, 1(4), 312–319.
- Bristow, D. A., Tharayil, M., and Alleyne, A. G. (2006). A survey of
  iterative learning control. *IEEE Control Systems Magazine*, 26(3),
  96–114.
- Kawato, M., Furukawa, K., and Suzuki, R. (1987). A hierarchical
  neural-network model for control and learning of voluntary movement.
  *Biological Cybernetics*, 57, 169–185.
- Longman, R. W. (2000). Iterative learning control and repetitive
  control for engineering practice. *International Journal of Control*,
  73(10), 930–954.
- Miyamura, A., and Kimura, H. (2002). Stability of feedback error
  learning scheme. *Systems & Control Letters*, 45(4), 303–316.
- Miyamura Ideta, A. (2006). Stability of feedback error learning method
  with time delay. *Neurocomputing*, 69(13–15), 1645–1654.
- Nakanishi, J., and Schaal, S. (2004). Feedback error learning and
  nonlinear adaptive control. *Neural Networks*, 17(10), 1453–1465.
- Sadegh, N., and Horowitz, R. (1990). Stability and robustness analysis
  of a class of adaptive controllers for robotic manipulators.
  *International Journal of Robotics Research*, 9(3), 74–92.
- Slotine, J.-J. E., and Li, W. (1987). On the adaptive control of robot
  manipulators. *International Journal of Robotics Research*, 6(3),
  49–59. Not consulted for this note.
- Slotine, J.-J. E., and Li, W. (1991). *Applied Nonlinear Control*.
  Prentice Hall, Englewood Cliffs, NJ.
