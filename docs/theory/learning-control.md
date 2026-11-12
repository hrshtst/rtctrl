# Position-based learning control

*A theory note for the position-based learning study. It models the
servo-side follower as a spring with lag, shows that a position offset
acts as a feedforward torque, and restates three classical learning
schemes in that position form; what the servo-like simulation showed is
in [simulation acceptance](#simulation-acceptance), and the status of
each scheme in the [comparison](#comparison). Prerequisite reading:
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
10.5 rad/s² (the end is at rest). In the archived plain run of that
reference, the elbow (canonical joint 3) has a mean tracking error of
39.8 mrad with the arm below the reference ($e$ = +39.8 mrad; the tool
reports $q - q_{\mathrm{ref}}$ = -39.8 mrad) and an RMS of 48.1 mrad:
most of the elbow's error is a static sag.

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
release the compensation in one step. In a zero-offset trial the ramp removes
the homing correction instead, so the arm's static sag (on the demonstration
reference, the elbow's mean error of 39.8 mrad; see the [evidence](#evidence))
shows before tracking starts; that sag is what the first updates learn. The
final hold also supplies the error
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
damping, so it is optimistic. The usable range of $\Psi$ is therefore
not assumed from it but tried in simulation with the command lag
included: on the demonstration reference of the
[simulation acceptance](#the-demonstration-reference), `psi_s` up to
0.06 s worked and 0.1 s was refused by the offset rate limit (on the
pick reference the sweep did not discriminate), and the nominal
candidate uses $\Psi = 0$. The learn configuration of `x7_follow_learn`
refuses $\Psi$ above 0.1 s (an engineering cap from the library's
convergence tests, not this bound) and $\Phi$ of 2 or more (the
low-frequency condition of the next section).

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
the servo model, so none of the guarantees is assumed to carry over. The
in-loop form was studied in simulation; see
[Slotine and Li in simulation](#slotine-and-li-in-simulation).

**Proper form.** With the regressor on measured state and
$\dot q_r = \dot q_d + \Lambda e$ containing the tracking error, the
offset carries measured signals back into the command at 100 Hz, through
a loop with tens of milliseconds of lag: host-side feedback added around
the servo. In the servo-like simulation that loop is not inherently
unstable: with physically meaningful weights held fixed it was stable
for every $\Lambda$ tested, and what failed was the in-loop adaptation on
the measured-state regressors (see
[Slotine and Li in simulation](#slotine-and-li-in-simulation)). The
simulated joints are rigid, so the arm's structural mode is not in the
simulator; on the arm such feedback acts in the frequency range where
that mode and the earlier host-side tracking oscillation are recorded
(see the [computed-torque chapter](../records/history.md#computed-torque)).
Project position: this form is studied in simulation only, until
stability margins are demonstrated there with command lag and joint
flexibility included.

## Simulation acceptance

This section records what the simulation acceptance of the schemes above
showed: the trial-wise ILC and feedback-error learning of
`x7_follow_learn`, and a simulation-only study of Slotine and Li's
controller in position form. Every run is in the
[servo-like simulation](../guide/usage.md#servo-like-simulation) of
`x7_follow_sim` (`config/follow_sim_servo_like.toml`, unless a case
changes it). Its servo is exactly the spring-with-lag structure this note
assumes, and its arm is exactly the rigid model the regressors are
computed from. A simulation pass is necessary, never sufficient: nothing
in this section is a result on the arm.

Units and terms: "RMS" is the aggregate tracking RMS of one trial over
all 8 joints ($e = q_d - q$ over its tracking samples), in mrad. The
"above-3-Hz error" is the RMS of the error above 3 Hz (the band split of
`follow_learnability.py anatomy`), the proxy for vibration that learning
might add. Trials count from 0, and trial 0 runs the zero offset.

### The plan and its verdict

The criteria were fixed before the runs, in
`config/learn_study/acceptance.toml`. The harness
`tools/follow_learn_study.py` (see
[the simulation study harness](../guide/usage.md#the-simulation-study-harness))
reproduces the runs with the command in that file's header:

```sh
uv run --project tools python tools/follow_learn_study.py \
    config/learn_study/acceptance.toml --jobs 4
```

The nominal candidate, case N1, is ILC with `phi = 0.5` ($\Phi$),
`psi_s = 0` ($\Psi$), `advance_s = 0.05` ($\Delta$, in s) and
`cutoff_hz = 2.0`, a trial-step clamp of 0.05 rad, and offset limits of
0.15 rad and 0.5 rad/s (the defaults), on the planned pick reference
(`config/ptp_pick_example.toml`, 12.9 s), for trials 0 to 5. N1 has five
criteria: (a) every trial and update ran and the follower accepted every
offset as written; (b) the RMS is strictly decreasing over trials 0 to 5;
(c) RMS(5) is at most 15 % of RMS(0); (d) no ILC clamp changed a sample
in any update; (e) the above-3-Hz error of trial 5 is at most 1.5 times
trial 0's. The robustness cases R1 to R14 change one thing each and have
four: (a), (b), RMS(5) at most 30 % of RMS(0), and (e). D1, F1 to F4, T1
and T2 have criteria of their own, given in their subsections; S1 has
none, and the margin sweeps and the runs on the demonstration reference
are not part of the verdict.

**Verdict: 77 criteria, 76 passed, 1 failed** (criterion b of R10, below).
The 77 are 5 for N1, 4 for each of R1 to R14, 3 for D1, 11 for F1 to F4,
and 1 for each of T1 and T2.

| Case | Change from nominal | Criteria | RMS(5)/RMS(0) | RMS, trials 0 to 5 |
|---|---|---|---|---|
| N1 | none | a to e pass | 0.0410 (limit 0.15) | 22.4, 11.6, 5.98, 3.08, 1.63, 0.919 |
| R1 | servo stiffness and damping halved (`position_kp` and `position_kd` times 0.5) | pass | 0.0465 | 41.9, 23.9, 12.7, 6.78, 3.62, 1.95 |
| R2 | servo stiffness and damping doubled | pass | 0.0464 | 12.6, 6.37, 3.24, 1.68, 0.938, 0.583 |
| R3 | command delay 0 cycles (nominal: 1) | pass | 0.0414 | 22.1, 11.4, 5.91, 3.04, 1.61, 0.915 |
| R4 | command delay 2 cycles | pass | 0.0406 | 22.8, 11.7, 6.06, 3.12, 1.64, 0.926 |
| R5 | command delay 3 cycles | pass | 0.0404 | 23.2, 11.9, 6.15, 3.16, 1.66, 0.936 |
| R6 | damping halved (`position_kd` times 0.5) | pass | 0.0416 | 21.9, 11.3, 5.85, 3.02, 1.60, 0.908 |
| R7 | damping doubled | pass | 0.0398 | 24.1, 12.4, 6.36, 3.26, 1.71, 0.959 |
| R8 | joint friction on (Coulomb 0.15 Nm and viscous 0.05 Nm·s/rad; 0.4 and 0.3 on joint 1) | pass | 0.0402 | 24.8, 12.7, 6.56, 3.39, 1.78, 0.997 |
| R9 | no encoder quantization | pass | 0.0362 | 22.4, 11.6, 5.95, 3.06, 1.58, 0.813 |
| R10 | `phi = 1.0` | a, c, e pass; **b fails** | at most 0.30 (c passes) | 22.4, 3.60, 0.445, 0.446, 0.434, 0.447 |
| R11 | `cutoff_hz = 3.0` | pass | 0.0409 | 22.4, 11.6, 5.98, 3.08, 1.63, 0.919 |
| R12 | the pick reference with its first 0.6 s removed | pass | 0.0409 | 22.5, 11.6, 5.99, 3.10, 1.63, 0.919 |
| R13 | the short 2 s reference (`config/ptp_example.toml`) | pass | 0.0342 | 23.4, 11.9, 6.06, 3.05, 1.59, 0.800 |
| R14 | a second reference, B (`config/learn_study/ptp_b.toml`, 10.4 s) | pass | 0.0388 | 22.2, 11.4, 5.80, 2.97, 1.56, 0.862 |

In N1 no clamp acted, and the above-3-Hz error of trial 5 was 0.555
times trial 0's (limit 1.5).

**The failed criterion**, as it stands: R10 reaches a floor at trial 2
(2 % of trial 0's RMS) and stays flat within ±1.5 % after it. The floor
is the encoder's rounding level: at trial 5 every arm joint is between
0.44 and 0.51 mrad, against a rounding floor of 0.443 mrad per joint,
and the above-3-Hz error stays at 0.385 to 0.390 mrad; without
quantization (R9, at the nominal gain) the error keeps falling.
"Strictly decreasing" cannot
hold at such a floor, and the criterion did not anticipate one. It is
recorded as failed, and `phi = 1.0` is not the hardware candidate. R10's
first update also reached the trial-step clamp, on 1140 samples of
joint 3.

Notes on the nominal case:

- At the nominal gain the error roughly halves per trial: the
  quasi-static contraction $|1 - \Phi|$ of the
  [sampled law](#what-100-hz-sampling-adds).
- Joint 3 (the elbow) dominates the error: 57.2 mrad at trial 0 and
  2.16 mrad at trial 5, while the other joints are near 0.5 mrad at
  trial 5.
- At trial 5 the RMS over the first second, the middle and the last
  second of the reference is 0.90, 0.92 and 0.92 mrad: no start or end
  transient is left.
- R12 is a weak test of a start that is not at rest: its first sample
  moves at 0.0069 rad/s, while the demonstration reference starts at up
  to 0.09 rad/s (see [below](#the-demonstration-reference)).

### A disturbance in one trial

Case D1 runs the nominal case for 8 trials with a link force in trial 2
only: 3 N along world +y on the tool point (`crane_x7_tcp_link`),
starting 4.0 s into tracking, rising over 0.3 s, held for 1.5 s and
falling over 0.3 s (`config/learn_study/d1_link_force.toml`). It raised
trial 2's RMS to 1.54 times the undisturbed run's, which is N1 extended
to 8 trials. Criteria: (a) as for N1; (b) the RMS of trials 5, 6 and 7
at most 1.5 times the undisturbed run's at the same trials; (c) no trial
from 3 on above trial 0's RMS.

| Run | RMS, trials 0 to 7 |
|---|---|
| D1 (force in trial 2) | 22.4, 11.6, 9.21, 4.70, 2.38, 1.25, 0.725, 0.514 |
| undisturbed | 22.4, 11.6, 5.98, 3.08, 1.63, 0.919, 0.593, 0.474 |

All three criteria passed: trials 5, 6 and 7 were within 1.358, 1.223
and 1.084 times the undisturbed run's (limit 1.5), and no later trial
rose above trial 0. The learning recovers from a one-off disturbance at
the rate it learns.

### An over-aggressive setting

Case S1 is deliberately too aggressive: no advance (`advance_s = 0`),
`cutoff_hz = 20` and `phi = 1.9`. It was planned for 10 trials and has
no pass criterion; it records what stops such a run.

| Trial | 0 | 1 | 2 | 3 |
|---|---|---|---|---|
| RMS | 22.4 | 9.24 | 8.81 | 8.60 |
| above-3-Hz error | 0.695 | 0.612 | 0.746 | 1.26 |
| maximum offset rate [rad/s] | 0 (zero offset) | 0.119 | 0.224 | 0.294 |

The update after trial 3 was refused by the offset rate limit: the
offset it computed changed at 0.517 rad/s on joint 0, against the limit
of 0.5. The trial-step clamp acted on 1982 samples in the first update.
The aggregate RMS never rose above trial 0's, yet the above-3-Hz error grew
to 1.81 times trial 0's by trial 3 (about four times on joints 4 to 6),
and the offset's maximum rate grew at every trial.

The consequence: a stop rule on the aggregate RMS alone would not have
caught this run. The above-3-Hz error and the offset rate did, and the
offset rate limit stopped it.

### Margins on the pick reference

The sweeps of `config/learn_study/margins.toml` are exploratory and have
no pass or fail: each changes one setting of the nominal case (two in the
cross of cut-off and command delay) and runs 8 trials on the pick
reference. Every case was monotone (the RMS fell at every trial) except
`phi = 1.0`, which reached the encoder floor as R10 did.

| `phi` | RMS, trials 0 to 7 |
|---|---|
| 0.25 | 22.4, 17.0, 12.9, 9.77, 7.40, 5.61, 4.26, 3.25 |
| 1.5 | 22.4, 6.20, 3.29, 1.81, 1.05, 0.689, 0.542, 0.489 (1295 clamped samples) |
| 1.9 | 22.4, 9.24, 8.78, 8.39, 8.04, 7.73, 7.47, 7.24 (1978 clamped samples) |

On this reference the sweeps of the cut-off (1 to 8 Hz), the advance (0
to 0.2 s), `psi_s` (0 to 0.1 s) and the command delay did not
discriminate: all share trials 0 to 4 to three digits and differ only
near the floor (trial 7 between 0.468 and 0.507 mrad), because the pick
reference's error is nearly quasi-static. **These sweeps therefore do
not establish the usable range of the advance, the cut-off or
`psi_s`.**

### The demonstration reference

The hand-taught reference chosen for the learning demonstration
(14.05 s, processed with a 5 Hz low-pass filter as in its archived
hardware run) was run with the same harness, for 8 trials, with the
nominal settings except as stated. The reference is archived privately,
not in the repository, so this subsection is reproducible only with it.
Unlike the pick reference, it discriminates between settings.

| Change from nominal | RMS, trials 0 to 7, or outcome |
|---|---|
| none | 24.2, 12.3, 6.34, 3.50, 2.25, 1.81, 1.65, 1.61 |
| joint friction on | 25.4, 13.1, 6.87, 3.86, 2.50, 1.94, 1.74, 1.66 (offset rate up to 0.40 rad/s) |
| servo stiffness halved | 46.8, 27.8, 15.1, 8.21, 5.62, 4.94, 4.79, 4.87 (the offset reaches the 0.15 rad limit, on 677 samples) |
| `cutoff_hz` 1 | trial 7 at 3.80 (offset rate 0.15 rad/s) |
| `cutoff_hz` 3 | trial 7 at 0.904 (offset rate 0.47 rad/s) |
| `cutoff_hz` 5 and 8 | refused by the offset rate limit after trial 2 (on joint 5, about 10.2 s into the reference, its fastest wrist motion) |
| `advance_s` 0, 0.03, 0.08, 0.12, 0.2 | trial 7 at 1.80, 1.66, 1.58, 1.59, 2.00 (nominal 0.05: 1.61) |
| `psi_s` 0.03 and 0.06 | trial 7 at 1.47 and 1.32 |
| `psi_s` 0.1 | refused by the offset rate limit after trial 1 |
| `phi` 1.0 | 24.2, 6.94, 1.76, 1.39, 1.33, 1.30, 1.29, 1.29 (698 clamped samples in the first update) |
| `phi` 1.5 | refused by the offset rate limit after trial 0 |
| command delay 3 cycles, `cutoff_hz` 2 | trial 7 at 1.91 |
| command delay 3 cycles, `cutoff_hz` 5 and 8 | refused after trial 1 |

In the nominal run the trial-step clamp acted on 50 samples in the first
update; the offset settles at a maximum of 104 mrad and a maximum rate
of 0.33 rad/s, against limits of 150 mrad and 0.5 rad/s; the above-3-Hz
error is 0.74 mrad at trial 0 and 0.65 mrad at trial 7. With the servo
stiffness halved the error stops falling near 4.9 mrad.

Reading: on this reference the offset rate limit is what bounds the
cut-off (between 3 and 5 Hz) and the gains; the nominal setting keeps
about a third of the rate limit in hand; a softer servo than assumed
makes the value limit bind. The reference starts at up to 0.09 rad/s
and 10.5 rad/s² (see the [conditions](#conditions-the-papers-assume)),
and the nominal run's RMS fell at every trial regardless.

### Feedback-error learning between trials

Cases F1 to F4 run `x7_follow_learn fel` on the pick reference, trials 0
to 5, with the learn defaults except as stated (regressor and error
smoothing 0.04 s, advance 0.05 s, no error lead). Their criteria: F1, (a)
as for N1, RMS(1) at most 20 % of RMS(0), and no later trial more than
1.2 times RMS(1); F2, (a), strictly decreasing, and RMS(5) at most 10 %
of RMS(0); F3, (a) and strictly decreasing; F4, (a), strictly
decreasing, and RMS(5) at most 30 % of RMS(0).

| Case | Step, gain | Criteria | RMS, trials 0 to 5 |
|---|---|---|---|
| F1 | least squares, 1 | pass | 22.4, 0.908, 0.415, 0.415, 0.418, 0.417 |
| F2 | least squares, 0.5 | pass | 22.4, 11.6, 5.97, 3.09, 1.63, 0.917 |
| F3 | normalised gradient, 0.2 | pass | 22.4, 8.30, 5.36, 4.53, 4.12, 3.80 |
| F4 | least squares, 0.5, joint friction on (as R8) | pass | 24.8, 12.7, 6.55, 3.40, 1.77, 0.987 |

F3 decreases, but its error stays larger at the two ends of the
reference: 6.2 and 6.7 mrad over the first and the last second, against
3.1 mrad in the middle.

Transfer (T1, T2): weights trained with F2's settings for five updates,
synthesised for another reference, which then runs once with the zero
offset and once with the synthesised one, without further learning; the
criterion is a ratio of at most 0.5. Trained on the pick reference and
applied to reference B (T1): 0.808 mrad against 22.2 mrad with the zero
offset (ratio 0.0364). Trained on B and applied to the pick reference
(T2): 0.991 mrad against 22.4 mrad (ratio 0.0442). Both passed.

!!! warning "The simulation flatters feedback-error learning"
    The simulated servo and arm are exactly what the regressors express.
    On hardware logs the same regressors, fitted on other trajectories,
    left a baseline-to-residual ratio of only 2.47 (see the
    [evidence](#evidence)). The transfer result shows that the machinery
    works, not what the arm will do.

On the demonstration reference, least squares at gain 0.5 with the
default regressor smoothing (`fel.regressors.smooth_sigma_s = 0.04`)
reached 24.2, 12.2, 6.13, 3.14 mrad and was then refused by the offset
rate limit (its offset rate had reached 0.47 rad/s). With regressor
smoothing of 0.06, 0.08 and 0.12 s it completed 8 trials, ending at
0.713, 1.04 and 1.86 mrad with offset rates of 0.50, 0.45 and
0.36 rad/s. A smoother regressor trades accuracy for margin to the rate
limit.

### Slotine and Li in simulation

The study binary `x7_adaptive_follow_study` (see
[the adaptive-control study](../guide/usage.md#the-adaptive-control-study))
is simulation only and, like the exact feedback linearization study
(`x7_efl_study`), offline by charter: it has no bus code and is not a
controller for the arm. Its controller is the position form of
[the textbook law](#slotine-and-li-adaptive-control-in-position-form),
with these departures (from the header of
`apps/study/adaptive_position.hpp`):

- Per-regressor weights in position units replace the physical parameter
  vector: per arm joint the offset is
  $\delta_j = \sum_l w_{jl}\,\phi_{jl}$, clamped to ±0.15 rad, with the
  six regressors of the
  [feedback-error learning table](#feedback-error-learning-in-position-form),
  one scale per regressor and joint.
- The servo's own PD supplies the sliding-variable feedback; the study
  computes no $-K_D s$.
- Adaptation runs every control cycle,
  $w_{jl} \leftarrow w_{jl} + h\,\gamma_{jl}\,\phi_{jl}(t - \Delta)\,s_j(t)$
  with $s_j = e_j + T_s\dot e_j$ and $h$ the control period, and with
  per-regressor rates $\gamma_{jl} = \gamma / \overline{\phi_{jl}^2}$
  normalised on the desired trajectory, so that $\gamma$ (in 1/s) is a
  common rate. There is no parameter projection.
- In the desired-compensation form the regressors are evaluated on the
  desired trajectory only. In the proper form they are evaluated on the
  measured state with the reference velocity $\dot q_r = \dot q_d + \Lambda e$
  (and $\ddot q_r = \ddot q_d + \Lambda\dot e$).

Each pass runs the pick reference once on a fresh servo-like simulated
arm (homing, the follower's offset ramp, tracking with the in-loop
offset, a 1 s hold); the weights carry over between passes, and every
run of the study had 8 passes. Without adaptation the RMS is 22.4 mrad.

**Desired-compensation form.**

| $\gamma$ [1/s] | RMS, passes 0 to 7 |
|---|---|
| 0.5 | 4.38, 1.82, 1.31, 1.09, 0.953, 0.805, 0.696, 0.613 |
| 1 | 3.50, 1.21, 0.989, 0.972, 0.842, 0.722, 0.589, 0.574 |

At $\gamma = 0.5$ no clamp acted, and the above-3-Hz error was
0.894 mrad in the first pass and 0.395 mrad in the last. The scheme
learns within the first pass, where the trial-wise schemes learn
nothing, and then keeps falling at every pass. Part of this is slow
feedback inside the pass and not learned feedforward: the weights after
pass 7, held fixed, give 1.14 mrad. The stability window is narrow:
$\gamma = 1.25$ is stable but wanders between passes, $\gamma = 1.375$
raises the above-3-Hz error past twice the baseline's, and
$\gamma = 1.5$ and above reach the 0.15 rad offset clamp (at
$\gamma = 2$, about 52 mrad in every pass). With a command delay of 0 to
3 cycles, $\gamma = 0.5$ stays monotone (pass 7 between 0.602 and
0.640 mrad), while $\gamma = 1$ becomes non-monotone at 2 and 3 cycles.
An error lead $T_s$ of 0.02 or 0.04 s at $\gamma = 0.5$ changes little
(pass 7 at 0.641 and 0.695 mrad).

**Proper form, with adaptation.** It is stable only at $\Lambda = 0$,
where it comes close to the desired form (pass 7 at 0.605 mrad, against
0.613). For
every $\Lambda$ in {2, 5, 10, 20} 1/s and every $\gamma$ in
{0.5, 2, 5, 20} 1/s the offset reaches its clamp from the first pass;
for example, at $\Lambda = 2$ and $\gamma = 0.5$ the RMS is 19.2, 14.4,
15.1, 21.7, 13.0, 14.4, 17.4, 13.6 mrad, with the above-3-Hz error at
10.9 mrad against the baseline's 0.70. The textbook-consistent setting
($\Lambda = K/D = 25$ 1/s, $T_s = D/K = 0.04$ s, $\gamma = 0.5$) fails
in the same way from zero weights and from physical weights (about 37 to
42 mrad in every pass).

**Proper form, without adaptation.** With physically meaningful weights
held fixed (per joint, $1/K$ on the model torques, the simulator's
reflected inertia over $K$ on the acceleration, the fitted lag on the
velocity) the proper form is stable for every $\Lambda$ tested, and
better than the desired form with the same weights: 0.910 mrad for the
desired form, and 0.859, 0.773, 0.693 and 0.625 mrad at $\Lambda$ = 2,
5, 10 and 25 1/s, with the above-3-Hz error rising from 0.389 mrad to
0.392, 0.402, 0.422 and 0.479 mrad; no clamp acted.

What this establishes: in this simulation, the loop that feeds the
measured state back through the position command at 100 Hz is not
inherently unstable. What fails is the in-loop adaptation on
measured-state regressors. Their columns are far larger than the
desired-trajectory columns the adaptation rates were normalised on (in
mean square, 60 to 490 times for the model dynamic torque and 18 to 114
times for the acceleration, at $\Lambda = 25$), which pushes the
effective rate past the discrete update's own bound. And non-physical
weights, such as those the desired form learns (for example $-17.7$ on a
model-torque column whose physical value is 0.05), turn into
destabilising feedback gains in the proper form.

What it does not establish: robustness to wrong weights (the fixed
weights were the simulator's own exact parameters), the arm's structural
mode, PWM saturation, or the lag of the real velocity estimate. None of
the conditions for trying the proper form on the arm is established.

This revises the paragraph on the proper form in the
[Slotine and Li section](#slotine-and-li-adaptive-control-in-position-form),
which no longer presents the loop itself as the demonstrated problem.
The project position is unchanged: the proper form stays in simulation.

### What the simulation acceptance supports

- The nominal ILC setting met every predeclared criterion and is the
  candidate for a hardware proposal.
- The stop rule for hardware must watch the above-3-Hz error and the
  offset rate, not only the aggregate RMS (case S1).
- The offset limits are an effective backstop; on the demonstration
  reference they bind first through the rate limit.
- For trial-wise feedback-error learning the least-squares step is the
  one to use; on the demonstration reference it needs smoother
  regressors to stay inside the rate limit.
- In-loop adaptation (either Slotine and Li form) is not proposed for
  hardware.

## Comparison

| Scheme | What is learned | Persists between trials | Transfers to a new trajectory | What the cited papers prove | What the position form adds | Status here |
|---|---|---|---|---|---|---|
| ILC (Arimoto) | offset signal $\delta(t)$ | $\delta(t)$ | no | convergence without gravity or with direct compensation; no delay | advance $\Delta$, zero-phase $Q$, identical start | simulation acceptance done ([verdict](#the-plan-and-its-verdict)); hardware not run and not approved |
| FEL (Kawato), trial-wise | regressor weights $w$ | $w$ | yes, within the regressors' span | open-loop identifier converges in mean; no closed-loop proof | position-unit regressors, delayed pairing, batch update | simulation acceptance done ([results](#feedback-error-learning-between-trials)); hardware not run and not approved |
| Slotine and Li, desired-compensation form | parameters $\hat a$ | $\hat a$ | yes | stability with large gains and an auxiliary term (Sadegh and Horowitz) | servo PD as $-K_D s$, $D\dot q_d$ regressor | in the loop: works in simulation within a narrow stability window, not proposed for hardware ([study](#slotine-and-li-in-simulation)); between trials it is the FEL update |
| Slotine and Li, proper form | parameters $\hat a$ | $\hat a$ | yes | Lyapunov stability for torque input, rigid model, no delay | host-side feedback through the 100 Hz lag | simulation only: in-loop adaptation drives the offset to its clamp for every $\Lambda > 0$ tested, while fixed physical weights are stable ([study](#slotine-and-li-in-simulation)) |

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
  machinery and its stability with the modeled lag. Its results are in
  [simulation acceptance](#simulation-acceptance).
- **Success for ILC** is a monotone fall of the aggregate tracking RMS
  over trials toward the non-repeatable floor. That fall alone is not a
  safe stop rule: in the simulation's over-aggressive case
  ([S1](#an-over-aggressive-setting)) the aggregate RMS never rose above
  trial 0's while the above-3-Hz error grew and the offset rate climbed
  until the rate limit refused an update. A hardware stop rule must
  therefore also watch the above-3-Hz error and the offset rate.
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
