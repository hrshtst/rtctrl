# Flexible-mode identification

*Implemented in `apps/ident_common.hpp` (probe state machine, online
demodulation, safety monitors), `apps/x7_ident.cpp` /
`apps/x7_ident_sim.cpp` (hardware app and simulation twin), and
`tools/ident_analysis.py` (offline estimation). Design record:
[identification plan](../IDENTIFICATION_PLAN.md); operator procedure:
[identification protocol](../IDENTIFICATION_PROTOCOL.md); run order and
outcomes: [campaign checklist](../IDENTIFICATION_CAMPAIGN.md).
Prerequisite reading:
[computed torque](computed-torque.md), whose limitations this page
starts from. The campaign this theory served was **closed 2026-07-28
with a null result** — the conclusion section quantifies why.*

## Motivation: where computed torque hits its limit

The [computed-torque loop](computed-torque.md) treats the arm as a
rigid-body chain. The real arm is not rigid: each joint drives its link
through a gear train with finite stiffness, and at scale $> 0.6$ the
tracking excursions reach configurations whose structural mode near
4–5 Hz the 100 Hz loop *pumps* (2026-07-21 runs 7–8: coherent
whole-arm oscillation, both ended in manual power cuts).

The mechanism is phase lag in the damping path. The D-term is meant to
produce a force opposing velocity, but every element between the true
link velocity and the applied torque delays it: the PD low-pass
(first-order, $\tau_f = 0.05\,\mathrm{s}$), the velocity estimator
filter ($\tau_v = 0.02\,\mathrm{s}$), and the sampled bus pipeline
(feedback age, write-to-apply, zero-order hold — about two control
cycles). A first-order filter contributes $\arctan(\omega\tau)$ and a
pure delay $T$ contributes $\omega T$, so at the mode frequency
$f_0 = 4.5\,\mathrm{Hz}$:

```math
\varphi \;=\; \underbrace{\arctan(2\pi f_0 \tau_f)}_{\approx 55^\circ}
\;+\; \underbrace{\arctan(2\pi f_0 \tau_v)}_{\approx 30^\circ}
\;+\; \underbrace{2\pi f_0 T_{\text{bus}}}_{\approx 32^\circ}
\;\approx\; 117^\circ .
```

Delayed damping stops damping once the lag passes $90^\circ$. With a
modal velocity $v(t) = V\cos\omega t$ the delayed feedback force is
$f(t) = -K_d V \cos(\omega t - \varphi)$, and its average power into
the mode is

```math
\bar P \;=\; \langle f\,v\rangle
\;=\; -\tfrac{1}{2} K_d V^2 \cos\varphi
\;>\; 0 \quad \text{for } \varphi > 90^\circ,
```

so at $117^\circ$ the "damping" *injects* energy: the loop is an
oscillator, not a regulator, at that frequency. Input shaping cannot
fix this — it only reduces reference excitation, and this instability
is internal to the feedback loop.

The remedy would be a notch or phase-compensated D-path — but designing
one requires knowing, per posture, the mode frequency $f_n$, damping
$\zeta$, and which joints participate. Producing that table was the
goal of the identification campaign. Until it existed, the excursion
scale stayed capped at 0.6; the cap is now **final** (see the
conclusion).

## The plant to identify

The minimal model of one geared joint is two inertias coupled by the
gear stiffness: motor-side $J_m$ (rotor and gears, reflected) and
link-side $J_l$, coupled by stiffness $K_g$ and damping $C_g$:

```math
\begin{aligned}
J_m \ddot\theta_m &= \tau_m - K_g(\theta_m - \theta_l)
                    - C_g(\dot\theta_m - \dot\theta_l),\\
J_l \ddot\theta_l &= K_g(\theta_m - \theta_l)
                    + C_g(\dot\theta_m - \dot\theta_l) - b\,\dot\theta_l .
\end{aligned}
```

Its elastic mode has

```math
\omega_0^2 \;=\; K_g\!\left(\frac{1}{J_m} + \frac{1}{J_l}\right),
\qquad
\zeta \;=\; \frac{C_g\,\omega_0}{2 K_g}
\left(\text{equivalently } \frac{C_g}{2}\sqrt{\tfrac{1/J_m + 1/J_l}{K_g}}\right),
```

and $J_l$ is configuration-dependent (the arm's inertia about the
joint changes with posture), which is why the mode is *posture-
dependent* and the campaign planned four postures P1–P4. The
simulation twin (`apps/two_mass_arm.hpp`) plants exactly this model —
4.5 Hz, $\zeta = 0.03$ on joint 1 and 13 Hz, $\zeta = 0.05$ on
joint 5 — and the full pipeline recovers 4.51 Hz / $\zeta$ 0.033 and
13.01 Hz / $\zeta$ 0.058 from it, which validated probe, logging, and
analysis against known truth before hardware.

One geometric fact matters more than any other below: the XM430's
magnetic encoder sits on the **output shaft**, after the gearbox. The
sensor is collocated with the link, so gear wind-up
$\delta = \theta_m - \theta_l$ — the elastic state itself — is
invisible; the encoder only sees the mode when the *output* moves.

## Probing a closed loop without modeling the controller

The arm cannot be excited open-loop: gravity would drop it. The probe
therefore rides on the proven anchor controller (a `ComputedTorque`
holding a constant posture), and a stepped-sine torque

```math
\tau_p(t) \;=\; A(f)\,\sin\varphi(t),
\qquad
\varphi(t) = 2\pi f \!\int\! dt \ \ (\text{accumulated from measured } dt)
```

is added to one joint at a time, one dwell per frequency over the
survey grid $\{2,\dots,20\}\,\mathrm{Hz}$.

Closed-loop identification usually requires de-embedding the
controller. It does not here, because the telemetry records the
**total applied torque**. Writing the plant as
$Q(j\omega) = G(j\omega)\,T_{\text{tot}}(j\omega)$ where
$T_{\text{tot}} = T_p + T_{\text{fb}}$ is probe plus feedback
reaction, both $Q$ and $T_{\text{tot}}$ are *measured* — so the
direct ratio

```math
\hat G(j\omega) \;=\; \frac{\mathcal{F}\{q\}(j\omega)}
                           {\mathcal{F}\{\tau_{\text{tot}}\}(j\omega)}
```

is the plant FRF with no controller model at all. Two variants are
computed: the primary uses measured torque (current $\times$ torque
constant, sampled at the same events as $q$), the secondary uses the
commanded total delay-corrected by the receipt-matched apply latency —
and their ratio is itself a deliverable, the commanded→measured
**actuator transfer** (it survived the campaign; see the conclusion).

One caution the campaign taught: the resulting FRF is
**controller-specific** on a multivariable arm. The other joints'
coherent feedback torques are coupled inputs that this scalar ratio's
denominator does not contain, so runs under different tunings are not
one dataset — the analysis refuses to merge them (the sidecar records
the full tuning; `same_tuning` is a merge gate).

**Amplitude rule.** Off resonance a free inertia responds with
$|G| \approx 1/(\hat J\omega^2)$, so to target a response amplitude
$x_t$ the probe needs

```math
A(f) \;=\; \operatorname{clamp}\!\big(x_t\,\hat J\,(2\pi f)^2,\;
0.05\,\mathrm{Nm},\; A_{\text{cap}}\big),
```

with $x_t = 5\,\mathrm{mrad}$, default cap $0.15\,\mathrm{Nm}$ and an
unbypassable hard cap $0.3\,\mathrm{Nm}$. Near resonance the response
is $\times Q = 1/(2\zeta)$ larger (~8 mrad at 4.5 Hz for 0.15 Nm at
$\zeta=0.03$), which is where the fit gets its information. This rule
assumes a *free linear* plant — the assumption the hardware falsified.

## Demodulation as least squares

Each dwell's measurement window (an integer number of probe periods,
minimum $\max(10\,\text{periods}, 3\,\mathrm{s})$) is reduced to one
complex point per signal by a **dt-weighted least-squares fit** on the
regressors $[\,1,\; t,\; \sin\varphi,\; \cos\varphi\,]$:

```math
y(t_k) \;=\; a + b\,t_k + I\sin\varphi_k + Q\cos\varphi_k + n_k,
\qquad \hat Z = I + jQ .
```

Plain correlation ($\hat I \propto \sum y_k \sin\varphi_k$) assumes
the regressors are orthogonal over the window. They are not: sampling
jitter means the window never ends exactly on a $2\pi n$ phase
crossing, so a constant offset $c$ leaks into the correlation at
order $c/(\omega T')$ per residual fraction of a period. That is not
academic — at posture P1 the still joint 4 sits at
$q_4 = -2.456\,\mathrm{rad}$, and correlation demodulation "measured"
a 0.096 rad response on a motionless joint, three times the response
cap. The LS fit absorbs offset and drift *explicitly* and is exact
for any window end; the window still ends near the phase crossing for
conditioning.

Before the window opens, an **adaptive pre-measure hold** waits out
the onset transient, which decays as $e^{-\zeta\omega_0 t}$ with time
constant $1/(\zeta\omega_0) \approx 1.2\,\mathrm{s}$ at 4.5 Hz,
$\zeta = 0.03$. The hold accepts when two consecutive one-period block
estimates agree,

```math
\frac{|\hat Z_k - \hat Z_{k-1}|}
     {\max(|\hat Z_k|, |\hat Z_{k-1}|, Z_{\text{floor}})} \;<\; 0.1 ,
```

on *both* signals, with per-signal floors calibrated from the unforced
lead-in (three times the RMS of one-period block magnitudes with the
probe off), bounded to $[1, 4]\,\mathrm{s}$; a timeout marks the dwell
low-confidence rather than blocking the run.

## From FRF points to modes

Each detected peak band gets a complex single-mode fit

```math
H(f) \;\approx\; \frac{c}{\,f_n^2 - f^2 + 2j\zeta f_n f\,} + d,
\qquad c, d \in \mathbb{C},
```

which is linear in $(c, d)$ for fixed $(f_n, \zeta)$ — so the fit
scans a fine $(f_n, \zeta)$ grid, solves the linear subproblem at each
point, and keeps the residual minimum; confidence intervals are the
grid extent of the $\Delta\chi^2$ band. The survey grid alone cannot
produce damping: the half-power width of a $\zeta = 0.03$ mode at
4.5 Hz is

```math
\Delta f \;=\; 2\zeta f_n \;\approx\; 0.27\,\mathrm{Hz},
```

narrower than the 0.5 Hz survey spacing — hence mandatory refinement
grids (0.15 Hz steps around 4–5 Hz). A mode entry is only marked
*refined* with three pieces of evidence: a fine-enough local grid,
per-frequency repeat visits from two invocations traversing the band
in **both** sweep directions (repeatability and hysteresis), and a
half-amplitude repeat of the peak dwell (gear compliance and backlash
are amplitude-dependent — a linearity spot check). Anything less is
survey-confidence only.

## Observability: what the output encoder can resolve

The encoder quantizes to $\Delta = 2\pi/4096 \approx 1.53\,\mathrm{mrad}$
per count, with quantization noise
$\sigma_q = \Delta/\sqrt{12} \approx 0.44\,\mathrm{mrad}$. A sinusoid
amplitude estimated by LS over $N$ samples has standard deviation
$\sigma_A = \sigma_q\sqrt{2/N}$, so a 3 s window at 100 Hz
($N \approx 300$) gives the analytic floor

```math
\sigma_A \;=\; 0.44\,\mathrm{mrad}\times\sqrt{2/300}
\;\approx\; 3.6\times10^{-5}\,\mathrm{rad}.
```

That derivation carries a hidden assumption: quantization noise only
averages down when the signal *dithers across code boundaries*. A
stationary joint parked inside one code is **tick-frozen** — the
recorded signal is exactly constant, the LS sine coefficients are
numerical zeros, and no amount of averaging manufactures resolution.
The campaign therefore measured its floors empirically too (the
lead-in calibration above), and the analysis applies a per-dwell
effective floor $\max(3.6\times10^{-5}\,\mathrm{rad},\ \text{measured
floor})$ and refuses to fit modes unless at least five dwells are both
confident and above it — a refusal reads "insufficient usable data",
never a fit of noise.

## What the hardware said

**Stabilizing the anchor first.** The stationary hold itself had to be
qualified before probing, and it exposed a mechanism worth recording:
integrator–stiction hunting. A joint held *inside* tolerance by static
friction has a small constant error $e$, so the integral term winds at
constant rate,

```math
\dot\tau_I \;=\; K_i\,e \;\approx\; 6 \times 0.012 \;\approx\;
0.07\,\mathrm{Nm/s},
```

until the accumulated torque exceeds the static-friction margin and
the joint breaks away (~26 mrad slips, recurring after every honest
re-acceptance — observed as joint 3 winding $-0.07 \to -0.54$ Nm).
The production fix freezes each joint's integral once that joint has
individually held in-band and quiet for 0.3 s (per-joint latching; the
frozen bias keeps acting, it is never reset), with global capture
admission requiring all joints latched plus 0.3 s of simultaneous
quiet. Under that controller the P1 hold qualified: three consecutive
30 s holds, then a production-path equivalence hold, all under
continuous $\pm 0.02$ rad and $0.05$ rad/s gates, with a placement
transition sag of 3.2 mrad against the 80 mrad envelope.

**The null result.** The survey and the reviewer-directed amplitude
pilot then answered the identification question negatively — precisely
scoped: **joint 1 at posture P1 is not identifiable with this
stationary small-signal current probe up to the 0.30 Nm hard cap.**

- In the survey (0.05–0.15 Nm), the probe joint's encoder reported a
  *single constant count* through 13 of 16 measurement windows: the
  probe torque sits below the gearbox's static-friction breakaway, so
  the output shaft simply does not move. The demodulated responses
  were numerical zeros ($\sim 10^{-16}\,\mathrm{rad}$).
- The pilot stepped one 4.5 Hz dwell through 0.20 / 0.25 / 0.30 Nm.
  Torque was delivered faithfully (measured 0.201 / 0.251 / 0.302 Nm),
  and the responses stayed noise: 22 / 68 / 38 µrad against
  run-measured floors of 423 / 733 / 1051 µrad (response-to-floor
  ratios 0.05 / 0.09 / 0.04), non-monotonic in amplitude where a real
  $|H|$ would be constant, with phase jumping by $\sim 150^\circ$
  between adjacent amplitudes.
- The physics closes consistently: below breakaway the probe only
  winds up the gear train, $\delta = \tau_p/K_g$, on the *motor* side
  of the output encoder — the one place the collocated sensor cannot
  see — and the whole-arm mode that forced the cap participates most
  strongly at the gravity-loaded shoulder joints, exactly where
  stiction is highest.

**What was measured cleanly.** The actuator transfer came out as a
textbook pure delay: magnitude $1.00$–$1.05$ flat across 2–20 Hz and
phase falling linearly from $-8^\circ$ to $-76^\circ$, i.e.
$e^{-j\omega T_d}$ with

```math
T_d \;\approx\; \frac{76^\circ}{360^\circ \times 20\,\mathrm{Hz}}
\;\approx\; 10.6\,\mathrm{ms},
```

consistent with the receipt-measured one-cycle apply delay
(~9.9 ms). Any future compensator design has its actuator phase
budget. The campaign also produced the first empirical stationary
noise floors (0.4–1.1 mrad — 10–30× the analytic figure, because
ambient motion is only 1–2 counts) and the qualified hold controller.

**Conclusion.** By owner decision (2026-07-28), following the
reviewer-confirmed stop condition, the **0.6 excursion-scale cap in
`x7_track` is the final supported boundary**; the notch/phase D-path
redesign is closed, not deferred. Other joints, postures, sensing, and
excitation strategies were *not tested* — the generalization is a
judgment, recorded as such. Reopening requires a concrete need for
scale $> 0.6$ plus one of: external link-side sensing (the flexible
response was invisible to the servo encoder in the tested
configuration), identification around a *moving* operating point
(motion linearizes the friction that defeated the stationary probe —
the modes were originally observed during tracking), or
dither-assisted excitation — each restarting from external review.
Full evidence and provenance:
[identification plan, Closure](../IDENTIFICATION_PLAN.md) and the
[data archive manifest](../DATA_ARCHIVE.md).
