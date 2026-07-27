# Flexible-mode identification protocol

> **CAMPAIGN CLOSED 2026-07-28** — the stationary probe reached its
> documented stop condition: joint 1 at P1 is not identifiable up to
> the 0.30 Nm hard cap (other joints, postures, sensing, and
> excitations were not tested), and the owner closed the campaign —
> see the [campaign checklist](IDENTIFICATION_CAMPAIGN.md) §3a and
> the [design record's Closure section](IDENTIFICATION_PLAN.md).
> **The 0.6 excursion-scale cap in `x7_track` is the final supported
> boundary.** This procedure is retained as the historical record
> and for any documented reopening.

Operator procedure for the pass-2 stepped-sine identification campaign
([design record](IDENTIFICATION_PLAN.md)). The goal: per-posture tables
of the arm's flexible modes — the configuration-dependent ~4–5 Hz
whole-arm structural mode and the ~13 Hz gear mode — with frequency,
damping, and joint participation.

## Safety

- Current mode; **actuator power cutoff within reach at all times** —
  it is the final physical mechanism behind every software layer.
- Workspace clear; hand support ready for the first settle at each new
  posture (P3/P4 first settles are the riskiest moments).
- Session deadline ladder: graceful stop at **T_stop = 177.5 s** after
  activation; the independent session watchdog silences the bus at
  **T_quiesce = 179.5 s** so the servos' own Bus Watchdogs stop the arm
  — servos stop by 180 s as a conservative designed bound, not a
  proven exact deadline.
- **Hard faults** (thermal, voltage, position gate, ≥ 0.08 rad anchor
  deviation, saturation, stale feedback, latency, I/O) abort and
  deactivate with **no retry** — investigate before rerunning.
  **Soft events** (0.03 rad demodulated response on any joint, response
  growth in the measure window, sustained probe clipping) take a 50 ms
  kill ramp, re-settle, and at most one automatic half-amplitude retry;
  a recurrence aborts.

## Postures

**Primary: integrated placement (`x7_ident --pose-first`,
reviewer-approved).** The app moves to the `--anchor-ref` posture in
POSITION mode and converges the MEASURED posture (goal-offset
iterations close the friction sag), switches to current mode
in-process with a clamped gravity-current preload (the servos hold
from the instant torque re-enables), then runs a CAPTURE phase: a
bounded, gently ramped reference from the measured posture onto the
canonical anchor under the QUALIFIED identification anchor controller
(joint-0 PD scale 0.5, shipped scales elsewhere — see the plan
addendum; x7_track's tuning is unchanged), with the quiescence metric
and the ±0.02 rad anchor gate evaluated UNDER that hold. During
capture, each joint's learned integral bias LATCHES individually the
moment that joint has been in-band and quiet for 0.3 s — a stable
joint must not keep winding while another joint blocks the arm-wide
admission — and admission itself requires all eight latched plus the
unchanged simultaneous gates for a further 0.3 s. Latched biases are
never reset or resumed: the captured friction/gravity bias keeps
pinning the posture through lead-in, probing, retries and the final
hold, and each joint's bias and latch time are recorded in the
sidecar. A post-transition displacement beyond the 0.08 rad capture
envelope aborts — capture closes a small expected residual, never an
arbitrary offset. One controller instance spans capture, gates,
lead-in and probe. This replaced the hand handover after four
hardware sessions showed the limp gap loses 0.06–0.18 rad on the
gravity-loaded joints.

**Fallback: hand placement.** Deactivate (or run before activation),
place the limp arm by hand at the target vector, then let activation +
gravity compensation hold it; the strict settle gate verifies
stillness and the app refuses to probe unless the settled anchor
matches the reference within **±0.02 rad per joint** (`--anchor-ref`).
`x7_pose` gives rough positioning, but the torque-off handover does
NOT preserve the posture — expect anchor-gate refusals; the gates are
doing their job. NOTE: the fallback has no capture phase, so its
integrator stays LIVE — a different effective controller. The sidecar
records that mode and the analysis refuses to merge it with frozen-
integrator pose-first data: **manual-flow surveys are not campaign
data** (debugging only).

| posture | description | canonical vector [rad] |
|---|---|---|
| P1 | pass-1 anchor (proven) — `config/postures/p1.json` | `-0.357, -0.831, 2.126, -1.572, -2.456, -0.106, 0.563, -0.014` |
| P2 | track7/8 anchor (structural-mode posture) — `config/postures/p2.json` | `-0.377, -0.873, 2.174, -1.608, -2.264, 0.109, 0.561, 0.086` |
| P3 | extended endpoint region (nominal) | `0.00, -0.60, 0.00, -0.80, 0.00, -0.30, 0.00, 0.00` |
| P4 | near-horizontal max extension (nominal) | `0.00, -1.30, 0.00, -0.35, 0.00, -0.15, 0.00, 0.00` |

P1 and P2 have CHECKED-IN reference files, used in a two-step policy:
the FIRST run of a session set (the survey) passes
`--anchor-ref config/postures/p1.json` (resp. `p2.json`), so no run
can silently establish an arbitrary posture as canonical; every
SUBSEQUENT run (the refinement sweeps) passes that survey's
`.dwells.json` sidecar, preserving the actually-settled anchor so the
combined dataset stays within the analysis merge guard's ±0.02 rad of
itself. Accepted consequence: a refinement anchor can sit up to
~0.04 rad from the canonical vector in the worst chain, while the
dataset remains internally coherent and its survey is within 0.02 rad
of canonical.

Work in the proven→new order P1 → P2 → P3 → P4. P3/P4 are nominal
design targets: on the FIRST session at each, the actually-settled
anchor recorded in the `.dwells.json` sidecar becomes the reference
for every later run there. Gravity load rises toward P4 — the
headroom precheck may reduce amplitudes or skip dwells; that is the
system working, not a fault. The posture-sensitivity micro-test
(repeat the P2 survey after deliberate re-placement) calibrates the
±0.02 rad tolerance empirically.

## Joint / frequency matrix

Default probe joints: **1** (shoulder tilt — structural mode), **3**
(elbow), **5** (wrist pitch — gear mode). One probe joint, one posture
per invocation (`--joint`, exactly one value).

- **Survey grid** (default `--freqs`):
  2, 3, 3.5, 4, 4.5, 5, 5.5, 6, 7, 8, 10, 12, 13, 14, 16, 20 Hz.
- **Refinement grids** (after the survey names the peaks): ±0.75 Hz in
  0.15 Hz steps around a 4–5 Hz peak; ±1.5 Hz in 0.3 Hz steps around a
  ~13 Hz peak. The 0.5 Hz survey spacing cannot resolve ζ ≈ 0.03 —
  a mode table without refinement data is marked survey-confidence
  only.

## Amplitude rule

`A(f) = clamp(x_t · Ĵ · ω², 0.05 Nm, cap)` with x_t = 0.005 rad and Ĵ
the diagonal inertia at the settled anchor; cap 0.15 Nm default,
raisable via `--amp` to the 0.3 Nm hard cap. Cap-bound dwells whose
predicted response falls below 3× the window noise floor request a ×4
window extension, granted only when the session budget admits it. A
per-dwell override `f@amp` in `--freqs` sets an explicit amplitude
(used for the half-amplitude linearity point).

## The three invocations per probe joint-posture

Each invocation fits the budget on its own (~1.5–2.5 min); the
worst-case base total is 146 + 108 + 99 ≈ 353 s ≈ 6 min of collection
per probe joint-posture, EXCLUDING cooldown and operator handling.
Cross-invocation combination is legitimate through the anchor-reference
gate.

1. **Survey** — default grid, integrated placement against the
   canonical reference:

   ```sh
   ./build/apps/x7_ident --joint 1 --pose-first \
       --anchor-ref config/postures/p1.json \
       --label p1-j1-survey --log p1_j1_survey.csv
   ```

   Analyze BEFORE refining: verify the achieved noise floor and dwell
   verdicts, and read the peak frequencies from the mode table.

2. **Refinement up-sweep** — ascending grid around the peak, PLUS the
   half-amplitude repeat of the peak dwell appended (`f@amp` with HALF
   the amplitude the survey ACTUALLY ran at that frequency — read it
   from the survey's dwell summary; a headroom-reduced survey
   amplitude makes it less than 0.075 Nm. An unreplaced placeholder
   refuses to run rather than dropping the dwell):

   ```sh
   ./build/apps/x7_ident --joint 1 --pose-first \
       --anchor-ref p1_j1_survey.csv.dwells.json \
       --freqs "3.75,3.9,4.05,4.2,4.35,4.5,4.65,4.8,4.95,5.1,5.25,<peak>@<half-amp>" \
       --label p1-j1-up --log p1_j1_up.csv
   ```

   (e.g. `4.5@0.075` — correct only if the survey ran 0.150 Nm at
   4.5 Hz.)

3. **Refinement down-sweep** — the same grid in DESCENDING order, a
   separate invocation:

   ```sh
   ./build/apps/x7_ident --joint 1 --pose-first \
       --anchor-ref p1_j1_survey.csv.dwells.json \
       --freqs "5.25,5.1,4.95,4.8,4.65,4.5,4.35,4.2,4.05,3.9,3.75" \
       --label p1-j1-down --log p1_j1_down.csv
   ```

The up- and down-sweep visits ARE the two measurements per frequency
(repeatability and hysteresis in one pass); the half-amplitude peak
point is the linearity spot-check — the analysis flags any FRF point
that moves.

## Thermal / voltage limits and cooldown

- **Pre-run gate** (checked before every invocation, and the
  between-invocation cooldown rule): every servo ≤ 55 °C, supply
  11.0–13.0 V — the app refuses otherwise; wait and rerun.
- **Per-cycle hard fault**: any servo ≥ 65 °C, or supply < 10.5 V /
  > 14.0 V.

## Abort semantics (what the operator does)

| outcome | meaning | action |
|---|---|---|
| `done` | all dwells processed | analyze; proceed |
| `GRACEFUL STOP at T_stop` | budget exhausted; remaining dwells not run | analyze what ran; split the rest into a new invocation |
| `ABORTED (soft-event recurrence)` | a response/clipping event survived its retry | inspect the CSV around the dwell; reduce `--amp` or drop that frequency |
| `ABORTED: <hard fault>` | thermal/voltage/gate/deviation/saturation/stale/latency/I/O | do NOT simply rerun — resolve the cause (cool down, re-place, inspect telemetry) |
| `SESSION WATCHDOG` message | bus silenced at T_quiesce; servo watchdogs halted the arm | treat as a hard fault; power-cycle if needed |

Every abort names its cause; every run leaves the full telemetry.

## Analysis and outputs

```sh
uv run --project tools tools/ident_analysis.py \
    p1_j1_survey.csv p1_j1_up.csv p1_j1_down.csv --out p1_j1
```

- Combines runs only when the recorded anchors agree within ±0.02 rad
  (the merge guard refuses otherwise), all probe the same joint, AND
  all ran the SAME recorded controller tuning — the FRFs are
  **controller-specific**: on a multivariable arm the other joints'
  coherent feedback torques are coupled inputs that the primary
  estimator's denominator does not contain, so a gain change changes
  the reported FRF. Every sidecar records the complete tuning
  (gains, scales, filter constants, and the EFFECTIVE frozen-integral
  mode) plus the ACTUAL frozen bias vector — run state, surfaced for
  repeatability comparisons (compare repeat-survey peaks alongside it;
  observed joint-2 bias varied −0.109 to −0.427 Nm across qualifying
  holds) but never a merge criterion. The mode table carries the
  tuning as the `controller` record; transferring results to a
  different tuning (e.g. x7_track's) must be demonstrated, not
  assumed. Dwell verdicts
  come from the sidecars: skipped or incomplete dwells are dropped,
  low-confidence dwells are excluded from the mode fits and flagged.
  A retried dwell contributes only its FINAL completed attempt (the
  `dwell_attempt` CSV column; the sidecar records the accepted
  attempt's reduced amplitude as `amp_eff_nm`).
- **Mode fitting refuses rather than fitting noise**: unless at least
  five dwells are both confident AND above the PER-DWELL
  observability floor on the demodulated probe-joint response, the
  analysis reports **INSUFFICIENT USABLE DATA** with the reasons and
  an empty mode list. The effective floor is run-specific:
  max(the analytic window-noise default 3.6e-5 rad — `--obs-floor-rad`
  overrides — and the run's own recorded lead-in `floor_q`); each
  dwell's response, effective floor, and their ratio are reported as
  `resp_probe_rad` / `obs_floor_rad` / `obs_snr` (`obs_snr` ≥ 1 plus
  a converged hold is the pilot-success criterion — see the campaign
  checklist). The dwell table and the actuator transfer are still
  emitted — they are valid measurements regardless. A tick-frozen
  probe joint (one encoder count per window) demodulates to numerical
  zero and never clears any floor: the 2026-07-27 P1 joint-1 survey
  was exactly this shape, and the modes its analysis would previously
  have fitted were numerical artifacts; the amplitude pilots that
  followed cleared the analytic floor at 0.25/0.30 Nm yet sat 10–30×
  below their run-measured floors — noise, correctly excluded.
- `p1_j1.mode_table.json` / `.md`: ONE entry per detected mode band —
  the 4–5 Hz and ~13 Hz bands when both appear; the ~13 Hz gear band
  may be weak or absent on joint 1 (probe joint 5 covers it) — fitted
  frequency and damping with grid confidence intervals, per-dwell FRF
  (τ_meas primary; the delay-corrected SUBMITTED TOTAL command
  secondary; their ratio = the actuator transfer for the notch phase
  budget), all-joint participation vector, fit residuals, and repeat /
  half-amplitude consistency flags. A mode is marked **refined** only
  with all three pieces of evidence — a grid fine enough for ζ ≈ 0.03;
  per-frequency full-amplitude repeat visits from two invocations
  whose recorded dwell order traverses the band in BOTH directions
  (one ascending, one descending — a duplicated same-direction sweep
  is rejected); and the half-amplitude linearity point — otherwise
  survey-only, with the missing evidence named.

## Preview first

The full pipeline runs against the two-mass fixture with planted
modes — validate any procedure change there before hardware:

```sh
./build/apps/x7_ident_sim --joint 1 --log ident_sim.csv
uv run --project tools tools/ident_analysis.py ident_sim.csv --out sim
```
