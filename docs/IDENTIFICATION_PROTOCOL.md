# Flexible-mode identification protocol

Operator procedure for the pass-2 stepped-sine identification campaign
([design record](IDENTIFICATION_PLAN.md)). The goal: per-posture tables
of the arm's flexible modes — the configuration-dependent ~4–5 Hz
whole-arm structural mode and the ~13 Hz gear mode — with frequency,
damping, and joint participation. **The 0.6 excursion-scale cap in
`x7_track` stands until the notch design consuming these tables
lands.**

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

Hand placement: deactivate (or run before activation), place the limp
arm by hand at the target vector, then let activation + gravity
compensation hold it. The strict settle gate verifies stillness; the
app refuses to probe unless the settled anchor matches the reference
within **±0.02 rad per joint** (`--anchor-ref`).

| posture | description | canonical vector [rad] |
|---|---|---|
| P1 | pass-1 anchor (proven) — `config/postures/p1.json` | `-0.357, -0.831, 2.126, -1.572, -2.456, -0.106, 0.563, -0.014` |
| P2 | track7/8 anchor (structural-mode posture) — `config/postures/p2.json` | `-0.377, -0.873, 2.174, -1.608, -2.264, 0.109, 0.561, 0.086` |
| P3 | extended endpoint region (nominal) | `0.00, -0.60, 0.00, -0.80, 0.00, -0.30, 0.00, 0.00` |
| P4 | near-horizontal max extension (nominal) | `0.00, -1.30, 0.00, -0.35, 0.00, -0.15, 0.00, 0.00` |

P1 and P2 have CHECKED-IN reference files: every P1/P2 run — including
the very first survey — passes `--anchor-ref config/postures/p1.json`
(resp. `p2.json`), so no run can silently establish an arbitrary
posture as canonical.

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

1. **Survey** — default grid, against the canonical reference:

   ```sh
   ./build/apps/x7_ident --joint 1 --anchor-ref config/postures/p1.json \
       --label p1-j1-survey --log p1_j1_survey.csv
   ```

   Analyze BEFORE refining: verify the achieved noise floor and dwell
   verdicts, and read the peak frequencies from the mode table.

2. **Refinement up-sweep** — ascending grid around the peak, PLUS the
   half-amplitude repeat of the peak dwell appended (`f@amp` with half
   the amplitude the survey table reports for that frequency):

   ```sh
   ./build/apps/x7_ident --joint 1 --anchor-ref p1_j1_survey.csv.dwells.json \
       --freqs "3.75,3.9,4.05,4.2,4.35,4.5,4.65,4.8,4.95,5.1,5.25,4.5@0.075" \
       --label p1-j1-up --log p1_j1_up.csv
   ```

3. **Refinement down-sweep** — the same grid in DESCENDING order, a
   separate invocation:

   ```sh
   ./build/apps/x7_ident --joint 1 --anchor-ref p1_j1_survey.csv.dwells.json \
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
  (the merge guard refuses otherwise) and all probe the same joint.
  Dwell verdicts come from the `.dwells.json` sidecars: skipped or
  incomplete dwells are dropped, low-confidence dwells are excluded
  from the mode fits and flagged.
- `p1_j1.mode_table.json` / `.md`: ONE entry per detected mode band
  (both the 4–5 Hz and ~13 Hz peaks from a single survey) — fitted
  frequency and damping with grid confidence intervals, per-dwell FRF
  (τ_meas primary; the delay-corrected SUBMITTED TOTAL command
  secondary; their ratio = the actuator transfer for the notch phase
  budget), all-joint participation vector, fit residuals, and repeat /
  half-amplitude consistency flags. A mode is marked **refined** only
  with all three pieces of evidence — a grid fine enough for ζ ≈ 0.03,
  visits from at least two invocations (up/down), and the
  half-amplitude linearity point — otherwise survey-only, with the
  missing evidence named.

## Preview first

The full pipeline runs against the two-mass fixture with planted
modes — validate any procedure change there before hardware:

```sh
./build/apps/x7_ident_sim --joint 1 --log ident_sim.csv
uv run --project tools tools/ident_analysis.py ident_sim.csv --out sim
```
