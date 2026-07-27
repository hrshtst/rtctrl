# Identification campaign checklist

The concrete run order for the pass-2 hardware campaign. The full
operator procedure, posture vectors, and abort semantics live in
[IDENTIFICATION_PROTOCOL.md](IDENTIFICATION_PROTOCOL.md); the design
rationale in [IDENTIFICATION_PLAN.md](IDENTIFICATION_PLAN.md).

## 1. Pre-flight (one-time)

- [ ] Build current `main`; `ctest` fully green,
      `uv run mkdocs build --strict` clean.
- [ ] Actuator power cutoff within reach; workspace clear. With
      `--pose-first` the arm starts servo-held in POSITION mode for
      the placement, then switches to current mode in-place (a ~25-
      transaction torque-off window with the gravity preload landing
      before re-enable). The manual fallback is current mode
      throughout and free-falls until the gravity hold lands.
- [ ] Optional dry run without the arm to see the output shape:
      `dxl_emu` in one terminal, then
      `x7_ident --port <link> --joint 1 --freqs "5,8"` in another.

## 2. Stabilize the anchor hold FIRST (required)

The first hardware capture (2026-07-27) exposed a **~8.9 Hz,
±13 mrad joint-0 limit cycle under the stationary anchor hold** at
full pan PD scale — the controller pumps it (+11 mW at the mode), and
its 0.34 Nm p-p torque exceeds the probe amplitude. Treat 8.9 Hz as a
CLOSED-LOOP limit-cycle candidate, not a confirmed mechanical mode; do
not add joint 0 to the probe set on this evidence alone.

Status after two hold sessions (2026-07-27): `--scale0 0.5` resolved
the joint-0 cycle (0.030–0.040 rad/s, slightly dissipative), but
**joint 2 shows integral-driven stick-slip**: stuck IN tolerance while
the integral winds at Ki·e (~0.13 Nm over seconds), then breakaway
slips ~26 mrad across the band. The reviewer-directed experiment is
the **frozen-integral hold**: the learned integral state freezes at
capture acceptance — never reset, so the captured friction/gravity
bias keeps acting — preventing further winding inside the band.

- [ ] Run the frozen-integral hold diagnostic at scale 0.5, UNIQUE log
      names per attempt:

  ```sh
  ./build/apps/x7_ident --joint 1 --pose-first \
      --anchor-ref config/postures/p1.json \
      --hold 30 --scale0 0.5 --freeze-i \
      --label p1-hold-s050-fz-r1 --log p1_hold_s050_fz_r1.csv
  ```

      The hold must COMPLETE: continuous stability — every joint
      inside ±0.02 rad and the unchanged 0.05 rad/s metric at every
      sample after a 1 s settling grace, all hard monitors live. The
      report lists each joint's max metric and p-p motion; the sidecar
      records the controller mode (`integral_frozen_at_capture`).
- [ ] Require THREE complete 30 s holds before selecting the tuning.
      If joint 2 still stick-slips with the freeze, the next
      alternatives (reviewer-sequenced) are a leaky/deadband
      integrator or posture-specific gravity/bias compensation.
- [ ] Reassess the isolated joint-6 ~4.4 Hz event only after joint-2
      winding is eliminated (one short occurrence is not evidence for
      tuning joint 6).
- [ ] Repeat the passing scale at least once at P1, and validate it at
      EVERY posture before that posture's surveys — P1 stability does
      not establish P2–P4 stability.
- [ ] The selected scale then becomes the ONE campaign tuning: report
      it for baking into the ident controller (a code change) — the
      FRFs are CONTROLLER-SPECIFIC (a multivariable arm: other joints'
      coherent feedback torques are coupled inputs), the full tuning is
      recorded in every sidecar, and the analysis refuses to merge
      runs under different tunings. Transferring the resulting mode
      tables to x7_track's scale-1.0 tuning must be demonstrated, not
      assumed — or the chosen scale promoted to x7_track as well
      (decide with the reviewer once the value is known).

## 3. First survey: P1 posture, joint 1

- [ ] One command does placement AND survey — `--pose-first` (the
      reviewer-approved integrated startup, after four hand-handover
      sessions failed the anchor gate): the app moves to P1 in
      position mode with measured-posture convergence, switches to
      current mode in-process with a gravity-current preload, captures
      the residual onto the canonical anchor under the restoring
      controller, and only then probes. No hands on the arm; the
      quiescence and ±0.02 rad gates still decide, and a
      post-transition displacement beyond the 0.08 rad envelope
      aborts:

  ```sh
  ./build/apps/x7_ident --joint 1 --pose-first \
      --anchor-ref config/postures/p1.json \
      --label p1-j1-survey --log p1_j1_survey.csv
  ```

      `--anchor-ref` is REQUIRED here (the placement target IS the
      canonical anchor) — and without it in any flow, nothing enforces
      the canonical posture. The app refuses on its own — placement
      envelope, capture admission, soft-limit band,
      temperature/voltage, budget — and names the reason.
- [ ] Fallback only (e.g. debugging the placement): hand placement of
      the limp arm plus `x7_pose` for rough positioning. Be aware the
      four trial sessions showed the limp handover loses 0.06–0.18 rad
      on the gravity-loaded joints — expect anchor-gate refusals.

## 4. Analyze BEFORE any refinement (the protocol's hard gate)

```sh
uv run --project tools tools/ident_analysis.py p1_j1_survey.csv --out p1_j1_survey
```

Verify four things:

- [ ] **Timing/command path clean** — no latency aborts, no
      saturation/gate flags during the run (the app aborts on these
      anyway; skim the summary).
- [ ] **Noise floors** — `floor_q`/`floor_tau` per dwell in
      `p1_j1_survey.csv.dwells.json`: response magnitudes at the
      interesting dwells sit well above them. This is the first
      measurement of the REAL floors (vs the analytic 3.6e-5 rad
      figure).
- [ ] **Dwell verdicts** — `done` on most dwells; hold timeouts
      (low-confidence) far off-resonance are normal, but
      low-confidence AT the peak means the amplitude or window needs
      revisiting.
- [ ] **Peak locations** — the mode table names a peak near 4–5 Hz
      (possibly the 13 Hz mode weakly from joint 1). It is marked
      SURVEY-ONLY: expected at this stage.

## 5. Refinement sweeps (same posture)

Same session or after cooldown — the pre-run gate enforces the
cooldown automatically; just rerun.

- [ ] Up-sweep: ascending grid around the survey peak, plus the
      half-amplitude repeat of the peak dwell appended. Replace BOTH
      placeholders from the survey's dwell summary: `<half-amp>` is
      half the amplitude the survey ACTUALLY ran at the peak (a
      headroom-reduced survey makes it less than 0.075 Nm). An
      unreplaced placeholder refuses to run rather than dropping the
      dwell. Refinement runs reference the survey's sidecar, not the
      canonical file — that keeps the combined dataset inside the
      merge guard (see the protocol's anchor-reference policy):

  ```sh
  ./build/apps/x7_ident --joint 1 --pose-first \
      --anchor-ref p1_j1_survey.csv.dwells.json \
      --freqs "3.75,3.9,4.05,4.2,4.35,4.5,4.65,4.8,4.95,5.1,5.25,<peak>@<half-amp>" \
      --label p1-j1-up --log p1_j1_up.csv
  ```

- [ ] Down-sweep: the same grid DESCENDING, a separate invocation:

  ```sh
  ./build/apps/x7_ident --joint 1 --pose-first \
      --anchor-ref p1_j1_survey.csv.dwells.json \
      --freqs "5.25,5.1,4.95,4.8,4.65,4.5,4.35,4.2,4.05,3.9,3.75" \
      --label p1-j1-down --log p1_j1_down.csv
  ```

- [ ] Combined analysis (the merge guard confirms the posture held;
      the mode table should now read "refined" with tight CIs):

  ```sh
  uv run --project tools tools/ident_analysis.py \
      p1_j1_survey.csv p1_j1_up.csv p1_j1_down.csv --out p1_j1
  ```

## 6. Expand the campaign

- [ ] Joints 3 and 5 at P1 (survey → analyze → refine, same pattern;
      each first survey gates on `config/postures/p1.json`).
- [ ] All three joints at P2 — each first survey gates on
      `config/postures/p2.json` — plus the P2 re-placement micro-test
      (repeat the survey after deliberately re-placing the arm) — this
      calibrates the ±0.02 rad tolerance empirically.
- [ ] P3, then P4: nominal vectors; the FIRST session's settled anchor
      (from the `.dwells.json` sidecar) becomes the reference for every
      later run there. Hand support ready for the first settles —
      these are the riskiest moments. Headroom-reduced amplitudes or
      skipped dwells near P4 are the system working, not a fault.

## Cautions for run 1

The first run is the first time the probe machinery touches hardware:
treat any soft event or hard fault as DATA — the telemetry around the
offending dwell says what happened. Do not simply rerun after a hard
fault; resolve the cause first (cool down, re-place, inspect the CSV).

## After the campaign

The per-posture mode tables (frequency, damping, participation,
actuator transfer) feed the notch/phase-compensated D-path design —
the next pass-2 step. The 0.6 excursion-scale cap in x7_track stands
until that design lands.
