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

**QUALIFIED at P1 (2026-07-27)**: three consecutive complete 30 s
frozen-integral holds at joint-0 scale 0.5. The selected tuning is now
the PRODUCTION DEFAULT for every pose-first/capture-carrying
identification run (surveys included; the manual fallback has no
capture, so its integrator stays live — see step 3; x7_track's
shipped tuning unchanged): joint-0 PD scale 0.5
(`kIdentScale0`) + per-joint integral latching during capture
(each joint freezes at its own readiness; all-latched at admission),
persisting uninterrupted through probing. The sidecar
records the full tuning AND the actual frozen bias vector (it varied
−0.109 to −0.427 Nm on joint 2 across qualifying holds — compare
repeat-survey peaks alongside it). The 8.9 Hz joint-0 signature stays
a CLOSED-LOOP limit-cycle candidate, not a confirmed mechanical mode;
the isolated joint-6 ~4.4 Hz event did not recur in 3×30 s.

- [ ] EQUIVALENCE CHECK before the first survey: one 30 s P1 hold
      through the production path (no diagnostic overrides — this
      validates exactly the code path the survey uses):

  ```sh
  ./build/apps/x7_ident --joint 1 --pose-first \
      --anchor-ref config/postures/p1.json \
      --hold 30 --label p1-hold-prod-r1 --log p1_hold_prod_r1.csv
  ```

      It must COMPLETE under the unchanged continuous gates. Then
      proceed to the survey.
- [ ] At every NEW posture, before that posture's surveys: the same
      plain `--hold 30` validation (P1 stability does not establish
      P2–P4 stability). `--scale0` remains available for stabilization
      experiments if a posture fails.

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
      the limp arm plus `x7_pose` for rough positioning. Two cautions:
      the four trial sessions showed the limp handover loses
      0.06–0.18 rad on the gravity-loaded joints (expect anchor-gate
      refusals), and a manual-flow run has NO capture phase, so its
      integrator stays LIVE — the sidecar records that effective mode
      and the analysis refuses to merge it with frozen-integrator
      pose-first data. Manual-flow surveys are not campaign data.

## 3a. P1 joint 1: NOT IDENTIFIABLE with the stationary probe
      (2026-07-27 — campaign paused for a protocol-design decision)

The first valid-procedure survey (`p1_j1_survey_r2.csv`) was
mechanically flawless but **scientifically null**: joint 1 held a
single encoder count through 13 of 16 measurement windows while the
actuator faithfully applied the 0.05–0.15 Nm probes. The
reviewer-directed amplitude pilot then stepped one 4.5 Hz dwell
through 0.20 / 0.25 / 0.30 Nm (the unchanged hard cap) — clean runs,
torque delivered faithfully — and the joint-1 response stayed
noise-dominated at every amplitude: 22 / 68 / 38 µrad against
run-measured noise floors of 423 / 733 / 1051 µrad (response/floor
0.05 / 0.09 / 0.04), non-monotonic in amplitude, phase-incoherent,
raw motion 1–2 ambient encoder counts, all three adaptive holds
timed out. **The documented stop condition is reached: joint 1 at P1
is not identifiable with this stationary small-signal current-probe
method.**

Standing consequences (reviewer-confirmed):

- The three pilot runs are preserved as null-result evidence; they
  never enter mode fitting.
- No further joint-1/P1 surveys with this excitation; no amplitude
  above the 0.30 Nm hard cap; no refinement sweeps on null data.
- Confidence and observability thresholds are NOT widened to admit
  this data.

**Pilot success definition** (applies to every future excitation
pilot): a pilot dwell succeeds only when BOTH hold — (a) the
adaptive hold CONVERGED (the dwell is not low-confidence), and
(b) the demodulated probe-joint response clears the RUN-SPECIFIC
observability floor — max(the 3.6e-5 rad analytic floor, the run's
recorded `floor_q`) — i.e. the analysis's reported `obs_snr` ≥ 1.
Merely exceeding the fixed analytic floor is NOT success: the
0.25 Nm pilot did, at 0.09× its actual run floor.

- [ ] Next: a protocol-design decision (reviewer), not another retry.
      Plausible directions on the table: a posture with lower joint-1
      stiction/load; probing a different joint with measurable output
      motion (joints 3/5 are the other planned probe joints); a
      controlled motion/dither excitation identifying around a moving
      operating condition; or external link-side sensing if the
      flexible response is invisible to the servo-output encoder.

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
      revisiting. The analysis REFUSES mode fitting ("INSUFFICIENT
      USABLE DATA") unless at least five dwells are confident and
      above the observability floor on the probe-joint response — a
      refusal means the survey must be diagnosed (amplitude,
      posture), never that its noise should be fitted.
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
