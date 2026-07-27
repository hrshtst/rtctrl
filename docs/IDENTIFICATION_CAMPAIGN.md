# Identification campaign checklist

The concrete run order for the pass-2 hardware campaign. The full
operator procedure, posture vectors, and abort semantics live in
[IDENTIFICATION_PROTOCOL.md](IDENTIFICATION_PROTOCOL.md); the design
rationale in [IDENTIFICATION_PLAN.md](IDENTIFICATION_PLAN.md).

## 1. Pre-flight (one-time)

- [ ] Build current `main`; `ctest` fully green,
      `uv run mkdocs build --strict` clean.
- [ ] Actuator power cutoff within reach; workspace clear. Current
      mode throughout — the arm free-falls until the gravity hold
      lands, exactly as x7_track.
- [ ] Optional dry run without the arm to see the output shape:
      `dxl_emu` in one terminal, then
      `x7_ident --port <link> --joint 1 --freqs "5,8"` in another.

## 2. First run: P1 posture, joint 1, survey

- [ ] Get the arm to the P1 vector. Easiest: let the servos do the
      placing — `x7_pose` moves there in position mode, holds, and
      goes limp on Enter WHILE YOU SUPPORT the shoulder and elbow
      (the arm drops the moment torque cuts; keep supporting it until
      x7_ident's gravity comp takes over):

  ```sh
  ./build/apps/x7_pose --posture config/postures/p1.json
  ```

      Hand placement of the limp arm remains the fallback ("roughly
      where it sat for pass 1"). Either way the settle gate and the
      ±0.02 rad anchor tolerance do the verifying — x7_ident is
      unchanged.
- [ ] Run WITH the checked-in P1 reference — without `--anchor-ref`
      nothing enforces the canonical posture and the first survey
      would silently establish an arbitrary anchor as "P1"
      (~2.7 min worst case including the setup allowance; the app
      refuses on its own — settle, soft-limit band,
      temperature/voltage, anchor, budget — and names the reason):

  ```sh
  ./build/apps/x7_ident --joint 1 --anchor-ref config/postures/p1.json \
      --label p1-j1-survey --log p1_j1_survey.csv
  ```

## 3. Analyze BEFORE any refinement (the protocol's hard gate)

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

## 4. Refinement sweeps (same posture)

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
  ./build/apps/x7_ident --joint 1 --anchor-ref p1_j1_survey.csv.dwells.json \
      --freqs "3.75,3.9,4.05,4.2,4.35,4.5,4.65,4.8,4.95,5.1,5.25,<peak>@<half-amp>" \
      --label p1-j1-up --log p1_j1_up.csv
  ```

- [ ] Down-sweep: the same grid DESCENDING, a separate invocation:

  ```sh
  ./build/apps/x7_ident --joint 1 --anchor-ref p1_j1_survey.csv.dwells.json \
      --freqs "5.25,5.1,4.95,4.8,4.65,4.5,4.35,4.2,4.05,3.9,3.75" \
      --label p1-j1-down --log p1_j1_down.csv
  ```

- [ ] Combined analysis (the merge guard confirms the posture held;
      the mode table should now read "refined" with tight CIs):

  ```sh
  uv run --project tools tools/ident_analysis.py \
      p1_j1_survey.csv p1_j1_up.csv p1_j1_down.csv --out p1_j1
  ```

## 5. Expand the campaign

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
