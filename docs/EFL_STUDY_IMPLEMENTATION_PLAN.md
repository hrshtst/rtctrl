# Offline EFL study — implementation plan

> Companion to [ENVELOPE_STABILITY_PLAN.md](ENVELOPE_STABILITY_PLAN.md)
> (the reviewer-authored study definition). That document is the
> authority on the question, the preregistered gain grid, the case
> matrix, the metrics, and the decision rule; this one maps its
> milestones onto the repository and freezes the values the study
> definition names but does not number. Its constraints bind
> throughout: offline only, no hardware runs, `x7_track`/CLI/defaults
> untouched, experimental code outside the public API, no changes to
> the preregistered numbers.

*Status: PROPOSED, revised 2026-07-29 across three external
implementation-review rounds (round 1: four blocking findings, four
corrections; round 2: torque-limit source, executable verdict
formulas, modal censoring, untracked-file cleanliness; round 3:
censored-growth case, flexible-case coverage in C2/C3, exact
least-squares amplitude fit; round 4: explicit single-mode fixtures,
complete C4 comparator ordering, frozen development pose — all
integrated below). No milestone implemented.*

## Reuse map

- **Estimator parity.** `arm::ComputedTorque` exposes its host velocity
  estimate through the `velocityEstimate()` telemetry accessor, so the
  study's standalone estimator is verified against the shipped one on a
  deterministic position/timestamp trace (with dt jitter, a duplicate
  timestamp, and the first sample) rather than by shared code — as the
  study definition requires. Semantics to replicate: first sample
  records positions with a zero estimate and feedforward-only output;
  raw-dt backward difference; EMA `α = dt_f/(0.02 s + dt_f)` with
  `dt_f = min(dt, 3·nominal_dt)`; duplicate timestamps hold all state.
- **Delay wrapper.** `LaggedArm` (currently private to
  `apps/x7_track_sim.cpp`) already models command lag plus
  position/velocity LSB quantization; it is extracted and generalized
  (configurable delay depth, exact-velocity option, `arm::Arm&` inner).
  The tracking integration tests use bare `SimArm`, so they do NOT pin
  the extraction — new characterization tests do (see Tests).
- **Harness pattern.** The synchronous read→update→write→step loop of
  `trackingRms` in `tests/integration/tracking_sim_test.cpp`.
- **Disturbance.** `SimArm::setDisturbance(joint, torque)` for D+/D−.
- **P1 start pose.** `x7::loadAnchorRef` (`apps/ident_common.hpp`) reads
  `config/postures/p1.json`.
- **Round trip and baseline.** The excursion definition and
  `sqrt(amplitude)` duration rule are mirrored from `x7_track.cpp` with
  a provenance comment. The PRACTICAL baseline is the **shipped
  controller tuning with a frozen offline torque-limit vector**
  (hardware reads servo current limits at runtime and runs a settle
  phase; offline, the real limit formula is evaluated with the frozen
  assumed current-limit vector below, and there is no settle phase): a
  bare
  `ComputedTorque` configured with `kKp/kKd/kKi`, `kIntegralClampNm`,
  `kGainScale`, `kNominalDt`, and the frozen vector below. One shared
  `tau_max` vector, trajectory, duration, and scoring path for every
  controller in a case; the vector is recorded in every result row.
- **Flexible fixtures.** `TwoMassArm::plantMode`, constructed
  EXPLICITLY per case rather than from the fixture defaults (the
  defaults plant modes on joints 1 AND 5, which would leave unintended
  flexible modes coupled through the model-based controller's
  commands): every joint gets the stiff, well-damped parameters
  `plantMode(0.1, 0.05, 40.0, 0.5)`; F4 then replaces ONLY joint 0
  with `plantMode(0.4, 0.05, 4.5, 0.03)`; F13 replaces ONLY joint 5
  with `plantMode(0.01, 0.05, 13.0, 0.05)`. Exactly one flexible mode
  exists per screen. Gear deflection is seeded with the EXISTING
  `TwoMassArm::deflectGear(i, delta)` test hook — no fixture change is
  needed.
- **Mass-matrix action.** The two-`inverseDynamics`-call idiom
  (`apps/ident_common.hpp`) verifies the acceleration channel:
  `τ(v₁) − τ(v₂) = M(q)(v₁ − v₂)`.

## Preregistered constants (frozen now; code must match these numbers)

The study definition requires these fixed but leaves them unnumbered;
they are frozen here, before implementation, and baked into the study
executable as constants.

- **Torque-limit vector** [Nm], the REAL hardware rule
  `max(0, min(effort_limit, kt·I_servo) − current_limit_margin·kt)`
  evaluated with a frozen ASSUMED servo current-limit vector (offline
  there is no runtime reading), in raw units:
  `[1193, 2047, 1193, 1193, 1193, 1193, 1193, 1193]` — 1193 is the
  XM430-W350 factory default recorded in `src/emu/motor_emulator.cpp`;
  2047 is the XM540-W270 vendor factory default (an assumption, not
  repo-recorded). With the checked-in `config/crane_x7.toml`
  efforts/margins, `kCurrentUnitAmps = 0.00269`, and kt 1.783
  (XM430-W350) / 2.409 (XM540-W270):

  ```text
  [4.8305, 8.7955, 3.1085, 3.1085, 3.1085, 3.1085, 3.1085, 3.1085]
  ```

  Joint 0 is current-limit bound (kt·I = 5.7220 Nm < the 10 Nm
  effort limit). Both the assumed current-limit vector and the
  resulting torque limits are recorded in every result row.
- **Development-scenario start pose** (the authority's "existing
  rigid-sim start pose", frozen numerically — the
  `tests/integration/tracking_sim_test.cpp` initial pose):

  ```text
  [0.0, 0.2, 0.0, -0.4, 0.0, -0.2, 0.0, 0.1]
  ```

- **Gear deflection (F4/F13):** `deflectGear(probe_joint, 0.005)` —
  5 mrad motor-side against the held link, applied once before the
  first control cycle, identically for every compared controller.
- **Flexible-case run length:** 3.0 s. **Modal fitting window:**
  t ∈ [0.5, 2.5] s, split at 1.5 s into two equal halves.
- **Amplitude estimator:** fixed-frequency least-squares fit of
  `a·cos(2πft) + b·sin(2πft) + c` to the probe joint's link position
  over each window (f = the planted mode frequency), amplitude
  `A = √(a² + b²)` [rad] — exact for arbitrary phase and noninteger
  cycle counts, unlike a normalized Goertzel coefficient. Windows:
  `A_early` on [0.0, 0.5) s, `A₁` on [0.5, 1.5) s, `A₂` on
  [1.5, 2.5) s. **Censoring rule** (floor ε = 1e-9 rad): a below-floor
  amplitude is a censored observation, never a zero. Per screen and
  controller, exhaustively and mutually exclusively:

  - `A₁ ≥ ε ∧ A₂ ≥ ε` → uncensored rate `ln(A₂/A₁)/1.0 s`;
  - `A₁ ≥ ε ∧ A₂ < ε` → **censored-decayed**;
  - `A₁ < ε ∧ A₂ ≥ ε` → **censored-grown** (vanished, then
    reappeared): fails C4 outright;
  - `A₁ < ε ∧ A₂ < ε ∧ A_early ≥ ε` → **censored-decayed** (the ring
    existed and vanished below resolution);
  - `A₁ < ε ∧ A₂ < ε ∧ A_early < ε` → **inconclusive** (nothing
    measurable was excited): fails C4, recorded with its amplitudes.

  Censored-decayed satisfies the negativity requirement and compares
  as more negative than any uncensored rate; two censored-decayed
  results tie (which satisfies the no-greater-than-comparator test).
- **Delay-queue initialization (study mode):** `delay_cycles = 2`, the
  queue preloaded with two zero-current commands — the command accepted
  at cycle k is applied at cycle k+2, from the first cycle on, with no
  first-command passthrough. (Legacy mode — one-cycle delay with
  first-command passthrough — remains the default and is what
  `x7_track_sim` keeps using.)
- **D+/D− hold and scoring:** the loop runs for
  `trip.duration() + 3.0 s`, holding the trajectory's clamped endpoint
  sample as the reference after the trip ends (verify the endpoint
  clamp at implementation; otherwise hold the final sample explicitly).
  Settling time = first time after trajectory end at which the
  disturbed joint's |e| remains ≤ 0.01 rad continuously for 0.5 s;
  steady-state error = mean |e| over the final 1.0 s. Joint 1 (the
  disturbed joint) is scored; all joints are recorded.

### Frozen decision-rule formulas

The authority's four-part rule
([ENVELOPE_STABILITY_PLAN.md](ENVELOPE_STABILITY_PLAN.md), Metrics and
decision rule), made executable without judgment. Comparator:
PRACTICAL in the six rigid/delayed/disturbed cases
`S = {R1, R2, L1, L2, D+, D−}`; PRACTICAL-GF in `{F4, F13}`. Per case,
RMS is the total all-joint, all-sample tracking RMS; peaks are maxima
over all joints and samples; `sat(c)` is a controller's
controller-side clamp event count.

- **C1** (≥20% improvement in a majority):
  `|{c ∈ S : RMS_EFL(c) ≤ 0.8 · RMS_PRACTICAL(c)}| ≥ 4`.
- **C2** (no regression, required in EVERY `c ∈ T` where
  `T = S ∪ {F4, F13}`, against that case's comparator `cmp` —
  PRACTICAL in S, PRACTICAL-GF in F4/F13):
  `peak_err_EFL(c) ≤ 1.1 · peak_err_cmp(c)` and
  `peak_τ_EFL(c) ≤ 1.1 · peak_τ_cmp(c)` and
  `sat_EFL(c) ≤ sat_cmp(c)` (which implies no saturation where the
  comparator has none).
- **C3** (delayed completion): EFL-host completes L1, L2, D+, D−, F4,
  and F13 with finite states and metrics. A validly censored modal
  result counts as a COMPLETED metric (the classification is the
  result); its numeric rate is encoded as JSON `null` — never an
  infinity or a sentinel value.
- **C4** (flexible screens): for BOTH F4 and F13, EFL-host is
  uncensored-negative or censored-decayed, AND it is no worse than
  PRACTICAL-GF under the complete comparator ordering:
  comparator censored-decayed → only EFL censored-decayed is no worse;
  comparator uncensored → EFL censored-decayed passes, and EFL
  uncensored requires `rate_EFL ≤ rate_cmp`;
  comparator censored-grown → any uncensored-negative or
  censored-decayed EFL result is better;
  comparator inconclusive → the comparison is inconclusive and C4
  fails.
  EFL censored-grown or inconclusive fails C4 regardless of the
  comparator.

Verdict `promising = C1 ∧ C2 ∧ C3 ∧ C4`, computed by the executable
and written into the JSON as per-criterion booleans plus the
conjunction; prose interpretation stays in EFL-2.

## New and changed files

1. `apps/lagged_arm.hpp` — extracted `LaggedArm` with options
   `{delay_cycles (default 1), first_passthrough (default true — the
   legacy semantics), vel_tau, exact_velocity, quantize_pos}`. The
   study constructs it with `{2, false (zero-preloaded queue), …}` per
   the frozen rule above; `x7_track_sim` constructs it with defaults.
2. `apps/exact_feedback_linearization.hpp` —
   `HostVelocityEstimator` (standalone, parity-tested);
   `AccelDomainController` (evaluation mode Desired/Measured × velocity
   source host/state; `v = q̈_d + K_d′(q̇_d − v̂) + K_p′(q_d − q)`;
   Measured: `τ = ID(q, v̂, v)`, Desired: `τ = ID(q_d, q̇_d, v)`; one
   RNEA call; gravity-free option subtracting `gravityTorque` at the
   same evaluation point; torque clamp and saturation flags mirroring
   `ComputedTorque`; first-sample soft start; duplicate-timestamp
   re-emit; telemetry accessors). Covers DESIRED-host, EFL-host, and
   EFL-ideal.
   For the flexible cases the practical comparator is **PRACTICAL-GF**:
   an app-local replica of the practical law whose feedforward is
   `ID(q_d, q̇_d, q̈_d) − gravityTorque(q_d)`, so gravity is removed
   BEFORE the integrator's anti-windup and the final clamp (wrapping
   the shipped controller and subtracting afterwards would let
   anti-windup react to fictitious gravity, corrupt the saturation
   telemetry, and break the common limit). The replica in ordinary
   (gravity-on) mode is parity-tested against `ComputedTorque` —
   identical outputs, integral state, and saturation flags on a
   deterministic trace. R1–D− continue to use the real shipped
   `ComputedTorque` as PRACTICAL.
3. `apps/study_metrics.hpp` — pure-logic scoring: RMS/max error, RMS
   and peak commanded torque, saturation counts, settling time and
   steady-state error per the frozen definitions, fixed-frequency
   least-squares band amplitudes, and the modal rate/censoring
   classification (including censored-grown) per the frozen rules;
   JSON record emission.
4. `apps/x7_efl_study.cpp` — the study executable. All preregistered
   constants baked in (grid, development scenario, selection rule, the
   eight cases with the incident start vector, EFL-ideal only in
   R1/R2/L1/L2, the four-part decision rule emitted as machine-readable
   pass/fail). CLI: `--out <dir>` (default `build/efl_study/`) and
   `--case <id>` for smoke testing — no tuning knobs.
   **Provenance and canonicality:** a build-time generated source
   (custom command re-run on every build) embeds the FULL commit SHA
   and dirty flag — not configure-time, not abbreviated. Worktree
   cleanliness is defined as EMPTY output of
   `git status --porcelain=v1 --untracked-files=normal`: tracked
   modifications AND untracked files both dirty the tree (an untracked
   experimental source must never yield a nominally canonical result);
   the build-time dirty flag and the startup re-check share this
   definition, so these plan documents themselves must be committed
   before any canonical run. At startup, before creating any output,
   the executable checks that the embedded SHA equals the current
   `HEAD`, that the worktree is clean by that definition, and that the
   invocation is a complete unfiltered run; after the run it checks
   that every expected cell is present. Any failed check marks the
   output noncanonical with the reason recorded in the JSON. `--case`
   output is therefore ALWAYS noncanonical; only a complete, clean-tree
   matrix may be promoted to `data/efl_study/results.json`.
5. Tests —
   `tests/unit/lagged_arm_test.cpp`: an extraction-equivalence
   characterization of the legacy one-cycle semantics (impulse sequence
   asserting exactly which command is applied at each step, including
   the first-command passthrough); the same impulse-schedule assertion
   for study mode (zero-preloaded, k → k+2); position/velocity
   quantization; the exact-velocity option.
   `tests/unit/efl_test.cpp`: estimator parity; zero-error ⇒
   pure-feedforward identity; acceleration-channel linearity;
   PRACTICAL-GF replica parity in ordinary mode; gravity-free zero
   torque at rest; clamp parity; duplicate-timestamp behavior.
   `tests/unit/study_metrics_test.cpp`: synthetic sinusoid →
   least-squares amplitude (exact recovery at arbitrary phase and
   noninteger cycle counts); planted exponential envelopes → fitted
   rate (both signs, plus the censored-decayed, censored-grown, and
   inconclusive classifications); settling metrics; the decision-rule
   booleans on synthetic result sets (each criterion flipped
   independently).
   Integration smoke (`LABELS integration`): run one case end-to-end
   to well-formed, explicitly noncanonical JSON.
6. `data/efl_study/README.md` and, after the run, the canonical
   `data/efl_study/results.json` (invocation, full revision + dirty
   status, build configuration, the frozen constants above including
   the torque-limit vector, full grid and selected gains, every case
   with metrics and pass/fail, schema version). `data/README.md` gains
   the distinction between simulation-study results and hardware
   evidence sidecars; [DATA_ARCHIVE.md](DATA_ARCHIVE.md) stays
   hardware-specific.
7. `docs/theory/computed-torque.md` (EFL-2) — the offline-results
   section: rigid, delayed/disturbed, and flexible findings stated
   separately, negative results included, linked to the tracked
   evidence; the study definition's *Status* line updated. No other
   claim changes.

## Milestone → commit mapping

1. EFL-0: `refactor(apps): extract the sim lag model into a reusable
   header` — with the characterization tests above; `x7_track_sim`
   behavior unchanged.
2. EFL-0: `feat(apps): EFL study controllers, estimator parity, and
   metrics` — items 2–3 with their unit tests.
3. EFL-0/1: `feat(apps): x7_efl_study preregistered study runner` —
   item 4, the smoke test, the build-time revision generation.
4. EFL-1: run the executable from a clean tree; `data: canonical
   offline EFL study results` — item 6.
5. EFL-2: `docs(theory): offline EFL study findings; plan status` —
   item 7.

## Verification

- Every code commit: `cmake --build build -j` and
  `ctest --test-dir build --output-on-failure` (new unit and smoke
  tests included).
- EFL-1 exit: every planned cell holds a result or an explicit,
  reproducible failure in `results.json`; the grid is reported in
  full; the canonicality checks pass (clean tree, HEAD match, complete
  matrix).
- EFL-2 exit: full ctest pass and `uv run mkdocs build --strict`.
- Guardrails: no edits to `x7_track.cpp`, `crane_x7_tuning.hpp`, safety
  gates, or the study definition's preregistered numbers; its stop
  conditions and no-hardware-authorization language remain verbatim.
