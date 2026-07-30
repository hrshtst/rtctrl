# Gravity-compensation current calibration — remediation plan

> The 2026-07-29 `x7_float` session is classified a FAILED hardware
> float test (external review of `float1.csv`). Two phases must be
> distinguished: the arm was placed at rest, but current-mode
> activation commands zero current, so the controller's FIRST feedback
> already showed a moving arm (phase 1 — the startup drop, an
> implementation gap, not itself evidence of miscalibration); the
> compensation then REVERSED that motion and accelerated the untouched
> arm toward the upright, higher-potential posture, peaking near
> 2.37 rad/s (phase 2 — strong evidence of excess actuation: with
> command scaling α > 1 the closed loop is M q̈ + C q̇ = (α−1) g(q)).
> Until this plan's hardware milestone passes, **every current-mode
> controller is PARKED**: `x7_track` (already parked — settle-phase
> incident), and now `x7_float` AND `x7_ident`, which share the same
> nominal torque-to-current conversion (M-GC0 records this in the
> operator-facing documents).

*Status: PROPOSED 2026-07-29; revised same day across three external
design-review rounds (round 1: preload route, single scaling
boundary, evidence honesty, displacement-bounded acceptance, parking
scope, config validation; round 2: M-GC3 calibrated-config mandate,
explicit conversion formula, logged release marker with numerical
drift bound, scale interval capped at 1.0; round 3: release marker
must PRECEDE the physical release, marker-anchored run termination,
execution-status correction). Reviewer-confirmed ready 2026-07-30.
**M-GC0: COMPLETE** (parking `cc38079`, analysis Addendum, telemetry
archived with manifest rows). **M-GC1: IMPLEMENTED** (commits
`1d463c8`, `2e31040`, plus the round-5 review fixes) — the shared
boundary with its [0.5, 1.0] validation and STRICT type parsing, the
vendor-scale config with a field-for-field drift guard, both
producers routed, the position-hold → preloaded-switch startup with
the 15 s minimum duration (round 6: window-completion is the ONLY
success verdict, so an outer deadline can never truncate an
evaluation into a false "done"; late-marker regression in the smoke)
and honest shutdown messaging, the marker with anchored
termination. **M-GC2's
automated gates are GREEN**, now including the INDEPENDENT vendor
fixture (revised gate above; round 6: post-hoc envelope labeling,
held-out evaluation, per-model current tolerances, reproducible
cross-checks): conversion parity 1e-4, model parity at seven
postures with the incident request reproduced exactly,
preload/cyclic continuity, the two-phase emulator float
(marker-timeout abort AND late-marker window completion,
tau_applied = scale × tau_request end-to-end), `gravity_sim_test`
unchanged. **M-GC1 and M-GC2 SIGNED OFF by the external reviewer
(2026-07-30)** — 186/186 tests and the strict docs build
independently reproduced. **M-GC3 acceptance run PASSED
(2026-07-30, `float2.csv` — see the Addendum): all four
preregistered conditions with wide margins.** Remaining in M-GC3:
the back-drive feel check, then the reviewer + owner decision on
default adoption and un-parking; the parking remains in force until
that decision.*

## Incident and evidence (`float1.csv`, 3001 cycles, archived — see [DATA_ARCHIVE.md](DATA_ARCHIVE.md))

- First ~1.2 s, operator not touching the arm: physical joint 2 moved
  −1.39 → +0.21 rad, joint 3 +1.78 → +2.31 rad, joint 4 −1.74 →
  ~0 rad, peak ~2.37 rad/s. Joints 3 and 4 then rode their upper
  limits (418 and 317 position-gated cycles). All 3001 submissions
  accepted; zero clamp events; the current loop tracked its command;
  no oscillation. The controller is exactly τ = g(q)
  (`arm::GravityComp`) — no PD, no damping.
- **What the log CAN and CANNOT show:** measured torque is
  reconstructed from current through the same nominal constant that
  produced the command, so the log identifies current-loop tracking
  only — it cannot independently estimate the output-torque gain α
  (that would need accelerations, coupled inverse dynamics, reflected
  inertia, and a friction model; the logged velocity is quantized and
  the limit-gated intervals are unusable). The per-second agreement
  print is blind to this failure class for the same reason.

## Root-cause ranking (external review)

1. **Torque→current calibration (hypothesis, vendor-derived).**
   rtctrl converts with the e-manual/SDK constants (1.783 Nm/A XM430,
   2.409 XM540); the vendor sample uses larger, empirically tuned
   constants (2.20, 3.60) — for identical model torque rtctrl
   commands ~23 % more current on XM430 joints and ~49 % more on the
   shoulder (e.g. 2.022 Nm on physical joint 2: 0.839 A vs 0.562 A).
   The corresponding command scales are the FULL vendor ratios,
   stated as hypotheses, not measurements:
   **1.783/2.20 = 0.810455 (XM430), 2.409/3.60 = 0.669167 (XM540)**.
2. Contact with the joint limits during this particular run.
3. Extra distal hand mass in the rtctrl model (the vendor's dynamics
   end at arm link 7, excluding the ~30 g hand assembly).
4. Ordinary uncompensated gearbox friction (present in both stacks).

Contributing implementation gap: the zero-current activation drop
(phase 1). Damping is deliberately NOT proposed — see Non-goals.

## Correction strategy (reviewer-directed)

Reproduce the vendor-equivalent per-model currents OFFLINE first, then
combine the corrected scaling with a gravity-preloaded startup, and
only then run a conservative, displacement-bounded hardware test.

## Milestones

### M-GC0 — status, evidence, and offline analysis (no hardware)

- **Park the current-mode controllers** in the operational documents
  (one page is never the sole authority): amend the HARDWARE_BRINGUP
  parked notice — it currently claims `x7_float` is unaffected — to
  park `x7_float` AND `x7_ident` alongside `x7_track` (x7_ident runs
  current mode through the same nominal conversion and builds its own
  preload; parking it costs nothing with the campaign closed). Scope
  the gravity-compensation theory chapter's hardware acceptance claim
  to the 2026-07 M7 sessions and add the dated failed-float status;
  add a PARKED banner to `apps/x7_float.cpp`'s operator-facing
  header. `x7_read`, `x7_onoff`, and the position-mode apps remain
  usable.
- **Archive** `float1.csv` per [DATA_ARCHIVE.md](DATA_ARCHIVE.md)
  (unique name, manifest row + SHA-256).
- **Offline analysis, scoped to what the data supports:** using ONLY
  the untouched pre-gate interval, document the two phases (the
  initial drop-seeded motion vs the reversal and upward acceleration)
  and the joint-2/3 gate windows. NO per-joint α estimates from this
  log (see Evidence above). The earlier M7 float pass is reconciled
  as a HYPOTHESIS (compact posture → smaller gravity torques →
  residual likely below breakaway), explicitly not as a conclusion
  from assumed residuals against assumed stiction.
- **Exit:** findings recorded as an addendum here before any code
  changes.

### M-GC1 — single scaling boundary + preloaded startup (code, reviewer sign-off before merge)

- **One shared torque→command-current mapping.** The conversion today
  lives inline in `RealArm::writeCommand` (`src/arm/real_arm.cpp`)
  and is DUPLICATED in x7_ident's manual preload construction
  (`apps/x7_ident.cpp`); `hw::CraneX7` is amp-valued throughout and
  is NOT where a τ→A scale belongs. Introduce
  `hw::commandCurrentFromTorque(joint_config, tau_nm)` implementing
  EXACTLY

  ```math
  i_{\text{cmd}} \;=\; s_{\text{command}} \cdot
  \frac{\tau_{\text{requested}}}{k_{t,\text{nominal}}}
  ```

  — the scale multiplies the RESULTING current, so
  s = 0.669167 REDUCES the commanded current (the ambiguous
  "kt × scale" phrasing could be read as scaling the denominator,
  which would do the opposite). Route BOTH call sites through it.
  CraneX7's explicitly amp-valued APIs and preload paths stay
  unscaled: the scale applies exactly once, at the torque boundary.
- **Config:** per-joint `command_torque_scale` in
  `config/crane_x7.toml`, default 1.0 (bit-identical behavior).
  Validation at `Config::load`, BEFORE any bus contact:
  strict-parsed, finite, inside the preregistered interval
  **[0.5, 1.0]** — non-finite, zero, negative, or out-of-interval
  values reject the config. The cap at 1.0 is deliberate: the
  incident is associated with EXCESSIVE current at scale 1.0, so this
  remediation permits attenuation only; widening beyond 1.0 requires
  an independently reviewed calibration demanding amplification. The
  vendor-equivalent test configuration is the checked-in
  **`config/crane_x7_vendor_scale.toml`** carrying
  0.810455 / 0.669167.
- **Preloaded startup for `x7_float`, via the existing feasible
  route** (the previously proposed read-then-`activate()` sequence is
  not implementable — feedback exists only after activation): adopt
  the x7_ident pose-first pattern. `x7_float` activates in POSITION
  mode (the arm is HELD — no drop at all), reads the held posture,
  refuses a start inside the soft-limit margin band, computes the
  scaled gravity currents through the shared mapping, and performs
  the in-place `CraneX7::switchToCurrentModeWithPreload()` with the
  background thread stopped, honoring both documented refusal
  outcomes (a pre-sequence refusal leaves the arm ACTIVE and HELD —
  the app deactivates and reports). The unsupported interval shrinks
  from "until the first controller command" to the switch's
  torque-off→torque-on span.
- **Run provenance and release marker in `x7_float`:** the app PRINTS
  the active per-joint command scales at startup and records them in
  the `--log` header, so every log carries its configuration; and it
  gains an explicit release marker — an operator keypress recorded as
  a `released` 0/1 column transition in the CSV. The marker is
  pressed WHILE STILL SUPPORTING the arm and acknowledged with a
  console cue; the operator releases ON the cue, so the logged marker
  slightly PRECEDES the physical release (conservative: the window
  can only include extra supported time, never miss initial motion —
  a marker pressed after release would let human reaction delay
  exclude the sub-second acceleration this incident showed).
  Marker-anchored termination: the run ends automatically 5 s after
  the marker (the complete evaluation window is always logged); if no
  marker arrives within 8 s of the mode switch, the app deactivates
  and reports an aborted test. The commanded duration is only the
  outer deadline.
- **Diagnostic side effect (document it):** with command-side-only
  scaling the agreement print stops being self-cancelling — it shows
  measured ≈ scale × predicted, making the configured scale visible
  on hardware for the first time.
- **Tests:** config validation (accept/reject matrix, fail before
  bus); default-1.0 bit-identity through the emulator;
  **preload continuity** — the staged preload current equals the
  first cycle's commanded current, continuous and scaled exactly
  ONCE (pty emulator regression covering both the RealArm path and
  the preload producer); the position-hold → switch startup and both
  refusal paths in the float smoke.

### M-GC2 — offline validation (exit gate for any hardware)

- Vendor parity, automated in TWO parts (revised per review — the
  original single "1e-4 at reference postures" gate conflated
  conversion parity with model parity and never exercised the vendor
  algorithm):
  1. **Conversion parity** (algebraic): the boundary reproduces the
     vendor's τ/kt_vendor within 1e-4 relative.
  2. **Model parity against an INDEPENDENT fixture**: the vendor's OWN
     gravity algorithm (samples03, in-tree; their link CSV, their
     hand-excluding recursion) executed at four reference postures —
     generator committed at `tests/fixtures/vendor_gravity_dump.cpp`,
     outputs frozen in `tests/unit/vendor_gravity_test.cpp`. The
     per-joint model difference is bounded by a **POST-HOC engineering
     envelope** — max(0.06 Nm, 9 % of |τ_rtctrl|), the 9 % sitting
     1.7 percentage points above the development-set maximum among
     percentage-controlled cells (7.3 %; floor-governed low-torque
     cells show larger percentages) —
     set from four DEVELOPMENT postures and then EVALUATED on three
     HELD-OUT postures (the P1 anchor, the tracking-acceptance goal,
     a moderate spread), where it holds with ≥ 11 % headroom
     (held-out maxima: 8.0 % relative, 0.128 Nm absolute at the
     highest-load posture). An earlier flat 0.10 Nm development
     envelope FAILED the held-out evaluation — precisely the failure
     mode a held-out set exists to catch, and why this bound is
     labeled post hoc, not preregistered. The differences are
     CONSISTENT WITH (not isolated to) the vendor's excluded ~30 g
     hand assembly and link-parameter differences; the vendor
     computes no gripper torque, so that row is rtctrl-only. The
     rtctrl model must reproduce the incident log's j1 request
     (2.0219 Nm) exactly at the float posture. Cross-checks are
     REPRODUCIBLE: the static-torque-sum check is embedded in the
     generator (aborts on divergence > 1e-4), and the FK agreement is
     a live unit-test assertion against frozen vendor link-8
     positions (≤ 0.1 mm at every posture).
- Emulator float end-to-end: preload present from the first cycle, no
  gate/clamp events from a mid-range posture, print ratio ≈ scale.
- Full ctest and strict docs green; `gravity_sim_test` unchanged —
  the scale is a hardware-config property (`SimArm` has no gearbox
  and never applies it).

### M-GC3 — conservative hardware float test (owner-run, reviewer-gated)

- Protocol per [HARDWARE_BRINGUP.md](HARDWARE_BRINGUP.md), plus:
  compact mid-range start posture (every joint clear of the margin
  band), arm lightly supported through activation and the mode
  switch, hands on, cutoff in reach, unique `--log` name. **Timing
  (matches the M-GC1 marker-anchored termination):** the marker must
  be pressed within 8 s of the mode switch (else the app aborts and
  the attempt is void); the evaluation window is the guaranteed 5 s
  after the marker; the app self-terminates at t_marker + 5 s; the
  commanded 15 s duration is the explicit OUTER deadline, not the
  run length.
- **Mandatory calibrated configuration:** the run is exactly
  `./build/apps/x7_float --config config/crane_x7_vendor_scale.toml
  --log <unique>.csv 15`. The startup print and the log header must
  show the vendor-equivalent scales (0.810455 / 0.669167 per model);
  a run whose log carries the default all-1.0 scales — the
  configuration of the FAILED run — is INVALID for M-GC3 regardless
  of its outcome.
- **Release procedure (marker FIRST — the order defined in M-GC1):**
  the operator supports the arm through the switch, presses the
  release key WHILE STILL SUPPORTING, and releases on the console
  cue; the marker therefore slightly precedes the physical release,
  so the window can only include extra supported time and can never
  miss the initial acceleration. The untouched evaluation window is
  [t_marker, t_marker + 5 s] with t_marker taken FROM THE LOG's
  `released` column transition, never from operator notes.
- **Acceptance — each condition fails INDEPENDENTLY:**
  1. peak per-joint speed in the window > 0.1 rad/s;
  2. per-joint displacement from the release posture > 0.05 rad at
     any point in the window (a bound on drift, not just speed);
  3. any position-gate or clamp event;
  4. sustained drift: any joint's |q(t_end) − q(t_end − 1 s)| >
     0.01 rad over the window's final second (position-based — the
     logged velocity is LSB-quantized).
  Then: back-drive feel checked in both directions on each arm
  joint. Any autonomous acceleration → power cut, stop, re-analysis.
- **Exit:** on a pass, reviewer + owner decide whether the calibrated
  scale becomes the default configuration and which parked apps are
  released (x7_float first; x7_ident and x7_track have their own
  gates), with the theory chapter and PARITY.md updated to record
  the calibration and its provenance.

## Addendum — M-GC0 offline analysis (2026-07-30)

Scoped to what `float1.csv` supports (canonical joint indices;
physical joint = canonical + 1). The untouched pre-gate interval is
**[0, 1.17 s)** — the first position-gate event ends it.

- **Phase 1 — the startup drop.** The controller's FIRST feedback
  already showed a falling arm: j1 −0.408 rad/s, j3 −0.144, j4
  −0.264 (current-mode activation commands zero current; the first
  compensation command followed ~26 ms later, j1 receiving
  +2.022 Nm). Phase 1 is an implementation gap — the arm was placed
  at rest, but the controller never saw it at rest — and is NOT
  itself evidence of miscalibration.
- **Phase 2 — reversal and climb (the excess-actuation evidence).**
  Within 60–90 ms of the first command the falling joints REVERSED
  (j1 at 0.06 s, j3 at 0.09 s) and then accelerated upward for over
  half a second against gravity: j1 +1.60 rad (−1.393 → +0.204,
  peak +1.77 rad/s at 0.60 s), j3 +1.76 rad (−1.743 → +0.012, peak
  +2.37 rad/s at 0.88 s), j2 +0.53 rad (peak +1.41 rad/s at
  0.22 s). Exact compensation could at most arrest the drop;
  sustained upward acceleration toward the higher-potential posture
  requires net torque beyond gravity.
- **Gate windows.** j3 rode its upper limit for 317 cycles across
  1.17–11.88 s (the climb's end); j2 for 418 cycles across
  16.51–20.68 s (during the subsequent hand exploration). Neither
  window is usable for dynamics analysis.
- **What is NOT claimed.** No per-joint α is estimated from this
  log: measured torque is reconstructed through the same nominal
  constant that produced the command, so the log proves current-loop
  tracking only. The 0.810455/0.669167 scales remain vendor-derived
  HYPOTHESES. The earlier M7 float pass is reconciled only as a
  hypothesis (compact posture → smaller gravity torques → any
  residual below breakaway); it is not established by this data.
- **Archive:** DONE 2026-07-30 — `float1.csv` (and the settle
  incident's `track9.csv`/`track9.csv.settle`) carry manifest rows
  with SHA-256 in [DATA_ARCHIVE.md](DATA_ARCHIVE.md),
  `incidents-2026-07/` section.

## Addendum — M-GC3 acceptance run (2026-07-30, PASSED)

Run: `x7_float --config config/crane_x7_vendor_scale.toml --log
float2.csv 15` from a compact posture; startup print confirmed the
vendor scales; the arm was held through activation and the preloaded
switch; marker at t = 4.080 s (pressed while supporting, released on
the cue); self-termination at 9.090 s; 911/911 submissions accepted;
telemetry archived (`gravity-calibration/float2.csv`,
[DATA_ARCHIVE.md](DATA_ARCHIVE.md)).

All four preregistered conditions PASSED, evaluated over
[t_marker, t_marker + 5 s]:

| condition | bound | measured (worst joint) |
|---|---|---|
| peak per-joint speed | ≤ 0.1 rad/s | 0.024 rad/s (one velocity LSB) |
| displacement from release posture | ≤ 0.05 rad | 0.0123 rad (j2, peaking at t = 6.72 s, mid-window) |
| gate/clamp events | 0 | 0 (window AND whole run) |
| final-second drift | ≤ 0.01 rad | 0.0015 rad (one position LSB) |

The console agreement print showed measured ≈ scale × predicted
throughout (j1 ≈ 0.67×, j3 ≈ 0.80×) — the calibrated scale visible on
hardware exactly as designed. Contrast with the 2026-07-29 failed
run: same controller family, same arm; the untouched arm previously
peaked at 2.37 rad/s and rode its limits — now the final-second
drift is within ONE position count and the total release-relative
excursion is EIGHT counts (12.3 mrad).

**Back-drive feel check (2026-07-30, `float3.csv` — not acceptance
evidence): OPEN.** The operator reported distinctly lighter motion.
The log corroborates bidirectional encoder motion on j0–j6 under
active compensation, with zero gates/clamps and 503/503 accepted
submissions. (The logged torque fields are the requested/applied
gravity compensation and the current-derived motor torque — they do
NOT measure the operator's external hand torque; the log
corroborates joint movement under active compensation, not the
hand's effort.) Because coupled arm motion cannot establish
deliberate per-joint feel, symmetry, or smoothness, the back-drive
REQUIREMENT — every arm joint, both directions — remains OPEN
pending explicit operator confirmation that (1) every joint j0–j6
was deliberately checked in both directions, (2) resistance felt
reasonably symmetric, and (3) there were no notchy spots away from
the limits and no autonomous motion; otherwise repeat runs cover the
unchecked joints (one joint per run, waiting ~1 s after the
`floating` prompt before pressing ENTER). Only with that
confirmation does the milestone reduce to the reviewer + owner
decision on default adoption and un-parking.

## Non-goals and cautions

- **No damping in the float.** Host-side damping walks into the
  delayed-D-path anti-damping that parked `x7_track`'s settle phase
  (4.33 Hz pan incident). The position-hold startup plus preload
  removes the kick instead; damping design stays out of scope.
- **No change to `dxl::torqueConstant`** or any raw-unit conversion —
  the calibration is an explicit, configured, command-side quantity
  with provenance, applied at exactly one boundary.
- The vendor-equivalent scales are HYPOTHESES calibrating RT's model
  on RT's arms: a proper per-joint calibration experiment (known mass
  at a known lever arm; breakaway asymmetry in both directions)
  belongs to the M7 friction/parameter identification follow-up and
  may refine or replace them.
- `x7_track` remains parked regardless (its settle-phase fix is a
  separate reviewer-gated plan); the 0.6 cap and the pass-2 closure
  are untouched.
