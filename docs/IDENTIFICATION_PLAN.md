# Pass 2, step A — Flexible-mode identification protocol

> Deliverables: probe app + sim twin, offline analysis, two-mass validation fixture,
> safety design, protocol document. The notch/phase D-path redesign is a LATER step that
> consumes this step's mode tables. The 0.6 scale cap stands throughout. *(Closure
> 2026-07-28: no mode tables were produced — the campaign stopped at its first gate and
> the cap is final; see the Closure section.)*

*Status: **CLOSED 2026-07-28 — null result; the 0.6 scale cap is the
final supported boundary** (see the Closure section at the end).
Planned 2026-07-23, revised through nine design-review rounds,
reviewer-accepted, implemented, and executed on hardware 2026-07-27.
Pass 2 step A of [REMEDIATION_PLAN.md](REMEDIATION_PLAN.md).*

## Context

Pass 1 (docs/REMEDIATION_PLAN.md, complete and hardware-validated 2026-07-23) made the
100 Hz torque loop observable: measured time, sequenced feedback/commands, receipt-matched
latency, full-loop telemetry. Pass 2 begins by identifying the arm's flexible modes — the
configuration-dependent ~4–5 Hz whole-arm structural mode and the ~13 Hz gear mode — as
functions of posture (frequency, damping, joint participation), the data the notch design
requires. User decisions: postures are HAND-PLACED (limp arm positioned by the operator;
activation + gravity comp holds it — proven by x7_float; the strict settle gate verifies
stillness; this deviates from the plan sketch's "position mode" because a position→current
mode switch requires a torque-off instant at a gravity-loaded pose); scope is
identification ONLY.

## Method (validated design)

**Excitation**: stepped-sine torque probe on ONE joint at a time, superposed on the
proven anchor stabilizer — a ComputedTorque holding a CONSTANT trajectory at the settled
anchor with the SHIPPED tuning unchanged (Kp6/Kd1/Ki6, PD LP 0.05; the integrator is what
pins the posture — Ki=0 would sag ~0.15 rad and trip the deviation abort). Probe torque is
added after the inner update; the sum re-clamps to the hardware τ_max with a
`probe_clipped` flag.

**Primary estimator — direct ratio, no de-embedding** (review correction C2): the D8
telemetry logs the TOTAL applied torque, so the plant FRF is F{q_link}/F{τ_total} per
dwell — a closed loop with known injection needs no controller model when both plant
input and output are measured. Two variants: (a) τ_meas-based (measured current × kt,
sampled at the same feedback events as q — perfectly aligned; PRIMARY); (b)
commanded-based, phase-corrected by the receipt-map `first_apply_delay`. The ratio
(b)/(a) is itself a deliverable: the commanded→measured actuator transfer for the notch
phase budget. The closed-loop q/probe FRF is also kept (SNR + online-monitor
consistency).

**Dwell state machine** (per probe joint, inside ONE `arm::run()` — per-dwell run()
invocations are broken: run() re-latches its origin, ComputedTorque sees dt ≤ 0 and
freezes; C1): lead-in hold of max(2 s, 4 periods of the lowest scheduled
frequency) — 2 s on the default grids; the fresh integrator converges onto the
friction offset and the unforced data calibrates the noise floors (below) →
per frequency: 0.5 s half-cosine amplitude ramp-in → **ADAPTIVE pre-measure hold** (the
onset-excited free component decays with 1/(ζω₀) ≈ 1.2 s at 4.5 Hz/ζ0.03; a fixed 1 s
would leave ~43% of it in the window and bias the fitted damping — review round 2):
hold until two consecutive 1-period online I/Q block estimates agree —
`|Z_k − Z_{k−1}| / max(|Z_k|, |Z_{k−1}|, Z_floor) < 0.1` — required on BOTH signals
with SIGNAL-SPECIFIC floors (round 4): `Z_floor,q` [rad] for the probe-joint
response and `Z_floor,τ` [Nm] for the measured probe-frequency torque, each
calibrated per run from the UNFORCED lead-in (probe off): per scheduled frequency
the lead-in is partitioned into consecutive NON-overlapping 1-period blocks, the
same 1-period LS estimator runs on each block, and the floor is **3× the RMS of
the block |Ẑ| magnitudes**, minimum 4 blocks — guaranteed by sizing the lead-in
as **max(2 s, 4 periods of the lowest scheduled frequency)** (= 2 s for the
default ≥2 Hz grids; too few blocks for a robust percentile, hence RMS) — an
empirical per-frequency, per-signal floor, not derived from the 3 s position
figure, AND `|Z| ≥ Z_floor`
for each signal (two near-zero noisy blocks agreeing accidentally is not
convergence) — bounded to [1 s, 4 s] (~3.3 decay constants); timeout marks the
dwell low-confidence. Synthetic tests plant a decaying transient plus noise and
assert no premature acceptance (round 3) → measurement window of an integer number
of probe PERIODS, ended near the accumulated-phase 2πn crossing, minimum
**max(10 periods, 3 s)** →
0.5 s ramp-out → next frequency. Run duration = schedule + 10% slack; the controller
holds the anchor after completion; runner ends on measured time.

**Demodulation / estimation**: probe phase accumulated from MEASURED dt (φ += ω·Δt
from state.t); the probe is generated from φ. Per-dwell estimation is a **dt-weighted
least-squares fit on the regressors [1, t, sin φ, cos φ]** — orthogonality is NOT
assumed (a sampled phase "crossing" does not land exactly on 2πn under jitter, so
plain correlation after detrending would not be exactly orthogonal; the LS fit
absorbs offset and drift explicitly and is exact for any window end). The window
still ends near the phase crossing for conditioning. Noise floor ≈ 3.6e-5 rad at 3 s
windows (encoder quantization σ≈4.4e-4 rad, dither-linearized).

**Amplitude rule** (per dwell, computed before the run, printed in the schedule):
`A(f) = clamp(x_t·Ĵ·ω², 0.05 Nm, A_cap)` with target response x_t = 0.005 rad (~3.3
LSB), Ĵ = diagonal inertia at the anchor from two `inverseDynamics` calls, A_cap
default 0.15 Nm (CLI-raisable to the 0.3 Nm hard cap), floor 0.05 Nm ≈ 10 torque LSB.
Cap-bound dwells (tilt above ~1.4 Hz) get a predicted-SNR gate: if x̂ = A/(Ĵω²) < 3×
the window floor and f is not within ±30% of the expected mode, extend the window ×4 or
mark low-confidence. The SDOF fit is dominated by the resonance band where response is
×Q higher (~8 mrad at 4.5 Hz for 0.15 Nm).

**Frequency plan**: survey grid {2, 3, 3.5, 4, 4.5, 5, 5.5, 6, 7, 8, 10, 12, 13, 14,
16, 20} Hz, then a mandatory operator-driven REFINEMENT run via `--freqs` around each
found peak (±0.75 Hz in 0.15 Hz steps at 4–5 Hz; ±1.5 Hz in 0.3 Hz steps at 13 Hz) —
the 0.5 Hz survey grid cannot resolve ζ≈0.03 (half-power width ~0.27 Hz; C4). Damping
comes from a complex SDOF fit on the FRF points, never half-power on the survey grid.
**Refinement evidence requirements** (rounds 2–4): each frequency is measured twice
— the UP-sweep and DOWN-sweep visits ARE the two measurements (repeatability AND
hysteresis; no additional immediate repeat — that double-counting broke the 180 s
session budget). The two sweeps run as SEPARATE app invocations (each fits the
budget on its own; see the session-budget partitions), paired for analysis through
the anchor-reference gate; the peak dwell additionally repeats once at HALF
amplitude (gear compliance/backlash are amplitude-dependent — a linearity
spot-check, flagged in the output when the FRF point moves); the analysis reports
per-dwell fit residuals and confidence intervals, and a mode-table entry without
refinement data is marked survey-confidence only.

**Posture identity** (rounds 2–3): hand placement is not reproducible by itself, and
two different anchors are not one FRF dataset. The protocol doc pins CANONICAL joint
vectors for P1–P4 with an initial **±0.02 rad** per-joint tolerance (~1.1° — start
strict, relax only from evidence: ±0.05 rad ≈ 3°/joint could shift a narrow 4.5 Hz
mode across a 0.15 Hz refinement grid). P1 = pass-1 anchor, P2 = track7/8 anchor,
P3/P4 nominal. The app takes `--anchor-ref <summary.json>` (or the documented
vector): after settling it compares the settled anchor per joint and REFUSES to run
outside tolerance, printing the deltas. The analysis refuses to combine runs whose
recorded anchors differ beyond tolerance AND flags repeat runs whose fitted peak
shifts by more than the fit's confidence interval even when the anchors pass — the
protocol includes a posture-sensitivity micro-test (P2 survey repeated after
deliberate re-placement) to calibrate the tolerance empirically.

**Ring-down: REMOVED from this step** (round 2): a constant probe offset is
progressively cancelled by the shipped Ki=6 integrator, so the release excites the
controller's stored integral state rather than a clean two-mass decay, and the
intended 0.05 rad deflection exceeds the 0.03 rad response cap. If wanted later it
becomes its own protocol with a controlled integrator freeze/reset, its own caps and
ramps, and an explicitly closed-loop model.

## Safety (delta over inherited pass-1 aborts)

Inherited unchanged: strict settle gate (no override), soft-limit start guard, latency
verifier, stale-feedback policy, write/read-failure escalation, abort-then-deactivate
ordering.

**All monitors are ALL-JOINT scoped** (round 2 — the target is a COHERENT whole-arm
mode; excitation at joint 1 can produce the larger response or torque on joint 3):
- online demodulated response amplitude cap 0.03 rad and 1.5× second-half growth
  check on EVERY controlled joint, per dwell;
- deviation-from-anchor abort at 0.08 rad on every joint (whole run);
- controller/hardware saturation flags and position-gate flags on any joint end the
  dwell;
- per-dwell headroom precheck τ_max − |τ_anchor,measured| ≥ A + 0.2 Nm on EVERY joint
  (the anchor PD on non-probe joints also draws torque; binds at near-horizontal tilt
  — reduce A or skip and report).
**Fault taxonomy (round 3 — hard faults are NOT retryable dwell failures)**:
- **HARD faults → abort the run and deactivate immediately** (cooling or operator
  intervention may be required; automatic re-settle/retry is unsafe): thermal or
  supply-voltage limit, position-gate engagement on any joint, anchor deviation
  ≥ 0.08 rad, controller/hardware saturation on any joint, stale-feedback policy
  trip, latency violation, any I/O failure. Same abort-then-deactivate ordering as
  x7_track.
- **SOFT dwell events → emergency probe removal, re-settle, at most ONE
  reduced-amplitude retry**: response-cap exceedance (0.03 rad demodulated on any
  joint), 1.5× second-half growth on any joint, ≥5 consecutive probe-clipped cycles.
  A second soft event on the same dwell aborts the run.
**Emergency probe removal** (distinct from the ordinary 0.5 s dwell ramp-out, which
would continue exciting a growing mode for 2.25 cycles at 4.5 Hz / 6.5 at 13 Hz): a
50 ms half-cosine kill ramp — ≤ 0.23 cycles at 4.5 Hz, ≤ 0.65 at 13 Hz, torque step
bounded by the (small) probe amplitude — followed by the anchor hold and settleArm.
Unit-tested as its own path.

**Thermal / supply supervision** (round 2 — sessions are materially longer than
x7_track; P3/P4 are gravity-heavy): the grouped feedback already carries per-servo
temperature and voltage (`dxl::Feedback`), but `ident_common` must stay
hardware-free and testable — a **`x7::HealthProvider`** (injected callable returning
per-joint {temperature °C, voltage V}) is the interface (round 3): the hardware app
adapts `CraneX7::lastFeedbackStamped()` (slow signals; snapshot coherence not
required), the sim twin supplies constants, tests script faults. Per-joint temp/volt
columns land in the ident CSV. Limits: PRE-RUN gate — every servo ≤ 55 °C and
11.0–13.0 V, else refuse (doubles as the between-run cooldown rule); PER-CYCLE hard
fault — any servo ≥ 65 °C, or voltage < 10.5 V / > 14.0 V.

**Session duration budget (rounds 3–8)**: a shutdown reserve and three boundaries,
consistent by construction:
- **Shutdown reserve and deadline ladder (rounds 8–9)**: the target is stated
  at the SERVOS — every servo stops by 180.0 s — as a CONSERVATIVE DESIGNED
  BOUND with stacked allowances, not a mathematically exact deadline (servo
  watchdog tick tolerance, host–servo clock drift, and driver behavior are not
  characterized; the physical actuator cutoff remains the true last line).
  Working backwards: **T_quiesce = 180 − B_io − B_sched − B_servo_wd −
  B_guard = 179.5 s**, with **B_io = 50 ms** an upper bound on the longest
  single transaction (derived from the SDK's own packet-timeout formula —
  ~34 ms fixed latency allowance + byte time, more than a whole 10 ms cycle,
  which is why a quiesce request cannot stop an in-flight transaction —
  asserted by a FORCED full sync-read timeout in the emulator test),
  B_sched = 50 ms scheduling margin, B_servo_wd = 100 ms (the reg-98 watchdog
  interval), and **B_guard = 300 ms residual margin (round 9)** absorbing the
  uncharacterized tolerances above — negligible against a three-minute
  session. The graceful path must finish deactivation by T_quiesce: reserve
  2.0 s (0.5 s ramp-out + 1.5 s deactivation allowance) → **graceful-stop
  threshold T_stop = 177.5 s**. Reaching T_stop triggers graceful early
  termination — ramp out, anchor hold, deactivate, remaining dwells reported
  as not-run.
- **Designed servo-stop bound at 180 s (rounds 6–9) — enforced
  by an independent session watchdog through a purpose-built quiesce request,
  never by re-entering the shutdown path.** The existing escalation
  (`escalate()` → `deactivate()` → port close) is NOT usable from a second
  thread: it would concurrently join the same background std::thread, write
  through the same PacketIO, and race `Port::close()` (non-atomic `open_`, no
  locking) against in-flight SDK transactions. Instead, a dedicated watchdog
  thread — armed at activation on the HOST STEADY clock (independent of the
  feedback path by design), disarmed when `deactivate()` returns — fires at
  activation + T_quiesce and does exactly one thing: sets a new atomic quiesce
  flag (`CraneX7::requestQuiesce()` — idempotent, callable from any thread)
  and reports the cause. **The flag gates EVERY bus-producing path (round 8)**:
  the background thread checks it before each read, between the read and write
  halves of a cycle, and before each write (writes AND reads cease — the servo
  Bus Watchdog counts reads too), without being joined; and `deactivate()`
  itself re-checks it after every in-flight transaction and, once quiesced,
  issues NO further bus operations — its quiesced cleanup only stops/joins the
  thread, marks the session inactive, and closes the port after all in-flight
  I/O has returned (single-threaded by then, so the close races nothing). This
  covers both orderings: a watchdog firing while the app is mid-`deactivate()`
  silences the remaining ~24 writes, and an `arm::run()` that returns first
  cannot restart traffic — and reset the servo watchdog — through its ordinary
  `deactivate()`. No further host action is needed: bus silence makes the
  servo-side Bus Watchdog (reg 98, 0.1 s, armed at activation, firmware ≥ 38
  enforced) stop every servo within 0.1 s regardless of host state; the app
  thread, whose individual SDK transactions terminate on bounded packet
  timeouts, unblocks on its own and performs the quiesced cleanup AFTER it
  does. A host hung inside a kernel write is
  already bus-silent — same servo-side guarantee — and the operator's
  independent actuator-power cutoff, mandated within reach, remains the final
  physical mechanism exactly as crane_x7.hpp documents.
- **Pre-activation check on the BASE schedule**: a 15 s setup allowance (settle
  timeout + gates; round 6) + lead-in + all ramps + MAXIMUM adaptive holds (4 s)
  + minimum windows + 10% slack must fit ≤ T_stop, else the schedule is refused
  and the operator splits it. Extensions and retries are NOT pre-reserved
  (reserving one retry per dwell plus every ×4 extension would refuse even the
  advertised schedules); they are admitted at runtime:
- **Post-settle admission (round 6)**: the deadline is activation-to-
  deactivation, and gravity hold, settling, and the gates all spend it — so
  immediately before the first dwell the check repeats against the ORIGINAL
  activation timestamp: the remaining time to T_stop must still cover the
  worst-case base schedule, else no probe starts and the session deactivates
  gracefully. This check, not the pre-activation allowance, is authoritative.
- **Runtime admission**: a ×4 window extension or soft-event retry is granted ONLY
  while worst-case remaining time — elapsed since activation + the rest of the
  base schedule at maximum holds + the requested extension/retry — stays inside
  T_stop; otherwise it is skipped and the dwell is marked accordingly.
**Mandatory partitions in the protocol** (worst-case BASE arithmetic against
T_stop = 177.5 s, per dwell = 0.5 s ramp-in + 4 s max hold + max(10 periods, 3 s)
window + 0.5 s ramp-out, + the 15 s setup allowance): one survey = one invocation
(16 dwells: 2 Hz→10 s, 3 Hz→8.3 s, 14 × 8 s ≈ 130 s, + 2 s lead-in + 10% slack
≈ 146 s; +15 s ≈ 161 s ✓); one refinement DIRECTION = one invocation — the
up-sweep run (11 dwells + the half-amplitude peak point = 12 × 8 s + lead + slack
≈ 108 s; +15 s ≈ 123 s ✓) and the down-sweep run (11 × 8 s + lead + slack ≈ 99 s;
+15 s ≈ 114 s ✓) are separate invocations, paired for the two-measurement
evidence through the anchor-reference gate (cross-invocation combination is
legitimate by design). `--joint`: exactly one probe joint per invocation,
validated before activation.

Posture order proven→new: P1 mild anchor, P2 track7/8 anchor, P3 extended endpoint
region, P4 near-horizontal max extension — the settle gate itself validates
holdability before any probe; P3/P4 first settles are the riskiest moments (hand
support ready, cutoff in reach). One posture per app invocation.

## Implementation steps (gate order: unit → integration → sim twin + analysis →
emulator smoke → hardware)

1. **Extract `x7::LatencyVerifier`** — new `apps/latency_verifier.hpp`; `TrackingRun`
   (apps/track_common.hpp) delegates. Pure refactor pinned by the EXISTING
   tests/unit/track_common_test.cpp latency cases (unmodified = the behavior pin). Do
   first and alone so any regression is bisectable.
2. **`TwoMassArm` fixture** — new `apps/two_mass_arm.hpp` (implements arm::Arm):
   per-probed-joint two-mass plant J_m q̈_m = τ − K_g(q_m−q_l) − C_g(q̇_m−q̇_l);
   J_l q̈_l = K_g(...) + C_g(...) − b_l q̇_l; torque motor-side, `state.q` reports
   LINK side only (non-collocated, as hardware); semi-implicit Euler at 1e-4 s
   (explicit Euler is unstable for lightly damped oscillators; ω₀h ≈ 0.008, margin
   ~250×); SimArm-style synchronous CommandSnapshot synthesis (a default-constructed
   snapshot would blow the latency verifier's deadline instantly; C6a). Planted modes:
   joint 1 J_l 0.4/J_m 0.05 → K_g ≈ 35.5, ζ 0.03 → C_g ≈ 0.075 (4.5 Hz); joint 5
   J_l 0.01/J_m 0.05 → K_g ≈ 55.6, ζ 0.05 → C_g ≈ 0.068 (13 Hz). New
   `tests/unit/two_mass_arm_test.cpp` (fixture ring-down frequency/decay, snapshot
   sanity under the verifier).
3. **`IdentController`/`IdentRun`** — new `apps/ident_common.hpp`: constant-anchor
   ComputedTorque wrap (`MinJerkTrajectory(q_anchor, q_anchor, 1.0)` — the
   track_common_test pattern), phase-accumulated stepped sine + ramps + ADAPTIVE
   pre-measure hold (I/Q block convergence, [1 s, 4 s]) + near-phase-crossing window
   end, amplitude rule + ALL-JOINT headroom precheck, sum + re-clamp +
   `probe_clipped`, ALL-JOINT online demod/deviation/saturation/gate monitors with
   the HARD/SOFT taxonomy (HARD fault — thermal, voltage, gate, deviation,
   saturation, stale feedback, latency, I/O — terminates arm::run and deactivates,
   NO retry; SOFT event — response cap, growth, probe clipping — takes the 50 ms
   kill ramp, anchor hold, re-settle, at most one reduced-amplitude retry),
   thermal/voltage supervision via the injected HealthProvider, session budget
   (base-schedule pre-check with the 15 s setup allowance, post-settle admission
   against the activation timestamp, T_stop graceful termination, conditional
   extensions/retries), anchor-reference gate
   (`--anchor-ref`, ±0.02 rad/joint), ident CSV writer (full D8 block + per-row
   `dwell_id, dwell_phase, probe_joint, probe_hz, probe_amp, probe_phase, probe_tau,
   cmd_total, probe_clipped, resp_amp_est, dwell_attempt` + per-joint `temp, volt`;
   `#` header gains
   `--label`, anchor q, planned amplitudes, tuning snapshot) + per-dwell JSON
   summary sidecar (online I/Q per joint, verdicts, anchor vector). New
   `tests/unit/ident_common_test.cpp`: state-machine timing on scripted mocks, clamp
   flag, ALL-JOINT monitor trips (a SOFT event on a NON-probe joint must trigger the
   50 ms emergency kill ramp then re-settle; a HARD fault — thermal, gate,
   deviation, saturation — must abort and deactivate with NO retry), the emergency
   kill-ramp profile itself, adaptive-hold convergence normalization (planted
   decaying transient + noise: no premature acceptance), session budget (base
   schedules exceeding T_stop refused pre-activation; post-settle admission
   refusal when setup ate the margin; an extension/retry that would breach
   T_stop is skipped; T_stop hit → graceful ramp-out + deactivate with remaining
   dwells reported not-run), thermal/voltage pre-run and per-cycle fault paths
   via a scripted HealthProvider, anchor-ref refusal at ±0.02 rad, LS-window
   property. Plus **`x7::SessionWatchdog`** — own header
   `apps/session_watchdog.hpp`: a dedicated thread with a HOST-STEADY-CLOCK
   deadline and an injected expiry callable; arm-at-activation /
   disarm-on-deactivation API; fires the callable EXACTLY ONCE on expiry, never
   after disarm; the callable must be safe from any thread — on hardware it is
   `requestQuiesce()` plus a report, NEVER `deactivate()` or `Port::close()`
   (round 7: neither is concurrency-safe against a blocked shutdown). New
   `tests/unit/session_watchdog_test.cpp`: no fire before deadline, no fire
   after disarm, single fire on expiry while the arming thread is deliberately
   blocked. Plus **`CraneX7::requestQuiesce()`** — a new atomic, idempotent,
   any-thread-safe request gating EVERY bus-producing path (round 8): the
   background thread checks it before each read, between the read and write
   halves of a cycle, and before each write (writes AND reads cease — the
   servo Bus Watchdog counts reads too), without being joined; `deactivate()`
   re-checks it after every in-flight transaction and, once quiesced, issues
   NO further bus operations — its quiesced cleanup only stops/joins the
   thread, marks the session inactive, and closes the port after in-flight
   I/O has returned. Feedback goes stale by design, so a still-running
   arm::run aborts via the existing stale policy, and its subsequent ordinary
   `deactivate()` returns WITHOUT restarting traffic (which would reset the
   servo watchdog). Covered in tests/unit/hw_test.cpp: traffic ceases at the
   next gate check, quiesce from a second thread mid-loop, quiesce landing
   mid-deactivate() leaves the remaining writes unsent,
   deactivate-after-quiesce produces zero bus operations.
4. **Synthetic regression** — new `tests/integration/ident_sim_test.cpp`: full dwell
   sequence against TwoMassArm with the anchor at the ZERO pose (model gravity ≈ 0
   there; the gravity-free fixture must not be fed a real gravity feedforward — C6b);
   assert recovered peak within ±0.3 Hz and ζ within ×2 via in-test complex SDOF fit
   on a fine local grid. Register new test files in tests/CMakeLists.txt.
5. **Apps** — new `apps/x7_ident.cpp` (mirror the x7_track skeleton: openSession
   current mode → activate + ARM SessionWatchdog (fires at activation +
   T_quiesce = 179.5 s; expiry action = `requestQuiesce()` + report ONLY —
   traffic silence trips the servo Bus Watchdog; the watchdog never calls
   deactivate() or touches the port) → gravity hold →
   strict settle → start guard → thermal pre-run gate → anchor-ref gate →
   post-settle admission check → ONE fresh IdentRun → ONE arm::run →
   deactivate-first-report-after → DISARM watchdog (no per-joint loop — one
   probe joint per activation session by design); CLI `--joint` (exactly ONE
   value, validated BEFORE activation), `--freqs`, `--amp`, `--label`, `--log`,
   `--anchor-ref`; NO ring-down flag) and
   `apps/x7_ident_sim.cpp` (TwoMassArm twin — the CSV source for validating the Python
   analysis; synthesizes constant temp/volt; x7_track/x7_track_sim pattern). Register
   in apps/CMakeLists.txt.
6. **Offline analysis** — new `tools/ident_analysis.py` (+ numpy in
   tools/pyproject.toml; run as `uv run --project tools tools/ident_analysis.py`):
   per-dwell dt-weighted least-squares fit on [1, t, sin φ, cos φ]; direct-ratio
   FRF (τ_meas primary; commanded + delay-corrected secondary; their ratio = actuator
   transfer); all-joint participation vectors (every joint's q is in the CSV); complex
   SDOF fit with per-dwell residuals and confidence intervals; repeat-dwell /
   up-vs-down / half-amplitude consistency checks flagged in the output; ANCHOR MERGE
   GUARD — refuses to combine runs whose recorded anchors differ beyond the ±0.02 rad
   tolerance; per-posture mode table (JSON + markdown) with survey-only entries marked
   low-confidence. dzco `dz_tf_ident_fr` noted as an optional later cross-check only.
7. **Protocol doc** — new `docs/IDENTIFICATION_PROTOCOL.md` + mkdocs nav: CANONICAL
   P1–P4 joint vectors with the ±0.02 rad tolerance (P1 = pass-1 anchor, P2 =
   track7/8 anchor, P3/P4 nominal), the 4 postures in proven→new order,
   joint/frequency matrix (default probe joints {1 tilt, 3 elbow, 5 wr.pitch}),
   amplitude rule and caps, the THREE-invocation procedure per probe joint-posture
   (survey, refinement up-sweep, refinement down-sweep — separate paired
   invocations) with the refinement evidence requirements (half-amplitude
   linearity check at the peak), thermal/voltage limits and cooldown rules, the
   graceful-stop threshold T_stop = 177.5 s, the T_quiesce = 179.5 s silence
   watchdog with the servos-stop-by-180 s designed bound (and the physical
   cutoff as the true last line), the mandatory partitions, ~1.5–2.5 min per INVOCATION (worst-case base
   146 + 108 + 99 ≈ 353 s ≈ 6 min of collection per probe joint-posture,
   EXCLUDING cooldown and operator handling), operator steps, abort semantics,
   output table
   semantics, and the explicit statement that the 0.6 scale cap stands until the
   notch design lands *(superseded by the Closure: the cap is final)*. Status touch
   in docs/REMEDIATION_PLAN.md.

## Verification

- `ctest --output-on-failure`: all existing tests green (latency cases unmodified pin
  the verifier extraction); new unit tests (two_mass_arm, ident_common) and the
  ident_sim integration regression green.
- `./build/apps/x7_ident_sim` end-to-end → CSV; `uv run --project tools
  tools/ident_analysis.py <csv>` recovers 4.5 Hz/ζ0.03 (joint 1) and 13 Hz/ζ0.05
  (joint 5) within the regression tolerances — this validates the ENTIRE pipeline
  (probe → log → analysis) against known truth before hardware.
- Watchdog real-mechanism test on the emulator: a shortened watchdog deadline
  expiring mid-session — two SEPARATE timing assertions (round 9): a
  deterministic SDK/configuration check that a FORCED full sync-read timeout
  (the emulator made deliberately non-responsive so the SDK runs its timeout
  to expiry) stays ≤ the B_io = 50 ms budget — measuring only healthy
  transactions would not validate the worst case — and a loaded integration
  check that request-to-traffic-silence (reads and writes) stays ≤ B_io +
  B_sched under concurrent CPU load. Plus: a deadline landing
  mid-deactivate() leaves the remaining
  writes unsent, that the in-progress run aborts via the stale-feedback
  policy, and that the app thread completes the quiesced cleanup (zero further
  bus operations) and reports the cause. (The servo-side Bus Watchdog reaction to
  bus silence is the hardware's own documented layer-1 guarantee, enforced by
  the firmware ≥ 38 activation gate — not reproducible on the emulator and not
  re-proven here; the operator's actuator-power cutoff remains the final
  physical backstop.)
- Emulator smoke: `dxl_emu` + `x7_ident --port <pty> --joint 1` — bus plumbing,
  settle gate, abort paths (the emulator has no flexible modes; no FRF expectations).
- `uv run mkdocs build --strict` clean.
- **Hardware protocol (operator)**: P1 mild anchor first — survey run on joint 1,
  verify the achieved noise floor and dwell verdicts in the analysis output BEFORE any
  refinement run or new posture; then per the protocol doc. Every run produces the
  full telemetry; any abort names its cause.

## Out of scope

The notch/phase-compensated D-path design (consumes this step's mode tables); lifting
the 0.6 scale cap; extending SimArm with gear elasticity (TwoMassArm covers the
validation need); dzco-based identification (optional cross-check later); any
ring-down protocol (removed in review — it would need its own integrator
freeze/reset design, caps, and closed-loop model).

## Addendum — integrated placement/capture startup (2026-07-27)

Hardware sessions showed the hand handover cannot preserve a posture
within the ±0.02 rad anchor tolerance: four consecutive anchor-gate
refusals with a systematic 0.06–0.18 rad loss on the gravity-loaded
joints during the limp gap between torque-off and the gravity hold,
on top of 0.03–0.06 rad of position-mode friction sag in the
placement itself. Reviewer-approved change (supersedes the pure
hand-placement decision above; the gates' authority is unchanged):

`x7_ident --pose-first` performs the position-mode move with
MEASURED-posture convergence (goal-offset iterations,
`apps/pose_common.hpp`), an in-process mode switch with a clamped
gravity-current preload written before torque enable
(`CraneX7::setActivationCurrentPreload`, shared limiter with
writeCurrents), a confirmed-application release cue, and a
pre-lead-in CAPTURE phase in `IdentRun`: a bounded min-jerk reference
from the measured first sample onto the canonical anchor under the
shipped restoring ComputedTorque (admission envelope 0.08 rad — abort
beyond it, never pull through), with the quiescence metric
(`apps/quiescence_metric.hpp`, extracted from the settle gate) and
the ±0.02 rad anchor gate evaluated UNDER the hold. One controller
instance spans capture, gates, lead-in and probe (no integrator
reset across the gate). The capture worst case (20 s) is budgeted in
the pre-activation check; the position phase precedes current-mode
activation and does not consume the session deadline. Hand placement
remains the documented fallback.

## Addendum — qualified identification controller tuning (2026-07-27)

The Method section's "SHIPPED tuning unchanged" and "the integrator is
what pins the posture" statements are SUPERSEDED for the identification
anchor controller (x7_track's shipped tuning is untouched). Hardware
stabilization sessions at P1 (nine hold-diagnostic runs) found and
fixed two hold instabilities the float-settled tracking flows never
exposed:

- **Joint-0 (pan) PD scale 0.5** (`kIdentScale0`): the shipped scale
  1.0 pan limit-cycles at ~8.9 Hz (±13 mrad, controller-pumped,
  0.34 Nm p-p) under the stationary anchor hold. At 0.5 the pan is
  bounded and slightly dissipative (0.030–0.047 rad/s across six
  qualifying runs).
- **Integral frozen at capture acceptance**: the live integrator winds
  at Ki·e against stiction-held, in-tolerance error (joint 2: −0.436
  → −0.564 Nm) until ~26 mrad breakaway slips recur. The freeze stops
  the winding while the CAPTURED bias keeps acting — it is that stored
  bias, not ongoing integration, that pins the posture (the original
  "Ki=0 would sag" concern applied to an empty integrator). The bias
  persists uninterrupted through lead-in, probing, retries and the
  final hold, and the ACTUAL frozen vector is recorded per run in the
  sidecar (observed joint-2 bias varied −0.109 to −0.427 Nm across
  qualifying holds — visible to the analysis for repeatability
  comparisons, never a merge criterion).

Qualification: three consecutive complete 30 s P1 holds under the
continuous gates (2026-07-27); the unchanged quiescence and anchor
gates remain the authority. The FRFs are controller-specific to this
recorded tuning (see the earlier addendum); the sidecar tuning block
and the analysis merge guard enforce dataset consistency.

### Refinement (2026-07-27, same day): per-joint integral latching

The single freeze-at-acceptance proved insufficient on hardware: in
the first production survey capture, joint 3 — stable and in-band at
+0.012 rad — wound its integral from −0.07 to −0.54 Nm while joint 2
blocked the arm-wide admission, then broke away ~37 mrad and the
capture timed out. Capture now LATCHES each joint's integral
individually once that joint's own in-band + quiet readiness sustains
0.3 s (never resumed; a latched joint that PD + stored bias cannot
hold in-band times the capture out), with global admission requiring
all eight latched plus the unchanged simultaneous gates for a further
0.3 s. Per-joint biases and latch times are recorded in the sidecar.
Tolerance, speed threshold, timeout and the no-probe-before-admission
rule are unchanged.

## Addendum — the null survey and the observability limit (2026-07-27)

The first valid-procedure P1 joint-1 survey completed every dwell
with clean safety, timing, and thermal margins — and measured no
plant response at all: joint 1's output encoder reported a SINGLE
count through 13 of 16 measurement windows (two adjacent counts at 2,
3, and 20 Hz) while τ_meas tracked the full 0.05–0.15 Nm probe. Two
design assumptions failed on hardware together:

- **Excitation**: the amplitude rule's 0.005 rad target assumed a
  free linear plant. At a gravity-held stationary posture the probe
  sits below joint 1's effective stiction/deadband; the XM430's
  encoder is on the OUTPUT shaft, so gear-train wind-up from the
  motor-side probe is invisible and the output simply does not move.
- **Observability**: the 3.6e-5 rad analytic noise floor assumed the
  encoder dithers across counts. A stationary joint is tick-frozen —
  a constant signal demodulates to numerical zero, and the unforced
  lead-in's floor calibration degenerates the same way (~1e-15),
  which is why every adaptive hold timed out: the SNR truly was
  unresolved. What the survey DID measure cleanly is the
  commanded→measured actuator transfer (magnitude ~1.0 flat 2–20 Hz,
  phase −8° to −76°, consistent with the recorded ~9.9 ms
  first-apply delay) — a real notch-phase-budget deliverable.

Consequences (reviewer-directed): the analysis REFUSES mode fitting
without at least five confident dwells above an explicit
observability floor on the probe-joint response — "insufficient
usable data", never a fit of quantization noise (the null survey's
former 5.04/12.06 Hz "fits" at |H| ≈ 5e-16 rad/Nm, with
participation ratios of 10^10, were numerical artifacts); a retried
dwell is analyzed only over its final completed attempt, with the
CSV labeling attempts (`dwell_attempt`) and the sidecar recording
the accepted attempt's effective amplitude (`amp_eff_nm`). The next
hardware step is a single-dwell amplitude pilot at 4.5 Hz stepping
0.20 → 0.25 → at most the unchanged 0.30 Nm hard cap, one run at a
time, all gates unchanged; if 0.30 Nm still produces no observable
joint-1 response, the small-signal stationary current-probe method
is not viable for joint 1 at P1 and the protocol or probe/posture
choice must change (reviewer decision).

### Pilot result (2026-07-27): stop condition reached

The single-dwell 4.5 Hz amplitude pilot ran at 0.20, 0.25, and
0.30 Nm (the unchanged hard cap), one run at a time, all gates
unchanged — mechanically clean, torque delivered faithfully. The
joint-1 response stayed noise-dominated at every step: 22/68/38 µrad
against run-measured noise floors of 423/733/1051 µrad,
non-monotonic in amplitude, phase-incoherent, 1–2 ambient encoder
counts of raw motion, all adaptive holds timed out. Joint 1 at P1 is
NOT identifiable with the stationary small-signal current probe; the
pilots are preserved as null-result evidence and never enter mode
fitting. The pilot incidentally produced the first non-degenerate
run-specific noise-floor measurements (ambient tick-toggling gave
the lead-in calibration real signal): ~0.4–1.1 mrad, 10–30× the
analytic figure. Consequences: pilot success is now DEFINED (campaign
checklist) as a converged hold AND a response clearing the
run-specific floor — max(analytic 3.6e-5 rad, recorded floor_q),
reported by the analysis as obs_snr — and the analyzer applies that
per-dwell effective floor. The campaign is paused for a
protocol-design decision (posture with lower joint-1 load, a
different probe joint, motion/dither excitation, or external
link-side sensing); no direction is chosen in this plan.
*(Resolved 2026-07-28: closed without pursuing any direction — see
the Closure section below.)*

## Closure (2026-07-28) — the 0.6 cap is the final supported boundary

Decision (owner, 2026-07-28, following the reviewer-confirmed stop
condition): **the 0.6 excursion-scale cap in `x7_track` is the FINAL
SUPPORTED operating boundary.** The notch/phase-compensated D-path
redesign and the "full-amplitude torque mode" end goal are CLOSED,
not deferred. Pass 2 step A ends here.

**What was executed.** The implementation was qualified and the
campaign executed through its first scientific gate, where the stop
condition was reached — the wider campaign (refinement sweeps,
joints 3/5, postures P2–P4) was never entered. Concretely:
implementation with sim-twin validation (planted modes recovered at
4.51 Hz ζ 0.033 and 13.01 Hz ζ 0.058), the integrated pose-first
placement/capture startup, the qualified identification controller
(joint-0 scale 0.5, per-joint integral latching; three qualifying
30 s holds plus a production-path equivalence hold), one
valid-procedure survey at P1 joint 1, and the reviewer-directed
single-dwell amplitude pilot at 4.5 Hz stepping 0.20 / 0.25 /
0.30 Nm — the unchanged hard cap. The final survey and the three
pilot runs were mechanically clean — placement within ±0.02 rad,
capture admission ~1.6 s, zero saturation, clipping, overruns, or
I/O failures, torque delivered faithfully at every amplitude —
whereas the development trials before them (failed hand-placement
handovers, capture timeouts, the joint-0 limit cycle and joint-2/3
integrator–stiction hunting) are recorded in the addenda above.

**What was found (the null result — precisely scoped).** What the
experiment established is exactly this: **joint 1 at posture P1 is
not identifiable with the stationary small-signal current probe up
to the 0.30 Nm hard cap.** Gearbox stiction locked the output shaft
at every admissible amplitude (the probe joint reported a single
encoder count through 13 of 16 survey windows; pilot responses of
22–68 µrad sat 10–30× below their run-measured 0.4–1.1 mrad noise
floors, non-monotonic in amplitude and phase-incoherent), and the
servo's output-shaft encoder is structurally blind to motor-side
gear wind-up. Other joints, other postures, other sensing, and
other excitation strategies were NOT tested. The closure
generalizes by judgment, not by experiment: the ~4–5 Hz whole-arm
mode that forced the cap participates most strongly at the
gravity-loaded shoulder joints — exactly where stiction is highest —
so in-method adjustments were judged unlikely to close the gap, and
the out-of-method routes were judged not worth their cost absent a
concrete need for scale > 0.6. That judgment is the owner decision
above, not an experimentally established impossibility.

**What survives the closure.** The commanded→measured actuator
transfer (magnitude ~1.00–1.05 flat across 2–20 Hz, phase −8° to
−76°, consistent with the measured ~9.9 ms one-cycle apply delay) —
the phase-budget input for any future compensator; the first
empirical stationary-hold noise floors (0.4–1.1 mrad, 10–30× the
analytic figure — the servo encoder cannot support small-signal
stationary work); the session-safety machinery (deadline ladder,
quiesce, watchdogs), the pose-first startup, and the analysis
tooling with its dataset-integrity guards; and the preserved
null-result datasets (`p1_j1_survey_r2`, `p1_j1_amp020/025/030`),
which never enter mode fitting.

**Reopening conditions (documented, not planned).** Reopening
requires BOTH a concrete application need for scale > 0.6 AND one of:
external link-side sensing (IMU/accelerometer — in the tested
configuration the flexible response was invisible to the servo
encoder); identification around a moving
operating condition, where motion linearizes the friction that
defeated the stationary probe (a pragmatic sub-variant: mode
estimates from existing full-amplitude tracking telemetry, a
deliberately wide conservative notch, and cautious empirical
scale-stepping under the existing gates); or dither-assisted
excitation with its own safety analysis. Any reopening restarts from
the external review process. Until then, large fast motions beyond
scale 0.6 belong to position-mode tracking.
