# x7_track instrumentation & hardening — Pass 1 of 2

> Pass 1 does NOT itself enable scale 1.0: it makes the loop observable, its timing
> monitored and bounded (abort on violation), and its controller state continuous, so
> that pass 2 (identification + D-path redesign) can.

*Status: pass 1 COMPLETE — implemented and hardware-validated
2026-07-23 (`pass1.csv`/`pass1.csv.settle`, scale 0.5): feedback
intervals 10.0 ms median / 10.4 ms max with zero sequence gaps,
receipt-matched first-apply delay 9.6–10.3 ms across 401 commands
(bound 20 ms), zero saturation/gate engagements, turnaround torque
steps within ordinary cycle deltas, `ff+pd+i ≡ tau_raw` holding on
hardware, and the settle gate correctly riding out ~1.2 rad/s of real
equilibration motion before opening at 0.024 rad/s residual. Pass 2
(identification protocol, then the notch/phase D-path redesign) not
started; the 0.6 scale cap stands.*

## Context

External review: the ~4.5 Hz configuration-dependent oscillation at full amplitude is a
sampled-feedback architecture problem — ~117° of D-path phase at the mode (55° from the
50 ms PD low-pass whose corner is 3.2 Hz, 30° velocity filter, ~32° bus pipeline), so
nominal damping injects energy; synthetic controller timing; controller state discarded
at the turnaround; a quiescence gate that doesn't gate; logging that hides the loop
signals; tests that never exercise the shipped configuration. Decisions: two passes
(pass 1 = this plan; pass 2 = identification then notch/phase D-path redesign);
full-amplitude torque mode remains the end goal; the shipped PD filter and the 0.6 scale
cap stay untouched through pass 1.

**Plan-review round 2 findings incorporated below (referenced as F1–F7):**
F1 command timing not observable (no target/applied sequencing, no post-limit values);
F2 the proposed anti-windup froze integration in the unwinding direction too;
F3 large feedback gaps need an explicit abort/hold policy, not only a clamped filter dt;
F4 a smooth scale-0.5 run cannot identify the scale-1 mode (instrumentation validation
only — pass 2 needs a dedicated identification protocol);
F5 `ctest -L unit -L integration` intersects labels and selects nothing;
F6 seeding the velocity estimate from servo `state.dq` on the first sample contaminates
later filtered samples;
F7 a `--force-start` settle override weakens the safety remediation on hardware.

## Design (incorporating both plan-review rounds)

### D1 — Trajectory polymorphism + round trip
Abstract `model::Trajectory` (virtual `sample(t,q,dq=nullptr,ddq=nullptr) const`,
`duration()`, `size()`, virtual dtor); `MinJerkTrajectory` derives (signature already
matches, all by-value uses prvalue-elided, no slicing paths). New
`model::RoundTripTrajectory` (two MinJerk segments stored by value, `outDuration()`,
C²-continuous split). `SinusoidTrajectory` untouched. Consumers switch member/param type
to `const Trajectory&`: `ComputedTorque`, `TrackingRun`, tests' `PdOnly`/`trackingRms`.

### D2 — Feedback stamping and sequencing; one clock, one origin (round 4)
`CraneX7::readAll` stores `feedback_time_ = now_()` and `++feedback_seq_` under
`state_mutex_`; `servoCurrentLimitAmps()` accessor. `JointState` gains
`std::uint64_t seq = 0` (no brace-init/memcpy users exist).
**Clock discipline**: `JointState::t` carries the ABSOLUTE stamped feedback time (same
injectable `now_()` clock as everything in CraneX7); `AppliedTargetRecord`'s
`first_time`/`latest_time` (D3) use the same absolute clock; the RUNNER alone subtracts
the first `state.t` to produce the controller's relative `t`. RealArm therefore has NO
time-origin latch of its own (the earlier H2 latch is superseded), and the D8 timing
columns are well-defined.
`SimArm` fills seq from its step counter and `state.t` from sim time (its own consistent
clock/origin).

### D3 — Command sequencing, receipts, and a COHERENT applied-command snapshot
(F1 + rounds 3–4)
Commands must be as observable as feedback, the row that logs a command must know that
command's own submission sequence, and feedback + applied-command data must come from
ONE atomic snapshot (querying live hardware after the write is race-dependent):
- **One record cannot represent both first application and the current actuator goal
  (round 8)**: the hardware re-evaluates limits and position gating on EVERY
  retransmission of the same target, so the freshest applied values/flags and the
  first-application time are different facts about the same `target_seq` (e.g. first
  write ungated, joint later enters the soft-limit band, retransmission gated to
  zero). The bridge-level record in `arm/types.hpp` therefore carries BOTH:
  `AppliedTargetRecord { bool valid = false; uint64_t target_seq;
  uint64_t first_cycle; double first_time;            // latency verification only
  uint64_t latest_cycle; double latest_time; uint8_t mode;
  double latest_applied[8]; uint8_t latest_flags[8];  // current sat/gate state }` —
  `latest_applied[]` in the MODE'S NATIVE UNITS (rad / rad·s⁻¹ / Nm; the Nm
  conversion applies in current mode, which is what x7_track logs), flags =
  {magnitude-clamped, position-gated}.
  **Update rule** (threadLoop, under `state_mutex_` alongside
  `feedback_`/`feedback_time_`/`feedback_seq_`), on every write ATTEMPT:
  `last_write_attempt = attempt;` and, only when `attempt.ok`:
  `if (!rec.valid || attempt.target_seq != rec.target_seq) { rec.target_seq =
  attempt.target_seq; rec.first_cycle = cycle; rec.first_time = attempt.time;
  rec.valid = true; } rec.latest_cycle = cycle; rec.latest_time = attempt.time;
  rec.latest_applied = attempt.limited_values; rec.latest_flags = attempt.flags;`
  — first-application data is preserved across retransmissions (latency stays
  correct), the latest successful transmission owns the current sat/gate state, a
  failed attempt updates only `last_write_attempt` (leaving the latest successful
  actuator goal intact), and a failed FIRST attempt for a new sequence keeps the
  sequences different so the first successful retry correctly becomes that command's
  initial application.
  `WriteAttemptRecord { bool valid = false; uint64_t target_seq; double time;
  bool ok; }` — updated on EVERY transmission and retransmission.
  **Validity (round 7)**: both records start `valid = false` — startup defaults must
  not read as a genuine sequence-zero application (the initial cached target may
  itself be seq 0); latency verification ignores invalid records until the first
  receipt has had a cycle to apply.
  **Both records travel together**: `CommandSnapshot { AppliedTargetRecord applied;
  WriteAttemptRecord last_attempt; }` is copied under the same `state_mutex_` hold as
  the feedback — the API is `Arm::readState(JointState&, CommandSnapshot* = nullptr)`
  and the observer receives the CommandSnapshot (this is the only path D8's
  `attempt_*` columns can flow through).
  **Write-failure escalation (round 7)**: consecutive background group-write failures
  become a hardware-layer escalation condition symmetric to `max_read_failures`
  (`Options::max_write_failures`, default 5) — bus reads can stay healthy while writes
  fail, leaving an old actuator goal active; waiting for the 250 ms deadman is too
  slow, and the settle phase (which runs without the latency observer) is otherwise
  blind to it. Escalation stops the thread, so `step()` returns false and settling or
  tracking aborts identically. `CycleStats` gains `write_failures` alongside
  `read_failures`, and the apps' end-of-run stats line prints it.
- **Coherent snapshot**: `Arm::readState(JointState&, CommandSnapshot* = nullptr)` —
  RealArm fills state AND CommandSnapshot from a single `CraneX7` accessor that copies
  feedback + stamp + seq + both command records under one `state_mutex_` hold. SimArm
  synthesizes (synchronous application: applied = its last accepted command, attempt
  mirrors it). The observer receives this snapshot; it never queries hardware
  post-write.
- `setTarget*` stores `++target_seq_` and `submission_time = now_()` atomically with
  the targets (it already calls `now_()` for deadman freshness) and reports both via an
  optional out-param. `Arm::writeCommand(const JointCommand&, CommandReceipt* receipt =
  nullptr)` — `CommandReceipt { bool accepted; uint64_t submitted_seq;
  double submission_time; }` (same absolute clock as D2, so `apply_time −
  submission_time` is well-defined and both enforced and logged).
- **LaggedArm sequencing (round 4)**: the wrapper keeps its OWN sequence: the receipt
  identifies the newly accepted (pending) command; when the pending command is written
  to SimArm on the next cycle, its original wrapper sequence becomes the applied
  sequence in the wrapper-synthesized AppliedTargetRecord. A pass-through would associate the
  current row's request with the previous command's sequence and corrupt the sim-twin
  latency verification.
- `run()` gains an optional `CycleObserver` invoked AFTER writeCommand (before step):
  `bool observe(t, state, command_snapshot, cmd, receipt)` — **returning false aborts
  the run** (run() returns false). `TrackingRun` implements Controller + CycleObserver;
  all CSV writing happens in `observe()`.
- **Latency verification with sequence-keyed receipts (round 9)**: the applied record
  in a row belongs to a PREVIOUS submission, so its delay must use the MATCHING
  historical receipt — never the receipt created in the same row. `TrackingRun` keeps
  a small pending map `submitted_seq → submission_time`; in `observe()` it
  (1) matches `snapshot.applied.target_seq` to its stored receipt,
  (2) computes `first_apply_delay = applied.first_time − matched_submission_time`,
  (3) removes completed older receipts,
  (4) aborts if the applied sequence skips a pending sequence or any pending receipt
  exceeds the 2-cycle deadline — an applied sequence NOT present in the map and older
  than every pending entry (e.g. the settle phase's last command at tracking start) is
  a pre-run baseline, not a violation — and
  (5) adds the CURRENT receipt to the map after evaluating the pre-write snapshot.
  Abort on the FIRST confirmed violation. Pass-1 timing is thereby "monitored and
  bounded, abort on violation".

### D4 — Measured time base with an explicit stale-feedback policy (F3 + round 3)
`run()` (runner.hpp): latch `t0` from the first `state.t`; each cycle compute
`t = state.t − t0` and, from `state.seq` and `state.t`:
- **duplicate sample** (seq unchanged): skip update and write, but STILL call
  `arm.step()` (no busy-wait), continue;
- **backward timestamp** (t decreases): abort — clock fault, return false;
- **sequence gap or stale feedback**: abort when seq jumps by more than 2 (i.e. at most
  ONE missed sample tolerated) or the sample interval exceeds 25 ms. Justification: at
  the 4.5 Hz mode, 25 ms is ~40° of modal phase — a bounded, damped perturbation —
  whereas 50 ms (~81°) followed by a trajectory catch-up could itself excite the mode
  under remediation. No catch-up after larger gaps: abort rather than jump the
  trajectory forward (CraneX7's read-failure escalation covers the arm side in
  parallel);
- iteration cap ~`4·duration/dt` stays as a final backstop only.
`settleArm` uses the same pattern via a shared helper. Controllers therefore never see
dt ≤ 0 or a multi-cycle dt beyond the policy limit; ComputedTorque's H3 clamp
(`min(dt, 3 × nominal)` for filter/integrator alphas; raw dt for the backward
difference) remains as defense-in-depth. **Nominal dt source (round 5)**: an explicit
`ComputedTorque::setNominalDt(double)` configured by the application from
`crane_x7_tuning.hpp`'s `kNominalDt` (= the 0.01 s control cycle); documented default
0.01 so a reusable controller stays explicit rather than guessing `Arm::dt()`.

### D5 — ComputedTorque hardening (F2, F6 + H1/H3)
- **First-sample init (F6)**: record `q_prev_`, set `dq_est_ = 0`, and enable the
  D term only from the second fresh sample — never seed from servo `state.dq` (removes
  the controller's last servo-velocity consumer; `useStateVelocity(true)` test path
  unaffected). Same bumpless init in `SettleController`.
- **Duplicate-t freeze (H1)**: `dt == 0` (should not reach the controller after D4, but
  defense-in-depth) holds all estimator/filter/integrator state.
- **Direction-aware anti-windup via the candidate update (F2 + rounds 3–4)**: per
  joint, each cycle:
  (1) `i_cand = clamp(i + Ki·e·dt_clamped, −i_clamp, +i_clamp)` — the existing ±1.5 Nm
  integral-STATE clamp is retained explicitly;
  (2) `τ_cand = ff + pd + i_cand` (decision quantity only);
  (3) DECIDE: `accepted = !((τ_cand > +τ_max && i_cand > i) ||
  (τ_cand < −τ_max && i_cand < i))` — rejection only when the update drives farther
  into saturation; the unwinding direction always commits;
  (4) **the emitted command is recomputed from the COMMITTED state (round 5)**:
  `i_next = accepted ? i_cand : i`; `τ_raw = ff + pd + i_next`;
  `τ_commanded = clamp(τ_raw, ±τ_max)`; `i = i_next` — a rejected candidate never
  leaks into the output (logged `ff + pd + i` always equals the logged `τ_raw`).
  τ_max uses exactly the hardware's limits including the outer guard:
  `τ_max = max(0, min(effort_limit, kt·servoCurrentLimitAmps()) − margin·kt)` per joint
  (mirrors `writeCurrents`; an oversized margin cannot yield a negative limit). Sim
  twin uses `effort_limit8`.
  **Position gating is deliberately outside the pass-1 anti-windup**: the start guard
  and the qf clamp keep compliant trajectories a buffer away from the soft limits, so
  the gate cannot engage in-envelope; the now-logged `gate` flag makes any engagement
  visible, and gate-aware windup handling is revisited in pass 2 with the
  applied-record feedback available.
- **Instrumentation**: retain per-cycle ff/pd/i/dq_est vectors; const accessors;
  `τ_raw`, `τ_commanded`, and a `controller_sat` flag are exposed for logging —
  controller-side clamping would otherwise hide saturation from the hardware flags
  (the writer normally receives an already-limited value and reports `sat = false`).

### D6 — Gating (F7: no override on hardware)
`settleArm` returns `{io_ok, quiescent, elapsed, residual}` — policy-free. `x7_track`
**aborts** (deactivate, exit 1) when `!quiescent`; **no `--force-start`** — a settle
timeout means real residual motion or a broken metric, both grounds to stop.
`x7_track_sim` keeps warn-and-continue (its `--disturb` seeds legitimately never settle).

### D7 — Tuning constants to the library
New `include/rtctrl/arm/crane_x7_tuning.hpp`: kGainScale + shipped Kp 6 / Kd 1 / Ki 6 /
clamp 1.5 / filter taus / `kNominalDt` (0.01 s control cycle, consumed by
`setNominalDt`). Apps and tests consume the same numbers (tests link only
`rtctrl::rtctrl`; duplication is how the shipped config became untested).

### D8 — Widened CSV with explicit event semantics (rounds 3–4)
A row mixes three event times; the fields name them unambiguously, all timestamps share
one absolute clock (D2), and the header comment documents the relationships:
- **Times**: `control_t` (runner-relative, drives the trajectory) and absolute
  `feedback_time` (= state.t) are both logged.
- **Feedback event** (this row's measurement, from the coherent snapshot):
  `feedback_seq`, `feedback_time`, per joint `q`, `dq_servo`, `tau_meas` (measured
  current × kt at the feedback event).
- **This cycle's request** (computed now, applied later): `submitted_seq`,
  `submission_time`, per joint `qd`, `dqd`, `dq_est`, `ff`, `pd`, `i`, `tau_raw`
  (pre-controller-clamp), `tau_commanded` (post-controller-clamp), `controller_sat`.
  Offline analysis obtains an applied command's own submission time by joining
  `applied_seq` to the earlier row whose `submitted_seq` matches (documented in the
  CSV header; `first_apply_delay = first_apply_time − matched_submission_time` — the
  current row's `submission_time` belongs to a NEWER command and must not be used).
- **Applied-target record IN the same snapshot** (belongs to a PREVIOUS submission;
  failed attempts never overwrite it): `applied_valid`, `applied_seq`,
  `first_apply_time`, `first_apply_cycle` (latency facts, preserved across
  retransmissions), `latest_apply_time`, `latest_apply_cycle`, per joint `tau_applied`
  (hardware post-limit at the LATEST successful transmission, native units — Nm in
  current mode), `hardware_sat`, `gate` (current state); plus the last write ATTEMPT:
  `attempt_valid`, `attempt_seq`, `attempt_time`, `attempt_ok`. Derived timing columns
  are named by the event they use: `first_apply_delay = first_apply_time −
  matched_submission_time` (the latency metric; matched via the receipt map) and
  `feedback_minus_latest_apply = feedback_time − latest_apply_time` (SIGNED and
  documented: the background cycle reads before it writes, so the latest apply can
  post-date this row's feedback). **Repeated snapshots of a completed sequence**: the
  pending receipt is removed on first match, but the same applied sequence may stay
  current across further rows while a newer command is in flight — the logger caches
  the last matched `first_apply_delay` per applied sequence and re-emits the cached
  value on those rows, so every row stays self-contained (offline analysis can always
  re-derive it by joining `applied_seq` to the historical `submitted_seq`).
`openCsvLog` header regenerated in the same commit.

## Steps (implementation order)

1. Trajectory interface + RoundTrip (`model/trajectory.{hpp,cpp}`, consumer type
   switches, `tests/unit/trajectory_test.cpp` continuity cases).
2. Feedback stamping + seq (D2) (`hw/crane_x7.{hpp,cpp}`, `arm/types.hpp`,
   `arm/real_arm.cpp`, `arm/sim_arm.{hpp,cpp}`; hw_test: stamp/seq advance, frozen on
   failed read, follows injected time).
3. Command-side sequencing + records (D3) (`hw/crane_x7.{hpp,cpp}`; hw_test: applied
   record matches submitted target seq; flags set under clamp/gate; retransmission and
   validity semantics; write-failure escalation stops the thread like the read-failure
   path).
4. Measured time base + stale-feedback policy (D4) (`arm/runner.hpp`,
   `apps/track_common.hpp` settleArm; new runner unit test with a scripted mock Arm
   emitting duplicate / skipped / backward / over-age samples; re-verify thread_test
   "RealArm drives the bridge").
5. ComputedTorque hardening (D5) (new `tests/unit/computed_torque_test.cpp`:
   irregular-dt estimate, duplicate-t freeze, first-sample D-term enablement, and
   anti-windup entry / sustained saturation / unwinding / recovery).
6. crane_x7_tuning.hpp (D7); apps consume.
7. App restructure (riskiest step; verified by the sequence below, not by anything that
   precedes it): one RoundTrip + one TrackingRun (Controller + CycleObserver) across
   both legs, per-leg report split at `outDuration()`, settle gate without override,
   τ_max wiring, receipt/applied-record logging + latency verification, widened CSV
   (D8) — in `apps/x7_track.cpp`, `apps/x7_track_sim.cpp`, `apps/track_common.hpp`.
8. Integration tests (after step 7, since they exercise the restructured pieces):
   shipped-configuration tracking case (host velocity, 0.05 filter, Ki 6/1.5,
   kGainScale, x7_track amplitudes; calibrate the RMS bound on first run — regression
   pin, not a grade) and turnaround-continuity case (commanded-τ step across the split
   comparable to neighboring cycle deltas) in `tests/integration/tracking_sim_test.cpp`;
   register new unit test files in `tests/CMakeLists.txt`. Then, in order: sim-twin
   runs → emulator smoke → hardware. Gate order is strictly: unit tests (steps 1–6) →
   restructure (7) → integration tests (8) → sim twin → emulator → hardware.
9. Docs: `docs/theory/computed-torque.md` — correct "below the corner" (corner 3.2 Hz,
   mode above it, ~117° D-path budget) and drop input shaping as a stabilizer;
   `docs/IMPLEMENTATION_PLAN.md` — pass-1/pass-2 record at M8;
   `docs/REMEDIATION_PLAN.md` — status update; add to mkdocs nav.

## Explicitly out of scope (pass 2)

Notch/phase-compensated D-path design; two-mass/flexible-joint sampled-data regression
model; lifting the 0.6 scale cap; any Kp/Kd retuning; the identification protocol design
itself (sketch: low-amplitude broadband or ring-down probes at several postures,
extended postures reached in position mode — known stable — before small torque-mode
perturbations, with defined abort thresholds).

## Verification

- `ctest --output-on-failure` (note: `-L unit -L integration` intersects labels and
  selects nothing here; use no label filter or `-L 'unit|integration'`).
- New tests listed per step, all green; whole suite green. Rounds 4–6 additions:
  atomically coherent feedback/applied snapshots (thread advancing between reads must
  not mix epochs); equal timestamp origins and correct SIGNED
  `feedback_minus_latest_apply`; LaggedArm request→application sequence mapping;
  observer-triggered abort propagates out of `run()`; controller saturation vs
  hardware saturation distinguished in the record;
  failed attempts land only in the attempt record and never overwrite the applied
  record; retransmitting the same sequence does not change its
  `first_apply_time`/`first_apply_cycle`; a failed first attempt followed by success records the
  retry as the first application (asserting `first_apply_time`/`first_apply_cycle`);
  latency verification uses first application, not latest retransmission, joined to
  the sequence-matched historical receipt. Round-7 additions: applied and attempt records returned
  atomically through one CommandSnapshot; startup records report invalid rather than a
  sequence-zero application (and a genuine first seq-0 command can still become
  valid); a failed background write triggers the hardware write-failure escalation and
  aborts settling; rejected anti-windup candidates satisfy
  `tau_raw == ff + pd + committed_i`. Round-8 addition: a target initially applied
  ungated whose later retransmission becomes position-gated preserves its
  first-application time/cycle while `tau_applied`/`latest_flags` reflect the gated
  state. Round-9 addition: latency-queue startup — a snapshot already containing a
  valid settle-phase command absent from the tracking receipt map is treated as the
  pre-run baseline, not a skipped tracking command.
- Application-orchestration tests (`tests/unit/track_common_test.cpp`,
  including `apps/track_common.hpp` directly): settle-gate sustained-
  window semantics (accepted / final-low-sample-rejected / timeout-
  rejected) and TrackingRun's receipt-map latency verification
  (baseline, match, deadline, skipped sequence, delayed first
  application). LaggedArm's request→application mapping is executable-
  local and is validated through the sim twin's CSV
  (`first_apply_delay` ≡ one cycle), not unit-tested.
- Sim twin end-to-end: `./build/apps/x7_track_sim 0.5` and `--start <track8 pose>`:
  widened CSV with sane fb_seq/applied_seq/t columns, no turnaround torque step.
- Emulator smoke: `dxl_emu` + `x7_track --port <pty> 0.3`: settle gate enforced,
  single-run banner, per-leg report, stats line.
- `uv run mkdocs build --strict` clean.
- **Hardware (after merge): instrumentation validation only** — one run at the proven
  scale 0.5 confirms timing/sequencing columns are sane and the turnaround is smooth.
  This run is NOT sufficient input for pass-2 identification (wrong posture, little
  4–5 Hz excitation); pass 2 starts by designing the dedicated identification protocol.
