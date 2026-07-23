# x7_track instrumentation & hardening — Pass 1 of 2

> Pass 1 does NOT itself enable scale 1.0: it makes the loop observable, deterministic,
> and continuous so that pass 2 (identification + D-path redesign) can.

*Status: planned (2026-07-21), revised after two plan-review rounds.*

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
injectable `now_()` clock as everything in CraneX7); `AppliedRecord::time` (D3) uses the
same absolute clock; the RUNNER alone subtracts the first `state.t` to produce the
controller's relative `t`. RealArm therefore has NO time-origin latch of its own (the
earlier H2 latch is superseded), and `feedback_time − apply_time` is well-defined.
`SimArm` fills seq from its step counter and `state.t` from sim time (its own consistent
clock/origin).

### D3 — Command sequencing, receipts, and a COHERENT applied-command snapshot
(F1 + rounds 3–4)
Commands must be as observable as feedback, the row that logs a command must know that
command's own submission sequence, and feedback + applied-command data must come from
ONE atomic snapshot (querying live hardware after the write is race-dependent):
- `AppliedRecord` becomes a bridge-level type in `arm/types.hpp`:
  `{ uint64_t target_seq; uint64_t cycle; double time; bool write_ok; uint8_t mode;
  double applied[8]; uint8_t flags[8]; }` — `applied[]` is in the MODE'S NATIVE UNITS
  (rad / rad·s⁻¹ / Nm; the Nm conversion applies in current mode only, which is what
  x7_track logs), flags = {magnitude-clamped, position-gated}. `threadLoop` updates it
  under `state_mutex_` — alongside `feedback_`/`feedback_time_`/`feedback_seq_` — after
  EVERY write attempt; failed writes recorded with `write_ok = false` (distinguishable
  from an old command intentionally repeated).
- **Coherent snapshot**: `Arm::readState(JointState&, AppliedRecord* applied = nullptr)`
  — RealArm fills BOTH from a single `CraneX7` accessor that copies feedback + stamp +
  seq + the applied record under one `state_mutex_` hold. SimArm synthesizes (synchronous
  application: applied = its last accepted command). The observer receives this
  snapshot; it never queries hardware post-write.
- `setTarget*` stores `++target_seq_` atomically with the targets and reports it via an
  optional out-param. `Arm::writeCommand(const JointCommand&, CommandReceipt* receipt =
  nullptr)` — `CommandReceipt { bool accepted; uint64_t submitted_seq; }`.
- **LaggedArm sequencing (round 4)**: the wrapper keeps its OWN sequence: the receipt
  identifies the newly accepted (pending) command; when the pending command is written
  to SimArm on the next cycle, its original wrapper sequence becomes the applied
  sequence in the wrapper-synthesized AppliedRecord. A pass-through would associate the
  current row's request with the previous command's sequence and corrupt the sim-twin
  latency verification.
- `run()` gains an optional `CycleObserver` invoked AFTER writeCommand (before step):
  `bool observe(t, state, applied_snapshot, cmd, receipt)` — **returning false aborts
  the run** (run() returns false). `TrackingRun` implements Controller + CycleObserver;
  all CSV writing happens in `observe()`.
- **Latency verification**: each cycle, using the coherent snapshot, verify the
  PREVIOUS cycle's `submitted_seq` was applied within ≤ 2 cycles; abort on the FIRST
  confirmed violation. Pass-1 timing is thereby "monitored and bounded, abort on
  violation" — the honest formulation of the determinism claim.

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
difference) remains as defense-in-depth.

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
  (2) `τ_raw = ff + pd + i_cand`;
  (3) COMMIT `i = i_cand` unless (`τ_raw > +τ_max` and `i_cand > i`) or
  (`τ_raw < −τ_max` and `i_cand < i`) — rejection only when the update drives farther
  into saturation; the unwinding direction always commits;
  (4) the emitted command `τ_commanded = clamp(τ_raw, ±τ_max)` uses exactly the
  hardware's limits including the outer guard: `τ_max = max(0, min(effort_limit,
  kt·servoCurrentLimitAmps()) − margin·kt)` per joint (mirrors `writeCurrents`; an
  oversized margin cannot yield a negative limit). Sim twin uses `effort_limit8`.
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
clamp 1.5 / filter taus. Apps and tests consume the same numbers (tests link only
`rtctrl::rtctrl`; duplication is how the shipped config became untested).

### D8 — Widened CSV with explicit event semantics (rounds 3–4)
A row mixes three event times; the fields name them unambiguously, all timestamps share
one absolute clock (D2), and the header comment documents the relationships:
- **Times**: `control_t` (runner-relative, drives the trajectory) and absolute
  `feedback_time` (= state.t) are both logged.
- **Feedback event** (this row's measurement, from the coherent snapshot):
  `feedback_seq`, `feedback_time`, per joint `q`, `dq_servo`, `tau_meas` (measured
  current × kt at the feedback event).
- **This cycle's request** (computed now, applied later): `submitted_seq`, per joint
  `qd`, `dqd`, `dq_est`, `ff`, `pd`, `i`, `tau_raw` (pre-controller-clamp),
  `tau_commanded` (post-controller-clamp), `controller_sat`.
- **Most recent APPLIED command IN the same snapshot** (belongs to a PREVIOUS
  submission): `applied_seq`, `apply_time`, `apply_ok`, per joint `tau_applied`
  (hardware post-limit, native units — Nm in current mode), `hardware_sat`, `gate`.
  `apply_lag = feedback_time − apply_time` is SIGNED and documented (the background
  cycle reads before it writes, so the latest apply can post-date this row's feedback).
`openCsvLog` header regenerated in the same commit.

## Steps (implementation order)

1. Trajectory interface + RoundTrip (`model/trajectory.{hpp,cpp}`, consumer type
   switches, `tests/unit/trajectory_test.cpp` continuity cases).
2. Feedback stamping + seq (D2) (`hw/crane_x7.{hpp,cpp}`, `arm/types.hpp`,
   `arm/real_arm.cpp`, `arm/sim_arm.{hpp,cpp}`; hw_test: stamp/seq advance, frozen on
   failed read, follows injected time).
3. Command-side sequencing + applied record (D3) (`hw/crane_x7.{hpp,cpp}`; hw_test:
   applied record matches submitted target seq, flags set under clamp/gate).
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
- New tests listed per step, all green; whole suite green. Round-4 additions:
  atomically coherent feedback/applied snapshots (thread advancing between reads must
  not mix epochs); equal timestamp origins and correct SIGNED `apply_lag`; LaggedArm
  request→application sequence mapping; observer-triggered abort propagates out of
  `run()`; controller saturation vs hardware saturation distinguished in the record;
  failed command writes recorded with `write_ok = false`.
- Sim twin end-to-end: `./build/apps/x7_track_sim 0.5` and `--start <track8 pose>`:
  widened CSV with sane fb_seq/applied_seq/t columns, no turnaround torque step.
- Emulator smoke: `dxl_emu` + `x7_track --port <pty> 0.3`: settle gate enforced,
  single-run banner, per-leg report, stats line.
- `uv run mkdocs build --strict` clean.
- **Hardware (after merge): instrumentation validation only** — one run at the proven
  scale 0.5 confirms timing/sequencing columns are sane and the turnaround is smooth.
  This run is NOT sufficient input for pass-2 identification (wrong posture, little
  4–5 Hz excitation); pass 2 starts by designing the dedicated identification protocol.
