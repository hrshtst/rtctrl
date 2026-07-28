# Consistency-audit remediation — verified shutdown, strict CLI, doc/comment truth

*Status: **EXECUTED 2026-07-28** in the two commits below (the strict-CLI /
shutdown-guard commit, then this documentation commit). Planned 2026-07-28
from the combined consistency audit (internal four-dimension sweep + external
reviewer audit); revised same day per the reviewer's five plan findings and
four follow-up notes (exception-safe guard incl. x7_ident's own; x7_set_param
ranges + write verification; corrected telemetry scoping plus cause messages
on the silent aborts; seven-site activation-contract sweep; validation-command
and rationale corrections). No control behavior changes: the scope is shutdown
verification, argument parsing, and documentation/comment truth. The 0.6-cap
closure, campaign status, gates, and tuning are untouched.*

## Context

The combined audit found the identification closure and evidence coherent, but
ten issues remain: one High (hardware apps can print `done` over an incomplete
shutdown), three Medium (the activation contract overpromises "no motion";
"every run leaves the full telemetry" is false for setup-phase aborts;
permissive order-dependent CLI parsing), and a tail of Low/doc items (stale
comments describing superseded designs, a stale docs index, sidecar
version-skew overstatements, minor doc imprecision). This plan fixes all of
them in two commits.

Reviewer-offered choice made here: for the telemetry claim the statement is
SCOPED rather than adding a new early setup/event log — the campaign is
closed, so new logging machinery would serve no active procedure.

## Commit 1 — `fix(apps): verified shutdown guards and strict CLI parsing`

### 1a. apps/x7_common.hpp (shared plumbing; all additions)

- Add includes `<cmath>`, `<vector>`, `<cstdlib>` (strtod/strtol),
  `<cerrno>` (errno/ERANGE); the x7_set_param range checks include
  `<cstdint>` (or `<limits>`) where used.
- **Rework `Cli`/`parseCli` to be order-independent and strict**:
  `Cli{config_path, port_override, std::vector<const char*> rest, bool ok}`.
  Scan ALL argv: consume `--config`/`--port` wherever they appear; a flag
  missing its value prints `"%s requires a value"` to stderr and sets
  `ok=false` (return-based — matches the repo's bool-parse-helper convention
  and keeps the header unit-testable); everything else lands in `rest` in
  order. This also fixes the old `i < argc-1` lone-trailing-flag blind spot.
  Documented limitation (header comment): a token literally equal to
  `--config`/`--port` is always the flag, never another flag's value.
- **Move `parseStrictDouble`** here from x7_ident.cpp (verbatim body: strtod,
  full-token, isfinite) and add a `parseStrictLong` twin (strtol, base 10,
  full-token) for joint indices and servo parameters.
- **Add the shutdown guard**, message text verbatim from x7_ident.cpp's
  audited original, made EXCEPTION-SAFE inside `run()` (review finding:
  `CraneX7::deactivate()` is not noexcept — it allocates and joins a
  thread; a throw must still quiesce and report, and must not be
  swallowed by the `done` latch):

  ```cpp
  template <class Hw>
  struct ShutdownGuardT {
    explicit ShutdownGuardT(Hw& h) : hw(h) {}
    ShutdownGuardT(const ShutdownGuardT&) = delete;
    ShutdownGuardT& operator=(const ShutdownGuardT&) = delete;
    Hw& hw; bool done = false; bool clean = false;
    bool run() {              // idempotent; never throws
      if (done) return clean;
      done = true;
      try {
        clean = hw.deactivate();
      } catch (...) {
        clean = false;        // a throw is an unclean shutdown
      }
      if (!clean) { hw.requestQuiesce(); /* SHUTDOWN FAULT stderr text */ }
      return clean;
    }
    ~ShutdownGuardT() { if (!done) run(); }
  };
  using ShutdownGuard = ShutdownGuardT<rtctrl::hw::CraneX7>;
  ```

  Rationale verified: `RealArm::deactivate()` is a pure forward to
  `CraneX7::deactivate()` (src/arm/real_arm.cpp), so `CraneX7&` covers every
  app; `requestQuiesce()` is inline in crane_x7.hpp — no new link
  dependencies (examples/ already links `rtctrl::rtctrl`).

### 1b. Per-app conversion (uniform pattern)

Every parseCli caller (9 apps): `if (!cli.ok) return 1;` after parseCli;
`x7::ShutdownGuard shutdown{*session.arm};` INSIDE the try, immediately after
a successful `activate()` (never before — activation failures already release
servos internally); every post-activation exit funnels through
`shutdown.run()` explicitly (destructor = exception net only); success text
prints only when work-ok AND clean, else SHUTDOWN FAULT and exit 1.

- **x7_track.cpp** — strict arg loop over `cli.rest`: `--kp/--kd/--ki` via
  `parseStrictDouble` (atof-garbage→0 gains was the same silent-zero bug),
  `--log` value, unknown `--*` → `"unknown argument"` + return 1, positional
  scale via `parseStrictDouble` with a second-positional rejection (kills the
  unknown-flag→atof→minimum-scale hazard). Guard after activate. Bare
  `return 1`s (readState failures) → `shutdown.run(); return 1;`; the three
  unchecked deactivates → `shutdown.run()`. Final report: stats/fclose stay
  after the shutdown ("safe transition FIRST" comment stays true); then
  `SHUTDOWN FAULT (run done|ABORTED)` + exit 1 when !clean, else the existing
  done/ABORTED.
- **x7_float.cpp** — strict positional duration (reject >1 positional);
  final: `ok && clean` gating as above.
- **x7_onoff.cpp** — strict hold_s; deadman-fail →
  `shutdown.run(); return 1;`; read-failure `break` now tracked in `ok`
  (intended behavior change: no more `done`/exit 0 after a read failure).
- **x7_move_simple.cpp** — `parseStrictLong` joint, `parseStrictDouble`
  delta, reject >2 positionals; readAll-fail → `shutdown.run(); return 1;`;
  reorder the end: deactivate (via `shutdown.run()`) BEFORE the
  complete/ABORTED report (fixes success-printed-before-deactivation).
- **examples/x7_wave.cpp** — as x7_float, plus readState-fail →
  `shutdown.run(); return 1;`.
- **x7_pose.cpp** — `--vel` via `parseStrictDouble` (loop already rejects
  unknowns); guard after activate; the placement-fail abort and the
  escalated/deadman `return 1`s → `shutdown.run()`; final
  `const bool clean = arm.deactivate();` → `shutdown.run()` — PRESERVE the
  existing operator wording ("torque off INCOMPLETE — check the arm…"): the
  guard's stderr fault line is complementary, the stdout procedure text stays.
- **x7_read.cpp** — strict positional duration (`<= 0` = forever stays
  legal); no guard (never activates).
- **x7_set_param.cpp** — rewrite its broken loop (argc-1 bound, never
  advances past values, `atol("garbage")`→writes P-gain 0): each flag
  requires a value via `parseStrictLong`, unknowns rejected. Additionally
  (review finding): reject values outside the register ranges BEFORE
  narrowing — P gain in `0..UINT16_MAX`, profile velocity/acceleration in
  `0..UINT32_MAX` — and require EVERY requested write to succeed: any
  failed write prints which parameter failed and exits 1 (today a failed
  write can still exit 0). No guard (never activates).
- **x7_ident.cpp** — forced-minimum edits: arg loop head converts to
  `cli.rest`; delete the local `parseStrictDouble` (calls become
  `x7::parseStrictDouble`); `--joint` switches from `atoi` to
  `parseStrictLong` (same silent-garbage→joint-0 bug class). Its OWN
  ShutdownGuard receives the identical exception-safety fix in place (wrap
  the `robot.deactivate()` call in try/catch; a throw → `clean = false` →
  quiesce + SHUTDOWN FAULT; watchdog disarm unchanged) — leaving it
  byte-identical would leave the exception-path finding open there (review
  finding). The two silent post-activation `readState` aborts (the exits
  that print no cause) each gain an explicit stderr message naming the
  cause before `shutdown.run()`. Everything else stays as audited.

### 1c. New test: tests/unit/x7_common_test.cpp (+ register in tests/CMakeLists.txt)

- `Argv` helper (mutable storage for `char*[]`).
- `FakeHw{deactivates, quiesces, ok, throws}` driving
  `ShutdownGuardT<FakeHw>`: clean run (no quiesce, idempotent second run,
  no double-deactivate); failed deactivate (quiesce exactly once, `clean`
  false, idempotent); **throwing deactivate** (review finding: `run()`
  returns false, quiesce exactly once, nothing propagates); destructor net
  with and without failure, including the throwing variant.
- `parseCli`: flags at front (back-compat); flags AFTER positionals;
  interleaved flags; lone trailing `--config`/`--port` → `ok=false`;
  duplicate flag last-wins; argc==1 defaults; the documented
  `--config --port` limitation pinned.
- `parseStrictDouble` / `parseStrictLong` accept/reject cases —
  `parseStrictLong` must reject `ERANGE` overflow (review finding), pinned
  with an out-of-long-range literal.

### 1d. Validation for commit 1

- Build + `ctest --test-dir build` (all labels).
- Hardware-free CLI smokes (fail before any port open): `x7_read --port` →
  "requires a value"; `x7_track --bogus 2` → "unknown argument";
  `x7_track garbage` → strict-scale rejection (the reviewer's exact bug);
  `x7_float 5 --port /dev/null` → order-independence reaches openSession;
  `x7_set_param` with garbage, negative, overflow (`> UINT32_MAX`), and
  missing-value inputs → each rejected (review finding).
- Emulator end-to-end (EVERY app invocation carries `--port <pty>` — a
  bare invocation would target the configured physical port; review
  finding): `dxl_emu` + `./build/apps/x7_onoff --port <pty> 2` (clean-path
  guard: prints done, exit 0) and a short `./build/apps/x7_ident
  --port <pty> --joint 1 --freqs "5"` (proves the ident loop conversion
  didn't regress).

## Commit 2 — `docs: correct the activation contract, telemetry scope, and superseded comments`

### 2a. Activation contract (full sweep — review finding: four more sites)

- include/rtctrl/arm/arm.hpp — replace "clamps the goal to the present
  posture so activation causes no motion" with mode-split truth: activation
  never COMMANDS motion; position mode snaps goals to the present posture
  (arm held); current mode starts at ZERO current — the arm is unsupported
  and can fall under gravity unless a preload was staged
  (CraneX7::setActivationCurrentPreload); real hardware also arms safety.
- include/rtctrl/hw/crane_x7.hpp — the `activate()` doc comment ("snaps
  goals to present … No motion results") gets the same mode-split
  correction.
- src/hw/crane_x7.cpp — the corresponding implementation comment at the
  goal-snap step gets the same correction.
- src/arm/sim_arm.cpp — the activation comment claiming the posture is held
  regardless of mode gets the same correction (current-mode activation
  initializes zero torque; the simulated arm falls once stepped).
- docs/guide/architecture.md — `// torque on, safety armed, NO motion` →
  `// torque on, safety armed; holds in position mode, zero-current in
  current mode unless preloaded`.
- docs/guide/overview.md — the activation claim near its safety paragraph
  gets the same mode-split wording.
- README.md — "activation cannot cause motion" → "activation never commands
  motion (current-mode activation is zero-current — apps must command
  support immediately or explicitly stage a preload)". Wording per review:
  x7_track activates at zero current and sends gravity support on the first
  cycle — "apps stage a preload" alone would be false.

### 2b. Telemetry claim scoping

docs/IDENTIFICATION_PROTOCOL.md: "Every abort names its cause; every run
leaves the full telemetry." → the reviewer's corrected wording (the CSV
opens before `IdentRun`, so CAPTURE is recorded — but `--log` is OPTIONAL,
and manual settling writes its own `<log>.settle` file): "When `--log` is
provided, the primary CSV records the capture/probe controller run.
Earlier refusals do not create that CSV; manual settling may leave the
separate `.settle` log." The "every abort names its cause" half becomes
true rather than softened: commit 1 adds cause messages to the two silent
`readState` aborts in x7_ident (see 1b).

### 2c. Superseded/stale comments

- apps/x7_ident.cpp SAFETY prose — pose-first is the primary method
  (position-mode placement → in-place switch → capture); hand placement is
  the FALLBACK. (The usage block already documents --pose-first; only the
  prose changes.)
- apps/x7_ident.cpp `--scale0` comment — rewrite: an OPTIONAL diagnostic
  override restricted to `--hold`; the zero sentinel selects the qualified
  identification default `kIdentScale0` (0.5), not the shipped scale and not
  1.0 (both old claims are pre-qualification fossils).
- apps/ident_common.hpp — "(0 = shipped kGainScale)" → "(0 = the qualified
  identification default kIdentScale0)"; "[0.5, 30] Hz" → "[0.5, 20] Hz".
- apps/x7_pose.cpp header — reframe: the PRIMARY campaign flow was
  `x7_ident --pose-first` (no hands); x7_pose remains the visual
  posture-confirmation aid and the manual-fallback helper. The SAFETY
  paragraph stays (still true for its limp-release flow).
- include/rtctrl/hw/crane_x7.hpp — `feedCommand()` doesn't exist: reword to
  "each successful command write feeds the deadman; checkDeadman() escalates
  once the last one is older than the timeout…".

### 2d. Docs index + sidecar versioning + minor

- docs/README.md — add the missing entries mirroring the site nav:
  Identification protocol + Identification campaign checklist under
  Hardware; Remediation plan, Identification plan, Data archive manifest,
  AND this consistency remediation plan under Project records (one line
  each, closure-aware descriptions) — the index must list every nav
  document or the 2e validation claim would be false.
- docs/IDENTIFICATION_PROTOCOL.md — scope "Every sidecar records…":
  sidecars record the tuning as of the code version that wrote them; the
  effective-mode field, `frozen_bias_nm`, and `amp_eff_nm` appear from the
  versions that introduced them; the qualification-era sidecars
  (`p1_hold_s050*`) predate some fields, and the analyzer deliberately
  refuses a sidecar claiming a freeze without the bias vector. Add
  provenance to the bias range: "−0.109 to −0.427 Nm (from the qualifying
  holds' raw-CSV integral column — reviewer-verified per-run joint-2 biases
  −0.1088, −0.4173, −0.4268 Nm; their sidecars predate bias recording)".
- data/README.md — the same version-skew sentence (the three
  `p1_hold_s050_fz_r*` sidecars carry `integral_frozen_at_capture: 1`
  without `frozen_bias_nm`; the analyzer refuses them by design; their raw
  CSVs in the archive carry the integral column).
- docs/DATA_ARCHIVE.md — append to each of the three `fz` CSV-row roles:
  "(sidecar predates bias recording — analyzer refuses it by design)".
- docs/IDENTIFICATION_CAMPAIGN.md — the dry-run snippet gets the
  `./build/apps/` prefix.
- docs/IDENTIFICATION_PLAN.md — add `cmd_total` and `dwell_attempt` to the
  per-row CSV column list (both written and read; the protocol already cites
  `dwell_attempt`).
- THIS plan file (docs/CONSISTENCY_REMEDIATION_PLAN.md) is added to git and
  to the mkdocs nav under Project records in commit 2, with its status line
  updated to record execution.

### 2e. Validation for commit 2

The `/preflight` gauntlet: build (comments compile), full ctest, strict
mkdocs, ruff. Grep re-checks: `feedCommand` gone; `[0.5, 30]` gone;
docs/README lists all nav docs.

## Out of scope (explicitly)

- No early setup/event log for x7_ident (choice recorded above).
- dxl_inspect's separate arg loop (not a torque-enabling app).
- Sim apps' parsing: x7_track_sim/x7_ident_sim still use permissive
  `atof`/`atoi` in places — left out of scope because a sim app cannot
  torque hardware, NOT because they are strict (rationale corrected per
  review).
- No change to CraneX7 behavior, gates, tuning, budgets, or any campaign
  semantics (crane_x7.hpp/.cpp receive comment-only edits in 2a/2c).

## Reuse

- Guard semantics/message: apps/x7_ident.cpp (the audited original).
- Strict parse: x7_ident.cpp `parseStrictDouble` (moved, not rewritten).
- Unknown-argument wording: x7_ident.cpp.
- Test fixtures: Argv/FakeHw patterns are new but minimal; deactivate-failure
  hardware behavior itself is already covered by tests/unit/hw_test.cpp
  ("deactivate keeps the Bus Watchdog armed…").

## Verification (end-to-end)

1. `cmake --build build -j` — the Cli rework makes any missed caller a
   compile error (the safety feature of removing `argi`).
2. `ctest --test-dir build --output-on-failure` — all existing tests plus the
   new x7_common cases.
3. CLI smokes + emulator smokes per 1d.
4. `uv run mkdocs build --strict`; `uvx ruff check tools/`.
5. Two commits (Conventional Commits), push origin main after each.
