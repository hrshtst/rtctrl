# Gravity-compensation demo (`x7_gravity_demo`)

`x7_gravity_demo` floats the arm under pure gravity compensation —
held against gravity and intended to be **gently hand-guidable** —
with the torque-constant calibration exposed as an explicit,
per-model parameter. It is the **simplified sibling of `x7_float`**
and exists for demonstration and calibration exploration only.

> **Known limitations (waived, not fixed).** The float acceptance's
> subjective back-drive criterion FAILED and was explicitly waived —
> a risk/quality acceptance, not a test pass. Expect the **j1 notch**
> when hand-guiding across the low-current transition region
> q1 ≈ +0.27…+0.53 rad (mechanism not isolated) and a small **j4
> positive tendency** under hand-guiding (command ≤ 0.055 Nm). Guide
> the arm gently and do not force through a notch; full record in
> [history.md](../records/history.md#gravity-compensation-and-torque-constant-calibration).

**Demonstration-only status.** Every log this app writes self-labels
`# run_mode: demonstration` and is **never acceptance evidence**.
`x7_float` remains the M-GC3 acceptance instrument, with its own
protocol (release marker, evaluation windows, the vendor-calibration
gate) unchanged — see the
[bring-up checklist](bringup.md#after-bring-up-m6m8) and the
[gravity-calibration record](../records/history.md#gravity-compensation-and-torque-constant-calibration).

## Prerequisites

- Bring-up steps 1–6 completed
  ([checklist](bringup.md#steps)), and everything in its
  [safety section](bringup.md#safety-read-first) applies:
  current mode on real hardware, **power cutoff within reach**,
  workspace under the arm clear.
- This app enters the ordered post-bring-up ladder
  ([after bring-up](bringup.md#after-bring-up-m6m8))
  **after** a clean multi-joint position-mode session
  (`examples/x7_wave`) and at least one clean `x7_float` run on the
  approved vendor calibration — it must never be the arm's first
  current-mode session.
- Start the arm mid-range: a joint parked inside its soft-limit
  margin band is refused (the current gate would cut gravity support
  in one whole direction there).
- Offline rehearsal works like every other app:
  `./build/apps/dxl_emu --link /tmp/ttyDXL &` then add
  `--port /tmp/ttyDXL`.

## Approved default invocation

```sh
./build/apps/x7_gravity_demo --log demo1.csv
```

A run without torque-constant overrides **is** the approved vendor
calibration: the app overwrites the loaded config's
`command_torque_scale` from its built-in defaults (the
vendor-empirical constants, 2.20 / 3.60 Nm/A), reproducing the scales
of `config/crane_x7_vendor_scale.toml` within the app's 1e-6 gate
tolerance (the tracked config stores the ratios rounded to six
decimals) — whatever `--config` it loads.

Startup is the pose-first placement pattern proven by `x7_float`:
activation holds the arm in position mode (no free-fall instant), the
held posture is read, and the calibrated gravity currents are staged
through the in-place switch to current mode, so support flows from
the first torque-on instant. The console then compares measured
torque (current × kt_effective) against the model's g(q) each second;
press **ENTER** at any time to end the session early through the
verified shutdown. Duration defaults to 20 s and is bounded to
(0, 60] (the reviewed global bound).

Note the agreement caveat from the
[theory notes](../theory/gravity-compensation.md): measured-vs-model
agreement verifies the servo *current loop* tracking the command, not
output-shaft torque.

## Experimental calibration policy

Effective torque constants customize per servo model, in Nm/A. The
primary form of the command runs against the **emulator** — rehearse
every experimental value there before any hardware session:

```sh
./build/apps/dxl_emu --link /tmp/ttyDXL &
./build/apps/x7_gravity_demo --port /tmp/ttyDXL \
  --experimental-calibration --kt-xm430 2.35 --log demo2.csv 30
```

- The constants map onto `command_torque_scale = kt_nominal /
  kt_effective`, so every command flows through the single calibrated
  torque→current boundary; the reviewed config bound [0.5, 1.0]
  binds unchanged — in kt terms **[kt_nominal, 2 × kt_nominal]**,
  i.e. [1.783, 3.566] for the XM430-W350 and [2.409, 4.818] for the
  XM540-W270. Values outside refuse before bus contact.
- That bound is an *electrical* bound, **not** a validated
  gravity-compensation envelope (scale 1.0 inside it is the
  known-failed float configuration of the 2026-07-29 incident). Any
  kt deviating from the vendor calibration therefore additionally
  requires the unmistakable `--experimental-calibration` opt-in —
  without it, only the vendor values run.
- Deviations warn in **both** directions: below vendor commands
  hotter currents than the approved calibration (the incident class —
  risk of over-compensation); above vendor under-supports (the arm
  sinks toward gravity — be ready to hold it).
- The log self-labels `# calibration: EXPERIMENTAL` (vendor-equal
  runs label `vendor-approved`), so an experimental session can never
  be read back as the approved demonstration.

### Controlled hardware procedure

An experimental calibration on the real arm is a deliberate,
stepwise session — never a copy-paste of an arbitrary value pair:

1. Rehearse the invocation on the emulator first (the primary
   example above). Rehearsal validates the CLI, the startup sequence,
   the bus behavior, and the log contract — **not** the physical
   effect or safety of a proposed torque constant. The hardware
   session is then a separate command: the hardware port and a
   **fresh log filename** (the emulator run already created its log,
   and the exclusive-log rule refuses reuse):

    ```sh
    ./build/apps/x7_gravity_demo --port /dev/ttyUSB0 \
      --experimental-calibration --kt-xm430 2.35 --log demo2_hw.csv 30
    ```

2. Change **one servo model per session**, starting with a **small
   deviation** from the vendor value. Do not bias one model hotter
   and the other weaker in the same session: under- and
   over-supported joints produce competing motion tendencies.
3. Support the arm through the mode switch and the first seconds
   after it, exactly as for a float: with a weaker-than-vendor kt the
   arm sinks when let go; with a hotter one it can push upward.
4. Keep the power cutoff in hand for the whole session, and use a
   fresh `--log` filename per attempt.

## Logs and archive handling

- `--log` is **required** and the file is created exclusively — an
  existing file is refused, never overwritten — enforcing the
  unique-filename-per-attempt rule before bus contact.
- The CSV carries the full calibration provenance in its header
  (`kt_nominal`, `kt_effective`, `command_torque_scale` per joint)
  plus one row per cycle: `q`, `dq`, `tau_model` (the g(q) request),
  `tau_meas` (current × kt_effective), and the limiter
  `clamped`/`gated` flags.
- Raw hardware CSVs land at the repo root **gitignored** and belong
  in the operator's private archive with a manifest row — see
  [data-archive.md](../records/data-archive.md). They are never committed.

## Relationship to `x7_float`

| | `x7_gravity_demo` | `x7_float` |
|---|---|---|
| Purpose | demonstration, calibration exploration | M-GC3 acceptance instrument |
| Calibration | vendor by default; deviations via explicit experimental opt-in | approved vendor vector only (mode-independent gate) |
| Protocol | none — optional ENTER to end early | release marker, evaluation window / feel-check modes |
| Log | `run_mode: demonstration`, never acceptance evidence | validated by `check_float_log.py`; acceptance path exists |
| Shared | pose-first preloaded startup, soft-limit refusal, exclusive `--log`, 60 s bound, verified shutdown | same |
