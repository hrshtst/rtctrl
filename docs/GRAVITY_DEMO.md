# Gravity-compensation demo (`x7_gravity_demo`)

`x7_gravity_demo` floats the arm under pure gravity compensation —
held against gravity, freely back-drivable by hand — with the
torque-constant calibration exposed as an explicit, per-model
parameter. It is the **simplified sibling of `x7_float`** and exists
for demonstration and calibration exploration only.

**Demonstration-only status.** Every log this app writes self-labels
`# run_mode: demonstration` and is **never acceptance evidence**.
`x7_float` remains the M-GC3 acceptance instrument, with its own
protocol (release marker, evaluation windows, the vendor-calibration
gate) unchanged — see the
[bring-up checklist](HARDWARE_BRINGUP.md#after-bring-up-m6m8) and the
[gravity-calibration record](HISTORY.md#gravity-compensation-and-torque-constant-calibration).

## Prerequisites

- Bring-up steps 1–6 completed
  ([checklist](HARDWARE_BRINGUP.md#steps)), and everything in its
  [safety section](HARDWARE_BRINGUP.md#safety-read-first) applies:
  current mode on real hardware, **power cutoff within reach**,
  workspace under the arm clear.
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

A flagless run **is** the approved vendor calibration: the app
overwrites the loaded config's `command_torque_scale` from its
built-in defaults (the vendor-empirical constants, 2.20 / 3.60 Nm/A),
reproducing exactly the scales of
`config/crane_x7_vendor_scale.toml` — whatever `--config` it loads.

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
[theory notes](theory/gravity-compensation.md): measured-vs-model
agreement verifies the servo *current loop* tracking the command, not
output-shaft torque.

## Experimental calibration policy

Effective torque constants customize per servo model, in Nm/A:

```sh
./build/apps/x7_gravity_demo --experimental-calibration \
  --kt-xm430 2.35 --kt-xm540 3.40 --log demo2.csv 30
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
  [DATA_ARCHIVE.md](DATA_ARCHIVE.md). They are never committed.

## Relationship to `x7_float`

| | `x7_gravity_demo` | `x7_float` |
|---|---|---|
| Purpose | demonstration, calibration exploration | M-GC3 acceptance instrument |
| Calibration | vendor by default; deviations via explicit experimental opt-in | approved vendor vector only (mode-independent gate) |
| Protocol | none — optional ENTER to end early | release marker, evaluation window / feel-check modes |
| Log | `run_mode: demonstration`, never acceptance evidence | validated by `check_float_log.py`; acceptance path exists |
| Shared | pose-first preloaded startup, soft-limit refusal, exclusive `--log`, 60 s bound, verified shutdown | same |
