# Tracked evidence sidecars

The `.dwells.json` files here are the per-run summary sidecars of the
hardware identification campaign (2026-07-27): the complete controller
tuning record, per-dwell verdicts, demodulated estimates, frozen-bias
vectors, and capture telemetry that the project records quote — small,
text, and diffable, so they are tracked with the code.

The raw CSV time series they summarize live in the operator's private
data archive outside this repository; see
[docs/DATA_ARCHIVE.md](../docs/DATA_ARCHIVE.md) for each file's role
and SHA-256. These sidecars alone cannot be re-fitted (the analysis
needs the raw CSVs). They carry the controller, capture, dwell, and
demodulation values cited by `docs/IDENTIFICATION_PLAN.md`'s Closure
section and its addenda; raw timing and sample-level claims (apply
delays, encoder-count behavior, overrun/I/O counts) require the
archived CSVs.

Fields reflect the code version that wrote each sidecar: the three
`p1_hold_s050_fz_r*` files carry `integral_frozen_at_capture: 1`
without a `frozen_bias_nm` vector (they predate bias recording), and
the analyzer deliberately refuses them for exactly that reason —
their frozen biases are recoverable from the archived raw CSVs'
integral column.
