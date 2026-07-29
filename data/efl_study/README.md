# Offline EFL study — canonical results

`results.json` is the authoritative, machine-readable result table of
the preregistered offline exact-feedback-linearization study
([docs/ENVELOPE_STABILITY_PLAN.md](../../docs/ENVELOPE_STABILITY_PLAN.md);
implementation and frozen constants in
[docs/EFL_STUDY_IMPLEMENTATION_PLAN.md](../../docs/EFL_STUDY_IMPLEMENTATION_PLAN.md)).
It is produced by `build/apps/x7_efl_study` and may be committed here
ONLY when its own `canonical` field is true, which the runner grants
only to a complete, unfiltered matrix executed from a worktree that is
clean INCLUDING untracked files at the exact commit the binary embeds.

The file contains: schema version, the exact invocation, the full
build commit SHA with dirty status, the build type, every frozen
constant (torque-limit vector, assumed servo current limits, poses,
delay, windows, floors), the complete gain grid with the selected
gains (or null), all 28 case cells with their metrics and modal
classifications, and the C1–C4 decision booleans (or null with a
note).

Per-cycle simulator traces are reproducible scratch output under
`build/efl_study/` and are deliberately NOT tracked — rerunning the
same commit regenerates them. `docs/DATA_ARCHIVE.md` remains
hardware-specific and does not list simulation results.
