# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

rtctrl — C++17 control library for the CRANE-X7 7-DOF arm (mi-lib dynamics, DynamixelSDK bus).

## Build, test, docs

- First build: `git submodule update --init`, then `./tools/bootstrap_milib.sh`
  (drives the `third_party/mi-lib` metapackage: clones the member libraries, pins
  them via its `versions.lock`, installs to the prefix in its `config.local` —
  seeded to `.local/`; the generated `.envrc` + `direnv allow` provides
  PATH/LD_LIBRARY_PATH), then
  `cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j`.
- Tests: `ctest --test-dir build --output-on-failure` (labels `unit` and
  `integration`; integration is headless — pty emulator + roki sim). Tests run
  with the repo root as working directory (they load `models/` and `config/` by
  relative path).
- Docs: `uv run mkdocs build --strict` from the repo root. There are TWO
  separate uv projects: the root pyproject is docs-only; analysis tooling runs
  as `uv run --project tools tools/ident_analysis.py …`.
- Python lint: `uvx ruff check tools/` must stay clean (a standing review criterion).

## Gotchas

- Link only the plain C mi-lib libraries — the `_cpp` variants miscompile the
  roki-fd/zm ODE path (heap corruption). C++-only static members are defined in
  `src/milib_cpp_compat.cpp`; `cmake/FindMiLib.cmake` already handles this — do
  not "fix" it.
- Dynamixel layer: operating-mode writes require torque off; the servo Bus
  Watchdog counts reads as traffic too; `deactivate()` is NOT an e-stop.

## Conventions

- Conventional Commits, committed straight to `main` and pushed; a commit is a
  module plus its tests.
- Canonical 8-DOF joint order everywhere above the `dxl` layer; SI units (rad,
  Nm, A, V) outside `dxl/conversions.hpp` — raw servo units never leak upward
  (deliberate exceptions: operating-mode codes and the raw profile-register
  passthroughs on `CraneX7`).
- Testing ladder (the project's core rule): pure logic → Catch2 unit test in
  `tests/unit/`; anything touching the bus → `emu::FakePacketIO`, new wire
  behavior → the pty fixture in `tests/integration/`; anything producing motion
  → `SimArm` acceptance first, hardware only after. A sim pass is necessary,
  never sufficient.
- Controller, protocol, and safety-gate changes go through the owner's external
  reviewer: implement reviewer directives exactly, and never widen tolerances,
  gates, or timeouts as a remedy for a failing check.

## Data and hardware

- Raw hardware telemetry (root-level `*.csv` etc.) is NEVER committed
  (gitignored). It belongs in the operator's private archive per
  `docs/DATA_ARCHIVE.md` (manifest row with SHA-256; only the `.dwells.json`
  sidecars are tracked, under `data/`). Never write the archive's location into
  any tracked file.
- The 0.6 excursion-scale cap in `x7_track` is FINAL (closure 2026-07-28; see
  `docs/IDENTIFICATION_PLAN.md`, Closure). Do not propose lifting it.
- Hardware sessions follow `docs/HARDWARE_BRINGUP.md` (power cutoff within
  reach, first motion = wrist only); use a unique `--log` filename per attempt.
