# Repository Guidelines

## Project Structure & Module Organization

Public C++17 headers live in `include/rtctrl/`; implementations are grouped
under `src/{arm,dxl,emu,hw,model}`. Executable tools and hardware programs are
in `apps/`, while small API demonstrations are in `examples/`. Catch2 tests are
split between `tests/unit/` and `tests/integration/`. Robot models and deployment
settings belong in `models/` and `config/`; documentation is under `docs/`.
`third_party/` and the top-level legacy/reference projects are external code:
avoid modifying them unless the change explicitly targets that dependency.
Tracked experiment summaries live in `data/`; raw telemetry is external and
indexed by `docs/records/data-archive.md`.

## Build, Test, and Development Commands

Initialize dependencies and build from the repository root:

```sh
git submodule update --init third_party/{mi-lib,DynamixelSDK,crane_x7_description}
./tools/bootstrap_milib.sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Use `ctest --test-dir build -L unit` for fast logic tests and `-L integration`
for emulator and dynamics-simulation coverage. Build documentation with
`uv run mkdocs build --strict`. Python analysis tools use their own environment,
for example `uv run --project tools tools/ident_analysis.py <log.csv>`.

## Coding Style & Naming Conventions

Match the existing C++ style: two-space indentation, braces on the same line,
`snake_case` files, `lowerCamelCase` functions, `UpperCamelCase` types, and
`kCamelCase` constants. Keep public declarations in `include/` and implementation
details in `src/`. Prefer short comments that explain safety constraints or
non-obvious control decisions. Python follows standard Ruff-compatible style.

## Testing Guidelines

Add `*_test.cpp` coverage with descriptive Catch2 `TEST_CASE` names. Pure logic
belongs in `tests/unit/`; PTY, SDK-path, threading, or simulation behavior belongs
in `tests/integration/`. Tests must run without physical hardware. Hardware
changes require simulator/emulator coverage first and must preserve watchdog,
quiescence, saturation, and position-gate behavior.

## Commit & Pull Request Guidelines

Follow recent history: `feat(scope): ...`, `fix(scope): ...`, `docs: ...`, or
`chore: ...`, using an imperative, concise subject. Pull requests should explain
the motivation, safety impact, tests run, and documentation changes. Link issues
when applicable. Do not commit generated builds or raw CSV/ZVS telemetry; archive
hardware data externally and update the tracked manifest and evidence sidecars.
