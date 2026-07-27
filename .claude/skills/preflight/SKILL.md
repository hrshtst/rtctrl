---
name: preflight
description: Run the repo's full validation gauntlet — build, all ctest suites, strict mkdocs, ruff over tools/. Use before any commit or when asked to verify the working tree.
---

Run these from the repo root, in order, stopping to report on the first
failure. If `build/` does not exist, configure first with
`cmake -B build -DCMAKE_BUILD_TYPE=Release`.

1. `cmake --build build -j`
2. `ctest --test-dir build --output-on-failure`
3. `uv run mkdocs build --strict`
4. `uvx ruff check tools/`

Report one line per step (pass/fail, with counts — e.g. "132/132 tests").
All four must pass before a commit; do not skip the docs or ruff steps for
"code-only" changes — doc snippets and the Python analysis tooling are
review-audited too.
