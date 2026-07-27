---
name: archive-data
description: Archive new hardware telemetry per docs/DATA_ARCHIVE.md — checksum, copy to the operator's private archive, verify, add manifest rows, track the sidecars. Invoke after a hardware session leaves CSVs at the repo root.
disable-model-invocation: true
---

Follow this procedure exactly; the manifest and checksum discipline is a
review requirement.

1. List the untracked telemetry at the repo root (`*.csv`, `*.csv.*` — they
   are gitignored, so check `ls`, not `git status`).
2. Locate the private archive root: consult project memory or ask the
   operator. NEVER write that path into any tracked file — the manifest uses
   archive-relative paths only.
3. `sha256sum` every file BEFORE moving; keep the list.
4. `cp -p` each file into the appropriate campaign subdirectory of the
   archive (create a new one per campaign as needed); re-checksum the copies
   and verify every hash matches; only then delete the originals from the
   repo root.
5. Add one row per file to the matching table in `docs/DATA_ARCHIVE.md`:
   filename, role (what the run was; which project record cites it), SHA-256.
   Mark failed runs FAILED with a note that they must never enter analysis.
6. Copy each run's `.dwells.json` sidecar into `data/` and track it.
7. Verify `uv run mkdocs build --strict` passes, commit the manifest and
   sidecar changes (Conventional Commits), and confirm `git status` is clean.
