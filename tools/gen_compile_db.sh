#!/bin/sh
# Generate a repo-root compile_commands.json for clangd covering the
# rtctrl tree (incl. DynamixelSDK, via CMake's native export) and the
# mi-lib stack (captured by the metapackage's build_compile_commands.sh:
# bear over a clean rebuild, plus compdb header entries and a .clangd
# that points member sources at their in-tree headers).
#
# Usage: tools/gen_compile_db.sh [--rebuild-milib]
#   The mi-lib capture lives at third_party/mi-lib/compile_commands.json
#   (gitignored inside the submodule) and is reused until
#   --rebuild-milib is passed or the file is missing (the metapackage's
#   `make clean` also removes it); rebuild it after bumping the
#   third_party/mi-lib pin. The CMake side is re-read from
#   build/compile_commands.json on every run (configuring first if the
#   build directory is missing).
#
# Requires bear and compdb on PATH for the mi-lib capture.
set -eu

REPO_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
META="$REPO_ROOT/third_party/mi-lib"
MILIB_DB="$META/compile_commands.json"

if [ "${1:-}" = "--rebuild-milib" ] || [ ! -f "$MILIB_DB" ]; then
  for tool in bear compdb; do
    command -v "$tool" >/dev/null 2>&1 || {
      echo "error: $tool is required to capture the mi-lib compile commands" >&2
      exit 1
    }
  done
  if [ ! -f "$META/config.local" ]; then
    echo "error: $META/config.local is missing (run: tools/bootstrap_milib.sh first)" >&2
    exit 1
  fi
  # freeze --check, .clangd, make clean, bear-wrapped make (reinstalls
  # into the configured prefix), compdb header entries, freeze.
  # SKIP_CHECKS keeps the libraries' test/example targets out of the
  # capture, matching the bootstrap build. The final freeze writes the
  # tracked tools/milib_versions.lock (config.local points there):
  # byte-identical at the pinned state, a reviewable diff otherwise —
  # restore with git if the freeze was unintended.
  (cd "$META" && SKIP_CHECKS=1 ./build_compile_commands.sh)
  [ "$(jq length "$MILIB_DB")" -gt 0 ] || {
    echo "error: empty mi-lib compile database" >&2
    exit 1
  }
  # The mi-lib sources moved from third_party/<lib>/ to
  # third_party/mi-lib/<lib>/: drop the pre-metapackage capture and
  # clangd's background index (re-created lazily).
  rm -f "$REPO_ROOT/.local/milib_compile_commands.json"
  rm -rf "$REPO_ROOT/.cache/clangd/index"
fi

if [ ! -f "$REPO_ROOT/build/compile_commands.json" ]; then
  cmake -B "$REPO_ROOT/build" -DCMAKE_BUILD_TYPE=Release
fi

jq -s 'add' "$REPO_ROOT/build/compile_commands.json" "$MILIB_DB" \
  > "$REPO_ROOT/compile_commands.json"
echo "compile_commands.json: $(jq length "$REPO_ROOT/compile_commands.json") entries" \
  "($(jq length "$MILIB_DB") from mi-lib)"
