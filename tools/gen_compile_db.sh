#!/bin/sh
# Generate a repo-root compile_commands.json for clangd covering the
# rtctrl tree (incl. DynamixelSDK, via CMake's native export) and the
# mi-lib stack (captured with bear during a clean bootstrap rebuild).
#
# Usage: tools/gen_compile_db.sh [--rebuild-milib]
#   The mi-lib capture lives at $MILIB_PREFIX/milib_compile_commands.json
#   and is reused until --rebuild-milib is passed (or the file is
#   deleted); rebuild it after bumping the mi-lib submodules. The CMake
#   side is re-read from build/compile_commands.json on every run
#   (configuring first if the build directory is missing).
set -eu

REPO_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PREFIX=${MILIB_PREFIX:-$REPO_ROOT/.local}
export MILIB_PREFIX="$PREFIX"
MILIB_DB="$PREFIX/milib_compile_commands.json"

if [ "${1:-}" = "--rebuild-milib" ] || [ ! -f "$MILIB_DB" ]; then
  command -v bear >/dev/null 2>&1 || {
    echo "error: bear is required to capture the mi-lib compile commands" >&2
    exit 1
  }
  # Force full recompilation: an up-to-date make compiles nothing and
  # bear would record an empty database. Best-effort — clean needs
  # zeda-makefile-gen, which is absent before the very first bootstrap.
  export PATH="$PREFIX/bin:$PATH"
  for lib in ${MILIB_LIBS:-zeda zm zeo dzco roki roki-fd liw zx11 roki-gl}; do
    make -C "$REPO_ROOT/third_party/$lib" clean >/dev/null 2>&1 || true
  done
  mkdir -p "$PREFIX"
  # --force-wrapper: the generated <lib>-config scripts carry their
  # "#!/bin/sh" on line 3, so exec returns ENOEXEC and bear's preload
  # interceptor breaks the shell's plain-name fallback; wrapper mode
  # shims only the compiler names and leaves script execution alone.
  bear --force-wrapper --output "$MILIB_DB" -- \
    "$REPO_ROOT/tools/bootstrap_milib.sh" "$PREFIX"
  [ "$(jq length "$MILIB_DB")" -gt 0 ] || {
    echo "error: bear captured no compile commands (nothing recompiled?)" >&2
    exit 1
  }
fi

if [ ! -f "$REPO_ROOT/build/compile_commands.json" ]; then
  cmake -B "$REPO_ROOT/build" -DCMAKE_BUILD_TYPE=Release
fi

jq -s 'add' "$REPO_ROOT/build/compile_commands.json" "$MILIB_DB" \
  > "$REPO_ROOT/compile_commands.json"
echo "compile_commands.json: $(jq length "$REPO_ROOT/compile_commands.json") entries" \
  "($(jq length "$MILIB_DB") from mi-lib)"
