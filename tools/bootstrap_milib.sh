#!/bin/sh
# Build and install the mi-lib stack via the third_party/mi-lib
# metapackage: member libraries are cloned by its clone.sh (full
# clones of the forks selected by the submodule's origin URL — the
# https URL is load-bearing, and the first run needs network), thawed
# to the versions pinned by the tracked tools/milib_versions.lock,
# then built and installed in dependency order. Re-running is
# idempotent. To bump mi-lib versions, edit tools/milib_versions.lock
# (or check out the desired member versions and run the metapackage's
# freeze_versions.sh, which writes it via config.local) and re-run —
# the submodule pin governs the metapackage tooling only.
#
# Usage: tools/bootstrap_milib.sh [PREFIX]
#   PREFIX      installation prefix; overrides the configured one
#               (default: third_party/mi-lib/config.local, written on
#               first run with PREFIX=<repo>/.local; under CI=true the
#               metapackage skips config.local and its $HOME/usr
#               default applies)
#   MILIB_LIBS  space-separated, dependency-closed subset to build
#               (default: the configured UPSTREAM_LIBS); CI uses
#               "zeda zm zeo dzco roki roki-fd liw" to skip the
#               X11/GL-dependent zx11 and roki-gl.
#
# Only the C libraries are built — the _cpp variants (same sources
# rebuilt with CC=g++) miscompile the roki-fd/zm ODE path; rtctrl
# links the C libs and supplies the C++-only static-member definitions
# itself (src/milib_cpp_compat.cpp).
#
# System packages required: build-essential, libxml2-dev, liblzf-dev
# (plus X11/GL dev packages for zx11/roki-gl).
set -eu

REPO_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
META="$REPO_ROOT/third_party/mi-lib"

if [ ! -f "$META/clone.sh" ]; then
  echo "error: $META is missing (run: git submodule update --init third_party/mi-lib)" >&2
  exit 1
fi

FULL_LIBS="zeda zm zeo dzco roki roki-fd liw zx11 roki-gl"

# config.local is the metapackage's durable local configuration
# (gitignored inside the submodule; hand-editable). Seed it once with
# the repo-local prefix and the full rtctrl library set.
if [ ! -f "$META/config.local" ]; then
  cat > "$META/config.local" <<EOF
# Written by tools/bootstrap_milib.sh (edit freely; delete to reseed).
UPSTREAM_LIBS="${MILIB_LIBS:-$FULL_LIBS}"
CUSTOM_LIB=""
CUSTOM_LIB_DEPS=""
CUSTOM_TEST_CMD=""
PREFIX="$REPO_ROOT/.local"
# The single version lock, tracked in rtctrl (relative paths resolve
# against this repository). freeze writes it — review the diff, and
# restore with git if a freeze was unintended.
VERSIONS_LOCK="../../tools/milib_versions.lock"
EOF
  echo "wrote $META/config.local"
fi

PREFIX=${1:-$("$META/load_config.sh" --get PREFIX)}
LIBS=${MILIB_LIBS:-$("$META/load_config.sh" --get UPSTREAM_LIBS)}

# The environment outranks config.local AND survives the CI=true
# config.local skip, so the effective configuration is the same
# everywhere (load_config.sh honors set-but-empty values).
export PREFIX
export UPSTREAM_LIBS="$LIBS"
export CUSTOM_LIB="" CUSTOM_LIB_DEPS="" CUSTOM_TEST_CMD=""
export SKIP_CHECKS=1

# Restore the pinned state: thaw from a throwaway copy of rtctrl's
# tracked lock, filtered to the configured libraries (thaw_versions.sh
# iterates the lock, so it must not list unconfigured libraries).
RTCTRL_LOCK="$REPO_ROOT/tools/milib_versions.lock"
VERSIONS_LOCK=$(mktemp)
trap 'rm -f "$VERSIONS_LOCK"' EXIT
for lib in $LIBS; do
  grep -- "^$lib " "$RTCTRL_LOCK" >> "$VERSIONS_LOCK" || {
    echo "error: no entry for '$lib' in $RTCTRL_LOCK" >&2
    exit 1
  }
done
export VERSIONS_LOCK

# Trailing or leading the submodule's own versions.lock is fine —
# rtctrl's lock is authoritative — but make it visible.
drift=""
for lib in $LIBS; do
  theirs=$(grep -- "^$lib " "$META/versions.lock" | cut -d' ' -f2) || theirs=""
  ours=$(grep -- "^$lib " "$RTCTRL_LOCK" | cut -d' ' -f2)
  if [ -n "$theirs" ] && [ "$theirs" != "$ours" ]; then
    drift="$drift $lib"
  fi
done
if [ -n "$drift" ]; then
  echo "note: pins differ from the submodule's versions.lock for:$drift" \
    "(tools/milib_versions.lock is authoritative)"
fi

# The generated <lib>-config tools bake the prefix in; the metapackage
# refuses to build over another prefix's artifacts until cleaned.
if [ -f "$META/.milib-prefix" ] \
    && [ "$(cat "$META/.milib-prefix")" != "$PREFIX" ]; then
  echo "note: mi-lib prefix changed ($(cat "$META/.milib-prefix") -> $PREFIX); cleaning first"
  make -C "$META" clean
fi

(cd "$META" && ./clone.sh && ./thaw_versions.sh)

# Serial on purpose: the upstream sub-makes break the make jobserver.
make -C "$META"

# Dev convenience: generate the repo-root .envrc (direnv) once, and
# keep the machine-local hook present (docs/DATA_ARCHIVE.md relies on
# it; gen_envrc.sh does not emit it yet).
if [ "${CI:-}" != "true" ]; then
  if [ ! -e "$REPO_ROOT/.envrc" ]; then
    ENVRC_DIR="$REPO_ROOT" "$META/gen_envrc.sh" || true
  fi
  if [ -f "$REPO_ROOT/.envrc" ] \
      && ! grep -q "envrc.local" "$REPO_ROOT/.envrc"; then
    {
      echo ""
      echo "# Machine-local additions (never committed; anything private — e.g."
      echo "# the data-archive location — belongs here, per docs/DATA_ARCHIVE.md)."
      echo "source_env_if_exists .envrc.local"
    } >> "$REPO_ROOT/.envrc"
    echo "note: appended the .envrc.local hook to .envrc; run 'direnv allow' to reload"
  fi
fi

echo ""
echo "mi-lib installed under $PREFIX"
echo "Make sure your environment includes ('direnv allow' covers both):"
echo "  PATH=$PREFIX/bin:\$PATH"
echo "  LD_LIBRARY_PATH=$PREFIX/lib:\$LD_LIBRARY_PATH"
