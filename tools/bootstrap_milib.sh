#!/bin/sh
# Build and install the mi-lib stack from the third_party submodules in
# dependency order. Re-running rebuilds and reinstalls (idempotent).
#
# Usage: tools/bootstrap_milib.sh [PREFIX]
#   PREFIX      installation prefix (default: $MILIB_PREFIX or $HOME/usr)
#   MILIB_LIBS  space-separated subset to build (default: full stack);
#               CI uses "zeda zm zeo dzco roki roki-fd liw" to skip the
#               X11/GL-dependent zx11 and roki-gl.
#
# System packages required: build-essential, libxml2-dev, liblzf-dev
# (plus X11/GL dev packages for zx11/roki-gl).
set -eu

REPO_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
PREFIX=${1:-${MILIB_PREFIX:-$HOME/usr}}

# zeda must come first: it installs zeda-makefile-gen used by every other lib.
LIBS=${MILIB_LIBS:-"zeda zm zeo dzco roki roki-fd liw zx11 roki-gl"}

export PATH="$PREFIX/bin:$PATH"
mkdir -p "$PREFIX/bin" "$PREFIX/lib" "$PREFIX/include"

for lib in $LIBS; do
  dir="$REPO_ROOT/third_party/$lib"
  if [ ! -f "$dir/config.org" ]; then
    echo "error: $dir is missing (run: git submodule update --init third_party/$lib)" >&2
    exit 1
  fi
  echo "=== building $lib ==="
  # The generated makefile does `include ../config`; seed it from the
  # template with our PREFIX.
  sed "s|^PREFIX=.*|PREFIX=$PREFIX|" "$dir/config.org" > "$dir/config"
  make -C "$dir"
  make -C "$dir" install
done

echo ""
echo "mi-lib installed under $PREFIX"
echo "Make sure your environment includes:"
echo "  PATH=$PREFIX/bin:\$PATH"
echo "  LD_LIBRARY_PATH=$PREFIX/lib:\$LD_LIBRARY_PATH"
