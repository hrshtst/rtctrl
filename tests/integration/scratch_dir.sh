# shellcheck shell=bash
# tests/integration/scratch_dir.sh: a test's own scratch directory, and the
# only removals a test makes, sourced by the smoke tests that remove files.
#
# A test never removes the directory it is given. scratch_init creates a new,
# uniquely named child beside it, under an approved base, and every removal
# the test makes goes through scratch_rm or scratch_rmdir, confined to that
# child; scratch_release removes the child itself.
#
# scratch_init NAME DIR
#   Validates DIR before any change to the filesystem, then creates the child
#   NAME.XXXXXX (mktemp -d, mode 0700) in DIR's parent and marks it. Returns 2,
#   having changed nothing, unless: NAME is a plain name; DIR is absolute,
#   holds no "." or ".." component and no empty last component, and its last
#   component is NAME exactly; and DIR's parent, resolved (links followed), is
#   an existing directory owned by this user that is neither /, the home
#   directory, the repository (the one holding this file) nor an ancestor of
#   them, nor a directory listed in RTCTRL_SCRATCH_PROTECT (colon-separated;
#   it only adds protection, for this file's own test) or an ancestor of one,
#   lies in no
#   system tree (/bin, /boot, /dev, /etc, /lib*, /opt, /proc, /root, /run,
#   /sbin, /snap, /srv, /sys, /usr, /var), and is an approved base: a CMake
#   build directory (it holds CMakeCache.txt), a directory strictly below
#   the temporary directory (${TMPDIR:-/tmp}, resolved), or the calling
#   test's own child (the RTCTRL_SCRATCH it exported, still the directory its
#   scratch_init made and marked: a nested child, see scratch_release). On
#   success it sets and exports RTCTRL_SCRATCH (the child), RTCTRL_SCRATCH_ID
#   (its device and inode) and RTCTRL_SCRATCH_LIB (this file, for the test's
#   stand-in scripts to source), and names the child on stderr. Call it in
#   the test's own shell (not in a command substitution):
#   scratch_init NAME "$1" || exit 2.
# scratch_within PATH...
#   Status 0 only if every PATH lies inside the child: absolute, no "." or ".."
#   component, no empty last component, not the child itself, and its parent,
#   resolved, the child or below it (its last component is not resolved: a
#   link there is the link, never its target); and the child is still the
#   directory scratch_init made (the same device and inode, not a link).
# scratch_rm PATH...
#   Checks every PATH with scratch_within first and removes none if any fails
#   (status 2); then removes each, recursively (rm -rf -- "${path:?}").
# scratch_rmdir PATH...
#   The same, with rmdir (empty directories only).
# scratch_release
#   Removes the child itself, only if it is still that directory (the same
#   device and inode, not a link) and holds its own marker; status 2 if not
#   (a removal that fails part-way is reported and the child kept, status 0).
#   A nested child is left in place, for the calling test to read and for its
#   own release to remove.
# scratch_nested NAME
#   In the calling test: prints the one nested child NAME.XXXXXX that a test
#   it ran made in its child (a directory, no link, marked with NAME and its
#   own device and inode); status 1 if there is none or more than one.
# Every command these functions run is a plain command looked up on PATH, so
# that a test can stand in for rm, rmdir and mktemp.

scratch__plain_path() {  # PATH: absolute, no "." or ".." component, no empty last component (no expansion of PATH)
  case "$1" in /*) ;; *) return 1 ;; esac
  case "$1" in */) return 1 ;; esac
  case "$1/" in */./* | */../*) return 1 ;; esac
  return 0
}
scratch__inside() {  # CANDIDATE BASE: status 0 if CANDIDATE is BASE or below it (both resolved)
  [ "$1" = "$2" ] && return 0
  case "$1/" in "$2"/*) return 0 ;; esac
  return 1
}
scratch__id() {  # PATH: its device and inode, if it is a directory and no link
  [ -d "$1" ] && [ ! -L "$1" ] && stat -c '%d:%i' -- "$1"
}
scratch__live() {  # DIR ID: status 0 if DIR is still the scratch directory ID names (no link, its own marker)
  [ -n "$1" ] && [ -n "$2" ] && [ "$(scratch__id "$1")" = "$2" ] && [ -f "$1/.rtctrl-scratch" ] &&
    [ ! -L "$1/.rtctrl-scratch" ] && grep -qx "id=$2" "$1/.rtctrl-scratch"
}

scratch_init() {
  local name=${1:-} dir=${2:-} parent canon d tmp child outer=
  case "$name" in '' | *[!A-Za-z0-9._-]*) echo "scratch_init: not a plain test name: '$name'" >&2; return 2 ;; esac
  [ -n "$dir" ] || { echo "$name: no scratch directory given" >&2; return 2; }
  scratch__plain_path "$dir" || { echo "$name: not an absolute path without . or .. components: $dir" >&2; return 2; }
  [ "${dir##*/}" = "$name" ] || { echo "$name: the scratch directory must be named $name: $dir" >&2; return 2; }
  parent=${dir%/*}
  [ -n "$parent" ] || { echo "$name: $dir lies directly in /" >&2; return 2; }
  canon=$(realpath -e -- "$parent" 2>/dev/null) && [ -n "$canon" ] && [ -d "$canon" ] ||
    { echo "$name: the parent of $dir does not exist" >&2; return 2; }
  [ "$(stat -c %u -- "$canon")" = "$(id -u)" ] || { echo "$name: $canon is not this user's" >&2; return 2; }
  local repo protect=()
  repo=$(realpath -m -- "$(dirname -- "${BASH_SOURCE[0]}")/../..") || return 2
  IFS=: read -r -a protect <<<"${RTCTRL_SCRATCH_PROTECT:-}"
  for d in / "${HOME:-/}" "$repo" "${protect[@]}"; do
    [ -n "$d" ] || continue
    d=$(realpath -m -- "$d") || return 2
    scratch__inside "$d" "$canon" && { echo "$name: $canon is, or holds, $d" >&2; return 2; }
  done
  for d in /bin /boot /dev /etc /lib /lib32 /lib64 /libx32 /opt /proc /root /run /sbin /snap /srv /sys /usr /var; do
    scratch__inside "$canon" "$d" && { echo "$name: $canon lies in $d" >&2; return 2; }
  done
  tmp=$(realpath -m -- "${TMPDIR:-/tmp}") || return 2
  if [ -f "$canon/CMakeCache.txt" ] && [ ! -L "$canon/CMakeCache.txt" ]; then :
  elif [ "$canon" != "$tmp" ] && scratch__inside "$canon" "$tmp"; then :
  elif scratch__live "${RTCTRL_SCRATCH:-}" "${RTCTRL_SCRATCH_ID:-}" &&
    [ "$canon" = "$(realpath -e -- "$RTCTRL_SCRATCH")" ]; then
    outer=$canon
  else
    echo "$name: $canon is no approved base (a CMake build directory, a directory below $tmp, or the calling test's own scratch directory)" >&2
    return 2
  fi
  # Validated: the first change to the filesystem.
  child=$(mktemp -d -- "$canon/$name.XXXXXX") && [ -n "$child" ] || { echo "$name: cannot create a scratch directory in $canon" >&2; return 2; }
  RTCTRL_SCRATCH=$child
  RTCTRL_SCRATCH_OUTER=$outer
  RTCTRL_SCRATCH_ID=$(scratch__id "$child") || { echo "$name: $child is not the directory just made" >&2; return 2; }
  ( set -o noclobber; printf 'name=%s\nid=%s\n' "$name" "$RTCTRL_SCRATCH_ID" >"$child/.rtctrl-scratch" ) ||
    { echo "$name: cannot mark $child" >&2; return 2; }
  RTCTRL_SCRATCH_LIB=$(realpath -e -- "${BASH_SOURCE[0]}")
  export RTCTRL_SCRATCH RTCTRL_SCRATCH_ID RTCTRL_SCRATCH_LIB
  echo "$name: scratch directory $child" >&2
}

scratch_within() {
  local s p parent
  [ -n "${RTCTRL_SCRATCH:-}" ] && [ -n "${RTCTRL_SCRATCH_ID:-}" ] || { echo "scratch: no scratch directory" >&2; return 1; }
  [ "$(scratch__id "$RTCTRL_SCRATCH")" = "$RTCTRL_SCRATCH_ID" ] || { echo "scratch: $RTCTRL_SCRATCH is no longer the scratch directory" >&2; return 1; }
  s=$(realpath -e -- "$RTCTRL_SCRATCH") || return 1
  [ $# -gt 0 ] || return 1
  for p in "$@"; do
    [ -n "$p" ] && scratch__plain_path "$p" || { echo "scratch: refused '$p': not an absolute path without . or .. components" >&2; return 1; }
    [ "$(realpath -m -- "$p")" != "$s" ] || { echo "scratch: refused '$p': the scratch directory itself" >&2; return 1; }
    parent=$(realpath -m -- "${p%/*}") || return 1
    scratch__inside "$parent" "$s" || { echo "scratch: refused '$p': outside $s" >&2; return 1; }
  done
}

scratch_rm() {
  scratch_within "$@" || return 2
  local p
  for p in "$@"; do
    rm -rf -- "${p:?}" || return 2
  done
}

scratch_rmdir() {
  scratch_within "$@" || return 2
  local p
  for p in "$@"; do
    rmdir -- "${p:?}" || return 2
  done
}

scratch_release() {
  local s=${RTCTRL_SCRATCH:-}
  scratch__live "$s" "${RTCTRL_SCRATCH_ID:-}" || { echo "scratch: $s is not the scratch directory made; kept" >&2; return 2; }
  if [ -n "${RTCTRL_SCRATCH_OUTER:-}" ]; then
    echo "scratch: $s left for the release of $RTCTRL_SCRATCH_OUTER" >&2
    return 0
  fi
  rm -rf -- "${s:?}" || echo "scratch: $s could not be removed completely; kept" >&2
  return 0
}

scratch_nested() {
  local name=${1:-} d found=
  case "$name" in '' | *[!A-Za-z0-9._-]*) echo "scratch_nested: not a plain test name: '$name'" >&2; return 1 ;; esac
  scratch__live "${RTCTRL_SCRATCH:-}" "${RTCTRL_SCRATCH_ID:-}" || { echo "scratch: no scratch directory" >&2; return 1; }
  for d in "$RTCTRL_SCRATCH/$name".??????; do
    [ -d "$d" ] && [ ! -L "$d" ] && [ -f "$d/.rtctrl-scratch" ] && [ ! -L "$d/.rtctrl-scratch" ] &&
      grep -qx "name=$name" "$d/.rtctrl-scratch" && grep -qx "id=$(scratch__id "$d")" "$d/.rtctrl-scratch" || continue
    [ -z "$found" ] || { echo "scratch: more than one $name directory in $RTCTRL_SCRATCH" >&2; return 1; }
    found=$d
  done
  [ -n "$found" ] || { echo "scratch: no $name directory in $RTCTRL_SCRATCH" >&2; return 1; }
  printf '%s\n' "$found"
}
