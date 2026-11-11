#!/usr/bin/env bash
# Regressions for tests/integration/scratch_dir.sh, on a fake home, a fake
# repository (the working directory), a fake build directory and a fake
# temporary directory, with rm, rmdir and mktemp stood in for: each logs its
# call and acts only inside this test's own scratch directory (anything else
# is refused and fails the test).
#
# scratch_init refuses, having changed nothing (no call of any stand-in), an
# empty or relative path, "." or ".." components, a trailing slash, a name
# other than the test's, a missing parent, a parent that is the home
# directory, the repository (a fake one, and the real one holding the guard),
# an ancestor of either, a directory in the home
# with the test's name in it but no build there ("notes-<name>"), a system
# tree, /tmp when it is not the temporary directory, a link to the home, an
# in-source build (the repository holds CMakeCache.txt), and the home, this
# repository or this user's /run directory below a temporary directory that
# holds it;
# it accepts a CMake build directory, a directory below the temporary one,
# and a link to the build directory (the child made under its resolved path),
# from any working directory (one inside the build directory included).
# scratch_rm and scratch_rmdir remove inside the child and refuse, removing
# nothing, an empty path, a path outside, traversal out of the child, a path
# through a link that leads out, a trailing slash, the child itself, and a
# mixed list; a link inside the child is removed as a link, its target kept.
# A tampered RTCTRL_SCRATCH refuses every removal; scratch_release refuses a
# child replaced by a link, and removes the child it made. A test run by
# another makes its child in the caller's own child (refused below it, or
# when the caller's identity or marker does not match) and leaves it there;
# scratch_nested finds exactly one such child.
#
# usage: scratch_dir_test.sh <scratch dir>
set -u
. "$(dirname "$0")/scratch_dir.sh"
scratch_init scratch_dir_test "${1:-}" || exit 2
S=$RTCTRL_SCRATCH
fail() { echo "FAILED: $1"; [ -n "${2:-}" ] && cat "$2"; exit 1; }

# The fake layout, and the stand-ins.
mkdir -p "$S/home/docs" "$S/repo" "$S/build" "$S/tmp/sub" "$S/elsewhere" "$S/stub"
: >"$S/build/CMakeCache.txt"
printf 'PRESERVE\n' >"$S/elsewhere/victim"
ln -s "$S/home" "$S/home-link"
ln -s "$S/build" "$S/build-link"
for cmd in rm rmdir mktemp; do
  real=$(command -v "$cmd")
  cat >"$S/stub/$cmd" <<EOF
#!/usr/bin/env bash
echo "$cmd \$*" >>"$S/calls"
for a in "\$@"; do
  case "\$a" in -*) continue ;; esac
  case "\$(realpath -m -- "\$a")/" in "$S"/*) ;; *) echo "OUTSIDE $cmd \$a" >>"$S/calls"; exit 99 ;; esac
done
exec "$real" "\$@"
EOF
  chmod +x "$S/stub/$cmd"
done
fake() {  # runs "$@" with the fake home, repository, temporary directory and stand-ins; the calls start empty
  rm -f -- "${S:?}/calls"
  (export HOME=$S/home TMPDIR=$S/tmp PATH=$S/stub:$PATH RTCTRL_SCRATCH_PROTECT=$S/repo; cd "$S/repo" && "$@")
}
calls() { cat "$S/calls" 2>/dev/null; }
refused() {  # WHAT DIR: scratch_init refuses DIR, with no call of any stand-in
  local st
  fake bash -c '. "$1"; unset RTCTRL_SCRATCH RTCTRL_SCRATCH_ID; scratch_init case_smoke "$2"' _ "$RTCTRL_SCRATCH_LIB" "$2" 2>"$S/err"; st=$?
  [ "$st" -eq 2 ] && [ -z "$(calls)" ] || fail "scratch_init: $1 ($2): status $st, calls: $(calls)" "$S/err"
}
accepted() {  # WHAT DIR BASE: scratch_init makes the child under BASE, marked
  local out
  out=$(fake bash -c '. "$1"; scratch_init case_smoke "$2" 2>/dev/null && printf "%s\n" "$RTCTRL_SCRATCH"' _ "$RTCTRL_SCRATCH_LIB" "$2") ||
    fail "scratch_init: $1 ($2) refused"
  case "$out" in "$3"/case_smoke.??????) ;; *) fail "scratch_init: $1: the child $out is not under $3" ;; esac
  [ -d "$out" ] && grep -q '^name=case_smoke$' "$out/.rtctrl-scratch" && [ "$(stat -c %a "$out")" = 700 ] || fail "scratch_init: $1: the child $out"
  ! grep -q OUTSIDE "$S/calls" || fail "scratch_init: $1: a call outside" "$S/calls"
}

refused "an empty path" ""
refused "a relative path" "build/case_smoke"
refused "a .. component" "$S/build/x/../case_smoke"
refused "a . component" "$S/build/./case_smoke"
refused "a trailing slash" "$S/build/case_smoke/"
refused "another name" "$S/build/other_smoke"
refused "a name that only contains the test's" "$S/build/my-case_smoke"
refused "a missing parent" "$S/build/missing/case_smoke"
refused "the home as parent" "$S/home/case_smoke"
refused "the review's notes-<name> case, in the home" "$S/home/notes-case_smoke"
refused "a directory in the home without a build" "$S/home/docs/case_smoke"
refused "the repository as parent" "$S/repo/case_smoke"
refused "this repository (the one holding the guard) as parent" "$(realpath -- "$(dirname -- "$0")/../..")/case_smoke"
refused "an ancestor of the home and the repository" "$S/case_smoke"
refused "/ as parent" "/case_smoke"
refused "a system tree" "/usr/case_smoke"
refused "a system tree, /run" "/run/user/$(id -u)/case_smoke"
refused "/tmp when it is not the temporary directory" "/tmp/case_smoke"
refused "the temporary directory itself as parent" "$S/tmp/case_smoke"
refused "a link to the home" "$S/home-link/case_smoke"
# Where only the ancestry rule refuses: an in-source build (the repository holds CMakeCache.txt), and a temporary
# directory that holds the home (a parent in the home would otherwise lie below it).
: >"$S/repo/CMakeCache.txt"
refused "the repository, an in-source build directory" "$S/repo/case_smoke"
rm -f -- "${S:?}/repo/CMakeCache.txt"
refused_tmp() {  # WHAT DIR TMP
  local st
  rm -f -- "${S:?}/calls"
  (export HOME=$S/home TMPDIR=$3 PATH=$S/stub:$PATH RTCTRL_SCRATCH_PROTECT=$S/repo; cd "$S/repo" &&
    bash -c '. "$1"; unset RTCTRL_SCRATCH RTCTRL_SCRATCH_ID; scratch_init case_smoke "$2"' _ "$RTCTRL_SCRATCH_LIB" "$2") 2>"$S/err"; st=$?
  [ "$st" -eq 2 ] && [ -z "$(calls)" ] || fail "scratch_init: $1 ($2): status $st, calls: $(calls)" "$S/err"
}
refused_tmp "the home, below a temporary directory that holds it" "$S/home/case_smoke" "$S"
# Where only the repository rule refuses: a fake repository holding a copy of the guard (its repository is then the
# fake one), below a temporary directory that holds it; and this repository likewise (the mktemp stand-in refuses
# anything outside this test's scratch directory, whatever the guard does).
mkdir -p "$S/fakerepo/tests/integration"
cp -- "$(dirname "$0")/scratch_dir.sh" "$S/fakerepo/tests/integration/scratch_dir.sh"
rm -f -- "${S:?}/calls"
(export HOME=$S/home TMPDIR=$S PATH=$S/stub:$PATH; cd "$S/repo" &&
  bash -c '. "$1"; unset RTCTRL_SCRATCH RTCTRL_SCRATCH_ID; scratch_init case_smoke "$2"' _ "$S/fakerepo/tests/integration/scratch_dir.sh" "$S/fakerepo/case_smoke") 2>"$S/err"
st=$?
[ "$st" -eq 2 ] && [ -z "$(calls)" ] || fail "scratch_init: the repository holding the guard, below the temporary directory: status $st, calls: $(calls)" "$S/err"
REPO=$(realpath -- "$(dirname -- "$RTCTRL_SCRATCH_LIB")/../..")
refused_tmp "this repository, below a temporary directory that holds it" "$REPO/case_smoke" "$(dirname -- "$REPO")"
# Where only the system-tree rule refuses: this user's own directory in /run, below a temporary directory /run/user
# (where that directory exists; the mktemp stand-in refuses anything outside this test's scratch directory anyway).
if [ -d "/run/user/$(id -u)" ] && [ "$(stat -c %u -- "/run/user/$(id -u)")" = "$(id -u)" ]; then
  refused_tmp "this user's directory in /run, below a temporary directory /run/user" "/run/user/$(id -u)/case_smoke" /run/user
else
  echo "scratch_dir_test: no /run/user/$(id -u) of this user's: the system-tree case is not run"
fi
accepted "a CMake build directory" "$S/build/case_smoke" "$S/build"
accepted "a directory below the temporary one" "$S/tmp/sub/case_smoke" "$S/tmp/sub"
accepted "a link to the build directory" "$S/build-link/case_smoke" "$S/build"
# The working directory does not matter: ctest runs a test without one from inside the build directory.
mkdir -p "$S/build/tests"
out=$(rm -f -- "${S:?}/calls"; export HOME=$S/home TMPDIR=$S/tmp PATH=$S/stub:$PATH RTCTRL_SCRATCH_PROTECT=$S/repo; cd "$S/build/tests" &&
  bash -c '. "$1"; scratch_init case_smoke "$2" 2>/dev/null && printf "%s\n" "$RTCTRL_SCRATCH"' _ "$RTCTRL_SCRATCH_LIB" "$S/build/case_smoke") ||
  fail "scratch_init from inside the build directory refused"
case "$out" in "$S/build"/case_smoke.??????) ;; *) fail "scratch_init from inside the build directory: $out" ;; esac

# Removals, from a child made in the fake build directory.
run() {  # SCRIPT: runs SCRIPT (bash) after scratch_init in the fake build directory, with $1 the child
  fake bash -c '. "$1"; scratch_init case_smoke "$2/build/case_smoke" 2>/dev/null || exit 9; C=$RTCTRL_SCRATCH; '"$1" _ "$RTCTRL_SCRATCH_LIB" "$S"
}
out=$(run 'mkdir -p "$C/out/sub" "$C/empty"; : >"$C/out/file"
  scratch_rm "$C/out/file" && [ ! -e "$C/out/file" ] && echo removed-file
  scratch_rm "$C/out" && [ ! -e "$C/out" ] && echo removed-tree
  scratch_rmdir "$C/empty" && [ ! -e "$C/empty" ] && echo removed-empty
  ln -s "'"$S"'/elsewhere" "$C/link"
  scratch_rm "$C/link/victim" 2>/dev/null; echo "through-link $?"
  scratch_rm "$C/link" && [ ! -L "$C/link" ] && echo removed-link
  scratch_within "$C/tracefs/instances/x" && echo within
  scratch_within "'"$S"'/elsewhere" 2>/dev/null || echo not-within
  scratch_release && [ ! -e "$C" ] && echo released')
[ "$out" = "$(printf '%s\n' removed-file removed-tree removed-empty 'through-link 2' removed-link within not-within released)" ] ||
  fail "removals inside the child: $out" "$S/calls"
[ "$(cat "$S/elsewhere/victim")" = PRESERVE ] || fail "the link's target was touched"
for case in '""' '"'"$S"'/elsewhere/victim"' '"$C/../case_smoke-other"' '"$C/out/../../x"' '"$C/out/"' '"$C"' \
            '"$C/ok" "'"$S"'/elsewhere/victim"' '"${UNSET_VARIABLE:-}"'; do
  out=$(run 'mkdir -p "$C/out"; : >"$C/ok"; rm -f -- "'"$S"'/calls"; scratch_rm '"$case"' 2>/dev/null; echo "status $?"; [ -e "$C/ok" ] && echo kept; cat "'"$S"'/calls" 2>/dev/null')
  [ "$(head -n 1 <<<"$out")" = "status 2" ] && grep -q '^kept$' <<<"$out" && ! grep -q '^rm ' <<<"$out" ||
    fail "scratch_rm $case was not refused with nothing removed: $out"
done
[ "$(cat "$S/elsewhere/victim")" = PRESERVE ] || fail "a refused removal touched the victim"
out=$(run 'export RTCTRL_SCRATCH='"$S"'/elsewhere; scratch_rm "$RTCTRL_SCRATCH/victim" 2>/dev/null; echo "status $?"')
[ "$out" = "status 2" ] && [ -e "$S/elsewhere/victim" ] || fail "a tampered RTCTRL_SCRATCH: $out"
out=$(run 'mv "$C" "$C.moved"; ln -s "'"$S"'/elsewhere" "$C"; scratch_release 2>/dev/null; echo "status $?"; rm -- "$C"; mv "$C.moved" "$C"; scratch_release && echo released')
[ "$out" = "$(printf '%s\n' 'status 2' released)" ] && [ -e "$S/elsewhere/victim" ] || fail "a child replaced by a link: $out"

# Nested children: a test run by another may make its child in the caller's own child (still the one its
# scratch_init made and marked), never below it, and leaves it for the caller's release.
cat >"$S/inner.sh" <<'EOF'
#!/usr/bin/env bash
. "$RTCTRL_SCRATCH_LIB" || exit 9
scratch_init "$1" "$2" || exit 2
: >"$RTCTRL_SCRATCH/output"
scratch_release || exit 3
printf '%s\n' "$RTCTRL_SCRATCH"
EOF
out=$(run 'I=$(bash "'"$S"'/inner.sh" inner_smoke "$C/inner_smoke" 2>/dev/null); echo "inner $?"
  case "$I" in "$C"/inner_smoke.??????) echo under-caller ;; esac
  [ -f "$I/output" ] && echo left-for-caller
  [ "$(scratch_nested inner_smoke)" = "$I" ] && echo found
  mkdir -p "$C/sub"; bash "'"$S"'/inner.sh" inner_smoke "$C/sub/inner_smoke" 2>/dev/null; echo "below $?"
  RTCTRL_SCRATCH_ID=0:0 bash "'"$S"'/inner.sh" inner_smoke "$C/inner_smoke" 2>/dev/null; echo "wrong-id $?"
  cp -- "$C/.rtctrl-scratch" "$C/mark.bak"; printf "name=case_smoke\nid=0:0\n" >|"$C/.rtctrl-scratch"
  bash "'"$S"'/inner.sh" inner_smoke "$C/inner_smoke" 2>/dev/null; echo "marker $?"
  mv -- "$C/mark.bak" "$C/.rtctrl-scratch"
  set -- "$C"/inner_smoke.?????? "$C"/sub/inner_smoke.??????; [ $# -eq 2 ] && [ ! -e "$2" ] && echo one-child
  scratch_nested missing_smoke 2>/dev/null; echo "none $?"
  bash "'"$S"'/inner.sh" inner_smoke "$C/inner_smoke" >/dev/null 2>&1; scratch_nested inner_smoke >/dev/null 2>&1; echo "two $?"
  scratch_release && [ ! -e "$C" ] && echo released')
[ "$out" = "$(printf '%s\n' 'inner 0' under-caller left-for-caller found 'below 2' 'wrong-id 2' 'marker 2' one-child 'none 1' 'two 1' released)" ] ||
  fail "nested children: $out" "$S/calls"

scratch_release
echo "scratch dir test OK"
