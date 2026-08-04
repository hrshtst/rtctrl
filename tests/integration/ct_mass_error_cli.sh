#!/usr/bin/env bash
# Seed-parsing contract of x7_ct_mass_error (review finding on
# 7f73334): --seed is a uint64 parsed exactly. The double-based
# parser accepted inf and out-of-range values (UB on conversion) and
# collapsed seeds above 2^53, silently breaking the advertised
# bit-reproducibility of a seed.
set -eu
BIN="$1"
OUT="$2"
mkdir -p "$OUT"

# Rejections: infinity, sign, fraction, 2^64 overflow, empty.
for bad in inf -1 1.5 18446744073709551616 ""; do
  if "$BIN" --seed "$bad" --out "$OUT/bad.csv" >/dev/null 2>&1; then
    echo "accepted invalid seed: '$bad'"
    exit 1
  fi
done

# --zvs requires a NON-EMPTY value (empty would silently alias the
# option-absent state and skip logging without a word).
if "$BIN" --out "$OUT/bad.csv" --zvs >/dev/null 2>&1; then
  echo "accepted --zvs without a value"
  exit 1
fi
if "$BIN" --out "$OUT/bad.csv" --zvs "" >/dev/null 2>&1; then
  echo "accepted an empty --zvs value"
  exit 1
fi

# --out/--zvs collisions are rejected on NORMALIZED paths: the same
# file opened twice interleaves CSV rows and zvs frames.
if "$BIN" --out "$OUT/same.csv" --zvs "$OUT/same.csv" >/dev/null 2>&1; then
  echo "accepted identical --out and --zvs paths"
  exit 1
fi
if "$BIN" --out "$OUT/same.csv" \
    --zvs "$OUT/../$(basename "$OUT")/same.csv" >/dev/null 2>&1; then
  echo "accepted an aliased --out/--zvs path"
  exit 1
fi

# Name normalization cannot see file IDENTITY: a hard link to the CSV
# and a symlink that dangles until the CSV is created must both be
# rejected by the post-open equivalent() check.
rm -f "$OUT/hl.csv" "$OUT/hl.zvs" "$OUT/dang.csv" "$OUT/dang.zvs"
touch "$OUT/hl.csv" && ln "$OUT/hl.csv" "$OUT/hl.zvs"
if "$BIN" --out "$OUT/hl.csv" --zvs "$OUT/hl.zvs" >/dev/null 2>&1; then
  echo "accepted a hard-linked --out/--zvs pair"
  exit 1
fi
ln -s "$OUT/dang.csv" "$OUT/dang.zvs"  # dangles until the CSV opens
if "$BIN" --out "$OUT/dang.csv" --zvs "$OUT/dang.zvs" >/dev/null 2>&1; then
  echo "accepted a dangling-symlink --zvs onto the CSV"
  exit 1
fi

# The uint64 maximum is a valid seed; --zvs writes one 9-coordinate
# frame per control cycle (451 for the fixed round trip).
"$BIN" --mass-error 0.2 --com-error 0.01 --seed 18446744073709551615 \
  --out "$OUT/max.csv" --zvs "$OUT/max.zvs" > /dev/null
frames=$(wc -l < "$OUT/max.zvs")
if [ "$frames" -ne 451 ]; then
  echo "expected 451 zvs frames, got $frames"
  exit 1
fi

# 2^53 and 2^53+1 are DISTINCT seeds (indistinguishable as doubles):
# their perturbed runs must differ.
"$BIN" --mass-error 0.2 --com-error 0.01 --seed 9007199254740992 \
  --out "$OUT/p53.csv" > /dev/null
"$BIN" --mass-error 0.2 --com-error 0.01 --seed 9007199254740993 \
  --out "$OUT/p53_1.csv" > /dev/null
if cmp -s "$OUT/p53.csv" "$OUT/p53_1.csv"; then
  echo "seeds 2^53 and 2^53+1 collapsed to the same run"
  exit 1
fi
echo "seed CLI contract holds"
