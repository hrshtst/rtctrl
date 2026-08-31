#!/usr/bin/env bash
# zvs_to_mp4.sh: render a .zvs joint-displacement sequence to an mp4 clip.
#
# Drives rk_anim's offline frame capture (-captureserial) and assembles the
# frames with ffmpeg. Needs a reachable X display (the capture window opens
# briefly); the export itself runs unthrottled, faster than real time.
#
# Usage:
#   tools/zvs_to_mp4.sh [options] <sequence.zvs> <output.mp4> [-- <rk_anim args>]
#
# Options:
#   -m, --model <file>        chain model (default: models/crane_x7/crane_x7.ztk)
#   -s, --secperframe <sec>   sequence time per captured frame (default: rk_anim's
#                             0.033; use the .zvs dt, e.g. 0.01, for frame-exact export)
#   --fps <rate>              output frame rate (default: 1/secperframe)
#   -k, --keep-frames <dir>   keep captured PNG frames in <dir> (default: temp dir)
#   -h, --help                show this help
#
# Everything after `--` is passed to rk_anim verbatim and appended after the
# default camera options, so single options can be overridden (zeda's parser
# lets the last occurrence win). Default camera: side view from -y,
#   -x 0 -y -- -1.8 -z 0.3 -pan -- -90
# Note rk_anim option values that are negative need a bare `--` before them.
#
# Examples:
#   # PTP plan preview, default side view at 30 fps
#   tools/zvs_to_mp4.sh ptp.zvs /tmp/ptp.mp4
#
#   # frame-exact export of a 0.01 s-step sequence (100 fps), keep the PNGs
#   tools/zvs_to_mp4.sh -s 0.01 -k /tmp/frames follow-pick.zvs /tmp/follow.mp4
#
#   # custom rk_anim options: camera pulled back and tilted down a little
#   # (bare -- before each negative value), timestamp strip blanked
#   tools/zvs_to_mp4.sh ptp.zvs /tmp/ptp.mp4 -- -y -- -2.2 -z 0.5 -tilt -- -10 -notimestamp
#
#   # another model and a bigger window, frames kept for inspection
#   tools/zvs_to_mp4.sh -m other/model.ztk -k /tmp/frames seq.zvs /tmp/seq.mp4 -- -width 800 -height 800

set -u

usage() { awk 'NR > 1 && !/^#/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "$0"; exit "${1:-0}"; }
die() { echo "zvs_to_mp4: error: $*" >&2; exit 1; }

repo_root=$(cd "$(dirname "$0")/.." && pwd)
model="$repo_root/models/crane_x7/crane_x7.ztk"
secperframe=""
fps=""
keep_dir=""
extra=()
positional=()

while [ $# -gt 0 ]; do
  case "$1" in
    -m|--model)       [ $# -ge 2 ] || die "$1 needs a value"; model=$2; shift 2 ;;
    -s|--secperframe) [ $# -ge 2 ] || die "$1 needs a value"; secperframe=$2; shift 2 ;;
    --fps)            [ $# -ge 2 ] || die "$1 needs a value"; fps=$2; shift 2 ;;
    -k|--keep-frames) [ $# -ge 2 ] || die "$1 needs a value"; keep_dir=$2; shift 2 ;;
    -h|--help)        usage 0 ;;
    --)               shift; extra=("$@"); break ;;
    -*)               die "unknown option: $1 (see --help)" ;;
    *)                positional+=("$1"); shift ;;
  esac
done
[ ${#positional[@]} -eq 2 ] || { echo "zvs_to_mp4: expected <sequence.zvs> <output.mp4>" >&2; usage 1 >&2; }
zvs=${positional[0]}
out=${positional[1]}

command -v rk_anim > /dev/null || die "rk_anim not found on PATH (direnv not loaded?)"
command -v ffmpeg  > /dev/null || die "ffmpeg not found on PATH"
[ -r "$zvs" ]   || die "cannot read sequence file: $zvs"
[ -r "$model" ] || die "cannot read model file: $model"
xwininfo -root > /dev/null 2>&1 || die "cannot connect to X display '${DISPLAY:-unset}'"

if [ -z "$fps" ]; then
  fps=$(awk -v s="${secperframe:-0.033}" 'BEGIN { printf "%.6g", 1 / s }')
fi

if [ -n "$keep_dir" ]; then
  mkdir -p "$keep_dir" || die "cannot create frame directory: $keep_dir"
  frame_dir=$keep_dir
else
  frame_dir=$(mktemp -d) || die "mktemp failed"
  trap 'rm -rf "$frame_dir"' EXIT
fi

anim_args=(-captureserial png -title "$frame_dir/frame"
           -x 0 -y -- -1.8 -z 0.3 -pan -- -90)
[ -n "$secperframe" ] && anim_args+=(-secperframe "$secperframe")
anim_args+=(${extra[@]+"${extra[@]}"})

echo "zvs_to_mp4: capturing '$zvs' (model: $model)"
rk_anim "$model" "$zvs" "${anim_args[@]}"
status=$?
[ $status -eq 0 ] || die "rk_anim failed (exit $status)"
nframes=$(find "$frame_dir" -maxdepth 1 -name 'frame*.png' | wc -l)
[ "$nframes" -gt 0 ] || die "rk_anim wrote no frames to $frame_dir"

echo "zvs_to_mp4: assembling $nframes frames at $fps fps"
ffmpeg -y -loglevel error -framerate "$fps" -i "$frame_dir/frame%05d.png" \
  -c:v libx264 -pix_fmt yuv420p "$out"
status=$?
[ $status -eq 0 ] || die "ffmpeg failed (exit $status)"

duration=$(ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 "$out" 2>/dev/null)
echo "zvs_to_mp4: wrote $out (${duration:-?} s, $nframes frames at $fps fps)"
