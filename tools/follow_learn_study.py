#!/usr/bin/env python3
"""Simulation acceptance study of learning between follow trials.

Runs the trial loops of a study plan with the built binaries and evaluates
the plan's predeclared criteria on what the runs recorded. It invokes only
x7_plan_ptp, x7_follow_sim and x7_follow_learn, none of which opens the bus.

One case is a reference, a follow (simulation) configuration, a learn
configuration, a method (ilc or fel), a number of trials, optional per-trial
perturbations and criteria. For each case the study writes the case's follow
and learn configurations into a directory of its own, runs `init` (and
`init-weights` for fel), then for every trial k runs

  x7_follow_sim --config F --offset offset_k.csv --log trial_k.csv
                --motion trial_k.zvs

and, after every trial but the last, the update with --report (report_k.toml
reads trial k and writes offset_k+1). Any nonzero exit, or a run whose result
the study cannot confirm (the follower's acceptance line for exactly the
offset file passed, an offset's lineage), stops that case and is recorded
verbatim; nothing is retried with other settings.

Recorded per trial: the simulator's exit status and result line; the
aggregate tracking RMS over all 8 joints and over arm joints 0-6 (from the
update's report, the last trial's from its log through follow_learnability);
the per-joint tracking RMS; the RMS of the error above the band split (default
3 Hz, follow_learnability.anatomy), the proxy for vibration the learning may
add; the offset's max |delta| and max rate; the ILC clamp counts of the
update. "RMS" in the criteria is the aggregate over all 8 joints.

The plan (TOML; paths relative to the plan file):

  format = "rtctrl-learn-study-plan"
  version = 1
  split_hz = 3.0                 # optional band split [Hz]

  [references.NAME]              # exactly one source:
  plan = "ptp.toml"              #   planned by x7_plan_ptp, or
  zvs = "ref.zvs"                #   a prepared reference, copied, or
  from = "OTHER"                 #   OTHER without its leading frames:
  cut_leading_s = 0.6            #     those that start before 0.6 s
                                 #     (or cut_leading_frames = N)

  [defaults]                     # every case starts from these
  reference = "NAME"
  follow = "follow.toml"         # base follow configuration
  learn = "learn.toml"           # base learn configuration
  method = "ilc"                 # or "fel"
  trials = 6                     # trials 0..5: five updates
  follow_set = {...}             # values set in the follow configuration
  follow_scale = {...}           # factors on its numbers or number arrays
  learn_set = {...}              # values set in the learn configuration

  [criteria]
  SET = [{ id = "a", check = "runs_ok" }, ...]

  [[case]]
  name = "N1"
  note = "..."                   # optional
  like = "CASE"                  # optional: start from an earlier case
  ...                            # any key of [defaults], then:
  perturb = [{ trials = [2], follow_set = {...}, follow_scale = {...} }]
  transfer_to = "NAME"           # fel: synth for NAME, run it zero/learned
  criteria = "SET"               # or an inline list; never inherited

  [[sweep]]                      # exploratory: one case per combination
  name = "phi"
  like = "CASE"                  # optional
  ...                            # overrides as in a case, no criteria
  vary = { "learn.ilc.phi" = [0.25, 0.5],
           "follow.simulation.command_delay_cycles" = [1, 3] }

The *_set and *_scale tables are nested like the configuration they change
and merge into what the case starts from. The follow configuration's input
paths stay valid in the copy (they are made absolute); `reference` and
`[learning] offset` belong to the study and cannot be set.

Checks (trial indices count from 0; `at_most` is a factor, inclusive):
  runs_ok      every planned trial and update ran, every simulation
               succeeded, and the follower accepted every offset as written
  decreasing   RMS strictly decreasing over trials `from`..`to` (default all)
  ratio        RMS(trial) <= at_most * RMS(of)
  bounded      RMS(k) <= at_most * RMS(of) for every k in `trials`
  no_clamp     no ILC clamp changed a sample in any update
  hf_ratio     above-split RMS(trial) <= at_most * above-split RMS(of)
  vs_case      RMS(k) <= at_most * RMS(k) of case `case`, for k in `trials`
  transfer     RMS with the synthesised offset <= at_most * RMS with zero

  uv run --project tools python tools/follow_learn_study.py PLAN.toml \\
      [--out DIR] [--bin-dir build/apps] [--jobs N]

DIR (default build/learn_study/PLAN-STEM-YYYYmmdd-HHMMSS) must not exist; it
receives a directory per case, the references, a copy of the plan and
summary.json (SI units). Cases run one after the other; --jobs N runs N at a
time, each in a child process of this script (`--run-case JOB.json`, for
that use only), in its own directory. Standard output gets one line per case: its name,
the verdict of each criterion and the RMS curve in mrad.

Exit status: 0 when every case completed and every criterion passed, 1 when
a criterion failed or a case stopped, 2 on a usage error, an invalid plan or
a reference that could not be prepared.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import datetime
import hashlib
import itertools
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
import tomllib
import traceback
from pathlib import Path

# The analyses are small: BLAS threads would only spin, the more so with
# cases in parallel processes.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np  # noqa: E402

import follow_learnability as fl  # noqa: E402

PLAN_FORMAT = "rtctrl-learn-study-plan"
PLAN_VERSION = 1
SUMMARY_FORMAT = "rtctrl-learn-study-summary"
SUMMARY_VERSION = 1
# The only programs the study runs. None of them opens the bus.
BINARIES = ("x7_plan_ptp", "x7_follow_sim", "x7_follow_learn")
METHODS = ("ilc", "fel")
DOF = 8
ARM = 7
COMMAND_TIMEOUT_S = 900.0
EDGE_S = 1.0          # the start and end windows of the edge RMS
DEFAULT_SPLIT_HZ = 3.0

# Input paths of a follow configuration (relative to the file naming them).
FOLLOW_PATH_KEYS = (("model",), ("reference",), ("hardware_config",),
                    ("motor_parameters",), ("simulation", "link_force"),
                    ("learning", "offset"))
# Set by the study itself; a plan cannot set them.
STUDY_KEYS = (("reference",), ("learning", "offset"))

DEFAULT_KEYS = ("reference", "follow", "learn", "method", "trials",
                "follow_set", "follow_scale", "learn_set")
CASE_KEYS = ("name", "note", "like", *DEFAULT_KEYS, "perturb", "transfer_to",
             "criteria")
SWEEP_KEYS = ("name", "note", "like", *DEFAULT_KEYS, "perturb", "vary")
PERTURB_KEYS = ("trials", "follow_set", "follow_scale")
MERGED_KEYS = ("follow_set", "follow_scale", "learn_set")
REFERENCE_SOURCES = ("plan", "zvs", "from")

CHECK_PARAMETERS = {
    "runs_ok": ((), ()),
    "decreasing": ((), ("from", "to")),
    "ratio": (("trial", "of", "at_most"), ()),
    "bounded": (("trials", "of", "at_most"), ()),
    "no_clamp": ((), ()),
    "hf_ratio": (("trial", "of", "at_most"), ()),
    "vs_case": (("case", "trials", "at_most"), ()),
    "transfer": (("at_most",), ()),
}

NAME_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
ZVS_FRAME = re.compile(
    r"(?P<dt>\S+)[ \t]+(?P<dim>\d+)[ \t]*\((?P<values>[^()]*)\)[ \t]*")
ACCEPTED = re.compile(r"command offset accepted: method (\S+), trial (\d+), "
                      r"max \|delta\| (\S+) rad, sha256 ([0-9a-f]{64})")
RESULT = re.compile(r"follow simulation: (\S+?),")


class StudyError(Exception):
    """An invalid plan, input or setup; the message names it."""


class CaseStop(Exception):
    """A step of a case failed; `record` says which and how."""

    def __init__(self, record: dict):
        super().__init__(record.get("reason", "stopped"))
        self.record = record


# ------------------------------------------------------------------ files

def sha256_file(path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def zvs_frames(text: str, source: str = "zvs") -> list:
    """The frames of a ZVS text as written by ZvsWriter, one per line:
    'DT DIM (v1 ... vDIM )'. Returns (dt, values, line) per frame, `line`
    with its line ending. Anything else is refused."""
    frames = []
    for number, line in enumerate(text.splitlines(keepends=True), 1):
        body = line.rstrip("\r\n")
        match = ZVS_FRAME.fullmatch(body)
        if match is None:
            raise StudyError(f"{source}:{number}: not a ZVS frame line "
                             f"'DT DIM (values)'")
        try:
            dt = float(match["dt"])
            values = [float(v) for v in match["values"].split()]
        except ValueError as error:
            raise StudyError(f"{source}:{number}: not a number") from error
        if not (math.isfinite(dt) and dt > 0.0):
            raise StudyError(f"{source}:{number}: dt must be finite and > 0")
        if len(values) != int(match["dim"]):
            raise StudyError(f"{source}:{number}: {len(values)} values, "
                             f"dimension {match['dim']}")
        if not all(math.isfinite(v) for v in values):
            raise StudyError(f"{source}:{number}: a value is not finite")
        frames.append((dt, values, line))
    if not frames:
        raise StudyError(f"{source}: no frames")
    if not frames[-1][2].endswith("\n"):
        raise StudyError(f"{source}: no final newline")
    return frames


def frames_before(frames: list, seconds: float) -> int:
    """How many leading frames start before `seconds`. Frame i starts at
    the sum of the dt of frames 0..i-1 (ZvsTrajectory's time base); a start
    within 1e-9 s of `seconds` counts as at it."""
    start, count = 0.0, 0
    for dt, _, _ in frames:
        if start >= seconds - 1e-9:
            break
        count += 1
        start += dt
    return count


def cut_leading_frames(text: str, frames: int, source: str = "zvs"):
    """`text` without its first `frames` frames (whole frames, every other
    line kept byte for byte), and the time those frames spanned. At least
    two frames must remain."""
    parsed = zvs_frames(text, source)
    if isinstance(frames, bool) or not isinstance(frames, int) or frames < 0:
        raise StudyError(f"{source}: the frame count to cut must be an "
                         f"integer >= 0")
    if len(parsed) - frames < 2:
        raise StudyError(f"{source}: cutting {frames} of {len(parsed)} "
                         f"frames leaves fewer than 2")
    removed = sum(dt for dt, _, _ in parsed[:frames])
    return "".join(line for _, _, line in parsed[frames:]), removed


def parse_offset(path) -> dict:
    """The preamble and the value statistics of an offset file."""
    preamble, rows, header = {}, [], None
    with open(path) as stream:
        for line in stream:
            line = line.rstrip("\n")
            if line.startswith("# "):
                key, _, value = line[2:].partition(":")
                preamble[key] = value.strip()
            elif header is None:
                header = line.split(",")
            elif line:
                rows.append([float(v) for v in line.split(",")])
    if header is None or len(header) != DOF + 1 or not rows:
        raise StudyError(f"{path}: not an offset file")
    values = np.array(rows, dtype=float)[:, 1:]
    dt = float(preamble["dt_s"])
    rate = (np.abs(np.diff(values, axis=0)) / dt if len(values) > 1
            else np.zeros((1, DOF)))
    return {
        "path": str(path),
        "sha256": sha256_file(path),
        "method": preamble.get("method"),
        "trial": int(preamble["trial"]),
        "samples": len(values),
        "dt_s": dt,
        "reference_sha256": preamble.get("reference_sha256"),
        "parent_sha256": preamble.get("parent_sha256"),
        "source_log_sha256": preamble.get("source_log_sha256"),
        "max_abs_rad": float(np.max(np.abs(values))),
        "max_abs_rad_by_joint": [float(v) for v in
                                 np.max(np.abs(values), axis=0)],
        "max_rate_rad_s": float(np.max(rate)),
        "max_rate_rad_s_by_joint": [float(v) for v in np.max(rate, axis=0)],
    }


# ------------------------------------------------------------------- TOML

def _toml_key(key: str) -> str:
    return key if re.fullmatch(r"[A-Za-z0-9_-]+", key) else json.dumps(key)


def toml_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if math.isnan(value):
            return "nan"
        if math.isinf(value):
            return "inf" if value > 0 else "-inf"
        return repr(value)
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, list):
        if any(isinstance(item, dict) for item in value):
            raise StudyError("arrays of tables are not supported")
        return "[" + ", ".join(toml_value(item) for item in value) + "]"
    raise StudyError(f"cannot write a {type(value).__name__} as TOML")


def toml_text(table: dict, header: str = "") -> str:
    """`table` as TOML: its values, then each sub-table under [a.b]."""
    lines = [f"# {line}" if line else "#" for line in header.splitlines()]

    def emit(node: dict, prefix: tuple):
        values = [(k, v) for k, v in node.items() if not isinstance(v, dict)]
        tables = [(k, v) for k, v in node.items() if isinstance(v, dict)]
        if prefix and (values or not tables):
            if lines:
                lines.append("")
            lines.append("[" + ".".join(_toml_key(p) for p in prefix) + "]")
        for key, value in values:
            lines.append(f"{_toml_key(key)} = {toml_value(value)}")
        for key, value in tables:
            emit(value, prefix + (key,))

    emit(table, ())
    return "\n".join(lines) + "\n"


def write_new_text(path, text: str) -> None:
    with open(path, "x") as stream:
        stream.write(text)


# --------------------------------------------------------- dict utilities

def deep_merge(base: dict, extra: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def leaves(tree: dict, prefix: tuple = ()):
    for key, value in tree.items():
        if isinstance(value, dict):
            yield from leaves(value, prefix + (key,))
        else:
            yield prefix + (key,), value


def get_path(tree: dict, path: tuple):
    node = tree
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node


def set_path(tree: dict, path: tuple, value) -> None:
    node = tree
    for key in path[:-1]:
        child = node.get(key)
        if child is None:
            child = node[key] = {}
        elif not isinstance(child, dict):
            raise StudyError(f"'{'.'.join(path)}': '{key}' is not a table")
        node = child
    node[path[-1]] = value


def is_number(value) -> bool:
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def apply_scale(config: dict, scale: dict, source: str) -> None:
    """Multiplies numbers (floats) or arrays of them in `config` in place."""
    for path, factor in leaves(scale):
        name = ".".join(path)
        value = get_path(config, path)
        if value is None:
            raise StudyError(f"{source}: cannot scale '{name}': not in the "
                             f"configuration")
        if isinstance(value, float):
            set_path(config, path, value * factor)
        elif (isinstance(value, list) and value
              and all(isinstance(v, float) for v in value)):
            set_path(config, path, [v * factor for v in value])
        else:
            raise StudyError(f"{source}: cannot scale '{name}': not a float "
                             f"or an array of floats")


def derive_config(base_path: str, set_tree: dict, scale_tree: dict,
                  path_keys: tuple, source: str) -> dict:
    """The base configuration with its input paths made absolute, then the
    set values merged in, then the factors applied."""
    try:
        with open(base_path, "rb") as stream:
            config = tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise StudyError(f"{base_path}: cannot read: {error}") from error
    base_dir = Path(base_path).resolve().parent
    for path in path_keys:
        value = get_path(config, path)
        if isinstance(value, str):
            set_path(config, path, str((base_dir / value).resolve()))
    config = deep_merge(config, set_tree)
    apply_scale(config, scale_tree, source)
    return config


def derive_follow(case: dict, reference_path: str, extra_set=None,
                  extra_scale=None) -> dict:
    """A case's follow configuration on `reference_path`."""
    set_tree = deep_merge(case["follow_set"], extra_set or {})
    scale_tree = deep_merge(case["follow_scale"], extra_scale or {})
    config = derive_config(case["follow"], set_tree, scale_tree,
                           FOLLOW_PATH_KEYS, case["name"])
    if get_path(config, ("learning", "offset")) is not None:
        raise StudyError(f"{case['name']}: the follow configuration names a "
                         f"[learning] offset; the study passes --offset")
    config["reference"] = str(reference_path)
    return config


def derive_learn(case: dict) -> dict:
    return derive_config(case["learn"], case["learn_set"], {}, (),
                         case["name"])


def config_header(case: dict, kind: str, extra: dict | None = None) -> str:
    lines = [f"{kind} configuration of study case {case['name']}, written by",
             "follow_learn_study.py.",
             f"base: {case[kind]}"]
    trees = {"follow": ("follow_set", "follow_scale"),
             "learn": ("learn_set",)}[kind]
    for tree in trees:
        for path, value in leaves(case[tree]):
            lines.append(f"{tree}: {'.'.join(path)} = {value!r}")
    for tree, values in (extra or {}).items():
        for path, value in leaves(values):
            lines.append(f"{tree} (this trial): {'.'.join(path)} = "
                         f"{value!r}")
    return "\n".join(lines)


# ------------------------------------------------------------------- plan

def _require_table(value, where: str) -> dict:
    if not isinstance(value, dict):
        raise StudyError(f"{where} must be a table")
    return value


def _reject_unknown(table: dict, allowed, where: str) -> None:
    unknown = [key for key in table if key not in allowed]
    if unknown:
        raise StudyError(f"{where}: unknown key(s) {', '.join(unknown)}")


def _plan_path(plan_dir: Path, value, where: str) -> str:
    if not isinstance(value, str) or not value:
        raise StudyError(f"{where} must be a non-empty path")
    return str((plan_dir / value).resolve())


def _check_name(name, where: str) -> str:
    if not isinstance(name, str) or not NAME_PATTERN.fullmatch(name):
        raise StudyError(f"{where}: name must match "
                         f"{NAME_PATTERN.pattern}, got {name!r}")
    return name


def _follow_tree(value, plan_dir: Path, where: str, scale: bool) -> dict:
    tree = copy.deepcopy(_require_table(value, where))
    for path, leaf in list(leaves(tree)):
        name = ".".join(path)
        if path in STUDY_KEYS:
            raise StudyError(f"{where}: '{name}' is set by the study")
        if scale:
            if not is_number(leaf) or leaf <= 0.0:
                raise StudyError(f"{where}: factor '{name}' must be a "
                                 f"finite number > 0")
        elif path in FOLLOW_PATH_KEYS:
            set_path(tree, path, _plan_path(plan_dir, leaf, f"{where} "
                                                            f"'{name}'"))
    return tree


def _overlap(set_tree: dict, scale_tree: dict, where: str) -> None:
    set_paths = {path for path, _ in leaves(set_tree)}
    for path, _ in leaves(scale_tree):
        if path in set_paths:
            raise StudyError(f"{where}: '{'.'.join(path)}' is both set and "
                             f"scaled")


def _apply_overrides(case: dict, raw: dict, plan_dir: Path, where: str):
    for key, value in raw.items():
        if key in ("name", "note", "like", "criteria", "vary"):
            continue
        if key in ("follow", "learn"):
            case[key] = _plan_path(plan_dir, value, f"{where} {key}")
        elif key in ("follow_set", "follow_scale"):
            case[key] = deep_merge(case[key], _follow_tree(
                value, plan_dir, f"{where} {key}", key == "follow_scale"))
        elif key == "learn_set":
            case[key] = deep_merge(case[key],
                                   _require_table(value, f"{where} {key}"))
        elif key == "perturb":
            if not isinstance(value, list):
                raise StudyError(f"{where} perturb must be an array of "
                                 f"tables")
            perturbs = []
            for i, item in enumerate(value):
                at = f"{where} perturb[{i}]"
                item = _require_table(item, at)
                _reject_unknown(item, PERTURB_KEYS, at)
                trials = item.get("trials")
                if (not isinstance(trials, list) or not trials
                        or not all(_is_index(t) for t in trials)):
                    raise StudyError(f"{at}: trials must be a non-empty "
                                     f"array of trial indices")
                perturbs.append({
                    "trials": list(trials),
                    "follow_set": _follow_tree(item.get("follow_set", {}),
                                               plan_dir, at, False),
                    "follow_scale": _follow_tree(
                        item.get("follow_scale", {}), plan_dir, at, True)})
            case[key] = perturbs
        else:
            case[key] = copy.deepcopy(value)


def _is_index(value) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) \
        and value >= 0


def _criteria(value, sets: dict, where: str) -> list:
    if isinstance(value, str):
        if value not in sets:
            raise StudyError(f"{where}: no criteria set '{value}'")
        value = sets[value]
    if not isinstance(value, list):
        raise StudyError(f"{where}: criteria must be a set name or an array")
    out, ids = [], set()
    for i, item in enumerate(value):
        at = f"{where} criterion {i}"
        item = copy.deepcopy(_require_table(item, at))
        check = item.get("check")
        if check not in CHECK_PARAMETERS:
            raise StudyError(f"{at}: unknown check {check!r}")
        ident = item.get("id")
        if not isinstance(ident, str) or not ident or ident in ids:
            raise StudyError(f"{at}: id must be a unique non-empty string")
        ids.add(ident)
        required, optional = CHECK_PARAMETERS[check]
        _reject_unknown(item, ("id", "check", *required, *optional), at)
        missing = [key for key in required if key not in item]
        if missing:
            raise StudyError(f"{at}: {check} needs {', '.join(missing)}")
        for key in ("trial", "of", "from", "to"):
            if key in item and not _is_index(item[key]):
                raise StudyError(f"{at}: {key} must be a trial index")
        if "trials" in item and (
                not isinstance(item["trials"], list) or not item["trials"]
                or not all(_is_index(t) for t in item["trials"])):
            raise StudyError(f"{at}: trials must be an array of indices")
        if "at_most" in item and (not is_number(item["at_most"])
                                  or item["at_most"] <= 0.0):
            raise StudyError(f"{at}: at_most must be a number > 0")
        if "case" in item and not isinstance(item["case"], str):
            raise StudyError(f"{at}: case must be a case name")
        out.append(item)
    return out


def _validate_case(case: dict, references: dict, where: str) -> None:
    if case.get("reference") not in references:
        raise StudyError(f"{where}: reference {case.get('reference')!r} is "
                         f"not in [references]")
    for key in ("follow", "learn"):
        if not isinstance(case.get(key), str):
            raise StudyError(f"{where}: no {key} configuration")
        if not os.path.isfile(case[key]):
            raise StudyError(f"{where}: {case[key]}: no such file")
    if case.get("method") not in METHODS:
        raise StudyError(f"{where}: method must be one of "
                         f"{', '.join(METHODS)}")
    trials = case.get("trials")
    if isinstance(trials, bool) or not isinstance(trials, int) or trials < 1:
        raise StudyError(f"{where}: trials must be an integer >= 1")
    for perturb in case["perturb"]:
        for k in perturb["trials"]:
            if k >= trials:
                raise StudyError(f"{where}: perturbed trial {k} is not run "
                                 f"(trials 0..{trials - 1})")
    _overlap(case["follow_set"], case["follow_scale"], where)
    target = case.get("transfer_to")
    if target is not None:
        if target not in references:
            raise StudyError(f"{where}: transfer_to {target!r} is not in "
                             f"[references]")
        if case["method"] != "fel" or trials < 2:
            raise StudyError(f"{where}: transfer_to needs method fel and at "
                             f"least one update")
    for criterion in case["criteria"]:
        at = f"{where} criterion {criterion['id']}"
        for key in ("trial", "of", "from", "to"):
            if key in criterion and criterion[key] >= trials:
                raise StudyError(f"{at}: trial {criterion[key]} is not run")
        for k in criterion.get("trials", []):
            if k >= trials:
                raise StudyError(f"{at}: trial {k} is not run")
        if criterion["check"] == "transfer" and target is None:
            raise StudyError(f"{at}: transfer needs transfer_to")


def _value_label(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(value)
    return str(value)


def dir_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.=+-]", "_", name).rstrip("_")


def load_plan(path) -> dict:
    """Parses and resolves a plan: every case fully specified, paths
    absolute, sweeps expanded."""
    path = Path(path)
    try:
        raw_bytes = path.read_bytes()
        raw = tomllib.loads(raw_bytes.decode("utf-8"))
    except (OSError, UnicodeDecodeError, tomllib.TOMLDecodeError) as error:
        raise StudyError(f"{path}: cannot read the plan: {error}") from error
    where = str(path)
    _reject_unknown(raw, ("format", "version", "split_hz", "references",
                          "defaults", "criteria", "case", "sweep"), where)
    if raw.get("format") != PLAN_FORMAT or raw.get("version") != PLAN_VERSION:
        raise StudyError(f"{where}: expected format = \"{PLAN_FORMAT}\" and "
                         f"version = {PLAN_VERSION}")
    split_hz = raw.get("split_hz", DEFAULT_SPLIT_HZ)
    if not is_number(split_hz) or split_hz <= 0.0:
        raise StudyError(f"{where}: split_hz must be a number > 0")
    plan_dir = path.resolve().parent

    references = {}
    for name, spec in _require_table(raw.get("references", {}),
                                     f"{where} [references]").items():
        at = f"{where} [references.{name}]"
        _check_name(name, at)
        spec = _require_table(spec, at)
        _reject_unknown(spec, (*REFERENCE_SOURCES, "cut_leading_s",
                               "cut_leading_frames"), at)
        sources = [key for key in REFERENCE_SOURCES if key in spec]
        if len(sources) != 1:
            raise StudyError(f"{at}: give exactly one of "
                             f"{', '.join(REFERENCE_SOURCES)}")
        kind = sources[0]
        entry = {"kind": kind}
        if kind == "from":
            if spec["from"] not in references:
                raise StudyError(f"{at}: from {spec['from']!r} must name an "
                                 f"earlier reference")
            cuts = [key for key in ("cut_leading_s", "cut_leading_frames")
                    if key in spec]
            if len(cuts) != 1:
                raise StudyError(f"{at}: give exactly one of cut_leading_s "
                                 f"and cut_leading_frames")
            value = spec[cuts[0]]
            if cuts[0] == "cut_leading_s" and (not is_number(value)
                                               or value <= 0.0):
                raise StudyError(f"{at}: cut_leading_s must be > 0")
            if cuts[0] == "cut_leading_frames" and (not _is_index(value)
                                                    or value == 0):
                raise StudyError(f"{at}: cut_leading_frames must be >= 1")
            entry.update({"from": spec["from"], cuts[0]: value})
        else:
            if "cut_leading_s" in spec or "cut_leading_frames" in spec:
                raise StudyError(f"{at}: a cut needs from")
            entry[kind] = _plan_path(plan_dir, spec[kind], f"{at} {kind}")
            if not os.path.isfile(entry[kind]):
                raise StudyError(f"{at}: {entry[kind]}: no such file")
        references[name] = entry

    sets = {}
    for name, value in _require_table(raw.get("criteria", {}),
                                      f"{where} [criteria]").items():
        sets[name] = _criteria(value, {}, f"{where} [criteria.{name}]")

    defaults = {"follow_set": {}, "follow_scale": {}, "learn_set": {},
                "perturb": []}
    raw_defaults = _require_table(raw.get("defaults", {}),
                                  f"{where} [defaults]")
    _reject_unknown(raw_defaults, DEFAULT_KEYS, f"{where} [defaults]")
    _apply_overrides(defaults, raw_defaults, plan_dir, f"{where} [defaults]")

    cases, by_name = [], {}

    def add(case: dict, at: str) -> None:
        if case["name"] in by_name:
            raise StudyError(f"{at}: case name {case['name']!r} is not "
                             f"unique")
        case["dir"] = dir_name(case["name"])
        if any(other["dir"] == case["dir"] for other in cases):
            raise StudyError(f"{at}: directory {case['dir']!r} is not "
                             f"unique")
        _validate_case(case, references, at)
        cases.append(case)
        by_name[case["name"]] = case

    def start(raw_case: dict, at: str) -> dict:
        like = raw_case.get("like")
        if like is None:
            base = copy.deepcopy(defaults)
        elif like in by_name:
            base = copy.deepcopy(by_name[like])
            for key in ("criteria", "note", "kind", "sweep", "vary", "dir",
                        "like"):
                base.pop(key, None)
        else:
            raise StudyError(f"{at}: like {like!r} must name an earlier "
                             f"case")
        base["like"] = like
        _apply_overrides(base, raw_case, plan_dir, at)
        return base

    raw_cases = raw.get("case", [])
    if not isinstance(raw_cases, list):
        raise StudyError(f"{where}: case must be an array of tables")
    for i, raw_case in enumerate(raw_cases):
        at = f"{where} case[{i}]"
        raw_case = _require_table(raw_case, at)
        _reject_unknown(raw_case, CASE_KEYS, at)
        name = _check_name(raw_case.get("name"), at)
        at = f"{where} case {name}"
        case = start(raw_case, at)
        case.update({"name": name, "kind": "case",
                     "note": raw_case.get("note", ""),
                     "criteria": _criteria(raw_case.get("criteria", []),
                                           sets, at)})
        case.setdefault("transfer_to", None)
        add(case, at)

    raw_sweeps = raw.get("sweep", [])
    if not isinstance(raw_sweeps, list):
        raise StudyError(f"{where}: sweep must be an array of tables")
    for i, raw_sweep in enumerate(raw_sweeps):
        at = f"{where} sweep[{i}]"
        raw_sweep = _require_table(raw_sweep, at)
        _reject_unknown(raw_sweep, SWEEP_KEYS, at)
        sweep = _check_name(raw_sweep.get("name"), at)
        at = f"{where} sweep {sweep}"
        vary = _require_table(raw_sweep.get("vary"), f"{at} vary")
        if not vary:
            raise StudyError(f"{at}: vary is empty")
        axes = []
        for key, values in vary.items():
            prefix, _, rest = key.partition(".")
            if prefix not in ("learn", "follow") or not rest:
                raise StudyError(f"{at}: vary key {key!r} must start with "
                                 f"'learn.' or 'follow.'")
            if not isinstance(values, list) or not values:
                raise StudyError(f"{at}: vary {key!r} needs an array of "
                                 f"values")
            axes.append((prefix, tuple(rest.split(".")), values))
        for combination in itertools.product(*(axis[2] for axis in axes)):
            raw_case = {k: v for k, v in raw_sweep.items() if k != "vary"}
            labels, chosen = [], {}
            for (prefix, key_path, _), value in zip(axes, combination):
                tree = f"{prefix}_set"
                merged = copy.deepcopy(raw_case.get(tree, {}))
                set_path(merged, key_path, value)
                raw_case[tree] = merged
                labels.append(f"{key_path[-1]}={_value_label(value)}")
                chosen[f"{prefix}.{'.'.join(key_path)}"] = value
            name = (f"{sweep}={_value_label(combination[0])}"
                    if len(axes) == 1 else f"{sweep}[{','.join(labels)}]")
            case = start(raw_case, f"{at} {name}")
            case.update({"name": name, "kind": "sweep", "sweep": sweep,
                         "vary": chosen, "note": raw_sweep.get("note", ""),
                         "criteria": [], "transfer_to": None})
            add(case, f"{at} {name}")

    for case in cases:
        for criterion in case["criteria"]:
            other = criterion.get("case")
            if other is not None:
                if other not in by_name:
                    raise StudyError(f"{where} case {case['name']} criterion "
                                     f"{criterion['id']}: no case {other!r}")
                for k in criterion["trials"]:
                    if k >= by_name[other]["trials"]:
                        raise StudyError(
                            f"{where} case {case['name']} criterion "
                            f"{criterion['id']}: case {other} does not run "
                            f"trial {k}")
    if not cases:
        raise StudyError(f"{where}: no cases")
    return {"path": str(path.resolve()), "sha256":
            hashlib.sha256(raw_bytes).hexdigest(), "split_hz": float(split_hz),
            "references": references, "cases": cases}


# ---------------------------------------------------------------- running

class Tools:
    """Runs the allowed binaries, keeping each step's output in files."""

    def __init__(self, bin_dir, work_dir):
        self.bin_dir = Path(bin_dir)
        self.work_dir = Path(work_dir)

    def run(self, step: str, binary: str, args: list) -> dict:
        if binary not in BINARIES:
            raise StudyError(f"refusing to run {binary!r}: the study runs "
                             f"only {', '.join(BINARIES)}")
        argv = [str(self.bin_dir / binary)] + [str(arg) for arg in args]
        stem = self.work_dir / step
        started = time.monotonic()
        record = {"step": step, "argv": argv}
        try:
            done = subprocess.run(argv, stdin=subprocess.DEVNULL,
                                  capture_output=True, text=True,
                                  timeout=COMMAND_TIMEOUT_S)
            record.update(exit=done.returncode, stdout=done.stdout,
                          stderr=done.stderr)
        except subprocess.TimeoutExpired as expired:
            record.update(exit=None, timeout_s=COMMAND_TIMEOUT_S,
                          stdout=_text(expired.stdout),
                          stderr=_text(expired.stderr))
        except OSError as error:
            record.update(exit=None, stdout="", stderr=f"cannot run: {error}")
        record["wall_s"] = time.monotonic() - started
        for suffix, key in ((".out", "stdout"), (".err", "stderr")):
            with open(f"{stem}{suffix}", "w") as stream:
                stream.write(record[key])
        return record


def _text(value) -> str:
    if value is None:
        return ""
    return value.decode(errors="replace") if isinstance(value, bytes) \
        else value


def require_ok(record: dict, reason: str) -> dict:
    if record["exit"] != 0:
        how = ("timed out" if record.get("timeout_s") is not None
               else f"exit {record['exit']}")
        raise CaseStop({**record, "reason": f"{reason}: {how}"})
    return record


def analyse_log(path, split_hz: float) -> dict:
    """The tracking-phase metrics of a follow log (follow_learnability)."""
    run = fl.load_tracking(path, fl.ANATOMY_COLUMNS)
    anatomy = fl.anatomy(run, split_hz)
    joints = anatomy["joints"]
    error = np.column_stack([run.joint("error", j) for j in range(DOF)])
    elapsed = run.time - run.time[0]
    start = elapsed < EDGE_S
    end = elapsed > run.duration - EDGE_S
    middle = ~(start | end)

    def window(mask):
        return (float(np.sqrt(np.mean(np.square(error[mask]))))
                if np.any(mask) else None)

    above = [joint["e_rms_above_split_rad"] for joint in joints]
    return {
        "path": str(path),
        "samples": run.samples,
        "duration_s": run.duration,
        "rms_rad": anatomy["aggregate"]["all"]["e_rms_rad"],
        "arm_rms_rad": anatomy["aggregate"]["arm"]["e_rms_rad"],
        "rms_rad_by_joint": [joint["e_rms_rad"] for joint in joints],
        "max_abs_rad_by_joint": [joint["e_max_abs_rad"] for joint in joints],
        "mean_rad_by_joint": [joint["e_mean_rad"] for joint in joints],
        "hf_rms_rad": fl.aggregate_rms(above),
        "hf_arm_rms_rad": fl.aggregate_rms(above[:ARM]),
        "hf_rms_rad_by_joint": above,
        "edge_rms_rad": {"first_s": window(start), "middle": window(middle),
                         "last_s": window(end), "window_s": EDGE_S},
        "dqref_first_abs_rad_s": [joint["dqref_first_abs_rad_s"]
                                  for joint in joints],
        "ddqref_first_abs_rad_s2": [joint["ddqref_first_abs_rad_s2"]
                                    for joint in joints],
    }


def analyse(log, split_hz: float, what: str) -> dict:
    try:
        return analyse_log(log, split_hz)
    except fl.InputError as error:
        raise CaseStop({"step": f"analysis of {what}",
                        "reason": f"{what}: the log cannot be analysed: "
                                  f"{error}"}) from error


def read_report(path) -> dict:
    with open(path, "rb") as stream:
        report = tomllib.load(stream)
    out = {
        "measured_trial": report["measured_trial"],
        "output_trial": report["output_trial"],
        "rms_rad": report["tracking"]["aggregate_rms_rad"],
        "arm_rms_rad": report["tracking"]["arm_rms_rad"],
        "rms_rad_by_joint": report["tracking"]["error_rms_rad"],
        "new_offset_max_abs_rad_by_joint": report["offset"]["max_abs_rad"],
    }
    if "ilc" in report:
        out["step_clamped"] = report["ilc"]["step_clamped"]
        out["abs_clamped"] = report["ilc"]["abs_clamped"]
    if "fel" in report:
        out["fel"] = {key: report["fel"][key] for key in
                      ("kept_columns", "skipped", "lambda_max", "lambda_min",
                       "s_rms_rad", "residual_rms_rad")}
    return out


def simulate(tools: Tools, step: str, config, offset, log, motion,
             expected_trial: int) -> dict:
    record = tools.run(step, "x7_follow_sim",
                       ["--config", config, "--offset", offset, "--log", log,
                        "--motion", motion])
    require_ok(record, f"{step} failed")
    lines = record["stdout"].splitlines()
    accepted = [line for line in lines if ACCEPTED.fullmatch(line)]
    result = [line for line in lines if RESULT.match(line)]
    record["accepted_line"] = accepted[0] if accepted else None
    record["result_line"] = result[0] if result else None
    digest = sha256_file(offset)
    match = ACCEPTED.fullmatch(accepted[0]) if len(accepted) == 1 else None
    if (match is None or int(match[2]) != expected_trial
            or match[4] != digest):
        raise CaseStop({**record, "reason": f"{step}: the follower's "
                        f"acceptance of {Path(offset).name} (trial "
                        f"{expected_trial}, sha256 {digest}) is not "
                        f"confirmed"})
    if len(result) != 1 or RESULT.match(result[0])[1] != "success":
        raise CaseStop({**record, "reason": f"{step}: no success line"})
    return record


def check_lineage(offset: dict, *, method: str, trial: int, reference: str,
                  parent: str, source: str) -> list:
    wrong = []
    for key, got, want in (("method", offset["method"], method),
                           ("trial", offset["trial"], trial),
                           ("reference_sha256", offset["reference_sha256"],
                            reference),
                           ("parent_sha256", offset["parent_sha256"], parent),
                           ("source_log_sha256", offset["source_log_sha256"],
                            source)):
        if got != want:
            wrong.append(f"{key} is {got!r}, expected {want!r}")
    return wrong


def _trial_config(case: dict, k: int):
    """The perturbation of trial k, merged (None when there is none)."""
    set_tree, scale_tree, hit = {}, {}, False
    for perturb in case["perturb"]:
        if k in perturb["trials"]:
            hit = True
            set_tree = deep_merge(set_tree, perturb["follow_set"])
            scale_tree = deep_merge(scale_tree, perturb["follow_scale"])
    return (set_tree, scale_tree) if hit else None


def run_case(job: dict) -> dict:
    """Runs one case in its own new directory; returns its record."""
    case = job["case"]
    work = Path(job["dir"])
    split_hz = job["split_hz"]
    references = job["references"]
    started = time.monotonic()
    result = {"name": case["name"], "dir": str(work), "spec": case,
              "status": "running", "stop": None, "setup": [], "trials": [],
              "transfer": None}
    try:
        work.mkdir()
        tools = Tools(job["bin_dir"], work)
        reference = references[case["reference"]]["path"]
        follow = work / "follow.toml"
        write_new_text(follow, toml_text(derive_follow(case, reference),
                                         config_header(case, "follow")))
        learn = work / "learn.toml"
        write_new_text(learn, toml_text(derive_learn(case),
                                        config_header(case, "learn")))
        configs = {}
        for k in range(case["trials"]):
            perturbation = _trial_config(case, k)
            if perturbation is None:
                configs[k] = follow
                continue
            path = work / f"follow_trial{k}.toml"
            extra = {"follow_set": perturbation[0],
                     "follow_scale": perturbation[1]}
            write_new_text(path, toml_text(
                derive_follow(case, reference, *perturbation),
                config_header(case, "follow", extra)))
            configs[k] = path
        result["follow_config"] = str(follow)
        result["learn_config"] = str(learn)

        offset_path = work / "offset_0.csv"
        result["setup"].append(require_ok(tools.run(
            "init", "x7_follow_learn",
            ["init", "--follow-config", follow, "--output", offset_path]),
            "init failed"))
        signature = parse_offset(offset_path)["reference_sha256"]
        result["reference_signature"] = signature
        fel = case["method"] == "fel"
        weights = work / "weights_0.toml"
        if fel:
            result["setup"].append(require_ok(tools.run(
                "init_weights", "x7_follow_learn",
                ["init-weights", "--config", learn, "--output", weights]),
                "init-weights failed"))

        for k in range(case["trials"]):
            trial = {"trial": k, "follow_config": str(configs[k]),
                     "perturbed": configs[k] != follow}
            result["trials"].append(trial)
            trial["offset"] = parse_offset(offset_path)
            log = work / f"trial_{k}.csv"
            trial["simulation"] = simulate(
                tools, f"sim_{k}", configs[k], offset_path, log,
                work / f"trial_{k}.zvs", k)
            trial["log"] = analyse(log, split_hz, f"trial {k}")
            trial.update(rms_rad=trial["log"]["rms_rad"],
                         arm_rms_rad=trial["log"]["arm_rms_rad"],
                         rms_source="log",
                         hf_rms_rad=trial["log"]["hf_rms_rad"])
            if k == case["trials"] - 1:
                break
            next_offset = work / f"offset_{k + 1}.csv"
            report = work / f"report_{k}.toml"
            args = [case["method"], "--config", learn, "--follow-config",
                    configs[k], "--log", log, "--offset", offset_path]
            next_weights = work / f"weights_{k + 1}.toml"
            if fel:
                args += ["--weights", weights, "--output-weights",
                         next_weights]
            args += ["--output", next_offset, "--report", report]
            update = tools.run(f"update_{k}", "x7_follow_learn", args)
            trial["update"] = update
            require_ok(update, f"update after trial {k} refused")
            update["report"] = read_report(report)
            new = parse_offset(next_offset)
            wrong = check_lineage(
                new, method=case["method"], trial=k + 1,
                reference=signature,
                parent=sha256_file(next_weights if fel else offset_path),
                source=sha256_file(log))
            if update["report"]["measured_trial"] != k:
                wrong.append(f"report measured_trial is "
                             f"{update['report']['measured_trial']}")
            if wrong:
                raise CaseStop({**update, "reason": f"update after trial "
                                f"{k}: lineage: {'; '.join(wrong)}"})
            trial.update(rms_rad=update["report"]["rms_rad"],
                         arm_rms_rad=update["report"]["arm_rms_rad"],
                         rms_source="report")
            offset_path = next_offset
            if fel:
                weights = next_weights

        target = case.get("transfer_to")
        if target is not None:
            result["transfer"] = run_transfer(case, tools, work, references,
                                              target, learn, weights,
                                              split_hz)
        result["status"] = "completed"
    except CaseStop as stop:
        result["status"] = "stopped"
        result["stop"] = stop.record
    except Exception as error:  # a harness failure, kept with the case
        result["status"] = "stopped"
        result["stop"] = {"step": "harness", "reason": f"harness error: "
                          f"{error}", "traceback": traceback.format_exc()}
    result["wall_s"] = time.monotonic() - started
    result = jsonable(result)
    if work.is_dir():
        with open(work / "case.json", "w") as stream:
            json.dump(result, stream, indent=1, allow_nan=False)
    return result


def run_transfer(case, tools, work, references, target, learn, weights,
                 split_hz) -> dict:
    """The trained weights on another reference: one run with the zero
    offset, one with the offset `synth` gives."""
    tdir = work / "transfer"
    tdir.mkdir()
    ttools = Tools(tools.bin_dir, tdir)
    follow = tdir / "follow.toml"
    write_new_text(follow, toml_text(
        derive_follow(case, references[target]["path"]),
        config_header(case, "follow")))
    out = {"reference": target, "follow_config": str(follow),
           "weights": str(weights), "runs": {}}
    zero = tdir / "offset_zero.csv"
    require_ok(ttools.run("init", "x7_follow_learn",
                          ["init", "--follow-config", follow, "--output",
                           zero]), "transfer init failed")
    synth = tdir / "offset_synth.csv"
    for name, offset in (("zero", zero), ("synth", synth)):
        if name == "synth":
            out["synth"] = require_ok(ttools.run(
                "synth", "x7_follow_learn",
                ["synth", "--config", learn, "--weights", weights,
                 "--follow-config", follow, "--output", synth]),
                "transfer synth refused")
        stats = parse_offset(offset)
        log = tdir / f"{name}.csv"
        record = simulate(ttools, f"sim_{name}", follow, offset, log,
                          tdir / f"{name}.zvs", stats["trial"])
        analysis = analyse(log, split_hz, f"transfer {name}")
        out["runs"][name] = {"offset": stats, "simulation": record,
                             "log": analysis,
                             "rms_rad": analysis["rms_rad"],
                             "arm_rms_rad": analysis["arm_rms_rad"]}
        out[f"{name}_rms_rad"] = analysis["rms_rad"]
    return out


def run_case_process(job: dict, job_path: Path) -> dict:
    """Runs one case in a child process of this script (--jobs > 1) and
    returns the record it wrote."""
    with open(job_path, "x") as stream:
        json.dump(job, stream, indent=1)
    done = subprocess.run([sys.executable, os.path.abspath(__file__),
                           "--run-case", str(job_path)],
                          stdin=subprocess.DEVNULL, capture_output=True,
                          text=True)
    record = Path(job["dir"]) / "case.json"
    if done.returncode == 0 and record.is_file():
        with open(record) as stream:
            return json.load(stream)
    return jsonable({"name": job["case"]["name"], "dir": job["dir"],
                     "spec": job["case"], "status": "stopped",
                     "stop": {"step": "case process",
                              "reason": f"the case process failed: exit "
                                        f"{done.returncode}",
                              "stdout": done.stdout, "stderr": done.stderr},
                     "setup": [], "trials": [], "transfer": None,
                     "wall_s": 0.0})


def jsonable(value):
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, (np.floating, float)):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, Path):
        return str(value)
    return value


# ------------------------------------------------------------- evaluation

def metrics(result: dict) -> dict:
    """What the criteria read from a case record."""
    trials = result["trials"]
    updates = []
    for trial in trials:
        update = trial.get("update")
        if update is not None and "report" in update:
            report = update["report"]
            updates.append({
                "trial": trial["trial"],
                "step_clamped": int(sum(report.get("step_clamped", []))),
                "abs_clamped": int(sum(report.get("abs_clamped", [])))})
    rms = [trial.get("rms_rad") for trial in trials]
    hf = [trial.get("hf_rms_rad") for trial in trials]
    return {"planned_trials": result["spec"]["trials"],
            "rms": rms, "hf": hf, "updates": updates,
            "completed": result["status"] == "completed",
            "stop": (result["stop"] or {}).get("reason"),
            "transfer": result.get("transfer")}


def _value(series: list, k: int, what: str):
    if k >= len(series) or series[k] is None:
        raise LookupError(f"{what} of trial {k} not measured")
    return series[k]


def mrad(value) -> str:
    return "-" if value is None else sig3(value * 1e3)


def sig3(value: float) -> str:
    """Three significant digits, trailing zeros kept."""
    if value == 0.0:
        return "0.00"
    rounded = float(f"{value:.3g}")
    exponent = math.floor(math.log10(abs(rounded)))
    if -3 <= exponent < 4:
        return f"{rounded:.{max(0, 2 - exponent)}f}"
    return f"{rounded:.2e}"


def evaluate(criterion: dict, own: dict, others: dict) -> dict:
    """One criterion's verdict, PASS or FAIL, with the numbers behind it.
    A value the runs did not produce fails the criterion."""
    check = criterion["check"]
    rms, hf = own["rms"], own["hf"]
    verdict, detail = False, ""
    try:
        if check == "runs_ok":
            ran = sum(1 for value in rms if value is not None)
            verdict = own["completed"]
            detail = (f"{ran} of {own['planned_trials']} trials, "
                      f"{len(own['updates'])} updates" if verdict
                      else f"stopped: {own['stop']}")
        elif check == "decreasing":
            first = criterion.get("from", 0)
            last = criterion.get("to", own["planned_trials"] - 1)
            values = [_value(rms, k, "RMS") for k in range(first, last + 1)]
            rises = [first + i for i in range(1, len(values))
                     if not values[i] < values[i - 1]]
            verdict = not rises
            detail = (f"trials {first}..{last} strictly decreasing" if verdict
                      else "not below the previous trial at trial(s) "
                      + ", ".join(f"{k} ({mrad(rms[k])} after "
                                  f"{mrad(rms[k - 1])} mrad)"
                                  for k in rises))
        elif check in ("ratio", "hf_ratio"):
            series = rms if check == "ratio" else hf
            what = "RMS" if check == "ratio" else "above-split RMS"
            k, of, bound = (criterion["trial"], criterion["of"],
                            criterion["at_most"])
            ratio = _value(series, k, what) / _value(series, of, what)
            verdict = ratio <= bound
            detail = (f"{what} {k}/{of} = {mrad(series[k])}/"
                      f"{mrad(series[of])} mrad = {ratio:.4f} "
                      f"(at most {bound:g})")
        elif check == "bounded":
            of, bound = criterion["of"], criterion["at_most"]
            base = _value(rms, of, "RMS")
            over = [k for k in criterion["trials"]
                    if not _value(rms, k, "RMS") <= bound * base]
            verdict = not over
            worst = max(criterion["trials"], key=lambda k: rms[k])
            detail = (f"max over trials {criterion['trials']}: trial {worst} "
                      f"{mrad(rms[worst])} mrad = {rms[worst] / base:.4f} x "
                      f"trial {of} ({mrad(base)} mrad; at most {bound:g})")
            if over:
                detail += f"; above at trial(s) {over}"
        elif check == "no_clamp":
            planned = own["planned_trials"] - 1
            if len(own["updates"]) < planned:
                raise LookupError(f"{len(own['updates'])} of {planned} "
                                  f"updates ran")
            step = sum(update["step_clamped"] for update in own["updates"])
            value = sum(update["abs_clamped"] for update in own["updates"])
            verdict = step == 0 and value == 0
            detail = (f"step clamp {step}, value clamp {value} samples over "
                      f"{planned} updates")
        elif check == "vs_case":
            other = others.get(criterion["case"])
            if other is None:
                raise LookupError(f"case {criterion['case']} not run")
            bound = criterion["at_most"]
            parts, verdict = [], True
            for k in criterion["trials"]:
                ratio = (_value(rms, k, "RMS")
                         / _value(other["rms"], k, f"{criterion['case']} "
                                                   f"RMS"))
                verdict = verdict and ratio <= bound
                parts.append(f"{k}: {mrad(rms[k])}/{mrad(other['rms'][k])} "
                             f"= {ratio:.3f}")
            detail = (f"against {criterion['case']} (at most {bound:g}): "
                      + ", ".join(parts))
        elif check == "transfer":
            transfer = own["transfer"]
            if not transfer or transfer.get("synth_rms_rad") is None:
                raise LookupError("the transfer runs did not complete")
            ratio = transfer["synth_rms_rad"] / transfer["zero_rms_rad"]
            verdict = ratio <= criterion["at_most"]
            detail = (f"synthesised {mrad(transfer['synth_rms_rad'])} / zero "
                      f"{mrad(transfer['zero_rms_rad'])} mrad = {ratio:.4f} "
                      f"(at most {criterion['at_most']:g})")
    except LookupError as missing:
        verdict = False
        detail = f"not evaluable: {missing.args[0]}"
        if own["stop"]:
            detail += f" (case stopped: {own['stop']})"
    return {**criterion, "verdict": "PASS" if verdict else "FAIL",
            "detail": detail}


def observations(result: dict) -> dict:
    """Facts recorded for every case, criteria or not."""
    own = metrics(result)
    rms = [value for value in own["rms"] if value is not None]
    hf = [value for value in own["hf"] if value is not None]
    offsets = [trial["offset"] for trial in result["trials"]
               if "offset" in trial]
    out = {
        "trials_run": len(rms),
        "completed": own["completed"],
        "stop": own["stop"],
        "rms_mrad": [value * 1e3 for value in rms],
        "arm_rms_mrad": [trial["arm_rms_rad"] * 1e3
                         for trial in result["trials"]
                         if trial.get("arm_rms_rad") is not None],
        "hf_rms_mrad": [value * 1e3 for value in hf],
        "monotone": (all(rms[k] < rms[k - 1] for k in range(1, len(rms)))
                     if len(rms) > 1 else None),
        "trial_of_min": int(np.argmin(rms)) if rms else None,
        "min_rms_mrad": min(rms) * 1e3 if rms else None,
        "last_hf_rms_mrad": hf[-1] * 1e3 if hf else None,
        "above_trial0": [k for k in range(1, len(rms)) if rms[k] > rms[0]],
        "step_clamped": sum(u["step_clamped"] for u in own["updates"]),
        "abs_clamped": sum(u["abs_clamped"] for u in own["updates"]),
        "max_offset_abs_rad": max((o["max_abs_rad"] for o in offsets),
                                  default=None),
        "max_offset_rate_rad_s": max((o["max_rate_rad_s"] for o in offsets),
                                     default=None),
    }
    transfer = result.get("transfer")
    if transfer:
        out["transfer"] = {key: transfer.get(key) for key in
                           ("reference", "zero_rms_rad", "synth_rms_rad")}
    return out


def table_line(result: dict, width: int) -> str:
    verdicts = " ".join(f"{c['id']}:{c['verdict']}"
                        for c in result["criteria"]) or "(record)"
    obs = result["observations"]
    curve = " ".join(sig3(v) for v in obs["rms_mrad"])
    line = f"{result['name']:<{width}}  {verdicts}  | rms {curve}"
    if not result["criteria"] and obs["trials_run"]:
        monotone = {True: "yes", False: "no", None: "-"}[obs["monotone"]]
        line += (f" | monotone {monotone}, min at trial "
                 f"{obs['trial_of_min']}, last above-split "
                 f"{sig3(obs['last_hf_rms_mrad'])}, clamps "
                 f"{obs['step_clamped']}/{obs['abs_clamped']}")
    if obs.get("transfer"):
        t = obs["transfer"]
        line += (f" | {t['reference']}: zero {mrad(t['zero_rms_rad'])} "
                 f"synth {mrad(t['synth_rms_rad'])}")
    if not obs["completed"]:
        line += f" | STOPPED: {obs['stop']}"
    return line


def format_table(results: list, wall_s: float) -> str:
    width = max(len(result["name"]) for result in results)
    lines = [f"study results (RMS over all 8 joints by trial, mrad); "
             f"wall time {wall_s:.1f} s"]
    lines += [table_line(result, width) for result in results]
    fails = [(result, c) for result in results for c in result["criteria"]
             if c["verdict"] == "FAIL"]
    stopped = [result for result in results
               if not result["observations"]["completed"]]
    lines.append(f"{sum(len(r['criteria']) for r in results)} criteria, "
                 f"{len(fails)} FAIL; {len(stopped)} case(s) stopped")
    for result, criterion in fails:
        lines.append(f"FAIL {result['name']} {criterion['id']} "
                     f"({criterion['check']}): {criterion['detail']}")
    return "\n".join(lines)


# ------------------------------------------------------------------ plan run

def prepare_references(plan: dict, out: Path, bin_dir) -> dict:
    ref_dir = out / "references"
    ref_dir.mkdir()
    tools = Tools(bin_dir, ref_dir)
    prepared = {}
    for name, spec in plan["references"].items():
        target = ref_dir / f"{name}.zvs"
        info = {"name": name, **spec, "path": str(target)}
        if spec["kind"] == "plan":
            record = tools.run(f"plan_{name}", "x7_plan_ptp",
                               ["--config", spec["plan"], "--output", target,
                                "--diagnostics",
                                ref_dir / f"{name}_diagnostics.csv"])
            info["planner"] = record
            if record["exit"] != 0:
                raise StudyError(f"reference {name}: x7_plan_ptp exit "
                                 f"{record['exit']}: {record['stderr']}")
        elif spec["kind"] == "zvs":
            shutil.copyfile(spec["zvs"], target)
        else:
            source = Path(prepared[spec["from"]]["path"])
            text = source.read_text()
            frames = zvs_frames(text, str(source))
            count = (spec["cut_leading_frames"]
                     if "cut_leading_frames" in spec
                     else frames_before(frames, spec["cut_leading_s"]))
            cut, removed = cut_leading_frames(text, count, str(source))
            write_new_text(target, cut)
            info.update(frames_cut=count, time_cut_s=removed)
        frames = zvs_frames(Path(target).read_text(), str(target))
        info.update(sha256=sha256_file(target), frames=len(frames),
                    duration_s=sum(dt for dt, _, _ in frames),
                    first_frame=frames[0][1], last_frame=frames[-1][1])
        prepared[name] = info
    return prepared


def run_plan(plan_path, out_dir, bin_dir, jobs: int = 1,
             progress=None) -> dict:
    progress = sys.stderr if progress is None else progress
    started_at = datetime.datetime.now().astimezone()
    started = time.monotonic()
    plan = load_plan(plan_path)
    out = Path(os.path.abspath(out_dir))
    bin_dir = os.path.abspath(bin_dir)
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        out.mkdir()
    except FileExistsError as error:
        raise StudyError(f"{out}: exists; the study writes a new "
                         f"directory") from error
    shutil.copyfile(plan["path"], out / "plan.toml")
    binaries = {}
    for name in BINARIES:
        path = Path(bin_dir) / name
        binaries[name] = {"path": str(path.resolve()),
                          "sha256": sha256_file(path) if path.is_file()
                          else None}
    summary = {"format": SUMMARY_FORMAT, "version": SUMMARY_VERSION,
               "plan": plan["path"], "plan_sha256": plan["sha256"],
               "split_hz": plan["split_hz"], "out": str(out.resolve()),
               "binaries": binaries, "started": started_at.isoformat(),
               "jobs": jobs, "references": {}, "cases": []}
    try:
        summary["references"] = prepare_references(plan, out, bin_dir)
    except StudyError as error:
        summary["error"] = str(error)
        write_summary(out / "summary.json", summary)
        raise
    job_list = [{"case": case, "dir": str(out / case["dir"]),
                 "bin_dir": str(bin_dir), "split_hz": plan["split_hz"],
                 "references": summary["references"]}
                for case in plan["cases"]]
    results = {}

    def done(result):
        results[result["name"]] = result
        print(f"[{len(results)}/{len(job_list)}] {result['name']}: "
              f"{result['status']}, {len(result['trials'])} trials, "
              f"{result['wall_s']:.1f} s", file=progress, flush=True)

    if jobs <= 1:
        for job in job_list:
            done(run_case(job))
    else:
        # Each case in a child process of this script; the threads only
        # wait for them.
        jobs_dir = out / "jobs"
        jobs_dir.mkdir()
        with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
            futures = [pool.submit(run_case_process, job,
                                   jobs_dir / f"{Path(job['dir']).name}.json")
                       for job in job_list]
            for future in concurrent.futures.as_completed(futures):
                done(future.result())
    ordered = [results[case["name"]] for case in plan["cases"]]
    measured = {result["name"]: metrics(result) for result in ordered}
    for result in ordered:
        result["criteria"] = [evaluate(criterion, measured[result["name"]],
                                       measured)
                              for criterion in result["spec"]["criteria"]]
        result["observations"] = observations(result)
    summary["cases"] = ordered
    summary["finished"] = datetime.datetime.now().astimezone().isoformat()
    summary["wall_s"] = time.monotonic() - started
    write_summary(out / "summary.json", summary)
    return summary


def write_summary(path, summary: dict) -> None:
    with open(path, "x") as stream:
        json.dump(jsonable(summary), stream, indent=1, allow_nan=False)
        stream.write("\n")


def all_passed(summary: dict) -> bool:
    return all(result["status"] == "completed"
               and all(c["verdict"] == "PASS" for c in result["criteria"])
               for result in summary["cases"])


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="follow_learn_study.py", description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("plan", metavar="PLAN.toml")
    parser.add_argument("--out", metavar="DIR",
                        help="new output directory (default build/"
                             "learn_study/PLAN-STEM-YYYYmmdd-HHMMSS)")
    parser.add_argument("--bin-dir", default="build/apps", metavar="DIR",
                        help="where the binaries are (default build/apps)")
    parser.add_argument("--jobs", type=int, default=1, metavar="N",
                        help="run N cases at a time, each in a child "
                             "process of this script (default 1: one after "
                             "the other in this process)")
    return parser


def main(argv: list | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    if argv[:1] == ["--run-case"] and len(argv) == 2:
        with open(argv[1]) as stream:
            run_case(json.load(stream))
        return 0
    args = build_parser().parse_args(argv)
    if args.jobs < 1:
        print("follow_learn_study: error: --jobs must be >= 1",
              file=sys.stderr)
        return 2
    out = args.out
    if out is None:
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        out = Path("build/learn_study") / f"{Path(args.plan).stem}-{stamp}"
    try:
        summary = run_plan(args.plan, out, args.bin_dir, args.jobs)
    except StudyError as error:
        print(f"follow_learn_study: error: {error}", file=sys.stderr)
        return 2
    print(format_table(summary["cases"], summary["wall_s"]))
    print(f"summary: {Path(out) / 'summary.json'}")
    return 0 if all_passed(summary) else 1


if __name__ == "__main__":
    raise SystemExit(main())
