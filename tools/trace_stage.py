#!/usr/bin/env python3
"""Staging of a run's capture streams on a private, size-bounded noswap
tmpfs for tools/trace_session.sh (docs/hardware/control-rate.md, recorder
staging): the budget, the mount's verification and marker, an in-run guard,
and the publication of the staged files to the output directory with their
release.

The session (as root) creates a mount point under the stage root, owned by
the operator with mode 0700, and mounts a tmpfs there with size=S and
noswap; this tool checks that mount, marks it as the session's, watches it
during the run, and once every writer has stopped publishes it:

- inventory: every staged file is hashed once (SHA-256) into an immutable
  inventory, kept in the stage and written durably to the output directory
  (<base>.staging.inventory.json) before any copy;
- publish: each file is copied to a temporary name in the output directory
  (.<name>.partial-<token>, the stage's own token), fsynced, hashed as read
  back and compared with the inventory, then linked to its final name without
  overwriting, and its directory fsynced. A final file that already holds the
  inventory's hash counts as published (a retry after a crash past a link); a
  different one stops publication. Only the stage's own partial names are
  ever removed. The ledger (<base>.staging.sha256, sha256sum format) is
  written last and fsynced with its directory. Nothing syncs the whole
  filesystem: the Dirty and Writeback figures are recorded before and after,
  with a bounded settling check;
- release: only when the ledger matches the inventory and every destination
  file re-hashes to it, the staged files are deleted, the tmpfs unmounted and
  its mount point removed. A stage without a complete marker is released by
  abort, and only while it holds no data.

Past an unmount nothing inside the mount point is read or deleted: the
stage's files and records went with the tmpfs, and the bare mount point is
removed with rmdir (through sudo), which removes only an empty directory. So
a retry after a failure or an interruption between the unmount and the
removal needs neither the vanished marker nor access to the directory, and
completes the same way (release or abort). Only a directory named as the
session names its stages (<session>-<attempt>-<seq>-<8 hex digits>) is ever
unmounted or removed.

The budget (provisional, empirical constants for the measured timer
configuration; a guard, not a reservation): per run,
  S = 1.25 (R + C + A) + Q, R = rho (D + 10 s) raw trace, C = R / 4
assembled, A the other enabled streams (at least 512 MiB in a traced run),
each mode budgeted for the streams it enables, Q = 64 MiB the guard's
reserve; and before mounting MemAvailable must be at least S + K + H, K the
kernel ring buffers' data-plus-reader allocation (below), H = max(8 GiB, 15%
of MemTotal). The guard
stops the run's recorders when the stage's free space falls below C + Q (what
the run still writes after the stop, the assembled trace, and the reserve;
Q alone in an untraced run) or MemAvailable below H / 2. The stage holds its
reserve on top of the budgeted streams, so in every mode it stays above the
bound empty, while recording (the raw trace and the other streams, their
25% margin used too) and at the assembly's end (the assembled trace beside
the raw files). An untraced run has no recorder to stop: a trip there is
recorded and fails the run.

The kernel's ring buffers (Linux 7.0 kernel/trace/ring_buffer.c and
trace.c), from each trace array's own geometry: its page header
(events/header_page) gives the payload p of a subbuffer and the offset of
the data in it, the subbuffer q = offset + payload (a power of two, at least
1 KiB, and buffer_subbuf_size_kb times 1024 where that file exists). For B
requested bytes per CPU the kernel takes n = max(2, ceil(B / p)) subbuffers.
Two quantities follow, kept apart:
- the expected tracefs readback: floor(n p / 1024) KiB per CPU in
  per_cpu/cpuN/buffer_size_kb, and their sum, truncated per CPU before
  summing, in buffer_total_size_kb; the session requires exactly these, for
  every array and every configured CPU, before the probe (buffers);
- the physical budget K: (n + 1) q bytes per CPU and array, the data
  subbuffers plus the reader's, bookkeeping and transient allocations
  excluded; it enters the RAM check only, never the readback check.
The session reads the main array's geometry before any run (geometry) and
checks every array against it once they exist; a missing or malformed
geometry refuses.

usage: trace_stage.py budget --mode M --duration-s D [--rate-hz R] --buffer-kb B --cpus N
                             [--payload P --subbuf Q] [--usbmon 0|1] [--perf-hz F] [--perf-stack S]
                             [--exerciser 0|1]
       trace_stage.py geometry --tracefs DIR [--instance NAME]
       trace_stage.py buffers --tracefs DIR [--instance NAME] --buffer-kb B --cpu-list LIST --payload P --subbuf Q
       trace_stage.py verify --stage DIR --size-mib M
       trace_stage.py mark --stage DIR --session ID --base NAME --outdir DIR
       trace_stage.py guard --stage DIR --min-free B --min-available B [--interval S] [--stop-pid PID ...]
       trace_stage.py usage --stage DIR
       trace_stage.py publish --stage DIR --outdir DIR
       trace_stage.py release --stage DIR --outdir DIR
       trace_stage.py abort --stage DIR
       trace_stage.py leftovers --root DIR
(--meminfo PATH replaces /proc/meminfo; FINDMNT, UMOUNT, RMDIR and
TRACE_STAGE_SUDO replace findmnt, umount, rmdir and sudo, "none" running
without it; TRACE_STAGE_FAIL injects a failure for the tests: copy, hash,
link, crash-after-link, crash-before-ledger or crash-after-unmount; and
TRACE_STAGE_CAPACITY=findmnt models the tests' stand-in stage, a directory on
a large filesystem, as the bounded mount it stands for: findmnt's size, less
its files in 4 KiB pages.)
"""

from __future__ import annotations

import argparse
import datetime
import errno
import hashlib
import json
import math
import os
import pathlib
import re
import secrets
import signal
import subprocess
import sys
import time

MIB = 1 << 20
GIB = 1 << 30
RHO = 250_000_000          # raw trace bytes per second (provisional)
MARGIN = 1.25
COMPRESSION = 4            # the assembled trace's assumed floor (an assumption)
A_TRACED = 512 * MIB       # the other streams of a traced run, at least
OVERHEAD_S = 10            # recording before and after the probe
PROBE_PER_CYCLE = 1024     # the probe's log and summary per cycle
SNAPSHOTS = 16 * MIB       # process listing and task snapshot
USBMON_RATE = 2_000_000    # bytes per second of the adapter's capture
TSC_TEXT_FACTOR = 4        # the TSC instance's text, per byte of one CPU's buffer
SAMPLER = MIB              # sampler pairs and SMI counts, fixed part
SAMPLER_RATE = 64 * 1024   # and per second
TK_FILES = 4 * MIB         # the timekeeping record: the kernel log and the small files
TK_RATE = (64 + 160) * 2000  # and per second, its raw data and text: two records per update at 1 kHz
EXERCISER = 16 * MIB
CONTROLS_TXT = 16 * MIB
PERF_TEXT_PER_SAMPLE = 8 * 1024
GUARD_RESERVE = 64 * MIB    # Q: held on top of the budgeted streams, the guard's bound beyond C
MODES = ("untraced", "traced", "profiled", "timer", "controls")
MARKER = ".rtctrl-stage"
INVENTORY = ".rtctrl-inventory.json"
OWN = (MARKER, MARKER + ".tmp", INVENTORY, INVENTORY + ".tmp")
CHUNK = 4 * MIB
PAGE = 4096
STAGE_NAME = re.compile(r"^\d+-\d{8}T\d{6}-\d+-\d+-[0-9a-f]{8}$")
HEADER_DATA = re.compile(r"field:\s*char\s+data(?:\[\d*\])?;\s*offset:\s*(\d+);\s*size:\s*(\d+);")


class StageError(Exception):
    pass


def meminfo(path: str) -> dict[str, int]:
    """/proc/meminfo's kB fields, in bytes."""
    out = {}
    with open(path) as f:
        for line in f:
            k, _, v = line.partition(":")
            parts = v.split()
            if parts and parts[0].isdigit():
                out[k.strip()] = int(parts[0]) * (1024 if len(parts) > 1 and parts[1] == "kB" else 1)
    return out


def ceil_mib(n: float) -> int:
    return int(math.ceil(n / MIB))


def parse_geometry(header: str, subbuf_kb: str | None) -> dict:
    """A trace array's subbuffer geometry from its page header (and its
    buffer_subbuf_size_kb, when the kernel offers one)."""
    m = HEADER_DATA.findall(header)
    if len(m) != 1:
        raise StageError(f"the page header has {len(m)} data fields, not one")
    offset, payload = int(m[0][0]), int(m[0][1])
    subbuf = offset + payload
    if offset < 1 or payload < 1 or subbuf < 1024 or subbuf & (subbuf - 1):
        raise StageError(f"unsupported subbuffer geometry: data at offset {offset}, {payload} bytes")
    if subbuf_kb is not None:
        v = subbuf_kb.strip()
        if not v.isdigit() or int(v) * 1024 != subbuf:
            raise StageError(f"buffer_subbuf_size_kb reads {v!r}, not the page header's {subbuf // 1024} KiB")
    return {"offset": offset, "payload": payload, "subbuf": subbuf, "subbuf_kb_file": subbuf_kb is not None}


def array_dir(tracefs: pathlib.Path, instance: str | None) -> pathlib.Path:
    return tracefs if instance is None else tracefs / "instances" / instance


def read_geometry(tracefs: pathlib.Path, instance: str | None) -> dict:
    d = array_dir(tracefs, instance)
    try:
        header = (d / "events" / "header_page").read_text()
    except OSError as e:
        raise StageError(f"no page header ({e})") from e
    try:
        subbuf_kb: str | None = (d / "buffer_subbuf_size_kb").read_text()
    except FileNotFoundError:
        subbuf_kb = None
    return parse_geometry(header, subbuf_kb)


def describe_geometry(g: dict) -> str:
    return (f"subbuffer {g['subbuf']} bytes, payload {g['payload']} at offset {g['offset']} (page header"
            f"{', buffer_subbuf_size_kb' if g['subbuf_kb_file'] else '; no buffer_subbuf_size_kb'})")


def capacity(buffer_kb: int, payload: int, subbuf: int) -> dict:
    """The kernel's subbuffers for buffer_kb KiB per CPU: their count, the
    expected readback per CPU (KiB, truncated) and the data-plus-reader bytes."""
    n = max(2, -(-buffer_kb * 1024 // payload))
    return {"pages": n, "readback_kib": (n * payload) >> 10, "physical": (n + 1) * subbuf}


def expand_cpus(cpu_list: str) -> list[int]:
    out: list[int] = []
    for part in cpu_list.strip().split(","):
        a, _, b = part.partition("-")
        if not a.isdigit() or (b and not b.isdigit()):
            raise StageError(f"not a CPU list: {cpu_list!r}")
        out.extend(range(int(a), int(b or a) + 1))
    return out


def read_kib(path: pathlib.Path) -> int:
    """A buffer size file's KiB: a plain number (an unexpanded buffer, "N
    (expanded: M)", or one with unequal CPUs, "X", is no capacity)."""
    t = path.read_text().strip()
    if not t.isdigit():
        raise StageError(f"{path.name} reads {t!r}")
    return int(t)


def check_buffers(tracefs: pathlib.Path, instances: list[str], buffer_kb: int, cpu_list: str, payload: int,
                  subbuf: int, out) -> bool:
    """Every array's geometry as budgeted, and its per-CPU and total
    capacities exactly as the kernel's rounding gives them. An instance
    given as NAME:KB has its own requested size."""
    cpus = expand_cpus(cpu_list)
    ok = True
    physical = 0
    per_array = []
    for spec in [None, *instances]:
        inst, this_kb = spec, buffer_kb
        if spec is not None and ":" in spec:
            inst, kb = spec.split(":", 1)
            this_kb = int(kb)
        cap = capacity(this_kb, payload, subbuf)
        physical += cap["physical"] * len(cpus)
        name = "main" if inst is None else inst
        per_array.append(f"{name} {cap['physical'] // 1024} KiB")
        d = array_dir(tracefs, inst)
        problems = []
        try:
            g = read_geometry(tracefs, inst)
            if (g["payload"], g["subbuf"]) != (payload, subbuf):
                problems.append(f"its geometry ({describe_geometry(g)}) is not the budget's (subbuffer {subbuf} "
                                f"bytes, payload {payload})")
        except StageError as e:
            problems.append(f"its geometry: {e}")
        found = sorted(int(c.name[3:]) for c in (d / "per_cpu").glob("cpu*") if c.name[3:].isdigit())
        if found != cpus:
            problems.append(f"per-CPU buffers for {len(found)} CPUs, not the configured {cpu_list} ({len(cpus)})")
        actual: dict[int, int] = {}
        for c in cpus:
            try:
                actual[c] = read_kib(d / "per_cpu" / f"cpu{c}" / "buffer_size_kb")
            except (OSError, StageError) as e:
                problems.append(f"cpu{c}: {e}")
        wrong = sorted((c, v) for c, v in actual.items() if v != cap["readback_kib"])
        if wrong:
            problems.append(f"{len(wrong)} CPUs not at {cap['readback_kib']} KiB: "
                            + ", ".join(f"cpu{c} {v} KiB" for c, v in wrong[:8]) + (" ..." if len(wrong) > 8 else ""))
        expected_total = cap["readback_kib"] * len(cpus)
        try:
            total = read_kib(d / "buffer_total_size_kb")
            if total != expected_total:
                problems.append(f"buffer_total_size_kb reads {total}, not {expected_total}")
            elif len(actual) == len(cpus) and sum(actual.values()) != total:
                problems.append(f"buffer_total_size_kb reads {total}, not its CPUs' sum {sum(actual.values())}")
        except (OSError, StageError) as e:
            problems.append(f"buffer_total_size_kb: {e}")
        head = (f"kernel buffers, array {name}: requested {this_kb} KiB per CPU on CPUs {cpu_list}; "
                f"{cap['pages']} subbuffers of {subbuf} bytes (payload {payload}); expected {cap['readback_kib']} KiB "
                f"per CPU and {expected_total} KiB in all")
        if problems:
            ok = False
            print(f"{head}: NOT AS EXPECTED: " + "; ".join(problems), file=out)
        else:
            print(f"{head}: read back exactly ({len(cpus)} CPUs at {cap['readback_kib']} KiB, buffer_total_size_kb "
                  f"{expected_total}; geometry as budgeted)", file=out)
    print(f"kernel buffers' physical budget: per CPU {', '.join(per_array)}, each array's data subbuffers plus the "
          f"reader's ({subbuf} bytes each), {physical} bytes for these {1 + len(instances)} arrays; bookkeeping and "
          "transient allocations excluded", file=out)
    return ok


def budget(mode: str, duration_s: float, rate_hz: float, buffer_kb: int, cpus: int, usbmon: bool,
           perf_hz: int, perf_stack: int, exerciser: bool, mem: dict[str, int], payload: int | None = None,
           subbuf: int | None = None, tk_buffer_kb: int | None = None) -> dict:
    """The stage's size and the RAM a run needs, for the streams its mode
    enables (a traced mode needs the trace arrays' geometry)."""
    if mode not in MODES:
        raise StageError(f"unknown mode {mode}")
    t = duration_s + OVERHEAD_S
    traced = mode != "untraced"
    raw = RHO * t if traced else 0
    assembled = raw / COMPRESSION
    items: dict[str, float] = {"snapshots": float(SNAPSHOTS)}
    if mode == "controls":
        items["controls output"] = CONTROLS_TXT
    else:
        items["probe log"] = PROBE_PER_CYCLE * rate_hz * t
    if traced and mode != "controls" and usbmon:
        items["usbmon"] = USBMON_RATE * t
    if mode == "timer":
        items["TSC instance text"] = TSC_TEXT_FACTOR * buffer_kb * 1024
        items["sampler and SMI counts"] = SAMPLER + SAMPLER_RATE * t
        items["timekeeping record"] = TK_FILES + TK_RATE * t
        if exerciser:
            items["exerciser"] = EXERCISER
    if mode == "profiled":
        items["perf.data"] = perf_hz * (perf_stack + 1024) * t
        items["perf text"] = perf_hz * t * PERF_TEXT_PER_SAMPLE
    other = sum(items.values())
    a = max(A_TRACED, other) if traced else other
    size_mib = ceil_mib(MARGIN * (raw + assembled + a) + GUARD_RESERVE)
    arrays = {"untraced": 0, "traced": 1, "profiled": 1, "controls": 1, "timer": 2}[mode]
    cap = {"pages": 0, "readback_kib": 0, "physical": 0}
    if arrays:
        if payload is None or subbuf is None:
            raise StageError(f"a {mode} run's budget needs the trace buffers' geometry (--payload, --subbuf)")
        cap = capacity(buffer_kb, payload, subbuf)
    kernel = cap["physical"] * cpus * arrays
    tk_physical = 0
    if mode == "timer":  # the timekeeping recorder's private array, sized by itself
        if tk_buffer_kb is None or payload is None or subbuf is None:
            raise StageError("a timer run's budget needs the timekeeping recorder's buffer (--tk-buffer-kb)")
        tk_physical = capacity(tk_buffer_kb, payload, subbuf)["physical"] * cpus
        kernel += tk_physical
    total, avail = mem.get("MemTotal"), mem.get("MemAvailable")
    if total is None or avail is None:
        raise StageError("MemTotal or MemAvailable is missing from meminfo")
    headroom = max(8 * GIB, int(0.15 * total))
    need = size_mib * MIB + kernel + headroom
    return {"mode": mode, "duration_s": duration_s, "raw": int(raw), "assembled": int(assembled), "other": int(other),
            "items": {k: int(v) for k, v in items.items()}, "a": int(a), "reserve": GUARD_RESERVE,
            "size_mib": size_mib, "kernel": kernel, "tk_physical": tk_physical, "arrays": arrays, "pages": cap["pages"],
            "readback_kib": cap["readback_kib"], "physical": cap["physical"], "headroom": headroom, "need": need,
            "available": avail, "ok": avail >= need, "guard_min_free": int(assembled) + GUARD_RESERVE,
            "guard_min_available": headroom // 2}


def gb(n: float) -> str:
    return f"{n / 1e9:.2f} GB"


def sudo() -> list[str]:
    s = os.environ.get("TRACE_STAGE_SUDO", "sudo")
    return [] if s == "none" else [s]


def findmnt(stage: pathlib.Path) -> tuple[str, int, list[str]] | None:
    """The mount exactly at stage: its type, size in bytes and options."""
    r = subprocess.run([os.environ.get("FINDMNT", "findmnt"), "-n", "-b", "-o", "FSTYPE,SIZE,OPTIONS",
                        "--mountpoint", str(stage)], capture_output=True, text=True)
    if r.returncode != 0 or not r.stdout.strip():
        return None
    parts = r.stdout.split()
    if len(parts) != 3 or not parts[1].isdigit():
        raise StageError(f"findmnt printed {r.stdout.strip()!r}")
    return parts[0], int(parts[1]), parts[2].split(",")


def verify(stage: pathlib.Path, size_mib: int) -> str:
    m = findmnt(stage)
    if m is None:
        raise StageError(f"{stage} is not a mount point")
    fstype, size, opts = m
    if fstype != "tmpfs":
        raise StageError(f"{stage} is a {fstype} mount, not tmpfs")
    if "noswap" not in opts:
        raise StageError(f"{stage} is a tmpfs without noswap (options {','.join(opts)})")
    if size != size_mib * MIB:
        raise StageError(f"{stage} holds {size} bytes, not the {size_mib} MiB asked for")
    st = stage.stat()
    if st.st_uid != os.getuid() or st.st_mode & 0o777 != 0o700:
        raise StageError(f"{stage} is not private to this user (uid {st.st_uid}, mode {st.st_mode & 0o777:o})")
    return f"tmpfs {size} bytes, options {','.join(opts)}"


def space(stage: pathlib.Path) -> tuple[int, int]:
    """The stage's used and free bytes: its filesystem's statvfs, or with
    TRACE_STAGE_CAPACITY=findmnt (the tests' stand-in stage) the bounded mount
    it stands for, findmnt's size less its files in 4 KiB pages."""
    if os.environ.get("TRACE_STAGE_CAPACITY") == "findmnt":
        m = findmnt(stage)
        if m is None:
            raise StageError(f"{stage} is not a mount point")
        used = 0
        for d, _, names in os.walk(stage):
            for name in names:
                try:
                    used += -(-os.lstat(os.path.join(d, name)).st_size // PAGE) * PAGE
                except FileNotFoundError:
                    pass
        return used, max(0, m[1] - used)
    st = os.statvfs(stage)
    return (st.f_blocks - st.f_bfree) * st.f_frsize, st.f_bavail * st.f_frsize


def write_json_atomic(path: pathlib.Path, obj: dict) -> None:
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
        f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def mark(stage: pathlib.Path, session: str, base: str, outdir: pathlib.Path) -> dict:
    if (stage / MARKER).exists():
        raise StageError(f"{stage} is marked already")
    marker = {"session": session, "base": base, "outdir": str(outdir.resolve()), "token": secrets.token_hex(8),
              "pid": os.getppid(), "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds")}
    write_json_atomic(stage / MARKER, marker)
    return marker


def read_marker(stage: pathlib.Path) -> dict:
    try:
        m = json.loads((stage / MARKER).read_text())
    except (OSError, ValueError) as e:
        raise StageError(f"{stage} has no complete marker ({e})") from e
    for k in ("session", "base", "outdir", "token"):
        if not isinstance(m.get(k), str) or not m[k]:
            raise StageError(f"{stage}'s marker has no {k}")
    return m


def staged_files(stage: pathlib.Path) -> list[str]:
    """Every regular file under the stage but its own records, as relative paths."""
    out = []
    for p in sorted(stage.rglob("*")):
        rel = p.relative_to(stage)
        if len(rel.parts) == 1 and rel.name in OWN:
            continue
        if p.is_symlink() or (not p.is_file() and not p.is_dir()):
            raise StageError(f"{p} is not a regular file")
        if p.is_file():
            out.append(str(rel))
    return out


def sha256(path: pathlib.Path) -> tuple[str, int]:
    h, n = hashlib.sha256(), 0
    with open(path, "rb") as f:
        while b := f.read(CHUNK):
            h.update(b)
            n += len(b)
    return h.hexdigest(), n


def inventory(stage: pathlib.Path) -> dict:
    """The stage's immutable inventory: taken once, then only checked."""
    marker = read_marker(stage)
    files = []
    for rel in staged_files(stage):
        h, n = sha256(stage / rel)
        files.append({"path": rel, "size": n, "sha256": h})
    inv = {"base": marker["base"], "token": marker["token"], "session": marker["session"], "files": files}
    p = stage / INVENTORY
    if p.exists():
        old = json.loads(p.read_text())
        if old != inv:
            raise StageError(f"{stage} changed after its inventory was taken: publication refused")
        return old
    write_json_atomic(p, inv)
    return inv


def fsync_path(path: pathlib.Path) -> None:
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def fail_point(name: str) -> bool:
    return os.environ.get("TRACE_STAGE_FAIL") == name


def write_once(path: pathlib.Path, data: bytes, token: str) -> str:
    """A durable file of these bytes at path, never overwriting another."""
    if path.exists():
        if path.read_bytes() != data:
            raise StageError(f"{path} exists with other contents: never overwritten")
        fsync_path(path)
        return "present"
    tmp = path.with_name(f".{path.name}.partial-{token}")
    tmp.unlink(missing_ok=True)
    try:
        with open(tmp, "xb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.link(tmp, path)
    finally:
        tmp.unlink(missing_ok=True)
    fsync_path(path.parent)
    return "written"


def publish_file(src: pathlib.Path, dst: pathlib.Path, entry: dict, token: str) -> str:
    if not dst.parent.exists():
        dst.parent.mkdir(parents=True)
        fsync_path(dst.parent.parent)
    if os.path.lexists(dst):
        if dst.is_symlink() or not dst.is_file():
            raise StageError(f"{dst} exists and is not a regular file: never overwritten")
        h, n = sha256(dst)
        if (h, n) != (entry["sha256"], entry["size"]):
            raise StageError(f"{dst} exists with other contents: never overwritten")
        fsync_path(dst)
        return "present"
    tmp = dst.with_name(f".{dst.name}.partial-{token}")
    tmp.unlink(missing_ok=True)  # the stage's own, from an earlier attempt
    try:
        h, n = hashlib.sha256(), 0
        with open(src, "rb") as fi, open(tmp, "xb") as fo:
            while b := fi.read(CHUNK):
                if fail_point("copy"):
                    raise OSError(errno.ENOSPC, "No space left on device (injected)", str(tmp))
                h.update(b)
                n += len(b)
                fo.write(b)
            fo.flush()
            os.fsync(fo.fileno())
        if (h.hexdigest(), n) != (entry["sha256"], entry["size"]):
            raise StageError(f"{src} changed after its inventory was taken")
        back, nb = sha256(tmp)
        if fail_point("hash"):
            back = "0" * 64
        if (back, nb) != (entry["sha256"], entry["size"]):
            raise StageError(f"the copy {tmp} does not read back as {entry['sha256']}")
        if fail_point("link"):
            raise OSError(errno.EIO, "Input/output error (injected)", str(dst))
        try:
            os.link(tmp, dst)
        except FileExistsError:
            if sha256(dst) != (entry["sha256"], entry["size"]):
                raise StageError(f"{dst} appeared with other contents: never overwritten") from None
    finally:
        tmp.unlink(missing_ok=True)
    fsync_path(dst.parent)
    if fail_point("crash-after-link"):
        os._exit(9)
    return "copied"


def ledger_text(inv: dict) -> bytes:
    return "".join(f"{e['sha256']}  {e['path']}\n" for e in inv["files"]).encode()


def inventory_bytes(inv: dict) -> bytes:
    return (json.dumps(inv, indent=1, sort_keys=True) + "\n").encode()


def writeback(mem_path: str) -> tuple[int | None, int | None]:
    try:
        m = meminfo(mem_path)
    except OSError:
        return None, None
    return m.get("Dirty"), m.get("Writeback")


def kb(v: int | None) -> str:
    return "unavailable" if v is None else f"{v // 1024} kB"


def publish(stage: pathlib.Path, outdir: pathlib.Path, mem_path: str, settle_s: float, out) -> dict:
    marker = read_marker(stage)
    if pathlib.Path(marker["outdir"]) != outdir.resolve():
        raise StageError(f"{stage} belongs to the output directory {marker['outdir']}, not {outdir.resolve()}")
    d0, w0 = writeback(mem_path)
    t0 = time.monotonic()
    inv = inventory(stage)
    token, base = inv["token"], inv["base"]
    write_once(outdir / f"{base}.staging.inventory.json", inventory_bytes(inv), token)
    counts = {"copied": 0, "present": 0}
    for e in inv["files"]:
        counts[publish_file(stage / e["path"], outdir / e["path"], e, token)] += 1
    if fail_point("crash-before-ledger"):
        os._exit(9)
    write_once(outdir / f"{base}.staging.sha256", ledger_text(inv), token)
    fsync_path(outdir)
    d1, w1 = writeback(mem_path)
    settled, t_settle = False, time.monotonic()
    while True:
        _, w = writeback(mem_path)
        if w is not None and w <= MIB:
            settled = True
            break
        if time.monotonic() - t_settle >= settle_s:
            break
        time.sleep(0.5)
    size = sum(e["size"] for e in inv["files"])
    print(f"published: {len(inv['files'])} files, {size} bytes ({counts['copied']} copied, {counts['present']} "
          f"already in place) in {time.monotonic() - t0:.1f} s; ledger {base}.staging.sha256, inventory "
          f"{base}.staging.inventory.json; each file, the ledger and the directory fsynced", file=out)
    print(f"writeback (system-wide, /proc/meminfo): Dirty {kb(d0)}, Writeback {kb(w0)} before; Dirty {kb(d1)}, "
          f"Writeback {kb(w1)} after; Writeback {'at or under 1 MiB' if settled else 'not under 1 MiB'} "
          f"{time.monotonic() - t_settle:.1f} s later (bound {settle_s:.0f} s)", file=out)
    return inv


def mounted(stage: pathlib.Path) -> bool:
    return findmnt(stage) is not None


def unmount_and_remove(stage: pathlib.Path) -> None:
    """Unmounts the stage when it is mounted, then removes the bare mount
    point with rmdir, which removes only an empty directory. Nothing inside
    the mount point is read or deleted past the unmount (see the module's
    description), so a retry completes the same way."""
    if not STAGE_NAME.match(stage.name):
        raise StageError(f"{stage} is not named as a stage of tools/trace_session.sh "
                         "(<session>-<attempt>-<seq>-<8 hex digits>): not touched")
    if mounted(stage):
        r = subprocess.run([*sudo(), os.environ.get("UMOUNT", "umount"), str(stage)], capture_output=True, text=True)
        if r.returncode != 0:
            raise StageError(f"the unmount of {stage} failed: {(r.stderr or r.stdout).strip()}")
        if mounted(stage):
            raise StageError(f"{stage} is still mounted after its unmount")
        if fail_point("crash-after-unmount"):
            os._exit(9)
    if not os.path.lexists(stage):
        return
    if stage.is_symlink() or not stage.is_dir():
        raise StageError(f"{stage} is not a directory: not removed")
    r = subprocess.run([*sudo(), os.environ.get("RMDIR", "rmdir"), str(stage)], capture_output=True, text=True)
    if r.returncode != 0:
        raise StageError(f"the mount point {stage} is unmounted but was not removed (rmdir, which removes only an "
                         f"empty directory): {(r.stderr or r.stdout).strip()}")
    if os.path.lexists(stage):
        raise StageError(f"{stage} remains")


def empty_tree(stage: pathlib.Path) -> None:
    """Deletes the staged data, keeping the stage's own records until its unmount."""
    for p in sorted(stage.rglob("*"), key=lambda q: len(q.parts), reverse=True):
        if len(p.relative_to(stage).parts) == 1 and p.name in OWN:
            continue
        if p.is_dir() and not p.is_symlink():
            p.rmdir()
        else:
            p.unlink()


def release(stage: pathlib.Path, outdir: pathlib.Path, out) -> None:
    if not os.path.lexists(stage):
        print(f"released: {stage} is gone already", file=out)
        return
    if not mounted(stage):
        # A release (or abort) unmounted it and was interrupted or failed
        # before the removal: the staged files and the marker went with the
        # tmpfs, and the bare mount point is all that remains.
        unmount_and_remove(stage)
        print(f"released: {stage} was no longer mounted (its staged files and records went with its tmpfs); its "
              f"bare mount point removed. Nothing was verified again here: the ledger in {outdir} stays checkable "
              "with sha256sum -c", file=out)
        return
    marker = read_marker(stage)
    try:
        inv = json.loads((stage / INVENTORY).read_text())
    except (OSError, ValueError) as e:
        raise StageError(f"{stage} was never published (no inventory): release refused") from e
    base, token = inv["base"], marker["token"]
    if inv.get("token") != token:
        raise StageError(f"{stage}'s inventory is not its own: release refused")
    led = outdir / f"{base}.staging.sha256"
    if not led.exists() or led.read_bytes() != ledger_text(inv):
        raise StageError(f"the ledger {led} is missing or does not match the inventory: release refused")
    ip = outdir / f"{base}.staging.inventory.json"
    if not ip.exists() or ip.read_bytes() != inventory_bytes(inv):
        raise StageError(f"the inventory {ip} is missing or does not match: release refused")
    for e in inv["files"]:
        dst = outdir / e["path"]
        if not dst.is_file() or sha256(dst) != (e["sha256"], e["size"]):
            raise StageError(f"{dst} does not hold the inventory's contents: release refused")
        fsync_path(dst)
    for d in {(outdir / e["path"]).parent for e in inv["files"]} | {outdir}:
        fsync_path(d)
    empty_tree(stage)
    unmount_and_remove(stage)
    print(f"released: {len(inv['files'])} files verified in {outdir} against the ledger; {stage} emptied, "
          "unmounted and removed", file=out)


def abort(stage: pathlib.Path, out) -> None:
    """A stage without a complete marker, or never written: undone only while
    it holds no data. A mounted one is checked through its mounted root (the
    operator's); a bare mount point (never mounted, or unmounted already) is
    never entered, rmdir removing it only when empty."""
    if not os.path.lexists(stage):
        print(f"aborted: {stage} does not exist", file=out)
        return
    if not mounted(stage):
        unmount_and_remove(stage)
        print(f"aborted: {stage} was not mounted; its bare mount point removed (it was empty)", file=out)
        return
    data = [str(p.relative_to(stage)) for p in stage.iterdir() if p.name not in OWN]
    if data:
        raise StageError(f"{stage} holds data ({', '.join(sorted(data)[:5])}): kept")
    if (stage / INVENTORY).exists():
        raise StageError(f"{stage} has an inventory: publish and release it instead")
    unmount_and_remove(stage)
    print(f"aborted: {stage} held no data; unmounted and removed", file=out)


def leftovers(root: pathlib.Path, out) -> int:
    if not root.is_dir():
        return 0
    n = 0
    for d in sorted(root.iterdir()):
        n += 1
        if not mounted(d):
            print(f"{d}: not mounted, a bare mount point (its tmpfs and records are gone); release or abort removes "
                  "it (rmdir: only when empty)", file=out)
            continue
        try:
            m = read_marker(d)
            published = (d / INVENTORY).exists()
            print(f"{d}: run {m['base']} of session {m['session']}, output directory {m['outdir']}; "
                  f"{'inventory taken' if published else 'not inventoried'}", file=out)
        except (StageError, OSError) as e:
            print(f"{d}: no complete marker ({e})", file=out)
    return n


def guard(stage: pathlib.Path, min_free: int, min_avail: int, interval: float, stop_pids: list[int],
          mem_path: str, out) -> None:
    """Samples the stage's free space and MemAvailable until SIGINT or SIGTERM;
    below either bound it stops the given recorders (SIGINT) once, and says why."""
    stop = False

    def on_signal(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    n, peak, least_free, least_avail, tripped = 0, 0, None, None, None
    print(f"guard: every {interval:g} s, stop below {min_free} bytes free or {min_avail} bytes MemAvailable; "
          f"recorders {','.join(map(str, stop_pids)) or 'none'}", file=out, flush=True)
    while not stop:
        used, free = space(stage)
        try:
            avail = meminfo(mem_path).get("MemAvailable")
        except OSError:
            avail = None
        n += 1
        peak = max(peak, used)
        least_free = free if least_free is None else min(least_free, free)
        if avail is not None:
            least_avail = avail if least_avail is None else min(least_avail, avail)
        if tripped is None and (free < min_free or (avail is not None and avail < min_avail)):
            why = (f"the stage has {free} bytes free (bound {min_free})" if free < min_free else
                   f"MemAvailable is {avail} bytes (bound {min_avail})")
            tripped = why
            sent = []
            for pid in stop_pids:
                try:
                    os.kill(pid, signal.SIGINT)
                    sent.append(str(pid))
                except OSError as e:
                    sent.append(f"{pid} ({e.strerror})")
            print(f"guard stop: {why} at monotonic_ns {time.clock_gettime_ns(time.CLOCK_MONOTONIC)}; "
                  f"SIGINT to {', '.join(sent) or 'no recorder'}", file=out, flush=True)
        time.sleep(interval)
    print(f"guard: {n} samples; sampled peak used {peak} bytes; least free {least_free} bytes; least MemAvailable "
          f"{least_avail if least_avail is not None else 'unavailable'} bytes; stop: {tripped or 'none'}",
          file=out, flush=True)


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("budget")
    b.add_argument("--mode", required=True, choices=MODES)
    b.add_argument("--duration-s", type=float, required=True)
    b.add_argument("--rate-hz", type=float, default=0.0)
    b.add_argument("--buffer-kb", type=int, required=True)
    b.add_argument("--cpus", type=int, required=True)
    b.add_argument("--usbmon", type=int, default=1)
    b.add_argument("--perf-hz", type=int, default=0)
    b.add_argument("--perf-stack", type=int, default=0)
    b.add_argument("--exerciser", type=int, default=0)
    b.add_argument("--payload", type=int)
    b.add_argument("--subbuf", type=int)
    b.add_argument("--tk-buffer-kb", type=int)
    g = sub.add_parser("geometry")
    g.add_argument("--tracefs", type=pathlib.Path, required=True)
    g.add_argument("--instance")
    bu = sub.add_parser("buffers")
    bu.add_argument("--tracefs", type=pathlib.Path, required=True)
    bu.add_argument("--instance", action="append", default=[])
    bu.add_argument("--buffer-kb", type=int, required=True)
    bu.add_argument("--cpu-list", required=True)
    bu.add_argument("--payload", type=int, required=True)
    bu.add_argument("--subbuf", type=int, required=True)
    for name in ("verify", "mark", "guard", "usage", "publish", "release", "abort"):
        s = sub.add_parser(name)
        s.add_argument("--stage", type=pathlib.Path, required=True)
        if name == "verify":
            s.add_argument("--size-mib", type=int, required=True)
        if name == "mark":
            s.add_argument("--session", required=True)
            s.add_argument("--base", required=True)
        if name in ("mark", "publish", "release"):
            s.add_argument("--outdir", type=pathlib.Path, required=True)
        if name == "guard":
            s.add_argument("--min-free", type=int, required=True)
            s.add_argument("--min-available", type=int, required=True)
            s.add_argument("--interval", type=float, default=0.2)
            s.add_argument("--stop-pid", type=int, action="append", default=[])
        if name == "publish":
            s.add_argument("--settle-s", type=float, default=30.0)
    lo = sub.add_parser("leftovers")
    lo.add_argument("--root", type=pathlib.Path, required=True)
    for s in (b, sub.choices["guard"], sub.choices["publish"]):
        s.add_argument("--meminfo", default="/proc/meminfo")
    a = p.parse_args()
    out = sys.stdout
    try:
        if a.cmd == "budget":
            r = budget(a.mode, a.duration_s, a.rate_hz, a.buffer_kb, a.cpus, bool(a.usbmon), a.perf_hz,
                       a.perf_stack, bool(a.exerciser), meminfo(a.meminfo), a.payload, a.subbuf, tk_buffer_kb=a.tk_buffer_kb)
            for k in ("size_mib", "kernel", "tk_physical", "headroom", "need", "available", "guard_min_free",
                      "guard_min_available"):
                print(f"{k}={r[k]}", file=out)
            items = ", ".join(f"{k} {gb(v)}" for k, v in r["items"].items())
            print(f"staging budget ({a.mode}, {a.duration_s:g} s, provisional): raw trace {gb(r['raw'])}, assembled "
                  f"{gb(r['assembled'])}, other streams {gb(r['a'])} ({items}); guard reserve "
                  f"{r['reserve'] // MIB} MiB; tmpfs cap {r['size_mib']} MiB; "
                  + (f"kernel buffers {gb(r['kernel'])} ({r['arrays']} arrays x {a.cpus} CPUs x "
                     f"{r['physical'] // 1024} KiB, {r['pages']} subbuffers plus the reader's of {a.subbuf} bytes; "
                     f"data-plus-reader allocation, bookkeeping excluded; expected readback {r['readback_kib']} KiB "
                     f"per CPU for {a.buffer_kb} KiB requested"
                     + (f"; timekeeping recorder {gb(r['tk_physical'])} for {a.tk_buffer_kb} KiB per CPU" if r["tk_physical"] else "")
                     + "), " if r["arrays"] else "no kernel buffers, ")
                  + f"headroom {gb(r['headroom'])}: MemAvailable needed "
                  f"{gb(r['need'])}, available {gb(r['available'])}{'' if r['ok'] else ': NOT ENOUGH'}", file=out)
            return 0 if r["ok"] else 1
        if a.cmd == "geometry":
            gm = read_geometry(a.tracefs, a.instance)
            print(f"payload={gm['payload']}\nsubbuf={gm['subbuf']}\noffset={gm['offset']}", file=out)
            print(f"kernel buffers' geometry, array {a.instance or 'main'}: {describe_geometry(gm)}", file=out)
            return 0
        if a.cmd == "buffers":
            return 0 if check_buffers(a.tracefs, a.instance, a.buffer_kb, a.cpu_list, a.payload, a.subbuf, out) else 1
        if a.cmd == "verify":
            print(f"verified: {a.stage}: {verify(a.stage, a.size_mib)}", file=out)
        elif a.cmd == "mark":
            m = mark(a.stage, a.session, a.base, a.outdir)
            print(f"marked: {a.stage}: run {m['base']}, session {m['session']}, token {m['token']}", file=out)
        elif a.cmd == "guard":
            guard(a.stage, a.min_free, a.min_available, a.interval, a.stop_pid, a.meminfo, out)
        elif a.cmd == "usage":
            used, free = space(a.stage)
            print(f"used {used} free {free}", file=out)
        elif a.cmd == "publish":
            publish(a.stage, a.outdir, a.meminfo, a.settle_s, out)
        elif a.cmd == "release":
            release(a.stage, a.outdir, out)
        elif a.cmd == "abort":
            abort(a.stage, out)
        elif a.cmd == "leftovers":
            return 1 if leftovers(a.root, out) else 0
    except (StageError, OSError, ValueError) as e:
        print(f"trace_stage: {a.cmd}: {e}", file=sys.stderr)
        if a.cmd == "publish":
            print(f"  the stage is kept; retry: python3 {sys.argv[0]} publish --stage {a.stage} --outdir {a.outdir}, "
                  f"then: python3 {sys.argv[0]} release --stage {a.stage} --outdir {a.outdir}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
