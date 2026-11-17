#!/usr/bin/env python3
"""The servos' state outside the timed runs, for the loaded emulator validation (tools/option_a.sh, its third run):
every listed servo's Torque Enable, Bus Watchdog and Hardware Error Status, read with dxl_inspect, each required to be
0. Read-only on the bus; it writes only its record.

    servo_state.py --port PORT --inspect PATH --out FILE --label NAME [--ids 2-9] [--timeout S]

It reads each (servo, register) once, in order, three registers for each listed ID (24 reads for 2-9). A read passes
only if dxl_inspect exits 0 and prints exactly one line holding a decimal integer; the state passes only if every read
passes and every value is 0. A declared all-zero baseline: a nonzero value fails it, whatever it means (a nonzero Bus
Watchdog is not by itself an expiry).

The record: a header (its label, time, port, the inspector's path and SHA-256, the IDs), one row per read (its ID,
register, status, value or "-", and the read's error text if any), then "rows=N expected=M" and a last line
"result=OK" or "result=FAIL reason=TEXT". It is created exclusively without following a link (a FILE that exists in
any form refuses before anything is read), written, synced (the file and its directory), then read back: the bytes
must equal what was written, and parsed again, the reread must hold exactly the expected rows, each (ID, register)
once, and the same result. An empty, truncated or partial record therefore never passes. A failed check is still
recorded, with everything read so far.

Exit: 0 the record is complete, verified, and every value 0; 1 a read failed, a value is not 0 or not an integer, or
the record could not be written or verified (then "the record was not written" is said, never OK); 2 a usage error,
or FILE exists (nothing read, nothing written).
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import os
import pathlib
import re
import subprocess
import sys

REGISTERS = ("torque_enable", "bus_watchdog", "hardware_error_status")
ROW = re.compile(r"id=(\d+) register=(\w+) status=(\S+) value=(\S+)(?: error=(.*))?")
VALUE = re.compile(r"(\d+)\n")


def ids_of(spec: str) -> list[int]:
    """A strict ID list ('2-9', '2,3,5-7'): ascending, no repeats, each 1 to 252; ValueError otherwise."""
    out: list[int] = []
    for part in spec.split(","):
        a, sep, b = part.partition("-")
        if not a.isdigit() or (sep and not b.isdigit()):
            raise ValueError(spec)
        lo, hi = int(a), int(b) if sep else int(a)
        if hi < lo or lo < 1 or hi > 252 or (out and lo <= out[-1]):
            raise ValueError(spec)
        out.extend(range(lo, hi + 1))
    return out


def read_one(inspect: str, port: str, sid: int, reg: str, timeout: float) -> tuple[str, str, str]:
    """(status, value or '-', error text) of one dxl_inspect read."""
    try:
        r = subprocess.run([inspect, "--port", port, "read", str(sid), reg], stdin=subprocess.DEVNULL,
                           capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return "timeout", "-", f"no answer within {timeout:g} s"
    except OSError as e:
        return "error", "-", str(e)
    err = " / ".join(x.strip() for x in r.stderr.splitlines() if x.strip())[:200]
    m = VALUE.fullmatch(r.stdout)
    if r.returncode != 0:
        return str(r.returncode), "-", err or "(no message)"
    if not m:
        return "0", "-", f"output not one integer line: {r.stdout[:60]!r}"
    return "0", m[1], err


def judge(rows: list[tuple[int, str, str, str, str]], ids: list[int]) -> str:
    """'' if the rows are complete and all zero, else why not."""
    want = {(i, r) for i in ids for r in REGISTERS}
    got = [(i, r) for i, r, _, _, _ in rows]
    if len(got) != len(want) or set(got) != want:
        return f"{len(set(got) & want)} of {len(want)} expected reads"
    bad = [f"id {i} {r}: status {s}" for i, r, s, v, _ in rows if s != "0" or v == "-"]
    if bad:
        return "a read failed: " + "; ".join(bad)[:300]
    nonzero = [f"id {i} {r} = {v}" for i, r, _, v, _ in rows if int(v) != 0]
    if nonzero:
        return "not 0: " + "; ".join(nonzero)[:300]
    return ""


def render(head: list[str], rows: list[tuple[int, str, str, str, str]], n_want: int, why: str) -> str:
    out = list(head)
    for i, r, s, v, e in rows:
        out.append(f"id={i} register={r} status={s} value={v}" + (f" error={e}" if e else ""))
    out.append(f"rows={len(rows)} expected={n_want}")
    out.append("result=OK" if not why else f"result=FAIL reason={why}")
    return "\n".join(out) + "\n"


def reread(text: str, ids: list[int]) -> str:
    """Why a written record does not hold the complete, consistent evidence ('' if it does)."""
    lines = text.splitlines()
    if not lines or not text.endswith("\n"):
        return "empty or unterminated"
    rows = []
    for ln in lines:
        if ln.startswith("#"):
            continue
        m = ROW.fullmatch(ln)
        if m:
            rows.append((int(m[1]), m[2], m[3], m[4], m[5] or ""))
    tail = [ln for ln in lines if not ln.startswith("#") and not ROW.fullmatch(ln)]
    n_want = len(ids) * len(REGISTERS)
    if len(tail) != 2 or tail[0] != f"rows={len(rows)} expected={n_want}" or not tail[1].startswith("result="):
        return "its summary lines are missing or wrong"
    why = judge(rows, ids)
    if tail[1] != ("result=OK" if not why else f"result=FAIL reason={why}"):
        return "its result does not follow from its rows"
    return ""


def write_record(path: pathlib.Path, text: str) -> str:
    """'' once the record is written exclusively, synced and read back equal; else why not."""
    data = text.encode()
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
    except OSError as e:
        return f"cannot create it: {e}"
    try:
        os.fchmod(fd, 0o644)
        view = memoryview(data)
        while view:
            n = os.write(fd, view)
            view = view[n:]
        os.fsync(fd)
    except OSError as e:
        return f"cannot write it: {e}"
    finally:
        os.close(fd)
    try:
        dfd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(dfd)
        finally:
            os.close(dfd)
        if path.read_bytes() != data:
            return "it reads back differently"
    except OSError as e:
        return f"cannot verify it: {e}"
    return ""


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("--port", required=True, help="the emulator's pty (passed to dxl_inspect --port)")
    p.add_argument("--inspect", required=True, help="the dxl_inspect binary")
    p.add_argument("--out", type=pathlib.Path, required=True, help="the record to create (must not exist)")
    p.add_argument("--label", required=True, help="the record's label (start, A, B)")
    p.add_argument("--ids", default="2-9", help="the servo IDs (default 2-9)")
    p.add_argument("--timeout", type=float, default=10.0, help="each read's limit in seconds (default 10)")
    a = p.parse_args(argv)
    try:
        ids = ids_of(a.ids)
    except ValueError:
        print(f"servo_state: --ids {a.ids!r} is not a strict ID list", file=sys.stderr)
        return 2
    if not re.fullmatch(r"[A-Za-z0-9_-]+", a.label) or a.timeout <= 0:
        print("servo_state: --label must be a plain word and --timeout positive", file=sys.stderr)
        return 2
    if os.path.lexists(a.out):
        print(f"servo_state {a.label}: {a.out} exists: nothing read, nothing written", file=sys.stderr)
        return 2
    try:
        digest = hashlib.sha256(pathlib.Path(a.inspect).read_bytes()).hexdigest()
    except OSError as e:
        digest = f"unreadable ({e})"
    head = [f"# servo state {a.label}: Torque Enable, Bus Watchdog and Hardware Error Status, each required to be 0",
            f"# time {datetime.datetime.now().astimezone().isoformat()}",
            f"# port {a.port}", f"# inspect {a.inspect} sha256 {digest}", f"# ids {a.ids}"]
    rows = [(sid, reg, *read_one(a.inspect, a.port, sid, reg, a.timeout)) for sid in ids for reg in REGISTERS]
    why = judge(rows, ids)
    text = render(head, rows, len(ids) * len(REGISTERS), why)
    problem = write_record(a.out, text)
    if not problem:
        try:
            r = reread(a.out.read_text(), ids)
        except (OSError, UnicodeDecodeError) as e:
            r = str(e)
        problem = f"the reread: {r}" if r else ""
    if problem:
        print(f"servo_state {a.label}: FAIL: the record was not written: {problem} ({a.out})")
        return 1
    if why:
        print(f"servo_state {a.label}: FAIL: {why} ({a.out})")
        return 1
    print(f"servo_state {a.label}: OK ({len(rows)} reads, all 0; {a.out})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
