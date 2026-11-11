#!/usr/bin/env python3
"""The evidence of the timekeeping certificate (tools/timer_clock.py): the
state reads a timer session takes, the kernel's provenance, and the loader
that assembles the recorder's files into the certificate's evidence.

    tk_record.py state --out FILE [--cmdline FILE]     one state read, with the kernel's identity (no privilege)
    tk_record.py log --journal FILE --from-t N --to-t N --out FILE
                                                       the kernel log's scan over the interval
    tk_record.py tool --trace-cmd NAME --out FILE      the identity of the trace-cmd that NAME runs (no privilege)

A state read is adjtimex(2) with modes 0, then with ADJ_OFFSET_SS_READ (for
the adjtime remainder), neither of which changes anything; the two monotonic
stamps bracket the reads. load() returns None when a capture has no record at
all. A record with a missing, unreadable or malformed part names that part's
read_status: the certificate treats it as UNSUPPORTED, and never as an empty
successful record. A row that repeats, contradicts another or is not
understood makes its file malformed.

A tool's identity is the file its name resolves to, that file's SHA-256, and
the package that installed it with its version and architecture (dpkg-query,
or $DPKG_QUERY). The certificate requires the audited build for every use of
trace-cmd it rests on; the libraries it runs with are recorded, not judged.

The files of a capture <run>.csv, written by tools/trace_session.sh:
    .tk-before.json .tk-after.json   the state reads around the capture
    .tk-log.json                     the kernel log's scan
    .tk-times.txt                    armed_t and confirmed_t
    .tk-enable-before.txt .tk-enable-after.txt   the probes' enable states
    .tk-profile.txt .tk-stats.txt    the probes' hits and misses, the buffers' losses
    .tk.txt                          trace-cmd report -t of the recorder's instance
    .tk-tool-before.json .tk-tool-after.json     trace-cmd's identity before the main
                                     trace starts and after the recorder's report
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
import pathlib
import platform
import re
import shutil
import subprocess
import sys
import threading
import time

INSTANCE = "rtctrl_tk"
PROBES = ("ntp_adjtimex", "ntp_clear", "update_entry", "update_return")
ADJ_OFFSET_SS_READ = 0xa001
CLOCKSOURCE = "/sys/devices/system/clocksource/clocksource0/current_clocksource"
SUFFIXES = (".tk-before.json", ".tk-after.json", ".tk-log.json", ".tk-times.txt", ".tk-enable-before.txt",
            ".tk-enable-after.txt", ".tk-profile.txt", ".tk-stats.txt", ".tk.txt", ".tk-tool-before.json",
            ".tk-tool-after.json")
# The libraries trace-cmd runs with (libtracecmd is linked into it): recorded beside its identity, not part of the audit.
TRACE_CMD_LIBRARIES = ("libtraceevent1", "libtraceevent1-plugin", "libtracefs1")
DPKG_OWNER = re.compile(r"([a-z0-9][a-z0-9+.-]*)(?::[a-z0-9-]+)?: (.+)")
# The kernel log's time is the printk clock, not CLOCK_MONOTONIC: a clock event this near the interval counts as in it.
LOG_GUARD_NS = 5_000_000_000
# What the interval must not contain: a clocksource change, TSC trouble, a suspend.
LOG_EVENT = re.compile(r"clocksource|\btsc\b|\bTSC\b|PM: (?:suspend|hibernation)|timekeeping", re.IGNORECASE)
LOG_LINE = re.compile(r"^\[\s*(\d+)\.(\d{6})\]\s+\S+\s+kernel:\s?(.*)$")
TSC_RATE = re.compile(r"tsc: (?:Refined TSC clocksource calibration: |Detected )(\d+\.\d+) MHz(?: TSC)?$")
# trace-cmd report -t: nanosecond stamps only, optionally prefixed by the instance's name.
REC_LINE = re.compile(r"^(?:\s*(?P<inst>[A-Za-z_][A-Za-z0-9_]*):\s+)?\s*(?P<comm>.*?)-(?P<pid>\d+)\s+\[(?P<cpu>\d+)\]\s+"
                      r"(?:\S+\s+)?(?P<s>\d+)\.(?P<ns>\d{9}):\s+(?P<ev>\w+):\s*(?P<rest>.*)$")
REC_HEADER = re.compile(r"^(?:cpus=\d+|CPU\s*\d+ is empty|\s*)$")
ARGS = re.compile(r"\btkid=(\S+)(?:\s+modes=(\S+))?")


class Timex(ctypes.Structure):
    _fields_ = [("modes", ctypes.c_uint), ("offset", ctypes.c_long), ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long), ("esterror", ctypes.c_long), ("status", ctypes.c_int),
                ("constant", ctypes.c_long), ("precision", ctypes.c_long), ("tolerance", ctypes.c_long),
                ("time_sec", ctypes.c_long), ("time_usec", ctypes.c_long), ("tick", ctypes.c_long),
                ("ppsfreq", ctypes.c_long), ("jitter", ctypes.c_long), ("shift", ctypes.c_int),
                ("stabil", ctypes.c_long), ("jitcnt", ctypes.c_long), ("calcnt", ctypes.c_long),
                ("errcnt", ctypes.c_long), ("stbcnt", ctypes.c_long), ("tai", ctypes.c_int),
                ("pad", ctypes.c_int * 11)]


def read_text(path: str) -> tuple[str, str | None]:
    """(contents, None), or ("", the reason it could not be read). Kernel text is taken byte for byte."""
    try:
        return pathlib.Path(path).read_bytes().decode("latin-1"), None
    except OSError as e:
        return "", f"{path}: {e.strerror or e}"


def state_read(cmdline_path: str = "/proc/cmdline", clocksource_path: str = CLOCKSOURCE,
               config_dir: str = "/boot") -> dict:
    """One read of the NTP state, bracketed, with the kernel's identity. Only reads."""
    libc = ctypes.CDLL("libc.so.6", use_errno=True)
    plain, slew = Timex(), Timex()
    plain.modes, slew.modes = 0, ADJ_OFFSET_SS_READ
    t0 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    r0 = libc.adjtimex(ctypes.byref(plain))
    r1 = libc.adjtimex(ctypes.byref(slew))
    t1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    out: dict = {"read_status": "ok", "t_start": t0, "t_end": t1, "tid": threading.get_native_id(), "ret": int(r0),
                 "offset": int(plain.offset), "freq": int(plain.freq), "status": int(plain.status),
                 "constant": int(plain.constant), "tick": int(plain.tick), "time_adjust": int(slew.offset)}
    if r0 < 0 or r1 < 0:
        out["read_status"] = f"adjtimex failed ({r0}, {r1})"
    release = platform.release()
    out["release"] = release
    cmdline, why = read_text(cmdline_path)
    out["cmdline"] = cmdline.rstrip("\n") if why is None else None
    out["cmdline_status"] = why or "ok"
    cs, why = read_text(clocksource_path)
    out["clocksource"] = cs.strip() if why is None else None
    out["clocksource_status"] = why or "ok"
    try:
        out["config_sha256"] = hashlib.sha256(pathlib.Path(config_dir, f"config-{release}").read_bytes()).hexdigest()
        out["config_status"] = "ok"
    except OSError as e:
        out["config_sha256"], out["config_status"] = None, str(e)
    return out


def tool_identity(name: str, dpkg_query: str | None = None) -> dict:
    """The identity of the executable NAME resolves to: its path, its SHA-256, and the package that installed it, with
    its version and architecture. Only reads. A part that cannot be read names read_status, and nothing is assumed."""
    dq = dpkg_query or os.environ.get("DPKG_QUERY", "dpkg-query")
    found = shutil.which(name)
    if found is None:
        return {"read_status": f"{name} is not found"}
    path = os.path.realpath(found)
    out: dict = {"read_status": "ok", "path": path}
    try:
        out["sha256"] = hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
    except OSError as e:
        out["read_status"] = f"{path}: {e.strerror or e}"
        return out

    def query(*args: str) -> tuple[int, str, str]:
        try:
            p = subprocess.run([dq, *args], capture_output=True, text=True, timeout=30, env={**os.environ, "LC_ALL": "C"})
        except (OSError, subprocess.SubprocessError) as e:
            return -1, "", f"{dq}: {e}"
        return p.returncode, p.stdout, p.stderr.strip()

    rc, text, err = query("-S", path)
    rows = text.splitlines()
    m = DPKG_OWNER.fullmatch(rows[0]) if rc == 0 and len(rows) == 1 else None
    if m is None or m[2] != path:
        out["read_status"] = f"the package that installed {path} is not known ({(err or text.strip())[:200] or f'exit {rc}'})"
        return out
    rc, text, err = query("-W", "-f", "${Package}\t${Version}\t${Architecture}\t${db:Status-Abbrev}\n", m[1])
    rows = text.splitlines()
    f = rows[0].split("\t") if rc == 0 and len(rows) == 1 else []
    if len(f) != 4 or f[0] != m[1] or not f[3].startswith("ii"):
        out["read_status"] = f"the package {m[1]} is not read as installed ({(err or text.strip())[:200] or f'exit {rc}'})"
        return out
    out.update(package=f[0], version=f[1], architecture=f[2])
    # dpkg-query exits 1 when one of them is not installed, and still lists the others.
    rc, text, err = query("-W", "-f", "${Package}\t${Version}\t${db:Status-Abbrev}\n", *TRACE_CMD_LIBRARIES)
    libs = {g[0]: g[1] for g in (ln.split("\t") for ln in text.splitlines()) if len(g) == 3 and g[2].startswith("ii")}
    out["libraries"] = {lib: libs.get(lib) for lib in TRACE_CMD_LIBRARIES}
    return out


def read_tool(path) -> dict:
    """A recorded identity (tool_identity's), or one whose read_status says why it could not be read."""
    v, st = read_json(str(path))
    if st != "ok":
        return {"read_status": st}
    return v if isinstance(v, dict) else {"read_status": f"{path}: not a mapping"}


def scan_log(text: str, from_t: int, to_t: int) -> dict:
    """The kernel log's clock events near [from_t, to_t], how much of it the log covers, and the TSC's rate."""
    first, events, khz, rows = None, [], None, 0
    for line in text.splitlines():
        m = LOG_LINE.match(line)
        if not m:
            continue
        rows += 1
        t = int(m[1]) * 1_000_000_000 + int(m[2]) * 1000
        first = t if first is None else first
        msg = m[3]
        r = TSC_RATE.search(msg)
        if r:
            khz = round(float(r[1]) * 1000)
        if from_t - LOG_GUARD_NS <= t <= to_t + LOG_GUARD_NS and LOG_EVENT.search(msg):
            events.append(msg[:200])
    if not rows:
        return {"read_status": "the kernel log has no readable rows"}
    return {"read_status": "ok", "covers": [first, to_t], "clock_events": events, "tsc_khz": khz}


def _unique_keys(pairs: list) -> dict:
    keys = [k for k, _ in pairs]
    dup = sorted({k for k in keys if keys.count(k) > 1})
    if dup:
        raise ValueError(f"the key {dup[0]!r} appears more than once")
    return dict(pairs)


def _no_constant(name: str):
    raise ValueError(f"{name} is not a number")


def read_json(path: str) -> tuple[dict | None, str]:
    """JSON read strictly: a key given twice, or NaN or Infinity, is malformed rather than a value chosen silently."""
    text, why = read_text(path)
    if why:
        return None, why
    try:
        v = json.loads(text, object_pairs_hook=_unique_keys, parse_constant=_no_constant)
    except ValueError as e:
        return None, f"{path}: not JSON ({e})"
    return v, "ok"


# Every evidence file is read whole: each expected key exactly once, nothing else, and every value in its exact form. A row
# that repeats, contradicts or is not understood makes the file unreadable; it is never overwritten, skipped or normalised.
STATE_ROW = re.compile(r"([a-z_]+) ([01])")
PROFILE_ROW = re.compile(r"([a-z_]+) (\d+) (\d+)")
TIME_ROW = re.compile(r"(armed_t|confirmed_t) (\d+)")
# The kernel's per-CPU buffer stats (kernel/trace/trace.c tracing_stats_read): these keys, each once per CPU.
STATS_KEYS = ("entries", "overrun", "commit overrun", "bytes", "oldest event ts", "now ts", "dropped events", "read events")
STATS_COUNTS = ("overrun", "commit overrun", "dropped events")
STATS_ROW = re.compile(r"(" + "|".join(STATS_KEYS) + r"):\s*(\S+)\s*")


def _rows(text: str, row: re.Pattern, keys: tuple, what: str) -> tuple[dict, str]:
    """Rows of `row`, one per key of `keys`, every key present; the matches by key, or the first reason it fails."""
    out: dict = {}
    for ln in text.splitlines():
        m = row.fullmatch(ln)
        if not m or m[1] not in keys:
            return {}, f"an unreadable {what} row: {ln!r}"
        if m[1] in out:
            return {}, f"the {what} row for {m[1]} appears more than once"
        out[m[1]] = m
    missing = [k for k in keys if k not in out]
    if missing:
        return {}, f"no {what} row for {', '.join(missing)}"
    return out, "ok"


def parse_states(text: str) -> tuple[dict, str]:
    """Each probe's enable state, exactly "0" or "1" (the kernel's soft-mode "1*" is neither)."""
    rows, st = _rows(text, STATE_ROW, PROBES, "enable state")
    return ({k: m[2] == "1" for k, m in rows.items()}, st) if st == "ok" else ({}, st)


def parse_profile(text: str) -> tuple[dict, str]:
    rows, st = _rows(text, PROFILE_ROW, PROBES, "profile")
    return ({k: (int(m[2]), int(m[3])) for k, m in rows.items()}, st) if st == "ok" else ({}, st)


def parse_times(text: str) -> tuple[dict, str]:
    """armed_t, then confirmed_t (written only once the end's read-back confirmed every probe)."""
    rows, st = _rows(text, TIME_ROW, ("armed_t", "confirmed_t"), "recorder time")
    if st != "ok":
        return {}, st
    if list(rows) != ["armed_t", "confirmed_t"]:
        return {}, "confirmed_t comes before armed_t"
    return {k: int(m[2]) for k, m in rows.items()}, "ok"


def parse_stats(text: str) -> tuple[dict, str]:
    """The sums of the CPUs' overruns and drops. Every CPU the recorder listed must be here, each key at most once, with
    all three counts; the CPU count comes last and once."""
    blocks: dict[int, dict] = {}
    declared, cpu = None, None
    for ln in text.splitlines():
        if declared is not None:
            return {}, f"a stats row after the CPU count: {ln!r}"
        m = re.fullmatch(r"CPU: (\d+)", ln)
        if m:
            cpu = int(m[1])
            if cpu in blocks:
                return {}, f"CPU {cpu}'s stats appear twice"
            blocks[cpu] = {}
            continue
        m = re.fullmatch(r"cpus: (\d+)", ln)
        if m:
            declared, cpu = int(m[1]), None
            continue
        m = STATS_ROW.fullmatch(ln)
        if not m or cpu is None:
            return {}, f"an unreadable stats row: {ln!r}"
        if m[1] in blocks[cpu]:
            return {}, f"CPU {cpu}'s {m[1]} appears twice"
        if m[1] in STATS_COUNTS and not m[2].isdigit():
            return {}, f"CPU {cpu}'s {m[1]} is {m[2]!r}"
        blocks[cpu][m[1]] = m[2]
    if declared is None or declared != len(blocks) or not blocks:
        return {}, f"the stats cover {len(blocks)} CPUs, and the recorder listed {declared}"
    for c, b in sorted(blocks.items()):
        if any(k not in b for k in STATS_COUNTS):
            return {}, f"CPU {c}'s stats lack a count"
    return {"read_status": "ok", "overrun": sum(int(b["overrun"]) + int(b["commit overrun"]) for b in blocks.values()),
            "dropped": sum(int(b["dropped events"]) for b in blocks.values())}, "ok"


def parse_record(text: str, name: str = INSTANCE) -> dict:
    """The recorder's report: its calls, its updates paired per CPU, and whatever could not be read.
    A row is an event of the four probes, a known header, or a reason the record is not readable."""
    calls, rows, why = [], [], None
    for n, ln in enumerate(text.splitlines(), 1):
        if REC_HEADER.match(ln):
            continue
        m = REC_LINE.match(ln)
        if not m or m["ev"] not in PROBES or (m["inst"] is not None and m["inst"] != name):
            why = why or f"line {n} is not a nanosecond record of the recorder: {ln.strip()[:100]!r}"
            continue
        t, cpu, pid, ev, rest = int(m["s"]) * 1_000_000_000 + int(m["ns"]), int(m["cpu"]), int(m["pid"]), m["ev"], m["rest"]
        if ev in ("ntp_adjtimex", "ntp_clear"):
            a = ARGS.search(rest)
            try:
                if not a or (ev == "ntp_adjtimex" and a[2] is None):
                    raise ValueError
                calls.append({"t": t, "name": ev, "tkid": int(a[1], 0), "modes": int(a[2], 0) if a[2] else 0, "tid": pid})
            except ValueError:
                why = why or f"line {n}: unreadable arguments: {rest[:80]!r}"
        else:
            rows.append((t, cpu, ev))
    updates, un_e, un_r, open_ = [], [], [], {}
    # A stable sort by time keeps the file's order for equal stamps, so an entry stays before its return.
    for t, cpu, ev in sorted(rows, key=lambda r: r[0]):
        if ev == "update_entry":
            if cpu in open_:
                un_e.append(open_[cpu])
            open_[cpu] = t
        elif cpu in open_:
            updates.append({"entry": open_.pop(cpu), "ret": t, "cpu": cpu})
        else:
            un_r.append(t)
    un_e.extend(open_.values())
    return {"read_status": why or "ok", "calls": calls, "updates": updates,
            "unmatched": {"entries": sorted(un_e), "returns": sorted(un_r)}}


def load(csv_path, report_tool: dict | None = None) -> dict | None:
    """The evidence of a capture, or None when it has no timekeeping record at all. report_tool is the identity of the
    trace-cmd that reported the main trace for this analysis (tool_identity's); without it that use is unidentified."""
    base = str(csv_path)
    if not any(pathlib.Path(base + s).exists() for s in SUFFIXES):
        return None
    before, st_b = read_json(base + ".tk-before.json")
    after, st_a = read_json(base + ".tk-after.json")
    logv, st_l = read_json(base + ".tk-log.json")
    before = before if isinstance(before, dict) else {}
    after = after if isinstance(after, dict) else {}
    logv = logv if isinstance(logv, dict) else {}

    def state(v: dict, status: str) -> dict:
        s = {k: v[k] for k in ("t_start", "t_end", "tid", "ret", "offset", "freq", "status", "constant", "tick",
                               "time_adjust") if k in v}
        s["read_status"] = v.get("read_status", status) if status == "ok" else status
        return s

    ev: dict = {"schema": 1}
    kernel_status = next((v for k in ("cmdline_status", "config_status") for v in [before.get(k, st_b)] if v != "ok"), "ok")
    ev["kernel"] = {"read_status": kernel_status if st_b == "ok" else st_b}
    for src, dst in (("release", "release"), ("config_sha256", "config_sha256"), ("cmdline", "cmdline")):
        if before.get(src) is not None:
            ev["kernel"][dst] = before[src]
    if logv.get("tsc_khz") is not None:
        ev["kernel"]["tsc_khz"] = logv["tsc_khz"]
    ev["clocksource"] = {"read_status": "ok" if before.get("clocksource_status") == "ok" and after.get("clocksource_status") == "ok"
                         else f"{before.get('clocksource_status', st_b)}; {after.get('clocksource_status', st_a)}"}
    if before.get("clocksource") is not None:
        ev["clocksource"]["before"] = before["clocksource"]
    if after.get("clocksource") is not None:
        ev["clocksource"]["after"] = after["clocksource"]
    ev["log"] = {"read_status": logv.get("read_status", st_l) if st_l == "ok" else st_l}
    for k in ("covers", "clock_events"):
        if k in logv:
            ev["log"][k] = logv[k]
    ev["state"] = {"before": state(before, st_b), "after": state(after, st_a)}
    # Every use of trace-cmd the certificate rests on: the acquisition (identified before the main trace starts), the
    # recorder's extraction and report (identified after it), and the report of the main trace this analysis read.
    ev["trace_cmd"] = {"before": read_tool(base + ".tk-tool-before.json"), "after": read_tool(base + ".tk-tool-after.json"),
                       "report": report_tool if report_tool is not None
                       else {"read_status": "the trace-cmd that reported the main trace is not identified"}}

    rec: dict = {"read_status": "ok", "buffer": {"read_status": "ok"}}

    def need(suffix: str) -> tuple[str, str | None]:
        text, why = read_text(base + suffix)
        return text, why

    def fail(msg: str) -> None:
        if rec["read_status"] == "ok":
            rec["read_status"] = msg

    times, why = need(".tk-times.txt")
    if why:
        fail(why)
    else:
        got_t, st = parse_times(times)
        if st != "ok":
            fail(st)
        rec.update(got_t)
    states = {}
    for side, suffix in (("enabled_before", ".tk-enable-before.txt"), ("enabled_after", ".tk-enable-after.txt")):
        text, why = need(suffix)
        if why:
            fail(why)
            continue
        got, st = parse_states(text)
        if st != "ok":
            fail(st)
        states[side] = got
    prof_text, why = need(".tk-profile.txt")
    prof: dict = {}
    if why:
        fail(why)
    else:
        prof, st = parse_profile(prof_text)
        if st != "ok":
            fail(st)
    probes: dict = {}
    for p in PROBES:
        e: dict = {}
        for side in ("enabled_before", "enabled_after"):
            if p in states.get(side, {}):
                e[side] = states[side][p]
        if p in prof:
            e["hits"], e["missed"] = prof[p]
        probes[p] = e
    rec["probes"] = probes
    stats_text, why = need(".tk-stats.txt")
    if why:
        rec["buffer"] = {"read_status": why}
    else:
        got, st = parse_stats(stats_text)
        rec["buffer"] = got if st == "ok" else {"read_status": st}
    rec_text, why = need(".tk.txt")
    if why:
        fail(why)
    else:
        parsed = parse_record(rec_text)
        if parsed["read_status"] != "ok":
            fail(parsed["read_status"])
        for k in ("calls", "updates", "unmatched"):
            rec[k] = parsed[k]
    ev["recorder"] = rec
    return ev


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("state")
    s.add_argument("--out", required=True)
    s.add_argument("--cmdline", default="/proc/cmdline")
    s.add_argument("--clocksource", default=CLOCKSOURCE)
    s.add_argument("--config-dir", default="/boot")
    lg = sub.add_parser("log")
    lg.add_argument("--journal", required=True)
    lg.add_argument("--from-t", type=int, required=True)
    lg.add_argument("--to-t", type=int, required=True)
    lg.add_argument("--out", required=True)
    tl = sub.add_parser("tool")
    tl.add_argument("--trace-cmd", required=True, help="the trace-cmd the session runs (a name on PATH, or a path)")
    tl.add_argument("--out", required=True)
    args = p.parse_args()
    if args.cmd == "state":
        out = state_read(args.cmdline, args.clocksource, args.config_dir)
    elif args.cmd == "tool":
        out = tool_identity(args.trace_cmd)
    else:
        text, why = read_text(args.journal)
        out = {"read_status": why} if why else scan_log(text, args.from_t, args.to_t)
    try:
        pathlib.Path(args.out).write_text(json.dumps(out) + "\n")
    except OSError as e:
        print(f"tk_record: cannot write {args.out}: {e}", file=sys.stderr)
        return 1
    return 0 if out.get("read_status") == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
