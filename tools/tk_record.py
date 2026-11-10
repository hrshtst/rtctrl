#!/usr/bin/env python3
"""The evidence of the timekeeping certificate (tools/timer_clock.py): the
state reads a timer session takes, the kernel's provenance, and the loader
that assembles the recorder's files into the certificate's evidence.

    tk_record.py state --out FILE [--cmdline FILE]     one state read, with the kernel's identity (no privilege)
    tk_record.py log --journal FILE --from-t N --to-t N --out FILE
                                                       the kernel log's scan over the interval

A state read is adjtimex(2) with modes 0, then with ADJ_OFFSET_SS_READ (for
the adjtime remainder), neither of which changes anything; the two monotonic
stamps bracket the reads. load() returns None when a capture has no record at
all. A record with a missing, unreadable or malformed part names that part's
read_status: the certificate treats it as UNSUPPORTED, and never as an empty
successful record.

The files of a capture <run>.csv, written by tools/trace_session.sh:
    .tk-before.json .tk-after.json   the state reads around the capture
    .tk-log.json                     the kernel log's scan
    .tk-times.txt                    armed_t and confirmed_t
    .tk-enable-before.txt .tk-enable-after.txt   the probes' enable states
    .tk-profile.txt .tk-stats.txt    the probes' hits and misses, the buffers' losses
    .tk.txt                          trace-cmd report -t of the recorder's instance
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import pathlib
import platform
import re
import sys
import threading
import time

INSTANCE = "rtctrl_tk"
PROBES = ("ntp_adjtimex", "ntp_clear", "update_entry", "update_return")
ADJ_OFFSET_SS_READ = 0xa001
CLOCKSOURCE = "/sys/devices/system/clocksource/clocksource0/current_clocksource"
SUFFIXES = (".tk-before.json", ".tk-after.json", ".tk-log.json", ".tk-times.txt", ".tk-enable-before.txt",
            ".tk-enable-after.txt", ".tk-profile.txt", ".tk-stats.txt", ".tk.txt")
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


def read_json(path: str) -> tuple[dict | None, str]:
    text, why = read_text(path)
    if why:
        return None, why
    try:
        v = json.loads(text)
    except ValueError as e:
        return None, f"{path}: not JSON ({e})"
    return v, "ok"


def parse_states(text: str) -> tuple[dict, str]:
    out = {}
    for ln in text.splitlines():
        f = ln.split()
        if len(f) != 2 or f[0] not in PROBES:
            return {}, f"an unreadable enable state: {ln!r}"
        out[f[0]] = f[1] == "1"
    return out, "ok"


def parse_profile(text: str) -> tuple[dict, str]:
    out = {}
    for ln in text.splitlines():
        f = ln.split()
        if len(f) != 3 or f[0] not in PROBES or not f[1].isdigit() or not f[2].isdigit():
            return {}, f"an unreadable profile row: {ln!r}"
        out[f[0]] = (int(f[1]), int(f[2]))
    return out, "ok"


def parse_stats(text: str) -> tuple[dict, str]:
    """The sums of the CPUs' overruns and drops. Every CPU the recorder listed must be here with all three counts."""
    keys = ("overrun", "commit overrun", "dropped events")
    blocks: dict[int, dict] = {}
    declared, cpu = None, None
    for ln in text.splitlines():
        m = re.fullmatch(r"\s*CPU:\s*(\d+)\s*", ln)
        if m:
            cpu = int(m[1])
            if cpu in blocks:
                return {}, f"CPU {cpu}'s stats appear twice"
            blocks[cpu] = {}
            continue
        m = re.fullmatch(r"\s*cpus:\s*(\d+)\s*", ln)
        if m:
            declared, cpu = int(m[1]), None
            continue
        m = re.fullmatch(r"\s*(overrun|commit overrun|dropped events):\s*(\S+)\s*", ln)
        if m and cpu is not None:
            if not m[2].isdigit():
                return {}, f"CPU {cpu}'s {m[1]} is {m[2]!r}"
            blocks[cpu][m[1]] = int(m[2])
    if declared is None or declared != len(blocks) or not blocks:
        return {}, f"the stats cover {len(blocks)} CPUs, and the recorder listed {declared}"
    for c, b in sorted(blocks.items()):
        if any(k not in b for k in keys):
            return {}, f"CPU {c}'s stats lack a count"
    return {"read_status": "ok", "overrun": sum(b["overrun"] + b["commit overrun"] for b in blocks.values()),
            "dropped": sum(b["dropped events"] for b in blocks.values())}, "ok"


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


def load(csv_path) -> dict | None:
    """The evidence of a capture, or None when it has no timekeeping record at all."""
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
        m_a, m_c = re.search(r"^armed_t (\d+)$", times, re.M), re.search(r"^confirmed_t (\d+)$", times, re.M)
        if m_a:
            rec["armed_t"] = int(m_a[1])
        if m_c:
            rec["confirmed_t"] = int(m_c[1])
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
    args = p.parse_args()
    if args.cmd == "state":
        out = state_read(args.cmdline, args.clocksource, args.config_dir)
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
