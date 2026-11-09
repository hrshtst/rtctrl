#!/usr/bin/env python3
"""Read-only, system-wide memory and writeback counters for
tools/trace_session.sh (docs/hardware/control-rate.md, recorder staging):
taken right before and after every run's probe, untraced runs included, as
context for the recorders' waits.

A snapshot reads, between two CLOCK_MONOTONIC stamps, the cumulative
/proc/vmstat counters pswpin and pswpout (pages swapped in and out),
pgmajfault (major faults), pgscan_direct (pages scanned by direct reclaim),
nr_dirtied and nr_written (pages dirtied and written), every allocstall_*
counter the kernel offers (direct-reclaim stalls per zone; this kernel has
allocstall_dma, _dma32, _normal, _movable and _device, no bare allocstall),
the gauges nr_dirty and nr_writeback (pages), and the /proc/meminfo gauges
MemAvailable, Dirty, Writeback, SwapFree and Shmem (kB). A counter the kernel
does not offer is marked unavailable, never read as zero. Their scope is the
whole system, not the run: a change says what the system did meanwhile, not
which resource a recorder waited on, and swap occupancy alone is no evidence
of memory pressure.

The delta prints each counter's change with its unit, the allocstall
counters both one by one and summed over those present in both snapshots
(named), and each gauge before and after.

usage: vm_counters.py snapshot --label before [--vmstat PATH] [--meminfo PATH]
       vm_counters.py delta BEFORE AFTER
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import time

COUNTERS = {"pswpin": "pages", "pswpout": "pages", "pgmajfault": "faults", "pgscan_direct": "pages",
            "nr_dirtied": "pages", "nr_written": "pages"}
GAUGES = {"nr_dirty": "pages", "nr_writeback": "pages"}
MEMINFO = ("MemAvailable", "Dirty", "Writeback", "SwapFree", "Shmem")
STALL = "allocstall_"
UNAVAILABLE = "unavailable"
HEAD = re.compile(r"^vm counters \((?P<label>[^)]*)\): monotonic_ns (?P<a>\d+) (?P<b>\d+)")
FIELD = re.compile(r"(\w+)=(\S+)")


def read_kv(path: str, sep: str | None) -> dict[str, str] | None:
    try:
        text = pathlib.Path(path).read_text()
    except OSError:
        return None
    out = {}
    for line in text.splitlines():
        if sep:
            k, _, v = line.partition(sep)
        else:
            k, _, v = line.partition(" ")
        parts = v.split()
        if k and parts:
            out[k.strip()] = parts[0]
    return out


def snapshot(vmstat: str, meminfo: str) -> dict:
    t0 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    vm = read_kv(vmstat, None)
    mi = read_kv(meminfo, ":")
    t1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    v: dict[str, str] = {}
    for k in (*COUNTERS, *GAUGES):
        v[k] = vm.get(k, UNAVAILABLE) if vm is not None else UNAVAILABLE
    for k in sorted(vm or {}):
        if k.startswith(STALL):
            v[k] = (vm or {})[k]
    m = {k: (mi.get(k, UNAVAILABLE) if mi is not None else UNAVAILABLE) for k in MEMINFO}
    return {"t": (t0, t1), "vmstat": v, "meminfo": m}


def format_snapshot(snap: dict, label: str, vmstat: str, meminfo: str) -> str:
    t0, t1 = snap["t"]
    return (f"vm counters ({label}): monotonic_ns {t0} {t1} (read between them); system-wide, read-only "
            f"({vmstat}, {meminfo})\n"
            "vmstat " + " ".join(f"{k}={v}" for k, v in snap["vmstat"].items()) + "\n"
            "meminfo " + " ".join(f"{k}={v}" for k, v in snap["meminfo"].items()) + " (kB)\n")


def parse(text: str) -> dict:
    snap: dict = {"t": None, "vmstat": {}, "meminfo": {}}
    for line in text.splitlines():
        h = HEAD.match(line)
        if h:
            snap["t"] = (int(h["a"]), int(h["b"]))
        elif line.startswith("vmstat "):
            snap["vmstat"] = dict(FIELD.findall(line))
        elif line.startswith("meminfo "):
            snap["meminfo"] = dict(FIELD.findall(line))
    if snap["t"] is None:
        raise ValueError("not a vm counter snapshot")
    return snap


def change(b: str | None, a: str | None) -> int | None:
    if b is None or a is None or not b.isdigit() or not a.isdigit():
        return None
    return int(a) - int(b)


def delta(before: dict, after: dict) -> str:
    mid = lambda s: (s["t"][0] + s["t"][1]) // 2  # noqa: E731
    span = (mid(after) - mid(before)) / 1e9
    bv, av = before["vmstat"], after["vmstat"]
    parts = []
    for k, unit in COUNTERS.items():
        d = change(bv.get(k), av.get(k))
        parts.append(f"{k} {UNAVAILABLE}" if d is None else f"{k} {d:+d} {unit}")
    stalls = sorted(k for k in set(bv) | set(av) if k.startswith(STALL))
    both = [k for k in stalls if change(bv.get(k), av.get(k)) is not None]
    for k in stalls:
        d = change(bv.get(k), av.get(k))
        parts.append(f"{k} {UNAVAILABLE}" if d is None else f"{k} {d:+d} stalls")
    if both:
        total = sum(change(bv[k], av[k]) or 0 for k in both)
        parts.append(f"allocstall (sum of {', '.join(both)}) {total:+d} stalls")
    else:
        parts.append(f"allocstall (sum) {UNAVAILABLE}: no allocstall_* counter in both snapshots")
    gauges = [f"{k} {bv.get(k, UNAVAILABLE)} -> {av.get(k, UNAVAILABLE)} {u}" for k, u in GAUGES.items()]
    gauges += [f"{k} {before['meminfo'].get(k, UNAVAILABLE)} -> {after['meminfo'].get(k, UNAVAILABLE)} kB"
               for k in MEMINFO]
    return (f"vm counter changes (after - before; {span:.3f} s between the snapshots' midpoints; system-wide, "
            "context, not a cause):\n  " + ", ".join(parts) + "\n  gauges: " + ", ".join(gauges) + "\n")


def main() -> int:
    p = argparse.ArgumentParser(description="Read-only, system-wide memory and writeback counters.")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("snapshot")
    s.add_argument("--label", required=True)
    s.add_argument("--vmstat", default="/proc/vmstat")
    s.add_argument("--meminfo", default="/proc/meminfo")
    d = sub.add_parser("delta")
    d.add_argument("before", type=pathlib.Path)
    d.add_argument("after", type=pathlib.Path)
    args = p.parse_args()
    if args.cmd == "snapshot":
        sys.stdout.write(format_snapshot(snapshot(args.vmstat, args.meminfo), args.label, args.vmstat, args.meminfo))
        return 0
    try:
        before, after = parse(args.before.read_text()), parse(args.after.read_text())
    except (OSError, ValueError) as e:
        print(f"vm_counters: {e}", file=sys.stderr)
        return 2
    sys.stdout.write(delta(before, after))
    return 0


if __name__ == "__main__":
    sys.exit(main())
