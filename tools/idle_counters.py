"""Read-only snapshots of CPU idle-state counters for tools/trace_session.sh
(docs/hardware/control-rate.md): taken before and after every run, untraced
runs included, for the real-time CPU and its SMT siblings.

A snapshot reads, for each CPU and each of its cpuidle states, the state's
identity (name, description, exit latency, target residency), its disable
flag and its counters (usage, time; above, below and rejected where the
kernel offers them), bracketed by two CLOCK_MONOTONIC reads. Nothing is
written. A CPU without a cpuidle directory, and a counter the kernel does not
offer, is marked unavailable, never read as zero. The delta of two snapshots
is taken per state only when the state's identity is unchanged.

usage: idle_counters.py snapshot --cpus 2,3 --label before [--root DIR]
       idle_counters.py delta BEFORE AFTER
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys
import time

ROOT = "/sys/devices/system/cpu"
IDENTITY = ("name", "desc", "latency", "residency")
COUNTERS = ("usage", "time", "above", "below", "rejected")
OPTIONAL = ("above", "below", "rejected")
UNAVAILABLE = "unavailable"
HEAD = re.compile(r"^idle counters \((?P<label>[^)]*)\): monotonic_ns (?P<a>\d+) (?P<b>\d+)")
LINE = re.compile(r"^cpu(?P<cpu>\d+) (?P<rest>.*)$")
FIELD = re.compile(r'(\w+)=("(?:[^"\\]|\\.)*"|\S+)')


def read(path: pathlib.Path) -> str | None:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def snapshot(root: pathlib.Path, cpus: list[int]) -> dict:
    """The CPUs' idle states, read between two monotonic stamps."""
    t0 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    out: dict[int, dict] = {}
    for cpu in cpus:
        d = root / f"cpu{cpu}" / "cpuidle"
        if not d.is_dir():
            out[cpu] = {"unavailable": "no cpuidle directory"}
            continue
        states: dict[int, dict] = {}
        for s in sorted(d.glob("state[0-9]*"), key=lambda p: int(p.name[5:])):
            st: dict[str, str] = {}
            for key in IDENTITY + ("disable",) + COUNTERS:
                v = read(s / key)
                st[key] = UNAVAILABLE if v is None else v
            states[int(s.name[5:])] = st
        out[cpu] = {"states": states} if states else {"unavailable": "no idle states"}
    t1 = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    return {"t": (t0, t1), "cpus": out}


def value(v: str) -> str:
    return json.dumps(v) if (not v or any(c.isspace() for c in v) or '"' in v) else v


def format_snapshot(snap: dict, label: str, root: str) -> str:
    t0, t1 = snap["t"]
    lines = [f"idle counters ({label}): monotonic_ns {t0} {t1} (read between them), root {root}"]
    for cpu, c in snap["cpus"].items():
        if "unavailable" in c:
            lines.append(f"cpu{cpu} unavailable ({c['unavailable']})")
            continue
        for i, st in c["states"].items():
            fields = " ".join(f"{k}={value(st[k])}" for k in IDENTITY + ("disable",) + COUNTERS)
            lines.append(f"cpu{cpu} state{i} {fields}")
    return "\n".join(lines) + "\n"


def parse(text: str) -> dict:
    """A formatted snapshot back to its values."""
    snap: dict = {"t": None, "cpus": {}}
    for line in text.splitlines():
        h = HEAD.match(line)
        if h:
            snap["t"] = (int(h["a"]), int(h["b"]))
            snap["label"] = h["label"]
            continue
        m = LINE.match(line)
        if not m:
            continue
        cpu, rest = int(m["cpu"]), m["rest"]
        if rest.startswith("unavailable"):
            snap["cpus"][cpu] = {"unavailable": rest[len("unavailable"):].strip(" ()")}
            continue
        s = re.match(r"state(\d+) (.*)$", rest)
        if not s:
            continue
        st = {k: (json.loads(v) if v.startswith('"') else v) for k, v in FIELD.findall(s[2])}
        snap["cpus"].setdefault(cpu, {"states": {}})["states"][int(s[1])] = st
    if snap["t"] is None:
        raise ValueError("not an idle counter snapshot")
    return snap


def delta(before: dict, after: dict) -> str:
    """After minus before, per state, where the state is unchanged."""
    mid = lambda s: (s["t"][0] + s["t"][1]) // 2  # noqa: E731
    span = (mid(after) - mid(before)) / 1e9
    lines = [f"idle counter deltas (after - before; {span:.3f} s between the snapshots' midpoints):"]
    for cpu in sorted(set(before["cpus"]) | set(after["cpus"])):
        b, a = before["cpus"].get(cpu, {}), after["cpus"].get(cpu, {})
        if "states" not in b or "states" not in a:
            why = b.get("unavailable") or a.get("unavailable") or "missing from a snapshot"
            lines.append(f"cpu{cpu}: no delta ({why})")
            continue
        for i in sorted(set(b["states"]) | set(a["states"])):
            sb, sa = b["states"].get(i), a["states"].get(i)
            if sb is None or sa is None:
                lines.append(f"cpu{cpu} state{i}: no delta (present in one snapshot only)")
                continue
            ident_b = tuple(sb.get(k) for k in IDENTITY)
            ident_a = tuple(sa.get(k) for k in IDENTITY)
            if ident_b != ident_a:
                lines.append(f"cpu{cpu} state{i}: no delta (identity changed: {ident_b} -> {ident_a})")
                continue
            parts = []
            for k in COUNTERS:
                vb, va = sb.get(k, UNAVAILABLE), sa.get(k, UNAVAILABLE)
                if vb.isdigit() and va.isdigit():
                    parts.append(f"{k} {int(va) - int(vb):+d}")
                else:
                    parts.append(f"{k} {UNAVAILABLE}")
            note = ""
            if sb.get("disable") != sa.get("disable"):
                note = f" (disable changed: {sb.get('disable')} -> {sa.get('disable')})"
            lines.append(f"cpu{cpu} state{i} {sa.get('name')}: " + ", ".join(parts) + note)
    return "\n".join(lines) + "\n"


def main() -> int:
    p = argparse.ArgumentParser(description="Read-only snapshots of CPU idle-state counters.")
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("snapshot")
    s.add_argument("--cpus", required=True, help="comma-separated CPU numbers")
    s.add_argument("--label", required=True)
    s.add_argument("--root", default=ROOT)
    d = sub.add_parser("delta")
    d.add_argument("before", type=pathlib.Path)
    d.add_argument("after", type=pathlib.Path)
    args = p.parse_args()
    if args.cmd == "snapshot":
        try:
            cpus = [int(c) for c in args.cpus.split(",") if c != ""]
        except ValueError:
            print(f"idle_counters: --cpus {args.cpus!r} is not a CPU list", file=sys.stderr)
            return 2
        if not cpus:
            print("idle_counters: --cpus is empty", file=sys.stderr)
            return 2
        sys.stdout.write(format_snapshot(snapshot(pathlib.Path(args.root), cpus), args.label, args.root))
        return 0
    try:
        before, after = parse(args.before.read_text()), parse(args.after.read_text())
    except (OSError, ValueError) as e:
        print(f"idle_counters: {e}", file=sys.stderr)
        return 2
    sys.stdout.write(delta(before, after))
    return 0


if __name__ == "__main__":
    sys.exit(main())
