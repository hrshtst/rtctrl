#!/usr/bin/env python3
"""Whether the emulator stayed off the real-time CPU during a timer capture, for the loaded emulator validation's Option A
second run (docs/hardware/control-rate.md): launched with an affinity without that CPU, and checked so by the supervisor
(tools/option_a.sh), none of its threads may run there, be woken onto it or be migrated onto it. Read-only.

    emulator_residency.py <run>.csv --experiment FILE [--report FILE] [--rt-cpu N]

The emulator comes from the supervisor's experiment record: its emulator_start=, emulator_A= and emulator_B= lines, each
"TIME pid=P start=S comm=C tids=T,... allowed=L;..." (every thread's Cpus_allowed_list). All three must be there, name
the same pid and start time, and give every thread a well-formed list without the CPU; otherwise the record is
unreadable (an emulator check that failed is the supervisor's stop, not this tool's question). Its threads are the
recorded ones and every task one of them is seen to create in the trace (sched_process_fork from one of them).

Inside the probe's window (its first wake stamp to its last cycle-end stamp, as tools/unbound_work.py takes it), from the
main trace (trace-cmd report -t under tools/tk_record.py's launch, or a saved --report):

- runs: no switch onto the real-time CPU of an emulator thread (sched_switch to it there);
- placements: no wakeup of one onto that CPU (sched_wakeup ... CPU:N) and no migration of one onto it
  (sched_migrate_task ... dest_cpu=N);
- seen: at least one switch to an emulator thread in the window, on any CPU (else nothing establishes where it ran).

The evidence must be complete: every sched_switch, sched_wakeup, sched_migrate_task and sched_process_fork must parse,
every event line after the first must be an event line, the report must reach past the window's end, and it is read to
its end: a decoder that exits nonzero, at any point, makes the evidence unreadable. Descriptive: the CPUs the emulator's
threads were switched to, and their wake-to-run waits.

Exit: 0 every check passed; 1 a check failed; 2 the evidence could not be read.
"""

from __future__ import annotations

import argparse
import collections
import os
import pathlib
import re
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tk_cost  # noqa: E402
import tk_record  # noqa: E402
import trace_correlate  # noqa: E402

WAKEUP = re.compile(r"^(?P<comm>.*):(?P<pid>\d+) \[-?\d+\] CPU:(?P<cpu>\d+)$")
MIGRATE = re.compile(r"\bpid=(?P<pid>\d+) prio=-?\d+ orig_cpu=(?P<orig>\d+) dest_cpu=(?P<dest>\d+)$")
FORK = re.compile(r"\bpid=(?P<pid>\d+) child_comm=.* child_pid=(?P<child>\d+)$")
RECORD = re.compile(r"^(?P<t>\S+) pid=(?P<pid>\d+) start=(?P<start>\d+) comm=(?P<comm>\S+) tids=(?P<tids>[\d,]+) "
                    r"allowed=(?P<allowed>[\d,;-]+)$")
EVENTS = {"sched_switch", "sched_wakeup", "sched_migrate_task", "sched_process_fork"}


class Unreadable(Exception):
    pass


def cpus(spec: str) -> set:
    """A Cpus_allowed_list as a set; a malformed list (a descending range included) raises ValueError."""
    out: set = set()
    for part in spec.split(","):
        a, sep, b = part.partition("-")
        if not a.isdigit() or (sep and not b.isdigit()):
            raise ValueError(spec)
        lo, hi = int(a), (int(b) if sep else int(a))
        if hi < lo:
            raise ValueError(spec)
        out.update(range(lo, hi + 1))
    return out


def emulator(record: pathlib.Path, rt: int) -> dict:
    """The emulator's pid, start time and recorded threads from the experiment record; raises Unreadable."""
    try:
        lines = record.read_text().splitlines()
    except OSError as e:
        raise Unreadable(f"the experiment record {record}: {e}") from None
    seen: dict = {}
    for ln in lines:
        k, sep, v = ln.partition("=")
        if k not in ("emulator_start", "emulator_A", "emulator_B"):
            continue
        if k in seen:
            raise Unreadable(f"{record}: {k} twice")
        m = RECORD.match(v)
        if not m:
            raise Unreadable(f"{record}: {k} is no emulator state (a failed check?): {v!r}")
        tids, lists = m["tids"].split(","), m["allowed"].split(";")
        if len(tids) != len(lists):
            raise Unreadable(f"{record}: {k}: {len(tids)} threads, {len(lists)} affinity lists")
        try:
            sets = [cpus(x) for x in lists]
        except ValueError as e:
            raise Unreadable(f"{record}: {k}: a malformed affinity list {e}") from None
        if any(rt in s or not s for s in sets):
            raise Unreadable(f"{record}: {k}: a thread may run on CPU {rt} (or on no CPU): {m['allowed']}")
        seen[k] = (int(m["pid"]), m["start"], {int(t) for t in tids})
    missing = [k for k in ("emulator_start", "emulator_A", "emulator_B") if k not in seen]
    if missing:
        raise Unreadable(f"{record}: no {', '.join(missing)}")
    ids = {(pid, start) for pid, start, _ in seen.values()}
    if len(ids) != 1:
        raise Unreadable(f"{record}: the emulator's identity changes: {sorted(ids)}")
    pid, start = ids.pop()
    return dict(pid=pid, start=start, tids=set().union(*(t for _, _, t in seen.values())))


def analyse(lines, cap, emu: dict, rt: int, out) -> dict:
    """The checks above on report lines (read to their end); raises Unreadable on incomplete evidence."""
    lo, hi = cap.window
    tids = set(emu["tids"])
    last = None
    started = False
    unparsed = malformed = 0
    runs_rt: list = []      # (ts, tid) switches to an emulator thread on the real-time CPU in the window
    woken_rt: list = []     # (ts, tid) wakeups onto it
    moved_rt: list = []     # (ts, tid, orig) migrations onto it
    forks: list = []        # (ts, parent, child)
    ran = collections.Counter()
    waits: list = []
    woken: dict = {}        # tid -> wakeup ts, awaiting its switch-in
    for ln in lines:
        ln = ln.rstrip("\n")
        m = trace_correlate.LINE.match(ln)
        if not m:
            if started and ln.strip():
                unparsed += 1
            continue
        started = True
        ts = int(m["s"]) * 1_000_000_000 + int(m["ns"])
        last = ts
        ev, rest, cpu = m["ev"], m["rest"], int(m["cpu"])
        if ev not in EVENTS or ts > hi:
            continue
        inside = ts >= lo
        if ev == "sched_process_fork":
            f = FORK.search(rest)
            if not f:
                malformed += 1
                continue
            if int(f["pid"]) in tids:
                tids.add(int(f["child"]))
                forks.append((ts, int(f["pid"]), int(f["child"])))
        elif ev == "sched_switch":
            s = trace_correlate.SWITCH.match(rest)
            if not s:
                malformed += 1
                continue
            npid = int(s["npid"])
            if npid in tids and inside:
                ran[cpu] += 1
                if npid in woken:
                    waits.append(ts - woken.pop(npid))
                if cpu == rt:
                    runs_rt.append((ts, npid))
        elif ev == "sched_wakeup":
            w = WAKEUP.match(rest)
            if not w:
                malformed += 1
                continue
            if int(w["pid"]) in tids and inside:
                woken[int(w["pid"])] = ts
                if int(w["cpu"]) == rt:
                    woken_rt.append((ts, int(w["pid"])))
        else:
            g = MIGRATE.search(rest)
            if not g:
                malformed += 1
                continue
            if int(g["pid"]) in tids and inside and int(g["dest"]) == rt:
                moved_rt.append((ts, int(g["pid"]), int(g["orig"])))
    if last is None or last < hi:
        raise Unreadable(f"the report ends at {last} ns, before the window's end ({hi} ns)")
    if unparsed or malformed:
        raise Unreadable(f"{malformed} malformed required events, {unparsed} lines that are no events")
    seen = sum(ran.values())
    print(f"window: {lo} to {hi} ns ({(hi - lo) / 1e9:.6f} s); real-time CPU {rt}", file=out)
    print(f"  the emulator: pid {emu['pid']} (start {emu['start']}), recorded threads {','.join(map(str, sorted(emu['tids'])))}"
          + (f"; created in the trace: {', '.join(f'{c} by {p}' for _, p, c in forks)}" if forks else "; none created in the trace"), file=out)
    print(f"  switches to it in the window: {seen}" + (" (CPUs: " + ", ".join(f"{c} {n}" for c, n in sorted(ran.items())) + ")" if seen else ""), file=out)
    for what, xs in (("switches to it on CPU %d" % rt, runs_rt), ("wakeups of it onto CPU %d" % rt, woken_rt),
                     ("migrations of it onto CPU %d" % rt, moved_rt)):
        print(f"  {what}: {len(xs)}" + (" (first at " + ", ".join(f"{(x[0] - lo) / 1e6:+.3f} ms" for x in xs[:5]) + ")" if xs else ""), file=out)
    if waits:
        print(f"  wake to run (descriptive): {tk_cost.dist(sorted(waits))}", file=out)
    if not seen:
        print("  the emulator was not seen to run in the window: nothing establishes where it ran", file=out)
    ok = seen > 0 and not runs_rt and not woken_rt and not moved_rt
    print(f"emulator residency: {'PASS' if ok else 'FAIL'}", file=out)
    return dict(ok=ok, seen=seen, runs_rt=len(runs_rt), woken_rt=len(woken_rt), moved_rt=len(moved_rt), forks=len(forks),
                cpus=dict(ran), waits=waits)


def run_report(csv_path: pathlib.Path, cap, emu: dict, rt: int, out) -> dict:
    """analyse() over trace-cmd report -t under the launch, read to its end; the decoder's exit is always collected."""
    decoder = os.environ.get("EMULATOR_RESIDENCY_DECODER") if os.environ.get("EMULATOR_RESIDENCY_TEST") == "1" else None
    if decoder is None:
        tool = tk_record.tool_identity("trace-cmd")
        if tool.get("read_status") != "ok" or not tool.get("path"):
            raise Unreadable(f"trace-cmd is not identified ({tool.get('read_status')})")
        decoder = tool["path"]
    proc = subprocess.Popen([decoder, "report", "-t", "-i", f"{csv_path}.trace.dat"], stdout=subprocess.PIPE,
                            stderr=subprocess.DEVNULL, text=True, errors="replace", env=tk_record.launch_env())
    assert proc.stdout is not None
    try:
        r = analyse(proc.stdout, cap, emu, rt, out)
    finally:
        if proc.poll() is None:
            for _ in proc.stdout:  # drain to the end, so that its exit is its own
                pass
        proc.stdout.close()
        status = proc.wait()
    if status != 0:
        raise Unreadable(f"the decoder exited {status}")
    return r


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("csv", type=pathlib.Path, help="the timer capture's probe log (<run>.csv)")
    p.add_argument("--experiment", type=pathlib.Path, required=True, help="the supervisor's experiment.record")
    p.add_argument("--report", type=pathlib.Path, help="a saved trace-cmd report -t of the capture (default: run trace-cmd)")
    p.add_argument("--rt-cpu", type=int, help="the real-time CPU (default: the run record's, else 3)")
    a = p.parse_args()
    try:
        cap = tk_cost.Capture(a.csv)
    except (OSError, KeyError, ValueError, SystemExit) as e:
        print(f"emulator_residency: {a.csv}: unreadable ({e})", file=sys.stderr)
        return 2
    rt = a.rt_cpu if a.rt_cpu is not None else tk_cost.rt_cpu_of(a.csv)
    print(f"{a.csv.name}: tools/emulator_residency.py")
    try:
        emu = emulator(a.experiment, rt)
        if a.report:
            with open(a.report, errors="replace") as f:
                r = analyse(f, cap, emu, rt, sys.stdout)
        else:
            r = run_report(a.csv, cap, emu, rt, sys.stdout)
    except (OSError, Unreadable) as e:
        print(f"emulator_residency: UNREADABLE: {e}", file=sys.stderr)
        return 2
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
