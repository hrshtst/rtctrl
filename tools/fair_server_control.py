"""The fair server's positive and negative controls for the timer mode's
rehearsal (tools/trace_session.sh --fs-controls; docs/hardware/control-rate.md,
Stage 1). Run over the emulator only, never during a run on the arm.

On one spare CPU, in order:
  positive        a normal task that records its progress, and a SCHED_FIFO
                  busy loop beside it, both pinned to the CPU: the fair
                  server should give the normal task some time;
  settle-1        nothing of the tool's on the CPU;
  negative-normal the normal task alone;
  settle-2        nothing;
  negative-idle   nothing (the idle CPU).
The FIFO loop stops itself at its deadline (CLOCK_MONOTONIC). This process is
its supervisor, pinned to another CPU: it kills the loop with SIGKILL at the
deadline plus a margin if the loop still runs, and says so. A normal task
stops at its own deadline. Every phase's boundaries, the loop's start and
end, and each normal task's progress samples (CLOCK_MONOTONIC, the trace's
clock, about once per millisecond of its running) are written out; the
trace's analysis (tools/fair_server.py controls) judges them.

usage: fair_server_control.py run --cpu N --supervisor-cpu M --output FILE
           [--fifo-priority P] [--phase-s S] [--settle-s S] [--margin-s S]

For tests only, FAIR_SERVER_CONTROL_TEST=no-rt runs the loop without
SCHED_FIFO (honouring its deadline) and =ignore-deadline without SCHED_FIFO
and past its deadline (the supervisor's kill). The analysis checks in the
trace that the loop ran at a real-time priority, so neither passes a
control.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time

SAMPLE_NS = 1_000_000


def now() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


def fifo(cpu: int, priority: int, deadline: int) -> int:
    """The busy loop: pinned first, then SCHED_FIFO, until its deadline."""
    os.sched_setaffinity(0, {cpu})
    test = os.environ.get("FAIR_SERVER_CONTROL_TEST", "")
    if test in ("no-rt", "ignore-deadline"):
        print(f"fifo: test mode {test}, no SCHED_FIFO", flush=True)
    else:
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
        except PermissionError as e:
            print(f"fifo: SCHED_FIFO {priority} refused: {e}", flush=True)
            return 3
    print(f"fifo start {now()} pid {os.getpid()}", flush=True)
    if test == "ignore-deadline":
        while True:
            pass
    while now() < deadline:
        pass
    print(f"fifo end {now()}", flush=True)
    return 0


def normal(cpu: int, deadline: int) -> int:
    """The normal task: pinned, it says it is ready, then counts and samples
    the clock about once per millisecond of its running, until its deadline."""
    os.sched_setaffinity(0, {cpu})
    print(f"ready {os.getpid()}", flush=True)
    samples = []
    last = now()
    n = 0
    while last < deadline:
        n += 1
        if n % 256 == 0:
            t = now()
            if t - last >= SAMPLE_NS:
                samples.append(t)
                last = t
            elif t >= deadline:
                break
    print("samples " + " ".join(str(t) for t in samples), flush=True)
    return 0


def child(args: list[str]) -> subprocess.Popen:
    return subprocess.Popen([sys.executable, __file__, *args], stdout=subprocess.PIPE, text=True)


def run(cpu: int, sup: int, prio: int, phase_ns: int, settle_ns: int, margin_ns: int, out) -> int:
    if cpu == sup:
        print("fair_server_control: the supervisor needs another CPU", file=sys.stderr)
        return 2
    os.sched_setaffinity(0, {sup})
    print(f"# fair_server_control cpu={cpu} supervisor_cpu={sup} fifo_priority={prio} "
          f"phase_ns={phase_ns} settle_ns={settle_ns} margin_ns={margin_ns}", file=out)
    status = 0

    def normal_task(deadline: int) -> subprocess.Popen:
        p = child(["_normal", str(cpu), str(deadline)])
        assert p.stdout is not None
        line = p.stdout.readline().split()
        if not line or line[0] != "ready":
            raise RuntimeError("the normal task did not start")
        return p

    def finish_normal(p: subprocess.Popen, phase: str) -> None:
        nonlocal status
        assert p.stdout is not None
        text = p.stdout.read()
        code = p.wait()
        samples = next((ln.split()[1:] for ln in text.splitlines() if ln.startswith("samples")), [])
        print(f"normal {phase} pid={p.pid} exit={code} samples={len(samples)}", file=out)
        print(f"samples {phase} " + " ".join(samples), file=out)
        if code != 0:
            status = 1

    # positive
    t0 = now()
    end = t0 + phase_ns
    nt = normal_task(end + margin_ns)
    ft = child(["_fifo", str(cpu), str(prio), str(end)])
    assert ft.stdout is not None
    killed = "no"
    try:
        code = ft.wait(timeout=(end + margin_ns - now()) / 1e9)
    except subprocess.TimeoutExpired:
        ft.send_signal(signal.SIGKILL)
        code = ft.wait()
        killed = "yes"
        status = 1
    ftext = ft.stdout.read()
    fstart = next((ln.split()[2] for ln in ftext.splitlines() if ln.startswith("fifo start")), "none")
    fend = next((ln.split()[2] for ln in ftext.splitlines() if ln.startswith("fifo end")), "none")
    if code != 0:
        status = 1
    finish_normal(nt, "positive")  # it runs on past the loop by the margin
    t1 = now()
    print(f"phase positive {t0} {t1}", file=out)
    print(f"fifo start={fstart} end={fend} exit={code} killed={killed} pid={ft.pid}", file=out)
    for ln in ftext.splitlines():
        if ln.startswith("fifo:"):
            print(f"# {ln}", file=out)

    # the rest, in order
    for phase, task in (("settle-1", False), ("negative-normal", True), ("settle-2", False),
                        ("negative-idle", False)):
        dur = settle_ns if phase.startswith("settle") else phase_ns
        t0 = now()
        if task:
            nt = normal_task(t0 + dur)
            nt.wait()
            t1 = now()
            finish_normal(nt, phase)
        else:
            time.sleep(dur / 1e9)
            t1 = now()
        print(f"phase {phase} {t0} {t1}", file=out)
    return status


def main() -> int:
    if len(sys.argv) > 1 and sys.argv[1] == "_fifo":
        return fifo(int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
    if len(sys.argv) > 1 and sys.argv[1] == "_normal":
        return normal(int(sys.argv[2]), int(sys.argv[3]))
    p = argparse.ArgumentParser(description="The fair server's positive and negative controls.")
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--cpu", type=int, required=True)
    r.add_argument("--supervisor-cpu", type=int, required=True)
    r.add_argument("--output", required=True)
    r.add_argument("--fifo-priority", type=int, default=10)
    r.add_argument("--phase-s", type=float, default=3.0)
    r.add_argument("--settle-s", type=float, default=2.0)
    r.add_argument("--margin-s", type=float, default=0.5)
    a = p.parse_args()
    if not 1 <= a.fifo_priority <= 99 or a.phase_s <= 0 or a.phase_s > 10 or a.settle_s < 0 or a.margin_s <= 0:
        print("fair_server_control: a FIFO priority of 1 to 99, a phase of up to 10 s, a margin above 0",
              file=sys.stderr)
        return 2
    with open(a.output, "w") as out:
        try:
            return run(a.cpu, a.supervisor_cpu, a.fifo_priority, int(a.phase_s * 1e9), int(a.settle_s * 1e9),
                       int(a.margin_s * 1e9), out)
        except (OSError, RuntimeError) as e:
            print(f"fair_server_control: {e}", file=sys.stderr)
            return 1


if __name__ == "__main__":
    sys.exit(main())
