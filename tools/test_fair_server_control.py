#!/usr/bin/env python3
"""Regressions for fair_server_control.py, run under ctest.

No real-time priority is needed or taken: the loop runs in the tool's test
modes (FAIR_SERVER_CONTROL_TEST). Pinned: the phases in order with their
boundaries, the loop's start and end within the positive phase, the normal
tasks' progress samples, the supervisor's kill of a loop that ignores its
deadline (and the failing status), the refusals, and a loop refused
SCHED_FIFO without a real-time limit.
"""

import os
import pathlib
import re
import subprocess
import sys
import tempfile


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def run(out: pathlib.Path, *args: str, test: str | None = None) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env.pop("FAIR_SERVER_CONTROL_TEST", None)
    if test:
        env["FAIR_SERVER_CONTROL_TEST"] = test
    return subprocess.run([sys.executable, "tools/fair_server_control.py", "run", "--output", str(out), *args],
                          capture_output=True, text=True, env=env, timeout=60)


def main() -> int:
    cpus = sorted(os.sched_getaffinity(0))
    if len(cpus) < 2:
        print("fair server control regressions OK (not run: one CPU)")
        return 0
    cpu, sup = str(cpus[-1]), str(cpus[0])
    with tempfile.TemporaryDirectory() as t:
        out = pathlib.Path(t) / "c.txt"
        r = run(out, "--cpu", cpu, "--supervisor-cpu", sup, "--phase-s", "0.3", "--settle-s", "0.1", test="no-rt")
        text = out.read_text()
        expect(r.returncode == 0, f"a test-mode run: {r.stderr}\n{text}")
        phases = re.findall(r"^phase (\S+) (\d+) (\d+)$", text, re.M)
        expect([p[0] for p in phases] == ["positive", "settle-1", "negative-normal", "settle-2", "negative-idle"],
               f"the phases in order: {phases}")
        ends = [(int(a), int(b)) for _, a, b in phases]
        expect(all(a < b for a, b in ends) and all(ends[i][1] <= ends[i + 1][0] for i in range(len(ends) - 1)),
               f"the phases follow each other: {ends}")
        f = re.search(r"^fifo start=(\d+) end=(\d+) exit=0 killed=no pid=\d+$", text, re.M)
        expect(f is not None and ends[0][0] <= int(f[1]) < int(f[2]) <= ends[0][1],
               f"the loop ran inside the positive phase: {text}")
        for phase in ("positive", "negative-normal"):
            m = re.search(rf"^samples {phase} (.*)$", text, re.M)
            expect(m is not None and len(m[1].split()) > 10, f"the {phase} normal task's progress samples")
        expect("# fifo: test mode no-rt, no SCHED_FIFO" in text, "the test mode is recorded")

        # A loop that ignores its deadline is killed by the supervisor.
        r = run(out, "--cpu", cpu, "--supervisor-cpu", sup, "--phase-s", "0.2", "--settle-s", "0.05",
                "--margin-s", "0.2", test="ignore-deadline")
        text = out.read_text()
        expect(r.returncode == 1 and re.search(r"^fifo start=\d+ end=none exit=-9 killed=yes ", text, re.M) is not None,
               f"the supervisor's kill: {r.returncode}\n{text}")

        # Refusals.
        r = run(out, "--cpu", cpu, "--supervisor-cpu", cpu, test="no-rt")
        expect(r.returncode == 2 and "another CPU" in r.stderr, "the supervisor on the loop's CPU is refused")
        for bad in (["--fifo-priority", "0"], ["--phase-s", "11"], ["--margin-s", "0"]):
            r = run(out, "--cpu", cpu, "--supervisor-cpu", sup, *bad)
            expect(r.returncode == 2, f"{bad} is refused")

        # Without a real-time limit, SCHED_FIFO is refused: exit 3, a failing run.
        lim = subprocess.run(["sh", "-c", "ulimit -r"], capture_output=True, text=True).stdout.strip()
        if lim == "0":
            r = run(out, "--cpu", cpu, "--supervisor-cpu", sup, "--phase-s", "0.1", "--settle-s", "0.05")
            text = out.read_text()
            expect(r.returncode == 1 and "fifo: SCHED_FIFO 10 refused" in text and "exit=3" in text,
                   f"a refused SCHED_FIFO: {text}")
    print("fair server control regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
