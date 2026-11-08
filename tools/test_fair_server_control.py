#!/usr/bin/env python3
"""Regressions for fair_server_control.py, run under ctest.

No real-time priority is needed or taken: the loop runs in the tool's test
modes (FAIR_SERVER_CONTROL_TEST). Pinned: the phases in order with their
boundaries, the loop's start and end within the positive phase, the normal
tasks' progress samples, the supervisor's kill of a loop that ignores its
deadline (and the failing status), the refusals, and a loop refused
SCHED_FIFO without a real-time limit. And the supervisor's own failure
modes (reviewer, 2026-11-08): with 4 kB pipes and enough progress samples to
fill them, the run completes (the output drained while each child is waited
for); a SIGTERM during the positive phase kills and reaps every child, marks
the output incomplete and exits 130; and a normal task that never says it is
ready ends the run within the readiness bound (status 1, marked incomplete,
no child left).
"""

import os
import pathlib
import re
import signal
import subprocess
import sys
import tempfile
import time
import uuid

# A run whose children have 4 kB pipes (the reviewer's reproducer's setting).
SMALL_PIPES = r"""
import os, subprocess, sys
sys.path.insert(0, "tools")
import fair_server_control as m
m.child = lambda args: subprocess.Popen([sys.executable, m.__file__, *args], stdout=subprocess.PIPE, text=True,
                                        pipesize=4096)
cpu, sup, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
with open(out, "w") as f:
    sys.exit(m.run(cpu, sup, 10, 800_000_000, 50_000_000, 100_000_000, f))
"""


def leftovers(token: str) -> list[int]:
    """The processes still carrying the run's token in their environment."""
    found = []
    for d in pathlib.Path("/proc").iterdir():
        if d.name.isdigit() and int(d.name) != os.getpid():
            try:
                if token.encode() in (d / "environ").read_bytes():
                    found.append(int(d.name))
            except OSError:
                pass
    return found


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
        for phase in ("settle-1", "settle-2"):
            c = re.search(rf"^check {phase} start=(\d+) end=(\d+) exit=0$", text, re.M)
            span = [(int(a), int(b)) for n, a, b in phases if n == phase][0]
            expect(c is not None and span[0] <= int(c[1]) <= int(c[2]) <= span[1],
                   f"a state check at the end of {phase}, inside it: {text[:300]}")

        # A loop that ignores its deadline is killed by the supervisor.
        r = run(out, "--cpu", cpu, "--supervisor-cpu", sup, "--phase-s", "0.2", "--settle-s", "0.05",
                "--margin-s", "0.2", test="ignore-deadline")
        text = out.read_text()
        expect(r.returncode == 1 and re.search(r"^fifo start=\d+ end=none exit=-9 killed=yes ", text, re.M) is not None,
               f"the supervisor's kill: {r.returncode}\n{text}")

        # Full pipes: the run completes, its samples well past 4 kB per phase.
        env = dict(os.environ, FAIR_SERVER_CONTROL_TEST="no-rt")
        p = subprocess.Popen([sys.executable, "-c", SMALL_PIPES, cpu, sup, str(out)], env=env,
                             start_new_session=True)
        try:
            code = p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()
            expect(False, "a run with full pipes hangs")
        text = out.read_text()
        m = re.search(r"^normal negative-normal pid=\d+ exit=0 samples=(\d+)$", text, re.M)
        expect(code == 0 and m is not None and int(m[1]) * 16 > 4096 and "phase negative-idle" in text,
               f"a run with full pipes completes: {code}\n{text[:400]}")

        # A SIGTERM during the positive phase: every child killed and reaped.
        token = uuid.uuid4().hex
        env = dict(os.environ, FAIR_SERVER_CONTROL_TEST="no-rt", RTCTRL_TEST_TOKEN=token)
        p = subprocess.Popen([sys.executable, "tools/fair_server_control.py", "run", "--output", str(out),
                              "--cpu", cpu, "--supervisor-cpu", sup, "--phase-s", "5"], env=env)
        time.sleep(1.0)
        expect(len(leftovers(token)) >= 3, "the supervisor and its two children run")
        t0 = time.monotonic()
        p.send_signal(signal.SIGTERM)
        code = p.wait(timeout=10)
        text = out.read_text()
        expect(code == 130 and time.monotonic() - t0 < 5 and "# interrupted by signal 15" in text,
               f"a SIGTERM ends the run at once, marked incomplete: {code}\n{text}")
        time.sleep(0.2)
        expect(leftovers(token) == [], f"no child outlives the supervisor: {leftovers(token)}")

        # A normal task that never says it is ready: the readiness wait is bounded.
        token = uuid.uuid4().hex
        silent = SMALL_PIPES.replace(
            'm.child = lambda args: subprocess.Popen([sys.executable, m.__file__, *args], stdout=subprocess.PIPE, text=True,\n'
            '                                        pipesize=4096)',
            'm.child = lambda args: subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],\n'
            '                                        stdout=subprocess.PIPE, text=True)')
        expect(silent != SMALL_PIPES, "the silent child's script")
        t0 = time.monotonic()
        p = subprocess.Popen([sys.executable, "-c", silent, cpu, sup, str(out)],
                             env=dict(os.environ, RTCTRL_TEST_TOKEN=token), start_new_session=True)
        try:
            code = p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()
            expect(False, "the readiness wait is unbounded")
        text = out.read_text()
        expect(code == 1 and time.monotonic() - t0 < 6 and "did not say it was ready within 2 s" in text,
               f"a silent normal task ends the run: {code}\n{text}")
        time.sleep(0.2)
        expect(leftovers(token) == [], f"no child outlives a silent start: {leftovers(token)}")

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
