#!/usr/bin/env python3
"""Regressions for tools/timer_exerciser.py: a 0.4 s run, unpinned, writes
its header with its thread ID and, per round, the actions its kind
prescribes (an arm; then a cancellation of the same expiry, or a re-arm
earlier or later), each bracketed by its monotonic times; every timer that
was not cancelled is seen expiring at or after its expiry; the same seed
gives the same rounds; SIGTERM ends a run with status 0. No timing bound
(an expiry may already have passed when a preempted call ends): the test
suite runs in parallel."""

from __future__ import annotations

import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

TOOL = pathlib.Path(__file__).resolve().parent / "timer_exerciser.py"


def fail(msg: str) -> None:
    print(f"FAILED: {msg}")
    sys.exit(1)


def run(out: pathlib.Path, seed: int, duration: float = 0.4) -> list[list[str]]:
    p = subprocess.run([sys.executable, str(TOOL), "--output", str(out), "--duration", str(duration),
                        "--seed", str(seed)], capture_output=True, text=True)
    if p.returncode != 0:
        fail(f"exit {p.returncode}: {p.stderr}")
    lines = out.read_text().splitlines()
    if not lines[0].startswith("# timer_exerciser tid=") or "clock=CLOCK_MONOTONIC" not in lines[0]:
        fail(f"header {lines[0]}")
    if lines[1] != "round,kind,action,mono_before_ns,mono_after_ns,expiry_ns":
        fail(f"column header {lines[1]}")
    return [line.split(",") for line in lines[2:]]


def main() -> int:
    if not hasattr(os, "timerfd_create"):
        print("timer exerciser regressions OK (not run: Python without os.timerfd_create)")
        return 0
    with tempfile.TemporaryDirectory() as d:
        rows = run(pathlib.Path(d, "a.csv"), seed=3)
        rounds: dict[int, list] = {}
        for r in rows:
            rounds.setdefault(int(r[0]), []).append(r)
        if len(rounds) < 20:
            fail(f"only {len(rounds)} rounds")
        for n, rs in rounds.items():
            kind = rs[0][1]
            acts = [r[2] for r in rs]
            want = {"expire": ["arm", "expired"], "cancel": ["arm", "cancel"],
                    "earlier": ["arm", "rearm", "expired"], "later": ["arm", "rearm", "expired"]}[kind]
            if acts != want and not (n == max(rounds) and want[:len(acts)] == acts):
                fail(f"round {n} ({kind}): {acts}")
            for r in rs:
                before, after, expiry = int(r[3]), int(r[4]), int(r[5])
                if after < before:
                    fail(f"round {n}: the bracket goes backwards")
                if r[2] == "expired" and before < expiry:
                    fail(f"round {n}: seen expired {expiry - before} ns before its expiry")
            arm = int(rs[0][5])
            if kind == "cancel" and len(rs) > 1 and int(rs[1][5]) != arm:
                fail(f"round {n}: the cancellation names another expiry")
            if kind == "earlier" and len(rs) > 1 and not int(rs[1][5]) < arm:
                fail(f"round {n}: the re-arm is not earlier")
            if kind == "later" and len(rs) > 1 and not int(rs[1][5]) > arm:
                fail(f"round {n}: the re-arm is not later")
        again = run(pathlib.Path(d, "b.csv"), seed=3, duration=0.2)
        kinds_a = [rounds[k][0][1] for k in sorted(rounds)]
        kinds_b = []
        for r in again:
            if int(r[0]) > len(kinds_b):
                kinds_b.append(r[1])
        if kinds_b != kinds_a[:len(kinds_b)]:
            fail("the same seed gave other rounds")
        p = subprocess.Popen([sys.executable, str(TOOL), "--output", str(pathlib.Path(d, "c.csv")),
                              "--duration", "60"])
        time.sleep(0.3)
        p.send_signal(signal.SIGTERM)
        if p.wait(timeout=10) != 0:
            fail("SIGTERM did not end the run with status 0")
    print("timer exerciser regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
