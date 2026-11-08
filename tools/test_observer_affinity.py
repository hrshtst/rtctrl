#!/usr/bin/env python3
"""Regressions for observer_affinity.py, run under ctest, on real processes
of the test's own (no privilege needed: they are the invoking user's).

A stand-in observer (a process with two threads and a child) is confined
off one CPU and restored. Pinned: the write-ahead record (every thread's
identity, start time and mask before any change); every thread confined to
its mask less the excluded CPUs; a thread whose mask holds only excluded
CPUs left and reported (status 1); the refusals (an existing record, a
missing process, a process in an exempt tree); restore putting back the
recorded masks, leaving a mask changed during the session and a reused ID;
new threads and processes given their process's single original mask, and
left where the process had none; a repeated restore; and the check of a
run's task snapshot (observer threads and the probe's helpers, not its
worker, kept off the excluded CPU).
"""

import json
import os
import pathlib
import subprocess
import sys
import tempfile
import time

CHILD = r"""
import os, subprocess, sys, threading, time
def idle():
    while True:
        time.sleep(1)
threading.Thread(target=idle, daemon=True).start()
kid = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
print(os.getpid(), kid.pid, flush=True)
for line in sys.stdin:
    w = line.split()
    if w and w[0] == "thread":
        threading.Thread(target=idle, daemon=True).start()
        time.sleep(0.1)
        print("ok", flush=True)
    elif w and w[0] == "child":
        k = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(600)"])
        time.sleep(0.2)
        print(k.pid, flush=True)
"""


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def tool(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "tools/observer_affinity.py", *args], capture_output=True, text=True)


def tids(pid: int) -> list[int]:
    return sorted(int(t) for t in os.listdir(f"/proc/{pid}/task"))


def masks(pid: int) -> dict[int, set[int]]:
    return {t: os.sched_getaffinity(t) for t in tids(pid)}


def start(child: subprocess.Popen) -> tuple[int, int]:
    assert child.stdout is not None
    a, b = child.stdout.readline().split()
    return int(a), int(b)


def say(child: subprocess.Popen, what: str) -> str:
    assert child.stdin is not None and child.stdout is not None
    child.stdin.write(what + "\n")
    child.stdin.flush()
    return child.stdout.readline().strip()


def main() -> int:
    mine = sorted(os.sched_getaffinity(0))
    if len(mine) < 3:
        print("observer affinity regressions OK (not run: fewer than three CPUs)")
        return 0
    ex = mine[-1]
    allowed = set(mine) - {ex}
    procs: list[subprocess.Popen] = []
    kids: list[int] = []  # the observers' children, killed at the end
    try:
        with tempfile.TemporaryDirectory() as t:
            root = pathlib.Path(t)
            obs = subprocess.Popen([sys.executable, "-c", CHILD], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                   text=True)
            procs.append(obs)
            pid, kid = start(obs)
            kids.append(kid)
            time.sleep(0.2)
            before = {**masks(pid), **masks(kid)}
            expect(all(m == set(mine) for m in before.values()), "the observer starts unconfined")
            # One thread pinned to the excluded CPU alone, another changed later.
            ts = tids(pid)
            os.sched_setaffinity(ts[1], {ex})
            rec = root / "r.jsonl"
            r = tool("confine", "--pids", str(pid), "--exclude-cpus", str(ex), "--record", str(rec))
            expect(r.returncode == 1 and "1 left (only excluded CPUs)" in r.stdout,
                   f"a thread on the excluded CPU only is left: {r.stdout} {r.stderr}")
            lines = [json.loads(x) for x in rec.read_text().splitlines()]
            kinds = [x["kind"] for x in lines]
            expect(kinds[0] == "header" and kinds.index("state") > max(i for i, k in enumerate(kinds) if k == "thread")
                   and kinds[-1] == "state" and lines[-1]["state"] == "confined",
                   f"the record: header, every thread, then the changes: {kinds}")
            th = [x for x in lines if x["kind"] == "thread"]
            expect({x["tid"] for x in th} == set(tids(pid)) | set(tids(kid)) and all(x["start"] for x in th),
                   "every thread of the tree is recorded with its start time")
            now = {**masks(pid), **masks(kid)}
            expect(all(m == allowed for t_, m in now.items() if t_ != ts[1]) and now[ts[1]] == {ex},
                   f"the threads confined: {now}")
            # Refusals.
            expect(tool("confine", "--pids", str(pid), "--exclude-cpus", str(ex), "--record", str(rec)).returncode == 2,
                   "an existing record is refused")
            expect(tool("confine", "--pids", "999999999", "--exclude-cpus", str(ex),
                        "--record", str(root / "x.jsonl")).returncode == 2, "a missing process is refused")
            r = tool("confine", "--pids", str(kid), "--exclude-cpus", str(ex), "--record", str(root / "y.jsonl"),
                     "--exempt-pid", str(pid))
            expect(r.returncode == 2 and "exempt tree" in r.stderr, "a process in an exempt tree is refused")
            # During the session: a new thread, a new child, a mask changed by the application.
            expect(say(obs, "thread") == "ok", "a new thread")
            grandkid = int(say(obs, "child"))
            kids.append(grandkid)
            os.sched_setaffinity(ts[0], {mine[0]})
            new_tids = set(tids(pid)) - {x["tid"] for x in th}
            expect(len(new_tids) == 1 and os.sched_getaffinity(new_tids.pop()) == allowed,
                   "the new thread inherited the confined mask")
            r = tool("restore", "--record", str(rec))
            after = masks(pid)
            expect(r.returncode == 0 and after[ts[0]] == {mine[0]} and "changed 1" in r.stdout,
                   f"a mask changed during the session is left: {r.stdout}")
            # The observer's recorded threads had two masks (one was on the
            # excluded CPU alone), so its new thread and its new child (whose
            # nearest recorded process is the observer) are left; the existing
            # child's recorded threads are restored.
            expect("new left 2" in r.stdout and "new restored 0" in r.stdout, f"no single original mask: {r.stdout}")
            expect(all(after[t_] == set(mine) for t_ in tids(pid) if t_ in {x["tid"] for x in th} and t_ != ts[0]
                       and t_ != ts[1]), f"the recorded threads restored: {after}")
            expect(masks(kid) == {t_: set(mine) for t_ in tids(kid)}, "the child restored")
            expect(os.sched_getaffinity(grandkid) == allowed, "the new process left at its mask")
            r = tool("restore", "--record", str(rec))
            expect(r.returncode == 0, "a repeated restore")

            # A process whose threads shared one mask: its new thread and new
            # child get it back; a reused ID is left.
            obs2 = subprocess.Popen([sys.executable, "-c", CHILD], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    text=True)
            procs.append(obs2)
            pid2, kid2 = start(obs2)
            kids.append(kid2)
            time.sleep(0.2)
            rec2 = root / "r2.jsonl"
            r = tool("confine", "--pids", str(pid2), "--exclude-cpus", str(ex), "--record", str(rec2))
            expect(r.returncode == 0, f"a clean confinement: {r.stdout} {r.stderr}")
            # The run's task snapshot: the observer's threads as confined, the
            # probe's worker on the excluded CPU and a helper off it pass; a
            # helper or an observer thread allowed there fails.
            def snapshot(helper: set[int], extra: str = "") -> pathlib.Path:
                fmt = lambda m: ",".join(map(str, sorted(m)))  # noqa: E731
                lines = [f"/proc/{p_}/task/{t_}/status {fmt(os.sched_getaffinity(t_))}"
                         for p_ in (pid2, kid2) for t_ in tids(p_)]
                lines += [f"/proc/4242/task/4243/status {ex}", f"/proc/4242/task/4242/status {fmt(helper)}", extra]
                path = root / "tasks.txt"
                path.write_text("taken: x\n" + "\n".join(lines) + "\n")
                return path
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(allowed)), "--worker-tid", "4243")
            expect(r.returncode == 0 and "0 allowed on" in r.stdout and "1 probe helper threads, 0 allowed" in r.stdout,
                   f"a confined run's snapshot passes: {r.stdout} {r.stderr}")
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(set(mine))), "--worker-tid", "4243")
            expect(r.returncode == 1 and "1 probe helper threads, 1 allowed there" in r.stdout,
                   f"a helper on the excluded CPU fails: {r.stdout}")
            r = tool("check", "--record", str(rec2), "--tasks",
                     str(snapshot(allowed, f"/proc/{pid2}/task/{pid2}/status {','.join(map(str, mine))}")),
                     "--worker-tid", "4243")
            expect(r.returncode == 1 and "1 allowed on" in r.stdout, f"an observer thread there fails: {r.stdout}")
            r = tool("check", "--record", str(rec2), "--tasks", str(snapshot(allowed)), "--worker-tid", "1")
            expect(r.returncode == 1 and "not in the snapshot" in r.stderr, "a missing worker fails")
            expect(say(obs2, "thread") == "ok", "a new thread")
            gk2 = int(say(obs2, "child"))
            kids.append(gk2)
            # Fake a reused ID for the child's main thread in the record.
            entries = [json.loads(x) for x in rec2.read_text().splitlines()]
            for x in entries:
                if x["kind"] == "thread" and x["tid"] == kid2:
                    x["start"] = 1
            rec2.write_text("".join(json.dumps(x, sort_keys=True) + "\n" for x in entries))
            r = tool("restore", "--record", str(rec2))
            expect(r.returncode == 0 and "new restored 2" in r.stdout and "reused 1" in r.stdout,
                   f"new threads with a single original mask, and a reused ID: {r.stdout}")
            expect(all(m == set(mine) for m in masks(pid2).values()) and os.sched_getaffinity(gk2) == set(mine),
                   f"the new thread and process restored: {masks(pid2)} {os.sched_getaffinity(gk2)}\n{r.stdout}\n"
                   f"{rec2.read_text()}")
            expect(os.sched_getaffinity(kid2) == allowed, "the reused ID left alone")
    finally:
        for p in procs:
            p.kill()
            p.wait()
        for k in kids:
            try:
                os.kill(k, 9)
            except OSError:
                pass
    print("observer affinity regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
