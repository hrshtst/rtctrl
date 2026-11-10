#!/usr/bin/env python3
"""Regressions for tools/refresh_check.py: complete evidence (fixtures in the
session's real formats) passes, and every missing, unfinished, unreadable,
failed or contrary piece of it fails (or, without a refresh in the run
window, is inconclusive), with a stand-in trace-cmd."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
CHECK = HERE / "refresh_check.py"
FAILS: list[str] = []

RECORD = (
    "trace: stdbuf -oL -eL trace-cmd record -C mono -e sched:sched_switch "
    "-e exceptions:page_fault_kernel -f CPU\\ ==\\ 3 -o x.dat\n"
    "excluded CPUs (no -excl load, no tracers): 2-3; tracer and -excl load CPUs: 0,1,4\n"
    "sudo refresh: loop 300 on CPUs 0-1,4 (read back at the session's start), now 0-1,4\n"
    "halfway snapshot: the task listing's exit 0\n"
)
PROCS = "    PID    PPID COMMAND\n      1       0 systemd\n    300     200 bash\n    302     300 sleep\n"
# The halfway snapshot up to its lineage's closing line (a listing cut off there
# is an unfinished one).
TASKS = (
    "taken: now\n--- ps -eLo pid,tid,psr,cls,rtprio,ni,stat,comm ---\n300 300 1 TS - 0 S bash\n"
    "--- Cpus_allowed_list per task (/proc/<pid>/task/<tid>/status) ---\n/proc/300/task/300/status 0-1,4\n"
    "--- task lineage (observer_affinity.py tasks: pid tid ppid pstart start allowed) ---\n"
    "taken monotonic_ns: 101000000000 101200000000\n"
    "300 300 200 5 5 0-1,4\n303 303 300 9 9 0-1,4\n"
)
COMPLETE_TASKS = TASKS + "lineage complete: 2 tasks\n"
# The window: origin 100 s, first wake at +1 ms, last cycle's end at +10 s.
SUMMARY = "[result]\nmonotonic_origin_ns = 100000000000\nworker_tid = 7\n"
CSV = "cycle,wake_ns,cycle_end_ns\n0,1000000,1500000\n1,9999000000,10000000000\n"
TRACE = """\
           sleep-302   [004] d..2. 100.500000: sched_switch:         sleep:302 [120] S ==> swapper/4:0 [120]
            bash-300   [001] ..... 101.000000: sched_process_fork:   comm=bash pid=300 child_comm=bash child_pid=304
            sudo-304   [001] ..... 101.100000: sched_process_exec:   filename=/usr/bin/sudo pid=304 old_pid=304
            sudo-304   [001] d..2. 101.200000: sched_switch:         sudo:304 [120] S ==> swapper/1:0 [120]
            bash-300   [001] ..... 101.300000: sched_process_fork:   comm=bash pid=300 child_comm=bash child_pid=303
           sleep-303   [000] ..... 101.310000: sched_process_exec:   filename=/usr/bin/sleep pid=303 old_pid=303
     x7_bus_timi-7     [003] d.... 102.000000: page_fault_kernel:    address=0x7f0000001000 ip=x error_code=0x2
"""


def run(name: str, want: int, expect: str = "", trace: str = TRACE, status: int = 0, record: str = RECORD,
        procs: str | None = PROCS, tasks: str | None = COMPLETE_TASKS) -> None:
    with tempfile.TemporaryDirectory(prefix="refresh-check-") as td:
        d = Path(td)
        run_csv = d / "rate-trace-x-timer-20261110-10.csv"
        run_csv.write_text(CSV)
        Path(str(run_csv) + ".summary.toml").write_text(SUMMARY)
        Path(str(run_csv) + ".run.txt").write_text(record)
        if procs is not None:
            Path(str(run_csv) + ".procs.txt").write_text(procs)
        if tasks is not None:
            Path(str(run_csv) + ".tasks.txt").write_text(tasks)
        (d / "trace.txt").write_text(trace)
        fake = d / "trace-cmd"
        fake.write_text(f'#!/bin/sh\ncat "{d}/trace.txt"\necho "stand-in diagnostics" >&2\nexit {status}\n')
        fake.chmod(0o700)
        r = subprocess.run([sys.executable, str(CHECK), str(run_csv)], capture_output=True, text=True,
                           env=dict(os.environ, TRACE_CMD=str(fake)))
        ok = r.returncode == want and expect in r.stdout
        if not ok:
            FAILS.append(f"{name}: exit {r.returncode} (want {want}), expected {expect!r}\n{r.stdout}{r.stderr}")


def main() -> int:
    run("complete evidence", 0, "verdict: PASS")
    run("trace-cmd fails after a plausible prefix", 1, "failed: trace-cmd report exited 1: stand-in diagnostics", status=1)
    run("no halfway snapshot", 1, "failed: no usable halfway snapshot", tasks=None)
    run("a halfway snapshot without its lineage", 1, "has no task lineage", tasks=TASKS.split("--- task lineage")[0])
    run("the loop missing from the halfway lineage", 1, "the halfway snapshot does not list the loop",
        tasks=TASKS.replace("300 300 200 5 5 0-1,4\n", "") + "lineage complete: 1 tasks\n")
    # The lineage must be finished and readable: its parser's completeness rule.
    run("an unfinished lineage (no completeness line)", 1, "it has no completeness line", tasks=TASKS)
    run("an explicitly incomplete lineage", 1, "lineage INCOMPLETE: 1 unreadable",
        tasks=TASKS + "lineage INCOMPLETE: 1 unreadable: denied task 399\n")
    run("a lineage cut off mid-row", 1, "a line it cannot read",
        tasks=TASKS + "303 30\n")
    run("a malformed lineage row", 1, "a line it cannot read",
        tasks=TASKS.replace("303 303 300 9 9 0-1,4\n", "303 303 x 9 9 0-1,4\n") + "lineage complete: 2 tasks\n")
    run("a lineage mask that is not a CPU list", 1, "no usable halfway snapshot",
        tasks=TASKS.replace("303 303 300 9 9 0-1,4", "303 303 300 9 9 0-x") + "lineage complete: 2 tasks\n")
    run("a lineage whose count is not its rows", 1, "holds 2 tasks but closes with",
        tasks=TASKS + "lineage complete: 3 tasks\n")
    run("a lineage that repeats a task", 1, "holds 2 tasks but closes with",
        tasks=TASKS + "303 303 300 9 9 0-1,4\nlineage complete: 3 tasks\n")
    run("a lineage with no time of taking", 1, "does not say when it was taken",
        tasks=COMPLETE_TASKS.replace("taken monotonic_ns: 101000000000 101200000000\n", ""))
    run("a tree member's mask not listed", 1, "task 303 (pid 303) of the refresh is allowed on CPUs (not listed) halfway",
        tasks=TASKS.replace("303 303 300 9 9 0-1,4", "303 303 300 9 9 -") + "lineage complete: 2 tasks\n")
    # The recorded status of that listing.
    run("a halfway listing that failed", 1, "the halfway task listing exited 1",
        record=RECORD.replace("task listing's exit 0", "task listing's exit 1"))
    run("a record without the halfway listing's status", 1, "has 0 halfway task-listing statuses",
        record=RECORD.replace("halfway snapshot: the task listing's exit 0\n", ""))
    # The pre-run listing: every row readable, and finished.
    run("no pre-run process listing", 1, "failed: no usable pre-run process listing", procs=None)
    run("a malformed pre-run row", 1, "has a row it cannot read: 'broken row'", procs=PROCS + "broken row\n")
    run("a pre-run row with no command", 1, "has a row it cannot read", procs=PROCS + "    304     300\n")
    run("a pre-run listing cut off mid-row", 1, "ends with an unfinished line", procs=PROCS + "    30")
    run("a pre-run listing without rows", 1, "lists no processes", procs="    PID    PPID COMMAND\n")
    run("a pre-run listing that repeats a process", 1, "lists process 302 twice", procs=PROCS + "    302     300 sleep\n")
    run("a pre-run listing with another header", 1, "is not a PID PPID COMMAND listing",
        procs=PROCS.replace("    PID    PPID COMMAND\n", "    PID    PPID\n"))
    run("a command with spaces is one row", 0, "verdict: PASS", procs=PROCS + "    305       1 Web Content\n")
    run("a sleep from before the trace runs on CPU 3", 1, "task 302 of the refresh was on CPUs [3]",
        trace=TRACE.replace("sleep-302   [004]", "sleep-302   [003]"))
    run("a halfway mask with an excluded CPU", 1, "task 303 (pid 303) of the refresh is allowed on CPUs 0-4 halfway",
        tasks=COMPLETE_TASKS.replace("303 303 300 9 9 0-1,4", "303 303 300 9 9 0-4"))
    # Every thread of the tree's processes, not one per process.
    threads = TASKS.replace("303 303 300 9 9 0-1,4\n", "303 303 300 9 9 0-1,4\n303 309 300 9 10 %s\n303 310 300 9 11 0-1,4\n")
    run("a multi-threaded process with every mask on the tracers", 0, "verdict: PASS",
        tasks=threads % "0-1,4" + "lineage complete: 4 tasks\n")
    run("a bad mask on a middle thread is not overwritten", 1, "task 309 (pid 303) of the refresh is allowed on CPUs 0-4 halfway",
        tasks=threads % "0-4" + "lineage complete: 4 tasks\n")
    run("a thread of a process that is not the refresh's is not checked", 0, "verdict: PASS",
        tasks=TASKS.replace("303 303 300 9 9 0-1,4\n", "303 303 300 9 9 0-1,4\n400 400 1 9 9 0-31\n400 401 1 9 10 0-31\n") + "lineage complete: 4 tasks\n")
    run("a thread of a child known only to the lineage", 1, "task 311 (pid 306) of the refresh is allowed on CPUs 0-31 halfway",
        tasks=TASKS + "306 306 303 9 9 0-1,4\n306 311 303 9 12 0-31\nlineage complete: 4 tasks\n")
    # The trace follows those threads from the first event.
    nonleader = "            sudo-309   [003] ..... 101.400000: sys_enter:            NR 332 (0, 0, 0)\n"
    run("an existing thread that is not a leader, seen on CPU 3", 1, "task 309 of the refresh was on CPUs [3]",
        tasks=threads % "0-1,4" + "lineage complete: 4 tasks\n", trace=TRACE + nonleader)
    run("an existing thread switched in on CPU 2", 1, "task 310 of the refresh was on CPUs [2]",
        tasks=threads % "0-1,4" + "lineage complete: 4 tasks\n",
        trace=TRACE + "       swapper/2-0     [002] d..2. 101.150000: sched_switch:         swapper/2:0 [120] R ==> sudo:310 [120]\n")
    run("a thread forked from an existing thread, seen on CPU 3", 1, "task 312 of the refresh was on CPUs [3]",
        tasks=threads % "0-1,4" + "lineage complete: 4 tasks\n",
        trace=TRACE + "            sudo-309   [000] ..... 101.500000: sched_process_fork:   comm=sudo pid=309 child_comm=sudo child_pid=312\n"
                      "            sudo-312   [003] ..... 101.600000: sys_enter:            NR 332 (0, 0, 0)\n")
    run("a thread of another process on CPU 3 is not the refresh's", 0, "verdict: PASS",
        trace=TRACE + "            bash-999   [003] ..... 101.400000: sys_enter:            NR 332 (0, 0, 0)\n")
    run("a refresh switched in on CPU 2", 1, "task 304 of the refresh was on CPUs [2]",
        trace=TRACE + "       swapper/2-0     [002] d..2. 101.150000: sched_switch:         swapper/2:0 [120] R ==> sudo:304 [120]\n")
    run("a refresh woken onto CPU 3", 1, "task 304 of the refresh was on CPUs [3]",
        trace=TRACE + "            bash-300   [001] d..2. 101.120000: sched_wakeup:         sudo:304 [120] CPU:003\n")
    run("a NULL-page fault on CPU 3 in the window", 1, "1 kernel page faults at NULL-page addresses on CPU 3",
        trace=TRACE + "            sudo-305   [003] d.... 103.000000: page_fault_kernel:    address=0x0 ip=strncpy_from_user error_code=0x0\n")
    run("a NULL-page fault outside the window", 0, "verdict: PASS",
        trace=TRACE + "            sudo-305   [003] d.... 111.000000: page_fault_kernel:    address=0x0 ip=strncpy_from_user error_code=0x0\n")
    run("no refresh inside the window", 2, "verdict: INCONCLUSIVE",
        trace=TRACE.replace("101.100000: sched_process_exec:   filename=/usr/bin/sudo", "99.100000: sched_process_exec:   filename=/usr/bin/sudo"))
    run("a read-back that is not the tracers' CPUs", 1, "read back CPUs 0-4 at the session's start",
        record=RECORD.replace("on CPUs 0-1,4 (read", "on CPUs 0-4 (read"))
    run("a record without the refresh line", 1, "the run record has no sudo refresh line",
        record="\n".join(ln for ln in RECORD.splitlines() if not ln.startswith("sudo refresh")) + "\n")
    run("a record without the page-fault filter", 1, "traces no kernel page faults",
        record=RECORD.replace("-e exceptions:page_fault_kernel -f CPU\\ ==\\ 3 ", ""))
    run("an empty trace", 1, "the trace has no events", trace="")
    if FAILS:
        print("\n".join(FAILS))
        return 1
    print("refresh check regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
