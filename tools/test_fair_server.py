#!/usr/bin/env python3
"""Regressions for fair_server.py, run under ctest.

The probes' layout comes from an excerpt of this kernel's BTF
(tests/data/btf_raw_fair_server.txt, with a decoy struct rq that lacks the
needed members). Pinned: the offsets and flag bits, the decoy skipped, the
refusals (a missing member, flags outside one 32-bit word), the
definitions' text (the fetch arguments the analysis decodes), the filters,
and the layout's round trip through a session record.

The analysis runs on synthetic events of one CPU: a selection made before
the worker's wake (server-backed), a pick returning no task, a selection
followed by another switch, server-backed and stopper time (mixed),
interrupt time taken out of a label, a contradictory server state, a
re-selection of the running task, a normal task left running after the wake
(preemption delay, only with complete probes; the 54.2 us case's shape),
a waking that finds the worker running, the server calls of another
server or CPU ignored, the deadline-class timers counted, and the report
(no probes: unattributed; probes with no pick: an evidence failure).
"""

import importlib.util
import io
import pathlib
import subprocess
import sys

spec = importlib.util.spec_from_file_location("fair_server", "tools/fair_server.py")
assert spec is not None and spec.loader is not None
fs = importlib.util.module_from_spec(spec)
sys.modules["fair_server"] = fs
spec.loader.exec_module(fs)

FIXTURE = pathlib.Path("tests/data/btf_raw_fair_server.txt")


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def refused(lines: list[str], what: str) -> None:
    try:
        fs.layout_from_btf_raw(lines)
    except fs.LayoutError:
        return
    expect(False, what)


def layout_checks() -> dict:
    lines = FIXTURE.read_text().splitlines()
    lay = fs.layout_from_btf_raw(lines)
    want = {"rq.fair_server": 3192, "rq.ext_server": 2952, "rq.cpu": 3688, "se.runtime": 64,
            "se.deadline": 72, "se.rq": 216, "se.flags_word": 84, "task.pid": 2800,
            "task.policy": 1356, "task.prio": 108, "bit.dl_throttled": 0, "bit.dl_non_contending": 2,
            "bit.dl_server": 4, "bit.dl_server_active": 5, "bit.dl_defer": 6, "bit.dl_defer_armed": 7,
            "bit.dl_defer_running": 8, "bit.dl_defer_idle": 9}
    expect(lay == want, f"the layout from this kernel's BTF: {lay}")
    # A missing member, and a flag moved to another word, are refused.
    refused([ln for ln in lines if "'dl_server_active'" not in ln], "a missing member")
    refused([ln.replace("bits_offset=681", "bits_offset=705") for ln in lines], "flags in two words")
    refused([ln.replace("'pid' type_id=141337 bits_offset=22400", "'pid' type_id=141337 bits_offset=22401")
             for ln in lines], "an unaligned member")
    # The layout survives a session record.
    rec = ["mode: timer", fs.layout_text(lay), "other: x"]
    expect(fs.parse_layout(rec) == lay, "the layout round-trips through a record")
    expect(fs.parse_layout(["mode: timer"]) is None, "a record without a layout")
    return lay


def definition_checks(lay: dict) -> None:
    d = fs.definitions(lay, "rtctrl_fs")
    expect(len(d) == len(fs.EVENTS), "one definition per event")
    names = [line.split()[0].split("/")[1] for line in d]
    expect(tuple(names) == fs.EVENTS, f"the events in order: {names}")
    expect(d[0] == "p:rtctrl_fs/rtfs_dl_pick pick_task_dl rq=$arg1:x64 rq_cpu=+3688($arg1):s32 "
                  "fs_bits=+3276($arg1):x32 fs_runtime=+3256($arg1):s64 fs_deadline=+3264($arg1):u64",
           f"the deadline class's pick: {d[0]}")
    expect(d[1] == "r:rtctrl_fs/rtfs_dl_pick_ret pick_task_dl task=$retval:x64 pid=+2800($retval):s32 "
                  "policy=+1356($retval):u32 prio=+108($retval):s32", f"its return: {d[1]}")
    expect(d[2] == "p:rtctrl_fs/rtfs_fs_pick fair_server_pick_task se=$arg1:x64 srv_rq=+216($arg1):x64 "
                  "srv_cpu=+3688(+216($arg1)):s32 bits=+84($arg1):x32 runtime=+64($arg1):s64 "
                  "deadline=+72($arg1):u64", f"the fair server's pick: {d[2]}")
    expect(d[3].startswith("r:rtctrl_fs/rtfs_fs_pick_ret fair_server_pick_task task=$retval:x64 "),
           "the fair server pick's return")
    expect(d[6].startswith("p:rtctrl_fs/rtfs_srv_timer dl_server_timer se=$arg2:x64 srv_rq=+216($arg2):x64 "
                           "srv_cpu=+3688(+216($arg2)):s32"), f"the timer's server is its second argument: {d[6]}")
    expect(all("$arg1" not in line for line in d[6:]), "the timer reads no first argument")
    f = dict(fs.filters(3))
    expect(f["rtfs_dl_pick"] == "CPU == 3" and f["rtfs_fs_pick_ret"] == "CPU == 3" and
           f["rtfs_srv_start"] == "srv_cpu == 3" and f["rtfs_srv_timer"] == "srv_cpu == 3",
           f"the filters: {f}")


def cli_checks() -> None:
    r = subprocess.run([sys.executable, "tools/fair_server.py", "definitions", "--group", "g1",
                        "--btf-raw", str(FIXTURE)], capture_output=True, text=True)
    expect(r.returncode == 0 and r.stdout.startswith("p:g1/rtfs_dl_pick "), f"definitions: {r.stderr}")
    r = subprocess.run([sys.executable, "tools/fair_server.py", "definitions", "--group", "a/b",
                        "--btf-raw", str(FIXTURE)], capture_output=True, text=True)
    expect(r.returncode == 2, "a group name with a slash is refused")
    r = subprocess.run([sys.executable, "tools/fair_server.py", "layout", "--btf-raw", "/nonexistent"],
                       capture_output=True, text=True)
    expect(r.returncode == 2 and "fair_server:" in r.stderr, "an unreadable dump is refused")


W = 500          # the worker
VALID = 0x160    # dl_server_active (bit 5), dl_defer (6) and dl_defer_running (8); not throttled
INACTIVE = 0x40  # dl_defer only: not active


def new(lay: dict | None) -> "fs.FairServerEvents":
    return fs.FairServerEvents(3, lay, W)


def sw(ev, ts: int, prev: str, ppid: int, pprio: int, state: str, nxt: str, npid: int, nprio: int) -> None:
    ev.feed(ts, 3, ppid, prev, "sched_switch", f"{prev}:{ppid} [{pprio}] {state} ==> {nxt}:{npid} [{nprio}]")


def sel(ev, ts: int, pid: int, bits: int = VALID, task: int = 0xA0, cpu: int = 3) -> None:
    ev.feed(ts, cpu, 1, "x", "rtfs_dl_pick", "(pick_task_dl+0x0/0x10) rq=0xff00 rq_cpu=3 fs_bits=0x0")
    ev.feed(ts + 100, cpu, 1, "x", "rtfs_fs_pick",
            f"(fair_server_pick_task+0x0/0x10) se=0x10 srv_rq=0xff00 srv_cpu=3 bits={hex(bits)} runtime=1 deadline=2")
    ev.feed(ts + 200, cpu, 1, "x", "rtfs_fs_pick_ret", f"(a <- b) task={hex(task)} pid={pid} policy=0")
    ev.feed(ts + 300, cpu, 1, "x", "rtfs_dl_pick_ret", f"(a <- b) task={hex(task)} pid={pid} policy=0 prio=120")


def waking(ev, ts: int, waker: str = "swapper/3", cpu: int = 3) -> None:
    ev.feed(ts, cpu, 0, waker, "sched_waking", f"comm=w pid={W} prio=19 target_cpu=003")


def one_wait(ev, complete: bool = True) -> dict:
    ev.finish(10**9)
    ws = ev.waits(complete=complete)
    expect(len(ws) == 1, f"one wait: {ws}")
    return ws[0]


def analysis_checks(lay: dict) -> None:
    us = 1000
    # A selection before the wake: the task it dispatched runs into the wait.
    ev = new(lay)
    sw(ev, 0, "w", W, 19, "S", "swapper/3", 0, 120)
    sel(ev, 1 * us, 100)
    sw(ev, 2 * us, "swapper/3", 0, 120, "R", "a", 100, 120)
    waking(ev, 10 * us)
    ev.feed(10 * us + 900, 3, 0, "a", "sched_wakeup", f"w:{W} [19] CPU:003")
    sw(ev, 110 * us, "a", 100, 120, "S", "w", W, 19)
    w = one_wait(ev)
    expect(w["ns"] == 100 * us and w["by"]["server-backed"] == 100 * us and w["class"] == "server-backed",
           f"a selection before the wake: {w}")
    expect(w["wakeup"] == 10 * us + 900, "the completed wakeup is kept")
    expect(ev.counts["fair selections"] == 1 and ev.counts["dispatched"] == 1, f"counts: {ev.counts}")
    out = io.StringIO()
    failures: list[str] = []
    fs.report(ev, 0, 10, out, failures)
    text = out.getvalue()
    expect("waits of 50 us or more: 1: server-backed 1, mixed 0, other mechanism 0, INDETERMINATE 0" in text
           and "sched_wakeup +0.9" in text and not failures, f"the report:\n{text}")

    # A pick returning no task, then a normal task with no selection after the wake.
    ev = new(lay)
    sw(ev, 0, "w", W, 19, "S", "swapper/3", 0, 120)
    ev.feed(1 * us, 3, 1, "x", "rtfs_dl_pick", "rq=0xff00 rq_cpu=3 fs_bits=0x0")
    ev.feed(1 * us + 100, 3, 1, "x", "rtfs_fs_pick", f"se=0x10 srv_rq=0xff00 srv_cpu=3 bits={hex(VALID)}")
    ev.feed(1 * us + 200, 3, 1, "x", "rtfs_fs_pick_ret", "task=0x0 pid=0 policy=0")
    ev.feed(1 * us + 300, 3, 1, "x", "rtfs_dl_pick_ret", "task=0x0 pid=0 policy=0 prio=0")
    waking(ev, 10 * us)
    sw(ev, 20 * us, "swapper/3", 0, 120, "R", "b", 101, 120)
    sw(ev, 80 * us, "b", 101, 120, "S", "w", W, 19)
    w = one_wait(ev)
    expect(ev.counts["no task"] == 1 and w["by"]["normal unvalidated"] == 60 * us and w["class"] == "INDETERMINATE",
           f"a pick with no task, then an unvalidated normal task: {w} {ev.counts}")

    # A selection followed by a switch to another task.
    ev = new(lay)
    sw(ev, 0, "w", W, 19, "S", "swapper/3", 0, 120)
    sel(ev, 1 * us, 100)
    sw(ev, 2 * us, "swapper/3", 0, 120, "R", "c", 102, 120)
    expect(ev.counts["not dispatched"] == 1 and ev.counts["dispatched"] == 0, f"not dispatched: {ev.counts}")

    # Server-backed, then the stopper: mixed.
    ev = new(lay)
    sw(ev, 0, "w", W, 19, "S", "swapper/3", 0, 120)
    sel(ev, 1 * us, 100)
    sw(ev, 2 * us, "swapper/3", 0, 120, "R", "a", 100, 120)
    waking(ev, 10 * us)
    sw(ev, 50 * us, "a", 100, 120, "R", "migration/3", 21, 0)
    sw(ev, 70 * us, "migration/3", 21, 0, "S", "w", W, 19)
    w = one_wait(ev)
    expect(w["by"]["server-backed"] == 40 * us and w["by"]["higher class"] == 20 * us and w["class"] == "mixed",
           f"server-backed and the stopper: {w}")
    expect(w["ran"] == ["a", "migration/3"], f"what ran: {w['ran']}")

    # Interrupt time comes out of the label it overlaps.
    ev = new(lay)
    sw(ev, 0, "w", W, 19, "S", "swapper/3", 0, 120)
    sel(ev, 1 * us, 100)
    sw(ev, 2 * us, "swapper/3", 0, 120, "R", "a", 100, 120)
    waking(ev, 10 * us)
    ev.feed(30 * us, 3, 100, "a", "irq_handler_entry", "irq=125 name=xhci_hcd")
    ev.feed(32 * us, 3, 100, "a", "softirq_entry", "vec=1 [action=TIMER]")
    ev.feed(38 * us, 3, 100, "a", "softirq_exit", "vec=1 [action=TIMER]")
    ev.feed(40 * us, 3, 100, "a", "irq_handler_exit", "irq=125 ret=handled")
    sw(ev, 70 * us, "a", 100, 120, "S", "w", W, 19)
    w = one_wait(ev)
    expect(w["by"]["server-backed"] == 50 * us and w["by"]["interrupt"] == 10 * us and w["class"] == "server-backed",
           f"interrupt time, nested: {w}")

    # A contradictory state: the server picked while inactive.
    ev = new(lay)
    sw(ev, 0, "w", W, 19, "S", "swapper/3", 0, 120)
    sel(ev, 1 * us, 100, bits=INACTIVE)
    sw(ev, 2 * us, "swapper/3", 0, 120, "R", "a", 100, 120)
    waking(ev, 10 * us)
    sw(ev, 70 * us, "a", 100, 120, "S", "w", W, 19)
    w = one_wait(ev)
    expect(ev.counts["contradictory"] == 1 and w["by"]["contradictory"] == 60 * us and w["class"] == "INDETERMINATE",
           f"a contradictory state: {w} {ev.counts}")

    # A re-selection of the running task: from the pick on, server-backed.
    ev = new(lay)
    sw(ev, 0, "w", W, 19, "S", "a", 100, 120)
    waking(ev, 10 * us)
    sel(ev, 11 * us, 100)
    ev.feed(13 * us, 3, 1, "x", "rtfs_dl_pick", "rq=0xff00 rq_cpu=3 fs_bits=0x0")
    sw(ev, 80 * us, "a", 100, 120, "S", "w", W, 19)
    w = one_wait(ev)
    expect(ev.counts["re-selected"] == 1 and w["by"]["preemption delay"] == 1300 and
           w["by"]["server-backed"] == 70 * us - 1300 and w["class"] == "server-backed",
           f"a re-selection: {w} {ev.counts}")

    # A normal task left running after the wake, then the stopper (the 54.2 us
    # case's shape): another mechanism with complete probes, INDETERMINATE without.
    for complete, want in ((True, "other mechanism"), (False, "INDETERMINATE")):
        ev = new(lay)
        sw(ev, 0, "w", W, 19, "S", "kworker/u128:1", 300, 120)
        waking(ev, 10 * us, waker="kworker/u128:1")
        sw(ev, 30 * us, "kworker/u128:1", 300, 120, "R", "migration/3", 21, 0)
        sw(ev, 64_200, "migration/3", 21, 0, "S", "w", W, 19)
        w = one_wait(ev, complete)
        expect(w["ns"] == 54_200 and w["class"] == want, f"preemption delay, complete={complete}: {w}")

    # A waking that finds the worker running is no wait.
    ev = new(lay)
    sw(ev, 0, "swapper/3", 0, 120, "R", "w", W, 19)
    waking(ev, 10 * us, waker="w")
    sw(ev, 20 * us, "w", W, 19, "S", "swapper/3", 0, 120)
    ev.finish(10**9)
    expect(ev.waits(complete=True) == [], "a waking while running")

    # The server calls: only CPU 3's fair server counts; the deadline-class timers.
    ev = new(lay)
    ev.feed(1, 2, 1, "x", "rtfs_srv_start", f"se={hex(0xff00 + 3192)} srv_rq=0xff00 srv_cpu=3 bits=0x20")
    ev.feed(2, 3, 1, "x", "rtfs_srv_start", f"se={hex(0xff00 + 2952)} srv_rq=0xff00 srv_cpu=3 bits=0x20")
    ev.feed(3, 3, 1, "x", "rtfs_srv_stop", f"se={hex(0xee00 + 3192)} srv_rq=0xee00 srv_cpu=2 bits=0x0")
    ev.feed(4, 3, 1, "x", "hrtimer_start", "hrtimer=0x1 function=dl_task_timer/0x0 expires=5 softexpires=5 mode=0x0")
    ev.feed(5, 3, 1, "x", "hrtimer_expire_entry", "hrtimer=0x2 now=5 function=inactive_task_timer/0x0")
    ev.feed(6, 2, 1, "x", "hrtimer_start", "hrtimer=0x3 function=dl_task_timer/0x0 expires=5 softexpires=5 mode=0x0")
    expect(ev.server_calls == {"rtfs_srv_start": 1, "rtfs_srv_stop": 0, "rtfs_srv_timer": 0},
           f"the server calls: {ev.server_calls}")
    expect(ev.dl_timers["dl_task_timer start"] == 1 and ev.dl_timers["inactive_task_timer expiry"] == 1,
           f"the deadline-class timers: {ev.dl_timers}")

    # The report: no probes in the capture, and probes that recorded no pick.
    for layout_, want, fail_ in ((None, "fair-server probes: not in this capture", False),
                                 (lay, "deadline-class picks 0", True)):
        ev = new(layout_)
        sw(ev, 0, "w", W, 19, "S", "swapper/3", 0, 120)
        ev.finish(10**9)
        out = io.StringIO()
        failures = []
        fs.report(ev, None, 10, out, failures)
        expect(want in out.getvalue() and bool(failures) == fail_, f"the report ({want}): {out.getvalue()} {failures}")


def main() -> int:
    lay = layout_checks()
    definition_checks(lay)
    cli_checks()
    analysis_checks(lay)
    print("fair server regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
