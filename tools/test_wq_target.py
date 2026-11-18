#!/usr/bin/env python3
"""Regressions for tools/wq_target.py on a stand-in CPU sysfs and /proc: the accepted target 4-5 for real-time CPU 3
(siblings 2-3), its layout in a two-group mask, and each refusal (a list that is not canonical in any of its forms, an
offline CPU, the real-time CPU or its sibling, an isolated CPU, a CPU outside the original, a real-time CPU that is
offline or whose siblings list is missing, malformed or does not hold it, a usage error); the census's counts (within
the list, allowed on every original CPU, unknown entries and malformed lists, a gone entry); the CLI's exits."""

from __future__ import annotations

import os
import pathlib
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import wq_target as wt  # noqa: E402

FAILURES: list[str] = []


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def cpu_sysfs(d: pathlib.Path, online: str = "0-31", siblings: dict | None = None) -> str:
    root = d / "cpu"
    root.mkdir(exist_ok=True)
    (root / "online").write_text(online + "\n")
    for c, s in ({3: "2-3"} if siblings is None else siblings).items():
        (root / f"cpu{c}" / "topology").mkdir(parents=True, exist_ok=True)
        (root / f"cpu{c}" / "topology" / "thread_siblings_list").write_text(s + "\n")
    return str(root)


def refused(f, *args) -> str | None:
    try:
        f(*args)
    except wt.Refused as e:
        return str(e)
    return None


def targets() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = pathlib.Path(t)

        def sysfs(name: str, **kw) -> str:
            (d / name).mkdir()
            return cpu_sysfs(d / name, **kw)
        s = sysfs("good")
        got = wt.target("ffffffff", "00000000", "4-5", "3", s)
        expect(got == ["target=00000030", "target_list=4-5", "rt_cpu=3", "rt_siblings=2-3"], f"the accepted 4-5: {got}")
        got = wt.target("ffffffff", "00000000", "0-1,6-31", "3", s)
        expect(got[0] == "target=ffffffc3", f"a list of two ranges: {got}")
        got = wt.target("ffffffff,ffffffff", "00000000,00000000", "4-5,40", "3", sysfs("wide", online="0-63"))
        expect(got[0] == "target=00000100,00000030", f"a two-group layout: {got}")
        for spec in ("", "4-", "-5", "5-4", "4-4", "4,4", "4,5", "5,4", "04", "4-05", "4-5,5", "4-5,6", " 4-5", "4-5 ",
                     "٤-٥", "4-5,", ",4-5", "a", "4-5,40"):
            expect(refused(wt.target, "ffffffff", "00000000", spec, "3", s) is not None, f"the list {spec!r} was accepted")
        for what, args, want in (
            ("the real-time CPU in the list", ("ffffffff", "00000000", "3-5", "3", s), "the real-time CPU 3 or its SMT siblings"),
            ("its sibling in the list", ("ffffffff", "00000000", "2", "3", s), "the real-time CPU 3 or its SMT siblings"),
            ("an isolated CPU", ("ffffffff", "00000010", "4-5", "3", s), "isolated: 4"),
            ("a CPU outside the original", ("ffffffcf", "00000000", "4-5", "3", s), "outside the original mask: 4,5"),
            ("a malformed original", ("fffffffz", "00000000", "4-5", "3", s), "not a CPU mask"),
            ("a malformed real-time CPU", ("ffffffff", "00000000", "4-5", "03", s), "not a CPU number"),
            ("offline CPUs", ("ffffffff", "00000000", "4-5", "3", sysfs("off", online="0-3,6-31")), "offline: 4,5"),
            ("an offline real-time CPU", ("ffffffff", "00000000", "4-5", "3", sysfs("rt", online="0-2,4-31")), "not online"),
            ("a siblings list without the CPU", ("ffffffff", "00000000", "6", "3", sysfs("sib1", siblings={3: "4-5"})),
             "does not hold it"),
            ("a malformed siblings list", ("ffffffff", "00000000", "4-5", "3", sysfs("sib2", siblings={3: "3-2"})),
             "not a CPU list"),
            ("no siblings list", ("ffffffff", "00000000", "4-5", "3", sysfs("sib3", siblings={})), "cannot read"),
        ):
            got = refused(wt.target, *args)
            expect(got is not None and want in got, f"{what}: {got}")


def censuses() -> None:
    with tempfile.TemporaryDirectory() as t:
        proc = pathlib.Path(t)

        def entry(pid: int, comm: str, allowed: str | None, mode: int | None = None) -> None:
            p = proc / str(pid)
            p.mkdir()
            (p / "comm").write_text(comm + "\n")
            (p / "status").write_text("Name:\tx\n" + (f"Cpus_allowed_list:\t{allowed}\n" if allowed is not None else ""))
            if mode is not None:
                os.chmod(p / "status", mode)
        entry(101, "kworker/u128:3", "4-5")
        entry(102, "kworker/R-writeback", "4")
        entry(103, "kworker/u128:4-events_unbound", "0-31")
        entry(104, "bash", "0-31")
        entry(105, "kworker/3:1", "3")  # a per-CPU worker: outside the census
        got = wt.census(str(proc), "4-5", "ffffffff")
        expect(got == "3 2 1 0", f"within, full, unknown: {got}")
        entry(106, "kworker/u128:5", "3-1")
        entry(107, "kworker/u128:6", None)
        got = wt.census(str(proc), "4-5", "ffffffff")
        expect(got == "3 2 1 2", f"a descending range and a missing list are unknown: {got}")
        if os.geteuid() != 0:
            entry(108, "kworker/u128:7", "4-5", mode=0)
            got = wt.census(str(proc), "4-5", "ffffffff")
            expect(got == "3 2 1 3", f"an unreadable worker is unknown: {got}")
        (proc / "109").mkdir()  # an entry without comm that still exists: unknown
        got = wt.census(str(proc), "4-5", "ffffffff")
        expect(got.split()[3] == ("4" if os.geteuid() != 0 else "3"), f"an existing entry without comm: {got}")


def cli() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = pathlib.Path(t)
        s = cpu_sysfs(d)
        tool = [sys.executable, "-I", "tools/wq_target.py"]
        p = subprocess.run(tool + ["target", "ffffffff", "00000000", "4-5", "3", "--cpu-sysfs", s], capture_output=True, text=True)
        expect(p.returncode == 0 and p.stdout == "target=00000030\ntarget_list=4-5\nrt_cpu=3\nrt_siblings=2-3\n",
               f"the CLI's target: {p.returncode} {p.stdout!r}")
        p = subprocess.run(tool + ["target", "ffffffff", "00000000", "2-5", "3", "--cpu-sysfs", s], capture_output=True, text=True)
        expect(p.returncode == 2 and "wq_target: the list 2-5 names CPUs" in p.stderr and not p.stdout,
               f"the CLI's refusal: {p.returncode} {p.stderr!r}")
        for args in ([], ["target", "ffffffff"], ["target", "ffffffff", "00000000", "4-5", "3", "--cpu", s], ["census"],
                     ["census", str(d / "missing"), "4-5", "ffffffff"]):
            p = subprocess.run(tool + args, capture_output=True, text=True)
            expect(p.returncode == 2 and not p.stdout, f"the CLI with {args}: {p.returncode}")


def main() -> int:
    targets()
    censuses()
    cli()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print("wq target regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
