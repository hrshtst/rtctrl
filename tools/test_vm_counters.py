#!/usr/bin/env python3
"""Regressions for vm_counters.py, run under ctest.

Fake /proc/vmstat and /proc/meminfo files stand for two moments. Pinned: the
snapshot's monotonic bracket, scope and fields; a counter the kernel does not
offer, and an unreadable file, marked unavailable (never zero); the round
trip through the text; the changes with their units; the allocstall_*
counters one by one and summed over those in both snapshots, named; the
gauges before and after; and the command line, on the host's own files too.
"""

import importlib.util
import pathlib
import subprocess
import sys
import tempfile

spec = importlib.util.spec_from_file_location("vc", "tools/vm_counters.py")
assert spec is not None and spec.loader is not None
vc = importlib.util.module_from_spec(spec)
sys.modules["vc"] = vc
spec.loader.exec_module(vc)


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def write(root: pathlib.Path, name: str, vm: dict, mi: dict) -> tuple[str, str]:
    v, m = root / f"{name}.vmstat", root / f"{name}.meminfo"
    v.write_text("".join(f"{k} {x}\n" for k, x in vm.items()))
    m.write_text("".join(f"{k}:{x:>16} kB\n" for k, x in mi.items()))
    return str(v), str(m)


def main() -> int:
    with tempfile.TemporaryDirectory() as t:
        root = pathlib.Path(t)
        vm0 = {"nr_dirty": 10, "nr_writeback": 0, "pswpin": 100, "pswpout": 200, "pgmajfault": 5,
               "pgscan_direct": 0, "nr_dirtied": 1000, "nr_written": 900, "allocstall_dma": 0,
               "allocstall_dma32": 0, "allocstall_normal": 3, "allocstall_movable": 1, "allocstall_device": 0}
        mi0 = {"MemTotal": 64_625_280, "MemAvailable": 43_000_000, "Dirty": 40, "Writeback": 0,
               "SwapFree": 2_000_000, "Shmem": 5_000_000}
        vm1 = dict(vm0, nr_dirty=12, pswpin=104, pgmajfault=9, nr_dirtied=1500, nr_written=1400, allocstall_normal=7)
        del vm1["pgscan_direct"]
        del vm1["allocstall_device"]
        mi1 = dict(mi0, MemAvailable=42_000_000, Dirty=4000)
        p0 = write(root, "a", vm0, mi0)
        p1 = write(root, "b", vm1, mi1)
        s0 = vc.snapshot(*p0)
        s1 = vc.snapshot(*p1)
        expect(s0["t"][0] <= s0["t"][1] <= s1["t"][0], "monotonic brackets in order")
        expect(s1["vmstat"]["pgscan_direct"] == vc.UNAVAILABLE, "a missing counter is unavailable")
        t0 = vc.format_snapshot(s0, "before", *p0)
        expect("system-wide, read-only" in t0 and "allocstall_normal=3" in t0 and "MemAvailable=43000000" in t0,
               f"the snapshot's text: {t0}")
        r0 = vc.parse(t0)
        expect(r0["vmstat"] == s0["vmstat"] and r0["meminfo"] == s0["meminfo"], "the text round trip")
        d = vc.delta(r0, vc.parse(vc.format_snapshot(s1, "after", *p1)))
        for want in ("pswpin +4 pages", "pswpout +0 pages", "pgmajfault +4 faults", "pgscan_direct unavailable",
                     "nr_dirtied +500 pages", "nr_written +500 pages", "allocstall_normal +4 stalls",
                     "allocstall_device unavailable",
                     "allocstall (sum of allocstall_dma, allocstall_dma32, allocstall_movable, allocstall_normal) "
                     "+4 stalls", "nr_dirty 10 -> 12 pages", "MemAvailable 43000000 -> 42000000 kB",
                     "Dirty 40 -> 4000 kB", "system-wide, context, not a cause"):
            expect(want in d, f"the delta has {want!r}: {d}")
        gone = vc.snapshot(str(root / "none"), str(root / "none2"))
        expect(all(v == vc.UNAVAILABLE for v in gone["vmstat"].values()) and
               all(v == vc.UNAVAILABLE for v in gone["meminfo"].values()), "unreadable files: all unavailable")
        d = vc.delta(gone, gone)
        expect("pswpin unavailable" in d and "allocstall (sum) unavailable" in d, f"no zeros invented: {d}")
        # The command line, on the host's files and through saved snapshots.
        for label, paths in (("before", p0), ("after", p1)):
            r = subprocess.run([sys.executable, "tools/vm_counters.py", "snapshot", "--label", label,
                                "--vmstat", paths[0], "--meminfo", paths[1]], capture_output=True, text=True)
            expect(r.returncode == 0, f"snapshot: {r.stderr}")
            (root / f"{label}.txt").write_text(r.stdout)
        r = subprocess.run([sys.executable, "tools/vm_counters.py", "delta", str(root / "before.txt"),
                            str(root / "after.txt")], capture_output=True, text=True)
        expect(r.returncode == 0 and "pswpin +4 pages" in r.stdout, f"delta: {r.stdout}{r.stderr}")
        r = subprocess.run([sys.executable, "tools/vm_counters.py", "snapshot", "--label", "host"],
                           capture_output=True, text=True)
        expect(r.returncode == 0 and r.stdout.startswith("vm counters (host)"), f"the host's own files: {r.stderr}")
        r = subprocess.run([sys.executable, "tools/vm_counters.py", "delta", str(root / "none"), str(root / "none")],
                           capture_output=True, text=True)
        expect(r.returncode == 2, "a missing snapshot file is an error")
    print("vm counters regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
