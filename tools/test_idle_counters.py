#!/usr/bin/env python3
"""Regressions for idle_counters.py, run under ctest.

A fake cpuidle tree stands for two CPUs: one with three states (one without
the optional above/below counters), one without a cpuidle directory.
Pinned: the snapshot's monotonic bracket and fields, unavailable counters
and CPUs marked as such (never zero), the round trip through the text, the
deltas per state, no delta across a changed identity, a changed disable
flag noted, and the command line.
"""

import importlib.util
import pathlib
import subprocess
import sys
import tempfile

spec = importlib.util.spec_from_file_location("ic", "tools/idle_counters.py")
assert spec is not None and spec.loader is not None
ic = importlib.util.module_from_spec(spec)
sys.modules["ic"] = ic
spec.loader.exec_module(ic)


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def state(root: pathlib.Path, cpu: int, i: int, name: str, desc: str, lat: int, res: int,
          usage: int, time_us: int, above: int | None = 0, below: int | None = 0) -> pathlib.Path:
    d = root / f"cpu{cpu}" / "cpuidle" / f"state{i}"
    d.mkdir(parents=True, exist_ok=True)
    for k, v in (("name", name), ("desc", desc), ("latency", lat), ("residency", res), ("disable", 0),
                 ("usage", usage), ("time", time_us), ("rejected", 0)):
        (d / k).write_text(f"{v}\n")
    for k, v in (("above", above), ("below", below)):
        if v is not None:
            (d / k).write_text(f"{v}\n")
    return d


def main() -> int:
    with tempfile.TemporaryDirectory() as t:
        root = pathlib.Path(t)
        state(root, 3, 0, "POLL", "CPUIDLE CORE POLL IDLE", 0, 0, 10, 100)
        state(root, 3, 1, "C1_ACPI", "ACPI FFH MWAIT 0x0", 1, 1, 20, 200)
        s2 = state(root, 3, 2, "C2_ACPI", "ACPI FFH MWAIT 0x10", 127, 381, 30, 300, above=None, below=None)
        (root / "cpu2").mkdir()

        before = ic.snapshot(root, [2, 3])
        t0, t1 = before["t"]
        expect(0 < t0 <= t1, "the snapshot is bracketed by two monotonic stamps")
        expect(before["cpus"][2] == {"unavailable": "no cpuidle directory"}, "a CPU without cpuidle")
        st2 = before["cpus"][3]["states"][2]
        expect(st2["name"] == "C2_ACPI" and st2["latency"] == "127" and st2["residency"] == "381",
               "a state's identity")
        expect(st2["above"] == ic.UNAVAILABLE and st2["below"] == ic.UNAVAILABLE,
               "a missing optional counter is unavailable, not zero")
        text = ic.format_snapshot(before, "before", str(root))
        expect(text.startswith(f"idle counters (before): monotonic_ns {t0} {t1}"), "the header line")
        expect("cpu2 unavailable (no cpuidle directory)" in text, "the unavailable CPU is printed")
        expect('desc="ACPI FFH MWAIT 0x10"' in text and "above=unavailable" in text,
               "a description with spaces is quoted, an unavailable counter named")
        back = ic.parse(text)
        expect(back["t"] == before["t"] and back["cpus"] == before["cpus"], "the text round-trips")

        # Counters move; state 1's identity changes; state 2 gets disabled.
        (root / "cpu3/cpuidle/state0/usage").write_text("15\n")
        (root / "cpu3/cpuidle/state0/time").write_text("160\n")
        (root / "cpu3/cpuidle/state1/latency").write_text("2\n")
        (s2 / "usage").write_text("31\n")
        (s2 / "disable").write_text("1\n")
        after = ic.parse(ic.format_snapshot(ic.snapshot(root, [2, 3]), "after", str(root)))
        d = ic.delta(back, after)
        expect("cpu3 state0 POLL: usage +5, time +60, above +0, below +0, rejected +0" in d, f"a delta:\n{d}")
        expect("cpu3 state1: no delta (identity changed" in d, f"no delta across a changed identity:\n{d}")
        expect("cpu3 state2 C2_ACPI: usage +1, time +0, above unavailable, below unavailable, rejected +0"
               " (disable changed: 0 -> 1)" in d, f"unavailable counters and a changed disable flag:\n{d}")
        expect("cpu2: no delta (no cpuidle directory)" in d, f"an unavailable CPU has no delta:\n{d}")
        try:
            ic.parse("nothing here")
            expect(False, "a text without a header is refused")
        except ValueError:
            pass

        # The command line.
        b, a = root / "b.txt", root / "a.txt"
        r = subprocess.run([sys.executable, "tools/idle_counters.py", "snapshot", "--cpus", "2,3",
                            "--label", "before", "--root", str(root)], capture_output=True, text=True)
        expect(r.returncode == 0 and "idle counters (before)" in r.stdout, f"snapshot: {r.stderr}")
        b.write_text(r.stdout)
        a.write_text(r.stdout)
        r = subprocess.run([sys.executable, "tools/idle_counters.py", "delta", str(b), str(a)],
                           capture_output=True, text=True)
        expect(r.returncode == 0 and "cpu3 state0 POLL: usage +0" in r.stdout, f"delta: {r.stderr}")
        r = subprocess.run([sys.executable, "tools/idle_counters.py", "snapshot", "--cpus", "x",
                            "--label", "before"], capture_output=True, text=True)
        expect(r.returncode == 2, "a bad CPU list is refused")
    print("idle counters regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
