#!/usr/bin/env python3
"""Regressions for trace_stage.py, run under ctest without root.

A stand-in findmnt answers from a small mount table the test writes, and a
stand-in umount removes the stage's line from it (or fails when told to), so
no real mount is made. Pinned: the budget by mode (the timer examples of the
approved design, the corrected kernel buffers of both trace arrays, the
untraced, controls and profiled budgets, the refusal below the RAM needed);
the mount's verification (tmpfs with noswap and the exact size, private to
the user; ext4, a tmpfs without noswap, a wrong size, no mount and another
mode refused); the marker; publication (the inventory written first, every
file under its final name and hash, the ledger last and checkable with
sha256sum -c, nothing staged changed); failures that keep the stage and
remove only the stage's own partial copies (an injected ENOSPC, a hash that
does not read back, a failed link, a different existing final file, a stage
changed after its inventory), each retried to completion; crashes after a
final link and before the ledger, completed idempotently by a retry; release
only after verification (refused without the ledger or with an altered
copy), a failed unmount reported and then retried; abort of an empty stage
and refusal of one holding data; the leftovers' listing; and the guard,
which stops a recorder below its free-space or memory bound and says why.
"""

from __future__ import annotations

import hashlib
import json
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import time

TOOL = [sys.executable, "tools/trace_stage.py"]
MIB = 1 << 20

FINDMNT = r"""#!/usr/bin/env bash
# The stand-in findmnt: -n -b -o FSTYPE,SIZE,OPTIONS --mountpoint DIR
target=${@: -1}
while read -r path fstype size opts; do
  [ "$path" = "$target" ] && { echo "$fstype $size $opts"; exit 0; }
done <"$FAKE_MOUNTS"
exit 1
"""
UMOUNT = r"""#!/usr/bin/env bash
# The stand-in umount: drops the line of its target, or fails when told to.
[ -e "$FAKE_UMOUNT_FAIL" ] && { echo "umount: $1: target is busy" >&2; exit 32; }
grep -v "^$1 " "$FAKE_MOUNTS" >"$FAKE_MOUNTS.new"; mv "$FAKE_MOUNTS.new" "$FAKE_MOUNTS"
"""


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


class Env:
    def __init__(self, root: pathlib.Path) -> None:
        self.root = root
        self.bin = root / "bin"
        self.bin.mkdir()
        for name, text in (("findmnt", FINDMNT), ("umount", UMOUNT)):
            (self.bin / name).write_text(text)
            (self.bin / name).chmod(0o755)
        self.mounts = root / "mounts"
        self.mounts.write_text("")
        self.umount_fail = root / "umount-fail"
        self.meminfo = root / "meminfo"
        self.set_mem(64_625_280, 43_557_304)  # this host on 2026-11-09, kB
        self.env = dict(os.environ, FINDMNT=str(self.bin / "findmnt"), UMOUNT=str(self.bin / "umount"),
                        TRACE_STAGE_SUDO="none", FAKE_MOUNTS=str(self.mounts), FAKE_UMOUNT_FAIL=str(self.umount_fail))
        self.env.pop("TRACE_STAGE_FAIL", None)

    def set_mem(self, total_kb: int, avail_kb: int, dirty_kb: int = 100, wb_kb: int = 0) -> None:
        self.meminfo.write_text(f"MemTotal:       {total_kb} kB\nMemFree:        1000 kB\n"
                                f"MemAvailable:   {avail_kb} kB\nDirty:          {dirty_kb} kB\n"
                                f"Writeback:      {wb_kb} kB\n")

    def run(self, *args: str, fail: str | None = None) -> subprocess.CompletedProcess:
        env = dict(self.env)
        if fail:
            env["TRACE_STAGE_FAIL"] = fail
        return subprocess.run([*TOOL, *args], capture_output=True, text=True, env=env, timeout=60)

    def mount(self, stage: pathlib.Path, fstype: str = "tmpfs", size_mib: int = 64,
              opts: str = "rw,nosuid,nodev,noexec,relatime,size=65536k,mode=700,noswap") -> None:
        stage.mkdir()
        stage.chmod(0o700)
        with open(self.mounts, "a") as f:
            f.write(f"{stage} {fstype} {size_mib * MIB} {opts}\n")


def budget(e: Env, *args: str) -> tuple[int, dict[str, int], str]:
    r = e.run("budget", "--meminfo", str(e.meminfo), *args)
    kv = {}
    for line in r.stdout.splitlines():
        k, eq, v = line.partition("=")
        if eq and v.isdigit():
            kv[k] = int(v)
    return r.returncode, kv, r.stdout + r.stderr


def sha(p: pathlib.Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def fill(stage: pathlib.Path, base: str) -> dict[str, bytes]:
    files = {f"{base}.trace.dat": os.urandom(3 * MIB + 17), f"{base}": b"cycle,wake_ns\n1,2\n",
             f"{base}.smi.csv": b"0.01,0\n", f"{base}.trace-tmp/{base}.trace.dat.cpu3": b"left by a failed assembly"}
    for rel, data in files.items():
        (stage / rel).parent.mkdir(parents=True, exist_ok=True)
        (stage / rel).write_bytes(data)
    return files


def published(out: pathlib.Path, base: str, files: dict[str, bytes]) -> None:
    for rel, data in files.items():
        expect((out / rel).read_bytes() == data, f"{rel} published with its contents")
    led = out / f"{base}.staging.sha256"
    expect(led.exists(), "the ledger exists")
    expect(len(led.read_text().splitlines()) == len(files), "the ledger lists every staged file")
    r = subprocess.run(["sha256sum", "--quiet", "-c", led.name], cwd=out, capture_output=True, text=True)
    expect(r.returncode == 0, f"the ledger checks with sha256sum -c: {r.stdout}{r.stderr}")
    inv = json.loads((out / f"{base}.staging.inventory.json").read_text())
    expect(sorted(x["path"] for x in inv["files"]) == sorted(files), "the inventory lists every staged file")
    expect(not list(out.rglob(".*partial-*")), "no partial copy is left")


def main() -> int:
    with tempfile.TemporaryDirectory() as t:
        root = pathlib.Path(t)
        e = Env(root)

        # The budget: the approved timer examples with both arrays' buffers.
        common = ("--rate-hz", "1000", "--buffer-kb", "16384", "--cpus", "32")
        st, kv, text = budget(e, "--mode", "timer", "--duration-s", "60", *common)
        expect(st == 0 and kv["kernel"] == 2 * 32 * 16 * MIB, f"a timer run's kernel buffers are both arrays': {text}")
        expect(abs(kv["need"] / 1e9 - 39.0) < 0.06, f"the 60 s timer run needs about 39.0 GB: {kv['need']}")
        expect(abs(kv["size_mib"] * MIB / 1e9 - 28.0) < 0.06, f"its tmpfs cap is about 28.0 GB: {kv['size_mib']}")
        st, kv, text = budget(e, "--mode", "timer", "--duration-s", "10", *common)
        expect(st == 0 and abs(kv["need"] / 1e9 - 19.5) < 0.06, f"the 10 s timer run needs about 19.5 GB: {text}")
        expect(kv["guard_min_free"] == int(250e6 * 20 / 4) + 64 * MIB and kv["guard_min_available"] == kv["headroom"] // 2,
               f"the guard's bounds: {text}")
        st, kv, text = budget(e, "--mode", "untraced", "--duration-s", "60", *common)
        expect(st == 0 and kv["kernel"] == 0 and kv["size_mib"] < 128, f"an untraced run's small budget: {text}")
        st, kv, text = budget(e, "--mode", "controls", "--duration-s", "23", "--buffer-kb", "16384", "--cpus", "32")
        expect(st == 0 and "probe log" not in text and kv["kernel"] == 32 * 16 * MIB,
               f"the controls' budget is their own: {text}")
        st, kv1, _ = budget(e, "--mode", "profiled", "--duration-s", "60", *common, "--perf-hz", "999",
                            "--perf-stack", "4096")
        st, kv2, text = budget(e, "--mode", "profiled", "--duration-s", "60", *common, "--perf-hz", "20000",
                               "--perf-stack", "65528")
        expect(kv2["size_mib"] > kv1["size_mib"] and "perf.data" in text, f"the perf settings are budgeted: {text}")
        e.set_mem(64_625_280, 30_000_000)
        st, kv, text = budget(e, "--mode", "timer", "--duration-s", "60", *common)
        expect(st == 1 and "NOT ENOUGH" in text, f"too little memory refuses: {text}")
        e.set_mem(64_625_280, 43_557_304)

        # The mount's verification.
        good = root / "stage-good"
        e.mount(good)
        r = e.run("verify", "--stage", str(good), "--size-mib", "64")
        expect(r.returncode == 0 and "noswap" in r.stdout, f"a private noswap tmpfs passes: {r.stdout}{r.stderr}")
        for name, fstype, size_mib, opts, why in (
                ("ext4", "ext4", 64, "rw,noswap", "not tmpfs"),
                ("swap", "tmpfs", 64, "rw,size=65536k,mode=700", "without noswap"),
                ("size", "tmpfs", 32, "rw,size=32768k,mode=700,noswap", "not the 64 MiB")):
            s = root / f"stage-{name}"
            e.mount(s, fstype, size_mib, opts)
            r = e.run("verify", "--stage", str(s), "--size-mib", "64")
            expect(r.returncode == 1 and why in r.stderr, f"{name} is refused: {r.stderr}")
        s = root / "stage-none"
        s.mkdir(mode=0o700)
        r = e.run("verify", "--stage", str(s), "--size-mib", "64")
        expect(r.returncode == 1 and "not a mount point" in r.stderr, f"no mount is refused: {r.stderr}")
        good.chmod(0o755)
        r = e.run("verify", "--stage", str(good), "--size-mib", "64")
        expect(r.returncode == 1 and "not private" in r.stderr, f"another mode is refused: {r.stderr}")
        good.chmod(0o700)

        # A normal publication and release.
        out = root / "out"
        out.mkdir()
        base = "rate-trace-x-timer-20000101-1.csv"
        r = e.run("mark", "--stage", str(good), "--session", "s1", "--base", base, "--outdir", str(out))
        expect(r.returncode == 0 and "token" in r.stdout, f"marked: {r.stderr}")
        r = e.run("mark", "--stage", str(good), "--session", "s1", "--base", base, "--outdir", str(out))
        expect(r.returncode == 1, "a stage is marked once")
        files = fill(good, base)
        r = e.run("release", "--stage", str(good), "--outdir", str(out))
        expect(r.returncode == 1 and "never published" in r.stderr, f"release before publication is refused: {r.stderr}")
        r = e.run("publish", "--stage", str(good), "--outdir", str(out), "--meminfo", str(e.meminfo), "--settle-s", "1")
        expect(r.returncode == 0 and "4 files" in r.stdout and "Writeback" in r.stdout, f"published: {r.stdout}{r.stderr}")
        published(out, base, files)
        expect(all((good / rel).exists() for rel in files), "publication leaves the stage as it was")
        r = e.run("publish", "--stage", str(good), "--outdir", str(out), "--meminfo", str(e.meminfo), "--settle-s", "1")
        expect(r.returncode == 0 and "4 already in place" in r.stdout, f"a repeated publication is idempotent: {r.stdout}")
        (out / f"{base}.smi.csv").write_bytes(b"altered\n")
        r = e.run("release", "--stage", str(good), "--outdir", str(out))
        expect(r.returncode == 1 and "does not hold" in r.stderr and good.exists(), f"an altered copy refuses: {r.stderr}")
        (out / f"{base}.smi.csv").write_bytes(files[f"{base}.smi.csv"])
        e.umount_fail.write_text("")
        r = e.run("release", "--stage", str(good), "--outdir", str(out))
        expect(r.returncode == 1 and "unmount" in r.stderr and good.exists(), f"a failed unmount is reported: {r.stderr}")
        expect(not (good / f"{base}.trace.dat").exists() and (good / ".rtctrl-stage").exists(),
               "the data is gone, the stage's records stay for the retry")
        e.umount_fail.unlink()
        r = e.run("release", "--stage", str(good), "--outdir", str(out))
        expect(r.returncode == 0 and not good.exists() and str(good) not in e.mounts.read_text(),
               f"the retried release unmounts and removes: {r.stdout}{r.stderr}")
        published(out, base, files)

        # Failures keep the stage; the retry completes.
        for n, fail, why in ((2, "copy", "No space left"), (3, "hash", "does not read back"), (4, "link", "Input/output")):
            stage, out2 = root / f"stage-f{n}", root / f"out-f{n}"
            out2.mkdir()
            e.mount(stage)
            e.run("mark", "--stage", str(stage), "--session", "s", "--base", base, "--outdir", str(out2))
            files = fill(stage, base)
            foreign = out2 / f".{base}.smi.csv.partial-0123456789abcdef"
            foreign.write_text("another stage's partial copy")
            r = e.run("publish", "--stage", str(stage), "--outdir", str(out2), "--meminfo", str(e.meminfo), fail=fail)
            expect(r.returncode == 1 and why in r.stderr and "retry:" in r.stderr, f"{fail}: {r.stderr}")
            expect(stage.exists() and all((stage / rel).exists() for rel in files), f"{fail}: the stage is kept")
            expect(foreign.exists() and len(list(out2.glob(".*partial-*"))) == 1,
                   f"{fail}: only the stage's own partial copies are removed")
            expect(not (out2 / f"{base}.staging.sha256").exists(), f"{fail}: no ledger")
            foreign.unlink()
            r = e.run("publish", "--stage", str(stage), "--outdir", str(out2), "--meminfo", str(e.meminfo),
                      "--settle-s", "1")
            expect(r.returncode == 0, f"{fail}: the retry publishes: {r.stderr}")
            published(out2, base, files)
            r = e.run("release", "--stage", str(stage), "--outdir", str(out2))
            expect(r.returncode == 0 and not stage.exists(), f"{fail}: then releases")

        # Crashes after a final link and before the ledger.
        for n, fail in ((5, "crash-after-link"), (6, "crash-before-ledger")):
            stage, out2 = root / f"stage-c{n}", root / f"out-c{n}"
            out2.mkdir()
            e.mount(stage)
            e.run("mark", "--stage", str(stage), "--session", "s", "--base", base, "--outdir", str(out2))
            files = fill(stage, base)
            r = e.run("publish", "--stage", str(stage), "--outdir", str(out2), "--meminfo", str(e.meminfo), fail=fail)
            expect(r.returncode == 9, f"{fail}: the crash: {r.returncode} {r.stderr}")
            expect((out2 / f"{base}.staging.inventory.json").exists() and not (out2 / f"{base}.staging.sha256").exists(),
                   f"{fail}: the inventory is durable before the copies, the ledger absent")
            r = e.run("release", "--stage", str(stage), "--outdir", str(out2))
            expect(r.returncode == 1 and stage.exists(), f"{fail}: no release without the ledger")
            r = e.run("publish", "--stage", str(stage), "--outdir", str(out2), "--meminfo", str(e.meminfo),
                      "--settle-s", "1")
            expect(r.returncode == 0 and "already in place" in r.stdout, f"{fail}: the retry completes: {r.stdout}")
            published(out2, base, files)
            expect(e.run("release", "--stage", str(stage), "--outdir", str(out2)).returncode == 0, f"{fail}: released")

        # A different existing final file, and a stage changed after its inventory.
        stage, out2 = root / "stage-x", root / "out-x"
        out2.mkdir()
        e.mount(stage)
        e.run("mark", "--stage", str(stage), "--session", "s", "--base", base, "--outdir", str(out2))
        files = fill(stage, base)
        (out2 / f"{base}.smi.csv").write_bytes(b"someone else's\n")
        r = e.run("publish", "--stage", str(stage), "--outdir", str(out2), "--meminfo", str(e.meminfo))
        expect(r.returncode == 1 and "never overwritten" in r.stderr, f"a different final file is refused: {r.stderr}")
        expect((out2 / f"{base}.smi.csv").read_bytes() == b"someone else's\n", "and left untouched")
        (out2 / f"{base}.smi.csv").unlink()
        (stage / f"{base}.smi.csv").write_bytes(b"written late\n")
        r = e.run("publish", "--stage", str(stage), "--outdir", str(out2), "--meminfo", str(e.meminfo))
        expect(r.returncode == 1 and "changed after its inventory" in r.stderr, f"a changed stage is refused: {r.stderr}")
        r = e.run("publish", "--stage", str(stage), "--outdir", str(root / "elsewhere"), "--meminfo", str(e.meminfo))
        expect(r.returncode == 1 and "belongs to the output directory" in r.stderr, "another output directory refused")

        # Abort: an empty unmarked or marked stage is undone; one with data is kept.
        s = root / "stage-a1"
        e.mount(s)
        r = e.run("abort", "--stage", str(s))
        expect(r.returncode == 0 and not s.exists() and str(s) not in e.mounts.read_text(), f"abort: {r.stderr}")
        s = root / "stage-a2"
        e.mount(s)
        e.run("mark", "--stage", str(s), "--session", "s", "--base", base, "--outdir", str(out2))
        expect(e.run("abort", "--stage", str(s)).returncode == 0 and not s.exists(), "a marked empty stage is aborted")
        s = root / "stage-a3"
        e.mount(s)
        (s / "data").write_text("x")
        r = e.run("abort", "--stage", str(s))
        expect(r.returncode == 1 and "holds data" in r.stderr and (s / "data").exists(), f"data is kept: {r.stderr}")
        s = root / "stage-a4"
        s.mkdir()
        expect(e.run("abort", "--stage", str(s)).returncode == 0 and not s.exists(), "an unmounted mount point goes")

        # Leftovers.
        lroot = root / "stage-root"
        lroot.mkdir()
        expect(e.run("leftovers", "--root", str(lroot)).returncode == 0, "no leftover")
        (lroot / "123-x").mkdir()
        r = e.run("leftovers", "--root", str(lroot))
        expect(r.returncode == 1 and "no complete marker" in r.stdout, f"a leftover is listed: {r.stdout}")

        # The guard stops a recorder and says why, on free space and on memory.
        for name, mem, args in (("space", (64_625_280, 43_557_304), ("--min-free", str(1 << 60), "--min-available", "1")),
                                ("memory", (64_625_280, 1000), ("--min-free", "1", "--min-available", str(1 << 30)))):
            e.set_mem(*mem)
            s = root / f"stage-g-{name}"
            e.mount(s)
            rec = subprocess.Popen(["sleep", "30"])
            g = subprocess.Popen([*TOOL, "guard", "--stage", str(s), *args, "--interval", "0.05",
                                  "--stop-pid", str(rec.pid), "--meminfo", str(e.meminfo)],
                                 stdout=subprocess.PIPE, text=True, env=e.env)
            code = None
            try:
                code = rec.wait(timeout=10)
            except subprocess.TimeoutExpired:
                rec.kill()
                g.kill()
                expect(False, f"the guard did not stop the recorder ({name})")
            time.sleep(0.2)
            g.send_signal(signal.SIGTERM)
            text = g.communicate(timeout=10)[0]
            expect(code == -signal.SIGINT and "guard stop:" in text and ("free" if name == "space" else "MemAvailable")
                   in text.split("guard stop:")[1] and "sampled peak used" in text and g.returncode == 0,
                   f"the guard's {name} stop: {code} {text}")
        e.set_mem(64_625_280, 43_557_304)
    print("trace stage regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
