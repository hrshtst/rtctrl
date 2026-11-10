#!/usr/bin/env python3
"""Regressions for trace_stage.py, run under ctest without root.

A stand-in findmnt answers from a small mount table the test writes, and a
stand-in umount emulates a real unmount: the tmpfs's contents vanish and the
underlying mount point reappears as it was before the mount, its mode
included (000 stands for a root-owned 0700 directory, which refuses the
operator); it fails when told to, and so does a stand-in rmdir. No real
mount is made. Pinned: the kernel's ring-buffer geometry and rounding
(Linux 7.0, review 2026-11-09): the page header's payload and offset, its
validation (no data field, two, a subbuffer that is no power of two, a
buffer_subbuf_size_kb that disagrees), n = max(2, ceil(B / p)) subbuffers,
the expected readback truncated per CPU before summing (16,387 KiB per CPU
and 524,384 KiB per array for 16 MiB requested; the two-subbuffer minimum's
7 KiB), the physical data-plus-reader budget apart (16,456 KiB per CPU,
1,078,460,416 bytes for a timer run's two arrays), and the readback check
on a stand-in tracefs: exact, undersized, oversized, unequal CPUs behind a
matching total, an unexpanded buffer, a missing CPU, a missing or malformed
page header and an instance whose geometry disagrees; the budget by mode
(the timer examples of the approved design, both trace arrays' kernel
buffers, the untraced, controls and profiled budgets, a traced budget
refused without geometry, the refusal below the RAM needed);
in every mode and setting, the stage's capacity above the guard's bound
when empty, with the budgeted streams while recording and at the assembly's
end; the guard on a stage modeled with its bounded capacity (findmnt's size,
TRACE_STAGE_CAPACITY=findmnt), not the large filesystem holding the
stand-in: a short untraced stage and a timer stage pass empty and with
their expected files, and trip on exhaustion (the reviewer's case,
2026-11-09: a 45 MiB untraced stage under a 64 MiB bound);
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
copy), a failed unmount reported and then retried; across the unmount
(the reviewer's case, 2026-11-09): a release whose mount point refuses the
operator once unmounted, a crash between the unmount and the removal and a
failed rmdir, each completed by a retried release without the vanished
marker, a bare mount point holding something kept; abort of an empty stage
(mounted, or a bare mount point after a failed mount, accessible or not)
and refusal of one holding data; only stage-named directories touched; the
leftovers' listing, bare mount points included; and the guard, which stops a
recorder below its free-space or memory bound and says why.
"""

from __future__ import annotations

import hashlib
import importlib.util
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
while read -r path fstype size opts under; do
  [ "$path" = "$target" ] && { echo "$fstype $size $opts"; exit 0; }
done <"$FAKE_MOUNTS"
exit 1
"""
UMOUNT = r"""#!/usr/bin/env bash
# The stand-in umount, as a real one: the tmpfs's contents vanish and the
# underlying mount point reappears with its own mode (the table's last
# field); or it fails when told to.
[ -e "$FAKE_UMOUNT_FAIL" ] && { echo "umount: $1: target is busy" >&2; exit 32; }
line=$(grep "^$1 " "$FAKE_MOUNTS") || { echo "umount: $1: not mounted" >&2; exit 32; }
find "$1" -mindepth 1 -delete
chmod "${line##* }" "$1"
grep -v "^$1 " "$FAKE_MOUNTS" >"$FAKE_MOUNTS.new"; mv "$FAKE_MOUNTS.new" "$FAKE_MOUNTS"
"""
RMDIR = r"""#!/usr/bin/env bash
[ -e "$FAKE_RMDIR_FAIL" ] && { echo "rmdir: failed to remove '$1': Device or resource busy" >&2; exit 1; }
exec rmdir "$@"
"""

spec = importlib.util.spec_from_file_location("trace_stage", "tools/trace_stage.py")
assert spec is not None and spec.loader is not None
ts = importlib.util.module_from_spec(spec)
sys.modules["trace_stage"] = ts
spec.loader.exec_module(ts)


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


class Env:
    def __init__(self, root: pathlib.Path) -> None:
        self.root = root
        self.bin = root / "bin"
        self.bin.mkdir()
        for name, text in (("findmnt", FINDMNT), ("umount", UMOUNT), ("rmdir", RMDIR)):
            (self.bin / name).write_text(text)
            (self.bin / name).chmod(0o755)
        self.mounts = root / "mounts"
        self.mounts.write_text("")
        self.umount_fail = root / "umount-fail"
        self.rmdir_fail = root / "rmdir-fail"
        self.n = 0
        self.meminfo = root / "meminfo"
        self.set_mem(64_625_280, 43_557_304)  # this host on 2026-11-09, kB
        self.env = dict(os.environ, FINDMNT=str(self.bin / "findmnt"), UMOUNT=str(self.bin / "umount"),
                        RMDIR=str(self.bin / "rmdir"), TRACE_STAGE_SUDO="none", FAKE_MOUNTS=str(self.mounts),
                        FAKE_UMOUNT_FAIL=str(self.umount_fail), FAKE_RMDIR_FAIL=str(self.rmdir_fail))
        for k in ("TRACE_STAGE_FAIL", "TRACE_STAGE_CAPACITY"):
            self.env.pop(k, None)

    def stage(self, parent: pathlib.Path | None = None) -> pathlib.Path:
        """A fresh stage path, named as the session names its stages."""
        self.n += 1
        return (parent or self.root) / f"{4000 + self.n}-20000101T000000-1-{self.n}-{self.n:08x}"

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
              opts: str = "rw,nosuid,nodev,noexec,relatime,size=65536k,mode=700,noswap", under: str = "700") -> None:
        """The session's mount point (the operator's, 0700) with a stand-in
        mount on it; under is the mode the mount point shows once unmounted."""
        stage.mkdir()
        stage.chmod(0o700)
        with open(self.mounts, "a") as f:
            f.write(f"{stage} {fstype} {size_mib * MIB} {opts} {under}\n")


HEADER = ("\tfield: u64 timestamp;\toffset:0;\tsize:8;\tsigned:0;\n"
          "\tfield: local_t commit;\toffset:8;\tsize:8;\tsigned:1;\n"
          "\tfield: int overwrite;\toffset:8;\tsize:1;\tsigned:1;\n"
          "\tfield: char data;\toffset:16;\tsize:4080;\tsigned:0;\n")  # this kernel's, from a trace.dat
GEO = ("--payload", "4080", "--subbuf", "4096")


def fake_array(d: pathlib.Path, per_cpu: dict[int, str], total: str, header: str | None = HEADER,
               subbuf_kb: str | None = "4") -> None:
    """A stand-in trace array: page header, subbuffer size, per-CPU and total capacities."""
    (d / "events").mkdir(parents=True, exist_ok=True)
    if header is not None:
        (d / "events" / "header_page").write_text(header)
    if subbuf_kb is not None:
        (d / "buffer_subbuf_size_kb").write_text(subbuf_kb + "\n")
    for c, v in per_cpu.items():
        (d / "per_cpu" / f"cpu{c}").mkdir(parents=True, exist_ok=True)
        (d / "per_cpu" / f"cpu{c}" / "buffer_size_kb").write_text(v + "\n")
    (d / "buffer_total_size_kb").write_text(total + "\n")


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


def guard_run(e: Env, stage: pathlib.Path, b: dict, files: dict[str, int], recorder: bool = False,
              linger: float = 0.6) -> tuple[str, int | None]:
    """The guard on a stage modeled with its bounded capacity, after the given
    sparse files (relative path: apparent size) are placed: its record, and
    the recorder's exit when one is given."""
    for rel, size in files.items():
        (stage / rel).parent.mkdir(parents=True, exist_ok=True)
        with open(stage / rel, "wb") as f:
            f.truncate(size)
    env = dict(e.env, TRACE_STAGE_CAPACITY="findmnt")
    rec = subprocess.Popen(["sleep", "30"]) if recorder else None
    g = subprocess.Popen([*TOOL, "guard", "--stage", str(stage), "--min-free", str(b["guard_min_free"]),
                          "--min-available", str(b["guard_min_available"]), "--interval", "0.05",
                          *(["--stop-pid", str(rec.pid)] if rec else []), "--meminfo", str(e.meminfo)],
                         stdout=subprocess.PIPE, text=True, env=env)
    code = None
    try:
        if rec:
            try:
                code = rec.wait(timeout=10)
            except subprocess.TimeoutExpired:
                rec.kill()
        time.sleep(linger)
    finally:
        g.send_signal(signal.SIGTERM)
    return g.communicate(timeout=10)[0], code


def main() -> int:
    with tempfile.TemporaryDirectory() as t:
        root = pathlib.Path(t)
        e = Env(root)

        # The kernel's ring-buffer geometry and rounding.
        g = ts.parse_geometry(HEADER, "4\n")
        expect((g["offset"], g["payload"], g["subbuf"]) == (16, 4080, 4096), f"this kernel's geometry: {g}")
        expect(ts.parse_geometry(HEADER, None)["subbuf"] == 4096, "no buffer_subbuf_size_kb: the header alone")
        for header, kb_file, why in (
                (HEADER, "8", "buffer_subbuf_size_kb reads '8'"),
                (HEADER.replace("\tfield: char data;\toffset:16;\tsize:4080;\tsigned:0;\n", ""), None,
                 "0 data fields"),
                (HEADER + "\tfield: char data;\toffset:16;\tsize:4080;\tsigned:0;\n", None, "2 data fields"),
                (HEADER.replace("size:4080", "size:4000"), None, "unsupported subbuffer geometry"),
                (HEADER.replace("offset:16;\tsize:4080", "offset:x;\tsize:4080"), None, "0 data fields")):
            try:
                ts.parse_geometry(header, kb_file)
                expect(False, f"malformed geometry accepted ({why})")
            except ts.StageError as err:
                expect(why in str(err), f"malformed geometry ({why}): {err}")
        cap = ts.capacity(16384, 4080, 4096)
        expect(cap == {"pages": 4113, "readback_kib": 16387, "physical": 4114 * 4096}, f"16 MiB per CPU: {cap}")
        expect(cap["physical"] // 1024 == 16456, "16,456 KiB data-plus-reader per CPU")
        expect(ts.capacity(1, 4080, 4096) == {"pages": 2, "readback_kib": 7, "physical": 3 * 4096},
               "the two-subbuffer minimum (the kernel's unexpanded 7 KiB)")
        expect(ts.capacity(16384, 8176, 8192)["readback_kib"] == 16391, "another geometry rounds by its own payload")
        expect(32 * cap["readback_kib"] == 524384 and (32 * cap["pages"] * 4080) >> 10 == 524407,
               "the total is the per-CPU KiB summed after truncation (524,384), not the truncated sum (524,407)")
        # The readback check on a stand-in tracefs, main array and instance.
        tfs = root / "tracefs"
        cpus32 = {c: "16387" for c in range(32)}
        fake_array(tfs, cpus32, "524384")
        fake_array(tfs / "instances" / "rtctrl_tsc", cpus32, "524384")
        bargs = ("--tracefs", str(tfs), "--buffer-kb", "16384", "--cpu-list", "0-31", *GEO)
        r = e.run("geometry", "--tracefs", str(tfs))
        expect(r.returncode == 0 and "payload=4080" in r.stdout and "subbuf=4096" in r.stdout and
               "subbuffer 4096 bytes, payload 4080 at offset 16" in r.stdout, f"the geometry: {r.stdout}{r.stderr}")
        r = e.run("buffers", *bargs, "--instance", "rtctrl_tsc")
        expect(r.returncode == 0 and r.stdout.count("read back exactly") == 2 and
               "4113 subbuffers of 4096 bytes (payload 4080); expected 16387 KiB per CPU and 524384 KiB in all" in r.stdout
               and "per CPU main 16456 KiB, rtctrl_tsc 16456 KiB," in r.stdout and "1078460416 bytes for these 2 arrays" in r.stdout and
               "bookkeeping and transient allocations excluded" in r.stdout, f"exact readbacks: {r.stdout}{r.stderr}")
        # An instance with its own size is read back and budgeted at that size, and a wrong one is caught.
        tk_cap = ts.capacity(1024, 4080, 4096)
        tk_total = str(tk_cap["readback_kib"] * 32)
        fake_array(tfs / "instances" / "rtctrl_tk", {c: str(tk_cap["readback_kib"]) for c in range(32)}, tk_total)
        r = e.run("buffers", *bargs, "--instance", "rtctrl_tsc", "--instance", "rtctrl_tk:1024")
        expect(r.returncode == 0 and r.stdout.count("read back exactly") == 3 and "array rtctrl_tk: requested 1024 KiB" in r.stdout
               and f"rtctrl_tk {tk_cap['physical'] // 1024} KiB" in r.stdout and "for these 3 arrays" in r.stdout,
               f"an instance's own size: {r.stdout}{r.stderr}")
        r = e.run("buffers", *bargs, "--instance", "rtctrl_tk:2048")
        expect(r.returncode != 0 and "array rtctrl_tk: requested 2048 KiB" in r.stdout and "NOT AS EXPECTED" in r.stdout,
               f"an instance at another size: {r.stdout}{r.stderr}")
        for name, per_cpu, total, why in (
                ("undersized", {c: "16384" for c in range(32)}, "524288",
                 "32 CPUs not at 16387 KiB: cpu0 16384 KiB"),
                ("oversized", {**cpus32, 5: "16390"}, "524387", "1 CPUs not at 16387 KiB: cpu5 16390 KiB"),
                ("total", cpus32, "524385", "buffer_total_size_kb reads 524385, not 524384"),
                ("skew", {**cpus32, 0: "16390", 1: "16384"}, "524384", "2 CPUs not at 16387 KiB: cpu0 16390 KiB, cpu1"),
                ("unexpanded", {**cpus32, 3: "7 (expanded: 1408)"}, "524384", "cpu3: buffer_size_kb reads '7 (expanded"),
                ("missing CPU", {c: "16387" for c in range(31)}, "508097", "per-CPU buffers for 31 CPUs, not the configured")):
            d = root / f"tracefs-{name.replace(' ', '-')}"
            fake_array(d, per_cpu, total)
            r = e.run("buffers", "--tracefs", str(d), "--buffer-kb", "16384", "--cpu-list", "0-31", *GEO)
            expect(r.returncode == 1 and "NOT AS EXPECTED" in r.stdout and why in r.stdout,
                   f"{name} readback refused: {r.stdout}{r.stderr}")
        d = root / "tracefs-geo"
        fake_array(d, cpus32, "524384")
        fake_array(d / "instances" / "rtctrl_tsc", {c: "16391" for c in range(32)}, "524512",
                   header=HEADER.replace("size:4080", "size:8176"), subbuf_kb="8")
        r = e.run("buffers", "--tracefs", str(d), "--instance", "rtctrl_tsc", "--buffer-kb", "16384", "--cpu-list",
                  "0-31", *GEO)
        expect(r.returncode == 1 and "array main: requested" in r.stdout and "read back exactly" in r.stdout and
               "array rtctrl_tsc: requested 16384 KiB" in r.stdout and
               "its geometry (subbuffer 8192 bytes, payload 8176 at offset 16" in r.stdout and
               "is not the budget's (subbuffer 4096 bytes, payload 4080)" in r.stdout,
               f"an instance whose geometry disagrees: {r.stdout}{r.stderr}")
        d = root / "tracefs-noheader"
        fake_array(d, cpus32, "524384", header=None)
        r = e.run("buffers", "--tracefs", str(d), "--buffer-kb", "16384", "--cpu-list", "0-31", *GEO)
        expect(r.returncode == 1 and "its geometry: no page header" in r.stdout, f"no header: {r.stdout}{r.stderr}")
        r = e.run("geometry", "--tracefs", str(d))
        expect(r.returncode == 1 and "no page header" in r.stderr, f"geometry without a header: {r.stderr}")
        (d / "events" / "header_page").write_text("\tfield: u64 timestamp;\toffset:0;\tsize:8;\tsigned:0;\n")
        r = e.run("geometry", "--tracefs", str(d))
        expect(r.returncode == 1 and "the page header has 0 data fields" in r.stderr, f"malformed: {r.stderr}")

        # The budget: the approved timer examples with both arrays' buffers
        # (data-plus-reader allocation), the stage holding the guard's 64 MiB
        # reserve on top.
        common = ("--rate-hz", "1000", "--buffer-kb", "16384", "--cpus", "32", *GEO)
        st, kv, text = budget(e, "--mode", "timer", "--duration-s", "60", *common)
        expect(st == 1 and "needs the timekeeping recorder's buffer" in text, f"no recorder buffer, no timer budget: {text}")
        common = (*common, "--tk-buffer-kb", "11250")
        st, kv, text = budget(e, "--mode", "timer", "--duration-s", "60", *common)
        tk_physical = 32 * 4096 * (ts.capacity(11250, 4080, 4096)["pages"] + 1)
        expect(st == 0 and kv["tk_physical"] == tk_physical and kv["kernel"] == 1_078_460_416 + tk_physical,
               f"a timer run's kernel buffers are its arrays' and the recorder's: {text}")
        expect("expected readback 16387 KiB per CPU for 16384 KiB requested" in text and "16456 KiB" in text,
               f"the budget states the readback and the allocation apart: {text}")
        expect(abs(kv["need"] / 1e9 - 39.458) < 0.001, f"the 60 s timer run needs about 39.5 GB with the recorder's buffers: {kv['need']}")
        expect(abs(kv["size_mib"] * MIB / 1e9 - 28.08) < 0.01, f"its tmpfs cap is about 28.1 GB: {kv['size_mib']}")
        expect("guard reserve 64 MiB" in text, f"the reserve is stated: {text}")
        st, kv, text = budget(e, "--mode", "timer", "--duration-s", "60", "--rate-hz", "1000", "--buffer-kb", "16384",
                              "--cpus", "32")
        expect(st == 1 and "needs the trace buffers' geometry" in text, f"no geometry, no traced budget: {text}")
        st, kv, text = budget(e, "--mode", "timer", "--duration-s", "10", *common)
        expect(st == 0 and abs(kv["need"] / 1e9 - 19.926) < 0.001, f"the 10 s timer run needs about 19.93 GB with the recorder's buffers: {text}")
        expect(kv["guard_min_free"] == int(250e6 * 20 / 4) + 64 * MIB and kv["guard_min_available"] == kv["headroom"] // 2,
               f"the guard's bounds: {text}")
        st, kv, text = budget(e, "--mode", "untraced", "--duration-s", "10", *common)
        expect(st == 0 and kv["kernel"] == 0 and kv["guard_min_free"] == 64 * MIB and kv["size_mib"] == 109,
               f"the proposed untraced control: 109 MiB under a 64 MiB bound (45 MiB before the fix): {text}")
        st, kv, text = budget(e, "--mode", "untraced", "--duration-s", "60", *common)
        expect(st == 0 and kv["kernel"] == 0 and kv["size_mib"] < 256, f"an untraced run's small budget: {text}")
        st, kv, text = budget(e, "--mode", "controls", "--duration-s", "23", "--buffer-kb", "16384", "--cpus", "32", *GEO)
        expect(st == 0 and "probe log" not in text and kv["kernel"] == 32 * 4114 * 4096,
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

        # Capacity and the guard's bound agree in every mode and setting: the
        # stage stays above the bound empty, with the budgeted streams while
        # recording (the raw trace and the other streams, their 25% margin
        # used too), and at the assembly's end (the assembled trace beside
        # the raw files).
        mem = {"MemTotal": 64_625_280 * 1024, "MemAvailable": 43_557_304 * 1024}
        n = 0
        for mode in ts.MODES:
            for d in (1, 3, 10, 23, 60, 600):
                for rate in (0, 100, 500, 1000):
                    for usbmon in (False, True):
                        for perf in ((0, 0), (999, 4096), (20000, 65528)):
                            for ex in (False, True):
                                b = ts.budget(mode, d, rate, 16384, 32, usbmon, *perf, ex, mem, 4080, 4096, tk_buffer_kb=11250)
                                cap, bound = b["size_mib"] * MIB, b["guard_min_free"]
                                for what, used in (("empty", 0), ("recording", b["raw"] + b["a"]),
                                                   ("recording, its margin used", ts.MARGIN * (b["raw"] + b["a"])),
                                                   ("assembly's end", b["raw"] + b["assembled"] + b["a"])):
                                    expect(cap - used >= bound, f"{mode} {d} s {rate} Hz usbmon {usbmon} perf {perf} "
                                           f"exerciser {ex}: {what}: {cap - used} free under the bound {bound}")
                                n += 1
        expect(n == len(ts.MODES) * 6 * 4 * 2 * 3 * 2, "every setting was checked")

        # The guard on stages modeled with their bounded capacity. The
        # proposed untraced control (10 s at 1000 Hz): empty, then with its
        # expected files (the probe's log and the snapshots, at their budget),
        # it runs without a stop; filled to the bound, it trips (no recorder to
        # stop: recorded, and the run fails).
        b = ts.budget("untraced", 10, 1000, 16384, 32, True, 0, 0, False, mem, 4080, 4096)
        cap = b["size_mib"] * MIB
        s = e.stage()
        e.mount(s, size_mib=b["size_mib"])
        text, _ = guard_run(e, s, b, {})
        expect("stop: none" in text and f"least free {cap} bytes" in text, f"an empty untraced stage: {text}")
        expected = {"x.csv": b["items"]["probe log"], "x.csv.summary.toml": 8192,
                    "x.csv.tasks.txt": b["items"]["snapshots"] // 2, "x.csv.procs.txt": b["items"]["snapshots"] // 2}
        text, _ = guard_run(e, s, b, expected)
        expect("stop: none" in text and "sampled peak used 0 bytes" not in text,
               f"an untraced stage with its expected files: {text}")
        used = sum(-(-v // ts.PAGE) * ts.PAGE for v in expected.values())
        text, _ = guard_run(e, s, b, {"x.csv.fill": cap - b["guard_min_free"] - used + ts.PAGE})
        expect(f"guard stop: the stage has {b['guard_min_free'] - ts.PAGE} bytes free (bound {b['guard_min_free']})"
               in text and "SIGINT to no recorder" in text, f"an exhausted untraced stage trips: {text}")
        # A 10 s timer stage: empty, while recording (the raw trace in
        # trace-cmd's per-CPU temporary files and the other streams), and at
        # the assembly's end, no stop; exhausted, the guard stops the recorder.
        b = ts.budget("timer", 10, 1000, 16384, 32, True, 0, 0, False, mem, 4080, 4096, tk_buffer_kb=5000)
        cap = b["size_mib"] * MIB
        s = e.stage()
        e.mount(s, size_mib=b["size_mib"])
        text, _ = guard_run(e, s, b, {})
        expect("stop: none" in text, f"an empty timer stage: {text}")
        rec = {f"x.csv.trace-tmp/x.csv.trace.dat.cpu{i}": b["raw"] // 4 for i in range(4)}
        rec.update({"x.csv": b["items"]["probe log"], "x.csv.usbmon.pcap": b["items"]["usbmon"],
                    "x.csv.tsc.txt": b["items"]["TSC instance text"], "x.csv.smi.csv": b["items"]["sampler and SMI counts"],
                    "x.csv.other": b["a"] - b["other"] + b["items"]["snapshots"]})
        text, _ = guard_run(e, s, b, rec)
        expect("stop: none" in text, f"a timer stage while recording: {text}")
        text, _ = guard_run(e, s, b, {"x.csv.trace.dat": b["assembled"]})
        expect("stop: none" in text, f"a timer stage at the assembly's end: {text}")
        used = sum(-(-v // ts.PAGE) * ts.PAGE for v in (*rec.values(), b["assembled"]))
        text, code = guard_run(e, s, b, {"x.csv.fill": cap - used - b["guard_min_free"] + ts.PAGE}, recorder=True)
        expect(code == -signal.SIGINT and f"(bound {b['guard_min_free']})" in text.split("guard stop:")[1],
               f"an exhausted timer stage stops its recorder: {code} {text}")

        # The mount's verification.
        good = e.stage()
        e.mount(good)
        r = e.run("verify", "--stage", str(good), "--size-mib", "64")
        expect(r.returncode == 0 and "noswap" in r.stdout, f"a private noswap tmpfs passes: {r.stdout}{r.stderr}")
        for name, fstype, size_mib, opts, why in (
                ("ext4", "ext4", 64, "rw,noswap", "not tmpfs"),
                ("swap", "tmpfs", 64, "rw,size=65536k,mode=700", "without noswap"),
                ("size", "tmpfs", 32, "rw,size=32768k,mode=700,noswap", "not the 64 MiB")):
            s = e.stage()
            e.mount(s, fstype, size_mib, opts)
            r = e.run("verify", "--stage", str(s), "--size-mib", "64")
            expect(r.returncode == 1 and why in r.stderr, f"{name} is refused: {r.stderr}")
        s = e.stage()
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

        # Across the unmount. The mount point refuses the operator once
        # unmounted (as a root-owned 0700 one would): the release completes,
        # never entering it.
        def staged(under: str, parent: pathlib.Path | None = None) -> tuple[pathlib.Path, pathlib.Path]:
            stage, o = e.stage(parent), root / f"out-u{e.n}"
            o.mkdir()
            e.mount(stage, under=under)
            e.run("mark", "--stage", str(stage), "--session", "s", "--base", base, "--outdir", str(o))
            fill(stage, base)
            r = e.run("publish", "--stage", str(stage), "--outdir", str(o), "--meminfo", str(e.meminfo), "--settle-s", "1")
            expect(r.returncode == 0, f"published: {r.stderr}")
            return stage, o
        stage, o = staged("000")
        r = e.run("release", "--stage", str(stage), "--outdir", str(o))
        expect(r.returncode == 0 and not os.path.lexists(stage) and str(stage) not in e.mounts.read_text(),
               f"a mount point that refuses the operator once unmounted is removed: {r.stdout}{r.stderr}")
        # A crash between the unmount and the removal: the bare mount point
        # (its marker gone with the tmpfs) is listed as one, and the retried
        # release removes it.
        lroot = root / "leftovers-u"
        lroot.mkdir()
        stage, o = staged("000", lroot)
        r = e.run("release", "--stage", str(stage), "--outdir", str(o), fail="crash-after-unmount")
        expect(r.returncode == 9 and os.path.lexists(stage) and str(stage) not in e.mounts.read_text(),
               f"the crash after the unmount: {r.returncode} {r.stderr}")
        r = e.run("leftovers", "--root", str(lroot))
        expect(r.returncode == 1 and "not mounted, a bare mount point" in r.stdout, f"the bare mount point listed: {r.stdout}")
        r = e.run("release", "--stage", str(stage), "--outdir", str(o))
        expect(r.returncode == 0 and "was no longer mounted" in r.stdout and not os.path.lexists(stage),
               f"the retried release removes it without its marker: {r.stdout}{r.stderr}")
        r = e.run("release", "--stage", str(stage), "--outdir", str(o))
        expect(r.returncode == 0 and "gone already" in r.stdout, f"and again is a no-op: {r.stdout}{r.stderr}")
        # A failed rmdir after the unmount: reported, the stage unmounted; the
        # retried release removes it.
        stage, o = staged("700")
        e.rmdir_fail.write_text("")
        r = e.run("release", "--stage", str(stage), "--outdir", str(o))
        expect(r.returncode == 1 and "unmounted but was not removed" in r.stderr and stage.is_dir() and
               str(stage) not in e.mounts.read_text() and not list(stage.iterdir()),
               f"a failed removal is reported, the mount point bare: {r.stderr}")
        e.rmdir_fail.unlink()
        r = e.run("release", "--stage", str(stage), "--outdir", str(o))
        expect(r.returncode == 0 and not os.path.lexists(stage), f"the retried release removes it: {r.stdout}{r.stderr}")
        # A bare directory holding something is never emptied: rmdir refuses.
        for cmd in (("release", "--outdir", str(o)), ("abort",)):
            s = e.stage()
            s.mkdir(mode=0o700)
            (s / "kept").write_text("x")
            r = e.run(cmd[0], "--stage", str(s), *cmd[1:])
            expect(r.returncode == 1 and "not removed" in r.stderr and (s / "kept").exists(),
                   f"{cmd[0]}: a bare directory with contents is kept: {r.stderr}")
        # Only a directory named as a stage is ever unmounted or removed.
        s = root / "not-a-stage"
        s.mkdir()
        r = e.run("abort", "--stage", str(s))
        expect(r.returncode == 1 and "not named as a stage" in r.stderr and s.is_dir(), f"a foreign name: {r.stderr}")

        # Failures keep the stage; the retry completes.
        for n, fail, why in ((2, "copy", "No space left"), (3, "hash", "does not read back"), (4, "link", "Input/output")):
            stage, out2 = e.stage(), root / f"out-f{n}"
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
            stage, out2 = e.stage(), root / f"out-c{n}"
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
        stage, out2 = e.stage(), root / "out-x"
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

        # Abort: an empty unmarked or marked stage is undone; one with data is
        # kept; a bare mount point (a failed mount) goes, whether or not it
        # admits the operator.
        s = e.stage()
        e.mount(s)
        r = e.run("abort", "--stage", str(s))
        expect(r.returncode == 0 and not s.exists() and str(s) not in e.mounts.read_text(), f"abort: {r.stderr}")
        s = e.stage()
        e.mount(s, under="000")
        e.run("mark", "--stage", str(s), "--session", "s", "--base", base, "--outdir", str(out2))
        r = e.run("abort", "--stage", str(s))
        expect(r.returncode == 0 and not os.path.lexists(s), f"a marked empty stage is aborted: {r.stderr}")
        s = e.stage()
        e.mount(s)
        (s / "data").write_text("x")
        r = e.run("abort", "--stage", str(s))
        expect(r.returncode == 1 and "holds data" in r.stderr and (s / "data").exists(), f"data is kept: {r.stderr}")
        for mode in (0o700, 0o000):
            s = e.stage()
            s.mkdir()
            s.chmod(mode)
            r = e.run("abort", "--stage", str(s))
            expect(r.returncode == 0 and not os.path.lexists(s) and "was not mounted" in r.stdout,
                   f"a bare mount point (mode {mode:o}) goes: {r.stdout}{r.stderr}")

        # Leftovers: a bare mount point, a stage without a marker, a marked one.
        lroot = root / "stage-root"
        lroot.mkdir()
        expect(e.run("leftovers", "--root", str(lroot)).returncode == 0, "no leftover")
        (lroot / "123-x").mkdir()
        e.mount(lroot / "124-y")
        e.mount(lroot / "125-z")
        e.run("mark", "--stage", str(lroot / "125-z"), "--session", "s9", "--base", base, "--outdir", str(out2))
        r = e.run("leftovers", "--root", str(lroot))
        expect(r.returncode == 1 and "123-x: not mounted, a bare mount point" in r.stdout and
               "124-y: no complete marker" in r.stdout and f"125-z: run {base} of session s9" in r.stdout,
               f"the leftovers are listed: {r.stdout}")

        # The guard stops a recorder and says why, on free space and on memory.
        for name, mem_kb, args in (("space", (64_625_280, 43_557_304), ("--min-free", str(1 << 60), "--min-available", "1")),
                                   ("memory", (64_625_280, 1000), ("--min-free", "1", "--min-available", str(1 << 30)))):
            e.set_mem(*mem_kb)
            s = e.stage()
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
