#!/usr/bin/env python3
"""Regressions for fair_server.py, run under ctest.

The probes' layout comes from an excerpt of this kernel's BTF
(tests/data/btf_raw_fair_server.txt, with a decoy struct rq that lacks the
needed members). Pinned: the offsets and flag bits, the decoy skipped, the
refusals (a missing member, flags outside one 32-bit word), the
definitions' text (the fetch arguments the analysis decodes), the filters,
and the layout's round trip through a session record.
"""

import importlib.util
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


def main() -> int:
    lay = layout_checks()
    definition_checks(lay)
    cli_checks()
    print("fair server regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
