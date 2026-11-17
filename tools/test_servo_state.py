#!/usr/bin/env python3
"""Regressions for servo_state.py, run under ctest, without a bus: a stand-in dxl_inspect answers each read as a
scenario file says (a value, an exit status with a message, other output, or a pause).

Pinned: all 24 reads zero passes, with a complete record read back; each read's arguments (--port, the ID, the
register name) and its stdin not inherited; a nonzero value, a failed read, output that is not one integer line (other
text, nothing, two lines, no newline) and a read that does not answer in time each fail, with the failure and everything
read recorded; an existing record (a file, a dangling link) refuses before any read; a record that cannot be created
(a read-only directory) or written (a file size limit) fails without OK; a missing inspector fails; usage errors; and
the record's reread rejects an empty, truncated, inconsistent or duplicated record.
"""

import os
import pathlib
import resource
import signal
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import servo_state  # noqa: E402

TOOL = pathlib.Path(__file__).resolve().parent / "servo_state.py"
IDS = list(range(2, 10))
STANDIN = """#!/usr/bin/env python3
import json, os, sys, time
d = os.path.dirname(os.path.abspath(__file__))
a = sys.argv[1:]
with open(os.path.join(d, "calls"), "a") as f:
    f.write(" ".join(a) + " stdin=" + os.path.realpath("/proc/self/fd/0") + "\\n")
assert a[0] == "--port" and a[2] == "read", a
key = a[3] + " " + a[4]
s = json.load(open(os.path.join(d, "scenario.json"))).get(key, {"out": "0\\n"})
time.sleep(s.get("sleep", 0))
sys.stdout.write(s.get("out", ""))
sys.stderr.write(s.get("err", ""))
sys.exit(s.get("exit", 0))
"""
failures = []


def check(ok: bool, what: str) -> None:
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        failures.append(what)


def run(d: pathlib.Path, scenario: str = "{}", out: str = "rec.txt", extra=(), inspect=None, preexec=None):
    (d / "scenario.json").write_text(scenario)
    (d / "calls").unlink(missing_ok=True)
    cmd = [sys.executable, "-I", str(TOOL), "--port", "/tmp/rtctrl-test-tty", "--inspect", str(inspect or d / "inspect"),
           "--out", str(d / out), "--label", "start", *extra]
    r = subprocess.run(cmd, capture_output=True, text=True, preexec_fn=preexec)
    calls = (d / "calls").read_text().splitlines() if (d / "calls").exists() else []
    return r.returncode, r.stdout + r.stderr, calls


def main() -> int:
    with tempfile.TemporaryDirectory() as t:
        base = pathlib.Path(t)
        n = 0

        def fresh() -> pathlib.Path:
            nonlocal n
            n += 1
            d = base / f"case{n}"
            d.mkdir()
            (d / "inspect").write_text(STANDIN)
            (d / "inspect").chmod(0o755)
            return d

        print("servo_state.py regressions")
        d = fresh()
        st, out, calls = run(d)
        rec = (d / "rec.txt").read_text() if (d / "rec.txt").exists() else ""
        check(st == 0 and "OK (24 reads, all 0" in out and rec.endswith("rows=24 expected=24\nresult=OK\n")
              and servo_state.reread(rec, IDS) == "", f"all zero: exit {st}, a complete record")
        want = [f"--port /tmp/rtctrl-test-tty read {i} {r}" for i in IDS for r in servo_state.REGISTERS]
        check([c.split(" stdin=")[0] for c in calls] == want, "the 24 reads, in order, with --port and the names")
        check(all(c.endswith("stdin=/dev/null") for c in calls), "no read inherits stdin")
        check(oct((d / "rec.txt").stat().st_mode & 0o777) == "0o644", "the record's mode is 0644")

        cases = [
            ("a nonzero Bus Watchdog", '{"5 bus_watchdog": {"out": "7\\n"}}', "not 0: id 5 bus_watchdog = 7"),
            ("a failed read", '{"7 hardware_error_status": {"exit": 1, "err": "comm -3001, error 0x00\\n"}}',
             "a read failed: id 7 hardware_error_status: status 1"),
            ("other output", '{"3 torque_enable": {"out": "abc\\n"}}', "a read failed: id 3 torque_enable: status 0"),
            ("no output", '{"3 torque_enable": {"out": ""}}', "a read failed: id 3 torque_enable"),
            ("two lines", '{"3 torque_enable": {"out": "0\\n0\\n"}}', "a read failed: id 3 torque_enable"),
            ("no newline", '{"3 torque_enable": {"out": "0"}}', "a read failed: id 3 torque_enable"),
            ("a read without an answer in time", '{"9 bus_watchdog": {"sleep": 3}}',
             "a read failed: id 9 bus_watchdog: status timeout"),
        ]
        for name, scen, why in cases:
            d = fresh()
            st, out, calls = run(d, scen, extra=("--timeout", "0.5") if "time" in name else ())
            rec = (d / "rec.txt").read_text() if (d / "rec.txt").exists() else ""
            check(st == 1 and "FAIL" in out and " OK " not in out and f"result=FAIL reason={why}" in rec
                  and "rows=24 expected=24" in rec and len(calls) == 24 and servo_state.reread(rec, IDS) == "",
                  f"{name}: exit {st}, recorded with every read")

        d = fresh()
        (d / "rec.txt").write_text("old\n")
        st, out, calls = run(d)
        check(st == 2 and not calls and (d / "rec.txt").read_text() == "old\n", "an existing record: refused, no read")
        d = fresh()
        os.symlink(d / "nowhere", d / "rec.txt")
        st, out, calls = run(d)
        check(st == 2 and not calls and (d / "rec.txt").is_symlink(), "a dangling link as the record: refused, no read")

        d = fresh()
        (d / "ro").mkdir()
        (d / "ro").chmod(0o555)
        st, out, calls = run(d, out="ro/rec.txt")
        (d / "ro").chmod(0o755)
        check(st == 1 and "the record was not written" in out and " OK " not in out and not (d / "ro/rec.txt").exists(),
              f"a record that cannot be created: exit {st}, never OK")

        def small():
            signal.signal(signal.SIGXFSZ, signal.SIG_IGN)
            resource.setrlimit(resource.RLIMIT_FSIZE, (200, 200))
        d = fresh()
        st, out, calls = run(d, preexec=small)
        check(st == 1 and "the record was not written" in out and " OK " not in out,
              f"a record that cannot be written whole (file size limit): exit {st}, never OK")

        d = fresh()
        st, out, calls = run(d, inspect=d / "missing")
        rec = (d / "rec.txt").read_text() if (d / "rec.txt").exists() else ""
        check(st == 1 and "status error" in rec and "unreadable" in rec, f"a missing inspector: exit {st}, recorded")

        for args, what in ((("--ids", "9-2"), "a descending ID range"), (("--ids", "2,2"), "a repeated ID"),
                           (("--ids", "0-3"), "ID 0"), (("--label", "a b"), "a label with a space"),
                           (("--timeout", "0"), "a zero timeout")):
            d = fresh()
            st, out, calls = run(d, extra=args)
            check(st == 2 and not calls and not (d / "rec.txt").exists(), f"usage: {what}: refused, no read")

        good = servo_state.render(["# h"], [(i, r, "0", "0", "") for i in IDS for r in servo_state.REGISTERS], 24, "")
        check(servo_state.reread(good, IDS) == "", "reread: the complete record")
        lines = good.splitlines(keepends=True)
        check(servo_state.reread("", IDS) != "", "reread: an empty record")
        check(servo_state.reread("".join(lines[:10]), IDS) != "", "reread: a truncated record (rows only)")
        check(servo_state.reread("".join(lines[:-1]), IDS) != "", "reread: no result line")
        check(servo_state.reread(good[:-1], IDS) != "", "reread: an unterminated last line")
        dup = good.replace("id=9 register=hardware_error_status", "id=9 register=bus_watchdog")
        check(servo_state.reread(dup, IDS) != "", "reread: a duplicated (id, register) for a missing one")
        bad = good.replace("id=4 register=bus_watchdog status=0 value=0", "id=4 register=bus_watchdog status=0 value=3")
        check(servo_state.reread(bad, IDS) != "", "reread: result=OK over a nonzero row")
        short = servo_state.render(["# h"], [(i, r, "0", "0", "") for i in IDS[:-1] for r in servo_state.REGISTERS], 24,
                                   "21 of 24 expected reads")
        check(servo_state.reread(short, IDS) == "" and servo_state.judge(
            [(i, r, "0", "0", "") for i in IDS[:-1] for r in servo_state.REGISTERS], IDS) == "21 of 24 expected reads",
            "a record that truly says 21 rows and FAIL rereads as consistent, and its judgement refuses a pass")
        check(servo_state.judge([(i, r, "0", "0", "") for i in IDS for r in servo_state.REGISTERS][:-1], IDS)
              == "23 of 24 expected reads", "judge: a missing read")

    if failures:
        print(f"servo state regressions FAILED: {len(failures)}")
        return 1
    print("servo state regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
