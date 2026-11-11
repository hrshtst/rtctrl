#!/usr/bin/env python3
"""Regressions for tools/validation_gate.py on synthetic runs: a clean emulator run under a verified mask passes; each
check fails on its own defect (a read failure, a recovery, an incomplete summary, a recorded T3 that disagrees, a
verdict outside the allowed failures or doubled, the hardware adapter or no --port, a run mask other than the target,
a failed or late verification, no observation after the run or one that differs, a restoration before the run ended,
failed or missing controls, the wrong mode); unreadable evidence exits 2."""

from __future__ import annotations

import pathlib
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import validation_gate as vg  # noqa: E402

FAILURES: list[str] = []
PERIOD = 1_000_000
HEADER = ("cycle,scheduled_wake_ns,wake_ns,read_start_ns,instr_return_ns,first_reply_ns,last_reply_ns,read_end_ns,"
          "limit_start_ns,bus_write_start_ns,write_return_ns,drain_return_ns,ack_return_ns,completion_upper_ns,"
          "cycle_end_ns,read_ok,write_ok,ack_ok,skipped_periods,read_len")


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def write_run(d: pathlib.Path, name: str = "run", read_fail: int | None = None, counters: dict | None = None,
              log_complete: bool = True, t3_line: str = "PASS", verdicts: tuple = ("fail (T1,T2,T6a)",),
              tty: str = "14", port: str = "--port /tmp/rtctrl-rehearsal-tty ", mask: str = "fffffff7",
              mode: str = "untraced", started: str = "2026-11-12T10:00:00+09:00") -> pathlib.Path:
    rows = []
    for k in range(1, 201):
        s = (k - 1) * PERIOD
        w = s + 2_000
        rows.append([k, s, w, w + 1_000, w + 21_000, w + 500_000, w + 500_000, w + 501_000, w + 510_000, w + 512_000,
                     w + 530_000, -1, -1, -1, w + 540_000, 0 if k == read_fail else 1, 1, 0, 0, 10])
    path = d / f"{name}.csv"
    path.write_text(HEADER + "\n" + "\n".join(",".join(map(str, r)) for r in rows) + "\n")
    c = {"timeouts": 0, "malformed": 0, "servo_errors": 0, "recoveries": 0, "guard_waits": 0, "inhibited": 0,
         "read_failures": 0, "write_failures": 0, "ring_drops": 0} | (counters or {})
    br = ", ".join(f"{k} = {c[k]}" for k in vg.BOUNDED_ZERO)
    pathlib.Path(str(path) + ".summary.toml").write_text(
        'format = "rtctrl-bus-timing"\nversion = 2\nack_register = "model_number"\nkind = "production-traffic"\n'
        f'rate_hz = 1000.0\nlog_complete = {"true" if log_complete else "false"}\nread_path = "bounded"\n'
        f"bounded_read = {{ budget_s = 0.035, guard_s = 0.035, reads = 200, {br} }}\n"
        "[result]\nstarted = true\nerror = \"\"\ncycles = 200\nelapsed_s = 0.2\nack_invalid = 0\n"
        f"read_failures = {c['read_failures']}\nwrite_failures = {c['write_failures']}\nring_drops = {c['ring_drops']}\n"
        "late_wake_stop = false\nlate_wake_s = 0.0\nlate_wake_lateness_s = 0.0\n")
    pathlib.Path(str(path) + ".run.txt").write_text(
        f"rtctrl traced torque-off run\nstarted: {started}\ncondition: rt-loaded-excl\nmode: {mode}\n"
        f"command: ./build/apps/x7_bus_timing {port}--rate-hz 1000 --profile current --duration 20\n"
        f"unbound workqueues: cpumask {mask}, affinity scope cache\n"
        f"host: tty {tty}, latency_timer n/a, usb : \"\" (),  Mbit/s\n"
        f"  T1   measured  FAIL          x\n  T3   measured  {t3_line}          0 read / 0 write failures\n"
        + "".join(f"verdict: {v}\n" for v in verdicts))
    return path


GOOD_RECORD = ["started=2026-11-12T09:59:00.000000000+0900", "cpu=3", "original_cpumask=ffffffff",
               "original_requested=ffffffff", "original_isolated=00000000", "original_census=49 49", "target=fffffff7",
               "restore_owed=2026-11-12T09:59:00.100000000+0900", "set_write=2026-11-12T09:59:00.200000000+0900 status 0",
               "set_verified=2026-11-12T09:59:00.300000000+0900 cpumask fffffff7 requested fffffff7, workers 49 0",
               "observed=2026-11-12T09:59:30.000000000+0900 cpumask fffffff7 requested fffffff7 matches",
               "observed=2026-11-12T10:00:40.000000000+0900 cpumask fffffff7 requested fffffff7 matches"]


def record(d: pathlib.Path, lines: list[str]) -> pathlib.Path:
    p = d / "wq.record"
    p.write_text("\n".join(lines) + "\n")
    return p


def controls(d: pathlib.Path, verdicts: tuple = ("controls pass",)) -> pathlib.Path:
    p = d / "fs-controls.run.txt"
    p.write_text("controls\n" + "".join(f"verdict: {v}\n" for v in verdicts))
    return p


def failing(checks: list) -> list:
    return [n for n, ok, _ in checks if not ok]


def units() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = pathlib.Path(t)
        allow = {"T1", "T2", "T6a"}
        rec, ctl = record(d, GOOD_RECORD), controls(d)
        got = failing(vg.gate(write_run(d), ctl, rec, allow, "untraced"))
        expect(got == [], f"a clean run under a verified mask: {got}")
        cases = [
            ("a read failure", dict(read_fail=50, counters={"timeouts": 1, "read_failures": 1, "guard_waits": 1}, t3_line="FAIL",
                                    verdicts=("fail (T1,T2,T3,T6a)",)),
             {"T3 (timing_summary.py)", "T3 as the session recorded it", "the session's verdict, within the allowed failures",
              "the bounded read's and the run's failure counters"}),
            ("a recovery alone", dict(counters={"recoveries": 1}), {"the bounded read's and the run's failure counters"}),
            ("an incomplete summary", dict(log_complete=False),
             {"a complete production-traffic summary", "T3 (timing_summary.py)"}),
            ("a recorded T3 that disagrees", dict(t3_line="FAIL"), {"T3 as the session recorded it"}),
            ("a verdict failing T5", dict(verdicts=("fail (T1,T5)",)), {"the session's verdict, within the allowed failures"}),
            ("two verdicts", dict(verdicts=("pass", "pass")), {"the session's verdict, within the allowed failures"}),
            ("the hardware adapter", dict(tty="ttyUSB0"), {"the emulator's pty"}),
            ("no --port (the default, the arm's port)", dict(port=""), {"the emulator's pty"}),
            ("a run mask other than the target", dict(mask="ffffffff"), {"the run's recorded mask is the target"}),
            ("the wrong mode", dict(mode="timer"), {"the run's mode"}),
        ]
        for i, (what, kw, want) in enumerate(cases):
            got = set(failing(vg.gate(write_run(d, f"r{i}", **kw), ctl, rec, allow, "untraced")))
            expect(got == want, f"{what}: {sorted(got)}")
        run = write_run(d)
        late = [ln.replace("09:59:00.300", "10:00:05.000") if ln.startswith("set_verified") else ln for ln in GOOD_RECORD]
        for what, lines, want in [
            ("a failed set", GOOD_RECORD + ["set_failed=2026-11-12T09:59:00.400000000+0900 x"],
             {"the mask set and verified once, before the run, never failed"}),
            ("a verification after the run started", late, {"the mask set and verified once, before the run, never failed"}),
            ("no observation after the run", GOOD_RECORD[:-1], {"observed in effect after the run, never otherwise"}),
            ("an observation that differs", GOOD_RECORD + ["observed=2026-11-12T10:00:50.000000000+0900 cpumask ffffffff requested ffffffff differs"],
             {"observed in effect after the run, never otherwise"}),
            ("a restoration before the run ended", GOOD_RECORD + ["restored=2026-11-12T10:00:00.100000000+0900 restore: x"],
             {"restored after the run"}),
            ("a restoration after it", GOOD_RECORD + ["restored=2026-11-12T10:01:00.000000000+0900 restore: x"], set()),
        ]:
            got = set(failing(vg.gate(run, ctl, record(d, lines), allow, "untraced")))
            expect(got == want, f"{what}: {sorted(got)}")
        for what, v in (("failed controls", ("controls fail",)), ("no controls verdict", ())):
            got = set(failing(vg.gate(run, controls(d, v), record(d, GOOD_RECORD), allow, "untraced")))
            expect(got == {"the fair-server controls"}, f"{what}: {sorted(got)}")
        for what in ("no summary", "no record"):
            r2 = write_run(d, "u")
            record(d, GOOD_RECORD)
            (pathlib.Path(str(r2) + ".summary.toml") if what == "no summary" else d / "wq.record").unlink()
            try:
                vg.gate(r2, ctl, d / "wq.record", allow, None)
                expect(False, f"{what}: read")
            except vg.Unreadable:
                pass
        try:
            vg.gate(write_run(d, "v"), ctl, record(d, GOOD_RECORD + ["unknown=1"]), allow, None)
            expect(False, "a record with an unknown line: read")
        except vg.Unreadable:
            pass


def cli() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = pathlib.Path(t)
        rec, ctl = record(d, GOOD_RECORD), controls(d)
        base = [sys.executable, "tools/validation_gate.py", "--wq-record", str(rec), "--controls", str(ctl), "--mode", "untraced"]
        p = subprocess.run(base + [str(write_run(d))], capture_output=True, text=True)
        expect(p.returncode == 0 and "gate: PASS (run.csv: 11 of 11 checks)" in p.stdout, f"the CLI passes: {p.returncode} {p.stdout}")
        p = subprocess.run(base + [str(write_run(d, "bad", counters={"recoveries": 1}))], capture_output=True, text=True)
        expect(p.returncode == 1 and "gate: FAIL: the bounded read's" in p.stdout, f"the CLI fails: {p.returncode} {p.stdout}")
        r = write_run(d, "nosum")
        pathlib.Path(str(r) + ".summary.toml").unlink()
        p = subprocess.run(base + [str(r)], capture_output=True, text=True)
        expect(p.returncode == 2 and "gate: UNREADABLE" in p.stdout, f"the CLI on unreadable evidence: {p.returncode} {p.stdout}")


def main() -> int:
    units()
    cli()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print("validation gate regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
