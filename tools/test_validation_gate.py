#!/usr/bin/env python3
"""Regressions for tools/validation_gate.py on synthetic runs: a clean emulator run under a verified mask passes; each
check fails on its own defect (a read failure, a recovery, an incomplete summary, a recorded T3 that disagrees, a
verdict outside the allowed failures or doubled, the hardware adapter or no --port, a run mask other than the target,
a failed or late verification, no observation after the run or one that differs, a restoration before the run ended,
failed or missing controls, the wrong mode); unreadable evidence exits 2, contradictory or unexpected fields in the
mask record's steps or the experiment's session markers (a field twice, an empty or missing one, an extra one) included."""

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


TS = "2026-11-12T{}.000000000+0900"
GOOD_RECORD = [f"started={TS.format('09:59:00')}", "cpu=3", "original_cpumask=ffffffff", "original_requested=ffffffff",
               "original_isolated=00000000", "original_census=workers=49 allowed=49 unknown=0", "target=fffffff7",
               f"restore_owed={TS.format('09:59:01')}", f"set_write={TS.format('09:59:02')} status=0",
               f"set_verified={TS.format('09:59:03')} cpumask=fffffff7 requested=fffffff7 workers=49 allowed=0 unknown=0",
               f"observed={TS.format('10:00:40')} cpumask=fffffff7 requested=fffffff7 result=matches",
               f"observed={TS.format('10:04:10')} cpumask=fffffff7 requested=fffffff7 result=matches"]
CLOSED = [f"restore_write={TS.format('10:05:00')} status=0",
          f"restored={TS.format('10:05:01')} cpumask=ffffffff requested=ffffffff workers=49 allowed=49 unknown=0 context=restore"]
EXPERIMENT = [f"started={TS.format('09:58:00')} kernel=x", f"session_A_start={TS.format('09:59:30')}",
              f"session_A_end={TS.format('10:00:30')} status=0", f"check_A_end={TS.format('10:00:40')} status=0",
              f"session_B_start={TS.format('10:01:00')}", f"session_B_end={TS.format('10:04:00')} status=0"]


def record(d: pathlib.Path, lines: list[str]) -> pathlib.Path:
    p = d / "wq.record"
    p.write_text("\n".join(lines) + "\n")
    return p


def experiment(d: pathlib.Path, lines: list[str] = EXPERIMENT) -> pathlib.Path:
    p = d / "experiment.record"
    p.write_text("\n".join(lines) + "\n")
    return p


def controls(d: pathlib.Path, verdicts: tuple = ("controls pass",)) -> pathlib.Path:
    p = d / "fs-controls.run.txt"
    p.write_text("controls\n" + "".join(f"verdict: {v}\n" for v in verdicts))
    return p


def failing(checks: list) -> list:
    return [n for n, ok, _ in checks if not ok]


SET = "the mask set and verified before the session started, never failed"
OBS = "the target observed in effect after the session ended, never otherwise"
OPEN = "the record open, no restoration begun"
REST = "restored after the last session, verified"
FIX = "the mask record's fixed values"
BOUNDS = "the session's bounds in the experiment record"


def units() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = pathlib.Path(t)
        allow = {"T1", "T2", "T6a"}
        rec, ctl, exp = record(d, GOOD_RECORD), controls(d), experiment(d)

        def g(run, lines=None, session="A", state="open", exp_lines=None, ctl_=ctl, mode="untraced"):
            r = record(d, lines) if lines is not None else rec
            e = experiment(d, exp_lines) if exp_lines is not None else exp
            out = vg.gate(run, ctl_, r, allow, mode, e, session, state)
            if lines is not None:
                record(d, GOOD_RECORD)
            if exp_lines is not None:
                experiment(d)
            return out
        got = failing(g(write_run(d)))
        expect(got == [], f"a clean run under a verified mask, the record open: {got}")
        got = failing(g(write_run(d), GOOD_RECORD + CLOSED, state="closed"))
        expect(got == [], f"the same, the record closed after the experiment: {got}")
        got = failing(g(write_run(d), GOOD_RECORD + CLOSED, session="B", state="closed", mode="untraced"))
        expect(got == [], f"session B's bounds, the record closed: {got}")
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
            got = set(failing(g(write_run(d, f"r{i}", **kw))))
            expect(got == want, f"{what}: {sorted(got)}")
        run = write_run(d)

        def at(lines: list[str], key: str, new: str) -> list[str]:
            return [new if ln.startswith(key + "=") else ln for ln in lines]
        cpu2 = [ln.replace("cpu=3", "cpu=2").replace("fffffff7", "fffffffb") for ln in GOOD_RECORD]
        unchanged = at(GOOD_RECORD, "set_verified", f"set_verified={TS.format('09:59:03')} cpumask=ffffffff requested=ffffffff workers=49 allowed=49 unknown=0")
        for what, kw, want in [
            # The accounting review's counterexamples (2026-11-11, Option A review, finding 4).
            ("a complete journal clearing CPU 2, the run at its target", dict(lines=cpu2, run=write_run(d, "c2", mask="fffffffb")), {FIX}),
            ("a verification that reports CPU 3 still allowed", dict(lines=unchanged), {SET}),
            ("a restoration during the session (the record closed)", dict(lines=GOOD_RECORD[:-2] + [
                f"observed={TS.format('10:00:20')} cpumask=fffffff7 requested=fffffff7 result=matches",
                f"restore_write={TS.format('10:00:21')} status=0",
                f"restored={TS.format('10:00:22')} cpumask=ffffffff requested=ffffffff workers=49 allowed=49 unknown=0 context=restore"],
                state="closed"), {OBS, REST}),
            ("a delayed probe: its session ended 22.5 s after the run note, the observation and restoration before",
             dict(lines=GOOD_RECORD[:-2] + [f"observed={TS.format('10:00:20')} cpumask=fffffff7 requested=fffffff7 result=matches"] + [
                 f"restore_write={TS.format('10:00:21')} status=0",
                 f"restored={TS.format('10:00:21')} cpumask=ffffffff requested=ffffffff workers=49 allowed=49 unknown=0 context=restore"],
                  exp_lines=at(EXPERIMENT, "session_A_end", f"session_A_end={TS.format('10:00:22')} status=0"), state="closed"), {OBS, REST}),
            # Each element of the sequence.
            ("a target other than the original less CPU 3", dict(lines=at(GOOD_RECORD, "target", "target=fffffff3")), {FIX, SET, OBS, "the run's recorded mask is the target"}),
            ("original effective and requested masks that differ", dict(lines=at(GOOD_RECORD, "original_requested", "original_requested=fffffffe")), {FIX}),
            ("an incomplete original census", dict(lines=at(GOOD_RECORD, "original_census", "original_census=workers=49 allowed=49 unknown=1")), {FIX}),
            ("a failed set", dict(lines=GOOD_RECORD[:9] + [f"set_failed={TS.format('09:59:02')} reason=x"] + GOOD_RECORD[9:]), {SET, OBS, OPEN}),
            ("a failed write", dict(lines=at(GOOD_RECORD, "set_write", f"set_write={TS.format('09:59:02')} status=1")), {SET}),
            ("a verification after the session started", dict(lines=at(GOOD_RECORD, "set_verified", GOOD_RECORD[9].replace("09:59:03", "09:59:40"))), {SET}),
            ("a verification with an unknown worker", dict(lines=at(GOOD_RECORD, "set_verified", GOOD_RECORD[9].replace("unknown=0", "unknown=1"))), {SET}),
            ("no observation after the session", dict(lines=GOOD_RECORD[:-2] + [GOOD_RECORD[-2].replace("10:00:40", "10:00:10")]), {OBS}),
            ("an observation that differs", dict(lines=GOOD_RECORD + [f"observed={TS.format('10:04:20')} cpumask=ffffffff requested=ffffffff result=differs"]), {OBS}),
            ("an observation reading another mask, worded matches", dict(lines=at(GOOD_RECORD, "observed", f"observed={TS.format('10:00:40')} cpumask=ffffffff requested=ffffffff result=matches")), {OBS}),
            ("times going back", dict(lines=at(GOOD_RECORD, "restore_owed", f"restore_owed={TS.format('09:59:05')}")), {"the mask record's times never go back"}),
            ("an open record with a restoration", dict(lines=GOOD_RECORD + CLOSED), {OPEN}),
            ("a closed record without one", dict(lines=GOOD_RECORD, state="closed"), {REST}),
            ("a restoration before session B ended", dict(lines=GOOD_RECORD[:-1] + [ln.replace("10:05:0", "10:03:5") for ln in CLOSED], state="closed"), {REST}),
            ("a failed restoration before the good one", dict(lines=GOOD_RECORD + [CLOSED[0], f"restore_failed={TS.format('10:05:00')} reason=x"] + CLOSED, state="closed"), {REST}),
            ("a restoration to other masks", dict(lines=GOOD_RECORD + [CLOSED[0], CLOSED[1].replace("requested=ffffffff", "requested=fffffff7")], state="closed"), {REST}),
            ("a restoration with workers still barred", dict(lines=GOOD_RECORD + [CLOSED[0], CLOSED[1].replace("allowed=49", "allowed=48")], state="closed"), {REST}),
            ("a session that failed", dict(exp_lines=at(EXPERIMENT, "session_A_end", f"session_A_end={TS.format('10:00:30')} status=1")), {BOUNDS, SET, OBS}),
            ("no session end", dict(exp_lines=[ln for ln in EXPERIMENT if not ln.startswith("session_A_end")]), {BOUNDS, SET, OBS}),
        ]:
            got = set(failing(g(kw.pop("run", run), **kw)))
            expect(got == want, f"{what}: {sorted(got)}")
        for what, v in (("failed controls", ("controls fail",)), ("no controls verdict", ())):
            got = set(failing(g(run, ctl_=controls(d, v))))
            controls(d)
            expect(got == {"the fair-server controls"}, f"{what}: {sorted(got)}")
        for what, lines in (("the review's minimal record (no originals, no owed or write step)",
                             ["cpu=99", "target=ffffffff", f"set_verified={TS.format('09:59:03')}", f"observed={TS.format('10:00:40')} matches"]),
                            ("an unknown line", GOOD_RECORD + ["unknown=1"]),
                            ("a fixed value twice", GOOD_RECORD + ["target=fffffff7"]),
                            ("a step that is not key=value", GOOD_RECORD + [f"observed={TS.format('10:04:20')} matches"]),
                            # The re-review's contradictory fields (2026-11-11, Option A re-review, finding 4), and their kin.
                            ("a verification with allowed=49 then allowed=0",
                             at(GOOD_RECORD, "set_verified", GOOD_RECORD[9].replace("allowed=0", "allowed=49 allowed=0"))),
                            ("an observation with result=differs then result=matches",
                             at(GOOD_RECORD, "observed", GOOD_RECORD[10].replace("result=matches", "result=differs result=matches"))),
                            ("a write with status=1 then status=0",
                             at(GOOD_RECORD, "set_write", f"set_write={TS.format('09:59:02')} status=1 status=0")),
                            ("a write with an unexpected field", at(GOOD_RECORD, "set_write", f"set_write={TS.format('09:59:02')} status=0 forced=1")),
                            ("a write with an empty status", at(GOOD_RECORD, "set_write", f"set_write={TS.format('09:59:02')} status=")),
                            ("a verification without its unknown count", at(GOOD_RECORD, "set_verified", GOOD_RECORD[9].replace(" unknown=0", ""))),
                            ("an owed restoration with a field", at(GOOD_RECORD, "restore_owed", f"restore_owed={TS.format('09:59:01')} status=0")),
                            ("a start time with a field", at(GOOD_RECORD, "started", f"started={TS.format('09:59:00')} status=0")),
                            ("a restoration without its context", GOOD_RECORD + [CLOSED[0], CLOSED[1].replace(" context=restore", "")]),
                            ("a restoration with context twice", GOOD_RECORD + [CLOSED[0], CLOSED[1] + " context=restore"])):
            try:
                g(run, lines)
                expect(False, f"{what}: read")
            except vg.Unreadable:
                record(d, GOOD_RECORD)
        # The free-form reason keeps the rest of its line, field-like words included.
        _, f = vg.fields(f"{TS.format('09:59:02')} reason=write failed: status=1 cpumask=x", "set_failed")
        expect(f == {"reason": "write failed: status=1 cpumask=x"}, f"a reason holding key=value words: {f}")
        # The same rule for the experiment record's session markers.
        for what, exp_lines in (("a session end with status=1 then status=0 (re-review, finding 4)",
                                 at(EXPERIMENT, "session_A_end", f"session_A_end={TS.format('10:00:30')} status=1 status=0")),
                                ("a session end without its status", at(EXPERIMENT, "session_A_end", f"session_A_end={TS.format('10:00:30')}")),
                                ("a session end with an unexpected field",
                                 at(EXPERIMENT, "session_A_end", f"session_A_end={TS.format('10:00:30')} status=0 retried=1")),
                                ("a session start with a status", at(EXPERIMENT, "session_A_start", f"session_A_start={TS.format('09:59:30')} status=0")),
                                ("a session end twice", EXPERIMENT + [f"session_A_end={TS.format('10:00:31')} status=0"])):
            try:
                g(run, exp_lines=exp_lines)
                expect(False, f"{what}: read")
            except vg.Unreadable:
                experiment(d)
        for what in ("no summary", "no record", "no experiment record"):
            r2 = write_run(d, "u")
            record(d, GOOD_RECORD)
            experiment(d)
            {"no summary": pathlib.Path(str(r2) + ".summary.toml"), "no record": d / "wq.record",
             "no experiment record": d / "experiment.record"}[what].unlink()
            try:
                vg.gate(r2, ctl, d / "wq.record", allow, None, d / "experiment.record", "A", "open")
                expect(False, f"{what}: read")
            except vg.Unreadable:
                pass
        record(d, GOOD_RECORD)
        experiment(d)


def cli() -> None:
    with tempfile.TemporaryDirectory() as t:
        d = pathlib.Path(t)
        rec, ctl, exp = record(d, GOOD_RECORD), controls(d), experiment(d)
        base = [sys.executable, "tools/validation_gate.py", "--wq-record", str(rec), "--controls", str(ctl), "--mode", "untraced",
                "--experiment", str(exp), "--session", "A", "--record-state", "open"]
        p = subprocess.run(base + [str(write_run(d))], capture_output=True, text=True)
        expect(p.returncode == 0 and "gate: PASS (run.csv: 15 of 15 checks)" in p.stdout, f"the CLI passes: {p.returncode} {p.stdout}")
        p = subprocess.run(base + [str(write_run(d, "bad", counters={"recoveries": 1}))], capture_output=True, text=True)
        expect(p.returncode == 1 and "gate: FAIL: the bounded read's" in p.stdout, f"the CLI fails: {p.returncode} {p.stdout}")
        r = write_run(d, "nosum")
        pathlib.Path(str(r) + ".summary.toml").unlink()
        p = subprocess.run(base + [str(r)], capture_output=True, text=True)
        expect(p.returncode == 2 and "gate: UNREADABLE" in p.stdout, f"the CLI on unreadable evidence: {p.returncode} {p.stdout}")
        p = subprocess.run(base[:-6] + [str(write_run(d))], capture_output=True, text=True)
        expect(p.returncode == 2, f"the CLI without the experiment record: {p.returncode}")


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
