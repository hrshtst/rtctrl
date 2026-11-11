#!/usr/bin/env python3
"""The loaded emulator validation's gate on one finished run (docs/hardware/control-rate.md, Option A): checked between
the untraced and the timer invocations, so that a failed T3 stops the experiment before the timer run (the session
itself treats a failed criterion as data and goes on), and on the timer run before the acceptance analysis. Read-only.

    validation_gate.py <run>.csv [--controls fs-controls-<date>-<attempt>.run.txt] [--wq-record FILE]
                       [--allow T1,T2,T6a] [--mode untraced|timer]

It passes only if every check passes:

- completeness and T3, decided twice and agreeing: by tools/timing_summary.py's own criteria on the probe log and its
  summary (a complete summary of a production-traffic run that started, with no error), and as the session recorded
  them in <run>.csv.run.txt (its T3 line PASS, its verdict failing no criterion outside --allow: T1, T2 and T6a, the
  emulator's anticipated failures);
- the bounded read's counters all zero: timeouts, malformed replies, servo errors, recoveries, guard waits and
  inhibited sends; and the run's read failures, write failures and ring drops;
- the emulator: the probe was given a --port under /tmp and its device line names no USB serial adapter;
- the declared host condition (tools/wq_mask.sh's record, open or closed): set and verified once, never failed, before
  the run started; the run record's unbound-workqueue mask equals the record's target; an observation that the target
  was in effect after the run ended, and none that it was not; a restoration, if recorded, after the run ended;
- with --controls, the fair-server controls' record holds one verdict, and it is "controls pass".

Exit: 0 every check passed; 1 a check failed (stop); 2 the evidence could not be read (stop).
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
import pathlib
import re
import sys

try:
    import tomllib
except ImportError:  # Python < 3.11
    tomllib = None  # type: ignore[assignment]

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import timing_summary  # noqa: E402

BOUNDED_ZERO = ("timeouts", "malformed", "servo_errors", "recoveries", "guard_waits", "inhibited")
RESULT_ZERO = ("read_failures", "write_failures", "ring_drops")
WQ_TIME = re.compile(r"^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)\.(\d{9})([+-]\d{4})\b")
# The criteria thresholds do not enter T3; they are timing_summary's defaults, for its check() alone.
DEFAULTS = argparse.Namespace(period_tolerance=0.001, loaded_skips_per_10k=1.0, lateness_p999=0.10, lateness_max=0.5,
                              host_work_p999=0.70, estimate_p999=0.90)


class Unreadable(Exception):
    pass


def mask_value(m: str) -> int:
    if not re.fullmatch(r"[0-9a-f]+(,[0-9a-f]+)*", m):
        raise Unreadable(f"not a CPU mask: {m!r}")
    v = 0
    for g in m.split(","):
        v = (v << 32) | int(g, 16)
    return v


def wq_time(v: str) -> float:
    m = WQ_TIME.match(v)
    if not m:
        raise Unreadable(f"no time in the record line {v!r}")
    return dt.datetime.strptime(m[1] + m[3], "%Y-%m-%dT%H:%M:%S%z").timestamp() + int(m[2]) / 1e9


def read_wq(path: pathlib.Path) -> dict:
    """tools/wq_mask.sh's record: its fixed values and its steps in order."""
    try:
        lines = path.read_text().splitlines()
    except OSError as e:
        raise Unreadable(f"the workqueue mask record {path}: {e}") from None
    fixed: dict = {}
    steps: list = []
    for ln in lines:
        k, sep, v = ln.partition("=")
        if not sep:
            raise Unreadable(f"{path}: not a record line: {ln!r}")
        if k in ("cpu", "original_cpumask", "original_requested", "original_isolated", "original_census", "target"):
            if k in fixed:
                raise Unreadable(f"{path}: {k} twice")
            fixed[k] = v
        elif k in ("started", "restore_owed", "set_write", "set_verified", "set_failed", "observed", "restore_write",
                   "restore_failed", "restored"):
            steps.append((k, v))
        else:
            raise Unreadable(f"{path}: unknown record line {k}")
    if "target" not in fixed or "cpu" not in fixed:
        raise Unreadable(f"{path}: no target")
    return {"fixed": fixed, "steps": steps}


def gate(csv_path: pathlib.Path, controls: pathlib.Path | None, wq: pathlib.Path, allow: set, mode: str | None) -> list:
    """(check, ok, detail) for each check; raises Unreadable for evidence that cannot be read."""
    out: list = []
    note = pathlib.Path(str(csv_path) + ".run.txt")
    try:
        text = note.read_text(errors="replace")
    except OSError as e:
        raise Unreadable(f"the run record {note}: {e}") from None
    summ = pathlib.Path(str(csv_path) + ".summary.toml")
    if tomllib is None:
        raise Unreadable("needs Python 3.11 or later (tomllib)")
    try:
        kind, data = timing_summary.load(csv_path)
        run = timing_summary.load_run_summary(summ, kind)
    except SystemExit as e:  # timing_summary reports unreadable logs this way
        raise Unreadable(str(e)) from None
    if run is None:
        raise Unreadable(f"no run summary {summ}")
    res = run.get("result", {})
    rate = float(run.get("rate_hz", 0) or 0)
    if not rate > 0:
        raise Unreadable(f"{summ}: no rate_hz")
    if mode is not None:
        m = re.search(r"^mode: (\S+)$", text, re.M)
        out.append(("the run's mode", bool(m and m[1] == mode), f"{m[1] if m else 'none'} (expected {mode})"))

    # Completeness and T3: timing_summary's own criteria, and the session's record of them.
    s = timing_summary.summarize(kind, data, rate, None, None, run)
    t3 = next(v for n, _, v, _ in timing_summary.check(s, False, DEFAULTS) if n == "T3")
    started_ok = (kind == "probe" and run.get("kind") == "production-traffic" and bool(run.get("log_complete"))
                  and res.get("started") is True and res.get("error", "") == "")
    out.append(("a complete production-traffic summary", started_ok,
                f"kind {run.get('kind')}, log complete {run.get('log_complete')}, started {res.get('started')}, "
                f"error {res.get('error', '')!r}"))
    out.append(("T3 (timing_summary.py)", t3 == "PASS", t3))
    rec_t3 = re.findall(r"^  T3\s+measured\s+(\S+)", text, re.M)
    verdicts = re.findall(r"^verdict: (.*)$", text, re.M)
    out.append(("T3 as the session recorded it", rec_t3 == ["PASS"], ", ".join(rec_t3) or "no T3 line"))
    failed: set = set()
    ok = len(verdicts) == 1
    if ok:
        v = verdicts[0].strip()
        fm = re.fullmatch(r"fail \(([A-Z0-9a-z,]+)\)", v)
        if fm:
            failed = set(fm[1].split(","))
        elif v != "pass":
            ok = False
    out.append(("the session's verdict, within the allowed failures", ok and failed <= allow,
                f"{verdicts or 'none'}; allowed {','.join(sorted(allow)) or 'none'}"))

    # The bounded read and the run's own counters.
    br = run.get("bounded_read")
    if not isinstance(br, dict) or any(k not in br for k in BOUNDED_ZERO) or any(k not in res for k in RESULT_ZERO):
        raise Unreadable(f"{summ}: the bounded read's or the run's counters are missing")
    nonzero = {k: br[k] for k in BOUNDED_ZERO if br[k] != 0} | {k: res[k] for k in RESULT_ZERO if res[k] != 0}
    out.append(("the bounded read's and the run's failure counters", not nonzero,
                ", ".join(f"{k} {v}" for k, v in nonzero.items()) or f"all zero over {br.get('reads')} reads"))

    # The emulator.
    cmd = re.search(r"^command: (.*)$", text, re.M)
    port = re.search(r"--port (\S+)", cmd[1]) if cmd else None
    tty = re.search(r"^host: tty (\S+?),", text, re.M)
    out.append(("the emulator's pty", bool(port and port[1].startswith("/tmp/") and tty and not tty[1].startswith("ttyUSB")),
                f"--port {port[1] if port else 'none'}, device {tty[1] if tty else 'none'}"))

    # The declared host condition.
    rec = read_wq(wq)
    target = mask_value(rec["fixed"]["target"])
    st = re.search(r"^started: (\S+)$", text, re.M)
    if not st:
        raise Unreadable(f"{note}: no start time")
    t_start = dt.datetime.fromisoformat(st[1]).timestamp()
    t_end = t_start + float(res.get("elapsed_s", 0))
    steps = rec["steps"]
    verified = [wq_time(v) for k, v in steps if k == "set_verified"]
    out.append(("the mask set and verified once, before the run, never failed",
                len(verified) == 1 and verified[0] < t_start and not any(k == "set_failed" for k, _ in steps),
                f"{len(verified)} verifications, {sum(k == 'set_failed' for k, _ in steps)} failures"))
    line = re.search(r"^unbound workqueues: cpumask (\S+?),", text, re.M)
    out.append(("the run's recorded mask is the target", bool(line) and mask_value(line[1]) == target,
                f"{line[1] if line else 'none'} (target {rec['fixed']['target']})"))
    obs = [(wq_time(v), v.endswith(" matches")) for k, v in steps if k == "observed"]
    out.append(("observed in effect after the run, never otherwise", any(ok_ and t >= t_end for t, ok_ in obs)
                and all(ok_ for _, ok_ in obs), f"{len(obs)} observations, {sum(not o for _, o in obs)} not in effect"))
    restored = [wq_time(v) for k, v in steps if k == "restored"]
    if restored:
        out.append(("restored after the run", all(t >= t_end for t in restored), f"{len(restored)} restorations"))

    if controls is not None:
        try:
            ctext = controls.read_text(errors="replace")
        except OSError as e:
            raise Unreadable(f"the controls' record {controls}: {e}") from None
        cv = re.findall(r"^verdict: (.*)$", ctext, re.M)
        out.append(("the fair-server controls", cv == ["controls pass"], ", ".join(cv) or "no verdict"))
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("csv", type=pathlib.Path, help="the run's probe log (<run>.csv)")
    p.add_argument("--controls", type=pathlib.Path, help="the fair-server controls' run record")
    p.add_argument("--wq-record", type=pathlib.Path, help="tools/wq_mask.sh's record (default: the open one)")
    p.add_argument("--allow", default="T1,T2,T6a", help="criteria the session may fail (default: T1,T2,T6a)")
    p.add_argument("--mode", choices=("untraced", "timer"), help="the run's expected mode")
    a = p.parse_args()
    wq = a.wq_record or pathlib.Path(os.environ.get("XDG_STATE_HOME", pathlib.Path.home() / ".local/state")) / "rtctrl-wq-mask.record"
    allow = {x for x in a.allow.split(",") if x}
    try:
        checks = gate(a.csv, a.controls, wq, allow, a.mode)
    except Unreadable as e:
        print(f"gate: UNREADABLE: {e}")
        return 2
    for name, ok, detail in checks:
        print(f"gate: {'PASS' if ok else 'FAIL'}: {name}: {detail}")
    bad = [n for n, ok, _ in checks if not ok]
    print(f"gate: {'PASS' if not bad else 'FAIL'} ({a.csv.name}: {len(checks) - len(bad)} of {len(checks)} checks)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
