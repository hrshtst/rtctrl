#!/usr/bin/env python3
"""The loaded emulator validation's gate on one finished run (docs/hardware/control-rate.md, Option A), run by its
supervisor (tools/option_a.sh): between the untraced and the timer invocations, so that a failed T3 stops the experiment
before the timer run (the session itself treats a failed criterion as data and goes on), and on the timer run; and again
after the experiment, on the closed records. Read-only.

    validation_gate.py <run>.csv --wq-record FILE --experiment FILE --session A|B --record-state open|closed
                       [--controls fs-controls-<date>-<attempt>.run.txt] [--allow T1,T2,T6a] [--mode untraced|timer]
                       [--cpu 3]

It passes only if every check passes:

- completeness and T3, decided twice and agreeing: by tools/timing_summary.py's own criteria on the probe log and its
  summary (a complete summary of a production-traffic run that started, with no error), and as the session recorded
  them in <run>.csv.run.txt (its T3 line PASS, its verdict failing no criterion outside --allow: T1, T2 and T6a, the
  emulator's anticipated failures);
- the bounded read's counters all zero: timeouts, malformed replies, servo errors, recoveries, guard waits and
  inhibited sends; and the run's read failures, write failures and ring drops;
- the emulator: the probe was given a --port under /tmp and its device line names no USB serial adapter;
- the run's bounds: the supervisor's experiment record holds the session's start and end once each, the end after the
  start, with status 0; the probe ran inside them (the session started before it and returned after it), so a time
  before the start is before the run and a time after the end is after it;
- the declared host condition, tools/wq_mask.sh's record read as a checked sequence:
  - its fixed values, each once and in order (started, cpu, the original effective, requested and isolated masks, the
    original census, the target): the CPU is --cpu, the effective and requested originals are equal and hold it, the
    target is exactly the original less that CPU, the census is complete with at least one worker;
  - its first steps: the restoration owed, the write (status 0), the verification (both masks the target, at least one
    worker, none allowed on the CPU, none unknown), no failed set anywhere, the verification before the session started;
  - then only observations, each of both masks equal to the target, at least one after the session ended;
  - open (between the runs): nothing after them; closed (after the experiment): exactly one restoration write (status
    0), after the last session recorded ended, and the restoration (both masks their originals, a complete census, no
    more workers barred from the CPU than at the start), with no failed restoration; times never going back;
  - the run record's unbound-workqueue mask equal to the target;
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


FIXED = ("started", "cpu", "original_cpumask", "original_requested", "original_isolated", "original_census", "target")
STEPS = ("restore_owed", "set_write", "set_verified", "set_failed", "observed", "restore_write", "restore_failed", "restored")


def fields(v: str) -> tuple[float, dict]:
    """A step's time and its key=value fields (a reason= field takes the rest of the line)."""
    t = wq_time(v)
    rest = v.split(None, 1)[1] if " " in v else ""
    out: dict = {}
    while rest:
        k, sep, tail = rest.partition("=")
        if not sep or not re.fullmatch(r"[a-z_]+", k):
            raise Unreadable(f"not a key=value field: {rest!r}")
        if k == "reason":
            out[k], rest = tail, ""
        else:
            val, _, rest = tail.partition(" ")
            out[k] = val
    return t, out


def read_wq(path: pathlib.Path) -> dict:
    """tools/wq_mask.sh's record: its fixed values (each once, in order, at the top) and its steps in order."""
    try:
        lines = path.read_text().splitlines()
    except OSError as e:
        raise Unreadable(f"the workqueue mask record {path}: {e}") from None
    keys = [ln.partition("=")[0] for ln in lines]
    if any("=" not in ln for ln in lines):
        raise Unreadable(f"{path}: a line that is not key=value")
    if tuple(keys[:len(FIXED)]) != FIXED or any(k in FIXED for k in keys[len(FIXED):]):
        raise Unreadable(f"{path}: the fixed values are not each once, in order, at the top")
    if any(k not in STEPS for k in keys[len(FIXED):]):
        raise Unreadable(f"{path}: an unknown record line")
    fixed = {k: ln.partition("=")[2] for k, ln in zip(FIXED, lines)}
    steps = [(ln.partition("=")[0], *fields(ln.partition("=")[2])) for ln in lines[len(FIXED):]]
    return {"fixed": fixed, "started": wq_time(fixed["started"]), "steps": steps}


def read_experiment(path: pathlib.Path) -> dict:
    """The supervisor's experiment record: each NAME_start and NAME_end (with its status)."""
    try:
        lines = path.read_text().splitlines()
    except OSError as e:
        raise Unreadable(f"the experiment record {path}: {e}") from None
    out: dict = {}
    for ln in lines:
        k, sep, v = ln.partition("=")
        m = re.fullmatch(r"(session_[AB])_(start|end)", k)
        if not sep or not m:
            continue
        if k in out:
            raise Unreadable(f"{path}: {k} twice")
        t, f = fields(v)
        out[k] = (t, f.get("status"))
    return out


def mask_checks(rec: dict, cpu: int, bounds: tuple, last_end: float, state: str) -> list:
    """The checked sequence of tools/wq_mask.sh's record against the run's bounds; (check, ok, detail) each."""
    out: list = []
    fx = rec["fixed"]
    try:
        orig, req, target = mask_value(fx["original_cpumask"]), mask_value(fx["original_requested"]), mask_value(fx["target"])
        mask_value(fx["original_isolated"])
        census = re.fullmatch(r"workers=(\d+) allowed=(\d+) unknown=0", fx["original_census"])
        rcpu = int(fx["cpu"]) if fx["cpu"].isdigit() else -1
    except Unreadable as e:
        out.append(("the mask record's fixed values", False, str(e)))
        return out
    n0, a0 = (int(census[1]), int(census[2])) if census else (0, 0)
    ok = (rcpu == cpu and orig == req and orig >> cpu & 1 == 1 and target == orig & ~(1 << cpu) and census is not None
          and n0 >= 1 and a0 <= n0)
    out.append(("the mask record's fixed values", ok, f"cpu {fx['cpu']} (expected {cpu}), original {fx['original_cpumask']} "
                f"(requested {fx['original_requested']}), target {fx['target']}, census {fx['original_census']}"))
    steps = rec["steps"]
    times = [rec["started"]] + [s[1] for s in steps]
    out.append(("the mask record's times never go back", all(a <= b for a, b in zip(times, times[1:])), f"{len(steps)} steps"))
    start, end = bounds
    head = [s[0] for s in steps[:3]]
    ok = head == ["restore_owed", "set_write", "set_verified"] and not any(s[0] == "set_failed" for s in steps)
    if ok:
        w, v = steps[1][2], steps[2][2]
        ok = (w.get("status") == "0" and v.get("cpumask") is not None and v.get("requested") is not None
              and mask_value(v["cpumask"]) == target and mask_value(v["requested"]) == target and v.get("workers", "0").isdigit()
              and int(v["workers"]) >= 1 and v.get("allowed") == "0" and v.get("unknown") == "0" and steps[2][1] < start)
    out.append(("the mask set and verified before the session started, never failed", ok,
                " ".join(f"{k}{(' ' + ' '.join(f'{a}={b}' for a, b in f.items())) if f else ''}" for k, _, f in steps[:3])))
    rest = steps[3:]
    obs = []
    while rest and rest[0][0] == "observed":
        obs.append(rest.pop(0))

    def matches(f: dict) -> bool:
        return (f.get("result") == "matches" and "cpumask" in f and "requested" in f
                and mask_value(f["cpumask"]) == target and mask_value(f["requested"]) == target)
    out.append(("the target observed in effect after the session ended, never otherwise",
                bool(obs) and all(matches(f) for _, _, f in obs) and any(t >= end for _, t, _ in obs),
                f"{len(obs)} observations, {sum(not matches(f) for _, _, f in obs)} not of the target"))
    if state == "open":
        out.append(("the record open, no restoration begun", not rest, ", ".join(s[0] for s in rest) or "nothing after the observations"))
    else:
        ok = [s[0] for s in rest] == ["restore_write", "restored"]
        if ok:
            w, r = rest[0], rest[1]
            barred0 = n0 - a0
            ok = (w[2].get("status") == "0" and w[1] >= last_end and "cpumask" in r[2] and "requested" in r[2]
                  and mask_value(r[2]["cpumask"]) == orig and mask_value(r[2]["requested"]) == req
                  and r[2].get("unknown") == "0" and r[2].get("workers", "").isdigit() and r[2].get("allowed", "").isdigit()
                  and int(r[2]["workers"]) >= 1 and int(r[2]["workers"]) - int(r[2]["allowed"]) <= barred0
                  and r[2].get("context") == "restore")
        out.append(("restored after the last session, verified", ok, ", ".join(s[0] for s in rest) or "no restoration"))
    return out


def gate(csv_path: pathlib.Path, controls: pathlib.Path | None, wq: pathlib.Path, allow: set, mode: str | None,
         experiment: pathlib.Path, session: str, state: str, cpu: int = 3) -> list:
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

    # The run's bounds, from the supervisor's record: the probe ran inside its session.
    exp = read_experiment(experiment)
    s0, s1 = exp.get(f"session_{session}_start"), exp.get(f"session_{session}_end")
    ok = s0 is not None and s1 is not None and s1[1] == "0" and s0[0] < s1[0]
    out.append(("the session's bounds in the experiment record", ok,
                f"start {'recorded' if s0 else 'none'}, end {'status ' + str(s1[1]) if s1 else 'none'}"))
    bounds = (s0[0], s1[0]) if ok else (float("-inf"), float("inf"))  # without bounds, no time is before or after the run
    last_end = max((v[0] for k, v in exp.items() if k.endswith("_end")), default=float("inf"))

    # The declared host condition.
    rec = read_wq(wq)
    out += mask_checks(rec, cpu, bounds, last_end, state)
    line = re.search(r"^unbound workqueues: cpumask (\S+?),", text, re.M)
    out.append(("the run's recorded mask is the target", bool(line) and mask_value(line[1]) == mask_value(rec["fixed"]["target"]),
                f"{line[1] if line else 'none'} (target {rec['fixed']['target']})"))

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
    p.add_argument("--experiment", type=pathlib.Path, required=True, help="the supervisor's experiment record")
    p.add_argument("--session", choices=("A", "B"), required=True, help="the run's session in the experiment record")
    p.add_argument("--record-state", choices=("open", "closed"), required=True, help="open between the runs, closed after")
    p.add_argument("--cpu", type=int, default=3, help="the CPU cleared from the mask (default: 3)")
    p.add_argument("--allow", default="T1,T2,T6a", help="criteria the session may fail (default: T1,T2,T6a)")
    p.add_argument("--mode", choices=("untraced", "timer"), help="the run's expected mode")
    a = p.parse_args()
    wq = a.wq_record or pathlib.Path(os.environ.get("XDG_STATE_HOME", pathlib.Path.home() / ".local/state")) / "rtctrl-wq-mask.record"
    allow = {x for x in a.allow.split(",") if x}
    try:
        checks = gate(a.csv, a.controls, wq, allow, a.mode, a.experiment, a.session, a.record_state, a.cpu)
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
