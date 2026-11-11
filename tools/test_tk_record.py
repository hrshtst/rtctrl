#!/usr/bin/env python3
"""Regressions for the timekeeping record: tools/tk_record.py's parsers and
state read, its evidence through tools/timer_clock.py's certificate (a record
with any part missing, empty, truncated or contrary is never certified), and
tools/tk_recorder.sh on a scratch tree.

For the recorder, BASH_ENV loads functions standing in for the kernel's side
of tracefs: they log every write with its target, in order, make and remove
the event directories a definition implies, and give each write the failure a
test asks for. So the script itself runs, in its real order, without tracefs.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import timer_clock  # noqa: E402
import tk_record as tk  # noqa: E402

PROBES = ("ntp_adjtimex", "ntp_clear", "update_entry", "update_return")
ARMED, S0, S1 = 1_000_000_000, 1_500_000_000, 3_500_000_000
CONFIRMED = S1 + 500_000_000
TID = 777
# The audited trace-cmd as tools/tk_record.py tool records it, with its audited runtime.
AUDITED_TOOL = {"read_status": "ok", "path": "/usr/bin/trace-cmd", **timer_clock.AUDITED_TRACE_CMD,
                "runtime": {**copy.deepcopy(timer_clock.AUDITED_TRACE_RUNTIME),
                            "loaded": {"linux-vdso.so.1": None, "libtraceevent.so.1": "/usr/lib/x86_64-linux-gnu/libtraceevent.so.1",
                                       "libtracefs.so.1": "/usr/lib/x86_64-linux-gnu/libtracefs.so.1", "libz.so.1": "/usr/lib/x86_64-linux-gnu/libz.so.1",
                                       "libzstd.so.1": "/usr/lib/x86_64-linux-gnu/libzstd.so.1", "libc.so.6": "/usr/lib/x86_64-linux-gnu/libc.so.6",
                                       "/lib64/ld-linux-x86-64.so.2": "/lib64/ld-linux-x86-64.so.2"}}}


def stamp(t: int) -> str:
    return f"{t // 10**9}.{t % 10**9:09d}"


def adjtimex_rows(t: int, tid: int = TID) -> list[str]:
    """A state read's two calls, as trace-cmd report -t prints the probe."""
    return [f"       tk_record-{tid}   [001] d..1.  {stamp(t + 10_000)}: ntp_adjtimex:         (ntp_adjtimex+0x4/0x3f0) tkid=0 modes=0",
            f"       tk_record-{tid}   [001] d..1.  {stamp(t + 20_000)}: ntp_adjtimex:         (ntp_adjtimex+0x4/0x3f0) tkid=0 modes=40961"]


def update_rows(t: int, cpu: int = 1, took: int = 3000) -> list[str]:
    return [f"          <idle>-0     [{cpu:03d}] d.h1.  {stamp(t)}: update_entry:         (update_wall_time+0x4/0x110)",
            f"          <idle>-0     [{cpu:03d}] d.h1.  {stamp(t + took)}: update_return:        (tick_do_update_jiffies64+0xc7/0x100 <- update_wall_time)"]


def write_record(base, armed: int = ARMED, s0: int = S0, s1: int = S1, confirmed: int | None = None, tick: int = 10000,
                 rows_before: tuple = (), rows_after: tuple = (), hits: dict | None = None) -> None:
    """A complete record of a capture, the files as the session leaves them."""
    confirmed = s1 + 500_000_000 if confirmed is None else confirmed
    o0, tc = -37693, 7
    o1 = float(o0)
    for _ in range(round((s1 - s0) / 1e9)):
        o1 -= o1 / 2 ** (2 + tc)
    idn = {"release": "7.0.0-42-generic", "cmdline": "BOOT_IMAGE=/boot/vmlinuz root=UUID=x ro quiet splash",
           "config_sha256": timer_clock.AUDITED["config_sha256"], "clocksource": "tsc", "cmdline_status": "ok",
           "clocksource_status": "ok", "config_status": "ok"}

    def state(t: int, off: int) -> dict:
        return {"read_status": "ok", "t_start": t, "t_end": t + 50_000, "tid": TID, "ret": 0, "offset": off,
                "freq": 1646056, "status": 0x2001, "constant": tc, "tick": tick, "time_adjust": 0, **idn}

    b = str(base)
    pathlib.Path(b + ".tk-before.json").write_text(json.dumps(state(s0, o0)))
    pathlib.Path(b + ".tk-after.json").write_text(json.dumps(state(s1, round(o1))))
    pathlib.Path(b + ".tk-log.json").write_text(json.dumps({"read_status": "ok", "covers": [0, confirmed], "clock_events": [],
                                                            "tsc_khz": 3187200}))
    pathlib.Path(b + ".tk-times.txt").write_text(f"armed_t {armed}\nconfirmed_t {confirmed}\n")
    rows = ["cpus=32", "CPU 5 is empty", *rows_before, *adjtimex_rows(s0), *adjtimex_rows(s1)]
    updates = list(range(armed + 500_000, s1 + 5_000_000, 1_000_000))
    for t in updates:
        rows += update_rows(t)
    rows += rows_after
    pathlib.Path(b + ".tk.txt").write_text("\n".join(rows) + "\n")
    states = "".join(f"{p} 1\n" for p in PROBES)
    pathlib.Path(b + ".tk-enable-before.txt").write_text(states)
    pathlib.Path(b + ".tk-enable-after.txt").write_text(states)
    parsed = tk.parse_record("\n".join(rows))
    n = {"ntp_adjtimex": sum(c["name"] == "ntp_adjtimex" for c in parsed["calls"]),
         "ntp_clear": sum(c["name"] == "ntp_clear" for c in parsed["calls"]),
         "update_entry": len(parsed["updates"]) + len(parsed["unmatched"]["entries"]),
         "update_return": len(parsed["updates"]) + len(parsed["unmatched"]["returns"])}
    n.update(hits or {})
    pathlib.Path(b + ".tk-profile.txt").write_text("".join(f"{p} {n[p]} 0\n" for p in PROBES))
    pathlib.Path(b + ".tk-stats.txt").write_text(stats_text(2))
    pathlib.Path(b + ".tk-tool-before.json").write_text(json.dumps(AUDITED_TOOL))
    pathlib.Path(b + ".tk-tool-after.json").write_text(json.dumps(AUDITED_TOOL))


def stats_text(cpus: int, overrun: int = 0) -> str:
    return "".join(f"CPU: {c}\nentries: 1\noverrun: {overrun if c == 0 else 0}\ncommit overrun: 0\nbytes: 1\n"
                   "oldest event ts: 1.0\nnow ts: 2.0\ndropped events: 0\nread events: 1\n" for c in range(cpus)) + f"cpus: {cpus}\n"


FAILURES: list[str] = []


def expect(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}")


def parsers() -> None:
    r = tk.parse_record("\n".join(["cpus=32", "CPU 0 is empty", "", *adjtimex_rows(S0), *update_rows(ARMED)]))
    expect(r["read_status"] == "ok" and len(r["calls"]) == 2 and r["calls"][1]["modes"] == 0xa001 and r["calls"][0]["tid"] == TID
           and r["calls"][0]["t"] == S0 + 10_000 and r["updates"] == [{"entry": ARMED, "ret": ARMED + 3000, "cpu": 1}],
           f"a plain report: {r}")
    p = tk.parse_record("\n".join(f"  rtctrl_tk: {x.strip()}" for x in update_rows(ARMED)))
    expect(p["read_status"] == "ok" and len(p["updates"]) == 1, f"rows prefixed by the instance's name: {p}")
    r = tk.parse_record(f"  other_inst: {update_rows(ARMED)[0].strip()}")
    expect("not a nanosecond record" in r["read_status"], f"another instance's rows: {r['read_status']}")
    r = tk.parse_record("          <idle>-0     [001] d.h1.  1.000100: update_entry: (update_wall_time+0x4/0x110)")
    expect("not a nanosecond record" in r["read_status"], f"microsecond stamps: {r['read_status']}")
    r = tk.parse_record(f"          <idle>-0     [001] d.h1.  {stamp(ARMED)}: sched_switch: x")
    expect("not a nanosecond record" in r["read_status"], f"an event of something else: {r['read_status']}")
    r = tk.parse_record(f" tk_record-7 [001] d..1. {stamp(S0)}: ntp_adjtimex: (x) tkid=0")
    expect("unreadable arguments" in r["read_status"], f"no modes: {r['read_status']}")
    r = tk.parse_record(f" tk_record-7 [001] d..1. {stamp(S0)}: ntp_adjtimex: (x) tkid=q modes=0")
    expect("unreadable arguments" in r["read_status"], f"an unreadable id: {r['read_status']}")
    r = tk.parse_record(f" tk_record-7 [001] d..1. {stamp(S0)}: ntp_clear: (x) tkid=0x0")
    expect(r["read_status"] == "ok" and r["calls"][0]["name"] == "ntp_clear", f"ntp_clear needs no modes, hex ids read: {r}")
    # Boundaries, per CPU: a return with no entry, an entry with no return, and an entry that replaces an unreturned one.
    rows = [update_rows(100, took=10)[1], *update_rows(1000, took=10), update_rows(2000, took=10)[0], *update_rows(3000, took=10),
            update_rows(5000, took=10)[0]]
    r = tk.parse_record("\n".join(rows))
    expect(r["unmatched"]["entries"] == [2000, 5000], f"unmatched entries: {r['unmatched']}")
    expect(r["unmatched"]["returns"] == [110], f"unmatched returns: {r['unmatched']}")
    expect([u["entry"] for u in r["updates"]] == [1000, 3000], f"the complete updates: {r['updates']}")
    # Two CPUs interleaved stay separate; equal stamps keep the file's order.
    r = tk.parse_record("\n".join([*update_rows(1000, cpu=1), *update_rows(1500, cpu=2), *update_rows(9000, cpu=3, took=0)]))
    expect(len(r["updates"]) == 3 and not r["unmatched"]["entries"] and not r["unmatched"]["returns"], f"interleaved CPUs: {r}")

    states = "ntp_adjtimex 1\nntp_clear 1\nupdate_return 1\nupdate_entry 0\n"
    expect(tk.parse_states(states) == ({"ntp_adjtimex": True, "ntp_clear": True, "update_return": True, "update_entry": False}, "ok"), "states")
    for what, text in (("an unknown probe", states + "nope 1\n"), ("no state", states.replace("ntp_clear 1", "ntp_clear")),
                       ("a probe missing", states.replace("ntp_clear 1\n", "")), ("nothing", ""), ("soft mode", states.replace("ntp_clear 1", "ntp_clear 1*")),
                       ("a value other than 0 or 1", states.replace("ntp_clear 1", "ntp_clear 2")), ("extra spacing", states.replace("ntp_clear 1", "ntp_clear  1")),
                       ("a blank row", states + "\n"), ("a repeated row", states + "ntp_clear 1\n"), ("a contrary row after", states + "ntp_clear 0\n"),
                       ("a contrary row before", "ntp_clear 0\n" + states)):
        got, st = tk.parse_states(text)
        expect(st != "ok" and got == {}, f"states with {what}: {got} {st}")
    prof = "ntp_adjtimex 4 0\nntp_clear 3 0\nupdate_return 9 0\nupdate_entry 9 0\n"
    expect(tk.parse_profile(prof)[0]["ntp_clear"] == (3, 0), "profile")
    for what, text in (("a count that is not a number", prof.replace("ntp_clear 3 0", "ntp_clear x 0")), ("an unknown probe", prof + "nope 1 0\n"),
                       ("a probe missing", prof.replace("ntp_clear 3 0\n", "")), ("a missing probe's row from the helper", prof.replace("ntp_clear 3 0", "ntp_clear missing")),
                       ("a negative count", prof.replace("ntp_clear 3 0", "ntp_clear -3 0")), ("a repeated row", prof + "ntp_clear 3 0\n"),
                       ("a contrary row after", prof + "ntp_clear 3 1\n"), ("a contrary row before", "ntp_clear 3 1\n" + prof)):
        got, st = tk.parse_profile(text)
        expect(st != "ok" and got == {}, f"profile with {what}: {got} {st}")
    times = "armed_t 5\nconfirmed_t 9\n"
    expect(tk.parse_times(times) == ({"armed_t": 5, "confirmed_t": 9}, "ok"), "times")
    for what, text in (("no confirmation", "armed_t 5\n"), ("no arming", "confirmed_t 9\n"), ("the wrong order", "confirmed_t 9\narmed_t 5\n"),
                       ("an unreadable arming before", "armed_t unreadable\n" + times), ("an unreadable arming after", times + "armed_t unreadable\n"),
                       ("a repeated arming", "armed_t 5\n" + times), ("a contrary arming", times + "armed_t 6\n"), ("a repeated confirmation", times + "confirmed_t 9\n"),
                       ("another key", times + "started_t 1\n"), ("a blank row", times + "\n"), ("nothing", "")):
        got, st = tk.parse_times(text)
        expect(st != "ok" and got == {}, f"times with {what}: {got} {st}")

    got, st = tk.parse_stats(stats_text(3, overrun=4))
    expect(st == "ok" and got == {"read_status": "ok", "overrun": 4, "dropped": 0}, f"stats: {got} {st}")
    for what, text in (("no trailer", "CPU: 0\noverrun: 0\ncommit overrun: 0\ndropped events: 0\n"),
                       ("a count that disagrees", stats_text(2).replace("cpus: 2", "cpus: 3")),
                       ("a CPU cut short", "CPU: 0\noverrun: 0\ncpus: 1\n"),
                       ("a counter that is not a number", stats_text(1).replace("overrun: 0\ncommit", "overrun: n/a\ncommit", 1)),
                       ("a CPU twice", stats_text(1).replace("cpus: 1\n", "") + stats_text(1)),
                       ("a contrary overrun after", stats_text(1).replace("bytes: 1", "bytes: 1\noverrun: 5")),
                       ("a contrary overrun before", stats_text(1).replace("entries: 1", "entries: 1\noverrun: 5")),
                       ("a repeated count", stats_text(1).replace("bytes: 1", "bytes: 1\ndropped events: 0")),
                       ("a count before any CPU", "overrun: 5\n" + stats_text(1)), ("a row after the CPU count", stats_text(1) + "overrun: 5\n"),
                       ("a second CPU count", stats_text(1) + "cpus: 1\n"), ("an unknown row", stats_text(1).replace("bytes: 1", "bytes: 1\nlost: 3")),
                       ("a blank row", stats_text(1).replace("bytes: 1", "bytes: 1\n")), ("nothing", "")):
        expect(tk.parse_stats(text)[1] != "ok", f"stats with {what}")
    # The kernel's own rows, as the capability run's idle host printed them under the mono clock.
    kernel = ("CPU: 0\nentries: 98\noverrun: 0\ncommit overrun: 0\nbytes: 2400\noldest event ts: 24583.510045\nnow ts: 24586.922114\n"
              "dropped events: 0\nread events: 0\ncpus: 1\n")
    expect(tk.parse_stats(kernel) == ({"read_status": "ok", "overrun": 0, "dropped": 0}, "ok"), f"the kernel's stats: {tk.parse_stats(kernel)}")
    with tempfile.TemporaryDirectory() as d:
        for what, text in (("a key twice, the second winning", '{"freq": 1, "freq": 0}'), ("a key twice, equal", '{"freq": 0, "freq": 0}'),
                           ("NaN", '{"freq": NaN}'), ("Infinity", '{"freq": -Infinity}')):
            f = pathlib.Path(d, "x.json")
            f.write_text(text)
            v, st = tk.read_json(str(f))
            expect(v is None and "not JSON" in st, f"JSON with {what}: {v} {st}")

    log = ("[    1.000000] host kernel: tsc: Detected 3187.200 MHz TSC\n"
           "[   10.000000] host kernel: clocksource: Switched to clocksource tsc\n"
           "[  100.000000] host kernel: usb 1-9: new device\n"
           "[  104.000000] host kernel: clocksource: timekeeping watchdog on CPU3\n"
           "not a log row\n"
           "[  300.000000] host kernel: clocksource: far after\n")
    r = tk.scan_log(log, 105_000_000_000, 200_000_000_000)
    expect(r["read_status"] == "ok" and r["tsc_khz"] == 3187200 and r["covers"] == [1_000_000_000, 200_000_000_000]
           and len(r["clock_events"]) == 1 and "watchdog" in r["clock_events"][0], f"the log's scan: {r}")
    r = tk.scan_log("[    1.000000] host kernel: tsc: Refined TSC clocksource calibration: 3187.199 MHz\n", 5 * 10**10, 6 * 10**10)
    expect(r["tsc_khz"] == 3187199, f"the refined rate: {r}")
    expect(tk.scan_log("no rows\n", 0, 1)["read_status"] != "ok", "a log with no rows")


def state_reads() -> None:
    with tempfile.TemporaryDirectory() as d:
        s = tk.state_read(config_dir=d)
        expect(s["read_status"] == "ok" and s["t_start"] <= s["t_end"] and s["tid"] == threading.get_native_id()
               and isinstance(s["freq"], int) and s["tick"] > 0, f"a state read: {s}")
        expect(s["config_sha256"] is None and s["config_status"] != "ok", f"no config: {s['config_status']}")
        s = tk.state_read(cmdline_path=f"{d}/nope", clocksource_path=f"{d}/nope")
        expect(s["cmdline"] is None and s["cmdline_status"] != "ok" and s["clocksource"] is None
               and s["clocksource_status"] != "ok", f"unreadable provenance: {s}")
        c = pathlib.Path(d, "cmdline")
        c.write_bytes(b'root=x "ntp_tick_adj=1"\xff\n')
        s = tk.state_read(cmdline_path=str(c))
        expect(s["cmdline"] == 'root=x "ntp_tick_adj=1"\xff', f"the command line kept byte for byte: {s['cmdline']!r}")
        out = pathlib.Path(d, "s.json")
        p = subprocess.run([sys.executable, "tools/tk_record.py", "state", "--out", str(out), "--config-dir", d], capture_output=True, text=True)
        expect(p.returncode == 0 and json.loads(out.read_text())["read_status"] == "ok", f"the CLI: {p.stderr}")
        p = subprocess.run([sys.executable, "tools/tk_record.py", "log", "--journal", f"{d}/nope", "--from-t", "1", "--to-t", "2", "--out", str(out)],
                           capture_output=True, text=True)
        expect(p.returncode == 1 and json.loads(out.read_text())["read_status"] != "ok", f"an unreadable journal: {p.stderr}")


# A stand-in for dpkg-query, answering as dpkg does: TK_DPKG_FILES names each installed file's package ("file=package"
# words), TK_DPKG_PKGS each installed package's version ("package=version"), TK_DPKG_STATUS trace-cmd's status.
DPKG_STANDIN = r"""#!/usr/bin/env bash
case "$1" in
  -S) shift; rc=0
      for f in "$@"; do
        p=; for m in ${TK_DPKG_FILES:-}; do [ "${m%%=*}" = "$f" ] && p=${m#*=}; done
        if [ -n "$p" ]; then echo "$p: $f"; else echo "dpkg-query: no path found matching pattern $f" >&2; rc=1; fi
      done
      exit $rc ;;
  -W) shift 3; rc=0
      for p in "$@"; do
        v=; for m in ${TK_DPKG_PKGS:-}; do [ "${m%%=*}" = "$p" ] && v=${m#*=}; done
        st="ii "; [ "$p" = trace-cmd ] && st=${TK_DPKG_STATUS:-ii }
        if [ -n "$v" ]; then printf '%s\t%s\tamd64\t%s\n' "$p" "$v" "$st"; else echo "dpkg-query: no packages found matching $p" >&2; rc=1; fi
      done
      exit $rc ;;
  *) exit 2 ;;
esac
"""
# A stand-in for the dynamic loader's --list: TK_LD_ROWS is the list it prints, TK_LD_STATUS its exit status.
LD_STANDIN = '#!/bin/sh\ncat "$TK_LD_ROWS"\nexit "${TK_LD_STATUS:-0}"\n'


def sha256(path) -> str:
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def standin_runtime(d: pathlib.Path, exe_text: str = "#!/bin/sh\necho stand-in trace-cmd\n") -> dict:
    """A stand-in trace-cmd installation under D: the executable (EXE_TEXT, linked from D/bin), its two trace libraries
    (libtraceevent naming its compiled-in plugin directory, as the real one does), two plugins and a file that is not one,
    and a loader list and a dpkg-query that answer for them. Returns the environment that selects the stand-ins (the
    production code reads them as it reads the host), and the files."""
    d = pathlib.Path(os.path.realpath(d))
    bindir, pkg, lib = d / "bin", d / "pkg", d / "lib"
    plug = lib / "traceevent" / "plugins"
    for x in (bindir, pkg, plug):
        x.mkdir(parents=True)
    exe = pkg / "trace-cmd"
    exe.write_text(exe_text)
    exe.chmod(0o755)
    (bindir / "trace-cmd").symlink_to(exe)
    te, tf = lib / "libtraceevent.so.1.8.7", lib / "libtracefs.so.1.8.2"
    te.write_bytes(b"stand-in libtraceevent\x00" + str(plug).encode() + b"\x00.local/lib/traceevent/plugins/\x00")
    tf.write_bytes(b"stand-in libtracefs\x00")
    (lib / "libtraceevent.so.1").symlink_to(te.name)
    (lib / "libtracefs.so.1").symlink_to(tf.name)
    plugins = [plug / "plugin_hrtimer.so", plug / "plugin_sched_switch.so"]
    for x in plugins:
        x.write_bytes(f"stand-in {x.name}".encode())
    (plug / "README").write_text("not a plugin\n")
    rows = d / "ld-rows.txt"
    rows.write_text(f"\tlinux-vdso.so.1 (0x00007ffc00000000)\n\tlibtraceevent.so.1 => {lib}/libtraceevent.so.1 (0x00007f0000001000)\n"
                    f"\tlibtracefs.so.1 => {lib}/libtracefs.so.1 (0x00007f0000002000)\n"
                    "\tlibc.so.6 => /usr/lib/x86_64-linux-gnu/libc.so.6 (0x00007f0000003000)\n\t/lib64/ld-linux-x86-64.so.2 (0x00007f0000004000)\n")
    ldl, dq = d / "ld-list", d / "dpkg-query"
    ldl.write_text(LD_STANDIN)
    dq.write_text(DPKG_STANDIN)
    for x in (ldl, dq):
        x.chmod(0o755)
    owners = {exe: "trace-cmd", te: "libtraceevent1", tf: "libtracefs1", **{x: "libtraceevent1-plugin" for x in plugins}}
    env = {"DPKG_QUERY": str(dq), "TK_LD_LIST": str(ldl), "TK_LD_ROWS": str(rows),
           "TK_DPKG_FILES": " ".join(f"{f}={o}" for f, o in owners.items()),
           "TK_DPKG_PKGS": "trace-cmd=3.3.3-1ubuntu2 libtraceevent1=1:1.8.7-1 libtracefs1=1.8.2-1ubuntu1 libtraceevent1-plugin=1:1.8.7-1"}
    return dict(env=env, link=bindir / "trace-cmd", exe=exe, te=te, tf=tf, lib=lib, plugin_dir=plug, plugins=plugins, rows=rows, bindir=bindir)


def identities() -> None:
    with tempfile.TemporaryDirectory() as d:
        s = standin_runtime(pathlib.Path(d, "s"))
        no_preload = str(pathlib.Path(d, "no-ld.so.preload"))
        saved = dict(os.environ)
        # Selection inputs in the caller's environment: the launch removes them, so the identity does not see them.
        inputs = {"HOME": "/home/someone", "TRACEEVENT_PLUGIN_DIR": "/opt/unreviewed/plugins", "TRACECMD_PLUGIN_DIR": "/opt/x",
                  "LD_LIBRARY_PATH": "/opt/unreviewed/lib", "LD_TK_STANDIN": "1"}

        def ident(name: str = str(s["link"]), preload: str = no_preload, **env: str) -> dict:
            os.environ.clear()
            os.environ.update({**saved, **inputs, **s["env"], **env})
            try:
                return tk.tool_identity(name, preload_file=preload)
            finally:
                os.environ.clear()
                os.environ.update(saved)

        def entry(f, pkg) -> dict:
            return {"path": str(f), "sha256": sha256(f), "package": pkg, "version": "1:1.8.7-1" if pkg != "libtracefs1" else "1.8.2-1ubuntu1",
                    "architecture": "amd64"}

        lib = s["lib"]
        want_rt = {"inputs": {"HOME": None, "TRACEEVENT_PLUGIN_DIR": None, "TRACECMD_PLUGIN_DIR": None, "LD": []}, "preload": "none",
                   "loaded": {"linux-vdso.so.1": None, "libtraceevent.so.1": f"{lib}/libtraceevent.so.1", "libtracefs.so.1": f"{lib}/libtracefs.so.1",
                              "libc.so.6": "/usr/lib/x86_64-linux-gnu/libc.so.6", "/lib64/ld-linux-x86-64.so.2": "/lib64/ld-linux-x86-64.so.2"},
                   "libraries": {"libtraceevent.so.1": entry(s["te"], "libtraceevent1"), "libtracefs.so.1": entry(s["tf"], "libtracefs1")},
                   "plugin_dirs": [str(s["plugin_dir"])],
                   "plugins": {str(x): entry(x, "libtraceevent1-plugin") for x in s["plugins"]}}
        got = ident()
        expect(got == {"read_status": "ok", "path": str(s["exe"]), "sha256": sha256(s["exe"]), "package": "trace-cmd", "version": "3.3.3-1ubuntu2",
                       "architecture": "amd64", "runtime": want_rt},
               f"an identity with its runtime, the caller's search inputs removed, the plugins as libtraceevent chooses them: {got}")
        got = ident("trace-cmd", PATH=f"{s['bindir']}:{saved.get('PATH', '')}")
        expect(got.get("path") == str(s["exe"]) and got["read_status"] == "ok", f"a name on PATH: {got}")
        got = ident(TK_DPKG_PKGS=s["env"]["TK_DPKG_PKGS"].replace("libtracefs1=1.8.2-1ubuntu1", "libtracefs1=1.8.3-1"))
        expect(got["read_status"] == "ok" and got["runtime"]["libraries"]["libtracefs.so.1"]["version"] == "1.8.3-1", f"another library version is read as it is: {got}")
        extra = s["plugin_dir"] / "plugin_extra.so"
        extra.write_bytes(b"an unreviewed plugin")
        got = ident()
        extra.unlink()
        expect(got["read_status"] == "ok" and got["runtime"]["plugins"].get(str(extra)) == {"path": str(extra), "sha256": hashlib.sha256(b"an unreviewed plugin").hexdigest(),
                                                                                         "package": None, "version": None, "architecture": None},
               f"a plugin no package installed is read, with no package: {got.get('runtime', {}).get('plugins')}")
        p = pathlib.Path(d, "ld.so.preload")
        p.write_text("/usr/lib/libsomething.so\n")
        got = ident(preload=str(p))
        expect(got["read_status"] == "ok" and got["runtime"]["preload"] == "/usr/lib/libsomething.so", f"a preload file is read: {got.get('runtime', {}).get('preload')}")
        moved = s["plugin_dir"].with_name("plugins.moved")
        s["plugin_dir"].rename(moved)
        got = ident()
        moved.rename(s["plugin_dir"])
        expect(got["read_status"] == "ok" and got["runtime"]["plugins"] == {} and got["runtime"]["plugin_dirs"] == [str(s["plugin_dir"])],
               f"a plugin directory that does not exist loads nothing, as libtraceevent skips it: {got.get('runtime')}")

        rows = s["rows"].read_text()
        te_bytes = s["te"].read_bytes()
        failures = [("not found", {"name": str(s["bindir"] / "nope")}, "is not found", None),
                    ("an executable no package installed", {"TK_DPKG_FILES": s["env"]["TK_DPKG_FILES"].replace(f"{s['exe']}=trace-cmd", "")}, "is not known", None),
                    ("a package not installed", {"TK_DPKG_STATUS": "rc "}, "not read as installed", None),
                    ("no dpkg-query", {"DPKG_QUERY": str(pathlib.Path(d, "no-dpkg"))}, "is not known", None),
                    ("a loader list that fails", {"TK_LD_STATUS": "1"}, "exited 1", None),
                    ("the default loader on a script", {"TK_LD_LIST": ""}, "names no dynamic loader", None),
                    ("libtracefs not found", {}, "does not resolve libtracefs.so.1",
                     lambda: s["rows"].write_text(rows.replace(f"libtracefs.so.1 => {lib}/libtracefs.so.1 (0x00007f0000002000)", "libtracefs.so.1 => not found"))),
                    ("an unreadable loader row", {}, "an unreadable row", lambda: s["rows"].write_text(rows + "\tsomething else\n")),
                    ("a library listed twice", {}, "appears twice", lambda: s["rows"].write_text(rows + f"\tlibtracefs.so.1 => {lib}/libtracefs.so.1 (0x1)\n")),
                    ("libtraceevent naming no plugin directory", {}, "plugin directories, not one", lambda: s["te"].write_bytes(b"stand-in")),
                    ("libtraceevent naming two", {}, "plugin directories, not one",
                     lambda: s["te"].write_bytes(te_bytes + b"/opt/other/traceevent/plugins\x00")),
                    ("a .so that is a directory", {}, "is not a file", lambda: (s["plugin_dir"] / "plugin_dir.so").mkdir())]
        if os.geteuid() != 0:  # root reads anything
            failures += [("an executable that cannot be read", {}, "Permission denied", lambda: s["exe"].chmod(0o111)),
                         ("a plugin directory that cannot be listed", {}, "cannot be listed", lambda: s["plugin_dir"].chmod(0o111)),
                         ("a plugin that cannot be read", {}, "Permission denied", lambda: s["plugins"][0].chmod(0)),
                         ("a library that cannot be read", {}, "Permission denied", lambda: s["tf"].chmod(0))]
        for what, env, words, setup in failures:
            if setup:
                setup()
            try:
                got = ident(**env)
            finally:
                s["rows"].write_text(rows)
                s["te"].write_bytes(te_bytes)
                for x in (s["exe"], s["plugins"][0], s["tf"]):
                    x.chmod(0o755)
                s["plugin_dir"].chmod(0o755)
                if (s["plugin_dir"] / "plugin_dir.so").exists():
                    (s["plugin_dir"] / "plugin_dir.so").rmdir()
            expect(words in got["read_status"] and "runtime" not in got, f"an identity with {what}: {got}")

        # The real loader's own list, on this Python: the parser reads what ld.so --list prints.
        py = os.path.realpath(sys.executable)
        interp = tk.elf_interp(py)
        loaded = tk.loader_list(py, tk.launch_env()) if interp else {}
        expect(interp is not None and interp.startswith("/") and any(k.startswith("libc.so") for k in loaded), f"the real loader's list: {interp} {loaded}")
        expect(tk.elf_interp(str(s["exe"])) is None, "a script names no loader")
        # The launch: every search input removed, the fixed ones and any LD_ variable set.
        env = {"HOME": "/h", "TRACEEVENT_PLUGIN_DIR": "/p", "TRACECMD_PLUGIN_DIR": "/q", "LD_LIBRARY_PATH": "/l", "LD_FOO": "1", "PATH": "/bin", "LC_ALL": "C"}
        expect(tk.launch_env(env) == {"PATH": "/bin", "LC_ALL": "C"}, f"the launch environment: {tk.launch_env(env)}")
        argv = tk.launch_argv(env)
        expect(argv[0] == "env" and all(f"-u {n}" in " ".join(argv) for n in ("HOME", "TRACEEVENT_PLUGIN_DIR", "TRACECMD_PLUGIN_DIR", "LD_LIBRARY_PATH", "LD_PRELOAD", "LD_AUDIT", "LD_FOO")),
               f"the launch prefix: {argv}")
        p = subprocess.run([sys.executable, "tools/tk_record.py", "launch"], capture_output=True, text=True)
        expect(p.returncode == 0 and p.stdout.splitlines()[:3] == ["env", "-u", "HOME"], f"the launch CLI: {p.stdout!r}")

        out = pathlib.Path(d, "tool.json")
        env = {**saved, **s["env"]}
        p = subprocess.run([sys.executable, "tools/tk_record.py", "tool", "--trace-cmd", str(s["link"]), "--out", str(out)], capture_output=True, text=True, env=env)
        expect(p.returncode == 0 and tk.read_tool(out)["sha256"] == sha256(s["exe"]) and tk.read_tool(out)["runtime"]["plugin_dirs"] == [str(s["plugin_dir"])],
               f"the CLI: {p.returncode} {p.stderr}")
        p = subprocess.run([sys.executable, "tools/tk_record.py", "tool", "--trace-cmd", str(s["bindir"] / "nope"), "--out", str(out)], capture_output=True, text=True, env=env)
        expect(p.returncode == 1 and "is not found" in tk.read_tool(out)["read_status"], f"the CLI, not found: {p.returncode}")

        # A saved report: written under the launch, with the identity of the trace-cmd and runtime that wrote it.
        r = standin_runtime(pathlib.Path(d, "r"), exe_text='#!/bin/sh\necho "$@" >"$0.args"\nenv >"$0.env"\necho "stand-in report"\nexit "${TK_STANDIN_EXIT:-0}"\n')
        rep, tout = pathlib.Path(d, "rep.txt"), pathlib.Path(d, "rep-tool.json")
        env = {**saved, **inputs, **r["env"]}
        p = subprocess.run([sys.executable, "tools/tk_record.py", "report", "--trace-cmd", str(r["link"]), "--dat", "/x/run.csv.trace.dat",
                            "--out", str(rep), "--tool-out", str(tout)], capture_output=True, text=True, env=env)
        seen = pathlib.Path(str(r["exe"]) + ".env").read_text().splitlines()
        os.environ.update(env)
        try:
            want = tk.tool_identity(str(r["link"]))
        finally:
            os.environ.clear()
            os.environ.update(saved)
        expect(p.returncode == 0 and rep.read_text() == "stand-in report\n" and pathlib.Path(str(r["exe"]) + ".args").read_text() == "report -t -i /x/run.csv.trace.dat\n"
               and not any(x.split("=", 1)[0] in inputs for x in seen) and tk.read_tool(tout) == want and want["read_status"] == "ok",
               f"a saved report under the launch: {p.returncode} {p.stderr} {[x for x in seen if x.split('=', 1)[0] in inputs]}")
        p = subprocess.run([sys.executable, "tools/tk_record.py", "report", "--trace-cmd", str(r["link"]), "--dat", "/x/run.csv.trace.dat",
                            "--out", str(rep), "--tool-out", str(tout)], capture_output=True, text=True, env={**env, "TK_STANDIN_EXIT": "1"})
        expect(p.returncode == 1 and "exited 1" in tk.read_tool(tout)["read_status"], f"a report that failed names no tool: {p.returncode}")


def certified(base, report_tool: dict | None = AUDITED_TOOL) -> tuple[str, list[str]]:
    c: dict = timer_clock.certify(tk.load(base, report_tool=report_tool))
    return c["status"], c["reasons"]


def evidence() -> None:
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d, "run.csv")
        expect(tk.load(base) is None, "no files at all: no record")
        st, why = certified(base)
        expect(st != "CERTIFIED" and any("no timekeeping record" in x for x in why), f"no record: {why}")
        write_record(base)
        st, why = certified(base)
        expect(st == "CERTIFIED", f"a complete record: {why}")

        def case(what: str, want: bool, words: str = "", **kw) -> None:
            write_record(base, **kw)
            st, why = certified(base)
            expect((st == "CERTIFIED") == want and (not words or any(words in x for x in why)), f"{what}: {st} {why}")

        # Every file: gone, emptied, or cut off halfway. Never an empty success.
        for suffix in tk.SUFFIXES:
            write_record(base)
            pathlib.Path(str(base) + suffix).unlink()
            st, why = certified(base)
            expect(st != "CERTIFIED" and bool(why), f"{suffix} missing: {st} {why}")
            write_record(base)
            pathlib.Path(str(base) + suffix).write_text("")
            st, why = certified(base)
            expect(st != "CERTIFIED" and bool(why), f"{suffix} empty: {st} {why}")
            write_record(base)
            f = pathlib.Path(str(base) + suffix)
            f.write_text(f.read_text()[: f.stat().st_size // 2])
            st, why = certified(base)
            expect(st != "CERTIFIED" and bool(why), f"{suffix} cut off: {st} {why}")

        # Contrary, repeated or malformed rows, before or after the good ones (the implementation review's counterexamples
        # among them): each makes its file unreadable and the record UNSUPPORTED; none is overwritten, skipped or normalised.
        def mangled(suffix: str, fn) -> tuple[str, list[str]]:
            write_record(base)
            f = pathlib.Path(str(base) + suffix)
            f.write_text(fn(f.read_text()))
            return certified(base)

        times = pathlib.Path(str(base) + ".tk-times.txt")
        write_record(base)
        armed_row = times.read_text().splitlines()[0]
        confirmed_row = times.read_text().splitlines()[1]
        for suffix, what, fn in (
                (".tk-enable-before.txt", "a disabled state before the enabled one", lambda t: "ntp_clear 0\n" + t),
                (".tk-enable-before.txt", "a disabled state after the enabled one", lambda t: t + "ntp_clear 0\n"),
                (".tk-enable-before.txt", "a repeated enabled state", lambda t: t + "ntp_clear 1\n"),
                (".tk-enable-before.txt", "a soft-mode state", lambda t: t.replace("ntp_clear 1", "ntp_clear 1*")),
                (".tk-enable-after.txt", "a disabled state before the enabled one", lambda t: "update_entry 0\n" + t),
                (".tk-enable-after.txt", "a disabled state after the enabled one", lambda t: t + "update_entry 0\n"),
                (".tk-enable-after.txt", "an extra malformed row", lambda t: t + "garbage\n"),
                (".tk-profile.txt", "a miss before the zero count", lambda t: "ntp_clear 0 1\n" + t),
                (".tk-profile.txt", "a miss after the zero count", lambda t: t + "ntp_clear 0 1\n"),
                (".tk-profile.txt", "a repeated count", lambda t: t + t.splitlines()[0] + "\n"),
                (".tk-profile.txt", "an extra malformed row", lambda t: t + "update_entry 1\n"),
                (".tk-times.txt", "an unreadable arming before", lambda t: "armed_t unreadable\n" + t),
                (".tk-times.txt", "an unreadable arming after", lambda t: t + "armed_t unreadable\n"),
                (".tk-times.txt", "a repeated arming", lambda t: armed_row + "\n" + t),
                (".tk-times.txt", "a contrary confirmation", lambda t: t + "confirmed_t 1\n"),
                (".tk-times.txt", "a repeated confirmation", lambda t: t + confirmed_row + "\n"),
                (".tk-times.txt", "the boundaries swapped", lambda _: confirmed_row + "\n" + armed_row + "\n"),
                (".tk-stats.txt", "an overrun after the zero one", lambda t: t.replace("bytes: 1\n", "bytes: 1\noverrun: 3\n", 1)),
                (".tk-stats.txt", "an overrun before the zero one", lambda t: t.replace("entries: 1\n", "entries: 1\noverrun: 3\n", 1)),
                (".tk-stats.txt", "a row after the CPU count", lambda t: t + "dropped events: 4\n"),
                (".tk-before.json", "a key given twice", lambda t: t.replace('{"read_status": "ok"', '{"read_status": "ok", "freq": 0', 1)),
                (".tk-after.json", "a key given twice", lambda t: t.replace('"tick": 10000', '"tick": 10000, "tick": 9000', 1)),
                (".tk-log.json", "a key given twice", lambda t: t.replace('"clock_events": []', '"clock_events": ["clocksource: unstable"], "clock_events": []', 1))):
            st, why = mangled(suffix, fn)
            expect(st != "CERTIFIED" and bool(why), f"{suffix}, {what}: {st} {why}")

        # The record's boundaries, from the probes' order: return on first, entry off first.
        case("a return before any complete update", True, rows_before=(update_rows(ARMED + 100_000)[1],))
        case("an entry after the interval, its return disabled", True, rows_after=(update_rows(S1 + 100_000_000)[0],))
        case("a return after the interval, its entry disabled", True, rows_after=(update_rows(S1 + 100_000_000)[1],))
        case("an entry before arming with no return", False, "unmatched update entry", rows_before=(update_rows(ARMED - 100_000, cpu=9)[0],))
        case("an entry at the first complete update with no return", False, "unmatched update entry", rows_before=(update_rows(ARMED + 500_000, cpu=9)[0],))
        case("an unmatched entry inside the interval", False, "unmatched update entry", rows_after=(update_rows(S0 + 1_000, cpu=9)[0],))
        case("a return inside the interval", False, "unmatched update return", rows_after=(update_rows(S0 + 1_000, cpu=9)[1],))
        case("an update over the 10 ms guard", False, "over the", rows_after=tuple(update_rows(S0 + 1_000, cpu=9, took=10_000_001)))
        case("an update at the 10 ms guard", True, rows_after=tuple(update_rows(S0 + 1_000, cpu=9, took=10_000_000)))
        # A return whose probe was disabled while the instance was outstanding: the kernel counts the hit and writes no
        # record. The hit count must equal the records, so this acquisition is refused, not explained away.
        write_record(base)
        prof = pathlib.Path(str(base) + ".tk-profile.txt")
        lines = dict(ln.split(" ", 1) for ln in prof.read_text().splitlines())
        hits, miss = lines["update_return"].split()
        lines["update_return"] = f"{int(hits) + 1} {miss}"
        prof.write_text("".join(f"{k} {v}\n" for k, v in lines.items()))
        st, why = certified(base)
        expect(st != "CERTIFIED" and any("update_return hit" in x for x in why), f"an outstanding return: {st} {why}")

        # What each file says.
        def edit(suffix: str, old: str, new: str) -> tuple[str, list[str]]:
            write_record(base)
            f = pathlib.Path(str(base) + suffix)
            t = f.read_text()
            assert old in t, (suffix, old)
            f.write_text(t.replace(old, new))
            return certified(base)

        for suffix, old, new, what, words in (
                (".tk-enable-before.txt", "update_entry 1", "update_entry 0", "a probe off before", "not confirmed enabled"),
                (".tk-enable-after.txt", "update_return 1", "update_return 0", "a probe off after", "not confirmed enabled"),
                (".tk-times.txt", "confirmed_t", "unconfirmed_t", "no confirmation", "confirmed_t"),
                (".tk-times.txt", "armed_t", "x_t", "no arming", "armed_t"),
                (".tk-log.json", '"clock_events": []', '"clock_events": ["clocksource: unstable"]', "a clock event", "kernel log"),
                (".tk-log.json", '"read_status": "ok"', '"read_status": "the log was unreadable"', "an unread log", "was not read"),
                (".tk-before.json", '"clocksource": "tsc"', '"clocksource": "hpet"', "another clocksource before", "clocksource"),
                (".tk-after.json", '"clocksource": "tsc"', '"clocksource": "hpet"', "another clocksource after", "clocksource"),
                (".tk-before.json", '"config_status": "ok"', '"config_status": "unreadable"', "no kernel config", "kernel was not read"),
                (".tk-before.json", '"tick": 10000', '"tick": 9000', "a changed tick", "tick changed")):
            st, why = edit(suffix, old, new)
            expect(st != "CERTIFIED" and any(words in x for x in why), f"{what}: {st} {why}")
        st, why = edit(".tk-profile.txt", "update_entry", "update_entry_x")
        expect(st != "CERTIFIED", f"a profile of something else: {st} {why}")
        write_record(base)
        prof = pathlib.Path(str(base) + ".tk-profile.txt")
        prof.write_text(prof.read_text().replace("ntp_clear 0 0", "ntp_clear 0 2"))
        st, why = certified(base)
        expect(st != "CERTIFIED" and any("missed 2" in x for x in why), f"misses: {why}")
        write_record(base)
        pathlib.Path(str(base) + ".tk-stats.txt").write_text(stats_text(2, overrun=3))
        st, why = certified(base)
        expect(st != "CERTIFIED" and any("overrun" in x for x in why), f"an overrun: {why}")
        write_record(base)
        pathlib.Path(str(base) + ".tk.txt").write_text("\n".join(update_rows(ARMED + 500_000)) + "\ngarbage\n")
        st, why = certified(base)
        expect(st != "CERTIFIED" and any("not a nanosecond record" in x for x in why), f"garbage in the record: {why}")
        for boot, want in (('root=x "ntp_tick_adj=1000"', False), ('root=x "ntp-tick-adj=1000"', False), ("root=x ntp_tick_adj=0", False),
                           ("root=x notntp_tick_adj=1000", True)):
            write_record(base)
            for suffix in (".tk-before.json",):
                f = pathlib.Path(str(base) + suffix)
                v = json.loads(f.read_text())
                v["cmdline"] = boot
                f.write_text(json.dumps(v))
            st, why = certified(base)
            expect((st == "CERTIFIED") == want, f"boot {boot}: {st} {why}")

        # The trace-cmd behind each use: the acquisition's and the recorder's from their files, the main report's from the
        # analysis. Missing, unreadable or another build is UNSUPPORTED (the implementation review's finding 3).
        write_record(base)
        st, why = certified(base, report_tool=None)
        expect(st != "CERTIFIED" and any("reported the main trace" in x and "not identified" in x for x in why), f"no reporting tool: {st} {why}")
        for what, tool, words in (("another version", {**AUDITED_TOOL, "version": "3.3.4-1"}, "not the audited build"),
                                  ("another build", {**AUDITED_TOOL, "sha256": "f" * 64}, "not the audited build"),
                                  ("an unreadable identity", {"read_status": "trace-cmd is not found"}, "not identified")):
            st, why = certified(base, report_tool=tool)
            expect(st != "CERTIFIED" and any("main trace" in x and words in x for x in why), f"a reporting tool with {what}: {st} {why}")
            for suffix, use in ((".tk-tool-before.json", "started the acquisition"), (".tk-tool-after.json", "extracted and reported the recorder")):
                write_record(base)
                pathlib.Path(str(base) + suffix).write_text(json.dumps(tool))
                st, why = certified(base)
                expect(st != "CERTIFIED" and any(use in x and words in x for x in why), f"{suffix} with {what}: {st} {why}")
        write_record(base)
        pathlib.Path(str(base) + ".tk-tool-after.json").write_text(json.dumps([AUDITED_TOOL]))
        st, why = certified(base)
        expect(st != "CERTIFIED" and any("not a mapping" in x for x in why), f"an identity that is not a mapping: {st} {why}")
        # The runtime behind each use, with the executable unchanged (the re-review's decision): a trace library or plugin
        # changed, absent or added, a search input selected, a preload, or a runtime missing or not understood is
        # UNSUPPORTED; the audited baseline certifies (above).
        te, tf = "libtraceevent.so.1", "libtracefs.so.1"
        hr = f"{timer_clock._TE_PLUGINS}/plugin_hrtimer.so"

        def changed(fn) -> dict:
            t2 = copy.deepcopy(AUDITED_TOOL)
            fn(t2)
            return t2

        def add_plugin(t2: dict, d: str = "/usr/lib/x86_64-linux-gnu/traceevent/plugins") -> None:
            t2["runtime"]["plugins"][f"{d}/plugin_extra.so"] = {"path": f"{d}/plugin_extra.so", "sha256": "1" * 64, "package": None, "version": None, "architecture": None}

        def env_dir(t2: dict) -> None:
            t2["runtime"]["inputs"]["TRACEEVENT_PLUGIN_DIR"] = "/opt/plugins"
            t2["runtime"]["plugin_dirs"].append("/opt/plugins")
            add_plugin(t2, "/opt/plugins")

        for what, tool, words in (
                ("libtraceevent's file changed", changed(lambda x: x["runtime"]["libraries"][te].update(sha256="0" * 64)), f"{te}:"),
                ("libtracefs at another version", changed(lambda x: x["runtime"]["libraries"][tf].update(version="1.8.3-1")), f"{tf}:"),
                ("libtracefs resolved elsewhere", changed(lambda x: x["runtime"]["libraries"][tf].update(path="/usr/local/lib/libtracefs.so.1")), f"{tf}:"),
                ("libtracefs absent", changed(lambda x: x["runtime"]["libraries"].pop(tf)), f"{tf}: absent"),
                ("an unreviewed library", changed(lambda x: x["runtime"]["libraries"].update({"libtracecmd.so.1": {"path": "/x", "sha256": "2" * 64}})), "unreviewed library"),
                ("a plugin added", changed(add_plugin), "unreviewed plugin"),
                ("a plugin absent", changed(lambda x: x["runtime"]["plugins"].pop(hr)), "not loaded"),
                ("a plugin's file changed", changed(lambda x: x["runtime"]["plugins"][hr].update(sha256="3" * 64)), "plugin_hrtimer.so:"),
                ("a plugin directory from the environment", changed(env_dir), "plugin directories"),
                ("HOME in the launch", changed(lambda x: x["runtime"]["inputs"].update(HOME="/root")), "search inputs"),
                ("an LD_ variable in the launch", changed(lambda x: x["runtime"]["inputs"].update(LD=["LD_LIBRARY_PATH"])), "search inputs"),
                ("a preload", changed(lambda x: x["runtime"].update(preload="/usr/lib/libsomething.so")), "a preload"),
                ("no runtime", changed(lambda x: x.pop("runtime")), "missing trace_cmd"),
                ("a runtime that is not a mapping", changed(lambda x: x.update(runtime=[1])), "wrong type"),
                ("a runtime field not understood", changed(lambda x: x["runtime"].update(extra=1)), "other fields"),
                ("the first version's unjudged library list", changed(lambda x: x.update(libraries={"libtraceevent1": "unreviewed"})), "does not know: libraries")):
            write_record(base)
            st, why = certified(base, report_tool=tool)
            expect(st != "CERTIFIED" and any(words in x for x in why) and any("main trace" in x or "trace_cmd.report" in x for x in why),
                   f"the main report's runtime with {what}: {st} {why}")
            for suffix, use in ((".tk-tool-before.json", "before"), (".tk-tool-after.json", "after")):
                write_record(base)
                pathlib.Path(str(base) + suffix).write_text(json.dumps(tool))
                st, why = certified(base)
                expect(st != "CERTIFIED" and any(words in x for x in why) and any(timer_clock.TRACE_CMD_USES[use] in x or f"trace_cmd.{use}" in x for x in why),
                       f"{suffix} with {what}: {st} {why}")

        # The calls in the record.
        tc = [f" chronyd-55 [002] d..1. {stamp(S0 + 500_000)}: ntp_adjtimex: (x) tkid=0 modes=1"]
        case("a modifying call", False, "timekeeping changes", rows_after=tuple(tc))
        tc = [f" chronyd-55 [002] d..1. {stamp(S0 + 500_000)}: ntp_adjtimex: (x) tkid=1 modes=1"]
        case("a modifying call on a non-core timekeeper", True, rows_after=tuple(tc))
        tc = [f" kworker-55 [002] d..1. {stamp(S0 + 500_000)}: ntp_clear: (x) tkid=0"]
        case("ntp_clear", False, "timekeeping changes", rows_after=tuple(tc))
        write_record(base)
        t = pathlib.Path(str(base) + ".tk.txt")
        t.write_text(t.read_text().replace("tk_record-777", "tk_record-778"))
        st, why = certified(base)
        expect(st != "CERTIFIED" and any("state read is not in the record" in x for x in why), f"another thread's reads: {why}")


SHIM = r'''
# Stand-ins for the kernel's side of tracefs, loaded by BASH_ENV.
tfs_target() { readlink "/proc/$$/fd/1"; }
tfs_log() { builtin printf '%s\n' "$*" >>"$TK_FAKE_LOG"; }
tfs_fail() { local x; for x in ${TK_FAKE_FAIL:-}; do [[ $1 == *$x* ]] && return 0; done; return 1; }
echo() {
  local t; t=$(tfs_target)
  case "$t" in
    "$TK_FAKE_ROOT"/*)
      tfs_log "write ${t#"$TK_FAKE_ROOT"/} $*"
      tfs_fail "$t" && return 1
      case "$t" in
        */trace_clock) builtin echo "local [${TK_FAKE_CLOCK:-$1}] x86-tsc" ;;
        */enable) if [ -n "${TK_FAKE_READBACK:-}" ] && [[ $t == *"${TK_FAKE_READBACK%%:*}"* ]]; then builtin echo "${TK_FAKE_READBACK#*:}"; else builtin echo "$@"; fi ;;
        *) builtin echo "$@" ;;
      esac
      return 0 ;;
  esac
  builtin echo "$@"
}
printf() {
  local t l; t=$(tfs_target)
  case "$t" in
    */kprobe_events)
      l=$(builtin printf "$@")
      tfs_log "define ${l%% *}"
      tfs_fail "$l" && return 1
      builtin printf "$@"
      case "$l" in
        -:*) if ! tfs_fail "keep"; then rm -rf "$TK_FAKE_ROOT/events/${l#-:}"; command rmdir "$TK_FAKE_ROOT/events/${name:-x}" 2>/dev/null; fi ;;
        *) l=${l%% *}; if ! tfs_fail "noevent"; then command mkdir -p "$TK_FAKE_ROOT/events/${l#*:}"; fi ;;
      esac
      return 0 ;;
  esac
  builtin printf "$@"
}
mkdir() {
  tfs_log "mkdir ${1#"$TK_FAKE_ROOT"/}"
  tfs_fail "mkdir" && return 1
  command mkdir "$1" || return 1
  local e n=${1##*/}
  for e in ntp_adjtimex ntp_clear update_entry update_return; do command mkdir -p "$1/events/$n/$e"; done
}
rmdir() {
  tfs_log "rmdir ${1#"$TK_FAKE_ROOT"/}"
  tfs_fail "rmdir" && return 1
  rm -rf "$1"
}
'''


def recorder() -> None:
    with tempfile.TemporaryDirectory() as d:
        root, shim, log = pathlib.Path(d, "t"), pathlib.Path(d, "shim.sh"), pathlib.Path(d, "log")
        shim.write_text(SHIM)
        name = "rtctrl_tk"
        order_entry, order_return = f"instances/{name}/events/{name}/update_entry/enable", f"instances/{name}/events/{name}/update_return/enable"

        def fresh() -> None:
            subprocess.run(["rm", "-rf", str(root)], check=True)
            (root / "instances").mkdir(parents=True)
            (root / "events").mkdir()
            (root / "kprobe_events").write_text("p:other/keep_me sys_read\n")
            log.write_text("")

        def run(cmd: str, *rest: str, **env: str) -> subprocess.CompletedProcess:
            e = {**os.environ, "BASH_ENV": str(shim), "TK_FAKE_ROOT": str(root), "TK_FAKE_LOG": str(log), **env}
            return subprocess.run(["tools/tk_recorder.sh", cmd, str(root), name, *rest], capture_output=True, text=True, env=e, timeout=30)

        def writes() -> list[str]:
            return log.read_text().splitlines()

        fresh()
        p = run("start", "1024")
        w = writes()
        expect(p.returncode == 0 and "timekeeping recorder" in p.stdout, f"start: {p.returncode} {p.stderr}")
        expect(f"write {order_return} 1" in w and f"write {order_entry} 1" in w
               and w.index(f"write {order_return} 1") < w.index(f"write {order_entry} 1"), f"the return probe first: {w}")
        expect(w.index(f"write instances/{name}/tracing_on 1") < w.index(f"write {order_return} 1"), f"the instance on first: {w}")
        expect(w.index(f"write instances/{name}/trace_clock mono") < w.index(f"write instances/{name}/tracing_on 1")
               and f"write instances/{name}/buffer_size_kb 1024" in w, f"clock and buffer first: {w}")
        expect([x.split()[1] for x in w if x.startswith("define ")] == [f"p:{name}/ntp_adjtimex", f"p:{name}/ntp_clear", f"r:{name}/update_return", f"p:{name}/update_entry"],
               f"the definitions: {w}")
        ke = (root / "kprobe_events").read_text()
        expect(ke.startswith("p:other/keep_me sys_read\n"), f"another group's probe is untouched: {ke}")
        expect(f"p:{name}/ntp_adjtimex ntp_adjtimex tkid=$arg1:u32 modes=+0($arg2):u32" in ke, f"the probe's arguments: {ke}")

        p = run("states")
        expect(p.returncode == 0 and p.stdout.split("\n")[:4] == ["ntp_adjtimex 1", "ntp_clear 1", "update_return 1", "update_entry 1"], f"states: {p.stdout}")

        log.write_text("")
        p = run("disable")
        w = writes()
        enable_order = [x.split()[1].split("/")[-2] for x in w if x.endswith("enable 0")]
        expect(p.returncode == 0 and enable_order[:2] == ["update_entry", "update_return"] and len(enable_order) == 4, f"the entry probe off first: {w}")
        expect(w[-1] == f"write instances/{name}/tracing_on 0", f"the instance off last: {w}")

        for c in range(2):
            (root / "instances" / name / "per_cpu" / f"cpu{c}").mkdir(parents=True)
            (root / "instances" / name / "per_cpu" / f"cpu{c}" / "stats").write_text("entries: 1\noverrun: 0\ncommit overrun: 0\ndropped events: 0\n")
        p = run("stats")
        got, st = tk.parse_stats(p.stdout)
        expect(p.returncode == 0 and st == "ok" and got["overrun"] == 0, f"stats: {p.stdout} {st}")

        good = "".join(f"  {e:<44} {i + 5:>15} {i:>15}\n" for i, e in enumerate(("ntp_adjtimex", "ntp_clear", "update_return", "update_entry")))
        (root / "kprobe_profile").write_text(good + "  rtfs_other 9 9\n")
        p = run("profile")
        expect(p.returncode == 0 and p.stdout.splitlines() == ["ntp_adjtimex 5 0", "ntp_clear 6 1", "update_return 7 2", "update_entry 8 3"], f"profile: {p.stdout}")
        (root / "kprobe_profile").write_text("  ntp_clear 1 0\n")
        p = run("profile")
        expect(p.returncode == 1 and "ntp_adjtimex missing" in p.stdout, f"a missing probe: {p.stdout}")
        (root / "kprobe_profile").write_text(good + "  update_entry 1 0\n")
        p = run("profile")
        expect(p.returncode == 1, f"a duplicate name: {p.stdout}")

        p = run("exists")
        expect(p.returncode == 0 and p.stdout == "present\n", f"exists: {p.returncode} {p.stdout!r}")
        log.write_text("")
        p = run("remove")
        w = writes()
        expect(p.returncode == 0 and w[:2] == [f"write instances/{name}/tracing_on 0", f"rmdir instances/{name}"]
               and all(x.startswith("define -:") for x in w[2:]) and len(w) == 6, f"remove, the instance before the probes: {w} {p.stderr}")
        expect(not (root / "instances" / name).exists() and not (root / "events" / name).exists(), "removed, the group with its last event")
        expect(run("remove").returncode == 0, "removing what is absent")
        p = run("exists")
        expect(p.returncode == 0 and p.stdout == "absent\n", f"gone: {p.returncode} {p.stdout!r}")

        # The checked existence: an answer only from listings read in full. A directory that is missing or cannot be
        # listed answers nothing with status 1 (sudo's own failure status), and a removal cannot verify through it.
        fresh()
        (root / "events" / name).mkdir()
        p = run("exists")
        expect(p.returncode == 0 and p.stdout == "present\n", f"the group alone: {p.stdout!r}")
        fresh()
        (root / "instances").rmdir()
        p = run("exists")
        expect(p.returncode == 1 and p.stdout == "", f"no instances directory: {p.returncode} {p.stdout!r}")
        if os.geteuid() != 0:  # root lists any directory
            for sub in ("instances", "events"):
                fresh()
                run("start", "1024")
                (root / sub).chmod(0)
                try:
                    p = run("exists")
                    expect(p.returncode == 1 and p.stdout == "" and "cannot list" in p.stderr, f"an unlistable {sub}: {p.returncode} {p.stdout!r}")
                    p = run("remove")
                    expect(p.returncode == 1 and "cannot tell" in p.stderr, f"a removal through an unlistable {sub}: {p.returncode} {p.stderr}")
                finally:
                    (root / sub).chmod(0o755)
            fresh()
            run("start", "1024")
            (root / "events" / name).chmod(0)
            try:
                p = run("remove")
                expect(p.returncode == 1 and "cannot tell" in p.stderr, f"an unlistable group: {p.returncode} {p.stderr}")
            finally:
                (root / "events" / name).chmod(0o755)

        # Ownership: status 2 is not the caller's to remove; 4 is.
        fresh()
        (root / "instances" / name).mkdir()
        p = run("start", "1024")
        expect(p.returncode == 2 and not writes(), f"an existing instance: {p.returncode} {writes()}")
        fresh()
        (root / "events" / name).mkdir()
        expect(run("start", "1024").returncode == 2 and not writes(), "an existing group")
        for what, env, want in (("the first definition refused", {"TK_FAKE_FAIL": "ntp_adjtimex"}, 2),
                                ("a later definition refused", {"TK_FAKE_FAIL": "update_entry"}, 4),
                                ("no event directories", {"TK_FAKE_FAIL": "noevent"}, 4),
                                ("no instance", {"TK_FAKE_FAIL": "mkdir"}, 4),
                                ("a clock that is not mono", {"TK_FAKE_CLOCK": "local"}, 4),
                                ("no buffer", {"TK_FAKE_FAIL": "buffer_size_kb"}, 4),
                                ("a probe not enabled", {"TK_FAKE_FAIL": "update_return/enable"}, 4),
                                ("a read-back of 0", {"TK_FAKE_READBACK": "update_entry:0"}, 4)):
            fresh()
            p = run("start", "1024", **env)
            expect(p.returncode == want, f"{what}: status {p.returncode}: {p.stderr}")
        # A failure after arming still removes what it made, through the caller's remove.
        fresh()
        p = run("start", "1024", TK_FAKE_READBACK="update_entry:0")
        p = run("remove")
        expect(p.returncode == 0 and not (root / "instances" / name).exists() and not (root / "events" / name / "update_entry").exists(), f"a partial start's removal: {p.stderr}")

        fresh()
        run("start", "1024")
        p = run("remove", TK_FAKE_FAIL="rmdir")
        expect(p.returncode == 1 and (root / "instances" / name).exists(), f"the instance cannot be removed: {p.stderr}")
        p = run("remove", TK_FAKE_FAIL="keep")
        expect(p.returncode == 1 and "remains" in p.stderr, f"a probe remains: {p.stderr}")
        p = run("remove")
        expect(p.returncode == 0, f"the retried removal: {p.stderr}")
        fresh()
        p = run("states")
        expect(p.returncode == 1, "states with nothing there")
        p = run("disable")
        expect(p.returncode == 1, "disable with nothing there")
        fresh()
        run("start", "1024")
        p = run("disable", TK_FAKE_FAIL="update_entry/enable")
        expect(p.returncode == 1, "a failed disable is reported")
        (root / "instances" / name / "per_cpu").mkdir(exist_ok=True)
        p = run("stats")
        expect(p.returncode == 1, f"no per-CPU stats: {p.stdout}")
        expect(subprocess.run(["tools/tk_recorder.sh", "start", str(root), "bad name", "1"], capture_output=True).returncode == 2, "a bad name")


def main() -> int:
    t0 = time.monotonic()
    parsers()
    state_reads()
    identities()
    evidence()
    recorder()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print(f"timekeeping record regressions OK ({time.monotonic() - t0:.1f} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
