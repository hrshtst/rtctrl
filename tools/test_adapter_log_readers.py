#!/usr/bin/env python3
"""Regressions: the probe log's readers read an adapter-timing log as they read a grid log.

x7_bus_timing --timing adapter appends integer columns to the grid log's (bus_timing::kAdapterColumns) and adds
`timing = "adapter"` and an [adapter] section to the run summary. Each reader of the probe log (timing_summary.py,
trace_correlate.py, tk_cost.py) is run on a log twice: as the grid log, and with the adapter columns and summary
additions, its rows otherwise unchanged. Every output must be identical, and timing_summary must parse the new columns as
the integers they are. The column lists are read from the probe's own sources, so a change there is tested here.
"""

from __future__ import annotations

import csv
import importlib.util
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

FAILURES: list[str] = []
ROOT = pathlib.Path(__file__).resolve().parent.parent


def expect(cond: bool, what: str, detail: str = "") -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAILED: {what}\n{detail}")


def probe_columns() -> tuple[list[str], list[str]]:
    """The grid log's header (kCsvHeader in x7_bus_timing.cpp) and the adapter columns (bus_timing_adapter.hpp)."""
    src = (ROOT / "apps/bus/x7_bus_timing.cpp").read_text()
    m = re.search(r'constexpr const char\* kCsvHeader =\s*((?:"[^"]*"\s*)+);', src)
    assert m, "kCsvHeader not found"
    grid = "".join(re.findall(r'"([^"]*)"', m.group(1))).split(",")
    hdr = (ROOT / "apps/bus/bus_timing_adapter.hpp").read_text()
    m = re.search(r'constexpr const char\* kAdapterColumns =\s*((?:"[^"]*"\s*)+);', hdr)
    assert m, "kAdapterColumns not found"
    adapter = "".join(re.findall(r'"([^"]*)"', m.group(1))).split(",")
    return grid, adapter


GRID_COLUMNS, ADAPTER_COLUMNS = probe_columns()
SUMMARY_ADDITION = ('timing = "adapter"\n', '\n[adapter]\nlead_us = 780\nlead_alternation_us = 20\nprotocol_errors = 0\n'
                    '\n[adapter.locked]\ncycles = 3\n\n[[adapter.lock]]\nlocked_cycle = 2\nend = "run_end"\n')


def adapter_values(i: int) -> dict:
    """Plausible adapter columns for row i: the grid for the first rows, then a lattice with alternating offsets."""
    lattice = i >= 2
    return {"target_index": i if lattice else -1, "schedule": int(lattice), "lock_state": 2 if lattice else 0,
            "transition": int(i == 2), "predicted_completion_ns": 790_000 + i * 999_942 if lattice else -1,
            "lead_offset_ns": (20_000 if i % 2 == 0 else -20_000) if lattice else 0,
            "coupling_state": 1 if i > 4 else 0, "observation": 3 if lattice else 1, "recheck_skipped": int(i == 3)}


def with_adapter(csv_path: pathlib.Path, out: pathlib.Path) -> None:
    """The log at csv_path with the adapter columns appended to every row."""
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        fields = list(reader.fieldnames or [])
        rows = list(reader)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields + ADAPTER_COLUMNS, lineterminator="\n")
        w.writeheader()
        for i, r in enumerate(rows):
            w.writerow({**r, **adapter_values(i)})


def with_adapter_summary(summary: pathlib.Path, out: pathlib.Path) -> None:
    """The summary with the top-level `timing` key and an [adapter] section, as the probe writes them."""
    text = summary.read_text()
    first_table = text.find("\n[")
    head, rest = (text, "") if first_table < 0 else (text[:first_table + 1], text[first_table + 1:])
    out.write_text(head + SUMMARY_ADDITION[0] + rest + SUMMARY_ADDITION[1])


def twin(base: pathlib.Path, name: str) -> pathlib.Path:
    """A copy of base's log and every sidecar (base.csv.*) under name, with the adapter additions."""
    out = base.with_name(name + ".csv")
    with_adapter(base, out)
    for side in base.parent.glob(base.name + ".*"):
        dest = out.with_name(out.name + side.name[len(base.name):])
        if side.name.endswith(".summary.toml"):
            with_adapter_summary(side, dest)
        else:
            shutil.copyfile(side, dest)
    return out


def normalized(text: str, *paths: pathlib.Path) -> str:
    """The text with the variant's own paths and its file stem (xlog0000 or xlog1111, which no tool prints) replaced."""
    for p in paths:
        text = text.replace(str(p), "<path>")
    return text.replace("xlog0000", "<name>").replace("xlog1111", "<name>")


def timing_summary() -> None:
    spec = importlib.util.spec_from_file_location("ts", ROOT / "tools/timing_summary.py")
    assert spec is not None and spec.loader is not None
    ts = importlib.util.module_from_spec(spec)
    sys.modules["ts"] = ts
    spec.loader.exec_module(ts)
    with tempfile.TemporaryDirectory() as d:
        base = pathlib.Path(d, "xlog0000.csv")
        rows = []
        for k in range(1, 401):
            sched = (k - 1) * 1_000_000 + (20_000 if k % 2 else -20_000) * (k > 2)
            wake = sched + 5_000
            rs = wake + 3_000
            rows.append({"cycle": k, "scheduled_wake_ns": sched, "wake_ns": wake, "read_start_ns": rs,
                         "instr_return_ns": rs + 10_000, "first_reply_ns": rs + 640_000, "last_reply_ns": rs + 650_000,
                         "read_end_ns": rs + 651_000, "limit_start_ns": rs + 652_000,
                         "bus_write_start_ns": rs + 653_000, "write_return_ns": rs + 660_000, "drain_return_ns": -1,
                         "ack_return_ns": -1, "completion_upper_ns": -1, "cycle_end_ns": rs + 661_000,
                         "read_ok": 1, "write_ok": 1, "ack_ok": 0, "skipped_periods": int(k in (3, 200)),
                         "read_len": 13 if (k - 1) % 100 == 0 else 10})
        with open(base, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=GRID_COLUMNS, lineterminator="\n")
            w.writeheader()
            w.writerows(rows)
        pathlib.Path(str(base) + ".summary.toml").write_text(
            'format = "rtctrl-bus-timing"\nversion = 2\nkind = "production-traffic"\nrate_hz = 1000\n'
            'log_complete = true\n\n[result]\nstarted = true\ncycles = 400\nring_drops = 0\nlate_wake_stop = false\n'
            'late_wake_s = 0\nlate_wake_lateness_s = 0\n')
        adapter = twin(base, "xlog1111")
        header = adapter.read_text().splitlines()[0].split(",")
        expect(header == GRID_COLUMNS + ADAPTER_COLUMNS, "the adapter log's header is the grid's plus the adapter columns",
               str(header))
        kind_g, data_g = ts.load(base)
        kind_a, data_a = ts.load(adapter)
        expect(kind_g == kind_a == "probe", "both are probe logs", f"{kind_g} {kind_a}")
        expect(all((data_g[c] == data_a[c]).all() for c in GRID_COLUMNS), "the shared columns load identically")
        expect(all(data_a[c].dtype.kind == "i" for c in ADAPTER_COLUMNS), "the adapter columns load as integers")
        expect(int(data_a["predicted_completion_ns"][0]) == -1 and int(data_a["lead_offset_ns"][3]) == -20_000,
               "negative values survive", "")
        run_g = ts.load_run_summary(pathlib.Path(str(base) + ".summary.toml"), "probe")
        run_a = ts.load_run_summary(pathlib.Path(str(adapter) + ".summary.toml"), "probe")
        expect(run_a is not None and run_a.get("timing") == "adapter" and "adapter" in run_a,
               "the adapter summary carries its additions", str(run_a))
        s_g = ts.summarize(kind_g, data_g, 1000.0, None, None, run_g)
        s_a = ts.summarize(kind_a, data_a, 1000.0, None, None, run_a)
        s_g.pop("run")
        s_a.pop("run")
        expect(repr(s_g) == repr(s_a), "the reductions are identical", f"{s_g}\n{s_a}")
        outs = []
        for log in (base, adapter):
            p = subprocess.run([sys.executable, str(ROOT / "tools/timing_summary.py"), str(log), "--rate-hz", "1000",
                                "--loaded", "--check"], capture_output=True, text=True)
            outs.append((p.returncode, normalized(p.stdout + p.stderr, log)))
        expect(outs[0] == outs[1], "the report and its verdicts are identical", f"{outs[0][1]}\n---\n{outs[1][1]}")
        expect("T1" in outs[0][1] and "T3" in outs[0][1], "the report ran its criteria", outs[0][1])


def trace_correlate() -> None:
    import test_trace_correlate as ttc  # noqa: E402  (its fixture: a synthetic probe run and trace)
    with tempfile.TemporaryDirectory() as d:
        root = pathlib.Path(d)
        base = ttc.Run().write(root, "xlog0000")
        report = ttc.saved_report(root, "xlog0000")
        adapter = twin(base, "xlog1111")
        shutil.copyfile(root / "xlog0000.report.txt", root / "xlog1111.report.txt")
        shutil.copyfile(root / "xlog0000.report-tool.json", root / "xlog1111.report-tool.json")
        outs = []
        for name, log in (("xlog0000", base), ("xlog1111", adapter)):
            seg = root / f"{name}.segments.csv"
            args = [a.replace("/xlog0000.", f"/{name}.") for a in report]
            p = subprocess.run([sys.executable, str(ROOT / "tools/trace_correlate.py"), str(log), *args,
                                "--adapter", "1-9", "--segments-csv", str(seg)], capture_output=True, text=True)
            outs.append((p.returncode, normalized(p.stdout + p.stderr, log, root / f"{name}.report.txt", seg),
                         seg.read_text()))
        expect(outs[0][0] == 0, "the grid fixture passes", outs[0][1])
        expect(outs[0] == outs[1], "the correlation, its verdict and its segments are identical",
               f"{outs[0][1]}\n---\n{outs[1][1]}")


def tk_cost() -> None:
    import test_tk_cost as ttk  # noqa: E402
    with tempfile.TemporaryDirectory() as d:
        lines, ups, rows = ttk.capture()
        base = pathlib.Path(d, "xlog0000.csv")
        header = "cycle,scheduled_wake_ns,wake_ns,cycle_end_ns"
        base.write_text("\n".join([header] + [f"{r['cycle']},{r['scheduled_wake_ns']},{r['wake_ns']},{r['cycle_end_ns']}"
                                              for r in rows]) + "\n")
        pathlib.Path(str(base) + ".summary.toml").write_text(f"[result]\nmonotonic_origin_ns = {ttk.T}\n"
                                                             f"worker_tid = {ttk.TID}\n")
        pathlib.Path(str(base) + ".run.txt").write_text("command: x --rt-cpu 3 --rt-priority 80\n")
        rec = ["cpus=32"] + [f"rtctrl_tk:   <idle>-0  [{u['cpu']:03d}] d.h1. {ttk.stamp(u['entry'])}: update_entry: (x)\n"
                             f"rtctrl_tk:   <idle>-0  [{u['cpu']:03d}] d.h1. {ttk.stamp(u['ret'])}: update_return: "
                             "(x <- y)" for u in sorted(ups, key=lambda u: u["entry"])]
        pathlib.Path(str(base) + ".tk.txt").write_text("\n".join(rec) + "\n")
        rep = pathlib.Path(d, "run.report.txt")
        rep.write_text("\n".join(lines) + "\n")
        adapter = twin(base, "xlog1111")
        outs = []
        for log in (base, adapter):
            p = subprocess.run([sys.executable, str(ROOT / "tools/tk_cost.py"), str(log), "--report", str(rep)],
                               capture_output=True, text=True)
            outs.append((p.returncode, normalized(p.stdout + p.stderr, log)))
        expect(outs[0][0] == 0 and "judges and bounds nothing" in outs[0][1], "the grid fixture runs", outs[0][1])
        expect(outs[0] == outs[1], "the cost report is identical", f"{outs[0][1]}\n---\n{outs[1][1]}")


def main() -> int:
    timing_summary()
    trace_correlate()
    tk_cost()
    if FAILURES:
        print(f"{len(FAILURES)} FAILED")
        return 1
    print("adapter log readers OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
