#!/usr/bin/env python3
"""Regressions for latency_hold.py, run under ctest, without hardware.

A stand-in adapter plays the latency timer, the servo and the host's input
on a simulated monotonic clock, and the real sequence runs on it: a reply
held at 0 ms and delivered after the switch to 1 ms, no payload, a partial
and a corrupt payload, a reply at 0 ms, a setting that does not read back
or cannot be written, a cleanup that cannot restore 1 ms, an interruption,
a refusal and a silent baseline, a baseline that changes the timer, pings
not written whole (short, zero or failed writes) and a read error. Through
main(), with the port, the timer, privileges and the recorder replaced:
valid evidence, and a recorder that fails to stop, a missing or empty
capture, reported drops or no drop count, each invalidating the result
with the restore and close still done first. Pinned: the Protocol 2.0 ping
and its CRC, the payload classes, that only pings are sent, that nothing
is sent or flushed between the 0 ms ping and the end of the window after
the switch, that nothing is sent after an abort, the cleanup and the exit
statuses.

With --emulator DXL_EMU, the real serial port class runs instead, on
dxl_emu's pseudo-terminal (only the latency timer stands in): its
configuration, exclusive open, reads against monotonic deadlines and
flushes, with the emulated servo answering every ping.
"""

import contextlib
import importlib.util
import io
import os
import pathlib
import subprocess
import sys
import tempfile
import time
import types
from unittest.mock import patch

sys.path.insert(0, "tools")
spec = importlib.util.spec_from_file_location("lh", "tools/latency_hold.py")
assert spec is not None and spec.loader is not None
lh = importlib.util.module_from_spec(spec)
sys.modules["lh"] = lh
spec.loader.exec_module(lh)

MS = 1_000_000


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def reply(servo_id: int = 2, crc_ok: bool = True) -> bytes:
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, servo_id, 0x07, 0x00, 0x55, 0x00, 0xFC, 0x03, 0x2F])
    crc = lh.crc16(body) ^ (0 if crc_ok else 1)
    return body + crc.to_bytes(2, "little")


class Clock:
    def __init__(self) -> None:
        self.t = 5_000_000_000

    def now(self) -> int:
        return self.t


class Adapter:
    """The latency timer, the servo behind the adapter and the host's input.
    At 1 ms a reply reaches the host 0.6 ms after a ping; at 0 ms the
    scenario decides. Held bytes reach the host 1 ms after 1 ms is set."""

    def __init__(self, clock: Clock, scenario: str, start: int = 1, low: bool = False) -> None:
        self.clock, self.scenario = clock, scenario
        self.sysfs = self.device = start
        self.low = low
        self.host: list[tuple[int, bytes]] = []
        self.held = bytearray()
        self.events: list[tuple] = []        # ("write", bytes) / ("flush",) / ("set", ms)
        self.set_fail: set[int] = set()
        self.stuck = None
        self.on_write = {}                    # write number -> hook
        self.interrupt_on_write = None

    # The latency timer.
    def set(self, ms: int) -> None:
        self.events.append(("set", ms))
        if ms in self.set_fail:
            raise OSError("stand-in: cannot write the latency timer")
        self.sysfs = ms
        if self.stuck is None:
            self.device = ms
        if self.device == 1 and self.held:
            self.host.append((self.clock.now() + MS, bytes(self.held)))
            self.held.clear()

    def state(self) -> tuple:
        return (self.sysfs, self.device, self.low)

    # The serial port.
    def write(self, data: bytes) -> int:
        self.events.append(("write", data))
        n = sum(1 for e in self.events if e[0] == "write")
        if n in self.on_write:
            self.on_write[n]()
        now = self.clock.now()
        if self.scenario == "silent":
            return len(data)
        if self.device == 1 or self.scenario == "immediate":
            self.host.append((now + 600_000, reply()))
        elif self.scenario == "held":
            self.held += reply()
        elif self.scenario == "partial":
            self.host.append((now + 300_000, reply()[:6]))
        elif self.scenario == "corrupt":
            self.host.append((now + 600_000, reply(crc_ok=False)))
        return len(data)

    def read(self, timeout_ns: int) -> bytes:
        if self.interrupt_on_write is not None and \
                sum(1 for e in self.events if e[0] == "write") >= self.interrupt_on_write:
            self.interrupt_on_write = None
            raise lh.Interrupted()
        now = self.clock.now()
        due = [c for c in self.host if c[0] <= now]
        if due:
            self.host = [c for c in self.host if c[0] > now]
            return b"".join(c[1] for c in due)
        nxt = min((c[0] for c in self.host), default=None)
        if nxt is not None and nxt - now <= timeout_ns:
            self.clock.t = nxt
            return self.read(0)
        self.clock.t = now + timeout_ns
        return b""

    def flush_input(self) -> None:
        self.events.append(("flush",))
        now = self.clock.now()
        self.host = [c for c in self.host if c[0] > now]


class Serial:
    def __init__(self, a: Adapter) -> None:
        self.a = a

    def write(self, data):
        return self.a.write(data)

    def read(self, timeout_ns):
        return self.a.read(timeout_ns)

    def flush_input(self):
        self.a.flush_input()

    def low_latency(self):
        return self.a.low


class Latency:
    def __init__(self, a: Adapter) -> None:
        self.a = a

    def set(self, ms):
        self.a.set(ms)

    def read(self, serial):
        return self.a.state()


def run(scenario: str, **kw):
    clock = Clock()
    a = Adapter(clock, scenario, start=kw.pop("start", 1), low=kw.pop("low", False))
    for k, v in kw.items():
        setattr(a, k, v)
    hold = lh.Hold(Serial(a), Latency(a), clock, 2, 50 * MS)
    status = hold.run()
    return status, hold, a


def only_pings(a: Adapter) -> bool:
    return all(e[1] == lh.ping_packet(2) for e in a.events if e[0] == "write")


def quiet_after_zero_ping(hold) -> bool:
    """Nothing written or flushed from the 0 ms ping to the end of the window
    after the switch."""
    ev = hold.events
    start = next(i for i, e in enumerate(ev) if e[1] == "zero" and e[2] == "write")
    end = next(i for i, e in enumerate(ev) if e[1] == "after_switch" and e[2] == "window")
    return all(e[2] not in ("write", "flush") for e in ev[start + 1:end])


class EmulatedLatency:
    """The latency timer alone stands in; the port is real."""

    def __init__(self) -> None:
        self.ms = 1
        self.sets = []

    def set(self, ms):
        self.sets.append(ms)
        self.ms = ms

    def read(self, serial):
        return (self.ms, self.ms, False)


def emulator(emu: str) -> int:
    with tempfile.TemporaryDirectory() as tmp:
        link = os.path.join(tmp, "tty")
        proc = subprocess.Popen([emu, "--link", link, "--firmware", "47"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            for _ in range(50):
                if os.path.exists(link):
                    break
                time.sleep(0.1)
            expect(os.path.exists(link), "the emulator's link")
            serial = lh.Serial(link, 3000000)
            try:
                latency = EmulatedLatency()
                hold = lh.Hold(serial, latency, lh.Clock(), 2, 50 * MS)
                status = hold.run()
            finally:
                serial.close()
            expect(status == 0, f"emulator: status {status}: {hold.notes}")
            for phase in ("baseline", "zero", "control"):
                kind, detail = hold.result[phase]
                expect(kind == "reply" and "model 1020, firmware 47" in detail, f"emulator {phase}: {detail}")
            expect(hold.result["after_switch"][0] == "none", "nothing after the switch: the reply came at once")
            expect("the reply arrived at 0 ms" in hold.outcome(), hold.outcome())
            expect(latency.sets == [0, 1] and not hold.owed, "0 ms then 1 ms, then verified")
            writes = [e for e in hold.events if e[2] == "write"]
            expect(len(writes) == 3 and all(e[4] == lh.ping_packet(2) for e in writes), "three pings only")
            ts = [e[0] for e in hold.events]
            expect(ts == sorted(ts), "monotonic event times")
        finally:
            proc.terminate()
            proc.wait()
    print("latency hold emulator OK")
    return 0


class FaultySerial(Serial):
    """Fails the n-th write: a return value, or an exception to raise."""

    def __init__(self, a: Adapter, n: int, result) -> None:
        super().__init__(a)
        self.n, self.result, self.attempts = n, result, 0

    def write(self, data):
        self.attempts += 1
        if self.attempts == self.n:
            if isinstance(self.result, BaseException):
                raise self.result
            if self.result > 0:
                self.a.events.append(("write", data[:self.result]))
            return self.result
        return super().write(data)


def write_faults() -> None:
    for n, phase in ((1, "baseline"), (2, "zero")):
        for result, words in ((0, "short write (0 of 10"), (4, "short write (4 of 10"),
                              (OSError(5, "Input/output error"), "the ping was not written")):
            clock = Clock()
            a = Adapter(clock, "held")
            serial = FaultySerial(a, n, result)
            hold = lh.Hold(serial, Latency(a), clock, 2, 50 * MS)
            status = hold.run()
            what = f"{phase} write {result!r}"
            expect(status == 1 and any(words in x for x in hold.notes), f"{what}: {status} {hold.notes}")
            expect(serial.attempts == n, f"{what}: nothing written after the abort ({serial.attempts})")
            expect(a.state() == (1, 1, False) and not hold.owed, f"{what}: 1 ms left")
            expect("zero" not in hold.result if n == 2 else "baseline" not in hold.result,
                   f"{what}: no observation of a ping not sent")
    # A read error is a transport error, with the cleanup done.
    clock = Clock()
    a = Adapter(clock, "held")

    class BadRead(Serial):
        def read(self, timeout_ns):
            if sum(1 for e in a.events if e[0] == "write") >= 2:
                raise OSError(5, "Input/output error")
            return super().read(timeout_ns)
    hold = lh.Hold(BadRead(a), Latency(a), clock, 2, 50 * MS)
    status = hold.run()
    expect(status == 1 and any("a transport error" in x for x in hold.notes), f"read error: {hold.notes}")
    expect(a.state() == (1, 1, False) and any("restored to 1 ms" in x for x in hold.notes), "restored after it")


def through_main(rc=0, pcap="packets", output=None, drift_at=None, manifest=None):
    """main() with the port, the timer, privileges and the recorder replaced
    (and the archive manifest, given its text, or "missing"). Returns its
    status, the record (empty if none), and the order of close and stop."""
    clock = Clock()
    a = Adapter(clock, "held")
    order = []
    if drift_at is not None:
        def drift():
            a.device = 0
            a.set_fail.add(1)
        a.on_write = {drift_at: drift}
    if output is None:
        output = ["3564 packets captured", "3564 packets received by filter", "0 packets dropped by kernel"]

    class Port(Serial):
        def close(self):
            order.append("close")

    class Meta:
        bus, devnum, vid, pid, bcd, channel = 1, 4, "0403", "6014", 0x900, 1

        def read_sysfs(self):
            return a.sysfs

        def read_device(self):
            return a.device

        def write_sysfs(self, ms):
            a.set(ms)

    class Recorder:
        def __init__(self, bus, devnum, path, tcpdump):
            self.path, self.cmd, self.output = path, ["stand-in-tcpdump"], ["listening on usbmon1"]

        def wait_ready(self, seconds):
            return True

        def alive(self):
            return True

        def stop(self):
            order.append("stop")
            if pcap == "packets":
                self.path.write_bytes(b"\0" * 24 + b"\1" * 80)
            elif pcap == "header":
                self.path.write_bytes(b"\0" * 24)
            self.output += output
            return rc

    with tempfile.TemporaryDirectory() as tmp:
        out = io.StringIO()
        with patch.object(lh.os, "geteuid", return_value=0), \
                patch.object(lh.lt, "Adapter", return_value=Meta()), \
                patch.object(lh, "Serial", return_value=Port(a)), \
                patch.object(lh, "Clock", return_value=clock), \
                patch.object(lh, "Recorder", Recorder), \
                patch.object(lh.subprocess, "run", return_value=types.SimpleNamespace(returncode=0)), \
                patch.object(lh.os, "chown"), contextlib.redirect_stdout(out):
            extra = []
            if manifest is not None:
                m = pathlib.Path(tmp, "manifest.md")
                if manifest != "missing":
                    m.write_text(manifest)
                extra = ["--manifest", str(m)]
            status = lh.main(["--yes", "--port", os.path.join(tmp, "tty"), "--outdir", tmp,
                              "--date", "20000101", *extra])
        note = pathlib.Path(tmp, "rate-hold-20000101-1.csv.run.txt")
        record = note.read_text() if note.exists() else ""
    return status, record, order, out.getvalue(), a


def main_orchestration() -> None:
    status, record, order, out, a = through_main()
    expect(status == 0 and "exit status: 0" in record, f"valid evidence: {status}\n{record}")
    expect("outcome: a matching reply arrived only after the switch to 1 ms" in record
           and "INVALID" not in record + out, "the outcome stands")
    expect(order == ["close", "stop"] and a.state() == (1, 1, False), "closed before the recorder stops")
    for kw, words in (({"rc": 7}, "tcpdump exited 7"), ({"pcap": None}, "no capture file"),
                      ({"pcap": "header"}, "the capture holds no packet"),
                      ({"output": ["10 packets captured", "3 packets dropped by kernel"]},
                       "3 packets dropped by kernel"),
                      ({"output": ["10 packets captured"]}, "tcpdump reported no drop count")):
        status, record, order, out, a = through_main(**kw)
        expect(status == 1 and "exit status: 1" in record, f"{kw}: status {status}")
        expect(f"capture: INVALID EVIDENCE: {words}" in record and "outcome: INVALID EVIDENCE; observed:" in record
               and "INVALID EVIDENCE" in out, f"{kw}: {record}")
        expect(order == ["close", "stop"] and "cleanup: latency timer verified at 1 ms" in record
               and a.state() == (1, 1, False), f"{kw}: the restore and close come first")
    # A cleanup failure keeps status 3 with invalid evidence too.
    status, record, order, out, a = through_main(rc=7, drift_at=3)
    expect(status == 3 and "NOT restored" in record and "INVALID EVIDENCE" in record, f"3 kept: {status}")


def main() -> int:
    if len(sys.argv) == 3 and sys.argv[1] == "--emulator":
        return emulator(sys.argv[2])
    # The ping and the payload classes.
    expect(lh.ping_packet(2).hex() == "fffffd00020300011972", "the ping to id 2, as captured")
    expect(lh.crc16(reply()[:-2]) == 0xF312, "the captured reply's CRC")
    kind, detail = lh.classify(reply(), 2)
    expect(kind == "reply" and "model 1020, firmware 47" in detail, f"a reply: {detail}")
    expect(lh.classify(b"", 2)[0] == "none", "no payload")
    expect(lh.classify(reply()[:6], 2)[0] == "partial", "a cut reply")
    expect(lh.classify(b"\xff\xff", 2)[0] == "partial", "the start of a header")
    expect(lh.classify(reply(crc_ok=False), 2)[0] == "corrupt", "a bad CRC")
    expect(lh.classify(reply(3), 2)[0] == "corrupt", "a reply from another servo")
    expect(lh.classify(b"\x00\x01\x02", 2)[0] == "corrupt", "bytes without a header")
    expect("2 other bytes" in lh.classify(b"\x00\x00" + reply(), 2)[1], "bytes around a reply")

    # A reply held at 0 ms and delivered after the switch.
    status, hold, a = run("held")
    expect(status == 0, f"held: status {status}")
    expect(hold.result["zero"][0] == "none" and hold.result["after_switch"][0] == "reply"
           and hold.result["control"][0] == "reply", f"held: {hold.result}")
    expect("delayed delivery" in hold.outcome(), hold.outcome())
    expect(only_pings(a) and sum(1 for e in a.events if e[0] == "write") == 3, "three pings, nothing else")
    expect(quiet_after_zero_ping(hold), "nothing sent or flushed between the 0 ms ping and the switch window")
    expect([e[1] for e in a.events if e[0] == "set"] == [0, 1], "0 ms then 1 ms, set once each")
    expect(a.state() == (1, 1, False) and not hold.owed, "1 ms at the end")
    expect(any(n.startswith("cleanup: latency timer verified at 1 ms") for n in hold.notes), "verified")
    with tempfile.TemporaryDirectory() as tmp:
        path = pathlib.Path(tmp) / "hold.csv"
        hold.write_csv(path)
        lines = path.read_text().splitlines()
        expect(lines[0] == "t_ns,phase,event,detail,bytes_hex", "the CSV header")
        expect(any(",after_switch,read," in x and x.endswith(reply().hex()) for x in lines),
               "the deferred reply's raw bytes in the CSV")

    # The other payloads.
    status, hold, _ = run("none")
    expect(status == 0 and "no payload at 0 ms or after the switch" in hold.outcome()
           and "control ping at 1 ms: reply" in hold.outcome(), hold.outcome())
    status, hold, _ = run("partial")
    expect(status == 0 and hold.result["zero"][0] == "partial", f"partial: {hold.result}")
    status, hold, _ = run("corrupt")
    expect(status == 0 and hold.result["zero"][0] == "corrupt", f"corrupt: {hold.result}")
    status, hold, _ = run("immediate")
    expect(status == 0 and "the reply arrived at 0 ms" in hold.outcome(), hold.outcome())

    # A setting that does not read back, or cannot be written, aborts; the
    # cleanup leaves 1 ms.
    status, hold, a = run("held", stuck=1)
    expect(status == 1 and any("does not hold 0 ms" in n for n in hold.notes), f"stuck: {hold.notes}")
    expect(sum(1 for e in a.events if e[0] == "write") == 1, "no ping after a failed 0 ms readback")
    expect(a.state() == (1, 1, False) and any("restored to 1 ms" in n for n in hold.notes), "restored")
    status, hold, a = run("held", set_fail={0})
    expect(status == 1 and a.state() == (1, 1, False), "a write failure aborts with 1 ms kept")
    expect(any("verified at 1 ms" in n for n in hold.notes), "verified after the failed write")

    # A completed sequence whose cleanup cannot leave 1 ms exits 3.
    clock = Clock()
    a = Adapter(clock, "held")

    def drift():
        a.device = 0
        a.set_fail.add(1)
    a.on_write = {3: drift}
    hold = lh.Hold(Serial(a), Latency(a), clock, 2, 50 * MS)
    status = hold.run()
    expect(status == 3 and any("NOT restored to 1 ms" in n for n in hold.notes), f"cleanup failure: {hold.notes}")
    # The same failure after an abort keeps status 1.
    status, hold, a = run("held", set_fail={1})
    expect(status == 1 and any("NOT restored" in n for n in hold.notes), "an abort keeps its status")

    # An interruption in the 0 ms window: status 130, 1 ms restored.
    status, hold, a = run("held", interrupt_on_write=2)
    expect(status == 130 and "interrupted" in hold.notes, f"interrupted: {hold.notes}")
    expect(a.state() == (1, 1, False) and any("restored to 1 ms" in n for n in hold.notes), "restored after Ctrl-C")

    # Refused before anything is sent, and a silent baseline.
    for kw in ({"start": 0}, {"low": True}):
        status, hold, a = run("held", **kw)
        expect(status == 2 and not any(e[0] in ("write", "set") for e in a.events), f"refused {kw}")
    status, hold, a = run("silent")
    expect(status == 1 and any("baseline ping had no matching reply" in n for n in hold.notes), "silent baseline")
    expect(not any(e[0] == "set" for e in a.events) and not hold.owed, "the timer untouched")
    expect(any(n.startswith("cleanup: latency timer verified at 1 ms") for n in hold.notes),
           "the silent baseline's cleanup verifies 1 ms")
    # A baseline that changes the timer is caught by the cleanup, owed from
    # the verified start on.
    clock = Clock()
    a = Adapter(clock, "silent")

    def to16():
        a.sysfs = a.device = 16
    a.on_write = {1: to16}
    hold = lh.Hold(Serial(a), Latency(a), clock, 2, 50 * MS)
    status = hold.run()
    expect(status == 1 and a.state() == (1, 1, False), f"baseline drift: {status} {a.state()}")
    expect([e[1] for e in a.events if e[0] == "set"] == [1] and hold.cleanup_ok
           and any("restored to 1 ms" in n for n in hold.notes), f"restored after the drift: {hold.notes}")

    write_faults()
    main_orchestration()

    # An output name already in the archive manifest, through a sidecar's
    # bare row, a directory-qualified entry or a directory-qualified citation,
    # or a missing manifest, is refused before anything is sent or set;
    # another attempt's names (2 and 10, not 1) are not.
    row = "| `rate-hold-20000101-1.csv.usbmon.pcap` | archived | `0` |\n"
    qualified = "| `latency_timer/rate-hold-20000101-1.csv.run.txt` | archived | `0` |\n"
    cited = "as cited in latency_timer/rate-hold-20000101-1.csv.\n"
    for manifest, what in ((row, "a bare row"), (qualified, "a qualified entry"), (cited, "a qualified citation"),
                           ("missing", "a missing manifest")):
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            status, record, order, out, a = through_main(manifest=manifest)
        expect(status == 2 and record == "" and order == [] and a.state() == (1, 1, False)
               and a.events == [], f"{what}: refused untouched ({status}, {order}, {a.state()})")
        expect(("no archive manifest at" if manifest == "missing" else "is already in the archive manifest")
               in err.getvalue(), f"{what}: {err.getvalue()}")
    for other in ("20000101-2", "20000101-10"):
        status, *_ = through_main(manifest=row.replace("20000101-1", other) + qualified.replace("20000101-1", other))
        expect(status == 0, f"a free name runs beside attempt {other}: {status}")

    # The command line refuses without root, and bad arguments.
    if lh.os.geteuid() != 0:
        expect(lh.main(["--yes"]) == 2, "refused without root")
    expect(lh.main(["--window-ms", "5"]) == 2, "a window below 10 ms is refused")
    print("latency hold regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
