#!/usr/bin/env python3
"""Regressions for latency_hold.py, run under ctest, without hardware.

A stand-in adapter plays the latency timer, the servo and the host's input
on a simulated monotonic clock, and the real sequence runs on it: a reply
held at 0 ms and delivered after the switch to 1 ms, no payload, a partial
and a corrupt payload, a reply at 0 ms, a setting that does not read back
or cannot be written, a cleanup that cannot restore 1 ms, an interruption,
a refusal and a silent baseline. Pinned: the Protocol 2.0 ping and its
CRC, the payload classes, that only pings are sent, that nothing is sent
or flushed between the 0 ms ping and the end of the window after the
switch, the cleanup and the exit statuses.

With --emulator DXL_EMU, the real serial port class runs instead, on
dxl_emu's pseudo-terminal (only the latency timer stands in): its
configuration, exclusive open, reads against monotonic deadlines and
flushes, with the emulated servo answering every ping.
"""

import importlib.util
import os
import pathlib
import subprocess
import sys
import tempfile
import time

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

    # The command line refuses without root, and bad arguments.
    if lh.os.geteuid() != 0:
        expect(lh.main(["--yes"]) == 2, "refused without root")
    expect(lh.main(["--window-ms", "5"]) == 2, "a window below 10 ms is refused")
    print("latency hold regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
