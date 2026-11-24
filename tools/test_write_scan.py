#!/usr/bin/env python3
"""Regressions for tools/write_scan.py on synthetic usbmon captures (link type 220), with known answers.

- the default report: a probe run's startup and cycles pass; a write to Torque Enable or Drive Mode, an indirect slot
  pointing at either, and a CRC failure each fail; byte stuffing in a write is removed before decoding;
- the per-cycle check (--cycles N), for a subset read set and for every servo:
  - N cycles of a Fast Sync Read of the read set and a zero Goal Current Sync Write to every servo pass;
  - a write missing one servo, listing them out of order, carrying a nonzero current, at another address, a cycle
    missing, an extra cycle, a read of another set, a read where a write belongs, no initial read, and a grouped
    instruction after the last cycle each fail;
- the command line: exit 2 on an unreadable capture or a bad option."""

from __future__ import annotations

import io
import pathlib
import struct
import sys
import tempfile
from contextlib import redirect_stdout

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import write_scan as ws  # noqa: E402

FAILED = 0
ALL = [2, 3, 4, 5, 6, 7, 8, 9]
SUBSET = [2, 3, 4, 5]


def check(cond: bool, what: str, detail: object = "") -> None:
    global FAILED
    if not cond:
        FAILED += 1
        print(f"FAILED: {what}\n{detail}")


def packet(ident: int, inst: int, params: bytes, corrupt: bool = False) -> bytes:
    """A Dynamixel 2.0 instruction packet, byte-stuffed, its CRC over the stuffed packet."""
    body = bytes([inst]) + params
    body = body.replace(b"\xff\xff\xfd", b"\xff\xff\xfd\xfd")
    head = b"\xff\xff\xfd\x00" + bytes([ident]) + struct.pack("<H", len(body) + 2)
    crc = ws.crc16(head + body) ^ (1 if corrupt else 0)
    return head + body + struct.pack("<H", crc)


def read(ident: int, addr: int, n: int) -> bytes:
    return packet(ident, ws.READ, struct.pack("<HH", addr, n))


def write(ident: int, addr: int, data: bytes) -> bytes:
    return packet(ident, ws.WRITE, struct.pack("<H", addr) + data)


def grouped_read(inst: int, addr: int, n: int, ids: list[int]) -> bytes:
    return packet(0xFE, inst, struct.pack("<HH", addr, n) + bytes(ids))


def sync_write(addr: int, n: int, entries: list[tuple[int, bytes]]) -> bytes:
    return packet(0xFE, ws.SYNC_WRITE, struct.pack("<HH", addr, n) + b"".join(bytes([i]) + d for i, d in entries))


def zero_write(ids: list[int] = ALL) -> bytes:
    return sync_write(ws.GOAL_CURRENT, ws.GOAL_LEN, [(i, b"\x00\x00") for i in ids])


def startup() -> list[bytes]:
    """The probe's startup: pings, register reads, the indirect setup, the initial Sync Read of every servo."""
    out = [packet(i, ws.PING, b"") for i in ALL]
    out += [read(i, a, 1) for i in ALL for a in (ws.TORQUE, ws.DRIVE)]
    out += [write(i, 168 + 2 * s, struct.pack("<H", 126 + s)) for i in ALL for s in range(3)]
    out.append(grouped_read(ws.SYNC_READ, 224, 13, ALL))
    return out


def after() -> list[bytes]:
    return [read(i, a, 1) for i in ALL for a in (ws.TORQUE, ws.DRIVE)]


def cycles(n: int, read_ids: list[int], length: int = 10) -> list[bytes]:
    out = []
    for _ in range(n):
        out += [grouped_read(ws.FAST_SYNC_READ, 224, length, read_ids), zero_write()]
    return out


def pcap(payloads: list[bytes], path: pathlib.Path) -> pathlib.Path:
    """Each payload one bulk OUT submission ('S', transfer type 3, endpoint 2) of device 3, as usbmon records it."""
    out = bytearray(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 220))
    for k, data in enumerate(payloads):
        rec = bytearray(64)
        rec[8], rec[9], rec[10], rec[11] = ord("S"), 3, 2, 3
        struct.pack_into("<I", rec, 32, len(data))
        struct.pack_into("<I", rec, 36, len(data))
        rec += data
        out += struct.pack("<IIII", k, 0, len(rec), len(rec)) + rec
    path.write_bytes(bytes(out))
    return path


def run(payloads: list[bytes], *args: str) -> tuple[int, str]:
    with tempfile.TemporaryDirectory() as d:
        p = pcap(payloads, pathlib.Path(d) / "c.pcap")
        buf = io.StringIO()
        with redirect_stdout(buf):
            st = ws.main([str(p), *args])
        return st, buf.getvalue()


def main() -> int:
    good = startup() + cycles(5, SUBSET) + after()
    st, out = run(good)
    check(st == 0, "a subset run's capture passes the default report", out)
    check("writes covering Torque Enable (64): 0; covering Drive Mode (10): 0; indirect slots pointing at 64 or 10: "
          "none" in out, "the default report's last line", out)
    check("per cycle" not in out, "no per-cycle section without --cycles", out)
    st, out = run(good, "--cycles", "5", "--read-ids", "2,3,4,5")
    check(st == 0, "five cycles of a subset read and a zero write to all eight pass", out)
    check("Fast Sync Reads of [2, 3, 4, 5]: 5; Sync Writes of Goal Current to [2, 3, 4, 5, 6, 7, 8, 9]: 5, of them all "
          "zero: 5; failures 0" in out, "the per-cycle counts", out)
    st, out = run(startup() + cycles(4, ALL) + after(), "--cycles", "4")
    check(st == 0, "every servo read by default passes", out)
    st, out = run(good, "--cycles", "5")
    check(st == 1 and "not a Fast Sync Read of [2, 3, 4, 5, 6, 7, 8, 9]" in out,
          "a subset run checked as every servo fails", out)

    def broken(k: int, replacement: bytes) -> list[bytes]:
        """good with cycle k's (1-based) write replaced."""
        p = list(good)
        p[len(startup()) + 2 * (k - 1) + 1] = replacement
        return p
    for what, rep in (
            ("a write missing servo 9", sync_write(237, 2, [(i, b"\x00\x00") for i in ALL[:-1]])),
            ("a write out of order", sync_write(237, 2, [(i, b"\x00\x00") for i in reversed(ALL)])),
            ("a nonzero goal current", sync_write(237, 2, [(i, b"\x01\x00" if i == 7 else b"\x00\x00") for i in ALL])),
            ("a write at another address", sync_write(239, 2, [(i, b"\x00\x00") for i in ALL])),
            ("a read where the write belongs", grouped_read(ws.FAST_SYNC_READ, 224, 10, SUBSET))):
        st, out = run(broken(3, rep), "--cycles", "5", "--read-ids", "2,3,4,5")
        check(st == 1 and "cycle 3:" in out, f"{what} fails at cycle 3", out)
    st, out = run(startup() + cycles(4, SUBSET) + after(), "--cycles", "5", "--read-ids", "2,3,4,5")
    check(st == 1 and "not 2 x 5 cycles" in out, "a cycle missing fails", out)
    st, out = run(startup() + cycles(6, SUBSET) + after(), "--cycles", "5", "--read-ids", "2,3,4,5")
    check(st == 1 and "not 2 x 5 cycles" in out, "an extra cycle fails", out)
    other = startup() + cycles(2, SUBSET) + cycles(1, [6, 7, 8, 9]) + cycles(2, SUBSET) + after()
    st, out = run(other, "--cycles", "5", "--read-ids", "2,3,4,5")
    check(st == 1 and "cycle 3: its read is 0x8a listing [6, 7, 8, 9]" in out, "a read of another set fails", out)
    st, out = run(startup()[:-1] + cycles(5, SUBSET) + after(), "--cycles", "5", "--read-ids", "2,3,4,5")
    check(st == 1 and "no initial Sync Read" in out, "no initial read fails", out)
    st, out = run(good + [zero_write()], "--cycles", "5", "--read-ids", "2,3,4,5")
    check(st == 1, "a grouped instruction after the last cycle fails", out)
    st, out = run(startup() + [write(5, 100, b"\x00")] + cycles(5, SUBSET) + after(), "--cycles", "5",
                  "--read-ids", "2,3,4,5")
    check(st == 0, "a single write before the initial read is startup traffic", out)
    st, out = run(good[:-1] + [write(5, 100, b"\x00")], "--cycles", "5", "--read-ids", "2,3,4,5")
    check(st == 1 and "after the last cycle, instructions other than single reads" in out,
          "a single write after the last cycle fails", out)

    # The default report's own rules.
    st, out = run(good + [write(4, ws.TORQUE, b"\x01")])
    check(st == 1 and "writes covering Torque Enable (64): 1" in out, "a Torque Enable write fails", out)
    st, out = run(good + [write(4, 8, b"\x00\x00\x00")])
    check(st == 1 and "covering Drive Mode (10): 1" in out, "a write spanning Drive Mode fails", out)
    st, out = run(good + [write(4, 168 + 2 * 5, struct.pack("<H", ws.TORQUE))])
    check(st == 1 and "indirect slots pointing at 64 or 10: [64]" in out, "an indirect slot at Torque Enable fails", out)
    st, out = run(good + [packet(4, ws.WRITE, struct.pack("<H", 100) + b"\x00", corrupt=True)])
    check(st == 1 and "CRC failures 1" in out, "a CRC failure fails", out)
    stuffed = write(4, 300, b"\xff\xff\xfd\x00")
    st, out = run(good + [stuffed])
    check(st == 0 and "Write      address  300 length   4: 1" in out, "byte stuffing is removed before decoding", out)

    with tempfile.TemporaryDirectory() as d:
        p = pathlib.Path(d) / "bad.pcap"
        p.write_bytes(b"\x00" * 30)
        buf = io.StringIO()
        with redirect_stdout(buf):
            check(ws.main([str(p)]) == 2, "an unreadable capture exits 2")
            check(ws.main([str(pathlib.Path(d) / "none.pcap")]) == 2, "a missing capture exits 2")
            q = pcap(good, pathlib.Path(d) / "c.pcap")
            check(ws.main([str(q), "--cycles", "0"]) == 2, "--cycles 0 exits 2")
            check(ws.main([str(q), "--cycles", "5", "--read-ids", "2,x"]) == 2, "a bad read set exits 2")

    if FAILED:
        print(f"write scan regressions: {FAILED} failed")
        return 1
    print("write scan regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
