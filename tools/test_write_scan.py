#!/usr/bin/env python3
"""Regressions for tools/write_scan.py on synthetic usbmon captures (link type 220), with known answers.

The captures follow the probe's traffic: pings, register reads and the indirect setup (SyncGroup::setupIndirect's 23
slots per servo, written out here independently of the tool), the initial Sync Read of every servo, the cycles, and
the torque checks after.
- the default report: a probe run passes; a Torque Enable or Drive Mode write, an indirect slot pointing at either, a
  CRC failure and a malformed Sync Write or Write each fail; byte stuffing in a write is removed before decoding;
- the whole sequence (--cycles N): subset, single-servo, eight-servo and full-window runs pass, and a recovery read one
  byte longer; and each of these fails:
  - an extra packet anywhere (startup, between the initial read and the first cycle, inside a cycle's pair, between
    cycles, after the last cycle): a watchdog write, a nonzero direct current, a Reg Write, an Action, a Sync Write,
    a Bulk Write, a ping, an extra read;
  - a recipient other than the broadcast for the initial read, a cyclic read or a cyclic write; a startup or
    after-check packet that is not unicast to a servo;
  - a parameter block of the wrong length: a Sync Write with one or two bytes left over, a short header, a Fast Sync
    Read listing more or fewer servos, a read of another length or window;
  - an incomplete, repeated or altered indirect setup; a missing torque check before or after;
  - a cycle missing, an extra cycle, a CRC failure, bytes outside any packet;
- evidence that cannot be read (a truncated record, a transfer captured short, a packet cut off at the end of its
  transfer, not a pcap) exits 2, never 1 or 0; a bad option exits 2."""

from __future__ import annotations

import io
import pathlib
import struct
import sys
import tempfile
from contextlib import redirect_stderr, redirect_stdout

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import write_scan as ws  # noqa: E402

FAILED = 0
ALL = [2, 3, 4, 5, 6, 7, 8, 9]
SUBSET = [2, 3, 4, 5]
# SyncGroup::setupIndirect (src/dxl/sync_group.cpp): Present Current, Velocity, Position, Input Voltage, Temperature,
# Goal Current, Velocity, Position, one source address per indirect slot.
SOURCES = [126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 144, 145, 146, 102, 103, 104, 105, 106, 107, 116, 117,
           118, 119]


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


def grouped_read(inst: int, n: int, ids: list[int], target: int = 0xFE, addr: int = 224) -> bytes:
    return packet(target, inst, struct.pack("<HH", addr, n) + bytes(ids))


def sync_write(addr: int, n: int, entries: list[tuple[int, bytes]], target: int = 0xFE, tail: bytes = b"") -> bytes:
    return packet(target, ws.SYNC_WRITE, struct.pack("<HH", addr, n) + b"".join(bytes([i]) + d for i, d in entries) + tail)


def zero_write(ids: list[int] = ALL, target: int = 0xFE, tail: bytes = b"") -> bytes:
    return sync_write(237, 2, [(i, b"\x00\x00") for i in ids], target, tail)


def setup(ids: list[int] = ALL) -> list[bytes]:
    return [write(i, 168 + 2 * k, struct.pack("<H", src)) for i in ids for k, src in enumerate(SOURCES)]


def pre() -> list[bytes]:
    """The startup before the initial read: pings, the inventory's and the checks' reads, the indirect setup."""
    out = [packet(i, ws.PING, b"") for i in ALL]
    out += [read(i, a, 1) for i in ALL for a in (6, 8, 9, 10, 11, 64, 68, 70, 98)]
    return out + setup()


def initial() -> bytes:
    return grouped_read(ws.SYNC_READ, 13, ALL)


def after() -> list[bytes]:
    return [read(i, a, 1) for i in ALL for a in (64, 10)]


def cycles(n: int, read_ids: list[int], length: int = 10) -> list[bytes]:
    out = []
    for _ in range(n):
        out += [grouped_read(ws.FAST_SYNC_READ, length, read_ids), zero_write()]
    return out


def run_of(n: int, read_ids: list[int], length: int = 10) -> list[bytes]:
    return pre() + [initial()] + cycles(n, read_ids, length) + after()


def pcap(payloads: list[bytes], path: pathlib.Path, short: int | None = None, cut: int | None = None) -> pathlib.Path:
    """Each payload one bulk OUT submission ('S', transfer type 3, endpoint 2) of device 3, as usbmon records it.
    short: that record's transfer captured with one byte less than its length; cut: the file cut off inside that
    record."""
    out = bytearray(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, 220))
    for k, data in enumerate(payloads):
        rec = bytearray(64)
        rec[8], rec[9], rec[10], rec[11] = ord("S"), 3, 2, 3
        captured = data[:-1] if k == short else data
        struct.pack_into("<I", rec, 32, len(data))
        struct.pack_into("<I", rec, 36, len(captured))
        rec += captured
        if k == cut:
            out += struct.pack("<IIII", k, 0, len(rec), len(rec)) + rec[: len(rec) // 2]
            break
        out += struct.pack("<IIII", k, 0, len(rec), len(rec)) + rec
    path.write_bytes(bytes(out))
    return path


def run(payloads: list[bytes], *args: str, **kw) -> tuple[int, str]:
    with tempfile.TemporaryDirectory() as d:
        p = pcap(payloads, pathlib.Path(d) / "c.pcap", **kw)
        buf, err = io.StringIO(), io.StringIO()
        with redirect_stdout(buf), redirect_stderr(err):
            try:
                st = ws.main([str(p), *args])
            except SystemExit as e:  # a scanner whose option parser exits instead of returning
                st = e.code if isinstance(e.code, int) else 2
        return st, buf.getvalue() + err.getvalue()


def strict(payloads: list[bytes], n: int = 3, read_ids: list[int] = SUBSET, *extra: str, **kw) -> tuple[int, str]:
    return run(payloads, "--cycles", str(n), "--read-ids", ",".join(map(str, read_ids)), *extra, **kw)


def main() -> int:
    # The valid runs.
    for name, ids, length in (("four", SUBSET, 10), ("one", [2], 10), ("eight", ALL, 10), ("four, full window",
                                                                                           SUBSET, 13)):
        st, out = strict(run_of(3, ids, length), 3, ids)
        check(st == 0 and "failures 0" in out, f"a valid {name}-servo run passes the whole sequence", out)
    st, out = strict(run_of(3, SUBSET, 10), 3, SUBSET, "--read-len", "10")
    check(st == 0 and "Fast Sync Reads of [2, 3, 4, 5]: 3; Sync Writes of Goal Current to [2, 3, 4, 5, 6, 7, 8, 9]: "
          "3, of them all zero: 3; failures 0" in out, "the counts, with --read-len", out)
    good = run_of(3, SUBSET)
    st, out = run(good)
    check(st == 0 and "the whole sequence" not in out, "the default report alone: no sequence check", out)
    check(out.splitlines()[-1] == "writes covering Torque Enable (64): 0; covering Drive Mode (10): 0; indirect slots "
          "pointing at 64 or 10: none", "the default report's last line, unchanged", out)
    k0 = len(pre()) + 1  # the first cycle's read

    def at(seq: list[bytes], k: int, *new: bytes, drop: int = 0) -> list[bytes]:
        return seq[:k] + list(new) + seq[k + drop:]
    recovery = at(good, k0 + 2, grouped_read(ws.FAST_SYNC_READ, 11, SUBSET), drop=1)
    check(strict(recovery, 3, SUBSET, "--read-len", "10")[0] == 0, "a recovery read one byte longer passes")
    check(strict(at(good, k0 + 2, grouped_read(ws.FAST_SYNC_READ, 14, SUBSET), drop=1), 3, SUBSET, "--read-len",
                 "10")[0] == 1, "with --read-len 10, a 14-byte read fails")
    check(strict(at(good, k0, grouped_read(ws.FAST_SYNC_READ, 12, SUBSET), drop=1))[0] == 1, "a 12-byte read fails")
    check(strict(at(good, k0, grouped_read(ws.FAST_SYNC_READ, 10, SUBSET, addr=226), drop=1))[0] == 1,
          "a read of another window fails")

    # An extra packet at each position fails.
    extras = {"a watchdog write": write(2, 98, b"\x01"), "a nonzero direct current": write(2, 102, b"\x01\x00"),
              "a Reg Write": packet(2, 0x04, struct.pack("<H", 102) + b"\x01\x00"), "an Action": packet(0xFE, 0x05, b""),
              "a Sync Write": zero_write(), "a Bulk Write": packet(0xFE, 0x93, bytes([2]) + struct.pack("<HH", 102, 2)
                                                                   + b"\x00\x00"),
              "a ping": packet(2, ws.PING, b""), "an extra read": read(2, 64, 1)}
    positions = {"in the startup": 3, "between the initial read and the first cycle": k0,
                 "inside a cycle's pair": k0 + 1, "between cycles": k0 + 2, "after the last cycle": len(good) - 1}
    for where, k in positions.items():
        for what, p in extras.items():
            if what in ("a ping", "an extra read") and where in ("in the startup", "after the last cycle"):
                continue  # pings and reads are startup traffic, and reads are after-checks
            st, out = strict(at(good, k, p))
            check(st == 1, f"{what} {where} fails", out)
    for what in ("a ping", "an extra read of the watchdog"):
        p = packet(2, ws.PING, b"") if what == "a ping" else read(2, 98, 1)
        st, out = strict(good + [p])
        check(st == 1 and "after the last cycle" in out, f"{what} after the last cycle fails", out)

    # Recipients.
    check(strict(at(good, k0 - 1, grouped_read(ws.SYNC_READ, 13, ALL, target=2), drop=1))[0] == 1,
          "a unicast initial Sync Read fails")
    check(strict(at(good, k0, grouped_read(ws.FAST_SYNC_READ, 10, SUBSET, target=2), drop=1))[0] == 1,
          "a unicast cyclic Fast Sync Read fails")
    check(strict(at(good, k0 + 1, zero_write(target=2), drop=1))[0] == 1, "a unicast Sync Write fails")
    check(strict(at(good, 0, packet(0xFE, ws.PING, b""), drop=1))[0] == 1, "a broadcast ping in the startup fails")
    check(strict(at(good, 9, read(0xFE, 64, 1), drop=1))[0] == 1, "a broadcast read in the startup fails")
    check(strict(at(good, len(good) - 1, read(0xFE, 10, 1), drop=1))[0] == 1, "a broadcast after-check read fails")
    check(strict(at(good, 9, read(12, 64, 1)))[0] == 1, "a read of a servo not configured fails")

    # Parameter lengths.
    for what, p in (("one byte left over", zero_write(tail=b"\x09")), ("two bytes left over", zero_write(tail=b"\x09\x00")),
                    ("a short header", packet(0xFE, ws.SYNC_WRITE, struct.pack("<H", 237))),
                    ("one servo short", zero_write(ALL[:-1])), ("out of order", zero_write(list(reversed(ALL)))),
                    ("another address", sync_write(239, 2, [(i, b"\x00\x00") for i in ALL])),
                    ("a nonzero current", sync_write(237, 2, [(i, b"\x01\x00" if i == 7 else b"\x00\x00")
                                                              for i in ALL]))):
        st, out = strict(at(good, k0 + 1, p, drop=1))
        check(st == 1 and "cycle 1:" in out, f"a cyclic Sync Write with {what} fails at cycle 1", out)
    for what, ids in (("an extra servo", [*SUBSET, 6]), ("a servo short", SUBSET[:-1]), ("another set", [6, 7, 8, 9])):
        st, out = strict(at(good, k0, grouped_read(ws.FAST_SYNC_READ, 10, ids), drop=1))
        check(st == 1 and "cycle 1:" in out, f"a cyclic read listing {what} fails", out)
    check(strict(at(good, k0 - 1, grouped_read(ws.SYNC_READ, 13, ALL[:-1]), drop=1))[0] == 1,
          "an initial read of seven servos fails")
    check(strict(at(good, k0 - 1, grouped_read(ws.SYNC_READ, 10, ALL), drop=1))[0] == 1,
          "an initial read of the motion-only window fails")
    st, out = strict(at(good, 9, packet(2, ws.READ, struct.pack("<H", 64)), drop=1))
    check(st == 1, "a read with a short parameter block fails", out)
    st, out = strict(at(good, len(good) - 1, read(9, 10, 2), drop=1))
    check(st == 1, "an after-check read of two bytes fails", out)

    # The startup's setup and checks.
    n_pre = len(pre())
    check(strict(at(good, n_pre - 1, drop=1))[0] == 1, "an incomplete indirect setup (one slot missing) fails")
    check(strict(at(good, n_pre, setup()[0]))[0] == 1, "a slot written twice fails")
    check(strict(at(good, n_pre - 1, write(9, 168 + 2 * 22, struct.pack("<H", 64)), drop=1))[0] == 1,
          "a slot pointing at Torque Enable fails")
    check(strict(at(good, n_pre - 1, write(9, 168 + 2 * 22, struct.pack("<H", 118)), drop=1))[0] == 1,
          "a slot with another source fails")
    no_torque = [p for p in pre() if p != read(5, 64, 1)] + [initial()] + cycles(3, SUBSET) + after()
    st, out = strict(no_torque)
    check(st == 1 and "before the run, register 64 of servos [5] not read" in out, "a torque check missing before", out)
    st, out = strict(good[:-1])
    check(st == 1 and "after the run, register 10 of servos [9] not read" in out, "a torque check missing after", out)

    # Counts, CRC and stray bytes.
    st, out = strict(pre() + [initial()] + cycles(2, SUBSET) + after())
    check(st == 1 and "cycle 3:" in out, "a cycle missing fails at cycle 3", out)
    st, out = strict(pre() + [initial()] + cycles(2, SUBSET))
    check(st == 1 and "the capture ends before cycle 3 of 3" in out, "a capture ending early fails", out)
    check(strict(pre() + [initial()] + cycles(4, SUBSET) + after())[0] == 1, "an extra cycle fails")
    st, out = strict(at(good, 5, packet(2, ws.READ, struct.pack("<HH", 64, 1), corrupt=True), drop=1))
    check(st == 1 and "CRC failures 1" in out, "a CRC failure fails", out)
    st, out = strict(at(good, k0, b"\x00\x01" + grouped_read(ws.FAST_SYNC_READ, 10, SUBSET), drop=1))
    check(st == 1 and "2 bytes sent outside any packet" in out, "stray bytes fail the sequence", out)
    check(run(at(good, k0, b"\x00\x01" + grouped_read(ws.FAST_SYNC_READ, 10, SUBSET), drop=1))[0] == 0,
          "stray bytes are not the default report's concern")

    # The default report's own rules.
    st, out = run(good + [write(4, ws.TORQUE, b"\x01")])
    check(st == 1 and "writes covering Torque Enable (64): 1" in out, "a Torque Enable write fails", out)
    st, out = run(good + [write(4, 8, b"\x00\x00\x00")])
    check(st == 1 and "covering Drive Mode (10): 1" in out, "a write spanning Drive Mode fails", out)
    st, out = run(good + [write(4, 168 + 2 * 5, struct.pack("<H", ws.TORQUE))])
    check(st == 1 and "indirect slots pointing at 64 or 10: [64]" in out, "an indirect slot at Torque Enable fails", out)
    st, out = run(good + [packet(4, ws.WRITE, struct.pack("<H", 100) + b"\x00", corrupt=True)])
    check(st == 1 and "CRC failures 1" in out, "a CRC failure fails", out)
    st, out = run(good + [zero_write(tail=b"\x09")])
    check(st == 1 and "malformed write instructions: 1" in out, "a malformed Sync Write fails the default report", out)
    st, out = run(good + [packet(4, ws.WRITE, struct.pack("<H", 100))])
    check(st == 1 and "malformed write instructions: 1" in out, "a Write without data fails the default report", out)
    st, out = run(good + [write(4, 300, b"\xff\xff\xfd\x00")])
    check(st == 0 and "Write      address  300 length   4: 1" in out, "byte stuffing is removed before decoding", out)

    # Unreadable evidence exits 2.
    check(run(good, short=len(good) // 2)[0] == 2, "a transfer captured short exits 2")
    check(run(good, cut=len(good) // 2)[0] == 2, "a record cut off exits 2")
    check(run(good[:-1] + [good[-1][:-3]])[0] == 2, "a packet cut off at the end of its transfer exits 2")
    with tempfile.TemporaryDirectory() as d:
        p = pathlib.Path(d) / "bad.pcap"
        p.write_bytes(b"\x00" * 30)
        q = pcap(good, pathlib.Path(d) / "c.pcap")
        buf = io.StringIO()
        with redirect_stdout(buf), redirect_stderr(buf):
            check(ws.main([str(p)]) == 2, "not a pcap exits 2")
            check(ws.main([str(pathlib.Path(d) / "none.pcap")]) == 2, "a missing capture exits 2")
            check(ws.main([str(q), "--cycles", "0"]) == 2, "--cycles 0 exits 2")
            check(ws.main([str(q), "--cycles", "3", "--read-ids", "2,x"]) == 2, "a bad read set exits 2")
            check(ws.main([str(q), "--cycles", "3", "--read-ids", "2,2"]) == 2, "a repeated ID exits 2")
            check(ws.main([str(q), "--cycles", "3", "--read-len", "12"]) == 2, "a read length other than 10 or 13 exits 2")

    if FAILED:
        print(f"write scan regressions: {FAILED} failed")
        return 1
    print("write scan regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
