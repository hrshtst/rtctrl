#!/usr/bin/env python3
"""Every Dynamixel write the host sent in a usbmon capture of the adapter, and, for a bus timing probe run, its
per-cycle traffic (read-only).

Reads a pcap of link type 220 (USB_LINUX_MMAPPED, as tcpdump -i usbmonN writes it), takes the bulk OUT submissions
(the bytes the host sent to the FTDI adapter), splits them into Dynamixel protocol 2.0 instruction packets (byte
stuffing removed, CRC checked), and lists every write instruction (Write, Reg Write, Sync Write, Bulk Write) by
address range. It reports whether any write covers Torque Enable (64) or Drive Mode (10), and decodes the
indirect-address programming (indirect addresses 1-28 at 168-223 and 29-56 at 578-633), so a torque write through
an indirect data slot would also show. This is the full-feedback candidate's write_scan.py
(review-notes/2026-11-24-ff-candidate/, sealed), its report unchanged, made a tracked tool.

With --cycles N (a torque-off x7_bus_timing run of profile current and N log rows; the traffic-reduction
diagnostic's write scan, 2026-11-24), it also checks the run's grouped instructions in the order sent: the initial
Sync Read of every servo (--write-ids), then N cycles, each one Fast Sync Read listing exactly the read set
(--read-ids, by default every servo) and one Sync Write of zero goal current (address 237, 2 bytes per servo)
listing exactly every servo, in order; and no Sync Write or grouped read besides. Before the initial read only
pings, single reads and single writes; after the last cycle only single reads (the torque checks).

    python3 -I -B write_scan.py CAPTURE.pcap [--cycles N [--read-ids 2,3,4,5] [--write-ids 2,3,4,5,6,7,8,9]]

Exit status: 0 if no write covers Torque Enable or Drive Mode, no indirect slot points at either, every CRC holds,
and (with --cycles) every per-cycle check holds; 1 otherwise; 2 on an unreadable capture.
"""
import argparse
import collections
import struct
import sys

TORQUE, DRIVE = 64, 10
GOAL_CURRENT, GOAL_LEN = 237, 2  # the probe's goal window starts with Goal Current (indirect data slot 13)
NAMES = {0x03: "Write", 0x04: "Reg Write", 0x83: "Sync Write", 0x93: "Bulk Write"}
PING, READ, WRITE, SYNC_READ, SYNC_WRITE, FAST_SYNC_READ = 0x01, 0x02, 0x03, 0x82, 0x83, 0x8A
ALL = [2, 3, 4, 5, 6, 7, 8, 9]


class CaptureError(Exception):
    pass


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def out_payloads(path):
    with open(path, "rb") as f:
        g = f.read(24)
        if len(g) < 24:
            raise CaptureError("not a pcap (short global header)")
        magic, = struct.unpack("<I", g[:4])
        if magic != 0xA1B2C3D4:
            raise CaptureError(f"not a little-endian microsecond pcap (magic {magic:#x})")
        link, = struct.unpack("<I", g[20:24])
        if link != 220:
            raise CaptureError(f"link type {link}, not 220")
        while True:
            h = f.read(16)
            if len(h) < 16:
                return
            _, _, incl, _ = struct.unpack("<IIII", h)
            rec = f.read(incl)
            typ, xfer, ep, dev = rec[8], rec[9], rec[10], rec[11]
            data_len, = struct.unpack("<I", rec[36:40])
            if chr(typ) == "S" and xfer == 3 and not ep & 0x80 and data_len:
                yield dev, ep, rec[64:64 + data_len]


def packets(stream):
    i = 0
    while True:
        j = stream.find(b"\xff\xff\xfd\x00", i)
        if j < 0 or j + 7 > len(stream):
            return
        ident = stream[j + 4]
        ln, = struct.unpack("<H", stream[j + 5:j + 7])
        raw = stream[j:j + 7 + ln]
        if len(raw) < 7 + ln:
            return
        body = raw[7:-2]
        crc_ok = crc16(raw[:-2]) == struct.unpack("<H", raw[-2:])[0]
        un = body.replace(b"\xff\xff\xfd\xfd", b"\xff\xff\xfd")
        yield ident, un[0], un[1:], crc_ok
        i = j + 7 + ln


def sync_entries(p):
    """A Sync Write's (address, length per servo, [(id, data)])."""
    a, n = struct.unpack("<HH", p[:4])
    q, out = p[4:], []
    while len(q) >= 1 + n:
        out.append((q[0], q[1:1 + n]))
        q = q[1 + n:]
    return a, n, out


def per_cycle(seq, cycles, read_ids, write_ids):
    """The per-cycle checks on the instructions in the order sent, (instruction, params): (the failures, the reads of
    the read set, the Goal Current writes to every servo, of them all zero)."""
    bad = []
    grouped = [(k, inst, p) for k, (inst, p) in enumerate(seq) if inst in (SYNC_READ, FAST_SYNC_READ, SYNC_WRITE, 0x93)]
    if not grouped or grouped[0][1] != SYNC_READ:
        return ["no initial Sync Read before the cycles"], 0, 0, 0
    k0, _, p0 = grouped[0]
    if list(p0[4:]) != write_ids:
        bad.append(f"the initial Sync Read lists {list(p0[4:])}, not every servo {write_ids}")
    rest = grouped[1:]
    if len(rest) != 2 * cycles:
        bad.append(f"{len(rest)} grouped instructions after the initial read, not 2 x {cycles} cycles")
    reads = writes = zero = 0
    for c in range(min(cycles, len(rest) // 2)):
        (_, ri, rp), (_, wi, wp) = rest[2 * c], rest[2 * c + 1]
        if ri != FAST_SYNC_READ or list(rp[4:]) != read_ids:
            bad.append(f"cycle {c + 1}: its read is {ri:#04x} listing {list(rp[4:])}, not a Fast Sync Read of {read_ids}")
        else:
            reads += 1
        if wi != SYNC_WRITE:
            bad.append(f"cycle {c + 1}: its write is {wi:#04x}, not a Sync Write")
            continue
        a, n, entries = sync_entries(wp)
        if (a, n) != (GOAL_CURRENT, GOAL_LEN) or [i for i, _ in entries] != write_ids:
            bad.append(f"cycle {c + 1}: its Sync Write is address {a} length {n} listing {[i for i, _ in entries]}, "
                       f"not Goal Current to every servo {write_ids}")
            continue
        writes += 1
        if any(any(d) for _, d in entries):
            bad.append(f"cycle {c + 1}: a nonzero goal current")
        else:
            zero += 1
    before = {inst for inst, _ in seq[:k0]}
    if not before <= {PING, READ, WRITE}:
        bad.append(f"before the initial read, instructions other than ping, read and write: {sorted(before)}")
    last = rest[-1][0] if rest else k0
    after = {inst for inst, _ in seq[last + 1:]}
    if not after <= {READ}:
        bad.append(f"after the last cycle, instructions other than single reads: {sorted(after)}")
    return bad, reads, writes, zero


def main(argv=None):
    ap = argparse.ArgumentParser(description="Every Dynamixel write in a usbmon capture of the adapter")
    ap.add_argument("capture")
    ap.add_argument("--cycles", type=int, help="the probe run's log rows: check every cycle's read and write")
    ap.add_argument("--read-ids", default=",".join(map(str, ALL)), help="the read set (default every servo)")
    ap.add_argument("--write-ids", default=",".join(map(str, ALL)), help="every servo the write lists")
    a = ap.parse_args(argv)
    try:
        read_ids = [int(x) for x in a.read_ids.split(",")]
        write_ids = [int(x) for x in a.write_ids.split(",")]
    except ValueError:
        print("write_scan: --read-ids and --write-ids are comma-separated IDs", file=sys.stderr)
        return 2
    if a.cycles is not None and a.cycles < 1:
        print("write_scan: --cycles must be at least 1", file=sys.stderr)
        return 2
    insts = collections.Counter()
    writes = collections.Counter()
    bad_crc = 0
    torque = drive = 0
    indirect = collections.Counter()
    nonzero = collections.Counter()  # (address, length): writes with any nonzero data byte
    devs = collections.Counter()
    seq = []  # (instruction, params), in the order sent
    try:
        for dev, ep, data in out_payloads(a.capture):
            devs[(dev, ep)] += 1
            for ident, inst, p, ok in packets(data):
                insts[inst] += 1
                bad_crc += not ok
                seq.append((inst, p))
                entries = []  # (id, addr, length, data)
                if inst in (0x03, 0x04):
                    addr, = struct.unpack("<H", p[:2])
                    entries.append((ident, addr, len(p) - 2, p[2:]))
                elif inst == 0x83:
                    addr, n, listed = sync_entries(p)
                    entries += [(i, addr, n, d) for i, d in listed]
                elif inst == 0x93:
                    q = p
                    while len(q) >= 5:
                        i2, addr, n = q[0], *struct.unpack("<HH", q[1:5])
                        entries.append((i2, addr, n, q[5:5 + n]))
                        q = q[5 + n:]
                for i2, addr, n, d in entries:
                    writes[(NAMES[inst], addr, n)] += 1
                    nonzero[(NAMES[inst], addr, n)] += any(d)
                    torque += addr <= TORQUE < addr + n
                    drive += addr <= DRIVE < addr + n
                    for base, first in ((168, 1), (578, 29)):
                        for k in range(0, n - 1, 2):
                            at = addr + k
                            if base <= at < base + 56:
                                slot = first + (at - base) // 2
                                indirect[(i2, slot, struct.unpack("<H", d[k:k + 2])[0])] += 1
    except (OSError, CaptureError) as e:
        print(f"write_scan: {e}", file=sys.stderr)
        return 2
    print(f"capture: {a.capture}")
    print(f"bulk OUT submissions by (device, endpoint): {dict(devs)}")
    print("instructions sent: " + ", ".join(f"{k:#04x} {v}" for k, v in sorted(insts.items())) + f"; CRC failures {bad_crc}")
    print("write instructions by (kind, address, length): count")
    for (k, addr, n), c in sorted(writes.items(), key=lambda x: (x[0][1], x[0][0])):
        print(f"  {k:<10} address {addr:>4} length {n:>3}: {c} (with nonzero data: {nonzero[(k, addr, n)]})")
    targets = collections.defaultdict(set)
    for _, slot, t in indirect:
        targets[slot].add(t)
    print("indirect addresses programmed (slot: target addresses, over every servo):")
    for slot in sorted(targets):
        print(f"  {slot:>2}: {sorted(targets[slot])}")
    hit = sorted({t for s in targets.values() for t in s} & {TORQUE, DRIVE})
    print(f"writes covering Torque Enable (64): {torque}; covering Drive Mode (10): {drive}; "
          f"indirect slots pointing at 64 or 10: {hit or 'none'}")
    ok = torque == 0 and drive == 0 and not hit and bad_crc == 0
    if a.cycles is not None:
        bad, reads, wrote, zero = per_cycle(seq, a.cycles, read_ids, write_ids)
        print(f"per cycle ({a.cycles} cycles): Fast Sync Reads of {read_ids}: {reads}; Sync Writes of Goal Current to "
              f"{write_ids}: {wrote}, of them all zero: {zero}; failures {len(bad)}")
        for b in bad[:20]:
            print(f"  {b}")
        if len(bad) > 20:
            print(f"  ... {len(bad) - 20} more")
        ok = ok and not bad
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
