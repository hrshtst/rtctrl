#!/usr/bin/env python3
"""Every Dynamixel write the host sent in a usbmon capture of the adapter, and, for a bus timing probe run, its
complete instruction sequence (read-only).

Reads a pcap of link type 220 (USB_LINUX_MMAPPED, as tcpdump -i usbmonN writes it), takes the bulk OUT submissions
(the bytes the host sent to the FTDI adapter), splits them into Dynamixel protocol 2.0 instruction packets (byte
stuffing removed, CRC checked), and lists every write instruction (Write, Reg Write, Sync Write, Bulk Write) by
address range. It reports whether any write covers Torque Enable (64) or Drive Mode (10), and decodes the
indirect-address programming (indirect addresses 1-28 at 168-223 and 29-56 at 578-633), so a torque write through
an indirect data slot would also show. This is the full-feedback candidate's write_scan.py
(review-notes/2026-11-24-ff-candidate/, sealed), its report unchanged for a well-formed capture, made a tracked tool.
A write whose parameters do not divide into whole entries (a Sync Write or Bulk Write with bytes left over, a Write
without data) is malformed: it is reported and fails the scan, whatever its prefix decodes to.

With --cycles N (a torque-off x7_bus_timing run of profile current and N log rows; the traffic-reduction diagnostic,
2026-11-24, and its code review), it checks the run's whole instruction sequence, every packet's recipient and every
parameter block's exact length, in the order sent:
1. the startup, before the initial read: only pings (no parameters), single reads (a 2-byte address and a nonzero
   2-byte length) and the indirect-address setup, each unicast to one of the servos (--write-ids). The setup is
   SyncGroup::setupIndirect's (src/dxl/sync_group.cpp): slot k (address 168 + 2 k, k = 0..22) of every servo, once,
   holding the k-th of Present Current (126-127), Present Velocity (128-131), Present Position (132-135), Present
   Input Voltage (144-145), Present Temperature (146), Goal Current (102-103), Goal Velocity (104-107) and Goal
   Position (116-119). Torque Enable and Drive Mode of every servo are read in it;
2. the initial read: one Sync Read broadcast (ID 254) of the 13-byte feedback window (address 224) listing every
   servo, and nothing else in it;
3. then N cycles, each exactly two packets: a Fast Sync Read broadcast of the feedback window (address 224, the
   window's length or one more for a recovery read: --read-len, else 10, 11, 13 or 14) listing exactly the read set
   (--read-ids), then a Sync Write broadcast of Goal Current (address 237, 2 bytes per servo) listing exactly every
   servo, in order, with zero data and no byte left over;
4. after the last cycle, only single reads of Torque Enable and Drive Mode (1 byte each), unicast to the servos, every
   servo's both.
Any other instruction (a Write, Reg Write, Action, Bulk Write, an extra read, a grouped instruction out of place)
anywhere fails, as do a CRC failure and bytes outside any packet.

    python3 -I -B write_scan.py CAPTURE.pcap [--cycles N [--read-ids 2,3,4,5] [--write-ids 2,...,9] [--read-len L]]

Exit status: 0 if no write covers Torque Enable or Drive Mode, no indirect slot points at either, every CRC holds, no
write is malformed, and (with --cycles) the whole sequence is as above; 1 if any of that fails (the traffic does not
match); 2 if the capture cannot be read as complete evidence (not a little-endian microsecond pcap of link type 220,
a truncated record, a USB transfer captured short of its length, a packet cut off at the end of its transfer) or an
option is malformed.
"""
import argparse
import collections
import struct
import sys

TORQUE, DRIVE = 64, 10
FEEDBACK, FEEDBACK_LEN = 224, 13  # SyncGroup::kFeedbackAddr, kFeedbackSlots
GOAL_CURRENT, GOAL_LEN = 237, 2  # the probe's goal window starts with Goal Current (indirect data slot 13)
NAMES = {0x03: "Write", 0x04: "Reg Write", 0x83: "Sync Write", 0x93: "Bulk Write"}
PING, READ, WRITE, SYNC_READ, SYNC_WRITE, FAST_SYNC_READ = 0x01, 0x02, 0x03, 0x82, 0x83, 0x8A
BROADCAST = 0xFE
ALL = [2, 3, 4, 5, 6, 7, 8, 9]
READ_LENS = (10, 11, 13, 14)  # the motion-only and full windows, and each one's recovery read (one byte more)
# SyncGroup::setupIndirect's sources, one per indirect slot k at address 168 + 2 k.
INDIRECT_SOURCES = ([*range(126, 128), *range(128, 132), *range(132, 136), *range(144, 146), 146, *range(102, 104),
                     *range(104, 108), *range(116, 120)])


class CaptureError(Exception):
    """The capture cannot be read as complete evidence."""


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
        k = 0
        while True:
            h = f.read(16)
            if not h:
                return
            if len(h) < 16:
                raise CaptureError(f"record {k}: a truncated record header")
            _, _, incl, _ = struct.unpack("<IIII", h)
            rec = f.read(incl)
            if len(rec) < incl or incl < 64:
                raise CaptureError(f"record {k}: truncated ({len(rec)} of {incl} bytes)")
            typ, xfer, ep, dev = rec[8], rec[9], rec[10], rec[11]
            length, data_len = struct.unpack("<II", rec[32:40])
            if chr(typ) == "S" and xfer == 3 and not ep & 0x80 and length:
                if data_len < length or len(rec) < 64 + data_len:
                    raise CaptureError(f"record {k}: a bulk OUT transfer captured short ({data_len} of {length} bytes)")
                yield dev, ep, rec[64:64 + data_len]
            k += 1


def packets(stream):
    """(target, instruction, parameters, CRC ok) per packet, then the bytes outside any packet; a packet cut off at the
    end of the transfer raises CaptureError."""
    i, outside = 0, 0
    out = []
    while True:
        j = stream.find(b"\xff\xff\xfd\x00", i)
        if j < 0:
            outside += len(stream) - i
            return out, outside
        outside += j - i
        if j + 7 > len(stream):
            raise CaptureError("a packet cut off at the end of its transfer (header)")
        ident = stream[j + 4]
        ln, = struct.unpack("<H", stream[j + 5:j + 7])
        raw = stream[j:j + 7 + ln]
        if len(raw) < 7 + ln or ln < 3:
            raise CaptureError("a packet cut off at the end of its transfer")
        body = raw[7:-2]
        crc_ok = crc16(raw[:-2]) == struct.unpack("<H", raw[-2:])[0]
        un = body.replace(b"\xff\xff\xfd\xfd", b"\xff\xff\xfd")
        out.append((ident, un[0], un[1:], crc_ok))
        i = j + 7 + ln


def sync_entries(p):
    """A Sync Write's (address, length per servo, [(id, data)], whole): whole is False when its parameters are shorter
    than the 4-byte header or leave bytes after the last whole entry."""
    if len(p) < 4:
        return 0, 0, [], False
    a, n = struct.unpack("<HH", p[:4])
    q, out = p[4:], []
    while len(q) >= 1 + n:
        out.append((q[0], q[1:1 + n]))
        q = q[1 + n:]
    return a, n, out, not q and n > 0


def bulk_entries(p):
    """A Bulk Write's [(id, address, length, data)] and whether its parameters divide into whole entries."""
    q, out = p, []
    while len(q) >= 5:
        i2, a, n = q[0], *struct.unpack("<HH", q[1:5])
        if len(q) < 5 + n:
            return out, False
        out.append((i2, a, n, q[5:5 + n]))
        q = q[5 + n:]
    return out, not q


def expected_setup(write_ids):
    """The indirect setup's writes: {(id, address): data}."""
    return {(i, 168 + 2 * k): struct.pack("<H", src) for i in write_ids for k, src in enumerate(INDIRECT_SOURCES)}


def strict(seq, cycles, read_ids, write_ids, read_lens):
    """The probe run's whole sequence, (target, instruction, parameters) in the order sent: (the failures, the cycles'
    reads of the read set, their Goal Current writes to every servo, of them all zero)."""
    bad = []
    servos = set(write_ids)

    def unicast_read(t, p):
        return t in servos and len(p) == 4 and struct.unpack("<H", p[2:4])[0] > 0

    # 1. The startup.
    setup, setup_seen = expected_setup(write_ids), collections.Counter()
    pre_reads = set()
    k = 0
    while k < len(seq) and seq[k][1] != SYNC_READ:
        t, inst, p = seq[k]
        if inst == PING and t in servos and not p:
            pass
        elif inst == READ and unicast_read(t, p):
            pre_reads.add((t, struct.unpack("<H", p[:2])[0]))
        elif inst == WRITE and t in servos and len(p) == 4 and setup.get((t, struct.unpack("<H", p[:2])[0])) == p[2:]:
            setup_seen[(t, struct.unpack("<H", p[:2])[0])] += 1
        else:
            bad.append(f"startup packet {k}: instruction {inst:#04x} to {t} with {len(p)} parameter bytes is not a "
                       "ping, a single read or the indirect setup, unicast to a servo")
        k += 1
    missing = sorted(set(setup) - set(setup_seen))
    twice = sorted(x for x, c in setup_seen.items() if c > 1)
    if missing:
        bad.append(f"the indirect setup is incomplete: {len(missing)} slots not written (the first: servo {missing[0][0]} "
                   f"address {missing[0][1]})")
    if twice:
        bad.append(f"the indirect setup wrote {len(twice)} slots more than once (the first: servo {twice[0][0]} address "
                   f"{twice[0][1]})")
    for reg in (TORQUE, DRIVE):
        unread = sorted(i for i in write_ids if (i, reg) not in pre_reads)
        if unread:
            bad.append(f"before the run, register {reg} of servos {unread} not read")
    # 2. The initial read.
    if k >= len(seq):
        return [*bad, "no initial Sync Read"], 0, 0, 0
    t, _, p = seq[k]
    if t != BROADCAST or p != struct.pack("<HH", FEEDBACK, FEEDBACK_LEN) + bytes(write_ids):
        bad.append(f"the initial Sync Read (to {t}, {len(p)} parameter bytes) is not a broadcast of the {FEEDBACK_LEN}-byte "
                   f"window at {FEEDBACK} listing every servo {write_ids}")
    k += 1
    # 3. The cycles.
    reads = writes = zero = 0
    want_write = struct.pack("<HH", GOAL_CURRENT, GOAL_LEN)
    for c in range(1, cycles + 1):
        if k + 1 >= len(seq):
            bad.append(f"the capture ends before cycle {c} of {cycles}")
            break
        (rt, ri, rp), (wt, wi, wp) = seq[k], seq[k + 1]
        if (ri == FAST_SYNC_READ and rt == BROADCAST and len(rp) == 4 + len(read_ids) and rp[4:] == bytes(read_ids)
                and struct.unpack("<H", rp[:2])[0] == FEEDBACK and struct.unpack("<H", rp[2:4])[0] in read_lens):
            reads += 1
        else:
            bad.append(f"cycle {c}: its read is {ri:#04x} to {rt} with {len(rp)} parameter bytes listing {list(rp[4:])}, "
                       f"not a Fast Sync Read broadcast of the feedback window listing {read_ids}")
        a, n, entries, whole = sync_entries(wp) if wi == SYNC_WRITE else (0, 0, [], False)
        if (wi == SYNC_WRITE and wt == BROADCAST and whole and wp[:4] == want_write
                and [i for i, _ in entries] == write_ids):
            writes += 1
            if any(any(d) for _, d in entries):
                bad.append(f"cycle {c}: a nonzero goal current")
            else:
                zero += 1
        else:
            bad.append(f"cycle {c}: its write is {wi:#04x} to {wt} with {len(wp)} parameter bytes (address {a}, length "
                       f"{n}, listing {[i for i, _ in entries]}, whole {whole}), not a Goal Current Sync Write broadcast "
                       f"to every servo {write_ids}")
        k += 2
    # 4. After the last cycle.
    post = set()
    for j in range(k, len(seq)):
        t, inst, p = seq[j]
        if inst == READ and unicast_read(t, p) and p == struct.pack("<HH", struct.unpack("<H", p[:2])[0], 1) and \
                struct.unpack("<H", p[:2])[0] in (TORQUE, DRIVE):
            post.add((t, struct.unpack("<H", p[:2])[0]))
        else:
            bad.append(f"after the last cycle, packet {j}: instruction {inst:#04x} to {t} with {len(p)} parameter bytes "
                       "is not a read of Torque Enable or Drive Mode unicast to a servo")
    for reg in (TORQUE, DRIVE):
        unread = sorted(i for i in write_ids if (i, reg) not in post)
        if unread:
            bad.append(f"after the run, register {reg} of servos {unread} not read")
    return bad, reads, writes, zero


def ids_arg(text):
    try:
        out = [int(x) for x in text.split(",")]
    except ValueError:
        raise argparse.ArgumentTypeError(f"comma-separated IDs, not {text!r}") from None
    if not out or len(set(out)) != len(out) or not all(0 <= i <= 252 for i in out):
        raise argparse.ArgumentTypeError(f"distinct IDs 0 to 252, not {text!r}")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Every Dynamixel write in a usbmon capture of the adapter")
    ap.add_argument("capture")
    ap.add_argument("--cycles", type=int, help="the probe run's log rows: check its whole instruction sequence")
    ap.add_argument("--read-ids", type=ids_arg, default=ALL, help="the read set (default every servo)")
    ap.add_argument("--write-ids", type=ids_arg, default=ALL, help="every servo the probe writes and checks")
    ap.add_argument("--read-len", type=int, choices=(10, 13),
                    help="the run's window: each cycle's read is it, or one byte more for a recovery read")
    try:
        a = ap.parse_args(argv)
    except SystemExit as e:
        return 2 if e.code else 0
    if a.cycles is not None and a.cycles < 1:
        print("write_scan: --cycles must be at least 1", file=sys.stderr)
        return 2
    read_lens = (a.read_len, a.read_len + 1) if a.read_len else READ_LENS
    insts = collections.Counter()
    writes = collections.Counter()
    bad_crc = 0
    torque = drive = 0
    indirect = collections.Counter()
    nonzero = collections.Counter()  # (address, length): writes with any nonzero data byte
    devs = collections.Counter()
    seq = []  # (target, instruction, params), in the order sent
    malformed = []
    outside = 0
    try:
        for dev, ep, data in out_payloads(a.capture):
            devs[(dev, ep)] += 1
            found, junk = packets(data)
            outside += junk
            for ident, inst, p, ok in found:
                insts[inst] += 1
                bad_crc += not ok
                seq.append((ident, inst, p))
                entries = []  # (id, addr, length, data)
                if inst in (0x03, 0x04):
                    if len(p) < 3:
                        malformed.append(f"{NAMES[inst]} to {ident} with {len(p)} parameter bytes")
                        continue
                    addr, = struct.unpack("<H", p[:2])
                    entries.append((ident, addr, len(p) - 2, p[2:]))
                elif inst == 0x83:
                    addr, n, listed, whole = sync_entries(p)
                    if not whole:
                        malformed.append(f"Sync Write to {ident}: {len(p)} parameter bytes do not divide into whole "
                                         f"entries of {n}")
                    entries += [(i, addr, n, d) for i, d in listed]
                elif inst == 0x93:
                    listed, whole = bulk_entries(p)
                    if not whole:
                        malformed.append(f"Bulk Write to {ident}: {len(p)} parameter bytes do not divide into whole "
                                         "entries")
                    entries += listed
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
        print(f"write_scan: {a.capture}: cannot be read as complete evidence: {e}", file=sys.stderr)
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
    if malformed:
        print(f"malformed write instructions: {len(malformed)} (the first: {malformed[0]})")
    ok = torque == 0 and drive == 0 and not hit and bad_crc == 0 and not malformed
    if a.cycles is not None:
        bad, reads, wrote, zero = strict(seq, a.cycles, a.read_ids, a.write_ids, read_lens)
        if outside:
            bad.append(f"{outside} bytes sent outside any packet")
        if bad_crc:
            bad.append(f"{bad_crc} CRC failures")
        print(f"the whole sequence ({a.cycles} cycles): Fast Sync Reads of {a.read_ids}: {reads}; Sync Writes of Goal "
              f"Current to {a.write_ids}: {wrote}, of them all zero: {zero}; failures {len(bad)}")
        for b in bad[:20]:
            print(f"  {b}")
        if len(bad) > 20:
            print(f"  ... {len(bad) - 20} more")
        ok = ok and not bad
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
