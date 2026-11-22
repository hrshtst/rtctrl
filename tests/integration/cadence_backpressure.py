"""dxl_emu --cadence against a client that stops reading its replies.

Usage: cadence_backpressure.py EMU DIR

Starts EMU (an independent cadence, recorded) linked in DIR, writes Pings
to it without ever reading the replies until the emulator stops (at most
20,000 Pings, 10 s), and checks that the full pty failed the run
explicitly, with a truthful record:
- the emulator stopped by itself, with status 1, naming the held reply not
  written in full and how many of its bytes the pty accepted;
- its record: every reply before that one written in full, that one with
  the bytes accepted (fewer than its own) and EAGAIN, and every later one
  unsent (nothing is written after an incomplete reply);
- its summary: the replies held are those written in full and those not,
  as recorded.
The accepted bytes cannot be read back here (the master's close discards
them); tests/integration/standalone_loop_test.cpp compares them with what
the client reads.
"""

import csv
import errno
import os
import re
import select
import subprocess
import sys
import time
import tty
from pathlib import Path


def ping(dxl_id):
    msg = bytes([0xFF, 0xFF, 0xFD, 0x00, dxl_id, 3, 0, 0x01])
    crc = 0
    for b in msg:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x8005 if crc & 0x8000 else 0)) & 0xFFFF
    return msg + crc.to_bytes(2, "little")


def main():
    emu, d = sys.argv[1], Path(sys.argv[2])
    link, log, out_path = d / "backpressure.tty", d / "backpressure.csv", d / "backpressure.emu.log"
    with open(out_path, "w") as out:
        p = subprocess.Popen([emu, "--link", str(link), "--firmware", "45", "--cadence", "independent",
                              "--cadence-log", str(log)], stdout=out, stderr=subprocess.STDOUT)
    fd = None
    try:
        give_up = time.monotonic() + 5
        while not link.exists():
            assert p.poll() is None, "the emulator exited before serving"
            assert time.monotonic() < give_up, "the emulator did not serve"
            time.sleep(0.01)
        fd = os.open(link, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(fd)
        payload, pos = ping(2) * 20000, 0
        give_up = time.monotonic() + 10
        while pos < len(payload) and p.poll() is None and time.monotonic() < give_up:
            try:
                pos += os.write(fd, payload[pos:pos + 4096])
            except BlockingIOError:
                select.select([], [fd], [], 0.05)
            except OSError as e:
                if e.errno != errno.EIO:
                    raise
                break  # the emulator closed the pty
        try:
            status = p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            raise AssertionError("the emulator did not stop on a full pty") from None
    finally:
        if fd is not None:
            os.close(fd)
        if p.poll() is None:
            p.kill()
            p.wait()

    text = out_path.read_text()
    assert status == 1, ("the emulator's status", status, text)
    m = re.search(r"held reply (\d+) \(counted from 0\) was not written in full: the pty accepted (\d+) of its "
                  r"(\d+) bytes \(([^)]*)\); the cadence cannot be kept, stopping with (\d+) later replies unsent",
                  text)
    assert m, ("no write failure reported", text)
    reply, accepted, size, why, later = int(m[1]), int(m[2]), int(m[3]), m[4], int(m[5])
    s = re.search(r"cadence: (\d+) replies held, (\d+) recorded \((\d+) beyond the record\), \d+ on delayed "
                  r"completions, \d+ missed completions passed over, (\d+) written in full, (\d+) not at exit", text)
    assert s, ("no cadence summary", text)
    held, recorded, beyond, full, not_full = (int(x) for x in s.groups())
    with open(log, newline="") as f:
        recs = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(f)]
    assert len(recs) == recorded == held and beyond == 0, (len(recs), recorded, held, beyond)
    assert held == full + not_full and not_full == later + 1 and reply == full and full > 0, (held, full, not_full)
    for i, r in enumerate(recs):
        if i < reply:
            ok = r["written_ns"] >= r["due_ns"] and r["written_bytes"] == r["bytes"] and r["write_error"] == 0
        elif i == reply:
            ok = (r["written_ns"] == -1 and r["written_bytes"] == accepted < r["bytes"] == size and
                  r["write_error"] == errno.EAGAIN)
        else:
            ok = r["written_ns"] == -1 and r["written_bytes"] == 0 and r["write_error"] == 0
        assert ok, ("record", i, "failed at", reply, r)
    total = sum(r["written_bytes"] for r in recs)
    print(f"backpressure: the pty accepted {full} replies in full, then {accepted} of the next one's {size} bytes "
          f"({why}): {total} bytes; {later} later replies unsent; the emulator stopped with status 1")


if __name__ == "__main__":
    main()
