"""dxl_emu as a process, over its pty: the standalone loop's behaviour that only the real process shows.

    python3 tests/integration/dxl_emu_process_check.py --emulator <dxl_emu> --dir <scratch dir>
        [--measure-only]

Checks (each on a fresh emulator; prints "dxl_emu process checks OK" when all pass):
  - no client yet, and after a client detaches: the emulator does not spin (its CPU time over 1 s stays small);
  - a frame split across two writes is answered once; after a reattach no frame is lost;
  - model time on wall time: a servo in velocity mode at Goal Velocity 64 moves one pulse per 1 ms tick, so its
    Present Position read 2 s apart must advance 1000 pulses/s within 1 percent;
  - SIGINT and SIGTERM, each while waiting (a client attached and idle), detached and busy (a client sending without
    pause): exit status 0 within a loose bound, the link removed, and the exit line's model time behind wall time by
    less than the catch-up cap;
  - the catch-up cap: stopped (SIGSTOP) for 200 ms, the emulator exits with status 3 and says why.
The reply latency to pings sent at random phases is reported as a distribution, not asserted (host-dependent).
--measure-only runs only the latency and model-rate measurements and asserts nothing (for comparing emulators).
Shutdown bounds asserted here include host dispatch; the code's own bound is one loop step and one wait (a tick).
"""
import argparse
import os
import random
import re
import select
import signal
import statistics
import subprocess
import sys
import termios
import threading
import time
import tty
from pathlib import Path

PING, READ, WRITE = 0x01, 0x02, 0x03
EXIT_LINE = re.compile(r"^dxl_emu: model time ([0-9.]+) s \((\d+) ticks of ([0-9.]+) ms\) over ([0-9.]+) s of wall time;"
                       r" steps (\d+), of them late \(more than one tick\) (\d+), most ticks at once (\d+); reads at the"
                       r" (\d+)-byte bound (\d+); waits by time alone \(no client\) (\d+)$", re.MULTILINE)
CAP_S = 0.050
FAILS = []


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def stuff(body: bytes) -> bytes:
    out, run = bytearray(), 0
    for b in body:
        out.append(b)
        if run >= 2 and b == 0xFD:
            out.append(0xFD)
            run = 0
            continue
        run = run + 1 if b == 0xFF else 0
    return bytes(out)


def frame(dev_id: int, inst: int, params: bytes = b"") -> bytes:
    body = stuff(bytes([inst]) + params)
    f = bytes([0xFF, 0xFF, 0xFD, 0x00, dev_id]) + (len(body) + 2).to_bytes(2, "little") + body
    return f + crc16(f).to_bytes(2, "little")


def write_reg(dev_id, addr, value, n):
    return frame(dev_id, WRITE, addr.to_bytes(2, "little") + value.to_bytes(n, "little"))


def read_reg(dev_id, addr, n):
    return frame(dev_id, READ, addr.to_bytes(2, "little") + n.to_bytes(2, "little"))


def statuses(data: bytes):
    out, i = [], 0
    while i + 7 <= len(data):
        if data[i:i + 3] != b"\xff\xff\xfd":
            i += 1
            continue
        n = int.from_bytes(data[i + 5:i + 7], "little")
        if i + 7 + n > len(data):
            break
        body = data[i + 7:i + 7 + n - 2]
        out.append((data[i + 4], body[1], body[2:]))
        i += 7 + n
    return out


def check(ok: bool, what: str):
    print(("  ok    " if ok else "  FAIL  ") + what)
    if not ok:
        FAILS.append(what)


class Emu:
    def __init__(self, exe: str, d: Path, name: str):
        self.link = d / f"{name}.tty"
        self.log = d / f"{name}.out"
        self.err = d / f"{name}.err"
        with open(self.log, "w") as out, open(self.err, "w") as err:
            self.proc = subprocess.Popen([exe, "--link", str(self.link)], stdout=out, stderr=err,
                                         stdin=subprocess.DEVNULL)
        give_up = time.monotonic() + 10
        while not self.link.exists():
            if time.monotonic() > give_up or self.proc.poll() is not None:
                raise SystemExit(f"{name}: the emulator did not link {self.link}")
            time.sleep(0.01)

    def cpu_s(self) -> float:
        f = Path(f"/proc/{self.proc.pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(f[11]) + int(f[12])) / os.sysconf("SC_CLK_TCK")

    def stop(self, sig=signal.SIGINT, bound=2.0):
        t = time.monotonic()
        self.proc.send_signal(sig)
        try:
            status = self.proc.wait(timeout=bound)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
            return None, time.monotonic() - t
        return status, time.monotonic() - t

    def exit_line(self):
        m = EXIT_LINE.search(self.log.read_text())
        return m


class Client:
    def __init__(self, path: Path):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self):
        os.close(self.fd)

    def send(self, data: bytes):
        while data:
            try:
                n = os.write(self.fd, data)
                data = data[n:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.01)

    def receive(self, want: int, timeout: float) -> bytes:
        out, give_up = bytearray(), time.monotonic() + timeout
        while len(out) < want and time.monotonic() < give_up:
            select.select([self.fd], [], [], 0.005)
            try:
                out += os.read(self.fd, 4096)
            except BlockingIOError:
                pass
        return bytes(out)

    def transact(self, f: bytes, want: int, timeout=1.0):
        """(reply bytes, send time, first reply byte time)."""
        t0 = time.monotonic()
        self.send(f)
        t_first, out, give_up = None, bytearray(), t0 + timeout
        while len(out) < want and time.monotonic() < give_up:
            select.select([self.fd], [], [], 0.002)
            try:
                chunk = os.read(self.fd, 4096)
            except BlockingIOError:
                continue
            if chunk and t_first is None:
                t_first = time.monotonic()
            out += chunk
        return bytes(out), t0, t_first


def latency_and_rate(exe, d, label):
    """Ping latency at random phases, and the model rate over 2 s; returns (latencies us, pulses/s)."""
    e = Emu(exe, d, f"measure-{label}")
    c = Client(e.link)
    lat = []
    rnd = random.Random(1)
    for _ in range(300):
        time.sleep(rnd.uniform(0, 0.003))
        data, t0, tf = c.transact(frame(2, PING), 14)
        if tf is not None and len(statuses(data)) == 1:
            lat.append((tf - t0) * 1e6)
    for f in (write_reg(2, 11, 1, 1), write_reg(2, 104, 64, 4), write_reg(2, 64, 1, 1)):
        c.transact(f, 11)

    def position():
        data, t0, _ = c.transact(read_reg(2, 132, 4), 15)
        s = statuses(data)
        return int.from_bytes(s[0][2], "little", signed=True), (t0 + time.monotonic()) / 2

    p0, t0 = position()
    time.sleep(2.0)
    p1, t1 = position()
    c.close()
    e.stop()
    rate = (p1 - p0) / (t1 - t0)
    q = statistics.quantiles(lat, n=100) if len(lat) > 10 else [float("nan")] * 99
    print(f"  {label}: ping reply latency at random phases ({len(lat)} of 300 answered), us: p50 {q[49]:.0f} p90 "
          f"{q[89]:.0f} p99 {q[98]:.0f} max {max(lat):.0f}; model rate {rate:.1f} pulses/s (1000 = wall time)")
    return lat, rate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--emulator", required=True)
    ap.add_argument("--dir", required=True)
    ap.add_argument("--measure-only", action="store_true")
    a = ap.parse_args()
    d = Path(a.dir)
    d.mkdir(parents=True, exist_ok=True)
    exe = a.emulator

    print("measurements:")
    _, rate = latency_and_rate(exe, d, "this emulator")
    if a.measure_only:
        return 0
    check(abs(rate - 1000.0) <= 10.0, f"model time on wall time: {rate:.1f} pulses/s, 1000 within 1 percent")

    print("no spin, partial frames, reattach:")
    e = Emu(exe, d, "detach")
    c0 = e.cpu_s()
    time.sleep(1.0)
    used = e.cpu_s() - c0
    check(used < 0.25, f"no client yet: {used:.2f} s of CPU over 1 s")
    c = Client(e.link)
    ping = frame(2, PING)
    c.send(ping[:6])
    time.sleep(0.02)
    check(c.receive(1, 0.05) == b"", "half a frame: no reply yet")
    c.send(ping[6:])
    check(len(statuses(c.receive(14, 1.0))) == 1, "the rest of the frame: one reply")
    c.close()
    c0 = e.cpu_s()
    time.sleep(1.0)
    used = e.cpu_s() - c0
    check(used < 0.25, f"after the client detached: {used:.2f} s of CPU over 1 s")
    c = Client(e.link)
    got = 0
    for _ in range(20):
        data, _, _ = c.transact(ping, 14)
        got += len(statuses(data))
    check(got == 20, f"after a reattach: {got} of 20 pings answered")
    c.close()
    status, took = e.stop(signal.SIGTERM)
    m = e.exit_line()
    check(status == 0 and m is not None and int(m[10]) > 0,
          f"its exit: status {status}, waits by time alone {m[10] if m else '?'} (more than 0)")

    print("signals while waiting, detached and busy:")
    for sig in (signal.SIGINT, signal.SIGTERM):
        for state in ("waiting", "detached", "busy"):
            e = Emu(exe, d, f"{sig.name}-{state}")
            c = Client(e.link)
            flood = threading.Event()
            writer = None
            if state == "detached":
                c.transact(ping, 14)
                c.close()
            elif state == "busy":
                burst = ping * 400

                def pour(fd=c.fd, burst=burst, flood=flood):
                    while not flood.is_set():
                        try:
                            os.write(fd, burst)
                        except BlockingIOError:
                            time.sleep(0.0005)
                        except OSError:  # the emulator gone (EIO)
                            return
                writer = threading.Thread(target=pour)
                writer.start()
            time.sleep(0.5)
            status, took = e.stop(sig)
            flood.set()
            if writer:
                writer.join()
            if state != "detached":
                c.close()
            m = e.exit_line()
            lag = (float(m[4]) - float(m[1])) if m else None
            check(status == 0 and took < 2.0 and not e.link.exists() and m is not None and lag is not None
                  and -0.001 <= lag < CAP_S,
                  f"{sig.name} {state}: status {status} after {took * 1e3:.1f} ms, link removed {not e.link.exists()}, "
                  f"model behind wall time by {lag * 1e3 if lag is not None else float('nan'):.1f} ms"
                  + (f", reads at the bound {m[9]}" if m else ""))

    print("the catch-up cap:")
    e = Emu(exe, d, "overrun")
    time.sleep(0.2)
    e.proc.send_signal(signal.SIGSTOP)
    time.sleep(0.2)
    e.proc.send_signal(signal.SIGCONT)
    try:
        status = e.proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        e.proc.kill()
        status = None
    err = e.err.read_text()
    check(status == 3 and "more than the cap of 50" in err and not e.link.exists(),
          f"stopped 200 ms: status {status}, link removed {not e.link.exists()}, says: {err.strip()[:120]}")

    if FAILS:
        print(f"dxl_emu process checks FAILED: {len(FAILS)}")
        return 1
    print("dxl_emu process checks OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
