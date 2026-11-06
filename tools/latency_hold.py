#!/usr/bin/env python3
"""Hold diagnostic of the adapter's 0 ms latency timer (torque-off; root).

usage: sudo tools/latency_hold.py [--port P] [--id N] [--window-ms W]
                                  [--outdir DIR] [--date YYYYMMDD]
                                  [--attempt N] [--manifest FILE] [--yes]

At a 0 ms latency timer the adapter returned only its two status bytes
after a ping (2026-11-06). This asks whether it received the reply and
held it. On one serial descriptor, open throughout:
  1. the adapter's latency timer must read 1 ms (sysfs, the adapter's own
     value, the port's low-latency flag on the held descriptor), and a ping
     at 1 ms must get a matching reply: the baseline;
  2. the input is drained and flushed, 0 ms is set and read back, exactly
     one ping is sent and the input is observed for W ms;
  3. with nothing sent and no flush, reset, close or reconfiguration of the
     port, 1 ms is set and read back and the input is observed for W ms;
  4. a fresh control ping at 1 ms is sent and observed for W ms.
The latency timer is set through sysfs and read back through sysfs, the
adapter's own Get Latency Timer request (usbfs) and the held descriptor,
so the sequence adds no open or reset. Only ping instructions go to the
servos. usbmon records the adapter's USB traffic from before the first
ping to after the last window, and every write and read is logged with
its CLOCK_MONOTONIC time and raw bytes.

Each window's payload is classified: a matching reply from the servo
(complete, CRC good), partial, corrupt, or none. A matching reply in step
3, before anything further is sent, supports delayed delivery by the
adapter (it does not timestamp the adapter's reception); no payload
leaves reception, transmission and dropping unresolved.

Once the starting 1 ms is verified, before the baseline ping, the cleanup
is owed: on every handled exit (the end, an abort, Ctrl-C, SIGTERM,
SIGHUP) the 1 ms is verified, and set again if another value is in effect.
A ping that is not written whole (a short write or a write error) aborts
the sequence, as does any other transport error: no ping is retried, so
the experimental ping stays the only one. The capture is evidence only if
tcpdump stopped with status 0, the capture holds packets and tcpdump
reports no drop; otherwise the result is marked invalid. Exit status: 0
completed with valid evidence (whatever the outcome); 1 aborted (the
baseline had no matching reply, a ping was not written whole, a transport
error, a setting did not read back) or the evidence is invalid; 2 refused
before anything was sent; 3 the cleanup did not leave 1 ms; 130
interrupted. A failed cleanup or invalid evidence is reported whatever the
status, and a stop, a cleanup failure or an interruption keeps its status.

Outputs in --outdir (gitignored in the repo root): <base>.csv (every write
and read), <base>.csv.run.txt (the record) and <base>.csv.usbmon.pcap,
with <base> = rate-hold-<date>-<attempt>. A <base>.csv that already
appears in the archive manifest (docs/records/data-archive.md, or
--manifest), bare or directory-qualified, itself or as a sidecar's prefix,
is refused, since the output directory forgets the archived runs: pass a
free --attempt.
"""

from __future__ import annotations

import argparse
import datetime
import fcntl
import os
import pathlib
import re
import select
import signal
import subprocess
import sys
import termios
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import latency_timer as lt  # noqa: E402

TIOCEXCL = 0x540C
STATUS_INSTRUCTION = 0x55
BAUDS = {3000000: termios.B3000000, 1000000: termios.B1000000, 57600: termios.B57600}
WORKING_MS = 1


class Refused(Exception):
    """Refused before anything was sent to the servos."""


class Aborted(Exception):
    """The sequence cannot go on (the cleanup still runs)."""


class Interrupted(BaseException):
    """A handled signal: Ctrl-C, SIGTERM or SIGHUP."""


def crc16(data: bytes) -> int:
    """Dynamixel Protocol 2.0 CRC-16 (polynomial 0x8005)."""
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ping_packet(servo_id: int) -> bytes:
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, servo_id, 0x03, 0x00, 0x01])
    return body + crc16(body).to_bytes(2, "little")


HEADER = bytes([0xFF, 0xFF, 0xFD, 0x00])


def classify(data: bytes, servo_id: int) -> tuple[str, str]:
    """A window's payload: ("reply", ...) for a complete status packet from
    servo_id with a good CRC, else "partial", "corrupt" or "none"."""
    if not data:
        return "none", "no payload"
    at = data.find(HEADER)
    if at < 0:
        tail = next((n for n in range(min(3, len(data)), 0, -1) if data.endswith(HEADER[:n])), 0)
        if tail == len(data):
            return "partial", f"{len(data)} bytes: the start of a header"
        return "corrupt", f"{len(data)} bytes without a packet header"
    pkt = data[at:]
    if len(pkt) < 7:
        return "partial", f"{len(data)} bytes: a header without its length"
    length = pkt[5] | pkt[6] << 8
    total = 7 + length
    if len(pkt) < total:
        return "partial", f"{len(data)} bytes: {len(pkt)} of a {total}-byte packet"
    pkt = pkt[:total]
    if crc16(pkt[:-2]) != int.from_bytes(pkt[-2:], "little"):
        return "corrupt", f"{len(data)} bytes: a {total}-byte packet with a bad CRC"
    if pkt[4] != servo_id or pkt[7] != STATUS_INSTRUCTION or length != 7:
        return "corrupt", (f"{len(data)} bytes: a valid packet, but id {pkt[4]} instruction "
                           f"{pkt[7]:#04x} length {length}, not a ping reply from id {servo_id}")
    model = pkt[9] | pkt[10] << 8
    extra = len(data) - total  # bytes before or after the packet
    return "reply", (f"a ping reply from id {servo_id} (error {pkt[8]:#04x}, model {model}, "
                     f"firmware {pkt[11]}){f', with {extra} other bytes' if extra else ''}")


class Serial:
    """The held descriptor: opened and configured once."""

    def __init__(self, path: str, baud: int) -> None:
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(self.fd, TIOCEXCL)  # no other program may open it now
            a = termios.tcgetattr(self.fd)
            a[0] = termios.IGNPAR
            a[1] = 0
            a[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            a[3] = 0
            a[4] = a[5] = BAUDS[baud]
            a[6][termios.VMIN] = 0
            a[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
        except BaseException:
            os.close(self.fd)
            raise

    def write(self, data: bytes) -> int:
        return os.write(self.fd, data)

    def read(self, timeout_ns: int) -> bytes:
        """What arrives within timeout_ns (possibly nothing)."""
        r, _, _ = select.select([self.fd], [], [], max(0, timeout_ns) / 1e9)
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def flush_input(self) -> None:
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def low_latency(self) -> bool | None:
        return lt.low_latency_of(self.fd)

    def close(self) -> None:
        os.close(self.fd)


class Latency:
    """The adapter's latency timer, set through sysfs and read without
    opening the port: sysfs, the adapter's own value (usbfs) and the held
    descriptor's flag."""

    def __init__(self, adapter: lt.Adapter) -> None:
        self.adapter = adapter

    def set(self, ms: int) -> None:
        self.adapter.write_sysfs(ms)

    def read(self, serial: Serial | None) -> tuple:
        return (self.adapter.read_sysfs(), self.adapter.read_device(),
                serial.low_latency() if serial is not None else None)


class Clock:
    @staticmethod
    def now() -> int:
        return time.monotonic_ns()


def state_text(st: tuple) -> str:
    sysfs, device, low = st

    def show(v):
        return "n/a" if v is None else str(int(v))
    return f"sysfs {show(sysfs)} device {show(device)} low_latency {show(low)}"


def holds(st: tuple, ms: int) -> bool:
    return st[0] == ms and st[1] == ms and st[2] is False


class Hold:
    """The sequence and its cleanup, on injected port, timer and clock."""

    def __init__(self, serial, latency, clock, servo_id: int, window_ns: int) -> None:
        self.serial, self.latency, self.clock = serial, latency, clock
        self.servo_id, self.window_ns = servo_id, window_ns
        self.origin = clock.now()
        self.events: list[tuple] = []  # (t_ns from origin, phase, event, detail, bytes)
        self.notes: list[str] = []
        self.result: dict[str, tuple] = {}
        self.owed = False
        self.cleanup_ok = True

    def log(self, phase: str, event: str, detail: str = "", data: bytes = b"") -> None:
        self.events.append((self.clock.now() - self.origin, phase, event, detail, data))

    def note(self, line: str) -> None:
        self.notes.append(line)

    def check(self, phase: str, ms: int) -> bool:
        st = self.latency.read(self.serial)
        self.log(phase, "latency", f"requested {ms} ms: {state_text(st)}")
        self.note(f"{phase}: latency timer (requested {ms} ms): {state_text(st)}")
        return holds(st, ms)

    def set(self, phase: str, ms: int) -> bool:
        self.owed = True
        try:
            self.latency.set(ms)
        except OSError as e:
            self.log(phase, "latency", f"setting {ms} ms failed: {e}")
            self.note(f"{phase}: setting the latency timer to {ms} ms failed: {e}")
            return False
        return self.check(phase, ms)

    def ping(self, phase: str) -> None:
        """One ping, written whole or the sequence aborts (never retried:
        the experimental ping must stay the only one)."""
        pkt = ping_packet(self.servo_id)
        try:
            n = self.serial.write(pkt)
        except OSError as e:
            self.log(phase, "write", f"ping id {self.servo_id} failed: {e}")
            raise Aborted(f"{phase}: the ping was not written ({e})") from e
        self.log(phase, "write", f"ping id {self.servo_id}, {n} of {len(pkt)} bytes", pkt[:max(0, n)])
        if n != len(pkt):
            raise Aborted(f"{phase}: a short write ({n} of {len(pkt)} bytes); no ping is retried")

    def observe(self, phase: str) -> bytes:
        deadline = self.clock.now() + self.window_ns
        got = bytearray()
        while True:
            remaining = deadline - self.clock.now()
            if remaining <= 0:
                break
            chunk = self.serial.read(remaining)
            if chunk:
                self.log(phase, "read", f"{len(chunk)} bytes", chunk)
                got += chunk
        self.result[phase] = classify(bytes(got), self.servo_id)
        self.log(phase, "window", f"{self.result[phase][0]}: {self.result[phase][1]}")
        self.note(f"{phase}: {self.result[phase][0]} ({self.result[phase][1]})")
        return bytes(got)

    def sequence(self) -> None:
        st = self.latency.read(self.serial)
        self.log("start", "latency", state_text(st))
        self.note(f"start: latency timer {state_text(st)}")
        if not holds(st, WORKING_MS):
            raise Refused(f"the adapter's latency timer is not the working {WORKING_MS} ms "
                          f"with the flag clear ({state_text(st)}): nothing was sent")
        # From here the cleanup verifies the working value, and sets it again
        # if anything (the baseline included) changed it.
        self.owed = True
        # 1. The baseline at 1 ms.
        self.serial.flush_input()
        self.log("baseline", "flush")
        self.ping("baseline")
        self.observe("baseline")
        if self.result["baseline"][0] != "reply":
            raise Aborted("the 1 ms baseline ping had no matching reply")
        # 2. Drain, flush, 0 ms, one ping.
        self.observe("drain")
        self.serial.flush_input()
        self.log("drain", "flush")
        if not self.set("zero", 0):
            raise Aborted("the adapter does not hold 0 ms")
        self.ping("zero")
        self.observe("zero")
        # 3. 1 ms again, nothing sent, no flush: observe.
        switched = self.set("after_switch", WORKING_MS)
        self.observe("after_switch")
        if not switched:
            raise Aborted("the adapter does not hold 1 ms again")
        # 4. A fresh control ping at 1 ms.
        self.ping("control")
        self.observe("control")

    def cleanup(self) -> bool:
        """Leaves the working 1 ms in effect: verified, or set again."""
        if not self.owed:
            return True
        try:
            st = self.latency.read(self.serial)
        except OSError:
            st = (None, None, None)
        if holds(st, WORKING_MS):
            self.note(f"cleanup: latency timer verified at {WORKING_MS} ms: {state_text(st)}")
            self.owed = False
            return True
        try:
            self.latency.set(WORKING_MS)
            st = self.latency.read(self.serial)
        except OSError as e:
            self.note(f"cleanup: latency timer NOT restored to {WORKING_MS} ms: {e}")
            return False
        if holds(st, WORKING_MS):
            self.note(f"cleanup: latency timer restored to {WORKING_MS} ms: {state_text(st)}")
            self.owed = False
            return True
        self.note(f"cleanup: latency timer NOT restored to {WORKING_MS} ms: {state_text(st)}")
        return False

    def run(self) -> int:
        """The sequence, then the cleanup; the exit status."""
        status = 0
        try:
            self.sequence()
        except Refused as e:
            self.note(f"refused: {e}")
            status = 2
        except Aborted as e:
            self.note(f"aborted: {e}")
            status = 1
        except (OSError, termios.error) as e:
            self.note(f"aborted: a transport error: {e}")
            status = 1
        except Interrupted:
            self.note("interrupted")
            status = 130
        finally:
            self.cleanup_ok = self.cleanup()
        if not self.cleanup_ok:
            self.note("the run fails: its cleanup did not leave the working latency timer")
            if status == 0:
                status = 3
        return status

    def outcome(self) -> str:
        r = self.result
        if "zero" not in r:
            return "no 0 ms observation"
        zero, after = r["zero"][0], r.get("after_switch", ("-",))[0]
        if zero == "reply":
            verdict = "the reply arrived at 0 ms"
        elif after == "reply":
            verdict = "a matching reply arrived only after the switch to 1 ms (delayed delivery)"
        elif zero == "none" and after == "none":
            verdict = "no payload at 0 ms or after the switch (reception, transmission and dropping unresolved)"
        else:
            verdict = f"0 ms: {zero}, after the switch: {after}"
        if "control" in r:
            verdict += f"; control ping at 1 ms: {r['control'][0]}"
        return verdict

    def write_csv(self, path: pathlib.Path) -> None:
        with open(path, "w") as f:
            f.write("t_ns,phase,event,detail,bytes_hex\n")
            for t, phase, event, detail, data in self.events:
                f.write(f"{t},{phase},{event},\"{detail}\",{data.hex()}\n")


def clock_pair() -> str:
    best = None
    for _ in range(2000):
        a = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        r = time.clock_gettime_ns(time.CLOCK_REALTIME)
        b = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        if best is None or b - a < best[2] - best[0]:
            best = (a, r, b)
    assert best is not None
    a, r, b = best
    return (f"monotonic_before_ns {a} realtime_ns {r} monotonic_after_ns {b} "
            f"realtime_minus_monotonic_ns {r - (a + b) // 2} bracket_ns {b - a}")


class Recorder:
    """tcpdump on the adapter's usbmon bus, limited to its device."""

    def __init__(self, bus: int, devnum: int, pcap: pathlib.Path, tcpdump: str) -> None:
        self.cmd = [tcpdump, "-i", f"usbmon{bus}", "-Z", "root", "-s", "0", "-w", str(pcap),
                    f"link[11] == {devnum}"]
        # Pipes, not files: AppArmor would null tcpdump's inherited files.
        # Its own session: a Ctrl-C at the terminal must not end the
        # recording before the cleanup has run.
        self.proc = subprocess.Popen(self.cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, start_new_session=True)
        self.output: list[str] = []

    def wait_ready(self, seconds: float) -> bool:
        assert self.proc.stdout is not None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            r, _, _ = select.select([self.proc.stdout], [], [], 0.1)
            if r:
                line = self.proc.stdout.readline()
                if not line:
                    return False
                self.output.append(line.rstrip())
                if "listening on" in line:
                    return True
        return False

    def alive(self) -> bool:
        return self.proc.poll() is None

    def stop(self) -> int:
        if self.alive():
            self.proc.send_signal(signal.SIGINT)
        try:
            out, _ = self.proc.communicate(timeout=30)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            out, _ = self.proc.communicate()
        self.output += (out or "").splitlines()
        return self.proc.returncode


def capture_problems(rc, pcap: pathlib.Path, output: list[str]) -> list[str]:
    """Why the usbmon capture is not usable evidence (none: it is)."""
    problems = []
    if rc != 0:
        problems.append(f"tcpdump exited {rc}")
    if not pcap.exists():
        problems.append("no capture file")
    elif pcap.stat().st_size <= 24:  # the pcap header alone
        problems.append("the capture holds no packet")
    dropped = [int(m.split()[0]) for m in output if m.strip().endswith("packets dropped by kernel")
               and m.split()[0].isdigit()]
    if not dropped:
        problems.append("tcpdump reported no drop count")
    elif dropped[-1] > 0:
        problems.append(f"{dropped[-1]} packets dropped by kernel")
    return problems


def config_port_baud(config: pathlib.Path) -> tuple[str, int]:
    port, baud = "/dev/ttyUSB0", 3000000
    for line in config.read_text().splitlines():
        s = line.split("#", 1)[0].strip()
        if "=" not in s:
            continue
        key, value = (x.strip() for x in s.split("=", 1))
        if key == "port":
            port = value.strip('"')
        elif key == "baudrate":
            baud = int(value)
    return port, baud


def archived(manifest: str, name: str) -> bool:
    """Whether the manifest holds the file name: every name-like token,
    reduced to its basename (a table row's, a directory-qualified entry's
    or citation's), equal to it or the start of its sidecar's."""
    for token in re.findall(r"[A-Za-z0-9._/+-]+", manifest):
        base = token.rsplit("/", 1)[-1]
        if base == name or base.startswith(name + "."):
            return True
    return False


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("--port", help="serial port (default: config/crane_x7.toml's)")
    p.add_argument("--id", type=int, default=2, help="the servo to ping (default 2)")
    p.add_argument("--window-ms", type=int, default=50, help="each observation window (default 50)")
    p.add_argument("--outdir", default=".", help="where the outputs go (default the current directory)")
    p.add_argument("--date", default=datetime.date.today().strftime("%Y%m%d"))
    p.add_argument("--attempt", type=int, default=1)
    p.add_argument("--manifest", default="docs/records/data-archive.md",
                   help="the archive manifest the output names must not repeat (default %(default)s)")
    p.add_argument("--yes", action="store_true", help="confirm in advance (no prompt)")
    a = p.parse_args(argv)
    if not 1 <= a.id <= 252 or not 10 <= a.window_ms <= 1000 or a.attempt < 1:
        print("latency_hold: --id 1 to 252, --window-ms 10 to 1000, --attempt from 1", file=sys.stderr)
        return 2
    if os.geteuid() != 0:
        print("latency_hold: needs root (sudo): it sets the adapter's latency timer and records usbmon",
              file=sys.stderr)
        return 2
    port, baud = config_port_baud(pathlib.Path("config/crane_x7.toml"))
    port = a.port or port
    if baud not in BAUDS:
        print(f"latency_hold: no termios speed for {baud} bps", file=sys.stderr)
        return 2
    tty = os.path.basename(os.path.realpath(port))
    try:
        adapter = lt.Adapter(tty)
    except (OSError, ValueError) as e:
        print(f"latency_hold: {port} is not a USB serial adapter: {e}", file=sys.stderr)
        return 2
    base = pathlib.Path(a.outdir) / f"rate-hold-{a.date}-{a.attempt}.csv"
    note_path = pathlib.Path(f"{base}.run.txt")
    pcap = pathlib.Path(f"{base}.usbmon.pcap")
    if any(x.exists() for x in (base, note_path, pcap)):
        print(f"latency_hold: attempt {a.attempt} of {a.date} exists; pass a new --attempt", file=sys.stderr)
        return 2
    try:
        manifest = pathlib.Path(a.manifest).read_text(errors="replace")
    except OSError as e:
        print(f"latency_hold: no archive manifest at {a.manifest} to check the output names against "
              f"({e.strerror}): run from the repo root, or pass --manifest", file=sys.stderr)
        return 2
    if archived(manifest, base.name):
        print(f"latency_hold: {base.name} is already in the archive manifest {a.manifest}; "
              "pass a new --attempt", file=sys.stderr)
        return 2
    if not a.yes:
        print(f"This diagnostic pings servo {a.id} on {port} (pings only) and sets the adapter's "
              "latency timer to 0 ms and back to 1 ms. Check that:\n"
              "  - every servo is torque-off and the arm is at rest and supported;\n"
              "  - the power cutoff is within reach;\n"
              "  - nothing else uses the port.")
        try:
            with open("/dev/tty") as t:
                print("Type yes to start: ", end="", flush=True)
                answer = t.readline().strip()
        except OSError:
            answer = ""
        if answer != "yes":
            print("latency_hold: not confirmed; nothing was sent", file=sys.stderr)
            return 2

    record = [f"rtctrl latency hold diagnostic, started {datetime.datetime.now().isoformat()}",
              f"port {port} ({tty}), {baud} bps, servo {a.id}, windows {a.window_ms} ms",
              f"adapter {adapter.vid}:{adapter.pid} bcdDevice {adapter.bcd:04x} channel {adapter.channel}, "
              f"bus {adapter.bus} device {adapter.devnum}"]
    loaded_usbmon = False
    if not pathlib.Path("/sys/module/usbmon").exists():
        if subprocess.run([os.environ.get("MODPROBE", "modprobe"), "usbmon"]).returncode != 0:
            print("latency_hold: modprobe usbmon failed; nothing was sent", file=sys.stderr)
            return 2
        loaded_usbmon = True
    recorder = Recorder(adapter.bus, adapter.devnum, pcap, os.environ.get("TCPDUMP", "tcpdump"))
    record.append("usbmon: " + " ".join(recorder.cmd))
    serial = None
    hold = None
    status = 2
    previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}

    def interrupt(signum, frame):
        raise Interrupted()
    try:
        if not recorder.wait_ready(10):
            record.append("refused: tcpdump did not start")
            print("latency_hold: tcpdump did not start; nothing was sent", file=sys.stderr)
            return 2
        for s in previous:
            signal.signal(s, interrupt)
        try:
            serial = Serial(port, baud)
        except OSError as e:
            record.append(f"refused: cannot open {port}: {e}")
            print(f"latency_hold: cannot open {port} ({e}); nothing was sent", file=sys.stderr)
            return 2
        record.append(f"clock pair before: {clock_pair()}")
        hold = Hold(serial, Latency(adapter), Clock(), a.id, a.window_ms * 1_000_000)
        record.append(f"origin: CLOCK_MONOTONIC {hold.origin} ns (the CSV's t_ns count from it)")
        status = hold.run()
        if status == 0 and not recorder.alive():
            hold.note("aborted: tcpdump ended before the last window")
            status = 1
    except Interrupted:
        status = 130
        if hold is not None:
            hold.note("interrupted")
    finally:
        for s in previous:
            signal.signal(s, signal.SIG_IGN)
        if hold is not None and hold.owed:  # an interrupt during the cleanup itself
            hold.cleanup_ok = hold.cleanup()
            if status == 3 and hold.cleanup_ok:  # the retry left 1 ms after all
                status = 0
        if serial is not None:
            serial.close()
        record.append(f"clock pair after: {clock_pair()}")
        # The restore and the close are done; now the recorder.
        try:
            rc = recorder.stop()
        except (OSError, subprocess.SubprocessError) as e:
            rc = None
            record.append(f"tcpdump did not stop: {e}")
        record.append(f"--- tcpdump (exit {rc}) ---")
        record += recorder.output
        invalid = capture_problems(rc, pcap, recorder.output) if hold is not None else []
        if loaded_usbmon:
            subprocess.run([os.environ.get("MODPROBE", "modprobe"), "-r", "usbmon"])
        if hold is not None:
            if hold.cleanup_ok is False and status == 0:
                status = 3
            if invalid and status == 0:
                status = 1
            record.append("--- sequence ---")
            record += hold.notes
            if invalid:
                record.append("capture: INVALID EVIDENCE: " + "; ".join(invalid))
            record.append(f"outcome: {'INVALID EVIDENCE; observed: ' if invalid else ''}{hold.outcome()}")
            hold.write_csv(base)
        record.append(f"exit status: {status}")
        note_path.write_text("\n".join(record) + "\n")
        uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
        for f in (base, note_path, pcap):
            if uid and gid and f.exists():
                os.chown(f, int(uid), int(gid))
        for line in (hold.notes if hold is not None else []):
            print(line)
        if hold is not None:
            if invalid:
                print("capture: INVALID EVIDENCE: " + "; ".join(invalid))
            print(f"outcome: {'INVALID EVIDENCE; observed: ' if invalid else ''}{hold.outcome()}")
        print(f"record: {note_path} (exit status {status})")
        for s, h in previous.items():
            signal.signal(s, h)
    return status


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
