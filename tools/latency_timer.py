#!/usr/bin/env python3
"""Read or set an FTDI USB serial adapter's latency timer (needs root).

usage: latency_timer.py read TTY
       latency_timer.py set TTY MS

The torque-off adapter experiment (tools/trace_session.sh --latency-steps)
uses this to set the timer and to check what is in effect. Three values
are read, since the driver's sysfs attribute alone cannot show the
adapter's setting:
- sysfs: /sys/bus/usb-serial/devices/TTY/latency_timer, the driver's cached
  value (ftdi_sio shows 1 whenever the port's low-latency flag is set);
- device: the value the adapter returns to a Get Latency Timer request
  (vendor request 0x0a on endpoint 0, addressed to the port's channel as
  ftdi_sio addresses it), through usbfs;
- low_latency: the port's ASYNC_LOW_LATENCY flag (TIOCGSERIAL), with which
  ftdi_sio writes 1 ms whatever value is stored.
Reading the flag opens the port, which makes the driver send its open-time
reset; the device value is read after it.

read prints one line,
  tty ttyUSB0 sysfs 1 device 1 low_latency 0 adapter 0403:6014 bcdDevice 0900 channel 1
with n/a for a value it cannot read, and exits 0 only if it read all three.
set writes MS (0 to 255) to the sysfs attribute, which the driver writes to
the adapter, then reads the three values back and prints them the same
way; it exits 0 only if sysfs and the device both read MS and the
low-latency flag is clear.

Use it only while no program has the port open.
"""

from __future__ import annotations

import ctypes
import fcntl
import os
import pathlib
import sys

TIOCGSERIAL = 0x541E
ASYNC_LOW_LATENCY = 1 << 13
SERIAL_FLAGS_OFFSET = 16       # struct serial_struct: type, line, port, irq, flags
GET_LATENCY_TIMER = 0x0A
REQUEST_TYPE_VENDOR_IN = 0xC0
CHANNEL_A = 1


class CtrlTransfer(ctypes.Structure):
    """struct usbdevfs_ctrltransfer (linux/usbdevice_fs.h)."""
    _fields_ = [("bRequestType", ctypes.c_uint8), ("bRequest", ctypes.c_uint8),
                ("wValue", ctypes.c_uint16), ("wIndex", ctypes.c_uint16),
                ("wLength", ctypes.c_uint16), ("timeout", ctypes.c_uint32),
                ("data", ctypes.c_void_p)]


# _IOWR('U', 0, struct usbdevfs_ctrltransfer)
USBDEVFS_CONTROL = (3 << 30) | (ctypes.sizeof(CtrlTransfer) << 16) | (ord("U") << 8) | 0


def channel_for(bcd_device: int, interface: int) -> int | None:
    """The channel ftdi_sio addresses (ftdi_determine_type); None for chips
    without a latency timer (SIO, FT232A)."""
    if bcd_device < 0x200:
        return None                       # SIO
    if bcd_device in (0x200, 0x400, 0x600):
        return 0                          # FT232A/B, FT232R
    return CHANNEL_A + interface


class Adapter:
    def __init__(self, tty: str, sysfs: pathlib.Path = pathlib.Path("/sys"),
                 dev: pathlib.Path = pathlib.Path("/dev")) -> None:
        self.tty = tty
        self.sysfs = sysfs
        self.dev = dev
        self.attr = sysfs / "bus/usb-serial/devices" / tty / "latency_timer"
        port = (sysfs / "class/tty" / tty / "device").resolve()
        iface = port.parent
        usb = iface.parent
        self.bus = int((usb / "busnum").read_text())
        self.devnum = int((usb / "devnum").read_text())
        self.vid = (usb / "idVendor").read_text().strip()
        self.pid = (usb / "idProduct").read_text().strip()
        self.bcd = int((usb / "bcdDevice").read_text(), 16)
        self.interface = int((iface / "bInterfaceNumber").read_text(), 16)
        self.channel = channel_for(self.bcd, self.interface)

    def read_sysfs(self) -> int | None:
        try:
            return int(self.attr.read_text())
        except (OSError, ValueError):
            return None

    def write_sysfs(self, ms: int) -> None:
        self.attr.write_text(f"{ms}\n")

    def read_low_latency(self) -> bool | None:
        try:
            fd = os.open(self.dev / self.tty, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError:
            return None
        try:
            buf = bytearray(128)
            fcntl.ioctl(fd, TIOCGSERIAL, buf)
        except OSError:
            return None
        finally:
            os.close(fd)
        flags = int.from_bytes(buf[SERIAL_FLAGS_OFFSET:SERIAL_FLAGS_OFFSET + 4], "little", signed=True)
        return bool(flags & ASYNC_LOW_LATENCY)

    def read_device(self) -> int | None:
        if self.channel is None:
            return None
        path = self.dev / f"bus/usb/{self.bus:03d}/{self.devnum:03d}"
        try:
            fd = os.open(path, os.O_RDWR)
        except OSError:
            return None
        try:
            data = (ctypes.c_uint8 * 1)()
            xfer = CtrlTransfer(REQUEST_TYPE_VENDOR_IN, GET_LATENCY_TIMER, 0, self.channel, 1, 1000,
                                ctypes.cast(data, ctypes.c_void_p))
            libc = ctypes.CDLL(None, use_errno=True)
            n = libc.ioctl(fd, ctypes.c_ulong(USBDEVFS_CONTROL), ctypes.byref(xfer))
            return int(data[0]) if n == 1 else None
        finally:
            os.close(fd)

    def state(self) -> tuple[int | None, int | None, bool | None]:
        low = self.read_low_latency()      # opens the port: read first
        return self.read_sysfs(), self.read_device(), low

    def line(self, state: tuple) -> str:
        sysfs, device, low = state

        def show(v):
            return "n/a" if v is None else str(int(v))
        return (f"tty {self.tty} sysfs {show(sysfs)} device {show(device)} low_latency {show(low)} "
                f"adapter {self.vid}:{self.pid} bcdDevice {self.bcd:04x} "
                f"channel {'n/a' if self.channel is None else self.channel}")


def main(argv: list[str], sysfs: pathlib.Path = pathlib.Path("/sys"),
         dev: pathlib.Path = pathlib.Path("/dev")) -> int:
    if len(argv) < 2 or argv[0] not in ("read", "set") or (argv[0] == "set") != (len(argv) == 3):
        print(__doc__.split("\n\n")[1] if __doc__ else "usage error", file=sys.stderr)
        return 2
    ms = None
    if argv[0] == "set":
        if not argv[2].isdigit() or not 0 <= int(argv[2]) <= 255:
            print(f"latency_timer: MS must be an integer from 0 to 255: {argv[2]}", file=sys.stderr)
            return 2
        ms = int(argv[2])
    try:
        adapter = Adapter(argv[1], sysfs, dev)
    except (OSError, ValueError) as e:
        print(f"latency_timer: {argv[1]} is not a USB serial adapter: {e}", file=sys.stderr)
        return 2
    if ms is not None:
        try:
            adapter.write_sysfs(ms)
        except OSError as e:
            print(f"latency_timer: cannot write {adapter.attr}: {e}", file=sys.stderr)
            return 1
    state = adapter.state()
    print(adapter.line(state))
    sysfs_v, device_v, low = state
    if ms is None:
        return 0 if None not in state else 1
    if sysfs_v != ms or device_v != ms or low is not False:
        print(f"latency_timer: requested {ms} ms, but sysfs reads {sysfs_v}, the adapter {device_v}"
              f"{' and the low-latency flag is set' if low else ''}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
