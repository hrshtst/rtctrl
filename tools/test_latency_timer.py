#!/usr/bin/env python3
"""Regressions for latency_timer.py, run under ctest.

A fake sysfs tree stands for the adapter (an FT232H behind ttyUSB0), and
the two reads that need the real device, the port's flags (TIOCGSERIAL)
and the adapter's Get Latency Timer request (usbfs), are replaced: nothing
here touches a real port. Pinned: the driver's channel mapping, the usbfs
request number, the printed line, and set's checks (sysfs and the adapter
must both read the value, with the low-latency flag clear).
"""

import contextlib
import ctypes
import importlib.util
import io
import pathlib
import sys
import tempfile

spec = importlib.util.spec_from_file_location("lt", "tools/latency_timer.py")
assert spec is not None and spec.loader is not None
lt = importlib.util.module_from_spec(spec)
sys.modules["lt"] = lt
spec.loader.exec_module(lt)


def expect(cond: bool, what: str) -> None:
    if not cond:
        print(f"FAILED: {what}")
        sys.exit(1)


def fake_sysfs(root: pathlib.Path, bcd: str = "0900", interface: str = "00", latency: str = "1") -> None:
    usb = root / "devices/pci0000:00/0000:00:14.0/usb1/1-9"
    port = usb / "1-9:1.0/ttyUSB0"
    port.mkdir(parents=True)
    for name, value in (("busnum", "1"), ("devnum", "4"), ("idVendor", "0403"),
                        ("idProduct", "6014"), ("bcdDevice", bcd)):
        (usb / name).write_text(value + "\n")
    (usb / "1-9:1.0/bInterfaceNumber").write_text(interface + "\n")
    tty = root / "class/tty/ttyUSB0"
    tty.mkdir(parents=True)
    (tty / "device").symlink_to(port)
    attr = root / "bus/usb-serial/devices/ttyUSB0"
    attr.mkdir(parents=True)
    (attr / "latency_timer").write_text(latency + "\n")


class Device:
    """The adapter: a stored latency the driver writes, and the port's flag."""

    def __init__(self, latency: int = 1, low_latency: bool = False, follows: bool = True) -> None:
        self.latency, self.low_latency, self.follows = latency, low_latency, follows
        self.reads = []

    def install(self, root: pathlib.Path) -> None:
        attr = root / "bus/usb-serial/devices/ttyUSB0/latency_timer"
        dev = self

        def write_sysfs(self_, ms):
            attr.write_text(f"{ms}\n")
            if dev.follows:
                dev.latency = 1 if dev.low_latency else ms

        def read_device(self_):
            dev.reads.append(("device", self_.channel))
            return dev.latency

        def read_low_latency(self_):
            dev.reads.append(("flags",))
            return dev.low_latency
        lt.Adapter.write_sysfs = write_sysfs
        lt.Adapter.read_device = read_device
        lt.Adapter.read_low_latency = read_low_latency


def run(argv: list[str], root: pathlib.Path) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = lt.main(argv, sysfs=root, dev=root / "dev")
    return code, out.getvalue(), err.getvalue()


def main() -> int:
    # The channel ftdi_sio addresses, and chips without a latency timer.
    expect(lt.channel_for(0x900, 0) == 1, "FT232H: channel A")
    expect(lt.channel_for(0x700, 1) == 2, "FT2232H's second interface: channel B")
    expect(lt.channel_for(0x600, 0) == 0, "FT232R: channel 0")
    expect(lt.channel_for(0x400, 0) == 0 and lt.channel_for(0x200, 0) == 0, "FT232A/B: channel 0")
    expect(lt.channel_for(0x100, 0) is None, "SIO: no latency timer")
    if ctypes.sizeof(ctypes.c_void_p) == 8:
        expect(lt.USBDEVFS_CONTROL == 0xC0185500, "the usbfs control request number")

    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        fake_sysfs(root)
        dev = Device(latency=1)
        dev.install(root)

        code, out, _ = run(["read", "ttyUSB0"], root)
        expect(code == 0, "read succeeds")
        expect(out.strip() == "tty ttyUSB0 sysfs 1 device 1 low_latency 0 adapter 0403:6014 "
               "bcdDevice 0900 channel 1", f"the printed line: {out!r}")
        expect(dev.reads == [("flags",), ("device", 1)], "the flags read (it opens the port) comes first")

        code, out, _ = run(["set", "ttyUSB0", "0"], root)
        expect(code == 0, "set 0 succeeds when the adapter follows")
        expect("sysfs 0 device 0 low_latency 0" in out, "set reads back")
        expect((root / "bus/usb-serial/devices/ttyUSB0/latency_timer").read_text() == "0\n", "sysfs written")

        dev.follows = False
        code, out, err = run(["set", "ttyUSB0", "1"], root)
        expect(code == 1, "set fails when the adapter keeps another value")
        expect("requested 1 ms, but sysfs reads 1, the adapter 0" in err, f"the mismatch named: {err!r}")

        dev.follows, dev.low_latency, dev.latency = True, True, 1
        code, out, err = run(["set", "ttyUSB0", "0"], root)
        expect(code == 1, "set fails with the low-latency flag set")
        expect("low_latency 1" in out and "the low-latency flag is set" in err, "the flag named")

        dev.low_latency = False
        for bad in (["set", "ttyUSB0", "256"], ["set", "ttyUSB0", "-1"], ["set", "ttyUSB0"], ["get", "ttyUSB0"]):
            code, _, _ = run(bad, root)
            expect(code == 2, f"{bad} refused")
        code, _, err = run(["read", "ttyS0"], root)
        expect(code == 2 and "not a USB serial adapter" in err, "a port without an adapter is refused")

        # A value the program cannot read is n/a, and read then fails.
        lt.Adapter.read_device = lambda self_: None
        code, out, _ = run(["read", "ttyUSB0"], root)
        expect(code == 1 and "device n/a" in out, "an unreadable adapter value")
    print("latency timer regressions OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
