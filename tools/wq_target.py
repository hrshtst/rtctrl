#!/usr/bin/env python3
"""The bounded target of the kernel's unbound workqueue mask for a placement that sets it to a list of CPUs (plan L1),
shared by tools/wq_mask.sh (set) and tools/root_keeper.sh (serve --target), so that both compute it the same way.
Neither infers the real-time CPU: the caller names it. Standard library only; it changes nothing.

    wq_target.py target ORIGINAL ISOLATED LIST RT_CPU [--cpu-sysfs DIR]
    wq_target.py census PROC LIST ORIGINAL

target prints four lines, target=M (LIST as a mask in ORIGINAL's layout of 32-bit groups), target_list=LIST,
  rt_cpu=N and rt_siblings=L (RT_CPU's SMT siblings, itself included, as DIR/cpuN/topology/thread_siblings_list
  gives them), if:
  - LIST is a canonical CPU list: ASCII digits without leading zeros, its parts ascending and separated by a gap,
    each a CPU or a range lo-hi with lo < hi (so "4-5" and "1,4-5", never "4,5", "5,4", "4-4", "4-5,5" or "04");
  - RT_CPU is a canonical CPU number, online, and its siblings list holds it;
  - every CPU of LIST is online (DIR/online), in ORIGINAL, not in ISOLATED, and neither RT_CPU nor one of its
    siblings;
  otherwise it exits 2 with the reason. DIR defaults to /sys/devices/system/cpu.
census prints "<workers> <within> <full> <unknown>" over the unbound kernel workers and rescuers (kworker/u*,
  kworker/R-*) under PROC, read as tools/wq_mask.sh's census reads them: within counts those allowed only on LIST's
  CPUs, full those allowed on every CPU of ORIGINAL; unknown counts the entries that exist but cannot be read and the
  workers whose Cpus_allowed_list is missing or malformed (a descending range is malformed, never empty). A gone
  entry counts nowhere.

Exit: 0 done; 2 refused or a usage error (the reason on stderr).
"""

from __future__ import annotations

import os
import re
import sys

LIST_PART = re.compile(r"(0|[1-9][0-9]*)(?:-(0|[1-9][0-9]*))?", re.ASCII)
MASK = re.compile(r"[0-9a-f]+(,[0-9a-f]+)*", re.ASCII)


class Refused(Exception):
    pass


def canonical_list(spec: str) -> list[int]:
    """The CPUs of a canonical list (see above), ascending; Refused otherwise."""
    if not spec:
        raise Refused("an empty CPU list")
    cpus: list[int] = []
    for part in spec.split(","):
        m = LIST_PART.fullmatch(part)
        if not m:
            raise Refused(f"not a canonical CPU list: {spec!r}")
        lo = int(m[1])
        hi = int(m[2]) if m[2] is not None else lo
        if m[2] is not None and hi <= lo:
            raise Refused(f"not a canonical CPU list (a range must ascend, lo < hi): {spec!r}")
        if cpus and lo <= cpus[-1] + 1:
            raise Refused(f"not a canonical CPU list (parts ascending, with a gap between them): {spec!r}")
        cpus.extend(range(lo, hi + 1))
    return cpus


def kernel_list(spec: str) -> set[int]:
    """A CPU list as the kernel writes it (sysfs, Cpus_allowed_list): ascending parts, lo <= hi; ValueError otherwise."""
    out: set[int] = set()
    last = -1
    for part in spec.split(","):
        m = LIST_PART.fullmatch(part)
        if not m:
            raise ValueError(spec)
        lo = int(m[1])
        hi = int(m[2]) if m[2] is not None else lo
        if hi < lo or lo <= last:  # a descending range is malformed, never empty
            raise ValueError(spec)
        out.update(range(lo, hi + 1))
        last = hi
    return out


def mask_value(m: str) -> int:
    if not MASK.fullmatch(m):
        raise Refused(f"not a CPU mask: {m!r}")
    v = 0
    for g in m.split(","):
        v = (v << 32) | int(g, 16)
    return v


def mask_cpus(m: str) -> set[int]:
    v = mask_value(m)
    return {i for i in range(v.bit_length()) if v >> i & 1}


def in_layout(cpus: list[int], layout: str) -> str:
    """The CPUs as a mask in the layout of the mask `layout` (its groups and their widths)."""
    groups = layout.split(",")
    v = 0
    for c in cpus:
        v |= 1 << c
    if v >> (32 * len(groups)):
        raise Refused(f"CPUs beyond the mask's {32 * len(groups)} bits")
    return ",".join(reversed([format((v >> (32 * i)) & 0xFFFFFFFF, "0%dx" % len(g)) for i, g in enumerate(reversed(groups))]))


def read(path: str) -> str:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError as e:
        raise Refused(f"cannot read {path}: {e.strerror}") from None


def target(original: str, isolated: str, spec: str, rt: str, cpu_sysfs: str) -> list[str]:
    orig, iso = mask_cpus(original), mask_cpus(isolated)
    cpus = canonical_list(spec)
    if not re.fullmatch(r"0|[1-9][0-9]*", rt, re.ASCII):
        raise Refused(f"not a CPU number: {rt!r}")
    rt_cpu = int(rt)
    try:
        online = kernel_list(read(f"{cpu_sysfs}/online"))
    except ValueError:
        raise Refused(f"{cpu_sysfs}/online is not a CPU list") from None
    if rt_cpu not in online:
        raise Refused(f"the real-time CPU {rt_cpu} is not online")
    sib_text = read(f"{cpu_sysfs}/cpu{rt_cpu}/topology/thread_siblings_list")
    try:
        siblings = kernel_list(sib_text)
    except ValueError:
        raise Refused(f"CPU {rt_cpu}'s thread_siblings_list is not a CPU list: {sib_text!r}") from None
    if rt_cpu not in siblings:
        raise Refused(f"CPU {rt_cpu}'s thread_siblings_list ({sib_text}) does not hold it")
    for what, bad in (("offline", [c for c in cpus if c not in online]),
                      ("outside the original mask", [c for c in cpus if c not in orig]),
                      ("isolated", [c for c in cpus if c in iso]),
                      (f"the real-time CPU {rt_cpu} or its SMT siblings ({sib_text})", [c for c in cpus if c in siblings])):
        if bad:
            raise Refused(f"the list {spec} names CPUs {what}: {','.join(map(str, bad))}")
    return [f"target={in_layout(cpus, original)}", f"target_list={spec}", f"rt_cpu={rt_cpu}", f"rt_siblings={sib_text}"]


def census(proc: str, spec: str, original: str) -> str:
    allowed_set, orig = set(canonical_list(spec)), mask_cpus(original)
    n = within = full = unknown = 0
    for pid in os.listdir(proc):
        if not pid.isdigit():
            continue
        d = f"{proc}/{pid}"
        try:
            with open(f"{d}/comm") as f:
                comm = f.read().strip()
        except OSError:
            unknown += os.path.exists(d)  # gone: it runs nowhere; still there: perhaps a worker
            continue
        if not comm.startswith(("kworker/u", "kworker/R-")):
            continue
        try:
            with open(f"{d}/status") as f:
                status = f.read().splitlines()
        except OSError:
            unknown += os.path.exists(d)
            continue
        spec_lines = [ln.split(":", 1)[1].strip() for ln in status if ln.startswith("Cpus_allowed_list:")]
        try:
            if len(spec_lines) != 1:
                raise ValueError(spec_lines)
            allowed = kernel_list(spec_lines[0])
        except ValueError:
            unknown += 1
            continue
        n += 1
        within += allowed <= allowed_set
        full += orig <= allowed
    return f"{n} {within} {full} {unknown}"


def main(argv: list[str]) -> int:
    try:
        if len(argv) >= 1 and argv[0] == "target" and len(argv) in (5, 7):
            cpu_sysfs = "/sys/devices/system/cpu"
            if len(argv) == 7:
                if argv[5] != "--cpu-sysfs":
                    raise Refused("usage")
                cpu_sysfs = argv[6]
            print("\n".join(target(argv[1], argv[2], argv[3], argv[4], cpu_sysfs)))
            return 0
        if len(argv) == 4 and argv[0] == "census":
            print(census(argv[1], argv[2], argv[3]))
            return 0
        raise Refused("usage: wq_target.py target ORIGINAL ISOLATED LIST RT_CPU [--cpu-sysfs DIR] | "
                      "census PROC LIST ORIGINAL")
    except Refused as e:
        print(f"wq_target: {e}", file=sys.stderr)
        return 2
    except OSError as e:
        print(f"wq_target: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
