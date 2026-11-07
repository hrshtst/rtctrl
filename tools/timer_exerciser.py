#!/usr/bin/env python3
"""Timer exerciser for the timer service diagnostics' rehearsal (never a
run on the arm; docs/hardware/control-rate.md).

usage: timer_exerciser.py --output FILE --duration S [--cpu N] [--seed N]

A normal-priority process, pinned to --cpu (the real-time CPU in the
rehearsal), that arms one CLOCK_MONOTONIC timerfd at absolute expiries
and, in rounds chosen at random from --seed:
  expire   arms it 200 to 900 us ahead and lets it expire;
  cancel   arms it 300 to 900 us ahead and cancels it before it expires;
  earlier  arms it 500 to 900 us ahead, then re-arms it earlier;
  later    arms it 200 to 500 us ahead, then re-arms it 300 to 600 us later.
Each action's row gives the monotonic times just before and after its
timerfd_settime call and the requested expiry; an expiry seen by the
process (its read returning) is logged as an "expired" row, for
information. The header gives its thread ID, which tools/timer_service.py
matches with the trace's timer events. Rows go to --output as they happen;
SIGINT or SIGTERM ends it with status 0. Needs Python 3.13 or later
(os.timerfd_create).
"""

from __future__ import annotations

import argparse
import os
import random
import signal
import sys
import threading
import time

MONO = time.CLOCK_MONOTONIC
US = 1_000


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=(__doc__ or "").split("\n\n")[0])
    p.add_argument("--output", required=True)
    p.add_argument("--duration", type=float, required=True, help="seconds")
    p.add_argument("--cpu", type=int)
    p.add_argument("--seed", type=int, default=1)
    a = p.parse_args(argv)
    if not hasattr(os, "timerfd_create"):
        print("timer_exerciser: needs Python 3.13 or later (os.timerfd_create)", file=sys.stderr)
        return 2
    if a.duration <= 0:
        print("timer_exerciser: --duration must be positive", file=sys.stderr)
        return 2
    if a.cpu is not None:
        os.sched_setaffinity(0, {a.cpu})
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    rnd = random.Random(a.seed)
    fd = os.timerfd_create(MONO)
    now = time.clock_gettime_ns
    end = now(MONO) + int(a.duration * 1e9)
    with open(a.output, "w", buffering=1) as out:
        out.write(f"# timer_exerciser tid={threading.get_native_id()} cpu={a.cpu} seed={a.seed} "
                  "clock=CLOCK_MONOTONIC\n")
        out.write("round,kind,action,mono_before_ns,mono_after_ns,expiry_ns\n")

        def act(rnd_no: int, kind: str, action: str, expiry: int) -> None:
            before = now(MONO)
            os.timerfd_settime_ns(fd, flags=os.TFD_TIMER_ABSTIME, initial=expiry)
            after = now(MONO)
            out.write(f"{rnd_no},{kind},{action},{before},{after},{expiry}\n")

        def cancel(rnd_no: int, kind: str, expiry: int) -> None:
            before = now(MONO)
            os.timerfd_settime_ns(fd, flags=os.TFD_TIMER_ABSTIME, initial=0)
            after = now(MONO)
            out.write(f"{rnd_no},{kind},cancel,{before},{after},{expiry}\n")

        def expired(rnd_no: int, kind: str, expiry: int) -> None:
            os.read(fd, 8)
            t = now(MONO)
            out.write(f"{rnd_no},{kind},expired,{t},{t},{expiry}\n")

        r = 0
        while not stop.is_set() and now(MONO) < end:
            r += 1
            kind = rnd.choice(("expire", "cancel", "earlier", "later"))
            t = now(MONO)
            if kind == "expire":
                x = t + rnd.randint(200, 900) * US
                act(r, kind, "arm", x)
                expired(r, kind, x)
            elif kind == "cancel":
                x = t + rnd.randint(300, 900) * US
                act(r, kind, "arm", x)
                time.sleep(rnd.randint(50, 150) / 1e6)
                cancel(r, kind, x)
                time.sleep(max(0, x - now(MONO)) / 1e9 + 200e-6)
            elif kind == "earlier":
                x = t + rnd.randint(500, 900) * US
                act(r, kind, "arm", x)
                time.sleep(50e-6)
                y = t + (x - t) // 2  # earlier than x however late the call runs
                act(r, kind, "rearm", y)
                expired(r, kind, y)
            else:
                x = t + rnd.randint(200, 500) * US
                act(r, kind, "arm", x)
                time.sleep(50e-6)
                y = x + rnd.randint(300, 600) * US
                act(r, kind, "rearm", y)
                expired(r, kind, y)
            time.sleep(rnd.randint(1, 3) / 1e3)
    os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
