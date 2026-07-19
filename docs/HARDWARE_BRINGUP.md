# CRANE-X7 hardware bring-up (M5 checklist)

Every app below was verified against the emulator first (`dxl_emu`).
Run the steps **in order**; do not skip ahead.

## Safety — read first

- **`deactivate()` is not an emergency stop.** Keep the arm's power
  switch (actuator supply cutoff) **within reach** for every powered
  step. If anything looks wrong: cut power. The arm will fall limp —
  keep the workspace under it clear.
- Two watchdog layers run during every powered app:
  the servo-side Bus Watchdog (100 ms — halts the servos if the host
  dies or the cable drops) and the host-side deadman (250 ms — silences
  the bus if command writes stall, which forces the servo watchdogs to
  fire). These protect against *communication* loss, not against wrong
  commands: stay clear of the arm's envelope while it is torqued.
- First motion is the **wrist (canonical joint 6, DXL id 8)** only,
  small and slow, before any multi-joint motion.

## One-time host setup

USB latency (needed for stable 100 Hz+ cycles on FTDI adapters):

```
# /etc/udev/rules.d/99-crane-x7.rules
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

```
sudo udevadm control --reload-rules && sudo udevadm trigger
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # expect: 1
```

Serial-port permission: `sudo usermod -aG dialout $USER` (re-login).

## Steps

Power on the arm, connect USB, then:

1. **Scan** (no torque, safe):
   `./build/apps/dxl_inspect --port /dev/ttyUSB0 scan`
   Expect ids 2–9; id 3 reports model 1120 (XM540-W270), the rest 1020.
   Then dump each servo's registers by its bus id —
   `./build/apps/dxl_inspect --port /dev/ttyUSB0 dump <id>` for ids 2–9
   (id 2 = shoulder pan … id 8 = wrist rotate, id 9 = gripper) — and
   check `firmware_version >= 38` on every one (Bus Watchdog support —
   activation refuses older firmware).

2. **Read streaming** (no torque, safe):
   `./build/apps/x7_read 20`
   Move joints by hand; positions must track smoothly, voltage ~12 V,
   temperatures sane. This validates the sync-read path and the
   canonical joint order against the physical arm.

3. **ON/OFF** (first torque — hands clear, power switch in reach):
   `./build/apps/x7_onoff 3`
   Expect: **zero motion** on activation, a gentle sag on deactivation
   (limp gains before torque-off). If anything jumps: cut power and
   stop here.

4. **Parameter modification** (torque off):
   `./build/apps/x7_set_param --p-gain 640` then restore `--p-gain 800`.
   Values must read back as written.

5. **Watchdog drill** (recommended once, before any motion): run
   `./build/apps/x7_onoff 30`, then pull the USB cable mid-hold. The
   arm must go limp on its own within ~100 ms (servo Bus Watchdog).
   Reconnect, power-cycle the servos, rerun `scan`.

6. **First motion** — wrist only, small and slow:
   `./build/apps/x7_move_simple 6 0.3`
   ~0.3 rad out and back at ≤0.5 rad/s. Then, if clean, try another
   single joint. Multi-joint motion belongs to M6.

## Troubleshooting

- `no response from id N`: check baud (3 Mbps), cabling, power.
- `firmware vNN lacks Bus Watchdog`: update servo firmware with the
  ROBOTIS tools before proceeding — activation intentionally refuses.
- `DEADMAN: command stream stale`: the host loop stalled; the bus was
  silenced and the servos halted themselves. Investigate host timing
  (latency_timer, CPU load) before retrying.
- Offline rehearsal of every step: `./build/apps/dxl_emu --link
  /tmp/ttyDXL &` then add `--port /tmp/ttyDXL` to any app.
