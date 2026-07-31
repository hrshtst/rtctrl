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
  the bus if command writes stall *or feedback reads fail persistently*,
  which forces the servo watchdogs to fire). These protect against
  *communication* loss, not against wrong commands: stay clear of the
  arm's envelope while it is torqued.
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

Serial-port permission: `sudo usermod -aG dialout $USER` (then log out
and back in — this must be done even if the port initially works: a
desktop session grants a temporary device ACL that does NOT survive
unplugging and replugging the adapter, which the watchdog drill does).

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
   `./build/apps/x7_onoff 30`, then pull the USB cable mid-hold.
   Expected (verified on hardware): the arm **freezes in place with
   torque still on** — the servo Bus Watchdog halts motion and locks
   out goal commands (register 98 reads 255). It does NOT go limp;
   freezing is the firmware's designed comm-loss response and the safer
   one for a vertical arm. To release afterwards: reconnect USB (wait
   ~10 s for the adapter to re-enumerate), then run
   `./build/apps/x7_onoff 2` — activation clears the tripped watchdogs
   and hands the arm back gently — or simply power-cycle the servos.

6. **First motion** — wrist only, small and slow:
   `./build/apps/x7_move_simple 6 0.3`
   ~0.3 rad out and back at ≤0.5 rad/s. Then, if clean, try another
   single joint. Multi-joint motion belongs to M6.

## After bring-up (M6–M8)

The controller phases build on this checklist in order:
`examples/x7_wave` (multi-joint position mode), `apps/x7_float`
(gravity compensation — the arm floats and is hand-guidable), and
`apps/x7_track` (computed-torque tracking; rehearse the identical run
offline with `apps/x7_track_sim` first). `x7_track` settles the arm
before moving, refuses to start from a joint parked at a soft limit,
and caps its excursion at the verified stability envelope — the
reasons live in the
[computed-torque theory notes](theory/computed-torque.md#what-the-hardware-taught-us).
Feature coverage is mapped in [PARITY.md](PARITY.md).

> **`x7_track` and `x7_ident` are PARKED — do NOT run them on
> hardware. `x7_float` is UN-PARKED (2026-07-31) under the
> conditions in its bullet below.** Two independent incidents:
>
> - **`x7_track` (2026-07-28):** a scale-0.5 session never left the
>   settle phase — from a compact resting posture the pan (canonical
>   joint 0) anti-damped at 4.33 Hz, growing ~50 → ~350 mrad
>   peak-to-peak within a second of torque-on; operator power cut.
>   The failure precedes the scale argument (no scale is safe) and no
>   code or configuration change preceded it. Parked until a
>   reviewer-approved settle-phase fix lands.
> - **`x7_float` (2026-07-29 incident; UN-PARKED 2026-07-31):** a
>   float session accelerated the UNTOUCHED arm toward the upright
>   posture (peak ~2.37 rad/s, joints riding their upper limits) —
>   gravity over-compensation, root-caused to the torque-to-current
>   calibration commanding 23–49 % more current per model-torque
>   than the vendor's empirically tuned constants. The remediation
>   PASSED its objective acceptance (M-GC3); the subjective
>   back-drive criterion FAILED and the owner explicitly WAIVED it
>   (a risk/quality acceptance, not a test pass) covering two
>   characterized behaviors: the j1 notch (energized actuator-side
>   behavior, strongly associated with crossing the low-current
>   transition region q1 ≈ +0.27…+0.53 rad; mechanism not isolated)
>   and a small j4 positive tendency under hand-guiding (command
>   ≤ 0.055 Nm). CONDITIONS: `x7_float` refuses to touch the bus
>   without the approved vendor calibration
>   (`config/crane_x7_vendor_scale.toml` — default-scale adoption
>   was declined, so the repo default config remains the
>   known-failed all-1.0 configuration); the marker protocol is
>   unchanged; power cutoff within reach; unique `--log` filename
>   per attempt. Full record:
>   [GRAVITY_CALIBRATION_PLAN.md](GRAVITY_CALIBRATION_PLAN.md).
> - **`x7_ident` remains PARKED** pending its own disposition: it
>   runs current mode through the SAME conversion and stages its
>   own preload, but performs autonomous excitation maneuvers with
>   different operator exposure — un-parking requires a separate
>   reviewer + owner decision tied to a concrete campaign (its
>   pass-2 campaign is closed, so parking costs nothing).
>
> Telemetry of both incidents is ARCHIVED with manifest rows —
> `track9.csv`/`track9.csv.settle` and `float1.csv`, section
> `incidents-2026-07/` in [DATA_ARCHIVE.md](DATA_ARCHIVE.md).
> `x7_wave`, `x7_move_simple`,
> `x7_pose`, `x7_read`, and `x7_onoff` remain usable (position mode
> or no torque; none touches the current-command path); the
> post-incident recovery ladder is steps 1–6 above.

## Troubleshooting

- `no response from id N`: check baud (3 Mbps), cabling, power.
- `firmware vNN lacks Bus Watchdog`: update servo firmware with the
  ROBOTIS tools before proceeding — activation intentionally refuses.
- `DEADMAN: command stream stale`: the host silenced the bus and the
  servos halted themselves. Two causes: the host command loop stalled
  (investigate latency_timer, CPU load), or the feedback reads died —
  e.g. servo power was cut mid-run — which apps report as a nonzero
  `read failures` count.
- Offline rehearsal of every step: `./build/apps/dxl_emu --link
  /tmp/ttyDXL &` then add `--port /tmp/ttyDXL` to any app.
