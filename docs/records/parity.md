# rt_manipulators_cpp feature parity

The vendor library's *capabilities* mapped to their rtctrl equivalents
and the test or example that proves each — functional coverage for
whole-arm CRANE-X7 control, not a 1:1 API port. Consciously simplified
or omitted vendor surface is listed at the end. (Vendor reference:
`third_party/rt_manipulators_cpp/rt_manipulators_lib` — originally slated
for post-stabilization deletion, kept for reference per the owner's
2026-08-03 decision.)

## Hardware class

| Vendor (`rt_manipulators_cpp::Hardware`) | rtctrl | Proven by |
|---|---|---|
| `connect/disconnect` | `dxl::Port` ctor / `close()` | pty tests; hardware bring-up |
| `load_config_file` (YAML) | `hw::Config::load` (TOML) | `hw_test` config case |
| `torque_on/off(group)` | `CraneX7::activate/deactivate` (adds fw check, watchdog arming, goal snap, soft gains) | `hw_test` activation cases; hardware checklist step 3 |
| `sync_read(group)` | `SyncGroup::readAll` (one FastSync-style indirect read: pos/vel/current/voltage/temp) | `dxl_test`, pty tests |
| `sync_write(group)` | `SyncGroup::writeGoal*` | `dxl_test`, pty tests |
| `start_thread/stop_thread` | `CraneX7::startThread/stopThread` (+ cycle stats, deadman integration, zero-on-stop) | `thread_test` |
| `get_position(s)/velocities/currents/voltage/temperature` | `CraneX7::lastFeedback` / `RealArm::readState` (canonical vectors) | `thread_test` bridge case |
| `set_position(s)` | `writePositions` / `setTargetPositions` (limit-clamped) | `hw_test` clamp case |
| `set_velocity/velocities` | `writeVelocities` (velocity-limit clamp + position gating) | `hw_modes_test` |
| `set_current(s)` | `writeCurrents` (effort→A and servo CurrentLimit clamp + gating) | `hw_modes_test` |
| software pos-limit rule for vel/current modes | positional gating in the limited writers; requires previously ACQUIRED feedback (a command with no feedback ever read is rejected; persistent read failures escalate via the bounded-failure deadman policy — there is deliberately no per-command freshness gate) | `hw_modes_test` gating cases (velocity AND current modes) + no-feedback rejections |
| write 0 vel/current on stop | `stopThread` | `thread_test` zeroing cases (velocity and current; non-vacuous: fresh submissions, no escalation, nonzero goal proven immediately before the stop) |
| `write_position_pid_gain` / `write_velocity_pi_gain` / SI-unit profile setters | `writePositionPGain` (position P only — see omissions); profiles in BOTH forms: raw register passthroughs and `writeProfileVelocityRadPerSec` / `writeProfileAccelerationRadPerSec2` with the vendor-exact truncation, [1, 32767] clamp, and slowest-not-unlimited zero semantics. The hardware-facing `x7_set_param` additionally requires every servo to report torque off and verifies exact before/after reads. | `hw_modes_test` SI-profile case; `bringup_test` torque/readback cases; `x7_set_param` on hardware |
| operating modes 0/1/3 from config (the vendor also supports mode 5 — see omissions) | `operating_mode` per joint in TOML; mode-checked writers | `hw_modes_test` rejections |
| mock-injected `Communicator` for tests | `dxl::PacketIO` seam + `emu::FakePacketIO` + pty emulator (tests the *unmodified* SDK too) | entire `emu` suite |

## Kinematics / dynamics (vendor `kinematics.hpp`, samples02/03)

| Vendor | rtctrl | Proven by |
|---|---|---|
| link-CSV model + FK | roki chain from URDF-ported ztk; `ChainModel::fk` | `model_test` FK vs URDF ground truth |
| numerical IK (LM) | `IkSolver` (roki LM + error-damped solver, structured `IkResult`) | `ik_test` incl. singular + unreachable |
| analytic 3-DOF IK (samples03) | not ported — the robust numerical solver covers the use case; add analytic seeds later if speed demands | — |
| gravity compensation (samples03) | `arm::GravityComp` (`rkChainID_G`-based, canonical 8↔9 mapping) | `gravity_test` (potential-gradient identity), `gravity_sim_test`, `apps/x7_float` on hardware (measured vs predicted within ~0.03 Nm) |
| Jacobian utilities | available through the raw roki escape hatch (`ChainModel::chain()`), currently UNWRAPPED and UNUSED — no rtctrl call or test exercises that surface; wrap when task-space control needs it | — |

## Samples

| Vendor sample | rtctrl equivalent |
|---|---|
| samples01 onoff / read_position / write_position / thread / read_present_values / write_velocity / write_current | `apps/x7_onoff`, `apps/x7_read`, `apps/x7_move_simple` (soft-limit-adjusted endpoint plus measured return gate), thread: `thread_test` + `examples/x7_wave`, velocity/current: `hw_modes_test` + `apps/x7_float`/`apps/x7_track` on hardware |
| samples02 FK / IK | `examples/make_motion`, `ik_test` |
| samples03 gravity comp / 3-DOF IK | `apps/x7_float` / not ported (see above) |

## Consciously simplified or not ported

Intentional design decisions, not gaps (recorded in the implementation
plan's post-completion review notes):

- **Named, multiple joint groups and per-ID/per-name accessors** — the
  vendor's `Hardware` supports arbitrary group definitions; rtctrl
  exposes exactly one ordered all-joint group, because the canonical
  joint order *is* the project's coordinate contract (`Config::load`
  now rejects any other ordering). Sub-group control has no CRANE-X7
  use case here yet; add named groups if one appears.
- **Position I/D and velocity PI gain writers** — the vendor exposes
  position PID and velocity PI writers (it has NO feedforward-gain
  writers); rtctrl exposes only the position P gain (the soft-start
  knob activation needs). rtctrl's dynamics controllers do their
  feedback host-side in torque mode instead of tuning servo-internal
  loops, and `dxl_inspect` reaches any register raw when a one-off
  tuning experiment demands it. Add dedicated writers only if
  servo-loop tuning becomes an operational workflow.
- **Current-based position mode (operating mode 5)** — the vendor
  supports it; rtctrl's config deliberately rejects it
  (`src/hw/config.cpp`). Supporting it properly needs a command
  model, activation/limit behavior, and `RealArm` semantics of its
  own — not merely permitting the configuration value. Add it when a
  concrete use case appears.
- **Analytic 3-DOF IK** — see the table above (robust numerical solver
  covers the use case).
- **FastSyncRead (0x8A)** — the port uses ordinary `GroupSyncRead`:
  measured at 3 Mbps it sustains the 100 Hz cycle over 8 servos with
  zero overruns, so the fast variant remains an unneeded optimization.
  The wire-level emulator likewise implements only the ordinary
  variant.
- **SDK `.model` file parsing** — `dxl_inspect` ships its own register
  table (`control_table.hpp` is the single source of truth) rather
  than reading the DynamixelSDK's `.model` data files.

## Beyond parity (rtctrl only)

- Servo Bus Watchdog + host deadman with bus-silence escalation — on
  stale command writes *and* on persistently failing feedback reads
  (the frozen-feedback trap).
- Computed-torque trajectory tracking (`arm::ComputedTorque`,
  `apps/x7_track`): inverse-dynamics feedforward + hardware-hardened
  filtered PID — no vendor equivalent exists.
- Offline twin of the tracking run (`apps/x7_track_sim`) with logged
  hardware-pose replay, disturbance seeding, and lag models.
- Firmware/model verification before torque; activation goal snap.
- Sim⇄real bridge (`Arm`): identical controller code on roki dynamics
  and hardware; motor emulator down to the wire protocol.
- Structured IK results; canonical joint mapping with virtual-work
  torque reduction.

## Failure behavior: bus loss and actuator power loss

How the two libraries behave through the two failure drills every
hardware session must anticipate. (Either way, `deactivate()` is not
an e-stop and the actuator power cutoff stays within reach.)

**USB disconnect — servos powered, bus dead:**

| Vendor (`rt_manipulators_cpp::Hardware`) | rtctrl | Proven by |
|---|---|---|
| Nothing: the library never arms a watchdog, so the servos complete the last written goal at profile speed and hold it, torque on, indefinitely; host-side, `read_write_thread` loops printing errors — no failure path clears `thread_enable_` | Servo Bus Watchdog halts motion within 100 ms (mid-profile), locks the goal registers, torque stays on — a freeze, not a fall. Host-side, two paths by session type: background-thread sessions (`startThread`, i.e. `RealArm`) escalate after five consecutive failed reads — best-effort release, then the escalation hook closes the port; simple foreground apps abort on the first failed read, record every failed command write, and return nonzero after even a transient dropped command. Persistent write failure reaches the host deadman and bus-silence path. | `dxl_test` watchdog cases; `hw_test` escalation cases (thread path); `bringup_test` write monitoring; bring-up checklist step 5 (USB-pull drill = the foreground path, verified on hardware) |

**Actuator power cutoff — servos dead, bus alive:** the physics is
identical for both libraries — torque vanishes at the hardware level
and the arm falls limp; no software changes that. The during and
after differ:

| Vendor | rtctrl | Proven by |
|---|---|---|
| No detection: errors print to `cerr` until the operator kills the process; the session object stays live and willing | Bounded detection: the background thread escalates after five failed reads (foreground apps abort on the first) and the shipped session wiring closes the port; the app exits through its shutdown guard | `hw_test` read/write-failure escalation cases; `track9.csv` incident record |

Recovery deserves a fair statement. For the disciplined workflow —
cut power, kill the host, restart both — the end states are
*equivalent*: rebooted servos come up torque-off, and a fresh vendor
session's `torque_on` is jump-safe (XM firmware snaps the goal
register to present at enable; the library then aligns its host-side
goals from a sync read). The real difference is **who enforces that
workflow**. The vendor library keeps a zombie session alive across
the outage: its thread resumes sync-writing the pre-cut goals the
moment rebooted servos answer, and a later `torque_on` *without a
process restart* overwrites the enable-snap with those stale targets
on the next cycle — a commanded jump to the pre-cut target. Safe by
convention; the convention erodes under supervisors, retry logic, or
any long-lived process. rtctrl closes the class in its shipped
session wiring: every `x7_*` app's `openSession` installs an
escalation hook that closes the port, after which nothing — reads
included — can transmit, and a restart with a fresh `activate()`
(re-verifying the servos, snapping goals to the arm's *present*
posture) is the only path back onto the bus. Scope this claim
precisely: it is a property of the session layer, not the bare
`CraneX7` class — at the class level the command writers reject while
`escalated_` is set, but `readAll` remains callable and a deliberate
`activate()` clears the latch, so a caller that wires no escalation
hook keeps only the writer lockout. In one line: the vendor's
recovery safety is procedural, rtctrl's is mechanical in the shipped
applications.
