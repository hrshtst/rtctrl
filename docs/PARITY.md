# rt_manipulators_cpp feature parity

Every feature of the vendor library mapped to its rtctrl equivalent and
the test or example that proves it. (Vendor reference:
`rt_manipulators_cpp/rt_manipulators_lib`, deleted after this project
stabilizes.)

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
| software pos-limit rule for vel/current modes | positional gating in the limited writers; rejects commands without fresh feedback | `hw_modes_test` |
| write 0 vel/current on stop | `stopThread` | `thread_test` zeroing case |
| `write_position_pid_gain` / profiles | `writePositionPGain/writeProfileVelocity/Acceleration` | `x7_set_param` on hardware |
| operating modes 0/1/3 from config | `operating_mode` per joint in TOML; mode-checked writers | `hw_modes_test` rejections |
| mock-injected `Communicator` for tests | `dxl::PacketIO` seam + `emu::FakePacketIO` + pty emulator (tests the *unmodified* SDK too) | entire `emu` suite |

## Kinematics / dynamics (vendor `kinematics.hpp`, samples02/03)

| Vendor | rtctrl | Proven by |
|---|---|---|
| link-CSV model + FK | roki chain from URDF-ported ztk; `ChainModel::fk` | `model_test` FK vs URDF ground truth |
| numerical IK (LM) | `IkSolver` (roki LM + error-damped solver, structured `IkResult`) | `ik_test` incl. singular + unreachable |
| analytic 3-DOF IK (samples03) | not ported — the robust numerical solver covers the use case; add analytic seeds later if speed demands | — |
| gravity compensation (samples03) | M7 (`rkChainID_G`-based, canonical 8↔9 mapping) | planned |
| Jacobian utilities | roki `rkChainLinkWldLinJacobi` et al. via `ChainModel::chain()` | used from M7 on |

## Samples

| Vendor sample | rtctrl equivalent |
|---|---|
| samples01 onoff / read_position / write_position / thread / read_present_values / write_velocity / write_current | `apps/x7_onoff`, `apps/x7_read`, `apps/x7_move_simple`, thread: `thread_test` + `examples/x7_wave`, velocity/current: `hw_modes_test` (+ hardware phase of M7/M8) |
| samples02 FK / IK | `examples/make_motion`, `ik_test` |
| samples03 gravity comp / 3-DOF IK | M7 / not ported (see above) |

## Beyond parity (rtctrl only)

- Servo Bus Watchdog + host deadman with bus-silence escalation.
- Firmware/model verification before torque; activation goal snap.
- Sim⇄real bridge (`Arm`): identical controller code on roki dynamics
  and hardware; motor emulator down to the wire protocol.
- Structured IK results; canonical joint mapping with virtual-work
  torque reduction.
