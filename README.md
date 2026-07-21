# rtctrl

A C++17 control library for the
[CRANE-X7](https://rt-net.jp/products/crane-x7/) 7-DOF robot arm.

- **Robust inverse kinematics** (error-damped Levenberg–Marquardt via
  [mi-lib/roki](https://github.com/mi-lib), structured results)
- **Dynamics-based control**: gravity compensation and computed-torque
  trajectory tracking through Dynamixel current control
- **Sim⇄real bridge**: controllers run unchanged on a roki dynamics
  simulator and on the hardware
- **Test-driven down to the wire**: an XM-servo emulator serves
  Protocol 2.0 over a pseudo-terminal, so the unmodified DynamixelSDK
  path is covered in CI — no robot required
- **Layered safety**: servo-side Bus Watchdog plus a host deadman
  (stale commands or frozen feedback) that escalates to bus silence;
  activation cannot cause motion

Robotics computation is delegated to
[mi-lib](https://github.com/mi-lib), motor communication to
[DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK); rtctrl is
the CRANE-X7-specific layer joining them.

## Quick start

```sh
git submodule update --init third_party/zeda third_party/zm \
    third_party/zeo third_party/dzco third_party/roki \
    third_party/roki-fd third_party/liw third_party/DynamixelSDK \
    third_party/crane_x7_description
./tools/bootstrap_milib.sh
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
ctest --test-dir build

# no robot? rehearse everything against the emulator:
./build/apps/dxl_emu --link /tmp/ttyDXL &
./build/apps/dxl_inspect --port /tmp/ttyDXL scan
```

## Documentation

Start at **[docs/](docs/README.md)**: a user guide (philosophy,
architecture, getting started, usage), the control-theory notes with
full derivations, the hardware bring-up checklist, and the project
records.

Hardware sessions: read
[docs/HARDWARE_BRINGUP.md](docs/HARDWARE_BRINGUP.md) first — and keep
the power cutoff within reach.
