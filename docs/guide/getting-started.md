# Getting started

Everything here works **without a robot** — the last section covers
hardware.

## Prerequisites

- Linux, GCC with C++17, CMake ≥ 3.16, git.
- System packages: `build-essential cmake libxml2-dev liblzf-dev`
  (mi-lib), and for the visualization tools additionally the X11/GL
  dev set (`freeglut3-dev libglew-dev libglfw3-dev libx11-dev
  libxext-dev libxpm-dev libxft-dev libpng-dev libjpeg-dev libtiff-dev
  libwebp-dev`).
- [`uv`](https://docs.astral.sh/uv/) for the Python tooling
  (model regeneration).
- For hardware only: membership in the `dialout` group and the
  latency udev rule — see the
  [bring-up checklist](../HARDWARE_BRINGUP.md).

## Build

```sh
git clone <repo> rtctrl && cd rtctrl
git submodule update --init third_party/zeda third_party/zm \
    third_party/zeo third_party/dzco third_party/roki \
    third_party/roki-fd third_party/liw third_party/DynamixelSDK \
    third_party/crane_x7_description
# (add third_party/zx11 third_party/roki-gl for rk_pen/rk_anim)

./tools/bootstrap_milib.sh          # builds mi-lib into ~/usr
export PATH="$HOME/usr/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/usr/lib:$LD_LIBRARY_PATH"

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build -L unit       # fast suite
ctest --test-dir build               # + wire-level & dynamics-sim tests
```

`bootstrap_milib.sh` installs the pinned mi-lib submodules in
dependency order into `$HOME/usr` (override with `MILIB_PREFIX` /
first argument; narrow with `MILIB_LIBS="zeda zm ..."`). CMake finds
them through `cmake/FindMiLib.cmake`, which wraps the `<lib>-config`
scripts; Catch2 and toml++ are fetched and pinned automatically.

## Five-minute tour (no robot)

**1. Serve a fake CRANE-X7 bus and inspect it:**

```sh
./build/apps/dxl_emu --link /tmp/ttyDXL &     # wire-level emulator
./build/apps/dxl_inspect --port /tmp/ttyDXL scan
./build/apps/dxl_inspect --port /tmp/ttyDXL dump 8
```

Every `x7_*` app accepts the same `--port /tmp/ttyDXL` — the entire
hardware workflow can be rehearsed offline.

**2. Torque cycle and a first motion against the emulator:**

```sh
./build/apps/x7_onoff --port /tmp/ttyDXL 3
./build/apps/x7_move_simple --port /tmp/ttyDXL 6 0.3
./build/examples/x7_wave --port /tmp/ttyDXL 10
```

**3. See the model and a simulated motion** (needs roki-gl):

```sh
rk_pen models/crane_x7/crane_x7.ztk            # pose editor
./build/examples/make_motion motion.zvs        # kinematic min-jerk sweep
rk_anim models/crane_x7/crane_x7.ztk motion.zvs
```

## Regenerating the model

`models/crane_x7/crane_x7.ztk` is generated from the URDF in
`third_party/crane_x7_description` and committed. To regenerate after
an upstream change:

```sh
uv run --project tools tools/port_model.py
```

The pipeline expands the xacro standalone, rewrites mesh paths
(model-relative — the mi-lib viewers chdir into the model's
directory), converts with roki's `urdf2ztk`, and re-adds what the
converter drops: per-joint `trq` motors bounded by the URDF effort
limits and the nominal viscous damping. Velocity/effort limits live in
`config/crane_x7.toml` (cross-checked against the URDF by a unit
test).

## Configuration

`config/crane_x7.toml` is the deployment config: serial port and baud
rate, plus one `[[joint]]` entry per servo in canonical order — bus
id, model, operating mode (raw Dynamixel value: 3 position,
1 velocity, 0 current), velocity/effort limits and safety margins.
Tests validate it against both the URDF and the canonical joint table.

## Hardware

Follow the [bring-up checklist](../HARDWARE_BRINGUP.md) **in order**
the first time — it interleaves the apps above with physical checks
(and a watchdog drill) and states the safety rules. After bring-up,
the controller phases are `x7_float` (gravity compensation) and
`x7_track` (computed-torque tracking).
