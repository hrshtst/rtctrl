# Getting started

Everything here works **without a robot**; the last section covers
hardware.

## Prerequisites

- Linux, GCC with C++17, CMake ≥ 3.16, git.
- A headless build and the full test suite need only
  `build-essential cmake libxml2-dev liblzf-dev` (the exact set CI
  installs).
- The mi-lib visualization tools (`rk_pen`, `rk_anim`) need the
  X11/GL set as well; install everything with the metapackage's own
  `third_party/mi-lib/install_prereq.sh` (available after the
  submodule init below; `--check` only reports what is missing).
- [`uv`](https://docs.astral.sh/uv/) for the Python tooling
  (model regeneration).
- [`direnv`](https://direnv.net/) is recommended: the bootstrap
  generates and allows an `.envrc` for the mi-lib prefix.
- For hardware only: membership in the `dialout` group and the
  latency udev rule; see the
  [bring-up checklist](../hardware/bringup.md).

## Build

```sh
git clone <repo> rtctrl && cd rtctrl
git submodule update --init third_party/mi-lib \
    third_party/DynamixelSDK third_party/crane_x7_description

# clones + builds the mi-lib stack into .local/ (first run needs
# network); headless machines can skip the X11/GL viewers with
#   MILIB_LIBS="zeda zm zeo dzco roki roki-fd liw" ./tools/bootstrap_milib.sh
./tools/bootstrap_milib.sh
# the bootstrap also generates AND allows .envrc when direnv is
# installed (PATH, LD_LIBRARY_PATH); without direnv, export the two
# lines it prints

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build -L unit       # fast suite
ctest --test-dir build               # + wire-level & dynamics-sim tests
```

`bootstrap_milib.sh` drives the `third_party/mi-lib` metapackage:
member libraries are cloned on the first run and pinned by the
tracked `tools/milib_versions.lock` (the submodule pin governs the
metapackage tooling only), then built and installed in dependency
order into the prefix configured in `third_party/mi-lib/config.local`
(seeded on first run with `<repo>/.local`; override with the first
argument, narrow with `MILIB_LIBS="zeda zm ..."`). CMake finds the
libraries through `cmake/FindMiLib.cmake`, which wraps the installed
`<lib>-config` scripts found on `PATH`; Catch2 and toml++ are fetched
and pinned automatically.

## Five-minute tour (no robot)

**1. Serve a fake CRANE-X7 bus and inspect it:**

```sh
./build/apps/dxl_emu --link /tmp/ttyDXL &     # wire-level emulator
./build/apps/dxl_inspect --port /tmp/ttyDXL scan
./build/apps/dxl_inspect --port /tmp/ttyDXL dump 8
./build/apps/dxl_inspect --port /tmp/ttyDXL \
  dump-params /tmp/x7-params.toml 2 3
./build/apps/dxl_load_params --port /tmp/ttyDXL /tmp/x7-params.toml 2
```

Every hardware `x7_*` app accepts the same `--port /tmp/ttyDXL`: the
entire hardware workflow can be rehearsed offline. (The `*_sim` twins
run pure simulation and take no bus arguments.)
The parameter loader skips unchanged values, so loading an unedited dump is a
readback-verified no-op. See [usage](usage.md#backing-up-and-restoring-motor-parameters)
for selective edits and the safety contract.

**2. Torque cycle and a first motion against the emulator:**

```sh
./build/apps/x7_onoff --port /tmp/ttyDXL 3
./build/apps/x7_move_simple --port /tmp/ttyDXL 6 0.3
./build/examples/x7_wave --port /tmp/ttyDXL 10
```

`x7_move_simple` prints `complete` only after fresh feedback confirms
that every joint returned within 0.05 rad of its starting posture. If a
requested endpoint reaches a soft limit, it warns with the adjusted
displacement; any dropped position command makes the run exit nonzero.

**3. See the model and a simulated motion** (needs roki-gl):

```sh
rk_pen models/crane_x7/crane_x7.ztk            # pose editor
./build/examples/make_motion motion.zvs        # kinematic min-jerk sweep
rk_anim models/crane_x7/crane_x7.ztk motion.zvs

# Cartesian TCP line with continuation-seeded IK at every sample
./build/apps/x7_plan_ptp --config config/ptp_example.toml
rk_anim models/crane_x7/crane_x7.ztk ptp.zvs
uv run --project tools tools/plot/ptp_trajectory.py ptp.csv \
  --output /tmp/ptp-review.png

# Or create a portable archive containing config, model, meshes, and output
./build/apps/x7_plan_ptp --config config/ptp_example.toml \
  --bundle /tmp/ptp-example-bundle
```

`x7_plan_ptp` is offline and never opens the motor bus. Its TOML file
specifies the start and end TCP poses, timing constraints, profile, and
IK policy. See [Cartesian PTP planning](usage.md#cartesian-ptp-planning)
for the full contract.

## Regenerating the model

`models/crane_x7/crane_x7.ztk` is generated from the URDF in
`third_party/crane_x7_description` and committed. To regenerate after
an upstream change:

```sh
uv run --project tools tools/port_model.py
```

The pipeline expands the xacro standalone, rewrites mesh paths
(model-relative: the mi-lib viewers chdir into the model's
directory), converts with roki's `urdf2ztk`, and re-adds what the
converter drops: per-joint `trq` motors bounded by the URDF effort
limits and the nominal viscous damping. Velocity/effort limits live in
`config/crane_x7.toml` (cross-checked against the URDF by a unit
test).

## Configuration

`config/crane_x7.toml` is the deployment config: serial port and baud
rate, plus one `[[joint]]` entry per servo in canonical order: bus
id, model, operating mode (raw Dynamixel value: 3 position,
1 velocity, 0 current), velocity/effort limits and safety margins.
Tests validate it against both the URDF and the canonical joint table.

## Hardware

Follow the [bring-up checklist](../hardware/bringup.md) **in order**
the first time: it interleaves the apps above with physical checks
(and a watchdog drill) and states the safety rules. After bring-up,
`x7_gravity_demo` floats the arm under gravity compensation with
customizable torque constants (see its
[operator page](../hardware/gravity-demo.md)). `x7_track`
(computed-torque tracking) is **parked**: do not run it on hardware.
Its sim twin `x7_track_sim` still previews the identical tracking
run offline, including replay of logged hardware poses and
disturbances.
