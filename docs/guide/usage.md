# Using the library

All code below is real API; headers live under `include/rtctrl/`.
Link against the `rtctrl::rtctrl` CMake target.

## Model and coordinates

```cpp
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/zvector.hpp"

using namespace rtctrl;

model::ChainModel chain("models/crane_x7/crane_x7.ztk");
model::JointMap map(chain);   // canonical 8-DOF ↔ DXL ids ↔ roki indices

model::ZVector q8(model::kCanonicalDof);   // RAII zVec, starts zeroed
model::ZVector q9(model::kModelDof);
q8[1] = 0.6;                    // shoulder tilt
map.expand(q8, q9);             // finger B mimics finger A
chain.fk(q9);                   // forward kinematics
```

`ChainModel::chain()` exposes the underlying `rkChain*` for any roki
API not wrapped yet — the wrappers are conveniences, not a wall.
Model loading resolves mesh paths relative to the `.ztk` file (the
mi-lib convention), so it works from any working directory.

## Inverse kinematics

```cpp
#include "rtctrl/model/ik_solver.hpp"

model::IkSolver ik(chain, map);          // arm joints only; fingers excluded
zVec3D target_pos; zMat3D target_att;    // world-frame gripper-base pose
// ... fill target ...

model::ZVector init(model::kModelDof), solution(model::kModelDof);
const auto result = ik.solve(target_pos, target_att, init, solution);
if (!result.converged) {
  // unreachable or constrained: residuals/iterations say why.
  // A finite-but-wrong answer never comes back as success.
}
```

`IkResult` carries position/attitude residuals, the iteration count,
and joint-limit/finiteness flags. The solver uses roki's
Levenberg–Marquardt iteration with the error-damped equation solver —
it converges at reachable singular poses (that robustness is pinned by
tests).

## Trajectories and motion files

```cpp
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvs_writer.hpp"

auto traj = model::MinJerkTrajectory::withVelocityLimit(q_start, q_goal,
                                                        /*vel limit*/ 1.0);
model::ZVector q(model::kCanonicalDof), dq(model::kCanonicalDof),
    ddq(model::kCanonicalDof);
traj.sample(t, q, dq, ddq);     // C² quintic; dq/ddq optional

model::ZvsWriter log("motion.zvs");   // one "<dt> <zVec>" line per frame
log.frame(0.01, q9);                  // view: rk_anim <model.ztk> motion.zvs
```

## Writing a controller (the bridge)

A controller is one function, written once, run anywhere:

```cpp
#include "rtctrl/arm/runner.hpp"

struct MyController : arm::Controller {
  void update(const arm::JointState& state, arm::JointCommand& cmd,
              double t) override {
    cmd.mode = arm::ControlMode::Position;      // or Velocity / Current
    zVecCopyNC(state.q.get(), cmd.q.get());     // e.g. hold + offset
    zVecElemNC(cmd.q.get(), 6) = 0.3 * std::sin(t);
  }
};
```

**In simulation** (roki forward dynamics, deterministic, no hardware):

```cpp
#include "rtctrl/arm/sim_arm.hpp"

arm::SimArm::Options opt;
opt.initial_q8 = {0, 0.6, 0, -1.2, 0, -0.5, 0, 0.2};
arm::SimArm sim(opt);
sim.setMode(arm::ControlMode::Position);
sim.activate();                       // no motion on activation
MyController c;
arm::run(sim, c, /*seconds=*/10.0);   // read → update → write → step
```

**On the robot** (identical controller code):

```cpp
#include "rtctrl/arm/real_arm.hpp"
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/hw/crane_x7.hpp"

auto config = hw::Config::load("config/crane_x7.toml");
dxl::Port port(config.port, config.baudrate);
hw::CraneX7 hardware(port, config);
hardware.onEscalate([&port] { port.close(); });  // deadman → bus silence

arm::RealArm robot(hardware);
robot.setMode(arm::ControlMode::Position);  // must match the config's modes
robot.activate();     // verifies servos, arms watchdogs, snaps goals
arm::run(robot, c, 10.0);
robot.deactivate();
```

`readState` gives positions, velocities, and torque estimates
($\hat\tau = k_t\,i$); `step()` blocks on the background read-write
cycle and returns `false` after a safety escalation. The shipped
controllers `arm::GravityComp` and `arm::ComputedTorque` follow
exactly this pattern — see the
[theory documents](../theory/gravity-compensation.md) and their
sources for worked examples.

## Talking to servos directly

For tools and experiments below the bridge:

```cpp
#include "rtctrl/dxl/port.hpp"
#include "rtctrl/dxl/sync_group.hpp"

dxl::Port port("/dev/ttyUSB0", 3000000);
dxl::SyncGroup group(port, {2, 3, 4, 5, 6, 7, 8, 9});
group.setupIndirect();                 // once, torque OFF (EEPROM-gated)

std::vector<dxl::Feedback> fb;
group.readAll(fb);                     // ONE bus transaction, SI units
port.write8(8, dxl::reg::kLed.addr, 1);
```

The `dxl_inspect` app is this API as a CLI. In tests, substitute
`emu::FakePacketIO` for `Port` behind the same `PacketIO` interface —
or run against `dxl_emu`'s pseudo-terminal with `Port` itself.

## Testing your additions

- Pure logic → Catch2 unit test (`tests/unit/`), label `unit`.
- Anything touching the bus → drive it against `emu::FakePacketIO`
  (fast) and, if it exercises new wire behavior, the pty fixture
  (`tests/integration/`).
- Anything producing motion → acceptance test on `SimArm` first;
  hardware only after (that ordering is the project's core rule).
- Line-speed/serial-timing behavior is invisible to pty tests
  (a lesson learned the hard way — see the implementation plan's
  hardware findings): budget a hardware check for it.

## Conventions

- Conventional Commits; a commit is a module plus its tests.
- Canonical joint order everywhere above the `dxl` layer.
- SI units (rad, rad/s, Nm, A, V) outside `dxl/conversions.hpp`;
  raw servo units never leak upward.
- New mi-lib quirks belong in the implementation plan's findings and
  the project memory — several cost hours to discover.
