# rtctrl documentation

Control library for the CRANE-X7 7-DOF arm: robust IK, dynamics-based
control, a sim⇄real bridge, and a fully emulated test path down to the
wire protocol.

## Guide

1. [Overview & philosophy](guide/overview.md) — what this is and the
   principles behind it
2. [Getting started](guide/getting-started.md) — build, test, and the
   five-minute no-robot tour
3. [Architecture](guide/architecture.md) — layers, canonical
   coordinates, the bridge, safety
4. [Usage](guide/usage.md) — the library as an API, with code

## Theory

- [Dynamics foundations](theory/dynamics-foundations.md) — equations of
  motion, the 8↔9 coordinate spaces, virtual-work force mapping,
  torque↔current
- [Gravity compensation](theory/gravity-compensation.md) — the floating
  arm: law, stability, verification
- [Computed-torque control](theory/computed-torque.md) —
  inverse-dynamics feedforward tracking: law, error dynamics,
  trajectories, results, and what the hardware taught us

## Hardware

- [Bring-up checklist](HARDWARE_BRINGUP.md) — the ordered physical
  procedure, safety rules, watchdog drill, troubleshooting

## Project records

- [PLAN.md](PLAN.md) — the original specification (with its review
  history)
- [Implementation plan](IMPLEMENTATION_PLAN.md) — the approved
  milestone plan, updated with outcomes and hardware findings
- [Vendor parity](PARITY.md) — `rt_manipulators_cpp` capabilities
  mapped to their rtctrl equivalents and proving tests, plus the
  consciously simplified vendor surface
