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
- [Identification protocol](IDENTIFICATION_PROTOCOL.md) — the
  stepped-sine operator procedure (campaign closed 2026-07-28;
  retained as the historical record)
- [Identification campaign checklist](IDENTIFICATION_CAMPAIGN.md) —
  the concrete run order and outcomes, including the stop condition
  that closed the campaign

## Project records

- [PLAN.md](PLAN.md) — the original specification (with its review
  history)
- [Implementation plan](IMPLEMENTATION_PLAN.md) — the approved
  milestone plan, updated with outcomes and hardware findings
- [Remediation plan](REMEDIATION_PLAN.md) — pass-1 instrumentation and
  hardening of the torque loop (complete; pass 2 closed with a null
  result)
- [Identification plan](IDENTIFICATION_PLAN.md) — the pass-2 design
  record, its addenda, and the Closure section (the 0.6 cap is final)
- [Consistency remediation plan](CONSISTENCY_REMEDIATION_PLAN.md) —
  the post-closure audit fixes: verified shutdown, strict CLI, and
  documentation truth
- [Data archive manifest](DATA_ARCHIVE.md) — every archived hardware
  dataset's role and SHA-256; bare-filename citations resolve here
- [Vendor parity](PARITY.md) — `rt_manipulators_cpp` capabilities
  mapped to their rtctrl equivalents and proving tests, plus the
  consciously simplified vendor surface
