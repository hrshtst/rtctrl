# Dynamics foundations

Shared background for the control-theory documents
([gravity compensation](gravity-compensation.md),
[computed torque](computed-torque.md)): the rigid-body model, the
coordinate spaces of the CRANE-X7, and how torques map between them.

## Equations of motion

A serial rigid-body chain with generalized coordinates
$q \in \mathbb{R}^n$ obeys

```math
M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + g(q) = \tau,
```

where

- $M(q) \in \mathbb{R}^{n\times n}$ is the symmetric, positive-definite
  joint-space inertia matrix,
- $C(q,\dot{q})\,\dot{q}$ collects Coriolis and centrifugal torques,
- $g(q)$ is the gravity torque vector,
- $\tau$ is the vector of joint torques applied by the actuators.

The gravity term is the gradient of the potential energy
$U(q) = \sum_i m_i\, \mathbf{g}^\mathsf{T} \mathbf{p}_{c_i}(q)$
(taking $\mathbf{g}$ pointing up and $\mathbf{p}_{c_i}$ the world
position of link $i$'s center of mass):

```math
g(q) = \frac{\partial U}{\partial q}.
```

rtctrl never forms these matrices symbolically. roki's recursive
Newton–Euler implementation (`rkChainID_G`) evaluates the **inverse
dynamics** function directly:

```math
\mathrm{ID}(q, \dot{q}, \ddot{q}) \;=\;
M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + g(q),
```

i.e. the torque that *realizes* a given motion state. Both controllers
below are built from calls to this one function.

## Coordinate spaces of the CRANE-X7

Three different dimensions coexist and must never be confused:

| Space | Dim | Contents |
|---|---|---|
| canonical (controller) | 8 | arm joints 1–7 + one gripper coordinate |
| actuator (bus) | 8 | Dynamixel IDs 2–9 (gripper servo drives finger A) |
| model (roki) | 9 | 7 arm joints + finger A + finger B |

Finger B mechanically mimics finger A with multiplier 1. The model
keeps it as a real revolute joint (correct mass distribution and
collision geometry), so the canonical and model spaces are related by
the **selection matrix** $E \in \mathbb{R}^{9\times 8}$ of the
kinematic constraint $q_{fB} = q_{fA}$:

```math
q_9 = E\, q_8,
\qquad
E_{ij} =
\begin{cases}
1 & \text{model joint } i \text{ is canonical joint } j,\\
1 & i = f_B,\ j = \text{gripper},\\
0 & \text{otherwise.}
\end{cases}
```

Velocities and accelerations expand with the same $E$
($\dot q_9 = E \dot q_8$, $\ddot q_9 = E \ddot q_8$).

## Force mapping: the virtual-work identity

Generalized forces map the *opposite* way. For any virtual displacement
$\delta q_8$ the two descriptions must do equal work,

```math
\tau_8^\mathsf{T}\, \delta q_8
  = \tau_9^\mathsf{T}\, \delta q_9
  = \tau_9^\mathsf{T} E\, \delta q_8
\quad\Longrightarrow\quad
\boxed{\;\tau_8 = E^\mathsf{T} \tau_9\;}
```

so the canonical gripper torque is the **sum** of both finger torques,
$\tau_{\text{grip}} = \tau_{fA} + \tau_{fB}$ — not finger A's alone.

In code, `model::JointMap` *is* $E$: `expand()` applies $E$,
`reduceTorque()` applies $E^\mathsf{T}$ (`joint_map.cpp`), with the
index mapping resolved by link name at load time. The unit test
`JointMap torque reduction satisfies virtual work` checks the boxed
identity numerically for arbitrary force fields.

## Torque and current

Dynamixel XM servos in current-control mode regulate winding current,
which maps to output-shaft torque through an effective torque constant
$k_t$ (gearbox included):

```math
\tau = k_t\, i,
\qquad
k_t =
\begin{cases}
1.783\ \mathrm{Nm/A} & \text{XM430-W350 (all joints but 2)},\\
2.409\ \mathrm{Nm/A} & \text{XM540-W270 (joint 2)}.
\end{cases}
```

`dxl::conversions.hpp` owns these constants; the bridge converts
controller torques to current commands, and conversely estimates
$\hat\tau = k_t\, i_{\text{measured}}$ from the measured current
(`RealArm::readState`). The constants are nominal values from the
manufacturer data; friction and gear efficiency are not modeled, which
bounds the fidelity of $\hat\tau$ (measured on hardware to be a few
percent at static poses — see the M7 notes in the
[implementation plan](../IMPLEMENTATION_PLAN.md)).
