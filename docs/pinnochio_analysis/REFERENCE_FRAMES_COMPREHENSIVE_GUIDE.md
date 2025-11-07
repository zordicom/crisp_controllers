# Pinocchio Reference Frames – Quick Guide

Quick reference for the three reference frame options in Pinocchio (`WORLD`, `LOCAL`, and `LOCAL_WORLD_ALIGNED`).

> **TL;DR:**
>
> - Use **`WORLD`** for dynamics-based control (operational space, dynamic nullspace, RNEA/CRBA)
> - Use **`LOCAL_WORLD_ALIGNED`** for nearly all practical robot control (kinematic impedance, velocity control, IK, trajectory planning, manipulation)
> - Use **`LOCAL`** rarely (only for specialized tool-axis-relative control)
>
> **For detailed explanations**, see `WORLD_JACOBIAN_SPATIAL_VELOCITY_EXPLAINED.md`.

---

## The Three Frames

All three describe **the same physical motion**, differing only in measurement point and axes:

| Frame | Measurement Point | Axes Orientation | Linear Velocity Represents |
|-------|------------------|------------------|----------------------------|
| **WORLD** | Imaginary point (at world origin, attached to EE body) | World axes | Velocity of imaginary point on EE at origin |
| **LOCAL_WORLD_ALIGNED** | End-effector point | World axes | Velocity at actual EE in world coords |
| **LOCAL** | End-effector point | EE axes | Velocity at actual EE in EE coords |

### WORLD Frame

`J = pin.computeFrameJacobian(model, data, q, ee_id, pin.WORLD)`

Spatial velocity measured at an **imaginary point** rigidly attached to the EE body, located at the world origin. Used for dynamics algorithms.

$$
J_{\text{WORLD}}(q) \dot{q} = \begin{bmatrix}
\mathbf{v}_{\text{origin}} \\
\boldsymbol{\omega}
\end{bmatrix}
$$

- **Top 3 rows:** $\mathbf{v}_{\text{origin}}$ - velocity of imaginary point at origin (NOT actual EE!)
- **Bottom 3 rows:** $\boldsymbol{\omega}$ - angular velocity of EE body

### LOCAL_WORLD_ALIGNED Frame

`J = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL_WORLD_ALIGNED)`

Velocity at the actual end-effector point, expressed in world axes. The familiar textbook Jacobian.

$$
J_{\text{ALIGNED}}(q) \dot{q} = \begin{bmatrix}
\mathbf{v}_{\text{ee}} \\
\boldsymbol{\omega}
\end{bmatrix}
$$

- **Top 3 rows:** Velocity of actual EE point (what you usually want!)
  - $\mathbf{v}_{\text{ee}} = \frac{\partial \mathbf{p}_{\text{ee}}}{\partial q}$
- **Bottom 3 rows:** $\boldsymbol{\omega}$ - angular velocity of EE body

### LOCAL Frame

`J = pin.computeFrameJacobian(model, data, q, ee_id, pin.LOCAL)`

Velocity at the actual end-effector point, expressed in the EE's own body axes. For tool-centric control.

$$
J_{\text{LOCAL}}(q) \dot{q} = \begin{bmatrix}
\mathbf{v}_{\text{ee}}^{\text{local}} \\
\boldsymbol{\omega}^{\text{local}}
\end{bmatrix}
$$

- **Top 3 rows:** $\mathbf{v}_{\text{ee}}^{\text{local}}$ - velocity of actual EE point in EE body frame
- **Bottom 3 rows:** $\boldsymbol{\omega}^{\text{local}}$ - angular velocity in EE body frame

---

## When to Use Each Frame

| Frame | Use Cases | Example |
|-------|-----------|---------|
| **WORLD** | Dynamics algorithms (RNEA, CRBA), operational space control ($\Lambda = (J(q) M^{-1} J(q)^T)^{-1}$), dynamic nullspace projection, spatial algebra, contact dynamics | MANDATORY when computing with $M$ or $M^{-1}$ |
| **LOCAL_WORLD_ALIGNED** | Kinematic impedance control, Cartesian velocity control (no dynamics), differential IK, trajectory planning, visualization, grasping, assembly, compliant manipulation, teleoperation, most manipulation tasks | "Move gripper to [x,y,z] in the room", most practical robot control |
| **LOCAL** | Tool-axis-relative control | "Push forward along tool axis regardless of orientation" |

**Key insight:** When joint 1 is at the world origin, its WORLD Jacobian column has zero linear terms (pivot point), but LOCAL_WORLD_ALIGNED has non-zero terms (measuring at distant EE). Both are correct - same motion, different measurement points.

**Critical distinction for control:**

- **Use WORLD when computing with dynamics** ($M$, $M^{-1}$, $\Lambda$, dynamic nullspace): Maintains consistency with spatial inertias computed at world origin
- **Use LOCAL_WORLD_ALIGNED for pure kinematics** (no mass matrix): More intuitive, measures actual EE velocity, standard for nearly all practical robot control
- **Use LOCAL very rarely**: Only when you specifically need tool-axis-relative commands

---

## Common Questions

**Q: Which frame matches textbook $\frac{\partial x}{\partial q}$ derivations?**
A: `LOCAL_WORLD_ALIGNED`

**Q: Which frame for inverse kinematics (no dynamics)?**
A: `LOCAL_WORLD_ALIGNED`

**Q: Which frame for operational space control?**
A: `WORLD` - maintains consistency with how $M$ is computed from spatial inertias at world origin

**Q: Which frame for kinematic impedance control ($J(q)^T F$, no mass matrix)?**
A: Almost always `LOCAL_WORLD_ALIGNED` - this is the standard for practical robot control where commands are given in world coordinates.

**Q: When should I use LOCAL frame?**
A: Rarely. Only for specialized tool-axis-relative control where you want commands like "push forward along tool axis" to be independent of the tool's world orientation. Most practical applications (grasping, assembly, compliant manipulation, teleoperation) use LOCAL_WORLD_ALIGNED.

**Q: Why is LOCAL_WORLD_ALIGNED preferred for most tasks?**
A: Because:
- Commands are typically given in world coordinates ("move down", "go to [x,y,z]")
- Easier to plan trajectories and reason about motion
- More intuitive for operators and debugging
- Matches how most robot programming frameworks work
- Works seamlessly with world-frame perception (cameras, force sensors)

**Q: Why use WORLD if LOCAL_WORLD_ALIGNED is more intuitive?**
A: WORLD is required when you compute operational space mass matrix $\Lambda = (J(q) M^{-1} J(q)^T)^{-1}$ or dynamic nullspace projection. It pairs correctly with spatial inertias for dynamics computations (see detailed document). For pure kinematics without dynamics, LOCAL_WORLD_ALIGNED is preferred.

**Q: Are these mathematically equivalent?**
A: Yes! All describe the exact same motion, just from different viewpoints (different measurement points/axes).
