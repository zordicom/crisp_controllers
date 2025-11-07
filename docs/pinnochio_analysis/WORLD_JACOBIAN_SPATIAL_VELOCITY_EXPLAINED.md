# WORLD Frame Jacobian: A Clear Mathematical Explanation

This document explains what the "WORLD" Jacobian represents using classical rigid-body kinematics.

---

## 1. The Key Insight First

When you compute:

```python
J_world = pin.computeFrameJacobian(model, data, q, END_EFFECTOR_ID, pin.WORLD)
```

You get the end-effector Jacobian, and this equation holds:

$$
\mathcal{V}_{\text{ee}} = J_{\text{WORLD}} \, \dot{q}
$$

**BUT:** The linear velocity component represents the velocity of an imaginary point rigidly attached to the end-effector body but located at the world origin, not the velocity of the actual end-effector point itself.

---

## 2. What is a Spatial Velocity (aka Twist)?

A **spatial velocity** (or twist) describes the motion of a rigid body. For the end-effector:

$$
\mathcal{V}_{\text{ee}} = \begin{bmatrix} \boldsymbol{\omega} \\ \mathbf{v} \end{bmatrix}
$$

where:

- $\boldsymbol{\omega}$ = angular velocity (same everywhere on the rigid body)
- $\mathbf{v}$ = linear velocity, but **at which point?**

The key: **$\mathbf{v}$ depends on which point you choose as the reference!**

### Same Motion, Different Measurement Points

The end-effector is spinning and moving. You can describe this same motion with different reference points:

**Measured at world origin (0,0,0):**


$$
\mathcal{V}_{\text{WORLD}} = \begin{bmatrix} \mathbf{v}_{\text{origin}} \\ \boldsymbol{\omega} \end{bmatrix} = J_{\text{WORLD}}(q) \, \dot{q}
$$


- **Top 3 rows**: $\mathbf{v}_{\text{origin}}$ = linear velocity of imaginary point at world origin
- **Bottom 3 rows**: $\boldsymbol{\omega}$ = angular velocity of the end-effector body

**Measured at end-effector point:**


$$
\mathcal{V}_{\text{ALIGNED}} = \begin{bmatrix} \mathbf{v}_{\text{ee point}} \\ \boldsymbol{\omega} \end{bmatrix} = J_{\text{ALIGNED}}(q) \, \dot{q}
$$

- **Top 3 rows**: $\mathbf{v}_{\text{ee}}$ = linear velocity of actual end-effector point
- **Bottom 3 rows**: $\boldsymbol{\omega}$ = angular velocity of the end-effector body

The linear velocities are related by the **velocity transport formula**:

$$
\mathbf{v}_{\text{ee point}} = \mathbf{v}_{\text{origin}} + \boldsymbol{\omega} \times \mathbf{r}_{\text{ee}}
$$

where $\mathbf{r}_{\text{ee}}$ is the position vector from the world origin to the end-effector point.

### Connecting Spatial Velocity to the Jacobian

The Jacobian relates joint velocities to spatial velocity:

$$
\mathcal{V}_{\text{ee}} = J(q) \, \dot{q}
$$

Both Jacobians are $(6 \times n)$ matrices where $n$ is the number of joints:

$$
J(q) = \begin{bmatrix}
\text{col}_1 & \text{col}_2 & \cdots & \text{col}_n
\end{bmatrix}
$$

Each column $j$ is the partial derivative:  $\text{col}_j = \frac{\partial \mathcal{V}_{\text{ee}}}{\partial \dot{q}_j}$ , meaning "when only joint $j$ moves at 1 rad/s, this is the resulting spatial velocity (of an imaginary point attached to the ee going through the origin for $\mathcal{V}_\text{world}$ , or the ee itself for $\mathcal{V}_\text{aligned}$ )."

### Concrete Example (2-link planar robot)

Consider a 2-link planar robot with 0.5 m links at configuration $q = [0, 0]$ (straight up). End-effector position: $\mathbf{r}_{\text{ee}} = [0, 1.0, 0]$, Joint 2 at: $\mathbf{r}_{J2} = [0, 0.5, 0]$.

**Key insight:** When computing $\mathbf{v}_{\text{origin}}$ (WORLD frame), imagine the end-effector rigid body extended back to the origin. When a joint rotates:

- If the joint is AT the origin (J1), the imaginary point at the origin is at the pivot → $\mathbf{v}_{\text{origin}} = 0$
- If the joint is elsewhere (J2), the imaginary point swings around with the body → $\mathbf{v}_{\text{origin}} \neq 0$

**For 1 rad/s rotation:**

| Motion | $\boldsymbol{\omega}$ | $\mathbf{v}_{\text{origin}}$ | $\mathbf{v}_{\text{ee}}$ |
|--------|---------|-----------------|----------|
| J1 only | $[0,0,1]$ | $[0,0,0]$ | $[1.0, 0, 0]$ |
| J2 only | $[0,0,1]$ | $[-0.5, 0, 0]$ | $[0.5, 0, 0]$ |

Using the velocity transport formula $\mathbf{v}_{\text{ee}} = \mathbf{v}_{\text{origin}} + \boldsymbol{\omega} \times \mathbf{r}_{\text{ee}}$ , we verify: for J2, $[-0.5,0,0] + [0,0,1] \times [0,1.0,0] = [0.5, 0, 0]$ ✓

**Resulting Jacobians at $q = [0, 0]$:**

$$
J_{\text{WORLD}} = \begin{bmatrix}
0 & -0.5 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
1 & 1
\end{bmatrix}, \quad
J_{\text{ALIGNED}} = \begin{bmatrix}
1.0 & 0.5 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
0 & 0 \\
1 & 1
\end{bmatrix}
$$

**Key observation:** $J_{\text{WORLD}}$ has zero linear terms in column 1 (origin is the pivot), while $J_{\text{ALIGNED}}$ has non-zero terms (measuring at distant EE point).

---

## 3. How Columns Are Computed

### For WORLD Frame

For a **revolute joint j** at position $\mathbf{p}_j(q)$ with rotation axis $\hat{z}_j$ in world coordinates:

$$
J_{\text{WORLD}}(q)[:, j] = \begin{bmatrix}
\mathbf{p}_j(q) \times (R_j(q) \hat{z}_j) \\
R_j(q) \hat{z}_j
\end{bmatrix}
$$

where:

- $R_j(q) \hat{z}_j$ = joint j's rotation axis in world coordinates
- $\mathbf{p}_j(q) \times (R_j(q) \hat{z}_j)$ = linear velocity at world origin created by this rotation
- If $\mathbf{p}_j(q) = \mathbf{0}$ (joint at origin), linear part is zero

#### Geometric Interpretation

When only joint $j$ rotates at unit rate ($\dot{q}_j = 1$ rad/s):

$$
\boxed{J_{\text{WORLD}}[0:3, j] = \mathbf{p}_j(q) \times (R_j(q) \hat{z}_j)},\qquad \boxed{J_{\text{WORLD}}[3:6, j] = R_j(q)\hat{z}_j}
$$

**Physical meaning:**

- **Angular part (bottom 3 rows):** $R_j(q)\hat{z}_j$ is simply the joint's rotation axis in world coordinates
- **Linear part (top 3 rows):** $\mathbf{p}_j(q) \times (R_j(q) \hat{z}_j)$ is the velocity at the world origin created by this rotation
  - If joint $j$ is at the origin ($\mathbf{p}_j = 0$), the cross product is zero → no linear velocity at origin
  - If joint $j$ is elsewhere, the imaginary point at the origin swings around → non-zero linear velocity

### For LOCAL_WORLD_ALIGNED Frame

The derivation for $J_{\text{ALIGNED}}$ differs only in the linear part:

**Linear velocity part:**

For the actual end-effector point, the velocity is the time derivative of position:

$$
\mathbf{v}_{\text{ee}} = \frac{d\mathbf{p}_{\text{ee}}}{dt} = \sum_i \frac{\partial \mathbf{p}_{\text{ee}}}{\partial q_i} \dot{q}_i
$$

Therefore, the Jacobian column $j$ is simply the **geometric Jacobian** (position derivative):

$$
\boxed{J_{\text{ALIGNED}}[:, j] = \frac{\partial \mathbf{p}_{\text{ee}}}{\partial q_j}}
$$

This is the familiar Jacobian from kinematics textbooks: the change in end-effector position per unit change in joint angle!

**Relationship between the two:**

Using the velocity transport formula $\mathbf{v}_{\text{ee}} = \mathbf{v}_{\text{origin}} + \boldsymbol{\omega} \times \mathbf{r}_{\text{ee}}$:

$$
\begin{align}
\frac{\partial \mathbf{p}_{\text{ee}}}{\partial q_j} &= \frac{\partial \mathbf{v}_{\text{origin}}}{\partial \dot{q}_j} + \frac{\partial \boldsymbol{\omega}}{\partial \dot{q}_j} \times \mathbf{r}_{\text{ee}} \\
&= [\mathbf{p}_j(q) \times (R_j(q) \hat{z}_j)] + [(R_j(q) \hat{z}_j) \times \mathbf{p}_{\text{ee}}(q)]
\end{align}
$$

---

## 4. Why WORLD Frame Exists (And When You MUST Use It)

> **For practical "when to use which frame" guidance**, see `REFERENCE_FRAMES_COMPREHENSIVE_GUIDE.md`.

WORLD frame is not an arbitrary choice - it's **essential for dynamics algorithms** because spatial inertias, forces, and velocities must be expressed at a common reference point.

### When WORLD Frame is REQUIRED

**You MUST use WORLD frame Jacobian when:**

1. Computing operational space mass matrix: $\Lambda = (J M^{-1} J^T)^{-1}$
2. Computing dynamic nullspace projection: $N = I - J^T (J^\#)^T$ where $J^\# = M^{-1} J^T \Lambda$ is the dynamically consistent pseudoinverse
3. Any control law that explicitly uses the joint-space mass matrix M or its inverse

**Why?** Because M is computed by CRBA using spatial inertias at the world origin. To maintain frame consistency in the spatial algebra framework, the Jacobian must also use WORLD frame.

### When LOCAL_WORLD_ALIGNED Can Be Used

**You CAN use LOCAL_WORLD_ALIGNED when:**

1. Kinematic impedance control: τ = J^T * F (no mass matrix involved)
2. Kinematic nullspace projection: N = I - J_pinv * J (no dynamics)
3. Differential inverse kinematics: Δq = J_pinv * Δx
4. Velocity control: v_desired = J * q̇
5. Visualization and monitoring

### When LOCAL Frame SHOULD Be Used

**Use LOCAL frame when you want forces/velocities expressed in the tool's own coordinate system:**

**1. Tool-centric force control:**

```python
# Example: "Push forward along tool axis with 10N, regardless of tool orientation"
J_local = pin.computeFrameJacobian(model, data, q, tool_id, pin.LOCAL)
F_tool = [10.0, 0, 0, 0, 0, 0]  # Force along tool's X-axis
tau = J_local.T @ F_tool

# As tool rotates, "forward" always means along tool's local X-axis
# This is intuitive for end-effector-relative control
```

**2. Compliant manipulation (admittance control):**

```python
# Measure forces in tool frame - more intuitive for contact tasks
F_sensed_tool = J_local.T @ tau_sensed  # Force felt by the tool in tool coordinates
# If F_sensed_tool[0] > threshold: "something is pushing on the tool tip"
```

**3. Grasping and assembly:**

```python
# Grasp control: "Close gripper fingers" is defined in gripper's local frame
# Assembly: "Insert peg" requires motion along peg's local axis
```

**4. Teleoperation with tool-relative commands:**

```python
# Operator commands: "move forward/back/left/right" relative to tool
# Useful when camera is mounted on tool - commands match visual perspective
```

**When NOT to use LOCAL:**

- **Don't use for world-relative Cartesian control**: If you want "move +X in the room," use LOCAL_WORLD_ALIGNED
- **Don't use for visualization**: Velocity arrows would rotate with tool (confusing)
- **Don't use for trajectory following in world frame**: Path would be interpreted in constantly rotating frame
- **Don't use with dynamics (M, M^{-1})**: Breaks spatial algebra consistency

**Key distinction:**
- **LOCAL**: "Push the tool forward along its own nose" (tool-centric)
- **LOCAL_WORLD_ALIGNED**: "Move the tool northward in the room" (world-centric but measured at tool point)
- **WORLD**: "Move point at world origin attached to tool" (used for dynamics algorithms)

### Spatial Dynamics Fundamentals

The Newton-Euler equation in spatial form: $\mathcal{F} = \mathcal{I} \, \mathcal{V}$ where $\mathcal{F}$ = spatial force (6×1), $\mathcal{I}$ = spatial inertia (6×6), $\mathcal{V}$ = spatial velocity (6×1).

**RNEA (Recursive Newton-Euler) propagates spatial quantities:**

$$
\mathcal{V}_i = \mathcal{V}_{\text{parent}(i)} + S_i \dot{q}_i, \quad
\tau_i = S_i^T \mathcal{F}_i
$$

All operations require a consistent reference frame - WORLD provides this.

### Operational Space Control - CRITICAL

For operational space control, the **task-space mass matrix** is:

$$
\Lambda = (J M^{-1} J^T)^{-1}
$$

where $M$ is the joint-space mass matrix.

**CRITICAL REQUIREMENT**: You **MUST** use WORLD frame Jacobian here:

```python
M = pin.crba(model, data, q)  # Uses spatial inertias at world origin
J = pin.computeFrameJacobian(model, data, q, ee_id, pin.WORLD)  # ✓ REQUIRED!
Lambda = np.linalg.inv(J @ np.linalg.inv(M) @ J.T)

# Operational space control law
F_task = Lambda @ (K_p * error - K_d * J @ dq)  # Task-space force
tau = J.T @ F_task  # Map to joint torques
```

**Why WORLD frame is mandatory:**

1. M is computed from spatial inertias expressed at world origin
2. The quantity J M^{-1} J^T involves coordinate transformations that assume consistent frames
3. Spatial algebra operations (used internally by CRBA) require all quantities at the same reference point
4. Using `LOCAL_WORLD_ALIGNED` would break frame consistency, potentially causing incorrect dynamics

**Common mistake**: Computing J in LOCAL_WORLD_ALIGNED then using it with M. This may appear to work but gives incorrect task-space inertia!

**Bottom line:** Spatial algebra frameworks (Featherstone, Pinocchio) are designed around WORLD frame for all dynamics computations. While `LOCAL_WORLD_ALIGNED` is intuitive for pure kinematics, `WORLD` is **mandatory** for dynamics algorithms (RNEA, CRBA, operational space control, dynamic nullspace projection).

---

## 5. Textbook References

This formulation is standard in **screw theory** and **spatial algebra**:

### Murray, Li, Sastry (1994)

*A Mathematical Introduction to Robotic Manipulation*, Chapter 3

- Spatial Jacobian defined using twists
- Reference point affects linear part of spatial velocity

Free: <https://www.cse.lehigh.edu/~trink/Courses/RoboticsII/reading/murray-li-sastry-94-complete.pdf>

### Roy Featherstone (2008)

*Rigid Body Dynamics Algorithms*, Chapter 2

- Spatial velocities and reference points
- Foundation for spatial-vector dynamics

### Lynch & Park (2017)

*Modern Robotics*, Chapter 5

- Body vs. spatial Jacobians
- Clear treatment of reference frames

---

## 6. Summary: Practical Decision Tree

**Decision tree for choosing reference frame:**

```text
Are you computing with dynamics (M or M^{-1})?
├── YES → Use WORLD frame (MANDATORY for frame consistency)
│   ├── Operational space control (Λ = (J M^{-1} J^T)^{-1})
│   ├── Dynamic nullspace projection
│   ├── Contact dynamics
│   └── Any algorithm using joint-space mass matrix
│
└── NO (Pure kinematics) → Choose based on control semantics:
    │
    ├── Want tool-relative control? → Use LOCAL frame
    │   ├── Tool-centric force control ("push along tool axis")
    │   ├── Compliant manipulation (measure forces in tool frame)
    │   ├── Grasping and assembly tasks
    │   ├── Teleoperation (camera mounted on tool)
    │   └── Any task where commands are relative to tool orientation
    │
    └── Want world-relative control? → Use LOCAL_WORLD_ALIGNED
        ├── Kinematic impedance (J^T F) in world coordinates
        ├── Kinematic nullspace (I - J_pinv J)
        ├── Differential IK for world-frame trajectories
        ├── Velocity control in world frame
        ├── Visualization and monitoring
        └── Most Cartesian manipulation tasks
```

**Quick reference table:**

| Frame | Control Semantics | When to Use |
|-------|------------------|-------------|
| **WORLD** | Imaginary point at origin | **ONLY** for dynamics algorithms (mandatory) |
| **LOCAL_WORLD_ALIGNED** | Tool position, world axes | Most Cartesian control (world-relative commands) |
| **LOCAL** | Tool position, tool axes | Tool-relative control (e.g., "push tool forward") |

**Key takeaways:**

1. **WORLD is mandatory** when using M or M^{-1} (spatial algebra consistency)
2. **LOCAL vs LOCAL_WORLD_ALIGNED** depends on whether commands are tool-relative or world-relative
3. Most robotic manipulation uses **LOCAL_WORLD_ALIGNED** (world-frame commands)
4. **LOCAL** is for specialized tool-centric applications
