# Reference Frame Usage: Recommendation & Assessment

## 🎯 RECOMMENDATION (TL;DR)

**For crisp_controllers CartesianController, use `LOCAL_WORLD_ALIGNED`. For force/velocity telemetry, LOCAL frame is actually appropriate.**

### Quick Fix Summary

| Controller | Current | Change To | Reason |
|------------|---------|-----------|---------|
| **CartesianController** | WORLD/LOCAL + Adjoint transform | `LOCAL_WORLD_ALIGNED` (no transform) | Defaults to kinematic impedance (no dynamics) |
| **TorqueFeedbackController** | LOCAL | **Keep LOCAL** (or make configurable) | Tool-frame wrench is intuitive for force feedback |
| **TwistBroadcaster** | LOCAL (implicit) | **Keep LOCAL** (make explicit) or make configurable | Depends on use case: teleoperation vs monitoring |

### Implementation

**CartesianController** (position control - use LOCAL_WORLD_ALIGNED):
```cpp
// Use LOCAL_WORLD_ALIGNED for position control
pinocchio::computeFrameJacobian(model_, data_, q_pin, ee_frame_id_,
                                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

// Remove ALL Adjoint transforms (lines 198-210)
// No more: J = Ad * J transformations!

// Compute error in WORLD frame (matches target commands)
pinocchio::SE3 ee_pose_world = data_.oMf[ee_frame_id_];
error.head(3) = target_pose_world_.translation() - ee_pose_world.translation();
error.tail(3) = pinocchio::log3(target_pose_world_.rotation() *
                                ee_pose_world.rotation().transpose());
```
**TwistBroadcaster** (velocity telemetry - keep LOCAL, make explicit):
```cpp
// Make frame explicit (currently defaults to LOCAL)
auto current_velocity = pinocchio::getFrameVelocity(
    model_, data_, end_effector_frame_id,
    pinocchio::ReferenceFrame::LOCAL  // Explicit for clarity
);

twist_msg.header.frame_id = params_.end_effector_frame;  // Tool-relative velocity
```

### Why This Works

**Key insight 1**: CartesianController defaults to **kinematic impedance control** (no dynamics):
- `use_operational_space = False`, `nullspace.projector_type = "kinematic"` (default) →**No $M^{-1}$ computation** 
- → No need for WORLD frame!
- Pure kinematics: $\boldsymbol{\tau} = J^T F$ where $F = K_s (x_{\text{target}} - x) - K_d \dot{x}$ (stiffness $K_s$, damping $K_d$, no mass matrix $\Lambda$) 
- → Use LOCAL_WORLD_ALIGNED for position control


### Exception: If You Enable Operational Space Control

**Only if you set `use_operational_space: true` or `projector_type: "dynamic"`:**

```cpp
// Use mode-dependent frame selection
pinocchio::ReferenceFrame ref_frame;
if (params_.use_operational_space || params_.nullspace.projector_type == "dynamic") {
  ref_frame = pinocchio::ReferenceFrame::WORLD;  // Required for dynamics
} else {
  ref_frame = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;  // Default case
}
pinocchio::computeFrameJacobian(model_, data_, q_pin, ee_frame_id_, ref_frame, J);
```

But since **you don't use operational space mode**, the main change is CartesianController.


---

## Why This Works (Mathematical Justification)

### For Kinematic Impedance Control (Current Use Case)

Control law: $\boldsymbol{\tau} = J^T F$

Where $F = K_s (x_{\text{target}} - x) - K_d \dot{x}$ (stiffness $K_s$, damping $K_d$)

**Key insight**: This is a coordinate transformation, frame-independent in the sense that:
- Error is in world frame
- $J^T$ maps from world frame to joint space
- Result is joint torques (frame-independent)

**Which Jacobian to use?**
- **LOCAL_WORLD_ALIGNED**: Maps joint velocities → velocity at actual EE in world axes
- WORLD: Maps joint velocities → velocity at imaginary point at world origin (confusing!)
- LOCAL: Maps joint velocities → velocity in tool frame (unintuitive for world commands)
