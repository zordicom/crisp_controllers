# CRISP Controller Refactoring Guide

This document outlines the refactoring changes to improve the CRISP controller codebase by reducing technical debt and improving maintainability.

## Overview of Issues Identified

1. **CSV logging is intrusive** - ~250 lines mixed with control logic
2. **Excessive debug logging** - Clutters the update() method
3. **Commented-out code** - Should rely on version control
4. **Magic numbers** - Hardcoded values without named constants
5. **Complex update() method** - Doing too much in one function
6. **Mixed torque components** - Individual member variables instead of structured data
7. **TODO comments** - Unaddressed technical debt

## Refactoring Branch 1: Extract CSV Logging

### Purpose
Separate CSV logging from the main controller logic for better separation of concerns.

### New Files to Create

#### 1. `include/crisp_controllers/utils/csv_logger.hpp`
```cpp
#pragma once

#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace crisp_controllers {

struct ControllerLogData {
  double timestamp;

  // Task space forces (6 DOF each)
  Eigen::Vector<double, 6> task_force_P;
  Eigen::Vector<double, 6> task_force_D;
  Eigen::Vector<double, 6> task_force_total;

  // Torque components (per joint)
  Eigen::VectorXd tau_task;
  Eigen::VectorXd tau_nullspace;
  Eigen::VectorXd tau_joint_limits;
  Eigen::VectorXd tau_friction;
  Eigen::VectorXd tau_coriolis;
  Eigen::VectorXd tau_gravity;
  Eigen::VectorXd tau_wrench;
  Eigen::VectorXd tau_total;

  // Error metrics
  Eigen::Vector<double, 6> error;
  double error_rot_magnitude;
  double error_pos_magnitude;

  // Poses
  pinocchio::SE3 current_pose;
  pinocchio::SE3 target_pose;

  // Stiffness and damping (diagonal elements)
  Eigen::Vector<double, 6> stiffness_diag;
  Eigen::Vector<double, 6> damping_diag;

  // Joint states
  Eigen::VectorXd q_raw;
  Eigen::VectorXd q_filtered;
  Eigen::VectorXd dq_raw;
  Eigen::VectorXd dq_filtered;
  Eigen::VectorXd q_goal;
  Eigen::VectorXd dq_goal;

  // Filter parameters
  double filter_q;
  double filter_dq;
  double filter_output_torque;
};

class ControllerCSVLogger {
public:
  ControllerCSVLogger(const std::string& controller_name, rclcpp::Logger logger);
  ~ControllerCSVLogger();

  bool initialize(size_t num_joints, const rclcpp::Time& start_time);
  void logData(const ControllerLogData& data, const rclcpp::Time& current_time);
  void close();
  bool isLoggingEnabled() const { return logging_enabled_ && csv_file_.is_open(); }

private:
  void writeHeader(size_t num_joints);
  void writePose(const pinocchio::SE3& pose);
  void writeRPY(const pinocchio::SE3& pose);

  std::string controller_name_;
  rclcpp::Logger logger_;
  std::ofstream csv_file_;
  bool logging_enabled_ = false;
  rclcpp::Time start_time_;
  size_t flush_counter_ = 0;
  size_t num_joints_ = 0;

  static constexpr size_t FLUSH_INTERVAL = 50;
};

} // namespace crisp_controllers
```

#### 2. `src/utils/csv_logger.cpp`
Create the implementation file with all CSV writing logic moved from cartesian_controller.cpp (see full implementation in previous messages).

### Changes to Existing Files

#### `include/crisp_controllers/cartesian_controller.hpp`

**Remove:**
```cpp
#include <fstream>
#include <filesystem>

/** @brief CSV log file stream for controller diagnostics */
std::ofstream csv_log_file_;
/** @brief Flag to track if CSV logging is enabled */
bool csv_logging_enabled_ = false;
/** @brief Start time for CSV logging (to compute relative timestamps) */
rclcpp::Time csv_log_start_time_;
/** @brief Counter to track cycles for periodic CSV flushing */
size_t csv_flush_counter_ = 0;
```

**Add:**
```cpp
#include <memory>

// Forward declaration
class ControllerCSVLogger;

/** @brief CSV logger instance for controller diagnostics */
std::unique_ptr<ControllerCSVLogger> csv_logger_;
```

#### `src/cartesian_controller.cpp`

**Add include:**
```cpp
#include "crisp_controllers/utils/csv_logger.hpp"
```

**Replace in `on_activate()` (lines 947-1055):**
```cpp
// Old: 100+ lines of CSV header initialization
// New:
if (params_.log.enabled) {
  csv_logger_ = std::make_unique<ControllerCSVLogger>(
      get_node()->get_name(), get_node()->get_logger());

  if (!csv_logger_->initialize(num_joints, get_node()->now())) {
    return CallbackReturn::ERROR;
  }
}
```

**Replace in `on_deactivate()` (lines 971-977):**
```cpp
// Old: Manual file closing
// New:
if (csv_logger_) {
  csv_logger_->close();
  csv_logger_.reset();
}
```

**Replace in `log_debug_info()` (lines 1142-1266):**
```cpp
// Old: 125 lines of CSV writing
// New:
if (csv_logger_ && csv_logger_->isLoggingEnabled()) {
  ControllerLogData log_data;

  // Populate log data structure
  log_data.timestamp = (time - get_node()->now()).seconds();
  log_data.task_force_P = task_force_P_;
  log_data.task_force_D = task_force_D_;
  log_data.task_force_total = task_force_total_;

  log_data.tau_task = tau_task;
  log_data.tau_nullspace = tau_nullspace;
  log_data.tau_joint_limits = tau_joint_limits;
  log_data.tau_friction = tau_friction;
  log_data.tau_coriolis = tau_coriolis;
  log_data.tau_gravity = tau_gravity;
  log_data.tau_wrench = tau_wrench;
  log_data.tau_total = tau_d;

  log_data.error = error;
  log_data.error_rot_magnitude = error.tail(3).norm();
  log_data.error_pos_magnitude = error.head(3).norm();

  log_data.current_pose = end_effector_pose;
  log_data.target_pose = target_pose_;

  for (int i = 0; i < 6; ++i) {
    log_data.stiffness_diag[i] = stiffness(i, i);
    log_data.damping_diag[i] = damping(i, i);
  }

  log_data.q_raw = q_raw;
  log_data.q_filtered = q;
  log_data.dq_raw = dq_raw;
  log_data.dq_filtered = dq;
  log_data.q_goal = q_goal;
  log_data.dq_goal = dq_goal;

  log_data.filter_q = params_.filter.q;
  log_data.filter_dq = params_.filter.dq;
  log_data.filter_output_torque = params_.filter.output_torque;

  csv_logger_->logData(log_data, time);
}
```

#### `CMakeLists.txt`

**Add to library sources:**
```cmake
add_library(
    ${PROJECT_NAME}
    SHARED
    src/cartesian_controller.cpp
    src/pose_broadcaster.cpp
    src/torque_feedback_controller.cpp
    src/twist_broadcaster.cpp
    src/utils/csv_logger.cpp  # Add this line
)
```

## Refactoring Branch 2: Remove Debug Logging

### Purpose
Remove excessive debug logging that clutters the code and impacts readability.

### Changes to `src/cartesian_controller.cpp`

**Remove lines 68-73:**
```cpp
// Log first update to compare with on_activate values
static bool first_update = true;
if (first_update) {
  RCLCPP_INFO(get_node()->get_logger(),
              "First update() call - checking joint positions...");
}
```

**Remove lines 101-111:**
```cpp
// Log velocity filtering for first joint to verify it's working
if (i == 0) {
  static int vel_log_counter = 0;
  if (vel_log_counter++ % 100 == 0) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Joint0: q_raw=%.4f q_filt=%.4f dq_raw=%.4f dq_filt=%.4f "
                "alpha_dq=%.3f",
                q_raw[i], q[i], dq_raw[i], dq[i], params_.filter.dq);
  }
}
```

**Remove lines 124-141:**
```cpp
// Log first update to verify state interface data
if (first_update) {
  RCLCPP_INFO(get_node()->get_logger(),
              "First update: Joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, "
              "%.3f, %.3f]",
              q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
  // ... more logging
  first_update = false;
}
```

**Remove lines 165-172:**
```cpp
// Log error every 100 cycles (~1Hz at 100Hz update rate)
static int log_counter = 0;
if (log_counter++ % 100 == 0) {
  RCLCPP_INFO(
      get_node()->get_logger(),
      "Cartesian error: pos=[%.4f, %.4f, %.4f]m, ori=[%.4f, %.4f, %.4f]rad",
      error[0], error[1], error[2], error[3], error[4], error[5]);
}
```

**Remove lines 179-186:**
```cpp
// Log damping force computation to verify filtering effect
static int damping_log_counter = 0;
if (damping_log_counter++ % 100 == 0) {
  RCLCPP_INFO(get_node()->get_logger(),
              "Damping: task_vel_x=%.4f, d_pos=%.2f, force_D_x=%.4f (P=%.4f)",
              task_velocity[0], damping(0, 0), task_force_D_[0],
              task_force_P_[0]);
}
```

## Refactoring Branch 3: Remove Commented Code

### Purpose
Remove all commented-out code and rely on version control for history.

### Lines to Remove

**In `src/cartesian_controller.cpp`:**
- Line 136-137: `/*target_pose_ = pinocchio::SE3(...*/`
- Line 1107: `/*filterJointValues(msg->name, msg->position, params_.joints, q_ref);*/`
- Line 1113: `/*filterJointValues(msg->name, msg->velocity, params_.joints, dq_ref);*/`
- Lines 1136-1139: `/*RCLCPP_INFO_STREAM_THROTTLE...*/`
- Lines 1218-1220: `/*RCLCPP_INFO_STREAM_THROTTLE...*/`

## Refactoring Branch 4: Replace Magic Numbers

### Purpose
Replace hardcoded values with named constants.

### Add to `cartesian_controller.hpp`:
```cpp
private:
  // Logging constants
  static constexpr int LOG_CYCLE_INTERVAL = 100;  // Log every 100 cycles
  static constexpr int CSV_FLUSH_INTERVAL = 50;   // Flush CSV every 50 cycles
  static constexpr int DEBUG_LOG_THROTTLE_MS = 1000;  // Throttle debug logs to 1Hz
  static constexpr int TIMING_LOG_THROTTLE_MS = 2000; // Throttle timing logs
```

### Replace in code:
- Line 104: `if (vel_log_counter++ % 100 == 0)` → `if (vel_log_counter++ % LOG_CYCLE_INTERVAL == 0)`
- Line 167: `if (log_counter++ % 100 == 0)` → `if (log_counter++ % LOG_CYCLE_INTERVAL == 0)`
- Line 1262: `if (csv_flush_counter_ >= 50)` → `if (csv_flush_counter_ >= CSV_FLUSH_INTERVAL)`
- All `1000` in throttle macros → `DEBUG_LOG_THROTTLE_MS`
- Line 1233: `2000` → `TIMING_LOG_THROTTLE_MS`

## Refactoring Branch 5: Consolidate Torque Components

### Purpose
Group related torque components into a structured data type.

### Add to `cartesian_controller.hpp`:
```cpp
struct TorqueComponents {
  Eigen::VectorXd task;
  Eigen::VectorXd nullspace;
  Eigen::VectorXd friction;
  Eigen::VectorXd coriolis;
  Eigen::VectorXd gravity;
  Eigen::VectorXd wrench;
  Eigen::VectorXd joint_limits;
  Eigen::VectorXd secondary;  // Before nullspace projection

  Eigen::VectorXd computeTotal() const {
    return task + nullspace + friction + coriolis +
           gravity + wrench + joint_limits;
  }

  void initialize(size_t size) {
    task = Eigen::VectorXd::Zero(size);
    nullspace = Eigen::VectorXd::Zero(size);
    friction = Eigen::VectorXd::Zero(size);
    coriolis = Eigen::VectorXd::Zero(size);
    gravity = Eigen::VectorXd::Zero(size);
    wrench = Eigen::VectorXd::Zero(size);
    joint_limits = Eigen::VectorXd::Zero(size);
    secondary = Eigen::VectorXd::Zero(size);
  }
};
```

### Replace individual torque members:
```cpp
// Old:
Eigen::VectorXd tau_task;
Eigen::VectorXd tau_joint_limits;
Eigen::VectorXd tau_secondary;
Eigen::VectorXd tau_nullspace;
Eigen::VectorXd tau_friction;
Eigen::VectorXd tau_coriolis;
Eigen::VectorXd tau_gravity;
Eigen::VectorXd tau_wrench;

// New:
TorqueComponents torques_;
```

### Update references throughout:
- `tau_task` → `torques_.task`
- `tau_nullspace` → `torques_.nullspace`
- etc.

## Refactoring Branch 6: Break Up update() Method

### Purpose
Split the monolithic update() method into smaller, focused functions.

### New private methods to add:
```cpp
private:
  void readAndFilterJointStates();
  void updateTargets();
  void computeKinematics();
  void computeTaskSpaceError();
  void computeJacobian();
  void computeNullspaceProjection();
  void computeTaskForces();
  void computeTorqueComponents();
  void applyTorqueLimits();
  void writeCommands();
```

### Refactored update() method:
```cpp
controller_interface::return_type
CartesianController::update(const rclcpp::Time &time,
                            const rclcpp::Duration & /*period*/) {
  readAndFilterJointStates();
  updateTargets();
  computeKinematics();
  computeTaskSpaceError();
  computeJacobian();
  computeNullspaceProjection();
  computeTaskForces();
  computeTorqueComponents();
  applyTorqueLimits();
  writeCommands();

  if (params_.log.enabled) {
    log_debug_info(time);
  }

  return controller_interface::return_type::OK;
}
```

## Implementation Order

1. **Branch 1: Extract CSV Logging** (High Priority)
   - Biggest impact on code reduction
   - Clear separation of concerns
   - ~250 lines removed from main controller

2. **Branch 2: Remove Debug Logging** (Medium Priority)
   - Quick win for readability
   - ~40 lines removed

3. **Branch 3: Remove Commented Code** (Easy)
   - Simple cleanup
   - ~10 lines removed

4. **Branch 4: Replace Magic Numbers** (Easy)
   - Improves maintainability
   - No line count change

5. **Branch 5: Consolidate Torque Components** (Medium)
   - Better data organization
   - Cleaner interfaces

6. **Branch 6: Break Up update() Method** (Complex)
   - Most complex refactoring
   - Significantly improves maintainability
   - Makes testing easier

## Testing After Each Refactoring

After each branch:
1. Build the package: `colcon build --packages-select crisp_controllers`
2. Run existing tests if available
3. Test on hardware/simulation to ensure functionality unchanged
4. Verify CSV logging still works (for Branch 1)
5. Check that control performance is unchanged

## Git Workflow

```bash
# Start from main branch
git checkout main

# For each refactoring:
git checkout -b refactor/extract-csv-logging
# Make changes
git add -A
git commit -m "refactor: Extract CSV logging into separate class"

# Create PR and merge
git checkout main
git pull

# Next refactoring
git checkout -b refactor/remove-debug-logging
# etc...
```

## Benefits Summary

- **Code Reduction**: ~300 lines removed
- **Improved Separation**: Control logic separated from diagnostics
- **Better Maintainability**: Smaller, focused functions
- **Cleaner Interfaces**: Structured data instead of scattered variables
- **Professional Code**: No magic numbers or commented code
- **Easier Testing**: Modular design enables unit testing

## Notes

- Each refactoring is independent and can be applied separately
- The CSV logger can be reused for other controllers
- Consider making the debug logger configurable via ROS parameters
- The TorqueComponents struct could be moved to a common header for reuse