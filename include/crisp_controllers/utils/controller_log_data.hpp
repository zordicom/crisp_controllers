#pragma once

#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>

namespace crisp_controllers {

/**
 * @brief Data structure for logging controller state and diagnostics
 */
struct ControllerLogData {
  double timestamp;

  // Task space forces (6 DOF each)
  Eigen::Vector<double, 6> task_force_P;
  Eigen::Vector<double, 6> task_force_D;
  Eigen::Vector<double, 6> task_force_total;

  // Task space velocity (6 DOF) - J*dq
  Eigen::Vector<double, 6> task_velocity;

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

  // Timing information
  double loop_duration_ms;  // Control loop duration in milliseconds
};

} // namespace crisp_controllers