#pragma once

#include "controller_log_data_interface.hpp"
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/math/rpy.hpp>
#include <sstream>
#include <iomanip>

namespace crisp_controllers {

/**
 * @brief Data structure for logging controller state and diagnostics
 */
class ControllerLogData : public ControllerLogDataInterface {
public:
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

  // MIT controller specific: goal positions and velocities sent to motors
  Eigen::VectorXd q_goal;
  Eigen::VectorXd dq_goal;

  // Filter parameters
  double filter_q;
  double filter_dq;
  double filter_output_torque;

  // Timing information
  double loop_duration_ms;  // Control loop duration in milliseconds

  // Interface implementation
  double getTimestamp() const override { return timestamp; }

  std::string generateHeader() const override {
    std::stringstream ss;

    size_t num_joints = q_raw.size();

    // Basic info
    ss << "timestamp";

    // Current pose (position + quaternion + RPY)
    ss << ",current_x,current_y,current_z";
    ss << ",current_qw,current_qx,current_qy,current_qz";
    ss << ",current_roll,current_pitch,current_yaw";

    // Target pose (position + quaternion + RPY)
    ss << ",target_x,target_y,target_z";
    ss << ",target_qw,target_qx,target_qy,target_qz";
    ss << ",target_roll,target_pitch,target_yaw";

    // Cartesian errors
    ss << ",error_x,error_y,error_z,error_rx,error_ry,error_rz";
    ss << ",error_pos_magnitude,error_rot_magnitude";

    // Task space forces
    ss << ",task_force_P_x,task_force_P_y,task_force_P_z";
    ss << ",task_force_P_rx,task_force_P_ry,task_force_P_rz";
    ss << ",task_force_D_x,task_force_D_y,task_force_D_z";
    ss << ",task_force_D_rx,task_force_D_ry,task_force_D_rz";
    ss << ",task_force_total_x,task_force_total_y,task_force_total_z";
    ss << ",task_force_total_rx,task_force_total_ry,task_force_total_rz";

    // Task velocity
    ss << ",task_vel_x,task_vel_y,task_vel_z";
    ss << ",task_vel_rx,task_vel_ry,task_vel_rz";

    // Joint states
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_raw_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_filtered_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_raw_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_filtered_" << i;
    }

    // Torque components
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_task_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_nullspace_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_joint_limits_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_friction_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_coriolis_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_gravity_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_wrench_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_total_" << i;
    }

    // Impedance parameters
    ss << ",k_pos_x,k_pos_y,k_pos_z,k_rot_x,k_rot_y,k_rot_z";
    ss << ",d_pos_x,d_pos_y,d_pos_z,d_rot_x,d_rot_y,d_rot_z";

    // Filter parameters
    ss << ",filter_q,filter_dq,filter_output_torque";

    // Timing
    ss << ",loop_duration_ms";

    return ss.str();
  }

  std::string generateLine() const override {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);

    // Timestamp
    ss << timestamp;

    // Current pose
    writePose(ss, current_pose);
    writeRPY(ss, current_pose);

    // Target pose
    writePose(ss, target_pose);
    writeRPY(ss, target_pose);

    // Cartesian errors
    for (int i = 0; i < 6; ++i) {
      ss << "," << error[i];
    }
    ss << "," << error_pos_magnitude;
    ss << "," << error_rot_magnitude;

    // Task space forces
    for (int i = 0; i < 6; ++i) {
      ss << "," << task_force_P[i];
    }
    for (int i = 0; i < 6; ++i) {
      ss << "," << task_force_D[i];
    }
    for (int i = 0; i < 6; ++i) {
      ss << "," << task_force_total[i];
    }

    // Task velocity
    for (int i = 0; i < 6; ++i) {
      ss << "," << task_velocity[i];
    }

    // Joint states
    for (int i = 0; i < q_raw.size(); ++i) {
      ss << "," << q_raw[i];
    }
    for (int i = 0; i < q_filtered.size(); ++i) {
      ss << "," << q_filtered[i];
    }
    for (int i = 0; i < dq_raw.size(); ++i) {
      ss << "," << dq_raw[i];
    }
    for (int i = 0; i < dq_filtered.size(); ++i) {
      ss << "," << dq_filtered[i];
    }

    // Torque components
    for (int i = 0; i < tau_task.size(); ++i) {
      ss << "," << tau_task[i];
    }
    for (int i = 0; i < tau_nullspace.size(); ++i) {
      ss << "," << tau_nullspace[i];
    }
    for (int i = 0; i < tau_joint_limits.size(); ++i) {
      ss << "," << tau_joint_limits[i];
    }
    for (int i = 0; i < tau_friction.size(); ++i) {
      ss << "," << tau_friction[i];
    }
    for (int i = 0; i < tau_coriolis.size(); ++i) {
      ss << "," << tau_coriolis[i];
    }
    for (int i = 0; i < tau_gravity.size(); ++i) {
      ss << "," << tau_gravity[i];
    }
    for (int i = 0; i < tau_wrench.size(); ++i) {
      ss << "," << tau_wrench[i];
    }
    for (int i = 0; i < tau_total.size(); ++i) {
      ss << "," << tau_total[i];
    }

    // Impedance parameters
    for (int i = 0; i < 6; ++i) {
      ss << "," << stiffness_diag[i];
    }
    for (int i = 0; i < 6; ++i) {
      ss << "," << damping_diag[i];
    }

    // Filter parameters
    ss << "," << filter_q;
    ss << "," << filter_dq;
    ss << "," << filter_output_torque;

    // Timing
    ss << "," << loop_duration_ms;

    return ss.str();
  }

private:
  void writePose(std::stringstream& ss, const pinocchio::SE3& pose) const {
    const auto& trans = pose.translation();
    const auto& quat = Eigen::Quaterniond(pose.rotation());

    ss << "," << trans[0] << "," << trans[1] << "," << trans[2];
    ss << "," << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z();
  }

  void writeRPY(std::stringstream& ss, const pinocchio::SE3& pose) const {
    auto rpy = pinocchio::rpy::matrixToRpy(pose.rotation());
    ss << "," << rpy[0] << "," << rpy[1] << "," << rpy[2];
  }
};

} // namespace crisp_controllers