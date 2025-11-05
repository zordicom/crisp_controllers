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

  // Timing information
  double loop_duration_ms;  // Control loop duration in milliseconds
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