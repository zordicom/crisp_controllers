#include "crisp_controllers/utils/csv_logger.hpp"
#include <iostream>
#include <iomanip>
#include <pinocchio/math/rpy.hpp>

namespace crisp_controllers {

ControllerCSVLogger::ControllerCSVLogger(const std::string& controller_name,
                                         rclcpp::Logger logger)
    : controller_name_(controller_name), logger_(logger) {}

ControllerCSVLogger::~ControllerCSVLogger() {
  close();
}

bool ControllerCSVLogger::initialize(size_t num_joints, const rclcpp::Time& start_time) {
  num_joints_ = num_joints;
  start_time_ = start_time;

  // Create log file directory
  std::filesystem::path log_dir = "/tmp/controller_logs";
  std::filesystem::create_directories(log_dir);

  // Generate filename with timestamp
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream filename_stream;
  filename_stream << log_dir.string() << "/" << controller_name_
                  << "_log_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
                  << ".csv";

  std::string log_filename = filename_stream.str();
  csv_file_.open(log_filename, std::ios::out);

  if (csv_file_.is_open()) {
    logging_enabled_ = true;
    RCLCPP_INFO(logger_, "CSV logging enabled, writing to: %s", log_filename.c_str());
    writeHeader(num_joints);
    return true;
  } else {
    RCLCPP_ERROR(logger_, "Failed to open CSV log file: %s", log_filename.c_str());
    logging_enabled_ = false;
    return false;
  }
}

void ControllerCSVLogger::close() {
  if (csv_file_.is_open()) {
    csv_file_.flush();
    csv_file_.close();
    RCLCPP_INFO(logger_, "CSV log file closed");
  }
  logging_enabled_ = false;
}

void ControllerCSVLogger::writeHeader(size_t num_joints) {
  // Write CSV header
  csv_file_ << "timestamp";

  // Task space forces header
  csv_file_ << ",task_force_P_x,task_force_P_y,task_force_P_z,task_"
               "force_P_rx,task_force_P_ry,task_force_P_rz";
  csv_file_ << ",task_force_D_x,task_force_D_y,task_force_D_z,task_"
               "force_D_rx,task_force_D_ry,task_force_D_rz";
  csv_file_ << ",task_force_total_x,task_force_total_y,task_force_total_z,"
               "task_force_total_rx,task_force_total_ry,task_force_total_rz";

  // Torque components header (per joint)
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",tau_task_" << i << ",tau_nullspace_" << i
              << ",tau_joint_limits_" << i << ",tau_friction_" << i
              << ",tau_coriolis_" << i << ",tau_gravity_" << i
              << ",tau_wrench_" << i << ",tau_total_" << i;
  }

  // Error metrics header
  csv_file_ << ",error_x,error_y,error_z,error_rx,error_ry,error_rz";
  csv_file_ << ",error_rot_magnitude,error_pos_magnitude";

  // Pose headers
  csv_file_ << ",current_x,current_y,current_z,current_qw,current_qx,"
               "current_qy,current_qz";
  csv_file_ << ",current_roll,current_pitch,current_yaw";
  csv_file_ << ",target_x,target_y,target_z,target_qw,target_qx,target_"
               "qy,target_qz";
  csv_file_ << ",target_roll,target_pitch,target_yaw";

  // Stiffness and damping headers
  csv_file_ << ",k_pos_x,k_pos_y,k_pos_z,k_rot_x,k_rot_y,k_rot_z";
  csv_file_ << ",d_pos_x,d_pos_y,d_pos_z,d_rot_x,d_rot_y,d_rot_z";

  // Joint states headers
  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",q_raw_" << i;
  }

  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",q_filtered_" << i;
  }

  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",dq_raw_" << i;
  }

  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",dq_filtered_" << i;
  }

  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",q_goal_" << i;
  }

  for (size_t i = 0; i < num_joints; ++i) {
    csv_file_ << ",dq_goal_" << i;
  }

  // Filter parameters header
  csv_file_ << ",filter_q,filter_dq,filter_output_torque";

  csv_file_ << std::endl;
}

void ControllerCSVLogger::writePose(const pinocchio::SE3& pose) {
  const auto& trans = pose.translation();
  const auto& quat = Eigen::Quaterniond(pose.rotation());

  csv_file_ << "," << trans[0] << "," << trans[1] << "," << trans[2];
  csv_file_ << "," << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z();
}

void ControllerCSVLogger::writeRPY(const pinocchio::SE3& pose) {
  auto rpy = pinocchio::rpy::matrixToRpy(pose.rotation());
  csv_file_ << "," << rpy[0] << "," << rpy[1] << "," << rpy[2];
}

void ControllerCSVLogger::logData(const ControllerLogData& data, const rclcpp::Time& current_time) {
  if (!logging_enabled_ || !csv_file_.is_open()) {
    return;
  }

  // Compute timestamp relative to start
  double timestamp = (current_time - start_time_).seconds();
  csv_file_ << timestamp;

  // Write task space forces
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.task_force_P[i];
  }
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.task_force_D[i];
  }
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.task_force_total[i];
  }

  // Write torque components
  for (int i = 0; i < data.tau_task.size(); ++i) {
    csv_file_ << "," << data.tau_task[i]
              << "," << data.tau_nullspace[i]
              << "," << data.tau_joint_limits[i]
              << "," << data.tau_friction[i]
              << "," << data.tau_coriolis[i]
              << "," << data.tau_gravity[i]
              << "," << data.tau_wrench[i]
              << "," << data.tau_total[i];
  }

  // Write error metrics
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.error[i];
  }
  csv_file_ << "," << data.error_rot_magnitude << "," << data.error_pos_magnitude;

  // Write poses
  writePose(data.current_pose);
  writeRPY(data.current_pose);
  writePose(data.target_pose);
  writeRPY(data.target_pose);

  // Write stiffness and damping
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.stiffness_diag[i];
  }
  for (int i = 0; i < 6; ++i) {
    csv_file_ << "," << data.damping_diag[i];
  }

  // Write joint states
  for (int i = 0; i < data.q_raw.size(); ++i) {
    csv_file_ << "," << data.q_raw[i];
  }
  for (int i = 0; i < data.q_filtered.size(); ++i) {
    csv_file_ << "," << data.q_filtered[i];
  }
  for (int i = 0; i < data.dq_raw.size(); ++i) {
    csv_file_ << "," << data.dq_raw[i];
  }
  for (int i = 0; i < data.dq_filtered.size(); ++i) {
    csv_file_ << "," << data.dq_filtered[i];
  }
  for (int i = 0; i < data.q_goal.size(); ++i) {
    csv_file_ << "," << data.q_goal[i];
  }
  for (int i = 0; i < data.dq_goal.size(); ++i) {
    csv_file_ << "," << data.dq_goal[i];
  }

  // Write filter parameters
  csv_file_ << "," << data.filter_q << "," << data.filter_dq << "," << data.filter_output_torque;

  csv_file_ << std::endl;

  // Periodic flush
  flush_counter_++;
  if (flush_counter_ >= FLUSH_INTERVAL) {
    csv_file_.flush();
    flush_counter_ = 0;
  }
}

} // namespace crisp_controllers