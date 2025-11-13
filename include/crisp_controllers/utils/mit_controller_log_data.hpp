#pragma once

#include "controller_log_data_interface.hpp"
#include <Eigen/Dense>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/math/rpy.hpp>
#include <sstream>
#include <iomanip>

namespace crisp_controllers {

/**
 * @brief Log data specific to MIT Cartesian Controller
 *
 * This class logs only the values that the MIT controller actually computes:
 * - Current and target poses
 * - Position and orientation errors
 * - Task space velocity (J*dq)
 * - Joint states (q, dq)
 * - Goal joint states (q_goal, dq_goal) - what we send to motors
 * - Feedforward torques (tau_ff) - primarily gravity compensation
 * - Cartesian stiffness and damping parameters
 * - Control loop timing
 */
class MITControllerLogData : public ControllerLogDataInterface {
public:
  // Timestamp
  double timestamp;

  // Cartesian pose and error
  pinocchio::SE3 current_pose;
  pinocchio::SE3 target_pose;
  Eigen::Vector<double, 6> error;  // [position_error; orientation_error]

  // Joint states (current)
  Eigen::VectorXd q;
  Eigen::VectorXd dq;

  // Joint goals (sent to motors)
  Eigen::VectorXd q_goal;
  Eigen::VectorXd dq_goal;

  // Feedforward torques (sent to motors)
  Eigen::VectorXd tau_ff;

  // Motor PD gains (sent to motors)
  Eigen::VectorXd mot_K_p;
  Eigen::VectorXd mot_K_d;

  // Cartesian impedance parameters
  Eigen::Vector<double, 6> stiffness_diag;
  Eigen::Vector<double, 6> damping_diag;

  // Control parameters
  double alpha;  // Velocity gain

  // Timing
  double loop_duration_ms;

  double getTimestamp() const override { return timestamp; }

  std::string generateHeader() const override {
    std::stringstream ss;

    size_t num_joints = q.size();

    // Basic info
    ss << "timestamp";

    // Current pose (position + quaternion)
    ss << ",current_x,current_y,current_z";
    ss << ",current_qw,current_qx,current_qy,current_qz";
    ss << ",current_roll,current_pitch,current_yaw";

    // Target pose (position + quaternion)
    ss << ",target_x,target_y,target_z";
    ss << ",target_qw,target_qx,target_qy,target_qz";
    ss << ",target_roll,target_pitch,target_yaw";

    // Cartesian errors
    ss << ",error_x,error_y,error_z";
    ss << ",error_rx,error_ry,error_rz";
    ss << ",error_pos_magnitude,error_rot_magnitude";

    // Joint states
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_" << i;
    }

    // Joint goals (sent to motors)
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_goal_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_goal_" << i;
    }

    // Feedforward torques
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",tau_ff_" << i;
    }

    // Motor PD gains
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",mot_K_p_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",mot_K_d_" << i;
    }

    // Cartesian impedance parameters
    ss << ",k_pos_x,k_pos_y,k_pos_z,k_rot_x,k_rot_y,k_rot_z";
    ss << ",d_pos_x,d_pos_y,d_pos_z,d_rot_x,d_rot_y,d_rot_z";

    // Control parameters
    ss << ",alpha";

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
    ss << "," << error.head(3).norm();  // position error magnitude
    ss << "," << error.tail(3).norm();  // rotation error magnitude

    // Joint states
    for (int i = 0; i < q.size(); ++i) {
      ss << "," << q[i];
    }
    for (int i = 0; i < dq.size(); ++i) {
      ss << "," << dq[i];
    }

    // Joint goals
    for (int i = 0; i < q_goal.size(); ++i) {
      ss << "," << q_goal[i];
    }
    for (int i = 0; i < dq_goal.size(); ++i) {
      ss << "," << dq_goal[i];
    }

    // Feedforward torques
    for (int i = 0; i < tau_ff.size(); ++i) {
      ss << "," << tau_ff[i];
    }

    // Motor PD gains
    for (int i = 0; i < mot_K_p.size(); ++i) {
      ss << "," << mot_K_p[i];
    }
    for (int i = 0; i < mot_K_d.size(); ++i) {
      ss << "," << mot_K_d[i];
    }

    // Cartesian impedance parameters
    for (int i = 0; i < 6; ++i) {
      ss << "," << stiffness_diag[i];
    }
    for (int i = 0; i < 6; ++i) {
      ss << "," << damping_diag[i];
    }

    // Control parameters
    ss << "," << alpha;

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
