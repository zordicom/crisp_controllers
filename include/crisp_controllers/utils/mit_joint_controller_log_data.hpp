#pragma once

#include "controller_log_data_interface.hpp"
#include <Eigen/Dense>
#include <sstream>
#include <iomanip>

namespace crisp_controllers {

/**
 * @brief Log data specific to MIT Joint Controller
 *
 * Logs all the important values for debugging joint control:
 * - Current joint positions and velocities
 * - Target positions and velocities (from user)
 * - Goal positions and velocities (sent to motors)
 * - Joint errors (target - current)
 * - Feedforward torques
 * - Motor PD gains
 * - Joint impedance parameters
 * - Control loop timing
 */
class MITJointControllerLogData : public ControllerLogDataInterface {
public:
  // Timestamp
  double timestamp;

  // Control mode
  std::string control_mode;

  // Joint states (current)
  Eigen::VectorXd q;           // Current position
  Eigen::VectorXd dq;          // Current velocity
  Eigen::VectorXd dq_filtered; // Filtered velocity

  // Joint targets (from user via /target_joint topic)
  Eigen::VectorXd q_target;    // Target position
  Eigen::VectorXd dq_target;   // Target velocity

  // Joint goals (sent to motors)
  Eigen::VectorXd q_goal;      // Goal position command
  Eigen::VectorXd dq_goal;     // Goal velocity command

  // Joint errors
  Eigen::VectorXd q_error;     // Position error (target - current)

  // Feedforward torques (sent to motors)
  Eigen::VectorXd tau_ff;

  // Motor PD gains (sent to motors)
  Eigen::VectorXd mot_K_p;
  Eigen::VectorXd mot_K_d;

  // Joint impedance parameters
  Eigen::VectorXd joint_stiffness;
  Eigen::VectorXd joint_damping;

  // Control parameters
  double alpha;              // Velocity gain
  double max_position_error; // Error clipping limit
  double max_velocity;       // Velocity limit

  // Timing
  double loop_duration_ms;

  double getTimestamp() const override { return timestamp; }

  std::string generateHeader() const override {
    std::stringstream ss;

    size_t num_joints = q.size();

    // Basic info
    ss << "timestamp,control_mode";

    // Current joint states
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_filtered_" << i;
    }

    // Target joint states (from user)
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_target_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_target_" << i;
    }

    // Goal joint states (sent to motors)
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_goal_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",dq_goal_" << i;
    }

    // Joint errors
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",q_error_" << i;
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

    // Joint impedance parameters
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",K_joint_" << i;
    }
    for (size_t i = 0; i < num_joints; ++i) {
      ss << ",D_joint_" << i;
    }

    // Control parameters
    ss << ",alpha,max_position_error,max_velocity";

    // Timing
    ss << ",loop_duration_ms";

    return ss.str();
  }

  std::string generateLine() const override {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(6);

    // Timestamp and mode
    ss << timestamp << "," << control_mode;

    // Current joint states
    for (int i = 0; i < q.size(); ++i) {
      ss << "," << q[i];
    }
    for (int i = 0; i < dq.size(); ++i) {
      ss << "," << dq[i];
    }
    for (int i = 0; i < dq_filtered.size(); ++i) {
      ss << "," << dq_filtered[i];
    }

    // Target joint states
    for (int i = 0; i < q_target.size(); ++i) {
      ss << "," << q_target[i];
    }
    for (int i = 0; i < dq_target.size(); ++i) {
      ss << "," << dq_target[i];
    }

    // Goal joint states
    for (int i = 0; i < q_goal.size(); ++i) {
      ss << "," << q_goal[i];
    }
    for (int i = 0; i < dq_goal.size(); ++i) {
      ss << "," << dq_goal[i];
    }

    // Joint errors
    for (int i = 0; i < q_error.size(); ++i) {
      ss << "," << q_error[i];
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

    // Joint impedance parameters
    for (int i = 0; i < joint_stiffness.size(); ++i) {
      ss << "," << joint_stiffness[i];
    }
    for (int i = 0; i < joint_damping.size(); ++i) {
      ss << "," << joint_damping[i];
    }

    // Control parameters
    ss << "," << alpha << "," << max_position_error << "," << max_velocity;

    // Timing
    ss << "," << loop_duration_ms;

    return ss.str();
  }
};

} // namespace crisp_controllers
