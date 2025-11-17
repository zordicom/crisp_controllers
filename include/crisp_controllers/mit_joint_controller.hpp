#pragma once

/**
 * @file mit_joint_controller.hpp
 * @brief Joint-space impedance controller for MIT mode motors
 * @details This controller provides compliant joint-space control by sending
 * goal positions, velocities, and feedforward torques to leverage the motor's
 * MIT mode high-frequency internal control loop.
 */

#include <Eigen/Dense>
#include <array>
#include <controller_interface/controller_interface.hpp>
#include <crisp_controllers/mit_joint_controller_parameters.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "crisp_controllers/utils/csv_logger_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

/**
 * @brief Joint-space impedance controller for MIT mode motors
 *
 * This controller computes desired joint positions, velocities, and
 * feedforward torques based on joint-space targets, then sends all three
 * to the motors to leverage MIT mode's high-frequency internal control.
 *
 * Features:
 * - Per-joint impedance control (stiffness and damping)
 * - Multiple control modes (gravity, gravity+velocity, etc.)
 * - Designed for easy extension to trajectory tracking
 * - Optional oscillation detection per joint
 */
class MITJointController : public controller_interface::ControllerInterface {
public:
  /**
   * @brief Get the command interface configuration
   * @return Interface configuration specifying required command interfaces
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  /**
   * @brief Get the state interface configuration
   * @return Interface configuration specifying required state interfaces
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  /**
   * @brief Update function called periodically
   * @param time Current time
   * @param period Time since last update
   * @return Success/failure of update
   */
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  /**
   * @brief Initialize the controller
   * @return Success/failure of initialization
   */
  CallbackReturn on_init() override;

  /**
   * @brief Configure the controller
   * @param previous_state Previous lifecycle state
   * @return Success/failure of configuration
   */
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Activate the controller
   * @param previous_state Previous lifecycle state
   * @return Success/failure of activation
   */
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Deactivate the controller
   * @param previous_state Previous lifecycle state
   * @return Success/failure of deactivation
   */
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

private:
  /** @brief Subscription for target joint state messages */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_target_sub_;

  /** @brief Publisher for target end-effector pose (for visualization) */
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_pub_;

  /** @brief Realtime buffer for target joint state */
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>>
      target_joint_buffer_;

  /** @brief Flag indicating new target received */
  bool new_target_;

  /** @brief Target joint positions */
  Eigen::VectorXd q_target_;
  /** @brief Target joint velocities */
  Eigen::VectorXd dq_target_;

  /** @brief Parameter listener for dynamic parameter updates */
  std::shared_ptr<mit_joint_controller::ParamListener> params_listener_;
  /** @brief Current parameter values */
  mit_joint_controller::Params params_;

  /** @brief Pinocchio robot model */
  pinocchio::Model model_;
  /** @brief Pinocchio data for computations */
  pinocchio::Data data_;

  /** @brief Per-joint stiffness vector */
  Eigen::VectorXd K_joint_;
  /** @brief Per-joint damping vector */
  Eigen::VectorXd D_joint_;

  /** @brief Current joint positions (nv) */
  Eigen::VectorXd q_;
  /** @brief Current joint positions for Pinocchio (nq - may differ from nv) */
  Eigen::VectorXd q_pin_;
  /** @brief Current joint velocities (nv) */
  Eigen::VectorXd dq_;
  /** @brief Filtered joint velocities (nv) - reduces quantization noise */
  Eigen::VectorXd dq_filtered_;

  /** @brief Goal joint positions to send to motors (nv) */
  Eigen::VectorXd q_goal_;
  /** @brief Goal joint velocities to send to motors (nv) */
  Eigen::VectorXd dq_goal_;
  /** @brief Feedforward torques to send to motors (nv) */
  Eigen::VectorXd tau_ff_;

  /** @brief Motor Kp control loop parameters (per joint) */
  Eigen::VectorXd mot_K_p_;
  /** @brief Motor Kd control loop parameters (per joint) */
  Eigen::VectorXd mot_K_d_;

  /** @brief Joint position error */
  Eigen::VectorXd q_error_;

  /**
   * @brief Parse target joint state from realtime buffer
   */
  void parse_target_joint_();

  /**
   * @brief Gravity compensation only
   */
  void compute_gravity_();

  /**
   * @brief Gravity + Coriolis compensation
   */
  void compute_gravity_coriolis_();

  /**
   * @brief Gravity compensation + joint velocity damping
   */
  void compute_gravity_velocity_();

  /**
   * @brief Gravity compensation + joint impedance control
   * Uses feedforward torques: tau = K*(q_target - q) - D*dq
   */
  void compute_gravity_impedance_();

  /**
   * @brief Full impedance with position/velocity commands
   * Sends position and velocity goals to motors with dynamics compensation
   */
  void compute_impedance_posvel_();

  /** @brief CSV logger for controller data */
  std::unique_ptr<CSVLoggerInterface> csv_logger_;
  /** @brief Start time for CSV logging timestamp calculations */
  rclcpp::Time csv_log_start_time_;

  /** @brief Circular buffer for per-joint position error history (for
   * oscillation detection) Matrix where each row is a joint and each column is
   * a time sample */
  static constexpr size_t MAX_ERROR_HISTORY = 500;
  static constexpr size_t MAX_JOINTS = 7;
  std::array<std::array<double, MAX_ERROR_HISTORY>, MAX_JOINTS>
      joint_error_history_;
  /** @brief Current index in error history buffer */
  size_t error_history_idx_;
  /** @brief Number of valid samples in error history */
  size_t error_history_count_;
  /** @brief Flag to indicate controller should stop due to oscillation */
  bool oscillation_detected_;

  /**
   * @brief Detect oscillations in joint position error
   * @param dt Control loop period in seconds
   * @return true if oscillation detected
   */
  bool detect_oscillation_(double dt);

  // Throttle debug logs
  static constexpr int DEBUG_LOG_THROTTLE_MS = 1000;
};

} // namespace crisp_controllers
