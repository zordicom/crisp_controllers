#pragma once

/**
 * @file mit_cartesian_controller.hpp
 * @brief Simplified Cartesian impedance controller for MIT mode
 * @details This controller leverages the motor's MIT mode high-frequency
 * operation by sending goal positions, velocities, and feedforward torques
 * rather than just torques.
 */

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <crisp_controllers/mit_cartesian_controller_parameters.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "crisp_controllers/utils/csv_logger_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

/**
 * @brief Simplified Cartesian controller for MIT mode motors
 *
 * This controller computes desired joint positions, velocities, and
 * feedforward torques based on Cartesian error, then sends all three
 * to the motors to leverage MIT mode's high-frequency internal control.
 */
class MITCartesianController
    : public controller_interface::ControllerInterface {
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
  /** @brief Subscription for target pose messages */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  /** @brief Realtime buffer for target pose */
  realtime_tools::RealtimeBuffer<
      std::shared_ptr<geometry_msgs::msg::PoseStamped>>
      target_pose_buffer_;

  /** @brief Flag indicating new target pose received */
  bool new_target_pose_;

  /** @brief Target pose as SE3 transformation in world frame */
  pinocchio::SE3 target_pose_;

  /** @brief Parameter listener for dynamic parameter updates */
  std::shared_ptr<mit_cartesian_controller::ParamListener> params_listener_;
  /** @brief Current parameter values */
  mit_cartesian_controller::Params params_;

  /** @brief Frame ID of the end effector in the robot model */
  int ee_frame_id_;
  /** @brief Frame ID of the base frame in the robot model */
  int base_frame_id_;
  /** @brief Static transformation from world to base frame */
  pinocchio::SE3 base_frame_pose_world_;

  /** @brief Pinocchio robot model */
  pinocchio::Model model_;
  /** @brief Pinocchio data for computations */
  pinocchio::Data data_;

  /** @brief Cartesian stiffness matrix (6x6) */
  Eigen::Matrix<double, 6, 6> K_cart_;
  /** @brief Cartesian damping matrix (6x6) */
  Eigen::Matrix<double, 6, 6> D_cart_;

  /** @brief Current joint positions (nv) */
  Eigen::VectorXd q_;
  /** @brief Current joint positions for Pinocchio (nq - may differ from nv) */
  Eigen::VectorXd q_pin_;
  /** @brief Current joint velocities (nv) */
  Eigen::VectorXd dq_;
  /** @brief Filtered joint velocities (nv) - reduces 12-bit quantization noise */
  Eigen::VectorXd dq_filtered_;

  /** @brief Goal joint positions to send to motors (nv) */
  Eigen::VectorXd q_goal_;
  /** @brief Goal joint velocities to send to motors (nv) */
  Eigen::VectorXd dq_goal_;
  /** @brief Feedforward torques to send to motors (nv) */
  Eigen::VectorXd tau_ff_;

  /** @brief Motor Kp control loop parameter. */
  Eigen::VectorXd mot_K_p_;
  /** @brief Motor Kd control loop parameter. */
  Eigen::VectorXd mot_K_d_;

  /** @brief Current end effector pose in world frame */
  pinocchio::SE3 ee_pose_world_;
  /** @brief End effector Jacobian matrix (6 x nv) */
  pinocchio::Data::Matrix6x J_;
  /** @brief End effector Jacobian matrix (nv x 6) */
  Eigen::MatrixXd J_t_;
  /** @brief Pre-allocated Jacobian pseudo-inverse for realtime compliance */
  Eigen::MatrixXd J_pinv_;

  /**
   * @brief Parse target pose from realtime buffer
   */
  void parse_target_pose_();

  /** @brief CSV logger for controller data */
  std::unique_ptr<CSVLoggerInterface> csv_logger_;
  /** @brief Start time for CSV logging timestamp calculations */
  rclcpp::Time csv_log_start_time_;

  // Throttle debug logs
  static constexpr int DEBUG_LOG_THROTTLE_MS = 1000;
};

} // namespace crisp_controllers
