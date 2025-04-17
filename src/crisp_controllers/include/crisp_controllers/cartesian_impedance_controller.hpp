#pragma once

/**
 * @file cartesian_impedance_controller.hpp
 * @brief Cartesian impedance controller implementation for robot manipulation
 * @author Your Organization
 */

#include <Eigen/Dense>

#include <Eigen/src/Geometry/Transform.h>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cartesian_impedance_controller_parameters.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

/**
 * @brief Controller implementing Cartesian impedance control
 *
 * This controller implements Cartesian impedance control for robotic manipulation,
 * allowing for compliant interaction with the environment while maintaining
 * desired position and orientation targets.
 */
class CartesianImpedanceController
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

  /*CartesianImpedanceController();*/

private:
  /** @brief Subscription for target pose messages */
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  /** @brief Subscription for target joint state messages */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  /**
   * @brief Callback for target joint state messages
   *
   * Updates the target joint configuration for the posture task
   * @param msg Target joint state message containing joint positions
   */
  void
  target_joint_callback_(const sensor_msgs::msg::JointState::SharedPtr msg);

  void
  target_pose_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Set the stiffness and damping matrices based on parameters
   */
  void setStiffnessAndDamping();

  /** @brief Target position in Cartesian space */
  Eigen::Vector3d target_position_;
  /** @brief Target orientation as quaternion */
  Eigen::Quaterniond target_orientation_;
  /** @brief Combined target pose as SE3 transformation */
  pinocchio::SE3 target_pose_;

  /** @brief Parameter listener for dynamic parameter updates */
  std::shared_ptr<cartesian_impedance_controller::ParamListener>
      params_listener_;
  /** @brief Current parameter values */
  cartesian_impedance_controller::Params params_;

  /** @brief Frame ID of the end effector in the robot model */
  int end_effector_frame_id;

  /** @brief Pinocchio robot model */
  pinocchio::Model model_;
  /** @brief Pinocchio data for computations */
  pinocchio::Data data_;

  /** @brief Cartesian stiffness matrix (6x6) */
  Eigen::MatrixXd stiffness = Eigen::MatrixXd::Zero(6, 6);
  /** @brief Cartesian damping matrix (6x6) */
  Eigen::MatrixXd damping = Eigen::MatrixXd::Zero(6, 6);

  /** @brief Nullspace stiffness matrix for posture control */
  Eigen::MatrixXd nullspace_stiffness;
  /** @brief Nullspace damping matrix for posture control */
  Eigen::MatrixXd nullspace_damping;

  /** @brief Current joint positions */
  Eigen::VectorXd q;
  /** @brief Current joint velocities */
  Eigen::VectorXd dq;
  /** @brief Current measured torque */
  Eigen::VectorXd tau;
  /** @brief Reference joint positions for posture task */
  Eigen::VectorXd q_ref;
  /** @brief Reference joint velocities */
  Eigen::VectorXd dq_ref;

  /** @brief Previously computed torque */
  Eigen::VectorXd tau_previous;

  /** @brief Current end effector pose */
  pinocchio::SE3 end_effector_pose;
  /** @brief End effector Jacobian matrix */
  pinocchio::Data::Matrix6x J;

};

} // namespace crisp_controllers
