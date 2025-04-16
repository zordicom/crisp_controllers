#pragma once
#include <Eigen/Dense>

#include <Eigen/src/Core/Matrix.h>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <crisp_controllers/joint_impedance_controller_parameters.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

/**
 * TODO:
 */
class JointImpedanceController
    : public controller_interface::ControllerInterface {
public:
  /**
   * @brief Specifies the command interfaces required by the controller
   * @return Interface configuration specifying effort command interfaces for
   * each joint
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  /**
   * @brief Specifies the state interfaces required by the controller
   * @return Interface configuration specifying position, velocity and effort
   * state interfaces for each joint
   */
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  /**
   * @brief Main control loop update function
   *
   * @param time Current time
   * @param period Time since last update
   * @return Success/failure of the update
   */
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  /**
   * @brief Initializes the controller
   *
   * @return Success/failure of initialization
   */
  CallbackReturn on_init() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

private:
  /**
   * @brief Updates controller parameters using a low-pass filter
   *
   * Smoothly updates parameters like velocity scaling and task weights
   * to avoid sudden changes in robot behavior
   */
  void updateParametersWithLowPassFilter();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  /**
   * @brief Callback for target joint state messages
   *
   * Updates the target joint configuration for the posture task
   * @param msg Target joint state message containing joint positions
   */
  void
  target_joint_callback_(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::shared_ptr<joint_impedance_controller::ParamListener> params_listener_;
  joint_impedance_controller::Params params_;

  std::vector<std::string> joint_names_;
  int num_joints_;

  rclcpp::Time start_time_;

  Eigen::VectorXd q_;
  Eigen::VectorXd dq_;
  Eigen::VectorXd tau_;
  Eigen::VectorXd tau_d;
  Eigen::VectorXd ddq_;
  std::vector<double> q_home_list;
  Eigen::VectorXd q_ref_;
  Eigen::VectorXd dq_ref_;

  Eigen::VectorXd q_max_;
  Eigen::VectorXd q_min_;
  Eigen::VectorXd dq_max_;
  Eigen::VectorXd tau_max_;

  Eigen::VectorXd joint_stiffness_;
  Eigen::VectorXd joint_damping_;
};

} // namespace crisp_controllers
