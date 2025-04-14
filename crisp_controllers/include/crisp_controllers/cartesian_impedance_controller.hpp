#pragma once

#include <Eigen/Dense>

#include <Eigen/src/Geometry/Transform.h>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cartesian_impedance_controller_parameters.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

class CartesianImpedanceController
    : public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  CallbackReturn on_init() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /*CartesianImpedanceController();*/

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  void
  target_pose_callback_(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void setStiffnessAndDamping();

  Eigen::Vector3d target_position_;
  Eigen::Quaterniond target_orientation_;
  pinocchio::SE3 target_pose_;

  std::shared_ptr<cartesian_impedance_controller::ParamListener>
      params_listener_;
  cartesian_impedance_controller::Params params_;

  std::vector<std::string> joint_names_;
  int num_joints_;
  std::string end_effector_frame_;
  int end_effector_frame_id;
  bool use_fake_hardware_;

  pinocchio::Model model_;
  pinocchio::Data data_;

  // Compliance and kinematic parameters
  double translational_stiffness;
  double rotational_stiffness;
  double nullspace_stiffness_;
  double nullspace_damping_;
  double pi = 3.14159265358979323846;
  Eigen::Matrix<double, 9, 1> q_d_nullspace_{
      0.0, -pi / 4.0, 0.0, -3.0 * pi / 4.0, 0.0, pi / 2.0, pi / 4.0, 0.0, 0.0};

  Eigen::MatrixXd stiffness = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd damping = Eigen::MatrixXd::Zero(6, 6);
  bool limit_torques_ = false;
  pinocchio::Data::Matrix6x J;

  // Robot state dynamically set
  Eigen::VectorXd q;
  Eigen::VectorXd dq;
  Eigen::VectorXd tau;

  pinocchio::SE3 end_effector_pose;
};

} // namespace crisp_controllers
