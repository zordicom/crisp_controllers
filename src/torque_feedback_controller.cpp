#include <crisp_controllers/torque_feedback_controller.hpp>
#include <crisp_controllers/utils/friction_model.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <binders.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>

#define USE_WARM_START

using namespace std::chrono_literals;

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
TorqueFeedbackController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
TorqueFeedbackController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
  }
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/velocity");
  }
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::return_type
TorqueFeedbackController::update(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // Update joint states
  for (int i = 0; i < num_joints_; i++) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints_ + i].get_value();
    tau_[i] = state_interfaces_[2 * num_joints_ + i].get_value();
  }

  // Apply threshold to external torques based on vector magnitude
  Eigen::VectorXd tau_ext_thresholded = Eigen::VectorXd::Zero(num_joints_);
  double tau_ext_magnitude = tau_ext_.norm();
  if (tau_ext_magnitude > torque_threshold_) {
    tau_ext_thresholded = tau_ext_;
  }
  
  auto tau_d = -kp_fb_ * tau_ext_thresholded - kd_ * dq_;
  auto tau_f = get_friction(dq_, params_.friction.fp1, params_.friction.fp2, params_.friction.fp3);

  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_d[i] + tau_f[i]);
  }

  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();
  kp_fb_ = params_.k_fb;
  kd_ = params_.kd;
  torque_threshold_ = params_.torque_threshold;

  return controller_interface::return_type::OK;
}

CallbackReturn TorqueFeedbackController::on_init() {
  // Initialize parameters
  params_listener_ =
      std::make_shared<torque_feedback_controller::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  // Set basic parameters
  joint_names_ = params_.joints;
  num_joints_ = joint_names_.size();
  use_fake_hardware_ = params_.use_fake_hardware;
  kp_fb_ = params_.k_fb;
  kd_ = params_.kd;
  torque_threshold_ = params_.torque_threshold;

  // Initialize state vectors
  q_ = Eigen::VectorXd::Zero(7);
  dq_ = Eigen::VectorXd::Zero(7);
  tau_ = Eigen::VectorXd::Zero(7);

  tau_ext_ = Eigen::VectorXd::Zero(7);

  joint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      params_.joint_source_topic,
      rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile(),
      std::bind(&TorqueFeedbackController::target_joint_callback_, this,
                std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn TorqueFeedbackController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

CallbackReturn TorqueFeedbackController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (int i = 0; i < num_joints_; i++) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints_ + i].get_value();
    tau_[i] = state_interfaces_[2 * num_joints_ + i].get_value();
    tau_ext_[i] = 0.0;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TorqueFeedbackController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void TorqueFeedbackController::target_joint_callback_(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->effort.size(); i++) {
    if ((int)i < num_joints_) {
      tau_ext_[i] = msg->effort[i];
    }
  }
}

} // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::TorqueFeedbackController,
                       controller_interface::ControllerInterface)
