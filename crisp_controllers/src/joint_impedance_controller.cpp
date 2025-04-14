#include <crisp_controllers/friction_model.hpp>
#include <crisp_controllers/joint_impedance_controller.hpp>
#include <crisp_controllers/pseudo_inverse.hpp>
#include <crisp_controllers/torque_rate_saturation.hpp>

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
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
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
JointImpedanceController::update(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {
  // Update joint states
  for (int i = 0; i < num_joints_; i++) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints_ + i].get_value();
    tau_[i] = state_interfaces_[2 * num_joints_ + i].get_value();
  }

  tau_d = joint_stiffness_.cwiseProduct(q_ref_ - q_) +
          joint_damping_.cwiseProduct(-dq_);

  auto tau_f = params_.use_friction ? get_friction(dq_)
                                    : Eigen::VectorXd::Zero(num_joints_);
  auto tau_task = tau_d + tau_f;

  /*tau_d = saturateTorqueRate(tau_d, tau_);*/

  for (int i = 0; i < num_joints_; i++) {
    command_interfaces_[i].set_value(tau_task[i]);
  }

  updateParametersWithLowPassFilter();

  return controller_interface::return_type::OK;
}

void JointImpedanceController::updateParametersWithLowPassFilter() {
  params_ = params_listener_->get_params();
  float alpha = 0.95;
  joint_stiffness_ =
      alpha * joint_stiffness_ +
      (1 - alpha) * Eigen::Map<Eigen::VectorXd>(params_.joint_stiffness.data(),
                                                num_joints_);
  if (params_.joint_damping.size() == 0) {
    joint_damping_ = 2.0 * joint_stiffness_.cwiseSqrt();
  } else {
    joint_damping_ =
        alpha * joint_damping_ +
        (1 - alpha) * Eigen::Map<Eigen::VectorXd>(params_.joint_damping.data(),
                                                  num_joints_);
  }
}

CallbackReturn JointImpedanceController::on_init() {
  // Initialize parameters
  params_listener_ =
      std::make_shared<joint_impedance_controller::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  // Set basic parameters
  joint_names_ = params_.joints;
  num_joints_ = joint_names_.size();
  q_ref_ = Eigen::Map<Eigen::VectorXd>(params_.q_home.data(), num_joints_);
  dq_ref_ = Eigen::VectorXd::Zero(num_joints_);
  joint_stiffness_ =
      Eigen::Map<Eigen::VectorXd>(params_.joint_stiffness.data(), num_joints_);
  if (params_.joint_damping.size() == 0)
    joint_damping_ = 2.0 * joint_stiffness_.cwiseSqrt();
  else
    joint_damping_ =
        Eigen::Map<Eigen::VectorXd>(params_.joint_damping.data(), num_joints_);

  // Initialize state vectors
  q_ = Eigen::VectorXd::Zero(num_joints_);
  dq_ = Eigen::VectorXd::Zero(num_joints_);
  ddq_ = Eigen::VectorXd::Zero(num_joints_);
  tau_ = Eigen::VectorXd::Zero(num_joints_);

  // Get model bounds
  /*q_min_ = model_.lowerPositionLimit;*/
  /*q_max_ = model_.upperPositionLimit;*/
  /*dq_max_ = model_.velocityLimit;*/
  /*tau_max_ = model_.effortLimit;*/

  joint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "target_joint", rclcpp::QoS(1),
      std::bind(&JointImpedanceController::target_joint_callback_, this,
                std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (int i = 0; i < num_joints_; i++) {
    q_[i] = state_interfaces_[i].get_value();
    q_ref_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints_ + i].get_value();
    dq_ref_[i] = state_interfaces_[num_joints_ + i].get_value();
  }

  start_time_ = get_node()->get_clock()->now();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::target_joint_callback_(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->position.size(); i++) {
    if ((int)i < num_joints_) {
      q_ref_[i] = msg->position[i];
    }
  }
  for (size_t i = 0; i < msg->velocity.size(); i++) {
    if ((int)i < num_joints_) {
      dq_ref_[i] = msg->velocity[i];
    }
  }
}

} // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::JointImpedanceController,
                       controller_interface::ControllerInterface)
