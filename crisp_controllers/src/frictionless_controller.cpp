#include <crisp_controllers/friction_model.hpp>
#include <crisp_controllers/frictionless_controller.hpp>

#define USE_WARM_START

using namespace std::chrono_literals;

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
FrictionlessController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(arm_prefix_ + joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
FrictionlessController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    config.names.push_back(arm_prefix_ + joint_name + "/velocity");
  }
  return config;
}

controller_interface::return_type
FrictionlessController::update(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {
  // Update joint states
  for (size_t i = 0; i < joint_names_.size(); i++) {
    dq_[i] = state_interfaces_[i].get_value();
  }

  auto tau_f = get_friction(dq_);

  for (size_t i = 0; i < joint_names_.size(); i++) {
    command_interfaces_[i].set_value(tau_f[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn FrictionlessController::on_init() {
  auto arm_prefix = get_node()->get_parameter("arm_id").as_string();
  arm_prefix_ = arm_prefix == "" ? "" : arm_prefix + "_";
  return CallbackReturn::SUCCESS;
}

} // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::FrictionlessController,
                       controller_interface::ControllerInterface)
