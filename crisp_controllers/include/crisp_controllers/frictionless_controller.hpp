#pragma once
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

class FrictionlessController
    : public controller_interface::ControllerInterface {
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
  CallbackReturn on_init() override;

private:
  std::string arm_prefix_;
  std::vector<std::string> joint_names_{"joint1", "joint2", "joint3", "joint4",
                                        "joint5", "joint6", "joint7"};
  Eigen::VectorXd dq_ = Eigen::VectorXd::Zero(7);
};

} // namespace crisp_controllers
