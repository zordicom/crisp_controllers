#pragma once
#include <Eigen/Dense>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <crisp_controllers/pose_broadcaster_parameters.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>


using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace crisp_controllers {

class PoseBroadcaster
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

private:
  std::shared_ptr<pose_broadcaster::ParamListener> params_listener_;
  pose_broadcaster::Params params_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
    realtime_pose_publisher_;


  std::string end_effector_frame_;
  int end_effector_frame_id;
  int base_frame_id;

  pinocchio::Model model_;
  pinocchio::Data data_;

  /** @brief Allowed type of joints **/
  const std::unordered_set<std::basic_string<char>> allowed_joint_types = {
    "JointModelRX",
    "JointModelRY",
    "JointModelRZ",
    "JointModelRevoluteUnaligned",
    "JointModelRUBX",
    "JointModelRUBY",
    "JointModelRUBZ",
  };
  /** @brief Continous joint types that should be considered separetly. **/
  const std::unordered_set<std::basic_string<char>> continous_joint_types =
    {"JointModelRUBX", "JointModelRUBY", "JointModelRUBZ"};

  Eigen::VectorXd q;
  rclcpp::Time last_publish_time_;
};

} // namespace crisp_controllers
