#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <crisp_controllers/pose_broadcaster.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <binders.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
PoseBroadcaster::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
PoseBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/position");
  }
  return config;
}

controller_interface::return_type
PoseBroadcaster::update(const rclcpp::Time &time,
                                   const rclcpp::Duration & /*period*/) {

  for (size_t i = 0; i < params_.joints.size(); i++) {
    q[i] = state_interfaces_[i].get_value();
  }

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  auto current_pose = data_.oMf[end_effector_frame_id];
  auto current_quaternion =
      Eigen::Quaterniond(current_pose.rotation());

  auto pose_msg = geometry_msgs::msg::PoseStamped();
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = params_.base_frame;
  pose_msg.pose.position.x = current_pose.translation()[0];
  pose_msg.pose.position.y = current_pose.translation()[1];
  pose_msg.pose.position.z = current_pose.translation()[2];
  pose_msg.pose.orientation.x = current_quaternion.x();
  pose_msg.pose.orientation.y = current_quaternion.y();
  pose_msg.pose.orientation.z = current_quaternion.z();
  pose_msg.pose.orientation.w = current_quaternion.w();

  pose_publisher_->publish(pose_msg);

  return controller_interface::return_type::OK;
}

CallbackReturn PoseBroadcaster::on_init() {
  // Initialize parameters
  params_listener_ =
      std::make_shared<pose_broadcaster::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PoseBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();

  std::string robot_description_;
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get robot_description parameter.");
    return CallbackReturn::ERROR;
  }

  pinocchio::urdf::buildModelFromXML(robot_description_, model_);
  data_ = pinocchio::Data(model_);
  end_effector_frame_id = model_.getFrameId(params_.end_effector_frame);
  q = Eigen::VectorXd::Zero(model_.nq);

  pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          "current_pose", rclcpp::SystemDefaultsQoS());
  return CallbackReturn::SUCCESS;
}

CallbackReturn PoseBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PoseBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

} // namespace crisp_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(crisp_controllers::PoseBroadcaster,
                       controller_interface::ControllerInterface)
