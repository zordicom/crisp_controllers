#include <Eigen/src/Core/Matrix.h>
#include <crisp_controllers/pose_broadcaster.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <binders.h>
#include <cassert>
#include <cmath>
#include <memory>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
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

  // Decide whether to publish the pose or not
  bool should_publish = true;
  if (params_.publish_frequency > 0.0) {
    auto time_since_last = time - last_publish_time_;
    auto min_interval =
        rclcpp::Duration::from_seconds(1.0 / params_.publish_frequency);
    should_publish = time_since_last >= min_interval;
  }

  if (!should_publish) {
    return controller_interface::return_type::OK;
  }

  size_t num_joints = params_.joints.size();
  Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(model_.nq);

  for (size_t i = 0; i < num_joints; i++) {
    auto joint_name = params_.joints[i];
    auto joint_id = model_.getJointId(joint_name);
    auto joint = model_.joints[joint_id];

    q[i] = state_interfaces_[i].get_value();
    if (continous_joint_types.count(
            joint.shortname())) { // Then we are handling a continous joint that
                                  // is SO(2)
      q_pin[joint.idx_q()] = std::cos(q[i]);
      q_pin[joint.idx_q() + 1] = std::sin(q[i]);
    } else {
      q_pin[joint.idx_q()] = q[i];
    }
  }

  pinocchio::forwardKinematics(model_, data_, q_pin);
  pinocchio::updateFramePlacements(model_, data_);

  // Get poses in world frame
  pinocchio::SE3 ee_pose_world =
      data_.oMf[end_effector_frame_id];                       // World -> EE
  pinocchio::SE3 base_pose_world = data_.oMf[base_frame_id];  // World -> Base
  pinocchio::SE3 world_pose_base = base_pose_world.inverse(); // Base -> World

  // Transform to base frame: Base -> EE = (World -> Base)^-1 * (World -> EE)
  pinocchio::SE3 current_pose = world_pose_base * ee_pose_world;
  auto current_quaternion = Eigen::Quaterniond(current_pose.rotation());

  if (rt_pose_publisher_ && rt_pose_publisher_->trylock()) {
    auto &pose_msg = rt_pose_publisher_->msg_;

    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = params_.base_frame;
    pose_msg.pose.position.x = current_pose.translation()[0];
    pose_msg.pose.position.y = current_pose.translation()[1];
    pose_msg.pose.position.z = current_pose.translation()[2];
    pose_msg.pose.orientation.x = current_quaternion.x();
    pose_msg.pose.orientation.y = current_quaternion.y();
    pose_msg.pose.orientation.z = current_quaternion.z();
    pose_msg.pose.orientation.w = current_quaternion.w();
    rt_pose_publisher_->unlockAndPublish();
    last_publish_time_ = time;
  }

  if (rt_world_pose_publisher_ && rt_world_pose_publisher_->trylock()) {
    auto &pose_msg = rt_world_pose_publisher_->msg_;

    auto world_quat = Eigen::Quaterniond(world_pose_base.rotation());

    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = params_.base_frame;
    pose_msg.pose.position.x = world_pose_base.translation()[0];
    pose_msg.pose.position.y = world_pose_base.translation()[1];
    pose_msg.pose.position.z = world_pose_base.translation()[2];
    pose_msg.pose.orientation.x = world_quat.x();
    pose_msg.pose.orientation.y = world_quat.y();
    pose_msg.pose.orientation.z = world_quat.z();
    pose_msg.pose.orientation.w = world_quat.w();
    rt_world_pose_publisher_->unlockAndPublish();
    last_publish_time_ = time;
  }

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

  pinocchio::Model raw_model_;
  pinocchio::urdf::buildModelFromXML(robot_description_, raw_model_);

  RCLCPP_INFO(get_node()->get_logger(), "Checking available joints in model:");
  for (int joint_id = 0; joint_id < raw_model_.njoints; joint_id++) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Joint " << joint_id << " with name "
                                << raw_model_.names[joint_id] << " is of type "
                                << raw_model_.joints[joint_id].shortname());
  }

  // First we check that the passed joints exist in the kineatic tree
  for (auto &joint : params_.joints) {
    if (not raw_model_.existJointName(joint)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Failed to configure because "
                              << joint
                              << " is not part of the kinematic tree but it "
                                 "has been passed in the parameters.");
      return CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(get_node()->get_logger(),
              "All joints passed in the parameters exist in the kinematic tree "
              "of the URDF.");
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Removing the rest of the joints that are not used: ");
  // Now we fix all joints that are not referenced in the tree
  std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
  for (auto &joint : raw_model_.names) {
    if (std::find(params_.joints.begin(), params_.joints.end(), joint) ==
            params_.joints.end() and
        joint != "universe") {
      RCLCPP_INFO_STREAM(
          get_node()->get_logger(),
          "Joint " << joint << " is not used, removing it from the model.");
      list_of_joints_to_lock_by_id.push_back(raw_model_.getJointId(joint));
    }
  }

  Eigen::VectorXd q_locked = Eigen::VectorXd::Zero(raw_model_.nq);
  model_ = pinocchio::buildReducedModel(raw_model_,
                                        list_of_joints_to_lock_by_id, q_locked);
  data_ = pinocchio::Data(model_);

  for (int joint_id = 0; joint_id < model_.njoints; joint_id++) {
    if (model_.names[joint_id] == "universe") {
      continue;
    }
    if (not allowed_joint_types.count(model_.joints[joint_id].shortname())) {
      RCLCPP_ERROR_STREAM(
          get_node()->get_logger(),
          "Joint type "
              << model_.joints[joint_id].shortname() << " is unsupported ("
              << model_.names[joint_id]
              << "), only revolute/continous like joints can be used.");
      return CallbackReturn::ERROR;
    }
  }

  end_effector_frame_id = model_.getFrameId(params_.end_effector_frame);

  // Base frame is required for proper operation
  if (params_.base_frame.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "base_frame parameter is required but not specified!");
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Please set the base_frame parameter in the controller configuration.");
    return CallbackReturn::ERROR;
  }

  // Check if base frame exists in the model
  if (!model_.existFrame(params_.base_frame)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Base frame '" << params_.base_frame
                                       << "' not found in model!");
    RCLCPP_ERROR(get_node()->get_logger(), "Available frames:");
    for (size_t i = 0; i < model_.frames.size(); i++) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "  - " << model_.frames[i].name);
    }
    return CallbackReturn::ERROR;
  }

  base_frame_id = model_.getFrameId(params_.base_frame);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Found base frame '" << params_.base_frame
                                          << "' with ID: " << base_frame_id);
  RCLCPP_INFO_STREAM(
      get_node()->get_logger(),
      "Pose broadcaster will publish end-effector poses relative to: "
          << params_.base_frame);

  q = Eigen::VectorXd::Zero(model_.nv);

  // Create node-private topic using explicit namespacing
  // This ensures each broadcaster instance has its own topic namespace
  std::string node_name = get_node()->get_name();
  std::string pose_topic = node_name + "/current_pose";

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Publishing current pose to: " << pose_topic);

  pose_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          pose_topic, rclcpp::SystemDefaultsQoS());

  rt_pose_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      pose_publisher_);

  std::string world_pose_topic = node_name + "/world_pose";
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Publishing world pose to: " << world_pose_topic);

  world_pose_publisher_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          world_pose_topic, rclcpp::SystemDefaultsQoS());

  rt_world_pose_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
      world_pose_publisher_);

  last_publish_time_ = this->get_node()->now();
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
