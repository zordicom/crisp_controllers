#include <crisp_controllers/mit_cartesian_controller.hpp>
#include <crisp_controllers/pch.hpp>
#include <crisp_controllers/utils/pseudo_inverse.hpp>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <rclcpp/logging.hpp>

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
MITCartesianController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // We need position, velocity, and effort command interfaces
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/position");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/velocity");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
MITCartesianController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // We need position and velocity state interfaces
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/position");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

controller_interface::return_type
MITCartesianController::update(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {

  size_t num_joints = params_.joints.size();

  // Read current state from hardware
  for (size_t i = 0; i < num_joints; i++) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints + i].get_value();
    // For now, assume simple revolute joints (q_pin = q)
    q_pin_[i] = q_[i];
  }

  // Update target pose if new one received
  if (new_target_pose_) {
    parse_target_pose_();
    new_target_pose_ = false;
  }

  // Compute forward kinematics
  pinocchio::forwardKinematics(model_, data_, q_pin_, dq_);
  pinocchio::updateFramePlacements(model_, data_);

  // Get end-effector pose in world frame
  ee_pose_world_ = data_.oMf[ee_frame_id_];

  // Compute Cartesian error in world frame
  Eigen::Vector<double, 6> error;
  error.head(3) = target_pose_.translation() - ee_pose_world_.translation();
  error.tail(3) = pinocchio::log3(target_pose_.rotation() *
                                  ee_pose_world_.rotation().transpose());

  // Compute Jacobian in world frame
  J_.setZero();
  pinocchio::computeFrameJacobian(model_, data_, q_pin_, ee_frame_id_,
                                  pinocchio::ReferenceFrame::WORLD, J_);

  // Compute task space velocity
  Eigen::Vector<double, 6> dx = J_ * dq_;

  // Compute task space forces (impedance control law)
  Eigen::Vector<double, 6> F_task = K_cart_ * error - D_cart_ * dx;

  // Compute Jacobian pseudo-inverse
  // NOTE: pseudo_inverse() may allocate internally via SVD. For stricter
  // realtime compliance, consider implementing a damped least-squares solver
  // with fully pre-allocated matrices.
  J_pinv_ = pseudo_inverse(J_, params_.lambda);

  // Compute desired joint velocities from task space velocity error
  Eigen::Vector<double, 6> dx_desired = params_.alpha * error;
  dq_goal_ = J_pinv_ * dx_desired;

  // Compute feedforward torques
  tau_ff_.setZero();

  // Add task space forces if enabled
  if (params_.use_feedforward_forces) {
    tau_ff_ = J_.transpose() * F_task;
  }

  // Add gravity compensation if enabled
  if (params_.use_gravity_compensation) {
    Eigen::VectorXd tau_gravity =
        params_.gravity_scale *
        pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);
    tau_ff_ += tau_gravity;
  }

  // Compute goal joint positions (current + velocity * dt)
  // Using a small dt since MIT mode runs at high frequency
  double dt = params_.dt_goal;
  q_goal_ = q_ + dq_goal_ * dt;

  // Apply joint limits if configured
  if (params_.limit_commands) {
    // Clip goal positions to joint limits
    q_goal_ = q_goal_.cwiseMax(model_.lowerPositionLimit)
                  .cwiseMin(model_.upperPositionLimit);

    // Clip goal velocities
    dq_goal_ =
        dq_goal_.cwiseMax(-model_.velocityLimit).cwiseMin(model_.velocityLimit);

    // Clip feedforward torques
    tau_ff_ =
        tau_ff_.cwiseMax(-model_.effortLimit * params_.torque_safety_factor)
            .cwiseMin(model_.effortLimit * params_.torque_safety_factor);
  }

  // Send commands to hardware
  if (!params_.stop_commands) {
    for (size_t i = 0; i < num_joints; ++i) {
      command_interfaces_[i].set_value(q_goal_[i]);
      command_interfaces_[num_joints + i].set_value(dq_goal_[i]);
      command_interfaces_[2 * num_joints + i].set_value(tau_ff_[i]);
    }
  } else {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "stop_commands is TRUE - not sending commands to hardware!");
  }

  // Debug logging
  if (params_.log.enabled) {
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "Error: pos=" << error.head(3).transpose()
                                              << " ori="
                                              << error.tail(3).transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "q_goal: " << q_goal_.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "dq_goal: " << dq_goal_.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "tau_ff: " << tau_ff_.transpose());
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MITCartesianController::on_init() {
  params_listener_ =
      std::make_shared<mit_cartesian_controller::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MITCartesianController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(),
              "Starting MITCartesianController configuration...");

  // Get robot description
  auto param_node = std::make_shared<rclcpp::Node>(
      "mit_cartesian_controller_param_client",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          false));

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      param_node, "robot_state_publisher");

  RCLCPP_INFO(get_node()->get_logger(),
              "Waiting for robot_state_publisher service...");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) !=
                             std::future_status::ready) {
    rclcpp::spin_some(param_node);
  }

  auto result = future.get();
  std::string robot_description_;
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get robot_description parameter.");
    return CallbackReturn::ERROR;
  }

  // Build Pinocchio model
  RCLCPP_INFO(get_node()->get_logger(),
              "Building Pinocchio model from URDF...");
  pinocchio::Model raw_model_;
  pinocchio::urdf::buildModelFromXML(robot_description_, raw_model_);

  // Verify joints exist
  for (auto &joint : params_.joints) {
    if (!raw_model_.existJointName(joint)) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "Joint " << joint << " not found in model!");
      return CallbackReturn::ERROR;
    }
  }

  // Build reduced model with only the joints we care about
  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (auto &joint : raw_model_.names) {
    if (std::find(params_.joints.begin(), params_.joints.end(), joint) ==
            params_.joints.end() &&
        joint != "universe") {
      joints_to_lock.push_back(raw_model_.getJointId(joint));
    }
  }

  Eigen::VectorXd q_locked = Eigen::VectorXd::Zero(raw_model_.nq);
  model_ = pinocchio::buildReducedModel(raw_model_, joints_to_lock, q_locked);
  data_ = pinocchio::Data(model_);

  RCLCPP_INFO(get_node()->get_logger(), "Reduced model built successfully!");

  // Get end-effector frame
  if (!model_.existFrame(params_.end_effector_frame)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "End effector frame '" << params_.end_effector_frame
                                               << "' not found in model!");
    return CallbackReturn::ERROR;
  }
  ee_frame_id_ = model_.getFrameId(params_.end_effector_frame);

  // Get base frame
  if (params_.base_frame.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "base_frame parameter is required!");
    return CallbackReturn::ERROR;
  }
  if (!model_.existFrame(params_.base_frame)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Base frame '" << params_.base_frame
                                       << "' not found in model!");
    return CallbackReturn::ERROR;
  }
  base_frame_id_ = model_.getFrameId(params_.base_frame);

  // Initialize vectors (pre-allocate for realtime compliance)
  size_t num_joints = params_.joints.size();
  q_ = Eigen::VectorXd::Zero(num_joints);
  q_pin_ = Eigen::VectorXd::Zero(model_.nq);
  dq_ = Eigen::VectorXd::Zero(num_joints);
  q_goal_ = Eigen::VectorXd::Zero(num_joints);
  dq_goal_ = Eigen::VectorXd::Zero(num_joints);
  tau_ff_ = Eigen::VectorXd::Zero(num_joints);
  J_ = Eigen::MatrixXd::Zero(6, num_joints);
  J_pinv_ = Eigen::MatrixXd::Zero(num_joints, 6);

  // Set stiffness and damping
  K_cart_.setZero();
  K_cart_.diagonal() << params_.k_pos_x, params_.k_pos_y, params_.k_pos_z,
      params_.k_rot_x, params_.k_rot_y, params_.k_rot_z;

  D_cart_.setZero();
  D_cart_.diagonal() << 2.0 * std::sqrt(params_.k_pos_x),
      2.0 * std::sqrt(params_.k_pos_y), 2.0 * std::sqrt(params_.k_pos_z),
      2.0 * std::sqrt(params_.k_rot_x), 2.0 * std::sqrt(params_.k_rot_y),
      2.0 * std::sqrt(params_.k_rot_z);

  // Initialize target pose
  target_pose_ = pinocchio::SE3::Identity();
  new_target_pose_ = false;

  // Setup target pose subscription
  auto target_pose_callback =
      [this](
          const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) -> void {
    if (!params_.base_frame.empty() &&
        msg->header.frame_id != params_.base_frame) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Rejecting target pose: frame_id '%s' does not match "
                   "expected base_frame '%s'",
                   msg->header.frame_id.c_str(), params_.base_frame.c_str());
      return;
    }

    target_pose_buffer_.writeFromNonRT(msg);
    new_target_pose_ = true;
  };

  std::string node_name = get_node()->get_name();
  std::string pose_topic = node_name + "/target_pose";

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Subscribing to pose topic: " << pose_topic);

  pose_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, rclcpp::QoS(1), target_pose_callback);

  RCLCPP_INFO(get_node()->get_logger(),
              "MITCartesianController configuration completed successfully!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn MITCartesianController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  size_t num_joints = params_.joints.size();

  // Read current state
  for (size_t i = 0; i < num_joints; i++) {
    q_[i] = state_interfaces_[i].get_value();
    q_pin_[i] = q_[i];
    dq_[i] = state_interfaces_[num_joints + i].get_value();
  }

  // Compute forward kinematics
  pinocchio::forwardKinematics(model_, data_, q_pin_, dq_);
  pinocchio::updateFramePlacements(model_, data_);

  // Get current pose
  ee_pose_world_ = data_.oMf[ee_frame_id_];
  base_frame_pose_world_ = data_.oMf[base_frame_id_];

  // Initialize target to current pose
  target_pose_ = ee_pose_world_;

  // Initialize goals to current state
  q_goal_ = q_;
  dq_goal_ = dq_;
  tau_ff_.setZero();

  RCLCPP_INFO(
      get_node()->get_logger(),
      "MITCartesianController activated at position: [%.3f, %.3f, %.3f]",
      ee_pose_world_.translation().x(), ee_pose_world_.translation().y(),
      ee_pose_world_.translation().z());

  return CallbackReturn::SUCCESS;
}

CallbackReturn MITCartesianController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(get_node()->get_logger(), "MITCartesianController deactivated.");
  return CallbackReturn::SUCCESS;
}

void MITCartesianController::parse_target_pose_() {
  auto msg = *target_pose_buffer_.readFromRT();

  // Create SE3 from incoming pose (in base_frame coordinates)
  Eigen::Vector3d pose_in_base;
  pose_in_base << msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z;

  Eigen::Quaterniond quat_in_base(
      msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z);

  pinocchio::SE3 target_in_base(quat_in_base.toRotationMatrix(), pose_in_base);

  // Transform target from base frame to world frame
  target_pose_ = base_frame_pose_world_ * target_in_base;
}

}  // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(crisp_controllers::MITCartesianController,
                       controller_interface::ControllerInterface)
