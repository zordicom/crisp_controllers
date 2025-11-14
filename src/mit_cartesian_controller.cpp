#include <crisp_controllers/mit_cartesian_controller.hpp>
#include <crisp_controllers/pch.hpp>
#include <crisp_controllers/utils/async_csv_logger.hpp>
#include <crisp_controllers/utils/csv_logger.hpp>
#include <crisp_controllers/utils/mit_controller_log_data.hpp>
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
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/kp");
  }
  for (const auto &joint_name : params_.joints) {
    config.names.push_back(joint_name + "/kd");
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
MITCartesianController::update(const rclcpp::Time &time,
                               const rclcpp::Duration &period) {
  // Start timing the control loop for logging
  auto loop_start_time = get_node()->get_clock()->now();

  size_t num_joints = params_.joints.size();

  // Read current state from hardware
  for (size_t i = 0; i < num_joints; i++) {
    q_[i] = state_interfaces_[i].get_value();
    dq_[i] = state_interfaces_[num_joints + i].get_value();
    // For now, assume simple revolute joints (q_pin = q)
    q_pin_[i] = q_[i];
  }

  // Apply low-pass filter to velocity feedback to reduce 12-bit quantization
  // noise EMA filter: dq_filtered = alpha * dq_measured + (1-alpha) *
  // dq_filtered_prev
  dq_filtered_ = params_.dq_filter_alpha * dq_ +
                 (1.0 - params_.dq_filter_alpha) * dq_filtered_;

  // Update target pose if new one received
  if (new_target_pose_) {
    parse_target_pose_();
    new_target_pose_ = false;
  }

  // Compute forward kinematics using filtered velocity
  pinocchio::forwardKinematics(model_, data_, q_pin_, dq_filtered_);
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

  // Compute Jacobian pseudo-inverse for IK
  J_pinv_ = pseudo_inverse(J_, params_.lambda);

  // Position command: Use pseudo-inverse IK to find goal joint positions
  if (params_.use_position_command) {
    // Iterative pseudo-inverse Jacobian IK
    // Start from current position and take a step toward the target
    Eigen::VectorXd delta_q = J_pinv_ * error;
    q_goal_ = q_ + delta_q;
    mot_K_p_ = Eigen::VectorXd::Ones(num_joints);
  } else {
    q_goal_ = q_;                                 // Set to current position
    mot_K_p_ = Eigen::VectorXd::Zero(num_joints); // Disable position control
  }

  // Velocity command: Set to zero (let motor PD handle it)
  dq_goal_.setZero();
  mot_K_d_ = Eigen::VectorXd::Ones(num_joints);

  // Feedforward torques: Cartesian impedance + gravity compensation
  tau_ff_.setZero();

  // Add Cartesian impedance via feedforward torque
  // tau = J^T * (K * error - D * x_dot)
  Eigen::Vector<double, 6> task_velocity = J_ * dq_filtered_;
  Eigen::Vector<double, 6> cart_force =
      K_cart_ * error - D_cart_ * task_velocity;
  tau_ff_ = J_.transpose() * cart_force;

  // Add gravity compensation if enabled
  if (params_.use_gravity_compensation) {
    Eigen::VectorXd tau_gravity =
        params_.gravity_scale *
        pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);
    tau_ff_ += tau_gravity;
  }

  // Apply joint limits if configured
  if (params_.limit_commands) {
    // Clip goal velocities
    dq_goal_ =
        dq_goal_.cwiseMax(-model_.velocityLimit).cwiseMin(model_.velocityLimit);

    // Clip feedforward torques
    tau_ff_ =
        tau_ff_.cwiseMax(-model_.effortLimit * params_.torque_safety_factor)
            .cwiseMin(model_.effortLimit * params_.torque_safety_factor);
  }

  // Clamp motor gains to safe limits for MIT actuators
  // Kp must be in [0, 500], Kd must be in [0, 5]
  mot_K_p_ = mot_K_p_.cwiseMax(0.0).cwiseMin(500.0);
  mot_K_d_ = mot_K_d_.cwiseMax(0.0).cwiseMin(5.0);

  // Send commands to hardware
  if (!params_.stop_commands) {
    for (size_t i = 0; i < num_joints; ++i) {
      command_interfaces_[0 * num_joints + i].set_value(q_goal_[i]);
      command_interfaces_[1 * num_joints + i].set_value(dq_goal_[i]);
      command_interfaces_[2 * num_joints + i].set_value(tau_ff_[i]);
      command_interfaces_[3 * num_joints + i].set_value(mot_K_p_[i]);
      command_interfaces_[4 * num_joints + i].set_value(mot_K_d_[i]);
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
                                "mot_K_p: " << mot_K_p_.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "mot_K_d: " << mot_K_d_.transpose());
    RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(),
                                *get_node()->get_clock(), DEBUG_LOG_THROTTLE_MS,
                                "tau_ff: " << tau_ff_.transpose());
  }

  // Write CSV data if logging is enabled
  if (csv_logger_ && csv_logger_->isLoggingEnabled()) {
    MITControllerLogData log_data;

    // Populate log data structure - only what MIT controller actually computes
    log_data.timestamp = (time - csv_log_start_time_).seconds();

    // Poses
    log_data.current_pose = ee_pose_world_;
    log_data.target_pose = target_pose_;

    // Cartesian error
    log_data.error = error;

    // Joint states
    log_data.q = q_;
    log_data.dq = dq_;
    log_data.dq_filtered = dq_filtered_;

    // MIT controller specific: goal positions and velocities sent to motors
    log_data.q_goal = q_goal_;
    log_data.dq_goal = dq_goal_;

    // Feedforward torques sent to motors
    log_data.tau_ff = tau_ff_;

    // Motor PD gains sent to motors
    log_data.mot_K_p = mot_K_p_;
    log_data.mot_K_d = mot_K_d_;

    // Cartesian impedance parameters
    log_data.stiffness_diag = K_cart_.diagonal();
    log_data.damping_diag = D_cart_.diagonal();

    // Control parameters
    log_data.alpha = params_.alpha;

    // Calculate control loop duration
    auto loop_end_time = get_node()->get_clock()->now();
    log_data.loop_duration_ms =
        (loop_end_time - loop_start_time).nanoseconds() * 1e-6;

    // Log the data using the interface
    csv_logger_->logData(log_data);
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
  dq_filtered_ = Eigen::VectorXd::Zero(num_joints);
  q_goal_ = Eigen::VectorXd::Zero(num_joints);
  dq_goal_ = Eigen::VectorXd::Zero(num_joints);
  tau_ff_ = Eigen::VectorXd::Zero(num_joints);
  J_ = Eigen::MatrixXd::Zero(6, num_joints);
  J_t_ = Eigen::MatrixXd::Zero(num_joints, 6);
  J_pinv_ = Eigen::MatrixXd::Zero(num_joints, 6);
  mot_K_p_ = Eigen::VectorXd::Ones(num_joints);
  mot_K_d_ = Eigen::VectorXd::Ones(num_joints);

  // Set stiffness and damping
  K_cart_.setZero();
  K_cart_.diagonal() << params_.k_pos_x, params_.k_pos_y, params_.k_pos_z,
      params_.k_rot_x, params_.k_rot_y, params_.k_rot_z;

  D_cart_.setZero();
  D_cart_.diagonal() << params_.d_pos_x, params_.d_pos_y, params_.d_pos_z,
      params_.d_rot_x, params_.d_rot_y, params_.d_rot_z;

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

  // Initialize filtered velocity to current velocity
  dq_filtered_ = dq_;

  // Compute forward kinematics
  pinocchio::forwardKinematics(model_, data_, q_pin_, dq_filtered_);
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

  // Initialize CSV logger if enabled
  if (params_.log.enabled) {
    // Store the start time for consistent timestamp calculations
    csv_log_start_time_ = get_node()->now();

    // Create a dummy log data object just for header generation
    MITControllerLogData dummy_log;
    dummy_log.q = Eigen::VectorXd::Zero(num_joints);
    dummy_log.dq = Eigen::VectorXd::Zero(num_joints);
    dummy_log.dq_filtered = Eigen::VectorXd::Zero(num_joints);
    dummy_log.q_goal = Eigen::VectorXd::Zero(num_joints);
    dummy_log.dq_goal = Eigen::VectorXd::Zero(num_joints);
    dummy_log.tau_ff = Eigen::VectorXd::Zero(num_joints);
    dummy_log.mot_K_p = Eigen::VectorXd::Zero(num_joints);
    dummy_log.mot_K_d = Eigen::VectorXd::Zero(num_joints);

    if (params_.log.use_async_logging) {
      // Use async logger for better real-time performance
      csv_logger_ = std::make_unique<AsyncCSVLogger>(get_node()->get_name(),
                                                     get_node()->get_logger());

      RCLCPP_INFO(get_node()->get_logger(),
                  "Using ASYNC CSV logging for improved real-time performance");
    } else {
      // Fallback to synchronous logger
      csv_logger_ = std::make_unique<ControllerCSVLogger>(
          get_node()->get_name(), get_node()->get_logger());

      RCLCPP_WARN(
          get_node()->get_logger(),
          "Using SYNCHRONOUS CSV logging - may impact real-time performance!");
    }

    if (!csv_logger_->initialize(csv_log_start_time_, dummy_log)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize CSV logger");
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn MITCartesianController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Close CSV logger if it was opened
  if (csv_logger_) {
    csv_logger_->close();
    csv_logger_.reset();
  }

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

} // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(crisp_controllers::MITCartesianController,
                       controller_interface::ControllerInterface)
