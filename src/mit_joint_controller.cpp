#include <crisp_controllers/mit_joint_controller.hpp>
#include <crisp_controllers/pch.hpp>
#include <crisp_controllers/utils/async_csv_logger.hpp>
#include <crisp_controllers/utils/csv_logger.hpp>
#include <crisp_controllers/utils/mit_joint_controller_log_data.hpp>
#include <future>
#include <limits>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>

namespace crisp_controllers {

controller_interface::InterfaceConfiguration
MITJointController::command_interface_configuration() const {
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
MITJointController::state_interface_configuration() const {
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
MITJointController::update(const rclcpp::Time &time,
                           const rclcpp::Duration &period) {
  struct timespec loop_start, section_start, section_end;
  clock_gettime(CLOCK_MONOTONIC, &loop_start);

  if (params_.stop_commands) {
    return controller_interface::return_type::OK;
  }

  size_t num_joints = params_.joints.size();

  // 1. Read current state
  clock_gettime(CLOCK_MONOTONIC, &section_start);
  for (size_t i = 0; i < num_joints; i++) {
    q_[i] = state_interfaces_[i].get_value();
    q_pin_[i] = q_[i];
    dq_[i] = state_interfaces_[num_joints + i].get_value();
  }
  clock_gettime(CLOCK_MONOTONIC, &section_end);
  timing_stats_.state_read.add_sample(timespec_diff_us(section_start, section_end));

  // 2. Apply velocity filtering to reduce quantization noise
  clock_gettime(CLOCK_MONOTONIC, &section_start);
  dq_filtered_ = params_.dq_filter_alpha * dq_ +
                 (1.0 - params_.dq_filter_alpha) * dq_filtered_;
  clock_gettime(CLOCK_MONOTONIC, &section_end);
  timing_stats_.velocity_filter.add_sample(timespec_diff_us(section_start, section_end));

  // 3. Parse target and apply smoothing
  clock_gettime(CLOCK_MONOTONIC, &section_start);
  // Parse target if new one available
  if (new_target_) {
    parse_target_joint_();
  }

  // Apply target smoothing if enabled
  if (params_.target_smoothing.enabled) {
    if (smoothing_active_) {
      // Update smoothing
      smoothing_time_ += period.seconds();
      double blend_ratio =
          std::min(1.0, smoothing_time_ / params_.target_smoothing.blend_time);

      // Smooth blend using cubic interpolation for smoother velocity profile
      double t = blend_ratio;
      double blend = t * t * (3.0 - 2.0 * t); // Smoothstep function

      q_target_ = (1.0 - blend) * q_target_start_ + blend * q_target_end_;

      // Finish smoothing when complete
      if (blend_ratio >= 1.0) {
        smoothing_active_ = false;
        q_target_ = q_target_end_;
      }
    }
  }
  clock_gettime(CLOCK_MONOTONIC, &section_end);
  timing_stats_.target_parse.add_sample(timespec_diff_us(section_start, section_end));

  // Compute joint error
  q_error_ = q_target_ - q_;

  // Clip error to prevent excessive torques when far from target
  double max_error = params_.max_position_error;
  q_error_ = q_error_.cwiseMax(-max_error).cwiseMin(max_error);

  // 4. Oscillation detection (if enabled)
  if (params_.oscillation_detection.enabled) {
    clock_gettime(CLOCK_MONOTONIC, &section_start);

    // Store per-joint position errors in history
    for (size_t i = 0; i < num_joints && i < MAX_JOINTS; ++i) {
      joint_error_history_[i][error_history_idx_] = q_error_(i);
    }
    error_history_idx_ = (error_history_idx_ + 1) % MAX_ERROR_HISTORY;
    if (error_history_count_ < MAX_ERROR_HISTORY) {
      error_history_count_++;
    }

    // Check for oscillations in any joint
    if (detect_oscillation_(period.seconds())) {
      oscillation_detected_ = true;
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Oscillation detected! Setting stop_commands=true.");
      params_.stop_commands = true;
    }

    clock_gettime(CLOCK_MONOTONIC, &section_end);
    timing_stats_.oscillation_detect.add_sample(timespec_diff_us(section_start, section_end));
  }

  // Stop sending commands if oscillation detected
  if (oscillation_detected_) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Controller stopped due to oscillation detection! stop_commands=true");
    return controller_interface::return_type::ERROR;
  }

  // 5. Control computation
  clock_gettime(CLOCK_MONOTONIC, &section_start);
  // Call the appropriate control mode function
  if (params_.control_mode == "gravity") {
    compute_gravity_();
  } else if (params_.control_mode == "impedance_posvel") {
    compute_impedance_posvel_();
  } else {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          1000, "Unknown control mode: %s",
                          params_.control_mode.c_str());
    return controller_interface::return_type::ERROR;
  }
  clock_gettime(CLOCK_MONOTONIC, &section_end);
  timing_stats_.control_compute.add_sample(timespec_diff_us(section_start, section_end));

  // Apply joint limits with safety buffer if enabled
  if (params_.limit_commands) {
    q_goal_ = q_goal_.cwiseMax(joint_lower_limit_).cwiseMin(joint_upper_limit_);
  }

  // Apply torque safety limits
  tau_ff_ = tau_ff_.cwiseMax(-tau_limits_).cwiseMin(tau_limits_);

  // 6. Write commands to hardware
  clock_gettime(CLOCK_MONOTONIC, &section_start);
  for (size_t i = 0; i < num_joints; i++) {
    // Position command
    command_interfaces_[i].set_value(q_goal_[i]);

    // Velocity command
    command_interfaces_[num_joints + i].set_value(dq_goal_[i]);

    // Feedforward torque
    command_interfaces_[2 * num_joints + i].set_value(tau_ff_[i]);

    // Motor gains
    command_interfaces_[3 * num_joints + i].set_value(mot_K_p_[i]);
    command_interfaces_[4 * num_joints + i].set_value(mot_K_d_[i]);
  }
  clock_gettime(CLOCK_MONOTONIC, &section_end);
  timing_stats_.command_write.add_sample(timespec_diff_us(section_start, section_end));

  // 7. CSV logging if enabled
  if (params_.log.enabled && csv_logger_) {
    clock_gettime(CLOCK_MONOTONIC, &section_start);

    MITJointControllerLogData log_data;
    log_data.timestamp =
        (time - csv_log_start_time_).nanoseconds() * 1e-9; // seconds

    // Set control mode
    log_data.control_mode = params_.control_mode;

    // Log joint states (current)
    log_data.q = q_;
    log_data.dq = dq_;
    log_data.dq_filtered = dq_filtered_;

    // Log joint targets (from user)
    log_data.q_target = q_target_;
    log_data.dq_target = dq_target_;

    // Log joint goals (sent to motors)
    log_data.q_goal = q_goal_;
    log_data.dq_goal = dq_goal_;

    // Log joint errors
    log_data.q_error = q_error_;

    // Log feedforward torques
    log_data.tau_ff = tau_ff_;

    // Log motor PD gains
    log_data.mot_K_p = mot_K_p_;
    log_data.mot_K_d = mot_K_d_;

    // Log joint impedance parameters
    log_data.joint_stiffness = K_joint_;
    log_data.joint_damping = D_joint_;

    // Log control parameters
    log_data.alpha = params_.alpha;
    log_data.max_position_error = params_.max_position_error;
    log_data.max_velocity = params_.max_velocity;

    // Compute loop duration for CSV log
    struct timespec loop_end_for_csv;
    clock_gettime(CLOCK_MONOTONIC, &loop_end_for_csv);
    log_data.loop_duration_ms = timespec_diff_us(loop_start, loop_end_for_csv) * 0.001;

    csv_logger_->logData(log_data);

    clock_gettime(CLOCK_MONOTONIC, &section_end);
    timing_stats_.csv_logging.add_sample(timespec_diff_us(section_start, section_end));
  }

  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  // 8. Record total loop time and log statistics
  struct timespec loop_end;
  clock_gettime(CLOCK_MONOTONIC, &loop_end);
  timing_stats_.total_loop.add_sample(timespec_diff_us(loop_start, loop_end));

  // Log timing statistics every 1 second
  auto now_steady = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now_steady - last_timing_log_).count() > 1000) {
    log_timing_statistics_();
    timing_stats_.reset_all();
    last_timing_log_ = now_steady;
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MITJointController::on_init() {
  params_listener_ =
      std::make_shared<mit_joint_controller::ParamListener>(get_node());
  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MITJointController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Load robot model from URDF
  std::string robot_description;

  // Try to read robot_description from this controller's node parameters first
  if (get_node()->has_parameter("robot_description")) {
    robot_description = get_node()->get_parameter("robot_description").as_string();
    if (!robot_description.empty()) {
      RCLCPP_INFO(get_node()->get_logger(),
                  "Got robot_description parameter from controller node (length: %zu bytes)",
                  robot_description.size());
    }
  }

  // Fallback: subscribe to robot_description topic if not found on controller node
  if (robot_description.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
                "robot_description not found on controller node, waiting for %s topic",
                params_.robot_description_topic.c_str());

    std::promise<std::string> urdf_promise;
    auto urdf_future = urdf_promise.get_future();
    bool received = false;

    auto sub = get_node()->create_subscription<std_msgs::msg::String>(
        params_.robot_description_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
        [&](const std_msgs::msg::String::SharedPtr msg) {
          if (!received) {
            urdf_promise.set_value(msg->data);
            received = true;
          }
        });

    // Wait for message with timeout
    if (urdf_future.wait_for(std::chrono::seconds(5)) == std::future_status::timeout) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Timeout waiting for %s topic", params_.robot_description_topic.c_str());
      return CallbackReturn::ERROR;
    }

    robot_description = urdf_future.get();
    if (robot_description.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description from topic is empty");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(),
                "Got robot_description from %s topic (length: %zu bytes)",
                params_.robot_description_topic.c_str(), robot_description.size());
  }

  // Load full robot model from URDF
  pinocchio::Model full_model;
  pinocchio::urdf::buildModelFromXML(robot_description, full_model);

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Loaded full robot model with " << full_model.njoints
                                                << " joints (nv=" << full_model.nv
                                                << ", nq=" << full_model.nq << ")");

  size_t num_joints = params_.joints.size();

  // If the number of controlled joints matches the full model, use it directly
  if (num_joints == static_cast<size_t>(full_model.nv)) {
    model_ = full_model;
    RCLCPP_INFO(get_node()->get_logger(),
                "All joints in model will be controlled");
  } else if (num_joints < static_cast<size_t>(full_model.nv)) {
    // Build a reduced model containing only the controlled joints
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Building reduced model with " << num_joints
                       << " controlled joints out of " << full_model.nv);

    // Create list of joint IDs to keep
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (pinocchio::JointIndex joint_id = 1; joint_id < full_model.joints.size(); ++joint_id) {
      const std::string& joint_name = full_model.names[joint_id];

      // Check if this joint is in the controlled joints list
      bool is_controlled = false;
      for (const auto& controlled_joint : params_.joints) {
        if (joint_name == controlled_joint) {
          is_controlled = true;
          break;
        }
      }

      // Lock joints that are not controlled
      if (!is_controlled) {
        joints_to_lock.push_back(joint_id);
      }
    }

    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Locking " << joints_to_lock.size()
                       << " uncontrolled joints");

    // Build reduced model with locked joints at their zero configuration
    Eigen::VectorXd reference_config = Eigen::VectorXd::Zero(full_model.nq);
    pinocchio::buildReducedModel(full_model, joints_to_lock, reference_config, model_);

    RCLCPP_INFO_STREAM(get_node()->get_logger(),
                       "Reduced model has " << model_.njoints
                                            << " joints (nv=" << model_.nv
                                            << ", nq=" << model_.nq << ")");

    // Validate the reduced model has the correct number of DOF
    if (static_cast<size_t>(model_.nv) != num_joints) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Reduced model nv (%d) does not match number of controlled joints (%zu). "
                   "Check that all joint names in params exist in the URDF.",
                   model_.nv, num_joints);
      return CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Number of joints in params (%zu) is greater than model nv (%d). "
                 "Check your joint configuration.",
                 num_joints, full_model.nv);
    return CallbackReturn::ERROR;
  }

  data_ = pinocchio::Data(model_);

  // Initialize vectors
  q_ = Eigen::VectorXd::Zero(num_joints);
  q_pin_ = Eigen::VectorXd::Zero(model_.nq);
  dq_ = Eigen::VectorXd::Zero(num_joints);
  dq_filtered_ = Eigen::VectorXd::Zero(num_joints);
  q_goal_ = Eigen::VectorXd::Zero(num_joints);
  dq_goal_ = Eigen::VectorXd::Zero(num_joints);
  tau_ff_ = Eigen::VectorXd::Zero(num_joints);
  mot_K_p_ = Eigen::VectorXd::Ones(num_joints);
  mot_K_d_ = Eigen::VectorXd::Ones(num_joints);
  q_error_ = Eigen::VectorXd::Zero(num_joints);
  q_target_ = Eigen::VectorXd::Zero(num_joints);
  dq_target_ = Eigen::VectorXd::Zero(num_joints);

  // Initialize target smoothing
  smoothing_active_ = false;
  q_target_start_ = Eigen::VectorXd::Zero(num_joints);
  q_target_end_ = Eigen::VectorXd::Zero(num_joints);
  smoothing_time_ = 0.0;

  // Initialize oscillation detection
  for (size_t i = 0; i < MAX_JOINTS; ++i) {
    joint_error_history_[i].fill(0.0);
  }
  error_history_idx_ = 0;
  error_history_count_ = 0;
  oscillation_detected_ = false;

  // Set per-joint impedance gains
  K_joint_ = Eigen::VectorXd::Zero(num_joints);
  D_joint_ = Eigen::VectorXd::Zero(num_joints);

  for (size_t i = 0; i < num_joints; ++i) {
    K_joint_[i] = params_.joint_stiffness[i];
    if (params_.joint_damping[i] > 0) {
      D_joint_[i] = params_.joint_damping[i];
    } else {
      // Critical damping: 2*sqrt(k)
      D_joint_[i] = 2.0 * std::sqrt(K_joint_[i]);
    }
  }

  // Preallocate limit vectors to avoid heap allocations in update loop
  joint_lower_limit_ =
      model_.lowerPositionLimit.array() + params_.joint_limit_buffer;
  joint_upper_limit_ =
      model_.upperPositionLimit.array() - params_.joint_limit_buffer;
  tau_limits_ = params_.torque_safety_factor * model_.effortLimit;

  // Set motor gains once (they're constant for each control mode)
  // These will be written to hardware on every cycle but not recomputed
  if (params_.control_mode == "gravity") {
    mot_K_p_.setZero();
    mot_K_d_.setZero();
  } else if (params_.control_mode == "impedance_posvel") {
    mot_K_p_ = K_joint_;
    mot_K_d_ = D_joint_;
  }

  // Initialize target to zero (will be set to current position on activate)
  new_target_ = false;

  // Create publisher for target pose
  std::string node_name = get_node()->get_name();
  std::string pose_topic = node_name + "/target_pose";
  target_pose_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
          pose_topic, rclcpp::QoS(1));

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Publishing target pose to: " << pose_topic);

  // Find end-effector frame index
  pinocchio::FrameIndex ee_frame_id =
      model_.getFrameId(params_.end_effector_frame);
  if (ee_frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "End-effector frame '" << params_.end_effector_frame
                                               << "' not found in model!");
    return CallbackReturn::ERROR;
  }

  // Find base frame index
  pinocchio::FrameIndex base_frame_id = model_.getFrameId(params_.base_frame);
  if (base_frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "Base frame '" << params_.base_frame
                                       << "' not found in model!");
    return CallbackReturn::ERROR;
  }

  // Setup target joint state subscription with FK publishing
  auto target_joint_callback =
      [this, ee_frame_id, base_frame_id](
          const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
    target_joint_buffer_.writeFromNonRT(msg);
    new_target_ = true;

    // Compute forward kinematics and publish target pose
    if (msg->position.size() >= static_cast<size_t>(model_.nv)) {
      // Create temporary configuration vector
      Eigen::VectorXd q_temp = Eigen::VectorXd::Zero(model_.nq);
      for (size_t i = 0; i < static_cast<size_t>(model_.nv); ++i) {
        q_temp[i] = msg->position[i];
      }

      // Compute forward kinematics
      pinocchio::Data data_temp(model_);
      pinocchio::forwardKinematics(model_, data_temp, q_temp);
      pinocchio::updateFramePlacements(model_, data_temp);

      // Get poses in world frame
      pinocchio::SE3 ee_pose_world = data_temp.oMf[ee_frame_id]; // World -> EE
      pinocchio::SE3 base_pose_world =
          data_temp.oMf[base_frame_id]; // World -> Base

      // Transform to base frame: Base -> EE = (World -> Base)^-1 * (World ->
      // EE)
      pinocchio::SE3 ee_pose_base = base_pose_world.inverse() * ee_pose_world;

      // Create and publish pose message
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = get_node()->now();
      pose_msg.header.frame_id = params_.base_frame;

      // Position
      pose_msg.pose.position.x = ee_pose_base.translation()[0];
      pose_msg.pose.position.y = ee_pose_base.translation()[1];
      pose_msg.pose.position.z = ee_pose_base.translation()[2];

      // Orientation (convert rotation matrix to quaternion)
      Eigen::Quaterniond quat(ee_pose_base.rotation());
      pose_msg.pose.orientation.x = quat.x();
      pose_msg.pose.orientation.y = quat.y();
      pose_msg.pose.orientation.z = quat.z();
      pose_msg.pose.orientation.w = quat.w();

      target_pose_pub_->publish(pose_msg);
    }
  };

  std::string joint_topic = node_name + "/target_joint";

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Subscribing to joint topic: " << joint_topic);

  joint_target_sub_ =
      get_node()->create_subscription<sensor_msgs::msg::JointState>(
          joint_topic, rclcpp::SensorDataQoS(), target_joint_callback);

  RCLCPP_INFO(get_node()->get_logger(),
              "MITJointController configuration completed successfully!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn MITJointController::on_activate(
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

  // Initialize targets to current state
  q_target_ = q_;
  dq_target_.setZero(); // Start with zero velocity target

  // Initialize goals to current state
  q_goal_ = q_;
  dq_goal_ = dq_;
  tau_ff_.setZero();

  RCLCPP_INFO(get_node()->get_logger(),
              "MITJointController activated at position: [%.3f, %.3f, %.3f, "
              "%.3f, %.3f, %.3f, %.3f]",
              q_[0], q_[1], q_[2], q_[3], q_[4], q_[5], q_[6]);

  // Initialize CSV logger if enabled
  if (params_.log.enabled) {
    csv_log_start_time_ = get_node()->now();

    std::string controller_name = get_node()->get_name();

    if (params_.log.use_async_logging) {
      csv_logger_ = std::make_unique<AsyncCSVLogger>(controller_name,
                                                     get_node()->get_logger());
      RCLCPP_INFO(get_node()->get_logger(),
                  "Initialized async CSV logger for controller: %s",
                  controller_name.c_str());
    } else {
      csv_logger_ = std::make_unique<ControllerCSVLogger>(
          controller_name, get_node()->get_logger());
      RCLCPP_INFO(get_node()->get_logger(),
                  "Initialized sync CSV logger for controller: %s",
                  controller_name.c_str());
    }

    // Initialize the logger with dummy data for header generation
    MITJointControllerLogData dummy_data;
    dummy_data.q = q_;
    dummy_data.dq = dq_;
    dummy_data.dq_filtered = dq_filtered_;
    dummy_data.q_target = q_target_;
    dummy_data.dq_target = dq_target_;
    dummy_data.q_goal = q_goal_;
    dummy_data.dq_goal = dq_goal_;
    dummy_data.q_error = q_error_;
    dummy_data.tau_ff = tau_ff_;
    dummy_data.mot_K_p = mot_K_p_;
    dummy_data.mot_K_d = mot_K_d_;
    dummy_data.joint_stiffness = K_joint_;
    dummy_data.joint_damping = D_joint_;
    dummy_data.alpha = params_.alpha;
    dummy_data.max_position_error = params_.max_position_error;
    dummy_data.max_velocity = params_.max_velocity;

    csv_logger_->initialize(csv_log_start_time_, dummy_data);
  }

  // Initialize timing statistics
  timing_stats_.reset_all();
  last_timing_log_ = std::chrono::steady_clock::now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MITJointController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Close CSV logger if active
  if (csv_logger_) {
    csv_logger_.reset();
    RCLCPP_INFO(get_node()->get_logger(), "CSV logger closed");
  }

  return CallbackReturn::SUCCESS;
}

void MITJointController::parse_target_joint_() {
  auto msg = *target_joint_buffer_.readFromRT();

  // Update target positions
  if (msg->position.size() == params_.joints.size()) {
    Eigen::VectorXd q_target_new(params_.joints.size());
    for (size_t i = 0; i < msg->position.size(); ++i) {
      q_target_new[i] = msg->position[i];
    }

    // Check if smoothing should be triggered
    if (params_.target_smoothing.enabled) {
      double max_jump = (q_target_new - q_target_).cwiseAbs().maxCoeff();

      if (max_jump > params_.target_smoothing.jump_threshold) {
        // Large jump detected - start smoothing
        smoothing_active_ = true;
        q_target_start_ = q_target_;
        q_target_end_ = q_target_new;
        smoothing_time_ = 0.0;
        // q_target_ will be interpolated in update loop
      } else {
        // Small change - apply directly
        q_target_ = q_target_new;
      }
    } else {
      // Smoothing disabled - apply directly
      q_target_ = q_target_new;
    }
  }

  // Update target velocities if provided
  if (msg->velocity.size() == params_.joints.size()) {
    for (size_t i = 0; i < msg->velocity.size(); ++i) {
      dq_target_[i] = msg->velocity[i];
    }
  } else {
    dq_target_.setZero();
  }

  new_target_ = false;
}

void MITJointController::compute_gravity_() {
  // Set position and velocity goals
  q_goal_ = q_; // Current position
  dq_goal_.setZero();

  // Update target to track current position so mode switches are smooth
  q_target_ = q_;
  dq_target_.setZero();

  tau_ff_.setZero();
  pinocchio::computeAllTerms(model_, data_, q_pin_, dq_filtered_);
  // data_.nle contains nonlinear effects (Coriolis + centrifugal + gravity)
  tau_ff_ = params_.gravity_scale * data_.nle;
}

void MITJointController::compute_impedance_posvel_() {
  // Send position goal to motors
  q_goal_ = q_target_;

  // Compute velocity goal from position error for smooth approach
  // This provides natural deceleration as the arm approaches the target
  dq_goal_ = params_.alpha * q_error_;

  // Clip velocity to max_velocity for safety
  double max_vel = params_.max_velocity;
  dq_goal_ = dq_goal_.cwiseMax(-max_vel).cwiseMin(max_vel);

  // If target velocity is provided, add it to the velocity goal
  if (dq_target_.norm() > 1e-6) {
    dq_goal_ += dq_target_;
    dq_goal_ = dq_goal_.cwiseMax(-max_vel).cwiseMin(max_vel);
  }

  // Full dynamics compensation
  tau_ff_.setZero();
  pinocchio::computeAllTerms(model_, data_, q_pin_, dq_filtered_);
  // data_.nle contains nonlinear effects (Coriolis + centrifugal + gravity)
  tau_ff_ = params_.gravity_scale * data_.nle;
}

bool MITJointController::detect_oscillation_(double dt) {
  // Need minimum samples to detect oscillation
  size_t window_size =
      std::min(static_cast<size_t>(params_.oscillation_detection.window),
               error_history_count_);
  if (window_size < 10) {
    return false; // Not enough data
  }

  // Check each joint for oscillation
  size_t num_joints = static_cast<size_t>(q_.size());
  for (size_t joint_idx = 0; joint_idx < num_joints && joint_idx < MAX_JOINTS;
       ++joint_idx) {
    int zero_crossings = 0;
    double mean = 0.0;
    double min_val = std::numeric_limits<double>::max();
    double max_val = std::numeric_limits<double>::lowest();

    // Calculate mean and min/max of the window
    for (size_t i = 0; i < window_size; ++i) {
      size_t idx = (error_history_idx_ + MAX_ERROR_HISTORY - window_size + i) %
                   MAX_ERROR_HISTORY;
      double val = joint_error_history_[joint_idx][idx];
      mean += val;
      min_val = std::min(min_val, val);
      max_val = std::max(max_val, val);
    }
    mean /= window_size;

    // Calculate peak-to-peak amplitude
    double amplitude = max_val - min_val;

    // Skip this joint if amplitude is below threshold (filtering noise)
    if (amplitude < params_.oscillation_detection.min_amplitude) {
      continue;
    }

    // Count zero crossings around the mean
    for (size_t i = 1; i < window_size; ++i) {
      size_t idx_prev =
          (error_history_idx_ + MAX_ERROR_HISTORY - window_size + i - 1) %
          MAX_ERROR_HISTORY;
      size_t idx_curr =
          (error_history_idx_ + MAX_ERROR_HISTORY - window_size + i) %
          MAX_ERROR_HISTORY;

      double val_prev = joint_error_history_[joint_idx][idx_prev] - mean;
      double val_curr = joint_error_history_[joint_idx][idx_curr] - mean;

      if ((val_prev < 0 && val_curr >= 0) || (val_prev >= 0 && val_curr < 0)) {
        zero_crossings++;
      }
    }

    // Calculate the frequency of oscillation using actual control period
    double window_duration_sec = window_size * dt;
    double oscillation_freq = (zero_crossings / 2.0) / window_duration_sec;

    // Oscillation detected if frequency exceeds threshold
    if (oscillation_freq >= params_.oscillation_detection.frequency) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Joint %zu oscillating at %.2f Hz (threshold: %.2f Hz) with "
          "amplitude %.4f rad (threshold: %.4f rad, zero crossings: %d)",
          joint_idx, oscillation_freq, params_.oscillation_detection.frequency,
          amplitude, params_.oscillation_detection.min_amplitude,
          zero_crossings);
      return true;
    }
  }

  return false;
}

void MITJointController::log_timing_statistics_() {
  RCLCPP_INFO(get_node()->get_logger(),
              "Control loop timing: "
              "total.avg=%ldus total.std=%ldus total.max=%ldus | "
              "state_read.avg=%ldus | velocity_filter.avg=%ldus | "
              "target_parse.avg=%ldus | control_compute.avg=%ldus control_compute.max=%ldus | "
              "oscillation.avg=%ldus | command_write.avg=%ldus | "
              "csv_log.avg=%ldus | cycles=%zu",
              timing_stats_.total_loop.avg_us(),
              timing_stats_.total_loop.std_us(),
              timing_stats_.total_loop.max_us,
              timing_stats_.state_read.avg_us(),
              timing_stats_.velocity_filter.avg_us(),
              timing_stats_.target_parse.avg_us(),
              timing_stats_.control_compute.avg_us(),
              timing_stats_.control_compute.max_us,
              timing_stats_.oscillation_detect.avg_us(),
              timing_stats_.command_write.avg_us(),
              timing_stats_.csv_logging.avg_us(),
              timing_stats_.total_loop.count);
}

} // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(crisp_controllers::MITJointController,
                       controller_interface::ControllerInterface)
