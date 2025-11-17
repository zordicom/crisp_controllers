#include <crisp_controllers/mit_joint_controller.hpp>
#include <crisp_controllers/pch.hpp>
#include <crisp_controllers/utils/async_csv_logger.hpp>
#include <crisp_controllers/utils/csv_logger.hpp>
#include <crisp_controllers/utils/mit_controller_log_data.hpp>
#include <limits>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hxx>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/logging.hpp>

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
  auto loop_start_time = get_node()->get_clock()->now();

  if (params_.stop_commands) {
    return controller_interface::return_type::OK;
  }

  size_t num_joints = params_.joints.size();

  // Read current state
  for (size_t i = 0; i < num_joints; i++) {
    q_[i] = state_interfaces_[i].get_value();
    q_pin_[i] = q_[i];
    dq_[i] = state_interfaces_[num_joints + i].get_value();
  }

  // Apply velocity filtering to reduce quantization noise
  dq_filtered_ = params_.dq_filter_alpha * dq_ +
                 (1.0 - params_.dq_filter_alpha) * dq_filtered_;

  // Parse target if new one available
  if (new_target_) {
    parse_target_joint_();
  }

  // Compute forward kinematics (needed for gravity/dynamics)
  pinocchio::forwardKinematics(model_, data_, q_pin_, dq_filtered_);
  pinocchio::updateFramePlacements(model_, data_);

  // Compute joint error
  q_error_ = q_target_ - q_;

  // Clip error to prevent excessive torques when far from target
  double max_error = params_.max_position_error;
  q_error_ = q_error_.cwiseMax(-max_error).cwiseMin(max_error);

  // Update error history for oscillation detection
  if (params_.oscillation_detection.enabled) {
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
  }

  // Stop sending commands if oscillation detected
  if (oscillation_detected_) {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "Controller stopped due to oscillation detection! stop_commands=true");
    return controller_interface::return_type::ERROR;
  }

  // Call the appropriate control mode function
  if (params_.control_mode == "gravity") {
    compute_gravity_();
  } else if (params_.control_mode == "gravity_coriolis") {
    compute_gravity_coriolis_();
  } else if (params_.control_mode == "gravity_velocity") {
    compute_gravity_velocity_();
  } else if (params_.control_mode == "gravity_impedance") {
    compute_gravity_impedance_();
  } else if (params_.control_mode == "impedance_posvel") {
    compute_impedance_posvel_();
  } else {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          1000, "Unknown control mode: %s",
                          params_.control_mode.c_str());
    return controller_interface::return_type::ERROR;
  }

  // Apply joint limits if enabled
  if (params_.limit_commands) {
    q_goal_ = q_goal_.cwiseMax(model_.lowerPositionLimit)
                  .cwiseMin(model_.upperPositionLimit);
  }

  // Apply torque safety limits
  Eigen::VectorXd tau_limits =
      params_.torque_safety_factor * model_.effortLimit;
  tau_ff_ = tau_ff_.cwiseMax(-tau_limits).cwiseMin(tau_limits);

  // Write commands to hardware
  for (size_t i = 0; i < num_joints; i++) {
    // Position command
    if (params_.use_position_command) {
      command_interfaces_[i].set_value(q_goal_[i]);
    } else {
      command_interfaces_[i].set_value(q_[i]);
    }

    // Velocity command
    if (params_.use_velocity_command) {
      command_interfaces_[num_joints + i].set_value(dq_goal_[i]);
    } else {
      command_interfaces_[num_joints + i].set_value(0.0);
    }

    // Feedforward torque
    command_interfaces_[2 * num_joints + i].set_value(tau_ff_[i]);

    // Motor gains
    command_interfaces_[3 * num_joints + i].set_value(mot_K_p_[i]);
    command_interfaces_[4 * num_joints + i].set_value(mot_K_d_[i]);
  }

  // CSV logging if enabled
  if (params_.log.enabled && csv_logger_) {
    MITControllerLogData log_data;
    log_data.timestamp =
        (time - csv_log_start_time_).nanoseconds() * 1e-9; // seconds

    // Set control mode
    log_data.control_mode = params_.control_mode;

    // Log joint states (vectors)
    log_data.q = q_;
    log_data.dq = dq_;
    log_data.dq_filtered = dq_filtered_;
    log_data.q_goal = q_goal_;
    log_data.dq_goal = dq_goal_;
    log_data.tau_ff = tau_ff_;
    log_data.mot_K_p = mot_K_p_;
    log_data.mot_K_d = mot_K_d_;

    // For joint controller, we don't have Cartesian data, so set dummy values
    log_data.current_pose = pinocchio::SE3::Identity();
    log_data.target_pose = pinocchio::SE3::Identity();
    log_data.error = Eigen::Vector<double, 6>::Zero();
    log_data.stiffness_diag = Eigen::Vector<double, 6>::Zero();
    log_data.damping_diag = Eigen::Vector<double, 6>::Zero();
    log_data.alpha = params_.alpha;

    auto loop_end_time = get_node()->get_clock()->now();
    log_data.loop_duration_ms =
        (loop_end_time - loop_start_time).nanoseconds() * 1e-6;

    csv_logger_->logData(log_data);
  }

  params_listener_->refresh_dynamic_parameters();
  params_ = params_listener_->get_params();

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
  auto robot_description =
      get_node()->get_parameter("robot_description").as_string();

  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "robot_description parameter is empty!");
    return CallbackReturn::ERROR;
  }

  pinocchio::urdf::buildModelFromXML(robot_description, model_);
  data_ = pinocchio::Data(model_);

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Loaded robot model with " << model_.njoints
                                                << " joints (nv=" << model_.nv
                                                << ", nq=" << model_.nq << ")");

  // Validate joint count
  size_t num_joints = params_.joints.size();
  if (num_joints != static_cast<size_t>(model_.nv)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Number of joints in params (%zu) does not match model nv "
                 "(%d)",
                 num_joints, model_.nv);
    return CallbackReturn::ERROR;
  }

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

  // Initialize target to zero (will be set to current position on activate)
  new_target_ = false;

  // Setup target joint state subscription
  auto target_joint_callback =
      [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) -> void {
    target_joint_buffer_.writeFromNonRT(msg);
    new_target_ = true;
  };

  std::string node_name = get_node()->get_name();
  std::string joint_topic = node_name + "/target_joint";

  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Subscribing to joint topic: " << joint_topic);

  joint_target_sub_ =
      get_node()->create_subscription<sensor_msgs::msg::JointState>(
          joint_topic, rclcpp::QoS(1), target_joint_callback);

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
  dq_target_ = dq_;

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
      csv_logger_ = std::make_unique<AsyncCSVLogger>(controller_name, get_node()->get_logger());
      RCLCPP_INFO(get_node()->get_logger(),
                  "Initialized async CSV logger for controller: %s", controller_name.c_str());
    } else {
      csv_logger_ = std::make_unique<ControllerCSVLogger>(controller_name, get_node()->get_logger());
      RCLCPP_INFO(get_node()->get_logger(),
                  "Initialized sync CSV logger for controller: %s", controller_name.c_str());
    }

    // Initialize the logger with dummy data for header generation
    MITControllerLogData dummy_data;
    dummy_data.q = q_;
    dummy_data.dq = dq_;
    dummy_data.dq_filtered = dq_filtered_;
    dummy_data.q_goal = q_goal_;
    dummy_data.dq_goal = dq_goal_;
    dummy_data.tau_ff = tau_ff_;
    dummy_data.mot_K_p = mot_K_p_;
    dummy_data.mot_K_d = mot_K_d_;
    dummy_data.current_pose = pinocchio::SE3::Identity();
    dummy_data.target_pose = pinocchio::SE3::Identity();
    dummy_data.error = Eigen::Vector<double, 6>::Zero();
    dummy_data.stiffness_diag = Eigen::Vector<double, 6>::Zero();
    dummy_data.damping_diag = Eigen::Vector<double, 6>::Zero();

    csv_logger_->initialize(csv_log_start_time_, dummy_data);
  }

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
    for (size_t i = 0; i < msg->position.size(); ++i) {
      q_target_[i] = msg->position[i];
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
  size_t num_joints = params_.joints.size();

  // Set position, velocity, kp, and kd to zero
  q_goal_ = q_; // Current position
  dq_goal_.setZero();
  mot_K_p_ = Eigen::VectorXd::Zero(num_joints);
  mot_K_d_ = Eigen::VectorXd::Zero(num_joints);

  // Only gravity compensation
  tau_ff_.setZero();
  if (params_.use_gravity_compensation) {
    Eigen::VectorXd tau_gravity =
        params_.gravity_scale *
        pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);
    tau_ff_ = tau_gravity;
  }
}

void MITJointController::compute_gravity_coriolis_() {
  size_t num_joints = params_.joints.size();

  // Set position, velocity, kp, and kd to zero
  q_goal_ = q_; // Current position
  dq_goal_.setZero();
  mot_K_p_ = Eigen::VectorXd::Zero(num_joints);
  mot_K_d_ = Eigen::VectorXd::Zero(num_joints);

  // Gravity + Coriolis compensation
  tau_ff_.setZero();

  if (params_.use_gravity_compensation) {
    // Gravity compensation
    Eigen::VectorXd tau_gravity =
        params_.gravity_scale *
        pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);

    // Coriolis compensation
    pinocchio::computeAllTerms(model_, data_, q_pin_, dq_filtered_);
    Eigen::VectorXd tau_coriolis =
        pinocchio::computeCoriolisMatrix(model_, data_, q_pin_, dq_filtered_) *
        dq_filtered_;

    tau_ff_ = tau_gravity + tau_coriolis;
  }
}

void MITJointController::compute_gravity_velocity_() {
  size_t num_joints = params_.joints.size();

  // Set position and velocity goals
  q_goal_ = q_; // Current position
  dq_goal_.setZero();

  // Use motor gains for velocity damping
  mot_K_p_ = Eigen::VectorXd::Zero(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    mot_K_d_[i] = params_.motor_kd[i];
  }

  // Gravity compensation
  tau_ff_.setZero();
  if (params_.use_gravity_compensation) {
    Eigen::VectorXd tau_gravity =
        params_.gravity_scale *
        pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);
    tau_ff_ = tau_gravity;
  }
}

void MITJointController::compute_gravity_impedance_() {
  size_t num_joints = params_.joints.size();

  // Set motor gains to zero (we'll control via feedforward torques)
  q_goal_ = q_; // Current position
  dq_goal_.setZero();
  mot_K_p_ = Eigen::VectorXd::Zero(num_joints);
  mot_K_d_ = Eigen::VectorXd::Zero(num_joints);

  // Compute impedance torques: tau = K*(q_target - q) - D*dq
  Eigen::VectorXd tau_impedance =
      K_joint_.cwiseProduct(q_error_) - D_joint_.cwiseProduct(dq_filtered_);

  // Gravity compensation
  tau_ff_.setZero();
  if (params_.use_gravity_compensation) {
    Eigen::VectorXd tau_gravity =
        params_.gravity_scale *
        pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);
    tau_ff_ = tau_gravity;
  }

  // Add impedance torques
  tau_ff_ += tau_impedance;
}

void MITJointController::compute_impedance_posvel_() {
  size_t num_joints = params_.joints.size();

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

  // Use motor gains from parameters
  for (size_t i = 0; i < num_joints; ++i) {
    mot_K_p_[i] = params_.motor_kp[i];
    mot_K_d_[i] = params_.motor_kd[i];
  }

  // Full dynamics compensation
  tau_ff_.setZero();
  if (params_.use_gravity_compensation) {
    // Gravity compensation
    Eigen::VectorXd tau_gravity =
        params_.gravity_scale *
        pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);

    // Coriolis compensation
    pinocchio::computeAllTerms(model_, data_, q_pin_, dq_filtered_);
    Eigen::VectorXd tau_coriolis =
        pinocchio::computeCoriolisMatrix(model_, data_, q_pin_, dq_filtered_) *
        dq_filtered_;

    tau_ff_ = tau_gravity + tau_coriolis;
  }
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

} // namespace crisp_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(crisp_controllers::MITJointController,
                       controller_interface::ControllerInterface)
